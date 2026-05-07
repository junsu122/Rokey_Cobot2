"""
webcam_teleop_node.py
─────────────────────
책임:
  1. 웹캠 영상에서 MediaPipe 손 추적 (Index-tip / Palm-center)
  2. WAITING → CALIBRATING → CONTROLLING ↔ PAUSED 상태 머신 운영
  3. GestureEvent 토픽 퍼블리시 (~30 Hz)  ← 21개 랜드마크 포함
  4. orchestrator_node 로부터 teleop/enable 구독
     - True + CONTROLLING 상태이면 move_line 서비스 호출
  5. robot_state 구독 → 현재 TCP 추적 (캘리브레이션 기준점)
  6. /is_picking + /selected_object 구독 → PICKING 오버레이 표시

퍼블리시:  /gesture_event   (gesture_robot_interfaces/msg/GestureEvent)
구독:      /teleop/enable   (std_msgs/Bool)
           /is_picking      (std_msgs/Bool)
           /selected_object (gesture_robot_interfaces/msg/SelectedObject)
           /{ROBOT_ID}/state (dsr_msgs2/RobotState)
서비스:    /{ROBOT_ID}/motion/move_line
"""

import time
import threading
import os
from collections import deque

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python.vision import (
    HandLandmarker, HandLandmarkerOptions, RunningMode
)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from dsr_msgs2.srv import MoveLine
from dsr_msgs2.msg import RobotState

from gesture_robot_interfaces.msg import GestureEvent, SelectedObject
from gesture_robot_pkg.constants import (
    ROBOT_ID, WEBCAM_INDEX, SMOOTHING_FRAMES, CALIB_SEC,
    DEAD_ZONE, POSITION_CONTROL, REPEAT_HZ,
    ABS_VEL, ABS_ACC, ABS_BLEND_R, ABS_MAX_STEP, POS_THR_MM,
    MIN_STEP_MM, SMOOTH_ALPHA, CMD_CHANGE_THR,
    STATE_WAITING, STATE_CALIBRATING, STATE_CONTROLLING, STATE_PAUSED,
    COLORS,
)
from gesture_robot_pkg.utils import (
    is_fist, is_index_pointing, is_in_center,
    compute_delta, compute_target,
    draw_landmarks_manual, draw_workspace_grid, draw_picking_overlay,
)


# ── MediaPipe 초기화 헬퍼 ──────────────────────────────────────────────

_MP_VER = tuple(int(x) for x in mp.__version__.split('.')[:2])
USE_NEW_API = _MP_VER >= (0, 10)


def _init_mediapipe_new():
    import urllib.request, tempfile
    MODEL_URL  = ('https://storage.googleapis.com/mediapipe-models/'
                  'hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task')
    MODEL_PATH = os.path.join(tempfile.gettempdir(), 'hand_landmarker.task')
    if not os.path.exists(MODEL_PATH):
        print('[webcam_teleop] Downloading MediaPipe model...')
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)

    for delegate in (mp_python.BaseOptions.Delegate.GPU,
                     mp_python.BaseOptions.Delegate.CPU):
        try:
            opts = HandLandmarkerOptions(
                base_options=mp_python.BaseOptions(
                    model_asset_path=MODEL_PATH,
                    delegate=delegate,
                ),
                running_mode=RunningMode.IMAGE,
                num_hands=1,
                min_hand_detection_confidence=0.6,
                min_tracking_confidence=0.5,
            )
            return HandLandmarker.create_from_options(opts)
        except Exception as e:
            print(f'[webcam_teleop] MediaPipe delegate {delegate} failed: {e}')
    raise RuntimeError('[webcam_teleop] MediaPipe HandLandmarker init failed on all delegates')


def _init_mediapipe_legacy():
    mod   = mp.solutions.hands
    hands = mod.Hands(max_num_hands=1,
                      min_detection_confidence=0.6,
                      min_tracking_confidence=0.5)
    return mod, hands


# ══════════════════════════════════════════════════════
#  Node
# ══════════════════════════════════════════════════════

class WebcamTeleopNode(Node):

    def __init__(self):
        super().__init__('webcam_teleop_node')

        # ── 파라미터 ──────────────────────────────────
        self.declare_parameter('webcam_index', WEBCAM_INDEX)
        self.declare_parameter('position_control', POSITION_CONTROL)
        self._cam_idx  = self.get_parameter('webcam_index').value
        self._pos_ctrl = self.get_parameter('position_control').value

        # ── 퍼블리셔 ──────────────────────────────────
        self._pub_gesture = self.create_publisher(GestureEvent, '/gesture_event', 10)

        # ── 구독 ─────────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            Bool, '/teleop/enable', self._teleop_enable_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Bool, '/is_picking', self._is_picking_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            SelectedObject, '/selected_object', self._selected_obj_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            RobotState, f'/{ROBOT_ID}/state', self._robot_state_cb, 10,
            callback_group=self._cb_group)

        # ── move_line 서비스 클라이언트 ──────────────
        svc = f'/{ROBOT_ID}/motion/move_line'
        self._move_cli = self.create_client(MoveLine, svc,
                                            callback_group=self._cb_group)
        self.get_logger().info(f'Waiting for service: {svc}')
        # if not self._move_cli.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().warn('move_line service not found – teleop disabled')

        # ── 내부 상태 ──────────────────────────────────
        self._teleop_enabled = False
        self._current_pos    = [0.0] * 6
        self._pos_received   = False
        self._pending_count  = 0
        self._pending_lock   = threading.Lock()

        # 픽앤플레이스 상태 (orchestrator 에서 수신)
        self._is_picking_ext = False
        self._picking_label  = ''

        # 제스처 상태 머신
        self._state          = STATE_WAITING
        self._calib_data: list[float] = []
        self._calib_start    = 0.0
        self._calib_progress = 0.0
        self._base_dist      = 0.0
        self._calib_tcp      = [0.0] * 6

        # 손 위치 스무딩
        self._x_hist = deque(maxlen=SMOOTHING_FRAMES)
        self._y_hist = deque(maxlen=SMOOTHING_FRAMES)
        self._d_hist = deque(maxlen=SMOOTHING_FRAMES)

        # 위치 제어용 추적
        self._target_pos  = None
        self._prev_target = [None] * 6

        # 속도 제어용
        self._delta        = [0.0, 0.0, 0.0]
        self._smooth_delta = [0.0, 0.0, 0.0]
        self._prev_sent    = [0.0, 0.0, 0.0]

        # ── MediaPipe 초기화 ─────────────────────────
        if USE_NEW_API:
            self._detector = _init_mediapipe_new()
        else:
            self._mp_mod, self._mp_hands = _init_mediapipe_legacy()
            self._mp_draw   = mp.solutions.drawing_utils
            self._mp_styles = mp.solutions.drawing_styles

        # ── 타이머: 로봇 명령 반복 전송 (REPEAT_HZ) ─
        self._cmd_timer = self.create_timer(1.0 / REPEAT_HZ, self._cmd_repeat_cb,
                                            callback_group=self._cb_group)

        self.get_logger().info(
            f'WebcamTeleopNode ready  cam={self._cam_idx}  '
            f'mode={"POSITION" if self._pos_ctrl else "VELOCITY"}'
        )

    # ── 구독 콜백 ─────────────────────────────────────

    def _teleop_enable_cb(self, msg: Bool):
        self._teleop_enabled = msg.data
        if not msg.data:
            self._target_pos = None
            self._delta      = [0.0, 0.0, 0.0]
        # self.get_logger().info(f'teleop/enable → {msg.data}')

    def _is_picking_cb(self, msg: Bool):
        self._is_picking_ext = msg.data
        if not msg.data:
            self._picking_label = ''

    def _selected_obj_cb(self, msg: SelectedObject):
        self._picking_label = msg.label

    def _robot_state_cb(self, msg: RobotState):
        if len(msg.current_posx) >= 6:
            self._current_pos  = list(msg.current_posx)
            self._pos_received = True

    # ── 로봇 명령 타이머 ──────────────────────────────

    def _cmd_repeat_cb(self):
        """CONTROLLING 상태이고 픽앤플레이스 중이 아닐 때 주기적으로 move_line 호출."""
        if self._state != STATE_CONTROLLING or self._is_picking_ext:
            return

        if self._pos_ctrl:
            self._send_position_cmd()
        else:
            self._send_velocity_cmd()

    def _send_position_cmd(self):
        if self._target_pos is None or self._prev_target[0] is None:
            return
        with self._pending_lock:
            if self._pending_count >= 2:   ##5.7 수정부  4->2
                return
        delta_xyz = [self._target_pos[i] - self._prev_target[i] for i in range(3)]
        dist = sum(d ** 2 for d in delta_xyz) ** 0.5
        if dist < POS_THR_MM:
            return
        if dist > ABS_MAX_STEP:
            scale     = ABS_MAX_STEP / dist
            delta_xyz = [d * scale for d in delta_xyz]
            self._prev_target = [self._prev_target[i] + delta_xyz[i]
                                  for i in range(3)] + list(self._prev_target[3:])
        else:
            self._prev_target = list(self._target_pos[:3]) + list(self._prev_target[3:])
        with self._pending_lock:
            self._pending_count += 1
        self._send_rel_move(delta_xyz)

    def _send_velocity_cmd(self):
        for i in range(3):
            self._smooth_delta[i] = (SMOOTH_ALPHA * self._delta[i]
                                     + (1.0 - SMOOTH_ALPHA) * self._smooth_delta[i])
        sd      = [v if abs(v) >= MIN_STEP_MM else 0.0 for v in self._smooth_delta]
        changed = any(abs(sd[i] - self._prev_sent[i]) >= CMD_CHANGE_THR for i in range(3))
        if any(v != 0.0 for v in sd) and changed:
            self._prev_sent = list(sd)
            self._send_rel_move(sd)
        elif all(v == 0.0 for v in sd) and any(v != 0.0 for v in self._prev_sent):
            self._prev_sent = [0.0, 0.0, 0.0]

    def _send_rel_move(self, delta: list):
        if not self._move_cli.service_is_ready():
            return
        req            = MoveLine.Request()
        req.pos        = [delta[0], delta[1], delta[2], 0.0, 0.0, 0.0]
        req.vel        = [ABS_VEL,  50.0]
        req.acc        = [ABS_ACC, 100.0]
        req.time       = 0.0
        req.radius     = max(ABS_BLEND_R, sum(abs(d) for d in delta) * 0.5) # 스텝 크기에 비례하도록 변경. 원본: ABS_BLEND_R
        req.ref        = 0
        req.mode       = 1
        req.blend_type = 1
        req.sync_type  = 0
        future = self._move_cli.call_async(req)
        future.add_done_callback(self._move_done_cb)

    def _move_done_cb(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(f'MoveLine failed: {res.error_code}')
                with self._pending_lock:
                    self._pending_count = 0
            else:
                with self._pending_lock:
                    self._pending_count = max(0, self._pending_count - 1)
        except Exception as e:
            self.get_logger().error(f'Service error: {e}')
            with self._pending_lock:
                self._pending_count = 0

    # ── 메인 캡처 루프 (별도 스레드에서 호출) ─────────

    def run_capture_loop(self):
        cap = cv2.VideoCapture(self._cam_idx)
        if not cap.isOpened():
            self.get_logger().fatal(f'Cannot open webcam {self._cam_idx}')
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.get_logger().info(
            f'Webcam opened: {int(cap.get(3))} x {int(cap.get(4))}')

        cv2.namedWindow('Gesture Teleop', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Gesture Teleop', 1280, 720)

        try:
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    break
                frame = cv2.flip(frame, 1)

                (avg_x, avg_y, curr_dist, index_x, index_y,
                 hand_visible, current_lms,
                 fist, pointing) = self._process_hand(frame)

                if not self._is_picking_ext:
                    self._update_state_machine(avg_x, avg_y, curr_dist,
                                               hand_visible, current_lms,
                                               fist, pointing)

                self._publish_gesture_event(avg_x, avg_y, curr_dist,
                                            index_x, index_y,
                                            hand_visible, current_lms,
                                            fist, pointing)

                self._draw_ui(frame, avg_x, avg_y, hand_visible)
                cv2.imshow('Gesture Teleop', frame)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), ord('Q'), 27):
                    break
                elif key in (ord('r'), ord('R')) and not self._is_picking_ext:
                    self._reset_calibration()
                elif key == ord(' ') and not self._is_picking_ext:
                    self._toggle_pause()
        finally:
            cap.release()
            cv2.destroyAllWindows()
            if USE_NEW_API:
                self._detector.close()
            else:
                self._mp_hands.close()

    # ── 손 추적 ───────────────────────────────────────

    def _process_hand(self, frame):
        """MediaPipe 실행 → (avg_x, avg_y, curr_dist, ix, iy, visible, lms, fist, pointing)"""
        avg_x = avg_y = curr_dist = index_x = index_y = None
        hand_visible = False
        current_lms  = None
        fist         = False
        pointing     = False

        if USE_NEW_API:
            mp_img = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            result = self._detector.detect(mp_img)
            if result.hand_landmarks:
                hand_visible = True
                lms   = result.hand_landmarks[0]
                tip   = lms[8]; wrist = lms[0]
                self._x_hist.append(tip.x)
                self._y_hist.append(tip.y)
                self._d_hist.append(
                    ((tip.x - wrist.x) ** 2 + (tip.y - wrist.y) ** 2) ** 0.5)
                avg_x     = float(np.mean(self._x_hist))
                avg_y     = float(np.mean(self._y_hist))
                curr_dist = float(np.mean(self._d_hist))
                index_x, index_y = tip.x, tip.y
                current_lms = lms
                fist     = is_fist(lms)
                pointing = is_index_pointing(lms)
                color = (0, 0, 220) if fist else (0, 220, 255) if pointing else (0, 200, 100)
                draw_landmarks_manual(frame, lms, color=color)
            else:
                self._x_hist.clear(); self._y_hist.clear(); self._d_hist.clear()
        else:
            rgb    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self._mp_hands.process(rgb)
            if result.multi_hand_landmarks:
                hand_visible = True
                hand  = result.multi_hand_landmarks[0]
                tip   = hand.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]
                wrist = hand.landmark[mp.solutions.hands.HandLandmark.WRIST]
                self._x_hist.append(tip.x)
                self._y_hist.append(tip.y)
                self._d_hist.append(
                    ((tip.x - wrist.x) ** 2 + (tip.y - wrist.y) ** 2) ** 0.5)
                avg_x     = float(np.mean(self._x_hist))
                avg_y     = float(np.mean(self._y_hist))
                curr_dist = float(np.mean(self._d_hist))
                index_x, index_y = tip.x, tip.y
                current_lms = hand.landmark
                fist     = is_fist(current_lms)
                pointing = is_index_pointing(current_lms)
                self._mp_draw.draw_landmarks(
                    frame, hand,
                    self._mp_mod.HAND_CONNECTIONS,
                    self._mp_styles.get_default_hand_landmarks_style(),
                    self._mp_styles.get_default_hand_connections_style())
            else:
                self._x_hist.clear(); self._y_hist.clear(); self._d_hist.clear()

        return avg_x, avg_y, curr_dist, index_x, index_y, hand_visible, current_lms, fist, pointing

    # ── 상태 머신 ─────────────────────────────────────

    def _update_state_machine(self, avg_x, avg_y, curr_dist,
                               hand_visible, current_lms, fist, pointing):
        if not hand_visible or avg_x is None:
            return

        in_center = is_in_center(avg_x, avg_y)

        if self._state == STATE_WAITING:
            if in_center:
                self._state       = STATE_CALIBRATING
                self._calib_data  = []
                self._calib_start = time.time()
                self.get_logger().info('[CALIB START]')

        elif self._state == STATE_CALIBRATING:
            if in_center:
                elapsed               = time.time() - self._calib_start
                self._calib_progress  = min(elapsed / CALIB_SEC, 1.0)
                self._calib_data.append(curr_dist)
                if elapsed >= CALIB_SEC:
                    self._base_dist = float(np.mean(self._calib_data))
                    if self._pos_ctrl and self._pos_received:
                        self._calib_tcp = list(self._current_pos)
                    else:
                        self._calib_tcp = [0.0] * 6
                    self._prev_target = list(self._calib_tcp)
                    self._target_pos  = list(self._calib_tcp)
                    self._state       = STATE_CONTROLLING
                    self.get_logger().info(
                        f'[CALIB DONE] base={self._base_dist:.4f}  '
                        f'TCP={[f"{v:.1f}" for v in self._calib_tcp[:3]]}'
                    )
            else:
                self._state = STATE_WAITING
                self.get_logger().info('[CALIB CANCELLED]')

        elif self._state == STATE_CONTROLLING:
            if fist:
                self._state      = STATE_PAUSED
                self._target_pos = None
                self._delta      = [0.0, 0.0, 0.0]
                self.get_logger().info('[PAUSED] fist detected')
            elif pointing:
                # 포인팅 중: 위치 제어 목표는 마지막 값 유지(로봇이 목표까지 계속 이동),
                # 속도 제어는 v6 방식으로 delta=0 → 정지
                self._delta = [0.0, 0.0, 0.0]
            else:
                # 일반 손 제어: 목표 또는 delta 계산
                if self._pos_ctrl:
                    self._target_pos = compute_target(
                        avg_x, avg_y, curr_dist,
                        self._base_dist, self._calib_tcp)
                else:
                    dx, dy, dz  = compute_delta(avg_x, avg_y, curr_dist, self._base_dist)
                    self._delta = [dx, dy, dz]
                    if dx == 0.0 and dy == 0.0 and dz == 0.0:
                        self._smooth_delta = [0.0, 0.0, 0.0]

        elif self._state == STATE_PAUSED:
            if not fist:
                self._state = STATE_CONTROLLING
                self.get_logger().info('[RESUMED]')

    # ── GestureEvent 퍼블리시 ─────────────────────────

    def _publish_gesture_event(self, avg_x, avg_y, curr_dist,
                                index_x, index_y,
                                hand_visible, current_lms,
                                fist, pointing):
        msg = GestureEvent()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.gesture_state   = self._state
        msg.hand_visible    = hand_visible
        msg.is_fist         = fist
        msg.is_pointing     = pointing
        msg.avg_x           = float(avg_x)      if avg_x      is not None else 0.0
        msg.avg_y           = float(avg_y)      if avg_y      is not None else 0.0
        msg.index_tip_x     = float(index_x)   if index_x    is not None else 0.0
        msg.index_tip_y     = float(index_y)   if index_y    is not None else 0.0
        msg.curr_dist       = float(curr_dist)  if curr_dist  is not None else 0.0
        msg.base_dist       = self._base_dist
        msg.calib_progress  = self._calib_progress
        msg.calib_tcp       = [float(v) for v in self._calib_tcp]
        msg.target_pos_mm   = ([float(v) for v in self._target_pos]
                                if self._target_pos else [0.0] * 6)
        msg.velocity_delta  = [float(v) for v in self._delta]

        # 21개 손 랜드마크 (vision_node 에서 RealSense 창에 스켈레톤 표시용)
        if current_lms is not None and len(current_lms) >= 21:
            msg.landmarks_x = [float(current_lms[i].x) for i in range(21)]
            msg.landmarks_y = [float(current_lms[i].y) for i in range(21)]
        else:
            msg.landmarks_x = [0.0] * 21
            msg.landmarks_y = [0.0] * 21

        self._pub_gesture.publish(msg)

    # ── UI 드로잉 (v6 동등 수준) ──────────────────────

    def _draw_ui(self, frame, avg_x, avg_y, hand_visible):
        h, w = frame.shape[:2]
        s_color  = COLORS.get(self._state, COLORS['NONE'])
        mode_tag = '[POS]' if self._pos_ctrl else '[VEL]'
        mid_x, mid_y = w // 2, h // 2

        # ── PICKING 오버레이 ────────────────────────
        if self._is_picking_ext:
            draw_picking_overlay(frame, self._picking_label)

        # ── 상태 바 ────────────────────────────────
        roi = frame[:100, :]
        ov  = roi.copy()
        ov[:] = (15, 15, 15)
        cv2.addWeighted(ov, 0.65, roi, 0.35, 0, roi)

        state_labels = {
            STATE_WAITING:     f'{mode_tag} WAITING  →  Move hand into CENTER box',
            STATE_CALIBRATING: f'{mode_tag} CALIBRATING...  {self._calib_progress * 100:.0f}%',
            STATE_CONTROLLING: f'{mode_tag} CONTROLLING  (X / Y / Z active)',
            STATE_PAUSED:      'PAUSED  (open hand / SPACE to resume)',
        }
        if self._is_picking_ext:
            label = f'PICKING  ({self._picking_label})'
        elif hand_visible or self._state in (STATE_WAITING, STATE_CALIBRATING):
            label = state_labels.get(self._state, self._state)
        else:
            label = 'NO HAND'

        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, _), _ = cv2.getTextSize(label, font, 0.85, 2)
        cv2.putText(frame, label, ((w - tw) // 2, 45), font, 0.85, s_color, 2, cv2.LINE_AA)

        # ── PAUSED 대형 오버레이 ────────────────────
        if self._state == STATE_PAUSED and not self._is_picking_ext:
            pause_ov = np.full_like(frame, (150, 30, 0))
            cv2.addWeighted(pause_ov, 0.10, frame, 0.90, 0, frame)
            ptxt = 'PAUSED'
            (ptw, pth), _ = cv2.getTextSize(ptxt, cv2.FONT_HERSHEY_DUPLEX, 2.5, 4)
            cv2.putText(frame, ptxt, ((w - ptw) // 2, (h + pth) // 2),
                        cv2.FONT_HERSHEY_DUPLEX, 2.5, (0, 60, 220), 4, cv2.LINE_AA)

        # ── TCP / 목표 좌표 표시 ────────────────────
        if self._pos_received:
            p = self._current_pos
            cv2.putText(frame,
                        f'TCP  X:{p[0]:7.1f}  Y:{p[1]:7.1f}  Z:{p[2]:7.1f} mm',
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 220, 255), 1)
        else:
            cv2.putText(frame, 'TCP: Waiting... (no state topic)',
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 180), 1)

        if self._state == STATE_CONTROLLING:
            if self._pos_ctrl and self._target_pos is not None:
                t = self._target_pos
                cv2.putText(frame,
                            f'tgt  X:{t[0]:7.1f}  Y:{t[1]:7.1f}  Z:{t[2]:7.1f} mm',
                            (10, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 255, 160), 1)
            elif not self._pos_ctrl:
                sd     = self._smooth_delta
                active = any(abs(v) >= MIN_STEP_MM for v in sd)
                d_col  = (80, 255, 160) if active else (70, 70, 70)
                cv2.putText(frame,
                            f'cmd  X:{sd[0]:+5.1f}  Y:{sd[1]:+5.1f}  Z:{sd[2]:+5.1f}  mm/step',
                            (10, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.45, d_col, 1)

        # ── 십자선 ─────────────────────────────────
        cv2.line(frame, (mid_x, 100), (mid_x, h), (160, 160, 160), 1, cv2.LINE_AA)
        cv2.line(frame, (0, mid_y),   (w, mid_y), (160, 160, 160), 1, cv2.LINE_AA)

        # ── Dead-zone 박스 ──────────────────────────
        lz = int((0.5 - DEAD_ZONE) * w); rz = int((0.5 + DEAD_ZONE) * w)
        tz = int((0.5 - DEAD_ZONE) * h); bz = int((0.5 + DEAD_ZONE) * h)
        dz_roi = frame[tz:bz, lz:rz]
        dz_ov  = dz_roi.copy()
        dz_ov[:] = (50, 50, 50)
        cv2.addWeighted(dz_ov, 0.25, dz_roi, 0.75, 0, dz_roi)
        border_col = s_color if self._state == STATE_CALIBRATING else (110, 110, 110)
        cv2.rectangle(frame, (lz, tz), (rz, bz), border_col, 2)
        cv2.putText(frame, 'CENTER / CALIB', (lz + 6, mid_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 160, 160), 1)

        if self._state == STATE_CALIBRATING and self._calib_progress > 0:
            bar_filled = int((rz - lz) * self._calib_progress)
            cv2.rectangle(frame, (lz, bz + 5), (lz + bar_filled, bz + 14),
                          COLORS[STATE_CALIBRATING], -1)
            cv2.rectangle(frame, (lz, bz + 5), (rz, bz + 14), (100, 100, 100), 1)

        # ── 손 커서 + 중심-손 연결선 ────────────────
        if hand_visible and avg_x is not None:
            cx = int(avg_x * w); cy = int(avg_y * h)
            cv2.line(frame, (mid_x, mid_y), (cx, mid_y), s_color, 2, cv2.LINE_AA)
            cv2.line(frame, (mid_x, mid_y), (mid_x, cy), s_color, 2, cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 14, s_color,       -1, cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 14, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, 'INDEX', (cx + 16, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, s_color, 1, cv2.LINE_AA)

        # ── 축 레이블 ───────────────────────────────
        cv2.putText(frame, '< X+',  (14, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 100, 100), 1)
        cv2.putText(frame, 'X- >',  (w - 75, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 100, 100), 1)
        cv2.putText(frame, '^ Z+',  (mid_x - 25, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 100, 100), 1)
        cv2.putText(frame, 'v Z-',  (mid_x - 25, h - 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 100, 100), 1)

        # ── 워크스페이스 그리드 (위치 제어 시) ────────
        if self._pos_ctrl and self._state == STATE_CONTROLLING:
            draw_workspace_grid(frame, avg_x, avg_y,
                                self._calib_tcp, self._target_pos, self._state)

        cv2.putText(frame, 'R:recalib  SPACE:pause  C:clear  Q:quit',
                    (w - 330, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (90, 90, 90), 1)

    # ── 키보드 헬퍼 ───────────────────────────────────

    def _reset_calibration(self):
        self._state          = STATE_WAITING
        self._base_dist      = 0.0
        self._calib_progress = 0.0
        self._calib_tcp      = [0.0] * 6
        self._target_pos     = None
        self._prev_target    = [None] * 6
        self._delta          = [0.0, 0.0, 0.0]
        self.get_logger().info('[RECALIB] Move hand to CENTER box.')

    def _toggle_pause(self):
        if self._state == STATE_CONTROLLING:
            self._state      = STATE_PAUSED
            self._target_pos = None
            self._delta      = [0.0, 0.0, 0.0]
        elif self._state == STATE_PAUSED:
            self._state = STATE_CONTROLLING


# ══════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = WebcamTeleopNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_capture_loop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
