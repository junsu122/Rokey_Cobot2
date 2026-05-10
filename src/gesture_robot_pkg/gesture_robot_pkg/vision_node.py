"""
vision_node.py
──────────────
책임:
  1. RealSense 컬러 이미지 구독
  2. YOLOv8 물체 검출 (bbox, class, confidence) — 제스처 호버 선택용
  3. GestureEvent 구독 → 검지 끝 좌표로 호버 판정 + 손 스켈레톤 표시
  4. HOVER_SEC 동안 검지가 bbox 위에 머물면 SelectedObject 퍼블리시
  5. /is_picking 구독 → PICKING 오버레이 표시
  6. 어노테이션된 프레임을 cv2 창에 표시
  7. /voice_intent 구독 → 탐색 대상 설정 / 미발견 시 /object_not_found 발행
  8. from_scan=true 수신 시 best_bbox 로 즉시 SelectedObject 발행

구독:
  /camera/camera/color/image_raw  (sensor_msgs/Image)
  /gesture_event                  (GestureEvent)
  /is_picking                     (std_msgs/Bool)
  /voice_intent                   (std_msgs/String)  ← publisher.py 연동

퍼블리시:
  /selected_object                (SelectedObject)
  /object_not_found               (std_msgs/String)  ← publisher.py 연동
"""

import threading
import time
import json
from datetime import datetime

import cv2
import torch
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from gesture_robot_interfaces.msg import GestureEvent, SelectedObject
from gesture_robot_pkg.constants import (
    REALSENSE_COLOR_TOPIC,
    YOLO_MODEL_PATH, YOLO_CONF_THRESHOLD,
    HOVER_SEC, HAND_CONNECTIONS,
    COLORS,
)
from gesture_robot_pkg.utils import (
    draw_hover_progress, draw_selected_label, draw_picking_overlay,
)

# publisher.py 가 발행하는 bring_* 액션 목록
# "bring_object" 를 포함한 모든 bring_* 액션을 허용하기 위해
# 고정 튜플 대신 prefix 비교 함수로 판정한다.
def _is_bring_action(action: str) -> bool:
    return isinstance(action, str) and action.startswith("bring_")


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter('yolo_model',              YOLO_MODEL_PATH)
        self.declare_parameter('yolo_conf',               YOLO_CONF_THRESHOLD)
        self.declare_parameter('hover_sec',               HOVER_SEC)
        self.declare_parameter('voice_not_found_timeout', 3.0)  # 미발견 판정 대기 시간(초)

        self._yolo_path              = self.get_parameter('yolo_model').value
        self._yolo_conf              = self.get_parameter('yolo_conf').value
        self._hover_sec              = self.get_parameter('hover_sec').value
        self._voice_not_found_timeout = float(
            self.get_parameter('voice_not_found_timeout').value)

        # ── 퍼블리셔 ──────────────────────────────────────────────────────
        self._pub_obj       = self.create_publisher(SelectedObject, '/selected_object', 10)
        self._pub_not_found = self.create_publisher(String, '/object_not_found', 10)

        # ── 구독 ──────────────────────────────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Image, REALSENSE_COLOR_TOPIC, self._color_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            GestureEvent, '/gesture_event', self._gesture_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Bool, '/is_picking', self._is_picking_cb, 10,
            callback_group=self._cb_group)
        # publisher.py → /voice_intent 구독 (신규)
        self.create_subscription(
            String, '/voice_intent', self._voice_intent_cb, 10,
            callback_group=self._cb_group)

        # ── YOLO ──────────────────────────────────────────────────────────
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'YOLO device={device}  model={self._yolo_path}')
        try:
            self._yolo = YOLO(self._yolo_path).to(device)
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            self._yolo = None

        # YOLO 는 매 N 프레임마다 실행 (입력 ~30 Hz → 검출 ~10 Hz)
        self._yolo_skip  = 0
        self._yolo_every = 3

        # ── CvBridge ──────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── 내부 상태 (스레드 안전) ───────────────────────────────────────
        self._latest_frame      = None
        self._frame_lock        = threading.Lock()

        self._detections: list  = []
        self._detect_lock       = threading.Lock()

        # 최신 GestureEvent
        self._gesture_state = ''
        self._is_pointing   = False
        self._is_fist       = False
        self._index_tip_x   = 0.0
        self._index_tip_y   = 0.0
        self._hand_visible  = False
        self._landmarks_x   = [0.0] * 21
        self._landmarks_y   = [0.0] * 21
        self._gesture_lock  = threading.Lock()

        # 픽앤플레이스 진행 중 여부 + 호버 추적
        self._is_picking_ext              = False
        self._hover_target: str | None    = None
        self._hover_start: float | None   = None
        self._selected_label: str | None  = None
        self._selected_box: list | None   = None
        self._pick_state_lock             = threading.Lock()

        # ── voice_intent 탐색 대기 상태 ───────────────────────────────────
        # publisher.py 가 /voice_intent 를 발행하면 아래 상태로 진입.
        # _voice_not_found_timeout 초 안에 YOLO 로 대상을 찾지 못하면
        # /object_not_found 를 발행해 publisher.py → scan_request 흐름을 트리거.
        self._voice_targets: list[str]  = []   # 탐색 대상 물체 목록
        self._voice_deadline: float     = 0.0  # 0.0 = 비활성
        self._voice_lock                = threading.Lock()

        # 이미지 해상도 (gesture 좌표 변환용)
        self._img_w = 640
        self._img_h = 480

        # ── 타이머: 0.3 초마다 voice_targets 미발견 체크 ─────────────────
        self.create_timer(0.3, self._check_voice_targets_cb,
                          callback_group=self._cb_group)

        self.get_logger().info('VisionNode ready')
        self.get_logger().info(
            f'voice_not_found_timeout={self._voice_not_found_timeout}s')

    # ─────────────────────────────────────────────────────────────────────
    # 구독 콜백
    # ─────────────────────────────────────────────────────────────────────

    def _gesture_cb(self, msg: GestureEvent):
        with self._gesture_lock:
            self._gesture_state = msg.gesture_state
            self._is_pointing   = msg.is_pointing
            self._is_fist       = msg.is_fist
            self._index_tip_x   = msg.index_tip_x
            self._index_tip_y   = msg.index_tip_y
            self._hand_visible  = msg.hand_visible
            self._landmarks_x   = list(msg.landmarks_x)
            self._landmarks_y   = list(msg.landmarks_y)

    def _is_picking_cb(self, msg: Bool):
        with self._pick_state_lock:
            self._is_picking_ext = msg.data
            if not msg.data:
                self._selected_label = None
                self._selected_box   = None
                self._hover_target   = None
                self._hover_start    = None

    # ── /voice_intent 콜백 (publisher.py 연동) ────────────────────────────

    def _voice_intent_cb(self, msg: String):
        """
        publisher.py → /voice_intent 수신 시 즉시 물체 존재 여부 확인.

        수신 형식:
          {"action": "bring_object", "target_object": ["water"],
           "urgency": "normal", "timestamp": "2026-05-07 21:21:37.280"}
          {"action": "bring_food", "target_object": ["사과"],
           "urgency": "normal", "from_scan": true,
           "best_pose": {...}, "best_bbox": [...], "timestamp": "..."}

        처리 흐름:
          ① bring_* 액션 여부 확인 (bring_object 포함 모든 bring_* 허용)
          ② from_scan=true 이면 → best_bbox 로 즉시 SelectedObject 발행
          ③ 일반 경우 → 현재 YOLO 탐지 결과를 즉시 확인
             - 발견된 대상 있음 : 즉시 SelectedObject 발행 → pick-and-place 트리거
             - 일부/전부 미발견 : _voice_not_found_timeout 초 동안 추가 대기
               → 타이머(_check_voice_targets_cb)가 최종 판정 후 /object_not_found 발행
        """
        try:
            data    = json.loads(msg.data)
            action  = data.get('action', '')
            targets = data.get('target_object', [])

            # ── ① bring_* 액션만 처리 (bring_object 포함) ───────────────
            if not _is_bring_action(action):
                return

            if isinstance(targets, str):
                targets = [targets]
            if not targets:
                return

            from_scan = bool(data.get('from_scan', False))
            best_bbox = data.get('best_bbox', [])

            self.get_logger().info(
                f'[VOICE_INTENT] action={action} targets={targets} '
                f'from_scan={from_scan}')

            # ── ② 스캔 후 재발행: YOLO 우선 탐지, 실패 시 VLM bbox 폴백 ──
            if from_scan and targets:
                label = targets[0]
                # YOLO 즉시 확인
                with self._detect_lock:
                    current_dets = list(self._detections)
                for det in current_dets:
                    if det['name'] == label:
                        self.get_logger().info(
                            f'[FROM_SCAN] ✅ YOLO 즉시 발견: {label}  '
                            f'conf={det["conf"]:.2f}')
                        self._auto_select_from_scan(
                            label, det['box'], det['conf'])
                        return
                # YOLO 미발견 → 백그라운드에서 timeout 초 폴링 후 이미지 중앙 픽
                self.get_logger().info(
                    f'[FROM_SCAN] YOLO 미발견 → '
                    f'{self._voice_not_found_timeout}s 대기')
                threading.Thread(
                    target=self._wait_for_yolo_then_pick,
                    args=(label, self._voice_not_found_timeout),
                    daemon=True,
                ).start()
                return

            # ── ③ 일반 경우: 현재 YOLO 탐지 결과 즉시 확인 ─────────────
            with self._detect_lock:
                current_dets  = list(self._detections)

            detected_names = {d['name'] for d in current_dets}

            found_now     = [t for t in targets if t in detected_names]
            missing_now   = [t for t in targets if t not in detected_names]

            self.get_logger().info(
                f'[VOICE] 즉시 확인 → 발견={found_now}, 미발견={missing_now}')

            # 현재 화면에 발견된 대상 → hover 없이 즉시 픽업 트리거
            if found_now:
                det_info = next(
                    d for d in current_dets if d['name'] == found_now[0])
                self.get_logger().info(
                    f'[VOICE] ✅ 현재 화면에 {found_now[0]} 발견 '
                    f'(conf={det_info["conf"]:.3f}) → 즉시 픽업 트리거')
                self._auto_select_from_scan(
                    found_now[0],
                    det_info['box'],
                    float(det_info.get('conf', 1.0)),
                )

            # 미발견 대상이 있으면 타임아웃 대기 후 재판정
            if missing_now:
                self.get_logger().warn(
                    f'[VOICE] ⚠️  현재 미발견: {missing_now} '
                    f'— {self._voice_not_found_timeout}s 대기 후 재판정')
                with self._voice_lock:
                    # 이미 발견된 것은 제외하고 미발견만 타이머 대기
                    self._voice_targets  = list(missing_now)
                    self._voice_deadline = time.time() + self._voice_not_found_timeout

        except Exception as exc:
            self.get_logger().warn(f'voice_intent 파싱 오류: {exc}')

    # ── 타이머: voice_targets 최종 판정 ──────────────────────────────────

    def _check_voice_targets_cb(self):
        """
        0.3 초마다 실행. deadline 이 지난 _voice_targets 를 최종 판정.

        _voice_intent_cb 즉시 확인 후의 두 가지 케이스:
          A) 이미 발견됨 (deadline=+0.4s): 타이머 통과 → 상태만 초기화
          B) 미발견 (deadline=+timeout): YOLO 재확인 후
             - 이번엔 발견됨 → 상태 초기화 (화면에서 찾은 것)
             - 여전히 없음   → /object_not_found 발행 → publisher.py 가 스캔 요청
        """
        with self._voice_lock:
            targets  = list(self._voice_targets)
            deadline = self._voice_deadline

        if not targets or deadline == 0.0 or time.time() < deadline:
            return

        # 최종 YOLO 결과 확인
        with self._detect_lock:
            detected_names = {d['name'] for d in self._detections}

        found     = [t for t in targets if t in detected_names]
        not_found = [t for t in targets if t not in detected_names]

        # 상태 초기화 (중복 발행 방지)
        with self._voice_lock:
            self._voice_targets  = []
            self._voice_deadline = 0.0

        if not_found:
            self.get_logger().warn(
                f'[VOICE] 최종 미발견: {not_found}  발견: {found}')
            self._publish_object_not_found(found, not_found)
        else:
            self.get_logger().info(
                f'[VOICE] 최종 확인 — 모두 화면에 존재: {found}')

    # ── /object_not_found 발행 ────────────────────────────────────────────

    def _publish_object_not_found(self, found: list, not_found: list):
        """
        publisher.py 가 구독하는 /object_not_found 발행.
        publisher.py 수신 형식:
          {"found": ["컵"], "not_found": ["사과"]}
        """
        payload = {
            'found'    : found,
            'not_found': not_found,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        }
        msg      = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_not_found.publish(msg)

        self.get_logger().info(
            f'[OBJECT_NOT_FOUND] → found={found}, not_found={not_found}')

    # ── 스캔 후 YOLO 대기 ────────────────────────────────────────────────

    def _wait_for_yolo_then_pick(self, label: str, timeout: float):
        """
        센터링 후 YOLO가 대상을 잡을 때까지 timeout 초 대기.
        발견 시 YOLO bbox로 픽 트리거.
        시간 초과 시 이미지 중앙 bbox로 픽 트리거 (로봇이 물체 위에 있으므로).
        VLM bbox는 사용하지 않음 (VLM은 센터링 전용).
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._detect_lock:
                dets = list(self._detections)
            for det in dets:
                if det['name'] == label:
                    self.get_logger().info(
                        f'[FROM_SCAN] ✅ YOLO 발견: {label}  '
                        f'conf={det["conf"]:.2f} → YOLO bbox로 픽')
                    self._auto_select_from_scan(
                        label, det['box'], det['conf'])
                    return
            time.sleep(0.1)

        # 타임아웃 → 이미지 중앙 bbox 사용 (센터링으로 로봇이 물체 위에 있음)
        self.get_logger().warn(
            f'[FROM_SCAN] YOLO {timeout:.1f}s 탐지 실패 '
            f'→ 이미지 중앙 bbox로 픽 시도: {label}')
        cx = self._img_w // 2
        cy = self._img_h // 2
        m  = 40
        center_bbox = [cx - m, cy - m, cx + m, cy + m]
        self._auto_select_from_scan(label, center_bbox, 0.5)

    # ── 스캔 결과로부터 즉시 선택 ─────────────────────────────────────────

    def _auto_select_from_scan(self, label: str, bbox: list, conf: float = 1.0):
        """
        workspace_scan_coordinator 가 찾은 물체를 hover 없이 즉시 선택.
        publisher.py 가 /voice_intent (from_scan=true) 로 재발행한
        best_bbox 좌표를 그대로 사용한다.

        SelectedObject 를 발행해 픽앤플레이스 파이프라인을 트리거.
        """
        box_int = [int(v) for v in bbox[:4]] if len(bbox) >= 4 else []

        # 선택 상태 갱신 (디스플레이용)
        with self._pick_state_lock:
            self._selected_label = label
            self._selected_box   = box_int
            self._hover_target   = None
            self._hover_start    = None

        # SelectedObject 발행
        so_msg              = SelectedObject()
        so_msg.header.stamp = self.get_clock().now().to_msg()
        so_msg.label        = label
        so_msg.confidence   = float(conf)
        so_msg.box          = box_int
        self._pub_obj.publish(so_msg)

        self.get_logger().info(
            f'[FROM SCAN] 자동 선택 → label={label} '
            f'conf={conf:.3f} bbox={box_int}')

    # ─────────────────────────────────────────────────────────────────────
    # 컬러 이미지 콜백 (YOLO + 제스처 + 호버)
    # ─────────────────────────────────────────────────────────────────────

    def _color_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        self._img_h, self._img_w = frame.shape[:2]

        # ── YOLO 검출 (매 _yolo_every 프레임마다 실행) ───────────────────
        if self._yolo is not None:
            self._yolo_skip = (self._yolo_skip + 1) % self._yolo_every
            if self._yolo_skip == 0:
                try:
                    results   = self._yolo.predict(
                        source=frame, conf=self._yolo_conf,
                        imgsz=640, save=False, verbose=False)
                    annotated = results[0].plot()
                    new_dets  = []
                    for box in results[0].boxes:
                        b        = box.xyxy[0].cpu().numpy().astype(int)
                        cls_id   = int(box.cls[0])
                        cls_name = self._yolo.names[cls_id]
                        conf     = float(box.conf[0])
                        new_dets.append(
                            {'name': cls_name, 'box': b.tolist(), 'conf': conf})
                    with self._detect_lock:
                        self._detections = new_dets
                except Exception as e:
                    self.get_logger().warn(f'YOLO predict error: {e}')
                    annotated = frame.copy()
            else:
                # YOLO 스킵: 캐시된 bbox 를 현재 프레임 위에 그림
                annotated = frame.copy()
                with self._detect_lock:
                    cached = list(self._detections)
                for det in cached:
                    b  = det['box']
                    lb = f"{det['name']} {det['conf']:.2f}"
                    cv2.rectangle(
                        annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                    cv2.putText(
                        annotated, lb, (b[0], b[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                        cv2.LINE_AA)
        else:
            annotated = frame.copy()

        with self._detect_lock:
            detections = list(self._detections)

        # ── 제스처 스냅샷 ─────────────────────────────────────────────────
        with self._gesture_lock:
            gesture_state = self._gesture_state
            is_pointing   = self._is_pointing
            is_fist_g     = self._is_fist
            tip_x         = self._index_tip_x
            tip_y         = self._index_tip_y
            hand_visible  = self._hand_visible
            landmarks_x   = list(self._landmarks_x)
            landmarks_y   = list(self._landmarks_y)

        ix = int(tip_x * self._img_w)
        iy = int(tip_y * self._img_h)

        # ── 픽/호버 상태 스냅샷 ───────────────────────────────────────────
        with self._pick_state_lock:
            is_picking_ext = self._is_picking_ext
            hover_target   = self._hover_target
            hover_start    = self._hover_start
            selected_label = self._selected_label
            selected_box   = self._selected_box

        # ── 호버 판정 ─────────────────────────────────────────────────────
        current_hover_name: str | None  = None
        current_hover_box: list | None  = None
        if is_pointing and not is_picking_ext:
            for det in detections:
                b = det['box']
                if b[0] <= ix <= b[2] and b[1] <= iy <= b[3]:
                    current_hover_name = det['name']
                    current_hover_box  = b
                    break

        if not is_picking_ext:
            if current_hover_name is not None:
                if current_hover_name == hover_target:
                    elapsed  = time.time() - hover_start
                    progress = min(elapsed / self._hover_sec, 1.0)
                    draw_hover_progress(
                        annotated, current_hover_box, progress, current_hover_name)
                    if (elapsed >= self._hover_sec and
                            selected_label != current_hover_name):
                        selected_label = current_hover_name
                        selected_box   = current_hover_box
                        hover_target   = None
                        hover_start    = None
                        self.get_logger().info(f'[SELECTED] {current_hover_name}')
                        self._publish_selected(
                            current_hover_name, current_hover_box, detections)
                else:
                    hover_target = current_hover_name
                    hover_start  = time.time()
            else:
                hover_target = None
                hover_start  = None

        with self._pick_state_lock:
            self._hover_target   = hover_target
            self._hover_start    = hover_start
            self._selected_label = selected_label
            self._selected_box   = selected_box

        # ── 손 스켈레톤 ───────────────────────────────────────────────────
        if (hand_visible and len(landmarks_x) == 21
                and any(v != 0.0 for v in landmarks_x)):
            if is_fist_g:
                sk_color  = (0, 0, 220)
                hint_text = 'PAUSE'
            elif is_pointing and current_hover_name:
                sk_color  = (0, 220, 255)
                hint_text = f'☝ → {current_hover_name}'
            elif is_pointing:
                sk_color  = (200, 200, 0)
                hint_text = '☝ POINTING'
            else:
                sk_color  = (0, 200, 100)
                hint_text = ''

            pts = [(int(landmarks_x[i] * self._img_w),
                    int(landmarks_y[i] * self._img_h)) for i in range(21)]
            for a, b_conn in HAND_CONNECTIONS:
                cv2.line(annotated, pts[a], pts[b_conn], sk_color, 2, cv2.LINE_AA)
            for px, py in pts:
                cv2.circle(annotated, (px, py), 5, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(annotated, (px, py), 5, sk_color, 1, cv2.LINE_AA)

            if is_pointing:
                tip_pt = (ix, iy)
                cv2.circle(annotated, tip_pt, 14, sk_color, 2, cv2.LINE_AA)
                cv2.circle(annotated, tip_pt,  5, sk_color, -1, cv2.LINE_AA)
                if hint_text:
                    cv2.putText(annotated, hint_text,
                                (tip_pt[0] + 16, tip_pt[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, sk_color, 2,
                                cv2.LINE_AA)

            s_color = COLORS.get(gesture_state, COLORS['NONE'])
            cv2.putText(annotated, f'[{gesture_state}]', (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, s_color, 2, cv2.LINE_AA)

        # ── voice_intent 탐색 중 오버레이 ─────────────────────────────────
        with self._voice_lock:
            v_targets  = list(self._voice_targets)
            v_deadline = self._voice_deadline

        if v_targets and v_deadline > 0.0:
            remaining = max(0.0, v_deadline - time.time())
            overlay   = f'탐색 중: {", ".join(v_targets)}  ({remaining:.1f}s)'
            cv2.putText(annotated, overlay, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2,
                        cv2.LINE_AA)

        # ── 선택 / PICKING 오버레이 ───────────────────────────────────────
        if selected_label and not is_picking_ext:
            draw_selected_label(annotated, selected_label)
        if is_picking_ext:
            draw_picking_overlay(annotated, selected_label or '')

        # ── 하단 힌트 ─────────────────────────────────────────────────────
        cv2.putText(annotated,
                    '☝ point at obj + hold=PICK  |  PAUSE  |  C:clear',
                    (10, self._img_h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

        with self._frame_lock:
            self._latest_frame = annotated

    # ─────────────────────────────────────────────────────────────────────
    # 발행 헬퍼
    # ─────────────────────────────────────────────────────────────────────

    def _publish_selected(self, label: str, box: list, detections: list):
        """제스처 호버 선택 → /selected_object 발행"""
        conf             = next(
            (d['conf'] for d in detections if d['name'] == label), 0.0)
        msg              = SelectedObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.label        = label
        msg.confidence   = float(conf)
        msg.box          = [int(v) for v in box]
        self._pub_obj.publish(msg)

    # ─────────────────────────────────────────────────────────────────────
    # 디스플레이 루프 (별도 스레드)
    # ─────────────────────────────────────────────────────────────────────

    def run_display_loop(self):
        cv2.namedWindow('RealSense | YOLO + Hover', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RealSense | YOLO + Hover', 1280, 720)

        while rclpy.ok():
            with self._frame_lock:
                frame = self._latest_frame

            if frame is not None:
                cv2.imshow('RealSense | YOLO + Hover', frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('c'), ord('C')):
                with self._pick_state_lock:
                    self._selected_label = None
                    self._selected_box   = None
                    self._hover_target   = None
                    self._hover_start    = None
                with self._voice_lock:
                    self._voice_targets  = []
                    self._voice_deadline = 0.0
                self.get_logger().info('[CLEAR] Selection + voice targets reset.')

        cv2.destroyAllWindows()


# ══════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_display_loop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
