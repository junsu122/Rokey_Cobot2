"""
orchestrator_node.py
────────────────────
책임:
  1. GestureEvent 구독 → teleop enable/disable
  2. SelectedObject 구독 → pick-and-place 트리거
  3. robot_state 구독 → TCP 캐시 (pick_and_place_node 에도 동일하게 구독됨)
  4. PickAndPlace 액션 결과 수신 → teleop 재개

퍼블리시:  /teleop/enable   (std_msgs/Bool)
           /is_picking       (std_msgs/Bool)
구독:      /gesture_event   (GestureEvent)
           /selected_object (SelectedObject)
           /{ROBOT_ID}/state (dsr_msgs2/RobotState)
액션 클라이언트: /pick_and_place (PickAndPlace)
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, String
from dsr_msgs2.msg import RobotState

from gesture_robot_interfaces.msg import GestureEvent, SelectedObject
from gesture_robot_interfaces.action import PickAndPlace
from gesture_robot_pkg.constants import (ROBOT_ID, STATE_CALIBRATING, STATE_CONTROLLING)


POINT_LEFT = 'POINT_LEFT'
POINT_RIGHT = 'POINT_RIGHT'
POINT_CENTER = 'POINT_CENTER'
POINT_NONE = 'NONE'
UPPER_POINT_LEFT = 'POINT_LEFT'
UPPER_POINT_RIGHT = 'POINT_RIGHT'


class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator_node')

        self._cb_group = ReentrantCallbackGroup()

        # ── 퍼블리셔 ──────────────────────────────────
        self._pub_teleop     = self.create_publisher(Bool, '/teleop/enable',  10)
        self._pub_is_picking = self.create_publisher(Bool, '/is_picking',     10)
        self._pub_debug      = self.create_publisher(String, '/orchestrator/fusion_debug', 10)

        # ── 구독 ─────────────────────────────────────
        self.create_subscription(
            GestureEvent, '/gesture_event', self._gesture_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/skeleton/upper_body', self._upper_body_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            SelectedObject, '/selected_object', self._selected_obj_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            RobotState, f'/{ROBOT_ID}/state', self._robot_state_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/scan_result', self._scan_result_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/voice_intent', self._voice_intent_cb, 10,
            callback_group=self._cb_group)

        # ── PickAndPlace 액션 클라이언트 ────────────
        self._pick_client = ActionClient(
            self, PickAndPlace, '/pick_and_place',
            callback_group=self._cb_group)
        self.get_logger().info('Waiting for /pick_and_place action server...')
        if not self._pick_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('/pick_and_place server not found — pick will be queued')

        # ── 내부 상태 ──────────────────────────────────
        self._gesture_state       = ''
        self._is_picking          = False
        self._pick_lock           = threading.Lock()   # _is_picking 원자적 체크-앤-셋
        self._pick_start_time     = None               # 픽 시작 시각 (타임아웃용)
        self._teleop_ever_enabled = False
        self._current_pos         = [0.0] * 6
        self._pos_received        = False
        self._next_pick_from_scan = False              # 스캔 결과로 트리거된 픽 여부

        # 픽 타임아웃 워치독 (120초 초과 시 강제 리셋)
        PICK_TIMEOUT_SEC = 120.0
        self.create_timer(10.0, lambda: self._pick_watchdog(PICK_TIMEOUT_SEC))

        self._pos_received        = False
        self._state_lock          = threading.Lock()
        self._last_warn_time      = 0.0

        # 최근 GestureEvent 원본
        self._is_pointing         = False
        self._index_tip_x         = 0.0
        self._index_tip_y         = 0.0
        self._avg_x               = 0.0
        self._avg_y               = 0.0
        self._gesture_target_pos  = [0.0] * 6
        self._gesture_velocity    = [0.0, 0.0, 0.0]

        # 최근 upper_body skeleton 상태
        self._upper_body_payload  = {}
        self._upper_body_visible   = False
        self._upper_body_gesture   = 'NONE'
        self._arm_reach            = None
        self._arm_reach_valid      = False
        self._base_arm_reach       = None
        self._arm_reach_samples    = []
        self._calibrating_for_arm  = False

        # 최종 판단 결과
        self._final_point_gesture  = POINT_NONE
        self._point_confidence     = 0.0
        self._y_norm               = 0.0
        self._y_delta_mm           = 0.0

        self._set_teleop(False)
        self.get_logger().info('OrchestratorNode ready')

    # ── 구독 콜백 ─────────────────────────────────────

    def _robot_state_cb(self, msg: RobotState):
        if len(msg.current_posx) >= 6:
            self._current_pos  = list(msg.current_posx)
            self._pos_received = True

    def _voice_intent_cb(self, msg: String):
        """
        /voice_intent 수신 시 from_scan=True 확인 → _next_pick_from_scan 세팅.
        workspace_scan_node가 각 물체마다 voice_intent를 직접 발행하므로
        첫 번째뿐 아니라 두 번째, 세 번째 픽에도 from_scan=True가 적용됨.
        """
        try:
            data = json.loads(msg.data)
            if data.get('from_scan'):
                self._next_pick_from_scan = True
                self.get_logger().info(
                    '[VOICE_INTENT] from_scan=True → 다음 픽 HOME 스킵 예약')
        except Exception as e:
            self.get_logger().warn(f'[VOICE_INTENT CB] 파싱 오류: {e}')

    def _scan_result_cb(self, msg: String):
        """스캔 결과 수신 (TTS/상태 확인용 — 픽은 workspace_scan_node가 완료 후 발행)"""
        try:
            data   = json.loads(msg.data)
            status = data.get('status', '')
            if status == 'found':
                self._next_pick_from_scan = True
                self.get_logger().info(
                    '[SCAN] found → 다음 픽은 스캔 위치에서 수행 (from_scan=True)')
        except Exception as e:
            self.get_logger().warn(f'[SCAN RESULT] 파싱 오류: {e}')

    def _gesture_cb(self, msg: GestureEvent):
        with self._state_lock:
            prev               = self._gesture_state
            self._gesture_state = msg.gesture_state
            self._is_pointing   = msg.is_pointing
            self._index_tip_x   = msg.index_tip_x
            self._index_tip_y   = msg.index_tip_y
            self._avg_x         = msg.avg_x
            self._avg_y         = msg.avg_y
            self._gesture_target_pos = list(msg.target_pos_mm)
            self._gesture_velocity   = list(msg.velocity_delta)

            self._update_arm_reach_calibration_locked(prev, msg.gesture_state)
            self._recompute_fusion_locked()
            self._publish_debug_locked()

            if msg.gesture_state == STATE_CONTROLLING and not self._is_picking:
                if not self._teleop_ever_enabled or prev != STATE_CONTROLLING:
                    self._teleop_ever_enabled = True
                    self._set_teleop(True)
            elif msg.gesture_state != STATE_CONTROLLING and not self._is_picking:
                self._set_teleop(False)

    def _upper_body_cb(self, msg: String):
        with self._state_lock:
            try:
                payload = json.loads(msg.data)
            except Exception as exc:
                self._throttled_warn(f'[UPPER BODY] JSON parse failed: {exc}')
                return

            self._upper_body_payload = payload
            self._upper_body_visible = bool(payload.get('body_visible', False))
            self._upper_body_gesture = str(payload.get('upper_body_gesture', 'NONE'))
            self._arm_reach, self._arm_reach_valid = self._compute_arm_reach(payload)

            self._update_arm_reach_calibration_locked(self._gesture_state, self._gesture_state)
            self._recompute_fusion_locked()
            self._publish_debug_locked()

    def _selected_obj_cb(self, msg: SelectedObject):
        """물체가 선택되면 텔레오퍼레이션을 중지하고 pick-and-place를 시작한다."""
        with self._pick_lock:
            if self._is_picking:
                self.get_logger().warn('[PICK SKIP] already picking')
                return
            self._is_picking      = True
            self._pick_start_time = time.time()

        if not self._pos_received:
            self.get_logger().warn(
                '[PICK WARN] robot state not yet received — TCP may be zero. '
                'Proceeding anyway (pick_and_place_node will wait for TCP).')

        self.get_logger().info(
            f'[PICK START] label={msg.label}  box={list(msg.box)}  '
            f'TCP={[f"{v:.1f}" for v in self._current_pos[:3]]}  '
            f'gesture={self._gesture_state}')

        self._set_teleop(False)
        self._set_is_picking(True)
        self._send_pick_goal(msg)

    # ── 액션 ─────────────────────────────────────────

    def _send_pick_goal(self, obj_msg: SelectedObject):
        try:
            from_scan      = self._next_pick_from_scan
            self._next_pick_from_scan = False

            goal           = PickAndPlace.Goal()
            goal.label     = obj_msg.label
            goal.box       = [int(v) for v in obj_msg.box]
            goal.robot_tcp = [float(v) for v in self._current_pos]
            goal.from_scan = from_scan

            if from_scan:
                self.get_logger().info('[PICK] from_scan=True → HOME 단계 스킵')

            fut = self._pick_client.send_goal_async(
                goal, feedback_callback=self._pick_feedback_cb)
            fut.add_done_callback(self._pick_goal_accepted_cb)
        except Exception as e:
            self.get_logger().error(f'[SEND GOAL] {e}')
            self._on_pick_done(success=False)

    def _pick_goal_accepted_cb(self, future):
        try:
            gh = future.result()
        except Exception as e:
            self.get_logger().error(f'[PICK] send_goal exception: {e}')
            self._on_pick_done(success=False)
            return
        if not gh.accepted:
            self.get_logger().error('[PICK] goal rejected')
            self._on_pick_done(success=False)
            return
        self.get_logger().info('[PICK] goal accepted')
        gh.get_result_async().add_done_callback(self._pick_result_cb)

    def _pick_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[PICK FB] stage={fb.stage}  {fb.progress*100:.0f}%')

    def _pick_result_cb(self, future):
        try:
            res = future.result().result
            self.get_logger().info(
                f'[PICK RESULT] success={res.success}  "{res.message}"')
            self._on_pick_done(success=res.success)
        except Exception as e:
            self.get_logger().error(f'[PICK] result exception: {e}')
            self._on_pick_done(success=False)

    def _pick_watchdog(self, timeout_sec: float):
        """픽이 timeout_sec 초 이상 걸리면 _is_picking을 강제 리셋."""
        with self._pick_lock:
            if not self._is_picking or self._pick_start_time is None:
                return
            elapsed = time.time() - self._pick_start_time
            if elapsed < timeout_sec:
                return
        self.get_logger().error(
            f'[WATCHDOG] 픽 타임아웃 {elapsed:.0f}s — _is_picking 강제 리셋')
        self._on_pick_done(success=False)

    def _on_pick_done(self, success: bool):
        with self._pick_lock:
            self._is_picking = False
        self._set_is_picking(False)
        if self._gesture_state == STATE_CONTROLLING:
            self._set_teleop(True)
        self.get_logger().info(f'[PICK DONE] success={success}  teleop resumed')

    # ── 헬퍼 ─────────────────────────────────────────

    def _throttled_warn(self, message: str):
        now = time.monotonic()
        if now - self._last_warn_time >= 1.0:
            self.get_logger().warn(message)
            self._last_warn_time = now

    def _distance(self, a: list[float], b: list[float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx, dy)

    def _compute_arm_reach(self, payload: dict) -> tuple[float | None, bool]:
        """상체 payload에서 팔 뻗음 정도를 1.0 기준 근처의 비율로 계산한다."""
        if not payload.get('body_visible', False):
            return None, False

        keys = (
            'left_shoulder', 'right_shoulder',
            'left_elbow', 'right_elbow',
            'left_wrist', 'right_wrist',
        )
        if any(payload.get(key) is None for key in keys):
            return None, False

        left_shoulder = payload['left_shoulder']
        right_shoulder = payload['right_shoulder']
        left_wrist = payload['left_wrist']
        right_wrist = payload['right_wrist']

        shoulder_width = self._distance(left_shoulder, right_shoulder)
        if shoulder_width <= 1e-6:
            return None, False

        left_reach = self._distance(left_shoulder, left_wrist) / shoulder_width
        right_reach = self._distance(right_shoulder, right_wrist) / shoulder_width
        return (left_reach + right_reach) / 2.0, True

    def _update_arm_reach_calibration_locked(self, prev_state: str, current_state: str):
        """CALIBRATING 상태에서 arm_reach 기준값을 모으고, 종료 시 base_arm_reach로 고정한다."""
        if current_state == STATE_CALIBRATING:
            if not self._calibrating_for_arm:
                self._arm_reach_samples = []
                self._base_arm_reach = None
                self._calibrating_for_arm = True
            if self._arm_reach_valid and self._arm_reach is not None:
                self._arm_reach_samples.append(self._arm_reach)
            return

        if self._calibrating_for_arm and prev_state == STATE_CALIBRATING:
            if self._arm_reach_samples:
                self._base_arm_reach = sum(self._arm_reach_samples) / len(self._arm_reach_samples)
            self._arm_reach_samples = []
            self._calibrating_for_arm = False

    def _resolve_point_gesture_locked(self):
        """검지 포인팅을 최우선으로 최종 POINT_LEFT/RIGHT/POINT_CENTER를 계산한다."""
        if not self._is_pointing:
            return POINT_NONE, 0.0

        if self._index_tip_x < 0.4:
            final_point = POINT_LEFT
        elif self._index_tip_x > 0.6:
            final_point = POINT_RIGHT
        else:
            return POINT_CENTER, 0.0

        point_confidence = 0.7
        upper = self._upper_body_gesture

        if upper == final_point:
            point_confidence = 1.0
        elif upper in (UPPER_POINT_LEFT, UPPER_POINT_RIGHT) and upper != final_point:
            point_confidence = 0.3
            self._throttled_warn(
                f'[POINT WARN] finger={final_point} upper_body={upper} '
                '-> finger direction wins')
        elif upper != 'NONE':
            point_confidence = 0.85

        return final_point, point_confidence

    def _resolve_y_delta_locked(self):
        """arm_reach 기반 Y 이동값을 계산한다."""
        if not self._upper_body_visible or not self._arm_reach_valid:
            return 0.0, 0.0
        if self._base_arm_reach is None:
            return 0.0, 0.0

        y_norm = self._arm_reach - self._base_arm_reach
        if abs(y_norm) < ARM_REACH_DEAD_ZONE:
            return y_norm, 0.0

        y_delta_mm = -1.0 * y_norm * Y_SCALE_MM
        return y_norm, y_delta_mm

    def _recompute_fusion_locked(self):
        self._final_point_gesture, self._point_confidence = self._resolve_point_gesture_locked()
        self._y_norm, self._y_delta_mm = self._resolve_y_delta_locked()

    def _publish_debug_locked(self):
        """디버그/확인용 상태를 JSON으로 publish한다."""
        msg = String()
        msg.data = json.dumps({
            'gesture_state': self._gesture_state,
            'final_point_gesture': self._final_point_gesture,
            'point_confidence': round(float(self._point_confidence), 3),
            'upper_body_gesture': self._upper_body_gesture,
            'body_visible': self._upper_body_visible,
            'arm_reach': round(float(self._arm_reach), 4) if self._arm_reach is not None else None,
            'base_arm_reach': round(float(self._base_arm_reach), 4) if self._base_arm_reach is not None else None,
            'y_norm': round(float(self._y_norm), 4),
            'y_delta_mm': round(float(self._y_delta_mm), 3),
            'is_pointing': self._is_pointing,
            'index_tip_x': round(float(self._index_tip_x), 4),
            'index_tip_y': round(float(self._index_tip_y), 4),
            'avg_x': round(float(self._avg_x), 4),
            'avg_y': round(float(self._avg_y), 4),
            'left_elbow': self._upper_body_payload.get('left_elbow'),
            'right_elbow': self._upper_body_payload.get('right_elbow'),
            'left_wrist': self._upper_body_payload.get('left_wrist'),
            'right_wrist': self._upper_body_payload.get('right_wrist'),
        })
        self._pub_debug.publish(msg)

    def _set_teleop(self, enabled: bool):
        msg      = Bool()
        msg.data = enabled
        self._pub_teleop.publish(msg)
        # self.get_logger().info(f'[TELEOP] enable={enabled}')

    def _set_is_picking(self, picking: bool):
        msg      = Bool()
        msg.data = picking
        self._pub_is_picking.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
