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

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import json
from std_msgs.msg import Bool, String
from dsr_msgs2.msg import RobotState
from dsr_msgs2.srv import MoveLine

from gesture_robot_interfaces.msg import GestureEvent, SelectedObject
from gesture_robot_interfaces.action import PickAndPlace
from gesture_robot_pkg.constants import ROBOT_ID, STATE_CONTROLLING, ABS_VEL, ABS_ACC, ABS_BLEND_R


class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator_node')

        self._cb_group = ReentrantCallbackGroup()

        # ── 퍼블리셔 ──────────────────────────────────
        self._pub_teleop     = self.create_publisher(Bool, '/teleop/enable',  10)
        self._pub_is_picking = self.create_publisher(Bool, '/is_picking',     10)

        # ── 구독 ─────────────────────────────────────
        self.create_subscription(
            GestureEvent, '/gesture_event', self._gesture_cb, 10,
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

        # ── move_line 서비스 클라이언트 (teleop용) ──
        self._move_line_cli = self.create_client(
            MoveLine, f'/{ROBOT_ID}/motion/move_line',
            callback_group=self._cb_group)
        if not self._move_line_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('move_line service not found')

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
            self.get_logger().info(f'[SCAN RESULT] status={status}')
        except Exception as e:
            self.get_logger().warn(f'[SCAN RESULT] 파싱 오류: {e}')

    def _gesture_cb(self, msg: GestureEvent):
        prev               = self._gesture_state
        self._gesture_state = msg.gesture_state

        if msg.gesture_state == STATE_CONTROLLING and not self._is_picking:
            if not self._teleop_ever_enabled or prev != STATE_CONTROLLING:
                self._teleop_ever_enabled = True
                self._set_teleop(True)
            # target_pos_mm이 유효하면 move_line 호출
            if (len(msg.target_pos_mm) >= 6
                    and any(v != 0.0 for v in msg.target_pos_mm[:3])):
                ready = self._move_line_cli.service_is_ready()
                self.get_logger().info(f'[MOVE] service_ready={ready} pos={list(msg.target_pos_mm[:3])}')
                if ready:
                    self._call_move_line(list(msg.target_pos_mm))
        elif msg.gesture_state != STATE_CONTROLLING and not self._is_picking:
            self._set_teleop(False)

    def _call_move_line(self, pos: list):
        req            = MoveLine.Request()
        req.pos        = [float(v) for v in pos[:6]]
        req.vel        = [ABS_VEL, ABS_VEL]
        req.acc        = [ABS_ACC, ABS_ACC]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = 0   # 0=base frame
        req.mode       = 0   # 0=absolute
        req.blend_type = 0
        req.sync_type  = 1   # 1=async (블로킹 없음)
        self._move_line_cli.call_async(req)

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
            goal.from_scan = from_scan   # ← HOME 스킵 여부 전달

            self.get_logger().info(f'[SEND GOAL] from_scan={from_scan}')
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