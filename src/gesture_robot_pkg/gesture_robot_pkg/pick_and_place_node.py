"""
pick_and_place_node.py
──────────────────────────────────────────────────────────
[수정 사항]
  ★ v2 핵심 버그 제거
      self.executor.create_task() — executor 미설정으로 AttributeError 발생
      → 모든 스테이지가 즉시 실패해 로봇이 전혀 움직이지 않던 원인
      → v1 방식(단순 await + threading.Event)으로 복원

  ★ descent 계산식 v6 복원
      v2: np.clip(h*0.5, min(DESCENT_MIN_MM, h*0.4), DESCENT_MAX_MM)
      → v6: np.clip(h*0.5, DESCENT_MIN_MM, DESCENT_MAX_MM)

  ★ 속도 향상
      PICK_VEL / PICK_ACC 를 constants.py 에서 가져옴 (200 mm/s)

  ★ [FIX] 하강 목표 Z 안전 범위 검증 추가
      pick_target[2] 만 체크하던 기존 코드에서
      실제 하강 목표 Z (pick_target[2] - descent_d) 도 PICK_SAFE_Z_MIN_MM 와 비교
      → DSR 이 모션 거부하던 원인 제거

  ★ [FIX] 하강 속도 0 방어 처리
      PICK_VEL // 2 가 0 이 되는 경우를 max(1, ...) 로 방어

  ★ [FIX] obj_height_mm 진단 로그 강화
      하강량이 너무 작을 때 원인 파악 가능하도록 로그 추가

[설계 유지]
  - sync_type=0 : 각 이동 완료 후 다음 단계 진행 (DSR SYNC=0, ASYNC=1)
  - MultiThreadedExecutor + ReentrantCallbackGroup
  - threading.Event 로 TCP 대기 (MultiThreadedExecutor 에서는 블로킹 OK)
  - rclpy.task.Future 직접 await (asyncio 미사용)
  - 깊이 기반 3D 위치 추정 (v6 3-step 알고리즘)

책임:
  1. PickAndPlace 액션 서버
  2. depth + camera_info 구독 → 3D 위치 추정
  3. robot_state 구독 → 실행 시점 최신 TCP
  4. 카메라 → 로봇 베이스 좌표 변환 (hand-eye calibration)
  5. 물체 방향 보정 (종횡비 기반 rz 오프셋)
  6. 물체 높이 비례 approach/descent/lift 자동 계산
  7. 픽앤플레이스 시퀀스 실행
"""

import time
import threading

import numpy as np
from std_msgs.msg import Float32 
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from dsr_msgs2.srv import MoveLine, MoveJoint, SetRobotMode, GetCurrentPosx
from dsr_msgs2.msg import RobotState

from gesture_robot_interfaces.action import PickAndPlace
from gesture_robot_pkg.constants import (
    ROBOT_ID,
    GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT,
    GRIPPER2CAM_PATH,
    REALSENSE_DEPTH_TOPIC, REALSENSE_INFO_TOPIC,
    PICK_VEL, PICK_ACC,
    PICK_MIN_DEPTH_MM, PICK_MAX_DEPTH_MM, DEPTH_SAMPLE_MARGIN,
    PICK_DEPTH_OFFSET,
    APPROACH_MIN_MM, APPROACH_MAX_MM,
    LIFT_MIN_MM, LIFT_MAX_MM,
    PICK_EXTRA_DESCENT_MM,
    GRIPPER_TABLE_CLEARANCE_MM,
    SPIN_ANGLE_OFFSET,
    HOME_JOINT,
    PICK_SAFE_Z_MIN_MM, PICK_SAFE_Z_MAX_MM,
    PICK_OFFSET_X_MM, PICK_OFFSET_Y_MM, PICK_OFFSET_Z_MM,
)
from gesture_robot_pkg.utils import transform_camera_to_base


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_node')

        self._cb_group = ReentrantCallbackGroup()
        self._bridge   = CvBridge()

        self.angle_sub = self.create_subscription(
            Float32, 
            '/object_angle', 
            self._angle_cb, 
            10,
            callback_group=self._cb_group)
        self._spin_angle = 180.0 # 수신된 각도를 저장할 변수  ####################각도 변환 준수가 쓰는 변수####################3


        # ── 액션 서버 ─────────────────────────────────────────────────────
        self._action_server = ActionServer(
            self, PickAndPlace, '/pick_and_place',
            execute_callback = self._execute_cb,
            goal_callback    = self._goal_cb,
            cancel_callback  = self._cancel_cb,
            callback_group   = self._cb_group,
        )

        # ── depth / camera_info ───────────────────────────────────────────
        self.create_subscription(
            Image, REALSENSE_DEPTH_TOPIC, self._depth_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            CameraInfo, REALSENSE_INFO_TOPIC, self._info_cb, 10,
            callback_group=self._cb_group)
        self._depth_frame    = None
        self._depth_lock     = threading.Lock()
        self._intrinsics     = None
        self._intr_lock      = threading.Lock()
        self._table_depth_mm = None   # 전체 프레임 기반 테이블 depth EMA
        self._table_lock     = threading.Lock()

        # ── 로봇 상태 (실행 시점 최신 TCP + 로봇 상태코드) ─────────────────
        self.create_subscription(
            RobotState, f'/{ROBOT_ID}/state', self._robot_state_cb, 10,
            callback_group=self._cb_group)
        self._live_tcp          = None
        self._live_robot_state  = -1    # -1 = 미수신, 1=STANDBY, 2=MOVING ...
        self._tcp_lock          = threading.Lock()
        self._tcp_received      = threading.Event()

        # ── 로봇 서비스 클라이언트 ─────────────────────────────────────────
        self._movel_cli    = self.create_client(
            MoveLine,        f'/{ROBOT_ID}/motion/move_line',
            callback_group=self._cb_group)
        self._movej_cli    = self.create_client(
            MoveJoint,       f'/{ROBOT_ID}/motion/move_joint',
            callback_group=self._cb_group)
        self._set_mode_cli = self.create_client(
            SetRobotMode,    f'/{ROBOT_ID}/system/set_robot_mode',
            callback_group=self._cb_group)
        self._get_posx_cli = self.create_client(
            GetCurrentPosx,  f'/{ROBOT_ID}/aux_control/get_current_posx',
            callback_group=self._cb_group)
        for cli, svc in [(self._movel_cli,    'move_line'),
                         (self._movej_cli,    'move_joint'),
                         (self._set_mode_cli, 'set_robot_mode'),
                         (self._get_posx_cli, 'get_current_posx')]:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'{svc} service not found')

        # ── 그리퍼 ───────────────────────────────────────────────────────
        self._gripper = None
        try:
            from gesture_robot_pkg.onrobot import RG
            self._gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)
            self._gripper.open_gripper()
            self.get_logger().info(f'Gripper ({GRIPPER_NAME}) connected.')
        except Exception as e:
            self.get_logger().warn(f'Gripper init failed: {e}')

        self.get_logger().info('PickAndPlaceNode ready')

    # ── 코드 실행시 home으로 이동 로직 ───────────────────────────── ###################################### 홈으로 잘 가는지 확인필요
        self.get_logger().info("🚀 [PICK_AND_PLACE_NODE] Initialized & Ready") 

        # [추가] 노드 시작 시 로봇을 홈 위치로 자동 이동
        self._home_timer = self.create_timer(1.0, self._initial_home_move, callback_group=self._cb_group)


    # ── 구독 콜백 ─────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if raw.dtype == np.uint16:
                mm = raw.astype(np.float32)
            elif raw.dtype in (np.float32, np.float64):
                if msg.encoding == '32FC1':
                    # RealSense 32FC1 → 미터 단위, mm로 변환
                    mm = raw.astype(np.float32) * 1000.0
                else:
                    # 그 외: 유한값만 골라 median으로 단위 추정 (fallback)
                    nz = raw[(raw > 0) & np.isfinite(raw)]
                    mm = (raw.astype(np.float32) * 1000.0
                          if (len(nz) and float(np.median(nz)) < 10.0)
                          else raw.astype(np.float32))
            else:
                mm = raw.astype(np.float32)
            with self._depth_lock:
                self._depth_frame = mm
        except Exception as e:
            self.get_logger().error(f'Depth CvBridge: {e}')
            return

        # ── 테이블 depth 상시 추정 (EMA) ──────────────────────────────
        # 카메라가 위에서 내려다볼 때 테이블이 화면 대부분을 차지.
        # 물체는 테이블보다 카메라에 가까우므로 (depth 작음) 하위에 분포.
        # 유효 픽셀의 75th percentile ≈ 테이블 depth.
        try:
            valid = mm[(mm > PICK_MIN_DEPTH_MM) & (mm < PICK_MAX_DEPTH_MM)]
            if len(valid) > 200:
                candidate = float(np.percentile(valid, 75))
                with self._table_lock:
                    if self._table_depth_mm is None:
                        self._table_depth_mm = candidate
                    else:
                        self._table_depth_mm = self._table_depth_mm * 0.97 + candidate * 0.03
        except Exception:
            pass

    def _info_cb(self, msg: CameraInfo):
        with self._intr_lock:
            self._intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'ppx': msg.k[2], 'ppy': msg.k[5],
            }

    def _robot_state_cb(self, msg: RobotState):
        if len(msg.current_posx) >= 6:
            with self._tcp_lock:
                self._live_tcp         = list(msg.current_posx)
                self._live_robot_state = int(msg.robot_state)
            self._tcp_received.set()

    def _angle_cb(self, msg: Float32):
        """ /object_angle 토픽 수신 시 호출되는 콜백 """
        self._spin_angle = float(msg.data)
        # [로그 추가] 수신된 각도를 소수점 첫째자리까지 출력
        self.get_logger().info(f'[TOPIC] Received object_angle: {self._spin_angle:.1f}°')

    # ── 액션 콜백 ─────────────────────────────────────────────────────────

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f'[GOAL] label={goal_request.label}  box={list(goal_request.box)}')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _):
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        """
        픽앤플레이스 전체 시퀀스.

        async def 이지만 MultiThreadedExecutor + ReentrantCallbackGroup 환경이므로
        threading.Event.wait / time.sleep 같은 단기 블로킹 호출은 허용.
        다른 구독 콜백은 별도 스레드에서 계속 동작.

        await 는 오직 rclpy.task.Future (call_async 반환값) 만 사용.
        """
        goal   = goal_handle.request
        self._current_goal_box = list(goal.box)
        result = PickAndPlace.Result()
        try:
            return await self._execute_inner(goal_handle, goal, result)
        except Exception as e:
            self.get_logger().error(
                f'[EXECUTE] 예상치 못한 예외 발생 — goal_handle.abort() 강제 호출: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            result.success = False
            result.message = f'Unexpected exception: {e}'
            goal_handle.abort()
            return result

    async def _execute_inner(self, goal_handle, goal, result):

        # ── TCP 획득: GetCurrentPosx 서비스 우선, 상태 토픽 fallback ──────
        # dsr_hw_interface2 드라이버가 /dsr01/state 토픽을 발행하지 않는 경우가 있어
        # GetCurrentPosx 서비스를 먼저 시도한다.
        robot_tcp = await self._query_tcp_from_service()
        tcp_src   = 'service(GetCurrentPosx)'

        if robot_tcp is None:
            # 서비스 실패 시 상태 토픽 fallback (최대 3초 대기)
            self.get_logger().warn('[TCP] GetCurrentPosx 실패 — 상태 토픽 fallback 시도 (3초)')
            self._tcp_received.wait(timeout=3.0)
            with self._tcp_lock:
                live = self._live_tcp
            
            if live is not None:
                robot_tcp = live
                tcp_src   = 'state_topic'
            else:
                robot_tcp = list(goal.robot_tcp)
                tcp_src   = 'fallback(goal)'
                self.get_logger().warn('[TCP] 상태 토픽 미수신 — goal TCP로 대체')

        # 로봇 상태 진단 (상태 토픽 수신 분만)
        with self._tcp_lock:
            robot_state = self._live_robot_state
        STATE_NAMES = {-1:'NOT_RECEIVED', 0:'INITIALIZING', 1:'STANDBY',
                       2:'MOVING', 3:'SAFE_OFF', 4:'TEACHING',
                       5:'SAFE_STOP', 6:'EMERGENCY_STOP', 15:'NOT_READY'}
        state_str = STATE_NAMES.get(robot_state, f'UNKNOWN({robot_state})')
        self.get_logger().info(f'[STATE] robot_state={robot_state} ({state_str})')
        if robot_state not in (1, 2, -1):
            self.get_logger().error(
                f'[STATE] 로봇이 이동 가능 상태가 아닙니다! state={state_str}. '
                f'TeachPendant에서 Auto/Run 모드를 확인하세요.')

        # TCP 유효성 검증
        if robot_tcp is None or all(abs(v) < 1e-6 for v in robot_tcp):
            self.get_logger().error(
                '[TCP] TCP 가 모두 0! GetCurrentPosx 서비스와 /dsr01/state 토픽 모두 실패. '
                'DSR 드라이버 연결 상태를 확인하세요.')
            result.success = False
            result.message = 'TCP is all zeros — GetCurrentPosx service and state topic both failed'
            goal_handle.abort()
            return result

        self.get_logger().info(
            f'[TCP] src={tcp_src}  '
            f'xyz=[{robot_tcp[0]:.1f}, {robot_tcp[1]:.1f}, {robot_tcp[2]:.1f}]mm  '
            f'rpy=[{robot_tcp[3]:.2f}, {robot_tcp[4]:.2f}, {robot_tcp[5]:.2f}]deg')

        # ── 로봇 Autonomous 모드 강제 설정 (v6 동일) ────────────────────
        # MANUAL 모드에서는 move_line이 success=True를 반환해도 로봇이 실제로 안 움직임.
        # 픽 동작 전 반드시 AUTONOMOUS(1) 모드로 전환.
        await self._ensure_autonomous_mode()

        # ── 깊이 기반 3D 위치 추정 ───────────────────────────────────────
        pos_result = self._get_3d_position(list(goal.box))
        if pos_result is None:
            result.success = False
            result.message = 'Depth estimation failed'
            goal_handle.abort()
            return result

        cam_xyz_top, cam_xyz_table = pos_result
        self.get_logger().info(
            f'[CAM XYZ] 물체 꼭대기(mm): '
            f'X={cam_xyz_top[0]*1000:.1f}  Y={cam_xyz_top[1]*1000:.1f}  Z={cam_xyz_top[2]*1000:.1f}')

        # ── 카메라 → 로봇 베이스 좌표 변환 (물체 꼭대기 + 테이블 동시) ──
        try:
            base_xyz       = transform_camera_to_base(cam_xyz_top,   GRIPPER2CAM_PATH, robot_tcp)
            base_xyz_table = transform_camera_to_base(cam_xyz_table, GRIPPER2CAM_PATH, robot_tcp)
        except FileNotFoundError as e:
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result
        except Exception as e:
            self.get_logger().error(f'[TRANSFORM] {e}')
            result.success = False
            result.message = f'transform failed: {e}'
            goal_handle.abort()
            return result

        self.get_logger().info(
            f'[BASE XYZ] 물체(mm): X={base_xyz[0]:.1f}  Y={base_xyz[1]:.1f}  Z={base_xyz[2]:.1f}  |  '
            f'테이블 Z={base_xyz_table[2]:.1f}mm')

        # ── 로봇 Z 공간에서 물체 높이 계산 ──────────────────────────────
        # 카메라 depth 차이를 직접 쓰면 카메라 기울기만큼 오차 발생.
        # 두 점을 각각 로봇 base로 변환한 뒤 Z 차이를 사용해야 정확하다.
        obj_height_robot_z = max(base_xyz[2] - base_xyz_table[2], 10.0)
        self.get_logger().info(
            f'[HEIGHT] 로봇Z 물체높이={obj_height_robot_z:.1f}mm  '
            f'(꼭대기Z={base_xyz[2]:.1f}  테이블Z={base_xyz_table[2]:.1f})')

        # ── 좌표 보정 offset 적용 (constants.py 에서 튜닝) ───────────────
        base_xyz[0] += PICK_OFFSET_X_MM
        base_xyz[1] += PICK_OFFSET_Y_MM
        base_xyz[2] += PICK_OFFSET_Z_MM
        if any(abs(v) > 0.01 for v in [PICK_OFFSET_X_MM, PICK_OFFSET_Y_MM, PICK_OFFSET_Z_MM]):
            self.get_logger().info(
                f'[OFFSET] 보정 적용: '
                f'dX={PICK_OFFSET_X_MM:+.1f}  dY={PICK_OFFSET_Y_MM:+.1f}  dZ={PICK_OFFSET_Z_MM:+.1f}mm  '
                f'→ X={base_xyz[0]:.1f}  Y={base_xyz[1]:.1f}  Z={base_xyz[2]:.1f}')

        orientation = list(robot_tcp[3:])
        pick_target = list(base_xyz) + orientation

        # ── 워크스페이스 Z 안전 범위 검증 ──────────────────────────────
        self.get_logger().info(
            f'[WORKSPACE] pick_target 6D = '
            f'[{pick_target[0]:.1f}, {pick_target[1]:.1f}, {pick_target[2]:.1f}, '
            f'{pick_target[3]:.2f}, {pick_target[4]:.2f}, {pick_target[5]:.2f}]  '
            f'Z허용=[{PICK_SAFE_Z_MIN_MM}, {PICK_SAFE_Z_MAX_MM}]')
        if not (PICK_SAFE_Z_MIN_MM <= pick_target[2] <= PICK_SAFE_Z_MAX_MM):
            result.success = False
            result.message = (f'pick_target Z={pick_target[2]:.1f}mm out of safe range '
                              f'[{PICK_SAFE_Z_MIN_MM}, {PICK_SAFE_Z_MAX_MM}]')
            self.get_logger().error(f'[WORKSPACE] {result.message}')
            goal_handle.abort()
            return result

        # ── 현재 TCP와 pick_target 간 거리 검증 ────────────────────────
        dist_to_target = float(np.linalg.norm(
            np.array(pick_target[:3]) - np.array(robot_tcp[:3])))
        self.get_logger().info(
            f'[DIST] 현재 TCP → pick_target 거리: {dist_to_target:.1f}mm  '
            f'(이 값이 매우 작으면 좌표 계산 오류)')
        if dist_to_target < 5.0:
            self.get_logger().warn(
                f'[DIST] pick_target이 현재 TCP와 거의 동일! ({dist_to_target:.1f}mm) '
                f'T_gripper2camera.npy 캘리브레이션을 확인하세요.')

        # ── 하강 거리 계산 ────────────────────────────────────────────────
        # pick_target[2] = 물체 꼭대기 로봇 Z
        # obj_height_robot_z = 꼭대기 Z - 테이블 Z  (로봇 Z 공간, 카메라 각도 무관)
        #
        # 이상적 파지 위치 = 꼭대기에서 h/2 하강 → 물체 중심
        # 단, 그리퍼 손가락이 테이블에 닿지 않도록 여유(GRIPPER_TABLE_CLEARANCE_MM) 확보:
        #   조건: h - descent_d >= GRIPPER_TABLE_CLEARANCE_MM
        #
        # descent_d = min(h/2, h - clearance)
        #   물체가 충분히 두꺼우면 → h/2 (중심 파지)
        #   물체가 얇으면 → h - clearance (테이블 충돌 방지 우선)
        h          = float(obj_height_robot_z)
        approach_h = float(np.clip(h * 1.0, APPROACH_MIN_MM, APPROACH_MAX_MM))
        descent_d  = min(float(h * 0.5) + PICK_EXTRA_DESCENT_MM, h - GRIPPER_TABLE_CLEARANCE_MM)
        lift_h     = float(np.clip(h * 1.5, LIFT_MIN_MM,     LIFT_MAX_MM))

        self.get_logger().info(
            f'[DESCENT] h={h:.1f}mm  h/2={h*0.5:.1f}mm  '
            f'→ descent_d={descent_d:.1f}mm  '
            f'(테이블여유 예상={h - descent_d:.1f}mm)')

        approach_z       = pick_target[2] + approach_h
        descend_z_target = pick_target[2] - descent_d
        lift_z           = pick_target[2] + lift_h

        self.get_logger().info(
            f'[PICK TARGET] xyz=[{pick_target[0]:.1f}, {pick_target[1]:.1f}, {pick_target[2]:.1f}]mm  '
            f'rpy=[{pick_target[3]:.2f}, {pick_target[4]:.2f}, {pick_target[5]:.2f}]deg  '
            f'obj_h={obj_height_robot_z:.1f}mm')
        self.get_logger().info(
            f'[SEQUENCE Z]  approach_z={approach_z:.1f}  '
            f'top_z={pick_target[2]:.1f}  '
            f'descend_z={descend_z_target:.1f}(파지)  '
            f'lift_z={lift_z:.1f}  '
            f'approach_h={approach_h:.1f}  descent_d={descent_d:.1f}  lift_h={lift_h:.1f}')

        # ── [FIX] 하강 목표 Z 안전 범위 검증 ────────────────────────────
        # pick_target[2] = 물체 꼭대기 높이
        # 실제 하강(파지) 목표 Z = pick_target[2] - descent_d (= 물체 중간) 를 검증
        if descend_z_target < PICK_SAFE_Z_MIN_MM:
            result.success = False
            result.message = (
                f'descend Z={descend_z_target:.1f}mm < PICK_SAFE_Z_MIN_MM({PICK_SAFE_Z_MIN_MM}mm). '
                f'obj_h={obj_height_robot_z:.1f}mm  descent_d={descent_d:.1f}mm  '
                f'pick_target_z={pick_target[2]:.1f}mm')
            self.get_logger().error(f'[WORKSPACE] {result.message}')
            goal_handle.abort()
            return result
        self.get_logger().info(
            f'[WORKSPACE] descend_z={descend_z_target:.1f}mm  OK')

        # ── 시퀀스 실행 ──────────────────────────────────────────────────
        stages = [
            ('APPROACH',     self._stage_approach,     pick_target, approach_h),
            ('OPEN_GRIPPER', self._stage_open_gripper),
            ('SPIN_CHUCK',   self._stage_spin_angle,   pick_target, approach_h),
            ('DESCEND',      self._stage_descend,      pick_target, descent_d),
            ('GRASP',        self._stage_grasp),
            ('LIFT',         self._stage_lift,         pick_target, lift_h),
            ('HOME',         self._stage_home),
        ]

        GRASP_INDEX = 4
        grasped = False

        for i, (stage_name, stage_fn, *fn_args) in enumerate(stages):
            if goal_handle.is_cancel_requested:
                if grasped and self._gripper is not None:
                    self.get_logger().warn('[CANCEL] opening gripper before cancel')
                    self._gripper.open_gripper()
                    self._wait_gripper(5.0)
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                return result

            fb          = PickAndPlace.Feedback()
            fb.stage    = stage_name
            fb.progress = i / len(stages)
            goal_handle.publish_feedback(fb)

            self.get_logger().info(f'[{i+1}/{len(stages)}] {stage_name}')
            try:
                await stage_fn(*fn_args)
                if i == GRASP_INDEX:
                    grasped = True
            except Exception as e:
                self.get_logger().error(f'[{stage_name}] {e}')
                if grasped and self._gripper is not None:
                    self.get_logger().warn(f'[{stage_name}] opening gripper for safety')
                    self._gripper.open_gripper()
                    self._wait_gripper(5.0)
                result.success = False
                result.message = f'{stage_name} failed: {e}'
                goal_handle.abort()
                return result

        result.success = True
        result.message = 'Pick and place completed'
        goal_handle.succeed()
        self.get_logger().info('[DONE] Pick and place succeeded.')
        return result

    # ── 로봇 이동 비동기 헬퍼 ────────────────────────────────────────────
    # DSR sync_type 규약: SYNC=0 (동작 완료 후 응답), ASYNC=1 (즉시 반환)
    # sync_type=0 을 사용해야 이동 완료 후 다음 단계로 진행됨.
    # await call_async(req) 는 rclpy.task.Future 를 yield 하므로
    # 다른 콜백이 executor 를 공유할 수 있음.

    async def _movel_async(self, pos: list, vel: int, acc: int):
        if not self._movel_cli.service_is_ready():
            raise RuntimeError('move_line service not available — /dsr01/motion/move_line 미연결')
        req            = MoveLine.Request()
        req.pos        = [float(v) for v in pos]
        req.vel        = [float(vel), 50.0]
        req.acc        = [float(acc), 100.0]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = 0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0          # SYNC=0: 이동 완료까지 블로킹 (ASYNC=1 아님)
        self.get_logger().info(
            f'[MOVEL REQ] pos=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, '
            f'{pos[3]:.2f}, {pos[4]:.2f}, {pos[5]:.2f}]  '
            f'vel={vel}  acc={acc}  ref=BASE  mode=ABS  sync=SYNC')
        res = await self._movel_cli.call_async(req)
        if res is None:
            raise RuntimeError('movel failed: 서비스 응답 없음 (timeout?)')
        self.get_logger().info(f'[MOVEL RES] success={res.success}')
        if not res.success:
            raise RuntimeError(
                f'movel failed: success=False  '
                f'(로봇이 좌표를 거부함. 워크스페이스 초과 또는 모드 문제)')

    async def _movej_async(self, joints: list, vel: int, acc: int):
        if not self._movej_cli.service_is_ready():
            raise RuntimeError('move_joint service not available — /dsr01/motion/move_joint 미연결')
        req            = MoveJoint.Request()
        req.pos        = [float(v) for v in joints]
        req.vel        = float(vel)
        req.acc        = float(acc)
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0          # SYNC=0: 이동 완료까지 블로킹 (ASYNC=1 아님)
        self.get_logger().info(
            f'[MOVEJ REQ] joints={[f"{v:.1f}" for v in joints]}  '
            f'vel={vel}  acc={acc}  sync=SYNC')
        res = await self._movej_cli.call_async(req)
        if res is None:
            raise RuntimeError('movej failed: 서비스 응답 없음')
        self.get_logger().info(f'[MOVEJ RES] success={res.success}')
        if not res.success:
            raise RuntimeError(
                f'movej failed: success=False  '
                f'(로봇이 조인트 이동 거부. 모드 또는 조인트 한계 문제)')

    # ── 그리퍼 헬퍼 (블로킹 — MultiThreadedExecutor 스레드이므로 허용) ──────

    def _wait_gripper(self, timeout_sec=5.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                if not self._gripper.get_status()[0]:
                    return True
            except Exception:
                return False
            time.sleep(0.15)
        return False

    def _close_with_retry(self, max_retry=3) -> bool:
        for attempt in range(1, max_retry + 1):
            self._wait_gripper(4.0)
            try:
                self._gripper.close_gripper()
            except Exception as e:
                self.get_logger().warn(f'[GRIPPER] close failed: {e}')
                time.sleep(0.5)
                continue
            if self._wait_gripper(6.0):
                try:
                    st = self._gripper.get_status()
                    self.get_logger().info(
                        f'[GRIPPER] attempt {attempt}: '
                        f'{"grip detected" if st[1] else "closed (no grip)"}')
                except Exception:
                    pass
                return True
            self.get_logger().warn(f'[GRIPPER] timeout attempt {attempt}')
        return False

    # ── 픽앤플레이스 단계 ────────────────────────────────────────────────

    async def _stage_approach(self, pt: list, h: float):
        pos = [pt[0], pt[1], pt[2] + h] + pt[3:]
        self.get_logger().info(
            f'  APPROACH → target=['
            f'{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]mm')
        await self._movel_async(pos, PICK_VEL, PICK_ACC)
        self._log_current_tcp('APPROACH 완료')

    async def _stage_open_gripper(self):
        if self._gripper is None:
            return
        self._wait_gripper(3.0)
        self._gripper.open_gripper()
        self._wait_gripper(5.0)
        time.sleep(0.4)

###################################################준수추가코드###################################################
    async def _stage_spin_angle(self, pt: list, z_offset: float = 0.0):
        """
        approach 높이(z_offset)에서 rz를 회전한 뒤, pick_target(pt)의 rz를 in-place 갱신.
        이후 DESCEND / LIFT 등 모든 단계가 갱신된 rz를 그대로 유지한다.
        target_rz = 현재 rz + spin_angle(물체 각도 오프셋) + SPIN_ANGLE_OFFSET(프레임 보정)
        """
        current_rz = pt[5]
        target_rz  = current_rz + self._spin_angle + SPIN_ANGLE_OFFSET
        pt[5]      = target_rz   # pick_target rz를 갱신 → 이후 모든 단계에 반영

        spin_pos = [pt[0], pt[1], pt[2] + z_offset, pt[3], pt[4], target_rz]

        self.get_logger().info(
            f'  [SPIN_CHUCK] current_rz={current_rz:.1f}°  '
            f'offset={self._spin_angle:.1f}°  '
            f'frame_correction={SPIN_ANGLE_OFFSET:.1f}°  '
            f'→ target_rz={target_rz:.1f}°  (z_offset={z_offset:.1f}mm)')
        await self._movel_async(spin_pos, PICK_VEL, PICK_ACC)
        time.sleep(0.2)
###################################################################################################

    async def _stage_descend(self, pt: list, d: float):
        pos = [pt[0], pt[1], pt[2] - d] + pt[3:]
        self.get_logger().info(
            f'  DESCEND  → target=['
            f'{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]mm  (d={d:.1f}mm)')
        vel = max(1, int(PICK_VEL) // 2)
        acc = max(1, int(PICK_ACC) // 2)
        await self._movel_async(pos, vel, acc)
        time.sleep(0.3)
        self._log_current_tcp('DESCEND 완료')

    async def _stage_grasp(self):
        if self._gripper is None:
            return
        self._close_with_retry(3)
        time.sleep(0.3)

    async def _stage_lift(self, pt: list, h: float):
        pos = [pt[0], pt[1], pt[2] + h] + pt[3:]
        self.get_logger().info(
            f'  LIFT     → target=['
            f'{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]mm')
        await self._movel_async(pos, PICK_VEL, PICK_ACC)
        self._log_current_tcp('LIFT 완료')

    async def _stage_home(self):
        self.get_logger().info(f'  HOME     → joints={HOME_JOINT}')
        await self._movej_async(HOME_JOINT, PICK_VEL, PICK_ACC)
        if self._gripper is not None:
            self._gripper.open_gripper()
            self._wait_gripper(5.0)

    def _log_current_tcp(self, label: str):
        with self._tcp_lock:
            tcp = self._live_tcp
        if tcp:
            self.get_logger().info(
                f'  [{label}] 실제 TCP=['
                f'{tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}]mm')

    async def _query_tcp_from_service(self):
        """
        GetCurrentPosx 서비스로 현재 TCP를 직접 조회한다.
        /dsr01/state 토픽이 발행되지 않는 dsr_hw_interface2 환경에서 사용.
        실패 시 None 반환.
        """
        if not self._get_posx_cli.service_is_ready():
            self.get_logger().warn('[TCP] get_current_posx 서비스 미연결')
            return None
        req     = GetCurrentPosx.Request()
        req.ref = 0  # DR_BASE
        res = await self._get_posx_cli.call_async(req)
        if res is None:
            self.get_logger().warn('[TCP] get_current_posx 응답 없음 (timeout?)')
            return None
        if not res.success:
            self.get_logger().warn('[TCP] get_current_posx success=False')
            return None
        if not res.task_pos_info or len(res.task_pos_info[0].data) < 6:
            self.get_logger().warn('[TCP] get_current_posx 응답 데이터 부족')
            return None
        tcp = list(res.task_pos_info[0].data[:6])
        self.get_logger().info(
            f'[TCP] GetCurrentPosx 성공: '
            f'xyz=[{tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f}]mm  '
            f'rpy=[{tcp[3]:.2f}, {tcp[4]:.2f}, {tcp[5]:.2f}]deg')
        return tcp

    async def _ensure_autonomous_mode(self):
        """픽 동작 전 로봇을 AUTONOMOUS 모드로 전환. v6의 set_robot_mode() 대응."""
        if not self._set_mode_cli.service_is_ready():
            self.get_logger().warn(
                '[MODE] set_robot_mode 서비스 미연결 — 모드 설정 건너뜀. '
                '로봇이 MANUAL 모드면 이동하지 않을 수 있음.')
            return
        req = SetRobotMode.Request()
        req.robot_mode = 1  # ROBOT_MODE_AUTONOMOUS
        res = await self._set_mode_cli.call_async(req)
        if res is None:
            self.get_logger().warn('[MODE] set_robot_mode 응답 없음')
        elif res.success:
            self.get_logger().info('[MODE] AUTONOMOUS 모드 설정 완료')
        else:
            self.get_logger().warn(
                '[MODE] AUTONOMOUS 모드 설정 실패 (이미 해당 모드이거나 오류). '
                '계속 진행합니다.')

    # ── 깊이 기반 3D 위치 추정 (v6 알고리즘 이식) ───────────────────────
    #
    # Step1&2: bbox 전체 depth 분포 → 하위20%(카메라에 가까운 점) = 물체 표면,
    #           상위20%(카메라에서 먼 점) = 테이블 표면 (이미지 공간 기준 X)
    # Step3: obj_height = table_mm - obj_top_mm,  descent = min(h/2, h-clearance)
    # Step4: X=(cx-ppx)*d/fx, Y=(cy-ppy)*d/fy, Z=d

    def _get_3d_position(self, box: list, wait_sec=5.0):
        """bbox [x1,y1,x2,y2] → (cam_xyz_top [m], cam_xyz_table [m]) | 실패 시 None"""
        deadline = time.time() + wait_sec
        while time.time() < deadline:
            with self._depth_lock:
                dm = self._depth_frame
            with self._intr_lock:
                intr = self._intrinsics
            if dm is not None and intr is not None:
                break
            time.sleep(0.1)
        else:
            self.get_logger().error('[DEPTH] frame/intrinsics not ready')
            return None

        dh, dw = dm.shape[:2]
        x1 = int(np.clip(box[0], 0, dw - 1))
        y1 = int(np.clip(box[1], 0, dh - 1))
        x2 = int(np.clip(box[2], 0, dw - 1))
        y2 = int(np.clip(box[3], 0, dh - 1))
        cx = int(np.clip((x1 + x2) // 2, 0, dw - 1))
        cy = int(np.clip((y1 + y2) // 2, 0, dh - 1))
        fx, fy   = intr['fx'], intr['fy']
        ppx, ppy = intr['ppx'], intr['ppy']

        self.get_logger().info(
            f'[DEPTH] box=[{x1},{y1},{x2},{y2}]  cx={cx} cy={cy}  '
            f'fx={fx:.1f} fy={fy:.1f}')

        # ══════════════════════════════════════════════════════
        # Step 1 & 2: 물체 표면·테이블 depth 추정
        #   [물체 표면] bbox 내 하위 20% median (depth 최소 = 카메라에 가장 가까운 점)
        #   [테이블]    우선순위:
        #     1) bbox 내 상위 20% median  (bbox 안에 테이블 픽셀 있을 때)
        #     2) 프레임 전체(bbox 제외) 75th percentile  ← bbox 부족 시 안정적 대안
        #     3) EMA 전체 프레임 이동평균
        #     4) 픽셀비례 추정 (최후 수단)
        # ══════════════════════════════════════════════════════
        # ── bbox 내부 유효 픽셀 ──
        full_roi   = dm[y1:y2, x1:x2]
        full_valid = full_roi[
            (full_roi > PICK_MIN_DEPTH_MM) & (full_roi < PICK_MAX_DEPTH_MM)
        ].ravel()

        # ── bbox 외부 유효 픽셀 (테이블 추정 2순위) ──
        outside_mask = np.ones(dm.shape, dtype=bool)
        outside_mask[y1:y2, x1:x2] = False
        outside_raw   = dm[outside_mask]
        outside_valid = outside_raw[
            (outside_raw > PICK_MIN_DEPTH_MM) & (outside_raw < PICK_MAX_DEPTH_MM)
        ]

        with self._table_lock:
            ema_table = self._table_depth_mm

        # ── obj_top_mm 계산 ──
        if len(full_valid) < 10:
            self.get_logger().warn('[DEPTH] bbox 내 유효 픽셀 부족 → 중심 fallback')
            fb = dm[
                max(0, cy - DEPTH_SAMPLE_MARGIN): min(dh, cy + DEPTH_SAMPLE_MARGIN + 1),
                max(0, cx - DEPTH_SAMPLE_MARGIN): min(dw, cx + DEPTH_SAMPLE_MARGIN + 1),
            ]
            fb_valid = fb[(fb > PICK_MIN_DEPTH_MM) & (fb < PICK_MAX_DEPTH_MM)]
            if len(fb_valid) == 0:
                self.get_logger().error('[DEPTH] 유효한 depth 없음')
                return None
            obj_top_mm    = float(np.median(fb_valid))
            bbox_table_mm = None
        else:
            p20 = float(np.percentile(full_valid, 20))
            p80 = float(np.percentile(full_valid, 80))
            obj_top_mm    = float(np.median(full_valid[full_valid <= p20]))
            bbox_table_mm = float(np.median(full_valid[full_valid >= p80]))

            # 하위 20% 픽셀들의 실제 위치 중심으로 cx, cy 갱신
            top_mask = (full_roi > PICK_MIN_DEPTH_MM) & (full_roi <= p20)
            ys, xs = np.where(top_mask)
            if len(xs) > 0:
                cx = int(np.clip(int(np.mean(xs)) + x1, 0, dw - 1))
                cy = int(np.clip(int(np.mean(ys)) + y1, 0, dh - 1))

            self.get_logger().info(
                f'[DEPTH] bbox 하위20%→물체={obj_top_mm:.1f}mm  '
                f'상위20%→테이블후보={bbox_table_mm:.1f}mm  '
                f'꼭대기중심=({cx},{cy})')

        # ── table_mm 결정 (우선순위) ──
        table_mm = None

        # 1순위: bbox 내 상위 20%
        if bbox_table_mm is not None and bbox_table_mm > obj_top_mm + 5.0:
            table_mm = bbox_table_mm
            self.get_logger().info(f'[DEPTH] 테이블(bbox 상위20%): {table_mm:.1f}mm')

        # 2순위: 프레임 전체(bbox 제외) 75th percentile
        if table_mm is None and len(outside_valid) > 200:
            frame_p75 = float(np.percentile(outside_valid, 75))
            if frame_p75 > obj_top_mm + 5.0:
                table_mm = frame_p75
                self.get_logger().info(
                    f'[DEPTH] 테이블(프레임 bbox제외 p75): {table_mm:.1f}mm  '
                    f'({len(outside_valid)}px)')

        # 3순위: EMA
        if table_mm is None and ema_table is not None and ema_table > obj_top_mm + 5.0:
            table_mm = ema_table
            self.get_logger().info(f'[DEPTH] 테이블(EMA): {table_mm:.1f}mm')

        # 4순위: 고정 기본값 (최후 수단)
        if table_mm is None:
            height_est = 30.0  # mm
            table_mm   = obj_top_mm + height_est
            self.get_logger().warn(f'[DEPTH] 테이블 추정 불가 → 고정 기본값 {height_est:.0f}mm 사용: {table_mm:.1f}mm')

        # ══════════════════════════════════════════════════════
        # Step 3: 진단 로그 (카메라 depth 공간 기준 참고값)
        #   실제 물체 높이(로봇 Z 공간)는 execute에서 두 점의 변환 후 차이로 계산
        # ══════════════════════════════════════════════════════
        cam_depth_h = table_mm - obj_top_mm
        self.get_logger().info(
            f'[DEPTH] 꼭대기={obj_top_mm:.1f}mm  테이블={table_mm:.1f}mm  '
            f'카메라depth차={cam_depth_h:.1f}mm  (실제 로봇Z 높이는 변환 후 계산)')

        # ══════════════════════════════════════════════════════
        # Step 4: 카메라 3D 좌표 변환
        #   물체 꼭대기와 테이블을 각각 카메라 좌표로 만들어 반환.
        #   로봇 Z 공간의 높이 차이는 execute에서 두 점 변환 후 계산한다.
        #   (카메라 기울기 때문에 depth 차이 ≠ 로봇 Z 차이)
        # ══════════════════════════════════════════════════════
        depth_m       = obj_top_mm * 0.001
        X = (cx - ppx) * depth_m / fx
        Y = (cy - ppy) * depth_m / fy
        Z = depth_m + PICK_DEPTH_OFFSET * 0.001

        table_m       = table_mm * 0.001
        X_t = (cx - ppx) * table_m / fx
        Y_t = (cy - ppy) * table_m / fy
        Z_t = table_m

        self.get_logger().info(
            f'[DEPTH] 물체(cam): X={X*1000:.1f}  Y={Y*1000:.1f}  Z={Z*1000:.1f}mm  |  '
            f'테이블(cam): Z={Z_t*1000:.1f}mm')
        return [X, Y, Z], [X_t, Y_t, Z_t]
    
########################################################################################### 실행시 홈으로 가기 로직 추가
    def _initial_home_move(self):
        if self._home_timer:
            self._home_timer.cancel()

        self.get_logger().info("🏠 [INIT] 노드 시작: 홈 위치로 이동을 시도합니다.")

        # 별도 스레드에서 실행하되 spin_until_future_complete를 제거합니다.
        def run_home():
            try:
                if not self._movej_cli.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn('[INIT] 서비스 미연결')
                    return

                req = MoveJoint.Request()
                req.pos = [float(v) for v in HOME_JOINT]
                req.vel, req.acc = float(PICK_VEL), float(PICK_ACC)
                req.sync_type = 0 # 로봇 드라이버 단에서 블로킹 대기

                # .call()을 사용하여 이 스레드만 블로킹되게 합니다 (spin_until_... 대신)
                # MultiThreadedExecutor이므로 다른 스레드에서 액션 콜백은 계속 돕니다.
                res = self._movej_cli.call(req)
                
                if res and res.success:
                    self.get_logger().info('🏠 [INIT] 홈 위치 이동 완료')
                else:
                    self.get_logger().warn('🏠 [INIT] 홈 위치 이동 실패')
            except Exception as e:
                self.get_logger().error(f'🏠 [INIT] 예외: {e}')

        threading.Thread(target=run_home, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
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