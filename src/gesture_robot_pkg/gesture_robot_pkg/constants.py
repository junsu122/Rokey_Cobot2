"""
constants.py
────────────
gesture_robot_control_v6 에서 추출한 공유 상수.
모든 노드가 이 모듈을 임포트하여 동일한 값을 사용한다.
"""

from ament_index_python.packages import get_package_share_directory
import os

_PKG_SHARE = get_package_share_directory('gesture_robot_pkg')

YOLO_MODEL_PATH  = os.path.join(_PKG_SHARE, 'data', '5.8final.pt')
GRIPPER2CAM_PATH = os.path.join(_PKG_SHARE, 'data', 'T_gripper2camera.npy')


# ── 로봇 ─────────────────────────────────────────────────────────────
ROBOT_ID    = 'dsr01'
ROBOT_MODEL = 'm0609'
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

# ── 카메라 ────────────────────────────────────────────────────────────
WEBCAM_INDEX          = 6
REALSENSE_COLOR_TOPIC = '/camera/camera/color/image_raw'
REALSENSE_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
REALSENSE_INFO_TOPIC  = '/camera/camera/color/camera_info'

# ── YOLO ──────────────────────────────────────────────────────────────
YOLO_CONF_THRESHOLD  = 0.5

# ── 그리퍼 ────────────────────────────────────────────────────────────
GRIPPER_NAME     = "rg2"
TOOLCHANGER_IP   = "192.168.1.1"
TOOLCHANGER_PORT = "502"

# ── Pick & Place 기본값 ───────────────────────────────────────────────
PICK_VEL            = 200
PICK_ACC            = 200
HOME_JOINT          = [0, 0, 90, 0, 90, 0]
GIVE_JOINT          = [-47.52, -16.55, 101.54, 1.37, 94.23, -47.05]
GIVE_LINE           = [176.72,-178.17,400.26,12.77,178.44,13.00]
WAY_POINT_JOINT     = [-8.24, 1.14, 88.66, 1.92, 88.11, -6.78]
PICK_MIN_DEPTH_MM   = 105.0    # v6 기준값 (D435 최소 유효 거리 고려 시 105 이상 권장)
PICK_MAX_DEPTH_MM   = 1500.0
DEPTH_SAMPLE_MARGIN = 8

# ── 좌표 보정 offset (로봇 베이스 프레임, mm) ─────────────────────────
# 캘리브레이션 잔여 오차를 이 값으로 보정한다.
# 로봇이 물체 위에서 X방향으로 밀려있으면 PICK_OFFSET_X_MM 을 반대 방향으로 조정.
# 로봇이 물체 위에서 Y방향으로 밀려있으면 PICK_OFFSET_Y_MM 을 반대 방향으로 조정.
# 그리퍼가 물체를 내려갈 때 높이가 맞지 않으면 PICK_OFFSET_Z_MM 을 조정.
PICK_OFFSET_X_MM    = 0.0   # + : 로봇 베이스 X+ 방향으로 이동
PICK_OFFSET_Y_MM    = 0.0   # + : 로봇 베이스 Y+ 방향으로 이동
PICK_OFFSET_Z_MM    = 0.0   # + : 위로 이동 (픽 위치 높아짐)

# ── 픽 타깃 워크스페이스 안전 범위 ──────────────────────────────────────
# v6 에는 이 체크 없음. 넓게 설정해 잘못된 abort 방지.
PICK_SAFE_Z_MIN_MM  = -500.0
PICK_SAFE_Z_MAX_MM  = 1500.0

# ── 높이 비례 계산 범위 ────────────────────────────────────────────────
APPROACH_MIN_MM = 40.0
APPROACH_MAX_MM = 150.0
LIFT_MIN_MM     = 80.0
LIFT_MAX_MM     = 200.0

# ── 그리퍼 테이블 여유 거리 ──────────────────────────────────────────────
# 그리퍼가 물체를 파지할 때 손가락 끝이 테이블에 닿지 않도록 확보할 최소 높이(mm).
# TCP 하강 목표: min(높이차/2, 높이차 - GRIPPER_TABLE_CLEARANCE_MM)
# → 물체가 얇을수록 절반보다 조금 높은 위치에서 멈춰 테이블 충돌 방지.
# 그리퍼 손가락이 테이블에 닿는 경우 이 값을 늘려서 튜닝.
GRIPPER_TABLE_CLEARANCE_MM = 8.0
PICK_EXTRA_DESCENT_MM      = 45.0  # 기본 하강량에 추가로 더 내려가는 거리(mm)
SPIN_ANGLE_OFFSET          = 0.0   # 센서 프레임↔로봇 베이스 프레임 yaw 오차 보정값(deg), 실측 후 튜닝

# ── 물체 방향 보정 ────────────────────────────────────────────────────
ASPECT_THRESHOLD = 1.2

# ── 제어 모드 ──────────────────────────────────────────────────────────
POSITION_CONTROL = True

# ── 제어 공통 ──────────────────────────────────────────────────────────
DEAD_ZONE        = 0.12
CTRL_DEAD        = 0.08
DEPTH_DEAD       = 0.015
SMOOTHING_FRAMES = 3
CALIB_SEC        = 2.5

REPEAT_HZ        = 20.0
DEPTH_MAX_RANGE  = 0.10

RANGE_X_MM = 250.0
RANGE_Z_MM = 250.0
RANGE_Y_MM = 250.0

LIMIT_X_MM = 280.0
LIMIT_Z_MM = 280.0
LIMIT_Y_MM = 280.0

ABS_VEL      = 400.0
ABS_ACC      = 500.0
ABS_BLEND_R  =   50.00
POS_THR_MM   =   1.5
ABS_MAX_STEP =  50.0

MIN_STEP_MM    = 0.5
MAX_STEP_MM    = 80.0
SMOOTH_ALPHA   = 0.6
CMD_CHANGE_THR = 0.3
CTRL_MAX_RANGE = 0.5 - CTRL_DEAD

MOVE_VEL = 300.0
MOVE_ACC = 400.0

# ── 상태 상수 ──────────────────────────────────────────────────────────
STATE_WAITING     = 'WAITING'
STATE_CALIBRATING = 'CALIBRATING'
STATE_CONTROLLING = 'CONTROLLING'
STATE_PAUSED      = 'PAUSED'
STATE_PICKING     = 'PICKING'

COLORS = {
    STATE_WAITING:     (120, 120, 120),
    STATE_CALIBRATING: (80,  220, 255),
    STATE_CONTROLLING: (80,  220,  80),
    STATE_PAUSED:      (0,   60,  220),
    STATE_PICKING:     (0,  165,  255),
    'NONE':            (60,   60,  60),
}

# ── 제스처 임계값 ──────────────────────────────────────────────────────
FIST_FINGER_COUNT = 3
HOVER_SEC         = 2.0

# ── 손 랜드마크 연결 ────────────────────────────────────────────────────
HAND_CONNECTIONS = [
    (0, 1),  (1, 2),  (2, 3),  (3, 4),
    (0, 5),  (5, 6),  (6, 7),  (7, 8),
    (5, 9),  (9,10), (10,11), (11,12),
    (9,13), (13,14), (14,15), (15,16),
    (13,17),(17,18), (18,19), (19,20), (0,17),
]
