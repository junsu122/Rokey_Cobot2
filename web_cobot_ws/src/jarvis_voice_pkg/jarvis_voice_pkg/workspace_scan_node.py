#!/usr/bin/env python3
"""
workspace_scan_node.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
JARVIS 작업공간 스캔 노드 (jarvis_voice_pkg 통합 버전)

역할:
  1. publisher.py 의 /scan_request 수신 → 스캔 시작 지시
  2. 지그재그 경로로 로봇 이동 (scan.py 동일 경로)
  3. 각 스캔 포즈에서 카메라 이미지를 캡처하여 GPT-4o Vision(VLM)으로 물체 탐지
  4. 타겟 물체 발견 여부 확인 후 /scan_result 발행
  5. publisher.py 가 /scan_request 로 취소 요청 시 즉시 중단

구독 토픽:
  /scan_request                          (std_msgs/String) : publisher.py → 스캔 시작/취소 명령
  /camera/camera/color/image_raw         (sensor_msgs/Image) : RealSense 컬러 이미지

발행 토픽:
  /scan_result        (std_msgs/String) : publisher.py → 스캔 결과 (found/not_found)
  /scan_status        (std_msgs/String) : 스캔 진행 상태 실시간 발행

실행:
  ros2 run jarvis_voice_pkg workspace_scan_node
  ros2 run jarvis_voice_pkg workspace_scan_node --ros-args -p nx:=4 -p ny:=3
"""

import base64
import json
import math
import os
import re
import time
import threading
from datetime import datetime

import cv2
import numpy as np
import openai
import rclpy
import DR_init
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String

# ── 로봇 식별자 ────────────────────────────────────────────────────────────────
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

# ── 카메라 / VLM ───────────────────────────────────────────────────────────────
REALSENSE_COLOR_TOPIC = '/camera/camera/color/image_raw'
VLM_MODEL             = 'gpt-4o'
VLM_DETECTABLE_OBJECTS = [
    'umbrella', 'bag', 'apple', 'banana', 'pill',
    'phone', 'juice', 'sun_cream', 'water', 'candy', 'mask', 'bread',
]

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ── 스캔 작업공간 정의 (scan.py 동일) ──────────────────────────────────────────
LEFT_TOP_POSE     = [237.47,81.55,188.77,179.67,-179.69,179.63]
RIGHT_BOTTOM_POSE = [513.16,-262.44,187.29,148.58,-179.14,148.61]

X_MIN  = LEFT_TOP_POSE[0]
X_MAX  = RIGHT_BOTTOM_POSE[0]
Y_MAX  = LEFT_TOP_POSE[1]
Y_MIN  = RIGHT_BOTTOM_POSE[1]
Z_SCAN = (LEFT_TOP_POSE[2] + RIGHT_BOTTOM_POSE[2]) / 2  # 188.03mm

RX = LEFT_TOP_POSE[3]
RY = LEFT_TOP_POSE[4]
RZ = LEFT_TOP_POSE[5]

# ── 로봇 상태 ──────────────────────────────────────────────────────────────────
ROBOT_MODE_AUTONOMOUS = 1
UNSAFE_STATES         = {3, 5, 6}

# ── 기본 파라미터 ───────────────────────────────────────────────────────────────
DEFAULT_NX                      = 2
DEFAULT_NY                      = 2
DEFAULT_VEL                     = 60.0
DEFAULT_ACC                     = 60.0
DEFAULT_CAPTURE_WAIT_SEC        = 0.8
DEFAULT_DETECTION_WINDOW_SEC    = 1.2
DEFAULT_LOG_FILE_PATH           = "./scan_log/jarvis_scan_result.log"
DEFAULT_POSE_DISTANCE_THRESHOLD = 260.0
DEFAULT_IMG_DISTANCE_THRESHOLD  = 350.0
DEFAULT_MIN_CONFIDENCE          = 0.45


def get_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def make_xy_scan_points(nx: int, ny: int) -> list:
    """
    지그재그 TCP 포즈 목록 생성.
    짝수 행: 좌→우, 홀수 행: 우→좌 순서.
    """
    nx = max(1, int(nx))
    ny = max(1, int(ny))
    points = []
    for row in range(ny):
        y = (Y_MAX + Y_MIN) / 2.0 if ny == 1 else (
            Y_MAX - (Y_MAX - Y_MIN) * row / (ny - 1))

        row_pts = []
        for col in range(nx):
            x = (X_MIN + X_MAX) / 2.0 if nx == 1 else (
                X_MIN + (X_MAX - X_MIN) * col / (nx - 1))
            row_pts.append([
                round(x, 2), round(y, 2), round(Z_SCAN, 2),
                round(RX, 2), round(RY, 2), round(RZ, 2),
            ])

        if row % 2 == 1:
            row_pts.reverse()
        points.extend(row_pts)
    return points


# ══════════════════════════════════════════════════════════════════════════════
# WorkspaceScanNode
# ══════════════════════════════════════════════════════════════════════════════

class WorkspaceScanNode(Node):
    """
    /scan_request 를 받아 로봇을 지그재그로 이동시키고
    각 포즈에서 RealSense 이미지를 GPT-4o Vision(VLM)으로 분석해
    /scan_result 로 반환하는 노드.
    """

    def __init__(self):
        super().__init__("jarvis_workspace_scan_node", namespace=ROBOT_ID)

        # ── 파라미터 선언 ──────────────────────────────────────────────────
        self.declare_parameter("nx",                        DEFAULT_NX)
        self.declare_parameter("ny",                        DEFAULT_NY)
        self.declare_parameter("vel",                       DEFAULT_VEL)
        self.declare_parameter("acc",                       DEFAULT_ACC)
        self.declare_parameter("capture_wait_sec",          DEFAULT_CAPTURE_WAIT_SEC)
        self.declare_parameter("centering_wait_sec",        1.5)   # 센터링 후 안정화 대기(초) — 포인트클라우드 갱신 여유
        self.declare_parameter("detection_window_sec",      DEFAULT_DETECTION_WINDOW_SEC)
        self.declare_parameter("log_file_path",             DEFAULT_LOG_FILE_PATH)
        self.declare_parameter("pose_distance_threshold",   DEFAULT_POSE_DISTANCE_THRESHOLD)
        self.declare_parameter("image_distance_threshold",  DEFAULT_IMG_DISTANCE_THRESHOLD)
        self.declare_parameter("min_confidence",            DEFAULT_MIN_CONFIDENCE)
        self.declare_parameter("centering_depth_mm",        300.0)

        self.nx                       = int(self.get_parameter("nx").value)
        self.ny                       = int(self.get_parameter("ny").value)
        self.vel                      = float(self.get_parameter("vel").value)
        self.acc                      = float(self.get_parameter("acc").value)
        self.capture_wait_sec         = float(self.get_parameter("capture_wait_sec").value)
        self.centering_wait_sec       = float(self.get_parameter("centering_wait_sec").value)
        self.detection_window_sec     = float(self.get_parameter("detection_window_sec").value)
        self.log_file_path            = str(self.get_parameter("log_file_path").value)
        self.pose_distance_threshold  = float(self.get_parameter("pose_distance_threshold").value)
        self.image_distance_threshold = float(self.get_parameter("image_distance_threshold").value)
        self.min_confidence           = float(self.get_parameter("min_confidence").value)
        self.centering_depth_mm       = float(self.get_parameter("centering_depth_mm").value)

        self.scan_points = make_xy_scan_points(self.nx, self.ny)

        # ── 내부 상태 ──────────────────────────────────────────────────────
        self.scan_running     : bool      = False
        self.scan_requested   : bool      = False
        self.cancel_requested : bool      = False
        self.target_objects   : list[str] = []
        self.scan_records     : list      = []
        self.summary_written  : bool      = False
        self.robot_api        : dict      = {}

        # ── 픽 완료 감지 (/is_picking 구독) ───────────────────────────────
        self._is_picking_ext       : bool             = False
        self._pick_complete_event  : threading.Event  = threading.Event()

        # ── 카메라 / VLM ──────────────────────────────────────────────────
        self._latest_frame    : object      = None
        self._frame_lock      = threading.Lock()
        self._bridge          = CvBridge()
        self._vlm_client      = openai.OpenAI(
            api_key=os.getenv("OPENAI_API_KEY", ""))
        self._cam_intrinsics  = None
        self._cam_intr_lock   = threading.Lock()

        # ── T_gripper2camera 로드 (센터링용) ──────────────────────────────
        try:
            _pkg_share = get_package_share_directory('gesture_robot_pkg')
            _gripper2cam_path = os.path.join(_pkg_share, 'data', 'T_gripper2camera.npy')
            self._T_gripper2cam = np.load(_gripper2cam_path)
            self.get_logger().info(f'T_gripper2camera 로드 완료: {_gripper2cam_path}')
        except Exception as _e:
            self.get_logger().warn(f'T_gripper2camera.npy 로드 실패: {_e} — 센터링 비활성화')
            self._T_gripper2cam = None

        # ── 콜백 그룹 ──────────────────────────────────────────────────────
        self._sub_cb_group = ReentrantCallbackGroup()

        # ── 구독 ───────────────────────────────────────────────────────────
        self.create_subscription(
            String, "/scan_request",
            self._on_scan_request, 10,
            callback_group=self._sub_cb_group)

        self.create_subscription(
            Image, REALSENSE_COLOR_TOPIC,
            self._on_camera_image, 1,
            callback_group=self._sub_cb_group)

        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._on_camera_info, 10,
            callback_group=self._sub_cb_group)

        self.create_subscription(
            Bool, '/is_picking',
            self._on_is_picking, 10,
            callback_group=self._sub_cb_group)

        # ── 발행 ───────────────────────────────────────────────────────────
        self._result_pub          = self.create_publisher(String, "/scan_result",   10)
        self._status_pub          = self.create_publisher(String, "/scan_status",   10)
        self._voice_intent_pub    = self.create_publisher(String, "/voice_intent",  10)

        self.get_logger().info(
            f"✅ WorkspaceScanNode 초기화 완료 "
            f"[nx={self.nx}, ny={self.ny}, "
            f"포즈 수={len(self.scan_points)}, "
            f"탐지 윈도우={self.detection_window_sec}s]")

    # ── 로봇 API 주입 ────────────────────────────────────────────────────────

    def configure_robot_api(
        self,
        set_robot_mode=None,
        set_tool=None,
        set_tcp=None,
        get_robot_state=None,
        movel=None,
        wait=None,
        move_mod_abs=None,
    ):
        self.robot_api = {
            "set_robot_mode" : set_robot_mode,
            "set_tool"       : set_tool,
            "set_tcp"        : set_tcp,
            "get_robot_state": get_robot_state,
            "movel"          : movel,
            "wait"           : wait,
            "move_mod_abs"   : move_mod_abs,
        }

    # ── 구독 콜백 ────────────────────────────────────────────────────────────

    def _on_scan_request(self, msg: String):
        """
        publisher.py → /scan_request 수신
        action: "start"  → 스캔 시작
        action: "cancel" → 스캔 취소
        """
        try:
            data   = json.loads(msg.data)
            action = data.get("action", "start")
        except Exception:
            data   = {}
            action = "start"

        if action == "cancel":
            if self.scan_running:
                self.cancel_requested = True
                self.get_logger().warn(
                    f"🛑 스캔 취소 요청: reason={data.get('reason', 'unknown')}")
                self._publish_status("cancelling", {})
            return

        if self.scan_running:
            self.get_logger().warn("스캔 요청 무시: 이미 스캔 중")
            return

        self.target_objects   = data.get("target_objects", [])
        self.scan_requested   = True
        self.cancel_requested = False

        self.get_logger().info(
            f"📥 스캔 요청 수신: 타겟={self.target_objects or '전체'}")

    def _on_camera_image(self, msg: Image):
        """RealSense 컬러 이미지 수신 → 최신 프레임 갱신"""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as exc:
            self.get_logger().warn(f"카메라 프레임 수신 오류: {exc}")

    def _on_is_picking(self, msg: Bool):
        """is_picking False→True→False 전환 감지 → _pick_complete_event 발생"""
        prev = self._is_picking_ext
        self._is_picking_ext = msg.data
        if prev and not msg.data:
            self._pick_complete_event.set()

    def _wait_for_pick_complete(self, timeout: float = 120.0) -> bool:
        """
        픽이 시작(is_picking=True)된 후 완료(is_picking=False)될 때까지 대기.
        반환: 정상 완료=True, 타임아웃/취소=False
        """
        # 이벤트를 먼저 clear → 픽 완료 신호를 놓치지 않음 (race condition 방지)
        self._pick_complete_event.clear()

        # 픽 시작 대기 (최대 30초)
        start_deadline = time.time() + 30.0
        while not self._is_picking_ext and time.time() < start_deadline:
            if self.cancel_requested:
                return False
            time.sleep(0.2)
        if not self._is_picking_ext:
            self.get_logger().warn('[WAIT_PICK] 픽 시작 안 됨 (30s 타임아웃)')
            return False

        # 픽 완료 대기
        completed = self._pick_complete_event.wait(timeout=timeout)
        if not completed:
            self.get_logger().warn(f'[WAIT_PICK] 픽 완료 타임아웃 ({timeout:.0f}s)')
        return completed

    def _on_camera_info(self, msg: CameraInfo):
        """카메라 인트린식 수신 → 저장 (센터링 계산용)"""
        with self._cam_intr_lock:
            self._cam_intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'ppx': msg.k[2], 'ppy': msg.k[5],
            }

    def _compute_centering_pose(self, best_pose: list, bbox: list) -> list | None:
        """
        VLM이 찾은 bbox 중심을 카메라 이미지 중앙으로 이동시키는 로봇 TCP 포즈 계산.

        best_pose : 스캔 당시 로봇 TCP [X, Y, Z, RX, RY, RZ] (mm, deg)
        bbox      : VLM bbox [x1, y1, x2, y2] (pixels)
        반환      : 센터링된 TCP 포즈 (X, Y 만 변경) | 계산 불가 시 None
        """
        if self._T_gripper2cam is None:
            return None
        with self._cam_intr_lock:
            intr = self._cam_intrinsics
        if intr is None:
            self.get_logger().warn('[CENTER] 카메라 인트린식 미수신 — 센터링 스킵')
            return None
        if len(bbox) < 4:
            return None

        x1, y1, x2, y2 = bbox[:4]
        bx = (x1 + x2) / 2.0
        by = (y1 + y2) / 2.0
        fx, fy   = intr['fx'], intr['fy']
        ppx, ppy = intr['ppx'], intr['ppy']

        # 픽셀 오프셋: 양수 = 물체가 이미지 주점(principal point)의 오른쪽/아래쪽
        dpx = bx - ppx
        dpy = by - ppy

        # 카메라 프레임에서의 이동량 (mm)
        # 물체가 오른쪽(dpx>0)이면 카메라도 오른쪽으로 이동 → 물체가 중앙으로 이동
        # ΔX_cam = dpx * Z / fx  (부호 양수)
        Z_est = self.centering_depth_mm
        delta_cam = np.array([dpx * Z_est / fx,
                               dpy * Z_est / fy,
                               0.0])

        # TCP 포즈 → 그리퍼→베이스 변환 행렬 T_{b←g}
        x, y, z, rx, ry, rz = best_pose
        R_bg = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
        T_bg = np.eye(4)
        T_bg[:3, :3] = R_bg
        T_bg[:3, 3]  = [x, y, z]

        # T_{b←c} = T_{b←g} @ T_{g←c}  (T_gripper2cam = T_{g←c}: 카메라→그리퍼)
        T_bc    = T_bg @ self._T_gripper2cam
        R_bc    = T_bc[:3, :3]

        # 카메라 프레임 델타 → 로봇 베이스 프레임으로 회전
        delta_base = R_bc @ delta_cam  # mm

        new_pose    = list(best_pose)
        new_pose[0] = float(np.clip(best_pose[0] + delta_base[0], X_MIN, X_MAX))
        new_pose[1] = float(np.clip(best_pose[1] + delta_base[1], Y_MIN, Y_MAX))
        # Z / RX / RY / RZ 유지

        self.get_logger().info(
            f'[CENTER] δ픽셀=({dpx:.0f},{dpy:.0f})px  '
            f'δ카메라=({delta_cam[0]:.1f},{delta_cam[1]:.1f})mm  '
            f'δ베이스=({delta_base[0]:.1f},{delta_base[1]:.1f})mm  '
            f'Z_est={Z_est:.0f}mm')
        return new_pose

    # ── 탐지 수집 ────────────────────────────────────────────────────────────

    def _collect_detections_at_pose(self) -> list:
        """
        현재 포즈에서 카메라 이미지를 캡처하고 VLM으로 물체 탐지.
        capture_wait_sec 만큼 카메라 안정화 대기 후 GPT-4o Vision 1회 호출.
        """
        if self.cancel_requested:
            return []

        # 카메라 안정화 대기 (movel 완료 직후 진동 감소)
        settle_deadline = time.time() + self.capture_wait_sec
        while time.time() < settle_deadline:
            if self.cancel_requested:
                return []
            time.sleep(0.05)

        with self._frame_lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None

        if frame is None:
            self.get_logger().warn("카메라 프레임 없음 — VLM 탐지 스킵")
            return []

        detections = self._analyze_frame_with_vlm(frame)

        best_by_name: dict = {}
        for det in detections:
            name = det.get("name", "")
            conf = det.get("conf", 0.0)
            if self.target_objects and name not in self.target_objects:
                continue
            if conf < self.min_confidence:
                continue
            if name not in best_by_name or conf > best_by_name[name]["conf"]:
                best_by_name[name] = {
                    "name"     : name,
                    "conf"     : round(conf, 3),
                    "box"      : det.get("box", []),
                    "center_xy": self._box_center(det.get("box", [])),
                }

        return list(best_by_name.values())

    def _analyze_frame_with_vlm(self, frame) -> list:
        """
        GPT-4o Vision으로 프레임 내 물체 탐지.
        반환: [{"name": str, "conf": float, "box": [x1,y1,x2,y2]}, ...]
        """
        h, w = frame.shape[:2]
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        b64    = base64.b64encode(buf.tobytes()).decode()

        # 타겟이 지정된 경우 해당 물체 중심으로 프롬프트 집중
        if self.target_objects:
            target_str  = ', '.join(self.target_objects)
            all_str     = ', '.join(VLM_DETECTABLE_OBJECTS)
            prompt = (
                f"이미지는 로봇 그리퍼에 장착된 RealSense 카메라로 촬영한 작업 테이블 영상입니다.\n"
                f"이미지 해상도: {w}x{h} 픽셀\n\n"
                f"【중요】 반드시 찾아야 할 물체: {target_str}\n"
                f"테이블 위를 꼼꼼히 살펴 위 물체가 조금이라도 보이면 반드시 포함하세요.\n"
                f"부분적으로 보이거나 가려진 경우도 포함하세요.\n\n"
                f"추가로 테이블 위에 보이는 다른 물체도 함께 보고하세요.\n"
                f"감지 가능 물체 목록(영문 키 사용): {all_str}\n\n"
                f"각 물체의 바운딩박스를 픽셀 좌표(x1, y1, x2, y2)로 표시하고,\n"
                f"해당 물체가 맞을 확률(confidence 0.0~1.0)을 추정하세요.\n\n"
                f"JSON 형식으로만 답변하세요. 응답 형식:\n"
                f'{{"objects": [{{"name": "영문물체이름", "confidence": 0.95, "bbox": [x1, y1, x2, y2]}}]}}\n'
                f"물체가 없으면: {{\"objects\": []}}"
            )
        else:
            objects_str = ', '.join(VLM_DETECTABLE_OBJECTS)
            prompt = (
                f"이미지는 로봇 그리퍼에 장착된 RealSense 카메라로 촬영한 작업 테이블 영상입니다.\n"
                f"이미지 해상도: {w}x{h} 픽셀\n\n"
                f"테이블 위에 있는 물건들을 찾아 JSON으로만 답변하세요.\n"
                f"감지 대상 물체(영문 키 사용): {objects_str}\n\n"
                f"각 물체의 바운딩박스를 픽셀 좌표(x1, y1, x2, y2)로 표시하고,\n"
                f"해당 물체가 그 물체가 맞을 확률(confidence 0.0~1.0)을 추정하세요.\n\n"
                f"응답 형식:\n"
                f'{{"objects": [{{"name": "영문물체이름", "confidence": 0.95, "bbox": [x1, y1, x2, y2]}}]}}\n'
                f"물체가 없으면: {{\"objects\": []}}"
            )

        try:
            resp = self._vlm_client.chat.completions.create(
                model=VLM_MODEL,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "image_url",
                         "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                        {"type": "text", "text": prompt},
                    ],
                }],
                max_tokens=512,
                response_format={"type": "json_object"},
            )

            data = json.loads(resp.choices[0].message.content)
            detections = []
            for obj in data.get("objects", []):
                name = str(obj.get("name", "")).strip()
                conf = float(obj.get("confidence", 0.8))
                bbox = obj.get("bbox", [])
                if not name or len(bbox) != 4:
                    continue
                x1, y1, x2, y2 = [int(v) for v in bbox]
                # 이미지 경계 클램핑
                x1 = max(0, min(x1, w)); x2 = max(0, min(x2, w))
                y1 = max(0, min(y1, h)); y2 = max(0, min(y2, h))
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append({
                    "name": name,
                    "conf": round(conf, 3),
                    "box" : [x1, y1, x2, y2],
                })

            det_summary = [f"{d['name']}({d['conf']:.2f})" for d in detections]
            self.get_logger().info(f"[VLM] 탐지 결과: {det_summary}")
            return detections

        except Exception as exc:
            self.get_logger().error(f"[VLM] 분석 오류: {exc}")
            return []

    def _box_center(self, box: list) -> list:
        if len(box) < 4:
            return [0.0, 0.0]
        x1, y1, x2, y2 = box[:4]
        return [(x1 + x2) / 2.0, (y1 + y2) / 2.0]

    # ── 발행 헬퍼 ────────────────────────────────────────────────────────────

    def _publish_status(self, status: str, extra: dict):
        payload = {
            "status"        : status,
            "target_objects": self.target_objects,
            "timestamp"     : get_timestamp(),
            **extra,
        }
        msg      = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)

    def _publish_voice_intent_direct(self, name: str, best_pose: list):
        """
        센터링 완료 후 vision_node에 직접 /voice_intent 발행.
        publisher.py를 거치지 않고 즉시 YOLO 탐지 + 픽을 트리거한다.
        best_bbox 없이 보내므로 vision_node는 YOLO bbox만 사용.
        """
        payload = {
            "action"       : "bring_object",
            "target_object": [name],
            "urgency"      : "normal",
            "from_scan"    : True,
            "best_pose"    : best_pose,
            "timestamp"    : get_timestamp(),
        }
        msg      = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._voice_intent_pub.publish(msg)
        self.get_logger().info(
            f'[DIRECT PICK] /voice_intent 발행 → vision_node: {name}  '
            f'pose=({best_pose[0]:.1f},{best_pose[1]:.1f})')

    def _publish_scan_result(self, found_objects: list, cancelled: bool = False,
                             voice_intent_sent: bool = False):
        """
        스캔 완료 후 /scan_result 발행 → publisher.py 수신
        found_objects 의 키를 publisher.py 가 기대하는 형식으로 변환.
        """
        ts     = get_timestamp()
        status = ("cancelled" if cancelled
                  else ("found" if found_objects else "not_found"))

        serializable = [
            {
                "label"         : obj.get("name", ""),
                "confidence"    : obj.get("conf", 0.0),
                "best_pose"     : obj.get("best_pose", []),
                "best_bbox_xyxy": obj.get("box", []),
            }
            for obj in found_objects
        ]

        payload = {
            "status"            : status,
            "target_objects"    : self.target_objects,
            "found_objects"     : serializable,
            "voice_intent_sent" : voice_intent_sent,
            "timestamp"         : ts,
        }
        msg      = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._result_pub.publish(msg)

        W = 56
        print(f"\n{'═'*W}")
        print(f"  📡  /scan_result 발행  [{ts}]")
        print(f"{'═'*W}")
        print(f"  status         : {status}")
        print(f"  target_objects : {self.target_objects}")
        if found_objects:
            for obj in found_objects:
                print(f"  ✅  {obj.get('name')}  (conf={obj.get('conf', 0.0):.3f})")
        else:
            print(f"  ❌  미발견: {self.target_objects or '(전체 스캔)'}")
        print(f"{'═'*W}\n")

    # ── 로그 ────────────────────────────────────────────────────────────────

    def _reset_log(self):
        log_dir = os.path.dirname(self.log_file_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        with open(self.log_file_path, "w", encoding="utf-8"):
            pass

    def _append_log(self, record: dict):
        log_dir = os.path.dirname(self.log_file_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        with open(self.log_file_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

    # ── 클러스터링 (최종 요약) ───────────────────────────────────────────────

    def _xy_distance(self, a: list, b: list) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def build_final_summary(self, scan_records: list) -> list:
        """복수 포즈에서 같은 물체를 중복 카운트하지 않도록 클러스터링."""
        clusters_by_label: dict = {}

        for record in scan_records:
            pose = record["pose"]
            for det in record["detections"]:
                label     = det.get("name", "unknown")
                center_xy = det.get("center_xy") or self._box_center(
                    det.get("box", []))
                if not center_xy:
                    continue

                pose_xy  = [float(pose[0]), float(pose[1])]
                clusters = clusters_by_label.setdefault(label, [])
                matched  = None

                for cluster in clusters:
                    if any(
                        self._xy_distance(pose_xy, obs["pose_xy"])
                        < self.pose_distance_threshold
                        and self._xy_distance(center_xy, obs["center_xy"])
                        < self.image_distance_threshold
                        for obs in cluster["observations"]
                    ):
                        matched = cluster
                        break

                if matched is None:
                    matched = {
                        "label"          : label,
                        "best_conf"      : det["conf"],
                        "detected_count" : 0,
                        "best_scan_index": record["scan_index"],
                        "best_pose"      : pose,
                        "best_box"       : det.get("box", []),
                        "observations"   : [],
                        "pose_sum_xy"    : [0.0, 0.0],
                        "center_sum_xy"  : [0.0, 0.0],
                    }
                    clusters.append(matched)

                matched["detected_count"]   += 1
                matched["observations"].append(
                    {"pose_xy": pose_xy, "center_xy": center_xy})
                matched["pose_sum_xy"][0]   += pose_xy[0]
                matched["pose_sum_xy"][1]   += pose_xy[1]
                matched["center_sum_xy"][0] += center_xy[0]
                matched["center_sum_xy"][1] += center_xy[1]

                if det["conf"] > matched["best_conf"]:
                    matched["best_conf"]       = det["conf"]
                    matched["best_scan_index"] = record["scan_index"]
                    matched["best_pose"]       = pose
                    matched["best_box"]        = det.get("box", [])

        summary = []
        for label, clusters in clusters_by_label.items():
            prefix = re.sub(r"[^0-9A-Za-z_]+", "_", label.strip()).strip("_") or "obj"
            for idx, cluster in enumerate(clusters, start=1):
                n = cluster["detected_count"]
                summary.append({
                    "object_id"      : f"{prefix}_{idx}",
                    "label"          : cluster["label"],
                    "best_confidence": round(cluster["best_conf"], 3),
                    "detected_count" : n,
                    "best_scan_index": cluster["best_scan_index"],
                    "best_pose"      : cluster["best_pose"],
                    "best_bbox_xyxy" : cluster["best_box"],
                    "avg_pose_xy"    : [
                        round(cluster["pose_sum_xy"][0] / n, 1),
                        round(cluster["pose_sum_xy"][1] / n, 1)],
                    "avg_center_xy"  : [
                        round(cluster["center_sum_xy"][0] / n, 1),
                        round(cluster["center_sum_xy"][1] / n, 1)],
                })
        return summary

    def _write_final_summary(self, summary: list):
        object_counts = {}
        for item in summary:
            label = item["label"]
            object_counts[label] = object_counts.get(label, 0) + 1

        root, ext = os.path.splitext(self.log_file_path)
        path      = f"{root}_summary.json" if ext else f"{self.log_file_path}_summary.json"
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump({
                "unique_objects"          : len(summary),
                "object_counts"           : object_counts,
                "target_objects"          : self.target_objects,
                "pose_distance_threshold" : self.pose_distance_threshold,
                "image_distance_threshold": self.image_distance_threshold,
                "objects"                 : summary,
                "timestamp"               : time.time(),
            }, f, ensure_ascii=False, indent=2)

        self.get_logger().info("===== JARVIS SCAN SUMMARY =====")
        for label, count in sorted(object_counts.items()):
            self.get_logger().info(f"  - {label} x{count}")
        self.get_logger().info(f"summary → {path}")
        self.summary_written = True

    # ── 스캔 실행 ────────────────────────────────────────────────────────────

    def run_scan(self):
        """로봇 지그재그 이동 + vision_node 탐지 수집."""
        api = self.robot_api
        if not all([api.get("set_robot_mode"), api.get("set_tool"),
                    api.get("set_tcp"),       api.get("get_robot_state"),
                    api.get("movel"),          api.get("wait")]):
            raise RuntimeError("robot_api 미설정 — configure_robot_api() 먼저 호출하세요")

        set_robot_mode  = api["set_robot_mode"]
        set_tool        = api["set_tool"]
        set_tcp         = api["set_tcp"]
        get_robot_state = api["get_robot_state"]
        movel           = api["movel"]
        wait            = api["wait"]
        move_mod_abs    = api["move_mod_abs"]

        self.get_logger().info(
            f"🔍 스캔 시작 — 타겟: {self.target_objects or '전체'}")

        self._reset_log()
        self.scan_records       = []
        self.summary_written    = False
        found_objects_map       : dict = {}
        cancelled               = False

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        time.sleep(0.5)

        total = len(self.scan_points)

        for index, pose in enumerate(self.scan_points, start=1):
            if self.cancel_requested:
                self.get_logger().warn("취소 요청 → 스캔 중단")
                cancelled = True
                break

            state = get_robot_state()
            if state in UNSAFE_STATES:
                self.get_logger().error(f"로봇 상태 {state} 비정상 → 스캔 중단")
                break

            self.get_logger().info(
                f"[{index}/{total}] 이동 → {pose[:2]}  (state={state})")
            movel(pose, vel=self.vel, acc=self.acc, mod=move_mod_abs)
            wait(self.capture_wait_sec)

            self._publish_status("scanning", {
                "current_pose_index": index,
                "total_poses"       : total,
                "current_pose"      : pose,
            })

            detections = self._collect_detections_at_pose()

            if not detections and not self.cancel_requested:
                self.get_logger().info("  탐지 없음 → 1회 재시도")
                detections = self._collect_detections_at_pose()

            record = {
                "scan_index": index,
                "total"     : total,
                "pose"      : pose,
                "detections": detections,
                "timestamp" : time.time(),
            }
            self.scan_records.append(record)
            self._append_log(record)

            for det in detections:
                self.get_logger().info(
                    f"  ✅ {det['name']} conf={det['conf']:.3f}")
                name = det["name"]
                if (name not in found_objects_map or
                        det["conf"] > found_objects_map[name]["conf"]):
                    found_objects_map[name] = {**det, "best_pose": pose}

            if not detections:
                self.get_logger().info("  (탐지 없음)")

            # 타겟 물체 모두 발견 시 → 스캔 조기 종료 (픽은 아래 공통 블록에서 처리)
            if self.target_objects:
                remaining = [t for t in self.target_objects
                             if t not in found_objects_map]
                if not remaining:
                    self.get_logger().info(
                        f"✅ 모든 타겟 발견 [{index}/{total}] → 스캔 조기 종료")
                    break

        # ── 발견 물체 순차 처리: 센터링 → 픽 트리거 → 완료 대기 ──────────────
        # 물체마다 (1)센터링으로 로봇을 물체 위로 이동 (2)/voice_intent 발행
        # (3)픽 완료 대기 후 다음 물체로 진행.
        # → 항상 해당 물체 위에서 YOLO/depth 계산이 이루어져 정확한 픽 가능.

        # 스캔 도중 진행 중이던 픽이 있으면 완료 대기 후 시작
        if self._is_picking_ext:
            self.get_logger().info('[CENTER+PICK] 이전 픽 완료 대기 중...')
            self._pick_complete_event.clear()
            if not self._pick_complete_event.wait(timeout=120.0):
                self.get_logger().warn('[CENTER+PICK] 이전 픽 완료 타임아웃 — 강제 진행')

        if found_objects_map and not cancelled:
            self.get_logger().info(
                f'[CENTER+PICK] 순차 처리 시작: {list(found_objects_map.keys())}')
            for name, obj in list(found_objects_map.items()):
                if self.cancel_requested:
                    cancelled = True
                    break

                # ① 센터링: 로봇을 물체 정중앙으로 이동
                centered = self._compute_centering_pose(
                    obj['best_pose'], obj.get('box', []))
                if centered is not None:
                    self.get_logger().info(
                        f'[CENTER] {name}: '
                        f'({obj["best_pose"][0]:.1f},{obj["best_pose"][1]:.1f})'
                        f' → ({centered[0]:.1f},{centered[1]:.1f})')
                    movel(centered, vel=self.vel, acc=self.acc, mod=move_mod_abs)
                    time.sleep(self.centering_wait_sec)
                    obj['best_pose'] = centered
                else:
                    self.get_logger().warn(
                        f'[CENTER] {name}: 계산 불가 — 현재 포즈 유지')

                # ② /voice_intent 발행 → orchestrator from_scan=True 세팅 → 픽 트리거
                # - orchestrator: _next_pick_from_scan=True → goal.from_scan=True
                # - vision_node: /selected_object 발행 → 각도 파이프라인 시작
                # - pick_and_place: HOME 스킵, 각도 보존, SPIN_CHUCK(30s)에서 각도 수신
                if not self.cancel_requested:
                    self._publish_voice_intent_direct(name, obj['best_pose'])

                # ③ 픽 완료 대기 (다음 물체로 넘어가기 전)
                if not self.cancel_requested:
                    self._wait_for_pick_complete(timeout=120.0)

        found_list = list(found_objects_map.values())
        self.get_logger().info(
            f"스캔 {'취소' if cancelled else '완료'} — "
            f"발견: {[o['name'] for o in found_list]}")

        # 모든 픽 완료 후 scan_result 발행 (publisher.py TTS/상태 알림용)
        voice_intent_sent = bool(found_list and not cancelled)
        self._publish_scan_result(found_list, cancelled=cancelled,
                                  voice_intent_sent=voice_intent_sent)

        if self.scan_records:
            summary = self.build_final_summary(self.scan_records)
            self._write_final_summary(summary)

        self._publish_status("idle", {"cancelled": cancelled})

    # ── 요청 대기 메인 루프 ──────────────────────────────────────────────────

    def spin_until_scan_request(self):
        self.get_logger().info(f"⏳ 스캔 요청 대기 중 [/scan_request]")

        while rclpy.ok():
            time.sleep(0.1)

            if not self.scan_requested:
                continue

            self.scan_requested  = False
            self.scan_running    = True
            self.summary_written = False

            self._publish_status("starting", {
                "target_objects": self.target_objects})

            try:
                self.run_scan()
            except Exception as exc:
                self.get_logger().error(f"스캔 오류: {exc}")
                self._publish_scan_result([], cancelled=False)
            finally:
                self.scan_running     = False
                self.cancel_requested = False
                self.target_objects   = []
                self.get_logger().info("⏳ 스캔 요청 대기 중 [/scan_request]")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceScanNode()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        DR_MV_MOD_ABS,
        get_robot_state,
        movel,
        set_robot_mode,
        set_tcp,
        set_tool,
        wait,
    )
    node.configure_robot_api(
        set_robot_mode  = set_robot_mode,
        set_tool        = set_tool,
        set_tcp         = set_tcp,
        get_robot_state = get_robot_state,
        movel           = movel,
        wait            = wait,
        move_mod_abs    = DR_MV_MOD_ABS,
    )
    node.get_logger().info("✅ DSR_ROBOT2 API 바인딩 완료")

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin, daemon=True, name="ros2_spin")
    spin_thread.start()

    try:
        node.spin_until_scan_request()
    except KeyboardInterrupt:
        node.get_logger().warn("KeyboardInterrupt → 종료")
    finally:
        if not node.summary_written and node.scan_records:
            summary = node.build_final_summary(node.scan_records)
            node._write_final_summary(summary)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()