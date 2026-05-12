"""
vision_node.py  ── UI 버전
──────────────────────────
구독:
  /camera/camera/color/image_raw  (sensor_msgs/Image)
  /is_picking                     (std_msgs/Bool)
  /voice_intent                   (std_msgs/String)

발행:
  /annotated_frame   (sensor_msgs/Image)  → webrtc_vision_server → 브라우저 영상
  /yolo_detections   (std_msgs/String)    → webrtc_vision_server → 브라우저 bbox
  /selected_object   (SelectedObject)     → pick_and_place_node
  /object_not_found  (std_msgs/String)    → jarvis_voice_pkg
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

from gesture_robot_interfaces.msg import SelectedObject
from gesture_robot_pkg.constants import (
    REALSENSE_COLOR_TOPIC,
    YOLO_MODEL_PATH, YOLO_CONF_THRESHOLD,
    HOVER_SEC,
)


def _is_bring_action(action: str) -> bool:
    return isinstance(action, str) and action.startswith("bring_")


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter('yolo_model',              YOLO_MODEL_PATH)
        self.declare_parameter('yolo_conf',               YOLO_CONF_THRESHOLD)
        self.declare_parameter('hover_sec',               HOVER_SEC)
        self.declare_parameter('voice_not_found_timeout', 3.0)
        self.declare_parameter('scan_yolo_timeout',       1.0)

        self._yolo_path               = self.get_parameter('yolo_model').value
        self._yolo_conf               = self.get_parameter('yolo_conf').value
        self._voice_not_found_timeout = float(self.get_parameter('voice_not_found_timeout').value)
        self._scan_yolo_timeout       = float(self.get_parameter('scan_yolo_timeout').value)

        # ── 퍼블리셔 ──────────────────────────────────────────────────────
        self._pub_annotated  = self.create_publisher(Image,          '/annotated_frame',  10)
        self._pub_yolo       = self.create_publisher(String,         '/yolo_detections',  10)
        self._pub_obj        = self.create_publisher(SelectedObject, '/selected_object',  10)
        self._pub_not_found  = self.create_publisher(String,         '/object_not_found', 10)

        # ── 구독 ──────────────────────────────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Image, REALSENSE_COLOR_TOPIC, self._color_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Bool, '/is_picking', self._is_picking_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/voice_intent', self._voice_intent_cb, 10,
            callback_group=self._cb_group)
        # 브라우저 호버 선택 수신 (webrtc_vision_server → vision_node)
        # box 없이 label만 오면 현재 YOLO detections에서 원본 좌표 추출
        self.create_subscription(
            String, '/selected_label', self._selected_label_cb, 10,
            callback_group=self._cb_group)

        # ── YOLO ──────────────────────────────────────────────────────────
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'YOLO device={device}  model={self._yolo_path}')
        try:
            self._yolo = YOLO(self._yolo_path).to(device)
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            self._yolo = None

        # ── CvBridge ──────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── 내부 상태 ─────────────────────────────────────────────────────
        self._img_w = 640
        self._img_h = 480

        self._detections: list = []
        self._detect_lock      = threading.Lock()

        self._is_picking_ext             = False
        self._selected_label: str | None = None
        self._selected_box:  list | None = None
        self._pick_state_lock            = threading.Lock()

        self._voice_targets: list[str] = []
        self._voice_deadline: float    = 0.0
        self._voice_lock               = threading.Lock()

        # ── YOLO 비동기 처리용 ────────────────────────────────────────────
        self._latest_frame      = None          # _color_cb가 항상 최신 프레임 저장
        self._latest_frame_lock = threading.Lock()
        self._yolo_thread = threading.Thread(target=self._yolo_loop, daemon=True)
        self._yolo_thread.start()

        self.create_timer(0.3, self._check_voice_targets_cb,
                          callback_group=self._cb_group)

        self.get_logger().info('VisionNode ready')
        self.get_logger().info(f'voice_not_found_timeout={self._voice_not_found_timeout}s')

    # ─────────────────────────────────────────────────────────────────────
    # 구독 콜백
    # ─────────────────────────────────────────────────────────────────────

    def _is_picking_cb(self, msg: Bool):
        with self._pick_state_lock:
            self._is_picking_ext = msg.data
            if not msg.data:
                self._selected_label = None
                self._selected_box   = None

    def _selected_label_cb(self, msg: String):
        """
        /selected_label 수신 (브라우저 호버 선택)
        → 현재 YOLO detections에서 해당 라벨 찾아 원본 RealSense 좌표로 /selected_object 발행
        """
        try:
            data  = json.loads(msg.data)
            label = data.get('label', '')
            conf  = float(data.get('confidence', 0.0))

            if not label:
                return

            with self._pick_state_lock:
                if self._is_picking_ext:
                    self.get_logger().warn(
                        f'[SELECT] 픽 진행 중 — {label} 무시')
                    return

            with self._detect_lock:
                current_dets = list(self._detections)

            # 현재 YOLO detections에서 해당 라벨 탐색
            det = next((d for d in current_dets if d['name'] == label), None)

            if det is None:
                self.get_logger().warn(
                    f'[SELECT] {label} YOLO에서 미발견 (현재 감지: '
                    f'{[d["name"] for d in current_dets]})')
                return

            # orig_box: 실제 RealSense 해상도 원본 좌표 사용
            box = det['box']  # _yolo_loop에서 원본 좌표 그대로 저장

            so_msg              = SelectedObject()
            so_msg.header.stamp = self.get_clock().now().to_msg()
            so_msg.label        = label
            so_msg.confidence   = float(det.get('conf', conf))
            so_msg.box          = [int(v) for v in box]
            self._pub_obj.publish(so_msg)

            self.get_logger().info(
                f'[SELECT] {label} conf={so_msg.confidence:.3f} '
                f'box={box} (RealSense 원본 좌표)')

        except Exception as e:
            self.get_logger().warn(f'[SELECT] 파싱 오류: {e}')

    def _color_cb(self, msg: Image):
        # 최신 프레임만 저장 (YOLO 스레드가 비동기로 처리)
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        self._img_h, self._img_w = frame.shape[:2]
        with self._latest_frame_lock:
            self._latest_frame = frame

    def _yolo_loop(self):
        """YOLO를 별도 스레드에서 실행 — _color_cb 블로킹 없음"""
        while rclpy.ok():
            with self._latest_frame_lock:
                frame = self._latest_frame
                self._latest_frame = None   # 처리했음을 표시

            if frame is None:
                time.sleep(0.005)
                continue

            # ── YOLO 검출 ────────────────────────────────────────────────
            if self._yolo is not None:
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

                    # /yolo_detections 발행
                    # box: 640x480 스케일 (VisionCanvas 호버 판정용)
                    # orig_box: 원본 해상도 (pick_and_place_node depth 조회용)
                    sx = 640.0 / self._img_w
                    sy = 480.0 / self._img_h
                    scaled_dets = [
                        {'name': d['name'], 'conf': d['conf'],
                         'box': [int(d['box'][0]*sx), int(d['box'][1]*sy),
                                 int(d['box'][2]*sx), int(d['box'][3]*sy)],
                         'orig_box': d['box']}   # 원본 해상도 bbox
                        for d in new_dets
                    ]
                    yolo_msg      = String()
                    yolo_msg.data = json.dumps(
                        {'detections': scaled_dets,
                         'timestamp' : datetime.now().isoformat()},
                        ensure_ascii=False)
                    self._pub_yolo.publish(yolo_msg)

                except Exception as e:
                    self.get_logger().warn(f'YOLO predict error: {e}')
                    annotated = frame.copy()
            else:
                annotated = frame.copy()

            # ── voice_intent 탐색 중 오버레이 ────────────────────────────
            with self._voice_lock:
                v_targets  = list(self._voice_targets)
                v_deadline = self._voice_deadline

            if v_targets and v_deadline > 0.0:
                remaining = max(0.0, v_deadline - time.time())
                overlay   = f'탐색 중: {", ".join(v_targets)}  ({remaining:.1f}s)'
                cv2.putText(annotated, overlay, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA)

            # ── /annotated_frame 발행 ─────────────────────────────────────
            try:
                out     = cv2.resize(annotated, (640, 480))
                img_msg = self._bridge.cv2_to_imgmsg(out, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self._pub_annotated.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f'annotated_frame 발행 오류: {e}')

    # ─────────────────────────────────────────────────────────────────────
    # voice_intent 처리
    # ─────────────────────────────────────────────────────────────────────

    def _voice_intent_cb(self, msg: String):
        try:
            data    = json.loads(msg.data)
            action  = data.get('action', '')
            targets = data.get('target_object', [])

            if not _is_bring_action(action):
                return

            if isinstance(targets, str):
                targets = [targets]
            if not targets:
                return

            from_scan = bool(data.get('from_scan', False))
            self.get_logger().info(
                f'[VOICE_INTENT] action={action} targets={targets} from_scan={from_scan}')

            if from_scan and targets:
                label = targets[0]
                with self._detect_lock:
                    current_dets = list(self._detections)
                for det in current_dets:
                    if det['name'] == label:
                        self.get_logger().info(
                            f'[FROM_SCAN] ✅ YOLO 즉시 발견: {label} conf={det["conf"]:.2f}')
                        self._auto_select_from_scan(label, det['box'], det['conf'])
                        return
                self.get_logger().info(
                    f'[FROM_SCAN] YOLO 미발견 → {self._scan_yolo_timeout}s 대기')
                threading.Thread(
                    target=self._wait_for_yolo_then_pick,
                    args=(label, self._scan_yolo_timeout),
                    daemon=True).start()
                return

            with self._detect_lock:
                current_dets = list(self._detections)

            detected_names = {d['name'] for d in current_dets}
            found_now      = [t for t in targets if t in detected_names]
            missing_now    = [t for t in targets if t not in detected_names]

            self.get_logger().info(
                f'[VOICE] 즉시 확인 → 발견={found_now}, 미발견={missing_now}')

            if found_now:
                det_info = next(d for d in current_dets if d['name'] == found_now[0])
                self.get_logger().info(f'[VOICE] ✅ {found_now[0]} 발견 → 즉시 픽업 트리거')
                self._auto_select_from_scan(
                    found_now[0], det_info['box'], float(det_info.get('conf', 1.0)))

            if missing_now:
                self.get_logger().warn(
                    f'[VOICE] ⚠️  미발견: {missing_now} — {self._voice_not_found_timeout}s 대기')
                with self._voice_lock:
                    self._voice_targets  = list(missing_now)
                    self._voice_deadline = time.time() + self._voice_not_found_timeout

        except Exception as exc:
            self.get_logger().warn(f'voice_intent 파싱 오류: {exc}')

    def _check_voice_targets_cb(self):
        with self._voice_lock:
            targets  = list(self._voice_targets)
            deadline = self._voice_deadline

        if not targets or deadline == 0.0 or time.time() < deadline:
            return

        with self._detect_lock:
            detected_names = {d['name'] for d in self._detections}
            current_dets   = list(self._detections)

        found     = [t for t in targets if t in detected_names]
        not_found = [t for t in targets if t not in detected_names]

        with self._voice_lock:
            self._voice_targets  = []
            self._voice_deadline = 0.0

        if not_found:
            self.get_logger().warn(f'[VOICE] 최종 미발견: {not_found}  발견: {found}')
            self._publish_object_not_found(found, not_found)

        if found:
            self.get_logger().info(f'[VOICE] 타임아웃 후 발견: {found} → 즉시 픽업 트리거')
            for label in found:
                det_info = next((d for d in current_dets if d['name'] == label), None)
                if det_info:
                    self._auto_select_from_scan(
                        label, det_info['box'], float(det_info.get('conf', 1.0)))

    def _publish_object_not_found(self, found: list, not_found: list):
        payload  = {'found': found, 'not_found': not_found,
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}
        msg      = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_not_found.publish(msg)
        self.get_logger().info(f'[OBJECT_NOT_FOUND] found={found}, not_found={not_found}')

    def _wait_for_yolo_then_pick(self, label: str, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._detect_lock:
                dets = list(self._detections)
            for det in dets:
                if det['name'] == label:
                    self.get_logger().info(
                        f'[FROM_SCAN] ✅ YOLO 발견: {label} conf={det["conf"]:.2f}')
                    self._auto_select_from_scan(label, det['box'], det['conf'])
                    return
            time.sleep(0.1)

        cx, cy = self._img_w // 2, self._img_h // 2
        m = 40
        self.get_logger().warn(
            f'[FROM_SCAN] YOLO {timeout:.1f}s 탐지 실패 → 이미지 중앙 bbox로 픽: {label}')
        self._auto_select_from_scan(label, [cx-m, cy-m, cx+m, cy+m], 0.5)

    def _auto_select_from_scan(self, label: str, bbox: list, conf: float = 1.0):
        box_int = [int(v) for v in bbox[:4]] if len(bbox) >= 4 else []
        with self._pick_state_lock:
            self._selected_label = label
            self._selected_box   = box_int

        so_msg              = SelectedObject()
        so_msg.header.stamp = self.get_clock().now().to_msg()
        so_msg.label        = label
        so_msg.confidence   = float(conf)
        so_msg.box          = box_int
        self._pub_obj.publish(so_msg)

        self.get_logger().info(
            f'[FROM SCAN] 자동 선택 → label={label} conf={conf:.3f} bbox={box_int}')


# ══════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
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