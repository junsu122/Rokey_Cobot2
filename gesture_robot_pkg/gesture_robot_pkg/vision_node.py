"""
vision_node.py
──────────────
책임:
  1. RealSense 컬러 이미지 구독
  2. YOLOv8 물체 검출 (bbox, class, confidence)
  3. GestureEvent 구독 → 검지 끝 좌표로 호버 판정 + 손 스켈레톤 표시
  4. HOVER_SEC 동안 검지가 bbox 위에 머물면 SelectedObject 퍼블리시
  5. /is_picking 구독 → PICKING 오버레이 표시
  6. 어노테이션된 프레임을 cv2 창에 표시

구독:  /camera/camera/color/image_raw  (sensor_msgs/Image)
       /gesture_event                  (GestureEvent)
       /is_picking                     (std_msgs/Bool)
퍼블리시: /selected_object             (SelectedObject)
"""

import threading
import time

import cv2
import torch
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
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


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ── 파라미터 ──────────────────────────────────
        self.declare_parameter('yolo_model',     YOLO_MODEL_PATH)
        self.declare_parameter('yolo_conf',      YOLO_CONF_THRESHOLD)
        self.declare_parameter('hover_sec',      HOVER_SEC)
        self._yolo_path = self.get_parameter('yolo_model').value
        self._yolo_conf = self.get_parameter('yolo_conf').value
        self._hover_sec = self.get_parameter('hover_sec').value

        # ── 퍼블리셔 ──────────────────────────────────
        self._pub_obj = self.create_publisher(SelectedObject, '/selected_object', 10)

        # ── 구독 ─────────────────────────────────────
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

        # ── YOLO ──────────────────────────────────────
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'YOLO device={device}  model={self._yolo_path}')
        try:
            self._yolo = YOLO(self._yolo_path).to(device)
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            self._yolo = None

        # YOLO는 매 N 프레임마다만 실행 (입력 ~30 Hz → 검출 ~10 Hz)
        self._yolo_skip  = 0
        self._yolo_every = 3

        # ── CvBridge ─────────────────────────────────
        self._bridge = CvBridge()

        # ── 내부 상태 (스레드 안전) ────────────────────
        self._latest_frame      = None
        self._frame_lock        = threading.Lock()

        self._detections: list  = []
        self._detect_lock       = threading.Lock()

        # 최신 GestureEvent
        self._gesture_state   = ''
        self._is_pointing     = False
        self._is_fist         = False
        self._index_tip_x     = 0.0
        self._index_tip_y     = 0.0
        self._hand_visible    = False
        self._landmarks_x     = [0.0] * 21
        self._landmarks_y     = [0.0] * 21
        self._gesture_lock    = threading.Lock()

        # 픽앤플레이스 진행 중 여부 + 호버 추적 (함께 보호)
        self._is_picking_ext  = False
        self._hover_target: str | None   = None
        self._hover_start: float | None  = None
        self._selected_label: str | None = None
        self._selected_box: list | None  = None
        self._pick_state_lock = threading.Lock()

        # 이미지 해상도 (gesture 좌표 변환용)
        self._img_w = 640
        self._img_h = 480

        self.get_logger().info('VisionNode ready')

    # ── 구독 콜백 ─────────────────────────────────────

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

    def _color_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        self._img_h, self._img_w = frame.shape[:2]

        # ── YOLO 검출 (매 _yolo_every 프레임마다 실행) ──────────
        if self._yolo is not None:
            self._yolo_skip = (self._yolo_skip + 1) % self._yolo_every
            if self._yolo_skip == 0:
                try:
                    results = self._yolo.predict(
                        source=frame, conf=self._yolo_conf,
                        imgsz=640, save=False, verbose=False)
                    annotated = results[0].plot()
                    new_dets = []
                    for box in results[0].boxes:
                        b        = box.xyxy[0].cpu().numpy().astype(int)
                        cls_id   = int(box.cls[0])
                        cls_name = self._yolo.names[cls_id]
                        conf     = float(box.conf[0])
                        new_dets.append({'name': cls_name, 'box': b.tolist(), 'conf': conf})
                    with self._detect_lock:
                        self._detections = new_dets
                except Exception as e:
                    self.get_logger().warn(f'YOLO predict error: {e}')
                    annotated = frame.copy()
            else:
                # YOLO 스킵: 캐시된 bbox를 현재 프레임 위에 직접 그림
                annotated = frame.copy()
                with self._detect_lock:
                    cached = list(self._detections)
                for det in cached:
                    b  = det['box']
                    lb = f"{det['name']} {det['conf']:.2f}"
                    cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                    cv2.putText(annotated, lb, (b[0], b[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            annotated = frame.copy()

        with self._detect_lock:
            detections = list(self._detections)

        # ── 제스처 스냅샷 ─────────────────────────────
        with self._gesture_lock:
            gesture_state = self._gesture_state
            is_pointing   = self._is_pointing
            is_fist_g     = self._is_fist
            tip_x         = self._index_tip_x
            tip_y         = self._index_tip_y
            hand_visible  = self._hand_visible
            landmarks_x   = list(self._landmarks_x)
            landmarks_y   = list(self._landmarks_y)

        # 검지 픽셀 좌표
        ix = int(tip_x * self._img_w)
        iy = int(tip_y * self._img_h)

        # ── 픽/호버 상태 스냅샷 (짧게 락 획득) ──────────
        with self._pick_state_lock:
            is_picking_ext = self._is_picking_ext
            hover_target   = self._hover_target
            hover_start    = self._hover_start
            selected_label = self._selected_label
            selected_box   = self._selected_box

        # ── 호버 판정 (로컬 변수만 사용) ─────────────────
        current_hover_name: str | None = None
        current_hover_box: list | None = None
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
                    draw_hover_progress(annotated, current_hover_box,
                                        progress, current_hover_name)
                    if elapsed >= self._hover_sec and selected_label != current_hover_name:
                        selected_label = current_hover_name
                        selected_box   = current_hover_box
                        hover_target   = None
                        hover_start    = None
                        self.get_logger().info(f'[SELECTED] {current_hover_name}')
                        self._publish_selected(current_hover_name, current_hover_box,
                                               detections)
                else:
                    hover_target = current_hover_name
                    hover_start  = time.time()
            else:
                hover_target = None
                hover_start  = None

        # 계산 결과를 한 번에 write-back (짧게 락 획득)
        with self._pick_state_lock:
            self._hover_target   = hover_target
            self._hover_start    = hover_start
            self._selected_label = selected_label
            self._selected_box   = selected_box

        # ── 손 스켈레톤 (v6: 제스처별 색상) ─────────
        if hand_visible and len(landmarks_x) == 21 and any(v != 0.0 for v in landmarks_x):
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
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, sk_color, 2, cv2.LINE_AA)

            # 상태 레이블
            s_color = COLORS.get(gesture_state, COLORS['NONE'])
            cv2.putText(annotated, f'[{gesture_state}]', (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, s_color, 2, cv2.LINE_AA)

        # ── 선택 레이블 ─────────────────────────────
        if selected_label and not is_picking_ext:
            draw_selected_label(annotated, selected_label)

        # ── PICKING 오버레이 ─────────────────────────
        if is_picking_ext:
            draw_picking_overlay(annotated, selected_label or '')

        # ── 하단 힌트 텍스트 ─────────────────────────
        cv2.putText(annotated,
                    '☝ point at obj + hold=PICK  |  PAUSE  |  C:clear',
                    (10, self._img_h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

        with self._frame_lock:
            self._latest_frame = annotated

    # ── SelectedObject 퍼블리시 ───────────────────────

    def _publish_selected(self, label: str, box: list, detections: list):
        conf = next((d['conf'] for d in detections if d['name'] == label), 0.0)
        msg             = SelectedObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.label       = label
        msg.confidence  = float(conf)
        msg.box         = [int(v) for v in box]
        self._pub_obj.publish(msg)

    # ── 디스플레이 루프 (별도 스레드) ─────────────────

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
                self._selected_label = None
                self._selected_box   = None
                self._hover_target   = None
                self._hover_start    = None
                self.get_logger().info('[CLEAR] Selection reset.')

        cv2.destroyAllWindows()


# ══════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════

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
