import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped  # 🚀 3D 좌표 발행을 위한 메시지 타입
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class YoloClickTo3DNode(Node):
    def __init__(self):
        super().__init__('yolo_click_to_3d_node')
        
        # 1. 모델 및 브릿지 초기화
        self.model = YOLO("./src/object_width_filter/yolo_model/best_first.pt")
        self.bridge = CvBridge()
        
        # 2. 데이터 저장용 변수
        self.current_color_frame = None
        self.current_depth_frame = None
        self.detected_results = None

        # 3. 제공해주신 camera_info 파라미터 적용 (우리 realsense에 맞춘 정보) (5.5 확인 완료)
        self.fx = 908.65698
        self.fy = 908.91968
        self.cx = 637.27130
        self.cy = 358.38000

        # 4. ROS 2 구독(Sub) 및 발행(Pub) 설정
        self.img_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        
        # 🚀 [추가] 변환된 3D 좌표를 발행하는 Publisher
        self.point_pub = self.create_publisher(PointStamped, '_2d_to_3d_point', 10)

        # 5. OpenCV 윈도우 및 마우스 콜백 설정
        cv2.namedWindow("YOLO_Detection")
        cv2.setMouseCallback("YOLO_Detection", self.mouse_click_event)

        self.get_logger().info("🚀 YOLO 3D 노드가 실행되었습니다. 화면을 클릭하면 '2d_to_3d_point'로 좌표를 보냅니다.")

    def color_callback(self, msg):
        self.current_color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(source=self.current_color_frame, conf=0.25, verbose=False)
        self.detected_results = results[0]
        
        annotated_frame = self.detected_results.plot()
        cv2.imshow("YOLO_Detection", annotated_frame)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.current_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def mouse_click_event(self, event, u, v, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.current_depth_frame is None or self.detected_results is None:
                self.get_logger().warn("데이터가 아직 충분하지 않습니다.")
                return

            boxes = self.detected_results.boxes
            clicked_on_box = False

            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                
                if x1 <= u <= x2 and y1 <= v <= y2:
                    clicked_on_box = True
                    
                    u_center_color = (x1 + x2) // 2
                    v_center_color = (y1 + y2) // 2
                    
                    # Depth 이미지 해상도(848x480)에 맞춰 스케일링
                    u_center_depth = int(u_center_color * (848.0 / 1280.0))
                    v_center_depth = int(v_center_color * (480.0 / 720.0))
                    
                    try:
                        depth_mm = self.current_depth_frame[v_center_depth, u_center_depth]
                    except IndexError:
                        continue
                    
                    if depth_mm == 0:
                        self.get_logger().warn("클릭한 지점의 Depth 값이 0입니다.")
                        continue

                    depth_m = float(depth_mm) / 1000.0
                    x_3d = (u_center_color - self.cx) * depth_m / self.fx
                    y_3d = (v_center_color - self.cy) * depth_m / self.fy
                    z_3d = depth_m

                    # 터미널 출력
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = self.model.names[cls_id]
                    print(f"🎯 Detected: {cls_name} | Pub 완료")

                    # 🚀 [추가] 3D 좌표 메시지 생성 및 발행
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = "camera_depth_optical_frame"  # 또는 적절한 프레임 ID
                    # 🚀 frame_id에 클래스 이름을 담아서 보냅니다.
                    point_msg.header.frame_id = cls_name
                    point_msg.point.x = float(x_3d)
                    point_msg.point.y = float(y_3d)
                    point_msg.point.z = float(z_3d)

                    self.point_pub.publish(point_msg)
                    self.get_logger().info(f"📤 Published Point: X={x_3d:.4f}, Y={y_3d:.4f}, Z={z_3d:.4f}")

                    break
            
            if not clicked_on_box:
                self.get_logger().info("바운딩 박스 외부를 클릭하셨습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = YoloClickTo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()