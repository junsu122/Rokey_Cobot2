import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

# VisionNode에서 정의한 커스텀 메시지 임포트
from gesture_robot_interfaces.msg import SelectedObject

class SelectedObjectTo3DNode(Node):
    def __init__(self):
        super().__init__('selected_object_to_3d_node')
        
        self.bridge = CvBridge()
        
        # 1. 데이터 저장용 변수
        self.current_depth_frame = None

        # 2. Realsense 카메라 파라미터 (제공해주신 값 유지)
        self.fx = 908.65698
        self.fy = 908.91968
        self.cx = 637.27130
        self.cy = 358.38000

        # 3. ROS 2 구독(Sub) 설정
        # Depth 이미지는 좌표 계산을 위해 계속 구독합니다.[cite: 3]
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        
        # 🚀 마우스 클릭 대신 VisionNode의 결과 토픽을 구독합니다.
        self.selected_obj_sub = self.create_subscription(
            SelectedObject, '/selected_object', self.selected_object_callback, 10)
        
        # 4. ROS 2 발행(Pub) 설정
        self.point_pub = self.create_publisher(PointStamped, '_2d_to_3d_point', 10)

        self.get_logger().info("🚀 SelectedObject 기반 3D 변환 노드가 실행되었습니다.")

    def depth_callback(self, msg):
        # Depth 프레임 업데이트[cite: 3]
        self.current_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def selected_object_callback(self, msg: SelectedObject):
        """
        /selected_object 토픽이 들어오면 자동으로 3D 좌표를 계산합니다.[cite: 1, 2]
        """
        if self.current_depth_frame is None:
            self.get_logger().warn("Depth 프레임이 아직 수신되지 않았습니다.")
            return

        # 1. 메시지에서 바운딩 박스 정보 추출
        # box: [x1, y1, x2, y2]
        x1, y1, x2, y2 = msg.box
        cls_name = msg.label
        
        # 2. 중심점 계산 (2D)[cite: 3]
        u_center_color = (x1 + x2) // 2
        v_center_color = (y1 + y2) // 2
        
        # 3. Depth 이미지 해상도에 맞춰 스케일링 (1280x720 -> 848x480)[cite: 3]
        u_center_depth = int(u_center_color * (848.0 / 1280.0))
        v_center_depth = int(v_center_color * (480.0 / 720.0))
        
        try:
            # 해당 중심점의 깊이값(mm) 추출[cite: 3]
            depth_mm = self.current_depth_frame[v_center_depth, u_center_depth]
        except IndexError:
            self.get_logger().error("계산된 좌표가 Depth 이미지 범위를 벗어났습니다.")
            return
        
        if depth_mm == 0:
            self.get_logger().warn(f"[{cls_name}] 지점의 Depth 값이 0입니다. (인식 불능)")
            return

        # 4. 2D 픽셀 -> 3D 공간 좌표 변환[cite: 3]
        depth_m = float(depth_mm) / 1000.0
        x_3d = (u_center_color - self.cx) * depth_m / self.fx
        y_3d = (v_center_color - self.cy) * depth_m / self.fy
        z_3d = depth_m

        # 5. 3D 좌표 메시지 생성 및 발행[cite: 3]
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = cls_name  # 클래스 이름을 frame_id에 담음
        
        point_msg.point.x = float(x_3d)
        point_msg.point.y = float(y_3d)
        point_msg.point.z = float(z_3d)

        self.point_pub.publish(point_msg)
        self.get_logger().info(f"📤 [AUTO] Published {cls_name}: X={x_3d:.4f}, Y={y_3d:.4f}, Z={z_3d:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = SelectedObjectTo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()