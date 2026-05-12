import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np

class XyzVisualizerNode(Node):
    def __init__(self):
        super().__init__('xyz_visualizer_node')

        # 1. 구독 및 발행 설정
        self.img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/xyz_pointcloud', self.pc_callback, 10)
        self.image_pub = self.create_publisher(Image, '/projected_xyz_image', 10)

        self.bridge = CvBridge()
        self.latest_points = []

        # 🚀 2. 알려주신 내부 파라미터 직접 설정
        # fx, fy: 초점 거리 / cx, cy: 주점(중심점)
        self.fx, self.fy = 908.65698, 908.91968
        self.cx, self.cy = 637.27130, 358.38000
        
        # 카메라 행렬 (Intrinsic Matrix K) 구성
        self.camera_matrix = np.array([
            [self.fx, 0,       self.cx],
            [0,       self.fy, self.cy],
            [0,       0,       1]
        ], dtype=np.float32)

        # 왜곡 계수 (알 수 없는 경우 0으로 설정, RealSense는 보통 보정되어 나옴)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.get_logger().info("🚀 [XYZ 시각화 노드] 고정 파라미터 버전 가동 중")

    def pc_callback(self, msg):
        """xyz_pointcloud에서 점들을 수신"""
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.latest_points = [[p[0], p[1], p[2]] for p in points_gen]

    def image_callback(self, msg):
        """3D 좌표를 2D 이미지 평면에 투영"""
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if not self.latest_points:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
                return

            # 3D 점들을 numpy 배열로 변환
            pts_3d = np.array(self.latest_points, dtype=np.float32)

            # 🚀 3D to 2D 투영 (이미 카메라 좌표계이므로 rvec, tvec은 0)
            rvec = np.zeros((3, 1), dtype=np.float32)
            tvec = np.zeros((3, 1), dtype=np.float32)

            # OpenCV의 projectPoints 사용하여 픽셀 좌표 계산
            pts_2d, _ = cv2.projectPoints(pts_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs)

            # 결과 그리기
            for i, pt in enumerate(pts_2d):
                u, v = map(int, pt.ravel())
                
                # 이미지 경계 검사
                if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                    # 물체 위치에 원 그리기
                    cv2.circle(cv_image, (u, v), 8, (0, 0, 255), -1)  # 빨간색 점
                    cv2.circle(cv_image, (u, v), 9, (255, 255, 255), 1) # 테두리
                    
                    # 정보 표시 (Z값 및 픽셀 좌표)
                    z_val = self.latest_points[i][2]
                    label = f"Z: {z_val:.3f}m ({u}, {v})"
                    cv2.putText(cv_image, label, (u + 12, v), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(cv_image, label, (u + 12, v), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            # 이미지 발행
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Image projection error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = XyzVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()