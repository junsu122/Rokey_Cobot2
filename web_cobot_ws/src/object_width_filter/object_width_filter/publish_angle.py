import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32
from shapely.geometry import Polygon

class ClickAngleExtractorNode(Node):
    def __init__(self):
        super().__init__('click_angle_extractor_node')

        # 1. Perception 노드에서 발행하는 엣지 마커 구독
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/z_edge_marker',
            self.marker_callback,
            10
        )

        # 2. YOLO 노드에서 클릭 시 발행하는 3D 좌표 구독
        # (토픽명은 준수님이 적어주신 '_2d_to_3d_point'로 맞췄습니다)
        self.point_sub = self.create_subscription(
            PointStamped,
            '_2d_to_3d_point',
            self.click_callback,
            10
        )

        # 3. 결과 시각화 및 각도 발행용 Publisher
        self.min_length_pub = self.create_publisher(MarkerArray, '/min_length', 10)
        self.angle_pub = self.create_publisher(Float32, '/object_angle', 10)

        self.latest_markers = None
        self.dist_threshold = 0.1  # 🚀 동일 객체로 판단할 거리 임계값 (10cm)

        self.get_logger().info("🚀 [좌표 비교형 각도 추출 노드] 가동되었습니다.")

    def marker_callback(self, msg):
        self.latest_markers = msg

    def click_callback(self, msg):
        if self.latest_markers is None:
            self.get_logger().warn("⚠️ 수신된 '/z_edge_marker' 데이터가 없습니다.")
            return

        # 클릭한 지점의 3D 좌표 (YOLO 결과)
        target_x = msg.point.x
        target_y = msg.point.y
        class_name = msg.header.frame_id
        
        selected_marker = None
        min_dist = float('inf')

        # 1) YOLO 포인트와 가장 가까운 마커 찾기
        for marker in self.latest_markers.markers:
            if marker.action == Marker.DELETEALL or len(marker.points) < 4:
                continue

            # 마커의 중심 좌표 계산 (꼭짓점들의 평균)
            edge_pts_np = np.array([[p.x, p.y] for p in marker.points])
            center_x, center_y = np.mean(edge_pts_np, axis=0)

            # 유클리드 거리 계산
            dist = np.sqrt((target_x - center_x)**2 + (target_y - center_y)**2)

            # 가장 가까운 마커 업데이트 (임계값 이내일 경우만)
            if dist < self.dist_threshold and dist < min_dist:
                min_dist = dist
                selected_marker = marker

        if selected_marker is None:
            self.get_logger().warn(f"❌ 거리 {self.dist_threshold}m 이내에 일치하는 객체가 없습니다. (최단거리: {min_dist:.3f}m)")
            return

        # 2) 선택된 물체의 PCA 기반 각도 추출
        edge_pts_all = np.array([[p.x, p.y, p.z] for p in selected_marker.points])
        if np.linalg.norm(edge_pts_all[0] - edge_pts_all[-1]) < 1e-4:
            edge_pts_all = edge_pts_all[:-1]

        z_height = float(np.mean(edge_pts_all[:, 2]))
        edge_pts_2d = edge_pts_all[:, :2]
        mean_2d = np.mean(edge_pts_2d, axis=0)

        # PCA 연산
        centered_pts = edge_pts_2d - mean_2d
        cov_matrix = np.cov(centered_pts.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

        # 단축 방향(그리퍼 진입 방향)
        minor_axis_idx = np.argmin(eigenvalues)
        minor_axis_vector = eigenvectors[:, minor_axis_idx]

        # 각도 계산 (X축 기준)
        angle_rad = np.arctan2(minor_axis_vector[1], minor_axis_vector[0])
        angle_deg = np.degrees(angle_rad)

        # -90 ~ 90 정규화
        if angle_deg > 90.0: angle_deg -= 180.0
        elif angle_deg < -90.0: angle_deg += 180.0

        # 3) 결과 발행
        angle_msg = Float32()
        angle_msg.data = float(angle_deg)
        self.angle_pub.publish(angle_msg)
        self.get_logger().info(f"🎯 매칭 성공! [{class_name}] | 거리: {min_dist:.3f}m | 각도: {angle_deg:.2f}°")
        
        # 4) 시각화 (/min_length 발행)
        self.publish_visual_markers(selected_marker, mean_2d, minor_axis_vector, z_height)

    def publish_visual_markers(self, selected_marker, mean_2d, minor_axis_vector, z_height):
        # (기존 시각화 로직과 동일)
        marker_array = MarkerArray()
        
        # 이전 마커 삭제
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        # 중심점 (노란 구체)
        c_m = Marker()
        c_m.header = selected_marker.header
        c_m.ns = "match_center"
        c_m.id = 1
        c_m.type = Marker.SPHERE
        c_m.pose.position = Point(x=float(mean_2d[0]), y=float(mean_2d[1]), z=z_height)
        c_m.scale.x = 0.02; c_m.scale.y = 0.02; c_m.scale.z = 0.02
        c_m.color.r = 1.0; c_m.color.g = 1.0; c_m.color.a = 1.0
        marker_array.markers.append(c_m)

        # 관통선 (빨간 선) - 간략화된 버전
        l_m = Marker()
        l_m.header = selected_marker.header
        l_m.ns = "match_axis"
        l_m.id = 2
        l_m.type = Marker.LINE_LIST
        l_m.scale.x = 0.005
        l_m.color.r = 1.0; l_m.color.a = 1.0
        
        p1 = mean_2d + minor_axis_vector * 0.05
        p2 = mean_2d - minor_axis_vector * 0.05
        l_m.points.append(Point(x=p1[0], y=p1[1], z=z_height))
        l_m.points.append(Point(x=p2[0], y=p2[1], z=z_height))
        marker_array.markers.append(l_m)

        self.min_length_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ClickAngleExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
