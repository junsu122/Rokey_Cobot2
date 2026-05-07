import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from shapely.geometry import Polygon, LineString

class PCAMinAxisNode(Node):
    def __init__(self):
        super().__init__('pca_min_axis_node')

        self.subscription = self.create_subscription(
            MarkerArray,
            '/z_edge_marker',
            self.listener_callback,
            10
        )

        self.min_length_publisher = self.create_publisher(
            MarkerArray,
            '/min_length',
            10
        )

        self.get_logger().info("🚀 [PCA 기반 물체 중심 및 관통선 노드]가 가동되었습니다.")

    def listener_callback(self, msg):
        try:
            min_length_marker_array = MarkerArray()
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            min_length_marker_array.markers.append(clear_marker)

            valid_marker_id = 0

            for marker in msg.markers:
                if marker.action == Marker.DELETEALL or len(marker.points) < 4:
                    continue

                # 엣지 점 수집 (Z값 유지)
                edge_points_np = np.array([[p.x, p.y, p.z] for p in marker.points])
                if np.linalg.norm(edge_points_np[0] - edge_points_np[-1]) < 1e-4:
                    edge_points_np = edge_points_np[:-1]
                
                z_height = float(np.mean(edge_points_np[:, 2]))
                edge_points_2d = edge_points_np[:, :2]

                # 🚀 1) PCA를 통해 물체의 주축과 진짜 형태 중심을 계산
                # 2D 점들의 평균(중심) 구하기
                mean_2d = np.mean(edge_points_2d, axis=0)
                
                # 중심을 0으로 맞춘(Centering) 데이터 생성
                centered_pts = edge_points_2d - mean_2d
                
                # 공분산 행렬 계산 후 고유값(Eigenvalue)과 고유벡터(Eigenvector) 추출
                cov_matrix = np.cov(centered_pts.T)
                eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
                
                # 고유값이 작은 쪽이 '물체의 단축(폭 방향)'입니다.
                minor_axis_idx = np.argmin(eigenvalues)
                minor_axis_vector = eigenvectors[:, minor_axis_idx]

                # 🚀 2) 진짜 형태 중심에서 단축 방향(양방향)으로 광선을 쏘아 엣지와의 교점 계산
                poly = Polygon(edge_points_2d)
                if not poly.is_valid:
                    poly = poly.buffer(0)

                # 물체 크기를 넉넉히 감싸는 가상의 긴 직선 축 생성 (진짜 중심 기준)
                # (중심에서 단축 벡터 방향으로 길게 뻗는 선분)
                extent = 1.0  # 1m
                p1_test = mean_2d + minor_axis_vector * extent
                p2_test = mean_2d - minor_axis_vector * extent
                
                test_line = LineString([p1_test, p2_test])
                
                # 다각형과 이 축이 겹치는 선분(Intersection)만 추출
                intersection = poly.intersection(test_line)

                if not intersection.is_empty:
                    # 교점이 단일 선분으로 나온 경우 그 끝점들을 사용
                    if intersection.geom_type == 'LineString':
                        coords = list(intersection.coords)
                        best_p1, best_p2 = coords[0], coords[1]
                    # 교점이 여러 개로 쪼개진 경우 가장 긴 선분 선택
                    elif intersection.geom_type == 'MultiLineString':
                        longest_line = max(intersection.geoms, key=lambda x: x.length)
                        coords = list(longest_line.coords)
                        best_p1, best_p2 = coords[0], coords[1]
                    else:
                        continue

                    # 3) 단축 관통 선 마커 생성 (LINE_LIST)
                    line_marker = Marker()
                    line_marker.header = marker.header
                    line_marker.header.stamp = self.get_clock().now().to_msg()
                    line_marker.ns = "pca_min_axis"
                    line_marker.id = valid_marker_id * 2
                    line_marker.type = Marker.LINE_LIST
                    line_marker.action = Marker.ADD
                    line_marker.pose.orientation.w = 1.0
                    
                    # 빨간색 관통 선 (두께 8mm)
                    line_marker.scale.x = 0.008
                    line_marker.color.r = 1.0; line_marker.color.g = 0.0; line_marker.color.b = 0.0; line_marker.color.a = 1.0

                    line_marker.points.append(Point(x=float(best_p1[0]), y=float(best_p1[1]), z=z_height))
                    line_marker.points.append(Point(x=float(best_p2[0]), y=float(best_p2[1]), z=z_height))

                    min_length_marker_array.markers.append(line_marker)

                    # 4) 물체의 진짜 중심 시각화 (SPHERE)
                    center_marker = Marker()
                    center_marker.header = marker.header
                    center_marker.header.stamp = self.get_clock().now().to_msg()
                    center_marker.ns = "pca_center"
                    center_marker.id = valid_marker_id * 2 + 1
                    center_marker.type = Marker.SPHERE
                    center_marker.action = Marker.ADD
                    center_marker.pose.position = Point(x=float(mean_2d[0]), y=float(mean_2d[1]), z=z_height)
                    center_marker.pose.orientation.w = 1.0
                    
                    # 노란색 구체
                    center_marker.scale.x = 0.015; center_marker.scale.y = 0.015; center_marker.scale.z = 0.015
                    center_marker.color.r = 1.0; center_marker.color.g = 1.0; center_marker.color.b = 0.0; center_marker.color.a = 1.0

                    min_length_marker_array.markers.append(center_marker)
                    valid_marker_id += 1

            if len(min_length_marker_array.markers) > 1:
                self.min_length_publisher.publish(min_length_marker_array)
                # self.get_logger().info(f"📤 {valid_marker_id}개 물체의 PCA 기반 중심 및 관통선을 발행했습니다.") ## 런치 실행시 제외

        except Exception as e:
            self.get_logger().error(f"Error in PCAMin Node: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PCAMinAxisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()