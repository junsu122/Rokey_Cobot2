import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from shapely.geometry import Polygon, LineString

class PCAMinAxisNode(Node):
    def __init__(self):
        super().__init__('pca_min_axis_node')

        # 🚀 설정: True면 장축/단축 모두 표시, False면 단축만 표시
        self.DEBUG_MODE = True

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

        self.get_logger().info(f"🚀 [PCA 노드] 가동 (디버그 모드: {self.DEBUG_MODE})")

    def create_axis_marker(self, marker_header, m_id, p1, p2, z, color_rgb, ns):
        """축 시각화를 위한 마커 생성 함수"""
        line_marker = Marker()
        line_marker.header = marker_header
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = ns
        line_marker.id = m_id
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.008 # 두께
        line_marker.color.r = color_rgb[0]
        line_marker.color.g = color_rgb[1]
        line_marker.color.b = color_rgb[2]
        line_marker.color.a = 1.0
        line_marker.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=z))
        line_marker.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=z))
        return line_marker

    def listener_callback(self, msg):
        try:
            min_length_marker_array = MarkerArray()
            min_length_marker_array.markers.append(Marker(action=Marker.DELETEALL))

            valid_marker_id = 0

            for marker in msg.markers:
                if marker.action == Marker.DELETEALL or len(marker.points) < 4:
                    continue

                edge_points_np = np.array([[p.x, p.y, p.z] for p in marker.points])
                if np.linalg.norm(edge_points_np[0] - edge_points_np[-1]) < 1e-4:
                    edge_points_np = edge_points_np[:-1]
                
                z_height = float(np.mean(edge_points_np[:, 2]))
                edge_points_2d = edge_points_np[:, :2]

                # 1) PCA 계산
                mean_2d = np.mean(edge_points_2d, axis=0)
                centered_pts = edge_points_2d - mean_2d
                cov_matrix = np.cov(centered_pts.T)
                eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
                
                # 축 인덱스 분리
                minor_idx = np.argmin(eigenvalues)
                major_idx = np.argmax(eigenvalues)
                
                poly = Polygon(edge_points_2d)
                if not poly.is_valid: poly = poly.buffer(0)

                # 2) 축 관통선 계산 함수
                def get_intersection_points(vector):
                    extent = 1.0
                    test_line = LineString([mean_2d + vector * extent, mean_2d - vector * extent])
                    inter = poly.intersection(test_line)
                    if inter.is_empty: return None
                    line = inter if inter.geom_type == 'LineString' else max(inter.geoms, key=lambda x: x.length)
                    return list(line.coords)[0], list(line.coords)[1]

                # --- 마커 생성 및 추가 ---
                
                # [단축 - 빨간색] 항상 표시
                minor_pts = get_intersection_points(eigenvectors[:, minor_idx])
                if minor_pts:
                    min_length_marker_array.markers.append(
                        self.create_axis_marker(marker.header, valid_marker_id*3, minor_pts[0], minor_pts[1], z_height, [1.0, 0.0, 0.0], "pca_minor")
                    )

                # [장축 - 파란색] DEBUG_MODE가 True일 때만 표시
                if self.DEBUG_MODE:
                    major_pts = get_intersection_points(eigenvectors[:, major_idx])
                    if major_pts:
                        min_length_marker_array.markers.append(
                            self.create_axis_marker(marker.header, valid_marker_id*3 + 1, major_pts[0], major_pts[1], z_height, [0.0, 0.0, 1.0], "pca_major")
                        )

                # [중심점 - 노란색]
                center_marker = Marker(header=marker.header, type=Marker.SPHERE, action=Marker.ADD, ns="pca_center", id=valid_marker_id*3 + 2)
                center_marker.pose.position = Point(x=float(mean_2d[0]), y=float(mean_2d[1]), z=z_height)
                center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.015
                center_marker.color.r = 1.0; center_marker.color.g = 1.0; center_marker.color.a = 1.0
                min_length_marker_array.markers.append(center_marker)

                valid_marker_id += 1

            self.min_length_publisher.publish(min_length_marker_array)

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