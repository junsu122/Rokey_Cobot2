import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# 🚀 ConvexHull 대신 Concave Hull을 만들어주는 라이브러리
import alphashape

class ZNormalizationEdgeNode(Node):
    def __init__(self):
        super().__init__('z_normalization_edge_node')

        # 1. /no_ground 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/no_ground',
            self.listener_callback,
            10
        )

        # 2. 평면화된 포인트 클라우드 발행용
        self.publisher = self.create_publisher(
            PointCloud2,
            '/z_normalization',
            10
        )

        # 3. 추출된 객체 수 발행용
        self.count_publisher = self.create_publisher(
            Int32,
            '/object_count',
            10
        )

        # 4. 객체별 외곽선(Edge) 마커 발행용
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/z_edge_marker',
            10
        )

        self.get_logger().info("🚀 [Z-평면화 및 Concave Hull Edge 노드]가 가동되었습니다.")

        # 💡 [중요] 군집화 파라미터 (eps를 0.05m = 5cm로 정상 복구합니다)
        self.eps = 0.01             
        self.min_cluster_size = 5   

        # 💡 곡선(Concave)을 얼마나 정교하게 딸지 결정하는 파라미터
        # 이 값이 클수록 더 굴곡진 선을 따고, 너무 크면 선이 끊어집니다. (15~25가 적당)
        self.alpha = 20.0

    def extract_clusters(self, points):
        visited = np.zeros(len(points), dtype=bool)
        clusters = []

        for i in range(len(points)):
            if visited[i]:
                continue
            
            cluster = []
            queue = [i]
            visited[i] = True

            while queue:
                curr = queue.pop(0)
                cluster.append(curr)

                diff = points - points[curr]
                dist_sq = np.sum(diff**2, axis=1)
                
                neighbors = np.where((dist_sq < self.eps**2) & (~visited))[0]
                
                for neighbor in neighbors:
                    visited[neighbor] = True
                    queue.append(neighbor)

            clusters.append(cluster)

        return clusters

    def listener_callback(self, msg):
        try:
            # A. /no_ground에서 점 데이터 추출
            points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = [[p['x'], p['y'], p['z']] for p in points_gen]

            if len(points_list) == 0:
                return

            points = np.array(points_list, dtype=np.float32)

            # B. 들어온 점들을 객체별로 군집화
            clusters = self.extract_clusters(points)

            normalized_points = []
            valid_cluster_count = 0

            # RViz2에 발행할 마커 배열 생성
            marker_array = MarkerArray()
            
            # 기존 화면 마커들을 초기화(삭제)하여 잔상 방지
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)

            # C. 각 군집 처리
            for cluster_idx, cluster in enumerate(clusters):
                if len(cluster) >= self.min_cluster_size:
                    valid_cluster_count += 1
                    cluster_pts = points[cluster]
                    
                    # 1) Z값 평균 평면화
                    mean_z = float(np.mean(cluster_pts[:, 2]))
                    cluster_pts[:, 2] = mean_z
                    
                    # 평면화 포인트 수집
                    for pt in cluster_pts:
                        normalized_points.append(pt)

                    # 2) 🚀 [핵심 교체] ConvexHull 대신 alphashape(Concave) 사용
                    if len(cluster_pts) >= 4:
                        try:
                            # X, Y 평면 좌표만 추출
                            points_2d = cluster_pts[:, :2]
                            
                            # Concave Hull 다각형 생성
                            alpha_shape = alphashape.alphashape(points_2d, self.alpha)
                            
                            # 외곽선 점 추출
                            if alpha_shape.geom_type == 'Polygon':
                                # 단일 다각형인 경우
                                x_coords, y_coords = alpha_shape.exterior.coords.xy
                            elif alpha_shape.geom_type == 'MultiPolygon':
                                # 여러 개로 쪼개진 경우 가장 큰 것 선택
                                largest_poly = max(alpha_shape.geoms, key=lambda a: a.area)
                                x_coords, y_coords = largest_poly.exterior.coords.xy
                            else:
                                continue

                            # 3) Line Strip 마커 생성
                            marker = Marker()
                            marker.header = msg.header
                            marker.header.stamp = self.get_clock().now().to_msg() # 시간 동기화
                            marker.ns = "object_edges"
                            marker.id = cluster_idx
                            marker.type = Marker.LINE_STRIP
                            marker.action = Marker.ADD
                            
                            marker.pose.orientation.w = 1.0
                            
                            # 선 속성 설정 (두께 8mm, 선명한 초록색 선)
                            marker.scale.x = 0.004  
                            marker.color.r = 0.0
                            marker.color.g = 1.0
                            marker.color.b = 0.0
                            marker.color.a = 1.0

                            # 추출된 엣지 포인트들을 순서대로 마커에 담기
                            for cx, cy in zip(x_coords, y_coords):
                                marker.points.append(Point(x=float(cx), y=float(cy), z=mean_z))

                            marker_array.markers.append(marker)

                        except Exception as e:
                            # 간혹 alpha 값이 맞지 않아 생기는 에러 방지
                            continue

            # D. 데이터 발행
            # 1) 객체 수 발행
            count_msg = Int32()
            count_msg.data = valid_cluster_count
            self.count_publisher.publish(count_msg)

            # 2) 포인트 클라우드 발행
            if len(normalized_points) > 0:
                valid_points = np.array(normalized_points, dtype=np.float32)
                z_norm_msg = pc2.create_cloud_xyz32(msg.header, valid_points.tolist())
                self.publisher.publish(z_norm_msg)

            # 3) 엣지 마커 발행
            if len(marker_array.markers) > 1:
                self.marker_publisher.publish(marker_array)

            self.get_logger().info(f"🎯 [완료] 객체: {valid_cluster_count}개 감지 및 정밀 곡선 엣지 생성 완료.")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ZNormalizationEdgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()