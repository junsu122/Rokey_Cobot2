# import rclpy
# from rclpy.node import Node
# import numpy as np
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point, PointStamped
# from std_msgs.msg import Float32
# from shapely.geometry import Polygon

# class ClickAngleExtractorNode(Node):
#     def __init__(self):
#         super().__init__('click_angle_extractor_node')

#         # 1. Perception 노드에서 발행하는 엣지 마커 구독
#         self.marker_sub = self.create_subscription(
#             MarkerArray,
#             '/z_edge_marker',
#             self.marker_callback,
#             10
#         )

#         # 2. YOLO 노드에서 클릭 시 발행하는 3D 좌표 구독
#         # (토픽명은 준수님이 적어주신 '_2d_to_3d_point'로 맞췄습니다)
#         self.point_sub = self.create_subscription(
#             PointStamped,
#             '_2d_to_3d_point',
#             self.click_callback,
#             10
#         )

#         # 3. 결과 시각화 및 각도 발행용 Publisher
#         self.min_length_pub = self.create_publisher(MarkerArray, '/min_length', 10)
#         self.angle_pub = self.create_publisher(Float32, '/object_angle', 10)

#         self.latest_markers = None
#         self.dist_threshold = 0.1  # 🚀 동일 객체로 판단할 거리 임계값 (10cm)

#         self.get_logger().info("🚀 [좌표 비교형 각도 추출 노드] 가동되었습니다.")

#     def marker_callback(self, msg):
#         self.latest_markers = msg

#     def click_callback(self, msg):
#         if self.latest_markers is None:
#             self.get_logger().warn("⚠️ 수신된 '/z_edge_marker' 데이터가 없습니다.")
#             return

#         # 클릭한 지점의 3D 좌표 (YOLO 결과)
#         target_x = msg.point.x
#         target_y = msg.point.y
#         class_name = msg.header.frame_id
        
#         selected_marker = None
#         min_dist = float('inf')

#         # 1) YOLO 포인트와 가장 가까운 마커 찾기
#         for marker in self.latest_markers.markers:
#             if marker.action == Marker.DELETEALL or len(marker.points) < 4:
#                 continue

#             # 마커의 중심 좌표 계산 (꼭짓점들의 평균)
#             edge_pts_np = np.array([[p.x, p.y] for p in marker.points])
#             center_x, center_y = np.mean(edge_pts_np, axis=0)

#             # 유클리드 거리 계산
#             dist = np.sqrt((target_x - center_x)**2 + (target_y - center_y)**2)

#             # 가장 가까운 마커 업데이트 (임계값 이내일 경우만)
#             if dist < self.dist_threshold and dist < min_dist:
#                 min_dist = dist
#                 selected_marker = marker

#         if selected_marker is None:
#             self.get_logger().warn(f"❌ 거리 {self.dist_threshold}m 이내에 일치하는 객체가 없습니다. (최단거리: {min_dist:.3f}m)")
#             return

#         # 2) 선택된 물체의 PCA 기반 각도 추출
#         edge_pts_all = np.array([[p.x, p.y, p.z] for p in selected_marker.points])
#         if np.linalg.norm(edge_pts_all[0] - edge_pts_all[-1]) < 1e-4:
#             edge_pts_all = edge_pts_all[:-1]

#         z_height = float(np.mean(edge_pts_all[:, 2]))
#         edge_pts_2d = edge_pts_all[:, :2]
#         mean_2d = np.mean(edge_pts_2d, axis=0)

#         # PCA 연산
#         centered_pts = edge_pts_2d - mean_2d
#         cov_matrix = np.cov(centered_pts.T)
#         eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

#         # 단축 방향(그리퍼 진입 방향)
#         minor_axis_idx = np.argmin(eigenvalues)
#         minor_axis_vector = eigenvectors[:, minor_axis_idx]

#         # 각도 계산 (X축 기준)
#         angle_rad = np.arctan2(minor_axis_vector[1], minor_axis_vector[0])
#         angle_deg = np.degrees(angle_rad)

#         # -90 ~ 90 정규화
#         if angle_deg > 90.0: angle_deg -= 180.0
#         elif angle_deg < -90.0: angle_deg += 180.0

#         # 3) 결과 발행
#         angle_msg = Float32()
#         angle_msg.data = float(-angle_deg)
#         self.angle_pub.publish(angle_msg)
#         self.get_logger().info(f"🎯 매칭 성공! [{class_name}] | 거리: {min_dist:.3f}m | 각도: {-angle_deg:.2f}°")
        
#         # 4) 시각화 (/min_length 발행)
#         self.publish_visual_markers(selected_marker, mean_2d, minor_axis_vector, z_height)

#     def publish_visual_markers(self, selected_marker, mean_2d, minor_axis_vector, z_height):
#         # (기존 시각화 로직과 동일)
#         marker_array = MarkerArray()
        
#         # 이전 마커 삭제
#         del_marker = Marker()
#         del_marker.action = Marker.DELETEALL
#         marker_array.markers.append(del_marker)

#         # 중심점 (노란 구체)
#         c_m = Marker()
#         c_m.header = selected_marker.header
#         c_m.ns = "match_center"
#         c_m.id = 1
#         c_m.type = Marker.SPHERE
#         c_m.pose.position = Point(x=float(mean_2d[0]), y=float(mean_2d[1]), z=z_height)
#         c_m.scale.x = 0.02; c_m.scale.y = 0.02; c_m.scale.z = 0.02
#         c_m.color.r = 1.0; c_m.color.g = 1.0; c_m.color.a = 1.0
#         marker_array.markers.append(c_m)

#         # 관통선 (빨간 선) - 간략화된 버전
#         l_m = Marker()
#         l_m.header = selected_marker.header
#         l_m.ns = "match_axis"
#         l_m.id = 2
#         l_m.type = Marker.LINE_LIST
#         l_m.scale.x = 0.005
#         l_m.color.r = 1.0; l_m.color.a = 1.0
        
#         p1 = mean_2d + minor_axis_vector * 0.05
#         p2 = mean_2d - minor_axis_vector * 0.05
#         l_m.points.append(Point(x=p1[0], y=p1[1], z=z_height))
#         l_m.points.append(Point(x=p2[0], y=p2[1], z=z_height))
#         marker_array.markers.append(l_m)

#         self.min_length_pub.publish(marker_array)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ClickAngleExtractorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32, Float32MultiArray

class ClickAngleExtractorNode(Node):
    def __init__(self):
        super().__init__('click_angle_extractor_node')

        self.marker_sub = self.create_subscription(MarkerArray, '/z_edge_marker', self.marker_callback, 10)
        self.point_sub = self.create_subscription(PointStamped, '_2d_to_3d_point', self.click_callback, 10)
        
        # 🚀 Depth 데이터 구독 추가
        self.depth_sub = self.create_subscription(Float32MultiArray, '/object_depth_data', self.depth_callback, 10)
        
        self.angle_pub = self.create_publisher(Float32, '/object_angle', 10)
        self.min_length_pub = self.create_publisher(MarkerArray, '/min_length', 10)

        self.latest_markers = None
        self.current_depth_data = None
        self.dist_threshold = 0.2  # 🚀 매칭 확률을 높이기 위해 10cm에서 20cm로 확장
        self.max_samples = 5
        self.angle_buffer = []
        self.is_collecting = False

        self.get_logger().info("🚀 [각도 및 Depth 통합 노드] 가동")

    def depth_callback(self, msg):
        self.current_depth_data = msg.data

    def click_callback(self, msg):
        self.target_pos = (msg.point.x, msg.point.y)
        self.angle_buffer = []
        self.is_collecting = True
        self.get_logger().info(f"🎯 [{msg.header.frame_id}] 수집 시작...")

    def marker_callback(self, msg):
        self.latest_markers = msg
        if not self.is_collecting or len(self.angle_buffer) >= self.max_samples: return

        res = self.calculate_current_angle(self.target_pos[0], self.target_pos[1])
        if res:
            self.angle_buffer.append(res['angle'])
            self.get_logger().info(f"📥 샘플 수집 [{len(self.angle_buffer)}/5]: {res['angle']:.2f}°")
            self.publish_visual_markers(res['marker'], res['mean'], res['vector'], res['z'])

            if len(self.angle_buffer) >= self.max_samples:
                self.is_collecting = False
                self.process_result()

    def process_result(self):
        if not self.angle_buffer:
            self.get_logger().warn("⚠️ 수집된 샘플이 없어 계산을 취소합니다.")
            return

        # 1. 이상치 제거 (중앙값 기반 Median Absolute Deviation 활용)
        # 샘플이 5개로 적으므로, 중앙값과 너무 먼 값은 필터링합니다.
        samples = np.array(self.angle_buffer)
        median = np.median(samples)
        diff = np.abs(samples - median)
        med_abs_deviation = np.median(diff)
        
        # 편차가 있는 경우에만 필터링 (모든 값이 같으면 제외)
        if med_abs_deviation > 0:
            threshold = 2.0  # 감도 조절
            filtered_samples = samples[diff < threshold * med_abs_deviation]
        else:
            filtered_samples = samples

        if len(filtered_samples) == 0:
            filtered_samples = samples

        # 2. 원형 통계 적용 (0~180도 대응을 위해 2배각 사용)
        # 각도를 2배로 불려 0~360도 공간에서 벡터 합산 후 다시 절반으로 나눕니다.
        angles_rad = np.radians(filtered_samples * 2.0)
        
        cos_avg = np.mean(np.cos(angles_rad))
        sin_avg = np.mean(np.sin(angles_rad))
        
        # 평균 벡터의 각도 계산 (라디안)
        avg_rad = np.arctan2(sin_avg, cos_avg)
        
        # 다시 0~180도 범위로 복원
        final_angle = np.degrees(avg_rad) / 2.0
        
        # 결과값을 0~180도 사이로 정규화 (음수 방지)
        final_angle = final_angle % 180.0

        # 3. 로그 출력 및 발행
        self.get_logger().info("--------------------------------------------------")
        self.get_logger().info(f"📊 원본 샘플: {[round(a, 1) for a in samples]}")
        self.get_logger().info(f"✅ 필터링 후 샘플: {[round(a, 1) for a in filtered_samples]}")
        self.get_logger().info(f"🎯 최종 계산 각도 (원형 평균): {final_angle:.2f}°")
        
        if self.current_depth_data:
            h, d_m, t_m = self.current_depth_data
            self.get_logger().info(f"📐 Depth 정보: 차이 {h:.1f}mm | 물체Z {d_m:.3f}m")
        self.get_logger().info("--------------------------------------------------")

        msg = Float32()
        msg.data = float(final_angle)
        self.angle_pub.publish(msg)

    def calculate_current_angle(self, tx, ty):
        if not self.latest_markers: return None
        sel = None; m_dist = float('inf')
        for m in self.latest_markers.markers:
            if m.action == Marker.DELETEALL or len(m.points) < 4: continue
            pts = np.array([[p.x, p.y] for p in m.points])
            center = np.mean(pts, axis=0)
            dist = np.sqrt((tx - center[0])**2 + (ty - center[1])**2)
            if dist < self.dist_threshold and dist < m_dist:
                m_dist = dist; sel = m
        
        if sel:
            pts_all = np.array([[p.x, p.y, p.z] for p in sel.points])
            mean_2d = np.mean(pts_all[:, :2], axis=0)
            cov = np.cov(pts_all[:, :2] - mean_2d, rowvar=False)
            evals, evecs = np.linalg.eig(cov)
            vec = evecs[:, np.argmin(evals)]
            deg = np.degrees(np.arctan2(vec[1], vec[0]))
            if deg > 90.0: deg -= 180.0
            elif deg < -90.0: deg += 180.0
            return {'angle': float(deg), 'marker': sel, 'mean': mean_2d, 'vector': vec, 'z': float(np.mean(pts_all[:, 2]))}
        return None

    def publish_visual_markers(self, sel, mean, vec, z):
        ma = MarkerArray(); dm = Marker(); dm.action = Marker.DELETEALL; ma.markers.append(dm)
        # (중심점 및 축 시각화 로직 동일...)
        self.min_length_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = ClickAngleExtractorNode(); rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()