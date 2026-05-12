import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class AdvancedGroundRemover(Node):
    def __init__(self):
        super().__init__('advanced_ground_remover')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/no_ground',
            10
        )

        self.get_logger().info("🚀 [물체 보호 버전] 바닥 제거 노드가 가동되었습니다.")

        # 💡 조절 가능한 핵심 파라미터
        self.slice_step = 15
        self.plane_distance_thresh = 0.012  # 1. 얇은 평면 판정 (1.2cm로 줄여서 물체 보호)
        
        # 🚀 2. 관심 영역(ROI) 필터링: Y축(아래쪽)의 최소/최대값 지정
        # 카메라 렌즈보다 너무 위거나, 너무 아래에 있는 노이즈를 1차로 쳐냅니다.
        self.min_y = -0.3   # 카메라 위쪽 30cm부터
        self.max_y = 0.5    # 카메라 아래쪽 50cm까지 (바닥 높이에 맞춰 조절)

    def fit_plane_ransac(self, points):
        """가장 큰 바닥 평면을 찾되, 인라이어 기준을 깐깐하게 적용"""
        best_inliers = []
        num_points = len(points)
        if num_points < 3:
            return []

        # 바닥 후보는 주로 Y축 값이 큰(아래쪽) 쪽에 몰려 있으므로, 
        # 아래쪽 점들 위주로 샘플링하여 연산 속도와 정확도를 높입니다.
        y_sorted_indices = np.argsort(points[:, 1])
        bottom_points_indices = y_sorted_indices[int(num_points * 0.5):]  # 하위 50% 점들

        for _ in range(40):  # 반복 횟수
            if len(bottom_points_indices) < 3:
                break
            
            # 아래쪽 영역에서 무작위로 3개 추출
            sample_idx = np.random.choice(bottom_points_indices, 3, replace=False)
            p1, p2, p3 = points[sample_idx]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm == 0:
                continue
            normal = normal / norm
            d = -np.dot(normal, p1)

            # 모든 점과 평면 사이의 거리 계산
            distances = np.abs(np.dot(points, normal) + d)
            inliers = np.where(distances < self.plane_distance_thresh)[0]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        return best_inliers

    def listener_callback(self, msg):
        try:
            points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = [[p['x'], p['y'], p['z']] for p in points_gen]

            if len(points_list) == 0:
                return

            points = np.array(points_list, dtype=np.float32)[::self.slice_step]
            if len(points) < 50:
                return

            # 🚀 [해결 방법 1] Y축(높이) PassThrough 필터링 적용
            # 바닥보다 더 아래에 있는 점들을 원천 차단
            roi_mask = (points[:, 1] >= self.min_y) & (points[:, 1] <= self.max_y)
            filtered_points = points[roi_mask]

            if len(filtered_points) < 3:
                return

            # RANSAC으로 바닥 인덱스 찾기
            ground_inliers = self.fit_plane_ransac(filtered_points)

            # 🚀 [해결 방법 2] 바닥으로 판정된 점들을 제거
            all_indices = np.arange(len(filtered_points))
            object_indices = np.setdiff1d(all_indices, ground_inliers)

            if len(object_indices) == 0:
                return

            valid_points = filtered_points[object_indices]

            # 퍼블리시
            out_header = msg.header
            no_ground_msg = pc2.create_cloud_xyz32(out_header, valid_points.tolist())
            self.publisher.publish(no_ground_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedGroundRemover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()