import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_camera_node')
        
        # 1. YOLO 모델 로드
        self.model = YOLO("./rokey_fruit.pt")
        self.bridge = CvBridge()

        # 💡 [구독] 뎁스 카메라의 컬러 이미지 토픽을 구독합니다.
        # 준수님의 실제 카메라 토픽명으로 변경 가능 (예: '/camera/camera/color/image_raw')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.conf_threshold = 0.25
        self.get_logger().info("🚀 [YOLO 실시간 카메라 노드]가 가동되었습니다. 토픽을 기다리는 중...")

    def image_callback(self, msg):
        try:
            # 1. ROS 2 Image 메시지를 OpenCV(BGR) 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 2. YOLO 추론 실행
            results = self.model.predict(source=cv_image, conf=self.conf_threshold, verbose=False)

            # 3. 바운딩 박스 결과 처리
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 2D 바운딩 박스 좌표 추출 (x_min, y_min, x_max, y_max)
                    coords = box.xyxy[0].cpu().numpy()
                    u_min, v_min, u_max, v_max = map(int, coords)

                    # 🚀 바운딩 박스의 '중심 픽셀 좌표 (u, v)' 계산
                    u_center = (u_min + u_max) // 2
                    v_center = (v_min + v_max) // 2

                    # 클래스 이름과 정확도(Conf) 추출
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = self.model.names[cls_id]
                    conf = float(box.conf[0].cpu().numpy())

                    # 터미널 출력
                    self.get_logger().info(
                        f"🎯 [감지] '{cls_name}' (확률: {conf:.2f}) -> 중심 픽셀 좌표: (u={u_center}, v={v_center})"
                    )

            # (옵션) 감지 결과를 실시간 화면으로 보고 싶다면 아래 주석을 해제하세요.
            # annotated_frame = results[0].plot()
            # cv2.imshow("YOLO Live Detection", annotated_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"이미지 처리 중 에러 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    # 준수님의 실제 모델 경로를 넣어주세요.
    node = YoloCameraNode()
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