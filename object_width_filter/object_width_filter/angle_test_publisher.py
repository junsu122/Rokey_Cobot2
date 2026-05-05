import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys

class AnglePublisher(Node):
    def __init__(self, input_angle):
        super().__init__('angle_single_publisher')
        
        # 퍼블리셔 설정
        self.publisher_ = self.create_publisher(
            Float32,
            '/object_angle',
            10)
        
        # 입력받은 각도 저장
        self.angle_to_pub = float(input_angle)
        
        # 노드 생성 후 즉시 실행될 함수 호출
        self.publish_once()

    def publish_once(self):
        msg = Float32()
        msg.data = self.angle_to_pub
        
        # 메시지 발행
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published angle: {msg.data} to /object_angle')
        
        # 발행 후 노드가 바로 종료될 수 있도록 약간의 여유를 둡니다.

def main(args=None):
    rclpy.init(args=args)

    # 실행 시 터미널에서 입력을 받거나 인자로 전달받음
    if len(sys.argv) > 1:
        user_angle = sys.argv[1]
    else:
        user_angle = input("발행할 각도를 입력하세요: ")

    try:
        node = AnglePublisher(user_angle)
        # 한 번만 발행하고 종료하기 위해 spin_once를 사용하거나 짧게 실행
        rclpy.spin_once(node, timeout_sec=1.0)
    except ValueError:
        print("숫자 형식으로 각도를 입력해주세요.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()