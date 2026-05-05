import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import DR_init

# 홈 포지션 설정
HOME_POS = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

class SimpleJ6Control(Node):
    def __init__(self):
        super().__init__('simple_j6_control')
        
        # 3. 함수 안에서 임포트하여 노드 할당 후 로드되게 함
        from DSR_ROBOT2 import movej
        
        self.get_logger().info("🏠 Moving to Home...")
        movej(HOME_POS, vel=60, acc=30)
        
        self.subscription = self.create_subscription(
            Float32,
            '/object_angle',
            self.listener_callback,
            10)
        self.get_logger().info("✅ Ready! Waiting for J6 angle...")



########################################################### 나중에 추가 부분############################################################
    def listener_callback(self, msg):
        # 콜백 안에서도 필요한 함수 임포트
        from DSR_ROBOT2 import movej, get_current_posj
        
        target_angle = msg.data
        self.get_logger().info(f"📥 Received: {target_angle:.2f}")

        current_pos = get_current_posj()
        new_joint_pos = [
            current_pos[0], current_pos[1], current_pos[2], 
            current_pos[3], target_angle,  current_pos[5] 
        ]

        movej(new_joint_pos, vel=60, acc=30)
#####################################################################################################################################

def main(args=None):
    rclpy.init(args=args)
    
    # 1. 노드 먼저 생성 및 설정
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"
    node = rclpy.create_node("j6_mover", namespace=ROBOT_ID)

    # 2. DR_init에 노드 정보 먼저 주입
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    simple_node = SimpleJ6Control()

    try:
        rclpy.spin(simple_node)
    except KeyboardInterrupt:
        from DSR_ROBOT2 import movej
        simple_node.get_logger().info("🔙 Returning to Home...")
        movej(HOME_POS, vel=60, acc=30)
    finally:
        simple_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()