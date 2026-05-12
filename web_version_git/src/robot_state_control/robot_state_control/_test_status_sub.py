#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber_node')
        
        # 새로운 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/dsr01/robot_state_summary',
            self.listener_callback,
            10
        )
        self.get_logger().info("Robot Status Monitoring Started...")

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            state_str = data.get('state_str')
            recovering = "YES" if data.get('recovering') else "NO"
            
            # 터미널에 보기 좋게 로그 출력
            print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                  f"상태: {state_str:20} | 복구진행중: {recovering}")
            
        except Exception as e:
            self.get_logger().error(f"Error parsing status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()