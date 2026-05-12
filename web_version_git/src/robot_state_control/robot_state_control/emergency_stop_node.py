#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 실제 로봇 상태 서비스를 위한 타입 추가
from dsr_msgs2.srv import GetRobotState

ROBOT_ID = "dsr01"

STATE_ROBOT_STANDBY = 1
STATE_ROBOT_SAFE_OFF = 3
STATE_ROBOT_PROTECTIVE_STOP = 5
STATE_ROBOT_EMERGENCY_STOP = 6

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__("emergency_stop_node")

        # 1. 퍼블리셔: 예외 발생 알림
        self.event_pub = self.create_publisher(String, "/exception/event", 10)

        # 2. 서비스 클라이언트: 로봇 상태 조회용
        self.get_state_cli = self.create_client(GetRobotState, f"/{ROBOT_ID}/system/get_robot_state")
        
        self.last_stop_code = None

        # 0.3초 주기로 상태 모니터링
        self.timer = self.create_timer(0.3, self.monitor_robot_state)
        self.get_logger().info("emergency_stop_node started (Service Client Mode)")

    def monitor_robot_state(self):
        # 서비스 서버가 준비되었는지 확인
        if not self.get_state_cli.wait_for_service(timeout_sec=0.1):
            # 아직 서비스가 안 올라왔으면 스킵
            return

        # 서비스 호출
        req = GetRobotState.Request()
        future = self.get_state_cli.call_async(req)
        
        # 결과를 기다리기 위해 별도의 콜백이나 처리를 하지 않고 
        # 다음 타이머 루프에서 완료 여부를 확인하거나, 간단하게 수신
        future.add_done_callback(self.state_response_callback)

    def state_response_callback(self, future):
        try:
            response = future.result()
            hw_code = response.robot_state
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")
            return

        # 비상/보호 정지 상태인지 확인
        if hw_code in [STATE_ROBOT_EMERGENCY_STOP, STATE_ROBOT_PROTECTIVE_STOP, STATE_ROBOT_SAFE_OFF]:
            if self.last_stop_code != hw_code:
                self.publish_event(hw_code)
                self.last_stop_code = hw_code
        else:
            # 정상 상태로 돌아오면 코드 초기화
            self.last_stop_code = None

    def publish_event(self, code):
        type_map = {
            STATE_ROBOT_SAFE_OFF: "SAFE_OFF",
            STATE_ROBOT_PROTECTIVE_STOP: "PROTECTIVE_STOP",
            STATE_ROBOT_EMERGENCY_STOP: "EMERGENCY_STOP"
        }
        
        ex_type = type_map.get(code, "UNKNOWN_STOP")
        
        msg = String()
        msg.data = json.dumps({
            "type": ex_type,
            "status": "ACTIVE",
            "hw_code": code
        })
        self.event_pub.publish(msg)
        self.get_logger().error(f"★★★ EXCEPTION DETECTED: {ex_type} (code: {code}) ★★★")

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import json
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String


# class EmergencyStopNode(Node):
#     def __init__(self):
#         super().__init__('emergency_stop_node')

#         self.emergency_pub = self.create_publisher(
#             String,
#             '/exception/event',
#             10
#         )

#         self.resolved_pub = self.create_publisher(
#             String,
#             '/exception/resolved',
#             10
#         )

#         self.trigger_sub = self.create_subscription(
#             String,
#             '/emergency/trigger',
#             self.trigger_callback,
#             10
#         )

#         self.reset_sub = self.create_subscription(
#             String,
#             '/emergency/reset',
#             self.reset_callback,
#             10
#         )

#         self.is_emergency = False

#         self.get_logger().info('emergency_stop_node started')

#     def trigger_callback(self, msg):
#         if self.is_emergency:
#             self.get_logger().warn('Emergency already active')
#             return

#         self.is_emergency = True

#         out = String()
#         out.data = json.dumps({
#             'type': 'EMERGENCY_STOP',
#             'status': 'ACTIVE'
#         })

#         self.emergency_pub.publish(out)
#         self.get_logger().error(f'publish /exception/event: {out.data}')

#     def reset_callback(self, msg):
#         if not self.is_emergency:
#             self.get_logger().warn('Emergency is not active')
#             return

#         self.is_emergency = False

#         out = String()
#         out.data = json.dumps({
#             'type': 'EMERGENCY_STOP',
#             'status': 'RESOLVED'
#         })

#         self.resolved_pub.publish(out)
#         self.get_logger().info(f'publish /exception/resolved: {out.data}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = EmergencyStopNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()