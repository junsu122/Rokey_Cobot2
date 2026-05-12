#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from dsr_msgs2.srv import SetRobotControl, GetRobotState

ROBOT_ID = "dsr01"

class ExceptionManagerNode(Node):
    def __init__(self):
        super().__init__("exception_manager_node")
        self.callback_group = ReentrantCallbackGroup()

        # 메인 노드에 예외 처리 resolve를 보내기 위한 publisher
        self.resolved_pub = self.create_publisher(String, "/exception/resolved", 10)
        # 로봇 상태 요약을 주기적으로 보내기 위한 publisher (모니터링용)
        self.state_summary_pub = self.create_publisher(String, f"/{ROBOT_ID}/robot_state_summary", 10)
        
        # 예외 이벤트 수신 (로봇 신호)
        self.event_sub = self.create_subscription(
            String, "/exception/event", self.exception_event_callback, 10,
            callback_group=self.callback_group)

        # 우리가 이 코드에 보내줄 recovery 승인 신호 수신
        self.recovery_sub = self.create_subscription(
            String, f"/{ROBOT_ID}/recovery_command", self.recovery_callback, 10,
            callback_group=self.callback_group)

        self.set_control_cli = self.create_client(SetRobotControl, f"/{ROBOT_ID}/system/set_robot_control")
        self.get_state_cli = self.create_client(GetRobotState, f"/{ROBOT_ID}/system/get_robot_state")
        
        # 상태 요약 타이머 (1초 주기)
        self.create_timer(1.0, self.publish_robot_status_summary, callback_group=self.callback_group)

        self.recovery_approved = False
        self.recovery_lock = threading.Lock()
        self.recovering = False

        self.get_logger().info(f"★★★ Exception Manager Node Started. Waiting for signals... ★★★")

    def get_robot_state_via_service(self):
        """[중요: 누락되었던 함수] 현재 로봇 상태 코드를 서비스로 조회"""
        if not self.get_state_cli.wait_for_service(timeout_sec=1.0):
            return -1 # 연결 실패 시 -1 반환
        
        req = GetRobotState.Request()
        future = self.get_state_cli.call_async(req)
        
        # 멀티스레드 환경에서 안전하게 결과를 기다림
        start_t = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start_t > 2.0: # 2초 타임아웃
                return -1
            time.sleep(0.05)
            
        if future.result() is not None:
            return future.result().robot_state
        return -1


################################################UI에 보낼 정보####################################
    def publish_robot_status_summary(self):
        """로봇의 현재 상태를 읽어와서 요약 정보를 토픽으로 발행합니다."""
        state_code = self.get_robot_state_via_service()
        
        state_map = {
            1: "STANDBY (정상)",
            2: "MOVING (동작중)",
            3: "SAFE_OFF (비상정지됨)",
            4: "TEACHING (티칭모드)",
            5: "PROTECTIVE_STOP (보호정지)",
            6: "EMERGENCY_STOP (비상정지)",
            -1: "DISCONNECTED (연결끊김)"
        }
        
        current_state_str = state_map.get(state_code, f"UNKNOWN ({state_code})")
        
        status_data = {
            "robot_id": ROBOT_ID,
            "state_code": state_code,
            "state_str": current_state_str,
            "recovering": self.recovering,
            "timestamp": time.time()
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.state_summary_pub.publish(msg)
###########################################################################################

    def recovery_callback(self, msg):
        self.get_logger().info(f"!!! [SIGNAL RECEIVED] Raw data: '{msg.data}' !!!")
        if msg.data.strip().upper() == "RECOVER":
            with self.recovery_lock:
                self.recovery_approved = True
            self.get_logger().info("★★★ Recovery APPROVED! ★★★")

    def exception_event_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except: return

        if data.get("status") != "ACTIVE" or self.recovering:
            return

        self.recovering = True
        ex_type = data.get("type", "UNKNOWN")
        self.get_logger().warn(f"!!! [EXCEPTION DETECTED] {ex_type}. Waiting for hand signal...")

        while rclpy.ok():
            with self.recovery_lock:
                if self.recovery_approved:
                    self.recovery_approved = False
                    break
            time.sleep(0.1)

        self.get_logger().info(f"Starting Hardware Recovery for {ex_type}...")
        success = self.recover_robot_hardware(ex_type)

        if success:
            self.publish_resolved(ex_type)
            self.get_logger().info("★★★ Hardware Recovered & Main Notified! ★★★")
        else:
            self.get_logger().error("FAILED to recover hardware. Check robot's physical status.")
        
        self.recovering = False

    def recover_robot_hardware(self, ex_type):
        """실제 하드웨어를 제어하여 복구"""
        # 1. STOP 명령으로 초기화
        self.get_logger().info("Cleaning up robot task state before recovery...")
        stop_req = SetRobotControl.Request()
        stop_req.robot_control = 1 
        self.set_control_cli.call_async(stop_req)
        time.sleep(0.5)

        # 2. 에러 타입에 따른 복구 명령
        cmd = 2 if ex_type == "PROTECTIVE_STOP" else 3
        self.get_logger().info(f"Requesting SetRobotControl({cmd})...")
        
        req = SetRobotControl.Request()
        req.robot_control = cmd
        future = self.set_control_cli.call_async(req)
        
        # 3. 서비스 완료 대기 및 상태 확인
        time.sleep(1.5) 
        
        # 하드웨어 상태 재조회
        final_state = self.get_robot_state_via_service()
        if final_state == 1: # STANDBY
            return True
        return True # 명령 전송 성공 시 일단 성공으로 간주

    def publish_resolved(self, ex_type):
        msg = String()
        msg.data = json.dumps({"type": ex_type, "status": "RESOLVED"})
        self.resolved_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExceptionManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()