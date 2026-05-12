#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MainNode(Node):
    def __init__(self):
        super().__init__("main_node")

        self.system_state = "BASIC_RUNNING"
        self.active_exception = None

        self.basic_stage = 'UNKNOWN'
        self.basic_paused = False  
        self.basic_stopped = False

        self.basic_cmd_pub = self.create_publisher(
            String,
            "/main/basic_command",
            10
        )

        self.system_state_pub = self.create_publisher(
            String,
            '/main/system_state',
            10
        )

        self.exception_sub = self.create_subscription(
            String,
            "/exception/event",
            self.exception_callback,
            10
        )

        self.resolved_sub = self.create_subscription(
            String,
            "/exception/resolved",
            self.resolved_callback,
            10
        )

        self.control_state_sub = self.create_subscription(
            String,
            '/dsr01/state',
            self.control_state_callback,
            10
        )


        self.get_logger().info("main_node started")

        self.send_basic_command("START")
        self.publish_system_state()

    def exception_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON parse error: {e}")
            return

        exception_type = data.get("type", "UNKNOWN")
        status = data.get("status", "ACTIVE")
        hw_code = data.get("hw_code", -1)

        if status != "ACTIVE":
            return

        self.active_exception = exception_type

        if exception_type == "EMERGENCY_STOP":
            self.system_state = "EMERGENCY_STOPPED"
            self.send_basic_command("STOP")
            self.publish_system_state()

            self.get_logger().error(
                f"[MAIN] {exception_type}, hw_code={hw_code} → BASIC STOP"
            )

        else:
            self.system_state = "PAUSED_BY_EXCEPTION"
            self.send_basic_command("PAUSE")
            self.publish_system_state()

            self.get_logger().warn(
                f"[MAIN] {exception_type}, hw_code={hw_code} → BASIC PAUSE"
            )

    def resolved_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON parse error: {e}")
            return

        exception_type = data.get("type", "UNKNOWN")
        status = data.get("status", "RESOLVED")

        if status != "RESOLVED":
            return

        self.get_logger().info(
            f"[MAIN] RESOLVED: {exception_type}"
        )

        self.active_exception = None
        

        self.system_state = "BASIC_RUNNING"

        self.send_basic_command("RESUME")
        self.publish_system_state()

        self.get_logger().info(
            "[MAIN] BASIC RESUME"
        )
    def control_state_callback(self, msg):

        try:
            data = json.loads(msg.data)

        except Exception as e:
            self.get_logger().error(
                f'Control state JSON parse error: {e}'
            )
            return

        self.basic_stage = data.get('stage', 'UNKNOWN')
        self.basic_paused = bool(data.get('paused', False))
        self.basic_stopped = bool(data.get('stopped', False))

        self.get_logger().info(
            f'[BASIC STATE] '
            f'stage={self.basic_stage}, '
            f'paused={self.basic_paused}, '
            f'stopped={self.basic_stopped}'
        )
        self.publish_system_state()


    def publish_system_state(self):

        msg = String()

        msg.data = json.dumps({

            'system_state': self.system_state,

            'active_exception': self.active_exception,

            'basic_stage': self.basic_stage,

            'basic_paused': self.basic_paused,

            'basic_stopped': self.basic_stopped
        })

        self.system_state_pub.publish(msg)

        self.get_logger().info(
            f'publish /main/system_state: {msg.data}'
        )

    def send_basic_command(self, command):
        msg = String()
        msg.data = json.dumps({
            "command": command,
            "state": self.system_state,
            "active_exception": self.active_exception
        })

        self.basic_cmd_pub.publish(msg)

        self.get_logger().info(
            f"publish /main/basic_command: {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

