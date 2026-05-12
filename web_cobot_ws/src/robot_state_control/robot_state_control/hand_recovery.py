#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import time

class HandRecoveryNode(Node):
    def __init__(self):
        super().__init__('hand_recovery_node')
        
        # Exception Manager가 기다리는 토픽
        self.recovery_pub = self.create_publisher(String, '/dsr01/recovery_command', 10)
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        self.detection_start_time = None
        self.recovery_triggered = False
        self.required_duration = 2.0  # 요청하신 대로 2초로 변경
        
        # 버튼 영역 설정 (오른쪽 하단)
        self.btn_x1, self.btn_y1 = 450, 350
        self.btn_x2, self.btn_y2 = 620, 460
        
        self.cap = cv2.VideoCapture(0)
        self.create_timer(0.033, self.process_webcam)
        
        self.get_logger().info("Virtual Button Node Started. Put your finger in the RED BOX for 2s.")

    def process_webcam(self):
        success, img = self.cap.read()
        if not success: return
        
        # 좌우 반전 (거울 모드)
        img = cv2.flip(img, 1)
        h, w, _ = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        # 1. 가상 버튼 영역 그리기 (기본 빨간색)
        btn_color = (0, 0, 255) 
        is_inside = False

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 검지 손가락 끝(INDEX_FINGER_TIP) 좌표 추출
                index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                ix, iy = int(index_tip.x * w), int(index_tip.y * h)
                
                # 손가락 끝에 작은 점 그리기
                cv2.circle(img, (ix, iy), 10, (255, 255, 0), -1)

                # 2. 영역 안에 손가락 끝이 있는지 체크
                if self.btn_x1 < ix < self.btn_x2 and self.btn_y1 < iy < self.btn_y2:
                    is_inside = True
                    btn_color = (0, 255, 0) # 안에 있으면 초록색으로 변경

        # 3. 로직 처리
        if is_inside:
            if self.detection_start_time is None:
                self.detection_start_time = time.time()
            
            elapsed = time.time() - self.detection_start_time
            remaining = max(0, self.required_duration - elapsed)

            # 버튼 영역에 게이지 표시
            cv2.putText(img, f"{remaining:.1f}s", (self.btn_x1+10, self.btn_y1+40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            if elapsed >= self.required_duration and not self.recovery_triggered:
                self.send_recovery_command()
                self.recovery_triggered = True
        else:
            self.detection_start_time = None
            self.recovery_triggered = False

        # 가상 버튼 렌더링
        cv2.rectangle(img, (self.btn_x1, self.btn_y1), (self.btn_x2, self.btn_y2), btn_color, 3)
        cv2.putText(img, "RECOVERY", (self.btn_x1, self.btn_y1-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, btn_color, 2)

        # cv2.imshow("Virtual Recovery Button", img)
        # cv2.waitKey(1)

    def send_recovery_command(self):
        msg = String()
        msg.data = "RECOVER"
        self.recovery_pub.publish(msg)
        self.get_logger().info("★★★ Virtual Button Triggered: RECOVER sent! ★★★")
        
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandRecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()