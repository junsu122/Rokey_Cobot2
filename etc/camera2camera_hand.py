import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
from ultralytics import YOLO
import time
import torch
import threading
import DR_init
# 멀티스레드 실행을 위해 Executor 추가
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# =========================
# ROBOT CONFIG (main_controller 구조 따름)
# =========================
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 속도/가속도
HOME_V_J = 300;  HOME_ACC_J = 200
TASK_V_L = 300;  TASK_A_L   = 200

# 홈 관절 좌표
HOME_JReady = [19.20, -6.90, 86.79, 0.07, 100.94, 13.81]

# Z 접근 오프셋 (mm)
Z_APPROACH_OFFSET = 50


class FruitRobotNode(Node):
    def __init__(self):
        super().__init__('fruit_robot_node', namespace=ROBOT_ID)
        self.bridge = CvBridge()
        
        # ─── 멀티스레드 콜백 그룹 설정 (에러 방지 핵심) ───
        self.callback_group = ReentrantCallbackGroup()

        # ─── YOLO ───
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"🚀 YOLO device: {device}")
        self.model = YOLO("/home/rokey/junsu/rokey_fruit.pt").to(device)

        # ─── 카메라 구독 (callback_group 추가) ───
        self.create_subscription(Image, '/camera1/image_raw', self.cam1_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.cam2_callback, 10, callback_group=self.callback_group)

        # ─── MediaPipe ───
        self.mp_hands = mp.solutions.hands
        self.mp_draw  = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False, max_num_hands=1,
            min_detection_confidence=0.7, min_tracking_confidence=0.5
        )

        # ─── 상태 변수 ───
        self.latest_hand_landmarks = None
        self.grip_state      = "Open"
        self.prev_grip_state = "Open"

        self.hover_start_time = None
        self.hover_class_name = None
        self.selected_logged  = False

        self.action_state  = "IDLE"
        self.gripped_class = None

        self.robot_busy = False
        self.robot_lock = threading.Lock()

        # ─── 좌표 매핑 ───
        self.apple_pose = [281.25, 165.12, 304.82, 88.38, 177.95, 87.79]

        self.class_to_robot_pose = {
            "apple":  self.apple_pose,
            "orange": [400.0,    0.0, 200.0, 0.0, 180.0, 0.0],
            "banana": [400.0,  100.0, 200.0, 0.0, 180.0, 0.0],
        }

        # Apple의 경우 놓는 위치도 같은 곳으로 요청하셨으므로 동일하게 설정
        self.class_to_place_pose = {
            "apple":  self.apple_pose,
            "orange": [300.0,    0.0, 200.0, 0.0, 180.0, 0.0],
            "banana": [300.0,  200.0, 200.0, 0.0, 180.0, 0.0],
        }

        self.get_logger().info("✅ FruitRobotNode 준비 완료")

    def initialize_robot(self):
        from DSR_ROBOT2 import (
            set_tool, set_tcp, set_ref_coord,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS
        )
        set_ref_coord(0)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        self.get_logger().info("🤖 로봇 초기화 완료")
        time.sleep(1)

    def gripper_open(self):
        from DSR_ROBOT2 import set_digital_output, OFF, ON, wait
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.3)
        self.get_logger().info("🖐 그리퍼 OPEN")

    def gripper_close(self):
        from DSR_ROBOT2 import set_digital_output, OFF, ON, wait
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.3)
        self.get_logger().info("✊ 그리퍼 CLOSE")

    def _pick_sequence(self, class_name):
        from DSR_ROBOT2 import posx, movel, movej
        pose = self.class_to_robot_pose.get(class_name)
        if pose is None:
            self.get_logger().warn(f"⚠️ '{class_name}' 좌표 매핑 없음 — 동작 스킵")
            with self.robot_lock: self.robot_busy = False
            return
        try:
            approach = pose.copy(); approach[2] += Z_APPROACH_OFFSET
            movel(posx(approach), vel=TASK_V_L, acc=TASK_A_L)
            movel(posx(pose), vel=TASK_V_L, acc=TASK_A_L)
            self.gripper_close()
            movel(posx(approach), vel=TASK_V_L, acc=TASK_A_L)
            movej(HOME_JReady, vel=HOME_V_J, acc=HOME_ACC_J)
        except Exception as e:
            self.get_logger().error(f"❌ Pick 시퀀스 오류: {e}")
        finally:
            with self.robot_lock: self.robot_busy = False

    def _place_sequence(self, class_name):
        from DSR_ROBOT2 import posx, movel, movej
        pose = self.class_to_place_pose.get(class_name)
        if pose is None:
            self.get_logger().warn(f"⚠️ '{class_name}' 놓기 좌표 없음")
            self.gripper_open()
            movej(HOME_JReady, vel=HOME_V_J, acc=HOME_ACC_J)
            with self.robot_lock: self.robot_busy = False
            return
        try:
            approach = pose.copy(); approach[2] += Z_APPROACH_OFFSET
            movel(posx(approach), vel=TASK_V_L, acc=TASK_A_L)
            movel(posx(pose), vel=TASK_V_L, acc=TASK_A_L)
            self.gripper_open()
            movel(posx(approach), vel=TASK_V_L, acc=TASK_A_L)
            movej(HOME_JReady, vel=HOME_V_J, acc=HOME_ACC_J)
        except Exception as e:
            self.get_logger().error(f"❌ Place 시퀀스 오류: {e}")
        finally:
            with self.robot_lock: self.robot_busy = False

    def start_pick(self, class_name):
        with self.robot_lock:
            if self.robot_busy: return
            self.robot_busy = True
        threading.Thread(target=self._pick_sequence, args=(class_name,), daemon=True).start()

    def start_place(self, class_name):
        with self.robot_lock:
            if self.robot_busy: return
            self.robot_busy = True
        threading.Thread(target=self._place_sequence, args=(class_name,), daemon=True).start()

    def cam1_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)
            results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            if results.multi_hand_landmarks:
                self.latest_hand_landmarks = results.multi_hand_landmarks[0]
                self._check_grip_state(self.latest_hand_landmarks)
            else: self.latest_hand_landmarks = None
        except Exception as e: self.get_logger().error(f"Cam1 오류: {e}")

    def _check_grip_state(self, landmarks):
        tips = [8, 12, 16, 20]; pips = [6, 10, 14, 18]
        closed = sum(1 for t, p in zip(tips, pips) if landmarks.landmark[t].y > landmarks.landmark[p].y)
        new_state = "Close" if closed >= 3 else "Open"
        
        # 상태가 변할 때만 로그 출력
        if new_state != self.grip_state:
            self.get_logger().info(f"✋ 손 상태 변경: {self.grip_state} -> {new_state}")
            self.grip_state = new_state

    def _get_palm_center(self, landmarks, w, h):
        ids = [0, 5, 9, 13, 17]
        cx = int(np.mean([landmarks.landmark[i].x for i in ids]) * w)
        cy = int(np.mean([landmarks.landmark[i].y for i in ids]) * h)
        return cx, cy
    
    def _full_sequence(self, class_name):
        """Pick -> Home -> Place 통합 시퀀스 (에러 방지를 위해 하나로 합침)"""
        from DSR_ROBOT2 import posx, movel, movej, wait
        
        pick_pose = self.class_to_robot_pose.get(class_name.lower())
        place_pose = self.class_to_place_pose.get(class_name.lower())

        if pick_pose is None:
            self.get_logger().warn(f"⚠️ '{class_name}' 좌표 매핑 없음")
            with self.robot_lock: self.robot_busy = False
            return

        try:
            # 1. Pick (접근 -> 집기 -> 후퇴)
            self.get_logger().info(f"🍎 {class_name} 작업을 시작합니다.")
            approach_pick = pick_pose.copy(); approach_pick[2] += Z_APPROACH_OFFSET
            
            movel(posx(approach_pick), vel=TASK_V_L, acc=TASK_A_L)
            movel(posx(pick_pose), vel=TASK_V_L, acc=TASK_A_L)
            self.gripper_close()
            movel(posx(approach_pick), vel=TASK_V_L, acc=TASK_A_L)

            # 2. Home (중간 경유)
            self.get_logger().info("🏠 홈 위치로 이동 중...")
            movej(HOME_JReady, vel=HOME_V_J, acc=HOME_ACC_J)
            wait(0.5)

            # 3. Place (접근 -> 놓기 -> 후퇴)
            self.get_logger().info(f"📍 {class_name} 내려놓기 중...")
            approach_place = place_pose.copy(); approach_place[2] += Z_APPROACH_OFFSET
            
            movel(posx(approach_place), vel=TASK_V_L, acc=TASK_A_L)
            movel(posx(place_pose), vel=TASK_V_L, acc=TASK_A_L)
            self.gripper_open()
            movel(posx(approach_place), vel=TASK_V_L, acc=TASK_A_L)
            
            # 최종 Home 복귀
            movej(HOME_JReady, vel=HOME_V_J, acc=HOME_ACC_J)
            
        except Exception as e:
            self.get_logger().error(f"❌ 시퀀스 오류: {e}")
        finally:
            with self.robot_lock: 
                self.robot_busy = False
                self.action_state = "IDLE" # 상태 초기화

    # 기존 start_pick, start_place를 통합하거나 아래와 같이 수정
    def start_combined_action(self, class_name):
        with self.robot_lock:
            if self.robot_busy: return
            self.robot_busy = True
        threading.Thread(target=self._full_sequence, args=(class_name,), daemon=True).start()

    def cam2_callback(self, msg):
        try:
            bg_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = bg_frame.shape
            
            # YOLO 예측 (device=0 혹은 device='cuda' 명시)
            yolo_results = self.model.predict(source=bg_frame, conf=0.5, save=False, verbose=False)
            annotated = yolo_results[0].plot()
            current_hover_target = None

            # 1. 그리퍼 상태 변화 감지 (콜백 시작 시점의 상태 저장)
            # _check_grip_state는 cam1에서 돌고 있으므로, 여기서 시점을 잡아줍니다.
            grip_just_closed = (self.grip_state == "Close" and self.prev_grip_state == "Open")

            if self.latest_hand_landmarks:
                px, py = self._get_palm_center(self.latest_hand_landmarks, w, h)
                cv2.circle(annotated, (px, py), 15, (255, 0, 255), 2) 
                
                # 2. 타겟 체크
                for box in yolo_results[0].boxes:
                    b = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id].lower()
                    
                    if b[0] <= px <= b[2] and b[1] <= py <= b[3]:
                        current_hover_target = cls_name
                        cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 4)
                        break

                # 3. 호버링 로직
                if current_hover_target:
                    if current_hover_target == self.hover_class_name:
                        elapsed = time.time() - self.hover_start_time
                        if elapsed >= 2.0 and not self.selected_logged:
                            self.get_logger().info(f"🎯 TARGET LOCKED: {current_hover_target}")
                            self.selected_logged = True
                    else:
                        self.hover_start_time = time.time()
                        self.hover_class_name = current_hover_target
                        self.selected_logged = False
                else:
                    self.hover_start_time = None
                    self.hover_class_name = None
                    self.selected_logged = False

                # 4. 로봇 실행 (조건: 타겟이 선정된 상태에서 주먹을 쥐었을 때)
                if self.action_state == "IDLE":
                    if grip_just_closed and self.selected_logged: # 확정된 타겟이 있을 때만
                        self.get_logger().info(f"🚀 Action Start with {self.hover_class_name}")
                        self.action_state = "EXECUTING"
                        self.start_combined_action(self.hover_class_name)

                # 상태 업데이트
                self.prev_grip_state = self.grip_state
                self.mp_draw.draw_landmarks(annotated, self.latest_hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

            # GUI 텍스트 표시
            cv2.putText(annotated, f"STATE: {self.action_state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(annotated, f"GRIP: {self.grip_state}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Fruit Robot System", annotated)
            cv2.waitKey(1)

        except Exception as e:
            # 에러가 나면 반드시 로그를 찍도록 수정
            self.get_logger().error(f"❌ Cam2 Critical Error: {e}")

# ═══════════════════════════════════════════
# 수정된 main 함수 (멀티스레드 적용)
# ═══════════════════════════════════════════
def main():
    rclpy.init()
    node = FruitRobotNode()
    DR_init.__dsr__node = node

    # MultiThreadedExecutor 사용 (스레드 간 핸들 충돌 방지)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.initialize_robot()
        # rclpy.spin(node) 대신 executor.spin() 사용
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()