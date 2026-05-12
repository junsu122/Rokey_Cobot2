std_msgs/Header header

# 제스처 상태 머신 (webcam_teleop_node 내부)
string gesture_state     # WAITING | CALIBRATING | CONTROLLING | PAUSED

# 손 감지 여부 및 제스처 분류
bool hand_visible
bool is_fist
bool is_pointing

# 손바닥 중심 (정규화 0-1, palm center for deadzone / calibration)
float32 avg_x
float32 avg_y

# 검지 끝 위치 (정규화 0-1, vision_node 에서 호버 판정에 사용)
float32 index_tip_x
float32 index_tip_y

# 손목-검지 거리 (depth 추정용)
float32 curr_dist
float32 base_dist

# 캘리브레이션
float32 calib_progress     # 0.0 ~ 1.0
float32[6] calib_tcp       # mm + deg, 캘리브레이션 완료 시점 TCP

# 텔레오퍼레이션 목표값 (orchestrator 가 teleop/enable 시 로봇에 전달)
float32[6] target_pos_mm   # POSITION_CONTROL 모드
float32[3] velocity_delta  # VELOCITY 모드 (mm/step)

# 21개 손 랜드마크 (정규화 0-1), vision_node 가 RealSense 창에 스켈레톤 표시할 때 사용
float32[21] landmarks_x
float32[21] landmarks_y
