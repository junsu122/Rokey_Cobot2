# 🤖 JARVIS — 시니어 보조 협동로봇

ROS2 + WebRTC 기반 음성·제스처·비전 통합 협동로봇 제어 시스템

---

## 시스템 다이어그램

![JARVIS 시스템 아키텍처](./senior_drawio.svg)

---

## 구동 방법 (웹 사용 버젼)

### 1. ROS2 빌드

```bash
cd ~/본인워크스페이스
colcon build
source install/setup.bash
```

### 2. ROS2 시스템 실행
홈에서 실행
```bash
source ~/본인워크스페이스/install/setup.bash
ros2 launch jarvis_bringup jarvis_main.launch.py
```

### 3. 웹 서버 실행 (별도 터미널)
홈에서 실행
```bash
source ~/본인워크스페이스/install/setup.bash
bash ~/본인워크스페이스/jarvis_webrtc/jarvis_web.sh
```

### 4. 웹앱 빌드 & 배포 (UI 변경 시)
홈에서 실행
```bash
cd ~/본인워크스페이스/jarvis-ui/web-app
npm run build
firebase deploy --only hosting
```
---

## 구동 방법 (로컬 사용 버젼)

### 1. ROS2 빌드

```bash
cd ~/본인워크스페이스
colcon build
source install/setup.bash
```

### 2. 센서 연동

realsense 및 dsr01 연결

### 3. ROS2 시스템 실행 (메인 컨트롤러 런치)

```bash
ros2 launch gesture_robot_pkg gesture_robot.launch.py 
```

### 4. ROS2 시스템 실행 (상태 컨트롤러 런치)

```bash
ros2 launch robot_state_control robot_state_control.launch.py 
```

### 5. ROS2 시스템 실행 (객체 너비 확인 노드 런치)

```bash
ros2 launch object_width_filter object_width_filter.launch.py
```

docker로 대체 가능

```bash
# 1. 도커 컨테이너의 GUI 출력을 허용 (터미널에 입력)
xhost +local:docker

# 2. 통신을 위한 환경 변수 설정 (Bag 파일 실행 PC와 동일하게 맞춤)
export ROS_DOMAIN_ID=95
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3. 최신 이미지 다운로드
docker pull junsu122/object_width_filter:v2.1

# 4. 컨테이너 실행 (네트워크, 디스플레이, 카메라 권한 포함)
docker run -it --rm \
  --net=host \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  junsu122/object_width_filter:v2.1

# 5. 런치 파일 실행 (모든 노드 가동)
ros2 launch object_width_filter object_width_filter.launch.py
```

```bash
# 다른 PC의 ~/.bashrc에 추가
alias rf='xhost +local:docker && docker run -it --rm --net=host --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged junsu122/object_width_filter:v2.1'
```

---


## 전체 흐름

### 브라우저 호버 선택 픽

```
브라우저: 물체 위 손가락 hover
    ↓ DataChannel
webrtc_vision_server.py
    → /selected_label 발행
    → 브라우저로 is_picking=True 전송 (모달 표시)
    ↓
vision_node.py (_selected_label_cb)
    → YOLO 탐지 결과에서 원본 좌표 추출
    → /selected_object 발행
    ↓
┌─────────────────────────────────────────┐
│ 동시 처리                                │
│  orchestrator_node.py                   │
│    → PickAndPlace 액션 트리거            │
│                                         │
│  vision_node2publish_angle.py           │
│    → aligned depth + camera_info       │
│    → /_2d_to_3d_point 발행              │
│    → publish_angle.py                  │
│    → /object_angle 발행                 │
└─────────────────────────────────────────┘
    ↓
pick_and_place_node.py
    HOME → APPROACH
    → /selected_object 재발행 (각도 재계산)
    → SPIN_CHUCK (30초 대기, /object_angle 수신)
    → DESCEND → GRASP → LIFT → GIVE
    ↓
orchestrator_node.py → /is_picking=False
    ↓
브라우저: "작업 완료" 모달 표시
```

---

### 음성 명령 픽 (예: "사과 가져다줘")

```
브라우저 마이크 → /browser_stt
    ↓
voice_intent_node.py
    → GPT-4o 3단계 의도 추론
      (intent=bring_object, target=apple)
    → /voice_intent 발행
    ↓
vision_node.py (_voice_intent_cb)
    → YOLO에서 apple 탐지
    → /selected_object 발행
    ↓
orchestrator_node.py + vision_node2publish_angle.py (동시)
    ↓
pick_and_place_node.py
    HOME → APPROACH → SPIN_CHUCK → DESCEND → GRASP → LIFT → GIVE
```

---

### 배고프다 스캔 픽 (예: "나 배고파")

```
voice_intent_node.py
    → 현재 카메라에 음식 있으면 → 바로 픽
    → 음식 없으면 → /scan_request 발행
    ↓
workspace_scan_node.py
    → 지그재그 전체 스캔 (3×3 격자)
    → 각 위치에서 VLM으로 음식 탐지 + 저장
    → 스캔 완료 후 발견 물체 순차 처리:
        ① 물체 위로 센터링 이동
        ② /voice_intent 발행 (from_scan=True)
        ③ 픽 완료 대기
        ④ 다음 물체 반복
    → 모든 픽 완료 후 /scan_result 발행
    ↓
pick_and_place_node.py (from_scan=True)
    HOME 스킵 → APPROACH → SPIN_CHUCK → DESCEND → GRASP → LIFT → GIVE
```

---

### 외출 준비 (예: "나갈게")

```
voice_intent_node.py
    → intent=going_out
    → _plan_going_out_items():
        날씨 API + 카메라 이미지 → o4-mini
        → 우산/썬크림/물/마스크 중 필요한 것 선택
    → 각 물건 순서대로 픽 처리
    ↓
pick_and_place_node.py
    HOME → APPROACH → SPIN_CHUCK → DESCEND → GRASP → LIFT
    → WAY_POINT → GIVE_JOINT → GIVE_LINE (직접교시 위치)
```

---

### 로봇 상태 이상 처리

```
exception_manager_node.py
    → state_code 감지
    → /dsr01/robot_state_summary 발행
    ↓
webrtc_server.py → 브라우저로 전달
    ↓
Dashboard.jsx
    state_code=5  → ⚠️ 안전정지 모달 (노란 톤, 충돌 감지)
    state_code=3,6 → 🚨 비상정지 모달 (빨간 톤)
    → 복구 버튼 → recovery_command 발행
```

---

## 각도 계산 파이프라인

```
/selected_object (label + bbox)
    ↓
vision_node2publish_angle.py
    - depth: /camera/camera/aligned_depth_to_color/image_raw
    - intrinsics: /camera/camera/color/camera_info (동적 수신)
    - bbox 중심 5×5 패치 중앙값으로 depth 추출
    - 2D → 3D 변환 (스케일링 없음)
    ↓
/_2d_to_3d_point (PointStamped)
    ↓
publish_angle.py
    → PCA 기반 물체 장축 방향 계산
    ↓
/object_angle (Float32, degree)
    ↓
pick_and_place_node.py SPIN_CHUCK
    → gripper 회전 적용
```

---

## 픽앤플레이스 스테이지

```
from_scan=False (브라우저/음성):
  HOME → OPEN_GRIPPER → APPROACH → SPIN_CHUCK
  → DESCEND → GRASP → LIFT → WAY_POINT → GIVE → HOME

from_scan=True (스캔 픽, HOME 스킵):
  OPEN_GRIPPER → APPROACH → SPIN_CHUCK
  → DESCEND → GRASP → LIFT → WAY_POINT → GIVE → HOME
```

APPROACH 완료 시 `/selected_object` 재발행 → 각도 파이프라인 재트리거 → SPIN_CHUCK에서 정확한 각도 수신

---

## 주요 파일 경로

| 파일 | 경로 |
|---|---|
| ROS2 런처 | `src/jarvis_bringup/launch/jarvis_main.launch.py` |
| 비전 노드 | `src/gesture_robot_pkg/gesture_robot_pkg/vision_node.py` |
| 픽앤플레이스 | `src/gesture_robot_pkg/gesture_robot_pkg/pick_and_place_node.py` |
| 오케스트레이터 | `src/gesture_robot_pkg/gesture_robot_pkg/orchestrator_node.py` |
| 각도 계산 | `src/object_width_filter/object_width_filter/vision_node2publish_angle.py` |
| 음성 의도 | `src/jarvis_voice_pkg/jarvis_voice_pkg/voice_intent_node.py` |
| 스캔 노드 | `src/jarvis_voice_pkg/jarvis_voice_pkg/workspace_scan_node.py` |
| 상태 관리 | `src/jarvis_voice_pkg/jarvis_voice_pkg/publisher.py` |
| 상수 | `src/gesture_robot_pkg/gesture_robot_pkg/constants.py` |
| WebRTC 제스처 | `jarvis_webrtc/webrtc_server.py` |
| WebRTC 비전 | `jarvis_webrtc/webrtc_vision_server.py` |
| 대시보드 | `jarvis-ui/web-app/src/Dashboard.jsx` |

---

## 주요 ROS2 토픽

| 토픽 | 설명 |
|---|---|
| `/selected_label` | 브라우저 호버 선택 (라벨) |
| `/selected_object` | YOLO 원본 좌표 기반 픽 트리거 |
| `/object_angle` | 물체 그리퍼 회전 각도 (degree) |
| `/voice_intent` | 음성/스캔 기반 픽 트리거 |
| `/is_picking` | 픽 진행 상태 (브라우저 모달용) |
| `/scan_request` | 스캔 시작/취소 |
| `/scan_result` | 스캔 완료 결과 |
| `/browser_stt` | 브라우저 음성 입력 |
| `/_2d_to_3d_point` | 2D bbox → 3D 카메라 좌표 |

---
## 시스템 설계 및 플로우차트
시스템 설계
<img width="1132" height="505" alt="Screenshot from 2026-05-13 13-53-52" src="https://github.com/user-attachments/assets/2ef7c24a-d3a6-4500-ac6e-e8bb8d3056a1" />

플로우 차트
<img width="1132" height="505" alt="Screenshot from 2026-05-13 13-54-02" src="https://github.com/user-attachments/assets/2c30125a-49b6-4e37-89c6-de0efb87182f" />

---
## 의존성
추가 필요
---
