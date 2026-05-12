# Rokey_Cobot2

두산로봇(DSR-01)과 OnRobot RG2 그리퍼, Intel RealSense D435i를 활용한 협동 로봇 제어 및 비전 통합 프로젝트 저장소입니다.

---

## 1. 프로젝트 내용 (Project Description)
*여기에 프로젝트의 상세 목적과 주요 기능을 작성하세요.*
- **주요 목표**: [예: AI 비전 기반 물체 인식 및 자동화 Pick-and-Place 구현]
- **핵심 기능**: 
    - RealSense D435i를 이용한 3D 환경 인식 및 객체 탐지
    - ROS2 Humble 기반의 로봇 암(DSR-01) 및 그리퍼(RG2) 통합 제어
    - [예: 음성 명령 인터페이스 또는 Unity 디지털 트윈 연동]

## 2. 시스템 아키텍처 (System Architecture)
전체 시스템의 하드웨어 구성과 소프트웨어 모듈 간의 연결 구조입니다.

| 분류 | 세부 사양 |
| :--- | :--- |
| **Robot Arm** | Doosan Robotics DSR-01 (6-Axis) |
| **Gripper** | OnRobot RG2 |
| **Camera** | Intel RealSense D435i |
| **OS / Middleware** | Ubuntu 22.04 / ROS2 Humble |

*(추후 이곳에 아키텍처 다이어그램 이미지를 추가하세요)*

## 3. 노드 아키텍처 (Node Architecture)
본 프로젝트의 주요 ROS2 노드 구성입니다.

- `/camera_node`: RealSense 데이터 발행 (RGB-D)
- `/perception_node`: 비전 알고리즘 처리 및 객체 좌표 정보 계산
- `/main_control_node`: 전체 시퀀스 제어 및 상태 머신 관리 (State Machine)
- `/dsr_driver_node`: 두산 로봇 하드웨어 인터페이스 통신
- `/gripper_node`: OnRobot RG2 그리퍼 제어 서비스/액션 서버

## 4. 설치 및 실행 방법 (Getting Started)

### 환경 요구 사항
- **ROS Version:** ROS2 Humble
- **Required Drivers:** - `dsr_ros2` (Doosan Robot Driver)
  - `realsense2_camera`
  - `onrobot_rg_control` (Gripper Driver)

### 설치 방법
```bash
# 워크스페이스 생성 및 이동
mkdir -p ~/rokey_ws/src
cd ~/rokey_ws/src

# 저장소 클론
git clone [https://github.com/junsu122/Rokey_Cobot2.git](https://github.com/junsu122/Rokey_Cobot2.git)

# 의존성 설치 및 빌드
cd ~/rokey_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
