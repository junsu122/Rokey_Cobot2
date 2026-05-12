#!/bin/bash
# jarvis_ros.sh — ROS2 노드 그룹 시작
source /home/kng/cobot_ws/install/setup.bash
source ~/jarvis_webrtc/jarvis_helpers.sh

# RealSense 시작
echo "📷 RealSense 카메라 시작..."
ros2 launch realsense2_camera rs_align_depth_launch.py \
  depth_module.depth_profile:=424x240x30 \
  rgb_camera.color_profile:=640x480x30 \
  initial_reset:=true \
  align_depth.enable:=true \
  enable_rgbd:=true \
  pointcloud.enable:=true \
  2>&1 | tee /tmp/realsense.log &
echo "⏳ RealSense 초기화 대기 (5초)..."
sleep 5

echo "┌─────────────────────────────────────┐"
echo "│  🟢 ROS2 노드 그룹 시작             │"
echo "└─────────────────────────────────────┘"
echo ""

# jarvis_main.launch.py 로 전체 노드 실행
ros2 launch jarvis_bringup jarvis_main.launch.py 2>&1 | tee /tmp/ros_launch.log &

# orchestrator가 마지막으로 뜨므로 이걸 기다림
wait_ros_node "orchestrator_node" 30
wait_log /tmp/ros_launch.log "OrchestratorNode ready" "초기화 대기" 40

echo ""
echo "  ✅ ROS2 노드 그룹 준비 완료"
echo ""
