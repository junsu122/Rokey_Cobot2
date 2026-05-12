#!/bin/bash
# jarvis_helpers.sh — 공통 헬퍼 함수 (source로 불러서 사용)

wait_process() {
  local name=$1 timeout=${2:-10} i=0
  printf "      프로세스 대기"
  while ! pgrep -f "$name" > /dev/null 2>&1; do
    sleep 0.5; printf "."; i=$((i+1))
    if [ $i -ge $((timeout*2)) ]; then echo " ❌ 타임아웃"; return 1; fi
  done
  echo " ✅"; return 0
}

wait_ros_node() {
  local node=$1 timeout=${2:-15} i=0
  printf "      노드 등록 대기"
  while ! ros2 node list 2>/dev/null | grep -q "$node"; do
    sleep 0.5; printf "."; i=$((i+1))
    if [ $i -ge $((timeout*2)) ]; then echo " ❌ 타임아웃"; return 1; fi
  done
  echo " ✅"; return 0
}

wait_log() {
  local logfile=$1 pattern=$2 label=$3 timeout=${4:-30} i=0
  printf "      %-20s" "$label"
  while ! grep -q "$pattern" "$logfile" 2>/dev/null; do
    sleep 0.5; printf "."; i=$((i+1))
    if [ $i -ge $((timeout*2)) ]; then echo " ❌ 타임아웃 (${timeout}s)"; return 1; fi
  done
  echo " ✅"; return 0
}
