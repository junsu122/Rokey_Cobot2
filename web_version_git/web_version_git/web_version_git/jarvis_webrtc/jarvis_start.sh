#!/bin/bash
# jarvis_start.sh — 전체 시스템 시작
source /home/kng/cobot_ws/install/setup.bash
source ~/jarvis_webrtc/jarvis_helpers.sh

echo ""
echo "╔══════════════════════════════════════╗"
echo "║       🤖 짱구네 시스템 시작          ║"
echo "╚══════════════════════════════════════╝"

# ── 기존 프로세스 정리 ───────────────────────────────────────────────────────
echo ""
echo "🛑 기존 프로세스 종료 중..."
pkill -9 -f "webrtc_server|webrtc_vision|vision_node|vite|flask|orchestrator_node|pick_and_place_node|webcam_teleop_node" 2>/dev/null
sleep 2
rm -f /tmp/webrtc_server.log /tmp/webrtc_vision.log /tmp/vision_node.log \
       /tmp/pick_and_place.log /tmp/orchestrator.log /tmp/vite.log
echo "   완료 ✅"

# ── 1단계: ROS2 노드 그룹 ────────────────────────────────────────────────────
echo ""
bash ~/jarvis_webrtc/jarvis_ros.sh
if [ $? -ne 0 ]; then
  echo "❌ ROS2 노드 시작 실패 — 중단합니다."
  exit 1
fi

# ── 2단계: 웹 서버 그룹 ─────────────────────────────────────────────────────
bash ~/jarvis_webrtc/jarvis_web.sh
if [ $? -ne 0 ]; then
  echo "❌ 웹 서버 시작 실패 — 중단합니다."
  exit 1
fi

# ── WebRTC 브라우저 연결 대기 ────────────────────────────────────────────────
echo "  ⏳ WebRTC 브라우저 연결 대기 중..."
echo "     → 웹앱(http://localhost:5173)을 브라우저에서 열어주세요"
wait_log /tmp/webrtc_server.log "스트리밍 중" "  WebRTC 연결" 60

# ── 완료 ────────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════╗"
echo "║                                      ║"
echo "║   ✅  전체 시스템 연결 완료!         ║"
echo "║                                      ║"
echo "║   🌐  http://localhost:5173          ║"
echo "║   🤖  제스처 제어 사용 가능          ║"
echo "║                                      ║"
echo "╚══════════════════════════════════════╝"
echo ""
echo -e "\a"

echo "  로그 확인:"
echo "    tail -f /tmp/webrtc_server.log   # WebRTC 제스처"
echo "    tail -f /tmp/webrtc_vision.log   # WebRTC 비전"
echo "    tail -f /tmp/orchestrator.log    # Orchestrator"
echo "    tail -f /tmp/pick_and_place.log  # Pick & Place"
echo "    tail -f /tmp/vision_node.log     # Vision"
echo ""
echo "  개별 재시작:"
echo "    bash ~/jarvis_webrtc/jarvis_ros.sh   # ROS만"
echo "    bash ~/jarvis_webrtc/jarvis_web.sh   # 웹만"
echo ""
echo "  종료: pkill -9 -f 'webrtc|vision_node|vite|orchestrator|pick_and_place'"
echo "════════════════════════════════════════"

wait
