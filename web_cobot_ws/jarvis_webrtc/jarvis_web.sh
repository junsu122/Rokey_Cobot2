#!/bin/bash
# jarvis_web.sh — 웹 서버 그룹 시작 (WebRTC + Vite)
source /home/rokey/cobot_ws/src/web_cobot_ws/install/setup.bash
source ~/cobot_ws/src/web_cobot_ws/jarvis_webrtc/jarvis_helpers.sh

export SIGNALING_URL="https://jarvis-signaling-production.up.railway.app"

TOTAL=3; STEP=0
progress() { STEP=$((STEP+1)); echo ""; echo "  [WEB $STEP/$TOTAL] $1"; }

echo "┌─────────────────────────────────────┐"
echo "│  🌐 웹 서버 그룹 시작               │"
echo "└─────────────────────────────────────┘"

# [1/3] WebRTC 제스처 서버
progress "WebRTC 제스처 서버..."
python3 ~/cobot_ws/src/web_cobot_ws/jarvis_webrtc/webrtc_server.py 2>&1 | tee /tmp/webrtc_server.log &
wait_process "webrtc_server.py" 10

# [2/3] WebRTC 비전 서버
progress "WebRTC 비전 서버..."
python3 ~/cobot_ws/src/web_cobot_ws/jarvis_webrtc/webrtc_vision_server.py 2>&1 | tee /tmp/webrtc_vision.log &
wait_process "webrtc_vision_server.py" 10

# [3/3] Vite 프론트엔드
progress "프론트엔드 (Vite)..."
cd ~/Rokey_Cobot2/web_version_git/jarvis-ui/web-app && npm run dev 2>&1 | tee /tmp/vite.log &
wait_log /tmp/vite.log "Local" "서버 대기" 20

echo ""
echo "  ✅ 웹 서버 그룹 준비 완료"
echo "  🌐 http://localhost:5173"
echo ""
