#!/usr/bin/env python3
"""
webrtc_server.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
브라우저 WebRTC DataChannel 수신
→ /gesture_event ROS2 토픽 발행

[수정]
  - /is_picking (std_msgs/Bool) 구독 추가
  - is_picking 상태 변화 시 브라우저로 {'type': 'is_picking', 'picking': bool} 전송

[실행]
    source /home/kng/cobot_ws/install/setup.bash
    python3 webrtc_server.py
"""

import asyncio
import json
import threading
import requests
from datetime import datetime

from aiortc import (
    RTCPeerConnection, RTCSessionDescription,
    RTCConfiguration, RTCIceServer
)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from gesture_robot_interfaces.msg import GestureEvent
    from dsr_msgs2.msg import RobotState
    from dsr_msgs2.srv import GetCurrentPosx
    from std_msgs.msg import String, Bool
    from gesture_robot_pkg.constants import ROBOT_ID
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    ROBOT_ID = 'dsr01'
    print("⚠️  ROS2 없음 — 콘솔 출력 모드")

import os
SIGNALING_URL = os.getenv('SIGNALING_URL', 'https://jarvis-signaling-production.up.railway.app')
ROOM          = "jarvis-gesture"
STUN_SERVER   = "stun:stun.l.google.com:19302"

# ── 로그 헬퍼 (줄 덮어쓰기 없이 출력) ────────────────────────────────────────
def log(msg: str):
    ts = datetime.now().strftime('%H:%M:%S')
    print(f"[{ts}] {msg}", flush=True)


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 퍼블리셔
# ══════════════════════════════════════════════════════════════════════════════

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('webrtc_gesture_publisher')
        self._pub             = self.create_publisher(GestureEvent, '/gesture_event', 10)
        self._pub_browser_stt = self.create_publisher(String, '/browser_stt', 10)

        self._channel  = None
        self._loop     = None
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            RobotState, f'/{ROBOT_ID}/state',
            self._robot_state_cb, 10,
            callback_group=self._cb_group)

        # ── /is_picking 구독 추가 ──────────────────────────────────────────
        self.create_subscription(
            Bool, '/is_picking',
            self._is_picking_cb, 10,
            callback_group=self._cb_group)

        self._get_posx_cli = self.create_client(
            GetCurrentPosx, f'/{ROBOT_ID}/aux_control/get_current_posx',
            callback_group=self._cb_group)
        if not self._get_posx_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('get_current_posx service not found')

        self.create_timer(1.0, self._poll_tcp, callback_group=self._cb_group)

        self.create_subscription(String, '/voice_command',  self._voice_command_cb,  10, callback_group=self._cb_group)
        self.create_subscription(String, '/intent_result',  self._intent_result_cb,  10, callback_group=self._cb_group)
        self.create_subscription(String, '/tts_output',     self._tts_output_cb,     10, callback_group=self._cb_group)
        self.create_subscription(String, f'/{ROBOT_ID}/robot_state_summary',
                                 self._robot_state_summary_cb, 10, callback_group=self._cb_group)

        # /recovery_command 퍼블리셔
        self._pub_recovery = self.create_publisher(String, f'/{ROBOT_ID}/recovery_command', 10)

        self.get_logger().info('✅ /gesture_event 퍼블리셔 준비')
        self.get_logger().info(f'✅ /{ROBOT_ID}/state 구독 시작')
        self.get_logger().info('✅ 음성 토픽 구독 시작')
        self.get_logger().info('✅ /is_picking 구독 시작')

    def _poll_tcp(self):
        if not self._get_posx_cli.service_is_ready(): return
        if self._channel is None or self._channel.readyState != 'open': return
        req = GetCurrentPosx.Request(); req.ref = 0
        self._get_posx_cli.call_async(req).add_done_callback(self._poll_tcp_cb)

    def _poll_tcp_cb(self, future):
        try:
            res = future.result()
            if res and res.success and res.task_pos_info and len(res.task_pos_info[0].data) >= 6:
                self._send_tcp_to_browser([float(v) for v in res.task_pos_info[0].data[:6]])
        except Exception: pass

    def _robot_state_cb(self, msg: RobotState):
        if len(msg.current_posx) >= 6:
            self._send_tcp_to_browser([float(v) for v in msg.current_posx])

    def _is_picking_cb(self, msg: Bool):
        """
        /is_picking 수신 → 브라우저로 전달
        형식: {'type': 'is_picking', 'picking': true/false}
        """
        self._send_to_browser({'type': 'is_picking', 'picking': msg.data})
        self.get_logger().info(f'[IS_PICKING] → 브라우저: picking={msg.data}')

    def _send_tcp_to_browser(self, tcp: list):
        if self._channel is None or self._channel.readyState != 'open': return
        tcp_data = json.dumps({'type': 'robot_state', 'tcp': tcp})
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._send_data(tcp_data), self._loop)

    async def _send_data(self, data: str):
        try:
            if self._channel and self._channel.readyState == 'open':
                self._channel.send(data)
        except Exception as e:
            self.get_logger().warn(f'DataChannel 전송 오류: {e}')

    def _send_to_browser(self, data: dict):
        if self._channel is None or self._loop is None: return
        if self._channel.readyState != 'open': return
        asyncio.run_coroutine_threadsafe(
            self._send_data(json.dumps(data, ensure_ascii=False)), self._loop)

    def _voice_command_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._send_to_browser({'type': 'voice_command', 'text': data.get('text', '')})
        except Exception: pass

    def publish_browser_stt(self, text: str):
        msg = String(); msg.data = json.dumps({'text': text}, ensure_ascii=False)
        self._pub_browser_stt.publish(msg)
        self.get_logger().info(f'[BROWSER STT] → /browser_stt: "{text}"')

    def _intent_result_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._send_to_browser({
                'type': 'intent_result', 'intent': data.get('intent', ''),
                'target_object': data.get('target_object'),
                'confidence': data.get('confidence', 0.0),
                'response_message': data.get('response_message', ''),
            })
        except Exception: pass

    def _tts_output_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._send_to_browser({'type': 'tts_output', 'message': data.get('message', '')})
        except Exception: pass

    def _robot_state_summary_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._send_to_browser({'type': 'robot_state_summary',
                                   'state_code': data.get('state_code', -1),
                                   'state_str':  data.get('state_str', ''),
                                   'recovering': data.get('recovering', False)})
        except Exception: pass

    def publish(self, data: dict):
        msg = GestureEvent()
        msg.gesture_state  = data.get('gesture_state',  'NONE')
        msg.hand_visible   = bool(data.get('hand_visible',   False))
        msg.is_fist        = bool(data.get('is_fist',         False))
        msg.is_pointing    = bool(data.get('is_pointing',     False))
        msg.avg_x          = float(data.get('avg_x',          0.0))
        msg.avg_y          = float(data.get('avg_y',          0.0))
        msg.index_tip_x    = float(data.get('index_tip_x',    0.0))
        msg.index_tip_y    = float(data.get('index_tip_y',    0.0))
        msg.curr_dist      = float(data.get('curr_dist',      0.0))
        msg.base_dist      = float(data.get('base_dist',      0.0))
        msg.calib_progress = float(data.get('calib_progress', 0.0))
        msg.calib_tcp      = [float(v) for v in data.get('calib_tcp',      [0.0]*6)]
        msg.target_pos_mm  = [float(v) for v in data.get('target_pos_mm',  [0.0]*6)]
        msg.velocity_delta = [float(v) for v in data.get('velocity_delta', [0.0]*3)]
        msg.landmarks_x    = [float(v) for v in data.get('landmarks_x',    [0.0]*21)]
        msg.landmarks_y    = [float(v) for v in data.get('landmarks_y',    [0.0]*21)]
        self._pub.publish(msg)


_publisher_node: "GesturePublisher | None" = None

def start_ros2():
    global _publisher_node
    rclpy.init()
    _publisher_node = GesturePublisher()
    rclpy.spin(_publisher_node)


# ══════════════════════════════════════════════════════════════════════════════
# 메시지 처리
# ══════════════════════════════════════════════════════════════════════════════

_last_state = ''  # gesture 상태 변화 시에만 로그 출력

def on_message(message: str):
    global _last_state
    try:
        data = json.loads(message)

        if data.get('type') == 'browser_stt':
            text = data.get('text', '').strip()
            if text and ROS2_AVAILABLE and _publisher_node:
                _publisher_node.publish_browser_stt(text)
            return

        if data.get('type') == 'recovery_command':
            if ROS2_AVAILABLE and _publisher_node:
                msg = String()
                msg.data = 'RECOVER'
                _publisher_node._pub_recovery.publish(msg)
                log('[RECOVERY] → /dsr01/recovery_command 발행')
            return

        if data.get('type') != 'gesture':
            return

        if ROS2_AVAILABLE and _publisher_node:
            _publisher_node.publish(data)

        # 상태 변화 시에만 로그 (매 프레임 출력 X)
        state = data.get('gesture_state', '—')
        if state != _last_state:
            _last_state = state
            hand = '✋' if data.get('hand_visible') else '—'
            log(f"[GESTURE] state={state}  hand={hand}")

    except Exception as e:
        log(f"❌ 메시지 파싱 오류: {e}")


# ══════════════════════════════════════════════════════════════════════════════
# WebRTC 서버
# ══════════════════════════════════════════════════════════════════════════════

async def run_webrtc():
    config = RTCConfiguration(
        iceServers=[
            RTCIceServer(urls='stun:stun.l.google.com:19302'),
            RTCIceServer(urls='turn:openrelay.metered.ca:80',
                         username='openrelayproject', credential='openrelayproject'),
            RTCIceServer(urls='turn:openrelay.metered.ca:443',
                         username='openrelayproject', credential='openrelayproject'),
        ]
    )
    pc = RTCPeerConnection(configuration=config)

    @pc.on("datachannel")
    def on_datachannel(channel):
        log(f"✅ DataChannel 수신: {channel.label}")
        if ROS2_AVAILABLE and _publisher_node:
            _publisher_node._channel = channel
            _publisher_node._loop    = asyncio.get_event_loop()

        @channel.on("open")
        def on_open():
            log("✅ DataChannel 열림 → 수신 중...")

        @channel.on("message")
        def on_msg(message):
            on_message(message)

        @channel.on("close")
        def on_close():
            log("❌ DataChannel 닫힘")
            if ROS2_AVAILABLE and _publisher_node:
                _publisher_node._channel = None

    @pc.on("connectionstatechange")
    async def on_state():
        log(f"🔗 [gesture] 연결 상태: {pc.connectionState}")

    # offer 폴링
    log(f"⏳ [gesture] offer 대기 중... (room={ROOM})")
    wait_sec = 0
    while True:
        try:
            resp = requests.get(f"{SIGNALING_URL}/offer/{ROOM}", timeout=5)
            if resp.status_code == 200:
                offer_data = resp.json()
                break
        except Exception:
            pass
        await asyncio.sleep(1.0)
        wait_sec += 1
        if wait_sec % 10 == 0:
            log(f"⏳ [gesture] offer 대기 중... {wait_sec}s")

    log("✅ [gesture] Offer 수신!")
    await pc.setRemoteDescription(RTCSessionDescription(sdp=offer_data['sdp'], type=offer_data['type']))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    await asyncio.sleep(1.5)

    requests.post(f"{SIGNALING_URL}/answer", json={
        'room': ROOM, 'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type,
    })
    log("✅ [gesture] Answer 전송 → 연결 수립!")
    log("📡 [gesture] 스트리밍 중...")

    try:
        while True:
            await asyncio.sleep(1.0)
            if pc.connectionState in ('failed', 'closed'):
                log(f"🔴 [gesture] 연결 종료 ({pc.connectionState}) → 재시작")
                break
    except asyncio.CancelledError:
        pass
    finally:
        await pc.close()
        requests.delete(f"{SIGNALING_URL}/clear/{ROOM}")


async def main():
    log("🤖 JARVIS WebRTC 제스처 서버 시작")
    log(f"   시그널링: {SIGNALING_URL}")
    log(f"   Room:     {ROOM}")

    if ROS2_AVAILABLE:
        t = threading.Thread(target=start_ros2, daemon=True)
        t.start()
        log("✅ ROS2 퍼블리셔 스레드 시작")
    else:
        log("⚠️  테스트 모드 — ROS2 발행 없음")

    while True:
        try:
            await run_webrtc()
        except KeyboardInterrupt:
            log("👋 종료합니다.")
            break
        except Exception as e:
            log(f"❌ 오류: {e} — 3초 후 재연결...")
            await asyncio.sleep(3)


if __name__ == '__main__':
    asyncio.run(main())