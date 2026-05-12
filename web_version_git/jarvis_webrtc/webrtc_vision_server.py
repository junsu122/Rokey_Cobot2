#!/usr/bin/env python3
"""
webrtc_vision_server.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[구독]
  /annotated_frame  → VideoTrack (RealSense 영상 스트리밍)
  /yolo_detections  → DataChannel → 브라우저 (bbox 목록)

[DataChannel 수신]
  브라우저 → select → /selected_object 발행
             (브라우저에서 RealSense 영상 위 MediaPipe 호버 판정 결과)

[실행]
    source /home/kng/cobot_ws/install/setup.bash
    python3 webrtc_vision_server.py
"""

import asyncio
import json
import threading
import requests
import numpy as np
import cv2

from av import VideoFrame
from aiortc import (
    RTCPeerConnection, RTCSessionDescription,
    RTCConfiguration, RTCIceServer,
    VideoStreamTrack,
)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    from gesture_robot_interfaces.msg import SelectedObject
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("⚠️  ROS2 없음 — 테스트 모드")

import os
SIGNALING_URL = os.getenv('SIGNALING_URL', 'https://jarvis-signaling-production.up.railway.app')
ROOM          = "jarvis-vision"
STUN_SERVER   = "stun:stun.l.google.com:19302"
FPS           = 30


# ══════════════════════════════════════════════════════════════════════════════
# 공유 상태
# ══════════════════════════════════════════════════════════════════════════════

class SharedState:
    def __init__(self):
        self._frame      = None
        self._frame_lock = threading.Lock()
        self._channel    = None
        self._loop       = None

    def set_frame(self, frame):
        with self._frame_lock:
            self._frame = frame.copy()

    def get_frame(self):
        with self._frame_lock:
            if self._frame is not None:
                return self._frame.copy()
            return None

    def send(self, data: dict):
        if self._channel is None or self._loop is None:
            return
        payload = json.dumps(data, ensure_ascii=False)
        asyncio.run_coroutine_threadsafe(self._do_send(payload), self._loop)

    async def _do_send(self, payload: str):
        try:
            if self._channel:
                self._channel.send(payload)
        except Exception:
            pass  # 전송 실패 무시


shared = SharedState()


# ══════════════════════════════════════════════════════════════════════════════
# VideoTrack
# ══════════════════════════════════════════════════════════════════════════════

class AnnotatedFrameTrack(VideoStreamTrack):
    kind = "video"

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = shared.get_frame()
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame_rgb             = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame           = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts       = pts
        video_frame.time_base = time_base
        return video_frame


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 노드
# ══════════════════════════════════════════════════════════════════════════════

class VisionStreamNode(Node):

    def __init__(self):
        super().__init__('webrtc_vision_server')

        self._bridge   = CvBridge()
        self._cb_group = ReentrantCallbackGroup()

        # 퍼블리셔 — 브라우저 호버 선택 결과 → /selected_object
        self._pub_selected = self.create_publisher(
            SelectedObject, '/selected_object', 10)

        # 구독
        self.create_subscription(
            Image, '/annotated_frame', self._annotated_frame_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            String, '/yolo_detections', self._yolo_detections_cb, 10,
            callback_group=self._cb_group)

        self.get_logger().info('VisionStreamNode ready')
        self.get_logger().info('  /annotated_frame → VideoTrack')
        self.get_logger().info('  /yolo_detections → DataChannel → 브라우저')
        self.get_logger().info('  브라우저 호버 선택 → DataChannel → /selected_object')

    def _annotated_frame_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            shared.set_frame(frame)
        except Exception as e:
            self.get_logger().warn(f'annotated_frame 변환 오류: {e}')

    def _yolo_detections_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            shared.send({
                'type'      : 'yolo_detections',
                'detections': data.get('detections', []),
                'timestamp' : data.get('timestamp', ''),
            })
        except Exception as e:
            self.get_logger().warn(f'yolo_detections 파싱 오류: {e}')

    def on_browser_message(self, message: str):
        """
        브라우저 DataChannel 메시지 처리
        형식: {"type": "select", "label": "apple", "box": [...], "confidence": 0.9}
        """
        try:
            data = json.loads(message)
            if data.get('type') != 'select':
                return

            label = data.get('label', '')
            box   = data.get('box', [])
            conf  = float(data.get('confidence', 0.0))

            if not label:
                return

            msg              = SelectedObject()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.label        = label
            msg.confidence   = conf
            msg.box          = [int(v) for v in box]
            self._pub_selected.publish(msg)

            self.get_logger().info(
                f'[BROWSER SELECT] {label} conf={conf:.3f}')

        except Exception as e:
            self.get_logger().warn(f'DataChannel 메시지 파싱 오류: {e}')


_stream_node: "VisionStreamNode | None" = None


def start_ros2():
    global _stream_node
    rclpy.init()
    _stream_node = VisionStreamNode()
    executor = MultiThreadedExecutor()
    executor.add_node(_stream_node)
    executor.spin()


# ══════════════════════════════════════════════════════════════════════════════
# WebRTC 서버
# ══════════════════════════════════════════════════════════════════════════════

async def run_webrtc():
    config = RTCConfiguration(iceServers=[RTCIceServer(urls='stun:stun.l.google.com:19302'), RTCIceServer(urls='turn:openrelay.metered.ca:80', username='openrelayproject', credential='openrelayproject')])
    pc     = RTCPeerConnection(configuration=config)
    track  = AnnotatedFrameTrack()
    pc.addTrack(track)

    shared._loop = asyncio.get_event_loop()

    @pc.on("datachannel")
    def on_datachannel(channel):
        print(f"\n✅ DataChannel 수신: {channel.label}")
        shared._channel = channel
        shared._loop    = asyncio.get_event_loop()

        @channel.on("open")
        def on_open():
            print("✅ DataChannel 열림")

        @channel.on("message")
        def on_msg(message):
            if _stream_node:
                _stream_node.on_browser_message(message)

        @channel.on("close")
        def on_close():
            print("\n❌ DataChannel 닫힘")
            shared._channel = None

    @pc.on("connectionstatechange")
    async def on_state():
        print(f"\n🔗 연결 상태: {pc.connectionState}")

    count = 0
    while True:
        try:
            resp = requests.get(f"{SIGNALING_URL}/offer/{ROOM}", timeout=5)
            if resp.status_code == 200:
                offer_data = resp.json()
                print()
                break
        except Exception:
            pass
        count += 1
        print(f"\r⏳ [vision] offer 대기 중... {count}s", end='', flush=True)
        await asyncio.sleep(1.0)

    print("✅ Offer 수신!")
    await pc.setRemoteDescription(RTCSessionDescription(
        sdp=offer_data['sdp'], type=offer_data['type']))

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    await asyncio.sleep(1.5)

    requests.post(f"{SIGNALING_URL}/answer", json={
        'room': ROOM,
        'sdp' : pc.localDescription.sdp,
        'type': pc.localDescription.type,
    })
    print("✅ Answer 전송 → 연결 수립!")
    print("📡 스트리밍 중...\n")

    try:
        while True:
            await asyncio.sleep(1.0)
            if pc.connectionState in ('failed', 'closed'):
                break
    except asyncio.CancelledError:
        pass
    finally:
        await pc.close()
        requests.delete(f"{SIGNALING_URL}/clear/{ROOM}")


async def main():
    print("🤖 JARVIS WebRTC Vision 스트리밍 서버")
    print(f"   Room: {ROOM}\n")

    if ROS2_AVAILABLE:
        t = threading.Thread(target=start_ros2, daemon=True)
        t.start()
        print("✅ ROS2 스트림 노드 시작\n")
        await asyncio.sleep(2.0)
    else:
        print("⚠️  테스트 모드\n")

    while True:
        try:
            await run_webrtc()
        except KeyboardInterrupt:
            print("\n👋 종료합니다.")
            break
        except Exception as e:
            print(f"❌ 오류: {e} — 3초 후 재연결...")
            await asyncio.sleep(3)


if __name__ == '__main__':
    asyncio.run(main())
