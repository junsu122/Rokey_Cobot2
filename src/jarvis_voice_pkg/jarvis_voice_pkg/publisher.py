#!/usr/bin/env python3
"""
publisher.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ROS2 토픽 발행 + 콘솔 로그 모듈

발행 토픽:
    /voice_command   : STT 결과 텍스트
    /intent_result   : Intent JSON 전체
    /tts_output      : TTS 메시지
    /voice_cancel    : 취소 신호 (cancel intent 시)
    /voice_intent    : Vision 연동용 (bring_* intent 시)
    /scan_request    : 작업공간 스캔 요청 + 취소 명령

구독 토픽:
    /object_not_found : vision_node에서 물체 미감지 시 수신
    /scan_result      : scan_node에서 스캔 결과 수신

스캔 취소:
    /scan_request → {"action": "cancel", "reason": "user_cancel"}
"""

import json
import threading
from datetime import datetime
from jarvis_voice_pkg.config import Config

# ── ROS2 (선택적) ─────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# ── Vision 연동 대상 intent ───────────────────────────────────────────────────
ROBOT_INTENTS = ("bring_water", "bring_medicine", "bring_food")

# 스캔 타임아웃 (초)
SCAN_TIMEOUT_SEC = 60.0


def get_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class DualOutputPublisher:

    def __init__(self):
        self._node                = None
        self._voice_pub           = None
        self._intent_pub          = None
        self._tts_pub             = None
        self._cancel_pub          = None
        self._voice_intent_pub    = None
        self._scan_request_pub    = None

        # ── 스캔 상태 ─────────────────────────────────────────────────────
        self.is_scanning   : bool        = False
        self.scan_result   : dict | None = None
        self.scan_cancelled: bool        = False
        self._scan_event   = threading.Event()

        if ROS2_AVAILABLE:
            rclpy.init()
            self._node = Node("jarvis_voice_intent_node")

            # ── 발행 토픽 ─────────────────────────────────────────────────
            self._voice_pub  = self._node.create_publisher(
                String, Config.TOPIC_VOICE_CMD,    10)
            self._intent_pub = self._node.create_publisher(
                String, Config.TOPIC_INTENT,       10)
            self._tts_pub    = self._node.create_publisher(
                String, Config.TOPIC_TTS,          10)
            self._cancel_pub = self._node.create_publisher(
                String, Config.TOPIC_CANCEL,       10)
            self._voice_intent_pub = self._node.create_publisher(
                String, Config.TOPIC_VOICE_INTENT, 10)
            self._scan_request_pub = self._node.create_publisher(
                String, Config.TOPIC_SCAN_REQUEST, 10)

            # ── 구독 토픽 ─────────────────────────────────────────────────
            self._node.create_subscription(
                String, Config.TOPIC_OBJECT_NOT_FOUND,
                self._object_not_found_cb, 10)
            self._node.create_subscription(
                String, Config.TOPIC_SCAN_RESULT,
                self._scan_result_cb, 10)

            print("✅ [ROS2] jarvis_voice_intent_node 초기화 완료")
        else:
            print("⚠️  [ROS2 없음] 콘솔 전용 모드로 실행")

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────────

    def _pub(self, publisher, data: str):
        if publisher:
            m = String()
            m.data = data
            publisher.publish(m)

    def _with_timestamp(self, payload: dict) -> str:
        payload["timestamp"] = get_timestamp()
        return json.dumps(payload, ensure_ascii=False)

    def _say(self, text: str):
        """TTS 발행 헬퍼"""
        print(f"🔊 [TTS] {text}")
        self._pub(self._tts_pub, self._with_timestamp({"message": text}))

    # ── 구독 콜백 ─────────────────────────────────────────────────────────────

    def _object_not_found_cb(self, msg: String):
        """
        vision_node로부터 물체 미감지 수신
        → TTS 안내 + /scan_request 발행 + 스캔 대기 상태 진입
        """
        try:
            data      = json.loads(msg.data)
            not_found = data.get("not_found", [])
            found     = data.get("found",     [])
            ts        = get_timestamp()

            print(f"\n⚠️  [물체 미감지]  {ts}")
            print(f"     감지됨  : {found}")
            print(f"     미감지  : {not_found}")

            if not_found:
                items = ", ".join(not_found)

                # TTS 안내
                self._say(
                    f"{items}을 찾을 수 없어요. "
                    f"작업 공간을 탐색할게요. "
                    f"취소하려면 말씀해 주세요."
                )

                # 스캔 상태 진입
                self.is_scanning    = True
                self.scan_result    = None
                self.scan_cancelled = False
                self._scan_event.clear()

                # /scan_request 발행 (스캔 시작 명령)
                scan_payload = self._with_timestamp({
                    "action"        : "start",
                    "target_objects": not_found,
                    "reason"        : "object_not_found",
                })
                self._pub(self._scan_request_pub, scan_payload)
                print(f"🔍 [SCAN REQUEST] 탐색 요청 발행: {not_found}  {ts}")

        except Exception as e:
            print(f"❌ [object_not_found 파싱 오류] {e}")

    def _scan_result_cb(self, msg: String):
        """
        scan_node로부터 스캔 결과 수신
        → 결과 저장 + 스캔 완료 이벤트 발생
        """
        try:
            data           = json.loads(msg.data)
            status         = data.get("status",         "unknown")
            target_objects = data.get("target_objects", [])
            found_objects  = data.get("found_objects",  [])
            ts             = data.get("timestamp",      get_timestamp())

            print(f"\n{'═'*56}")
            print(f"  📥  /scan_result 수신  [{ts}]")
            print(f"{'═'*56}")
            print(f"  status         : {status}")
            print(f"  target_objects : {target_objects}")

            self.scan_result = data

            if status == "found":
                found_labels = [obj["label"] for obj in found_objects]
                print(f"  ✅ found       : {found_labels}")

                self._say(
                    f"{', '.join(found_labels)}을 찾았어요! 가져다 드릴게요.")

                # /voice_intent 재발행 → vision_node pick 수행
                for obj in found_objects:
                    self._pub(self._voice_intent_pub, self._with_timestamp({
                        "action"       : "bring_food",
                        "target_object": [obj["label"]],
                        "urgency"      : "normal",
                        "from_scan"    : True,
                        "best_pose"    : obj.get("best_pose"),
                        "best_bbox"    : obj.get("best_bbox_xyxy"),
                    }))
                    print(f"  📡 /voice_intent 재발행: {obj['label']}")

            else:
                items = ", ".join(target_objects)
                print(f"  ❌ not found   : {target_objects}")
                self._say(
                    f"작업 공간을 전부 탐색했지만 "
                    f"{items}을 찾을 수 없었어요.")

            print(f"{'═'*56}")

        except Exception as e:
            print(f"❌ [scan_result 파싱 오류] {e}")

        finally:
            # 스캔 완료 → 대기 해제
            self.is_scanning = False
            self._scan_event.set()

    def wait_for_scan_result(self, cancel_checker) -> str:
        """
        스캔 결과 대기

        Args:
            cancel_checker: 취소 여부 확인 함수

        Returns:
            "found" | "not_found" | "cancelled" | "timeout"
        """
        import time
        deadline = time.time() + SCAN_TIMEOUT_SEC

        while time.time() < deadline:
            completed = self._scan_event.wait(timeout=0.5)

            if completed:
                result = self.scan_result or {}
                return result.get("status", "not_found")

            # 취소 명령 감지
            if cancel_checker():
                self.scan_cancelled = True
                self.is_scanning    = False

                # ── /scan_request 로 취소 명령 발행 ──────────────────────
                cancel_payload = self._with_timestamp({
                    "action": "cancel",
                    "reason": "user_cancel",
                })
                self._pub(self._scan_request_pub, cancel_payload)
                print(f"🛑 [SCAN CANCEL] /scan_request → cancel 발행")

                self._say("탐색을 취소했어요.")
                self._scan_event.set()
                return "cancelled"

        # 타임아웃
        self.is_scanning = False
        self._say("탐색 시간이 초과됐어요.")
        return "timeout"

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def publish_all(self, voice_text: str, intent_result: dict):
        """ROS2 토픽 발행 + 콘솔 출력 동시 실행"""
        ts          = get_timestamp()
        intent_name = intent_result.get("intent", "?")
        tts_msg     = intent_result.get("response_message", "")
        is_cancel   = (intent_name == "cancel")
        is_robot    = (intent_name in ROBOT_INTENTS)

        if ROS2_AVAILABLE:
            self._pub(self._voice_pub, self._with_timestamp({
                "text": voice_text,
            }))

            intent_result["timestamp"] = ts
            self._pub(self._intent_pub,
                      json.dumps(intent_result, ensure_ascii=False))

            self._pub(self._tts_pub, self._with_timestamp({
                "message": tts_msg,
            }))

            if is_cancel:
                self._pub(self._cancel_pub, self._with_timestamp({
                    "action": "stop",
                    "reason": "user_cancel",
                }))

            if is_robot:
                target = intent_result.get("target_object")
                if isinstance(target, str):
                    target = [target]
                self._pub(self._voice_intent_pub, self._with_timestamp({
                    "action"       : intent_name,
                    "target_object": target,
                    "urgency"      : intent_result.get("urgency"),
                }))

        # 콘솔 출력
        mode = "ROS2 + 콘솔" if ROS2_AVAILABLE else "콘솔"
        W = 58
        print(f"\n{'━'*W}")
        print(f"  📡  출력 모드: {mode}  [{ts}]")
        print(f"{'━'*W}")
        print(f"  {Config.TOPIC_VOICE_CMD:<26} → \"{voice_text}\"")
        print(f"  {Config.TOPIC_INTENT:<26} → intent={intent_name}")
        print(f"  {Config.TOPIC_TTS:<26} → \"{tts_msg}\"")
        if is_cancel:
            print(f"  {Config.TOPIC_CANCEL:<26} → stop (Replanning)")
        if is_robot:
            print(f"  {Config.TOPIC_VOICE_INTENT:<26} → "
                  f"target={intent_result.get('target_object')}")
        print(f"{'━'*W}")

    def shutdown(self):
        if ROS2_AVAILABLE and self._node:
            try:
                self._node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"[shutdown] {e}")
            print("👋 ROS2 노드 종료")


# ══════════════════════════════════════════════════════════════════════════════
# 콘솔 로그 유틸
# ══════════════════════════════════════════════════════════════════════════════

def log_stt(text: str, logprob: float):
    conf_pct = max(0, min(100, int((1 + logprob) * 100)))
    print(f"\n🗣️  [STT 결과]  [{get_timestamp()}]")
    print(f"     텍스트 : \"{text}\"")
    print(f"     신뢰도 : {conf_pct}%  (avg_logprob={logprob:.3f})")


def log_intent(result: dict):
    W = 58
    print("\n" + "═"*W)
    print(f"  🧠  Intent Engine 결과  [{get_timestamp()}]")
    print("═"*W)
    print(f"  Intent        : {result.get('intent')}")
    print(f"  Target Object : {result.get('target_object')}")
    print(f"  Urgency       : {result.get('urgency')}")
    print(f"  Confidence    : {result.get('confidence', 0):.2f}")

    scores   = result.get("scores", {})
    selected = result.get("intent")
    mx       = max(scores.values(), default=1)

    print("\n  📊 Action Scores:")
    for act, sc in sorted(scores.items(), key=lambda x: -x[1]):
        bar = (
            "█" * int(sc / max(mx, 1) * 20) +
            "░" * (20 - int(sc / max(mx, 1) * 20))
        )
        tag = "  ◀ SELECTED" if act == selected else ""
        print(f"    {act:<20} {sc:>3}  {bar}{tag}")

    print("\n  📋 Reason Log:")
    for r in result.get("reason_log", []):
        print(f"    • {r}")
    print(f"\n  🔊 TTS : {result.get('response_message')}")
    print("═"*W)