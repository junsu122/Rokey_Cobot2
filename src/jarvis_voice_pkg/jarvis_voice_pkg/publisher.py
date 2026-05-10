#!/usr/bin/env python3
"""
publisher.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ROS2 토픽 발행 + 콘솔 로그 모듈

발행 토픽:
    /voice_command   : STT 결과 텍스트
    /intent_result   : Intent JSON 전체
    /tts_output      : TTS 메시지
    /voice_intent    : Vision 연동용 (bring_object intent 시)
    /scan_request    : 작업공간 스캔 요청 + 취소 명령 (cancel intent 시 포함)

구독 토픽:
    /object_not_found : vision_node에서 물체 미감지 시 수신
    /scan_result      : scan_node에서 스캔 결과 수신

스캔 취소:
    /scan_request → {"action": "cancel", "reason": "user_cancel"}
"""

import json
import queue
import threading
import time
from datetime import datetime
from jarvis_voice_pkg.config import Config, OBJECT_NAMES_KR

# ── ROS2 (선택적) ─────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# ── Vision 연동 대상 intent ───────────────────────────────────────────────────
# going_out / take_medicine 도 최종적으로 물건을 집어야 하므로 포함
ROBOT_INTENTS = ("bring_object", "going_out", "take_medicine")

# 스캔 타임아웃 (초)
SCAN_TIMEOUT_SEC = 60.0


def get_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _josa_eul_reul(word: str) -> str:
    """받침 있으면 '을', 없으면 '를'"""
    if not word:
        return "을"
    last = word[-1]
    if '가' <= last <= '힣':
        return "을" if (ord(last) - 0xAC00) % 28 > 0 else "를"
    return "을"


class DualOutputPublisher:

    def __init__(self):
        self._node                = None
        self._voice_pub           = None
        self._intent_pub          = None
        self._tts_pub             = None
        self._voice_intent_pub    = None
        self._scan_request_pub    = None

        # ── 스캔 상태 ─────────────────────────────────────────────────────
        self.is_scanning   : bool        = False
        self.scan_result   : dict | None = None
        self.scan_cancelled: bool        = False
        self._scan_event   = threading.Event()

        # ── 멀티 픽 큐 ────────────────────────────────────────────────────
        self._is_picking        : bool        = False
        self._prev_is_picking   : bool        = False   # 이전 상태 (전환 감지용)
        self._scan_pick_pending : bool        = False   # 스캔 트리거 픽 완료 대기 중
        self._queue_busy        : bool        = False   # 큐 워커가 아이템 처리 중
        self._pick_queue        : queue.Queue = queue.Queue()
        self._queue_thread = threading.Thread(
            target=self._pick_queue_worker, daemon=True)
        self._queue_thread.start()

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
            self._node.create_subscription(
                Bool, "/is_picking",
                self._is_picking_cb, 10)

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

    # ── 멀티 픽 큐 워커 ──────────────────────────────────────────────────────

    def _is_picking_cb(self, msg: Bool):
        prev = self._is_picking
        self._is_picking = msg.data

        # 픽 시작 시 진행 중인 스캔 자동 취소
        if msg.data and self.is_scanning and not self._scan_pick_pending:
            cancel_payload = self._with_timestamp({
                "action": "cancel",
                "reason": "pick_started",
            })
            self._pub(self._scan_request_pub, cancel_payload)
            print("🛑 [SCAN AUTO-CANCEL] 픽 시작 감지 → 스캔 자동 취소")

        # 스캔-트리거 픽 완료 감지 → is_scanning 해제
        if prev and not msg.data and self._scan_pick_pending:
            self._scan_pick_pending = False
            self.is_scanning = False
            self._scan_event.set()
            print("✅ [SCAN-PICK 완료] 픽 완료 → is_scanning=False, 다음 타겟 허용")

    def is_robot_busy(self) -> bool:
        """로봇이 픽/스캔/큐 작업 중이면 True (새 음성 명령 차단용)"""
        return (self._is_picking
                or self.is_scanning
                or self._queue_busy
                or not self._pick_queue.empty())

    def _pick_queue_worker(self):
        """큐에 쌓인 target_object를 완전히 하나씩 순서대로 처리"""
        while True:
            try:
                item = self._pick_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            self._queue_busy = True
            try:
                # 앞선 픽/스캔이 완전히 끝날 때까지 대기
                prior_deadline = time.time() + 180.0
                while (self._is_picking or self.is_scanning) and time.time() < prior_deadline:
                    time.sleep(0.3)

                # 이 타겟 발행
                self._pub(self._voice_intent_pub, self._with_timestamp(item))
                print(f"  📡 [멀티픽 큐] /voice_intent 발행: {item.get('target_object')}")

                # 시스템이 이 타겟 처리를 시작할 때까지 대기 (최대 10s)
                # — YOLO hover_sec(2s) + vision_node 처리 시간 고려
                startup_deadline = time.time() + 10.0
                while (not self._is_picking and not self.is_scanning
                       and time.time() < startup_deadline):
                    time.sleep(0.2)

                # 이 타겟 처리 완료까지 대기
                # (_scan_pick_pending=True 동안 is_scanning=True 유지 → 스캔-트리거 픽 포함)
                completion_deadline = time.time() + 180.0
                while (self._is_picking or self.is_scanning) and time.time() < completion_deadline:
                    time.sleep(0.3)
            finally:
                self._queue_busy = False

    # ── 구독 콜백 ─────────────────────────────────────────────────────────────

    def _object_not_found_cb(self, msg: String):
        """
        vision_node로부터 물체 미감지 수신
        → TTS 안내 + /scan_request 발행 + 스캔 대기 상태 진입
        """
        # 스캔 결과 처리 중이거나 픽 진행 중이면 재스캔 억제
        # (센터링 후 YOLO 재탐지 실패 시 무한 스캔 루프 방지)
        if self._is_picking or self.is_scanning:
            print("⚠️  [object_not_found] 스캔/픽 진행 중 → 재스캔 스킵")
            return
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
                not_found_kr = [OBJECT_NAMES_KR.get(o, o) for o in not_found]
                items_kr = ", ".join(not_found_kr)
                josa = _josa_eul_reul(not_found_kr[-1] if not_found_kr else "")
                self._say(
                    f"{items_kr}{josa} 찾을 수 없어요. "
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
                found_labels       = [obj["label"] for obj in found_objects]
                voice_intent_sent  = data.get("voice_intent_sent", False)
                print(f"  ✅ found       : {found_labels}")

                found_kr = [OBJECT_NAMES_KR.get(l, l) for l in found_labels]
                josa = _josa_eul_reul(found_kr[-1] if found_kr else "")
                self._say(
                    f"{', '.join(found_kr)}{josa} 찾았어요! 가져다 드릴게요.")

                # 스캔 트리거 픽이 완료될 때까지 is_scanning=True 유지
                self._scan_pick_pending = True

                if voice_intent_sent:
                    # workspace_scan_node가 이미 직접 발행 → 중복 방지
                    print("  📡 /voice_intent 이미 발행됨 (scan_node 직접) → 재발행 스킵")
                else:
                    # /voice_intent 재발행 → vision_node pick 수행
                    for obj in found_objects:
                        self._pub(self._voice_intent_pub, self._with_timestamp({
                            "action"       : "bring_object",
                            "target_object": [obj["label"]],
                            "urgency"      : "normal",
                            "from_scan"    : True,
                            "best_pose"    : obj.get("best_pose"),
                        }))
                        print(f"  📡 /voice_intent 재발행: {obj['label']}")

            else:
                target_kr = [OBJECT_NAMES_KR.get(o, o) for o in target_objects]
                items_kr  = ", ".join(target_kr)
                josa = _josa_eul_reul(target_kr[-1] if target_kr else "")
                print(f"  ❌ not found   : {target_objects}")
                self._say(
                    f"작업 공간을 전부 탐색했지만 "
                    f"{items_kr}{josa} 찾을 수 없었어요.")

            print(f"{'═'*56}")

        except Exception as e:
            print(f"❌ [scan_result 파싱 오류] {e}")

        finally:
            # 스캔 완료 → 대기 해제
            # _scan_pick_pending=True 면 is_scanning은 픽 완료 시 해제
            if not self._scan_pick_pending:
                self.is_scanning = False
            self._scan_event.set()  # wait_for_scan_result() 블록 해제용

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

            # ── cancel intent → /scan_request 로 취소 명령 발행 ──────────
            if is_cancel:
                self._pub(self._scan_request_pub, self._with_timestamp({
                    "action": "cancel",
                    "reason": "user_cancel",
                }))

            if is_robot:
                objects = intent_result.get("target_objects") or []
                if not objects:
                    t = intent_result.get("target_object")
                    objects = [t] if t else []

                for obj in objects:
                    payload = {
                        # vision_node 의 _is_bring_action() 체크 통과를 위해
                        # going_out / take_medicine 도 bring_object 로 통일
                        "action"       : "bring_object",
                        "target_object": [obj],
                        "urgency"      : intent_result.get("urgency"),
                        "origin_intent": intent_name,
                    }
                    # 모두 큐를 통해 순서 보장 (첫 번째도 즉시 발행하지 않음)
                    self._pick_queue.put(payload)
                    print(f"  🗂️  [멀티픽 큐 적재] {obj}")

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
            print(f"  {Config.TOPIC_SCAN_REQUEST:<26} → cancel (user_cancel)")
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