#!/usr/bin/env python3
"""
voice_intent_node.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
JARVIS 메인 노드

파이프라인:
    마이크 → STT → 긴급 키워드 체크
                  ↓ 긴급     ↓ 일반
            analyze_fast  analyze (3단계 CoT)
                  ↓              ↓
            후처리 → ROS2 & 콘솔 출력

스캔 대기 모드:
    물체 미감지 → scan_request 발행 → 스캔 결과 대기
    대기 중 취소 명령 반복 수신 가능

TUI:
    노드 시작 시 jarvis_monitor.py 새 터미널에서 자동 실행
    노드 종료 시 TUI 프로세스도 자동 종료
"""

import json
import os
import sys
import time
import base64
import threading
import traceback
import subprocess

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image

from jarvis_voice_pkg.config        import Config
from jarvis_voice_pkg.stt           import LocalWhisperSTT
from jarvis_voice_pkg.recorder      import AudioRecorder
from jarvis_voice_pkg.intent_engine import IntentEngine
from jarvis_voice_pkg.publisher     import DualOutputPublisher, log_stt, log_intent
from jarvis_voice_pkg.weather       import get_weather, get_weather_detail

SEARCH_CHECK_SEC    = 5.0   # 두리번 감지 주기
SEARCH_COOLDOWN_SEC = 20.0  # 감지 후 재감지 억제 시간

# 취소로 판단할 키워드
CANCEL_KEYWORDS = ["취소", "아니야", "됐어", "멈춰", "그만", "중단", "싫어"]

# 긴급 키워드 (fast 추론 사용)
EMERGENCY_KEYWORDS = ["도와줘", "응급", "119", "살려줘", "쓰러"]

# TUI 프로세스
_tui_process: subprocess.Popen | None = None


# ══════════════════════════════════════════════════════════════════════════════
# TUI 자동 실행 / 종료
# ══════════════════════════════════════════════════════════════════════════════

def launch_tui() -> subprocess.Popen | None:
    monitor_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "jarvis_monitor.py"
    )
    if not os.path.exists(monitor_path):
        print(f"⚠️  TUI 모니터 파일 없음: {monitor_path}")
        return None

    try:
        proc = subprocess.Popen([
            "gnome-terminal", "--title=JARVIS TUI Monitor",
            "--", "python3", monitor_path
        ])
        print("✅ [TUI] JARVIS Monitor 자동 실행 (gnome-terminal)")
        return proc
    except FileNotFoundError:
        pass

    try:
        proc = subprocess.Popen([
            "xterm", "-title", "JARVIS TUI Monitor",
            "-e", f"python3 {monitor_path}"
        ])
        print("✅ [TUI] JARVIS Monitor 자동 실행 (xterm)")
        return proc
    except FileNotFoundError:
        pass

    try:
        proc = subprocess.Popen(
            ["python3", monitor_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print("✅ [TUI] JARVIS Monitor 백그라운드 실행")
        return proc
    except Exception as e:
        print(f"⚠️  TUI 자동 실행 실패: {e}")
        return None


def shutdown_tui(proc: subprocess.Popen | None):
    if proc is None:
        return
    try:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
            print("✅ [TUI] JARVIS Monitor 종료됨")
    except Exception as e:
        print(f"⚠️  TUI 종료 오류: {e}")


# ══════════════════════════════════════════════════════════════════════════════
# 메인 노드
# ══════════════════════════════════════════════════════════════════════════════

class JARVISVoiceNode:

    def __init__(self):
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise EnvironmentError(
                "❌ OPENAI_API_KEY 환경변수를 설정하세요.\n"
                "   export OPENAI_API_KEY='sk-...'"
            )

        print("\n🤖 JARVIS Voice Intent Node 초기화 중...")
        self.stt       = LocalWhisperSTT()
        self.engine    = IntentEngine(api_key)
        self.publisher = DualOutputPublisher()
        self.recorder  = AudioRecorder()

        # ── ROS2 spin 스레드 ──────────────────────────────────────────────
        if self.publisher._node:
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self.publisher._node)
            self._spin_thread = threading.Thread(
                target=self._executor.spin, daemon=True)
            self._spin_thread.start()
            print("✅ [ROS2] spin 스레드 시작")

        # ── 외부 모듈 연동 상태 ───────────────────────────────────────────
        self.detected_objects : list[str]  = []
        self.current_action   : str | None = None
        self.gesture          : str | None = None

        # ── 스캔 중 취소 감지 플래그 ──────────────────────────────────────
        self._cancel_detected : bool = False

        # ── 웹캠 프레임 (두리번 감지용 — 사용자 방향) ────────────────────
        self._latest_frame     : np.ndarray | None = None
        self._frame_lock       = threading.Lock()
        self._cv_bridge        = CvBridge()
        self._search_next_time : float = 0.0

        # ── 테이블 카메라 프레임 (VLM 외출 준비용 — 테이블 방향) ──────────
        self._latest_table_frame : np.ndarray | None = None
        self._table_frame_lock   = threading.Lock()

        if self.publisher._node:
            self.publisher._node.create_subscription(
                Image, '/webcam/image_raw',
                self._webcam_cb, 1)
            self.publisher._node.create_subscription(
                Image, '/camera/camera/color/image_raw',
                self._table_cam_cb, 1)

        threading.Thread(target=self._searching_detector, daemon=True).start()

        print("✅ 초기화 완료\n")

    # ── 웹캠 콜백 (사용자 방향 — 두리번거림 감지용) ──────────────────────────

    def _webcam_cb(self, msg: Image):
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            print(f"⚠️  [웹캠 콜백] {e}")

    # ── 테이블 카메라 콜백 (로봇 RealSense — 테이블 물건 VLM용) ─────────────

    def _table_cam_cb(self, msg: Image):
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._table_frame_lock:
                self._latest_table_frame = frame
        except Exception as e:
            print(f"⚠️  [테이블 카메라 콜백] {e}")

    def _get_table_image_b64(self) -> str | None:
        """로봇 RealSense(테이블 방향) 최신 프레임을 JPEG base64로 변환"""
        with self._table_frame_lock:
            frame = self._latest_table_frame.copy() \
                if self._latest_table_frame is not None else None
        if frame is None:
            return None
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return base64.b64encode(buf.tobytes()).decode()

    def _scan_scene_with_vlm(self) -> list[str]:
        """
        음성 명령 직전 테이블 씬을 VLM(GPT-4o Vision)으로 1회 파악.
        detected_objects 갱신용 — 호출 비용 최소화를 위해 음성 입력 시에만 실행.
        """
        import openai
        image_b64 = self._get_table_image_b64()
        if image_b64 is None:
            return []

        detectable = (
            'umbrella, bag, apple, banana, pill, phone, '
            'juice, sun_cream, water, candy, mask, bread'
        )
        try:
            client = openai.OpenAI(api_key=os.environ.get('OPENAI_API_KEY', ''))
            resp = client.chat.completions.create(
                model='gpt-4o',
                messages=[{
                    'role': 'user',
                    'content': [
                        {'type': 'image_url',
                         'image_url': {'url': f'data:image/jpeg;base64,{image_b64}'}},
                        {'type': 'text', 'text': (
                            f'테이블 위에 있는 물건을 아래 목록에서 찾아 JSON으로만 답변.\n'
                            f'목록: {detectable}\n'
                            f'{{"objects": ["영문키1", "영문키2"]}} — 없으면 {{"objects": []}}'
                        )},
                    ],
                }],
                max_tokens=80,
                response_format={'type': 'json_object'},
            )
            objects = json.loads(resp.choices[0].message.content).get('objects', [])
            print(f"👁️  [VLM 씬 파악] {objects}")
            return objects
        except Exception as e:
            print(f"⚠️  [VLM 씬 파악 오류] {e}")
            return []

    # ── 두리번거리는 감지 (백그라운드 스레드) ────────────────────────────────

    def _searching_detector(self):
        """5초마다 웹캠 프레임을 GPT-4o로 분석 — 두리번거리면 TTS로 알림"""
        print("👀 [두리번 감지] 백그라운드 스레드 시작")
        while True:
            time.sleep(SEARCH_CHECK_SEC)

            if time.time() < self._search_next_time:
                continue
            if self.publisher.is_scanning:
                continue

            with self._frame_lock:
                frame = self._latest_frame.copy() \
                    if self._latest_frame is not None else None
            if frame is None:
                continue

            try:
                _, buf = cv2.imencode('.jpg', frame,
                                      [cv2.IMWRITE_JPEG_QUALITY, 75])
                b64 = base64.b64encode(buf.tobytes()).decode()

                import openai, os
                client = openai.OpenAI(api_key=os.environ.get('OPENAI_API_KEY', ''))
                resp = client.chat.completions.create(
                    model='gpt-4o',
                    messages=[{
                        'role': 'user',
                        'content': [
                            {'type': 'image_url',
                             'image_url': {'url': f'data:image/jpeg;base64,{b64}'}},
                            {'type': 'text', 'text': (
                                '이미지 속 사람이 무언가를 찾는 듯 두리번거리거나 '
                                '주변을 살피고 있나요? '
                                'JSON으로만 답변: '
                                '{"is_searching": true or false, "reason": "한 줄"}'
                            )},
                        ],
                    }],
                    max_tokens=80,
                    response_format={'type': 'json_object'},
                )
                import json
                data = json.loads(resp.choices[0].message.content)
                if data.get('is_searching'):
                    print(f"👀 [두리번 감지] {data.get('reason')}")
                    self.publisher._say('뭔가 찾으시나요? 말씀해 주세요!')
                    self._search_next_time = time.time() + SEARCH_COOLDOWN_SEC

            except Exception as e:
                print(f"⚠️  [두리번 감지 오류] {e}")

    # ── 추론 모드 선택 ────────────────────────────────────────────────────────

    def _is_emergency(self, voice_text: str) -> bool:
        """긴급 키워드 포함 여부 확인"""
        return any(kw in voice_text for kw in EMERGENCY_KEYWORDS)

    def _get_image_b64(self) -> str | None:
        """현재 웹캠 프레임을 base64로 변환"""
        with self._frame_lock:
            frame = self._latest_frame.copy() \
                if self._latest_frame is not None else None
        if frame is None:
            return None
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return base64.b64encode(buf.tobytes()).decode()

    # ── 외출 짐 계획 (o4-mini + 날씨 + 씬) ──────────────────────────────────

    def _plan_going_out_items(self) -> tuple[list[str], str]:
        """
        o4-mini로 날씨 + 현재 씬(카메라+감지 물건) 기반 외출 짐 계획 수립

        Returns:
            (items, tts_message)
            items: ["bag", "umbrella", ...] 순서 포함
        """
        import json, openai

        weather = get_weather_detail()

        weather_ctx = (
            f"기온: {weather.get('temp', '?')}°C, "
            f"날씨: {weather.get('desc', '?')}, "
            f"UV지수: {weather.get('uv', '?')}({weather.get('uv_level', '?')}), "
            f"미세먼지PM10: {weather.get('pm10', '?')}({weather.get('pm_level', '?')}), "
            f"비예보: {'있음' if weather.get('need_umbrella') else '없음'}, "
            f"자외선강함: {'예' if weather.get('need_sunscreen') else '아니오'}, "
            f"미세먼지나쁨: {'예' if weather.get('need_mask') else '아니오'}, "
            f"추운날씨: {'예' if weather.get('cold') else '아니오'}"
        )

        # 테이블 카메라(RealSense) 이미지로 VLM 입력 — 웹캠(사용자 방향) 아님
        image_b64 = self._get_table_image_b64()
        scene_ctx = (
            f"현재 작업 테이블 위 YOLO 감지 물건: "
            f"{self.detected_objects if self.detected_objects else '없음'}"
        )

        prompt = (
            f"시니어 보조 협동로봇 JARVIS입니다. 외출 준비를 도와야 합니다.\n\n"
            f"[날씨 정보]\n{weather_ctx}\n\n"
            f"[작업 테이블 현황]\n{scene_ctx}\n"
            f"{'(테이블 카메라 이미지 첨부 — 실제 테이블 위 물건 직접 확인)' if image_b64 else ''}\n\n"
            f"로봇이 가져올 수 있는 물건: bag(가방), umbrella(우산), sun_cream(썬크림), water(물)\n"
            f"bag은 반드시 포함. 날씨와 테이블 위 실제 물건 현황을 함께 고려해 나머지 추가 여부를 판단해줘.\n\n"
            f"JSON으로만 답변:\n"
            f'{{\"items\": [\"bag\", ...], \"reason\": \"판단 근거 한 줄\", '
            f'\"tts\": \"사용자에게 전달할 자연스러운 한국어 메시지\"}}'
        )

        content: list | str = prompt
        if image_b64:
            content = [
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
                {"type": "text", "text": prompt},
            ]

        try:
            client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY", ""))
            resp = client.chat.completions.create(
                model="o4-mini",
                messages=[{"role": "user", "content": content}],
                max_tokens=300,
            )
            raw = resp.choices[0].message.content
            s = raw.find("{")
            e = raw.rfind("}") + 1
            data = json.loads(raw[s:e]) if s != -1 and e > s else {}

            items  = data.get("items",  ["bag"])
            tts    = data.get("tts",    "외출 준비물을 챙겨드릴게요!")
            reason = data.get("reason", "")
            print(f"🧳 [외출 짐 계획] items={items}  이유: {reason}")
            return items, tts

        except Exception as e:
            print(f"⚠️  [외출 짐 계획 오류] {e}")
            return ["bag"], "외출 가방을 가져다드릴게요!"

    def _analyze(self, voice_text: str) -> dict:
        """
        상황에 따라 추론 모드 선택
        - 긴급 키워드 → analyze_fast() (단일 호출, 빠른 응답)
        - 일반 상황   → analyze()     (3단계 CoT + VLM, 정확한 추론)
        """
        image_b64 = self._get_image_b64()

        if self._is_emergency(voice_text):
            print("🚨 [긴급 감지] 빠른 추론 모드 실행...")
            return self.engine.analyze_fast(
                voice_text,
                detected_objects=self.detected_objects,
                current_action=self.current_action,
                gesture=self.gesture,
                image_b64=image_b64,
            )
        else:
            print("🧠 [3단계 CoT 추론 시작...]")
            return self.engine.analyze(
                voice_text,
                detected_objects=self.detected_objects,
                current_action=self.current_action,
                gesture=self.gesture,
                image_b64=image_b64,
            )

    # ── 스캔 대기 모드 ────────────────────────────────────────────────────────

    def _listen_for_cancel(self) -> bool:
        try:
            audio = self.recorder.record()
            if self.recorder.is_silence(audio):
                return False

            voice_text, _ = self.stt.transcribe(audio)
            if not voice_text:
                return False

            print(f"🎤 [스캔 중 음성]: \"{voice_text}\"")

            if any(kw in voice_text for kw in CANCEL_KEYWORDS):
                print("🛑 [취소 명령 감지]")
                self._cancel_detected = True
                return True
            else:
                print("⚠️  취소 명령이 아니에요. 탐색 계속 진행 중...")
                return False

        except Exception as e:
            print(f"❌ [스캔 중 STT 오류] {e}")
            return False

    def _wait_for_scan_with_cancel(self):
        def cancel_checker() -> bool:
            return self._cancel_detected

        result_status = self.publisher.wait_for_scan_result(cancel_checker)
        print(f"🔍 [스캔 완료] 결과: {result_status}")
        self.current_action = None
        return result_status

    # ── 핵심 파이프라인 ───────────────────────────────────────────────────────

    def process_once(self, test_text: str | None = None) -> dict | None:
        """
        1회 처리: 마이크 → STT → 추론 → 후처리 → 출력

        긴급 상황: analyze_fast() → 빠른 응답
        일반 상황: analyze()     → 3단계 CoT 추론
        """
        try:
            # ── STEP 1: 음성 입력 ─────────────────────────────────────────
            if test_text is not None:
                voice_text, logprob = test_text, 0.0
                print(f'\n💬 [테스트 입력] "{voice_text}"')
            else:
                audio = self.recorder.record()
                if self.recorder.is_silence(audio):
                    print("🔇 묵음 — 스킵")
                    return None

                print("📝 [Whisper 로컬 추론 중...]")
                voice_text, logprob = self.stt.transcribe(audio)
                if not voice_text:
                    print("⚠️  STT 결과 없음 — 스킵")
                    return None
                log_stt(voice_text, logprob)

            # ── STEP 2: 씬 파악 (VLM 1회 호출) ──────────────────────────
            print("👁️  [VLM 씬 파악 중...]")
            self.detected_objects = self._scan_scene_with_vlm()

            # ── STEP 3: 추론 (긴급/일반 분기) ─────────────────────────────
            result = self._analyze(voice_text)

            # ── STEP 3: 콘솔 상세 출력 ───────────────────────────────────
            log_intent(result)

            # ── STEP 4: intent별 후처리 ───────────────────────────────────
            intent = result.get("intent")

            if intent == "weather_query":
                print("🌤️  [날씨 API 호출 중...]")
                result["response_message"] = get_weather()

            elif intent == "going_out":
                print("🚪 [외출 준비 — o4-mini 날씨+씬 기반 짐 계획 중...]")
                items, tts = self._plan_going_out_items()
                result["target_objects"]   = items
                result["target_object"]    = items[0] if items else None
                result["response_message"] = tts

            elif intent == "general_query":
                print("💬 [일반 질문 — GPT 답변 생성 중...]")
                answer = self.engine.answer_general(voice_text)
                result["response_message"] = answer
                print(f"💡 [GPT 답변] {answer}")

            # ── STEP 5: 신뢰도 체크 ──────────────────────────────────────
            if result.get("confidence", 0) < Config.CONFIDENCE_MIN:
                print("⚠️  신뢰도 낮음 — 재확인 요청")
                result["response_message"] = (
                    "죄송합니다, 잘 못 들었어요. 다시 말씀해 주세요."
                )

            # ── STEP 6: 발행 순서 최적화 (감지된 물건 먼저, 없는 물건 나중) ──
            if intent in ("bring_object", "going_out", "take_medicine"):
                objs = result.get("target_objects") or []
                if len(objs) > 1:
                    visible  = [o for o in objs if o in self.detected_objects]
                    missing  = [o for o in objs if o not in self.detected_objects]
                    reordered = visible + missing
                    if reordered != objs:
                        print(f"🔀 [픽 순서 재정렬] {objs} → {reordered}  "
                              f"(감지={visible}, 미감지={missing})")
                        result["target_objects"] = reordered
                        result["target_object"]  = reordered[0] if reordered else None

            # ── STEP 7: ROS2 & 콘솔 동시 발행 ────────────────────────────
            self.publisher.publish_all(voice_text, result)

            # ── STEP 8: 상태 업데이트 ────────────────────────────────────
            if intent == "cancel":
                print("🔄 Replanning: current_action 초기화")
                self.current_action = None
            elif intent not in ("unknown", "weather_query", "general_query", None):
                self.current_action = intent

            self.gesture = None

            return result

        except Exception as e:
            print(f"❌ [ERROR] {e}")
            traceback.print_exc()
            return None

    # ── 실행 모드 ─────────────────────────────────────────────────────────────

    def run_loop(self):
        """마이크 VAD 연속 루프 (실제 운용)"""
        print("🤖 JARVIS 시작 — Ctrl+C 종료\n")
        try:
            while True:

                # ── 스캔 대기 중이면 취소 명령 반복 수신 ─────────────────
                if self.publisher.is_scanning:
                    print("\n🔍 [스캔 진행 중] 취소 명령을 말씀해 주세요...")
                    self._cancel_detected = False

                    while self.publisher.is_scanning:
                        cancelled = self._listen_for_cancel()
                        if cancelled:
                            self._wait_for_scan_with_cancel()
                            break
                        if not self.publisher.is_scanning:
                            break

                    continue

                # ── 픽/큐 진행 중이면 취소 명령만 허용 ──────────────────
                if self.publisher.is_robot_busy():
                    print("\n⏳ [로봇 작업 중] 취소 명령만 말씀해 주세요...")

                    while self.publisher.is_robot_busy():
                        cancelled = self._listen_for_cancel()
                        if cancelled:
                            # 큐에 남은 작업 모두 제거
                            cleared = 0
                            while not self.publisher._pick_queue.empty():
                                try:
                                    self.publisher._pick_queue.get_nowait()
                                    cleared += 1
                                except Exception:
                                    break
                            print(f"🛑 [큐 초기화] 대기 작업 {cleared}개 취소")
                            self.publisher._say(
                                "알겠어요. 현재 작업이 끝나는 대로 멈출게요.")
                            break
                        if not self.publisher.is_robot_busy():
                            break

                    continue

                # ── 일반 음성 명령 처리 ───────────────────────────────────
                self.process_once()
                time.sleep(0.3)

        except KeyboardInterrupt:
            print("\n\n👋 종료합니다.")
        finally:
            self.publisher.shutdown()

    def run_test(self):
        """텍스트 테스트 모드"""
        scenarios = [
            ("어지러워",                ["water"],             None,            None,     "긴급 - 물 요청"),
            ("도와줘",                  [],                    None,            None,     "긴급 - 응급"),
            ("약 줘",                   ["pill", "water"],     None,            None,     "약 복용 보조"),
            ("나갈 준비 도와줘",         ["bag", "phone"],      None,            None,     "외출 준비"),
            ("아니야",                  ["water"],             "bring_object",  "reject", "Replanning"),
            ("사과 줘",                 ["apple", "banana"],   None,            None,     "음식 요청"),
            ("오늘 날씨 어때?",          [],                    None,            None,     "날씨 조회"),
            ("아인슈타인이 누구야?",     [],                    None,            None,     "일반 질문"),
        ]

        print(f"\n🧪 JARVIS 테스트 모드 — {len(scenarios)}개 시나리오\n")
        for i, (text, objs, action, gesture, desc) in enumerate(scenarios, 1):
            print(f"\n{'─'*58}")
            print(f"  [시나리오 {i}] {desc}")

            self.detected_objects = objs
            self.current_action   = action
            self.gesture          = gesture

            self.process_once(test_text=text)
            time.sleep(0.3)

        self.publisher.shutdown()
        print("\n✅ 전체 테스트 완료")


# ══════════════════════════════════════════════════════════════════════════════
# 실행
# ══════════════════════════════════════════════════════════════════════════════

def main():
    global _tui_process

    is_test = len(sys.argv) > 1 and sys.argv[1] == "test"

    if not is_test:
        _tui_process = launch_tui()

    node = JARVISVoiceNode()
    try:
        if is_test:
            node.run_test()
        else:
            node.run_loop()
    finally:
        shutdown_tui(_tui_process)
        _tui_process = None


if __name__ == "__main__":
    main()