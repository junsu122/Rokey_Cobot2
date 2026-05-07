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

import os
import sys
import time
import threading
import traceback
import subprocess

from rclpy.executors import MultiThreadedExecutor

from jarvis_voice_pkg.config        import Config
from jarvis_voice_pkg.stt           import LocalWhisperSTT
from jarvis_voice_pkg.recorder      import AudioRecorder
from jarvis_voice_pkg.intent_engine import IntentEngine
from jarvis_voice_pkg.publisher     import DualOutputPublisher, log_stt, log_intent
from jarvis_voice_pkg.weather       import get_weather

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

        print("✅ 초기화 완료\n")

    # ── 추론 모드 선택 ────────────────────────────────────────────────────────

    def _is_emergency(self, voice_text: str) -> bool:
        """긴급 키워드 포함 여부 확인"""
        return any(kw in voice_text for kw in EMERGENCY_KEYWORDS)

    def _analyze(self, voice_text: str) -> dict:
        """
        상황에 따라 추론 모드 선택
        - 긴급 키워드 → analyze_fast() (단일 호출, 빠른 응답)
        - 일반 상황   → analyze()     (3단계 CoT, 정확한 추론)
        """
        if self._is_emergency(voice_text):
            print("🚨 [긴급 감지] 빠른 추론 모드 실행...")
            return self.engine.analyze_fast(
                voice_text,
                detected_objects=self.detected_objects,
                current_action=self.current_action,
                gesture=self.gesture,
            )
        else:
            print("🧠 [3단계 CoT 추론 시작...]")
            return self.engine.analyze(
                voice_text,
                detected_objects=self.detected_objects,
                current_action=self.current_action,
                gesture=self.gesture,
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

            # ── STEP 2: 추론 (긴급/일반 분기) ─────────────────────────────
            result = self._analyze(voice_text)

            # ── STEP 3: 콘솔 상세 출력 ───────────────────────────────────
            log_intent(result)

            # ── STEP 4: intent별 후처리 ───────────────────────────────────
            intent = result.get("intent")

            if intent == "weather_query":
                print("🌤️  [날씨 API 호출 중...]")
                result["response_message"] = get_weather()

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

            # ── STEP 6: ROS2 & 콘솔 동시 발행 ────────────────────────────
            self.publisher.publish_all(voice_text, result)

            # ── STEP 7: 상태 업데이트 ────────────────────────────────────
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