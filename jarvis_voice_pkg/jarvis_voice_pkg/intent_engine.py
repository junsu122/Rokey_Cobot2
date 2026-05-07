#!/usr/bin/env python3
"""
intent_engine.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GPT-4o 기반 3단계 Chain of Thought 의도 추론 엔진

[추론 단계]
  STEP 1 — Perception  : 상황 파악
  STEP 2 — Reasoning   : 의도 추론
  STEP 3 — Planning    : 행동 계획 수립
  STEP 4 — Final       : 최종 JSON 출력

- analyze()        : 전체 파이프라인 실행
- answer_general() : general_query 전용 자유 답변
"""

import json
from openai import OpenAI
from jarvis_voice_pkg.config import Config, SYSTEM_PROMPT, GENERAL_QUERY_PROMPT


# ══════════════════════════════════════════════════════════════════════════════
# 단계별 프롬프트
# ══════════════════════════════════════════════════════════════════════════════

PERCEPTION_PROMPT = """
당신은 시니어 보조 협동로봇 JARVIS의 상황 파악 모듈입니다.

주어진 입력을 분석하여 현재 상황을 파악하세요.
아래 항목을 간결하게 분석하고 자연어로 응답하세요. JSON 아님.

분석 항목:
1. 음성 입력의 핵심 키워드와 의미
2. 제스처가 있다면 그 의미
3. 현재 감지된 객체와 관련성
4. 현재 로봇 동작과의 관계
5. 긴급 여부 판단
6. 전체 상황 한 줄 요약
"""

REASONING_PROMPT = """
당신은 시니어 보조 협동로봇 JARVIS의 의도 추론 모듈입니다.

상황 파악 결과를 바탕으로 사용자의 의도를 추론하세요.
아래 항목을 분석하고 자연어로 응답하세요. JSON 아님.

분석 항목:
1. 가장 가능성 높은 intent는 무엇인가?
   (bring_object / going_out / take_medicine / emergency /
    cancel / weather_query / general_query / unknown)
2. 목표 물체(target_object)는 무엇인가?
   (umbrella / bag / apple / banana / pill / phone /
    juice / sun_cream / water / candy / mask / bread / null)
3. 긴급도(urgency)는? (high / normal / low)
4. 멀티모달 입력 충돌이 있는가? 있다면 어떻게 해결하는가?
5. 신뢰도(confidence)는 얼마인가? (0.0~1.0)
"""

PLANNING_PROMPT = """
당신은 시니어 보조 협동로봇 JARVIS의 행동 계획 모듈입니다.

상황 파악 + 의도 추론 결과를 바탕으로 구체적인 행동 계획을 수립하세요.
아래 항목을 분석하고 자연어로 응답하세요. JSON 아님.

분석 항목:
1. 1순위로 수행할 행동과 대상 물체는?
2. 추가로 필요한 물체나 행동이 있는가? (순서 포함)
3. 날씨/상황 정보가 행동에 영향을 미치는가?
4. 실패 시 대체 행동은?
5. 사용자에게 전달할 TTS 메시지는?
"""


# ══════════════════════════════════════════════════════════════════════════════
# Intent Engine
# ══════════════════════════════════════════════════════════════════════════════

class IntentEngine:

    def __init__(self, api_key: str):
        self._client = OpenAI(api_key=api_key)

    # ── 단계별 추론 ───────────────────────────────────────────────────────────

    def _step1_perception(self, context: str) -> str:
        """STEP 1: 상황 파악"""
        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": PERCEPTION_PROMPT},
                {"role": "user",   "content": context},
            ],
            temperature=0.1,
        )
        result = resp.choices[0].message.content.strip()
        print(f"\n🔍 [STEP 1 - 상황 파악]\n{result}")
        return result

    def _step2_reasoning(self, context: str, perception: str) -> str:
        """STEP 2: 의도 추론"""
        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": REASONING_PROMPT},
                {"role": "user",   "content": (
                    f"[원본 입력]\n{context}\n\n"
                    f"[상황 파악 결과]\n{perception}"
                )},
            ],
            temperature=0.1,
        )
        result = resp.choices[0].message.content.strip()
        print(f"\n🧠 [STEP 2 - 의도 추론]\n{result}")
        return result

    def _step3_planning(self, context: str, perception: str,
                        reasoning: str) -> str:
        """STEP 3: 행동 계획"""
        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": PLANNING_PROMPT},
                {"role": "user",   "content": (
                    f"[원본 입력]\n{context}\n\n"
                    f"[상황 파악]\n{perception}\n\n"
                    f"[의도 추론]\n{reasoning}"
                )},
            ],
            temperature=0.1,
        )
        result = resp.choices[0].message.content.strip()
        print(f"\n📋 [STEP 3 - 행동 계획]\n{result}")
        return result

    def _step4_final(self, context: str, perception: str,
                     reasoning: str, planning: str) -> dict:
        """STEP 4: 최종 JSON 출력"""
        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user",   "content": (
                    f"[원본 입력]\n{context}\n\n"
                    f"[STEP1 상황 파악]\n{perception}\n\n"
                    f"[STEP2 의도 추론]\n{reasoning}\n\n"
                    f"[STEP3 행동 계획]\n{planning}\n\n"
                    f"위 3단계 분석을 바탕으로 최종 JSON을 출력하세요."
                )},
            ],
            temperature=Config.GPT_TEMPERATURE,
            response_format={"type": "json_object"},
        )
        return json.loads(resp.choices[0].message.content)

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def analyze(
        self,
        voice_text      : str,
        detected_objects: list[str]  = None,
        current_action  : str | None = None,
        gesture         : str | None = None,
    ) -> dict:
        """
        3단계 Chain of Thought 의도 추론

        Args:
            voice_text       : STT 결과 텍스트
            detected_objects : YOLO 감지 객체 리스트
            current_action   : 로봇 현재 동작
            gesture          : 제스처 결과

        Returns:
            dict: situation, intent, action_plan, scores, reason_log 등
        """
        # 컨텍스트 구성
        lines = [f'음성 입력: "{voice_text}"']
        if detected_objects:
            lines.append(f"현재 감지 객체: {detected_objects}")
        if current_action:
            lines.append(f"현재 로봇 동작: {current_action}")
        if gesture:
            lines.append(f"사용자 제스처: {gesture}")
        context = "\n".join(lines)

        print(f"\n{'─'*50}")
        print(f"🤖 [Intent Engine] 3단계 추론 시작")
        print(f"{'─'*50}")

        # 3단계 순차 추론
        perception = self._step1_perception(context)
        reasoning  = self._step2_reasoning(context, perception)
        planning   = self._step3_planning(context, perception, reasoning)
        result     = self._step4_final(context, perception, reasoning, planning)

        print(f"\n✅ [STEP 4 - 최종 결과] intent={result.get('intent')}")
        print(f"{'─'*50}")

        return result

    def analyze_fast(
        self,
        voice_text      : str,
        detected_objects: list[str]  = None,
        current_action  : str | None = None,
        gesture         : str | None = None,
    ) -> dict:
        """
        단일 호출 빠른 추론 (3단계 생략, 응답 속도 우선)
        긴급 상황이나 간단한 명령에 사용

        Returns:
            dict: intent 결과
        """
        lines = [f'음성 입력: "{voice_text}"']
        if detected_objects:
            lines.append(f"현재 감지 객체: {detected_objects}")
        if current_action:
            lines.append(f"현재 로봇 동작: {current_action}")
        if gesture:
            lines.append(f"사용자 제스처: {gesture}")

        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user",   "content": "\n".join(lines)},
            ],
            temperature=Config.GPT_TEMPERATURE,
            response_format={"type": "json_object"},
        )
        return json.loads(resp.choices[0].message.content)

    def answer_general(self, voice_text: str) -> str:
        """
        general_query 전용 자유 답변 생성
        시니어 눈높이에 맞는 친절한 한국어 답변
        """
        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": GENERAL_QUERY_PROMPT},
                {"role": "user",   "content": voice_text},
            ],
            temperature=0.7,
        )
        return resp.choices[0].message.content.strip()