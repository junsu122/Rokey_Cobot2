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

O4_MINI_MODEL = "o4-mini"


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
    cancel / weather_query / general_query / hungry / unknown)
   ★ 배고픔 표현("배고파", "배가 고파", "허기져", "뭐 먹을 거 없나") → hungry
      (bring_object가 아님. 시스템이 VLM으로 테이블을 확인 후 음식 여부로 분기함)
2. 목표 물체(target_objects)는 무엇인가? 복수일 수 있음.
   (umbrella / bag / apple / banana / pill / phone /
    juice / sun_cream / water / candy / mask / bread / null)
   건강 우선 원칙:
   - 단 것 요청("달콤한 거", "단 거") → candy 대신 banana 선택
   - hungry intent 시 target_objects는 [] (빈 리스트) — 시스템이 자동 결정
3. 긴급도(urgency)는? (high / normal / low)
4. 멀티모달 입력 충돌이 있는가? 있다면 어떻게 해결하는가?
5. 신뢰도(confidence)는 얼마인가? (0.0~1.0)
"""

PLANNING_PROMPT = """
당신은 시니어 보조 협동로봇 JARVIS의 행동 계획 모듈입니다.

상황 파악 + 의도 추론 결과를 바탕으로 구체적인 행동 계획을 수립하세요.
아래 항목을 분석하고 자연어로 응답하세요. JSON 아님.

분석 항목:
1. 집어야 할 물건 목록과 우선순위는? (물건이 여럿이면 순서도 명시)
2. 복수 물건일 때 효율적인 픽 순서는? (가벼운 것 먼저, 액체·넘어지기 쉬운 것 나중)
3. 날씨/상황 정보가 행동에 영향을 미치는가?
4. 실패 시 대체 행동은?
5. 사용자에게 전달할 TTS 메시지는?
   (복수면 "A랑 B 가져다드릴게요" 형태로 자연스럽게)
"""


# ══════════════════════════════════════════════════════════════════════════════
# Intent Engine
# ══════════════════════════════════════════════════════════════════════════════

class IntentEngine:

    def __init__(self, api_key: str):
        self._client = OpenAI(api_key=api_key)

    # ── 헬퍼 ─────────────────────────────────────────────────────────────────

    def _build_content(self, text: str, image_b64: str | None):
        """텍스트 + 이미지(있을 때)를 GPT content 형식으로 변환"""
        if image_b64:
            return [
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
                {"type": "text", "text": text},
            ]
        return text

    def _order_with_o4mini(self, objects: list[str]) -> list[str]:
        """o4-mini로 복수 물건의 최적 픽 순서 결정"""
        try:
            resp = self._client.chat.completions.create(
                model=O4_MINI_MODEL,
                messages=[{"role": "user", "content": (
                    f"로봇 팔로 집어야 할 물건 목록: {objects}\n"
                    "가장 효율적인 픽 순서를 결정해줘.\n"
                    "원칙: 가벼운 것 먼저, 액체/쏟아질 수 있는 것 나중.\n"
                    "JSON으로만 답변: {\"order\": [\"물건1\", \"물건2\", ...]}"
                )}],
                max_tokens=100,
            )
            content = resp.choices[0].message.content
            start = content.find("{")
            end   = content.rfind("}") + 1
            if start != -1 and end > start:
                data = json.loads(content[start:end])
                ordered = data.get("order", objects)
                print(f"\n⚙️  [o4-mini 순서] {objects} → {ordered}")
                return ordered
        except Exception as e:
            print(f"⚠️  [o4-mini] 순서 결정 실패 ({e}) — 기본 순서 사용")
        return objects

    # ── 단계별 추론 ───────────────────────────────────────────────────────────

    def _step1_perception(self, context: str,
                          image_b64: str | None = None) -> str:
        """STEP 1: 상황 파악 (이미지 있으면 VLM으로 분석)"""
        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": PERCEPTION_PROMPT},
                {"role": "user",
                 "content": self._build_content(context, image_b64)},
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
                     reasoning: str, planning: str,
                     image_b64: str | None = None) -> dict:
        """STEP 4: 최종 JSON 출력 (이미지 있으면 VLM으로 최종 확인)"""
        user_text = (
            f"[원본 입력]\n{context}\n\n"
            f"[STEP1 상황 파악]\n{perception}\n\n"
            f"[STEP2 의도 추론]\n{reasoning}\n\n"
            f"[STEP3 행동 계획]\n{planning}\n\n"
            "위 3단계 분석을 바탕으로 최종 JSON을 출력하세요."
        )
        resp = self._client.chat.completions.create(
            model=Config.GPT_MODEL,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user",
                 "content": self._build_content(user_text, image_b64)},
            ],
            temperature=Config.GPT_TEMPERATURE,
            response_format={"type": "json_object"},
        )
        result = json.loads(resp.choices[0].message.content)

        # target_objects 정규화: 없으면 target_object로 채움
        if "target_objects" not in result or not result["target_objects"]:
            t = result.get("target_object")
            result["target_objects"] = [t] if t else []

        return result

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def analyze(
        self,
        voice_text      : str,
        detected_objects: list[str]  = None,
        current_action  : str | None = None,
        gesture         : str | None = None,
        image_b64       : str | None = None,
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
        perception = self._step1_perception(context, image_b64)
        reasoning  = self._step2_reasoning(context, perception)
        planning   = self._step3_planning(context, perception, reasoning)
        result     = self._step4_final(context, perception, reasoning,
                                       planning, image_b64)

        # 복수 물건이면 o4-mini로 순서 최적화
        objects = result.get("target_objects", [])
        if len(objects) > 1:
            result["target_objects"] = self._order_with_o4mini(objects)
            result["target_object"]  = result["target_objects"][0]

        print(f"\n✅ [STEP 4 - 최종 결과] intent={result.get('intent')}  "
              f"objects={result.get('target_objects')}")
        print(f"{'─'*50}")

        return result

    def analyze_fast(
        self,
        voice_text      : str,
        detected_objects: list[str]  = None,
        current_action  : str | None = None,
        gesture         : str | None = None,
        image_b64       : str | None = None,
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
                {"role": "user",
                 "content": self._build_content("\n".join(lines), image_b64)},
            ],
            temperature=Config.GPT_TEMPERATURE,
            response_format={"type": "json_object"},
        )
        result = json.loads(resp.choices[0].message.content)

        if "target_objects" not in result or not result["target_objects"]:
            t = result.get("target_object")
            result["target_objects"] = [t] if t else []

        return result

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