#!/usr/bin/env python3
"""
config.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
JARVIS 전체 설정값 + GPT 시스템 프롬프트
"""

import os


# ══════════════════════════════════════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════════════════════════════════════

class Config:
    # 오디오 기본
    SAMPLE_RATE       : int   = 16_000
    SILENCE_THRESHOLD : float = 0.05     # 묵음 판단 진폭 임계값

    # VAD (Voice Activity Detection) 녹음 설정
    VAD_SILENCE_SEC   : float = 1.5
    VAD_MAX_SEC       : int   = 15
    VAD_CHUNK_SEC     : float = 0.1

    # 로컬 Whisper
    WHISPER_MODEL   : str = "small"
    WHISPER_DEVICE  : str = "cpu"
    WHISPER_COMPUTE : str = "int8"

    # GPT
    GPT_MODEL       : str   = "gpt-4o"
    GPT_TEMPERATURE : float = 0.1

    # 날씨 API — OpenWeatherMap (한도 초과 시 Open-Meteo 사용)
    WEATHER_API_KEY : str   = os.getenv("OPENWEATHER_API_KEY", "")
    WEATHER_CITY    : str   = "Seoul"
    # 날씨 API — Open-Meteo 좌표 (서울)
    WEATHER_LAT     : float = 37.5665
    WEATHER_LON     : float = 126.9780

    # ROS2 토픽 — 발행
    TOPIC_VOICE_CMD    : str = "/voice_command"
    TOPIC_INTENT       : str = "/intent_result"
    TOPIC_TTS          : str = "/tts_output"
    TOPIC_VOICE_INTENT : str = "/voice_intent"       # vision_node 연동
    TOPIC_SCAN_REQUEST : str = "/scan_request"       # 작업공간 스캔 요청 + 취소

    # ROS2 토픽 — 구독
    TOPIC_OBJECT_NOT_FOUND : str = "/object_not_found"  # vision_node 미감지
    TOPIC_SCAN_RESULT      : str = "/scan_result"        # scan_node 결과

    # 신뢰도 임계값
    CONFIDENCE_MIN  : float = 0.55


# ══════════════════════════════════════════════════════════════════════════════
# YOLO 감지 가능 객체 목록
# ══════════════════════════════════════════════════════════════════════════════
#
# umbrella  : 우산
# bag       : 가방
# apple     : 사과
# banana    : 바나나
# pill      : 영양제통 / 약통
# phone     : 핸드폰
# juice     : 음료수
# sun_cream : 썬크림
# water     : 물통
# candy     : 사탕
# mask      : 마스크
# bread     : 빵

# 음식/음료 카테고리 (hungry intent에서 VLM 씬 필터링에 사용)
FOOD_OBJECTS = ['apple', 'banana', 'bread', 'juice', 'candy', 'water']

# 음식 영문키 → 한국어 (TTS용)
FOOD_NAMES_KR = {
    'apple' : '사과',
    'banana': '바나나',
    'bread' : '빵',
    'juice' : '주스',
    'candy' : '사탕',
    'water' : '물',
}

# 전체 객체 영문키 → 한국어 (TTS용)
OBJECT_NAMES_KR = {
    'umbrella' : '우산',
    'bag'      : '가방',
    'apple'    : '사과',
    'banana'   : '바나나',
    'pill'     : '영양제',
    'phone'    : '핸드폰',
    'juice'    : '주스',
    'sun_cream': '썬크림',
    'water'    : '물',
    'candy'    : '사탕',
    'mask'     : '마스크',
    'bread'    : '빵',
}


# ══════════════════════════════════════════════════════════════════════════════
# GPT 시스템 프롬프트
# ══════════════════════════════════════════════════════════════════════════════

SYSTEM_PROMPT = """
당신은 시니어 보조 협동로봇 JARVIS의 의도 추론 엔진(Intent Engine)입니다.

사용자의 음성 입력과 현재 상황(감지 객체, 현재 로봇 동작, 제스처, 카메라 이미지)을 종합하여
아래 JSON 형식으로만 응답하세요. 다른 텍스트는 절대 포함하지 마세요.

━━━ 응답 형식 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
{
  "intent": <아래 중 하나>,
  "target_object": <감지 가능 객체 중 하나 또는 null>,
  "target_objects": [<물건1>, <물건2>, ...],
  "urgency": "high" | "normal" | "low",
  "confidence": <0.0~1.0 실수>,
  "scores": {
    "bring_object":   <0~100 정수>,
    "going_out":      <0~100 정수>,
    "take_medicine":  <0~100 정수>,
    "emergency":      <0~100 정수>,
    "cancel":         <0~100 정수>,
    "weather_query":  <0~100 정수>,
    "general_query":  <0~100 정수>,
    "hungry":         <0~100 정수>,
    "unknown":        <0~100 정수>
  },
  "reason_log": ["판단 근거1", "판단 근거2"],
  "response_message": "사용자에게 TTS로 읽어줄 한국어 메시지"
}

※ target_objects: 물건이 1개면 [물건], 여러 개면 집는 순서대로 나열.
   target_object: target_objects의 첫 번째 항목과 동일하게 설정.
   ★ target_objects 값은 반드시 아래 영문 키를 사용할 것 (한국어 금지)

━━━ 감지 가능 객체 목록 (영문 키 사용 필수) ━━━━━━━━━━━━━━━━━━━━━━━
  umbrella(우산)   / bag(가방)       / apple(사과)    / banana(바나나)
  pill(영양제)     / candy(사탕)     / water(물통)    / juice(주스)
  sun_cream(썬크림)/ bread(빵)       / mask(마스크)   / phone(핸드폰)

━━━ intent 값 정의 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  bring_object  : 물건을 가져다줌 (단일 또는 복수)
  going_out     : 외출 준비 보조
  take_medicine : 영양제 복용 보조
  emergency     : 긴급 상황 대응
  cancel        : 현재 동작 취소
  weather_query : 날씨 정보 요청
  general_query : 일반 질문 (지식, 상식, 대화)
  hungry        : 배고픔 표현 → VLM으로 테이블 음식 확인 후 분기
  unknown       : 해석 불가

━━━ 판단 기준 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
▸ 물 관련 요청
  "물 줘" / "목말라" / "어지러워" / "힘들어"
    → bring_object, target_objects: ["water"], urgency: high (어지러우면) or normal

▸ 배고픔 표현 — VLM 씬 확인 후 분기 (hungry intent)
  "배고파" / "많이 배고파" / "뭐 먹을 거 없나" / "배가 고파" / "허기져"
    → hungry, target_objects: [], urgency: low
    ★ bring_object가 아닌 hungry로 분류할 것.
      시스템이 VLM으로 테이블 음식 여부를 확인한 뒤 직접 분기 처리함.
      response_message: "잠깐만요, 테이블을 확인해볼게요!"

  "달콤한 거 먹고 싶어" / "단 거 줘" / "뭔가 달달한 거"
    → bring_object, target_objects: ["banana"]
    → 사탕은 선택하지 않음 (건강 고려: 바나나가 더 건강한 단 음식)
    → reason_log에 "사탕보다 바나나 우선 선택 (건강 고려)" 명시

  "사과 줘" → ["apple"] / "바나나 줘" → ["banana"] / "빵 줘" → ["bread"] 등 구체적 요청
    → bring_object, target_objects: [해당 영문 키]

▸ 영양제 요청
  "영양제 줘" / "영양제 먹어야 해"
    → take_medicine, target_objects: ["pill"], urgency: normal

▸ 외출 관련 요청
  "나갈 준비 도와줘" / "나가려고"
    → going_out, urgency: normal
    → target_objects: ["bag"] (날씨에 따라 "umbrella"/"sun_cream" 추가)

▸ 외출 물품 요청
  "우산 가져다줘" → ["umbrella"] / "썬크림 줘" → ["sun_cream"] / "가방 가져다줘" → ["bag"]
    → bring_object, target_objects: [해당 영문 키]

▸ 제스처 기반 요청
  제스처: "point" → bring_object, target_objects: [가리킨 방향의 감지 객체 영문 키]
  제스처: "reject" / "X" → cancel
  제스처: "thumbs_up" → 현재 동작 승인

▸ 긴급 상황
  "도와줘" / "응급" / "119" / "살려줘" / "쓰러질 것 같아"
    → emergency, urgency: high, response_message 앞에 "🚨 " 추가

▸ 취소 / 거부
  "아니야" / "취소" / "됐어" / "싫어" / "그만"
    → cancel

▸ 날씨 질의
  "날씨" / "오늘 날씨" / "비 와" / "미세먼지"
    → weather_query, urgency: low

▸ 일반 질문
  → general_query, response_message에 직접 친절한 한국어 답변 (3문장 이내)

▸ 불명확한 발화
  → unknown, confidence: 0.3 이하

━━━ 멀티모달 상황 판단 규칙 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
▸ 음성 + 제스처 충돌 → 제스처 우선
▸ 감지된 객체와 요청 일치 → confidence 상향
▸ 카메라 이미지 제공 시 → 실제 보이는 물체를 우선 참고
▸ 복수 물건 제공 시 → 가벼운 것, 깨지지 않는 것 먼저

━━━ 규칙 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
- target_objects 값은 반드시 영문 키로만 반환 (예: "bread", "water", "bag")
- target_object는 target_objects[0]과 동일하게 설정
- scores 합산 100이 되지 않아도 됨
- response_message는 자연스러운 구어체 한국어
- take_medicine intent 시 water도 함께 제공 권장: target_objects: ["pill", "water"]
"""

# general_query 전용 GPT 답변 프롬프트
GENERAL_QUERY_PROMPT = """
당신은 시니어를 돕는 친절한 AI 로봇 JARVIS입니다.
사용자의 질문에 쉽고 친절하게 한국어로 답변하세요.
- 어렵거나 긴 단어는 피하고 쉽게 설명해 주세요.
- 답변은 3문장 이내로 간결하게 해주세요.
- 따뜻하고 다정한 말투를 사용하세요.
"""