#!/usr/bin/env python3
"""
weather.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
날씨 조회 모듈

현재: Open-Meteo (무료, 키 없음, 무제한)
  - 현재 날씨 (기온, 습도, 날씨코드)
  - 자외선 지수 (UV Index)
  - 미세먼지 PM10 / PM2.5

대기: OpenWeatherMap (주석 해제 후 사용)
"""

import requests
from jarvis_voice_pkg.config import Config


# ── 날씨 코드 → 한국어 변환표 (Open-Meteo WMO 코드) ──────────────────────────
WEATHER_CODE_MAP = {
    0 : "맑음",
    1 : "대체로 맑음",
    2 : "구름 조금",
    3 : "흐림",
    45: "안개",
    48: "안개",
    51: "이슬비",
    53: "이슬비",
    55: "이슬비",
    61: "비",
    63: "비",
    65: "강한 비",
    71: "눈",
    73: "눈",
    75: "강한 눈",
    80: "소나기",
    81: "소나기",
    82: "강한 소나기",
    95: "뇌우",
    96: "뇌우",
    99: "뇌우",
}


def _uv_level(uv: float) -> str:
    """자외선 지수 → 한국어 등급"""
    if uv <= 2:
        return "낮음"
    elif uv <= 5:
        return "보통"
    elif uv <= 7:
        return "높음"
    elif uv <= 10:
        return "매우 높음"
    else:
        return "위험"


def _pm_level(pm10: float, pm25: float) -> str:
    """미세먼지 PM10 기준 한국어 등급"""
    if pm10 <= 30:
        return "좋음"
    elif pm10 <= 80:
        return "보통"
    elif pm10 <= 150:
        return "나쁨"
    else:
        return "매우 나쁨"


def get_weather() -> str:
    """
    Open-Meteo API로 실시간 날씨 + 자외선 + 미세먼지 조회

    Returns:
        str: TTS로 읽어줄 날씨 안내 문자열
    """
    try:
        # ── OpenWeatherMap (일일 1,000회 한도 / 활성화 후 주석 해제) ──────────
        # url = (
        #     f"https://api.openweathermap.org/data/2.5/weather"
        #     f"?q={Config.WEATHER_CITY}"
        #     f"&appid={Config.WEATHER_API_KEY}"
        #     f"&units=metric"
        #     f"&lang=kr"
        # )
        # resp = requests.get(url, timeout=5)
        # resp.raise_for_status()
        # data  = resp.json()
        # desc  = data["weather"][0]["description"]
        # temp  = data["main"]["temp"]
        # feels = data["main"]["feels_like"]
        # humid = data["main"]["humidity"]
        # result = (
        #     f"현재 {Config.WEATHER_CITY} 날씨는 {desc}이고, "
        #     f"기온은 {temp:.1f}도, "
        #     f"체감온도는 {feels:.1f}도, "
        #     f"습도는 {humid}%입니다."
        # )

        # ── Open-Meteo 날씨 + 자외선 ─────────────────────────────────────────
        weather_url = (
            f"https://api.open-meteo.com/v1/forecast"
            f"?latitude={Config.WEATHER_LAT}"
            f"&longitude={Config.WEATHER_LON}"
            f"&current=temperature_2m,relative_humidity_2m,weathercode,uv_index"
            f"&timezone=Asia/Seoul"
        )
        weather_resp = requests.get(weather_url, timeout=5)
        weather_resp.raise_for_status()
        weather_data = weather_resp.json()

        temp  = weather_data["current"]["temperature_2m"]
        humid = weather_data["current"]["relative_humidity_2m"]
        code  = weather_data["current"]["weathercode"]
        uv    = weather_data["current"].get("uv_index", 0)
        desc  = WEATHER_CODE_MAP.get(code, "알 수 없음")
        uv_str = _uv_level(uv)

        # ── Open-Meteo 미세먼지 ───────────────────────────────────────────────
        air_url = (
            f"https://air-quality-api.open-meteo.com/v1/air-quality"
            f"?latitude={Config.WEATHER_LAT}"
            f"&longitude={Config.WEATHER_LON}"
            f"&current=pm10,pm2_5"
            f"&timezone=Asia/Seoul"
        )
        try:
            air_resp = requests.get(air_url, timeout=5)
            air_resp.raise_for_status()
            air_data  = air_resp.json()
            pm10      = air_data["current"]["pm10"]
            pm25      = air_data["current"]["pm2_5"]
            pm_str    = _pm_level(pm10, pm25)
            has_air   = True
        except Exception:
            pm10, pm25, pm_str, has_air = 0, 0, "확인 불가", False

        # ── 결과 문자열 조합 ─────────────────────────────────────────────────
        result = (
            f"현재 서울 날씨는 {desc}이고, "
            f"기온은 {temp}도, "
            f"습도는 {humid}%입니다. "
            f"자외선 지수는 {uv:.1f}로 {uv_str}이에요. "
        )

        if has_air:
            result += f"미세먼지는 {pm_str}이에요."

        # ── 외출 추천 멘트 ────────────────────────────────────────────────────
        tips = []
        if code in (61, 63, 65, 80, 81, 82):      # 비
            tips.append("우산을 챙기세요")
        if uv >= 6:                                  # 자외선 높음 이상
            tips.append("썬크림을 바르세요")
        if has_air and pm10 > 80:                    # 미세먼지 나쁨 이상
            tips.append("마스크를 착용하세요")
        if temp <= 10:                               # 기온 10도 이하
            tips.append("따뜻하게 입으세요")

        if tips:
            result += " " + ", ".join(tips) + "!"

        print(f"🌤️  [날씨] {result}")
        return result

    except Exception as e:
        print(f"❌ [날씨 API 오류] {e}")
        return "죄송해요, 날씨 정보를 가져오지 못했어요."


def get_weather_detail() -> dict:
    """
    날씨 상세 정보를 dict로 반환 (SYSTEM_PROMPT 컨텍스트 제공용)

    Returns:
        dict: {temp, humid, desc, uv, uv_level, pm10, pm25, pm_level,
               need_umbrella, need_sunscreen, need_mask, cold}
    """
    try:
        weather_url = (
            f"https://api.open-meteo.com/v1/forecast"
            f"?latitude={Config.WEATHER_LAT}"
            f"&longitude={Config.WEATHER_LON}"
            f"&current=temperature_2m,relative_humidity_2m,weathercode,uv_index"
            f"&timezone=Asia/Seoul"
        )
        weather_resp = requests.get(weather_url, timeout=5)
        weather_resp.raise_for_status()
        weather_data = weather_resp.json()

        temp  = weather_data["current"]["temperature_2m"]
        humid = weather_data["current"]["relative_humidity_2m"]
        code  = weather_data["current"]["weathercode"]
        uv    = weather_data["current"].get("uv_index", 0)
        desc  = WEATHER_CODE_MAP.get(code, "알 수 없음")

        try:
            air_url = (
                f"https://air-quality-api.open-meteo.com/v1/air-quality"
                f"?latitude={Config.WEATHER_LAT}"
                f"&longitude={Config.WEATHER_LON}"
                f"&current=pm10,pm2_5"
                f"&timezone=Asia/Seoul"
            )
            air_resp = requests.get(air_url, timeout=5)
            air_resp.raise_for_status()
            air_data = air_resp.json()
            pm10     = air_data["current"]["pm10"]
            pm25     = air_data["current"]["pm2_5"]
        except Exception:
            pm10, pm25 = 0, 0

        return {
            "temp"          : temp,
            "humid"         : humid,
            "desc"          : desc,
            "uv"            : uv,
            "uv_level"      : _uv_level(uv),
            "pm10"          : pm10,
            "pm25"          : pm25,
            "pm_level"      : _pm_level(pm10, pm25),
            "need_umbrella" : code in (61, 63, 65, 80, 81, 82),
            "need_sunscreen": uv >= 6,
            "need_mask"     : pm10 > 80,
            "cold"          : temp <= 10,
        }

    except Exception as e:
        print(f"❌ [날씨 상세 오류] {e}")
        return {}