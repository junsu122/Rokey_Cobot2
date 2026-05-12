#!/usr/bin/env python3
"""
stt.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
faster-whisper 기반 로컬 STT 모듈 (오프라인 동작)

첫 실행 시 모델 자동 다운로드, 이후 캐시 사용
모델 크기: tiny / base / small / medium / large-v3
"""

import os
import tempfile

import numpy as np
from scipy.io import wavfile
from faster_whisper import WhisperModel

from jarvis_voice_pkg.config import Config


class LocalWhisperSTT:

    def __init__(self):
        print(f"⏳ [Whisper] 모델 로딩 중... ({Config.WHISPER_MODEL})")
        self._model = WhisperModel(
            Config.WHISPER_MODEL,
            device=Config.WHISPER_DEVICE,
            compute_type=Config.WHISPER_COMPUTE,
        )
        print(f"✅ [Whisper] 준비 완료 "
              f"— {Config.WHISPER_MODEL} / {Config.WHISPER_DEVICE}")

    def transcribe(self, audio: np.ndarray) -> tuple[str, float]:
        """
        numpy float32 오디오 → (텍스트, 평균 logprob)

        logprob 해석:
            0에 가까울수록 높은 신뢰도
            -1 이하이면 낮은 신뢰도
        """
        # 임시 WAV 저장 후 Whisper에 전달
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wavfile.write(
                tmp.name,
                Config.SAMPLE_RATE,
                (audio * 32767).astype(np.int16)
            )
            tmp_path = tmp.name

        segments, _ = self._model.transcribe(
            tmp_path,
            language="ko",
            beam_size=5,
            vad_filter=True,
            vad_parameters={"min_silence_duration_ms": 500},
        )

        os.unlink(tmp_path)  # 임시 파일 삭제

        texts, logprobs = [], []
        for seg in segments:
            texts.append(seg.text.strip())
            logprobs.append(getattr(seg, "avg_logprob", -0.5))

        text     = " ".join(texts).strip()
        avg_logp = float(np.mean(logprobs)) if logprobs else -1.0
        return text, avg_logp
