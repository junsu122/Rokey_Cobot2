#!/usr/bin/env python3
"""
recorder.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VAD(Voice Activity Detection) 기반 오디오 녹음 모듈

- 소리 감지되면 자동 녹음 시작
- 묵음이 VAD_SILENCE_SEC 지속되면 자동 종료
- VAD_MAX_SEC 초과 시 강제 종료
"""

import numpy as np
import sounddevice as sd
from jarvis_voice_pkg.config import Config


class AudioRecorder:

    @staticmethod
    def record() -> np.ndarray:
        """
        VAD 방식 녹음
        Returns:
            numpy float32 오디오 배열 (묵음이면 빈 배열)
        """
        chunk_size     = int(Config.SAMPLE_RATE * Config.VAD_CHUNK_SEC)
        max_chunks     = int(Config.VAD_MAX_SEC  / Config.VAD_CHUNK_SEC)
        silence_chunks = int(Config.VAD_SILENCE_SEC / Config.VAD_CHUNK_SEC)

        print(f"\n🎤  말씀해 주세요... "
              f"(말하면 자동 시작 / 멈추면 자동 종료 / 최대 {Config.VAD_MAX_SEC}초)")

        recorded     = []
        silent_count = 0
        started      = False

        with sd.InputStream(
            samplerate=Config.SAMPLE_RATE,
            channels=1,
            dtype="float32"
        ) as stream:
            for _ in range(max_chunks):
                chunk, _ = stream.read(chunk_size)
                chunk     = chunk.flatten()
                amplitude = float(np.max(np.abs(chunk)))

                if not started:
                    # 음성 시작 대기
                    if amplitude > Config.SILENCE_THRESHOLD:
                        started = True
                        print("🔴  녹음 중...")
                        recorded.append(chunk)
                else:
                    # 녹음 중
                    recorded.append(chunk)
                    if amplitude < Config.SILENCE_THRESHOLD:
                        silent_count += 1
                        if silent_count >= silence_chunks:
                            break         # 묵음 지속 → 종료
                    else:
                        silent_count = 0  # 다시 말하면 리셋

        if not recorded:
            print("🔇  음성 감지 안 됨")
            return np.array([])

        print("✅  녹음 완료")
        return np.concatenate(recorded)

    @staticmethod
    def is_silence(audio: np.ndarray) -> bool:
        """빈 배열이거나 진폭이 임계값 이하이면 묵음"""
        return (
            len(audio) == 0 or
            float(np.max(np.abs(audio))) < Config.SILENCE_THRESHOLD
        )
