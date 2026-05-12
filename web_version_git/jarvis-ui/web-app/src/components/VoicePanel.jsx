// src/components/VoicePanel.jsx
//
// 웨이크업 워드 "흰둥아" → Web Speech API 연속 인식
// → 감지 시 명령 인식 모드 진입 → DataChannel → /browser_stt

import { useState, useEffect, useRef, useCallback } from 'react';
import './VoicePanel.css';

const WAKE_WORD     = '흰둥아';
const COMMAND_SEC   = 4;   // 웨이크업 후 명령 대기 시간(초)
const RESTART_DELAY = 300; // 인식 종료 후 재시작 딜레이(ms)

export default function VoicePanel({ channel }) {
  const [wakeMode,   setWakeMode]   = useState('idle');  // idle | listening | command
  const [voiceText,  setVoiceText]  = useState('');
  const [intent,     setIntent]     = useState(null);
  const [ttsMessage, setTtsMessage] = useState('');
  const [supported,  setSupported]  = useState(true);
  const [countdown,  setCountdown]  = useState(0);

  const recognitionRef  = useRef(null);
  const commandTimerRef = useRef(null);
  const countdownRef    = useRef(null);
  const restartRef      = useRef(null);
  const activeRef       = useRef(false);  // 상시 청취 활성 여부
  const wakeModeRef     = useRef('idle'); // 최신 wakeMode를 ref로도 유지

  // ── Web Speech API 지원 확인 ──────────────────────────────────────────
  useEffect(() => {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SR) { setSupported(false); return; }
  }, []);

  // ── DataChannel 메시지 수신 ───────────────────────────────────────────
  useEffect(() => {
    if (!channel) return;
    const onMessage = (e) => {
      try {
        const data = JSON.parse(e.data);
        switch (data.type) {
          case 'voice_command':
            setVoiceText(data.text || '');
            // 다음 명령이 올 때까지 유지
            break;
          case 'intent_result':
            setIntent({
              intent    : data.intent,
              target    : data.target_object,
              confidence: data.confidence,
              response  : data.response_message,
            });
            setTimeout(() => setIntent(null), 8000);
            break;
          case 'tts_output': {
            const msg = data.message || '';
            if (msg && window.speechSynthesis) {
              const utt = new SpeechSynthesisUtterance(msg);
              utt.lang = 'ko-KR';
              utt.rate = 1.0;
              utt.pitch = 1.0;
              // 실제 발화 시작 시점에 화면 업데이트 → 큐 순서와 표시 순서 일치
              utt.onstart = () => setTtsMessage(msg);
              window.speechSynthesis.speak(utt);
            }
            break;
          }
        }
      } catch (_) {}
    };
    channel.addEventListener('message', onMessage);
    return () => channel.removeEventListener('message', onMessage);
  }, [channel]);

  // ── 카운트다운 시작 ───────────────────────────────────────────────────
  const startCountdown = useCallback(() => {
    setCountdown(COMMAND_SEC);
    countdownRef.current = setInterval(() => {
      setCountdown(prev => {
        if (prev <= 1) { clearInterval(countdownRef.current); return 0; }
        return prev - 1;
      });
    }, 1000);
  }, []);

  // ── 명령 모드 종료 → 대기 모드로 복귀 ───────────────────────────────
  const exitCommandMode = useCallback(() => {
    clearTimeout(commandTimerRef.current);
    clearInterval(countdownRef.current);
    setCountdown(0);
    wakeModeRef.current = 'listening';
    setWakeMode('listening');
  }, []);

  // ── 인식 시작 ─────────────────────────────────────────────────────────
  const startRecognition = useCallback(() => {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SR || !activeRef.current) return;

    const r = new SR();
    r.lang              = 'ko-KR';
    r.continuous        = false;
    r.interimResults    = false;
    r.maxAlternatives   = 3;
    recognitionRef.current = r;

    r.onresult = (e) => {
      // 모든 후보 텍스트 합쳐서 웨이크업 워드 검색
      const texts = Array.from(e.results)
        .flatMap(res => Array.from(res))
        .map(alt => alt.transcript.trim());

      const fullText = texts.join(' ');
      console.log(`[STT] "${fullText}"`);

      const isWake = texts.some(t =>
        t.includes(WAKE_WORD) ||
        t.includes('흰동아') ||  // 유사 발음 대응
        t.includes('흰 둥아')
      );

      if (wakeModeRef.current === 'command') {
        // 명령 모드: 웨이크 워드 제거 후 명령어만 추출
        const cmd = fullText
          .replace(WAKE_WORD, '').replace('흰동아','').replace('흰 둥아','').trim();
        if (cmd && channel?.readyState === 'open') {
          console.log(`[COMMAND] "${cmd}"`);
          channel.send(JSON.stringify({ type: 'browser_stt', text: cmd }));
          setVoiceText(cmd);
          // 다음 명령이 올 때까지 유지
        }
        exitCommandMode();
      } else if (isWake) {
        // 대기 모드에서 웨이크업 감지
        console.log(`[WAKE] "${WAKE_WORD}" 감지!`);
        clearTimeout(commandTimerRef.current);
        clearInterval(countdownRef.current);
        wakeModeRef.current = 'command';
        setWakeMode('command');
        startCountdown();
        // COMMAND_SEC초 후 자동 복귀
        commandTimerRef.current = setTimeout(exitCommandMode, COMMAND_SEC * 1000);
      }
    };

    r.onend = () => {
      if (!activeRef.current) return;
      // 자동 재시작
      restartRef.current = setTimeout(startRecognition, RESTART_DELAY);
    };

    r.onerror = (e) => {
      if (e.error === 'aborted' || e.error === 'no-speech') return;
      console.warn('[STT 오류]', e.error);
      if (!activeRef.current) return;
      restartRef.current = setTimeout(startRecognition, 1000);
    };

    try { r.start(); } catch (_) {}
  }, [channel, exitCommandMode, startCountdown]);

  // ── 마운트 시 자동 청취 시작 (channel 연결되는 순간 자동 ON) ─────────
  useEffect(() => {
    if (!supported || !channel) return;
    // 이미 활성화 중이면 중복 시작 방지
    if (activeRef.current) return;
    activeRef.current = true;
    wakeModeRef.current = 'listening';
    setWakeMode('listening');
    startRecognition();
  }, [channel, supported, startRecognition]);

  // ── 상시 청취 수동 ON/OFF (버튼용) ───────────────────────────────────
  const toggleListening = useCallback(() => {
    if (!supported) return;

    if (activeRef.current) {
      // 끄기
      activeRef.current = false;
      clearTimeout(restartRef.current);
      clearTimeout(commandTimerRef.current);
      clearInterval(countdownRef.current);
      recognitionRef.current?.abort();
      recognitionRef.current = null;
      wakeModeRef.current = 'idle';
      setWakeMode('idle');
      setCountdown(0);
    } else {
      // 켜기
      activeRef.current = true;
      wakeModeRef.current = 'listening';
      setWakeMode('listening');
      startRecognition();
    }
  }, [supported, startRecognition]);

  // ── 언마운트 정리 ─────────────────────────────────────────────────────
  useEffect(() => {
    return () => {
      activeRef.current = false;
      clearTimeout(restartRef.current);
      clearTimeout(commandTimerRef.current);
      clearInterval(countdownRef.current);
      recognitionRef.current?.abort();
    };
  }, []);

  // ── intent 색상 ───────────────────────────────────────────────────────
  const intentColor = {
    bring_object  : '#5A7A3A',
    cancel        : '#C0503A',
    emergency     : '#C0503A',
    weather_query : '#6B4226',
    general_query : '#C89040',
    take_medicine : '#7A6A9A',
    going_out     : '#C89040',
    unknown       : '#A89080',
  }[intent?.intent] || '#A89080';

  const isIdle      = wakeMode === 'idle';
  const isListening = wakeMode === 'listening';
  const isCommand   = wakeMode === 'command';

  return (
    <div className="vp-root">
      {/* 헤더 + 버튼 */}
      <div className="vp-header">
        <span className="vp-title">🎙 VOICE COMMAND</span>
        <div className="vp-mic-area">
          {isListening && (
            <span className="vp-listening-label">"{WAKE_WORD}" 대기 중...</span>
          )}
          {isCommand && (
            <span className="vp-command-label">🐕 말씀하세요! ({countdown}초)</span>
          )}
          <button
            className={`vp-mic-btn ${isListening ? 'listening' : ''} ${isCommand ? 'active' : ''} ${!supported ? 'disabled' : ''}`}
            onClick={toggleListening}
            disabled={!supported || !channel}
            title={!channel ? '연결 대기 중' : isIdle ? '켜기' : '끄기'}
          >
            {isIdle ? '🎤' : isCommand ? '🐕' : '👂'}
          </button>
        </div>
      </div>

      {/* 웨이크업 상태 표시 */}
      {isListening && (
        <div className="vp-wake-status">
          <span className="vp-wake-dot" />
          <span>"{WAKE_WORD}"라고 말씀하시면 명령을 듣습니다</span>
        </div>
      )}

      {isCommand && (
        <div className="vp-command-status">
          <div className="vp-command-bar">
            <div
              className="vp-command-progress"
              style={{ width: `${(countdown / COMMAND_SEC) * 100}%` }}
            />
          </div>
          <span>명령을 말씀해 주세요</span>
        </div>
      )}

      {/* STT 결과 */}
      {voiceText && (
        <div className="vp-stt">
          <span className="vp-stt-icon">🗣</span>
          <span className="vp-stt-text">"{voiceText}"</span>
        </div>
      )}

      {/* TTS 응답 */}
      {ttsMessage && (
        <div className="vp-tts">
          <span className="vp-tts-icon">🔊</span>
          <span className="vp-tts-text">{ttsMessage}</span>
        </div>
      )}



      {/* 대기 상태 */}
      {isIdle && !voiceText && !ttsMessage && !intent && (
        <div className="vp-idle">
          {!channel ? '연결 후 자동으로 활성화됩니다' :
           !supported ? 'Web Speech API 미지원 브라우저' :
           '🎤 버튼을 눌러 음성 명령을 활성화하세요'}
        </div>
      )}
    </div>
  );
}