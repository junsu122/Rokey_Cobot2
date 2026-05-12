// src/Dashboard.jsx

import { useState, useEffect, useCallback, useRef } from 'react';
import GestureCanvas from './components/GestureCanvas';
import VisionCanvas  from './components/VisionCanvas';
import VoicePanel    from './components/VoicePanel';
import './Dashboard.css';

// ── 연결 중 오버레이 ────────────────────────────────────────────────────────
function ConnectingOverlay({ gestureConn, visionConn }) {
  const items = [
    { label: '제스처 모니터', done: gestureConn },
    { label: '비전 모니터',   done: visionConn  },
  ];
  return (
    <div style={{
      position: 'fixed', inset: 0, zIndex: 999,
      background: 'rgba(244,242,236,0.88)',
      backdropFilter: 'blur(8px)',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }}>
      <div style={{
        background: '#FFFFFF',
        border: '1.5px solid rgba(58,102,50,0.15)',
        borderRadius: 24, padding: '48px 64px',
        display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 32,
        boxShadow: '0 8px 48px rgba(58,102,50,0.10)',
        minWidth: 360,
      }}>

        {/* 아이콘 + 스피너 */}
        <div style={{ position: 'relative', width: 72, height: 72, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
          <span style={{ fontSize: 36, lineHeight: 1 }}>🤖</span>
          <div style={{
            position: 'absolute', inset: -4,
            border: '2.5px solid rgba(58,102,50,0.15)',
            borderTopColor: '#3A6632',
            borderRadius: '50%',
            animation: 'jarvis-spin 1.2s linear infinite',
          }}/>
        </div>

        {/* 타이틀 */}
        <div style={{ textAlign: 'center' }}>
          <div style={{ color: '#3A6632', fontSize: 11, letterSpacing: '0.2em', marginBottom: 8, fontWeight: 700, textTransform: 'uppercase' }}>
            짱구네 시스템
          </div>
          <div style={{ color: '#1A1A16', fontSize: 20, fontWeight: 700, marginBottom: 4 }}>
            로봇과 연결 중입니다
          </div>
          <div style={{ color: '#9A9A90', fontSize: 13 }}>
            잠시만 기다려 주세요...
          </div>
        </div>

        {/* 항목별 진행 상태 */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: 10, width: '100%' }}>
          {items.map(({ label, done }) => (
            <div key={label} style={{
              display: 'flex', alignItems: 'center', gap: 12,
              padding: '12px 18px',
              background: done ? 'rgba(58,102,50,0.07)' : '#F4F2EC',
              border: `1.5px solid ${done ? 'rgba(58,102,50,0.25)' : 'rgba(0,0,0,0.07)'}`,
              borderRadius: 12,
              transition: 'all 0.4s ease',
            }}>
              {done
                ? <span style={{ fontSize: 15, color: '#3A6632' }}>✓</span>
                : <div style={{
                    width: 14, height: 14, flexShrink: 0,
                    border: '2px solid rgba(58,102,50,0.2)',
                    borderTopColor: '#3A6632',
                    borderRadius: '50%',
                    animation: 'jarvis-spin 0.9s linear infinite',
                  }}/>
              }
              <span style={{
                fontSize: 13, fontWeight: 600,
                color: done ? '#3A6632' : '#52524A',
              }}>
                {label}
              </span>
              <span style={{
                marginLeft: 'auto', fontSize: 11, fontWeight: 600,
                color: done ? '#3A6632' : '#9A9A90',
                letterSpacing: '0.05em',
              }}>
                {done ? 'CONNECTED' : 'CONNECTING...'}
              </span>
            </div>
          ))}
        </div>

        <style>{`@keyframes jarvis-spin { to { transform: rotate(360deg); } }`}</style>
      </div>
    </div>
  );
}

// ── Dashboard ────────────────────────────────────────────────────────────────
// ── 리커버리 모달 ─────────────────────────────────────────────────────────────────
function RecoveryModal({ stateStr, recovering, onRecover }) {
  return (
    <div style={{
      position: 'fixed', inset: 0, zIndex: 1000,
      background: 'rgba(244,242,236,0.90)',
      backdropFilter: 'blur(6px)',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }}>
      <div style={{
        background: '#FFFFFF',
        border: '1.5px solid rgba(184,74,54,0.25)',
        borderRadius: 24, padding: '48px 64px',
        display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 28,
        boxShadow: '0 8px 48px rgba(184,74,54,0.10)',
        minWidth: 360, textAlign: 'center',
      }}>
        <div style={{ fontSize: 48 }}>⚠️</div>
        <div>
          <div style={{ color: '#B84A36', fontSize: 11, fontWeight: 700,
                        letterSpacing: '0.2em', marginBottom: 8, textTransform: 'uppercase' }}>
            ROBOT ALERT
          </div>
          <div style={{ color: '#1A1A16', fontSize: 20, fontWeight: 700, marginBottom: 6 }}>
            로봇이 정지 상태입니다
          </div>
          <div style={{ color: '#9A9A90', fontSize: 13 }}>{stateStr}</div>
        </div>
        {recovering ? (
          <div style={{ display: 'flex', alignItems: 'center', gap: 10,
                        color: '#52524A', fontSize: 13 }}>
            <div style={{
              width: 16, height: 16,
              border: '2px solid rgba(58,102,50,0.2)',
              borderTopColor: '#3A6632', borderRadius: '50%',
              animation: 'jarvis-spin 0.9s linear infinite',
            }}/>
            복구 중...
            <style>{`@keyframes jarvis-spin { to { transform: rotate(360deg); } }`}</style>
          </div>
        ) : (
          <button onClick={onRecover} style={{
            fontFamily: 'Inter, sans-serif',
            fontSize: 14, fontWeight: 700,
            padding: '12px 32px', borderRadius: 24,
            border: 'none', background: '#3A6632', color: '#fff',
            cursor: 'pointer', letterSpacing: '0.3px',
          }}>
            🔄 로봇 복구
          </button>
        )}
      </div>
    </div>
  );
}

export default function Dashboard() {
  const [now,         setNow]         = useState(new Date());
  const [selectMode,  setSelectMode]  = useState(false);
  const [channel,     setChannel]     = useState(null);
  const [gestureConn, setGestureConn] = useState(false);
  const [visionConn,  setVisionConn]  = useState(false);
  const [connecting,   setConnecting]  = useState(false);
  const [robotState,   setRobotState]  = useState(null); // {state_code, state_str, recovering}
  const [recoverySent, setRecoverySent] = useState(false); // 복구 버튼 눌렀는지

  const gestureRef     = useRef(null);
  const visionRef      = useRef(null);
  const visionTimerRef = useRef(null);

  const connected = gestureConn && visionConn;

  // 두 채널 모두 연결되면 오버레이 자동으로 닫힘
  useEffect(() => {
    if (connected) setConnecting(false);
  }, [connected]);

  useEffect(() => {
    const t = setInterval(() => setNow(new Date()), 1000);
    return () => clearInterval(t);
  }, []);

  const handleModeChange = useCallback((isPointing) => {
    setSelectMode(isPointing);
  }, []);

  const handleChannel = useCallback((ch) => {
    setChannel(ch);
    setGestureConn(!!ch);
    if (ch) {
      ch.addEventListener('message', (e) => {
        try {
          const data = JSON.parse(e.data);
          if (data.type === 'robot_state_summary') {
            setRobotState(data);
            // 복구 완료되면 (state_code=1 STANDBY) recoverySent 리셋
            if (data.state_code === 1) setRecoverySent(false);
          }
        } catch (_) {}
      });
    }
  }, []);

  const handleVisionChannel = useCallback((isConnected) => {
    setVisionConn(isConnected);
  }, []);

  const handleHandData = useCallback((data) => {
    visionRef.current?.updateHandData(data);
  }, []);

  // ── 연결 / 해제 ──────────────────────────────────────────────────────────
  const handleConnect = useCallback(() => {
    setConnecting(true);
    setGestureConn(false);
    setVisionConn(false);
    gestureRef.current?.connect();
    // STUN/ICE 동시 요청 충돌 방지: 1초 간격
    visionTimerRef.current = setTimeout(() => {
      visionRef.current?.connect();
    }, 1000);
  }, []);

  const handleRecovery = useCallback(() => {
    if (channel?.readyState === 'open') {
      channel.send(JSON.stringify({ type: 'recovery_command' }));
      setRecoverySent(true);
    }
  }, [channel]);

  const handleDisconnect = useCallback(() => {
    clearTimeout(visionTimerRef.current);
    gestureRef.current?.disconnect();
    visionRef.current?.disconnect();
    setGestureConn(false);
    setVisionConn(false);
    setConnecting(false);
    setChannel(null);
  }, []);

  const btnLabel = connected ? '연결 해제' : '로봇 시스템 연결';
  const btnClass = `db-connect-btn ${connected ? 'connected' : ''} ${connecting ? 'connecting' : ''}`;

  return (
    <div className="db-root">
      <div className="db-grid-bg" />

      {/* ── 연결 중 오버레이 ── */}
      {connecting && (
        <ConnectingOverlay gestureConn={gestureConn} visionConn={visionConn} />
      )}

      {/* ── 리커버리 모달 ── */}
      {connected && robotState && [3,5,6].includes(robotState.state_code) && (
        <RecoveryModal
          stateStr={robotState.state_str}
          recovering={recoverySent}
          onRecover={handleRecovery}
        />
      )}

      {/* ── 헤더 ── */}
      <header className="db-header">
        <div className="db-logo">
          <span className="db-logo-icon">⬡</span>
          <div>
            <div className="db-logo-title">짱구네</div>
            <div className="db-logo-sub">GESTURE CONTROL · v2.0</div>
          </div>
        </div>

        <div className="db-header-center">
          <div className="db-clock">
            {now.toLocaleTimeString('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false })}
          </div>
          <div className="db-date">
            {now.toLocaleDateString('ko-KR', { month: 'long', day: 'numeric', weekday: 'short' })}
          </div>
        </div>

        <div className="db-header-right">
          <button
            className={btnClass}
            onClick={connected ? handleDisconnect : handleConnect}
            disabled={connecting}
          >
            {btnLabel}
          </button>
          <div className={`db-mode-badge ${selectMode ? 'select' : 'gesture'}`}>
            {selectMode ? '☝ 물체 선택 모드' : '✋ 제스처 제어 모드'}
          </div>
          <div className="db-system-status">
            <span className="db-pulse" />SYSTEM ONLINE
          </div>
        </div>
      </header>

      {/* ── 메인 ── */}
      <main className={`db-main ${selectMode ? 'select-mode' : ''}`}>
        <section className="db-card db-gesture-card">
          <GestureCanvas
            ref={gestureRef}
            onModeChange={handleModeChange}
            onChannel={handleChannel}
            onHandData={handleHandData}
          />
        </section>
        <section className="db-card db-vision-card">
          <VisionCanvas
            ref={visionRef}
            onConnect={handleVisionChannel}
          />
        </section>
      </main>

      {/* ── 음성 패널 ── */}
      <div className="db-voice">
        <VoicePanel channel={channel} />
      </div>
    </div>
  );
}
