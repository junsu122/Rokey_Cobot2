// src/components/VisionCanvas.jsx

import { useEffect, useRef, useState, useCallback, forwardRef, useImperativeHandle } from 'react';
import './VisionCanvas.css';

const SIGNALING_URL = import.meta.env.VITE_SIGNALING_URL || 'http://localhost:5000';
const ROOM          = 'jarvis-vision';
const HOVER_SEC     = 2.0;
const RECONNECT_SEC = 3;
const ICE_SERVERS   = [
  { urls: 'stun:stun.l.google.com:19302' },
  { urls: 'turn:openrelay.metered.ca:80',  username: 'openrelayproject', credential: 'openrelayproject' },
  { urls: 'turn:openrelay.metered.ca:443', username: 'openrelayproject', credential: 'openrelayproject' },
];

const HAND_CONNECTIONS = [
  [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],
  [5,9],[9,10],[10,11],[11,12],[9,13],[13,14],[14,15],[15,16],
  [13,17],[17,18],[18,19],[19,20],[0,17],
];

const VisionCanvas = forwardRef(function VisionCanvas({ onConnect }, ref) {
  // ── refs ──────────────────────────────────────────────────────────────────
  const videoRef       = useRef(null);
  const displayRef     = useRef(null);
  const pcRef          = useRef(null);
  const channelRef     = useRef(null);
  const pollRef        = useRef(null);
  const rafRef         = useRef(null);
  const answeredRef    = useRef(false);
  const reconnectTimer = useRef(null);
  const reconnectCount = useRef(0);
  const doConnectRef   = useRef(null);

  // 손 데이터 (GestureCanvas → updateHandData 로 업데이트)
  const handDataRef    = useRef(null);
  // 호버 상태
  const hoverRef       = useRef({ target: null, startTime: null });
  const detectionsRef  = useRef([]);
  const lastDetRef     = useRef(null);

  // ── state ─────────────────────────────────────────────────────────────────
  const [status,       setStatus]       = useState('disconnected');
  const [detections,   setDetections]   = useState([]);
  const [hoverInfo,    setHoverInfo]    = useState(null);
  const [selected,     setSelected]     = useState(null);
  const [cameraOnline, setCameraOnline] = useState(false);
  const [reconnectIn,  setReconnectIn]  = useState(0);

  // ── sendChannel: ref 직접 읽어서 stale closure 없음 ──────────────────────
  const sendChannel = useCallback((data) => {
    if (channelRef.current?.readyState === 'open')
      channelRef.current.send(JSON.stringify(data));
  }, []);

  // ── RAF 비디오 루프: [] deps → 안정적, 모든 값은 ref에서 call-time 읽기 ──
  const drawVideoLoop = useCallback(() => {
    const video   = videoRef.current;
    const display = displayRef.current;
    if (!video || !display) {
      rafRef.current = requestAnimationFrame(drawVideoLoop);
      return;
    }

    if (video.readyState >= 2) {
      // 캔버스 크기 동기화
      if (display.width !== display.clientWidth && display.clientWidth > 0) {
        display.width  = display.clientWidth;
        display.height = display.clientHeight;
      }
      const W = display.width || 640;
      const H = display.height || 480;
      const ctx = display.getContext('2d');

      // ① RealSense 영상 그리기
      ctx.drawImage(video, 0, 0, W, H);

      // ② 손 스켈레톤 오버레이 (handDataRef.current 는 항상 최신)
      const hand = handDataRef.current;
      if (hand) {
        const { landmarks: lms, pointing, indexX, indexY } = hand;
        const dets   = detectionsRef.current;
        const color  = pointing ? '#C4956A' : '#7A9E5A';
        const pts    = lms.map(l => ({ x: l.x * W, y: l.y * H }));

        ctx.lineWidth   = 2;
        ctx.strokeStyle = color;
        HAND_CONNECTIONS.forEach(([a, b]) => {
          ctx.beginPath();
          ctx.moveTo(pts[a].x, pts[a].y);
          ctx.lineTo(pts[b].x, pts[b].y);
          ctx.stroke();
        });
        pts.forEach((pt, i) => {
          const isTip = [4, 8, 12, 16, 20].includes(i);
          ctx.beginPath();
          ctx.arc(pt.x, pt.y, isTip ? 5 : 3, 0, Math.PI * 2);
          ctx.fillStyle = isTip ? '#fff' : color;
          ctx.fill();
        });

        // ③ 호버 판정
        const tipX = indexX * W;
        const tipY = indexY * H;
        let hoverName = null, hoverBox = null;

        if (pointing) {
          const sx = W / (video.videoWidth  || 640);
          const sy = H / (video.videoHeight || 480);
          for (const det of dets) {
            const [x1, y1, x2, y2] = det.box;
            if (tipX >= x1*sx && tipX <= x2*sx && tipY >= y1*sy && tipY <= y2*sy) {
              hoverName = det.name;
              hoverBox  = [x1*sx, y1*sy, x2*sx, y2*sy];
              break;
            }
          }
          ctx.beginPath();
          ctx.arc(tipX, tipY, 12, 0, Math.PI * 2);
          ctx.strokeStyle = hoverName ? '#C89040' : '#C4956A';
          ctx.lineWidth   = 2;
          ctx.stroke();
          if (hoverName) {
            ctx.font      = '700 13px Nanum Gothic';
            ctx.fillStyle = '#C89040';
            ctx.fillText(`☝ → ${hoverName}`, tipX + 16, tipY - 8);
          }
        }

        // ④ 호버 프로그레스 & 선택 확정
        const hover = hoverRef.current;
        const now   = Date.now();
        if (hoverName) {
          if (hoverName === hover.target && hover.startTime) {
            const elapsed  = (now - hover.startTime) / 1000;
            const progress = Math.min(elapsed / HOVER_SEC, 1);
            if (hoverBox) {
              const [sx1, sy1, sx2] = hoverBox;
              const bw = sx2 - sx1;
              ctx.fillStyle = 'rgba(28,18,8,0.75)';
              ctx.fillRect(sx1, sy1 - 16, bw, 10);
              ctx.fillStyle = '#C89040';
              ctx.fillRect(sx1, sy1 - 16, bw * progress, 10);
              ctx.strokeStyle = '#C89040';
              ctx.lineWidth   = 1;
              ctx.strokeRect(sx1, sy1 - 16, bw, 10);
              ctx.font      = '700 12px Nanum Gothic';
              ctx.fillStyle = '#C89040';
              ctx.fillText(`${hoverName}  ${Math.round(progress * 100)}%`, sx1, sy1 - 20);
            }
            setHoverInfo({ label: hoverName, progress });
            if (elapsed >= HOVER_SEC) {
              const det = detectionsRef.current.find(d => d.name === hoverName);
              // orig_box: 원본 해상도 bbox (pick_and_place depth 조회용)
              const box = det?.orig_box || det?.box || [];
              sendChannel({ type: 'select', label: hoverName, box: box, confidence: det?.conf || 0 });
              setSelected({ label: hoverName, confidence: det?.conf || 0 });
              setHoverInfo(null);
              hoverRef.current = { target: null, startTime: null };
              setTimeout(() => setSelected(null), 3000);
            }
          } else {
            hoverRef.current = { target: hoverName, startTime: now };
          }
        } else {
          if (hover.target) {
            hoverRef.current = { target: null, startTime: null };
            setHoverInfo(null);
          }
        }
      } else {
        // 손 없으면 호버 초기화
        if (hoverRef.current.target) {
          hoverRef.current = { target: null, startTime: null };
          setHoverInfo(null);
        }
      }
    }

    rafRef.current = requestAnimationFrame(drawVideoLoop);
  }, []); // [] - refs는 call-time에 읽히므로 항상 최신값

  // ── cleanup ───────────────────────────────────────────────────────────────
  function cleanup() {
    cancelAnimationFrame(rafRef.current);
    if (channelRef.current) {
      channelRef.current.onopen = null;
      channelRef.current.onclose = null;
      channelRef.current.onmessage = null;
      channelRef.current = null;
    }
    if (pcRef.current) {
      pcRef.current.onconnectionstatechange = null;
      pcRef.current.onicegatheringstatechange = null;
      pcRef.current.ontrack = null;
      pcRef.current.close();
      pcRef.current = null;
    }
  }

  // ── scheduleReconnect ─────────────────────────────────────────────────────
  function scheduleReconnect() {
    clearTimeout(reconnectTimer.current);
    let count = RECONNECT_SEC;
    setReconnectIn(count);
    const tick = () => {
      count--; setReconnectIn(count);
      if (count <= 0) { reconnectCount.current++; doConnectRef.current?.(); }
      else reconnectTimer.current = setTimeout(tick, 1000);
    };
    reconnectTimer.current = setTimeout(tick, 1000);
  }

  // ── camera online 감시 ────────────────────────────────────────────────────
  useEffect(() => {
    const t = setInterval(() => {
      if (status !== 'connected') return;
      const video = videoRef.current;
      const isPlaying = video && !video.paused && video.readyState >= 2;
      if (isPlaying) {
        setCameraOnline(true);
      } else if (!lastDetRef.current || Date.now() - lastDetRef.current > 3000) {
        setCameraOnline(false);
      }
    }, 500);
    return () => clearInterval(t);
  }, [status]);

  // ── connect ───────────────────────────────────────────────────────────────
  const connect = useCallback(async () => {
    clearTimeout(reconnectTimer.current);
    clearInterval(pollRef.current);
    setReconnectIn(0);
    setStatus('connecting');
    cleanup();

    try {
      const pc = new RTCPeerConnection({ iceServers: ICE_SERVERS });
      pcRef.current = pc;

      pc.ontrack = (e) => {
        const video = videoRef.current;
        if (video) {
          video.srcObject = e.streams[0];
          video.play()
            .then(() => {
              rafRef.current = requestAnimationFrame(drawVideoLoop);
              setCameraOnline(true); // 영상 재생 시작 = 카메라 연결됨
            })
            .catch(() => {});
        }
      };

      const ch = pc.createDataChannel('vision', { ordered: false, maxRetransmits: 0 });
      channelRef.current = ch;
      ch.onopen  = () => { setStatus('connected'); reconnectCount.current = 0; if (onConnect) onConnect(true); };
      ch.onclose = () => { if (pcRef.current) { setStatus('disconnected'); if (onConnect) onConnect(false); scheduleReconnect(); } };
      ch.onmessage = (e) => {
        try {
          const data = JSON.parse(e.data);
          if (data.type === 'yolo_detections') {
            lastDetRef.current = Date.now(); setCameraOnline(true);
            detectionsRef.current = data.detections || [];
            setDetections(data.detections || []);
          }
        } catch (_) {}
      };

      pc.onconnectionstatechange = () => {
        if (pc.connectionState === 'failed') { setStatus('disconnected'); scheduleReconnect(); }
      };

      pc.onicegatheringstatechange = async () => {
        if (pc.iceGatheringState !== 'complete') return;
        answeredRef.current = false;
        await fetch(`${SIGNALING_URL}/offer`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'ngrok-skip-browser-warning': '1' },
          body: JSON.stringify({ room: ROOM, sdp: pc.localDescription.sdp, type: pc.localDescription.type }),
        });
        clearInterval(pollRef.current);
        pollRef.current = setInterval(async () => {
          if (answeredRef.current) { clearInterval(pollRef.current); return; }
          if (!pcRef.current || pc.signalingState === 'closed') { clearInterval(pollRef.current); return; }
          try {
            const resp = await fetch(`${SIGNALING_URL}/answer/${ROOM}`, { headers: { 'ngrok-skip-browser-warning': '1' } });
            if (resp.status !== 200) return;
            const data = await resp.json();
            if (data.status === 'waiting' || answeredRef.current) return;
            answeredRef.current = true;
            clearInterval(pollRef.current);
            if (pc.signalingState === 'have-local-offer')
              await pc.setRemoteDescription(new RTCSessionDescription({ sdp: data.sdp, type: data.type }));
          } catch (e) { if (!answeredRef.current) console.error('Answer 폴링 오류:', e); }
        }, 1000);
      };

      pc.addTransceiver('video', { direction: 'recvonly' });
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
    } catch (e) {
      console.error('연결 오류:', e); setStatus('disconnected'); scheduleReconnect();
    }
  }, [drawVideoLoop, onConnect]); // eslint-disable-line

  useEffect(() => { doConnectRef.current = connect; }, [connect]);

  // ── disconnect ────────────────────────────────────────────────────────────
  const disconnect = useCallback(() => {
    clearTimeout(reconnectTimer.current);
    clearInterval(pollRef.current);
    reconnectCount.current = 0; setReconnectIn(0);
    cleanup();
    fetch(`${SIGNALING_URL}/clear/${ROOM}`, { method: 'DELETE', headers: { 'ngrok-skip-browser-warning': '1' } }).catch(() => {});
    if (videoRef.current) videoRef.current.srcObject = null;
    setStatus('disconnected'); setDetections([]); setHoverInfo(null);
    setSelected(null); setCameraOnline(false);
    detectionsRef.current = []; hoverRef.current = { target: null, startTime: null }; lastDetRef.current = null;
    handDataRef.current = null;
  }, []); // eslint-disable-line

  useEffect(() => () => { clearTimeout(reconnectTimer.current); disconnect(); }, [disconnect]);

  // ── ref 노출: connect / disconnect / updateHandData ───────────────────────
  useImperativeHandle(ref, () => ({
    connect,
    disconnect,
    updateHandData: (data) => { handDataRef.current = data; },
  }), [connect, disconnect]);

  const statusColor = { connected: '#7A9E5A', connecting: '#C89040', disconnected: '#A89080' }[status] || '#A89080';

  return (
    <div className="vc-root">
      <div className="vc-header">
        <span className="vc-title">◈ VISION MONITOR</span>
        <div className="vc-status">
          <span className="vc-dot" style={{ background: statusColor, boxShadow: `0 0 6px ${statusColor}` }}/>
          <span style={{ color: statusColor, fontSize: 10 }}>{status.toUpperCase()}</span>
        </div>
      </div>

      <video ref={videoRef} style={{ display: 'none' }} playsInline muted/>

      <div className="vc-video-wrap">
        <canvas ref={displayRef} className="vc-canvas"/>

        {status === 'connecting' && (
          <div className="vc-overlay"><div className="vc-spinner"/><div className="vc-overlay-text">연결 중...</div></div>
        )}
        {status === 'disconnected' && reconnectIn > 0 && (
          <div className="vc-reconnect-overlay">
            <div className="vc-reconnect-text">
              {reconnectCount.current > 0 ? `재연결 시도 ${reconnectCount.current}회` : '연결이 끊겼습니다'}
            </div>
            <div className="vc-reconnect-text" style={{ fontSize: 20, color: '#C4956A', marginTop: 4 }}>
              {reconnectIn}초 후 재연결
            </div>
            <button className="vc-reconnect-btn" onClick={() => { clearTimeout(reconnectTimer.current); setReconnectIn(0); connect(); }}>
              지금 재연결
            </button>
          </div>
        )}
        {status === 'disconnected' && reconnectIn === 0 && (
          <div className="vc-overlay"><div className="vc-overlay-text">연결 안 됨</div></div>
        )}
        {status === 'connected' && !cameraOnline && (
          <div className="vc-overlay">
            <div className="vc-camera-icon">📷</div>
            <div className="vc-overlay-text">카메라를 연결해주세요</div>
          </div>
        )}
        {selected && (
          <div className="vc-selected-badge">
            <span className="vc-selected-icon">✓</span>
            <span className="vc-selected-label">{selected.label}</span>
            <span className="vc-selected-conf">{(selected.confidence * 100).toFixed(0)}%</span>
          </div>
        )}
      </div>

      <div className="vc-info">
        <div className="vc-info-row">
          <span className="vc-info-key">OBJECTS</span>
          <span className="vc-info-val">{detections.length > 0 ? detections.map(d => d.name).join(', ') : '–'}</span>
        </div>
        <div className="vc-info-row">
          <span className="vc-info-key">COUNT</span>
          <span className="vc-info-val">{detections.length}</span>
        </div>
        {hoverInfo && (
          <div className="vc-info-row">
            <span className="vc-info-key">HOVER</span>
            <span className="vc-info-val" style={{ color: '#C89040' }}>
              {hoverInfo.label} {Math.round(hoverInfo.progress * 100)}%
            </span>
          </div>
        )}
      </div>
    </div>
  );
});

export default VisionCanvas;