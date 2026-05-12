// src/components/GestureCanvas.jsx

import { useEffect, useRef, useState, useCallback, forwardRef, useImperativeHandle } from 'react';
import './GestureCanvas.css';

const DEAD_ZONE         = 0.20;
const CTRL_DEAD         = 0.08;
const DEPTH_DEAD        = 0.015;
const SMOOTHING_FRAMES  = 8;
const CALIB_SEC         = 2.5;
const FIST_FINGER_COUNT = 3;
const DEPTH_MAX_RANGE   = 0.10;
const MIN_STEP_MM       = 20.0;
const MAX_STEP_MM       = 80.0;
const CTRL_MAX_RANGE    = 0.5 - CTRL_DEAD;
const SMOOTH_ALPHA      = 0.15;
const CMD_CHANGE_THR    = 0.5;
const RANGE_X_MM        = 250.0;
const RANGE_Y_MM        = 250.0;
const RANGE_Z_MM        = 180.0;
const LIMIT_X_MM        = 280.0;
const LIMIT_Y_MM        = 250.0;
const LIMIT_Z_MM        = 210.0;
const SEND_RATE_HZ      = 20;
const RECONNECT_SEC     = 3;

const STATE = {
  WAITING    : 'WAITING',
  CALIBRATING: 'CALIBRATING',
  CONTROLLING: 'CONTROLLING',
  PAUSED     : 'PAUSED',
};

const COLORS = {
  WAITING    : '#A89080',
  CALIBRATING: '#7A9E5A',
  CONTROLLING: '#C4956A',
  PAUSED     : '#8B5E3C',
  NONE       : '#6B5040',
};

const HAND_CONNECTIONS = [
  [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],
  [5,9],[9,10],[10,11],[11,12],[9,13],[13,14],[14,15],[15,16],
  [13,17],[17,18],[18,19],[19,20],[0,17],
];

const SIGNALING_URL = import.meta.env.VITE_SIGNALING_URL || 'http://localhost:5000';
const ROOM          = 'jarvis-gesture';
const ICE_SERVERS   = [
  { urls: 'stun:stun.l.google.com:19302' },
  { urls: 'turn:openrelay.metered.ca:80',  username: 'openrelayproject', credential: 'openrelayproject' },
  { urls: 'turn:openrelay.metered.ca:443', username: 'openrelayproject', credential: 'openrelayproject' },
];

// ── utils ─────────────────────────────────────────────────────────────────────

function isFist(lms) {
  let c=0;
  [[8,6],[12,10],[16,14],[20,18]].forEach(([t,p])=>{ if(lms[t].y>lms[p].y) c++; });
  return c >= FIST_FINGER_COUNT;
}
function isIndexPointing(lms) {
  return lms[8].y<lms[6].y && [[12,10],[16,14],[20,18]].every(([t,p])=>lms[t].y>lms[p].y);
}
function getDist(lms) {
  return Math.sqrt((lms[8].x-lms[0].x)**2+(lms[8].y-lms[0].y)**2);
}
function isInCenter(x,y) {
  return Math.abs(x-0.5)<=DEAD_ZONE && Math.abs(y-0.5)<=DEAD_ZONE;
}
function proportionalStep(offset,dead,maxRange) {
  if(Math.abs(offset)<=dead) return 0;
  return -(MIN_STEP_MM+Math.min((Math.abs(offset)-dead)/maxRange,1)*(MAX_STEP_MM-MIN_STEP_MM))*Math.sign(offset);
}
function computeDelta(ax,ay,cd,bd) {
  const od=cd-bd;
  return [
    proportionalStep(ax-0.5,CTRL_DEAD,CTRL_MAX_RANGE),
    proportionalStep(od*(0.5/DEPTH_MAX_RANGE),DEPTH_DEAD*(0.5/DEPTH_MAX_RANGE),0.5-DEPTH_DEAD*(0.5/DEPTH_MAX_RANGE)),
    proportionalStep(ay-0.5,CTRL_DEAD,CTRL_MAX_RANGE),
  ];
}
function computeTarget(ax,ay,cd,bd,tcp) {
  let ox=ax-0.5,oy=ay-0.5,od=cd-bd;
  if(Math.abs(ox)<CTRL_DEAD)ox=0; if(Math.abs(oy)<CTRL_DEAD)oy=0; if(Math.abs(od)<DEPTH_DEAD)od=0;
  const remap=(v,d)=>!v?0:Math.sign(v)*Math.min((Math.abs(v)-d)/(0.5-d),1);
  const rx=remap(ox,CTRL_DEAD),ry=remap(oy,CTRL_DEAD),rd=Math.max(-1,Math.min(1,od/DEPTH_MAX_RANGE));
  const cl=(v,c,l)=>Math.max(c-l,Math.min(c+l,v));
  return [cl(tcp[0]-rx*RANGE_X_MM,tcp[0],LIMIT_X_MM),cl(tcp[1]+rd*RANGE_Y_MM,tcp[1],LIMIT_Y_MM),
          cl(tcp[2]-ry*RANGE_Z_MM,tcp[2],LIMIT_Z_MM),tcp[3],tcp[4],tcp[5]];
}

class SmoothQueue {
  constructor(n=SMOOTHING_FRAMES){ this.buf=[]; this.n=n; }
  push(v){ this.buf.push(v); if(this.buf.length>this.n) this.buf.shift(); }
  mean(){ return this.buf.length?this.buf.reduce((s,v)=>s+v,0)/this.buf.length:0; }
  clear(){ this.buf=[]; }
}

// ── 드로잉 ───────────────────────────────────────────────────────────────────

function drawHand(ctx,lms,stateKey,W,H) {
  const fist=isFist(lms),pointing=isIndexPointing(lms);
  const color=fist?'#8B4513':pointing?'#C4956A':COLORS[stateKey]||COLORS.NONE;
  const pts=lms.map(l=>({x:l.x*W,y:l.y*H}));
  ctx.shadowBlur=0; ctx.lineWidth=2; ctx.strokeStyle=color;
  HAND_CONNECTIONS.forEach(([a,b])=>{ ctx.beginPath(); ctx.moveTo(pts[a].x,pts[a].y); ctx.lineTo(pts[b].x,pts[b].y); ctx.stroke(); });
  pts.forEach((pt,i)=>{
    const isTip=[4,8,12,16,20].includes(i);
    ctx.beginPath(); ctx.arc(pt.x,pt.y,isTip?5.5:3,0,Math.PI*2);
    ctx.fillStyle=isTip?'#fff':color; ctx.fill();
    ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.stroke();
  });
}

function drawUI(ctx,sm,avgX,avgY,W,H) {
  const color=COLORS[sm.state]||COLORS.NONE,midX=W/2,midY=H/2;
  ctx.save(); ctx.setLineDash([4,4]);
  ctx.strokeStyle='rgba(196,149,106,0.25)'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(midX,0); ctx.lineTo(midX,H); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0,midY); ctx.lineTo(W,midY); ctx.stroke();
  ctx.setLineDash([]); ctx.restore();

  const zoneW=DEAD_ZONE*2*W, sz=Math.round(zoneW*0.85);
  ctx.save(); ctx.globalAlpha=sm.state===STATE.CALIBRATING?0.55:0.13;
  ctx.font=`${sz}px serif`; ctx.textAlign='center'; ctx.textBaseline='middle';
  ctx.fillText('🖐',midX,midY+sz*0.05); ctx.restore();

  const radius=zoneW*0.48;
  if(sm.state===STATE.CALIBRATING&&sm.calibProgress>0){
    ctx.beginPath(); ctx.arc(midX,midY,radius,0,Math.PI*2);
    ctx.strokeStyle='rgba(122,158,90,0.2)'; ctx.lineWidth=6; ctx.stroke();
    ctx.beginPath(); ctx.arc(midX,midY,radius,-Math.PI/2,-Math.PI/2+sm.calibProgress*Math.PI*2);
    ctx.strokeStyle=COLORS.CALIBRATING; ctx.lineWidth=6;
    ctx.shadowBlur=8; ctx.shadowColor=COLORS.CALIBRATING; ctx.stroke(); ctx.shadowBlur=0;
    ctx.font='700 16px Inter'; ctx.fillStyle=COLORS.CALIBRATING;
    ctx.textAlign='center'; ctx.textBaseline='middle';
    ctx.fillText(`${Math.round(sm.calibProgress*100)}%`,midX,midY+radius+16);
    ctx.textAlign='left'; ctx.textBaseline='alphabetic';
  }

  ctx.strokeStyle='rgba(107,80,64,0.35)'; ctx.lineWidth=1; ctx.setLineDash([3,3]);
  ctx.strokeRect((0.5-CTRL_DEAD)*W,(0.5-CTRL_DEAD)*H,CTRL_DEAD*2*W,CTRL_DEAD*2*H);
  ctx.setLineDash([]);

  if(avgX!=null){
    const cx=avgX*W,cy=avgY*H;
    ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.setLineDash([4,4]);
    ctx.beginPath(); ctx.moveTo(midX,midY); ctx.lineTo(cx,midY); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(midX,midY); ctx.lineTo(midX,cy); ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(cx,cy,10,0,Math.PI*2);
    ctx.fillStyle=color; ctx.globalAlpha=0.85; ctx.fill(); ctx.globalAlpha=1;
    ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.stroke();
    ctx.font='700 13px Inter'; ctx.fillStyle=color; ctx.fillText('INDEX',cx+14,cy-6);
  }

  ctx.font='800 15px Inter'; ctx.fillStyle='rgba(196,149,106,0.9)';
  ctx.fillText('LEFT', 14, midY-4);
  ctx.textAlign='right'; ctx.fillText('RIGHT', W-14, midY-4); ctx.textAlign='left';
  ctx.textAlign='center';
  ctx.fillText('UP', midX, (0.5-DEAD_ZONE)*H+18);
  ctx.fillText('DOWN', midX, H-10);
  ctx.textAlign='left';

  ctx.fillStyle='rgba(28,18,8,0.80)'; ctx.fillRect(0,0,W,54);
  const labels={WAITING:'→ 손을 CENTER로 이동',CALIBRATING:`캘리브레이션 중... ${Math.round(sm.calibProgress*100)}%`,
    CONTROLLING:'제어 중 (주먹 → 일시정지)',PAUSED:'일시정지 (손 펴기 → 재개)'};
  ctx.font='800 17px Nanum Gothic'; ctx.fillStyle=color; ctx.textAlign='center';
  ctx.fillText(`[${sm.state}]  ${labels[sm.state]||''}`,W/2,32); ctx.textAlign='left';
  if(sm.state===STATE.CONTROLLING&&sm.targetPos){
    const t=sm.targetPos; ctx.font='700 13px DM Mono'; ctx.fillStyle='rgba(196,149,106,1)';
    ctx.fillText(`X:${t[0].toFixed(0)}  Y:${t[1].toFixed(0)}  Z:${t[2].toFixed(0)} mm`,10,48);
  }
  if(sm.state===STATE.PAUSED){
    ctx.fillStyle='rgba(40,20,10,0.5)'; ctx.fillRect(0,0,W,H);
    ctx.font='700 44px Nanum Gothic'; ctx.fillStyle='rgba(139,94,60,0.9)';
    ctx.textAlign='center'; ctx.fillText('일시정지',W/2,H/2+16); ctx.textAlign='left';
  }
  if(sm.state===STATE.CONTROLLING&&avgX!=null)
    drawWorkspaceGrid(ctx,avgX,avgY,sm.calibTcp,sm.targetPos,W,H);

  ctx.textAlign='left'; // 단축키 힌트 제거
}

function drawWorkspaceGrid(ctx,avgX,avgY,calibTcp,targetPos,W,H){
  const gw=140,gh=110,mg=12,x0=W-gw-mg,y0=H-gh-mg-28;
  ctx.fillStyle='rgba(28,18,8,0.80)'; ctx.fillRect(x0-4,y0-18,gw+8,gh+22);
  ctx.strokeStyle='rgba(107,66,38,0.4)'; ctx.lineWidth=1; ctx.strokeRect(x0-4,y0-18,gw+8,gh+22);
  ctx.font='700 11px Inter'; ctx.fillStyle='rgba(168,144,128,0.9)'; ctx.fillText('WORKSPACE',x0-2,y0-5);
  for(let i=0;i<=4;i++){
    ctx.strokeStyle='rgba(107,66,38,0.2)';
    ctx.beginPath(); ctx.moveTo(x0+i/4*gw,y0); ctx.lineTo(x0+i/4*gw,y0+gh); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x0,y0+i/4*gh); ctx.lineTo(x0+gw,y0+i/4*gh); ctx.stroke();
  }
  const cx=x0+gw/2,cz=y0+gh/2;
  ctx.strokeStyle='rgba(168,144,128,0.5)'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(cx-6,cz); ctx.lineTo(cx+6,cz); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx,cz-6); ctx.lineTo(cx,cz+6); ctx.stroke();
  if(calibTcp&&targetPos){
    const tpx=Math.max(x0,Math.min(x0+gw,cx-(targetPos[0]-calibTcp[0])/LIMIT_X_MM*gw/2));
    const tpz=Math.max(y0,Math.min(y0+gh,cz-(targetPos[2]-calibTcp[2])/LIMIT_Z_MM*gh/2));
    ctx.fillStyle='#7A9E5A'; ctx.beginPath(); ctx.arc(tpx,tpz,6,0,Math.PI*2); ctx.fill();
    ctx.strokeStyle='#fff'; ctx.lineWidth=1; ctx.stroke();
  }
  if(avgX!=null){
    ctx.fillStyle='#C4956A';
    ctx.beginPath(); ctx.arc(Math.max(x0,Math.min(x0+gw,x0+avgX*gw)),Math.max(y0,Math.min(y0+gh,y0+avgY*gh)),5,0,Math.PI*2); ctx.fill();
  }
}

// ── CDN 로드 ──────────────────────────────────────────────────────────────────

function loadScript(src){
  return new Promise((resolve,reject)=>{
    if(document.querySelector(`script[src="${src}"]`)){resolve();return;}
    const s=document.createElement('script'); s.src=src; s.crossOrigin='anonymous';
    s.onload=resolve; s.onerror=reject; document.head.appendChild(s);
  });
}
async function loadMediaPipe(){
  await loadScript('https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4.1675469240/hands.js');
  await loadScript('https://cdn.jsdelivr.net/npm/@mediapipe/camera_utils@0.3.1675466862/camera_utils.js');
}

// ── 공상과학 배경 ────────────────────────────────────────────────────────────────────
function drawSciFiBg(ctx, W, H) {
  const cx=W/2, cy=H/2, t=Date.now()/1000;
  const A=(a,v=1)=>`rgba(196,149,106,${(v*a).toFixed(3)})`;
  const G=(a,v=1)=>`rgba(122,158,90,${(v*a).toFixed(3)})`;

  // ① 방사형 그라데이션 배경 (중앙이 살짝 밝게)
  const radGrad=ctx.createRadialGradient(cx,cy,0,cx,cy,Math.max(W,H)*0.7);
  radGrad.addColorStop(0,'rgba(40,22,8,1)');
  radGrad.addColorStop(1,'rgba(10,5,2,1)');
  ctx.fillStyle=radGrad; ctx.fillRect(0,0,W,H);

  // ② 육각형 그리드
  const hr=44;
  ctx.lineWidth=0.5;
  for(let row=-1;row<Math.ceil(H/(hr*1.732))+2;row++){
    for(let col=-1;col<Math.ceil(W/(hr*2))+2;col++){
      const hx=col*hr*2+(row%2)*hr, hy=row*hr*1.732;
      const dist=Math.hypot(hx-cx,hy-cy)/Math.max(W,H);
      const alpha=Math.max(0,0.13-dist*0.18);
      if(alpha<=0) continue;
      ctx.beginPath();
      for(let i=0;i<6;i++){
        const a=Math.PI/3*i+Math.PI/6;
        i===0?ctx.moveTo(hx+hr*Math.cos(a),hy+hr*Math.sin(a))
             :ctx.lineTo(hx+hr*Math.cos(a),hy+hr*Math.sin(a));
      }
      ctx.closePath();
      ctx.strokeStyle=A(alpha); ctx.stroke();
    }
  }

  // ③ 동심 레이더 링
  for(let i=1;i<=6;i++){
    const r=Math.min(W,H)*0.09*i;
    const pulse=i===2?(Math.sin(t*1.5)+1)*0.5:0;
    ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2);
    ctx.strokeStyle=A(0.06+pulse*0.1+i*0.01);
    ctx.lineWidth=i===2?1+pulse:0.5; ctx.stroke();
  }

  // ④ 회전 스캔 부채꼴
  const ang=t*0.5;
  const sR=Math.min(W,H)*0.48;
  const sg=ctx.createConicalGradient?.(cx,cy,ang-0.5,ang) ??
    ctx.createLinearGradient(cx,cy,cx+Math.cos(ang)*sR,cy+Math.sin(ang)*sR);
  sg.addColorStop(0,A(0)); sg.addColorStop(1,A(0.14));
  ctx.save(); ctx.beginPath(); ctx.moveTo(cx,cy);
  ctx.arc(cx,cy,sR,ang-0.45,ang); ctx.closePath();
  ctx.fillStyle=A(0.11); ctx.fill(); ctx.restore();

  // ⑤ 스캔 선 (날카로운 라인)
  ctx.save();
  ctx.translate(cx,cy); ctx.rotate(ang);
  const lg=ctx.createLinearGradient(0,0,sR,0);
  lg.addColorStop(0,A(0.5)); lg.addColorStop(1,A(0));
  ctx.strokeStyle=lg; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(sR,0); ctx.stroke();
  ctx.restore();

  // ⑥ 십자선 (중앙 크로스헤어)
  const cLen=18, gap=6;
  ctx.strokeStyle=A(0.35); ctx.lineWidth=1;
  [[-1,0],[1,0],[0,-1],[0,1]].forEach(([dx,dy])=>{
    ctx.beginPath();
    ctx.moveTo(cx+dx*gap,cy+dy*gap);
    ctx.lineTo(cx+dx*(gap+cLen),cy+dy*(gap+cLen));
    ctx.stroke();
  });

  // ⑦ 파티클 (별처럼 깜빡이는 점들)
  const seed=42;
  for(let i=0;i<28;i++){
    const px=((Math.sin(i*seed)*0.5+0.5)*W);
    const py=((Math.cos(i*seed*1.3)*0.5+0.5)*H);
    const flicker=(Math.sin(t*1.7+i*2.1)+1)*0.5;
    const size=0.8+flicker*1.2;
    const isGreen=i%5===0;
    ctx.beginPath(); ctx.arc(px,py,size,0,Math.PI*2);
    ctx.fillStyle=isGreen?G(0.3+flicker*0.4):A(0.2+flicker*0.5);
    ctx.fill();
  }

  // ⑧ 수평 스캔 라인 (CRT 효과)
  for(let y=0;y<H;y+=4){
    ctx.fillStyle='rgba(0,0,0,0.06)';
    ctx.fillRect(0,y,W,1);
  }

  // ⑨ 코너 브래킷 (더 정교하게)
  const bL=28,bO=12;
  ctx.lineWidth=1.5; ctx.strokeStyle=A(0.6);
  [[bO,bO,1,1],[W-bO,bO,-1,1],[bO,H-bO,1,-1],[W-bO,H-bO,-1,-1]].forEach(([x,y,dx,dy])=>{
    ctx.beginPath(); ctx.moveTo(x+dx*bL,y); ctx.lineTo(x,y); ctx.lineTo(x,y+dy*bL); ctx.stroke();
    // 작은 내부 점
    ctx.fillStyle=A(0.5); ctx.fillRect(x+dx*2-1,y+dy*2-1,2,2);
  });

  // ⑩ 중앙 펄스 링
  const p=(Math.sin(t*2.2)+1)*0.5;
  ctx.beginPath(); ctx.arc(cx,cy,10+p*6,0,Math.PI*2);
  ctx.strokeStyle=A(0.15+p*0.25); ctx.lineWidth=1+p; ctx.stroke();
  ctx.beginPath(); ctx.arc(cx,cy,3,0,Math.PI*2);
  ctx.fillStyle=A(0.4+p*0.4); ctx.fill();
}

// ── GestureCanvas ─────────────────────────────────────────────────────────────

const GestureCanvas = forwardRef(function GestureCanvas({ onModeChange, onChannel, onHandData }, ref) {
  const videoRef   = useRef(null);
  const canvasRef  = useRef(null);
  const pcRef      = useRef(null);
  const bgImageRef = useRef(null);
  const channelRef = useRef(null);
  const cameraRef  = useRef(null);
  const pollRef    = useRef(null);
  const dataRef    = useRef(null);
  const fpsRef          = useRef({ count:0, last:Date.now() });
  const answeredRef     = useRef(false);
  const reconnectTimer  = useRef(null);
  const reconnectCount  = useRef(0);
  // ★ useCallback 대신 useRef로 함수 저장 → 순환 참조 및 초기화 순서 문제 해결
  const doConnectRef    = useRef(null);

  const sm = useRef({
    state:STATE.WAITING, calibStart:0, calibProgress:0, calibData:[],
    baseDist:0, calibTcp:[0,0,0,0,0,0], targetPos:null,
    delta:[0,0,0], smoothDelta:[0,0,0], prevSent:[0,0,0],
    xHist:new SmoothQueue(), yHist:new SmoothQueue(), dHist:new SmoothQueue(),
  });

  const [status,        setStatus]        = useState('disconnected');
  const [gestureState,  setGestureState]  = useState(STATE.WAITING);
  const [fps,           setFps]           = useState(0);
  const [handVisible,   setHandVisible]   = useState(false);
  const [calibProgress, setCalibProgress] = useState(0);
  const [mpLoaded,      setMpLoaded]      = useState(false);
  const [targetInfo,    setTargetInfo]    = useState(null);
  const [reconnectIn,   setReconnectIn]   = useState(0);

  useEffect(()=>{
    const img=new Image();
    img.src='/scifi-bg.png';
    img.onload=()=>{ bgImageRef.current=img; };
  },[]);

  useEffect(()=>{
    loadMediaPipe().then(()=>setMpLoaded(true)).catch(e=>console.error('MediaPipe 로드 실패:',e));
  },[]);

  useEffect(()=>{
    const t=setInterval(()=>{
      const ch=channelRef.current,d=dataRef.current;
      if(ch?.readyState==='open'&&d) ch.send(JSON.stringify(d));
    },1000/SEND_RATE_HZ);
    return()=>clearInterval(t);
  },[]);

  // ── cleanup (이벤트 핸들러 먼저 제거 후 닫기) ─────────────────────────────
  function cleanup(){
    if(channelRef.current){
      channelRef.current.onopen=null;
      channelRef.current.onclose=null;
      channelRef.current.onmessage=null;
      channelRef.current=null;
    }
    if(pcRef.current){
      pcRef.current.onconnectionstatechange=null;
      pcRef.current.onicegatheringstatechange=null;
      pcRef.current.close();
      pcRef.current=null;
    }
  }

  // ── scheduleReconnect ────────────────────────────────────────────────────
  function scheduleReconnect(){
    clearTimeout(reconnectTimer.current);
    let count=RECONNECT_SEC;
    setReconnectIn(count);
    const tick=()=>{
      count--; setReconnectIn(count);
      if(count<=0){ reconnectCount.current++; doConnectRef.current?.(); }
      else reconnectTimer.current=setTimeout(tick,1000);
    };
    reconnectTimer.current=setTimeout(tick,1000);
  }

  const onResults = useCallback((results)=>{
    const canvas=canvasRef.current; if(!canvas) return;
    const ctx=canvas.getContext('2d'),W=canvas.width,H=canvas.height,s=sm.current;
    drawSciFiBg(ctx,W,H);

    fpsRef.current.count++;
    const now=Date.now();
    if(now-fpsRef.current.last>=1000){ setFps(fpsRef.current.count); fpsRef.current.count=0; fpsRef.current.last=now; }

    let avgX=null,avgY=null,currDist=null,indexX=0,indexY=0;
    let handVis=false,fist=false,pointing=false,lms=null;

    if(results.multiHandLandmarks?.length>0){
      handVis=true;
      lms=results.multiHandLandmarks[0].map(l=>({...l,x:1-l.x}));
      s.xHist.push(lms[8].x); s.yHist.push(lms[8].y); s.dHist.push(getDist(lms));
      avgX=s.xHist.mean(); avgY=s.yHist.mean(); currDist=s.dHist.mean();
      indexX=lms[8].x; indexY=lms[8].y;
      fist=isFist(lms); pointing=isIndexPointing(lms);
      drawHand(ctx,lms,s.state,W,H);
    } else { s.xHist.clear(); s.yHist.clear(); s.dHist.clear(); }

    if(handVis&&avgX!=null){
      const inCenter=isInCenter(avgX,avgY);
      if(s.state===STATE.WAITING){
        if(inCenter){ s.state=STATE.CALIBRATING; s.calibStart=Date.now(); s.calibData=[]; s.calibProgress=0; }
      } else if(s.state===STATE.CALIBRATING){
        if(inCenter){
          const elapsed=(Date.now()-s.calibStart)/1000;
          s.calibProgress=Math.min(elapsed/CALIB_SEC,1.0); s.calibData.push(currDist);
          if(elapsed>=CALIB_SEC){
            s.baseDist=s.calibData.reduce((a,b)=>a+b,0)/s.calibData.length;
            s.calibTcp=s.calibTcp.some(v=>v!==0)?s.calibTcp:[0,0,0,0,0,0];
            s.targetPos=[...s.calibTcp]; s.prevTarget=[...s.calibTcp];
            s.state=STATE.CONTROLLING; s.calibProgress=1.0;
          }
        } else { s.state=STATE.WAITING; }
      } else if(s.state===STATE.CONTROLLING){
        if(fist){ s.state=STATE.PAUSED; s.targetPos=null; s.delta=[0,0,0]; }
        else if(pointing){ s.delta=[0,0,0]; }
        else{
          s.targetPos=computeTarget(avgX,avgY,currDist,s.baseDist,s.calibTcp);
          s.delta=computeDelta(avgX,avgY,currDist,s.baseDist);
          if(!s.delta.some(v=>v)) s.smoothDelta=[0,0,0];
        }
        for(let i=0;i<3;i++) s.smoothDelta[i]=SMOOTH_ALPHA*s.delta[i]+(1-SMOOTH_ALPHA)*s.smoothDelta[i];
        const sd=s.smoothDelta.map(v=>Math.abs(v)>=MIN_STEP_MM?v:0);
        const changed=sd.some((v,i)=>Math.abs(v-s.prevSent[i])>=CMD_CHANGE_THR);
        if(sd.some(v=>v)&&changed) s.prevSent=[...sd];
        else if(!sd.some(v=>v)&&s.prevSent.some(v=>v)) s.prevSent=[0,0,0];
      } else if(s.state===STATE.PAUSED){
        if(!fist) s.state=STATE.CONTROLLING;
      }
    }

    drawUI(ctx,s,avgX,avgY,W,H);
    setHandVisible(handVis); setGestureState(s.state); setCalibProgress(s.calibProgress);
    if(s.targetPos) setTargetInfo([...s.targetPos]);
    if(onModeChange) onModeChange(pointing&&handVis);

    // VisionCanvas에 손 데이터 공유 (MediaPipe 중복 실행 방지)
    if(onHandData) onHandData(handVis&&lms ? { landmarks:lms, pointing, indexX, indexY } : null);

    dataRef.current={
      type:'gesture',gesture_state:s.state,is_pointing:pointing,is_fist:fist,
      hand_visible:handVis,avg_x:avgX??0,avg_y:avgY??0,
      index_tip_x:indexX,index_tip_y:indexY,
      curr_dist:currDist??0,base_dist:s.baseDist,
      calib_progress:s.calibProgress,calib_tcp:s.calibTcp,
      target_pos_mm:s.targetPos??[0,0,0,0,0,0],
      velocity_delta:s.delta,smooth_delta:s.smoothDelta,
      landmarks_x:lms?lms.map(l=>l.x):Array(21).fill(0),
      landmarks_y:lms?lms.map(l=>l.y):Array(21).fill(0),
    };
  },[onModeChange]);

  const startCamera = useCallback(()=>{
    if(!window.Hands||!window.Camera) return;
    const hands=new window.Hands({locateFile:f=>`https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4.1675469240/${f}`});
    hands.setOptions({maxNumHands:1,modelComplexity:1,minDetectionConfidence:0.6,minTrackingConfidence:0.5});
    hands.onResults(onResults);
    const cam=new window.Camera(videoRef.current,{onFrame:async()=>hands.send({image:videoRef.current}),width:640,height:480});
    cam.start(); cameraRef.current=cam;
  },[onResults]);

  // MediaPipe 로드 완료 시점에 이미 connected 상태면 카메라 즉시 시작
  useEffect(()=>{
    if(mpLoaded && status==='connected' && !cameraRef.current) startCamera();
  },[mpLoaded, status, startCamera]);

  const connect = useCallback(async()=>{
    clearTimeout(reconnectTimer.current);
    clearInterval(pollRef.current);
    setReconnectIn(0);
    setStatus('connecting');
    cleanup();

    const s=sm.current;
    s.state=STATE.WAITING; s.calibProgress=0; s.baseDist=0;
    s.targetPos=null; s.delta=[0,0,0]; s.calibTcp=[0,0,0,0,0,0];
    s.xHist.clear(); s.yHist.clear(); s.dHist.clear();

    try{
      const pc=new RTCPeerConnection({iceServers:ICE_SERVERS});
      pcRef.current=pc;
      const ch=pc.createDataChannel('gesture',{ordered:false,maxRetransmits:0});
      channelRef.current=ch;

      ch.onopen=()=>{
        setStatus('connected'); reconnectCount.current=0;
        if(onChannel) onChannel(ch);
        // MediaPipe 로드 완료까지 300ms 간격으로 재시도
        const tryStart=()=>{
          if(!channelRef.current||channelRef.current.readyState!=='open') return;
          if(cameraRef.current) return; // 이미 시작됨
          if(window.Hands&&window.Camera){ startCamera(); }
          else { setTimeout(tryStart,300); }
        };
        tryStart();
      };
      ch.onclose=()=>{ if(pcRef.current){ setStatus('disconnected'); scheduleReconnect(); } };
      ch.onmessage=(e)=>{
        try{
          const d=JSON.parse(e.data);
          if(d.type==='robot_state'&&Array.isArray(d.tcp)) sm.current.calibTcp=d.tcp;
        }catch(_){}
      };
      pc.onconnectionstatechange=()=>{
        if(pc.connectionState==='failed'){ setStatus('disconnected'); scheduleReconnect(); }
      };
      pc.onicegatheringstatechange=async()=>{
        if(pc.iceGatheringState!=='complete') return;
        answeredRef.current=false;
        await fetch(`${SIGNALING_URL}/offer`,{
          method:'POST',
          headers:{'Content-Type':'application/json','ngrok-skip-browser-warning':'1'},
          body:JSON.stringify({room:ROOM,sdp:pc.localDescription.sdp,type:pc.localDescription.type}),
        });
        // answer 폴링
        clearInterval(pollRef.current);
        pollRef.current=setInterval(async()=>{
          if(answeredRef.current){clearInterval(pollRef.current);return;}
          if(!pcRef.current||pc.signalingState==='closed'){clearInterval(pollRef.current);return;}
          try{
            const resp=await fetch(`${SIGNALING_URL}/answer/${ROOM}`,{headers:{'ngrok-skip-browser-warning':'1'}});
            if(resp.status!==200) return;
            const data=await resp.json();
            if(data.status==='waiting') return;
            if(answeredRef.current) return;
            answeredRef.current=true;
            clearInterval(pollRef.current);
            if(pc.signalingState==='have-local-offer')
              await pc.setRemoteDescription(new RTCSessionDescription({sdp:data.sdp,type:data.type}));
          }catch(e){ if(!answeredRef.current) console.error('Answer 폴링 오류:',e); }
        },1000);
      };

      const offer=await pc.createOffer();
      await pc.setLocalDescription(offer);
    }catch(e){
      console.error('연결 오류:',e);
      setStatus('disconnected');
      scheduleReconnect();
    }
  },[startCamera,onChannel]); // eslint-disable-line

  // ★ doConnectRef에 최신 connect 저장
  useEffect(()=>{ doConnectRef.current=connect; },[connect]);

  const disconnect = useCallback(()=>{
    clearTimeout(reconnectTimer.current);
    clearInterval(pollRef.current);
    reconnectCount.current=0;
    setReconnectIn(0);
    cleanup();
    cameraRef.current?.stop();
    cameraRef.current=null;
    fetch(`${SIGNALING_URL}/clear/${ROOM}`,{method:'DELETE',headers:{'ngrok-skip-browser-warning':'1'}}).catch(()=>{});
    setStatus('disconnected'); setHandVisible(false);
    setGestureState(STATE.WAITING); setCalibProgress(0); dataRef.current=null;
    const canvas=canvasRef.current;
    if(canvas) canvas.getContext('2d').clearRect(0,0,canvas.width,canvas.height);
  },[]); // eslint-disable-line

  useEffect(()=>()=>{ clearTimeout(reconnectTimer.current); disconnect(); },[disconnect]);

  // ── 재보정 (WAITING 상태로 리셋) ─────────────────────────────────────────
  const recalibrate = useCallback(()=>{
    const s=sm.current;
    s.state=STATE.WAITING; s.calibProgress=0; s.baseDist=0;
    s.targetPos=null; s.delta=[0,0,0]; s.smoothDelta=[0,0,0];
    s.xHist.clear(); s.yHist.clear(); s.dHist.clear();
    setGestureState(STATE.WAITING); setCalibProgress(0); setTargetInfo(null);
  },[]);

  // ── 키보드 단축키: R=재보정, Space=일시정지 토글 ───────────────────────
  useEffect(()=>{
    const onKey=(e)=>{
      if(status!=='connected') return;
      if(e.code==='KeyR'){ recalibrate(); }
      if(e.code==='Space'){
        e.preventDefault();
        const s=sm.current;
        if(s.state===STATE.CONTROLLING) s.state=STATE.PAUSED;
        else if(s.state===STATE.PAUSED)  s.state=STATE.CONTROLLING;
      }
    };
    window.addEventListener('keydown',onKey);
    return ()=>window.removeEventListener('keydown',onKey);
  },[status, recalibrate]);

  // ── ref로 connect/disconnect 노출 ────────────────────────────────────────
  useImperativeHandle(ref, () => ({ connect, disconnect }), [connect, disconnect]);

  const statusColor={connected:'#7A9E5A',connecting:'#C89040',disconnected:'#A89080'}[status]||'#A89080';
  const stateColor=COLORS[gestureState]||COLORS.NONE;

  return (
    <div className="gc-root">
      <div className="gc-header">
        <span className="gc-title">◈ GESTURE MONITOR</span>
        <div className="gc-status">
          <span className="gc-dot" style={{background:statusColor,boxShadow:`0 0 6px ${statusColor}`}}/>
          <span style={{color:statusColor,fontSize:10}}>{status.toUpperCase()}</span>
        </div>
      </div>

      <video ref={videoRef} style={{display:'none'}} playsInline/>

      <div className="gc-canvas-wrap">
        <canvas ref={canvasRef} width={640} height={480} className="gc-canvas"/>
        {status!=='connected'&&(
          <div className="gc-overlay">
            {status==='connecting'
              ? <><div className="gc-spinner"/><div className="gc-overlay-text">연결 중...</div></>
              : reconnectIn>0
                ? <>
                    <div className="gc-overlay-text">
                      {reconnectCount.current>0?`재연결 시도 ${reconnectCount.current}회`:'연결이 끊겼습니다'}
                    </div>
                    <div className="gc-overlay-text" style={{fontSize:20,color:'#C4956A',marginTop:4}}>
                      {reconnectIn}초 후 재연결
                    </div>
                    <button className="gc-reconnect-btn" onClick={()=>{clearTimeout(reconnectTimer.current);setReconnectIn(0);connect();}}>
                      지금 재연결
                    </button>
                  </>
                : <div className="gc-overlay-text">연결 안 됨</div>
            }
          </div>
        )}
        {/* 재보정 버튼: 캔버스 위에 절대 위치로 띄워서 모니터 크기 변화 없음 */}
        {status==='connected' && (gestureState===STATE.CONTROLLING||gestureState===STATE.PAUSED) && (
          <button
            className="gc-btn gc-btn-recalib"
            onClick={recalibrate}
            style={{position:'absolute', bottom:12, right:12, zIndex:10}}
          >
            ↺ 재보정
          </button>
        )}
      </div>

      <div className="gc-info">
        <div className="gc-info-row">
          <span className="gc-info-key">STATE</span>
          <span className="gc-info-val" style={{color:stateColor}}>{gestureState}</span>
        </div>
        <div className="gc-info-row">
          <span className="gc-info-key">HAND</span>
          <span className="gc-info-val" style={{color:handVisible?'#7A9E5A':'#A89080'}}>
            {handVisible?'감지됨':'없음'}
          </span>
        </div>
        {gestureState===STATE.CALIBRATING&&(
          <div className="gc-info-row">
            <span className="gc-info-key">CALIB</span>
            <span className="gc-info-val" style={{color:'#C89040'}}>{Math.round(calibProgress*100)}%</span>
          </div>
        )}

        <div className="gc-info-row">
          <span className="gc-info-key">FPS</span>
          <span className="gc-info-val">{fps}</span>
        </div>
      </div>



    </div>
  );
});

export default GestureCanvas;