// seed.mjs — Firebase 테스트 데이터 입력
// 실행: node seed.mjs

import { initializeApp } from 'firebase/app';
import {
  getFirestore, doc, setDoc, addDoc,
  collection, Timestamp
} from 'firebase/firestore';

const app = initializeApp({
  apiKey     : "AIzaSyDHe2LXAZ-ivauB7w_k4ITy7WavpwuqPRk",
  authDomain : "jarvis-senior-robo.firebaseapp.com",
  projectId  : "jarvis-senior-robo",
  appId      : "1:188701557160:web:93d5b12947b1b2460ddd9d",
});

const db  = getFirestore(app);
const UID = 'user_01';

console.log('🌱 JARVIS 테스트 데이터 입력 시작...\n');

// ── 프로필 ───────────────────────────────────────────────────────────────────
await setDoc(doc(db, 'users', UID, 'data', 'profile'), {
  name             : '홍길동',
  age              : 75,
  phone            : '010-1234-5678',
  emergencyContact : '홍철수',
  emergencyPhone   : '010-9876-5432',
  notes            : '당뇨 및 고혈압 관리 중. 견과류 알레르기 있음.',
});
console.log('✅ 프로필 입력 완료');

// ── 건강 데이터 (최근 7일) ────────────────────────────────────────────────────
const healthData = [
  { bloodSugar: 112, bloodPressure: '118/76', weight: 65.2 },
  { bloodSugar: 128, bloodPressure: '122/80', weight: 65.0 },
  { bloodSugar: 105, bloodPressure: '116/74', weight: 65.3 },
  { bloodSugar: 145, bloodPressure: '130/85', weight: 65.1 },
  { bloodSugar: 98,  bloodPressure: '114/72', weight: 64.9 },
  { bloodSugar: 118, bloodPressure: '120/78', weight: 65.2 },
  { bloodSugar: 110, bloodPressure: '117/75', weight: 65.0 },
];

for (let i = 0; i < 7; i++) {
  const d = new Date();
  d.setDate(d.getDate() - i);
  const date = d.toISOString().split('T')[0];
  await setDoc(doc(db, 'users', UID, 'health', date), {
    ...healthData[i],
    updatedAt: Timestamp.fromDate(d),
  });
}
console.log('✅ 건강 데이터 7일치 입력 완료');

// ── 복약 관리 ────────────────────────────────────────────────────────────────
const medications = [
  { name: '혈압약 (암로디핀)',  time: '아침',   dose: '1정',  taken: true,  notes: '식후 30분' },
  { name: '당뇨약 (메트포르민)', time: '점심',  dose: '1정',  taken: false, notes: '식사 중' },
  { name: '혈압약 (로사르탄)',  time: '저녁',   dose: '1정',  taken: false, notes: '식후 30분' },
  { name: '종합 영양제',        time: '아침',   dose: '2정',  taken: true,  notes: '식후' },
  { name: '오메가3',            time: '저녁',   dose: '1캡슐', taken: false, notes: '식후' },
];

for (const m of medications) {
  await addDoc(collection(db, 'users', UID, 'medications'), m);
}
console.log('✅ 복약 데이터 입력 완료');

// ── 선호도 ───────────────────────────────────────────────────────────────────
await setDoc(doc(db, 'users', UID, 'data', 'preferences'), {
  favoriteFoods : ['사과', '바나나', '빵', '물', '음료수'],
  allergies     : ['견과류', '갑각류'],
});
console.log('✅ 선호도 입력 완료');

// ── 요청 기록 ────────────────────────────────────────────────────────────────
const history = [
  { voiceText: '물 가져다 줘',      intent: 'bring_object',  targetObject: 'water',    success: true,  situation: '수분 섭취 요청' },
  { voiceText: '약 줘',             intent: 'take_medicine', targetObject: 'pill',     success: true,  situation: '복약 보조' },
  { voiceText: '오늘 날씨 어때?',   intent: 'weather_query', targetObject: null,       success: true,  situation: '날씨 조회' },
  { voiceText: '나갈 준비 도와줘',  intent: 'going_out',     targetObject: 'bag',      success: true,  situation: '외출 준비' },
  { voiceText: '사과 가져다줘',     intent: 'bring_object',  targetObject: 'apple',    success: true,  situation: '간식 요청' },
  { voiceText: '도와줘',            intent: 'emergency',     targetObject: null,       success: true,  situation: '긴급 상황' },
  { voiceText: '바나나 줘',         intent: 'bring_object',  targetObject: 'banana',   success: false, situation: '물체 미감지' },
  { voiceText: '영양제 줘',         intent: 'take_medicine', targetObject: 'pill',     success: true,  situation: '복약 보조' },
  { voiceText: '우산 가져다줘',     intent: 'bring_object',  targetObject: 'umbrella', success: true,  situation: '외출 준비물' },
  { voiceText: '아인슈타인이 누구야?', intent: 'general_query', targetObject: null,    success: true,  situation: '일반 질문' },
];

for (let i = 0; i < history.length; i++) {
  const d = new Date();
  d.setHours(d.getHours() - i * 2);
  await addDoc(collection(db, 'users', UID, 'history'), {
    ...history[i],
    timestamp: Timestamp.fromDate(d),
  });
}
console.log('✅ 요청 기록 10개 입력 완료');

console.log('\n🎉 모든 테스트 데이터 입력 완료!');
process.exit(0);
