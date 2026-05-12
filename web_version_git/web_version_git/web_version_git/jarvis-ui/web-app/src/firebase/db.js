// src/firebase/db.js
import {
  doc, getDoc, setDoc, updateDoc,
  collection, addDoc, getDocs,
  query, orderBy, limit,
  serverTimestamp,
} from 'firebase/firestore';
import { db } from './config';

const USER_ID = 'user_01'; // 추후 Auth 연동 시 동적으로 변경

// ── 사용자 프로필 ────────────────────────────────────────────────────────────

export async function getProfile() {
  const snap = await getDoc(doc(db, 'users', USER_ID, 'data', 'profile'));
  return snap.exists() ? snap.data() : null;
}

export async function saveProfile(data) {
  await setDoc(doc(db, 'users', USER_ID, 'data', 'profile'), data, { merge: true });
}

// ── 건강 데이터 ──────────────────────────────────────────────────────────────

export async function saveHealth(data) {
  const today = new Date().toISOString().split('T')[0];
  await setDoc(
    doc(db, 'users', USER_ID, 'health', today),
    { ...data, updatedAt: serverTimestamp() },
    { merge: true }
  );
}

export async function getHealthList(count = 7) {
  const q = query(
    collection(db, 'users', USER_ID, 'health'),
    orderBy('updatedAt', 'desc'),
    limit(count)
  );
  const snap = await getDocs(q);
  return snap.docs.map(d => ({ id: d.id, ...d.data() }));
}

// ── 복약 관리 ────────────────────────────────────────────────────────────────

export async function getMedications() {
  const snap = await getDocs(collection(db, 'users', USER_ID, 'medications'));
  return snap.docs.map(d => ({ id: d.id, ...d.data() }));
}

export async function addMedication(data) {
  return await addDoc(
    collection(db, 'users', USER_ID, 'medications'),
    { ...data, createdAt: serverTimestamp() }
  );
}

export async function updateMedication(medId, data) {
  await updateDoc(doc(db, 'users', USER_ID, 'medications', medId), data);
}

// ── 선호도 ───────────────────────────────────────────────────────────────────

export async function getPreferences() {
  const snap = await getDoc(doc(db, 'users', USER_ID, 'data', 'preferences'));
  return snap.exists() ? snap.data() : { favoriteFoods: [], allergies: [] };
}

export async function savePreferences(data) {
  await setDoc(
    doc(db, 'users', USER_ID, 'data', 'preferences'),
    data,
    { merge: true }
  );
}

// ── 요청 기록 ────────────────────────────────────────────────────────────────

export async function getHistory(count = 20) {
  const q = query(
    collection(db, 'users', USER_ID, 'history'),
    orderBy('timestamp', 'desc'),
    limit(count)
  );
  const snap = await getDocs(q);
  return snap.docs.map(d => ({ id: d.id, ...d.data() }));
}

export async function addHistory(data) {
  return await addDoc(
    collection(db, 'users', USER_ID, 'history'),
    { ...data, timestamp: serverTimestamp() }
  );
}

// ── RAG용 전체 사용자 데이터 조회 ────────────────────────────────────────────

export async function getUserContextForRAG() {
  const [profile, prefs, meds, health] = await Promise.all([
    getProfile(),
    getPreferences(),
    getMedications(),
    getHealthList(3),
  ]);
  return { profile, preferences: prefs, medications: meds, recentHealth: health };
}
