// src/firebase/config.js
import { initializeApp } from 'firebase/app';
import { getFirestore } from 'firebase/firestore';
import { getAuth } from 'firebase/auth';

const firebaseConfig = {
  apiKey: "AIzaSyDHe2LXAZ-ivauB7w_k4ITy7WavpwuqPRk",
  authDomain: "jarvis-senior-robo.firebaseapp.com",
  projectId: "jarvis-senior-robo",
  storageBucket: "jarvis-senior-robo.firebasestorage.app",
  messagingSenderId: "188701557160",
  appId: "1:188701557160:web:93d5b12947b1b2460ddd9d",
  measurementId: "G-JKY84VD6C2"
};

const app  = initializeApp(firebaseConfig);
export const db   = getFirestore(app);
export const auth = getAuth(app);
export default app;
