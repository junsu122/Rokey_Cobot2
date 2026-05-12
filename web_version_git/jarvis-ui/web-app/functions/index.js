const { onRequest } = require("firebase-functions/v2/https");
const { defineString } = require("firebase-functions/params");

// 메모리 저장소
const offers  = {};
const answers = {};

const corsHeaders = {
  'Access-Control-Allow-Origin' : '*',
  'Access-Control-Allow-Headers': 'Content-Type, ngrok-skip-browser-warning',
  'Access-Control-Allow-Methods': 'GET, POST, DELETE, OPTIONS',
};

exports.signaling = onRequest({ cors: true, region: 'asia-northeast3' }, (req, res) => {
  // CORS preflight
  Object.entries(corsHeaders).forEach(([k, v]) => res.set(k, v));
  if (req.method === 'OPTIONS') return res.status(204).send('');

  const url = req.path;

  // POST /offer
  if (req.method === 'POST' && url === '/offer') {
    const { room, sdp, type } = req.body;
    offers[room] = { room, sdp, type };
    console.log(`[offer] room=${room}`);
    return res.json({ status: 'ok' });
  }

  // GET /offer/:room
  if (req.method === 'GET' && url.startsWith('/offer/')) {
    const room = url.split('/')[2];
    if (!offers[room]) return res.status(404).json({ status: 'waiting' });
    return res.json(offers[room]);
  }

  // POST /answer
  if (req.method === 'POST' && url === '/answer') {
    const { room, sdp, type } = req.body;
    answers[room] = { room, sdp, type };
    console.log(`[answer] room=${room}`);
    return res.json({ status: 'ok' });
  }

  // GET /answer/:room
  if (req.method === 'GET' && url.startsWith('/answer/')) {
    const room = url.split('/')[2];
    if (!answers[room]) return res.status(404).json({ status: 'waiting' });
    return res.json(answers[room]);
  }

  // DELETE /clear/:room
  if (req.method === 'DELETE' && url.startsWith('/clear/')) {
    const room = url.split('/')[2];
    delete offers[room];
    delete answers[room];
    return res.json({ status: 'ok' });
  }

  // GET /health
  if (req.method === 'GET' && url === '/health') {
    return res.json({ status: 'ok', service: 'JARVIS WebRTC Signaling' });
  }

  return res.status(404).json({ error: 'Not found' });
});
