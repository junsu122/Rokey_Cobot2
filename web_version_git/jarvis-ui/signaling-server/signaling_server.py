#signaling_server.py
#!/usr/bin/env python3
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import json

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

_offers  : dict = {}
_answers : dict = {}
_lock    = threading.Lock()

@app.after_request
def add_headers(response):
    response.headers['ngrok-skip-browser-warning'] = '1'
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Headers'] = '*'
    response.headers['Access-Control-Allow-Methods'] = '*'
    return response

@app.route('/offer', methods=['POST'])
def post_offer():
    data = request.json
    room = data.get('room', 'default')
    with _lock:
        _offers[room] = data
    return jsonify({'status': 'ok'})

@app.route('/offer/<room>', methods=['GET'])
def get_offer(room):
    with _lock:
        offer = _offers.get(room)
    if not offer:
        return jsonify({'status': 'waiting'}), 404
    return jsonify(offer)

@app.route('/answer', methods=['POST'])
def post_answer():
    data = request.json
    room = data.get('room', 'default')
    with _lock:
        _answers[room] = data
    return jsonify({'status': 'ok'})

@app.route('/answer/<room>', methods=['GET'])
def get_answer(room):
    with _lock:
        answer = _answers.get(room)
    if not answer:
        return jsonify({'status': 'waiting'}), 404
    return jsonify(answer)

@app.route('/clear/<room>', methods=['DELETE'])
def clear_room(room):
    with _lock:
        _offers.pop(room, None)
        _answers.pop(room, None)
    return jsonify({'status': 'ok'})

@app.route('/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok', 'service': 'JARVIS WebRTC Signaling'})

if __name__ == '__main__':
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    print("🌐 JARVIS WebRTC 시그널링 서버 시작")
    print("   http://0.0.0.0:5000")
    import os; app.run(host='0.0.0.0', port=int(os.environ.get('PORT', 5000)), debug=False)
