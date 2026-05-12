// src/pages/HistoryPage.jsx
import { useState, useEffect } from 'react';
import { getHistory } from '../firebase/db';

const INTENT_LABELS = {
  bring_object  : { label: '물건 전달',  cls: 'badge-info' },
  going_out     : { label: '외출 준비',  cls: 'badge-warning' },
  take_medicine : { label: '복약 보조',  cls: 'badge-success' },
  emergency     : { label: '긴급 상황',  cls: 'badge-danger' },
  cancel        : { label: '취소',       cls: 'badge-danger' },
  weather_query : { label: '날씨 조회',  cls: 'badge-info' },
  general_query : { label: '일반 질문',  cls: 'badge-info' },
  unknown       : { label: '알 수 없음', cls: 'badge-warning' },
};

export default function HistoryPage() {
  const [history, setHistory] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    getHistory(20).then(data => {
      setHistory(data);
      setLoading(false);
    });
  }, []);

  if (loading) return <div className="text-muted">로딩 중...</div>;

  return (
    <div>
      <div className="card">
        <div className="card-title">📋 최근 요청 기록</div>
        {history.length === 0 ? (
          <div className="text-muted">아직 요청 기록이 없어요.</div>
        ) : (
          <table className="data-table">
            <thead>
              <tr>
                <th>시각</th>
                <th>음성 입력</th>
                <th>Intent</th>
                <th>대상</th>
                <th>결과</th>
              </tr>
            </thead>
            <tbody>
              {history.map(item => {
                const badge = INTENT_LABELS[item.intent] || INTENT_LABELS.unknown;
                const ts = item.timestamp?.toDate?.()?.toLocaleString('ko-KR') || '-';
                return (
                  <tr key={item.id}>
                    <td className="mono text-muted" style={{ fontSize: 12 }}>{ts}</td>
                    <td>"{item.voiceText || '-'}"</td>
                    <td><span className={`badge ${badge.cls}`}>{badge.label}</span></td>
                    <td className="text-accent">{item.targetObject || '-'}</td>
                    <td>
                      {item.success !== undefined && (
                        <span className={`badge ${item.success ? 'badge-success' : 'badge-danger'}`}>
                          {item.success ? '성공' : '실패'}
                        </span>
                      )}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        )}
      </div>
    </div>
  );
}
