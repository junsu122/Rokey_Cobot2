// src/pages/HealthPage.jsx
import { useState, useEffect } from 'react';
import { saveHealth, getHealthList } from '../firebase/db';

export default function HealthPage() {
  const [form, setForm]     = useState({ bloodSugar: '', bloodPressure: '', weight: '' });
  const [list, setList]     = useState([]);
  const [saved, setSaved]   = useState(false);

  useEffect(() => { loadList(); }, []);

  const loadList = async () => {
    const data = await getHealthList(7);
    setList(data);
  };

  const handleChange = e =>
    setForm(prev => ({ ...prev, [e.target.name]: e.target.value }));

  const handleSave = async () => {
    await saveHealth(form);
    setSaved(true);
    setTimeout(() => setSaved(false), 2000);
    loadList();
    setForm({ bloodSugar: '', bloodPressure: '', weight: '' });
  };

  const bloodSugarStatus = (val) => {
    const v = Number(val);
    if (!v) return null;
    if (v < 70)  return { label: '저혈당', cls: 'badge-danger' };
    if (v <= 140) return { label: '정상',   cls: 'badge-success' };
    return           { label: '고혈당', cls: 'badge-danger' };
  };

  return (
    <div>
      <div className="card">
        <div className="card-title">📊 오늘 건강 데이터 입력</div>
        <div className="grid-3">
          <div className="form-group">
            <label className="form-label">혈당 (mg/dL)</label>
            <input className="form-input mono" name="bloodSugar" type="number"
              value={form.bloodSugar} onChange={handleChange} placeholder="120" />
            {form.bloodSugar && (() => {
              const s = bloodSugarStatus(form.bloodSugar);
              return s ? <span className={`badge ${s.cls} mt-8`}>{s.label}</span> : null;
            })()}
          </div>
          <div className="form-group">
            <label className="form-label">혈압 (mmHg)</label>
            <input className="form-input mono" name="bloodPressure"
              value={form.bloodPressure} onChange={handleChange} placeholder="120/80" />
          </div>
          <div className="form-group">
            <label className="form-label">체중 (kg)</label>
            <input className="form-input mono" name="weight" type="number"
              value={form.weight} onChange={handleChange} placeholder="65" />
          </div>
        </div>
        <button className="btn btn-primary" onClick={handleSave}>💾 오늘 데이터 저장</button>
      </div>

      <div className="card">
        <div className="card-title">📈 최근 7일 기록</div>
        {list.length === 0 ? (
          <div className="text-muted">기록이 없어요.</div>
        ) : (
          <table className="data-table">
            <thead>
              <tr>
                <th>날짜</th>
                <th>혈당</th>
                <th>혈압</th>
                <th>체중</th>
                <th>상태</th>
              </tr>
            </thead>
            <tbody>
              {list.map(item => {
                const s = bloodSugarStatus(item.bloodSugar);
                return (
                  <tr key={item.id}>
                    <td className="mono text-muted">{item.id}</td>
                    <td className="mono">{item.bloodSugar || '-'}</td>
                    <td className="mono">{item.bloodPressure || '-'}</td>
                    <td className="mono">{item.weight || '-'}</td>
                    <td>{s ? <span className={`badge ${s.cls}`}>{s.label}</span> : '-'}</td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        )}
      </div>

      {saved && <div className="toast toast-success">✅ 저장됐어요!</div>}
    </div>
  );
}
