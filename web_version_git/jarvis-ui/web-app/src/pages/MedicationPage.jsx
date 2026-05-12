// src/pages/MedicationPage.jsx
import { useState, useEffect } from 'react';
import { getMedications, addMedication, updateMedication } from '../firebase/db';

const TIMES = ['아침', '점심', '저녁', '취침 전'];

export default function MedicationPage() {
  const [meds, setMeds]     = useState([]);
  const [form, setForm]     = useState({ name: '', time: '아침', dose: '', notes: '' });
  const [saved, setSaved]   = useState(false);

  useEffect(() => { loadMeds(); }, []);

  const loadMeds = async () => {
    const data = await getMedications();
    setMeds(data);
  };

  const handleChange = e =>
    setForm(prev => ({ ...prev, [e.target.name]: e.target.value }));

  const handleAdd = async () => {
    if (!form.name) return;
    await addMedication({ ...form, taken: false });
    setForm({ name: '', time: '아침', dose: '', notes: '' });
    setSaved(true);
    setTimeout(() => setSaved(false), 2000);
    loadMeds();
  };

  const toggleTaken = async (med) => {
    await updateMedication(med.id, { taken: !med.taken });
    loadMeds();
  };

  const grouped = TIMES.reduce((acc, t) => {
    acc[t] = meds.filter(m => m.time === t);
    return acc;
  }, {});

  return (
    <div>
      <div className="card">
        <div className="card-title">💊 복약 추가</div>
        <div className="grid-2">
          <div className="form-group">
            <label className="form-label">약 이름</label>
            <input className="form-input" name="name"
              value={form.name} onChange={handleChange} placeholder="혈압약" />
          </div>
          <div className="form-group">
            <label className="form-label">복용 시간</label>
            <select className="form-input" name="time"
              value={form.time} onChange={handleChange}>
              {TIMES.map(t => <option key={t} value={t}>{t}</option>)}
            </select>
          </div>
          <div className="form-group">
            <label className="form-label">용량</label>
            <input className="form-input" name="dose"
              value={form.dose} onChange={handleChange} placeholder="1정" />
          </div>
          <div className="form-group">
            <label className="form-label">메모</label>
            <input className="form-input" name="notes"
              value={form.notes} onChange={handleChange} placeholder="식후 30분" />
          </div>
        </div>
        <button className="btn btn-primary" onClick={handleAdd}>+ 추가</button>
      </div>

      {TIMES.map(time => (
        grouped[time].length > 0 && (
          <div className="card" key={time}>
            <div className="card-title">⏰ {time}</div>
            {grouped[time].map(med => (
              <div key={med.id} className="flex-between"
                style={{ padding: '12px 0', borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                <div>
                  <div style={{ fontWeight: 600 }}>{med.name}</div>
                  <div className="text-muted" style={{ fontSize: 13 }}>
                    {med.dose} {med.notes && `· ${med.notes}`}
                  </div>
                </div>
                <button
                  className={`btn ${med.taken ? 'btn-ghost' : 'btn-primary'}`}
                  onClick={() => toggleTaken(med)}
                  style={{ minWidth: 80 }}>
                  {med.taken ? '✅ 완료' : '복용'}
                </button>
              </div>
            ))}
          </div>
        )
      ))}

      {saved && <div className="toast toast-success">✅ 저장됐어요!</div>}
    </div>
  );
}
