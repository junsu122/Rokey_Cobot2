// src/pages/ProfilePage.jsx
import { useState, useEffect } from 'react';
import { getProfile, saveProfile } from '../firebase/db';

export default function ProfilePage() {
  const [form, setForm]     = useState({
    name: '', age: '', phone: '', emergencyContact: '', emergencyPhone: '', notes: ''
  });
  const [saved, setSaved]   = useState(false);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    getProfile().then(data => {
      if (data) setForm(data);
      setLoading(false);
    });
  }, []);

  const handleChange = e =>
    setForm(prev => ({ ...prev, [e.target.name]: e.target.value }));

  const handleSave = async () => {
    await saveProfile(form);
    setSaved(true);
    setTimeout(() => setSaved(false), 2000);
  };

  if (loading) return <div className="text-muted">로딩 중...</div>;

  return (
    <div>
      <div className="card">
        <div className="card-title">👤 사용자 기본 정보</div>
        <div className="grid-2">
          <div className="form-group">
            <label className="form-label">이름</label>
            <input className="form-input" name="name"
              value={form.name} onChange={handleChange} placeholder="홍길동" />
          </div>
          <div className="form-group">
            <label className="form-label">나이</label>
            <input className="form-input" name="age" type="number"
              value={form.age} onChange={handleChange} placeholder="75" />
          </div>
          <div className="form-group">
            <label className="form-label">전화번호</label>
            <input className="form-input" name="phone"
              value={form.phone} onChange={handleChange} placeholder="010-0000-0000" />
          </div>
        </div>
      </div>

      <div className="card">
        <div className="card-title">🚨 비상 연락처</div>
        <div className="grid-2">
          <div className="form-group">
            <label className="form-label">이름</label>
            <input className="form-input" name="emergencyContact"
              value={form.emergencyContact} onChange={handleChange} placeholder="보호자 이름" />
          </div>
          <div className="form-group">
            <label className="form-label">전화번호</label>
            <input className="form-input" name="emergencyPhone"
              value={form.emergencyPhone} onChange={handleChange} placeholder="010-0000-0000" />
          </div>
        </div>
      </div>

      <div className="card">
        <div className="card-title">📝 특이사항</div>
        <div className="form-group">
          <textarea className="form-input" name="notes" rows={4}
            value={form.notes} onChange={handleChange}
            placeholder="건강 상태, 주의사항 등을 입력하세요" />
        </div>
      </div>

      <button className="btn btn-primary" onClick={handleSave}>
        💾 저장
      </button>

      {saved && (
        <div className="toast toast-success">✅ 저장됐어요!</div>
      )}
    </div>
  );
}
