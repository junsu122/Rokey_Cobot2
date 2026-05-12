// src/pages/PreferencePage.jsx
import { useState, useEffect } from 'react';
import { getPreferences, savePreferences } from '../firebase/db';

const FOOD_SUGGESTIONS   = ['사과', '바나나', '빵', '사탕', '음료수', '물'];
const ALLERGY_SUGGESTIONS = ['견과류', '유제품', '밀가루', '달걀', '갑각류'];

export default function PreferencePage() {
  const [prefs, setPrefs]   = useState({ favoriteFoods: [], allergies: [] });
  const [foodInput, setFoodInput]     = useState('');
  const [allergyInput, setAllergyInput] = useState('');
  const [saved, setSaved]   = useState(false);

  useEffect(() => {
    getPreferences().then(data => {
      if (data) setPrefs(data);
    });
  }, []);

  const addFood = (food) => {
    const f = food || foodInput.trim();
    if (!f || prefs.favoriteFoods.includes(f)) return;
    setPrefs(prev => ({ ...prev, favoriteFoods: [...prev.favoriteFoods, f] }));
    setFoodInput('');
  };

  const removeFood = (food) =>
    setPrefs(prev => ({ ...prev, favoriteFoods: prev.favoriteFoods.filter(f => f !== food) }));

  const addAllergy = (allergy) => {
    const a = allergy || allergyInput.trim();
    if (!a || prefs.allergies.includes(a)) return;
    setPrefs(prev => ({ ...prev, allergies: [...prev.allergies, a] }));
    setAllergyInput('');
  };

  const removeAllergy = (allergy) =>
    setPrefs(prev => ({ ...prev, allergies: prev.allergies.filter(a => a !== allergy) }));

  const handleSave = async () => {
    await savePreferences(prefs);
    setSaved(true);
    setTimeout(() => setSaved(false), 2000);
  };

  return (
    <div>
      <div className="card">
        <div className="card-title">🍎 선호 음식</div>
        <div style={{ marginBottom: 12 }}>
          {prefs.favoriteFoods.map(f => (
            <span key={f} className="tag">
              {f}
              <button className="tag-remove" onClick={() => removeFood(f)}>×</button>
            </span>
          ))}
        </div>
        <div className="flex gap-8" style={{ marginBottom: 12 }}>
          <input className="form-input" value={foodInput}
            onChange={e => setFoodInput(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && addFood()}
            placeholder="음식 입력 후 Enter"
            style={{ flex: 1 }} />
          <button className="btn btn-primary" onClick={() => addFood()}>추가</button>
        </div>
        <div>
          {FOOD_SUGGESTIONS.filter(f => !prefs.favoriteFoods.includes(f)).map(f => (
            <button key={f} className="btn btn-ghost"
              style={{ margin: '4px', fontSize: 12 }}
              onClick={() => addFood(f)}>{f}</button>
          ))}
        </div>
      </div>

      <div className="card">
        <div className="card-title">⚠️ 알레르기</div>
        <div style={{ marginBottom: 12 }}>
          {prefs.allergies.map(a => (
            <span key={a} className="tag" style={{ borderColor: 'rgba(255,68,85,0.3)', color: '#ff4455' }}>
              {a}
              <button className="tag-remove" onClick={() => removeAllergy(a)}>×</button>
            </span>
          ))}
        </div>
        <div className="flex gap-8" style={{ marginBottom: 12 }}>
          <input className="form-input" value={allergyInput}
            onChange={e => setAllergyInput(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && addAllergy()}
            placeholder="알레르기 입력 후 Enter"
            style={{ flex: 1 }} />
          <button className="btn btn-primary" onClick={() => addAllergy()}>추가</button>
        </div>
        <div>
          {ALLERGY_SUGGESTIONS.filter(a => !prefs.allergies.includes(a)).map(a => (
            <button key={a} className="btn btn-ghost"
              style={{ margin: '4px', fontSize: 12 }}
              onClick={() => addAllergy(a)}>{a}</button>
          ))}
        </div>
      </div>

      <button className="btn btn-primary" onClick={handleSave}>💾 저장</button>
      {saved && <div className="toast toast-success">✅ 저장됐어요!</div>}
    </div>
  );
}
