// 提示条和确认框（页面里自己画，不用浏览器的 confirm）
import { $ } from './util.js';

let toastT;
export function toast(msg, k = 'ok'){
  const t = $('#toast'); t.dataset.k = k;
  t.querySelector('i').textContent = k === 'ok' ? '✓' : '!';
  t.querySelector('span').textContent = msg;
  t.classList.add('show'); clearTimeout(toastT);
  toastT = setTimeout(() => t.classList.remove('show'), 2600);
}

export function confirmBox({ title, body, ok = '确认', danger = false, onOk }){
  const s = document.createElement('div'); s.className = 'scrim';
  s.innerHTML = `<div class="modal" role="dialog" aria-modal="true"><h3></h3><p></p><div class="btn-row"><button class="btn ghost" data-a="c">取消</button><button class="btn ${danger ? 'danger' : 'pri'}" data-a="o"></button></div></div>`;
  s.querySelector('h3').textContent = title; s.querySelector('p').textContent = body; s.querySelector('[data-a=o]').textContent = ok;
  const close = () => { s.remove(); document.removeEventListener('keydown', esc); };
  const esc = e => { if (e.key === 'Escape') close(); };
  s.addEventListener('click', e => { const a = e.target.dataset.a; if (e.target === s || a === 'c') close(); if (a === 'o'){ close(); onOk && onOk(); } });
  document.addEventListener('keydown', esc);
  document.body.appendChild(s); s.querySelector('[data-a=o]').focus();
}

// 带表单的确认框：rows = [[名称, 值], …] 显示成清单；build(box) 往里加输入控件；valid(box) 为假时确认按钮置灰
export function formBox({ title, body = '', rows = [], warn = '', ok = '确认', danger = false, build, valid, onOk, onCancel }){
  const s = document.createElement('div'); s.className = 'scrim';
  s.innerHTML = `<div class="modal" role="dialog" aria-modal="true"><h3></h3><div class="modal-kv" hidden></div><div class="modal-warn" hidden></div><p></p><div class="modal-form"></div>
    <div class="btn-row"><button class="btn ghost" data-a="c">取消</button><button class="btn ${danger ? 'danger' : 'pri'}" data-a="o"></button></div></div>`;
  s.querySelector('h3').textContent = title; s.querySelector('p').textContent = body; s.querySelector('p').hidden = !body;
  s.querySelector('[data-a=o]').textContent = ok;
  const kv = s.querySelector('.modal-kv');
  if (rows.length){ kv.hidden = false; for (const [k, v] of rows){ const a = document.createElement('span'), b = document.createElement('b'); a.textContent = k; b.textContent = v; kv.append(a, b); } }
  const w = s.querySelector('.modal-warn'); if (warn){ w.hidden = false; w.textContent = warn; }
  const form = s.querySelector('.modal-form'), okBtn = s.querySelector('[data-a=o]');
  if (build) build(form);
  const sync = () => { okBtn.disabled = !!valid && !valid(form); };
  form.addEventListener('input', sync); form.addEventListener('change', sync); sync();
  let done = false;
  const close = ok_ => { if (done) return; done = true; s.remove(); document.removeEventListener('keydown', esc); if (!ok_ && onCancel) onCancel(); };
  const esc = e => { if (e.key === 'Escape') close(false); };
  s.addEventListener('click', e => { const a = e.target.dataset.a; if (e.target === s || a === 'c') close(false);
    if (a === 'o' && !okBtn.disabled){ close(true); onOk && onOk(form); } });
  document.addEventListener('keydown', esc);
  document.body.appendChild(s); okBtn.focus();
}
// formBox 里常用的两个控件
export function stillCheck(form){
  const l = document.createElement('label'); l.className = 'modal-chk';
  l.innerHTML = '<input type="checkbox" data-still><span>机器人已经站立、静止，周围没有人走动（FAST-LIO 初始化时机器人不能动）</span>';
  form.appendChild(l);
}
export function mapPicker(form, maps, value){
  const t = document.createElement('div'); t.className = 'modal-label'; t.textContent = '使用的地图';
  const sel = document.createElement('select'); sel.dataset.map = ''; sel.className = 'modal-select';
  for (const m of maps) sel.appendChild(new Option(m, m, false, m === value));
  if (!maps.length) sel.appendChild(new Option('data/maps 里没有地图', ''));
  form.append(t, sel);
}
