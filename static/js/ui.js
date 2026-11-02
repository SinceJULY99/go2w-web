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
