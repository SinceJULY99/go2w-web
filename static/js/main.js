// 入口：选数据源（?mock → 模拟，否则连后端），组装界面，跑主循环
import { $$ } from './util.js';
import { Bus } from './util.js';
import { createStore } from './store.js';
import { createMockSource } from './source/mock.js';
import { createRosSource } from './source/ros.js';
import { createActions } from './actions.js';
import { createMapView } from './map_view.js';
import { createPanels } from './panels.js';
import { toast } from './ui.js';

const store = createStore(), bus = new Bus();
const mock = new URLSearchParams(location.search).has('mock');

// 面板要先订阅 log，数据源启动时打的日志才不会丢：先收着，面板建好后回放
const early = []; bus.on('log', (...a) => early && early.push(a));
bus.on('toast', (m, k) => toast(m, k));

const src = mock ? createMockSource(store, bus) : createRosSource(store, bus);
const actions = createActions(store, src);
const mapView = createMapView(store, actions);
const panels = createPanels(store, src, actions, mapView, bus);
const replay = early.splice(0); bus.h.log.shift(); replay.forEach(a => bus.emit('log', ...a));
bus.on('map', () => mapView.fit());
// 调试用：地址加 ?debug 时可以在浏览器控制台里用 go2w.store / go2w.actions
if (new URLSearchParams(location.search).has('debug')) window.go2w = { store, src, actions, mapView };

/* ---------- 键盘 ---------- */
const keys = {};
function teleCmd(){
  const k = keys, v = (k.w ? .3 : 0) - (k.s ? .2 : 0), w = (k.a ? .8 : 0) - (k.d ? .8 : 0), vy = (k.q ? .15 : 0) - (k.e ? .15 : 0);
  if (src.can('teleop')) src.cmd.teleop(v || w || vy ? { v, w, vy } : null);
}
function enterTeleop(){
  if (!src.can('teleop')){ src.cmd.notice('teleop'); return false; }
  if (!actions.canMove()){ toast(store.estop ? '急停中，无法遥控' : !store.armed ? '底盘已锁定，先解锁再遥控' : '导航或里程计没有运行', 'err'); return false; }
  if (store.task.type !== 'idle') toast('已取消当前任务，进入键盘遥控', 'warn');
  return true;
}
function setTool(t){
  if (mapView.tool === 'teleop' && t !== 'teleop'){ for (const k in keys) keys[k] = false; $$('.keys kbd').forEach(x => x.classList.remove('dn')); if (src.can('teleop')) src.cmd.teleop(null); }
  mapView.setTool(t, enterTeleop);
}
document.getElementById('toolbar').onclick = e => { const b = e.target.closest('[data-tool]'); if (b) setTool(b.dataset.tool); };
window.addEventListener('keydown', e => {
  if (e.target.closest('input,select,textarea')) return;
  const k = e.key.toLowerCase();
  if (e.code === 'Space'){ e.preventDefault(); actions.estop(); return; }
  if (mapView.tool === 'teleop' && 'wasdqe'.includes(k) && k.length === 1){
    if (!keys[k]){ keys[k] = true; $$(`.keys kbd[data-k="${k}"]`).forEach(x => x.classList.add('dn')); teleCmd(); }
    return;
  }
  if (k === 'escape') return setTool('pan');
  if (k === '[') return panels.toggleLeft();
  if (k === ']') return panels.toggleRight();
  const n = +k; if (n >= 1 && n <= 6) setTool(mapView.TOOLS[n-1]);
  if (k === 't') setTool(mapView.tool === 'teleop' ? 'pan' : 'teleop');
  if (k === 'f') document.getElementById('vFollow').click();
});
window.addEventListener('keyup', e => { const k = e.key.toLowerCase(); if (keys[k]){ keys[k] = false; $$(`.keys kbd[data-k="${k}"]`).forEach(x => x.classList.remove('dn')); teleCmd(); } });
window.addEventListener('blur', () => { if (Object.values(keys).some(Boolean)){ for (const k in keys) keys[k] = false; $$('.keys kbd').forEach(x => x.classList.remove('dn')); teleCmd(); } });

/* ---------- 主循环 ---------- */
setTool('pan');
panels.fast(); panels.medium(); panels.slow();
let last = performance.now(), a10 = 0, a5 = 0, a4 = 0, a1 = 0;
function frame(t){
  const dt = Math.min(.05, (t - last)/1000); last = t;
  src.tick(dt);
  a10 += dt; a5 += dt; a4 += dt; a1 += dt;
  if (a10 >= .1){ a10 = 0; panels.pushHist(); }
  if (a5 >= .2){ a5 = 0; panels.fast(); }
  if (a4 >= .25){ a4 = 0; panels.medium(); }
  if (a1 >= 1){ a1 = 0; panels.slow(); }
  mapView.draw(dt);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
