// 入口：选数据源（?mock → 模拟，否则连后端），组装界面，跑主循环
import { $$ } from './util.js';
import { Bus } from './util.js';
import { createStore } from './store.js';
import { createMockSource } from './source/mock.js';
import { createRosSource } from './source/ros.js';
import { createActions } from './actions.js';
import { createMapView } from './map_view.js';
import { createPanels } from './panels.js';
import { setupCamera } from './camera.js';
import { setupRunState } from './status.js';
import { setupAlerts } from './alerts.js';
import { setupFlow } from './flow.js';
import { setupReloc } from './reloc.js';
import { toast } from './ui.js';

const store = createStore(), bus = new Bus();
let flow = null;   // 右栏任务流程（setupFlow），setTool 里会用到
const mock = new URLSearchParams(location.search).has('mock');

// 面板要先订阅 log，数据源启动时打的日志才不会丢：先收着，面板建好后回放
const early = []; bus.on('log', (...a) => early && early.push(a));
bus.on('toast', (m, k) => toast(m, k));

const src = mock ? createMockSource(store, bus) : createRosSource(store, bus);
const actions = createActions(store, src);
const mapView = createMapView(store, actions);
const panels = createPanels(store, src, actions, mapView, bus, setTool);
const reloc = setupReloc(store, actions, mapView);
flow = setupFlow(store, src, actions, setTool, reloc);
const replay = early.splice(0); bus.h.log.shift(); replay.forEach(a => bus.emit('log', ...a));
bus.on('map', () => mapView.fit());
setupCamera(src, bus);
const alerts = setupAlerts(store, bus);                     // 服务异常：横幅 + 提示 + 日志
const runState = setupRunState(store, actions, mapView);   // 地图右上角的运行状态牌
// 调试用：地址加 ?debug 时可以在浏览器控制台里用 go2w.store / go2w.actions
if (new URLSearchParams(location.search).has('debug')) window.go2w = { store, src, actions, mapView };

/* ---------- 键盘 ---------- */
const keys = {};
// 遥控档位：前进 / 倒车 m/s（两者相同，速度桥给倒车加 0.11 补偿）、转向 rad/s、横移 m/s
const GEARS = [{ name: '慢', v: .3, w: .6, vy: .15 }, { name: '中', v: .6, w: .9, vy: .2 }, { name: '快', v: 1.0, w: 1.2, vy: .2 }];
let gear = 1;
try { const g = +localStorage.getItem('go2w.teleGear'); if (g >= 0 && g < GEARS.length && localStorage.getItem('go2w.teleGear') !== null) gear = g; } catch {}
function setGear(g){
  gear = Math.max(0, Math.min(GEARS.length - 1, g));
  try { localStorage.setItem('go2w.teleGear', gear); } catch {}
  $$('#teleGear button').forEach((b, i) => b.classList.toggle('on', i === gear));
  if (mapView.tool === 'teleop' && Object.values(keys).some(Boolean)) teleCmd();
}
$$('#teleGear button').forEach((b, i) => { b.onclick = () => setGear(i); });
setGear(gear);
function teleCmd(){
  const k = keys, G = GEARS[gear], v = (k.w ? G.v : 0) - (k.s ? G.v : 0), w = (k.a ? G.w : 0) - (k.d ? G.w : 0), vy = (k.q ? G.vy : 0) - (k.e ? G.vy : 0);
  if (src.can('teleop')) src.cmd.teleop(v || w || vy ? { v, w, vy } : null);
}
function enterTeleop(){
  if (!src.can('teleop')){ src.cmd.notice('teleop'); return false; }
  if (store.estop){ toast('急停中，无法遥控', 'err'); return false; }
  if (!actions.canTeleop()){ toast(!store.link.ok ? '没有连上后端' : store.profiles.navigation === 'run' ? '里程计不健康，不能遥控' : '速度桥还没起来（刚开机或刚切换档位时要等几秒）', 'err'); return false; }
  if (store.task.type === 'route' && store.task.status === 'active'){ toast('请先取消多点任务', 'err'); return false; }
  if (store.task.type !== 'idle') toast('已取消当前任务，进入键盘遥控', 'warn');
  // 进遥控 = 自动解锁（用户 11-10 要求；点遥控本身就是明确操作，不再弹确认框）。急停中不会走到这里
  if (!store.armed && src.can('arm')){
    Promise.resolve(src.cmd.arm(true)).then(r => {
      if (r && r.ok === false){ toast('自动解锁失败：' + (r.msg || '未知原因'), 'err'); if (mapView.tool === 'teleop') setTool('pan'); return; }
      store.armed = true;   // 先乐观更新，后端状态随后会覆盖
      toast('底盘已解锁，可以遥控', 'ok');
    });
  }
  return true;
}
function setTool(t){
  if (mapView.tool === 'teleop' && t !== 'teleop'){ for (const k in keys) keys[k] = false; $$('.keys kbd').forEach(x => x.classList.remove('dn')); if (src.can('teleop')) src.cmd.teleop(null); }
  if (t === 'route'){
    flow?.openRoute();
    if (document.body.classList.contains('right-off')) panels.toggleRight();
  }
  mapView.setTool(t, enterTeleop);
}
$$('[data-tool]').forEach(button => { button.onclick = () => setTool(button.dataset.tool); });
window.addEventListener('keydown', e => {
  if (e.target.closest('input,select,textarea')) return;
  const k = e.key.toLowerCase();
  if (e.code === 'Space'){ e.preventDefault(); actions.estop(); return; }
  if (mapView.tool === 'teleop' && 'wasdqe'.includes(k) && k.length === 1){
    if (!keys[k]){ keys[k] = true; $$(`.keys kbd[data-k="${k}"]`).forEach(x => x.classList.add('dn')); teleCmd(); }
    return;
  }
  if (mapView.tool === 'teleop' && (k === 'z' || k === 'c')) return setGear(gear + (k === 'c' ? 1 : -1));
  // 重定位：方向键平移、Q / E 旋转（不在遥控时）
  if (reloc.on && mapView.tool !== 'teleop'){
    const n = { arrowup: 'u', arrowdown: 'd', arrowleft: 'l', arrowright: 'r', q: 'ccw', e: 'cw' }[k];
    if (n){ e.preventDefault(); return reloc.nudge(n); }
  }
  if (k === 'escape') return setTool('pan');
  if (k === '[') return panels.toggleLeft();
  if (k === ']') return panels.toggleRight();
  const n = +k; if (n >= 1 && n <= mapView.TOOLS.length) setTool(mapView.TOOLS[n-1]);
  if (k === 't') setTool(mapView.tool === 'teleop' ? 'pan' : 'teleop');
  if (k === 'f') document.getElementById('vFollow').click();
});
window.addEventListener('keyup', e => { const k = e.key.toLowerCase(); if (keys[k]){ keys[k] = false; $$(`.keys kbd[data-k="${k}"]`).forEach(x => x.classList.remove('dn')); teleCmd(); } });
window.addEventListener('blur', () => { if (Object.values(keys).some(Boolean)){ for (const k in keys) keys[k] = false; $$('.keys kbd').forEach(x => x.classList.remove('dn')); teleCmd(); } });

/* ---------- 主循环 ---------- */
setTool('pan');
panels.fast(); panels.medium(); panels.slow(); alerts(); flow.render(); runState();
let last = performance.now(), a10 = 0, a5 = 0, a4 = 0, a1 = 0;
function frame(t){
  const dt = Math.min(.05, (t - last)/1000); last = t;
  src.tick(dt);
  a10 += dt; a5 += dt; a4 += dt; a1 += dt;
  if (a10 >= .1){ a10 = 0; panels.pushHist(); }
  if (a5 >= .2){ a5 = 0; panels.fast(); alerts(); flow.render(); runState(); }
  if (a4 >= .25){ a4 = 0; panels.medium(); }
  if (a1 >= 1){ a1 = 0; panels.slow(); }
  mapView.draw(dt);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
