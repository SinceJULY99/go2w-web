// 不开导航也能遥控：先运行 web/dev/run.sh，再 node web/dev/test_teleop_bridge.mjs [ws://127.0.0.1:8080/ws]
// 只能对本地假 ROS 运行
const URL_ = process.argv[2] || 'ws://127.0.0.1:8080/ws';
const sleep = ms => new Promise(r => setTimeout(r, ms));
const ws = new WebSocket(URL_); let state = null, seq = 0; const acks = new Map(), toasts = [];
ws.onmessage = e => { if (typeof e.data !== 'string') return; const m = JSON.parse(e.data);
  if (m.t === 'state') state = m; if (m.t === 'toast') toasts.push(m.msg); if (m.t === 'ack') { acks.get(m.id)?.(m); acks.delete(m.id); } };
const ping = setInterval(() => { if (ws.readyState === 1) ws.send('{"t":"ping","ts":0}'); }, 500);
const cmd = (op, a = {}) => new Promise(r => { const id = ++seq; acks.set(id, r); ws.send(JSON.stringify({ t: 'cmd', id, op, ...a })); });
const until = async (fn, ms, what) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { if (fn()) return; await sleep(100); } throw new Error('超时：' + what); };
const res = []; const ok = (n, c, x = '') => res.push(`${c ? 'PASS' : 'FAIL'}  ${n}${x ? '  · ' + x : ''}`);
const prof = k => state.profiles[k];
await new Promise(r => ws.onopen = r); await until(() => state, 6000, '首个状态');
try {
  await cmd('arm', { on: false });
  let r = await cmd('set_mode', { mode: null }); ok('停掉所有档位', r.ok, r.msg);
  await until(() => ['mapping', 'localization', 'navigation'].every(m => prof(m) === 'stop') && !state.busy, 15000, '档位全停');
  await until(() => prof('bridge') === 'run', 15000, '遥控速度桥起来'); ok('不开导航时遥控速度桥自动起来', true);
  r = await cmd('arm', { on: true }); ok('不开导航也能解锁', r.ok, r.msg);
  await until(() => state.armed === true, 3000, '解锁状态');
  for (let i = 0; i < 5; i++) { ws.send(JSON.stringify({ t: 'teleop', v: 0.3, vy: 0, w: 0 })); await sleep(100); }
  ok('遥控被接受（后端在发 /cmd_vel）', state.teleop === true && !toasts.some(t => t.includes('遥控没有生效')), toasts.join(' / '));
  await sleep(600); ok('松手后停止遥控', state.teleop === false);
  r = await cmd('set_mode', { mode: 'navigation' }); ok('启动导航：接受', r.ok, r.msg);
  await until(() => prof('navigation') === 'run' && !state.busy, 20000, '导航起来');
  await sleep(2500);
  ok('导航运行时遥控速度桥已停', prof('bridge') === 'stop');
  ok('启动导航时底盘被上锁', state.armed !== true);
  r = await cmd('set_mode', { mode: 'mapping' }); await until(() => prof('mapping') === 'run' && !state.busy, 20000, '建图起来');
  await until(() => prof('bridge') === 'run', 8000, '建图时遥控速度桥起来'); ok('建图时遥控速度桥也在（导航停了就拉起）', true);
  r = await cmd('set_mode', { mode: 'navigation' }); await until(() => prof('navigation') === 'run' && !state.busy, 20000, '导航恢复');
  await sleep(2500); ok('恢复导航后只有导航自带速度桥', prof('bridge') === 'stop');
} catch (e) { res.push('FAIL  ' + e.message); }
console.log(res.join('\n')); clearInterval(ping); ws.close();
process.exit(res.some(x => x.startsWith('FAIL')) ? 1 : 0);
