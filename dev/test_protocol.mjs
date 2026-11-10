// P1 协议回归测试：先运行 dev/run.sh，再在 Mac 上运行 node dev/test_protocol.mjs [ws://127.0.0.1:8080/ws]
// 会解锁、导航、急停、断心跳，只能对本地假 ROS 运行，不要指向真机
// 测试目标点是按默认地图 data/maps/lab_20261021 选的空旷处（离墙和未知区域 0.6 m 以上），换地图会失败
const URL_ = process.argv[2] || 'ws://127.0.0.1:8080/ws';
const sleep = ms => new Promise(r => setTimeout(r, ms));
function client(){
  const ws = new WebSocket(URL_); ws.binaryType = 'arraybuffer';
  const c = { ws, state: null, task: undefined, pose: null, logs: [], acks: new Map(), seq: 0, ping: true };
  ws.onmessage = e => { if (typeof e.data !== 'string') return; const m = JSON.parse(e.data);
    if (m.t === 'state') c.state = m; if (m.t === 'task') c.task = m.task; if (m.t === 'pose') c.pose = m;
    if (m.t === 'log') c.logs.push(m.level + ' ' + m.msg); if (m.t === 'logs') for (const [lv, , msg] of m.items) c.logs.push(lv + ' ' + msg); if (m.t === 'ack') { c.acks.get(m.id)?.(m); c.acks.delete(m.id); } };
  c.timer = setInterval(() => { if (c.ping && ws.readyState === 1) ws.send(JSON.stringify({ t: 'ping', ts: 0 })); }, 500);
  c.cmd = (op, a = {}) => new Promise(r => { const id = ++c.seq; c.acks.set(id, r); ws.send(JSON.stringify({ t: 'cmd', id, op, ...a })); });
  c.open = new Promise(r => ws.onopen = r);
  c.until = async (fn, ms, what) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { if (fn()) return true; await sleep(100); } throw new Error('超时：' + what); };
  c.close = () => { clearInterval(c.timer); ws.close(); };
  return c;
}
const results = []; const ok = (name, cond, extra = '') => { results.push(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  · ' + extra : ''}`); };

const c = client(); await c.open; await c.until(() => c.state && c.pose, 5000, '首个状态');
ok('初始上锁', c.state.armed === false, `armed=${c.state.armed}`);
// 每次从同一个起点开始：机器人可能停在上次测试的终点
await c.cmd('arm', { on: false }); await c.cmd('release_estop');
await c.cmd('initial_pose', { x: 0.01, y: 0.0, th: 0 }); await sleep(800);

// 1. 正常导航
let r = await c.cmd('arm', { on: true }); ok('解锁', r.ok, r.msg);
await c.until(() => c.state.armed === true, 3000, '解锁生效');
r = await c.cmd('goal', { x: 3.06, y: -2.71, th: 1.57 }); ok('发目标', r.ok, r.msg);
const phases = new Set(); let rem0 = null, remMin = 1e9;
const t0 = Date.now();
// 只看这个单点目标：刚连上时可能还收到上一条路线任务结束后保留的结果
while (Date.now() - t0 < 40000) { const t = c.task; if (t?.type === 'goal') { phases.add(t.phase); if (t.remaining != null) { rem0 ??= t.remaining; remMin = Math.min(remMin, t.remaining); } if (t.status !== 'active') break; } await sleep(100); }
ok('到达目标', c.task?.status === 'succeeded', `status=${c.task?.status} 用时 ${((Date.now()-t0)/1000).toFixed(1)} s`);
ok('看到规划→跟随→到达阶段', phases.has('plan') && phases.has('follow') && phases.has('arrived'), [...phases].join(','));
ok('剩余距离递减', rem0 > 1 && remMin < 0.3, `从 ${rem0?.toFixed(2)} 到 ${remMin.toFixed(2)} m`);
const p = c.pose; ok('位置在目标附近', Math.hypot(p.x - 3.06, p.y + 2.71) < 0.25, `(${p.x.toFixed(2)}, ${p.y.toFixed(2)})`);

// 2. 急停
r = await c.cmd('goal', { x: -1.44, y: 6.54, th: 0 }); ok('发第二个目标', r.ok, r.msg);
await c.until(() => c.task?.phase === 'follow', 5000, '开始跟随');
await sleep(1500);
const before = { ...c.pose };
r = await c.cmd('estop'); ok('急停', r.ok, r.msg);
await c.until(() => c.state.estop && c.state.armed === false, 3000, '急停状态');
ok('急停后上锁', c.state.armed === false);
await c.until(() => c.task?.status !== 'active', 3000, '任务结束');
ok('急停后导航取消', c.task?.status === 'canceled', c.task?.status);
await sleep(1500); const moved = Math.hypot(c.pose.x - before.x, c.pose.y - before.y);
ok('急停后停住', Math.hypot(c.pose.v) < 0.02, `急停后 1.5 s 速度 ${c.pose.v}`);
r = await c.cmd('arm', { on: true }); ok('急停中拒绝解锁', !r.ok, r.msg);
r = await c.cmd('goal', { x: 0, y: 0, th: 0 }); ok('急停中拒绝目标', !r.ok, r.msg);
r = await c.cmd('release_estop'); ok('解除急停', r.ok, r.msg);
await c.until(() => !c.state.estop, 2000, '解除');
ok('解除后仍上锁', c.state.armed === false);

// 3. 心跳丢失
r = await c.cmd('arm', { on: true }); ok('重新解锁', r.ok, r.msg);
r = await c.cmd('goal', { x: 0.0, y: 0.0, th: 0 }); ok('发回原点的目标', r.ok, r.msg);
await c.until(() => c.task?.phase === 'follow' || c.task?.phase === 'plan', 4000, '任务开始');
await sleep(1000);
c.ping = false; const tLost = Date.now();
await c.until(() => c.state.armed === false, 4000, '心跳丢失后上锁');
const lockDelay = (Date.now() - tLost) / 1000;
ok('心跳丢失后自动上锁', lockDelay < 2.2, `停发心跳后 ${lockDelay.toFixed(1)} s 上锁（含 state 2 Hz 推送延迟）`);
await c.until(() => c.task?.status !== 'active', 3000, '心跳丢失后任务结束');
ok('心跳丢失后导航取消', c.task?.status === 'canceled', c.task?.status);
ok('日志里有心跳丢失', c.logs.some(l => l.includes('心跳丢失')), c.logs.filter(l => l.includes('心跳')).pop() || '');
c.ping = true; await sleep(1500);
ok('恢复心跳后保持上锁', c.state.armed === false);

// 4. 调参
r = await c.cmd('set_param', { key: 'vmax', value: 0.3 }); ok('设置 max_vx', r.ok, r.msg);
await c.until(() => c.state.params?.vmax === 0.3, 7000, '参数回读');
ok('参数回读为 0.3', c.state.params.vmax === 0.3);
r = await c.cmd('set_param', { key: 'vmax', value: -1 }); ok('拒绝负数参数', !r.ok, r.msg);
await c.cmd('set_param', { key: 'vmax', value: 1.0 });

// 5. 初始位姿
r = await c.cmd('initial_pose', { x: 3.06, y: -2.71, th: 0 }); ok('初始位姿', r.ok, r.msg);
await sleep(600); ok('位姿跳到初始位姿', Math.hypot(c.pose.x - 3.06, c.pose.y + 2.71) < 0.05, `(${c.pose.x.toFixed(2)}, ${c.pose.y.toFixed(2)})`);

// 6. 控制方断开
r = await c.cmd('arm', { on: true }); ok('解锁（准备断开测试）', r.ok, r.msg);
const watcher = client(); await watcher.open; await watcher.until(() => watcher.state?.armed === true, 3000, '旁观端看到解锁');
c.close();
await watcher.until(() => watcher.state?.armed === false, 3000, '控制方断开后上锁');
ok('控制方网页关闭后自动上锁', watcher.state.armed === false);
watcher.close();

console.log(results.join('\n'));
console.log(`\n${results.filter(x => x.startsWith('PASS')).length}/${results.length} 通过`);
process.exit(0);
