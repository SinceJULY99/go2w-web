// P3 协议回归测试：先运行 web/dev/run.sh，再在 Mac 上运行 node web/dev/test_p3.mjs [ws://127.0.0.1:8080/ws]
// 会切换档位、巡逻、录包、存图、遥控，只能对本地假 ROS 运行，不要指向真机
// 测试点是按默认地图 data/maps/lab_20261021 选的空旷处
const URL_ = process.argv[2] || 'ws://127.0.0.1:8080/ws';
const sleep = ms => new Promise(r => setTimeout(r, ms));
function client(){
  const ws = new WebSocket(URL_); ws.binaryType = 'arraybuffer';
  const c = { ws, state: null, task: undefined, pose: null, logs: [], toasts: [], msgs: [], acks: new Map(), seq: 0 };
  ws.onmessage = e => { if (typeof e.data !== 'string') return; const m = JSON.parse(e.data);
    if (m.t === 'state') c.state = m; if (m.t === 'task') c.task = m.task; if (m.t === 'pose') c.pose = m;
    if (m.t === 'logs') for (const [lv, node, msg] of m.items) c.logs.push(`${lv} ${node} ${msg}`);
    if (m.t === 'toast') c.toasts.push(m.msg); if (m.t === 'msg') c.msgs.push(m);
    if (m.t === 'ack') { c.acks.get(m.id)?.(m); c.acks.delete(m.id); } };
  c.timer = setInterval(() => { if (ws.readyState === 1) ws.send('{"t":"ping","ts":0}'); }, 500);
  c.cmd = (op, a = {}) => new Promise(r => { const id = ++c.seq; c.acks.set(id, r); ws.send(JSON.stringify({ t: 'cmd', id, op, ...a })); });
  c.send = o => ws.send(JSON.stringify(o));
  c.open = new Promise(r => ws.onopen = r);
  c.until = async (fn, ms, what) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { if (fn()) return true; await sleep(100); } throw new Error('超时：' + what); };
  return c;
}
const results = []; const ok = (name, cond, extra = '') => { results.push(`${cond ? 'PASS' : 'FAIL'}  ${name}${extra ? '  · ' + extra : ''}`); };
const c = client(); await c.open; await c.until(() => c.state && c.pose, 6000, '首个状态');
const prof = k => c.state.profiles[k];
const stamp = Date.now() % 100000;
const A = { x: 0.01, y: 0, th: 0 }, B = { x: 0.61, y: -3.96, th: 1.57 };

async function ensureNav(){
  if (prof('navigation') !== 'run'){ await c.cmd('set_mode', { mode: 'navigation' }); await c.until(() => prof('navigation') === 'run' && !c.state.busy, 12000, '导航起来'); }
  await sleep(600);
}
try {
  await c.cmd('stop_patrol'); await ensureNav(); await c.cmd('arm', { on: false }); await c.cmd('initial_pose', A); await sleep(600);

  // 1. 档位切换
  await c.cmd('arm', { on: true }); await c.cmd('goal', { x: -6.34, y: -4.8, th: 0 });
  await c.until(() => c.state.armed === true && c.task?.status === 'active', 3000, '解锁并开始导航');
  let r = await c.cmd('set_mode', { mode: 'mapping' }); ok('导航 → 建图：接受', r.ok, r.msg);
  r = await c.cmd('set_mode', { mode: 'localization' }); ok('切换中拒绝再次切换', !r.ok, r.msg);
  await c.until(() => prof('navigation') === 'stopping', 2000, '显示停止中'); ok('显示「停止中」', true);
  await c.until(() => c.state.armed !== true, 3000, '切换时上锁'); ok('切换时底盘上锁', true);
  await c.until(() => prof('mapping') === 'starting', 4000, '显示启动中'); ok('显示「启动中」', true);
  await c.until(() => prof('mapping') === 'run', 8000, '建图起来'); ok('建图运行中', prof('navigation') === 'stop');
  ok('建图时没有 cmd_bridge', c.state.armed === null, `armed=${c.state.armed}`);
  r = await c.cmd('arm', { on: true }); ok('建图时拒绝解锁', !r.ok, r.msg);
  r = await c.cmd('goal', { ...B }); ok('建图时拒绝目标', !r.ok, r.msg);
  ok('切换时任务已结束', !c.task || c.task.status !== 'active', c.task?.status);
  r = await c.cmd('set_mode', { mode: 'bogus' }); ok('拒绝未知档位', !r.ok, r.msg);

  // 2. 存图
  const mapName = `t${stamp}`;
  for (const bad of ['a;touch /tmp/x', '../evil', 'a b', '']) { r = await c.cmd('save_map', { name: bad }); ok(`存图拒绝非法名字「${bad}」`, !r.ok, r.msg); }
  r = await c.cmd('save_map', { name: mapName }); ok('建图中存图', r.ok, r.msg);
  r = await c.cmd('save_map', { name: 'lab_20261021' }); ok('不覆盖同名地图', !r.ok, r.msg);

  // 3. 全部停止 → 选图 → 启动
  r = await c.cmd('set_mode', { mode: null }); ok('全部停止', r.ok, r.msg);
  await c.until(() => ['mapping', 'localization', 'navigation'].every(m => prof(m) === 'stop') && !c.state.busy, 6000, '都停了');
  await c.until(() => c.pose && c.pose.valid === false, 3000, '没有 TF'); ok('停止后没有位姿', true);
  await c.until(() => c.state.nodes.every(n => !n.up), 3000, '节点都没了'); ok('停止后节点都下线', true);
  await c.until(() => c.state.maps.some(m => m.name === mapName), 8000, '新地图进列表'); ok('新地图出现在列表', true);
  r = await c.cmd('load_map', { name: '../../etc/passwd' }); ok('拒绝列表外的地图', !r.ok, r.msg);
  r = await c.cmd('load_map', { name: mapName }); ok('没在跑时选图', r.ok && c.state !== null, r.msg);
  await sleep(700); ok('记下启动用的地图', c.state.mapChoice === mapName, c.state.mapChoice);
  r = await c.cmd('set_mode', { mode: 'navigation' }); ok('启动导航', r.ok, r.msg);
  await c.until(() => prof('navigation') === 'run', 10000, '导航起来');
  await c.until(() => c.state.mapCurrent === mapName, 8000, 'map_server 用选好的地图'); ok('用选好的地图启动', true, c.state.mapCurrent);
  r = await c.cmd('load_map', { name: 'lab_20261021' }); ok('运行中切换地图', r.ok, r.msg);
  await c.until(() => c.state.mapCurrent === 'lab_20261021', 8000, '切回来'); ok('切换生效', true);
  await c.cmd('initial_pose', A); await sleep(600);

  // 4. 录包
  for (const [name, topics, why] of [['a;b', ['/odom'], '名字'], ['ok', ['/odom;reboot'], 'topic'], ['ok', [], '空 topic']]) {
    r = await c.cmd('bag_start', { name, topics }); ok(`录包拒绝非法${why}`, !r.ok, r.msg);
  }
  const bag = `b${stamp}`;
  r = await c.cmd('bag_start', { name: bag, topics: ['/odom', '/scan'] }); ok('开始录包', r.ok, r.msg);
  await c.until(() => prof('bag') === 'run', 4000, '录包运行中'); ok('录包运行中', true);
  r = await c.cmd('bag_start', { topics: ['/odom'] }); ok('拒绝重复录包', !r.ok, r.msg);
  await sleep(2000);
  r = await c.cmd('bag_stop'); ok('停止录包', r.ok, r.msg);
  await c.until(() => prof('bag') === 'stop', 4000, '录包停止');
  await c.until(() => c.state.bags.some(b => b.name === bag), 8000, '包进列表');
  const b0 = c.state.bags.find(b => b.name === bag); ok('包出现在列表、时长正确', b0.dur > 1.5 && b0.dur < 6, `${b0.dur?.toFixed(1)} s, ${b0.size} MB`);

  // 5. topic 查看
  c.send({ t: 'inspect', topic: '/odom', on: true });
  await c.until(() => c.msgs.some(m => m.topic === '/odom'), 3000, '收到 /odom 内容');
  const m0 = c.msgs.find(m => m.topic === '/odom'); ok('查看 /odom 有内容', /pose/.test(m0.text) && /twist/.test(m0.text), m0.text.split('\n')[0]);
  await sleep(2000); const n2 = c.msgs.filter(m => m.topic === '/odom').length; ok('刷新不超过 2 Hz', n2 <= 6, `约 2 s 收到 ${n2} 条`);
  c.send({ t: 'inspect', topic: '/odom', on: false }); await sleep(300); const before = c.msgs.length; await sleep(1500);
  ok('关闭后不再收到', c.msgs.length === before);
  c.send({ t: 'inspect', topic: '/no_such_topic', on: true }); await c.until(() => c.msgs.some(m => m.topic === '/no_such_topic'), 2000, '错误回复');
  ok('不存在的 topic 给出说明', !!c.msgs.find(m => m.topic === '/no_such_topic').err);

  // 6. 键盘遥控
  await c.cmd('arm', { on: false }); await sleep(700);
  c.send({ t: 'teleop', v: 0.3, vy: 0, w: 0 }); await sleep(800);
  ok('上锁时遥控不动，并提示', Math.abs(c.pose.v) < 0.02 && c.toasts.length > 0, c.toasts.at(-1));
  await c.cmd('arm', { on: true }); await c.cmd('goal', { x: -6.34, y: -4.8, th: 0 });
  await c.until(() => c.task?.status === 'active', 3000, '导航开始');
  await sleep(600);
  const hold = setInterval(() => c.send({ t: 'teleop', v: 0.3, vy: 0, w: 0 }), 100);
  await c.until(() => !c.task || c.task.status !== 'active', 4000, '遥控取消导航'); ok('遥控开始时取消导航', true, c.task?.status);
  await sleep(1500); ok('遥控时机器人在动', Math.abs(c.pose.v - 0.3) < 0.08, `v=${c.pose.v}`);
  clearInterval(hold); const tRel = Date.now();
  await c.until(() => Math.abs(c.pose.v) < 0.03, 2000, '松手后停车'); ok('松手后停车', true, `${((Date.now()-tRel)/1000).toFixed(1)} s`);
  const fast = setInterval(() => c.send({ t: 'teleop', v: 5, vy: 0, w: 0 }), 100); await sleep(1500); clearInterval(fast);
  ok('遥控速度被限幅', c.pose.v <= 0.55, `发 5 m/s，实际 ${c.pose.v}`);
  await c.until(() => Math.abs(c.pose.v) < 0.03, 2000, '再次停车');
  await c.cmd('initial_pose', A); await sleep(600);

  // 7. 巡逻
  await c.cmd('arm', { on: false }); await sleep(500);
  await c.until(() => c.state.patrol, 1000, 'patrol 状态');
  r = await c.cmd('patrol_point', { k: 'C', x: 0, y: 0, th: 0 }); ok('拒绝未知巡逻点', !r.ok, r.msg);
  r = await c.cmd('patrol_point', { k: 'A', ...A }); ok('设 A 点', r.ok, r.msg);
  r = await c.cmd('patrol_point', { k: 'B', ...B }); ok('设 B 点', r.ok, r.msg);
  r = await c.cmd('start_patrol', { loops: 1 }); ok('上锁时也能启动，提示等解锁', r.ok && /解锁/.test(r.msg), r.msg);
  await c.until(() => prof('patrol') === 'run', 4000, 'patrol 运行');
  await sleep(2500); ok('上锁时 patrol 不出发', Math.abs(c.pose.v) < 0.02 && !c.task);
  r = await c.cmd('patrol_point', { k: 'A', ...B }); ok('巡逻中拒绝改点', !r.ok, r.msg);
  r = await c.cmd('start_patrol', { loops: 1 }); ok('拒绝重复启动巡逻', !r.ok, r.msg);
  r = await c.cmd('goal', { ...B }); ok('巡逻中拒绝单点目标', !r.ok, r.msg);
  await c.cmd('arm', { on: true });
  await c.until(() => c.task?.type === 'patrol' && c.task.status === 'active', 6000, '开始巡逻'); ok('任务卡片显示巡逻', true, `去 ${c.task.leg}`);
  await c.until(() => c.task?.type === 'patrol' && c.task.legs >= 1, 20000, '完成第一段');
  await c.until(() => c.task?.leg === 'B' && c.task.remaining > 1, 15000, '去 B 且有进度'); ok('外部目标也有进度', true, `剩 ${c.task.remaining.toFixed(1)} m`);
  await c.until(() => c.task?.legs >= 2, 40000, '到 B');
  await c.until(() => prof('patrol') === 'stop', 45000, '巡逻完成退出'); ok('往返 1 次后结束（A→B→A）', c.logs.some(l => l.includes('巡逻完成')));
  ok('结束后任务空闲', !c.task, JSON.stringify(c.task));

  // 巡逻中取消：不能被 patrol.py 重试带着又走起来
  await c.cmd('start_patrol', { loops: 0 });
  await c.until(() => c.task?.type === 'patrol' && c.task.legs >= 0 && c.task.remaining != null, 15000, '巡逻走起来');
  r = await c.cmd('cancel'); ok('巡逻中取消', r.ok, r.msg);
  await c.until(() => prof('patrol') === 'stop', 4000, '巡逻被停'); await sleep(4500);
  ok('取消后不会重试', Math.abs(c.pose.v) < 0.02 && !c.task && prof('patrol') === 'stop', `v=${c.pose.v}`);

  // 巡逻中上锁 → patrol 自己结束
  await c.cmd('start_patrol', { loops: 0 });
  await c.until(() => c.task?.type === 'patrol', 12000, '巡逻开始');
  await c.cmd('arm', { on: false });
  await c.until(() => prof('patrol') === 'stop', 5000, '上锁后巡逻结束'); ok('上锁后巡逻结束', true);

  await c.cmd('initial_pose', A);
} catch (e) { results.push('FAIL  ' + e.message); }
console.log(results.join('\n'));
console.log(`\n${results.filter(x => x.startsWith('PASS')).length}/${results.length} 通过`);
process.exit(0);
