// 只能对 web/dev/run.sh 的假 ROS 服务运行。
import assert from 'node:assert/strict';
import { setTimeout as sleep } from 'node:timers/promises';

const url = process.argv[2] || 'ws://127.0.0.1:8080/ws';
assert.match(url, /^ws:\/\/(127\.0\.0\.1|localhost):\d+\/ws$/);
const map = 'lab_20261021', first = { x: .01, y: 0, th: 0 };
const points = [
  { ...first, name:'入口', manualHeading:false },
  { x:.11, y:-.70, th:-Math.PI / 2, name:'走廊', manualHeading:true },
  { x:.21, y:-1.35, th:-Math.PI / 2, name:'工位', manualHeading:true },
];

async function client(){
  const ws = new WebSocket(url), replies = new Map(), tasks = [];
  const c = { ws, tasks, ping: true };
  let id = 0;
  ws.binaryType = 'arraybuffer';
  ws.addEventListener('message', ({ data }) => {
    if (typeof data !== 'string') return;
    const m = JSON.parse(data);
    if (m.t === 'ack') replies.set(m.id, m);
    if (m.t === 'state') c.state = m;
    if (m.t === 'routes') c.routes = m.items;
    if (m.t === 'task'){
      c.task = m.task;
      if (m.task?.type === 'route') tasks.push(m.task);
    }
    if (m.t === 'pose') c.pose = m;
  });
  await new Promise((resolve, reject) => {
    ws.addEventListener('open', resolve, { once: true });
    ws.addEventListener('error', reject, { once: true });
  });
  const ping = () => {
    if (c.ping && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ t:'ping', ts:0 }));
  };
  ping();
  const timer = setInterval(ping, 350);
  c.close = () => { clearInterval(timer); ws.close(); };
  c.cmd = async (op, args = {}) => {
    const key = ++id;
    ws.send(JSON.stringify({ t:'cmd', id:key, op, ...args }));
    await until(() => replies.has(key), op);
    const reply = replies.get(key);
    replies.delete(key);
    return reply;
  };
  return c;
}

async function until(check, label, timeout = 5000){
  const end = Date.now() + timeout;
  while (!check()){
    assert.ok(Date.now() < end, `超时：${label}`);
    await sleep(30);
  }
}

const c = await client(), observer = await client();
try {
  await until(() => c.state?.mapCurrent === map && c.pose?.frame === 'map', '地图和定位');
  async function command(op, args){
    const result = await c.cmd(op, args);
    assert.equal(result.ok, true, result.msg);
    return result.data;
  }
  async function save(name, pts){
    return command('route_save', { map, name, points: pts });
  }
  const checked = await command('route_check', { map, points });
  assert.equal(checked.simulated, true);
  assert.deepEqual(checked.paths.map(p => p.to), [2, 3]);
  assert.deepEqual(checked.paths[0].points[0], [points[0].x, points[0].y]);
  assert.deepEqual(checked.paths[1].points.at(-1), [points[2].x, points[2].y]);
  assert.equal(c.state.armed, false, '检查无需解锁');
  const route = await save('三点测试', points);
  assert.equal((await c.cmd('route_save', { map, name:'非法', points:[first] })).ok, false);
  assert.equal((await c.cmd('route_save', { map:'错误地图', name:'非法', points })).ok, false);
  assert.equal((await c.cmd('route_start', { route_id:route.id })).ok, false, '上锁时拒绝执行');
  const edited = await command('route_save', { route_id:route.id, map, name:'三点任务', points });
  assert.equal(edited.id, route.id);
  assert.deepEqual(edited.points, points, '保存并更新后仍保留站名和朝向');
  assert.equal((await c.cmd('route_save', {
    map, name:'非法站名', points:[{ ...first, name:'长'.repeat(41) }, points[1]],
  })).ok, false);
  await until(() => observer.routes?.some(r => r.id === route.id && r.name === '三点任务'), '另一客户端收到更新');
  await command('initial_pose', first);
  await command('arm', { on:true });
  await command('route_start', { route_id:route.id });
  await until(() => c.task?.name === '三点任务' && c.task.status === 'succeeded', '到达三个点', 25000);
  assert.deepEqual([...new Set(c.tasks.filter(t => t.name === '三点任务').map(t => t.point))], [1, 2, 3]);
  assert.ok(Math.hypot(c.pose.x - points[2].x, c.pose.y - points[2].y) < .3);
  assert.deepEqual([...new Set(c.tasks.filter(t => t.name === '三点任务').map(t => t.pointName))], ['入口', '走廊', '工位']);
  await command('route_delete', { route_id:route.id });
  await until(() => observer.routes && !observer.routes.some(r => r.id === route.id), '另一客户端收到删除');
  assert.equal((await c.cmd('route_start', { route_id:route.id })).ok, false);
  console.log('PASS 站名和自定义朝向保存、执行进度显示站名、任务删除同步');
  console.log('PASS 站间路径、保存、更新、多客户端同步、按顺序到达三点');

  const fail = await save('失败时不跳点', [points[2], { x: 100, y: 100, th: 0 }, first]);
  const failedCheck = await c.cmd('route_check', { map, points:fail.points });
  assert.equal(failedCheck.ok, false);
  assert.equal(failedCheck.data.failed, 2);
  await command('route_start', { route_id:fail.id });
  await until(() => c.task?.name === fail.name && c.task.status === 'aborted', '第二点失败');
  assert.equal(c.task.point, 2);
  assert.equal(c.tasks.some(t => t.name === fail.name && t.point === 3), false);
  assert.equal(c.tasks.some(t => t.name === fail.name && t.phase === 'follow'), false);
  assert.match(c.task.message, /规划检查失败/);
  console.log('PASS 第二站不可达，出发前拒绝整条任务');

  async function safetyCase(name, stop, expectLock){
    await command('initial_pose', first);
    await command('arm', { on:true });
    const r = await save(name, [{ x:.61, y:-3.96, th:0 }, first, points[2]]);
    await command('route_start', { route_id:r.id });
    await until(() => observer.task?.name === r.name && observer.task.status === 'active' &&
      observer.task.phase === 'follow', `${name}开始跟随`);
    assert.equal((await observer.cmd('goal', first)).ok, false, '执行中拒绝其他网页抢发目标');
    assert.equal((await observer.cmd('route_delete', { route_id:r.id })).ok, false, '运行中拒绝删除当前任务');
    await stop();
    await until(() => observer.task?.name === r.name && observer.task.status === 'canceled', `${name}后取消任务`);
    if (expectLock) await until(() => observer.state.armed === false, `${name}后上锁`);
    await sleep(500);
    assert.equal(observer.task.point, 1);
    assert.ok(Math.abs(observer.pose.v) < .05);
  }
  await safetyCase('手动取消', () => command('cancel'), false);
  await safetyCase('急停', () => command('estop'), true);
  await command('release_estop');
  await safetyCase('心跳丢失', () => { c.ping = false; }, true);
  c.ping = true;
  await sleep(500);
  assert.equal(observer.state.armed, false);
  await safetyCase('网页关闭', () => c.close(), true);
  console.log('PASS 取消、急停、心跳丢失、关闭网页均不会派发下一点');
} finally {
  c.close();
  observer.close();
}
