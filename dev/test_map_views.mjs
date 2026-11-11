// 地图显示方向协议测试：先运行 web/dev/run.sh，再运行 node web/dev/test_map_views.mjs [ws://127.0.0.1:8080/ws]
// 会切到建图档位存一张图，最后切回导航（默认地图）。只对本地假 ROS 运行
import assert from 'node:assert/strict';
import { setTimeout as sleep } from 'node:timers/promises';

const url = process.argv[2] || 'ws://127.0.0.1:8080/ws';
assert.match(url, /^ws:\/\/(127\.0\.0\.1|localhost):\d+\/ws$/);
const MAP = 'lab_20261021';

async function client(){
  const ws = new WebSocket(url), acks = new Map(), c = { ws, views: null, viewMsgs: 0, state: null };
  let id = 0;
  ws.addEventListener('message', ({ data }) => {
    if (typeof data !== 'string') return;
    const m = JSON.parse(data);
    if (m.t === 'ack') acks.get(m.id)?.(m);
    if (m.t === 'map_views'){ c.views = m.items; c.viewMsgs++; }
    if (m.t === 'state') c.state = m;
  });
  await new Promise((res, rej) => { ws.addEventListener('open', res, { once:true }); ws.addEventListener('error', rej, { once:true }); });
  const timer = setInterval(() => ws.readyState === 1 && ws.send('{"t":"ping","ts":0}'), 400);
  c.close = () => { clearInterval(timer); ws.close(); };
  c.cmd = (op, args = {}) => new Promise(r => { const k = ++id; acks.set(k, r); ws.send(JSON.stringify({ t:'cmd', id:k, op, ...args })); });
  return c;
}
async function until(fn, what, ms = 8000){
  const end = Date.now() + ms;
  while (!fn()){ assert.ok(Date.now() < end, `超时：${what}`); await sleep(50); }
}

const a = await client(), b = await client();
try {
  await until(() => a.views && b.views && a.state, '连上就收到方向表');
  const before = { ...a.views };

  let r = await a.cmd('map_view', { map:MAP, angle:0.5 });
  assert.equal(r.ok, true, r.msg);
  await until(() => b.views[MAP] === 0.5, '另一个网页收到新方向');
  console.log('PASS 设置方向后，其他网页同步收到');

  const c = await client();
  await until(() => c.views, 'c 收到方向表');
  assert.equal(c.views[MAP], 0.5);
  c.close();
  console.log('PASS 新打开的网页拿到已保存的方向');

  r = await a.cmd('map_view', { map:MAP, angle:0.5 + 2 * Math.PI });
  assert.equal(r.ok, true);
  await sleep(300);
  assert.ok(Math.abs(a.views[MAP] - 0.5) < 1e-6, `角度归一 ${a.views[MAP]}`);
  r = await a.cmd('map_view', { map:MAP, angle:-3 * Math.PI / 2 });
  await until(() => Math.abs(a.views[MAP] - Math.PI / 2) < 1e-6, `-270° 归成 90°（${a.views[MAP]}）`);
  console.log('PASS 角度归到 -180°～180°');

  for (const bad of [{ map:'../x', angle:1 }, { map:'a b', angle:1 }, { map:MAP, angle:'1' }, { map:MAP, angle:null }, { map:MAP, angle:true }, { angle:1 }]){
    r = await a.cmd('map_view', bad);
    assert.equal(r.ok, false, `应拒绝 ${JSON.stringify(bad)}`);
  }
  r = await a.cmd('map_view', { map:MAP, angle:Number.MAX_VALUE * 10 });
  assert.equal(r.ok, false, '拒绝无穷大');
  console.log('PASS 拒绝非法地图名和角度');

  // 建图时用 _mapping 暂存，存图时带给新地图
  const prof = k => a.state.profiles[k];
  r = await a.cmd('set_mode', { mode:'mapping' });
  assert.equal(r.ok, true, r.msg);
  await until(() => prof('mapping') === 'run' && !a.state.busy, '建图起来', 15000);
  r = await a.cmd('map_view', { map:'_mapping', angle:0.3 });
  assert.equal(r.ok, true, r.msg);
  const name = `view_test_${Date.now() % 100000}`;
  r = await a.cmd('save_map', { name });
  assert.equal(r.ok, true, r.msg);
  await until(() => Math.abs((a.views[name] ?? 0) - 0.3) < 1e-6, '新地图继承建图时的方向');
  console.log('PASS 存图时，新地图继承建图中调好的方向');
  // 再导出一张内容不同的地图（转过的副本：尺寸、原点都不一样），下面测切换
  r = await a.cmd('align_map', { source:name, name:name + '_al', angle:0.3 });
  assert.equal(r.ok, true, r.msg);

  // 设回 0：从表里删掉
  r = await a.cmd('map_view', { map:name, angle:0 });
  await until(() => !(name in a.views), '0° 从表里删除');
  await a.cmd('map_view', { map:'_mapping', angle:0 });
  for (const [k, v] of Object.entries(before)) await a.cmd('map_view', { map:k, angle:v });
  if (!(MAP in before)) await a.cmd('map_view', { map:MAP, angle:0 });
  await until(() => !(MAP in a.views) === !(MAP in before), '恢复原来的方向');
  console.log('PASS 设成 0° 后删除记录');

  r = await a.cmd('set_mode', { mode:'navigation', map:MAP });
  assert.equal(r.ok, true, r.msg);
  await until(() => prof('navigation') === 'run' && !a.state.busy && a.state.mapCurrent === MAP, '切回导航', 20000);

  // 「切换地图」用 map_server 的 load_map 服务，yaml_filename 参数不会变（真机如此，假 map_server 也照做）。
  // 当前地图名要跟着换，后端每 6 s 刷新一次时也不能被改回启动时的那张
  for (const target of [name + '_al', name, MAP]){
    r = await a.cmd('load_map', { name:target });
    assert.equal(r.ok, true, r.msg);
    await until(() => a.state.mapCurrent === target, `切换到 ${target}`);
    await sleep(7000);
    assert.equal(a.state.mapCurrent, target, `过了一次刷新后当前地图应仍是 ${target}`);
  }
  console.log('PASS 切换地图后当前地图名正确，不会被改回启动时的地图（含内容相同的地图）');
} finally {
  a.close(); b.close();
}
