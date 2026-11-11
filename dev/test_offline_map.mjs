// 导航没开时也能看地图、编辑任务（11-11）：后端直接读 data/maps 的文件显示；出发时带上速度上限。
// 只能对 web/dev/run.sh 的假 ROS 服务运行：node dev/test_offline_map.mjs [ws://127.0.0.1:8080/ws]
// 会在数据副本（GO2W_DATA，默认 ~/.cache/go2w_web_data）里放一张小地图 offline_test。
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { setTimeout as sleep } from 'node:timers/promises';

const url = process.argv[2] || 'ws://127.0.0.1:8080/ws';
assert.match(url, /^ws:\/\/(127\.0\.0\.1|localhost):\d+\/ws$/);
const DATA = process.env.GO2W_DATA || path.join(os.homedir(), '.cache', 'go2w_web_data');
const MAP = 'lab_20261021', OTHER = 'offline_test';
const points = [{ x: .01, y: 0, th: 0, name: '入口', manualHeading: false }, { x: .11, y: -.70, th: -Math.PI / 2, name: '走廊', manualHeading: true }];

// 40×30 的小图：四周是墙，左下角一格未知（像素 205）
function writeOtherMap(){
  const w = 40, h = 30, px = Buffer.alloc(w * h, 254);
  for (let x = 0; x < w; x++){ px[x] = 0; px[(h - 1) * w + x] = 0; }
  for (let y = 0; y < h; y++){ px[y * w] = 0; px[y * w + w - 1] = 0; }
  px[(h - 2) * w + 1] = 205;                               // 图片倒数第 2 行 = 地图 y 方向第 1 行
  fs.writeFileSync(path.join(DATA, 'maps', `${OTHER}.pgm`), Buffer.concat([Buffer.from(`P5\n${w} ${h}\n255\n`), px]));
  fs.writeFileSync(path.join(DATA, 'maps', `${OTHER}.yaml`),
    `image: ${OTHER}.pgm\nmode: trinary\nresolution: 0.1\norigin: [-1.0, -2.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n`);
}

async function client(){
  const ws = new WebSocket(url), replies = new Map();
  const c = { ws, maps: [] };
  let id = 0;
  ws.binaryType = 'arraybuffer';
  ws.addEventListener('message', async ({ data }) => {
    if (typeof data !== 'string'){
      const dv = new DataView(data);
      if (dv.getUint8(0) !== 1) return;
      const jlen = dv.getUint32(1, true);
      const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(data, 5, jlen)));
      const grid = new Uint8Array(await new Response(new Blob([new Uint8Array(data, 5 + jlen)]).stream()
        .pipeThrough(new DecompressionStream('deflate'))).arrayBuffer());
      c.maps.push({ ...meta, grid });
      return;
    }
    const m = JSON.parse(data);
    if (m.t === 'ack') replies.set(m.id, m);
    if (m.t === 'state') c.state = m;
    if (m.t === 'routes') c.routes = m.items;
    if (m.t === 'pose') c.pose = m;
  });
  await new Promise((resolve, reject) => { ws.addEventListener('open', resolve, { once: true }); ws.addEventListener('error', reject, { once: true }); });
  const timer = setInterval(() => { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ t: 'ping', ts: 0 })); }, 350);
  c.close = () => { clearInterval(timer); ws.close(); };
  c.cmd = async (op, args = {}, timeout = 5000) => {
    const key = ++id;
    ws.send(JSON.stringify({ t: 'cmd', id: key, op, ...args }));
    await until(() => replies.has(key), op, timeout);
    const r = replies.get(key); replies.delete(key); return r;
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
const ok = async (c, op, args, timeout) => { const r = await c.cmd(op, args, timeout); assert.equal(r.ok, true, `${op}：${r.msg}`); return r; };
const stopped = s => ['mapping', 'localization', 'navigation'].every(m => s?.profiles[m] === 'stop');

writeOtherMap();
const c = await client();
let routeId = null;
try {
  // 1. 全部停掉：仍然显示上次用的地图（文件），mapLive = false
  await until(() => c.state, 'state');
  await ok(c, 'set_mode', { mode: null });
  await until(() => stopped(c.state) && !c.state.busy, '全部停止', 20000);
  c.maps.length = 0;
  await until(() => c.state.mapLive === false && c.state.mapCurrent === MAP && c.maps.length, '导航停了以后显示文件地图', 8000);
  assert.equal(c.maps.at(-1).w, 308);
  assert.equal(c.maps.at(-1).h, 435);
  console.log('PASS 导航停了以后照样显示地图（直接读文件），mapLive = false');

  // 2. 离线编辑任务：能保存；路径检查说明要等自检
  const saved = await ok(c, 'route_save', { map: MAP, name: '离线任务', points });
  routeId = saved.data.id;
  const check = await c.cmd('route_check', { map: MAP, points });
  assert.equal(check.ok, false);
  assert.match(check.msg, /导航没有运行/);
  assert.equal((await c.cmd('route_save', { map: OTHER, name: '错图', points })).ok, false, '不是显示中的地图不能保存');
  console.log('PASS 导航没开时可以保存任务；路径检查提示出发前自检时再查');

  // 3. 离线换图：立刻显示新图（尺寸、原点、未知格），记进 last_map.json
  await until(() => c.state.maps.some(m => m.name === OTHER), '地图列表里有 offline_test', 10000);
  c.maps.length = 0;
  await ok(c, 'load_map', { name: OTHER });
  await until(() => c.state.mapCurrent === OTHER && c.maps.length, '显示 offline_test', 5000);
  const g = c.maps.at(-1);
  assert.deepEqual([g.w, g.h, g.res, g.ox, g.oy], [40, 30, .1, -1, -2]);
  assert.equal(g.grid[0], 100, '左下角是墙');
  assert.equal(g.grid[1 * 40 + 1], 255, '未知格保持未知（free_thresh 0.196）');
  assert.equal(g.grid[5 * 40 + 5], 0, '里面是空地');
  assert.equal(c.state.mapLive, false);
  assert.equal(JSON.parse(fs.readFileSync(path.join(DATA, 'last_map.json'), 'utf8')).name, OTHER);
  console.log('PASS 导航没开时换图立刻显示，栅格和 map_server 规则一致，记住上次的图');
  await ok(c, 'load_map', { name: MAP });
  await until(() => c.state.mapCurrent === MAP, '换回 lab', 5000);

  // 4. 自检启动导航后变成 map_server 的图；出发时带速度上限
  await ok(c, 'set_mode', { mode: 'navigation', map: MAP });
  await until(() => c.state.profiles.navigation === 'run' && c.state.mapLive && c.state.mapCurrent === MAP && c.pose?.frame === 'map', '导航起来', 30000);
  await ok(c, 'initial_pose', { x: .5, y: -.3, th: 0 });
  assert.equal((await c.cmd('route_start', { route_id: routeId, arm: true, vmax: 5 })).ok, false, '速度上限超范围要拒绝');
  assert.equal(c.state.armed, false, '拒绝时没有解锁');
  await ok(c, 'route_start', { route_id: routeId, arm: true, vmax: .33, wmax: .9 });
  await until(() => Math.abs((c.state.params?.vmax ?? 0) - .33) < 1e-6 && Math.abs((c.state.params?.wmax ?? 0) - .9) < 1e-6, '速度上限生效');
  await ok(c, 'cancel');
  await ok(c, 'arm', { on: false });
  console.log('PASS 出发时把网页选的速度上限设给 cmd_bridge，超范围拒绝且不解锁');
} finally {
  if (routeId) await c.cmd('route_delete', { route_id: routeId }).catch(() => {});
  if (c.state?.profiles.navigation !== 'run') await c.cmd('set_mode', { mode: 'navigation', map: MAP }).catch(() => {});
  for (const ext of ['yaml', 'pgm']) fs.rmSync(path.join(DATA, 'maps', `${OTHER}.${ext}`), { force: true });
  c.close();
}
