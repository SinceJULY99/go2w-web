// ROS 数据源：连 go2w_web 后端的 /ws。
// P0：地图、位姿、scan、健康状态；P1：初始位姿、导航目标、取消、解锁 / 上锁、急停、调参、心跳
// P2：代价地图、全局路径、局部轨迹（按 /cmd_vel 外推）、点云、AMCL 粒子、/rosout 日志
import { gridToCanvas, costToCanvas, wallMask, CLOUD } from '../util.js';

const T_MAP = 1, T_SCAN = 2, T_GCOST = 3, T_LCOST = 4, T_CLOUD = 5, T_PART = 6, T_CAM = 7;
// 已接入真机的操作；其余操作给出说明，不发任何指令
const OPS = new Set(['initialPose', 'autoLocalize', 'cancel', 'arm', 'estop', 'releaseEstop', 'setParam', 'simulateDrop',
  'setMode', 'toggle', 'standUp', 'lieDown', 'standHold', 'prepHold', 'saveRoute', 'startRoute', 'checkRoute', 'deleteRoute', 'saveMap', 'alignMap', 'loadMap', 'teleop', 'setMapView', 'deleteBag', 'uploadBag', 'setRollback', 'saveRollback']);
const NOT_YET = {
  pause: '真机暂不支持暂停：可以取消任务，或者锁定底盘',
};
const LATER = '真机上暂时还不能用';

async function inflate(bytes){
  const ds = new DecompressionStream('deflate');
  const buf = await new Response(new Blob([bytes]).stream().pipeThrough(ds)).arrayBuffer();
  return new Uint8Array(buf);
}

export function createRosSource(store, bus){
  const log = (lv, node, msg) => bus.emit('log', lv, node, msg);
  const t0 = performance.now(), now = () => (performance.now() - t0) / 1000;
  const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
  let ws = null, lastMsg = 0, mapGrid = null, rateWarned = -Infinity, seq = 0, pingPausedUntil = 0, lastTask = null;
  let camWanted = false;                            // 摄像头标签页开着：断线重连后要重新申请
  let layers = [], gcost = null, rawPlan = null;   // rawPlan：后端最近一次的 /plan（任务结束后不再显示）
  let teleCmd = null, teleTimer = null;             // 键盘遥控：按住时每 0.1 s 发一次，后端 0.3 s 收不到就停
  const watchers = new Map();                       // topic 查看器：topic → Set(回调)
  const sendWatches = () => { for (const topic of watchers.keys()) ws.send(JSON.stringify({ t: 'inspect', topic, on: true })); };
  const sendLayers = () => { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: 'sub', layers })); };
  const pending = new Map();

  store.source = 'ros';
  store.link = { ok: false, ms: null };
  store.robot.valid = false;
  store.armed = null;

  // 心跳放在 Worker 里：页面切到后台时主线程计时器会被降到 1 Hz 甚至更低，会误触发「心跳丢失」
  const beat = new Worker(URL.createObjectURL(new Blob(['setInterval(() => postMessage(0), 500)'], { type: 'text/javascript' })));
  beat.onmessage = () => {
    if (ws && ws.readyState === 1 && performance.now() >= pingPausedUntil) ws.send(JSON.stringify({ t: 'ping', ts: performance.now() }));
  };

  function connect(){
    ws = new WebSocket(url);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => { store.link.ok = true; lastMsg = performance.now(); log('INFO', 'go2w_web', `connected to ${url}`); sendLayers(); sendWatches(); if (camWanted) ws.send(JSON.stringify({ t: 'cam', on: true })); };
    ws.onmessage = e => { lastMsg = performance.now(); typeof e.data === 'string' ? onJson(JSON.parse(e.data)) : onBinary(e.data); };
    ws.onclose = () => {
      if (store.link.ok) log('WARN', 'go2w_web', 'connection lost, retrying');
      store.link = { ok: false, ms: null };
      store.robot.valid = false; store.odomOk = false; store.armed = null;
      for (const [, res] of pending) res({ ok: false, msg: '和后端的连接断开了' });
      pending.clear();
      if (camWanted) bus.emit('cam', { state: 'disconnected' });
      setTimeout(connect, 1000);
    };
  }

  function send(op, args = {}, timeout = 6000){
    return new Promise(resolve => {
      if (!ws || ws.readyState !== 1) return resolve({ ok: false, msg: '没有连上后端' });
      const id = ++seq;
      pending.set(id, resolve);
      ws.send(JSON.stringify({ t: 'cmd', id, op, ...args }));
      setTimeout(() => { if (pending.delete(id)) resolve({ ok: false, msg: '后端没有回复（超时）' }); }, timeout);
    });
  }

  function onTask(t){
    if (!t){ store.task = { type: 'idle' }; store.goal = null; lastTask = null; return; }
    // 结果只提示一次（后端会把结束的任务保留 5 s）
    if (lastTask && lastTask.status === 'active' && t.status !== 'active'){
      if (t.type === 'route') bus.emit('toast', t.message, t.status === 'aborted' ? 'err' : t.status === 'canceled' ? 'warn' : 'ok');
      else if (t.status === 'succeeded' && t.goal) bus.emit('toast', `已到达目标 (${t.goal.x.toFixed(2)}, ${t.goal.y.toFixed(2)})`);
      else if (t.status === 'aborted') bus.emit('toast', '导航失败：Nav2 放弃了这个目标，看日志了解原因', 'err');
    }
    lastTask = t;
    store.task = { ...t, paused: false };
    store.goal = ['goal', 'route'].includes(t.type) && t.status === 'active' && t.phase !== 'checking' ? t.goal : null;
  }

  function onJson(m){
    switch (m.t){
      case 'pong': store.link.ms = performance.now() - m.ts; break;
      case 'cam': bus.emit('cam', m); break;
      case 'ack': { const res = pending.get(m.id); if (res){ pending.delete(m.id); res({ ok: m.ok, msg: m.msg, data: m.data }); } break; }
      case 'log': log(m.level, m.node, m.msg); break;
      case 'logs': for (const [lv, node, msg] of m.items) log(lv, node, msg); break;
      case 'plan': rawPlan = m.pts.length > 1 ? m.pts : null; break;
      case 'toast': bus.emit('toast', m.msg, m.level || 'warn'); break;
      case 'msg': for (const cb of watchers.get(m.topic) || []) cb(m); break;
      case 'routes': store.routes = m.items; break;
      case 'map_views': store.mapViews = m.items || {}; break;
      case 'task': onTask(m.task); break;
      case 'pose': {
        const R = store.robot;
        if (!m.valid){ R.valid = false; break; }
        const { speed, tilt, cmd, ...pose } = m;
        Object.assign(R, pose); R.valid = true;
        if (cmd) store.cmd = { v: cmd[0], vy: cmd[1], w: cmd[2] };
        store.speedCheck = speed || null;
        store.imu = tilt ? { roll: tilt[0], pitch: tilt[1] } : { roll: null, pitch: null };
        if (speed && Math.abs(speed.wall[0]) > .1 && Math.abs(speed.rate - 1) > .05 && performance.now() - rateWarned > 10000){
          rateWarned = performance.now();
          log('WARN', 'go2w_web', `/odom 时间戳走速是真实时间的 ${speed.rate.toFixed(2)} 倍：twist ${m.v.toFixed(2)} m/s，按真实时间 ${speed.wall[0].toFixed(2)} m/s`);
        }
        store.fixed = m.frame;
        const tr = store.trail, last = tr[tr.length-1];
        if (!last || Math.hypot(last[0]-R.x, last[1]-R.y) > .08){ tr.push([R.x, R.y]); if (tr.length > 900) tr.shift(); }
        break;
      }
      case 'state': {
        store.mapBusy = !!m.mapBusy;
        if (store.mapCurrent && m.mapCurrent && store.mapCurrent !== m.mapCurrent){
          store.goal = null;
          store.routeDraft = { id: null, map: null, points: [], confirmed: false };
          store.routeCheck = null;
        }
        store.odomOk = m.odomOk; store.armed = m.armed; store.sys = m.sys; store.nodes = m.nodes; store.estop = !!m.estop;
        Object.assign(store.params, m.params || {});
        for (const [k, st] of Object.entries(m.profiles)){   // stop / starting / run / stopping
          if (st === 'run' && store.profiles[k] !== 'run') store.since[k] = now();
          store.profiles[k] = st;
        }
        store.loc.running = m.profiles.localization === 'run' || m.profiles.navigation === 'run';
        // 地图：data/maps 列表，网页上显示的那张（导航没开时是后端直接读的文件，mapLive = false），没在跑时选好的「下次启动用」
        const cur = m.mapCurrent, choice = m.mapChoice;
        store.maps = (m.maps || []).map(x => ({ name: x.name, meta: `${x.w ?? '?'}×${x.h ?? '?'} · ${x.res} m`, cur: x.name === cur, choice: x.name === choice }));
        store.mapCurrent = cur; store.mapChoice = choice; store.mapLive = !!m.mapLive;
        store.bags = (m.bags || []).map(b => ({ name: b.name, dur: b.dur ?? 0, size: b.size }));
        store.uploads = m.uploads || {};
        store.bagLive = m.bagLive || []; store.bagSegment = m.bagSegment || null;
        if (m.rollback) store.rollback = m.rollback;
        store.procsKnown = m.procsKnown; store.busy = m.busy;
        Object.assign(store.loc, { std: null, yaw: null, match: null }, m.loc || {});
        store.autoLoc = m.autoLoc || null;
        // 运控：{ on, state: Passive/FixStand/Velocity/null(收不到), ready, seq, op: up/down, msg, stood, auto, hold, lieIn }
        store.ctrl = m.ctrl || { on: false };   // 自动定位：{ state: idle/run/ok/fail, seq, map, msg, x, y, th, score, ratio }
        store.topics = m.topics.map(t => ({ name: t.name, type: t.type, d0: false, alive: t.pubs > 0, hz: t.hz }));
        break;
      }
    }
  }

  async function onBinary(buf){
    const dv = new DataView(buf), kind = dv.getUint8(0), jlen = dv.getUint32(1, true);
    const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 5, jlen)));
    const payload = new Uint8Array(buf, 5 + jlen);
    // 摄像头帧必须按到达顺序同步交给播放器（下面的分支有 await，会打乱顺序）
    if (kind === T_CAM){ bus.emit('camFrame', meta, payload); return; }
    if (kind === T_SCAN){
      store.scan = new Float32Array(payload.slice().buffer);
    } else if (kind === T_GCOST || kind === T_LCOST){
      const grid = await inflate(payload);
      const cm = { img: costToCanvas(grid, meta.w, meta.h), w: meta.w*meta.res, h: meta.h*meta.res, ox: meta.ox, oy: meta.oy, yaw: meta.oyaw || 0 };
      if (kind === T_LCOST) store.lcostmap = cm;
      else { store.costmap = cm; gcost = { grid, ...meta }; }
    } else if (kind === T_CLOUD){
      // 负载：n 个 float32 (x, y)，后面跟 n 个 uint8 高度档（0..5，对应 0.3 → 1.6 m）
      const n = meta.n, xy = new Float32Array(payload.slice(0, n*8).buffer), band = payload.subarray(n*8, n*9);
      const cloud = CLOUD.map(() => []);
      for (let i=0;i<n;i++) cloud[band[i]].push(xy[2*i], xy[2*i+1]);
      store.cloud = cloud;
    } else if (kind === T_PART){
      store.particles = new Float32Array(payload.slice().buffer);
    } else if (kind === T_MAP){
      const grid = await inflate(payload);
      mapGrid = { grid, ...meta };
      store.wallAt = wallMask(grid, meta.w, meta.h, meta.res, meta.ox, meta.oy, meta.oyaw || 0);
      store.map = { img: gridToCanvas(grid, meta.w, meta.h), w: meta.w*meta.res, h: meta.h*meta.res,
        ox: meta.ox, oy: meta.oy, yaw: meta.oyaw || 0, res: meta.res, name: '/map', cells: `${meta.w}×${meta.h}` };
      // 光标读数和目标校验：有全局代价地图就用它（换算到 costmap_2d 的 0..254），否则用地图占用值粗略判断
      store.costAt = (x, y) => {
        if (gcost && Math.abs(gcost.oyaw || 0) < 1e-3){
          const g = gcost, gx = Math.floor((x - g.ox)/g.res), gy = Math.floor((y - g.oy)/g.res);
          if (gx >= 0 && gy >= 0 && gx < g.w && gy < g.h){
            const v = g.grid[gy*g.w + gx];
            return v === 255 ? 254 : v >= 100 ? 254 : v >= 99 ? 253 : Math.round(v * 2.52);
          }
        }
        const g = mapGrid, cosine = Math.cos(g.oyaw || 0), sine = Math.sin(g.oyaw || 0);
        const dx = x-g.ox, dy = y-g.oy;
        const gx = Math.floor((cosine*dx+sine*dy)/g.res), gy = Math.floor((-sine*dx+cosine*dy)/g.res);
        if (gx < 0 || gy < 0 || gx >= g.w || gy >= g.h) return 254;
        const v = g.grid[gy*g.w + gx]; return v === 255 ? 254 : v >= 65 ? 254 : 0;
      };
      log('INFO', 'map', `received ${meta.w}×${meta.h} @ ${+meta.res.toFixed(3)} m`);
      bus.emit('map');
    }
  }

  const real = {
    initialPose: (p, fine) => send('initial_pose', fine ? { ...p, cov: 'fine' } : p),
    autoLocalize: () => send('auto_localize', {}, 95000),   // 后端先等站稳最多 15 s，子进程再最多算 60 s
    cancel: () => send('cancel'),
    arm: on => send('arm', { on }),
    estop: () => send('estop'),
    releaseEstop: () => send('release_estop'),
    setParam: (key, value) => send('set_param', { key, value }),
    // P3：运行档位、巡逻、录包、地图
    setMode: (mode, map) => send('set_mode', map ? { mode, map } : { mode }),
    toggle: () => store.profiles.bag === 'stop'
      ? send('bag_start', { topics: (store.bagTopics || '').trim().split(/\s+/).filter(Boolean) })
      : send('bag_stop'),
    saveRoute: route => send('route_save', route),
    setMapView: (map, angle) => send('map_view', { map, angle }),
    checkRoute: route => send('route_check', route, 35000),
    // 站起：趴着 → 站稳（2 s 插值）→ 策略接管，后端每步最多等 3 s
    standUp: () => send('stand_up', {}, 15000),
    lieDown: () => send('lie_down', {}, 15000),
    standHold: on => send('stand_hold', { on }),
    prepHold: () => send('prep_hold'),
    deleteRoute: route_id => send('route_delete', { route_id }),
    deleteBag: name => send('bag_delete', { name }, 30000),
    uploadBag: name => send('bag_upload', { name }),
    setRollback: on => send('rollback_set', { on }),
    saveRollback: () => send('rollback_save', {}, 25000),
    // vmax / wmax：这次的速度上限（导航没开时在网页上选的），后端出发前设给 cmd_bridge
    startRoute: (route_id, o = {}) => send('route_start', { route_id, round_trip: !!o.roundTrip, loops: o.loops ?? 1, arm: !!o.arm,
      ...(o.vmax != null ? { vmax: o.vmax } : {}), ...(o.wmax != null ? { wmax: o.wmax } : {}) }),
    saveMap: name => send('save_map', { name }, 45000),
    alignMap: (source, name, angle) => send('align_map', { source, name, angle }, 125000),
    loadMap: name => send('load_map', { name }),
    // 键盘遥控：按住时每 0.1 s 发一次（后端 0.3 s 收不到就发零速）；松手立即发零
    teleop: k => {
      teleCmd = k;
      const tx = () => { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: 'teleop', ...(teleCmd || { v: 0, vy: 0, w: 0 }) })); };
      tx();
      if (k && !teleTimer) teleTimer = setInterval(tx, 100);
      if (!k && teleTimer){ clearInterval(teleTimer); teleTimer = null; }
    },
    // 真实测试：停发心跳 3 s，后端应在超时后取消导航并上锁
    simulateDrop: () => {
      pingPausedUntil = performance.now() + 3000;
      log('WARN', 'go2w_web', '安全测试：停止发送心跳 3 s');
      return Promise.resolve({ ok: true, msg: '已停止发心跳 3 s：如果底盘解锁着，应在约 1 s 后自动上锁' });
    },
    notice: op => bus.emit('toast', NOT_YET[op] || LATER, 'warn'),
  };
  const cmd = new Proxy(real, { get: (o, k) => o[k] || (() => o.notice(k)) });

  connect();
  return {
    name: 'ros', cmd, now,
    can: op => OPS.has(op),
    // 任务卡片的步骤（真机上从 Nav2 行为树日志看得到的阶段）
    steps: [['plan', '规划路径', 'Theta*'], ['follow', '跟随路径', 'RotationShim · MPPI'], ['recover', '恢复行为', '需要时'], ['arrive', '到达', 'goal checker']],
    inspect: () => null,
    // 摄像头：on = 要看 / 不看了（后端最多 3 人同时看）
    camera: { set(on){ camWanted = on; if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: 'cam', on })); } },
    // topic 查看器：订阅一个 topic 的消息内容（后端最多 2 Hz），返回取消函数
    watch(topic, cb){
      if (!watchers.has(topic)){ watchers.set(topic, new Set()); if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: 'inspect', topic, on: true })); }
      watchers.get(topic).add(cb);
      return () => {
        const s = watchers.get(topic); if (!s) return;
        s.delete(cb);
        if (!s.size){ watchers.delete(topic); if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: 'inspect', topic, on: false })); }
      };
    },
    setLayers(ids){ if (ids.join() !== layers.join()){ layers = ids; sendLayers(); }
      // 关掉的重图层清空，避免重新打开时先闪一下旧数据
      if (!ids.includes('cloud')) store.cloud = [];
      if (!ids.includes('particles')) store.particles = new Float32Array(0); },
    get paused(){ return false; },
    tick(){
      if (store.link.ok && performance.now() - lastMsg > 2000){ store.link.ms = null; }
      const R = store.robot, T = store.task;
      // 导航栈停了之后不再有新数据：清掉最后一帧，别让旧的 scan、点云、代价地图留在画面上
      if (!R.valid){ store.scan = new Float32Array(0); store.cloud = []; store.particles = new Float32Array(0); }
      if (store.profiles.navigation !== 'run'){ store.lcostmap = null; store.costmap = null; gcost = null; }
      // 全局路径只在任务进行中显示（Nav2 结束任务时不会发空路径）；已走过的部分按离机器人最近的点区分
      store.plan = rawPlan && T.status === 'active' && T.phase !== 'checking' ? { pts: rawPlan } : null;
      if (store.plan && R.valid){
        let best = 0, bd = Infinity;
        rawPlan.forEach((p, i) => { const d = (p[0]-R.x)**2 + (p[1]-R.y)**2; if (d < bd){ bd = d; best = i; } });
        store.planIdx = best;
      }
      // 局部轨迹：MPPI 没开 visualize，按实际下发的 /cmd_vel 从当前位姿外推 1.6 s
      const c = store.cmd;
      if (R.valid && (Math.abs(c.v) > .01 || Math.abs(c.w) > .01)){
        const pts = []; let x = R.x, y = R.y, th = R.th;
        for (let t=0;t<1.6;t+=.08){ x += (c.v*Math.cos(th) - c.vy*Math.sin(th))*.08; y += (c.v*Math.sin(th) + c.vy*Math.cos(th))*.08; th += c.w*.08; pts.push(x, y); }
        store.localPlan = pts;
      } else store.localPlan = null;
    },
  };
}
