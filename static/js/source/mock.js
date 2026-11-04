// 模拟数据源：示例实验室地图 + 机器人仿真。用于不连机器人时调界面（地址后加 ?mock）
import { clamp, wrap, deg, gauss, mulberry32, gridToCanvas, CLOUD } from '../util.js';

/* ---------- 示例地图 24 × 16 m @ 0.05 ---------- */
const RES = 0.05, MW = 24, MH = 16, GW = Math.round(MW/RES), GH = Math.round(MH/RES);
const WALLS = [[0,0,24,.15],[0,15.85,24,16],[0,0,.15,16],[23.85,0,24,16],
  [9,0,9.15,5.5],[9,7,9.15,16],[9,10,16,10.15],[17.4,10,24,10.15],[16.5,0,16.65,3],[16.5,4.4,16.65,8.6]];
const OBST = [[1.8,1.8,4.4,3],[1.8,6.2,4.4,7.4],[5.2,11.2,8,12.4],[.15,13.4,1.3,15.85],[12,3.8,12.5,4.3],[12,7.2,12.5,7.7],
  [19,1,23.4,1.6],[19,4,23.4,4.6],[19,7,23.4,7.6],[10.8,12.6,13.6,14.2],[19.6,12.2,21.2,14.6],[14.2,14.8,16.4,15.85]];

function buildMap(){
  const rnd = mulberry32(20261030), occ = new Uint8Array(GW*GH); // 0 空闲 1 占用 2 未知
  const cells = (x0,y0,x1,y1) => { const a=Math.round(x0/RES), b=Math.round(y0/RES); return [a, b, Math.max(a+1,Math.round(x1/RES)), Math.max(b+1,Math.round(y1/RES))]; };
  for (const r of WALLS){ const [a,b,c,d] = cells(...r); for (let y=b;y<d;y++) for (let x=a;x<c;x++) occ[y*GW+x] = 1; }
  for (const r of OBST){ const [a,b,c,d] = cells(...r);
    for (let y=b;y<d;y++) for (let x=a;x<c;x++) occ[y*GW+x] = (x<a+2||x>=c-2||y<b+2||y>=d-2) ? 1 : 2; }
  const add = [];
  for (let y=1;y<GH-1;y++) for (let x=1;x<GW-1;x++){ const i=y*GW+x; if (occ[i]) continue;
    if ((occ[i-1]===1||occ[i+1]===1||occ[i-GW]===1||occ[i+GW]===1) && rnd()<.14) add.push(i); }
  for (const i of add) occ[i] = 1;
  const grid = new Uint8Array(GW*GH); for (let i=0;i<grid.length;i++) grid[i] = occ[i]===1 ? 100 : occ[i]===2 ? 255 : 0;
  return { occ, grid };
}

/* ---------- 代价地图 + 规划（0.1 m 栅格，Theta* 风格任意角度捷径） ---------- */
const PR = 0.1, PW = GW/2, PH = GH/2;
function buildCost(occ){
  const cost = new Uint8Array(PW*PH), dist = new Float32Array(PW*PH);
  for (let y=0;y<PH;y++) for (let x=0;x<PW;x++){ const i0=(2*y)*GW+2*x; dist[y*PW+x] = (occ[i0]||occ[i0+1]||occ[i0+GW]||occ[i0+GW+1]) ? 0 : 1e9; }
  for (let y=0;y<PH;y++) for (let x=0;x<PW;x++){ const i=y*PW+x; let d=dist[i];
    if (x>0) d=Math.min(d,dist[i-1]+1); if (y>0){ d=Math.min(d,dist[i-PW]+1); if(x>0)d=Math.min(d,dist[i-PW-1]+1.414); if(x<PW-1)d=Math.min(d,dist[i-PW+1]+1.414);} dist[i]=d; }
  for (let y=PH-1;y>=0;y--) for (let x=PW-1;x>=0;x--){ const i=y*PW+x; let d=dist[i];
    if (x<PW-1) d=Math.min(d,dist[i+1]+1); if (y<PH-1){ d=Math.min(d,dist[i+PW]+1); if(x<PW-1)d=Math.min(d,dist[i+PW+1]+1.414); if(x>0)d=Math.min(d,dist[i+PW-1]+1.414);} dist[i]=d; }
  const INSC = 0.22, INFL = 0.7;
  for (let i=0;i<PW*PH;i++){ const d = dist[i]*PR; cost[i] = d<=0.001 ? 254 : d<INSC ? 253 : d<INFL ? Math.round(252*Math.exp(-2.5*(d-INSC))) : 0; }
  const cv = document.createElement('canvas'); cv.width = PW; cv.height = PH;
  const c = cv.getContext('2d'), im = c.createImageData(PW, PH);
  for (let y=0;y<PH;y++) for (let x=0;x<PW;x++){ const v=cost[y*PW+x], p=((PH-1-y)*PW+x)*4; if (!v) continue; const t = v/254;
    im.data[p]=Math.round(140+115*t); im.data[p+1]=Math.round(80*(1-t)+31.6*t); im.data[p+2]=Math.round(255-92*t); im.data[p+3]=Math.round(v>=253 ? 150 : 25+110*t); }
  c.putImageData(im, 0, 0);
  return { cost, img: cv };
}

export function createMockSource(store, bus){
  const log = (lv, node, msg) => bus.emit('log', lv, node, msg);
  const toast = (m, k) => bus.emit('toast', m, k);
  const { occ, grid } = buildMap();
  const { cost, img: costImg } = buildCost(occ);
  const costAt = (x, y) => { const gx = x/PR|0, gy = y/PR|0; return (x<0||y<0||gx>=PW||gy>=PH) ? 254 : cost[gy*PW+gx]; };

  store.source = 'mock';
  store.map = { img: gridToCanvas(grid, GW, GH), w: MW, h: MH, ox: 0, oy: 0, res: RES, name: 'lab_20261021', cells: `${GW}×${GH}` };
  store.costmap = { img: costImg, w: MW, h: MH, ox: 0, oy: 0 };
  store.costAt = costAt;
  store.link = { ok: true, ms: 23 };
  store.armed = true; store.battery = 76; store.odo = 1284;
  store.patrol = { A: { x: 5.6, y: 4.6, th: Math.PI }, B: { x: 22.5, y: 11.2, th: 0 }, loops: 5 };
  store.mapCurrent = store.map.name;
  const routesKey = 'go2w.mock.routes';
  try {
    const saved = JSON.parse(localStorage.getItem(routesKey) || '[]');
    if (Array.isArray(saved)) store.routes = saved;
  } catch {
    log('WARN', 'routes', '模拟路线读取失败');
  }
  store.maps = [{ name:'lab_20261021', meta:`${GW}×${GH} · 0.05 m`, cur:true }, { name:'corridor_20261028', meta:'612×204 · 0.05 m' }, { name:'lab_20261018_old', meta:'470×318 · 0.05 m' }];
  store.bags = [{ name:'s7_nav4_162403', dur:1840, size:8120 }, { name:'s7_nav1_114954', dur:612, size:2790 }];
  Object.assign(store.robot, { valid: true, x: 5.6, y: 4.6, th: 0 });
  store.profiles = { mapping: 'stop', localization: 'stop', navigation: 'run', patrol: 'run', bag: 'stop' };
  store.since = { navigation: -2405 };
  store.loc = { running: true, std: .035, yaw: 1.5, match: 91 };

  const R = store.robot, S = { t: 0, paused: false, keys: null, link: true, bagT0: 0 };
  const run = id => store.profiles[id] === 'run';
  const anyMode = () => run('mapping') || run('localization') || run('navigation');
  const odomOk = () => anyMode() && S.link;
  const locRun = () => run('localization') || run('navigation');
  const canMove = () => store.armed && !store.estop && run('navigation') && odomOk();
  let task = { type: 'idle', phase: 'idle' };

  /* ---------- 规划 ---------- */
  function astar(sx, sy, gx, gy){
    const N = PW*PH, g = new Float32Array(N).fill(Infinity), came = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
    const s = sy*PW+sx, goal = gy*PW+gx, hf = [], hi = [];
    const push = (f, i) => { hf.push(f); hi.push(i); let k = hf.length-1; while (k>0){ const p=(k-1)>>1; if (hf[p]<=hf[k]) break; [hf[p],hf[k]]=[hf[k],hf[p]]; [hi[p],hi[k]]=[hi[k],hi[p]]; k=p; } };
    const pop = () => { const r = hi[0], lf = hf.pop(), li = hi.pop(); if (hf.length){ hf[0]=lf; hi[0]=li; let k=0; for(;;){ const l=2*k+1, rr=l+1; let m=k; if(l<hf.length&&hf[l]<hf[m])m=l; if(rr<hf.length&&hf[rr]<hf[m])m=rr; if(m===k)break; [hf[m],hf[k]]=[hf[k],hf[m]]; [hi[m],hi[k]]=[hi[k],hi[m]]; k=m; } } return r; };
    g[s] = 0; push(0, s);
    const D = [[1,0,1],[-1,0,1],[0,1,1],[0,-1,1],[1,1,1.414],[1,-1,1.414],[-1,1,1.414],[-1,-1,1.414]];
    while (hf.length){ const i = pop(); if (closed[i]) continue; closed[i] = 1; if (i===goal) break;
      const x = i%PW, y = (i/PW)|0;
      for (const [dx,dy,l] of D){ const nx=x+dx, ny=y+dy; if (nx<0||ny<0||nx>=PW||ny>=PH) continue; const n=ny*PW+nx;
        if (closed[n] || cost[n]>=253) continue; const ng = g[i] + l*(1+cost[n]/40);
        if (ng<g[n]){ g[n]=ng; came[n]=i; push(ng+Math.hypot(gx-nx,gy-ny), n); } } }
    if (came[goal]<0) return null;
    const out = []; for (let i=goal;i!==-1;i=came[i]) out.push([(i%PW+.5)*PR, (((i/PW)|0)+.5)*PR]); return out.reverse();
  }
  function los(a, b){ const L = Math.hypot(b[0]-a[0], b[1]-a[1]), n = Math.ceil(L/.05);
    for (let k=1;k<n;k++){ const t=k/n; if (costAt(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t) > 90) return false; } return true; }
  function plan(from, to){
    const sx = clamp(from.x/PR|0,0,PW-1), sy = clamp(from.y/PR|0,0,PH-1), gx = to.x/PR|0, gy = to.y/PR|0;
    const t0 = performance.now(), raw = astar(sx, sy, gx, gy); if (!raw) return null;
    raw[0] = [from.x, from.y]; raw[raw.length-1] = [to.x, to.y];
    const key = [raw[0]]; let i = 0;
    while (i < raw.length-1){ let j = i+1; while (j+1<raw.length && los(raw[i], raw[j+1])) j++; key.push(raw[j]); i = j; }
    const pts = [key[0]], s = [0];
    for (let k=1;k<key.length;k++){ const a=key[k-1], b=key[k], L=Math.hypot(b[0]-a[0],b[1]-a[1]), n=Math.max(1,Math.ceil(L/.1));
      for (let m=1;m<=n;m++){ pts.push([a[0]+(b[0]-a[0])*m/n, a[1]+(b[1]-a[1])*m/n]); s.push(s[s.length-1]+L/n); } }
    return { pts, s, total: s[s.length-1], ms: performance.now()-t0, corners: key.length };
  }

  /* ---------- 任务 ---------- */
  const goalOf = () => task.type==='patrol' ? store.patrol[task.leg] : task.goal;
  function startPatrol(){ task = { type:'patrol', leg:'B', loop:1, phase:'plan', planAt:S.t+.35, t0:S.t, recov:0 }; log('INFO','patrol',`start A↔B patrol, ${store.patrol.loops} loops`); }
  function startGoal(g){ task = { type:'goal', goal:g, phase:'plan', planAt:S.t+.35, t0:S.t, recov:0 };
    log('INFO','bt_navigator',`Begin navigating from (${R.x.toFixed(2)}, ${R.y.toFixed(2)}) to (${g.x.toFixed(2)}, ${g.y.toFixed(2)})`); }
  const routeActive = () => task.type === 'route' && !task.status;
  function routeLeg(route, index, started){
    task = { type:'route', route, index, started, goal:route.points[index], phase:'plan',
      planAt:S.t+.35, t0:S.t, recov:0 };
  }
  function finishRoute(status, message){
    task = { ...task, phase:status === 'succeeded' ? 'arrived' : status, status, message, doneAt:S.t };
    log(status === 'aborted' ? 'ERROR' : 'INFO', 'routes', message);
    toast(message, status === 'aborted' ? 'err' : status === 'canceled' ? 'warn' : 'ok');
  }
  function cancelTask(reason){
    if (routeActive()) finishRoute('canceled', reason || '已取消路线任务');
    else if (task.type !== 'idle'){
      log('INFO','bt_navigator', reason || 'Goal canceled');
      task = { type:'idle', phase:'idle' };
    }
    S.paused = false;
  }
  function lock(reason){ if (!store.armed) return; store.armed = false; log('WARN','go2w_cmd_bridge',`StopMove sent, LOCKED (${reason})`); }
  function arrive(){
    log('INFO','bt_navigator',`Goal succeeded (${(S.t-task.legT0).toFixed(1)} s)`);
    if (task.type==='patrol'){ task.phase = 'dwell'; task.dwellUntil = S.t + 1.2; }
    else if (task.type==='route'){
      if (task.index + 1 === task.route.points.length) finishRoute('succeeded', '已到达全部导航点');
      else routeLeg(task.route, task.index + 1, task.started);
    }
    else { const g = task.goal; toast(`已到达目标 (${g.x.toFixed(2)}, ${g.y.toFixed(2)})`); task.phase = 'arrived'; task.doneAt = S.t; }
  }

  function step(dt){
    S.t += dt;
    let tv = 0, tw = 0, tvy = 0;
    if (task.phase==='plan' && S.t>=task.planAt){
      const g = goalOf(), p = plan(R, g);
      if (!p){
        if (routeActive()) finishRoute('aborted', `第 ${task.index + 1} 点失败：找不到可行路径`);
        else {
          log('ERROR','planner_server','Failed to create a plan: no valid path');
          toast('规划失败：找不到可行路径','err');
          task = { type:'idle', phase:'idle' };
        }
      }
      else { task.path = p; task.idx = 0; task.phase = 'align'; task.legT0 = S.t;
        log('INFO','planner_server',`Theta*: ${p.pts.length} poses, ${p.total.toFixed(2)} m, ${p.corners} vertices (${p.ms.toFixed(1)} ms)`); }
    }
    const active = ['align','follow','final'].includes(task.phase);
    if (active && !S.paused && canMove()){
      const P = task.path, g = goalOf();
      let best = task.idx, bd = 1e9;
      for (let i=task.idx;i<Math.min(P.pts.length, task.idx+40);i++){ const d = Math.hypot(P.pts[i][0]-R.x, P.pts[i][1]-R.y); if (d<bd){ bd=d; best=i; } }
      task.idx = best;
      let la = best; while (la<P.pts.length-1 && P.s[la] < P.s[best]+.6) la++;
      const lp = P.pts[la], err = wrap(Math.atan2(lp[1]-R.y, lp[0]-R.x) - R.th);
      const dGoal = Math.hypot(g.x-R.x, g.y-R.y), rem = P.total - P.s[best], { vmax, wmax } = store.params;
      if (task.phase==='align'){ tw = Math.sign(err)*Math.min(wmax*.75, Math.abs(err)*2.2+.15); if (Math.abs(err) < .14) task.phase = 'follow'; }
      else if (task.phase==='follow'){
        if (Math.abs(err) > .95 && dGoal > .5) task.phase = 'align';
        tv = Math.min(vmax, .12 + rem*.55) * Math.max(.2, Math.cos(err)); tw = clamp(2.4*err, -wmax, wmax);
        if (dGoal < .12 || (rem < .12 && dGoal < .3)) task.phase = 'final';
      } else { const e2 = wrap(g.th - R.th); tw = clamp(1.6*e2, -wmax*.7, wmax*.7); if (Math.abs(e2) < .04){ tw = 0; arrive(); } }
    }
    if (task.phase==='dwell' && S.t>=task.dwellUntil){
      if (task.leg==='B') task.leg = 'A'; else { task.leg = 'B'; task.loop++; }
      if (task.loop > store.patrol.loops){ log('INFO','patrol','patrol finished'); toast('巡逻完成'); task = { type:'idle', phase:'idle' }; }
      else { task.phase = 'plan'; task.planAt = S.t+.3; log('INFO','patrol',`leg ${task.leg==='B'?'A→B':'B→A'} (loop ${task.loop}/${store.patrol.loops})`); }
    }
    if (task.doneAt != null && S.t - task.doneAt > (task.type === 'route' ? 10 : 4)) task = { type:'idle', phase:'idle' };
    if (S.keys && canMove()){ tv = S.keys.v; tw = S.keys.w; tvy = S.keys.vy; }
    store.cmd = { v: tv, w: tw, vy: tvy };
    const av = 1.0*dt, aw = 2.0*dt;
    R.v += clamp(tv - R.v, -av, av); R.w += clamp(tw - R.w, -aw, aw); R.vy += clamp(tvy - R.vy, -av, av);
    if (!canMove()){ const k = Math.max(0, 1-dt*6); R.v *= k; R.w *= k; R.vy *= k; }
    const nx = R.x + (R.v*Math.cos(R.th) - R.vy*Math.sin(R.th))*dt, ny = R.y + (R.v*Math.sin(R.th) + R.vy*Math.cos(R.th))*dt;
    if (costAt(nx, ny) < 253){ R.x = nx; R.y = ny; } else { R.v = 0; R.vy = 0; }
    R.th = wrap(R.th + R.w*dt);
    store.odo += Math.hypot(R.v, R.vy)*dt;
    store.imu = { pitch: .9 + (tv - R.v)*3 + Math.sin(S.t*7)*.08, roll: -.4 + R.w*.6 + Math.sin(S.t*5.3)*.06 };
    const tr = store.trail, last = tr[tr.length-1];
    if (!last || Math.hypot(last[0]-R.x, last[1]-R.y) > .08){ tr.push([R.x, R.y]); if (tr.length > 900) tr.shift(); }
    const L = store.loc;
    if (locRun()){
      L.running = true;
      const tgt = .03 + .012*Math.abs(R.v) + .01*Math.abs(R.w);
      L.std += (tgt - L.std)*Math.min(1, dt*.6); L.yaw += ((1.3 + 1.1*Math.abs(R.w)) - L.yaw)*Math.min(1, dt*.6);
      L.match += ((93 - 5*Math.abs(R.w) - 40*Math.max(0, L.std-.05)) - L.match)*Math.min(1, dt*.8);
    } else L.running = false;
    syncTask();
  }

  // 把内部任务状态整理成界面需要的样子
  function syncTask(){
    store.plan = task.path && ['align','follow','final','dwell'].includes(task.phase) && run('navigation') ? task.path : null;
    store.planIdx = task.phase==='dwell' && task.path ? task.path.pts.length-1 : (task.idx || 0);
    store.goal = (task.type==='goal' && task.phase!=='arrived') || routeActive() ? task.goal : null;
    if (task.type==='idle'){ store.task = { type:'idle' }; return; }
    const p = task.path, rem = p ? (['dwell','arrived'].includes(task.phase) ? 0 : Math.max(0, p.total - p.s[task.idx||0])) : null;
    store.task = { type: task.type, phase: task.phase, paused: S.paused, recoveries: task.recov,
      status: task.status || (task.phase === 'arrived' ? 'succeeded' : 'active'),
      remaining: rem, total: p ? p.total : null, elapsed: S.t - (task.legT0 ?? task.t0),
      goal: task.goal, loop: task.loop, loops: store.patrol.loops, leg: task.leg };
    if (task.type === 'route'){
      Object.assign(store.task, { name:task.route.name, point:task.index + 1, count:task.route.points.length,
        message:task.message, routeId:task.route.id, pointName:task.goal.name, elapsed:(task.doneAt ?? S.t) - task.started });
    }
  }

  /* ---------- 传感器与可视化 ---------- */
  function sense(){
    if (!odomOk()){ store.scan = new Float32Array(0); store.cloud = []; return; }
    const ox = R.x + .187*Math.cos(R.th), oy = R.y + .187*Math.sin(R.th), out = [], rng = [];
    for (let i=0;i<360;i++){ const a = i*Math.PI/180, c = Math.cos(a), s = Math.sin(a); let r = .1;
      while (r < 15){ const gx = (ox+c*r)/RES|0, gy = (oy+s*r)/RES|0; if (gx<0||gy<0||gx>=GW||gy>=GH){ r = 99; break; } if (occ[gy*GW+gx]===1) break; r += .04; }
      if (r < 15){ const n = gauss()*.015; out.push(ox+c*(r+n), oy+s*(r+n)); rng.push(r); } }
    store.scan = Float32Array.from(out);
    const cloud = CLOUD.map(() => []);
    for (let k=0;k<rng.length;k++){ const r = rng[k]; if (r > 11) continue;
      for (let m=0;m<(r<5?3:2);m++){ const z = .3 + Math.random()*1.3, ci = Math.min(CLOUD.length-1, ((z-.3)/1.3*CLOUD.length)|0);
        cloud[ci].push(out[2*k] + gauss()*.03, out[2*k+1] + gauss()*.03); } }
    store.cloud = cloud;
  }
  function rollout(v, w, T = 1.6){ const pts = []; let x = R.x, y = R.y, th = R.th; for (let t=0;t<T;t+=.08){ x += v*Math.cos(th)*.08; y += v*Math.sin(th)*.08; th += w*.08; pts.push(x, y); } return pts; }
  function controllerViz(){
    const c = store.cmd, active = (['align','follow','final'].includes(task.phase) && !S.paused) || S.keys;
    if (!active || !canMove()){ store.mppi = []; store.localPlan = null; return; }
    store.mppi = Array.from({ length: 26 }, () => rollout(Math.max(0, c.v + gauss()*.12), c.w + gauss()*.35));
    store.localPlan = rollout(c.v || R.v, c.w || R.w);
  }
  function amclViz(){
    if (!locRun()){ store.particles = new Float32Array(0); return; }
    const n = 160, a = new Float32Array(n*3), L = store.loc, sy = L.yaw*Math.PI/180;
    for (let i=0;i<n;i++){ a[i*3] = R.x + gauss()*L.std*1.6; a[i*3+1] = R.y + gauss()*L.std*1.6; a[i*3+2] = R.th + gauss()*sy*1.6; }
    store.particles = a;
  }

  /* ---------- 档位 / 节点 / topic ---------- */
  const PKG = { mapping:'mapping.launch.py', localization:'localization.launch.py', navigation:'navigation.launch.py', patrol:'patrol.py', bag:'ros2 bag record' };
  function setState(id, st){ store.profiles[id] = st; if (st==='run') store.since[id] = S.t; }
  function startProfile(id, delay = 0){
    setTimeout(() => { setState(id, 'starting');
      setTimeout(() => { setState(id, 'run'); log('INFO','launch',`[${PKG[id]}] process started (pid ${4000+((Math.random()*5000)|0)})`); onUp(id); }, 1200);
    }, delay);
  }
  function stopProfile(id){
    if (store.profiles[id]==='stop') return;
    setState(id, 'stopping'); onDown(id);
    setTimeout(() => { setState(id, 'stop'); log('INFO','launch',`[${PKG[id]}] process has finished cleanly`); }, 700);
  }
  function onUp(id){
    if (id==='patrol' && task.type!=='patrol') startPatrol();
    if (id==='bag') S.bagT0 = S.t;
    if (id==='localization' || id==='navigation') Object.assign(store.loc, { std: .35, yaw: 12, match: 40 });
  }
  function onDown(id){
    if (['mapping','localization','navigation'].includes(id)){ log('ERROR','planar_odom','FAST-LIO odometry gap > 0.5 s, /odom_healthy = false'); lock('odom unhealthy'); }
    if (id==='navigation'){ cancelTask('Goal canceled: navigation stack stopped'); lock('nav stopped'); }
    if (id==='patrol' && task.type==='patrol') cancelTask('patrol stopped');
    if (id==='bag'){ const d = S.t - S.bagT0; store.bags.unshift({ name:`go2w_${new Date().toTimeString().slice(0,5).replace(':','')}`, dur:d, size:d*4.6 }); toast('录包已保存'); }
  }
  const NODES = [['livox_lidar_publisher','雷达驱动','odom'],['fast_lio','FAST-LIO','odom'],['planar_odom','planar_odom','odom'],['pcl2scan_slam','pcl2scan','odom'],
    ['amcl','AMCL','loc'],['slam_toolbox','slam_toolbox','mapping'],['bt_navigator','Nav2','navigation'],['go2w_cmd_bridge','cmd_bridge','navigation']];
  const TOPICS = [
    ['/livox/lidar','livox_ros_driver2/msg/CustomMsg',10,'odom'], ['/livox/imu','sensor_msgs/msg/Imu',200,'odom'],
    ['/cloud_registered_body','sensor_msgs/msg/PointCloud2',10,'odom'], ['/scan','sensor_msgs/msg/LaserScan',10,'odom'],
    ['/odom','nav_msgs/msg/Odometry',50,'odom'], ['/odom_healthy','std_msgs/msg/Bool',2,'odom'], ['/tf','tf2_msgs/msg/TFMessage',100,'odom'],
    ['/lowstate','unitree_go/msg/LowState',516,'robot',1], ['/map','nav_msgs/msg/OccupancyGrid',0,'odom'],
    ['/amcl_pose','geometry_msgs/msg/PoseWithCovarianceStamped',5,'loc'], ['/particle_cloud','nav2_msgs/msg/ParticleCloud',5,'loc'],
    ['/plan','nav_msgs/msg/Path',1,'navigation'], ['/local_plan','nav_msgs/msg/Path',20,'navigation'], ['/cmd_vel','geometry_msgs/msg/Twist',20,'navigation'],
    ['/global_costmap/costmap','nav_msgs/msg/OccupancyGrid',1,'navigation'], ['/local_costmap/costmap','nav_msgs/msg/OccupancyGrid',5,'navigation'],
    ['/go2w_cmd_bridge/armed','std_msgs/msg/Bool',10,'navigation'], ['/patrol_pose','geometry_msgs/msg/PoseStamped',0,'patrol'], ['/rosout','rcl_interfaces/msg/Log',0,'robot'],
  ];
  const alive = src => src==='robot' ? S.link : src==='odom' ? odomOk() : src==='loc' ? locRun() : run(src);
  function syncGraph(){
    store.nodes = NODES.map(([id, name, src]) => ({ id, name, up: alive(src) }));
    store.topics = TOPICS.map(([name, type, hz, src, d0]) => ({ name, type, d0: !!d0, alive: alive(src),
      hz: !alive(src) ? null : hz===0 ? 0 : +(hz*(1+gauss()*.02)).toFixed(hz>=100?0:1) }));
    const busy = Object.values(store.profiles).filter(s => s==='run');
    store.sys = { cpu: 8 + busy.length*9 + gauss(), mem: 1.4 + busy.length*.35, memTotal: 7.4 };
    store.link.ms = S.link ? clamp((store.link.ms || 23) + gauss()*2, 14, 48) : null;
    store.link.ok = S.link;
    store.odomOk = odomOk();
    store.bagSince = run('bag') ? S.t - S.bagT0 : null;
  }

  /* ---------- 指令（界面已经做过前置检查和确认） ---------- */
  const cmd = {
    initialPose(p){ cancelTask('Goal canceled: initial pose reset'); Object.assign(R, { x:p.x, y:p.y, th:p.th, v:0, w:0, vy:0 }); store.trail = [];
      Object.assign(store.loc, { std: .4, yaw: 14, match: 38 }); amclViz(); log('INFO','amcl',`Setting pose: ${p.x.toFixed(3)} ${p.y.toFixed(3)} ${p.th.toFixed(3)}`); },
    goal(g){
      if (routeActive()) return { ok:false, msg:'多点任务进行中：请先取消任务' };
      if (task.type==='patrol') stopProfile('patrol');
      startGoal(g);
    },
    saveRoute({ route_id, name, map, points }){
      const id = route_id || globalThis.crypto?.randomUUID?.() || `mock-${Date.now()}-${Math.random()}`;
      const record = { id, name:name.trim(), map, points:points.map(p => ({ ...p, name:(p.name || '').trim() })) };
      const routes = route_id ? store.routes.map(r => r.id === id ? record : r) : [...store.routes, record];
      try {
        localStorage.setItem(routesKey, JSON.stringify(routes));
      } catch {
        return { ok:false, msg:'浏览器无法保存模拟任务' };
      }
      store.routes = routes;
      return { ok:true, data:record };
    },
    deleteRoute(id){
      if (routeActive() && task.route.id === id) return { ok:false, msg:'请先结束正在执行的任务' };
      const routes = store.routes.filter(r => r.id !== id);
      if (routes.length === store.routes.length) return { ok:false, msg:'找不到要删除的任务' };
      try {
        localStorage.setItem(routesKey, JSON.stringify(routes));
      } catch {
        return { ok:false, msg:'浏览器无法删除模拟任务' };
      }
      store.routes = routes;
      return { ok:true };
    },
    startRoute(id){
      const route = store.routes.find(r => r.id === id);
      if (!route || route.map !== store.mapCurrent || !canMove() || S.keys || run('patrol') ||
          (task.type !== 'idle' && task.doneAt == null)){
        return { ok:false, msg:'请确认地图、导航及底盘状态，并结束其他任务' };
      }
      routeLeg(route, 0, S.t);
      return { ok:true };
    },
    setPatrolPoint(k, p){ store.patrol[k] = p; log('INFO','patrol',`point ${k} set to (${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${deg(p.th).toFixed(0)}°)`);
      if (task.type==='patrol'){ task.phase = 'plan'; task.planAt = S.t+.2; } },
    cancel(){ if (task.type==='patrol') stopProfile('patrol'); cancelTask(); },
    pause(on){ S.paused = on; log('INFO','bt_navigator', on ? 'Navigation paused' : 'Navigation resumed'); },
    arm(on){
      if (on){ store.armed = true; log('INFO','go2w_cmd_bridge','ARMED'); }
      else {
        if (routeActive()) cancelTask('底盘上锁，已取消路线任务');
        lock('manual');
      }
    },
    estop(){ if (routeActive()) cancelTask('急停，已取消路线任务'); store.estop = true; store.armed = false; S.paused = true; R.v = R.w = R.vy = 0; S.keys = null; log('ERROR','go2w_cmd_bridge','E-STOP from web console: StopMove sent, LOCKED'); },
    releaseEstop(){ store.estop = false; log('INFO','go2w_cmd_bridge','E-STOP released (still LOCKED)'); },
    setParam(k, v){ store.params[k] = v; log('INFO','param',`ros2 param set /go2w_cmd_bridge ${{ vmax:'max_vx', wmax:'max_wz', tmo:'cmd_timeout' }[k]} ${v.toFixed(2)}`); },
    // mode = mapping | localization | navigation | null（全部停止）
    setMode(mode){
      const cur = ['mapping','localization','navigation'].filter(m => store.profiles[m]!=='stop');
      const down = cur.filter(m => m!==mode);
      if (down.length){
        if (routeActive()) cancelTask('切换运行档位，已取消路线任务');
        stopProfile('patrol');
      }
      down.forEach(stopProfile);
      if (mode && !cur.includes(mode)) startProfile(mode, down.length ? 900 : 0);
    },
    toggle(id){
      if (store.profiles[id]==='run') return stopProfile(id);
      if (id==='patrol' && !run('navigation')){ cmd.setMode('navigation'); startProfile('patrol', 2400); return; }
      startProfile(id);
    },
    startPatrol(){ run('patrol') ? startPatrol() : cmd.toggle('patrol'); },
    teleop(k){ if (k && task.type!=='idle') cancelTask('Goal canceled: manual teleop'); S.keys = k; },
    saveMap(name){ store.maps.unshift({ name, meta:`${GW}×${GH} · 0.05 m` }); log('INFO','slam_toolbox',`Saved map to data/maps/${name}`); },
    loadMap(name){ store.mapCurrent = name; store.maps.forEach(m => m.cur = m.name===name); cancelTask('Goal canceled: map changed'); log('INFO','map_server',`Loading yaml file: data/maps/${name}.yaml`); },
    simulateDrop(){ if (!S.link) return; S.link = false; log('WARN','go2w_web','client heartbeat lost (> 1.0 s)');
      setTimeout(() => {
        if (routeActive()) cancelTask('网页断开，已取消路线任务');
        lock('web heartbeat lost');
      }, 1000);
      setTimeout(() => { S.link = true; log('INFO','go2w_web','client reconnected, chassis stays LOCKED'); toast('已重连，底盘保持锁定，需要手动解锁','warn'); }, 3000); },
  };

  /* ---------- topic 消息内容（检查器） ---------- */
  function inspect(n){
    const q = (k, v) => [k, v], yaw = R.th, now = (Date.now()/1000).toFixed(3), L = store.loc;
    switch (n){
      case '/odom': return [q('header.stamp',now),q('header.frame_id','odom'),q('child_frame_id','base_footprint'),q('pose.position.x',+R.x.toFixed(3)),q('pose.position.y',+R.y.toFixed(3)),q('pose.orientation.z',+Math.sin(yaw/2).toFixed(4)),q('pose.orientation.w',+Math.cos(yaw/2).toFixed(4)),q('twist.linear.x',+R.v.toFixed(3)),q('twist.linear.y',+R.vy.toFixed(3)),q('twist.angular.z',+R.w.toFixed(3))];
      case '/cmd_vel': return [q('linear.x',+store.cmd.v.toFixed(3)),q('linear.y',+store.cmd.vy.toFixed(3)),q('angular.z',+store.cmd.w.toFixed(3))];
      case '/scan': return [q('header.frame_id','base_footprint'),q('angle_min',-3.1416),q('angle_max',3.1416),q('angle_increment',.01745),q('range_min',.1),q('range_max',15),q('ranges[]',`${store.scan.length/2} valid`)];
      case '/amcl_pose': return [q('header.frame_id','map'),q('pose.pose.position.x',+R.x.toFixed(3)),q('pose.pose.position.y',+R.y.toFixed(3)),q('yaw (deg)',+deg(yaw).toFixed(1)),q('covariance[0]',+(L.std**2).toFixed(5))];
      case '/lowstate': return [q('imu_state.rpy[0]',+(store.imu.roll*Math.PI/180).toFixed(4)),q('imu_state.rpy[1]',+(store.imu.pitch*Math.PI/180).toFixed(4)),q('imu_state.rpy[2]',+yaw.toFixed(4)),q('bms_state.soc',store.battery)];
      case '/odom_healthy': return [q('data', String(odomOk()))];
      case '/go2w_cmd_bridge/armed': return [q('data', String(!!store.armed))];
      case '/plan': return store.plan ? [q('header.frame_id','map'),q('poses[]',store.plan.pts.length),q('length (m)',+store.plan.total.toFixed(2))] : [q('poses[]',0)];
      case '/map': return [q('info.resolution',RES),q('info.width',GW),q('info.height',GH),q('info.origin.position','[0.0, 0.0, 0.0]')];
      default: return null;
    }
  }

  /* ---------- 启动 ---------- */
  [['INFO','launch','navigation.launch.py: all nodes started'], ['INFO','planar_odom','tilt_source = lowstate (domain 0, best-effort)'],
   ['INFO','amcl','initialPoseReceived, particle cloud converged'], ['INFO','go2w_cmd_bridge','limits vx 0.45 m/s · wz 1.20 rad/s · timeout 0.30 s'],
   ['INFO','go2w_cmd_bridge','ARMED']].forEach(l => log(...l));
  for (let i=0;i<300;i++) step(.05); // 预跑 15 s：一打开就是机器人在路上的状态
  sense(); amclViz(); controllerViz(); syncGraph();

  const AMBIENT = [['INFO','controller_server','Passing new path to controller.'], ['INFO','amcl','Received a 360 X 2 laser scan'],
    ['INFO','planar_odom','tilt roll -0.4° pitch 0.9°'], ['INFO','local_costmap','Cleared voxel layer around robot']];
  let a10 = 0, a5 = 0, a1 = 0, aLog = 0;
  return {
    name: 'mock', cmd, inspect,
    can: () => true,
    now: () => S.t,
    steps: [['plan', '规划路径', 'Theta*'], ['align', '原地对准', 'RotationShim'], ['follow', '跟随路径', 'MPPI'], ['final', '终点朝向对准', 'goal yaw'], ['arrive', '到达', '停留 1 s']],
    get paused(){ return S.paused; },
    tick(dt){
      step(dt); a10 += dt; a5 += dt; a1 += dt; aLog += dt;
      if (a10 >= .1){ a10 = 0; sense(); controllerViz(); }
      if (a5 >= .2){ a5 = 0; amclViz(); }
      if (a1 >= 1){ a1 = 0; syncGraph(); }
      if (aLog >= 3.5){ aLog = 0; if (run('navigation') && task.type!=='idle' && !S.paused && canMove()){ log(...AMBIENT[(Math.random()*AMBIENT.length)|0]);
        if (Math.random() < .08) log('WARN','controller_server','Control loop missed its desired rate of 20.0000 Hz (0.0512 s)'); } }
    },
  };
}
