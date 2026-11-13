// 中间的地图画布：渲染各图层 + 地图工具（平移、初始位姿、目标、A/B 点、测距）
import { $, $$, clamp, ACC, TRAJ, CLOUD } from './util.js';
import { on } from './layers.js';
import { toast } from './ui.js';
import { rotatePoint, wallAngle, screenToWorld } from './map_alignment.js';

export function createMapView(store, actions){
  const cv = $('#map'), ctx = cv.getContext('2d'), stage = $('#stage');
  let W = 0, H = 0, DPR = 1, t = 0;
  const view = { cx: 12, cy: 8, s: 40, follow: false };
  let touched = false; // 用户动过视角后，窗口尺寸变化不再自动适配
  const alignment = () => store.alignment;
  // 地图显示方向：每张地图一个角度，存在后端（store.mapViews）。建图时还没有地图名，用 MAPPING_VIEW 暂存
  const MAPPING_VIEW = '_mapping', SNAP = 3 * Math.PI / 180, QUARTER = Math.PI / 2;
  let viewKey = null, savedAngle = null;
  const currentKey = () => store.mapCurrent || (actions.curMode() === 'mapping' ? MAPPING_VIEW : null);
  const wrapPi = a => Math.atan2(Math.sin(a), Math.cos(a));
  const flash = () => { alignment().flashUntil = performance.now() + 2500; };
  function setAngle(a, save = false){
    alignment().angle = wrapPi(a); flash(); syncCompass();
    const key = currentKey();
    if (save && key) actions.setMapView(key, alignment().angle);
  }
  function syncCompass(){
    const deg = alignment().angle * 180 / Math.PI;
    $('#vRotNeedle').style.transform = `rotate(${-alignment().angle}rad)`;
    $('#vRot').title = `地图显示方向 ${deg.toFixed(1)}° · 单击顺时针转 90°，双击复位 · Shift 或右键拖动地图可自由旋转（按住 Alt 不吸附）`;
  }
  // aim：点了一下位置、还在等第二下确定朝向（点两下 = 按住拖动的替代，触控板更好用）
  const V = { tool: 'pan', measure: [], drag: null, aim: null };
  const POSE_TOOLS = { init: '初始位姿' };
  const hint = $('#aimHint');
  function showHint(){
    if (V.tool === 'align'){
      hint.hidden = false;
      hint.textContent = alignment().points.length ? '再选墙上的第二点 · 优先选取至少 1 米的长直墙' : '沿同一段长直墙点两下 · 选择至少 0.5 米的墙段';
      return;
    }
    if (V.tool === 'route'){
      hint.hidden = false;
      hint.textContent = '点击添加一站 · 按住拖拽指定到站朝向';
      return;
    }
    const name = POSE_TOOLS[V.tool];
    hint.hidden = !name;
    if (name) hint.innerHTML = V.aim ? `<b>${name}</b> · 移动鼠标选择朝向，再点一下确定 · Esc 取消` : `<b>${name}</b> · 点一下选位置，再点一下定朝向（也可以按住拖动）`;
  }

  const sx = x => W/2 + (x-view.cx)*view.s, sy = y => H/2 - (y-view.cy)*view.s;
  const world = (px, py) => screenToWorld(px, py, W, H, view, alignment().angle);
  function resize(){ const r = stage.getBoundingClientRect(); DPR = window.devicePixelRatio||1; W = r.width; H = r.height; cv.width = Math.round(W*DPR); cv.height = Math.round(H*DPR); }
  function fit(){
    const m = store.map; if (!m || !W) return;
    const corners = [[0,0],[m.w,0],[0,m.h],[m.w,m.h]].map(([x,y]) => {
      const p = rotatePoint({x,y}, m.yaw || 0);
      return rotatePoint({x:p.x+m.ox,y:p.y+m.oy}, alignment().angle);
    });
    const xs = corners.map(p=>p.x), ys = corners.map(p=>p.y);
    const left = Math.min(...xs), right = Math.max(...xs), bottom = Math.min(...ys), top = Math.max(...ys);
    view.s = Math.min(W/(right-left), (H-60)/(top-bottom))*.94;
    const center = rotatePoint({x:(left+right)/2,y:(bottom+top)/2-10/view.s}, -alignment().angle);
    view.cx = center.x; view.cy = center.y;
  }
  const inMap = (x, y) => {
    const m = store.map; if (!m) return false;
    const p = rotatePoint({x:x-m.ox,y:y-m.oy}, -(m.yaw || 0));
    return p.x>=0 && p.y>=0 && p.x<m.w && p.y<m.h;
  };
  // 清掉「沿墙选两点」的选点（换地图时这些点就没意义了）
  function clearWall(){
    Object.assign(alignment(), { points:[], target:null, source:null });
    if (V.tool === 'align') setTool('pan');
  }
  // 「复位方向」：回到 0°，并存到后端
  function resetAlignment(){
    clearWall(); V.drag = V.aim = null;
    setAngle(0, true); fit();
  }
  function alignWall(target){
    if (store.mapSaving || store.mapBusy) return;
    const points = alignment().points;
    if (points.length !== 2) return toast('请先沿一段墙选择两个点', 'warn');
    alignment().target = target;
    setAngle(wallAngle(points, target), true);
    touched = true; fit();
  }
  // 地图名变了：换成那张图存的方向；别的网页改了方向：跟着变（自己正在拖的时候不打断）
  function syncView(){
    const key = currentKey();
    const saved = key ? (store.mapViews[key] ?? 0) : null;
    if (key !== viewKey){
      viewKey = key; savedAngle = saved;
      if (key === null) return;           // 暂时不知道是哪张图（连接断开、档位切换中）：保持当前方向
      clearWall();
      alignment().angle = saved; syncCompass();
      fit();
    } else if (key !== null && saved !== savedAngle){
      savedAngle = saved;
      if (V.drag?.tool !== 'rot'){ alignment().angle = saved; flash(); syncCompass(); }
    }
  }
  // 栅格图像：原点 (ox, oy) 在左下角，可以带 yaw（真机的局部代价地图在 odom 下，转到 map 后是斜的）
  function drawGridImg(m, img){
    if (!m.yaw) return ctx.drawImage(img, sx(m.ox), sy(m.oy+m.h), m.w*view.s, m.h*view.s);
    ctx.save(); ctx.translate(sx(m.ox), sy(m.oy)); ctx.rotate(-m.yaw);
    ctx.drawImage(img, 0, -m.h*view.s, m.w*view.s, m.h*view.s); ctx.restore();
  }

  function polyline(arr, step = 2){ ctx.beginPath(); for (let i=0;i<arr.length;i+=step){ const x = sx(arr[i]), y = sy(arr[i+1]); i ? ctx.lineTo(x,y) : ctx.moveTo(x,y); } }
  function drawPin(p, label, col, showHeading = true){
    const x = sx(p.x), y = sy(p.y);
    ctx.save(); ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 1.5;
    ctx.globalAlpha = .25; ctx.beginPath(); ctx.arc(x, y, 10, 0, 7); ctx.fill(); ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.fill();
    if (showHeading){ ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(-p.th)*20, y + Math.sin(-p.th)*20); ctx.stroke(); }
    ctx.fillStyle = '#04070c'; ctx.beginPath(); ctx.roundRect(x-9, y-32, 18, 18, 4); ctx.fill(); ctx.stroke();
    ctx.fillStyle = col; ctx.font = '600 11px "Chakra Petch",sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(label, x, y-23);
    ctx.beginPath(); ctx.moveTo(x, y-14); ctx.lineTo(x, y-4); ctx.stroke(); ctx.restore();
  }
  function drawArrow(x0, y0, x1, y1, col){
    ctx.save(); ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    const a = Math.atan2(y1-y0, x1-x0); ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1-12*Math.cos(a-.45), y1-12*Math.sin(a-.45)); ctx.lineTo(x1-12*Math.cos(a+.45), y1-12*Math.sin(a+.45)); ctx.fill();
    ctx.beginPath(); ctx.arc(x0, y0, 5, 0, 7); ctx.fill(); ctx.restore();
  }
  function drawGoal(g){
    const x = sx(g.x), y = sy(g.y), pulse = (t*1.4)%1;
    ctx.save(); ctx.strokeStyle = ACC; ctx.lineWidth = 1.5;
    ctx.globalAlpha = 1-pulse; ctx.beginPath(); ctx.arc(x, y, 8+pulse*22, 0, 7); ctx.stroke(); ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(x, y, 8, 0, 7); ctx.stroke();
    ctx.fillStyle = ACC; ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.fill();
    ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x+Math.cos(-g.th)*22, y+Math.sin(-g.th)*22); ctx.stroke(); ctx.restore();
  }
  // 重定位：修正后的位置（青色虚线车身 + 车头圆点，拖车身平移、拖圆点旋转）
  const knobLen = () => Math.max(0.8, 34 / view.s);
  function drawRelocGhost(){
    const P = store.reloc.pose(); if (!P) return;
    const changed = store.reloc.changed(), x = sx(P.x), y = sy(P.y), L = .7*view.s, Wd = .42*view.s;
    const kx = sx(P.x + Math.cos(P.th)*knobLen()), ky = sy(P.y + Math.sin(P.th)*knobLen());
    ctx.save(); ctx.strokeStyle = '#38e1dc'; ctx.fillStyle = 'rgba(56,225,220,.12)'; ctx.lineWidth = 2;
    ctx.setLineDash(changed ? [5,4] : []);
    ctx.save(); ctx.translate(x, y); ctx.rotate(-P.th); ctx.beginPath(); ctx.roundRect(-L/2, -Wd/2, L, Wd, 4); ctx.fill(); ctx.stroke(); ctx.restore();
    ctx.setLineDash([]); ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(kx, ky); ctx.stroke();
    ctx.fillStyle = '#38e1dc'; ctx.beginPath(); ctx.arc(kx, ky, 6, 0, 7); ctx.fill();
    ctx.fillStyle = '#04070c'; ctx.beginPath(); ctx.arc(kx, ky, 2.5, 0, 7); ctx.fill();
    ctx.restore();
  }
  // 小车颜色跟着任务走：空闲青色 / 执行中琥珀色（呼吸） / 到位绿色（停在终点附近一直保持） / 失败红色 / 急停红色
  const COL = { idle: ACC, run: '#f3b13f', ok: '#35d48a', fail: '#ff4b5c', off: '#9db0c7' };
  const rgba = (hex, a) => `rgba(${parseInt(hex.slice(1,3),16)},${parseInt(hex.slice(3,5),16)},${parseInt(hex.slice(5,7),16)},${a.toFixed(3)})`;
  const RS = { prev: null, point: null, arrived: null, burstAt: -1e9, burstCol: COL.ok };
  const ARRIVED_KEEP = 0.3; // 离开终点超过 0.3 m 就不再显示「到位」
  function robotState(R){
    const T = store.task || {}, active = T.type && T.type !== 'idle' && T.status === 'active';
    const done = T.type && T.type !== 'idle' && !active ? T.status : null;
    const burst = col => { RS.burstAt = performance.now(); RS.burstCol = col; };
    // 结束瞬间：成功记下到位位置并放一次波纹；失败放红色波纹
    if (done && RS.prev !== done){
      if (done === 'succeeded'){ RS.arrived = { x: R.x, y: R.y }; burst(COL.ok); }
      else if (done === 'aborted'){ RS.arrived = null; burst(COL.fail); }
      else RS.arrived = null;
    }
    // 多点任务：每到一站（站号前进）也闪一次绿
    if (active && T.type === 'route' && RS.point != null && T.point !== RS.point) burst(COL.ok);
    RS.point = active && T.type === 'route' ? T.point : null;
    RS.prev = done || (active ? 'active' : 'idle');
    if (active) RS.arrived = null;
    if (RS.arrived && Math.hypot(R.x - RS.arrived.x, R.y - RS.arrived.y) > ARRIVED_KEEP) RS.arrived = null;
    if (store.estop) return { col: COL.fail, glow: COL.fail };
    if (active && T.phase !== 'checking') return { col: COL.run, glow: COL.run, pulse: true };
    if (done === 'aborted') return { col: COL.fail, glow: COL.fail };
    if (RS.arrived) return { col: COL.ok, glow: COL.ok };
    return actions.canMove() ? { col: COL.idle, glow: COL.idle } : { col: COL.off, glow: COL.idle };
  }
  function drawRobot(){
    const R = store.robot, s = view.s, x = sx(R.x), y = sy(R.y);
    if (on('rings')){ ctx.save(); ctx.strokeStyle = 'rgba(220,235,250,.22)'; ctx.lineWidth = 1; ctx.setLineDash([3,5]);
      for (const r of [1,2,4]){ ctx.beginPath(); ctx.arc(x, y, r*s, 0, 7); ctx.stroke(); }
      ctx.setLineDash([]); ctx.fillStyle = 'rgba(220,235,250,.45)'; ctx.font = '10px "JetBrains Mono",monospace'; ctx.textAlign = 'left';
      for (const r of [2,4]) ctx.fillText(r+' m', x + r*s*.707 + 3, y - r*s*.707 - 3); ctx.restore(); }
    const st = robotState(R), col = st.col;
    ctx.save(); ctx.translate(x, y);
    const rg = Math.max(24, .9*s), pulse = st.pulse ? .65 + .35*Math.sin(t*5) : 1, gl = ctx.createRadialGradient(0,0,0,0,0,rg*(st.pulse ? .9 + .15*Math.sin(t*5) : 1));
    gl.addColorStop(0, rgba(st.glow, .35*pulse)); gl.addColorStop(1, rgba(st.glow, 0));
    ctx.fillStyle = gl; ctx.beginPath(); ctx.arc(0, 0, rg*1.1, 0, 7); ctx.fill();
    // 到位瞬间：绿色波纹扩散两圈
    const age = performance.now() - RS.burstAt;
    if (age < 1600){ ctx.lineWidth = 2.5;
      for (const d of [0, 350]){ const k = (age - d) / 1250; if (k <= 0 || k >= 1) continue;
        ctx.strokeStyle = rgba(RS.burstCol, 1 - k); ctx.beginPath(); ctx.arc(0, 0, rg*(.5 + 1.6*k), 0, 7); ctx.stroke(); } }
    ctx.rotate(-R.th);
    const Lb = Math.max(22, .76*s), Wb = Math.max(14, .48*s);   // nav2.yaml 的 footprint：0.76 × 0.48 m
    ctx.fillStyle = col; for (const [a,b] of [[1,1],[1,-1],[-1,1],[-1,-1]]) ctx.fillRect(a*Lb*.36 - Lb*.1, b*(Wb/2+2) - 2.5, Lb*.2, 5);
    ctx.fillStyle = 'rgba(8,16,26,.92)'; ctx.strokeStyle = col; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.roundRect(-Lb/2, -Wb/2, Lb, Wb, 4); ctx.fill(); ctx.stroke();
    ctx.fillStyle = col; ctx.beginPath(); ctx.moveTo(Lb/2-2, 0); ctx.lineTo(Lb*.1, -Wb*.32); ctx.lineTo(Lb*.1, Wb*.32); ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#ff4fa3'; ctx.fillRect(Lb*.2, -2.5, 5, 5); // MID360
    ctx.restore();
  }
  function drawTF(){
    const ax = (x, y, th, len, lbl) => { const X = sx(x), Y = sy(y); ctx.lineWidth = 2;
      ctx.strokeStyle = '#ff6b6b'; ctx.beginPath(); ctx.moveTo(X,Y); ctx.lineTo(X+Math.cos(-th)*len, Y+Math.sin(-th)*len); ctx.stroke();
      ctx.strokeStyle = '#5be08d'; ctx.beginPath(); ctx.moveTo(X,Y); ctx.lineTo(X+Math.cos(-th-Math.PI/2)*len, Y+Math.sin(-th-Math.PI/2)*len); ctx.stroke();
      ctx.fillStyle = 'rgba(220,235,250,.7)'; ctx.font = '10px "JetBrains Mono",monospace'; ctx.textAlign='left'; ctx.fillText(lbl, X+5, Y+13); };
    ax(0, 0, 0, 30, store.fixed); if (store.robot.valid) ax(store.robot.x, store.robot.y, store.robot.th, 24, 'base_link');
  }

  function draw(dt){
    t += dt;
    if (!store.mapSaving) syncView();
    const R = store.robot, m = store.map, s = view.s;
    if (view.follow && R.valid){ view.cx += (R.x - view.cx)*Math.min(1, dt*4); view.cy += (R.y - view.cy)*Math.min(1, dt*4); }
    ctx.setTransform(DPR,0,0,DPR,0,0); ctx.clearRect(0,0,W,H);
    ctx.fillStyle = 'rgba(120,150,180,.08)';
    if (s > 8){
      const center = rotatePoint({x:view.cx,y:view.cy}, alignment().angle);
      ctx.strokeStyle = 'rgba(140,180,210,.07)'; ctx.lineWidth = 1; ctx.beginPath();
      for (let x=Math.floor(center.x-W/2/s);x<=center.x+W/2/s;x++){
        const px = W/2+(x-center.x)*s; ctx.moveTo(px,0); ctx.lineTo(px,H);
      }
      for (let y=Math.floor(center.y-H/2/s);y<=center.y+H/2/s;y++){
        const py = H/2-(y-center.y)*s; ctx.moveTo(0,py); ctx.lineTo(W,py);
      }
      ctx.stroke();
    }
    ctx.save(); ctx.translate(W/2,H/2); ctx.rotate(-alignment().angle); ctx.translate(-W/2,-H/2);
    ctx.imageSmoothingEnabled = false;
    if (m && on('map')){
      ctx.save(); ctx.shadowColor = 'rgba(56,225,220,.25)'; ctx.shadowBlur = 18; drawGridImg(m, m.img); ctx.restore();
    }
    const cm = store.costmap;
    if (cm && on('gcost')){ ctx.globalAlpha = .55; drawGridImg(cm, cm.img); ctx.globalAlpha = 1; }
    const lc = store.lcostmap;
    if (lc && on('lcost')){
      // 真机：costmap_2d 发的局部代价地图本身，外加边框
      ctx.globalAlpha = .9; drawGridImg(lc, lc.img); ctx.globalAlpha = 1;
      ctx.save(); ctx.translate(sx(lc.ox), sy(lc.oy)); ctx.rotate(-(lc.yaw || 0));
      ctx.strokeStyle = 'rgba(201,100,214,.55)'; ctx.setLineDash([4,4]); ctx.strokeRect(.5, -lc.h*s+.5, lc.w*s, lc.h*s); ctx.restore();
    } else if (cm && on('lcost') && R.valid && store.odomOk){
      // 模拟模式：从全局代价地图裁出机器人周围 5 × 5 m
      const x0 = sx(R.x-2.5), y0 = sy(R.y+2.5), w = 5*s;
      ctx.save(); ctx.beginPath(); ctx.rect(x0, y0, w, w); ctx.clip(); ctx.globalAlpha = .9; drawGridImg(cm, cm.img); ctx.restore();
      ctx.save(); ctx.strokeStyle = 'rgba(201,100,214,.55)'; ctx.setLineDash([4,4]); ctx.strokeRect(x0+.5, y0+.5, w, w); ctx.restore();
    }
    const tr = store.trail;
    if (on('trail') && tr.length > 1){ ctx.strokeStyle = 'rgba(138,160,184,.35)'; ctx.lineWidth = 1.5; ctx.beginPath(); tr.forEach((p,i) => i ? ctx.lineTo(sx(p[0]), sy(p[1])) : ctx.moveTo(sx(p[0]), sy(p[1]))); ctx.stroke(); }
    if (on('cloud')){ const z = Math.max(1.6, .05*s); ctx.globalAlpha = .85;
      store.cloud.forEach((arr, ci) => { ctx.fillStyle = CLOUD[ci]; for (let i=0;i<arr.length;i+=2) ctx.fillRect(sx(arr[i])-z/2, sy(arr[i+1])-z/2, z, z); }); ctx.globalAlpha = 1; }
    const rv = store.reloc?.on ? store.reloc.view() : null;
    if (rv){
      // 重定位：激光按修正后的位姿画，贴墙绿、悬空红（不管激光图层开没开）
      const { pts, ok } = rv;
      for (const [want, col] of [[1, '#35d48a'], [0, '#ff4b5c']]){ ctx.fillStyle = col;
        for (let i=0, k=0; i<pts.length; i+=2, k++) if (ok[k] === want) ctx.fillRect(sx(pts[i])-1.6, sy(pts[i+1])-1.6, 3.2, 3.2); }
    } else if (on('scan')){ const sc = store.scan; ctx.fillStyle = '#fff'; for (let i=0;i<sc.length;i+=2) ctx.fillRect(sx(sc[i])-1.2, sy(sc[i+1])-1.2, 2.4, 2.4); }
    // 小车画在路线下面：路径、局部轨迹、站点、目标点都压在车上，导航时路线更显眼
    if (R.valid) drawRobot();
    const P = store.plan;
    if (P && on('plan')){
      const pts = P.pts, k = Math.min(store.planIdx, pts.length-1);
      ctx.save(); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.strokeStyle = 'rgba(56,225,220,.22)'; ctx.lineWidth = 2; ctx.beginPath(); for (let i=0;i<=k;i++){ const x=sx(pts[i][0]), y=sy(pts[i][1]); i?ctx.lineTo(x,y):ctx.moveTo(x,y); } ctx.stroke();
      ctx.strokeStyle = ACC; ctx.lineWidth = 2.5; ctx.shadowColor = ACC; ctx.shadowBlur = 10;
      ctx.beginPath(); for (let i=k;i<pts.length;i++){ const x=sx(pts[i][0]), y=sy(pts[i][1]); i===k?ctx.moveTo(x,y):ctx.lineTo(x,y); } ctx.stroke(); ctx.restore();
    }
    if (on('mppi') && store.mppi.length){ ctx.strokeStyle = 'rgba(178,140,255,.22)'; ctx.lineWidth = 1; for (const mm of store.mppi){ polyline(mm); ctx.stroke(); } }
    if (on('local') && store.localPlan){ ctx.save(); ctx.strokeStyle = TRAJ; ctx.lineWidth = 3; ctx.lineCap = 'round'; ctx.shadowColor = TRAJ; ctx.shadowBlur = 8; polyline(store.localPlan); ctx.stroke(); ctx.restore(); }
    if (on('particles')){ const pa = store.particles; ctx.strokeStyle = 'rgba(255,224,138,.7)'; ctx.lineWidth = 1; ctx.beginPath();
      for (let i=0;i<pa.length;i+=3){ const x = sx(pa[i]), y = sy(pa[i+1]), a = pa[i+2]; ctx.moveTo(x,y); ctx.lineTo(x+Math.cos(-a)*7, y+Math.sin(-a)*7); } ctx.stroke(); }
    const draft = store.routeDraft;
    if (draft.map && draft.map === store.mapCurrent && draft.points.length){
      const check = store.routeCheck;
      if (check?.draft === draft && check.paths?.length){
        ctx.save();
        ctx.strokeStyle = '#94e8a8';
        ctx.lineWidth = 2;
        for (const leg of check.paths){
          ctx.beginPath();
          leg.points.forEach(([x, y], i) => i ? ctx.lineTo(sx(x), sy(y)) : ctx.moveTo(sx(x), sy(y)));
          ctx.stroke();
        }
        ctx.restore();
      }
      draft.points.forEach((p, i) => drawPin(p, String(i + 1),
        draft.confirmed && i === draft.points.length - 1 ? '#94e8a8' : '#ffe08a', draft.confirmed || p.manualHeading !== false));
    }
    if (store.goal) drawGoal(store.goal);
    if (store.reloc?.on) drawRelocGhost();
    if (on('tf')) drawTF();
    const wall = alignment().points;
    if (wall.length){
      ctx.save(); ctx.strokeStyle = ctx.fillStyle = '#ffe08a'; ctx.lineWidth = 2;
      ctx.beginPath(); wall.forEach((p,i)=>i ? ctx.lineTo(sx(p.x),sy(p.y)) : ctx.moveTo(sx(p.x),sy(p.y))); ctx.stroke();
      for (const p of wall){ ctx.beginPath(); ctx.arc(sx(p.x),sy(p.y),5,0,2*Math.PI); ctx.fill(); }
      ctx.restore();
    }
    const ms = V.measure;
    if (ms.length){ ctx.save(); ctx.strokeStyle = '#ffe08a'; ctx.fillStyle = '#ffe08a'; ctx.lineWidth = 1.5; ctx.setLineDash([5,4]);
      const a = ms[0], b = ms[1] || ms[0];
      ctx.beginPath(); ctx.moveTo(sx(a.x), sy(a.y)); ctx.lineTo(sx(b.x), sy(b.y)); ctx.stroke(); ctx.setLineDash([]);
      for (const p of ms){ ctx.beginPath(); ctx.arc(sx(p.x), sy(p.y), 3.5, 0, 7); ctx.fill(); }
      if (ms[1]){ const d = Math.hypot(b.x-a.x, b.y-a.y), mx = (sx(a.x)+sx(b.x))/2, my = (sy(a.y)+sy(b.y))/2, txt = d.toFixed(2)+' m';
        ctx.font = '500 12px "JetBrains Mono",monospace'; const tw = ctx.measureText(txt).width + 14;
        ctx.fillStyle = 'rgba(6,11,18,.9)'; ctx.beginPath(); ctx.roundRect(mx-tw/2, my-24, tw, 20, 5); ctx.fill();
        ctx.fillStyle = '#ffe08a'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(txt, mx, my-14); }
      ctx.restore(); }
    const d = V.drag && V.drag.moved && (V.drag.tool === 'route' || POSE_TOOLS[V.drag.tool]) ? V.drag : V.aim;
    if (d){
      const col = d.tool==='init' ? '#6fb6ff' : '#ffe08a';
      const X0 = sx(d.x0), Y0 = sy(d.y0), dx = sx(d.x1)-X0, dy = sy(d.y1)-Y0, len = Math.hypot(dx, dy);
      // 箭头至少 30 px 长；鼠标还没离开起点时先朝右画
      const X1 = len < 1 ? X0+30 : X0 + dx*Math.max(1, 30/len), Y1 = len < 1 ? Y0 : Y0 + dy*Math.max(1, 30/len);
      drawArrow(X0, Y0, X1, Y1, col);
      const a = Math.round(Math.atan2(-(Y1-Y0), X1-X0)*180/Math.PI);
      ctx.save(); ctx.font = '500 11px "JetBrains Mono",monospace'; const txt = `θ ${a}°`, tw = ctx.measureText(txt).width + 12;
      ctx.fillStyle = 'rgba(6,11,18,.9)'; ctx.beginPath(); ctx.roundRect(X0+10, Y0+10, tw, 18, 4); ctx.fill();
      ctx.fillStyle = col; ctx.textBaseline = 'middle'; ctx.fillText(txt, X0+16, Y0+19); ctx.restore();
    }
    ctx.restore();
  }

  /* ---------- 工具与指针 ---------- */
  // 数字键 1–5 按工具栏从左到右的顺序
  const TOOLS = ['pan','init','route','measure','align'];
  function setTool(tool, onTeleop){
    if ((store.mapSaving || store.mapBusy) && tool === 'align') return;
    if (tool === 'align'){
      if (!store.map) return toast('地图尚未加载', 'warn');
      // 重新选墙：只清掉上次选的两个点，不动这张地图已保存的显示方向
      Object.assign(alignment(), { points:[], target:null, source:null });
    }
    if (tool==='teleop' && V.tool!=='teleop' && onTeleop && !onTeleop()) return;
    V.tool = tool; $$('[data-tool]').forEach(b => b.classList.toggle('on', b.dataset.tool===tool));
    $('#teleHint').hidden = tool!=='teleop'; if (tool!=='measure') V.measure = [];
    V.aim = null; showHint();
    cv.style.cursor = tool==='pan' ? 'grab' : tool==='teleop' ? 'default' : 'crosshair';
  }
  // 拖动 / 瞄准的锚点都用世界坐标（米）记：视角在跟随机器人移动时，箭头仍钉在点下的位置
  function commit(tool, x, y, x1, y1){
    const th = Math.atan2(y1-y, x1-x);
    V.aim = null; showHint();
    if (!inMap(x, y)) return toast('位置在地图范围之外', 'err');
    const p = { x, y, th };
    if (tool==='init') actions.initialPose(p);
  }
  const local = e => { const r = cv.getBoundingClientRect(); return [e.clientX-r.left, e.clientY-r.top]; };
  const stopFollow = () => { touched = true; view.follow = false; $('#vFollow').classList.remove('on'); };
  // 旋转：绕画面中心转，指针绕中心转多少地图就转多少
  const pointerAngle = (px, py) => Math.atan2(-(py - H/2), px - W/2);
  cv.addEventListener('contextmenu', e => e.preventDefault());   // 右键拖动用来旋转
  cv.addEventListener('pointerdown', e => {
    if (e.button === 2 || (e.button === 0 && e.shiftKey)){
      const [px, py] = local(e); cv.setPointerCapture(e.pointerId);
      V.drag = { tool:'rot', prev: pointerAngle(px, py), turned: 0, ang0: alignment().angle, start: alignment().angle };
      cv.style.cursor = 'grabbing'; return;
    }
    if (e.button !== 0 && e.button !== 1) return;
    const [px, py] = local(e); cv.setPointerCapture(e.pointerId);
    const RP = e.button === 0 && V.tool === 'pan' && store.reloc?.on ? store.reloc.pose() : null;
    if (RP){
      const {x,y} = world(px,py), kl = knobLen();
      if (Math.hypot(x - (RP.x + Math.cos(RP.th)*kl), y - (RP.y + Math.sin(RP.th)*kl)) < 14/view.s){ V.drag = { tool:'relocRot' }; cv.style.cursor = 'grabbing'; return; }
      if (Math.hypot(x - RP.x, y - RP.y) < Math.max(.45, 16/view.s)){ V.drag = { tool:'relocMove', dx: RP.x - x, dy: RP.y - y }; cv.style.cursor = 'grabbing'; return; }
    }
    if (V.tool==='pan' || V.tool==='teleop' || e.button===1){ V.drag = { tool:'pan', px, py, cx:view.cx, cy:view.cy }; cv.style.cursor = 'grabbing'; stopFollow(); return; }
    const {x,y} = world(px,py);
    if (V.tool === 'align'){
      if (!inMap(x,y)) return toast('请在地图范围内选墙', 'warn');
      const points = alignment().points;
      if (points.length && Math.hypot(x-points[0].x,y-points[0].y) < .5) return toast('墙段太短，请选取至少 0.5 米', 'warn');
      points.push({x,y});
      if (points.length === 2){
        const horizontal = wallAngle(points,'horizontal'), vertical = wallAngle(points,'vertical');
        alignWall(Math.abs(horizontal)<=Math.abs(vertical) ? 'horizontal' : 'vertical');
        setTool('pan');
      } else showHint();
      return;
    }
    if (V.tool==='measure'){ const p = {x,y}; V.measure = V.measure.length>=2 ? [p] : [...V.measure, p]; return; }
    if (V.aim){ const a = V.aim; return commit(a.tool, a.x0, a.y0, x, y); } // 第二下：确定朝向
    V.drag = { tool:V.tool, x0:x, y0:y, x1:x, y1:y, px, py, moved:false };
  });
  cv.addEventListener('pointermove', e => {
    const [px, py] = local(e), {x,y} = world(px,py);
    const c = store.costAt && inMap(x, y) ? store.costAt(x, y) : '--';
    $('#cursor').innerHTML = inMap(x, y) ? `x <b>${x.toFixed(2)}</b>  y <b>${y.toFixed(2)}</b>  cost <b>${c}</b>` : 'x <b>--</b>  y <b>--</b>  cost <b>--</b>';
    if (V.aim){ V.aim.x1 = x; V.aim.y1 = y; }
    const d = V.drag; if (!d) return;
    if (d.tool === 'relocMove' || d.tool === 'relocRot'){
      const D = store.reloc.begin(); if (!D) return;
      if (d.tool === 'relocMove'){ D.x = x + d.dx; D.y = y + d.dy; } else D.th = Math.atan2(y - D.y, x - D.x);
      store.reloc.render(); return;
    }
    if (d.tool === 'rot'){
      const cur = pointerAngle(px, py);
      if (Math.hypot(px - W/2, py - H/2) < 24){ d.prev = cur; return; }   // 太靠近中心，角度不稳，先不算
      d.turned += wrapPi(cur - d.prev); d.prev = cur;
      let a = d.ang0 + d.turned;
      const q = Math.round(a / QUARTER) * QUARTER;
      if (!e.altKey && Math.abs(a - q) < SNAP) a = q;   // 靠近 0° / 90° / 180° / 270° 时吸附
      setAngle(a); return;
    }
    if (d.tool==='pan'){
      const delta = rotatePoint({x:(px-d.px)/view.s,y:-(py-d.py)/view.s}, -alignment().angle);
      view.cx = d.cx-delta.x; view.cy = d.cy-delta.y; return;
    }
    d.x1 = x; d.y1 = y; if (Math.hypot(px-d.px, py-d.py) > 6) d.moved = true;
  });
  cv.addEventListener('pointerup', () => {
    const d = V.drag; if (!d) return; V.drag = null;
    if (d.tool === 'relocMove' || d.tool === 'relocRot'){ cv.style.cursor = 'grab'; return; }
    if (d.tool === 'rot'){
      cv.style.cursor = V.tool==='pan' ? 'grab' : 'default';
      if (Math.abs(wrapPi(alignment().angle - d.start)) > 1e-4) setAngle(alignment().angle, true);
      return;
    }
    if (d.tool==='pan'){ cv.style.cursor = V.tool==='pan' ? 'grab' : 'default'; return; }
    if (d.tool === 'route'){
      if (!inMap(d.x0, d.y0)) return toast('位置在地图范围之外', 'err');
      const th = d.moved ? Math.atan2(d.y1 - d.y0, d.x1 - d.x0) : 0;
      return actions.routePoint({ x:d.x0, y:d.y0, th, manualHeading:d.moved });
    }
    if (d.moved) return commit(d.tool, d.x0, d.y0, d.x1, d.y1);           // 按住拖动
    if (!inMap(d.x0, d.y0)) return toast('位置在地图范围之外', 'err');
    V.aim = { tool: d.tool, x0: d.x0, y0: d.y0, x1: d.x0, y1: d.y0 }; showHint(); // 点了一下：等第二下
  });
  cv.addEventListener('wheel', e => { e.preventDefault(); touched = true; const [px, py] = local(e), before = world(px,py);
    view.s = clamp(view.s * Math.exp(-e.deltaY*.0015), 8, 400);
    const after = world(px,py); view.cx += before.x-after.x; view.cy += before.y-after.y;
  }, { passive:false });

  $('#vFollow').onclick = () => { touched = true; view.follow = !view.follow; $('#vFollow').classList.toggle('on', view.follow); if (view.follow && view.s < 50) view.s = 60; };
  $('#vFit').onclick = () => { stopFollow(); touched = false; fit(); };
  // 指南针：单击顺时针转 90°，双击复位（单击要等一下，看是不是双击）
  let rotClick = null;
  $('#vRot').onclick = () => {
    if (rotClick){ clearTimeout(rotClick); rotClick = null; return; }
    rotClick = setTimeout(() => { rotClick = null; setAngle(alignment().angle - QUARTER, true); }, 240);
  };
  $('#vRot').ondblclick = () => { clearTimeout(rotClick); rotClick = null; resetAlignment(); };
  $('#vIn').onclick = () => { touched = true; view.s = clamp(view.s*1.3, 8, 400); };
  $('#vOut').onclick = () => { touched = true; view.s = clamp(view.s/1.3, 8, 400); };

  $('#hudToggle').onclick = () => $('#hudTl').classList.toggle('mini');

  resize(); fit(); syncCompass();
  new ResizeObserver(() => { resize(); if (!touched) fit(); }).observe(stage);
  // 屏幕方向（像素，y 向下）→ 地图里的单位向量：地图可能转了显示角度
  const screenDir = (dx, dy) => rotatePoint({ x: dx, y: -dy }, -alignment().angle);
  return { draw, fit, resize, setTool, resetAlignment, alignWall, screenDir, TOOLS, view, get tool(){ return V.tool; } };
}
