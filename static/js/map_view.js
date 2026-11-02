// 中间的地图画布：渲染各图层 + 地图工具（平移、初始位姿、目标、A/B 点、测距）
import { $, $$, clamp, ACC, TRAJ, CLOUD } from './util.js';
import { on } from './layers.js';
import { toast } from './ui.js';

export function createMapView(store, actions){
  const cv = $('#map'), ctx = cv.getContext('2d'), stage = $('#stage');
  let W = 0, H = 0, DPR = 1, t = 0;
  const view = { cx: 12, cy: 8, s: 40, follow: false };
  let touched = false; // 用户动过视角后，窗口尺寸变化不再自动适配
  // aim：点了一下位置、还在等第二下确定朝向（点两下 = 按住拖动的替代，触控板更好用）
  const V = { tool: 'pan', measure: [], drag: null, aim: null };
  const POSE_TOOLS = { init: '初始位姿', goal: '导航目标', A: '巡逻点 A', B: '巡逻点 B' };
  const hint = $('#aimHint');
  function showHint(){
    const name = POSE_TOOLS[V.tool];
    hint.hidden = !name;
    if (name) hint.innerHTML = V.aim ? `<b>${name}</b> · 移动鼠标选择朝向，再点一下确定 · Esc 取消` : `<b>${name}</b> · 点一下选位置，再点一下定朝向（也可以按住拖动）`;
  }

  const sx = x => W/2 + (x-view.cx)*view.s, sy = y => H/2 - (y-view.cy)*view.s;
  const wx = px => (px-W/2)/view.s + view.cx, wy = py => view.cy - (py-H/2)/view.s;
  function resize(){ const r = stage.getBoundingClientRect(); DPR = window.devicePixelRatio||1; W = r.width; H = r.height; cv.width = Math.round(W*DPR); cv.height = Math.round(H*DPR); }
  function fit(){
    const m = store.map; if (!m || !W) return;
    view.s = Math.min(W/m.w, (H-60)/m.h)*.94; view.cx = m.ox + m.w/2; view.cy = m.oy + m.h/2 - 10/view.s;
  }
  const inMap = (x, y) => { const m = store.map; return m && x>=m.ox && y>=m.oy && x<m.ox+m.w && y<m.oy+m.h; };
  // 栅格图像：原点 (ox, oy) 在左下角，可以带 yaw（真机的局部代价地图在 odom 下，转到 map 后是斜的）
  function drawGridImg(m, img){
    if (!m.yaw) return ctx.drawImage(img, sx(m.ox), sy(m.oy+m.h), m.w*view.s, m.h*view.s);
    ctx.save(); ctx.translate(sx(m.ox), sy(m.oy)); ctx.rotate(-m.yaw);
    ctx.drawImage(img, 0, -m.h*view.s, m.w*view.s, m.h*view.s); ctx.restore();
  }

  function polyline(arr, step = 2){ ctx.beginPath(); for (let i=0;i<arr.length;i+=step){ const x = sx(arr[i]), y = sy(arr[i+1]); i ? ctx.lineTo(x,y) : ctx.moveTo(x,y); } }
  function drawPin(p, label, col){
    const x = sx(p.x), y = sy(p.y);
    ctx.save(); ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 1.5;
    ctx.globalAlpha = .25; ctx.beginPath(); ctx.arc(x, y, 10, 0, 7); ctx.fill(); ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.fill();
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(-p.th)*20, y + Math.sin(-p.th)*20); ctx.stroke();
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
  function drawRobot(){
    const R = store.robot, s = view.s, x = sx(R.x), y = sy(R.y);
    if (on('rings')){ ctx.save(); ctx.strokeStyle = 'rgba(220,235,250,.22)'; ctx.lineWidth = 1; ctx.setLineDash([3,5]);
      for (const r of [1,2,4]){ ctx.beginPath(); ctx.arc(x, y, r*s, 0, 7); ctx.stroke(); }
      ctx.setLineDash([]); ctx.fillStyle = 'rgba(220,235,250,.45)'; ctx.font = '10px "JetBrains Mono",monospace'; ctx.textAlign = 'left';
      for (const r of [2,4]) ctx.fillText(r+' m', x + r*s*.707 + 3, y - r*s*.707 - 3); ctx.restore(); }
    const red = store.estop, col = red ? '#ff4b5c' : actions.canMove() ? ACC : '#9db0c7';
    ctx.save(); ctx.translate(x, y);
    const rg = Math.max(24, .9*s), gl = ctx.createRadialGradient(0,0,0,0,0,rg);
    gl.addColorStop(0, red ? 'rgba(255,75,92,.35)' : 'rgba(56,225,220,.3)'); gl.addColorStop(1, 'rgba(56,225,220,0)');
    ctx.fillStyle = gl; ctx.beginPath(); ctx.arc(0, 0, rg, 0, 7); ctx.fill();
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
    const R = store.robot, m = store.map, s = view.s;
    if (view.follow && R.valid){ view.cx += (R.x - view.cx)*Math.min(1, dt*4); view.cy += (R.y - view.cy)*Math.min(1, dt*4); }
    ctx.setTransform(DPR,0,0,DPR,0,0); ctx.clearRect(0,0,W,H);
    ctx.fillStyle = 'rgba(120,150,180,.08)';
    if (s > 8){ const x0 = Math.floor(wx(0)), x1 = Math.ceil(wx(W)), y0 = Math.floor(wy(H)), y1 = Math.ceil(wy(0));
      for (let x=x0;x<=x1;x++) for (let y=y0;y<=y1;y++) ctx.fillRect(sx(x)-1, sy(y)-1, 2, 2); }
    ctx.imageSmoothingEnabled = false;
    if (m && on('map')){
      ctx.save(); ctx.shadowColor = 'rgba(56,225,220,.25)'; ctx.shadowBlur = 18; drawGridImg(m, m.img); ctx.restore();
      if (s > 14){ ctx.strokeStyle = 'rgba(140,180,210,.05)'; ctx.lineWidth = 1; ctx.beginPath();
        for (let x=Math.ceil(m.ox);x<=m.ox+m.w;x++){ ctx.moveTo(sx(x)+.5, sy(m.oy)); ctx.lineTo(sx(x)+.5, sy(m.oy+m.h)); }
        for (let y=Math.ceil(m.oy);y<=m.oy+m.h;y++){ ctx.moveTo(sx(m.ox), sy(y)+.5); ctx.lineTo(sx(m.ox+m.w), sy(y)+.5); } ctx.stroke(); }
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
    if (on('scan')){ const sc = store.scan; ctx.fillStyle = '#fff'; for (let i=0;i<sc.length;i+=2) ctx.fillRect(sx(sc[i])-1.2, sy(sc[i+1])-1.2, 2.4, 2.4); }
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
    if (store.patrol.A) drawPin(store.patrol.A, 'A', '#f2f6fb');
    if (store.patrol.B) drawPin(store.patrol.B, 'B', '#f2f6fb');
    if (store.goal) drawGoal(store.goal);
    if (R.valid) drawRobot();
    if (on('tf')) drawTF();
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
    const d = V.drag && V.drag.moved && V.drag.tool!=='pan' ? V.drag : V.aim;
    if (d){
      const col = d.tool==='init' ? '#6fb6ff' : d.tool==='goal' ? ACC : '#f2f6fb';
      const X0 = sx(d.x0), Y0 = sy(d.y0), dx = sx(d.x1)-X0, dy = sy(d.y1)-Y0, len = Math.hypot(dx, dy);
      // 箭头至少 30 px 长；鼠标还没离开起点时先朝右画
      const X1 = len < 1 ? X0+30 : X0 + dx*Math.max(1, 30/len), Y1 = len < 1 ? Y0 : Y0 + dy*Math.max(1, 30/len);
      drawArrow(X0, Y0, X1, Y1, col);
      const a = Math.round(Math.atan2(-(Y1-Y0), X1-X0)*180/Math.PI);
      ctx.save(); ctx.font = '500 11px "JetBrains Mono",monospace'; const txt = `θ ${a}°`, tw = ctx.measureText(txt).width + 12;
      ctx.fillStyle = 'rgba(6,11,18,.9)'; ctx.beginPath(); ctx.roundRect(X0+10, Y0+10, tw, 18, 4); ctx.fill();
      ctx.fillStyle = col; ctx.textBaseline = 'middle'; ctx.fillText(txt, X0+16, Y0+19); ctx.restore();
    }
  }

  /* ---------- 工具与指针 ---------- */
  const TOOLS = ['pan','init','goal','A','B','measure','teleop'];
  function setTool(tool, onTeleop){
    if (tool==='teleop' && V.tool!=='teleop' && onTeleop && !onTeleop()) return;
    V.tool = tool; $$('.tool').forEach(b => b.classList.toggle('on', b.dataset.tool===tool));
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
    else if (tool==='goal') actions.goal(p);
    else actions.patrolPoint(tool, p);
  }
  const local = e => { const r = cv.getBoundingClientRect(); return [e.clientX-r.left, e.clientY-r.top]; };
  const stopFollow = () => { touched = true; view.follow = false; $('#vFollow').classList.remove('on'); };
  cv.addEventListener('pointerdown', e => {
    const [px, py] = local(e); cv.setPointerCapture(e.pointerId);
    if (V.tool==='pan' || V.tool==='teleop' || e.button===1){ V.drag = { tool:'pan', px, py, cx:view.cx, cy:view.cy }; cv.style.cursor = 'grabbing'; stopFollow(); return; }
    if (V.tool==='measure'){ const p = { x:wx(px), y:wy(py) }; V.measure = V.measure.length>=2 ? [p] : [...V.measure, p]; return; }
    const x = wx(px), y = wy(py);
    if (V.aim){ const a = V.aim; return commit(a.tool, a.x0, a.y0, x, y); } // 第二下：确定朝向
    V.drag = { tool:V.tool, x0:x, y0:y, x1:x, y1:y, px, py, moved:false };
  });
  cv.addEventListener('pointermove', e => {
    const [px, py] = local(e), x = wx(px), y = wy(py);
    const c = store.costAt && inMap(x, y) ? store.costAt(x, y) : '--';
    $('#cursor').innerHTML = inMap(x, y) ? `x <b>${x.toFixed(2)}</b>  y <b>${y.toFixed(2)}</b>  cost <b>${c}</b>` : 'x <b>--</b>  y <b>--</b>  cost <b>--</b>';
    if (V.aim){ V.aim.x1 = x; V.aim.y1 = y; }
    const d = V.drag; if (!d) return;
    if (d.tool==='pan'){ view.cx = d.cx - (px-d.px)/view.s; view.cy = d.cy + (py-d.py)/view.s; return; }
    d.x1 = x; d.y1 = y; if (Math.hypot(px-d.px, py-d.py) > 6) d.moved = true;
  });
  cv.addEventListener('pointerup', () => {
    const d = V.drag; if (!d) return; V.drag = null;
    if (d.tool==='pan'){ cv.style.cursor = V.tool==='pan' ? 'grab' : 'default'; return; }
    if (d.moved) return commit(d.tool, d.x0, d.y0, d.x1, d.y1);           // 按住拖动
    if (!inMap(d.x0, d.y0)) return toast('位置在地图范围之外', 'err');
    V.aim = { tool: d.tool, x0: d.x0, y0: d.y0, x1: d.x0, y1: d.y0 }; showHint(); // 点了一下：等第二下
  });
  cv.addEventListener('wheel', e => { e.preventDefault(); touched = true; const [px, py] = local(e), x = wx(px), y = wy(py);
    view.s = clamp(view.s * Math.exp(-e.deltaY*.0015), 8, 400); view.cx = x - (px-W/2)/view.s; view.cy = y + (py-H/2)/view.s; }, { passive:false });

  $('#vFollow').onclick = () => { touched = true; view.follow = !view.follow; $('#vFollow').classList.toggle('on', view.follow); if (view.follow && view.s < 50) view.s = 60; };
  $('#vFit').onclick = () => { stopFollow(); touched = false; fit(); };
  $('#vIn').onclick = () => { touched = true; view.s = clamp(view.s*1.3, 8, 400); };
  $('#vOut').onclick = () => { touched = true; view.s = clamp(view.s/1.3, 8, 400); };

  $('#hudToggle').onclick = () => $('#hudTl').classList.toggle('mini');

  resize(); fit();
  new ResizeObserver(() => { resize(); if (!touched) fit(); }).observe(stage);
  return { draw, fit, resize, setTool, TOOLS, view, get tool(){ return V.tool; } };
}
