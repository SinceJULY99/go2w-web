// 左右两栏、顶栏、HUD 和底部抽屉：全部从 store 读，操作交给 actions
import { $, $$, clamp, deg, fmt, fmtDur, tstr, ACC, TRAJ } from './util.js';
import { LAYERS, L } from './layers.js';
import { MODES, MODE_PKG } from './actions.js';

// 内容没变就不动 DOM：高频重建按钮会吞掉正在进行的点击
const setHTML = (sel, html) => { const el = $(sel); if (el._html !== html){ el._html = html; el.innerHTML = html; } };

export function createPanels(store, src, actions, mapView, bus){
  const ros = src.name === 'ros';
  const run = id => store.profiles[id] === 'run';
  const now = () => src.now();

  /* ---------- 日志 ---------- */
  const logs = []; let logFilter = 'ALL';
  const logEl = $('#pLogs');
  function appendLog(l){
    const r = document.createElement('div'); r.className = 'lrow ' + l.level;
    r.innerHTML = `<span class="lt">${l.t}</span><span class="lv ${l.level}">${l.level}</span><span class="ln"></span><span class="lm"></span>`;
    r.children[2].textContent = `[${l.node}]`; r.lastChild.textContent = l.msg; logEl.appendChild(r);
    while (logEl.children.length > 300) logEl.firstChild.remove();
    logEl.scrollTop = logEl.scrollHeight;
  }
  bus.on('log', (level, node, msg) => {
    const l = { t: tstr(), level, node, msg }; logs.push(l); if (logs.length > 400) logs.shift();
    if (logFilter==='ALL' || logFilter===level) appendLog(l);
  });
  $('#logFilters').onclick = e => { const b = e.target.closest('[data-lv]'); if (!b) return; logFilter = b.dataset.lv;
    $$('#logFilters .fchip').forEach(x => x.classList.toggle('on', x===b));
    logEl.textContent = ''; logs.filter(l => logFilter==='ALL'||l.level===logFilter).forEach(appendLog); };

  /* ---------- 图层 ---------- */
  // 图层开关记在本机；真机模式下告诉后端打开了哪些重图层（点云、代价地图、粒子没人看就不算不发）
  try { const s = JSON.parse(localStorage.getItem('go2w.layers')); if (s) for (const l of LAYERS) if (typeof s[l.id] === 'boolean') l.on = s[l.id]; } catch {}
  function renderLayers(){
    $('#layers').innerHTML = LAYERS.map(l => { const na = ros && !l.ros, topic = ros ? (l.ros || l.topic) : l.topic;
      return `<div class="layer ${na?'na':''}" title="${na ? l.note : ''}"><span class="sw" style="background:${l.c==='grad'?'linear-gradient(135deg,#ff4fa3,#6fb6ff)':l.c}"></span>
      <div class="nm">${l.name}<small>${topic}${na ? ' · 真机上没有' : ''}</small></div>
      <button class="tg" role="switch" aria-label="${l.name}" aria-checked="${l.on}" data-l="${l.id}"></button></div>`; }).join('');
    $('#layerCount').textContent = `${LAYERS.filter(l => l.on).length}/${LAYERS.length}`;
    if (src.setLayers) src.setLayers(LAYERS.filter(l => l.on && l.heavy).map(l => l.id));
  }
  $('#layers').addEventListener('click', e => { const b = e.target.closest('[data-l]'); if (!b) return;
    const l = L[b.dataset.l];
    if (ros && !l.ros && !l.on) bus.emit('toast', l.note, 'warn');
    l.on = !l.on; renderLayers();
    try { localStorage.setItem('go2w.layers', JSON.stringify(Object.fromEntries(LAYERS.map(x => [x.id, x.on])))); } catch {} });

  /* ---------- topic 列表 + 检查器 ---------- */
  function renderTopics(){
    const q = $('#topicSearch').value.trim().toLowerCase();
    const list = store.topics.filter(t => !q || t.name.toLowerCase().includes(q) || t.type.toLowerCase().includes(q));
    setHTML('#topics', list.map(t => {
      const shown = !t.alive ? '—' : t.hz == null ? '' : t.hz===0 ? 'latched' : `${t.hz} Hz`;
      return `<button class="topic ${t.alive?'':'off'}" data-t="${t.name}"><i></i><span style="min-width:0"><span class="tn">${t.name}${t.d0?'<span class="dom0">domain 0</span>':''}</span><span class="tt">${t.type.replace('/msg/','/')}</span></span><span class="th">${shown}</span></button>`; }).join('')
      || `<div class="empty">${ros && !store.link.ok ? '未连接' : '没有 topic'}</div>`);
    $('#topicCount').textContent = `${store.topics.filter(t => t.alive).length} 在线 / ${store.topics.length}`;
  }
  $('#topicSearch').addEventListener('input', renderTopics);
  $('#topics').addEventListener('click', e => { const b = e.target.closest('[data-t]'); if (b) openInspector(b.dataset.t); });
  let inspN = 0;
  function openInspector(n){
    const t = store.topics.find(x => x.name===n) || { type: '' }, f = document.createElement('div'); f.className = 'float';
    const k = inspN++ % 5; f.style.left = (window.innerWidth/2 - 160 + k*26) + 'px'; f.style.top = (120 + k*26) + 'px';
    f.innerHTML = `<div class="float-h"><b></b><span></span><button class="ib" style="width:24px;height:24px" aria-label="关闭"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M6 6l12 12M18 6L6 18"/></svg></button></div><pre></pre>`;
    f.querySelector('b').textContent = n; f.querySelector('.float-h span').textContent = t.type.split('/').pop();
    document.body.appendChild(f);
    const pre = f.querySelector('pre'), esc = s => String(s).replace(/[&<>]/g, c => ({ '&':'&amp;', '<':'&lt;', '>':'&gt;' }[c]));
    const note = s => `<span class="key">${esc(s)}</span>`;
    let close;
    if (src.watch){
      // 真机：后端按需订阅，最多 2 Hz 推 YAML 文本；关掉面板就取消订阅
      pre.innerHTML = note('（等待消息……）');
      let got = false;
      const unwatch = src.watch(n, m => {
        got = true;
        if (m.err){ pre.innerHTML = note(`（${m.err}）`); return; }
        f.querySelector('.float-h span').textContent = `${(m.type || '').split('/').pop()} · ${m.hz} Hz`;
        // YAML：键名灰色、数字青色
        pre.innerHTML = esc(m.text || '').replace(/^(\s*-?\s*)([\w.]+):/gm, (_, a, k) => `${a}<span class="key">${k}:</span>`)
          .replace(/(:\s|- |\[|, )(-?\d+(\.\d+)?(e-?\d+)?)(?=[\s,\]]|$)/gm, (_, a, num) => `${a}<span class="num">${num}</span>`);
      });
      const quiet = setTimeout(() => { if (!got) pre.innerHTML = note('（5 s 内没有收到消息：发布者可能很久才发一次，比如 latched 的 /map）'); }, 5000);
      close = () => { clearTimeout(quiet); unwatch(); };
    } else {
      const upd = () => {
        const tt = store.topics.find(x => x.name===n);
        if (!tt || !tt.alive){ pre.innerHTML = note('（没有发布者：相关进程没有运行）'); return; }
        const lines = src.inspect(n);
        pre.innerHTML = lines ? lines.map(([k, v]) => `<span class="key">${esc(k)}:</span> ${typeof v==='number' ? `<span class="num">${v}</span>` : esc(v)}`).join('\n')
          : note('（模拟模式没有模拟这个 topic 的内容）');
      };
      upd(); const iv = setInterval(upd, 200);
      close = () => clearInterval(iv);
    }
    f.querySelector('button').onclick = () => { close(); f.remove(); };
    const h = f.querySelector('.float-h');
    h.addEventListener('pointerdown', e => { if (e.target.closest('button')) return; const ox = e.clientX - f.offsetLeft, oy = e.clientY - f.offsetTop; h.setPointerCapture(e.pointerId);
      const mv = ev => { f.style.left = clamp(ev.clientX-ox, 0, window.innerWidth-60)+'px'; f.style.top = clamp(ev.clientY-oy, 0, window.innerHeight-40)+'px'; };
      const up = () => { h.removeEventListener('pointermove', mv); h.removeEventListener('pointerup', up); };
      h.addEventListener('pointermove', mv); h.addEventListener('pointerup', up); });
  }

  /* ---------- 运行档位 ---------- */
  const OVERLAY = [['patrol', '巡逻', 'patrol.py'], ['bag', '录包', 'ros2 bag record']];
  const uptime = id => run(id) && store.since[id] != null ? fmtDur(now() - store.since[id]) : '';
  function renderProfiles(){
    setHTML('#modes', Object.entries(MODES).map(([id, name]) => { const st = store.profiles[id];
      const meta = st==='run' ? uptime(id) : st==='starting' ? '启动中…' : st==='stopping' ? '停止中…' : MODE_PKG[id];
      return `<button class="mode" data-s="${st}" data-mode="${id}"><b><i></i>${name}</b><small>${meta}</small></button>`; }).join(''));
    setHTML('#procs', OVERLAY.map(([id, name, pkg]) => { const st = store.profiles[id], busy = st==='starting'||st==='stopping';
      const meta = st==='run' ? uptime(id) : st==='starting' ? '启动中…' : st==='stopping' ? '停止中…' : '未运行';
      return `<div class="proc" data-s="${st}"><i></i><div class="pn">${name}<small>${pkg}</small></div><div class="pm">${meta}</div>
        <button class="btn sm ${st==='run'?'ghost':''}" data-p="${id}" ${busy?'disabled':''}>${st==='run'?'停止':'启动'}</button></div>`; }).join(''));
    setHTML('#nodes', store.nodes.map(n => `<div class="node ${n.up?'up':''}"><i></i><span>${n.name}</span></div>`).join(''));
    $('#modeStop').disabled = !actions.curMode();
  }
  $('#modes').onclick = e => { const b = e.target.closest('[data-mode]'); if (b) actions.setMode(b.dataset.mode); };
  $('#procs').onclick = e => { const b = e.target.closest('[data-p]'); if (b) actions.toggle(b.dataset.p); };
  $('#modeStop').onclick = () => actions.setMode(null);
  $('#simDrop').onclick = () => actions.simulateDrop();

  /* ---------- 任务卡片 ---------- */
  // 步骤由数据源给出：模拟器能看到所有内部阶段，真机只能从 Nav2 行为树日志里看到规划 / 跟随 / 恢复
  const PHASES = src.steps.map(s => s[0]);
  $('#steps').innerHTML = src.steps.map(([p, name, note]) => `<li data-p="${p}">${name}<small>${note}</small></li>`).join('');
  if (!src.can('pause')){ $('#btnPause').title = '真机暂不支持暂停：可以取消任务，或者锁定底盘'; }
  function renderTask(){
    const T = store.task, idle = T.type==='idle', done = !idle && T.status && T.status!=='active';
    $('#taskBody').hidden = idle; $('#taskEmpty').hidden = !idle;
    let st = 'run', label = '执行中';
    if (idle){ st = ''; label = '空闲'; }
    else if (T.status==='aborted'){ st = 'crit'; label = '失败'; }
    else if (T.status==='canceled'){ st = ''; label = '已取消'; }
    else if (T.type==='patrol' && T.phase==='arrived'){ label = '到点停留'; }
    else if (T.phase==='arrived' || T.status==='succeeded'){ st = 'ok'; label = '已完成'; }
    else if (store.estop){ st = 'crit'; label = '急停'; }
    else if (T.paused){ st = 'warn'; label = '已暂停'; }
    else if (T.phase==='recover'){ st = 'warn'; label = '恢复中'; }
    else if (!actions.canMove() && T.phase!=='plan'){ st = 'warn'; label = !store.armed ? '等待解锁' : '底盘不可用'; }
    const pill = $('#taskPill'); pill.dataset.s = st; pill.textContent = label;
    $('#btnPause').disabled = idle || done || !src.can('pause'); $('#btnPause').textContent = T.paused ? '继续' : '暂停';
    $('#btnCancel').disabled = idle || done;
    $('#btnPatrol').textContent = T.type==='patrol' ? '巡逻中' : '开始巡逻'; $('#btnPatrol').disabled = T.type==='patrol';
    if (idle) return;
    if (T.type==='patrol' && T.legs != null){   // 真机：patrol.py 的进度来自它的日志
      $('#taskTitle').textContent = 'A ↔ B 巡逻';
      $('#taskSub').textContent = `${T.loops ? `往返 ${T.loops} 次` : '一直往返'} · 已完成 ${T.legs} 段${T.leg ? ` · 正在去 ${T.leg} 点` : ''}`;
    }
    else if (T.type==='patrol'){ $('#taskTitle').textContent = 'A ↔ B 巡逻'; $('#taskSub').textContent = `${T.loops >= 1e9 ? `第 ${T.loop} 圈 · 一直往返` : `第 ${T.loop} / ${T.loops} 圈`} · ${T.leg==='B'?'A → B':'B → A'}`; }
    else if (T.type==='external'){ $('#taskTitle').textContent = '外部导航目标'; $('#taskSub').textContent = 'RViz 或 patrol 发出的目标 ·「取消」同样有效'; }
    else { const g = T.goal; $('#taskTitle').textContent = '单点导航'; $('#taskSub').textContent = `目标 (${g.x.toFixed(2)}, ${g.y.toFixed(2)}) · ${deg(g.th).toFixed(0)}°`; }
    const ph = ['dwell','arrived'].includes(T.phase) || T.status==='succeeded' ? 'arrive' : T.phase, rem = T.remaining;
    $('#progBar').style.width = rem!=null && T.total ? (100*clamp(1 - rem/T.total, 0, 1)).toFixed(1)+'%' : '0%';
    $('#tRem').textContent = rem==null ? '--' : rem.toFixed(1)+' m';
    const eta = T.eta != null ? T.eta : rem != null ? rem/Math.max(.2, store.params.vmax*.8) : null;
    $('#tEta').textContent = eta==null || done ? '--' : rem < .05 ? '0 s' : Math.ceil(eta)+' s';
    $('#tEl').textContent = T.elapsed==null ? '--' : fmtDur(T.elapsed); $('#tRec').textContent = T.recoveries || 0;
    const ci = PHASES.indexOf(ph);
    $$('#steps li').forEach((li, i) => { li.className = i<ci ? 'done' : i===ci ? (ph==='arrive' ? 'done' : 'cur') : ''; });
  }
  $('#btnPause').onclick = () => actions.pause();
  $('#btnCancel').onclick = () => actions.cancel();
  $('#btnPatrol').onclick = () => actions.startPatrol();
  // 巡逻往返次数（patrol.py 的 loops 参数，0 = 一直往返）
  const loopsEl = $('#patrolLoops');
  const setLoops = () => { const n = clamp(Math.round(+loopsEl.value || 0), 0, 1000); store.patrolLoops = n; if (!ros) store.patrol.loops = n || 1e9; };
  loopsEl.onchange = () => { setLoops(); loopsEl.value = store.patrolLoops; };
  setLoops();

  /* ---------- 底盘控制 ---------- */
  const ICON_LOCK = '<svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><rect x="5" y="11" width="14" height="9" rx="2"/><path d="M8 11V8a4 4 0 018 0v3"/></svg>';
  const ICON_OPEN = '<svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><rect x="5" y="11" width="14" height="9" rx="2"/><path d="M8 11V8a4 4 0 017.5-2"/></svg>';
  function renderArm(){
    const s = store.estop ? 'estop' : store.armed==null ? 'unknown' : store.armed ? 'armed' : 'locked';
    $('#armBox').dataset.s = s; $('#armIco').innerHTML = s==='armed' ? ICON_OPEN : ICON_LOCK;
    $('#armT').textContent = { armed:'已解锁', estop:'急停锁定', locked:'已锁定', unknown:'状态未知' }[s];
    $('#armS').textContent = { armed:`执行 /cmd_vel · 超时 ${store.params.tmo.toFixed(1)} s 自动停车`, estop:'先解除急停', locked:'不执行任何速度指令', unknown:'cmd_bridge 没有运行' }[s];
    const b = $('#armBtn'); b.textContent = store.armed ? '锁定' : '解锁'; b.className = 'btn sm ' + (store.armed ? '' : 'pri'); b.disabled = store.estop || store.armed==null;
    $('#estopBanner').hidden = !store.estop;
  }
  $('#armBtn').onclick = () => actions.arm();
  $('#estopBtn').onclick = () => actions.estop();
  $('#estopRelease').onclick = () => actions.releaseEstop();
  // 滑条平时跟着实际参数走（真机上来自 cmd_bridge）；拖动中不打断，松手后发设置，失败就弹回实际值
  const sliders = [['#slV','#oV','vmax','m/s',2], ['#slW','#oW','wmax','rad/s',1], ['#slT','#oT','tmo','s',1]].map(([id, out, key, unit, dec]) => {
    const el = $(id), show = v => $(out).textContent = `${(+v).toFixed(dec)} ${unit}`;
    let busy = false;
    el.oninput = () => { busy = true; show(el.value); };
    el.onchange = () => { Promise.resolve(actions.setParam(key, +el.value)).finally(() => { busy = false; }); };
    return () => { if (!busy && store.params[key] != null && +el.value !== store.params[key]){ el.value = store.params[key]; show(el.value); } };
  });
  const syncSliders = () => sliders.forEach(f => f());

  /* ---------- 顶栏 / HUD / 机器人卡片 ---------- */
  function chip(id, s, txt){ const c = $(id); c.dataset.s = s; c.querySelector('b').textContent = txt; }
  function renderChips(){
    const lk = store.link, anyMode = !!actions.curMode();
    chip('#chLink', lk.ok ? (lk.ms==null ? 'warn' : lk.ms>40 ? 'warn' : 'ok') : 'crit', lk.ok ? (lk.ms==null ? '无响应' : `${lk.ms.toFixed(0)} ms`) : (ros ? '连接中' : '断开'));
    const Lc = store.loc;
    if (!Lc.running) chip('#chLoc', run('mapping') ? 'ok' : '', run('mapping') ? '建图' : '未运行');
    else if (Lc.std==null) chip('#chLoc', 'ok', '运行中');
    // 主要看 scan 和地图的匹配率。σ 只用来认「刚设完初始位姿」（此时 σ = 0.5 m）：
    // AMCL 只在机器人移动后才更新，静止时 σ 不变；真机定位正常时也有约 0.15 m（11-02 实测，匹配率 82–90%）
    else if (Lc.std>.3) chip('#chLoc', 'warn', '收敛中');
    else if (Lc.match!=null && Lc.match<60) chip('#chLoc', 'crit', '匹配差');
    else chip('#chLoc', 'ok', '良好');
    chip('#chOdom', store.odomOk ? 'ok' : anyMode ? 'crit' : '', store.odomOk ? '健康' : anyMode ? '不可用' : '未运行');
    chip('#chBase', store.estop ? 'crit' : store.armed==null ? '' : store.armed ? 'ok' : 'warn', store.estop ? '急停' : store.armed==null ? '--' : store.armed ? '已解锁' : '锁定');
    const sy = store.sys;
    chip('#chCpu', sy.cpu==null ? '' : sy.cpu>80 ? 'crit' : sy.cpu>60 ? 'warn' : 'ok', sy.cpu==null ? '--' : `${sy.cpu.toFixed(0)}%`);
    chip('#chMem', sy.mem==null ? '' : sy.mem/sy.memTotal>.85 ? 'warn' : 'ok', sy.mem==null ? '--' : `${sy.mem.toFixed(1)} / ${sy.memTotal} G`);
    $('#chRec').hidden = !run('bag'); if (run('bag')) $('#chRec').querySelector('b').textContent = uptime('bag');
    $('#mapTag').hidden = !run('mapping') || mapView.tool==='teleop';
    const T = store.task;
    const rs = !lk.ok && ros ? ['','离线'] : store.estop ? ['crit','急停'] : mapView.tool==='teleop' ? ['run','遥控中'] : run('mapping') ? ['run','建图中']
      : T.type!=='idle' && T.phase!=='arrived' ? ['run','导航中'] : anyMode ? ['ok','待命'] : ['','未启动'];
    const rp = $('#robotPill'); rp.dataset.s = rs[0]; rp.textContent = rs[1];
    const R = store.robot;
    $('#sBat').textContent = store.battery==null ? '--' : `${store.battery}%`;
    $('#sSpd').textContent = R.valid ? Math.hypot(R.v, R.vy).toFixed(2) : '--';
    $('#sOdo').textContent = store.odo==null ? '--' : `${store.odo.toFixed(0)} m`;
    $('#sFrame').textContent = R.valid ? store.fixed : '--';
    // 三项各自有数据就显示（真机上匹配率来自 scan，σ 来自 /amcl_pose，不一定同时有）
    const locVal = (id, v, dec, unit) => { if (Lc.running && v != null) $(id).innerHTML = `${v.toFixed(dec)}<small>${unit}</small>`; else $(id).textContent = '--'; };
    locVal('#lStd', Lc.std, 3, 'm'); locVal('#lYaw', Lc.yaw, 1, '°'); locVal('#lMatch', Lc.match, 0, '%');
    // 连接 / 等待地图提示
    const off = $('#offline');
    if (ros && !lk.ok){ off.hidden = false; $('#offT').textContent = '正在连接后端…'; $('#offS').textContent = `ws://${location.host}/ws`; }
    else if (ros && !store.map){ off.hidden = false; $('#offT').textContent = '已连接，等待 /map'; $('#offS').textContent = '建图、定位或导航运行后才会有地图'; }
    else off.hidden = true;
  }
  function renderHud(){
    const R = store.robot, m = store.map;
    $('#hMap').textContent = m ? `${m.name} · ${m.cells} · ${+m.res.toFixed(3)} m` : '--';
    $('#hPose').textContent = R.valid ? `x ${fmt(R.x)}  y ${fmt(R.y)}  θ ${fmt(deg(R.th),1)}°` : '--';
    $('#hVel').textContent = R.valid ? `vx ${fmt(R.v)}  vy ${fmt(R.vy)}  wz ${fmt(R.w)}` : '--';
    $('#hFrame').textContent = `${store.fixed} → base_footprint`;
    const m1 = [.5,1,2,5,10].find(c => c*mapView.view.s >= 60) || 10;
    $('#scaleBar').style.width = (m1*mapView.view.s).toFixed(0)+'px'; $('#scaleTxt').textContent = m1+' m';
  }

  /* ---------- 地图与录包 ---------- */
  function renderMaps(){
    // 定位 / 导航在跑：「切换」= map_server 立即加载；没在跑：「选用」= 下次启动用这张
    const live = run('localization') || run('navigation');
    setHTML('#mapList', store.maps.length ? store.maps.map((m,i) => {
      const tag = m.cur ? '<span class="pill" data-s="run">当前</span>' : m.choice ? '<span class="pill" data-s="warn">启动时用</span>'
        : `<button class="btn sm" data-m="${i}">${live || !ros ? '切换' : '选用'}</button>`;
      return `<div class="mrow"><span class="mn">${m.name}</span><span class="mm">${m.meta}</span>${tag}</div>`; }).join('')
      : `<div class="empty">data/maps 里没有地图</div>`);
    setHTML('#bagList', store.bags.length ? store.bags.map(b => `<div class="mrow"><span class="mn">${b.name}</span><span class="mm">${fmtDur(b.dur)}</span><span class="mm">${b.size>1024?(b.size/1024).toFixed(1)+' GB':b.size.toFixed(0)+' MB'}</span></div>`).join('')
      : `<div class="empty">data/bags 里没有录包</div>`);
    const bs = store.profiles.bag;
    $('#bagBtn').textContent = bs==='run' ? '停止录包' : bs==='stop' ? '开始录包' : bs==='starting' ? '启动中…' : '停止中…';
    $('#bagBtn').disabled = bs==='starting' || bs==='stopping';
  }
  $('#mapList').onclick = e => { const b = e.target.closest('[data-m]'); if (b) actions.loadMap(store.maps[+b.dataset.m].name); };
  $('#saveMap').onclick = () => actions.saveMap($('#mapName').value.trim() || 'map');
  // 录包的 topic 列表来自输入框；运行档位里的「录包」行也用它
  store.bagTopics = $('#bagTopics').value;
  $('#bagTopics').oninput = () => { store.bagTopics = $('#bagTopics').value; };
  $('#bagBtn').onclick = () => actions.toggle('bag');

  /* ---------- 曲线 ---------- */
  const hist = { v: [], vo: [], vw: [], w: [], wo: [], ww: [], roll: [], pitch: [], match: [] };
  function pushHist(){
    const p = (a, v) => { a.push(v); if (a.length > 300) a.shift(); }, R = store.robot, c = store.cmd, sc = store.speedCheck;
    p(hist.v, R.valid ? c.v : null); p(hist.vo, R.valid ? R.v : null); p(hist.w, R.valid ? c.w : null); p(hist.wo, R.valid ? R.w : null);
    p(hist.vw, sc ? sc.wall[0] : null); p(hist.ww, sc ? sc.wall[2] : null);
    p(hist.roll, store.imu.roll); p(hist.pitch, store.imu.pitch); p(hist.match, store.loc.running ? store.loc.match : null);
  }
  function chart(canvas, series, opt){
    const r = canvas.getBoundingClientRect(); if (!r.width) return; const d = window.devicePixelRatio||1;
    if (canvas.width !== Math.round(r.width*d) || canvas.height !== Math.round(r.height*d)){ canvas.width = Math.round(r.width*d); canvas.height = Math.round(r.height*d); }
    const c = canvas.getContext('2d'); c.setTransform(d,0,0,d,0,0); c.clearRect(0,0,r.width,r.height);
    const pl = opt.pad ? 44 : 2, pr = opt.pad ? 14 : 2, pb = opt.pad ? 22 : 4;
    // 图例先排版：放不下就换行，图的上边距跟着图例行数走（多图并排时每张图很窄）
    let pt = opt.pad ? 0 : 4; const legend = [];
    if (opt.pad){
      c.font = '10px "JetBrains Mono",monospace';
      let lx = pl, ly = 8;
      for (const s of series){ const iw = 16 + c.measureText(s.name).width;
        if (lx > pl && lx + iw > r.width - pr){ lx = pl; ly += 14; }
        legend.push([s, lx, ly]); lx += iw + 18; }
      pt = ly + 14;
    }
    const w = r.width-pl-pr, h = r.height-pt-pb;
    if (h < 10) return;
    const [lo, hi] = opt.range, Y = v => pt + h - (v-lo)/(hi-lo)*h, n = 300, X = i => pl + i/(n-1)*w;
    if (opt.pad){ c.fillStyle = '#5f718a'; c.strokeStyle = 'rgba(140,170,200,.08)'; c.textAlign = 'right'; c.textBaseline = 'middle';
      for (const tk of opt.ticks){ c.beginPath(); c.moveTo(pl, Y(tk)+.5); c.lineTo(pl+w, Y(tk)+.5); c.stroke(); c.fillText(tk.toFixed(opt.dec??1), pl-6, Y(tk)); }
      c.textAlign = 'center'; c.textBaseline = 'top'; for (const s of [-30,-20,-10,0]) c.fillText(s ? `${s} s` : '现在', pl + (1+s/30)*w, pt+h+6);
      c.textAlign = 'left'; c.textBaseline = 'middle';
      for (const [s, lx, ly] of legend){ c.fillStyle = s.c; c.fillRect(lx, ly-1.5, 12, 3); c.fillStyle = '#9db0c7'; c.fillText(s.name, lx+16, ly); } }
    for (const s of series){ const a = s.d, off = n - a.length; c.beginPath(); let started = false, first = -1, lastI = -1;
      a.forEach((v, i) => { if (v==null){ started = false; return; } const x = X(i+off), y = Y(clamp(v, lo, hi)); started ? c.lineTo(x, y) : c.moveTo(x, y); started = true; if (first<0) first = i; lastI = i; });
      c.strokeStyle = s.c; c.lineWidth = s.w || 1.6; c.setLineDash(s.dash || []); c.stroke(); c.setLineDash([]);
      if (s.fill && first>=0 && lastI===a.length-1){ c.lineTo(X(n-1), pt+h); c.lineTo(X(first+off), pt+h); c.closePath(); const g = c.createLinearGradient(0, pt, 0, pt+h); g.addColorStop(0, s.fill); g.addColorStop(1, 'rgba(0,0,0,0)'); c.fillStyle = g; c.fill(); }
      const lv = a[a.length-1]; if (lv!=null){ c.fillStyle = s.c; c.beginPath(); c.arc(X(n-1), Y(clamp(lv,lo,hi)), 2.8, 0, 7); c.fill(); } }
  }
  // 曲线可多选：选中的按固定顺序并排显示；面板够高时 3 张以上排成两行
  const PLOTS = ['v', 'w', 'tilt', 'loc'];
  const PLOT_NAME = { v:'线速度 vx (m/s)', w:'角速度 wz (rad/s)', tilt:'姿态 roll / pitch (°)', loc:'定位匹配率 (%)' };
  let plotSel = ['v'];
  try { const s = JSON.parse(localStorage.getItem('go2w.plots')); if (Array.isArray(s)) plotSel = s.filter(m => PLOTS.includes(m)); } catch {}
  const plotCanvas = {};
  function layoutPlots(){
    $$('#plotSide [data-m]').forEach(b => b.setAttribute('aria-pressed', String(plotSel.includes(b.dataset.m))));
    const g = $('#plotGrid'), sel = PLOTS.filter(m => plotSel.includes(m));
    g.innerHTML = ''; g.className = sel.length ? 'plot-grid' : 'plot-grid plot-empty';
    if (!sel.length){ g.textContent = '在左边勾选要看的曲线，可以同时选多条'; return; }
    const rows = sel.length >= 3 && g.clientHeight >= 300 ? 2 : 1, cols = Math.ceil(sel.length / rows);
    g.style.gridTemplateColumns = `repeat(${cols}, minmax(0, 1fr))`;
    g.style.gridTemplateRows = `repeat(${rows}, minmax(0, 1fr))`;
    for (const m of sel){
      const cell = document.createElement('div'); cell.className = 'plot-cell';
      cell.innerHTML = `<h5>${PLOT_NAME[m]}</h5><div class="plot-body"><canvas></canvas></div>`;
      g.appendChild(cell); plotCanvas[m] = cell.querySelector('canvas');
    }
    for (const m of PLOTS) if (!sel.includes(m)) delete plotCanvas[m];
  }
  $('#plotSide').onclick = e => { const b = e.target.closest('[data-m]'); if (!b) return; const m = b.dataset.m;
    plotSel = plotSel.includes(m) ? plotSel.filter(x => x!==m) : [...plotSel, m];
    try { localStorage.setItem('go2w.plots', JSON.stringify(plotSel)); } catch {}
    layoutPlots(); renderCharts(); };
  // 面板高度变化时重新判断一行还是两行
  let lastRows = null;
  new ResizeObserver(() => { const rows = plotSel.length >= 3 && $('#plotGrid').clientHeight >= 300 ? 2 : 1; if (rows !== lastRows){ lastRows = rows; layoutPlots(); } }).observe($('#plotGrid'));
  layoutPlots();
  function renderCharts(){
    chart($('#spark'), [{ d: hist.match, c: ACC, fill: 'rgba(56,225,220,.22)' }], { range: [30, 100] });
    if ($('#pPlots').hidden || $('#drawer').classList.contains('collapsed') || document.body.classList.contains('demo')) return;
    // ROS 模式多画一条「按真实时间算的速度」，和 odom.twist 对照；图例里带上时间戳走速比
    const sc = store.speedCheck, wallName = `按真实时间${sc ? `（时间戳 ×${sc.rate.toFixed(2)}）` : ''}`;
    const vSeries = [{ d:hist.v, c:'#9db0c7', name:'cmd_vel', dash:[4,3] }, { d:hist.vo, c:ACC, name:'odom.twist', fill:'rgba(56,225,220,.12)' }];
    const wSeries = [{ d:hist.w, c:'#9db0c7', name:'cmd_vel', dash:[4,3] }, { d:hist.wo, c:TRAJ, name:'odom.twist' }];
    if (ros){ vSeries.push({ d:hist.vw, c:'#ffe08a', name:wallName }); wSeries.push({ d:hist.ww, c:'#ffe08a', name:wallName }); }
    const cfg = {
      v: [vSeries, { range:[-.1,1.2], ticks:[0,.4,.8,1.2], dec:1 }],
      w: [wSeries, { range:[-1.5,1.5], ticks:[-1,0,1], dec:0 }],
      tilt: [[{ d:hist.roll, c:'#ff6b6b', name:'roll' }, { d:hist.pitch, c:'#5be08d', name:'pitch' }], { range:[-3,3], ticks:[-2,0,2], dec:0 }],
      loc: [[{ d:hist.match, c:ACC, name:'scan / map 匹配率', fill:'rgba(56,225,220,.12)' }], { range:[30,100], ticks:[40,60,80,100], dec:0 }],
    };
    for (const [m, cv] of Object.entries(plotCanvas)) chart(cv, cfg[m][0], { ...cfg[m][1], pad:true });
  }

  // 底部面板：拖上边缘调高度，双击恢复默认；高度记在本机
  const drawer = $('#drawer'), DH_DEFAULT = 212;
  const setDH = px => { const v = clamp(Math.round(px), 150, Math.round(window.innerHeight*.6)); drawer.style.setProperty('--dh', v+'px'); return v; };
  try { const v = +localStorage.getItem('go2w.drawerH'); if (v) setDH(v); } catch {}
  const grip = $('#drawerGrip');
  grip.addEventListener('pointerdown', e => {
    e.preventDefault(); grip.setPointerCapture(e.pointerId); drawer.classList.add('dragging');
    const y0 = e.clientY, h0 = drawer.getBoundingClientRect().height;
    const mv = ev => setDH(h0 + (y0 - ev.clientY));
    const up = () => { grip.removeEventListener('pointermove', mv); grip.removeEventListener('pointerup', up); drawer.classList.remove('dragging');
      try { localStorage.setItem('go2w.drawerH', String(Math.round(drawer.getBoundingClientRect().height))); } catch {} };
    grip.addEventListener('pointermove', mv); grip.addEventListener('pointerup', up);
  });
  grip.ondblclick = () => { setDH(DH_DEFAULT); try { localStorage.removeItem('go2w.drawerH'); } catch {} };

  /* ---------- 模式 / 抽屉 ---------- */
  const setDemo = demo => { document.body.classList.toggle('demo', demo); $('#mDemo').classList.toggle('on', demo); $('#mDebug').classList.toggle('on', !demo);
    setTimeout(() => { mapView.resize(); mapView.fit(); }, 30); };
  $('#mDebug').onclick = () => setDemo(false);
  $('#mDemo').onclick = () => setDemo(true);
  // 左右栏收起 / 展开；记在本机浏览器里，刷新后保持
  function sidePanel(side, label, key){
    const cls = `${side}-off`, b = $(`#${side}Toggle`), store_ = `go2w.${side}Off`;
    const set = off => {
      document.body.classList.toggle(cls, off);
      b.setAttribute('aria-expanded', String(!off));
      b.title = `${off ? '展开' : '收起'}${label} ( ${key} )`; b.setAttribute('aria-label', `${off ? '展开' : '收起'}${label}`);
      try { localStorage.setItem(store_, off ? '1' : '0'); } catch {}
    };
    const toggle = () => set(!document.body.classList.contains(cls));
    b.onclick = toggle;
    try { if (localStorage.getItem(store_) === '1') set(true); } catch {}
    return toggle;
  }
  const toggleLeft = sidePanel('left', '左栏', '['), toggleRight = sidePanel('right', '右栏', ']');
  $$('.dtab').forEach(b => b.onclick = () => { $$('.dtab').forEach(x => x.classList.toggle('on', x===b)); $$('.dpane').forEach(p => p.hidden = p.id!==b.dataset.pane);
    $('#logFilters').style.visibility = b.dataset.pane==='pLogs' ? 'visible' : 'hidden'; $('#drawer').classList.remove('collapsed'); });
  $('#drawerToggle').onclick = () => $('#drawer').classList.toggle('collapsed');

  // 机器人照片：依次试 img/go2w.jpg / .png / .webp，都没有就保留图标
  (function loadPhoto(exts){
    if (!exts.length) return;
    const probe = new Image();
    probe.onload = () => {
      $$('.robot-img').forEach(el => { el.src = probe.src; el.hidden = false; });
      $$('.robot-fallback').forEach(el => el.style.display = 'none');
      document.body.classList.add('has-photo');
    };
    probe.onerror = () => loadPhoto(exts.slice(1));
    probe.src = `img/go2w.${exts[0]}`;
  })(['jpg', 'png', 'webp']);

  const badge = $('#srcBadge'); badge.dataset.s = src.name; badge.textContent = ros ? '实时 · ROS 2' : '模拟数据';
  renderLayers();

  return {
    fast(){ renderTask(); renderHud(); renderArm(); syncSliders(); }, // 5 Hz
    medium(){ renderChips(); renderCharts(); renderProfiles(); },  // 4 Hz
    slow(){ renderTopics(); renderMaps(); $('#clock').textContent = new Date().toTimeString().slice(0,8); }, // 1 Hz
    pushHist,                                                      // 10 Hz
    toggleLeft, toggleRight,
  };
}
