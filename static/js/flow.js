// 右栏「任务」：导航 / 重定位 / 建图三个标签 + 导航的五步流程（原型见 web/prototype/right_panel.html）
// 导航（11-11 改）：地图 → 任务 → 参数 → 出发前自检 → 出发。前三步不用开导航（后端直接读地图文件显示）；
// 第 3 步「开始自检」才启动导航、自动定位、用 Nav2 检查路径，全部通过后「出发」才能点。
// 建图 / 重定位标签照旧：点标签就切换机器人档位，切换时先停车（取消任务、底盘上锁）。
import { $, $$, deg } from './util.js';
import { toast, formBox, stillCheck } from './ui.js';

export function setupFlow(store, src, actions, setTool, reloc){
  const prof = id => store.profiles[id];
  const up = id => ['run', 'starting'].includes(prof(id));       // 在跑或正在起来
  const run = id => prof(id) === 'run';
  const exec = p => Promise.resolve(p).then(r => { if (r && r.ok === false) toast(r.msg || '操作失败', 'err'); else if (r?.msg) toast(r.msg); return r; });
  store.uiTask = null;          // 用户选的标签（同一个导航栈上可以是「导航」或「重定位」）
  store.initPose = null;        // 这一轮导航里给过初始位置（或确认过当前位置正确）
  const openSteps = new Set();
  // 地图来自 map_server（可以定位、规划）；模拟模式没有这个区分
  const liveMap = () => store.source !== 'ros' || !!store.mapLive;

  // 当前标签：跟着机器人实际在跑的档位走；什么都没开时默认「导航」（离线选图、编辑任务）
  function tab(){
    if (up('mapping')) return 'map';
    if (up('localization')) return store.uiTask === 'nav' ? 'nav' : 'reloc';
    if (up('navigation')) return store.uiTask === 'reloc' ? 'reloc' : 'nav';
    return store.uiTask || 'nav';
  }
  const routeActive = () => store.task.type !== 'idle' && store.task.status === 'active';
  const mapNames = () => store.maps.map(m => m.name);
  const leavingMapWarn = () => {
    if (!up('mapping')) return '';
    const saved = store.mapSavedAt;
    return saved ? `建图上次保存是 ${Math.max(1, Math.round((Date.now() - saved) / 60000))} 分钟前，之后走过的部分会丢失。`
      : '这个页面上还没有保存过当前地图，停止建图后地图会丢失。';
  };
  const busyMsg = () => store.mapSaving || store.mapBusy ? '地图正在保存，请稍等' : store.busy ? '上一个启停还没完成，稍等几秒' : '';
  // 停车：取消任务 + 上锁（导航栈不动）
  async function park(){
    if (routeActive() || store.task.type !== 'idle') await exec(src.cmd.cancel());
    if (store.armed) await exec(src.cmd.arm(false));
  }

  function startStack(mode, title, body, uiTask){
    const b = busyMsg(); if (b) return toast(b, 'warn');
    formBox({ title, body, warn: leavingMapWarn(), ok: title.replace('…', ''), danger: !!up('mapping') && !store.mapSavedAt,
      build: f => stillCheck(f),
      valid: f => f.querySelector('[data-still]').checked,
      onOk: () => {
        if (mode === 'mapping'){ store.mapSavedAt = null; store.mapSavedName = null; }
        store.uiTask = uiTask; store.initPose = null; openSteps.clear(); resetPrep();
        exec(src.cmd.setMode(mode, mode === 'mapping' ? null : store.mapCurrent));
      } });
  }
  // 标签对应的档位已经在跑（或正在起来）
  const live = t => t === 'nav' ? up('navigation') : t === 'reloc' ? up('navigation') || up('localization') : up('mapping');
  function switchTo(t){
    if (t === tab() && live(t)) return;
    if (tab() === 'reloc' && t !== 'reloc' && reloc.changed()) return reloc.leave(() => switchTo(t));
    const b = busyMsg(); if (b) return toast(b, 'warn');
    if (t === 'nav'){
      // 导航不在这里启动：显示五步流程，第 3 步自检时再启动
      if (up('mapping')){
        const saved = store.mapSavedName;
        return formBox({ title: '停止建图', body: `切到导航会停止建图${saved ? `，之后用刚保存的 ${saved}` : ''}。`, warn: leavingMapWarn(),
          ok: '停止建图', danger: !store.mapSavedAt,
          onOk: async () => {
            store.uiTask = 'nav'; resetPrep(); render();
            await exec(src.cmd.setMode(null));
            if (saved) exec(src.cmd.loadMap(saved));   // 导航没开：只是选好显示 / 下次启动用的地图
          } });
      }
      store.uiTask = 'nav';
      return render();
    }
    if (t === 'reloc'){
      if (run('navigation')){
        const go = async () => { store.uiTask = 'reloc'; resetPrep(); render(); await park(); toast('导航栈保持运行，底盘已上锁', 'warn'); };
        if (routeActive()) return formBox({ title: '切换到重定位', body: '当前多站任务会被取消，底盘会上锁。导航栈保持运行，改完位置可以直接回到导航。', ok: '取消任务并切换', danger: true, onOk: go });
        return go();
      }
      return startStack('localization', '启动定位', `${up('mapping') ? '将停止建图，' : ''}用地图 ${store.mapCurrent || '（默认）'} 启动 FAST-LIO、map_server 和 AMCL（不启动 Nav2），然后修正位置。`, 'reloc');
    }
    if (t === 'map'){
      const from = up('navigation') ? '将停止导航（正在执行的任务会取消），' : up('localization') ? '将停止定位，' : '';
      return startStack('mapping', '开始建图', `${from}启动 FAST-LIO 和 slam_toolbox。地图从机器人当前位置开始，x 轴取机器人此刻的朝向。`, 'map');
    }
  }
  $$('#taskTabs button').forEach(b => b.onclick = () => switchTo(b.dataset.t));

  // 第 1 步：地图。导航没开时选了就换（后端直接读文件显示）；导航开着时 map_server 换图，要确认
  const mapSel = $('#navMapSel');
  const pickMap = () => { const v = mapSel.value; if (v && v !== store.mapCurrent) actions.loadMap(v); };
  $('#navMapLoad').onclick = pickMap;
  mapSel.onchange = () => { if (!up('navigation') && !up('localization')) pickMap(); };
  // 定位（第 3 步里失败时手动补救）
  $('#navInitBtn').onclick = () => setTool('init');
  $('#navInitOk').onclick = () => { store.initPose = { manual: true, map: store.mapCurrent }; render(); };
  // 地图工具栏发出的初始位姿成功后算定位完成（actions.initialPose 会调这个）
  store.onInitialPose = () => { store.initPose = { map: store.mapCurrent }; render(); };
  $$('#navFlow [data-open]').forEach(b => b.onclick = () => { const k = b.dataset.open; openSteps.has(k) ? openSteps.delete(k) : openSteps.add(k); render(); });
  // 重定位 / 建图
  $('#relocInit').onclick = () => setTool('init');
  // 自动定位：按钮手动触发；导航 / 定位启动后后端也会自动跑一次。成功（state.autoLoc.seq 变了且 ok）就算定位完成
  $('#navAutoBtn').onclick = $('#relocAuto').onclick = () => actions.autoLocalize();
  let autoSeq = null;
  function autoLocRender(){
    const a = store.autoLoc;
    // 刚打开网页时（autoSeq 还是 null）后端已经定位成功的也算：后端每次启动档位都会把状态复位成 idle
    if (a && a.seq !== autoSeq && a.state === 'ok' && a.map === store.mapCurrent && run('navigation')){
      store.initPose = { auto: true, map: store.mapCurrent };
    }
    if (a) autoSeq = a.seq;
    const busy = a?.state === 'run';
    const text = busy ? '自动定位中…（机器人保持静止）' : a && a.map === store.mapCurrent && ['ok', 'fail'].includes(a.state) ? a.msg : '';
    for (const id of ['navAutoNote', 'relocAutoNote']){
      const el = $('#' + id);
      el.hidden = !text; el.textContent = text;
      el.classList.toggle('warn', busy || a?.state === 'fail');
    }
    $('#navAutoBtn').disabled = $('#relocAuto').disabled = busy;
    $('#navAutoBtn').textContent = $('#relocAuto').textContent = busy ? '定位中…' : '自动定位';
  }

  // ---------- 第 3 步：出发前自检 ----------
  // 站立（趴着就站起来、启动策略）→ 导航（没开就用任务的地图启动；开着但图不对就切图）→ 自动定位 → Nav2 检查站间路径。
  // 每一项都从后端推来的状态判断，全部通过后 store.prepOk(任务) 为真，「出发」才能点（actions.routeStartBlocker）。
  // 自检通过后：导航停了、定位作废、任务改了、出发了，都要重新自检。
  const prep = { on: false };
  const sig = r => r ? `${r.id}|${r.map}|${JSON.stringify(r.points)}` : '';
  const curRoute = () => store.routes.find(r => r.id === $('#routeSelect').value && r.map === store.mapCurrent) || null;
  function resetPrep(){ for (const k of Object.keys(prep)) delete prep[k]; prep.on = false; }
  store.prepOk = route => !!(prep.on && prep.done && !prep.fail && route && prep.sig === sig(route) && run('navigation') && store.initPose && actions.walking());
  // 自检进行中（以及通过后 2 分钟内等出发）每 2 s 告诉后端，别把这段时间当空闲自动趴下
  const PREP_HOLD_MS = 120000;
  function prepHold(now){
    if (!prep.on || prep.fail || (prep.doneT && now - prep.doneT > PREP_HOLD_MS) || now - (prep.holdT || 0) < 2000) return;
    prep.holdT = now;
    if (src.can('prepHold')) src.cmd.prepHold();
  }
  function prepFail(k, msg, soft = false){ prep.fail = { k, msg, soft }; prep.done = false; }

  function startPrep(){
    const route = curRoute();
    if (!route) return toast('先在第 2 步选择（或保存）一个任务', 'warn');
    if (store.routeDraft.points.length && (store.routeDraft.id !== route.id || !store.routeDraft.confirmed)) return toast('第 2 步的路线还没保存', 'warn');
    if (up('mapping')) return toast('先停止建图', 'warn');
    const b = busyMsg(); if (b) return toast(b, 'warn');
    if (store.estop) return toast('急停中：先解除急停', 'warn');
    const begin = () => {
      resetPrep();
      Object.assign(prep, { on: true, done: false, fail: null, route: route.id, sig: sig(route), map: route.map, t0: Date.now(),
        seq0: store.autoLoc?.seq ?? -1, standT: 0, standOk: false, doneT: 0, holdT: 0, navT: 0, runT: 0, loadT: 0, autoT: 0, check: null, retried: false });
      openSteps.add('stPrep');
      render();
    };
    if (up('navigation')) return begin();
    // 要启动导航：FAST-LIO 初始化时机器人要站好不动
    formBox({ title: '出发前自检', ok: '开始自检',
      body: `将用地图 ${route.map} 启动导航（约 10–20 秒），然后自动定位、检查路径。${up('localization') ? '当前的「定位」档位会先停掉。' : ''}`,
      build: f => stillCheck(f), valid: f => f.querySelector('[data-still]').checked, onOk: begin });
  }
  $('#prepRun').onclick = startPrep;
  $('#prepStop').onclick = () => { resetPrep(); render(); };

  function prepTick(){
    if (!prep.on) return;
    if (routeActive()){ resetPrep(); return; }                // 已经出发：这次自检用掉了
    if (prep.fail){
      if (prep.fail.k === 'loc' && store.initPose && run('navigation')) prep.fail = null;   // 手动补救了定位，接着往下走
      else return;
    }
    const route = store.routes.find(r => r.id === prep.route);
    if (!route || sig(route) !== prep.sig) return prepFail('route', '任务改过了，请重新自检');
    if (store.estop) return prepFail('nav', '急停中');
    const now = Date.now();
    prepHold(now);
    // 0 站立：导航启动（FAST-LIO 初始化）和定位都要在站好之后做
    const C = store.ctrl;
    if (C?.on){
      if (!(C.state === 'Velocity' && C.ready)){
        if (prep.standOk) return prepFail('stand', C.state ? `机器狗不在行走状态（${C.state}），请重新自检` : '收不到运控状态，请重新自检');
        if (C.state == null) return prepFail('stand', '收不到运控状态：go2w-ctrl.service 没在运行？');
        if (!prep.standT){
          prep.standT = now;
          const t0 = prep.t0;
          actions.standUp().then(r => {
            if (prep.on && prep.t0 === t0 && r?.ok === false){ prepFail('stand', r.msg || '站起失败'); render(); }
          });
        }
        return;
      }
      prep.standOk = true;
    }
    // 1 导航
    if (!run('navigation')){
      if (prep.runT) return prepFail('nav', '导航停了');
      if (up('navigation') || store.busy) return;              // 启动中
      if (!prep.navT){ prep.navT = now; exec(src.cmd.setMode('navigation', prep.map)); return; }
      if (now - prep.navT > 8000) prepFail('nav', '导航没有启动起来：看「日志」里的报错');
      return;
    }
    if (!prep.runT) prep.runT = now;
    if (!store.mapCurrent || !liveMap()) return;                // 刚起来：等 map_server 的地图名
    if (store.mapCurrent !== prep.map){
      if (!prep.loadT){ prep.loadT = now; exec(src.cmd.loadMap(prep.map)); return; }
      if (now - prep.loadT > 15000) prepFail('nav', `切换到地图 ${prep.map} 没有成功`);
      return;
    }
    // 2 定位
    if (!store.initPose){
      if (prep.done) return prepFail('loc', '定位作废了（换过地图或导航重启），请重新自检');
      if (store.source !== 'ros'){ store.initPose = { auto: true, map: store.mapCurrent }; return; }   // 模拟：定位没有误差
      const a = store.autoLoc;
      if (a?.state === 'run') return;
      if (a && a.seq > prep.seq0 && a.map === prep.map && a.state === 'fail') return prepFail('loc', a.msg || '自动定位失败', true);
      // 自检启动的导航：后端起来后会自动定位一次，等它（最多 40 s）；导航本来就开着就自己请求
      if (prep.navT && !prep.autoT && (!a || a.state === 'idle') && now - prep.runT < 40000) return;
      if (!prep.autoT){
        prep.autoT = now;
        const t0 = prep.t0, seq = a?.seq ?? -1;
        // 后端没开始就拒绝了（不会更新 autoLoc.seq）：直接显示原因，别干等
        actions.autoLocalize().then(r => {
          if (prep.on && prep.t0 === t0 && r?.ok === false && (store.autoLoc?.seq ?? -1) <= seq && !store.initPose){
            prepFail('loc', r.msg || '自动定位失败', true); render();
          }
        });
        return;
      }
      if (now - prep.autoT > 110000) prepFail('loc', '自动定位没有结果', true);
      return;
    }
    // 3 路径检查
    if (!prep.check){
      if (store.source !== 'ros'){ prep.check = { state: 'ok', msg: '本地模拟，没有用 Nav2 检查' }; return; }
      prep.check = { state: 'run' };
      const t0 = prep.t0;
      actions.checkRoute(route).then(r => {
        if (!prep.on || prep.t0 !== t0) return;
        if (r?.ok){
          prep.check = { state: 'ok', msg: r.msg || '全部站间路径可规划' };
          // 地图上画出检查过的路径（第 2 步正在看这条任务时）
          if (store.routeDraft.id === route.id && store.routeDraft.confirmed)
            store.routeCheck = { draft: store.routeDraft, pending: false, paths: r.data?.paths, message: r.msg };
        } else if (!prep.retried && /未就绪|没有运行/.test(r?.msg || '')){
          prep.retried = true;                                  // Nav2 刚起来，规划器可能还差一点：3 s 后再试一次
          setTimeout(() => { if (prep.on && prep.t0 === t0){ prep.check = null; render(); } }, 3000);
        } else {
          prep.check = { state: 'fail', msg: r?.msg || '检查失败' };
          prepFail('route', prep.check.msg);
        }
        render();
      });
      return;
    }
    if (prep.check.state === 'ok'){ prep.done = true; prep.doneT ||= now; }
  }

  function prepRender(route){
    const F = prep.fail, R = store.robot, m = store.loc.match;
    const navHere = run('navigation') && liveMap() && store.mapCurrent === route?.map;
    const C = store.ctrl;
    const items = {
      stand: !C?.on ? ['skip', store.source === 'ros' ? '运控状态未接入（GO2W_CTRL=0）：用遥控器站起来' : '模拟：没有运控']
        : F?.k === 'stand' ? ['fail', F.msg]
        : C.op === 'up' ? ['run', '站起中…（站稳约 2 s，再启动策略）']
        : C.op === 'down' ? ['run', '趴下中…']
        : C.state === 'Velocity' && C.ready ? ['ok', '已站起，策略运行中']
        : C.state == null ? ['todo', '收不到运控状态']
        : ['todo', C.state === 'Passive' ? '趴着，自检时自动站起' : C.state === 'FixStand' ? '站着（策略未接管），自检时启动策略' : C.state],
      nav: F?.k === 'nav' ? ['fail', F.msg]
        : navHere ? ['ok', `运行中 · ${store.mapCurrent}`]
        : prep.on && prep.loadT ? ['run', `切换到地图 ${prep.map}…`]
        : prep.on || up('navigation') ? ['run', run('navigation') ? '等待地图…' : '启动中…（机器人保持静止）']
        : ['todo', run('localization') ? '只开了定位，自检时启动导航' : '没有运行，自检时自动启动'],
      loc: F?.k === 'loc' ? ['fail', F.msg]
        : store.initPose && navHere ? ['ok', `(${R.x.toFixed(2)}, ${R.y.toFixed(2)}) · ${Math.round(deg(R.th)) || 0}°${m != null ? ` · 匹配率 ${m.toFixed(0)}%` : ''}`]
        : store.autoLoc?.state === 'run' ? ['run', '定位中…（机器人保持静止）']
        : prep.on && navHere ? ['run', '等待自动定位…']
        : ['todo', '导航启动后自动定位'],
      route: F?.k === 'route' ? ['fail', F.msg]
        : prep.check?.state === 'ok' ? ['ok', prep.check.msg]
        : prep.check?.state === 'run' ? ['run', '检查中…']
        : ['todo', '用 Nav2 规划每一段站间路径'],
    };
    for (const li of $$('#prepList li')){
      const [s, t] = items[li.dataset.k];
      if (li.dataset.s !== s) li.dataset.s = s;
      const sm = li.querySelector('small'); if (sm.textContent !== t) sm.textContent = t;
    }
    const running = prep.on && !prep.done && !F;
    $('#prepRun').hidden = running;
    $('#prepRun').textContent = prep.done || F ? '重新自检' : '开始自检';
    $('#prepRun').disabled = !route || !!store.busy;
    $('#prepStop').hidden = !running;
    // 定位失败（或导航开着但还没定位）时给手动补救的按钮
    $('#prepLocRow').hidden = !(navHere && !store.initPose && (F?.k === 'loc' || !prep.on));
    $('#navInitOk').hidden = !R.valid || store.fixed !== 'map';
  }

  $('#relocBack').onclick = () => switchTo('nav');   // 有没应用的改动时 switchTo 会先问
  $('#mapTele').onclick = () => setTool('teleop');
  $('#mapToNav').onclick = () => switchTo('nav');
  $('#mapStart').onclick = () => switchTo('map');
  $('#mapStop').onclick = () => actions.setMode(null);          // 没保存会先提醒
  $('#mapBag').onchange = e => { e.target.checked = !e.target.checked; actions.toggle('bag'); };   // 勾选框跟着实际状态走
  // 地图名：默认 lab_年月日；保存成功后换成下一个没用过的名字（后端不覆盖同名地图）
  const mapName = $('#mapName'), today = () => { const d = new Date(); return `lab_${d.getFullYear()}${String(d.getMonth() + 1).padStart(2, '0')}${String(d.getDate()).padStart(2, '0')}`; };
  const freeName = base => { const used = new Set(mapNames()); if (!used.has(base)) return base; let i = 2; while (used.has(`${base}_${i}`)) i++; return `${base}_${i}`; };
  let nameTouched = false, lastSaved = null;
  mapName.oninput = () => { nameTouched = true; };
  const fmtUp = s => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

  function setStep(id, state, sub){
    const el = $('#' + id);
    if (el.dataset.s !== state) el.dataset.s = state;
    el.classList.toggle('open', state === 'done' && openSteps.has(id));
    const more = el.querySelector('.step-more');
    if (more){ more.hidden = state !== 'done'; more.textContent = openSteps.has(id) ? '收起' : id === 'stPrep' ? '详情' : '更改'; }
    const s = $('#' + id + 'Sub'); if (s.textContent !== sub) s.textContent = sub;
  }
  let lastMaps, lastNav, lastMapCur, lastStood;
  function render(){
    // 导航停了或换了地图：初始位置作废
    const navOn = run('navigation');
    if (!navOn && lastNav) store.initPose = null;
    lastNav = navOn;
    // 从趴着站起来：雷达的高度和姿态变了，定位要重新做（自检会自动定位）
    const stood = store.ctrl?.stood;
    if (stood != null && lastStood != null && stood !== lastStood) store.initPose = null;
    if (stood != null) lastStood = stood;
    if (store.initPose && store.initPose.map !== store.mapCurrent) store.initPose = null;
    autoLocRender();
    prepTick();

    const t = tab();
    $$('#taskTabs button').forEach(b => { b.classList.toggle('on', b.dataset.t === t); b.disabled = !!store.busy; });
    const stack = up('mapping') ? '建图' : up('navigation') ? '导航' : up('localization') ? '定位' : '';
    const starting = ['mapping', 'localization', 'navigation'].some(m => prof(m) === 'starting'), stopping = ['mapping', 'localization', 'navigation'].some(m => prof(m) === 'stopping');
    const pill = $('#stackPill');
    const ps = store.estop ? ['crit', '急停'] : starting ? ['warn', '启动中…'] : stopping ? ['warn', '停止中…'] : routeActive() ? ['run', '执行任务'] : stack ? ['ok', stack + '运行中'] : ['', '未运行'];
    if (pill.textContent !== ps[1]){ pill.dataset.s = ps[0]; pill.textContent = ps[1]; }
    $('#stackTxt').textContent = stack ? `${stack}${starting ? '（启动中）' : ''}${store.mapCurrent && stack !== '建图' ? ' · ' + store.mapCurrent : ''}` : '无';
    $('#armTxt').textContent = store.estop ? '急停' : store.armed == null ? '--' : store.armed ? '已解锁' : '已上锁';

    $('#flowNone').hidden = !!t;
    $('#flowNav').hidden = t !== 'nav';
    $('#flowReloc').hidden = t !== 'reloc';
    $('#flowMap').hidden = t !== 'map';
    // 执行中：步骤收起来，只看进度卡片
    const showTask = store.task.type !== 'idle';
    $('#taskCard').hidden = !(showTask && (t === 'nav' || !t));
    $('#navFlow').hidden = showTask;

    if (t === 'nav' && !showTask){
      // 1 地图
      // 11-09 修：原来每次刷新只要下拉框没焦点就改回当前地图，点「加载」时焦点移到按钮，选择被冲掉，点了没反应。
      // 现在只在当前地图变化时同步；列表刷新时保留用户的选择
      if (store.maps !== lastMaps){
        const keep = mapSel.value;
        lastMaps = store.maps;
        mapSel.replaceChildren(...mapNames().map(n => new Option(n, n)));
        if (keep && mapNames().includes(keep)) mapSel.value = keep;
        else if (store.mapCurrent) mapSel.value = store.mapCurrent;
      }
      if (store.mapCurrent !== lastMapCur){ lastMapCur = store.mapCurrent; if (store.mapCurrent) mapSel.value = store.mapCurrent; }
      const mapOk = !!store.mapCurrent;
      const navLive = up('navigation') || up('localization');
      setStep('stMap', mapOk ? 'done' : 'cur', mapOk ? `${store.mapCurrent}${navLive ? ' · 导航在用' : ''}` : '等待地图');
      $('#navMapHint').textContent = navLive
        ? '导航正在用这张图：换图后 map_server 立即加载新图，要重新定位（重新自检）。'
        : '导航不用先开：选好地图就能编辑任务，出发前自检时用这张图启动导航。';
      $('#navMapLoad').hidden = !navLive;       // 导航没开时选了就换
      // 2 导航任务（routes.js 管里面的内容）
      const sel = $('#routeSelect').value, route = store.routes.find(r => r.id === sel && r.map === store.mapCurrent);
      const draft = store.routeDraft, editing = draft.points.length && (!route || draft.id !== route.id || !draft.confirmed);
      const routeOk = mapOk && route && !editing;
      setStep('stRoute', !mapOk ? 'todo' : routeOk ? 'done' : 'cur',
        route ? `${route.name} · ${route.points.length} 站` : editing ? '正在编辑新路线' : store.routes.some(r => r.map === store.mapCurrent) ? '选择一个任务' : '这张地图还没有任务，点「新建」');
      // 3 出发前自检
      if (prep.on && route && prep.route !== route.id) resetPrep();   // 换了任务：上一次自检不算
      prepRender(routeOk ? route : null);
      const ok = routeOk && store.prepOk(route);
      setStep('stPrep', !routeOk ? 'todo' : ok ? 'done' : 'cur',
        ok ? '全部通过' : prep.fail ? `未通过：${prep.fail.msg}` : prep.on ? '自检中…' : '站立 → 导航 → 定位 → 路径检查');
      if (ok && !prep.shown){ prep.shown = true; openSteps.delete('stPrep'); }
      // 4 运行参数：自检通过后才展开，和「出发」一起停在眼前（11-11 修：原来任务一选好就标成完成并收起，看起来被跳过了）
      // 导航没开时先记在滑条上，出发时带给后端
      const round = $('#routeRound').checked, loops = +$('#routeLoops').value || 0;
      setStep('stParam', ok ? 'cur' : 'todo',
        `${round ? (loops ? `循环 ${loops} 圈` : '一直循环') : '单程'} · ${(+$('#slV').value).toFixed(2)} m/s · ${(+$('#slW').value).toFixed(1)} rad/s`);
      // 5 出发
      setStep('stGo', ok ? 'cur' : 'todo', ok ? '自检通过，可以出发' : '先完成出发前自检');
    }
    if (t === 'reloc'){
      $('#relocMeta').textContent = run('navigation') ? '导航栈保持运行' : up('localization') ? '只开了定位（没有 Nav2）' : '';
      $('#relocNote').innerHTML = run('navigation') ? '<b>导航栈保持运行</b> · 底盘已上锁。在当前估计的基础上修正，改完直接回到导航。'
        : run('localization') ? '<b>只开了定位</b>（map_server + AMCL）。修正完点「返回导航」，出发前自检时启动导航。'
        : '定位正在启动…';
      $('#relocBack').textContent = '完成，返回导航';
    }
    // 重定位的激光上色、拖动只在这个标签、定位在跑时生效
    reloc.on = t === 'reloc' && (run('navigation') || run('localization'));
    reloc.render();
    if (t === 'map'){
      const on = run('mapping'), starting_ = prof('mapping') === 'starting';
      $('#mapOff').hidden = on || starting_;
      $('#mapFlow').hidden = !on && !starting_;
      if (on || starting_){
        const since = store.since?.mapping, el = since != null && on ? src.now() - since : null;
        setStep('mpRun', on ? 'done' : 'cur', on ? `slam_toolbox · 已运行 ${el != null ? fmtUp(el) : '--'}` : '正在启动，机器人保持静止…');
        const tele = document.querySelector('[data-tool="teleop"]')?.classList.contains('on');
        const bag = prof('bag');
        setStep('mpTele', on ? 'cur' : 'todo', `${tele ? '遥控中' : '按 T 进入遥控，WASD 移动'}${bag === 'run' ? ' · 录包中' : ''}`);
        const mb = $('#mapBag'); mb.checked = bag === 'run' || bag === 'starting'; mb.disabled = !on || bag === 'starting' || bag === 'stopping';
        const ang = Math.round((store.alignment?.angle || 0) * 180 / Math.PI);
        setStep('mpAlign', on ? 'cur' : 'todo', ang ? `显示方向 ${ang}°` : '没有调整');
        // 地图名：没手动改过就用默认名；刚保存成功就换下一个
        if (store.mapSavedName && store.mapSavedName !== lastSaved){ lastSaved = store.mapSavedName; nameTouched = false; mapName.value = freeName(store.mapSavedName); }
        if (!nameTouched && document.activeElement !== mapName && !mapName.value) mapName.value = freeName(today());
        const saved = store.mapSavedAt;
        setStep('mpSave', on ? 'cur' : 'todo', store.mapSaving ? '正在保存…' : saved ? `已保存 ${store.mapSavedName || ''} · ${Math.max(0, Math.round((Date.now() - saved) / 60000))} 分钟前` : '还没有保存');
        setStep('mpEnd', on ? 'cur' : 'todo', saved ? `导航会用 ${store.mapSavedName || '刚保存的地图'}` : '先保存，否则停止后地图会丢失');
        $('#mapToNav').disabled = !on || !saved;
        $('#mapStop').disabled = !on;
      }
    }
  }
  const openRoute = () => { openSteps.add('stRoute'); render(); };
  store.onRouteEdit = openRoute;
  return { render, switchTo, openRoute };
}
