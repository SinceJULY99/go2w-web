// 右栏「任务」：导航 / 重定位 / 建图三个标签 + 导航的五步流程（原型见 web/prototype/right_panel.html）
// 点标签就是真的切换机器人档位；切换时一律先停车：取消任务、底盘上锁。
// 导航 → 重定位不重启导航栈，只上锁；导航没在跑时进重定位才启动「定位」档位。
import { $, $$, deg } from './util.js';
import { toast, formBox, stillCheck, mapPicker } from './ui.js';

export function setupFlow(store, src, actions, setTool, reloc){
  const prof = id => store.profiles[id];
  const up = id => ['run', 'starting'].includes(prof(id));       // 在跑或正在起来
  const run = id => prof(id) === 'run';
  const exec = p => Promise.resolve(p).then(r => { if (r && r.ok === false) toast(r.msg || '操作失败', 'err'); else if (r?.msg) toast(r.msg); return r; });
  store.uiTask = null;          // 用户选的标签（同一个导航栈上可以是「导航」或「重定位」）
  store.initPose = null;        // 这一轮导航里给过初始位置（或确认过当前位置正确）
  const openSteps = new Set();

  // 当前标签：跟着机器人实际在跑的档位走；导航栈在跑时由用户在「导航」「重定位」之间选
  function tab(){
    if (up('mapping')) return 'map';
    if (up('localization')) return 'reloc';
    if (up('navigation')) return store.uiTask === 'reloc' ? 'reloc' : 'nav';
    return store.uiTask;
  }
  const routeActive = () => store.task.type !== 'idle' && store.task.status === 'active';
  const mapNames = () => store.maps.map(m => m.name);
  const defaultMap = () => (up('mapping') && store.mapSavedName) || store.mapChoice || store.mapCurrent || store.maps[0]?.name || '';
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
    const needMap = mode !== 'mapping';
    formBox({ title, body, warn: leavingMapWarn(), ok: title.replace('…', ''), danger: !!up('mapping') && !store.mapSavedAt,
      build: f => { if (needMap){ const d = defaultMap(), names = mapNames(); if (d && !names.includes(d)) names.unshift(d); mapPicker(f, names, d); } stillCheck(f); },
      valid: f => f.querySelector('[data-still]').checked && (!needMap || !!f.querySelector('[data-map]').value),
      onOk: f => {
        const map = needMap ? f.querySelector('[data-map]').value : null;
        if (mode === 'mapping'){ store.mapSavedAt = null; store.mapSavedName = null; }
        store.uiTask = uiTask; store.initPose = null; openSteps.clear();
        exec(src.cmd.setMode(mode, map));
      } });
  }
  // 标签对应的档位已经在跑（或正在起来）
  const live = t => t === 'nav' ? up('navigation') : t === 'reloc' ? up('navigation') || up('localization') : up('mapping');
  function switchTo(t){
    if (t === tab() && live(t)) return;
    if (tab() === 'reloc' && t !== 'reloc' && reloc.changed()) return reloc.leave(() => switchTo(t));
    const b = busyMsg(); if (b) return toast(b, 'warn');
    if (t === 'nav'){
      if (up('navigation')){ store.uiTask = 'nav'; return render(); }
      const from = up('mapping') ? '将停止建图，' : up('localization') ? '将停止定位，' : '';
      return startStack('navigation', '启动导航', `${from}启动 FAST-LIO、map_server、AMCL 和 Nav2，约需 10 秒，底盘保持锁定。`, 'nav');
    }
    if (t === 'reloc'){
      if (run('navigation')){
        const go = async () => { store.uiTask = 'reloc'; render(); await park(); toast('导航栈保持运行，底盘已上锁', 'warn'); };
        if (routeActive()) return formBox({ title: '切换到重定位', body: '当前多站任务会被取消，底盘会上锁。导航栈保持运行，改完位置可以直接回到导航。', ok: '取消任务并切换', danger: true, onOk: go });
        return go();
      }
      return startStack('localization', '启动定位', `${up('mapping') ? '将停止建图，' : ''}启动 FAST-LIO、map_server 和 AMCL（不启动 Nav2），然后修正位置。`, 'reloc');
    }
    if (t === 'map'){
      const from = up('navigation') ? '将停止导航（正在执行的任务会取消），' : up('localization') ? '将停止定位，' : '';
      return startStack('mapping', '开始建图', `${from}启动 FAST-LIO 和 slam_toolbox。地图从机器人当前位置开始，x 轴取机器人此刻的朝向。`, 'map');
    }
  }
  $$('#taskTabs button').forEach(b => b.onclick = () => switchTo(b.dataset.t));
  $('#navStart').onclick = () => switchTo('nav');

  // 第 1 步：地图
  const mapSel = $('#navMapSel');
  $('#navMapLoad').onclick = () => { const v = mapSel.value; if (v && v !== store.mapCurrent) actions.loadMap(v); };
  // 第 2 步：初始位置
  $('#navInitBtn').onclick = () => setTool('init');
  $('#navInitOk').onclick = () => { store.initPose = { manual: true, map: store.mapCurrent }; render(); };
  // 地图工具栏发出的初始位姿成功后算这一步完成（actions.initialPose 会调这个）
  store.onInitialPose = () => { store.initPose = { map: store.mapCurrent }; openSteps.delete('stInit'); render(); };
  $$('#navFlow [data-open]').forEach(b => b.onclick = () => { const k = b.dataset.open; openSteps.has(k) ? openSteps.delete(k) : openSteps.add(k); render(); });
  // 重定位 / 建图（第 2、3 版细化）
  $('#relocInit').onclick = () => setTool('init');
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
    if (more){ more.hidden = state !== 'done'; more.textContent = openSteps.has(id) ? '收起' : '更改'; }
    const s = $('#' + id + 'Sub'); if (s.textContent !== sub) s.textContent = sub;
  }
  let lastMaps, lastNav, lastMapCur;
  function render(){
    // 导航停了或换了地图：初始位置作废
    const navOn = run('navigation');
    if (!navOn && lastNav) store.initPose = null;
    lastNav = navOn;
    if (store.initPose && store.initPose.map !== store.mapCurrent) store.initPose = null;

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
    $('#navFlow').hidden = !navOn || showTask;
    $('#navOff').hidden = navOn || up('navigation');

    if (t === 'nav' && navOn && !showTask){
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
      setStep('stMap', mapOk ? 'done' : 'cur', store.mapCurrent || '等待 map_server');
      // 2 初始位置
      const R = store.robot, m = store.loc.match, init = !!store.initPose;
      const where = R.valid ? `(${R.x.toFixed(2)}, ${R.y.toFixed(2)}) · ${Math.round(deg(R.th)) || 0}°` : '';
      setStep('stInit', !mapOk ? 'todo' : init ? 'done' : 'cur',
        init ? `${where}${m != null ? ` · 匹配率 ${m.toFixed(0)}%` : ''}` : R.valid ? `AMCL 当前 ${where}，确认或重新设置` : '还没有设置');
      $('#navInitOk').hidden = !R.valid || store.fixed !== 'map';
      // 3 导航任务（routes.js 管里面的内容）
      const sel = $('#routeSelect').value, route = store.routes.find(r => r.id === sel && r.map === store.mapCurrent);
      const draft = store.routeDraft, editing = draft.points.length && (!route || draft.id !== route.id || !draft.confirmed);
      const routeOk = init && route && !editing;
      setStep('stRoute', !init ? 'todo' : routeOk ? 'done' : 'cur',
        route ? `${route.name} · ${route.points.length} 站` : editing ? '正在编辑新路线' : store.routes.some(r => r.map === store.mapCurrent) ? '选择一个任务' : '这张地图还没有任务，点「新建」');
      // 4 运行参数
      const round = $('#routeRound').checked, loops = +$('#routeLoops').value || 0;
      const P = store.params;
      setStep('stParam', routeOk ? 'done' : 'todo',
        `${round ? (loops ? `循环 ${loops} 圈` : '一直循环') : '单程'} · ${P.vmax != null ? P.vmax.toFixed(2) : '--'} m/s · ${P.wmax != null ? P.wmax.toFixed(1) : '--'} rad/s`);
      // 5 启动
      setStep('stGo', routeOk ? 'cur' : 'todo', routeOk ? '确认后自动解锁出发' : '先完成上面几步');
    }
    if (t === 'reloc'){
      $('#relocMeta').textContent = run('navigation') ? '导航栈保持运行' : up('localization') ? '只开了定位（没有 Nav2）' : '';
      $('#relocNote').innerHTML = run('navigation') ? '<b>导航栈保持运行</b> · 底盘已上锁。在当前估计的基础上修正，改完直接回到导航。'
        : run('localization') ? '<b>只开了定位</b>（map_server + AMCL）。修正完点「返回导航」会启动导航。'
        : '定位正在启动…';
      $('#relocBack').textContent = run('navigation') ? '完成，返回导航' : '完成，启动导航…';
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
