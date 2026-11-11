import { $, deg } from './util.js';

export function setupRouteEditor(store, actions, setTool){
  const select = $('#routeSelect'), name = $('#routeName'), list = $('#routePoints');
  const save = $('#routeSave'), run = $('#routeRun'), hint = $('#routeHint'), remove = $('#routeDelete');
  const confirm = $('#routeConfirm'), check = $('#routeCheck'), saveRow = $('#routeSaveRow');
  const runHint = $('#routeRunHint'), round = $('#routeRound'), loops = $('#routeLoops');
  let lastRoutes, lastMap, lastDraft, savedDraft = store.routeDraft, saving = false, starting = false;
  const selected = () => store.routes.find(route => route.id === select.value);

  function edit(route = null){
    store.routeDraft = route ? { ...route, confirmed: true } : { id:null, map:null, points:[], confirmed:false };
    store.routeCheck = null;
    savedDraft = store.routeDraft;
    name.value = route?.name || '';
  }

  async function verify(draft){
    if (store.routeCheck?.pending) return;
    // 导航没开（只是在文件地图上编辑）：没有 Nav2 可问，出发前自检时再检查
    if (store.source === 'ros' && store.profiles.navigation !== 'run'){
      store.routeCheck = { draft, pending:false, message:'导航没开：出发前自检时会用 Nav2 检查站间路径' };
      return render();
    }
    const preview = { draft, pending:true };
    store.routeCheck = preview;
    render();
    const reply = await actions.checkRoute(draft);
    if (store.routeCheck !== preview || store.routeDraft !== preview.draft) return;
    Object.assign(preview, { pending:false, paths:reply.data?.paths, message:reply.msg, simulated:reply.data?.simulated });
    render();
  }

  // 往返循环：1 → … → N → … → 1 算一次；0 = 一直往返。只在启动时生效，不存进任务
  const readLoops = () => Math.min(1000, Math.max(0, Math.round(+loops.value || 0)));
  // 勾选循环后最后一站不再是终点：重画站点列表
  round.onchange = () => { loops.disabled = !round.checked; lastDraft = null; render(); };
  loops.onchange = () => { loops.value = readLoops(); };

  select.onchange = () => { edit(selected()); setTool('pan'); render(); };
  $('#routeNew').onclick = () => { select.value = ''; edit(); setTool('route'); render(); };
  run.onclick = async () => {
    if (run.disabled) return;
    const route = selected();
    starting = true;
    render();
    try {
      await actions.startRoute(route, { roundTrip: round.checked, loops: readLoops(), vmax: +$('#slV').value, wmax: +$('#slW').value });
    } finally {
      starting = false;
      render();
    }
  };
  remove.onclick = () => {
    const route = selected();
    if (!route) return;
    actions.deleteRoute(route, () => {
      if (store.routeDraft.id === route.id) edit();
      render();
    });
  };
  confirm.onclick = () => {
    const draft = store.routeDraft, points = draft.points;
    // 沿着路线给每站一个默认朝向。首站朝向下一站，之后各站沿上一段的方向到达。
    const oriented = points.map((p, i) => {
      if (p.manualHeading !== false) return p;
      const a = points[Math.max(0, i - 1)], b = i === 0 ? points[1] : p;
      return { ...p, th: Math.atan2(b.y - a.y, b.x - a.x) };
    });
    store.routeDraft = { ...draft, points:oriented, confirmed:true };
    store.routeCheck = null;
    setTool('pan');
    render();
    name.focus();
    verify(store.routeDraft);
  };
  check.onclick = () => verify(store.routeDraft);
  save.onclick = async () => {
    const draft = store.routeDraft, input = name.value;
    saving = true;
    render();
    try {
      const reply = await actions.saveRoute(input);
      if (reply?.ok && store.routeDraft === draft && name.value === input){
        const preview = store.routeCheck;
        edit(reply.data);
        if (preview?.draft === draft){
          preview.draft = store.routeDraft;
          store.routeCheck = preview;
        }
        render();
        select.value = reply.data.id;
      }
    } finally {
      saving = false;
      render();
    }
  };

  function render(){
    let draft = store.routeDraft;
    if (store.routes !== lastRoutes || store.mapCurrent !== lastMap){
      lastRoutes = store.routes; lastMap = store.mapCurrent;
      if (draft.id && !store.routes.some(r => r.id === draft.id)){
        if (draft === savedDraft && name.value === (savedDraft.name || '')) edit();
        else store.routeDraft = { ...draft, id:null };
        draft = store.routeDraft;
      }
      // 只列当前地图的任务（正在编辑的那条例外，免得换图后选择框里找不到它）
      const mine = store.routes.filter(r => r.map === store.mapCurrent || r.id === draft.id);
      const options = mine.map(r => new Option(`${r.name} · ${r.points.length} 站${r.map === store.mapCurrent ? '' : ' · ' + r.map}`, r.id));
      select.replaceChildren(new Option(mine.length ? '选择任务…' : '这张地图还没有任务', ''), ...options);
      select.value = draft.id || '';
    }
    if (draft !== lastDraft){
      lastDraft = draft;
      list.replaceChildren(...draft.points.map((p, i) => {
        const li = document.createElement('li'), label = document.createElement('span');
        const end = draft.confirmed && i === draft.points.length - 1;
        const angle = `${Math.round(deg(p.th))}°`;
        const heading = p.manualHeading === false ? (draft.confirmed ? `自动 ${angle}` : '自动朝向') : angle;
        label.textContent = `${i + 1} 号点${end ? (round.checked ? ' · 之后回到 1 号点' : ' · 终点') : ''} · ${heading}`;
        label.title = `坐标 (${p.x.toFixed(2)}, ${p.y.toFixed(2)}) · ${heading}`;
        label.classList.toggle('route-end', end);
        const pointName = document.createElement('input');
        pointName.type = 'text';
        pointName.className = 'route-point-name';
        pointName.maxLength = 40;
        pointName.placeholder = `第 ${i + 1} 站名称（可选）`;
        pointName.setAttribute('aria-label', pointName.placeholder);
        pointName.value = p.name || '';
        pointName.oninput = () => {
          const current = store.routeDraft;
          const points = current.points.map((point, j) => j === i ? { ...point, name:pointName.value } : point);
          const renamed = { ...current, points };
          store.routeDraft = renamed;
          // 名称不影响路径。保留规划结果和输入框焦点。
          if (store.routeCheck?.draft === current) store.routeCheck.draft = renamed;
          if (lastDraft === current) lastDraft = renamed;
          render();
        };
        const button = (text, title, action, disabled = false) => {
          const b = document.createElement('button');
          b.className = 'btn sm'; b.textContent = text; b.title = title;
          b.setAttribute('aria-label', title); b.disabled = disabled;
          b.onclick = () => {
            const current = store.routeDraft;
            const points = [...current.points];
            action(points);
            store.routeDraft = { ...current, points, confirmed:false };
            render();
          };
          return b;
        };
        const move = target => points => { [points[i], points[target]] = [points[target], points[i]]; };
        li.append(label, button('↑', '上移第 ' + (i + 1) + ' 点', move(i - 1), i === 0),
          button('↓', '下移第 ' + (i + 1) + ' 点', move(i + 1), i === draft.points.length - 1),
          button('×', '删除第 ' + (i + 1) + ' 点', points => points.splice(i, 1)), pointName);
        return li;
      }));
    }
    const preview = store.routeCheck?.draft === draft ? store.routeCheck : null;
    const dirty = draft !== savedDraft || name.value !== (savedDraft.name || '');
    const matching = draft.map && draft.map === store.mapCurrent;
    const running = store.task.type !== 'idle' && store.task.status === 'active';
    const route = selected();
    remove.hidden = !route;
    remove.disabled = saving || (running && store.task.routeId === route?.id);
    confirm.hidden = draft.confirmed;
    confirm.disabled = !matching || draft.points.length < 2;
    check.hidden = !draft.confirmed;
    check.disabled = !matching || running || !!preview?.pending || store.profiles.navigation !== 'run';
    saveRow.hidden = !draft.confirmed;
    save.disabled = saving || !matching || !draft.confirmed || !name.value.trim();
    const runningSelected = running && !!route && store.task.routeId === route.id;
    let runMessage = actions.routeStartBlocker(route);
    if (runningSelected) runMessage = '查看下方「导航任务」卡片了解进度。';
    else if (starting) runMessage = '正在向后端提交任务。';
    else if (saving) runMessage = '正在保存任务。';
    else if (dirty && draft.points.length) runMessage = '请先确认并保存当前路线。';
    else if (preview?.pending) runMessage = '正在检查路径，完成后可以启动。';
    run.disabled = runningSelected || starting || !!runMessage;
    const runText = runningSelected ? '导航任务执行中' : starting ? '正在出发…' : '出发';
    if (run.textContent !== runText) run.textContent = runText;
    if (runHint.textContent !== runMessage) runHint.textContent = runMessage;
    let message = '用「导航点」单击加点，拖拽可指定朝向。至少选两站。';
    if (draft.map && !matching) message = `路线属于地图 ${draft.map}，请先切图。`;
    else if (!draft.confirmed && draft.points.length >= 2) message = '选完后点击「确认路线」，最后一站将标为终点。';
    else if (draft.confirmed){
      if (preview?.pending) message = '正在检查站间路径…';
      else if (preview?.message) message = (preview.simulated ? '本地模拟 · ' : '') + preview.message;
      else message = '路线已确认，尚未检查站间路径。';
    }
    if (hint.textContent !== message) hint.textContent = message;
  }
  name.oninput = render;
  return render;
}
