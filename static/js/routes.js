import { $ } from './util.js';

export function setupRouteEditor(store, actions, setTool){
  const select = $('#routeSelect'), name = $('#routeName'), list = $('#routePoints');
  const save = $('#routeSave'), run = $('#routeRun'), hint = $('#routeHint');
  const confirm = $('#routeConfirm'), check = $('#routeCheck'), saveRow = $('#routeSaveRow');
  let lastRoutes, lastDraft, savedDraft = store.routeDraft, saving = false;
  const selected = () => store.routes.find(route => route.id === select.value);

  function edit(route = null){
    store.routeDraft = route ? { ...route, confirmed: true } : { id:null, map:null, points:[], confirmed:false };
    store.routeCheck = null;
    savedDraft = store.routeDraft;
    name.value = route?.name || '';
  }

  async function verify(draft){
    if (store.routeCheck?.pending) return;
    const preview = { draft, pending:true };
    store.routeCheck = preview;
    render();
    const reply = await actions.checkRoute(draft);
    if (store.routeCheck !== preview || store.routeDraft !== preview.draft) return;
    Object.assign(preview, { pending:false, paths:reply.data?.paths, message:reply.msg, simulated:reply.data?.simulated });
    render();
  }

  select.onchange = () => { edit(selected()); setTool('pan'); render(); };
  $('#routeNew').onclick = () => { select.value = ''; edit(); setTool('route'); render(); };
  run.onclick = () => actions.startRoute(selected());
  confirm.onclick = () => {
    const draft = store.routeDraft, points = draft.points;
    // 沿着路线给每站一个默认朝向。首站朝向下一站，之后各站沿上一段的方向到达。
    const oriented = points.map((p, i) => {
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
    const draft = store.routeDraft;
    if (store.routes !== lastRoutes){
      lastRoutes = store.routes;
      const options = store.routes.map(r => new Option(`${r.name} · ${r.map}`, r.id));
      select.replaceChildren(new Option('选择已保存任务', ''), ...options);
      select.value = draft.id || '';
    }
    if (draft !== lastDraft){
      lastDraft = draft;
      list.replaceChildren(...draft.points.map((p, i) => {
        const li = document.createElement('li'), label = document.createElement('span');
        const end = draft.confirmed && i === draft.points.length - 1;
        label.textContent = `${i + 1} 号点${end ? ' · 终点' : ''}（${p.x.toFixed(1)}, ${p.y.toFixed(1)}）`;
        label.classList.toggle('route-end', end);
        const button = (text, title, action, disabled = false) => {
          const b = document.createElement('button');
          b.className = 'btn sm'; b.textContent = text; b.title = title;
          b.setAttribute('aria-label', title); b.disabled = disabled;
          b.onclick = () => {
            const points = [...draft.points];
            action(points);
            store.routeDraft = { ...draft, points, confirmed:false };
            render();
          };
          return b;
        };
        const move = target => points => { [points[i], points[target]] = [points[target], points[i]]; };
        li.append(label, button('↑', '上移第 ' + (i + 1) + ' 点', move(i - 1), i === 0),
          button('↓', '下移第 ' + (i + 1) + ' 点', move(i + 1), i === draft.points.length - 1),
          button('×', '删除第 ' + (i + 1) + ' 点', points => points.splice(i, 1)));
        return li;
      }));
    }
    const preview = store.routeCheck?.draft === draft ? store.routeCheck : null;
    const dirty = draft !== savedDraft || name.value !== (savedDraft.name || '');
    const matching = draft.map && draft.map === store.mapCurrent;
    const running = store.task.type !== 'idle' && store.task.status === 'active';
    confirm.hidden = draft.confirmed;
    confirm.disabled = !matching || draft.points.length < 2;
    check.hidden = !draft.confirmed;
    check.disabled = !matching || running || !!preview?.pending;
    saveRow.hidden = !draft.confirmed;
    save.disabled = saving || !matching || !draft.confirmed || !name.value.trim();
    run.disabled = saving || !!preview?.pending || dirty || !matching || !selected() || running || !actions.canMove();
    let message = '用地图工具栏的「导航点」连续点击选点，至少选两站。';
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
