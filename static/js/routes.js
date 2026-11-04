import { $, deg } from './util.js';

export function setupRouteEditor(store, actions){
  const select = $('#routeSelect'), name = $('#routeName'), list = $('#routePoints');
  const save = $('#routeSave'), run = $('#routeRun'), hint = $('#routeHint');
  let lastRoutes, lastDraft, savedDraft = store.routeDraft, saving = false;
  const selected = () => store.routes.find(route => route.id === select.value);
  const edit = (route = null) => {
    store.routeDraft = route || { id: null, map: null, points: [] };
    savedDraft = store.routeDraft;
    name.value = route?.name || '';
  };
  select.onchange = () => edit(selected());
  $('#routeNew').onclick = () => { select.value = ''; edit(); name.focus(); };
  run.onclick = () => actions.startRoute(selected());
  save.onclick = async () => {
    const draft = store.routeDraft, input = name.value;
    saving = true;
    render();
    try {
      const reply = await actions.saveRoute(input);
      if (reply?.ok && store.routeDraft === draft && name.value === input){
        edit(reply.data);
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
        label.textContent = `${i + 1}. ${p.x.toFixed(1)}, ${p.y.toFixed(1)} · ${deg(p.th).toFixed(0)}°`;
        const button = (text, title, action, disabled = false) => {
          const b = document.createElement('button');
          b.className = 'btn sm'; b.textContent = text; b.title = title;
          b.setAttribute('aria-label', title); b.disabled = disabled;
          b.onclick = () => {
            const points = [...draft.points];
            action(points);
            store.routeDraft = { ...draft, points };
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
    const dirty = draft !== savedDraft || name.value !== (savedDraft.name || '');
    const matching = draft.map && draft.map === store.mapCurrent;
    const running = store.task.type !== 'idle' && store.task.status === 'active';
    save.disabled = saving || !matching || draft.points.length < 2 || !name.value.trim();
    run.disabled = saving || dirty || !matching || !selected() || running || !actions.canMove();
    hint.textContent = draft.map && !matching ? `路线属于地图 ${draft.map}，请先切图。` : dirty && selected()
      ? '路线有未保存的修改。' : '用地图上的「路线点」选位置和朝向，保存后可按顺序执行。';
  }
  name.oninput = render;
  return render;
}
