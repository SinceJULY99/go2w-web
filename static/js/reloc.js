// 重定位：在 AMCL 当前估计的基础上手动微调（不是重新给初始位姿）。
// 激光点按修正后的位姿重新画：贴墙绿、悬空红，匹配率实时算（和后端一样：离墙 0.10 m 以内算对上）。
// 机器人上锁不动，所以「激光点 → 修正后」= draft ∘ robot⁻¹ ∘ 点。点「应用」才发给 AMCL（小协方差）。
import { $, $$, deg } from './util.js';
import { toast, confirmBox } from './ui.js';

const STEPS = [[0.01, 0.5], [0.05, 1], [0.20, 5]];   // 步长：米、度
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

export function setupReloc(store, actions, mapView){
  const S = store.reloc = { on: false, draft: null, step: 1, view: null };

  const base = () => store.robot.valid && store.fixed === 'map' ? store.robot : null;
  const pose = () => S.draft || base();
  // 当前 scan 按 draft 变换后的点 + 每点是否贴墙 + 匹配率；没有 draft 时就是原始 scan
  function scanAt(P){
    const R = base(), sc = store.scan, wall = store.wallAt;
    if (!R || !P || !wall || !sc.length) return null;
    const c = Math.cos(P.th - R.th), s = Math.sin(P.th - R.th);
    const pts = new Float32Array(sc.length), ok = new Uint8Array(sc.length / 2); let hit = 0;
    for (let i = 0, k = 0; i < sc.length; i += 2, k++){
      const dx = sc[i] - R.x, dy = sc[i+1] - R.y;
      const x = P.x + c*dx - s*dy, y = P.y + s*dx + c*dy;
      pts[i] = x; pts[i+1] = y;
      if (wall(x, y)){ ok[k] = 1; hit++; }
    }
    return { pts, ok, match: ok.length ? 100 * hit / ok.length : null };
  }
  // 地图视图每帧调：给它要画的点（scan 一帧大约几百点，每帧算一次不费事）
  S.view = () => S.on ? scanAt(pose()) : null;
  S.pose = pose;
  S.begin = () => { if (!S.draft && base()) S.draft = { x: base().x, y: base().y, th: base().th }; return S.draft; };
  S.changed = () => !!S.draft && !!base() && (Math.hypot(S.draft.x - base().x, S.draft.y - base().y) > 0.002 || Math.abs(wrap(S.draft.th - base().th)) > 0.002);

  // 方向键按屏幕方向走（地图可能转了显示角度）；Q 逆时针、E 顺时针
  function nudge(k){
    if (!S.on || !S.begin()) return;
    const [d, a] = STEPS[S.step], D = S.draft;
    const dir = { u: [0, -1], d: [0, 1], l: [-1, 0], r: [1, 0] }[k];
    if (dir){ const v = mapView.screenDir(dir[0], dir[1]); D.x += v.x * d; D.y += v.y * d; }
    if (k === 'ccw') D.th = wrap(D.th + a * Math.PI / 180);
    if (k === 'cw') D.th = wrap(D.th - a * Math.PI / 180);
    render();
  }
  S.nudge = nudge;
  const undo = () => { S.draft = null; render(); };
  async function apply(){
    if (!S.changed()) return;
    const p = { ...S.draft }, after = scanAt(p)?.match;
    const r = await actions.initialPose(p, true);
    if (r?.ok === false) return;
    S.draft = null;
    toast(`位置已修正，发给 AMCL${after != null ? `（匹配率约 ${after.toFixed(0)}%）` : ''}`);
    render();
  }
  // 离开重定位：还有没应用的改动就问一下
  S.leave = go => {
    if (!S.changed()) { S.draft = null; return go(); }
    confirmBox({ title: '还有改动没应用', body: '位置改动还没有发给 AMCL。要先应用吗？「取消」留在重定位继续改。', ok: '应用并离开',
      onOk: async () => { await apply(); if (!S.changed()) go(); } });
  };

  $('#relocSteps').onclick = e => { const b = e.target.closest('[data-i]'); if (b){ S.step = +b.dataset.i; render(); } };
  $$('#relocPad [data-n]').forEach(b => b.onclick = () => nudge(b.dataset.n));
  $('#relocUndo').onclick = undo;
  $('#relocApply').onclick = apply;

  let lastOn = false;
  function render(){
    if (!S.on){ if (lastOn) S.draft = null; lastOn = false; return; }
    lastOn = true;
    const R = base(), ready = !!R && !!store.wallAt;
    $('#relocBody').hidden = !ready;
    $('#relocWait').hidden = ready;
    if (!ready){
      $('#relocWait').textContent = store.fixed !== 'map' ? '等待定位：AMCL 还没有发出 map → odom（先给一次初始位置）。' : '等待地图和激光数据…';
      return;
    }
    const cur = scanAt(R)?.match, aft = S.draft ? scanAt(S.draft)?.match : null;
    const col = m => m == null ? '' : m >= 80 ? 'var(--ok)' : m >= 60 ? 'var(--warn)' : 'var(--crit)';
    const m = S.changed() ? aft : cur;
    $('#relocMatch').textContent = m == null ? '--' : m.toFixed(0) + '%'; $('#relocMatch').style.color = col(m);
    $('#relocMatchSub').textContent = S.changed() && cur != null ? `修正前 ${cur.toFixed(0)}%` : '激光贴墙比例（匹配率）';
    $('#relocBar').style.width = (m ?? 0) + '%'; $('#relocBar').style.background = col(m) || 'var(--muted)';
    $$('#relocSteps [data-i]').forEach(b => b.classList.toggle('on', +b.dataset.i === S.step));
    const [d, a] = STEPS[S.step]; $('#relocUnit').innerHTML = `${Math.round(d * 100)} cm<br>${a}°`;
    const D = S.draft, ch = S.changed();
    $('#relocDelta').textContent = ch ? `改动 Δx ${(D.x - R.x).toFixed(2)} · Δy ${(D.y - R.y).toFixed(2)} m · Δθ ${deg(wrap(D.th - R.th)).toFixed(1)}°` : '还没有改动';
    $('#relocUndo').disabled = $('#relocApply').disabled = !ch;
  }
  S.render = render;
  return S;
}
