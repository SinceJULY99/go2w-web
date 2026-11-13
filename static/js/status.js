// 地图右上角的运行状态牌：一眼看出机器狗现在在干什么（11-12）。
// 只从已有状态推出来（任务、出发流程、运控、各档位、急停、遥控），不需要后端额外数据。
// 同时满足几个时取最上面的：急停 > 出发失败 > 准备出发 > 任务（恢复中 / 暂停 / 运行中）> 已到达 / 任务失败
//   > 遥控 > 建图 / 重定位 > 档位启停 > 待命 / 未就绪。
// 颜色和地图上的小车一致：运行琥珀、到达绿、故障红。
import { $ } from './util.js';

const KEEP = 0.3;   // m：到达 / 失败的结果一直显示，直到机器人离开那个位置这么远，或开始新的操作

export function setupRunState(store, actions, mapView){
  const box = $('#runState'), txt = $('#runStateTxt'), sub = $('#runStateSub');
  let last = null, result = null;   // result：{ kind: 'ok' | 'fail', x, y, msg }

  const up = id => ['run', 'starting'].includes(store.profiles[id]);
  const run = id => store.profiles[id] === 'run';

  function state(){
    const T = store.task || {}, R = store.robot, C = store.ctrl;
    const active = T.type && T.type !== 'idle' && T.status === 'active';
    const ended = T.type && T.type !== 'idle' && !active ? T.status : null;
    // 记住任务结果：结束那一刻的位置
    const key = ended ? `${T.type}|${T.routeId || ''}|${ended}|${T.goal?.x ?? ''}` : null;
    if (key && key !== last){
      if (ended === 'succeeded') result = { kind: 'ok', x: R.x, y: R.y, msg: T.type === 'route' ? T.name : '单点导航' };
      else if (ended === 'aborted') result = { kind: 'fail', x: R.x, y: R.y, msg: T.message || 'Nav2 放弃了这个目标' };
      else result = null;
    }
    last = key;
    const prep = store.prep?.();
    if (active || (prep && !prep.fail)) result = null;
    if (result && R.valid && Math.hypot(R.x - result.x, R.y - result.y) > KEEP) result = null;

    if (!store.link.ok) return ['off', '未连接', '等待后端'];
    if (store.estop) return ['crit', '急停', '解除急停后才能动'];
    const bad = store.alerts?.find(a => a.level === 'crit');      // 运控离线、节点掉了、没有激光……（alerts.js）
    if (bad) return ['crit', bad.title, store.alerts.length > 1 ? `等 ${store.alerts.length} 项 · 见上方提示` : '见上方提示'];
    if (prep?.fail) return ['crit', '出发失败', `${prep.name} · ${prep.fail.msg}`];
    if (prep) return ['run', '准备出发', `${prep.name}中…`, true];
    if (active){
      const where = T.type === 'route'
        ? `${T.roundTrip ? (T.loops ? `第 ${T.loop}/${T.loops} 圈 · ` : `第 ${T.loop} 圈 · `) : ''}${T.back ? '回到起点' : `第 ${T.point}/${T.count} 站`}`
        : T.type === 'external' ? '外部目标' : '单点导航';
      if (T.phase === 'checking') return ['run', '准备出发', '清除旧障碍…', true];
      if (T.paused) return ['warn', '已暂停', where];
      if (T.phase === 'recover') return ['warn', '恢复中', `${where} · 后退 / 原地转`];
      if (!store.armed) return ['warn', '运行中', `${where} · 等待解锁`];
      return ['run', '运行中', where];
    }
    if (result?.kind === 'ok') return ['ok', '已到达', result.msg];
    if (result?.kind === 'fail') return ['crit', '任务失败', result.msg];
    if (mapView.tool === 'teleop') return ['acc', '遥控中', 'WASD 移动 · Esc 退出'];
    if (up('mapping')) return run('mapping') ? ['acc', '建图中', 'slam_toolbox'] : ['warn', '启动建图…', '机器人保持静止'];
    const starting = ['mapping', 'localization', 'navigation'].find(m => store.profiles[m] === 'starting');
    const stopping = ['mapping', 'localization', 'navigation'].some(m => store.profiles[m] === 'stopping');
    if (starting) return ['warn', '启动中…', starting === 'navigation' ? '导航 · 机器人保持静止' : '定位'];
    if (stopping) return ['warn', '停止中…', ''];
    if (store.uiTask === 'reloc' && (run('navigation') || run('localization'))) return ['acc', '重定位中', '修正位置'];
    if (run('localization')) return ['idle', '只开了定位', '没有 Nav2'];
    if (run('navigation')){
      if (!store.initPose) return ['off', '未就绪', '还没有定位'];
      if (C?.on && C.state != null && !(C.state === 'Velocity' && C.ready)) return ['idle', '待命', C.state === 'Passive' ? '趴着 · 出发时自动站起' : '站着 · 出发时启动策略'];
      return ['idle', '待命', `已定位${store.mapCurrent ? ' · ' + store.mapCurrent : ''}`];
    }
    return ['off', '未就绪', '导航没开 · 点「出发」自动启动'];
  }

  return function render(){
    const [s, t, d, pulse] = state();
    if (box.dataset.s !== s) box.dataset.s = s;
    box.classList.toggle('pulse', !!pulse);
    if (txt.textContent !== t) txt.textContent = t;
    if (sub.textContent !== d){ sub.textContent = d; sub.hidden = !d; }
    box.title = d ? `${t} · ${d}` : t;
  };
}
