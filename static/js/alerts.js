// 服务异常提醒（11-12）：运控、后端连接、各档位的节点、激光、里程计、档位启动超时、回溯录制。
// 每 0.2 s 从已有状态算一遍「现在有哪些问题」，持续超过各自的宽限时间才算数（避免启停瞬间的抖动）：
//   - 地图上方的横幅：一直显示最严重的一条，写清楚影响和怎么处理；多条时可以展开看全部
//   - 刚出现 / 刚恢复时弹一次提示，并写进网页日志（运控的离线 / 恢复后端也会记一条）
//   - store.alerts：给右上角状态牌（status.js）和「出发」按钮（actions.routeGoBlocker）用
import { $ } from './util.js';
import { toast } from './ui.js';
import { MODES } from './actions.js';

// 三个档位都会起雷达驱动 + FAST-LIO + planar_odom + pcl2scan（odometry.launch.py），再各加自己的
const BASE_NODES = ['livox_lidar_publisher', 'fast_lio', 'planar_odom', 'pcl2scan_slam'];
const MODE_NODES = {
  mapping: [...BASE_NODES, 'slam_toolbox'],
  localization: [...BASE_NODES, 'amcl'],
  navigation: [...BASE_NODES, 'amcl', 'bt_navigator', 'go2w_cmd_bridge'],
};
const START_SLOW = 60;    // s：档位「启动中」超过这么久提醒

export function setupAlerts(store, bus){
  const bar = $('#alertBar'), more = $('#alertMore'), list = $('#alertList');
  const first = new Map();      // id → 第一次出现的时间
  let shown = new Map();        // id → 已经确认（过了宽限时间）的提醒
  const startT = {};            // 档位进入「启动中」的时间
  let expanded = false;
  store.alerts = [];
  more.onclick = () => { expanded = !expanded; paint(); };

  const run = m => store.profiles[m] === 'run';
  const nodeUp = id => store.nodes?.find(n => n.id === id)?.up;
  const nodeName = id => store.nodes?.find(n => n.id === id)?.name || id;
  const hz = name => store.topics?.find(t => t.name === name)?.hz ?? 0;

  // 当前的问题（还没过宽限时间的也在里面）。level：crit / warn；go：有它时不能出发
  function raw(){
    const out = [], add = (id, level, title, detail, fix, grace = 3, go = false) => out.push({ id, level, title, detail, fix, grace, go });
    if (store.source === 'ros' && !store.link.ok){
      // 和后端断了：别的状态都不是最新的，只报这一条
      add('link', 'crit', '和后端断开', '网页收不到扩展坞的数据，页面上的状态都不是最新的。', '检查扩展坞有没有开机、网络通不通；连上后自动恢复。', 3, true);
      return out;
    }
    const C = store.ctrl;
    if (C?.on && C.state == null){
      add('ctrl', 'crit', '运控离线',
        C.age == null ? '后端启动以来没有收到运控状态（/go2w_ctrl/state）。' : `已经 ${Math.round(C.age)} 秒没有收到运控状态（/go2w_ctrl/state）。`,
        '不能站起、不能出发。在扩展坞上检查：sudo systemctl status go2w-ctrl', 2, true);
    }
    const now = performance.now() / 1000;
    let stackBad = false;
    for (const m of Object.keys(MODE_NODES)){
      const p = store.profiles[m];
      if (p === 'starting'){ startT[m] ??= now; if (now - startT[m] > START_SLOW) add(`slow_${m}`, 'warn', `${MODES[m]}启动太慢`, `已经 ${Math.round(now - startT[m])} 秒还没就绪。`, '看「日志」里的报错；可以停止后重新启动。', 0); }
      else delete startT[m];
      if (p !== 'run') continue;
      const missing = MODE_NODES[m].filter(id => nodeUp(id) === false);
      if (missing.length){
        stackBad = true;
        const lidar = missing.includes('livox_lidar_publisher');
        add(`nodes_${m}`, 'crit', lidar ? '雷达驱动没有运行' : `${MODES[m]}缺少节点`,
          `${MODES[m]}在运行，但这些节点不在：${missing.map(nodeName).join('、')}。`,
          '看「日志」里的报错；停止后重新启动这个档位。', 5, true);
      }
    }
    const anyMode = Object.keys(MODE_NODES).some(run);
    if (anyMode && !stackBad && nodeUp('livox_lidar_publisher') && hz('/scan') === 0){
      stackBad = true;
      add('scan', 'crit', '没有激光数据', '雷达驱动在运行，但 /scan 没有数据。', '检查 MID360 的网线和供电；雷达刚上电要等十几秒。', 4, true);
    }
    if (anyMode && !stackBad && !store.odomOk){
      add('odom', 'crit', '里程计不可用', 'FAST-LIO 没有输出，或 /odom_healthy 报告不健康。', '机器人保持静止等一会儿；不行就停止后重新启动这个档位。', 4, true);
    }
    const rb = store.rollback;
    if (anyMode && rb?.on && rb.run === false){
      add('rollback', 'warn', '回溯录制没在运行', '出问题时没法保存之前 20 秒的数据。', '看「日志」里的报错；可以在录包面板里关掉再打开回溯。', 15);
    }
    return out;
  }

  function paint(){
    const A = store.alerts;
    bar.hidden = !A.length;
    document.body.classList.toggle('has-alert', !!A.length);
    if (!A.length) return;
    const top = A[0];
    bar.dataset.s = top.level;
    $('#alertTitle').textContent = top.title;
    $('#alertDetail').textContent = top.detail;
    $('#alertFix').textContent = top.fix;
    more.hidden = A.length < 2;
    more.textContent = expanded ? '收起' : `还有 ${A.length - 1} 项`;
    list.hidden = !expanded || A.length < 2;
    if (!list.hidden) list.replaceChildren(...A.slice(1).map(a => {
      const li = document.createElement('li'); li.dataset.s = a.level;
      const b = document.createElement('b'); b.textContent = a.title;
      const sp = document.createElement('span'); sp.textContent = `${a.detail} ${a.fix}`;
      li.append(b, sp); return li;
    }));
    document.documentElement.style.setProperty('--alert-h', `${bar.offsetHeight + 8}px`);
  }

  return function render(){
    const now = performance.now() / 1000, cur = raw(), ids = new Set(cur.map(a => a.id));
    for (const id of first.keys()) if (!ids.has(id)) first.delete(id);
    for (const a of cur) if (!first.has(a.id)) first.set(a.id, now);
    const next = new Map(cur.filter(a => now - first.get(a.id) >= a.grace || shown.has(a.id)).map(a => [a.id, a]));
    // 刚出现 / 刚恢复：提示一次，写进网页日志（运控离线后端也会记）
    const fresh = [...next.values()].filter(a => !shown.has(a.id));
    for (const a of fresh){ if (a.id !== 'ctrl') bus.emit('log', a.level === 'crit' ? 'ERROR' : 'WARN', 'web', `${a.title}：${a.detail}`); }
    if (fresh.length){ const a = fresh.find(x => x.level === 'crit') || fresh[0]; toast(`${a.title}${fresh.length > 1 ? ` 等 ${fresh.length} 项` : ''}：详见地图上方提示`, a.level === 'crit' ? 'err' : 'warn'); }
    const gone = [...shown.values()].filter(a => !next.has(a.id));
    for (const a of gone){ if (a.id !== 'ctrl') bus.emit('log', 'INFO', 'web', `${a.title}：已恢复`); }
    if (gone.length && !fresh.length) toast(gone.length === 1 ? `${gone[0].title}：已恢复` : `${gone.length} 项问题已恢复`, 'ok');
    shown = next;
    store.alerts = [...next.values()].sort((a, b) => (a.level === 'crit' ? 0 : 1) - (b.level === 'crit' ? 0 : 1));
    paint();
  };
}
