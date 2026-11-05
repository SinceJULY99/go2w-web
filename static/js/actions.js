// 用户操作：先按 store 做前置检查、必要时弹确认，再交给数据源执行。两种数据源共用这套规则。
// 数据源的指令可以返回 { ok, msg }（或它的 Promise）：ROS 源以后端的回复为准，模拟源不返回就算成功
import { toast, confirmBox } from './ui.js';

export const MODES = { mapping: '建图', localization: '定位', navigation: '导航' };
export const MODE_PKG = { mapping: 'slam_toolbox', localization: 'map_server · AMCL', navigation: 'AMCL · Nav2 · cmd_bridge' };

export function createActions(store, src){
  const run = id => store.profiles[id] === 'run';
  const curMode = () => Object.keys(MODES).find(m => store.profiles[m] !== 'stop') || null;
  const canMove = () => store.armed && !store.estop && run('navigation') && store.odomOk && store.link.ok;
  function routeStartBlocker(route){
    if (!route) return '请先保存或选择一个任务。';
    if (!store.link.ok) return '等待连接后端。';
    if (store.task.status === 'active' || store.profiles.patrol !== 'stop') return '请先结束当前导航或巡逻。';
    if (store.estop) return '请先解除急停。';
    if (store.busy) return '等待运行档位切换完成。';
    if (!run('navigation')) return '请先在「运行档位」中启动导航。';
    if (route.map !== store.mapCurrent) return `请先切换到路线地图 ${route.map}。`;
    if (!store.odomOk || !store.robot.valid || store.fixed !== 'map') return '等待地图定位和里程计就绪。';
    if (!store.armed) return '请先在「底盘控制」中解锁。';
    return '';
  }
  const blocked = cost => cost != null && cost >= 253;
  // 数据源还不支持这个操作时（比如真机上的暂停），给出说明并中止
  const ready = op => src.can(op) || (src.cmd.notice(op), false);
  // 执行并提示结果：失败一律显示原因
  const exec = (p, okMsg, okKind = 'ok') => Promise.resolve(p).then(r => {
    if (r && r.ok === false) toast(r.msg || '操作失败', 'err');
    else if (okMsg !== null || (r && r.msg)) toast((r && r.msg) || okMsg, okKind);
    return r;
  });

  const A = {
    canMove, curMode, routeStartBlocker,
    initialPose(p){
      if (!ready('initialPose')) return;
      if (!run('localization') && !run('navigation')) return toast('定位没有运行，无法设置初始位姿', 'err');
      if (store.costAt && blocked(store.costAt(p.x, p.y))) return toast('该位置在障碍物上', 'err');
      exec(src.cmd.initialPose(p), '初始位姿已发布，粒子收敛中');
    },
    goal(g){
      if (!ready('goal')) return;
      if (store.estop) return toast('急停中：先解除急停', 'err');
      if (!run('navigation')) return toast('导航没有运行：先在「运行档位」里启动导航', 'err');
      if (store.costAt && blocked(store.costAt(g.x, g.y))) return toast('目标在障碍物或膨胀区内，请换个位置', 'err');
      exec(src.cmd.goal(g), store.armed ? '目标已下发' : '目标已下发，底盘锁定中，解锁后开始运动', store.armed ? 'ok' : 'warn');
    },
    routePoint(p){
      const draft = store.routeDraft, map = store.mapCurrent;
      if (!map || store.fixed !== 'map') return toast('请先加载地图并等待地图定位', 'err');
      if (draft.map && draft.map !== map) return toast('路线属于另一张地图，请先新建路线', 'err');
      if (draft.points.length >= 50) return toast('每条路线最多 50 个点', 'err');
      if (store.costAt && blocked(store.costAt(p.x, p.y))) return toast('该位置在障碍物或膨胀区内', 'err');
      const last = draft.points.at(-1);
      if (last && Math.hypot(p.x - last.x, p.y - last.y) < .05) return toast('这一站已添加', 'warn');
      store.routeDraft = { ...draft, map, points: [...draft.points, p], confirmed: false };
      document.getElementById('routesPanel').open = true;
    },
    checkRoute(draft){
      if (src.name !== 'ros') return Promise.resolve({ ok:false, msg:'纯前端模拟，尚未用 Nav2 检查路径' });
      return src.cmd.checkRoute({ map:draft.map, points:draft.points });
    },
    saveRoute(name){
      const { id, map, points, confirmed } = store.routeDraft;
      if (!confirmed) return toast('请先确认路线', 'err');
      if (!name.trim() || points.length < 2 || map !== store.mapCurrent) return toast('请在当前地图添加至少两个点，并填写任务名', 'err');
      return exec(src.cmd.saveRoute({ route_id: id, name, map, points }), '任务已保存');
    },
    deleteRoute(route, onDeleted){
      confirmBox({ title:'删除任务', body:`确定删除「${route.name}」？`, ok:'删除', danger:true,
        onOk: async () => {
          const reply = await exec(src.cmd.deleteRoute(route.id), '任务已删除');
          if (reply?.ok) onDeleted();
        } });
    },
    startRoute(route){
      const reason = routeStartBlocker(route);
      if (reason) return toast(reason, 'err');
      return exec(src.cmd.startRoute(route.id), '多点任务已启动');
    },
    patrolPoint(k, p){
      if (!ready('patrolPoint')) return;
      if (store.costAt && blocked(store.costAt(p.x, p.y))) return toast('该位置在障碍物或膨胀区内', 'err');
      exec(src.cmd.setPatrolPoint(k, p), `巡逻点 ${k} 已更新`);
    },
    startPatrol(){
      if (!ready('startPatrol')) return;
      if (!store.patrol.A || !store.patrol.B) return toast('先用 A 点、B 点工具在地图上设置两个巡逻点', 'err');
      if (src.name === 'ros' && !run('navigation')) return toast('导航没有运行：先在「运行档位」里启动导航', 'err');
      if (!run('navigation')) return confirmBox({ title: '开始巡逻', body: '巡逻需要导航在运行。将先启动导航，再启动巡逻。', ok: '启动', onOk: () => src.cmd.startPatrol() });
      exec(src.cmd.startPatrol(), null);
    },
    cancel(){ if (ready('cancel')) exec(src.cmd.cancel(), '任务已取消'); },
    pause(){ if (ready('pause')) src.cmd.pause(!src.paused); },
    arm(){
      if (!ready('arm') || store.estop) return;
      if (store.armed) return exec(src.cmd.arm(false), '底盘已锁定');
      if (!run('navigation')) return toast('导航没有运行，cmd_bridge 不在线', 'err');
      if (!store.odomOk) return toast('里程计不健康，不能解锁', 'err');
      confirmBox({ title: '解锁底盘', body: '解锁后机器人会执行导航和遥控的速度指令。确认机器人周围没有人和障碍物。', ok: '解锁',
        onOk: () => exec(src.cmd.arm(true), '底盘已解锁') });
    },
    estop(){
      if (!ready('estop') || store.estop) return;
      store.estop = true; // 先在界面上锁存，后端的状态随后确认
      Promise.resolve(src.cmd.estop()).then(r => {
        if (r && r.ok === false) toast(store.link.ok ? `急停没有完全生效：${r.msg}` : '急停没有发出去：网页和后端断开了，请用遥控器停车', 'err');
      });
    },
    releaseEstop(){ exec(src.cmd.releaseEstop(), '急停已解除，底盘仍处于锁定', 'warn'); },
    setParam(k, v){ if (ready('setParam')) return exec(src.cmd.setParam(k, v), null); },
    setMode(m){
      if (store.mapSaving || store.mapBusy) return toast('地图正在保存，请稍等', 'warn');
      if (!ready('setMode')) return;
      if (store.busy) return toast('上一个启停还没完成，稍等几秒', 'warn');
      const cur = curMode();
      if (m === cur) return;
      const still = 'FAST-LIO 初始化时机器人要站好、停在平地上不要动。';
      const mapNote = m === 'localization' || m === 'navigation' ? `地图：${store.mapChoice || store.mapCurrent || '默认'}。` : '';
      if (!cur){
        if (!m) return;
        return confirmBox({ title: `启动「${MODES[m]}」`, body: `${mapNote}${still}`, ok: '启动', onOk: () => { if (m === 'mapping') store.mapSavedAt = null; exec(src.cmd.setMode(m), null); } });
      }
      // 离开建图：slam_toolbox 一停，没保存的地图就没了。保存记录只在这个页面里，刷新后算没有
      const leavingMap = cur === 'mapping';
      const saved = store.mapSavedAt;
      const mapWarn = !leavingMap ? '' : saved
        ? `地图上次保存是 ${Math.max(1, Math.round((Date.now() - saved) / 60000))} 分钟前，之后走过的部分会丢失。`
        : '这个页面上还没有保存过当前地图，停止建图后地图会丢失。要保存就点「取消」，到「地图与录包」里保存。';
      const extra = [run('patrol') && '巡逻'].filter(Boolean);
      const body = m
        ? `「${MODES[m]}」和「${MODES[cur]}」不能同时运行。将先停止${MODES[cur]}${extra.length ? '和' + extra.join('、') : ''}，再启动${MODES[m]}。正在执行的任务会取消，底盘会锁定。${mapNote}${still}`
        : `将停止${MODES[cur]}${extra.length ? '和' + extra.join('、') : ''}。正在执行的任务会取消，底盘会锁定。`;
      confirmBox({ title: m ? `切换到「${MODES[m]}」` : '停止全部', body: mapWarn + body, danger: true,
        ok: leavingMap && !saved ? (m ? '不保存，继续切换' : '不保存，继续停止') : m ? '切换' : '停止',
        onOk: () => { if (m === 'mapping') store.mapSavedAt = null; exec(src.cmd.setMode(m), null); } });
    },
    toggle(id){
      if (!ready('toggle')) return;
      if (id === 'patrol' && !run('patrol')) return A.startPatrol();
      exec(src.cmd.toggle(id), null);
    },
    async saveMap(name){
      if (!ready('saveMap')) return;
      if (store.mapSaving || store.mapBusy) return toast('地图正在保存，请稍等', 'warn');
      if (!run('mapping')) return toast('保存地图需要「建图」正在运行', 'err');
      store.mapSaving = true;
      try {
        const result = await exec(src.cmd.saveMap(name), `地图已保存：${name}`);
        if (!result || result.ok !== false) store.mapSavedAt = Date.now();
        return result;
      } finally { store.mapSaving = false; }
    },
    async saveAlignedMap(name){
      if (!ready('alignMap') || store.mapSaving || store.mapBusy) return;
      if (!run('mapping')) return toast('保存对齐地图需要「建图」正在运行', 'err');
      if (!/^[A-Za-z0-9_-]{1,40}$/.test(name)) return toast('原图名限 1–40 个字母、数字、下划线或减号', 'err');
      const alignment = store.alignment;
      if (alignment.points.length !== 2 || !alignment.target) return toast('先沿墙选两点并选择对齐方向', 'err');
      const angle = alignment.angle;
      store.mapSaving = true;
      try {
        if (alignment.source !== name){
          const result = await exec(src.cmd.saveMap(name), null);
          if (result && result.ok === false) return result;
          alignment.source = name;
          store.mapSavedAt = Date.now();
        }
        return await exec(src.cmd.alignMap(name, name + '_aligned', angle), null);
      } finally { store.mapSaving = false; }
    },
    loadMap(name){
      if (store.mapSaving || store.mapBusy) return toast('地图正在保存，请稍等', 'warn');
      if (!ready('loadMap')) return;
      const live = run('localization') || run('navigation');
      if (src.name === 'ros' && !live) return exec(src.cmd.loadMap(name), null);   // 没在跑：只是选好下次启动用的地图
      confirmBox({ title: `切换到地图 ${name}`, body: 'map_server 会立即加载这张地图，正在执行的导航会取消。切换后要重新设置初始位姿。', ok: '切换',
        onOk: () => exec(src.cmd.loadMap(name), src.name === 'mock' ? `已加载 ${name}（模拟模式仍显示示例地图）` : null) });
    },
    simulateDrop(){ if (ready('simulateDrop')) exec(src.cmd.simulateDrop(), null); },
  };
  return A;
}
