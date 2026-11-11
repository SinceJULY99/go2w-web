// 用户操作：先按 store 做前置检查、必要时弹确认，再交给数据源执行。两种数据源共用这套规则。
// 数据源的指令可以返回 { ok, msg }（或它的 Promise）：ROS 源以后端的回复为准，模拟源不返回就算成功
import { toast, confirmBox, formBox } from './ui.js';
import { deg } from './util.js';

export const MODES = { mapping: '建图', localization: '定位', navigation: '导航' };
export const MODE_PKG = { mapping: 'slam_toolbox', localization: 'map_server · AMCL', navigation: 'AMCL · Nav2 · cmd_bridge' };

export function createActions(store, src){
  const run = id => store.profiles[id] === 'run';
  const curMode = () => Object.keys(MODES).find(m => store.profiles[m] !== 'stop') || null;
  const canMove = () => store.armed && !store.estop && run('navigation') && store.odomOk && store.link.ok;
  // 能不能遥控（不管解没解锁）：导航档的速度桥，或不开导航时后端拉起的遥控速度桥（不查里程计）
  // 机器狗在策略（Velocity）里、能走；没接运控（ctrl.on = false）时不管
  const walking = () => !store.ctrl?.on || (store.ctrl.state === 'Velocity' && store.ctrl.ready);
  const canTeleop = () => store.link.ok && !store.estop && ((run('navigation') && store.odomOk) || run('bridge'));
  function routeStartBlocker(route){
    if (!route) return '请先保存或选择一个任务。';
    if (!store.link.ok) return '等待连接后端。';
    if (store.task.status === 'active' || store.profiles.patrol !== 'stop') return '请先结束当前导航或巡逻。';
    if (store.estop) return '请先解除急停。';
    if (store.busy) return '等待运行档位切换完成。';
    if (route.map !== store.mapCurrent) return `请先切换到路线地图 ${route.map}。`;
    if (!run('navigation')) return '先完成第 3 步「出发前自检」（会自动启动导航）。';
    if (!store.odomOk || !store.robot.valid || store.fixed !== 'map') return '等待地图定位和里程计就绪。';
    if (!store.initPose) return '先完成第 3 步「出发前自检」（定位还没完成）。';
    if (!walking()) return '机器狗没有站起来：重新做第 3 步「出发前自检」。';
    if (store.prepOk && !store.prepOk(route)) return '先完成第 3 步「出发前自检」。';
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
    canMove, canTeleop, curMode, routeStartBlocker, walking,
    // 站起：趴着 → 站稳 → 策略接管（后端一步步等运控的状态）
    standUp(){
      if (!ready('standUp')) return Promise.resolve({ ok: false });
      if (store.estop) return Promise.resolve((toast('急停中：先解除急停', 'err'), { ok: false, msg: '急停中' }));
      return exec(src.cmd.standUp(), null);
    },
    // 趴下：1005 是卸力直接塌下去。任务、导航会先取消，底盘上锁，等停稳再趴
    lieDown(){
      if (!ready('lieDown')) return;
      const moving = store.task.type !== 'idle' && store.task.status === 'active';
      confirmBox({ title: '趴下', ok: '趴下', danger: true,
        body: `${moving ? '当前任务会被取消、底盘上锁，停稳后' : ''}机器狗原地卸力趴下（直接塌下去，不是慢慢坐下）。确认周围没人、地面平整。`,
        onOk: () => exec(src.cmd.lieDown(), null) });
    },
    standHold(on){ if (ready('standHold')) exec(src.cmd.standHold(on), null); },
    // fine = 重定位的微调：后端用小协方差，AMCL 不会把粒子撒开
    initialPose(p, fine = false){
      const no = msg => (toast(msg, 'err'), { ok: false, msg });
      if (!ready('initialPose')) return Promise.resolve({ ok: false });
      if (!run('localization') && !run('navigation')) return Promise.resolve(no('定位没有运行，无法设置初始位姿'));
      if (!fine && store.costAt && blocked(store.costAt(p.x, p.y))) return Promise.resolve(no('该位置在障碍物上'));
      return exec(src.cmd.initialPose(p, fine), fine ? null : '初始位姿已发布，粒子收敛中').then(r => { if (r?.ok !== false) store.onInitialPose?.(); return r; });
    },
    // 自动定位：后端用当前 scan 在整张地图上找机器人，可信才发给 AMCL（不可信只报告，不改位姿）
    autoLocalize(){
      if (!ready('autoLocalize')) return Promise.resolve({ ok: false });
      if (!run('localization') && !run('navigation')) return Promise.resolve((toast('定位没有运行', 'err'), { ok: false }));
      if (store.task.type !== 'idle' && store.task.status === 'active') return Promise.resolve((toast('任务进行中：先取消再自动定位', 'err'), { ok: false }));
      toast('自动定位中：机器人保持静止，约需 5–10 秒', 'warn');
      return exec(src.cmd.autoLocalize(), null);
    },
    routePoint(p){
      // 导航没开时地图是后端直接读的文件，点的坐标同样是地图坐标，可以照常选点
      const draft = store.routeDraft, map = store.mapCurrent;
      if (!map || !store.map) return toast('还没有地图：先在第 1 步选择地图', 'err');
      if (draft.map && draft.map !== map) return toast('路线属于另一张地图，请先新建路线', 'err');
      if (draft.points.length >= 50) return toast('每条路线最多 50 个点', 'err');
      if (store.costAt && blocked(store.costAt(p.x, p.y))) return toast('该位置在障碍物或膨胀区内', 'err');
      const last = draft.points.at(-1);
      if (last && Math.hypot(p.x - last.x, p.y - last.y) < .05) return toast('这一站已添加', 'warn');
      store.routeDraft = { ...draft, map, points: [...draft.points, p], confirmed: false };
      store.onRouteEdit?.();   // 右栏展开第 2 步
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
    // 「出发」：第 3 步自检已经把地图、定位、路线都核对过，点一下就自动解锁出发（后端 route_start 带 arm）。
    // opts.vmax / wmax：第 4 步选的速度上限，后端出发前设给 cmd_bridge。
    // 只有激光和地图对不太上（匹配率 < 60%）时才多问一句
    startRoute(route, opts = {}){
      const reason = routeStartBlocker(route);
      if (reason) return toast(reason, 'err');
      const go = () => exec(src.cmd.startRoute(route.id, { ...opts, arm: true }), '底盘已解锁，任务已出发');
      const R = store.robot, m = store.loc.match;
      if (m == null || m >= 60) return go();
      return new Promise(resolve => formBox({ title: '定位可能不准', ok: '仍然出发', danger: true,
        rows: [['当前位置', `(${R.x.toFixed(2)}, ${R.y.toFixed(2)}) ${Math.round(deg(R.th)) || 0}°`], ['匹配率', `${m.toFixed(0)}%`]],
        warn: `激光和地图的匹配率只有 ${m.toFixed(0)}%，建议先在第 3 步重新定位。`,
        onCancel: () => resolve(null), onOk: () => resolve(go()) }));
    },
    cancel(){ if (ready('cancel')) exec(src.cmd.cancel(), '任务已取消'); },
    pause(){ if (ready('pause')) src.cmd.pause(!src.paused); },
    arm(){
      if (!ready('arm') || store.estop) return;
      if (store.armed) return exec(src.cmd.arm(false), '底盘已锁定');
      if (!run('navigation') && !run('bridge')) return toast('速度桥不在线（刚开机或刚切换档位时要等几秒）', 'err');
      if (run('navigation') && !store.odomOk) return toast('里程计不健康，不能解锁', 'err');
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
        : '这个页面上还没有保存过当前地图，停止建图后地图会丢失。要保存就点「取消」，在右栏「建图」第 4 步保存。';
      const extra = [run('patrol') && '巡逻（命令行启动的 patrol.py）'].filter(Boolean);
      const body = m
        ? `「${MODES[m]}」和「${MODES[cur]}」不能同时运行。将先停止${MODES[cur]}${extra.length ? '和' + extra.join('、') : ''}，再启动${MODES[m]}。正在执行的任务会取消，底盘会锁定。${mapNote}${still}`
        : `将停止${MODES[cur]}${extra.length ? '和' + extra.join('、') : ''}。正在执行的任务会取消，底盘会锁定。`;
      confirmBox({ title: m ? `切换到「${MODES[m]}」` : '停止全部', body: mapWarn + body, danger: true,
        ok: leavingMap && !saved ? (m ? '不保存，继续切换' : '不保存，继续停止') : m ? '切换' : '停止',
        onOk: () => { if (m === 'mapping') store.mapSavedAt = null; exec(src.cmd.setMode(m), null); } });
    },
    toggle(id){
      if (!ready('toggle')) return;
      exec(src.cmd.toggle(id), null);
    },
    async saveMap(name){
      if (!ready('saveMap')) return;
      if (store.mapSaving || store.mapBusy) return toast('地图正在保存，请稍等', 'warn');
      if (!run('mapping')) return toast('保存地图需要「建图」正在运行', 'err');
      store.mapSaving = true;
      try {
        const result = await exec(src.cmd.saveMap(name), `地图已保存：${name}`);
        if (!result || result.ok !== false){ store.mapSavedAt = Date.now(); store.mapSavedName = name; }
        return result;
      } finally { store.mapSaving = false; }
    },
    // 地图显示方向：存到后端（按地图名），失败才提示
    setMapView(map, angle){
      if (!src.cmd.setMapView) return;
      return Promise.resolve(src.cmd.setMapView(map, angle)).then(r => {
        if (r && r.ok === false) toast(r.msg || '保存地图方向失败', 'err');
        return r;
      });
    },
    async saveAlignedMap(name){
      if (!ready('alignMap') || store.mapSaving || store.mapBusy) return;
      if (!run('mapping')) return toast('保存对齐地图需要「建图」正在运行', 'err');
      if (!/^[A-Za-z0-9_-]{1,40}$/.test(name)) return toast('原图名限 1–40 个字母、数字、下划线或减号', 'err');
      const alignment = store.alignment;
      // 按当前显示方向导出：图片最多转 ±90°，剩下的 180° 记成新地图的显示方向
      const view = alignment.angle, angle = Math.atan(Math.tan(view));
      if (Math.abs(angle) < 1e-4) return toast('当前显示方向已经是水平 / 竖直，不需要导出对齐地图', 'warn');
      const rest = Math.atan2(Math.sin(view - angle), Math.cos(view - angle));
      store.mapSaving = true;
      try {
        if (alignment.source !== name){
          const result = await exec(src.cmd.saveMap(name), null);
          if (result && result.ok === false) return result;
          alignment.source = name;
          store.mapSavedAt = Date.now();
        }
        const result = await exec(src.cmd.alignMap(name, name + '_aligned', angle), null);
        if (result?.ok !== false && Math.abs(rest) > 1e-4) await A.setMapView(name + '_aligned', rest);
        return result;
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
    deleteBag(bag){
      if (!ready('deleteBag')) return;
      const size = bag.size > 1024 ? (bag.size/1024).toFixed(1) + ' GB' : bag.size.toFixed(0) + ' MB';
      confirmBox({ title:'删除录包', body:`确定删除 data/bags/${bag.name}（${size}）？删除后无法恢复。`, ok:'删除', danger:true,
        onOk: () => exec(src.cmd.deleteBag(bag.name), `已删除录包 ${bag.name}`) });
    },
    setRollback(on){
      if (!ready('setRollback')) return;
      store.rollback = { ...store.rollback, on };
      exec(src.cmd.setRollback(on), null);
    },
    saveRollback(){
      if (!ready('saveRollback')) return;
      exec(src.cmd.saveRollback(), null);
    },
    uploadBag(bag){
      if (!ready('uploadBag')) return;
      const u = store.uploads[bag.name];
      if (u?.state === 'done') return confirmBox({ title:'重新上传', body:`${bag.name} 已在 ${u.msg} 上传过。再传一次会补齐 166 上缺的部分，已有的文件不重复传。`, ok:'上传',
        onOk: () => exec(src.cmd.uploadBag(bag.name), null) });
      exec(src.cmd.uploadBag(bag.name), null);
    },
    simulateDrop(){ if (ready('simulateDrop')) exec(src.cmd.simulateDrop(), null); },
  };
  return A;
}
