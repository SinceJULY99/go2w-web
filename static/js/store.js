// 界面只读这个 store；模拟源和 ROS 源负责把它填满。字段为 null 表示“没有数据”，界面显示 --
export function createStore(){
  return {
    source: 'mock',                 // 'mock' | 'ros'
    link: { ok: false, ms: null },  // 浏览器 ↔ 后端
    fixed: 'map',                   // 位姿和 scan 所在坐标系（没有 map TF 时后端退回 odom）
    // 地图：img 画布覆盖世界坐标 [ox, ox+w] × [oy, oy+h]（米）
    map: null,                      // { img, w, h, ox, oy, res, name, cells }
    costmap: null,                  // 同上，全局代价地图
    lcostmap: null,                 // 局部代价地图（真机：odom 下的滚动窗口，带 yaw）；没有时用全局的裁一块（模拟模式）
    costAt: null,                   // (x, y) => 0..254，用于光标读数和目标校验
    robot: { valid: false, x: 0, y: 0, th: 0, v: 0, vy: 0, w: 0 },
    cmd: { v: 0, vy: 0, w: 0 },
    // 速度诊断（仅 ROS）：{ wall: [vx, vy, wz] 按真实时间, stamp: [...] 按消息时间戳, rate: 时间戳走速 / 真实时间 }
    speedCheck: null,
    imu: { roll: null, pitch: null },
    scan: new Float32Array(0),      // [x, y, ...]（fixed 坐标系）
    cloud: [],                      // 按高度分桶，每桶 [x, y, ...]
    plan: null,                     // { pts: [[x,y]], s: [], total }
    planIdx: 0,
    localPlan: null, mppi: [], particles: new Float32Array(0),
    trail: [],
    goal: null,
    routes: [],
    routeDraft: { id: null, map: null, points: [], confirmed: false },
    routeCheck: null,
    alignment: { points: [], angle: 0, target: null, source: null },
    mapSaving: false,
    mapBusy: false,
    patrol: { A: null, B: null, loops: 5 },
    // task.phase: plan | align | follow | final | dwell | arrived
    task: { type: 'idle' },
    armed: null, estop: false, odomOk: false,
    loc: { running: false, std: null, yaw: null, match: null },
    // 运行档位：mode 三选一，patrol / bag 可叠加
    profiles: { mapping: 'stop', localization: 'stop', navigation: 'stop', patrol: 'stop', bag: 'stop' },
    since: {},                      // 档位启动时间（秒）
    nodes: [],                      // [{ id, name, up }]
    sys: { cpu: null, mem: null, memTotal: null },
    topics: [],                     // [{ name, type, hz, alive, d0 }]
    params: { vmax: .45, wmax: 1.2, tmo: .3 },
    maps: [], bags: [],
    odo: null, battery: null,
  };
}
