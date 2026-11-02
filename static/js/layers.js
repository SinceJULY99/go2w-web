import { ACC, TRAJ } from './util.js';

// 图层清单。ros：真机上的数据来源；没有 ros 字段的图层真机上不可用，note 说明原因
// heavy：数据量大，只有打开时后端才计算和发送（见 go2w_web.py 的 HEAVY）
export const LAYERS = [
  { id:'map', name:'栅格地图', topic:'/map', c:'#5acdd6', on:true, ros:'/map' },
  { id:'gcost', name:'全局代价地图', topic:'/global_costmap/costmap', c:'#ff4fa3', on:false, ros:'/global_costmap/costmap', heavy:true },
  { id:'lcost', name:'局部代价地图', topic:'/local_costmap · 6×6 m', c:'#c964d6', on:true, ros:'/local_costmap · 6×6 m', heavy:true },
  { id:'cloud', name:'点云（俯视投影）', topic:'/cloud_registered_body', c:'grad', on:true, ros:'/cloud_registered_body · 0.3–1.6 m', heavy:true },
  { id:'scan', name:'激光', topic:'/scan', c:'#ffffff', on:true, ros:'/scan' },
  { id:'plan', name:'全局路径', topic:'/plan', c:ACC, on:true, ros:'/plan' },
  { id:'local', name:'局部轨迹', topic:'/local_plan', c:TRAJ, on:true, ros:'按 /cmd_vel 外推 1.6 s' },
  { id:'mppi', name:'MPPI 采样', topic:'controller_server · 600', c:'rgba(178,140,255,.5)', on:false,
    note:'真机上没有：nav2.yaml 里 MPPI 的 visualize 是 false（打开会明显多占扩展坞 CPU 和带宽）' },
  { id:'particles', name:'AMCL 粒子', topic:'/particle_cloud', c:'#ffe08a', on:false, ros:'/particle_cloud · 最多 300', heavy:true },
  { id:'trail', name:'里程轨迹', topic:'base_footprint', c:'#8aa0b8', on:true, ros:'base_footprint' },
  { id:'rings', name:'测距环', topic:'1 / 2 / 4 m', c:'rgba(255,255,255,.4)', on:true, ros:'1 / 2 / 4 m' },
  { id:'tf', name:'TF 坐标轴', topic:'map · base_link', c:'#ff6b6b', on:false, ros:'map · base_link' },
];
export const L = Object.fromEntries(LAYERS.map(l => [l.id, l]));
export const on = id => L[id].on;
