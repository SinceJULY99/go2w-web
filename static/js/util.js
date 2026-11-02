// 通用小工具，界面和数据源都用
export const $ = s => document.querySelector(s);
export const $$ = s => [...document.querySelectorAll(s)];
export const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
export const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
export const deg = r => r * 180 / Math.PI;
export const fmt = (v, n = 2) => (v >= 0 ? ' ' : '') + v.toFixed(n);
export const gauss = () => { let u = 0; while (!u) u = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random()); };
export function mulberry32(a){ return function(){ a|=0; a=a+0x6D2B79F5|0; let t=Math.imul(a^a>>>15,1|a); t=t+Math.imul(t^t>>>7,61|t)^t; return ((t^t>>>14)>>>0)/4294967296; }; }
export const fmtDur = s => { s = Math.max(0, s|0); const h = s/3600|0, m = (s%3600)/60|0, ss = s%60;
  return h ? `${h}:${String(m).padStart(2,'0')}:${String(ss).padStart(2,'0')}` : `${String(m).padStart(2,'0')}:${String(ss).padStart(2,'0')}`; };
export const tstr = (d = new Date()) => d.toTimeString().slice(0,8) + '.' + String(d.getMilliseconds()).padStart(3,'0');

export const ACC = '#38e1dc', TRAJ = '#b28cff';
export const CLOUD = ['#ff4fa3','#e95ab9','#c964d6','#a86bff','#8a8fff','#6fb6ff'];

// 极简事件总线：log / toast
export class Bus {
  constructor(){ this.h = {}; }
  on(e, f){ (this.h[e] ??= []).push(f); }
  emit(e, ...a){ (this.h[e] || []).forEach(f => f(...a)); }
}

// costmap_2d 发布的代价（uint8：0 空闲，1..98 膨胀代价，99 内切，100 致命，255 未知）→ 半透明画布
// 和模拟模式的配色一致：越危险越偏洋红、越不透明；空闲和未知透明
export function costToCanvas(grid, w, h){
  const cv = document.createElement('canvas'); cv.width = w; cv.height = h;
  const c = cv.getContext('2d'), im = c.createImageData(w, h);
  for (let y=0;y<h;y++) for (let x=0;x<w;x++){
    const v = grid[y*w+x]; if (!v || v===255) continue;
    const p = ((h-1-y)*w+x)*4, t = Math.min(v, 100)/100;
    im.data[p]=Math.round(140+115*t); im.data[p+1]=Math.round(80*(1-t)+31.6*t); im.data[p+2]=Math.round(255-92*t);
    im.data[p+3]=v>=99 ? 150 : Math.round(25+110*t);
  }
  c.putImageData(im, 0, 0);
  return cv;
}

// OccupancyGrid（uint8：0..100 占用，255 未知，行 0 在 origin 一侧）→ 画布；颜色和原型一致
export function gridToCanvas(grid, w, h){
  const cv = document.createElement('canvas'); cv.width = w; cv.height = h;
  const c = cv.getContext('2d'), im = c.createImageData(w, h);
  for (let y=0;y<h;y++) for (let x=0;x<w;x++){
    const v = grid[y*w+x], p = ((h-1-y)*w+x)*4, n = ((x*7+y*13)%5);
    const col = v===255 ? [7,11,17] : v>=65 ? [90,206,214] : v>25 ? [40,90,100] : [13+n,23+n,34+n];
    im.data[p]=col[0]; im.data[p+1]=col[1]; im.data[p+2]=col[2]; im.data[p+3]=255;
  }
  c.putImageData(im, 0, 0);
  return cv;
}
