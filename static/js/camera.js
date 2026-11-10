// 摄像头浮窗：Go2W 前置摄像头（H.264，后端原样转发），用 jmuxer 封装成 MSE 给 <video> 播放。
// 浮窗在地图区里，默认关着；地图右上角的摄像头按钮打开 / 关闭，标题栏 ✕ 关闭。
// 标题栏拖动换位置，右下角拖动按画面比例缩放；位置和大小记在本机浏览器里。
// 只在浮窗开着、浏览器标签页在前台时才向后端要画面；关掉就停，省扩展坞的 Wi-Fi。
// 后端同一时间只给一个网页传画面，别人在看时显示是谁，可以「接管」。
import { $ } from './util.js';

const LAG_MAX = 0.5;          // s：播放落后缓冲这么多就跳到最新一帧
const MARGIN = 8;             // px：浮窗离地图区边缘至少这么远
const W_MIN = 200, W_DEFAULT = 320;
const KEY = 'go2w.camWin';    // localStorage：{ x, y, w }，相对地图区左上角

export function setupCamera(src, bus){
  const stage = $('#stage'), win = $('#camFloat'), head = $('#camHead'), grip = $('#camGrip');
  const btn = $('#vCam'), video = $('#camVideo'), msg = $('#camMsg'), info = $('#camInfo'), take = $('#camTake');
  const show = text => { msg.hidden = !text; if (text) msg.textContent = text; };
  let ready = false, want = false, mux = null, stats = null, frames = 0;

  /* ---------- 浮窗：开关、位置、大小 ---------- */
  let open = false, ar = 16 / 9, geo = null;     // geo：用户最后摆放的位置；显示时再按地图区大小夹住
  try { const g = JSON.parse(localStorage.getItem(KEY)); if (g && isFinite(g.x) && isFinite(g.y) && isFinite(g.w)) geo = g; } catch {}
  const save = () => { try { geo ? localStorage.setItem(KEY, JSON.stringify(geo)) : localStorage.removeItem(KEY); } catch {} };

  // 标题栏 + 信息栏的高度（不随宽度变）
  const chrome = () => head.offsetHeight + info.parentElement.offsetHeight;
  const heightOf = w => w / ar + chrome();
  function maxW(){
    const sw = stage.clientWidth, sh = stage.clientHeight;
    return Math.max(W_MIN, Math.min(sw - 2 * MARGIN, (sh - 2 * MARGIN - chrome()) * ar));
  }
  function defaultGeo(w){
    // 左下角，摞在比例尺 / 坐标那块上方
    const bl = stage.querySelector('.hud-bl'), sh = stage.clientHeight;
    const above = bl ? sh - bl.offsetTop + 8 : 66;
    return { x: 14, y: sh - above - heightOf(w), w };
  }
  // 把 g 夹到地图区里面，然后写到浮窗上
  function place(g){
    const w = Math.max(W_MIN, Math.min(g.w, maxW())), h = heightOf(w);
    const x = Math.max(MARGIN, Math.min(g.x, stage.clientWidth - w - MARGIN));
    const y = Math.max(MARGIN, Math.min(g.y, stage.clientHeight - h - MARGIN));
    win.style.width = w + 'px'; win.style.left = x + 'px'; win.style.top = y + 'px';
    return { x, y, w };
  }
  const layout = () => { if (open) place(geo || defaultGeo(W_DEFAULT)); };

  function setOpen(v){
    open = v; win.hidden = !v;
    btn.classList.toggle('on', v); btn.setAttribute('aria-pressed', String(v));
    btn.setAttribute('aria-label', v ? '关闭摄像头浮窗' : '打开摄像头浮窗');
    layout(); if (ready) update();
  }
  btn.onclick = () => setOpen(!open);
  $('#camClose').onclick = () => setOpen(false);
  new ResizeObserver(layout).observe(stage);

  function setAspect(next){
    if (!(next > 0.2 && next < 5) || Math.abs(next - ar) < 1e-3) return;
    ar = next; win.style.setProperty('--ar', String(ar)); layout();
  }
  video.addEventListener('resize', () => { if (video.videoWidth) setAspect(video.videoWidth / video.videoHeight); });

  // 拖标题栏：移动
  head.addEventListener('pointerdown', e => {
    if (e.button !== 0 || e.target.closest('button')) return;
    e.preventDefault(); head.setPointerCapture(e.pointerId); win.classList.add('moving');
    const x0 = e.clientX, y0 = e.clientY, l0 = win.offsetLeft, t0 = win.offsetTop, w = win.offsetWidth;
    const mv = ev => { geo = place({ x: l0 + ev.clientX - x0, y: t0 + ev.clientY - y0, w }); };
    const up = () => { head.removeEventListener('pointermove', mv); head.removeEventListener('pointerup', up); head.removeEventListener('pointercancel', up);
      win.classList.remove('moving'); geo = place({ x: win.offsetLeft, y: win.offsetTop, w: win.offsetWidth }); save(); };
    head.addEventListener('pointermove', mv); head.addEventListener('pointerup', up); head.addEventListener('pointercancel', up);
  });
  // 拖右下角：左上角不动，按比例缩放（横向、纵向哪个拖得多听哪个）
  grip.addEventListener('pointerdown', e => {
    if (e.button !== 0) return;
    e.preventDefault(); e.stopPropagation(); grip.setPointerCapture(e.pointerId); win.classList.add('moving');
    const x0 = e.clientX, y0 = e.clientY, l = win.offsetLeft, t = win.offsetTop, w0 = win.offsetWidth;
    const mv = ev => {
      const dx = ev.clientX - x0, dy = (ev.clientY - y0) * ar;
      const w = Math.min(w0 + (Math.abs(dx) >= Math.abs(dy) ? dx : dy), stage.clientWidth - l - MARGIN,
        (stage.clientHeight - t - MARGIN - chrome()) * ar);
      geo = place({ x: l, y: t, w });
    };
    const up = () => { grip.removeEventListener('pointermove', mv); grip.removeEventListener('pointerup', up); grip.removeEventListener('pointercancel', up);
      win.classList.remove('moving'); save(); };
    grip.addEventListener('pointermove', mv); grip.addEventListener('pointerup', up); grip.addEventListener('pointercancel', up);
  });
  grip.ondblclick = () => { geo = null; save(); layout(); };

  /* ---------- 画面 ---------- */
  if (!src.camera || typeof JMuxer === 'undefined'){
    show(src.camera ? '播放器没有加载（js/vendor/jmuxer.min.js）' : '模拟模式没有摄像头画面');
    return;
  }
  ready = true;

  const visible = () => open && !document.hidden && !document.body.classList.contains('demo');

  function startMux(){
    stopMux();
    frames = 0;
    mux = new JMuxer({ node: video, mode: 'video', flushingTime: 0, fps: 15, clearBuffer: true, debug: false,
      onError: () => { if (want) startMux(); } });   // 缓冲出错：重建播放器，等下一个关键帧
  }
  function stopMux(){
    if (mux){ try { mux.destroy(); } catch {} mux = null; }
    video.removeAttribute('src'); video.load();
  }

  function update(){
    const v = visible();
    if (v !== want){
      want = v;
      take.hidden = true;
      if (v){ startMux(); show('正在连接摄像头…'); src.camera.set(true); }
      else { src.camera.set(false); stopMux(); show('浏览器标签页在后台，已暂停传输画面'); stats = null; info.textContent = '同一时间只给一个网页传画面'; }
    }
    if (!mux) return;
    const b = video.buffered;
    if (b.length){
      const end = b.end(b.length - 1);
      if (end - video.currentTime > LAG_MAX) video.currentTime = end - 0.05;
      if (video.paused) video.play().catch(() => {});
    }
    if (stats){
      const lag = b.length ? Math.max(0, b.end(b.length - 1) - video.currentTime) : null;
      const size = stats.size ? `${stats.size[0]}×${stats.size[1]}` : '--';
      info.textContent = `${size} · ${stats.fps} 帧/秒 · ${(stats.kbps / 1000).toFixed(2)} Mbit/s` +
        (lag != null ? ` · 缓冲 ${lag.toFixed(1)} s` : '') + (stats.dropped ? ` · 网络跟不上，跳过 ${stats.dropped} 次` : '');
      info.title = info.textContent;
    }
  }

  bus.on('camFrame', (meta, data) => {
    if (!mux) return;
    mux.feed({ video: data, duration: meta.dur || undefined });
    if (++frames === 1) show('');
  });
  bus.on('cam', m => {
    if (!want) return;
    take.hidden = !['busy', 'taken'].includes(m.state);
    if (m.size && m.size[1]) setAspect(m.size[0] / m.size[1]);
    switch (m.state){
      case 'starting': startMux(); show('正在等待关键帧（最多约 2 秒）…'); break;
      case 'on': stats = m; if (!frames) show('正在等待关键帧（最多约 2 秒）…'); break;
      case 'nosignal': stats = m; show('没有收到摄像头画面：主控没有发出视频，或扩展坞的有线口断开'); break;
      case 'busy': stopMux(); show(`摄像头正在被 ${m.by} 查看（同一时间只给一个网页传画面）`); break;
      case 'taken': stopMux(); stats = null; show(`画面已被 ${m.by} 接管`); break;
      case 'error': stopMux(); show(m.msg || '摄像头出错'); break;
      case 'disconnected': stats = null; show('和后端的连接断开了，重连后自动恢复'); break;
    }
  });
  take.onclick = () => { take.hidden = true; startMux(); show('正在接管…'); src.camera.set(true, true); };
  document.addEventListener('visibilitychange', update);
  setInterval(update, 250);
}
