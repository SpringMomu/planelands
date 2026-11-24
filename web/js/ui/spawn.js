// "Start position" editor: place the orbiter anywhere (map click / drag, distance + bearing,
// altitude, heading, speed, flight-path angle) and start from there.  The backend answers a
// debounced `spawn_check` with the energy-band reachability the guidance would plan with.
const D2R = Math.PI / 180;
const KEY = 'planelands.spawn.v1';
const VIEW = 85000;                       // half-width of the placement map (m)

const DEFAULTS = { dist_km: 40, bearing_deg: 200, alt_km: 15, heading_mode: 'runway', heading_deg: 20,
  speed_unit: 'auto', speed: 1.0, gamma_deg: -8 };
const UNIT = { auto: { lab: '自动', min: 0, max: 0, step: 1 }, mach: { lab: '马赫', min: 0.3, max: 3, step: 0.01 },
  keas: { lab: 'KEAS', min: 150, max: 450, step: 1 }, tas: { lab: 'm/s', min: 80, max: 900, step: 1 } };

const wrap360 = (a) => ((a % 360) + 360) % 360;

export class SpawnPanel {
  /** root: container element; hud: Hud (terrain image + terrain); link: SimLink; onStart(): episode cleanup */
  constructor({ root, hud, link, world, onStart }) {
    this.root = root; this.hud = hud; this.link = link; this.world = world; this.onStart = onStart;
    this.p = { ...DEFAULTS };
    try { Object.assign(this.p, JSON.parse(localStorage.getItem(KEY) || '{}')); } catch { /* ignore */ }
    this.check = null; this.checkId = 0; this._timer = null; this.vehicle = null;
    this._build();
    this._sync();
    this.requestCheck();
  }

  // ---------------------------------------------------------------- DOM
  _build() {
    const r = this.root;
    r.innerHTML = `
      <canvas class="spawn-map" width="592" height="440" title="单击/拖动：放置位置 · Shift+拖动或右键拖动：设定航向"></canvas>
      <div class="note spawn-hint">单击或拖动地图放置飞机；<b>Shift / 右键拖动</b>设航向。圈距 20 km。</div>
      <div class="row"><label>距跑道</label><input type="range" data-k="dist_km" min="0" max="95" step="0.5"><output data-o="dist_km"></output></div>
      <div class="row"><label>方位（自跑道）</label><input type="range" data-k="bearing_deg" min="0" max="359" step="1"><output data-o="bearing_deg"></output></div>
      <div class="row"><label>高度（海拔）</label><input type="range" data-k="alt_km" min="0.3" max="35" step="0.1"><output data-o="alt_km"></output></div>
      <div class="row seg"><label>航向</label>
        <button data-hm="runway">朝向跑道</button><button data-hm="free">指定</button></div>
      <div class="row hdg-row"><label>航向（真）</label><input type="range" data-k="heading_deg" min="0" max="359" step="1"><output data-o="heading_deg"></output></div>
      <div class="row seg"><label>速度</label>${Object.entries(UNIT).map(([k, u]) => `<button data-su="${k}">${u.lab}</button>`).join('')}</div>
      <div class="row spd-row"><label>速度值</label><input type="range" data-k="speed"><output data-o="speed"></output></div>
      <div class="row"><label>航迹角</label><input type="range" data-k="gamma_deg" min="-25" max="5" step="0.5"><output data-o="gamma_deg"></output></div>
      <div class="note spawn-status">—</div>
      <div class="row btns">
        <button class="accent" data-act="start" title="按上面的参数重置仿真">▶ 从这里开始</button>
        <button data-act="here" title="把飞机当前的位置、高度、航迹和马赫数填进来">取当前状态</button>
        <button data-act="defaults">默认值</button>
      </div>`;
    this.cv = r.querySelector('.spawn-map'); this.g = this.cv.getContext('2d');
    this.status = r.querySelector('.spawn-status');
    r.querySelectorAll('input[data-k]').forEach((el) => el.addEventListener('input', () => {
      this.p[el.dataset.k] = +el.value;
      if (el.dataset.k === 'heading_deg') this.p.heading_mode = 'free';
      this._changed();
    }));
    r.querySelectorAll('[data-hm]').forEach((b) => b.addEventListener('click', () => { this.p.heading_mode = b.dataset.hm; this._changed(); }));
    r.querySelectorAll('[data-su]').forEach((b) => b.addEventListener('click', () => {
      const u = b.dataset.su;
      if (u !== this.p.speed_unit) {
        this.p.speed_unit = u;
        // carry the current speed over when the backend has told us what "auto" means here
        const c = this.check;
        if (c && c.mach_now) this.p.speed = u === 'mach' ? c.mach_now : u === 'keas' ? c.keas_now : u === 'tas' ? c.tas_now : this.p.speed;
        const U = UNIT[u]; if (u !== 'auto') this.p.speed = Math.min(U.max, Math.max(U.min, this.p.speed));
      }
      this._changed();
    }));
    r.querySelector('[data-act="start"]').addEventListener('click', () => this.start());
    r.querySelector('[data-act="here"]').addEventListener('click', () => this.fromVehicle());
    r.querySelector('[data-act="defaults"]').addEventListener('click', () => { this.p = { ...DEFAULTS }; this._changed(); });
    // map interaction
    let drag = null;
    this.cv.addEventListener('contextmenu', (e) => e.preventDefault());
    this.cv.addEventListener('pointerdown', (e) => {
      drag = (e.shiftKey || e.button === 2) ? 'hdg' : 'pos';
      this.cv.setPointerCapture(e.pointerId); this._pointer(e, drag);
    });
    this.cv.addEventListener('pointermove', (e) => { if (drag) this._pointer(e, drag); });
    const up = () => { drag = null; };
    this.cv.addEventListener('pointerup', up); this.cv.addEventListener('pointercancel', up);
  }

  _toPx(e, n) { const W = this.cv.width, H = this.cv.height, s = Math.min(W, H) / (2 * VIEW); return [W / 2 + e * s, H / 2 - n * s]; }
  _fromPx(x, y) { const W = this.cv.width, H = this.cv.height, s = Math.min(W, H) / (2 * VIEW); return [(x - W / 2) / s, -(y - H / 2) / s]; }
  _en() { const d = this.p.dist_km * 1000, b = this.p.bearing_deg * D2R; return [d * Math.sin(b), d * Math.cos(b)]; }

  _pointer(ev, mode) {
    const rc = this.cv.getBoundingClientRect();
    const x = (ev.clientX - rc.left) * this.cv.width / rc.width, y = (ev.clientY - rc.top) * this.cv.height / rc.height;
    const [e, n] = this._fromPx(x, y);
    if (mode === 'pos') {
      this.p.dist_km = Math.round(Math.min(Math.hypot(e, n) / 1000, 95) * 2) / 2;
      this.p.bearing_deg = Math.round(wrap360(Math.atan2(e, n) / D2R));
    } else {
      const [e0, n0] = this._en();
      if (Math.hypot(e - e0, n - n0) < 800) return;
      this.p.heading_deg = Math.round(wrap360(Math.atan2(e - e0, n - n0) / D2R));
      this.p.heading_mode = 'free';
    }
    this._changed();
  }

  _changed() {
    try { localStorage.setItem(KEY, JSON.stringify(this.p)); } catch { /* ignore */ }
    this._sync();
    clearTimeout(this._timer);
    this._timer = setTimeout(() => this.requestCheck(), 220);
  }

  _sync() {
    const p = this.p, r = this.root;
    const U = UNIT[p.speed_unit] || UNIT.auto;
    const sp = r.querySelector('input[data-k="speed"]');
    sp.min = U.min; sp.max = U.max; sp.step = U.step;
    r.querySelectorAll('input[data-k]').forEach((el) => { if (+el.value !== p[el.dataset.k]) el.value = p[el.dataset.k]; });
    const fmt = {
      dist_km: `${p.dist_km.toFixed(1)} km`, bearing_deg: `${Math.round(p.bearing_deg)}°`, alt_km: `${p.alt_km.toFixed(1)} km`,
      heading_deg: `${Math.round(p.heading_deg)}°`, gamma_deg: `${p.gamma_deg.toFixed(1)}°`,
      speed: p.speed_unit === 'mach' ? `M ${p.speed.toFixed(2)}` : p.speed_unit === 'keas' ? `${Math.round(p.speed)} kt` : `${Math.round(p.speed)} m/s`,
    };
    r.querySelectorAll('output[data-o]').forEach((o) => { o.textContent = fmt[o.dataset.o]; });
    r.querySelectorAll('[data-hm]').forEach((b) => b.classList.toggle('on', b.dataset.hm === p.heading_mode));
    r.querySelectorAll('[data-su]').forEach((b) => b.classList.toggle('on', b.dataset.su === p.speed_unit));
    r.querySelector('.hdg-row').style.opacity = p.heading_mode === 'free' ? 1 : 0.45;
    r.querySelector('.spd-row').style.display = p.speed_unit === 'auto' ? 'none' : '';
    this.draw();
  }

  spec() {
    const p = this.p;
    const s = { dist_km: p.dist_km, bearing_deg: p.bearing_deg, alt_km: p.alt_km, gamma_deg: p.gamma_deg,
      heading_deg: p.heading_mode === 'runway' ? null : p.heading_deg, speed_unit: p.speed_unit };
    if (p.speed_unit !== 'auto') s.speed = p.speed;
    return s;
  }

  // ---------------------------------------------------------------- backend
  requestCheck() {
    if (!this.link.connected) { this.check = null; this._status(); return; }
    this.checkId += 1;
    this.link.send({ type: 'sim', action: 'spawn_check', id: this.checkId, spawn: this.spec() });
  }

  onCheck(m) {
    if (m.id !== undefined && m.id !== this.checkId) return;       // stale answer
    this.check = m;
    const v = /Mach ([\d.]+)/.exec(m.desc || '');
    if (v) {
      // remember what the clamped / automatic speed is, for switching units without a jump
      const mach = +v[1], alt = m.pos ? m.pos[2] : this.p.alt_km * 1000;
      const T = alt < 11000 ? 288.15 - 0.0065 * alt : 216.65, a = Math.sqrt(1.4 * 287.05 * T);
      const rho = 1.225 * Math.exp(-alt / 8500);
      m.mach_now = Math.round(mach * 100) / 100; m.tas_now = Math.round(mach * a);
      m.keas_now = Math.round(mach * a * Math.sqrt(rho / 1.225) * 1.94384);
    }
    this._status(); this.draw();
  }

  _status() {
    const c = this.check, el = this.status;
    if (!this.link.connected) { el.innerHTML = '离线预览：只设定位置 / 高度 / 航向（无能量判断）'; el.dataset.s = ''; return; }
    if (!c) { el.textContent = '…'; return; }
    const side = c.side > 0 ? '右' : '左';
    let txt;
    if (c.reachable) {
      txt = `<b>可达</b>：RWY${c.rwy} ${side}侧 HAC${c.loops ? `（绕 ${c.loops} 圈）` : ''}，能量带位置 ${c.band.toFixed(2)}` +
        (c.level ? `，需缩短进近（第 ${c.level} 级）` : '') +
        (c.relaxed ? `<br>能量偏高：拉长航迹（HAC 半径 ${(c.hac_r / 1000).toFixed(1)} km），已避开强逆风航段` : '');
      el.dataset.s = c.level || c.relaxed ? 'warn' : 'ok';
    } else {
      const low = c.note ? c.note.startsWith('LOW ENERGY') : c.e_avail < c.e_min;
      txt = low ? `<b>能量不足</b>：到 RWY${c.rwy} 需 ${(c.e_min / 1000).toFixed(1)} km 能量高度，只有 ${(c.e_avail / 1000).toFixed(1)} km（几何估算）— 制导会尽力飞，可能到不了跑道`
        : c.relaxed && c.note && c.note.includes('headwind')
          ? `<b>能量过高</b>：只有顶着强逆风（平均 ${Math.round(c.head)} m/s）拉长航迹才到得了 RWY${c.rwy}，没有余量 — 制导会尽力飞`
          : `<b>能量过高</b>：离跑道太近，到 RWY${c.rwy} 最多耗散 ${(c.e_max / 1000).toFixed(1)} km 能量高度，现有 ${(c.e_avail / 1000).toFixed(1)} km（几何估算）— 制导会尽力耗散，可能冲过 A/L 接口`;
      el.dataset.s = 'bad';
    }
    const extra = [];
    if (c.desc) {
      const m = /Mach ([\d.]+), qbar ([\d.]+) kPa.*alpha ([\d.]+) deg/.exec(c.desc);
      if (m) extra.push(`马赫 ${m[1]} · 动压 ${m[2]} kPa · 迎角 ${m[3]}°`);
    }
    if (c.agl !== undefined) extra.push(`离地 ${(c.agl / 1000).toFixed(1)} km`);
    for (const w of c.warnings || []) extra.push(`⚠ ${zhWarn(w)}`);
    el.innerHTML = txt + (extra.length ? `<br><span class="dim">${extra.join('<br>')}</span>` : '');
  }

  start() {
    const spec = this.spec();
    if (this.link.connected) this.link.send({ type: 'sim', action: 'reset', scenario: 'custom', spawn: spec });
    else this.link.send({ type: 'sim', action: 'reset', spawn: spec });       // offline preview honours it too
    this.onStart?.();
  }

  fromVehicle() {
    const s = this.vehicle;
    if (!s || !s.pos) return;
    const rc = this.world.runway.center_en || [0, 0];
    const e = s.pos[0] - rc[0], n = s.pos[1] - rc[1];
    this.p.dist_km = Math.round(Math.min(Math.hypot(e, n) / 1000, 95) * 2) / 2;
    this.p.bearing_deg = Math.round(wrap360(Math.atan2(e, n) / D2R));
    this.p.alt_km = Math.max(0.3, Math.round(s.pos[2] / 100) / 10);
    const air = s.air || {};
    if (isFinite(air.track)) { this.p.heading_deg = Math.round(wrap360(air.psi ?? air.track)); this.p.heading_mode = 'free'; }
    if (isFinite(air.mach)) { this.p.speed_unit = 'mach'; this.p.speed = Math.round(Math.min(3, Math.max(0.3, air.mach)) * 100) / 100; }
    if (isFinite(air.gamma)) this.p.gamma_deg = Math.round(Math.min(5, Math.max(-25, air.gamma)) * 2) / 2;
    this._changed();
  }

  setVehicle(st) { this.vehicle = st; }

  // ---------------------------------------------------------------- map
  draw() {
    const g = this.g, W = this.cv.width, H = this.cv.height;
    if (!this.root.offsetParent) return;                    // tab hidden
    g.clearRect(0, 0, W, H);
    g.fillStyle = '#0b1118'; g.fillRect(0, 0, W, H);
    const T = this.hud.terrain, img = this.hud.mapImg;
    if (img && T) {
      const [x0, y0] = this._toPx(-T.extent / 2, T.extent / 2), [x1] = this._toPx(T.extent / 2, 0);
      g.globalAlpha = 0.9; g.drawImage(img, x0, y0, x1 - x0, x1 - x0); g.globalAlpha = 1;
    }
    const [cx, cy] = this._toPx(0, 0), s = Math.min(W, H) / (2 * VIEW);
    // range rings
    g.strokeStyle = 'rgba(255,255,255,0.22)'; g.lineWidth = 1; g.setLineDash([3, 5]);
    g.font = '18px ui-monospace, monospace'; g.fillStyle = 'rgba(255,255,255,0.55)';
    for (let k = 20; k <= 80; k += 20) {
      g.beginPath(); g.arc(cx, cy, k * 1000 * s, 0, Math.PI * 2); g.stroke();
      g.fillText(`${k}`, cx + 4, cy - k * 1000 * s + 18);
    }
    g.setLineDash([]);
    // runway
    const rw = this.world.runway, h = rw.heading_deg * D2R, L = rw.length;
    const a = this._toPx(-Math.sin(h) * L / 2, -Math.cos(h) * L / 2), b = this._toPx(Math.sin(h) * L / 2, Math.cos(h) * L / 2);
    g.strokeStyle = '#fff'; g.lineWidth = 5; g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke();
    g.fillStyle = '#fff'; g.font = '600 18px ui-monospace, monospace';
    g.fillText('33', a[0] - 30, a[1] + 20); g.fillText('15', b[0] + 8, b[1] - 6);
    // current vehicle
    const v = this.vehicle;
    if (v && v.pos) {
      const [vx, vy] = this._toPx(v.pos[0], v.pos[1]);
      g.fillStyle = 'rgba(110,220,255,0.9)'; g.beginPath(); g.arc(vx, vy, 5, 0, Math.PI * 2); g.fill();
    }
    // spawn marker + heading arrow
    const [e0, n0] = this._en(), [px, py] = this._toPx(e0, n0);
    const c = this.check;
    const col = !c ? '#ffb547' : !c.reachable ? '#ff5d5d' : c.level ? '#ffb547' : '#3dff6e';
    let hd = this.p.heading_mode === 'runway' ? Math.atan2(-e0, -n0) : this.p.heading_deg * D2R;
    if (this.p.heading_mode === 'runway' && Math.hypot(e0, n0) < 1) hd = 0;
    const len = 46, ex = px + Math.sin(hd) * len, ey = py - Math.cos(hd) * len;
    g.strokeStyle = col; g.lineWidth = 3; g.beginPath(); g.moveTo(px, py); g.lineTo(ex, ey); g.stroke();
    const ah = 12, a1 = hd + Math.PI * 0.85, a2 = hd - Math.PI * 0.85;
    g.beginPath(); g.moveTo(ex, ey); g.lineTo(ex + Math.sin(a1) * ah, ey - Math.cos(a1) * ah);
    g.moveTo(ex, ey); g.lineTo(ex + Math.sin(a2) * ah, ey - Math.cos(a2) * ah); g.stroke();
    g.fillStyle = col; g.beginPath(); g.arc(px, py, 8, 0, Math.PI * 2); g.fill();
    g.strokeStyle = '#000'; g.lineWidth = 1.5; g.stroke();
    g.fillStyle = '#fff'; g.font = '600 20px ui-monospace, monospace';
    const lbl = `${this.p.alt_km.toFixed(1)} km`;
    g.fillText(lbl, px + 12, py + (ey > py ? -12 : 26));
    // chosen runway end highlight
    if (c && c.reachable) {
      const P = c.rwy === '33' ? a : b;
      g.strokeStyle = col; g.lineWidth = 2; g.beginPath(); g.arc(P[0], P[1], 12, 0, Math.PI * 2); g.stroke();
    }
  }
}

function zhWarn(w) {
  return w
    .replace(/^position clamped to the map \((\d+) km\)/, '位置超出地图，已限制在 ±$1 km')
    .replace(/^altitude raised to (\d+) m \(150 m above the terrain\)/, '高度太低，已抬到 $1 m（离地 150 m）')
    .replace(/^speed clamped to (\d+) m\/s/, '速度超限，已限制为 $1 m/s')
    .replace(/^too slow for 1 g at this altitude: alpha limited to ([\d.]+) deg \(will sink \/ accelerate\)/, '这个高度速度太低，平飞配平不了：迎角限制在 $1°（会掉高度加速）');
}
