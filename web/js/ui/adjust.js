// In-flight adjustment: change airspeed, the direction of flight through the air (heading and
// flight-path angle) and altitude while flying, plus one-click steps (+-2 km, +-30 deg).  The sliders
// follow the live state until one is moved; "apply" sends the edited values (`sim` / `adjust`), the
// backend turns the attitude with the velocity (angle of attack, sideslip, bank kept) and re-plans.
const D2R = Math.PI / 180;
const UNIT = { mach: { lab: '马赫', min: 0.3, max: 3, step: 0.01 }, keas: { lab: 'KEAS', min: 120, max: 500, step: 1 },
  tas: { lab: 'm/s', min: 75, max: 900, step: 1 } };
const KEY = 'planelands.adjust.unit';
const wrap360 = (a) => ((a % 360) + 360) % 360;

export class AdjustPanel {
  /** root: container element; link: SimLink; onJump(): episode cleanup after an altitude change */
  constructor({ root, link, onJump }) {
    this.root = root; this.link = link; this.onJump = onJump;
    let unit = 'mach';
    try { unit = localStorage.getItem(KEY) || unit; } catch { /* ignore */ }
    this.unit = UNIT[unit] ? unit : 'mach';
    this.p = { speed: 0.9, heading_deg: 0, gamma_deg: -8, alt_km: 15 };
    this.dirty = new Set();              // sliders the user moved (they stop following the vehicle)
    this.live = null;                    // current values from the state message
    this.flying = false;
    this._build();
    this._sync();
  }

  _build() {
    const r = this.root;
    r.innerHTML = `
      <h3 class="subh">飞行中调整</h3>
      <div class="row btns">
        <button data-q="up" title="保持当量空速，立即升高 2 km">▲ 升高 2 km</button>
        <button data-q="down" title="保持当量空速，立即降低 2 km（不低于离地 150 m）">▼ 降低 2 km</button>
      </div>
      <div class="row btns">
        <button data-q="left" title="空速方向左转 30°，姿态随之转动">↺ 左转 30°</button>
        <button data-q="right" title="空速方向右转 30°，姿态随之转动">↻ 右转 30°</button>
      </div>
      <div class="row seg"><label>速度单位</label>${Object.entries(UNIT).map(([k, u]) => `<button data-su="${k}">${u.lab}</button>`).join('')}</div>
      <div class="row"><label>速度</label><input type="range" data-k="speed"><output data-o="speed"></output></div>
      <div class="row" title="空速方向的真航向"><label>航向</label><input type="range" data-k="heading_deg" min="0" max="359" step="1"><output data-o="heading_deg"></output></div>
      <div class="row" title="空速方向与水平面的夹角（负 = 下降）"><label>航迹角</label><input type="range" data-k="gamma_deg" min="-30" max="15" step="0.5"><output data-o="gamma_deg"></output></div>
      <div class="row"><label>高度（海拔）</label><input type="range" data-k="alt_km" min="0.3" max="40" step="0.1"><output data-o="alt_km"></output></div>
      <div class="row btns">
        <button class="accent" data-act="apply" title="把改动过的量（高亮）设给飞机">✔ 应用改动</button>
        <button data-act="follow" title="放弃改动，滑块重新跟随当前飞行状态">取当前状态</button>
      </div>
      <div class="note adjust-note">滑块跟随当前状态；拖动后高亮并停止跟随，点「应用改动」生效。迎角、侧滑和坡度保持不变，制导随即重新规划；只改高度时保持当量空速。仅空中可用。</div>`;
    r.querySelectorAll('input[data-k]').forEach((el) => el.addEventListener('input', () => {
      this.p[el.dataset.k] = +el.value;
      this.dirty.add(el.dataset.k);
      this._sync();
    }));
    r.querySelectorAll('[data-su]').forEach((b) => b.addEventListener('click', () => {
      if (b.dataset.su === this.unit) return;
      this.unit = b.dataset.su;
      try { localStorage.setItem(KEY, this.unit); } catch { /* ignore */ }
      this.dirty.delete('speed');        // show the current speed in the new unit
      this._follow();
      this._sync();
    }));
    const Q = { up: { d_alt: 2000 }, down: { d_alt: -2000 }, left: { d_heading_deg: -30 }, right: { d_heading_deg: 30 } };
    r.querySelectorAll('[data-q]').forEach((b) => b.addEventListener('click', () => this.send(Q[b.dataset.q])));
    r.querySelector('[data-act="apply"]').addEventListener('click', () => this.apply());
    r.querySelector('[data-act="follow"]').addEventListener('click', () => { this.dirty.clear(); this._follow(); this._sync(); });
  }

  send(spec) {
    if (!this.flying) return;
    this.link.send({ type: 'sim', action: 'adjust', spec });
    if (spec.d_alt !== undefined || spec.alt !== undefined) this.onJump?.();
  }

  apply() {
    const d = this.dirty, p = this.p, spec = {};
    if (!d.size) return;
    if (d.has('speed')) { spec.speed = p.speed; spec.speed_unit = this.unit; }
    if (d.has('heading_deg')) spec.heading_deg = p.heading_deg;
    if (d.has('gamma_deg')) spec.gamma_deg = p.gamma_deg;
    if (d.has('alt_km')) spec.alt = p.alt_km * 1000;
    this.send(spec);
    d.clear();
    this._sync();
  }

  /** the latest state message (called a few times per second) */
  setVehicle(st) {
    const a = st && st.air;
    if (!a) return;
    const w = a.wind || [0, 0, 0];
    const ve = st.vel[0] - w[0], vn = st.vel[1] - w[1];
    this.live = {
      speed: { mach: a.mach, keas: a.keas, tas: a.tas },
      heading_deg: wrap360(Math.atan2(ve, vn) / D2R), gamma_deg: a.gamma_air, alt_km: st.pos[2] / 1000,
    };
    const flying = st.status === 'FLIGHT' && !(st.gear?.wow || []).some(Boolean) && !(st.chute > 0);
    if (flying !== this.flying) {
      this.flying = flying;
      this.root.querySelectorAll('button[data-q], button[data-act="apply"]').forEach((b) => { b.disabled = !flying; });
    }
    this._follow();
    this._sync();
  }

  _follow() {
    const L = this.live;
    if (!L) return;
    for (const k of ['speed', 'heading_deg', 'gamma_deg', 'alt_km']) {
      if (this.dirty.has(k)) continue;
      this.p[k] = k === 'speed' ? L.speed[this.unit] : L[k];
    }
  }

  _sync() {
    const p = this.p, r = this.root, U = UNIT[this.unit];
    const sp = r.querySelector('input[data-k="speed"]');
    if (sp.min !== String(U.min)) { sp.min = U.min; sp.max = U.max; sp.step = U.step; }
    r.querySelectorAll('input[data-k]').forEach((el) => {
      if (document.activeElement !== el && Math.abs(+el.value - p[el.dataset.k]) > 1e-9) el.value = p[el.dataset.k];
      el.closest('.row').classList.toggle('edited', this.dirty.has(el.dataset.k));
    });
    r.querySelectorAll('[data-su]').forEach((b) => b.classList.toggle('on', b.dataset.su === this.unit));
    const fmt = {
      speed: this.unit === 'mach' ? `M ${p.speed.toFixed(2)}` : this.unit === 'keas' ? `${Math.round(p.speed)} kt` : `${Math.round(p.speed)} m/s`,
      heading_deg: `${Math.round(wrap360(p.heading_deg))}°`, gamma_deg: `${p.gamma_deg.toFixed(1)}°`, alt_km: `${p.alt_km.toFixed(1)} km`,
    };
    r.querySelectorAll('output[data-o]').forEach((o) => { if (o.textContent !== fmt[o.dataset.o]) o.textContent = fmt[o.dataset.o]; });
    const ap = r.querySelector('[data-act="apply"]');
    ap.textContent = this.dirty.size ? `✔ 应用改动（${this.dirty.size}）` : '✔ 应用改动';
  }
}
