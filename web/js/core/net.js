// WebSocket link to the Python simulator + state buffering / interpolation.
// Falls back to a local kinematic preview when the backend is not reachable.
import { eulerToQuat, qmul } from './frames.js';

function slerp(a, b, t) {
  let [aw, ax, ay, az] = a; let [bw, bx, by, bz] = b;
  let d = aw * bw + ax * bx + ay * by + az * bz;
  if (d < 0) { bw = -bw; bx = -bx; by = -by; bz = -bz; d = -d; }
  if (d > 0.9995) {
    const r = [aw + (bw - aw) * t, ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t];
    const l = Math.hypot(...r); return r.map((v) => v / l);
  }
  const th = Math.acos(d), s = Math.sin(th);
  const k0 = Math.sin((1 - t) * th) / s, k1 = Math.sin(t * th) / s;
  return [aw * k0 + bw * k1, ax * k0 + bx * k1, ay * k0 + by * k1, az * k0 + bz * k1];
}
const lerpObj = (a, b, t) => {
  if (!a || !b) return b || a;
  const o = {};
  for (const k in b) o[k] = typeof b[k] === 'number' && typeof a[k] === 'number' ? a[k] + (b[k] - a[k]) * t : b[k];
  return o;
};

export class SimLink {
  constructor({ port = 8765, onHello, onEvent, onAero, onStatus, onPlan, onMpc, onState, onEpisode } = {}) {
    this.port = port;
    this.onHello = onHello; this.onEvent = onEvent; this.onAero = onAero; this.onStatus = onStatus;
    this.onPlan = onPlan; this.onMpc = onMpc;
    this.onState = onState;          // every raw state as it arrives (also while the tab is hidden)
    this.onEpisode = onEpisode;      // a new episode started (reset / reconnect / discontinuity)
    this.episode = 0;
    this.buf = [];
    this.connected = false;
    this.offline = false;
    this.renderT = null;
    this.lastRecv = 0;
    this.latest = null;
    this.ws = null;
    this._retry = 0;
    this._connect();
    this._offlineTimer = setTimeout(() => { if (!this.connected) this._startOffline(); }, 2500);
  }

  _connect() {
    const host = location.hostname || 'localhost';
    let ws;
    try { ws = new WebSocket(`ws://${host}:${this.port}`); } catch (e) { this._scheduleRetry(); return; }
    this.ws = ws;
    ws.onopen = () => {
      this.connected = true; this.offline = false; this._retry = 0;
      this._newEpisode();
      this.onStatus?.('online');
    };
    ws.onclose = () => {
      if (this.connected) this.onStatus?.('offline');
      this.connected = false;
      this._scheduleRetry();
    };
    ws.onerror = () => {};
    ws.onmessage = (ev) => {
      let m; try { m = JSON.parse(ev.data); } catch { return; }
      switch (m.type) {
        case 'hello': this.onHello?.(m); break;
        case 'state': this._push(m); break;
        case 'aero': this.onAero?.(m); break;
        case 'event': this.onEvent?.(m); break;
        case 'plan': this.onPlan?.(m); break;
        case 'mpc': this.onMpc?.(m); break;
      }
    };
  }
  _scheduleRetry() {
    this._retry = Math.min(this._retry + 1, 6);
    setTimeout(() => { if (!this.connected) this._connect(); }, 600 * this._retry);
    if (!this.offline && !this.connected && this._retry > 2) this._startOffline();
  }

  send(obj) { if (this.connected && this.ws.readyState === 1) this.ws.send(JSON.stringify(obj)); else if (this.offline) this._offlineCmd(obj); }

  _newEpisode() {
    this.episode++;
    this.buf = []; this.renderT = null;
    this.onEpisode?.(this.episode);
  }

  _push(s) {
    const last = this.buf[this.buf.length - 1];
    if (last) {
      // reset / new episode: the clock went backwards, the server says so, or the vehicle teleported
      const dt = s.t - last.t;
      const jump = Math.hypot(s.pos[0] - last.pos[0], s.pos[1] - last.pos[1], s.pos[2] - last.pos[2]);
      const vmax = Math.max(Math.hypot(...last.vel), Math.hypot(...s.vel));
      if (dt < -1e-6 || (s.episode !== undefined && s.episode !== last.episode) || jump > vmax * Math.max(dt, 0.05) * 3 + 300) this._newEpisode();
    }
    s._ep = this.episode;
    this.onState?.(s);
    this.buf.push(s);
    if (this.buf.length > 90) this.buf.shift();
    this.latest = s;
    this.lastRecv = performance.now();
  }

  /** advance the render clock and return an interpolated state */
  sample(dtReal) {
    if (this.offline) this._offlineStep(dtReal);
    const b = this.buf;
    if (!b.length) return null;
    const last = b[b.length - 1];
    const ts = last.paused ? 0 : (last.time_scale || 1);
    const delay = 0.06 * Math.max(ts, 0.25);
    const target = last.t - delay;
    if (this.renderT === null || Math.abs(this.renderT - target) > 1.0 * Math.max(ts, 1)) this.renderT = target;
    else {
      this.renderT += dtReal * ts;
      this.renderT += (target - this.renderT) * Math.min(1, dtReal * 3);
    }
    const t = this.renderT;
    let i = b.length - 1;
    while (i > 0 && b[i - 1].t > t) i--;
    const s1 = b[Math.max(i, 0)], s0 = b[Math.max(i - 1, 0)];
    if (s0 === s1 || s1.t <= s0.t) return { ...s1 };
    const u = Math.min(1, Math.max(0, (t - s0.t) / (s1.t - s0.t)));
    const h = s1.t - s0.t;
    // Hermite position from velocities
    const h00 = 2 * u ** 3 - 3 * u ** 2 + 1, h10 = u ** 3 - 2 * u ** 2 + u, h01 = -2 * u ** 3 + 3 * u ** 2, h11 = u ** 3 - u ** 2;
    const pos = [0, 1, 2].map((k) => h00 * s0.pos[k] + h10 * h * s0.vel[k] + h01 * s1.pos[k] + h11 * h * s1.vel[k]);
    const vel = [0, 1, 2].map((k) => s0.vel[k] + (s1.vel[k] - s0.vel[k]) * u);
    return {
      ...s1, t, pos, vel,
      quat: slerp(s0.quat, s1.quat, u),
      air: lerpObj(s0.air, s1.air, u),
      surf: lerpObj(s0.surf, s1.surf, u),
      omega: s1.omega,
    };
  }

  // ------------------------------------------------------------------ offline preview
  _startOffline() {
    if (this.offline || this.connected) return;
    this.offline = true;
    this.onStatus?.('local');
    this._offlineReset();
  }
  _offlineReset() {
    const r = Math.random;
    const dist = 18000 + r() * 40000, brg = r() * Math.PI * 2;
    const alt = 7000 + r() * 15000;
    const hdg = r() * 360;
    const rho = 1.225 * Math.exp(-alt / 8500);
    const V = Math.min(Math.sqrt(2 * 10000 / rho), 700);
    this.off = { t: 0, e: Math.sin(brg) * dist, n: Math.cos(brg) * dist, u: alt, V, hdg, gam: -8, bank: 0, paused: false, ts: 1 };
    this._newEpisode();
    this.onEvent?.({ level: 'warn', text: `后端未连接：本地预览模式（随机出生点 ${(dist / 1000).toFixed(1)} km / ${(alt / 1000).toFixed(1)} km）` });
  }
  _offlineCmd(obj) {
    if (obj.type === 'sim') {
      if (obj.action === 'reset') this._offlineReset();
      if (obj.action === 'toggle_pause') this.off.paused = !this.off.paused;
      if (obj.action === 'pause') this.off.paused = true;
      if (obj.action === 'resume') this.off.paused = false;
      if (obj.action === 'set_time_scale') this.off.ts = obj.value;
    }
  }
  _offlineStep(dt) {
    const o = this.off;
    if (!o) return;
    const step = o.paused ? 0 : dt * o.ts;
    o.t += step;
    // gentle S-turns, constant flight-path angle
    o.bank = 25 * Math.sin(o.t * 0.05);
    const g = 9.81;
    o.hdg += (g * Math.tan(o.bank * Math.PI / 180) / o.V) * step * 180 / Math.PI;
    const gr = o.gam * Math.PI / 180, hr = o.hdg * Math.PI / 180;
    const ve = o.V * Math.cos(gr) * Math.sin(hr), vn = o.V * Math.cos(gr) * Math.cos(hr), vu = o.V * Math.sin(gr);
    o.e += ve * step; o.n += vn * step; o.u += vu * step;
    if (o.u < 1500) this._offlineReset();
    const alpha = 9;
    const q = eulerToQuat(o.hdg, o.gam + alpha, o.bank);
    const rho = 1.225 * Math.exp(-o.u / 8500);
    const a = 340 - 0.004 * Math.min(o.u, 11000);
    this._push({
      type: 'state', t: o.t, paused: o.paused, time_scale: o.ts, mode: 'PREVIEW', status: 'FLIGHT', guidance: 'local-preview',
      pos: [o.e, o.n, o.u], vel: [ve, vn, vu], quat: q, omega: [0, 0, 0],
      air: { alpha, beta: 0, mach: o.V / a, tas: o.V, eas: o.V * Math.sqrt(rho / 1.225), keas: o.V * Math.sqrt(rho / 1.225) * 1.9438,
        qbar: 0.5 * rho * o.V * o.V, rho, nz: 1 / Math.cos(o.bank * Math.PI / 180), gamma: o.gam, track: ((o.hdg % 360) + 360) % 360,
        bank: o.bank, phi: o.bank, theta: o.gam + alpha, psi: ((o.hdg % 360) + 360) % 360, alt: o.u, agl: o.u, vs: vu, ld: 4.5, wind: [0, 0, 0] },
      surf: { elevon_li: 2 + o.bank * 0.05, elevon_lo: 2 + o.bank * 0.05, elevon_ri: 2 - o.bank * 0.05, elevon_ro: 2 - o.bank * 0.05, bodyflap: 5, rudder: 0, speedbrake: 25 },
      cmd: { alpha, bank: o.bank, speedbrake: 25 }, gear: { pos: 0, wow: [false, false, false], compress: [0, 0, 0] }, chute: 0, brakes: 0,
    });
  }
}
export { qmul };
