// Pilot head motion in the flight deck: the view moves the way a strapped-in pilot's head does.
//
// Physics, not canned shake:
//   * specific force at the commander's eye = load factor at the CG (aero + gear forces from the
//     simulator, so gusts, turbulence, manoeuvres, touchdown, braking and the drag chute are all in
//     it) + the rotational terms  dw/dt x r + w x (w x r)  for the eye station 14 m ahead of the CG
//     (pitch and yaw accelerations throw the cockpit around much more than the CG);
//   * the head (on torso and neck, held by the harness) is a damped spring-mass per axis relative to
//     the seat: fore-aft, vertical (stiff, the spine) and lateral (soft, the neck); a sustained load
//     factor gives a steady offset (2 g sinks the head ~1.6 cm), changes make it oscillate;
//   * the head lags the cabin under angular acceleration and nods / tilts with its displacement;
//   * high-frequency layers that the 60 Hz flight model cannot carry: aerodynamic buffet (dynamic
//     pressure, transonic, speedbrake), turbulence chop and storm cells, runway rumble with braking,
//     and impulses at main and nose gear touchdown.
// Model frame: x forward, y up, z right.  Output: head offset (m, model frame) and a small head
// rotation (camera-local pitch / yaw / roll, rad).
import * as THREE from 'three';

const G = 9.80665;
const ss = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

// translational head axes: static gain (m per g), natural frequency (Hz), damping ratio
// (soft and well damped: the view should settle, not ring -- fast oscillation of the whole view is
// what makes a cockpit camera nauseating)
const AX = [
  { k: 0.020, f: 1.4, z: 0.62 },     // x fore-aft
  { k: 0.016, f: 1.6, z: 0.60 },     // y vertical
  { k: 0.030, f: 1.1, z: 0.62 },     // z lateral
];
const ROT = { f: 1.2, z: 0.75, max: 0.022 };
const OUT_TAU = 0.07;                // final low-pass of the camera offset / rotation (s)

// band-limited vibration: a few sines per axis with slowly wandering amplitude
class Vib {
  constructor(freqs, seed) {
    this.osc = freqs.map((f, i) => ({ f: f * (0.9 + 0.2 * Math.abs(Math.sin(seed * 12.9898 + i * 78.233))), ph: (seed * 3.7 + i * 1.91) % (Math.PI * 2) }));
    this.seed = seed;
  }
  at(t) {
    let s = 0;
    for (const o of this.osc) s += Math.sin(t * 2 * Math.PI * o.f + o.ph) * (0.6 + 0.4 * Math.sin(t * 0.83 * o.f / 7 + o.ph * 2.1));
    return s / Math.sqrt(this.osc.length);
  }
}

export class HeadMotion {
  constructor() {
    this.offset = new THREE.Vector3();
    this.rot = new THREE.Euler(0, 0, 0, 'YXZ');
    this.vib = {
      buffet: [new Vib([7.1, 9.8, 13.2], 1), new Vib([8.3, 11.6, 15.1], 2), new Vib([6.4, 10.4, 12.7], 3)],
      chop: [new Vib([0.55, 0.9, 1.5], 4), new Vib([0.6, 1.05, 1.6], 5), new Vib([0.5, 0.85, 1.35], 6)],
      rumble: [new Vib([9.5, 13.8, 17.9], 7), new Vib([8.1, 12.3, 16.4], 8), new Vib([10.2, 14.6, 19.3], 9)],
    };
    this.reset();
  }

  reset() {
    this.d = [0, 0, 0]; this.v = [0, 0, 0];
    this.a = [0, 0, 0]; this.av = [0, 0, 0];          // head pitch / yaw / roll (camera frame) and rates
    this.wPrev = null; this.tPrev = null;
    this.wdot = new THREE.Vector3();
    this.wowPrev = null; this.t = 0;
    this.offset.set(0, 0, 0); this.rot.set(0, 0, 0);
  }

  /**
   * st: interpolated simulator state; eye: eye station in the model frame (origin = CG);
   * gain: 0 off, 1 realistic, 2 exaggerated.  Returns { offset, rot }.
   */
  update(dtReal, st, eye, gain = 1) {
    if (!st || !st.air || gain <= 0) { this.reset(); return this; }
    // simulation time step (follows pause and time scale)
    let dt = this.tPrev === null ? 0 : st.t - this.tPrev;
    if (dt < 0 || dt > 0.5) { this.reset(); dt = 0; }
    this.tPrev = st.t;
    if (dt <= 0) return this;
    dt = Math.min(dt, 0.1);
    this.t += dt;
    const a = st.air;

    // body rates, model frame (FRD p, q, r -> x fwd, y up, z right) and their derivative
    const D2R = Math.PI / 180;
    const w = new THREE.Vector3((a.p || 0) * D2R, -(a.r || 0) * D2R, (a.q || 0) * D2R);
    if (this.wPrev) {
      const raw = w.clone().sub(this.wPrev).divideScalar(dt);
      raw.clampLength(0, 4);                                  // rad/s^2
      this.wdot.lerp(raw, Math.min(1, dt / 0.15));
    }
    this.wPrev = w.clone();

    // specific force at the eye (g): CG load factor + rotational terms
    const f = new THREE.Vector3(a.nx || 0, isFinite(a.nz) ? a.nz : 1, a.ny || 0);
    const r = eye;
    const rot = new THREE.Vector3().crossVectors(this.wdot, r).add(new THREE.Vector3().crossVectors(w, new THREE.Vector3().crossVectors(w, r)));
    f.addScaledVector(rot, 1 / G);
    const df = [clamp(f.x, -2, 2), clamp(f.y - 1, -2, 3), clamp(f.z, -2, 2)];

    // touchdown impulses (rising edge of weight on wheels)
    const wow = st.gear?.wow || [false, false, false];
    if (this.wowPrev) {
      const sink = Math.abs(Math.min(0, a.vs || 0));
      if ((wow[1] && !this.wowPrev[1]) || (wow[2] && !this.wowPrev[2])) this.v[1] -= clamp(0.06 + 0.10 * sink, 0, 0.35);
      if (wow[0] && !this.wowPrev[0]) { this.v[1] -= 0.08; this.av[0] -= 0.12; }
    }
    this.wowPrev = wow.slice();

    // integrate the head springs (sub-stepped for stability)
    const n = Math.max(1, Math.ceil(dt / 0.004)), h = dt / n;
    for (let s = 0; s < n; s++) {
      for (let i = 0; i < 3; i++) {
        const P = AX[i], wn = 2 * Math.PI * P.f;
        const target = -P.k * df[i] * gain;
        const acc = wn * wn * (target - this.d[i]) - 2 * P.z * wn * this.v[i];
        this.v[i] += acc * h; this.d[i] += this.v[i] * h;
      }
      // head rotation lags the cabin and follows the displacement (nod forward, tilt sideways)
      const tgt = [
        clamp((-0.007 * this.wdot.z - 0.5 * this.d[0]) * gain, -ROT.max, ROT.max),          // pitch
        clamp((0.004 * -this.wdot.y) * gain, -ROT.max, ROT.max),                              // yaw
        clamp((0.006 * this.wdot.x - 0.5 * this.d[2]) * gain, -ROT.max, ROT.max),            // roll
      ];
      const wr = 2 * Math.PI * ROT.f;
      for (let i = 0; i < 3; i++) {
        const acc = wr * wr * (tgt[i] - this.a[i]) - 2 * ROT.z * wr * this.av[i];
        this.av[i] += acc * h; this.a[i] += this.av[i] * h;
      }
    }

    // vibration layers (m)
    const qbar = a.qbar || 0, mach = a.mach || 0, env = st.env || {};
    const sb = (st.surf?.speedbrake ?? 0) / 98.6;
    const onGnd = wow.some(Boolean);
    const gs = Math.hypot(st.vel[0], st.vel[1]);
    const buffet = 0.00012 * ss(6000, 16000, qbar)
      + 0.00030 * Math.exp(-(((mach - 1.0) / 0.12) ** 2)) * ss(2000, 10000, qbar)           // transonic
      + 0.00016 * sb * ss(3000, 12000, qbar)
      + 0.00014 * ss(14, 22, a.alpha || 0) * ss(2000, 8000, qbar);                            // high alpha
    const chop = (0.0030 * clamp(env.turb ?? 0, 0, 1.5) + 0.0045 * clamp(env.storm ?? 0, 0, 1)) * ss(800, 7000, qbar) * (onGnd ? 0.2 : 1);
    const rumble = onGnd ? 0.0008 * clamp(gs / 90, 0, 1.3) * (1 + 0.4 * (st.brakes || 0)) : 0;
    const t = this.t, V = this.vib;
    const vx = buffet * 0.5 * V.buffet[0].at(t) + chop * 0.4 * V.chop[0].at(t) + rumble * 0.4 * V.rumble[0].at(t);
    const vy = buffet * V.buffet[1].at(t) + chop * V.chop[1].at(t) + rumble * V.rumble[1].at(t);
    const vz = buffet * 0.6 * V.buffet[2].at(t) + chop * 0.7 * V.chop[2].at(t) + rumble * 0.5 * V.rumble[2].at(t);

    // turbulence sways the head slowly; everything passes a final low-pass so the camera never jerks
    const ox = this.d[0] + vx * gain, oy = this.d[1] + vy * gain, oz = this.d[2] + vz * gain;
    const rx = this.a[0] + chop * 1.6 * gain * V.chop[0].at(t * 0.9), ry = this.a[1] + chop * 0.6 * gain * V.chop[1].at(t * 0.7), rz = this.a[2] + chop * 2.0 * gain * V.chop[2].at(t * 1.1);
    const k = 1 - Math.exp(-dt / OUT_TAU);
    this.offset.set(this.offset.x + (ox - this.offset.x) * k, this.offset.y + (oy - this.offset.y) * k, this.offset.z + (oz - this.offset.z) * k);
    this.rot.set(this.rot.x + (rx - this.rot.x) * k, this.rot.y + (ry - this.rot.y) * k, this.rot.z + (rz - this.rot.z) * k, 'YXZ');
    return this;
  }
}
