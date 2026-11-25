// Synthesised storm sound (WebAudio, no assets): a faint rain ambience for cameras near the ground in
// rain, and thunder computed from the lightning channel itself.  The context and master bus also
// carry the airflow sound of the orbiter (aero_audio.js).
//
// Thunder (after Few 1969/1982, Ribner & Roy 1982, Bass 1980):
// * source: the hot channel expands as a cylindrical shock that decays into weak N-waves; every metre
//   of the tortuous channel is a source.  A straight piece of channel delivers its N-wave spread over
//   the difference of its end ranges (box-convolved N-wave), so pieces perpendicular to the line of
//   sight arrive at once and add up coherently (the claps), pieces along it smear out (the rumble).
//   The nearest part of the channel arrives first: a ripping crack for a close strike, then seconds
//   of rolling thunder from the farther parts (in-cloud channels included).  N-wave length ~6-15 ms,
//   growing ~ r^(1/4): most energy at 50-150 Hz
// * propagation: spherical spreading; sound speed from the US76 temperature (arrival times); air
//   absorption (ISO 9613-1 like, ~f^1.5, stronger in thin air aloft) - distant thunder keeps only the
//   low rumble; upward refraction (temperature lapse): beyond the acoustic horizon
//   sqrt(2 R hs) + sqrt(2 R hl) (R ~ 87 km) the listener is in the shadow zone, so thunder is rarely
//   heard from the ground beyond ~20-25 km; acoustic impedance (rho c) at source and listener: the
//   same energy gives a lower pressure in thin air; a ground reflection (image source)
// * listener: the camera, moving (arrival times, Doppler; a listener outrunning the sound hears
//   nothing), in the cockpit (hull transmission loss: quiet, low frequencies only); terrain / cloud
//   echoes as a convolution reverb that fades with height above the ground
// The waveform is synthesised in a worker (no frame hitch) and scheduled at its arrival time.

/** Thunder waveform from a lightning channel. Pure function (also runs in the worker / node).
 * @param {object} P { seg: Float32Array (x1,y1,z1,x2,y2,z2 per segment, three world, y up), w: Float32Array
 *   (energy per length, main channel 1), mask: Int32Array (bit k: lit by stroke k), strokes: [{t0, peak}],
 *   ic: bool, L: [x,y,z] listener, v: [vx,vy,vz] listener velocity (m/s), gL: ground height below the
 *   listener, gS: ground height at the strike, right: [x,y,z] listener right (stereo), fs, seed }
 * @returns {{ L: Float32Array, R: Float32Array, t0: number, fs: number, peak: number }} t0: arrival of
 *   the first sample after the strike (s); empty buffers if nothing reaches the listener */
export function synthThunder(P) {
  let seed = (P.seed >>> 0) || 1;
  const rnd = () => { seed = (seed + 0x6D2B79F5) >>> 0; let t = seed; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  const gauss = () => (rnd() + rnd() + rnd() - 1.5) * 2;
  const fs = P.fs || 24000;
  // US76 troposphere / lower stratosphere
  const Tz = (z) => (z < 11000 ? 288.15 - 0.0065 * Math.max(z, -500) : 216.65);
  const cz = (z) => 20.05 * Math.sqrt(Tz(z));
  const pz = (z) => (z < 11000 ? 101325 * Math.pow(Tz(z) / 288.15, 5.2559) : 22632 * Math.exp(-(z - 11000) / 6341.6));
  const rhoz = (z) => pz(z) / (287.05 * Tz(z));
  const [Lx, Ly, Lz] = P.L, [vx, vy, vz] = P.v, [rx, ry, rz] = P.right;
  const hl = Math.max(Ly - P.gL, 1.5);
  const Rc = 87000;                                    // ray curvature radius: c / |dc/dz| (6.5 K/km lapse)
  const A0 = 45.0;                                     // calibration (pressure units of the output)
  // ---- refine the channel to ~metre-scale tortuosity (the fine structure makes the crackle up close)
  const S = P.seg, nS = S.length / 6;
  const ex = [], ew = [], em = [];
  const stack = [];
  for (let i = 0; i < nS; i++) {
    const o = i * 6;
    const mx = (S[o] + S[o + 3]) / 2 - Lx, my = (S[o + 1] + S[o + 4]) / 2 - Ly, mz = (S[o + 2] + S[o + 5]) / 2 - Lz;
    const lmax = Math.min(15, Math.max(3, Math.hypot(mx, my, mz) / 1200));
    stack.push(S[o], S[o + 1], S[o + 2], S[o + 3], S[o + 4], S[o + 5]);
    while (stack.length) {
      const qz = stack.pop(), qy = stack.pop(), qx = stack.pop(), pz_ = stack.pop(), py = stack.pop(), px = stack.pop();
      const dx = qx - px, dy = qy - py, dz = qz - pz_, len = Math.hypot(dx, dy, dz);
      if (len > lmax && ex.length < 6 * 60000) {
        // midpoint displaced perpendicular to the piece
        let ox = gauss(), oy = gauss(), oz = gauss();
        const pr = (ox * dx + oy * dy + oz * dz) / (len * len);
        ox -= pr * dx; oy -= pr * dy; oz -= pr * dz;
        const k = len * 0.16;
        const cx = (px + qx) / 2 + ox * k, cy = (py + qy) / 2 + oy * k, czz = (pz_ + qz) / 2 + oz * k;
        stack.push(px, py, pz_, cx, cy, czz, cx, cy, czz, qx, qy, qz);
      } else { ex.push(px, py, pz_, qx, qy, qz); ew.push(P.w[i]); em.push(P.mask[i]); }
    }
  }
  const nE = ew.length;
  // ---- absorption per distance bin (cut-off where the loss reaches 6 dB), thin air absorbs more
  const zMean = (Ly + (P.gS + 2000)) / 2;
  const pfac = 101325 / pz(Math.max(zMean, 0));
  const edges = [300, 700, 1500, 3000, 6000, 12000, 24000];
  const reps = [150, 460, 1020, 2120, 4240, 8490, 17000, 34000];
  const NB = reps.length;
  const fc = reps.map((d) => Math.min(fs * 0.45, Math.max(45, 1000 * Math.pow(6 / (4.66 * d / 1000 * pfac), 1 / 1.5))));
  const binOf = (d) => { let b = 0; while (b < edges.length && d > edges[b]) b++; return b; };
  // ---- arrival of a sound emitted at t=0 from (sx,sy,sz) at a listener moving with constant velocity
  const arrive = (sx, sy, sz, lx, ly, lz, wy, c) => {
    const dx = lx - sx, dy = ly - sy, dz = lz - sz;
    const a = vx * vx + wy * wy + vz * vz - c * c, b = 2 * (dx * vx + dy * wy + dz * vz), cc = dx * dx + dy * dy + dz * dz;
    if (Math.abs(a) < 1e-6) return b < 0 ? -cc / b : -1;
    const disc = b * b - 4 * a * cc;
    if (disc < 0) return -1;
    const sq = Math.sqrt(disc);
    const t1 = (-b - sq) / (2 * a), t2 = (-b + sq) / (2 * a);
    const lo = Math.min(t1, t2), hi = Math.max(t1, t2);
    return lo > 0 ? lo : hi > 0 ? hi : -1;
  };
  // ---- arrivals (two passes: direct and ground-reflected image listener)
  const evA = [], evB = [], evT = [], evAmp = [], evGl = [], evGr = [], evBin = [];
  const strokes = P.strokes && P.strokes.length ? P.strokes : [{ t0: 0, peak: 1 }];
  for (let pass = 0; pass < 2; pass++) {
    const ly = pass ? 2 * P.gL - Ly : Ly, wy = pass ? -vy : vy, Rg = pass ? 0.7 : 1;
    if (pass && hl < 0.5) break;
    for (let e = 0; e < nE; e++) {
      const o = e * 6;
      const x1 = ex[o], y1 = ex[o + 1], z1 = ex[o + 2], x2 = ex[o + 3], y2 = ex[o + 4], z2 = ex[o + 5];
      const len = Math.hypot(x2 - x1, y2 - y1, z2 - z1);
      if (len < 1e-3) continue;
      const mx = (x1 + x2) / 2, my = (y1 + y2) / 2, mz = (z1 + z2) / 2;
      const ddx = Lx - mx, ddy = ly - my, ddz = Lz - mz, D = Math.hypot(ddx, ddy, ddz);
      const c = (cz(my) + cz(Ly) + 2 * cz((my + Ly) / 2)) / 4;
      const ta = arrive(x1, y1, z1, Lx, ly, Lz, wy, c), tb = arrive(x2, y2, z2, Lx, ly, Lz, wy, c);
      if (ta < 0 || tb < 0) continue;                      // the listener outruns this sound
      // acoustic shadow (upward refraction), impedance, spreading
      const hs = Math.max(my - P.gS, 2), Dh = Math.hypot(ddx, ddz);
      const exc = Dh - (Math.sqrt(2 * Rc * hs) + Math.sqrt(2 * Rc * hl));
      const sh = exc > 0 ? Math.pow(10, -Math.min(30, 12 * Math.min(1, exc / 1500) + exc / 1000 * 3) / 20) : 1;
      const imp = Math.sqrt(rhoz(Ly) * cz(Ly) / (rhoz(my) * cz(my)));
      const close = -(vx * ddx + wy * ddy + vz * ddz) / Math.max(D, 1);     // listener closing speed
      const dop = 1 / Math.max(0.2, 1 + close / c);
      const pan = Math.max(-1, Math.min(1, (-(ddx) * rx + -(ddy) * ry + -(ddz) * rz) / Math.max(D, 1)));
      const gl = Math.cos((pan + 1) * Math.PI / 4), gr = Math.sin((pan + 1) * Math.PI / 4);
      const base = A0 * len / Math.max(D, 30) * sh * imp * Rg;
      const T0 = 0.0085 * Math.sqrt(ew[e]) * Math.pow(Math.max(D, 50) / 1000, 0.25);
      const bin = binOf(D);
      for (let k = 0; k < strokes.length; k++) {
        const m = em[e];
        if (!((m >> Math.min(k, 30)) & 1)) continue;
        if (P.ic && k > 0 && ((e * 2654435761 + k * 40503) >>> 0) % 10 >= 6) continue;   // IC pulses: part of the network
        const amp = base * ew[e] * strokes[k].peak;
        if (amp < 1e-7) continue;
        const T = Math.min(0.04, Math.max(0.002, T0 * (0.8 + 0.4 * rnd()))) * dop;
        evA.push(strokes[k].t0 + Math.min(ta, tb)); evB.push(strokes[k].t0 + Math.max(ta, tb)); evT.push(T);
        evAmp.push(amp); evGl.push(gl); evGr.push(gr); evBin.push(bin);
      }
    }
  }
  const nV = evA.length;
  if (!nV) return { L: new Float32Array(0), R: new Float32Array(0), t0: 0, fs, peak: 0 };
  let tmin = Infinity, tmax = 0;
  for (let i = 0; i < nV; i++) { tmin = Math.min(tmin, evA[i]); tmax = Math.max(tmax, evB[i] + evT[i]); }
  tmin = Math.max(0, tmin - 0.02);
  const N = Math.min(Math.ceil((tmax - tmin + 0.5) * fs), Math.ceil(45 * fs));
  const outL = new Float32Array(N), outR = new Float32Array(N);
  const tL = new Float32Array(N), tR = new Float32Array(N);
  for (let b = 0; b < NB; b++) {
    let lo = N, hi = 0;
    for (let i = 0; i < nV; i++) {
      if (evBin[i] !== b) continue;
      const T = evT[i], t1 = evA[i] - tmin, t2 = evB[i] - tmin;
      const w = Math.max(t2 - t1, 1 / fs);
      const a = evAmp[i] / w, al = a * evGl[i], ar = a * evGr[i];
      // box-convolved N-wave: (len / w) * [M(t - t1) - M(t - t2)], M(t) = t - t^2 / T on [0, T]
      for (let side = 0; side < 2; side++) {
        const ts = side ? t2 : t1, sg = side ? -1 : 1;
        const n0 = Math.ceil(ts * fs), n1 = Math.min(N - 1, Math.floor((ts + T) * fs));
        for (let n = n0; n <= n1; n++) {
          const u = n / fs - ts;
          const M = (u - u * u / T) * sg;
          tL[n] += al * M; tR[n] += ar * M;
        }
        if (n0 < lo) lo = n0;
        if (n1 > hi) hi = n1;
      }
    }
    if (hi < lo) continue;
    // air absorption of this distance band: two cascaded one-pole low-passes at fc
    const k = 1 - Math.exp(-2 * Math.PI * fc[b] / fs);
    let a1 = 0, a2 = 0, b1 = 0, b2 = 0;
    const end = Math.min(N, hi + Math.ceil(fs * 0.05));
    for (let n = lo; n < end; n++) {
      a1 += k * (tL[n] - a1); a2 += k * (a1 - a2);
      b1 += k * (tR[n] - b1); b2 += k * (b1 - b2);
      outL[n] += a2; outR[n] += b2;
      tL[n] = 0; tR[n] = 0;
    }
  }
  // inaudible infrasound out (a large part of distant thunder's energy is below 20 Hz)
  const kh = Math.exp(-2 * Math.PI * 22 / fs);
  let pl = 0, pr = 0, yl = 0, yr = 0, peak = 0;
  for (let n = 0; n < N; n++) {
    const xl = outL[n], xr = outR[n];
    yl = kh * (yl + xl - pl); yr = kh * (yr + xr - pr);
    pl = xl; pr = xr;
    outL[n] = yl; outR[n] = yr;
    peak = Math.max(peak, Math.abs(yl), Math.abs(yr));
  }
  return { L: outL, R: outR, t0: tmin, fs, peak };
}

export class StormAudio {
  constructor() {
    this.ctx = null;
    this.aero = null;            // AeroAudio (aero_audio.js): the airflow sound, on the same context / master bus
    this.enabled = true;
    this.ambient = 0;
    this.lis = { pos: [0, 0, 0], vel: [0, 0, 0], groundY: 0, right: [1, 0, 0], inCockpit: false };
    this.worker = null;
    this.jobs = new Map();
    this.jobId = 0;
    const unlock = () => this._init();
    addEventListener('pointerdown', unlock);
    addEventListener('keydown', unlock);
  }

  _noise(seconds) {
    const ctx = this.ctx, n = Math.floor(ctx.sampleRate * seconds);
    const buf = ctx.createBuffer(1, n, ctx.sampleRate);
    const d = buf.getChannelData(0);
    for (let i = 0; i < n; i++) d[i] = Math.random() * 2 - 1;
    return buf;
  }

  /** diffuse echoes from terrain and cloud: decaying noise, darker as it decays, decorrelated L/R */
  _reverbIR() {
    const ctx = this.ctx, fs = ctx.sampleRate, n = Math.floor(fs * 5.0);
    const buf = ctx.createBuffer(2, n, fs);
    const pre = Math.floor(fs * 0.08);
    for (let ch = 0; ch < 2; ch++) {
      const d = buf.getChannelData(ch);
      let lp = 0, e = 0;
      for (let i = pre; i < n; i++) {
        const t = (i - pre) / fs;
        const k = 1 - Math.exp(-2 * Math.PI * (2500 * Math.exp(-t / 0.9) + 120) / fs);   // HF dies first
        lp += k * ((Math.random() * 2 - 1) - lp);
        // sparse early echoes (hills, buildings) then a dense tail; T60 ~ 4 s
        const early = t < 0.9 && Math.random() < 0.004 ? (Math.random() * 2 - 1) * 6 : 0;
        d[i] = (lp + early * Math.exp(-t / 0.3)) * Math.exp(-t / 0.58);
        e += d[i] * d[i];
      }
      const g = 1 / Math.sqrt(e / fs + 1e-9) * 0.35;
      for (let i = 0; i < n; i++) d[i] *= g;
    }
    return buf;
  }

  _init() {
    if (this.ctx) { if (this.ctx.state === 'suspended') this.ctx.resume(); return; }
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return;
    try { this.ctx = new AC(); } catch { return; }
    const ctx = this.ctx;
    this.master = ctx.createGain();
    this.master.gain.value = this.enabled ? 0.9 : 0;
    const comp = ctx.createDynamicsCompressor();
    comp.threshold.value = -10; comp.knee.value = 8; comp.ratio.value = 6; comp.attack.value = 0.002; comp.release.value = 0.4;
    // a close strike's first N-waves are faster than the compressor: soft saturation instead of
    // hard digital clipping (unity gain for normal levels)
    const pre = ctx.createGain(); pre.gain.value = 1 / 3;
    const sat = ctx.createWaveShaper();
    const curve = new Float32Array(2049);
    for (let i = 0; i < curve.length; i++) curve[i] = Math.tanh(3 * (i / 1024 - 1));
    sat.curve = curve;
    this.master.connect(comp).connect(pre).connect(sat).connect(ctx.destination);
    // faint ground-level rain ambience
    const hiss = ctx.createBufferSource(); hiss.buffer = this._noise(4); hiss.loop = true;
    const hp = ctx.createBiquadFilter(); hp.type = 'highpass'; hp.frequency.value = 900;
    const lp = ctx.createBiquadFilter(); lp.type = 'lowpass'; lp.frequency.value = 6000;
    this.ambG = ctx.createGain(); this.ambG.gain.value = 0;
    hiss.connect(hp).connect(lp).connect(this.ambG).connect(this.master);
    hiss.start();
    // thunder bus: dry + echoes, then the listener's enclosure (cockpit hull) if any
    this.tBus = ctx.createGain();
    this.dry = ctx.createGain();
    this.conv = ctx.createConvolver(); this.conv.normalize = false; this.conv.buffer = this._reverbIR();
    this.wet = ctx.createGain(); this.wet.gain.value = 0.3;
    this.cabLP = ctx.createBiquadFilter(); this.cabLP.type = 'lowpass'; this.cabLP.Q.value = 0.5; this.cabLP.frequency.value = 20000;
    this.cabG = ctx.createGain();
    this.tBus.connect(this.dry).connect(this.cabLP);
    this.tBus.connect(this.conv).connect(this.wet).connect(this.cabLP);
    this.cabLP.connect(this.cabG).connect(this.master);
    this.setRain(this.ambient);
    this._applyListener(true);
    if (this.aero) this.aero.attach(ctx, this.master);
  }

  /** the airflow sound shares the context (created on the first user gesture) and the master bus */
  attachAero(aero) {
    this.aero = aero;
    if (this.ctx) aero.attach(this.ctx, this.master);
  }

  setEnabled(v) {
    this.enabled = v;
    if (this.aero) this.aero.setEnabled(v);
    if (this.master) this.master.gain.setTargetAtTime(v ? 0.9 : 0, this.ctx.currentTime, 0.1);
  }

  /** ambient: rain around a camera near the ground (0..1) */
  setRain(ambient = 0) {
    this.ambient = ambient;
    if (!this.ctx) return;
    this.ambG.gain.setTargetAtTime(0.035 * Math.min(1, Math.max(0, ambient)), this.ctx.currentTime, 0.8);
  }

  /** the listener (camera): { pos, vel (m/s), groundY, right (unit), inCockpit } - three world */
  setListener(o) {
    const L = this.lis;
    L.pos[0] = o.pos.x; L.pos[1] = o.pos.y; L.pos[2] = o.pos.z;
    if (o.vel) { L.vel[0] = o.vel.x; L.vel[1] = o.vel.y; L.vel[2] = o.vel.z; }
    if (o.right) { L.right[0] = o.right.x; L.right[1] = o.right.y; L.right[2] = o.right.z; }
    L.groundY = o.groundY ?? 0;
    const changed = L.inCockpit !== !!o.inCockpit;
    L.inCockpit = !!o.inCockpit;
    this._applyListener(changed);
  }

  _applyListener(now) {
    if (!this.ctx) return;
    const L = this.lis, t = this.ctx.currentTime, tc = now ? 0.02 : 0.5;
    // orbiter crew cabin: heavy pressure hull + insulation, ~25 dB down and only the low end through
    this.cabLP.frequency.setTargetAtTime(L.inCockpit ? 240 : 20000, t, tc);
    this.cabG.gain.setTargetAtTime(L.inCockpit ? 0.06 : 1, t, tc);
    // echoes from the terrain around: strong near the ground, gone far above it
    const h = Math.max(0, L.pos[1] - L.groundY);
    this.wet.gain.setTargetAtTime(0.06 + 0.34 / (1 + h / 400), t, 0.5);
  }

  _worker() {
    if (this.worker !== null) return this.worker;
    try {
      const src = `const synthThunder = ${synthThunder.toString()};\n` +
        'onmessage = (e) => { const r = synthThunder(e.data); postMessage({ id: e.data.id, ...r }, [r.L.buffer, r.R.buffer]); };';
      this.worker = new Worker(URL.createObjectURL(new Blob([src], { type: 'text/javascript' })));
      this.worker.onmessage = (e) => this._play(e.data);
      this.worker.onerror = () => { this.worker = false; };
    } catch { this.worker = false; }
    return this.worker;
  }

  /** thunder from a lightning event (lightning.js: ev.acoustic, ev.strokes, ev.type, ev.groundY) */
  thunder(ev) {
    if (!this.ctx || !this.enabled || this.ctx.state !== 'running' || !ev || !ev.acoustic) return;
    const ac = ev.acoustic, L = this.lis;
    const job = {
      id: ++this.jobId, seg: new Float32Array(ac.seg), w: new Float32Array(ac.w), mask: new Int32Array(ac.mask),
      strokes: ev.strokes.map((s) => ({ t0: s.t0, peak: s.peak })), ic: ev.type !== 'cg',
      L: L.pos.slice(), v: L.vel.slice(), gL: L.groundY, gS: ev.groundY ?? 0, right: L.right.slice(),
      fs: 24000, seed: (Math.random() * 4294967296) >>> 0,
    };
    this.jobs.set(job.id, this.ctx.currentTime);
    const w = this._worker();
    if (w) w.postMessage(job, [job.seg.buffer, job.w.buffer, job.mask.buffer]);
    else this._play({ id: job.id, ...synthThunder(job) });
  }

  _play(r) {
    const tStrike = this.jobs.get(r.id);
    this.jobs.delete(r.id);
    if (tStrike === undefined || !this.ctx || !r.L.length || r.peak < 1e-4) return;
    const ctx = this.ctx;
    const buf = ctx.createBuffer(2, r.L.length, r.fs);
    buf.copyToChannel(r.L, 0); buf.copyToChannel(r.R, 1);
    const s = ctx.createBufferSource(); s.buffer = buf;
    s.connect(this.tBus);
    const when = tStrike + r.t0, now = ctx.currentTime;
    if (when >= now) s.start(when);
    else if (now - when < buf.duration) s.start(now, now - when);     // synthesis took longer than the sound
  }
}
