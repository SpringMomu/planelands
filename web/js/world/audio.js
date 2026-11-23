// Synthesised storm sound (WebAudio, no assets): a faint rain ambience only for cameras near the
// ground in rain, and thunder per strike - a sharp crack for close strikes and a long rolling
// rumble, delayed by the distance at the speed of sound and low-passed with distance.
export class StormAudio {
  constructor() {
    this.ctx = null;
    this.enabled = true;
    this.ambient = 0;
    const unlock = () => this._init();
    addEventListener('pointerdown', unlock);
    addEventListener('keydown', unlock);
  }

  _noise(seconds, brown) {
    const ctx = this.ctx, n = Math.floor(ctx.sampleRate * seconds);
    const buf = ctx.createBuffer(1, n, ctx.sampleRate);
    const d = buf.getChannelData(0);
    let last = 0;
    for (let i = 0; i < n; i++) {
      const w = Math.random() * 2 - 1;
      if (brown) { last = (last + 0.02 * w) / 1.02; d[i] = last * 3.5; } else d[i] = w;
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
    comp.threshold.value = -14; comp.ratio.value = 4;
    this.master.connect(comp).connect(ctx.destination);
    this.white = this._noise(4, false);
    this.brown = this._noise(6, true);
    // faint ground-level rain ambience
    const hiss = ctx.createBufferSource(); hiss.buffer = this.white; hiss.loop = true;
    const hp = ctx.createBiquadFilter(); hp.type = 'highpass'; hp.frequency.value = 900;
    const lp = ctx.createBiquadFilter(); lp.type = 'lowpass'; lp.frequency.value = 6000;
    this.ambG = ctx.createGain(); this.ambG.gain.value = 0;
    hiss.connect(hp).connect(lp).connect(this.ambG).connect(this.master);
    hiss.start();
    this.setRain(this.ambient);
  }

  setEnabled(v) {
    this.enabled = v;
    if (this.master) this.master.gain.setTargetAtTime(v ? 0.9 : 0, this.ctx.currentTime, 0.1);
  }

  /** ambient: rain around a camera near the ground (0..1) */
  setRain(ambient = 0) {
    this.ambient = ambient;
    if (!this.ctx) return;
    this.ambG.gain.setTargetAtTime(0.035 * Math.min(1, Math.max(0, ambient)), this.ctx.currentTime, 0.8);
  }

  /** thunder for a strike `dist` metres away; strength 0..1 */
  thunder(dist, strength = 1, cg = true) {
    if (!this.ctx || !this.enabled || this.ctx.state !== 'running') return;
    const ctx = this.ctx;
    const t0 = ctx.currentTime + dist / 343;
    const vol = Math.min(0.9, strength * 1.5 / (1 + dist / 1600));
    if (vol < 0.01) return;
    const far = Math.min(1, dist / 15000);
    // crack: only audible for close cloud-to-ground strikes
    if (cg && dist < 3000) {
      const s = ctx.createBufferSource(); s.buffer = this.white;
      const bp = ctx.createBiquadFilter(); bp.type = 'bandpass'; bp.frequency.value = 2200; bp.Q.value = 0.6;
      const g = ctx.createGain();
      g.gain.setValueAtTime(0, t0);
      g.gain.linearRampToValueAtTime(vol * 0.9, t0 + 0.006);
      g.gain.exponentialRampToValueAtTime(0.001, t0 + 0.35);
      s.connect(bp).connect(g).connect(this.master);
      s.start(t0, Math.random() * 3); s.stop(t0 + 0.4);
    }
    // rumble: brown noise through a sweeping low-pass, several rolling swells
    const dur = 4 + far * 6 + Math.random() * 3;
    const s = ctx.createBufferSource(); s.buffer = this.brown; s.loop = true;
    const lp = ctx.createBiquadFilter(); lp.type = 'lowpass';
    lp.frequency.setValueAtTime(cg ? 1400 - far * 1000 : 600 - far * 300, t0);
    lp.frequency.exponentialRampToValueAtTime(90, t0 + dur);
    const g = ctx.createGain();
    g.gain.setValueAtTime(0.0001, t0);
    let t = t0;
    const swells = 3 + Math.floor(Math.random() * 4);
    for (let i = 0; i < swells; i++) {
      const tn = t + (i === 0 ? 0.05 : 0.3 + Math.random() * (dur / swells));
      if (tn + 0.8 > t0 + dur) break;
      t = tn;
      const peak = vol * (i === 0 ? 1 : 0.35 + Math.random() * 0.6) * (1 - i / (swells + 2));
      g.gain.exponentialRampToValueAtTime(Math.max(peak, 0.0002), t);
      g.gain.exponentialRampToValueAtTime(Math.max(peak * 0.35, 0.0002), t + 0.25 + Math.random() * 0.5);
    }
    g.gain.exponentialRampToValueAtTime(0.0001, t0 + dur);
    s.connect(lp).connect(g).connect(this.master);
    s.start(t0, Math.random() * 4); s.stop(t0 + dur + 0.1);
  }
}
