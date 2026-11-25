// Synthesiser of the airflow sound (runs in the AudioWorklet, aero_worklet.js, or on the main thread
// through a ScriptProcessorNode where AudioWorklet is unavailable): stationary Gaussian pressure noise
// with the aeroacoustic model's 1/3-octave band levels (web/js/world/aeroacoustics.js), heard with two
// ears.
//
// * 1/3-octave filter bank: every band is its own white Gaussian noise through a 6th-order Butterworth
//   band-pass (bilinear, band edges of IEC 61260); its input gain is set from the filter's exact noise
//   bandwidth (integral of |H|^2); independent noise per band, so the overlapping skirts add in power
//   (no crossover interference).  The skirts of a loud band spill into its quiet neighbours (a cabin
//   resonance 18 dB above the next band): the filters' powers are solved for so that the power falling
//   in every 1/3-octave band is the model's (leakage matrix, non-negative Richardson-Lucy iterations)
// * gains follow the model continuously: a 15 ms smoothing per 128 samples, ramped sample by sample;
//   the only other delay is the filters' own group delay (a few ms at 1 kHz) - the sound follows the
//   camera and the flight without frames or steps
// * two ears (spherical head, radius 8.75 cm): the directional part reaches the far ear later by the
//   Woodworth time difference (a / c)(phi + sin phi) (up to 0.66 ms) and through the Brown & Duda (1998)
//   head-shadow filter |H|^2 = (1 + (alpha w / 2 w0)^2) / (1 + (w / 2 w0)^2), w0 = c / a,
//   alpha(theta) = 1.05 + 0.95 cos(theta / 150 deg * 180 deg): about no level difference at low
//   frequencies, up to ~17 dB at high ones; a frontal source is the reference (0 dB).  The diffuse part
//   (cabin reverberation) is independent noise at each ear
// * real-time: the same small amount of work in every render call, no allocation, Gaussian numbers
//   from a table drawn once (Box-Muller) read at hashed positions
//
// messages: { bands: Float64Array(31) centre frequencies }  once;
//           { dir, lat, dif: Float32Array(31) (band Pa^2 of the directional sound, its lateral direction
//             sin(azimuth) -1 left .. +1 right, band Pa^2 of the diffuse sound), gain (digital / Pa) }

const ORDER = 3;                    // Butterworth prototype order (band-pass order 2 * ORDER)
const GT = 1 << 16;                 // Gaussian table (indexed by 16-bit hashes)
const HEAD_A = 0.0875, C_AIR = 343;
const TAU = 0.015;                  // gain smoothing (s)
const MAXD = 64;                    // delay line per band (samples, > 0.66 ms at 96 kHz)

// complex helpers
const cmul = (a, b) => [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
const cdiv = (a, b) => { const d = b[0] * b[0] + b[1] * b[1]; return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d]; };
const csqrt = (z) => { const r = Math.hypot(z[0], z[1]); const re = Math.sqrt(Math.max(0, (r + z[0]) / 2)); const im = Math.sqrt(Math.max(0, (r - z[0]) / 2)); return [re, z[1] < 0 ? -im : im]; };

/** biquad sections [b0, b2, a1, a2] (b1 = 0, numerator b0 (1 - z^-2)) of a digital Butterworth band-pass */
export function butterBandpass(f1, f2, fs, n = ORDER) {
  const w1 = 2 * fs * Math.tan(Math.PI * f1 / fs), w2 = 2 * fs * Math.tan(Math.PI * f2 / fs);
  const w0 = Math.sqrt(w1 * w2), B = w2 - w1, K = 2 * fs;
  const poles = [];                                     // s-plane band-pass poles with Im > 0
  for (let k = 1; k <= n; k++) {
    const th = Math.PI * (2 * k + n - 1) / (2 * n), p = [Math.cos(th), Math.sin(th)];
    if (p[1] < -1e-12) continue;                        // conjugates come with their partner
    const pB = [p[0] * B, p[1] * B];
    const D = csqrt([pB[0] * pB[0] - pB[1] * pB[1] - 4 * w0 * w0, 2 * pB[0] * pB[1]]);
    for (const s of [[(pB[0] + D[0]) / 2, (pB[1] + D[1]) / 2], [(pB[0] - D[0]) / 2, (pB[1] - D[1]) / 2]]) {
      if (Math.abs(p[1]) < 1e-12 && s[1] < 0) continue; // real prototype pole: one conjugate pair
      poles.push(s[1] >= 0 ? s : [s[0], -s[1]]);
    }
  }
  const secs = poles.map((s) => {
    const z = cdiv([K + s[0], s[1]], [K - s[0], -s[1]]);  // bilinear
    return [1, -1, -2 * z[0], z[0] * z[0] + z[1] * z[1]];
  });
  // unit gain at the centre frequency
  const wc = 2 * Math.atan(w0 / K), e1 = [Math.cos(wc), -Math.sin(wc)], e2 = [Math.cos(2 * wc), -Math.sin(2 * wc)];
  let H = [1, 0];
  for (const [b0, b2, a1, a2] of secs) H = cmul(H, cdiv([b0 + b2 * e2[0], b2 * e2[1]], [1 + a1 * e1[0] + a2 * e2[0], a1 * e1[1] + a2 * e2[1]]));
  const g = Math.pow(1 / Math.hypot(H[0], H[1]), 1 / secs.length);
  return secs.map(([b0, b2, a1, a2]) => [b0 * g, b2 * g, a1, a2]);
}

/** |H(f)|^2 of biquad sections */
export function sectionsPower(secs, f, fs) {
  const w = 2 * Math.PI * f / fs, e1 = [Math.cos(w), -Math.sin(w)], e2 = [Math.cos(2 * w), -Math.sin(2 * w)];
  let p = 1;
  for (const [b0, b2, a1, a2] of secs) {
    const nr = b0 + b2 * e2[0], ni = b2 * e2[1], dr = 1 + a1 * e1[0] + a2 * e2[0], di = a1 * e1[1] + a2 * e2[1];
    p *= (nr * nr + ni * ni) / (dr * dr + di * di);
  }
  return p;
}

/** Brown & Duda spherical-head shadow |H|^2 at frequency f for the angle theta (rad) between the ear
 *  axis and the source direction */
export function headShadow2(f, theta) {
  const al = 1.05 + 0.95 * Math.cos(theta * 180 / 150), x = Math.PI * f * HEAD_A / C_AIR;   // w / (2 w0)
  return (1 + al * al * x * x) / (1 + x * x);
}

export class SpectralNoise {
  constructor(fs) {
    this.fs = fs;
    this.seed = (Math.random() * 4294967296) >>> 0;
    this.gt = new Float32Array(GT);
    for (let i = 0; i < GT; i += 2) {
      const r = Math.sqrt(-2 * Math.log(this._rnd())), a = 2 * Math.PI * this._rnd();
      this.gt[i] = r * Math.cos(a); this.gt[i + 1] = r * Math.sin(a);
    }
    this.nb = 0;
    this.w = 0;
  }

  _rnd() {            // uniform (0, 1]  (mulberry32)
    let t = (this.seed = (this.seed + 0x6D2B79F5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return (((t ^ (t >>> 14)) >>> 0) + 1) / 4294967297;
  }

  _setup(fc) {
    const nb = fc.length, fs = this.fs, S = ORDER;
    this.nb = nb; this.fc = Float64Array.from(fc);
    this.on = new Uint8Array(nb);
    this.co = new Float64Array(nb * S * 4);              // b0, b2, a1, a2 per section
    this.st = new Float64Array(nb * 2 * S * 2);          // per band, ear, section: 2 states
    this.inv = new Float64Array(nb);                     // 1 / rms of the filter output for unit white input
    this.ring = new Float32Array(nb * MAXD);
    for (let i = 0; i < nb; i++) {
      const f1 = fc[i] * Math.pow(10, -0.05), f2 = fc[i] * Math.pow(10, 0.05);
      if (f2 >= 0.45 * fs) continue;
      const secs = butterBandpass(f1, f2, fs);
      secs.forEach((s, k) => this.co.set(s, (i * S + k) * 4));
      // output variance for unit-variance white input: integral of |H|^2 (2 / fs) df
      let v = 0;
      const n = 3000, l0 = Math.log(f1 / 16), l1 = Math.log(Math.min(fs / 2 * 0.9999, f2 * 16));
      let fp = Math.exp(l0), pp = sectionsPower(secs, fp, fs);
      for (let k = 1; k <= n; k++) {
        const f = Math.exp(l0 + (l1 - l0) * k / n), p = sectionsPower(secs, f, fs);
        v += (p + pp) / 2 * (f - fp) * 2 / fs; fp = f; pp = p;
      }
      this.inv[i] = 1 / Math.sqrt(v);
      this.on[i] = 1;
      this.secs = this.secs || []; this.secs[i] = secs;
    }
    // leakage: share of filter j's output power falling in band i
    this.M = new Float64Array(nb * nb);
    for (let j = 0; j < nb; j++) {
      if (!this.on[j]) continue;
      for (let i = Math.max(0, j - 4); i <= Math.min(nb - 1, j + 4); i++) {
        const a = Math.log(fc[i] * Math.pow(10, -0.05)), b = Math.log(Math.min(fs / 2 * 0.9999, fc[i] * Math.pow(10, 0.05)));
        if (b <= a) continue;
        let v = 0;
        const n = 200;
        let fp = Math.exp(a), pp = sectionsPower(this.secs[j], fp, fs);
        for (let k = 1; k <= n; k++) {
          const f = Math.exp(a + (b - a) * k / n), q = sectionsPower(this.secs[j], f, fs);
          v += (q + pp) / 2 * (f - fp) * 2 / fs; fp = f; pp = q;
        }
        this.M[i * nb + j] = v * this.inv[j] * this.inv[j];
      }
    }
    this.xD = new Float64Array(nb); this.xU = new Float64Array(nb); this.mx = new Float64Array(nb);
    // smoothed parameters and the gains last applied
    this.cDir = new Float64Array(nb); this.cDif = new Float64Array(nb); this.cLat = new Float64Array(nb); this.cGain = 0;
    this.tDir = new Float64Array(nb); this.tDif = new Float64Array(nb); this.tLat = new Float64Array(nb); this.tGain = 0;
    this.gL = new Float64Array(nb); this.gR = new Float64Array(nb); this.gU = new Float64Array(nb);
    this.hasT = false;
  }

  /** a message from the main thread (see the header) */
  message(d) {
    if (d.bands) { this._setup(d.bands); return; }
    if (!this.nb) return;
    this.tDir.set(d.dir); this.tDif.set(d.dif); this.tLat.set(d.lat); this.tGain = d.gain;
    if (!this.hasT) { this.cDir.set(d.dir); this.cDif.set(d.dif); this.cLat.set(d.lat); this.cGain = d.gain; this.hasT = true; }
  }

  /** filter output powers x >= 0 whose power in each band (M x) matches the targets t (Richardson-Lucy) */
  _unleak(t, x) {
    const nb = this.nb, M = this.M, mx = this.mx;
    const on = this.on;                                  // (bands above ~0.45 fs cannot be made: left out)
    for (let j = 0; j < nb; j++) x[j] = on[j] ? Math.max(0, t[j]) : 0;
    for (let it = 0; it < 10; it++) {
      for (let i = 0; i < nb; i++) {
        let v = 0;
        for (let j = Math.max(0, i - 4); j <= Math.min(nb - 1, i + 4); j++) v += M[i * nb + j] * x[j];
        mx[i] = v;
      }
      for (let j = 0; j < nb; j++) {
        if (!(x[j] > 0)) continue;
        let num = 0, den = 0;
        for (let i = Math.max(0, j - 4); i <= Math.min(nb - 1, j + 4); i++) {
          if (!on[i]) continue;
          const m = M[i * nb + j];
          if (mx[i] > 0) num += m * t[i] / mx[i];
          den += m;
        }
        if (den > 0) x[j] *= num / den;
      }
    }
  }

  /** fill two output channels (any length) */
  render(L, R) {
    L.fill(0); R.fill(0);
    if (!this.nb || !this.hasT) return;
    for (let o = 0; o < L.length; o += 128) this._block(L, R, o, Math.min(128, L.length - o));
  }

  _block(L, R, off, m) {
    const fs = this.fs, nb = this.nb, S = ORDER, k = 1 - Math.exp(-m / fs / TAU);
    this.cGain += (this.tGain - this.cGain) * k;
    const gt = this.gt, ring = this.ring, co = this.co, st = this.st, w0 = this.w;
    let seed = this.seed;
    for (let i = 0; i < nb; i++) {
      this.cDir[i] += (this.tDir[i] - this.cDir[i]) * k;
      this.cDif[i] += (this.tDif[i] - this.cDif[i]) * k;
      this.cLat[i] += (this.tLat[i] - this.cLat[i]) * k;
    }
    this._unleak(this.cDir, this.xD); this._unleak(this.cDif, this.xU);
    for (let i = 0; i < nb; i++) {
      if (!this.on[i]) continue;
      // target gains of this block: level, head shadow (relative to a frontal source), time difference
      const lat = Math.max(-1, Math.min(1, this.cLat[i])), f = this.fc[i];
      const thR = Math.acos(lat), front = headShadow2(f, Math.PI / 2);
      const hR = Math.sqrt(headShadow2(f, thR) / front), hL = Math.sqrt(headShadow2(f, Math.PI - thR) / front);
      const amp = this.cGain * this.inv[i];
      const ad = Math.sqrt(this.xD[i]) * amp, au = Math.sqrt(this.xU[i]) * amp;
      const gL1 = ad * hL, gR1 = ad * hR, gU1 = au;
      const gL0 = this.gL[i], gR0 = this.gR[i], gU0 = this.gU[i];
      this.gL[i] = gL1; this.gR[i] = gR1; this.gU[i] = gU1;
      const sb = i * 2 * S * 2;
      if (gL0 + gR0 + gU0 + gL1 + gR1 + gU1 < 1e-12) {
        st.fill(0, sb, sb + 2 * S * 2);                  // silent band: let its filters rest
        continue;
      }
      const phi = Math.asin(lat), itd = HEAD_A / C_AIR * (phi + Math.sin(phi)) * fs;   // > 0: source on the right
      const dL = Math.min(MAXD - 1, Math.round(Math.max(0, itd))), dR = Math.min(MAXD - 1, Math.round(Math.max(0, -itd)));
      const rb = i * MAXD;
      const cb = i * S * 4;
      for (let t = 0; t < m; t++) {
        const u = (t + 1) / m;
        // three Gaussian numbers from two hashes
        let h = (seed = (seed + 0x6D2B79F5) >>> 0);
        h = Math.imul(h ^ (h >>> 15), h | 1); h ^= h + Math.imul(h ^ (h >>> 7), h | 61); h = (h ^ (h >>> 14)) >>> 0;
        let h2 = (seed = (seed + 0x6D2B79F5) >>> 0);
        h2 = Math.imul(h2 ^ (h2 >>> 15), h2 | 1); h2 ^= h2 + Math.imul(h2 ^ (h2 >>> 7), h2 | 61); h2 = (h2 ^ (h2 >>> 14)) >>> 0;
        const n1 = gt[h & 0xffff], n2 = gt[h >>> 16], n3 = gt[h2 & 0xffff];
        const wi = (w0 + t) & (MAXD - 1);
        ring[rb + wi] = n1;
        const xL = (gL0 + (gL1 - gL0) * u) * ring[rb + ((wi - dL) & (MAXD - 1))] + (gU0 + (gU1 - gU0) * u) * n2;
        const xR = (gR0 + (gR1 - gR0) * u) * ring[rb + ((wi - dR) & (MAXD - 1))] + (gU0 + (gU1 - gU0) * u) * n3;
        // cascades (direct form II transposed; b1 = 0)
        let yL = xL, yR = xR;
        for (let s = 0; s < S; s++) {
          const c = cb + s * 4, b0 = co[c], b2 = co[c + 1], a1 = co[c + 2], a2 = co[c + 3];
          const pL = sb + s * 2, pR = sb + (S + s) * 2;
          const oL = b0 * yL + st[pL];
          st[pL] = st[pL + 1] - a1 * oL; st[pL + 1] = b2 * yL - a2 * oL;
          const oR = b0 * yR + st[pR];
          st[pR] = st[pR + 1] - a1 * oR; st[pR + 1] = b2 * yR - a2 * oR;
          yL = oL; yR = oR;
        }
        L[off + t] += yL; R[off + t] += yR;
      }
    }
    this.seed = seed;
    this.w = (w0 + m) & (MAXD - 1);
  }
}
