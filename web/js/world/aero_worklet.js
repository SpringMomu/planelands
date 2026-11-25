// AudioWorklet: stationary Gaussian pressure noise with a prescribed spectrum (the aeroacoustic model's
// 1/3-octave band levels, web/js/world/aeroacoustics.js), stereo.  Random-phase spectral synthesis:
// every hop an 8192-point spectrum with independent complex-Gaussian bins of variance N fs G(f) / 2
// (G: one-sided PSD, log-log interpolated between the band centres, then rescaled so each band holds
// exactly its mean-square pressure) is inverse transformed, Hann windowed and overlap-added at 75 %
// (the squared windows sum to 1.5, divided out): the output has the requested band levels and Gaussian
// statistics; a new spectrum every 43 ms follows the flight.
// Direct sound: the same noise on both channels with per-band level difference (correlated, a source
// direction); diffuse sound: independent noise per channel (cabin reverberation).
//
// messages: { bands: Float64Array(31) centre frequencies }  once;
//           { dir, wl, dif: Float32Array(31) (band Pa^2, left share 0..1, band Pa^2), gain (digital/Pa) }

const N = 8192, H = 2048, LOG2N = 13;

class AeroNoise extends AudioWorkletProcessor {
  constructor() {
    super();
    this.fs = sampleRate;
    this.re = new Float64Array(N); this.im = new Float64Array(N);
    this.cos = new Float64Array(N / 2); this.sin = new Float64Array(N / 2);
    for (let k = 0; k < N / 2; k++) { this.cos[k] = Math.cos(2 * Math.PI * k / N); this.sin[k] = Math.sin(2 * Math.PI * k / N); }
    this.rev = new Uint32Array(N);
    for (let i = 0; i < N; i++) { let r = 0; for (let b = 0; b < LOG2N; b++) r |= ((i >> b) & 1) << (LOG2N - 1 - b); this.rev[i] = r; }
    this.win = new Float64Array(N);
    const norm = 1 / Math.sqrt(1.5);
    for (let n = 0; n < N; n++) this.win[n] = (0.5 - 0.5 * Math.cos(2 * Math.PI * n / N)) * norm;
    this.accL = new Float64Array(N); this.accR = new Float64Array(N);
    this.outL = new Float32Array(H); this.outR = new Float32Array(H); this.outPos = H;
    this.nb = 0; this.binA = null; this.binW = null;
    this.tgt = null; this.cur = null;
    this.seed = (Math.random() * 4294967296) >>> 0;
    this.spare = null;
    this.port.onmessage = (e) => this._msg(e.data);
  }

  _msg(d) {
    if (d.bands) {
      // per FFT bin: lower band index and interpolation weight on log frequency
      const fc = d.bands, nb = fc.length;
      this.nb = nb;
      this.binA = new Int16Array(N / 2); this.binW = new Float32Array(N / 2); this.binB = new Int16Array(N / 2).fill(-1);
      this.pd = new Float64Array(N / 2); this.pu = new Float64Array(N / 2);
      const lo = fc[0] * Math.pow(10, -0.05), hi = fc[nb - 1] * Math.pow(10, 0.05);
      for (let k = 0; k < N / 2; k++) {
        const f = k * this.fs / N;
        if (f < lo || f > hi || f > 0.48 * this.fs) { this.binA[k] = -1; continue; }
        const x = Math.min(nb - 1, Math.max(0, 10 * Math.log10(f / fc[0])));   // fractional band index
        const a = Math.min(nb - 2, Math.floor(x));
        this.binA[k] = a; this.binW[k] = x - a;
        const b = Math.round(10 * Math.log10(f / fc[0]));                     // the band this bin belongs to
        if (b >= 0 && b < nb) this.binB[k] = b;
      }
      this.bw = Float64Array.from(fc, (f) => f * (Math.pow(10, 0.05) - Math.pow(10, -0.05)));
      return;
    }
    this.tgt = d;
    if (!this.cur) this.cur = { dir: Float64Array.from(d.dir), wl: Float64Array.from(d.wl), dif: Float64Array.from(d.dif), gain: d.gain };
  }

  _rnd() {            // uniform (0, 1]  (mulberry32)
    let t = (this.seed = (this.seed + 0x6D2B79F5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return (((t ^ (t >>> 14)) >>> 0) + 1) / 4294967297;
  }

  _gauss() {          // standard normal (Box-Muller, pairs)
    if (this.spare !== null) { const s = this.spare; this.spare = null; return s; }
    const r = Math.sqrt(-2 * Math.log(this._rnd())), a = 2 * Math.PI * this._rnd();
    this.spare = r * Math.sin(a);
    return r * Math.cos(a);
  }

  _ifft() {
    const re = this.re, im = this.im, rev = this.rev;
    for (let i = 0; i < N; i++) {
      const j = rev[i];
      if (j > i) { let t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (let size = 2; size <= N; size <<= 1) {
      const half = size >> 1, step = N / size;
      for (let s = 0; s < N; s += size) {
        for (let k = 0; k < half; k++) {
          const c = this.cos[k * step], sn = this.sin[k * step];      // exp(+j 2 pi k / size): inverse
          const a = s + k, b = a + half;
          const tr = re[b] * c - im[b] * sn, ti = re[b] * sn + im[b] * c;
          re[b] = re[a] - tr; im[b] = im[a] - ti;
          re[a] += tr; im[a] += ti;
        }
      }
    }
  }

  _hop() {
    const re = this.re, im = this.im;
    re.fill(0); im.fill(0);
    const T = this.tgt, C = this.cur;
    let on = false;
    if (T && C && this.binA) {
      // parameters glide towards the latest message (~30 ms)
      const k = 1 - Math.exp(-H / this.fs / 0.03);
      for (let i = 0; i < this.nb; i++) {
        C.dir[i] += (T.dir[i] - C.dir[i]) * k; C.dif[i] += (T.dif[i] - C.dif[i]) * k; C.wl[i] += (T.wl[i] - C.wl[i]) * k;
      }
      C.gain += (T.gain - C.gain) * k;
      const g2 = C.gain * C.gain, nb = this.nb;
      const lD = new Float64Array(nb), lU = new Float64Array(nb);
      let any = 0;
      for (let i = 0; i < nb; i++) {
        lD[i] = Math.log(Math.max(1e-30, C.dir[i] * g2 / this.bw[i]));
        lU[i] = Math.log(Math.max(1e-30, C.dif[i] * g2 / this.bw[i]));
        any = Math.max(any, C.dir[i] * g2, C.dif[i] * g2);
      }
      on = any > 1e-14;
      if (on) {
        // PSD per bin, then each band rescaled to its exact mean square (steep / resonant spectra)
        const pd = this.pd, pu = this.pu, df = this.fs / N;
        const sD = new Float64Array(nb), sU = new Float64Array(nb);
        for (let kb = 1; kb < N / 2; kb++) {
          const a = this.binA[kb];
          if (a < 0) { pd[kb] = 0; pu[kb] = 0; continue; }
          const w = this.binW[kb];
          pd[kb] = Math.exp(lD[a] + (lD[a + 1] - lD[a]) * w);
          pu[kb] = Math.exp(lU[a] + (lU[a + 1] - lU[a]) * w);
          const b = this.binB[kb];
          if (b >= 0) { sD[b] += pd[kb] * df; sU[b] += pu[kb] * df; }
        }
        for (let i = 0; i < nb; i++) {
          sD[i] = sD[i] > 0 ? C.dir[i] * g2 / sD[i] : 1;
          sU[i] = sU[i] > 0 ? C.dif[i] * g2 / sU[i] : 1;
        }
        const sc = N * this.fs / 4;                // variance per real / imaginary part of a bin
        for (let kb = 1; kb < N / 2; kb++) {
          const a = this.binA[kb];
          if (a < 0) continue;
          const w = this.binW[kb], b = this.binB[kb];
          const sd = Math.sqrt(pd[kb] * (b >= 0 ? sD[b] : 1) * sc);
          const su = Math.sqrt(pu[kb] * (b >= 0 ? sU[b] : 1) * sc);
          const wl = C.wl[a] + (C.wl[a + 1] - C.wl[a]) * w;
          const gl = Math.sqrt(2 * Math.min(1, Math.max(0, wl))), gr = Math.sqrt(2 * Math.min(1, Math.max(0, 1 - wl)));
          const xr = this._gauss() * sd, xi = this._gauss() * sd;
          // left / right bin values
          const a1 = gl * xr + su * this._gauss(), b1 = gl * xi + su * this._gauss();
          const c1 = gr * xr + su * this._gauss(), d1 = gr * xi + su * this._gauss();
          // pack: z = l + j r  ->  Z_k = L_k + j R_k,  Z_{N-k} = conj(L_k) + j conj(R_k)
          re[kb] = a1 - d1; im[kb] = b1 + c1;
          re[N - kb] = a1 + d1; im[N - kb] = c1 - b1;
        }
        this._ifft();
      }
    }
    const aL = this.accL, aR = this.accR, w = this.win;
    if (on) for (let n = 0; n < N; n++) { aL[n] += re[n] / N * w[n]; aR[n] += im[n] / N * w[n]; }
    for (let n = 0; n < H; n++) { this.outL[n] = aL[n]; this.outR[n] = aR[n]; }
    aL.copyWithin(0, H); aR.copyWithin(0, H);
    aL.fill(0, N - H); aR.fill(0, N - H);
    this.outPos = 0;
  }

  process(inputs, outputs) {
    const out = outputs[0];
    const L = out[0], R = out[1] || out[0];
    for (let n = 0; n < L.length; n++) {
      if (this.outPos >= H) this._hop();
      L[n] = this.outL[this.outPos]; R[n] = this.outR[this.outPos];
      this.outPos++;
    }
    return true;
  }
}

registerProcessor('aero-noise', AeroNoise);
