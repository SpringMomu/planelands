// Spectral noise synthesiser of the airflow sound (runs in the AudioWorklet, aero_worklet.js, or on the
// main thread through a ScriptProcessorNode where AudioWorklet is unavailable): stationary Gaussian
// pressure noise with a prescribed spectrum (the aeroacoustic model's 1/3-octave band levels,
// web/js/world/aeroacoustics.js), stereo.  Random-phase spectral synthesis: every hop an 8192-point
// spectrum with independent complex-Gaussian bins of variance N fs G(f) / 2 (G: one-sided PSD, log-log
// interpolated between the band centres, then rescaled so each band holds exactly its mean-square
// pressure) is inverse transformed, Hann windowed and overlap-added at 75 % (the squared windows sum
// to 1.5, divided out): the output has the requested band levels and Gaussian statistics; a new
// spectrum every 43 ms follows the flight.
// Direct sound: the same noise on both channels with per-band level difference (correlated, a source
// direction); diffuse sound: independent noise per channel (cabin reverberation).
//
// Real-time safety: the next frame is computed while the current one plays, in ~33 small steps spread
// over the render calls (3 per 128 samples, a few tens of microseconds each) - no render call carries a
// whole 8192-point synthesis, which overran the audio callback budget and crackled.  Gaussian numbers
// come from a table drawn once (Box-Muller) and read at hashed random positions; no allocation per frame.
//
// messages: { bands: Float64Array(31) centre frequencies }  once;
//           { dir, wl, dif: Float32Array(31) (band Pa^2, left share 0..1, band Pa^2), gain (digital/Pa) }

const N = 8192, H = 2048, LOG2N = 13;
const CHUNK_PSD = 1024, CHUNK_NOISE = 256;   // spectrum bins per work step
const GT = 1 << 17;                // table of standard normal numbers, read at random positions
const STEPS_PER_128 = 3;           // work steps per 128 output samples (a frame needs 35 of the 48)

export class SpectralNoise {
  constructor(fs) {
    this.fs = fs;
    this.re = new Float64Array(N); this.im = new Float64Array(N);
    this.cos = new Float64Array(N / 2); this.sin = new Float64Array(N / 2);
    for (let k = 0; k < N / 2; k++) { this.cos[k] = Math.cos(2 * Math.PI * k / N); this.sin[k] = Math.sin(2 * Math.PI * k / N); }
    this.rev = new Uint32Array(N);
    for (let i = 0; i < N; i++) { let r = 0; for (let b = 0; b < LOG2N; b++) r |= ((i >> b) & 1) << (LOG2N - 1 - b); this.rev[i] = r; }
    this.win = new Float64Array(N);
    const norm = 1 / Math.sqrt(1.5);
    for (let n = 0; n < N; n++) this.win[n] = (0.5 - 0.5 * Math.cos(2 * Math.PI * n / N)) * norm / N;   // (1/N of the inverse FFT)
    this.accL = new Float64Array(N); this.accR = new Float64Array(N);
    this.outL = new Float32Array(H); this.outR = new Float32Array(H); this.outPos = H;
    this.nb = 0; this.binA = null; this.binW = null; this.binB = null;
    this.tgt = null; this.cur = null;
    this.seed = (Math.random() * 4294967296) >>> 0;
    this.spare = null;
    this.gt = new Float32Array(GT);
    for (let i = 0; i < GT; i++) this.gt[i] = this._gaussBM();
    this.job = null; this.jobFinished = true; this.on = false;
  }

  /** a message from the main thread (see the header) */
  message(d) {
    if (d.bands) {
      // per FFT bin: lower band index and interpolation weight on log frequency, and its own band
      const fc = d.bands, nb = fc.length;
      this.nb = nb;
      this.binA = new Int16Array(N / 2); this.binW = new Float32Array(N / 2); this.binB = new Int16Array(N / 2).fill(-1);
      this.pd = new Float64Array(N / 2); this.pu = new Float64Array(N / 2);
      this.lD = new Float64Array(nb); this.lU = new Float64Array(nb); this.sD = new Float64Array(nb); this.sU = new Float64Array(nb);
      const lo = fc[0] * Math.pow(10, -0.05), hi = fc[nb - 1] * Math.pow(10, 0.05);
      for (let k = 0; k < N / 2; k++) {
        const f = k * this.fs / N;
        if (f < lo || f > hi || f > 0.48 * this.fs) { this.binA[k] = -1; continue; }
        const x = Math.min(nb - 1, Math.max(0, 10 * Math.log10(f / fc[0])));   // fractional band index
        const a = Math.min(nb - 2, Math.floor(x));
        this.binA[k] = a; this.binW[k] = x - a;
        const b = Math.round(10 * Math.log10(f / fc[0]));
        if (b >= 0 && b < nb) this.binB[k] = b;
      }
      this.bw = Float64Array.from(fc, (f) => f * (Math.pow(10, 0.05) - Math.pow(10, -0.05)));
      return;
    }
    if (!this.cur) this.cur = { dir: Float64Array.from(d.dir), wl: Float64Array.from(d.wl), dif: Float64Array.from(d.dif), gain: d.gain };
    if (!this.tgt) this.tgt = { dir: new Float64Array(d.dir.length), wl: new Float64Array(d.dir.length), dif: new Float64Array(d.dir.length), gain: 0 };
    this.tgt.dir.set(d.dir); this.tgt.wl.set(d.wl); this.tgt.dif.set(d.dif); this.tgt.gain = d.gain;
  }

  _rnd() {            // uniform (0, 1]  (mulberry32)
    let t = (this.seed = (this.seed + 0x6D2B79F5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return (((t ^ (t >>> 14)) >>> 0) + 1) / 4294967297;
  }

  _gauss() {          // standard normal: the table at a random position (one integer hash per number)
    let t = (this.seed = (this.seed + 0x6D2B79F5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return this.gt[((t ^ (t >>> 14)) >>> 0) & (GT - 1)];
  }

  _gaussBM() {        // standard normal (Box-Muller, pairs)
    if (this.spare !== null) { const s = this.spare; this.spare = null; return s; }
    const r = Math.sqrt(-2 * Math.log(this._rnd())), a = 2 * Math.PI * this._rnd();
    this.spare = r * Math.sin(a);
    return r * Math.cos(a);
  }

  /** the next frame (spectrum -> inverse FFT into re / im), one small step per next() */
  * _frame() {
    const re = this.re, im = this.im, T = this.tgt, C = this.cur;
    this.on = false;
    if (!T || !C || !this.binA) return;
    // parameters glide towards the latest message (~30 ms)
    const k = 1 - Math.exp(-H / this.fs / 0.03), nb = this.nb;
    for (let i = 0; i < nb; i++) {
      C.dir[i] += (T.dir[i] - C.dir[i]) * k; C.dif[i] += (T.dif[i] - C.dif[i]) * k; C.wl[i] += (T.wl[i] - C.wl[i]) * k;
    }
    C.gain += (T.gain - C.gain) * k;
    const g2 = C.gain * C.gain, lD = this.lD, lU = this.lU, sD = this.sD, sU = this.sU;
    let any = 0;
    for (let i = 0; i < nb; i++) {
      lD[i] = Math.log(Math.max(1e-30, C.dir[i] * g2 / this.bw[i]));
      lU[i] = Math.log(Math.max(1e-30, C.dif[i] * g2 / this.bw[i]));
      any = Math.max(any, C.dir[i] * g2, C.dif[i] * g2);
      sD[i] = 0; sU[i] = 0;
    }
    if (!(any > 1e-14)) return;
    re.fill(0); im.fill(0);
    yield;
    // PSD per bin, then each band rescaled to its exact mean square (steep / resonant spectra)
    const pd = this.pd, pu = this.pu, df = this.fs / N;
    for (let k0 = 1; k0 < N / 2; k0 += CHUNK_PSD) {
      for (let kb = k0; kb < Math.min(N / 2, k0 + CHUNK_PSD); kb++) {
        const a = this.binA[kb];
        if (a < 0) { pd[kb] = 0; pu[kb] = 0; continue; }
        const w = this.binW[kb];
        pd[kb] = Math.exp(lD[a] + (lD[a + 1] - lD[a]) * w);
        pu[kb] = Math.exp(lU[a] + (lU[a + 1] - lU[a]) * w);
        const b = this.binB[kb];
        if (b >= 0) { sD[b] += pd[kb] * df; sU[b] += pu[kb] * df; }
      }
      yield;
    }
    for (let i = 0; i < nb; i++) {
      sD[i] = sD[i] > 0 ? C.dir[i] * g2 / sD[i] : 1;
      sU[i] = sU[i] > 0 ? C.dif[i] * g2 / sU[i] : 1;
    }
    const sc = N * this.fs / 4;                  // variance per real / imaginary part of a bin
    for (let k0 = 1; k0 < N / 2; k0 += CHUNK_NOISE) {
      for (let kb = k0; kb < Math.min(N / 2, k0 + CHUNK_NOISE); kb++) {
        const a = this.binA[kb];
        if (a < 0) continue;
        const w = this.binW[kb], b = this.binB[kb];
        const sd = Math.sqrt(pd[kb] * (b >= 0 ? sD[b] : 1) * sc);
        const su = Math.sqrt(pu[kb] * (b >= 0 ? sU[b] : 1) * sc);
        const wl = C.wl[a] + (C.wl[a + 1] - C.wl[a]) * w;
        const gl = Math.sqrt(2 * Math.min(1, Math.max(0, wl))), gr = Math.sqrt(2 * Math.min(1, Math.max(0, 1 - wl)));
        const xr = this._gauss() * sd, xi = this._gauss() * sd;
        const a1 = gl * xr + su * this._gauss(), b1 = gl * xi + su * this._gauss();
        const c1 = gr * xr + su * this._gauss(), d1 = gr * xi + su * this._gauss();
        // pack two real signals: z = l + j r  ->  Z_k = L_k + j R_k,  Z_{N-k} = conj(L_k) + j conj(R_k)
        re[kb] = a1 - d1; im[kb] = b1 + c1;
        re[N - kb] = a1 + d1; im[N - kb] = c1 - b1;
      }
      yield;
    }
    // inverse FFT: bit reversal, then one butterfly stage per step
    const rev = this.rev;
    for (let i = 0; i < N; i++) {
      const j = rev[i];
      if (j > i) { let t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    yield;
    for (let size = 2; size <= N; size <<= 1) {
      const half = size >> 1, step = N / size;
      for (let s = 0; s < N; s += size) {
        for (let j = 0; j < half; j++) {
          const c = this.cos[j * step], sn = this.sin[j * step];       // exp(+j 2 pi j / size): inverse
          const a = s + j, b = a + half;
          const tr = re[b] * c - im[b] * sn, ti = re[b] * sn + im[b] * c;
          re[b] = re[a] - tr; im[b] = im[a] - ti;
          re[a] += tr; im[a] += ti;
        }
      }
      if (size < N) yield;
    }
    this.on = true;
  }

  /** hop boundary: finish the frame if needed, overlap-add it, hand out the next H samples, start the next frame */
  _advance() {
    if (this.job && !this.jobFinished) while (!this.job.next().done);     // (normally already done)
    const aL = this.accL, aR = this.accR;
    if (this.on) {
      const re = this.re, im = this.im, w = this.win;
      for (let n = 0; n < N; n++) { aL[n] += re[n] * w[n]; aR[n] += im[n] * w[n]; }
    }
    for (let n = 0; n < H; n++) { this.outL[n] = aL[n]; this.outR[n] = aR[n]; }
    aL.copyWithin(0, H); aR.copyWithin(0, H);
    aL.fill(0, N - H); aR.fill(0, N - H);
    this.outPos = 0;
    this.job = this._frame(); this.jobFinished = false;
  }

  /** fill two output channels (any length) */
  render(L, R) {
    let n = 0;
    while (n < L.length) {
      if (this.outPos >= H) this._advance();
      const m = Math.min(L.length - n, H - this.outPos, 128);
      for (let i = 0; i < m; i++) { L[n + i] = this.outL[this.outPos + i]; R[n + i] = this.outR[this.outPos + i]; }
      n += m; this.outPos += m;
      if (!this.jobFinished) for (let s = 0; s < STEPS_PER_128; s++) if (this.job.next().done) { this.jobFinished = true; break; }
    }
  }
}
