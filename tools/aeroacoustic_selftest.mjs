// Self-test of the aeroacoustic model (web/js/world/aeroacoustics.js) against reference values, plus
// the levels it gives in typical flight conditions.      node tools/aeroacoustic_selftest.mjs
import fs from 'node:fs';
import * as A from '../web/js/world/aeroacoustics.js';

const geo = JSON.parse(fs.readFileSync(new URL('../shared/orbiter_geometry.json', import.meta.url)));
let fails = 0;
const check = (name, ok, info = '') => { console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}${info ? '  ' + info : ''}`); if (!ok) fails++; };
const dB = (p2) => 10 * Math.log10(p2 / 4e-10);
const sum = (a) => a.reduce((s, v) => s + v, 0);

// ---- ISO 9613-1 against ISO 9613-2 Table 2 (10 C, 70 % RH, dB/km at the octave mid-bands)
{
  const T = 283.15, p = 101325, h = A.molarH2O(70, T, p);
  const ref = [[63, 0.1], [125, 0.4], [250, 1.0], [500, 1.9], [1000, 3.7], [2000, 9.7], [4000, 32.8], [8000, 117]];
  let worst = 0;
  // (the table is rounded: 0.1 dB/km steps, 3 significant digits)
  for (const [f, a] of ref) worst = Math.max(worst, Math.abs(A.isoAlpha(f, T, p, h) * 1000 - a) / Math.max(a, 1.0));
  check('ISO 9613-1 absorption vs ISO 9613-2 table (10 C, 70 %)', worst < 0.05, `max error ${(worst * 100).toFixed(1)} % (of max(a, 1 dB/km))`);
}

// ---- US76
{
  const a = A.atmosphere(11000 * 6356766 / (6356766 - 11000));     // 11 km geopotential
  check('US76 at the tropopause', Math.abs(a.T - 216.65) < 0.05 && Math.abs(a.p - 22632) / 22632 < 2e-3, `T ${a.T.toFixed(2)} p ${a.p.toFixed(0)}`);
}

// ---- Fink spectra: each sums to 1 over 1/3-octave bands
for (const [k, F] of Object.entries(A.FINK_SPECTRA)) {
  let s = 0;
  for (let n = -60; n <= 60; n++) s += F(Math.pow(10, n / 10));
  check(`Fink ${k} sums to 1 per 1/3 octave`, Math.abs(s - 1) < 0.02, s.toFixed(4));
}

// ---- TBL: level against Lowson's flight correlation; bands carry part of p_rms^2 (the rest is ultrasonic)
for (const [U, alt] of [[150, 0], [265, 11000], [740, 25000]]) {
  const atm = A.atmosphere(alt);
  const bl = A.boundaryLayer(U, atm, 6, 1.0);          // smooth wall, as Lowson's data
  const b = A.tblBands(U, atm, 6, undefined, 1.0);
  const frac = sum(b) / bl.prms2, dl = 10 * Math.log10(bl.prms2 / bl.lowson);
  check(`TBL M${bl.M.toFixed(2)}: Farabee-Casarella level within 5 dB of Lowson (smooth wall)`, Math.abs(dl) < 5 && frac > 0.2 && frac <= 1.0001,
    `p_rms ${Math.sqrt(bl.prms2).toFixed(1)} Pa (${dB(bl.prms2).toFixed(1)} dB, Lowson ${dl >= 0 ? '+' : ''}${dl.toFixed(1)} dB), delta ${(bl.delta * 1000).toFixed(1)} mm, Re_tau ${bl.ReTau.toFixed(0)}, audible share ${frac.toFixed(2)}`);
}

// ---- transfer matrix: single limp plate, normal incidence = mass law; double wall resonance
{
  const air = { rho: 1.2, c: 343 };
  const t = A.transmissionBands([{ m: 10, D: 0, fr: 0.001, eta: 0 }], air, air, air, undefined, 0);
  const i = 17, f = A.FC[i];
  const mass = 1 / (1 + Math.pow(Math.PI * f * 10 / (1.2 * 343), 2));
  check('TMM single plate = normal-incidence mass law (1 kHz)', Math.abs(10 * Math.log10(t[i] / mass)) < 0.1,
    `TL ${(-10 * Math.log10(t[i])).toFixed(2)} dB vs ${(-10 * Math.log10(mass)).toFixed(2)} dB`);
  const m1 = 10, m2 = 10, d = 0.1;
  const f0 = Math.sqrt(1.2 * 343 * 343 * (m1 + m2) / (d * m1 * m2)) / (2 * Math.PI);
  const t2 = A.transmissionBands([{ m: m1, D: 0, fr: 0.001, eta: 0 }, { d, air: 'out', sigma: 0 }, { m: m2, D: 0, fr: 0.001, eta: 0 }], air, air, air, undefined, 0);
  let ib = 0; for (let k = 0; k < A.NB; k++) if (t2[k] > t2[ib]) ib = k;
  check('TMM double wall: mass-air-mass resonance', Math.abs(Math.log2(A.FC[ib] / f0)) < 0.34, `peak ${A.FC[ib].toFixed(0)} Hz vs f0 ${f0.toFixed(0)} Hz`);
}

// ---- straight flight helper
function flight({ V = 150, alt = 1000, t0 = -60, t1 = 0, dt = 0.05, gear = 0, surf = {}, x0 = 0 } = {}) {
  const ac = new A.AeroAcoustics(geo);
  for (let t = t0; t <= t1 + 1e-9; t += dt) {
    ac.push({ t, pos: [x0 + V * t, alt, 0], vel: [V, 0, 0], wind: [0, 0, 0], fwd: [1, 0, 0], up: [0, 1, 0], right: [0, 0, 1],
      alt, agl: alt, surf, gear });
  }
  return ac;
}
const lis = (pos, extra = {}) => ({ t: 0, pos, vel: [0, 0, 0], alt: pos[1], groundAlt: 0, right: [0, 0, 1], cabin: false, rh: 60, ...extra });

// ---- retarded time: subsonic flyover, listener 300 m below the path at x = 0
{
  const ac = flight({ V: 150, alt: 300 });
  const c = A.atmosphere(150).c;          // (the model averages c over the layer; close to the mid value)
  const out = { ...ac.listen(lis([0, 1.7, 0])) };
  // analytic for the centre of gravity: |(0,h) - (V te, h)| = c (0 - te)  ->  te = -h' / sqrt(c^2 - V^2)
  const h = 300 - 1.7, te = -h / Math.sqrt(c * c - 150 * 150);
  const [tc] = ac._emissions(0, [0, 1.7, 0], c, [0, 0, 0], 100);
  check('retarded time (subsonic flyover)', Math.abs(tc - te) < 0.005 && Math.abs(out.delay + te) < 0.1,
    `CG emission ${(-tc).toFixed(3)} s before vs ${(-te).toFixed(3)} s; first part heard ${out.delay.toFixed(3)} s, ${out.spl.toFixed(1)} dB`);
}

// ---- supersonic: ahead of the Mach cone silence, behind it two arrivals
{
  const ac = flight({ V: 600, alt: 5000, t0: -90, x0: 0 });
  const ahead = { ...ac.listen(lis([2000, 1.7, 0])) };
  const behind = { ...ac.listen(lis([-20000, 1.7, 0])) };
  check('supersonic: silent ahead of the Mach cone', ahead.arrivals === 0 && ahead.spl === 0);
  check('supersonic: two emission points inside the cone', behind.arrivals === 2, `arrivals ${behind.arrivals}, ${behind.spl.toFixed(1)} dB`);
}

// ---- supersonic chase camera near the Mach cone: convective amplification stays bounded
{
  const ac = flight({ V: 420, alt: 14000 });          // M 1.4
  let worst = 0;
  for (let k = 0; k <= 12; k++) {                      // camera on an arc 75 m behind / above
    const a = k / 12 * Math.PI / 2;
    const o = ac.listen({ ...lis([-75 * Math.cos(a), 14000 + 75 * Math.sin(a), 0]), vel: [420, 0, 0] });
    worst = Math.max(worst, o.spl);
  }
  check('supersonic chase: bounded near the Mach cone', worst > 60 && worst < 110, `max ${worst.toFixed(1)} dB`);
}

// ---- cabin: aft sources cannot reach the flight deck in supersonic flight
{
  const sub = flight({ V: 200, alt: 3000, surf: { speedbrake: 60 } }), sup = flight({ V: 450, alt: 12000, surf: { speedbrake: 60 } });
  const a = { ...sub.listen({ cabin: true }) }, b = { ...sup.listen({ cabin: true }) };
  check('cabin: levels finite', a.spl > 40 && a.spl < 130 && b.spl > 30 && b.spl < 130, `M0.6 ${a.spl.toFixed(1)} dB, M1.5 ${b.spl.toFixed(1)} dB`);
}

// ---- the synthesiser (AudioWorklet / ScriptProcessor): band levels, two ears, latency, real-time cost
{
  const S = await import('../web/js/world/aero_synth.js');
  const fs = 48000;
  const make = (msg) => { const p = new S.SpectralNoise(fs); p.message({ bands: Float64Array.from(A.FC) }); if (msg) p.message(msg); return p; };
  const run = (p, seconds) => {
    const n = Math.floor(seconds * fs / 128) * 128, L = new Float32Array(n), R = new Float32Array(n);
    for (let o = 0; o < n; o += 128) p.render(L.subarray(o, o + 128), R.subarray(o, o + 128));
    return { L: L.subarray(fs / 2), R: R.subarray(fs / 2) };      // skip the filters' start-up
  };
  // 1/3-octave analysis: averaged Hann periodograms
  const fft = (re, im) => {
    const n = re.length;
    for (let i = 1, j = 0; i < n; i++) { let b = n >> 1; for (; j & b; b >>= 1) j ^= b; j ^= b; if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; } }
    for (let len = 2; len <= n; len <<= 1) {
      const ang = -2 * Math.PI / len;
      for (let i = 0; i < n; i += len) for (let k = 0; k < len / 2; k++) {
        const c = Math.cos(ang * k), sn = Math.sin(ang * k), a = i + k, b = a + len / 2;
        const tr = re[b] * c - im[b] * sn, ti = re[b] * sn + im[b] * c;
        re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti;
      }
    }
  };
  const bandsOf = (x) => {
    const n = 16384, out = new Float64Array(A.NB);
    let segs = 0, wss = 0;
    const w = Float64Array.from({ length: n }, (_, i) => 0.5 - 0.5 * Math.cos(2 * Math.PI * i / n));
    for (const v of w) wss += v * v;
    for (let o = 0; o + n <= x.length; o += n / 2) {
      const re = new Float64Array(n), im = new Float64Array(n);
      for (let i = 0; i < n; i++) re[i] = x[o + i] * w[i];
      fft(re, im);
      for (let k = 1; k < n / 2; k++) {
        const f = k * fs / n, b = Math.round(10 * Math.log10(f / A.FC[0]));
        if (b >= 0 && b < A.NB) out[b] += 2 * (re[k] * re[k] + im[k] * im[k]) / (n * wss);
      }
      segs++;
    }
    return out.map((v) => v / segs);
  };
  const ms = (x) => x.reduce((s, v) => s + v * v, 0) / x.length;
  const z = new Float32Array(A.NB), one = new Float32Array(A.NB).fill(1e-4);
  // levels: a flat spectrum, and the steep, resonant cabin spectrum
  {
    // (estimates over T seconds scatter by ~4.3 / sqrt(B T) dB: ~0.3 dB at 63 Hz in 20 s)
    const flat = bandsOf(run(make({ dir: one, lat: z, dif: z, gain: 1 }), 20).L);
    let wf = 0; for (let i = 8; i < 30; i++) wf = Math.max(wf, Math.abs(10 * Math.log10(flat[i] / 1e-4)));
    const ac = flight({ V: 135, alt: 100, gear: 1, surf: { speedbrake: 40 } });
    const o = ac.listen({ cabin: true });
    const g = 0.01, tgt = Array.from(o.dir, (v, i) => (v + o.dif[i]) * g * g);
    const { L } = run(make({ dir: Float32Array.from(o.dir), lat: z, dif: Float32Array.from(o.dif), gain: g }), 12);
    const got = bandsOf(L);
    let worst = 0, wb = 0;
    for (let i = 4; i < 29; i++) { const e = Math.abs(10 * Math.log10(got[i] / tgt[i])); if (e > worst) { worst = e; wb = i; } }
    check('synth: band levels follow the model (flat 63 Hz-16 kHz; cabin 50 Hz-12.5 kHz)', wf < 0.75 && worst < 1.5,
      `flat worst ${wf.toFixed(2)} dB; cabin worst ${worst.toFixed(2)} dB at ${A.FC[wb].toFixed(0)} Hz, overall ${(10 * Math.log10(ms(L) / sum(tgt))).toFixed(2)} dB`);
  }
  const corr = (x, y, lag = 0) => { let a = 0, b = 0, c = 0; for (let i = 64; i < x.length - 64; i++) { a += x[i] * y[i + lag]; b += x[i] * x[i]; c += y[i + lag] * y[i + lag]; } return a / Math.sqrt(b * c); };
  {
    const d = run(make({ dir: one, lat: z, dif: z, gain: 1 }), 3), u = run(make({ dir: z, lat: z, dif: one, gain: 1 }), 3);
    check('synth: frontal direct sound identical at both ears, diffuse sound decorrelated', corr(d.L, d.R) > 0.999 && Math.abs(corr(u.L, u.R)) < 0.03,
      `direct ${corr(d.L, d.R).toFixed(4)}, diffuse ${corr(u.L, u.R).toFixed(3)}`);
  }
  // a source on the right: level difference grows with frequency, the left ear hears it later
  {
    const right = new Float32Array(A.NB).fill(1);
    const r = run(make({ dir: one, lat: right, dif: z, gain: 1 }), 8);
    const bl = bandsOf(r.L), br = bandsOf(r.R);
    const ild = (i) => 10 * Math.log10(br[i] / bl[i]);
    const want = (i) => 10 * Math.log10(S.headShadow2(A.FC[i], 0) / S.headShadow2(A.FC[i], Math.PI));
    let bestLag = 0, best = -1;
    for (let lag = 0; lag < 50; lag++) { const c = corr(r.R, r.L, lag); if (c > best) { best = c; bestLag = lag; } }
    const itd = 0.0875 / 343 * (Math.PI / 2 + 1) * fs;
    check('synth: source on the right - head-shadow level difference, Woodworth time difference',
      Math.abs(ild(10) - want(10)) < 1 && Math.abs(ild(26) - want(26)) < 1.5 && ild(10) < 3 && ild(26) > 12 && Math.abs(bestLag - itd) <= 1,
      `ILD ${ild(10).toFixed(1)} dB at 200 Hz, ${ild(26).toFixed(1)} dB at 8 kHz (model ${want(10).toFixed(1)} / ${want(26).toFixed(1)}); ITD ${bestLag} samples (${(bestLag / fs * 1000).toFixed(2)} ms, Woodworth ${(itd / fs * 1000).toFixed(2)} ms)`);
  }
  // no staircase: the page at 30 fps (1600 samples per frame) raising a band by 0.5 dB per frame - the
  // synthesiser's level must rise in every 128-sample block (a staircase: ~9x between the first block of
  // a frame and the last), not jump once a frame; 128-sample blocks against 1600-sample frames leave
  // ~0.5x in a boundary block
  {
    const glide = (jitter) => {
      const p = make(null), inc = [];
      let next = 0, lvl = -60, last = null;
      for (let o = 0; o < 4 * fs; o += 128) {
        if (o >= next) {
          const d = new Float32Array(A.NB); d[17] = Math.pow(10, lvl / 10);
          p.message({ dir: d, lat: z, dif: z, gain: 1 });
          lvl += 0.5; next += jitter ? 1200 + Math.floor(Math.random() * 800) : 1600;
        }
        p.render(new Float32Array(128), new Float32Array(128));
        const v = p.cDir[17] * 10 / Math.LN10;
        if (o > fs && last !== null) inc.push(v - last);
        last = v;
      }
      const mean = inc.reduce((x, y) => x + y, 0) / inc.length;
      return { mean, lo: Math.min(...inc) / mean, hi: Math.max(...inc) / mean, flat: inc.filter((x) => x < 0.05 * mean).length / inc.length };
    };
    const g = glide(false), gj = glide(true);
    check('synth: smooth glide between frames (30 fps, no steps)', g.lo > 0.3 && g.hi < 1.5 && gj.flat < 0.15,
      `steady frames: block increments ${g.lo.toFixed(2)}-${g.hi.toFixed(2)} x mean; jittered frames (25-40 fps): ${(gj.flat * 100).toFixed(0)} % of blocks without change`);
  }
  // latency: a step of the level while the page sends a spectrum every frame (16 ms), time until the
  // 1 kHz band is within 3 dB of the new level; and the glide is monotonic (no steps)
  {
    const p = make({ dir: z, lat: z, dif: z, gain: 1 });
    const frame = 768, quiet = { dir: z, lat: z, dif: z, gain: 1 };
    const tmp = new Float32Array(128), tmp2 = new Float32Array(128);
    for (let o = 0; o < 0.6 * fs; o += 128) { if (o % frame === 0) p.message(quiet); p.render(tmp, tmp2); }
    const b17 = new Float32Array(A.NB); b17[17] = 1e-2;
    const loud = { dir: b17, lat: z, dif: z, gain: 1 };
    const n = fs, L = new Float32Array(n), R = new Float32Array(n);
    for (let o = 0; o < n; o += 128) { if (o % frame === 0) p.message(loud); p.render(L.subarray(o, o + 128), R.subarray(o, o + 128)); }
    const win = 240;                                      // 5 ms rms windows
    let t3 = -1;
    for (let o = 0; o + win <= n; o += win / 4) {
      let e = 0; for (let i = o; i < o + win; i++) e += L[i] * L[i];
      if (e / win > 1e-2 / 2) { t3 = (o + win / 2) / fs * 1000; break; }
    }
    // real-time cost per 128-sample render call, all bands active (cabin spectrum + direction)
    const q = make({ dir: one, lat: new Float32Array(A.NB).fill(0.4), dif: one, gain: 1 });
    const a = new Float32Array(128), b = new Float32Array(128), ts = [];
    for (let i = 0; i < 6000; i++) { const t0 = performance.now(); q.render(a, b); ts.push(performance.now() - t0); }
    const so = ts.slice(500).sort((x, y) => x - y), pq = (x) => so[Math.floor(x * (so.length - 1))];
    check('synth: follows a change within ~30 ms; steady cost per render call', t3 > 0 && t3 < 35 && pq(0.999) < 1.0,
      `1 kHz band within 3 dB after ${t3.toFixed(1)} ms; per 128 samples p50 ${pq(0.5).toFixed(3)} ms, p99.9 ${pq(0.999).toFixed(3)} ms (budget 2.67 ms)`);
  }
}

// ---- playback mapping: physical below the knee, continuous and monotonic soft knee above
{
  const { playbackDb, KNEE } = await import('../web/js/world/aero_audio.js');
  let ok = true, prev = -1e9;
  for (let x = 60; x <= 140; x += 0.25) { const y = x + playbackDb(x); if (y <= prev || Math.abs(playbackDb(x) - playbackDb(x - 0.25)) > 0.2) ok = false; prev = y; }
  check('playback mapping: unchanged below the knee, smooth 2:1 above', ok && playbackDb(85) === 0 && Math.abs(playbackDb(KNEE + 20) + 10) < 1e-9,
    `85 -> ${85 + playbackDb(85)}, 105 -> ${(105 + playbackDb(105)).toFixed(1)}, 115 -> ${(115 + playbackDb(115)).toFixed(1)} dB`);
  // the chase camera around the orbiter (75 m, horizontal plane): physical and played levels
  for (const [name, V, alt, surf] of [['M0.6 5 km, clean', 192, 5000, {}], ['M0.9 10 km, speed brake 60', 270, 10000, { speedbrake: 60 }]]) {
    const ac = flight({ V, alt, surf });
    const row = [];
    for (const az of [0, 45, 90, 135, 180]) {
      const a = az * Math.PI / 180;
      const o = ac.listen({ ...lis([75 * Math.cos(a), alt, 75 * Math.sin(a)]), vel: [V, 0, 0] });
      row.push(`${az}°:${o.spl.toFixed(0)}→${(o.spl + playbackDb(o.spl)).toFixed(0)}`);
    }
    console.log(`     chase around, ${name.padEnd(28)} ${row.join('  ')}  (dB SPL → played)`);
  }
}

// ---- levels in typical conditions (information)
console.log('\nlevels (dB re 20 uPa, overall; A-weighted in brackets)');
const aw = (f) => { const f2 = f * f; const r = 12194 ** 2 * f2 * f2 / ((f2 + 20.6 ** 2) * Math.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194 ** 2)); return 20 * Math.log10(r) + 2.0; };
const lev = (o) => { let p = 0, pa = 0; for (let i = 0; i < A.NB; i++) { const v = o.dir[i] + o.dif[i]; p += v; pa += v * Math.pow(10, aw(A.FC[i]) / 10); } return `${dB(p).toFixed(1)} (${dB(pa).toFixed(1)} dBA)`; };
const spec = (o) => [0, 5, 10, 13, 16, 19, 22, 25, 28].map((i) => `${A.FC[i] < 1000 ? A.FC[i].toFixed(0) : (A.FC[i] / 1000).toFixed(1) + 'k'}:${dB(o.dir[i] + o.dif[i]).toFixed(0)}`).join(' ');
const cases = [
  ['cockpit  TAEM  M2.5 25 km', { V: 740, alt: 25000 }, { cabin: true }],
  ['cockpit  TAEM  M0.9 11 km', { V: 265, alt: 11000 }, { cabin: true }],
  ['cockpit  final 155 m/s 3 km, SB 60', { V: 155, alt: 3000, surf: { speedbrake: 60 } }, { cabin: true }],
  ['cockpit  final 135 m/s 100 m, gear', { V: 135, alt: 100, gear: 1, surf: { speedbrake: 40 } }, { cabin: true }],
  ['chase 75 m behind, 155 m/s 3 km', { V: 155, alt: 3000 }, 'chase'],
  ['flyover 135 m/s, 100 m overhead, gear', { V: 135, alt: 100, gear: 1, surf: { speedbrake: 40 } }, lis([0, 1.7, 0])],
  ['sideline 135 m/s, 100 m alt, 500 m', { V: 135, alt: 100, gear: 1, surf: { speedbrake: 40 } }, lis([0, 1.7, 500])],
  ['tower 3 km sideline, 155 m/s 600 m', { V: 155, alt: 600 }, lis([0, 30, 3000])],
];
for (const [name, f, L] of cases) {
  const ac = flight(f);
  let o;
  if (L === 'chase') o = ac.listen({ ...lis([-75, f.alt + 15, 0]), vel: [f.V, 0, 0] });
  else o = ac.listen(L.cabin ? L : L);
  console.log(`  ${name.padEnd(40)} ${lev(o).padEnd(22)} ${spec(o)}`);
}

console.log(fails ? `\n${fails} check(s) FAILED` : '\nall checks passed');
process.exit(fails ? 1 : 0);
