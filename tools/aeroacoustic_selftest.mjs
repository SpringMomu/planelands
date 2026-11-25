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
  const out = ac.listen(lis([0, 1.7, 0]));
  // analytic: |(0,h) - (V te, h)| = c (0 - te)  ->  te = -h' / sqrt(c^2 - V^2)
  const h = 300 - 1.7, te = -h / Math.sqrt(c * c - 150 * 150);
  check('retarded time (subsonic flyover)', Math.abs(out.delay + te) < 0.02, `delay ${out.delay.toFixed(3)} s vs ${(-te).toFixed(3)} s, ${out.spl.toFixed(1)} dB`);
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

// ---- the spectral synthesiser (AudioWorklet / ScriptProcessor) reproduces a requested spectrum
{
  const { SpectralNoise } = await import('../web/js/world/aero_synth.js');
  const run = (msg, seconds) => {
    const p = new SpectralNoise(48000);
    p.message({ bands: Float64Array.from(A.FC) });
    p.message(msg);
    const n = Math.floor(seconds * 48000 / 128) * 128, L = new Float32Array(n), R = new Float32Array(n);
    for (let o = 0; o < n; o += 128) p.render(L.subarray(o, o + 128), R.subarray(o, o + 128));
    return { L: L.subarray(8192), R: R.subarray(8192) };          // skip the start-up
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
        const f = k * 48000 / n, b = Math.round(10 * Math.log10(f / A.FC[0]));
        if (b >= 0 && b < A.NB) out[b] += 2 * (re[k] * re[k] + im[k] * im[k]) / (n * wss);   // one-sided power per bin
      }
      segs++;
    }
    return out.map((v) => v / segs);
  };
  // a smooth spectrum (cockpit, final approach) as direct + diffuse parts
  const ac = flight({ V: 135, alt: 100, gear: 1, surf: { speedbrake: 40 } });
  const o = ac.listen({ cabin: true });
  const g = 0.01;
  const tgt = Array.from(o.dir, (v, i) => (v + o.dif[i]) * g * g);
  const { L, R } = run({ dir: Float32Array.from(o.dir), wl: Float32Array.from(o.wl), dif: Float32Array.from(o.dif), gain: g }, 12);
  const got = bandsOf(L);
  let worst = 0, wb = 0;
  for (let i = 4; i < 29; i++) { const e = Math.abs(10 * Math.log10(got[i] / tgt[i])); if (e > worst) { worst = e; wb = i; } }
  const ms = L.reduce((s, v) => s + v * v, 0) / L.length, msT = sum(tgt);
  check('synth: band levels follow the model (50 Hz - 12.5 kHz)', worst < 1.5, `worst ${worst.toFixed(2)} dB at ${A.FC[wb].toFixed(0)} Hz; overall ${(10 * Math.log10(ms / msT)).toFixed(2)} dB`);
  const corr = (x, y) => { let a = 0, b = 0, c = 0; for (let i = 0; i < x.length; i++) { a += x[i] * y[i]; b += x[i] * x[i]; c += y[i] * y[i]; } return a / Math.sqrt(b * c); };
  const z = new Float32Array(A.NB), w5 = new Float32Array(A.NB).fill(0.5), one = new Float32Array(A.NB).fill(1e-4);
  const d = run({ dir: one, wl: w5, dif: z, gain: 1 }, 3), u = run({ dir: z, wl: w5, dif: one, gain: 1 }, 3);
  check('synth: direct sound correlated, diffuse sound decorrelated', corr(d.L, d.R) > 0.99 && Math.abs(corr(u.L, u.R)) < 0.05,
    `direct ${corr(d.L, d.R).toFixed(3)}, diffuse ${corr(u.L, u.R).toFixed(3)}`);
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
