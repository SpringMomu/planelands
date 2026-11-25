// Aeroacoustics of the gliding orbiter: the "wind" heard in the flight deck and around the vehicle,
// computed from the flow, not from recordings.  Pure functions / classes, no WebAudio, no three.js
// (tools/aeroacoustic_selftest.mjs runs them in node).  All sound is returned as mean-square
// pressure per 1/3-octave band (Pa^2) at the listener; web/js/world/aero_audio.js renders it.
//
// An unpowered vehicle makes sound only through its flow, and there are two ways to hear it:
//
// 1. In the flight deck: the turbulent boundary layer on the forward fuselage
//    * wall-pressure spectrum: Goody (2004, AIAA J 42(9)) shape, with the boundary layer of a flat
//      plate at the running length from the nose (Schultz-Grunow local skin friction, Eckert reference
//      temperature for compressibility, adiabatic wall, the same tile-roughness factor as sim/aero.py);
//      level in inner variables, Farabee & Casarella (1991): (p_rms / tau_w)^2 = 6.5 + 1.86 ln(Re_tau / 333)
//      (Lowson's 1968 flight correlation p_rms / q = 0.006 / (1 + 0.14 M^2) agrees within ~4 dB:
//      tools/aeroacoustic_selftest.mjs)
//    * transmission into the crew module: plane-wave transfer matrices of the real layers (oblique
//      incidence, diffuse field average 0-78 deg): outer skin + tiles, the gap between the forward
//      fuselage and the crew module (vented: at the OUTSIDE pressure, partly filled with insulation -
//      Delany-Bazley), the crew-module wall; and the windows: thermal pane, vented gap, pressure
//      panes.  Plates: mass, bending stiffness (coincidence), the panel's fundamental resonance, loss
//      factor.  Thin air outside and in the vented gap: different impedances on the two sides, softer
//      air spring between the walls (lower mass-air-mass resonance): high up the airborne path through
//      the gap almost vanishes and what remains is structure-borne, through the crew module's attach
//      points (Sharp 1978 point-bridge model: solid mass law of both leaves + 20 lg(e f_c) +
//      20 lg(m1 / (m1 + m2)) - 45 dB)
//    * cabin: reverberant field 4 W / R (room constant from an absorption that rises with frequency)
//      plus the direct field of the windscreen next to the eye
//    * the airframe noise sources below (landing gear right under the cabin, elevons, speed brake far
//      aft) reach the cabin through the same hull; in supersonic flight nothing from behind arrives
//
// 2. Outside (chase, tower, runway, fly-by cameras): airframe noise radiated to the far field
//    * Fink (1977, FAA-RD-77-29) airframe noise method in its ANOPP form (Zorumski 1982):
//      p^2 = rho^2 c^4 Pi* D(theta,phi) F(S) / (4 pi r^2 (1 - M cos theta)^4), Pi* the normalised
//      acoustic power, F(S) the 1/3-octave spectrum (each one sums to 1 over the bands), S a
//      Strouhal number formed with the source-frame frequency.  Components: wing, fin and body-flap
//      trailing edges (Pi* = 4.464e-5 M^5 delta b, turbulent boundary layer thickness at the edge),
//      deflected elevons, body flap and the two split-rudder / speed-brake panels as trailing-edge
//      flaps (2.787e-4 M^6 S_f sin^2 delta, dipole normal to the deflected surface), landing gear
//      wheels (4.349e-4 M^6 N d^2) and struts (2.753e-4 M^6 d l), scaled by the gear extension
//    * propagation: retarded time solved on the recorded trajectory (the sound heard now left the
//      vehicle where it was then: seconds of delay at the tower, the roar arrives after it has passed),
//      in air moving with the wind; Doppler shift of moving source and moving listener; convective
//      amplification (Ffowcs Williams' finite-coherence form, bounded on the Mach cone); supersonic:
//      two emission points or none (silence outside the Mach cone);
//      spherical spreading; ISO 9613-1 air absorption integrated along the slant path (humidity falls
//      off with height, thin air absorbs more); ground reflection (Delany-Bazley ground impedance,
//      plane-wave reflection coefficient, interference averaged over each band); acoustic shadow of
//      upward refraction; impedance change rho c from source to listener
//
// Known simplifications: the turbulent boundary layer drives the hull as an equivalent diffuse
// acoustic field (its convected wavenumber content is not resolved), freestream conditions are used at
// the windows (no local acceleration over the nose, no shock layer above Mach 1), the Fink correlations
// come from conventional subsonic transports (their power / Strouhal scaling is held at M 0.8 above
// that), the sonic boom is not modelled.

// ------------------------------------------------------------------------------------------ bands
/** 1/3-octave bands 20 Hz - 20 kHz (base-10 centres, IEC 61260) */
export const NB = 31;
export const FC = new Float64Array(NB), F_LO = new Float64Array(NB), F_HI = new Float64Array(NB), BW = new Float64Array(NB);
for (let i = 0; i < NB; i++) {
  FC[i] = 1000 * Math.pow(10, (i - 17) / 10);
  F_LO[i] = FC[i] * Math.pow(10, -0.05);
  F_HI[i] = FC[i] * Math.pow(10, 0.05);
  BW[i] = F_HI[i] - F_LO[i];
}
const SUBF = [Math.pow(10, -1 / 30), 1, Math.pow(10, 1 / 30)];     // three frequencies across a band

// ------------------------------------------------------------------------------------- atmosphere
const R_AIR = 287.05287, GAM = 1.4, G0 = 9.80665, RE = 6356766;
const LAY = [[0, 288.15, -0.0065], [11000, 216.65, 0], [20000, 216.65, 0.001], [32000, 228.65, 0.0028],
  [47000, 270.65, 0], [51000, 270.65, -0.0028], [71000, 214.65, -0.002], [84852, 186.946, 0]];
const PB = [101325];
for (let i = 0; i < LAY.length - 1; i++) {
  const [hb, tb, lr] = LAY[i], dh = LAY[i + 1][0] - hb;
  PB.push(lr === 0 ? PB[i] * Math.exp(-G0 * dh / (R_AIR * tb)) : PB[i] * Math.pow((tb + lr * dh) / tb, -G0 / (R_AIR * lr)));
}
export const sutherland = (T) => 1.458e-6 * T * Math.sqrt(T) / (T + 110.4);

/** US Standard Atmosphere 1976 (as sim/atmosphere.py) at geometric altitude h (m) */
export function atmosphere(h, out = {}) {
  h = Math.min(86000, Math.max(-1000, h));
  const H = RE * h / (RE + h);
  let i = LAY.length - 1;
  while (i > 0 && H < LAY[i][0]) i--;
  const [hb, tb, lr] = LAY[i], dh = H - hb, T = tb + lr * dh;
  const p = lr === 0 ? PB[i] * Math.exp(-G0 * dh / (R_AIR * tb)) : PB[i] * Math.pow(T / tb, -G0 / (R_AIR * lr));
  out.T = T; out.p = p; out.rho = p / (R_AIR * T); out.c = Math.sqrt(GAM * R_AIR * T); out.mu = sutherland(T);
  return out;
}

// --------------------------------------------------------------------- air absorption (ISO 9613-1)
/** saturation vapour pressure over the reference pressure (ISO 9613-1 eq. B.1) */
export const psatRatio = (T) => Math.pow(10, -6.8346 * Math.pow(273.16 / T, 1.261) + 4.6151);
/** molar concentration of water vapour (%) from relative humidity (%) */
export const molarH2O = (rh, T, p) => rh * psatRatio(T) / (p / 101325);

/** pure-tone attenuation coefficient (dB/m), ISO 9613-1 eqs. 3-5. f Hz, T K, p Pa, h molar % */
export function isoAlpha(f, T, p, h) {
  const pr = p / 101325, tr = T / 293.15, f2 = f * f;
  const frO = pr * (24 + 4.04e4 * h * (0.02 + h) / (0.391 + h));
  const frN = pr / Math.sqrt(tr) * (9 + 280 * h * Math.exp(-4.170 * (Math.pow(tr, -1 / 3) - 1)));
  return 8.686 * f2 * (1.84e-11 / pr * Math.sqrt(tr) + Math.pow(tr, -2.5) *
    (0.01275 * Math.exp(-2239.1 / T) / (frO + f2 / frO) + 0.1068 * Math.exp(-3352.0 / T) / (frN + f2 / frN)));
}

/** water vapour at height z: the ground value (relative humidity rh0 at zg) falls off with a ~2.2 km
 * scale height (precipitable-water profile), never above saturation */
function vapourAt(z, zg, rh0, a) {
  const g = atmosphere(zg), h0 = molarH2O(rh0, g.T, g.p);
  return Math.min(h0 * Math.exp(-Math.max(0, z - zg) / 2200), molarH2O(100, a.T, a.p));
}

/** band-averaged transmission (energy) of air absorption along a straight path z1 -> z2 of length r */
export function absorptionBands(z1, z2, r, zg, rh0, out = new Float64Array(NB)) {
  const a = {};
  const T = [], P = [], H = [];
  for (let k = 0; k < 4; k++) {
    const z = z1 + (z2 - z1) * (k + 0.5) / 4;
    atmosphere(z, a); T.push(a.T); P.push(a.p); H.push(vapourAt(z, zg, rh0, a));
  }
  for (let i = 0; i < NB; i++) {
    let e = 0;
    for (const s of SUBF) {
      const f = FC[i] * s;
      let al = 0;
      for (let k = 0; k < 4; k++) al += isoAlpha(f, T[k], P[k], H[k]) / 4;
      e += Math.pow(10, -al * r / 10) / 3;
    }
    out[i] = e;
  }
  return out;
}

// ------------------------------------------------------------------ turbulent boundary layer (TBL)
const goody = (w, c3) => 3.0 * w * w / (Math.pow(Math.pow(w, 0.75) + 0.5, 3.7) + Math.pow(c3 * w, 7));

/** compressible flat-plate turbulent boundary layer at running length x (m), freestream U (m/s) and
 * atmosphere atm.  rough: tile roughness factor on c_f (as sim/aero.py) */
export function boundaryLayer(U, atm, x, rough = 1.35) {
  const M = U / atm.c, Te = atm.T;
  const Taw = Te * (1 + 0.89 * 0.2 * M * M);                            // adiabatic wall, recovery 0.89
  const Ts = Te * (1 + 0.032 * M * M + 0.58 * (Taw / Te - 1));          // Eckert reference temperature
  const rhoS = atm.rho * Te / Ts, muS = sutherland(Ts);
  const Rex = Math.max(rhoS * U * x / muS, 1e5);
  const cf = 0.370 / Math.pow(Math.log10(Rex), 2.584) * rough;         // Schultz-Grunow local c_f
  const tauW = cf * 0.5 * rhoS * U * U;
  const delta = 0.37 * x * Math.pow(Rex, -0.2);
  const rhoW = atm.rho * Te / Taw, nuW = sutherland(Taw) / rhoW;
  const uTau = Math.sqrt(tauW / rhoW);
  const RT = uTau * uTau * delta / (U * nuW);                          // outer / inner time scale ratio
  const q = 0.5 * atm.rho * U * U;
  const ReTau = uTau * delta / nuW;
  const prms2 = tauW * tauW * (6.5 + 1.86 * Math.log(Math.max(ReTau, 333) / 333));   // Farabee & Casarella 1991
  const lowson = Math.pow(0.006 * q / (1 + 0.14 * M * M), 2);                       // Lowson 1968 (check)
  return { M, q, tauW, delta, uTau, RT, ReTau, prms2, lowson, cf, Rex };
}

/** TBL wall pressure per 1/3-octave band (Pa^2): Goody spectral shape, Farabee-Casarella level */
export function tblBands(U, atm, x, out = new Float64Array(NB), rough = 1.35) {
  out.fill(0);
  if (U < 3) return out;
  const bl = boundaryLayer(U, atm, x, rough);
  const c3 = 1.1 * Math.pow(bl.RT, -0.57);
  // total: integral of the shape over ln(w), w = omega delta / U
  let tot = 0;
  const n = 240, l0 = Math.log(1e-4), l1 = Math.log(1e4), dl = (l1 - l0) / n;
  for (let k = 0; k <= n; k++) { const w = Math.exp(l0 + k * dl); tot += goody(w, c3) * w * (k === 0 || k === n ? 0.5 : 1); }
  tot *= dl;
  const s = 2 * Math.PI * bl.delta / U;
  for (let i = 0; i < NB; i++) {
    // Simpson over ln(w) across the band
    const a = Math.log(F_LO[i] * s), b = Math.log(F_HI[i] * s), m = 6, h = (b - a) / m;
    let acc = 0;
    for (let k = 0; k <= m; k++) { const w = Math.exp(a + k * h); acc += goody(w, c3) * w * (k === 0 || k === m ? 1 : k % 2 ? 4 : 2); }
    out[i] = bl.prms2 * acc * h / 3 / tot;
  }
  return out;
}

// ----------------------------------------------------------------- hull transmission (transfer matrices)
// complex numbers as [re, im]; time convention exp(j w t)
const cm = (a, b) => [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
const cd = (a, b) => { const d = b[0] * b[0] + b[1] * b[1]; return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d]; };
const ca = (a, b) => [a[0] + b[0], a[1] + b[1]];
function csqrtDecay(z) {             // the root with Im <= 0 (decaying / outgoing for exp(j w t - j k z))
  const r = Math.hypot(z[0], z[1]);
  let re = Math.sqrt(Math.max(0, (r + z[0]) / 2)), im = Math.sqrt(Math.max(0, (r - z[0]) / 2));
  if (z[1] < 0) im = -im;
  if (im > 0) { re = -re; im = -im; }
  return [re, im];
}
const ccos = (z) => [Math.cos(z[0]) * Math.cosh(z[1]), -Math.sin(z[0]) * Math.sinh(z[1])];
const csin = (z) => [Math.sin(z[0]) * Math.cosh(z[1]), Math.cos(z[0]) * Math.sinh(z[1])];
const m2 = (A, B) => [ca(cm(A[0], B[0]), cm(A[1], B[2])), ca(cm(A[0], B[1]), cm(A[1], B[3])),
  ca(cm(A[2], B[0]), cm(A[3], B[2])), ca(cm(A[2], B[1]), cm(A[3], B[3]))];

/** Layers (outside -> cabin).  plate: { m kg/m^2, D N m (bending), fr Hz (panel fundamental), eta };
 *  gap: { d m, air: 'out' (vented) | 'cabin' (sealed), sigma: flow resistivity Pa s/m^2 (0: plain air) } */
export const HULL = {
  // forward fuselage skin (2024 Al, stringer-stiffened, ~2 mm) + RSI tiles / felt pad: ~14 kg/m^2;
  // vented gap to the crew module (insulation blankets on the outer structure), 2219 Al crew-module
  // wall (integrally machined, ~6 mm equivalent): ~17 kg/m^2
  wall: [
    { m: 14, D: 500, fr: 150, eta: 0.08 },
    { d: 0.08, air: 'out', sigma: 15000 },
    { d: 0.22, air: 'out', sigma: 0 },
    { m: 17, D: 1400, fr: 110, eta: 0.04 },
  ],
  // flight-deck windows (approximate): outer fused-silica thermal pane 15.9 mm, vented gap, centre
  // pressure pane 33 mm and inner pane 15.9 mm (alumino-silicate) with a sealed gap at cabin pressure
  window: [
    { m: 35, D: 24800, fr: 160, eta: 0.02 },
    { d: 0.05, air: 'out', sigma: 0 },
    { m: 81, D: 222000, fr: 340, eta: 0.02 },
    { d: 0.02, air: 'cabin', sigma: 0 },
    { m: 39, D: 24800, fr: 200, eta: 0.02 },
  ],
  // structure-borne bypass of the gap (Sharp point bridges): outer / inner leaf, the radiating inner
  // leaf's bending stiffness, spacing of the connections (crew module: a few attach points + the
  // window frames, ~2.5 m)
  wallBridge: { m1: 14, m2: 17, D2: 1400, fr: 110, eta: 0.04, e: 2.5 },
  windowBridge: { m1: 35, m2: 120, D2: 246800, fr: 340, eta: 0.02, e: 2.5 },
};
/** crew cabin acoustics: hull areas exposed to the flow (m^2), interior surface, eye to forward windows */
export const CABIN = { Swall: 26, Swin: 3.5, Sfwd: 2.7, rEye: 0.9, Sroom: 40, rho: 1.18, c: 345.5 };
export const cabinAlpha = (f) => 0.08 + 0.27 * f * f / (f * f + 400 * 400);

/** diffuse-field (0-78 deg) power transmission coefficient per band of a layered partition.
 * ext / gap / cab: { rho, c } of the outside air, the vented gap air and the cabin.  thetaDeg: a
 * single incidence angle instead of the diffuse average (tests) */
export function transmissionBands(layers, ext, gap, cab, out = new Float64Array(NB), thetaDeg = null) {
  const NA = thetaDeg === null ? 14 : 1, thMax = 78 * Math.PI / 180;
  let wsum = 0;
  const ws = [], ths = [];
  for (let a = 0; a < NA; a++) {
    const th = thetaDeg === null ? (a + 0.5) / NA * thMax : thetaDeg * Math.PI / 180;
    ths.push(th); ws.push(thetaDeg === null ? Math.sin(th) * Math.cos(th) : 1); wsum += ws[a];
  }
  for (let i = 0; i < NB; i++) {
    let acc = 0;
    for (const sf of SUBF) {
      const f = FC[i] * sf, w = 2 * Math.PI * f;
      for (let a = 0; a < NA; a++) {
        const th = ths[a], kt = w / ext.c * Math.sin(th);                // trace wavenumber (Snell)
        let M = [[1, 0], [0, 0], [0, 0], [1, 0]];
        for (const L of layers) {
          let T;
          if (L.m !== undefined) {
            // thin plate: Z = j w m - j (D kt^4 + s)(1 + j eta) / w  (bending + panel-fundamental stiffness)
            const kt4 = kt * kt * kt * kt, s = L.m * Math.pow(2 * Math.PI * L.fr, 2);
            const st = (L.D * kt4 + s) / w;
            T = [[1, 0], [L.eta * st, w * L.m - st], [0, 0], [1, 0]];
          } else {
            const air = L.air === 'cabin' ? cab : gap;
            let Zc = [air.rho * air.c, 0], k = [w / air.c, 0];
            if (L.sigma > 0) {                                            // Delany-Bazley porous layer
              const X = Math.min(1, Math.max(0.01, air.rho * f / L.sigma));
              Zc = [air.rho * air.c * (1 + 0.0571 * Math.pow(X, -0.754)), -air.rho * air.c * 0.087 * Math.pow(X, -0.732)];
              k = [w / air.c * (1 + 0.0978 * Math.pow(X, -0.700)), -w / air.c * 0.189 * Math.pow(X, -0.595)];
            }
            const kz = csqrtDecay(ca(cm(k, k), [-kt * kt, 0]));
            const Zz = cd(cm(Zc, k), kz);
            const arg = [kz[0] * L.d, kz[1] * L.d];
            const C = ccos(arg), S = csin(arg);
            T = [C, cm([0, 1], cm(Zz, S)), cm([0, 1], cd(S, Zz)), C];
          }
          M = m2(M, T);
        }
        const kc2 = Math.pow(w / cab.c, 2) - kt * kt;
        if (kc2 <= 0) continue;                                           // evanescent in the cabin
        const Zt = [cab.rho * w / Math.sqrt(kc2), 0];                      // cabin impedance (normal)
        const Zo = [ext.rho * ext.c / Math.cos(th), 0];
        const p1 = ca(M[0], cd(M[1], Zt)), v1 = ca(M[2], cd(M[3], Zt));
        const pi2 = ca(p1, cm(Zo, v1));                                    // 2 p_i / p_t
        const t2 = 4 / (pi2[0] * pi2[0] + pi2[1] * pi2[1]);
        acc += t2 * Zo[0] / Zt[0] * ws[a];                                 // intensity ratio
      }
    }
    out[i] = acc / (3 * wsum);
  }
  return out;
}

// --------------------------------------------------------------- airframe noise (Fink / ANOPP)
const F_TE = (S) => { const x = 10 * S; return 0.613 * x * x * x * x * Math.pow(Math.pow(x, 1.5) + 0.5, -4); };
const F_FLAP = (S) => (S < 2 ? 0.048 * S : S < 20 ? 0.1406 * Math.pow(S, -0.55) : 216.49 * Math.pow(S, -3));
const F_WHEEL = (S) => 13.59 * S * S * Math.pow(12.5 + S * S, -2.25);
const F_STRUT = (S) => 5.32 * S * S / (30 + Math.pow(S, 8));
export const FINK_SPECTRA = { F_TE, F_FLAP, F_WHEEL, F_STRUT };

/** the noise components of the orbiter from shared/orbiter_geometry.json (body FRD, metres) */
export function orbiterComponents(g) {
  const C = [];
  const w = g.wing, ref = g.reference;
  const dz = (y) => w.z_root - (y - w.y_root) * Math.tan(w.dihedral_deg * Math.PI / 180);
  const teX = (y) => { const [[x0, y0], [x1, y1]] = w.te; return x0 + (x1 - x0) * (y - y0) / (y1 - y0); };
  // wing trailing edge (each half radiates half of the wing's power)
  for (const s of [1, -1]) {
    const y = (w.y_root + w.tip_y) / 2;
    C.push({ kind: 'te', name: 'wing', pos: [teX(y), s * y, dz(y)], plane: 'z', S: ref.S, b: ref.b, share: 0.5 });
  }
  // vertical tail
  const vt = g.vtail, cr = vt.root.te_x - vt.root.le_x, ct = vt.tip.te_x - vt.tip.le_x, hv = Math.abs(vt.root.z - vt.tip.z);
  const Sv = Math.abs(cr + ct) / 2 * hv, zv = (vt.root.z + vt.tip.z) / 2;
  C.push({ kind: 'te', name: 'fin', pos: [(vt.root.te_x + vt.tip.te_x) / 2, 0, zv], plane: 'y', S: Sv, b: hv, share: 1 });
  // body flap: a small "horizontal tail" trailing edge, and a trailing-edge flap when deflected
  const bf = g.bodyflap, Sbf = bf.chord * 2 * bf.half_span;
  C.push({ kind: 'te', name: 'bodyflap', pos: [bf.hinge_x - bf.chord, 0, bf.z], plane: 'z', S: Sbf, b: 2 * bf.half_span, share: 1 });
  C.push({ kind: 'flap', name: 'bodyflap', pos: [bf.hinge_x - bf.chord / 2, 0, bf.z], Sf: Sbf, bf: 2 * bf.half_span, normal: 'down', surf: 'bodyflap' });
  // elevons (plain trailing-edge flaps); right-wing names in the geometry, left ones mirrored
  for (const e of w.elevons) {
    const y = (e.y0 + e.y1) / 2, Sf = (e.y1 - e.y0) * e.chord;
    for (const s of [1, -1]) {
      C.push({ kind: 'flap', name: e.name, pos: [teX(y) + e.chord / 2, s * y, dz(y)], Sf, bf: e.y1 - e.y0, normal: 'down',
        surf: s > 0 ? e.name : e.name.replace('_r', '_l') });
    }
  }
  // split rudder / speed brake: two panels opening outward (left = rudder + sb/2, right = rudder - sb/2)
  const rd = vt.rudder, zr = (rd.z0 + rd.z1) / 2, hr = Math.abs(rd.z1 - rd.z0);
  const fz = (zr - vt.root.z) / (vt.tip.z - vt.root.z);
  const chordAt = Math.abs(cr + (ct - cr) * fz), teAt = vt.root.te_x + (vt.tip.te_x - vt.root.te_x) * fz;
  const Sp = rd.chord_frac * chordAt * hr, xp = teAt + rd.chord_frac * chordAt / 2;
  C.push({ kind: 'flap', name: 'sb_left', pos: [xp, 0, zr], Sf: Sp, bf: hr, normal: 'left', surf: 'sbL' });
  C.push({ kind: 'flap', name: 'sb_right', pos: [xp, 0, zr], Sf: Sp, bf: hr, normal: 'right', surf: 'sbR' });
  // landing gear: two wheels per leg; strut from the lower mould line (z ~ 2.3) to the axle
  for (const k of ['nose', 'main_r', 'main_l']) {
    const G = g.gear[k], d = 2 * G.wheel_radius, zAx = G.z - G.wheel_radius;
    C.push({ kind: 'wheel', name: k, pos: [G.x, G.y, zAx], d, N: 2 });
    C.push({ kind: 'strut', name: k, pos: [G.x, G.y, (zAx + 2.3) / 2], d, l: Math.max(0.5, zAx - 2.3) });
  }
  return C;
}

/** deflection (rad) of a flap component from the sim's surface angles (deg) */
function flapDelta(c, surf) {
  if (!surf) return 0;
  const D = Math.PI / 180;
  switch (c.surf) {
    case 'bodyflap': return (surf.bodyflap || 0) * D;
    case 'sbL': return ((surf.rudder || 0) + (surf.speedbrake || 0) / 2) * D;   // left panel, outward (left)
    case 'sbR': return ((surf.speedbrake || 0) / 2 - (surf.rudder || 0)) * D;  // right panel, outward (right)
    default: return (surf[c.surf] || 0) * D;
  }
}

/**
 * Mean-square pressure per band (Pa^2, added into out) of one component heard at distance r.
 * src: { M, rho, c, mu, V } at the source (air-relative), dirs: unit vectors u (flight direction
 * through the air), dn (body down, normal to u), rt (body right, normal to u), n (from the emission
 * point to the listener).  dopL: listener Doppler factor (1 - n.V_L/c).  amp: extra factor per band
 * (absorption, ground) or null.
 */
export const FINK_M_MAX = 0.8;            // validity of the correlations (subsonic transports)

export function finkComponent(c, src, dirs, r, dopL, gearPos, surf, out, amp = null) {
  const { M, rho, c: a, mu } = src;
  if (M < 0.01) return;
  const { u, dn, rt, n } = dirs;
  // acoustic power and Strouhal scaling with the Mach number held at the correlations' limit: beyond
  // transonic speed the external sound is the shock system (sonic boom, not modelled), not edge noise
  // growing as M^5..6; the Doppler / convection factor keeps the true Mach number
  const Mf = Math.min(M, FINK_M_MAX), V = Mf * a;
  const ct = n[0] * u[0] + n[1] * u[1] + n[2] * u[2];
  const nd = n[0] * dn[0] + n[1] * dn[1] + n[2] * dn[2], nr = n[0] * rt[0] + n[1] * rt[1] + n[2] * rt[2];
  const st2 = Math.max(0, 1 - ct * ct);
  // Doppler / convection factor (1 - M cos theta), regularised by the finite coherence time of the
  // turbulent sources (Ffowcs Williams 1963: sqrt((1 - M cos th)^2 + (alpha M)^2), alpha ~ 0.3): bounded
  // on the Mach cone of a supersonic vehicle instead of the singular point-source value
  const dop = Math.sqrt((1 - M * ct) * (1 - M * ct) + 0.09 * M * M);
  const a5 = Mf * Mf * Mf * Mf * Mf;
  let P, D, L, F;                                                        // P: Pi* b^2 (m^2)
  if (c.kind === 'te') {
    const cbar = c.S / c.b, delta = 0.37 * cbar * Math.pow(Math.max(1e4, rho * V * cbar / mu), -0.2);
    P = 4.464e-5 * a5 * delta * c.b * c.share;
    const cphi2 = st2 > 1e-6 ? (c.plane === 'z' ? nd * nd : nr * nr) / st2 : 0.5;
    D = 4 * cphi2 * (1 + ct) / 2;                                        // 4 cos^2 phi cos^2 (theta / 2)
    L = delta; F = F_TE;
  } else if (c.kind === 'flap') {
    const del = flapDelta(c, surf), s2 = Math.sin(del) ** 2;
    if (s2 < 1e-8) return;
    P = 2.787e-4 * a5 * Mf * c.Sf * s2;
    const nn = c.normal === 'down' ? nd : c.normal === 'right' ? nr : -nr;
    const proj = Math.sin(del) * ct + Math.cos(del) * nn;               // 3 (sin d cos th + cos d sin th cos phi)^2
    D = 3 * proj * proj;
    L = c.Sf / c.bf; F = F_FLAP;
  } else {
    if (!(gearPos > 0.01)) return;
    if (c.kind === 'wheel') { P = 4.349e-4 * a5 * Mf * c.N * c.d * c.d * gearPos; D = 1.5 * st2; F = F_WHEEL; }
    else { P = 2.753e-4 * a5 * Mf * c.d * c.l * gearPos; D = 3 * nr * nr; F = F_STRUT; }      // 3 sin^2 th sin^2 phi
    L = c.d;
  }
  if (!(D > 0)) return;
  const k0 = rho * rho * a * a * a * a * P * D / (4 * Math.PI * r * r * dop * dop * dop * dop);
  const sf = L * dop / (Mf * a);                                          // S = f L (1 - M cos th) / (M c)
  for (let i = 0; i < NB; i++) {
    const v = k0 * F(FC[i] / dopL * sf);
    out[i] += amp ? v * amp[i] : v;
  }
}

// ------------------------------------------------------------------ ground reflection (band average)
/** |direct + reflected|^2 / |direct|^2 per band for source / listener heights hs, hl over the
 * horizontal distance dh.  Plane-wave reflection with Delany-Bazley ground impedance (sigma Pa s/m^2,
 * ~3e5 grassland) */
export function groundBands(hs, hl, dh, c, out = new Float64Array(NB), sigma = 3e5) {
  hs = Math.max(hs, 0.5); hl = Math.max(hl, 0.5);
  const r1 = Math.hypot(dh, hs - hl), r2 = Math.hypot(dh, hs + hl), dl = r2 - r1;
  const sinPsi = (hs + hl) / r2, g = r1 / r2;
  for (let i = 0; i < NB; i++) {
    let acc = 0;
    const m = 7;
    for (let k = 0; k < m; k++) {
      const f = F_LO[i] * Math.pow(F_HI[i] / F_LO[i], (k + 0.5) / m);
      const X = Math.max(1e-3, 1.2 * f / sigma);
      const zr = 1 + 0.0571 * Math.pow(X, -0.754), zi = -0.087 * Math.pow(X, -0.732);
      const nr = zr * sinPsi - 1, ni = zi * sinPsi, dr = zr * sinPsi + 1, di = zi * sinPsi;
      const dd = dr * dr + di * di;
      const Rr = (nr * dr + ni * di) / dd, Ri = (ni * dr - nr * di) / dd;
      const R = Math.hypot(Rr, Ri), ph = Math.atan2(Ri, Rr) - 2 * Math.PI * f * dl / c;
      acc += 1 + R * R * g * g + 2 * R * g * Math.cos(ph);
    }
    out[i] = acc / m;
  }
  return out;
}

// ------------------------------------------------------------------------------- the listener model
const v3 = () => [0, 0, 0];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a, b, o = v3()) => { o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2]; return o; };
function orthoUnit(v, u, o = v3()) {        // v without its component along unit u, normalised
  const k = dot(v, u);
  o[0] = v[0] - k * u[0]; o[1] = v[1] - k * u[1]; o[2] = v[2] - k * u[2];
  const l = Math.hypot(o[0], o[1], o[2]) || 1;
  o[0] /= l; o[1] /= l; o[2] /= l;
  return o;
}

/**
 * Vehicle samples (pushed every frame, in sim time) and the sound at a listener.
 * Frame: any right-handed world frame with y up (three.js world); positions in metres.
 */
export class AeroAcoustics {
  constructor(geometry) {
    this.comps = orbiterComponents(geometry);
    this.eye = geometry.cockpit_eye || [14.4, -0.55, -2.3];
    this.nose = geometry.fuselage.nose_x;
    this.winX = (geometry.fuselage.windows_x[0] + geometry.fuselage.windows_x[1]) / 2;
    this.hist = [];
    this.histMax = 120;          // s of trajectory kept (sound from ~40 km)
    this._tau = { key: '', wall: new Float64Array(NB), win: new Float64Array(NB) };
    this._b = new Float64Array(NB); this._b2 = new Float64Array(NB); this._abs = new Float64Array(NB); this._gnd = new Float64Array(NB);
    this._amp = new Float64Array(NB); this._ff = new Float64Array(NB); this._pr = new Float64Array(NB);
    this.out = { dir: new Float64Array(NB), lat: new Float64Array(NB), dif: new Float64Array(NB), spl: 0, delay: 0, arrivals: 0 };
  }

  reset() { this.hist.length = 0; }

  /** s: { t, pos, vel (inertial), wind, fwd, up, right (unit, body x / -z / y), alt (m MSL), agl,
   *  surf (deg, as the sim's state message), gear (0..1) } - arrays [x,y,z] in the world frame */
  push(s) {
    const H = this.hist, last = H[H.length - 1];
    if (last && s.t < last.t - 1e-6) H.length = 0;                       // new episode / time went back
    else if (last && (s.t - last.t < 1e-4 || (H.length > 1 && s.t - H[H.length - 2].t < 0.03))) {
      H[H.length - 1] = s;                                                // ~30 ms spacing, newest always current
      return;
    }
    H.push(s);
    let k = 0;
    while (H.length - k > 2 && H[H.length - 1].t - H[k + 1].t > this.histMax) k++;
    if (k) H.splice(0, k);
  }

  /** vehicle state at time t (linear interpolation in the history) */
  _at(t) {
    const H = this.hist;
    let j = H.length - 1;
    while (j > 0 && H[j - 1].t > t) j--;
    const b = H[j], a = H[Math.max(0, j - 1)];
    if (a === b || b.t - a.t < 1e-6) return b;
    const f = Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))), g = 1 - f;
    const L = (p, q) => [p[0] * g + q[0] * f, p[1] * g + q[1] * f, p[2] * g + q[2] * f];
    return { t, pos: L(a.pos, b.pos), vel: L(a.vel, b.vel), wind: L(a.wind, b.wind), fwd: L(a.fwd, b.fwd), up: L(a.up, b.up),
      right: L(a.right, b.right), alt: a.alt * g + b.alt * f, agl: a.agl * g + b.agl * f, surf: f < 0.5 ? a.surf : b.surf,
      gear: a.gear * g + b.gear * f };
  }

  /** emission times of the sound reaching Lp at time t: roots of c (t - te) = |Lp - S(te) - W (t - te)|.
   * One root for a subsonic vehicle, two or none for a supersonic one (inside / outside its Mach cone) */
  _emissions(t, Lp, c) {
    const H = this.hist, roots = [];
    const g = (s) => {
      const dt = t - s.t;
      return c * dt - Math.hypot(Lp[0] - s.pos[0] - s.wind[0] * dt, Lp[1] - s.pos[1] - s.wind[1] * dt, Lp[2] - s.pos[2] - s.wind[2] * dt);
    };
    let gb = g(H[H.length - 1]);
    for (let j = H.length - 1; j > 0 && roots.length < 2; j--) {
      const ga = g(H[j - 1]);
      if ((ga >= 0) !== (gb >= 0)) {
        const f = gb / (gb - ga);                                           // fraction from j towards j-1
        roots.push(H[j].t + (H[j - 1].t - H[j].t) * f);
      }
      gb = ga;
    }
    return roots;
  }

  /**
   * Sound at the listener.  L: { t (sim s), pos, vel (world, m/s), alt (m MSL), groundAlt (terrain under
   * the listener, m MSL), right (unit), cabin (bool: in the flight deck), rh (% at the ground) }.
   * Returns this.out: dir (band Pa^2 of the directional sound), lat (its arrival direction across the
   * listener: sin(azimuth), -1 left .. +1 right, energy-weighted per band),
   * dif (band Pa^2 of the diffuse / decorrelated sound), spl (overall dB re 20 uPa), delay (s), arrivals
   */
  listen(L) {
    const o = this.out;
    o.dir.fill(0); o.dif.fill(0); o.lat.fill(0); o.spl = 0; o.delay = 0; o.arrivals = 0;
    const H = this.hist;
    if (!H.length) return o;
    if (L.cabin) this._cabin(H[H.length - 1], o, L.right);
    else this._external(L, o);
    let p2 = 0;
    for (let i = 0; i < NB; i++) p2 += o.dir[i] + o.dif[i];
    o.spl = p2 > 0 ? 10 * Math.log10(p2 / 4e-10) : 0;
    return o;
  }

  _srcState(s, atm) {
    const va = sub(s.vel, s.wind), V = Math.hypot(va[0], va[1], va[2]);
    const u = V > 1e-3 ? [va[0] / V, va[1] / V, va[2] / V] : s.fwd.slice();
    const dn = orthoUnit([-s.up[0], -s.up[1], -s.up[2]], u), rt = orthoUnit(s.right, u);
    return { src: { M: V / atm.c, rho: atm.rho, c: atm.c, mu: atm.mu, V }, va, u, dn, rt };
  }

  _worldPos(s, p) {            // body FRD offset -> world
    const x = p[0], up = -p[2], r = p[1];
    return [s.pos[0] + s.fwd[0] * x + s.up[0] * up + s.right[0] * r,
      s.pos[1] + s.fwd[1] * x + s.up[1] * up + s.right[1] * r,
      s.pos[2] + s.fwd[2] * x + s.up[2] * up + s.right[2] * r];
  }

  /** diffuse transmission coefficients of the hull walls / windows for this outside air (cached) */
  hullTransmission(atm) {
    const gapT = 290, gap = { rho: atm.p / (R_AIR * gapT), c: Math.sqrt(GAM * R_AIR * gapT) };   // vented gap
    const key = Math.round(Math.log(atm.rho) * 50) + ':' + Math.round(atm.c);
    if (this._tau.key !== key) {
      const cab = { rho: CABIN.rho, c: CABIN.c }, tb = new Float64Array(NB);
      const bridged = (layers, B, out) => {
        transmissionBands(layers, atm, gap, cab, out);
        transmissionBands([{ m: B.m1 + B.m2, D: B.D2, fr: B.fr, eta: B.eta }], atm, gap, cab, tb);
        const fc2 = cab.c * cab.c / (2 * Math.PI) * Math.sqrt(B.m2 / B.D2);
        const dTL = 20 * Math.log10(B.e * fc2) + 20 * Math.log10(B.m1 / (B.m1 + B.m2)) - 45;
        for (let i = 0; i < NB; i++) out[i] += tb[i] * Math.pow(10, -Math.max(0, dTL) / 10);
      };
      bridged(HULL.wall, HULL.wallBridge, this._tau.wall);
      bridged(HULL.window, HULL.windowBridge, this._tau.win);
      this._tau.key = key;
    }
    return this._tau;
  }

  // ---- in the flight deck
  _cabin(s, o, right) {
    const atm = atmosphere(s.alt), ss = this._srcState(s, atm), U = ss.src.V;
    if (U < 3) return;
    const { wall: tw, win: tg } = this.hullTransmission(atm);
    // wall pressure: TBL at the windows and along the cabin walls (3 stations)
    const pw = this._b, pg = this._b2, t = this._pr;
    tblBands(U, atm, Math.max(0.5, this.nose - this.winX), pg);
    pw.fill(0);
    for (const xr of [3, 6, 9]) { tblBands(U, atm, xr, t); for (let i = 0; i < NB; i++) pw[i] += t[i] / 3; }
    // airframe sources at the cabin (co-moving: constant-velocity retarded time)
    const ff = this._ff;
    ff.fill(0);
    const eyeW = this._worldPos(s, this.eye), c = atm.c;
    const dirs = { u: ss.u, dn: ss.dn, rt: ss.rt, n: v3() };
    for (const comp of this.comps) {
      const P = this._worldPos(s, comp.pos), d0 = sub(eyeW, P);
      // |d0 + Va tau| = c tau  (in the air the source has moved back by Va tau since emission)
      const A = U * U - c * c, B = 2 * dot(d0, ss.va), C0 = dot(d0, d0);
      let tau = -1;
      if (Math.abs(A) < 1e-6) tau = B < 0 ? -C0 / B : -1;
      else {
        const disc = B * B - 4 * A * C0;
        if (disc >= 0) {
          const q1 = (-B - Math.sqrt(disc)) / (2 * A), q2 = (-B + Math.sqrt(disc)) / (2 * A);
          const lo = Math.min(q1, q2), hi = Math.max(q1, q2);
          tau = lo > 0 ? lo : hi > 0 ? hi : -1;
        }
      }
      if (tau <= 0) continue;                                             // cannot reach the cabin (supersonic, from aft)
      const r = c * tau;
      dirs.n[0] = (d0[0] + ss.va[0] * tau) / r; dirs.n[1] = (d0[1] + ss.va[1] * tau) / r; dirs.n[2] = (d0[2] + ss.va[2] * tau) / r;
      const dopL = 1 - dot(dirs.n, ss.va) / c;                              // the cabin moves with the vehicle
      finkComponent(comp, ss.src, dirs, Math.max(2.0, r), dopL, s.gear, s.surf, ff);
    }
    // power into the cabin (diffuse incidence on a wall: I = <p_wall^2> / (8 rho c)), reverberant + direct
    const zo = atm.rho * atm.c, zc = CABIN.rho * CABIN.c;
    const gd = Math.min(1, CABIN.Sfwd / (2 * Math.PI * CABIN.rEye * CABIN.rEye));
    for (let i = 0; i < NB; i++) {
      const Wtbl = (tw[i] * pw[i] * CABIN.Swall + tg[i] * pg[i] * CABIN.Swin) / (8 * zo);
      // airframe sources: free-field p^2 doubled at the hull, on the half of the hull facing them
      const Wsrc = (tw[i] * CABIN.Swall + tg[i] * CABIN.Swin) * 0.5 * (2 * ff[i]) / (8 * zo);
      const al = cabinAlpha(FC[i]), R = CABIN.Sroom * al / (1 - al);
      o.dif[i] = 4 * zc * (Wtbl + Wsrc) / R;
      o.dir[i] = zc * tg[i] * pg[i] / (8 * zo) * gd;                     // forward windscreen next to the eye
    }
    // the windscreen is ahead of the crew: turning the head moves it to the side
    if (right) o.lat.fill(Math.max(-1, Math.min(1, dot(s.fwd, right))));
  }

  // ---- outside
  _external(L, o) {
    const H = this.hist, now = H[H.length - 1];
    const aL = atmosphere(L.alt);
    // propagation sound speed: mean over the layer between listener and vehicle
    const aS0 = atmosphere(now.alt), aM = atmosphere((now.alt + L.alt) / 2);
    const cP = (aS0.c + aL.c + 2 * aM.c) / 4;
    const Lp = L.pos;
    const roots = this._emissions(L.t, Lp, cP);
    if (!roots.length) return;
    o.arrivals = roots.length;
    o.delay = L.t - roots[0];
    const pr = this._pr, wl = this._b;
    wl.fill(0);
    for (const te of roots) {
      const s = this._at(te), dt = L.t - te;
      const aS = atmosphere(s.alt), ss = this._srcState(s, aS);
      const W = s.wind;
      // geometry of the CG path (absorption, ground, shadow) - the components differ by metres
      const d = [Lp[0] - s.pos[0] - W[0] * dt, Lp[1] - s.pos[1] - W[1] * dt, Lp[2] - s.pos[2] - W[2] * dt];
      const r = Math.max(1, Math.hypot(d[0], d[1], d[2]));
      const dh = Math.hypot(d[0], d[2]);
      absorptionBands(s.alt, L.alt, r, L.groundAlt, L.rh ?? 60, this._abs);
      const hs = s.alt - L.groundAlt, hl = L.alt - L.groundAlt;
      groundBands(hs, hl, dh, cP, this._gnd);
      // acoustic shadow of upward refraction (ray radius ~ c / |dc/dz| ~ 87 km), as for the thunder
      const Rc = 87000, exc = dh - (Math.sqrt(2 * Rc * Math.max(hs, 2)) + Math.sqrt(2 * Rc * Math.max(hl, 1.5)));
      const sh = exc > 0 ? Math.pow(10, -Math.min(30, 12 * Math.min(1, exc / 1500) + exc / 1000 * 3) / 10) : 1;
      const imp = (aL.rho * aL.c) / (aS.rho * aS.c);
      for (let i = 0; i < NB; i++) this._amp[i] = this._abs[i] * this._gnd[i] * sh * imp;
      const VL = sub(L.vel, W);
      pr.fill(0);
      const dirs = { u: ss.u, dn: ss.dn, rt: ss.rt, n: v3() };
      for (const comp of this.comps) {
        const P = this._worldPos(s, comp.pos);
        const e = [Lp[0] - P[0] - W[0] * dt, Lp[1] - P[1] - W[1] * dt, Lp[2] - P[2] - W[2] * dt];
        const rc = Math.max(3, Math.hypot(e[0], e[1], e[2]));
        dirs.n[0] = e[0] / rc; dirs.n[1] = e[1] / rc; dirs.n[2] = e[2] / rc;
        const dopL = Math.max(0.05, 1 - dot(dirs.n, VL) / cP);
        finkComponent(comp, ss.src, dirs, rc, dopL, s.gear, s.surf, pr, this._amp);
      }
      // arrival direction across the listener (the sound comes from -d)
      const lat = Math.max(-1, Math.min(1, -(d[0] * L.right[0] + d[1] * L.right[1] + d[2] * L.right[2]) / r));
      for (let i = 0; i < NB; i++) { o.dir[i] += pr[i]; wl[i] += pr[i] * lat; }
    }
    for (let i = 0; i < NB; i++) o.lat[i] = o.dir[i] > 0 ? wl[i] / o.dir[i] : 0;
  }
}
