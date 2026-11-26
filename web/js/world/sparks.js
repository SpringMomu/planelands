// Sparks where the airframe grinds on the runway (wing tips, body flap / tail, engine nozzles, belly).
//
// The 6-DOF simulator reports every structural contact: where, how fast it slides, the normal force
// and the friction work done since the last state message (sim/gear.py drain_scrape).  The sparks
// are computed from that, particle by particle:
//
// * debris: abrasive wear removes material in proportion to the friction work (Archard); the
//   aluminium structure and silica tiles go to dust (aluminium does not ignite from friction), the
//   sparks are the titanium fittings and steel / Inconel parts in the worn region.  Lognormal sizes
//   (median ~0.15 mm), flash-heated in the contact toward their melting point the faster they slide
// * motion: thrown out of the contact with a fraction of the sliding speed (the stationary runway
//   asperities tear them off the moving part), deflected upward / sideways by the rough surface;
//   aerodynamic drag with the sphere drag law Cd(Re) against the local air (wind, vehicle wake,
//   wing-tip vortices: wake.js), gravity, bounces on the runway (restitution, friction, scatter off
//   the macrotexture), skittering along it
// * heat: lumped-capacity particle, convection (Ranz-Marshall Nu = 2 + 0.6 Re^1/2 Pr^1/3 with film
//   properties) and radiation (eps sigma T^4), implicit in time; titanium and steel ignite in air and
//   burn (diffusion-limited, burn time ~ d^2) near their flame temperatures (TiO2 ~3000 K, iron
//   oxide ~2200 K); carbon in steel boils out as CO at the end and bursts the droplet into the
//   forked "star" sparks; Inconel does not burn, it glows and cools.  Contact with the concrete
//   quenches them
// * light: a black body at the particle temperature times its emissivity (CIE 1931 matching, linear
//   sRGB, luminance in the scene's units: the sun's 20 = 128 klx).  A 0.1 mm spark is far below a
//   pixel, so each is drawn as a motion-blurred streak (relative to the camera, over the shutter time)
//   that carries exactly its radiant flux: pixel value = L x projected area / streak area.  The shutter
//   follows the exposure like a camera's auto-exposure (~1/400 s in sunlight: short bright dashes,
//   ~1/60 s at night: long trails).  Bloom does the rest, as in a camera.  The whole shower also lights its surroundings (a point light with
//   the summed luminous intensity and mean colour)
//
// One simulated particle stands for `w` real ones when the real count exceeds the budget (the light
// it carries is scaled by w, so the shower's total light is right).
import * as THREE from 'three';

const MAX = 16000;
const K_SCENE = 6400;              // cd/m^2 per scene radiance unit (sun irradiance 20 units = 128 klx)
const SIGMA = 5.670e-8;
const T_AIR = 290;
const RHO_AIR = 1.2;
const G = 9.81;
const RATE_CAP = 7000;             // simulated sparks per second and contact point

// ------------------------------------------------------------------ black body table
// CIE 1931 2-degree matching functions, multi-lobe fit (Wyman, Sloan & Shirley 2013)
const lobe = (l, m, s1, s2) => { const s = l < m ? s1 : s2; return Math.exp(-0.5 * ((l - m) / s) ** 2); };
const xbar = (l) => 1.056 * lobe(l, 599.8, 37.9, 31.0) + 0.362 * lobe(l, 442.0, 16.0, 26.7) - 0.065 * lobe(l, 501.1, 20.4, 26.2);
const ybar = (l) => 0.821 * lobe(l, 568.8, 46.9, 40.5) + 0.286 * lobe(l, 530.9, 16.3, 31.1);
const zbar = (l) => 1.217 * lobe(l, 437.0, 11.8, 36.0) + 0.681 * lobe(l, 459.0, 26.0, 13.8);
const TB0 = 600, TBS = 20, TBN = 160;          // 600 .. 3780 K
const BB = new Float32Array(TBN * 4);           // linear sRGB radiance (scene units) + Lv (cd/m^2), emissivity 1
{
  const h = 6.62607e-34, c = 2.99792458e8, k = 1.380649e-23;
  for (let i = 0; i < TBN; i++) {
    const T = TB0 + i * TBS;
    let X = 0, Y = 0, Z = 0;
    for (let l = 380; l <= 780; l += 4) {
      const lm = l * 1e-9;
      const B = 2 * h * c * c / lm ** 5 / Math.expm1(h * c / (lm * k * T)) * 4e-9;   // W / (m^2 sr) in the 4 nm bin
      X += xbar(l) * B; Y += ybar(l) * B; Z += zbar(l) * B;
    }
    const Lv = 683 * Y;
    const r = 3.2406 * X - 1.5372 * Y - 0.4986 * Z, g = -0.9689 * X + 1.8758 * Y + 0.0415 * Z, b = 0.0557 * X - 0.2040 * Y + 1.0570 * Z;
    // scale the chromaticity to the luminance in scene units (deep red: clip the negative lobes)
    const R = Math.max(r, 0), Gg = Math.max(g, 0), Bb = Math.max(b, 0);
    const Ys = 0.2126 * R + 0.7152 * Gg + 0.0722 * Bb;
    const s = Ys > 0 ? Lv / K_SCENE / Ys : 0;
    BB[i * 4] = R * s; BB[i * 4 + 1] = Gg * s; BB[i * 4 + 2] = Bb * s; BB[i * 4 + 3] = Lv;
  }
}
/** black-body radiance at T into out[0..2] (scene units), returns the luminance (cd/m^2) */
function blackbody(T, out) {
  const f = (T - TB0) / TBS;
  if (f <= 0) { out[0] = out[1] = out[2] = 0; return 0; }
  const i = Math.min(Math.floor(f), TBN - 2), u = Math.min(f - i, 1), o = i * 4;
  for (let k = 0; k < 3; k++) out[k] = BB[o + k] + (BB[o + 4 + k] - BB[o + k]) * u;
  // the luminance falls ~exp(-c2 / lambda T): interpolate it in log space
  return Math.exp(Math.log(BB[o + 3] + 1e-30) * (1 - u) + Math.log(BB[o + 7] + 1e-30) * u);
}

// ------------------------------------------------------------------ materials
// rho kg/m^3, c J/(kg K) (at high temperature), Tm melting K, Tign ignition in air (fresh debris), Tburn
// particle temperature while burning, Kb burn time / d^2 (s/m^2), burst: chance of the CO burst at
// burn-out, eps emissivity (oxide)
const MAT = [
  { name: 'steel', rho: 7850, c: 650, Tm: 1720, Tign: 1300, Tburn: 2200, Kb: 1.4e6, burst: 0.6, eps: 0.75 },
  { name: 'ti', rho: 4430, c: 640, Tm: 1930, Tign: 1500, Tburn: 2950, Kb: 6.5e6, burst: 0.12, eps: 0.6 },
  { name: 'inconel', rho: 8190, c: 480, Tm: 1610, Tign: 1e9, Tburn: 0, Kb: 0, burst: 0, eps: 0.8 },
];
// what grinds at each contact region: hot metal fragments per joule of friction work (wear volume per
// joule (K/H)/mu x metal share / mean fragment volume) and the metal mix; dust: tile / concrete / Al fines
const REGION = {
  wing_l: { perJ: 0.4, mix: [0.4, 0.6, 0], dust: 1.0 },
  wing_r: { perJ: 0.4, mix: [0.4, 0.6, 0], dust: 1.0 },
  tail: { perJ: 0.7, mix: [0.25, 0.3, 0.45], dust: 0.7 },
  nozzle: { perJ: 1.5, mix: [0.65, 0, 0.35], dust: 0.25 },
  belly: { perJ: 0.35, mix: [0.6, 0.4, 0], dust: 1.0 },
  nose: { perJ: 0.35, mix: [0.6, 0.4, 0], dust: 1.0 },
};

/** relative amount of abrasion dust (tiles, concrete, aluminium fines) a contact region grinds off */
export const dustShare = (region) => (REGION[region] || REGION.belly).dust;

const gauss = () => { let u = 0; while (u === 0) u = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(6.2831853 * Math.random()); };

// ------------------------------------------------------------------ shaders
const VS = /* glsl */ `
precision highp float;
in vec3 position;            // x: 0 tail .. 1 head, y: -1 .. 1 across
in vec4 iP;                  // head position (world), diameter (m)
in vec4 iS;                  // xyz: streak vector (head - tail, world m)
in vec4 iC;                  // rgb: radiance x emissivity x weight (scene units)
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
uniform vec2 uRes;
out vec3 vCol; out vec2 vQ; out float vViewZ; out float vLen;
void main(){
  vec3 head = iP.xyz, tail = iP.xyz - iS.xyz;
  vec4 v0 = viewMatrix * vec4(tail, 1.0), v1 = viewMatrix * vec4(head, 1.0);
  // clip the streak against the near plane
  if (v1.z > -0.3 && v0.z > -0.3){ gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
  if (v0.z > -0.3) v0 = mix(v1, v0, (v1.z + 0.3) / (v1.z - v0.z));
  if (v1.z > -0.3) v1 = mix(v0, v1, (v0.z + 0.3) / (v0.z - v1.z));
  vec4 c0 = projectionMatrix * v0, c1 = projectionMatrix * v1;
  vec2 s0 = c0.xy / c0.w * uRes * 0.5, s1 = c1.xy / c1.w * uRes * 0.5;
  vec2 d = s1 - s0;
  float L = length(d);
  vec2 dir = L > 1e-4 ? d / L : vec2(1.0, 0.0);
  vec2 perp = vec2(-dir.y, dir.x);
  vec4 vp = mix(v0, v1, position.x);
  // projected particle diameter (px); the streak is drawn at least ~1.4 px wide
  float dpx = iP.w * projectionMatrix[1][1] * 0.5 * uRes.y / max(-vp.z, 0.3);
  float w = max(dpx, 1.4);
  // energy: the particle's flux over one shutter (radiance x pi/4 dp^2 px^2) spread over the streak
  // capsule (L + w) x w; the across profile below has unit mean
  vCol = iC.rgb * (0.7854 * dpx * dpx) / ((L + w) * w);
  vec2 sp = mix(s0, s1, position.x) + perp * position.y * w * 0.5 + dir * (position.x * 2.0 - 1.0) * w * 0.5;
  vec4 c = projectionMatrix * vp;
  c.xy = sp / (uRes * 0.5) * c.w;
  gl_Position = c;
  vQ = vec2(position.x * 2.0 - 1.0, position.y);
  vViewZ = -vp.z;
  vLen = L / (L + w);
}`;

const FS = /* glsl */ `
precision highp float;
uniform sampler2D uDepth;
uniform vec2 uRes;
uniform float uLogFar;
in vec3 vCol; in vec2 vQ; in float vViewZ; in float vLen;
out vec4 outColor;
void main(){
  float d = texture(uDepth, gl_FragCoord.xy / uRes).r;
  float sw = d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
  if (vViewZ > sw + 0.12 + 0.004 * vViewZ) discard;              // behind the airframe / runway
  float across = 1.5 * max(1.0 - vQ.y * vQ.y, 0.0);
  // round caps over the last half-width at each end
  float e = max(abs(vQ.x) - vLen, 0.0) / max(1.0 - vLen, 1e-3);
  float cap = max(1.0 - e * e, 0.0) * 1.15;
  outColor = vec4(vCol * across * mix(1.0, cap, step(vLen, abs(vQ.x))), 0.0);
}`;

export class Sparks {
  constructor() {
    this.scene = new THREE.Scene();
    const quad = new THREE.PlaneGeometry(1, 2, 1, 1);
    quad.translate(0.5, 0, 0);
    const g = new THREE.InstancedBufferGeometry();
    g.index = quad.index;
    g.setAttribute('position', quad.getAttribute('position'));
    this.iP = new Float32Array(MAX * 4);
    this.iS = new Float32Array(MAX * 4);
    this.iC = new Float32Array(MAX * 4);
    for (const [k, a] of [['iP', this.iP], ['iS', this.iS], ['iC', this.iC]]) g.setAttribute(k, new THREE.InstancedBufferAttribute(a, 4).setUsage(THREE.DynamicDrawUsage));
    g.instanceCount = 0;
    this.geo = g;
    this.u = { uDepth: { value: null }, uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 } };
    this.mat = new THREE.RawShaderMaterial({
      vertexShader: VS, fragmentShader: FS, uniforms: this.u, glslVersion: THREE.GLSL3,
      transparent: true, depthTest: false, depthWrite: false,
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
      blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor,
      blendSrcAlpha: THREE.ZeroFactor, blendDstAlpha: THREE.OneFactor,
    });
    this.mesh = new THREE.Mesh(g, this.mat);
    this.mesh.frustumCulled = false;
    this.scene.add(this.mesh);
    const F = () => new Float32Array(MAX);
    this.p = {
      // fuel: > 0 burning (fraction of the metal left), 0 not ignited, -1 burnt out
      x: F(), y: F(), z: F(), vx: F(), vy: F(), vz: F(), d: F(), T: F(), Tb: F(), fuel: F(), age: F(), w: F(),
      mat: new Uint8Array(MAX), gen: new Uint8Array(MAX), ground: new Uint8Array(MAX),
    };
    this.n = 0;
    this.active = false;
    this.light = { I: 0, pos: new THREE.Vector3(), col: new THREE.Color() };   // the shower as a light source
    this.dustOut = [];
    this._c = [0, 0, 0];
    this._u = new THREE.Vector3();
    this._sv = new THREE.Vector3();
    this.lastEmit = new Map();       // contact id -> previous emission point (three world)
  }

  setSize(w, h) { this.u.uRes.value.set(w, h); }
  clear() { this.n = 0; this.active = false; this.geo.instanceCount = 0; this.lastEmit.clear(); this.light.I = 0; }

  /**
   * Debris from one reported contact.
   * @param {object} c    record from the simulator (region, W J, dt s, N, v ENU, id)
   * @param {Vector3} at  contact point on the runway (three world)
   * @param {Vector3} vs  sliding velocity of the airframe point over the runway (three world)
   * @returns {number}    real fragments released (for the dust)
   */
  emit(c, at, vs) {
    const reg = REGION[c.region] || REGION.belly;
    const spd = Math.hypot(vs.x, vs.z);
    // below a few m/s sliding the flash temperature stays far from glowing
    const fv = 1 - Math.exp(-spd / 25);
    const nReal = c.W * reg.perJ * THREE.MathUtils.smoothstep(spd, 2, 12);
    if (nReal < 0.5) return 0;
    let nSim = Math.min(nReal, RATE_CAP * Math.max(c.dt, 1e-3));
    nSim = Math.floor(nSim) + (Math.random() < nSim % 1 ? 1 : 0);
    if (!nSim) return nReal;
    const w = nReal / nSim;
    const sx = spd > 0.1 ? vs.x / spd : 1, sz = spd > 0.1 ? vs.z / spd : 0;
    // spread the release over the stretch the point slid since its last emission
    const prev = this.lastEmit.get(c.id);
    let bx = at.x - vs.x * c.dt, bz = at.z - vs.z * c.dt;
    if (prev && prev.distanceToSquared(at) < (spd * c.dt * 3 + 1) ** 2) { bx = prev.x; bz = prev.z; }
    if (prev) prev.copy(at); else this.lastEmit.set(c.id, at.clone());
    const P = this.p, mix = reg.mix;
    for (let k = 0; k < nSim && this.n < MAX; k++) {
      const i = this.n++;
      const u = Math.random();
      P.x[i] = bx + (at.x - bx) * u - sz * gauss() * 0.12;
      P.z[i] = bz + (at.z - bz) * u + sx * gauss() * 0.12;
      P.y[i] = at.y + 0.01 + Math.random() * 0.04;
      // thrown out with part of the sliding speed, scattered by the asperities: mostly flat, a few high
      const r = Math.random();
      const sp = spd * (0.06 + 0.5 * Math.random() ** 1.3);
      const psi = gauss() * 0.28, phi = 0.03 + 0.55 * r * r * r + 0.08 * Math.random();
      const ch = Math.cos(phi) * sp, cp = Math.cos(psi), sn = Math.sin(psi);
      P.vx[i] = (sx * cp - sz * sn) * ch;
      P.vz[i] = (sz * cp + sx * sn) * ch;
      P.vy[i] = Math.sin(phi) * sp;
      // size: lognormal, median 0.15 mm
      P.d[i] = Math.min(Math.max(1.5e-4 * Math.exp(0.65 * gauss()), 3e-5), 1.5e-3);
      const m = Math.random();
      const mt = m < mix[0] ? 0 : m < mix[0] + mix[1] ? 1 : 2;
      const M = MAT[mt];
      P.mat[i] = mt;
      // flash temperature: toward melting at high sliding speed
      P.T[i] = T_AIR + (M.Tm - T_AIR) * fv * (0.72 + 0.28 * Math.random());
      P.fuel[i] = P.T[i] >= M.Tign ? 1 : 0;
      P.Tb[i] = M.Tburn * (0.93 + 0.1 * Math.random());
      P.age[i] = 0; P.w[i] = w; P.gen[i] = 0; P.ground[i] = 0;
    }
    return nReal;
  }

  _burst(i) {
    const P = this.p;
    const n = 3 + Math.floor(Math.random() * 6);
    for (let k = 0; k < n && this.n < MAX; k++) {
      const j = this.n++;
      P.x[j] = P.x[i]; P.y[j] = P.y[i]; P.z[j] = P.z[i];
      // the CO bubble bursts the droplet: fragments fly off in all directions
      let ux = gauss(), uy = gauss(), uz = gauss();
      const l = Math.hypot(ux, uy, uz) || 1;
      const s = 5 + 11 * Math.random();
      P.vx[j] = P.vx[i] + ux / l * s; P.vy[j] = P.vy[i] + uy / l * s; P.vz[j] = P.vz[i] + uz / l * s;
      P.d[j] = P.d[i] * (0.16 + 0.22 * Math.random());
      P.T[j] = P.Tb[i]; P.Tb[j] = P.Tb[i] * (0.97 + 0.05 * Math.random());
      P.fuel[j] = 1; P.mat[j] = P.mat[i]; P.gen[j] = 1; P.age[j] = 0; P.w[j] = P.w[i]; P.ground[j] = 0;
    }
  }

  /**
   * @param {number} dt   sim-time step (s)
   * @param {object} s    { wake: WakeField }
   */
  update(dt, s) {
    const P = this.p;
    if (dt <= 0 || !this.n) { this.active = this.n > 0; return; }
    const nsub = Math.min(6, Math.ceil(dt / 0.01));
    const h = dt / nsub;
    const wake = s.wake, ua = this._u;
    for (let it = 0; it < nsub; it++) {
      const n0 = this.n;
      for (let i = 0; i < n0; i++) {
        if (P.T[i] < 0) continue;                       // already dead (compacted below)
        const M = MAT[P.mat[i]];
        const d = P.d[i];
        let T = P.T[i];
        wake.at(P.x[i], P.y[i], P.z[i], ua);
        const rx = P.vx[i] - ua.x, ry = P.vy[i] - ua.y, rz = P.vz[i] - ua.z;
        const vr = Math.hypot(rx, ry, rz);
        // film properties (air between the hot particle and the ambient)
        const Tf = 0.5 * (T + T_AIR);
        const mu = 1.716e-5 * (Tf / 273) ** 0.7, kf = 0.0241 * (Tf / 273) ** 0.81;
        const Re = Math.max(RHO_AIR * (T_AIR / Tf) * vr * d / mu, 1e-3);
        // drag: Schiller-Naumann sphere drag, relaxation toward the air velocity (exact for constant tau)
        const Cd = Re < 1000 ? 24 / Re * (1 + 0.15 * Re ** 0.687) : 0.44;
        const tau = (4 / 3) * M.rho * d / (RHO_AIR * Cd * Math.max(vr, 1e-3));
        const ex = Math.exp(-h / tau), gt = G * tau * (1 - ex);
        const nvx = ua.x + rx * ex, nvy = ua.y + ry * ex - gt, nvz = ua.z + rz * ex;
        P.x[i] += 0.5 * (P.vx[i] + nvx) * h; P.y[i] += 0.5 * (P.vy[i] + nvy) * h; P.z[i] += 0.5 * (P.vz[i] + nvz) * h;
        P.vx[i] = nvx; P.vy[i] = nvy; P.vz[i] = nvz;
        // heat
        if (P.fuel[i] > 0) {
          // burning: held near the flame temperature until the metal is consumed (t_b ~ d^2)
          T = P.Tb[i] + (T - P.Tb[i]) * Math.exp(-h / 0.004);
          P.fuel[i] -= h / (M.Kb * d * d);
          if (P.fuel[i] <= 0) {
            P.fuel[i] = -1;                              // burnt out: an oxide droplet now
            if (P.gen[i] === 0 && Math.random() < M.burst) {
              this._burst(i);
              if (Math.random() < 0.7) T = -1;           // the droplet is gone in the burst
            }
          }
        } else {
          const Nu = 2 + 0.6 * Math.sqrt(Re) * 0.89;
          const hc = Nu * kf / d * (P.ground[i] ? 4 : 1);     // lying on the concrete: conduction
          const k6 = 6 / (M.rho * M.c * d);
          const f = -(hc * (T - T_AIR) + M.eps * SIGMA * (T ** 4 - T_AIR ** 4)) * k6;
          const fp = -(hc + 4 * M.eps * SIGMA * T ** 3) * k6;
          T += h * f / (1 - h * fp);
          if (T >= M.Tign && P.fuel[i] === 0) P.fuel[i] = 1;           // ignites (never burnt yet)
        }
        // runway
        if (T > 0) {
          const gy = wake.groundAt(P.x[i], P.z[i]) + 0.5 * d;
          if (P.y[i] < gy) {
            P.y[i] = gy;
            if (P.vy[i] < -0.4) {
              // bounce off the macrotexture: restitution, friction, scatter; the concrete quenches it
              const e = 0.2 + 0.3 * Math.random(), kt = 0.55 + 0.3 * Math.random();
              const vt = Math.hypot(P.vx[i], P.vz[i]);
              P.vy[i] = -P.vy[i] * e + Math.abs(gauss()) * 0.12 * vt;
              const a = gauss() * 0.35, ca = Math.cos(a), sa = Math.sin(a);
              const vx = P.vx[i] * kt, vz = P.vz[i] * kt;
              P.vx[i] = vx * ca - vz * sa; P.vz[i] = vz * ca + vx * sa;
              T = T_AIR + (T - T_AIR) * 0.82;
              // a burning steel droplet can shatter on impact
              if (P.fuel[i] > 0 && P.mat[i] === 0 && P.gen[i] === 0 && Math.random() < 0.25) { this._burst(i); T = -1; }
            } else {
              // skittering along the runway: sliding friction; lying on the concrete quenches the flame
              P.vy[i] = 0;
              P.ground[i] = 1;
              if (P.fuel[i] > 0 && Math.random() < h / 0.15) P.fuel[i] = -1;
              const vt = Math.hypot(P.vx[i], P.vz[i]);
              if (vt > 1e-3) { const k = Math.max(0, 1 - 0.5 * G * h / vt); P.vx[i] *= k; P.vz[i] *= k; }
            }
          } else if (P.y[i] > gy + 0.02) P.ground[i] = 0;
        }
        P.T[i] = T;
        P.age[i] += h;
      }
    }
    // compact: drop dark (< 950 K: invisible even at night), dead or old particles
    let wI = 0;
    let Isum = 0, cx = 0, cy = 0, cz = 0, cr = 0, cg = 0, cb = 0, c2 = 0;
    const col = this._c;
    for (let i = 0; i < this.n; i++) {
      if (P.T[i] < 950 || P.age[i] > 5) continue;
      if (wI !== i) for (const k in P) P[k][wI] = P[k][i];
      // luminous intensity of this particle (cd): Lv x eps x projected area x weight
      const M = MAT[P.mat[wI]];
      const Lv = blackbody(P.T[wI], col);
      const I = Lv * M.eps * 0.7854 * P.d[wI] * P.d[wI] * P.w[wI];
      Isum += I; cx += P.x[wI] * I; cy += P.y[wI] * I; cz += P.z[wI] * I; c2 += (P.x[wI] ** 2 + P.z[wI] ** 2) * I;
      const yl = 0.2126 * col[0] + 0.7152 * col[1] + 0.0722 * col[2] || 1;
      cr += col[0] / yl * I; cg += col[1] / yl * I; cb += col[2] / yl * I;
      wI++;
    }
    this.n = wI;
    this.active = wI > 0;
    const L = this.light;
    L.I = Isum / K_SCENE;
    if (Isum > 0) {
      // a shower metres long is no point source: lift the equivalent light by its spread so the runway
      // under it gets the irradiance of the distributed sparks, not 1/r^2 of their sum
      const mx = cx / Isum, mz = cz / Isum;
      const spread = Math.sqrt(Math.max(c2 / Isum - mx * mx - mz * mz, 0));
      L.pos.set(mx, cy / Isum + 0.3 + 0.6 * spread, mz);
      L.col.setRGB(cr / Isum, cg / Isum, cb / Isum);
    }
  }

  /** draw into the (premultiplied) particle layer rt; clear it first when nothing else is in it.
   *  shutter: exposure time (s of real time), ts: sim time scale */
  render(renderer, camera, depthTexture, rt, clear, camVel, ts, shutter) {
    if (!this.active || !rt) return false;
    const P = this.p, n = this.n, col = this._c;
    const sh = shutter * ts;
    for (let i = 0; i < n; i++) {
      const M = MAT[P.mat[i]];
      blackbody(P.T[i], col);
      const k = M.eps * P.w[i];
      const o = i * 4;
      this.iP[o] = P.x[i]; this.iP[o + 1] = P.y[i]; this.iP[o + 2] = P.z[i]; this.iP[o + 3] = P.d[i];
      // the streak: motion relative to the camera over the shutter (no longer than the particle has lived)
      const t = Math.min(sh, P.age[i]);
      this.iS[o] = (P.vx[i] - camVel.x) * t; this.iS[o + 1] = (P.vy[i] - camVel.y) * t; this.iS[o + 2] = (P.vz[i] - camVel.z) * t;
      this.iC[o] = col[0] * k; this.iC[o + 1] = col[1] * k; this.iC[o + 2] = col[2] * k;
    }
    this.geo.instanceCount = n;
    this.geo.attributes.iP.needsUpdate = true;
    this.geo.attributes.iS.needsUpdate = true;
    this.geo.attributes.iC.needsUpdate = true;
    this.u.uDepth.value = depthTexture;
    this.u.uLogFar.value = Math.log2(camera.far + 1.0);
    const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
    renderer.setRenderTarget(rt);
    if (clear) { renderer.setClearColor(0x000000, 0); renderer.clear(true, false, false); }
    renderer.render(this.scene, camera);
    renderer.setClearColor(cc, ca);
    return true;
  }
}
