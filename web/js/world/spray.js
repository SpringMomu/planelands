// Tyre spray on a wet runway: the water the tyres push out of their way, thrown up, broken into
// drops and mist, carried by the vehicle's wake and wing-tip vortices, lit by the sun and the sky.
//
// Lagrangian spray parcels (as in engine-spray CFD): each parcel is a cloud of n equal drops of
// diameter d, spread over a gaussian of width sigma (plus the streak it was laid along).
//
// * source (per tyre, two per gear): a tyre at speed V on a water film of depth h displaces
//   rho h b V kg/s.  The film is squeezed out ahead of the footprint and leaves its edges as two
//   sheets at about the tyre speed relative to the tyre, at a small plan angle to the track (wider at
//   low speed) and a low elevation (ESDU 83042, NASA TN D-4864 spray patterns); between the two tyres of
//   a gear the inner sheets collide and turn up into a central plume.  The water caught in the tread
//   grooves is flung off the rear of the tyre (rolling: v = V (1 - cos b) forward, V sin b up at the
//   angle b behind the contact): the "rooster tail".  Water held in the runway macrotexture stays.
//   During the touchdown spin-up the skidding tyre planes on the film: no tread throw, stronger
//   sheets.  Above the dynamic hydroplaning speed (9 sqrt(p psi) kt) in deep water the bow wave
//   flattens and widens the sheets
// * dense spray: the water drags the air along (two-way coupling through the mass loading), so the
//   sheets penetrate metres sideways and upward before they thin out and stop
// * drops: sheet break-up into mm-size drops; aerodynamic drag with Cd(Re) (deformed drops drag
//   more); secondary break-up above the critical Weber number We = rho_a v_rel^2 d / sigma_w = 12
//   after the bag break-up time (~5 d / v_rel sqrt(rho_w / rho_a)), into fragments of ~1/4 the
//   size (the parcel keeps its water mass): fast drops atomise into the mist that hangs behind the
//   vehicle.  Gravity; drops hitting the runway mostly merge with the film, fast ones splash
//   (Mundo K = We^1/2 Re^1/4 > 57.7) and re-eject a fraction as small drops.  Spray thrown up against
//   the underside of the wings / fuselage runs along it.  Turbulent diffusion grows the parcels
//   (strong in the wake), the wake and the trailing vortices in ground effect (wake.js) carry the
//   mist forward, up and outward, the wind drifts it
// * look: coarse spray is drawn with fibrous "feather" sprites (drop streaks and droplets laid along its
//   motion), mist with wispy billows, both generated procedurally at start-up into one texture atlas and
//   normalised so that each sprite carries exactly the parcel's extinction cross-section; the sprite
//   cross-fades from feather to mist as the drops atomise
// * light: optical depth of a parcel = n Q_ext pi d^2/4 / (area), Q_ext = 2 (drops >> wavelength);
//   single-scattering albedo 1.  Phase function of water drops: strong forward diffraction lobe,
//   weaker back lobe, the primary (42 deg, red outside) and secondary (51 deg, reversed) rainbows
//   around the antisolar point, narrowing for big drops (Airy width ~ d^-2/3) and washing out into a
//   white fog bow for mist; sun light attenuated through the parcel, multiple-scattering glow of dense
//   spray, sky and runway light
import * as THREE from 'three';

const MAX = 16000;
const RHO_W = 1000, RHO_A = 1.2, SIG_W = 0.072, MU_A = 1.8e-5, G = 9.81;
const EMIT_RATE = 110;            // parcels per second per sheet at full flow
// per gear: tyre width b, radius r, half spacing of the two tyres, tyre pressure (psi)
const TYRE = [{ b: 0.22, r: 0.41, half: 0.2, psi: 300 }, { b: 0.38, r: 0.56, half: 0.3, psi: 315 }, { b: 0.38, r: 0.56, half: 0.3, psi: 315 }];
const TEXTURE_H = 0.4e-3;         // water held in the runway macrotexture / grooves (m)

/** depth of the water film on the runway (m) from the wetness 0..1 (and the rain falling on it) */
export function waterDepth(wet, rain = 0) {
  const w = Math.min(1, Math.max(0, (wet - 0.15) / 0.85));
  return 3.0e-3 * w ** 1.6 * (1 + 0.25 * Math.min(1, rain));
}

const gauss = () => { let u = 0; while (u === 0) u = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(6.2831853 * Math.random()); };
const ss = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

// ------------------------------------------------------------------ sprite atlas
// 4 x 2 tiles of TS^2: row 0 "feathers" (coarse spray: streaks of drops along the motion, x axis, plus
// single droplets), row 1 mist billows.  Every tile fades to zero at its border (no mip bleeding).
// Returns the texture and the mean density of each row (the shader normalises by it, so a sprite
// carries the same extinction as the gaussian it replaces).
const TS = 128;
function makeAtlas() {
  const W = TS * 4, H = TS * 2;
  const D = new Float32Array(W * H);
  let seed = 1234567;
  const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  const rg = () => { let u = 0; while (u === 0) u = rnd(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(6.2831853 * rnd()); };
  const splat = (ox, oy, x, y, s, a) => {
    const r = Math.ceil(s * 2.5);
    for (let j = Math.max(1, Math.floor(y - r)); j <= Math.min(TS - 2, Math.ceil(y + r)); j++) {
      for (let i = Math.max(1, Math.floor(x - r)); i <= Math.min(TS - 2, Math.ceil(x + r)); i++) {
        const d2 = ((i - x) ** 2 + (j - y) ** 2) / (s * s);
        if (d2 < 6.25) D[(oy + j) * W + ox + i] += a * Math.exp(-0.5 * d2);
      }
    }
  };
  // value noise for the mist
  const hash = (i, j, k) => { let h = (i * 374761393 + j * 668265263 + k * 2147483647) >>> 0; h = ((h ^ (h >>> 13)) * 1274126177) >>> 0; return ((h ^ (h >>> 16)) >>> 0) / 4294967296; };
  const vnoise = (x, y, k) => {
    const i = Math.floor(x), j = Math.floor(y), fx = x - i, fy = y - j, u = fx * fx * (3 - 2 * fx), v = fy * fy * (3 - 2 * fy);
    const a = hash(i, j, k), b = hash(i + 1, j, k), c = hash(i, j + 1, k), d = hash(i + 1, j + 1, k);
    return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
  };
  for (let t = 0; t < 4; t++) {
    // feathers: a fan of drop streaks, densest along the axis, plus scattered droplets
    const ox = t * TS;
    for (let k = 0; k < 70; k++) {
      const y0 = 0.5 + rg() * 0.12, xa = 0.1 + rnd() * 0.35, L = 0.2 + rnd() * 0.5;
      const slope = (y0 - 0.5) * 0.45 + rg() * 0.03;
      const s = 0.45 + rnd() * 1.1, a = 0.25 + rnd() * 0.75;
      for (let x = xa; x < Math.min(xa + L, 0.92); x += 0.5 / TS) {
        const f = (x - xa) / L;
        splat(ox, 0, x * TS, (y0 + slope * (x - xa)) * TS, s, a * (1 - f) ** 0.6 * 0.5);
      }
    }
    for (let k = 0; k < 320; k++) {
      splat(ox, 0, (0.5 + rg() * 0.2) * TS, (0.5 + rg() * 0.16) * TS, 0.5 + rnd() * 0.9, 0.3 + rnd() * 0.7);
    }
    // mist: domain-warped fbm under a soft round envelope, eroded into wisps at the edge
    const oy = TS;
    for (let j = 1; j < TS - 1; j++) {
      for (let i = 1; i < TS - 1; i++) {
        const x = i / TS - 0.5, y = j / TS - 0.5;
        const r = Math.hypot(x, y) * 2;
        let n = 0, amp = 0.5, fq = 3;
        const wx = vnoise(x * 3 + 7 * t, y * 3, 9) * 0.6, wy = vnoise(x * 3, y * 3 + 7 * t, 11) * 0.6;
        for (let o = 0; o < 5; o++) { n += amp * vnoise((x + wx) * fq + 13 * t, (y + wy) * fq, o + 20 * t); amp *= 0.5; fq *= 2.07; }
        const env = Math.max(0, 1 - r * r) ** 1.3;
        D[(oy + j) * W + ox + i] = env * ss(0.2, 0.9, n * 1.25 * (0.55 + 0.6 * env));
      }
    }
  }
  // feather tiles: soft envelope, normalise each row to max 1
  const mean = [0, 0];
  for (let row = 0; row < 2; row++) {
    let mx = 0;
    for (let j = 0; j < TS; j++) for (let i = 0; i < W; i++) {
      const k = (row * TS + j) * W + i;
      if (row === 0) {
        const x = (i % TS) / TS - 0.5, y = j / TS - 0.5;
        D[k] *= Math.exp(-((x / 0.33) ** 2) - (y / 0.27) ** 2);
      }
      mx = Math.max(mx, D[k]);
    }
    let sum = 0;
    for (let j = 0; j < TS; j++) for (let i = 0; i < W; i++) { const k = (row * TS + j) * W + i; D[k] = Math.min(1, D[k] / mx); sum += D[k]; }
    mean[row] = sum / (TS * W);
  }
  const data = new Uint8Array(W * H);
  for (let k = 0; k < W * H; k++) data[k] = Math.round(D[k] * 255);
  const tex = new THREE.DataTexture(data, W, H, THREE.RedFormat, THREE.UnsignedByteType);
  tex.minFilter = THREE.LinearMipmapLinearFilter; tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true; tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.needsUpdate = true;
  return { tex, mean };
}

const VS = /* glsl */ `
precision highp float;
in vec3 position;
in vec4 iPos;      // xyz world position, w gaussian width sigma (m)
in vec4 iE;        // xyz streak the parcel was laid along (world m), w drop diameter (m)
in vec4 iPar;      // x optical depth scale n d^2 / 4 (m^2), y seed, z age (s), w life fraction
in vec3 iV;        // velocity relative to the camera x shutter (world m)
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
out vec2 vUV; out float vTau; out float vSeed; out float vD;
out float vViewZ; out float vSig; out vec3 vVPos; out vec3 vAx; out vec3 vAy; out vec3 vToCam;
void main(){
  // the parcel's streak (as laid + motion blur), projected onto the image plane at the parcel
  vec3 E = iE.xyz + iV;
  vec3 vc = (viewMatrix * vec4(iPos.xyz, 1.0)).xyz;
  vec3 Ev = mat3(viewMatrix) * E;
  vec3 vdir = normalize(vc);
  vec3 Ep = Ev - vdir * dot(Ev, vdir);                     // component across the line of sight
  float l = length(Ep);
  vec3 ax = l > 1e-4 ? Ep / l : normalize(cross(vdir, vec3(0.0, 1.0, 0.0)) + vec3(1e-4));
  vec3 ay = normalize(cross(ax, vdir));                   // (ax, ay) faces the camera
  float sig = iPos.w;
  float hl = 0.5 * l;
  // column optical depth at the centre: the drops' extinction cross-section (n Q pi d^2/4, Q = 2) over the
  // projected area of the gaussian (2 pi sigma^2) drawn out along the projected streak
  vTau = iPar.x / (sig * sig) / (1.0 + l / (2.5066 * sig));
  float R = 2.6 * sig;
  vec2 q = position.xy * 2.0;                               // -1..1
  vec3 p = vc + ax * q.x * (hl + R) + ay * q.y * R;
  // sprite coordinates, mirrored at random so neighbouring parcels never repeat
  vec2 fl = vec2(fract(iPar.y * 17.3) < 0.5 ? 1.0 : -1.0, fract(iPar.y * 31.7) < 0.5 ? 1.0 : -1.0);
  vUV = q * fl * 0.5 + 0.5;
  vSig = sig; vSeed = iPar.y; vD = iE.w;
  vAx = ax; vAy = ay; vToCam = -vdir;
  vViewZ = -p.z; vVPos = p;
  gl_Position = projectionMatrix * vec4(p, 1.0);
}`;

const FS = /* glsl */ `
precision highp float;
uniform sampler2D uDepth;
uniform sampler2D uAtlas;  // row 0 feathers (coarse spray), row 1 mist
uniform vec2 uNorm;        // gaussian mean / row mean: a sprite carries the parcel's extinction
uniform vec2 uRes;
uniform float uLogFar;
uniform vec3 uSunView;     // direction toward the sun (view space)
uniform vec3 uUpView;
uniform vec3 uSunRad;      // sun irradiance (cloud-shadowed)
uniform vec3 uAmb;         // sky light
uniform vec3 uGround;      // runway light
in vec2 vUV; in float vTau; in float vSeed; in float vD;
in float vViewZ; in float vSig; in vec3 vVPos; in vec3 vAx; in vec3 vAy; in vec3 vToCam;
out vec4 outColor;
float hgp(float c, float g){ float g2 = g * g; return (1.0 - g2) / (12.566 * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5)); }
void main(){
  // coarse drops: streaky feather; atomised drops: mist (cross-fade over the break-up)
  float coarse = smoothstep(1.0e-4, 5.0e-4, vD);
  float tf = floor(fract(vSeed * 7.13) * 4.0), tm = floor(fract(vSeed * 3.71) * 4.0);
  float f = coarse > 0.01 ? texture(uAtlas, vec2((tf + vUV.x) * 0.25, vUV.y * 0.5)).r * uNorm.x : 0.0;
  float m = coarse < 0.99 ? texture(uAtlas, vec2((tm + vUV.x) * 0.25, 0.5 + vUV.y * 0.5)).r * uNorm.y : 0.0;
  float tau = vTau * mix(m, f, coarse);
  if (tau < 0.002) discard;
  float a = 1.0 - exp(-tau);
  float d = texture(uDepth, gl_FragCoord.xy / uRes).r;
  float sw = d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
  a *= clamp((sw - vViewZ) / max(0.5 * vSig, 0.15), 0.0, 1.0);
  a *= smoothstep(0.3, 2.0, vViewZ);
  if (a < 0.002) discard;
  vec3 V = normalize(vVPos);
  float mu = dot(V, uSunView);                      // 1: looking toward the sun
  float anti = acos(clamp(-mu, -1.0, 1.0));          // angle from the antisolar point
  // water drops: diffraction + refraction forward lobe, reflection back lobe
  float ph = 0.86 * hgp(mu, 0.85) + 0.14 * hgp(mu, -0.3);
  // rainbows: primary 40.6 (blue) .. 42.3 deg (red), secondary 50.4 (red) .. 53.4 (blue);
  // Airy width ~ d^-2/3; mist (< 50 um) only keeps a broad white fog bow
  float big = smoothstep(3e-5, 2e-4, vD);
  float w = clamp(0.011 * pow(1e-3 / max(vD, 1e-5), 0.6667), 0.005, 0.12);
  vec3 c1 = vec3(0.7383, 0.7243, 0.7086), c2 = vec3(0.8796, 0.9064, 0.9320);
  vec3 b1 = (vec3(anti) - c1) / w, b2 = (vec3(anti) - c2) / (w * 1.4);
  vec3 bow = exp(-b1 * b1) * 0.040 + exp(-b2 * b2) * 0.016;
  bow += vec3(0.010) * (1.0 - smoothstep(c1.g - 0.06, c1.g, anti)) * smoothstep(0.35, 0.6, anti);  // bright inside the bow
  float fb = (anti - 0.70) / 0.10;
  float fog = 0.022 * exp(-fb * fb);
  vec3 phase = vec3(ph) + mix(vec3(fog), bow, big);
  // sun light reaching this part of the parcel through the rest of it (sphere-like parcel)
  vec2 qn = (vUV - 0.5) * 1.6;
  vec3 nv = normalize(vAx * qn.x + vAy * qn.y + vToCam * (sqrt(max(1.0 - dot(qn, qn), 0.0)) + 0.25));
  float toSun = clamp(1.0 - dot(nv, uSunView), 0.0, 2.0);
  float Ts = exp(-vTau * 0.6 * toSun);
  // dense spray: multiple scattering makes it a bright white surface on its sunlit side
  float ms = 1.0 - exp(-0.45 * tau);
  vec3 sun = uSunRad * (Ts * phase + ms * 0.08 * (0.35 + 0.65 * max(dot(nv, uSunView), 0.0)));
  float upn = dot(nv, uUpView);
  vec3 sky = uAmb * (0.55 + 0.35 * upn) * mix(1.0, 0.8, ms);
  vec3 gnd = uGround * (0.5 - 0.5 * upn);
  outColor = vec4((sun + sky + gnd) * a, a);
}`;

export class TireSpray {
  constructor() {
    this.scene = new THREE.Scene();
    const quad = new THREE.PlaneGeometry(1, 1);
    const g = new THREE.InstancedBufferGeometry();
    g.index = quad.index;
    g.setAttribute('position', quad.getAttribute('position'));
    this.iPos = new Float32Array(MAX * 4);
    this.iE = new Float32Array(MAX * 4);
    this.iPar = new Float32Array(MAX * 4);
    this.iV = new Float32Array(MAX * 3);
    g.setAttribute('iPos', new THREE.InstancedBufferAttribute(this.iPos, 4).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('iE', new THREE.InstancedBufferAttribute(this.iE, 4).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('iPar', new THREE.InstancedBufferAttribute(this.iPar, 4).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('iV', new THREE.InstancedBufferAttribute(this.iV, 3).setUsage(THREE.DynamicDrawUsage));
    g.instanceCount = 0;
    this.geo = g;
    this.u = {
      uDepth: { value: null }, uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 },
      uSunView: { value: new THREE.Vector3(0, 1, 0) }, uUpView: { value: new THREE.Vector3(0, 1, 0) },
      uSunRad: { value: new THREE.Color(1, 1, 1) }, uAmb: { value: new THREE.Color(0.2, 0.2, 0.2) },
      uGround: { value: new THREE.Color(0, 0, 0) },
      uAtlas: { value: null }, uNorm: { value: new THREE.Vector2(1, 1) },
    };
    const atlas = makeAtlas();
    this.u.uAtlas.value = atlas.tex;
    // mean of the gaussian exp(-r^2 / 2 sigma^2) over its 5.2 sigma square sprite = 2 pi / 27
    this.u.uNorm.value.set(0.2327 / atlas.mean[0], 0.2327 / atlas.mean[1]);
    this.mat = new THREE.RawShaderMaterial({
      vertexShader: VS, fragmentShader: FS, uniforms: this.u, glslVersion: THREE.GLSL3,
      transparent: true, depthTest: false, depthWrite: false, side: THREE.DoubleSide,
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
      blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
      blendSrcAlpha: THREE.OneFactor, blendDstAlpha: THREE.OneMinusSrcAlphaFactor,
    });
    this.mesh = new THREE.Mesh(g, this.mat);
    this.mesh.frustumCulled = false;
    this.scene.add(this.mesh);
    const F = () => new Float32Array(MAX);
    this.p = {
      x: F(), y: F(), z: F(), vx: F(), vy: F(), vz: F(), ex: F(), ey: F(), ez: F(),
      d: F(), n: F(), sig: F(), age: F(), life: F(), bu: F(), seed: F(), wall: F(),
    };
    this.n = 0;
    this.active = false;
    // per gear: emission remainders and the touchdown spin-up boost
    this.em = [0, 1, 2].map(() => ({ acc: new Float32Array(6), boost: 0, bt: 0, bdur: 0 }));
    this.order = new Uint32Array(MAX);
    this.key = new Uint16Array(MAX);
    this.cnt = new Uint32Array(2049);
    this._u = new THREE.Vector3();
    this._inv = new THREE.Matrix4();
    this._l = new THREE.Vector3();
    this.mass = 0;           // water in the air (kg), for the display
  }

  setSize(w, h) { this.u.uRes.value.set(w, h); }
  clear() { this.n = 0; this.active = false; this.geo.instanceCount = 0; for (const e of this.em) { e.acc.fill(0); e.boost = 0; } }

  /** main / nose gear touching down: the skidding tyres plane on the film while they spin up */
  touchdown(i, dv) {
    const e = this.em[i];
    e.boost = Math.min(2.5, dv / 40);
    e.bt = 0;
    e.bdur = (i > 0 ? 0.1 : 0.08) + (i > 0 ? 0.0028 : 0.002) * dv;
  }

  /** sheet: length of intact liquid sheet / ligament before it tears into drops (m) */
  _spawn(x, y, z, vx, vy, vz, ex, ey, ez, d, mass, sig, sheet = 0) {
    if (this.n >= MAX || mass <= 0) return;
    const P = this.p, k = this.n++;
    P.x[k] = x; P.y[k] = y; P.z[k] = z; P.vx[k] = vx; P.vy[k] = vy; P.vz[k] = vz;
    P.ex[k] = ex; P.ey[k] = ey; P.ez[k] = ez;
    P.d[k] = d; P.n[k] = mass / (RHO_W * 0.5236 * d * d * d); P.sig[k] = sig;
    P.age[k] = 0; P.life[k] = 7 + Math.random() * 6; P.bu[k] = sheet; P.seed[k] = Math.random(); P.wall[k] = 0;
  }

  /**
   * @param {number} dt sim-time step
   * @param {object} s  { wake, h (water depth m), fwd (Vector3, ground-plane heading), vel (Vector3),
   *                      gears: [{ c: Vector3 contact (three world), wow, load (N), spin (tread speed / V) } x3],
   *                      model: Object3D (orbiter, up-to-date matrixWorld) }
   */
  update(dt, s) {
    if (dt <= 0) { this.active = this.n > 0; return; }
    const P = this.p;
    const h = s.h || 0;
    const fwd = s.fwd, rx = -fwd.z, rz = fwd.x;            // right of the rolling direction
    // ---------------- emission
    if (h > 1e-5) {
      for (let i = 0; i < 3; i++) {
        const gr = s.gears[i], e = this.em[i];
        if (!gr || !gr.wow || !gr.c) { e.boost = 0; continue; }
        const V = Math.max(0, s.vel.x * fwd.x + s.vel.z * fwd.z);
        if (V < 2) continue;
        const ty = TYRE[i];
        // footprint: width ~85 % of the section, length from the tyre deflection under the load
        const bc = 0.85 * ty.b;
        const share = i === 0 ? 0.2 : 0.4;
        const Lf = Math.max(0.15, (i === 0 ? 0.3 : 0.45) * Math.sqrt(Math.max(gr.load || 0, 1e3) / (share * 9.79e5)));
        const hs = Math.max(h - TEXTURE_H, 0);
        let spin = Math.min(1, Math.max(0, gr.spin ?? 1));
        let boost = 1;
        if (e.boost > 0) {
          e.bt += dt;
          const u = e.bt / Math.max(e.bdur, 1e-3);
          if (u >= 1) e.boost = 0; else { boost = 1 + e.boost * (1 - u); spin = Math.min(spin, u); }
        }
        // dynamic hydroplaning (share of the load on the water wedge, from the simulator's tyre model;
        // NASA: V_p = 9 sqrt(p) kt): the sheets flatten and widen, the unloaded tyre spins down and
        // stops throwing water off its tread
        const Vp = 9 * Math.sqrt(ty.psi) * 0.5144;
        const hp = gr.hydro ?? Math.min(0.95, ss(1e-3, 3e-3, h) * (V / Vp) ** 2);
        spin = Math.min(spin, 1 - 0.8 * hp);
        const mDisp = RHO_W * hs * bc * V * boost;                       // displaced, kg/s per tyre
        const mTread = RHO_W * Math.min(h, 1.5e-3) * bc * V * 0.3 * spin; // carried round in the grooves
        const dt_e = 1 / EMIT_RATE;
        for (let t = 0; t < 2; t++) {
          const side = t ? 1 : -1;                                       // tyre on the left / right of the axle
          const cx = gr.c.x + rx * ty.half * side, cz = gr.c.z + rz * ty.half * side, cy = gr.c.y;
          // sheets: outer side, inner side (the two inner sheets meet: central plume), tread throw
          for (let k = 0; k < 3; k++) {
            const m = k < 2 ? 0.42 * mDisp : mTread;
            if (m <= 1e-4) continue;
            const slot = t * 3 + k;
            e.acc[slot] += EMIT_RATE * Math.min(1, 0.15 + m / 4) * dt;
            const nE = Math.floor(e.acc[slot]);
            e.acc[slot] -= nE;
            if (!nE) continue;
            const mp = m * dt / nE;
            for (let j = 0; j < nE; j++) {
              const u = (j + Math.random()) / nE;
              const back = V * dt * (1 - u);                              // spread over the frame's travel
              if (k < 2) {
                const outer = k === 0;
                const sd = outer ? side : -side;                          // lateral direction of this sheet
                const U = V * (0.8 + 0.17 * Math.random());
                let th = (0.2 + 0.28 * Math.exp(-V / 35) + 0.12 * hp) + gauss() * 0.06;
                let ph = (0.12 + 0.1 * Math.sqrt(h / 3e-3)) * (0.55 + 0.9 * Math.random()) * (1 - 0.3 * hp);
                if (!outer) { th *= 0.4; ph = ph * 1.8 + 0.12; }
                const ct = Math.cos(th), st = Math.sin(th), cp = Math.cos(ph), sp = Math.sin(ph);
                // tyre frame: backward along the track, outward, up -> ground frame (+ tyre velocity)
                const ux = -ct * cp, ul = st * cp * sd, uy = sp;
                const vx = V * fwd.x + U * (ux * fwd.x + ul * rx);
                const vz = V * fwd.z + U * (ux * fwd.z + ul * rz);
                const ox = (Math.random() - 0.2) * Lf * 0.5, ol = sd * bc * 0.5;
                const px = cx + fwd.x * (ox - back) + rx * ol, pz = cz + fwd.z * (ox - back) + rz * ol;
                // sheet break-up: broad spectrum, mass median ~0.5 mm, finer from faster / thinner sheets
                const d = Math.min(3e-3, Math.max(3e-5, 5e-4 * (h / 2e-3) ** 0.25 * (60 / Math.max(V, 15)) ** 0.3 * Math.exp(0.9 * gauss())));
                // the water leaves as an intact sheet that tears into drops after ~0.4-1.2 m (wave break-up)
                this._spawn(px, cy + 0.03, pz, vx, U * uy, vz, fwd.x * V * dt_e, 0, fwd.z * V * dt_e, d, mp, 0.05 + 0.04 * Math.random(),
                  0.4 + 0.8 * Math.random());
              } else {
                // thrown off the tread at angle b behind the contact
                const bta = 0.15 + 1.2 * Math.random() ** 1.6;
                const sb = Math.sin(bta), cb = Math.cos(bta);
                const Vt = V * spin;
                const lat = gauss() * 0.12 * Vt;
                const vx = V * fwd.x - Vt * cb * fwd.x + rx * lat;
                const vz = V * fwd.z - Vt * cb * fwd.z + rz * lat;
                const px = cx - fwd.x * (ty.r * sb + back) + rx * gauss() * bc * 0.25;
                const pz = cz - fwd.z * (ty.r * sb + back) + rz * gauss() * bc * 0.25;
                const d = Math.min(2e-3, Math.max(3e-5, 4e-4 * Math.exp(0.8 * gauss())));
                this._spawn(px, cy + ty.r * (1 - cb), pz, vx, Vt * sb, vz, fwd.x * V * dt_e, 0, fwd.z * V * dt_e, d, mp, 0.05,
                  0.15 + 0.3 * Math.random());
              }
            }
          }
        }
      }
    }
    // ---------------- parcels
    const nsub = Math.min(4, Math.ceil(dt / 0.012));
    const hh = dt / nsub;
    const wake = s.wake, ua = this._u;
    const model = s.model;
    const inv = model ? this._inv.copy(model.matrixWorld).invert() : null;
    const me = inv ? inv.elements : null;
    const vv = s.vel;
    let mass = 0;
    for (let it = 0; it < nsub; it++) {
      for (let i = 0; i < this.n; i++) {
        if (P.n[i] <= 0) continue;
        const d = P.d[i];
        wake.at(P.x[i], P.y[i], P.z[i], ua);
        // dense spray drags the air along (two-way coupling): with a water mass loading c per m^3 the
        // air inside the parcel moves at the fraction c / (c + rho_a) of the drops' own velocity, so
        // dense sheets penetrate metres while single drops would stop within centimetres
        const sgc = P.sig[i], L = 2.5 * sgc;
        const load = P.n[i] * RHO_W * 0.5236 * d * d * d / (L * L * (Math.hypot(P.ex[i], P.ey[i], P.ez[i]) + L));
        const ent = load / (load + RHO_A);
        ua.x += (P.vx[i] - ua.x) * ent; ua.y += (P.vy[i] - ua.y) * ent; ua.z += (P.vz[i] - ua.z) * ent;
        const rxv = P.vx[i] - ua.x, ryv = P.vy[i] - ua.y, rzv = P.vz[i] - ua.z;
        const vr = Math.hypot(rxv, ryv, rzv);
        if (P.bu[i] > 0) {
          // still an intact sheet / ligament: edge-on to the flow it hardly feels drag - ballistic
          const s0 = Math.hypot(P.vx[i], P.vy[i], P.vz[i]) * hh;
          P.bu[i] -= s0;
          P.x[i] += P.vx[i] * hh; P.y[i] += (P.vy[i] - 0.5 * G * hh) * hh; P.z[i] += P.vz[i] * hh;
          P.vy[i] -= G * hh;
          P.age[i] += hh;
          if (P.y[i] < wake.groundAt(P.x[i], P.z[i])) P.bu[i] = 0;
          continue;
        }
        const Re = Math.max(RHO_A * vr * d / MU_A, 1e-3);
        const We = RHO_A * vr * vr * d / SIG_W;
        // sphere drag (Schiller-Naumann), more for a drop flattened by the airflow
        const Cd = (Re < 1000 ? 24 / Re * (1 + 0.15 * Re ** 0.687) : 0.44) * (1 + 1.2 * Math.min(We / 12, 1));
        const tau = (4 / 3) * RHO_W * d / (RHO_A * Cd * Math.max(vr, 1e-3));
        const ex = Math.exp(-hh / tau), gt = G * tau * (1 - ex);
        const nvx = ua.x + rxv * ex, nvy = ua.y + ryv * ex - gt, nvz = ua.z + rzv * ex;
        P.x[i] += 0.5 * (P.vx[i] + nvx) * hh; P.y[i] += 0.5 * (P.vy[i] + nvy) * hh; P.z[i] += 0.5 * (P.vz[i] + nvz) * hh;
        P.vx[i] = nvx; P.vy[i] = nvy; P.vz[i] = nvz;
        // secondary break-up (bag / multimode) above the critical Weber number: over the break-up time
        // (~5 d / v_rel sqrt(rho_w / rho_a)) the drops go to fragments of ~1/4 the size (not below the
        // size that is stable at this relative speed); the parcel keeps its water
        if (We > 12) {
          const tb = 5 * d / Math.max(vr, 1e-3) * 28.9;
          const dt_ = Math.max(d * 0.25, 1.2 * 12 * SIG_W / (RHO_A * vr * vr), 8e-6);
          if (dt_ < d) {
            const dn = d + (dt_ - d) * (1 - Math.exp(-hh / tb));
            P.n[i] *= (d / dn) ** 3; P.d[i] = dn;
            P.sig[i] += (0.03 + 0.004 * vr) * (d - dn) / d;      // the fragments spread
          }
        }
        // turbulent diffusion: strong in the wake behind the vehicle, weak in the open air
        const dxv = P.x[i] - wake.pos.x, dzv = P.z[i] - wake.pos.z;
        const r2 = dxv * dxv + dzv * dzv;
        // inertial drops follow the eddies less (Lagrangian time scale ~0.3 s)
        const K = (0.1 + 1.6 * Math.min(wake.V / 80, 1.5) * Math.exp(-r2 / 900)) / (1 + Math.min(tau, 5) / 0.3);
        const sg = P.sig[i];
        P.sig[i] = Math.sqrt(sg * sg + 2 * K * hh);
        // the runway
        const gy = wake.groundAt(P.x[i], P.z[i]);
        if (P.d[i] > 8e-5) {
          if (P.y[i] < gy && P.vy[i] < 0) {
            // drop impact on the film: merge with it, or splash a part back up (Mundo et al.)
            const vn = -P.vy[i], dd = P.d[i];
            const K_s = Math.sqrt(RHO_W * vn * vn * dd / SIG_W) * Math.pow(RHO_W * vn * dd / 1e-3, 0.25);
            if (K_s > 57.7) {
              const f = 0.25, dn = dd * 0.3;
              P.n[i] *= f * (dd / dn) ** 3; P.d[i] = dn;
              P.vy[i] = vn * 0.18; P.vx[i] *= 0.6; P.vz[i] *= 0.6;
              P.y[i] = gy + 0.02;
            } else { P.n[i] = 0; continue; }
          }
        } else if (P.y[i] < gy + 0.3 * P.sig[i]) {
          // mist sits on the runway and spreads along it
          P.y[i] = gy + 0.3 * P.sig[i];
          if (P.vy[i] < 0) P.vy[i] = 0;
        }
        // the underside of the wings / fuselage (orbiter frame: x fwd, y up, z right)
        if (me && P.y[i] - gy < 6) {
          const x = P.x[i], y = P.y[i], z = P.z[i];
          const lx = me[0] * x + me[4] * y + me[8] * z + me[12];
          const ly = me[1] * x + me[5] * y + me[9] * z + me[13];
          const lz = me[2] * x + me[6] * y + me[10] * z + me[14];
          const az = Math.abs(lz);
          if (az < 11.9 && lx < 12 && lx > -15) {
            const ceil = az < 2.6 ? -2.3 : -2.25 + (az - 2.6) * 0.061;
            // wing planform: leading edge 45 deg outboard of the glove, trailing edge -13.9 .. -12.2
            const xle = az < 4.72 ? 11.2 - (az - 2.6) * 6.005 : -1.53 - (az - 4.72) * 0.9986;
            const xte = -13.86 + (az - 2.6) * 0.1785;
            const under = az < 2.6 ? lx < 11 && lx > -14.5 : lx < xle && lx > xte;
            if (under && ly > ceil && ly < ceil + 0.6) {
              // hits the airframe: part sticks to the skin (once), the rest runs along it with the airflow
              if (!P.wall[i]) P.n[i] *= 0.6;
              P.wall[i] = 1;
              const vy0 = P.vy[i] - vv.y;
              if (vy0 > 0) P.vy[i] = vv.y;
              P.vx[i] = vv.x + (P.vx[i] - vv.x) * 0.5; P.vz[i] = vv.z + (P.vz[i] - vv.z) * 0.5;
              P.y[i] -= ly - ceil;                               // back onto the skin (rollout: ~level)
            }
          }
        }
        P.age[i] += hh;
      }
    }
    // compact: drop deposited / dissolved / old parcels
    let w = 0;
    for (let i = 0; i < this.n; i++) {
      const tau = P.n[i] * P.d[i] * P.d[i] / (4 * P.sig[i] * P.sig[i]);
      if (P.n[i] <= 0 || P.age[i] > P.life[i] || (tau < 0.002 && P.age[i] > 0.4)) continue;
      if (w !== i) for (const k in P) P[k][w] = P[k][i];
      mass += P.n[w] * RHO_W * 0.5236 * P.d[w] ** 3;
      w++;
    }
    this.n = w;
    this.mass = mass;
    this.active = w > 0;
  }

  /** draw into the premultiplied particle layer rt (back to front), clear it first if asked;
   *  shutter: camera exposure time (s of real time) for the motion blur, ts: sim time scale */
  render(renderer, camera, depthTexture, light, rt, clear, camVel, ts, shutter) {
    if (!this.active || !rt) return false;
    const P = this.p, n = this.n;
    const cp = camera.position;
    // back to front: counting sort on log distance
    const cnt = this.cnt; cnt.fill(0);
    const key = this.key;
    for (let k = 0; k < n; k++) {
      const dx = P.x[k] - cp.x, dy = P.y[k] - cp.y, dz = P.z[k] - cp.z;
      const q = Math.min(2047, Math.max(0, Math.floor((Math.log2(dx * dx + dy * dy + dz * dz + 1) * 0.5) * 120)));
      key[k] = 2047 - q;
      cnt[key[k] + 1]++;
    }
    for (let b = 1; b < 2049; b++) cnt[b] += cnt[b - 1];
    for (let k = 0; k < n; k++) this.order[cnt[key[k]]++] = k;
    const sh = shutter * ts;
    for (let j = 0; j < n; j++) {
      const k = this.order[j];
      const o = j * 4;
      this.iPos[o] = P.x[k]; this.iPos[o + 1] = P.y[k]; this.iPos[o + 2] = P.z[k]; this.iPos[o + 3] = P.sig[k];
      // the stretch of track the parcel's water was laid along (it stays laid out; diffusion widens it)
      this.iE[o] = P.ex[k]; this.iE[o + 1] = P.ey[k]; this.iE[o + 2] = P.ez[k]; this.iE[o + 3] = P.d[k];
      this.iPar[o] = P.n[k] * P.d[k] * P.d[k] / 4 * Math.min(1, P.age[k] / 0.03 + 0.3) * (1 - (P.age[k] / P.life[k]) ** 3);
      this.iPar[o + 1] = P.seed[k]; this.iPar[o + 2] = P.age[k]; this.iPar[o + 3] = P.age[k] / P.life[k];
      const t = Math.min(sh, P.age[k]);
      this.iV[j * 3] = (P.vx[k] - camVel.x) * t; this.iV[j * 3 + 1] = (P.vy[k] - camVel.y) * t; this.iV[j * 3 + 2] = (P.vz[k] - camVel.z) * t;
    }
    this.geo.instanceCount = n;
    for (const a of ['iPos', 'iE', 'iPar', 'iV']) this.geo.attributes[a].needsUpdate = true;
    const u = this.u, view = camera.matrixWorldInverse;
    u.uDepth.value = depthTexture;
    u.uLogFar.value = Math.log2(camera.far + 1.0);
    u.uSunView.value.copy(light.sunDir).transformDirection(view);
    u.uUpView.value.set(0, 1, 0).transformDirection(view);
    u.uSunRad.value.copy(light.sunRad);
    u.uAmb.value.copy(light.amb);
    const sy = Math.max(light.sunDir.y, 0);
    u.uGround.value.copy(light.sunRad).multiplyScalar(0.12 * sy * 0.3);
    u.uGround.value.r += light.amb.r * 0.2; u.uGround.value.g += light.amb.g * 0.2; u.uGround.value.b += light.amb.b * 0.2;
    const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
    renderer.setRenderTarget(rt);
    if (clear) { renderer.setClearColor(0x000000, 0); renderer.clear(true, false, false); }
    renderer.render(this.scene, camera);
    renderer.setClearColor(cc, ca);
    return true;
  }
}
