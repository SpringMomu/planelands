// Tyre smoke: burnt rubber from the touchdown spin-up of the main tyres (and the nose tyres at
// slap-down).  CPU particles drawn as camera-facing soft volumetric puffs into their own premultiplied
// HDR layer with a soft depth test against the scene depth buffer; the composite pass lays the layer
// over the image after the clouds (so the smoke also shows against the sky).
//
// Physics, roughly as seen on touchdown footage:
// * every tyre (two per gear) skids while it spins up, ~0.1 s + ~2.5 ms per m/s of slip; the vehicle
//   moves on meanwhile, so each tyre lays a short streak of smoke along the runway, densest at first
//   contact.  The amount follows the slip energy (main.js: (slip speed)^2, wet runway much less)
// * the smoke leaves the contact patch with part of the vehicle speed (entrained air), stops in the
//   air within a second, rises while it is hot, is rolled outward by the downwash / wing-tip flow
//   close to the ground, wanders with the turbulence and drifts with the wind
// * puffs grow by turbulent diffusion (size^2 ~ t), their column density falls accordingly, the
//   edges fray into wisps as they dilute
// * fine particles: strong forward scattering (bright against the sun), dense cores self-shadow and
//   look grey, thin parts slightly blue-white; lit by sun, sky and the runway below
// * the same puffs carry the dust ground off where the airframe scrapes the runway (sparks.js): silica
//   tile fibres, concrete and aluminium fines, grey-beige, barely buoyant
import * as THREE from 'three';
import { makeRT } from '../render/pass.js';

const MAX = 4000;

const VS = /* glsl */ `
precision highp float;
in vec3 position;
in vec4 iPos;      // xyz world position, w size (m)
in vec4 iPar;      // x optical density, y rotation, z seed, w age (s)
in vec4 iAux;      // x age / life, y stretch along the screen-space motion, z/w stretch direction
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
out vec2 vQ; out vec2 vQR; out float vDens; out float vSeed; out float vAge; out float vAgeN;
out float vViewZ; out float vSize; out vec3 vVPos;
void main(){
  vec2 q = position.xy * 2.0;
  float c = cos(iPar.y), s = sin(iPar.y);
  vec2 qr = vec2(c * q.x - s * q.y, s * q.x + c * q.y);
  vec3 right = vec3(viewMatrix[0][0], viewMatrix[1][0], viewMatrix[2][0]);
  vec3 up = vec3(viewMatrix[0][1], viewMatrix[1][1], viewMatrix[2][1]);
  // young puffs are drawn out along their motion (the streak laid by the rolling tyre)
  vec2 sd = iAux.zw;
  vec2 qs = q + sd * dot(q, sd) * iAux.y;
  vec3 wp = iPos.xyz + (right * qs.x + up * qs.y) * iPos.w * 0.5;
  vec4 vp = viewMatrix * vec4(wp, 1.0);
  vQ = q; vQR = qr; vDens = iPar.x; vSeed = iPar.z; vAge = iPar.w; vAgeN = iAux.x; vSize = iPos.w;
  vViewZ = -vp.z; vVPos = vp.xyz;
  gl_Position = projectionMatrix * vp;
}`;

const FS = /* glsl */ `
precision highp float;
uniform sampler2D uDepth;
uniform vec2 uRes;
uniform float uLogFar;
uniform vec3 uSunView;     // direction toward the sun (view space)
uniform vec3 uUpView;      // world up (view space)
uniform vec3 uSunRad;
uniform vec3 uAmb;         // sky light
uniform vec3 uGround;      // light bounced off the runway
in vec2 vQ; in vec2 vQR; in float vDens; in float vSeed; in float vAge; in float vAgeN;
in float vViewZ; in float vSize; in vec3 vVPos;
out vec4 outColor;
float h21(vec2 p){ p = fract(p * vec2(123.34, 456.21)); p += dot(p, p + 45.32); return fract(p.x * p.y); }
float vn(vec2 p){
  vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p){ float s = 0.0, a = 0.5; for (int i = 0; i < 5; i++){ s += a * vn(p); p = p * 2.03 + 17.1; a *= 0.5; } return s / 0.97; }
float hgp(float c, float g){ float g2 = g * g; return (1.0 - g2) / (12.566 * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5)); }
void main(){
  float r2 = dot(vQ, vQ);
  if (r2 > 1.0) discard;
  float dust = step(1.5, vSeed);                          // seed + 2: abrasion dust instead of rubber smoke
  float seed = vSeed - 2.0 * dust;
  // billowing structure: domain-warped fbm turning over slowly, plus rounded "cauliflower" lobes
  vec2 p = vQR * 1.5 + seed * 31.7;
  vec2 warp = vec2(fbm(p + vec2(0.0, vAge * 0.35)), fbm(p + vec2(5.2, -vAge * 0.28)));
  float n = fbm(p + warp * 1.4 + vec2(vAge * 0.12, vAge * 0.45));
  float lobes = 1.0 - abs(2.0 * fbm(p * 2.2 + warp * 0.8 - vAge * 0.2) - 1.0);
  float edge = sqrt(r2) + (n - 0.5) * 0.85 - (lobes - 0.5) * 0.25;
  float dens = (1.0 - smoothstep(0.1, 1.0, edge)) * (0.5 + 0.7 * n + 0.3 * lobes);
  // diluting smoke frays into wisps
  dens *= mix(1.0, smoothstep(0.2 + 0.45 * vAgeN, 0.75, n * 0.8 + lobes * 0.35), smoothstep(0.1, 0.8, vAgeN));
  float tau = dens * vDens * 2.4;                         // optical depth through the puff
  float a = 1.0 - exp(-tau);
  float d = texture(uDepth, gl_FragCoord.xy / uRes).r;
  float sw = d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
  a *= clamp((sw - vViewZ) / max(0.35 * vSize, 0.3), 0.0, 1.0);    // soft intersection with the ground / airframe
  a *= smoothstep(0.4, 2.5, vViewZ);                                // don't fill the screen when the camera is inside
  if (a < 0.002) discard;
  // lighting: sphere-like puff, sunlight attenuated on its way through it
  vec3 nv = normalize(vec3(vQ, sqrt(max(1.0 - r2, 0.0)) + 0.25));
  float toSun = clamp(1.0 - dot(nv, uSunView), 0.0, 2.0);         // ~path toward the sun inside the puff
  float Ts = exp(-vDens * 2.4 * toSun * 0.75);
  vec3 V = normalize(vVPos);
  float mu = dot(V, uSunView);                                      // 1 = looking toward the sun
  float ph = mix(hgp(mu, 0.72), hgp(mu, -0.25), 0.25);              // fine particles: strong forward lobe
  float powder = 1.0 - exp(-tau * 2.0);
  vec3 sun = uSunRad * (Ts * ph * 2.4 * (0.55 + 0.45 * powder) + (0.5 + 0.5 * dot(nv, uSunView)) * exp(-vDens * 0.9) * 0.12);
  float upn = dot(nv, uUpView);
  vec3 sky = uAmb * (0.7 + 0.3 * upn) * mix(1.0, 0.65, smoothstep(0.3, 1.5, tau));
  vec3 gnd = uGround * (0.5 - 0.5 * upn);
  // thin smoke slightly blue-white, dense core grey (some soot)
  vec3 alb = mix(vec3(0.86, 0.88, 0.93), vec3(0.62, 0.62, 0.63), smoothstep(0.3, 1.4, tau));
  alb = mix(alb, mix(vec3(0.50, 0.47, 0.43), vec3(0.34, 0.32, 0.30), smoothstep(0.3, 1.4, tau)), dust);
  vec3 col = alb * (sun + sky + gnd);
  outColor = vec4(col * a, a);
}`;

const gauss = () => (Math.random() + Math.random() + Math.random() - 1.5) * 2.0;   // ~N(0, 1)

export class TireSmoke {
  constructor() {
    this.scene = new THREE.Scene();
    const quad = new THREE.PlaneGeometry(1, 1);
    const g = new THREE.InstancedBufferGeometry();
    g.index = quad.index;
    g.setAttribute('position', quad.getAttribute('position'));
    this.iPos = new Float32Array(MAX * 4);
    this.iPar = new Float32Array(MAX * 4);
    this.iAux = new Float32Array(MAX * 4);
    g.setAttribute('iPos', new THREE.InstancedBufferAttribute(this.iPos, 4).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('iPar', new THREE.InstancedBufferAttribute(this.iPar, 4).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('iAux', new THREE.InstancedBufferAttribute(this.iAux, 4).setUsage(THREE.DynamicDrawUsage));
    g.instanceCount = 0;
    this.geo = g;
    this.u = {
      uDepth: { value: null }, uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 },
      uSunView: { value: new THREE.Vector3(0, 1, 0) }, uUpView: { value: new THREE.Vector3(0, 1, 0) },
      uSunRad: { value: new THREE.Color(1, 1, 1) }, uAmb: { value: new THREE.Color(0.2, 0.2, 0.2) },
      uGround: { value: new THREE.Color(0, 0, 0) },
    };
    this.mat = new THREE.RawShaderMaterial({
      vertexShader: VS, fragmentShader: FS, uniforms: this.u, glslVersion: THREE.GLSL3,
      transparent: true, depthTest: false, depthWrite: false,
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
      blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
      blendSrcAlpha: THREE.OneFactor, blendDstAlpha: THREE.OneMinusSrcAlphaFactor,
    });
    this.mesh = new THREE.Mesh(g, this.mat);
    this.mesh.frustumCulled = false;
    this.scene.add(this.mesh);
    // particle state (structure of arrays)
    const F = () => new Float32Array(MAX);
    this.p = {
      x: F(), y: F(), z: F(), vx: F(), vy: F(), vz: F(), tx: F(), ty: F(), tz: F(), age: F(), life: F(),
      s0: F(), k: F(), a0: F(), heat: F(), rot: F(), rotv: F(), seed: F(),
    };
    this.n = 0;
    this.emit = [0, 1, 2].map(() => ({ t: 1e9, dur: 0, n: 0, str: 0, last: null, acc: 0 }));
    this.order = new Uint16Array(MAX);
    this.key = new Float32Array(MAX);
    this.rt = null;
    this.active = false;
    this._side = new THREE.Vector3();
    this._out = new THREE.Vector3();
    this._mid = new THREE.Vector3();
    this._v = new THREE.Vector3();
    this._sv = new THREE.Vector3();
  }

  setSize(w, h) { this.rt?.dispose(); this.rt = makeRT(w, h); this.u.uRes.value.set(w, h); }
  get texture() { return this.rt ? this.rt.texture : null; }
  clear() { this.n = 0; for (const e of this.emit) { e.t = 1e9; e.last = null; e.acc = 0; } this.geo.instanceCount = 0; this.active = false; }

  /** spin-up skid at gear i (0 nose, 1/2 mains): strength 0..1.2 (slip energy), dv slip speed (m/s) */
  touchdown(i, strength, dv = 90) {
    const e = this.emit[i];
    const main = i > 0;
    e.t = 0;
    e.dur = (main ? 0.1 : 0.08) + (main ? 0.0028 : 0.002) * dv;     // spin-up time
    e.n = (main ? 260 : 110) * strength;                            // particles per tyre for the whole skid
    e.str = Math.min(1.2, strength);
    e.acc = 0;
  }

  _spawn(px, py, pz, s, str, main, out, u) {
    if (this.n >= MAX) return;
    const P = this.p, k = this.n++;
    const r = Math.random, vel = s.vel, wind = s.wind;
    P.x[k] = px + gauss() * 0.12; P.y[k] = py + 0.15 + r() * 0.25; P.z[k] = pz + gauss() * 0.12;
    // entrained air carries the fresh smoke along at part of the vehicle speed; it is thrown up off the
    // tyre and, for the mains, rolled outward by the downwash spreading under the wing
    const f = 0.18 + r() * 0.22;
    const ow = main ? (0.6 + r() * 2.2) : (r() - 0.5) * 1.2;
    P.vx[k] = wind.x + (vel.x - wind.x) * f + out.x * ow + gauss() * 1.2;
    P.vy[k] = 0.8 + r() * 2.2;
    P.vz[k] = wind.z + (vel.z - wind.z) * f + out.z * ow + gauss() * 1.2;
    P.tx[k] = gauss() * 0.6; P.ty[k] = gauss() * 0.3; P.tz[k] = gauss() * 0.6;
    P.age[k] = 0;
    P.life[k] = (main ? 5.0 : 3.5) + r() * 3.0;
    P.s0[k] = (main ? 0.55 : 0.4) + r() * 0.35;
    P.k[k] = (main ? 14 : 8) * (0.7 + r() * 0.6);                    // size^2 growth (m^2/s)
    // the first contact smokes hardest (hottest, fastest slip)
    P.a0[k] = (0.55 + r() * 0.45) * str * (1.15 - 0.6 * u);
    P.heat[k] = (0.6 + r() * 0.6) * (1.1 - 0.5 * u);
    P.rot[k] = r() * 6.283; P.rotv[k] = gauss() * 0.35;
    P.seed[k] = r();
  }

  /**
   * a puff of abrasion dust where the airframe grinds on the runway (three world position, velocity m/s)
   * @param {number} a0 initial column density (0..1)
   */
  dust(px, py, pz, vx, vy, vz, a0) {
    if (this.n >= MAX) return;
    const P = this.p, k = this.n++;
    const r = Math.random;
    P.x[k] = px + gauss() * 0.15; P.y[k] = py + 0.1 + r() * 0.2; P.z[k] = pz + gauss() * 0.15;
    P.vx[k] = vx; P.vy[k] = vy; P.vz[k] = vz;
    P.tx[k] = gauss() * 0.6; P.ty[k] = gauss() * 0.3; P.tz[k] = gauss() * 0.6;
    P.age[k] = 0;
    P.life[k] = 3.5 + r() * 3.5;
    P.s0[k] = 0.35 + r() * 0.3;
    P.k[k] = 7 * (0.7 + r() * 0.6);
    P.a0[k] = a0 * (0.6 + 0.4 * r());
    P.heat[k] = 0.15 + r() * 0.25;                // friction-heated, but mineral dust is heavy
    P.rot[k] = r() * 6.283; P.rotv[k] = gauss() * 0.35;
    P.seed[k] = 2 + r() * 0.999;
    this.active = true;
  }

  /**
   * @param {number} dt  sim-time step (s)
   * @param {object} s   { contacts: [Vector3 x3] (three world wheel contact points), vel: Vector3, wind: Vector3,
   *                       groundY(x, z), brakes (0..1), gs (m/s), wow: [bool x3] }
   */
  update(dt, s) {
    const P = this.p;
    // emitters: two tyres per gear, spread across the axle
    if (s && s.contacts && s.vel) {
      const side = this._side.set(-s.vel.z, 0, s.vel.x);
      if (side.lengthSq() < 1e-6) side.set(1, 0, 0); else side.normalize();
      const c1 = s.contacts[1], c2 = s.contacts[2];
      const mid = c1 && c2 ? this._mid.copy(c1).add(c2).multiplyScalar(0.5) : null;
      for (let i = 0; i < 3; i++) {
        const e = this.emit[i], c = s.contacts[i];
        if (!c) continue;
        const main = i > 0;
        if (e.last && e.last.distanceToSquared(c) > 2500) e.last.copy(c);      // teleport (reset / tab was hidden)
        if (e.t < e.dur && dt > 0 && e.last) {
          // emission profile over the skid: sharp onset, then tailing off as the slip vanishes
          const u0 = e.t / e.dur, u1 = Math.min((e.t + dt) / e.dur, 1);
          const cum = (u) => 1 - (1 - u) ** 2.2;
          e.acc += e.n * (cum(u1) - cum(u0));
          const m = Math.floor(e.acc);
          e.acc -= m;
          const out = this._out.copy(side);
          if (main && mid && this._v.copy(c).sub(mid).dot(side) < 0) out.negate();
          const half = main ? 0.45 : 0.3;
          for (let k = 0; k < m; k++) {
            const u = (k + Math.random()) / Math.max(m, 1);
            const tyre = k % 2 ? half : -half;
            const ux = u0 + (u1 - u0) * u;
            this._spawn(e.last.x + (c.x - e.last.x) * u + side.x * tyre, e.last.y + (c.y - e.last.y) * u,
              e.last.z + (c.z - e.last.z) * u + side.z * tyre, s, e.str, main, out, ux);
          }
          e.t += dt;
        }
        if (!e.last) e.last = c.clone(); else e.last.copy(c);
      }
    }
    // integrate + compact
    let w = 0;
    const wx = s && s.wind ? s.wind.x : 0, wz = s && s.wind ? s.wind.z : 0;
    const tdec = Math.exp(-dt / 0.8);
    const tamp = Math.sqrt(Math.max(1 - tdec * tdec, 0));
    for (let k = 0; k < this.n; k++) {
      const age = P.age[k] + dt;
      if (age >= P.life[k]) continue;
      // mean motion relaxes to the wind (entrained air slows down by turbulent mixing)
      const kd = Math.exp(-dt / (0.35 + age * 0.45));
      const vx = wx + (P.vx[k] - wx) * kd, vz = wz + (P.vz[k] - wz) * kd;
      // hot smoke rises, cools by mixing
      const heat = P.heat[k] * Math.exp(-dt / 1.6);
      let vy = P.vy[k] * Math.exp(-dt / 0.9) + heat * 0.9 * dt;
      // turbulent wander (Ornstein-Uhlenbeck), growing with the puff
      const sig = 0.8 + Math.min(age, 4) * 0.25;
      const tx = P.tx[k] * tdec + sig * tamp * gauss(), tz = P.tz[k] * tdec + sig * tamp * gauss();
      const ty = P.ty[k] * tdec + sig * 0.4 * tamp * gauss();
      let x = P.x[k] + (vx + tx) * dt, y = P.y[k] + (vy + ty) * dt, z = P.z[k] + (vz + tz) * dt;
      const size = Math.sqrt(P.s0[k] * P.s0[k] + P.k[k] * age);
      if (s && s.groundY) { const gy = s.groundY(x, z) + size * 0.25; if (y < gy) { y = gy; vy = Math.max(vy, 0); } }
      if (w !== k) for (const key in P) P[key][w] = P[key][k];
      P.x[w] = x; P.y[w] = y; P.z[w] = z; P.vx[w] = vx; P.vy[w] = vy; P.vz[w] = vz;
      P.tx[w] = tx; P.ty[w] = ty; P.tz[w] = tz; P.heat[w] = heat; P.age[w] = age;
      P.rot[w] += P.rotv[w] * dt;
      w++;
    }
    this.n = w;
    this.active = w > 0;
  }

  render(renderer, camera, depthTexture, light) {
    if (!this.active || !this.rt) return false;
    const P = this.p, n = this.n;
    // back-to-front
    const cp = camera.position;
    for (let k = 0; k < n; k++) { this.order[k] = k; const dx = P.x[k] - cp.x, dy = P.y[k] - cp.y, dz = P.z[k] - cp.z; this.key[k] = dx * dx + dy * dy + dz * dz; }
    const ord = Array.from(this.order.subarray(0, n)).sort((a, b) => this.key[b] - this.key[a]);
    const view = camera.matrixWorldInverse;
    const sv = this._sv;
    for (let j = 0; j < n; j++) {
      const k = ord[j];
      const age = P.age[k], life = P.life[k];
      const size = Math.sqrt(P.s0[k] * P.s0[k] + P.k[k] * age);
      // column density ~ mass / area: falls as the puff spreads; fades out at the end of its life
      const dens = P.a0[k] * Math.min(1, age / 0.05 + 0.15) * Math.pow(P.s0[k] / size, 1.25) * 1.2
        * (1 - Math.pow(age / life, 2.5));
      // stretch along the (screen-space) motion while the puff is young and fast
      sv.set(P.vx[k], P.vy[k], P.vz[k]).transformDirection(view);
      const spd = Math.hypot(P.vx[k], P.vy[k], P.vz[k]);
      const hl = Math.hypot(sv.x, sv.y) || 1;
      const st = Math.min(1.5, spd * 0.06) * Math.exp(-age * 1.5);
      this.iPos[j * 4] = P.x[k]; this.iPos[j * 4 + 1] = P.y[k]; this.iPos[j * 4 + 2] = P.z[k]; this.iPos[j * 4 + 3] = size;
      this.iPar[j * 4] = dens; this.iPar[j * 4 + 1] = P.rot[k]; this.iPar[j * 4 + 2] = P.seed[k]; this.iPar[j * 4 + 3] = age;
      this.iAux[j * 4] = age / life; this.iAux[j * 4 + 1] = st; this.iAux[j * 4 + 2] = sv.x / hl; this.iAux[j * 4 + 3] = sv.y / hl;
    }
    this.geo.instanceCount = n;
    this.geo.attributes.iPos.needsUpdate = true;
    this.geo.attributes.iPar.needsUpdate = true;
    this.geo.attributes.iAux.needsUpdate = true;
    const u = this.u;
    u.uDepth.value = depthTexture;
    u.uLogFar.value = Math.log2(camera.far + 1.0);
    u.uSunView.value.copy(light.sunDir).transformDirection(view);
    u.uUpView.value.set(0, 1, 0).transformDirection(view);
    u.uSunRad.value.copy(light.sunRad);
    u.uAmb.value.copy(light.amb);
    // runway / grass bounce: albedo ~0.2 of the sunlit ground plus the sky light reflected
    const sy = Math.max(light.sunDir.y, 0);
    u.uGround.value.copy(light.sunRad).multiplyScalar(0.2 * sy * 0.3).add(this._c(light.amb, 0.25));
    const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
    renderer.setRenderTarget(this.rt);
    renderer.setClearColor(0x000000, 0);
    renderer.clear(true, false, false);
    renderer.render(this.scene, camera);
    renderer.setClearColor(cc, ca);
    return true;
  }

  _c(col, f) { return (this._tmpC || (this._tmpC = new THREE.Color())).copy(col).multiplyScalar(f); }
}
