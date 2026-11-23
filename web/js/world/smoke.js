// Tyre smoke: burnt-rubber puffs when the wheels spin up at touchdown (main gear, nose-gear slap-down)
// plus a few wisps under heavy braking.  CPU particles drawn as camera-facing soft puffs into their own
// premultiplied HDR layer with a soft depth test against the scene depth buffer; the composite pass
// lays the layer over the image after the clouds (so the smoke also shows against the sky).
import * as THREE from 'three';
import { makeRT } from '../render/pass.js';

const MAX = 1800;

const VS = /* glsl */ `
precision highp float;
in vec3 position;
in vec4 iPos;      // xyz world position, w size (m)
in vec4 iPar;      // x alpha, y rotation, z seed, w age (s)
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
out vec2 vQ; out vec2 vQR; out float vAlpha; out float vSeed; out float vAge; out float vViewZ; out float vSize;
void main(){
  vec2 q = position.xy * 2.0;
  float c = cos(iPar.y), s = sin(iPar.y);
  vec2 qr = vec2(c * q.x - s * q.y, s * q.x + c * q.y);
  vec3 right = vec3(viewMatrix[0][0], viewMatrix[1][0], viewMatrix[2][0]);
  vec3 up = vec3(viewMatrix[0][1], viewMatrix[1][1], viewMatrix[2][1]);
  vec3 wp = iPos.xyz + (right * q.x + up * q.y) * iPos.w * 0.5;
  vec4 vp = viewMatrix * vec4(wp, 1.0);
  vQ = q; vQR = qr; vAlpha = iPar.x; vSeed = iPar.z; vAge = iPar.w; vSize = iPos.w;
  vViewZ = -vp.z;
  gl_Position = projectionMatrix * vp;
}`;

const FS = /* glsl */ `
precision highp float;
uniform sampler2D uDepth;
uniform vec2 uRes;
uniform float uLogFar;
uniform vec3 uSunView;
uniform vec3 uSunRad;
uniform vec3 uAmb;
in vec2 vQ; in vec2 vQR; in float vAlpha; in float vSeed; in float vAge; in float vViewZ; in float vSize;
out vec4 outColor;
float h21(vec2 p){ p = fract(p * vec2(123.34, 456.21)); p += dot(p, p + 45.32); return fract(p.x * p.y); }
float vn(vec2 p){
  vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p){ float s = 0.0, a = 0.5; for (int i = 0; i < 4; i++){ s += a * vn(p); p = p * 2.03 + 17.1; a *= 0.5; } return s; }
void main(){
  float r2 = dot(vQ, vQ);
  if (r2 > 1.0) discard;
  float n = fbm(vQR * 1.7 + vSeed * 31.7 + vec2(0.0, vAge * 0.9));
  float shape = 1.0 - smoothstep(0.0, 1.0, sqrt(r2) + (n - 0.5) * 0.7);
  float a = shape * vAlpha * (0.45 + 0.75 * n);
  float d = texture(uDepth, gl_FragCoord.xy / uRes).r;
  float sw = d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
  a *= clamp((sw - vViewZ) / max(0.35 * vSize, 0.3), 0.0, 1.0);    // soft intersection with the ground / airframe
  a *= smoothstep(0.4, 2.5, vViewZ);                                // don't fill the screen when the camera is inside
  if (a < 0.002) discard;
  vec3 nv = normalize(vec3(vQ, sqrt(max(1.0 - r2, 0.0)) + 0.35));
  float diff = clamp(dot(nv, uSunView) * 0.5 + 0.5, 0.0, 1.0);
  float mu = -uSunView.z;                                            // view ray ~ -Z in view space
  float g = 0.55;
  float ph = (1.0 - g * g) / (12.566 * pow(1.0 + g * g - 2.0 * g * mu, 1.5));
  vec3 alb = vec3(0.74, 0.75, 0.78) - 0.08 * n;
  vec3 col = alb * (uSunRad * (diff * 0.30 + ph * 0.55) + uAmb);
  outColor = vec4(col * a, a);
}`;

export class TireSmoke {
  constructor() {
    this.scene = new THREE.Scene();
    const quad = new THREE.PlaneGeometry(1, 1);
    const g = new THREE.InstancedBufferGeometry();
    g.index = quad.index;
    g.setAttribute('position', quad.getAttribute('position'));
    this.iPos = new Float32Array(MAX * 4);
    this.iPar = new Float32Array(MAX * 4);
    g.setAttribute('iPos', new THREE.InstancedBufferAttribute(this.iPos, 4).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('iPar', new THREE.InstancedBufferAttribute(this.iPar, 4).setUsage(THREE.DynamicDrawUsage));
    g.instanceCount = 0;
    this.geo = g;
    this.u = {
      uDepth: { value: null }, uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 },
      uSunView: { value: new THREE.Vector3(0, 1, 0) }, uSunRad: { value: new THREE.Color(1, 1, 1) }, uAmb: { value: new THREE.Color(0.2, 0.2, 0.2) },
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
    this.p = { x: F(), y: F(), z: F(), vx: F(), vy: F(), vz: F(), age: F(), life: F(), s0: F(), s1: F(), a0: F(), rot: F(), rotv: F(), seed: F() };
    this.n = 0;
    this.emit = [0, 1, 2].map(() => ({ t: 1e9, dur: 0, rate: 0, str: 0, last: null }));
    this.order = new Uint16Array(MAX);
    this.key = new Float32Array(MAX);
    this.rt = null;
    this.active = false;
  }

  setSize(w, h) { this.rt?.dispose(); this.rt = makeRT(w, h); this.u.uRes.value.set(w, h); }
  get texture() { return this.rt ? this.rt.texture : null; }
  clear() { this.n = 0; for (const e of this.emit) { e.t = 1e9; e.last = null; } this.geo.instanceCount = 0; this.active = false; }

  /** start a burst at wheel i (0 nose, 1/2 mains); strength 0..1 */
  touchdown(i, strength) {
    const e = this.emit[i];
    const main = i > 0;
    e.t = 0; e.dur = main ? 0.8 : 0.45; e.rate = main ? 230 : 90; e.str = Math.min(1.2, strength);
  }

  _spawn(px, py, pz, vel, wind, str, main) {
    if (this.n >= MAX) return;
    const P = this.p, k = this.n++;
    const r = Math.random;
    const side = new THREE.Vector3(-vel.z, 0, vel.x);
    const sl = side.length() || 1;
    const lat = (r() - 0.5) * (main ? 1.0 : 0.5);
    P.x[k] = px + side.x / sl * lat; P.y[k] = py + 0.25 + r() * 0.3; P.z[k] = pz + side.z / sl * lat;
    // smoke leaves the contact patch with part of the wheel speed, then the air stops it
    const f = 0.22 + r() * 0.18;
    P.vx[k] = wind.x + (vel.x - wind.x) * f + (r() - 0.5) * 3.0;
    P.vy[k] = 0.6 + r() * 1.8;
    P.vz[k] = wind.z + (vel.z - wind.z) * f + (r() - 0.5) * 3.0;
    P.age[k] = 0;
    P.life[k] = (main ? 2.6 : 2.0) + r() * 2.0;
    P.s0[k] = (main ? 0.7 : 0.5) + r() * 0.4;
    P.s1[k] = (main ? 5.5 : 3.5) + r() * 3.0;
    P.a0[k] = (0.22 + r() * 0.22) * str;
    P.rot[k] = r() * 6.283; P.rotv[k] = (r() - 0.5) * 0.8;
    P.seed[k] = r();
  }

  /**
   * @param {number} dt  sim-time step (s)
   * @param {object} s   { contacts: [Vector3 x3] (three world wheel contact points), vel: Vector3, wind: Vector3,
   *                       groundY(x, z), brakes (0..1), gs (m/s), wow: [bool x3] }
   */
  update(dt, s) {
    const P = this.p;
    // emitters
    if (s && s.contacts) {
      for (let i = 0; i < 3; i++) {
        const e = this.emit[i], c = s.contacts[i];
        if (!c) continue;
        const main = i > 0;
        let rate = 0, str = 0;
        if (e.t < e.dur) {
          const u = e.t / e.dur;
          rate = e.rate * (u < 0.3 ? 1 : Math.pow((1 - u) / 0.7, 1.5));
          str = e.str * (1 - 0.6 * u);
          e.t += dt;
        }
        // light wisps under hard braking at speed
        if (main && s.wow && s.wow[i] && s.brakes > 0.6 && s.gs > 15) { rate += 6 * s.brakes; str = Math.max(str, 0.25); }
        if (e.last && e.last.distanceToSquared(c) > 2500) e.last.copy(c);      // teleport (reset / tab was hidden)
        if (rate > 0 && dt > 0 && e.last) {
          e.acc = (e.acc || 0) + rate * dt;
          const m = Math.floor(e.acc);
          e.acc -= m;
          for (let k = 0; k < m; k++) {
            const u = (k + Math.random()) / Math.max(m, 1);
            this._spawn(e.last.x + (c.x - e.last.x) * u, e.last.y + (c.y - e.last.y) * u, e.last.z + (c.z - e.last.z) * u, s.vel, s.wind, str, main);
          }
        }
        if (!e.last) e.last = c.clone(); else e.last.copy(c);
      }
    }
    // integrate + compact
    let w = 0;
    const wind = s ? s.wind : null;
    for (let k = 0; k < this.n; k++) {
      const age = P.age[k] + dt;
      if (age >= P.life[k]) continue;
      const kd = Math.exp(-dt / (0.45 + age * 0.4));
      const wx = wind ? wind.x : 0, wz = wind ? wind.z : 0;
      let vx = wx + (P.vx[k] - wx) * kd, vz = wz + (P.vz[k] - wz) * kd;
      let vy = P.vy[k] * Math.exp(-dt / 1.5) + 0.35 * dt * Math.exp(-age * 0.5);
      let x = P.x[k] + vx * dt, y = P.y[k] + vy * dt, z = P.z[k] + vz * dt;
      const size = P.s0[k] + (P.s1[k] - P.s0[k]) * (1 - Math.exp(-age / 1.3));
      if (s && s.groundY) { const gy = s.groundY(x, z) + size * 0.22; if (y < gy) { y = gy; vy = Math.max(vy, 0); } }
      if (w !== k) for (const key in P) P[key][w] = P[key][k];
      P.x[w] = x; P.y[w] = y; P.z[w] = z; P.vx[w] = vx; P.vy[w] = vy; P.vz[w] = vz; P.age[w] = age;
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
    for (let j = 0; j < n; j++) {
      const k = ord[j];
      const age = P.age[k], life = P.life[k];
      const size = P.s0[k] + (P.s1[k] - P.s0[k]) * (1 - Math.exp(-age / 1.3));
      const a = P.a0[k] * Math.pow(1 - age / life, 1.6) * Math.min(1, age / 0.06 + 0.2) * (1.6 / (1 + size * 0.12));
      this.iPos[j * 4] = P.x[k]; this.iPos[j * 4 + 1] = P.y[k]; this.iPos[j * 4 + 2] = P.z[k]; this.iPos[j * 4 + 3] = size;
      this.iPar[j * 4] = a; this.iPar[j * 4 + 1] = P.rot[k]; this.iPar[j * 4 + 2] = P.seed[k]; this.iPar[j * 4 + 3] = age;
    }
    this.geo.instanceCount = n;
    this.geo.attributes.iPos.needsUpdate = true;
    this.geo.attributes.iPar.needsUpdate = true;
    const u = this.u;
    u.uDepth.value = depthTexture;
    u.uLogFar.value = Math.log2(camera.far + 1.0);
    u.uSunView.value.copy(light.sunDir).transformDirection(camera.matrixWorldInverse);
    u.uSunRad.value.copy(light.sunRad);
    u.uAmb.value.copy(light.amb);
    const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
    renderer.setRenderTarget(this.rt);
    renderer.setClearColor(0x000000, 0);
    renderer.clear(true, false, false);
    renderer.render(this.scene, camera);
    renderer.setClearColor(cc, ca);
    return true;
  }
}
