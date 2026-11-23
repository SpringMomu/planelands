// Near-camera rain: world-anchored drops in a box that wraps around the camera, drawn as
// motion-blurred streaks (drop velocity relative to the camera x shutter time, so rain slants and
// stretches into long streaks when the orbiter flies through it), plus splash crowns on the ground.
// Everything is computed in the vertex shader; the CPU only updates a few uniforms.  Drawn into its own
// premultiplied layer (manual depth test against the scene depth) that the composite lays over the
// final image - rain right in front of the lens is never hidden by the cloud the camera is flying in.
import * as THREE from 'three';
import { makeRT } from '../render/pass.js';

const N_DROPS = 36000;
const N_SPLASH = 4000;

const COMMON = /* glsl */ `
precision highp float;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
uniform vec3 uCamPos;
uniform float uTime;
uniform vec2 uRes;
uniform float uAmount;       // 0..1 share of drops / splashes alive
float hash11(float p){ p = fract(p * 0.1031); p *= p + 33.33; p *= p + p; return fract(p); }
vec2 hash21(float p){ vec3 p3 = fract(vec3(p) * vec3(0.1031, 0.1030, 0.0973)); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.xx + p3.yz) * p3.zy); }
`;

const DROP_VS = COMMON + /* glsl */ `
in vec3 position;            // x: 0 tail .. 1 head, y: -1 .. 1 across
in vec4 aSeed;               // xyz: position in the box, w: rank (amount threshold)
in float aSpeed;             // drop-size spread: speed factor
uniform vec3 uBox;
uniform vec3 uFall;          // drop velocity (wind + terminal fall), m/s
uniform vec3 uOffset;        // integrated drift of the rain since start (so changing wind never makes drops jump)
uniform vec3 uCamVel;
uniform float uExposure;     // shutter time (s, sim time)
out vec2 vQ; out float vA; out float vViewZ;
void main(){
  if (aSeed.w > uAmount){ gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
  vec3 v = uFall * vec3(1.0, aSpeed, 1.0);
  vec3 p = aSeed.xyz * uBox + uOffset * vec3(1.0, aSpeed, 1.0);
  vec3 rel = mod(p - uCamPos + uBox * 0.5, uBox) - uBox * 0.5;
  vec3 wp = uCamPos + rel;
  vec3 vr = v - uCamVel;
  vec3 sv = vr * uExposure;
  float sl = length(sv);
  if (sl > 4.5) sv *= 4.5 / sl;                 // cap the smear (very high speed / time scale)
  vec3 tail = wp - sv;
  vec4 c0 = projectionMatrix * viewMatrix * vec4(tail, 1.0);
  vec4 c1 = projectionMatrix * viewMatrix * vec4(wp, 1.0);
  if (c0.w < 0.25 || c1.w < 0.25){ gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
  vec2 s0 = c0.xy / c0.w, s1 = c1.xy / c1.w;
  vec2 dpx = (s1 - s0) * uRes * 0.5;
  float lenPx = length(dpx);
  vec2 dir = lenPx > 1e-3 ? dpx / lenPx : vec2(0.0, 1.0);
  vec2 perp = vec2(-dir.y, dir.x);
  vec4 c = mix(c0, c1, position.x);
  // a ~1.5 mm drop is sub-pixel: draw at >= 1.2 px and scale alpha to keep its energy
  float wPx = 0.0015 * projectionMatrix[1][1] * 0.5 * uRes.y / c.w;
  float drawPx = max(wPx, 1.2);
  vA = clamp(wPx / drawPx, 0.0, 1.0) * 0.85 + 0.15 * clamp(wPx * 4.0, 0.0, 1.0);
  vec2 off = perp * position.y * drawPx * 0.5 + dir * (position.x * 2.0 - 1.0) * 0.8;
  c.xy += off / (uRes * 0.5) * c.w;
  // fade at the box edge and right in front of the lens; long streaks spread their light thinner
  vec3 rb = abs(rel) / (uBox * 0.5);
  vA *= (1.0 - smoothstep(0.7, 1.0, max(max(rb.x, rb.z), rb.y))) * smoothstep(0.25, 1.0, length(rel));
  vA *= clamp(40.0 / max(lenPx, 1.0), 0.35, 1.0);
  vQ = position.xy;
  vViewZ = c.w;
  gl_Position = c;
}`;

const SPLASH_VS = COMMON + /* glsl */ `
in vec3 position;            // quad -0.5..0.5
in vec4 aSeed;
uniform float uGroundY;
uniform float uRadius;
out vec2 vQ; out float vA; out float vViewZ; out float vPh;
void main(){
  if (aSeed.w > uAmount){ gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
  float cyc = uTime * (2.2 + 1.6 * aSeed.z) + aSeed.x * 17.0;
  float k = floor(cyc);
  float ph = fract(cyc);
  vec2 r = hash21(k * 7.13 + aSeed.y * 1931.7) * 2.0 - 1.0;
  vec2 xz = uCamPos.xz + r * uRadius;
  float life = 0.18;
  float pl = ph / life;                       // splash only lives for the first part of the cycle
  if (pl > 1.0){ gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
  float size = 0.05 + 0.16 * sqrt(pl) * (0.6 + 0.8 * aSeed.z);
  vec3 right = vec3(viewMatrix[0][0], viewMatrix[1][0], viewMatrix[2][0]);
  vec3 wp = vec3(xz.x, uGroundY + size * 0.45, xz.y) + right * position.x * size + vec3(0.0, position.y * size, 0.0);
  vec4 vp = viewMatrix * vec4(wp, 1.0);
  vViewZ = -vp.z;
  vQ = position.xy * 2.0;
  vPh = pl;
  vA = (1.0 - pl) * (1.0 - smoothstep(uRadius * 0.4, uRadius, length(r * uRadius))) * smoothstep(0.5, 2.0, vViewZ);
  gl_Position = projectionMatrix * vp;
}`;

const FS_HEAD = /* glsl */ `
precision highp float;
uniform sampler2D uDepth;
uniform vec2 uRes;
uniform float uLogFar;
uniform vec3 uCol;
uniform float uAlpha;
out vec4 outColor;
float sceneW(){
  float d = texture(uDepth, gl_FragCoord.xy / uRes).r;
  return d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
}
`;

const DROP_FS = FS_HEAD + /* glsl */ `
in vec2 vQ; in float vA; in float vViewZ;
void main(){
  if (vViewZ > sceneW() + 0.05) discard;
  float across = 1.0 - vQ.y * vQ.y;
  float along = smoothstep(0.0, 0.35, vQ.x) * (0.75 + 0.25 * vQ.x);
  float a = vA * uAlpha * across * along;
  if (a < 0.002) discard;
  outColor = vec4(uCol * a, a);
}`;

const SPLASH_FS = FS_HEAD + /* glsl */ `
in vec2 vQ; in float vA; in float vViewZ; in float vPh;
void main(){
  if (vViewZ > sceneW() + 0.1) discard;
  // crown: a thin rim rising from the impact point plus a few flung droplets
  vec2 q = vQ;
  float r = length(vec2(q.x, (q.y + 0.9) * 1.6));
  float rx = (r - (0.4 + 0.6 * vPh)) / 0.12;
  float rim = exp(-rx * rx) * step(-0.9, q.y);
  float spray = 0.0;
  for (int i = 0; i < 5; i++){
    float fi = float(i);
    vec2 c = vec2((fi - 2.0) * 0.32, -0.6 + (0.9 + 0.3 * sin(fi * 3.1)) * vPh * 1.4 - vPh * vPh * 1.1);
    spray += exp(-dot(q - c, q - c) / 0.012);
  }
  float a = clamp(rim * 0.8 + spray * 0.7, 0.0, 1.0) * vA * uAlpha;
  if (a < 0.003) discard;
  outColor = vec4(uCol * a, a);
}`;

function layerMaterial(vs, fs, uniforms) {
  return new THREE.RawShaderMaterial({
    vertexShader: vs, fragmentShader: fs, uniforms, glslVersion: THREE.GLSL3,
    transparent: true, depthTest: false, depthWrite: false,
    blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
    blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
    blendSrcAlpha: THREE.OneFactor, blendDstAlpha: THREE.OneMinusSrcAlphaFactor,
  });
}

export class RainParticles {
  constructor() {
    this.scene = new THREE.Scene();
    this.shared = {
      uCamPos: { value: new THREE.Vector3() }, uTime: { value: 0 }, uRes: { value: new THREE.Vector2(1, 1) },
      uDepth: { value: null }, uLogFar: { value: 1 }, uCol: { value: new THREE.Color(1, 1, 1) },
    };
    // ---------------- drops
    const dg = new THREE.InstancedBufferGeometry();
    dg.setAttribute('position', new THREE.Float32BufferAttribute([0, -1, 0, 1, -1, 0, 0, 1, 0, 1, 1, 0], 3));
    dg.setIndex([0, 1, 2, 1, 3, 2]);
    const seed = new Float32Array(N_DROPS * 4), spd = new Float32Array(N_DROPS);
    for (let i = 0; i < N_DROPS; i++) {
      seed[i * 4] = Math.random(); seed[i * 4 + 1] = Math.random(); seed[i * 4 + 2] = Math.random(); seed[i * 4 + 3] = Math.random();
      spd[i] = 0.7 + Math.random() * 0.55;
    }
    dg.setAttribute('aSeed', new THREE.InstancedBufferAttribute(seed, 4));
    dg.setAttribute('aSpeed', new THREE.InstancedBufferAttribute(spd, 1));
    dg.instanceCount = N_DROPS;
    this.du = {
      ...this.shared, uAmount: { value: 0 }, uAlpha: { value: 0.7 },
      uBox: { value: new THREE.Vector3(50, 40, 50) }, uFall: { value: new THREE.Vector3(0, -8.5, 0) },
      uCamVel: { value: new THREE.Vector3() }, uExposure: { value: 0.02 }, uOffset: { value: new THREE.Vector3() },
    };
    this.drops = new THREE.Mesh(dg, layerMaterial(DROP_VS, DROP_FS, this.du));
    this.drops.frustumCulled = false;
    this.scene.add(this.drops);
    // ---------------- splashes
    const sg = new THREE.InstancedBufferGeometry();
    const quad = new THREE.PlaneGeometry(1, 1);
    sg.index = quad.index;
    sg.setAttribute('position', quad.getAttribute('position'));
    const ss = new Float32Array(N_SPLASH * 4);
    for (let i = 0; i < ss.length; i++) ss[i] = Math.random();
    sg.setAttribute('aSeed', new THREE.InstancedBufferAttribute(ss, 4));
    sg.instanceCount = N_SPLASH;
    this.su = { ...this.shared, uAmount: { value: 0 }, uAlpha: { value: 0.6 }, uGroundY: { value: 0 }, uRadius: { value: 22 } };
    this.splash = new THREE.Mesh(sg, layerMaterial(SPLASH_VS, SPLASH_FS, this.su));
    this.splash.frustumCulled = false;
    this.scene.add(this.splash);
    this.amount = 0;
    this.rt = null;
    this.off = [0, 0, 0];                 // float64 accumulation of the drift
  }

  setSize(w, h) { this.rt?.dispose(); this.rt = makeRT(w, h); }
  get texture() { return this.rt ? this.rt.texture : null; }

  /**
   * @param {object} p { amount 0..1, time (sim s), camPos, camVel (sim m/s), fall: Vector3 (drop velocity),
   *                     exposure (sim s), groundY (three y under the camera), camAgl, col: Color }
   */
  update(p) {
    this.amount = p.amount;
    const dt = p.dt || 0;
    this.off[0] += p.fall.x * dt; this.off[1] += p.fall.y * dt; this.off[2] += p.fall.z * dt;
    this.du.uOffset.value.set(this.off[0] % 100000, this.off[1], this.off[2] % 100000);
    const sh = this.shared;
    sh.uCamPos.value.copy(p.camPos);
    sh.uTime.value = p.time;
    sh.uCol.value.copy(p.col);
    this.du.uAmount.value = p.amount;
    this.du.uFall.value.copy(p.fall);
    this.du.uCamVel.value.copy(p.camVel);
    this.du.uExposure.value = p.exposure;
    this.su.uAmount.value = p.camAgl < 60 ? p.amount * (1 - Math.min(1, Math.max(0, (p.camAgl - 25) / 35))) : 0;
    this.su.uGroundY.value = p.groundY;
  }

  /** draws the rain layer; returns false when there is nothing to show */
  render(renderer, camera, depthTexture) {
    const rt = this.rt, clear = true;
    if (this.amount <= 0.002 || !rt) return false;
    const sh = this.shared;
    sh.uDepth.value = depthTexture;
    sh.uLogFar.value = Math.log2(camera.far + 1.0);
    sh.uRes.value.set(rt.width, rt.height);
    this.splash.visible = this.su.uAmount.value > 0.002;
    renderer.setRenderTarget(rt);
    if (clear) {
      const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
      renderer.setClearColor(0x000000, 0);
      renderer.clear(true, false, false);
      renderer.setClearColor(cc, ca);
    }
    const ac = renderer.autoClear;
    renderer.autoClear = false;
    renderer.render(this.scene, camera);
    renderer.autoClear = ac;
    return true;
  }
}
