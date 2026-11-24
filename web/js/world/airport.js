// Shuttle landing facility: runway with procedural markings, taxiways, apron, buildings,
// approach / edge / PAPI lights and instanced trees.  Everything is placed on the curved
// world (y' = y - r^2 / 2R) like the terrain.
import * as THREE from 'three';
import { PLANET_R } from '../render/glsl.js';
import { patchStandard } from './materials.js';

const drop = (x, z) => (x * x + z * z) / (2 * PLANET_R);

function digitAtlas() {
  const cv = document.createElement('canvas');
  cv.width = 1280; cv.height = 256;
  const c = cv.getContext('2d');
  c.fillStyle = '#000'; c.fillRect(0, 0, cv.width, cv.height);
  c.fillStyle = '#fff';
  c.textAlign = 'center'; c.textBaseline = 'middle';
  c.font = 'bold 250px "Arial Narrow", "Helvetica Neue", Arial, sans-serif';
  for (let d = 0; d < 10; d++) {
    c.save();
    c.translate(d * 128 + 64, 132);
    c.scale(0.62, 1.0);
    c.fillText(String(d), 0, 0);
    c.restore();
  }
  const t = new THREE.CanvasTexture(cv);
  t.colorSpace = THREE.NoColorSpace;
  t.anisotropy = 8;
  t.generateMipmaps = true;
  t.minFilter = THREE.LinearMipmapLinearFilter;
  return t;
}

const RUNWAY_GLSL = /* glsl */ `
uniform sampler2D uDigits;
uniform float uRwL; uniform float uRwW; uniform float uRwOver;
uniform vec4 uDesig;   // digits: end33 tens, ones, end15 tens, ones
varying vec2 vRw;
float rh(vec2 p){ ivec2 i = ivec2(floor(p)); uint h = uint(i.x) * 1597334677u ^ (uint(i.y) * 3812015801u); h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; return float(h & 0xffffu) / 65535.0; }
float rn(vec2 p){ vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f); return mix(mix(rh(i), rh(i + vec2(1, 0)), f.x), mix(rh(i + vec2(0, 1)), rh(i + vec2(1, 1)), f.x), f.y); }
// box-filtered coverage of the interval [-w, w] at offset x with filter half-width aa
float cov1(float x, float w, float aa){ aa = max(aa, 1e-4); return clamp((min(x + w, aa) - max(x - w, -aa)) / (2.0 * aa), 0.0, 1.0); }
// 2-D box mask with per-axis filter widths (p in metres)
float boxm(vec2 p, vec2 c, vec2 hs, vec2 aa){ vec2 d = p - c; return cov1(d.x, hs.x, aa.x) * cov1(d.y, hs.y, aa.y); }
// periodic stripes: period P, stripe half-width w (centred on multiples of P + o)
float stripes(float x, float P, float o, float w, float aa){ float d = x - o - P * floor((x - o) / P + 0.5); return cov1(d, w, aa); }
float digit(vec2 p, vec2 c, vec2 size, float d){
  vec2 q = (p - c) / size + 0.5;
  if (q.x < 0.0 || q.x > 1.0 || q.y < 0.0 || q.y > 1.0) return 0.0;
  return texture(uDigits, vec2((d + q.x) / 10.0, q.y)).r;
}
// a: along from the threshold (landing direction), b: lateral (pilot's right +)
float endMarkings(float a, float b, float tens, float ones, vec2 aa, out float yellow){
  yellow = 0.0;
  float m = 0.0;
  float ab = abs(b);
  if (a < 0.0){
    // blast pad / overrun chevrons
    if (ab < uRwW * 0.5 && a > -uRwOver){
      // chevrons (45 deg) pointing toward the threshold, 1.8 m wide every 30 m
      yellow = stripes((-a + ab) * 0.7071, 21.2, 0.0, 0.9, max(aa.x, aa.y) * 0.9);
    }
    return 0.0;
  }
  // threshold stripes
  if (a > 4.0 && a < 53.0 && ab < 30.0){
    float st = stripes(ab, 3.5, 2.625, 0.875, aa.y) * step(ab, 28.5);
    m = max(m, st * cov1(a - 28.5, 22.5, aa.x));
  }
  // designation
  m = max(m, digit(vec2(b, a), vec2(-4.3, 72.0), vec2(7.0, 18.3), tens));
  m = max(m, digit(vec2(b, a), vec2(4.3, 72.0), vec2(7.0, 18.3), ones));
  // aiming point
  m = max(m, boxm(vec2(a, ab), vec2(327.5, 20.0), vec2(22.5, 4.5), aa));
  // touchdown zone bars
  float tdz[6] = float[6](152.0, 457.0, 610.0, 762.0, 914.0, 1066.0);
  float nb[6] = float[6](3.0, 2.0, 2.0, 1.0, 1.0, 1.0);
  for (int i = 0; i < 6; i++){
    if (abs(a - tdz[i]) < 12.0){
      for (int j = 0; j < 3; j++){
        if (float(j) >= nb[i]) break;
        m = max(m, boxm(vec2(a, ab), vec2(tdz[i], 18.0 + float(j) * 3.35), vec2(11.25, 0.915), aa));
      }
    }
  }
  // centreline dashes
  if (a > 100.0 && ab < 1.0 + aa.y){
    m = max(m, cov1(ab, 0.45, aa.y) * stripes(a, 61.0, 100.0 + 18.3, 18.3, aa.x));
  }
  return m;
}
void runwaySurface(out vec3 col, out float rough){
  vec2 p = vRw;               // (u along toward RWY33 direction, v right)
  vec2 aa2 = fwidth(p) * 0.75 + 0.01;
  float aa = length(aa2);
  float halfL = uRwL * 0.5;
  float u = p.x, v = p.y;
  bool shoulder = abs(v) > uRwW * 0.5;
  // base: grooved concrete with slab joints
  vec2 slab = floor(p / vec2(7.62, 7.62));
  float sv = rh(slab);
  vec3 conc = vec3(0.205, 0.203, 0.193) * (0.96 + 0.06 * sv) * (0.88 + 0.24 * rn(p / 40.0)) * (0.94 + 0.12 * rn(p / 3.0));
  vec2 jf = abs(fract(p / 7.62) - 0.5) * 7.62;
  float joint = 1.0 - smoothstep(0.01, 0.03 + aa, min(3.81 - jf.x, 3.81 - jf.y));
  joint *= 1.0 - smoothstep(0.15, 0.6, aa);
  conc *= 1.0 - 0.16 * joint;
  // grooves (transverse)
  conc *= 1.0 - 0.06 * (1.0 - smoothstep(0.0, 0.5, aa)) * step(0.5, fract(u / 0.038));
  col = conc;
  rough = 0.86;
  if (shoulder){ col = vec3(0.075, 0.075, 0.072) * (0.9 + 0.2 * rn(p / 9.0)); rough = 0.9; }
  // ends
  float a, b, tens, ones;
  if (u < 0.0){ a = u + halfL; b = v; tens = uDesig.x; ones = uDesig.y; }
  else { a = halfL - u; b = -v; tens = uDesig.z; ones = uDesig.w; }
  if (a < 0.0 && !shoulder){ col = vec3(0.085, 0.083, 0.08) * (0.9 + 0.2 * rn(p / 6.0)); rough = 0.9; }
  // rubber deposits in the touchdown zones
  float rub = smoothstep(150.0, 400.0, a) * (1.0 - smoothstep(900.0, 1400.0, a)) * (1.0 - smoothstep(4.0, 13.0, abs(b)));
  float streak = rn(vec2(b * 2.5, a * 0.01)) * rn(vec2(b * 0.7 + 3.0, a * 0.003));
  col *= 1.0 - rub * (0.35 + 0.45 * streak);
  // oil / wear
  col *= 1.0 - 0.15 * smoothstep(0.75, 0.95, rn(p / 3.0 + 7.0)) * (1.0 - smoothstep(2.0, 4.0, abs(b)));
  float yel;
  float m = endMarkings(a, b, tens, ones, aa2, yel);
  // side stripes
  if (a >= 0.0) m = max(m, cov1(abs(v) - (uRwW * 0.5 - 1.45), 0.45, aa2.y));
  float wear = 0.82 + 0.18 * rn(p / 2.0);
  float paint = m * wear * (1.0 - rub * 0.5);
  col = mix(col, vec3(0.55, 0.55, 0.53), paint);
  col = mix(col, vec3(0.55, 0.40, 0.05), yel * 0.9);
  rough = mix(rough, 0.6, max(paint, yel));
}
`;

const TAXI_GLSL = /* glsl */ `
varying vec2 vRw;
uniform float uTaxiW;
uniform float uApron;
float th2(vec2 p){ ivec2 i = ivec2(floor(p)); uint h = uint(i.x) * 1597334677u ^ (uint(i.y) * 3812015801u); h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; return float(h & 0xffffu) / 65535.0; }
float tn(vec2 p){ vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f); return mix(mix(th2(i), th2(i + vec2(1, 0)), f.x), mix(th2(i + vec2(0, 1)), th2(i + vec2(1, 1)), f.x), f.y); }
void taxiSurface(out vec3 col, out float rough){
  vec2 p = vRw;
  float aa = length(fwidth(p)) * 0.75 + 0.02;
  if (uApron > 0.5){
    vec2 jf = abs(fract(p / 6.0) - 0.5) * 6.0;
    float joint = (1.0 - smoothstep(0.02, 0.05 + aa, min(3.0 - jf.x, 3.0 - jf.y))) * (1.0 - smoothstep(0.15, 0.6, aa));
    col = vec3(0.27, 0.265, 0.25) * (0.9 + 0.15 * th2(floor(p / 6.0))) * (1.0 - 0.3 * joint) * (0.9 + 0.2 * tn(p / 30.0));
    float ax = fwidth(p.x) * 0.75 + 0.01; float ld = abs(fract(p.x / 90.0) * 90.0 - 45.0);
    float lead = clamp((min(ld + 0.15, ax) - max(ld - 0.15, -ax)) / (2.0 * ax), 0.0, 1.0);
    col = mix(col, vec3(0.55, 0.40, 0.05), lead * step(10.0, p.y) * 0.9);
    col *= 1.0 - 0.25 * smoothstep(0.7, 0.95, tn(p / 5.0));
    rough = 0.85;
    return;
  }
  col = vec3(0.07, 0.07, 0.068) * (0.85 + 0.3 * tn(p / 8.0));
  float ay = fwidth(p.y) * 0.75 + 0.01;
  float cl = clamp((min(p.y + 0.08, ay) - max(p.y - 0.08, -ay)) / (2.0 * ay), 0.0, 1.0);
  float e1d = abs(p.y) - (uTaxiW * 0.5 - 0.5), e2d = abs(p.y) - (uTaxiW * 0.5 - 0.8);
  float e1 = clamp((min(e1d + 0.08, ay) - max(e1d - 0.08, -ay)) / (2.0 * ay), 0.0, 1.0);
  float e2 = clamp((min(e2d + 0.08, ay) - max(e2d - 0.08, -ay)) / (2.0 * ay), 0.0, 1.0);
  float m = cl + e1 + e2;
  col = mix(col, vec3(0.55, 0.40, 0.05), m * 0.9);
  rough = mix(0.9, 0.6, m);
}
`;

const LIGHT_VS = /* glsl */ `
#include <common>
#include <logdepthbuf_pars_vertex>
attribute vec3 aColor;
attribute float aIntensity;
attribute float aType;      // 0 steady, 1 PAPI, 2 sequenced flasher
attribute vec4 aParam;      // PAPI: aim dir (xz), threshold angle (rad); flasher: phase
uniform float uTime;
uniform float uPixelRatio;
uniform float uVis;          // 0 day .. 1 night
varying vec3 vCol;
void main(){
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  float dist = -mv.z;
  vec3 col = aColor;
  float inten = aIntensity;
  if (aType > 0.5 && aType < 1.5){
    vec3 wp = (modelMatrix * vec4(position, 1.0)).xyz;
    vec3 d = cameraPosition - wp;
    float horiz = length(d.xz);
    float elev = atan(d.y, horiz);
    float facing = dot(normalize(d.xz), aParam.xy);
    col = elev > aParam.z ? vec3(1.0, 0.95, 0.85) : vec3(1.0, 0.05, 0.02);
    inten *= smoothstep(0.2, 0.6, facing);
  } else if (aType > 1.5){
    float ph = fract(uTime * 2.0 - aParam.x);
    inten *= ph < 0.04 ? 4.0 : 0.0;
  }
  float px = clamp(inten * 900.0 / max(dist, 1.0), 1.6, 26.0) * uPixelRatio;
  gl_PointSize = px;
  gl_Position = projectionMatrix * mv;
  // keep total energy ~ constant as the sprite shrinks to its minimum size
  float energy = inten * 900.0 / max(dist, 1.0) / (px / uPixelRatio);
  vCol = col * mix(1.2, 30.0, uVis) * clamp(energy, 0.05, 1.0);
  #include <logdepthbuf_vertex>
}`;
const LIGHT_FS = /* glsl */ `
#include <logdepthbuf_pars_fragment>
varying vec3 vCol;
void main(){
  #include <logdepthbuf_fragment>
  vec2 c = gl_PointCoord - 0.5;
  float r2 = dot(c, c) * 4.0;
  float a = exp(-r2 * 7.0) + 0.12 * exp(-r2 * 2.5);
  a *= 1.0 - smoothstep(0.7, 1.0, r2);
  if (a < 0.015) discard;
  gl_FragColor = vec4(vCol * a, 1.0);
}`;

export class Airport {
  constructor(world, U, terrain) {
    this.world = world;
    this.U = U;
    this.terrain = terrain;
    this.group = new THREE.Group();
    this.group.name = 'airport';
    const rw = world.runway;
    this.rw = rw;
    const h = THREE.MathUtils.degToRad(rw.heading_deg);
    this.dir = new THREE.Vector2(Math.sin(h), Math.cos(h));   // EN
    this.right = new THREE.Vector2(Math.cos(h), -Math.sin(h));
    this.elev = rw.elevation;
    this.lightsTime = { value: 0 };
    this.lightsVis = { value: 0 };
    this._buildRunway();
    this._buildTaxiways();
    this._buildBuildings();
    this._buildLights();
    this._buildTrees();
  }

  /** runway frame (u along RWY33 direction, v right) + height -> three world (curved) */
  toWorld(u, v, y = 0, out = new THREE.Vector3()) {
    const e = this.rw.center_en[0] + u * this.dir.x + v * this.right.x;
    const n = this.rw.center_en[1] + u * this.dir.y + v * this.right.y;
    const x = e, z = -n;
    return out.set(x, this.elev + y - drop(x, z), z);
  }
  yawThree() { return -THREE.MathUtils.degToRad(this.rw.heading_deg) + Math.PI / 2; } // rotation about Y aligning local +X with RWY33 direction

  _gridGeo(u0, u1, v0, v1, du, dv, yOff, attrFn) {
    const nu = Math.max(1, Math.ceil((u1 - u0) / du)), nv = Math.max(1, Math.ceil((v1 - v0) / dv));
    const pos = [], rwa = [], nor = [], idx = [];
    const p = new THREE.Vector3();
    for (let j = 0; j <= nv; j++) for (let i = 0; i <= nu; i++) {
      const u = u0 + (u1 - u0) * i / nu, v = v0 + (v1 - v0) * j / nv;
      const [uu, vv] = attrFn ? attrFn(u, v) : [u, v];
      this.toWorld(u, v, yOff, p);
      pos.push(p.x, p.y, p.z); rwa.push(uu, vv); nor.push(0, 1, 0);
    }
    for (let j = 0; j < nv; j++) for (let i = 0; i < nu; i++) {
      const a = j * (nu + 1) + i, b = a + 1, c = a + nu + 1, d = c + 1;
      idx.push(a, c, b, b, c, d);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
    g.setAttribute('aRw', new THREE.Float32BufferAttribute(rwa, 2));
    g.setIndex(idx);
    g.computeBoundingSphere();
    return g;
  }

  _surfaceMaterial(glsl, fnName, uniforms, key) {
    const m = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.85, metalness: 0, side: THREE.DoubleSide });
    patchStandard(m, this.U, {
      key,
      uniforms: { ...uniforms, uWet: this.U.uWet },
      vertex: (sh) => {
        sh.vertexShader = sh.vertexShader.replace('#include <common>', '#include <common>\nattribute vec2 aRw;\nvarying vec2 vRw;')
          .replace('#include <begin_vertex>', '#include <begin_vertex>\nvRw = aRw;');
      },
      fragment: (sh) => {
        sh.fragmentShader = sh.fragmentShader.replace('varying vec3 vWPos;', 'varying vec3 vWPos;\nuniform float uWet;\n' + glsl)
          .replace('#include <color_fragment>', `#include <color_fragment>
vec3 sCol; float sRough; ${fnName}(sCol, sRough); diffuseColor.rgb *= sCol;
// rain: wet pavement (darker, smoother), standing water in ruts / low spots, paint stays brighter
float wetRefl = 0.0;
if (uWet > 0.0){
  float rainHere = uRain > 0.0 ? rainAt(vWPos.xz) / uRain : 0.0;
  float wet = uWet * (0.6 + 0.4 * smoothstep(0.02, 0.35, rainHere));
  float paint = smoothstep(0.45, 0.6, luma(sCol));
  float pn = vnoise(vWPos.xz / 4.5) * 0.5 + vnoise(vWPos.xz / 17.0) * 0.35 + vnoise(vWPos.xz / 61.0) * 0.15;
  float thr = 0.72 - 0.22 * wet;
  float pud = wet * smoothstep(thr, thr + 0.06, pn);
  diffuseColor.rgb *= mix(1.0, mix(0.48, 0.85, paint), wet);
  diffuseColor.rgb *= mix(1.0, 0.4, pud);
  sRough = mix(sRough, 0.45, wet);
  wetRefl = max(pud, wet * mix(0.42, 0.25, paint));
}`)
          .replace('#include <roughnessmap_fragment>', 'float roughnessFactor = sRough;')
          .replace('#include <opaque_fragment>', '#include <opaque_fragment>\ngl_FragColor.a = 1.0 + wetRefl;');
      },
    });
    return m;
  }

  _buildRunway() {
    const rw = this.rw;
    const L = rw.length, W = rw.width, O = rw.overrun, S = rw.shoulder;
    const geo = this._gridGeo(-L / 2 - O, L / 2 + O, -W / 2 - S, W / 2 + S, 20, (W + 2 * S) / 6, 0.0);
    const d33 = rw.ends[1], d15 = rw.ends[0];
    const mat = this._surfaceMaterial(RUNWAY_GLSL, 'runwaySurface', {
      uDigits: { value: digitAtlas() },
      uRwL: { value: L }, uRwW: { value: W }, uRwOver: { value: O },
      uDesig: { value: new THREE.Vector4(+d33[0], +d33[1], +d15[0], +d15[1]) },
    }, 'runway');
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    mesh.name = 'runway';
    this.group.add(mesh);
  }

  _strip(points, width, yOff) {
    // polyline strip in runway frame; aRw = (distance along, lateral)
    const pos = [], rwa = [], nor = [], idx = [];
    let s = 0;
    const p = new THREE.Vector3();
    const pts = [];
    for (let i = 0; i < points.length - 1; i++) {
      const [u0, v0] = points[i], [u1, v1] = points[i + 1];
      const len = Math.hypot(u1 - u0, v1 - v0);
      const n = Math.max(1, Math.ceil(len / 25));
      for (let k = (i === 0 ? 0 : 1); k <= n; k++) pts.push([u0 + (u1 - u0) * k / n, v0 + (v1 - v0) * k / n]);
    }
    for (let i = 0; i < pts.length; i++) {
      const a = pts[Math.max(0, i - 1)], b = pts[Math.min(pts.length - 1, i + 1)];
      let tu = b[0] - a[0], tv = b[1] - a[1];
      const l = Math.hypot(tu, tv) || 1; tu /= l; tv /= l;
      if (i > 0) s += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
      for (const side of [-1, 1]) {
        const u = pts[i][0] - tv * side * width / 2, v = pts[i][1] + tu * side * width / 2;
        this.toWorld(u, v, yOff, p);
        pos.push(p.x, p.y, p.z); rwa.push(s, side * width / 2); nor.push(0, 1, 0);
      }
    }
    for (let i = 0; i < pts.length - 1; i++) { const a = i * 2; idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
    g.setAttribute('aRw', new THREE.Float32BufferAttribute(rwa, 2));
    g.setIndex(idx);
    return g;
  }

  _buildTaxiways() {
    const L = this.rw.length;
    const tw = 23;
    const vT = -190;
    const mat = this._surfaceMaterial(TAXI_GLSL, 'taxiSurface', { uTaxiW: { value: tw }, uApron: { value: 0 } }, 'taxi');
    const add = (pts) => { const m = new THREE.Mesh(this._strip(pts, tw, -0.03), mat); m.receiveShadow = true; this.group.add(m); };
    add([[-L / 2 - 150, vT], [L / 2 + 150, vT]]);
    for (const u of [-L / 2 - 100, -1100, 0, 1100, L / 2 + 100]) add([[u, vT], [u + (u < 0 ? 60 : -60) * 0, -55]]);
    add([[-200, vT], [-200, -330]]); add([[350, vT], [350, -330]]);
    // apron
    const apronMat = this._surfaceMaterial(TAXI_GLSL, 'taxiSurface', { uTaxiW: { value: tw }, uApron: { value: 1 } }, 'apron');
    const ag = this._gridGeo(-650, 650, -560, -300, 50, 65, -0.05, (u, v) => [u, -v - 300]);
    const am = new THREE.Mesh(ag, apronMat); am.receiveShadow = true;
    this.group.add(am);
    // access road toward the west
    const roadMat = this._surfaceMaterial(TAXI_GLSL, 'taxiSurface', { uTaxiW: { value: 9 }, uApron: { value: 0 } }, 'road');
    const rm = new THREE.Mesh(this._strip([[0, -560], [-300, -900], [-900, -1350], [-2400, -1700]], 9, -0.06), roadMat);
    rm.receiveShadow = true;
    this.group.add(rm);
  }

  _box(w, h, d, mat, u, v, yaw = 0, y = 0) {
    const g = new THREE.BoxGeometry(w, h, d);
    g.translate(0, h / 2, 0);
    const m = new THREE.Mesh(g, mat);
    this.toWorld(u, v, y, m.position);
    m.rotation.y = this.yawThree() + yaw;
    m.castShadow = true; m.receiveShadow = true;
    this.group.add(m);
    return m;
  }

  _buildBuildings() {
    const U = this.U;
    const std = (c, r = 0.8, m = 0) => patchStandard(new THREE.MeshStandardMaterial({ color: c, roughness: r, metalness: m }), U, { key: 'b' });
    const wall = std(0xd9d6cf, 0.75), wall2 = std(0xb8bcc0, 0.6, 0.3), roofM = std(0x8f969c, 0.5, 0.6), dark = std(0x2a2d31, 0.6);
    const glass = patchStandard(new THREE.MeshStandardMaterial({ color: 0x0b1418, roughness: 0.08, metalness: 0.4 }), U, { key: 'g' });
    const steel = std(0xc9ccd0, 0.45, 0.8), red = std(0xa8322a, 0.6), white = std(0xf0f0ec, 0.5);
    // corrugated walls via a striped normal pattern
    const corr = new THREE.MeshStandardMaterial({ color: 0xcfd3d6, roughness: 0.55, metalness: 0.35 });
    patchStandard(corr, U, {
      key: 'corr',
      fragment: (sh) => {
        sh.fragmentShader = sh.fragmentShader.replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
          { float s = sin((vWPos.x + vWPos.z) * 9.0); normal = normalize(normal + 0.25 * s * normalize(cross(normal, vec3(0.0, 1.0, 0.0)) + 1e-4)); }`);
      },
    });

    // hangars on the apron edge (curved roofs)
    for (const [u, w] of [[-430, 95], [-250, 80], [430, 95]]) {
      const d = 65, hgt = 24;
      this._box(w, hgt, d, corr, u, -600, 0);
      const roof = new THREE.CylinderGeometry(d / 2, d / 2, w, 32, 1, false, 0, Math.PI);
      roof.rotateZ(Math.PI / 2); roof.rotateY(Math.PI / 2); roof.scale(1, 0.32, 1);
      roof.rotateY(Math.PI / 2);
      const rm = new THREE.Mesh(roof, roofM);
      this.toWorld(u, -600, hgt, rm.position);
      rm.rotation.y = this.yawThree();
      rm.castShadow = true; rm.receiveShadow = true;
      this.group.add(rm);
      const door = this._box(w * 0.8, hgt * 0.85, 0.5, dark, u, -600 + d / 2 + 0.3, 0);
      door.castShadow = false;
    }
    // control tower
    this._box(7, 34, 7, wall, 120, -640);
    this._box(13, 5.5, 13, glass, 120, -640, Math.PI / 4, 34);
    this._box(14, 1.2, 14, dark, 120, -640, Math.PI / 4, 39.5);
    const ant = this._box(0.4, 9, 0.4, red, 120, -640, 0, 40.7); ant.castShadow = false;
    // operations / admin buildings
    this._box(120, 14, 40, wall, 150, -760);
    this._box(60, 9, 30, wall2, -80, -760);
    this._box(30, 22, 30, wall, 300, -780);
    // fuel tanks
    for (let i = 0; i < 4; i++) {
      const g = new THREE.CylinderGeometry(9, 9, 13, 32); g.translate(0, 6.5, 0);
      const m = new THREE.Mesh(g, white);
      this.toWorld(600 + i * 26, -820, 0, m.position);
      m.castShadow = true; m.receiveShadow = true;
      this.group.add(m);
    }
    // large integration building landmark (north-west)
    this._box(160, 160, 210, wall, 2900, -1300);
    this._box(162, 30, 212, wall2, 2900, -1300, 0, 160);
    this._box(30, 120, 1, dark, 2900 + 60, -1300 + 105.6, 0);
    this._box(30, 120, 1, dark, 2900 - 60, -1300 + 105.6, 0);
    // mate-demate style gantry: steel lattice
    const beams = [];
    const gx = 520, gv = -380, gw = 28, gd = 32, gh = 32;
    const leg = (a, b) => beams.push([a, b]);
    for (const [cu, cv] of [[-gw / 2, -gd / 2], [gw / 2, -gd / 2], [-gw / 2, gd / 2], [gw / 2, gd / 2]]) leg([cu, 0, cv], [cu, gh, cv]);
    for (let y = 4; y <= gh; y += 7) {
      leg([-gw / 2, y, -gd / 2], [gw / 2, y, -gd / 2]); leg([-gw / 2, y, gd / 2], [gw / 2, y, gd / 2]);
      leg([-gw / 2, y, -gd / 2], [-gw / 2, y, gd / 2]); leg([gw / 2, y, -gd / 2], [gw / 2, y, gd / 2]);
      leg([-gw / 2, y, -gd / 2], [gw / 2, y + 7, -gd / 2]); leg([-gw / 2, y, gd / 2], [gw / 2, y + 7, gd / 2]);
    }
    leg([-gw / 2, gh, 0], [gw / 2, gh, 0]);
    const im = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), steel, beams.length);
    const base = this.toWorld(gx, gv, 0);
    const yaw = this.yawThree();
    const mtx = new THREE.Matrix4(), q = new THREE.Quaternion(), sc = new THREE.Vector3();
    const A = new THREE.Vector3(), B = new THREE.Vector3(), mid = new THREE.Vector3();
    beams.forEach(([a, b], i) => {
      A.set(a[0], a[1], a[2]).applyAxisAngle(new THREE.Vector3(0, 1, 0), yaw).add(base);
      B.set(b[0], b[1], b[2]).applyAxisAngle(new THREE.Vector3(0, 1, 0), yaw).add(base);
      mid.addVectors(A, B).multiplyScalar(0.5);
      const len = A.distanceTo(B);
      q.setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize());
      sc.set(0.7, len, 0.7);
      mtx.compose(mid, q, sc);
      im.setMatrixAt(i, mtx);
    });
    im.castShadow = true; im.receiveShadow = true;
    this.group.add(im);
    // parked vehicles
    const cars = new THREE.InstancedMesh(new THREE.BoxGeometry(4.5, 1.5, 1.9), std(0xffffff, 0.35, 0.6), 160);
    const col = new THREE.Color();
    let k = 0;
    for (let r = 0; r < 4; r++) for (let c = 0; c < 40; c++) {
      if (Math.random() < 0.25) continue;
      const p = this.toWorld(-20 + c * 3.0, -820 - r * 8, 0.75);
      mtx.compose(p, new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw + Math.PI / 2), new THREE.Vector3(1, 1, 1));
      cars.setMatrixAt(k, mtx);
      col.setHSL(Math.random(), Math.random() * 0.5, 0.2 + Math.random() * 0.6);
      cars.setColorAt(k, col);
      k++;
    }
    cars.count = k;
    cars.castShadow = true;
    this.group.add(cars);
  }

  _buildLights() {
    const rw = this.rw, L = rw.length, W = rw.width;
    const pos = [], colr = [], inten = [], type = [], param = [];
    const p = new THREE.Vector3();
    const add = (u, v, y, c, i, t = 0, prm = [0, 0, 0, 0]) => {
      this.toWorld(u, v, y, p);
      pos.push(p.x, p.y, p.z); colr.push(...c); inten.push(i); type.push(t); param.push(...prm);
    };
    const white = [1, 0.93, 0.8], yellow = [1, 0.75, 0.25], green = [0.2, 1, 0.45], red = [1, 0.1, 0.05], blue = [0.2, 0.4, 1];
    // edge lights
    for (let u = -L / 2; u <= L / 2 + 0.1; u += 61) for (const s of [-1, 1]) {
      const nearEnd = Math.min(u + L / 2, L / 2 - u) < 600;
      add(u, s * (W / 2 + 1.5), 0.4, nearEnd ? yellow : white, 0.25);
    }
    // centreline lights
    for (let u = -L / 2 + 30; u < L / 2 - 30; u += 15) add(u, 0.3, 0.05, white, 0.1);
    // threshold (green) / end (red) bars
    for (let v = -W / 2; v <= W / 2 + 0.1; v += 3.0) {
      add(-L / 2 - 1, v, 0.3, green, 0.35); add(L / 2 + 1, v, 0.3, green, 0.35);
      add(-L / 2 + 1, v, 0.3, red, 0.25); add(L / 2 - 1, v, 0.3, red, 0.25);
    }
    // approach lighting (ALSF style) with sequenced flashers, both ends
    for (const end of [-1, 1]) {
      const thr = end * L / 2;
      for (let d = 60; d <= 730; d += 30) {
        const u = thr + end * d;
        for (let k = -2; k <= 2; k++) add(u, k * 1.0, 0.5 + d * 0.004, white, 0.45);
        add(u, 0.0, 1.2 + d * 0.004, white, 0.9, 2, [1 - d / 730, 0, 0, 0]);
        if (d === 300) for (let k = -7; k <= 7; k++) add(u, k * 1.5, 0.8, white, 0.45);
      }
    }
    // PAPI for a 3 deg glide path (left side) and the orbiter's 20 deg outer glide-slope PAPI (right side, at the
    // OGS aim point 2.3 km out: sim/gnc/geometry.py ApproachConfig gamma_ogs_deg / x_aim_ogs)
    for (const end of [-1, 1]) {
      const thr = end * L / 2, inward = -end;
      const aim = new THREE.Vector2(-this.dir.x * inward, -this.dir.y * inward); // horizontal direction toward the approach (EN)
      const aimXZ = [aim.x, -aim.y];
      const papiAng = [3.5, 3.17, 2.83, 2.5].map((a) => THREE.MathUtils.degToRad(a));
      papiAng.forEach((a, i) => add(thr + inward * 320, inward * (-(W / 2 + 15 + i * 9)), 0.8, white, 0.9, 1, [aimXZ[0], aimXZ[1], a, 0]));
      const ogs = [21.0, 20.3, 19.7, 19.0].map((a) => THREE.MathUtils.degToRad(a));
      ogs.forEach((a, i) => add(thr - inward * 2300, inward * (W / 2 + 60 + i * 9), 0.8, white, 1.4, 1, [aimXZ[0], aimXZ[1], a, 0]));
    }
    // taxiway edge lights
    for (let u = -L / 2 - 150; u <= L / 2 + 150; u += 60) for (const s of [-1, 1]) add(u, -190 + s * 12.5, 0.3, blue, 0.12);
    // obstruction lights
    add(120, -640, 50, red, 0.8); add(2900, -1300, 192, red, 1.2);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('aColor', new THREE.Float32BufferAttribute(colr, 3));
    g.setAttribute('aIntensity', new THREE.Float32BufferAttribute(inten, 1));
    g.setAttribute('aType', new THREE.Float32BufferAttribute(type, 1));
    g.setAttribute('aParam', new THREE.Float32BufferAttribute(param, 4));
    const mat = new THREE.ShaderMaterial({
      vertexShader: LIGHT_VS, fragmentShader: LIGHT_FS,
      uniforms: { uTime: this.lightsTime, uPixelRatio: { value: Math.min(window.devicePixelRatio, 2) }, uVis: this.lightsVis },
      transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
    });
    const pts = new THREE.Points(g, mat);
    pts.frustumCulled = false;
    pts.renderOrder = 10;
    this.group.add(pts);
    this.lights = pts;
  }

  _buildTrees() {
    const T = this.terrain;
    if (!T) return;
    const U = this.U;
    // two tree archetypes merged into one geometry each (crown + trunk), vertex-coloured
    const mkTree = (crown) => {
      const trunk = new THREE.CylinderGeometry(0.18, 0.28, 3.5, 5); trunk.translate(0, 1.75 - 2.0, 0);
      const parts = [trunk, crown];
      const cols = [[0.22, 0.15, 0.09], [1, 1, 1]];
      let pos = [], nor = [], col = [], idx = [], off = 0;
      parts.forEach((p, i) => {
        const g = p.index ? p : p;
        const P = g.attributes.position.array, N = g.attributes.normal.array;
        for (let k = 0; k < P.length; k += 3) { pos.push(P[k], P[k + 1], P[k + 2]); nor.push(N[k], N[k + 1], N[k + 2]); col.push(...cols[i]); }
        if (g.index) { const I = g.index.array; for (let k = 0; k < I.length; k++) idx.push(I[k] + off); }
        else for (let k = 0; k < P.length / 3; k++) idx.push(k + off);
        off += P.length / 3;
      });
      const out = new THREE.BufferGeometry();
      out.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      out.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
      out.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
      out.setIndex(idx);
      return out;
    };
    const conifer = new THREE.ConeGeometry(2.6, 11, 7); conifer.translate(0, 1.5 + 5.5 - 2.0, 0);
    const broad = new THREE.IcosahedronGeometry(3.6, 1); broad.scale(1, 0.85, 1); broad.translate(0, 4.6 - 2.0, 0);
    const geos = [mkTree(conifer), mkTree(broad)];
    const mat = patchStandard(new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.95, metalness: 0, flatShading: false }), U, { key: 'tree' });
    const N = 26000;
    const meshes = geos.map((g) => new THREE.InstancedMesh(g, mat, N));
    const counts = [0, 0];
    const mtx = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(), p = new THREE.Vector3(), c = new THREE.Color();
    const R = 7000;
    let tries = 0;
    let seed = 1234567;
    const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
    while ((counts[0] + counts[1]) < N * 1.6 && tries < 400000) {
      tries++;
      const e = (rnd() * 2 - 1) * R, n = (rnd() * 2 - 1) * R;
      // keep clear of the airport operating area
      const u = e * this.dir.x + n * this.dir.y, v = e * this.right.x + n * this.right.y;
      if (Math.abs(u) < 3200 && v > -1000 && v < 420) continue;
      if (Math.abs(u) < 4200 && Math.abs(v) < 260) continue;
      const lc = T.landcoverAt(e, n);
      const h = T.heightAt(e, n);
      if (lc[3] > 0.05 || h < 0.8) continue;
      const dens = Math.pow(lc[0], 1.5) * 0.98 + 0.006 + lc[2] * 0.12;
      if (rnd() > dens) continue;
      const kind = rnd() < 0.55 ? 0 : 1;
      if (counts[kind] >= N) continue;
      const sc = 0.7 + rnd() * 0.75;
      const x = e, z = -n;
      p.set(x, h - drop(x, z) - 0.3 * (Math.abs(u) < 4300 && Math.abs(v) < 1500 ? 1 : 0), z);
      q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), rnd() * 6.28);
      s.set(sc, sc * (0.85 + rnd() * 0.4), sc);
      mtx.compose(p, q, s);
      meshes[kind].setMatrixAt(counts[kind], mtx);
      c.setRGB(0.10 + rnd() * 0.05, 0.17 + rnd() * 0.08, 0.06 + rnd() * 0.04);
      if (kind === 0) c.multiplyScalar(0.7);
      meshes[kind].setColorAt(counts[kind], c);
      counts[kind]++;
    }
    meshes.forEach((m, i) => { m.count = counts[i]; m.receiveShadow = true; m.castShadow = false; m.frustumCulled = false; this.group.add(m); });
    this.treeCount = counts[0] + counts[1];
  }

  update(t, night) {
    this.lightsTime.value = t;
    this.lightsVis.value = night;
  }
}
