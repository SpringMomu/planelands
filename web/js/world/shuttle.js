// Procedural orbiter model built from shared/orbiter_geometry.json.
// Local frame (three): X forward, Y up, Z right; origin = CG.  FRD body (x,y,z) -> (x, -z, y).
import * as THREE from 'three';

const D2R = Math.PI / 180;
const ss = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };
const lerp = (a, b, t) => a + (b - a) * t;

// ------------------------------------------------------------------ monotone cubic interpolation
function monotone(xs, ys) {
  const n = xs.length, d = new Array(n - 1), m = new Array(n);
  for (let i = 0; i < n - 1; i++) d[i] = (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]);
  m[0] = d[0]; m[n - 1] = d[n - 2];
  for (let i = 1; i < n - 1; i++) m[i] = d[i - 1] * d[i] <= 0 ? 0 : (d[i - 1] + d[i]) / 2;
  for (let i = 0; i < n - 1; i++) {
    if (d[i] === 0) { m[i] = 0; m[i + 1] = 0; continue; }
    const a = m[i] / d[i], b = m[i + 1] / d[i], s = a * a + b * b;
    if (s > 9) { const t = 3 / Math.sqrt(s); m[i] = t * a * d[i]; m[i + 1] = t * b * d[i]; }
  }
  return (x) => {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    let i = 0; while (x > xs[i + 1]) i++;
    const h = xs[i + 1] - xs[i], t = (x - xs[i]) / h;
    const h00 = 2 * t ** 3 - 3 * t ** 2 + 1, h10 = t ** 3 - 2 * t ** 2 + t, h01 = -2 * t ** 3 + 3 * t ** 2, h11 = t ** 3 - t ** 2;
    return h00 * ys[i] + h10 * h * m[i] + h01 * ys[i + 1] + h11 * h * m[i + 1];
  };
}

// ------------------------------------------------------------------ loft helper
// rings: array of arrays of [x,y,z]; each ring a closed loop with equal point count.
function loft(rings, { capStart = false, capEnd = false, flip = false, attr = null, closed = true } = {}) {
  const M = rings[0].length;
  const pos = [], ach = [], idx = [];
  for (let r = 0; r < rings.length; r++) for (let j = 0; j < M; j++) {
    pos.push(...rings[r][j]);
    ach.push(attr ? attr[r][j] : 0);
  }
  const seg = closed ? M : M - 1;
  for (let r = 0; r < rings.length - 1; r++) for (let j = 0; j < seg; j++) {
    const a = r * M + j, b = r * M + ((j + 1) % M), c = (r + 1) * M + j, d = (r + 1) * M + ((j + 1) % M);
    if (!flip) idx.push(a, c, b, b, c, d); else idx.push(a, b, c, b, d, c);
  }
  const cap = (r, reverse) => {
    const ring = rings[r];
    const cx = ring.reduce((s, p) => s + p[0], 0) / M, cy = ring.reduce((s, p) => s + p[1], 0) / M, cz = ring.reduce((s, p) => s + p[2], 0) / M;
    const base = pos.length / 3;
    // duplicate ring vertices for flat cap normals
    for (let j = 0; j < M; j++) { pos.push(...ring[j]); ach.push(attr ? attr[r][j] : 0); }
    pos.push(cx, cy, cz); ach.push(0.5);
    const ci = base + M;
    for (let j = 0; j < M; j++) {
      const a = base + j, b = base + (j + 1) % M;
      if (reverse !== flip) idx.push(ci, a, b); else idx.push(ci, b, a);
    }
  };
  if (capStart) cap(0, true);
  if (capEnd) cap(rings.length - 1, false);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('aChord', new THREE.Float32BufferAttribute(ach, 1));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// NACA-like half thickness (closed trailing edge), xi in [0,1]
function halfThick(xi, tc) {
  xi = Math.min(1, Math.max(0, xi));
  return 5 * tc * (0.2969 * Math.sqrt(xi) - 0.1260 * xi - 0.3516 * xi * xi + 0.2843 * xi ** 3 - 0.1036 * xi ** 4);
}
function cosineSpace(a, b, n, denseStart = true) {
  const out = [];
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const u = denseStart ? 1 - Math.cos(t * Math.PI / 2) : (1 - Math.cos(t * Math.PI)) / 2;
    out.push(a + (b - a) * u);
  }
  return out;
}

// ------------------------------------------------------------------ tile material
const TILE_GLSL = /* glsl */ `
uniform int uZone;
varying vec3 vObjPos;
varying vec3 vObjN;
varying float vChord;
float th(vec2 p){ ivec2 i = ivec2(floor(p)); uint h = uint(i.x) * 1597334677u ^ (uint(i.y) * 3812015801u); h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; return float(h & 0xffffu) / 65535.0; }
// tile grid: returns (groove 0..1, tile id hash)
vec2 tileGrid(vec2 p, vec2 size, float fw){
  vec2 q = p / size;
  q.x += 0.5 * step(0.5, fract(q.y * 0.5));   // staggered rows
  vec2 f = fract(q) - 0.5;
  vec2 e = (0.5 - abs(f)) * size;
  float w = 0.004 + fw * 0.5;
  float groove = 1.0 - smoothstep(w * 0.5, w, min(e.x, e.y));
  float fade = 1.0 - smoothstep(0.02, 0.09, fw);
  return vec2(groove * fade, th(floor(q)));
}
vec2 projPlane(vec3 p, vec3 n){
  vec3 a = abs(n);
  if (a.y >= a.x && a.y >= a.z) return p.xz;
  if (a.z >= a.x) return p.xy;
  return p.zy;
}
void shuttleSurface(out vec3 col, out float rough, out float metal, out float bump){
  vec3 p = vObjPos; vec3 n = normalize(vObjN);
  vec2 pp = projPlane(p, n);
  float fw = length(fwidth(pp));
  rough = 0.82; metal = 0.0; bump = 0.0;
  // base zones
  bool black = false; bool rcc = false; bool glass = false; bool door = false; bool quilt = false;
  if (uZone == 0){ // fuselage
    // black HRSI below the chine line: follows the wing glove forward and sweeps up around the nose
    float chine = mix(-1.62, -0.55, smoothstep(10.8, 18.5, p.x));
    chine = mix(chine, 0.55, smoothstep(19.6, 21.0, p.x));
    black = n.y < -0.45 || p.y < chine;
    rcc = p.x > 20.72;
    // windows: forward panes and side panes on the windshield band
    float wx = smoothstep(14.95, 15.05, p.x) * (1.0 - smoothstep(16.40, 16.5, p.x));
    float az = abs(p.z);
    float band = smoothstep(1.62, 1.72, p.y) * (1.0 - smoothstep(2.88, 2.98, p.y));
    float mull = 1.0 - (1.0 - smoothstep(0.035, 0.06, abs(az - 0.88))) - (1.0 - smoothstep(0.035, 0.06, abs(az - 1.72))) ;
    float paneF = wx * band * clamp(mull, 0.0, 1.0) * step(az, 2.35) * step(0.03, az);
    float over = smoothstep(13.35, 13.45, p.x) * (1.0 - smoothstep(14.15, 14.25, p.x)) * smoothstep(0.30, 0.36, az) * (1.0 - smoothstep(1.05, 1.11, az)) * step(2.95, p.y);
    float frame = smoothstep(14.55, 14.7, p.x) * (1.0 - smoothstep(16.75, 16.9, p.x)) * smoothstep(1.35, 1.5, p.y) * step(az, 2.55);
    frame = max(frame, smoothstep(13.15, 13.3, p.x) * (1.0 - smoothstep(14.35, 14.5, p.x)) * step(2.9, p.y) * step(az, 1.3));
    glass = paneF > 0.5 || over > 0.5;
    if (frame > 0.5) black = true;
    door = !black && n.y > 0.3 && p.y > 1.0 && p.x < 12.25 && p.x > -11.3;
    quilt = !black && !door && p.x < 12.4;
  } else if (uZone == 1){ // wing / elevon
    black = n.y < -0.05 || vChord < 0.11;
    rcc = vChord < 0.028;
    quilt = !black && vChord > 0.55;
  } else if (uZone == 2){ // vertical tail / rudder
    black = vChord < 0.06;
    quilt = !black;
  } else if (uZone == 3){ // OMS pods
    black = p.x > -3.2 && n.x > 0.2 && p.y < 4.2;
    quilt = !black;
  } else if (uZone == 4){ // body flap
    black = true;
  } else if (uZone == 5){ // base heat shield
    col = vec3(0.07, 0.068, 0.065); rough = 0.7; return;
  }
  if (glass){ col = vec3(0.004, 0.006, 0.008); rough = 0.05; metal = 0.0; return; }
  if (rcc){ col = vec3(0.13, 0.13, 0.135); rough = 0.55; vec2 t = tileGrid(pp, vec2(0.9, 0.6), fw); bump = t.x * 0.5; return; }
  if (black){
    vec2 t = tileGrid(pp, vec2(0.152, 0.152), fw);
    float v = t.y;
    col = vec3(0.028, 0.028, 0.030) * (0.75 + 0.6 * v);
    col = mix(col, vec3(0.06, 0.055, 0.05), step(0.93, v));           // replaced tiles
    col = mix(col, col * 0.6, t.x);
    rough = 0.86 + 0.08 * v; bump = t.x;
    return;
  }
  if (door){
    float seam = 1.0 - smoothstep(0.006, 0.02 + fw, abs(p.z));
    seam = max(seam, 1.0 - smoothstep(0.006, 0.02 + fw, abs(fract((p.x + 11.3) / 3.68) - 0.5) * 3.68 - 1.83));
    float pan = th(vec2(floor((p.x + 11.3) / 3.68), p.z > 0.0 ? 1.0 : 0.0));
    col = mix(vec3(0.80, 0.80, 0.78), vec3(0.72, 0.73, 0.72), pan) * (1.0 - seam * 0.55);
    rough = 0.62; bump = seam;
    return;
  }
  if (quilt){
    vec2 t = tileGrid(pp, vec2(0.76, 0.76), fw);
    vec2 q = fract(pp / 0.76 * 4.0) - 0.5;            // stitch lines inside each blanket
    float stitch = (1.0 - smoothstep(0.0, 0.08 + fw * 6.0, min(abs(q.x), abs(q.y)))) * (1.0 - smoothstep(0.03, 0.12, fw));
    col = mix(vec3(0.78, 0.78, 0.75), vec3(0.70, 0.70, 0.68), t.y) * (1.0 - 0.18 * stitch) * (1.0 - 0.35 * t.x);
    // dirt streaks along the flow
    float st = th(vec2(floor(p.z * 6.0), 3.0));
    col *= 1.0 - 0.06 * st * smoothstep(0.0, 6.0, -p.x);
    rough = 0.93; bump = t.x * 0.7 + stitch * 0.4;
    return;
  }
  vec2 t = tileGrid(pp, vec2(0.203, 0.203), fw);
  col = mix(vec3(0.74, 0.74, 0.72), vec3(0.58, 0.58, 0.57), step(0.86, t.y)) * (1.0 - 0.4 * t.x);
  rough = 0.9; bump = t.x;
}
`;

// beads of water and wind-driven rivulets streaming aft along the body axis (object space: +x forward)
const WET_GLSL = /* glsl */ `
float wh(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
void bodyWater(vec3 op, vec3 n, float t, out float bump, out float cov){
  vec3 an = abs(n);
  // cross-stream coordinate on the local surface (sides: height, top/bottom: span)
  float s = (op.y * an.z + op.z * an.y) / max(an.y + an.z, 1e-3);
  float x = op.x;
  float fw = max(fwidth(x), fwidth(s));
  float near = 1.0 - smoothstep(0.012, 0.05, fw);
  // rivulets: lanes ~4.5 cm wide, a meandering thin stream in some, broken into segments moving aft
  float lw = 0.045;
  float li = floor(s / lw);
  float r = wh(vec2(li, 3.7));
  float lf = fract(s / lw) - 0.5 - (r - 0.5) * 0.4 - 0.12 * sin(x * (3.0 + 4.0 * r) + r * 20.0);
  float seg = fract(x * (0.25 + 0.35 * r) + t * (0.35 + 0.5 * r) + r * 9.0);
  float riv = (1.0 - smoothstep(0.05, 0.16, abs(lf))) * step(0.45, r) * smoothstep(0.0, 0.2, seg) * (1.0 - smoothstep(0.55, 0.95, seg));
  // beads: droplets on a jittered 6 cm grid
  vec2 bc = vec2(x, s) / 0.06;
  vec2 bi = floor(bc);
  vec2 bf = fract(bc) - 0.5 - (vec2(wh(bi), wh(bi + 5.3)) - 0.5) * 0.5;
  float br = 0.14 + 0.2 * wh(bi + 9.1);
  float bd = length(bf) / br;
  float bead = step(0.4, wh(bi + 2.7)) * sqrt(max(1.0 - bd * bd, 0.0));
  bump = (bead + riv * 0.7) * near;
  cov = mix(0.35, clamp(bead + riv, 0.0, 1.0), near);
}
`;

// water on the skin after flying through rain (shared by all tile materials)
const WET = { uBodyWet: { value: 0 }, uWetTime: { value: 0 } };

function tileMaterial(zone) {
  const m = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.8, metalness: 0.0, envMapIntensity: 1.0, side: THREE.DoubleSide });
  m.userData.zone = zone;
  m.onBeforeCompile = (sh) => {
    sh.uniforms.uZone = { value: zone };
    sh.uniforms.uBodyWet = WET.uBodyWet;
    sh.uniforms.uWetTime = WET.uWetTime;
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nattribute float aChord;\nvarying vec3 vObjPos;\nvarying vec3 vObjN;\nvarying float vChord;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvObjPos = position; vObjN = normal; vChord = aChord;');
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', '#include <common>\n' + TILE_GLSL + '\nuniform float uBodyWet;\nuniform float uWetTime;\n' + WET_GLSL)
      .replace('#include <color_fragment>', `#include <color_fragment>
vec3 tCol; float tRough, tMetal, tBump; shuttleSurface(tCol, tRough, tMetal, tBump); diffuseColor.rgb *= tCol;
float wetBump = 0.0, wetCov = 0.0;
if (uBodyWet > 0.002){
  bodyWater(vObjPos, normalize(vObjN), uWetTime, wetBump, wetCov);
  // soaked tiles / blankets darken, a film of water makes the skin glossy
  diffuseColor.rgb *= mix(1.0, 0.68, uBodyWet) * mix(1.0, 0.85, wetCov * uBodyWet);
  tRough = mix(tRough, 0.16, uBodyWet * (0.55 + 0.45 * wetCov));
}`)
      .replace('#include <roughnessmap_fragment>', 'float roughnessFactor = tRough;')
      .replace('#include <metalnessmap_fragment>', 'float metalnessFactor = tMetal;')
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
        {
          float hB = -tBump * 0.0025 + wetBump * 0.0016 * uBodyWet;
          vec2 dH = vec2(dFdx(hB), dFdy(hB)) * 40.0;
          vec3 sx = normalize(dFdx(-vViewPosition)); vec3 sy = normalize(dFdy(-vViewPosition));
          vec3 R1 = cross(sy, normal); vec3 R2 = cross(normal, sx);
          float det = dot(sx, R1) * faceDirection;
          vec3 grad = sign(det) * (dH.x * R1 + dH.y * R2);
          normal = normalize(abs(det) * normal - grad);
        }`);
  };
  m.customProgramCacheKey = () => 'orbiter-tile';
  return m;
}

function metalMaterial(color, rough, metal, opts = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: metal, ...opts });
}

function nozzleMaterial(inside) {
  const m = new THREE.MeshStandardMaterial({ color: inside ? 0x3a2a22 : 0x9a8f86, roughness: inside ? 0.6 : 0.38, metalness: inside ? 0.6 : 0.9, side: inside ? THREE.BackSide : THREE.FrontSide });
  m.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader.replace('#include <common>', '#include <common>\nvarying vec3 vLP;').replace('#include <begin_vertex>', '#include <begin_vertex>\nvLP = position;');
    sh.fragmentShader = sh.fragmentShader.replace('#include <common>', '#include <common>\nvarying vec3 vLP;')
      .replace('#include <color_fragment>', `#include <color_fragment>
        float ang = atan(vLP.z, vLP.x);
        float tubes = 0.5 + 0.5 * cos(ang * 180.0);
        float heat = smoothstep(0.0, 3.0, -vLP.y);
        diffuseColor.rgb *= mix(vec3(1.0), vec3(0.75, 0.7, 0.85), heat * 0.5) * (0.85 + 0.15 * tubes);`);
  };
  m.customProgramCacheKey = () => 'nozzle' + (inside ? 'i' : 'o');
  return m;
}

// ------------------------------------------------------------------ builder
export class Shuttle {
  constructor(geom) {
    this.geom = geom;
    this.group = new THREE.Group();
    this.group.name = 'orbiter';
    this.model = new THREE.Group();        // local frame
    this.group.add(this.model);
    this.surfaces = {};
    this.mat = {
      fuse: tileMaterial(0), wing: tileMaterial(1), tail: tileMaterial(2), pod: tileMaterial(3), flap: tileMaterial(4), base: tileMaterial(5),
      tire: metalMaterial(0x111111, 0.9, 0.0), hub: metalMaterial(0x8a8d90, 0.35, 0.9), strut: metalMaterial(0xb8bcc0, 0.3, 0.95),
      dark: metalMaterial(0x202224, 0.7, 0.2), nozzle: nozzleMaterial(false), nozzleIn: nozzleMaterial(true),
    };
    this._buildFuselage();
    this._buildWings();
    this._buildBodyFlap();
    this._buildTail();
    this._buildPods();
    this._buildEngines();
    this._buildGear();
    this._buildChuteHousing();
    this.model.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    const ce = geom.cockpit_eye;
    this.cockpitEye = new THREE.Vector3(ce[0], -ce[2], ce[1]);
    this.gearPos = 0;
    this.wheelSpin = 0;
  }

  // ---------------- fuselage
  _sectionParams(x) {
    const st = this.geom.fuselage.stations.slice().sort((a, b) => a[0] - b[0]);
    if (!this._fInterp) {
      const xs = st.map((s) => s[0]);
      this._fInterp = { hw: monotone(xs, st.map((s) => s[1])), zt: monotone(xs, st.map((s) => s[2])), zb: monotone(xs, st.map((s) => s[3])) };
    }
    const f = this._fInterp;
    const hw = Math.max(0.0, f.hw(x));
    const yTop = -f.zt(x), yBot = -f.zb(x);
    const kc = lerp(0.5, 0.33, ss(19.5, 13.0, x));
    const yMid = yBot + kc * (yTop - yBot);
    return { hw, yTop, yBot, yMid, nTop: lerp(2.0, 2.35, ss(19.5, 12.0, x)), nBot: lerp(2.0, 5.0, ss(20.8, 14.5, x)) };
  }
  _ring(x, M) {
    const s = this._sectionParams(x);
    const pts = [];
    for (let j = 0; j < M; j++) {
      const ph = (j / M) * Math.PI * 2;
      const c = Math.cos(ph), sn = Math.sin(ph);
      const n = sn >= 0 ? s.nTop : s.nBot;
      const z = s.hw * Math.sign(c) * Math.abs(c) ** (2 / n);
      const a = sn >= 0 ? s.yTop - s.yMid : s.yMid - s.yBot;
      const y = s.yMid + a * Math.sign(sn) * Math.abs(sn) ** (2 / n);
      pts.push([x, y, z]);
    }
    return pts;
  }
  _buildFuselage() {
    const g = this.geom.fuselage;
    const M = 72;
    const xs = [];
    const nose = g.nose_x, base = g.base_x;
    for (let i = 1; i <= 36; i++) xs.push(nose - 4.0 * (i / 36) ** 1.8);
    for (let x = nose - 4.2; x > base; x -= 0.35) xs.push(x);
    xs.push(base);
    const rings = xs.map((x) => this._ring(x, M));
    // tip: tiny ring at the very nose
    const tip = this._sectionParams(nose - 0.02);
    rings.unshift(Array.from({ length: M }, () => [nose, (tip.yTop + tip.yBot) / 2, 0]));
    const geo = loft(rings, { capStart: false, capEnd: false, flip: true });
    const mesh = new THREE.Mesh(geo, this.mat.fuse);
    mesh.name = 'fuselage';
    this.model.add(mesh);
    // base plate
    const baseRing = this._ring(base, M);
    const bp = loft([baseRing, baseRing.map(([x, y, z]) => [x - 0.02, y * 0.98, z * 0.98])], { capEnd: true, flip: true });
    const bm = new THREE.Mesh(bp, this.mat.base);
    this.model.add(bm);
    // aft lower body-flap housing (under the engines)
    const hw = this.geom.bodyflap.half_span;
    const house = new THREE.Mesh(new THREE.BoxGeometry(base - this.geom.bodyflap.hinge_x + 0.1, 0.62, hw * 2 + 0.1), this.mat.flap);
    house.position.set((base + this.geom.bodyflap.hinge_x) / 2, -this.geom.bodyflap.z + 0.31, 0);
    this.model.add(house);
  }

  // ---------------- wing + elevons
  _wingStation(y) {
    const w = this.geom.wing;
    const le = w.le, te = w.te;
    let xle;
    if (y <= le[0][1]) xle = le[0][0] + (le[0][1] - y) * Math.tan(81 * D2R);
    else if (y <= le[1][1]) xle = lerp(le[0][0], le[1][0], (y - le[0][1]) / (le[1][1] - le[0][1]));
    else xle = lerp(le[1][0], le[2][0], (y - le[1][1]) / (le[2][1] - le[1][1]));
    const xte = lerp(te[0][0], te[1][0], (y - te[0][1]) / (te[1][1] - te[0][1]));
    const tcT = w.t_over_c;
    let tc = tcT[0][1];
    for (let i = 0; i < tcT.length - 1; i++) if (y >= tcT[i][0]) tc = lerp(tcT[i][1], tcT[i + 1][1], Math.min(1, (y - tcT[i][0]) / (tcT[i + 1][0] - tcT[i][0])));
    const yc = -w.z_root + Math.max(y - w.y_root, 0) * Math.tan(w.dihedral_deg * D2R);
    // elevon chord along the span (visual: elevons span root..tip)
    const ce = y < 7.25 ? 2.55 : 2.05;
    return { xle, xte, chord: xle - xte, tc, yc, ce };
  }
  _airfoilLoop(st, xiA, xiB, side, nU, withHingeFace) {
    // returns [points, chordAttr]; loop: upper xiB->xiA, lower xiA->xiB, (+ hinge face)
    const pts = [], ach = [];
    const xis = cosineSpace(xiA, xiB, nU, xiA === 0);
    const P = (xi, up) => {
      const ht = halfThick(xi, st.tc) * st.chord;
      const yy = st.yc + (up ? ht * 1.18 : -ht * 0.82);
      return [st.xle - xi * st.chord, yy, side * st.y];
    };
    for (let i = xis.length - 1; i >= 0; i--) { pts.push(P(xis[i], true)); ach.push(xis[i]); }
    for (let i = 1; i < xis.length; i++) { pts.push(P(xis[i], false)); ach.push(xis[i]); }
    if (withHingeFace) {
      const lo = P(xiB, false), upp = P(xiB, true);
      for (let k = 0; k <= 4; k++) { const t = k / 4; pts.push([lo[0], lerp(lo[1], upp[1], t), lo[2]]); ach.push(xiB); }
    }
    return [pts, ach];
  }
  _buildWings() {
    const w = this.geom.wing;
    const ys = [];
    for (let y = 2.0; y < w.le[1][1]; y += 0.18) ys.push(y);
    for (let y = w.le[1][1]; y < w.tip_y - 0.001; y += 0.32) ys.push(y);
    ys.push(w.tip_y);
    for (const side of [1, -1]) {
      const rings = [], attrs = [];
      for (const y of ys) {
        const st = this._wingStation(y);
        st.y = y;
        const xiH = 1 - st.ce / st.chord;
        const [pts, ach] = this._airfoilLoop(st, 0, Math.max(0.3, xiH), side, 22, true);
        rings.push(pts); attrs.push(ach);
      }
      const geo = loft(rings, { capEnd: true, flip: side < 0, attr: attrs });
      const mesh = new THREE.Mesh(geo, this.mat.wing);
      mesh.name = side > 0 ? 'wing_r' : 'wing_l';
      this.model.add(mesh);
      // elevons
      const spans = [['i', 2.62, 7.22], ['o', 7.28, 11.86]];
      for (const [io, y0, y1] of spans) {
        const ers = [], eat = [];
        const n = 8;
        for (let k = 0; k <= n; k++) {
          const y = lerp(y0, y1, k / n);
          const st = this._wingStation(y); st.y = y;
          const xiH = 1 - st.ce / st.chord;
          // elevon section: from hinge (with a rounded cove nose) to TE
          const pts = [], ach = [];
          const xis = cosineSpace(xiH + 0.004, 1.0, 10, false);
          const htH = halfThick(xiH, st.tc) * st.chord;
          const P = (xi, up) => { const ht = halfThick(xi, st.tc) * st.chord; return [st.xle - xi * st.chord, st.yc + (up ? ht * 1.18 : -ht * 0.82), side * y]; };
          for (let i = xis.length - 1; i >= 0; i--) { pts.push(P(xis[i], true)); ach.push(0.95); }
          const xh = st.xle - xiH * st.chord;
          const cy = st.yc + htH * 0.18;
          for (let a = 1; a < 8; a++) { const t = Math.PI / 2 - (a / 8) * Math.PI; pts.push([xh + htH * 0.85 * Math.cos(t) * 0.9, cy + htH * Math.sin(t), side * y]); ach.push(0.95); }
          for (let i = 0; i < xis.length; i++) { pts.push(P(xis[i], false)); ach.push(0.95); }
          ers.push(pts); eat.push(ach);
        }
        const eg = loft(ers, { capStart: true, capEnd: true, flip: side < 0, attr: eat });
        const s0 = this._wingStation(y0), s1 = this._wingStation(y1);
        const h0 = new THREE.Vector3(s0.xle - (1 - s0.ce / s0.chord) * s0.chord, s0.yc, side * y0);
        const h1 = new THREE.Vector3(s1.xle - (1 - s1.ce / s1.chord) * s1.chord, s1.yc, side * y1);
        const name = `elevon_${side > 0 ? 'r' : 'l'}${io}`;
        this._addHinged(name, eg, this.mat.wing, h0, h1, side > 0 ? 1 : -1);
      }
    }
  }
  // hinged control surface: geometry in model coords, rotation about the line h0->h1
  _addHinged(name, geo, mat, h0, h1, axisSign) {
    const pivot = new THREE.Group();
    pivot.position.copy(h0);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.copy(h0).multiplyScalar(-1);
    pivot.add(mesh);
    this.model.add(pivot);
    // +angle = trailing edge down <=> axis points toward +Z (right-hand rule)
    const axis = h1.clone().sub(h0).normalize();
    if (axis.z < 0) axis.multiplyScalar(-1);
    this.surfaces[name] = { pivot, axis, mesh };
    return pivot;
  }

  _buildBodyFlap() {
    const bf = this.geom.bodyflap;
    const x0 = bf.hinge_x, x1 = bf.hinge_x - bf.chord, hw = bf.half_span;
    const yb = -bf.z;
    const rings = [];
    const n = 10;
    for (let k = 0; k <= n; k++) {
      const z = lerp(-hw, hw, k / n);
      const ring = [];
      const xs = cosineSpace(0, 1, 8, false);
      for (let i = xs.length - 1; i >= 0; i--) { const x = lerp(x0, x1, xs[i]); ring.push([x, yb + bf.thickness * (1 - 0.7 * xs[i]), z]); }
      for (let i = 1; i < xs.length - 1; i++) { const x = lerp(x0, x1, xs[i]); ring.push([x, yb, z]); }
      rings.push(ring);
    }
    const geo = loft(rings, { capStart: true, capEnd: true, flip: false });
    const h0 = new THREE.Vector3(x0, yb + bf.thickness * 0.5, -hw), h1 = new THREE.Vector3(x0, yb + bf.thickness * 0.5, hw);
    this._addHinged('bodyflap', geo, this.mat.flap, h0, h1, 1);
  }

  // ---------------- vertical tail + split rudder
  _finStation(Y) {
    const vt = this.geom.vtail;
    const yr = -vt.root.z, yt = -vt.tip.z;
    const t = (Y - yr) / (yt - yr);
    const xle = lerp(vt.root.le_x, vt.tip.le_x, t), xte = lerp(vt.root.te_x, vt.tip.te_x, t);
    return { xle, xte, chord: xle - xte, tc: vt.t_over_c };
  }
  _buildTail() {
    const vt = this.geom.vtail, rd = vt.rudder;
    const yRoot = 2.75, yTip = -vt.tip.z;
    const r0 = -rd.z0, r1 = -rd.z1;
    const finLoop = (Y, xiB, hinge) => {
      const st = this._finStation(Y);
      const xis = cosineSpace(0, xiB, 20, true);
      const pts = [], ach = [];
      for (let i = xis.length - 1; i >= 0; i--) { const ht = halfThick(xis[i], st.tc) * st.chord; pts.push([st.xle - xis[i] * st.chord, Y, ht]); ach.push(xis[i]); }
      for (let i = 1; i < xis.length; i++) { const ht = halfThick(xis[i], st.tc) * st.chord; pts.push([st.xle - xis[i] * st.chord, Y, -ht]); ach.push(xis[i]); }
      if (hinge) { const ht = halfThick(xiB, st.tc) * st.chord; for (let k = 0; k <= 4; k++) { pts.push([st.xle - xiB * st.chord, Y, lerp(-ht, ht, k / 4)]); ach.push(xiB); } }
      else { for (let k = 0; k <= 4; k++) { pts.push([st.xle - st.chord, Y, 0]); ach.push(1); } }
      return [pts, ach];
    };
    const mk = (Y0, Y1, n, rud) => {
      const rings = [], at = [];
      for (let k = 0; k <= n; k++) {
        const Y = lerp(Y0, Y1, k / n);
        const [p, a] = finLoop(Y, rud ? 1 - rd.chord_frac : 1.0, rud);
        rings.push(p); at.push(a);
      }
      return { rings, at };
    };
    const parts = [mk(yRoot, r0, 4, false), mk(r0, r1, 14, true), mk(r1, yTip, 2, false)];
    parts.forEach((pt, i) => {
      const g = loft(pt.rings, { capStart: i === 0, capEnd: i === 2, attr: pt.at, flip: true });
      const m = new THREE.Mesh(g, this.mat.tail);
      m.name = 'fin' + i;
      this.model.add(m);
    });
    // rudder halves (left: z<0, right: z>0)
    for (const half of [1, -1]) {
      const rings = [], at = [];
      for (let k = 0; k <= 12; k++) {
        const Y = lerp(r0 + 0.02, r1 - 0.02, k / 12);
        const st = this._finStation(Y);
        const xiH = 1 - rd.chord_frac;
        const xis = cosineSpace(xiH + 0.003, 1, 10, false);
        const pts = [], ach = [];
        for (let i = xis.length - 1; i >= 0; i--) { const ht = halfThick(xis[i], st.tc) * st.chord; pts.push([st.xle - xis[i] * st.chord, Y, half * ht]); ach.push(0.9); }
        for (let i = 0; i < xis.length; i++) pts.push([st.xle - xis[i] * st.chord, Y, half * 0.002]), ach.push(0.9);
        rings.push(pts); at.push(ach);
      }
      const g = loft(rings, { capStart: true, capEnd: true, attr: at, flip: half > 0 });
      const s0 = this._finStation(r0), s1 = this._finStation(r1);
      const h0 = new THREE.Vector3(s0.xle - (1 - rd.chord_frac) * s0.chord, r0, 0);
      const h1 = new THREE.Vector3(s1.xle - (1 - rd.chord_frac) * s1.chord, r1, 0);
      const pivot = new THREE.Group();
      pivot.position.copy(h0);
      const mesh = new THREE.Mesh(g, this.mat.tail);
      mesh.position.copy(h0).multiplyScalar(-1);
      pivot.add(mesh);
      this.model.add(pivot);
      this.surfaces[half > 0 ? 'rudder_r' : 'rudder_l'] = { pivot, axis: h1.clone().sub(h0).normalize(), mesh };
    }
  }

  _buildPods() {
    const p = this.geom.oms_pods;
    for (const side of [1, -1]) {
      const rings = [];
      const n = 40, M = 40;
      const L = p.x_front - p.x_aft;
      for (let k = 0; k <= n; k++) {
        const t = k / n;
        const x = p.x_front - t * L;
        const nose = Math.sqrt(Math.max(0, 1 - (1 - Math.min(t / 0.28, 1)) ** 2));
        const w = 1.02 * nose * (1 - 0.12 * ss(0.65, 1.0, t));
        const hgt = 0.95 * nose;
        const yc = lerp(2.95, 3.35, ss(0, 0.55, t));
        const ring = [];
        for (let j = 0; j < M; j++) {
          const ph = j / M * Math.PI * 2;
          const c = Math.cos(ph), s = Math.sin(ph);
          const e = 2.5;
          ring.push([x, yc + hgt * Math.sign(s) * Math.abs(s) ** (2 / e) * (s < 0 ? 1.6 : 1), side * (p.y - 0.15) + w * Math.sign(c) * Math.abs(c) ** (2 / e) * (s < 0 ? 1.15 : 1)]);
        }
        rings.push(ring);
      }
      const g = loft(rings, { capEnd: true, flip: true });
      const m = new THREE.Mesh(g, this.mat.pod);
      this.model.add(m);
      // OMS nozzle
      const noz = this._nozzle(0.12, 0.62, 1.25, 28);
      noz.position.set(p.x_aft + 0.15, 3.45, side * (p.y - 0.1));
      noz.rotation.z = -Math.PI / 2;
      this.model.add(noz);
    }
  }

  _nozzle(rThroat, rExit, len, seg) {
    const pts = [];
    for (let i = 0; i <= 24; i++) {
      const t = i / 24;
      const r = rThroat + (rExit - rThroat) * (1 - (1 - t) ** 1.9);
      pts.push(new THREE.Vector2(r, -t * len));
    }
    const geo = new THREE.LatheGeometry(pts, seg);
    const grp = new THREE.Group();
    grp.add(new THREE.Mesh(geo, this.mat.nozzle));
    grp.add(new THREE.Mesh(geo, this.mat.nozzleIn));
    return grp;   // axis along -Y of the group
  }
  _buildEngines() {
    const e = this.geom.engines;
    for (const m of e.mounts) {
      const [x, y, z] = m;
      const noz = this._nozzle(0.3, e.exit_radius, e.length, 40);
      const pos = new THREE.Vector3(x, -z, y);
      noz.position.copy(pos);
      noz.rotation.z = -Math.PI / 2;       // lathe -Y -> model -X (aft)
      // slight cant like the stowed SSMEs
      noz.rotateX(y === 0 ? 0 : Math.sign(y) * 0.05);
      noz.rotateZ(y === 0 ? 0.10 : -0.06);
      this.model.add(noz);
      const ph = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.62, 0.7, 24), this.mat.dark);
      ph.position.set(x + 0.15, -z, y); ph.rotation.z = Math.PI / 2;
      this.model.add(ph);
    }
  }

  // ---------------- landing gear
  _wheelPair(r, w, sep) {
    const g = new THREE.Group();
    const tire = new THREE.CylinderGeometry(r, r, w, 28); tire.rotateX(Math.PI / 2);
    const hub = new THREE.CylinderGeometry(r * 0.55, r * 0.55, w * 1.04, 20); hub.rotateX(Math.PI / 2);
    for (const s of [-1, 1]) {
      const wg = new THREE.Group();
      wg.add(new THREE.Mesh(tire, this.mat.tire));
      wg.add(new THREE.Mesh(hub, this.mat.hub));
      wg.position.z = s * sep;
      g.add(wg);
    }
    return g;
  }
  _buildGear() {
    const G = this.geom.gear;
    this.gear = [];
    const mk = (key, isNose) => {
      const c = G[key];
      const contact = new THREE.Vector3(c.x, -c.z, c.y);
      const wheelC = contact.clone(); wheelC.y += c.wheel_radius;
      let wellY = -2.2;
      if (!isNose) { const st = this._wingStation(Math.abs(c.y)); const xi = (st.xle - c.x) / st.chord; wellY = st.yc - 0.82 * halfThick(xi, st.tc) * st.chord + 0.05; }
      const well = new THREE.Vector3(c.x, wellY, c.y);
      const root = new THREE.Group();       // rotates for retraction
      root.position.copy(well);
      this.model.add(root);
      const strutLen = well.y - wheelC.y;
      const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, strutLen, 14), this.mat.strut);
      strut.position.y = -strutLen / 2;
      root.add(strut);
      const slider = new THREE.Group();   // moves with compression
      slider.position.y = -strutLen;
      root.add(slider);
      const piston = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.6, 12), this.mat.hub);
      piston.position.y = 0.3;
      slider.add(piston);
      const wheels = this._wheelPair(c.wheel_radius, isNose ? 0.22 : 0.38, isNose ? 0.2 : 0.3);
      slider.add(wheels);
      // drag brace
      const brace = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, strutLen * 0.9, 8), this.mat.strut);
      brace.position.set(isNose ? -0.6 : 0.7, -strutLen * 0.45, 0);
      brace.rotation.z = (isNose ? 1 : -1) * 0.5;
      root.add(brace);
      // doors (two per well), hinge on the long edges
      const doors = [];
      const dl = isNose ? 2.2 : 3.6, dw = isNose ? 0.55 : 0.75;
      for (const s of [-1, 1]) {
        const hinge = new THREE.Group();
        hinge.position.set(c.x, wellY - 0.03, c.y + s * dw);
        const door = new THREE.Mesh(new THREE.BoxGeometry(dl, 0.05, dw), this.mat.flap);
        door.position.z = -s * dw / 2;
        hinge.add(door);
        this.model.add(hinge);
        doors.push({ hinge, s });
      }
      this.gear.push({ key, root, slider, wheels, doors, strutLen, isNose, r: c.wheel_radius });
    };
    mk('nose', true); mk('main_l', false); mk('main_r', false);
  }

  // drag-chute compartment: fairing at the base of the fin trailing edge, above SSME #1, aft-facing door
  _buildChuteHousing() {
    const a = this.geom.drag_chute.attach;
    this.chuteAttachLocal = new THREE.Vector3(a[0], -a[2], a[1]);
    // streamlined fairing: rounded nose blending into the fin root, flat aft door
    const L = 1.9, x1 = a[0] + 0.02;
    const prof = [];
    for (let i = 0; i <= 10; i++) { const t = i / 10; prof.push(new THREE.Vector2(Math.sin(t * Math.PI / 2) * 1.0, L - (1 - Math.cos(t * Math.PI / 2)) * 1.1)); }
    prof.push(new THREE.Vector2(1.0, 0.0));
    const body = new THREE.LatheGeometry(prof, 28);
    body.rotateZ(-Math.PI / 2);                      // lathe +Y -> +X: nose forward, door plane at x = 0
    body.scale(1, 0.6, 0.4);
    const m = new THREE.Mesh(body, this.mat.fuse);
    m.position.set(x1, this.chuteAttachLocal.y, 0);
    this.model.add(m);
    const cap = new THREE.CircleGeometry(1, 28);
    cap.rotateY(-Math.PI / 2);                       // facing aft (-X)
    cap.scale(1, 0.6, 0.4);
    const door = new THREE.Mesh(cap, this.mat.fuse);
    door.position.set(x1 - 0.005, this.chuteAttachLocal.y, 0);
    this.model.add(door);
    const hole = new THREE.Mesh(cap.clone().scale(0.9, 0.9, 0.9), this.mat.dark);
    hole.position.set(x1 + 0.04, this.chuteAttachLocal.y, 0);
    this.model.add(hole);
    this.chuteDoor = door;
  }

  // ------------------------------------------------------------------ animation API
  setSurfaces(s) {
    if (!s) return;
    const set = (name, deg) => {
      const sf = this.surfaces[name]; if (!sf) return;
      sf.pivot.setRotationFromAxisAngle(sf.axis, deg * D2R);
    };
    // FRD convention: + = trailing edge down; local hinge axes point outboard-ish (+Z right / -Z left)
    set('elevon_ri', s.elevon_ri || 0); set('elevon_ro', s.elevon_ro || 0);
    set('elevon_li', s.elevon_li || 0); set('elevon_lo', s.elevon_lo || 0);
    set('bodyflap', s.bodyflap || 0);
    const rud = s.rudder || 0, sb = s.speedbrake || 0;
    // rudder + = TE left (toward -Z); hinge axis points up/aft
    const left = rud + sb / 2, right = rud - sb / 2;
    const rl = this.surfaces.rudder_l, rr = this.surfaces.rudder_r;
    if (rl) rl.pivot.setRotationFromAxisAngle(rl.axis, -left * D2R * Math.sign(rl.axis.y || 1));
    if (rr) rr.pivot.setRotationFromAxisAngle(rr.axis, -right * D2R * Math.sign(rr.axis.y || 1));
  }

  setGear(pos, compress = [0, 0, 0], wow = [false, false, false], dt = 0, groundSpeed = 0) {
    this.gearPos = pos;
    const ext = ss(0.0, 0.85, pos);
    const door = ss(0.0, 0.35, pos);
    this.gear.forEach((g, i) => {
      // retract forward (main) / aft (nose): rotate about Z
      const ang = (1 - ext) * (g.isNose ? -1.45 : 1.5);
      g.root.rotation.z = ang;
      g.root.visible = pos > 0.02;
      const c = Math.min(compress[i] || 0, 0.45);
      g.slider.position.y = -g.strutLen + c;
      // wheels only spin up on contact (that spin-up is what makes the touchdown smoke), then coast down
      g.omega = wow[i] ? groundSpeed / g.r : (g.omega || 0) * Math.exp(-dt / 6);
      if (g.omega > 1e-3) g.wheels.children.forEach((w) => { w.rotation.z -= g.omega * dt; });
      for (const d of g.doors) d.hinge.rotation.x = -d.s * door * 1.45;
    });
  }

  /** water on the skin: wet 0..1, flow = time integral driving the rivulets */
  setWet(wet, flow) { WET.uBodyWet.value = wet; WET.uWetTime.value = flow; }

  /** compartment door: gone once the chute has been deployed */
  setChuteDoor(open) { if (this.chuteDoor) this.chuteDoor.visible = !open; }

  /** chute riser attach point in three world coordinates (needs an up-to-date matrixWorld) */
  chuteAttachWorld(out = new THREE.Vector3()) { return this.model.localToWorld(out.copy(this.chuteAttachLocal)); }

  /** wheel/runway contact points (three world) for nose, main_l, main_r */
  wheelContacts(out) {
    this.gear.forEach((g, i) => {
      const v = out[i] || (out[i] = new THREE.Vector3());
      g.wheels.getWorldPosition(v);
      v.y -= g.r;
    });
    return out;
  }
}
