// Airflow around the orbiter, driven by the simulator's own aerodynamic model (sim/aero.py).
//
// 1. Surface pressure paint (all modes).  The aero message carries, per strip, the terms of its
//    normal-force coefficient (attached potential flow from the flow angle, from the flap deflection,
//    Polhamus vortex lift, separated flat plate) and its dynamic-pressure ratio.  Each term is spread
//    over the chord with its own shape - flat-plate sqrt((1-x)/x) / thin-airfoil flap load subsonic,
//    Ackeret uniform load supersonic, a leading-edge-vortex hump, a uniform separated load - each
//    normalised so that it integrates back to the model's force.  The local surface speed is the
//    strip's inflow (free stream + rotation, as in the model) plus half the vortex-sheet strength
//    (thickness term, Riegels leading-edge factor) and the spanwise flow induced by the spanwise
//    gradient of the strip circulation; Cp = 1 - (V/U)^2.  Fuselage / pods: the slender-body
//    crossflow around the elliptic sections (the model's Munk term) plus the axial flow of its
//    thickness sources.  Base: the model's base pressure.
// 2. Wing section (mode "section"): an unsteady 2-D vortex simulation of the mesh's own section at the
//    chosen span station, elevon deflected as drawn (airflow_section.js, stepped every frame).  Its
//    free stream is the strip's local inflow; leading-edge shedding sets in where the model's strip
//    stalls (its attached fraction); its angle of attack is the inflow less the downwash the model's
//    wing implies (lifting line with the strip's 3-D lift slope; attached, the computed normal force is
//    trimmed onto the model's).  On the GPU: the velocity of the whole solution on a
//    grid, smoke from a rake upstream and dye from the nose carried through it, the pressure from the
//    local speed behind; the outer wing on the camera side is cut away at the section.
// 3. Wake (mode "wake"): the free vortex wake of all strips (airflow_wake.js): rings shed with the
//    strips' circulation, every node moved every frame by Biot-Savart of the whole wake.
import * as THREE from 'three';
import { FullscreenPass, makeRT } from '../render/pass.js';
import { PAINT } from './airflow_paint.js';
import { halfThick } from './shuttle.js';
import { SectionFlow } from './airflow_section.js';
import { FreeWake } from './airflow_wake.js';

const SURF_WING = 0, SURF_BF = 1, SURF_VT = 2;
const clamp = THREE.MathUtils.clamp;
const ss = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };

// ------------------------------------------------------------------ chordwise load shapes (int = 1)
const shapeAlpha = (xi) => { const x = clamp(xi, 0.003, 1); return 2 / Math.PI * Math.sqrt((1 - x) / x); };
const cumAlpha = (xi) => { const th = Math.acos(1 - 2 * clamp(xi, 0, 1)); return (th + Math.sin(th)) / Math.PI; };
const VA = 0.12, VNORM = 1 - Math.exp(-1 / VA) * (1 + 1 / VA);
const shapeVortex = (xi) => xi / (VA * VA) * Math.exp(-xi / VA) / VNORM;
// thin-airfoil flap load (hinge at 1 - cf), normalised numerically
const _flapCache = new Map();
function flapShape(cf) {
  if (!(cf > 0.01)) return () => 0;
  const key = Math.round(cf * 1000);
  if (_flapCache.has(key)) return _flapCache.get(key);
  const th0 = Math.acos(1 - 2 * (1 - cf));
  const raw = (xi) => {
    const th = Math.acos(1 - 2 * clamp(xi, 0.002, 0.998));
    const lg = Math.log(Math.abs(Math.sin((th + th0) / 2) / Math.max(Math.abs(Math.sin((th - th0) / 2)), 0.02)));
    return (Math.PI - th0) / Math.tan(th / 2) + lg;
  };
  let sum = 0;
  const n = 4000;
  for (let k = 0; k < n; k++) sum += raw((k + 0.5) / n) / n;
  const f = (xi) => raw(xi) / sum;
  _flapCache.set(key, f);
  return f;
}
// the model's base pressure (AeroModel.base_cp)
const baseCp = (M) => (M < 1.05 ? -0.22 - 0.23 * ss(0.6, 1.05, M) : -0.45 * (1.05 / M) ** 1.8);

// ------------------------------------------------------------------ section: GPU passes
// The unsteady 2-D solution (airflow_section.js) on a grid over the display domain (wind-aligned
// rectangle in the section's chord frame): free stream + bound vortex panels + thickness source panels
// + all free vortices; (velocity along / across the domain axes, speed)
const SEC_VEL = /* glsl */ `
uniform sampler2D uPan, uVort;
uniform int uNP, uNV;
uniform vec2 uC, uS, uN, uVinf;
const float TPI = 6.283185307;
void main(){
  vec2 P = uC + (vUv.x * 2.0 - 1.0) * uS + (vUv.y * 2.0 - 1.0) * uN;
  vec2 V = uVinf;
  vec4 a = texelFetch(uPan, ivec2(0, 0), 0);
  for (int j = 0; j < uNP; j++){
    vec4 b = texelFetch(uPan, ivec2(j + 1, 0), 0);
    vec2 d = b.xy - a.xy;
    float L = max(length(d), 1e-6);
    vec2 t = d / L, nn = vec2(-t.y, t.x), o = P - a.xy;
    float xl = dot(o, t), yl = dot(o, nn);
    float r1 = xl * xl + yl * yl, r2 = (xl - L) * (xl - L) + yl * yl;
    float D = atan(yl, xl - L) - atan(yl, xl);
    float Lg = log(max(r1, 1e-10) / max(r2, 1e-10));
    float gm = a.z / L, sg = a.w;
    V += (gm * D / TPI + sg * Lg / (2.0 * TPI)) * t + (-gm * Lg / (2.0 * TPI) + sg * D / TPI) * nn;
    a = b;
  }
  for (int k = 0; k < uNV; k++){
    vec4 v = texelFetch(uVort, ivec2(k & 63, k >> 6), 0);
    vec2 r = P - v.xy;
    float r2 = dot(r, r);
    V += v.z / (TPI * sqrt(r2 * r2 + v.w)) * vec2(r.y, -r.x);
  }
  outColor = vec4(dot(V, normalize(uS)), dot(V, normalize(uN)), length(V), 0.0);
}`;

// dye carried by the computed flow (semi-Lagrangian, Catmull-Rom): r = smoke from a rake upstream,
// g = dye released round the leading edge.  The domain turns with the flow angle, so the back-traced
// point is looked up in last frame's domain.
const SEC_DYE = /* glsl */ `
uniform sampler2D uDye, uVel, uCont;
uniform vec2 uC, uS, uN, uCp, uSp, uNp, uRes;
uniform float uDt, uTime, uRake, uInit;
uniform int uNC;
// signed distance to the section contour (chords; negative inside)
float secDist(vec2 P){
  float d = 1e9;
  int crs = 0;
  vec2 c0 = texelFetch(uCont, ivec2(0, 0), 0).xy;
  for (int j = 1; j <= uNC; j++){
    vec2 c1 = texelFetch(uCont, ivec2(j, 0), 0).xy;
    vec2 e = c1 - c0, w = P - c0;
    d = min(d, length(w - e * clamp(dot(w, e) / max(dot(e, e), 1e-12), 0.0, 1.0)));
    if ((c0.y > P.y) != (c1.y > P.y) && c0.x + (P.y - c0.y) * (c1.x - c0.x) / (c1.y - c0.y) > P.x) crs++;
    c0 = c1;
  }
  return (crs & 1) == 1 ? -d : d;
}
vec2 toUv(vec2 P, vec2 C, vec2 S, vec2 N){ vec2 o = P - C; return 0.5 + 0.5 * vec2(dot(o, S) / dot(S, S), dot(o, N) / dot(N, N)); }
vec2 velC(vec2 uv){ vec4 v = texture(uVel, uv); return v.x * normalize(uS) + v.y * normalize(uN); }
float hash1(float n){ return fract(sin(n) * 43758.5453); }
float vnoise(float x){ float i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f); return mix(hash1(i), hash1(i + 1.0), f); }
vec4 sampleCR(vec2 uv){
  vec2 p = uv * uRes - 0.5, f = fract(p);
  ivec2 b = ivec2(floor(p)) - 1;
  // Catmull-Rom weights
  vec2 t = f, t2 = t * t, t3 = t2 * t;
  vec2 q0 = -0.5 * t3 + t2 - 0.5 * t, q1 = 1.5 * t3 - 2.5 * t2 + 1.0, q2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t, q3 = 0.5 * t3 - 0.5 * t2;
  vec4 wx = vec4(q0.x, q1.x, q2.x, q3.x), wy = vec4(q0.y, q1.y, q2.y, q3.y);
  vec4 acc = vec4(0.0), lo = vec4(1e9), hi = vec4(-1e9);
  ivec2 mx = ivec2(uRes) - 1;
  for (int j = 0; j < 4; j++) for (int i = 0; i < 4; i++){
    vec4 c = texelFetch(uDye, clamp(b + ivec2(i, j), ivec2(0), mx), 0);
    acc += c * wx[i] * wy[j];
    if (i >= 1 && i <= 2 && j >= 1 && j <= 2){ lo = min(lo, c); hi = max(hi, c); }
  }
  return clamp(acc, lo, hi);
}
void main(){
  vec2 P = uC + (vUv.x * 2.0 - 1.0) * uS + (vUv.y * 2.0 - 1.0) * uN;
  float hs = length(uS), hn = length(uN);
  float s = (vUv.x * 2.0 - 1.0) * hs, n = (vUv.y * 2.0 - 1.0) * hn;
  vec4 dye = vec4(0.0);
  float q = (n + hn) / uRake, dn = abs(fract(q) - 0.5) * uRake;
  float streak = exp(-dn * dn / 1.6e-5);
  // each smoke tube puffs a little irregularly, as real smoke does
  float kq = floor(q);
  float puff = 0.45 + 0.55 * vnoise(uTime * 2.2 + kq * 13.7);
  if (uInit > 0.5){
    dye.r = streak * 0.8;
  } else {
    vec2 Pm = P - 0.5 * uDt * velC(vUv);
    vec2 Pb = P - uDt * velC(toUv(Pm, uC, uS, uN));
    vec2 uvp = toUv(Pb, uCp, uSp, uNp);
    if (all(greaterThan(uvp, vec2(0.0))) && all(lessThan(uvp, vec2(1.0)))) dye = sampleCR(uvp);
    // the rake
    if (s > -hs + 0.02 && s < -hs + 0.05) dye.r = max(dye.r, streak * puff);
    // dye round the nose
    if (length(P) < 0.014) dye.g = 0.85;
  }
  float sd = secDist(P);
  if (sd < 0.0) dye.rg = vec2(0.0);
  dye.a = sd;
  outColor = dye;
}`;
const PLANE_VS = /* glsl */ `
precision highp float;
in vec3 position;
in vec2 uv;
uniform mat4 modelViewMatrix, projectionMatrix;
out vec2 vUv;
out float vViewZ;
void main(){
  vUv = uv;
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  vViewZ = -mv.z;
  gl_Position = projectionMatrix * mv;
}`;

const PLANE_COMMON = /* glsl */ `
precision highp float;
uniform sampler2D uDepth;
uniform vec2 uRes;
uniform float uLogFar, uGain, uTime;
in vec2 vUv;
in float vViewZ;
out vec4 outColor;
bool hidden(){
  float d = texture(uDepth, gl_FragCoord.xy / uRes).r;
  float sw = d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
  return vViewZ > sw + 0.03 + 0.002 * vViewZ;
}
vec3 cpColor(float cp){
  // suction: grey-white -> blue -> deep blue (Cp -1.2); pressure: -> amber -> red (Cp +0.8)
  vec3 w = vec3(0.80, 0.80, 0.80);
  if (cp < 0.0){
    float t = clamp(-cp / 1.2, 0.0, 1.0);
    return mix(mix(w, vec3(0.22, 0.52, 0.95), smoothstep(0.0, 0.35, t)), vec3(0.05, 0.10, 0.62), smoothstep(0.35, 1.0, t));
  }
  float t = clamp(cp / 0.8, 0.0, 1.0);
  return mix(mix(w, vec3(1.0, 0.58, 0.12), smoothstep(0.0, 0.4, t)), vec3(0.80, 0.06, 0.04), smoothstep(0.4, 1.0, t));
}
float edgeFade(){ vec2 e = min(vUv, 1.0 - vUv); return smoothstep(0.0, 0.06, min(e.x, e.y)); }
`;

const SEC_FS = PLANE_COMMON + /* glsl */ `
uniform sampler2D uDye, uVel;
void main(){
  if (hidden()) discard;
  vec4 V = texture(uVel, vUv);
  vec4 d = texture(uDye, vUv);
  float fade = edgeFade();
  // background: pressure from the local speed (vortex cores show as low-pressure spots)
  float cp = 1.0 - V.z * V.z;
  vec3 col = cpColor(cp) * 0.3;
  float smoke = clamp(d.r, 0.0, 1.0), dye = clamp(d.g, 0.0, 1.0);
  col = mix(col, vec3(0.93, 0.95, 0.97), smoke * 0.95);
  col = mix(col, vec3(1.0, 0.55, 0.18), dye * 0.92);
  float a = (0.55 + 0.25 * min(abs(cp), 1.0) + 0.2 * max(smoke, dye)) * fade;
  // the cut through the wing (signed distance from the dye pass)
  float sd = d.a, fw = max(fwidth(sd), 1e-5);
  float sec = 1.0 - smoothstep(-0.5 * fw, 0.5 * fw, sd);
  float rim = 1.0 - smoothstep(0.5 * fw, 1.8 * fw, abs(sd + fw));
  col = mix(col, vec3(0.05, 0.052, 0.055), sec);
  col = mix(col, vec3(0.9), rim * 0.85);
  a = max(a, max(sec, rim) * fade);
  outColor = vec4(col * uGain * a, a);
}`;

export const FLOW_MODES = ['surface', 'section', 'wake'];

export class AirflowViz {
  /** shuttle: the Shuttle (meshes, section geometry, materials); geom: orbiter_geometry.json */
  constructor({ shuttle, geom }) {
    this.shuttle = shuttle;
    this.geom = geom;
    this.enabled = false;
    this.mode = 'surface';
    this.vis = 15;                         // display speed of the free stream (m/s)
    this.station = 0.45;                   // section station, fraction of the semispan
    this.side = -1;                        // section on the left (-1) or right wing (+1)
    this.gain = 1;
    this.sb = 0;
    this.surf = {};
    this.time = 0;
    this.haveAero = false;
    this.ready = false;
    this.info = '';
    const v3 = () => new THREE.Vector3();
    this.cur = { va: v3(), w: v3(), up: new THREE.Vector3(0, 0, -1), h: 1e4, mach: 0 };
    this.tgt = { va: v3(), w: v3(), up: new THREE.Vector3(0, 0, -1), h: 1e4, mach: 0 };
    this.scene = new THREE.Scene();
    this.layer = null;
    this.clip = new THREE.Plane(new THREE.Vector3(0, 1, 0), 1e7);
    this._wp = v3();
  }

  setSize(w, h) {
    this.layer?.dispose();
    this.layer = makeRT(w, h);
    this.res = new THREE.Vector2(w, h);
  }
  get texture() { return this.layer ? this.layer.texture : null; }

  /** static element geometry (hello message, sim/geometry.py geo.flow) */
  setGeometry(fg) {
    if (!fg || !fg.strips) return;
    this.fg = fg;
    const S = fg.strips, ns = S.length;
    this.ns = ns;
    // per-strip static data, body axes
    this.st = S.map((s) => ({
      qa: new THREE.Vector3(s[0], s[1], s[2]), qb: new THREE.Vector3(s[3], s[4], s[5]),
      chord: s[6], surf: s[7], cf: s[8] || 0, tc: s[9] || 0.1,
    }));
    for (const s of this.st) {
      s.mid = s.qa.clone().add(s.qb).multiplyScalar(0.5);
      s.width = s.surf === SURF_VT ? Math.abs(s.qb.z - s.qa.z) : Math.abs(s.qb.y - s.qa.y);
    }
    // span lists: right wing, left wing (by |y|), fin (by height), body flap (by y)
    const list = (pred, key) => S.map((_, i) => i).filter((i) => pred(this.st[i])).sort((a, b) => key(this.st[a]) - key(this.st[b]))
      .map((i) => ({ i, c: key(this.st[i]) }));
    this.lists = [
      list((s) => s.surf === SURF_WING && s.mid.y > 0, (s) => s.mid.y),
      list((s) => s.surf === SURF_WING && s.mid.y < 0, (s) => -s.mid.y),
      list((s) => s.surf === SURF_VT, (s) => -s.mid.z),
      list((s) => s.surf === SURF_BF, (s) => s.mid.y),
    ];
    this.span = Math.max(...this.st.filter((s) => s.surf === SURF_WING).map((s) => Math.max(Math.abs(s.qa.y), Math.abs(s.qb.y))));
    // dynamic per strip (smoothed): |G| (lift sense), cn terms, qr, fst
    const dyn = () => ({ gl: new Float32Array(ns), cn: new Float32Array(ns * 4), qr: new Float32Array(ns).fill(1), eq: new Float32Array(ns).fill(1),
      fst: new Float32Array(ns).fill(1), gam: new Float32Array(ns), dg: new Float32Array(ns), as: new Float32Array(ns).fill(2.5) });
    this.dyn = dyn();
    this.tdyn = dyn();
    this._initPaint();
    this._initGpu();
    this.ready = true;
  }

  /** aero message (~20 Hz): f = its flow block, m = the whole message (totals for the read-out) */
  setAero(f, m) {
    if (!f || !this.ready || !f.gam || f.gam.length !== this.ns || !f.cn) return;
    if (m) { this.lift = m.lift; this.drag = m.drag; this.nAero = (this.nAero || 0) + 1; }
    const t = this.tgt, d = this.tdyn;
    t.va.set(f.va[0], f.va[1], f.va[2]); t.w.set(f.w[0], f.w[1], f.w[2]); t.up.set(f.up[0], f.up[1], f.up[2]);
    t.h = f.h; t.mach = f.mach;
    for (let i = 0; i < this.ns; i++) {
      const c = f.cn.slice(i * 4, i * 4 + 4);
      const sum = c[0] + c[1] + c[2] + c[3];
      d.gl[i] = Math.abs(f.gam[i]) * (sum >= 0 ? 1 : -1);      // circulation, sign = lift along the strip normal
      d.gam[i] = f.gam[i];
      d.cn.set(c, i * 4);
      d.qr[i] = f.qr ? f.qr[i] : 1;
      d.eq[i] = f.eq ? f.eq[i] : 1;
      d.fst[i] = f.fst[i];
      if (f.as) d.as[i] = f.as[i];
    }
    if (!this.haveAero) this._snap();
    this.haveAero = true;
  }

  reset() { if (this.haveAero) this._snap(); this.wake?.reset(); this._secKey = null; }
  setMode(m) { this.mode = m; }
  setSurfaces(s) {
    if (!s) return;
    this.surf = s;
    this.sb = clamp(((s.speedbrake || 0) - 15) / 45, 0, 1);
  }

  _snap() {
    const c = this.cur, t = this.tgt;
    c.va.copy(t.va); c.w.copy(t.w); c.up.copy(t.up); c.h = t.h; c.mach = t.mach;
    for (const k of ['gl', 'cn', 'qr', 'eq', 'fst', 'gam', 'as']) this.dyn[k].set(this.tdyn[k]);
  }

  get active() { return this.enabled && this.ready && this.haveAero && this.cur.va.length() > 3; }

  // ================================================================ surface paint
  _initPaint() {
    const sh = this.shuttle, M = sh.mat;
    const tile = new Set([M.fuse, M.wing, M.tail, M.pod, M.flap, M.base, M.boot]);
    const bf = sh.surfaces.bodyflap?.mesh;
    this.paint = [];
    const body = this.fg.body;
    sh.model.updateMatrixWorld(true);
    const hingedMeshes = new Set(Object.values(sh.surfaces).map((s) => s.mesh));
    const modelInv = sh.model.matrixWorld.clone().invert();
    const isIdentity = (m) => m.elements.every((v, i) => Math.abs(v - (i % 5 === 0 ? 1 : 0)) < 1e-6);
    sh.model.traverse((o) => {
      if (!o.isMesh || !tile.has(o.material)) return;
      const g = o.geometry;
      const n = g.attributes.position.count;
      const a1 = new THREE.BufferAttribute(new Float32Array(n * 4), 4).setUsage(THREE.DynamicDrawUsage);
      const a2 = new THREE.BufferAttribute(new Float32Array(n * 4), 4).setUsage(THREE.DynamicDrawUsage);
      g.setAttribute('aPaint', a1);
      g.setAttribute('aPaint2', a2);
      let type;
      if (o.material === M.wing) type = 'lift';
      else if (o.material === M.tail) type = 'lift';
      else if (o === bf) type = 'lift';
      else if (o.material === M.base || o.material === M.boot) type = 'base';
      else if (/^oms_pod/.test(o.name)) type = 'pod';
      else type = 'body';
      const which = o.material === M.wing ? 'wing' : o.material === M.tail ? 'vt' : 'bf';
      const e = { mesh: o, type, n, a1, a2, which };
      // vertex positions / normals in body axes (geometry is in model coordinates, undeflected)
      const P = new Float32Array(n * 3), N = new Float32Array(n * 3);
      const pa = g.attributes.position.array, na = g.attributes.normal.array;
      // hinged control surfaces carry their geometry in model coordinates (pivot offset cancels at zero
      // deflection); anything else: its full transform into the model frame
      const hinged = hingedMeshes.has(o);
      const m = new THREE.Matrix4().copy(modelInv).multiply(o.matrixWorld);
      e.rot = hinged || isIdentity(m) ? null : new THREE.Matrix3().setFromMatrix4(m).transpose();   // model -> object
      const tv = new THREE.Vector3(), tn = new THREE.Vector3();
      const nm = new THREE.Matrix3().getNormalMatrix(m);
      for (let k = 0; k < n; k++) {
        tv.set(pa[k * 3], pa[k * 3 + 1], pa[k * 3 + 2]);
        tn.set(na[k * 3], na[k * 3 + 1], na[k * 3 + 2]);
        if (!hinged) { tv.applyMatrix4(m); tn.applyMatrix3(nm).normalize(); }
        P[k * 3] = tv.x; P[k * 3 + 1] = tv.z; P[k * 3 + 2] = -tv.y;           // model local -> FRD
        N[k * 3] = tn.x; N[k * 3 + 1] = tn.z; N[k * 3 + 2] = -tn.y;
      }
      e.P = P; e.N = N;
      e.V3 = new Float32Array(n * 3);                            // surface velocity (body axes, m/s)
      if (type === 'lift') this._initLift(e);
      else if (type === 'body' || type === 'pod') this._initBody(e, body);
      for (let k = 0; k < n; k++) a2.array[k * 4 + 1] = 1;
      a2.needsUpdate = true;
      this.paint.push(e);
    });
  }

  _interpList(L, x) {
    if (x <= L[0].c) return [L[0].i, L[0].i, 0];
    const last = L[L.length - 1];
    if (x >= last.c) return [last.i, last.i, 0];
    let k = 0;
    while (k < L.length - 2 && L[k + 1].c < x) k++;
    return [L[k].i, L[k + 1].i, (x - L[k].c) / (L[k + 1].c - L[k].c)];
  }

  _initLift(e) {
    const sh = this.shuttle, geom = this.geom, n = e.n, P = e.P, N = e.N;
    const f = (len) => new Float32Array(n * len);
    Object.assign(e, { I0: new Int16Array(n), I1: new Int16Array(n), W: f(1), SG: f(1), CT: f(3), STV: f(3), XI: f(1), CHD: f(1),
      GA: f(1), GF: f(1), GFS: f(1), GV: f(1), GC: f(1), EPS: f(1), FLE: f(1), LST: new Int8Array(n), RUD: new Uint8Array(n) });
    const bfG = geom.bodyflap, vt = geom.vtail;
    const tmp = new THREE.Vector3(), nb = new THREE.Vector3(), ct = new THREE.Vector3(), st = new THREE.Vector3();
    for (let k = 0; k < n; k++) {
      const x = P[k * 3], y = P[k * 3 + 1], z = P[k * 3 + 2];
      nb.set(N[k * 3], N[k * 3 + 1], N[k * 3 + 2]);
      // side: + on the skin facing along the strip normal (the suction side for positive load), from
      // the vertex position against the section's chord line (the loft winding, hence the normal
      // attribute, differs between the meshes)
      let li, sc, xle, chord, out, side;
      if (e.which === 'wing') {
        li = y >= 0 ? 0 : 1; sc = Math.abs(y);
        const s = sh._wingStation(clamp(sc, 2.0, geom.wing.tip_y));
        xle = s.xle; chord = s.chord;
        out = new THREE.Vector3(0, Math.sign(y) || 1, 0);
        side = (-z - s.yc) / (0.004 * chord + 0.01);                 // model y (up) = -z
      } else if (e.which === 'vt') {
        li = 2; sc = -z;
        const s = sh._finStation(clamp(-z, -vt.root.z, -vt.tip.z));
        xle = s.xle; chord = s.chord;
        out = new THREE.Vector3(0, 0, -1);
        side = y / (0.004 * chord + 0.01);
      } else {
        li = 3; sc = y;
        xle = bfG.hinge_x; chord = bfG.chord;
        out = new THREE.Vector3(0, Math.sign(y) || 1, 0);
        side = (-z - (-bfG.z + 0.15 * bfG.thickness)) / (0.05 * bfG.thickness + 0.005);
      }
      const [i0, i1, w] = this._interpList(this.lists[li], sc);
      e.I0[k] = i0; e.I1[k] = i1; e.W[k] = w; e.LST[k] = li;
      const xi = clamp((xle - x) / Math.max(chord, 0.1), 0, 1);
      e.XI[k] = xi; e.CHD[k] = chord;
      const s0 = this.st[i0], s1 = this.st[i1];
      const cf = s0.cf * (1 - w) + s1.cf * w, tc = s0.tc * (1 - w) + s1.tc * w;
      // side: + where the skin faces along the strip normal (the suction side for positive load)
      e.SG[k] = clamp(side, -1, 1);
      // chordwise (aft) and spanwise (outward) directions in the tangent plane
      ct.set(-1, 0, 0).addScaledVector(nb, nb.x); if (ct.lengthSq() < 1e-6) ct.set(-1, 0, 0); ct.normalize();
      st.copy(out).addScaledVector(nb, -nb.dot(out)).addScaledVector(ct, -ct.dot(out)); if (st.lengthSq() < 1e-6) st.copy(out); st.normalize();
      e.CT.set([ct.x, ct.y, ct.z], k * 3); e.STV.set([st.x, st.y, st.z], k * 3);
      e.GA[k] = shapeAlpha(xi);
      e.GC[k] = cumAlpha(xi);
      e.GV[k] = shapeVortex(xi);
      e.GF[k] = flapShape(cf)(xi);
      e.GFS[k] = cf > 0.01 ? ss(1 - cf - 0.02, 1 - cf + 0.02, xi) / cf : 0;
      e.EPS[k] = 1.4 * tc * (1 - xi) ** 0.7;
      const rle = 1.1019 * tc * tc;
      e.FLE[k] = Math.sqrt(xi / (xi + 0.5 * rle));
      e.RUD[k] = li === 2 && cf > 0.01 && xi > 1 - cf ? 1 : 0;
    }
  }

  _initBody(e, body) {
    const n = e.n, P = e.P;
    Object.assign(e, { K: new Float32Array(n * 4), SRC: new Float32Array(n * 3), Q: new Float32Array(n * 3), RD: new Float32Array(n * 2), FADE: new Float32Array(n) });
    // pods: crossflow around a circle on the pod axis (from the mesh's extent)
    let podC = null;
    if (e.type === 'pod') {
      let y0 = 1e9, y1 = -1e9, z0 = 1e9, z1 = -1e9;
      for (let k = 0; k < n; k++) { y0 = Math.min(y0, P[k * 3 + 1]); y1 = Math.max(y1, P[k * 3 + 1]); z0 = Math.min(z0, P[k * 3 + 2]); z1 = Math.max(z1, P[k * 3 + 2]); }
      podC = { y: (y0 + y1) / 2, z: (z0 + z1) / 2 + 0.3, r: Math.max(0.3, ((y1 - y0) + (z1 - z0)) / 4) };
    }
    const src = this.fg.src;
    for (let k = 0; k < n; k++) {
      const x = P[k * 3], y = P[k * 3 + 1], z = P[k * 3 + 2];
      let yc, zc, a, b, fade = 1;
      if (podC) { yc = podC.y; zc = podC.z; a = b = podC.r; }
      else {
        // fuselage station (nose first, x decreasing)
        let j = 0;
        if (x >= body[0][0]) j = -1;
        else if (x <= body[body.length - 1][0]) j = body.length;
        else while (j < body.length - 2 && body[j + 1][0] > x) j++;
        let r;
        if (j < 0) r = body[0];
        else if (j >= body.length) { r = body[body.length - 1]; fade = Math.exp(-(r[0] - x) / 3); }
        else { const t = (body[j][0] - x) / (body[j][0] - body[j + 1][0]); r = body[j].map((v, q) => v + (body[j + 1][q] - v) * t); }
        yc = r[1]; zc = r[2]; a = Math.max(r[3], 0.25); b = Math.max(r[4], 0.25);
      }
      e.FADE[k] = fade;
      e.Q.set([x, yc, zc], k * 3);
      const dy = y - yc, dz = z - zc;
      const rl = Math.hypot(dy, dz) || 1;
      e.RD[k * 2] = dy / rl; e.RD[k * 2 + 1] = dz / rl;
      // surface point of the ellipse at the same parametric angle: s = R e^{i th}; complex velocity per
      // unit crossflow Vy (Ubar = 1, U = 1) and Vz (Ubar = -i, U = i):  w = (Ubar - U R^2/s^2) / (1 - c^2/(4 s^2))
      const th = Math.atan2(dz / b, dy / a);
      const R = 0.5 * (a + b), c2 = a * a - b * b;
      const s2r = R * R * Math.cos(2 * th), s2i = R * R * Math.sin(2 * th);       // s^2
      const inv = (re, im) => { const m = re * re + im * im || 1e-12; return [re / m, -im / m]; };
      const [i2r, i2i] = inv(s2r, s2i);                                             // 1/s^2
      const dr = 1 - 0.25 * c2 * i2r, di = -0.25 * c2 * i2i;                         // 1 - c^2/(4 s^2)
      const div = (nr, ni) => { const m = dr * dr + di * di || 1e-12; return [(nr * dr + ni * di) / m, (ni * dr - nr * di) / m]; };
      // Vy: numerator 1 - R^2/s^2
      const [w1r, w1i] = div(1 - R * R * i2r, -R * R * i2i);
      // Vz: numerator -i - i R^2/s^2 = -i (1 + R^2/s^2)
      const pr = 1 + R * R * i2r, pi = R * R * i2i;
      const [w2r, w2i] = div(pi, -pr);
      // velocity (vy, vz) = (Re w, -Im w)
      e.K.set([w1r * fade, -w1i * fade, w2r * fade, -w2i * fade], k * 4);
      // axial thickness sources (strength U_x dA), softened
      let sx = 0, sy = 0, sz = 0;
      for (const s of src) {
        const rx = x - s[0], ry = y - s[1], rz = z - s[2];
        const r2 = rx * rx + ry * ry + rz * rz + 0.7;
        const kk = s[3] / (4 * Math.PI * r2 * Math.sqrt(r2));
        sx += kk * rx; sy += kk * ry; sz += kk * rz;
      }
      e.SRC.set([sx, sy, sz], k * 3);
    }
  }

  _updatePaint() {
    const c = this.cur, D = this.dyn, U = c.va.length(), U2 = U * U;
    const va = c.va, om = c.w, M = c.mach, ws = ss(0.85, 1.3, M);
    const cpBase = baseCp(M);
    this.cpMin = 1;
    // spanwise gradient of the circulation (lift sense) per strip, along each span list
    for (const L of this.lists) {
      for (let k = 0; k < L.length; k++) {
        const i = L[k].i;
        if (L.length < 2) { D.dg[i] = 0; continue; }
        if (k === L.length - 1) {                        // tip: the circulation drops to zero over the strip
          const s = this.st[i];
          D.dg[i] = -D.gl[i] / Math.max(s.width, 0.3);
        } else if (k === 0) D.dg[i] = (D.gl[L[1].i] - D.gl[i]) / (L[1].c - L[0].c);
        else D.dg[i] = (D.gl[L[k + 1].i] - D.gl[L[k - 1].i]) / (L[k + 1].c - L[k - 1].c);
      }
      if (L === this.lists[3]) for (const { i } of L) D.dg[i] = 0;
    }
    const u = new THREE.Vector3();
    for (const e of this.paint) {
      const A = e.a1.array, B = e.a2.array, P = e.P, Nn = e.N;
      for (let k = 0; k < e.n; k++) {
        const px = P[k * 3], py = P[k * 3 + 1], pz = P[k * 3 + 2];
        let cp, vx, vy, vz, sep = 0;
        if (e.type === 'base') {
          cp = cpBase; vx = 0; vy = 0.12 * Nn[k * 3 + 2]; vz = -0.12 * Nn[k * 3 + 1]; sep = 1;
          vx *= U; vy *= U; vz *= U;
        } else {
          // air relative to the skin point (the model's local flow: free stream + rotation)
          u.set(-(va.x + om.y * pz - om.z * py), -(va.y + om.z * px - om.x * pz), -(va.z + om.x * py - om.y * px));
          if (e.type === 'lift') {
            const i0 = e.I0[k], i1 = e.I1[k], w = e.W[k], w0 = 1 - w;
            const g = (arr, j) => arr[i0 * 4 + j] * w0 + arr[i1 * 4 + j] * w;
            const cna = g(D.cn, 0), cnf = g(D.cn, 1), cnv = g(D.cn, 2), cns = g(D.cn, 3);
            const qr = D.qr[i0] * w0 + D.qr[i1] * w, fst = D.fst[i0] * w0 + D.fst[i1] * w, dg = D.dg[i0] * w0 + D.dg[i1] * w;
            const eq = Math.max(D.eq[i0] * w0 + D.eq[i1] * w, 0.05);
            const li = e.LST[k];
            const nx = 0, ny = li === 2 ? 1 : 0, nz = li === 2 ? 0 : -1;
            const un = u.x * nx + u.y * ny + u.z * nz, uc = -u.x;
            const Up2 = Math.max(un * un + uc * uc, 1e-6), Up = Math.sqrt(Up2);
            const sg = e.SG[k];
            const dcp = cna * (e.GA[k] * (1 - ws) + ws) + cnf * (e.GF[k] * (1 - ws) + e.GFS[k] * ws);
            // speed jump +-dU with 4 eq uc dU / U^2 = qr dcp: the pressure difference is the model's load
            const dU = sg * qr * dcp * U2 / (4 * eq * Math.max(Math.abs(uc), 0.2 * U));
            // (thickness speed-up: subsonic only - linear supersonic theory has none on a symmetric section)
            let ut = e.FLE[k] * (uc * (1 + e.EPS[k] * (1 - ws)) + dU);
            const stx = e.STV[k * 3], sty = e.STV[k * 3 + 1], stz = e.STV[k * 3 + 2];
            const us = u.x * stx + u.y * sty + u.z * stz + sg * 0.5 * dg * e.GC[k];
            // (the inflow component normal to the surface counts as passing by: linear theory, no common
            // offset; the windward / leeward difference is carried by the model's load terms)
            cp = 1 - (eq * ut * ut + us * us + un * un) / U2;
            if (sg * cnv > 0) cp -= qr * Math.abs(cnv) * e.GV[k] * Math.abs(sg);
            const ctot = cna + cnf + cnv + cns;
            if (cns !== 0) cp += (sg * cns > 0 ? -0.45 : 0.55) * qr * Math.abs(cns) * Math.abs(sg);
            // separated (stalled strip): the suction side has reversed, wandering flow
            if (sg * ctot > 0 && fst < 0.98) {
              sep = (1 - fst) * Math.abs(sg);
              ut = ut * (1 - sep) - 0.25 * Up * sep;
            }
            if (e.RUD[k] && this.sb > 0) { sep = Math.max(sep, this.sb); cp += (0.25 - cp) * this.sb; }
            vx = ut * e.CT[k * 3] + us * stx; vy = ut * e.CT[k * 3 + 1] + us * sty; vz = ut * e.CT[k * 3 + 2] + us * stz;
          } else {
            // slender body: crossflow around the section + axial flow with the thickness sources
            const qx = e.Q[k * 3], qy = e.Q[k * 3 + 1], qz = e.Q[k * 3 + 2];
            const cy = -(va.y + om.z * qx - om.x * qz), cz = -(va.z + om.x * qy - om.y * qx);
            const K = e.K;
            vx = u.x + va.x * e.SRC[k * 3];
            vy = cy * K[k * 4] + cz * K[k * 4 + 2] + va.x * e.SRC[k * 3 + 1] + (1 - e.FADE[k]) * cy;
            vz = cy * K[k * 4 + 1] + cz * K[k * 4 + 3] + va.x * e.SRC[k * 3 + 2] + (1 - e.FADE[k]) * cz;
            const nx = Nn[k * 3], ny = Nn[k * 3 + 1], nz = Nn[k * 3 + 2];
            const vn = vx * nx + vy * ny + vz * nz;
            vx -= vn * nx; vy -= vn * ny; vz -= vn * nz;
            cp = 1 - (vx * vx + vy * vy + vz * vz) / U2;
            // lee side at high crossflow: separated (the model's viscous crossflow term)
            const cr = Math.hypot(cy, cz) / U;
            const lee = (e.RD[k * 2] * cy + e.RD[k * 2 + 1] * cz) / Math.max(Math.hypot(cy, cz), 1e-6);
            sep = ss(0.25, 0.6, cr) * ss(-0.2, 0.5, lee) * e.FADE[k];
            if (sep > 0) cp += (Math.max(cp, -0.6) - cp) * sep;
            if (e.FADE[k] < 0.999) { sep = Math.max(sep, 1 - e.FADE[k]); }
          }
        }
        cp = clamp(cp, -4, 1);
        let lx = vx / U, ly = -vz / U, lz = vy / U;                                     // FRD -> model local
        if (e.rot) { const r = e.rot.elements; const ox = r[0] * lx + r[3] * ly + r[6] * lz, oy = r[1] * lx + r[4] * ly + r[7] * lz; lz = r[2] * lx + r[5] * ly + r[8] * lz; lx = ox; ly = oy; }
        A[k * 4] = cp; A[k * 4 + 1] = lx; A[k * 4 + 2] = ly; A[k * 4 + 3] = lz;
        B[k * 4] = sep;
        e.V3[k * 3] = vx; e.V3[k * 3 + 1] = vy; e.V3[k * 3 + 2] = vz;
        if (e.type === 'lift' && cp < this.cpMin) this.cpMin = cp;
      }
      e.a1.needsUpdate = true; e.a2.needsUpdate = true;
    }
  }

  // ================================================================ GPU passes / planes
  _initGpu() {
    const fg = this.fg, ns = this.ns;
    const tex = (data, w, h) => {
      const t = new THREE.DataTexture(data, w, h, THREE.RGBAFormat, THREE.FloatType);
      t.minFilter = t.magFilter = THREE.NearestFilter; t.needsUpdate = true; return t;
    };
    const rtOpts = { type: THREE.HalfFloatType, depthBuffer: false, stencilBuffer: false, minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter, generateMipmaps: false };
    // section: the unsteady solver, its panels / vortices / contour as textures, velocity grid, dye
    this.NP = 140;
    this.flow2 = new SectionFlow();
    const ftex = (w, h) => { const d = new Float32Array(w * h * 4); const t = tex(d, w, h); return { d, t }; };
    this.panTex = ftex(this.flow2.n, 1);
    this.vortTex = ftex(64, Math.ceil(this.flow2.cap / 64));
    this.contTex = ftex(this.NP + 1, 1);
    this.secVelRT = new THREE.WebGLRenderTarget(512, 256, rtOpts);
    const dyeOpts = { ...rtOpts, minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter };
    this.dyeRT = [new THREE.WebGLRenderTarget(1024, 512, dyeOpts), new THREE.WebGLRenderTarget(1024, 512, dyeOpts)];
    this.dyeI = 0;
    const v2 = () => new THREE.Vector2();
    this.su = {
      uPan: { value: this.panTex.t }, uVort: { value: this.vortTex.t }, uCont: { value: this.contTex.t },
      uNP: { value: this.flow2.n - 1 }, uNV: { value: 0 }, uNC: { value: this.NP },
      uC: { value: v2() }, uS: { value: v2() }, uN: { value: v2() }, uVinf: { value: v2() },
      uCp: { value: v2() }, uSp: { value: v2() }, uNp: { value: v2() }, uRes: { value: new THREE.Vector2(1024, 512) },
      uDye: { value: null }, uVel: { value: this.secVelRT.texture },
      uDt: { value: 0 }, uTime: { value: 0 }, uRake: { value: 0.085 }, uInit: { value: 1 },
    };
    this.secVelPass = new FullscreenPass(SEC_VEL, this.su);
    this.dyePass = new FullscreenPass(SEC_DYE, this.su);
    // plane meshes (drawn into the flow layer)
    const planeMat = (fs, extra) => new THREE.RawShaderMaterial({
      vertexShader: PLANE_VS, fragmentShader: fs, glslVersion: THREE.GLSL3,
      uniforms: { uDepth: { value: null }, uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 }, uGain: { value: 1 }, uTime: { value: 0 }, ...extra },
      transparent: true, depthTest: false, depthWrite: false, side: THREE.DoubleSide,
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
      blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor, blendSrcAlpha: THREE.OneFactor, blendDstAlpha: THREE.OneMinusSrcAlphaFactor,
    });
    const quad = () => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(12), 3).setUsage(THREE.DynamicDrawUsage));
      g.setAttribute('uv', new THREE.Float32BufferAttribute([0, 0, 1, 0, 1, 1, 0, 1], 2));
      g.setIndex([0, 1, 2, 0, 2, 3]);
      return g;
    };
    this.secMesh = new THREE.Mesh(quad(), planeMat(SEC_FS, { uDye: { value: this.dyeRT[0].texture }, uVel: { value: this.secVelRT.texture } }));
    this.secMesh.frustumCulled = false; this.secMesh.matrixAutoUpdate = false; this.scene.add(this.secMesh);
    // free wake
    this.wake = new FreeWake(fg, this.lists, this.shuttle);
    this.scene.add(this.wake.mesh);
  }

  // ---------------------------------------------------------------- section: 2-D panel solution
  /** contour of the mesh's wing section at |y| (model frame X fwd, Y up), elevon deflected (deg, + TE down) */
  _sectionContour(ay, defl) {
    const sh = this.shuttle;
    const s = sh._wingStation(ay);
    const c = s.chord, xiH = 1 - s.ce / c, d = defl * Math.PI / 180;
    const hx = s.xle - xiH * c, hy = s.yc;
    const M = this.NP / 2;
    const pt = (xi, up) => {
      const ht = halfThick(xi, s.tc) * c;
      let X = s.xle - xi * c, Y = s.yc + (up ? ht * 1.18 : -ht * 0.82);
      const a = d * ss(xiH - 0.03, xiH + 0.03, xi);
      if (a !== 0) { const dx = X - hx, dy = Y - hy; X = hx + dx * Math.cos(a) - dy * Math.sin(a); Y = hy + dx * Math.sin(a) + dy * Math.cos(a); }
      return [X, Y];
    };
    const pts = [];
    for (let k = M; k >= 0; k--) pts.push(pt((1 - Math.cos(Math.PI * k / M)) / 2, false));
    for (let k = 1; k <= M; k++) pts.push(pt((1 - Math.cos(Math.PI * k / M)) / 2, true));
    pts[pts.length - 1] = pts[0].slice();
    return { pts, s };
  }

  /** section mode: drive the unsteady 2-D solver with the model's local flow at the chosen station,
   *  upload its state, recompute the velocity grid and carry the dye on */
  _section(renderer, dt, camera) {
    const sh = this.shuttle, c = this.cur, D = this.dyn, F = this.flow2, su = this.su;
    // the wing facing the camera (hysteresis around the plane of symmetry)
    const cl = sh.model.worldToLocal(this._wp.copy(camera.position));
    if (Math.abs(cl.z) > 3) this.side = cl.z > 0 ? 1 : -1;
    const side = this.side;
    const ay = clamp(this.station * this.span, 2.6, this.span - 0.4);
    const defl = this.surf[`elevon_${side > 0 ? 'r' : 'l'}${ay < 7.25 ? 'i' : 'o'}`] || 0;
    const { pts, s } = this._sectionContour(ay, defl);
    const ch = s.chord;
    // chord frame: x aft from the leading edge, y up from the chord line, in chords
    const xs = pts.map((q) => (s.xle - q[0]) / ch), ys = pts.map((q) => (q[1] - s.yc) / ch);
    // inflow at the section's quarter chord (body axes): free stream + rotation, as the model sees it
    const p = new THREE.Vector3(s.xle - 0.25 * ch, side * ay, -s.yc);
    const va = c.va, om = c.w;
    const ux = -(va.x + om.y * p.z - om.z * p.y), uz = -(va.z + om.x * p.y - om.y * p.x);
    const aGeo = Math.atan2(-uz, -ux);
    // the model at this station: normal-force coefficient of the strip (its four terms) and how much
    // of it is still attached
    const [i0, i1, w] = this._interpList(this.lists[side > 0 ? 0 : 1], ay);
    const at = (arr, k, n) => arr[i0 * n + k] * (1 - w) + arr[i1 * n + k] * w;
    const cnM = (at(D.cn, 0, 4) + at(D.cn, 1, 4) + at(D.cn, 2, 4) + at(D.cn, 3, 4)) * at(D.eq, 0, 1);
    const fst = at(D.fst, 0, 1);
    // shape: camber line and half thickness from the contour (lower / upper points at equal stations)
    const M = this.NP / 2, key = `${side}|${ay.toFixed(2)}`;
    if (this._secKey !== key || Math.abs(defl - this._secDefl) > 0.05) {
      const cx = [], ce = [], ct = [];
      for (let k = 0; k <= M; k++) {
        const lo = M - k, up = k === M ? 0 : M + k;
        cx.push(0.5 * (xs[lo] + xs[up])); ce.push(0.5 * (ys[lo] + ys[up]));
        ct.push(0.5 * Math.hypot(xs[up] - xs[lo], ys[up] - ys[lo]));
      }
      F.setShape(cx, ce, ct);
      for (let j = 0; j <= this.NP; j++) this.contTex.d.set([xs[j], ys[j], 0, 0], j * 4);
      this.contTex.t.needsUpdate = true;
      this._secDefl = defl;
      if (this._secKey !== key) {
        this._secKey = key;
        F.reset(this.secA ?? aGeo);
        this.secA = F.alpha; this.secAd = F.alpha; this.secAcc = 0; this.secCnF = cnM; this.secCnD = cnM;
        this.secHist = []; this.dcpF = new Float64Array(F.n - 1);
        su.uInit.value = 1;
      }
    }
    // angle of attack of the 2-D section: the local inflow less the downwash the model's own wing
    // implies.  Lifting line with the strip's 3-D lift slope a_s (per radian of the streamwise angle,
    // a_s / cos sweep):  a_i = C_L (1/a - 1/2pi),  C_L = cn cos a.  While attached the steady thin
    // aerofoil at the model's normal force gives the same angle exactly (taken instead, with camber
    // and flap); a small slow trim then matches the computed normal force to the model's.  After the
    // stall the 2-D section keeps the lifting-line angle - its separated force has no tip venting.
    const sI = this.st[w < 0.5 ? i0 : i1];
    const lam = Math.atan2(Math.abs(sI.qb.x - sI.qa.x), Math.max(Math.abs(sI.qb.y - sI.qa.y), 1e-3));
    const aS = Math.max(at(D.as, 0, 1), 0.5) / Math.max(Math.cos(lam), 0.3);
    const aLL = aGeo - clamp(cnM * Math.cos(aGeo) * (1 / aS - 1 / (2 * Math.PI)), -0.45, 0.45);
    const k = 0.5 * F.C1 - F.C0;
    const aAtt = Math.abs(aGeo) > Math.PI / 2 ? aLL : (Math.asin(clamp((cnM / Math.PI - k) / Math.hypot(1, k), -1, 1)) - Math.atan(k)) / 2;
    const aFF = fst * aAtt + (1 - fst) * aLL;
    this.secTrim = this.secTrim || 0;
    const dtn = Math.min(dt, 0.1) * this.vis / ch;           // chords travelled this frame
    this.secAcc = Math.min(this.secAcc + dtn, 6 * F.dt);
    let steps = 0, cnSum = 0;
    while (this.secAcc >= F.dt) {
      const aEff = clamp(aFF + this.secTrim, -Math.PI, Math.PI);
      const a0 = Math.abs(Math.sin(aEff) - F.C0 * Math.cos(aEff));
      const lesp = fst > 0.97 ? Infinity : Math.max(0.02, a0 * (0.12 + 0.95 * fst * fst));
      F.step(aEff, lesp);
      this.secA = aEff;
      this.secAcc -= F.dt; steps++;
      this.secCnD += (F.cnP - this.secCnD) * (1 - Math.exp(-F.dt / 0.25)); cnSum += this.secCnD;
      this.secCnF += (F.cnP - this.secCnF) * (1 - Math.exp(-F.dt / 1.5));
      let slope = fst * 2 * Math.PI * Math.cos(2 * aEff) + (1 - fst) * 2 * Math.cos(aEff);
      if (Math.abs(slope) < 1) slope = slope < 0 ? -1 : 1;
      this.secTrim = clamp(this.secTrim + (cnM - this.secCnF) / slope * F.dt / 2.5, -0.12, 0.12);
      for (let j = 0; j < F.n - 1; j++) this.dcpF[j] += (F.dcp[j] - this.dcpF[j]) * 0.25;
    }
    if (steps) {
      this.secHist.push([F.t, cnSum / steps, cnM]);
      while (this.secHist.length && F.t - this.secHist[0][0] > 12) this.secHist.shift();
    }
    // display domain: wind-aligned, turning slowly with the flow angle
    const prevC = su.uC.value.clone(), prevS = su.uS.value.clone(), prevN = su.uN.value.clone();
    this.secAd += (F.alpha - this.secAd) * (1 - Math.exp(-dtn / 0.6));
    const ca = Math.cos(this.secAd), sa = Math.sin(this.secAd), hs = 1.6, hn = 0.8;
    su.uC.value.set(0.5 + 0.6 * ca, 0.6 * sa);
    su.uS.value.set(ca * hs, sa * hs);
    su.uN.value.set(-sa * hn, ca * hn);
    if (su.uInit.value > 0.5) { prevC.copy(su.uC.value); prevS.copy(su.uS.value); prevN.copy(su.uN.value); }
    su.uCp.value.copy(prevC); su.uSp.value.copy(prevS); su.uNp.value.copy(prevN);
    su.uVinf.value.set(Math.cos(F.alpha), Math.sin(F.alpha));
    // solver state -> textures
    const pd = this.panTex.d;
    for (let j = 0; j < F.n; j++) pd.set([F.x[j], F.eta[j], j < F.n - 1 ? F.Gs[j] : 0, j < F.n - 1 ? F.sig[j] : 0], j * 4);
    this.panTex.t.needsUpdate = true;
    const vd = this.vortTex.d;
    for (let q = 0; q < F.nv; q++) { vd[q * 4] = F.vx[q]; vd[q * 4 + 1] = F.vy[q]; vd[q * 4 + 2] = F.vg[q]; vd[q * 4 + 3] = F.vr[q]; }
    this.vortTex.t.needsUpdate = true;
    su.uNV.value = F.nv;
    this.secVelPass.render(renderer, this.secVelRT);
    if (steps || su.uInit.value > 0.5) {
      su.uDt.value = steps * F.dt; su.uTime.value = F.t;
      su.uDye.value = this.dyeRT[this.dyeI].texture;
      this.dyeI ^= 1;
      this.dyePass.render(renderer, this.dyeRT[this.dyeI]);
      su.uInit.value = 0;
      this.secMesh.material.uniforms.uDye.value = this.dyeRT[this.dyeI].texture;
    }
    // plane quad in the model frame (local z = body y)
    const pos = this.secMesh.geometry.attributes.position;
    const Z = side * ay, C = su.uC.value, S = su.uS.value, N = su.uN.value;
    const corner = (u, v) => { const x = C.x + u * S.x + v * N.x, y = C.y + u * S.y + v * N.y; return [s.xle - x * ch, s.yc + y * ch, Z]; };
    pos.array.set([...corner(-1, -1), ...corner(1, -1), ...corner(1, 1), ...corner(-1, 1)]);
    pos.needsUpdate = true;
    this.secInfo = { y: side * ay, ch, aGeo, aEff: F.alpha, defl, fst, cnM, cnF: this.secCnF, sep: Math.abs(F.leRate) > 0.05 };
    this.clipLocal = { side, ay };
  }

  /** per frame: smooth toward the latest aero state, repaint the skin, recompute the active plane */
  update(renderer, dt, camera, sunIntensity) {
    PAINT.uPaintOn.value = this.active ? 1 : 0;
    this._clipOff();
    if (!this.active) { this.info = this.enabled ? '等待 6-DOF 仿真的气动数据…' : ''; return; }
    const c = this.cur, t = this.tgt, D = this.dyn, T = this.tdyn;
    const k = 1 - Math.exp(-Math.min(dt, 0.1) / 0.12);
    c.va.lerp(t.va, k); c.w.lerp(t.w, k); c.up.lerp(t.up, k).normalize();
    c.h += (t.h - c.h) * k; c.mach += (t.mach - c.mach) * k;
    for (const key of ['gl', 'cn', 'qr', 'eq', 'fst', 'gam', 'as']) { const a = D[key], b = T[key]; for (let i = 0; i < a.length; i++) a[i] += (b[i] - a[i]) * k; }
    this.time += dt;
    PAINT.uPaintEmit.value = 0.06 * sunIntensity + 0.1;
    this._updatePaint();
    const prev = renderer.getRenderTarget();
    if (this.mode === 'section') {
      this._section(renderer, dt, camera);
      const q = this.secInfo, R2D = 180 / Math.PI;
      this.info = `y = ${q.y.toFixed(1)} m 剖面，弦长 ${q.ch.toFixed(1)} m${Math.abs(q.defl) > 0.5 ? `，升降副翼${q.defl > 0 ? '下' : '上'}偏 ${Math.abs(q.defl).toFixed(1)}°` : ''}。` +
        `来流 ${(q.aGeo * R2D).toFixed(1)}°，扣除下洗后 ${(q.aEff * R2D).toFixed(1)}°。` +
        (q.fst < 0.5 ? '前缘分离，涡在脱落。' : q.fst < 0.97 ? '开始失速，前缘有涡脱落。' : '') +
        `法向力系数：剖面（二维）${q.cnF.toFixed(2)}，模型 ${q.cnM.toFixed(2)}` +
        (q.fst < 0.5 ? '（二维没有翼尖泄压，失速后偏大）' : '') +
        (c.mach > 0.95 ? '。剖面按不可压流计算，超声速时只作参考' : '');
    } else if (this.mode === 'wake') {
      if (this._lastMode !== 'wake') this.wake.reset();
      this.wake.update(renderer, dt, D.gam, c.va, c.w, this.vis);
      // root circulation and the spacing of the rolled-up tip vortices (centroids of the shed vorticity)
      const L = this.lists[0];
      let g0 = 0, m1 = 0;
      for (const { i } of L) g0 = Math.max(g0, Math.abs(D.gam[i]));
      for (let q = 0; q < L.length; q++) {
        const gi = Math.abs(D.gam[L[q].i]), go = q + 1 < L.length ? Math.abs(D.gam[L[q + 1].i]) : 0;
        m1 += (gi - go) * (q + 1 < L.length ? 0.5 * (L[q].c + L[q + 1].c) : this.span);
      }
      const gr = L.length ? Math.abs(D.gam[L[0].i]) : 0;
      this.info = gr > 1 ? `翼根环量 ${gr.toFixed(0)} m²/s，翼尖涡卷起后间距约 ${(2 * m1 / gr).toFixed(1)} m（翼展 ${(2 * this.span).toFixed(1)} m）` : '';
    } else {
      const L = this.lift, Dr = this.drag;
      this.info = Number.isFinite(L) ? `升力 ${(L / 1e3).toFixed(0)} kN　阻力 ${(Dr / 1e3).toFixed(0)} kN　升阻比 ${(L / Math.max(Dr, 1)).toFixed(2)}　机翼最低 Cp ${this.cpMin.toFixed(2)}` : '';
    }
    this._lastMode = this.mode;
    renderer.setRenderTarget(prev);
  }

  /** live chart in the legend: chordwise Cp of the section, or the spanwise load of the wing */
  drawChart(cv) {
    const dpr = Math.min(globalThis.devicePixelRatio || 1, 2);
    const cw = Math.round(cv.clientWidth * dpr), chh = Math.round(cv.clientHeight * dpr);
    if (cw > 0 && (cv.width !== cw || cv.height !== chh)) { cv.width = cw; cv.height = chh; }
    const g = cv.getContext('2d'), W = cv.width / dpr, H = cv.height / dpr;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    if (!this.active) return;
    const pad = { l: 34, r: 8, t: 8, b: 16 };
    const pw = W - pad.l - pad.r, ph = H - pad.t - pad.b;
    g.font = '10px ui-monospace, monospace';
    g.strokeStyle = 'rgba(255,255,255,0.18)'; g.lineWidth = 1;
    g.fillStyle = 'rgba(255,255,255,0.55)';
    const line = (pts, col, wid = 1.6) => {
      g.strokeStyle = col; g.lineWidth = wid; g.beginPath();
      pts.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y)));
      g.stroke();
    };
    if (this.mode === 'section' && this.secHist && this.secHist.length > 1) {
      // left: normal-force coefficient over the last 12 chords of travel (computed / model);
      // right: pressure difference lower - upper along the chord (smoothed over a few steps)
      const H = this.secHist, t1 = H[H.length - 1][0], F = this.flow2;
      const wl = pw * 0.56, gap = 30, wr = pw - wl - gap, xr = pad.l + wl + gap;
      let lo = 0, hi = 0.5;
      for (const [, a, b] of H) { lo = Math.min(lo, a, b); hi = Math.max(hi, a, b); }
      lo = Math.floor(lo * 2) / 2; hi = Math.ceil(hi * 2) / 2;
      const X = (t) => pad.l + (1 - (t1 - t) / 12) * wl, Y = (v) => pad.t + (hi - clamp(v, lo, hi)) / (hi - lo) * ph;
      line([[pad.l, Y(0)], [pad.l + wl, Y(0)]], 'rgba(255,255,255,0.25)', 1);
      g.fillText(`${hi}`, 4, pad.t + 8); g.fillText(`${lo}`, 4, pad.t + ph); g.fillText('Cn', 4, Y(0.5 * (lo + hi)) + 3);
      g.setLineDash([4, 3]);
      line(H.map(([t, , b]) => [X(t), Y(b)]), 'rgb(90,170,255)', 1.4);
      g.setLineDash([]);
      line(H.map(([t, a]) => [X(t), Y(a)]), 'rgba(255,255,255,0.9)', 1.2);
      g.fillStyle = 'rgba(255,255,255,0.9)'; g.fillText('剖面（实时）', pad.l + 4, pad.t + 9);
      g.fillStyle = 'rgb(90,170,255)'; g.fillText('模型', pad.l + 80, pad.t + 9);
      g.fillStyle = 'rgba(255,255,255,0.55)'; g.fillText('过去 12 倍弦长', pad.l, pad.t + ph + 12);
      const dc = this.dcpF;
      let m2 = 1;
      for (let j = 2; j < dc.length; j++) m2 = Math.max(m2, Math.abs(dc[j]));
      m2 = Math.min(Math.ceil(m2), 8);
      const XR = (x) => xr + x * wr, YR = (v) => pad.t + (m2 - clamp(v, -m2, m2)) / (2 * m2) * ph;
      line([[xr, YR(0)], [xr + wr, YR(0)]], 'rgba(255,255,255,0.25)', 1);
      const pts = [];
      for (let j = 0; j < dc.length; j++) pts.push([XR(F.mx[j]), YR(dc[j])]);
      line(pts, 'rgb(255,170,80)', 1.5);
      g.fillStyle = 'rgba(255,255,255,0.55)';
      g.fillText(`${m2}`, xr - 14, pad.t + 8); g.fillText('ΔCp', xr - 26, YR(0) + 3);
      g.fillText('前缘', xr, pad.t + ph + 12); g.fillText('后缘', xr + wr - 24, pad.t + ph + 12);
      return;
    }
    // spanwise circulation (lift per unit span / rho U), left tip to right tip; the fuselage's share is in
    // the carry-through strips
    const pts = [];
    let mx = 10;
    for (const [li, sg] of [[1, -1], [0, 1]]) {
      const L = this.lists[li].slice();
      if (sg < 0) L.reverse();
      for (const { i } of L) {
        const v = this.dyn.gl[i];
        pts.push([this.st[i].mid.y, v]); mx = Math.max(mx, Math.abs(v));
      }
    }
    mx = Math.ceil(mx * 1.15 / 50) * 50;
    const X = (y) => pad.l + (y / this.span * 0.5 + 0.5) * pw, Y = (v) => pad.t + (mx - clamp(v, -0.2 * mx, mx)) / (1.2 * mx) * ph;
    line([[X(0), pad.t], [X(0), pad.t + ph]], 'rgba(255,255,255,0.2)', 1);
    line([[pad.l, Y(0)], [pad.l + pw, Y(0)]], 'rgba(255,255,255,0.25)', 1);
    line(pts.map(([y, v]) => [X(y), Y(v)]), 'rgb(120,200,255)', 1.8);
    g.fillText(`${mx}`, 4, pad.t + 8); g.fillText('0', 4, Y(0) + 3);
    g.fillText('左翼尖', pad.l, H - 3); g.fillText('右翼尖', pad.l + pw - 36, H - 3);
    g.fillText('展向环量 Γ (m²/s)', pad.l + pw * 0.5 - 46, pad.t + 9);
  }

  _clipOff() { this.clip.set(this.clip.normal.set(0, 1, 0), 1e7); }

  /** world-space clipping plane cutting the outer wing on the camera side away at the section */
  updateClip(model) {
    const M = this.shuttle.mat.wing;
    if (!M.clippingPlanes) { M.clippingPlanes = [this.clip]; M.clipShadows = true; M.needsUpdate = true; }
    if (!(this.active && this.mode === 'section' && this.clipLocal)) { this._clipOff(); return; }
    const { side, ay } = this.clipLocal;
    // keep side * z_local <= ay  (three: points with negative distance are clipped)
    this.clip.set(new THREE.Vector3(0, 0, -side), ay);
    this.clip.applyMatrix4(model.matrixWorld);
  }

  /** draw the active plane into the flow layer; returns { texture, dist } or null */
  render(renderer, camera, depthTexture, model, exposure) {
    if (!this.active || !this.layer || !model.visible || this.mode === 'surface') return null;
    const mesh = this.mode === 'section' ? this.secMesh : this.wake.mesh;
    this.secMesh.visible = mesh === this.secMesh; this.wake.mesh.visible = mesh === this.wake.mesh;
    const u = mesh.material.uniforms;
    u.uDepth.value = depthTexture;
    u.uRes.value.set(this.layer.width, this.layer.height);
    u.uLogFar.value = Math.log2(camera.far + 1.0);
    u.uGain.value = 0.95 * this.gain / Math.max(exposure, 1e-3);
    if (u.uTime) u.uTime.value = this.time;
    if (u.uCam) u.uCam.value.copy(camera.position);
    mesh.matrix.copy(model.matrixWorld); mesh.matrixWorld.copy(model.matrixWorld);
    const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
    renderer.setRenderTarget(this.layer);
    renderer.setClearColor(0x000000, 0);
    renderer.clear(true, false, false);
    renderer.render(this.scene, camera);
    renderer.setClearColor(cc, ca);
    return { texture: this.layer.texture, dist: camera.position.distanceTo(model.getWorldPosition(this._wp)) };
  }
}
