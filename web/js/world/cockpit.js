// Orbiter forward flight deck, seen from the commander's seat.
//
// A complete, closed 3D crew-station model built procedurally in the cockpit frame (origin at the
// commander's eye, X forward, Y up, Z right; the cabin centreline is at z = +0.55 m, the pilot's eye
// at z = +1.10 m).  It lives in its own scene around a local origin at the vehicle (floating origin:
// the world is tens of km across, and float32 world positions would make the millimetre-scale cabin
// shadows shimmer).  A camera with the main camera's projection and orientation renders it into the
// same HDR target and depth buffer after the world, so it occludes clouds and terrain; it gets sun light and shadows
// (the sun only enters through the windows), sky light only through the windows (baked per-vertex
// window form factors with ray-traced visibility), ambient occlusion, cabin flood light at night,
// the integral (lettering) lighting of the panels and the glow of the displays.
//
// Contents (after the MEDS-era orbiter, OV-103/104/105, 2000-2011):
//   * six forward windows W1-W6 with thick frames, mullion fairings and deep three-pane reveals,
//     two overhead windows W7/W8
//   * glareshield with the F2/F3/F4 panels (flight-control mode pushbutton indicators, master alarm,
//     drag chute arm/deploy/jettison, HUD power), black anti-glare top, two HUD combiners
//   * F6/F7/F8 instrument panels with the nine forward MEDS MDUs (CDR1/2, CRT1/3/2, MFD1/2, PLT1/2)
//     showing live pages, caution & warning matrix, landing gear arm/down and talkbacks, event timer
//   * C2/C3 centre console (DPS keyboards, DAP pushbuttons), L1/L2 and R1/R2 side panels with the
//     speedbrake/throttle controller, O1-O9 overhead panels with meters, switches, circuit breakers
//     and the mission timer, rotational hand controllers, seats, aft bulkhead
//   * a glass pass on the windows: Fresnel reflections of the displays and the cabin, sun-lit dust
//     and scratches; rain on the outer panes is simulated drop by drop and refracted (windshield_rain.js)
import * as THREE from 'three';
import { patchStandard } from './materials.js';
import { paintPanel, legendAtlas, DynCanvas, structureCanvas } from './cockpit_art.js';
import { buildPanelSpecs, LEGENDS, MDU_W, MDU_H } from './cockpit_panels.js';
import { WindshieldRain } from './windshield_rain.js';

const CL = 0.55;                                    // cabin centreline (cockpit frame z)
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const mz = (p) => V(p.x, p.y, 2 * CL - p.z);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const ss = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const bil = (a, b, c, d, s, t) => {                // a (0,0)  b (1,0)  c (1,1)  d (0,1)
  const p = a.clone().lerp(b, s), q = d.clone().lerp(c, s);
  return p.lerp(q, t);
};

// ------------------------------------------------------------------ windshield geometry (cockpit frame)
// sill / header posts, left to right: W1 aft edge, W1|W2, W2|W3, centre post (mirrored for the pilot).
// The pilots look straight ahead through W3 / W4 (inner panes ~35 in diagonal); W2 / W5 are the
// forward-quarter windows and W1 / W6 the side windows.
const SILL_L = [V(0.05, -0.10, -0.86), V(0.62, -0.17, -0.70), V(0.93, -0.188, -0.22), V(0.98, -0.193, CL)];
const HEAD_L = [V(-0.12, 0.18, -0.70), V(0.22, 0.28, -0.55), V(0.46, 0.32, -0.18), V(0.50, 0.32, CL)];
const SILL = [...SILL_L, mz(SILL_L[2]), mz(SILL_L[1]), mz(SILL_L[0])];
const HEAD = [...HEAD_L, mz(HEAD_L[2]), mz(HEAD_L[1]), mz(HEAD_L[0])];
const POST_W = [0.055, 0.045, 0.040, 0.034, 0.040, 0.045, 0.055];    // half widths of the posts
const CABIN_C = V(0.05, -0.20, CL);                 // a point inside the cabin (orientation reference)
const FLOOR = -1.27;                                // eye ~50 in above the flight deck floor

// main instrument panel plane (F6/F7/F8): MDU faces ~30 in ahead of the eye, display reference line
// 16 in below it, panel tilted back 15 deg
const mp = (z, y) => V(0.76 + (y + 0.41) * 0.268, y, z);
const PANEL_TOP = -0.197;
// glareshield: aft lip, aft face (F2/F3/F4) down to its underside
// the lip sits ~9.7 deg below the eye: the window sills are just hidden behind it, so the over-the-nose
// view reaches the lip without a black band of glareshield top in between
const LIP = { x: 0.67, y: -0.115 }, GS_BOT = -0.195;
const gf = (z, y) => V(LIP.x + (LIP.y - y) * 0.3375, y, z);
const GZ0 = -0.40, GZ1 = 2 * CL - GZ0;               // glareshield extent
// overhead panel plane (front edge -> aft edge)
const OH_F = { x: 0.48, y: 0.34 }, OH_A = { x: 0.05, y: 0.50 };
const OH_Z0 = -0.20, OH_Z1 = 2 * CL - OH_Z0;
// side panels: F1 corner panel (under W2) and L1 (under W1), commander side
const F1P = { TL: V(0.53, -0.185, -0.69), TR: V(0.80, -0.21, -0.365), BL: V(0.45, -0.545, -0.69) };
const L1P = { TL: V(0.04, -0.13, -0.85), TR: V(0.53, -0.185, -0.69), BL: V(0.04, -0.43, -0.92) };
const CONS = { y: -0.48, x0: -0.45, x1: 0.38, zin: -0.42, zout: -0.96 };     // left side console
const oh = (z, f) => V(OH_F.x + (OH_A.x - OH_F.x) * f, OH_F.y + (OH_A.y - OH_F.y) * f, z);

// ------------------------------------------------------------------ geometry accumulator
class Acc {
  constructor() { this.P = []; this.N = []; this.UV = []; this.C = []; this.S = []; this.I = []; }
  get n() { return this.P.length / 3; }
  _v(p, n, u, v, ch, smp) {
    this.P.push(p.x, p.y, p.z); this.N.push(n.x, n.y, n.z); this.UV.push(u, v); this.C.push(ch); this.S.push(smp);
    return this.n - 1;
  }
  /** bilinear patch, subdivided; normal oriented toward o.toward (default: cabin interior) */
  patch(a, b, c, d, o = {}) {
    const seg = o.seg ?? 0.07;
    const su = o.su ?? Math.max(1, Math.ceil(Math.max(a.distanceTo(b), d.distanceTo(c)) / seg));
    const sv = o.sv ?? Math.max(1, Math.ceil(Math.max(a.distanceTo(d), b.distanceTo(c)) / seg));
    const uv = o.uv || [0, 0, 1, 1];
    const ctr = a.clone().add(b).add(c).add(d).multiplyScalar(0.25);
    const n0 = new THREE.Vector3().crossVectors(b.clone().sub(a).add(c.clone().sub(d)), d.clone().sub(a).add(c.clone().sub(b)));
    let flip = false;
    if (o.outward) flip = n0.dot(ctr.clone().sub(o.outward)) < 0;
    else if (o.toward !== null) flip = n0.dot((o.toward || CABIN_C).clone().sub(ctr)) < 0;
    const base = this.n;
    const ds = new THREE.Vector3(), dt = new THREE.Vector3(), nn = new THREE.Vector3();
    for (let j = 0; j <= sv; j++) for (let i = 0; i <= su; i++) {
      const s = i / su, t = j / sv;
      const p = bil(a, b, c, d, s, t);
      ds.copy(b).sub(a).lerp(c.clone().sub(d), t);
      dt.copy(d).sub(a).lerp(c.clone().sub(b), s);
      nn.crossVectors(ds, dt).normalize();
      if (flip) nn.negate();
      this._v(p, nn, uv[0] + s * (uv[2] - uv[0]), uv[1] + t * (uv[3] - uv[1]), o.ch || 0, o.smp ?? -1);
    }
    for (let j = 0; j < sv; j++) for (let i = 0; i < su; i++) {
      const i00 = base + j * (su + 1) + i, i10 = i00 + 1, i01 = i00 + su + 1, i11 = i01 + 1;
      if (!flip) this.I.push(i00, i10, i11, i00, i11, i01); else this.I.push(i00, i11, i10, i00, i01, i11);
    }
  }
  /** convex polygon as a fan, normal toward o.toward */
  poly(pts, o = {}) {
    const ctr = pts.reduce((s, p) => s.add(p), V(0, 0, 0)).multiplyScalar(1 / pts.length);
    const n = new THREE.Vector3();
    for (let i = 0; i < pts.length; i++) n.add(new THREE.Vector3().crossVectors(pts[i].clone().sub(ctr), pts[(i + 1) % pts.length].clone().sub(ctr)));
    n.normalize();
    let flip = false;
    if (o.toward !== null) flip = n.dot((o.toward || CABIN_C).clone().sub(ctr)) < 0;
    if (flip) n.negate();
    const c0 = this._v(ctr, n, 0.5, 0.5, o.ch || 0, o.smp ?? -1);
    const ids = pts.map((p) => this._v(p, n, 0.5, 0.5, o.ch || 0, o.smp ?? -1));
    for (let i = 0; i < ids.length; i++) {
      const a = ids[i], b = ids[(i + 1) % ids.length];
      if (!flip) this.I.push(c0, a, b); else this.I.push(c0, b, a);
    }
  }
  /** append a three geometry transformed by m */
  geom(g, m, o = {}) {
    const pos = g.attributes.position, nor = g.attributes.normal, uv = g.attributes.uv;
    const nm = new THREE.Matrix3().getNormalMatrix(m);
    const base = this.n, p = new THREE.Vector3(), n = new THREE.Vector3();
    const uvr = o.uv;                           // remap [0..1] uv into a rect
    for (let i = 0; i < pos.count; i++) {
      p.fromBufferAttribute(pos, i).applyMatrix4(m);
      n.fromBufferAttribute(nor, i).applyMatrix3(nm).normalize();
      let u = uv ? uv.getX(i) : 0, v = uv ? uv.getY(i) : 0;
      if (uvr) { u = uvr[0] + u * (uvr[2] - uvr[0]); v = uvr[1] + v * (uvr[3] - uvr[1]); }
      this._v(p, n, u, v, o.ch || 0, o.smp ?? -1);
    }
    if (g.index) for (let i = 0; i < g.index.count; i++) this.I.push(base + g.index.getX(i));
    else for (let i = 0; i < pos.count; i++) this.I.push(base + i);
  }
  build() {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(this.P, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(this.N, 3));
    g.setAttribute('uv', new THREE.Float32BufferAttribute(this.UV, 2));
    g.setAttribute('aCh', new THREE.Float32BufferAttribute(this.C, 1));
    g.setIndex(this.I);
    g.userData.smp = this.S;
    return g;
  }
}

// ------------------------------------------------------------------ primitive shapes
function rrShape(w, h, r) {
  const s = new THREE.Shape(), x = -w / 2, y = -h / 2;
  r = Math.min(r, w / 2 - 1e-4, h / 2 - 1e-4);
  s.moveTo(x + r, y); s.lineTo(x + w - r, y); s.quadraticCurveTo(x + w, y, x + w, y + r);
  s.lineTo(x + w, y + h - r); s.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  s.lineTo(x + r, y + h); s.quadraticCurveTo(x, y + h, x, y + h - r);
  s.lineTo(x, y + r); s.quadraticCurveTo(x, y, x + r, y);
  return s;
}
/** rounded box centred on the origin: w (x) h (y) d (z) */
function rrBox(w, h, d, r = 0.003, bevel = 0.0015) {
  const b = Math.min(bevel, d / 2 - 1e-4, w / 4, h / 4);
  const g = new THREE.ExtrudeGeometry(rrShape(w - 2 * b, h - 2 * b, r), { depth: Math.max(1e-4, d - 2 * b), bevelEnabled: b > 0, bevelThickness: b, bevelSize: b, bevelSegments: 2, curveSegments: 3 });
  g.translate(0, 0, -(d - 2 * b) / 2);
  return g;
}
/** rounded plate with a rectangular window (MDU bezel): hole centre (hx, hy), size hw x hh */
function rrFrame(w, h, d, r, hx, hy, hw, hh, hr = 0.002) {
  const sh = rrShape(w, h, r);
  const hole = new THREE.Path(), x = hx - hw / 2, y = hy - hh / 2;
  hole.moveTo(x + hr, y); hole.quadraticCurveTo(x, y, x, y + hr); hole.lineTo(x, y + hh - hr); hole.quadraticCurveTo(x, y + hh, x + hr, y + hh);
  hole.lineTo(x + hw - hr, y + hh); hole.quadraticCurveTo(x + hw, y + hh, x + hw, y + hh - hr); hole.lineTo(x + hw, y + hr); hole.quadraticCurveTo(x + hw, y, x + hw - hr, y); hole.lineTo(x + hr, y);
  sh.holes.push(hole);
  const b = 0.0015;
  const g = new THREE.ExtrudeGeometry(sh, { depth: d - 2 * b, bevelEnabled: true, bevelThickness: b, bevelSize: b * 0.8, bevelSegments: 2, curveSegments: 3 });
  g.translate(0, 0, -(d - 2 * b) / 2);
  return g;
}
const CACHE = {};
function cyl(rt, rb, h, seg = 12) {           // along +z, base at z=0
  const k = `${rt}|${rb}|${h}|${seg}`;
  if (!CACHE[k]) { const g = new THREE.CylinderGeometry(rt, rb, h, seg, 1, false); g.rotateX(Math.PI / 2); g.translate(0, 0, h / 2); CACHE[k] = g; }
  return CACHE[k];
}
function sphere(r) { const k = `s${r}`; if (!CACHE[k]) CACHE[k] = new THREE.SphereGeometry(r, 10, 8); return CACHE[k]; }

// ------------------------------------------------------------------ panel frame (placement basis)
class PanelFrame {
  /** TL, TR, BL: plate front-surface corners; plates stand 4 mm proud of their mounting surface */
  constructor(spec, TL, TR, BL) {
    this.spec = spec;
    this.ex = TR.clone().sub(TL).normalize();
    this.ey = TL.clone().sub(BL).normalize();
    this.ez = new THREE.Vector3().crossVectors(this.ex, this.ey).normalize();
    // re-orthogonalise ey (plates are rectangles)
    this.ey.crossVectors(this.ez, this.ex).normalize();
    this.TL = TL.clone();
    this.w = spec.w; this.h = spec.h;
  }
  P(u, v, z = 0) { return this.TL.clone().addScaledVector(this.ex, u).addScaledVector(this.ey, -v).addScaledVector(this.ez, z); }
  M(u, v, z = 0, rotZ = 0, tilt = 0) {
    const m = new THREE.Matrix4().makeBasis(this.ex, this.ey, this.ez).setPosition(this.P(u, v, z));
    if (rotZ) m.multiply(new THREE.Matrix4().makeRotationZ(rotZ));
    if (tilt) m.multiply(new THREE.Matrix4().makeRotationX(tilt));
    return m;
  }
}

// ------------------------------------------------------------------ cabin material (window-lit interior)
const FRAG_COMMON = /* glsl */ `
uniform float uChan[32];
uniform vec3 uCabin;        // inter-reflected (bounce) irradiance, cabin average
uniform vec3 uFlood;        // flood-light irradiance (night)
uniform vec3 uMduGlow;      // irradiance per unit display form factor
uniform float uPanelLit;    // integral (lettering) lighting
uniform float uSkyK;        // sky light through the windows
uniform float uLegendK;     // pushbutton legend luminance
uniform float uScreenK;     // display luminance
uniform sampler2D uStructTex;
varying vec3 vL;
varying vec3 vCkP;
varying vec3 vCkN;
flat varying int vCh;`;

function cabinPatch(sh, kind) {
  sh.vertexShader = sh.vertexShader
    .replace('#include <common>', '#include <common>\nattribute vec3 aL;\nattribute float aCh;\nvarying vec3 vL;\nvarying vec3 vCkP;\nvarying vec3 vCkN;\nflat varying int vCh;')
    .replace('#include <begin_vertex>', '#include <begin_vertex>\nvL = aL; vCh = int(aCh + 0.5); vCkP = position; vCkN = normal;')
    .replace('vWPos = (modelMatrix * wpc).xyz;', 'vWPos = (modelMatrix * wpc).xyz + uWorldOffset;')
    .replace('varying vec3 vWPos;', 'varying vec3 vWPos;\nuniform vec3 uWorldOffset;');
  let f = sh.fragmentShader.replace('#include <common>', '#include <common>\n' + FRAG_COMMON);
  f = f.replace('#include <map_fragment>', `float letterLit = 0.0; vec3 legendTex = vec3(0.0);
#include <map_fragment>
#ifdef USE_MAP
  ${kind === 'legend' ? `legendTex = sampledDiffuseColor.rgb; diffuseColor.rgb = vec3(0.006) + vec3(dot(legendTex, vec3(0.30, 0.55, 0.15))) * 0.05;   // unlit: smoked lens, legend a faint grey` : ''}
  ${kind === 'paint' ? `letterLit = smoothstep(0.80, 0.85, min(sampledDiffuseColor.r, min(sampledDiffuseColor.g, sampledDiffuseColor.b)));` : ''}
#endif
diffuseColor.a = 1.0;
${kind === 'struct' || kind === 'frame' || kind === 'floor' || kind === 'flock' ? `
  {
    vec3 tw = pow(abs(normalize(vCkN)), vec3(4.0)); tw /= (tw.x + tw.y + tw.z);
    vec3 sx = texture2D(uStructTex, vCkP.zy * ${kind === 'floor' ? '1.6' : '1.0'} + 0.31).rgb;
    vec3 sy = texture2D(uStructTex, vCkP.xz * ${kind === 'floor' ? '1.6' : '1.0'} + 0.57).rgb;
    vec3 sz = texture2D(uStructTex, vCkP.xy * ${kind === 'floor' ? '1.6' : '1.0'} + 0.13).rgb;
    vec3 st = sx * tw.x + sy * tw.y + sz * tw.z;
    diffuseColor.rgb *= mix(1.0, st.r * 2.0, ${kind === 'flock' ? '0.35' : '1.0'});
  }` : ''}`);
  f = f.replace('#include <emissivemap_fragment>', `#include <emissivemap_fragment>
  ${kind === 'paint' ? 'totalEmissiveRadiance += letterLit * uPanelLit * vec3(1.0, 0.96, 0.86);' : ''}
  ${kind === 'legend' ? 'totalEmissiveRadiance += legendTex * uChan[vCh] * uLegendK;' : ''}
  ${kind === 'screen' ? 'totalEmissiveRadiance *= uScreenK * (0.62 + 0.38 * pow(abs(dot(normal, normalize(vViewPosition))), 0.7));' : ''}
  ${kind === 'led' ? 'totalEmissiveRadiance *= uLegendK * 1.4;' : ''}`);
  f = f.replace('#include <lights_fragment_maps>', `#include <lights_fragment_maps>
  {
    float sky = vL.x * uSkyK;
    iblIrradiance *= sky;
    radiance *= clamp(sky * 2.5, 0.0, 1.0) * mix(0.35, 1.0, vL.y);
    irradiance += (uCabin + uFlood) * vL.y + uMduGlow * vL.z;
  }`);
  sh.fragmentShader = f;
}

// ------------------------------------------------------------------ lighting bake helpers
/** form factor from a differential area (p, n) to a polygon (Lambert), polygon clipped to the hemisphere */
function formFactor(p, n, poly) {
  // clip against the tangent plane
  const pts = [];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const da = n.dot(_t1.copy(a).sub(p)), db = n.dot(_t1.copy(b).sub(p));
    if (da > 1e-4) pts.push(a);
    if ((da > 1e-4) !== (db > 1e-4)) pts.push(a.clone().lerp(b, (1e-4 - da) / (db - da)));
  }
  if (pts.length < 3) return 0;
  let s = 0;
  for (let i = 0; i < pts.length; i++) {
    const r0 = _t1.copy(pts[i]).sub(p).normalize(), r1 = _t2.copy(pts[(i + 1) % pts.length]).sub(p).normalize();
    const g = Math.acos(clamp(r0.dot(r1), -1, 1));
    const c = _t3.crossVectors(r0, r1);
    const L = c.length();
    if (L > 1e-9) s += g * n.dot(c) / L;
  }
  return Math.abs(s) / (2 * Math.PI);
}
const _t1 = new THREE.Vector3(), _t2 = new THREE.Vector3(), _t3 = new THREE.Vector3();

class OccGrid {
  constructor(lo, hi, cs) {
    this.lo = lo; this.cs = cs;
    this.nx = Math.ceil((hi.x - lo.x) / cs); this.ny = Math.ceil((hi.y - lo.y) / cs); this.nz = Math.ceil((hi.z - lo.z) / cs);
    this.g = new Uint8Array(this.nx * this.ny * this.nz);
  }
  mark(p) {
    const i = Math.floor((p.x - this.lo.x) / this.cs), j = Math.floor((p.y - this.lo.y) / this.cs), k = Math.floor((p.z - this.lo.z) / this.cs);
    if (i < 0 || j < 0 || k < 0 || i >= this.nx || j >= this.ny || k >= this.nz) return;
    this.g[(k * this.ny + j) * this.nx + i] = 1;
  }
  tri(a, b, c) {
    const L = Math.max(a.distanceTo(b), b.distanceTo(c), c.distanceTo(a));
    const n = Math.max(1, Math.ceil(L / (this.cs * 0.6)));
    for (let i = 0; i <= n; i++) for (let j = 0; j <= n - i; j++) {
      const u = i / n, v = j / n;
      _t1.set(a.x + (b.x - a.x) * u + (c.x - a.x) * v, a.y + (b.y - a.y) * u + (c.y - a.y) * v, a.z + (b.z - a.z) * u + (c.z - a.z) * v);
      this.mark(_t1);
    }
  }
  occ(x, y, z) {
    const i = Math.floor((x - this.lo.x) / this.cs), j = Math.floor((y - this.lo.y) / this.cs), k = Math.floor((z - this.lo.z) / this.cs);
    if (i < 0 || j < 0 || k < 0 || i >= this.nx || j >= this.ny || k >= this.nz) return 0;
    return this.g[(k * this.ny + j) * this.nx + i];
  }
  /** true if the segment p -> q (skipping the first `skip` metres) is free */
  clear(p, q, skip, stopShort = 0) {
    const dx = q.x - p.x, dy = q.y - p.y, dz = q.z - p.z, L = Math.hypot(dx, dy, dz);
    const step = this.cs * 0.5, end = L - stopShort;
    for (let t = skip; t < end; t += step) {
      const f = t / L;
      if (this.occ(p.x + dx * f, p.y + dy * f, p.z + dz * f)) return false;
    }
    return true;
  }
}

// ------------------------------------------------------------------ panel MDU assignment (entry / landing)
const MDU_SPECS = {
  CDR1: { port: 'CDR 1', idp: 1, page: 'PFD', menu: 'FLT', res: 768, always: true },
  CDR2: { port: 'CDR 2', idp: 2, page: 'HSIT', menu: 'HSIT', res: 768, rng: 2 },
  CRT1: { port: 'CRT 1', idp: 1, page: 'VSIT', menu: 'DPS', res: 640 },
  CRT3: { port: 'CRT 3', idp: 3, page: 'GSUM', menu: 'DPS', res: 640 },
  CRT2: { port: 'CRT 2', idp: 2, page: 'HZS', menu: 'DPS', res: 640 },
  MFD1: { port: 'MFD 1', idp: 1, page: 'SPI', menu: 'SUB', res: 512, rate: 0.4 },
  MFD2: { port: 'MFD 2', idp: 2, page: 'HSIT', menu: 'HSIT', res: 512, rng: 4, sw: { orient: 1 }, rate: 0.5 },
  PLT1: { port: 'PLT 1', idp: 2, page: 'PFD', menu: 'FLT', res: 512, rate: 0.4 },
  PLT2: { port: 'PLT 2', idp: 2, page: 'HSIT', menu: 'HSIT', res: 512, rng: 3, rate: 0.5 },
};
const REFLECT_MDUS = ['CDR1', 'CDR2', 'CRT1', 'CRT3', 'CRT2', 'MFD1'];

// DPS keyboard: 8 rows x 4 columns
const KB_KEYS = ['FAULT SUMM', 'SYS SUMM', 'MSG RESET', 'ACK', 'GPC/ CRT', 'A', 'B', 'C', 'I/O RESET', 'D', 'E', 'F', 'ITEM', '1', '2', '3',
  'EXEC', '4', '5', '6', 'OPS', '7', '8', '9', 'SPEC', '−', '0', '+', 'RESUME', 'CLEAR', '.', 'PRO'];
KB_KEYS.forEach((s, i) => LEGENDS.push({ key: `kb${i}`, lines: s.split(' '), color: '#e8e8e2', aspect: 1.5 }));

const CH_NAMES = ['off', 'on', 'ma', 'pitchAC_A', 'pitchAC_B', 'rollAC_A', 'rollAC_B', 'sbAM_A', 'sbAM_B', 'bfAM_A', 'bfAM_B',
  'gearArm', 'gearDn', 'dcArm', 'dcDpy', 'dcJett', 'abort', 'meStatus', 'dap', 'cw', 'hud', 'cwGpc'];
// C&W lamps share the lamp-test channel, except GPC (row 5, col 5) which follows the GNC fault messages
const chIndex = (name) => {
  if (!name) return 0;
  const i = CH_NAMES.indexOf(name);                  // exact channel names first ('cwGpc' is not a lamp)
  if (i >= 0) return i;
  if (name === 'cw4_4') return CH_NAMES.indexOf('cwGpc');
  if (name.startsWith('cw')) return CH_NAMES.indexOf('cw');
  return 0;
};

// =========================================================================================== Cockpit
export class Cockpit {
  constructor({ U, meds }) {
    this.U = U; this.meds = meds;
    this.group = new THREE.Group();
    this.group.name = 'cockpit';
    this.group.visible = false;
    this.built = false;
    this.uni = {
      uChan: { value: new Float32Array(32) }, uCabin: { value: new THREE.Color() }, uFlood: { value: new THREE.Color() },
      uMduGlow: { value: new THREE.Color() }, uPanelLit: { value: 0 }, uSkyK: { value: 1 }, uLegendK: { value: 1 }, uScreenK: { value: 0.5 },
      uStructTex: { value: null }, uWorldOffset: { value: new THREE.Vector3() },
    };
    // local scene: cockpit, its own sun (tight shadow frustum around the cabin) and camera
    this.scene = new THREE.Scene();
    this.scene.add(this.group);
    this.cam = new THREE.PerspectiveCamera();
    this.cam.matrixAutoUpdate = true;
    this.sun = new THREE.DirectionalLight(0xffffff, 1);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    Object.assign(this.sun.shadow.camera, { left: -2.3, right: 2.3, top: 2.3, bottom: -2.3, near: 16, far: 24 });
    this.sun.shadow.camera.updateProjectionMatrix();
    // 2.2 mm texels: offset along the normal by ~3 texels so grazing sun on the panels does not acne
    this.sun.shadow.bias = -0.0003;
    this.sun.shadow.normalBias = 0.007;
    this.scene.add(this.sun, this.sun.target);
    this.origin = new THREE.Vector3();
    this.units = {};
    this.screens = {};          // id -> { o, u, v (cockpit frame), tex }
    this.keys = [];             // clickable MDU edge keys { unit, i, c, hu, hv, ex, ey, ez }
    this.buttons = [];          // other clickable pushbuttons (MASTER ALARM)
    this.apertures = [];        // window polygons (cockpit frame) for the bake and the glass pass
    this.hudCombiner = null;    // { corners: [4] }
    this._chuteSeen = false;
    this.headOffset = new THREE.Vector3();
  }

  // ------------------------------------------------------------------ materials
  _mat(kind, o = {}) {
    const m = new THREE.MeshStandardMaterial({ color: o.color ?? 0x808080, roughness: o.rough ?? 0.7, metalness: o.metal ?? 0, map: o.map || null, emissive: o.emissive ?? 0x000000, emissiveMap: o.emissiveMap || null });
    if (o.map) o.map.anisotropy = 8;
    m.shadowSide = THREE.DoubleSide;
    patchStandard(m, this.U, { key: 'cockpit-' + kind, uniforms: this.uni, fragment: (sh) => cabinPatch(sh, kind) });
    m.customProgramCacheKey = () => 'cockpit-' + kind;
    return m;
  }

  // ------------------------------------------------------------------ build
  build() {
    if (this.built) return;
    const t0 = performance.now();
    const stx = new THREE.CanvasTexture(structureCanvas());
    stx.wrapS = stx.wrapT = THREE.RepeatWrapping; stx.anisotropy = 8;
    this.uni.uStructTex.value = stx;
    const M = this.M = {
      struct: this._mat('struct', { color: 0x575b60, rough: 0.78 }),          // grey structure / trim
      frame: this._mat('frame', { color: 0x1d1f22, rough: 0.72 }),            // window frames (dark anti-glare)
      flock: this._mat('flock', { color: 0x1c1d1f, rough: 0.95 }),            // glareshield top, reveals
      metal: this._mat('metal', { color: 0xd7dadd, rough: 0.28, metal: 1.0 }), // toggle levers, guards
      black: this._mat('black', { color: 0x141516, rough: 0.45 }),             // knobs, housings, CBs
      white: this._mat('white', { color: 0xe8e8e4, rough: 0.5 }),              // knob index marks, CB collars
      bezel: this._mat('bezel', { color: 0x2b2d30, rough: 0.55 }),             // MDU bezels
      key: this._mat('key', { color: 0x3a3c3f, rough: 0.6 }),
      seat: this._mat('seat', { color: 0x474c53, rough: 0.95 }),               // seat cushions (fabric)
      belt: this._mat('belt', { color: 0x5e5543, rough: 0.9 }),                // harness webbing
      floor: this._mat('floor', { color: 0x2c2e31, rough: 0.9 }),
      orange: this._mat('orange', { color: 0xd85a1a, rough: 0.6 }),
    };
    const A = this.A = {};
    for (const k of Object.keys(M)) A[k] = new Acc();

    // legends (pushbuttons, C&W, keyboard) and dynamic indicators
    this.legend = legendAtlas(LEGENDS);
    const legTex = new THREE.CanvasTexture(this.legend.canvas); legTex.colorSpace = THREE.SRGBColorSpace; legTex.anisotropy = 8;
    M.legend = this._mat('legend', { color: 0xffffff, rough: 0.35, map: legTex, emissive: 0x000000 });
    A.legend = new Acc();
    this.dyn = new DynCanvas([{ key: 'tbN', kind: 'tb' }, { key: 'tbL', kind: 'tb' }, { key: 'tbR', kind: 'tb' }, { key: 'tbN2', kind: 'tb' }, { key: 'tbL2', kind: 'tb' }, { key: 'tbR2', kind: 'tb' }, { key: 'evt', kind: 'seg' }, { key: 'met', kind: 'seg' }]);
    this.dyn.draw({ tbN: 'up', tbL: 'up', tbR: 'up', tbN2: 'up', tbL2: 'up', tbR2: 'up', evt: '00:00', met: '000/00:00:00' });
    this.dynTex = new THREE.CanvasTexture(this.dyn.c); this.dynTex.colorSpace = THREE.SRGBColorSpace;
    M.tb = this._mat('tb', { color: 0xffffff, rough: 0.5, map: this.dynTex });
    M.led = this._mat('led', { color: 0x050505, rough: 0.3, emissive: 0xffffff, emissiveMap: this.dynTex });
    A.tb = new Acc(); A.led = new Acc();

    this._buildWindows();
    this._buildShell();
    const specs = buildPanelSpecs();
    this._buildPanels(specs);
    this._buildHud();
    this._buildSeatsAndControls();

    // meshes
    this.meshes = [];
    for (const [k, acc] of Object.entries(A)) {
      if (!acc.n) continue;
      const mesh = new THREE.Mesh(acc.build(), M[k]);
      mesh.castShadow = k !== 'legend' && k !== 'led' && k !== 'tb';
      mesh.receiveShadow = true;
      mesh.name = 'cockpit-' + k;
      this.meshes.push(mesh); this.group.add(mesh);
    }
    for (const pm of this.panelMeshes) { this.meshes.push(pm); this.group.add(pm); }
    for (const sm of this.screenMeshes) { this.meshes.push(sm); this.group.add(sm); }
    this._bake();
    this._buildGlass();
    this.built = true;
    console.info(`[cockpit] built in ${(performance.now() - t0).toFixed(0)} ms: ${this.meshes.reduce((s, m) => s + m.geometry.index.count / 3, 0)} triangles`);
  }

  // ------------------------------------------------------------------ windows W1..W6 + frames
  _buildWindows() {
    const A = this.A;
    this.bays = [];
    for (let i = 0; i < 6; i++) {
      const a = SILL[i], b = SILL[i + 1], c = HEAD[i + 1], d = HEAD[i];
      const wB = a.distanceTo(b), wT = d.distanceTo(c), hL = a.distanceTo(d);
      const s0 = POST_W[i] / Math.min(wB, wT), s1 = 1 - POST_W[i + 1] / Math.min(wB, wT);
      const side = i === 0 || i === 5;
      const t0 = (side ? 0.050 : 0.030) / hL, t1 = 1 - (side ? 0.050 : 0.058) / hL;
      // inward normal
      const ctr = bil(a, b, c, d, 0.5, 0.5);
      const nIn = new THREE.Vector3().crossVectors(b.clone().sub(a), d.clone().sub(a)).normalize();
      if (nIn.dot(CABIN_C.clone().sub(ctr)) < 0) nIn.negate();
      const ap = [bil(a, b, c, d, s0, t0), bil(a, b, c, d, s1, t0), bil(a, b, c, d, s1, t1), bil(a, b, c, d, s0, t1)];
      // frame ring
      const sub = (sa, sb, ta, tb) => [bil(a, b, c, d, sa, ta), bil(a, b, c, d, sb, ta), bil(a, b, c, d, sb, tb), bil(a, b, c, d, sa, tb)];
      const fo = { toward: ctr.clone().addScaledVector(nIn, 1.0), seg: 0.08 };
      A.frame.patch(...sub(0, 1, 0, t0), fo);
      A.frame.patch(...sub(0, 1, t1, 1), fo);
      A.frame.patch(...sub(0, s0, t0, t1), fo);
      A.frame.patch(...sub(s1, 1, t0, t1), fo);
      // deep reveal through the three panes (pressure, redundant, thermal).  The walls flare outward
      // along the pilots' lines of sight (as the real window openings are bevelled), so the reveal
      // frames the view without cutting into it -- a reveal square to the raked glass would hide
      // everything below ~5 deg.
      const depth = 0.115, apC = ap.reduce((s, p) => s.add(p), V(0, 0, 0)).multiplyScalar(0.25);
      const eyeFor = i < 3 ? V(0, 0, 0) : V(0, 0, 2 * CL);
      const along = (p, dn) => {                        // point dn deeper (along -nIn) on the flared wall
        const r = p.clone().sub(eyeFor).normalize(), c = Math.max(0.35, -r.dot(nIn));
        const dir = r.multiplyScalar(0.9 / c).addScaledVector(nIn, -0.1);
        return p.clone().addScaledVector(dir, dn);
      };
      const outer = ap.map((p) => along(p, depth));
      const axis = apC.clone().addScaledVector(nIn, -depth / 2);
      for (let k = 0; k < 4; k++) {
        const k1 = (k + 1) % 4;
        // inner lip (frame colour) then the black reveal
        const lip = [ap[k], ap[k1], along(ap[k1], 0.018), along(ap[k], 0.018)];
        A.frame.patch(lip[0], lip[1], lip[2], lip[3], { toward: axis, seg: 0.2 });
        A.flock.patch(lip[3], lip[2], outer[k1], outer[k], { toward: axis, seg: 0.2 });
      }
      // a dark gasket ring at the inner pane
      const gk = ap.map((p) => along(p, 0.022));
      const gkC = gk.reduce((s, p) => s.add(p), V(0, 0, 0)).multiplyScalar(0.25);
      const gi = gk.map((p) => p.clone().lerp(gkC, 0.02));
      for (let k = 0; k < 4; k++) { const k1 = (k + 1) % 4; A.black.patch(gk[k], gk[k1], gi[k1], gi[k], { toward: apC.clone().addScaledVector(nIn, 0.5), seg: 0.3 }); }
      this.bays.push({ a, b, c, d, nIn, ap, outer, apC });
      this.apertures.push({ name: `W${i + 1}`, poly: ap.map((p) => p.clone().addScaledVector(nIn, -0.03)), inner: gi, outer, nIn });
    }
    // mullion fairings on the posts between windows (raised, with a rounded face)
    for (let k = 1; k <= 5; k++) {
      const s = SILL[k], h = HEAD[k];
      const n1 = this.bays[k - 1].nIn, n2 = this.bays[k].nIn;
      const nb = n1.clone().add(n2).normalize();
      const dir = h.clone().sub(s).normalize();
      const side = new THREE.Vector3().crossVectors(dir, nb).normalize();
      const hw = POST_W[k] * 0.78, out = k === 3 ? 0.022 : 0.030;
      const pt = (p, a, o) => p.clone().addScaledVector(side, a).addScaledVector(nb, o);
      const sA = s.clone().addScaledVector(dir, 0.02), hA = h.clone().addScaledVector(dir, -0.02);
      const tw = { toward: s.clone().lerp(h, 0.5).addScaledVector(nb, 1.0), seg: 0.1 };
      A.frame.patch(pt(sA, -hw * 0.55, out), pt(sA, hw * 0.55, out), pt(hA, hw * 0.55, out), pt(hA, -hw * 0.55, out), tw);
      A.frame.patch(pt(sA, -hw * 1.25, 0), pt(sA, -hw * 0.55, out), pt(hA, -hw * 0.55, out), pt(hA, -hw * 1.25, 0), { toward: pt(s.clone().lerp(h, 0.5), -hw * 3, 0.6), seg: 0.1 });
      A.frame.patch(pt(sA, hw * 0.55, out), pt(sA, hw * 1.25, 0), pt(hA, hw * 1.25, 0), pt(hA, hw * 0.55, out), { toward: pt(s.clone().lerp(h, 0.5), hw * 3, 0.6), seg: 0.1 });
      // bolt heads along the fairing
      for (let j = 1; j < 7; j++) {
        const p = pt(sA.clone().lerp(hA, j / 7), 0, out + 0.0005);
        const m = new THREE.Matrix4().lookAt(V(0, 0, 0), nb.clone().negate(), dir).setPosition(p);
        A.metal.geom(cyl(0.0035, 0.0038, 0.0025, 6), m, { smp: -1 });
      }
      // grab handle on the W2|W3 and W4|W5 posts
      if (k === 2 || k === 4) {
        const p0 = pt(s.clone().lerp(h, 0.30), 0, out), p1 = pt(s.clone().lerp(h, 0.62), 0, out);
        this._bar(A.struct, p0, p0.clone().addScaledVector(nb, 0.045), 0.007);
        this._bar(A.struct, p1, p1.clone().addScaledVector(nb, 0.045), 0.007);
        this._bar(A.struct, p0.clone().addScaledVector(nb, 0.045), p1.clone().addScaledVector(nb, 0.045), 0.009);
      }
    }
    // overhead windows W7 / W8 (in the ceiling above and behind the seats)
    this.ovw = [];
    for (const zc of [0.27, 2 * CL - 0.27]) {
      const x0 = -0.74, x1 = -0.26, z0 = zc - 0.24, z1 = zc + 0.24, y = OH_A.y;          // 20 x 20 in clear view
      const ap = [V(x0, y, z0), V(x1, y, z0), V(x1, y, z1), V(x0, y, z1)];
      const out = ap.map((p) => p.clone().add(V(0, 0.11, 0)).lerp(V((x0 + x1) / 2, y + 0.11, zc), 0.05));
      const axis = V((x0 + x1) / 2, y + 0.05, zc);
      for (let k = 0; k < 4; k++) { const k1 = (k + 1) % 4; A.flock.patch(ap[k], ap[k1], out[k1], out[k], { toward: axis, seg: 0.2 }); }
      this.ovw.push({ x0, x1, z0, z1 });
      this.apertures.push({ name: 'W' + (7 + this.ovw.length - 1), poly: ap.map((p) => p.clone().add(V(0, 0.03, 0))), inner: ap.map((p) => p.clone().add(V(0, 0.02, 0))), outer: out, nIn: V(0, -1, 0) });
    }
  }

  /** cylinder from p0 to p1 */
  _bar(acc, p0, p1, r, smp = -1) {
    const d = p1.clone().sub(p0), L = d.length();
    if (L < 1e-5) return;
    const q = new THREE.Quaternion().setFromUnitVectors(V(0, 0, 1), d.divideScalar(L));
    acc.geom(cyl(r, r, 1, 10), new THREE.Matrix4().compose(p0, q, V(1, 1, L)), { smp });
  }

  // ------------------------------------------------------------------ cabin structure
  _buildShell() {
    const A = this.A;
    const lip = (z) => V(LIP.x, LIP.y, z);
    const sillAt = (z) => {                          // point on the windshield sill polyline at z
      for (let i = 0; i < 6; i++) {
        const p = SILL[i], q = SILL[i + 1];
        if ((z - p.z) * (z - q.z) <= 0 && Math.abs(q.z - p.z) > 1e-6) return p.clone().lerp(q, (z - p.z) / (q.z - p.z));
      }
      return null;
    };
    // ---- glareshield top (black anti-glare), from the lip forward to the window sills
    const gzs = [GZ0, SILL[2].z, SILL[3].z, SILL[4].z, GZ1];
    for (let i = 0; i < 4; i++) {
      const z0 = gzs[i], z1 = gzs[i + 1];
      A.flock.patch(lip(z0), lip(z1), sillAt(z1).add(V(0, -0.003, 0)), sillAt(z0).add(V(0, -0.003, 0)), { toward: V(0.75, 0.6, CL), seg: 0.08 });
    }
    this._bar(A.flock, lip(GZ0).add(V(0.006, -0.006, 0)), lip(GZ1).add(V(0.006, -0.006, 0)), 0.009);     // rounded lip edge
    // ---- glareshield aft face backing (F2/F3/F4 plates) and underside
    A.struct.patch(gf(GZ0, LIP.y - 0.002), gf(GZ1, LIP.y - 0.002), gf(GZ1, GS_BOT), gf(GZ0, GS_BOT), { toward: V(0, 0, CL) });
    A.frame.patch(gf(GZ0, GS_BOT), gf(GZ1, GS_BOT), mp(GZ1, GS_BOT), mp(GZ0, GS_BOT), { toward: V(0.5, -0.6, CL) });
    for (const zc of [GZ0, GZ1]) {                    // end caps
      const sgn = zc < CL ? -1 : 1;
      A.frame.patch(lip(zc), sillAt(zc), mp(zc, PANEL_TOP).add(V(0.02, 0, 0)), gf(zc, GS_BOT), { toward: V(0.7, -0.1, zc - sgn), seg: 0.1 });
    }
    // ---- main instrument panel backing (F6/F7/F8), centre column, knee wells, rudder pedals
    A.struct.patch(mp(GZ0, PANEL_TOP), mp(GZ1, PANEL_TOP), mp(GZ1, -0.685), mp(GZ0, -0.685), { toward: V(0, 0, CL) });
    A.struct.patch(mp(0.185, -0.685), mp(0.915, -0.685), mp(0.915, -0.712), mp(0.185, -0.712), { toward: V(0, 0, CL) });
    for (const [za, zb] of [[-0.42, 0.33], [0.77, 2 * CL + 0.42]]) {
      A.floor.patch(mp(za, -0.685), mp(zb, -0.685), V(1.02, -0.74, zb), V(1.02, -0.74, za), { toward: V(0.4, -1.4, (za + zb) / 2) });
      A.floor.patch(V(1.02, -0.74, za), V(1.02, -0.74, zb), V(1.02, FLOOR, zb), V(1.02, FLOOR, za), { toward: V(0, -1.0, (za + zb) / 2) });
      for (const zw of [za, zb]) {
        const inward = zw === za ? 1 : -1;
        A.floor.poly([mp(zw, -0.685), V(1.02, -0.74, zw), V(1.02, FLOOR, zw), V(0.62, FLOOR, zw), V(0.62, -0.70, zw)], { toward: V(0.8, -0.9, zw + inward) });
      }
      const zc = za < CL ? 0 : 2 * CL;
      for (const dz of [-0.12, 0.12]) A.black.geom(rrBox(0.09, 0.22, 0.025, 0.012, 0.004), new THREE.Matrix4().makeRotationZ(0.55).setPosition(V(0.92, -1.13, zc + dz)));
    }
    // ---- side structure: window shelves, corner (F1/F5) and L1/R1 mounting, side consoles, walls
    for (const side of [-1, 1]) {
      const M = side < 0 ? (p) => p.clone() : mz;
      const sa = SILL[side < 0 ? 0 : 6], s1 = SILL[side < 0 ? 1 : 5], ha = HEAD[side < 0 ? 0 : 6], h1 = HEAD[side < 0 ? 1 : 5], h2 = HEAD[side < 0 ? 2 : 4];
      const gEnd = sillAt(side < 0 ? GZ0 : GZ1);
      const F1TL = M(F1P.TL), F1TR = M(F1P.TR), F1BL = M(F1P.BL), F1BR = M(F1P.TR.clone().add(F1P.BL).sub(F1P.TL));
      const L1TL = M(L1P.TL), L1TR = M(L1P.TR), L1BL = M(L1P.BL), L1BR = M(L1P.TR.clone().add(L1P.BL).sub(L1P.TL));
      const eye = M(V(0, 0, 0));
      // shelves under W2 and W1
      A.struct.patch(F1TL, F1TR, gEnd, s1, { toward: M(V(0.3, 0.4, 0.2)), seg: 0.1 });
      A.struct.patch(L1TL, L1TR, s1, sa, { toward: M(V(0.1, 0.4, 0.2)), seg: 0.1 });
      A.frame.poly([F1TR, gEnd, mp(side < 0 ? GZ0 : GZ1, PANEL_TOP)], { toward: M(V(0.4, 0.3, 0.2)) });
      // backings
      A.struct.patch(F1TL, F1TR, F1BR, F1BL, { toward: eye });
      A.struct.patch(L1TL, L1TR, L1BR, L1BL, { toward: eye });
      // strip between F1 and the main panel's outboard edge
      A.struct.patch(F1TR, mp(side < 0 ? GZ0 : GZ1, PANEL_TOP), mp(side < 0 ? GZ0 : GZ1, -0.55), F1BR, { toward: eye });
      // skirts down to the floor
      A.struct.patch(F1BL, F1BR, F1BR.clone().setY(FLOOR), F1BL.clone().setY(FLOOR), { toward: M(V(0, -0.9, 0)) });
      A.struct.patch(L1BL, L1BR, L1BR.clone().setY(CONS.y), L1BL.clone().setY(CONS.y), { toward: eye });
      A.struct.poly([L1TR, L1BR, F1BL, F1TL], { toward: eye });
      // outboard closure under the side panels, forward of the console (no daylight through gaps)
      A.struct.patch(L1BR, F1BL, F1BL.clone().setY(FLOOR), L1BR.clone().setY(FLOOR), { toward: eye });
      A.floor.patch(M(V(CONS.x1, FLOOR + 0.002, -0.99)), M(V(1.02, FLOOR + 0.002, -0.99)), M(V(1.02, -0.40, -0.99)), M(V(CONS.x1, -0.40, -0.99)), { toward: eye });
      A.struct.patch(L1BL.clone().setY(CONS.y), L1BR.clone().setY(CONS.y), L1BR.clone().setY(FLOOR), M(V(CONS.x1, FLOOR, -0.96)), { toward: eye });
      // side console: top, inner face, front face
      const zin = M(V(0, 0, CONS.zin)).z, zout = M(V(0, 0, CONS.zout)).z;
      A.struct.patch(V(CONS.x0, CONS.y, zout), V(CONS.x1, CONS.y, zout), V(CONS.x1, CONS.y, zin), V(CONS.x0, CONS.y, zin), { toward: V(0, 0.5, zin) });
      A.struct.patch(V(CONS.x0, FLOOR, zin), V(CONS.x1, FLOOR, zin), V(CONS.x1, CONS.y, zin), V(CONS.x0, CONS.y, zin), { toward: V(0, -0.8, CL) });
      A.struct.patch(V(CONS.x1, FLOOR, zout), V(CONS.x1, FLOOR, zin), V(CONS.x1, CONS.y, zin), V(CONS.x1, CONS.y, zout), { toward: V(1.0, -0.8, zin) });
      A.struct.patch(V(CONS.x1, CONS.y, zout), V(CONS.x1, CONS.y, zin), F1BL.clone().setZ(zin).setY(CONS.y), F1BL.clone().setZ(zout).setY(CONS.y), { toward: V(0.5, 0.5, zin) });
      // side wall aft of L1 / W1
      A.struct.patch(M(V(-1.06, CONS.y, -0.98)), M(V(0.04, CONS.y, -0.95)), L1TL, M(V(-1.06, -0.13, -0.88)), { toward: M(V(0, -0.2, 0.2)) });
      A.struct.patch(M(V(-1.06, -0.13, -0.88)), L1TL, sa, M(V(-1.06, -0.10, -0.87)), { toward: M(V(0, -0.2, 0.2)) });
      A.struct.patch(M(V(-1.06, -0.10, -0.87)), sa, ha, M(V(-1.06, 0.18, -0.72)), { toward: M(V(-0.2, 0.0, 0.2)) });
      A.struct.patch(M(V(-1.06, FLOOR, -0.98)), M(V(CONS.x0, FLOOR, -0.98)), M(V(CONS.x0, CONS.y, -0.98)), M(V(-1.06, CONS.y, -0.98)), { toward: M(V(-0.5, -0.8, 0.2)) });
      // forward overhead closures next to the W1/W2 headers, ceiling slope
      const oF = oh(M(V(0, 0, OH_Z0)).z, 0), oA = oh(M(V(0, 0, OH_Z0)).z, 1), cA = V(-0.12, OH_A.y, oA.z);
      A.struct.patch(h1, h2, oF, oA, { toward: M(V(0.1, 0.0, 0.3)), seg: 0.1 });
      A.struct.patch(ha, h1, oA, cA, { toward: M(V(0.0, 0.0, 0.3)), seg: 0.1 });
      A.struct.patch(M(V(-1.06, 0.18, -0.72)), ha, cA, V(-1.06, OH_A.y, oA.z), { toward: M(V(-0.3, 0.0, 0.3)) });
      // cable tray / handrail along the side wall
      this._bar(A.black, M(V(-0.95, 0.10, -0.75)), M(V(-0.18, 0.12, -0.71)), 0.013);
    }
    // ---- overhead: strip between the window header and the overhead plates' front edge
    for (let i = 2; i <= 3; i++) {
      const h0 = HEAD[i], h1 = HEAD[i + 1];
      A.frame.patch(h0, h1, oh(h1.z, 0).add(V(0, 0.004, 0)), oh(clamp(h0.z, OH_Z0, OH_Z1), 0).add(V(0, 0.004, 0)), { toward: V(0.1, -0.3, CL), seg: 0.1 });
      const g0 = mz(h0), g1 = mz(h1);
      A.frame.patch(g1, g0, oh(clamp(g0.z, OH_Z0, OH_Z1), 0).add(V(0, 0.004, 0)), oh(g1.z, 0).add(V(0, 0.004, 0)), { toward: V(0.1, -0.3, CL), seg: 0.1 });
    }
    A.struct.patch(oh(OH_Z0, 0).add(V(0, 0.007, 0)), oh(OH_Z1, 0).add(V(0, 0.007, 0)), oh(OH_Z1, 1).add(V(0, 0.007, 0)), oh(OH_Z0, 1).add(V(0, 0.007, 0)), { toward: V(0, -0.5, CL) });
    // ---- ceiling (with the overhead windows), floor, aft bulkhead
    const cy = OH_A.y;
    const xs = [-1.06, -0.74, -0.26, -0.12, OH_A.x], zs = [OH_Z0, 0.03, 0.51, 0.59, 1.07, OH_Z1];
    for (let i = 0; i < 4; i++) for (let j = 0; j < 5; j++) {
      if (i === 1 && (j === 1 || j === 3)) continue;
      A.struct.patch(V(xs[i], cy, zs[j]), V(xs[i + 1], cy, zs[j]), V(xs[i + 1], cy, zs[j + 1]), V(xs[i], cy, zs[j + 1]), { toward: V(-0.3, -0.5, CL), seg: 0.15 });
    }
    A.floor.patch(V(-1.06, FLOOR, -0.98), V(1.02, FLOOR, -0.98), V(1.02, FLOOR, 2 * CL + 0.98), V(-1.06, FLOOR, 2 * CL + 0.98), { toward: V(0, 0, CL), seg: 0.25 });
    // aft bulkhead: outline follows the side walls and the ceiling slopes
    const prof = [[-0.98, FLOOR], [-0.98, CONS.y], [-0.88, -0.13], [-0.87, -0.10], [-0.72, 0.18], [OH_Z0, OH_A.y]];
    for (let i = 0; i < prof.length - 1; i++) {
      const [za, ya] = prof[i], [zb, yb] = prof[i + 1];
      A.struct.patch(V(-1.06, ya, za), V(-1.06, ya, 2 * CL - za), V(-1.06, yb, 2 * CL - zb), V(-1.06, yb, zb), { toward: V(0, 0, CL), seg: 0.2 });
    }
    // ---- centre console sides (C2 / C3 plates are panels)
    for (const z of [0.33, 0.77]) {
      A.struct.poly([mp(z, -0.708), V(0.468, -0.812, z), V(-0.05, -0.862, z), V(-0.05, FLOOR, z), V(0.70, FLOOR, z), V(0.66, -0.76, z)], { toward: V(0, -0.8, z < CL ? 0 : 1.1) });
    }
    A.struct.patch(V(-0.05, -0.862, 0.33), V(-0.05, -0.862, 0.77), V(-0.05, FLOOR, 0.77), V(-0.05, FLOOR, 0.33), { toward: V(-0.6, -0.9, CL) });
  }

  // ------------------------------------------------------------------ panels and their hardware
  _buildPanels(specs) {
    const P = {};
    const onPlane = (f, z0, yTop, h, yRef) => { const TL = f(z0, yTop), dd = f(z0, yRef).sub(TL).normalize(); return [TL, TL.clone().add(dd.multiplyScalar(h))]; };
    const place = (spec, TL, TR, BL, lift = 0.004) => {
      const fr = new PanelFrame(spec, TL, TR, BL);
      fr.TL.addScaledVector(fr.ez, lift);
      P[spec.id] = fr;
      return fr;
    };
    // forward instrument panels and glareshield face
    let TL, BL;
    [TL, BL] = onPlane(mp, -0.335, PANEL_TOP - 0.002, specs.F6.h, -0.9); place(specs.F6, TL, mp(0.180, PANEL_TOP), BL);
    [TL, BL] = onPlane(mp, 0.185, PANEL_TOP - 0.002, specs.F7.h, -0.9); place(specs.F7, TL, mp(0.915, PANEL_TOP), BL);
    [TL, BL] = onPlane(mp, 0.920, PANEL_TOP - 0.002, specs.F8.h, -0.9); place(specs.F8, TL, mp(1.435, PANEL_TOP), BL);
    [TL, BL] = onPlane(gf, GZ0, LIP.y - 0.006, specs.F2.h, -0.3); place(specs.F2, TL, gf(GZ0 + 0.6, LIP.y - 0.006), BL);
    [TL, BL] = onPlane(gf, 0.205, LIP.y - 0.006, specs.F3.h, -0.3); place(specs.F3, TL, gf(0.895, LIP.y - 0.006), BL);
    [TL, BL] = onPlane(gf, GZ1 - 0.6, LIP.y - 0.006, specs.F4.h, -0.3); place(specs.F4, TL, gf(GZ1, LIP.y - 0.006), BL);
    // corner and side panels (pilot side mirrored, keeping the lettering readable)
    place(specs.F1, F1P.TL, F1P.TR, F1P.BL);
    const F1BR = F1P.TR.clone().add(F1P.BL).sub(F1P.TL);
    place(specs.F5, mz(F1P.TR), mz(F1P.TL), mz(F1BR));
    place(specs.L1, L1P.TL, L1P.TR, L1P.BL);
    const L1BR = L1P.TR.clone().add(L1P.BL).sub(L1P.TL);
    place(specs.R1, mz(L1P.TR), mz(L1P.TL), mz(L1BR));
    place(specs.L2, V(0.10, CONS.y, -0.86), V(0.10, CONS.y, -0.50), V(-0.36, CONS.y, -0.86));
    place(specs.R2, V(0.10, CONS.y, 2 * CL + 0.50), V(0.10, CONS.y, 2 * CL + 0.86), V(-0.36, CONS.y, 2 * CL + 0.50));
    place(specs.C2, V(0.676, -0.706, 0.33), V(0.676, -0.706, 0.77), V(0.468, -0.810, 0.33));
    place(specs.C3, V(0.466, -0.812, 0.33), V(0.466, -0.812, 0.77), V(-0.050, -0.860, 0.33));
    // overhead: top edge = aft edge (readable looking up)
    let z = OH_Z0 + 0.003;
    const ohLen = Math.hypot(OH_A.x - OH_F.x, OH_A.y - OH_F.y);
    for (const id of ['O1', 'O2', 'O3', 'O8', 'O9']) {
      const s = specs[id], f0 = 0.012;
      place(s, oh(z, f0 + s.h / ohLen), oh(z + s.w, f0 + s.h / ohLen), oh(z, f0));
      z += s.w + 0.0045;
    }
    // aft bulkhead panels (seen when looking back)
    specs.A1 = { id: 'A1', w: 0.62, h: 0.55, items: [{ t: 'id', s: 'A1' }, { t: 'box', x: 0.02, y: 0.03, w: 0.58, h: 0.24, title: 'PAYLOAD / RMS' }, { t: 'mdu', x: 0.20, y: 0.30, w: MDU_W, h: MDU_H, id: 'AFD1' }] };
    specs.A2 = { id: 'A2', w: 0.62, h: 0.55, items: [{ t: 'id', s: 'A2' }, { t: 'box', x: 0.02, y: 0.03, w: 0.58, h: 0.24, title: 'AFT FLIGHT STATION' }, { t: 'win', x: 0.10, y: 0.33, w: 0.42, h: 0.18 }] };
    place(specs.A1, V(-1.055, 0.15, 0.48), V(-1.055, 0.15, -0.14), V(-1.055, -0.40, 0.48), 0.003);
    place(specs.A2, V(-1.055, 0.15, 1.24), V(-1.055, 0.15, 0.62), V(-1.055, -0.40, 1.24), 0.003);
    this.frames = P;

    // paint and build each plate
    this.panelMeshes = [];
    this.screenMeshes = [];
    for (const [id, fr] of Object.entries(P)) {
      const spec = fr.spec;
      const isOH = id.startsWith('O') || id.startsWith('A');
      const art = paintPanel(spec, isOH ? 1150 : 1500);
      const tex = new THREE.CanvasTexture(art.canvas); tex.colorSpace = THREE.SRGBColorSpace; tex.anisotropy = 8;
      const mat = this._mat('paint', { color: 0xffffff, rough: 0.62, map: tex });
      const acc = new Acc();
      const BL = fr.P(0, spec.h), BR = fr.P(spec.w, spec.h), TR = fr.P(spec.w, 0), TLp = fr.P(0, 0);
      acc.patch(BL, BR, TR, TLp, { toward: TLp.clone().add(fr.ez), seg: 0.06 });
      // plate edges (thickness), structure colour
      const th = -0.006;
      const e = [[TLp, TR], [TR, BR], [BR, BL], [BL, TLp]];
      const pc = fr.P(spec.w / 2, spec.h / 2, th / 2);
      for (const [p, q] of e) this.A.struct.patch(p, q, q.clone().addScaledVector(fr.ez, th), p.clone().addScaledVector(fr.ez, th), { outward: pc, seg: 0.3 });
      const mesh = new THREE.Mesh(acc.build(), mat);
      mesh.castShadow = true; mesh.receiveShadow = true; mesh.name = 'panel-' + id;
      this.panelMeshes.push(mesh);
      this._hardware(fr);
    }
  }

  /** switches, knobs, breakers, pushbuttons, displays mounted on a panel */
  _hardware(fr) {
    const A = this.A, spec = fr.spec;
    const smpOf = (u, v) => this._sample(fr.P(u, v, 0.01), fr.ez);
    for (const it of spec.items) {
      switch (it.t) {
        case 'tg': {
          const smp = smpOf(it.x, it.y);
          A.metal.geom(cyl(0.0060, 0.0060, 0.0030, 6), fr.M(it.x, it.y, 0, Math.PI / 6), { smp });       // hex nut
          A.metal.geom(cyl(0.0034, 0.0034, 0.0070, 10), fr.M(it.x, it.y, 0), { smp });                  // bushing
          const st = it.st ?? 0;
          const ang = st * 0.40;
          const lm = fr.M(it.x, it.y, 0.0065, 0, -ang);
          A.metal.geom(cyl(0.0015, 0.0021, 0.0185, 8), lm, { smp });                                    // lever
          A.metal.geom(sphere(0.0022), lm.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0, 0.0185)), { smp });
          if (it.guard) {
            for (const dx of [-0.0085, 0.0085]) A.metal.geom(rrBox(0.0015, 0.022, 0.016, 0.001, 0.0004), fr.M(it.x + dx, it.y, 0.008), { smp });
            A.metal.geom(rrBox(0.0185, 0.0015, 0.016, 0.001, 0.0004), fr.M(it.x, it.y + 0.0105, 0.008), { smp });
          }
          break;
        }
        case 'kn': {
          const smp = smpOf(it.x, it.y), r = it.r || 0.009;
          A.black.geom(cyl(r * 1.15, r * 1.25, 0.004, 20), fr.M(it.x, it.y, 0), { smp });
          A.black.geom(cyl(r * 0.78, r * 0.85, 0.013, 16), fr.M(it.x, it.y, 0.004), { smp });
          const rot = ((it.a0 ?? -60) + ((it.a1 ?? 60) - (it.a0 ?? -60)) * 0.5) * Math.PI / 180;
          A.black.geom(rrBox(r * 0.45, r * 2.0, 0.012, 0.001, 0.0008), fr.M(it.x, it.y, 0.010, -rot), { smp });  // pointer bar
          A.white.geom(rrBox(r * 0.12, r * 0.9, 0.0012, 0.0002, 0.0002), fr.M(it.x, it.y, 0.0165, -rot).multiply(new THREE.Matrix4().makeTranslation(0, r * 0.5, 0)), { smp });
          break;
        }
        case 'cb': {
          const smp = smpOf(it.x, it.y);
          A.black.geom(cyl(0.0040, 0.0044, it.out ? 0.010 : 0.006, 12), fr.M(it.x, it.y, 0), { smp });
          if (it.out) A.white.geom(cyl(0.0041, 0.0041, 0.0035, 12), fr.M(it.x, it.y, 0.0015), { smp });
          break;
        }
        case 'pb': case 'lens': {
          const smp = smpOf(it.x, it.y);
          const w = it.w || 0.019, h = it.h || 0.019, isLens = it.t === 'lens' || it.lens;
          const proud = isLens ? 0.003 : 0.009;
          A.black.geom(rrBox(w + 0.004, h + 0.004, proud + 0.001, 0.0015, 0.0008), fr.M(it.x, it.y, (proud + 0.001) / 2), { smp });
          // lens face with the legend (split legends: two channels)
          const L = LEGENDS.find((l) => l.key === it.leg);
          const r = this.legend.rect(it.leg);
          const z = proud + 0.0012;
          const c0 = fr.P(it.x - w / 2, it.y + h / 2, z), c1 = fr.P(it.x + w / 2, it.y + h / 2, z), c2 = fr.P(it.x + w / 2, it.y - h / 2, z), c3 = fr.P(it.x - w / 2, it.y - h / 2, z);
          const tw = fr.P(it.x, it.y, 1);
          if (it.ch === 'ma') this.buttons.push({ act: 'ma', c: fr.P(it.x, it.y, proud), ex: fr.ex.clone(), ey: fr.ey.clone(), ez: fr.ez.clone(), hu: w / 2 + 0.003, hv: h / 2 + 0.003 });
          if (L && L.split) {
            const mid0 = c0.clone().lerp(c3, 0.5), mid1 = c1.clone().lerp(c2, 0.5), vm = (r[1] + r[3]) / 2;
            A.legend.patch(c0, c1, mid1, mid0, { uv: [r[0], r[1], r[2], vm], ch: chIndex(it.ch + '_B'), smp, su: 1, sv: 1, toward: tw });
            A.legend.patch(mid0, mid1, c2, c3, { uv: [r[0], vm, r[2], r[3]], ch: chIndex(it.ch + '_A'), smp, su: 1, sv: 1, toward: tw });
          } else A.legend.patch(c0, c1, c2, c3, { uv: r, ch: chIndex(it.ch), smp, su: 1, sv: 1, toward: tw });
          if (it.guard) {                                    // wire guard over the button
            const g = 0.004 + w / 2;
            for (const dx of [-g, g]) this._bar(A.metal, fr.P(it.x + dx, it.y + h / 2 + 0.004, 0), fr.P(it.x + dx, it.y + h / 2 + 0.004, 0.016), 0.0011, smp);
            this._bar(A.metal, fr.P(it.x - g, it.y + h / 2 + 0.004, 0.016), fr.P(it.x + g, it.y + h / 2 + 0.004, 0.016), 0.0011, smp);
            this._bar(A.metal, fr.P(it.x - g, it.y + h / 2 + 0.004, 0.016), fr.P(it.x - g, it.y - h / 2 - 0.002, 0.016), 0.0011, smp);
            this._bar(A.metal, fr.P(it.x + g, it.y + h / 2 + 0.004, 0.016), fr.P(it.x + g, it.y - h / 2 - 0.002, 0.016), 0.0011, smp);
          }
          break;
        }
        case 'win': {
          const smp = smpOf(it.x + it.w / 2, it.y + it.h / 2);
          const z = -0.0005;
          const c0 = fr.P(it.x, it.y + it.h, z), c1 = fr.P(it.x + it.w, it.y + it.h, z), c2 = fr.P(it.x + it.w, it.y, z), c3 = fr.P(it.x, it.y, z);
          if (it.dyn) {
            const isLed = it.dyn === 'evt' || it.dyn === 'met';
            const key = fr.spec.id === 'F8' && it.dyn.startsWith('tb') ? it.dyn + '2' : it.dyn;
            (isLed ? A.led : A.tb).patch(c0, c1, c2, c3, { uv: this.dyn.rect(key), smp, su: 1, sv: 1, toward: fr.P(0, 0, 1) });
          }
          // recess walls
          const back = 0.004;
          const cs = [c3, c2, c1, c0];
          for (let k = 0; k < 4; k++) { const p = cs[k], q = cs[(k + 1) % 4]; A.black.patch(p.clone().addScaledVector(fr.ez, back), q.clone().addScaledVector(fr.ez, back), q, p, { toward: fr.P(it.x + it.w / 2, it.y + it.h / 2, 0.02), smp, su: 1, sv: 1 }); }
          break;
        }
        case 'meter': {
          const smp = smpOf(it.x + it.w / 2, it.y + it.h / 2);
          const lvl = 0.3 + 0.5 * ((Math.sin(it.x * 913 + it.y * 77) + 1) / 2);
          const yy = it.y + it.h * (1 - lvl);
          A.orange.geom(rrBox(it.w * 0.55, 0.0018, 0.001, 0.0002, 0.0002), fr.M(it.x + it.w * 0.40, yy, 0.0008), { smp });
          // glass cover with a black rim
          for (const [u0, v0, u1, v1] of [[it.x - 0.0015, it.y - 0.0015, it.x + it.w + 0.0015, it.y], [it.x - 0.0015, it.y + it.h, it.x + it.w + 0.0015, it.y + it.h + 0.0015], [it.x - 0.0015, it.y, it.x, it.y + it.h], [it.x + it.w, it.y, it.x + it.w + 0.0015, it.y + it.h]]) {
            A.black.geom(rrBox(u1 - u0, v1 - v0, 0.004, 0.0003, 0.0003), fr.M((u0 + u1) / 2, (v0 + v1) / 2, 0.002), { smp });
          }
          break;
        }
        case 'kb': {                                          // DPS keyboard: 8 x 4 keys in a black frame
          const kx = 0.034, ky = 0.0215;
          const w = 4 * kx + 0.012, h = 8 * ky + 0.012;
          A.black.geom(rrBox(w, h, 0.006, 0.004, 0.0015), fr.M(it.x + w / 2, it.y + h / 2, 0.003), { smp: smpOf(it.x + w / 2, it.y + h / 2) });
          for (let i = 0; i < 32; i++) {
            const u = it.x + 0.006 + (i % 4 + 0.5) * kx, v = it.y + 0.006 + (Math.floor(i / 4) + 0.5) * ky;
            const smp = smpOf(u, v);
            A.key.geom(rrBox(kx - 0.005, ky - 0.0045, 0.007, 0.0025, 0.0012), fr.M(u, v, 0.0095), { smp });
            const r = this.legend.rect(`kb${i}`), z = 0.0131, ww = (kx - 0.009) / 2, hh = (ky - 0.008) / 2;
            A.legend.patch(fr.P(u - ww, v + hh, z), fr.P(u + ww, v + hh, z), fr.P(u + ww, v - hh, z), fr.P(u - ww, v - hh, z), { uv: r, ch: 0, smp, su: 1, sv: 1, toward: fr.P(u, v, 1) });
          }
          break;
        }
        case 'sbtc': {                                        // speedbrake / throttle controller
          const smp = smpOf(it.x, it.y);
          const len = it.len || 0.20;
          A.black.geom(rrBox(0.040, len, 0.010, 0.006, 0.002), fr.M(it.x, it.y, 0.005), { smp });
          A.black.geom(rrBox(0.006, len - 0.03, 0.004, 0.002, 0.0008), fr.M(it.x, it.y, 0.011), { smp });
          const v = it.y - len * 0.3;                          // handle position (speedbrake ~ half open)
          A.metal.geom(cyl(0.006, 0.007, 0.050, 10), fr.M(it.x, v, 0.010), { smp });
          A.black.geom(rrBox(0.024, 0.060, 0.034, 0.012, 0.006), fr.M(it.x, v, 0.070, 0, 0.2), { smp });           // grip
          A.black.geom(rrBox(0.012, 0.010, 0.012, 0.003, 0.002), fr.M(it.x + 0.012, v - 0.020, 0.080), { smp });    // takeover switch
          break;
        }
        case 'mdu': {
          if (it.id === 'AFD1') { this._mdu(fr, it, null); break; }
          this._mdu(fr, it, MDU_SPECS[it.id]);
          break;
        }
        default: break;
      }
    }
  }

  /** MEDS multifunction display unit: bezel, recessed LCD (live page texture), six edge keys */
  _mdu(fr, it, spec) {
    const A = this.A;
    const bw = MDU_W, bh = MDU_H, cx = it.x + bw / 2, cy = it.y + bh / 2;
    const smp = this._sample(fr.P(cx, cy, 0.03), fr.ez);
    // screen (6.7 in square active area), recessed in the bezel with black margins
    const S = 0.1704, sx = cx, sy = it.y + 0.010 + S / 2, z = 0.0155;
    A.bezel.geom(rrFrame(bw, bh, 0.020, 0.010, 0, cy - sy, S + 0.008, S + 0.008), fr.M(cx, cy, 0.010), { smp });
    A.black.geom(rrBox(bw - 0.01, bh - 0.01, 0.004, 0.006, 0.001), fr.M(cx, cy, 0.002), { smp });
    A.black.geom(rrBox(S + 0.012, S + 0.012, 0.002, 0.003, 0.0006), fr.M(sx, sy, z + 0.0005), { smp });
    for (const [dx, dy, w, h] of [[0, -(S / 2 + 0.006), S + 0.014, 0.004], [0, S / 2 + 0.006, S + 0.014, 0.004], [-(S / 2 + 0.006), 0, 0.004, S + 0.014], [S / 2 + 0.006, 0, 0.004, S + 0.014]]) {
      A.bezel.geom(rrBox(w, h, 0.006, 0.001, 0.0008), fr.M(sx + dx, sy + dy, 0.0205), { smp });
    }
    const c0 = fr.P(sx - S / 2, sy + S / 2, z + 0.0016), c1 = fr.P(sx + S / 2, sy + S / 2, z + 0.0016), c2 = fr.P(sx + S / 2, sy - S / 2, z + 0.0016), c3 = fr.P(sx - S / 2, sy - S / 2, z + 0.0016);
    let unit = null, tex;
    if (spec) {
      unit = this.meds ? this.meds.panelUnit({ ...spec }) : null;
      this.units[it.id] = unit;
      tex = new THREE.CanvasTexture(unit ? unit.cv : document.createElement('canvas'));
      tex.colorSpace = THREE.SRGBColorSpace; tex.anisotropy = 8; tex.generateMipmaps = true; tex.minFilter = THREE.LinearMipmapLinearFilter;
    } else {
      const c = document.createElement('canvas'); c.width = c.height = 4; const g = c.getContext('2d'); g.fillStyle = '#020303'; g.fillRect(0, 0, 4, 4);
      tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace;
    }
    const mat = this._mat('screen', { color: 0x070808, rough: 0.18, emissive: 0xffffff, emissiveMap: tex });
    const acc = new Acc();
    acc.patch(c0, c1, c2, c3, { toward: fr.P(sx, sy, 1), su: 1, sv: 1, smp });
    const mesh = new THREE.Mesh(acc.build(), mat);
    mesh.receiveShadow = true; mesh.name = 'mdu-' + it.id;
    this.screenMeshes.push(mesh);
    if (spec) this.screens[it.id] = { o: c3.clone(), u: c2.clone().sub(c3), v: c0.clone().sub(c3), tex, unit, ver: -1, mesh };
    // edge keys
    const ky = it.y + 0.010 + S + 0.0175;
    for (let i = 0; i < 6; i++) {
      const kx = cx - S / 2 + (i + 0.5) * S / 6;
      A.key.geom(rrBox(0.021, 0.011, 0.007, 0.002, 0.0012), fr.M(kx, ky, 0.0225), { smp });
      if (unit) this.keys.push({ unit, i, c: fr.P(kx, ky, 0.026), ex: fr.ex.clone(), ey: fr.ey.clone(), ez: fr.ez.clone(), hu: 0.0115, hv: 0.0065 });
    }
    // brightness rocker and power switch on the bezel
    A.black.geom(cyl(0.0058, 0.0062, 0.009, 14), fr.M(cx + bw / 2 - 0.016, ky, 0.020), { smp });      // brightness knob
    A.metal.geom(cyl(0.0030, 0.0030, 0.004, 8), fr.M(cx - bw / 2 + 0.016, ky, 0.020), { smp });         // power switch
    A.metal.geom(cyl(0.0011, 0.0016, 0.011, 6), fr.M(cx - bw / 2 + 0.016, ky, 0.024, 0, -0.35), { smp });
  }

  // ------------------------------------------------------------------ HUD units
  _buildHud() {
    const A = this.A;
    this.huds = [];
    for (const zc of [0, 2 * CL]) {
      // combiner glass above the glareshield lip, in the pilot's line of sight, tilted back
      const w = 0.20, yb = -0.104, yt = 0.068, xb = 0.665, xt = 0.612;
      const corners = [V(xb, yb, zc - w / 2), V(xb, yb, zc + w / 2), V(xt, yt, zc + w / 2), V(xt, yt, zc - w / 2)];
      const smp = this._sample(V(0.64, 0.0, zc), V(-1, 0, 0));
      const fz = (p, dz) => p.clone().add(V(0, 0, dz));
      // frame: side posts and top bar (black anodised), lower rail, projector housing on the glareshield
      for (const sg of [-1, 1]) this._bar(A.black, fz(V(xb + 0.006, yb - 0.016, zc), sg * (w / 2 + 0.006)), fz(V(xt + 0.002, yt + 0.004, zc), sg * (w / 2 + 0.006)), 0.0038, smp);
      this._bar(A.black, V(xt + 0.002, yt + 0.004, zc - w / 2 - 0.006), V(xt + 0.002, yt + 0.004, zc + w / 2 + 0.006), 0.0034, smp);
      A.black.geom(rrBox(0.12, 0.026, w + 0.04, 0.008, 0.004), new THREE.Matrix4().setPosition(V(0.775, -0.148, zc)), { smp });    // projector housing (inside the glareshield)
      this.huds.push({ corners });
    }
    this.hudCombiner = this.huds[0];
  }

  // ------------------------------------------------------------------ seats, hand controllers
  _buildSeatsAndControls() {
    const A = this.A;
    const seat = (zc, full) => {
      const smp = this._sample(V(0.0, -0.6, zc), V(0, 1, 0));
      A.seat.geom(rrBox(0.46, 0.09, 0.50, 0.035, 0.02), new THREE.Matrix4().makeRotationZ(0.06).setPosition(V(0.04, -0.845, zc)), { smp });     // cushion
      A.struct.geom(rrBox(0.50, 0.04, 0.54, 0.01, 0.005), new THREE.Matrix4().setPosition(V(0.04, -0.91, zc)), { smp });
      for (const dz of [-0.25, 0.25]) A.struct.geom(rrBox(0.48, 0.07, 0.02, 0.005, 0.003), new THREE.Matrix4().setPosition(V(0.04, -0.90, zc + dz)), { smp });
      A.struct.geom(rrBox(0.12, -FLOOR - 0.93, 0.42, 0.01, 0.004), new THREE.Matrix4().setPosition(V(0.0, (FLOOR - 0.93) / 2, zc)), { smp });     // pedestal
      for (const dz of [-0.17, 0.17]) A.metal.geom(rrBox(0.60, 0.03, 0.03, 0.006, 0.003), new THREE.Matrix4().setPosition(V(0.02, FLOOR + 0.015, zc + dz)), { smp });   // seat rails
      if (!full) return;
      const tilt = new THREE.Matrix4().makeRotationZ(-0.16);
      const at = (x, y, z) => tilt.clone().setPosition(V(x, y, z));
      A.seat.geom(rrBox(0.13, 0.86, 0.44, 0.05, 0.03), at(-0.21, -0.36, zc), { smp });                       // back cushion
      for (const sg of [-1, 1]) A.seat.geom(rrBox(0.16, 0.62, 0.07, 0.03, 0.02), at(-0.19, -0.42, zc + sg * 0.235), { smp });   // bolsters
      A.struct.geom(rrBox(0.05, 0.96, 0.56, 0.015, 0.008), at(-0.30, -0.36, zc), { smp });                   // seat-back shell
      for (const sg of [-1, 1]) this._bar(A.metal, V(-0.33, -0.86, zc + sg * 0.27), V(-0.20, 0.10, zc + sg * 0.27), 0.012, smp);   // frame tubes
      A.seat.geom(rrBox(0.14, 0.22, 0.30, 0.05, 0.03), at(-0.13, 0.17, zc), { smp });                         // headrest
      for (const sg of [-1, 1]) A.seat.geom(rrBox(0.12, 0.18, 0.05, 0.02, 0.015), at(-0.10, 0.17, zc + sg * 0.16).multiply(new THREE.Matrix4().makeRotationY(sg * 0.35)), { smp });
      for (const sg of [-1, 1]) this._bar(A.belt, V(-0.15, 0.05, zc + sg * 0.10), V(0.07, -0.68, zc + sg * 0.06), 0.012, smp);               // shoulder straps
      this._bar(A.belt, V(0.03, -0.77, zc - 0.23), V(0.12, -0.76, zc), 0.013, smp);
      this._bar(A.belt, V(0.12, -0.76, zc), V(0.03, -0.77, zc + 0.23), 0.013, smp);
      A.metal.geom(rrBox(0.05, 0.05, 0.02, 0.012, 0.004), new THREE.Matrix4().makeRotationZ(-0.3).setPosition(V(0.12, -0.75, zc)), { smp });    // buckle
      for (const sg of [-1, 1]) A.seat.geom(rrBox(0.28, 0.05, 0.07, 0.02, 0.01), new THREE.Matrix4().setPosition(V(-0.05, -0.57, zc + sg * 0.29)), { smp });   // armrests
    };
    seat(0, false);
    seat(2 * CL, true);
    // rotational hand controllers: centre stick between the knees, boot and pistol grip
    for (const zc of [0, 2 * CL]) {
      const smp = this._sample(V(0.47, -0.72, zc), V(-1, 0.3, 0));
      const base = V(0.64, -1.10, zc), top = V(0.48, -0.74, zc);
      this._bar(A.black, base, top, 0.022, smp);
      A.black.geom(cyl(0.05, 0.068, 0.07, 16), new THREE.Matrix4().makeRotationY(Math.PI / 2).multiply(new THREE.Matrix4().makeRotationX(-0.42)).setPosition(base), { smp });
      A.black.geom(rrBox(0.040, 0.11, 0.034, 0.016, 0.008), new THREE.Matrix4().makeRotationZ(-0.35).setPosition(V(0.465, -0.675, zc)), { smp });
      A.black.geom(rrBox(0.030, 0.020, 0.040, 0.008, 0.004), new THREE.Matrix4().setPosition(V(0.452, -0.617, zc)), { smp });
      A.metal.geom(rrBox(0.010, 0.006, 0.010, 0.002, 0.001), new THREE.Matrix4().setPosition(V(0.447, -0.603, zc)), { smp });            // trim switch
      A.orange.geom(rrBox(0.008, 0.004, 0.008, 0.002, 0.001), new THREE.Matrix4().setPosition(V(0.464, -0.603, zc + 0.012)), { smp });  // push-to-talk
    }
  }

  // ------------------------------------------------------------------ bake (sky visibility, AO, display glow)
  _sample(p, n) {
    (this._samples ||= []).push({ p: p.clone(), n: n.clone().normalize() });
    return this._samples.length - 1;
  }
  _bake() {
    const t0 = performance.now();
    const grid = new OccGrid(V(-1.12, FLOOR - 0.05, -1.02), V(1.08, 0.66, 2 * CL + 1.02), 0.025);
    const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
    for (const m of this.meshes) {
      const g = m.geometry, pos = g.attributes.position, idx = g.index;
      for (let i = 0; i < idx.count; i += 3) {
        a.fromBufferAttribute(pos, idx.getX(i)); b.fromBufferAttribute(pos, idx.getX(i + 1)); c.fromBufferAttribute(pos, idx.getX(i + 2));
        grid.tri(a, b, c);
      }
    }
    // window sample points (inner face of each aperture, slightly inside)
    const wins = this.apertures.map((w) => {
      const c0 = w.poly.reduce((s, p) => s.add(p), V(0, 0, 0)).multiplyScalar(1 / w.poly.length);
      const pts = [c0, ...w.poly.map((p) => p.clone().lerp(c0, 0.45))];
      return { poly: w.poly, pts };
    });
    const scr = Object.values(this.screens).map((s) => [s.o, s.o.clone().add(s.u), s.o.clone().add(s.u).add(s.v), s.o.clone().add(s.v)]);
    // AO directions (cosine-weighted, fixed)
    const aoDirs = [];
    for (let i = 0; i < 14; i++) {
      const u1 = (i + 0.5) / 14, u2 = (i * 0.618034) % 1;
      const r = Math.sqrt(u1), phi = 2 * Math.PI * u2;
      aoDirs.push([r * Math.cos(phi), r * Math.sin(phi), Math.sqrt(1 - u1)]);
    }
    const cache = new Map();
    const tq = new THREE.Vector3(), tb = new THREE.Vector3(), tn = new THREE.Vector3();
    const evalAt = (p, n) => {
      const key = `${Math.round(p.x * 80)},${Math.round(p.y * 80)},${Math.round(p.z * 80)},${Math.round(n.x * 4)},${Math.round(n.y * 4)},${Math.round(n.z * 4)}`;
      const hit = cache.get(key); if (hit) return hit;
      const o = p.clone().addScaledVector(n, 0.004);
      let sky = 0;
      for (const w of wins) {
        const F = formFactor(o, n, w.poly);
        if (F < 2e-4) continue;
        let vis = 0;
        for (const q of w.pts) if (n.dot(tq.copy(q).sub(o)) > 0 && grid.clear(o, q, 0.045, 0.06)) vis++;
        sky += F * vis / w.pts.length;
      }
      // AO
      tb.set(1, 0, 0); if (Math.abs(n.x) > 0.8) tb.set(0, 1, 0);
      tb.crossVectors(n, tb).normalize(); tn.crossVectors(n, tb);
      let open = 0;
      for (const d of aoDirs) {
        tq.copy(tb).multiplyScalar(d[0]).addScaledVector(tn, d[1]).addScaledVector(n, d[2]).multiplyScalar(0.32).add(o);
        if (grid.clear(o, tq, 0.04)) open++;
      }
      const ao = 0.18 + 0.82 * open / aoDirs.length;
      let mdu = 0;
      for (const s of scr) mdu += formFactor(o, n, s);
      const r = [Math.min(1, sky), ao, mdu];
      cache.set(key, r);
      return r;
    };
    const sres = (this._samples || []).map((s) => evalAt(s.p, s.n));
    const p = new THREE.Vector3(), n = new THREE.Vector3();
    for (const m of this.meshes) {
      const g = m.geometry, pos = g.attributes.position, nor = g.attributes.normal, smp = g.userData.smp;
      const L = new Float32Array(pos.count * 3);
      for (let i = 0; i < pos.count; i++) {
        const si = smp ? smp[i] : -1;
        const r = si >= 0 ? sres[si] : evalAt(p.fromBufferAttribute(pos, i), n.fromBufferAttribute(nor, i));
        L[i * 3] = r[0]; L[i * 3 + 1] = r[1]; L[i * 3 + 2] = r[2];
      }
      g.setAttribute('aL', new THREE.BufferAttribute(L, 3));
    }
    console.info(`[cockpit] light bake ${(performance.now() - t0).toFixed(0)} ms, ${cache.size} samples`);
  }

  // ------------------------------------------------------------------ glass (windows + HUD combiners)
  _buildGlass() {
    this.glassScene = new THREE.Scene();
    this.glassGroup = new THREE.Group(); this.glassGroup.matrixAutoUpdate = false;
    this.glassScene.add(this.glassGroup);
    const refl = REFLECT_MDUS.map((id) => this.screens[id]).filter(Boolean);
    const gu = this.glassU = {
      uDepth: { value: null }, uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 },
      uCam: { value: new THREE.Vector3() }, uSunDir: { value: new THREE.Vector3(0, 1, 0) }, uSunCol: { value: new THREE.Color() },
      uCabinCol: { value: new THREE.Color() }, uScrK: { value: 0.5 }, uRain: { value: 0 }, uTime: { value: 0 }, uSpeed: { value: 0 },
      uHudCol: { value: new THREE.Color(0, 0, 0) }, uSkyCol: { value: new THREE.Color() },
    };
    const defs = [];
    for (let i = 0; i < 6; i++) {
      const s = refl[i];
      gu[`uScr${i}`] = { value: s ? s.tex : null };
      gu[`uScrO${i}`] = { value: new THREE.Vector3() }; gu[`uScrU${i}`] = { value: new THREE.Vector3() }; gu[`uScrV${i}`] = { value: new THREE.Vector3() };
      defs.push(s ? 1 : 0);
    }
    this._reflScreens = refl;
    const scrDecl = [0, 1, 2, 3, 4, 5].map((i) => `uniform sampler2D uScr${i}; uniform vec3 uScrO${i}; uniform vec3 uScrU${i}; uniform vec3 uScrV${i};`).join('\n');
    const scrCall = [0, 1, 2, 3, 4, 5].filter((i) => defs[i]).map((i) => `refl += scr(uScr${i}, uScrO${i}, uScrU${i}, uScrV${i}, P, R);`).join('\n    ');
    const mk = (kind) => new THREE.ShaderMaterial({
      uniforms: gu, transparent: true, depthTest: false, depthWrite: false, side: THREE.DoubleSide,
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.SrcAlphaFactor,
      blendSrcAlpha: THREE.ZeroFactor, blendDstAlpha: THREE.OneFactor,
      defines: { KIND: kind },
      vertexShader: /* glsl */ `
        varying vec3 vW; varying vec3 vN; varying vec3 vLoc; varying float vViewZ;
        void main(){
          vec4 w = modelMatrix * vec4(position, 1.0);
          vW = w.xyz; vN = normalize(mat3(modelMatrix) * normal); vLoc = position;
          vec4 mv = viewMatrix * w; vViewZ = -mv.z;
          gl_Position = projectionMatrix * mv;
        }`,
      fragmentShader: /* glsl */ `
        uniform sampler2D uDepth; uniform vec2 uRes; uniform float uLogFar;
        uniform vec3 uCam; uniform vec3 uSunDir; uniform vec3 uSunCol; uniform vec3 uCabinCol; uniform vec3 uSkyCol;
        uniform float uScrK; uniform float uRain; uniform float uTime; uniform float uSpeed; uniform vec3 uHudCol;
        ${scrDecl}
        varying vec3 vW; varying vec3 vN; varying vec3 vLoc; varying float vViewZ;
        float h21(vec2 p){ p = fract(p * vec2(123.34, 456.21)); p += dot(p, p + 45.32); return fract(p.x * p.y); }
        float vnoise(vec2 p){ vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
          return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y); }
        vec3 scr(sampler2D t, vec3 o, vec3 u, vec3 v, vec3 P, vec3 R){
          vec3 n = cross(u, v); float den = dot(R, n);
          if (abs(den) < 1e-6) return vec3(0.0);
          float tt = dot(o - P, n) / den; if (tt <= 0.0) return vec3(0.0);
          vec3 h = P + R * tt - o;
          float a = dot(h, u) / dot(u, u), b = dot(h, v) / dot(v, v);
          if (a < 0.0 || a > 1.0 || b < 0.0 || b > 1.0) return vec3(0.0);
          return texture(t, vec2(a, b)).rgb / (1.0 + tt * 0.6);
        }
        void main(){
          vec2 suv = gl_FragCoord.xy / uRes;
          float d = texture(uDepth, suv).r;
          float wS = d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
          if (wS < vViewZ - 0.004) discard;                  // cabin geometry in front of the glass
          vec3 V = normalize(vW - uCam), N = normalize(vN);
          if (dot(N, V) > 0.0) N = -N;
          float c = clamp(-dot(V, N), 0.0, 1.0);
          vec3 P = vW, R = reflect(V, N);
          vec3 refl = uCabinCol;
          ${scrCall}
          refl *= uScrK;
        #if KIND == 0
          // three panes: ~7 % normal-incidence reflectance, rising with angle
          float F = 0.07 + 0.93 * pow(1.0 - c, 5.0);
          // dust, smears and fine scratches lit by the sun (forward scattering)
          vec2 q = vLoc.zy * 9.0 + vLoc.x * 3.0;
          float dust = vnoise(q * 3.0) * 0.6 + vnoise(q * 11.0) * 0.4;
          float smear = smoothstep(0.55, 0.85, vnoise(q * 0.9 + 3.1));
          float scr2 = smoothstep(0.985, 1.0, vnoise(vec2(q.x * 0.4, q.y * 60.0)));
          float fwd = pow(max(dot(V, uSunDir), 0.0), 6.0);
          vec3 haze = uSunCol * (0.0005 + 0.010 * fwd) * (dust * 0.6 + smear * 1.2 + scr2 * 4.0);
          // (water on the outer pane: WindshieldRain, drawn before this pass)
          vec3 col = refl * F + haze;
          float T = (1.0 - F) * 0.90 * (1.0 - 0.06 * smear);
          gl_FragColor = vec4(col, T);
        #else
          // HUD combiner: partially reflective green-notch coating, slight tint
          float F = 0.05 + 0.95 * pow(1.0 - c, 5.0);
          vec3 col = refl * F + uHudCol * 0.02;
          gl_FragColor = vec4(col, (1.0 - F) * 0.93);
        #endif
        }`,
    });
    this.glassMat = mk(0); this.combMat = mk(1);
    const add = (pts, mat, order) => {
      const acc = new Acc();
      acc.patch(pts[0], pts[1], pts[2], pts[3], { toward: V(0, 0, 0), su: 1, sv: 1 });
      const m = new THREE.Mesh(acc.build(), mat); m.renderOrder = order; m.frustumCulled = false;
      this.glassGroup.add(m);
    };
    for (const w of this.apertures) add(w.inner, this.glassMat, 1);
    for (const h of this.huds) add(h.corners, this.combMat, 2);
    // rain on the outer panes (physical drop simulation + refraction)
    this.rain = new WindshieldRain(this.apertures);
  }

  // ------------------------------------------------------------------ per-frame
  /**
   * d: { st, dt, night, sunDir (world), sunRad (Color, sun irradiance at the vehicle), amb (Color),
   *      exposure, camera, rain, link }
   */
  update(d) {
    if (!this.built) return;
    const u = this.uni, st = d.st, night = d.night || 0;
    const now = performance.now();
    // ---- lighting uniforms
    // sky light entering the cabin: the env map is used directly, scaled by the baked window factor
    u.uSkyK.value = 5.5;
    // bounce light: sun patches + sky light through the windows, spread over the interior
    const qInv = this.group.getWorldQuaternion(new THREE.Quaternion()).invert();
    const sLoc = d.sunDir.clone().applyQuaternion(qInv);
    let sunIn = 0;
    for (const w of this.apertures) {
      const A = w.poly[0].distanceTo(w.poly[1]) * w.poly[0].distanceTo(w.poly[3]);
      sunIn += A * Math.max(0, -sLoc.dot(w.nIn));
    }
    const sunE = d.sunRad.clone().multiplyScalar(sunIn / 9.0 * (d.cloudSun ?? 1));
    const skyE = d.amb.clone().multiplyScalar(3.0);
    u.uCabin.value.copy(sunE.multiplyScalar(0.55)).add(skyE.multiplyScalar(0.40));
    // floodlights and integral panel lighting at dusk / night
    const lit = ss(0.25, 0.75, night);
    u.uFlood.value.setRGB(1.0, 0.93, 0.80).multiplyScalar(0.12 * lit);
    u.uPanelLit.value = 0.030 * lit;
    const ex = Math.max(1e-3, d.exposure || 1);
    // AMLCD MDUs are sunlight-readable: about 2.3x the earlier luminance (≈ +1.2 EV)
    u.uScreenK.value = 1.45 / Math.pow(ex / 0.62, 0.80);
    u.uLegendK.value = 0.9 / Math.pow(ex / 0.62, 0.80);
    u.uMduGlow.value.setRGB(0.55, 0.65, 0.62).multiplyScalar(u.uScreenK.value * 0.18);   // spill onto the panel (kept near its old level)
    // ---- light channels
    const ch = u.uChan.value; ch.fill(0); ch[1] = 1;
    if (st) {
      const auto = (st.mode || 'AUTO') !== 'MANUAL';
      ch[chIndex('pitchAC_A')] = auto ? 1 : 0; ch[chIndex('pitchAC_B')] = auto ? 0 : 1;
      ch[chIndex('rollAC_A')] = auto ? 1 : 0; ch[chIndex('rollAC_B')] = auto ? 0 : 1;
      ch[chIndex('sbAM_A')] = auto ? 1 : 0; ch[chIndex('sbAM_B')] = auto ? 0 : 1;
      ch[chIndex('bfAM_A')] = auto ? 1 : 0; ch[chIndex('bfAM_B')] = auto ? 0 : 1;
      const gp = st.gear ? st.gear.pos : 0, agl = st.air?.agl ?? 1e4;
      ch[chIndex('gearArm')] = gp > 0 || agl < 900 ? 1 : 0;
      ch[chIndex('gearDn')] = gp > 0.02 ? 1 : 0;
      const wow = st.gear?.wow?.some((w) => w);
      const cs = st.chute || 0;
      if (cs > 0.004) this._chuteSeen = true;
      if (agl > 500) this._chuteSeen = false;
      ch[chIndex('dcArm')] = wow || cs > 0 || this._chuteSeen ? 1 : 0;
      ch[chIndex('dcDpy')] = cs > 0.004 ? 1 : 0;
      ch[chIndex('dcJett')] = this._chuteSeen && cs <= 0.0005 ? 1 : 0;
      const alarm = !!(this.meds && this.meds.alarm);
      ch[chIndex('ma')] = alarm ? (Math.floor(now / 400) % 2) : 0;
      if (this.meds && this.meds.cwGpc) ch[chIndex('cwGpc')] = 1;
      // gear talkbacks and timers
      const tb = gp < 0.02 ? 'up' : gp > 0.98 ? 'dn' : 'bp';
      const t = st.t || 0, met = 12 * 86400 + 7 * 3600 + 41 * 60 + t;
      const dd = Math.floor(met / 86400), hh = Math.floor(met / 3600) % 24, mm = Math.floor(met / 60) % 60, s2 = Math.floor(met) % 60;
      const evt = Math.max(0, Math.floor(t));
      const p2 = (n) => String(n).padStart(2, '0');
      if (now - (this._dynT || 0) > 240) {
        this._dynT = now;
        if (this.dyn.draw({ tbN: tb, tbL: tb, tbR: tb, tbN2: tb, tbL2: tb, tbR2: tb, evt: `${p2(Math.floor(evt / 60) % 100)}:${p2(evt % 60)}`, met: `${String(dd).padStart(3, '0')}/${p2(hh)}:${p2(mm)}:${p2(s2)}` })) this.dynTex.needsUpdate = true;
      }
    }
    // ---- display textures (upload only when the unit redrew)
    for (const s of Object.values(this.screens)) {
      if (s.unit && s.unit.version !== s.ver) { s.ver = s.unit.version; s.tex.needsUpdate = true; }
      if (s.unit) s.mesh.material.emissiveIntensity = s.unit.st.power ? (0.16 + 0.98 * s.unit.st.brt) : 0;
    }
    // ---- glass uniforms
    const gu = this.glassU;
    gu.uSunDir.value.copy(d.sunDir);
    gu.uSunCol.value.copy(d.sunRad).multiplyScalar(d.cloudSun ?? 1);
    gu.uCabinCol.value.copy(u.uCabin.value).multiplyScalar(0.25 / Math.PI).add(u.uFlood.value.clone().multiplyScalar(0.25 / Math.PI));
    gu.uSkyCol.value.copy(d.amb).multiplyScalar(0.8);
    gu.uRain.value = clamp(d.rain || 0, 0, 1);
    gu.uTime.value = now / 1000;
    if (d.wx) {
      this.rain.update({
        dt: d.wx.dt, rain: d.rain || 0, qInv, vel: d.wx.vel, wind: d.wx.wind, fall: d.wx.fall,
        sunDir: d.sunDir, sunCol: gu.uSunCol.value, cabin: gu.uCabinCol.value, sky: gu.uSkyCol.value,
      });
    }
    gu.uSpeed.value = st?.air?.tas ?? 0;
    const sk = u.uScreenK.value;
    this._reflScreens.forEach((s, i) => {
      const m = this.group.matrixWorld;
      gu[`uScrO${i}`].value.copy(s.o).applyMatrix4(m);
      gu[`uScrU${i}`].value.copy(s.u).transformDirection(m).multiplyScalar(s.u.length());
      gu[`uScrV${i}`].value.copy(s.v).transformDirection(m).multiplyScalar(s.v.length());
    });
    // the screen reflection sampler returns texture colour: scale it to display luminance
    gu.uCabinCol.value.multiplyScalar(1 / Math.max(sk, 1e-4));
    gu.uScrK.value = sk;
  }

  /** water on the windows: drops, residue, mist (debug / tests) */
  rainStats() { return this.rain ? this.rain.stats() : null; }

  /**
   * Place the cockpit for this frame: local origin at the vehicle reference point (vehPos, world),
   * cockpit at eye (model frame) rotated by the vehicle attitude, camera = main camera relative to the
   * origin.  sun: the world's DirectionalLight (colour / intensity), sunDir: unit vector toward the sun.
   */
  place(camera, vehPos, vehQuat, eye, sunDir, sun, env) {
    this.origin.copy(vehPos);
    this.uni.uWorldOffset.value.copy(vehPos);
    this.group.quaternion.copy(vehQuat);
    this.group.position.copy(eye).applyQuaternion(vehQuat);
    this.group.updateMatrixWorld(true);
    const c = this.cam;
    c.position.copy(camera.position).sub(vehPos);
    c.quaternion.copy(camera.quaternion);
    c.fov = camera.fov; c.aspect = camera.aspect; c.near = camera.near; c.far = camera.far;
    c.projectionMatrix.copy(camera.projectionMatrix); c.projectionMatrixInverse.copy(camera.projectionMatrixInverse);
    c.updateMatrixWorld(true);
    this.sun.color.copy(sun.color); this.sun.intensity = sun.intensity;
    this.sun.position.copy(this.group.position).addScaledVector(sunDir, 20);
    this.sun.target.position.copy(this.group.position);
    this.sun.target.updateMatrixWorld();
    this.scene.environment = env;
  }

  /** draw the cockpit into the currently bound HDR target (after the world, sharing its depth) */
  render(renderer) {
    if (!this.built || !this.group.visible) return;
    const ac = renderer.autoClear;
    renderer.autoClear = false;
    renderer.render(this.scene, this.cam);
    renderer.autoClear = ac;
  }

  /** render the window glass into the HDR buffer (after the composite, before bloom) */
  renderGlass(renderer, camera0, depthTex, target) {
    const camera = this.cam;
    if (!this.built || !this.group.visible) return;
    const gu = this.glassU;
    gu.uDepth.value = depthTex;
    gu.uRes.value.set(target.width, target.height);
    gu.uLogFar.value = Math.log2(camera.far + 1.0);
    gu.uCam.value.copy(camera.position);
    this.glassGroup.matrix.copy(this.group.matrixWorld);
    this.glassGroup.matrixWorldNeedsUpdate = true;
    // outer panes first (the water), then the inner pane (reflections, dust)
    this.rain.render(renderer, camera, depthTex, target, this.group.matrixWorld);
    const ac = renderer.autoClear;
    renderer.autoClear = false;
    renderer.setRenderTarget(target);
    renderer.render(this.glassScene, camera);
    renderer.autoClear = ac;
  }

  /** screen-space polygon of the commander's HUD combiner (for clipping the HUD symbology) */
  hudClip(camera0, W, H) {
    if (!this.built) return null;
    const camera = this.cam;
    const m = this.group.matrixWorld;
    const pts = [];
    for (const c of this.hudCombiner.corners) {
      const p = c.clone().applyMatrix4(m).project(camera);
      if (p.z > 1 || p.z < -1) return null;
      pts.push([(p.x * 0.5 + 0.5) * W, (1 - (p.y * 0.5 + 0.5)) * H]);
    }
    return pts;
  }

  /** click on an MDU edge key: ray in world space */
  click(ray) {
    if (!this.built) return false;
    const inv = this.group.matrixWorld.clone().invert();
    const o = ray.origin.clone().sub(this.origin).applyMatrix4(inv), d = ray.direction.clone().transformDirection(inv);
    let best = null, bt = Infinity;
    for (const k of [...this.keys, ...this.buttons]) {
      const den = d.dot(k.ez); if (Math.abs(den) < 1e-6) continue;
      const t = k.c.clone().sub(o).dot(k.ez) / den; if (t <= 0 || t > bt) continue;
      const h = o.clone().addScaledVector(d, t).sub(k.c);
      if (Math.abs(h.dot(k.ex)) <= k.hu && Math.abs(h.dot(k.ey)) <= k.hv) { best = k; bt = t; }
    }
    if (!best) return false;
    this.meds?.clicks?.play('key');
    if (best.act === 'ma') { this.meds?.ackAlarm(); return true; }
    best.unit.edgeKey(best.i);
    return true;
  }
}
