// Water on the flight-deck windows: rain drops on the OUTER panes, simulated and rendered physically.
//
// Simulation (CPU, cockpit frame, metres, per window in its own 2D plane coordinates):
//   * impacts, per drop size class (Marshall-Palmer sizes for the rain rate, each class falling at its own
//     terminal speed and drifting with the wind): the flux through a pane is the expected positive part of
//     (drop velocity relative to the vehicle . pane normal) under the gust spread of the drop velocities;
//     on the lee side of the vehicle the flow is slowed and the rain falls more steeply.  Parked, every
//     pane that faces the sky catches rain (less in a tail wind, never none); flying, the forward panes
//     catch ~10x more water than the side ones.  Impacts splash by K = We^1/2 Re^1/4 (parked the big
//     drops already throw satellite droplets, in flight all do), the splat sweeps the fine droplets under
//     it and merges with the beads it touches, which jump toward the impact
//   * every tracked drop is a spherical cap (contact angle ~70 deg).  Forces in the pane plane: gravity
//     (minus the vehicle's acceleration: braking pushes the water forward), aerodynamic drag of the air
//     flowing over the glass (relative wind projected on the pane, ~ rho u^2 r^2) and contact-line
//     pinning (~ surface tension x r).  Pinning varies over the glass (dirt, coating wear): drops stop and
//     go, wander sideways toward cleaner glass and so run in wiggly paths.  Moving drops reach a viscous
//     terminal speed within a few hundredths of a second.
//   * so small drops stay put while big ones run: parked, the large beads run down the glass under
//     gravity (sideways in a crosswind); with speed the airflow takes over and the water is blown up and
//     back over the windshield (back along the side windows), in streaks that leave a wet trail
//   * running drops sweep up the small droplets in their path and shed tiny trail droplets behind them;
//     drops that touch coalesce (volume conserved) and big drops break up in strong airflow
//   * a running drop leaves a wet track: a thin rivulet that drains in ~12 s parked (stripped at once by
//     the airflow in flight) and pins the next drops less, so later drops run into the old paths
// Rendering (GPU): a water atlas (all panes, ~1.3 texels / mm) holds the bead / droplet water height
// (R, mm), the rivulet height of the wet tracks (G, mm) and the density of sub-millimetre droplets (B).
// A persistent residue layer keeps trail droplets, tracks and mist (with evaporation / blow-off); each
// frame the tracked drops are drawn on top as height fields.  The pane shader then refracts the view ray
// through the water surface (R + G: a bead is a strong lens, a rivulet bends the view at its edges)
// (glass -> water -> air, Snell + Fresnel, total internal reflection at the steep rims) and looks the
// refracted direction up in a mip-mapped copy of the frame: every drop shows its own small, inverted
// image of the world (sky at the bottom, ground at the top), dark rims, sun glints and runway lights.
import * as THREE from 'three';
import { FullscreenPass, makeRT } from '../render/pass.js';

const MAXD = 30000;                 // tracked drops (all panes)
const MAX_STAMP = 20000;            // residue stamps per frame
const MAX_ERASE = 16000;
const RHO_W = 1000, RHO_A = 1.225, G0 = 9.81;
const KH = 0.45;                    // cap height / contact radius (contact angle ~48 deg)
const CV = Math.PI * KH * (3 + KH * KH) / 6;      // cap volume = CV r^3
// Drops are shown VIS x their physical size (a 1 mm drop one metre from the eye is a single pixel).
// The force laws are rescaled with it so that every threshold (when a drop starts to run, when the
// airflow takes over, break-up) and every speed stays where the physical drop would have it.
const VIS = 2.3;
// contact-line pinning F = KR r (N), rain-repellent coated glass: drops on the ~45 deg forward panes
// start to run at r ~ 2-2.5 mm (physical)
const KR = 0.032 * VIS * VIS;
// air drag on a drop sitting in the boundary layer: F = KA rho u^2 r^2 (u: outer flow along the glass)
const KA = 0.045 * VIS;
// viscous resistance of a moving drop: F = KV r v
const KV = 0.18 * VIS * VIS;
const VMAX = 1.8;                   // m/s (water driven by the airflow over the glass)
const RMIN = 0.00028 * VIS;         // smaller tracked drops are handed to the residue layer
const RMAX = 0.005 * VIS;
const CELL = 0.010 * VIS;           // coalescence grid (>= 2 RMAX)
const ATLAS_W = 2048;
// wet tracks (cockpit frame, per pane): a running drop leaves wetted glass behind; drops landing on it
// or running into it are pinned less, so later drops follow the old paths
const TC = 0.0025;                  // track grid cell (m)
const TRACK_PIN = 0.35;             // pinning reduction on a fresh track
// rain drop size classes that become tracked drops on the glass (air diameter, mm); smaller drops
// go into the mist of sub-millimetre droplets
const DBIN = [0.7, 0.95, 1.3, 1.75, 2.35, 3.1, 4.2];
const N0_MP = 8000;                 // Marshall-Palmer intercept, m^-3 mm^-1
// terminal fall speed of a rain drop of diameter D (mm), m/s (Atlas et al. 1973)
const vTerm = (D) => Math.max(0.3, 9.65 - 10.3 * Math.exp(-0.6 * D));
// contact radius (m) of a spherical cap holding the volume of an air drop of diameter D (mm)
const capR = (D) => Math.cbrt((Math.PI * D * D * D / 6) / CV) * 0.001;

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const ss = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
// standard normal pdf / cdf (logistic fit, |err| < 0.01)
const phi = (x) => 0.3989423 * Math.exp(-0.5 * x * x);
const Phi = (x) => 1 / (1 + Math.exp(-1.702 * x));

function hash2(ix, iy, s) {
  let h = (Math.imul(ix, 374761393) + Math.imul(iy, 668265263) + Math.imul(s, 1442695041)) | 0;
  h = Math.imul(h ^ (h >>> 13), 1274126177); h ^= h >>> 16;
  return (h >>> 0) / 4294967296;
}
function vnoise(x, y, s) {
  const ix = Math.floor(x), iy = Math.floor(y);
  let fx = x - ix, fy = y - iy;
  fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy);
  const a = hash2(ix, iy, s), b = hash2(ix + 1, iy, s), c = hash2(ix, iy + 1, s), d = hash2(ix + 1, iy + 1, s);
  return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
}
// pinning strength of the glass at (x, y) (m): mean 1, patchy at the 3 mm and 15 mm scales
function sticky(w, x, y) {
  return 0.45 + 0.75 * vnoise(x / 0.0032, y / 0.0032, w * 7 + 1) + 0.55 * vnoise(x / 0.015, y / 0.015, w * 7 + 2);
}
function gauss() { return Math.sqrt(-2 * Math.log(Math.random() + 1e-12)) * Math.cos(6.2831853 * Math.random()); }
function poisson(l) {
  if (l <= 0) return 0;
  if (l > 25) return Math.max(0, Math.round(l + Math.sqrt(l) * gauss()));
  let n = 0, p = Math.random();
  const e = Math.exp(-l);
  while (p > e) { n++; p *= Math.random(); }
  return n;
}

// ------------------------------------------------------------------ shaders: water atlas
const BLOB_VS = /* glsl */ `
precision highp float;
in vec3 position;              // quad corners (-1..1)
in vec4 aP;                    // tail/previous (xy) -> head/current (zw), atlas px
in vec4 aR;                    // radius / rivulet half width (px), peak height (mm), tail length (in radii), seed
in float aK;                   // kind: 0 drop / droplet, 1 wet track (rivulet)
uniform vec2 uAtlas;
out vec2 vL; out vec4 vR; out float vLen; out float vK; out vec2 vA;
void main(){
  vec2 d = aP.zw - aP.xy; float L = length(d);
  vec2 dir = L > 1e-3 ? d / L : vec2(0.0, 1.0);
  vec2 perp = vec2(-dir.y, dir.x);
  float r = aR.x * 1.15, m = 1.5;
  float back = L + aR.z * aR.x + r;
  float u = position.x < 0.0 ? -back - m : r + m;
  float v = position.y * (r + m);
  vL = vec2(u, v); vR = aR; vLen = L; vK = aK;
  vec2 px = aP.zw + dir * u + perp * v;
  vA = px;
  gl_Position = vec4(px / uAtlas * 2.0 - 1.0, 0.0, 1.0);
}`;

const BLOB_FS = /* glsl */ `
precision highp float;
in vec2 vL; in vec4 vR; in float vLen; in float vK; in vec2 vA;
uniform int uMode;             // 0: drops into the water map (MAX)  1: erase residue (multiply)  2: residue stamps (MAX)
out vec4 o;
// spherical cap, contact angle 48 deg: height fraction at squared normalised radius q
float cap(float q){ return max(0.0, (sqrt(1.8107 - q) - 0.9004) / 0.4452); }
float h21(vec2 p){ vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }
float vn(vec2 p){ vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y); }
void main(){
  float r = vR.x, tail = vR.z;
  vec2 p = vL;
  float s = vR.w * 6.2831853;
  float ang = atan(p.y, p.x);
  // irregular outline (contact line pinned on defects), stronger on big drops
  float irr = 1.0 + (0.06 * sin(3.0 * ang + s) + 0.04 * sin(5.0 * ang + 1.7 * s) + 0.03 * sin(2.0 * ang + 2.9 * s)) * clamp(r / 4.0, 0.25, 1.0);
  float re = max(r * irr, 0.35);
  // rounded front, elongated back (running drop + motion path of this frame)
  float au = p.x >= 0.0 ? re : re + vLen + tail * r;
  float wv = p.x >= 0.0 ? re : re * mix(1.0, 0.6, clamp(-p.x / au, 0.0, 1.0));
  float q = (p.x * p.x) / (au * au) + (p.y * p.y) / (wv * wv);
  float tf = p.x >= 0.0 ? 1.0 : mix(1.0, 0.3, clamp(-p.x / au, 0.0, 1.0));
  if (uMode == 2 && vK > 0.5) {
    // wet track behind a running drop: a thin rivulet (G, water height in mm) along the path, a little
    // wavy in width; parabolic cross-section, so its edges are small cylindrical lenses
    float sx = clamp(p.x, -vLen, 0.0);
    // (noise in atlas space: continuous from one path segment to the next)
    float wob = 0.86 + 0.3 * vn(vA / max(2.5 * r, 1.5) + 17.0);
    float wr = max(r * wob, 0.6);
    float dd = length(vec2(p.x - sx, p.y)) / wr;
    if (dd >= 1.0) discard;
    o = vec4(0.0, vR.y * (1.0 - dd * dd), 0.0, 1.0);
    return;
  }
  if (q >= 1.0 && uMode != 1) discard;
  if (uMode == 0) {
    o = vec4(vR.y * tf * cap(q), 0.0, 0.0, 1.0);
  } else if (uMode == 1) {
    // a drop running or landing here takes up the fine droplets, the mist and most of an old track
    float m = 1.0 - smoothstep(0.75, 1.15, q);
    o = vec4(1.0 - m, 1.0 - 0.7 * m, 1.0 - 0.9 * m, 1.0);
  } else {
    o = vec4(vR.y * tf * cap(q), 0.0, 0.0, 1.0);
  }
}`;

const RECT_VS = /* glsl */ `
precision highp float;
in vec3 position;
in vec4 aRect;                 // atlas px x0 y0 x1 y1
in vec4 aK;                    // keep factors (R, G, B) or mist add (w)
uniform vec2 uAtlas;
out vec2 vPx; out vec4 vK;
void main(){
  vec2 t = position.xy * 0.5 + 0.5;
  vPx = mix(aRect.xy, aRect.zw, t); vK = aK;
  gl_Position = vec4(vPx / uAtlas * 2.0 - 1.0, 0.0, 1.0);
}`;
const RECT_FS = /* glsl */ `
precision highp float;
in vec2 vPx; in vec4 vK;
uniform int uMode;             // 0: multiply by keep factors  1: add mist
uniform float uSeed;
out vec4 o;
float h21(vec2 p){ vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }
float vn(vec2 p){ vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y); }
void main(){
  if (uMode == 0) { o = vec4(vK.xyz, 1.0); return; }
  vec2 o1 = vec2(uSeed * 173.1, fract(uSeed * 7.13) * 291.7), o2 = vec2(fract(uSeed * 3.71) * 157.3, uSeed * 211.9);
  float n = vn(vPx / 23.0 + o1) * 0.6 + vn(vPx / 7.0 + o2) * 0.4;
  o = vec4(0.0, 0.0, vK.w * (0.25 + 1.5 * n * n), 0.0);
}`;

const COPY_FS = /* glsl */ `
uniform sampler2D uSrc;
void main(){ outColor = texture(uSrc, vUv); }`;

// ------------------------------------------------------------------ shader: the outer pane
const PANE_VS = /* glsl */ `
attribute vec2 aAt; attribute vec3 aT; attribute vec3 aB;
varying vec2 vAt; varying vec3 vW; varying vec3 vT; varying vec3 vB; varying vec3 vN; varying float vViewZ;
void main(){
  vec4 w = modelMatrix * vec4(position, 1.0);
  vW = w.xyz; vAt = aAt;
  mat3 m = mat3(modelMatrix);
  vT = m * aT; vB = m * aB; vN = m * normal;
  vec4 mv = viewMatrix * w; vViewZ = -mv.z;
  gl_Position = projectionMatrix * mv;
}`;

const PANE_FS = /* glsl */ `
uniform mat4 projectionMatrix;
uniform sampler2D uWater; uniform sampler2D uBg; uniform sampler2D uDepth;
uniform vec2 uRes; uniform float uLogFar; uniform vec2 uAtlas; uniform float uTexelMM; uniform float uBgMax;
uniform vec3 uCam; uniform vec3 uSunDir; uniform vec3 uSunCol; uniform vec3 uCabin; uniform vec3 uSky;
uniform float uTime; uniform float uFlowT;
varying vec2 vAt; varying vec3 vW; varying vec3 vT; varying vec3 vB; varying vec3 vN; varying float vViewZ;
float h21(vec2 p){ vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }
float vn(vec2 p){ vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(h21(i), h21(i + vec2(1, 0)), f.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), f.x), f.y); }
float sceneW(vec2 uv){ float d = texture(uDepth, uv).r; return d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0; }
// direction (cam-local world) -> screen uv of the frame copy
vec2 dirUv(vec3 P, vec3 D, out float ok){
  vec4 c = projectionMatrix * viewMatrix * vec4(P + D * 3000.0, 1.0);
  ok = c.w > 0.0 ? 1.0 : 0.0;
  return clamp(c.xy / max(c.w, 1e-4) * 0.5 + 0.5, vec2(0.001), vec2(0.999));
}
// refracted look-up that never shows the cabin: walk back toward the pixel when it lands on the flight deck
vec2 outsideUv(vec2 suv, vec2 ruv){
  for (int i = 0; i < 3; i++) { if (sceneW(ruv) > 2.5) break; ruv = mix(suv, ruv, 0.5); }
  return ruv;
}
void main(){
  vec2 suv = gl_FragCoord.xy / uRes;
  if (sceneW(suv) < vViewZ - 0.004) discard;               // cabin structure in front of the pane
  const float LW = 0.55;                                    // defocus of the water layer (mip level)
  vec4 w0 = textureLod(uWater, vAt, LW);
  // water height (mm): beads and droplets (R) on top of the wet tracks / rivulets (G)
  float h = w0.r + w0.g;
  if (h < 0.002 && w0.b < 0.01) discard;
  vec2 tx = 1.0 / uAtlas;
  // gradient of the water surface over ~1.5 texels
  float e = 1.5;
  vec4 wx1 = textureLod(uWater, vAt + vec2(tx.x * e, 0.0), LW), wx0 = textureLod(uWater, vAt - vec2(tx.x * e, 0.0), LW);
  vec4 wy1 = textureLod(uWater, vAt + vec2(0.0, tx.y * e), LW), wy0 = textureLod(uWater, vAt - vec2(0.0, tx.y * e), LW);
  vec2 g = vec2(wx1.r + wx1.g - wx0.r - wx0.g, wy1.r + wy1.g - wy0.r - wy0.g) / (2.0 * e * uTexelMM);    // dh/ds (mm / mm)
  float mist = 1.0 - exp(-w0.b);

  vec3 T = normalize(vT), B = normalize(vB), N = normalize(vN);          // N: out of the cabin
  vec3 V = normalize(vW - uCam);
  // ---- mist of sub-millimetre droplets: scatters the view (blur), grain, forward scattering of daylight
  float ok;
  vec2 fuv = suv;
  float mlod = 0.4 + mist * 3.0;
  vec3 filmCol = textureLod(uBg, fuv, mlod).rgb;
  float grain = h21(floor(vAt * uAtlas * 1.7));
  filmCol *= 1.0 - mist * 0.18 * step(0.55, grain);
  filmCol += uSky * mist * 0.05 + uSunCol * mist * 0.004 * pow(max(dot(V, uSunDir), 0.0), 8.0);
  float aFilm = clamp(mist * 0.6, 0.0, 0.85);

  // ---- water: Snell through the flat glass/water interface and the curved water/air surface (a bead
  // is a strong lens; a rivulet is flat in the middle and bends the view at its edges)
  float cov = smoothstep(0.0, 0.03, h);
  vec3 dropCol = filmCol;
  if (cov > 0.0) {
    vec3 nd = normalize(N - T * g.x - B * g.y);       // outward normal of the water surface
    vec3 Vw = refract(V, -N, 1.0 / 1.333);
    vec3 Vo = refract(Vw, -nd, 1.333);
    float tir = dot(Vo, Vo) < 0.25 ? 1.0 : 0.0;
    float cosA = tir > 0.5 ? 0.0 : clamp(dot(Vo, nd), 0.0, 1.0);
    float F = tir > 0.5 ? 1.0 : 0.02 + 0.98 * pow(1.0 - cosA, 5.0);
    vec3 trans = vec3(0.0);
    if (tir < 0.5) {
      vec2 ruv = dirUv(vW, Vo, ok);
      ruv = outsideUv(suv, ruv);
      // the drop squeezes a wide field into a few pixels: pick the mip from the footprint of the
      // refracted look-up (eye focus is outside, so never fully sharp)
      vec2 dx = dFdx(ruv) * uRes, dy = dFdy(ruv) * uRes;
      float lod = clamp(0.5 * log2(max(dot(dx, dx), dot(dy, dy))), 0.0, uBgMax) * 0.85 + 0.7;
      trans = textureLod(uBg, ruv, lod).rgb * ok;
      // the sun seen through the drop: a tiny bright image
      trans += uSunCol * 0.06 * pow(max(dot(Vo, uSunDir), 0.0), 1200.0);
    }
    // rays reflected at the water surface go back into the cabin and show the dim flight deck; rays
    // trapped at the steep rim (total internal reflection) bounce inside the drop and leave it elsewhere,
    // carrying a dimmed, smeared mix of the surroundings
    vec3 around = textureLod(uBg, suv, 4.5).rgb;
    vec3 refl = tir > 0.5 ? mix(uCabin, around, 0.45) : uCabin + uSky * 0.25 * clamp(1.0 - abs(dot(nd, N)), 0.0, 1.0);
    dropCol = trans * (1.0 - F) * 0.97 + refl * F;
  }
  vec3 col = mix(filmCol, dropCol, cov);
  float a = max(cov, aFilm);
  if (a < 0.002) discard;
  gl_FragColor = vec4(col * a, 1.0 - a);
}`;

function setUpdate(attr, n, comps) {
  attr.clearUpdateRanges();
  attr.addUpdateRange(0, Math.max(1, n * comps));
  attr.needsUpdate = true;
}

export class WindshieldRain {
  /** apertures: the cockpit's window list ({ name, outer: [4 Vector3, cockpit frame], nIn }) */
  constructor(apertures) {
    this.win = [];
    // ---- pane frames + atlas layout
    for (const ap of apertures) {
      const q = ap.outer;
      const O = q[0].clone();
      const T = q[1].clone().sub(q[0]).add(q[2].clone().sub(q[3])).normalize();
      const Bv = q[3].clone().sub(q[0]).add(q[2].clone().sub(q[1]));
      const B = Bv.addScaledVector(T, -Bv.dot(T)).normalize();
      const N = new THREE.Vector3().crossVectors(T, B).normalize();
      if (N.dot(ap.nIn) > 0) N.negate();
      const pts = q.map((p) => { const d = p.clone().sub(O); return [d.dot(T), d.dot(B)]; });
      const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
      const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
      // edge normals (inward) for the point-in-pane test
      let area = 0;
      for (let k = 0; k < 4; k++) { const a = pts[k], b = pts[(k + 1) % 4]; area += a[0] * b[1] - b[0] * a[1]; }
      const sg = Math.sign(area) || 1;
      const edges = [];
      for (let k = 0; k < 4; k++) {
        const a = pts[k], b = pts[(k + 1) % 4];
        let nx = -(b[1] - a[1]) * sg, ny = (b[0] - a[0]) * sg;
        const l = Math.hypot(nx, ny); nx /= l; ny /= l;
        edges.push([nx, ny, -(nx * a[0] + ny * a[1])]);
      }
      let A = 0;
      for (let k = 0; k < 4; k++) { const a = pts[k], b = pts[(k + 1) % 4]; A += a[0] * b[1] - b[0] * a[1]; }
      this.win.push({
        name: ap.name, q, O, T, B, N, pts, x0, x1, y0, y1, edges, area: Math.abs(A) / 2,
        g: [0, 0], u: [0, 0], uS: 0, flux: 0, gust: Math.random() * 100, keep: [1, 1, 1], mistAdd: 0,
        rate: new Float32Array(DBIN.length - 1), vn: new Float32Array(DBIN.length - 1), tauT: 40,
      });
    }
    // shelf packing at the highest density (px / m) that fits the atlas width
    let dens = 1400, H = 0;
    const pad = 6;
    for (let it = 0; it < 30; it++) {
      const order = [...this.win].sort((a, b) => (b.y1 - b.y0) - (a.y1 - a.y0));
      let x = pad, y = pad, rowH = 0;
      for (const w of order) {
        const ww = Math.ceil((w.x1 - w.x0) * dens), hh = Math.ceil((w.y1 - w.y0) * dens);
        if (x + ww + pad > ATLAS_W) { x = pad; y += rowH + pad; rowH = 0; }
        w.ax = x; w.ay = y; x += ww + pad; rowH = Math.max(rowH, hh);
      }
      H = y + rowH + pad;
      if (H <= 2048) break;
      dens *= 0.93;
    }
    this.dens = dens;
    this.atlas = new THREE.Vector2(ATLAS_W, Math.ceil(H / 16) * 16);
    for (const w of this.win) {
      // pinning-free stamps outside the pane would never be wiped: clip everything to the pane rect
      w.rect = [w.ax, w.ay, w.ax + (w.x1 - w.x0) * dens, w.ay + (w.y1 - w.y0) * dens];
      const cols = Math.ceil((w.x1 - w.x0) / CELL) + 1, rows = Math.ceil((w.y1 - w.y0) / CELL) + 1;
      w.gc = cols; w.gr = rows; w.head = new Int32Array(cols * rows);
      // wet tracks: time each cell was last wetted by a running drop
      w.tw = Math.ceil((w.x1 - w.x0) / TC) + 2; w.th = Math.ceil((w.y1 - w.y0) / TC) + 2;
      w.trT = new Float32Array(w.tw * w.th).fill(-1e9);
    }
    this.clock = 0;
    const Atot = this.win.reduce((a, w) => a + w.area, 0);
    for (const w of this.win) { w.cap = Math.floor(MAXD * w.area / Atot); w.cnt = 0; }
    console.info(`[windshield] ${this.win.length} panes, water atlas ${this.atlas.x}x${this.atlas.y} @ ${(dens / 1000).toFixed(2)} px/mm`);

    // ---- drops (structure of arrays)
    this.n = 0;
    this.dw = new Uint8Array(MAXD);
    this.dx = new Float32Array(MAXD); this.dy = new Float32Array(MAXD);
    this.px = new Float32Array(MAXD); this.py = new Float32Array(MAXD);           // position at the start of the frame
    this.vx = new Float32Array(MAXD); this.vy = new Float32Array(MAXD);
    this.dr = new Float32Array(MAXD); this.seed = new Float32Array(MAXD);
    this.trail = new Float32Array(MAXD); this.dead = new Uint8Array(MAXD);
    this.stk = new Float32Array(MAXD); this.act = new Uint8Array(MAXD);          // pinning strength of the glass under the drop (cached while it sits)
    this.next = new Int32Array(MAXD);
    this._grid();

    // ---- GPU
    this.resRT = makeRT(this.atlas.x, this.atlas.y);
    // mip-mapped: the pane shader reads the water slightly defocused (the eye is focused outside)
    this.waterRT = makeRT(this.atlas.x, this.atlas.y, { generateMipmaps: true, minFilter: THREE.LinearMipmapLinearFilter });
    this._needClear = true;
    const quad = new THREE.BufferGeometry();
    quad.setAttribute('position', new THREE.Float32BufferAttribute([-1, -1, 0, 1, -1, 0, -1, 1, 0, 1, 1, 0], 3));
    quad.setIndex([0, 1, 2, 1, 3, 2]);
    const blobGeom = (max) => {
      const g = new THREE.InstancedBufferGeometry();
      g.index = quad.index; g.setAttribute('position', quad.getAttribute('position'));
      const P = new THREE.InstancedBufferAttribute(new Float32Array(max * 4), 4).setUsage(THREE.DynamicDrawUsage);
      const R = new THREE.InstancedBufferAttribute(new Float32Array(max * 4), 4).setUsage(THREE.DynamicDrawUsage);
      const K = new THREE.InstancedBufferAttribute(new Float32Array(max), 1).setUsage(THREE.DynamicDrawUsage);
      g.setAttribute('aP', P); g.setAttribute('aR', R); g.setAttribute('aK', K);
      g.instanceCount = 0;
      return g;
    };
    const blend = (mode) => mode === 'max'
      ? { blending: THREE.CustomBlending, blendEquation: THREE.MaxEquation, blendEquationAlpha: THREE.MaxEquation }
      : mode === 'mul'
        ? { blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.ZeroFactor, blendDst: THREE.SrcColorFactor, blendSrcAlpha: THREE.ZeroFactor, blendDstAlpha: THREE.OneFactor }
        : { blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneFactor, blendSrcAlpha: THREE.ZeroFactor, blendDstAlpha: THREE.OneFactor };
    const raw = (vs, fs, uniforms, b) => new THREE.RawShaderMaterial({
      vertexShader: vs, fragmentShader: fs, uniforms, glslVersion: THREE.GLSL3,
      depthTest: false, depthWrite: false, transparent: true, side: THREE.DoubleSide, ...blend(b),
    });
    const at = { value: this.atlas };
    this.dropG = blobGeom(MAXD); this.eraseG = blobGeom(MAX_ERASE); this.stampG = blobGeom(MAX_STAMP);
    const mesh = (g, m) => { const o = new THREE.Mesh(g, m); o.frustumCulled = false; return o; };
    this.dropMesh = mesh(this.dropG, raw(BLOB_VS, BLOB_FS, { uAtlas: at, uMode: { value: 0 } }, 'max'));
    this.eraseMesh = mesh(this.eraseG, raw(BLOB_VS, BLOB_FS, { uAtlas: at, uMode: { value: 1 } }, 'mul'));
    this.stampMesh = mesh(this.stampG, raw(BLOB_VS, BLOB_FS, { uAtlas: at, uMode: { value: 2 } }, 'max'));
    // per-pane rectangles: decay (multiply) and mist (add)
    const rg = new THREE.InstancedBufferGeometry();
    rg.index = quad.index; rg.setAttribute('position', quad.getAttribute('position'));
    this.rectA = new THREE.InstancedBufferAttribute(new Float32Array(this.win.length * 4), 4);
    this.rectK = new THREE.InstancedBufferAttribute(new Float32Array(this.win.length * 4), 4);
    this.win.forEach((w, i) => this.rectA.array.set(w.rect, i * 4));
    rg.setAttribute('aRect', this.rectA); rg.setAttribute('aK', this.rectK);
    rg.instanceCount = this.win.length;
    this.rectSeed = { value: 0 };
    this.decayMesh = mesh(rg, raw(RECT_VS, RECT_FS, { uAtlas: at, uMode: { value: 0 }, uSeed: this.rectSeed }, 'mul'));
    this.mistMesh = mesh(rg, raw(RECT_VS, RECT_FS, { uAtlas: at, uMode: { value: 1 }, uSeed: this.rectSeed }, 'add'));
    this.scene = new THREE.Scene();
    this.cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    this.copy = new FullscreenPass(COPY_FS, { uSrc: { value: null } });
    this.bgCopy = new FullscreenPass(COPY_FS, { uSrc: { value: null } });
    this.bgRT = null;
    this.stampN = 0; this.eraseN = 0;

    // ---- the outer panes
    this.pu = {
      uWater: { value: this.waterRT.texture }, uBg: { value: null }, uDepth: { value: null },
      uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 }, uAtlas: at, uTexelMM: { value: 1000 / dens }, uBgMax: { value: 8 },
      uCam: { value: new THREE.Vector3() }, uSunDir: { value: new THREE.Vector3(0, 1, 0) }, uSunCol: { value: new THREE.Color() },
      uCabin: { value: new THREE.Color() }, uSky: { value: new THREE.Color() }, uTime: { value: 0 }, uFlowT: { value: 0 },
    };
    this.paneMat = new THREE.ShaderMaterial({
      uniforms: this.pu, vertexShader: PANE_VS, fragmentShader: PANE_FS,
      transparent: true, depthTest: false, depthWrite: false, side: THREE.DoubleSide,
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.SrcAlphaFactor,
      blendSrcAlpha: THREE.ZeroFactor, blendDstAlpha: THREE.OneFactor,
    });
    this.panes = new THREE.Group();
    for (const w of this.win) this.panes.add(this._paneMesh(w));
    this.paneScene = new THREE.Scene();
    this.paneScene.add(this.panes);
    this.active = false;
    this.wetT = 0;                 // seconds since the panes last held water (skip the passes when dry)
    this.flowT = 0;
    this.prevVel = null;
    this.acc = new THREE.Vector3();
    this._warm = false;
  }

  _paneMesh(w) {
    const S = 10, P = [], N = [], AT = [], TT = [], BB = [], I = [];
    const [a, b, c, d] = w.q;
    for (let j = 0; j <= S; j++) for (let i = 0; i <= S; i++) {
      const s = i / S, t = j / S;
      const p = a.clone().lerp(b, s).lerp(d.clone().lerp(c, s), t);
      const r = p.clone().sub(w.O);
      const x = r.dot(w.T), y = r.dot(w.B);
      P.push(p.x, p.y, p.z); N.push(w.N.x, w.N.y, w.N.z);
      TT.push(w.T.x, w.T.y, w.T.z); BB.push(w.B.x, w.B.y, w.B.z);
      AT.push((w.ax + (x - w.x0) * this.dens) / this.atlas.x, (w.ay + (y - w.y0) * this.dens) / this.atlas.y);
    }
    for (let j = 0; j < S; j++) for (let i = 0; i < S; i++) {
      const k = j * (S + 1) + i;
      I.push(k, k + 1, k + S + 2, k, k + S + 2, k + S + 1);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(P, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(N, 3));
    g.setAttribute('aAt', new THREE.Float32BufferAttribute(AT, 2));
    g.setAttribute('aT', new THREE.Float32BufferAttribute(TT, 3));
    g.setAttribute('aB', new THREE.Float32BufferAttribute(BB, 3));
    g.setIndex(I);
    const m = new THREE.Mesh(g, this.paneMat);
    m.frustumCulled = false;
    return m;
  }

  // wetness of the track at (x, y) of pane w: 1 on a fresh track, fading as it drains / dries
  _track(w, x, y) {
    const fx = (x - w.x0) / TC, fy = (y - w.y0) / TC;
    const ix = Math.floor(fx), iy = Math.floor(fy);
    if (ix < 0 || iy < 0 || ix >= w.tw - 1 || iy >= w.th - 1) return 0;
    const T = w.trT, k = iy * w.tw + ix, c = this.clock, it = 1 / w.tauT;
    const a = Math.exp((T[k] - c) * it), b = Math.exp((T[k + 1] - c) * it);
    const d = Math.exp((T[k + w.tw] - c) * it), e = Math.exp((T[k + w.tw + 1] - c) * it);
    const tx = fx - ix, ty = fy - iy;
    return (a + (b - a) * tx) * (1 - ty) + (d + (e - d) * tx) * ty;
  }
  // pinning strength of the glass under a contact line at (x, y): patchy glass, less on wet tracks
  _sticky(wi, x, y) {
    const w = this.win[wi];
    return w.tauT < 1 ? sticky(wi, x, y) : sticky(wi, x, y) * (1 - TRACK_PIN * this._track(w, x, y));
  }
  // a running drop wets the glass along its path (half width hw, m)
  _markTrack(w, x0, y0, x1, y1, hw) {
    if (w.tauT < 1) return;                          // in flight the airflow strips the tracks at once
    const L = Math.hypot(x1 - x0, y1 - y0), ns = Math.max(1, Math.ceil(L / (TC * 0.7)));
    const rc = Math.max(1, Math.round(hw / TC)), rc2 = (hw / TC) * (hw / TC) + 0.25;
    const T = w.trT, c = this.clock;
    for (let s = 0; s <= ns; s++) {
      const f = s / ns;
      const cx = Math.round((x0 + (x1 - x0) * f - w.x0) / TC), cy = Math.round((y0 + (y1 - y0) * f - w.y0) / TC);
      for (let oy = -rc; oy <= rc; oy++) {
        const yy = cy + oy; if (yy < 0 || yy >= w.th) continue;
        for (let ox = -rc; ox <= rc; ox++) {
          const xx = cx + ox; if (xx < 0 || xx >= w.tw || ox * ox + oy * oy > rc2) continue;
          T[yy * w.tw + xx] = c;
        }
      }
    }
  }

  _inside(w, x, y, margin = 0) {
    for (const e of w.edges) if (e[0] * x + e[1] * y + e[2] < margin) return false;
    return true;
  }

  // atlas px of a pane point
  _ax(w, x) { return w.ax + (x - w.x0) * this.dens; }
  _ay(w, y) { return w.ay + (y - w.y0) * this.dens; }

  _stamp(wi, x0, y0, x1, y1, rPx, val, tail, kind) {
    if (this.stampN >= MAX_STAMP) return;
    const w = this.win[wi], i = this.stampN++;
    const P = this.stampG.attributes.aP.array, R = this.stampG.attributes.aR.array, K = this.stampG.attributes.aK.array;
    P[i * 4] = this._ax(w, x0); P[i * 4 + 1] = this._ay(w, y0); P[i * 4 + 2] = this._ax(w, x1); P[i * 4 + 3] = this._ay(w, y1);
    R[i * 4] = rPx; R[i * 4 + 1] = val; R[i * 4 + 2] = tail; R[i * 4 + 3] = Math.random();
    K[i] = kind;
  }
  _erase(wi, x0, y0, x1, y1, rPx) {
    if (this.eraseN >= MAX_ERASE) return;
    const w = this.win[wi], i = this.eraseN++;
    const P = this.eraseG.attributes.aP.array, R = this.eraseG.attributes.aR.array;
    P[i * 4] = this._ax(w, x0); P[i * 4 + 1] = this._ay(w, y0); P[i * 4 + 2] = this._ax(w, x1); P[i * 4 + 3] = this._ay(w, y1);
    R[i * 4] = rPx; R[i * 4 + 1] = 0; R[i * 4 + 2] = 0; R[i * 4 + 3] = 0;
  }
  // a residue droplet of contact radius r (m)
  _droplet(wi, x, y, r) {
    if (r * this.dens < 0.3) return;
    this._stamp(wi, x, y, x, y, r * this.dens, KH * r * 1000, 0, 0);
  }

  // a tracked drop on pane wi overlapping a new drop of radius r at (x, y), or -1 (grid of the last step)
  _hit(wi, x, y, r) {
    const w = this.win[wi];
    const cx = Math.floor((x - w.x0) / CELL), cy = Math.floor((y - w.y0) / CELL);
    for (let oy = -1; oy <= 1; oy++) {
      const yy = cy + oy; if (yy < 0 || yy >= w.gr) continue;
      for (let ox = -1; ox <= 1; ox++) {
        const xx = cx + ox; if (xx < 0 || xx >= w.gc) continue;
        for (let j = w.head[yy * w.gc + xx]; j >= 0; j = this.next[j]) {
          if (this.dead[j]) continue;
          const dx = this.dx[j] - x, dy = this.dy[j] - y, rr = (this.dr[j] + r) * 0.8;
          if (dx * dx + dy * dy < rr * rr) return j;
        }
      }
    }
    return -1;
  }

  _grid() {
    const W = this.win, next = this.next;
    for (const w of W) { w.head.fill(-1); w.cnt = 0; }
    for (let i = 0; i < this.n; i++) {
      if (this.dead[i]) continue;
      const w = W[this.dw[i]];
      const cx = clamp(Math.floor((this.dx[i] - w.x0) / CELL), 0, w.gc - 1), cy = clamp(Math.floor((this.dy[i] - w.y0) / CELL), 0, w.gr - 1);
      const c = cy * w.gc + cx;
      next[i] = w.head[c]; w.head[c] = i; w.cnt++;
    }
  }

  _add(wi, x, y, r, vx = 0, vy = 0) {
    if (this.n >= MAXD) return -1;
    const i = this.n++;
    this.dw[i] = wi; this.dx[i] = x; this.dy[i] = y; this.px[i] = x; this.py[i] = y;
    this.vx[i] = vx; this.vy[i] = vy; this.dr[i] = r; this.seed[i] = Math.random();
    this.trail[i] = Math.random() * 0.004; this.dead[i] = 0;
    this.stk[i] = this._sticky(wi, x, y); this.act[i] = 1;
    return i;
  }

  /**
   * p: { dt (sim s), rain (0..1 at the vehicle), qInv (world -> cockpit frame), vel, wind, fall (world,
   *      m/s), sunDir, sunCol, cabin (Color, flight-deck radiance), sky (Color) }
   */
  update(p) {
    const dt = Math.min(p.dt, 0.25);
    this.stampN = 0; this.eraseN = 0;
    const qInv = p.qInv;
    // specific force: gravity minus the vehicle's acceleration (low-passed; brakes, turns, gusts)
    if (dt > 1e-5) {
      if (this.prevVel) {
        const a = p.vel.clone().sub(this.prevVel).divideScalar(dt);
        if (a.length() < 60) this.acc.lerp(a, Math.min(1, dt / 0.3));
      }
      this.prevVel = (this.prevVel || new THREE.Vector3()).copy(p.vel);
    }
    const gC = new THREE.Vector3(0, -G0, 0).sub(this.acc).applyQuaternion(qInv);
    const airC = p.wind.clone().sub(p.vel).applyQuaternion(qInv);
    const rain = clamp(p.rain, 0, 1);
    const n = this.win.length;
    // ---- the rain reaching each pane, per drop size class.  Drops drift with the (low-passed) wind and
    // fall at their own terminal speed: small drops are blown at a slant, big ones fall steeply.  Gusts
    // spread the drop velocities (sigH), so a pane facing away from the mean rain still catches some of
    // it; on the lee side of the vehicle the flow is slowed and recirculates (shelter), which lets the
    // rain fall more steeply there (parked with a tail wind the windshield gets less rain, never none).
    const driftC = new THREE.Vector3(p.fall.x, 0, p.fall.z).sub(p.vel).applyQuaternion(qInv);
    const upC = new THREE.Vector3(0, 1, 0).applyQuaternion(qInv);
    const sigH = 0.5 + 0.3 * Math.hypot(p.wind.x, p.wind.z);
    // Marshall-Palmer drop sizes; rain 1 ~ 60 mm/h (thunderstorm core), 0.3 ~ 11 mm/h, 0.1 ~ 2.4 mm/h
    const Rmm = 60 * Math.pow(rain, 1.4);
    const lamMP = 4.1 * Math.pow(Math.max(Rmm, 0.05), -0.21);
    const nb = DBIN.length - 1;
    const posFlux = (mu, sg) => { const z = mu / sg; return [Math.max(0, mu * Phi(z) + sg * phi(z)), Phi(z)]; };
    const aU = airC.length();
    for (let k = 0; k < n; k++) {
      const w = this.win[k];
      w.g[0] = gC.dot(w.T); w.g[1] = gC.dot(w.B);
      // flow over the glass: the relative wind, turned along the pane (a little faster than the free stream
      // component on the steep forward panes, where the flow accelerates over the nose), with gusts
      w.gust += dt * (0.7 + Math.random() * 0.6);
      const gf = 1 + 0.18 * (vnoise(w.gust, 0.5, k) - 0.5) * 2;
      const ut = airC.clone().addScaledVector(w.N, -airC.dot(w.N));
      const un = Math.abs(airC.dot(w.N)), U = airC.length();
      const amp = 1 + 0.35 * (U > 1 ? un / U : 0);
      w.u[0] = ut.dot(w.T) * amp * gf; w.u[1] = ut.dot(w.B) * amp * gf;
      // lee side: the air leaves the pane; the flow over it is slowed (separated, recirculating)
      const lee = aU > 0.5 ? ss(0.0, 0.6, airC.dot(w.N) / aU) : 0;
      if (lee > 0) { const f = 1 - 0.6 * lee; w.u[0] *= f; w.u[1] *= f; }
      w.uS = Math.hypot(w.u[0], w.u[1]);
      const Ny = w.N.dot(upC);
      const sigN = (sigH * Math.sqrt(Math.max(0, 1 - Ny * Ny)) + 0.35 * sigH * Math.abs(Ny) + 0.1) * (1 + 0.6 * lee);
      const dN = driftC.dot(w.N);
      w.flux = 0;
      let Emist = 0;
      for (let b = 0; b < nb; b++) {
        const D0 = DBIN[b], D1 = DBIN[b + 1], D = 0.5 * (D0 + D1);
        const shel = lee * (0.65 - 0.3 * ss(0.7, 4, D));        // small drops follow the lee flow more
        const [E, P] = posFlux(-(dN * (1 - shel) - vTerm(D) * Ny), sigN);
        if (b === 3) Emist = E;
        // drops per second on the pane; the count is divided by VIS^2 so the drawn (VIS x larger) drops
        // cover the glass as fast as the physical ones would
        const nD = N0_MP * (Math.exp(-lamMP * D0) - Math.exp(-lamMP * D1)) / lamMP;
        w.rate[b] = rain > 0.005 ? nD * E * w.area / (VIS * VIS) : 0;
        w.vn[b] = E / Math.max(P, 1e-3);                         // mean normal impact speed
        w.flux += w.rate[b];
      }
      // residue: trail droplets evaporate (minutes) or are blown off (> ~45 m/s along the glass); the wet
      // tracks drain / dry in tens of seconds parked and are stripped by the airflow in flight; the mist
      // of tiny droplets survives up to higher speed
      const blowR = 9 * ss(30, 70, w.uS), blowB = 5 * ss(45, 110, w.uS);
      w.tauT = 1 / (1 / 12 + 3 * ss(8, 60, w.uS));
      w.keep[0] = Math.exp(-dt * (1 / 300 + blowR));
      w.keep[1] = Math.exp(-dt / w.tauT);
      w.keep[2] = Math.exp(-dt * (1 / 90 + blowB + 0.25 * ss(2, 12, w.uS)));
      // drizzle drops and impact spray feed the mist (scaled with the rain on the pane)
      w.mistAdd = rain > 0.005 ? dt * Emist * rain * 0.006 : 0;
    }
    // first use (or after a reset) in rain: the glass has been out in it already -- run the drops for
    // ~15 s of the current conditions (CPU only) so the cockpit does not open on dry windows
    if (!this._warm && dt > 0 && rain > 0.3) {
      this._warm = true;
      if (this.n < 50) {
        for (let k = 0; k < 150; k++) this._step(0.1, rain, false);
        for (let s = 0; s < this.n; s++) { this.vx[s] *= 0.2; this.vy[s] *= 0.2; }
        this.stampN = 0; this.eraseN = 0;
        // the rivulets of the warm-up are not on the GPU: start without tracks
        for (const w of this.win) w.trT.fill(-1e9);
      }
    }
    if (dt > 0) {
      // sub-steps: running drops cross many cells per frame at high speed
      const steps = Math.min(3, Math.max(1, Math.ceil(dt / 0.021)));
      for (let s = 0; s < this.n; s++) { this.px[s] = this.dx[s]; this.py[s] = this.dy[s]; }
      for (let s = 0; s < steps; s++) this._step(dt / steps, rain, s === steps - 1);
      this.flowT += dt * (0.3 + this.win[2].uS * 0.08);
    }
    // anything on the glass?
    const wetNow = this.n > 0 || rain > 0.005;
    this.wetT = wetNow ? 0 : this.wetT + dt;
    this.active = wetNow || this.wetT < 120;
    // ---- GPU buffers
    if (!this.active) { this._needClear = true; return; }
    const P = this.dropG.attributes.aP.array, R = this.dropG.attributes.aR.array;
    let m = 0;
    for (let i = 0; i < this.n; i++) {
      if (this.dead[i]) continue;
      const w = this.win[this.dw[i]], r = this.dr[i];
      const sp = Math.sqrt(this.vx[i] * this.vx[i] + this.vy[i] * this.vy[i]);
      // tail: the motion path of this frame (eye / frame integration) plus the elongated back of a running drop
      const x1 = this._ax(w, this.dx[i]), y1 = this._ay(w, this.dy[i]);
      let x0 = this._ax(w, this.px[i]), y0 = this._ay(w, this.py[i]);
      if (Math.hypot(x1 - x0, y1 - y0) > 140) { x0 = x1; y0 = y1; }
      if (sp < 0.003) { x0 = x1; y0 = y1; }
      P[m * 4] = x0; P[m * 4 + 1] = y0; P[m * 4 + 2] = x1; P[m * 4 + 3] = y1;
      // running drops are lower and longer (same volume)
      const run = ss(0.01, 0.6, sp);
      R[m * 4] = r * this.dens * (1 - 0.15 * run);
      R[m * 4 + 1] = KH * r * 1000 * (1 - 0.4 * run);
      R[m * 4 + 2] = 0.45 * run + Math.min(1.2, sp * 0.6);
      R[m * 4 + 3] = this.seed[i];
      m++;
    }
    this.dropG.instanceCount = m;
    setUpdate(this.dropG.attributes.aP, m, 4); setUpdate(this.dropG.attributes.aR, m, 4);
    this.eraseG.instanceCount = this.eraseN;
    setUpdate(this.eraseG.attributes.aP, this.eraseN, 4); setUpdate(this.eraseG.attributes.aR, this.eraseN, 4);
    this.stampG.instanceCount = this.stampN;
    setUpdate(this.stampG.attributes.aP, this.stampN, 4); setUpdate(this.stampG.attributes.aR, this.stampN, 4); setUpdate(this.stampG.attributes.aK, this.stampN, 1);
    const K = this.rectK.array;
    this.win.forEach((w, i) => { K[i * 4] = w.keep[0]; K[i * 4 + 1] = w.keep[1]; K[i * 4 + 2] = w.keep[2]; K[i * 4 + 3] = w.mistAdd; });
    this.rectK.needsUpdate = true;
    this.rectSeed.value = Math.random();
    // pane shader lighting
    const pu = this.pu;
    pu.uSunDir.value.copy(p.sunDir); pu.uSunCol.value.copy(p.sunCol);
    pu.uCabin.value.copy(p.cabin); pu.uSky.value.copy(p.sky);
    pu.uTime.value += dt; pu.uFlowT.value = this.flowT;
  }

  _step(h, rain, last) {
    const W = this.win, dens = this.dens;
    const { dw, dx, dy, vx: VX, vy: VY, dr, dead, act, stk } = this;
    this.clock += h;
    // ---- impacts (against the grid of the previous step)
    const nb = DBIN.length - 1;
    for (let k = 0; k < W.length; k++) {
      const w = W[k];
      if (w.flux <= 0) continue;
      const capS = Math.min(1, (6000 * w.area / 0.5) / w.flux);      // rate cap (fast flight in heavy rain)
      for (let b = 0; b < nb; b++) {
        const cnt = poisson(w.rate[b] * capS * h);
        if (!cnt) continue;
        const vn = w.vn[b];
        const hard = ss(10, 120, vn);            // very fast impacts shatter the drop into smaller pieces
        for (let c = 0; c < cnt; c++) {
          let x = 0, y = 0, ok = false;
          for (let t = 0; t < 6 && !ok; t++) {
            x = w.x0 + Math.random() * (w.x1 - w.x0); y = w.y0 + Math.random() * (w.y1 - w.y0);
            ok = this._inside(w, x, y, 0.001);
          }
          if (!ok) continue;
          const D = DBIN[b] + Math.random() * (DBIN[b + 1] - DBIN[b]);
          // splash of the impact on wet glass (K = We^1/2 Re^1/4 of the normal impact): parked, the big
          // drops (~6-9 m/s) already throw a crown of droplets; in flight every impact does
          const Dm = D * 0.001, vi = vn * (0.75 + 0.5 * Math.random());
          const K = Math.sqrt(RHO_W * vi * vi * Dm / 0.072) * Math.pow(vi * Dm / 1.0e-6, 0.25);
          const spl = Math.max(ss(130, 650, K), hard);
          const r = capR(D) * Math.cbrt(1 - 0.35 * spl) * (1 - 0.45 * hard) * VIS;
          // the splat spreads over ~1.5x the final bead: water landing on or next to a drop joins it, and the
          // merged bead jumps toward the impact (that is how beads grow between the runs)
          const j = this._hit(k, x, y, r * (1 + 0.6 * spl));
          if (j >= 0) {
            const Vj = dr[j] * dr[j] * dr[j], Vr = r * r * r, Vt = Vj + Vr;
            const nx = (dx[j] * Vj + x * Vr) / Vt, ny = (dy[j] * Vj + y * Vr) / Vt;
            if (this._inside(w, nx, ny, 0)) { dx[j] = nx; dy[j] = ny; }
            dr[j] = Math.min(RMAX, Math.cbrt(Vt));
            stk[j] = Math.min(stk[j], this._sticky(k, dx[j], dy[j]));      // the contact line re-forms
            act[j] = 1;
          } else if (w.cnt < w.cap) { this._add(k, x, y, r); w.cnt++; }
          else this._droplet(k, x, y, r * 0.6);
          // the splat sweeps up the fine droplets under it (shattering high-speed impacts spray instead)
          if (hard < 0.5) this._erase(k, x, y, x, y, r * this.dens * (1.2 + 0.7 * spl));
          // crown splash: satellite droplets thrown around the impact
          if (spl > 0.02) {
            const ns = Math.floor(spl * (2 + 5 * Math.random()) + Math.random() * 0.7);
            for (let s = 0; s < ns; s++) {
              const a = Math.random() * 6.2832, d = r * (1.5 + Math.random() * (1.5 + 2.5 * spl));
              this._droplet(k, x + Math.cos(a) * d, y + Math.sin(a) * d, r * (0.1 + 0.22 * Math.random()));
            }
          }
        }
      }
    }
    // ---- forces and motion
    const kk = 1 - Math.exp(-h / 0.05);
    for (const w of W) {
      const ul = 0.3 * w.uS;
      w.rb = clamp(0.30 / Math.max(RHO_A * ul * ul, 1e-6), 0.0012, 0.0065) * VIS;   // aerodynamic break-up radius
      w.fa = KA * RHO_A * w.uS;
    }
    const n0 = this.n;
    for (let i = 0; i < n0; i++) {
      if (dead[i]) continue;
      const wi = dw[i], w = W[wi];
      let r = dr[i];
      const m = RHO_W * CV * r * r * r;
      let fx = m * w.g[0] + w.fa * w.u[0] * r * r;
      let fy = m * w.g[1] + w.fa * w.u[1] * r * r;
      const fd = Math.sqrt(fx * fx + fy * fy);
      let vx = VX[i], vy = VY[i];
      const sp = Math.sqrt(vx * vx + vy * vy);
      // pinning: patchy glass; moving drops slip a little easier (receding contact line already wet)
      if (sp === 0 && fd < KR * r * stk[i] && r <= w.rb) continue;     // pinned (the common case)
      const x = dx[i], y = dy[i];
      // a moving drop is held by its advancing contact line (the receding one is on glass it has wetted)
      const st = sp > 0 ? (stk[i] = this._sticky(wi, x + vx / sp * r * 0.9, y + vy / sp * r * 0.9)) : stk[i];
      const fr = KR * r * st * (sp > 0.004 ? 0.72 : 1);
      let tx = 0, ty = 0;
      if (fd > fr) {
        // drops drift toward cleaner glass and into the wet tracks of earlier drops: the pinning gradient
        // ahead of the drop steers it across the driving force
        const e = 0.0012;
        const dx0 = fx / fd, dy0 = fy / fd;
        const qx = x + dx0 * r * 0.9, qy = y + dy0 * r * 0.9;
        const gxs = (this._sticky(wi, qx + e, qy) - this._sticky(wi, qx - e, qy)) / (2 * e);
        const gys = (this._sticky(wi, qx, qy + e) - this._sticky(wi, qx, qy - e)) / (2 * e);
        const lat = clamp(-(gxs * -dy0 + gys * dx0) * 0.0035, -0.8, 0.8);
        let dxn = dx0 - dy0 * lat, dyn = dy0 + dx0 * lat;
        const dl = Math.sqrt(dxn * dxn + dyn * dyn); dxn /= dl; dyn /= dl;
        const vt = Math.min(VMAX, (fd - fr) / (KV * r));
        tx = dxn * vt; ty = dyn * vt;
      }
      // momentum relaxes to the terminal (or zero) velocity in ~50 ms
      vx += (tx - vx) * kk; vy += (ty - vy) * kk;
      if (tx === 0 && ty === 0 && vx * vx + vy * vy < 1e-6) { vx = 0; vy = 0; }
      let nx = x + vx * h, ny = y + vy * h;
      const mdx = nx - x, mdy = ny - y, moved = Math.sqrt(mdx * mdx + mdy * mdy);
      if (moved > 1e-6) {
        act[i] = 1;
        // sweep the residue in the path, leave a wet trail, shed tiny droplets behind
        const spn = Math.sqrt(vx * vx + vy * vy);
        this._erase(wi, x, y, nx, ny, r * dens * 0.95);
        this._markTrack(w, x, y, nx, ny, r * 0.6);
        // the wet track: a rivulet ~0.6 of the drop wide, thinner behind fast drops (water height, mm)
        if (last || moved > r) {
          const hr = clamp(0.03 + 18 * r, 0.05, 0.28) / (1 + 0.8 * spn);
          this._stamp(wi, this.px[i], this.py[i], nx, ny, r * dens * 0.6, hr, 0, 1);
        }
        this.trail[i] += moved;
        const spacing = r * (2.0 + 3.0 * this.seed[i]) * (1 + spn * 2.5);
        let shed = 0;
        while (this.trail[i] > spacing && shed < 3) {
          this.trail[i] -= spacing; shed++;
          const f = clamp(1 - this.trail[i] / moved, 0, 1);
          const bx = x + mdx * f, by = y + mdy * f;
          const rt = r * (0.16 + 0.16 * Math.random()) * (spn > 1 ? 0.6 : 1);
          const dvx = spn > 1e-4 ? vx / spn : 0, dvy = spn > 1e-4 ? vy / spn : 0;
          this._droplet(wi, bx - dvx * r * 0.9 + (Math.random() - 0.5) * r * 0.6, by - dvy * r * 0.9, rt);
          r = Math.cbrt(Math.max(r * r * r - rt * rt * rt, 1e-15));
        }
        if (this.trail[i] > spacing) this.trail[i] = spacing * Math.random();
      }
      // aerodynamic break-up of big drops (Weber number of the flow along the glass)
      if (r > w.rb && this.n < MAXD) {
        const r2 = r * 0.7937;                         // halves the volume
        const sp2 = Math.sqrt(vx * vx + vy * vy) || 1;
        const ox = -vy / sp2 * r2, oy = vx / sp2 * r2;
        r = r2;
        const j = this._add(wi, nx - ox * 1.1, ny - oy * 1.1, r2 * (0.85 + Math.random() * 0.15), vx * 0.8, vy * 0.8);
        if (j >= 0) { this.px[j] = this.px[i]; this.py[j] = this.py[i]; }
        nx += ox * 1.1; ny += oy * 1.1;
        act[i] = 1;
      }
      dx[i] = nx; dy[i] = ny; VX[i] = vx; VY[i] = vy; dr[i] = Math.min(r, RMAX);
      // ran off the pane (under the frame), or too small to track
      if (act[i]) {
        if (!this._inside(w, nx, ny, -dr[i] * 0.5)) dead[i] = 1;
        else if (dr[i] < RMIN) { this._droplet(wi, nx, ny, dr[i]); dead[i] = 1; }
      }
    }
    // ---- compact, then the grid
    let o = 0;
    for (let i = 0; i < this.n; i++) {
      if (dead[i]) continue;
      if (o !== i) {
        dw[o] = dw[i]; dx[o] = dx[i]; dy[o] = dy[i]; this.px[o] = this.px[i]; this.py[o] = this.py[i];
        VX[o] = VX[i]; VY[o] = VY[i]; dr[o] = dr[i]; this.seed[o] = this.seed[i]; this.trail[o] = this.trail[i]; stk[o] = stk[i]; act[o] = act[i];
      }
      dead[o] = 0; o++;
    }
    this.n = o;
    this._grid();
    // ---- coalescence: only drops that moved, landed or grew can newly touch a neighbour
    const next = this.next;
    for (let i = 0; i < this.n; i++) {
      if (!act[i] || dead[i]) continue;
      act[i] = 0;
      const w = W[dw[i]];
      const cx = clamp(Math.floor((dx[i] - w.x0) / CELL), 0, w.gc - 1), cy = clamp(Math.floor((dy[i] - w.y0) / CELL), 0, w.gr - 1);
      let a = i;
      for (let oy = -1; oy <= 1 && !dead[i]; oy++) {
        const yy = cy + oy; if (yy < 0 || yy >= w.gr) continue;
        for (let ox = -1; ox <= 1 && !dead[i]; ox++) {
          const xx = cx + ox; if (xx < 0 || xx >= w.gc) continue;
          for (let j = w.head[yy * w.gc + xx]; j >= 0; j = next[j]) {
            if (j === i || dead[j]) continue;
            const ddx = dx[j] - dx[i], ddy = dy[j] - dy[i];
            const rr = dr[i] + dr[j];
            if (ddx * ddx + ddy * ddy > rr * rr) continue;
            // the smaller joins the larger (volume and momentum conserved)
            a = i; let b = j;
            if (dr[j] > dr[i]) { a = j; b = i; }
            const Va = dr[a] * dr[a] * dr[a], Vb = dr[b] * dr[b] * dr[b], Vt = Va + Vb;
            dx[a] = (dx[a] * Va + dx[b] * Vb) / Vt; dy[a] = (dy[a] * Va + dy[b] * Vb) / Vt;
            VX[a] = (VX[a] * Va + VX[b] * Vb) / Vt; VY[a] = (VY[a] * Va + VY[b] * Vb) / Vt;
            dr[a] = Math.min(RMAX, Math.cbrt(Vt));
            stk[a] = Math.min(stk[a], stk[b]);       // the merged contact line rests on the weaker spot
            dead[b] = 1; act[a] = 1;
            if (b === i) break;
          }
        }
      }
    }
  }

  /** atlas passes + the outer panes, drawn into target (the HDR frame) with camera; group: the cockpit frame */
  render(renderer, camera, depthTex, target, groupMatrix) {
    if (!this.active) return;
    const ac = renderer.autoClear;
    renderer.autoClear = false;
    const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
    // ---- residue layer
    renderer.setRenderTarget(this.resRT);
    if (this._needClear) { renderer.setClearColor(0x000000, 0); renderer.clear(true, false, false); this._needClear = false; }
    const pass = (obj) => { this.scene.add(obj); renderer.render(this.scene, this.cam); this.scene.remove(obj); };
    pass(this.decayMesh);
    pass(this.mistMesh);
    if (this.eraseG.instanceCount) pass(this.eraseMesh);
    if (this.stampG.instanceCount) pass(this.stampMesh);
    // ---- water = residue + drops
    this.copy.uniforms.uSrc.value = this.resRT.texture;
    this.copy.render(renderer, this.waterRT);
    renderer.setRenderTarget(this.waterRT);
    if (this.dropG.instanceCount) pass(this.dropMesh);
    // ---- frame copy (mip-mapped) for the refraction
    if (!this.bgRT || this.bgRT.width !== target.width || this.bgRT.height !== target.height) {
      this.bgRT?.dispose();
      this.bgRT = makeRT(target.width, target.height, { generateMipmaps: true, minFilter: THREE.LinearMipmapLinearFilter });
    }
    this.bgCopy.uniforms.uSrc.value = target.texture;
    this.bgCopy.render(renderer, this.bgRT);
    const pu = this.pu;
    pu.uBg.value = this.bgRT.texture;
    pu.uBgMax.value = Math.floor(Math.log2(Math.max(target.width, target.height)));
    pu.uDepth.value = depthTex;
    pu.uRes.value.set(target.width, target.height);
    pu.uLogFar.value = Math.log2(camera.far + 1.0);
    pu.uCam.value.copy(camera.position);
    this.panes.matrixAutoUpdate = false;
    this.panes.matrix.copy(groupMatrix);
    this.panes.matrixWorldNeedsUpdate = true;
    renderer.setRenderTarget(target);
    renderer.render(this.paneScene, camera);
    renderer.setClearColor(cc, ca);
    renderer.autoClear = ac;
  }

  /** remove all water (e.g. on a new episode) */
  clear() { this.n = 0; this._grid(); this._needClear = true; for (const w of this.win) w.trT.fill(-1e9); this._warm = false; this.prevVel = null; this.acc.set(0, 0, 0); }

  stats() { return { drops: this.n, stamps: this.stampN, erase: this.eraseN, active: this.active }; }
}
