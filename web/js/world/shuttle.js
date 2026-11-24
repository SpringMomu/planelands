// Procedural Space Shuttle orbiter, modelled on the real vehicle (OV-103/104 era) and sized by
// shared/orbiter_geometry.json (wing, fin, body flap, engines, gear, chute are taken from there so
// the visual model, the aerodynamics and the gear contact points agree).
// Local frame (three): X forward, Y up, Z right; origin = CG.  FRD body (x,y,z) -> (x, -z, y).
//
// Outer mould line: boxy mid fuselage with vertical sides and arched payload-bay doors, crew
// module with the six-pane windshield and overhead windows, flat-bottomed chined nose with the
// RCC nose cap and forward RCS ports, OMS/RCS pods rising out of the aft fuselage shoulders,
// three SSMEs with bell nozzles, hatbands and engine heat-shield boots.  Thermal protection
// system zoning (HRSI black / LRSI white tiles, FRSI felt, AFRSI blankets, RCC) and markings are
// procedural (no texture files).
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
// attr / attr2: per-vertex scalars (aChord / aMet) used by the TPS shader.
function loft(rings, { capStart = false, capEnd = false, flip = false, attr = null, attr2 = null, closed = true } = {}) {
  const M = rings[0].length;
  const pos = [], ach = [], am = [], idx = [];
  for (let r = 0; r < rings.length; r++) for (let j = 0; j < M; j++) {
    pos.push(...rings[r][j]);
    ach.push(attr ? attr[r][j] : 0);
    am.push(attr2 ? attr2[r][j] : 0);
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
    for (let j = 0; j < M; j++) { pos.push(...ring[j]); ach.push(attr ? attr[r][j] : 0); am.push(attr2 ? attr2[r][j] : 0); }
    pos.push(cx, cy, cz); ach.push(attr ? attr[r][0] : 0.5); am.push(attr2 ? attr2[r][0] : 0);
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
  g.setAttribute('aMet', new THREE.Float32BufferAttribute(am, 1));
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

// ------------------------------------------------------------------ fuselage outer mould line
// Side / plan profile (model frame, Y up), refined from the shared stations: flat-bottomed chined
// nose, crew-module roof 0.4 m above the payload-bay doors, slight flare of the aft fuselage.
const FUSE = (() => {
  const t = [
    // x       half-width  top     bottom
    [20.89, 0.90, 0.00, -1.12],
    [20.39, 1.25, 0.38, -1.36],
    [19.39, 1.70, 0.86, -1.68],
    [18.39, 2.02, 1.18, -1.95],
    [17.39, 2.25, 1.48, -2.12],
    [16.60, 2.39, 1.80, -2.20],
    [16.00, 2.47, 2.25, -2.24],
    [15.40, 2.53, 2.72, -2.27],
    [15.00, 2.56, 3.02, -2.28],
    [14.40, 2.59, 3.25, -2.29],
    [13.40, 2.60, 3.34, -2.30],
    [12.75, 2.60, 3.30, -2.30],
    [12.45, 2.60, 3.10, -2.30],
    [12.10, 2.60, 2.95, -2.30],
    [11.00, 2.60, 2.92, -2.30],
    [-7.60, 2.60, 2.92, -2.30],
    [-9.50, 2.68, 2.98, -2.30],
    [-11.38, 2.75, 3.05, -2.30],
  ].reverse();
  const xs = t.map((r) => r[0]);
  return { hw: monotone(xs, t.map((r) => r[1])), yt: monotone(xs, t.map((r) => r[2])), yb: monotone(xs, t.map((r) => r[3])) };
})();
const NOSE_X = 21.39, CAP_X = 20.89, BASE_X = -11.38;
// section point counts: bottom flat, chine fillet, side wall, top
const NB = 8, NC = 10, NW = 10, NT = 26;

// OMS/RCS pod (right side; mirrored): front where it starts to rise out of the aft fuselage,
// full section from POD_FULL aft, flat aft face with the OMS engine
const POD = { xf: -4.6, xfull: -7.3, xa: -12.05, zc: 1.78, yApex: 4.42, zOut: 2.74, zIn: 0.78, yBase: 2.55 };

// ------------------------------------------------------------------ markings (canvas atlas)
// atlas layout (u, v in [0,1], v up): "United States" (0,0.5)-(1,1); flag (0,0)-(0.5,0.5); "USA" (0.5,0)-(1,0.5)
let DECAL_TEX = null;
function decalTexture() {
  if (DECAL_TEX) return DECAL_TEX;
  const W = 2048, H = 1024;
  const cv = document.createElement('canvas'); cv.width = W; cv.height = H;
  const g = cv.getContext('2d');
  g.clearRect(0, 0, W, H);
  // "United States" (top half of the canvas = v 0.5..1)
  g.fillStyle = '#141414';
  g.textBaseline = 'middle'; g.textAlign = 'center';
  let fs = 330;
  g.font = `600 ${fs}px "Helvetica Neue", Helvetica, Arial, sans-serif`;
  while (g.measureText('United States').width > W * 0.96) { fs -= 6; g.font = `600 ${fs}px "Helvetica Neue", Helvetica, Arial, sans-serif`; }
  g.fillText('United States', W / 2, H * 0.25 + 10);
  // flag (bottom-left quarter): 13 stripes, canton 7 stripes high, 0.76 of the hoist... width 0.4
  const fx = 24, fy = H / 2 + 40, fw = W / 2 - 48, fh = H / 2 - 80;
  for (let i = 0; i < 13; i++) { g.fillStyle = i % 2 ? '#f4f4f2' : '#b22234'; g.fillRect(fx, fy + i * fh / 13, fw, fh / 13 + 1); }
  const cw = fw * 0.4, ch = fh * 7 / 13;
  g.fillStyle = '#3c3b6e'; g.fillRect(fx, fy, cw, ch);
  g.fillStyle = '#f4f4f2';
  for (let r = 0; r < 9; r++) {
    const n = r % 2 ? 5 : 6;
    for (let c = 0; c < n; c++) {
      const sx = fx + cw * ((c + (r % 2 ? 1 : 0.5)) / 6), sy = fy + ch * ((r + 0.5) / 9);
      star(g, sx, sy, ch * 0.046);
    }
  }
  // "USA" (bottom-right quarter)
  g.fillStyle = '#141414';
  g.font = `700 ${Math.round(H * 0.36)}px "Helvetica Neue", Helvetica, Arial, sans-serif`;
  g.fillText('USA', W * 0.75, H * 0.75 + 12);
  DECAL_TEX = new THREE.CanvasTexture(cv);
  DECAL_TEX.colorSpace = THREE.SRGBColorSpace;
  DECAL_TEX.anisotropy = 8;
  DECAL_TEX.generateMipmaps = true;
  DECAL_TEX.minFilter = THREE.LinearMipmapLinearFilter;
  return DECAL_TEX;
}
function star(g, x, y, r) {
  g.beginPath();
  for (let k = 0; k < 10; k++) {
    const a = -Math.PI / 2 + k * Math.PI / 5, rr = k % 2 ? r * 0.4 : r;
    k ? g.lineTo(x + rr * Math.cos(a), y + rr * Math.sin(a)) : g.moveTo(x + rr * Math.cos(a), y + rr * Math.sin(a));
  }
  g.closePath(); g.fill();
}

// ------------------------------------------------------------------ TPS material
const TILE_GLSL = /* glsl */ `
uniform int uZone;
uniform sampler2D uDecal;
varying vec3 vObjPos;
varying vec3 vObjN;
varying float vChord;
varying float vMet;
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
// tile grid on the three axis planes, blended by the normal (avoids seams where the plane switches)
vec2 tileGridN(vec3 p, vec3 n, vec2 size, float fw){
  vec3 w = pow(abs(n), vec3(6.0)); w /= (w.x + w.y + w.z);
  vec2 gx = tileGrid(p.zy, size, fw), gy = tileGrid(p.xz, size, fw), gz = tileGrid(p.xy, size, fw);
  float groove = gx.x * w.x + gy.x * w.y + gz.x * w.z;
  float id = w.y >= max(w.x, w.z) ? gy.y : (w.z >= w.x ? gz.y : gx.y);
  return vec2(groove, id);
}
vec2 projPlane(vec3 p, vec3 n){
  vec3 a = abs(n);
  if (a.y >= a.x && a.y >= a.z) return p.xz;
  if (a.z >= a.x) return p.xy;
  return p.zy;
}
// soft-edged rectangle / ellipse masks
float boxM(vec2 p, vec2 c, vec2 h, float fw){ vec2 d = abs(p - c) - h; return 1.0 - smoothstep(-fw, fw, max(d.x, d.y)); }
float ellM(vec2 p, vec2 c, vec2 r, float fw){ return 1.0 - smoothstep(1.0 - fw * 3.0 / min(r.x, r.y), 1.0, length((p - c) / r)); }
float rboxM(vec2 p, vec2 c, vec2 h, float r, float fw){ vec2 d = abs(p - c) - h + r; float s = length(max(d, 0.0)) + min(max(d.x, d.y), 0.0) - r; return 1.0 - smoothstep(-fw, fw, s); }
vec4 atlas(vec2 uv, vec4 rect){
  if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return vec4(0.0);
  return texture2D(uDecal, rect.xy + uv * rect.zw);
}

// ---- fuselage (zone 0): vChord = section segment 0 bottom / 1 chine / 2 side wall / 3 top
void zoneFuselage(vec3 p, vec3 n, vec2 pp, float fw, out int kind, out float aux){
  float x = p.x, y = p.y, az = abs(p.z), seg = vChord;
  aux = 0.0;
  // black HRSI below the chine line: level along the payload bay, sweeping up the nose sides
  float chineY = mix(-1.55, -0.32, smoothstep(10.6, 18.9, x));
  chineY = mix(chineY, 0.50, smoothstep(18.9, 20.75, x));
  bool black = seg < 1.55 || y < chineY;
  // aft fuselage lower sides behind the wing trailing edge
  if (x < -9.6 && y < -1.25) black = true;
  kind = black ? 1 : 0;
  // RCC nose cap (with its tile border)
  if (x > 20.80 - 0.08 * smoothstep(0.0, 0.9, -y - 0.2)) { kind = 2; return; }
  // ---- crew module windows
  // windshield: six panes between the base line (x 16.55) and the roof edge (x 15.05)
  float ang = atan(az, y - 0.35);                 // azimuth around the cabin from the top
  float xBase = 16.56 - 0.42 * ang * ang;          // windshield sill, sweeping aft on the sides
  float xTop = 15.10 - 0.16 * ang;                 // roof edge
  float inWs = (1.0 - smoothstep(xBase - 0.03, xBase, x)) * smoothstep(xTop, xTop + 0.03, x);
  float m1 = 1.0 - smoothstep(0.0, 0.012, abs(ang - 0.36) - 0.02);
  float m2 = 1.0 - smoothstep(0.0, 0.012, abs(ang - 0.78) - 0.02);
  float cpost = 1.0 - smoothstep(0.0, 0.01, az - 0.055);
  float wsPane = inWs * (1.0 - smoothstep(1.21, 1.22, ang)) * (1.0 - m1) * (1.0 - m2) * (1.0 - cpost);
  // overhead windows (two, on the roof aft of the windshield)
  float ovh = rboxM(vec2(x, az), vec2(13.82, 0.68), vec2(0.36, 0.32), 0.08, fw) * step(2.9, y);
  // aft flight deck / side hatch window (left side only)
  float hatchR = length(vec2(x - 13.55, y - 0.15));
  float hatchWin = (p.z < 0.0 && n.z < -0.5) ? 1.0 - smoothstep(0.125, 0.135, hatchR) : 0.0;
  if (wsPane > 0.5 || ovh > 0.5 || hatchWin > 0.5){ kind = 3; return; }
  // black mask around the windshield and the overhead windows
  float wsMask = (1.0 - smoothstep(xBase + 0.16, xBase + 0.2, x)) * smoothstep(xTop - 0.2, xTop - 0.16, x) * (1.0 - step(1.36, ang));
  float ovMask = rboxM(vec2(x, az), vec2(13.82, 0.68), vec2(0.52, 0.5), 0.15, fw) * step(2.9, y);
  if (wsMask > 0.5 || ovMask > 0.5){ kind = 1; return; }
  // side hatch outline
  if (p.z < 0.0 && n.z < -0.5 && abs(hatchR - 0.53) < 0.018 + fw){ kind = 1; return; }
  // forward RCS thrusters: three side-firing ports per side, three up-firing on the nose top
  float rcs = 0.0;
  if (az > 0.5 && abs(n.z) > 0.45){ for (int i = 0; i < 3; i++) rcs = max(rcs, ellM(vec2(x, y), vec2(19.40 + 0.42 * float(i), 0.24 - 0.07 * float(i)), vec2(0.17, 0.14), fw)); }
  if (n.y > 0.5){ for (int i = 0; i < 3; i++) rcs = max(rcs, ellM(vec2(x, p.z), vec2(19.0 + 0.03 * float(i), (float(i) - 1.0) * 0.42), vec2(0.16, 0.13), fw)); }
  if (rcs > 0.5){ kind = 4; return; }
  // payload bay doors (top segment, mid fuselage) and the FRSI-covered sides
  if (!black && seg > 3.02 && x < 12.42 && x > -5.72){ kind = 5; return; }
  if (!black && x < 12.42 && x > -5.72){ kind = 6; return; }
}

void shuttleSurface(out vec3 col, out float rough, out float metal, out float bump){
  vec3 p = vObjPos; vec3 n = normalize(vObjN);
  vec2 pp = projPlane(p, n);
  float fw = length(fwidth(p));          // continuous (fwidth(pp) jumps where the projection plane switches)
  rough = 0.82; metal = 0.0; bump = 0.0;
  // kind: 0 white LRSI tile, 1 black HRSI tile, 2 RCC, 3 glass, 4 thruster port, 5 bay door, 6 FRSI felt, 7 AFRSI blanket
  int kind = 0; float aux = 0.0;
  if (uZone == 0){
    zoneFuselage(p, n, pp, fw, kind, aux);
  } else if (uZone == 1){ // wing / elevons: vMet = metres aft of the leading edge (99 = elevon)
    bool strake = abs(p.z) < 4.9;
    float d = vMet;
    if (d < (strake ? 0.0 : 0.42)) kind = 2;
    else if (n.y < -0.05 || d < (strake ? 1.25 : 0.68)) kind = 1;
    else kind = vChord > 0.62 ? 6 : 0;
    if (d > 90.0) kind = n.y < -0.05 ? 1 : 6;
  } else if (uZone == 2){ // vertical tail / rudder-speedbrake
    kind = vMet < 0.24 ? 1 : (vChord > 0.6 ? 7 : 0);
  } else if (uZone == 3){ // OMS pods
    kind = 0;
    if (n.x > 0.72 && p.y < 3.7) kind = 1;           // forward-facing HRSI cap
    if (n.x < -0.7) kind = 1;                          // aft face
    // aft RCS ports: side-firing (outboard wall) and up-firing (top)
    float rcs = 0.0;
    if (abs(n.z) > 0.6){ for (int i = 0; i < 4; i++) rcs = max(rcs, boxM(vec2(p.x, p.y), vec2(-10.35 - 0.36 * float(i), 3.62), vec2(0.12, 0.1), fw)); }
    if (n.y > 0.7){ for (int i = 0; i < 3; i++) rcs = max(rcs, boxM(vec2(p.x, abs(p.z)), vec2(-10.6 - 0.34 * float(i), 1.78), vec2(0.11, 0.13), fw)); }
    if (rcs > 0.5) kind = 4;
    else if (kind == 0 && p.x < -7.6) kind = 7;
  } else if (uZone == 4){ // body flap
    kind = 1;
  } else if (uZone == 5){ // base heat shield
    vec2 t = tileGrid(pp, vec2(0.2, 0.2), fw);
    col = vec3(0.055, 0.054, 0.052) * (0.75 + 0.5 * t.y) * (1.0 - 0.4 * t.x); rough = 0.85; bump = t.x; return;
  } else if (uZone == 6){ // engine heat-shield boots (white blankets)
    vec2 t = tileGrid(vec2(atan(p.z, p.y) * 1.6, length(p.yz)), vec2(0.18, 0.12), fw);
    col = vec3(0.80, 0.79, 0.76) * (1.0 - 0.25 * t.x); rough = 0.95; bump = t.x; return;
  }

  if (kind == 3){ col = vec3(0.006, 0.008, 0.011); rough = 0.04; metal = 0.0; bump = 0.0; return; }
  if (kind == 4){ col = vec3(0.008, 0.008, 0.008); rough = 0.95; bump = 1.0; return; }
  if (kind == 2){ // reinforced carbon-carbon: smooth mid grey panels
    vec2 t = tileGrid(pp, vec2(0.75, 0.55), fw);
    col = vec3(0.055, 0.055, 0.058) * (0.9 + 0.15 * t.y); rough = 0.78; bump = t.x * 0.4; return;
  }
  if (kind == 1){
    vec2 t = tileGridN(p, n, vec2(0.152, 0.152), fw);
    float v = t.y;
    col = vec3(0.030, 0.030, 0.032) * (0.78 + 0.45 * v);
    col = mix(col, vec3(0.052, 0.05, 0.047), step(0.975, v));         // replaced / lighter tiles
    col = mix(col, col * 0.55, t.x);
    rough = 0.84 + 0.1 * v; bump = t.x;
    return;
  }
  if (kind == 5){ // payload-bay doors: five panels per door, centreline seam, expansion joints
    float seam = 1.0 - smoothstep(0.008, 0.022 + fw, abs(p.z));
    float ux = (p.x + 5.72) / 3.627;
    seam = max(seam, 1.0 - smoothstep(0.008, 0.022 + fw, (0.5 - abs(fract(ux) - 0.5)) * 3.627));
    seam = max(seam, 1.0 - smoothstep(0.006, 0.018 + fw, abs(vChord - 3.02) * 2.0));   // hinge line at the sill
    vec2 t = tileGridN(p, n, vec2(0.9, 0.9), fw);
    float pan = th(vec2(floor(ux), p.z > 0.0 ? 1.0 : 0.0));
    col = mix(vec3(0.90, 0.90, 0.88), vec3(0.85, 0.86, 0.85), pan) * (1.0 - seam * 0.6) * (1.0 - 0.05 * t.x);
    rough = 0.86; bump = seam + 0.3 * t.x;
    return;
  }
  if (kind == 6 || kind == 7){ // FRSI felt (large smooth panels) / AFRSI quilted blankets
    float sz = kind == 6 ? 1.2 : 0.76;
    vec2 t = tileGridN(p, n, vec2(sz, sz * (kind == 6 ? 0.8 : 1.0)), fw);
    float stitch = 0.0;
    if (kind == 7){
      vec2 q = fract(pp / sz * 4.0) - 0.5;
      stitch = (1.0 - smoothstep(0.0, 0.08 + fw * 6.0, min(abs(q.x), abs(q.y)))) * (1.0 - smoothstep(0.03, 0.12, fw));
    }
    col = mix(vec3(0.85, 0.85, 0.82), vec3(0.80, 0.80, 0.78), t.y) * (1.0 - 0.18 * stitch) * (1.0 - (kind == 6 ? 0.14 : 0.3) * t.x);
    float st = th(vec2(floor(p.z * 5.0), 3.0));                // grime streaks along the flow
    col *= 1.0 - 0.05 * st * smoothstep(0.0, 8.0, -p.x);
    rough = 0.93; bump = t.x * 0.7 + stitch * 0.4;
    return;
  }
  // white LRSI tiles (8 x 8 in), a few greyer replacements
  vec2 t = tileGridN(p, n, vec2(0.203, 0.203), fw);
  col = mix(vec3(0.80, 0.80, 0.78), vec3(0.66, 0.66, 0.64), step(0.9, t.y)) * (1.0 - 0.38 * t.x);
  rough = 0.9; bump = t.x;
}

// markings painted on the tiles: "United States" + flag on the fuselage sides, flag + "USA" on the left wing
void markings(inout vec3 col){
  vec3 p = vObjPos; vec3 n = normalize(vObjN);
  vec4 d = vec4(0.0);
  if (uZone == 0 && abs(n.z) > 0.55 && p.y > -0.2 && p.y < 1.62){
    // flag forward (canton toward the nose on both sides), lettering aft of it
    float x0 = 9.3, fl = 1.95;
    // canton at the viewer's upper left on both sides (forward on the left side, aft on the right)
    vec2 uvF = p.z < 0.0 ? vec2((x0 - p.x) / fl, (p.y - 0.42) / 1.05) : vec2((p.x - (x0 - fl)) / fl, (p.y - 0.42) / 1.05);
    d = atlas(uvF, vec4(0.0, 0.0, 0.5, 0.5));
    // lettering aft of the flag, reading left -> right on both sides
    float L = 7.6, xa = 7.05, xb = xa - L;
    vec2 uvT = p.z < 0.0 ? vec2((xa - p.x) / L, (p.y - 0.22) / 1.4) : vec2((p.x - xb) / L, (p.y - 0.22) / 1.4);
    vec4 tx = atlas(uvT, vec4(0.0, 0.5, 1.0, 0.5));
    d = d.a > 0.0 ? d : tx;
  } else if (uZone == 1 && n.y > 0.4 && p.z < 0.0){
    // left wing upper surface: flag inboard, "USA" outboard (letters' tops toward the nose)
    vec2 uvF = vec2((p.z + 6.15) / 2.0, (p.x + 8.55) / 1.15);
    d = atlas(uvF, vec4(0.0, 0.0, 0.5, 0.5));
    vec2 uvU = vec2((p.z + 10.35) / 4.0, (p.x + 9.55) / 2.0);
    vec4 u = atlas(uvU, vec4(0.5, 0.0, 0.5, 0.5));
    d = d.a > 0.0 ? d : u;
  }
  col = mix(col, d.rgb, d.a * 0.96);
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
  const decal = { value: decalTexture() };
  m.onBeforeCompile = (sh) => {
    sh.uniforms.uZone = { value: zone };
    sh.uniforms.uDecal = decal;
    sh.uniforms.uBodyWet = WET.uBodyWet;
    sh.uniforms.uWetTime = WET.uWetTime;
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nattribute float aChord;\nattribute float aMet;\nvarying vec3 vObjPos;\nvarying vec3 vObjN;\nvarying float vChord;\nvarying float vMet;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvObjPos = position; vObjN = normal; vChord = aChord; vMet = aMet;');
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', '#include <common>\n' + TILE_GLSL + '\nuniform float uBodyWet;\nuniform float uWetTime;\n' + WET_GLSL)
      .replace('#include <color_fragment>', `#include <color_fragment>
vec3 tCol; float tRough, tMetal, tBump; shuttleSurface(tCol, tRough, tMetal, tBump); markings(tCol); diffuseColor.rgb *= tCol;
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
  m.customProgramCacheKey = () => 'orbiter-tile2';
  return m;
}

function metalMaterial(color, rough, metal, opts = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: metal, ...opts });
}

// SSME / OMS nozzle: brazed coolant tubes (outside), discoloured hot wall (inside)
function nozzleMaterial(inside, oms = false) {
  const m = new THREE.MeshStandardMaterial({
    color: inside ? 0x2e2620 : (oms ? 0x6e6c6a : 0xcfc6b8), roughness: inside ? 0.62 : 0.38, metalness: inside ? 0.5 : 0.7,
    side: inside ? THREE.FrontSide : THREE.BackSide, envMapIntensity: 1.0,   // lathe front faces point inward
  });
  m.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader.replace('#include <common>', '#include <common>\nvarying vec3 vLP;').replace('#include <begin_vertex>', '#include <begin_vertex>\nvLP = position;');
    sh.fragmentShader = sh.fragmentShader.replace('#include <common>', '#include <common>\nvarying vec3 vLP;')
      .replace('#include <color_fragment>', `#include <color_fragment>
        float ang = atan(vLP.z, vLP.x);
        float L = ${oms ? '2.1' : '3.1'};
        float t = clamp(-vLP.y / L, 0.0, 1.0);
        float tubesN = ${oms ? '0.0' : '540.0'};
        float fwA = fwidth(ang * tubesN);
        float tubes = tubesN > 0.0 ? mix(0.5, 0.5 + 0.5 * cos(ang * tubesN), 1.0 - smoothstep(0.3, 1.2, fwA)) : 0.5;
        vec3 tint = ${inside ? 'mix(vec3(0.16, 0.13, 0.11), mix(vec3(0.5, 0.4, 0.33), vec3(0.24, 0.21, 0.22), t), smoothstep(0.0, 0.25, t))' : 'mix(vec3(0.78, 0.72, 0.80), vec3(1.02, 0.98, 0.93), smoothstep(0.0, 0.45, t))'};
        diffuseColor.rgb *= tint * (0.86 + 0.18 * tubes);`)
      .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
        roughnessFactor = clamp(roughnessFactor + 0.12 * (0.5 - tubes), 0.05, 1.0);`);
  };
  m.customProgramCacheKey = () => 'nozzle2' + (inside ? 'i' : 'o') + (oms ? 'm' : '');
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
      boot: tileMaterial(6),
      tire: metalMaterial(0x161616, 0.88, 0.0), hub: metalMaterial(0x9a9da0, 0.32, 0.9), strut: metalMaterial(0xc4c8cc, 0.28, 0.95),
      dark: metalMaterial(0x1c1d1f, 0.7, 0.2), nozzle: nozzleMaterial(false), nozzleIn: nozzleMaterial(true),
      omsNozzle: nozzleMaterial(false, true), omsNozzleIn: nozzleMaterial(true, true),
      band: metalMaterial(0xa89f93, 0.5, 0.6),
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
    const hw = Math.max(0.05, FUSE.hw(x)), yt = FUSE.yt(x), yb = FUSE.yb(x);
    const H = yt - yb;
    // shoulder: payload-bay door hinge line (sill) along the mid fuselage, lower on the crew module
    let ys = lerp(1.70, 1.12, ss(12.15, 13.0, x));
    ys = lerp(ys, 0.62, ss(13.0, 16.6, x));
    ys = lerp(ys, yb + 0.46 * H, ss(16.6, 20.4, x));
    ys = Math.min(Math.max(ys, yb + 0.32 * H), yt - 0.22 * H);
    const lean = 0.07 * ss(12.8, 16.2, x) + 0.06 * ss(17.0, 20.5, x);          // sides lean in toward the roof
    const wArc = 0.8 * (1 - ss(11.9, 12.5, x));                                  // arched bay doors vs. rounded cabin
    const nt = lerp(lerp(2.0, 2.45, ss(12.2, 13.2, x)), 2.05, ss(15.6, 18.5, x));
    let rc = lerp(0.30, 0.62, ss(12.0, 18.0, x));
    rc = Math.min(rc, 0.42 * hw, 0.42 * (ys - yb));
    return { hw, yt, yb, ys, lean, wArc, nt, rc };
  }
  // right half of the section from the bottom centre line to the top centre line: [[z, y, seg], ...]
  _half(x) {
    const s = this._sectionParams(x);
    const zc = s.hw, zS = s.hw * (1 - s.lean);
    let dx = zS - zc, dy = s.ys - s.yb; const dl = Math.hypot(dx, dy); dx /= dl; dy /= dl;
    const phi = Math.acos(Math.max(-1, Math.min(1, dx)));
    const L = s.rc * Math.tan(phi / 2);
    const T1 = [zc - L, s.yb], T2 = [zc + dx * L, s.yb + dy * L], O = [zc - L, s.yb + s.rc];
    const a1 = -Math.PI / 2, a2 = Math.atan2(T2[1] - O[1], T2[0] - O[0]);
    const out = [];
    for (let k = 0; k < NB; k++) { const t = k / NB; out.push([T1[0] * t, s.yb, t]); }
    for (let k = 0; k < NC; k++) { const t = k / NC, a = lerp(a1, a2, t); out.push([O[0] + s.rc * Math.cos(a), O[1] + s.rc * Math.sin(a), 1 + t]); }
    for (let k = 0; k < NW; k++) { const t = k / NW; out.push([lerp(T2[0], zS, t), lerp(T2[1], s.ys, t), 2 + t]); }
    const rise = s.yt - s.ys;
    const R = (zS * zS + rise * rise) / (2 * rise), yc = s.yt - R, a0 = Math.atan2(s.ys - yc, zS);
    for (let k = 0; k <= NT; k++) {
      const t = k / NT, th = t * Math.PI / 2;
      const c = Math.cos(th), sn = Math.sin(th);
      const ez = zS * Math.pow(c, 2 / s.nt), ey = s.ys + rise * Math.pow(sn, 2 / s.nt);
      const a = a0 + (Math.PI / 2 - a0) * t;
      const cz = R * Math.cos(a), cy = yc + R * Math.sin(a);
      out.push([lerp(ez, cz, s.wArc), lerp(ey, cy, s.wArc), 3 + t]);
    }
    out[out.length - 1][0] = 0;
    return out;
  }
  _ring(x) {
    const h = this._half(x);
    const pts = [], seg = [];
    for (const [z, y, sg] of h) { pts.push([x, y, z]); seg.push(sg); }
    for (let i = h.length - 2; i >= 1; i--) { pts.push([x, h[i][1], -h[i][0]]); seg.push(h[i][2]); }
    return [pts, seg];
  }
  _buildFuselage() {
    const xs = [];
    // RCC nose cap: ellipsoidal closure ahead of x = CAP_X
    const capRings = [], capAttr = [];
    const [r0, a0] = this._ring(CAP_X);
    const cy = (FUSE.yt(CAP_X) + FUSE.yb(CAP_X)) / 2 + 0.0;
    for (let k = 0; k <= 10; k++) {
      const u = k / 10;                                   // 0 = tip, 1 = CAP_X
      const xx = NOSE_X - (NOSE_X - CAP_X) * (1 - Math.cos(u * Math.PI / 2));
      const s = Math.sqrt(Math.max(0, 1 - ((xx - CAP_X) / (NOSE_X - CAP_X)) ** 2));
      capRings.push(r0.map(([, y, z]) => [xx, cy + (y - cy) * Math.max(s, 0.002), z * Math.max(s, 0.002)]));
      capAttr.push(a0);
    }
    for (let x = CAP_X - 0.12; x > 18.0; x -= 0.12) xs.push(x);
    for (let x = 18.0; x > 12.0; x -= 0.16) xs.push(x);
    for (let x = 12.0; x > -7.0; x -= 0.5) xs.push(x);
    for (let x = -7.0; x > BASE_X; x -= 0.3) xs.push(x);
    xs.push(BASE_X);
    const rings = capRings.slice(), attrs = capAttr.slice();
    for (const x of xs) { const [r, a] = this._ring(x); rings.push(r); attrs.push(a); }
    const geo = loft(rings, { flip: true, attr: attrs });
    const mesh = new THREE.Mesh(geo, this.mat.fuse);
    mesh.name = 'fuselage';
    this.model.add(mesh);
    // base heat shield (aft bulkhead)
    const [baseRing] = this._ring(BASE_X);
    const bp = loft([baseRing, baseRing.map(([x, y, z]) => [x - 0.02, y, z])], { capEnd: true, flip: true });
    this.model.add(new THREE.Mesh(bp, this.mat.base));
    // aft lower fuselage shelf carrying the body flap (under the engines)
    const bf = this.geom.bodyflap, hw = bf.half_span;
    const yTop = -bf.z + 0.55, yBot = -bf.z - 0.02;
    const shelf = [];
    for (const x of [BASE_X + 0.02, bf.hinge_x + 0.02]) {
      shelf.push([[x, yBot, -hw - 0.05], [x, yBot, hw + 0.05], [x, yTop, hw - 0.05], [x, yTop, -hw + 0.05]]);
    }
    const sh = loft(shelf, { capEnd: true, flip: false });
    this.model.add(new THREE.Mesh(sh, this.mat.flap));
  }

  // ---------------- wing + elevons
  _wingStation(y) {
    const w = this.geom.wing;
    const le = w.le, te = w.te;
    let xle;
    if (y <= le[0][1]) xle = le[0][0] + (le[0][1] - y) * Math.tan(81 * D2R);
    else if (y <= le[1][1]) xle = lerp(le[0][0], le[1][0], (y - le[0][1]) / (le[1][1] - le[0][1]));
    else xle = lerp(le[1][0], le[2][0], (y - le[1][1]) / (le[2][1] - le[1][1]));
    // rounded strake / wing leading-edge break
    const yb = le[1][1];
    if (Math.abs(y - yb) < 0.6) {
      const t = (y - yb + 0.6) / 1.2;
      const xa = lerp(le[0][0], le[1][0], (yb - 0.6 - le[0][1]) / (le[1][1] - le[0][1]));
      const xb = lerp(le[1][0], le[2][0], 0.6 / (le[2][1] - le[1][1]));
      const xq = (1 - t) * (1 - t) * xa + 2 * (1 - t) * t * le[1][0] + t * t * xb;
      xle = xq;
    }
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
    // returns [points, chordAttr, metresAttr]; loop: upper xiB->xiA, lower xiA->xiB, (+ hinge face)
    const pts = [], ach = [], am = [];
    const xis = cosineSpace(xiA, xiB, nU, xiA === 0);
    const P = (xi, up) => {
      const ht = halfThick(xi, st.tc) * st.chord;
      const yy = st.yc + (up ? ht * 1.18 : -ht * 0.82);
      return [st.xle - xi * st.chord, yy, side * st.y];
    };
    for (let i = xis.length - 1; i >= 0; i--) { pts.push(P(xis[i], true)); ach.push(xis[i]); am.push(xis[i] * st.chord); }
    for (let i = 1; i < xis.length; i++) { pts.push(P(xis[i], false)); ach.push(xis[i]); am.push(xis[i] * st.chord); }
    if (withHingeFace) {
      const lo = P(xiB, false), upp = P(xiB, true);
      for (let k = 0; k <= 4; k++) { const t = k / 4; pts.push([lo[0], lerp(lo[1], upp[1], t), lo[2]]); ach.push(xiB); am.push(xiB * st.chord); }
    }
    return [pts, ach, am];
  }
  _buildWings() {
    const w = this.geom.wing;
    const ys = [];
    for (let y = 2.0; y < w.le[1][1] - 0.7; y += 0.18) ys.push(y);
    for (let y = w.le[1][1] - 0.7; y < w.le[1][1] + 0.7; y += 0.1) ys.push(y);
    for (let y = w.le[1][1] + 0.7; y < w.tip_y - 0.001; y += 0.3) ys.push(y);
    ys.push(w.tip_y);
    for (const side of [1, -1]) {
      const rings = [], attrs = [], mets = [];
      for (const y of ys) {
        const st = this._wingStation(y);
        st.y = y;
        const xiH = 1 - st.ce / st.chord;
        const [pts, ach, am] = this._airfoilLoop(st, 0, Math.max(0.3, xiH), side, 28, true);
        rings.push(pts); attrs.push(ach); mets.push(am);
      }
      const geo = loft(rings, { capEnd: true, flip: side < 0, attr: attrs, attr2: mets });
      const mesh = new THREE.Mesh(geo, this.mat.wing);
      mesh.name = side > 0 ? 'wing_r' : 'wing_l';
      this.model.add(mesh);
      // elevons
      const spans = [['i', 2.62, 7.22], ['o', 7.28, 11.86]];
      for (const [io, y0, y1] of spans) {
        const ers = [], eat = [], emt = [];
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
          ers.push(pts); eat.push(ach); emt.push(ach.map(() => 99));
        }
        const eg = loft(ers, { capStart: true, capEnd: true, flip: side < 0, attr: eat, attr2: emt });
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

  // ---------------- vertical tail + split rudder / speedbrake
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
      const xis = cosineSpace(0, xiB, 22, true);
      const pts = [], ach = [], am = [];
      for (let i = xis.length - 1; i >= 0; i--) { const ht = halfThick(xis[i], st.tc) * st.chord; pts.push([st.xle - xis[i] * st.chord, Y, ht]); ach.push(xis[i]); am.push(xis[i] * st.chord); }
      for (let i = 1; i < xis.length; i++) { const ht = halfThick(xis[i], st.tc) * st.chord; pts.push([st.xle - xis[i] * st.chord, Y, -ht]); ach.push(xis[i]); am.push(xis[i] * st.chord); }
      if (hinge) { const ht = halfThick(xiB, st.tc) * st.chord; for (let k = 0; k <= 4; k++) { pts.push([st.xle - xiB * st.chord, Y, lerp(-ht, ht, k / 4)]); ach.push(xiB); am.push(xiB * st.chord); } }
      else { for (let k = 0; k <= 4; k++) { pts.push([st.xle - st.chord, Y, 0]); ach.push(1); am.push(st.chord); } }
      return [pts, ach, am];
    };
    const mk = (Y0, Y1, n, rud) => {
      const rings = [], at = [], mt = [];
      for (let k = 0; k <= n; k++) {
        const Y = lerp(Y0, Y1, k / n);
        const [p, a, m] = finLoop(Y, rud ? 1 - rd.chord_frac : 1.0, rud);
        rings.push(p); at.push(a); mt.push(m);
      }
      return { rings, at, mt };
    };
    const parts = [mk(yRoot, r0, 4, false), mk(r0, r1, 14, true), mk(r1, yTip, 2, false)];
    parts.forEach((pt, i) => {
      const g = loft(pt.rings, { capStart: i === 0, capEnd: i === 2, attr: pt.at, attr2: pt.mt, flip: true });
      const m = new THREE.Mesh(g, this.mat.tail);
      m.name = 'fin' + i;
      this.model.add(m);
    });
    // rudder halves (left: z<0, right: z>0)
    for (const half of [1, -1]) {
      const rings = [], at = [], mt = [];
      for (let k = 0; k <= 12; k++) {
        const Y = lerp(r0 + 0.02, r1 - 0.02, k / 12);
        const st = this._finStation(Y);
        const xiH = 1 - rd.chord_frac;
        const xis = cosineSpace(xiH + 0.003, 1, 10, false);
        const pts = [], ach = [];
        for (let i = xis.length - 1; i >= 0; i--) { const ht = halfThick(xis[i], st.tc) * st.chord; pts.push([st.xle - xis[i] * st.chord, Y, half * ht]); ach.push(0.9); }
        for (let i = 0; i < xis.length; i++) pts.push([st.xle - xis[i] * st.chord, Y, half * 0.002]), ach.push(0.9);
        rings.push(pts); at.push(ach); mt.push(ach.map(() => 9));
      }
      const g = loft(rings, { capStart: true, capEnd: true, attr: at, attr2: mt, flip: half > 0 });
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

  // ---------------- OMS / RCS pods
  // pod section (right side) at full size, outline from the inboard bottom (buried in the
  // fuselage) around the outboard wall, over the crown and down the inboard side
  _podSection(f, M) {
    const P = POD;
    const zc = P.zc, base = P.yBase;
    // control polygon of a rounded asymmetric arch
    const outline = [];
    const H = P.yApex - base;
    for (let j = 0; j < M; j++) {
      const t = j / M * Math.PI * 2;                 // 0 = outboard, pi/2 = top
      const c = Math.cos(t), s = Math.sin(t);
      const hw = c >= 0 ? (P.zOut - zc) : (zc - P.zIn);
      const e = c >= 0 ? 3.2 : 2.3;                    // squarer outboard wall, rounder inboard side
      let z = zc + hw * Math.sign(c) * Math.abs(c) ** (2 / e);
      let y = base + 0.55 * H + (s >= 0 ? 0.45 * H : 0.62 * H) * Math.sign(s) * Math.abs(s) ** (2 / (s >= 0 ? 2.4 : 3.0));
      // rise out of the fuselage at the front: shrink toward the buried base
      const yb = 2.55;
      y = yb + (y - yb) * f;
      z = zc + (z - zc) * lerp(0.62, 1, Math.sqrt(f));
      outline.push([z, y]);
    }
    return outline;
  }
  _buildPods() {
    const P = POD, M = 56;
    const xs = [];
    for (let k = 0; k <= 22; k++) xs.push(lerp(P.xf, P.xfull, 1 - Math.cos(k / 22 * Math.PI / 2)));
    for (let x = P.xfull - 0.3; x > P.xa + 0.35; x -= 0.3) xs.push(x);
    for (let k = 0; k <= 6; k++) xs.push(lerp(P.xa + 0.35, P.xa, Math.sin(k / 6 * Math.PI / 2)));
    for (const side of [1, -1]) {
      const rings = [];
      for (const x of xs) {
        let f = Math.min(1, (P.xf - x) / (P.xf - P.xfull));
        f = Math.sin(f * Math.PI / 2) ** 0.85;
        // slight inward rounding of the aft face edge
        const ra = 1 - 0.06 * ss(P.xa + 0.35, P.xa, x);
        const sec = this._podSection(Math.max(f, 0.02), M);
        rings.push(sec.map(([z, y]) => [x, 2.55 + (y - 2.55) * ra, side * (P.zc + (z - P.zc) * ra)]));
      }
      const g = loft(rings, { capEnd: true, flip: side > 0 });
      const m = new THREE.Mesh(g, this.mat.pod);
      m.name = side > 0 ? 'oms_pod_r' : 'oms_pod_l';
      this.model.add(m);
      // OMS engine nozzle on the aft face, canted slightly outboard and up
      const inset = 0.5;
      const noz = this._nozzle(0.24, 0.58, 1.6 + inset, 36, true);
      noz.position.set(P.xa + inset, 3.55, side * P.zc);
      noz.rotation.z = -Math.PI / 2;
      noz.rotateX(-side * 0.05);
      noz.rotateZ(-0.06);
      this.model.add(noz);
      // engine collar / heat-shield ring where the nozzle leaves the aft face
      const collar = new THREE.TorusGeometry(0.43, 0.07, 10, 36); collar.rotateY(Math.PI / 2);
      const cm = new THREE.Mesh(collar, this.mat.boot);
      cm.position.set(P.xa - 0.02, 3.55, side * P.zc);
      this.model.add(cm);
    }
  }

  _nozzle(rThroat, rExit, len, seg, oms = false) {
    // bell (Rao-like) contour: fast initial expansion, nearly straight near the exit
    const pts = [];
    for (let i = 0; i <= 32; i++) {
      const t = i / 32;
      const r = rThroat + (rExit - rThroat) * (1 - (1 - t) ** 2.3);
      pts.push(new THREE.Vector2(r, -t * len));
    }
    const geo = new THREE.LatheGeometry(pts, seg);
    const grp = new THREE.Group();
    // throat / injector side closed (no see-through to the base)
    const plug = new THREE.CircleGeometry(rThroat * 1.01, seg); plug.rotateX(Math.PI / 2); plug.translate(0, -0.02, 0);
    grp.add(new THREE.Mesh(plug, this.mat.dark));
    grp.add(new THREE.Mesh(geo, oms ? this.mat.omsNozzle : this.mat.nozzle));
    grp.add(new THREE.Mesh(geo, oms ? this.mat.omsNozzleIn : this.mat.nozzleIn));
    // exit lip
    const lip = new THREE.TorusGeometry(rExit, oms ? 0.018 : 0.035, 8, seg); lip.rotateX(Math.PI / 2); lip.translate(0, -len, 0);
    grp.add(new THREE.Mesh(lip, this.mat.band));
    if (!oms) {
      // hatbands (stiffener rings) and the coolant manifold at the exit
      for (const t of [0.28, 0.52, 0.76]) {
        const r = rThroat + (rExit - rThroat) * (1 - (1 - t) ** 2.3);
        const ring = new THREE.TorusGeometry(r + 0.03, 0.018, 6, seg); ring.rotateX(Math.PI / 2); ring.translate(0, -t * len, 0);
        grp.add(new THREE.Mesh(ring, this.mat.band));
      }
      const man = new THREE.TorusGeometry(rExit + 0.025, 0.032, 8, seg); man.rotateX(Math.PI / 2); man.translate(0, -len + 0.12, 0);
      grp.add(new THREE.Mesh(man, this.mat.band));
    }
    return grp;   // axis along -Y of the group
  }
  _buildEngines() {
    const e = this.geom.engines;
    for (const m of e.mounts) {
      const [x, y, z] = m;
      const noz = this._nozzle(0.62, e.exit_radius, e.length, 64);
      const pos = new THREE.Vector3(x, -z, y);
      noz.position.copy(pos);
      noz.rotation.z = -Math.PI / 2;       // lathe -Y -> model -X (aft)
      // stowed for entry: #1 pitched up ~2.6 deg, #2/#3 down ~1.7 deg and out ~1.1 deg, so the
      // 2.4 m exits clear each other (lips ~7 cm apart)
      if (y === 0) noz.rotateZ(-0.045);
      else { noz.rotateZ(0.03); noz.rotateX(-Math.sign(y) * 0.02); }
      this.model.add(noz);
      // engine heat-shield boot: quilted blanket collar between the nozzle and the base heat shield
      const prof = [];
      for (let i = 0; i <= 12; i++) {
        const t = i / 12;
        prof.push(new THREE.Vector2(lerp(1.08, 0.64, t), -0.32 * Math.sin(t * Math.PI / 2) ** 0.8 + 0.02));
      }
      const boot = new THREE.Mesh(new THREE.LatheGeometry(prof, 48), this.mat.boot);
      boot.position.set(x - 0.06, -z, y);
      boot.rotation.z = -Math.PI / 2;
      this.model.add(boot);
    }
  }

  // ---------------- landing gear
  _wheelPair(r, w, sep) {
    const g = new THREE.Group();
    const tire = new THREE.CylinderGeometry(r, r, w, 32); tire.rotateX(Math.PI / 2);
    const hub = new THREE.CylinderGeometry(r * 0.58, r * 0.58, w * 1.04, 24); hub.rotateX(Math.PI / 2);
    for (const s of [-1, 1]) {
      const wg = new THREE.Group();
      wg.add(new THREE.Mesh(tire, this.mat.tire));
      wg.add(new THREE.Mesh(hub, this.mat.hub));
      wg.position.z = s * sep;
      g.add(wg);
    }
    const axle = new THREE.CylinderGeometry(0.06, 0.06, sep * 2, 10); axle.rotateX(Math.PI / 2);
    g.add(new THREE.Mesh(axle, this.mat.strut));
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
      const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.17, strutLen, 16), this.mat.strut);
      strut.position.y = -strutLen / 2;
      root.add(strut);
      const slider = new THREE.Group();   // moves with compression
      slider.position.y = -strutLen;
      root.add(slider);
      const piston = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.6, 14), this.mat.hub);
      piston.position.y = 0.3;
      slider.add(piston);
      const wheels = this._wheelPair(c.wheel_radius, isNose ? 0.22 : 0.38, isNose ? 0.2 : 0.3);
      slider.add(wheels);
      // drag brace + torque link
      const brace = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, strutLen * 0.9, 10), this.mat.strut);
      brace.position.set(isNose ? -0.6 : 0.7, -strutLen * 0.45, 0);
      brace.rotation.z = (isNose ? 1 : -1) * 0.5;
      root.add(brace);
      const link = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.08, 0.06), this.mat.hub);
      link.position.set(isNose ? 0.22 : -0.22, -strutLen + 0.55, 0);
      link.rotation.z = isNose ? -0.6 : 0.6;
      root.add(link);
      // doors (two per well), hinge on the long edges; black tiles outside
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
    const m = new THREE.Mesh(body, this.mat.pod);
    m.position.set(x1, this.chuteAttachLocal.y, 0);
    this.model.add(m);
    const cap = new THREE.CircleGeometry(1, 28);
    cap.rotateY(-Math.PI / 2);                       // facing aft (-X)
    cap.scale(1, 0.6, 0.4);
    const door = new THREE.Mesh(cap, this.mat.pod);
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
