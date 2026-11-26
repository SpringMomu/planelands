// Airflow around the orbiter, drawn from the simulator's own aerodynamic model (sim/aero.py).
//
// The force model is a component build-up (lifting strips + slender body); the picture is made from
// those elements with the loads they computed, on the skin and on two kinds of cut plane:
//
// 1. Surface pressure paint (all modes).  The aero message carries, per strip, the terms of its
//    normal-force coefficient (attached potential flow from the flow angle, from the flap deflection,
//    Polhamus vortex lift, separated flat plate) and its dynamic-pressure ratio.  Each term is spread
//    over the chord with its own shape - flat-plate sqrt((1-x)/x) / thin-airfoil flap load subsonic,
//    Ackeret uniform load supersonic, a leading-edge-vortex hump, a uniform separated load - each
//    normalised so that it integrates back to the model's force.  The local surface speed is the
//    strip's inflow (free stream + rotation, as in the model) plus half the vortex-sheet strength
//    (thickness term, Riegels leading-edge factor) and the spanwise flow induced by the spanwise
//    gradient of the strip circulation (outboard under the tips, inboard on top); Cp = 1 - (V/U)^2.
//    Fuselage / pods: the slender-body crossflow around the elliptic sections (the model's Munk
//    term) plus the axial flow of its thickness sources.  Base: the model's base pressure.
//    Surface streamlines: the surface velocity is integrated along the loft from the stagnation line
//    (travel time + lane of each vertex), and dashes run along the lanes at the local speed; where the
//    model's flow is separated the lanes wander and the dashes slow down.
// 2. Wing section (mode "section"): exact 2-D potential flow (Hess-Smith source + vortex panels)
//    around the mesh's own section at the chosen span station, elevon deflected as drawn.  Its
//    circulation is the model's strip circulation (Kutta-Joukowski), so the effective angle of attack
//    comes out of the solution and the difference to the geometric one is the induced downwash.
//    Streamlines are traced on the GPU and drawn as contour lines of their upstream height (equal
//    flux between neighbours: they crowd where the air speeds up), with pulses moving at the local
//    speed; the outer wing on the camera side is cut away at the section.
// 3. Wake plane (mode "wake"): a plane across the stream behind the vehicle, with the crossflow of
//    the strips' horseshoe-vortex system (bound + trailing legs, Betz roll-up toward each half wing's
//    vorticity centroid, ground images, Mach-cone limit) drawn as animated line-integral convolution
//    and coloured by downwash / upwash.
import * as THREE from 'three';
import { FullscreenPass, makeRT } from '../render/pass.js';
import { PAINT } from './airflow_paint.js';
import { halfThick } from './shuttle.js';

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

const COMMON = /* glsl */ `
const float PI4 = 12.566370614;
uniform sampler2D uStrip, uGam, uBody, uSrc;
uniform int uNS, uNB, uNSrc;
uniform vec3 uVa, uOm, uD;
uniform float uU, uMach, uGroundOn, uSB, uTime;
uniform vec4 uGround;            // world up in body axes, CG height above the ground

// finite vortex segment a -> b, unit circulation (Biot-Savart, core-regularised)
vec3 segV(vec3 p, vec3 a, vec3 b, float rc2){
  vec3 r1 = p - a, r2 = p - b, r0 = b - a;
  vec3 c = cross(r1, r2);
  float den = dot(c, c) + rc2 * dot(r0, r0);
  float l1 = max(length(r1), 1e-4), l2 = max(length(r2), 1e-4);
  return c * (dot(r0, r1 / l1 - r2 / l2) / (PI4 * den));
}
// semi-infinite vortex from a along d (vortex vector +d), unit circulation
vec3 semiV(vec3 p, vec3 a, vec3 d, float rc){
  vec3 r = p - a;
  vec3 c = cross(d, r);
  float s = dot(d, r);
  float rcs = rc + 0.015 * max(s, 0.0);              // core grows downstream (diffusion)
  return c * ((1.0 + s / max(length(r), 1e-4)) / (PI4 * (dot(c, c) + rcs * rcs)));
}
// supersonic: only the inside of the element's downstream Mach cone feels it
float machGate(vec3 r){
  if (uMach <= 1.0) return 1.0;
  float B = sqrt(uMach * uMach - 1.0);
  float s = dot(r, uD);
  return smoothstep(-0.4, 0.4, s - B * length(r - s * uD));
}
float segDist(vec3 p, vec3 a, vec3 b){
  vec3 ab = b - a;
  float t = clamp(dot(p - a, ab) / max(dot(ab, ab), 1e-6), 0.0, 1.0);
  return length(p - a - t * ab);
}

// lifting strips: horseshoe vortices (+ separated-wake tag of stalled strips)
vec3 vortexV(vec3 p, inout float sep, bool tag){
  vec3 v = vec3(0.0);
  for (int i = 0; i < uNS; i++){
    vec4 s0 = texelFetch(uStrip, ivec2(i, 0), 0);
    vec4 s1 = texelFetch(uStrip, ivec2(i, 1), 0);
    vec2 g = texelFetch(uGam, ivec2(i, 0), 0).xy;
    vec3 a = s0.xyz, b = s1.xyz;
    float c = s0.w;
    vec3 mid = 0.5 * (a + b);
    if (tag){
      float off = max(1.0 - g.y, s1.w > 1.5 ? uSB : 0.0);       // surf 2 = fin (split rudder)
      if (off > 0.03){
        vec3 te = vec3(0.75 * c, 0.0, 0.0);
        float s = dot(p - mid + te, uD);
        if (s > -0.5 * c && s < 8.0 * c){
          float w = 0.3 * c + 0.22 * max(s, 0.0);
          float dd = segDist(p, a - te, b - te);
          sep = max(sep, off * (1.0 - smoothstep(0.5 * w, w, dd)) * (1.0 - smoothstep(4.0 * c, 8.0 * c, s)));
        }
      }
    }
    float gate = machGate(p - mid);
    if (abs(g.x) < 1e-3 || gate < 1e-3) continue;
    float rc = max(0.22, 0.1 * c);
    float rc2 = rc * rc;
    // trailing legs run from the edges to the rolled-up wake (Ra, Rb), then on downstream
    vec4 ra = texelFetch(uGam, ivec2(i, 1), 0), rb = texelFetch(uGam, ivec2(i, 2), 0);
    // chordwise: flat-plate loading lumped on two panels (82 % at c/8, 18 % at 5c/8)
    vec3 sh = vec3(0.125 * c, 0.0, 0.0);
    vec3 a1 = a + sh, b1 = b + sh;
    vec3 v1 = segV(p, a1, b1, rc2) + segV(p, b1, rb.xyz, rc2) - segV(p, a1, ra.xyz, rc2);
    vec3 sh2 = vec3(-0.375 * c, 0.0, 0.0);
    vec3 a2 = a + sh2, b2 = b + sh2;
    vec3 v2 = segV(p, a2, b2, rc2) + segV(p, b2, rb.xyz, rc2) - segV(p, a2, ra.xyz, rc2);
    vec3 v3 = semiV(p, rb.xyz, uD, rb.w) - semiV(p, ra.xyz, uD, ra.w);
    v += g.x * gate * (0.818 * v1 + 0.182 * v2 + v3);
  }
  return v;
}
// axial thickness sources of fuselage and OMS pods
vec3 srcV(vec3 p){
  vec3 v = vec3(0.0);
  for (int i = 0; i < uNSrc; i++){
    vec4 s = texelFetch(uSrc, ivec2(i, 0), 0);
    vec3 r = p - s.xyz;
    float r2 = dot(r, r) + 0.6;
    v += r * (uVa.x * s.w * machGate(r) / (PI4 * r2 * sqrt(r2)));
  }
  return v;
}
vec2 cmul(vec2 a, vec2 b){ return vec2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x); }
vec2 cdiv(vec2 a, vec2 b){ return vec2(a.x * b.x + a.y * b.y, a.y * b.x - a.x * b.y) / max(dot(b, b), 1e-12); }
vec2 csqrt(vec2 z){
  float r = length(z);
  float re = sqrt(max(0.5 * (r + z.x), 0.0)), im = sqrt(max(0.5 * (r - z.x), 0.0));
  return vec2(re, z.y < 0.0 ? -im : im);
}
// slender-body crossflow around the local elliptic section (+ lee-side / base separation tags);
// inside = 1 when p is inside the fuselage
vec3 bodyV(vec3 p, inout float sep, inout float inside){
  float x0 = texelFetch(uBody, ivec2(0, 0), 0).x;
  if (p.x > x0) return vec3(0.0);
  vec4 A = texelFetch(uBody, ivec2(uNB - 1, 0), 0), B = texelFetch(uBody, ivec2(uNB - 1, 1), 0);
  float fade = 1.0, behind = 0.0;
  if (p.x < A.x){
    behind = A.x - p.x;
    fade = exp(-behind / max(1.2 * B.x, 0.5));
  } else {
    for (int i = 0; i < uNB - 1; i++){
      vec4 a0 = texelFetch(uBody, ivec2(i, 0), 0), a1 = texelFetch(uBody, ivec2(i + 1, 0), 0);
      if (p.x <= a0.x && p.x >= a1.x){
        float t = (a0.x - p.x) / max(a0.x - a1.x, 1e-4);
        A = mix(a0, a1, t);
        B = mix(texelFetch(uBody, ivec2(i, 1), 0), texelFetch(uBody, ivec2(i + 1, 1), 0), t);
        break;
      }
    }
  }
  float ha = max(A.w, 0.05), hb = max(B.x, 0.05);      // semi-axes y (half width), z (half height)
  vec3 q = vec3(p.x, A.y, A.z);
  vec3 ua = -(uVa + cross(uOm, q));                     // air at the section
  vec2 Vc = ua.yz;
  vec2 zz = p.yz - A.yz;
  float er = length(zz / vec2(ha, hb));                 // 1 on the section contour
  if (behind == 0.0 && er < 1.0) inside = 1.0;
  // lee-side separation at high crossflow (viscous crossflow), separated wake behind the blunt base
  float cr = length(Vc) / max(uU, 1.0);
  float lee = dot(zz, Vc) / max(length(zz) * length(Vc), 1e-6);
  if (behind == 0.0)
    sep = max(sep, smoothstep(0.25, 0.6, cr) * smoothstep(-0.2, 0.5, lee) * (1.0 - smoothstep(1.2, 2.2, er)));
  else
    sep = max(sep, (1.0 - smoothstep(0.9, 1.7, er)) * (1.0 - smoothstep(10.0, 30.0, behind)));
  // potential flow past an ellipse: z = s + c^2 / (4 s) maps the circle |s| = (a + b) / 2 onto it
  float c2 = ha * ha - hb * hb, R = 0.5 * (ha + hb);
  vec2 rt = csqrt(cmul(zz, zz) - vec2(c2, 0.0));
  vec2 s1 = 0.5 * (zz + rt), s2 = 0.5 * (zz - rt);
  vec2 s = dot(s1, s1) >= dot(s2, s2) ? s1 : s2;
  vec2 ss = cmul(s, s);
  vec2 Ubar = vec2(Vc.x, -Vc.y);
  vec2 w = cdiv(Ubar - R * R * cdiv(Vc, ss), vec2(1.0, 0.0) - cdiv(vec2(c2 * 0.25, 0.0), ss));
  vec2 pert = vec2(w.x, -w.y) - Vc;
  if (er < 1.0) pert *= er * er;                        // (inside: only to keep particles moving out)
  return vec3(0.0, pert) * fade;
}
vec3 turb(vec3 p, float t){
  vec3 q = p * 0.8;
  return vec3(sin(q.y * 1.7 + 2.0 * sin(q.z * 2.3 + t * 3.1)) + 0.5 * sin(q.z * 3.9 - t * 5.3),
              sin(q.z * 1.9 + 2.0 * sin(q.x * 2.1 + t * 2.7)) + 0.5 * sin(q.x * 4.3 + t * 4.9),
              sin(q.x * 1.5 + 2.0 * sin(q.y * 2.9 + t * 3.7)) + 0.5 * sin(q.y * 3.7 - t * 6.1));
}
// air velocity relative to the body at p (body axes) + separation tag (w); inside -> sep < 0 flag
vec4 flow(vec3 p, out float inside){
  float sep = 0.0;
  inside = 0.0;
  vec3 u = -(uVa + cross(uOm, p));
  u += vortexV(p, sep, true) + srcV(p) + bodyV(p, sep, inside);
  if (uGroundOn > 0.5){
    vec3 n = uGround.xyz;
    vec3 pr = p - 2.0 * (uGround.w + dot(n, p)) * n;
    float sd = 0.0;
    vec3 ui = vortexV(pr, sd, false) + srcV(pr);
    u += ui - 2.0 * dot(ui, n) * n;
  }
  if (sep > 0.02) u += sep * uU * 0.2 * turb(p, uTime);
  return vec4(u, sep);
}
`;

// ------------------------------------------------------------------ section: GPU passes
const SEC_VEL = /* glsl */ `
uniform sampler2D uNodes;
uniform int uN;
uniform float uGam;
uniform vec2 uVinf;
uniform vec4 uDom;
void main(){
  vec2 P = mix(uDom.xy, uDom.zw, vUv);
  vec2 V = uVinf;
  int crs = 0;
  vec4 a = texelFetch(uNodes, ivec2(0, 0), 0);
  for (int j = 0; j < uN; j++){
    vec4 b = texelFetch(uNodes, ivec2(j + 1, 0), 0);
    vec2 S = a.xy, E = b.xy;
    vec2 r1 = S - P, r2 = E - P;
    float l1 = max(length(r1), 1e-6), l2 = max(length(r2), 1e-6);
    float beta = atan(r1.x * r2.y - r1.y * r2.x, dot(r1, r2));
    float L = log(l1 / l2);
    vec2 dir = (E - S) / max(length(E - S), 1e-9);
    // panel frame: source (L, beta) / 2pi, vortex (beta, -L) / 2pi
    vec2 vl = (a.z * vec2(L, beta) + uGam * vec2(beta, -L)) * 0.159154943;
    V += dir * vl.x + vec2(-dir.y, dir.x) * vl.y;
    if ((S.y > P.y) != (E.y > P.y) && S.x + (P.y - S.y) * (E.x - S.x) / (E.y - S.y) > P.x) crs++;
    a = b;
  }
  outColor = vec4(V, length(V), float(crs & 1));
}`;

// back-trace every texel to the upstream edge: streamline label (height across the free stream) and
// travel time (chords / U)
const SEC_TRACE = /* glsl */ `
uniform sampler2D uVel;
uniform vec4 uDom;
uniform vec2 uDir;
uniform float uSref;
vec4 velAt(vec2 p){ return texture(uVel, (p - uDom.xy) / (uDom.zw - uDom.xy)); }
void main(){
  vec2 P = mix(uDom.xy, uDom.zw, vUv);
  vec4 c = texture(uVel, vUv);
  if (c.w > 0.5){ outColor = vec4(0.0, 0.0, c.z, 1.0); return; }
  vec2 p = P;
  float tau = 0.0;
  const float h = 0.016;
  for (int i = 0; i < 360; i++){
    vec4 v = velAt(p);
    float s = length(v.xy);
    if (s < 0.003 || v.w > 0.5) break;
    vec2 vm = velAt(p - 0.5 * h * v.xy / s).xy;
    float sm = max(length(vm), 0.003);
    p -= h * vm / sm;
    tau += h / sm;
    if (p.x < uDom.x || p.y < uDom.y || p.x > uDom.z || p.y > uDom.w) break;
  }
  float eta = uDir.x * p.y - uDir.y * p.x;
  outColor = vec4(eta, tau + dot(p, uDir) - uSref, c.z, 0.0);
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
uniform sampler2D uTrace;
uniform float uSpacing, uPulse;          // label spacing (chords), pulse phase per unit travel time
void main(){
  if (hidden()) discard;
  vec4 T = texture(uTrace, vUv);
  float fade = edgeFade();
  float sr = T.z;
  vec3 fill = cpColor(1.0 - sr * sr);
  float f = T.x / uSpacing;
  float fw = fwidth(f);
  float d = abs(fract(f + 0.5) - 0.5) / max(fw, 1e-5);
  // (no lines where the label jumps within a pixel: the dividing streamline behind the trailing edge)
  float line = (1.0 - smoothstep(0.7, 1.7, d)) * (1.0 - smoothstep(0.35, 0.7, fw));
  // dashes carried along at the local speed: phase = (travel time - clock) / period
  float ph = fract(T.y * uPulse - uTime / 0.45);
  float pulse = smoothstep(0.0, 0.6, ph) * (1.0 - smoothstep(0.66, 0.74, ph));
  // dark panel, streamlines coloured by the local pressure (blue fast / low, amber slow / high), pulses
  vec3 lc = mix(vec3(1.0), fill * 1.25, 0.85);
  line *= 0.25 + 0.75 * pulse;
  vec3 col = mix(fill * 0.17, lc * (0.8 + 0.5 * pulse), line);
  float a = mix(0.9, 0.97, line) * fade;
  // the cut through the wing
  float sec = smoothstep(0.3, 0.7, T.w);
  float rim = 1.0 - smoothstep(0.0, 1.2, abs(T.w - 0.5) / max(fwidth(T.w), 1e-4));
  col = mix(col, vec3(0.05, 0.052, 0.055), sec);
  col = mix(col, vec3(0.9), rim * 0.8);
  a = max(a, max(sec, rim) * fade);
  outColor = vec4(col * uGain * a, a);
}`;

// ------------------------------------------------------------------ wake plane
const WAKE_VEL = /* glsl */ `
uniform vec3 uPC, uPE1, uPE2;            // plane centre, half-axis vectors (body axes)
${COMMON}
void main(){
  vec3 p = uPC + uPE1 * (vUv.x * 2.0 - 1.0) + uPE2 * (vUv.y * 2.0 - 1.0);
  float sd = 0.0;
  vec3 u = vortexV(p, sd, false);
  if (uGroundOn > 0.5){
    vec3 n = uGround.xyz;
    vec3 pr = p - 2.0 * (uGround.w + dot(n, p)) * n;
    float s2 = 0.0;
    vec3 ui = vortexV(pr, s2, false);
    u += ui - 2.0 * dot(ui, n) * n;
  }
  vec3 e1 = normalize(uPE1), e2 = normalize(uPE2);
  vec2 c = vec2(dot(u, e1), dot(u, e2)) / max(uU, 1.0);
  outColor = vec4(c, length(c), 0.0);
}`;

const WAKE_FS = PLANE_COMMON + /* glsl */ `
uniform sampler2D uWake, uNoise;
uniform vec2 uSize;                       // plane size (m)
void main(){
  if (hidden()) discard;
  vec4 W = texture(uWake, vUv);
  float mag = W.z;
  float rate = 12.0 * mag;                 // pattern speed ~ local crossflow speed
  // animated line-integral convolution of white noise along the crossflow
  vec2 toUv = 1.0 / uSize;
  float acc = 0.0, wsum = 0.0;
  for (int dirn = -1; dirn <= 1; dirn += 2){
    vec2 q = vUv;
    for (int i = 0; i < 18; i++){
      vec2 v = texture(uWake, q).xy;
      float s = length(v);
      if (s < 1e-4) break;
      float sl = float(i) * 0.14 * float(dirn);              // arclength (m), signed downstream
      float k = 0.6 + 0.4 * sin(6.2831853 * (sl / 2.4 - uTime * rate));
      float wgt = 1.0 - float(i) / 18.0;
      acc += texture(uNoise, q * uSize / 36.0).r * k * wgt;
      wsum += wgt;
      q += float(dirn) * 0.14 * v / s * toUv;
    }
  }
  float lic = acc / max(wsum, 1e-4);
  lic = clamp((lic - 0.25) * 2.6, 0.0, 1.0);
  // colour: downwash blue, upwash amber (vertical crossflow component), white where it swirls sideways
  float vz = W.y;
  vec3 col = vz < 0.0 ? mix(vec3(0.9), vec3(0.25, 0.55, 1.0), smoothstep(0.0, 0.08, -vz))
                      : mix(vec3(0.9), vec3(1.0, 0.62, 0.2), smoothstep(0.0, 0.08, vz));
  float a = smoothstep(0.006, 0.045, mag) * edgeFade() * (0.25 + 0.75 * lic);
  outColor = vec4(col * (0.35 + 0.9 * lic) * uGain * a, a);
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
      fst: new Float32Array(ns).fill(1), gam: new Float32Array(ns), dg: new Float32Array(ns) });
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
    }
    if (!this.haveAero) this._snap();
    this.haveAero = true;
  }

  reset() { if (this.haveAero) this._snap(); }
  setMode(m) { this.mode = m; }
  setSurfaces(s) {
    if (!s) return;
    this.surf = s;
    this.sb = clamp(((s.speedbrake || 0) - 15) / 45, 0, 1);
  }

  _snap() {
    const c = this.cur, t = this.tgt;
    c.va.copy(t.va); c.w.copy(t.w); c.up.copy(t.up); c.h = t.h; c.mach = t.mach;
    for (const k of ['gl', 'cn', 'qr', 'eq', 'fst', 'gam']) this.dyn[k].set(this.tdyn[k]);
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
      e.loft = g.userData.loft && type !== 'base' && g.userData.loft.M >= 8 ? g.userData.loft : null;
      if (e.loft) this._initLanes(e);
      const nl = e.loft ? e.loft.R * e.loft.M : 0;
      for (let k = 0; k < n; k++) a2.array[k * 4 + 1] = k < nl ? 2 : 1;      // 2: paint + flow lines
      a2.needsUpdate = true;
      this.paint.push(e);
    });
  }

  /** static data for the surface flow lines: along the rings of the wing / fin / body-flap lofts
   *  (chordwise), across the rings of the fuselage / pod lofts (lengthwise) */
  _initLanes(e) {
    const { M, R } = e.loft, P = e.P;
    e.TAU = new Float32Array(R * M); e.PSI = new Float32Array(R * M); e.DRIFT = new Float32Array(R * M);
    if (e.type === 'lift') {
      e.ringLE = new Int32Array(R); e.ringSpan = new Float32Array(R); e.ringChord = new Float32Array(R); e.ringXi0 = new Float32Array(R);
      for (let r = 0; r < R; r++) {
        let j0 = 0, span = 0;
        for (let j = 0; j < M; j++) {
          const k = r * M + j;
          if (e.XI[k] < e.XI[r * M + j0]) j0 = j;
          span += e.which === 'vt' ? -P[k * 3 + 2] : Math.abs(P[k * 3 + 1]);
        }
        e.ringLE[r] = j0; e.ringSpan[r] = span / M;
        e.ringChord[r] = e.CHD[r * M + j0]; e.ringXi0[r] = e.XI[r * M + j0];
      }
    } else {
      // arc-length fraction around each ring, ring tangent, perimeter
      e.ASF = new Float32Array(R * M); e.CIRC = new Float32Array(R * M * 3); e.PER = new Float32Array(R);
      for (let r = 0; r < R; r++) {
        const b = r * M;
        const d = (a, c) => Math.hypot(P[a * 3] - P[c * 3], P[a * 3 + 1] - P[c * 3 + 1], P[a * 3 + 2] - P[c * 3 + 2]);
        let acc = 0;
        for (let j = 0; j < M; j++) { e.ASF[b + j] = acc; acc += d(b + j, b + (j + 1) % M); }
        e.PER[r] = acc;
        for (let j = 0; j < M; j++) {
          e.ASF[b + j] /= Math.max(acc, 1e-6);
          const a = b + (j + 1) % M, c = b + (j + M - 1) % M;
          const tx = P[a * 3] - P[c * 3], ty = P[a * 3 + 1] - P[c * 3 + 1], tz = P[a * 3 + 2] - P[c * 3 + 2];
          const tl = Math.hypot(tx, ty, tz) || 1;
          e.CIRC.set([tx / tl, ty / tl, tz / tl], (b + j) * 3);
        }
      }
      e.laneN = e.type === 'pod' ? 28 : 52;
    }
  }

  /** travel time (as metres at the free-stream speed) and lane coordinate of the surface streamline
   *  through each vertex, integrated along the loft from the stagnation line with this frame's flow */
  _lanes(e, U) {
    const { M, R } = e.loft, P = e.P, V = e.V3, T = e.TAU, S = e.PSI, DR = e.DRIFT, B = e.a2.array;
    const vmin = 0.03 * U;
    const step = (k, kp, side) => {
      const dx = P[k * 3] - P[kp * 3], dy = P[k * 3 + 1] - P[kp * 3 + 1], dz = P[k * 3 + 2] - P[kp * 3 + 2];
      const ds = Math.hypot(dx, dy, dz) || 1e-6;
      const vx = 0.5 * (V[k * 3] + V[kp * 3]), vy = 0.5 * (V[k * 3 + 1] + V[kp * 3 + 1]), vz = 0.5 * (V[k * 3 + 2] + V[kp * 3 + 2]);
      const along = (vx * dx + vy * dy + vz * dz) / ds;
      const cross = vx * side[0] + vy * side[1] + vz * side[2];
      T[k] = T[kp] + ds * U / Math.max(Math.hypot(vx, vy, vz), vmin);
      return ds * clamp(cross / Math.max(Math.abs(along), vmin), -1.5, 1.5);
    };
    const sd = [0, 0, 0];
    if (e.type === 'lift') {
      const lane = 0.32;
      for (let r = 0; r < R; r++) {
        const b = r * M, j0 = e.ringLE[r], span = e.ringSpan[r];
        T[b + j0] = e.ringXi0[r] * e.ringChord[r];               // elevon / rudder lofts start at the hinge
        S[b + j0] = span / lane;
        for (const dir of [-1, 1]) {
          let drift = 0;
          for (let j = j0 + dir; j >= 0 && j < M; j += dir) {
            const k = b + j;
            sd[0] = e.STV[k * 3]; sd[1] = e.STV[k * 3 + 1]; sd[2] = e.STV[k * 3 + 2];
            drift += step(k, k - dir, sd);
            S[k] = (span - drift) / lane;
          }
        }
      }
    } else {
      // circumferential drift of the streamline as a fraction of the ring perimeter (the lanes spread
      // with the body as it widens behind the nose)
      const N = e.laneN;
      for (let j = 0; j < M; j++) { T[j] = 0; DR[j] = 0; S[j] = e.ASF[j] * N; }
      for (let r = 1; r < R; r++) {
        const per = Math.max(e.PER[r], 0.05);
        for (let j = 0; j < M; j++) {
          const k = r * M + j;
          sd[0] = e.CIRC[k * 3]; sd[1] = e.CIRC[k * 3 + 1]; sd[2] = e.CIRC[k * 3 + 2];
          DR[k] = DR[k - M] + clamp(step(k, k - M, sd) / per, -0.015, 0.015);
          S[k] = (e.ASF[k] - DR[k]) * N;
        }
      }
    }
    for (let k = 0; k < R * M; k++) { B[k * 4 + 2] = T[k]; B[k * 4 + 3] = S[k]; }
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
      if (e.loft) this._lanes(e, U);
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
    const st = new Float32Array(ns * 2 * 4);
    fg.strips.forEach((s, i) => { st.set([s[0], s[1], s[2], s[6]], i * 4); st.set([s[3], s[4], s[5], s[7]], (ns + i) * 4); });
    const nb = fg.body.length, bd = new Float32Array(nb * 2 * 4);
    fg.body.forEach((b, i) => { bd.set([b[0], b[1], b[2], b[3]], i * 4); bd.set([b[4], 0, 0, 0], (nb + i) * 4); });
    const sd = new Float32Array(fg.src.length * 4);
    fg.src.forEach((s, i) => sd.set(s, i * 4));
    this.gamData = new Float32Array(ns * 3 * 4);
    this.texGam = tex(this.gamData, ns, 3);
    const cu = {
      uStrip: { value: tex(st, ns, 2) }, uGam: { value: this.texGam }, uBody: { value: tex(bd, nb, 2) }, uSrc: { value: tex(sd, fg.src.length, 1) },
      uNS: { value: ns }, uNB: { value: nb }, uNSrc: { value: fg.src.length },
      uVa: { value: new THREE.Vector3() }, uOm: { value: new THREE.Vector3() }, uD: { value: new THREE.Vector3(-1, 0, 0) },
      uU: { value: 1 }, uMach: { value: 0 }, uGroundOn: { value: 0 }, uSB: { value: 0 }, uTime: { value: 0 },
      uGround: { value: new THREE.Vector4(0, 0, -1, 1e4) },
      uPC: { value: new THREE.Vector3() }, uPE1: { value: new THREE.Vector3() }, uPE2: { value: new THREE.Vector3() },
    };
    this.cu = cu;
    const rtOpts = { type: THREE.HalfFloatType, depthBuffer: false, stencilBuffer: false, minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter, generateMipmaps: false };
    // wake plane
    this.wakeRT = new THREE.WebGLRenderTarget(256, 192, rtOpts);
    this.wakePass = new FullscreenPass(WAKE_VEL, cu);
    const noise = new Uint8Array(256 * 256 * 4);
    for (let i = 0; i < noise.length; i++) noise[i] = Math.random() * 255;
    this.noise = new THREE.DataTexture(noise, 256, 256, THREE.RGBAFormat);
    this.noise.wrapS = this.noise.wrapT = THREE.RepeatWrapping;
    this.noise.minFilter = THREE.LinearFilter; this.noise.magFilter = THREE.LinearFilter; this.noise.needsUpdate = true;
    // section
    this.NP = 140;
    this.nodeData = new Float32Array((this.NP + 1) * 4);
    this.nodeTex = new THREE.DataTexture(this.nodeData, this.NP + 1, 1, THREE.RGBAFormat, THREE.FloatType);
    this.nodeTex.minFilter = this.nodeTex.magFilter = THREE.NearestFilter;
    this.secVelRT = new THREE.WebGLRenderTarget(448, 256, rtOpts);
    this.secTraceRT = new THREE.WebGLRenderTarget(448, 256, rtOpts);
    this.su = { uNodes: { value: this.nodeTex }, uN: { value: this.NP }, uGam: { value: 0 }, uVinf: { value: new THREE.Vector2(1, 0) },
      uDom: { value: new THREE.Vector4(-1, -1, 3, 1) }, uVel: { value: this.secVelRT.texture }, uDir: { value: new THREE.Vector2(1, 0) }, uSref: { value: 0 } };
    this.secVelPass = new FullscreenPass(SEC_VEL, this.su);
    this.secTracePass = new FullscreenPass(SEC_TRACE, this.su);
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
    this.secMesh = new THREE.Mesh(quad(), planeMat(SEC_FS, { uTrace: { value: this.secTraceRT.texture }, uSpacing: { value: 0.07 }, uPulse: { value: 1 } }));
    this.wakeMesh = new THREE.Mesh(quad(), planeMat(WAKE_FS, { uWake: { value: this.wakeRT.texture }, uNoise: { value: this.noise }, uSize: { value: new THREE.Vector2(30, 20) } }));
    for (const m of [this.secMesh, this.wakeMesh]) { m.frustumCulled = false; m.matrixAutoUpdate = false; this.scene.add(m); }
  }

  /** wing legs of each half converge onto the centroid of that half's shed vorticity (Betz) half a span
   *  behind the trailing edge; legs on the plane of symmetry, fin and body flap run straight */
  _rollUp(d, e1, e2) {
    const S = this.fg.strips, G = this.dyn.gam, D = this.gamData, ns = S.length;
    let sTE = -1e9, span = 0;
    for (const s of S) {
      sTE = Math.max(sTE, (s[0] - 0.75 * s[6]) * d.x + s[1] * d.y + s[2] * d.z, (s[3] - 0.75 * s[6]) * d.x + s[4] * d.y + s[5] * d.z);
      if (s[7] === 0) span = Math.max(span, Math.abs(s[1]), Math.abs(s[4]));
    }
    const sR = sTE + span;
    const acc = [[0, 0, 0, 0], [0, 0, 0, 0]];
    const edge = (s, which) => (which ? [s[3], s[4], s[5]] : [s[0], s[1], s[2]]);
    this._acc = acc;
    for (let i = 0; i < ns; i++) {
      const s = S[i];
      if (s[7] !== 0) continue;
      for (const which of [0, 1]) {
        const E = edge(s, which);
        if (Math.abs(E[1]) < 0.05) continue;
        const w = which ? G[i] : -G[i];
        const a = acc[E[1] > 0 ? 1 : 0];
        a[0] += w; a[1] += w * (E[0] * e1.x + E[1] * e1.y + E[2] * e1.z); a[2] += w * (E[0] * e2.x + E[1] * e2.y + E[2] * e2.z); a[3] += Math.abs(w);
      }
    }
    for (let i = 0; i < ns; i++) {
      const s = S[i];
      D[i * 4] = G[i]; D[i * 4 + 1] = this.dyn.fst[i];
      for (const which of [0, 1]) {
        const E = edge(s, which);
        const o = ((1 + which) * ns + i) * 4;
        const a = acc[E[1] > 0 ? 1 : 0];
        const sE = E[0] * d.x + E[1] * d.y + E[2] * d.z;
        if (s[7] === 0 && Math.abs(E[1]) >= 0.05 && Math.abs(a[0]) > 0.01 * a[3]) {
          const c1 = clamp(a[1] / a[0], -span, span), c2 = a[2] / a[0];
          D[o] = c1 * e1.x + c2 * e2.x + sR * d.x; D[o + 1] = c1 * e1.y + c2 * e2.y + sR * d.y; D[o + 2] = c1 * e1.z + c2 * e2.z + sR * d.z;
          D[o + 3] = 0.05 * 2 * span;
        } else {
          D[o] = E[0] + d.x * (sR - sE); D[o + 1] = E[1] + d.y * (sR - sE); D[o + 2] = E[2] + d.z * (sR - sE);
          D[o + 3] = Math.max(0.22, 0.1 * s[6]);
        }
      }
    }
    this.texGam.needsUpdate = true;
    const [aL, aR] = acc;
    this._rollInfo = Math.abs(aL[0]) > 1 && Math.abs(aR[0]) > 1
      ? { b: Math.abs(aR[1] / aR[0] - aL[1] / aL[0]), g: 0.5 * (Math.abs(aL[0]) + Math.abs(aR[0])) } : null;
    return { sTE, span };
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

  /** Hess-Smith: N source panels + uniform vortex, Kutta condition; returns unit solutions for
   *  free streams (1,0) and (0,1) in the solver frame (x aft, y up, lengths / chord) */
  static hessSmith(xs, ys) {
    const N = xs.length - 1;
    const th = new Float64Array(N), cx = new Float64Array(N), cy = new Float64Array(N);
    for (let j = 0; j < N; j++) { th[j] = Math.atan2(ys[j + 1] - ys[j], xs[j + 1] - xs[j]); cx[j] = 0.5 * (xs[j] + xs[j + 1]); cy[j] = 0.5 * (ys[j] + ys[j + 1]); }
    const A = new Float64Array(N * N), B = new Float64Array(N * N);
    for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
      let beta, L;
      if (i === j) { beta = Math.PI; L = 0; }
      else {
        const r1x = xs[j] - cx[i], r1y = ys[j] - cy[i], r2x = xs[j + 1] - cx[i], r2y = ys[j + 1] - cy[i];
        beta = Math.atan2(r1x * r2y - r1y * r2x, r1x * r2x + r1y * r2y);
        L = 0.5 * Math.log((r2x * r2x + r2y * r2y) / (r1x * r1x + r1y * r1y));      // ln(r2 / r1)
      }
      const sn = Math.sin(th[i] - th[j]), cs = Math.cos(th[i] - th[j]);
      A[i * N + j] = (sn * L + cs * beta) / (2 * Math.PI);
      B[i * N + j] = (sn * beta - cs * L) / (2 * Math.PI);
    }
    const n1 = N + 1, Mx = new Float64Array(n1 * n1), rhs = [new Float64Array(n1), new Float64Array(n1)];
    for (let i = 0; i < N; i++) {
      let sb = 0;
      for (let j = 0; j < N; j++) { Mx[i * n1 + j] = A[i * N + j]; sb += B[i * N + j]; }
      Mx[i * n1 + N] = -sb;
      rhs[0][i] = Math.sin(th[i]);              // -(V . n), n = (-sin, cos), V = (1, 0)
      rhs[1][i] = -Math.cos(th[i]);             // V = (0, 1)
    }
    let sa = 0;
    for (let j = 0; j < N; j++) { Mx[N * n1 + j] = B[j] + B[(N - 1) * N + j]; sa += A[j] + A[(N - 1) * N + j]; }
    Mx[N * n1 + N] = sa;
    rhs[0][N] = -(Math.cos(th[0]) + Math.cos(th[N - 1]));
    rhs[1][N] = -(Math.sin(th[0]) + Math.sin(th[N - 1]));
    // Gaussian elimination with partial pivoting, two right-hand sides
    for (let c = 0; c < n1; c++) {
      let p = c;
      for (let r = c + 1; r < n1; r++) if (Math.abs(Mx[r * n1 + c]) > Math.abs(Mx[p * n1 + c])) p = r;
      if (p !== c) {
        for (let k = 0; k < n1; k++) { const t = Mx[c * n1 + k]; Mx[c * n1 + k] = Mx[p * n1 + k]; Mx[p * n1 + k] = t; }
        for (const R of rhs) { const t = R[c]; R[c] = R[p]; R[p] = t; }
      }
      const piv = Mx[c * n1 + c];
      for (let r = c + 1; r < n1; r++) {
        const f = Mx[r * n1 + c] / piv;
        if (f === 0) continue;
        for (let k = c; k < n1; k++) Mx[r * n1 + k] -= f * Mx[c * n1 + k];
        for (const R of rhs) R[r] -= f * R[c];
      }
    }
    for (const R of rhs) for (let r = n1 - 1; r >= 0; r--) {
      let s = R[r];
      for (let k = r + 1; k < n1; k++) s -= Mx[r * n1 + k] * R[k];
      R[r] = s / Mx[r * n1 + r];
    }
    let per = 0;
    for (let j = 0; j < N; j++) per += Math.hypot(xs[j + 1] - xs[j], ys[j + 1] - ys[j]);
    return { q: rhs, per, N, A, B, th, cx };
  }

  _solveSection(camera) {
    const sh = this.shuttle, c = this.cur, D = this.dyn;
    // the wing facing the camera (hysteresis around the plane of symmetry)
    const cl = sh.model.worldToLocal(this._wp.copy(camera.position));
    if (Math.abs(cl.z) > 3) this.side = cl.z > 0 ? 1 : -1;
    const side = this.side;
    const ay = clamp(this.station * this.span, 2.6, this.span - 0.4);
    const inner = ay < 7.25;
    const key = `elevon_${side > 0 ? 'r' : 'l'}${inner ? 'i' : 'o'}`;
    const defl = this.surf[key] || 0;
    const { pts, s } = this._sectionContour(ay, defl);
    const ch = s.chord;
    // solver frame: x aft from the leading edge, y up from the chord line, in chords
    const xs = pts.map((p) => (s.xle - p[0]) / ch), ys = pts.map((p) => (p[1] - s.yc) / ch);
    const key2 = `${ay.toFixed(2)}|${defl.toFixed(1)}`;
    if (this._hsKey !== key2) { this._hs = AirflowViz.hessSmith(xs, ys); this._hsKey = key2; }
    const hs = this._hs;
    // inflow at the section's quarter chord (body axes): free stream + rotation
    const p = new THREE.Vector3(s.xle - 0.25 * ch, side * ay, -s.yc);
    const va = c.va, om = c.w;
    const u = new THREE.Vector3(-(va.x + om.y * p.z - om.z * p.y), -(va.y + om.z * p.x - om.x * p.z), -(va.z + om.x * p.y - om.y * p.x));
    const aft = -u.x, up = -u.z, U2d = Math.hypot(aft, up);
    const aGeo = Math.atan2(up, aft);
    // the model's strip circulation at this station, per unit span of the section (Kutta-Joukowski)
    const [i0, i1, w] = this._interpList(this.lists[side > 0 ? 0 : 1], ay);
    const gl = D.gl[i0] * (1 - w) + D.gl[i1] * w;
    const sI = this.st[w < 0.5 ? i0 : i1];
    const l = sI.qb.clone().sub(sI.qa);
    const g2 = gl * u.clone().cross(l).length() / Math.max(Math.abs(l.y) * U2d, 1e-6);
    // effective angle: the free-stream direction whose Kutta circulation equals the model's
    const gx = hs.q[0][hs.N], gy = hs.q[1][hs.N];
    const k = g2 / (Math.max(U2d, 1e-3) * ch * hs.per);
    const R = Math.hypot(gx, gy), ph = Math.atan2(gy, gx);
    const ac = Math.acos(clamp(k / Math.max(R, 1e-9), -1, 1));
    const cand = [ph + ac, ph - ac];
    const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    const aEff = Math.abs(wrap(cand[0] - aGeo)) < Math.abs(wrap(cand[1] - aGeo)) ? cand[0] : cand[1];
    const ca = Math.cos(aEff), sa = Math.sin(aEff);
    for (let j = 0; j <= hs.N; j++) {
      const qj = j < hs.N ? hs.q[0][j] * ca + hs.q[1][j] * sa : 0;
      this.nodeData.set([xs[j], ys[j], qj, 0], j * 4);
    }
    this.nodeTex.needsUpdate = true;
    // surface Cp along the section (tangential speed at the panel midpoints)
    const gm = hs.q[0][hs.N] * ca + hs.q[1][hs.N] * sa;
    this.secCp = { x: [], cp: [], n: hs.N };
    for (let i = 0; i < hs.N; i++) {
      let vt = Math.cos(aEff - hs.th[i]);
      for (let j = 0; j < hs.N; j++) vt += hs.B[i * hs.N + j] * (hs.q[0][j] * ca + hs.q[1][j] * sa) + hs.A[i * hs.N + j] * gm;
      this.secCp.x.push(hs.cx[i]); this.secCp.cp.push(1 - vt * vt);
    }
    this.secCl = 2 * gm * hs.per;
    const su = this.su;
    su.uGam.value = hs.q[0][hs.N] * ca + hs.q[1][hs.N] * sa;
    su.uVinf.value.set(ca, sa);
    // domain (chords): 1.1 ahead, 2.2 behind the leading edge... trailing edge, +-1.15 across
    const dom = su.uDom.value.set(-0.75, -0.7, 2.1, 0.7);
    su.uDir.value.set(ca, sa);
    su.uSref.value = Math.min(dom.x * ca + dom.y * sa, dom.x * ca + dom.w * sa, dom.z * ca + dom.y * sa, dom.z * ca + dom.w * sa);
    // plane quad in the model frame (local z = body y)
    const pos = this.secMesh.geometry.attributes.position;
    const Z = side * ay;
    const corner = (xn, yn) => [s.xle - xn * ch, s.yc + yn * ch, Z];
    pos.array.set([...corner(dom.x, dom.y), ...corner(dom.z, dom.y), ...corner(dom.z, dom.w), ...corner(dom.x, dom.w)]);
    pos.needsUpdate = true;
    this.secMesh.material.uniforms.uSpacing.value = 0.045;
    // pulses: one every 0.4 s of display time; travel time is in chords / U
    this.secMesh.material.uniforms.uPulse.value = ch / (this.vis * 0.45);
    this.secInfo = { y: side * ay, ch, aGeo, aEff, defl, fst: D.fst[i0] * (1 - w) + D.fst[i1] * w };
    this.clipLocal = { side, ay };
  }

  _wakePlane() {
    const c = this.cur, U = c.va.length();
    const d = c.va.clone().multiplyScalar(-1 / U);
    const e1 = new THREE.Vector3(0, 1, 0).addScaledVector(d, -d.y);
    if (e1.lengthSq() < 0.05) e1.set(0, 0, 1).addScaledVector(d, -d.z);
    e1.normalize();
    const e2 = new THREE.Vector3().crossVectors(d, e1);
    const { sTE, span } = this._rollUp(d, e1, e2);
    this.wakeInfo = this._rollInfo;
    // centre: half a span behind the trailing edge, at the wing's height
    let zc = 0, nz = 0;
    for (const s of this.st) if (s.surf === SURF_WING) { zc += s.mid.dot(e2); nz++; }
    zc /= Math.max(nz, 1);
    const sT = sTE + 0.5 * span;
    const W = span + 7, H = 0.75 * span + 3;
    const cu = this.cu;
    cu.uPC.value.copy(d).multiplyScalar(sT).addScaledVector(e2, zc);
    cu.uPE1.value.copy(e1).multiplyScalar(W);
    cu.uPE2.value.copy(e2).multiplyScalar(H);
    this.wakeMesh.material.uniforms.uSize.value.set(2 * W, 2 * H);
    const pos = this.wakeMesh.geometry.attributes.position;
    const cP = cu.uPC.value, A = cu.uPE1.value, B = cu.uPE2.value;
    const L = (a, b) => { const p = cP.clone().addScaledVector(A, a).addScaledVector(B, b); return [p.x, -p.z, p.y]; };    // FRD -> local
    pos.array.set([...L(-1, -1), ...L(1, -1), ...L(1, 1), ...L(-1, 1)]);
    pos.needsUpdate = true;
    return { d };
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
    for (const key of ['gl', 'cn', 'qr', 'eq', 'fst', 'gam']) { const a = D[key], b = T[key]; for (let i = 0; i < a.length; i++) a[i] += (b[i] - a[i]) * k; }
    this.time += dt;
    PAINT.uPaintTime.value = this.time;
    PAINT.uPaintVis.value = this.vis;
    PAINT.uPaintEmit.value = 0.06 * sunIntensity + 0.1;
    this._updatePaint();
    const U = c.va.length();
    const cu = this.cu;
    cu.uVa.value.copy(c.va); cu.uOm.value.copy(c.w); cu.uD.value.copy(c.va).multiplyScalar(-1 / U);
    cu.uU.value = U; cu.uMach.value = c.mach; cu.uSB.value = this.sb;
    cu.uGround.value.set(c.up.x, c.up.y, c.up.z, c.h); cu.uGroundOn.value = c.h < 75 ? 1 : 0;
    const prev = renderer.getRenderTarget();
    if (this.mode === 'section') {
      this._solveSection(camera);
      this.secVelPass.render(renderer, this.secVelRT);
      this.secTracePass.render(renderer, this.secTraceRT);
      const s = this.secInfo, R2D = 180 / Math.PI;
      this.info = `y = ${s.y.toFixed(1)} m 剖面，弦长 ${s.ch.toFixed(1)} m${Math.abs(s.defl) > 0.5 ? `，升降副翼 ${s.defl > 0 ? '下' : '上'}偏 ${Math.abs(s.defl).toFixed(1)}°` : ''}。` +
        `几何迎角 ${(s.aGeo * R2D).toFixed(1)}°，有效迎角 ${(s.aEff * R2D).toFixed(1)}°，下洗 ${((s.aGeo - s.aEff) * R2D).toFixed(1)}°，Cl ${this.secCl.toFixed(2)}` +
        (s.fst < 0.9 ? `。这里已部分失速（附着 ${(s.fst * 100).toFixed(0)}%）` : '') +
        (c.mach > 0.95 ? '。剖面按不可压势流算，超声速时只能参考' : '');
    } else if (this.mode === 'wake') {
      this._wakePlane();
      this.wakePass.render(renderer, this.wakeRT);
      const w = this.wakeInfo;
      this.info = w ? `尾涡间距 ${w.b.toFixed(1)} m，单侧环量 ${w.g.toFixed(0)} m²/s` : '';
    } else {
      const L = this.lift, Dr = this.drag;
      this.info = Number.isFinite(L) ? `升力 ${(L / 1e3).toFixed(0)} kN　阻力 ${(Dr / 1e3).toFixed(0)} kN　升阻比 ${(L / Math.max(Dr, 1)).toFixed(2)}　机翼最低 Cp ${this.cpMin.toFixed(2)}` : '';
    }
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
    if (this.mode === 'section' && this.secCp) {
      // -Cp up, x/c to the right
      const { x, cp, n } = this.secCp;
      let lo = -1, hi = 1;
      for (const v of cp) { lo = Math.min(lo, -v); hi = Math.max(hi, -v); }
      lo = Math.floor(lo * 2) / 2; hi = Math.ceil(Math.min(hi, 6) * 2) / 2;
      const X = (v) => pad.l + clamp(v, 0, 1) * pw, Y = (v) => pad.t + (hi - clamp(v, lo, hi)) / (hi - lo) * ph;
      line([[pad.l, Y(0)], [pad.l + pw, Y(0)]], 'rgba(255,255,255,0.25)', 1);
      g.fillText(`${hi}`, 4, pad.t + 8); g.fillText(`${lo}`, 4, pad.t + ph); g.fillText('−Cp', 4, Y(0) + 3);
      g.fillText('前缘', pad.l, H - 3); g.fillText('后缘', pad.l + pw - 24, H - 3);
      const half = n >> 1, lower = [], upper = [];
      for (let i = 0; i < n; i++) (i < half ? lower : upper).push([X(x[i]), Y(-cp[i])]);
      line(lower, 'rgb(255,160,60)'); line(upper, 'rgb(90,170,255)');
      g.fillStyle = 'rgb(90,170,255)'; g.fillText('上表面', pad.l + pw - 92, pad.t + 9);
      g.fillStyle = 'rgb(255,160,60)'; g.fillText('下表面', pad.l + pw - 46, pad.t + 9);
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
    const mesh = this.mode === 'section' ? this.secMesh : this.wakeMesh;
    this.secMesh.visible = mesh === this.secMesh; this.wakeMesh.visible = mesh === this.wakeMesh;
    const u = mesh.material.uniforms;
    u.uDepth.value = depthTexture;
    u.uRes.value.set(this.layer.width, this.layer.height);
    u.uLogFar.value = Math.log2(camera.far + 1.0);
    u.uGain.value = 0.95 * this.gain / Math.max(exposure, 1e-3);
    u.uTime.value = this.time;
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
