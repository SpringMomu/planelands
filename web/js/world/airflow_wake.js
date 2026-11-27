// Free vortex wake of the lifting strips (sim/aero.py), time-stepped on the GPU.
//
// Each strip of the aero model is a vortex ring on its planform (bound vortex on the quarter-chord
// line, legs to the trailing edge) carrying the strip's circulation from the latest aero message.
// Every ~1 m of travel a new row of nodes is put down along the trailing edges; the ring between it
// and the previous row gets the circulation the strip has at that moment (Kelvin: the change of the
// bound circulation is left behind as a spanwise vortex, differences between neighbouring strips as
// trailing vortices).  Every frame every free node moves with the local air velocity relative to the
// vehicle: free stream + rotation + Biot-Savart of all bound and wake segments (cores that grow with
// age).  So the sheet rolls up into the tip vortices by itself, and what the vehicle did a second ago
// is still visible downstream.  Time runs in slow motion: the free stream at the display speed, all
// induced velocities and the rotation scaled with it.
//
// Body axes (x fwd, y right, z down) about the CG; positions in a J x R float texture (columns along
// the trailing edges, rows = shedding times, ring buffer with `head` the newest row).
import * as THREE from 'three';
import { FullscreenPass } from '../render/pass.js';

const SURF_WING = 0, SURF_BF = 1, SURF_VT = 2;

const SEG = /* glsl */ `
const float PI4 = 12.566370614;
// finite vortex segment a -> b, unit circulation (Biot-Savart, core-regularised)
vec3 segV(vec3 p, vec3 a, vec3 b, float rc2){
  vec3 r1 = p - a, r2 = p - b, r0 = b - a;
  vec3 c = cross(r1, r2);
  float den = dot(c, c) + rc2 * dot(r0, r0);
  float l1 = max(length(r1), 1e-4), l2 = max(length(r2), 1e-4);
  return c * (dot(r0, r1 / l1 - r2 / l2) / (PI4 * den));
}`;

const STEP = /* glsl */ `
uniform sampler2D uPos, uTE, uRing, uBound;
uniform int uJ, uR, uHead, uNS;
uniform vec3 uVa, uOm, uDir;
uniform float uDt, uK, uDs, uInit, uVis;
${SEG}
float ringW(int k){ return 1.0 - smoothstep(0.65 * float(uR - 1), float(uR - 1), float(k)); }
void main(){
  ivec2 ij = ivec2(gl_FragCoord.xy);
  int j = ij.x, r = ij.y;
  int k = (r - uHead + uR) % uR;
  vec3 te = texelFetch(uTE, ivec2(j, 0), 0).xyz;
  if (uInit > 0.5){ outColor = vec4(te + uDir * (float(k) * uDs), float(k) * uDs / uVis); return; }
  if (k == 0){ outColor = vec4(te, 0.0); return; }
  vec4 P = texelFetch(uPos, ij, 0);
  vec3 p = P.xyz;
  vec3 u = -(uVa + cross(uOm, p));
  // strips: bound rings  (qa, qb, trailing edge at qb, at qa)
  for (int i = 0; i < uNS; i++){
    vec4 A = texelFetch(uBound, ivec2(i, 0), 0);
    if (abs(A.w) < 1e-3) continue;
    vec3 B = texelFetch(uBound, ivec2(i, 1), 0).xyz, C = texelFetch(uBound, ivec2(i, 2), 0).xyz, D = texelFetch(uBound, ivec2(i, 3), 0).xyz;
    const float rb = 0.36;
    u += A.w * (segV(p, A.xyz, B, rb) + segV(p, B, C, rb) + segV(p, C, D, rb) + segV(p, D, A.xyz, rb));
  }
  // wake rings
  for (int kk = 0; kk < uR - 1; kk++){
    int r0 = (uHead + kk) % uR, r1 = (uHead + kk + 1) % uR;
    vec4 a0 = texelFetch(uPos, ivec2(0, r0), 0), b0 = texelFetch(uPos, ivec2(0, r1), 0);
    float rc2 = 0.16 + 0.12 * a0.w;                    // core radius^2 (m^2) grows with age (display s)
    float w = ringW(kk);
    for (int jj = 0; jj < uJ - 1; jj++){
      vec4 a1 = texelFetch(uPos, ivec2(jj + 1, r0), 0), b1 = texelFetch(uPos, ivec2(jj + 1, r1), 0);
      vec2 G = texelFetch(uRing, ivec2(jj, r0), 0).xy;
      if (G.y > 0.5 && abs(G.x) > 1e-3)
        u += (G.x * w) * (segV(p, a0.xyz, a1.xyz, rc2) + segV(p, a1.xyz, b1.xyz, rc2) + segV(p, b1.xyz, b0.xyz, rc2) + segV(p, b0.xyz, a0.xyz, rc2));
      a0 = a1; b0 = b1;
    }
  }
  outColor = vec4(p + u * (uK * uDt), P.w + uDt);
}`;

const SHEET_VS = /* glsl */ `
precision highp float;
precision highp sampler2D;
in vec3 position;                 // (column, row from the trailing edge, 0)
uniform mat4 modelMatrix, viewMatrix, projectionMatrix;
uniform sampler2D uPos, uRing;
uniform int uJ, uR, uHead;
uniform float uGmax;
out vec3 vN, vW;
out vec2 vJK;
out float vViewZ, vG, vAge, vS;
vec3 nodeB(int j, int k){ j = clamp(j, 0, uJ - 1); k = clamp(k, 0, uR - 1); return texelFetch(uPos, ivec2(j, (uHead + k) % uR), 0).xyz; }
vec3 toLocal(vec3 b){ return vec3(b.x, -b.z, b.y); }
void main(){
  int j = int(position.x + 0.5), k = int(position.y + 0.5);
  vec3 p = nodeB(j, k);
  vec3 dj = nodeB(j + 1, k) - nodeB(j - 1, k), dk = nodeB(j, k + 1) - nodeB(j, k - 1);
  vN = normalize((modelMatrix * vec4(toLocal(cross(dj, dk)), 0.0)).xyz + 1e-6);
  // trailing vorticity at this node: difference of the neighbouring rings
  int r = (uHead + k) % uR;
  float gl = j > 0 ? texelFetch(uRing, ivec2(j - 1, r), 0).x : 0.0;
  float gr = j < uJ - 1 ? texelFetch(uRing, ivec2(j, r), 0).x : 0.0;
  vG = (gl - gr) / max(uGmax, 1.0);
  vS = max(abs(gl), abs(gr)) / max(uGmax, 1.0);        // how much circulation the sheet carries here
  vAge = float(k) / float(uR - 1);
  vJK = position.xy;
  vec4 wp = modelMatrix * vec4(toLocal(p), 1.0);
  vW = wp.xyz;
  vec4 mv = viewMatrix * wp;
  vViewZ = -mv.z;
  gl_Position = projectionMatrix * mv;
}`;

const SHEET_FS = /* glsl */ `
precision highp float;
uniform sampler2D uDepth;
uniform vec2 uRes;
uniform float uLogFar, uGain, uFrac;
uniform vec3 uCam;
in vec3 vN, vW;
in vec2 vJK;
in float vViewZ, vG, vAge, vS;
out vec4 outColor;
void main(){
  float d = texture(uDepth, gl_FragCoord.xy / uRes).r;
  float sw = d >= 0.999999 ? 1e9 : exp2(d * uLogFar) - 1.0;
  if (vViewZ > sw + 0.03 + 0.002 * vViewZ) discard;
  vec3 n = normalize(vN), v = normalize(uCam - vW);
  float facing = abs(dot(n, v));
  // lines along the columns (vortex lines leaving the trailing edge) and every third row (time lines)
  float fj = vJK.x, fk = (vJK.y - uFrac) / 3.0;
  float lj = 1.0 - smoothstep(0.6, 1.5, abs(fract(fj + 0.5) - 0.5) / max(fwidth(fj), 1e-4));
  float lk = 1.0 - smoothstep(0.6, 1.5, abs(fract(fk + 0.5) - 0.5) / max(fwidth(fk), 1e-4));
  // the sheet shows as strongly as the circulation it carries (a fin in symmetric flight: nothing)
  float carry = smoothstep(0.02, 0.3, vS), trail = smoothstep(0.0, 0.08, abs(vG));
  float line = max(lj * 0.75 * max(carry, trail), lk * 0.45 * carry);
  // colour by the sense of the trailing vorticity (left / right tip vortex), white where weak
  float g = clamp(vG * 6.0, -1.0, 1.0);
  vec3 c = g > 0.0 ? mix(vec3(0.92), vec3(1.0, 0.6, 0.22), g) : mix(vec3(0.92), vec3(0.3, 0.62, 1.0), -g);
  float fade = smoothstep(0.0, 0.05, vAge) * (1.0 - smoothstep(0.7, 1.0, vAge));
  float sheet = (0.06 + 0.22 * (1.0 - facing)) * carry;   // the rolled-up folds read brighter
  float a = clamp(sheet + line * 0.8, 0.0, 1.0) * fade;
  vec3 col = c * (0.55 + 0.45 * facing) * (0.6 + 0.6 * line);
  outColor = vec4(col * uGain * a, a);
}`;

export class FreeWake {
  /** fg: flow geometry (strips); lists: span-sorted strip lists of AirflowViz; shuttle: for the planform */
  constructor(fg, lists, shuttle, { rows = 40, ds = 1.3 } = {}) {
    this.R = rows; this.ds = ds;
    const S = fg.strips, ns = S.length;
    this.ns = ns;
    const V = (a) => new THREE.Vector3(a[0], a[1], a[2]);
    // columns: wing left tip -> right tip, fin root -> tip, body flap left -> right
    const groups = [
      { strips: lists[1].slice().reverse().concat(lists[0]).map((e) => e.i), key: (p) => p.y, kind: SURF_WING },
      { strips: lists[2].map((e) => e.i), key: (p) => -p.z, kind: SURF_VT },
      { strips: lists[3].map((e) => e.i), key: (p) => p.y, kind: SURF_BF },
    ];
    const te = [], ring = [];                     // node trailing-edge positions; per column: strip, sign, shown
    this.colStrip = []; this.colSign = []; this.colShow = [];
    const teX = (kind, p, i) => {
      if (kind === SURF_WING) { const st = shuttle._wingStation(Math.max(Math.abs(p.y), 2.6)); return st.xle - st.chord; }
      if (kind === SURF_VT) { const st = shuttle._finStation(-p.z); return st.xle - st.chord; }
      return p.x - 0.75 * S[i][6];
    };
    this.bound = new Float32Array(ns * 4 * 4);
    for (const g of groups) {
      const j0 = te.length;
      g.strips.forEach((i, q) => {
        const s = S[i], qa = V(s), qb = V(s.slice(3));
        const lo = g.key(qa) <= g.key(qb) ? qa : qb, hi = lo === qa ? qb : qa;
        if (q === 0) te.push(new THREE.Vector3(teX(g.kind, lo, i), lo.y, lo.z));
        te.push(new THREE.Vector3(teX(g.kind, hi, i), hi.y, hi.z));
        this.colStrip.push(i); this.colSign.push(lo === qa ? 1 : -1);
        this.colShow.push(g.kind !== SURF_WING || Math.max(Math.abs(qa.y), Math.abs(qb.y)) > 2.7 ? 1 : 0);
        // bound ring: qa, qb, trailing edge behind qb, behind qa
        const ta = te[te.length - (lo === qa ? 2 : 1)], tb = te[te.length - (lo === qa ? 1 : 2)];
        this.bound.set([qa.x, qa.y, qa.z, 0], (0 * ns + i) * 4);
        this.bound.set([qb.x, qb.y, qb.z, 0], (1 * ns + i) * 4);
        this.bound.set([tb.x, tb.y, tb.z, 0], (2 * ns + i) * 4);
        this.bound.set([ta.x, ta.y, ta.z, 0], (3 * ns + i) * 4);
      });
      this.colStrip.push(-1); this.colSign.push(0); this.colShow.push(0);   // no ring across groups
      g.j0 = j0;
    }
    this.colStrip.pop(); this.colSign.pop(); this.colShow.pop();
    const J = this.J = te.length;
    const tex = (data, w, h) => {
      const t = new THREE.DataTexture(data, w, h, THREE.RGBAFormat, THREE.FloatType);
      t.minFilter = t.magFilter = THREE.NearestFilter; t.needsUpdate = true; return t;
    };
    this.teData = new Float32Array(J * 4);
    te.forEach((p, j) => this.teData.set([p.x, p.y, p.z, 0], j * 4));
    this.teTex = tex(this.teData, J, 1);
    this.boundTex = tex(this.bound, ns, 4);
    this.ringData = new Float32Array(J * this.R * 4);
    this.ringTex = tex(this.ringData, J, this.R);
    const rt = () => new THREE.WebGLRenderTarget(J, this.R, { type: THREE.FloatType, format: THREE.RGBAFormat, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter, depthBuffer: false, stencilBuffer: false, generateMipmaps: false });
    this.pos = [rt(), rt()];
    this.pi = 0;
    this.head = 0;
    this.u = {
      uPos: { value: null }, uTE: { value: this.teTex }, uRing: { value: this.ringTex }, uBound: { value: this.boundTex },
      uJ: { value: J }, uR: { value: this.R }, uHead: { value: 0 }, uNS: { value: ns },
      uVa: { value: new THREE.Vector3() }, uOm: { value: new THREE.Vector3() }, uDir: { value: new THREE.Vector3(-1, 0, 0) },
      uDt: { value: 0 }, uK: { value: 0.1 }, uDs: { value: ds }, uInit: { value: 1 }, uVis: { value: 15 },
    };
    this.pass = new FullscreenPass(STEP, this.u);
    // the sheet: one vertex per node, quads over the rings that are drawn
    const pos = new Float32Array(J * this.R * 3), idx = [];
    for (let k = 0; k < this.R; k++) for (let j = 0; j < J; j++) pos.set([j, k, 0], (k * J + j) * 3);
    for (let k = 0; k < this.R - 1; k++) for (let j = 0; j < J - 1; j++) {
      if (!this.colShow[j]) continue;
      const a = k * J + j, b = a + 1, c = a + J + 1, d = a + J;
      idx.push(a, b, c, a, c, d);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setIndex(idx);
    this.mat = new THREE.RawShaderMaterial({
      vertexShader: SHEET_VS, fragmentShader: SHEET_FS, glslVersion: THREE.GLSL3,
      uniforms: {
        uPos: { value: null }, uRing: { value: this.ringTex }, uJ: { value: J }, uR: { value: this.R }, uHead: { value: 0 }, uGmax: { value: 1 },
        uDepth: { value: null }, uRes: { value: new THREE.Vector2(1, 1) }, uLogFar: { value: 1 }, uGain: { value: 1 }, uFrac: { value: 0 },
        uCam: { value: new THREE.Vector3() },
      },
      transparent: true, depthTest: false, depthWrite: false, side: THREE.DoubleSide,
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation,
      blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor, blendSrcAlpha: THREE.OneFactor, blendDstAlpha: THREE.OneMinusSrcAlphaFactor,
    });
    this.mesh = new THREE.Mesh(geo, this.mat);
    this.mesh.frustumCulled = false; this.mesh.matrixAutoUpdate = false;
    this.acc = 0; this.nEmit = 0;
    this.needInit = true;
  }

  reset() { this.needInit = true; }

  /** gam: circulation per strip (m^2/s, smoothed); va / om: body velocity rel. air / rotation (body
   *  axes); vis: display speed of the free stream; dt: frame time (s) */
  update(renderer, dt, gam, va, om, vis) {
    const U = Math.max(va.length(), 1), K = vis / U, R = this.R, J = this.J, ns = this.ns;
    for (let i = 0; i < ns; i++) this.bound[i * 4 + 3] = gam[i];
    this.boundTex.needsUpdate = true;
    let gmax = 1;
    for (let i = 0; i < ns; i++) gmax = Math.max(gmax, Math.abs(gam[i]));
    const writeRow = (r) => {
      for (let j = 0; j < J - 1; j++) {
        const i = this.colStrip[j], o = (r * J + j) * 4;
        this.ringData[o] = i >= 0 ? this.colSign[j] * gam[i] : 0;
        this.ringData[o + 1] = i >= 0 ? 1 : 0;
      }
    };
    const u = this.u;
    u.uVa.value.copy(va); u.uOm.value.copy(om); u.uK.value = K; u.uVis.value = vis;
    u.uDir.value.copy(va).multiplyScalar(-1 / U);
    if (this.needInit) {
      for (let r = 0; r < R; r++) writeRow(r);
      this.head = 0; this.acc = 0;
      u.uInit.value = 1;
    } else {
      u.uInit.value = 0;
      // a new row every ds of travel at the display speed (at most one per frame)
      this.acc += dt * vis;
      if (this.acc >= this.ds) {
        this.acc = Math.min(this.acc - this.ds, this.ds);
        this.head = (this.head + R - 1) % R;
        this.nEmit++;
      }
      writeRow(this.head);                       // the newest ring follows the strips until it is left behind
    }
    this.ringTex.needsUpdate = true;
    u.uHead.value = this.head; u.uDt.value = Math.min(dt, 0.05);
    u.uPos.value = this.pos[this.pi].texture;
    this.pi ^= 1;
    this.pass.render(renderer, this.pos[this.pi]);
    this.needInit = false;
    const m = this.mat.uniforms;
    m.uPos.value = this.pos[this.pi].texture; m.uHead.value = this.head; m.uGmax.value = gmax;
    m.uFrac.value = this.nEmit % 3;
  }
}
