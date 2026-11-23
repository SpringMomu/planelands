// Orbiter drag chute (visual): 12.2 m (40 ft) conical ribbon canopy with 32 gores, suspension
// lines to a confluence point, single riser back to the compartment at the base of the fin, and a
// 2.7 m pilot chute on a bridle behind the vent.  World-space object driven by the simulator's
// inflated-area fraction (pilot -> reefed -> disreefed); it trails along the air-relative velocity
// with a little lag, gravity droop, coning and canopy breathing, and when jettisoned it is released,
// collapses and drops onto the runway behind the orbiter.
import * as THREE from 'three';
import { patchStandard } from './materials.js';

const NG = 32;            // gores / suspension lines
const PER = 4;            // canopy columns per gore
const NA = NG * PER;      // columns around
const NS = 26;            // rows skirt -> vent
const NPL = 12;           // pilot-chute lines
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const ss = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const lerp = (a, b, t) => a + (b - a) * t;

/** simulator inflated-area fraction -> visual inflation k (0 streamer, 0.55 reefed, 1 full) */
function inflationFromState(s) {
  if (s <= 0.02) return 0.1 * s / 0.02;
  if (s <= 0.4) return 0.1 + 0.45 * (s - 0.02) / 0.38;
  return Math.min(1, 0.55 + 0.45 * (s - 0.4) / 0.6);
}

export class DragChute {
  constructor(geom, U) {
    const dc = geom.drag_chute;
    this.D0 = dc.diameter;
    this.Lr = dc.riser_length;
    this.Ls = dc.line_length || dc.diameter;
    this.Lb = dc.bridle_length || 8.0;
    this.Rp = 0.36 * (dc.pilot_diameter || 2.74);
    this.CdS = (dc.CD || 0.6) * Math.PI * (dc.diameter / 2) ** 2;
    this.group = new THREE.Group();
    this.group.name = 'drag-chute';
    this.group.visible = false;
    this.group.frustumCulled = false;

    // ---------------- canopy (dynamic grid)
    const nv = (NA + 1) * (NS + 1);
    this.cpos = new Float32Array(nv * 3);
    this.cnrm = new Float32Array(nv * 3);
    const uv = new Float32Array(nv * 2);
    for (let j = 0; j <= NS; j++) for (let i = 0; i <= NA; i++) { const v = j * (NA + 1) + i; uv[v * 2] = i / NA; uv[v * 2 + 1] = j / NS; }
    const idx = [];
    for (let j = 0; j < NS; j++) for (let i = 0; i < NA; i++) {
      const a = j * (NA + 1) + i, b = a + 1, c = a + NA + 1, d = c + 1;
      idx.push(a, b, c, b, d, c);
    }
    const cg = new THREE.BufferGeometry();
    cg.setAttribute('position', new THREE.BufferAttribute(this.cpos, 3).setUsage(THREE.DynamicDrawUsage));
    cg.setAttribute('normal', new THREE.BufferAttribute(this.cnrm, 3).setUsage(THREE.DynamicDrawUsage));
    cg.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    cg.setIndex(idx);
    this.canopyGeo = cg;

    this.u = { uSunView: { value: new THREE.Vector3(0, 1, 0) }, uSunRad: { value: new THREE.Color(1, 1, 1) } };
    const mkMat = (ribbons, key) => {
      const m = new THREE.MeshStandardMaterial({ color: 0xf2ede2, roughness: 0.78, metalness: 0, side: THREE.DoubleSide });
      patchStandard(m, U, {
        key, uniforms: this.u,
        vertex: (sh) => {
          sh.vertexShader = sh.vertexShader
            .replace('#include <common>', '#include <common>\nvarying vec2 vRibUv;')
            .replace('#include <uv_vertex>', '#include <uv_vertex>\nvRibUv = uv;');
        },
        fragment: (sh) => {
          sh.fragmentShader = sh.fragmentShader
            .replace('#include <common>', '#include <common>\nvarying vec2 vRibUv;\nuniform vec3 uSunView;\nuniform vec3 uSunRad;')
            .replace('#include <map_fragment>', `#include <map_fragment>
      // ribbons: circumferential bands separated by gaps (box-filtered so they fade to an average far away),
      // radial reinforcing tapes over the suspension-line seams, solid skirt hem and vent band
      float ribY = vRibUv.y * ${ribbons.toFixed(1)};
      float ribW = max(fwidth(ribY), 1e-4);
      float ribA = ribY + 0.5 * ribW, ribB = ribY - 0.5 * ribW;
      float ribCov = ((floor(ribA) * 0.74 + min(fract(ribA), 0.74)) - (floor(ribB) * 0.74 + min(fract(ribB), 0.74))) / ribW;
      float gorX = vRibUv.x * ${NG.toFixed(1)} + 0.04;
      float gorW = max(fwidth(gorX), 1e-4);
      float tpA = gorX + 0.5 * gorW, tpB = gorX - 0.5 * gorW;
      float ribTape = ((floor(tpA) * 0.08 + min(fract(tpA), 0.08)) - (floor(tpB) * 0.08 + min(fract(tpB), 0.08))) / gorW;
      float ribBand = max(1.0 - smoothstep(0.03, 0.05, vRibUv.y), smoothstep(0.92, 0.94, vRibUv.y));
      float ribSolid = clamp(max(max(ribCov, ribTape), ribBand), 0.0, 1.0);
      float ribGore = floor(gorX);
      float ribTint = 0.95 + 0.05 * fract(sin(ribGore * 12.9898) * 43758.5453);
      diffuseColor.rgb *= ribTint * mix(0.30, 1.0, ribSolid) * mix(1.0, 0.84, ribTape * (1.0 - ribBand));`)
            .replace('#include <emissivemap_fragment>', `#include <emissivemap_fragment>
      {
        // thin nylon: sunlight shining through the fabric and the ribbon gaps when back-lit
        float back = max(dot(-normal, uSunView), 0.0);
        totalEmissiveRadiance += uSunRad * back * (diffuseColor.rgb * 0.05 + (1.0 - ribSolid) * 0.06);
      }`);
        },
      });
      return m;
    };
    this.canopy = new THREE.Mesh(cg, mkMat(30, 'chute'));
    this.canopy.castShadow = true;
    this.canopy.frustumCulled = false;
    this.group.add(this.canopy);

    // ---------------- pilot chute (static small dome, axis +X, mouth toward -X)
    const pp = [];
    for (let i = 0; i <= 14; i++) {
      const p = 0.1 + i / 14 * 1.55;
      pp.push(new THREE.Vector2(this.Rp * Math.sin(p), this.Rp * 0.85 * Math.cos(p)));
    }
    const pg = new THREE.LatheGeometry(pp, 24);
    pg.rotateZ(-Math.PI / 2);                     // lathe +Y -> +X
    this.pilot = new THREE.Mesh(pg, mkMat(9, 'pilot'));
    this.pilot.castShadow = true;
    this.pilot.frustumCulled = false;
    this.group.add(this.pilot);

    // ---------------- lines: suspension lines, riser, bridle, pilot lines
    this.nLineV = NG * 2 + 2 + 2 + NPL * 2;
    this.lpos = new Float32Array(this.nLineV * 3);
    const lg = new THREE.BufferGeometry();
    lg.setAttribute('position', new THREE.BufferAttribute(this.lpos, 3).setUsage(THREE.DynamicDrawUsage));
    this.lineMat = new THREE.LineBasicMaterial({ color: 0xdddddd });
    this.lines = new THREE.LineSegments(lg, this.lineMat);
    this.lines.frustumCulled = false;
    this.group.add(this.lines);

    this.prof = { r: new Float32Array(NS + 1), x: new Float32Array(NS + 1) };
    this.reset();
  }

  reset() {
    this.mode = 'stowed';
    this.group.visible = false;
    this.group.scale.set(1, 1, 1);
    this.k = 0; this.tDep = 0; this.dir = null; this.peak = 0;
    this.free = null;
    this.phase = Math.random() * 10;
  }

  /** meridian profile (radius, axial distance from the skirt) for inflation k at row s (0 skirt .. 1 vent) */
  _profile(k, t) {
    const D0 = this.D0, Rf = 0.37 * D0, Hd = 0.26 * D0;
    const PHS = 1.72, PHV = 0.17, cS = Math.cos(PHS);
    const breathe = ss(0.6, 1.0, k) * 0.035;
    const P = this.prof;
    for (let j = 0; j <= NS; j++) {
      const s = j / NS;
      // full canopy: slightly over-hemispherical ellipse (skirt curls in a little)
      const ph = PHS + (PHV - PHS) * s;
      const rF = Rf * Math.sin(ph), xF = Hd * (Math.cos(ph) - cS);
      // reefed: same crown, narrower and longer, skirt pinched by the reefing line
      const rR = 0.58 * rF * lerp(0.28, 1.0, ss(0.0, 0.45, s)), xR = 1.45 * xF;
      // streamer (lines stretched, canopy still in its sleeve)
      const rS = 0.42 * (1 - 0.45 * s) + 0.08, xS = 6.4 * s;
      let r, x;
      if (k < 0.55) { const u = ss(0.1, 0.55, k); r = lerp(rS, rR, u); x = lerp(xS, xR, u); }
      else { const u = ss(0.55, 1.0, k); r = lerp(rR, rF, u); x = lerp(xR, xF, u); }
      const b = Math.sin(t * 2 * Math.PI * 0.9 + this.phase + 1.8 * s);
      P.r[j] = r * (1 + breathe * b);
      P.x[j] = x * (1 - breathe * 0.5 * b);
    }
    return P;
  }

  /**
   * @param {number} dt         real-time step scaled by the sim time scale
   * @param {object} p          { state, attach: Vector3 (three world), vAir: Vector3, vel: Vector3, wind: Vector3,
   *                              groundY(x, z), sunDir: Vector3 (world), sunRad: Color, camera, time }
   */
  update(dt, p) {
    const state = p.state || 0;
    if (this.mode === 'stowed') {
      if (state > 0.004) { this.mode = 'attached'; this.tDep = 0; this.k = 0; this.peak = 0; this.dir = null; }
      else { this.group.visible = false; return; }
    }
    if (this.mode === 'attached') {
      if (state <= 0.0005 && this.tDep > 1.0) {
        // jettisoned: release and let it fall behind
        this.mode = 'free';
        this.free = { vel: p.vel.clone(), t: 0, k0: this.k };
      } else if (state <= 0.0005) { this.reset(); return; }
    }
    const g = this.group;
    g.visible = true;
    this.tDep += dt;
    const time = p.time || 0;

    // ---------------- orientation / position
    const vAir = p.vAir;
    const spd = vAir.length();
    if (this.mode === 'attached') {
      g.position.copy(p.attach);
      const tgt = spd > 2 ? vAir.clone().multiplyScalar(-1 / spd) : (this.dir ? this.dir.clone() : new THREE.Vector3(-1, 0, 0));
      // gravity droop: canopy weight (~230 kg with lines) against its drag
      const drag = 0.5 * 1.2 * spd * spd * this.CdS * Math.max(state, 0.02);
      tgt.y -= Math.min(0.5, (230 * 9.81) / Math.max(drag, 1));
      tgt.normalize();
      if (!this.dir) this.dir = tgt.clone();
      else this.dir.lerp(tgt, Math.min(1, dt / 0.35)).normalize();
      this.k += (inflationFromState(state) - this.k) * Math.min(1, dt / 0.25);
      this.peak = Math.max(this.peak, this.k);
    } else {
      // released: decelerate to the wind, fall, collapse; lie on the runway
      const f = this.free;
      f.t += dt;
      const w = p.wind;
      const kd = Math.exp(-dt / 0.9);
      f.vel.x = w.x + (f.vel.x - w.x) * kd;
      f.vel.z = w.z + (f.vel.z - w.z) * kd;
      f.vel.y += (-3.2 - f.vel.y) * Math.min(1, dt / 0.6);
      g.position.addScaledVector(f.vel, dt);
      const gy = p.groundY(g.position.x, g.position.z) + 0.15;
      if (g.position.y < gy) { g.position.y = gy; f.vel.y = 0; f.vel.x *= 0.9; f.vel.z *= 0.9; }
      this.k = Math.max(0.0, f.k0 * Math.exp(-f.t / 0.7));
      this.dir.y += (-0.02 - this.dir.y) * Math.min(1, dt / 1.5);
      this.dir.normalize();
      if (f.t > 150) { this.group.visible = false; return; }
    }

    // ---------------- canopy shape
    const ext = this.mode === 'free' ? 1 : ss(0.0, 0.65, this.tDep);          // riser paid out
    const extL = this.mode === 'free' ? 1 : ss(0.25, 0.9, this.tDep);         // lines unfolded
    const k = this.k;
    const P = this._profile(k, time);
    const xc = this.Lr * ext;
    const rs = P.r[0];
    const Lline = Math.max(this.Ls * extL, 0.3);
    const xs = xc + Math.sqrt(Math.max(Lline * Lline - rs * rs, 0.01));
    const scal = 0.022 * ss(0.4, 1.0, k);                                         // gore bulge between radials
    const cp = this.cpos;
    for (let j = 0; j <= NS; j++) {
      const s = j / NS;
      const bulgeS = Math.sqrt(Math.sin(Math.PI * (0.12 + 0.88 * s)));
      const hem = (1 - s) ** 6 * 0.35 * ss(0.4, 1, k);                          // hem pulled toward the vent between lines
      for (let i = 0; i <= NA; i++) {
        const th = (i / NA) * Math.PI * 2;
        const f = (i % PER) / PER;
        const b = Math.sin(Math.PI * f);
        const r = P.r[j] * (1 + scal * b * bulgeS) * (1 + 0.012 * Math.sin(3 * th + time * 1.7 + this.phase) * ss(0.6, 1, k));
        const o = (j * (NA + 1) + i) * 3;
        cp[o] = xs + P.x[j] + hem * b;
        cp[o + 1] = r * Math.cos(th);
        cp[o + 2] = r * Math.sin(th);
      }
    }
    // normals from the grid (outward = away from the axis / toward the crown)
    const nr = this.cnrm, W = NA + 1;
    const a = new THREE.Vector3(), b = new THREE.Vector3(), n = new THREE.Vector3();
    for (let j = 0; j <= NS; j++) for (let i = 0; i <= NA; i++) {
      const im = i === 0 ? NA - 1 : i - 1, ip = i === NA ? 1 : i + 1;
      const jm = Math.max(0, j - 1), jp = Math.min(NS, j + 1);
      const o1 = (j * W + ip) * 3, o0 = (j * W + im) * 3, p1 = (jp * W + i) * 3, p0 = (jm * W + i) * 3;
      a.set(cp[o1] - cp[o0], cp[o1 + 1] - cp[o0 + 1], cp[o1 + 2] - cp[o0 + 2]);       // around
      b.set(cp[p1] - cp[p0], cp[p1 + 1] - cp[p0 + 1], cp[p1 + 2] - cp[p0 + 2]);       // toward the vent
      n.crossVectors(a, b).normalize();
      const o = (j * W + i) * 3;
      // make it point away from the axis
      if (n.y * cp[o + 1] + n.z * cp[o + 2] < 0) n.negate();
      nr[o] = n.x; nr[o + 1] = n.y; nr[o + 2] = n.z;
    }
    this.canopyGeo.attributes.position.needsUpdate = true;
    this.canopyGeo.attributes.normal.needsUpdate = true;

    // ---------------- lines
    const L = this.lpos;
    let q = 0;
    const put = (x, y, z) => { L[q++] = x; L[q++] = y; L[q++] = z; };
    for (let gi = 0; gi < NG; gi++) {
      const o = (gi * PER) * 3;
      put(cp[o], cp[o + 1], cp[o + 2]); put(xc, 0, 0);
    }
    put(0, 0, 0); put(xc, 0, 0);                                                 // riser
    const xv = xs + P.x[NS];
    const xp = xv + this.Lb * ss(0.05, 0.5, this.tDep) * (this.mode === 'free' ? Math.max(0.3, 1 - this.free.t / 3) : 1);
    put(xv, 0, 0); put(xp, 0, 0);                                                // bridle
    const pk = this.mode === 'free' ? Math.max(0.2, 1 - this.free.t / 2) : ss(0.0, 0.35, this.tDep);
    for (let i = 0; i < NPL; i++) {
      const th = i / NPL * Math.PI * 2;
      put(xp, 0, 0); put(xp + this.Rp * 1.5, Math.cos(th) * this.Rp * 0.98 * pk, Math.sin(th) * this.Rp * 0.98 * pk);
    }
    this.lines.geometry.attributes.position.needsUpdate = true;
    this.pilot.position.set(xp + this.Rp * 1.5 - 0.13 * this.Rp, 0, 0);
    this.pilot.scale.set(1, pk, pk);

    // ---------------- orientation with coning, keep the canopy above the ground
    const dir = this.dir.clone();
    if (this.mode === 'attached') {
      const cone = 0.035 * ss(0.5, 1, k);
      const up = Math.abs(dir.y) < 0.95 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
      const e1 = new THREE.Vector3().crossVectors(dir, up).normalize();
      const e2 = new THREE.Vector3().crossVectors(e1, dir).normalize();
      const w1 = time * 2 * Math.PI * 0.33 + this.phase, w2 = time * 2 * Math.PI * 0.21 + this.phase * 1.7;
      dir.addScaledVector(e1, cone * Math.sin(w1)).addScaledVector(e2, cone * 0.6 * Math.sin(w2)).normalize();
    }
    {
      // ground clearance of the canopy bottom
      const xm = xs + 0.5 * P.x[NS];
      let rmax = 0; for (let j = 0; j <= NS; j++) rmax = Math.max(rmax, P.r[j]);
      const cx = g.position.x + dir.x * xm, cz = g.position.z + dir.z * xm;
      const need = (p.groundY(cx, cz) + 0.25 + rmax - g.position.y) / xm;
      if (dir.y < need) { dir.y = need; dir.normalize(); this.dir.y = Math.max(this.dir.y, need); this.dir.normalize(); }
    }
    // basis with local +Y kept as close to world up as possible (so a canopy lying on the runway can be flattened)
    const zA = new THREE.Vector3().crossVectors(dir, new THREE.Vector3(0, 1, 0));
    if (zA.lengthSq() < 1e-6) zA.set(0, 0, 1);
    zA.normalize();
    const yA = new THREE.Vector3().crossVectors(zA, dir).normalize();
    g.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(dir, yA, zA));
    const flat = this.mode === 'free' ? ss(0.8, 3.0, this.free.t) : 0;
    g.scale.set(1, 1 - 0.75 * flat, 1 + 1.6 * flat);

    // ---------------- lighting for the unlit lines + translucency uniforms
    if (p.camera) {
      this.u.uSunView.value.copy(p.sunDir).transformDirection(p.camera.matrixWorldInverse);
    }
    this.u.uSunRad.value.copy(p.sunRad);
    const day = Math.max(p.sunDir.y, 0);
    const lit = 0.7 / Math.PI;
    this.lineMat.color.copy(p.sunRad).multiplyScalar(lit * (0.22 * Math.sqrt(day) + 0.14));
  }
}
