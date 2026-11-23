// 3-D guidance overlays: SCP planned trajectory (with drop-line curtain), MPC predicted
// horizon, heading-alignment cone, A/L interface gate, approach glide path with gates,
// aim points.  Overlay pixels write alpha 0.5 so the composite keeps them crisp (light haze).
import * as THREE from 'three';
import { Line2 } from '../../vendor/three/addons/lines/Line2.js';
import { LineGeometry } from '../../vendor/three/addons/lines/LineGeometry.js';
import { LineMaterial } from '../../vendor/three/addons/lines/LineMaterial.js';
import { LineSegments2 } from '../../vendor/three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from '../../vendor/three/addons/lines/LineSegmentsGeometry.js';
import { enuToThree } from '../core/frames.js';

const OVERLAY_ALPHA = 0.5;   // marker read by the composite pass

function lineMat(color, width, opts = {}) {
  const m = new LineMaterial({
    color: 0xffffff, linewidth: width, vertexColors: !!opts.vertexColors, dashed: !!opts.dashed,
    dashSize: opts.dashSize || 200, gapSize: opts.gapSize || 120, worldUnits: false,
    transparent: false, opacity: OVERLAY_ALPHA, depthWrite: true,
  });
  m.color.setRGB(color[0], color[1], color[2]);
  return m;
}

function catmull(points, samplesPerSeg = 8) {
  if (points.length < 3) return points;
  const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal');
  return curve.getPoints(Math.max(2, points.length * samplesPerSeg));
}

export class GncViz {
  constructor(terrain) {
    this.terrain = terrain;
    this.group = new THREE.Group();
    this.group.name = 'gnc-viz';
    this.mats = [];
    const reg = (m) => { this.mats.push(m); return m; };
    this.planMat = reg(lineMat([0.35, 1.5, 2.6], 3.2, { vertexColors: true }));
    this.curtainMat = reg(lineMat([1, 1, 1], 1.0, { vertexColors: true }));
    Object.assign(this.curtainMat, { transparent: true, blending: THREE.AdditiveBlending, depthWrite: false, opacity: 1.0 });
    this.mpcMat = reg(lineMat([0.5, 3.0, 0.9], 4.0));
    this.hacMat = reg(lineMat([2.4, 0.7, 2.2], 2.0, { dashed: true, dashSize: 260, gapSize: 160 }));
    this.apprMat = reg(lineMat([2.6, 1.7, 0.4], 2.6));
    this.gateMat = reg(lineMat([2.6, 1.7, 0.4], 2.0));
    this.aliMat = reg(lineMat([3.0, 2.2, 0.6], 3.0));
    this.aimMat = reg(lineMat([3.0, 0.9, 0.3], 2.5));
    this.objs = {};
    this.plan = null;
    this.planPts = [];       // sampled ENU points along the plan (for HUD / map)
    this.planTimes = [];
    this.visible = { plan: true, mpc: true, approach: true };
  }

  _replace(key, obj) {
    const old = this.objs[key];
    if (old) { this.group.remove(old); old.traverse((o) => { if (o.geometry && o.geometry.dispose) o.geometry.dispose(); }); }
    if (obj) { obj.frustumCulled = false; obj.renderOrder = 5; this.group.add(obj); }
    this.objs[key] = obj;
  }

  _line(pointsThree, mat, colors) {
    const g = new LineGeometry();
    const arr = [];
    for (const p of pointsThree) arr.push(p.x, p.y, p.z);
    g.setPositions(arr);
    if (colors) g.setColors(colors);
    const l = new Line2(g, mat);
    l.computeLineDistances();
    return l;
  }
  _segments(pairsThree, mat) {
    const g = new LineSegmentsGeometry();
    const arr = [];
    for (const [a, b] of pairsThree) arr.push(a.x, a.y, a.z, b.x, b.y, b.z);
    g.setPositions(arr);
    return new LineSegments2(g, mat);
  }

  setPlan(plan) {
    this.plan = plan;
    if (!plan || !plan.nodes || plan.nodes.length < 2) { this._replace('plan', null); this._replace('curtain', null); return; }
    const nodes = plan.nodes;
    const enu = nodes.map((n) => new THREE.Vector3(n[1], n[2], n[3]));
    const smooth = catmull(enu, 10);
    // times along the smoothed curve (linear in arc index)
    const t0 = nodes[0][0], t1 = nodes[nodes.length - 1][0];
    this.planPts = smooth.map((p) => [p.x, p.y, p.z]);
    this.planTimes = smooth.map((_, i) => t0 + (t1 - t0) * i / (smooth.length - 1));
    const pts = smooth.map((p) => enuToThree(p.x, p.y, p.z));
    // colour: cyan -> white toward the A/L interface, amber tint when using the fallback reference
    const cols = [];
    pts.forEach((_, i) => {
      const f = i / (pts.length - 1);
      if (plan.fallback) cols.push(2.4, 1.4 + 0.6 * f, 0.4 + 0.6 * f);
      else cols.push(0.35 + 1.6 * f, 1.5 + 0.9 * f, 2.6);
    });
    this._replace('plan', this._line(pts, this.planMat, cols));
    // curtain: drop lines to the terrain every ~25 samples
    const pairs = [], ccol = [];
    for (let i = 0; i < smooth.length; i += 18) {
      const p = smooth[i];
      const g = this.terrain.surfaceAt(p.x, p.y);
      pairs.push([enuToThree(p.x, p.y, p.z), enuToThree(p.x, p.y, g + 2)]);
      ccol.push(0.12, 0.45, 0.8, 0.0, 0.0, 0.0);      // fades out toward the ground (additive)
    }
    const cur = this._segments(pairs, this.curtainMat);
    cur.geometry.setColors(ccol);
    this._replace('curtain', cur);
    // HAC
    if (plan.hac && plan.hac.center) {
      const [cE, cN] = plan.hac.center, R = plan.hac.radius;
      // draw the cone as rings at a few heights taken from the plan altitudes near the HAC
      const near = smooth.filter((p) => Math.abs(Math.hypot(p.x - cE, p.y - cN) - R) < R * 0.25);
      const hTop = near.length ? Math.max(...near.map((p) => p.z)) : 6000;
      const hBot = plan.ali ? plan.ali[2] : 3000;
      const rings = [];
      for (const h of [hTop, (hTop + hBot) / 2, hBot]) {
        const ring = [];
        for (let k = 0; k <= 96; k++) { const a = k / 96 * Math.PI * 2; ring.push(enuToThree(cE + R * Math.sin(a), cN + R * Math.cos(a), h)); }
        rings.push(ring);
      }
      const grp = new THREE.Group();
      rings.forEach((r) => grp.add(this._line(r, this.hacMat)));
      this._replace('hac', grp);
    } else this._replace('hac', null);
    // A/L interface gate + approach glide path
    this._buildApproach(plan);
  }

  _buildApproach(plan) {
    const ap = plan.approach;
    if (!ap || ap.length < 2) { this._replace('appr', null); this._replace('gates', null); this._replace('ali', null); return; }
    const pts = ap.map((p) => enuToThree(p[0], p[1], p[2]));
    this._replace('appr', this._line(pts, this.apprMat));
    // gates every ~1.2 km along the approach (rectangles perpendicular to the path, shrinking near the runway)
    const pairs = [];
    let acc = 0;
    for (let i = 1; i < ap.length; i++) {
      const a = ap[i - 1], b = ap[i];
      const seg = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
      acc += seg;
      if (acc < 1200 && i !== 1) continue;
      acc = 0;
      const dE = b[0] - a[0], dN = b[1] - a[1], L = Math.hypot(dE, dN) || 1;
      const rE = dN / L, rN = -dE / L;                // right vector (horizontal)
      const hgt = Math.max(b[2] - 3, 1);
      const w = Math.min(120, 30 + hgt * 0.04), h = Math.min(80, 18 + hgt * 0.025);
      const c = [b[0], b[1], b[2]];
      const P = (sx, sy) => enuToThree(c[0] + rE * sx * w, c[1] + rN * sx * w, c[2] + sy * h);
      pairs.push([P(-1, -1), P(1, -1)], [P(1, -1), P(1, 1)], [P(1, 1), P(-1, 1)], [P(-1, 1), P(-1, -1)]);
    }
    this._replace('gates', this._segments(pairs, this.gateMat));
    // ALI: a bigger double gate
    if (plan.ali) {
      const [E, N, U] = plan.ali;
      const b = ap[1] || ap[0];
      const dE = b[0] - E, dN = b[1] - N, L = Math.hypot(dE, dN) || 1;
      const rE = dN / L, rN = -dE / L;
      const P = (sx, sy) => enuToThree(E + rE * sx * 260, N + rN * sx * 260, U + sy * 160);
      const sq = [[P(-1, -1), P(1, -1)], [P(1, -1), P(1, 1)], [P(1, 1), P(-1, 1)], [P(-1, 1), P(-1, -1)],
        [P(-1, 0), P(-0.6, 0)], [P(0.6, 0), P(1, 0)], [P(0, -1), P(0, -0.6)], [P(0, 0.6), P(0, 1)]];
      this._replace('ali', this._segments(sq, this.aliMat));
    }
    // aim points: crosses on the ground at the start/end of glide slopes (from the approach polyline)
    const last = ap[ap.length - 1];
    const crosses = [];
    const cross = (E, N, s) => {
      const g = this.terrain.surfaceAt(E, N) + 0.5;
      crosses.push([enuToThree(E - s, N, g), enuToThree(E + s, N, g)], [enuToThree(E, N - s, g), enuToThree(E, N + s, g)]);
    };
    if (plan.aim) for (const a of plan.aim) cross(a[0], a[1], 60);
    else cross(last[0], last[1], 40);
    this._replace('aim', this._segments(crosses, this.aimMat));
  }

  setMpc(pred) {
    if (!pred || pred.length < 2) { this._replace('mpc', null); return; }
    const pts = catmull(pred.map((p) => new THREE.Vector3(p[0], p[1], p[2])), 6).map((p) => enuToThree(p.x, p.y, p.z));
    this._replace('mpc', this._line(pts, this.mpcMat));
  }

  setVisible(key, v) {
    this.visible[key] = v;
    this._applyVis();
  }
  _applyVis() {
    const o = this.objs, v = this.visible;
    for (const k of ['plan', 'curtain', 'hac']) if (o[k]) o[k].visible = v.plan;
    if (o.mpc) o.mpc.visible = v.mpc;
    for (const k of ['appr', 'gates', 'ali', 'aim']) if (o[k]) o[k].visible = v.approach;
  }

  update(width, height) {
    for (const m of this.mats) m.resolution.set(width, height);
    this._applyVis();
  }

  clear() { for (const k of Object.keys(this.objs)) this._replace(k, null); this.plan = null; this.planPts = []; }

  /** reference point on the plan `ahead` seconds beyond the point closest to `enu` */
  refAhead(enu, ahead = 8) {
    const P = this.planPts; if (!P.length) return null;
    let best = 0, bd = Infinity;
    for (let i = 0; i < P.length; i++) {
      const d = (P[i][0] - enu[0]) ** 2 + (P[i][1] - enu[1]) ** 2 + (P[i][2] - enu[2]) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    const tt = this.planTimes[best] + ahead;
    let j = best; while (j < P.length - 1 && this.planTimes[j] < tt) j++;
    return { point: P[j], dist: Math.sqrt(bd), index: best };
  }
}
