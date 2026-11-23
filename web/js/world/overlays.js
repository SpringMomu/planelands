// Flight trail and per-element aerodynamic force vectors.
import * as THREE from 'three';
import { enuToThree, frdToLocal } from '../core/frames.js';

const GROUP_COLORS = {
  wing_l: 0x59d2ff, wing_r: 0x59d2ff, elevon_li: 0xffb547, elevon_lo: 0xffb547, elevon_ri: 0xffb547, elevon_ro: 0xffb547,
  bodyflap: 0xff6b5a, vtail: 0x9d8cff, rudder_sb: 0xff8cf0, fuselage: 0x6cf0a0, gear: 0xcccccc, chute: 0xffffff,
};

/**
 * Flight trail.  Points are fed from every state the simulator sends (so the trail stays complete
 * while the tab is in the background and rAF is paused) and drawn as segments up to the render
 * clock, plus a head segment attached to the interpolated vehicle position.  Gaps in the data
 * (lost connection, frozen tab) break the line instead of drawing a straight jump; a new episode
 * clears it.
 */
export class Trail {
  constructor(maxSeg = 12000) {
    this.max = maxSeg;
    this.pos = new Float32Array(maxSeg * 6);
    this.col = new Float32Array(maxSeg * 6);
    this.time = new Float64Array(maxSeg);
    this.geo = new THREE.BufferGeometry();
    this.geo.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    this.geo.setAttribute('color', new THREE.BufferAttribute(this.col, 3).setUsage(THREE.DynamicDrawUsage));
    this.geo.setDrawRange(0, 0);
    const mat = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9 });
    this.segs = new THREE.LineSegments(this.geo, mat);
    this.segs.frustumCulled = false;
    this.hpos = new Float32Array(6);
    this.hcol = new Float32Array(6);
    this.hgeo = new THREE.BufferGeometry();
    this.hgeo.setAttribute('position', new THREE.BufferAttribute(this.hpos, 3).setUsage(THREE.DynamicDrawUsage));
    this.hgeo.setAttribute('color', new THREE.BufferAttribute(this.hcol, 3).setUsage(THREE.DynamicDrawUsage));
    this.head = new THREE.LineSegments(this.hgeo, mat);
    this.head.frustumCulled = false;
    this.line = new THREE.Group();          // public handle (visibility toggle)
    this.line.add(this.segs, this.head);
    this.clear();
  }
  clear() {
    this.n = 0;
    this.lastPt = null; this.lastT = -1e9; this.lastCol = [0, 0, 0];
    this.geo.setDrawRange(0, 0);
    this.head.visible = false;
  }
  _color(speed, out) {
    const t = Math.min(1, speed / 700);
    out[0] = 2.4; out[1] = 1.2 + 0.6 * (1 - t); out[2] = 0.35 + 1.6 * t;
    return out;
  }
  /** raw simulator sample (sim time t, ENU position, speed) */
  pushRaw(t, enu, speed) {
    const p = enuToThree(enu[0], enu[1], enu[2]);
    const c = this._color(speed, [0, 0, 0]);
    if (!this.lastPt || t < this.lastT || t - this.lastT > 3.0 || this.lastPt.distanceTo(p) > Math.max(1500, speed * (t - this.lastT) * 3 + 200)) {
      // first point, or a discontinuity: start a new strand without connecting
      this.lastPt = p; this.lastT = t; this.lastCol = c;
      return;
    }
    if (this.lastPt.distanceTo(p) < 25) return;
    if (this.n >= this.max) {
      const drop = Math.floor(this.max * 0.1);
      this.pos.copyWithin(0, drop * 6); this.col.copyWithin(0, drop * 6); this.time.copyWithin(0, drop);
      this.n -= drop;
    }
    const o = this.n * 6;
    this.pos[o] = this.lastPt.x; this.pos[o + 1] = this.lastPt.y; this.pos[o + 2] = this.lastPt.z;
    this.pos[o + 3] = p.x; this.pos[o + 4] = p.y; this.pos[o + 5] = p.z;
    this.col.set(this.lastCol, o); this.col.set(c, o + 3);
    this.time[this.n] = t;
    this.n++;
    this.lastPt = p; this.lastT = t; this.lastCol = c;
    this.geo.attributes.position.needsUpdate = true;
    this.geo.attributes.color.needsUpdate = true;
  }
  /** draw everything up to the render clock and attach the head to the rendered vehicle */
  update(renderT, enu) {
    let nv = this.n;
    while (nv > 0 && this.time[nv - 1] > renderT + 1e-6) nv--;
    this.geo.setDrawRange(0, nv * 2);
    if (nv > 0 && enu) {
      const o = (nv - 1) * 6 + 3;
      const p = enuToThree(enu[0], enu[1], enu[2]);
      const dx = p.x - this.pos[o], dy = p.y - this.pos[o + 1], dz = p.z - this.pos[o + 2];
      if (dx * dx + dy * dy + dz * dz < 2000 * 2000 && renderT - this.time[nv - 1] < 3.0) {
        this.hpos[0] = this.pos[o]; this.hpos[1] = this.pos[o + 1]; this.hpos[2] = this.pos[o + 2];
        this.hpos[3] = p.x; this.hpos[4] = p.y; this.hpos[5] = p.z;
        this.hcol.set(this.col.subarray(o, o + 3), 0); this.hcol.set(this.col.subarray(o, o + 3), 3);
        this.hgeo.attributes.position.needsUpdate = true;
        this.hgeo.attributes.color.needsUpdate = true;
        this.head.visible = true;
        return;
      }
    }
    this.head.visible = false;
  }
}

export class ForceVectors {
  constructor(parent) {
    this.parent = parent;
    this.group = new THREE.Group();
    this.group.visible = false;
    parent.add(this.group);
    this.elements = null;
    this.scale = 1 / 25000;   // metres per newton
  }
  setElements(els) {
    this.elements = els;
    this.group.clear();
    const n = els.length;
    this.pos = new Float32Array(n * 6);
    this.col = new Float32Array(n * 6);
    const c = new THREE.Color();
    els.forEach((e, i) => {
      c.setHex(GROUP_COLORS[e.group] ?? 0xffffff);
      this.col.set([c.r * 2, c.g * 2, c.b * 2, c.r * 2, c.g * 2, c.b * 2], i * 6);
      const p = frdToLocal(e.pos);
      this.pos.set([p.x, p.y, p.z, p.x, p.y, p.z], i * 6);
    });
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(this.col, 3));
    this.lines = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, transparent: true }));
    this.lines.renderOrder = 20;
    this.lines.frustumCulled = false;
    this.group.add(this.lines);
    // dots at the application points
    const pg = new THREE.BufferGeometry();
    pg.setAttribute('position', new THREE.BufferAttribute(this.pos.filter((_, k) => (k % 6) < 3), 3));
    this.dots = new THREE.Points(pg, new THREE.PointsMaterial({ color: 0xffffff, size: 0.35, depthTest: false }));
    this.dots.renderOrder = 21;
    this.group.add(this.dots);
  }
  update(forces) {
    if (!this.elements || !forces || !this.lines) return;
    const attr = this.lines.geometry.attributes.position;
    const n = Math.min(forces.length, this.elements.length);
    for (let i = 0; i < n; i++) {
      const f = forces[i];
      const o = i * 6;
      attr.array[o + 3] = attr.array[o] + f[0] * this.scale;
      attr.array[o + 4] = attr.array[o + 1] - f[2] * this.scale;
      attr.array[o + 5] = attr.array[o + 2] + f[1] * this.scale;
    }
    attr.needsUpdate = true;
  }
}
