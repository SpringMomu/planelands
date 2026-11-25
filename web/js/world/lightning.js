// Lightning: cloud-to-ground bolts (fractal main channel + branches, several return strokes with
// continuing current) and intra-cloud flashes.  Each event drives the shared flash uniforms that
// light the cloud volume, rain shafts, terrain and (through a hemisphere light) all standard materials.
// Every event also keeps its channel geometry (ev.acoustic: segments + per-segment weight + which
// strokes light them) - the thunder is synthesised from it (audio.js): the visible CG channel plus
// the horizontal in-cloud channels that a CG flash grows from and an IC flash consists of.
import * as THREE from 'three';
import { LineSegments2 } from '../../vendor/three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from '../../vendor/three/addons/lines/LineSegmentsGeometry.js';
import { LineMaterial } from '../../vendor/three/addons/lines/LineMaterial.js';

const rnd = Math.random;
const rr = (a, b) => a + (b - a) * rnd();

/** midpoint-displacement polyline from a to b; returns an array of Vector3 */
function channel(a, b, depth, rough) {
  let pts = [a.clone(), b.clone()];
  for (let k = 0; k < depth; k++) {
    const np = [pts[0]];
    const r = rough * Math.pow(0.92, k);
    for (let i = 1; i < pts.length; i++) {
      const p = pts[i - 1], q = pts[i];
      const len = p.distanceTo(q);
      const m = p.clone().add(q).multiplyScalar(0.5);
      m.x += (rnd() - 0.5) * len * r;
      m.z += (rnd() - 0.5) * len * r;
      m.y += (rnd() - 0.5) * len * r * 0.35;
      np.push(m, q);
    }
    pts = np;
  }
  return pts;
}

/** acoustic channel: segments (6 floats each), weight (energy per length, 1 = main channel), stroke mask
 * (bit k: lit by stroke k; main channels by all, branches by the first) */
function acousticAdd(ac, pts, w, mask) {
  for (let i = 1; i < pts.length; i++) {
    const p = pts[i - 1], q = pts[i];
    ac.seg.push(p.x, p.y, p.z, q.x, q.y, q.z); ac.w.push(w); ac.mask.push(mask);
  }
}

/** horizontal in-cloud channel network from `o` (a few km, spreading roughly level) */
function cloudChannels(ac, o, n, lenLo, lenHi, w, yLo, yHi, mask) {
  for (let k = 0; k < n; k++) {
    const ang = rnd() * Math.PI * 2, L = rr(lenLo, lenHi);
    const e = new THREE.Vector3(o.x + Math.cos(ang) * L, Math.min(yHi, Math.max(yLo, o.y + rr(-0.12, 0.12) * L)), o.z + Math.sin(ang) * L);
    const pts = channel(o, e, 8, 0.55);
    acousticAdd(ac, pts, w * rr(0.7, 1.2), mask);
    if (rnd() < 0.6) {                                    // side branch
      const s = pts[Math.floor(rr(0.25, 0.75) * pts.length)];
      const a2 = ang + rr(-1.2, 1.2), L2 = L * rr(0.25, 0.55);
      const e2 = new THREE.Vector3(s.x + Math.cos(a2) * L2, Math.min(yHi, Math.max(yLo, s.y + rr(-0.1, 0.1) * L2)), s.z + Math.sin(a2) * L2);
      acousticAdd(ac, channel(s, e2, 7, 0.6), w * rr(0.35, 0.6), mask);
    }
  }
}

function buildBolt(top, bottom, ac) {
  const pos = [], col = [];
  const add = (pts, b, taper = true) => {
    acousticAdd(ac, pts, b, b >= 0.99 ? 0xffff : 1);
    for (let i = 1; i < pts.length; i++) {
      const p = pts[i - 1], q = pts[i];
      pos.push(p.x, p.y, p.z, q.x, q.y, q.z);
      const f0 = taper ? b * (1 - 0.55 * (i - 1) / pts.length) : b, f1 = taper ? b * (1 - 0.55 * i / pts.length) : b;
      col.push(f0 * 0.82, f0 * 0.88, f0, f1 * 0.82, f1 * 0.88, f1);
    }
  };
  const main = channel(top, bottom, 8, 0.6);
  add(main, 1.0, false);
  // branches leave the main channel downward/outward; some branch again
  const nb = 6 + Math.floor(rnd() * 9);
  const H = top.y - bottom.y;
  for (let k = 0; k < nb; k++) {
    const i = Math.floor(rr(0.05, 0.75) * main.length);
    const s = main[i];
    const L = H * rr(0.12, 0.42);
    const ang = rnd() * Math.PI * 2;
    const e = s.clone().add(new THREE.Vector3(Math.cos(ang) * L * rr(0.4, 0.9), -L * rr(0.35, 0.9), Math.sin(ang) * L * rr(0.4, 0.9)));
    e.y = Math.max(e.y, bottom.y + 30);
    const br = channel(s, e, 6, 0.65);
    add(br, rr(0.22, 0.45));
    if (rnd() < 0.45) {
      const j = Math.floor(rr(0.2, 0.7) * br.length);
      const s2 = br[j];
      const L2 = L * rr(0.25, 0.5), a2 = ang + rr(-1, 1);
      const e2 = s2.clone().add(new THREE.Vector3(Math.cos(a2) * L2, -L2 * rr(0.3, 0.8), Math.sin(a2) * L2));
      e2.y = Math.max(e2.y, bottom.y + 30);
      add(channel(s2, e2, 5, 0.7), rr(0.1, 0.22));
    }
  }
  const g = new LineSegmentsGeometry();
  g.setPositions(pos);
  g.setColors(col);
  return g;
}

export class Lightning {
  constructor(scene) {
    this.group = new THREE.Group();
    this.group.name = 'lightning';
    scene.add(this.group);
    this.events = [];
    this.timer = 2 + rnd() * 4;
    this.mats = new Set();
  }

  /**
   * schedule / spawn a strike.
   * @param {object} o { type: 'cg' | 'ic', pos: Vector3 (three, at the cloud base above the strike point),
   *                     groundY, base, thick, camPos }
   */
  spawn(o) {
    const ev = { type: o.type, t: 0, strokes: [], dur: 0, mesh: null, flashPos: new THREE.Vector3(), acoustic: { seg: [], w: [], mask: [] } };
    const ac = ev.acoustic;
    if (o.type === 'cg') {
      const n = 2 + Math.floor(rnd() * 4);
      let t = 0;
      for (let i = 0; i < n; i++) { ev.strokes.push({ t0: t, peak: i === 0 ? 1.0 : rr(0.45, 0.85) }); t += rr(0.04, 0.12); }
      ev.dur = t + rr(0.12, 0.35);
      const top = new THREE.Vector3(o.pos.x, o.base + rr(80, 400), o.pos.z);
      const bottom = new THREE.Vector3(o.pos.x + rr(-900, 900), o.groundY, o.pos.z + rr(-900, 900));
      const mat = new LineMaterial({ color: 0xffffff, linewidth: rr(2.0, 3.2), vertexColors: true, worldUnits: false, transparent: false, opacity: 1 });
      mat.resolution.copy(this._res || new THREE.Vector2(1920, 1080));
      ev.mesh = new LineSegments2(buildBolt(top, bottom, ac), mat);
      // the in-cloud part the CG flash starts from (negative leader network), lit by every stroke's
      // continuing current: inside the cloud, invisible, but a large part of the rumble
      cloudChannels(ac, top, 2 + Math.floor(rnd() * 3), 1500, 5000, 0.45, o.base + 80, o.base + o.thick * 0.55, 0xffff);
      ev.mesh.frustumCulled = false;
      ev.mesh.visible = false;
      this.group.add(ev.mesh);
      this.mats.add(mat);
      ev.flashPos.set(top.x * 0.5 + bottom.x * 0.5, o.groundY + (o.base - o.groundY) * 0.55, top.z * 0.5 + bottom.z * 0.5);
      ev.scale = 170;
    } else {
      const n = 3 + Math.floor(rnd() * 5);
      let t = 0;
      for (let i = 0; i < n; i++) { ev.strokes.push({ t0: t, peak: rr(0.35, 1.0) }); t += rr(0.03, 0.16); }
      ev.dur = t + 0.15;
      ev.flashPos.set(o.pos.x + rr(-1500, 1500), o.base + o.thick * rr(0.15, 0.55), o.pos.z + rr(-1500, 1500));
      ev.scale = 120;
      // intra-cloud flash: a few km of roughly horizontal channels, each pulse re-lighting part of them
      cloudChannels(ac, ev.flashPos, 3 + Math.floor(rnd() * 4), 1500, 6000, 0.55, o.base + 60, o.base + o.thick * 0.8, 0xffff);
    }
    ev.dist = ev.flashPos.distanceTo(o.camPos);
    ev.groundY = o.groundY;
    this.events.push(ev);
    return ev;
  }

  _intensity(ev) {
    let I = 0;
    for (const s of ev.strokes) {
      const dt = ev.t - s.t0;
      if (dt < 0) continue;
      I = Math.max(I, s.peak * (dt < 0.004 ? dt / 0.004 : Math.exp(-(dt - 0.004) / 0.03)));
    }
    if (ev.t < ev.dur) I = Math.max(I, 0.07 * (1 - ev.t / ev.dur) * (0.7 + 0.3 * Math.sin(ev.t * 180)));
    return I;
  }

  setResolution(w, h) {
    this._res = new THREE.Vector2(w, h);
    for (const m of this.mats) m.resolution.set(w, h);
  }

  /**
   * @param {number} dt sim-time step
   * @param {object} c { rate (strikes/s), camPos, findSite() -> {x, z} | null, groundY(x, z), base, thick, onStrike(ev),
   *                     vis(ev) -> { cam, dCam, gnd, dGnd }: fraction of the flash reaching the camera / the ground
   *                     below it through the clouds, and the distances }
   * @returns {{pos: Vector3, I: number, amb: number, ambG: number, vis: number}} strongest flash this frame;
   *          amb: flash light at the camera (objects around it), ambG: on the ground below it
   */
  update(dt, c) {
    if (c.rate > 0 && dt > 0) {
      this.timer -= dt;
      if (this.timer <= 0) {
        this.timer = -Math.log(1 - rnd()) / c.rate;
        const site = c.findSite();
        if (site) {
          const ev = this.spawn({ type: rnd() < 0.42 ? 'cg' : 'ic', pos: new THREE.Vector3(site.x, 0, site.z), groundY: c.groundY(site.x, site.z), base: c.base, thick: c.thick, camPos: c.camPos });
          c.onStrike?.(ev);
        }
      }
    }
    let best = null, bestI = 0, amb = 0, ambG = 0, bestVis = 1;
    for (const ev of this.events) {
      ev.t += dt;
      const I = this._intensity(ev);
      ev.I = I;
      if (ev.mesh) {
        ev.mesh.visible = I > 0.02;
        ev.mesh.material.color.setScalar(60 + I * 900);
      }
      const v = c.vis ? c.vis(ev) : { cam: 1, dCam: ev.dist, gnd: 1, dGnd: ev.dist };
      const w = I * ev.scale;
      if (w > bestI) { bestI = w; best = ev; bestVis = v.cam; }
      const k = I * (ev.type === 'cg' ? 4.0 : 2.5);
      amb += k * v.cam / (1 + (v.dCam / 7000) ** 2);
      ambG += k * v.gnd / (1 + (v.dGnd / 7000) ** 2);
    }
    // retire finished events
    for (let i = this.events.length - 1; i >= 0; i--) {
      const ev = this.events[i];
      if (ev.t > ev.dur + 0.05) {
        if (ev.mesh) { this.group.remove(ev.mesh); ev.mesh.geometry.dispose(); ev.mesh.material.dispose(); this.mats.delete(ev.mesh.material); }
        this.events.splice(i, 1);
      }
    }
    return { pos: best ? best.flashPos : null, I: bestI, amb, ambG, vis: bestVis };
  }

  clear() {
    for (const ev of this.events) if (ev.mesh) { this.group.remove(ev.mesh); ev.mesh.geometry.dispose(); ev.mesh.material.dispose(); }
    this.events = []; this.mats.clear();
  }
}
