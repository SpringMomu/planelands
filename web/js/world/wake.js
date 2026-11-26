// Air motion close to the orbiter on (or just above) the runway, for the particles it throws up
// (tyre spray, scrape sparks and dust): the wind, the wake the vehicle drags along, and the trailing
// wing-tip vortex pair in ground effect.
//
// * wake: air behind a bluff body moving at V is carried along at a fraction of V that decays with
//   the distance behind it (momentum deficit of the wake, ground frame)
// * trailing vortices: lift L at speed V sheds two vortices of circulation G = L / (rho V b0), with
//   b0 = pi/4 b for elliptic loading.  Near the ground each vortex is mirrored by an image of opposite
//   sense: the pair stops sinking and runs outward at G / (4 pi h), the spray it catches is rolled up
//   and carried sideways.  Lamb-Oseen cores growing with age, circulation decaying in ground effect.
//   The vortex lines lie along the track; a point's age is (distance behind the vehicle) / V.
import * as THREE from 'three';

const RHO = 1.2;
const B0 = Math.PI / 4 * 23.79;          // vortex span (elliptic loading)

export class WakeField {
  constructor() {
    this.pos = new THREE.Vector3();      // vehicle reference point (three world)
    this.fwd = new THREE.Vector3(1, 0, 0);   // ground-plane direction of motion
    this.right = new THREE.Vector3(0, 0, 1);
    this.V = 0;
    this.G = 0;                          // circulation of each trailing vortex (m^2/s)
    this.hv = 3.5;                       // vortex height above the ground (m)
    this.gy = 0;                         // ground height under the vehicle (three y)
    this.wind = new THREE.Vector3();
  }

  /**
   * @param {object} s { pos: Vector3, vel: Vector3 (three, m/s), wind: Vector3, lift: N (aerodynamic, >= 0),
   *                     wingY: height of the wing above the ground (m), groundY: ground three y }
   */
  set(s) {
    this.pos.copy(s.pos);
    this.wind.copy(s.wind);
    const vx = s.vel.x - s.wind.x, vz = s.vel.z - s.wind.z;
    const V = Math.hypot(s.vel.x, s.vel.z);
    this.V = V;
    if (V > 0.5) this.fwd.set(s.vel.x / V, 0, s.vel.z / V);
    this.right.set(-this.fwd.z, 0, this.fwd.x);
    const Va = Math.max(Math.hypot(vx, vz), 5);
    this.G = Math.max(0, s.lift) / (RHO * Va * B0);
    this.hv = Math.max(1.5, s.wingY ?? 3.5);
    this.gy = s.groundY;
  }

  /** local ground plane around the vehicle (three y = a + b (x - x0) + c (z - z0)) from three samples:
   *  the particles stay within a few hundred metres of it, where the runway is flat to millimetres */
  fitGround(groundY, x0, z0) {
    const d = 30;
    const h0 = groundY(x0, z0), hx = groundY(x0 + d, z0), hz = groundY(x0, z0 + d);
    this.g0 = { x0, z0, a: h0, b: (hx - h0) / d, c: (hz - h0) / d };
  }

  groundAt(x, z) {
    const g = this.g0;
    return g ? g.a + g.b * (x - g.x0) + g.c * (z - g.z0) : this.gy;
  }

  /** air velocity (three world, m/s) at x, y, z into out */
  at(x, y, z, out) {
    out.copy(this.wind);
    const V = this.V;
    if (V < 0.5) return out;
    const dx = x - this.pos.x, dz = z - this.pos.z;
    const s = -(dx * this.fwd.x + dz * this.fwd.z);          // distance behind the vehicle
    const r = dx * this.right.x + dz * this.right.z;         // right of the track
    const h = Math.max(y - this.gy, 0);
    if (s < -15) return out;
    const sb = Math.max(s, 0);
    // momentum wake behind the body + wing (forward in the ground frame)
    const wk = 0.22 * V * Math.exp(-sb / 22) * Math.exp(-(r * r) / 64) * Math.exp(-h / 5) * (s < 0 ? Math.exp(s / 4) : 1);
    let ur = 0, uh = 0;
    // trailing vortices with ground images
    const G0 = this.G;
    if (G0 > 1 && s > -2) {
      const age = sb / Math.max(V, 1);
      const G = G0 * Math.exp(-age / 8);
      // outward transport by the image vortex, integrated over the age
      const yv = B0 / 2 + (G0 / (4 * Math.PI * this.hv)) * 8 * (1 - Math.exp(-age / 8));
      // core radius ~5 % of the span when rolled up, growing by turbulent diffusion
      const rc2 = (1.2 + 0.5 * Math.sqrt(age)) ** 2;
      const hv = this.hv;
      // right vortex: counter-clockwise seen from behind (up outboard, down inboard) at (+yv, hv), its
      // image (-) at (+yv, -hv); left vortex mirrored
      const add = (rv, yy, sense) => {
        const a = r - rv, b = h - yy;
        const q2 = a * a + b * b + 1e-4;
        const k = sense * G / (2 * Math.PI * q2) * (1 - Math.exp(-q2 / rc2));
        ur += -k * b; uh += k * a;
      };
      add(yv, hv, 1); add(yv, -hv, -1);
      add(-yv, hv, -1); add(-yv, -hv, 1);
      const fade = s < 0 ? Math.exp(s / 1.5) : 1;          // vortices start at the wing tips
      ur *= fade; uh *= fade;
    }
    out.x += this.fwd.x * wk + this.right.x * ur;
    out.z += this.fwd.z * wk + this.right.z * ur;
    out.y += uh;
    return out;
  }
}
