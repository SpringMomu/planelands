// Unsteady 2-D flow past the wing section, time-stepped: thin-aerofoil discrete-vortex method with
// leading-edge vortex shedding (LDVM: Ramesh, Gopalarathnam, Granlund, Ol & Edwards, "Discrete-vortex
// method with novel shedding criterion for unsteady aerofoil flows with intermittent leading-edge
// vortex shedding", J. Fluid Mech. 751, 2014), plus a source sheet for the section's thickness.
//
// Frame: leading edge at (0, 0), trailing edge at (1, eta(1)), x aft, y up; lengths in chords, speeds
// in free-stream speeds, time in chords / U.  Free stream (cos a, sin a).  Circulation is positive
// clockwise (positive lift).
//
// Every step:
//  * the bound vortex sheet on the camber line is a Fourier series in theta (x = (1 - cos theta) / 2),
//    its coefficients from the normal-flow condition on the camber line (free stream + all free
//    vortices);
//  * a new vortex leaves the trailing edge with the strength Kelvin's theorem asks for (total
//    circulation constant);
//  * if the leading-edge suction parameter (LESP = A0) exceeds its critical value, a vortex leaves the
//    leading edge as well, strong enough to bring A0 back to the critical value;
//  * every free vortex moves with the local velocity (free stream + bound sheet + thickness sources +
//    all other vortices, Vatistas cores).
// Free vortices: cores grow with age (eddy viscosity), circulation decays after a while (the mixing of
// the two shear layers of a separated wake, which point vortices cannot do), close old vortices of
// the same sign merge.  What decays or leaves the box stays in Kelvin's balance as far-away vorticity
// carried off with the free stream.
// Force: rate of change of the vortical impulse (bound sheet + free + far vortices).

const PI = Math.PI, TWO_PI = 2 * Math.PI;

export class SectionFlow {
  constructor({ n = 72, nf = 36, cap = 1400, dt = 0.015, nu = 0.0003, decay0 = 0.8, decayT = 5 } = {}) {
    this.n = n; this.nf = nf; this.cap = cap; this.dt = dt;
    this.rc = 1.3 * dt;                                  // core radius at birth (1.3 x shedding spacing)
    this.nu = nu;                                        // core growth rc^2 = rc0^2 + 4 nu age
    this.decay0 = decay0; this.decayT = decayT;
    const th = this.th = new Float64Array(n), x = this.x = new Float64Array(n);
    for (let i = 0; i < n; i++) { th[i] = PI * i / (n - 1); x[i] = 0.5 * (1 - Math.cos(th[i])); }
    this.dth = PI / (n - 1);
    this.wq = new Float64Array(n).fill(this.dth); this.wq[0] *= 0.5; this.wq[n - 1] *= 0.5;
    this.cosT = new Float64Array((nf + 1) * n);
    for (let k = 0; k <= nf; k++) for (let i = 0; i < n; i++) this.cosT[k * n + i] = Math.cos(k * th[i]);
    this.eta = new Float64Array(n); this.deta = new Float64Array(n); this.yt = new Float64Array(n);
    this.A = new Float64Array(nf + 1); this.Ap = new Float64Array(nf + 1);
    const f = () => new Float64Array(cap);
    // free vortices: position, circulation, side of the camber line, kind (0 TE, 1 LE), birth time, core^4
    this.vx = f(); this.vy = f(); this.vg = f(); this.vs = new Int8Array(cap); this.vk = new Int8Array(cap); this.vb = f(); this.vr = f();
    this.ux = f(); this.uy = f();
    this.nv = 0; this.gRem = 0; this.irx = 0; this.iry = 0;
    // panels between the plate points: bound circulation, source strength, length, direction, midpoint
    const m = n - 1;
    this.Gs = new Float64Array(m); this.sig = new Float64Array(m); this.pl = new Float64Array(m);
    this.pc = new Float64Array(m); this.ps = new Float64Array(m); this.mx = new Float64Array(m); this.my = new Float64Array(m);
    this.cum = new Float64Array(m); this.cumP = new Float64Array(m); this.dcp = new Float64Array(m);
    this.u0 = new Float64Array(n); this.w0 = new Float64Array(n); this.uT = new Float64Array(n); this.wT = new Float64Array(n);
    this.uL = new Float64Array(n); this.wL = new Float64Array(n); this.W = new Float64Array(n);
    this.aT = new Float64Array(nf + 1); this.aL = new Float64Array(nf + 1);
    this.alpha = 0; this.t = 0; this.cn = 0; this.ct = 0; this.lesp = 0; this.shedLE = false; this.gb = 0; this.nstep = 0;
    this.leRate = 0;
    this.dom = { s0: -1.6, s1: 3.0, n: 1.9 };            // vortices beyond this box (wind axes) are dropped
    this.ready = false;
  }

  /** section shape: camber line eta(x) and half thickness yt(x), sampled at increasing x in [0, 1] */
  setShape(xs, eta, yt) {
    const n = this.n, lerp = (X, Y, x) => {
      if (x <= X[0]) return Y[0];
      if (x >= X[X.length - 1]) return Y[Y.length - 1];
      let lo = 0, hi = X.length - 1;
      while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (X[mid] <= x) lo = mid; else hi = mid; }
      return Y[lo] + (Y[hi] - Y[lo]) * (x - X[lo]) / Math.max(X[hi] - X[lo], 1e-12);
    };
    const e0 = lerp(xs, eta, 0);
    for (let i = 0; i < n; i++) {
      const x = this.x[i];
      this.eta[i] = lerp(xs, eta, x) - e0;
      this.yt[i] = Math.max(lerp(xs, yt, x), 0);
      const h = 0.004, xa = Math.max(x - h, 0), xb = Math.min(x + h, 1);
      this.deta[i] = (lerp(xs, eta, xb) - lerp(xs, eta, xa)) / (xb - xa);
    }
    const m = n - 1;
    for (let j = 0; j < m; j++) {
      const dx = this.x[j + 1] - this.x[j], dy = this.eta[j + 1] - this.eta[j], L = Math.hypot(dx, dy);
      this.pl[j] = L; this.pc[j] = dx / L; this.ps[j] = dy / L;
      this.mx[j] = 0.5 * (this.x[j] + this.x[j + 1]); this.my[j] = 0.5 * (this.eta[j] + this.eta[j + 1]);
    }
    // steady camber integrals: without a wake A0 = sin a - C0 cos a, A1 = C1 cos a
    let C0 = 0, C1 = 0;
    for (let i = 0; i < n; i++) { C0 += this.wq[i] * this.deta[i]; C1 += this.wq[i] * this.deta[i] * this.cosT[n + i]; }
    this.C0 = C0 / PI; this.C1 = 2 * C1 / PI;
    // trailing-edge direction
    const te = Math.atan(this.deta[n - 1]);
    this.teDir = [Math.cos(te), Math.sin(te)];
  }

  /** camber line / half thickness at any x in [0, 1] (linear in theta between the plate points) */
  _at(arr, x) {
    const th = Math.acos(Math.min(Math.max(1 - 2 * x, -1), 1)) / this.dth;
    const i = Math.min(Math.floor(th), this.n - 2), t = th - i;
    return arr[i] + (arr[i + 1] - arr[i]) * t;
  }

  /** steady attached circulation at angle a (no wake): pi (sin a + (C1/2 - C0) cos a) */
  steadyGamma(a) { return PI * (Math.sin(a) + (0.5 * this.C1 - this.C0) * Math.cos(a)); }

  /** restart from the steady attached flow at angle a: the starting vortex is far away */
  reset(a) {
    this.nv = 0; this.t = 0; this.alpha = a; this.shedLE = false; this.nstep = 0;
    const ca = Math.cos(a), sa = Math.sin(a), n = this.n;
    this.A.fill(0);
    this.A[0] = sa - this.C0 * ca;
    for (let q = 1; q <= this.nf; q++) {
      let s = 0;
      for (let i = 0; i < n; i++) s += this.wq[i] * this.deta[i] * ca * this.cosT[q * n + i];
      this.A[q] = 2 * s / PI;
    }
    this.Ap.set(this.A);
    this.gb = PI * (this.A[0] + 0.5 * this.A[1]);
    this.gRem = -this.gb; this.irx = 0; this.iry = 0;
    this._panels(ca);
    this.cumP.set(this.cum);
    this.imp = this._impulse();
    this.cn = TWO_PI * ca * (this.A[0] + 0.5 * this.A[1]);
    this.leRate = 0;
    this.ready = true;
  }

  // velocity of a unit vortex at (px, py) (birth core) on the plate points -> (U, W)
  _onPlate(px, py, U, W) {
    const n = this.n, rc4 = this.rc ** 4;
    for (let i = 0; i < n; i++) {
      const dx = this.x[i] - px, dy = this.eta[i] - py, r2 = dx * dx + dy * dy;
      const f = 1 / (TWO_PI * Math.sqrt(r2 * r2 + rc4));
      U[i] = f * dy; W[i] = -f * dx;
    }
  }

  _fourier(Wv, out) {
    const n = this.n, nf = this.nf, wq = this.wq, cosT = this.cosT;
    let s = 0;
    for (let i = 0; i < n; i++) s += wq[i] * Wv[i];
    out[0] = -s / PI;
    for (let k = 1; k <= nf; k++) {
      let t = 0;
      const o = k * n;
      for (let i = 0; i < n; i++) t += wq[i] * Wv[i] * cosT[o + i];
      out[k] = 2 * t / PI;
    }
  }

  // bound circulation of each panel (from the cumulative sheet strength of the Fourier series) and the
  // thickness source strengths
  _panels(ca) {
    const A = this.A, nf = this.nf, n = this.n, m = n - 1;
    const G = (th) => {
      let g = A[0] * (th + Math.sin(th)) + A[1] * (0.5 * th - 0.25 * Math.sin(2 * th));
      for (let k = 2; k <= nf; k++) g += 0.5 * A[k] * (Math.sin((k - 1) * th) / (k - 1) - Math.sin((k + 1) * th) / (k + 1));
      return g;
    };
    let g0 = G(this.th[0]);
    for (let j = 0; j < m; j++) {
      const g1 = G(this.th[j + 1]);
      this.Gs[j] = g1 - g0;
      this.cum[j] = 0.5 * (g0 + g1);
      g0 = g1;
      this.sig[j] = 2 * ca * (this.yt[j + 1] - this.yt[j]) / Math.max(this.x[j + 1] - this.x[j], 1e-9);
    }
  }

  /** velocity at (px, py): free stream + bound sheet + thickness + free vortices (skip index `self`) */
  velocity(px, py, self = -1, out = [0, 0]) {
    let u = Math.cos(this.alpha), v = Math.sin(this.alpha);
    const m = this.n - 1, rc2 = this.rc * this.rc;
    for (let j = 0; j < m; j++) {
      const dx = px - this.mx[j], dy = py - this.my[j], L = this.pl[j];
      const r2 = dx * dx + dy * dy;
      if (r2 > 4 * L * L) {
        // far: point vortex + point source
        const d = TWO_PI * (r2 + rc2), G = this.Gs[j], Q = this.sig[j] * L;
        u += (G * dy + Q * dx) / d; v += (-G * dx + Q * dy) / d;
      } else {
        const c = this.pc[j], s = this.ps[j];
        const ox = px - this.x[j], oy = py - this.eta[j];
        const xl = ox * c + oy * s, yl = -ox * s + oy * c;
        const r1 = xl * xl + yl * yl, r2b = (xl - L) * (xl - L) + yl * yl;
        const D = Math.atan2(yl, xl - L) - Math.atan2(yl, xl);
        const Lg = Math.log(Math.max(r1, 1e-12) / Math.max(r2b, 1e-12));
        const gm = this.Gs[j] / L, sg = this.sig[j];
        const ul = (gm * D) / TWO_PI + sg * Lg / (4 * PI), vl = -gm * Lg / (4 * PI) + sg * D / TWO_PI;
        u += ul * c - vl * s; v += ul * s + vl * c;
      }
    }
    for (let k = 0; k < this.nv; k++) {
      if (k === self) continue;
      const dx = px - this.vx[k], dy = py - this.vy[k], r2 = dx * dx + dy * dy;
      const f = this.vg[k] / (TWO_PI * Math.sqrt(r2 * r2 + this.vr[k]));
      u += f * dy; v -= f * dx;
    }
    out[0] = u; out[1] = v;
    return out;
  }

  _side(x, y) { return y >= this._at(this.eta, Math.min(Math.max(x, 0), 1)) ? 1 : -1; }

  // keep a point out of the section (back to its side)
  _outside(x, y, side, pad) {
    if (x > -0.02 && x < 1.0) {
      const xc = Math.max(x, 0), e = this._at(this.eta, xc), h = this._at(this.yt, xc) + pad;
      if (Math.abs(y - e) < h) return e + side * h;
    }
    return y;
  }

  _add(x, y, g, kind, side) {
    if (this.nv >= this.cap) this._drop(0);
    const k = this.nv++;
    this.vx[k] = x; this.vy[k] = y; this.vg[k] = g; this.vk[k] = kind; this.vb[k] = this.t; this.vs[k] = side;
    this.vr[k] = this.rc ** 4;
    return k;
  }

  _drop(k) {
    this.gRem += this.vg[k];
    this.irx += this.vg[k] * this.vx[k]; this.iry += this.vg[k] * this.vy[k];
    for (const a of [this.vx, this.vy, this.vg, this.vk, this.vs, this.vb, this.vr]) a.copyWithin(k, k + 1, this.nv);
    this.nv--;
  }

  /** one time step at free-stream angle a; lespCrit = Infinity: no leading-edge shedding */
  step(a, lespCrit) {
    const n = this.n, nf = this.nf, dt = this.dt;
    this.alpha = a;
    const ca = Math.cos(a), sa = Math.sin(a);
    // 0. cores of the free vortices
    const rc2 = this.rc * this.rc, vel = [0, 0];
    for (let k = 0; k < this.nv; k++) { const r2 = rc2 + 4 * this.nu * (this.t - this.vb[k]); this.vr[k] = r2 * r2; }
    // 1. nascent trailing-edge vortex: half a step's travel along the local flow off the edge
    const te = [1, this.eta[n - 1]], td = this.teDir;
    this.velocity(te[0] + 0.012 * td[0], te[1] + 0.012 * td[1], -1, vel);
    const vx = vel[0] + 0.3 * td[0], vy = vel[1] + 0.3 * td[1], vm = Math.hypot(vx, vy) || 1;
    const dT = 0.5 * dt * Math.max(Math.hypot(vel[0], vel[1]), 0.6);
    const tx = te[0] + dT * vx / vm;
    const ty = this._outside(tx, te[1] + dT * vy / vm, vel[1] >= 0 ? 1 : -1, 0.5 * this.rc);
    // 2. normal-flow condition with the existing vortices
    const u0 = this.u0, w0 = this.w0, W = this.W;
    u0.fill(0); w0.fill(0);
    let sumG = this.gRem;
    for (let k = 0; k < this.nv; k++) {
      const g = this.vg[k], px = this.vx[k], py = this.vy[k], r4 = this.vr[k];
      sumG += g;
      for (let i = 0; i < n; i++) {
        const dx = this.x[i] - px, dy = this.eta[i] - py, r2 = dx * dx + dy * dy;
        const f = g / (TWO_PI * Math.sqrt(r2 * r2 + r4));
        u0[i] += f * dy; w0[i] -= f * dx;
      }
    }
    for (let i = 0; i < n; i++) W[i] = this.deta[i] * (ca + u0[i]) - sa - w0[i];
    const A = this.A;
    this.Ap.set(A);
    this._fourier(W, A);
    // 3. trailing-edge vortex from Kelvin's theorem
    this._onPlate(tx, ty, this.uT, this.wT);
    for (let i = 0; i < n; i++) W[i] = this.deta[i] * this.uT[i] - this.wT[i];
    this._fourier(W, this.aT);
    const KT = PI * (this.aT[0] + 0.5 * this.aT[1]) + 1;
    const R = -(PI * (A[0] + 0.5 * A[1]) + sumG);
    let GT = R / KT, GL = 0;
    const a0 = A[0] + GT * this.aT[0];
    // 4. leading-edge vortex when the suction parameter is over the limit
    this.shedLE = false;
    let lx = 0, ly = 0, ls = 1;
    if (Math.abs(a0) > lespCrit) {
      ls = Math.sign(a0);
      const xb = 0.004, yb = ls * (this._at(this.yt, xb) + 0.5 * this.rc);
      this.velocity(0.012, ls * (this._at(this.yt, 0.012) + this.rc), -1, vel);
      const s = Math.hypot(vel[0], vel[1]) || 1, d = 0.5 * dt * Math.max(s, 0.6);
      lx = xb + d * vel[0] / s; ly = yb + d * vel[1] / s;
      ly = this._outside(lx, ly, ls, 0.5 * this.rc);
      this._onPlate(lx, ly, this.uL, this.wL);
      for (let i = 0; i < n; i++) W[i] = this.deta[i] * this.uL[i] - this.wL[i];
      this._fourier(W, this.aL);
      const KL = PI * (this.aL[0] + 0.5 * this.aL[1]) + 1;
      // [KT KL; aT0 aL0] [GT; GL] = [R; s lesp - A0]
      const det = KT * this.aL[0] - KL * this.aT[0];
      if (Math.abs(det) > 1e-12) {
        const r2 = ls * lespCrit - A[0];
        GT = (R * this.aL[0] - KL * r2) / det;
        GL = (KT * r2 - this.aT[0] * R) / det;
        this.shedLE = true;
      }
    }
    for (let k = 0; k <= nf; k++) A[k] += GT * this.aT[k] + (this.shedLE ? GL * this.aL[k] : 0);
    this.lesp = A[0];
    this.gb = PI * (A[0] + 0.5 * A[1]);
    this._add(tx, ty, GT, 0, this._side(tx, ty));
    if (this.shedLE) this._add(lx, ly, GL, 1, ls);
    this.leRate += ((this.shedLE ? GL / dt : 0) - this.leRate) * Math.min(dt / 0.3, 1);
    // 5. sheet strengths; chordwise pressure jump (lower minus upper):
    //    2 [(cos a + u_w) gamma + d/dt (int_0^x gamma) + leading-edge shedding rate]
    this._panels(ca);
    let cnP = 0;
    for (let j = 0; j < n - 1; j++) {
      const uw = 0.5 * (u0[j] + u0[j + 1]) + GT * 0.5 * (this.uT[j] + this.uT[j + 1]) + (this.shedLE ? GL * 0.5 * (this.uL[j] + this.uL[j + 1]) : 0);
      this.dcp[j] = 2 * ((ca + uw) * this.Gs[j] / this.pl[j] + (this.cum[j] - this.cumP[j]) / dt + (this.shedLE ? GL / dt : 0));
      cnP += this.dcp[j] * this.pl[j];
    }
    this.cnP = cnP;
    this.cumP.set(this.cum);
    // 6. move the free vortices (they stay out of the section)
    const nv = this.nv;
    for (let k = 0; k < nv; k++) { this.velocity(this.vx[k], this.vy[k], k, vel); this.ux[k] = vel[0]; this.uy[k] = vel[1]; }
    for (let k = 0; k < nv; k++) {
      const x = this.vx[k] + dt * this.ux[k];
      const y = this._outside(x, this.vy[k] + dt * this.uy[k], this.vs[k], 0.5 * this.rc);
      this.vx[k] = x; this.vy[k] = y;
      if (x > 0 && x < 1) this.vs[k] = this._side(x, y);
    }
    // 7. circulation decay of old vortices, merging, dropping what has left the box
    const kd = 1 - Math.exp(-dt / this.decayT);
    for (let k = 0; k < this.nv; k++) {
      if (this.t - this.vb[k] < this.decay0) continue;
      const dg = this.vg[k] * kd;
      this.vg[k] -= dg; this.gRem += dg; this.irx += dg * this.vx[k]; this.iry += dg * this.vy[k];
    }
    if ((this.nstep & 3) === 0) this._merge();
    const d = this.dom;
    for (let k = this.nv - 1; k >= 0; k--) {
      const rx = this.vx[k] - 0.5, ry = this.vy[k];
      const s = rx * ca + ry * sa, q = -rx * sa + ry * ca;
      if (s > d.s1 || s < d.s0 || Math.abs(q) > d.n || Math.abs(this.vg[k]) < 1e-7) this._drop(k);
    }
    // far vortices travel with the free stream
    this.irx += this.gRem * ca * dt; this.iry += this.gRem * sa * dt;
    this.t += dt; this.nstep++;
    // 8. force from the rate of change of the vortical impulse I = sum G r:  Fy = -dIx/dt, Fx = dIy/dt
    const imp = this._impulse();
    this.cn = -2 * (imp[0] - this.imp[0]) / dt;
    this.ct = 2 * (imp[1] - this.imp[1]) / dt;
    this.imp = imp;
  }

  // merge old vortices of the same sign that have come close (circulation-weighted, impulse kept)
  _merge() {
    const nv = this.nv, t = this.t;
    for (let i = 0; i < nv; i++) {
      if (this.vg[i] === 0 || t - this.vb[i] < 0.6) continue;
      const r2i = Math.sqrt(this.vr[i]);
      for (let j = i + 1; j < nv; j++) {
        if (this.vg[j] === 0 || t - this.vb[j] < 0.6 || (this.vg[i] > 0) !== (this.vg[j] > 0)) continue;
        const dx = this.vx[i] - this.vx[j], dy = this.vy[i] - this.vy[j];
        if (dx * dx + dy * dy > 0.35 * r2i) continue;
        const g = this.vg[i] + this.vg[j];
        this.vx[i] = (this.vg[i] * this.vx[i] + this.vg[j] * this.vx[j]) / g;
        this.vy[i] = (this.vg[i] * this.vy[i] + this.vg[j] * this.vy[j]) / g;
        this.vg[i] = g; this.vb[i] = Math.min(this.vb[i], this.vb[j]);
        this.vg[j] = 0;
      }
    }
    // compact (zero-circulation entries carry no impulse)
    let w = 0;
    for (let k = 0; k < nv; k++) {
      if (this.vg[k] === 0) continue;
      if (w !== k) for (const a of [this.vx, this.vy, this.vg, this.vk, this.vs, this.vb, this.vr]) a[w] = a[k];
      w++;
    }
    this.nv = w;
  }

  _impulse() {
    let ix = this.irx, iy = this.iry;
    for (let j = 0; j < this.n - 1; j++) { ix += this.Gs[j] * this.mx[j]; iy += this.Gs[j] * this.my[j]; }
    for (let k = 0; k < this.nv; k++) { ix += this.vg[k] * this.vx[k]; iy += this.vg[k] * this.vy[k]; }
    return [ix, iy];
  }
}
