// Head-up display (conformal, orbiter style), flight data panel, control-surface
// diagram and the navigation map.
import * as THREE from 'three';

const D2R = Math.PI / 180;
const _v = new THREE.Vector3(), _p = new THREE.Vector3();
const fmt = (v, d = 0) => (v === undefined || v === null || !isFinite(v) ? '—' : v.toFixed(d));

export class Hud {
  constructor({ canvas, panel, surfCanvas, mapCanvas, vsitCanvas, geometry, world, terrain }) {
    this.vs = vsitCanvas; this.vctx = vsitCanvas ? vsitCanvas.getContext('2d') : null;
    this.profHist = [];
    this.cv = canvas; this.ctx = canvas.getContext('2d');
    this.panel = panel;
    this.surf = surfCanvas; this.sctx = surfCanvas.getContext('2d');
    this.map = mapCanvas; this.mctx = mapCanvas.getContext('2d');
    this.geom = geometry; this.world = world; this.terrain = terrain;
    this.trail = [];
    this._lastPanel = 0;
    this.mapRange = 90000;
    this._buildMapImage();
    this.fields = {};
    panel.querySelectorAll('[data-f]').forEach((el) => { this.fields[el.dataset.f] = el; });
    this.gfields = {};
    panel.querySelectorAll('[data-g]').forEach((el) => { this.gfields[el.dataset.g] = el; });
    this.phaseEls = [...panel.querySelectorAll('#phases span')];
    this.ebar = panel.querySelector('#ebar');
    this._lastGnc = 0;
  }

  resize(w, h, dpr) {
    this.cv.width = Math.round(w * dpr); this.cv.height = Math.round(h * dpr);
    this.cv.style.width = w + 'px'; this.cv.style.height = h + 'px';
    this.dpr = dpr;
  }

  _buildMapImage() {
    const T = this.terrain; if (!T) return;
    const N = 512, img = document.createElement('canvas');
    img.width = N; img.height = N;
    const c = img.getContext('2d');
    const id = c.createImageData(N, N);
    const half = T.extent / 2;
    for (let j = 0; j < N; j++) for (let i = 0; i < N; i++) {
      const e = -half + (i + 0.5) / N * T.extent, n = half - (j + 0.5) / N * T.extent;
      const h = T.heightAt(e, n), hx = T.heightAt(e + 300, n), hy = T.heightAt(e, n + 300);
      const lc = T.landcoverAt(e, n);
      let r, g, b;
      if (lc[3] > 0.05 || h < 0) { r = 22; g = 52; b = 78; }
      else {
        const t = Math.min(1, h / 2600);
        r = 70 + 90 * t; g = 92 + 60 * t; b = 58 + 70 * t;
        r = r * (1 - lc[0] * 0.45); g = g * (1 - lc[0] * 0.3); b = b * (1 - lc[0] * 0.45);
        if (lc[1] > 0.3) { r += 18; g += 12; }
        if (lc[2] > 0.3) { r = g = b = 120; }
        if (h > 2300) { r = g = b = 225; }
      }
      const sh = THREE.MathUtils.clamp(1 + ((hx - h) - (hy - h)) * -0.004, 0.6, 1.4);
      const o = (j * N + i) * 4;
      id.data[o] = r * sh; id.data[o + 1] = g * sh; id.data[o + 2] = b * sh; id.data[o + 3] = 255;
    }
    c.putImageData(id, 0, 0);
    this.mapImg = img;
  }

  pushTrail(pos) {
    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(pos[0] - last[0], pos[1] - last[1]) > 150) this.trail.push([pos[0], pos[1], pos[2]]);
    if (this.trail.length > 3000) this.trail.shift();
  }
  clearTrail() { this.trail = []; this.profHist = []; }
  /** vertical-situation history, fed from every raw simulator state */
  pushProfile(state) {
    const g = state && state.gnc;
    if (!g || !isFinite(g.range_to_go)) return;
    const rtg = g.range_to_go, alt = state.air?.alt ?? state.pos[2];
    if (rtg < 50 && (state.air?.agl ?? alt) > 300) return;          // no plan yet (placeholder range)
    const last = this.profHist[this.profHist.length - 1];
    if (!last || Math.abs(last[0] - rtg) > 120 || Math.abs(last[1] - alt) > 60) this.profHist.push([rtg, alt]);
    if (this.profHist.length > 1500) this.profHist.shift();
  }

  // ------------------------------------------------------------------ cockpit frame (forward windows + glare shield)
  drawCockpit(state) {
    const ctx = this.ctx, W = this.cv.width, H = this.cv.height;
    const frame = '#0b0d10', edge = 'rgba(70,78,88,0.9)';
    ctx.save();
    // window frame: everything except six window panes is opaque structure
    ctx.beginPath();
    ctx.rect(0, 0, W, H);
    const top = H * 0.06, bot = H * 0.70, cx = W / 2;
    const panes = [
      [[cx - W * 0.012, top + H * 0.05], [cx - W * 0.20, top + H * 0.02], [cx - W * 0.235, bot - H * 0.02], [cx - W * 0.012, bot]],
      [[cx - W * 0.225, top + H * 0.035], [cx - W * 0.40, top + H * 0.09], [cx - W * 0.47, bot - H * 0.10], [cx - W * 0.26, bot - H * 0.025]],
      [[cx - W * 0.425, top + H * 0.11], [cx - W * 0.53, top + H * 0.20], [cx - W * 0.56, bot - H * 0.20], [cx - W * 0.495, bot - H * 0.12]],
    ];
    const all = [...panes, ...panes.map((p) => p.map(([x, y]) => [2 * cx - x, y]).reverse())];
    for (const p of all) { ctx.moveTo(p[0][0], p[0][1]); for (let i = 1; i < p.length; i++) ctx.lineTo(p[i][0], p[i][1]); ctx.closePath(); }
    ctx.fillStyle = frame;
    ctx.fill('evenodd');
    // pane edges
    ctx.strokeStyle = edge; ctx.lineWidth = 2;
    for (const p of all) { ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]); for (let i = 1; i < p.length; i++) ctx.lineTo(p[i][0], p[i][1]); ctx.closePath(); ctx.stroke(); }
    // glare shield + instrument panel
    const g = ctx.createLinearGradient(0, bot - H * 0.04, 0, H);
    g.addColorStop(0, '#1a1d22'); g.addColorStop(0.15, '#121418'); g.addColorStop(1, '#07080a');
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.moveTo(0, H); ctx.lineTo(0, bot + H * 0.02);
    ctx.quadraticCurveTo(cx, bot - H * 0.07, W, bot + H * 0.02); ctx.lineTo(W, H); ctx.closePath(); ctx.fill();
    ctx.strokeStyle = 'rgba(120,130,140,0.35)'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(0, bot + H * 0.02); ctx.quadraticCurveTo(cx, bot - H * 0.07, W, bot + H * 0.02); ctx.stroke();
    // multifunction displays
    const a = state?.air || {};
    const mfd = (x, y, w, h, lines) => {
      ctx.fillStyle = '#020403'; ctx.fillRect(x, y, w, h);
      ctx.strokeStyle = '#2a2f36'; ctx.lineWidth = 3; ctx.strokeRect(x, y, w, h);
      ctx.fillStyle = 'rgba(110,255,160,0.85)'; ctx.font = `${Math.round(h * 0.11)}px "JetBrains Mono", Consolas, monospace`;
      lines.forEach((t, i) => ctx.fillText(t, x + w * 0.07, y + h * (0.2 + i * 0.16)));
    };
    const mh = H * 0.17, mw = mh * 1.25, my = H - mh - H * 0.035;
    mfd(cx - mw * 2.2, my, mw, mh, ['ADI', `PITCH ${fmt(a.theta, 1)}`, `ROLL  ${fmt(a.phi, 1)}`, `AOA   ${fmt(a.alpha, 1)}`, `BETA  ${fmt(a.beta, 2)}`]);
    mfd(cx - mw * 0.5, my, mw, mh, ['HSI', `HDG ${fmt(a.psi, 0)}`, `TRK ${fmt(a.track, 0)}`, `RNG ${fmt(this._rng / 1000, 1)} KM`, `ALT ${fmt(a.alt, 0)} M`]);
    mfd(cx + mw * 1.2, my, mw, mh, ['AVVI', `KEAS ${fmt(a.keas, 0)}`, `MACH ${fmt(a.mach, 2)}`, `H-DOT ${fmt(a.vs, 1)}`, `NZ ${fmt(a.nz, 2)}`]);
    ctx.restore();
  }

  // ------------------------------------------------------------------ conformal HUD
  drawHud(camera, state, visible, cockpit = false) {
    const ctx = this.ctx, W = this.cv.width, H = this.cv.height, k = this.dpr || 1;
    ctx.clearRect(0, 0, W, H);
    if (cockpit) this.drawCockpit(state);
    if (!visible || !state) return;
    const proj = (dir) => {
      _p.copy(camera.position).addScaledVector(dir, 1000).project(camera);
      if (_p.z > 1) return null;
      return [(_p.x * 0.5 + 0.5) * W, (1 - (_p.y * 0.5 + 0.5)) * H];
    };
    const up = new THREE.Vector3(camera.position.x / 6371000, 1, camera.position.z / 6371000).normalize();
    const fwd = camera.getWorldDirection(new THREE.Vector3());
    const hdgV = Math.atan2(fwd.x, -fwd.z); // from north (three -Z) clockwise
    const col = 'rgba(120,255,160,0.95)';
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 1.6 * k;
    ctx.shadowColor = 'rgba(60,255,120,0.8)'; ctx.shadowBlur = 6 * k;
    ctx.font = `${12 * k}px "JetBrains Mono", Consolas, monospace`;
    const dirFrom = (hdg, el) => _v.set(Math.sin(hdg) * Math.cos(el), Math.sin(el), -Math.cos(hdg) * Math.cos(el)).clone();
    // pitch ladder
    for (let p = -30; p <= 30; p += 5) {
      const el = p * D2R;
      const a = proj(dirFrom(hdgV - 4.5 * D2R, el)), b = proj(dirFrom(hdgV + 4.5 * D2R, el));
      if (!a || !b) continue;
      const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
      if (my < H * 0.12 || my > H * 0.88) continue;
      const dx = (b[0] - a[0]), dy = (b[1] - a[1]), L = Math.hypot(dx, dy), ux = dx / L, uy = dy / L;
      const gap = p === 0 ? 70 * k : 40 * k, len = p === 0 ? 260 * k : 70 * k;
      ctx.setLineDash(p < 0 ? [6 * k, 5 * k] : []);
      ctx.beginPath();
      ctx.moveTo(mx - ux * (gap + len), my - uy * (gap + len)); ctx.lineTo(mx - ux * gap, my - uy * gap);
      ctx.moveTo(mx + ux * gap, my + uy * gap); ctx.lineTo(mx + ux * (gap + len), my + uy * (gap + len));
      if (p !== 0) {
        const tick = p > 0 ? 8 * k : -8 * k;
        ctx.moveTo(mx - ux * (gap + len), my - uy * (gap + len)); ctx.lineTo(mx - ux * (gap + len) - uy * -tick, my - uy * (gap + len) + ux * -tick);
        ctx.moveTo(mx + ux * (gap + len), my + uy * (gap + len)); ctx.lineTo(mx + ux * (gap + len) - uy * -tick, my + uy * (gap + len) + ux * -tick);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      if (p !== 0) { ctx.fillText(String(Math.abs(p)), mx + ux * (gap + len + 10 * k) - 6 * k, my + uy * (gap + len + 10 * k) + 4 * k); }
    }
    // flight path marker (inertial velocity direction)
    const v = state.vel3;
    if (v && v.lengthSq() > 1) {
      const m = proj(v.clone().normalize());
      if (m) {
        ctx.beginPath(); ctx.arc(m[0], m[1], 7 * k, 0, Math.PI * 2);
        ctx.moveTo(m[0] - 7 * k, m[1]); ctx.lineTo(m[0] - 20 * k, m[1]);
        ctx.moveTo(m[0] + 7 * k, m[1]); ctx.lineTo(m[0] + 20 * k, m[1]);
        ctx.moveTo(m[0], m[1] - 7 * k); ctx.lineTo(m[0], m[1] - 15 * k);
        ctx.stroke();
      }
    }
    // guidance diamond: reference point on the SCP/MPC plan ~8 s ahead
    if (state.guideDir) {
      const m = proj(state.guideDir);
      if (m) {
        ctx.save();
        ctx.strokeStyle = 'rgba(255,210,120,0.95)'; ctx.shadowColor = 'rgba(255,190,80,0.8)';
        ctx.beginPath(); ctx.moveTo(m[0], m[1] - 10 * k); ctx.lineTo(m[0] + 10 * k, m[1]); ctx.lineTo(m[0], m[1] + 10 * k); ctx.lineTo(m[0] - 10 * k, m[1]); ctx.closePath(); ctx.stroke();
        ctx.restore();
      }
    }
    if (state.gnc) {
      ctx.fillText(`${state.gnc.phase || ''}${state.gnc.sub ? ' ' + state.gnc.sub : ''}`.toUpperCase(), W / 2 - 40 * k, H * 0.83);
    }
    // boresight
    ctx.beginPath(); ctx.moveTo(W / 2 - 10 * k, H / 2); ctx.lineTo(W / 2 - 3 * k, H / 2); ctx.moveTo(W / 2 + 3 * k, H / 2); ctx.lineTo(W / 2 + 10 * k, H / 2);
    ctx.moveTo(W / 2, H / 2 - 10 * k); ctx.lineTo(W / 2, H / 2 - 3 * k); ctx.stroke();
    // heading tape
    const hdg = ((hdgV / D2R) % 360 + 360) % 360;
    const tx = W / 2, ty = H * 0.1;
    ctx.beginPath();
    for (let d = -30; d <= 30; d += 5) {
      const hv = Math.round(hdg / 5) * 5 + d;
      const x = tx + (hv - hdg) * 6 * k;
      ctx.moveTo(x, ty); ctx.lineTo(x, ty + (hv % 10 === 0 ? 10 : 5) * k);
      if (hv % 10 === 0) ctx.fillText(String(((hv % 360) + 360) % 360).padStart(3, '0'), x - 10 * k, ty - 4 * k);
    }
    ctx.stroke();
    ctx.strokeRect(tx - 22 * k, ty + 14 * k, 44 * k, 18 * k);
    ctx.fillText(String(Math.round(hdg)).padStart(3, '0'), tx - 12 * k, ty + 28 * k);
    // airspeed / altitude boxes
    const a = state.air || {};
    ctx.font = `${15 * k}px "JetBrains Mono", Consolas, monospace`;
    ctx.strokeRect(W * 0.22 - 50 * k, H / 2 - 14 * k, 100 * k, 28 * k);
    ctx.fillText(`${fmt(a.keas)} KEAS`, W * 0.22 - 44 * k, H / 2 + 6 * k);
    ctx.strokeRect(W * 0.78 - 60 * k, H / 2 - 14 * k, 120 * k, 28 * k);
    const alt = a.agl ?? a.alt;
    ctx.fillText(alt > 3048 ? `${fmt((a.alt || 0) / 0.3048 / 1000, 1)}K FT` : `${fmt(alt / 0.3048)} FT`, W * 0.78 - 52 * k, H / 2 + 6 * k);
    ctx.font = `${12 * k}px "JetBrains Mono", Consolas, monospace`;
    ctx.fillText(`M ${fmt(a.mach, 2)}`, W * 0.22 - 44 * k, H / 2 + 34 * k);
    ctx.fillText(`α ${fmt(a.alpha, 1)}°`, W * 0.22 - 44 * k, H / 2 + 52 * k);
    ctx.fillText(`G ${fmt(a.nz, 2)}`, W * 0.22 - 44 * k, H / 2 + 70 * k);
    ctx.fillText(`VS ${fmt((a.vs || 0) * 196.85)} FPM`, W * 0.78 - 52 * k, H / 2 + 34 * k);
    ctx.fillText(`γ ${fmt(a.gamma, 1)}°`, W * 0.78 - 52 * k, H / 2 + 52 * k);
    if (state.gear && state.gear.pos > 0.95) ctx.fillText('GEAR DN', W / 2 - 28 * k, H * 0.86);
    if (state.surf && state.surf.speedbrake > 1) ctx.fillText(`SB ${fmt(state.surf.speedbrake)}%`.replace('%', '°'), W / 2 - 28 * k, H * 0.89);
    ctx.shadowBlur = 0;
  }

  // ------------------------------------------------------------------ text panel
  updatePanel(s, extra) {
    const now = performance.now();
    if (now - this._lastPanel < 90 || !s) return;
    this._lastPanel = now;
    const a = s.air || {}, f = this.fields;
    const set = (k, v) => { if (f[k]) f[k].textContent = v; };
    set('alt', fmt(a.alt)); set('agl', fmt(a.agl)); set('tas', fmt(a.tas)); set('keas', fmt(a.keas));
    set('mach', fmt(a.mach, 2)); set('qbar', fmt((a.qbar || 0) / 1000, 2)); set('alpha', fmt(a.alpha, 1)); set('beta', fmt(a.beta, 2));
    set('bank', fmt(a.bank, 1)); set('gamma', fmt(a.gamma, 1)); set('hdg', fmt(a.psi ?? a.track, 0)); set('trk', fmt(a.track, 0));
    set('nz', fmt(a.nz, 2)); set('vs', fmt(a.vs, 1)); set('ld', fmt(a.ld, 2));
    this._rng = extra ? extra.range : NaN;
    set('rng', extra && isFinite(extra.range) ? (extra.range / 1000).toFixed(1) : '—');
    set('t', fmt(s.t, 1)); set('status', s.status || '—'); set('mode', s.mode || '—'); set('guid', s.guidance || '—');
    set('gear', s.gear ? (s.gear.pos > 0.98 ? '放下' : s.gear.pos < 0.02 ? '收起' : `${Math.round(s.gear.pos * 100)}%`) : '—');
    const e = Math.max(0, (a.alt || 0) + (a.tas || 0) ** 2 / (2 * 9.81));
    set('energy', (e / 1000).toFixed(1));
  }

  // ------------------------------------------------------------------ GNC panel
  static phaseKey(g) {
    const t = `${g.phase || ''} ${g.sub || ''}`.toUpperCase();
    if (/ROLL|STOP/.test(t)) return 'ROLLOUT';
    if (/FLARE|TOUCH/.test(t)) return 'FLARE';
    if (/OGS|IGS|A\/L|APPR|GLIDE|FINAL\b/.test(t) && !/PREFINAL/.test(t)) return 'OGS';
    if (/HAC/.test(t)) return 'HAC';
    if (/TAEM|ACQ|PREFINAL|S-?TURN|CRUISE|RETURN/.test(t)) return 'TAEM';
    if (/PLAN|INIT/.test(t)) return 'PLAN';
    return '';
  }
  updateGnc(s) {
    const now = performance.now();
    if (now - this._lastGnc < 120 || !s) return;
    this._lastGnc = now;
    const g = s.gnc, f = this.gfields, a = s.air || {}, sf = s.surf || {};
    const set = (k, v) => { if (f[k]) f[k].textContent = v; };
    if (!g) {
      set('phase', s.guidance || '—');
      this.phaseEls.forEach((e) => e.classList.remove('on', 'done'));
      set('aa', fmt(a.alpha, 1)); set('ab', fmt(a.bank, 1)); set('as', fmt(sf.speedbrake, 0));
      return;
    }
    set('phase', g.sub ? `${g.phase} · ${g.sub}` : g.phase);
    set('rwy', g.rwy ? `RWY ${g.rwy}` : '—');
    set('rtg', isFinite(g.range_to_go) ? (g.range_to_go / 1000).toFixed(1) : '—');
    set('ttg', isFinite(g.time_to_go) ? g.time_to_go.toFixed(0) : '—');
    set('eerr', isFinite(g.energy_err) ? `${g.energy_err >= 0 ? '+' : ''}${g.energy_err.toFixed(0)}` : '—');
    if (this.ebar && isFinite(g.energy_err)) this.ebar.style.left = `${50 + Math.max(-1, Math.min(1, g.energy_err / 2500)) * 48}%`;
    const r = g.ref || {};
    set('ra', fmt(r.alpha, 1)); set('rb', fmt(r.bank, 1)); set('rs', fmt(r.sb, 0));
    set('aa', fmt(a.alpha, 1)); set('ab', fmt(a.bank, 1)); set('as', fmt(sf.speedbrake, 0));
    const sc = g.scp || {};
    set('scp', sc.iter !== undefined ? `${sc.converged ? '收敛' : (sc.fallback ? '备用参考' : '未收敛')} · ${sc.iter} 次迭代 · ${fmt(sc.ms, 0)} ms · 缺陷 ${sc.defect !== undefined ? sc.defect.toExponential(1) : '—'} · ${fmt(sc.age, 0)} s 前` : '—');
    const mp = g.mpc || {};
    set('mpc', mp.ms !== undefined ? `${mp.status || ''} · ${fmt(mp.ms, 1)} ms` : '—');
    f.scp?.parentElement.classList.toggle('bad', !!(sc.fallback || (sc.iter !== undefined && !sc.converged)));
    const key = Hud.phaseKey(g);
    const order = ['PLAN', 'TAEM', 'HAC', 'OGS', 'FLARE', 'ROLLOUT'];
    const ki = order.indexOf(key);
    this.phaseEls.forEach((e) => {
      const i = order.indexOf(e.dataset.k);
      e.classList.toggle('on', i === ki);
      e.classList.toggle('done', ki >= 0 && i < ki);
    });
  }

  // ------------------------------------------------------------------ control surface diagram
  drawSurfaces(surf, cmd) {
    const c = this.sctx, W = this.surf.width, H = this.surf.height;
    c.clearRect(0, 0, W, H);
    if (!surf) return;
    const g = this.geom;
    const top = 8, areaH = H - 82;
    const L = g.fuselage.nose_x - (g.bodyflap.hinge_x - g.bodyflap.chord);
    const sc = Math.min((W - 90) / (2 * g.wing.tip_y), areaH / L);
    const ox = W / 2, oy = top;
    const P = (x, y) => [ox + y * sc, oy + (g.fuselage.nose_x - x) * sc];
    c.strokeStyle = 'rgba(200,215,230,0.55)'; c.lineWidth = 1; c.fillStyle = 'rgba(160,180,200,0.07)';
    c.beginPath();
    const le = g.wing.le, te = g.wing.te;
    const pts = [[g.fuselage.nose_x, 0], [le[0][0], le[0][1]], [le[1][0], le[1][1]], [le[2][0], le[2][1]], [te[1][0], te[1][1]], [te[0][0], te[0][1]], [g.bodyflap.hinge_x - g.bodyflap.chord, g.bodyflap.half_span]];
    const all = [...pts, ...pts.slice().reverse().map(([x, y]) => [x, -y])];
    all.forEach(([x, y], i) => { const [px, py] = P(x, y); i ? c.lineTo(px, py) : c.moveTo(px, py); });
    c.closePath(); c.fill(); c.stroke();
    const colFor = (d) => d >= 0 ? `rgba(255,${Math.round(170 - Math.min(d, 20) * 4)},60,0.9)` : `rgba(80,${Math.round(170 + Math.min(-d, 30) * 2)},255,0.9)`;
    c.font = '10px "JetBrains Mono", Consolas, monospace';
    const teX = (y) => te[0][0] + (Math.abs(y) - te[0][1]) * (te[1][0] - te[0][0]) / (te[1][1] - te[0][1]);
    const drawEl = (y0, y1, chord, d, labelRow) => {
      const q = [P(teX(y0) + chord, y0), P(teX(y1) + chord, y1), P(teX(y1), y1), P(teX(y0), y0)];
      c.fillStyle = colFor(d); c.beginPath(); q.forEach(([px, py], i) => (i ? c.lineTo(px, py) : c.moveTo(px, py))); c.closePath(); c.fill();
      // value label outboard of the wing tip, one row per elevon
      const side = Math.sign(y0);
      const [tx, ty] = P(teX(11.9) + 1.5, side * 11.9);
      c.fillStyle = '#dfe8f1'; c.textAlign = side < 0 ? 'right' : 'left';
      c.fillText(`${labelRow}${d >= 0 ? '+' : ''}${d.toFixed(1)}`, tx + side * 6, ty - 14 + (labelRow.startsWith('o') ? 0 : 12));
      c.textAlign = 'left';
    };
    drawEl(-3.05, -7.2, 2.55, surf.elevon_li ?? 0, 'i '); drawEl(-7.3, -11.45, 2.05, surf.elevon_lo ?? 0, 'o ');
    drawEl(3.05, 7.2, 2.55, surf.elevon_ri ?? 0, 'i '); drawEl(7.3, 11.45, 2.05, surf.elevon_ro ?? 0, 'o ');
    const bf = g.bodyflap;
    const b0 = P(bf.hinge_x, -bf.half_span), b1 = P(bf.hinge_x - bf.chord, bf.half_span);
    c.fillStyle = colFor(surf.bodyflap ?? 0); c.fillRect(b0[0], b0[1], b1[0] - b0[0], b1[1] - b0[1]);
    const bar = (y, label, v, lo, hi) => {
      const x0 = 8, w = W - 16;
      c.fillStyle = 'rgba(255,255,255,0.08)'; c.fillRect(x0, y, w, 5);
      const z = x0 + w * (0 - lo) / (hi - lo), p = x0 + w * (v - lo) / (hi - lo);
      c.fillStyle = colFor(v); c.fillRect(Math.min(z, p), y, Math.abs(p - z) || 1, 5);
      c.fillStyle = '#9fb2c6'; c.fillText(`${label} ${v.toFixed(1)}°`, x0, y - 3);
    };
    bar(H - 58, '体襟翼 BF', surf.bodyflap ?? 0, -11.7, 22.5);
    bar(H - 34, '方向舵 RUD', surf.rudder ?? 0, -27.1, 27.1);
    bar(H - 10, '减速板 SB', surf.speedbrake ?? 0, 0, 98.6);
  }

  // ------------------------------------------------------------------ vertical situation (altitude vs range-to-go)
  drawProfile(state, plan) {
    const c = this.vctx; if (!c) return;
    const W = this.vs.width, H = this.vs.height;
    c.clearRect(0, 0, W, H);
    const g = state && state.gnc;
    const alt = state ? state.air?.alt ?? state.pos[2] : 0;
    const rtg = g && isFinite(g.range_to_go) ? g.range_to_go : null;
    // reference profile from the plan: range-to-go along plan + approach
    let ref = [];
    if (plan && plan.nodes && plan.nodes.length > 1) {
      const ap = plan.approach || [];
      let Lapp = 0;
      for (let i = 1; i < ap.length; i++) Lapp += Math.hypot(ap[i][0] - ap[i - 1][0], ap[i][1] - ap[i - 1][1]);
      const appRef = []; let acc = Lapp;
      for (let i = 0; i < ap.length; i++) { if (i) acc -= Math.hypot(ap[i][0] - ap[i - 1][0], ap[i][1] - ap[i - 1][1]); appRef.push([acc, ap[i][2]]); }
      const nd = plan.nodes; const pr = []; let d = Lapp;
      for (let i = nd.length - 1; i >= 0; i--) { if (i < nd.length - 1) d += Math.hypot(nd[i + 1][1] - nd[i][1], nd[i + 1][2] - nd[i][2]); pr.push([d, nd[i][3]]); }
      ref = { plan: pr.reverse(), app: appRef };
    }
    const xs = [rtg ?? 0, ...(ref.plan ? ref.plan.map((p) => p[0]) : []), ...this.profHist.map((p) => p[0])];
    const ys = [alt, ...(ref.plan ? ref.plan.map((p) => p[1]) : []), ...this.profHist.map((p) => p[1])];
    const xmax = Math.max(15000, ...xs) * 1.05, ymax = Math.max(4000, ...ys) * 1.08;
    const pl = 34, pr_ = 8, pt = 16, pb = 18;
    const X = (r) => W - pr_ - (r / xmax) * (W - pl - pr_);
    const Y = (h) => H - pb - (h / ymax) * (H - pt - pb);
    c.font = '10px "JetBrains Mono", Consolas, monospace';
    c.fillStyle = '#9fb2c6'; c.fillText('VERT SIT  高度 / 待飞距离', 8, 11);
    // grid
    c.strokeStyle = 'rgba(160,190,220,0.12)'; c.lineWidth = 1;
    const stepX = xmax > 80000 ? 20000 : xmax > 30000 ? 10000 : 5000, stepY = ymax > 15000 ? 5000 : ymax > 6000 ? 2000 : 1000;
    c.beginPath();
    for (let r = 0; r <= xmax; r += stepX) { c.moveTo(X(r), pt); c.lineTo(X(r), H - pb); }
    for (let h = 0; h <= ymax; h += stepY) { c.moveTo(pl, Y(h)); c.lineTo(W - pr_, Y(h)); }
    c.stroke();
    c.fillStyle = 'rgba(159,178,198,0.8)';
    for (let r = 0; r <= xmax; r += stepX * 2) c.fillText(`${(r / 1000).toFixed(0)}`, X(r) - 6, H - 5);
    for (let h = stepY; h <= ymax; h += stepY * (ymax > 12000 ? 2 : 1)) c.fillText(`${(h / 1000).toFixed(0)}k`, 4, Y(h) + 3);
    const line = (pts, col, w = 1.6, dash = []) => { if (pts.length < 2) return; c.strokeStyle = col; c.lineWidth = w; c.setLineDash(dash); c.beginPath(); pts.forEach(([r, h], i) => (i ? c.lineTo(X(r), Y(h)) : c.moveTo(X(r), Y(h)))); c.stroke(); c.setLineDash([]); };
    if (ref.plan) { line(ref.plan, plan.fallback ? 'rgba(255,190,90,0.9)' : 'rgba(110,220,255,0.95)', 1.8); line(ref.app, 'rgba(255,190,80,0.95)', 1.8); }
    line(this.profHist, 'rgba(255,160,70,0.9)', 1.4);
    if (rtg !== null) {
      c.fillStyle = '#ffd36a'; c.beginPath(); c.arc(X(rtg), Y(alt), 3.5, 0, Math.PI * 2); c.fill();
      c.fillStyle = '#dfe8f1'; c.fillText(`${(alt / 1000).toFixed(1)} km`, Math.min(X(rtg) + 6, W - 60), Y(alt) - 5);
    } else { c.fillStyle = '#7f93a8'; c.fillText('等待制导数据…', W / 2 - 40, H / 2); }
  }

  // ------------------------------------------------------------------ navigation map
  drawMap(pos, track, rwy, plan = null) {
    const c = this.mctx, W = this.map.width, H = this.map.height;
    c.clearRect(0, 0, W, H);
    if (!this.mapImg || !pos) return;
    const T = this.terrain;
    const R = this.mapRange;
    const cx = pos[0] * 0.5, cy = pos[1] * 0.5;
    const s = W / (2 * R);
    const toPx = (e, n) => [W / 2 + (e - cx) * s, H / 2 - (n - cy) * s];
    // terrain image
    const [x0, y0] = toPx(-T.extent / 2, T.extent / 2);
    c.globalAlpha = 0.95;
    c.drawImage(this.mapImg, x0, y0, T.extent * s, T.extent * s);
    c.globalAlpha = 1;
    // runway
    const L = rwy.length, h = rwy.heading_deg * D2R;
    const a = toPx(-Math.sin(h) * L / 2, -Math.cos(h) * L / 2), b = toPx(Math.sin(h) * L / 2, Math.cos(h) * L / 2);
    c.strokeStyle = '#fff'; c.lineWidth = 3; c.beginPath(); c.moveTo(...a); c.lineTo(...b); c.stroke();
    // approach centreline extensions
    c.strokeStyle = 'rgba(255,255,255,0.35)'; c.lineWidth = 1; c.setLineDash([4, 4]); c.beginPath();
    const ext = 20000;
    c.moveTo(...a); c.lineTo(...toPx(-Math.sin(h) * (L / 2 + ext), -Math.cos(h) * (L / 2 + ext)));
    c.moveTo(...b); c.lineTo(...toPx(Math.sin(h) * (L / 2 + ext), Math.cos(h) * (L / 2 + ext)));
    c.stroke(); c.setLineDash([]);
    // planned trajectory, HAC, approach
    if (plan && plan.prev) {
      // approach abandoned by a HAC switch (grey, dashed)
      const pv = plan.prev; c.strokeStyle = 'rgba(190,190,190,0.55)'; c.lineWidth = 1.5; c.setLineDash([5, 4]);
      if (pv.track && pv.track.length > 1) { c.beginPath(); pv.track.forEach((p, i) => { const q = toPx(p[0], p[1]); i ? c.lineTo(...q) : c.moveTo(...q); }); c.stroke(); }
      if (pv.hac) { const [hx, hy] = toPx(pv.hac.center[0], pv.hac.center[1]); c.beginPath(); c.arc(hx, hy, pv.hac.radius * s, 0, Math.PI * 2); c.stroke(); }
      if (pv.approach && pv.approach.length > 1) { c.beginPath(); pv.approach.forEach((p, i) => { const q = toPx(p[0], p[1]); i ? c.lineTo(...q) : c.moveTo(...q); }); c.stroke(); }
      c.setLineDash([]);
    }
    if (plan && plan.pts && plan.pts.length > 1) {
      c.strokeStyle = plan.fallback ? 'rgba(255,190,90,0.9)' : 'rgba(110,220,255,0.95)'; c.lineWidth = 2; c.setLineDash([]);
      c.beginPath(); plan.pts.forEach((p, i) => { const q = toPx(p[0], p[1]); i ? c.lineTo(...q) : c.moveTo(...q); }); c.stroke();
      if (plan.hac) { const [hx, hy] = toPx(plan.hac.center[0], plan.hac.center[1]); c.strokeStyle = 'rgba(255,120,240,0.8)'; c.setLineDash([3, 3]); c.beginPath(); c.arc(hx, hy, plan.hac.radius * s, 0, Math.PI * 2); c.stroke(); c.setLineDash([]); }
      if (plan.approach) { c.strokeStyle = 'rgba(255,190,80,0.95)'; c.lineWidth = 2; c.beginPath(); plan.approach.forEach((p, i) => { const q = toPx(p[0], p[1]); i ? c.lineTo(...q) : c.moveTo(...q); }); c.stroke(); }
      if (plan.ali) { const [ax, ay] = toPx(plan.ali[0], plan.ali[1]); c.fillStyle = '#ffcf6a'; c.fillRect(ax - 3, ay - 3, 6, 6); }
    }
    if (plan && plan.mpc && plan.mpc.length > 1) {
      c.strokeStyle = 'rgba(120,255,150,0.95)'; c.lineWidth = 2.5; c.beginPath();
      plan.mpc.forEach((p, i) => { const q = toPx(p[0], p[1]); i ? c.lineTo(...q) : c.moveTo(...q); }); c.stroke();
    }
    // trail
    if (this.trail.length > 1) {
      c.strokeStyle = 'rgba(255,190,90,0.9)'; c.lineWidth = 1.6; c.beginPath();
      this.trail.forEach((p, i) => { const q = toPx(p[0], p[1]); i ? c.lineTo(...q) : c.moveTo(...q); });
      c.lineTo(...toPx(pos[0], pos[1]));
      c.stroke();
    }
    // orbiter icon
    const [px, py] = toPx(pos[0], pos[1]);
    c.save(); c.translate(px, py); c.rotate(track * D2R);
    c.fillStyle = '#ffd36a'; c.strokeStyle = '#000'; c.lineWidth = 1;
    c.beginPath(); c.moveTo(0, -9); c.lineTo(6, 6); c.lineTo(0, 3); c.lineTo(-6, 6); c.closePath(); c.fill(); c.stroke();
    c.restore();
    // scale bar
    c.fillStyle = 'rgba(0,0,0,0.5)'; c.fillRect(8, H - 22, 92, 16);
    c.strokeStyle = '#fff'; c.beginPath(); c.moveTo(12, H - 10); c.lineTo(12 + 20000 * s, H - 10); c.stroke();
    c.fillStyle = '#fff'; c.font = '10px "JetBrains Mono", Consolas, monospace'; c.fillText('20 km', 16 + 20000 * s, H - 9);
    c.fillText('N', W - 14, 14);
  }
}
