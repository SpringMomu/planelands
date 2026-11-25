// MEDS (Multifunction Electronic Display Subsystem) glass-cockpit units.
//
// Free-floating, draggable, resizable and collapsible MDU windows with a working bezel: six edge keys
// driving the MEDS menu tree, panel toggle switches (ADI ATTITUDE / ERROR / RATE, map orientation, MDU
// power), detented and continuous rotary knobs (brightness, map range), a power-on self test, and
// pages modelled on the orbiter displays:
//   A/E PFD    ascent/entry primary flight display: ADI 8-ball with rate scales and error needles,
//              α / M-VR(KEAS) / H / Ḣ tapes, Nz accelerometer, HSI with bearing pointers and course
//              deviation, glide-slope indicator, guidance mode annunciation
//   HSIT       graphic horizontal situation (moving map): runway, HAC, SCP plan, MPC prediction, trail,
//              turn predictor, wind, range rings
//   SPI        surface position indicator: elevons, body flap, rudder, aileron, speedbrake act/cmd
//   HORIZ SIT, VERT SIT, GNC SUMM   DPS pages (51 x 26 character grid, green, 1 Hz like the GPC)
// Update rates follow the avionics rather than the browser: the flight instruments (A/E PFD, SPI)
// are redrawn at 12.5 Hz (every second 25 Hz GPC flight-control cycle), the HSIT moving map at 5 Hz,
// the DPS pages once per second, and digital readouts are sampled and held at 4 Hz so the numbers
// are readable instead of flickering every frame.
//   MDU STATUS MEDS maintenance page
// The orbiter cockpit uses feet, knots and nautical miles, so the displays do too.

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const FT = 3.28084, KT = 1.943844, NM = 1 / 1852;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const fin = (v) => typeof v === 'number' && isFinite(v);
const wrap360 = (a) => ((a % 360) + 360) % 360;
const wrap180 = (a) => { a = wrap360(a); return a > 180 ? a - 360 : a; };
const pad = (n, w, c = '0') => String(n).padStart(w, c);
const LS_KEY = 'planelands.meds.v1';

const SANS = '"Arial Narrow", "Roboto Condensed", "Helvetica Neue", Arial, sans-serif';
const MONO = 'Consolas, "Lucida Console", "JetBrains Mono", "Courier New", monospace';
const COL = {
  white: '#f0f0ee', gray: '#a2a4a6', dgray: '#55585b', mag: '#ff4cf5', cyan: '#45e0ff', green: '#3dff6e',
  yellow: '#ffe04a', amber: '#ffae1a', red: '#ff3b30', dps: '#40ff7c', dpsHi: '#c8ffd6', dpsDim: '#1f9e48',
};
const RANGES = [5, 10, 20, 40, 80, 160];

// ------------------------------------------------------------------ stroke font
// Avionics symbol generators do not use typefaces: every character is a short list of vectors that
// the display generator strokes with the same pen as the symbology (the orbiter's MCDS was a stroke
// writer; EFIS / MEDS character sets keep the same look).  Glyphs live on an 8 x 12 grid (y up from
// the baseline), are drawn condensed (0.6 horizontal scale) with round caps and joins, upper case
// only (lower case is drawn as small capitals).  Pen width follows the weight; 700 is the normal
// display weight.  `▲ ▼` are filled.
const GLYPH_SRC = {
  '0': '2,0 6,0 8,2 8,10 6,12 2,12 0,10 0,2 2,0', '1': '1.5,9.5 4,12 4,0;1.5,0 6.5,0',
  '2': '0,10 2,12 6,12 8,10 8,8 0,1 0,0 8,0', '3': '0,10 2,12 6,12 8,10 8,8 6,6 3,6;6,6 8,4 8,2 6,0 2,0 0,2',
  '4': '6,0 6,12 0,4 0,3 8,3', '5': '8,12 1,12 0,7 2,8 6,8 8,6 8,2 6,0 2,0 0,2',
  '6': '7,12 4,12 1,9 0,6 0,2 2,0 6,0 8,2 8,5 6,7 2,7 0,5', '7': '0,12 8,12 8,11 3,0',
  '8': '2,6 0,8 0,10 2,12 6,12 8,10 8,8 6,6 2,6 0,4 0,2 2,0 6,0 8,2 8,4 6,6', '9': '8,7 6,5 2,5 0,7 0,10 2,12 6,12 8,10 8,6 7,3 4,0 1,0',
  A: '0,0 0,8 4,12 8,8 8,0;0,5 8,5', B: '0,0 0,12 6,12 8,10 8,8 6,6 0,6;6,6 8,4 8,2 6,0 0,0', C: '8,10 6,12 2,12 0,10 0,2 2,0 6,0 8,2',
  D: '0,0 0,12 5,12 8,9 8,3 5,0 0,0', E: '8,12 0,12 0,0 8,0;0,6 5.5,6', F: '8,12 0,12 0,0;0,6 5.5,6',
  G: '8,10 6,12 2,12 0,10 0,2 2,0 6,0 8,2 8,6 4.5,6', H: '0,0 0,12;8,0 8,12;0,6 8,6', I: '4,0 4,12;2,12 6,12;2,0 6,0',
  J: '8,12 8,2 6,0 2,0 0,2 0,4', K: '0,0 0,12;8,12 0,4;2.7,6.7 8,0', L: '0,12 0,0 8,0', M: '0,0 0,12 4,6 8,12 8,0',
  N: '0,0 0,12 8,0 8,12', O: '2,0 6,0 8,2 8,10 6,12 2,12 0,10 0,2 2,0', P: '0,0 0,12 6,12 8,10 8,8 6,6 0,6',
  Q: '2,0 6,0 8,2 8,10 6,12 2,12 0,10 0,2 2,0;5,3 8.5,-0.5', R: '0,0 0,12 6,12 8,10 8,8 6,6 0,6;4,6 8,0',
  S: '8,10 6,12 2,12 0,10 0,8 2,6 6,6 8,4 8,2 6,0 2,0 0,2', T: '0,12 8,12;4,12 4,0', U: '0,12 0,2 2,0 6,0 8,2 8,12',
  V: '0,12 4,0 8,12', W: '0,12 1.5,0 4,7 6.5,0 8,12', X: '0,0 8,12;0,12 8,0', Y: '0,12 4,6 8,12;4,6 4,0', Z: '0,12 8,12 0,0 8,0',
  '.': ['1,0 2,0 2,1 1,1 1,0', 3], ':': ['1,1 2,1 2,2 1,2 1,1;1,7 2,7 2,8 1,8 1,7', 3], ',': ['2,1 2,0 0.5,-2', 3],
  "'": ['1,12 1,9', 2], '-': '1,6 7,6', '_': '0,-1.5 8,-1.5', '+': '0,6 8,6;4,2 4,10', '=': '0,4 8,4;0,8 8,8',
  '/': '0,0 8,12', '|': ['1,-1 1,13', 2], '(': ['4,12 2,10 2,2 4,0', 5], ')': ['1,12 3,10 3,2 1,0', 5],
  '[': ['4,12 1,12 1,0 4,0', 5], ']': ['1,12 4,12 4,0 1,0', 5], '<': '8,12 0,6 8,0', '>': '0,12 8,6 0,0',
  '*': '4,2 4,10;0.5,8 7.5,4;0.5,4 7.5,8', '?': '0,10 2,12 6,12 8,10 8,8 4,5 4,3;4,0.6 4,0', '!': ['1,12 1,3;1,0.6 1,0', 2],
  '%': '0,12 2.5,12 2.5,9 0,9 0,12;5.5,3 8,3 8,0 5.5,0 5.5,3;8,12 0,0', '&': '8,0 1,9 1,11 2,12 4,12 5,11 5,9 0,4 0,2 2,0 5,0 8,4',
  '#': '2,0 3,12;5,0 6,12;0,4 8,4;0,8 8,8', '°': ['1,12 3,12 4,11 4,9 3,8 1,8 0,9 0,11 1,12', 5], '×': '1,3 7,9;1,9 7,3',
  'Δ': '0,0 4,12 8,0 0,0', 'α': '8,8 5,1 3,0 1,0 0,2 0,6 2,8 4,8 6,4 8,0', 'β': '0,-3 0,10 2,12 5,12 7,10 7,8 5,6 2,6;5,6 8,4 8,2 6,0 2,0 0,2',
  '↑': '4,0 4,12;1,9 4,12 7,9', '↓': '4,12 4,0;1,3 4,0 7,3', '→': '0,6 8,6;5,9 8,6 5,3', '←': '8,6 0,6;3,9 0,6 3,3',
  'Ḣ': '0,0 0,12;8,0 8,12;0,6 8,6;3.5,14.5 4.5,14.5 4.5,15.5 3.5,15.5 3.5,14.5', '▲': ['0,0 14,0 7,10 0,0', 14, true], '▼': ['0,11 14,11 7,1 0,11', 14, true],
};
const GLYPH = {};
for (const [k, v] of Object.entries(GLYPH_SRC)) {
  const [src, w, fill] = Array.isArray(v) ? v : [v, 8, false];
  GLYPH[k] = { w, fill, s: src.split(';').map((st) => st.trim().split(/\s+/).map((p) => p.split(',').map(Number))) };
}
const SF_X = 0.6, SF_GAP = 3.0;                   // horizontal condensation, inter-character gap (grid units)
function sfGlyph(ch) {
  const gl = GLYPH[ch]; if (gl) return [gl, 1];
  const up = GLYPH[ch.toUpperCase()]; if (up) return [up, 0.8];       // lower case: small capitals
  return [null, 1];
}
/** advance width (px) of a string in the stroke font at `size` (nominal em, cap height 0.72 em) */
function textW(s, size) {
  const u = size * 0.72 / 12; let w = 0; s = String(s);
  for (const ch of s) { const [gl, k] = sfGlyph(ch); w += ((gl ? gl.w * k : 8) * SF_X + SF_GAP) * u; }
  return Math.max(0, w - SF_GAP * u);
}
/** stroke-font text; halo: black outline under the characters (labels drawn over map symbology) */
function strokeText(g, s, x, y, size, color, align = 'left', weight = '700', base = 'alphabetic', halo = 0) {
  s = String(s); if (!s) return;
  const capH = size * 0.72, u = capH / 12, W = textW(s, size);
  let x0 = align === 'center' ? x - W / 2 : align === 'right' ? x - W : x;
  if (base === 'middle') y += capH / 2; else if (base === 'top' || base === 'hanging') y += capH;
  const lw = String(weight) === '700' ? Math.max(1, capH * 0.145) : Math.max(0.85, capH * 0.11);
  const path = new Path2D(), fills = new Path2D(); let anyFill = false;
  for (const ch of s) {
    const [gl, k] = sfGlyph(ch);
    if (gl) {
      const P = gl.fill ? (anyFill = true, fills) : path;
      for (const st of gl.s) st.forEach(([px, py], i) => P[i ? 'lineTo' : 'moveTo'](x0 + px * k * SF_X * u, y - py * k * u));
    }
    x0 += ((gl ? gl.w * k : 8) * SF_X + SF_GAP) * u;
  }
  g.save(); g.lineCap = 'round'; g.lineJoin = 'round'; g.setLineDash([]);
  if (halo) { g.strokeStyle = '#000'; g.lineWidth = lw + halo; g.stroke(path); if (anyFill) { g.fillStyle = '#000'; g.fill(fills); g.stroke(fills); } }
  g.strokeStyle = color; g.lineWidth = lw; g.stroke(path);
  if (anyFill) { g.fillStyle = color; g.fill(fills); g.stroke(fills); }
  g.restore();
}

// ------------------------------------------------------------------ small canvas helpers
function T(g, s, x, y, size = 12, color = COL.white, align = 'left', weight = '', font = SANS, base = 'alphabetic') {
  strokeText(g, s, x, y, size, color, align, weight === '' ? '400' : weight, base);
}
function line(g, x0, y0, x1, y1) { g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1, y1); g.stroke(); }
function tri(g, x, y, s, dir, fill, stroke = null) {
  g.beginPath();
  if (dir === 'down') { g.moveTo(x, y); g.lineTo(x - s * 0.6, y - s); g.lineTo(x + s * 0.6, y - s); }
  else if (dir === 'up') { g.moveTo(x, y); g.lineTo(x - s * 0.6, y + s); g.lineTo(x + s * 0.6, y + s); }
  else if (dir === 'left') { g.moveTo(x, y); g.lineTo(x + s, y - s * 0.6); g.lineTo(x + s, y + s * 0.6); }
  else { g.moveTo(x, y); g.lineTo(x - s, y - s * 0.6); g.lineTo(x - s, y + s * 0.6); }
  g.closePath();
  if (fill) { g.fillStyle = fill; g.fill(); }
  if (stroke) { g.strokeStyle = stroke; g.lineWidth = 1; g.stroke(); }
}
function dbox(g, x, y, w, h, stroke = COL.white, lw = 1.2) {
  g.fillStyle = '#000'; g.fillRect(x, y, w, h);
  g.strokeStyle = stroke; g.lineWidth = lw; g.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
}
function offFlag(g, x, y, w = 34, h = 16) {
  g.fillStyle = '#c4161c'; g.fillRect(x - w / 2, y - h / 2, w, h);
  g.strokeStyle = '#000'; g.lineWidth = 1; g.strokeRect(x - w / 2 + 0.5, y - h / 2 + 0.5, w - 1, h - 1);
  T(g, 'OFF', x, y + 0.5, 12, '#fff', 'center', '700', SANS, 'middle');
}
function fmtFtK(ft) {            // AVVI altitude: K format above 10 000 ft
  if (!fin(ft)) return '----';
  return Math.abs(ft) >= 10000 ? `${(ft / 1000).toFixed(1)}K` : Math.abs(ft) >= 1000 ? `${Math.round(ft / 10) * 10}` : `${Math.round(ft)}`;
}
function mmss(s) { if (!fin(s)) return '--:--'; s = Math.max(0, Math.round(s)); return `${pad(Math.floor(s / 60), 2)}:${pad(s % 60, 2)}`; }

// ------------------------------------------------------------------ attitude maths (NED, 3-2-1)
function dcm(phi, th, psi) {      // NED -> body
  const cf = Math.cos(phi), sf = Math.sin(phi), ct = Math.cos(th), st = Math.sin(th), cp = Math.cos(psi), sp = Math.sin(psi);
  return [[ct * cp, ct * sp, -st], [sf * st * cp - cf * sp, sf * st * sp + cf * cp, sf * ct], [cf * st * cp + sf * sp, cf * st * sp - sf * cp, cf * ct]];
}
const mT = (A) => [[A[0][0], A[1][0], A[2][0]], [A[0][1], A[1][1], A[2][1]], [A[0][2], A[1][2], A[2][2]]];
const mM = (A, B) => A.map((r) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
const euler = (C) => ({ th: Math.asin(clamp(-C[0][2], -1, 1)), phi: Math.atan2(C[1][2], C[2][2]), psi: Math.atan2(C[0][1], C[0][0]) });

// ------------------------------------------------------------------ geometry from the shared world
function runwayInfo(rw, gnc, pos) {
  const h = rw.heading_deg * D2R, L = rw.length, c = rw.center_en || [0, 0];
  const u = [Math.sin(h), Math.cos(h)];
  const thr33 = [c[0] - u[0] * L / 2, c[1] - u[1] * L / 2], thr15 = [c[0] + u[0] * L / 2, c[1] + u[1] * L / 2];
  let end = gnc && gnc.rwy ? String(gnc.rwy) : null;
  if (!end && pos) end = Math.hypot(pos[0] - thr33[0], pos[1] - thr33[1]) < Math.hypot(pos[0] - thr15[0], pos[1] - thr15[1]) ? '33' : '15';
  end = end || '33';
  const is33 = end === '33';
  return { end, other: is33 ? '15' : '33', thr: is33 ? thr33 : thr15, far: is33 ? thr15 : thr33, crs: is33 ? rw.heading_deg : wrap360(rw.heading_deg + 180), thr33, thr15, L, u, c, name: rw.name || '' };
}
/** tangency point on the HAC for a vehicle at p (dir +1: right turn, circle on the right) */
function hacTangent(p, hac) {
  if (!hac || !p) return null;
  const dx = p[0] - hac.center[0], dy = p[1] - hac.center[1], d = Math.hypot(dx, dy), R = hac.radius;
  if (d <= R * 1.001) return null;
  const b = Math.acos(R / d), a0 = Math.atan2(dy, dx);
  for (const s of [1, -1]) {
    const a = a0 + s * b, T0 = [hac.center[0] + R * Math.cos(a), hac.center[1] + R * Math.sin(a)];
    const tx = T0[0] - p[0], ty = T0[1] - p[1], cx = hac.center[0] - T0[0], cy = hac.center[1] - T0[1];
    const cross = tx * cy - ty * cx;                     // <0: centre to the right of the tangent course
    if ((hac.dir > 0 && cross < 0) || (hac.dir < 0 && cross > 0)) return T0;
  }
  return null;
}
const brgTo = (p, q) => wrap360(Math.atan2(q[0] - p[0], q[1] - p[1]) * R2D);

/** guidance mode annunciation in orbiter terms */
function guidMode(g) {
  if (!g) return ['', ''];
  const ph = g.phase || '', sub = g.sub || '';
  if (ph === 'PLAN') return ['TAEM', 'INIT'];
  if (ph === 'TAEM') return ['TAEM', { ACQ: 'ACQ', HAC: 'HDG', PRF: 'PRFNL', MAXRANGE: 'MXRNG', HOLD: 'HOLD', WAIT: 'INIT' }[sub] || sub];
  if (ph === 'A/L') return ['A/L', { OGS: 'OGS', PREFLARE: 'FLARE', IGS: 'FLARE', FLARE: 'FNLFL', ROLLOUT: 'ROLL' }[sub] || sub];
  if (ph === 'TOUCHDOWN') return ['A/L', 'DEROT'];
  if (ph === 'ROLLOUT') return ['A/L', 'ROLL'];
  if (ph === 'STOPPED') return ['A/L', 'STOP'];
  if (ph === 'CRASHED') return ['', 'CRASH'];
  return [ph, sub];
}
const isAL = (g) => g && /A\/L|TOUCH|ROLL|STOP/.test(g.phase || '');

// ------------------------------------------------------------------ menus and pages
const UP = { l: 'UP', up: true };
const MENUS = {
  MAIN: { title: 'MAIN MENU', keys: [null, { l: 'FLT\nINST', menu: 'FLT' }, { l: 'SUBSYS\nSTATUS', menu: 'SUB' }, { l: 'DPS', menu: 'DPS' }, { l: 'MEDS\nMAINT', menu: 'MAINT' }, null] },
  FLT: { title: 'FLT INST', up: 'MAIN', keys: [UP, { l: 'A/E\nPFD', page: 'PFD' }, { l: 'HSIT', page: 'HSIT', menu: 'HSIT' }, null, null, null] },
  HSIT: { title: 'HSIT', up: 'FLT', keys: [UP, { l: 'RNG\n▲', act: 'rngUp' }, { l: 'RNG\n▼', act: 'rngDn' },
    { l: (w) => (w.st.sw.orient ? 'N UP' : 'HDG\nUP'), act: 'orient' }, { l: 'PLAN', act: 'plan', on: (w) => w.st.plan }, { l: 'TRAIL', act: 'trail', on: (w) => w.st.trail }] },
  SUB: { title: 'SUBSYS STATUS', up: 'MAIN', keys: [UP, { l: 'SPI', page: 'SPI' }, null, null, null, null] },
  DPS: { title: 'DPS', up: 'MAIN', keys: [UP, { l: 'HORIZ\nSIT', page: 'HZS' }, { l: 'VERT\nSIT', page: 'VSIT' }, { l: 'GNC\nSUMM', page: 'GSUM' }, null, { l: 'MSG\nRESET', act: 'msgrst' }] },
  MAINT: { title: 'MEDS MAINT', up: 'MAIN', keys: [UP, { l: 'MDU\nSTATUS', page: 'MAINT' }, { l: 'SELF\nTEST', act: 'bit' }, null, null, null] },
};

// ------------------------------------------------------------------ audio: switch / key / detent clicks
class Clicks {
  constructor() { this.ctx = null; }
  _ok() {
    const s = document.querySelector('#sound');
    if (s && !s.checked) return false;
    if (!this.ctx) {
      try { this.ctx = new (window.AudioContext || window.webkitAudioContext)(); } catch { return false; }
      const n = this.ctx.sampleRate * 0.25, b = this.ctx.createBuffer(1, n, this.ctx.sampleRate), d = b.getChannelData(0);
      for (let i = 0; i < n; i++) d[i] = Math.random() * 2 - 1;
      this.noise = b;
    }
    if (this.ctx.state === 'suspended') this.ctx.resume();
    return true;
  }
  _burst(t, f, q, dur, gain) {
    const c = this.ctx, s = c.createBufferSource(), bp = c.createBiquadFilter(), gn = c.createGain();
    s.buffer = this.noise; bp.type = 'bandpass'; bp.frequency.value = f; bp.Q.value = q;
    gn.gain.setValueAtTime(0, t); gn.gain.linearRampToValueAtTime(gain, t + 0.0015); gn.gain.exponentialRampToValueAtTime(0.0005, t + dur);
    s.connect(bp).connect(gn).connect(c.destination); s.start(t, Math.random() * 0.2); s.stop(t + dur + 0.02);
  }
  _thump(t, f, dur, gain) {
    const c = this.ctx, o = c.createOscillator(), gn = c.createGain();
    o.frequency.setValueAtTime(f, t); o.frequency.exponentialRampToValueAtTime(f * 0.5, t + dur);
    gn.gain.setValueAtTime(gain, t); gn.gain.exponentialRampToValueAtTime(0.0005, t + dur);
    o.connect(gn).connect(c.destination); o.start(t); o.stop(t + dur + 0.02);
  }
  play(kind) {
    if (!this._ok()) return;
    const t = this.ctx.currentTime + 0.005;
    if (kind === 'toggle') { this._burst(t, 3200, 1.4, 0.03, 0.55); this._burst(t + 0.012, 1500, 2, 0.05, 0.4); this._thump(t, 180, 0.05, 0.25); }
    else if (kind === 'key') { this._burst(t, 2600, 2.5, 0.018, 0.28); this._thump(t, 240, 0.03, 0.12); this._burst(t + 0.06, 2000, 3, 0.012, 0.12); }
    else if (kind === 'detent') this._burst(t, 4200, 3, 0.012, 0.22);
    else if (kind === 'power') { this._burst(t, 900, 1, 0.06, 0.5); this._thump(t, 95, 0.12, 0.35); }
    else if (kind === 'wbtn') this._burst(t, 3000, 2, 0.015, 0.15);
  }
}

// ------------------------------------------------------------------ DPS character screen (51 x 26)
// rows 0..23 are clear of the edge-key legends (separator at y 474); page content must end above the
// fault message row, graphics above y ~446
const DPS_FAULT_ROW = 23;
class Dps {
  constructor() { this.cells = []; this.clear(); }
  clear() { this.cells = Array.from({ length: 26 }, () => Array(51).fill(null)); }
  put(r, c, s, a = 0) {        // a: 0 normal, 1 overbright, 2 reverse video, 3 dim
    s = String(s);
    for (let i = 0; i < s.length; i++) { const cc = c + i; if (r >= 0 && r < 26 && cc >= 0 && cc < 51) this.cells[r][cc] = s[i] === ' ' && a !== 2 ? null : [s[i], a]; }
  }
  draw(g) {
    const cw = 512 / 51, rh = 512 / 26.4;
    const fs = rh * 0.82;
    for (let r = 0; r < 26; r++) for (let c = 0; c < 51; c++) {
      const e = this.cells[r][c]; if (!e) continue;
      const x = (c + 0.5) * cw, y = (r + 0.86) * rh + 3;
      if (e[1] === 2) { g.fillStyle = COL.dps; g.fillRect(c * cw, r * rh + 3, cw + 0.3, rh); g.fillStyle = '#000'; }
      else g.fillStyle = e[1] === 1 ? COL.dpsHi : e[1] === 3 ? COL.dpsDim : COL.dps;
      if (e[0] !== ' ') strokeText(g, e[0], x, y, fs, g.fillStyle, 'center', e[1] === 1 ? '700' : '400');
    }
  }
  static cell(c, r) { return [(c + 0.5) * 512 / 51, (r + 0.5) * 512 / 26.4 + 3]; }
}

// =========================================================================================== pages
const PAGES = {
  PFD: { title: 'A/E PFD', hz: 12.5, draw: drawPFD },
  HSIT: { title: 'HSIT', hz: 6.25, draw: drawHSIT },
  SPI: { title: 'SPI', hz: 12.5, draw: drawSPI },
  HZS: { title: 'HORIZ SIT', hz: 1, draw: drawHorizSit, dps: true },
  VSIT: { title: 'VERT SIT', hz: 1, draw: drawVertSit, dps: true },
  GSUM: { title: 'GNC SUMM', hz: 1, draw: drawGncSumm, dps: true },
  MAINT: { title: 'MDU STATUS', hz: 2, draw: drawMaint },
};

// ------------------------------------------------------------------ edge-key legends (bottom of every display)
// MEDS menu area as flown (USA005512 fig. 3-7): cyan rule, cyan menu title under it, each legend in a
// cyan box above its bezel key, the UP key as an up-arrow outline.  A selected page / function shows
// a double box; a pressed key is reverse video until the IDP has processed it.
function drawKeyLegends(g, w) {
  const m = MENUS[w.st.menu] || MENUS.MAIN;
  const y0 = 470;
  g.fillStyle = '#000'; g.fillRect(0, y0, 512, 512 - y0);
  g.strokeStyle = COL.cyan; g.lineWidth = 1; line(g, 4, y0 + 0.5, 508, y0 + 0.5);
  T(g, m.title, 256, y0 + 11, 10.5, COL.cyan, 'center', '700');
  const cw = 512 / 6, bw = 70, bh = 25, by = 485;
  m.keys.forEach((k, i) => {
    if (!k) return;
    const lab = typeof k.l === 'function' ? k.l(w) : k.l;
    const x = Math.round((i + 0.5) * cw);
    const sel = (k.page && k.page === w.st.page) || (k.on && k.on(w));
    const flash = w.keyFlash[i] && performance.now() - w.keyFlash[i] < 140;
    const lines = lab.split('\n');
    g.strokeStyle = COL.cyan; g.lineWidth = 1.2;
    if (k.up) {
      const path = () => { g.beginPath(); g.moveTo(x, by); g.lineTo(x + 24, by + 12); g.lineTo(x + 12, by + 12); g.lineTo(x + 12, by + bh); g.lineTo(x - 12, by + bh); g.lineTo(x - 12, by + 12); g.lineTo(x - 24, by + 12); g.closePath(); };
      path(); if (flash) { g.fillStyle = COL.cyan; g.fill(); } g.stroke();
      T(g, 'UP', x, by + 21, 10.5, flash ? '#000' : COL.cyan, 'center', '700');
      return;
    }
    const x0 = x - bw / 2 + 0.5, y1 = by + 0.5;
    if (flash) { g.fillStyle = COL.cyan; g.fillRect(x0 - 0.5, by, bw, bh); }
    g.strokeRect(x0, y1, bw - 1, bh - 1);
    if (sel && !flash) g.strokeRect(x0 + 2.5, y1 + 2.5, bw - 6, bh - 6);
    const col = flash ? '#000' : COL.cyan;
    if (lines.length === 1) T(g, lines[0], x, by + 17, 11, col, 'center', '700');
    else { T(g, lines[0], x, by + 11, 10, col, 'center', '700'); T(g, lines[1], x, by + 21.5, 10, col, 'center', '700'); }
  });
}

// ------------------------------------------------------------------ screen base: LCD backlight bleed
function drawBacklight(g) {
  g.fillStyle = '#030405'; g.fillRect(0, 0, 512, 512);
  const r = g.createRadialGradient(256, 236, 60, 256, 256, 380);
  r.addColorStop(0, 'rgba(14,18,22,0)'); r.addColorStop(0.75, 'rgba(14,18,22,0.35)'); r.addColorStop(1, 'rgba(26,32,38,0.75)');
  g.fillStyle = r; g.fillRect(0, 0, 512, 512);
  // slightly uneven edge-lit backlight: brighter bands near the LED strips (top / bottom)
  const b = g.createLinearGradient(0, 0, 0, 512);
  b.addColorStop(0, 'rgba(30,36,44,0.30)'); b.addColorStop(0.08, 'rgba(30,36,44,0)'); b.addColorStop(0.92, 'rgba(30,36,44,0)'); b.addColorStop(1, 'rgba(30,36,44,0.28)');
  g.fillStyle = b; g.fillRect(0, 0, 512, 512);
}

// =========================================================================================== A/E PFD
const RATE_FS = [10, 5, 1];                                  // ADI RATE HIGH / MED / LOW  (deg/s)
const ERR_FS = [{ r: 25, p: 10, y: 10 }, { r: 10, p: 5, y: 5 }, { r: 5, p: 2, y: 2 }];   // ADI ERROR HIGH / MED / LOW (deg)

function drawPFD(g, w, d) {
  const st = d.st, a = st?.air, valid = !!a && d.link !== 'offline';
  const gnc = st?.gnc;
  const auto = (st?.mode || 'AUTO') !== 'MANUAL';
  // ---------------- header
  T(g, `Pitch: ${auto ? 'Auto' : 'CSS'}`, 8, 17, 13);
  T(g, `R/Y: ${auto ? 'Auto' : 'CSS'}`, 8, 33, 13);
  T(g, 'MM: 305', 504, 17, 13, COL.white, 'right');
  T(g, `ATT: ${['Inrtl', 'LVLH', 'Ref'][w.st.sw.att]}`, 504, 33, 13, COL.white, 'right');
  const [m1, m2] = guidMode(gnc);
  const modeStr = `${m1} ${m2}`;
  if (modeStr !== w.lastMode) { w.lastMode = modeStr; w.modeT = performance.now(); }
  const crashed = gnc && gnc.phase === 'CRASHED';
  if (m1 || m2) {
    T(g, m1, 214, 19, 14, COL.white, 'right', '700');
    const flash = performance.now() - w.modeT < 5000;
    const mw = textW(m2, 14);
    T(g, m2, 222, 19, 14, crashed ? COL.red : COL.white, 'left', '700');
    if (flash && Math.floor((performance.now() - w.modeT) / 500) % 2 === 0) { g.strokeStyle = COL.white; g.lineWidth = 1.2; g.strokeRect(218.5, 5.5, mw + 8, 18); }
  }
  // status annunciators (yellow / amber / red caution text)
  const ann = [];
  if (st?.paused) ann.push(['SIM HOLD', COL.yellow]);
  if (d.link === 'local') ann.push(['NAV: PREVIEW', COL.amber]);
  if (gnc && gnc.reachable === false) ann.push([gnc.energy_high ? 'ENERGY HIGH' : gnc.energy === 'turn' ? 'TURN LIMIT' : 'ENERGY LOW', COL.red]);
  if (gnc && gnc.terrain_conflict) ann.push(['TERRAIN', COL.red]);
  if (st?.gear?.pos > 0.98) ann.push(['GR DN', COL.green]); else if (st?.gear?.pos > 0.02) ann.push(['GR TRNS', COL.yellow]);
  ann.forEach(([s, c], i) => T(g, s, 256 + (i - (ann.length - 1) / 2) * 76, 38, 11, c, 'center', '700'));

  // ---------------- ADI
  const cx = 256, cy = 172, r = 92;
  const sw = w.st.sw;
  let C = null, eul = null;
  if (valid) {
    const phi = (a.phi ?? a.bank ?? 0) * D2R, th = (a.theta ?? 0) * D2R, psi = (a.psi ?? a.track ?? 0) * D2R;
    if (sw.att === 1) C = dcm(phi, th, psi - (a.track ?? a.psi ?? 0) * D2R);      // LVLH: yaw from the velocity azimuth
    else C = dcm(phi, th, psi);
    if (sw.att === 2) {
      if (!w.attRef) w.attRef = dcm(phi, th, psi);
      C = mM(dcm(phi, th, psi), mT(w.attRef));
    } else w.attRef = null;
    eul = euler(C);
  }
  drawADI(g, w, cx, cy, r, C);
  // roll ring (fixed case) + moving roll pointer
  g.save();
  g.lineWidth = 11; g.strokeStyle = '#1c1d1e'; g.beginPath(); g.arc(cx, cy, r + 6.5, 0, Math.PI * 2); g.stroke();
  g.lineWidth = 1; g.strokeStyle = '#6d6f71'; g.beginPath(); g.arc(cx, cy, r + 12.5, 0, Math.PI * 2); g.stroke();
  g.strokeStyle = COL.white;
  for (let k = 0; k < 36; k++) {
    const an = k * 10 * D2R, long = k % 3 === 0, L = long ? 9 : 5;
    g.lineWidth = long ? 1.6 : 1;
    line(g, cx + Math.sin(an) * (r + 1), cy - Math.cos(an) * (r + 1), cx + Math.sin(an) * (r + 1 + L), cy - Math.cos(an) * (r + 1 + L));
  }
  tri(g, cx, cy - r - 13, 8, 'up', COL.white);                         // fixed 0° index
  if (eul) {                                                             // sky pointer
    g.translate(cx, cy); g.rotate(-eul.phi);
    tri(g, 0, -r - 1, 8, 'up', COL.yellow, '#000');
  }
  g.restore();
  // digital R / P / Y
  if (eul) {
    const R = w.hold('R', wrap360(Math.round(eul.phi * R2D))), P = w.hold('P', wrap360(Math.round(eul.th * R2D))), Y = w.hold('Y', wrap360(Math.round(eul.psi * R2D)));
    [['R', R], ['P', P], ['Y', Y]].forEach(([k, v], i) => { T(g, k, 352, 64 + i * 15, 12, COL.gray); T(g, pad(v % 360, 3), 390, 64 + i * 15, 12, COL.white, 'right'); });
  }
  // rate scales: roll (top), pitch (right), yaw (bottom)
  const fsR = RATE_FS[sw.rate], fsE = ERR_FS[sw.err];
  const rates = valid ? { p: a.p ?? (st.omega ? st.omega[0] * R2D : 0), q: a.q ?? (st.omega ? st.omega[1] * R2D : 0), r: a.r ?? (st.omega ? st.omega[2] * R2D : 0) } : null;
  rateScale(g, 'h', cx, 54, 62, rates ? rates.p : null, fsR, 'down');
  rateScale(g, 'v', 374, cy, 62, rates ? -rates.q : null, fsR, 'left');
  rateScale(g, 'h', cx, 291, 62, rates ? rates.r : null, fsR, 'up');
  // error needles (magenta) from the guidance commands
  if (valid && st.cmd) {
    g.save(); g.beginPath(); g.arc(cx, cy, r - 1, 0, Math.PI * 2); g.clip();
    const er = wrap180((st.cmd.bank ?? 0) - (a.bank ?? 0)), ep = (st.cmd.alpha ?? a.alpha) - a.alpha, ey = -(a.beta ?? 0);
    const k = r * 0.6;
    const nx = cx + clamp(er / fsE.r, -1, 1) * k, ny = cy - clamp(ep / fsE.p, -1, 1) * k, yx = cx + clamp(ey / fsE.y, -1, 1) * k;
    const needle = (x0, y0, x1, y1) => { g.lineCap = 'round'; g.strokeStyle = '#000'; g.lineWidth = 4.5; line(g, x0, y0, x1, y1); g.strokeStyle = COL.mag; g.lineWidth = 2.6; line(g, x0, y0, x1, y1); };
    needle(nx, cy - r, nx, cy - r * 0.42);
    needle(cx + r, ny, cx + r * 0.42, ny);
    needle(yx, cy + r, yx, cy + r * 0.42);
    g.restore();
  }
  // fixed orbiter reference symbol
  g.save(); g.lineCap = 'butt';
  const wing = (s) => { g.beginPath(); g.moveTo(cx - 34, cy); g.lineTo(cx - 12, cy); g.lineTo(cx - 6, cy + 7); g.lineTo(cx, cy); g.lineTo(cx + 6, cy + 7); g.lineTo(cx + 12, cy); g.lineTo(cx + 34, cy); g.stroke(); };
  g.strokeStyle = '#000'; g.lineWidth = 5; wing(); g.strokeStyle = COL.yellow; g.lineWidth = 2.6; wing();
  g.fillStyle = COL.yellow; g.beginPath(); g.arc(cx, cy, 2.2, 0, Math.PI * 2); g.fill();
  g.restore();
  if (!valid) offFlag(g, cx, cy - 34);

  // ---------------- tapes
  const ty = 66, th = 214;
  const alpha = w.smooth('α', valid && a.air_valid !== false ? a.alpha : NaN, 0.25);     // ADTA data invalid at low speed on the ground
  drawTape(g, { x: 14, y: ty, w: 42, h: th, v: alpha, ppu: th / 30, minor: 1, major: 5, label: 5, fmt: (t) => `${t}`, title: 'α', side: 'r',
    digital: fin(alpha) ? w.hold('a', alpha).toFixed(1) : '', bugs: valid && st.cmd && fin(st.cmd.alpha) ? [st.cmd.alpha] : [] });
  const mach = w.smooth('M', valid ? a.mach : NaN, 0.3);
  if (w.keasMode === undefined) w.keasMode = false;
  if (fin(mach)) { if (w.keasMode && mach > 0.95) w.keasMode = false; else if (!w.keasMode && mach < 0.9) w.keasMode = true; }
  if (w.keasMode) {
    const ke = w.smooth('KEAS', valid ? a.keas : NaN, 0.3);
    const refK = gnc && gnc.ref && fin(gnc.ref.V) && fin(a.rho) ? gnc.ref.V * Math.sqrt(a.rho / 1.225) * KT : NaN;
    drawTape(g, { x: 62, y: ty, w: 46, h: th, v: ke, ppu: th / 110, minor: 5, major: 10, label: 20, fmt: (t) => `${t}`, title: 'KEAS', side: 'r',
      digital: fin(ke) ? `${Math.round(w.hold('k', ke))}` : '', bugs: fin(refK) ? [refK] : [] });
  } else {
    drawTape(g, { x: 62, y: ty, w: 46, h: th, v: mach, ppu: th / 1.0, minor: 0.05, major: 0.1, label: 0.2, fmt: (t) => t.toFixed(1), title: 'M/VR', side: 'r',
      digital: fin(mach) ? w.hold('m', mach).toFixed(2) : '' });
  }
  // altitude: radar altitude (R) below 5000 ft AGL
  const agl = valid ? (a.agl_gear ?? a.agl) * FT : NaN;
  const radar = fin(agl) && agl < 5000;
  const hft = w.smooth(radar ? 'R' : 'H', valid ? (radar ? agl : a.alt * FT) : NaN, radar && agl < 200 ? 0.12 : 0.3);
  const hs = !fin(hft) ? { span: 20000, minor: 1000, major: 5000, label: 5000 } : hft > 20000 ? { span: 20000, minor: 1000, major: 5000, label: 5000 } : hft > 2500 ? { span: 4000, minor: 200, major: 1000, label: 1000 } : { span: 1000, minor: 50, major: 100, label: 200 };
  const refH = gnc && gnc.ref && fin(gnc.ref.h) ? gnc.ref.h * FT - (radar && fin(a.alt) ? (a.alt * FT - agl) : 0) : NaN;
  drawTape(g, { x: 404, y: ty, w: 48, h: th, v: hft, ppu: th / hs.span, minor: hs.minor, major: hs.major, label: hs.label, lo: -200,
    fmt: (t) => (hs.span > 5000 || t >= 10000 ? `${t / 1000}K` : `${t}`), title: 'H', sub: radar ? 'R' : '', side: 'l',
    digital: fmtFtK(w.hold('h', hft)), bugs: fin(refH) && !isAL(gnc) ? [refH] : [] });
  const hdot = w.smooth('Hd', valid ? a.vs * FT : NaN, 0.3);
  const hdS = fin(hdot) && Math.abs(hdot) > 170 ? { span: 1000, minor: 20, major: 100, label: 200 } : { span: 400, minor: 10, major: 50, label: 100 };
  drawTape(g, { x: 458, y: ty, w: 44, h: th, v: hdot, ppu: th / hdS.span, minor: hdS.minor, major: hdS.major, label: hdS.label, fmt: (t) => `${t}`, title: 'Ḣ', side: 'l',
    digital: fin(hdot) ? `${Math.round(w.hold('hd', hdot))}` : '' });

  // ---------------- lower section: Nz dial, beta, HSI, GSI, readouts
  const nz = valid ? a.nz : NaN;
  drawDial(g, 62, 352, 40, nz, -1, 3, 'Nz', 'g', fin(nz) ? w.hold('nz', nz) : nz);
  T(g, 'Beta', 16, 438, 12, COL.gray);
  dbox(g, 54, 425, 54, 17, '#9a9c9e');
  if (valid && a.air_valid !== false) { const b = w.hold('b', a.beta ?? 0); T(g, `${Math.abs(b).toFixed(1)} ${b >= 0 ? 'R' : 'L'}`, 103, 438, 12, Math.abs(b) > 2 ? COL.yellow : COL.white, 'right'); }
  // speedbrake (cmd / act) — small digital block under beta (MEDS entry PFD)
  if (valid && st.surf) {
    const sbA = w.hold('sbA', (st.surf.speedbrake ?? 0) / 98.6 * 100), sbC = w.hold('sbC', ((st.surf_cmd?.speedbrake ?? st.cmd?.speedbrake ?? st.surf.speedbrake) / 98.6) * 100);
    T(g, 'SB', 16, 460, 12, COL.gray);
    T(g, `${pad(Math.round(sbA), 3)}`, 76, 460, 12, COL.white, 'right');
    T(g, `${pad(Math.round(clamp(sbC, 0, 100)), 3)}`, 106, 460, 12, COL.mag, 'right');
  }

  const rw = runwayInfo(d.runway, gnc, st?.pos);
  const hx = 250, hy = 392, hr = 64;
  drawHSI(g, w, d, hx, hy, hr, rw, valid);
  // glide-slope indicator (altitude error)
  drawGSI(g, 342, hy, gnc, valid);
  // readouts (right column)
  const rx = 372;
  const rows = [];
  if (valid && st.pos) {
    const p = st.pos;
    rows.push(['Rwy', `${rw.end}`]);
    rows.push(['Rng', `${(Math.hypot(p[0] - rw.thr[0], p[1] - rw.thr[1]) * NM).toFixed(1)}`]);
    const hac = d.plan && d.plan.hac;
    if (hac && !isAL(gnc)) rows.push(['HAC-C', `${(Math.max(0, Math.hypot(p[0] - hac.center[0], p[1] - hac.center[1])) * NM).toFixed(1)}`]);
    if (gnc && fin(gnc.range_to_go) && gnc.range_to_go > 0) rows.push(['R-T-G', `${(gnc.range_to_go * NM).toFixed(1)}`]);
    const cross = rw.end === '33' ? st.rwy?.cross : -(st.rwy?.cross ?? 0);
    if (fin(cross)) rows.push(['Xtrk', `${Math.abs(cross * FT / 1000).toFixed(2)}${cross >= 0 ? 'R' : 'L'}`]);
    if (gnc && fin(gnc.energy_err)) rows.push(['ΔE', `${gnc.energy_err >= 0 ? '+' : ''}${Math.round(gnc.energy_err * FT)}`]);
    if (gnc && fin(gnc.time_to_go) && gnc.time_to_go > 0) rows.push(['TTG', mmss(gnc.time_to_go)]);
    // ΔAz to the HAC tangency (TAEM) or the runway threshold (A/L)
    const tp = d.plan && !isAL(gnc) ? hacTangent(st.pos, d.plan.hac) : null;
    const daz = wrap180(brgTo(st.pos, tp || rw.thr) - (a.track ?? a.psi));
    rows.push(['ΔAz', `${Math.round(Math.abs(daz))}${daz >= 0 ? 'R' : 'L'}`, Math.abs(daz) > 45 ? COL.yellow : COL.white]);
  }
  w.hold('pfdRows', rows).forEach(([k, v, c], i) => { T(g, k, rx + 22, 336 + i * 17, 12, COL.gray); T(g, v, 504, 336 + i * 17, 13, c || COL.white, 'right'); });
  drawKeyLegends(g, w);
}

/** ADI 8-ball: per-pixel shaded sphere + vector markings, orthographic, pilot's view along +x body */
function drawADI(g, w, cx, cy, r, C) {
  const s = w.scale;
  const N = Math.max(16, Math.round(2 * r * s));
  if (!w.ball || w.ball.width !== N) {
    w.ball = document.createElement('canvas'); w.ball.width = w.ball.height = N;
    w.bctx = w.ball.getContext('2d'); w.bimg = w.bctx.createImageData(N, N);
  }
  const D = w.bimg.data, Ct = C ? mT(C) : null;
  const ux = Ct ? Ct[2][0] : 0, uy = Ct ? Ct[2][1] : 0, uz = Ct ? Ct[2][2] : 0;     // world-down in body axes
  const aa = N / 2.6;
  const Lx = 0.80, Ly = -0.38, Lz = -0.46;        // highlight direction (upper left, toward viewer)
  for (let j = 0; j < N; j++) {
    const sy = (j + 0.5) / N * 2 - 1;
    for (let i = 0; i < N; i++) {
      const sx = (i + 0.5) / N * 2 - 1, rr = sx * sx + sy * sy, o = (j * N + i) * 4;
      if (rr > 1.02) { D[o + 3] = 0; continue; }
      const bx = Math.sqrt(Math.max(0, 1 - rr));
      let t = 0.5;
      if (Ct) t = clamp(0.5 - (ux * bx + uy * sx + uz * sy) * aa, 0, 1);              // 1: sky (upper hemisphere)
      const shade = 0.66 + 0.34 * bx;
      const spec = Math.pow(Math.max(0, bx * Lx + sx * Ly + sy * Lz), 24) * 46;
      const R = (44 + 152 * t) * shade + spec, G = (45 + 151 * t) * shade + spec, B = (48 + 142 * t) * shade + spec;
      D[o] = R; D[o + 1] = G; D[o + 2] = B;
      D[o + 3] = clamp((1 - Math.sqrt(rr)) * N * 0.5 + 0.5, 0, 1) * 255;
    }
  }
  w.bctx.putImageData(w.bimg, 0, 0);
  if (!C) { g.save(); g.globalAlpha = 0.35; g.drawImage(w.ball, cx - r, cy - r, 2 * r, 2 * r); g.restore(); return; }
  g.drawImage(w.ball, cx - r, cy - r, 2 * r, 2 * r);
  // markings
  g.save(); g.beginPath(); g.arc(cx, cy, r - 0.5, 0, Math.PI * 2); g.clip();
  const P = (az, el) => {
    const ce = Math.cos(el), dn = [ce * Math.cos(az), ce * Math.sin(az), -Math.sin(el)];
    return [C[0][0] * dn[0] + C[0][1] * dn[1] + C[0][2] * dn[2], C[1][0] * dn[0] + C[1][1] * dn[1] + C[1][2] * dn[2], C[2][0] * dn[0] + C[2][1] * dn[1] + C[2][2] * dn[2]];
  };
  const poly = (fn, n, col, lw) => {
    g.strokeStyle = col; g.lineWidth = lw; g.beginPath();
    let pen = false;
    for (let k = 0; k <= n; k++) {
      const b = fn(k / n);
      if (b[0] > -0.02) { const x = cx + r * b[1], y = cy + r * b[2]; if (pen) g.lineTo(x, y); else g.moveTo(x, y); pen = true; } else pen = false;
    }
    g.stroke();
  };
  const dark = '#121314', lite = '#e9e9e6';
  for (let el = -80; el <= 80; el += 10) {
    if (el === 0) continue;
    const e = el * D2R, col = el > 0 ? dark : lite, lw = el % 30 === 0 ? 1.5 : 0.9;
    poly((u) => P(u * 2 * Math.PI, e), 120, col, lw);
    // 5° marks either side of every 30° meridian
    for (let az = 0; az < 360; az += 30) {
      const e5 = (el - 5 * Math.sign(el)) * D2R;
      poly((u) => P((az - 3 + 6 * u) * D2R, e5), 3, el > 0 ? dark : lite, 0.8);
    }
  }
  for (let az = 0; az < 360; az += 30) {
    const lw = az % 90 === 0 ? 1.5 : 1;
    poly((u) => P(az * D2R, (u * 86) * D2R), 30, dark, lw);
    poly((u) => P(az * D2R, (-u * 86) * D2R), 30, lite, lw);
  }
  // horizon
  poly((u) => P(u * 2 * Math.PI, 0), 160, '#000', 3.2);
  poly((u) => P(u * 2 * Math.PI, 0), 160, '#ffffff', 1.5);
  // numerals: pitch (on the 30° meridians) and yaw (along the equator), drawn in the ball's local orientation
  const label = (az, el, s, col) => {
    const b = P(az, el); if (b[0] < 0.28) return;
    const b2 = P(az + 0.03, el);
    const x = cx + r * b[1], y = cy + r * b[2], ang = Math.atan2(b2[2] - b[2], b2[1] - b[1]);
    g.save(); g.translate(x, y); g.rotate(ang); g.globalAlpha = clamp((b[0] - 0.28) * 3, 0, 1);
    T(g, s, 0, 0, 12 * (0.55 + 0.45 * b[0]), col, 'center', '700', SANS, 'middle'); g.restore();
  };
  for (let az = 0; az < 360; az += 90) for (const el of [30, 60, -30, -60]) {
    label((az + 15) * D2R, el * D2R, el > 0 ? String(el / 10) : String((360 + el) / 10), el > 0 ? dark : lite);
  }
  for (let az = 0; az < 360; az += 30) label(az * D2R, -5 * D2R, String(az / 10), lite);
  g.restore();
  // rim shadow
  const rg = g.createRadialGradient(cx, cy, r * 0.82, cx, cy, r);
  rg.addColorStop(0, 'rgba(0,0,0,0)'); rg.addColorStop(1, 'rgba(0,0,0,0.45)');
  g.fillStyle = rg; g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.fill();
}

function rateScale(g, o, x, y, half, v, fs, dir) {
  g.strokeStyle = COL.white; g.lineWidth = 1.2;
  if (o === 'h') {
    line(g, x - half, y, x + half, y);
    for (let k = -2; k <= 2; k++) line(g, x + k * half / 2, y - (k === 0 ? 5 : 3), x + k * half / 2, y + (k === 0 ? 5 : 3));
    T(g, String(fs), x - half - 4, y + 4, 11, COL.gray, 'right'); T(g, String(fs), x + half + 4, y + 4, 11, COL.gray, 'left');
    if (v !== null && fin(v)) { const px = x + clamp(v / fs, -1.08, 1.08) * half; tri(g, px, dir === 'down' ? y - 2 : y + 2, 9, dir, COL.white, '#000'); }
  } else {
    line(g, x, y - half, x, y + half);
    for (let k = -2; k <= 2; k++) line(g, x - (k === 0 ? 5 : 3), y + k * half / 2, x + (k === 0 ? 5 : 3), y + k * half / 2);
    T(g, String(fs), x, y - half - 4, 11, COL.gray, 'center'); T(g, String(fs), x, y + half + 12, 11, COL.gray, 'center');
    if (v !== null && fin(v)) { const py = y + clamp(v / fs, -1.08, 1.08) * half; tri(g, x + 2, py, 9, 'left', COL.white, '#000'); }
  }
}

/** moving vertical tape (AMI / AVVI style): drum shading, ticks, labels, command bug, digital window */
function drawTape(g, o) {
  const { x, y, w, h } = o, cy = y + h / 2, ok = fin(o.v);
  g.save();
  const gr = g.createLinearGradient(0, y, 0, y + h);
  gr.addColorStop(0, '#040404'); gr.addColorStop(0.16, '#262728'); gr.addColorStop(0.5, '#3a3b3c'); gr.addColorStop(0.84, '#262728'); gr.addColorStop(1, '#040404');
  g.fillStyle = gr; g.fillRect(x, y, w, h);
  g.beginPath(); g.rect(x, y, w, h); g.clip();
  if (ok) {
    const half = h / 2 / o.ppu;
    const k0 = Math.ceil((o.v - half) / o.minor), k1 = Math.floor((o.v + half) / o.minor);
    const mj = Math.round(o.major / o.minor), lb = Math.round(o.label / o.minor);
    g.strokeStyle = '#ececea'; g.lineWidth = 1.1;
    for (let k = k0; k <= k1; k++) {
      const t = +(k * o.minor).toFixed(6);
      if (o.lo !== undefined && t < o.lo) continue;
      const yy = cy - (t - o.v) * o.ppu;
      const isMaj = k % mj === 0, len = isMaj ? w * 0.26 : w * 0.13;
      if (o.side === 'r') line(g, x + w - len, yy, x + w, yy); else line(g, x, yy, x + len, yy);
      if (k % lb === 0) {
        const fade = 1 - Math.pow(Math.abs(yy - cy) / (h / 2), 3) * 0.6;
        g.globalAlpha = fade;
        T(g, o.fmt(t), o.side === 'r' ? x + w * 0.42 : x + w * 0.58, yy + 4, 11.5, '#f2f2f0', 'center', '700');
        g.globalAlpha = 1;
      }
    }
    for (const b of o.bugs || []) {
      if (!fin(b)) continue;
      const yy = clamp(cy - (b - o.v) * o.ppu, y + 4, y + h - 4);
      g.fillStyle = COL.mag;
      if (o.side === 'r') { g.beginPath(); g.moveTo(x + w, yy - 6); g.lineTo(x + w - 9, yy); g.lineTo(x + w, yy + 6); g.closePath(); g.fill(); }
      else { g.beginPath(); g.moveTo(x, yy - 6); g.lineTo(x + 9, yy); g.lineTo(x, yy + 6); g.closePath(); g.fill(); }
    }
  }
  g.restore();
  g.strokeStyle = '#88898b'; g.lineWidth = 1; g.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
  T(g, o.title, x + w / 2, y - 6, 14, COL.white, 'center', '700');
  if (o.sub) T(g, o.sub, x + w - 2, y - 6, 11, COL.yellow, 'right', '700');
  if (!ok) { offFlag(g, x + w / 2, cy, Math.min(34, w - 4)); return; }
  // digital readout window with pointer notch
  const bw = w + 6, bh = 22, bx = x - 3, by = cy - bh / 2;
  g.fillStyle = '#000'; g.strokeStyle = COL.white; g.lineWidth = 1.3;
  g.beginPath();
  if (o.side === 'r') { g.moveTo(bx, by); g.lineTo(bx + bw - 6, by); g.lineTo(bx + bw + 2, cy); g.lineTo(bx + bw - 6, by + bh); g.lineTo(bx, by + bh); }
  else { g.moveTo(bx + bw, by); g.lineTo(bx + 6, by); g.lineTo(bx - 2, cy); g.lineTo(bx + 6, by + bh); g.lineTo(bx + bw, by + bh); }
  g.closePath(); g.fill(); g.stroke();
  T(g, o.digital, x + w / 2 + (o.side === 'r' ? -2 : 2), cy + 5.5, 15, COL.white, 'center', '700');
}

function drawDial(g, cx, cy, r, v, lo, hi, label, unit, dv = v) {
  const A0 = 135 * D2R, SW = 270 * D2R, ang = (t) => A0 + (clamp(t, lo - 0.15, hi + 0.15) - lo) / (hi - lo) * SW;
  g.fillStyle = '#111213'; g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.fill();
  g.strokeStyle = '#77797b'; g.lineWidth = 1; g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.stroke();
  // caution / limit arcs
  g.lineWidth = 3.5;
  g.strokeStyle = COL.yellow; g.beginPath(); g.arc(cx, cy, r - 3, ang(2.0), ang(2.5)); g.stroke();
  g.strokeStyle = COL.red; g.beginPath(); g.arc(cx, cy, r - 3, ang(2.5), ang(hi)); g.stroke();
  g.strokeStyle = COL.white;
  for (let t = lo; t <= hi + 1e-6; t += 0.5) {
    const an = ang(t), maj = Math.abs(t - Math.round(t)) < 1e-6, L = maj ? 8 : 4;
    g.lineWidth = maj ? 1.5 : 1;
    line(g, cx + Math.cos(an) * (r - 1), cy + Math.sin(an) * (r - 1), cx + Math.cos(an) * (r - 1 - L), cy + Math.sin(an) * (r - 1 - L));
    if (maj) T(g, String(t), cx + Math.cos(an) * (r - 15), cy + Math.sin(an) * (r - 15) + 4, 10.5, COL.white, 'center');
  }
  T(g, label, cx - r, cy - r + 2, 13, COL.white, 'left', '700');
  if (!fin(v)) { offFlag(g, cx, cy + 16); return; }
  const an = ang(v);
  g.lineCap = 'round';
  g.strokeStyle = '#000'; g.lineWidth = 4.5; line(g, cx - Math.cos(an) * 6, cy - Math.sin(an) * 6, cx + Math.cos(an) * (r - 5), cy + Math.sin(an) * (r - 5));
  g.strokeStyle = COL.white; g.lineWidth = 2.4; line(g, cx - Math.cos(an) * 6, cy - Math.sin(an) * 6, cx + Math.cos(an) * (r - 5), cy + Math.sin(an) * (r - 5));
  g.lineCap = 'butt';
  g.fillStyle = '#4b4d4f'; g.beginPath(); g.arc(cx, cy, 3.5, 0, Math.PI * 2); g.fill();
  dbox(g, cx - 19, cy + r + 4, 38, 17, '#9a9c9e');
  T(g, dv.toFixed(1), cx, cy + r + 17, 12, dv > 2.5 ? COL.red : dv > 2 ? COL.yellow : COL.white, 'center');
}

const CARD = { 0: 'N', 3: '3', 6: '6', 9: 'E', 12: '12', 15: '15', 18: 'S', 21: '21', 24: '24', 27: 'W', 30: '30', 33: '33' };
function orbiterSymbol(g, x, y, s, col, rot = 0) {
  g.save(); g.translate(x, y); g.rotate(rot); g.scale(s, s);
  g.beginPath();
  g.moveTo(0, -10); g.lineTo(1.6, -6); g.lineTo(1.9, -1); g.lineTo(7.5, 5.5); g.lineTo(7.5, 7.2); g.lineTo(2.2, 7); g.lineTo(1.6, 9);
  g.lineTo(-1.6, 9); g.lineTo(-2.2, 7); g.lineTo(-7.5, 7.2); g.lineTo(-7.5, 5.5); g.lineTo(-1.9, -1); g.lineTo(-1.6, -6); g.closePath();
  g.fillStyle = col; g.fill(); g.lineWidth = 0.8 / s * 1.5; g.strokeStyle = '#000'; g.stroke();
  g.restore();
}

function drawHSI(g, w, d, cx, cy, r, rw, valid) {
  const st = d.st, a = st?.air, gnc = st?.gnc;
  const hdg = valid ? (a.psi ?? a.track) : 0;
  // compass card
  g.fillStyle = '#18191a'; g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.fill();
  g.strokeStyle = '#6e7072'; g.lineWidth = 1; g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.stroke();
  g.save(); g.translate(cx, cy); g.rotate(-hdg * D2R);
  g.strokeStyle = COL.white;
  for (let k = 0; k < 72; k++) {
    const an = k * 5 * D2R, L = k % 2 === 0 ? 7 : 4;
    g.lineWidth = k % 6 === 0 ? 1.5 : 1;
    line(g, Math.sin(an) * r, -Math.cos(an) * r, Math.sin(an) * (r - L), -Math.cos(an) * (r - L));
  }
  for (let k = 0; k < 36; k += 3) {
    g.save(); g.rotate(k * 10 * D2R);
    T(g, CARD[k], 0, -r + 19, k % 9 === 0 ? 13 : 11.5, COL.white, 'center', '700');
    g.restore();
  }
  g.restore();
  // lubber line + heading box
  tri(g, cx, cy - r + 1, 8, 'down', COL.white);
  dbox(g, cx - 19, cy - r - 21, 38, 17);
  if (valid) T(g, pad(w.hold('hsiHdg', Math.round(wrap360(hdg)) % 360), 3), cx, cy - r - 8, 13, COL.white, 'center', '700');
  if (!valid) { offFlag(g, cx, cy); return; }
  // course pointer + deviation bar
  const p = st.pos;
  let crs = rw.crs, dev = rw.end === '33' ? (st.rwy?.cross ?? 0) : -(st.rwy?.cross ?? 0);
  const hac = d.plan && d.plan.hac;
  const onHac = gnc && /HAC|PRF/.test(gnc.sub || '') && !isAL(gnc) && hac;
  if (onHac && (gnc.sub || '') === 'HAC') {
    const bC = brgTo(hac.center, p);
    crs = wrap360(bC + (hac.dir > 0 ? 90 : -90));
    const radial = Math.hypot(p[0] - hac.center[0], p[1] - hac.center[1]) - hac.radius;
    dev = hac.dir > 0 ? -radial : radial;                       // +: vehicle right of the course
  }
  const fs = isAL(gnc) ? 1000 / FT : 5000 / FT;               // full-scale deviation 1 kft (A/L), 5 kft (TAEM)
  const dx = clamp(-dev / fs, -1.1, 1.1) * r * 0.42;
  g.save(); g.translate(cx, cy); g.rotate((crs - hdg) * D2R);
  g.strokeStyle = COL.white; g.lineWidth = 1.2;
  for (const k of [-2, -1, 1, 2]) { g.beginPath(); g.arc(k * r * 0.21, 0, 2.6, 0, Math.PI * 2); g.stroke(); }
  g.strokeStyle = COL.yellow; g.fillStyle = COL.yellow; g.lineWidth = 3;
  line(g, 0, -r + 12, 0, -r * 0.46); line(g, 0, r * 0.46, 0, r - 8);
  g.beginPath(); g.moveTo(0, -r + 6); g.lineTo(-6, -r + 17); g.lineTo(6, -r + 17); g.closePath(); g.fill();
  line(g, dx, -r * 0.4, dx, r * 0.4);
  g.restore();
  // bearing pointers: primary (H: HAC tangency in TAEM, R: runway in A/L), secondary (C: HAC centre)
  const tp = hac && !isAL(gnc) && !onHac ? hacTangent(p, hac) : null;
  const bear = (b, lab, col, hollow) => {
    g.save(); g.translate(cx, cy); g.rotate((b - hdg) * D2R);
    g.beginPath(); g.moveTo(0, -r - 2); g.lineTo(-8.5, -r + 15); g.lineTo(8.5, -r + 15); g.closePath();
    if (hollow) { g.strokeStyle = col; g.lineWidth = 1.6; g.fillStyle = '#000'; g.fill(); g.stroke(); } else { g.fillStyle = col; g.fill(); g.strokeStyle = '#000'; g.lineWidth = 1; g.stroke(); }
    g.translate(0, -r + 9); g.rotate(-(b - hdg) * D2R);
    T(g, lab, 0, 0.5, 9.5, hollow ? col : '#000', 'center', '700', SANS, 'middle');
    g.restore();
    g.save(); g.translate(cx, cy); g.rotate((b - hdg) * D2R);
    g.strokeStyle = col; g.lineWidth = 1.6; g.setLineDash([3, 3]); line(g, 0, r - 2, 0, r - 12); g.setLineDash([]);
    g.restore();
  };
  if (tp) bear(brgTo(p, tp), 'H', COL.mag, false);
  else bear(brgTo(p, rw.thr), 'R', COL.mag, false);
  if (hac && !isAL(gnc)) bear(brgTo(p, hac.center), 'C', COL.cyan, true);
  orbiterSymbol(g, cx, cy, 1.25, COL.white);
}

function drawGSI(g, x, cy, gnc, valid) {
  const half = 52;
  T(g, 'GS', x, cy - half - 8, 12, COL.white, 'center', '700');
  g.strokeStyle = COL.white; g.lineWidth = 1.2;
  for (const k of [-2, -1, 1, 2]) { g.beginPath(); g.arc(x, cy + k * half / 2, 2.6, 0, Math.PI * 2); g.stroke(); }
  g.lineWidth = 2; line(g, x - 7, cy, x + 7, cy);
  const al = isAL(gnc);
  T(g, al ? '1K' : '5K', x, cy + half + 14, 10, COL.gray, 'center');
  if (!valid || !gnc || !fin(gnc.h_err) || /ROLL|STOP|TOUCH/.test(gnc.phase)) { offFlag(g, x, cy + 22, 28, 14); return; }
  const fs = (al ? 1000 : 5000) / FT;
  const py = cy + clamp(gnc.h_err / fs, -1.1, 1.1) * half;   // glide slope below the vehicle when high
  tri(g, x - 2, py, 10, 'right', COL.mag, '#000');
}

// =========================================================================================== HSIT (moving map)
//
// Graphic horizontal situation for TAEM and approach / landing.  Symbology follows the orbiter's own
// displays: the SPEC 50 HORIZ SIT graphic (orbiter fixed at the bottom pointing up, 20 / 40 / 60 s
// constant-bank predictor circles, HAC circle, touchdown circle joined to the HAC exit, Nz next to
// the orbiter, top scale = time to HAC intercept -> HAC radial error -> runway crosstrack, right
// scale = altitude error ±5K / ±1K fly-to) and the MEDS A/E PFD HSI in TAEM (H bug to the HAC
// tangency, C bug to the HAC centre, runway range via the HAC, HAC-C range, ΔAZ becoming HTA at
// HAC intercept).  Sources: DPS Dictionary (HORIZ SIT), Entry/TAEM/Approach workbook USA005512,
// STS 83-0020 displays & controls FSSR, CSI 21002, FAA AC 25-11B.
//
// Colour philosophy (NASA orbiter cockpit-upgrade colour table + AC 25-11B): light grey static
// labels, white scales and dynamic data, magenta targets / guidance, light green selected items,
// cyan missing data ('M') and edge-key legends, dark grey inactive elements, yellow caution, red
// warning.  DPS status characters follow the value: ↑ ↓ software limit, H L off scale, M missing.
//
// Display-processor model - the MDU sees what the GPC puts on the display bus, not the simulator:
//   * the GPC samples the navigation state twice a second (2.08 Hz, 480 ms) and the sample reaches
//     the screen 160 ms later (bus + IDP processing): the picture moves in steps and lags;
//   * slow cycle 960 ms (1.04 Hz): target waypoint, bearings, ΔAZ / HTA, ranges, time to go,
//     wind, energy, the message list - and every digital readout;
//   * a range / orientation change makes the IDP rebuild the map: the map area stays blank until
//     the next data cycle plus the transform time (more vectors at larger ranges), the range label
//     flashes meanwhile; entering the page draws the static format first;
//   * no new data for 3 s: big X over the display (the GPC stopped updating it).
// Blinking is 1.56 Hz (2 IDP frames on / 2 off, inside AC 25-11B's 0.8 - 4 Hz).  New messages come
// up in reverse video and blink for 5 s, then stay steady while the condition lasts; a changed
// guidance mode is boxed for 10 s.
const HS_BUS = 480, HS_LAT = 160, HS_SLOW = 960, HS_BLINK = 320;
const HC = { lbl: '#b4b8bc', dat: '#f4f4f2', dim: '#5c6064', sel: '#86ff9e', caut: '#ffe04a' };
const NZ_LIM = 1.8;                                          // TAEM normal-acceleration limit (g)

/** text with a black halo, for labels drawn over map symbology */
function TH(g, s, x, y, size, color, align = 'left', weight = '700') {
  strokeText(g, s, x, y, size, color, align, weight, 'alphabetic', 3);
}
/** right-aligned readout ending at x with an optional DPS status character after it */
function RO(g, x, y, val, status = '', col = HC.dat, size = 15) {
  T(g, val, x, y, size, col, 'right', '700');
  if (status) T(g, status, x + 2, y, size - 2, status === 'M' || status === '?' ? COL.cyan : HC.caut, 'left', '700');
}
/** boxed readout (MEDS style), right aligned inside a box of width bw ending at x */
function BRO(g, x, y, bw, val, col = HC.dat, status = '') {
  g.strokeStyle = HC.lbl; g.lineWidth = 1; g.strokeRect(Math.round(x - bw) + 0.5, y - 15.5, bw, 20);
  T(g, val, x - 4, y, 15, col, 'right', '700');
  if (status) T(g, status, x + 3, y, 13, status === 'M' ? COL.cyan : HC.caut, 'left', '700');
}
/** HAC exit: where the HAC touches the extended centreline of the landing runway */
function hacExit(hac, rw) {
  if (!hac) return null;
  const c = hac.center, u = rw.u, s = (c[0] - rw.c[0]) * u[0] + (c[1] - rw.c[1]) * u[1];
  return [rw.c[0] + u[0] * s, rw.c[1] + u[1] * s];
}
const polyLen = (P) => { let s = 0; for (let i = 1; i < (P ? P.length : 0); i++) s += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]); return s; };

/** slow-cycle navigation computations (sent by the GPC once per second) */
function hsNav(D, prev) {
  const st = D.st, a = st?.air, gnc = st?.gnc;
  if (!a || D.link === 'offline') return null;
  const pos = st.pos, rw = runwayInfo(D.runway, gnc, pos), plan = D.plan, hac = plan && plan.hac;
  const ph = gnc?.phase || '', sub = gnc?.sub || '';
  const al = isAL(gnc), gnd = /TOUCH|ROLL|STOP|CRASH/.test(ph);
  const onHac = !al && !!hac && sub === 'HAC', prf = !al && sub === 'PRF';
  const exit = hac ? hacExit(hac, rw) : null;
  const wp1 = hac && !al && !onHac && !prf ? hacTangent(pos, hac) : null;
  // touchdown point (WP2): 1000 ft past the threshold
  const sg = rw.end === '33' ? 1 : -1, td = [rw.thr[0] + sg * rw.u[0] * 1000 / FT, rw.thr[1] + sg * rw.u[1] * 1000 / FT];
  const trk = a.track ?? a.psi, gs = Math.hypot(st.vel[0], st.vel[1]);
  // range to the touchdown point via the HAC: remaining plan + approach, else straight / tangent + arc + final
  let rngTd = NaN;
  if (gnc && fin(gnc.range_to_go) && gnc.range_to_go > 0 && plan) rngTd = gnc.range_to_go + polyLen(plan.approach) + 1000 / FT;
  else rngTd = Math.hypot(td[0] - pos[0], td[1] - pos[1]);
  // HAC turn angle to go (deg, in the turn direction) from WP1 - or from the vehicle once on the HAC - to the exit
  let hta = NaN;
  if (hac && exit && (wp1 || onHac)) {
    const from = brgTo(hac.center, onHac ? pos : wp1), to = brgTo(hac.center, exit);
    hta = wrap360((to - from) * (hac.dir > 0 ? 1 : -1));
    if (onHac && prev && fin(prev.hta) && prev.hta < 30 && hta > 330) hta = 0;
  }
  // lateral deviation for the top scale: before the HAC time to intercept; on the HAC radial error;
  // prefinal and A/L runway crosstrack (ft, + right of the course)
  const dyRw = (rw.end === '33' ? 1 : -1) * (st.rwy?.cross ?? NaN) * FT;
  let top = null;
  if (!gnd) {
    if (wp1) top = { mode: 'THAC', t: Math.hypot(wp1[0] - pos[0], wp1[1] - pos[1]) / Math.max(gs, 30), right: hac.dir > 0 };
    else if (onHac) { const rad = (Math.hypot(pos[0] - hac.center[0], pos[1] - hac.center[1]) - hac.radius) * FT; top = { mode: 'HACR', dev: hac.dir > 0 ? -rad : rad, fs: 5000 }; }
    else if (prf || al) top = { mode: 'XTRK', dev: dyRw, fs: 2500 };
  }
  // wind at the vehicle (air data); landing-runway components of the surface wind (tail +, cross + from the left)
  const wnd = a.wind || [0, 0, 0], c = rw.crs * D2R;
  const sfc = st.env && fin(st.env.wind) && fin(st.env.dir) ? [-st.env.wind * Math.sin(st.env.dir * D2R), -st.env.wind * Math.cos(st.env.dir * D2R)] : wnd;
  const tail = sfc[0] * Math.sin(c) + sfc[1] * Math.cos(c), xw = sfc[0] * Math.cos(c) - sfc[1] * Math.sin(c);
  const appr = al || prf || a.alt * FT < 12000;                 // landing-wind limits matter from A/L scaling on
  const bH = hac && !al ? (wp1 ? brgTo(pos, wp1) : null) : null;
  const tgt = wp1 ? 'WP1' : onHac ? 'HAC' : `${rw.name}${rw.end}`;
  return {
    rw, td, exit, wp1, al, gnd, onHac, prf, hta, top, tgt,
    daz: wrap180(brgTo(pos, wp1 || td) - trk),
    rngTd, rngC: hac && !al ? Math.hypot(hac.center[0] - pos[0], hac.center[1] - pos[1]) : NaN,
    bH, bC: hac && !al && !gnd ? brgTo(pos, hac.center) : null, bR: al || !hac ? brgTo(pos, td) : null,
    ttg: gnc && fin(gnc.time_to_go) && gnc.time_to_go > 0 ? gnc.time_to_go : NaN,
    windFrom: wrap360(Math.atan2(-wnd[0], -wnd[1]) * R2D), ws: Math.hypot(wnd[0], wnd[1]), tail, xw,
    dyRw, de: gnc && fin(gnc.energy_err) ? gnc.energy_err * FT : NaN, hasGnc: !!gnc,
    gs: gs * KT, tas: a.tas * KT, keas: a.keas, mach: a.mach, h: a.alt * FT, agl: (a.agl_gear ?? a.agl ?? a.alt) * FT, hdot: a.vs * FT, trk: wrap360(trk),
    sbA: st.surf ? st.surf.speedbrake / 98.6 * 100 : NaN, sbC: ((st.surf_cmd?.speedbrake ?? st.cmd?.speedbrake ?? NaN) / 98.6) * 100,
    appr, msgs: hsMessages(D, gnc, plan, { xw, tail, gnd, appr }),
  };
}

/** message list, highest priority first: [id, text, level]  (2 warning, 1 caution, 0 advisory) */
function hsMessages(D, gnc, plan, x) {
  const st = D.st, m = [];
  if (gnc && gnc.terrain_conflict && !x.gnd) m.push(['terr', 'TERRAIN', 2]);
  if (gnc && gnc.reachable === false && !x.gnd) m.push(['nrg', gnc.energy_high ? 'ENERGY HIGH' : gnc.energy === 'turn' ? 'TURN LIMIT' : 'ENERGY LOW', 1]);
  if (D.link === 'local') m.push(['prev', 'GPC OFF - NAV PREVIEW', 1]);
  if (!gnc && D.link !== 'local') m.push(['nog', 'GUID DATA M', 1]);
  if (gnc && gnc.degraded && !x.gnd) m.push(['degr', 'GUID DEGRADED', 1]);
  if (plan && plan.fallback && !x.gnd) m.push(['fbk', 'PLAN FALLBACK', 1]);
  if (!x.gnd && x.appr && Math.abs(x.xw) * KT > 15) m.push(['xw', `XWIND ${Math.round(Math.abs(x.xw) * KT)} KT`, 1]);
  if (!x.gnd && x.appr && x.tail * KT > 10) m.push(['tw', `TAILWIND ${Math.round(x.tail * KT)} KT`, 1]);
  if (plan && plan.prev && st && fin(plan.prev.t) && st.t - plan.prev.t < 20)
    m.push([`sw${plan.id}`, `HAC SWITCH ${plan.prev.hac ? (plan.prev.hac.dir > 0 ? 'R' : 'L') : '-'} → ${plan.hac ? (plan.hac.dir > 0 ? 'R' : 'L') : '-'}`, 1]);
  if (gnc && gnc.scp && gnc.scp.pending && !x.gnd) m.push(['scp', 'REPLAN', 0]);
  if (st && st.paused) m.push(['hold', 'SIM HOLD', 0]);
  return m;
}

function hsState(w) {
  return (w.hs ||= { bus: -1, q: [], cur: null, nav: null, slow: -1, seen: {}, msgs: [], fma: {}, planId: null, newPlanT: -1e9, entryT: w.nowT, rxT: w.nowT, lastSt: null });
}
/** display bus: GPC sample every 480 ms, delivered 160 ms later; slow-cycle computations every 960 ms */
function hsBus(hs, d, now) {
  if (d.st !== hs.lastSt) { hs.lastSt = d.st; hs.rxT = now; }
  const k = Math.floor(now / HS_BUS);
  if (k !== hs.bus) {
    hs.bus = k;
    hs.q.push({ at: k * HS_BUS + HS_LAT, D: { st: d.st, plan: d.plan, planPts: d.planPts, trail: d.trail, trailN: d.trail ? d.trail.length : 0, runway: d.runway, link: d.link, rxT: hs.rxT } });
  }
  while (hs.q.length && hs.q[0].at <= now) {
    hs.cur = hs.q.shift().D;
    const s = Math.floor(now / HS_SLOW);
    if (s !== hs.slow) {
      hs.slow = s;
      hs.nav = hsNav(hs.cur, hs.nav);
      const p = hs.cur.plan;
      if (p && p.id !== hs.planId) { if (hs.planId !== null) hs.newPlanT = now; hs.planId = p.id; }
      const msgs = hs.nav ? hs.nav.msgs.slice() : [];
      if (now - hs.newPlanT < 6000 && hs.nav && !hs.nav.gnd) msgs.push([`np${hs.planId}`, `NEW PLAN ${hs.planId}`, 0]);
      const seen = {};
      for (const mm of msgs) seen[mm[0]] = hs.seen[mm[0]] ?? now;
      hs.seen = seen; hs.msgs = msgs;
    }
  }
}

function drawHSIT(g, w, d) {
  const now = w.nowT, hs = hsState(w);
  hsBus(hs, d, now);
  const D = hs.cur, st = D?.st, a = st?.air, gnc = st?.gnc;
  const valid = !!a && D.link !== 'offline';
  const nav = valid ? hs.nav : null;
  const R = RANGES[w.st.rng], headUp = !w.st.sw.orient;
  const blink = Math.floor(now / HS_BLINK) % 2 === 0;
  const ready = hs.cur && now - hs.entryT >= HS_BUS + HS_LAT;
  const rebuilding = now - (w.mapT ?? -1e9) < HS_BUS + HS_LAT + R * 2.5;
  const mapOn = valid && ready && !rebuilding;
  // ------------------------------------------------ geometry
  const ox = 256, oy = headUp ? 360 : 258, RA = headUp ? 272 : 176, ppn = RA / R, ARC = 52;
  const hdg = valid ? wrap360(a.psi ?? a.track) : 0, trk = valid ? wrap360(a.track ?? a.psi) : 0;
  const rot = headUp ? hdg : 0, pos = valid ? st.pos : [0, 0, 0];
  const cr = Math.cos(rot * D2R), sr = Math.sin(rot * D2R);
  const S = (e, n) => { const de = (e - pos[0]) * NM, dn = (n - pos[1]) * NM; return [ox + (de * cr - dn * sr) * ppn, oy - (de * sr + dn * cr) * ppn]; };
  const plan = valid && w.st.plan ? D.plan : null, hac = plan && plan.hac;

  // ------------------------------------------------ map (clipped to the compass circle)
  g.save();
  g.beginPath(); g.arc(ox, oy, RA - 1, 0, Math.PI * 2); g.clip();
  g.beginPath(); g.rect(0, 0, 512, 468); g.clip();
  if (mapOn) {
    const rw = nav ? nav.rw : runwayInfo(D.runway, gnc, pos);
    // half-range ring
    g.strokeStyle = HC.lbl; g.lineWidth = 1; g.setLineDash([5, 7]); g.beginPath();
    if (headUp) g.arc(ox, oy, RA / 2, -Math.PI / 2 - ARC * D2R, -Math.PI / 2 + ARC * D2R); else g.arc(ox, oy, RA / 2, 0, Math.PI * 2);
    g.stroke(); g.setLineDash([]);
    // runway to scale (at least 3 px wide), extended centreline of the landing end with 1 NM / 5 NM ticks
    {
      const A = S(...rw.thr33), B = S(...rw.thr15), Lp = Math.hypot(B[0] - A[0], B[1] - A[1]) || 1;
      const ux = (B[0] - A[0]) / Lp, uy = (B[1] - A[1]) / Lp, hw = Math.max(1.5, 91 * NM * ppn / 2);
      g.fillStyle = '#000'; g.strokeStyle = HC.dat; g.lineWidth = 1.2;
      g.beginPath(); g.moveTo(A[0] - uy * hw, A[1] + ux * hw); g.lineTo(B[0] - uy * hw, B[1] + ux * hw); g.lineTo(B[0] + uy * hw, B[1] - ux * hw); g.lineTo(A[0] + uy * hw, A[1] - ux * hw); g.closePath(); g.fill(); g.stroke();
      const sg = rw.end === '33' ? -1 : 1, T0 = S(...rw.thr), ext = 10;
      const E = S(rw.thr[0] + sg * rw.u[0] * ext / NM, rw.thr[1] + sg * rw.u[1] * ext / NM);
      const el = Math.hypot(E[0] - T0[0], E[1] - T0[1]) || 1, ex = (E[0] - T0[0]) / el, ey = (E[1] - T0[1]) / el;
      g.strokeStyle = HC.lbl; g.lineWidth = 1; g.setLineDash([7, 5]); line(g, T0[0], T0[1], E[0], E[1]); g.setLineDash([]);
      if (ppn >= 5) for (let k = 1; k <= ext; k++) {
        const px = T0[0] + ex * k * ppn, py = T0[1] + ey * k * ppn, L = k % 5 === 0 ? 7 : 4;
        line(g, px - ey * L, py + ex * L, px + ey * L, py - ex * L);
      }
      // touchdown point (small circle) and the straight-in final from the HAC exit
      const [tx, ty] = S(...nav?.td ?? rw.thr);
      if (nav && nav.exit && hac && !nav.al) { const [hx, hy] = S(...nav.exit); g.strokeStyle = COL.mag; g.lineWidth = 1.4; line(g, hx, hy, tx, ty); }
      g.strokeStyle = HC.dat; g.lineWidth = 1.4; g.fillStyle = '#000'; g.beginPath(); g.arc(tx, ty, 3.5, 0, Math.PI * 2); g.fill(); g.stroke();
      TH(g, rw.end, T0[0] + ex * 16 + 7, T0[1] + ey * 16 + 4, 12, HC.sel);
    }
    // abandoned approach after a HAC switch: inactive (dark grey, dashed), for one minute
    if (plan && plan.prev && plan.prev.hac && (!fin(plan.prev.t) || st.t - plan.prev.t < 60)) {
      const pv = plan.prev, [hx, hy] = S(...pv.hac.center);
      g.strokeStyle = HC.dim; g.lineWidth = 1.2; g.setLineDash([4, 5]);
      g.beginPath(); g.arc(hx, hy, pv.hac.radius * NM * ppn, 0, Math.PI * 2); g.stroke(); g.setLineDash([]);
      TH(g, `PREV HAC ${pv.hac.dir > 0 ? 'R' : 'L'}`, hx, hy + 4, 10.5, HC.dim, 'center');
    }
    // SCP plan from the vehicle to the A/L interface (magenta, amber if the fallback reference)
    if (plan && D.planPts && D.planPts.length > 1 && !(nav && nav.al)) {
      const P = D.planPts;
      let k0 = 0, best = Infinity;
      for (let i = 0; i < P.length; i++) { const q = (P[i][0] - pos[0]) ** 2 + (P[i][1] - pos[1]) ** 2; if (q < best) { best = q; k0 = i; } }
      g.strokeStyle = plan.fallback ? COL.amber : COL.mag; g.lineWidth = 1.2; g.setLineDash([2, 4]);
      g.beginPath(); g.moveTo(...S(P[k0][0], P[k0][1]));
      for (let i = k0 + 1; i < P.length; i++) g.lineTo(...S(P[i][0], P[i][1]));
      g.stroke(); g.setLineDash([]);
    }
    // HAC (magenta), centre cross, turn-direction arrowheads, WP1 tangency
    if (hac && !(nav && nav.al)) {
      const [hx, hy] = S(...hac.center), hr = hac.radius * NM * ppn;
      g.strokeStyle = COL.mag; g.lineWidth = 1.6; g.beginPath(); g.arc(hx, hy, hr, 0, Math.PI * 2); g.stroke();
      g.lineWidth = 1.2; line(g, hx - 4, hy, hx + 4, hy); line(g, hx, hy - 4, hx, hy + 4);
      if (hr > 18) for (const k of [0, 1, 2, 3]) {
        const an = k * Math.PI / 2 + Math.PI / 4, px = hx + Math.cos(an) * hr, py = hy + Math.sin(an) * hr;
        const dir = hac.dir > 0 ? 1 : -1, tx = -Math.sin(an) * dir, ty = Math.cos(an) * dir;
        g.fillStyle = COL.mag; g.beginPath(); g.moveTo(px + tx * 6, py + ty * 6);
        g.lineTo(px - tx * 2 + Math.cos(an) * 4, py - ty * 2 + Math.sin(an) * 4); g.lineTo(px - tx * 2 - Math.cos(an) * 4, py - ty * 2 - Math.sin(an) * 4); g.closePath(); g.fill();
      }
      { const dx = hx - ox, dy = hy - oy, dl = Math.hypot(dx, dy) || 1;     // label on the far side from the orbiter
        TH(g, `HAC ${hac.dir > 0 ? 'R' : 'L'}`, hx + dx / dl * 28, hy + dy / dl * 16 + 4, 10.5, COL.mag, 'center'); }
      if (nav && nav.wp1) {
        const [x, y] = S(...nav.wp1);
        g.strokeStyle = COL.mag; g.fillStyle = '#000'; g.beginPath(); g.moveTo(x, y - 6); g.lineTo(x + 6, y); g.lineTo(x, y + 6); g.lineTo(x - 6, y); g.closePath(); g.fill(); g.lineWidth = 1.6; g.stroke();
        TH(g, 'WP1', x + 9, y + 4, 11, COL.mag);
      }
    }
    // history: dark grey dots every ~300 m
    if (w.st.trail && D.trail && D.trailN) {
      g.fillStyle = HC.dim;
      for (let i = D.trailN - 1, k = 0; i >= 0 && k < 300; i -= 2, k++) { const [x, y] = S(D.trail[i][0], D.trail[i][1]); g.fillRect(x - 1, y - 1, 2, 2); }
    }
    // predictor circles: constant-bank ground path at 20 / 40 / 60 s (turn rate g·tanφ / Vg)
    if (!(nav && nav.gnd)) {
      const gs = Math.max(Math.hypot(st.vel[0], st.vel[1]), 30);
      const om = 9.80665 * Math.tan(clamp((a.bank ?? 0) * D2R, -1.3, 1.3)) / gs;
      let e = pos[0], n = pos[1], psi = trk * D2R;
      g.strokeStyle = HC.dat; g.lineWidth = 1.5; g.fillStyle = '#000';
      for (let t = 1; t <= 60; t++) {
        psi += om; e += gs * Math.sin(psi); n += gs * Math.cos(psi);
        if (t % 20 === 0) { const [x, y] = S(e, n); g.beginPath(); g.arc(x, y, 3.8, 0, Math.PI * 2); g.fill(); g.stroke(); }
      }
    }
  }
  g.restore();

  // ------------------------------------------------ compass (arc in HDG UP, full rose in N UP)
  g.strokeStyle = HC.dat; g.lineWidth = 1.4; g.beginPath();
  if (headUp) g.arc(ox, oy, RA, -Math.PI / 2 - ARC * D2R, -Math.PI / 2 + ARC * D2R); else g.arc(ox, oy, RA, 0, Math.PI * 2);
  g.stroke();
  for (let b = 0; b < 360; b += 5) {
    const rel = wrap180(b - rot);
    if (headUp && Math.abs(rel) > ARC) continue;
    const an = rel * D2R, maj = b % 10 === 0, L = maj ? 9 : 5;
    g.lineWidth = maj ? 1.4 : 1;
    line(g, ox + Math.sin(an) * RA, oy - Math.cos(an) * RA, ox + Math.sin(an) * (RA - L), oy - Math.cos(an) * (RA - L));
    if (b % 30 === 0 || (headUp && maj)) {
      const k = b / 10, lab = k % 9 === 0 ? 'NESW'[k / 9] : `${k}`;
      g.save(); g.translate(ox, oy); g.rotate(an);
      T(g, lab, 0, -RA + 23, b % 30 === 0 ? 14 : 11.5, HC.dat, 'center', '700');
      g.restore();
    }
  }
  // bearing bugs outside the compass (H: HAC tangency, green; C: HAC centre, white; R: touchdown, green)
  const bug = (b, lab, col) => {
    let rel = wrap180(b - rot); const peg = headUp && Math.abs(rel) > ARC; if (peg) rel = Math.sign(rel) * ARC;
    g.save(); g.translate(ox, oy); g.rotate(rel * D2R); g.translate(0, -RA - 2);
    g.beginPath(); g.moveTo(0, 0); g.lineTo(-6.5, -6); g.lineTo(-6.5, -16); g.lineTo(6.5, -16); g.lineTo(6.5, -6); g.closePath();
    g.fillStyle = peg ? '#000' : col; g.fill(); g.strokeStyle = peg ? col : '#000'; g.lineWidth = 1.2; g.stroke();
    g.translate(0, -9.5); g.rotate(-rel * D2R); T(g, lab, 0, 4, 10, peg ? col : '#000', 'center', '700');
    g.restore();
  };
  if (nav && ready) {
    if (nav.bC !== null && !nav.al) bug(nav.bC, 'C', HC.dat);
    if (nav.bH !== null) bug(nav.bH, 'H', COL.green);
    if (nav.bR !== null && !nav.gnd) bug(nav.bR, 'R', COL.green);
  }
  if (valid && ready) {
    // track pointer: hollow diamond inside the scale
    const tr = wrap180(trk - rot);
    if (!headUp || Math.abs(tr) <= ARC) {
      g.save(); g.translate(ox, oy); g.rotate(tr * D2R);
      g.strokeStyle = HC.dat; g.lineWidth = 1.5; g.beginPath(); g.moveTo(0, -RA + 1); g.lineTo(4.5, -RA + 9); g.lineTo(0, -RA + 17); g.lineTo(-4.5, -RA + 9); g.closePath(); g.stroke();
      g.restore();
    }
  }
  // heading readout + lubber line (HDG UP); in N UP the orbiter symbol shows heading
  {
    const by = headUp ? oy - RA - 46 : 44, bh = 22;
    if (headUp) { g.strokeStyle = HC.dat; g.lineWidth = 2.2; line(g, ox, by + bh, ox, oy - RA + 13); }
    dbox(g, 229, by, 54, bh, valid ? HC.dat : COL.amber, 1.3);
    if (valid && ready) T(g, pad(Math.round(hdg) % 360 || 360, 3), 256, by + 18, 18, HC.dat, 'center', '700');
    else if (!valid && D) T(g, 'HDG', 256, by + 17, 14, COL.amber, 'center', '700');
    T(g, headUp ? 'HDG UP' : 'N UP', 222, by + 16, 11, COL.cyan, 'right', '700');
    // range: on the half-range ring (blinks while the IDP rebuilds the map)
    const [lx, ly] = headUp ? [ox - Math.sin((ARC - 4) * D2R) * RA / 2, oy - Math.cos((ARC - 4) * D2R) * RA / 2] : [ox - RA * 0.354, oy - RA * 0.354];
    if (!rebuilding || blink) TH(g, `${R / 2}`, lx - 3, ly + 4, 12, rebuilding ? COL.cyan : HC.lbl, 'right');
    T(g, `${R} NM`, 290, by + 16, 11, COL.cyan, 'left', '700');
  }
  // orbiter (fixed, pointing up in HDG UP) with Nz beside it; both blink above the Nz limit
  if (valid && ready) {
    const nz = a.nz, over = fin(nz) && nz > NZ_LIM;
    if (!over || blink) {
      orbiterSymbol(g, ox, oy, 1.05, HC.dat, headUp ? 0 : hdg * D2R);
      if (fin(nz) && !(nav && nav.gnd)) TH(g, nz.toFixed(1), ox - 15, oy + 6, 12, over ? HC.caut : HC.dat, 'right');
    }
  }
  if (!valid && D) { dbox(g, 218, oy - 50, 76, 28, COL.amber, 1.6); T(g, 'MAP', 256, oy - 29, 19, COL.amber, 'center', '700'); }

  // ------------------------------------------------ top scale: time to HAC intercept / HAC radial error / crosstrack
  if (nav && nav.top && ready) {
    const x0 = 150, x1 = 350, y = 14, tp = nav.top;
    g.strokeStyle = HC.dat; g.lineWidth = 1.3;
    if (tp.mode === 'THAC') {
      // 0 .. 3 (x10 s); zero on the side of the turn; ▽ from 20 s, blinking until 10 s
      const X = (s) => (tp.right ? x1 - (s / 30) * (x1 - x0) : x0 + (s / 30) * (x1 - x0));
      line(g, x0, y, x1, y);
      for (let s = 0; s <= 3; s++) { line(g, X(s * 10), y - 5, X(s * 10), y + 5); T(g, `${s}`, X(s * 10), y + 17, 11, HC.lbl, 'center', '700'); }
      T(g, 'T HAC', X(25), y + 17, 10, HC.lbl, 'center', '700');          // between the 2 and 3 marks
      if (tp.t <= 20 && (tp.t <= 10 || blink)) tri(g, X(Math.max(tp.t, 0)), y - 2, 9, 'down', HC.dat, '#000');
    } else {
      const X = (v) => (x0 + x1) / 2 + clamp(v / tp.fs, -1, 1) * (x1 - x0) / 2;
      line(g, x0, y, x1, y);
      for (const f of [-1, -0.5, 0, 0.5, 1]) line(g, X(f * tp.fs), y - (f === 0 ? 7 : 4), X(f * tp.fs), y + (f === 0 ? 7 : 4));
      const lab = tp.fs >= 5000 ? '5K' : '2.5K';
      T(g, lab, x0, y + 17, 10, HC.lbl, 'center', '700'); T(g, lab, x1, y + 17, 10, HC.lbl, 'center', '700');
      T(g, tp.mode === 'HACR' ? 'ΔR HAC' : 'Y RWY', (x0 + x1) / 2, y + 20, 10, HC.lbl, 'center', '700');
      if (fin(tp.dev)) {
        const pinned = Math.abs(tp.dev) > tp.fs;
        if (!pinned || blink) orbiterSymbol(g, X(tp.dev), y + 1, 0.62, pinned ? HC.caut : HC.dat);
      }
    }
  }

  // ------------------------------------------------ right scale: altitude error (fly-to)
  if (nav && !nav.gnd && ready && gnc && fin(gnc.h_err)) {
    const x = 490, cy = headUp ? 250 : 258, half = 62, fs = (nav.al || nav.prf) ? 1000 : 5000;
    g.fillStyle = '#000'; g.fillRect(x - 16, cy - half - 24, 34, 2 * half + 48);
    g.strokeStyle = HC.dat; g.lineWidth = 1.3; line(g, x, cy - half, x, cy + half);
    for (const f of [-1, -0.5, 0, 0.5, 1]) line(g, x - (f === 0 ? 8 : 5), cy + f * half, x, cy + f * half);
    const lab = fs === 1000 ? '1K' : '5K';
    T(g, lab, x - 2, cy - half - 8, 10, HC.lbl, 'center', '700'); T(g, lab, x - 2, cy + half + 16, 10, HC.lbl, 'center', '700');
    const herr = gnc.h_err * FT, v = -herr / fs, pinned = Math.abs(v) > 1, py = cy + clamp(v, -1, 1) * half;
    if (!pinned || blink) tri(g, x + 1, py, 11, 'left', pinned ? HC.caut : COL.green, '#000');
  }

  // ------------------------------------------------ data blocks
  const LBL = (s, x, y) => T(g, s, x, y, 11, HC.lbl, 'left', '700');
  if (nav && ready) {
    // top left: ground speed, true airspeed, KEAS, Mach; wind (from / kt) with arrow; runway wind components
    LBL('GS', 6, 18); RO(g, 104, 18, `${Math.round(nav.gs)}`);
    LBL('TAS', 6, 35); RO(g, 104, 35, `${Math.round(nav.tas)}`);
    LBL('KEAS', 6, 52); RO(g, 104, 52, `${Math.round(nav.keas)}`);
    LBL('M', 6, 69); RO(g, 104, 69, nav.mach.toFixed(2).replace(/^0/, ''));
    const ws = nav.ws * KT;
    if (ws < 2) RO(g, 104, 90, 'CALM');
    else {
      RO(g, 104, 90, `${pad(Math.round(nav.windFrom) % 360 || 360, 3)}/${Math.round(ws)}`);
      g.save(); g.translate(16, 92); g.rotate((nav.windFrom + 180 - rot) * D2R);
      g.strokeStyle = HC.dat; g.fillStyle = HC.dat; g.lineWidth = 1.8; line(g, 0, 10, 0, -5);
      g.beginPath(); g.moveTo(0, -11); g.lineTo(-4, -4); g.lineTo(4, -4); g.closePath(); g.fill(); g.restore();
    }
    const hw = -nav.tail * KT, xw = nav.xw * KT;                 // surface wind on the landing runway
    LBL(`SFC ${hw >= 0 ? 'HW' : 'TW'}`, 6, 110); RO(g, 104, 110, `${Math.round(Math.abs(hw))}`, nav.appr && hw < -10 ? '↑' : '', HC.dat, 14);
    LBL('SFC XW', 6, 127); RO(g, 104, 127, `${Math.round(Math.abs(xw))}${Math.round(Math.abs(xw)) ? (xw > 0 ? 'L' : 'R') : ''}`, nav.appr && Math.abs(xw) > 15 ? '↑' : '', HC.dat, 14);

    // top right (MEDS HSI readouts): runway + range to touchdown via the HAC, HAC centre range, ΔAZ -> HTA, time to go
    const rx = 492;
    T(g, `${nav.rw.name}${nav.rw.end}`, rx - 108, 18, 12, HC.sel, 'left', '700'); BRO(g, rx, 18, 52, (nav.rngTd * NM).toFixed(1));
    if (fin(nav.rngC)) { LBL('HAC-C', rx - 108, 41); BRO(g, rx, 41, 52, (nav.rngC * NM).toFixed(1)); }
    if (fin(nav.hta) && nav.onHac) { LBL('HTA', rx - 108, 64); BRO(g, rx, 64, 52, `${Math.round(nav.hta)}°`); }
    else if (!nav.al && !nav.gnd) { LBL('ΔAZ', rx - 108, 64); BRO(g, rx, 64, 52, `${Math.round(Math.abs(nav.daz))}°`, HC.dat, Math.abs(nav.daz) > 30 && nav.wp1 ? '↑' : ''); }
    else if (fin(nav.dyRw)) { LBL('ΔY', rx - 108, 64); BRO(g, rx, 64, 52, `${Math.abs(Math.round(nav.dyRw))}${Math.abs(nav.dyRw) < 1 ? '' : nav.dyRw > 0 ? 'R' : 'L'}`, HC.dat, Math.abs(nav.dyRw) > 1000 ? '↑' : ''); }
    if (fin(nav.hta) && !nav.onHac) { LBL('HTA', rx - 108, 85); RO(g, rx - 2, 85, `${Math.round(nav.hta)}°`); }
    if (fin(nav.ttg) && !nav.gnd) { LBL('TTG', rx - 108, 102); RO(g, rx - 2, 102, mmss(nav.ttg)); }

    // bottom left: altitude (K format above 10 000 ft), altitude rate, ground track
    const blk = (x, y, wd, h) => { g.fillStyle = '#000'; g.fillRect(x, y, wd, h); };
    blk(0, 400, 118, 68); blk(394, 400, 118, 68);
    LBL('H', 6, 418); RO(g, 100, 418, fmtFtK(nav.h));
    LBL('Ḣ', 6, 436); RO(g, 100, 436, `${nav.hdot >= 0 ? '+' : ''}${Math.round(nav.hdot)}`, nav.agl < 100 && nav.hdot < -9.6 && !nav.gnd ? '↓' : '');   // touchdown sink limit
    LBL('TRK', 6, 454); RO(g, 100, 454, pad(Math.round(nav.trk) % 360 || 360, 3));
    // bottom right: energy error (corridor +8000 / -4000 ft), speedbrake actual / command
    LBL('ΔE', 400, 418);
    if (!nav.hasGnc) RO(g, 492, 418, '', 'M');
    else if (fin(nav.de) && !nav.gnd) RO(g, 492, 418, `${nav.de >= 0 ? '+' : ''}${Math.round(nav.de)}`, nav.de > 8000 ? '↑' : nav.de < -4000 ? '↓' : '');
    LBL('SB', 400, 436); if (fin(nav.sbA)) RO(g, 492, 436, `${Math.round(clamp(nav.sbA, 0, 100))}`);
    LBL('SB CMD', 400, 454); if (fin(nav.sbC)) RO(g, 492, 454, `${Math.round(clamp(nav.sbC, 0, 100))}`, '', COL.mag);
  }

  // ------------------------------------------------ guidance mode (bottom centre, boxed 10 s after a change)
  if (valid && ready) {
    const [m1, m2] = guidMode(gnc), css = (st.mode || 'AUTO') === 'MANUAL';
    const cells = [[m1 || '', 'ph', 212], [m2 || '', 'sub', 270], [css ? 'CSS' : 'AUTO', 'ctl', 326]];
    for (const [s, k, x] of cells) {
      if (!s) continue;
      if (hs.fma[k]?.s !== s) hs.fma[k] = { s, t: hs.fma[k] ? now : -1e9 };
      const col = s === 'CRASH' ? COL.red : k === 'ctl' ? (css ? HC.caut : HC.lbl) : COL.green;
      TH(g, s, x, 458, 14, col, 'center');
      if (now - hs.fma[k].t < 10000) { const tw = Math.round(textW(s, 14) + 10); g.strokeStyle = HC.dat; g.lineWidth = 1.2; g.strokeRect(Math.round(x - tw / 2) + 0.5, 443.5, tw, 19); }
    }
  }
  // ------------------------------------------------ messages (two highest priority above the mode line)
  if (D && hs.msgs.length && ready) {
    const shown = hs.msgs.slice(0, 2);
    shown.forEach(([id, s, lv], i) => {
      const col = lv === 2 ? COL.red : lv === 1 ? HC.caut : HC.dat, y = 432 - (shown.length - 1 - i) * 19;
      const isNew = now - (hs.seen[id] ?? now) < 5000;
      const tw = Math.round(textW(s, 13) + 10);
      if (isNew && blink) { g.fillStyle = col; g.fillRect(256 - tw / 2, y - 13, tw, 17); T(g, s, 256, y, 13, '#000', 'center', '700'); }
      else TH(g, s, 256, y, 13, col, 'center');
    });
    if (hs.msgs.length > 2) TH(g, `+${hs.msgs.length - 2}`, 256 + 80, 432, 11, HC.dat, 'left');
  }
  // ------------------------------------------------ no display updates for 3 s: big X
  if (D && now - hs.rxT > 3000) {
    g.strokeStyle = COL.red; g.lineWidth = 2.5; line(g, 16, 16, 496, 452); line(g, 496, 16, 16, 452);
    dbox(g, 404, 440, 102, 24, COL.mag, 1.2); T(g, 'POLL FAIL', 455, 457, 13, COL.mag, 'center', '700');
  }
  drawKeyLegends(g, w);
}

// =========================================================================================== SPI
function drawSPI(g, w, d) {
  const st = d.st, s = st?.surf, valid = !!s && d.link !== 'offline';
  const vscale = (x, y0, h, lo, hi, step, lstep, v, lab, digits = 1, showLabels = true, box = true) => {
    const Y = (t) => y0 + (t - lo) / (hi - lo) * h;
    g.strokeStyle = COL.white; g.lineWidth = 1.2; line(g, x, y0, x, y0 + h);
    for (let t = lo; t <= hi + 1e-6; t += step) {
      const yy = Y(t), maj = Math.abs(t / lstep - Math.round(t / lstep)) < 1e-6;
      line(g, x - (maj ? 9 : 5), yy, x, yy);
      if (maj && showLabels) T(g, `${Math.abs(t)}`, x - 13, yy + 4, 11, COL.white, 'right');
    }
    if (lab) T(g, lab, x, y0 + h + 20, 12, COL.white, 'center', '700');
    if (box) dbox(g, x - 20, y0 + h + 28, 40, 17, '#9a9c9e');
    if (!fin(v)) { offFlag(g, x, y0 + h / 2, 30, 14); return; }
    const yy = Y(clamp(v, lo, hi));
    tri(g, x + 1, yy, 11, 'left', COL.white, '#000');
    if (box) T(g, v.toFixed(digits), x + 16, y0 + h + 41, 12, COL.white, 'right');
  };
  T(g, 'ELEVONS', 150, 26, 14, COL.white, 'center', '700'); T(g, 'DEG', 150, 42, 11, COL.gray, 'center');
  T(g, 'UP', 30, 72, 10, COL.gray, 'center'); T(g, 'DN', 30, 330, 10, COL.gray, 'center');
  const el = valid ? [s.elevon_lo, s.elevon_li, s.elevon_ri, s.elevon_ro] : [NaN, NaN, NaN, NaN];
  ['L OB', 'L IB', 'R IB', 'R OB'].forEach((lab, i) => vscale(72 + i * 62, 66, 270, -35, 20, 5, 10, el[i], lab, 1, i === 0));
  const bfp = valid ? (s.bodyflap + 11.7) / 34.2 * 100 : NaN;
  T(g, 'BODY FLAP', 340, 26, 14, COL.white, 'center', '700'); T(g, '%', 340, 42, 11, COL.gray, 'center');
  vscale(350, 66, 270, 0, 100, 5, 20, bfp, 'BF', 0);
  const sbA = valid ? s.speedbrake / 98.6 * 100 : NaN, sbC = valid ? ((st.surf_cmd?.speedbrake ?? st.cmd?.speedbrake ?? s.speedbrake) / 98.6) * 100 : NaN;
  T(g, 'SPEEDBRAKE', 452, 26, 14, COL.white, 'center', '700'); T(g, '%', 452, 42, 11, COL.gray, 'center');
  vscale(440, 66, 200, 0, 100, 5, 20, sbA, '', 0, true, false);
  if (fin(sbC)) { const yy = 66 + clamp(sbC, 0, 100) / 100 * 200; tri(g, 437, yy, 11, 'right', COL.mag, '#000'); }
  T(g, 'ACTUAL', 470, 300, 11, COL.gray, 'right'); dbox(g, 474, 288, 34, 16, '#9a9c9e'); if (fin(sbA)) T(g, pad(Math.round(sbA), 3), 504, 300, 12, COL.white, 'right');
  T(g, 'COMMAND', 470, 322, 11, COL.gray, 'right'); dbox(g, 474, 310, 34, 16, COL.mag); if (fin(sbC)) T(g, pad(Math.round(clamp(sbC, 0, 100)), 3), 504, 322, 12, COL.mag, 'right');
  // rudder / aileron horizontal scales
  const hscale = (y, half, fs, step, lstep, v, lab) => {
    const x0 = 176, X = (t) => x0 + t / fs * half;
    T(g, lab, 20, y - 12, 12, COL.white, 'left', '700');
    g.strokeStyle = COL.white; g.lineWidth = 1.2; line(g, X(-fs), y, X(fs), y);
    for (let t = -fs; t <= fs + 1e-6; t += step) {
      const maj = Math.abs(t / lstep - Math.round(t / lstep)) < 1e-6;
      line(g, X(t), y, X(t), y + (maj ? 9 : 5));
      if (maj) T(g, `${Math.abs(t)}`, X(t), y + 21, 11, COL.white, 'center');
    }
    T(g, 'L', X(-fs) - 10, y + 4, 12, COL.gray, 'right'); T(g, 'R', X(fs) + 10, y + 4, 12, COL.gray, 'left');
    if (!fin(v)) { offFlag(g, x0, y - 8, 30, 14); return; }
    tri(g, X(clamp(v, -fs, fs)), y - 1, 11, 'down', COL.white, '#000');
    dbox(g, X(fs) + 24, y - 9, 46, 17, '#9a9c9e');
    T(g, `${Math.abs(v).toFixed(1)}${v >= 0 ? 'R' : 'L'}`, X(fs) + 66, y + 4, 12, COL.white, 'right');
  };
  hscale(392, 90, 30, 5, 10, valid ? -s.rudder : NaN, 'RUDDER');
  const ail = valid ? ((s.elevon_li + s.elevon_lo) / 2 - (s.elevon_ri + s.elevon_ro) / 2) / 2 : NaN;
  hscale(436, 90, 5, 1, 1, ail, 'AILERON');
  // mechanical status
  if (valid && st.gear) {
    const gp = st.gear.pos, wow = st.gear.wow || [];
    T(g, 'GEAR', 392, 376, 11, COL.gray); T(g, gp > 0.98 ? 'DN' : gp < 0.02 ? 'UP' : `${Math.round(gp * 100)}%`, 506, 376, 12, gp > 0.98 ? COL.green : gp < 0.02 ? COL.white : COL.yellow, 'right', '700');
    T(g, 'WOW', 392, 394, 11, COL.gray); ['N', 'L', 'R'].forEach((k, i) => T(g, k, 470 + i * 15, 394, 12, wow[i] ? COL.green : '#555', 'center', '700'));
    const ch = ['STOW', 'PILOT', 'REEF', 'DPY'][st.chute | 0] || 'DPY';
    T(g, 'D/C', 392, 412, 11, COL.gray); T(g, st.chute_jettisoned ? 'JETT' : ch, 506, 412, 12, st.chute_jettisoned ? COL.yellow : st.chute ? COL.green : COL.white, 'right', '700');
    T(g, 'BRAKE', 392, 430, 11, COL.gray); T(g, `${Math.round((st.brakes || 0) * 100)}%`, 506, 430, 12, COL.white, 'right');
    T(g, 'NWS', 392, 448, 11, COL.gray); T(g, `${(st.steer ?? 0).toFixed(1)}`, 506, 448, 12, COL.white, 'right');
  }
  drawKeyLegends(g, w);
}

// =========================================================================================== DPS pages
function dpsHeader(scr, d, w, spec, title) {
  const now = new Date();
  const doy = Math.floor((Date.UTC(now.getUTCFullYear(), now.getUTCMonth(), now.getUTCDate()) - Date.UTC(now.getUTCFullYear(), 0, 0)) / 864e5);
  const gmt = `${pad(doy, 3)}/${pad(now.getUTCHours(), 2)}:${pad(now.getUTCMinutes(), 2)}:${pad(now.getUTCSeconds(), 2)}`;
  const t = Math.max(0, d.st?.t || 0), met = `000/${pad(Math.floor(t / 3600), 2)}:${pad(Math.floor(t / 60) % 60, 2)}:${pad(Math.floor(t) % 60, 2)}`;
  scr.put(0, 0, `3051/${spec}/`);
  scr.put(0, Math.round(25 - title.length / 2), title);
  scr.put(0, 37, `${w.spec.idp} ${gmt}`);
  scr.put(1, 39, met);
  // fault message line: the last row above the edge-key legends (rows 24-25 lie under them, y > 468)
  if (w.m.fault && !w.m.faultAck) scr.put(DPS_FAULT_ROW, 0, w.m.fault.slice(0, 51), w.m.faultNew() ? 1 : 0);
}
const fx = (v, w, dgt = 0, sign = false) => {
  if (!fin(v)) return ' '.repeat(Math.max(0, w - 1)) + '-';
  const s = (sign && v >= 0 ? '+' : '') + v.toFixed(dgt);
  return s.length > w ? '*'.repeat(w) : s.padStart(w);
};

function drawHorizSit(g, w, d) {
  const scr = w.dps; scr.clear();
  const st = d.st, a = st?.air, gnc = st?.gnc, valid = !!a && d.link !== 'offline';
  dpsHeader(scr, d, w, '050', 'HORIZ SIT');
  const rw = runwayInfo(d.runway, gnc, valid ? st.pos : null);
  scr.put(2, 1, `LAND SITE   1  ${rw.name}`);
  scr.put(3, 1, ` PRI RW ${rw.end === '33' ? '33*' : '33 '}   SEC RW ${rw.end === '15' ? '15*' : '15 '}`);
  scr.put(2, 34, 'G&N    HSI');
  scr.put(3, 34, ` ${d.link === 'online' ? 'AUTO' : d.link === 'local' ? 'PREV' : 'NONE'}   ${isAL(gnc) ? 'APPR' : 'TAEM'}`);
  const hac = d.plan && d.plan.hac;
  scr.put(4, 1, `TAEM TGT  6 OVHD*  7 STRT    HAC ${hac ? (hac.dir > 0 ? 'R' : 'L') : '-'}`);
  scr.put(5, 1, ` HAC RAD ${hac ? fx(hac.radius * FT / 1000, 5, 1) : '    -'} K  CNTR ${hac && valid ? fx(Math.hypot(st.pos[0] - hac.center[0], st.pos[1] - hac.center[1]) * NM, 5, 1) : '    -'} NM`);
  if (valid) {
    const along = rw.end === '33' ? st.rwy.along + rw.L / 2 : rw.L / 2 - st.rwy.along;
    const cross = rw.end === '33' ? st.rwy.cross : -st.rwy.cross;
    scr.put(7, 1, 'NAV   X        Y        Z      (RWY, KFT)');
    scr.put(8, 1, `   ${fx(along * FT / 1000, 7, 2, true)}  ${fx(cross * FT / 1000, 7, 2, true)}  ${fx(a.alt * FT / 1000, 7, 2)}`);
    const gs = Math.hypot(st.vel[0], st.vel[1]);
    scr.put(9, 1, `   VG ${fx(gs * FT, 6)}  ḢDOT ${fx(a.vs * FT, 6, 0, true)}  HDG ${pad(Math.round(wrap360(a.psi)) % 360, 3)}`);
    const dthr = Math.hypot(st.pos[0] - rw.thr[0], st.pos[1] - rw.thr[1]);
    const tp = hac && !isAL(gnc) ? hacTangent(st.pos, hac) : null;
    const daz = wrap180(brgTo(st.pos, tp || rw.thr) - a.track);
    scr.put(10, 1, `   RNG ${fx(dthr * NM, 5, 1)}  ΔAZ ${fx(Math.abs(daz), 5, 1)}${daz >= 0 ? 'R' : 'L'}  TTG ${gnc ? mmss(gnc.time_to_go) : '--:--'}`);
    const e = gnc && fin(gnc.energy_err) ? gnc.energy_err * FT : NaN;
    scr.put(11, 1, `   ΔE ${fx(e, 6, 0, true)} FT  `);
    if (fin(e) && Math.abs(e) > 3000) scr.put(11, 16, e > 0 ? '↑' : '↓', 1);
    scr.put(11, 22, `REACH ${gnc ? (gnc.reachable === false ? 'NO ' : 'YES') : ' - '}`, gnc && gnc.reachable === false ? 2 : 0);
    const wnd = a.wind || [0, 0, 0], ws = Math.hypot(wnd[0], wnd[1]);
    scr.put(12, 1, `   WIND ${pad(Math.round(wrap360(Math.atan2(-wnd[0], -wnd[1]) * R2D)) % 360, 3)}/${pad(Math.round(ws * KT), 2)}  TURB ${fx(st.env?.turb, 4, 2)}`);
  } else scr.put(9, 12, 'NAV DATA NOT AVAILABLE', 2);
  scr.put(13, 1, ' TACAN  1  2  3     GPS  1  2  3     MLS  1  2  3');
  scr.put(14, 1, `        ${valid ? 'A  A  A' : '-  -  -'}           ${valid ? 'A  A  A' : '-  -  -'}           ${valid && isAL(gnc) ? 'A  A  A' : '-  -  -'}`);
  scr.draw(g);
  // graphic: runway-centred horizontal situation, fixed north-up, 20 NM square
  const gx0 = 20, gy0 = 302, gw = 472, gh = 140, cx = gx0 + gw / 2, cy = gy0 + gh / 2;   // ends above the fault row
  g.save(); g.beginPath(); g.rect(gx0, gy0, gw, gh); g.clip();
  g.strokeStyle = COL.dpsDim; g.lineWidth = 1; g.strokeRect(gx0 + 0.5, gy0 + 0.5, gw - 1, gh - 1);
  const ppn = gh / 2 / 12;
  const S = (e, n) => [cx + (e - rw.c[0]) * NM * ppn, cy - (n - rw.c[1]) * NM * ppn];
  g.strokeStyle = COL.dps; g.lineWidth = 3;
  { const A = S(...rw.thr33), B = S(...rw.thr15); line(g, A[0], A[1], B[0], B[1]); }
  g.lineWidth = 1; g.setLineDash([3, 4]);
  { const A = S(rw.thr[0] - (rw.end === '33' ? 1 : -1) * rw.u[0] * 10 / NM, rw.thr[1] - (rw.end === '33' ? 1 : -1) * rw.u[1] * 10 / NM), B = S(...rw.thr); line(g, A[0], A[1], B[0], B[1]); }
  g.setLineDash([]);
  if (hac) { const [hx, hy] = S(...hac.center); g.beginPath(); g.arc(hx, hy, hac.radius * NM * ppn, 0, Math.PI * 2); g.stroke(); }
  if (valid) {
    const [px, py] = S(st.pos[0], st.pos[1]);
    const on = px > gx0 && px < gx0 + gw && py > gy0 && py < gy0 + gh;
    if (on) {
      g.save(); g.translate(px, py); g.rotate(a.track * D2R); g.fillStyle = COL.dpsHi;
      g.beginPath(); g.moveTo(0, -7); g.lineTo(5, 5); g.lineTo(-5, 5); g.closePath(); g.fill();
      // 20 s / 40 s / 60 s predictors
      const gs = Math.hypot(st.vel[0], st.vel[1]) * NM * ppn;
      g.fillStyle = COL.dps; for (const t of [20, 40, 60]) g.fillRect(-1.5, -gs * t - 1.5, 3, 3);
      g.restore();
    } else {
      const b = Math.atan2(px - cx, -(py - cy));
      const ex = cx + Math.sin(b) * (gh / 2 - 8) * (gw / gh), ey = cy - Math.cos(b) * (gh / 2 - 8);
      g.save(); g.translate(clamp(ex, gx0 + 10, gx0 + gw - 10), clamp(ey, gy0 + 10, gy0 + gh - 10)); g.rotate(b);
      g.strokeStyle = COL.dpsHi; g.lineWidth = 1.5; g.beginPath(); g.moveTo(0, -7); g.lineTo(5, 5); g.lineTo(-5, 5); g.closePath(); g.stroke(); g.restore();
    }
  }
  g.restore();
  T(g, '12 NM', gx0 + 4, gy0 + 12, 11, COL.dpsDim, 'left', '', MONO);
  drawKeyLegends(g, w);
}

function drawVertSit(g, w, d) {
  const scr = w.dps; scr.clear();
  const st = d.st, a = st?.air, gnc = st?.gnc, valid = !!a && d.link !== 'offline';
  const rtg = gnc && fin(gnc.range_to_go) && gnc.range_to_go > 0 ? gnc.range_to_go : null;
  const h = valid ? a.alt * FT : NaN;
  const two = (rtg !== null ? rtg * NM < 12 : false) && h < 22000;
  dpsHeader(scr, d, w, '   ', two ? 'VERT SIT 2' : 'VERT SIT 1');
  // right-hand data column
  if (valid) {
    const sbA = st.surf ? st.surf.speedbrake / 98.6 * 100 : NaN, sbC = ((st.surf_cmd?.speedbrake ?? st.cmd?.speedbrake ?? NaN) / 98.6) * 100;
    const rows = [['SB', fx(sbA, 4)], [' CMD', fx(sbC, 4)], ['QBAR', fx(a.qbar * 0.0208854, 4)], ['α', fx(a.alpha, 5, 1)], ['NZ', fx(a.nz, 5, 2)],
      ['Ḣ', fx(a.vs * FT, 5, 0, true)], ['KEAS', fx(a.keas, 4)], ['M', fx(a.mach, 5, 2)], ['ΔE', fx(gnc?.energy_err * FT, 6, 0, true)], ['RNG', fx(rtg !== null ? rtg * NM : NaN, 5, 1)]];
    rows.forEach(([k, v], i) => { scr.put(3 + i * 2, 40, k); scr.put(3 + i * 2, 45, v, k === 'QBAR' && a.qbar > 14000 ? 2 : 0); });
  }
  scr.draw(g);
  // plot
  const X0 = 46, X1 = 384, Y0 = 52, Y1 = 412;                 // axis labels end above the fault row
  const xmax = two ? 12 : 60, ymax = two ? 25 : 100;                         // NM, kft
  const X = (r) => X0 + clamp(r / xmax, 0, 1.02) * (X1 - X0);
  const Y = (hk) => Y1 - clamp(hk / ymax, -0.02, 1.02) * (Y1 - Y0);
  g.strokeStyle = COL.dpsDim; g.lineWidth = 1; g.strokeRect(X0 + 0.5, Y0 + 0.5, X1 - X0, Y1 - Y0);
  g.setLineDash([2, 6]);
  for (let r = 0; r <= xmax; r += two ? 2 : 10) line(g, X(r), Y0, X(r), Y1);
  for (let hk = 0; hk <= ymax; hk += two ? 5 : 20) line(g, X0, Y(hk), X1, Y(hk));
  g.setLineDash([]);
  for (let r = 0; r <= xmax; r += two ? 4 : 20) T(g, `${r}`, X(r), Y1 + 14, 11, COL.dps, 'center', '', MONO);
  for (let hk = 0; hk <= ymax; hk += two ? 5 : 20) T(g, `${hk}`, X0 - 5, Y(hk) + 4, 11, COL.dps, 'right', '', MONO);
  T(g, 'H KFT', X0 - 4, Y0 - 6, 10.5, COL.dps, 'left', '', MONO);
  T(g, 'RNG NM', X1, Y1 + 28, 10.5, COL.dps, 'right', '', MONO);
  // energy boundary lines: MAX L/D (shallowest reachable), NOM (SCP plan), S-TURN (steepest)
  const lineFn = (f, lab, dash) => {
    g.strokeStyle = COL.dps; g.lineWidth = 1.3; g.setLineDash(dash); g.beginPath();
    for (let k = 0; k <= 60; k++) { const r = k / 60 * xmax, hk = f(r); k ? g.lineTo(X(r), Y(hk)) : g.moveTo(X(r), Y(hk)); }
    g.stroke(); g.setLineDash([]);
    let r = xmax * 0.92; while (f(r) > ymax * 0.9 && r > 0) r -= xmax / 100;
    T(g, lab, X(r) - 4, Y(f(r)) - 5, 10.5, COL.dps, 'right', '', MONO);
  };
  lineFn((r) => r * 6.076 / 4.6, 'MAX L/D', [6, 4]);
  lineFn((r) => r * 6.076 / 2.2, 'S-TURN', [2, 4]);
  // nominal from the plan (range-to-go along plan + approach)
  const plan = d.plan;
  if (plan && plan.nodes && plan.nodes.length > 1) {
    const ap = plan.approach || [];
    let Lapp = 0; for (let i = 1; i < ap.length; i++) Lapp += Math.hypot(ap[i][0] - ap[i - 1][0], ap[i][1] - ap[i - 1][1]);
    const pts = []; let acc = Lapp;
    for (let i = 0; i < ap.length; i++) { if (i) acc -= Math.hypot(ap[i][0] - ap[i - 1][0], ap[i][1] - ap[i - 1][1]); pts.push([acc, ap[i][2]]); }
    const nd = plan.nodes; let dd = Lapp; const pr = [];
    for (let i = nd.length - 1; i >= 0; i--) { if (i < nd.length - 1) dd += Math.hypot(nd[i + 1][1] - nd[i][1], nd[i + 1][2] - nd[i][2]); pr.push([dd, nd[i][3]]); }
    g.strokeStyle = COL.dpsHi; g.lineWidth = 1.6; g.beginPath();
    [...pts.reverse(), ...pr].forEach(([r, hh], i) => { const x = X(r * NM), y = Y(hh * FT / 1000); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.stroke();
    const r = Math.min(xmax * 0.6, pr.length ? pr[pr.length - 1][0] * NM : 0);
    T(g, 'NOM', X(r) + 4, Y((pr.find(([q]) => q * NM >= r) || [0, 0])[1] * FT / 1000) + 14, 10.5, COL.dpsHi, 'left', '', MONO);
  }
  // history
  g.fillStyle = COL.dps;
  (d.prof || []).forEach(([r, hh], i, arr) => { if ((arr.length - i) % 3 === 0) g.fillRect(X(r * NM) - 1.5, Y(hh * FT / 1000) - 1.5, 3, 3); });
  // orbiter symbol (points along the flight path)
  if (valid && rtg !== null) {
    const x = X(rtg * NM), y = Y(h / 1000);
    const gam = (a.gamma ?? 0) * D2R;
    g.save(); g.translate(x, y); g.rotate(-Math.PI / 2 - Math.atan2(Math.sin(gam) * 3, Math.cos(gam)));
    g.fillStyle = COL.dpsHi; g.beginPath(); g.moveTo(0, -8); g.lineTo(6, 6); g.lineTo(-6, 6); g.closePath(); g.fill();
    g.restore();
  } else if (!valid) T(g, 'NO DATA', (X0 + X1) / 2, (Y0 + Y1) / 2, 14, COL.dps, 'center', '', MONO);
  drawKeyLegends(g, w);
}

function drawGncSumm(g, w, d) {
  const scr = w.dps; scr.clear();
  const st = d.st, a = st?.air, gnc = st?.gnc, valid = !!a && d.link !== 'offline';
  dpsHeader(scr, d, w, '   ', 'GNC SUMM');
  const phases = ['PLAN', 'TAEM', 'HAC', 'OGS', 'FLARE', 'ROLL'];
  const key = gnc ? (() => {
    const t = `${gnc.phase} ${gnc.sub || ''}`;
    if (/ROLL|STOP/.test(t)) return 5; if (/FLARE|TOUCH|IGS/.test(t)) return 4; if (/OGS|A\/L/.test(t)) return 3; if (/HAC|PRF/.test(t)) return 2; if (/TAEM|ACQ/.test(t)) return 1; return 0;
  })() : -1;
  scr.put(2, 1, 'GUID PHASE');
  phases.forEach((p, i) => scr.put(3, 2 + i * 8, p, i === key ? 2 : i < key ? 0 : 3));
  scr.put(5, 1, `PHASE ${gnc ? (gnc.phase + (gnc.sub ? '/' + gnc.sub : '')).padEnd(18) : '-'}   RWY ${gnc?.rwy ?? '--'}`);
  scr.put(6, 1, `MODE  ${(st?.mode || '-').padEnd(8)}  GUID ${(st?.guidance || '-').split(':')[0].slice(0, 12)}`);
  scr.put(8, 1, '            REF      ACT      ERR');
  if (valid) {
    const r = gnc?.ref || {};
    const row = (i, k, rv, av, dg = 1) => scr.put(9 + i, 1, `${k.padEnd(8)}${fx(rv, 9, dg)}${fx(av, 9, dg)}${fx(fin(rv) && fin(av) ? av - rv : NaN, 9, dg, true)}`);
    row(0, 'ALPHA', r.alpha, a.alpha); row(1, 'BANK', r.bank, a.bank); row(2, 'SB DEG', r.sb, st.surf?.speedbrake, 0);
    row(3, 'H KFT', fin(r.h) ? r.h * FT / 1000 : NaN, a.alt * FT / 1000, 2); row(4, 'V FPS', fin(r.V) ? r.V * FT : NaN, a.tas * FT, 0);
    const sc = gnc?.scp || {};
    scr.put(15, 1, `SCP  ${sc.iter !== undefined ? (sc.converged ? 'CONV ' : sc.fallback ? 'FALBK' : 'NCONV') : '  -  '}  IT ${fx(sc.iter, 2)}  ${fx(sc.ms, 5)} MS  AGE ${fx(sc.age, 4)} S`, sc.fallback ? 2 : 0);
    scr.put(16, 1, `     DEFECT ${sc.defect !== undefined ? sc.defect.toExponential(1).toUpperCase() : '-'}   ${sc.pending ? 'SOLVING' : '       '}`);
    const mp = gnc?.mpc || {};
    scr.put(17, 1, `MPC  ${(mp.status || '-').toUpperCase().slice(0, 10).padEnd(10)} ${fx(mp.ms, 5, 1)} MS  TRK ERR ${fx(gnc?.track_err, 6, 0)} M`);
    scr.put(19, 1, `NZ ${fx(a.nz, 5, 2)}  NY ${fx(a.ny, 5, 2)}  QBAR ${fx(a.qbar * 0.0208854, 4)}  L/D ${fx(a.ld, 5, 2)}`);
    const wow = st.gear?.wow || [];
    scr.put(21, 1, `GEAR ${st.gear ? (st.gear.pos > 0.98 ? 'DN ' : st.gear.pos < 0.02 ? 'UP ' : 'TRN') : ' - '}  WOW ${wow.map((x) => (x ? '1' : '0')).join(' ')}  CHUTE ${st.chute_jettisoned ? 'JETT' : st.chute ? 'DPY ' : 'STOW'}  BRK ${fx((st.brakes || 0) * 100, 3)}`);
    scr.put(22, 1, `STATUS ${(st.status || '-').toUpperCase()}`, /CRASH/i.test(st.status || '') ? 2 : 0);
  } else scr.put(10, 14, 'GNC DATA NOT AVAILABLE', 2);
  scr.draw(g);
  drawKeyLegends(g, w);
}

// =========================================================================================== MEDS MAINT
function drawMaint(g, w, d) {
  T(g, 'MEDS MAINT', 256, 26, 16, COL.white, 'center', '700');
  T(g, 'MDU / IDP STATUS', 256, 46, 12, COL.cyan, 'center');
  const ports = [['CDR1', 1], ['CDR2', 2], ['CRT1', 1], ['CRT2', 2], ['CRT3', 3], ['MFD1', 3], ['MFD2', 2], ['PLT1', 2], ['PLT2', 1], ['CRT4', 4], ['AFD1', 4]];
  T(g, 'MDU', 30, 76, 12, COL.gray); T(g, 'PORT', 100, 76, 12, COL.gray); T(g, 'IDP', 160, 76, 12, COL.gray); T(g, 'STATUS', 210, 76, 12, COL.gray);
  ports.forEach(([p, idp], i) => {
    const y = 96 + i * 18;
    const win = w.m.wins.find((x) => x.spec.port.replace(' ', '') === p);
    const status = win ? (win.st.open && win.st.power ? 'ON' : win.st.power ? 'ON/HID' : 'OFF') : 'STBY';
    T(g, p, 30, y, 12, win === w ? COL.green : COL.white); T(g, 'PRI', 100, y, 12); T(g, `${idp}`, 166, y, 12);
    T(g, status, 210, y, 12, status === 'OFF' ? COL.gray : status === 'STBY' ? COL.gray : COL.green);
  });
  const now = performance.now(), up = (now - (w.powerT || now)) / 1000;
  const temp = 22 + 17 * (1 - Math.exp(-up / 240));
  const rows = [['THIS MDU', w.spec.port], ['IDP', `${w.spec.idp}  PRI`], ['BITE', w.bitePass ? 'PASS' : 'RUN'], ['ON TIME', mmss(up)],
    ['TEMP', `${temp.toFixed(1)} C`], ['BACKLIGHT', `${Math.round(w.st.brt * 100)} %`], ['FRAME', `${Math.round(w.fps)} HZ`], ['RES', `${w.cv.width} x ${w.cv.height}`],
    ['LINK', d.link === 'online' ? 'GPC OK' : d.link === 'local' ? 'PREVIEW' : 'POLL FAIL'], ['SW', 'OI-34 / MEDS 3.2']];
  rows.forEach(([k, v], i) => { T(g, k, 312, 96 + i * 22, 12, COL.gray); T(g, v, 500, 96 + i * 22, 13, k === 'LINK' && d.link === 'offline' ? COL.red : COL.white, 'right'); });
  // grey ramp + primaries (display health check strip)
  for (let i = 0; i < 16; i++) { const v = Math.round(i / 15 * 255); g.fillStyle = `rgb(${v},${v},${v})`; g.fillRect(30 + i * 28, 400, 28, 22); }
  [COL.red, COL.green, '#2a5cff', COL.yellow, COL.mag, COL.cyan, COL.white].forEach((c, i) => { g.fillStyle = c; g.fillRect(30 + i * 64, 428, 60, 20); });
  drawKeyLegends(g, w);
}

// =========================================================================================== boot / self-test / poll fail
function drawBoot(g, w, e) {
  g.fillStyle = '#000'; g.fillRect(0, 0, 512, 512);
  if (e < 0.35) { g.fillStyle = `rgba(22,26,30,${e / 0.35})`; g.fillRect(0, 0, 512, 512); return; }
  if (e < 0.62) {                                                     // LCD drive coming up: noisy rows
    drawBacklight(g);
    for (let i = 0; i < 40; i++) { const y = Math.random() * 512; g.fillStyle = `rgba(${150 + Math.random() * 100},${150 + Math.random() * 100},${160 + Math.random() * 95},${Math.random() * 0.5})`; g.fillRect(0, y, 512, 1 + Math.random() * 5); }
    return;
  }
  if (e < 0.78) { g.fillStyle = '#e8e8e8'; g.fillRect(0, 0, 512, 512); return; }  // all-white pixel test
  drawBacklight(g);
  const f = clamp((e - 0.78) / 1.5, 0, 1);
  T(g, 'MEDS', 256, 170, 54, COL.white, 'center', '700');
  T(g, 'MULTIFUNCTION DISPLAY UNIT', 256, 200, 14, COL.gray, 'center');
  T(g, `${w.spec.port}   IDP ${w.spec.idp}   PORT PRI`, 256, 230, 13, COL.cyan, 'center');
  T(g, f < 1 ? 'BITE IN PROGRESS' : 'BITE PASS', 256, 290, 14, f < 1 ? COL.yellow : COL.green, 'center', '700');
  g.strokeStyle = COL.white; g.lineWidth = 1; g.strokeRect(126.5, 304.5, 260, 10);
  g.fillStyle = COL.white; g.fillRect(128, 306, 257 * f, 7);
  [COL.red, COL.green, '#2a5cff', COL.white].forEach((c, i) => { g.fillStyle = c; g.fillRect(146 + i * 58, 350, 46, 30); });
  for (let i = 0; i < 8; i++) { const v = Math.round(i / 7 * 255); g.fillStyle = `rgb(${v},${v},${v})`; g.fillRect(146 + i * 27.5, 388, 27.5, 14); }
}
function drawPollFail(g) {
  T(g, 'POLL FAIL', 256, 250, 34, COL.mag, 'center', '700');
  T(g, 'IDP NOT RESPONDING', 256, 280, 13, COL.mag, 'center');
}

// =========================================================================================== window
const WIN_SPECS = [
  { id: 'pfd', port: 'CDR 1', idp: 1, hotkey: 'F', label: 'PFD', page: 'PFD', menu: 'FLT', ctl: 'pfd' },
  { id: 'nav', port: 'CDR 2', idp: 2, hotkey: 'N', label: 'NAV', page: 'HSIT', menu: 'HSIT', ctl: 'nav' },
];
const BOOT_S = 2.6;

class MduWindow {
  constructor(m, spec, saved, def) {
    this.m = m; this.spec = spec;
    // every MDU can show every page: keep a full switch set (an MDU without the ADI switches on its
    // panel shows the PFD with the defaults LVLH / MED / MED; one without the map switch: HDG UP)
    const sw = { att: 1, err: 1, rate: 1, orient: 0 };
    this.st = Object.assign({ open: true, collapsed: false, power: true, brt: 0.78, page: spec.page, menu: spec.menu, rng: 3, plan: true, trail: true }, def, saved || {});
    this.st.sw = Object.assign(sw, (saved && saved.sw) || {});
    if (!PAGES[this.st.page]) this.st.page = spec.page;
    if (!MENUS[this.st.menu]) this.st.menu = spec.menu;
    this.keyFlash = []; this.dps = new Dps(); this.fps = 30; this._fpsT = 0; this._fpsN = 0; this._held = {}; this.nowT = 0;
    this.lastDraw = 0; this.dirty = true; this.bootT0 = null; this.bitePass = false; this.powerT = performance.now();
    this._build();
    if (this.st.power) this._startBoot(); else this.scr.classList.add('off');
    this.place();
    this._applyBrt();
    this._syncUi();
  }

  _ctlHtml() {
    const tg = (c, plac, labs) => `<div class="grp tg" data-c="${c}" data-n="${labs.length}"><div class="plac">${plac}</div><div class="tg-body"><div class="tg-sw"><i class="tg-nut"></i><i class="tg-lev"></i></div><div class="tg-pos n${labs.length}">${labs.map((l) => `<span>${l}</span>`).join('')}</div></div></div>`;
    const kn = (c, plac) => `<div class="grp kn" data-c="${c}"><div class="plac">${plac}</div><div class="kn-dial"><svg viewBox="-50 -50 100 100"></svg><div class="kn-skirt"></div><div class="kn-cap"></div></div></div>`;
    if (this.spec.ctl === 'pfd') return tg('att', 'ADI ATTITUDE', ['INRTL', 'LVLH', 'REF']) + tg('err', 'ADI ERROR', ['HIGH', 'MED', 'LOW']) + tg('rate', 'ADI RATE', ['HIGH', 'MED', 'LOW']) + tg('pwr', 'MDU PWR', ['ON', 'OFF']) + kn('brt', 'BRT');
    return tg('orient', 'MAP', ['HDG UP', 'N UP']) + kn('rng', 'RANGE NM') + tg('pwr', 'MDU PWR', ['ON', 'OFF']) + kn('brt', 'BRT');
  }

  _build() {
    const el = document.createElement('div');
    el.className = 'mdu-win closed';
    el.dataset.id = this.spec.id;
    el.innerHTML = `
      <div class="mdu-strip" title="拖动移动 · 双击折叠">
        <span class="mdu-grip"></span><span class="mdu-port">${this.spec.port}</span><span class="mdu-ttl"></span>
        <span class="mdu-sp"></span><i class="mdu-led"></i>
        <button class="mdu-wbtn" data-act="collapse" title="折叠 / 展开"><b></b></button>
        <button class="mdu-wbtn" data-act="close" title="关闭（${this.spec.hotkey}）">✕</button>
      </div>
      <div class="mdu-fold"><div class="mdu-fold-in"><div class="mdu-panel">
        <div class="mdu-bezel">
          <i class="dzus tl"></i><i class="dzus tr"></i><i class="dzus bl"></i><i class="dzus br"></i>
          <div class="mdu-scr off"><canvas></canvas><div class="mdu-glass"></div></div>
          <div class="mdu-keys">${[0, 1, 2, 3, 4, 5].map((i) => `<button class="ek" data-k="${i}" tabindex="-1"></button>`).join('')}</div>
          <div class="mdu-bzl"><span>MDU</span><i>MEDS</i><span>${this.spec.port.replace(' ', '')}</span></div>
        </div>
        <div class="mdu-ctl">${this._ctlHtml()}</div>
      </div></div></div>
      <div class="mdu-rsz" title="缩放"></div>`;
    document.body.appendChild(el);
    this.el = el;
    this.scr = el.querySelector('.mdu-scr');
    this.cv = el.querySelector('canvas');
    this.g = this.cv.getContext('2d');
    this.led = el.querySelector('.mdu-led');
    this.ttl = el.querySelector('.mdu-ttl');
    this.scr.classList.toggle('off', !this.st.power);
    new ResizeObserver(() => this._resizeCanvas()).observe(this.scr);
    // ---- drag (anywhere on the hardware that is not a control)
    el.addEventListener('pointerdown', (e) => {
      this.m.front(this);
      if (e.button !== 0 || e.target.closest('.ek,.tg-sw,.kn-dial,.mdu-wbtn,.mdu-rsz')) return;
      e.preventDefault();
      const ox = e.clientX - this.st.x, oy = e.clientY - this.st.y;
      try { el.setPointerCapture(e.pointerId); } catch { /* */ } el.classList.add('dragging');
      const mv = (ev) => { this.st.x = ev.clientX - ox; this.st.y = ev.clientY - oy; this.place(); };
      const upf = () => { el.removeEventListener('pointermove', mv); el.removeEventListener('pointerup', upf); el.removeEventListener('pointercancel', upf); el.classList.remove('dragging'); this.m.save(); };
      el.addEventListener('pointermove', mv); el.addEventListener('pointerup', upf); el.addEventListener('pointercancel', upf);
    });
    el.querySelector('.mdu-strip').addEventListener('dblclick', (e) => { if (!e.target.closest('.mdu-wbtn')) this.setCollapsed(!this.st.collapsed); });
    el.querySelector('[data-act=collapse]').addEventListener('click', () => { this.m.clicks.play('wbtn'); this.setCollapsed(!this.st.collapsed); });
    el.querySelector('[data-act=close]').addEventListener('click', () => { this.m.clicks.play('wbtn'); this.setOpen(false); });
    // ---- resize (aspect locked: the whole unit scales)
    const rsz = el.querySelector('.mdu-rsz');
    rsz.addEventListener('pointerdown', (e) => {
      e.preventDefault(); e.stopPropagation(); this.m.front(this);
      try { rsz.setPointerCapture(e.pointerId); } catch { /* */ }
      const x0 = e.clientX, y0 = e.clientY, w0 = this.st.w, h0 = el.offsetHeight;
      const mv = (ev) => { const dw = ((ev.clientX - x0) + (ev.clientY - y0) * (w0 / h0)) / 2; this.st.w = clamp(w0 + dw, 260, Math.min(1100, innerWidth - 16)); this.place(); };
      const upf = () => { rsz.removeEventListener('pointermove', mv); rsz.removeEventListener('pointerup', upf); this.m.save(); };
      rsz.addEventListener('pointermove', mv); rsz.addEventListener('pointerup', upf);
    });
    // ---- edge keys
    el.querySelectorAll('.ek').forEach((b) => {
      b.addEventListener('pointerdown', (e) => { e.preventDefault(); b.classList.add('down'); this.m.clicks.play('key'); });
      const rel = () => b.classList.remove('down');
      b.addEventListener('pointerup', rel); b.addEventListener('pointerleave', rel);
      b.addEventListener('click', () => this.edgeKey(+b.dataset.k));
    });
    // ---- toggle switches
    el.querySelectorAll('.tg').forEach((t) => {
      const c = t.dataset.c, n = +t.dataset.n, sw = t.querySelector('.tg-sw');
      const set = (v) => {
        v = clamp(v, 0, n - 1);
        const cur = this._tgGet(c);
        if (v === cur) return;
        this.m.clicks.play(c === 'pwr' ? 'power' : 'toggle');
        this._tgSet(c, v);
      };
      sw.addEventListener('click', (e) => { const r = sw.getBoundingClientRect(); set(this._tgGet(c) + (e.clientY < r.top + r.height / 2 ? -1 : 1)); });
      sw.addEventListener('wheel', (e) => { e.preventDefault(); set(this._tgGet(c) + (e.deltaY > 0 ? 1 : -1)); }, { passive: false });
    });
    // ---- knobs
    el.querySelectorAll('.kn').forEach((k) => this._knob(k));
  }

  _tgGet(c) { return c === 'pwr' ? (this.st.power ? 0 : 1) : this.st.sw[c] ?? 0; }
  _tgSet(c, v) {
    if (c === 'pwr') this.setPower(v === 0);
    else { this.st.sw[c] = v; if (c === 'att' && v === 2) this.attRef = null; if (c === 'orient') this.mapT = performance.now(); this.dirty = true; }
    this._syncUi(); this.m.save();
  }

  _knob(k) {
    const c = k.dataset.c, dial = k.querySelector('.kn-dial'), svg = k.querySelector('svg');
    const detents = c === 'rng' ? RANGES.length : 0;
    const A0 = -135, A1 = 135;
    // engraved scale
    let s = '';
    if (detents) {
      for (let i = 0; i < detents; i++) {
        const an = (A0 + (A1 - A0) * i / (detents - 1)) * D2R, x = Math.sin(an), y = -Math.cos(an);
        s += `<line x1="${x * 33}" y1="${y * 33}" x2="${x * 39}" y2="${y * 39}" stroke="#f1f2ee" stroke-width="2.2"/>`;
        s += `<text x="${x * 49}" y="${y * 49 + 4}" fill="#f1f2ee" font-size="12" font-weight="700" text-anchor="middle" font-family="Arial Narrow, Arial">${RANGES[i]}</text>`;
      }
    } else {
      for (let i = 0; i <= 10; i++) {
        const an = (A0 + (A1 - A0) * i / 10) * D2R, x = Math.sin(an), y = -Math.cos(an), L = i % 5 === 0 ? 39 : 37;
        s += `<line x1="${x * 33}" y1="${y * 33}" x2="${x * L}" y2="${y * L}" stroke="#f1f2ee" stroke-width="${i % 5 === 0 ? 2.2 : 1.4}"/>`;
      }
      s += `<text x="-34" y="43" fill="#f1f2ee" font-size="11" font-weight="700" text-anchor="middle" font-family="Arial Narrow, Arial">MIN</text><text x="34" y="43" fill="#f1f2ee" font-size="11" font-weight="700" text-anchor="middle" font-family="Arial Narrow, Arial">MAX</text>`;
    }
    svg.innerHTML = s;
    const get = () => (c === 'rng' ? this.st.rng / (detents - 1) : this.st.brt);
    const put = (f) => {
      if (c === 'rng') {
        const i = clamp(Math.round(f * (detents - 1)), 0, detents - 1);
        if (i !== this.st.rng) { this.st.rng = i; this.mapT = performance.now(); this.m.clicks.play('detent'); this.dirty = true; }
      } else {
        const nf = clamp(f, 0, 1);
        if (Math.floor(nf * 20) !== Math.floor(this.st.brt * 20)) this.m.clicks.play('detent');
        this.st.brt = nf; this._applyBrt();
      }
      this._syncUi();
    };
    dial.addEventListener('pointerdown', (e) => {
      e.preventDefault(); e.stopPropagation(); this.m.front(this);
      try { dial.setPointerCapture(e.pointerId); } catch { /* */ }
      const r = dial.getBoundingClientRect(), cx = r.left + r.width / 2, cy = r.top + r.height / 2;
      let f = get(), lastAng = Math.atan2(e.clientX - cx, cy - e.clientY), lastY = e.clientY;
      const mv = (ev) => {
        // rotate by dragging around the centre, or up/down
        const ang = Math.atan2(ev.clientX - cx, cy - ev.clientY);
        let da = ang - lastAng; if (da > Math.PI) da -= 2 * Math.PI; if (da < -Math.PI) da += 2 * Math.PI;
        const dist = Math.hypot(ev.clientX - cx, ev.clientY - cy);
        const df = dist > r.width * 0.3 ? da / ((A1 - A0) * D2R) : (lastY - ev.clientY) / 160;
        lastAng = ang; lastY = ev.clientY;
        f = clamp(f + df, 0, 1); put(f);
      };
      const upf = () => { dial.removeEventListener('pointermove', mv); dial.removeEventListener('pointerup', upf); this.m.save(); };
      dial.addEventListener('pointermove', mv); dial.addEventListener('pointerup', upf);
    });
    dial.addEventListener('wheel', (e) => {
      e.preventDefault();
      const step = detents ? 1 / (detents - 1) : 0.05;
      put(get() + (e.deltaY < 0 ? step : -step)); this.m.save();
    }, { passive: false });
    this['_knobSync_' + c] = () => {
      const f = get(), a = A0 + (A1 - A0) * f;
      k.querySelector('.kn-skirt').style.setProperty('--a', `${a}deg`); k.querySelector('.kn-cap').style.setProperty('--a', `${a}deg`);
    };
  }

  _syncUi() {
    this.el.querySelectorAll('.tg').forEach((t) => {
      const c = t.dataset.c, n = +t.dataset.n, v = this._tgGet(c);
      t.dataset.pos = v === 0 ? '0' : v === n - 1 ? 'end' : 'mid';
      t.querySelectorAll('.tg-pos span').forEach((s, i) => s.classList.toggle('sel', i === v));
    });
    this._knobSync_brt?.(); this._knobSync_rng?.();
    this.ttl.textContent = PAGES[this.st.page].title;
    this.led.classList.toggle('on', this.st.power && !this.booting());
    this.led.classList.toggle('boot', this.st.power && this.booting());
  }

  _applyBrt() { this.cv.style.filter = `brightness(${(0.16 + 0.98 * this.st.brt).toFixed(3)})`; }

  _resizeCanvas() {
    const dpr = Math.min(devicePixelRatio || 1, 2.5);
    const W = Math.max(64, Math.round(this.scr.clientWidth * dpr));
    if (this.cv.width !== W) { this.cv.width = W; this.cv.height = W; this.dirty = true; }
    this.scale = W / 512;
  }

  place() {
    const s = this.st;
    s.w = clamp(s.w, 260, Math.max(260, Math.min(1100, innerWidth - 16)));
    this.el.style.width = `${s.w}px`;
    this.el.style.fontSize = `${s.w / 30}px`;
    const h = this.el.offsetHeight || s.w * 1.3;
    s.x = clamp(s.x, -s.w + 90, innerWidth - 90);
    s.y = clamp(s.y, 0, Math.max(0, innerHeight - 40));
    if (s.y + Math.min(h, 60) > innerHeight) s.y = innerHeight - Math.min(h, 60);
    this.el.style.left = `${Math.round(s.x)}px`; this.el.style.top = `${Math.round(s.y)}px`;
  }

  booting() { return this.bootT0 !== null && (performance.now() - this.bootT0) / 1000 < BOOT_S; }
  _startBoot() { this.bootT0 = performance.now(); this.bitePass = false; this.powerT = performance.now(); this.scr.classList.remove('off'); this._syncUi(); }

  setPower(on) {
    this.st.power = on;
    if (on) this._startBoot();
    else { this.bootT0 = null; this.scr.classList.add('off'); }
    this._syncUi();
  }
  setOpen(open) {
    this.st.open = open;
    this.el.classList.toggle('closed', !open);
    if (open) { this.el.classList.remove('opening'); void this.el.offsetWidth; this.el.classList.add('opening'); this.m.front(this); this.place(); this.dirty = true; }
    this.m.syncDock(); this.m.save();
  }
  setCollapsed(c) { this.st.collapsed = c; this.el.classList.toggle('collapsed', c); this.dirty = true; this.m.save(); }

  edgeKey(i) {
    this.keyFlash[i] = performance.now(); this.dirty = true;
    if (!this.st.power || this.booting()) return;
    const m = MENUS[this.st.menu], k = m && m.keys[i];
    if (!k) return;
    if (k.up) { this.st.menu = m.up || 'MAIN'; }
    else if (k.menu && !k.page) this.st.menu = k.menu;
    if (k.page) { if (k.page !== this.st.page) this.hs = null; this.st.page = k.page; if (k.menu) this.st.menu = k.menu; this.lastDraw = 0; }
    if (k.act === 'rngUp') this.st.rng = clamp(this.st.rng + 1, 0, RANGES.length - 1);
    if (k.act === 'rngDn') this.st.rng = clamp(this.st.rng - 1, 0, RANGES.length - 1);
    if (k.act === 'orient') this.st.sw.orient = this.st.sw.orient ? 0 : 1;
    if (/rng|orient/.test(k.act || '')) this.mapT = performance.now();      // IDP rebuilds the map
    if (k.act === 'plan') this.st.plan = !this.st.plan;
    if (k.act === 'trail') this.st.trail = !this.st.trail;
    if (k.act === 'msgrst') this.m.faultAck = true;
    if (k.act === 'bit') this._startBoot();
    this._syncUi(); this.m.save();
  }

  /** first-order low-pass of a displayed air-data value (time constant tau s; resets on invalid data or a jump) */
  smooth(key, v, tau) {
    const S = (this._sm ||= {}), e = S[key], t = performance.now();
    if (!fin(v)) { delete S[key]; return v; }
    if (!e || t - e.t > 1500 || Math.abs(v - e.v) > Math.max(50, Math.abs(v) * 0.2)) { S[key] = { v, t }; return v; }
    e.v += (v - e.v) * Math.min(1, (t - e.t) / 1000 / tau); e.t = t;
    return e.v;
  }

  /** sample-and-hold of a displayed value (digital readouts update at `hz`, not every redraw) */
  hold(key, v, hz = 4) {
    const h = this._held[key];
    const changedValidity = h && typeof v === 'number' && fin(h.v) !== fin(v);
    if (!h || changedValidity || this.nowT - h.t >= 1000 / hz - 1) this._held[key] = { v, t: this.nowT };
    return this._held[key].v;
  }
  frame(d, now) {
    if (!this.st.open || this.st.collapsed || !this.st.power) return;
    this.nowT = now;
    const g = this.g, boot = this.booting();
    const page = PAGES[this.st.page];
    const flashing = this.keyFlash.some((t) => t && now - t < 160);
    if (!boot && !this.dirty && !flashing && now - this.lastDraw < 1000 / page.hz - 2) return;
    if (!boot && this.bootT0 !== null && !this.bitePass) { this.bitePass = true; this._syncUi(); }
    this._fpsN++; if (now - this._fpsT > 1000) { this.fps = this._fpsN * 1000 / (now - this._fpsT); this._fpsT = now; this._fpsN = 0; }
    this.lastDraw = now; this.dirty = false;
    if (!this.scale) this._resizeCanvas();
    g.setTransform(this.scale, 0, 0, this.scale, 0, 0);
    g.lineCap = 'butt'; g.lineJoin = 'miter'; g.setLineDash([]); g.globalAlpha = 1;
    if (boot) { drawBoot(g, this, (now - this.bootT0) / 1000); return; }
    drawBacklight(g);
    if (d.link === 'offline' && !d.st) { drawPollFail(g); drawKeyLegends(g, this); return; }
    page.draw(g, this, d);
    // fade-in after the self test
    const since = (now - this.bootT0) / 1000 - BOOT_S;
    if (since < 0.35) { g.fillStyle = `rgba(0,0,0,${1 - since / 0.35})`; g.fillRect(0, 0, 512, 512); this.dirty = true; }
  }
}

// =========================================================================================== panel MDU (3D flight deck)
/**
 * Headless MDU mounted in the 3D flight deck: the same pages, menus and edge keys as a window, drawn
 * into an offscreen canvas that the cockpit uses as the display texture.  `version` increments on
 * every redraw so the texture is uploaded only when the picture changed.
 */
export class PanelMdu extends MduWindow {
  _build() {
    this.cv = document.createElement('canvas');
    this.cv.width = this.cv.height = this.spec.res || 768;
    this.g = this.cv.getContext('2d');
    this.scale = this.cv.width / 512;
    this.version = 0;
  }
  _startBoot() { this.bootT0 = null; this.bitePass = true; this.powerT = performance.now() - 3600e3 * Math.random(); }
  setPower(on) { this.st.power = on; if (on) { this.bootT0 = performance.now(); this.bitePass = false; } this.dirty = true; this.version++; }
  place() {}
  _applyBrt() {}
  _syncUi() {}
  _resizeCanvas() { this.scale = this.cv.width / 512; }
  setOpen() {}
  setCollapsed() {}
  frame(d, now) {
    const before = this.lastDraw;
    super.frame(d, now);
    if (this.lastDraw !== before || this.booting()) this.version++;
  }
}

// =========================================================================================== manager
function noiseUrl(size = 160) {
  const c = document.createElement('canvas'); c.width = c.height = size;
  const g = c.getContext('2d'), id = g.createImageData(size, size);
  for (let i = 0; i < size * size; i++) {
    const v = 128 + (Math.random() - 0.5) * 120 + (Math.random() < 0.004 ? 90 : 0);
    id.data[i * 4] = id.data[i * 4 + 1] = id.data[i * 4 + 2] = clamp(v, 0, 255); id.data[i * 4 + 3] = 255;
  }
  g.putImageData(id, 0, 0);
  return `url(${c.toDataURL()})`;
}

export class Meds {
  constructor({ dock } = {}) {
    document.documentElement.style.setProperty('--meds-noise', noiseUrl());
    this.clicks = new Clicks();
    this.z = 10;
    this.fault = ''; this.faultAck = true; this.faultT = 0;
    let saved = {};
    try { saved = JSON.parse(localStorage.getItem(LS_KEY) || '{}'); } catch { saved = {}; }
    const phone = matchMedia('(max-width: 760px)').matches;
    let defs;
    if (phone) {
      // narrow screens: closed until opened from the dock (camera sheet), centred, full width
      const W = Math.min(440, innerWidth - 16), y = Math.max(56, (innerHeight - W * 1.36) / 2);
      defs = { pfd: { x: (innerWidth - W) / 2, y, w: W, open: false }, nav: { x: (innerWidth - W) / 2, y, w: W, open: false } };
    } else {
      const W = clamp(Math.round(innerWidth * 0.235), 330, 440), H = W * 1.36;
      const y = Math.max(60, innerHeight - H - 64);
      defs = { pfd: { x: Math.max(320, innerWidth / 2 - W - 8), y, w: W }, nav: { x: Math.max(330 + W, innerWidth / 2 + 8), y, w: W } };
    }
    this.wins = WIN_SPECS.map((s) => new MduWindow(this, s, saved[s.id], defs[s.id]));
    // dock buttons
    if (dock) {
      const box = document.createElement('div'); box.className = 'meds-dock';
      box.innerHTML = this.wins.map((w) => `<button data-w="${w.spec.id}" title="${w.spec.port} MDU 窗口（${w.spec.hotkey}）"><kbd>${w.spec.hotkey}</kbd>${w.spec.label}</button>`).join('');
      dock.insertBefore(box, dock.querySelector('.hints'));
      box.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => this.toggle(b.dataset.w)));
      this.dock = box;
    }
    this.wins.forEach((w) => { if (w.st.open) w.setOpen(true); w.setCollapsed(!!w.st.collapsed); });
    this.syncDock();
    addEventListener('keydown', (e) => {
      if (e.target.closest && e.target.closest('input,select,textarea') || e.ctrlKey || e.metaKey || e.altKey) return;
      const w = this.wins.find((x) => `Key${x.spec.hotkey}` === e.code);
      if (w) this.toggle(w.spec.id);
    });
    addEventListener('resize', () => this.wins.forEach((w) => w.place()));
  }
  toggle(id) { const w = this.wins.find((x) => x.spec.id === id); if (w) w.setOpen(!w.st.open); }
  /** create an MDU for the 3D flight deck (spec: { port, idp, page, menu, res, sw? }) */
  panelUnit(spec) {
    const u = new PanelMdu(this, spec, null, { page: spec.page, menu: spec.menu, rng: spec.rng ?? 3, brt: spec.brt ?? 0.85 });
    if (spec.sw) Object.assign(u.st.sw, spec.sw);
    (this.panel ||= []).push(u);
    return u;
  }
  front(w) {
    if (w.el.style.zIndex == this.z) return;
    this.z++;
    if (this.z > 40) { this.z = 10; this.wins.forEach((x) => { x.el.style.zIndex = this.z; }); this.z++; }
    w.el.style.zIndex = this.z;
  }
  syncDock() { if (this.dock) this.wins.forEach((w) => this.dock.querySelector(`[data-w=${w.spec.id}]`).classList.toggle('on', w.st.open)); }
  save() {
    clearTimeout(this._saveT);
    this._saveT = setTimeout(() => {
      const o = {};
      this.wins.forEach((w) => { const { x, y, w: ww, open, collapsed, power, brt, page, menu, rng, plan, trail, sw } = w.st; o[w.spec.id] = { x, y, w: ww, open, collapsed, power, brt, page, menu, rng, plan, trail, sw }; });
      try { localStorage.setItem(LS_KEY, JSON.stringify(o)); } catch { /* private mode */ }
    }, 250);
  }
  /** GPC fault message line on the DPS pages (warnings / errors from the simulator) */
  event(m) {
    if (!m || (m.level !== 'warn' && m.level !== 'error')) return;
    const now = new Date();
    const t = `${pad(now.getUTCHours(), 2)}:${pad(now.getUTCMinutes(), 2)}:${pad(now.getUTCSeconds(), 2)}`;
    const txt = String(m.text || '').toUpperCase().replace(/[^A-Z0-9 .:+\-/%]/g, '').replace(/\s+/g, ' ').trim().slice(0, 36);
    this.fault = `${m.level === 'error' ? 'GNC FAIL ' : 'GNC '}${txt}`.padEnd(42).slice(0, 42) + ' ' + t;
    this.faultAck = false; this.faultT = performance.now();
    // latched master alarm: one-off malfunctions (errors, crash, tail scrape, hard / off-runway
    // landing).  Energy and degraded-mode alarms are conditions, evaluated from the telemetry.
    if (Meds.isLatchedAlarm(m)) { this.latched = true; this.latchedT = performance.now(); this._syncAlarm(); }
    this.wins.forEach((w) => { w.dirty = true; });
  }
  static isLatchedAlarm(m) {
    if (m.level === 'error') return true;
    return /tail scrape|OFF RUNWAY|non-finite|main gear touchdown/i.test(String(m.text || ''));
  }
  /**
   * Condition alarms from the GNC telemetry: the C&W light follows the condition; the master alarm
   * comes on when a condition appears, goes out when it clears (energy recovered, plan usable again)
   * or when acknowledged with the MASTER ALARM pushbutton.
   */
  _conditions(st) {
    const g = st && st.gnc, c = {};
    if (g && !/CRASH|STOPPED|ROLLOUT|TOUCHDOWN/.test(g.phase || '')) {
      if (g.reachable === false) c.energy = true;
      if (g.degraded) c.degraded = true;
    }
    return c;
  }
  _updateAlarms(st) {
    const cond = this._conditions(st);
    const act = (this.condAct ||= {}), ack = (this.condAck ||= {});
    for (const k of new Set([...Object.keys(act), ...Object.keys(cond)])) {
      if (cond[k] && !act[k]) ack[k] = false;            // new condition: alarm
      if (!cond[k]) delete ack[k];                       // cleared: forget the acknowledgement
    }
    this.condAct = cond;
    this._syncAlarm();
  }
  _syncAlarm() {
    const cond = this.condAct || {}, ack = this.condAck || {};
    this.alarm = !!this.latched || Object.keys(cond).some((k) => !ack[k]);
    // GPC caution light: on while a condition persists, or for a minute after a latched malfunction
    this.cwGpc = Object.keys(cond).length > 0 || (!!this.latchedT && performance.now() - this.latchedT < 60000);
  }
  /** MASTER ALARM pushbutton: extinguishes the light (the fault message stays until MSG RESET) */
  ackAlarm() {
    this.latched = false;
    for (const k of Object.keys(this.condAct || {})) (this.condAck ||= {})[k] = true;
    this._syncAlarm();
  }
  /** new flight: clear everything */
  resetAlarms() { this.latched = false; this.latchedT = 0; this.condAct = {}; this.condAck = {}; this._syncAlarm(); }
  faultNew() { return performance.now() - this.faultT < 6000 && Math.floor((performance.now() - this.faultT) / 500) % 2 === 0; }
  /** per animation frame */
  update(d) {
    const now = performance.now();
    this._updateAlarms(d.st);
    const lit = (d.night || 0) > 0.45;
    for (const w of this.wins) {
      if (w._lit !== lit) { w._lit = lit; w.el.classList.toggle('lit', lit); }
      if (w._booting !== w.booting()) { w._booting = w.booting(); w._syncUi(); }
      w.frame(d, now);
    }
    // flight-deck units only draw while the cockpit view is shown: at most two redraws per animation
    // frame, the most overdue first; secondary units refresh at a reduced rate (spec.rate)
    if (d.cockpit && this.panel) {
      const due = [];
      for (const u of this.panel) {
        if (!u.st.power) continue;
        const late = now - u.lastDraw - 1000 / (PAGES[u.st.page].hz * (u.spec.rate || 1));
        if (late >= 0 || u.dirty || u.booting() || u.keyFlash.some((t) => t && now - t < 160)) due.push([late + (u.spec.always ? 50 : 0), u]);
      }
      due.sort((a, b) => b[0] - a[0]);
      for (let i = 0; i < Math.min(2, due.length); i++) { due[i][1].dirty = true; due[i][1].frame(d, now); }
    }
  }
}
