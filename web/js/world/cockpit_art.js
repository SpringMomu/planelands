// Procedural panel artwork for the orbiter flight deck: painted panel faces (light-grey enamel with
// white engraved lettering, group demarcation lines, placards, Dzus fasteners), the legend atlas for
// lighted pushbutton indicators and the caution & warning matrix, and the dynamic indicator canvas
// (landing-gear talkbacks, event / mission timers).
//
// Panel-local units are metres: origin at the top-left corner of the plate, u to the right, v down.
// White lettering is detected in the shader (min(r,g,b) above a threshold) and lit at night as the
// panels' integral (electroluminescent) lighting.

export const PX_PER_M = 1500;
const FONT = '"Arial Narrow", "Roboto Condensed", "Helvetica Neue", Arial, sans-serif';
const INK = '#f3f3ee';
const PAINT = [131, 136, 140];         // orbiter panel grey (sRGB)

function rng(seed) {
  let s = seed >>> 0 || 1;
  return () => { s ^= s << 13; s >>>= 0; s ^= s >> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; };
}
function hashStr(t) { let h = 2166136261; for (let i = 0; i < t.length; i++) { h ^= t.charCodeAt(i); h = Math.imul(h, 16777619); } return h >>> 0; }

/** fit text into a width by condensing horizontally */
function textFit(g, s, x, y, maxW, align = 'center') {
  const w = g.measureText(s).width;
  if (w <= maxW || maxW <= 0) { g.textAlign = align; g.fillText(s, x, y); return; }
  g.save();
  const k = maxW / w;
  g.translate(x, y); g.scale(k, 1);
  g.textAlign = align; g.fillText(s, 0, 0);
  g.restore();
}

// ------------------------------------------------------------------ paint base
function paintBase(g, W, H, seed, col = PAINT) {
  g.fillStyle = `rgb(${col[0]},${col[1]},${col[2]})`;
  g.fillRect(0, 0, W, H);
  // fine enamel grain + a few scuffs and handling marks
  const id = g.getImageData(0, 0, W, H), d = id.data, r = rng(seed);
  for (let i = 0; i < d.length; i += 4) {
    const n = (r() - 0.5) * 7;
    d[i] += n; d[i + 1] += n; d[i + 2] += n;
  }
  g.putImageData(id, 0, 0);
  g.save();
  for (let i = 0; i < 26; i++) {
    const x = r() * W, y = r() * H, L = 6 + r() * 40, a = r() * Math.PI;
    g.strokeStyle = r() < 0.5 ? 'rgba(60,62,66,0.10)' : 'rgba(210,212,214,0.10)';
    g.lineWidth = 0.6 + r() * 1.2;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x + Math.cos(a) * L, y + Math.sin(a) * L); g.stroke();
  }
  // darker wear around the edges (hands, velcro)
  const eg = g.createLinearGradient(0, 0, 0, H);
  eg.addColorStop(0, 'rgba(40,42,46,0.10)'); eg.addColorStop(0.06, 'rgba(40,42,46,0)'); eg.addColorStop(0.94, 'rgba(40,42,46,0)'); eg.addColorStop(1, 'rgba(40,42,46,0.14)');
  g.fillStyle = eg; g.fillRect(0, 0, W, H);
  g.restore();
}

function dzus(g, x, y, r) {
  const gr = g.createRadialGradient(x - r * 0.3, y - r * 0.3, r * 0.1, x, y, r);
  gr.addColorStop(0, '#b9bcbf'); gr.addColorStop(1, '#5d6064');
  g.fillStyle = gr; g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill();
  g.strokeStyle = 'rgba(20,20,22,0.8)'; g.lineWidth = Math.max(1, r * 0.25);
  g.beginPath(); g.moveTo(x - r * 0.7, y - r * 0.25); g.lineTo(x + r * 0.7, y + r * 0.25); g.stroke();
}

/**
 * Paint one panel face.
 * spec: { id, w, h, items[], seed?, color? }   (see cockpit.js for the item types)
 * returns { canvas, ppm }
 */
export function paintPanel(spec, ppm = PX_PER_M) {
  const W = Math.max(16, Math.round(spec.w * ppm)), H = Math.max(16, Math.round(spec.h * ppm));
  const c = document.createElement('canvas'); c.width = W; c.height = H;
  const g = c.getContext('2d');
  const S = (m) => m * ppm;
  paintBase(g, W, H, hashStr(spec.id), spec.color || PAINT);
  g.textBaseline = 'alphabetic';
  const font = (mm, w = '700') => { g.font = `${w} ${(mm * ppm / 1000).toFixed(1)}px ${FONT}`; };
  const ink = (a = 1) => { g.fillStyle = a === 1 ? INK : `rgba(243,243,238,${a})`; g.strokeStyle = g.fillStyle; };
  const line = (x0, y0, x1, y1, w = 0.0011) => { g.lineWidth = S(w); g.beginPath(); g.moveTo(S(x0), S(y0)); g.lineTo(S(x1), S(y1)); g.stroke(); };

  for (const it of spec.items) {
    ink();
    switch (it.t) {
      case 'box': {                    // demarcation lines with the title in a gap of the top line
        const { x, y, w, h } = it;
        g.lineWidth = S(0.0012);
        font(it.ts || 5.2);
        const tw = it.title ? g.measureText(it.title).width / ppm + 0.008 : 0;
        const tx = x + w / 2;
        if (it.title) {
          line(x, y, tx - tw / 2, y); line(tx + tw / 2, y, x + w, y);
          textFit(g, it.title, S(tx), S(y + 0.0019), S(w - 0.004));
        } else line(x, y, x + w, y);
        if (!it.open) { line(x, y, x, y + h); line(x + w, y, x + w, y + h); line(x, y + h, x + w, y + h); }
        else { line(x, y, x, y + 0.006); line(x + w, y, x + w, y + 0.006); }
        break;
      }
      case 'txt': {
        font(it.size || 4.6, it.weight || '700');
        if (it.color) g.fillStyle = it.color;
        textFit(g, it.s, S(it.x), S(it.y), it.maxw ? S(it.maxw) : 0, it.align || 'center');
        break;
      }
      case 'id': {                     // panel designation, top-left corner, in a small box
        font(6.2);
        g.textAlign = 'left';
        g.fillText(it.s, S(it.x ?? 0.012), S(it.y ?? 0.016));
        break;
      }
      case 'tg': {                     // toggle switch: escutcheon ring, label above, positions
        const { x, y } = it;
        g.fillStyle = 'rgba(30,32,35,0.55)';
        g.beginPath(); g.arc(S(x), S(y), S(0.0068), 0, Math.PI * 2); g.fill();
        ink();
        if (it.lab) {
          font(it.ls || 4.0);
          const lines = it.lab.split('\n');
          lines.forEach((s, i) => textFit(g, s, S(x), S(y - 0.0105 - (lines.length - 1 - i) * 0.0046), S(it.lw || 0.034)));
        }
        const pos = it.pos || [];
        font(3.4);
        if (pos.length === 2) {
          textFit(g, pos[0], S(x), S(y - 0.0078 + (it.lab ? 0 : 0)), S(0.03));
          textFit(g, pos[1], S(x), S(y + 0.0118), S(0.03));
        } else if (pos.length === 3) {
          textFit(g, pos[0], S(x), S(y - 0.0078), S(0.03));
          g.textAlign = 'left'; g.fillText(pos[1], S(x + 0.0082), S(y + 0.0012));
          textFit(g, pos[2], S(x), S(y + 0.0118), S(0.03));
        }
        if (it.guard) {                // guard outline painted on the panel
          g.strokeStyle = 'rgba(25,25,28,0.5)'; g.lineWidth = S(0.0008);
          g.strokeRect(S(x - 0.0095), S(y - 0.011), S(0.019), S(0.022));
        }
        break;
      }
      case 'kn': {                     // rotary switch / pot: dial scale and position legends
        const { x, y } = it, r = it.r || 0.010;
        const pos = it.pos || [];
        g.lineWidth = S(0.0008);
        const n = pos.length || 11, a0 = it.a0 ?? -60, a1 = it.a1 ?? 60;
        for (let i = 0; i < n; i++) {
          const a = (a0 + (a1 - a0) * (n === 1 ? 0.5 : i / (n - 1))) * Math.PI / 180;
          const sx = Math.sin(a), sy = -Math.cos(a);
          line(x + sx * r * 1.15, y + sy * r * 1.15, x + sx * r * 1.45, y + sy * r * 1.45, 0.0008);
          if (pos.length) { font(3.2); textFit(g, pos[i], S(x + sx * r * 2.25), S(y + sy * r * 2.05 + 0.0013), S(0.03)); }
        }
        if (it.lab) { font(4.0); textFit(g, it.lab, S(x), S(y + r + (pos.length ? 0.0095 : 0.0075)), S(it.lw || 0.05)); }
        break;
      }
      case 'cb': {                     // circuit breaker: label under, amp rating
        const { x, y } = it;
        g.fillStyle = 'rgba(20,21,24,0.55)';
        g.beginPath(); g.arc(S(x), S(y), S(0.0052), 0, Math.PI * 2); g.fill();
        ink();
        font(2.9);
        if (it.lab) textFit(g, it.lab, S(x), S(y + 0.0098), S(0.016));
        if (it.amp) { font(2.6, '400'); textFit(g, String(it.amp), S(x), S(y - 0.0064), S(0.012)); }
        break;
      }
      case 'pb': {                     // lighted pushbutton: black surround painted, label above
        const { x, y, w = 0.019, h = 0.019 } = it;
        g.fillStyle = '#17181a'; g.fillRect(S(x - w / 2 - 0.0015), S(y - h / 2 - 0.0015), S(w + 0.003), S(h + 0.003));
        ink();
        if (it.lab) { font(3.6); textFit(g, it.lab, S(x), S(y - h / 2 - 0.0035), S(w + 0.016)); }
        break;
      }
      case 'mdu': {                    // cut-out shadow behind the display unit
        const { x, y, w, h } = it;
        g.fillStyle = 'rgba(12,13,15,0.85)'; g.fillRect(S(x - 0.002), S(y - 0.002), S(w + 0.004), S(h + 0.004));
        break;
      }
      case 'win': {                    // black window (talkback, digital readout, meter)
        const { x, y, w, h } = it;
        g.fillStyle = '#060708'; g.fillRect(S(x), S(y), S(w), S(h));
        g.strokeStyle = 'rgba(15,15,17,0.9)'; g.lineWidth = S(0.0015); g.strokeRect(S(x - 0.0008), S(y - 0.0008), S(w + 0.0016), S(h + 0.0016));
        ink();
        if (it.lab) { font(it.ls || 3.6); textFit(g, it.lab, S(x + w / 2), S(it.labBelow ? y + h + 0.0055 : y - 0.0025), S(Math.max(w, 0.02) + 0.01)); }
        break;
      }
      case 'meter': {                  // edgewise meter: black window with a white scale
        const { x, y, w, h } = it;
        g.fillStyle = '#0a0b0c'; g.fillRect(S(x), S(y), S(w), S(h));
        ink();
        const n = 10;
        for (let i = 0; i <= n; i++) {
          const yy = y + h * 0.08 + (h * 0.84) * i / n;
          line(x + w * 0.15, yy, x + w * (i % 5 === 0 ? 0.62 : 0.45), yy, 0.0006);
          if (i % 5 === 0) { font(2.4, '400'); g.textAlign = 'left'; g.fillText(String((n - i) * (it.k || 10)), S(x + w * 0.66), S(yy + 0.0009)); }
        }
        if (it.lab) { font(3.2); textFit(g, it.lab, S(x + w / 2), S(y + h + 0.0052), S(w + 0.012)); }
        break;
      }
      case 'stripe': {                 // black/yellow hazard border (abort, fire)
        const { x, y, w, h } = it;
        g.save(); g.beginPath(); g.rect(S(x), S(y), S(w), S(h)); g.clip();
        g.fillStyle = '#e8c21c'; g.fillRect(S(x), S(y), S(w), S(h));
        g.fillStyle = '#111';
        for (let k = -h; k < w + h; k += 0.008) { g.beginPath(); g.moveTo(S(x + k), S(y)); g.lineTo(S(x + k + 0.004), S(y)); g.lineTo(S(x + k + 0.004 - h), S(y + h)); g.lineTo(S(x + k - h), S(y + h)); g.fill(); }
        g.restore();
        g.fillStyle = `rgb(${PAINT[0]},${PAINT[1]},${PAINT[2]})`;
        g.fillRect(S(x + 0.003), S(y + 0.003), S(w - 0.006), S(h - 0.006));
        break;
      }
      case 'placard': {                // black placard with white text (cue / caution labels)
        const { x, y, w, h } = it;
        g.fillStyle = it.bg || '#151618'; g.fillRect(S(x), S(y), S(w), S(h));
        g.fillStyle = it.fg || INK; font(it.size || 3.6);
        const lines = it.s.split('\n');
        lines.forEach((s, i) => textFit(g, s, S(x + w / 2), S(y + h / 2 + (i - (lines.length - 1) / 2) * (it.size || 3.6) * 1.25 / 1000 + 0.0012), S(w - 0.003)));
        break;
      }
      case 'velcro': {                 // velcro patch (dark, fuzzy)
        const { x, y, w, h } = it;
        g.fillStyle = '#3b3c3a'; g.fillRect(S(x), S(y), S(w), S(h));
        const r = rng(hashStr(spec.id + x));
        for (let k = 0; k < w * h * 2e6; k++) { g.fillStyle = r() < 0.5 ? 'rgba(20,20,20,0.5)' : 'rgba(90,90,86,0.4)'; g.fillRect(S(x + r() * w), S(y + r() * h), 1, 1); }
        break;
      }
      case 'cue': {                    // laminated cue card
        const { x, y, w, h } = it;
        g.fillStyle = '#ecebe3'; g.fillRect(S(x), S(y), S(w), S(h));
        g.fillStyle = '#222'; font(2.6, '700'); g.textAlign = 'left';
        g.fillText(it.title || '', S(x + 0.003), S(y + 0.006));
        font(2.1, '400');
        const r = rng(hashStr(it.title || 'cue'));
        for (let k = 0; k < Math.floor((h - 0.01) / 0.0034); k++) {
          const yy = y + 0.0105 + k * 0.0034;
          g.fillStyle = r() < 0.12 ? '#1d4f9c' : '#333';
          g.fillRect(S(x + 0.003), S(yy - 0.0016), S((w - 0.006) * (0.35 + 0.6 * r())), Math.max(1, S(0.0009)));
        }
        g.strokeStyle = 'rgba(0,0,0,0.25)'; g.lineWidth = 1; g.strokeRect(S(x) + 0.5, S(y) + 0.5, S(w) - 1, S(h) - 1);
        break;
      }
      default: break;
    }
    if (it.t !== 'id' && it.t !== 'box') g.textAlign = 'center';
  }
  // Dzus fasteners in the corners
  const fr = 0.0032, m = 0.0075;
  if (!spec.noDzus) for (const [x, y] of [[m, m], [spec.w - m, m], [m, spec.h - m], [spec.w - m, spec.h - m]]) dzus(g, S(x), S(y), S(fr));
  // panel edge: slight bevel highlight / shadow
  g.strokeStyle = 'rgba(255,255,255,0.18)'; g.lineWidth = 2; g.beginPath(); g.moveTo(1, H - 1); g.lineTo(1, 1); g.lineTo(W - 1, 1); g.stroke();
  g.strokeStyle = 'rgba(0,0,0,0.30)'; g.beginPath(); g.moveTo(W - 1, 1); g.lineTo(W - 1, H - 1); g.lineTo(1, H - 1); g.stroke();
  return { canvas: c, ppm };
}

// ------------------------------------------------------------------ legend atlas (lighted pushbuttons, C&W)
export const LEGEND_COLORS = { white: '#ffffff', green: '#46ff6a', amber: '#ffb21e', red: '#ff3326', blue: '#59b8ff' };

/**
 * Lays out legends into one atlas.  legends: [{ key, lines: [..], color, split?: [top, bottom] }]
 * Returns { canvas, rect(key) -> [u0, v0, u1, v1] } (uv with v up, as three expects with flipY).
 */
export function legendAtlas(legends) {
  const CW = 128, CH = 128, cols = 16;
  const rows = Math.ceil(legends.length / cols);
  const c = document.createElement('canvas'); c.width = CW * cols; c.height = CH * Math.max(1, rows);
  const g = c.getContext('2d');
  g.fillStyle = '#000'; g.fillRect(0, 0, c.width, c.height);
  const rects = new Map();
  legends.forEach((L, i) => {
    const x0 = (i % cols) * CW, y0 = Math.floor(i / cols) * CH;
    const ar = L.aspect || 1;                         // width / height of the lens
    const w = ar >= 1 ? CW : CW * ar, h = ar >= 1 ? CH / ar : CH;
    const ox = x0 + (CW - w) / 2, oy = y0 + (CH - h) / 2;
    g.save(); g.beginPath(); g.rect(ox, oy, w, h); g.clip();
    g.fillStyle = '#000'; g.fillRect(ox, oy, w, h);
    const col = LEGEND_COLORS[L.color] || L.color || '#fff';
    const drawLines = (lines, cy, hh, color) => {
      const n = lines.length, fs = Math.min(hh / (n * 1.12), (w * 0.9) / Math.max(...lines.map((s) => s.length)) * 1.9, L.fsMax ? h * L.fsMax : 1e9);
      g.font = `700 ${fs.toFixed(1)}px ${FONT}`; g.fillStyle = color; g.textBaseline = 'middle';
      lines.forEach((s, k) => textFit(g, s, ox + w / 2, cy + (k - (n - 1) / 2) * fs * 1.08, w * 0.88));
    };
    if (L.split) {                     // two-legend split lens (e.g. AUTO over CSS)
      drawLines([L.split[0]], oy + h * 0.27, h * 0.42, LEGEND_COLORS[L.splitCol?.[0]] || col);
      drawLines([L.split[1]], oy + h * 0.73, h * 0.42, LEGEND_COLORS[L.splitCol?.[1]] || col);
      g.fillStyle = '#222'; g.fillRect(ox, oy + h * 0.49, w, h * 0.02);
    } else drawLines(L.lines, oy + h / 2, h * 0.82, col);
    g.restore();
    rects.set(L.key, [ox / c.width, 1 - (oy + h) / c.height, (ox + w) / c.width, 1 - oy / c.height]);
  });
  return { canvas: c, rect: (k) => rects.get(k) || [0, 0, 0.001, 0.001] };
}

// ------------------------------------------------------------------ dynamic indicators
/** small canvas for talkbacks and the digital timers; regions are fixed slots of 256 x 96 px */
export class DynCanvas {
  constructor(slots) {
    this.slots = slots;                         // [{ key, kind: 'tb' | 'seg', aspect }]
    this.SW = 256; this.SH = 96;
    this.c = document.createElement('canvas'); this.c.width = this.SW; this.c.height = this.SH * slots.length;
    this.g = this.c.getContext('2d');
    this.state = {};
  }
  rect(key) {
    const i = this.slots.findIndex((s) => s.key === key);
    const H = this.c.height;
    return [0, 1 - (i + 1) * this.SH / H, 1, 1 - i * this.SH / H];
  }
  /** returns true if anything changed */
  draw(values) {
    let changed = false;
    this.slots.forEach((s, i) => {
      const v = values[s.key];
      if (this.state[s.key] === v) return;
      this.state[s.key] = v; changed = true;
      const g = this.g, y0 = i * this.SH, W = this.SW, H = this.SH;
      g.save(); g.beginPath(); g.rect(0, y0, W, H); g.clip();
      if (s.kind === 'tb') {
        // talkback flag: grey "UP" / barber pole (in transit) / "DN"
        if (v === 'bp') {
          g.fillStyle = '#e9e9e4'; g.fillRect(0, y0, W, H);
          g.fillStyle = '#111';
          for (let k = -H; k < W + H; k += 44) { g.beginPath(); g.moveTo(k, y0); g.lineTo(k + 22, y0); g.lineTo(k + 22 - H, y0 + H); g.lineTo(k - H, y0 + H); g.fill(); }
        } else {
          g.fillStyle = '#7c8084'; g.fillRect(0, y0, W, H);
          g.fillStyle = '#f4f4ee'; g.font = `700 64px ${FONT}`; g.textAlign = 'center'; g.textBaseline = 'middle';
          g.fillText(v === 'dn' ? 'DN' : 'UP', W / 2, y0 + H / 2 + 3);
        }
      } else {
        // seven-segment style LED readout (red-orange), unlit segments faintly visible
        g.fillStyle = '#080404'; g.fillRect(0, y0, W, H);
        g.font = `700 70px "DSEG7 Classic", "Consolas", monospace`; g.textAlign = 'center'; g.textBaseline = 'middle';
        g.fillStyle = 'rgba(80,16,10,0.55)'; g.fillText(String(v).replace(/[0-9]/g, '8'), W / 2, y0 + H / 2 + 4);
        g.fillStyle = '#ff5a2a'; g.fillText(String(v), W / 2, y0 + H / 2 + 4);
      }
      g.restore();
    });
    return changed;
  }
}

// ------------------------------------------------------------------ structure detail (tiling, 1 m)
/** greyscale detail for the bare structure: mottling, panel seams with fasteners, scuffs (128 = 1.0) */
export function structureCanvas(N = 1024) {
  const c = document.createElement('canvas'); c.width = c.height = N;
  const g = c.getContext('2d');
  const r = rng(91731);
  g.fillStyle = 'rgb(128,128,128)'; g.fillRect(0, 0, N, N);
  // low-frequency mottling
  for (let i = 0; i < 260; i++) {
    const x = r() * N, y = r() * N, rad = 20 + r() * 140, v = 128 + (r() - 0.5) * 18;
    const gr = g.createRadialGradient(x, y, 0, x, y, rad);
    gr.addColorStop(0, `rgba(${v},${v},${v},0.18)`); gr.addColorStop(1, `rgba(${v},${v},${v},0)`);
    g.fillStyle = gr;
    for (const ox of [-N, 0, N]) for (const oy of [-N, 0, N]) { g.save(); g.translate(ox, oy); g.beginPath(); g.arc(x, y, rad, 0, Math.PI * 2); g.fill(); g.restore(); }
  }
  // grain
  const id = g.getImageData(0, 0, N, N), d = id.data;
  for (let i = 0; i < d.length; i += 4) { const n = (r() - 0.5) * 6; d[i] += n; d[i + 1] += n; d[i + 2] += n; }
  g.putImageData(id, 0, 0);
  // panel seams (closeout panels ~ 0.25-0.5 m) with fastener rows
  const seams = [0, 0.34, 0.71].map((f) => Math.round(f * N));
  g.lineWidth = 3;
  for (const s of seams) {
    g.strokeStyle = 'rgba(40,40,40,0.75)'; g.beginPath(); g.moveTo(s + 0.5, 0); g.lineTo(s + 0.5, N); g.stroke();
    g.beginPath(); g.moveTo(0, s + 0.5); g.lineTo(N, s + 0.5); g.stroke();
    g.strokeStyle = 'rgba(200,200,200,0.35)'; g.lineWidth = 1.5;
    g.beginPath(); g.moveTo(s + 2.5, 0); g.lineTo(s + 2.5, N); g.stroke(); g.beginPath(); g.moveTo(0, s + 2.5); g.lineTo(N, s + 2.5); g.stroke();
    g.lineWidth = 3;
    for (let t = 20; t < N; t += 52) {
      for (const [x, y] of [[s + 12, t], [t, s + 12]]) {
        const gr = g.createRadialGradient(x - 1, y - 1, 0.5, x, y, 4.5);
        gr.addColorStop(0, 'rgb(205,205,205)'); gr.addColorStop(0.7, 'rgb(120,120,120)'); gr.addColorStop(1, 'rgb(60,60,60)');
        g.fillStyle = gr; g.beginPath(); g.arc(x, y, 4.5, 0, Math.PI * 2); g.fill();
      }
    }
  }
  // scuffs and smudges
  for (let i = 0; i < 90; i++) {
    const x = r() * N, y = r() * N, L = 10 + r() * 70, a = r() * Math.PI;
    g.strokeStyle = r() < 0.6 ? 'rgba(70,70,70,0.25)' : 'rgba(190,190,190,0.25)'; g.lineWidth = 1 + r() * 2;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x + Math.cos(a) * L, y + Math.sin(a) * L); g.stroke();
  }
  return c;
}
