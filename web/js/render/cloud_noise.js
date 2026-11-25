// Tileable noise volumes for the volumetric clouds, generated in the browser (they used to be
// downloaded: 8.5 MB that hardly compress). JavaScript port of the former worldgen/clouds.py shape /
// detail volumes - same construction, frequencies and normalisation; the random tables come from a
// seeded PRNG, so the result is deterministic (identical for every visitor), just not bit-identical
// to the Python version. The weather map stays a downloaded file: the CPU side (storm forcing of the
// simulator's wind) reads it, so it must be the same everywhere.
//
//   shape  128^3 RGBA8 : R = Perlin-Worley (base billows), G/B/A = Worley fBm at 3 increasing frequencies
//   detail  32^3 RGBA8 : R/G/B = Worley fBm at 3 increasing frequencies (edge erosion), A = 255
// Layout [z][y][x][rgba], as the Data3DTexture expects.

// sfc32 seeded through splitmix32: fast, good quality, reproducible
function rng(seed) {
  let s = (seed * 0x9e3779b9) >>> 0;
  const split = () => {
    s = (s + 0x9e3779b9) >>> 0;
    let z = s;
    z = Math.imul(z ^ (z >>> 16), 0x85ebca6b);
    z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35);
    return (z ^ (z >>> 16)) >>> 0;
  };
  let a = split(), b = split(), c = split(), d = split();
  const next = () => {
    const t = (((a + b) | 0) + d) | 0;
    d = (d + 1) | 0;
    a = b ^ (b >>> 9);
    b = (c + (c << 3)) | 0;
    c = (c << 21) | (c >>> 11);
    c = (c + t) | 0;
    return (t >>> 0) / 4294967296;
  };
  for (let i = 0; i < 12; i++) next();
  return next;
}

const fade = (t) => t * t * t * (t * (t * 6 - 15) + 10);
const mod = (i, n) => ((i % n) + n) % n;

/** periodic 3-D Perlin noise on a res^3 grid, period = freq cells, ~[-1, 1] */
function perlin3(res, freq, seed) {
  const r = rng(seed);
  const n = freq * freq * freq;
  const g = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {               // isotropic unit gradients (normal deviates, normalised)
    let x, y, z, l;
    do {
      const u1 = Math.max(r(), 1e-12), u2 = r(), u3 = Math.max(r(), 1e-12), u4 = r();
      const m1 = Math.sqrt(-2 * Math.log(u1)), m2 = Math.sqrt(-2 * Math.log(u3));
      x = m1 * Math.cos(2 * Math.PI * u2); y = m1 * Math.sin(2 * Math.PI * u2); z = m2 * Math.cos(2 * Math.PI * u4);
      l = Math.hypot(x, y, z);
    } while (l < 1e-6);
    g[i * 3] = x / l; g[i * 3 + 1] = y / l; g[i * 3 + 2] = z / l;
  }
  const out = new Float32Array(res * res * res);
  // per-axis cell index and fraction (same for x, y, z)
  const ci = new Int32Array(res), cf = new Float32Array(res), cu = new Float32Array(res);
  for (let i = 0; i < res; i++) {
    const p = ((i + 0.5) / res) * freq;
    ci[i] = Math.floor(p); cf[i] = p - ci[i]; cu[i] = fade(cf[i]);
  }
  let o = 0;
  for (let z = 0; z < res; z++) {
    const iz = ci[z], fz = cf[z], w = cu[z];
    for (let y = 0; y < res; y++) {
      const iy = ci[y], fy = cf[y], v = cu[y];
      for (let x = 0; x < res; x++, o++) {
        const ix = ci[x], fx = cf[x], u = cu[x];
        let acc = 0;
        for (let dz = 0; dz < 2; dz++) {
          const wz = dz ? w : 1 - w, gz = mod(iz + dz, freq) * freq;
          for (let dy = 0; dy < 2; dy++) {
            const wy = dy ? v : 1 - v, gy = (gz + mod(iy + dy, freq)) * freq;
            for (let dx = 0; dx < 2; dx++) {
              const wx = dx ? u : 1 - u, k = (gy + mod(ix + dx, freq)) * 3;
              acc += wx * wy * wz * (g[k] * (fx - dx) + g[k + 1] * (fy - dy) + g[k + 2] * (fz - dz));
            }
          }
        }
        out[o] = acc * 1.6;
      }
    }
  }
  return out;
}

/** periodic inverted Worley (1 - F1) on a res^3 grid, freq cells per period, [0, 1] */
function worley3(res, freq, seed) {
  const r = rng(seed);
  const n = freq * freq * freq;
  const pts = new Float32Array(n * 3);
  for (let i = 0; i < n * 3; i++) pts[i] = r();
  const out = new Float32Array(res * res * res);
  const ci = new Int32Array(res), cp = new Float32Array(res);
  for (let i = 0; i < res; i++) { cp[i] = ((i + 0.5) / res) * freq; ci[i] = Math.floor(cp[i]); }
  const wrap = new Int32Array(freq + 2);           // cell index -1 .. freq -> wrapped
  for (let i = -1; i <= freq; i++) wrap[i + 1] = mod(i, freq);
  let o = 0;
  for (let z = 0; z < res; z++) {
    const iz = ci[z], pz = cp[z];
    for (let y = 0; y < res; y++) {
      const iy = ci[y], py = cp[y];
      for (let x = 0; x < res; x++, o++) {
        const ix = ci[x], px = cp[x];
        let best = 1e9;
        for (let oz = -1; oz <= 1; oz++) {
          const cz = iz + oz, kz = wrap[cz + 1] * freq;
          for (let oy = -1; oy <= 1; oy++) {
            const cy = iy + oy, ky = (kz + wrap[cy + 1]) * freq;
            for (let ox = -1; ox <= 1; ox++) {
              const cx = ix + ox, k = (ky + wrap[cx + 1]) * 3;
              const ddx = cx + pts[k] - px, ddy = cy + pts[k + 1] - py, ddz = cz + pts[k + 2] - pz;
              const d2 = ddx * ddx + ddy * ddy + ddz * ddz;
              if (d2 < best) best = d2;
            }
          }
        }
        const v = 1 - Math.sqrt(best);
        out[o] = v < 0 ? 0 : v > 1 ? 1 : v;
      }
    }
  }
  return out;
}

function normalise(a) {
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < a.length; i++) { const v = a[i]; if (v < lo) lo = v; if (v > hi) hi = v; }
  const k = 1 / (hi - lo + 1e-6);
  for (let i = 0; i < a.length; i++) a[i] = (a[i] - lo) * k;
  return a;
}

function pack(channels, n) {
  const out = new Uint8Array(n * 4);
  for (let c = 0; c < 4; c++) {
    const ch = channels[c];
    for (let i = 0; i < n; i++) {
      const v = Math.round(ch[i] * 255);
      out[i * 4 + c] = v < 0 ? 0 : v > 255 ? 255 : v;
    }
  }
  return out;
}

const wfbm = (ws, i, n) => {
  const out = new Float32Array(n);
  for (let k = 0; k < n; k++) out[k] = ws[i][k] * 0.625 + ws[i + 1][k] * 0.25 + ws[i + 2][k] * 0.125;
  return out;
};

export function shapeVolume(res = 128, seed = 11) {
  const n = res * res * res;
  const p4 = perlin3(res, 4, seed), p8 = perlin3(res, 8, seed + 1), p16 = perlin3(res, 16, seed + 2);
  const ws = [4, 8, 16, 32, 64].map((f, i) => worley3(res, f, seed + 10 + i));
  const wLow = wfbm(ws, 0, n);
  const pw = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    let p = (p4[i] * 0.5 + 0.5 * p8[i] * 0.5 + 0.25 * p16[i] * 0.5) / 1.4 * 0.5 + 0.5;
    p = p < 0 ? 0 : p > 1 ? 1 : p;
    const v = wLow[i] + p * (1 - wLow[i]);              // remap(perlin, 0, 1, w_low, 1)
    pw[i] = v < 0 ? 0 : v > 1 ? 1 : v;
  }
  normalise(pw);
  const g = wfbm(ws, 1, n), b = wfbm(ws, 2, n);
  const a = new Float32Array(n);
  for (let i = 0; i < n; i++) a[i] = ws[3][i] * 0.625 + ws[4][i] * 0.375;
  return pack([normalise(pw), normalise(g), normalise(b), normalise(a)], n);
}

export function detailVolume(res = 32, seed = 23) {
  const n = res * res * res;
  const ws = [2, 4, 8, 16, 32].map((f, i) => worley3(res, f, seed + i));
  const one = new Float32Array(n).fill(1);
  return pack([normalise(wfbm(ws, 0, n)), normalise(wfbm(ws, 1, n)), normalise(wfbm(ws, 2, n)), one], n);
}
