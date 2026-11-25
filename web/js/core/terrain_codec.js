// Lossless coding of the float32 terrain heightmap for download (tools/build_web.py encodes it):
// the float bit patterns as uint32, a planar predictor (left + up - up-left, mod 2^32), zigzag, and the
// four bytes of every residual split into planes. The smooth terrain leaves mostly small residuals, so
// the planes gzip to ~56 % of the raw float grid (7.4 instead of 13.3 MB). Decoding is bit-exact.
//
//   "PLH1" | width u32 | height u32 | 0 u32 | plane0[N] | plane1[N] | plane2[N] | plane3[N]   (LE)
export const PLH1 = 'plh1';

/** encoded bytes -> the original little-endian float32 bytes */
export function decodeHeights(buf) {
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  if (dv.getUint32(0, true) !== 0x31484c50) throw new Error('terrain: not a PLH1 stream');
  const W = dv.getUint32(4, true), H = dv.getUint32(8, true), N = W * H;
  if (buf.byteLength !== 16 + 4 * N) throw new Error('terrain: truncated PLH1 stream');
  const b0 = buf.subarray(16, 16 + N), b1 = buf.subarray(16 + N, 16 + 2 * N);
  const b2 = buf.subarray(16 + 2 * N, 16 + 3 * N), b3 = buf.subarray(16 + 3 * N, 16 + 4 * N);
  const out = new Int32Array(N);                 // platform byte order is little-endian everywhere
  let i = 0;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++, i++) {
      const z = b0[i] | (b1[i] << 8) | (b2[i] << 16) | (b3[i] << 24);
      const s = (z >>> 1) ^ -(z & 1);
      const pred = x > 0 ? (y > 0 ? out[i - 1] + out[i - W] - out[i - W - 1] : out[i - 1]) : (y > 0 ? out[i - W] : 0);
      out[i] = (pred + s) | 0;
    }
  }
  return new Uint8Array(out.buffer);
}
