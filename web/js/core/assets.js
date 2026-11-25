// Loads the generated world data (web/data) and the shared JSON definitions.
import * as THREE from 'three';
import { decodeHeights, PLH1 } from './terrain_codec.js';

// Browser build (tools/build_web.py): small JSON files live in the release folder, the big binaries
// under content-hashed URLs - both immutable, so the HTTP cache may keep them. Dev: ./data/, revalidated.
const PL = globalThis.PLANELANDS || {};
const CACHE = PL.ver ? 'default' : 'no-cache';
export const dataURL = (name) => PL.files?.[name] || (PL.data || './data/') + name;

async function fetchBin(url, onProgress, size = 0) {
  const res = await fetch(url, { cache: CACHE });
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  // progress against the real size: with Content-Encoding (gzip) Content-Length is the compressed size
  const total = size || (res.headers.get('Content-Encoding') ? 0 : +res.headers.get('Content-Length') || 0);
  if (!res.body || !total || !onProgress) return new Uint8Array(await res.arrayBuffer());
  const reader = res.body.getReader();
  const chunks = [];
  let got = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value); got += value.length;
    onProgress(Math.min(got / total, 1));
  }
  const out = new Uint8Array(got);
  let o = 0;
  for (const c of chunks) { out.set(c, o); o += c.length; }
  return out;
}
async function fetchJSON(url) {
  const res = await fetch(url, { cache: CACHE });
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  return res.json();
}

// cloud shape / detail noise volumes: computed here rather than downloaded (render/cloud_noise.js)
function cloudNoise() {
  const direct = () => import('../render/cloud_noise.js').then((m) => ({ shape: m.shapeVolume(128, 11), detail: m.detailVolume(32, 23) }));
  return new Promise((resolve) => {
    let w;
    try { w = new Worker(new URL('../render/cloud_noise_worker.js', import.meta.url), { type: 'module' }); } catch { resolve(direct()); return; }
    w.onmessage = (e) => { resolve(e.data); w.terminate(); };
    w.onerror = () => { w.terminate(); resolve(direct()); };
    w.postMessage(0);
  });
}

export async function loadAssets(progress = () => {}) {
  const [world, geometry, meta] = await Promise.all([
    fetchJSON(dataURL('world.json')),
    fetchJSON(dataURL('orbiter_geometry.json')),
    fetchJSON(dataURL('terrain_meta.json')),
  ]);
  const clouds = cloudNoise();                     // generated in a worker meanwhile
  progress('地形高程', 0);
  let hBytes = await fetchBin(dataURL('terrain_height.f32'), (f) => progress('地形高程', f), PL.sizes?.['terrain_height.f32']);
  if (PL.enc?.['terrain_height.f32'] === PLH1) hBytes = decodeHeights(hBytes);
  progress('地表覆盖', 0);
  const lcBytes = await fetchBin(dataURL('terrain_landcover.u8'), (f) => progress('地表覆盖', f), PL.sizes?.['terrain_landcover.u8']);
  progress('体积云噪声', 0);
  const [{ shape, detail }, weather] = await Promise.all([clouds, fetchBin(dataURL('cloud_weather.u8'))]);
  progress('体积云噪声', 1);
  const height = new Float32Array(hBytes.buffer, hBytes.byteOffset, hBytes.byteLength / 4);

  const tex3 = (data, n) => {
    const t = new THREE.Data3DTexture(data, n, n, n);
    t.format = THREE.RGBAFormat; t.type = THREE.UnsignedByteType;
    t.minFilter = THREE.LinearFilter; t.magFilter = THREE.LinearFilter;
    t.wrapS = t.wrapT = t.wrapR = THREE.RepeatWrapping;
    t.unpackAlignment = 1;
    t.needsUpdate = true;
    return t;
  };
  const shapeTex = tex3(shape, 128);
  const detailTex = tex3(detail, 32);
  const weatherTex = new THREE.DataTexture(weather, 512, 512, THREE.RGBAFormat, THREE.UnsignedByteType);
  weatherTex.wrapS = weatherTex.wrapT = THREE.RepeatWrapping;
  weatherTex.minFilter = THREE.LinearMipmapLinearFilter; weatherTex.magFilter = THREE.LinearFilter;
  weatherTex.generateMipmaps = true;
  weatherTex.needsUpdate = true;
  return { world, geometry, meta, height, landcover: lcBytes, shapeTex, detailTex, weatherTex, weatherData: weather };
}
