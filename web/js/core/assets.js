// Loads the generated world data (web/data) and the shared JSON definitions.
import * as THREE from 'three';

async function fetchBin(url, onProgress) {
  const res = await fetch(url, { cache: 'no-cache' });
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  const total = +res.headers.get('Content-Length') || 0;
  if (!res.body || !total || !onProgress) return new Uint8Array(await res.arrayBuffer());
  const reader = res.body.getReader();
  const out = new Uint8Array(total);
  let got = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    out.set(value, got); got += value.length;
    onProgress(got / total);
  }
  return out;
}
async function fetchJSON(url) {
  const res = await fetch(url, { cache: 'no-cache' });
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  return res.json();
}

export async function loadAssets(progress = () => {}) {
  const base = './data/';
  const [world, geometry, meta] = await Promise.all([
    fetchJSON(base + 'world.json'),
    fetchJSON(base + 'orbiter_geometry.json'),
    fetchJSON(base + 'terrain_meta.json'),
  ]);
  progress('地形高程', 0);
  const hBytes = await fetchBin(base + 'terrain_height.f32', (f) => progress('地形高程', f));
  progress('地表覆盖', 0);
  const lcBytes = await fetchBin(base + 'terrain_landcover.u8', (f) => progress('地表覆盖', f));
  progress('体积云噪声', 0);
  const [shape, detail, weather] = await Promise.all([
    fetchBin(base + 'cloud_shape.u8'), fetchBin(base + 'cloud_detail.u8'), fetchBin(base + 'cloud_weather.u8'),
  ]);
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
