// Boots Pyodide in a Web Worker with the simulator's Python package (browser build).
//   cfg.pyodide  base URL of a Pyodide distribution (pyodide.mjs, wasm, stdlib, package wheels)
//   cfg.bundle   zip of the Python sources + small data files (built by tools/build_web.py)
//   cfg.terrain  terrain_height.f32 (the same file the renderer loads - normally an HTTP cache hit),
//                cfg.terrainEnc 'plh1' if it is the losslessly coded download (core/terrain_codec.js)
import { decodeHeights, PLH1 } from '../core/terrain_codec.js';

export const PACKAGES = ['numpy', 'scipy', 'cvxpy-base', 'clarabel'];

async function fetchBytes(url, onProgress) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  const total = res.headers.get('Content-Encoding') ? 0 : +res.headers.get('Content-Length') || 0;
  if (!res.body || !total || !onProgress) return new Uint8Array(await res.arrayBuffer());
  const reader = res.body.getReader();
  const chunks = []; let got = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value); got += value.length;
    onProgress(Math.min(got / total, 1));
  }
  const out = new Uint8Array(got); let o = 0;
  for (const c of chunks) { out.set(c, o); o += c.length; }
  return out;
}

export async function bootPython(cfg, progress = () => {}, log = console.log) {
  progress('Python 运行时', 0);
  const { loadPyodide } = await import(cfg.pyodide + 'pyodide.mjs');
  const py = await loadPyodide({
    indexURL: cfg.pyodide,
    stdout: (s) => log(s), stderr: (s) => log(s),
  });
  progress('数值库', 0.3);
  await py.loadPackage(PACKAGES, { messageCallback: () => {}, errorCallback: (s) => log(s), checkIntegrity: true });
  progress('仿真代码', 0.8);
  const [zip, terrainRaw] = await Promise.all([fetchBytes(cfg.bundle), fetchBytes(cfg.terrain)]);
  const terrain = cfg.terrainEnc === PLH1 ? decodeHeights(terrainRaw) : terrainRaw;
  py.unpackArchive(zip, 'zip', { extractDir: '/app' });
  py.FS.mkdirTree('/app/web/data');
  py.FS.writeFile('/app/web/data/terrain_height.f32', terrain);
  py.runPython("import sys, os\nsys.path.insert(0, '/app')\nos.chdir('/app')");
  progress('仿真代码', 1);
  return py;
}
