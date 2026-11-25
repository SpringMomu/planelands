// SCP trajectory planner in its own Web Worker (browser build) - the counterpart of the server
// build's planner process. Requests / results are pickled Python dicts (sim.gnc.planner.plan_pickled).
// A plan takes ~0.3-3 s; only the newest queued request is solved (older ones were superseded).
import { bootPython } from './pyboot.js';

let ready = null;
let planner = null;
let queued = null;
let running = false;

const log = (s) => { if (!/^\[gnc\]|^\[sim\]/.test(s)) console.log('[planner]', s); };

async function boot(cfg) {
  const py = await bootPython(cfg, (stage, f) => postMessage({ type: 'status', stage, f }), log);
  planner = py.pyimport('sim.gnc.planner');
  planner.init_worker();               // planning model, terrain field, SCP problem compilation
  postMessage({ type: 'ready' });
}

function run(m) {
  let out;
  try {
    const r = planner.plan_pickled(m.data);
    out = r.toJs();
    r.destroy();
  } catch (err) {
    postMessage({ type: 'error', seq: m.seq, text: String(err?.message || err) });
    return;
  }
  postMessage({ type: 'result', seq: m.seq, data: out }, [out.buffer]);
}

async function pump() {
  if (running || !queued) return;
  try { await ready; } catch { return; }
  if (running || !queued) return;
  running = true;
  const m = queued; queued = null;
  run(m);
  running = false;
  setTimeout(pump, 0);
}

onmessage = (e) => {
  const m = e.data;
  if (m.type === 'init') {
    ready = boot(m.cfg);
    ready.catch((err) => postMessage({ type: 'fatal', text: String(err?.message || err) }));
  } else if (m.type === 'plan') {
    queued = m;
    pump();
  }
};
