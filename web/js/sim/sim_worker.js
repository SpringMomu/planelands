// The whole simulator in the browser: Python (Pyodide) 6-DOF plant + FCS + guidance in this Web
// Worker, the SCP planner in a nested one (plan_worker.js). Speaks the same JSON messages as the
// server build's WebSocket, so the page cannot tell the difference (see core/local_sim.js).
//
//   page -> worker: {type:'init', cfg} | {type:'msg', text}
//   worker -> page: {type:'status', stage, f} | {type:'ready'} | {type:'data', text} (\n-separated
//                   JSON messages) | {type:'fatal', text}
import { bootPython } from './pyboot.js';

const PERIOD = 1000 / 60;          // state rate, like server.py
const BUDGET = 0.9 * PERIOD / 1000; // this thread does nothing else (the server build keeps 25 % for I/O)

let bsim = null;
let planner = null;
let plannerDead = null;
const inbox = [];

const log = (s) => console.log(s);
const fatal = (err) => postMessage({ type: 'fatal', text: String(err?.message || err) });

function startPlanner(cfg) {
  planner = new Worker(new URL('./plan_worker.js', import.meta.url), { type: 'module' });
  planner.onmessage = (e) => {
    const m = e.data;
    if (m.type === 'result') bsim?.plan_result(m.seq, m.data);
    else if (m.type === 'error') { console.warn('[planner]', m.text); bsim?.plan_error(m.seq, m.text); }
    else if (m.type === 'status') postMessage({ type: 'status', stage: m.stage, f: m.f, who: 'planner' });
    else if (m.type === 'fatal') { plannerDead = m.text; console.error('[planner] failed:', m.text); }
  };
  planner.onerror = (e) => { plannerDead = e.message || 'planner worker error'; console.error('[planner]', e); };
  planner.postMessage({ type: 'init', cfg });
}

// called from Python (ExternalPlanner.submit): pickled request bytes
function postPlan(seq, data) {
  const bytes = data.toJs();
  if (plannerDead) { setTimeout(() => bsim?.plan_error(seq, `planner unavailable: ${plannerDead}`), 0); return; }
  planner.postMessage({ type: 'plan', seq, data: bytes }, [bytes.buffer]);
}

function loop() {
  let last = performance.now();
  let deadline = last;
  const tick = () => {
    const t0 = performance.now();
    const dt = (t0 - last) / 1000;
    last = t0;
    while (inbox.length) bsim.handle(inbox.shift());
    let out = '';
    try { out = bsim.step(dt, BUDGET); } catch (err) { console.error(err); }
    if (out) postMessage({ type: 'data', text: out });
    deadline += PERIOD;
    const now = performance.now();
    if (now - deadline > 250) deadline = now;          // fell far behind (tab throttled, GC...): resync
    setTimeout(tick, Math.max(0, deadline - now));
  };
  tick();
}

async function init(cfg) {
  startPlanner(cfg);
  const py = await bootPython(cfg, (stage, f) => postMessage({ type: 'status', stage, f, who: 'sim' }), log);
  postMessage({ type: 'status', stage: '初始化仿真', f: 1, who: 'sim' });
  const mod = py.pyimport('sim.browser');
  bsim = mod.BrowserSim.callKwargs(postPlan, { spawn: cfg.spawn || 'feasible', guidance: cfg.guidance || 'scp_mpc' });
  postMessage({ type: 'ready' });
  postMessage({ type: 'data', text: bsim.hello() });
  loop();
}

onmessage = (e) => {
  const m = e.data;
  if (m.type === 'init') init(m.cfg).catch(fatal);
  else if (m.type === 'msg') inbox.push(m.text);
};
