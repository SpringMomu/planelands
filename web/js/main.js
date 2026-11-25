import * as THREE from 'three';
import { loadAssets, dataURL } from './core/assets.js';
import { SimLink } from './core/net.js';
import { LocalSim } from './core/local_sim.js';
import { enuToThree, bodyQuatToThree, curvatureDrop } from './core/frames.js';
import { Atmosphere, sunDirection, sunTransmittanceCPU } from './render/atmosphere.js';
import { Terrain } from './render/terrain.js';
import { Clouds } from './render/clouds.js';
import { Post } from './render/post.js';
import { Airport } from './world/airport.js';
import { Shuttle } from './world/shuttle.js';
import { patchStandard } from './world/materials.js';
import { Trail, ForceVectors } from './world/overlays.js';
import { DragChute } from './world/chute.js';
import { TireSmoke } from './world/smoke.js';
import { zhEvent } from './ui/i18n.js';
import { Weather } from './world/weather.js';
import { GncViz } from './world/gnc_viz.js';
import { CameraRig, CAMERA_MODES, CAMERA_LABELS } from './ui/cameras.js';
import { Hud } from './ui/hud.js';
import { SpawnPanel } from './ui/spawn.js';
import { Meds } from './ui/meds.js';
import { Cockpit } from './world/cockpit.js';

const $ = (s) => document.querySelector(s);
const ld = (txt, f) => { $('#ld-text').textContent = txt; if (f !== undefined) $('#ld-fill').style.width = `${Math.round(f * 100)}%`; };
const nextFrame = () => new Promise((r) => requestAnimationFrame(() => r()));
// keep in sync with the mobile @media block in css/style.css
const MOBILE_MQ = matchMedia('(max-width: 760px), (pointer: coarse) and (max-height: 540px)');

// mobile sheets (flight / navpanel / camerabar / settings): one open at a time, null closes
function openSheet(name) {
  if (name) document.body.dataset.sheet = name; else delete document.body.dataset.sheet;
  document.querySelectorAll('#dock [data-sheet]').forEach((b) => b.classList.toggle('on', b.dataset.sheet === name));
}

const QUALITY = {
  low: { cloudScale: 0.35, steps: 48, light: 4, ap: 8, pr: 0.75 },
  med: { cloudScale: 0.5, steps: 72, light: 5, ap: 10, pr: 0.9 },
  high: { cloudScale: 0.5, steps: 96, light: 6, ap: 14, pr: 1.0 },
  ultra: { cloudScale: 0.7, steps: 144, light: 8, ap: 18, pr: 1.0 },
};

function toast(text, level = 'info', ms = 4200) {
  const el = document.createElement('div');
  el.className = `toast ${level}`;
  el.textContent = text;
  $('#toasts').appendChild(el);
  setTimeout(() => el.classList.add('out'), ms);
  setTimeout(() => el.remove(), ms + 700);
  while ($('#toasts').children.length > 4) $('#toasts').firstChild.remove();
}

async function main() {
  // Where the simulator runs. Browser build (tools/build_web.py sets window.PLANELANDS.sim = 'browser';
  // or ?sim=browser): Python in Web Workers on this machine - started right away so Pyodide loads while
  // the world data and the scene do. Otherwise the server.py WebSocket backend.
  const PL = globalThis.PLANELANDS || {};
  const qs = new URLSearchParams(location.search);
  const abs = (u) => new URL(u, document.baseURI).href;
  const local = (qs.get('sim') || PL.sim) !== 'browser' ? null : new LocalSim({
    pyodide: abs(PL.pyodide || 'https://cdn.jsdelivr.net/pyodide/v314.0.8/full/'),
    bundle: abs(PL.bundle || 'py/sim.zip'),
    terrain: abs(dataURL('terrain_height.f32')),
    terrainEnc: PL.enc?.['terrain_height.f32'],
    spawn: qs.get('spawn') || 'feasible',
  });

  ld('加载世界数据…', 0.02);
  const A = await loadAssets((name, f) => ld(`加载${name}… ${Math.round(f * 100)}%`, 0.05 + 0.5 * f));
  await nextFrame();

  // ------------------------------------------------------------------ renderer
  const canvas = $('#gl');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: false, logarithmicDepthBuffer: true, powerPreference: 'high-performance', stencil: false });
  renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
  renderer.toneMapping = THREE.NoToneMapping;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.autoClear = false;
  const caps = renderer.capabilities;
  if (!caps.isWebGL2) throw new Error('需要 WebGL2');

  const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.3, 2.0e6);
  const scene = new THREE.Scene();

  // ------------------------------------------------------------------ shared uniforms
  const U = {
    uTransLUT: { value: null }, uMSLUT: { value: null }, uSkyLUT: { value: null }, uIrrLUT: { value: null },
    uHaze: { value: 1.2 }, uSunDir: { value: new THREE.Vector3(0, 1, 0) }, uSunIntensity: { value: 20 },
    uSkyUp: { value: new THREE.Vector3(0, 1, 0) }, uSkyEast: { value: new THREE.Vector3(1, 0, 0) }, uSkyNorth: { value: new THREE.Vector3(0, 0, -1) },
    uCamR: { value: 6371000 },
    uShapeTex: { value: A.shapeTex }, uDetailTex: { value: A.detailTex }, uWeatherTex: { value: A.weatherTex },
    uCloudBase: { value: 1600 }, uCloudThick: { value: 2200 }, uCloudCoverage: { value: 0.48 }, uCloudDensity: { value: 1.0 },
    uCloudType: { value: 0.55 }, uWindOffset: { value: new THREE.Vector2() }, uCloudShadowStrength: { value: 0.85 }, uCloudTime: { value: 0 },
    uTime: { value: 0 }, uNight: { value: 0 }, uEnvCloud: { value: 0.3 }, uOvercast: { value: 0 },
    // precipitation / thunderstorm
    uRain: { value: 0 }, uWet: { value: 0 }, uStormCell: { value: new THREE.Vector4(0, 0, 1, 0) }, uRainSlant: { value: new THREE.Vector2() },
    uFlash: { value: new THREE.Vector4() }, uFlashCol: { value: new THREE.Vector3(0.78, 0.85, 1.0) }, uFlashAmb: { value: 0 },
  };

  ld('预计算大气散射 LUT…', 0.6); await nextFrame();
  const atmo = new Atmosphere(renderer, U);

  ld('构建地形 LOD…', 0.66); await nextFrame();
  const terrain = new Terrain({ heightData: A.height, landcoverData: A.landcover, meta: A.meta, world: A.world, uniforms: U });
  scene.add(terrain.group);

  ld('构建机场与跑道…', 0.74); await nextFrame();
  const airport = new Airport(A.world, U, terrain);
  scene.add(airport.group);

  ld('构建航天飞机模型…', 0.8); await nextFrame();
  let geometry = A.geometry;
  const shuttle = new Shuttle(geometry);
  shuttle.group.traverse((o) => { if (o.isMesh && o.material && o.material.isMeshStandardMaterial && !o.material.userData.csPatched) { patchStandard(o.material, U, { key: 'orb' }); o.material.userData.csPatched = true; } });
  scene.add(shuttle.group);
  const forces = new ForceVectors(shuttle.model);
  const trail = new Trail();
  scene.add(trail.line);
  const chute = new DragChute(geometry, U);
  scene.add(chute.group);
  const smoke = new TireSmoke();
  const weather = new Weather({ U, weatherData: A.weatherData, scene });
  const gncViz = new GncViz(terrain);
  scene.add(gncViz.group);
  let lastPlan = null, lastMpc = null;
  let prevWow = null;
  // tyre peripheral speed per gear (m/s): spun up by the runway, slowly spinning down in the air
  // (bearing friction + windage); the touchdown smoke comes from the spin-up slip
  const wheelV = [0, 0, 0];

  // sun light (shadows for the orbiter + its shadow on the ground)
  const sun = new THREE.DirectionalLight(0xffffff, 20);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.bias = -0.0003;
  sun.shadow.normalBias = 0.03;
  sun.shadow.camera.near = 1; sun.shadow.camera.far = 4000;
  scene.add(sun); scene.add(sun.target);

  ld('编译着色器…', 0.86); await nextFrame();
  const clouds = new Clouds(renderer, U, A);
  const post = new Post(renderer, U);

  // ------------------------------------------------------------------ render targets / resize
  let sceneRT = null;
  // phones / small touch screens start on the low preset (volumetric clouds are fill-rate heavy)
  let quality = 'low', renderScale = 1.0;
  $('#csteps').value = QUALITY[quality].steps; $('#csteps-o').textContent = QUALITY[quality].steps;
  document.querySelectorAll('#q-seg button').forEach((x) => x.classList.toggle('on', x.dataset.q === quality));
  const settings = { exposureEV: 0, bloom: 0.06, showHud: false, showTrail: true, showForces: false };
  function resize() {
    const q = QUALITY[quality];
    const pr = Math.min(devicePixelRatio, 1.5) * q.pr * renderScale;
    const w = Math.max(2, Math.floor(innerWidth * pr)), h = Math.max(2, Math.floor(innerHeight * pr));
    renderer.setPixelRatio(1);
    renderer.setSize(w, h, false);
    canvas.style.width = innerWidth + 'px'; canvas.style.height = innerHeight + 'px';
    sceneRT?.dispose();
    const depthTexture = new THREE.DepthTexture(w, h, THREE.FloatType);
    sceneRT = new THREE.WebGLRenderTarget(w, h, { type: THREE.HalfFloatType, samples: 4, depthTexture, depthBuffer: true, stencilBuffer: false });
    clouds.scale = q.cloudScale;
    clouds.setSize(w, h);
    post.setSize(w, h);
    smoke.setSize(w, h);
    weather.rain.setSize(w, h);
    weather.lightning.setResolution(w, h);
    camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
    hud.resize(innerWidth, innerHeight, Math.min(devicePixelRatio, 2));
    clouds.cu.uSteps.value = +$('#csteps').value;
    clouds.cu.uLightSteps.value = q.light;
    post.cu.uAPSteps.value = q.ap;
  }

  const hud = new Hud({ canvas: $('#hud'), panel: $('#flight'), surfCanvas: $('#surf'), mapCanvas: $('#map'), vsitCanvas: $('#vsit'), geometry, world: A.world, terrain });
  const rig = new CameraRig(camera, canvas, terrain, airport);
  // MEDS glass-cockpit windows (A/E PFD, HSIT nav display, SPI, DPS pages): F / N or the dock buttons
  const meds = new Meds({ dock: $('#camerabar') });
  // 3D flight deck for the cockpit view (built on first use, parented to the orbiter)
  const cockpit = new Cockpit({ U, meds });
  resize();
  addEventListener('resize', resize);

  // ------------------------------------------------------------------ simulator link
  let elements = null;
  // WebSocket backend (server.py): port ?ws=PORT, else whatever the HTTP server reports, else the default
  let wsPort = +qs.get('ws') || 0;
  if (!wsPort && !local) {
    try { wsPort = (await (await fetch('api/config', { cache: 'no-store' })).json()).ws_port || 0; } catch { wsPort = 0; }
  }
  const linkPill = (text) => { $('#pill-link em').textContent = text; };
  const link = new SimLink({
    port: wsPort || 8765,
    local,
    onLoading: (m) => {
      if (link.connected) return;
      linkPill(`加载仿真 · ${m.who === 'planner' ? '规划器 ' : ''}${m.stage} ${Math.round((m.f || 0) * 100)}%`);
    },
    onHello: (m) => {
      if (m.elements) { elements = m.elements; forces.setElements(m.elements); }
      if (m.guidance && m.guidance.available) {
        const sel = $('#guid-sel');
        const names = { scp_mpc: 'SCP + MPC', hold_glide: '占位滑翔（对比）' };
        sel.innerHTML = m.guidance.available.map((g) => `<option value="${g}">${names[g] || g}</option>`).join('');
        sel.value = m.guidance.active;
      }
      toast(local ? '浏览器内 Python 仿真已就绪（6-DOF 气动 + 飞控 + SCP/MPC 制导）' : '已连接 Python 仿真后端（6-DOF 气动 + 飞控）', 'info');
      sendEnv(true);
      setTimeout(() => spawnPanel?.requestCheck(), 50);
    },
    onAero: (m) => { if (settings.showForces) forces.update(m.f); lastAero = m; },
    onPlan: (m) => { lastPlan = m; gncViz.setPlan(m); },
    // every raw state (also while the tab is hidden): trail / map track / vertical-situation history
    onState: (m) => {
      trail.pushRaw(m.t, m.pos, Math.hypot(m.vel[0], m.vel[1], m.vel[2]));
      hud.pushTrail(m.pos);
      hud.pushProfile(m);
    },
    onEpisode: () => { trail.clear(); hud.clearTrail(); smoke.clear(); chute.reset(); shuttle.setChuteDoor(false); prevWow = null; meds.resetAlarms(); },
    onMpc: (m) => { lastMpc = m.pred; gncViz.setMpc(m.pred); },
    onEvent: (m) => { toast(zhEvent(m.text), m.level || 'info', m.level === 'error' ? 8000 : 5000); meds.event(m); },
    onStatus: (s, info) => {
      const p = $('#pill-link');
      p.classList.toggle('ok', s === 'online'); p.classList.toggle('bad', s !== 'online');
      p.querySelector('em').textContent = s === 'online' ? (local ? '本机 6-DOF' : '在线 6-DOF')
        : s === 'failed' ? '仿真加载失败 · 本地预览' : s === 'local' ? (local ? '加载仿真…' : '离线 · 本地预览') : '断开';
      if (s === 'failed') toast(`浏览器内仿真加载失败：${info}`, 'error', 12000);
    },
  });
  let lastAero = null;
  // browser build: tell open pages when a new release has been deployed (index.html is revalidated)
  if (PL.ver) {
    let told = false;
    setInterval(async () => {
      if (told || document.hidden) return;
      try {
        const v = await (await fetch(PL.versionURL || 'version.json', { cache: 'no-store' })).json();
        if (v.ver && v.ver !== PL.ver) { told = true; toast('网站已更新：刷新页面即可使用新版本', 'info', 20000); }
      } catch { /* offline */ }
    }, 5 * 60 * 1000);
  }
  // start-position editor (own tab): place the vehicle anywhere and start from there
  const spawnPanel = new SpawnPanel({ root: $('#spawn-panel'), hud, link, world: A.world, onStart: () => clearEpisode() });
  link.onSpawnCheck = (m) => spawnPanel.onCheck(m);

  // ------------------------------------------------------------------ UI bindings
  const bindRange = (id, fmt, fn) => {
    const el = $('#' + id), out = $('#' + id + '-o');
    const upd = () => { const v = +el.value; if (out) out.textContent = fmt(v); fn(v); };
    el.addEventListener('input', upd); upd();
    return el;
  };
  const TS = [0.25, 1, 2, 4, 8, 16];
  bindRange('ts', (v) => `${TS[v]}×`, (v) => link.send({ type: 'sim', action: 'set_time_scale', value: TS[v] }));
  // wind: sliders are the means; with automatic weather the simulator lets them wander and the storm
  // forcing (cloud field at the orbiter) makes them rougher inside / under cumulonimbus
  function sendEnv(quiet = false) {
    link.send({ type: 'env', wind_speed: +$('#wind').value, wind_dir: +$('#winddir').value, turbulence: +$('#turb').value,
      auto: $('#wind-auto').checked, quiet: quiet === true });
  }
  bindRange('wind', (v) => `${v} m/s`, sendEnv);
  bindRange('winddir', (v) => `${v}°`, sendEnv);
  bindRange('turb', (v) => v.toFixed(2), sendEnv);
  $('#wind-auto').addEventListener('change', () => { sendEnv(); if (!$('#wind-auto').checked) link.send({ type: 'storm', turb: 0, gust: 0, draft: 0 }); });
  let stormSendT = 0;
  bindRange('cbase', (v) => `${v} m`, (v) => { U.uCloudBase.value = v; });
  bindRange('cthick', (v) => `${v} m`, (v) => { U.uCloudThick.value = v; });
  bindRange('ccov', (v) => v.toFixed(2), (v) => { U.uCloudCoverage.value = v; });
  bindRange('cdens', (v) => v.toFixed(2), (v) => { U.uCloudDensity.value = v; });
  bindRange('ctype', (v) => v.toFixed(2), (v) => { U.uCloudType.value = v; });
  bindRange('cirrus', (v) => v.toFixed(2), (v) => { post.cu.uCirrus.value = v; });
  bindRange('cshadow', (v) => v.toFixed(2), (v) => { U.uCloudShadowStrength.value = v; });
  let tod = 15.6, doy = 172;
  bindRange('tod', (v) => `${String(Math.floor(v)).padStart(2, '0')}:${String(Math.floor((v % 1) * 60)).padStart(2, '0')}`, (v) => { tod = v; });
  bindRange('doy', (v) => String(v), (v) => { doy = v; });
  bindRange('haze', (v) => v.toFixed(2), (v) => { U.uHaze.value = v; });
  bindRange('ev', (v) => v.toFixed(1), (v) => { settings.exposureEV = v; });
  bindRange('bloom', (v) => v.toFixed(3), (v) => { post.fu.uBloomStrength.value = v; });
  bindRange('rscale', (v) => `${Math.round(v * 100)}%`, (v) => { renderScale = v; resize(); });
  bindRange('csteps', (v) => String(v), (v) => { clouds.cu.uSteps.value = v; });
  bindRange('cdist', (v) => `${v} km`, (v) => { clouds.cu.uMaxDist.value = v * 1000; });
  // precipitation / thunderstorm
  bindRange('rain', (v) => v.toFixed(2), (v) => { weather.settings.rain = v; });
  bindRange('lightning', (v) => v.toFixed(2), (v) => { weather.settings.lightning = v; });
  const wetEl = bindRange('wet', (v) => (weather.settings.wetAuto ? `自动 ${weather.wet.toFixed(2)}` : v.toFixed(2)), (v) => { weather.settings.wet = v; });
  wetEl.addEventListener('input', () => { weather.settings.wetAuto = false; $('#wet-auto').checked = false; $('#wet-o').textContent = (+wetEl.value).toFixed(2); });
  $('#wet-auto').addEventListener('change', (e) => { weather.settings.wetAuto = e.target.checked; if (!e.target.checked) weather.settings.wet = weather.wet; });
  $('#storm-cell').addEventListener('change', (e) => { weather.settings.cell = e.target.checked; clouds.invalidate(); });
  $('#sound').addEventListener('change', (e) => weather.audio.setEnabled(e.target.checked));
  $('#btn-strike').addEventListener('click', () => weather.strikeNow(camera, groundY));
  $('#show-forces').addEventListener('change', (e) => { settings.showForces = e.target.checked; forces.group.visible = e.target.checked; });
  $('#show-trail').addEventListener('change', (e) => { settings.showTrail = e.target.checked; trail.line.visible = e.target.checked; });
  bindRange('headshake', (v) => (v === 0 ? '关' : v.toFixed(2)), (v) => { rig.shake = v; });
  $('#show-hud').addEventListener('change', (e) => { settings.showHud = e.target.checked; });
  $('#show-plan').addEventListener('change', (e) => gncViz.setVisible('plan', e.target.checked));
  $('#show-mpc').addEventListener('change', (e) => gncViz.setVisible('mpc', e.target.checked));
  $('#show-appr').addEventListener('change', (e) => gncViz.setVisible('approach', e.target.checked));
  $('#guid-sel').addEventListener('change', (e) => link.send({ type: 'sim', action: 'set_guidance', value: e.target.value }));
  document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
    document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('on', x === b));
    document.querySelectorAll('section[data-pane]').forEach((s) => s.classList.toggle('on', s.dataset.pane === b.dataset.tab));
  }));
  let simMode = 'AUTO';
  document.querySelectorAll('#mode-seg button').forEach((b) => b.addEventListener('click', () => {
    simMode = b.dataset.mode;
    document.querySelectorAll('#mode-seg button').forEach((x) => x.classList.toggle('on', x === b));
    document.body.classList.toggle('manual', simMode === 'MANUAL');
    link.send({ type: 'mode', mode: simMode });
    // phone: switching to manual flight closes the sheet so the virtual stick shows up
    if (simMode === 'MANUAL' && MOBILE_MQ.matches) openSheet(null);
  }));
  document.querySelectorAll('#q-seg button').forEach((b) => b.addEventListener('click', () => {
    quality = b.dataset.q;
    document.querySelectorAll('#q-seg button').forEach((x) => x.classList.toggle('on', x === b));
    $('#csteps').value = QUALITY[quality].steps; $('#csteps-o').textContent = QUALITY[quality].steps;
    resize();
  }));
  const PRESETS = {
    fair: { cbase: 1500, cthick: 1300, ccov: 0.30, cdens: 0.9, ctype: 0.5, cirrus: 0.35, rain: 0, lightning: 0, cell: false },
    scattered: { cbase: 1600, cthick: 2200, ccov: 0.48, cdens: 1.0, ctype: 0.55, cirrus: 0.55, rain: 0, lightning: 0, cell: false },
    broken: { cbase: 1200, cthick: 2600, ccov: 0.66, cdens: 1.2, ctype: 0.6, cirrus: 0.4, rain: 0, lightning: 0, cell: false },
    overcast: { cbase: 900, cthick: 900, ccov: 0.92, cdens: 1.4, ctype: 0.08, cirrus: 0.0, rain: 0, lightning: 0, cell: false },
    storm: { cbase: 1000, cthick: 7500, ccov: 0.58, cdens: 1.6, ctype: 0.95, cirrus: 0.75, rain: 1, lightning: 0.6, cell: true },
  };
  document.querySelectorAll('.presets button').forEach((b) => b.addEventListener('click', () => {
    const p = PRESETS[b.dataset.preset];
    for (const [k, v] of Object.entries(p)) { if (k === 'cell') continue; const el = $('#' + k); el.value = v; el.dispatchEvent(new Event('input')); }
    $('#storm-cell').checked = !!p.cell; weather.settings.cell = !!p.cell;
    // switching into a storm: the ground has already been rained on
    if (p.rain > 0 && weather.settings.wetAuto) weather.wet = Math.max(weather.wet, 0.85);
    clouds.invalidate();
  }));
  const clearEpisode = () => { trail.clear(); hud.clearTrail(); gncViz.clear(); lastPlan = null; lastMpc = null; };
  $('#btn-reset').addEventListener('click', () => { link.send({ type: 'sim', action: 'reset' }); clearEpisode(); });
  $('#btn-pause').addEventListener('click', () => link.send({ type: 'sim', action: 'toggle_pause' }));
  $('#btn-final').addEventListener('click', () => {
    link.send({ type: 'sim', action: 'reset', scenario: 'final' }); clearEpisode();
    simMode = 'AUTO'; document.querySelectorAll('#mode-seg button').forEach((x) => x.classList.toggle('on', x.dataset.mode === 'AUTO'));
    document.body.classList.remove('manual');
    link.send({ type: 'mode', mode: 'AUTO' });
  });

  // ------------------------------------------------------------------ mobile: dock + sheets
  document.querySelectorAll('#dock [data-sheet]').forEach((b) => b.addEventListener('click', () => {
    openSheet(document.body.dataset.sheet === b.dataset.sheet ? null : b.dataset.sheet);
  }));
  const dockPause = $('#dock-pause');
  dockPause.addEventListener('click', () => link.send({ type: 'sim', action: 'toggle_pause' }));
  // a quick tap on the 3D view closes the open sheet (drags still orbit the camera)
  let tap = null;
  canvas.addEventListener('pointerdown', (e) => { tap = e.isPrimary ? { x: e.clientX, y: e.clientY, t: performance.now() } : null; });
  const raycaster = new THREE.Raycaster(), ndc = new THREE.Vector2();
  canvas.addEventListener('pointerup', (e) => {
    if (tap && e.isPrimary && performance.now() - tap.t < 350 && Math.hypot(e.clientX - tap.x, e.clientY - tap.y) < 8) {
      openSheet(null);
      if (rig.mode === 'cockpit') {             // press an MDU edge key in the flight deck
        const r = canvas.getBoundingClientRect();
        ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
        raycaster.setFromCamera(ndc, camera);
        cockpit.click(raycaster.ray);
      }
    }
    tap = null;
  });
  MOBILE_MQ.addEventListener('change', (e) => { if (!e.matches) openSheet(null); });
  const mstrip = [...document.querySelectorAll('#mstrip [data-m]')].map((el) => [el, $(`#flight [data-f="${el.dataset.m}"]`)]);
  const cams = $('#cams');
  CAMERA_MODES.forEach((m, i) => {
    const b = document.createElement('button');
    b.innerHTML = `<kbd>${i + 1}</kbd>${CAMERA_LABELS[m]}`;
    b.dataset.cam = m;
    b.addEventListener('click', () => { setCam(m); if (MOBILE_MQ.matches) openSheet(null); });
    cams.appendChild(b);
  });
  function setCam(m) {
    rig.setMode(m);
    cams.querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.cam === m));
    clouds.invalidate();
  }
  setCam('chase');

  // keyboard: cameras, sim, manual control
  const keys = new Set();
  const ctl = { gear: false, chute: false, sb: 0, brakes: 0 };
  addEventListener('keydown', (e) => {
    if (e.target.closest && e.target.closest('input,select,textarea')) return;
    keys.add(e.code);
    const n = parseInt(e.key, 10);
    if (n >= 1 && n <= CAMERA_MODES.length) setCam(CAMERA_MODES[n - 1]);
    if (e.code === 'KeyP') link.send({ type: 'sim', action: 'toggle_pause' });
    if (e.code === 'KeyR') { link.send({ type: 'sim', action: 'reset' }); clearEpisode(); }
    if (e.code === 'KeyH') document.body.classList.toggle('ui-hidden');
    if (rig.mode !== 'free') {
      if (e.code === 'KeyG') { ctl.gear = !ctl.gear; toast(ctl.gear ? '起落架放下指令' : '起落架收起指令'); }
      if (e.code === 'KeyC') { ctl.chute = !ctl.chute; toast(ctl.chute ? '减速伞展开指令' : '减速伞抛离指令'); }
    }
  });
  addEventListener('keyup', (e) => keys.delete(e.code));

  // touch: virtual stick (drag down = pull = nose up, like S / ArrowDown) + hold / toggle buttons
  const touch = { pitch: 0, roll: 0, yawL: false, yawR: false, sbUp: false, sbDn: false, brake: false };
  {
    const stick = $('#stick'), knob = stick.querySelector('i');
    const expo = (v) => Math.sign(v) * Math.abs(v) ** 1.5;   // finer control around centre
    let sid = null;
    const move = (e) => {
      const r = stick.getBoundingClientRect(), R = r.width / 2;
      let x = (e.clientX - r.left - R) / R, y = (e.clientY - r.top - R) / R;
      const m = Math.hypot(x, y);
      if (m > 1) { x /= m; y /= m; }
      touch.roll = expo(x); touch.pitch = expo(y);
      knob.style.transform = `translate(${x * R * 0.6}px, ${y * R * 0.6}px)`;
    };
    const end = (e) => {
      if (e.pointerId !== sid) return;
      sid = null; touch.roll = touch.pitch = 0;
      knob.style.transform = ''; stick.classList.remove('on');
    };
    stick.addEventListener('pointerdown', (e) => { sid = e.pointerId; stick.setPointerCapture(sid); stick.classList.add('on'); move(e); });
    stick.addEventListener('pointermove', (e) => { if (e.pointerId === sid) move(e); });
    stick.addEventListener('pointerup', end);
    stick.addEventListener('pointercancel', end);
    document.querySelectorAll('#touchctl [data-hold]').forEach((b) => {
      const set = (v) => { touch[b.dataset.hold] = v; b.classList.toggle('held', v); };
      b.addEventListener('pointerdown', (e) => { b.setPointerCapture(e.pointerId); set(true); });
      b.addEventListener('pointerup', () => set(false));
      b.addEventListener('pointercancel', () => set(false));
      b.addEventListener('contextmenu', (e) => e.preventDefault());
    });
    document.querySelectorAll('#touchctl [data-tog]').forEach((b) => b.addEventListener('click', () => {
      if (rig.mode === 'free') return;
      const k = b.dataset.tog;
      ctl[k] = !ctl[k];
      toast(k === 'gear' ? (ctl.gear ? '起落架放下指令' : '起落架收起指令') : (ctl.chute ? '减速伞展开指令' : '减速伞抛离指令'));
    }));
  }
  const tcGear = $('#tc-gear'), tcChute = $('#tc-chute'), tcSb = $('#tc-sb');
  const clamp1 = (v) => Math.max(-1, Math.min(1, v));
  setInterval(() => {
    if (simMode !== 'MANUAL' || rig.mode === 'free') return;
    const k = (a, b) => (keys.has(a) || keys.has(b) ? 1 : 0);
    if (keys.has('KeyX') || touch.sbUp) ctl.sb = Math.min(1, ctl.sb + 0.05);
    if (keys.has('KeyZ') || touch.sbDn) ctl.sb = Math.max(0, ctl.sb - 0.05);
    ctl.brakes = keys.has('KeyB') || touch.brake ? 1 : 0;
    tcGear.classList.toggle('on', ctl.gear); tcChute.classList.toggle('on', ctl.chute);
    tcSb.style.width = `${Math.round(ctl.sb * 100)}%`;
    link.send({
      type: 'control',
      pitch: clamp1(k('KeyS', 'ArrowDown') - k('KeyW', 'ArrowUp') + touch.pitch),
      roll: clamp1(k('KeyD', 'ArrowRight') - k('KeyA', 'ArrowLeft') + touch.roll),
      yaw: clamp1(k('KeyE', 'KeyE') - k('KeyQ', 'KeyQ') + (touch.yawR ? 1 : 0) - (touch.yawL ? 1 : 0)),
      speedbrake: ctl.sb, gear: ctl.gear, brakes: ctl.brakes, chute: ctl.chute,
    });
  }, 33);

  // ------------------------------------------------------------------ frame loop
  const pos3 = new THREE.Vector3(), vel3 = new THREE.Vector3(), quat3 = new THREE.Quaternion();
  const tmpQ = new THREE.Quaternion(), tmpV = new THREE.Vector3(), tmpV2 = new THREE.Vector3();
  const wind3 = new THREE.Vector3(), vAir3 = new THREE.Vector3(), attachW = new THREE.Vector3();
  const sunRad = new THREE.Color(), ambC = new THREE.Color();
  const contacts = [];
  const groundY = (x, z) => enuToThree(x, -z, terrain.surfaceAt(x, -z), tmpV).y;
  let prevEp = -1;
  let last = performance.now(), fpsAcc = 0, fpsN = 0, simState = null, todAnim = false;
  $('#tod-anim').addEventListener('change', (e) => { todAnim = e.target.checked; });
  let windOff = new THREE.Vector2();
  let frame = 0;

  ld('就绪', 1);
  setTimeout(() => $('#loading').classList.add('done'), 250);

  function loop() {
    requestAnimationFrame(loop);
    const now = performance.now();
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    fpsAcc += dt; fpsN++;
    if (fpsAcc > 0.5) { $('#pill-fps em').textContent = (fpsN / fpsAcc).toFixed(0); fpsAcc = 0; fpsN = 0; }
    frame++;

    // ---------------- simulation state
    const s = link.sample(dt);
    if (s) simState = s;
    const st = simState;
    if (st && !st.gnc && window.__planelands && window.__planelands.fakeGnc) st.gnc = window.__planelands.fakeGnc;   // debug preview only
    const ts = st ? (st.paused ? 0 : st.time_scale || 1) : 1;
    U.uTime.value += dt;
    U.uCloudTime.value += dt * ts;

    if (todAnim) { tod += dt * 120 / 3600; if (tod > 20.5) tod = 4.5; $('#tod').value = tod; $('#tod').dispatchEvent(new Event('input')); }
    sunDirection(28.6, doy, tod, U.uSunDir.value);
    const sd = U.uSunDir.value;
    const night = THREE.MathUtils.smoothstep(-sd.y, -0.02, 0.12);
    U.uNight.value = night;

    // wind drift of the clouds
    const ws = +$('#wind').value, wd = +$('#winddir').value * Math.PI / 180;
    const vE = -ws * Math.sin(wd), vN = -ws * Math.cos(wd);
    windOff.x -= vE * dt * ts * 1.0; windOff.y -= -vN * dt * ts * 1.0;
    U.uWindOffset.value.copy(windOff);

    let agl = 1e4;
    if (st) {
      enuToThree(st.pos[0], st.pos[1], st.pos[2], pos3);
      vel3.set(st.vel[0], st.vel[2], -st.vel[1]);
      bodyQuatToThree(st.quat, pos3, quat3);
      shuttle.group.position.copy(pos3);
      shuttle.group.quaternion.copy(quat3);
      shuttle.setSurfaces(st.surf);
      const gs = Math.hypot(st.vel[0], st.vel[1]);
      shuttle.setGear(st.gear ? st.gear.pos : 0, st.gear?.compress, st.gear?.wow, dt * ts, gs);
      agl = st.air?.agl ?? (st.pos[2] - terrain.surfaceAt(st.pos[0], st.pos[1]));
      trail.update(st.t, st.pos);
      $('#pill-status em').textContent = st.status || '—';
      $('#pill-mode em').textContent = st.mode || '—';
      $('#pill-guid em').textContent = st.gnc ? `${st.gnc.phase}${st.gnc.sub ? ' · ' + st.gnc.sub : ''}` : (st.guidance || '—');
      $('#pill-time em').textContent = `${(st.t || 0).toFixed(1)} s${st.paused ? ' ❚❚' : ''}`;
      $('#btn-pause').textContent = st.paused ? '▶ 继续' : '❚❚ 暂停';
      dockPause.classList.toggle('paused', !!st.paused);
    }

    // ---------------- camera
    const inCockpit = rig.mode === 'cockpit';
    if (inCockpit && !cockpit.built) { cockpit.build(); }
    cockpit.group.visible = inCockpit;
    // the trajectory overlays are not part of the real flight deck: the HUD carries the guidance there
    gncViz.group.visible = !inCockpit;
    trail.line.visible = settings.showTrail && !inCockpit;
    rig.update(dt, { pos: pos3, quat: quat3, vel: vel3, eye: shuttle.cockpitEye, st });
    shuttle.model.visible = !inCockpit;
    const near = inCockpit ? 0.1 : 0.3;          // (the composite's depth unprojection needs far/near <= ~2e7)
    if (camera.near !== near) { camera.near = near; camera.updateProjectionMatrix(); }

    // ---------------- lighting
    // bright cloud deck below the camera brightens the IBL lower hemisphere
    const aboveClouds = (st ? st.pos[2] : camera.position.y) > U.uCloudBase.value + U.uCloudThick.value * 0.5;
    U.uEnvCloud.value = THREE.MathUtils.clamp(U.uCloudCoverage.value * (aboveClouds ? 0.85 : 0.25), 0, 0.8);
    const belowTop = camera.position.y < U.uCloudBase.value + U.uCloudThick.value;
    U.uOvercast.value = belowTop ? THREE.MathUtils.clamp((U.uCloudCoverage.value - 0.2) * 1.2 * Math.min(1, U.uCloudDensity.value), 0, 0.8) : 0;
    U.uOvercast.value = Math.max(U.uOvercast.value, weather.rainCam * 0.85);
    const camAlt = atmo.update(camera.position);
    scene.environment = atmo.envTexture;
    const sh = st ? st.pos[2] : 3000;
    const T = sunTransmittanceCPU(sh, sd.y, U.uHaze.value);
    sun.color.setRGB(T[0], T[1], T[2]);
    sun.intensity = U.uSunIntensity.value;
    // shadow frustum: around the orbiter (and its ground shadow), or tight around the flight deck so
    // the window frames throw crisp sun patches into the cabin
    const sc = sun.shadow.camera;
    const half = THREE.MathUtils.lerp(28, 230, THREE.MathUtils.smoothstep(-agl, -1500, -60));
    if (sc.right !== half) { sc.left = -half; sc.right = half; sc.top = half; sc.bottom = -half; sc.updateProjectionMatrix(); }
    sun.position.copy(pos3).addScaledVector(sd, 1500);
    sun.target.position.copy(pos3);
    sun.target.updateMatrixWorld();
    if (inCockpit) cockpit.place(camera, pos3, quat3, shuttle.cockpitEye, sd, sun, atmo.envTexture);
    terrain.update(camera, { on: !inCockpit && !!sun.shadow.map && agl < 2500 && sd.y > 0, map: sun.shadow.map ? sun.shadow.map.texture : null, matrix: sun.shadow.matrix, size: 2048 });
    airport.update(U.uTime.value, night);

    // ---------------- drag chute + tyre smoke (sim-time driven)
    const light = { sunDir: sd, sunRad: sunRad.copy(sun.color).multiplyScalar(sun.intensity),
      amb: ambC.setRGB(0.55, 0.65, 0.85).multiplyScalar(U.uSunIntensity.value * 0.09 * THREE.MathUtils.clamp(sd.y * 2 + 0.05, 0.02, 1)) };
    if (st) {
      const sdt = dt * ts;
      const w = st.air?.wind || [0, 0, 0];
      wind3.set(w[0], w[2], -w[1]);
      shuttle.group.updateMatrixWorld(true);
      chute.update(sdt, {
        state: st.chute || 0, attach: shuttle.chuteAttachWorld(attachW), vAir: vAir3.copy(vel3).sub(wind3), vel: vel3, wind: wind3,
        groundY, sunDir: sd, sunRad: light.sunRad, camera, time: st.t || 0,
      });
      shuttle.setChuteDoor(chute.mode !== 'stowed');
      // touchdown spin-up smoke.  A tyre touching down skids until its peripheral speed has caught
      // up with the ground speed; the rubber burnt in that skid is the smoke, and the energy dissipated
      // in it is proportional to the square of the speed difference.  A wheel still spinning from the
      // first contact (bounce) therefore makes (almost) no smoke on the second one, a wet runway little
      const wow = st.gear?.wow || [false, false, false];
      const gsp = Math.hypot(st.vel[0], st.vel[1]);
      const fresh = !prevWow || prevEp !== st._ep;
      for (let i = 0; i < 3; i++) {
        if (fresh) { wheelV[i] = wow[i] ? gsp : 0; continue; }           // new flight: wheels at rest in the air
        if (wow[i]) {
          if (!prevWow[i]) {
            const dv = Math.max(gsp - wheelV[i], 0);
            const dry = 1 - 0.85 * THREE.MathUtils.clamp(U.uWet.value, 0, 1);
            const str = Math.min(1.2, (dv / 95) ** 2) * dry * (i === 0 ? 0.6 : 1);
            if (str > 0.015) smoke.touchdown(i, str, dv);
          }
          wheelV[i] = gsp;                                                 // rolling
        } else {
          wheelV[i] = Math.max(0, wheelV[i] * Math.exp(-sdt / 25) - 0.3 * sdt);
        }
      }
      if (prevEp !== st._ep) cockpit.rain?.clear();        // new episode: fresh, dry windows (re-wetted from the weather)
      prevWow = wow.slice(); prevEp = st._ep;
      smoke.update(sdt, { contacts: shuttle.wheelContacts(contacts), vel: vel3, wind: wind3, groundY, brakes: st.brakes || 0, gs: gsp, wow });
    }

    // ---------------- weather: rain, lightning, wet ground
    {
      const cgY = groundY(camera.position.x, camera.position.z);
      weather.update(dt * ts, dt, {
        camera, vehPos3: st ? pos3 : null, vehAlt: st ? st.pos[2] : 0, vehAgl: agl, vehTas: st ? (st.air?.tas ?? 0) : 0,
        onGround: !!(st && st.gear?.wow?.some((w) => w)), camDist: st ? camera.position.distanceTo(pos3) : 1e9,
        wind3, groundY, groundYCam: cgY, camAgl: camera.position.y - cgY, amb: light.amb, ts,
      });
      shuttle.setWet(weather.bodyWet, weather.flowTime);
      // storm forcing for the flight model (a few times per second)
      stormSendT += dt;
      if (st && stormSendT > 0.25 && $('#wind-auto').checked) {
        stormSendT = 0;
        const f = weather.storm;
        link.send({ type: 'storm', turb: +f.turb.toFixed(3), gust: +f.gust.toFixed(2), draft: +f.draft.toFixed(2) });
      }
      if (st && st.env && frame % 15 === 0) {
        const e = st.env;
        $('#wx-live').innerHTML = `当前风 <b>${e.wind.toFixed(1)}</b> m/s · 来向 <b>${e.dir.toFixed(0)}°</b> · 湍流 <b>${e.turb.toFixed(2)}</b>` +
          ` · 垂直气流 <b>${e.draft >= 0 ? '+' : ''}${e.draft.toFixed(1)}</b> m/s${e.storm > 0.05 ? ` · <span class="stormtag">风暴 ${(e.storm * 100).toFixed(0)}%</span>` : ''}` +
          `${weather.bodyWet > 0.05 ? ` · 机身潮湿 ${(weather.bodyWet * 100).toFixed(0)}%` : ''}`;
      }
      // less history when the camera races through cloud (reprojection of near fog is approximate) or lightning flashes
      clouds.resolveU.uHistW.value = weather.flashActive ? 0.5 : 0.9 - 0.25 * THREE.MathUtils.smoothstep(weather.camVel.length() * ts, 60, 250);
      post.cu.uRainNow.value = weather.rainCam;
      post.cu.uRainTime.value = U.uCloudTime.value;
      if (frame % 20 === 0 && weather.settings.wetAuto) { $('#wet').value = weather.wet; $('#wet-o').textContent = `自动 ${weather.wet.toFixed(2)}`; }
    }

    // ---------------- render
    // in the flight deck the eye adapts to the darker cabin: about +0.6 EV
    const exposure = Math.pow(2, settings.exposureEV) * 0.62 * (1 + night * 6) * (1 + 0.5 * weather.rainCam) * (inCockpit ? 1.5 : 1);
    if (inCockpit) {
      cockpit.update({ st, dt, night, sunDir: sd, sunRad: light.sunRad, amb: light.amb, exposure, rain: weather.rainCam,
        wx: st ? { dt: dt * ts, vel: vel3, wind: wind3, fall: weather.fall } : null });
    }
    renderer.setRenderTarget(sceneRT);
    renderer.setClearColor(0x000000, 1);
    renderer.clear(true, true, false);
    renderer.render(scene, camera);
    if (inCockpit) cockpit.render(renderer);
    clouds.render(camera, sceneRT.depthTexture);
    const smokeOn = smoke.render(renderer, camera, sceneRT.depthTexture, light);
    const rainOn = weather.renderParticles(renderer, camera, sceneRT.depthTexture);
    post.render(camera, sceneRT, clouds, exposure, smokeOn ? smoke.texture : null, rainOn && !inCockpit ? weather.rain.texture : null,
      inCockpit ? (r, hdr) => cockpit.renderGlass(r, camera, sceneRT.depthTexture, hdr) : null);

    // ---------------- HUD / panels
    gncViz.update(renderer.domElement.width, renderer.domElement.height);
    let guideDir = null;
    if (st && (inCockpit || settings.showHud)) {
      const ra = gncViz.refAhead(st.pos, 8);
      if (ra) guideDir = enuToThree(ra.point[0], ra.point[1], ra.point[2]).sub(camera.position).normalize();
    }
    hud.drawHud(camera, st ? { ...st, vel3, guideDir, bodyQuat: quat3 } : null, inCockpit || settings.showHud, inCockpit,
      inCockpit ? cockpit.hudClip(camera, hud.cv.width, hud.cv.height) : null);
    if (st) hud.updateGnc(st);
    if (st) {
      hud.updatePanel(st, { range: Math.hypot(st.pos[0], st.pos[1]) });
      if (MOBILE_MQ.matches) for (const [dst, src] of mstrip) if (src && dst.textContent !== src.textContent) dst.textContent = src.textContent;
      if (frame % 3 === 0) {
        hud.drawSurfaces(st.surf, st.cmd);
        hud.drawProfile(st, lastPlan);
        spawnPanel.setVehicle(st);
        if (frame % 15 === 0) spawnPanel.draw();
        hud.drawMap(st.pos, st.air?.track ?? 0, A.world.runway, lastPlan ? { pts: gncViz.planPts, hac: lastPlan.hac, approach: lastPlan.approach, ali: lastPlan.ali, fallback: lastPlan.fallback, prev: lastPlan.prev, mpc: lastMpc } : (lastMpc ? { pts: [], mpc: lastMpc } : null));
      }
    }
    meds.update({
      st, plan: lastPlan, planPts: gncViz.planPts, mpc: lastMpc, trail: hud.trail, prof: hud.profHist, runway: A.world.runway,
      link: link.connected ? 'online' : link.offline ? 'local' : 'offline', night: U.uNight.value, cockpit: inCockpit,
    });
  }
  loop();
  window.__planelands = {
    THREE, renderer, scene, camera, U, terrain, airport, shuttle, clouds, post, atmo, rig, link, setCam, gncViz, hud, meds, chute, smoke, trail, weather, cockpit,
    injectPlan(m) { lastPlan = m; gncViz.setPlan(m); }, injectMpc(p) { lastMpc = p; gncViz.setMpc(p); },
    // debug helper: free camera at runway-frame (u, v, height) looking at heading/pitch (deg)
    look(u, v, h, hdgDeg, pitchDeg) { setCam('free'); airport.toWorld(u, v, h, rig.free.pos); rig.free.yaw = -hdgDeg * Math.PI / 180; rig.free.pitch = pitchDeg * Math.PI / 180; },
    lookEN(e, n, h, hdgDeg, pitchDeg) { setCam('free'); rig.free.pos.copy(enuToThree(e, n, h)); rig.free.yaw = -hdgDeg * Math.PI / 180; rig.free.pitch = pitchDeg * Math.PI / 180; },
    setTime(h) { tod = h; $('#tod').value = h; $('#tod').dispatchEvent(new Event('input')); },
  };
}

main().catch((e) => {
  console.error(e);
  ld(`启动失败：${e.message}`);
  $('#ld-fill').style.background = '#ff6b5a';
});
