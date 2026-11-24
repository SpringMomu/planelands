// Stand-alone orbiter model viewer (web/orbiter.html).  URL parameters fix the view for
// reproducible screenshots: ?az=deg&el=deg&dist=m&tx=&ty=&tz=&fov=&gear=1&sb=&sun=&sunaz=
import * as THREE from 'three';
import { Shuttle } from '../world/shuttle.js';

const Q = new URLSearchParams(location.search);
const num = (k, d) => (Q.has(k) ? +Q.get(k) : d);
const D2R = Math.PI / 180;

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = num('exp', 1.0);
renderer.shadowMap.enabled = !Q.has('noshadow');
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(num('fov', 35), innerWidth / innerHeight, 0.1, 2000);

// sky-gradient environment (reflections / ambient) + background
function gradientEnv() {
  const s = new THREE.Scene();
  const g = new THREE.SphereGeometry(50, 48, 24);
  const m = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    vertexShader: 'varying vec3 vP; void main(){ vP = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
    fragmentShader: `varying vec3 vP; void main(){
      float h = normalize(vP).y;
      vec3 sky = mix(vec3(0.62, 0.74, 0.88), vec3(0.20, 0.38, 0.70), smoothstep(0.0, 0.8, h));
      vec3 gnd = mix(vec3(0.34, 0.33, 0.30), vec3(0.20, 0.21, 0.19), smoothstep(0.0, -0.6, h));
      gl_FragColor = vec4(h > 0.0 ? sky : gnd, 1.0); }`,
  });
  s.add(new THREE.Mesh(g, m));
  const pm = new THREE.PMREMGenerator(renderer);
  return pm.fromScene(s, 0.02).texture;
}
scene.environment = gradientEnv();
scene.background = new THREE.Color(0x8fa9c6);
const bgMode = Q.get('bg');
if (bgMode === 'dark') scene.background = new THREE.Color(0x10161e);

const sun = new THREE.DirectionalLight(0xfff4e6, 3.2);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
const sc = sun.shadow.camera; sc.left = -24; sc.right = 24; sc.top = 24; sc.bottom = -24; sc.near = 1; sc.far = 120;
sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.03;
scene.add(sun); scene.add(sun.target);
scene.add(new THREE.HemisphereLight(0xbfd4ee, 0x3a3630, 0.35));

// ground plane (shadow catcher) when the gear is down
const ground = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshStandardMaterial({ color: 0x55585a, roughness: 0.95 }));
ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true;
scene.add(ground);

const geom = await (await fetch('./data/orbiter_geometry.json', { cache: 'no-cache' })).json();
const shuttle = new Shuttle(geom);
scene.add(shuttle.group);

const $ = (id) => document.getElementById(id);
const ui = { gear: $('gear'), sb: $('sb'), el: $('el'), bf: $('bf'), sun: $('sun') };
ui.gear.checked = num('gear', 0) > 0; ui.sb.value = num('sb', 0); ui.el.value = num('elev', 0); ui.bf.value = num('bf', 0); ui.sun.value = num('sun', 38);
const sunAz = num('sunaz', 140);
function apply() {
  const g = ui.gear.checked ? 1 : 0;
  shuttle.setGear(g, [0, 0, 0], [false, false, false], 0, 0);
  const e = +ui.el.value;
  shuttle.setSurfaces({ elevon_li: e, elevon_lo: e, elevon_ri: e, elevon_ro: e, bodyflap: +ui.bf.value, rudder: 0, speedbrake: +ui.sb.value });
  ground.visible = g > 0 && !Q.has('noground');
  ground.position.y = -geom.gear.nose.z - 0.0;     // wheel contact plane (model Y = -FRD z)
  const se = +ui.sun.value * D2R, sa = sunAz * D2R;
  sun.position.set(Math.cos(se) * Math.cos(sa) * 60, Math.sin(se) * 60, Math.cos(se) * Math.sin(sa) * 60);
}
Object.values(ui).forEach((el) => el.addEventListener('input', apply));
apply();

// orbit camera
let az = num('az', 215) * D2R, el = num('el', 18) * D2R, dist = num('dist', 48);
const target = new THREE.Vector3(num('tx', 2), num('ty', 0.5), num('tz', 0));
function place() {
  camera.position.set(target.x + dist * Math.cos(el) * Math.cos(az), target.y + dist * Math.sin(el), target.z + dist * Math.cos(el) * Math.sin(az));
  camera.lookAt(target);
}
let drag = null;
renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());
renderer.domElement.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, b: e.button }; });
addEventListener('pointerup', () => { drag = null; });
addEventListener('pointermove', (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
  if (drag.b === 2) {
    const r = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 0), u = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 1);
    target.addScaledVector(r, -dx * dist * 0.0015).addScaledVector(u, dy * dist * 0.0015);
  } else { az += dx * 0.006; el = Math.min(1.5, Math.max(-1.5, el + dy * 0.006)); }
});
renderer.domElement.addEventListener('wheel', (e) => { dist = Math.min(400, Math.max(2, dist * Math.exp(e.deltaY * 0.001))); }, { passive: true });
addEventListener('resize', () => { renderer.setSize(innerWidth, innerHeight); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); });
if (Q.has('noui')) $('ui').style.display = 'none';

function frame() {
  place();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
frame();
window.__ready = true;
