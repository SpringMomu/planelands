// Physically based atmosphere (Hillaire 2020 style LUTs) + sun position + IBL environment.
import * as THREE from 'three';
import { COMMON, ATMOSPHERE, SKYVIEW, PLANET_R } from './glsl.js';
import { FullscreenPass, makeRT } from './pass.js';

const TRANS_FS = COMMON + ATMOSPHERE + /* glsl */ `
void main(){
  float xmu = (gl_FragCoord.x - 0.5) / 255.0;
  float xr = (gl_FragCoord.y - 0.5) / 63.0;
  float H = sqrt(R_TOP * R_TOP - R_GROUND * R_GROUND);
  float rho = H * xr;
  float r = sqrt(rho * rho + R_GROUND * R_GROUND);
  float dmin = R_TOP - r, dmax = rho + H;
  float d = dmin + xmu * (dmax - dmin);
  float mu = d <= 0.0 ? 1.0 : (H * H - rho * rho - d * d) / (2.0 * r * d);
  mu = clamp(mu, -1.0, 1.0);
  vec3 od = vec3(0.0);
  for (int i = 0; i < 64; i++){
    float t = (float(i) + 0.5) / 64.0 * d;
    float ri = sqrt(r * r + t * t + 2.0 * r * mu * t);
    vec3 rs; float ms; vec3 ext;
    mediumAt(ri - R_GROUND, rs, ms, ext);
    od += ext * (d / 64.0);
  }
  outColor = vec4(exp(-od), 1.0);
}`;

const MS_FS = COMMON + ATMOSPHERE + /* glsl */ `
void main(){
  float muS = (gl_FragCoord.x - 0.5) / 31.0 * 2.0 - 1.0;
  float h = max((gl_FragCoord.y - 0.5) / 31.0 * (R_TOP - R_GROUND), 10.0);
  vec3 ro = vec3(0.0, h, 0.0);
  vec3 sun = vec3(sqrt(max(1.0 - muS * muS, 0.0)), muS, 0.0);
  vec3 Lsum = vec3(0.0), fsum = vec3(0.0);
  const int ND = 64;
  for (int k = 0; k < ND; k++){
    float z = 1.0 - 2.0 * (float(k) + 0.5) / float(ND);
    float phi = float(k) * 2.39996323;
    float s = sqrt(max(1.0 - z * z, 0.0));
    vec3 rd = vec3(s * cos(phi), z, s * sin(phi));
    vec2 tg = raySphere2(ro, rd, PLANET_C, R_GROUND);
    vec2 ta = raySphere2(ro, rd, PLANET_C, R_TOP);
    bool hitG = tg.x > 0.0;
    float tMax = hitG ? tg.x : ta.y;
    vec3 L = vec3(0.0), f = vec3(0.0), T = vec3(1.0);
    const int NS = 20;
    float dt = tMax / float(NS);
    for (int i = 0; i < NS; i++){
      vec3 p = ro + rd * ((float(i) + 0.5) * dt);
      float r = length(p - PLANET_C);
      vec3 rs; float ms; vec3 ext;
      mediumAt(r - R_GROUND, rs, ms, ext);
      vec3 sc = rs + vec3(ms);
      vec3 Ts = exp(-ext * dt);
      vec3 S = sc * sunTransmittance(p, sun) * (1.0 / (4.0 * PI));
      vec3 inv = 1.0 / max(ext, vec3(1e-12));
      L += T * (S - S * Ts) * inv;
      f += T * (sc - sc * Ts) * inv;
      T *= Ts;
    }
    if (hitG){
      vec3 pg = ro + rd * tMax; vec3 ng = normalize(pg - PLANET_C);
      L += T * sunTransmittance(pg, sun) * max(dot(ng, sun), 0.0) * GROUND_ALBEDO / PI;
    }
    Lsum += L / float(ND);
    fsum += f / float(ND);
  }
  outColor = vec4(Lsum / max(1.0 - fsum, vec3(1e-3)), 1.0);
}`;

const SKY_FS = COMMON + ATMOSPHERE + /* glsl */ `
uniform vec3 uSkyUp; uniform vec3 uSkyEast; uniform vec3 uSkyNorth; uniform float uCamR;
void main(){
  vec2 uv = (gl_FragCoord.xy - 0.5) / vec2(255.0, 143.0);
  float r = max(uCamR, R_GROUND + 2.0);
  float vh = sqrt(r * r - R_GROUND * R_GROUND);
  float beta = acos(clamp(vh / r, -1.0, 1.0));
  float zh = PI - beta;
  float vz;
  if (uv.y < 0.5){ float c = 2.0 * uv.y; c = 1.0 - c; c *= c; c = 1.0 - c; vz = zh * c; }
  else { float c = uv.y * 2.0 - 1.0; c *= c; vz = zh + beta * c; }
  float az = (uv.x - 0.5) * 2.0 * PI;
  vec3 dir = uSkyUp * cos(vz) + sin(vz) * (uSkyEast * cos(az) + uSkyNorth * sin(az));
  vec3 ro = PLANET_C + uSkyUp * r;
  vec2 tg = raySphere2(ro, dir, PLANET_C, R_GROUND);
  vec2 ta = raySphere2(ro, dir, PLANET_C, R_TOP);
  float tMax = tg.x > 0.0 ? tg.x : ta.y;
  vec3 L, T;
  integrateAtmo(ro, dir, max(tMax, 1.0), 40, L, T);
  outColor = vec4(L, 1.0);
}`;

const IRR_FS = COMMON + ATMOSPHERE + SKYVIEW + /* glsl */ `
uniform float uOvercast;
void main(){
  vec2 uv = gl_FragCoord.xy / vec2(32.0, 16.0);
  float az = (uv.x - 0.5) * 2.0 * PI;
  float e = (uv.y - 0.5) * PI;
  vec3 n = uSkyUp * sin(e) + cos(e) * (uSkyEast * cos(az) + uSkyNorth * sin(az));
  float muS = dot(uSkyUp, uSunDir);
  vec3 Tg = sunTransmittance(PLANET_C + uSkyUp * (R_GROUND + 10.0), uSunDir);
  vec3 skyZen = texture(uSkyLUT, skyViewUV(uSkyUp)).rgb;
  vec3 Lground = GROUND_ALBEDO / PI * (Tg * max(muS, 0.0) + skyZen * PI * 0.9);
  float r = max(uCamR, R_GROUND + 2.0);
  float cosH = -sqrt(max(1.0 - (R_GROUND / r) * (R_GROUND / r), 0.0));
  vec3 E = vec3(0.0);
  const int NS = 160;
  for (int k = 0; k < NS; k++){
    float z = 1.0 - 2.0 * (float(k) + 0.5) / float(NS);
    float phi = float(k) * 2.39996323;
    float s = sqrt(max(1.0 - z * z, 0.0));
    vec3 w = uSkyUp * z + s * (uSkyEast * cos(phi) + uSkyNorth * sin(phi));
    float c = dot(n, w);
    if (c <= 0.0) continue;
    vec3 L = texture(uSkyLUT, skyViewUV(w)).rgb;
    if (z < cosH) L += Lground * 0.75;
    else L = mix(L, vec3(luma(L)) * 1.15, uOvercast);
    E += L * c;
  }
  E *= 4.0 * PI / float(NS);
  outColor = vec4(E, 1.0);
}`;

const ENV_VS = /* glsl */ `
varying vec3 vDir;
void main(){ vDir = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
const ENV_FS = COMMON + ATMOSPHERE + SKYVIEW + /* glsl */ `
uniform sampler2D uIrrLUT;
uniform float uEnvCloud;
uniform float uOvercast;
varying vec3 vDir;
void main(){
  vec3 d = normalize(vDir);
  vec3 L = texture(uSkyLUT, skyViewUV(d)).rgb;
  float r = max(uCamR, R_GROUND + 2.0);
  float cosH = -sqrt(max(1.0 - (R_GROUND / r) * (R_GROUND / r), 0.0));
  float up = dot(d, uSkyUp);
  if (up >= cosH) L = mix(L, vec3(luma(L)) * 1.15, uOvercast);
  if (up < cosH){
    // ground / cloud-deck radiance: sky irradiance + direct sun on the surface below
    vec3 Eg = texture(uIrrLUT, vec2(0.5, 1.0 - 0.5 / 16.0)).rgb;
    vec3 pg = PLANET_C + uSkyUp * (R_GROUND + 10.0);
    Eg += sunTransmittance(pg, uSunDir) * max(dot(uSkyUp, uSunDir), 0.0) * (1.0 - 0.5 * uEnvCloud * 0.0);
    vec3 Lg = mix(GROUND_ALBEDO, vec3(0.72), uEnvCloud) / PI * Eg;
    float haze = exp(-(cosH - up) * 6.0);
    L += Lg * (1.0 - haze * 0.7);
  }
  gl_FragColor = vec4(L * uSunIntensity, 1.0);
}`;

// ------------------------------------------------------------------ CPU helpers (mirror the GLSL medium)
function extinction(h, haze) {
  h = Math.max(h, 0);
  const rd = Math.exp(-h / 8000);
  const md = Math.exp(-h / 1200) * (0.6 + 0.4 * haze) + haze * 0.9 * Math.exp(-h / 650);
  const od = Math.max(0, 1 - Math.abs(h - 25000) / 15000);
  return [5.802e-6 * rd + 4.44e-6 * md + 0.65e-6 * od,
    13.558e-6 * rd + 4.44e-6 * md + 1.881e-6 * od,
    33.1e-6 * rd + 4.44e-6 * md + 0.085e-6 * od];
}

/** transmittance from altitude h (m) toward a direction with cos(zenith)=mu */
export function sunTransmittanceCPU(h, mu, haze) {
  const Rg = PLANET_R, Rt = PLANET_R + 100000;
  const r = Rg + Math.max(h, 0);
  const muH = -Math.sqrt(Math.max(0, 1 - (Rg / r) ** 2));
  const vis = Math.min(1, Math.max(0, (mu - (muH - 0.006)) / 0.012));
  if (vis <= 0) return [0, 0, 0];
  const disc = r * r * (mu * mu - 1) + Rt * Rt;
  const d = Math.max(0, -r * mu + Math.sqrt(Math.max(disc, 0)));
  const N = 48;
  const od = [0, 0, 0];
  for (let i = 0; i < N; i++) {
    const t = (i + 0.5) / N * d;
    const ri = Math.sqrt(r * r + t * t + 2 * r * mu * t);
    const e = extinction(ri - Rg, haze);
    od[0] += e[0] * d / N; od[1] += e[1] * d / N; od[2] += e[2] * d / N;
  }
  const s = vis * vis * (3 - 2 * vis);
  return [Math.exp(-od[0]) * s, Math.exp(-od[1]) * s, Math.exp(-od[2]) * s];
}

/** Sun direction in three world coords for a latitude, day of year and local solar time. */
export function sunDirection(latDeg, dayOfYear, hour, out = new THREE.Vector3()) {
  const lat = THREE.MathUtils.degToRad(latDeg);
  const decl = THREE.MathUtils.degToRad(23.44) * Math.sin(2 * Math.PI * (284 + dayOfYear) / 365);
  const H = THREE.MathUtils.degToRad(15 * (hour - 12));
  const sinEl = Math.sin(lat) * Math.sin(decl) + Math.cos(lat) * Math.cos(decl) * Math.cos(H);
  const el = Math.asin(sinEl);
  const az = Math.atan2(-Math.sin(H), Math.tan(decl) * Math.cos(lat) - Math.sin(lat) * Math.cos(H)); // from north, clockwise
  const e = Math.sin(az) * Math.cos(el), n = Math.cos(az) * Math.cos(el), u = Math.sin(el);
  return out.set(e, u, -n).normalize();
}

export class Atmosphere {
  constructor(renderer, uniforms) {
    this.renderer = renderer;
    this.u = uniforms;
    this.transRT = makeRT(256, 64, { minFilter: THREE.LinearFilter });
    this.msRT = makeRT(32, 32);
    this.skyRT = makeRT(256, 144, { wrapS: THREE.RepeatWrapping, wrapT: THREE.ClampToEdgeWrapping });
    this.irrRT = makeRT(32, 16, { wrapS: THREE.RepeatWrapping });
    uniforms.uTransLUT.value = this.transRT.texture;
    uniforms.uMSLUT.value = this.msRT.texture;
    uniforms.uSkyLUT.value = this.skyRT.texture;
    uniforms.uIrrLUT.value = this.irrRT.texture;
    this.transPass = new FullscreenPass(TRANS_FS, uniforms);
    this.msPass = new FullscreenPass(MS_FS, uniforms);
    this.skyPass = new FullscreenPass(SKY_FS, uniforms);
    this.irrPass = new FullscreenPass(IRR_FS, uniforms);
    this._haze = -1;

    // IBL environment
    this.cubeRT = new THREE.WebGLCubeRenderTarget(128, { type: THREE.HalfFloatType, generateMipmaps: false });
    this.cubeCam = new THREE.CubeCamera(1, 1000, this.cubeRT);
    this.envScene = new THREE.Scene();
    const envMat = new THREE.ShaderMaterial({ vertexShader: ENV_VS, fragmentShader: ENV_FS, uniforms, side: THREE.BackSide, depthWrite: false });
    this.envScene.add(new THREE.Mesh(new THREE.SphereGeometry(100, 48, 24), envMat));
    this.pmrem = new THREE.PMREMGenerator(renderer);
    this.envRT = null;
    this.envTexture = null;
    this._envFrame = 0;
    this._lastEnvSun = new THREE.Vector3();
    this._lastEnvAlt = -1e9;
  }

  updateBasis(camPos) {
    const C = new THREE.Vector3(0, -PLANET_R, 0);
    const up = camPos.clone().sub(C);
    const r = up.length();
    up.divideScalar(r);
    const east = new THREE.Vector3().crossVectors(up, new THREE.Vector3(0, 0, 1)).normalize();
    const north = new THREE.Vector3().crossVectors(up, east).normalize();
    this.u.uSkyUp.value.copy(up);
    this.u.uSkyEast.value.copy(east);
    this.u.uSkyNorth.value.copy(north);
    this.u.uCamR.value = r;
    return r - PLANET_R;
  }

  update(camPos, force = false) {
    const r = this.renderer;
    const prevTarget = r.getRenderTarget();
    const alt = this.updateBasis(camPos);
    if (this._haze !== this.u.uHaze.value || force) {
      this._haze = this.u.uHaze.value;
      this.transPass.render(r, this.transRT);
      this.msPass.render(r, this.msRT);
    }
    this.skyPass.render(r, this.skyRT);
    this.irrPass.render(r, this.irrRT);
    // environment map: refresh when the sun or altitude changed noticeably, or periodically
    this._envFrame++;
    const sunMoved = this._lastEnvSun.angleTo(this.u.uSunDir.value) > 0.004;
    const altMoved = Math.abs(alt - this._lastEnvAlt) > Math.max(150, Math.abs(alt) * 0.08);
    if (force || !this.envTexture || sunMoved || altMoved || this._envFrame > 240) {
      this._envFrame = 0;
      this._lastEnvSun.copy(this.u.uSunDir.value);
      this._lastEnvAlt = alt;
      this.cubeCam.update(r, this.envScene);
      const old = this.envRT;
      this.envRT = this.pmrem.fromCubemap(this.cubeRT.texture);
      this.envTexture = this.envRT.texture;
      if (old) old.dispose();
    }
    r.setRenderTarget(prevTarget);
    return alt;
  }
}
