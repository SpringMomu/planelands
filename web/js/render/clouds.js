// Volumetric clouds: half-res ray march through a spherical shell + temporal reprojection.
import * as THREE from 'three';
import { COMMON, ATMOSPHERE, CLOUD_COMMON } from './glsl.js';
import { FullscreenPass, makeRT } from './pass.js';

const RAY = /* glsl */ `
uniform mat4 uInvProj;
uniform mat4 uCamWorld;
uniform vec3 uCamPos;
uniform vec3 uCamFwd;
uniform float uLogFar;
uniform sampler2D uDepth;
uniform vec2 uFullRes;
vec3 rayDir(vec2 uv){
  vec4 v = uInvProj * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  return normalize((uCamWorld * vec4(v.xyz / v.w, 0.0)).xyz);
}
float depthToDist(float d, vec3 rd){
  if (d >= 0.999999) return 1e9;
  float w = exp2(d * uLogFar) - 1.0;
  return w / max(dot(rd, uCamFwd), 1e-4);
}
`;

const MARCH_FS = COMMON + ATMOSPHERE + CLOUD_COMMON + RAY + /* glsl */ `
uniform sampler2D uIrrLUT;
uniform float uFrame;
uniform float uSteps;
uniform float uLightSteps;
uniform float uMaxDist;
uniform float uCloudExt;
uniform float uAmbient;
uniform float uSilver;

// rain-shaft medium below the cloud base: broad curtains with fine vertical streaks falling with the rain
float rainDensity(vec3 p, float h){
  vec2 xz = p.xz - uRainSlant * (uCloudBase - h);
  float P = rainAt(xz);
  if (P <= 0.003) return 0.0;
  float big = textureLod(uShapeTex, vec3(xz.x / 3400.0, h / 16000.0 + uCloudTime * 0.0009, xz.y / 3400.0), 0.0).g;
  float fine = textureLod(uDetailTex, vec3(xz.x / 560.0, h / 7000.0 + uCloudTime * 0.014, xz.y / 560.0), 0.0).r;
  float m = smoothstep(0.18, 0.8, big * 0.7 + fine * 0.45 + P * 0.3);
  float prof = 0.6 + 0.4 * smoothstep(0.0, uCloudBase, h);
  return P * m * prof;
}

void main(){
  vec2 uv = vUv;
  vec3 ro = uCamPos;
  vec3 rd = rayDir(uv);
  ivec2 fp = ivec2(gl_FragCoord.xy) * 2;
  float d = texelFetch(uDepth, fp, 0).r;
  float sceneDist = depthToDist(d, rd);

  vec3 C = vec3(0.0, -R_GROUND, 0.0);
  float rBot = R_GROUND + uCloudBase, rTop = rBot + uCloudThick;
  float camR = length(ro - C);
  vec2 tT = raySphere2(ro, rd, C, rTop);
  vec2 tB = raySphere2(ro, rd, C, rBot);
  vec2 tG = raySphere2(ro, rd, C, R_GROUND);
  // per-pixel jitter: interleaved gradient noise shifted by the golden ratio every frame, so the
  // temporal filter sees a well distributed sequence (plain IGN repeats in rows -> horizontal bands)
  float jitter = fract(ign(gl_FragCoord.xy, 0.0) + mod(uFrame, 512.0) * 0.61803399);
  float cosT = dot(rd, uSunDir);
  vec3 irrUp = texture(uIrrLUT, vec2(0.5, 1.0 - 0.5 / 16.0)).rgb * uSunIntensity;
  vec3 irrDn = texture(uIrrLUT, vec2(0.5, 0.5 / 16.0)).rgb * uSunIntensity;

  // ---------------------------------------------------------------- cloud layer
  vec3 cCol = vec3(0.0);
  float cT = 1.0;
  float dAcc = 0.0, wAcc = 0.0;
  float t0 = -1.0, t1 = -1.0;
  if (camR < rBot){ t0 = tB.y; t1 = tT.y; }
  else if (camR < rTop){ t0 = 0.0; t1 = tB.x > 0.0 ? tB.x : tT.y; }
  else if (tT.y > 0.0 && tT.x > 0.0){ t0 = tT.x; t1 = tB.x > 0.0 ? tB.x : tT.y; }
  if (tG.x > 0.0) t1 = min(t1, tG.x);
  t1 = min(t1, min(sceneDist, uMaxDist));
  if (t1 > t0 && t0 >= 0.0){
    int N = int(uSteps);
    float seg = t1 - t0;
    // step length: segment / N, but bounded by a distance-dependent maximum so long grazing rays
    // through a thick layer do not slice the clouds into visible layers
    float baseStep = seg / float(N);
    float lightStep = uCloudThick * 0.045;
    float t = t0 + clamp(baseStep, 25.0, 140.0) * jitter;
    int zeroRun = 0;
    bool backed = false;
    int budget = N * 2 + 32;
    for (int i = 0; i < 448; i++){
      if (i >= budget || t > t1 || cT < 0.008) break;
      float st = clamp(baseStep, 25.0, mix(140.0, 900.0, smoothstep(3000.0, 80000.0, t)));
      vec3 p = ro + rd * t;
      float hf = cloudHeightFrac(p);
      float dens = cloudDensity(p, hf, false);
      if (dens > 0.0 && zeroRun >= 3 && !backed){
        // came in with coarse steps: back up and enter the cloud with fine steps
        t = max(t - st * 2.0, t0); zeroRun = 0; backed = true; continue;
      }
      bool near = t < 45000.0;
      if (dens > 0.0){
        if (near) dens = cloudDensity(p, hf, true);
        dens *= 1.0 - smoothstep(uMaxDist * 0.55, uMaxDist, t);
      }
      if (dens > 0.002){
        float sigma = dens * uCloudExt;
        float od = 0.0; float ls = lightStep; float lt = ls * 0.5;
        for (int j = 0; j < 8; j++){
          if (float(j) >= uLightSteps) break;
          vec3 q = p + uSunDir * lt;
          od += cloudDensity(q, cloudHeightFrac(q), j < 2 && near) * ls;
          ls *= 1.6; lt += ls;
        }
        od *= uCloudExt;
        vec3 sunCol = uSunIntensity * sunTransmittance(p, uSunDir);
        float sc = 0.0;
        float a = 1.0, b = 1.0, c = 1.0;
        for (int k = 0; k < 4; k++){
          float ph = mix(hg(cosT, 0.82 * c), hg(cosT, -0.22 * c), 0.28);
          ph += uSilver * hg(cosT, 0.97) * 0.2 * c;
          sc += a * exp(-od * b) * ph;
          a *= 0.55; b *= 0.35; c *= 0.6;
        }
        float powder = 1.0 - 0.75 * exp(-sigma * 120.0);
        vec3 Ls = sunCol * sc * powder;
        vec3 amb = (irrUp * (0.25 + 0.75 * smoothstep(0.0, 1.0, hf)) + irrDn * 0.35 * (1.0 - hf)) * (uAmbient / (2.0 * PI));
        // lightning inside / near the cloud lights it from within (scattered, softened by the cloud depth)
        vec3 Lf = flashLight(p) * (0.35 + 0.65 * exp(-od * 0.25));
        vec3 S = (Ls + amb + Lf) * sigma;
        float Ts = exp(-sigma * st);
        cCol += cT * (S - S * Ts) / sigma;
        float dT = cT * (1.0 - Ts);
        dAcc += dT * t; wAcc += dT;
        cT *= Ts;
        zeroRun = 0; backed = false;
        t += st;
      } else {
        zeroRun++;
        t += zeroRun > 3 ? st * 3.0 : st;
      }
    }
    if (wAcc > 1e-4){
      vec3 La, Ta;
      integrateAtmo(ro, rd, dAcc / wAcc, 10, La, Ta);
      cCol = cCol * Ta + La * uSunIntensity * (1.0 - cT);
    }
  }

  // ---------------------------------------------------------------- rain shafts below the base
  vec3 rCol = vec3(0.0);
  float rT = 1.0;
  float rdAcc = 0.0, rwAcc = 0.0;
  if (uRain > 0.0){
    float r0 = -1.0, r1 = -1.0;
    if (camR < rBot){ r0 = 0.0; r1 = tB.y > 0.0 ? tB.y : 1e9; }
    else if (tB.x > 0.0){ r0 = tB.x; r1 = tB.y; }
    if (tG.x > 0.0) r1 = min(r1, tG.x);
    r1 = min(r1, min(sceneDist, 70000.0));
    if (r1 > r0 && r0 >= 0.0){
      const int NR = 26;
      float L = r1 - r0;
      vec3 ambR = (irrUp * 0.6 + irrDn * 0.3) * (uAmbient / (2.0 * PI));
      float phR = hg(cosT, 0.55) * 0.6 + 0.4 / (4.0 * PI);
      for (int i = 0; i < NR; i++){
        float u = (float(i) + jitter) / float(NR);
        float tt = r0 + L * u * u;                                  // denser sampling near the start
        float dt = L * (2.0 * float(i) + 1.0) / float(NR * NR);
        vec3 p = ro + rd * tt;
        float h = length(p - C) - R_GROUND;
        if (h > uCloudBase) continue;
        float dens = rainDensity(p, h);
        if (dens <= 0.001) continue;
        float sigma = dens * 0.0034;
        // sunlight through the cell overhead / beside the shaft; the deck above also starves the ambient
        vec3 q = p + uSunDir * max(uCloudBase - h, 0.0) / max(uSunDir.y, 0.15);
        float covS = cloudCoverageAt(sampleWeather(q));
        float covU = cloudCoverageAt(sampleWeather(p));
        vec3 sunC = uSunIntensity * sunTransmittance(p, uSunDir) * exp(-covS * 5.0 * uCloudDensity) * phR;
        vec3 S = (sunC + ambR * mix(1.0, 0.28, covU) + flashLight(p) * 0.45) * vec3(0.86, 0.9, 0.97) * sigma;
        float Ts = exp(-sigma * dt);
        rCol += rT * (S - S * Ts) / sigma;
        float dT = rT * (1.0 - Ts);
        rdAcc += dT * tt; rwAcc += dT;
        rT *= Ts;
        if (rT < 0.01) break;
      }
      if (rwAcc > 1e-4){
        vec3 La, Ta;
        integrateAtmo(ro, rd, rdAcc / rwAcc, 6, La, Ta);
        rCol = rCol * Ta + La * uSunIntensity * (1.0 - rT);
      }
    }
  }

  // rain is in front of the clouds when looking from below the base, behind them from above
  vec3 col = camR < rBot ? rCol + rT * cCol : cCol + cT * rCol;
  float T = cT * rT;
  float wA = wAcc + rwAcc;
  float dMean = wA > 1e-4 ? (dAcc + rdAcc) / wA : 1e7;
  out0 = vec4(col, T);
  out1 = vec4(dMean, sceneDist, 0.0, 1.0);
}`;

const RESOLVE_FS = RAY + /* glsl */ `
uniform sampler2D uCur;
uniform sampler2D uCurDepth;
uniform sampler2D uHist;
uniform mat4 uPrevViewProj;
uniform float uHistValid;
uniform vec2 uHalfRes;
uniform float uHistW;
void main(){
  vec2 uv = vUv;
  vec4 cur = texture(uCur, uv);
  float dMean = texture(uCurDepth, uv).r;
  // neighbourhood statistics (variance clipping)
  vec4 m1 = vec4(0.0), m2 = vec4(0.0);
  for (int j = -1; j <= 1; j++) for (int i = -1; i <= 1; i++){
    vec4 s = texture(uCur, uv + vec2(i, j) / uHalfRes);
    m1 += s; m2 += s * s;
  }
  m1 /= 9.0; m2 /= 9.0;
  vec4 sigma = sqrt(max(m2 - m1 * m1, 0.0));
  vec4 lo = m1 - sigma * 1.5, hi = m1 + sigma * 1.5;
  vec3 rd = rayDir(uv);
  float dist = dMean < 1e6 ? dMean : 60000.0;
  vec3 wp = uCamPos + rd * dist;
  vec4 pc = uPrevViewProj * vec4(wp, 1.0);
  vec2 puv = pc.xy / pc.w * 0.5 + 0.5;
  vec4 outc = cur;
  if (uHistValid > 0.5 && pc.w > 0.0 && all(greaterThanEqual(puv, vec2(0.0))) && all(lessThanEqual(puv, vec2(1.0)))){
    vec4 h = texture(uHist, puv);
    h = clamp(h, lo, hi);
    outc = mix(cur, h, uHistW);
  }
  out0 = outc;
}`;

export class Clouds {
  constructor(renderer, uniforms, textures) {
    this.renderer = renderer;
    this.u = uniforms;
    this.scale = 0.5;
    this.frame = 0;
    this.prevViewProj = new THREE.Matrix4();
    this.histValid = false;
    const cu = {
      uInvProj: { value: new THREE.Matrix4() },
      uCamWorld: { value: new THREE.Matrix4() },
      uCamPos: { value: new THREE.Vector3() },
      uCamFwd: { value: new THREE.Vector3() },
      uLogFar: { value: 1 },
      uDepth: { value: null },
      uFullRes: { value: new THREE.Vector2(1, 1) },
      uFrame: { value: 0 },
      uSteps: { value: 96 },
      uLightSteps: { value: 6 },
      uMaxDist: { value: 140000 },
      uCloudExt: { value: 0.045 },
      uAmbient: { value: 1.0 },
      uSilver: { value: 1.0 },
    };
    this.cu = cu;
    this.march = new FullscreenPass(MARCH_FS, { ...uniforms, ...cu }, { outputs: 2 });
    this.resolveU = {
      uCur: { value: null }, uCurDepth: { value: null }, uHist: { value: null },
      uPrevViewProj: { value: this.prevViewProj }, uHistValid: { value: 0 },
      uHalfRes: { value: new THREE.Vector2(1, 1) }, uHistW: { value: 0.9 },
    };
    this.resolve = new FullscreenPass(RESOLVE_FS, { ...cu, ...this.resolveU }, { outputs: 1 });
    // resolve pass writes out0 (single output declared via outputs:1 => outColor); patch name
    this.resolve.material.fragmentShader = this.resolve.material.fragmentShader.replace('out0 = outc;', 'outColor = outc;');
    this.rtMarch = null;
    this.hist = [null, null];
    this.cur = 0;
  }

  setSize(w, h) {
    const hw = Math.max(2, Math.floor(w * this.scale)), hh = Math.max(2, Math.floor(h * this.scale));
    if (this.rtMarch && this.rtMarch.width === hw && this.rtMarch.height === hh) return;
    this.rtMarch?.dispose(); this.hist[0]?.dispose(); this.hist[1]?.dispose();
    this.rtMarch = new THREE.WebGLRenderTarget(hw, hh, { count: 2, type: THREE.FloatType, depthBuffer: false, minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter });
    for (const t of this.rtMarch.textures) { t.minFilter = THREE.NearestFilter; t.magFilter = THREE.NearestFilter; }
    this.hist = [makeRT(hw, hh), makeRT(hw, hh)];
    this.histValid = false;
    this.resolveU.uHalfRes.value.set(hw, hh);
    this.cu.uFullRes.value.set(w, h);
  }

  get texture() { return this.hist[this.cur].texture; }
  get depthTexture() { return this.rtMarch.textures[1]; }

  render(camera, depthTexture) {
    const r = this.renderer;
    const cu = this.cu;
    cu.uInvProj.value.copy(camera.projectionMatrixInverse);
    cu.uCamWorld.value.copy(camera.matrixWorld);
    cu.uCamPos.value.copy(camera.position);
    camera.getWorldDirection(cu.uCamFwd.value);
    cu.uLogFar.value = Math.log2(camera.far + 1.0);
    cu.uDepth.value = depthTexture;
    cu.uFrame.value = this.frame++;
    this.march.render(r, this.rtMarch);

    const prev = this.cur, next = 1 - this.cur;
    this.resolveU.uCur.value = this.rtMarch.textures[0];
    this.resolveU.uCurDepth.value = this.rtMarch.textures[1];
    this.resolveU.uHist.value = this.hist[prev].texture;
    this.resolveU.uHistValid.value = this.histValid ? 1 : 0;
    this.resolve.render(r, this.hist[next]);
    this.cur = next;
    this.histValid = true;
    this.prevViewProj.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
  }

  invalidate() { this.histValid = false; }
}
