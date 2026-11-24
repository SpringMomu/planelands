// HDR composite (sky, sun, aerial perspective, clouds, cirrus, stars) + bloom + tonemapping.
import * as THREE from 'three';
import { COMMON, ATMOSPHERE, SKYVIEW, CLOUD_COMMON } from './glsl.js';
import { FullscreenPass, makeRT } from './pass.js';

const COMPOSITE_FS = COMMON + ATMOSPHERE + SKYVIEW + /* glsl */ `
uniform sampler2D uScene;
uniform sampler2D uDepth;
uniform sampler2D uCloud;
uniform sampler2D uCloudDepth;
uniform vec2 uCloudRes;
uniform mat4 uInvProj;
uniform mat4 uCamWorld;
uniform vec3 uCamPos;
uniform vec3 uCamFwd;
uniform float uLogFar;
uniform float uTime;
uniform float uCirrus;
uniform float uCirrusAlt;
uniform vec2 uWindOffset;
uniform float uStars;
uniform float uCloudsOn;
uniform float uAPSteps;
uniform sampler2D uSmoke;
uniform float uSmokeOn;
uniform sampler2D uRainLayer;
uniform float uRainOn;
uniform float uCloudBase;
uniform mat4 uViewProj;
uniform float uRainNow;       // precipitation at the camera (ripples on nearby water)
uniform float uRainTime;
uniform float uOvercast;
uniform vec4 uFlash;
uniform vec3 uFlashCol;
vec3 rayDir(vec2 uv){
  vec4 v = uInvProj * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
  return normalize((uCamWorld * vec4(v.xyz / v.w, 0.0)).xyz);
}
float depthToDist(float d, vec3 rd){
  if (d >= 0.999999) return 1e9;
  float w = exp2(d * uLogFar) - 1.0;
  return w / max(dot(rd, uCamFwd), 1e-4);
}
// depth-aware upsample of the half-res cloud buffer
vec4 cloudUpsample(vec2 uv, float dist){
  vec2 st = uv * uCloudRes - 0.5;
  vec2 i = floor(st); vec2 f = st - i;
  vec4 acc = vec4(0.0); float wsum = 0.0;
  for (int k = 0; k < 4; k++){
    vec2 o = vec2(k & 1, k >> 1);
    vec2 tuv = (i + o + 0.5) / uCloudRes;
    float bw = (o.x > 0.5 ? f.x : 1.0 - f.x) * (o.y > 0.5 ? f.y : 1.0 - f.y);
    float hd = texture(uCloudDepth, tuv).g;
    float rel = abs(min(hd, 2e6) - min(dist, 2e6)) / max(min(dist, 2e6), 1.0);
    float w = bw * (1.0 / (1e-3 + rel * 40.0)) + 1e-6;
    acc += texture(uCloud, tuv) * w; wsum += w;
  }
  return acc / wsum;
}
// expanding rain-drop rings on standing water: radial slope field (two cell layers)
vec2 rippleGrad(vec2 x, float t){
  vec2 g = vec2(0.0);
  for (int L = 0; L < 2; L++){
    float sc = L == 0 ? 0.5 : 0.31;
    vec2 q = x / sc + float(L) * 37.1;
    vec2 cell = floor(q);
    for (int j = -1; j <= 1; j++) for (int i = -1; i <= 1; i++){
      vec2 c = cell + vec2(float(i), float(j));
      vec2 rnd = hash22(c);
      float ph = fract(t * (0.9 + 0.5 * rnd.x) + rnd.y);
      vec2 d = q - (c + 0.15 + 0.7 * hash22(c + 7.7));
      float r = length(d);
      float x2 = (r - ph * 1.2) / 0.09;
      float amp = (1.0 - ph) * (1.0 - ph);
      g += d / max(r, 1e-3) * amp * exp(-x2 * x2) * sin(x2 * 2.2);
    }
  }
  return g * 0.35;
}
// mirror reflection on wet pavement / puddles: screen-space march for the scene, sky + clouds otherwise.
// returns (reflected radiance, Fresnel)
vec4 wetReflect(vec3 P, vec3 rd, float dist, bool doSSR){
  vec3 C = vec3(0.0, -R_GROUND, 0.0);
  vec3 up = normalize(P - C);
  vec2 g = rippleGrad(P.xz, uRainTime) * uRainNow * (1.0 - smoothstep(25.0, 90.0, dist));
  vec3 wv = vnoised(P.xz / 3.1 + uRainTime * vec2(0.21, 0.13));
  g += wv.yz / 3.1 * 0.03 * (1.0 - smoothstep(60.0, 300.0, dist));
  vec3 n = normalize(up + vec3(-g.x, 0.0, -g.y));
  vec3 R = reflect(rd, n);
  float ru = dot(R, up);
  if (ru < 0.02) R = normalize(R + up * (0.02 - ru));
  float cosv = clamp(dot(-rd, n), 0.0, 1.0);
  float F = 0.02 + 0.98 * pow(1.0 - cosv, 5.0);
  vec3 hitCol = vec3(0.0);
  float hitW = 0.0;
  float stepLen = max(0.25, dist * 0.004);
  vec3 p = P + up * 0.03;
  vec3 prevP = p;
  for (int i = 0; i < 30; i++){
    if (!doSSR) break;
    prevP = p;
    p += R * stepLen;
    stepLen *= 1.22;
    vec4 cp = uViewProj * vec4(p, 1.0);
    if (cp.w <= 0.1) break;
    vec2 su = cp.xy / cp.w * 0.5 + 0.5;
    if (su.x < 0.0 || su.y < 0.0 || su.x > 1.0 || su.y > 1.0) break;
    float sd = texture(uDepth, su).r;
    if (sd >= 0.999999) continue;
    float diff = cp.w - (exp2(sd * uLogFar) - 1.0);
    if (diff > 0.0 && diff < stepLen * 2.0 + 0.5){
      vec3 a = prevP, b = p;
      for (int k = 0; k < 5; k++){
        vec3 m = (a + b) * 0.5;
        vec4 cm = uViewProj * vec4(m, 1.0);
        vec2 mu = cm.xy / cm.w * 0.5 + 0.5;
        float mw = exp2(texture(uDepth, mu).r * uLogFar) - 1.0;
        if (cm.w > mw) b = m; else a = m;
      }
      vec4 cb = uViewProj * vec4(b, 1.0);
      vec2 hu = cb.xy / cb.w * 0.5 + 0.5;
      vec2 e = min(hu, 1.0 - hu);
      hitW = smoothstep(0.0, 0.06, min(e.x, e.y)) * (1.0 - float(i) / 60.0);
      hitCol = texture(uScene, hu).rgb;
      break;
    }
  }
  vec3 sky = skyRadiance(R);
  sky = mix(sky, vec3(luma(sky)) * 0.5, uOvercast);
  vec4 cs = uViewProj * vec4(uCamPos + R * 1e5, 1.0);
  if (cs.w > 0.0){
    vec2 su = cs.xy / cs.w * 0.5 + 0.5;
    if (su.x > 0.0 && su.y > 0.0 && su.x < 1.0 && su.y < 1.0 && texture(uDepth, su).r >= 0.999999){
      vec4 cl = cloudUpsample(su, 1e9);
      sky = sky * cl.a + cl.rgb;
    }
  }
  return vec4(mix(sky, hitCol, hitW), F);
}
float cirrusDensity(vec2 p){
  vec2 q = p / 9000.0 + uWindOffset / 9000.0 * 1.6;
  vec2 warp = vec2(fbm2(q * 0.7, 4), fbm2(q * 0.7 + 7.3, 4));
  float streak = fbm2(vec2(q.x * 0.35, q.y * 2.2) + warp * 1.8, 6);
  float big = fbm2(q * 0.18 + 3.0, 3);
  return smoothstep(0.48, 0.85, streak * 0.75 + big * 0.55) * uCirrus;
}
vec3 starField(vec3 rd){
  vec3 p = rd * 320.0;
  vec3 c = floor(p);
  float h = hash13(c);
  if (h < 0.992) return vec3(0.0);
  vec3 f = fract(p) - 0.5;
  float s = exp(-dot(f, f) * 60.0) * (h - 0.992) * 125.0;
  return vec3(0.85, 0.9, 1.0) * s * (0.6 + 0.4 * sin(uTime * 3.0 + h * 400.0));
}
void main(){
  vec2 uv = vUv;
  vec3 rd = rayDir(uv);
  float d = texture(uDepth, uv).r;
  float dist = depthToDist(d, rd);
  vec3 col;
  vec3 C = vec3(0.0, -R_GROUND, 0.0);
  float camAlt = length(uCamPos - C) - R_GROUND;
  bool sky = dist > 1e8;
  if (sky){
    col = skyRadiance(rd);
    // sun disc with limb darkening, attenuated by the atmosphere along the view ray
    float cs = dot(rd, uSunDir);
    float sunR = 0.004654;
    float ang = acos(clamp(cs, -1.0, 1.0));
    if (ang < sunR * 1.6){
      float x = clamp(ang / sunR, 0.0, 1.0);
      float limb = 1.0 - 0.6 * (1.0 - sqrt(max(1.0 - x * x, 0.0)));
      float disc = 1.0 - smoothstep(0.92, 1.05, ang / sunR);
      vec3 up = normalize(uCamPos - C);
      vec3 Tv = transmittanceToTop(length(uCamPos - C), dot(up, rd));
      vec2 tg = raySphere2(uCamPos, rd, C, R_GROUND);
      if (tg.x < 0.0) col += Tv * uSunIntensity * 160.0 * limb * disc;
    }
    // stars at night / at high altitude
    float dark = 1.0 - smoothstep(0.0, 0.04, luma(col) / max(uSunIntensity, 1e-3));
    col += starField(rd) * dark * uStars;
    // lightning glow behind / between the clouds
    if (uFlash.w > 0.0){
      vec3 fd = uFlash.xyz - uCamPos;
      float fl = length(fd);
      float ca = max(dot(rd, fd / fl), 0.0);
      col += uFlashCol * uFlash.w * (0.003 * pow(ca, 8.0) + 0.0005) / (1.0 + fl * fl / 4e8);
    }
  } else {
    vec4 sc = texture(uScene, uv);
    col = sc.rgb;
    // wet surfaces write 1 + reflectivity into alpha
    float wr = clamp(sc.a - 1.0, 0.0, 1.0);
    if (wr > 0.02){
      vec4 wrf = wetReflect(uCamPos + rd * dist, rd, dist, wr > 0.15 && dist < 4000.0);
      col = mix(col, wrf.rgb, wrf.a * wr);
    }
    vec3 La, Ta;
    integrateAtmo(uCamPos, rd, dist, int(uAPSteps), La, Ta);
    float ov = clamp((1.0 - sc.a) * 2.0, 0.0, 1.0);          // guidance overlays write alpha 0.5
    col = mix(col * Ta + La * uSunIntensity, col * mix(Ta, vec3(1.0), 0.75) + La * uSunIntensity * 0.25, ov);
  }
  // cirrus layer (thin 2-D shell)
  vec4 cir = vec4(0.0);
  if (uCirrus > 0.0){
    float rC = R_GROUND + uCirrusAlt;
    vec2 tc = raySphere2(uCamPos, rd, C, rC);
    float tcir = camAlt < uCirrusAlt ? tc.y : (tc.x > 0.0 ? tc.x : -1.0);
    if (tcir > 0.0 && tcir < dist && tcir < 400000.0){
      vec3 p = uCamPos + rd * tcir;
      float dens = cirrusDensity(p.xz) * (1.0 - smoothstep(150000.0, 400000.0, tcir));
      if (dens > 0.001){
        vec3 sunC = uSunIntensity * sunTransmittance(p, uSunDir);
        float ph = hg(dot(rd, uSunDir), 0.75) * 0.7 + hg(dot(rd, uSunDir), -0.2) * 0.3;
        vec3 amb = skyRadiance(normalize(p - C)) * 0.6;
        vec3 cc = (sunC * ph * 2.2 + amb) * 0.9;
        vec3 La2, Ta2;
        integrateAtmo(uCamPos, rd, tcir, 6, La2, Ta2);
        cc = cc * Ta2 + La2 * uSunIntensity;
        float a = clamp(dens * 0.75, 0.0, 0.85);
        cir = vec4(cc * a, a);
      }
    }
  }
  bool cirrusInFront = camAlt > uCirrusAlt;
  if (!cirrusInFront) col = col * (1.0 - cir.a) + cir.rgb;
  float clT = 1.0;
  if (uCloudsOn > 0.5){
    vec4 cl = cloudUpsample(uv, dist);
    col = col * cl.a + cl.rgb;
    clT = cl.a;
  }
  // near-ground particle layer (tyre smoke): over the clouds seen from below, under them seen from above
  if (uSmokeOn > 0.5){
    vec4 sm = texture(uSmoke, uv) * (camAlt > uCloudBase ? clT : 1.0);
    col = col * (1.0 - sm.a) + sm.rgb;
  }
  if (cirrusInFront) col = col * (1.0 - cir.a) + cir.rgb;
  // rain right in front of the lens: over everything, also inside a cloud
  if (uRainOn > 0.5){
    vec4 rl = texture(uRainLayer, uv);
    col = col * (1.0 - rl.a) + rl.rgb;
  }
  outColor = vec4(max(col, vec3(0.0)), 1.0);
}`;

const PREFILTER_FS = /* glsl */ `
uniform sampler2D uSrc; uniform vec2 uTexel; uniform float uThreshold; uniform float uExposure;
vec3 safe(vec3 c){ return min(c, vec3(4000.0)); }
void main(){
  vec3 c = vec3(0.0);
  c += safe(texture(uSrc, vUv + uTexel * vec2(-1.0, -1.0)).rgb);
  c += safe(texture(uSrc, vUv + uTexel * vec2( 1.0, -1.0)).rgb);
  c += safe(texture(uSrc, vUv + uTexel * vec2(-1.0,  1.0)).rgb);
  c += safe(texture(uSrc, vUv + uTexel * vec2( 1.0,  1.0)).rgb);
  c *= 0.25 * uExposure;
  float l = max(c.r, max(c.g, c.b));
  float knee = uThreshold * 0.5;
  float soft = clamp(l - uThreshold + knee, 0.0, 2.0 * knee);
  soft = soft * soft / (4.0 * knee + 1e-4);
  float w = max(soft, l - uThreshold) / max(l, 1e-4);
  outColor = vec4(c * w, 1.0);
}`;
const DOWN_FS = /* glsl */ `
uniform sampler2D uSrc; uniform vec2 uTexel;
void main(){
  vec3 c = texture(uSrc, vUv).rgb * 4.0;
  c += texture(uSrc, vUv + uTexel * vec2(-1.0, -1.0)).rgb;
  c += texture(uSrc, vUv + uTexel * vec2( 1.0, -1.0)).rgb;
  c += texture(uSrc, vUv + uTexel * vec2(-1.0,  1.0)).rgb;
  c += texture(uSrc, vUv + uTexel * vec2( 1.0,  1.0)).rgb;
  outColor = vec4(c / 8.0, 1.0);
}`;
const UP_FS = /* glsl */ `
uniform sampler2D uSrc; uniform sampler2D uBase; uniform vec2 uTexel; uniform float uScatter;
void main(){
  vec3 c = vec3(0.0);
  c += texture(uSrc, vUv + uTexel * vec2(-1.0, 0.0)).rgb * 2.0;
  c += texture(uSrc, vUv + uTexel * vec2( 1.0, 0.0)).rgb * 2.0;
  c += texture(uSrc, vUv + uTexel * vec2(0.0, -1.0)).rgb * 2.0;
  c += texture(uSrc, vUv + uTexel * vec2(0.0,  1.0)).rgb * 2.0;
  c += texture(uSrc, vUv + uTexel * vec2(-1.0, -1.0)).rgb;
  c += texture(uSrc, vUv + uTexel * vec2( 1.0, -1.0)).rgb;
  c += texture(uSrc, vUv + uTexel * vec2(-1.0,  1.0)).rgb;
  c += texture(uSrc, vUv + uTexel * vec2( 1.0,  1.0)).rgb;
  c /= 12.0;
  outColor = vec4(texture(uBase, vUv).rgb + c * uScatter, 1.0);
}`;
const FINAL_FS = /* glsl */ `
uniform sampler2D uHDR; uniform sampler2D uBloom;
uniform float uExposure; uniform float uBloomStrength; uniform float uTime; uniform vec2 uRes;
uniform float uSaturation; uniform float uContrast; uniform float uVignette;
// ACES fitted (Stephen Hill)
vec3 RRTAndODTFit(vec3 v){ vec3 a = v * (v + 0.0245786) - 0.000090537; vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081; return a / b; }
vec3 aces(vec3 c){
  const mat3 IN = mat3(0.59719, 0.07600, 0.02840, 0.35458, 0.90834, 0.13383, 0.04823, 0.01566, 0.83777);
  const mat3 OUT = mat3(1.60475, -0.10208, -0.00327, -0.53108, 1.10813, -0.07276, -0.07367, -0.00605, 1.07602);
  c = IN * c; c = RRTAndODTFit(c); c = OUT * c; return clamp(c, 0.0, 1.0);
}
vec3 srgb(vec3 c){ return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(0.0031308, c)); }
float hashf(vec2 p){ return fract(sin(dot(p, vec2(12.9898, 78.233)) + uTime) * 43758.5453); }
void main(){
  vec3 c = texture(uHDR, vUv).rgb * uExposure;
  vec3 b = texture(uBloom, vUv).rgb;
  c += b * uBloomStrength;
  c = aces(c * 1.05);
  float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
  c = mix(vec3(l), c, uSaturation);
  c = clamp((c - 0.5) * uContrast + 0.5, 0.0, 1.0);
  vec2 q = vUv - 0.5;
  c *= 1.0 - uVignette * dot(q, q) * 1.6;
  c = srgb(c);
  c += (hashf(vUv * uRes) - 0.5) / 255.0;
  outColor = vec4(c, 1.0);
}`;

export class Post {
  constructor(renderer, uniforms) {
    this.renderer = renderer;
    this.u = uniforms;
    this.cu = {
      uScene: { value: null }, uDepth: { value: null }, uCloud: { value: null }, uCloudDepth: { value: null },
      uCloudRes: { value: new THREE.Vector2(1, 1) },
      uInvProj: { value: new THREE.Matrix4() }, uCamWorld: { value: new THREE.Matrix4() },
      uCamPos: { value: new THREE.Vector3() }, uCamFwd: { value: new THREE.Vector3() }, uLogFar: { value: 1 },
      uCirrus: { value: 0.6 }, uCirrusAlt: { value: 9800 }, uStars: { value: 1 }, uCloudsOn: { value: 1 },
      uAPSteps: { value: 14 }, uSmoke: { value: null }, uSmokeOn: { value: 0 }, uRainLayer: { value: null }, uRainOn: { value: 0 },
      uViewProj: { value: new THREE.Matrix4() }, uRainNow: { value: 0 }, uRainTime: { value: 0 },
    };
    this.composite = new FullscreenPass(COMPOSITE_FS, { ...uniforms, ...this.cu });
    this.bloomU = {
      uSrc: { value: null }, uBase: { value: null }, uTexel: { value: new THREE.Vector2() },
      uThreshold: { value: 2.2 }, uExposure: { value: 1 }, uScatter: { value: 0.85 },
    };
    this.prefilter = new FullscreenPass(PREFILTER_FS, this.bloomU);
    this.down = new FullscreenPass(DOWN_FS, this.bloomU);
    this.up = new FullscreenPass(UP_FS, this.bloomU);
    this.fu = {
      uHDR: { value: null }, uBloom: { value: null }, uExposure: { value: 1 }, uBloomStrength: { value: 0.06 },
      uTime: { value: 0 }, uRes: { value: new THREE.Vector2(1, 1) }, uSaturation: { value: 1.08 },
      uContrast: { value: 1.04 }, uVignette: { value: 0.22 },
    };
    this.final = new FullscreenPass(FINAL_FS, this.fu);
    this.hdr = null;
    this.mips = [];
    this.ups = [];
  }

  setSize(w, h) {
    this.hdr?.dispose();
    this.hdr = makeRT(w, h);
    for (const m of this.mips) m.dispose();
    for (const m of this.ups) m.dispose();
    this.mips = []; this.ups = [];
    let mw = Math.max(1, w >> 1), mh = Math.max(1, h >> 1);
    for (let i = 0; i < 7; i++) {
      this.mips.push(makeRT(mw, mh));
      this.ups.push(makeRT(mw, mh));
      mw = Math.max(1, mw >> 1); mh = Math.max(1, mh >> 1);
    }
    this.fu.uRes.value.set(w, h);
  }

  /** overlay(renderer, hdrTarget): extra HDR pass after the composite, before bloom (cockpit glass) */
  render(camera, sceneRT, clouds, exposure, smoke = null, rain = null, overlay = null) {
    const r = this.renderer;
    const cu = this.cu;
    cu.uSmoke.value = smoke;
    cu.uSmokeOn.value = smoke ? 1 : 0;
    cu.uRainLayer.value = rain;
    cu.uRainOn.value = rain ? 1 : 0;
    cu.uScene.value = sceneRT.texture;
    cu.uDepth.value = sceneRT.depthTexture;
    cu.uCloud.value = clouds.texture;
    cu.uCloudDepth.value = clouds.depthTexture;
    cu.uCloudRes.value.set(clouds.rtMarch.width, clouds.rtMarch.height);
    cu.uInvProj.value.copy(camera.projectionMatrixInverse);
    cu.uCamWorld.value.copy(camera.matrixWorld);
    cu.uCamPos.value.copy(camera.position);
    camera.getWorldDirection(cu.uCamFwd.value);
    cu.uLogFar.value = Math.log2(camera.far + 1.0);
    cu.uViewProj.value.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    this.composite.render(r, this.hdr);
    if (overlay) overlay(r, this.hdr);

    // bloom
    const b = this.bloomU;
    b.uExposure.value = exposure;
    b.uSrc.value = this.hdr.texture;
    b.uTexel.value.set(1 / this.hdr.width, 1 / this.hdr.height);
    this.prefilter.render(r, this.mips[0]);
    for (let i = 1; i < this.mips.length; i++) {
      b.uSrc.value = this.mips[i - 1].texture;
      b.uTexel.value.set(1 / this.mips[i - 1].width, 1 / this.mips[i - 1].height);
      this.down.render(r, this.mips[i]);
    }
    let src = this.mips[this.mips.length - 1];
    for (let i = this.mips.length - 2; i >= 0; i--) {
      b.uSrc.value = src.texture;
      b.uBase.value = this.mips[i].texture;
      b.uTexel.value.set(1 / src.width, 1 / src.height);
      this.up.render(r, this.ups[i]);
      src = this.ups[i];
    }
    const f = this.fu;
    f.uHDR.value = this.hdr.texture;
    f.uBloom.value = this.ups[0].texture;
    f.uExposure.value = exposure;
    f.uTime.value = (f.uTime.value + 0.618) % 100;
    this.final.render(r, null);
  }
}
