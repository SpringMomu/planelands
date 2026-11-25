// Geometry-clipmap terrain: nested square grids centred on the camera, crack-free CDLOD
// morphing, bicubic heightmap sampling with CPU-built mips, analytic far field, water
// surfaces, procedural land-cover shading, cloud + shuttle shadows.
import * as THREE from 'three';
import { COMMON, ATMOSPHERE, CLOUD_COMMON, PLANET_R } from './glsl.js';

const GRID_N = 192;          // quads per level side (multiple of 4)
const BASE_SPACING = 1.0;    // metres, level 0
const MAX_LEVELS = 14;

// ------------------------------------------------------------------ GLSL
const TERRAIN_FUNCS = /* glsl */ `
uniform highp sampler2D uHeight;   // R32F with explicit mips (texelFetch only)
uniform sampler2D uLandcover;      // RGBA8: forest, farm, urban, water depth (dm)
uniform sampler2D uNormalMap;      // RGBA8: n.x, n.z, cavity, 1
uniform float uMapHalf;            // half extent (m)
uniform float uTexel;              // metres per texel at lod 0
uniform float uMapSize;            // texels
uniform vec2 uRwyDir;              // runway along (E,N)
uniform float uRwyElev;

float coastX(float y){ return 14000.0 + 4000.0 * sin(y / 23000.0 + 0.4) + 2500.0 * sin(y / 9100.0 + 1.3); }
float farHeight(vec2 en){
  float x = en.x, y = en.y;
  float d = x - coastX(y);
  float inland = max(-d - 5000.0, 0.0);
  float land = 6.0 + 0.0009 * inland;
  float hills = 140.0 * smoothstep(15000.0, 40000.0, inland) * (0.55 + 0.45 * sin(x / 7000.0) * sin(y / 5300.0));
  float mount = smoothstep(50000.0, 80000.0, inland) * (1500.0 + 700.0 * sin(y / 13000.0 + 0.7) * sin(x / 9000.0 + 1.1));
  float hl = land + hills + mount;
  float hs = max(-8.0 - 0.004 * max(d, 0.0), -150.0);
  return hl + (hs - hl) * smoothstep(-300.0, 600.0, d);
}
// 0 outside the map, 1 inside (hard switch: the map border already equals the far field)
float inMap(vec2 en){ vec2 a = abs(en); return step(max(a.x, a.y), uMapHalf - 1.0); }

// bicubic B-spline height at an integer lod; also returns the bilinear water surface
float sampleHeightLod(vec2 en, int lod, out float waterSurf, out float wetness){
  float sc = exp2(-float(lod));
  vec2 st = (en + uMapHalf) / uTexel * sc - 0.5;
  ivec2 sz = textureSize(uHeight, lod);
  vec2 fi = floor(st); vec2 f = st - fi; ivec2 i0 = ivec2(fi);
  vec2 f2 = f * f, f3 = f2 * f;
  vec2 w0 = (1.0 - 3.0 * f + 3.0 * f2 - f3) / 6.0;
  vec2 w1 = (4.0 - 6.0 * f2 + 3.0 * f3) / 6.0;
  vec2 w2 = (1.0 + 3.0 * f + 3.0 * f2 - 3.0 * f3) / 6.0;
  vec2 w3 = f3 / 6.0;
  float wx[4] = float[4](w0.x, w1.x, w2.x, w3.x);
  float wy[4] = float[4](w0.y, w1.y, w2.y, w3.y);
  float h = 0.0;
  float hc[4] = float[4](0.0, 0.0, 0.0, 0.0);
  for (int y = 0; y < 4; y++){
    for (int x = 0; x < 4; x++){
      ivec2 c = clamp(i0 + ivec2(x - 1, y - 1), ivec2(0), sz - 1);
      float v = texelFetch(uHeight, c, lod).r;
      h += v * wx[x] * wy[y];
      if (x >= 1 && x <= 2 && y >= 1 && y <= 2) hc[(y - 1) * 2 + (x - 1)] = v;
    }
  }
  // water surface: weighted over wet texels of the bilinear footprint
  float ws = 0.0, ww = 0.0, wet = 0.0;
  for (int k = 0; k < 4; k++){
    ivec2 o = ivec2(k & 1, k >> 1);
    ivec2 c = clamp(i0 + o, ivec2(0), sz - 1);
    float wd = texelFetch(uLandcover, c, lod).a * 25.5;
    float bw = (o.x == 1 ? f.x : 1.0 - f.x) * (o.y == 1 ? f.y : 1.0 - f.y);
    float hv = hc[k];
    bool isWet = wd > 0.05 || hv < 0.0;
    float s = wd > 0.05 ? hv + wd : 0.0;
    if (isWet){ ws += s * bw; ww += bw; wet += bw; }
  }
  waterSurf = ww > 0.0 ? ws / ww : -1e5;
  wetness = wet;
  return h;
}
float terrainBase(vec2 en, float lodA, float lodB, float mixB, out float waterSurf, out float wetness){
  if (inMap(en) < 0.5){
    float h = farHeight(en);
    waterSurf = h < 0.0 ? 0.0 : -1e5; wetness = h < 0.0 ? 1.0 : 0.0;
    return h;
  }
  float wsA, wtA;
  float h = sampleHeightLod(en, int(lodA), wsA, wtA);
  waterSurf = wsA; wetness = wtA;
  if (mixB > 0.001 && lodB != lodA){
    float wsB, wtB;
    float hb = sampleHeightLod(en, int(lodB), wsB, wtB);
    if (mixB >= 0.999){ h = hb; waterSurf = wsB; wetness = wtB; }
    else {
      h = mix(h, hb, mixB);
      if (wsB > -1e4 && wsA > -1e4) waterSurf = mix(wsA, wsB, mixB);
      else if (mixB > 0.5) waterSurf = wsB;
      wetness = mix(wtA, wtB, mixB);
    }
  }
  return h;
}
float airportMask(vec2 en){
  vec2 r = vec2(dot(en, uRwyDir), dot(en, vec2(uRwyDir.y, -uRwyDir.x)));
  vec2 q = max(abs(r) - vec2(4300.0, 1500.0), 0.0);
  return 1.0 - smoothstep(0.0, 900.0, length(q));
}
// small-scale displacement (value noise with derivatives). returns (h, dh/dE, dh/dN)
vec3 detailNoise(vec2 en, float amp, float minWave){
  vec3 acc = vec3(0.0);
  float a = amp; float wl = 420.0;
  mat2 m = mat2(0.8, 0.6, -0.6, 0.8);
  vec2 p = en / wl;
  float scale = 1.0 / wl;
  for (int i = 0; i < 6; i++){
    float fade = smoothstep(minWave, minWave * 2.5, wl);
    if (fade <= 0.0) break;
    vec3 n = vnoised(p);
    // ridge-ish shaping: sharper crests
    acc += vec3(n.x, n.yz * scale) * a * fade;
    p = m * p * 2.0 + 3.7; scale *= 2.0; wl *= 0.5; a *= 0.45;
    // keep the derivative in world units: the rotation m mixes axes
  }
  return acc;
}
float detailAmplitude(float h, float apMask){
  return mix(0.7, 16.0, smoothstep(250.0, 1600.0, h)) * (1.0 - apMask) * smoothstep(-1.0, 3.0, h);
}
`;

const VERT = /* glsl */ `
#include <common>
#include <logdepthbuf_pars_vertex>
${COMMON}
${TERRAIN_FUNCS}
uniform vec2 uCenter;
uniform float uSpacing;
uniform float uHalf;
uniform float uLodA;
uniform float uLodB;
out vec3 vWorld;
out vec3 vFlat;      // (E, terrain height, N)
out float vWaterSurf;
out float vWet;
out float vDetailAmp;
void main(){
  vec2 g = position.xz;
  vec2 wp = uCenter + g * uSpacing;            // three XZ
  vec2 dd = abs(wp - uCenter) / uHalf;
  float m = max(dd.x, dd.y);
  float morph = clamp((m - 0.70) / 0.24, 0.0, 1.0);
  vec2 odd = mod(g, 2.0);
  wp -= odd * uSpacing * morph;
  vec2 en = vec2(wp.x, -wp.y);
  float ws, wet;
  float h = terrainBase(en, uLodA, uLodB, morph, ws, wet);
  float ap = airportMask(en);
  float amp = detailAmplitude(h, ap);
  float spacing = uSpacing * (1.0 + morph);
  vec3 dn = detailNoise(en, amp, spacing * 2.0);
  float hT = h + dn.x - 0.3 * ap;   // sink the ground a little under the airport pavements
  float y = max(hT, ws);
  vFlat = vec3(en.x, hT, en.y);
  vWaterSurf = ws;
  vWet = wet;
  vDetailAmp = amp;
  float drop = dot(wp, wp) / (2.0 * R_GROUND);
  vWorld = vec3(wp.x, y - drop, wp.y);
  gl_Position = projectionMatrix * viewMatrix * vec4(vWorld, 1.0);
  #include <logdepthbuf_vertex>
}
`;

const FRAG = /* glsl */ `
#include <common>
#include <packing>
#include <logdepthbuf_pars_fragment>
${COMMON}
${ATMOSPHERE}
${CLOUD_COMMON}
${TERRAIN_FUNCS}
uniform sampler2D uIrrLUT;
uniform sampler2D uSkyLUT;
uniform vec3 uSkyUp; uniform vec3 uSkyEast; uniform vec3 uSkyNorth; uniform float uCamR;
uniform float uTime;
uniform float uNight;
uniform float uWet;           // ground wetness 0..1
uniform float uFlashAmb;      // lightning: sky-wide ambient flash
uniform vec4 uHoleBox;        // xz centre, half size, enabled
uniform float uSpacing;
uniform sampler2D uShadowMap;
uniform mat4 uShadowMatrix;
uniform float uShadowOn;
uniform vec2 uShadowTexel;
in vec3 vWorld;
in vec3 vFlat;
in float vWaterSurf;
in float vWet;
in float vDetailAmp;

vec2 skyUV(vec3 dir){
  float r = max(uCamR, R_GROUND + 1.0);
  float vh = sqrt(r * r - R_GROUND * R_GROUND);
  float beta = acos(clamp(vh / r, -1.0, 1.0));
  float zh = PI - beta;
  float vz = acos(clamp(dot(dir, uSkyUp), -1.0, 1.0));
  float v;
  if (vz < zh){ float c = vz / zh; c = 1.0 - c; c = sqrt(max(c, 0.0)); c = 1.0 - c; v = c * 0.5; }
  else { float c = (vz - zh) / beta; c = sqrt(max(c, 0.0)); v = c * 0.5 + 0.5; }
  float az = atan(dot(dir, uSkyNorth), dot(dir, uSkyEast));
  return vec2(az / (2.0 * PI) + 0.5, v);
}
vec3 irradiance(vec3 n){
  float e = asin(clamp(dot(n, uSkyUp), -1.0, 1.0));
  float az = atan(dot(n, uSkyNorth), dot(n, uSkyEast));
  vec2 uv = vec2(az / (2.0 * PI) + 0.5, e / PI + 0.5);
  return texture(uIrrLUT, uv).rgb * uSunIntensity;
}
float shuttleShadow(vec3 wp, float ndl){
  if (uShadowOn < 0.5) return 1.0;
  vec4 sc = uShadowMatrix * vec4(wp, 1.0);
  vec3 c = sc.xyz / sc.w;
  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;
  float bias = 0.0008;
  float s = 0.0;
  for (int j = -1; j <= 1; j++) for (int i = -1; i <= 1; i++){
    float d = unpackRGBAToDepth(texture(uShadowMap, c.xy + vec2(i, j) * uShadowTexel * 1.5));
    s += step(c.z - bias, d);
  }
  return s / 9.0;
}
// ---------------------------------------------------------------- land cover palette (linear albedo)
vec3 fieldColor(float id){
  vec3 c;
  if (id < 0.20) c = vec3(0.052, 0.082, 0.030);       // green crop
  else if (id < 0.34) c = vec3(0.085, 0.105, 0.044);  // young crop
  else if (id < 0.46) c = vec3(0.185, 0.158, 0.080);  // ripe grain
  else if (id < 0.58) c = vec3(0.100, 0.082, 0.060);  // ploughed soil
  else if (id < 0.70) c = vec3(0.140, 0.128, 0.082);  // stubble / dry grass
  else if (id < 0.82) c = vec3(0.040, 0.064, 0.028);  // dark green / orchard
  else if (id < 0.93) c = vec3(0.072, 0.094, 0.044);  // pasture
  else c = vec3(0.150, 0.135, 0.098);                 // fallow
  return c * (0.88 + 0.24 * hash11(id * 9133.0));
}
// box-filtered coverage of a line of half-width w at signed distance x (filter half-width aa)
float lineCov(float x, float w, float aa){ aa = max(aa, 1e-3); return clamp((min(x + w, aa) - max(x - w, -aa)) / (2.0 * aa), 0.0, 1.0); }
vec3 farmland(vec2 p, float footprint, out float rowMask){
  // parcels are laid out on a straight grid whose orientation and size are constant per region
  vec2 reg = voronoi(p / 4200.0 + vnoise(p / 1500.0) * 0.35);
  float rid = reg.y;
  float ang = (rid - 0.5) * 3.14159;
  mat2 R = mat2(cos(ang), sin(ang), -sin(ang), cos(ang));
  vec2 q = R * p + rid * 977.0;
  vec2 size = vec2(300.0, 210.0) * (0.65 + 0.7 * fract(rid * 13.71));
  vec2 cell = floor(q / size);
  vec2 local = fract(q / size);
  float hsplit = hash12(cell + 3.0);
  float splitAt = 0.35 + 0.3 * hash12(cell + (hsplit < 0.5 ? 7.0 : 9.0));
  float sub = hsplit < 0.5 ? step(splitAt, local.x) : step(splitAt, local.y);
  float id = hash12(cell * 1.7 + sub * 31.0 + 0.5);
  // most parcels of a region carry the region's dominant crops
  float regional = fract(rid * 7.31 + vnoise(p / 9000.0) * 0.5);
  float pick = id < 0.6 ? fract(regional + id * 0.25) : id;
  vec3 col = fieldColor(pick) * (0.9 + 0.2 * hash12(cell + 5.0));
  // far away (parcels a few pixels wide) blend to the regional average to avoid sparkle
  vec3 avg = vec3(0.115, 0.115, 0.058) * (0.9 + 0.2 * rid);
  col = mix(col, avg, smoothstep(size.y * 0.06, size.y * 0.35, footprint));
  // hedgerows / field boundaries, box filtered (no moire)
  vec2 e = min(local, 1.0 - local) * size;
  float edge = min(e.x, e.y);
  float sp = abs((hsplit < 0.5 ? local.x : local.y) - splitAt) * (hsplit < 0.5 ? size.x : size.y);
  float aa = footprint * 0.7;
  float hedge = max(lineCov(edge, 2.2, aa), lineCov(sp, 1.2, aa) * 0.6);
  float hedgeKind = hash12(cell + 11.0) < 0.6 ? 1.0 : 0.35;
  col = mix(col, vec3(0.03, 0.05, 0.022), hedge * hedgeKind * (1.0 - smoothstep(40.0, 120.0, footprint)));
  // plough / crop rows (only close up)
  rowMask = 1.0 - smoothstep(0.3, 1.2, footprint);
  if (rowMask > 0.0){
    float rows = 0.5 + 0.5 * sin((id > 0.5 ? q.x : q.y) * 2.4);
    col *= mix(1.0, 0.82 + 0.3 * rows, rowMask * 0.6);
  }
  return col;
}
vec3 urbanColor(vec2 p, float u, float footprint, out float lightMask){
  vec2 bs = vec2(95.0, 75.0);
  float ang = vnoise(p / 4000.0) * 1.2;
  mat2 R = mat2(cos(ang), sin(ang), -sin(ang), cos(ang));
  vec2 q = R * p;
  vec2 cell = floor(q / bs); vec2 l = fract(q / bs) * bs;
  vec2 e = min(l, bs - l);
  float road = lineCov(min(e.x, e.y), 5.5, footprint * 0.7);
  float h = hash12(cell);
  vec3 roof = h < 0.3 ? vec3(0.16, 0.07, 0.05) : (h < 0.6 ? vec3(0.17, 0.165, 0.16) : (h < 0.8 ? vec3(0.30, 0.29, 0.27) : vec3(0.08, 0.085, 0.09)));
  // building footprints inside blocks
  vec2 bc = floor(l / 22.0); float bh = hash12(cell * 13.0 + bc);
  vec2 bl = fract(l / 22.0);
  float bld = step(0.18, bl.x) * step(bl.x, 0.84) * step(0.18, bl.y) * step(bl.y, 0.84) * step(0.25, bh);
  float detailVis = 1.0 - smoothstep(6.0, 30.0, footprint);
  vec3 ground = mix(vec3(0.06, 0.08, 0.04), vec3(0.11, 0.105, 0.10), u);
  vec3 c = mix(ground, roof * (0.8 + 0.4 * bh), mix(0.55 * u, bld, detailVis));
  vec3 avg = mix(ground, roof, 0.5 * u);
  c = mix(avg, c, detailVis * 0.8 + 0.2);
  c = mix(c, vec3(0.05, 0.05, 0.052), road * (0.5 + 0.5 * u));
  lightMask = road * (0.6 + 0.4 * hash12(cell + 5.0)) + bld * step(0.7, bh) * 0.3;
  return c;
}
void main(){
  #include <logdepthbuf_fragment>
  if (uHoleBox.w > 0.5){
    vec2 dq = abs(vWorld.xz - uHoleBox.xy);
    // margin relative to this level's spacing: the morphed fine-level rim coincides with this
    // level's geometry, so a small overlap is invisible while it removes interpolation cracks
    if (max(dq.x, dq.y) < uHoleBox.z - uSpacing * 0.3) discard;
  }
  vec2 en = vFlat.xz;
  float hT = vFlat.y;
  vec3 V = normalize(cameraPosition - vWorld);
  float dist = length(cameraPosition - vWorld);
  float footprint = max(length(fwidth(en)), 0.02);
  bool inside = inMap(en) > 0.5;
  vec2 uv = (en + uMapHalf) / (2.0 * uMapHalf);

  // ------------------------------------------------ normal
  vec3 n;
  float cavity = 0.5;
  if (inside){
    vec4 nm = texture(uNormalMap, uv);
    vec2 nxz = nm.xy * 2.0 - 1.0;
    n = vec3(nxz.x, sqrt(max(1.0 - dot(nxz, nxz), 0.0)), nxz.y);
    cavity = nm.z;
  } else {
    float e = 60.0;
    float hx = farHeight(en + vec2(e, 0.0)) - farHeight(en - vec2(e, 0.0));
    float hy = farHeight(en + vec2(0.0, e)) - farHeight(en - vec2(0.0, e));
    n = normalize(vec3(-hx / (2.0 * e), 1.0, hy / (2.0 * e)));
  }
  vec3 dn = detailNoise(en, vDetailAmp, footprint * 1.5);
  n = normalize(n + vec3(-dn.y, 0.0, dn.z));
  // bend the normal onto the sphere
  vec3 upC = normalize(vWorld - vec3(0.0, -R_GROUND, 0.0));
  vec3 tX = normalize(cross(upC, vec3(0.0, 0.0, 1.0)));
  vec3 tZ = cross(tX, upC);
  n = normalize(tX * n.x + upC * n.y + tZ * n.z);
  float slope = 1.0 - clamp(dot(n, upC), 0.0, 1.0);

  // ------------------------------------------------ land cover
  vec4 lc = inside ? texture(uLandcover, uv) : vec4(0.0);
  float forest = lc.r, farm = lc.g, urban = lc.b;
  float macro = fbm2(en / 2600.0, 4);
  float macro2 = vnoise(en / 15000.0);
  if (!inside){
    float inl = -(en.x - coastX(en.y));
    farm = smoothstep(0.35, 0.6, macro) * (1.0 - smoothstep(20000.0, 40000.0, inl));
    forest = smoothstep(0.45, 0.7, 1.0 - macro + 0.3 * smoothstep(10000.0, 40000.0, inl)) * (1.0 - farm);
  }
  float ap = airportMask(en);
  vec3 grass = mix(vec3(0.075, 0.098, 0.040), vec3(0.115, 0.118, 0.055), macro2);
  grass = mix(grass, vec3(0.13, 0.12, 0.07), smoothstep(0.55, 0.8, macro) * 0.6);
  // alpine meadow -> brown-green
  grass = mix(grass, vec3(0.11, 0.10, 0.06), smoothstep(1300.0, 2100.0, hT));
  // close-range blade / clump texture
  float nearG = 1.0 - smoothstep(0.08, 0.6, footprint);
  if (nearG > 0.0) grass *= mix(1.0, 0.78 + 0.22 * vnoise(en / 0.35) + 0.2 * vnoise(en / 1.7) - 0.1, nearG);
  vec3 albedo = grass;
  float rowMask;
  if (farm > 0.01) albedo = mix(albedo, farmland(en, footprint, rowMask), farm);
  // forest canopy (crowns visible close up)
  if (forest > 0.01){
    float crowns = vnoise(en / 7.0) * 0.6 + vnoise(en / 19.0) * 0.4;
    float crownVis = 1.0 - smoothstep(1.5, 6.0, footprint);
    vec3 fc = mix(vec3(0.030, 0.052, 0.022), vec3(0.020, 0.036, 0.022), smoothstep(500.0, 1400.0, hT));
    fc = mix(fc, vec3(0.045, 0.06, 0.025), vnoise(en / 420.0) * 0.6);
    fc *= mix(1.0, 0.55 + 0.8 * crowns, crownVis);
    float fmask = smoothstep(0.15, 0.55, forest + (vnoise(en / 60.0) - 0.5) * 0.4);
    albedo = mix(albedo, fc, fmask);
  }
  float lights = 0.0;
  if (urban > 0.02){
    float lm;
    vec3 uc = urbanColor(en, urban, footprint, lm);
    float um = smoothstep(0.05, 0.4, urban);
    albedo = mix(albedo, uc, um);
    lights = lm * um;
  }
  // airport: mowed grass with stripes
  if (ap > 0.0){
    vec2 rr = vec2(dot(en, uRwyDir), dot(en, vec2(uRwyDir.y, -uRwyDir.x)));
    // infield: dry grass / low scrub with patches
    vec3 infield = mix(vec3(0.066, 0.090, 0.038), vec3(0.098, 0.108, 0.052), vnoise(en / 140.0));
    infield = mix(infield, vec3(0.065, 0.080, 0.036), smoothstep(0.55, 0.9, vnoise(en / 260.0 + 3.0)) * 0.35);
    infield *= 0.93 + 0.14 * vnoise(en / 18.0);
    // mowed strip next to the runway and the parallel taxiway, faint stripes
    float nearRwy = 1.0 - smoothstep(140.0, 190.0, abs(rr.y + 60.0)) ;
    nearRwy *= 1.0 - smoothstep(2500.0, 2700.0, abs(rr.x));
    float stripe = step(0.5, fract(rr.y / 14.0)) * (1.0 - smoothstep(1.0, 5.0, footprint));
    vec3 mowed = mix(vec3(0.078, 0.105, 0.040), vec3(0.086, 0.113, 0.044), stripe) * (0.92 + 0.16 * vnoise(en / 70.0));
    vec3 apc = mix(infield, mowed, nearRwy);
    if (nearG > 0.0) apc *= mix(1.0, 0.78 + 0.22 * vnoise(en / 0.35) + 0.2 * vnoise(en / 1.7) - 0.1, nearG);
    albedo = mix(albedo, apc, ap);
  }
  // rock + snow on steep / high ground
  float rockN = vnoise(en / 35.0) * 0.5 + vnoise(en / 140.0) * 0.5;
  float strata = 0.85 + 0.15 * sin(hT * 0.09 + vnoise(en / 300.0) * 6.0);
  vec3 rock = mix(vec3(0.14, 0.125, 0.105), vec3(0.075, 0.07, 0.065), rockN) * strata;
  float rockMask = smoothstep(0.28, 0.48, slope + (rockN - 0.5) * 0.15) * smoothstep(80.0, 400.0, hT + 200.0 * slope);
  rockMask = max(rockMask, smoothstep(2150.0, 2500.0, hT + rockN * 200.0) * 0.8);
  albedo = mix(albedo, rock, rockMask);
  float snowLine = 2350.0 + 300.0 * (macro - 0.5) - 150.0 * (dot(n, vec3(0.0, 0.0, -1.0)) * 0.5 + 0.5);
  float snow = smoothstep(snowLine, snowLine + 180.0, hT) * (1.0 - smoothstep(0.55, 0.75, slope));
  snow = max(snow, smoothstep(snowLine + 400.0, snowLine + 600.0, hT) * 0.55);
  albedo = mix(albedo, vec3(0.78, 0.80, 0.85), snow);

  // beaches / mud flats near water
  float wetBlur = inside ? textureLod(uLandcover, uv, 2.0).a : 0.0;
  float nearSea = smoothstep(4.0, 0.5, hT);
  float beach = nearSea * (1.0 - smoothstep(0.0, 0.25, forest)) * (1.0 - urban) * (1.0 - ap);
  vec3 sand = mix(vec3(0.42, 0.37, 0.27), vec3(0.25, 0.22, 0.17), smoothstep(1.2, 0.2, hT));
  albedo = mix(albedo, sand, beach * smoothstep(-0.5, 1.5, hT + 1.0));
  // far rivers (narrower than the vertex grid) still show up through the land-cover water channel
  float farWater = inside ? smoothstep(0.02, 0.12, texture(uLandcover, uv).a) * smoothstep(40.0, 160.0, footprint * 30.0) : 0.0;

  albedo *= mix(0.75, 1.0, cavity) * (0.92 + 0.16 * macro);

  // ------------------------------------------------ rain: soaked ground and standing water
  float wetRefl = 0.0;
  if (uWet > 0.0){
    float rainHere = uRain > 0.0 ? rainAt(vWorld.xz) / uRain : 0.0;
    float wet = uWet * (0.5 + 0.5 * smoothstep(0.02, 0.35, rainHere)) * (1.0 - snow);
    float flatG = 1.0 - smoothstep(0.035, 0.12, slope);
    float pn = vnoise(en / 6.5) * 0.45 + vnoise(en / 27.0) * 0.35 + vnoise(en / 105.0) * 0.2;
    float thr = 0.70 - 0.2 * wet;
    float pud = wet * flatG * smoothstep(thr, thr + 0.05, pn) * (1.0 - smoothstep(0.25, 0.55, forest)) * (1.0 - urban * 0.5);
    pud = mix(pud, wet * flatG * 0.22, smoothstep(2.0, 14.0, footprint));      // far away: average amount
    albedo *= mix(1.0, 0.55, wet);                                            // soaked soil / grass
    albedo = mix(albedo, vec3(0.018, 0.022, 0.02), pud * 0.85);               // muddy water
    wetRefl = max(pud, wet * 0.05 * (1.0 - forest));
  }

  // ------------------------------------------------ lighting
  vec3 sunCol = uSunIntensity * sunTransmittance(vWorld, uSunDir);
  float cs = cloudShadow(vWorld, uSunDir);
  float ndl = max(dot(n, uSunDir), 0.0);
  float sh = shuttleShadow(vWorld, ndl);
  vec3 amb = irradiance(n) * mix(0.55, 1.0, cavity);
  vec3 color;

  bool water = vWaterSurf > hT - 0.02 && vWaterSurf > -1e4;
  if (water || farWater > 0.5){
    float depth = water ? max(vWaterSurf - hT, 0.0) : 2.0;
    bool sea = hT < 0.0 && vWaterSurf < 0.5;
    // waves: several scrolling noise layers, roughness grows with distance
    vec2 wp = en;
    float t = uTime;
    vec3 w1 = vnoised(wp / 38.0 + vec2(t * 0.05, t * 0.03));
    vec3 w2 = vnoised(wp / 13.0 + vec2(-t * 0.09, t * 0.07));
    vec3 w3 = vnoised(wp / 4.0 + vec2(t * 0.21, -t * 0.17));
    vec3 w4 = vnoised(wp / 160.0 + vec2(t * 0.012, t * 0.02));
    float waveAmp = sea ? 1.0 : 0.35;
    vec2 g = (w1.yz / 38.0 * 1.2 + w2.yz / 13.0 * 0.45 + w3.yz / 4.0 * 0.12 * (1.0 - smoothstep(0.3, 2.0, footprint)) + w4.yz / 160.0 * 4.0) * waveAmp;
    vec3 wn = normalize(upC - tX * g.x * 0.9 + tZ * g.y * 0.9);
    float rough = clamp(0.035 + footprint * 0.004, 0.035, 0.35);
    float NdV = max(dot(wn, V), 1e-3);
    float F = 0.02 + 0.98 * pow(1.0 - NdV, 5.0);
    vec3 R = reflect(-V, wn);
    R.y = abs(R.y) * 0.98 + 0.02;
    vec3 refl = texture(uSkyLUT, skyUV(normalize(R))).rgb * uSunIntensity;
    // GGX sun glint
    vec3 Hh = normalize(uSunDir + V);
    float NdH = max(dot(wn, Hh), 0.0);
    float a2 = rough * rough * rough * rough;
    float dd = NdH * NdH * (a2 - 1.0) + 1.0;
    float D = a2 / (PI * dd * dd);
    float NdL = max(dot(wn, uSunDir), 0.0);
    vec3 spec = sunCol * D * F * NdL / (4.0 * NdV * max(NdL, 1e-3) + 1e-3) * NdL * cs * sh;
    if (!sea) spec *= 1.0 - smoothstep(4.0, 25.0, footprint);   // narrow rivers: avoid stair-stepped glints far away
    vec3 deep = sea ? vec3(0.004, 0.018, 0.035) : vec3(0.010, 0.022, 0.018);
    vec3 shallow = sea ? vec3(0.030, 0.105, 0.105) : vec3(0.040, 0.060, 0.035);
    vec3 body = mix(shallow, deep, smoothstep(0.5, sea ? 18.0 : 6.0, depth));
    // light the water body from sun + sky
    vec3 bodyLit = body * (sunCol * max(uSunDir.y, 0.0) * cs + amb) / PI;
    // visible bottom in very shallow water
    bodyLit = mix(albedo * (sunCol * ndl * cs + amb) / PI, bodyLit, smoothstep(0.0, 1.8, depth));
    color = mix(bodyLit, refl, F) + spec;
    // shore foam
    float foam = (1.0 - smoothstep(0.0, 0.35 + 0.3 * w2.x, depth)) * step(0.5, waveAmp) * smoothstep(0.2, 0.8, vnoise(en / 6.0 + t * 0.3));
    color = mix(color, vec3(0.6) * (sunCol * max(uSunDir.y, 0.0) * cs + amb) / PI, foam * 0.6 * (water ? 1.0 : 0.0));
    if (!water) color = mix(albedo * (sunCol * ndl * cs * sh + amb) / PI, color, farWater * 0.85);
  } else {
    // forest: rough canopy -> slightly darker at grazing light, faint retro-reflection
    float canopy = smoothstep(0.2, 0.6, forest);
    float hot = pow(max(dot(V, uSunDir), 0.0), 8.0) * 0.25 * canopy;
    vec3 direct = sunCol * ndl * cs * sh * (1.0 + hot);
    color = albedo * (direct + amb) / PI;
    // snow specular sparkle
    if (snow > 0.1){
      vec3 Hh = normalize(uSunDir + V);
      color += sunCol * cs * snow * 0.04 * pow(max(dot(n, Hh), 0.0), 60.0);
    }
  }
  // night lights
  if (uNight > 0.0 && lights > 0.0){
    float flick = 0.7 + 0.3 * hash12(floor(en / 9.0));
    color += vec3(1.0, 0.62, 0.30) * lights * flick * uNight * 0.035 * (1.0 + 2.0 * step(0.97, hash12(floor(en / 15.0))));
  }
  // lightning
  if (uFlash.w > 0.0) color += albedo * (flashLightOcc(vWorld) * 0.6 + uFlashCol * uFlashAmb) / PI;
  gl_FragColor = vec4(color, 1.0 + wetRefl);
}
`;

// ------------------------------------------------------------------ CPU side
function buildHeightMips(data, size) {
  const mips = [{ data, width: size, height: size }];
  let cur = data, s = size;
  while (s > 1) {
    const ns = s >> 1;
    const nd = new Float32Array(ns * ns);
    for (let y = 0; y < ns; y++) {
      const r0 = 2 * y * s, r1 = (2 * y + 1) * s;
      for (let x = 0; x < ns; x++) {
        nd[y * ns + x] = 0.25 * (cur[r0 + 2 * x] + cur[r0 + 2 * x + 1] + cur[r1 + 2 * x] + cur[r1 + 2 * x + 1]);
      }
    }
    mips.push({ data: nd, width: ns, height: ns });
    cur = nd; s = ns;
  }
  return mips;
}

function buildLandcoverMips(data, size) {
  // RGB averaged, A (water depth) = max-preserving-ish average so rivers survive at distance
  const mips = [{ data, width: size, height: size }];
  let cur = data, s = size;
  while (s > 1) {
    const ns = s >> 1;
    const nd = new Uint8Array(ns * ns * 4);
    for (let y = 0; y < ns; y++) {
      for (let x = 0; x < ns; x++) {
        const i00 = ((2 * y) * s + 2 * x) * 4, i10 = i00 + 4, i01 = i00 + s * 4, i11 = i01 + 4;
        const o = (y * ns + x) * 4;
        for (let k = 0; k < 3; k++) nd[o + k] = (cur[i00 + k] + cur[i10 + k] + cur[i01 + k] + cur[i11 + k] + 2) >> 2;
        const a0 = cur[i00 + 3], a1 = cur[i10 + 3], a2 = cur[i01 + 3], a3 = cur[i11 + 3];
        nd[o + 3] = Math.max((a0 + a1 + a2 + a3 + 2) >> 2, Math.max(a0, a1, a2, a3) >> 1);
      }
    }
    mips.push({ data: nd, width: ns, height: ns });
    cur = nd; s = ns;
  }
  return mips;
}

function buildNormalMap(h, size, texel) {
  const out = new Uint8Array(size * size * 4);
  const at = (x, y) => h[Math.min(size - 1, Math.max(0, y)) * size + Math.min(size - 1, Math.max(0, x))];
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const dhdx = (at(x + 1, y) - at(x - 1, y)) / (2 * texel);   // d/dEast
      const dhdy = (at(x, y + 1) - at(x, y - 1)) / (2 * texel);   // d/dNorth
      // three normal: (-dh/dE, 1, +dh/dN)
      let nx = -dhdx, ny = 1, nz = dhdy;
      const l = Math.hypot(nx, ny, nz); nx /= l; nz /= l;
      const c = at(x, y);
      const lap = (at(x + 2, y) + at(x - 2, y) + at(x, y + 2) + at(x, y - 2)) * 0.25 - c;
      const cav = Math.max(0, Math.min(1, 0.55 - lap * 0.012));
      const o = (y * size + x) * 4;
      out[o] = Math.round((nx * 0.5 + 0.5) * 255);
      out[o + 1] = Math.round((nz * 0.5 + 0.5) * 255);
      out[o + 2] = Math.round(cav * 255);
      out[o + 3] = 255;
    }
  }
  return out;
}

export class Terrain {
  constructor({ heightData, landcoverData, meta, world, uniforms }) {
    this.meta = meta;
    this.size = meta.size;
    this.extent = meta.extent;
    this.texel = meta.extent / meta.size;
    this.heightData = heightData;
    this.landcover = landcoverData;
    this.group = new THREE.Group();
    this.group.name = 'terrain';

    const size = this.size;
    const hTex = new THREE.DataTexture(heightData, size, size, THREE.RedFormat, THREE.FloatType);
    hTex.mipmaps = buildHeightMips(heightData, size);
    hTex.minFilter = THREE.NearestMipmapNearestFilter;
    hTex.magFilter = THREE.NearestFilter;
    hTex.generateMipmaps = false;
    hTex.needsUpdate = true;
    this.maxLod = hTex.mipmaps.length - 1;

    const lcTex = new THREE.DataTexture(landcoverData, size, size, THREE.RGBAFormat, THREE.UnsignedByteType);
    lcTex.mipmaps = buildLandcoverMips(landcoverData, size);
    lcTex.minFilter = THREE.LinearMipmapLinearFilter;
    lcTex.magFilter = THREE.LinearFilter;
    lcTex.generateMipmaps = false;
    lcTex.anisotropy = 8;
    lcTex.needsUpdate = true;

    const nTex = new THREE.DataTexture(buildNormalMap(heightData, size, this.texel), size, size, THREE.RGBAFormat, THREE.UnsignedByteType);
    nTex.minFilter = THREE.LinearMipmapLinearFilter;
    nTex.magFilter = THREE.LinearFilter;
    nTex.generateMipmaps = true;
    nTex.anisotropy = 8;
    nTex.needsUpdate = true;

    const hdg = THREE.MathUtils.degToRad(world.runway.heading_deg);
    this.shared = {
      uHeight: { value: hTex },
      uLandcover: { value: lcTex },
      uNormalMap: { value: nTex },
      uMapHalf: { value: this.extent / 2 },
      uTexel: { value: this.texel },
      uMapSize: { value: size },
      uRwyDir: { value: new THREE.Vector2(Math.sin(hdg), Math.cos(hdg)) },
      uRwyElev: { value: world.runway.elevation },
    };
    this.shadowUniforms = {
      uShadowMap: { value: null },
      uShadowMatrix: { value: new THREE.Matrix4() },
      uShadowOn: { value: 0 },
      uShadowTexel: { value: new THREE.Vector2(1 / 2048, 1 / 2048) },
    };

    this.geometry = this._grid(GRID_N);
    this.levels = [];
    for (let L = 0; L < MAX_LEVELS; L++) {
      const per = {
        uCenter: { value: new THREE.Vector2() },
        uSpacing: { value: BASE_SPACING * 2 ** L },
        uHalf: { value: BASE_SPACING * 2 ** L * GRID_N / 2 },
        uLodA: { value: 0 },
        uLodB: { value: 0 },
        uHoleBox: { value: new THREE.Vector4(0, 0, 0, 0) },
      };
      const mat = new THREE.ShaderMaterial({
        vertexShader: VERT,
        fragmentShader: FRAG,
        uniforms: { ...uniforms, ...this.shared, ...this.shadowUniforms, ...per },
        extensions: {},
      });
      const mesh = new THREE.Mesh(this.geometry, mat);
      mesh.frustumCulled = false;
      mesh.renderOrder = -10 + L;  // fine levels first (early-z)
      mesh.userData.per = per;
      this.group.add(mesh);
      this.levels.push(mesh);
    }
  }

  _grid(n) {
    const verts = new Float32Array((n + 1) * (n + 1) * 3);
    let k = 0;
    for (let j = 0; j <= n; j++) for (let i = 0; i <= n; i++) { verts[k++] = i - n / 2; verts[k++] = 0; verts[k++] = j - n / 2; }
    const idx = new Uint32Array(n * n * 6);
    k = 0;
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
      const a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
      // alternate the diagonal for a symmetric pattern
      if (((i + j) & 1) === 0) { idx[k++] = a; idx[k++] = c; idx[k++] = b; idx[k++] = b; idx[k++] = c; idx[k++] = d; }
      else { idx[k++] = a; idx[k++] = c; idx[k++] = d; idx[k++] = a; idx[k++] = d; idx[k++] = b; }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    g.setIndex(new THREE.BufferAttribute(idx, 1));
    g.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1e9);
    return g;
  }

  /** CPU bilinear height (flat world, no detail noise). E/N metres. */
  heightAt(e, n) {
    const half = this.extent / 2;
    if (Math.abs(e) >= half - 1 || Math.abs(n) >= half - 1) return farHeightJS(e, n);
    const s = this.size;
    const x = (e + half) / this.texel - 0.5, y = (n + half) / this.texel - 0.5;
    const xi = Math.max(0, Math.min(s - 2, Math.floor(x))), yi = Math.max(0, Math.min(s - 2, Math.floor(y)));
    const fx = Math.min(1, Math.max(0, x - xi)), fy = Math.min(1, Math.max(0, y - yi));
    const h = this.heightData;
    const a = h[yi * s + xi], b = h[yi * s + xi + 1], c = h[(yi + 1) * s + xi], d = h[(yi + 1) * s + xi + 1];
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
  }
  /** land cover RGBA (0..1) at E/N, nearest texel */
  landcoverAt(e, n) {
    const half = this.extent / 2, s = this.size;
    const x = Math.floor((e + half) / this.texel), y = Math.floor((n + half) / this.texel);
    if (x < 0 || y < 0 || x >= s || y >= s) return [0, 0, 0, 0];
    const o = (y * s + x) * 4, l = this.landcover;
    return [l[o] / 255, l[o + 1] / 255, l[o + 2] / 255, l[o + 3] / 10];
  }
  /** surface (terrain or water) height used for camera collision */
  surfaceAt(e, n) {
    const h = this.heightAt(e, n);
    const lc = this.landcoverAt(e, n);
    if (lc[3] > 0.05) return Math.max(h, h + lc[3]);
    return Math.max(h, 0);
  }

  update(camera, shadowInfo) {
    const cam = camera.position;
    const en_e = cam.x, en_n = -cam.z;
    const ground = this.surfaceAt(en_e, en_n);
    const agl = Math.max(1, cam.y + (cam.x * cam.x + cam.z * cam.z) / (2 * PLANET_R) - ground);
    // first level whose box half-size covers ~0.6 x AGL
    let first = 0;
    while (first < MAX_LEVELS - 4 && BASE_SPACING * 2 ** first * GRID_N / 2 < agl * 0.6) first++;
    let prev = null;
    for (let L = 0; L < MAX_LEVELS; L++) {
      const mesh = this.levels[L];
      const per = mesh.userData.per;
      if (L < first) { mesh.visible = false; continue; }
      mesh.visible = true;
      const sp = BASE_SPACING * 2 ** L;
      const snap = 2 * sp;
      per.uCenter.value.set(Math.floor(cam.x / snap) * snap, Math.floor(cam.z / snap) * snap);
      // heightmap lod so that vertex spacing ~ texel spacing
      const lodA = Math.max(0, Math.min(this.maxLod, Math.floor(Math.log2(Math.max(sp / this.texel, 1)))));
      const lodB = Math.max(0, Math.min(this.maxLod, Math.floor(Math.log2(Math.max(2 * sp / this.texel, 1)))));
      per.uLodA.value = lodA; per.uLodB.value = lodB;
      if (prev) {
        const pc = prev.userData.per.uCenter.value;
        per.uHoleBox.value.set(pc.x, pc.y, prev.userData.per.uHalf.value, 1);
      } else per.uHoleBox.value.set(0, 0, 0, 0);
      prev = mesh;
    }
    if (shadowInfo) {
      this.shadowUniforms.uShadowOn.value = shadowInfo.on ? 1 : 0;
      this.shadowUniforms.uShadowMap.value = shadowInfo.map;
      this.shadowUniforms.uShadowMatrix.value = shadowInfo.matrix;
      if (shadowInfo.size) this.shadowUniforms.uShadowTexel.value.set(1 / shadowInfo.size, 1 / shadowInfo.size);
    }
  }
}

export function coastXJS(y) { return 14000 + 4000 * Math.sin(y / 23000 + 0.4) + 2500 * Math.sin(y / 9100 + 1.3); }
function ss(a, b, x) { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); }
export function farHeightJS(x, y) {
  const d = x - coastXJS(y);
  const inland = Math.max(-d - 5000, 0);
  const land = 6 + 0.0009 * inland;
  const hills = 140 * ss(15000, 40000, inland) * (0.55 + 0.45 * Math.sin(x / 7000) * Math.sin(y / 5300));
  const mount = ss(50000, 80000, inland) * (1500 + 700 * Math.sin(y / 13000 + 0.7) * Math.sin(x / 9000 + 1.1));
  const hl = land + hills + mount;
  const hs = Math.max(-8 - 0.004 * Math.max(d, 0), -150);
  return hl + (hs - hl) * ss(-300, 600, d);
}
