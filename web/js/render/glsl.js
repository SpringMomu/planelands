// Shared GLSL chunks.  World space = three.js (X=East, Y=Up, Z=-North), metres.
// The planet is a sphere of radius R centred at (0,-R,0); flat-world positions are
// bent onto it with y' = y - (x^2+z^2)/(2R) (see frames.js / terrain.js).

export const PLANET_R = 6371000.0;
export const ATMO_TOP = 100000.0;

export const COMMON = /* glsl */ `
#ifndef PI
#define PI 3.14159265359
#endif
#define R_GROUND 6371000.0
#define R_TOP 6471000.0
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
float remap(float v, float lo, float hi, float nlo, float nhi){ return nlo + (v - lo) * (nhi - nlo) / (hi - lo); }
float remapc(float v, float lo, float hi, float nlo, float nhi){ return nlo + clamp((v - lo) / (hi - lo), 0.0, 1.0) * (nhi - nlo); }
float luma(vec3 c){ return dot(c, vec3(0.2126, 0.7152, 0.0722)); }

// integer hashes (identical on every GPU)
uint hashU(uint x){ x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16; return x; }
float hash12(vec2 p){ ivec2 i = ivec2(floor(p)); uint h = hashU(uint(i.x) * 1597334677u ^ hashU(uint(i.y) + 0x9e3779b9u)); return float(h) * (1.0 / 4294967295.0); }
float hash11(float p){ return float(hashU(uint(int(floor(p))) + 374761393u)) * (1.0 / 4294967295.0); }
vec2 hash22(vec2 p){ ivec2 i = ivec2(floor(p)); uint h = hashU(uint(i.x) * 1597334677u ^ hashU(uint(i.y) + 0x9e3779b9u)); uint h2 = hashU(h ^ 0x68bc21ebu); return vec2(float(h), float(h2)) * (1.0 / 4294967295.0); }
float hash13(vec3 p){ ivec3 i = ivec3(floor(p)); uint h = hashU(uint(i.x) * 1597334677u ^ hashU(uint(i.y) * 3812015801u ^ hashU(uint(i.z) + 0x9e3779b9u))); return float(h) * (1.0 / 4294967295.0); }

// value noise with analytic derivatives: returns (value in [-1,1], d/dx, d/dy)
vec3 vnoised(vec2 x){
  vec2 p = floor(x); vec2 w = fract(x);
  vec2 u = w*w*w*(w*(w*6.0-15.0)+10.0);
  vec2 du = 30.0*w*w*(w*(w-2.0)+1.0);
  float a = hash12(p), b = hash12(p+vec2(1,0)), c = hash12(p+vec2(0,1)), d = hash12(p+vec2(1,1));
  float k0 = a, k1 = b-a, k2 = c-a, k4 = a-b-c+d;
  return vec3(-1.0 + 2.0*(k0 + k1*u.x + k2*u.y + k4*u.x*u.y), 2.0*du*vec2(k1 + k4*u.y, k2 + k4*u.x));
}
float vnoise(vec2 x){
  vec2 p = floor(x); vec2 w = fract(x);
  vec2 u = w*w*(3.0-2.0*w);
  float a = hash12(p), b = hash12(p+vec2(1,0)), c = hash12(p+vec2(0,1)), d = hash12(p+vec2(1,1));
  return mix(mix(a,b,u.x), mix(c,d,u.x), u.y);
}
float fbm2(vec2 p, int oct){
  float s = 0.0, a = 0.5;
  mat2 m = mat2(0.8, 0.6, -0.6, 0.8);
  for (int i = 0; i < 8; i++){ if (i >= oct) break; s += a * vnoise(p); p = m * p * 2.03 + 7.1; a *= 0.5; }
  return s;
}
// Voronoi: returns (F1 distance, cell id hash)
vec2 voronoi(vec2 x){
  vec2 n = floor(x), f = fract(x);
  float md = 8.0; float id = 0.0;
  for (int j=-1;j<=1;j++) for (int i=-1;i<=1;i++){
    vec2 g = vec2(float(i), float(j));
    vec2 o = hash22(n + g);
    vec2 r = g + o - f; float d = dot(r, r);
    if (d < md){ md = d; id = hash12(n + g + 17.0); }
  }
  return vec2(sqrt(md), id);
}
// interleaved gradient noise (for dithering / ray jitter)
float ign(vec2 px, float frame){ px += 5.588238 * frame; return fract(52.9829189 * fract(0.06711056 * px.x + 0.00583715 * px.y)); }
`;

// ----------------------------------------------------------------------------------- atmosphere
// Hillaire 2020 style: transmittance LUT, multiple-scattering LUT, sky-view LUT.
export const ATMOSPHERE = /* glsl */ `
uniform sampler2D uTransLUT;
uniform sampler2D uMSLUT;
uniform float uHaze;
uniform vec3 uSunDir;
uniform float uSunIntensity;

const vec3 RAY_SCAT = vec3(5.802e-6, 13.558e-6, 33.1e-6);
const float RAY_H = 8000.0;
const float MIE_SCAT = 3.996e-6;
const float MIE_EXT = 4.440e-6;
const float MIE_H = 1200.0;
const vec3 OZONE_ABS = vec3(0.650e-6, 1.881e-6, 0.085e-6);
const vec3 GROUND_ALBEDO = vec3(0.12, 0.13, 0.10);
const vec3 PLANET_C = vec3(0.0, -R_GROUND, 0.0);

void mediumAt(float h, out vec3 rayS, out float mieS, out vec3 ext){
  h = max(h, 0.0);
  float rd = exp(-h / RAY_H);
  // aerosols: exponential layer + a denser boundary-layer haze controlled by uHaze
  float md = exp(-h / MIE_H) * (0.6 + 0.4 * uHaze) + uHaze * 0.9 * exp(-h / 650.0);
  float od = max(0.0, 1.0 - abs(h - 25000.0) / 15000.0);
  rayS = RAY_SCAT * rd;
  mieS = MIE_SCAT * md;
  ext = rayS + vec3(MIE_EXT * md) + OZONE_ABS * od;
}
float rayleighPhase(float c){ return 3.0 / (16.0 * PI) * (1.0 + c * c); }
float cornetteShanks(float c, float g){
  float k = 3.0 / (8.0 * PI) * (1.0 - g * g) / (2.0 + g * g);
  return k * (1.0 + c * c) / pow(max(1.0 + g * g - 2.0 * g * c, 1e-4), 1.5);
}
float hg(float c, float g){ float g2 = g*g; return (1.0 - g2) / (4.0 * PI * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5)); }

float raySphere(vec3 ro, vec3 rd, vec3 c, float r){ // nearest positive hit or -1
  vec3 oc = ro - c; float b = dot(oc, rd); float cc = dot(oc, oc) - r * r;
  float disc = b * b - cc; if (disc < 0.0) return -1.0;
  float s = sqrt(disc); float t0 = -b - s; float t1 = -b + s;
  if (t0 >= 0.0) return t0; if (t1 >= 0.0) return t1; return -1.0;
}
vec2 raySphere2(vec3 ro, vec3 rd, vec3 c, float r){ // (tNear, tFar), tFar < 0 => miss
  vec3 oc = ro - c; float b = dot(oc, rd); float cc = dot(oc, oc) - r * r;
  float disc = b * b - cc; if (disc < 0.0) return vec2(-1.0, -1.0);
  float s = sqrt(disc); return vec2(-b - s, -b + s);
}

// transmittance LUT parameterisation (Bruneton)
vec2 transUV(float r, float mu){
  float H = sqrt(R_TOP * R_TOP - R_GROUND * R_GROUND);
  float rho = sqrt(max(r * r - R_GROUND * R_GROUND, 0.0));
  float disc = r * r * (mu * mu - 1.0) + R_TOP * R_TOP;
  float d = max(0.0, -r * mu + sqrt(max(disc, 0.0)));
  float dmin = R_TOP - r, dmax = rho + H;
  float xmu = (d - dmin) / (dmax - dmin);
  float xr = rho / H;
  return vec2(xmu * (255.0 / 256.0) + 0.5 / 256.0, xr * (63.0 / 64.0) + 0.5 / 64.0);
}
vec3 transmittanceToTop(float r, float mu){ return textureLod(uTransLUT, transUV(r, mu), 0.0).rgb; }
// sun transmittance at a point including the planet shadow (soft over the sun disc)
vec3 sunTransmittance(vec3 p, vec3 sunDir){
  float r = length(p - PLANET_C); vec3 up = (p - PLANET_C) / r;
  float mu = dot(up, sunDir);
  float muH = -sqrt(max(1.0 - (R_GROUND / r) * (R_GROUND / r), 0.0));
  float vis = smoothstep(muH - 0.006, muH + 0.006, mu);
  return transmittanceToTop(r, mu) * vis;
}
vec3 msLookup(float h, float muS){
  vec2 uv = vec2(muS * 0.5 + 0.5, clamp(h / (R_TOP - R_GROUND), 0.0, 1.0));
  uv = uv * (31.0 / 32.0) + 0.5 / 32.0;
  return textureLod(uMSLUT, uv, 0.0).rgb;
}

// single + multiple scattering along a ray segment; returns inscatter (per unit sun illuminance) and transmittance
void integrateAtmo(vec3 ro, vec3 rd, float tMax, int steps, out vec3 L, out vec3 T){
  L = vec3(0.0); T = vec3(1.0);
  float cosT = dot(rd, uSunDir);
  float pr = rayleighPhase(cosT);
  float pm = cornetteShanks(cosT, 0.8);
  float tPrev = 0.0;
  for (int i = 0; i < 64; i++){
    if (i >= steps) break;
    float f = (float(i) + 0.5) / float(steps);
    float t = tMax * f * f;  // denser samples near the camera
    float tn = tMax * ((float(i) + 1.0) / float(steps)) * ((float(i) + 1.0) / float(steps));
    float dt = tn - tPrev; tPrev = tn;
    vec3 p = ro + rd * t;
    float r = length(p - PLANET_C);
    float h = r - R_GROUND;
    vec3 rayS; float mieS; vec3 ext;
    mediumAt(h, rayS, mieS, ext);
    vec3 up = (p - PLANET_C) / r;
    float muS = dot(up, uSunDir);
    vec3 Tsun = sunTransmittance(p, uSunDir);
    vec3 ms = msLookup(h, muS);
    vec3 S = Tsun * (rayS * pr + mieS * pm) + ms * (rayS + mieS);
    vec3 Ts = exp(-ext * dt);
    vec3 Sint = (S - S * Ts) / max(ext, vec3(1e-12));
    L += T * Sint;
    T *= Ts;
  }
}
`;

// sky-view LUT lookup (camera-centric)
export const SKYVIEW = /* glsl */ `
uniform sampler2D uSkyLUT;
uniform vec3 uSkyUp;      // local up at the camera
uniform vec3 uSkyEast;    // local east at the camera
uniform vec3 uSkyNorth;   // local north at the camera
uniform float uCamR;      // camera distance from planet centre
vec2 skyViewUV(vec3 dir){
  float r = max(uCamR, R_GROUND + 1.0);
  float vh = sqrt(r * r - R_GROUND * R_GROUND);
  float beta = acos(clamp(vh / r, -1.0, 1.0));
  float zh = PI - beta;
  float vz = acos(clamp(dot(dir, uSkyUp), -1.0, 1.0));
  float v;
  if (vz < zh){ float c = vz / zh; c = 1.0 - c; c = sqrt(max(c, 0.0)); c = 1.0 - c; v = c * 0.5; }
  else { float c = (vz - zh) / beta; c = sqrt(max(c, 0.0)); v = c * 0.5 + 0.5; }
  float az = atan(dot(dir, uSkyNorth), dot(dir, uSkyEast));
  float u = az / (2.0 * PI) + 0.5;
  return vec2(u, v);
}
vec3 skyRadiance(vec3 dir){ return texture(uSkyLUT, skyViewUV(dir)).rgb * uSunIntensity; }
`;

// ----------------------------------------------------------------------------------- clouds
export const CLOUD_COMMON = /* glsl */ `
uniform highp sampler3D uShapeTex;
uniform highp sampler3D uDetailTex;
uniform sampler2D uWeatherTex;
uniform float uCloudBase;
uniform float uCloudThick;
uniform float uCloudCoverage;
uniform float uCloudDensity;
uniform float uCloudType;
uniform vec2 uWindOffset;
uniform float uCloudShadowStrength;
uniform float uCloudTime;
// precipitation / thunderstorm
uniform float uRain;          // global precipitation multiplier (0 = none)
uniform vec4 uStormCell;      // x, z (three world), radius, amount: forced cumulonimbus cell
uniform vec2 uRainSlant;      // horizontal drift of falling rain per metre of fall (wind / fall speed), three xz
uniform vec4 uFlash;          // lightning: xyz position (three world), w intensity
uniform vec3 uFlashCol;

#define SHAPE_SCALE (1.0 / 7200.0)
#define DETAIL_SCALE (1.0 / 1100.0)
#define WEATHER_SCALE (1.0 / 52000.0)

float cloudHeightFrac(vec3 p){
  float r = length(p - vec3(0.0, -R_GROUND, 0.0));
  return (r - (R_GROUND + uCloudBase)) / uCloudThick;
}
// vertical density profile for a cloud type in [0 stratus .. 0.5 cumulus .. 1 cumulonimbus]
float heightProfile(float h, float t){
  float strat = smoothstep(0.0, 0.06, h) * (1.0 - smoothstep(0.18, 0.42, h));
  float cumu  = smoothstep(0.0, 0.14, h) * (1.0 - smoothstep(0.42, 0.92, h));
  float cb    = smoothstep(0.0, 0.08, h) * (1.0 - smoothstep(0.80, 1.0, h));
  return t < 0.5 ? mix(strat, cumu, t * 2.0) : mix(cumu, cb, t * 2.0 - 2.0 * 0.5);
}
vec4 sampleWeather(vec3 p){
  vec2 q = (p.xz + uWindOffset * 0.7) * WEATHER_SCALE;
  vec4 w = textureLod(uWeatherTex, q, 0.0);
  if (uStormCell.w > 0.0){
    // a forced storm cell: ragged edge from the weather noise itself
    float d = length(p.xz - uStormCell.xy) + (w.g - 0.5) * 0.7 * uStormCell.z;
    float b = uStormCell.w * 0.9 * (1.0 - smoothstep(0.45 * uStormCell.z, uStormCell.z, d));   // < 1: keeps internal structure
    w.r = max(w.r, b); w.g = max(w.g, b * 0.95); w.b = mix(w.b, 1.0, b);
  }
  return w;
}
float cloudCoverageAt(vec4 w){
  // large-scale coverage + breakup; global coverage slider shifts everything
  float c = mix(w.r, w.g, 0.35);
  float cov = uCloudCoverage;
  return saturate(remap(c, 1.0 - cov * 1.12, 1.0, 0.0, 1.0)) ;
}
// precipitation rate (0..1) reaching the ground below cumulonimbus at xz (three world)
float rainAt(vec2 xz){
  if (uRain <= 0.0) return 0.0;
  vec4 w = sampleWeather(vec3(xz.x, 0.0, xz.y));
  float cov = cloudCoverageAt(w);
  float ctype = saturate(uCloudType + (w.b - 0.5) * 0.35);
  return uRain * smoothstep(0.32, 0.78, cov) * smoothstep(0.6, 0.84, ctype);
}
// light from the current lightning discharge reaching p
vec3 flashLight(vec3 p){
  if (uFlash.w <= 0.0) return vec3(0.0);
  float d = length(p - uFlash.xyz);
  return uFlashCol * uFlash.w * exp(-d / 7000.0) / (1.0 + d * d / (1600.0 * 1600.0));
}
// lod: 0 = full (with detail erosion), 1 = shape only
float cloudDensity(vec3 p, float hf, bool detail){
  if (hf <= 0.0 || hf >= 1.0) return 0.0;
  vec4 w = sampleWeather(p);
  float cov = cloudCoverageAt(w);
  // anvil: the top of a forced storm cell spreads out far beyond the cell
  float anv = 0.0;
  if (uStormCell.w > 0.0){
    float dc = length(p.xz - uStormCell.xy) + (w.g - 0.5) * 0.5 * uStormCell.z;
    anv = uStormCell.w * smoothstep(0.72, 0.86, hf) * (1.0 - smoothstep(0.93, 1.0, hf))
        * (1.0 - smoothstep(uStormCell.z * 1.05, uStormCell.z * 2.0, dc));
  }
  if (cov <= 0.001 && anv <= 0.001) return 0.0;
  float ctype = saturate(uCloudType + (w.b - 0.5) * 0.35);
  // tilt clouds slightly downwind with height (wind shear)
  vec3 q = p + vec3(uWindOffset.x, 0.0, uWindOffset.y) + vec3(hf * 600.0, 0.0, hf * 250.0);
  vec4 s = textureLod(uShapeTex, q * SHAPE_SCALE, 0.0);
  float lowFbm = dot(s.gba, vec3(0.625, 0.25, 0.125));
  // no flat planes: the base undulates and frays (raised by up to ~9 % of the layer), cumulonimbus
  // tops end at uneven heights (turrets) instead of one ceiling
  float lift = (saturate(lowFbm * 1.6 - 0.45) * 0.06 + (1.0 - s.r) * 0.03) * (1.0 - smoothstep(0.0, 0.3, hf));
  float drop = (0.6 - lowFbm) * 0.32 * smoothstep(0.4, 0.85, hf) * smoothstep(0.55, 0.9, ctype);
  float hp = heightProfile(hf - lift + drop, ctype);
  float shape = remap(s.r, lowFbm - 1.0, 1.0, 0.0, 1.0);
  float base = saturate(remap(shape * hp, 1.0 - cov, 1.0, 0.0, 1.0)) * cov;
  if (anv > 0.0) base = max(base, anv * saturate(remap(shape, 0.35, 1.0, 0.0, 1.0)) * 0.75);
  if (base <= 0.0) return 0.0;
  if (detail){
    vec3 dq = q * DETAIL_SCALE + vec3(0.0, uCloudTime * 0.004, 0.0);
    vec3 dn = textureLod(uDetailTex, dq, 0.0).rgb;
    float dfbm = dot(dn, vec3(0.625, 0.25, 0.125));
    float dmod = mix(dfbm, 1.0 - dfbm, saturate(hf * 6.0));
    // wispier, more eroded bottom edge
    float er = 0.32 + 0.18 * (1.0 - smoothstep(0.0, 0.12, hf));
    base = saturate(remap(base, dmod * er, 1.0, 0.0, 1.0));
  }
  return base * uCloudDensity;
}
// optical-depth based cloud shadow for a surface point (cheap: 5 shape-only taps through the layer)
float cloudShadow(vec3 p, vec3 sunDir){
  if (uCloudShadowStrength <= 0.0) return 1.0;
  vec3 C = vec3(0.0, -R_GROUND, 0.0);
  vec3 up = normalize(p - C);
  float sy = dot(up, sunDir);
  if (sy < 0.01) return 1.0;
  float h = length(p - C) - R_GROUND;
  float b0 = uCloudBase, b1 = uCloudBase + uCloudThick;
  if (h > b1) return 1.0;
  float t0 = max((b0 - h) / sy, 0.0);
  float t1 = (b1 - h) / sy;
  float od = 0.0; float dt = (t1 - t0) / 5.0;
  for (int i = 0; i < 5; i++){
    vec3 q = p + sunDir * (t0 + (float(i) + 0.5) * dt);
    od += cloudDensity(q, cloudHeightFrac(q), false) * dt;
  }
  return mix(1.0, exp(-od * 0.0018), uCloudShadowStrength);
}
`;
