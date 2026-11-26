// Surface pressure paint for the orbiter's skin (injected into the tile material, shuttle.js).
//
// Per vertex (computed in airflow.js from the aerodynamic model): aPaint = (Cp, surface flow velocity
// / free-stream speed in mesh coordinates), aPaint2 = (separation 0..1, painted flag, 0, 0).
// The fragment replaces the tile colour by a diverging Cp colour map with faint isobars and draws
// short streaks that run along the surface flow at its local speed (an animated oil-flow picture);
// where the flow is separated each streak wanders on its own.
export const PAINT = {
  uPaintOn: { value: 0 },
  uPaintTime: { value: 0 },
  uPaintVis: { value: 15 },          // display speed of the free stream (m/s)
  uPaintEmit: { value: 1 },          // self-illumination so the colours read in the shade
};

export const PAINT_VERT_DECL = /* glsl */ `
attribute vec4 aPaint;
attribute vec4 aPaint2;
varying vec4 vPaint;
varying vec4 vPaint2;`;

export const PAINT_VERT = /* glsl */ `vPaint = aPaint; vPaint2 = aPaint2;`;

export const PAINT_FRAG_DECL = /* glsl */ `
uniform float uPaintOn, uPaintTime, uPaintVis, uPaintEmit;
varying vec4 vPaint;
varying vec4 vPaint2;
vec3 cpColor(float cp){
  // suction: grey-white -> blue -> deep blue (Cp -1.2); pressure: -> amber -> red (Cp +0.8)
  vec3 w = vec3(0.80, 0.80, 0.80);
  if (cp < 0.0){
    float t = clamp(-cp / 1.2, 0.0, 1.0);
    return mix(mix(w, vec3(0.22, 0.52, 0.95), smoothstep(0.0, 0.35, t)), vec3(0.05, 0.10, 0.62), smoothstep(0.35, 1.0, t));
  }
  float t = clamp(cp / 0.8, 0.0, 1.0);
  return mix(mix(w, vec3(1.0, 0.58, 0.12), smoothstep(0.0, 0.4, t)), vec3(0.80, 0.06, 0.04), smoothstep(0.4, 1.0, t));
}
vec3 paintHash(vec3 p){
  p = fract(p * vec3(0.1031, 0.1030, 0.0973));
  p += dot(p, p.yxz + 33.33);
  return fract((p.xxy + p.yxx) * p.zyx);
}
// flow lines on the skin: lanes of the surface streamlines (lane coordinate psi), dashes that move
// along them at the local speed (tau: travel time from the stagnation line, as metres at the
// free-stream speed).  Where the flow is separated the lanes wander and the dashes slow down.
float skinLanes(float tau, float lane, float sep){
  float li0 = floor(lane);
  if (sep > 0.02){
    float h0 = fract(sin(li0 * 12.9898) * 43758.5453);
    lane += sep * 0.9 * sin(tau * 0.8 + uPaintTime * 2.3 + h0 * 6.28);
    tau -= sep * 0.75 * uPaintTime * uPaintVis;
  }
  float fl = max(fwidth(lane), 1e-5), ft = fwidth(tau) / 2.4;
  float vis = (1.0 - smoothstep(0.3, 0.6, fl)) * (1.0 - smoothstep(0.3, 0.6, ft));
  if (vis <= 0.0) return 0.0;
  float li = floor(lane);
  float h = fract(sin(li * 12.9898 + 4.1) * 43758.5453);
  // lines of constant screen width: where the streamlines spread apart they just get sparser
  float across = 1.0 - smoothstep(0.7, 1.5, abs(fract(lane) - 0.5) / fl);
  const float D = 2.4;                                    // dash spacing at the free-stream speed (m)
  float ph = fract(tau / D - uPaintTime * uPaintVis / D + h);
  float dash = smoothstep(0.05, 0.6, ph) * (1.0 - smoothstep(0.62, 0.7, ph));   // tail -> head (downstream)
  return across * dash * vis;
}
float skinStreaks(vec3 P, vec3 N, vec3 V, float sep){
  float s = length(V);
  if (s < 1e-3) return 0.0;
  vec3 t = V - N * dot(V, N);
  float tl = length(t);
  if (tl < 1e-4) return 0.0;
  t /= tl;
  vec3 bt = cross(N, t);
  const float C = 1.05;
  vec3 base = floor(P / C - 0.5);
  float acc = 0.0;
  for (int k = 0; k < 8; k++){
    vec3 cell = base + vec3(float(k & 1), float((k >> 1) & 1), float(k >> 2));
    vec3 h = paintHash(cell);
    if (fract(h.x * 7.13 + h.y * 3.1) < 0.35) continue;      // thin the streaks out
    vec3 q = (cell + h) * C;
    q -= N * dot(q - P, N);
    vec3 dir = t;
    if (sep > 0.02){
      float ang = sep * ((h.x - 0.5) * 5.0 + 1.3 * sin(uPaintTime * (1.3 + h.y) + 6.28 * h.z));
      dir = t * cos(ang) + bt * sin(ang);
    }
    float L = 1.8 * C;
    float ph = fract(uPaintTime * uPaintVis * s * (1.0 - 0.6 * sep) / L + h.z);
    vec3 d = P - (q + dir * (ph - 0.5) * L);
    float a = dot(d, dir) / 0.62;                       // along, in half-lengths (head at +0.25)
    float b = length(d - dir * dot(d, dir));
    float wd = 0.05 + 0.6 * fwidth(b);
    float along = smoothstep(-1.0, 0.25, a) * (1.0 - smoothstep(0.25, 0.42, a));
    float across = 1.0 - smoothstep(0.5 * wd, wd, b);
    acc = max(acc, along * across * sin(3.14159 * ph));
  }
  return acc;
}`;

// after the tile colour: tCol / tRough / tMetal / tBump are the tile shader's outputs
// after the lights: a matte paint - grazing reflections of the bright sky would wash the map out
export const PAINT_LIGHT = /* glsl */ `
if (uPaintOn > 0.5 && vPaint2.y > 0.5){ reflectedLight.indirectSpecular *= 0.0; reflectedLight.directSpecular *= 0.1; }`;

export const PAINT_FRAG = /* glsl */ `
if (uPaintOn > 0.5 && vPaint2.y > 0.5){
  float cp = vPaint.x;
  vec3 pc = cpColor(cp);
  // faint isobars every 0.2 in Cp
  float fi = cp / 0.2;
  float di = abs(fract(fi + 0.5) - 0.5) / max(fwidth(fi), 1e-4);
  pc *= 1.0 - 0.22 * (1.0 - smoothstep(0.4, 1.2, di));
  // a darker, saturated albedo keeps the colours under full sunlight; light streaks on top
  float st = vPaint2.y > 1.5 ? skinLanes(vPaint2.z, vPaint2.w, vPaint2.x) : skinStreaks(vObjPos, normalize(vObjN), vPaint.yzw, vPaint2.x);
  vec3 alb = pc * 0.42;
  diffuseColor.rgb = mix(alb, vec3(0.8), st * 0.85);
  totalEmissiveRadiance += (alb * 0.9 + vec3(0.7) * st) * uPaintEmit;
  tRough = 0.85; tMetal = 0.0; tBump = 0.0;
}`;
