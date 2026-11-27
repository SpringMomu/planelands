// Surface pressure paint for the orbiter's skin (injected into the tile material, shuttle.js).
//
// Per vertex (computed in airflow.js from the aerodynamic model): aPaint = (Cp, surface flow velocity
// / free-stream speed in mesh coordinates), aPaint2 = (separation 0..1, painted flag, 0, 0).
// The fragment replaces the tile colour by a diverging Cp colour map with faint isobars.
export const PAINT = {
  uPaintOn: { value: 0 },
  uPaintEmit: { value: 1 },          // self-illumination so the colours read in the shade
};

export const PAINT_VERT_DECL = /* glsl */ `
attribute vec4 aPaint;
attribute vec4 aPaint2;
varying vec4 vPaint;
varying vec4 vPaint2;`;

export const PAINT_VERT = /* glsl */ `vPaint = aPaint; vPaint2 = aPaint2;`;

export const PAINT_FRAG_DECL = /* glsl */ `
uniform float uPaintOn, uPaintEmit;
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
  // a darker, saturated albedo keeps the colours under full sunlight
  vec3 alb = pc * 0.42;
  diffuseColor.rgb = alb;
  totalEmissiveRadiance += alb * 0.9 * uPaintEmit;
  tRough = 0.85; tMetal = 0.0; tBump = 0.0;
}`;
