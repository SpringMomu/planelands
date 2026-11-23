// Patches three.js standard materials so they share the scene's cloud shadows (and any
// extra per-material GLSL injected by the caller).
import * as THREE from 'three';
import { COMMON, CLOUD_COMMON } from '../render/glsl.js';

const LIGHTS_BEGIN = THREE.ShaderChunk.lights_fragment_begin.replace(
  'getDirectionalLightInfo( directionalLight, directLight );',
  'getDirectionalLightInfo( directionalLight, directLight );\n\t\tdirectLight.color *= cloudShadowF;'
);

/**
 * @param {THREE.Material} mat  MeshStandardMaterial / MeshPhysicalMaterial
 * @param {object} U            global uniforms (cloud + sun)
 * @param {object} [opt]        { key, vertex(sh), fragment(sh), uniforms }
 */
export function patchStandard(mat, U, opt = {}) {
  const prev = mat.onBeforeCompile;
  const prevKey = mat.customProgramCacheKey ? mat.customProgramCacheKey() : '';
  mat.onBeforeCompile = (sh, renderer) => {
    if (prev && prev !== THREE.Material.prototype.onBeforeCompile) prev(sh, renderer);
    for (const k of ['uShapeTex', 'uDetailTex', 'uWeatherTex', 'uCloudBase', 'uCloudThick', 'uCloudCoverage', 'uCloudDensity',
      'uCloudType', 'uWindOffset', 'uCloudShadowStrength', 'uCloudTime', 'uSunDir',
      'uRain', 'uStormCell', 'uRainSlant', 'uFlash', 'uFlashCol']) if (U[k]) sh.uniforms[k] = U[k];
    if (opt.uniforms) Object.assign(sh.uniforms, opt.uniforms);
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vWPos;')
      .replace('#include <worldpos_vertex>', `#include <worldpos_vertex>
        {
          vec4 wpc = vec4(transformed, 1.0);
          #ifdef USE_INSTANCING
          wpc = instanceMatrix * wpc;
          #endif
          vWPos = (modelMatrix * wpc).xyz;
        }`);
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', '#include <common>\n' + COMMON + '\nuniform vec3 uSunDir;\n' + CLOUD_COMMON + '\nvarying vec3 vWPos;')
      .replace('#include <lights_fragment_begin>', 'float cloudShadowF = cloudShadow(vWPos, uSunDir);\n' + LIGHTS_BEGIN);
    if (opt.vertex) opt.vertex(sh);
    if (opt.fragment) opt.fragment(sh);
  };
  const key = 'cs|' + prevKey + '|' + (opt.key || '');
  mat.customProgramCacheKey = () => key;
  mat.needsUpdate = true;
  return mat;
}
