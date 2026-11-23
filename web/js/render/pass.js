// Fullscreen pass helper (RawShaderMaterial, GLSL 3).
import * as THREE from 'three';

const TRI = new THREE.BufferGeometry();
TRI.setAttribute('position', new THREE.Float32BufferAttribute([-1, -1, 0, 3, -1, 0, -1, 3, 0], 3));
TRI.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 10);
const CAM = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);

const VS = /* glsl */ `
in vec3 position;
out vec2 vUv;
void main(){ vUv = position.xy * 0.5 + 0.5; gl_Position = vec4(position.xy, 0.0, 1.0); }
`;

export const PASS_PREFIX = /* glsl */ `
precision highp float;
precision highp int;
precision highp sampler2D;
precision highp sampler3D;
`;

export class FullscreenPass {
  constructor(fragmentShader, uniforms = {}, { defines = {}, outputs = 1, blending = THREE.NoBlending } = {}) {
    const outs = outputs === 1
      ? 'layout(location = 0) out vec4 outColor;\n'
      : Array.from({ length: outputs }, (_, i) => `layout(location = ${i}) out vec4 out${i};`).join('\n') + '\n';
    this.material = new THREE.RawShaderMaterial({
      glslVersion: THREE.GLSL3,
      vertexShader: VS,
      fragmentShader: PASS_PREFIX + outs + 'in vec2 vUv;\n' + fragmentShader,
      uniforms,
      defines,
      depthTest: false,
      depthWrite: false,
      blending,
      transparent: blending !== THREE.NoBlending,
    });
    this.mesh = new THREE.Mesh(TRI, this.material);
    this.mesh.frustumCulled = false;
    this.scene = new THREE.Scene();
    this.scene.add(this.mesh);
    this.uniforms = this.material.uniforms;
  }
  render(renderer, target) {
    renderer.setRenderTarget(target);
    renderer.render(this.scene, CAM);
  }
}

export function makeRT(w, h, opts = {}) {
  return new THREE.WebGLRenderTarget(w, h, {
    type: THREE.HalfFloatType,
    format: THREE.RGBAFormat,
    minFilter: THREE.LinearFilter,
    magFilter: THREE.LinearFilter,
    depthBuffer: false,
    stencilBuffer: false,
    generateMipmaps: false,
    ...opts,
  });
}
