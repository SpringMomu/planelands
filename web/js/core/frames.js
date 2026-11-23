// Coordinate frames.
//   Sim / world : ENU (x East, y North, z Up), metres, origin = runway centre.
//   Body        : FRD (x forward, y right, z down), quaternion q=[w,x,y,z] rotates body -> ENU.
//   three world : (E, U, -N); the flat world is bent onto the planet: y' = y - (x^2+z^2)/(2R).
//   model local : (x_b, -z_b, y_b).
import * as THREE from 'three';
import { PLANET_R } from '../render/glsl.js';

export const curvatureDrop = (x, z) => (x * x + z * z) / (2 * PLANET_R);

export function enuToThree(e, n, u, out = new THREE.Vector3(), curved = true) {
  out.set(e, u, -n);
  if (curved) out.y -= curvatureDrop(out.x, out.z);
  return out;
}
export function threeToEnu(v) {
  return [v.x, -v.z, v.y + curvatureDrop(v.x, v.z)];
}

const _m = new THREE.Matrix4();
const _mw = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);      // ENU -> three
const _mbT = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);     // model local -> FRD (transpose of FRD->local)
const _q = new THREE.Quaternion();
const _up = new THREE.Vector3();
const _Y = new THREE.Vector3(0, 1, 0);

/** quaternion [w,x,y,z] (FRD->ENU) -> three quaternion for the model (local->three world) */
export function bodyQuatToThree(q, pos3, out = new THREE.Quaternion()) {
  const [w, x, y, z] = q;
  // rotation matrix of q (body -> ENU)
  _m.set(
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 0,
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 0,
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y), 0,
    0, 0, 0, 1);
  const R3 = new THREE.Matrix4().multiplyMatrices(_mw, _m).multiply(_mbT);
  out.setFromRotationMatrix(R3);
  if (pos3) {
    // tilt into the curved frame at this position
    _up.set(pos3.x / PLANET_R, 1, pos3.z / PLANET_R).normalize();
    _q.setFromUnitVectors(_Y, _up);
    out.premultiply(_q);
  }
  return out;
}

/** body FRD vector -> model local vector */
export function frdToLocal(v, out = new THREE.Vector3()) { return out.set(v[0], -v[2], v[1]); }

/** Euler angles (deg) -> FRD->ENU quaternion [w,x,y,z]; psi heading from north clockwise, theta pitch up, phi roll right */
export function eulerToQuat(psiDeg, thetaDeg, phiDeg) {
  // FRD->NED with 3-2-1, then NED->ENU
  const d = Math.PI / 180;
  const cy = Math.cos(psiDeg * d / 2), sy = Math.sin(psiDeg * d / 2);
  const cp = Math.cos(thetaDeg * d / 2), sp = Math.sin(thetaDeg * d / 2);
  const cr = Math.cos(phiDeg * d / 2), sr = Math.sin(phiDeg * d / 2);
  const qn = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
  // NED->ENU rotation quaternion: (x_e, y_n, z_u) = (y_ned, x_ned, -z_ned)
  const s = Math.SQRT1_2;
  const qe = [0, s, s, 0];
  return qmul(qe, qn);
}
export function qmul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}
