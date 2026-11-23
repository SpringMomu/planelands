// Camera rig: chase/orbit, cockpit, tower, runway-side, fly-by, free-fly and overview.
import * as THREE from 'three';
import { curvatureDrop } from '../core/frames.js';

export const CAMERA_MODES = ['chase', 'cockpit', 'tower', 'runway', 'flyby', 'free', 'overview'];
export const CAMERA_LABELS = { chase: '追随', cockpit: '驾驶舱', tower: '塔台', runway: '跑道旁', flyby: '飞掠', free: '自由', overview: '俯瞰' };

const _v = new THREE.Vector3(), _v2 = new THREE.Vector3(), _q = new THREE.Quaternion(), _e = new THREE.Euler();
const ALIGN = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2); // camera -Z -> model +X

export class CameraRig {
  constructor(camera, dom, terrain, airport) {
    this.camera = camera;
    this.dom = dom;
    this.terrain = terrain;
    this.airport = airport;
    this.mode = 'chase';
    this.yaw = 2.6; this.pitch = 0.18; this.dist = 75;
    this.lookYaw = 0; this.lookPitch = -0.05;
    this.free = { pos: new THREE.Vector3(), yaw: 0, pitch: 0, speed: 120 };
    this.keys = new Set();
    this.baseFov = 55;
    this.fov = 55;
    this.flyPos = null;
    this.smoothTarget = new THREE.Vector3();
    this.smoothHeading = 0;
    this._initInput();
  }

  _initInput() {
    let drag = false, lx = 0, ly = 0, button = 0;
    this.dom.addEventListener('contextmenu', (e) => e.preventDefault());
    this.dom.addEventListener('pointerdown', (e) => { drag = true; lx = e.clientX; ly = e.clientY; button = e.button; this.dom.setPointerCapture(e.pointerId); });
    this.dom.addEventListener('pointerup', (e) => { drag = false; this.dom.releasePointerCapture(e.pointerId); });
    this.dom.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const dx = (e.clientX - lx) / this.dom.clientHeight, dy = (e.clientY - ly) / this.dom.clientHeight;
      lx = e.clientX; ly = e.clientY;
      if (this.mode === 'chase' || this.mode === 'overview') { this.yaw -= dx * 3.2; this.pitch = THREE.MathUtils.clamp(this.pitch + dy * 2.4, -1.45, 1.45); }
      else if (this.mode === 'cockpit') { this.lookYaw -= dx * 2.0; this.lookPitch = THREE.MathUtils.clamp(this.lookPitch - dy * 2.0, -1.3, 1.3); }
      else if (this.mode === 'free') { this.free.yaw -= dx * 2.2; this.free.pitch = THREE.MathUtils.clamp(this.free.pitch - dy * 2.2, -1.5, 1.5); }
      else { this.fov = THREE.MathUtils.clamp(this.fov * (1 + dy * 2), 0.5, 70); }
    });
    this.dom.addEventListener('wheel', (e) => {
      e.preventDefault();
      const k = Math.exp(e.deltaY * 0.0012);
      if (this.mode === 'chase') this.dist = THREE.MathUtils.clamp(this.dist * k, 22, 6000);
      else if (this.mode === 'overview') this.dist = THREE.MathUtils.clamp(this.dist * k, 2000, 400000);
      else if (this.mode === 'free') this.free.speed = THREE.MathUtils.clamp(this.free.speed / k, 5, 20000);
      else this.fov = THREE.MathUtils.clamp(this.fov * k, 0.5, 75);
    }, { passive: false });
    window.addEventListener('keydown', (e) => { if (!e.target.closest || !e.target.closest('input,select,textarea')) this.keys.add(e.code); });
    window.addEventListener('keyup', (e) => this.keys.delete(e.code));
  }

  setMode(m) {
    if (!CAMERA_MODES.includes(m)) return;
    if (m === 'free') {
      this.free.pos.copy(this.camera.position);
      _e.setFromQuaternion(this.camera.quaternion, 'YXZ');
      this.free.yaw = _e.y; this.free.pitch = _e.x;
    }
    if (m === 'overview') { this.dist = Math.max(this.dist, 30000); this.pitch = 1.0; }
    if (m === 'chase' && this.dist > 6000) this.dist = 75;
    this.mode = m; this.flyPos = null;
    this.fov = m === 'cockpit' ? 62 : (m === 'tower' || m === 'runway' ? 20 : this.baseFov);
    if (m === 'cockpit') { this.lookYaw = 0; this.lookPitch = -0.08; }
  }

  _ground(p) {
    const e = p.x, n = -p.z;
    return this.terrain.surfaceAt(e, n) - curvatureDrop(p.x, p.z);
  }

  _autoZoom(dist, size = 38) {
    return THREE.MathUtils.clamp(2 * Math.atan(size * 1.7 / Math.max(dist, 1)) * 180 / Math.PI, 0.6, 60) * (this.fov / 20);
  }

  /** target: { pos (three), quat (three, model local->world), vel (three world) } */
  update(dt, target) {
    const cam = this.camera;
    const tp = target.pos;
    this.smoothTarget.copy(tp);
    const vel = target.vel;
    const speedH = Math.hypot(vel.x, vel.z);
    const hdg = speedH > 2 ? Math.atan2(vel.x, vel.z) : this.smoothHeading;
    let dh = hdg - this.smoothHeading; dh = Math.atan2(Math.sin(dh), Math.cos(dh));
    this.smoothHeading += dh * Math.min(1, dt * 1.5);
    let fov = this.fov;

    switch (this.mode) {
      case 'chase': {
        const yaw = this.smoothHeading + this.yaw;
        _v.set(Math.sin(yaw) * Math.cos(this.pitch), Math.sin(this.pitch), Math.cos(yaw) * Math.cos(this.pitch)).multiplyScalar(this.dist);
        cam.position.copy(tp).add(_v);
        const g = this._ground(cam.position) + 2.0;
        if (cam.position.y < g) cam.position.y = g;
        cam.up.set(0, 1, 0);
        cam.lookAt(tp);
        break;
      }
      case 'cockpit': {
        _v.copy(target.eye).applyQuaternion(target.quat).add(tp);
        cam.position.copy(_v);
        _q.setFromEuler(_e.set(this.lookPitch, this.lookYaw, 0, 'YXZ'));
        cam.quaternion.copy(target.quat).multiply(ALIGN).multiply(_q);
        break;
      }
      case 'tower': {
        this.airport.toWorld(120, -640, 38.5, cam.position);
        cam.up.set(0, 1, 0);
        cam.lookAt(tp);
        fov = this._autoZoom(cam.position.distanceTo(tp));
        break;
      }
      case 'runway': {
        const L = this.airport.rw.length;
        // pick the side of the runway the shuttle is approaching
        const e = tp.x, n = -tp.z;
        const u = e * this.airport.dir.x + n * this.airport.dir.y;
        const end = u < 0 ? -1 : 1;
        this.airport.toWorld(end * (L / 2 - 650), 135, 1.8, cam.position);
        cam.up.set(0, 1, 0);
        cam.lookAt(tp);
        fov = this._autoZoom(cam.position.distanceTo(tp));
        break;
      }
      case 'flyby': {
        const sp = vel.length();
        const ahead = Math.max(500, sp * 6);
        if (!this.flyPos || this.flyPos.distanceTo(tp) > ahead * 1.6 || _v2.copy(tp).sub(this.flyPos).dot(vel) > ahead * sp * 0.9) {
          const dir = _v.copy(vel).normalize();
          const side = new THREE.Vector3(-dir.z, 0, dir.x).normalize();
          this.flyPos = tp.clone().addScaledVector(dir, ahead).addScaledVector(side, 70 + Math.random() * 80);
          this.flyPos.y += 10 + Math.random() * 30;
          const g = this._ground(this.flyPos) + 3;
          if (this.flyPos.y < g) this.flyPos.y = g;
        }
        cam.position.copy(this.flyPos);
        cam.up.set(0, 1, 0);
        cam.lookAt(tp);
        fov = THREE.MathUtils.clamp(this._autoZoom(cam.position.distanceTo(tp), 45), 4, 60);
        break;
      }
      case 'free': {
        const f = this.free;
        _q.setFromEuler(_e.set(f.pitch, f.yaw, 0, 'YXZ'));
        const fwd = _v.set(0, 0, -1).applyQuaternion(_q), right = _v2.set(1, 0, 0).applyQuaternion(_q);
        const sp = f.speed * (this.keys.has('ShiftLeft') ? 6 : 1) * dt;
        if (this.keys.has('KeyW')) f.pos.addScaledVector(fwd, sp);
        if (this.keys.has('KeyS')) f.pos.addScaledVector(fwd, -sp);
        if (this.keys.has('KeyD')) f.pos.addScaledVector(right, sp);
        if (this.keys.has('KeyA')) f.pos.addScaledVector(right, -sp);
        if (this.keys.has('KeyE')) f.pos.y += sp;
        if (this.keys.has('KeyQ')) f.pos.y -= sp;
        const g = this._ground(f.pos) + 1.5;
        if (f.pos.y < g) f.pos.y = g;
        cam.position.copy(f.pos);
        cam.quaternion.copy(_q);
        break;
      }
      case 'overview': {
        const rwy = this.airport.toWorld(0, 0, 0);
        const mid = _v2.copy(tp).lerp(rwy, 0.35);
        const yaw = this.yaw;
        _v.set(Math.sin(yaw) * Math.cos(this.pitch), Math.sin(this.pitch), Math.cos(yaw) * Math.cos(this.pitch)).multiplyScalar(this.dist);
        cam.position.copy(mid).add(_v);
        cam.up.set(0, 1, 0);
        cam.lookAt(mid);
        break;
      }
    }
    if (Math.abs(cam.fov - fov) > 1e-3) { cam.fov += (fov - cam.fov) * Math.min(1, dt * 6); cam.updateProjectionMatrix(); }
    cam.updateMatrixWorld();
  }
}
