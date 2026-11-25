// Weather controller: precipitation under cumulonimbus (same field as the shaders, evaluated on the
// CPU from the weather map), near-camera rain, lightning scheduling, ground wetness and storm sound.
import * as THREE from 'three';
import { RainParticles } from './rain.js';
import { Lightning } from './lightning.js';
import { StormAudio } from './audio.js';

const WEATHER_SCALE = 1 / 52000;
const R_EARTH = 6371000;
const ss = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };
const sat = (x) => Math.min(1, Math.max(0, x));
// vertical density profile of the cloud types (heightProfile() in glsl.js)
function heightProfile(h, t) {
  const strat = ss(0, 0.06, h) * (1 - ss(0.18, 0.42, h));
  const cumu = ss(0, 0.14, h) * (1 - ss(0.42, 0.92, h));
  const cb = ss(0, 0.08, h) * (1 - ss(0.8, 1, h));
  return t < 0.5 ? strat + (cumu - strat) * t * 2 : cumu + (cb - cumu) * (t * 2 - 1);
}
const FLASH_EXT = 0.045;     // cloud extinction per unit density (clouds.js uCloudExt)

export class Weather {
  constructor({ U, weatherData, scene }) {
    this.U = U;
    this.data = weatherData;
    this.rain = new RainParticles();
    this.lightning = new Lightning(scene);
    this.audio = new StormAudio();
    this.hemi = new THREE.HemisphereLight(0xc6d4ff, 0x3a3f48, 0);
    scene.add(this.hemi);
    this.settings = { rain: 0, lightning: 0, wetAuto: true, wet: 0, cell: false };
    this.wet = 0;
    this.rainCam = 0;
    this.rainVeh = 0;
    this.bodyWet = 0;          // water on the orbiter's skin (0..1)
    this.flowTime = 0;         // drives the rivulets streaming aft
    this.storm = { turb: 0, gust: 0, draft: 0, level: 0 };
    this.flashActive = false;
    this.prevCam = null;
    this.camVel = new THREE.Vector3();
    this._w = new Float32Array(3);
    this._fall = new THREE.Vector3();
    this._col = new THREE.Color();
  }

  /** weather map (r, g, b) at three-world xz, bilinear + repeat like the GPU, with the forced storm cell */
  sample(x, z) {
    const U = this.U, wo = U.uWindOffset.value, d = this.data, w = this._w;
    const u = (x + wo.x * 0.7) * WEATHER_SCALE * 512 - 0.5, v = (z + wo.y * 0.7) * WEATHER_SCALE * 512 - 0.5;
    const i0 = Math.floor(u), j0 = Math.floor(v), fu = u - i0, fv = v - j0;
    w[0] = w[1] = w[2] = 0;
    for (let k = 0; k < 4; k++) {
      const di = k & 1, dj = k >> 1;
      const i = (((i0 + di) % 512) + 512) % 512, j = (((j0 + dj) % 512) + 512) % 512;
      const wt = (di ? fu : 1 - fu) * (dj ? fv : 1 - fv);
      const o = (j * 512 + i) * 4;
      w[0] += d[o] * wt; w[1] += d[o + 1] * wt; w[2] += d[o + 2] * wt;
    }
    w[0] /= 255; w[1] /= 255; w[2] /= 255;
    const c = U.uStormCell.value;
    if (c.w > 0) {
      const dd = Math.hypot(x - c.x, z - c.y) + (w[1] - 0.5) * 0.7 * c.z;
      const b = c.w * 0.9 * (1 - ss(0.45 * c.z, c.z, dd));
      w[0] = Math.max(w[0], b); w[1] = Math.max(w[1], b * 0.95); w[2] = w[2] + (1 - w[2]) * b;
    }
    return w;
  }

  /** precipitation 0..1 at three-world xz (mirrors rainAt() in glsl.js) */
  rainAt(x, z) {
    const U = this.U;
    if (U.uRain.value <= 0) return 0;
    const w = this.sample(x, z);
    const c = w[0] + (w[1] - w[0]) * 0.35;
    const lo = 1 - U.uCloudCoverage.value * 1.12;
    const cov = sat((c - lo) / (1 - lo));
    const ctype = sat(U.uCloudType.value + (w[2] - 0.5) * 0.35);
    return U.uRain.value * ss(0.32, 0.78, cov) * ss(0.6, 0.84, ctype);
  }

  /** cloud cover / type at xz: { cov, ctype, cb (cumulonimbus 0..1), cu (ordinary cumulus), core } */
  column(x, z) {
    const U = this.U, w = this.sample(x, z);
    const c = w[0] + (w[1] - w[0]) * 0.35;
    const lo = 1 - U.uCloudCoverage.value * 1.12;
    const cov = sat((c - lo) / (1 - lo));
    const ctype = sat(U.uCloudType.value + (w[2] - 0.5) * 0.35);
    const cb = ss(0.32, 0.78, cov) * ss(0.6, 0.84, ctype);
    const cu = ss(0.2, 0.7, cov) * ss(0.3, 0.6, ctype) * (1 - cb);
    return { cov, ctype, cb, cu, core: ss(0.55, 0.9, c) };
  }

  /**
   * storm forcing for the flight model at the vehicle: turbulence level (0..1), extra wind (gust front /
   * outflow, m/s) and vertical draft (m/s, + up): updrafts in cumulonimbus cores, downdrafts on their
   * flanks and in the rain shafts below the base.
   */
  stormAt(x, z, alt, agl) {
    const U = this.U;
    const col = this.column(x, z);
    const base = U.uCloudBase.value, thick = U.uCloudThick.value;
    const hf = (alt - base) / thick;
    const dens = Math.min(1.5, U.uCloudDensity.value);
    let turb = 0, gust = 0, draft = 0;
    if (hf > 0 && hf < 1) {
      const prof = Math.sin(Math.PI * Math.min(1, hf / 0.92));
      turb = Math.max(0.35 * col.cu, 0.95 * col.cb) * (0.55 + 0.45 * prof) * dens;
      draft = (col.cb * (16 * col.core - 6 * (1 - col.core)) + 2.5 * col.cu) * prof;
      gust = 7 * col.cb;
    } else if (hf <= 0) {
      const rain = U.uRain.value > 0 ? this.rainAt(x, z) / U.uRain.value : 0;
      turb = col.cb * (0.4 + 0.25 * rain);
      draft = -col.cb * (1.5 + 5.5 * rain) * ss(0, 400, agl);
      gust = col.cb * (3 + 5 * rain);
    } else {
      turb = 0.25 * col.cb * Math.exp(-(hf - 1) * 3);       // overshooting top
    }
    turb = Math.min(1, turb);
    return { turb, gust, draft, level: Math.max(col.cb * (hf < 1.1 ? 1 : 0), turb) };
  }

  /** mean cloud density at a point (the shader's density field averaged over its noise) */
  cloudDensityAvg(x, y, z) {
    const U = this.U, base = U.uCloudBase.value, thick = U.uCloudThick.value;
    const hf = (y + (x * x + z * z) / (2 * R_EARTH) - base) / thick;
    if (hf <= 0 || hf >= 1) return 0;
    const col = this.column(x, z);
    if (col.cov <= 0.001) return 0;
    const hp = heightProfile(hf, col.ctype);
    return col.cov * sat((0.65 * hp - (1 - col.cov)) / col.cov) * Math.min(1.5, U.uCloudDensity.value);
  }

  /** fraction of a flash at `a` reaching `b` through the clouds (flashTransfer() in glsl.js) */
  flashTransfer(a, bx, by, bz) {
    const N = 16, dx = bx - a.x, dy = by - a.y, dz = bz - a.z;
    const L = Math.hypot(dx, dy, dz);
    let od = 0;
    for (let i = 0; i < N; i++) {
      const f = (i + 0.5) / N;
      od += this.cloudDensityAvg(a.x + dx * f, a.y + dy * f, a.z + dz * f);
    }
    od *= L / N * FLASH_EXT;
    const Td = Math.exp(-od);
    return Td + (1 - Td) * 0.6 / (1 + 0.11 * od);
  }

  /** a cloud-to-ground strike a few km in front of the camera (demo button) */
  strikeNow(camera, groundY) {
    const U = this.U;
    const f = new THREE.Vector3();
    camera.getWorldDirection(f);
    f.y = 0;
    if (f.lengthSq() < 1e-6) f.set(1, 0, 0);
    f.normalize();
    const d = 2500 + Math.random() * 3500, side = (Math.random() - 0.5) * 2500;
    const x = camera.position.x + f.x * d - f.z * side, z = camera.position.z + f.z * d + f.x * side;
    const ev = this.lightning.spawn({ type: 'cg', pos: new THREE.Vector3(x, 0, z), groundY: groundY(x, z), base: U.uCloudBase.value, thick: U.uCloudThick.value, camPos: camera.position });
    this.audio.thunder(ev);
  }

  /**
   * @param {number} dtSim  sim-time step
   * @param {number} dtReal real-time step
   * @param {object} c { camera, vehPos3, wind3, groundY(x, z), groundYCam, camAgl, amb: Color, ts }
   */
  update(dtSim, dtReal, c) {
    const U = this.U, S = this.settings;
    U.uRain.value = S.rain;
    if (S.cell) U.uStormCell.value.set(1800, -2600, 11000, 1); else U.uStormCell.value.set(0, 0, 1, 0);
    // rain drifts with the low-passed wind (gusts must not shake the rain shafts)
    const kw = Math.min(1, dtReal / 1.5);
    this._fall.x += (c.wind3.x - this._fall.x) * kw;
    this._fall.z += (c.wind3.z - this._fall.z) * kw;
    this._fall.y = -8.5;
    U.uRainSlant.value.set(this._fall.x / 8.5, this._fall.z / 8.5);
    const cp = c.camera.position;
    // camera velocity in sim m/s (camera cuts are ignored)
    if (this.prevCam && dtSim > 1e-4) {
      const v = cp.clone().sub(this.prevCam).divideScalar(dtSim);
      if (v.length() < 1500) this.camVel.lerp(v, 0.5);
    }
    if (!this.prevCam) this.prevCam = new THREE.Vector3();
    this.prevCam.copy(cp);

    const base = U.uCloudBase.value, thick = U.uCloudThick.value;
    const camAlt = cp.y + (cp.x * cp.x + cp.z * cp.z) / (2 * R_EARTH);
    // rain falls from the lower half of the cumulonimbus down to the ground
    const hfac = (alt) => 1 - ss(base + thick * 0.5, base + thick * 0.95, alt);
    const P = this.rainAt(cp.x, cp.z) * hfac(camAlt);
    this.rainCam += (P - this.rainCam) * Math.min(1, dtReal / 1.2);

    // the orbiter: rain it flies through, storm forcing, water on the skin
    if (c.vehPos3) {
      const vx = c.vehPos3.x, vz = c.vehPos3.z;
      const Pv = this.rainAt(vx, vz) * hfac(c.vehAlt);
      this.rainVeh += (Pv - this.rainVeh) * Math.min(1, dtReal / 0.6);
      this.storm = this.stormAt(vx, vz, c.vehAlt, c.vehAgl);
      const colV = this.column(vx, vz);
      const inCloud = c.vehAlt > base && c.vehAlt < base + thick ? ss(0.25, 0.7, colV.cov) : 0;
      const wetIn = Math.min(1, Pv * 1.3 + inCloud * 0.3);      // cloud droplets alone only dampen the skin
      if (wetIn > this.bodyWet) this.bodyWet += (wetIn - this.bodyWet) * (1 - Math.exp(-dtSim / 5));
      else {
        const tau = c.onGround ? 900 : 60 + 240 / (1 + c.vehTas / 60);   // airflow dries it in flight
        this.bodyWet += (wetIn - this.bodyWet) * (1 - Math.exp(-dtSim / tau));
      }
      this.flowTime += dtSim * (0.15 + Math.min(c.vehTas, 250) / 70);
    }

    // ground wetness: soaks up over ~half a minute of rain, dries over ~10 minutes
    if (S.wetAuto) {
      let target = Math.max(this.rainCam, this.rainAt(0, 0));
      if (c.vehPos3) target = Math.max(target, this.rainAt(c.vehPos3.x, c.vehPos3.z));
      const tau = target > this.wet ? 30 : 600;
      this.wet += (target - this.wet) * (1 - Math.exp(-dtSim / tau));
    } else this.wet = S.wet;
    U.uWet.value = this.wet;

    // listener for the thunder: the camera (moving, possibly in the cockpit)
    this._right = this._right || new THREE.Vector3();
    this._right.setFromMatrixColumn(c.camera.matrixWorld, 0);
    this.audio.setListener({ pos: cp, vel: this.camVel, groundY: c.groundYCam, right: this._right, inCockpit: !!c.inCockpit });

    // lightning
    const rate = S.rain > 0 ? S.lightning * 0.45 : 0;
    const gY = c.groundYCam;
    const fl = this.lightning.update(dtSim, {
      rate, camPos: cp, base, thick, groundY: c.groundY,
      findSite: () => {
        for (let k = 0; k < 16; k++) {
          const r = 1500 + 26000 * Math.pow(Math.random(), 0.8), a = Math.random() * Math.PI * 2;
          const x = cp.x + Math.cos(a) * r, z = cp.z + Math.sin(a) * r;
          if (this.rainAt(x, z) > 0.4 * S.rain) return { x, z };
        }
        return null;
      },
      onStrike: (ev) => this.audio.thunder(ev),
      // the flash light reaching the camera (lights the objects around it: hemisphere light, sky glow)
      // and the ground below it, through whatever cloud lies in between - from above a thick deck a
      // flash inside / below it is only a faint diffuse glow of the cloud tops
      vis: (ev) => {
        const f = ev.flashPos;
        if (!ev._vis || ev._visT === undefined || Math.abs(ev.t - ev._visT) > 0.05) {
          ev._vis = {
            cam: this.flashTransfer(f, cp.x, cp.y, cp.z), dCam: f.distanceTo(cp),
            gnd: this.flashTransfer(f, cp.x, gY + 2, cp.z), dGnd: Math.hypot(f.x - cp.x, f.y - gY, f.z - cp.z),
          };
          ev._visT = ev.t;
        }
        return ev._vis;
      },
    });
    if (fl.pos) U.uFlash.value.set(fl.pos.x, fl.pos.y, fl.pos.z, fl.I); else U.uFlash.value.w = 0;
    U.uFlashAmb.value = fl.ambG;
    U.uFlashVis.value = fl.vis;
    this.hemi.intensity = fl.amb;
    this.flashActive = fl.I > 1;

    // near-camera rain
    const fc = U.uFlashCol.value;
    this._col.copy(c.amb).multiplyScalar(1.9 * (1 - 0.35 * this.rainCam));
    this._col.r += fc.x * fl.amb * 0.35; this._col.g += fc.y * fl.amb * 0.35; this._col.b += fc.z * fl.amb * 0.35;
    this.rain.update({
      amount: Math.min(1, this.rainCam * 1.15), time: U.uCloudTime.value, dt: dtSim, camPos: cp, camVel: this.camVel,
      fall: this._fall, exposure: 0.02 * c.ts, groundY: c.groundYCam, camAgl: c.camAgl, col: this._col,
    });
    this.audio.setRain(c.camAgl < 150 ? this.rainCam * (1 - c.camAgl / 150) : 0);
  }

  /** velocity of the falling rain (world, m/s: terminal fall + low-passed wind) */
  get fall() { return this._fall; }

  renderParticles(renderer, camera, depthTexture) {
    return this.rain.render(renderer, camera, depthTexture);
  }
}
