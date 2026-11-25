// The airflow sound of the gliding orbiter (WebAudio): web/js/world/aeroacoustics.js computes the
// pressure spectrum at the listener (the camera, or the flight deck) from the flight state every
// frame; the spectral synthesiser (aero_synth.js) turns it into sound - on the audio thread
// (aero_worklet.js), or through a ScriptProcessorNode where AudioWorklet is unavailable (pages opened over
// plain http from another machine: AudioWorklet exists only in secure contexts).  Levels are physical: 1 Pa (94 dB SPL) at
// the listener plays at FS_DB dBFS, so the cabin at final approach, a low fly-by and the tower keep
// their real level differences; the master compressor / soft clip of StormAudio catches the loudest.

import { AeroAcoustics, NB, FC } from './aeroacoustics.js';
import { SpectralNoise } from './aero_synth.js';

const FS_DB = -16;                         // dBFS (rms) of 1 Pa
const GAIN = Math.pow(10, FS_DB / 20);

export class AeroAudio {
  constructor(geometry) {
    this.model = new AeroAcoustics(geometry);
    this.node = null;
    this.ctx = null;
    this.mode = 'off';                     // 'worklet' | 'script' | 'off'
    this.enabled = true;
    this.spl = 0;                          // overall level at the listener (dB re 20 uPa), for display / debugging
    this.delay = 0;
    this._dir = new Float32Array(NB); this._wl = new Float32Array(NB); this._dif = new Float32Array(NB);
  }

  /** start the worklet on the audio context (after the user gesture that unlocked it) */
  attach(ctx, dest) {
    if (this.ctx) return;
    this.ctx = ctx;
    const fallback = (why) => {
      console.warn(`aero audio: AudioWorklet unavailable (${why}), synthesising on the main thread`);
      try {
        const synth = new SpectralNoise(ctx.sampleRate);
        synth.message({ bands: Float64Array.from(FC) });
        const sp = ctx.createScriptProcessor(2048, 1, 2);
        sp.onaudioprocess = (e) => synth.render(e.outputBuffer.getChannelData(0), e.outputBuffer.getChannelData(1));
        sp.connect(dest);
        this.node = { port: { postMessage: (m) => synth.message(m) }, sp };
        this.mode = 'script';
      } catch (e) { console.warn('aero audio: no synthesis path', e); }
    };
    if (!ctx.audioWorklet || typeof AudioWorkletNode === 'undefined') { fallback('not a secure context?'); return; }
    ctx.audioWorklet.addModule(new URL('./aero_worklet.js', import.meta.url)).then(() => {
      this.node = new AudioWorkletNode(ctx, 'aero-noise', { numberOfInputs: 0, numberOfOutputs: 1, outputChannelCount: [2] });
      this.node.port.postMessage({ bands: Float64Array.from(FC) });
      this.node.connect(dest);
      this.mode = 'worklet';
    }).catch((e) => fallback(e && e.message ? e.message : e));
  }

  setEnabled(v) { this.enabled = v; }

  /** vehicle sample (three world, Vector3): { t, pos, vel, wind, fwd, up, right (body x / -z / y axes),
   *  alt (m MSL), agl, surf (deg), gear (0..1) } */
  pushVehicle(v) {
    const a = (u) => [u.x, u.y, u.z];
    this.model.push({
      t: v.t, pos: a(v.pos), vel: a(v.vel), wind: a(v.wind), fwd: a(v.fwd), up: a(v.up), right: a(v.right),
      alt: v.alt, agl: v.agl, surf: v.surf || {}, gear: v.gear || 0,
    });
  }

  /** listener: { t, pos, vel (Vector3, sim m/s), right (Vector3), alt, groundAlt, cabin, rh, mute } */
  update(L) {
    const o = this.model.listen({
      t: L.t, pos: [L.pos.x, L.pos.y, L.pos.z], vel: [L.vel.x, L.vel.y, L.vel.z], right: [L.right.x, L.right.y, L.right.z],
      alt: L.alt, groundAlt: L.groundAlt, cabin: !!L.cabin, rh: L.rh,
    });
    this.spl = o.spl; this.delay = o.delay;
    if (!this.node) return;
    const on = this.enabled && !L.mute;
    for (let i = 0; i < NB; i++) {
      this._dir[i] = on ? o.dir[i] : 0; this._dif[i] = on ? o.dif[i] : 0; this._wl[i] = o.wl[i];
    }
    this.node.port.postMessage({ dir: this._dir, wl: this._wl, dif: this._dif, gain: GAIN });
  }

  reset() { this.model.reset(); }
}
