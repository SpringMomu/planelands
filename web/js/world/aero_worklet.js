// AudioWorklet processor of the airflow sound: the spectral noise synthesiser (aero_synth.js) on the
// audio thread.  Messages from aero_audio.js go straight to the synthesiser.
import { SpectralNoise } from './aero_synth.js';

class AeroNoise extends AudioWorkletProcessor {
  constructor() {
    super();
    this.synth = new SpectralNoise(sampleRate);
    this.port.onmessage = (e) => this.synth.message(e.data);
  }

  process(inputs, outputs) {
    const out = outputs[0];
    this.synth.render(out[0], out[1] || out[0]);
    return true;
  }
}

registerProcessor('aero-noise', AeroNoise);
