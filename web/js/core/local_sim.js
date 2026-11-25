// A WebSocket look-alike backed by the in-browser simulator (sim/sim_worker.js): SimLink uses it in
// place of the server connection; the messages are identical. It starts loading as soon as it is
// constructed; worker messages are held until SimLink attaches (see attach()).
export class LocalSim {
  constructor(cfg) {
    this.readyState = 0;                       // CONNECTING
    this.onopen = this.onclose = this.onmessage = this.onerror = null;
    this.onProgress = this.onFatal = null;
    this.attached = false;
    this._held = [];
    this.worker = new Worker(new URL('../sim/sim_worker.js', import.meta.url), { type: 'module' });
    this.worker.onmessage = (e) => (this.attached ? this._on(e.data) : this._held.push(e.data));
    this.worker.onerror = (e) => {
      const m = { type: 'fatal', text: e.message || 'worker error' };
      this.attached ? this._on(m) : this._held.push(m);
    };
    this.worker.postMessage({ type: 'init', cfg });
  }
  /** start delivering: set onopen / onmessage / onclose first */
  attach({ onProgress, onFatal } = {}) {
    this.onProgress = onProgress; this.onFatal = onFatal;
    // replay asynchronously, as a socket would deliver (the caller is still inside its constructor)
    setTimeout(() => {
      this.attached = true;
      const held = this._held; this._held = [];
      for (const m of held) this._on(m);
    }, 0);
  }
  _on(m) {
    switch (m.type) {
      case 'data':
        for (const line of m.text.split('\n')) if (line) this.onmessage?.({ data: line });
        break;
      case 'status': this.onProgress?.(m); break;
      case 'ready': this.readyState = 1; this.onopen?.(); break;
      case 'fatal': this._fail(m.text); break;
    }
  }
  _fail(text) {
    console.error('[local sim]', text);
    this.onFatal?.(text);
    this.close();
  }
  send(text) { if (this.readyState === 1) this.worker.postMessage({ type: 'msg', text }); }
  close() {
    if (this.readyState === 3) return;
    this.readyState = 3;
    this.worker.terminate();
    this.onclose?.();
  }
}
