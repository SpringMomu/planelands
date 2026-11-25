// Generates the cloud noise volumes off the main thread (see cloud_noise.js).
import { shapeVolume, detailVolume } from './cloud_noise.js';

onmessage = () => {
  const shape = shapeVolume(128, 11);
  const detail = detailVolume(32, 23);
  postMessage({ shape, detail }, [shape.buffer, detail.buffer]);
};
