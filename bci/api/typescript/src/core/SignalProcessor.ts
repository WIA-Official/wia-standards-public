/**
 * WIA BCI Signal Processor
 * @module wia-bci/core/SignalProcessor
 */

import type { BandPowers, FrequencyBand, PowerSpectrum, FREQUENCY_BANDS } from '../types';

/**
 * Frequency band ranges in Hz
 */
const BANDS: Record<string, [number, number]> = {
  delta: [0.5, 4],
  theta: [4, 8],
  alpha: [8, 13],
  beta: [13, 30],
  gamma: [30, 100],
};

/**
 * Signal Processing Utilities
 */
export class SignalProcessor {
  /**
   * Apply high-pass filter
   * Uses simple first-order IIR filter
   */
  static highpass(data: Float32Array, cutoff: number, fs: number): Float32Array {
    const result = new Float32Array(data.length);
    const rc = 1 / (2 * Math.PI * cutoff);
    const dt = 1 / fs;
    const alpha = rc / (rc + dt);

    result[0] = data[0];
    for (let i = 1; i < data.length; i++) {
      result[i] = alpha * (result[i - 1] + data[i] - data[i - 1]);
    }

    return result;
  }

  /**
   * Apply low-pass filter
   * Uses simple first-order IIR filter
   */
  static lowpass(data: Float32Array, cutoff: number, fs: number): Float32Array {
    const result = new Float32Array(data.length);
    const rc = 1 / (2 * Math.PI * cutoff);
    const dt = 1 / fs;
    const alpha = dt / (rc + dt);

    result[0] = data[0];
    for (let i = 1; i < data.length; i++) {
      result[i] = result[i - 1] + alpha * (data[i] - result[i - 1]);
    }

    return result;
  }

  /**
   * Apply band-pass filter
   */
  static bandpass(data: Float32Array, low: number, high: number, fs: number): Float32Array {
    const hp = this.highpass(data, low, fs);
    return this.lowpass(hp, high, fs);
  }

  /**
   * Apply notch filter (e.g., for 50/60 Hz line noise)
   */
  static notch(data: Float32Array, freq: number, fs: number, Q = 30): Float32Array {
    const result = new Float32Array(data.length);
    const w0 = (2 * Math.PI * freq) / fs;
    const alpha = Math.sin(w0) / (2 * Q);

    const b0 = 1;
    const b1 = -2 * Math.cos(w0);
    const b2 = 1;
    const a0 = 1 + alpha;
    const a1 = -2 * Math.cos(w0);
    const a2 = 1 - alpha;

    // Normalize
    const nb0 = b0 / a0;
    const nb1 = b1 / a0;
    const nb2 = b2 / a0;
    const na1 = a1 / a0;
    const na2 = a2 / a0;

    // Apply biquad filter
    let x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    for (let i = 0; i < data.length; i++) {
      const x = data[i];
      const y = nb0 * x + nb1 * x1 + nb2 * x2 - na1 * y1 - na2 * y2;

      x2 = x1;
      x1 = x;
      y2 = y1;
      y1 = y;

      result[i] = y;
    }

    return result;
  }

  /**
   * Compute Power Spectral Density using FFT
   */
  static psd(data: Float32Array, fs: number): PowerSpectrum {
    const n = data.length;
    const nfft = Math.pow(2, Math.ceil(Math.log2(n)));

    // Zero-pad and apply Hanning window
    const windowed = new Float32Array(nfft);
    for (let i = 0; i < n; i++) {
      const window = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (n - 1)));
      windowed[i] = data[i] * window;
    }

    // Simple DFT (for production, use proper FFT library)
    const powers = new Float32Array(nfft / 2);
    const frequencies = new Float32Array(nfft / 2);

    for (let k = 0; k < nfft / 2; k++) {
      let real = 0;
      let imag = 0;

      for (let n2 = 0; n2 < nfft; n2++) {
        const angle = (2 * Math.PI * k * n2) / nfft;
        real += windowed[n2] * Math.cos(angle);
        imag -= windowed[n2] * Math.sin(angle);
      }

      powers[k] = (real * real + imag * imag) / nfft;
      frequencies[k] = (k * fs) / nfft;
    }

    return {
      frequencies,
      powers,
      resolution: fs / nfft,
    };
  }

  /**
   * Calculate power in a frequency band
   */
  static bandpower(data: Float32Array, fs: number, band: FrequencyBand): number {
    const [low, high] = typeof band === 'string' ? BANDS[band] : band;

    const spectrum = this.psd(data, fs);
    let power = 0;
    let count = 0;

    for (let i = 0; i < spectrum.frequencies.length; i++) {
      const freq = spectrum.frequencies[i];
      if (freq >= low && freq <= high) {
        power += spectrum.powers[i];
        count++;
      }
    }

    return count > 0 ? power / count : 0;
  }

  /**
   * Calculate all standard frequency band powers
   */
  static allBandPowers(data: Float32Array, fs: number): BandPowers {
    return {
      delta: this.bandpower(data, fs, 'delta'),
      theta: this.bandpower(data, fs, 'theta'),
      alpha: this.bandpower(data, fs, 'alpha'),
      beta: this.bandpower(data, fs, 'beta'),
      gamma: this.bandpower(data, fs, 'gamma'),
    };
  }

  /**
   * Extract epoch from continuous data
   */
  static epoch(data: Float32Array, start: number, end: number): Float32Array {
    const length = end - start;
    const result = new Float32Array(length);

    for (let i = 0; i < length; i++) {
      result[i] = data[start + i] ?? 0;
    }

    return result;
  }

  /**
   * Generate sliding windows
   */
  static sliding(data: Float32Array, windowSize: number, step: number): Float32Array[] {
    const windows: Float32Array[] = [];

    for (let start = 0; start + windowSize <= data.length; start += step) {
      windows.push(this.epoch(data, start, start + windowSize));
    }

    return windows;
  }

  /**
   * Detect eye blinks in EOG channel
   */
  static detectBlinks(eog: Float32Array, threshold = 100): number[] {
    const blinks: number[] = [];
    let inBlink = false;

    for (let i = 0; i < eog.length; i++) {
      if (Math.abs(eog[i]) > threshold && !inBlink) {
        blinks.push(i);
        inBlink = true;
      } else if (Math.abs(eog[i]) < threshold / 2) {
        inBlink = false;
      }
    }

    return blinks;
  }

  /**
   * Calculate RMS (Root Mean Square)
   */
  static rms(data: Float32Array): number {
    let sum = 0;
    for (let i = 0; i < data.length; i++) {
      sum += data[i] * data[i];
    }
    return Math.sqrt(sum / data.length);
  }

  /**
   * Calculate mean
   */
  static mean(data: Float32Array): number {
    let sum = 0;
    for (let i = 0; i < data.length; i++) {
      sum += data[i];
    }
    return sum / data.length;
  }

  /**
   * Calculate variance
   */
  static variance(data: Float32Array): number {
    const m = this.mean(data);
    let sum = 0;
    for (let i = 0; i < data.length; i++) {
      const diff = data[i] - m;
      sum += diff * diff;
    }
    return sum / data.length;
  }

  /**
   * Count zero crossings
   */
  static zeroCrossings(data: Float32Array): number {
    let count = 0;
    for (let i = 1; i < data.length; i++) {
      if ((data[i] >= 0 && data[i - 1] < 0) || (data[i] < 0 && data[i - 1] >= 0)) {
        count++;
      }
    }
    return count;
  }
}
