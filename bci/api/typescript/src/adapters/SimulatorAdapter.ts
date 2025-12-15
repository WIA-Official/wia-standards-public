/**
 * WIA BCI Simulator Adapter
 * Generates synthetic EEG data for testing and development
 * @module wia-bci/adapters/SimulatorAdapter
 */

import { BaseAdapter } from './BaseAdapter';
import type {
  DeviceType,
  DeviceConfig,
  DeviceInfo,
  ChannelInfo,
  SignalEvent,
} from '../types';

/**
 * Simulator configuration options
 */
interface SimulatorOptions {
  /** Number of channels */
  channels?: number;
  /** Sampling rate in Hz */
  samplingRate?: number;
  /** Add alpha waves (8-13 Hz) */
  alpha?: boolean;
  /** Add noise */
  noise?: boolean;
  /** Noise amplitude in µV */
  noiseAmplitude?: number;
}

/**
 * Simulator Adapter
 * Generates realistic synthetic EEG signals for development and testing
 */
export class SimulatorAdapter extends BaseAdapter {
  readonly type: DeviceType = 'simulator';
  readonly name = 'WIA BCI Simulator';

  private options: Required<SimulatorOptions>;
  private intervalId?: ReturnType<typeof setInterval>;
  private sampleIndex = 0;
  private startTime = 0;

  constructor(options?: SimulatorOptions) {
    super();
    this.options = {
      channels: options?.channels ?? 8,
      samplingRate: options?.samplingRate ?? 250,
      alpha: options?.alpha ?? true,
      noise: options?.noise ?? true,
      noiseAmplitude: options?.noiseAmplitude ?? 10,
    };
  }

  async connect(config: DeviceConfig): Promise<void> {
    // Apply config overrides
    if (config.acquisition?.samplingRate) {
      this.options.samplingRate = config.acquisition.samplingRate;
    }
    if (config.acquisition?.channels) {
      this.options.channels = config.acquisition.channels.length;
    }

    // Create device info
    this.deviceInfo = {
      id: 'simulator-' + Date.now(),
      name: 'WIA BCI Simulator',
      type: 'simulator',
      manufacturer: 'WIA',
      model: 'Simulator v1.0',
      firmwareVersion: '1.0.0',
      capabilities: {
        channels: this.options.channels,
        maxSamplingRate: 250,
        supportedSamplingRates: [125, 250],
        resolution: 24,
        hasAccelerometer: false,
        hasGyroscope: false,
        hasImpedanceCheck: false,
        hasBatteryIndicator: false,
        supportedProtocols: [],
      },
      status: 'connected',
    };

    // Create channel info
    const channelLabels = ['Fp1', 'Fp2', 'F3', 'F4', 'C3', 'C4', 'O1', 'O2'];
    this.channels = Array.from({ length: this.options.channels }, (_, i) => ({
      index: i,
      label: channelLabels[i] ?? `CH${i + 1}`,
      type: 'eeg' as const,
      unit: 'µV',
      samplingRate: this.options.samplingRate,
      enabled: true,
    }));

    this._connected = true;
  }

  async disconnect(): Promise<void> {
    if (this._streaming) {
      await this.stopStream();
    }
    this._connected = false;
    this.deviceInfo = null;
    this.channels = [];
  }

  async startStream(): Promise<void> {
    if (!this._connected) {
      throw new Error('Not connected');
    }
    if (this._streaming) {
      return;
    }

    this.sampleIndex = 0;
    this.startTime = Date.now();
    this._streaming = true;

    // Calculate interval for sample generation
    const intervalMs = 1000 / this.options.samplingRate;

    this.intervalId = setInterval(() => {
      this.generateSample();
    }, intervalMs);
  }

  async stopStream(): Promise<void> {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = undefined;
    }
    this._streaming = false;
  }

  /**
   * Generate a synthetic EEG sample
   */
  private generateSample(): void {
    const timestamp = Date.now();
    const t = (timestamp - this.startTime) / 1000; // Time in seconds

    const data = new Float32Array(this.options.channels);

    for (let ch = 0; ch < this.options.channels; ch++) {
      let value = 0;

      // Add alpha rhythm (10 Hz sine wave, ~50 µV amplitude)
      if (this.options.alpha) {
        // Vary alpha by channel (stronger in occipital)
        const alphaAmplitude = ch >= 6 ? 50 : 25; // O1, O2 stronger
        value += alphaAmplitude * Math.sin(2 * Math.PI * 10 * t + ch * 0.5);
      }

      // Add some theta (6 Hz)
      value += 15 * Math.sin(2 * Math.PI * 6 * t + ch * 0.3);

      // Add beta (20 Hz, smaller amplitude)
      value += 8 * Math.sin(2 * Math.PI * 20 * t + ch * 0.7);

      // Add random noise
      if (this.options.noise) {
        value += (Math.random() - 0.5) * 2 * this.options.noiseAmplitude;
      }

      data[ch] = value;
    }

    const event: SignalEvent = {
      timestamp,
      sampleIndex: this.sampleIndex++,
      channels: Array.from({ length: this.options.channels }, (_, i) => i),
      data,
    };

    this.emitData(event);
  }
}
