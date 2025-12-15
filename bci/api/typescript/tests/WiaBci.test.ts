/**
 * WiaBci Tests
 */

import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { WiaBci, BciError, SignalProcessor } from '../src';

describe('WiaBci', () => {
  let bci: WiaBci;

  beforeEach(() => {
    bci = new WiaBci({ logLevel: 'error' });
  });

  afterEach(() => {
    bci.dispose();
  });

  describe('connection', () => {
    it('should connect to simulator', async () => {
      await bci.connect({ type: 'simulator' });
      expect(bci.isConnected()).toBe(true);
    });

    it('should throw when connecting twice', async () => {
      await bci.connect({ type: 'simulator' });

      await expect(bci.connect({ type: 'simulator' })).rejects.toThrow(BciError);
    });

    it('should disconnect', async () => {
      await bci.connect({ type: 'simulator' });
      await bci.disconnect();

      expect(bci.isConnected()).toBe(false);
    });

    it('should throw when disconnecting without connection', async () => {
      await expect(bci.disconnect()).rejects.toThrow(BciError);
    });
  });

  describe('streaming', () => {
    it('should start and stop stream', async () => {
      await bci.connect({ type: 'simulator' });

      await bci.startStream();
      expect(bci.isStreaming()).toBe(true);

      await bci.stopStream();
      expect(bci.isStreaming()).toBe(false);
    });

    it('should emit signal events', async () => {
      await bci.connect({ type: 'simulator' });

      const signals: unknown[] = [];
      bci.on('signal', (event) => {
        signals.push(event);
      });

      await bci.startStream();

      // Wait for some samples
      await new Promise((resolve) => setTimeout(resolve, 100));

      await bci.stopStream();

      expect(signals.length).toBeGreaterThan(0);
    });
  });

  describe('device info', () => {
    it('should return device info when connected', async () => {
      await bci.connect({ type: 'simulator' });

      const info = bci.getDeviceInfo();
      expect(info).not.toBeNull();
      expect(info?.type).toBe('simulator');
    });

    it('should return channels when connected', async () => {
      await bci.connect({ type: 'simulator' });

      const channels = bci.getChannels();
      expect(channels.length).toBeGreaterThan(0);
    });

    it('should list available devices', async () => {
      const devices = await bci.listDevices();
      expect(devices.length).toBeGreaterThan(0);
      expect(devices[0].type).toBe('simulator');
    });
  });
});

describe('SignalProcessor', () => {
  const createTestSignal = (length: number, freq: number, fs: number): Float32Array => {
    const data = new Float32Array(length);
    for (let i = 0; i < length; i++) {
      data[i] = Math.sin((2 * Math.PI * freq * i) / fs);
    }
    return data;
  };

  describe('filters', () => {
    it('should apply highpass filter', () => {
      const data = createTestSignal(250, 10, 250);
      const filtered = SignalProcessor.highpass(data, 5, 250);

      expect(filtered.length).toBe(data.length);
    });

    it('should apply lowpass filter', () => {
      const data = createTestSignal(250, 10, 250);
      const filtered = SignalProcessor.lowpass(data, 15, 250);

      expect(filtered.length).toBe(data.length);
    });

    it('should apply bandpass filter', () => {
      const data = createTestSignal(250, 10, 250);
      const filtered = SignalProcessor.bandpass(data, 8, 13, 250);

      expect(filtered.length).toBe(data.length);
    });
  });

  describe('band powers', () => {
    it('should calculate all band powers', () => {
      // Create signal with alpha (10 Hz)
      const data = createTestSignal(500, 10, 250);
      const powers = SignalProcessor.allBandPowers(data, 250);

      expect(powers.alpha).toBeGreaterThan(0);
      expect(typeof powers.delta).toBe('number');
      expect(typeof powers.theta).toBe('number');
      expect(typeof powers.beta).toBe('number');
      expect(typeof powers.gamma).toBe('number');
    });
  });

  describe('statistics', () => {
    it('should calculate mean', () => {
      const data = new Float32Array([1, 2, 3, 4, 5]);
      expect(SignalProcessor.mean(data)).toBe(3);
    });

    it('should calculate variance', () => {
      const data = new Float32Array([1, 2, 3, 4, 5]);
      expect(SignalProcessor.variance(data)).toBe(2);
    });

    it('should count zero crossings', () => {
      const data = new Float32Array([-1, 1, -1, 1, -1]);
      expect(SignalProcessor.zeroCrossings(data)).toBe(4);
    });
  });

  describe('windowing', () => {
    it('should extract epoch', () => {
      const data = new Float32Array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]);
      const epoch = SignalProcessor.epoch(data, 2, 5);

      expect(epoch.length).toBe(3);
      expect(epoch[0]).toBe(2);
      expect(epoch[2]).toBe(4);
    });

    it('should generate sliding windows', () => {
      const data = new Float32Array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]);
      const windows = SignalProcessor.sliding(data, 3, 2);

      expect(windows.length).toBe(4);
      expect(windows[0][0]).toBe(0);
      expect(windows[1][0]).toBe(2);
    });
  });
});
