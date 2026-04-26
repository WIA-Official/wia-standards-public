/**
 * WIA-SEMI-002 Memory Semiconductor TypeScript SDK
 * @version 1.0.0
 */

import { MemoryType, MemorySpec, DRAMTiming, BandwidthCalculation } from './types';

export class MemoryCalculator {
  static calculateBandwidth(dataRate: number, busWidth: number, channels: number = 1): BandwidthCalculation {
    const singleChannelBandwidth = (dataRate * busWidth) / 8 / 1000;
    const theoreticalBandwidth = singleChannelBandwidth * channels;
    const effectiveBandwidth = theoreticalBandwidth * 0.85;

    return { dataRate, busWidth, channels, theoreticalBandwidth, effectiveBandwidth };
  }

  static calculateLatency(timing: DRAMTiming, clockFrequency: number): number {
    const totalCycles = timing.tRCD + timing.tCL;
    return (totalCycles * 1000) / clockFrequency;
  }
}

export * from './types';
