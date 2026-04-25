/**
 * WIA Haptic Standard - Mock Adapter
 *
 * Mock adapter for testing and development.
 * Simulates haptic device behavior without hardware.
 *
 * 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라
 */

import {
  BaseHapticDevice,
  IHapticDevice,
} from '../device';

import {
  HapticCapabilities,
  HapticPrimitive,
  BodyLocation,
  ConnectionOptions,
  WaveformType,
} from '../types';

// ============================================================================
// Mock Configuration
// ============================================================================

export interface MockAdapterConfig {
  /** Device ID */
  deviceId?: string;

  /** Device name */
  deviceName?: string;

  /** Simulated capabilities */
  capabilities?: Partial<HapticCapabilities>;

  /** Simulate connection latency (ms) */
  connectionLatency?: number;

  /** Simulate play latency (ms) */
  playLatency?: number;

  /** Log pattern plays to console */
  logPatterns?: boolean;

  /** Callback when pattern is played */
  onPatternPlayed?: (pattern: HapticPrimitive, location: BodyLocation) => void;

  /** Simulate errors */
  simulateErrors?: {
    connectionFailRate?: number;  // 0-1
    playFailRate?: number;        // 0-1
  };
}

// ============================================================================
// Mock Haptic Adapter
// ============================================================================

/**
 * Mock adapter for testing and development.
 *
 * Usage:
 * ```typescript
 * const mock = new MockHapticAdapter({
 *   logPatterns: true,
 *   onPatternPlayed: (pattern, location) => {
 *     console.log(`Pattern ${pattern.id} played at ${location}`);
 *   },
 * });
 * await mock.connect();
 * await mock.play(PRIMITIVES.TAP_MEDIUM);
 * ```
 */
export class MockHapticAdapter extends BaseHapticDevice implements IHapticDevice {
  readonly deviceId: string;
  readonly deviceName: string;

  private config: Required<MockAdapterConfig>;
  private playHistory: Array<{
    pattern: HapticPrimitive;
    location: BodyLocation;
    timestamp: number;
    intensityMultiplier: number;
  }> = [];

  private intensityState: Map<BodyLocation, number> = new Map();
  private frequencyState: Map<BodyLocation, number> = new Map();

  constructor(config: MockAdapterConfig = {}) {
    super();
    this.deviceId = config.deviceId ?? 'mock-device';
    this.deviceName = config.deviceName ?? 'Mock Haptic Device';
    this.config = {
      deviceId: this.deviceId,
      deviceName: this.deviceName,
      capabilities: config.capabilities ?? {},
      connectionLatency: config.connectionLatency ?? 100,
      playLatency: config.playLatency ?? 10,
      logPatterns: config.logPatterns ?? false,
      onPatternPlayed: config.onPatternPlayed ?? (() => {}),
      simulateErrors: config.simulateErrors ?? {},
    };
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Abstract Implementation
  // ─────────────────────────────────────────────────────────────────────────

  protected async doConnect(_options?: ConnectionOptions): Promise<void> {
    // Simulate connection latency
    await this.sleep(this.config.connectionLatency);

    // Simulate random connection failures
    if (this.config.simulateErrors.connectionFailRate) {
      if (Math.random() < this.config.simulateErrors.connectionFailRate) {
        throw new Error('Simulated connection failure');
      }
    }
  }

  protected async doDisconnect(): Promise<void> {
    await this.sleep(50);
  }

  protected doGetCapabilities(): HapticCapabilities {
    return {
      deviceId: this.deviceId,
      deviceName: this.deviceName,
      actuatorType: 'lra',
      frequencyRange: { min: 1, max: 300, optimal: 80 },
      locations: [
        'left_wrist_top',
        'right_wrist_top',
        'chest_left',
        'chest_center',
        'chest_right',
        'back_center_middle',
      ],
      channelCount: 6,
      maxIntensity: 1.0,
      latency: this.config.playLatency,
      supportedWaveforms: ['sine', 'square', 'triangle', 'sawtooth', 'noise'],
      supportsADSR: true,
      supportsCustomWaveform: true,
      minDuration: 1,
      batteryLevel: 85,
      ...this.config.capabilities,
    };
  }

  protected async doPlayPrimitive(
    primitive: HapticPrimitive,
    location: BodyLocation,
    intensityMultiplier: number
  ): Promise<void> {
    // Simulate play latency
    await this.sleep(this.config.playLatency);

    // Simulate random play failures
    if (this.config.simulateErrors.playFailRate) {
      if (Math.random() < this.config.simulateErrors.playFailRate) {
        throw new Error('Simulated play failure');
      }
    }

    // Record in history
    this.playHistory.push({
      pattern: primitive,
      location,
      timestamp: Date.now(),
      intensityMultiplier,
    });

    // Log if enabled
    if (this.config.logPatterns) {
      this.logPattern(primitive, location, intensityMultiplier);
    }

    // Call callback
    this.config.onPatternPlayed(primitive, location);

    // Simulate duration
    await this.sleep(primitive.duration);
  }

  protected doStop(): void {
    if (this.config.logPatterns) {
      console.log(`[MockHaptic] STOP`);
    }
  }

  protected doSetIntensity(location: BodyLocation, intensity: number): void {
    this.intensityState.set(location, intensity);
    if (this.config.logPatterns) {
      console.log(`[MockHaptic] Intensity: ${location} = ${(intensity * 100).toFixed(0)}%`);
    }
  }

  protected doSetFrequency(location: BodyLocation, frequency: number): void {
    this.frequencyState.set(location, frequency);
    if (this.config.logPatterns) {
      console.log(`[MockHaptic] Frequency: ${location} = ${frequency}Hz`);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Test Utilities
  // ─────────────────────────────────────────────────────────────────────────

  /**
   * Get the history of played patterns.
   */
  getPlayHistory(): typeof this.playHistory {
    return [...this.playHistory];
  }

  /**
   * Clear the play history.
   */
  clearHistory(): void {
    this.playHistory = [];
  }

  /**
   * Get current intensity state for a location.
   */
  getIntensityState(location: BodyLocation): number | undefined {
    return this.intensityState.get(location);
  }

  /**
   * Get current frequency state for a location.
   */
  getFrequencyState(location: BodyLocation): number | undefined {
    return this.frequencyState.get(location);
  }

  /**
   * Assert that a pattern was played.
   * Useful for testing.
   */
  assertPatternPlayed(patternId: string): boolean {
    return this.playHistory.some(h => h.pattern.id === patternId);
  }

  /**
   * Get count of pattern plays.
   */
  getPatternPlayCount(patternId: string): number {
    return this.playHistory.filter(h => h.pattern.id === patternId).length;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Private Helpers
  // ─────────────────────────────────────────────────────────────────────────

  private logPattern(
    primitive: HapticPrimitive,
    location: BodyLocation,
    intensityMultiplier: number
  ): void {
    const effectiveIntensity = primitive.intensity * intensityMultiplier;
    const bar = this.createVisualization(effectiveIntensity, primitive.waveform);

    console.log(
      `[MockHaptic] ${primitive.id} @ ${location}\n` +
      `  ${bar}\n` +
      `  ${primitive.waveform} ${primitive.frequency}Hz ` +
      `${(effectiveIntensity * 100).toFixed(0)}% ` +
      `${primitive.duration}ms`
    );
  }

  private createVisualization(intensity: number, waveform: WaveformType): string {
    const width = 40;
    const filled = Math.round(intensity * width);

    const chars: Record<WaveformType, string> = {
      sine: '~',
      square: '█',
      triangle: '▲',
      sawtooth: '/',
      noise: '░',
    };

    const char = chars[waveform];
    return `[${char.repeat(filled)}${' '.repeat(width - filled)}]`;
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * Create a mock haptic device for testing.
 */
export function createMockDevice(config?: MockAdapterConfig): MockHapticAdapter {
  return new MockHapticAdapter(config);
}

/**
 * Create a mock device with console logging enabled.
 */
export function createLoggingMockDevice(): MockHapticAdapter {
  return new MockHapticAdapter({ logPatterns: true });
}

/**
 * Create a mock device that simulates frequent failures.
 * Useful for testing error handling.
 */
export function createUnreliableMockDevice(): MockHapticAdapter {
  return new MockHapticAdapter({
    simulateErrors: {
      connectionFailRate: 0.2,
      playFailRate: 0.1,
    },
  });
}
