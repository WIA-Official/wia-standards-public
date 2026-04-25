/**
 * WIA Haptic Standard - Web Vibration API Adapter
 *
 * Adapter for the standard Web Vibration API.
 * Limited capabilities but works in most browsers.
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
  HapticError,
} from '../types';

/**
 * Adapter for the Web Vibration API.
 *
 * Limitations:
 * - Single channel only
 * - No frequency control
 * - No waveform control
 * - Only on/off patterns
 *
 * The adapter converts WIA patterns to simple on/off sequences.
 */
export class WebHapticAdapter extends BaseHapticDevice implements IHapticDevice {
  readonly deviceId = 'web-vibration';
  readonly deviceName = 'Web Vibration API';

  private _navigator: Navigator | null = null;

  constructor() {
    super();
  }

  /**
   * Check if Web Vibration API is available.
   */
  static isSupported(): boolean {
    return typeof navigator !== 'undefined' && 'vibrate' in navigator;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Abstract Implementation
  // ─────────────────────────────────────────────────────────────────────────

  protected async doConnect(options?: ConnectionOptions): Promise<void> {
    if (!WebHapticAdapter.isSupported()) {
      throw new HapticError(
        'Web Vibration API not supported',
        'DEVICE_NOT_FOUND',
        this.deviceId
      );
    }

    this._navigator = navigator;

    // Test vibration
    try {
      this._navigator.vibrate(1);
    } catch (e) {
      throw new HapticError(
        'Vibration permission denied',
        'PERMISSION_DENIED',
        this.deviceId
      );
    }
  }

  protected async doDisconnect(): Promise<void> {
    this._navigator?.vibrate(0);
    this._navigator = null;
  }

  protected doGetCapabilities(): HapticCapabilities {
    return {
      deviceId: this.deviceId,
      deviceName: this.deviceName,
      actuatorType: 'erm',
      frequencyRange: { min: 1, max: 1, optimal: 1 }, // No frequency control
      locations: ['left_wrist_top'], // Single location
      channelCount: 1,
      maxIntensity: 1.0,
      latency: 50,
      supportedWaveforms: ['square'], // Only on/off
      supportsADSR: false,
      supportsCustomWaveform: false,
      minDuration: 10,
      batteryLevel: null,
    };
  }

  protected async doPlayPrimitive(
    primitive: HapticPrimitive,
    _location: BodyLocation,
    intensityMultiplier: number
  ): Promise<void> {
    if (!this._navigator) return;

    // Convert WIA primitive to Web Vibration pattern
    // Web API only supports on/off, so we simulate with pulse width
    const pattern = this.convertToVibrationPattern(primitive, intensityMultiplier);
    this._navigator.vibrate(pattern);

    // Wait for pattern to complete
    const totalDuration = Array.isArray(pattern)
      ? pattern.reduce((a, b) => a + b, 0)
      : pattern;
    await this.sleep(totalDuration);
  }

  protected doStop(): void {
    this._navigator?.vibrate(0);
  }

  protected doSetIntensity(_location: BodyLocation, _intensity: number): void {
    // Web API doesn't support intensity control
    // Ignore silently
  }

  protected doSetFrequency(_location: BodyLocation, _frequency: number): void {
    // Web API doesn't support frequency control
    // Ignore silently
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Pattern Conversion
  // ─────────────────────────────────────────────────────────────────────────

  /**
   * Convert WIA primitive to Web Vibration API pattern.
   *
   * Since Web API only supports on/off:
   * - Intensity is simulated via PWM (on/off cycling)
   * - Waveform is ignored
   * - Frequency is ignored
   */
  private convertToVibrationPattern(
    primitive: HapticPrimitive,
    intensityMultiplier: number
  ): number | number[] {
    const effectiveIntensity = Math.min(1, primitive.intensity * intensityMultiplier);
    const { attack, decay, sustain, release } = primitive.envelope;
    const totalDuration = primitive.duration;

    // For very short durations, just vibrate for the full duration
    if (totalDuration < 50) {
      return Math.round(totalDuration * effectiveIntensity);
    }

    // Build PWM pattern based on envelope
    const pattern: number[] = [];
    const cycleTime = 20; // 20ms cycles for PWM

    // Attack phase
    if (attack > 0) {
      const attackCycles = Math.max(1, Math.floor(attack / cycleTime));
      for (let i = 0; i < attackCycles; i++) {
        const intensity = (i / attackCycles) * effectiveIntensity;
        const onTime = Math.round(cycleTime * intensity);
        const offTime = cycleTime - onTime;
        if (onTime > 0) pattern.push(onTime);
        if (offTime > 0) pattern.push(offTime);
      }
    }

    // Sustain phase
    const sustainTime = totalDuration - attack - release;
    if (sustainTime > 0) {
      const sustainIntensity = sustain * effectiveIntensity;
      if (sustainIntensity > 0.9) {
        // High intensity - continuous vibration
        pattern.push(sustainTime);
      } else {
        // Lower intensity - PWM
        const sustainCycles = Math.max(1, Math.floor(sustainTime / cycleTime));
        for (let i = 0; i < sustainCycles; i++) {
          const onTime = Math.round(cycleTime * sustainIntensity);
          const offTime = cycleTime - onTime;
          if (onTime > 0) pattern.push(onTime);
          if (offTime > 0) pattern.push(offTime);
        }
      }
    }

    // Release phase
    if (release > 0) {
      const releaseCycles = Math.max(1, Math.floor(release / cycleTime));
      for (let i = 0; i < releaseCycles; i++) {
        const intensity = (1 - i / releaseCycles) * sustain * effectiveIntensity;
        const onTime = Math.round(cycleTime * intensity);
        const offTime = cycleTime - onTime;
        if (onTime > 0) pattern.push(onTime);
        if (offTime > 0) pattern.push(offTime);
      }
    }

    // Simplify pattern if too complex
    if (pattern.length > 100) {
      return this.simplifyPattern(pattern, 50);
    }

    return pattern.length === 1 ? pattern[0] : pattern;
  }

  /**
   * Simplify a long pattern by merging adjacent values.
   */
  private simplifyPattern(pattern: number[], maxLength: number): number[] {
    if (pattern.length <= maxLength) return pattern;

    const ratio = Math.ceil(pattern.length / maxLength);
    const result: number[] = [];

    for (let i = 0; i < pattern.length; i += ratio) {
      let sum = 0;
      for (let j = 0; j < ratio && i + j < pattern.length; j++) {
        sum += pattern[i + j];
      }
      result.push(sum);
    }

    return result;
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a Web Vibration API adapter.
 * @throws HapticError if not supported
 */
export function createWebHapticDevice(): WebHapticAdapter {
  if (!WebHapticAdapter.isSupported()) {
    throw new HapticError(
      'Web Vibration API not supported in this environment',
      'DEVICE_NOT_FOUND'
    );
  }
  return new WebHapticAdapter();
}
