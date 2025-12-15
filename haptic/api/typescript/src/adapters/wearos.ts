/**
 * WIA Haptic Standard - Wear OS Adapter
 * @version 1.0.0
 *
 * Adapter for Android Wear OS devices using Vibration API
 */

import { BodyLocation } from '../types';
import {
  BaseHapticDevice,
  HapticCapabilities,
  DeviceConfig,
  HapticError,
} from '../device';
import {
  HapticPattern,
  HapticPrimitive,
  HapticSequence,
  resolveEnvelope,
} from '../patterns';

// ============================================================================
// Android Vibration Types
// ============================================================================

interface VibrationEffect {
  type: 'oneShot' | 'waveform' | 'composition';
}

interface OneShotEffect extends VibrationEffect {
  type: 'oneShot';
  duration: number;      // milliseconds
  amplitude: number;     // 0-255 or -1 for default
}

interface WaveformEffect extends VibrationEffect {
  type: 'waveform';
  timings: number[];     // durations in ms
  amplitudes: number[];  // 0-255 for each segment
  repeat: number;        // -1 for no repeat, index to repeat from
}

interface CompositionEffect extends VibrationEffect {
  type: 'composition';
  primitives: AndroidHapticPrimitive[];
}

interface AndroidHapticPrimitive {
  primitiveId: number;   // Android primitive constant
  scale: number;         // 0.0-1.0
  delay: number;         // ms delay before this primitive
}

// Android Haptic Primitive IDs
const ANDROID_PRIMITIVES = {
  CLICK: 0,
  THUD: 1,
  SPIN: 2,
  QUICK_RISE: 3,
  SLOW_RISE: 4,
  QUICK_FALL: 5,
  TICK: 7,
  LOW_TICK: 8,
  TEXTURE_TICK: 21,
} as const;

// ============================================================================
// Wear OS Bridge Interface
// ============================================================================

interface WearOSBridge {
  isConnected(): Promise<boolean>;
  getDeviceInfo(): Promise<{ model: string; apiLevel: number }>;
  vibrate(effect: VibrationEffect): Promise<void>;
  cancel(): Promise<void>;
  hasAmplitudeControl(): Promise<boolean>;
  getBatteryLevel(): Promise<number>;
}

declare const WearOS: WearOSBridge;

// ============================================================================
// Wear OS Haptic Adapter
// ============================================================================

export class WearOSHapticAdapter extends BaseHapticDevice {
  private bridge: WearOSBridge | null = null;
  private hasAmplitudeControl: boolean = false;
  private apiLevel: number = 0;

  constructor(config: DeviceConfig = {}) {
    super('smartwatch', {
      name: 'Wear OS Device',
      ...config,
    });
  }

  // -------------------------------------------------------------------------
  // Connection
  // -------------------------------------------------------------------------

  async connect(): Promise<void> {
    this._state = 'connecting';

    try {
      if (typeof WearOS === 'undefined') {
        throw new HapticError(
          'WearOS bridge not available',
          'DEVICE_NOT_FOUND'
        );
      }

      this.bridge = WearOS;

      const isConnected = await Promise.race([
        this.bridge.isConnected(),
        new Promise<boolean>((_, reject) =>
          setTimeout(() => reject(new Error('Timeout')), this._config.connectionTimeout)
        ),
      ]);

      if (!isConnected) {
        throw new HapticError(
          'Wear OS device not connected',
          'CONNECTION_FAILED'
        );
      }

      const deviceInfo = await this.bridge.getDeviceInfo();
      this.apiLevel = deviceInfo.apiLevel;
      this.hasAmplitudeControl = await this.bridge.hasAmplitudeControl();

      this._capabilities = await this.queryCapabilities();
      this._state = 'connected';

      this.emit({
        type: 'connected',
        timestamp: Date.now(),
        data: deviceInfo,
      });
    } catch (error) {
      this._state = 'error';
      this.emit({
        type: 'error',
        timestamp: Date.now(),
        data: error,
      });
      throw error;
    }
  }

  async disconnect(): Promise<void> {
    this._state = 'disconnecting';
    await this.bridge?.cancel();
    this.bridge = null;
    this._capabilities = null;
    this._state = 'disconnected';

    this.emit({
      type: 'disconnected',
      timestamp: Date.now(),
    });
  }

  private async queryCapabilities(): Promise<HapticCapabilities> {
    const batteryLevel = await this.bridge?.getBatteryLevel() ?? null;

    return {
      actuatorType: 'lra',
      frequencyRange: { min: 1, max: 300 },
      locations: ['wrist_left_dorsal', 'wrist_right_dorsal'],
      maxIntensity: 1.0,
      latency: 20,
      actuatorCount: 1,
      supportsCustomWaveforms: this.apiLevel >= 26,
      supportsAmplitudeModulation: this.hasAmplitudeControl,
      supportsFrequencyModulation: false, // Android doesn't support frequency control
      batteryLevel,
    };
  }

  // -------------------------------------------------------------------------
  // Pattern Conversion
  // -------------------------------------------------------------------------

  private convertToAndroid(pattern: HapticPattern): VibrationEffect {
    if ('waveform' in pattern) {
      return this.convertPrimitive(pattern as HapticPrimitive);
    } else if ('steps' in pattern) {
      return this.convertSequence(pattern as HapticSequence);
    }

    // Spatial pattern - use first actuation
    const spatial = pattern as any;
    if (spatial.actuations?.[0]?.pattern) {
      return this.convertToAndroid(spatial.actuations[0].pattern);
    }

    return { type: 'oneShot', duration: 50, amplitude: 128 } as OneShotEffect;
  }

  private convertPrimitive(primitive: HapticPrimitive): VibrationEffect {
    const intensity = this.scaleIntensity(primitive.intensity);
    const amplitude = this.hasAmplitudeControl
      ? Math.round(intensity * 255)
      : -1; // Default amplitude

    // For short durations, use composition with Android primitives
    if (this.apiLevel >= 30 && primitive.duration < 50) {
      return this.createCompositionEffect(primitive);
    }

    // For longer durations, use waveform with envelope
    if (this.hasAmplitudeControl) {
      return this.createEnvelopedWaveform(primitive);
    }

    // Fallback to simple oneShot
    return {
      type: 'oneShot',
      duration: primitive.duration,
      amplitude,
    } as OneShotEffect;
  }

  private createCompositionEffect(primitive: HapticPrimitive): CompositionEffect {
    const intensity = this.scaleIntensity(primitive.intensity);

    // Map WIA primitive characteristics to Android primitives
    let primitiveId = ANDROID_PRIMITIVES.TICK;

    if (primitive.intensity > 0.7) {
      primitiveId = ANDROID_PRIMITIVES.CLICK;
    } else if (primitive.frequency < 100) {
      primitiveId = ANDROID_PRIMITIVES.THUD;
    } else if (primitive.frequency > 200) {
      primitiveId = ANDROID_PRIMITIVES.TEXTURE_TICK;
    }

    return {
      type: 'composition',
      primitives: [{
        primitiveId,
        scale: intensity,
        delay: 0,
      }],
    };
  }

  private createEnvelopedWaveform(primitive: HapticPrimitive): WaveformEffect {
    const envelope = resolveEnvelope(primitive.envelope);
    const intensity = this.scaleIntensity(primitive.intensity);
    const maxAmplitude = Math.round(intensity * 255);

    const timings: number[] = [];
    const amplitudes: number[] = [];

    // Attack phase
    if (envelope.attack > 0) {
      const steps = Math.ceil(envelope.attack / 10);
      for (let i = 0; i < steps; i++) {
        timings.push(envelope.attack / steps);
        amplitudes.push(Math.round(maxAmplitude * (i / steps)));
      }
    }

    // Decay phase
    if (envelope.decay > 0) {
      const sustainAmplitude = Math.round(maxAmplitude * envelope.sustain);
      const steps = Math.ceil(envelope.decay / 10);
      for (let i = 0; i < steps; i++) {
        timings.push(envelope.decay / steps);
        const progress = i / steps;
        amplitudes.push(Math.round(maxAmplitude - (maxAmplitude - sustainAmplitude) * progress));
      }
    }

    // Sustain phase
    const sustainDuration = primitive.duration - envelope.attack - envelope.decay - envelope.release;
    if (sustainDuration > 0) {
      timings.push(sustainDuration);
      amplitudes.push(Math.round(maxAmplitude * envelope.sustain));
    }

    // Release phase
    if (envelope.release > 0) {
      const sustainAmplitude = Math.round(maxAmplitude * envelope.sustain);
      const steps = Math.ceil(envelope.release / 10);
      for (let i = 0; i < steps; i++) {
        timings.push(envelope.release / steps);
        amplitudes.push(Math.round(sustainAmplitude * (1 - i / steps)));
      }
    }

    return {
      type: 'waveform',
      timings,
      amplitudes,
      repeat: -1,
    };
  }

  private convertSequence(sequence: HapticSequence): WaveformEffect {
    const timings: number[] = [];
    const amplitudes: number[] = [];

    for (const step of sequence.steps) {
      // Add delay (as silence)
      if (step.delayBefore && step.delayBefore > 0) {
        timings.push(step.delayBefore);
        amplitudes.push(0);
      }

      const repeatCount = step.repeatCount ?? 1;
      for (let i = 0; i < repeatCount; i++) {
        if (step.primitive) {
          const intensity = this.scaleIntensity(step.primitive.intensity);
          const scale = step.intensityScale ?? 1;
          const amplitude = Math.round(intensity * scale * 255);

          timings.push(step.primitive.duration);
          amplitudes.push(Math.min(255, amplitude));

          // Add repeat delay
          if (i < repeatCount - 1 && step.repeatDelay) {
            timings.push(step.repeatDelay);
            amplitudes.push(0);
          }
        }
      }
    }

    return {
      type: 'waveform',
      timings,
      amplitudes,
      repeat: sequence.loop ? 0 : -1,
    };
  }

  // -------------------------------------------------------------------------
  // Playback
  // -------------------------------------------------------------------------

  async play(pattern: HapticPattern): Promise<void> {
    this.assertConnected();

    const effect = this.convertToAndroid(pattern);
    await this.bridge!.vibrate(effect);

    this._isPlaying = true;

    const duration = this.estimateDuration(pattern);
    setTimeout(() => {
      this._isPlaying = false;
      this.emit({
        type: 'pattern_complete',
        timestamp: Date.now(),
      });
    }, duration);
  }

  stop(): void {
    this.bridge?.cancel();
    this._isPlaying = false;
  }

  // -------------------------------------------------------------------------
  // Real-time Control
  // -------------------------------------------------------------------------

  setIntensity(location: BodyLocation, intensity: number): void {
    // Android doesn't support real-time intensity changes
    // Would need to restart vibration with new amplitude
  }

  pulse(location: BodyLocation, duration: number, intensity: number = 0.5): void {
    this.assertConnected();

    const amplitude = this.hasAmplitudeControl
      ? Math.round(this.scaleIntensity(intensity) * 255)
      : -1;

    this.bridge!.vibrate({
      type: 'oneShot',
      duration,
      amplitude,
    } as OneShotEffect);
  }

  vibrate(location: BodyLocation, frequency: number, intensity: number): void {
    this.assertConnected();

    // Android doesn't support frequency control
    // Simulate with on-off pattern
    const amplitude = this.hasAmplitudeControl
      ? Math.round(this.scaleIntensity(intensity) * 255)
      : -1;

    const period = Math.round(1000 / frequency);
    const onTime = Math.round(period / 2);
    const offTime = period - onTime;

    this.bridge!.vibrate({
      type: 'waveform',
      timings: [onTime, offTime],
      amplitudes: [amplitude, 0],
      repeat: 0,
    } as WaveformEffect);
  }

  // -------------------------------------------------------------------------
  // Utility
  // -------------------------------------------------------------------------

  private estimateDuration(pattern: HapticPattern): number {
    if ('duration' in pattern) {
      return (pattern as HapticPrimitive).duration;
    }
    if ('steps' in pattern) {
      const seq = pattern as HapticSequence;
      let duration = 0;
      for (const step of seq.steps) {
        duration += step.delayBefore ?? 0;
        if (step.primitive) {
          duration += step.primitive.duration * (step.repeatCount ?? 1);
          duration += (step.repeatDelay ?? 0) * ((step.repeatCount ?? 1) - 1);
        }
      }
      return duration;
    }
    return 100;
  }
}
