/**
 * WIA Haptic Standard - Apple Watch Adapter
 * @version 1.0.0
 *
 * Adapter for Apple Watch using Core Haptics via WatchConnectivity
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
  resolveWaveform,
} from '../patterns';

// ============================================================================
// Core Haptics Types (Apple-specific)
// ============================================================================

interface CHHapticEvent {
  eventType: 'hapticTransient' | 'hapticContinuous' | 'audioContinuous' | 'audioCustom';
  time: number;
  duration?: number;
  parameters: CHHapticParameter[];
}

interface CHHapticParameter {
  parameterId: 'hapticIntensity' | 'hapticSharpness' | 'attackTime' | 'decayTime' | 'releaseTime';
  value: number;
}

interface CHHapticPattern {
  events: CHHapticEvent[];
}

// ============================================================================
// Watch Connectivity Bridge (simulated interface)
// ============================================================================

interface WatchConnectivityBridge {
  isReachable(): Promise<boolean>;
  sendHapticPattern(pattern: CHHapticPattern): Promise<void>;
  sendHapticCommand(command: string, params: Record<string, unknown>): Promise<void>;
  getBatteryLevel(): Promise<number>;
}

// Placeholder for actual WatchConnectivity implementation
declare const WatchConnectivity: WatchConnectivityBridge;

// ============================================================================
// Apple Watch Haptic Adapter
// ============================================================================

export class AppleWatchHapticAdapter extends BaseHapticDevice {
  private watchConnectivity: WatchConnectivityBridge | null = null;

  constructor(config: DeviceConfig = {}) {
    super('smartwatch', {
      name: 'Apple Watch',
      ...config,
    });
  }

  // -------------------------------------------------------------------------
  // Connection
  // -------------------------------------------------------------------------

  async connect(): Promise<void> {
    this._state = 'connecting';

    try {
      // Check if WatchConnectivity is available
      if (typeof WatchConnectivity === 'undefined') {
        throw new HapticError(
          'WatchConnectivity not available',
          'DEVICE_NOT_FOUND'
        );
      }

      this.watchConnectivity = WatchConnectivity;

      // Check if watch is reachable
      const isReachable = await Promise.race([
        this.watchConnectivity.isReachable(),
        new Promise<boolean>((_, reject) =>
          setTimeout(() => reject(new Error('Timeout')), this._config.connectionTimeout)
        ),
      ]);

      if (!isReachable) {
        throw new HapticError(
          'Apple Watch not reachable',
          'CONNECTION_FAILED'
        );
      }

      // Set capabilities
      this._capabilities = await this.queryCapabilities();
      this._state = 'connected';

      this.emit({
        type: 'connected',
        timestamp: Date.now(),
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
    this.watchConnectivity = null;
    this._capabilities = null;
    this._state = 'disconnected';

    this.emit({
      type: 'disconnected',
      timestamp: Date.now(),
    });
  }

  private async queryCapabilities(): Promise<HapticCapabilities> {
    const batteryLevel = await this.watchConnectivity?.getBatteryLevel() ?? null;

    return {
      actuatorType: 'lra', // Apple Watch uses Linear Resonant Actuator
      frequencyRange: { min: 1, max: 300 },
      locations: ['wrist_left_dorsal', 'wrist_right_dorsal'],
      maxIntensity: 1.0,
      latency: 15, // ~15ms latency typical
      actuatorCount: 1,
      supportsCustomWaveforms: true,
      supportsAmplitudeModulation: true,
      supportsFrequencyModulation: true,
      batteryLevel,
    };
  }

  // -------------------------------------------------------------------------
  // Pattern Conversion
  // -------------------------------------------------------------------------

  private convertToCoreHaptics(pattern: HapticPattern): CHHapticPattern {
    if ('waveform' in pattern) {
      return this.convertPrimitive(pattern as HapticPrimitive);
    } else if ('steps' in pattern) {
      return this.convertSequence(pattern as HapticSequence);
    } else {
      // Spatial pattern - extract first actuation
      const spatial = pattern as any;
      if (spatial.actuations?.[0]?.pattern) {
        return this.convertToCoreHaptics(spatial.actuations[0].pattern);
      }
    }
    return { events: [] };
  }

  private convertPrimitive(primitive: HapticPrimitive): CHHapticPattern {
    const envelope = resolveEnvelope(primitive.envelope);
    const waveform = resolveWaveform(primitive.waveform);
    const intensity = this.scaleIntensity(primitive.intensity);

    // Convert WIA intensity (0-1) to Core Haptics intensity
    // Convert WIA frequency to Core Haptics sharpness (0-1)
    const sharpness = Math.min(1.0, primitive.frequency / 300);

    const events: CHHapticEvent[] = [];

    if (primitive.duration < 50) {
      // Short duration: use transient event
      events.push({
        eventType: 'hapticTransient',
        time: 0,
        parameters: [
          { parameterId: 'hapticIntensity', value: intensity },
          { parameterId: 'hapticSharpness', value: sharpness },
        ],
      });
    } else {
      // Longer duration: use continuous event with envelope
      events.push({
        eventType: 'hapticContinuous',
        time: 0,
        duration: primitive.duration / 1000, // Convert to seconds
        parameters: [
          { parameterId: 'hapticIntensity', value: intensity },
          { parameterId: 'hapticSharpness', value: sharpness },
          { parameterId: 'attackTime', value: envelope.attack / 1000 },
          { parameterId: 'decayTime', value: envelope.decay / 1000 },
          { parameterId: 'releaseTime', value: envelope.release / 1000 },
        ],
      });
    }

    return { events };
  }

  private convertSequence(sequence: HapticSequence): CHHapticPattern {
    const events: CHHapticEvent[] = [];
    let currentTime = 0;

    for (const step of sequence.steps) {
      currentTime += (step.delayBefore ?? 0) / 1000;

      const repeatCount = step.repeatCount ?? 1;
      const repeatDelay = (step.repeatDelay ?? 0) / 1000;

      for (let i = 0; i < repeatCount; i++) {
        if (step.primitive) {
          const converted = this.convertPrimitive(step.primitive);
          for (const event of converted.events) {
            events.push({
              ...event,
              time: currentTime,
              parameters: event.parameters.map(p => {
                if (p.parameterId === 'hapticIntensity') {
                  return { ...p, value: p.value * (step.intensityScale ?? 1) };
                }
                return p;
              }),
            });
          }
          currentTime += (step.primitive.duration / 1000) + repeatDelay;
        }
      }
    }

    return { events };
  }

  // -------------------------------------------------------------------------
  // Playback
  // -------------------------------------------------------------------------

  async play(pattern: HapticPattern): Promise<void> {
    this.assertConnected();

    const coreHapticsPattern = this.convertToCoreHaptics(pattern);
    await this.watchConnectivity!.sendHapticPattern(coreHapticsPattern);

    this._isPlaying = true;

    // Emit completion after duration
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
    this.watchConnectivity?.sendHapticCommand('stop', {});
    this._isPlaying = false;
  }

  // -------------------------------------------------------------------------
  // Real-time Control
  // -------------------------------------------------------------------------

  setIntensity(location: BodyLocation, intensity: number): void {
    this.assertConnected();
    const scaledIntensity = this.scaleIntensity(intensity);
    this.watchConnectivity?.sendHapticCommand('setIntensity', {
      intensity: scaledIntensity,
    });
  }

  pulse(location: BodyLocation, duration: number, intensity: number = 0.5): void {
    this.assertConnected();

    const scaledIntensity = this.scaleIntensity(intensity);
    const pattern: CHHapticPattern = {
      events: [{
        eventType: duration < 50 ? 'hapticTransient' : 'hapticContinuous',
        time: 0,
        duration: duration / 1000,
        parameters: [
          { parameterId: 'hapticIntensity', value: scaledIntensity },
          { parameterId: 'hapticSharpness', value: 0.5 },
        ],
      }],
    };

    this.watchConnectivity?.sendHapticPattern(pattern);
  }

  vibrate(location: BodyLocation, frequency: number, intensity: number): void {
    this.assertConnected();

    const scaledIntensity = this.scaleIntensity(intensity);
    const sharpness = Math.min(1.0, frequency / 300);

    const pattern: CHHapticPattern = {
      events: [{
        eventType: 'hapticContinuous',
        time: 0,
        duration: 30, // 30 seconds max continuous
        parameters: [
          { parameterId: 'hapticIntensity', value: scaledIntensity },
          { parameterId: 'hapticSharpness', value: sharpness },
        ],
      }],
    };

    this.watchConnectivity?.sendHapticPattern(pattern);
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
    return 100; // Default
  }
}
