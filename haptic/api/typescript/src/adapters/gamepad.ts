/**
 * WIA Haptic Standard - Gamepad Adapter
 * @version 1.0.0
 *
 * Adapter for game controllers (DualSense, Xbox, etc.) using Web Gamepad API
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
// Gamepad Types
// ============================================================================

type GamepadType = 'dualsense' | 'xbox' | 'generic';

interface GamepadHapticActuator {
  type: 'vibration' | 'dual-rumble';
  playEffect(type: string, params: GamepadEffectParams): Promise<string>;
  reset(): Promise<string>;
}

interface GamepadEffectParams {
  duration: number;
  startDelay?: number;
  strongMagnitude?: number;
  weakMagnitude?: number;
  leftTrigger?: number;
  rightTrigger?: number;
}

interface ExtendedGamepad extends Gamepad {
  vibrationActuator?: GamepadHapticActuator;
  hapticActuators?: GamepadHapticActuator[];
}

// ============================================================================
// Gamepad Configuration
// ============================================================================

export interface GamepadConfig extends DeviceConfig {
  /** Gamepad index (0-3) */
  gamepadIndex?: number;

  /** Preferred gamepad type */
  preferredType?: GamepadType;

  /** Map hand to rumble motor */
  motorMapping?: {
    left: 'strong' | 'weak';
    right: 'strong' | 'weak';
  };

  /** Enable trigger haptics (DualSense) */
  enableTriggerHaptics?: boolean;
}

// ============================================================================
// Gamepad Haptic Adapter
// ============================================================================

export class GamepadHapticAdapter extends BaseHapticDevice {
  private gamepad: ExtendedGamepad | null = null;
  private gamepadIndex: number;
  private gamepadType: GamepadType = 'generic';
  private gpConfig: GamepadConfig;
  private pollInterval: NodeJS.Timeout | null = null;
  private currentEffect: { promise: Promise<void>; cancel: () => void } | null = null;

  constructor(config: GamepadConfig = {}) {
    super('gamepad', {
      name: 'Gamepad Controller',
      ...config,
    });

    this.gamepadIndex = config.gamepadIndex ?? 0;
    this.gpConfig = {
      motorMapping: {
        left: 'strong',
        right: 'weak',
      },
      enableTriggerHaptics: true,
      ...config,
    };
  }

  // -------------------------------------------------------------------------
  // Connection
  // -------------------------------------------------------------------------

  async connect(): Promise<void> {
    this._state = 'connecting';

    try {
      // Check for Gamepad API support
      if (!('getGamepads' in navigator)) {
        throw new HapticError(
          'Gamepad API not available',
          'DEVICE_NOT_FOUND'
        );
      }

      // Get gamepad
      const gamepads = navigator.getGamepads();
      const gamepad = gamepads[this.gamepadIndex] as ExtendedGamepad | null;

      if (!gamepad) {
        // Wait for gamepad connection
        await this.waitForGamepad();
      } else {
        this.gamepad = gamepad;
      }

      // Detect gamepad type
      this.gamepadType = this.detectGamepadType();

      // Check for haptic support
      if (!this.gamepad?.vibrationActuator && !this.gamepad?.hapticActuators?.length) {
        throw new HapticError(
          'Gamepad does not support haptics',
          'UNSUPPORTED_FEATURE'
        );
      }

      // Start polling for gamepad state
      this.startPolling();

      this._capabilities = this.buildCapabilities();
      this._state = 'connected';

      this.emit({
        type: 'connected',
        timestamp: Date.now(),
        data: {
          gamepadId: this.gamepad?.id,
          gamepadType: this.gamepadType,
        },
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

  private async waitForGamepad(): Promise<void> {
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        window.removeEventListener('gamepadconnected', handler);
        reject(new HapticError('Gamepad connection timeout', 'CONNECTION_TIMEOUT'));
      }, this._config.connectionTimeout);

      const handler = (event: GamepadEvent) => {
        if (event.gamepad.index === this.gamepadIndex) {
          clearTimeout(timeout);
          window.removeEventListener('gamepadconnected', handler);
          this.gamepad = event.gamepad as ExtendedGamepad;
          resolve();
        }
      };

      window.addEventListener('gamepadconnected', handler);
    });
  }

  async disconnect(): Promise<void> {
    this._state = 'disconnecting';

    this.stopPolling();
    await this.stopHaptics();

    this.gamepad = null;
    this._capabilities = null;
    this._state = 'disconnected';

    this.emit({
      type: 'disconnected',
      timestamp: Date.now(),
    });
  }

  private detectGamepadType(): GamepadType {
    const id = this.gamepad?.id?.toLowerCase() ?? '';

    if (id.includes('dualsense') || id.includes('054c:0ce6')) {
      return 'dualsense';
    }
    if (id.includes('xbox') || id.includes('045e:')) {
      return 'xbox';
    }

    return this.gpConfig.preferredType ?? 'generic';
  }

  private buildCapabilities(): HapticCapabilities {
    const locations: BodyLocation[] = ['palm_left', 'palm_right'];

    // DualSense has additional trigger haptics
    if (this.gamepadType === 'dualsense' && this.gpConfig.enableTriggerHaptics) {
      locations.push('index_left', 'index_right');
    }

    return {
      actuatorType: 'erm',
      frequencyRange: { min: 1, max: 100 }, // Limited frequency control
      locations,
      maxIntensity: 1.0,
      latency: 10,
      actuatorCount: this.gamepadType === 'dualsense' ? 4 : 2,
      supportsCustomWaveforms: false,
      supportsAmplitudeModulation: true,
      supportsFrequencyModulation: false,
      batteryLevel: null, // TODO: Read from gamepad if available
    };
  }

  private startPolling(): void {
    // Poll gamepad state to detect disconnection
    this.pollInterval = setInterval(() => {
      const gamepads = navigator.getGamepads();
      const gamepad = gamepads[this.gamepadIndex] as ExtendedGamepad | null;

      if (!gamepad && this._state === 'connected') {
        this._state = 'disconnected';
        this.emit({
          type: 'disconnected',
          timestamp: Date.now(),
        });
      } else if (gamepad) {
        this.gamepad = gamepad;
      }
    }, 100);
  }

  private stopPolling(): void {
    if (this.pollInterval) {
      clearInterval(this.pollInterval);
      this.pollInterval = null;
    }
  }

  // -------------------------------------------------------------------------
  // Haptic Control
  // -------------------------------------------------------------------------

  private async playDualRumble(
    strongMagnitude: number,
    weakMagnitude: number,
    duration: number,
    startDelay: number = 0
  ): Promise<void> {
    const actuator = this.gamepad?.vibrationActuator;
    if (!actuator) return;

    await actuator.playEffect('dual-rumble', {
      duration,
      startDelay,
      strongMagnitude: this.scaleIntensity(strongMagnitude),
      weakMagnitude: this.scaleIntensity(weakMagnitude),
    });
  }

  private async playTriggerHaptics(
    leftTrigger: number,
    rightTrigger: number,
    duration: number
  ): Promise<void> {
    if (this.gamepadType !== 'dualsense') return;

    const actuator = this.gamepad?.vibrationActuator;
    if (!actuator) return;

    // DualSense-specific trigger haptics
    await actuator.playEffect('trigger-rumble', {
      duration,
      leftTrigger: this.scaleIntensity(leftTrigger),
      rightTrigger: this.scaleIntensity(rightTrigger),
    });
  }

  private async stopHaptics(): Promise<void> {
    const actuator = this.gamepad?.vibrationActuator;
    if (actuator) {
      await actuator.reset();
    }
  }

  // -------------------------------------------------------------------------
  // Pattern Conversion
  // -------------------------------------------------------------------------

  private mapLocationToMotor(location: BodyLocation): { strong: number; weak: number } {
    const mapping = this.gpConfig.motorMapping!;

    switch (location) {
      case 'palm_left':
      case 'wrist_left_dorsal':
        return mapping.left === 'strong'
          ? { strong: 1, weak: 0 }
          : { strong: 0, weak: 1 };

      case 'palm_right':
      case 'wrist_right_dorsal':
        return mapping.right === 'strong'
          ? { strong: 1, weak: 0 }
          : { strong: 0, weak: 1 };

      case 'index_left':
        return { strong: 0, weak: 0 }; // Trigger only

      case 'index_right':
        return { strong: 0, weak: 0 }; // Trigger only

      default:
        // Default to both motors
        return { strong: 0.5, weak: 0.5 };
    }
  }

  private async playPrimitive(primitive: HapticPrimitive, location?: BodyLocation): Promise<void> {
    const envelope = resolveEnvelope(primitive.envelope);
    const intensity = primitive.intensity;

    // Map to motors
    const motors = location
      ? this.mapLocationToMotor(location)
      : { strong: 0.5, weak: 0.5 };

    // Handle trigger haptics for finger locations
    if (location === 'index_left' || location === 'index_right') {
      const leftTrigger = location === 'index_left' ? intensity : 0;
      const rightTrigger = location === 'index_right' ? intensity : 0;
      await this.playTriggerHaptics(leftTrigger, rightTrigger, primitive.duration);
      return;
    }

    // Simple envelope approximation using multiple rumbles
    const totalDuration = primitive.duration;
    const attackDuration = envelope.attack;
    const releaseDuration = envelope.release;
    const sustainDuration = totalDuration - attackDuration - envelope.decay - releaseDuration;

    // Attack phase
    if (attackDuration > 0) {
      const steps = Math.max(1, Math.ceil(attackDuration / 20));
      for (let i = 0; i < steps; i++) {
        const progress = (i + 1) / steps;
        const stepIntensity = intensity * progress;
        await this.playDualRumble(
          motors.strong * stepIntensity,
          motors.weak * stepIntensity,
          attackDuration / steps
        );
      }
    }

    // Sustain phase
    if (sustainDuration > 0) {
      const sustainIntensity = intensity * envelope.sustain;
      await this.playDualRumble(
        motors.strong * sustainIntensity,
        motors.weak * sustainIntensity,
        sustainDuration
      );
    }

    // Release phase
    if (releaseDuration > 0) {
      const sustainIntensity = intensity * envelope.sustain;
      const steps = Math.max(1, Math.ceil(releaseDuration / 20));
      for (let i = 0; i < steps; i++) {
        const progress = 1 - (i + 1) / steps;
        const stepIntensity = sustainIntensity * progress;
        await this.playDualRumble(
          motors.strong * stepIntensity,
          motors.weak * stepIntensity,
          releaseDuration / steps
        );
      }
    }
  }

  // -------------------------------------------------------------------------
  // Playback
  // -------------------------------------------------------------------------

  async play(pattern: HapticPattern): Promise<void> {
    this.assertConnected();

    this._isPlaying = true;

    try {
      if ('waveform' in pattern) {
        await this.playPrimitive(pattern as HapticPrimitive);
      } else if ('steps' in pattern) {
        const sequence = pattern as HapticSequence;
        const loopCount = sequence.loop ? (sequence.loopCount ?? 1) : 1;

        for (let loop = 0; loop < loopCount && !this._isPaused; loop++) {
          for (const step of sequence.steps) {
            if (this._isPaused) break;

            // Delay before
            if (step.delayBefore && step.delayBefore > 0) {
              await this.delay(step.delayBefore);
            }

            // Play with repeats
            const repeatCount = step.repeatCount ?? 1;
            for (let i = 0; i < repeatCount && !this._isPaused; i++) {
              if (step.primitive) {
                const scaledPrimitive = {
                  ...step.primitive,
                  intensity: step.primitive.intensity * (step.intensityScale ?? 1),
                };
                await this.playPrimitive(scaledPrimitive);
              }

              // Repeat delay
              if (i < repeatCount - 1 && step.repeatDelay) {
                await this.delay(step.repeatDelay);
              }
            }
          }
        }
      } else {
        // Spatial pattern
        const spatial = pattern as any;
        for (const actuation of spatial.actuations ?? []) {
          const location = actuation.location ?? actuation.locations?.[0];
          if (actuation.pattern && 'waveform' in actuation.pattern) {
            await this.playPrimitive(actuation.pattern, location);
          }
        }
      }

      this.emit({
        type: 'pattern_complete',
        timestamp: Date.now(),
      });
    } finally {
      this._isPlaying = false;
    }
  }

  stop(): void {
    this._isPaused = false;
    this._isPlaying = false;
    this.stopHaptics();
  }

  // -------------------------------------------------------------------------
  // Real-time Control
  // -------------------------------------------------------------------------

  setIntensity(location: BodyLocation, intensity: number): void {
    this.assertConnected();

    const motors = this.mapLocationToMotor(location);
    const scaledIntensity = this.scaleIntensity(intensity);

    this.playDualRumble(
      motors.strong * scaledIntensity,
      motors.weak * scaledIntensity,
      100 // Short duration, caller should keep calling
    );
  }

  pulse(location: BodyLocation, duration: number, intensity: number = 0.5): void {
    this.assertConnected();

    const motors = this.mapLocationToMotor(location);
    const scaledIntensity = this.scaleIntensity(intensity);

    this.playDualRumble(
      motors.strong * scaledIntensity,
      motors.weak * scaledIntensity,
      duration
    );
  }

  vibrate(location: BodyLocation, frequency: number, intensity: number): void {
    this.assertConnected();

    // Gamepad API doesn't support frequency control
    // Just use continuous vibration
    const motors = this.mapLocationToMotor(location);
    const scaledIntensity = this.scaleIntensity(intensity);

    this.playDualRumble(
      motors.strong * scaledIntensity,
      motors.weak * scaledIntensity,
      5000 // 5 second duration
    );
  }

  // -------------------------------------------------------------------------
  // Utility
  // -------------------------------------------------------------------------

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// ============================================================================
// Specific Gamepad Adapters
// ============================================================================

export class DualSenseAdapter extends GamepadHapticAdapter {
  constructor(config: Omit<GamepadConfig, 'preferredType'> = {}) {
    super({
      ...config,
      preferredType: 'dualsense',
      name: 'DualSense Controller',
      enableTriggerHaptics: true,
    });
  }
}

export class XboxAdapter extends GamepadHapticAdapter {
  constructor(config: Omit<GamepadConfig, 'preferredType'> = {}) {
    super({
      ...config,
      preferredType: 'xbox',
      name: 'Xbox Controller',
      enableTriggerHaptics: false,
    });
  }
}
