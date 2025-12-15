/**
 * WIA Haptic Standard - Web Vibration API Integration
 *
 * Web platform integration using the Vibration API and Gamepad Haptics API.
 */

import { HapticPattern, WaveformType } from '../../api/typescript/src/types';

/**
 * Web haptic capabilities
 */
export interface WebHapticCapabilities {
  vibrationAPI: boolean;
  gamepadHaptics: boolean;
  maxVibrationDuration: number;
}

/**
 * Check web haptic capabilities
 */
export function checkWebHapticCapabilities(): WebHapticCapabilities {
  return {
    vibrationAPI: 'vibrate' in navigator,
    gamepadHaptics: 'getGamepads' in navigator,
    maxVibrationDuration: 10000, // Most browsers limit to 10 seconds
  };
}

/**
 * Web Haptic Patterns for accessibility
 */
export const WEB_HAPTIC_PATTERNS = {
  // Focus events
  FOCUS_INTERACTIVE: {
    id: 'web.focus.interactive',
    name: 'Interactive Focus',
    description: 'Focus on interactive element',
    primitives: [
      { waveform: WaveformType.Square, frequency: 150, intensity: 0.5, duration: 30 },
    ],
    totalDuration: 30,
  } as HapticPattern,

  FOCUS_INPUT: {
    id: 'web.focus.input',
    name: 'Input Focus',
    description: 'Focus on input field',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.4, duration: 40 },
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.5, duration: 40, delay: 20 },
    ],
    totalDuration: 100,
  } as HapticPattern,

  // Click events
  CLICK: {
    id: 'web.action.click',
    name: 'Click',
    description: 'Element clicked',
    primitives: [
      { waveform: WaveformType.Square, frequency: 180, intensity: 0.7, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  // Form events
  FORM_SUBMIT: {
    id: 'web.form.submit',
    name: 'Form Submit',
    description: 'Form submitted successfully',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.6, duration: 80 },
      { waveform: WaveformType.Sine, frequency: 220, intensity: 0.7, duration: 100, delay: 50 },
    ],
    totalDuration: 230,
  } as HapticPattern,

  FORM_ERROR: {
    id: 'web.form.error',
    name: 'Form Error',
    description: 'Form validation error',
    primitives: [
      { waveform: WaveformType.Square, frequency: 100, intensity: 0.7, duration: 100 },
      { waveform: WaveformType.Square, frequency: 100, intensity: 0.7, duration: 100, delay: 80 },
    ],
    totalDuration: 280,
  } as HapticPattern,

  // Navigation events
  PAGE_LOAD: {
    id: 'web.nav.pageload',
    name: 'Page Loaded',
    description: 'New page finished loading',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.5, duration: 100 },
    ],
    totalDuration: 100,
  } as HapticPattern,

  LINK_NAVIGATE: {
    id: 'web.nav.link',
    name: 'Link Navigate',
    description: 'Navigating to link',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 160, intensity: 0.4, duration: 60 },
    ],
    totalDuration: 60,
  } as HapticPattern,

  // Notification events
  NOTIFICATION: {
    id: 'web.notification',
    name: 'Notification',
    description: 'New notification received',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.6, duration: 80 },
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.5, duration: 80, delay: 100 },
    ],
    totalDuration: 260,
  } as HapticPattern,

  // Loading states
  LOADING_START: {
    id: 'web.loading.start',
    name: 'Loading Started',
    description: 'Content loading started',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.3, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  LOADING_COMPLETE: {
    id: 'web.loading.complete',
    name: 'Loading Complete',
    description: 'Content loading finished',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.5, duration: 80 },
    ],
    totalDuration: 80,
  } as HapticPattern,
};

/**
 * Web Vibration Haptic Engine
 *
 * Uses the Web Vibration API and Gamepad Haptics API for haptic feedback.
 */
export class WebVibrationHaptic {
  private isEnabled: boolean = true;
  private capabilities: WebHapticCapabilities;

  constructor() {
    this.capabilities = checkWebHapticCapabilities();
  }

  /**
   * Check if vibration is supported
   */
  get isSupported(): boolean {
    return this.capabilities.vibrationAPI;
  }

  /**
   * Enable/disable haptic feedback
   */
  setEnabled(enabled: boolean): void {
    this.isEnabled = enabled;
  }

  /**
   * Play a WIA haptic pattern using Vibration API
   */
  playPattern(pattern: HapticPattern): boolean {
    if (!this.isEnabled || !this.capabilities.vibrationAPI) {
      return false;
    }

    const vibrationPattern = this.convertToVibrationPattern(pattern);
    return navigator.vibrate(vibrationPattern);
  }

  /**
   * Stop any ongoing vibration
   */
  stop(): void {
    if (this.capabilities.vibrationAPI) {
      navigator.vibrate(0);
    }
  }

  /**
   * Convert WIA pattern to Web Vibration API format
   *
   * The Vibration API uses alternating durations: [vibrate, pause, vibrate, pause, ...]
   */
  private convertToVibrationPattern(pattern: HapticPattern): number[] {
    const result: number[] = [];
    let needsInitialPause = false;

    for (const primitive of pattern.primitives) {
      // Add delay/pause if needed
      if (primitive.delay && primitive.delay > 0) {
        if (result.length === 0) {
          // Start with pause
          result.push(0); // 0ms vibration
          result.push(primitive.delay);
        } else {
          // Add pause to previous entry or add new pause
          if (result.length % 2 === 0) {
            // Even length means we need to add a pause
            result.push(primitive.delay);
          } else {
            // Odd length means last entry was vibration, add pause
            result.push(primitive.delay);
          }
        }
      } else if (result.length > 0 && result.length % 2 === 1) {
        // Need a pause between vibrations
        result.push(0);
      }

      // Add vibration duration
      // Note: Vibration API doesn't support intensity, so we just use duration
      result.push(primitive.duration);
    }

    return result;
  }

  /**
   * Play simple vibration
   */
  vibrate(duration: number): boolean {
    if (!this.isEnabled || !this.capabilities.vibrationAPI) {
      return false;
    }
    return navigator.vibrate(duration);
  }

  /**
   * Play vibration pattern (raw format)
   */
  vibratePattern(pattern: number[]): boolean {
    if (!this.isEnabled || !this.capabilities.vibrationAPI) {
      return false;
    }
    return navigator.vibrate(pattern);
  }
}

/**
 * Gamepad Haptic Engine
 *
 * Uses the Gamepad API for haptic feedback through game controllers.
 */
export class GamepadHaptic {
  private isEnabled: boolean = true;

  /**
   * Get connected gamepads with haptic support
   */
  getHapticGamepads(): Gamepad[] {
    if (!('getGamepads' in navigator)) {
      return [];
    }

    const gamepads = navigator.getGamepads();
    return Array.from(gamepads).filter((gp): gp is Gamepad => {
      return gp !== null && gp.vibrationActuator !== undefined;
    });
  }

  /**
   * Play haptic effect on gamepad
   */
  async playEffect(
    gamepadIndex: number,
    pattern: HapticPattern
  ): Promise<boolean> {
    if (!this.isEnabled) return false;

    const gamepads = navigator.getGamepads();
    const gamepad = gamepads[gamepadIndex];

    if (!gamepad?.vibrationActuator) {
      return false;
    }

    // Convert WIA pattern to gamepad haptic effect
    const effect = this.convertToGamepadEffect(pattern);

    try {
      // Use the Gamepad Haptic Actuator API
      // Note: This is still experimental and may vary by browser
      const actuator = gamepad.vibrationActuator as any;

      if ('playEffect' in actuator) {
        await actuator.playEffect('dual-rumble', effect);
        return true;
      } else if ('pulse' in actuator) {
        // Fallback for older API
        await actuator.pulse(effect.strongMagnitude, effect.duration);
        return true;
      }
    } catch (error) {
      console.error('Gamepad haptic error:', error);
    }

    return false;
  }

  /**
   * Convert WIA pattern to gamepad haptic effect
   */
  private convertToGamepadEffect(pattern: HapticPattern): GamepadEffectParameters {
    // Calculate weighted average intensity
    let totalIntensity = 0;
    let totalDuration = 0;

    for (const primitive of pattern.primitives) {
      totalIntensity += primitive.intensity * primitive.duration;
      totalDuration += primitive.duration + (primitive.delay || 0);
    }

    const avgIntensity = totalDuration > 0 ? totalIntensity / totalDuration : 0.5;

    return {
      duration: totalDuration,
      startDelay: 0,
      strongMagnitude: avgIntensity,
      weakMagnitude: avgIntensity * 0.5,
    };
  }

  /**
   * Stop all gamepad haptics
   */
  async stopAll(): Promise<void> {
    const gamepads = navigator.getGamepads();

    for (const gamepad of gamepads) {
      if (gamepad?.vibrationActuator) {
        try {
          const actuator = gamepad.vibrationActuator as any;
          if ('reset' in actuator) {
            await actuator.reset();
          }
        } catch {
          // Ignore errors
        }
      }
    }
  }
}

/**
 * Gamepad effect parameters
 */
interface GamepadEffectParameters {
  duration: number;
  startDelay: number;
  strongMagnitude: number;
  weakMagnitude: number;
}

/**
 * Accessibility event types for web
 */
export type WebAccessibilityEvent =
  | 'focus'
  | 'blur'
  | 'click'
  | 'submit'
  | 'error'
  | 'navigate'
  | 'load'
  | 'notification';

/**
 * Web Accessibility Haptic Integration
 *
 * Provides haptic feedback for web accessibility events.
 */
export class WebAccessibilityHaptic {
  private vibration: WebVibrationHaptic;
  private gamepad: GamepadHaptic;

  constructor() {
    this.vibration = new WebVibrationHaptic();
    this.gamepad = new GamepadHaptic();
  }

  /**
   * Initialize event listeners for accessibility haptics
   */
  initialize(): void {
    if (!this.vibration.isSupported) {
      console.warn('Vibration API not supported');
      return;
    }

    // Focus events
    document.addEventListener('focusin', (event) => {
      this.handleFocus(event.target as Element);
    });

    // Click events
    document.addEventListener('click', (event) => {
      this.handleClick(event.target as Element);
    });

    // Form events
    document.addEventListener('submit', () => {
      this.vibration.playPattern(WEB_HAPTIC_PATTERNS.FORM_SUBMIT);
    });

    // Page load
    window.addEventListener('load', () => {
      this.vibration.playPattern(WEB_HAPTIC_PATTERNS.PAGE_LOAD);
    });
  }

  /**
   * Handle focus event
   */
  private handleFocus(element: Element): void {
    const tagName = element.tagName.toLowerCase();
    const role = element.getAttribute('role');

    if (['input', 'textarea', 'select'].includes(tagName)) {
      this.vibration.playPattern(WEB_HAPTIC_PATTERNS.FOCUS_INPUT);
    } else if (
      ['button', 'a'].includes(tagName) ||
      role === 'button' ||
      role === 'link'
    ) {
      this.vibration.playPattern(WEB_HAPTIC_PATTERNS.FOCUS_INTERACTIVE);
    }
  }

  /**
   * Handle click event
   */
  private handleClick(element: Element): void {
    const tagName = element.tagName.toLowerCase();

    if (['button', 'a', 'input'].includes(tagName) ||
        element.getAttribute('role') === 'button') {
      this.vibration.playPattern(WEB_HAPTIC_PATTERNS.CLICK);
    }
  }

  /**
   * Play pattern for accessibility event
   */
  playEvent(event: WebAccessibilityEvent): void {
    switch (event) {
      case 'focus':
        this.vibration.playPattern(WEB_HAPTIC_PATTERNS.FOCUS_INTERACTIVE);
        break;
      case 'click':
        this.vibration.playPattern(WEB_HAPTIC_PATTERNS.CLICK);
        break;
      case 'submit':
        this.vibration.playPattern(WEB_HAPTIC_PATTERNS.FORM_SUBMIT);
        break;
      case 'error':
        this.vibration.playPattern(WEB_HAPTIC_PATTERNS.FORM_ERROR);
        break;
      case 'navigate':
        this.vibration.playPattern(WEB_HAPTIC_PATTERNS.LINK_NAVIGATE);
        break;
      case 'load':
        this.vibration.playPattern(WEB_HAPTIC_PATTERNS.PAGE_LOAD);
        break;
      case 'notification':
        this.vibration.playPattern(WEB_HAPTIC_PATTERNS.NOTIFICATION);
        break;
    }
  }

  /**
   * Play custom pattern
   */
  playPattern(pattern: HapticPattern): void {
    this.vibration.playPattern(pattern);
  }

  /**
   * Stop all haptics
   */
  stop(): void {
    this.vibration.stop();
    this.gamepad.stopAll();
  }
}

export default WebAccessibilityHaptic;
