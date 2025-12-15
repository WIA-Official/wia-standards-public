/**
 * WIA Haptic Standard - Eye Gaze Haptic Integration
 *
 * Provides haptic feedback for eye tracking / gaze control interfaces.
 * Designed to work with WIA Eye Gaze standard.
 */

import { IHapticDevice } from '../../api/typescript/src/device';
import { HapticPattern, WaveformType, BodyLocation } from '../../api/typescript/src/types';

/**
 * Gaze target information
 */
export interface GazeTarget {
  id: string;
  label: string;
  x: number;
  y: number;
  width: number;
  height: number;
  type: GazeTargetType;
}

export type GazeTargetType =
  | 'button'
  | 'link'
  | 'text_field'
  | 'menu'
  | 'keyboard_key'
  | 'slider'
  | 'scroll_area'
  | 'interactive'
  | 'static';

/**
 * Gaze event types
 */
export interface GazeEvent {
  type: GazeEventType;
  target?: GazeTarget;
  position: { x: number; y: number };
  timestamp: number;
  dwellProgress?: number;  // 0-1 for dwell selection
}

export type GazeEventType =
  | 'enter'           // Gaze entered target
  | 'leave'           // Gaze left target
  | 'dwell_start'     // Started dwelling on target
  | 'dwell_progress'  // Dwell progress update
  | 'dwell_complete'  // Dwell selection completed
  | 'dwell_cancel'    // Dwell cancelled (gaze moved away)
  | 'fixation'        // Eye fixation detected
  | 'saccade'         // Rapid eye movement
  | 'blink';          // Blink detected

/**
 * Eye gaze haptic configuration
 */
export interface EyeGazeHapticConfig {
  // Enable haptic for gaze direction feedback
  gazeDirectionEnabled: boolean;

  // Enable haptic for dwell progress
  dwellProgressEnabled: boolean;

  // Enable haptic for target hover
  targetHoverEnabled: boolean;

  // Enable haptic for selection confirmation
  selectionConfirmEnabled: boolean;

  // Dwell progress feedback interval (ms)
  dwellFeedbackInterval: number;

  // Intensity levels
  intensity: {
    hover: number;
    progress: number;
    selection: number;
  };
}

/**
 * Default eye gaze haptic configuration
 */
export const DEFAULT_EYE_GAZE_CONFIG: EyeGazeHapticConfig = {
  gazeDirectionEnabled: true,
  dwellProgressEnabled: true,
  targetHoverEnabled: true,
  selectionConfirmEnabled: true,
  dwellFeedbackInterval: 100,
  intensity: {
    hover: 0.3,
    progress: 0.5,
    selection: 0.8,
  },
};

/**
 * Eye gaze haptic patterns
 */
export const EYE_GAZE_PATTERNS = {
  // Target hover - subtle tick
  TARGET_ENTER: {
    id: 'gaze.target.enter',
    name: 'Target Enter',
    description: 'Gaze entered interactive target',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 150, intensity: 0.3, duration: 30 },
    ],
    totalDuration: 30,
  } as HapticPattern,

  // Dwell progress - rhythmic pulses
  DWELL_TICK: {
    id: 'gaze.dwell.tick',
    name: 'Dwell Tick',
    description: 'Progress tick during dwell',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 175, intensity: 0.4, duration: 20 },
    ],
    totalDuration: 20,
  } as HapticPattern,

  // Dwell complete - strong confirmation
  DWELL_COMPLETE: {
    id: 'gaze.dwell.complete',
    name: 'Dwell Complete',
    description: 'Dwell selection successful',
    primitives: [
      { waveform: WaveformType.Square, frequency: 200, intensity: 0.8, duration: 80 },
      { waveform: WaveformType.Square, frequency: 250, intensity: 0.9, duration: 100, delay: 50 },
    ],
    totalDuration: 230,
  } as HapticPattern,

  // Dwell cancelled
  DWELL_CANCEL: {
    id: 'gaze.dwell.cancel',
    name: 'Dwell Cancelled',
    description: 'Dwell selection cancelled',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 120, intensity: 0.3, duration: 100 },
    ],
    totalDuration: 100,
  } as HapticPattern,

  // Keyboard key hover
  KEY_HOVER: {
    id: 'gaze.keyboard.hover',
    name: 'Keyboard Key Hover',
    description: 'Gaze on keyboard key',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 100, intensity: 0.2, duration: 20 },
    ],
    totalDuration: 20,
  } as HapticPattern,

  // Edge of screen reached
  SCREEN_EDGE: {
    id: 'gaze.screen.edge',
    name: 'Screen Edge',
    description: 'Gaze reached screen edge',
    primitives: [
      { waveform: WaveformType.Square, frequency: 150, intensity: 0.5, duration: 50 },
    ],
    totalDuration: 50,
  } as HapticPattern,

  // Blink detected (intentional)
  BLINK_CONFIRM: {
    id: 'gaze.blink.confirm',
    name: 'Blink Confirmed',
    description: 'Intentional blink detected',
    primitives: [
      { waveform: WaveformType.Sine, frequency: 180, intensity: 0.6, duration: 100 },
    ],
    totalDuration: 100,
  } as HapticPattern,
};

/**
 * Eye Gaze Haptic Integration
 *
 * Provides haptic feedback for eye tracking interfaces.
 *
 * @example
 * ```typescript
 * const device = await HapticDeviceManager.connect('bluetooth', deviceId);
 * const gazeHaptic = new EyeGazeHaptic(device);
 *
 * // Connect to eye tracker events
 * eyeTracker.on('gaze', (event) => {
 *   gazeHaptic.onGazeEvent(event);
 * });
 * ```
 */
export class EyeGazeHaptic {
  private device: IHapticDevice;
  private config: EyeGazeHapticConfig;
  private isActive: boolean = false;
  private currentTarget: GazeTarget | null = null;
  private dwellStartTime: number = 0;
  private lastDwellTick: number = 0;

  // Screen dimensions for edge detection
  private screenWidth: number = 1920;
  private screenHeight: number = 1080;

  constructor(
    device: IHapticDevice,
    config?: Partial<EyeGazeHapticConfig>
  ) {
    this.device = device;
    this.config = { ...DEFAULT_EYE_GAZE_CONFIG, ...config };
  }

  /**
   * Start eye gaze haptic feedback
   */
  start(): void {
    this.isActive = true;
  }

  /**
   * Stop eye gaze haptic feedback
   */
  stop(): void {
    this.isActive = false;
    this.currentTarget = null;
  }

  /**
   * Set screen dimensions for edge detection
   */
  setScreenDimensions(width: number, height: number): void {
    this.screenWidth = width;
    this.screenHeight = height;
  }

  /**
   * Handle gaze event
   */
  async onGazeEvent(event: GazeEvent): Promise<void> {
    if (!this.isActive) return;

    switch (event.type) {
      case 'enter':
        await this.handleTargetEnter(event);
        break;
      case 'leave':
        await this.handleTargetLeave(event);
        break;
      case 'dwell_start':
        await this.handleDwellStart(event);
        break;
      case 'dwell_progress':
        await this.handleDwellProgress(event);
        break;
      case 'dwell_complete':
        await this.handleDwellComplete(event);
        break;
      case 'dwell_cancel':
        await this.handleDwellCancel(event);
        break;
      case 'blink':
        await this.handleBlink(event);
        break;
    }

    // Check for screen edge
    await this.checkScreenEdge(event.position);
  }

  /**
   * Handle gaze entering target
   */
  private async handleTargetEnter(event: GazeEvent): Promise<void> {
    if (!this.config.targetHoverEnabled || !event.target) return;

    this.currentTarget = event.target;

    // Different feedback based on target type
    if (event.target.type === 'keyboard_key') {
      await this.playPattern(EYE_GAZE_PATTERNS.KEY_HOVER);
    } else if (event.target.type !== 'static') {
      await this.playPattern(EYE_GAZE_PATTERNS.TARGET_ENTER);
    }
  }

  /**
   * Handle gaze leaving target
   */
  private async handleTargetLeave(event: GazeEvent): Promise<void> {
    this.currentTarget = null;
    this.dwellStartTime = 0;
    this.lastDwellTick = 0;
  }

  /**
   * Handle dwell start
   */
  private async handleDwellStart(event: GazeEvent): Promise<void> {
    this.dwellStartTime = event.timestamp;
    this.lastDwellTick = event.timestamp;
  }

  /**
   * Handle dwell progress
   */
  private async handleDwellProgress(event: GazeEvent): Promise<void> {
    if (!this.config.dwellProgressEnabled || event.dwellProgress === undefined) return;

    const timeSinceLastTick = event.timestamp - this.lastDwellTick;

    // Provide rhythmic feedback based on progress
    const interval = this.config.dwellFeedbackInterval * (1 - event.dwellProgress * 0.5);

    if (timeSinceLastTick >= interval) {
      const pattern = this.createProgressPattern(event.dwellProgress);
      await this.playPattern(pattern);
      this.lastDwellTick = event.timestamp;
    }
  }

  /**
   * Handle dwell complete
   */
  private async handleDwellComplete(event: GazeEvent): Promise<void> {
    if (!this.config.selectionConfirmEnabled) return;

    await this.playPattern(EYE_GAZE_PATTERNS.DWELL_COMPLETE);
  }

  /**
   * Handle dwell cancel
   */
  private async handleDwellCancel(event: GazeEvent): Promise<void> {
    this.dwellStartTime = 0;
    this.lastDwellTick = 0;

    if (this.config.dwellProgressEnabled) {
      await this.playPattern(EYE_GAZE_PATTERNS.DWELL_CANCEL);
    }
  }

  /**
   * Handle blink
   */
  private async handleBlink(event: GazeEvent): Promise<void> {
    if (this.config.selectionConfirmEnabled && this.currentTarget) {
      await this.playPattern(EYE_GAZE_PATTERNS.BLINK_CONFIRM);
    }
  }

  /**
   * Check for screen edge
   */
  private async checkScreenEdge(position: { x: number; y: number }): Promise<void> {
    if (!this.config.gazeDirectionEnabled) return;

    const edgeThreshold = 50; // pixels from edge

    const atEdge =
      position.x <= edgeThreshold ||
      position.x >= this.screenWidth - edgeThreshold ||
      position.y <= edgeThreshold ||
      position.y >= this.screenHeight - edgeThreshold;

    if (atEdge) {
      await this.playPattern(EYE_GAZE_PATTERNS.SCREEN_EDGE);
    }
  }

  /**
   * Create progress-based haptic pattern
   */
  private createProgressPattern(progress: number): HapticPattern {
    // Increase frequency and intensity as progress increases
    const frequency = 150 + Math.round(progress * 100);
    const intensity = this.config.intensity.progress + progress * 0.3;

    return {
      id: `gaze.dwell.progress.${Math.round(progress * 100)}`,
      name: `Dwell Progress ${Math.round(progress * 100)}%`,
      description: `Progress feedback at ${Math.round(progress * 100)}%`,
      primitives: [
        { waveform: WaveformType.Sine, frequency, intensity, duration: 30 },
      ],
      totalDuration: 30,
    };
  }

  /**
   * Create gaze direction pattern
   */
  createDirectionPattern(
    direction: number,
    screenWidth: number = this.screenWidth
  ): HapticPattern {
    // Map screen position to left/right intensity
    const normalizedX = direction / screenWidth;

    return {
      id: `gaze.direction.${Math.round(normalizedX * 100)}`,
      name: `Gaze Direction`,
      description: `Direction at ${Math.round(normalizedX * 100)}% across screen`,
      primitives: [
        {
          waveform: WaveformType.Sine,
          frequency: 150,
          intensity: 0.3,
          duration: 50,
        },
      ],
      totalDuration: 50,
      metadata: {
        leftIntensity: 1 - normalizedX,
        rightIntensity: normalizedX,
      },
    };
  }

  /**
   * Play haptic pattern
   */
  private async playPattern(pattern: HapticPattern): Promise<void> {
    try {
      await this.device.playPattern(pattern);
    } catch (error) {
      console.error('Failed to play haptic pattern:', error);
    }
  }
}

export default EyeGazeHaptic;
