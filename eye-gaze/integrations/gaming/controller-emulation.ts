/**
 * WIA Eye Gaze Standard - Gaming Integration
 *
 * Controller emulation and gaming accessibility features.
 * Enables gaze-based mouse/controller input for gaming.
 *
 * 弘益人間 - 널리 인간을 이롭게
 *
 * @module @anthropics/wia-eye-gaze/integrations/gaming
 */

import type { GazePoint, Vector2D, Vector3D } from '../../api/typescript/src/types';

/**
 * Mouse event for emulation
 */
export interface EmulatedMouseEvent {
  type: 'move' | 'click' | 'down' | 'up' | 'scroll';
  x: number;
  y: number;
  button?: 'left' | 'right' | 'middle';
  scrollDelta?: { x: number; y: number };
  timestamp: number;
}

/**
 * Gamepad analog stick value
 */
export interface AnalogStickValue {
  /** X axis (-1.0 to 1.0) */
  x: number;
  /** Y axis (-1.0 to 1.0) */
  y: number;
  /** Magnitude (0.0 to 1.0) */
  magnitude: number;
  /** Angle in degrees */
  angle: number;
}

/**
 * Gamepad state
 */
export interface GamepadState {
  /** Analog sticks */
  leftStick: AnalogStickValue;
  rightStick: AnalogStickValue;
  /** Triggers (0.0 to 1.0) */
  leftTrigger: number;
  rightTrigger: number;
  /** D-pad */
  dpad: {
    up: boolean;
    down: boolean;
    left: boolean;
    right: boolean;
  };
  /** Buttons */
  buttons: {
    a: boolean;
    b: boolean;
    x: boolean;
    y: boolean;
    lb: boolean;
    rb: boolean;
    back: boolean;
    start: boolean;
    leftStickPress: boolean;
    rightStickPress: boolean;
  };
}

/**
 * Aim assist configuration
 */
export interface AimAssistConfig {
  /** Enable aim assist */
  enabled: boolean;
  /** Assist strength (0.0 to 1.0) */
  strength: number;
  /** Slowdown zone radius (normalized) */
  slowdownRadius: number;
  /** Snap to target strength */
  snapStrength: number;
  /** Target lock time (ms) */
  lockTime: number;
}

/**
 * VR/XR gaze integration
 */
export interface XRGazeIntegration {
  /** OpenXR support */
  openXR: boolean;
  /** SteamVR support */
  steamVR: boolean;
  /** Meta Quest support */
  metaQuest: boolean;
  /** Current headset type */
  headset?: string;
}

/**
 * Gaming zone configuration
 */
export interface GamingZone {
  /** Zone ID */
  id: string;
  /** Zone type */
  type: 'movement' | 'aim' | 'button' | 'scroll';
  /** Bounds (normalized 0-1) */
  bounds: { x: number; y: number; width: number; height: number };
  /** Action when gazed */
  action: GamingAction;
}

/**
 * Gaming action
 */
export type GamingAction =
  | { type: 'move'; direction: 'up' | 'down' | 'left' | 'right' }
  | { type: 'look'; sensitivity: number }
  | { type: 'button'; key: string }
  | { type: 'scroll'; speed: number }
  | { type: 'aim'; targetId?: string };

/**
 * Target for aim assist
 */
export interface AimTarget {
  id: string;
  position: Vector2D | Vector3D;
  size: number;
  priority: number;
}

/**
 * Gaming Integration class
 *
 * Provides gaze-based gaming controls including mouse emulation,
 * gamepad emulation, and aim assist.
 */
export class GamingIntegration {
  /** Current gaze position */
  private currentGaze: GazePoint | null = null;

  /** Screen dimensions */
  private screenWidth = 1920;
  private screenHeight = 1080;

  /** Smoothing buffer */
  private gazeBuffer: GazePoint[] = [];
  private readonly bufferSize = 5;

  /** Aim assist state */
  private aimAssistConfig: AimAssistConfig = {
    enabled: false,
    strength: 0.5,
    slowdownRadius: 0.1,
    snapStrength: 0.3,
    lockTime: 200,
  };
  private aimTargets: Map<string, AimTarget> = new Map();
  private lockedTarget: AimTarget | null = null;
  private lockStartTime = 0;

  /** XR integration */
  private xrIntegration: XRGazeIntegration = {
    openXR: false,
    steamVR: false,
    metaQuest: false,
  };

  /** Gaming zones */
  private zones: Map<string, GamingZone> = new Map();

  /** Emulation callbacks */
  private mouseCallbacks: ((event: EmulatedMouseEvent) => void)[] = [];
  private gamepadCallbacks: ((state: GamepadState) => void)[] = [];

  constructor(screenWidth?: number, screenHeight?: number) {
    if (screenWidth) this.screenWidth = screenWidth;
    if (screenHeight) this.screenHeight = screenHeight;
  }

  /**
   * Process gaze point and generate input events
   */
  processGaze(point: GazePoint): void {
    // Add to smoothing buffer
    this.gazeBuffer.push(point);
    if (this.gazeBuffer.length > this.bufferSize) {
      this.gazeBuffer.shift();
    }

    // Apply smoothing
    const smoothed = this.smoothGaze(this.gazeBuffer);
    this.currentGaze = smoothed;

    // Apply aim assist if enabled
    let finalPoint = smoothed;
    if (this.aimAssistConfig.enabled) {
      finalPoint = this.applyAimAssist(smoothed);
    }

    // Generate mouse event
    const mouseEvent = this.gazeToMouse(finalPoint);
    this.emitMouseEvent(mouseEvent);

    // Check gaming zones
    this.checkZones(finalPoint);
  }

  /**
   * Convert gaze point to mouse event
   */
  gazeToMouse(point: GazePoint): EmulatedMouseEvent {
    const x = point.x * this.screenWidth;
    const y = point.y * this.screenHeight;

    return {
      type: 'move',
      x,
      y,
      timestamp: Date.now(),
    };
  }

  /**
   * Convert gaze to analog stick value
   *
   * Maps gaze position relative to center to stick deflection.
   */
  gazeToAnalogStick(point: GazePoint): AnalogStickValue {
    // Map from center (0.5, 0.5) to stick range (-1 to 1)
    const x = (point.x - 0.5) * 2;
    const y = (point.y - 0.5) * 2;

    // Calculate magnitude (clamped to 1.0)
    const magnitude = Math.min(Math.sqrt(x * x + y * y), 1.0);

    // Calculate angle
    const angle = Math.atan2(y, x) * (180 / Math.PI);

    // Apply deadzone
    const deadzone = 0.15;
    if (magnitude < deadzone) {
      return { x: 0, y: 0, magnitude: 0, angle: 0 };
    }

    // Normalize after deadzone
    const normalizedMagnitude = (magnitude - deadzone) / (1 - deadzone);
    const normalizedX = (x / magnitude) * normalizedMagnitude;
    const normalizedY = (y / magnitude) * normalizedMagnitude;

    return {
      x: normalizedX,
      y: normalizedY,
      magnitude: normalizedMagnitude,
      angle,
    };
  }

  /**
   * Apply aim assist to gaze point
   */
  private applyAimAssist(point: GazePoint): GazePoint {
    if (!this.aimAssistConfig.enabled) return point;

    // Find nearest target
    let nearestTarget: AimTarget | null = null;
    let nearestDistance = Infinity;

    for (const target of this.aimTargets.values()) {
      const pos = target.position as Vector2D;
      const dx = point.x - pos.x;
      const dy = point.y - pos.y;
      const distance = Math.sqrt(dx * dx + dy * dy);

      if (distance < nearestDistance && distance < this.aimAssistConfig.slowdownRadius) {
        nearestDistance = distance;
        nearestTarget = target;
      }
    }

    if (!nearestTarget) {
      this.lockedTarget = null;
      return point;
    }

    // Check for target lock
    if (this.lockedTarget?.id === nearestTarget.id) {
      if (Date.now() - this.lockStartTime > this.aimAssistConfig.lockTime) {
        // Apply snap
        const targetPos = nearestTarget.position as Vector2D;
        return {
          ...point,
          x: point.x + (targetPos.x - point.x) * this.aimAssistConfig.snapStrength,
          y: point.y + (targetPos.y - point.y) * this.aimAssistConfig.snapStrength,
        };
      }
    } else {
      this.lockedTarget = nearestTarget;
      this.lockStartTime = Date.now();
    }

    // Apply slowdown (reduce movement in target area)
    const slowdownFactor = 1 - (this.aimAssistConfig.strength * (1 - nearestDistance / this.aimAssistConfig.slowdownRadius));
    // Would apply to delta movement in a real implementation

    return point;
  }

  /**
   * Aim assist for 3D gaze direction (VR)
   */
  aimAssist(gazeDirection: Vector3D): void {
    // Apply aim assist for 3D VR aiming
    // Would integrate with game's target system
  }

  /**
   * Smooth gaze data
   */
  private smoothGaze(buffer: GazePoint[]): GazePoint {
    if (buffer.length === 0) {
      return { timestamp: 0, x: 0.5, y: 0.5, confidence: 0, valid: false };
    }

    if (buffer.length === 1) {
      return buffer[0];
    }

    // Exponential moving average
    const alpha = 0.3;
    let x = buffer[0].x;
    let y = buffer[0].y;
    let confidence = buffer[0].confidence;

    for (let i = 1; i < buffer.length; i++) {
      x = alpha * buffer[i].x + (1 - alpha) * x;
      y = alpha * buffer[i].y + (1 - alpha) * y;
      confidence = Math.max(confidence, buffer[i].confidence);
    }

    return {
      ...buffer[buffer.length - 1],
      x,
      y,
      confidence,
    };
  }

  /**
   * Register aim target
   */
  registerAimTarget(target: AimTarget): void {
    this.aimTargets.set(target.id, target);
  }

  /**
   * Remove aim target
   */
  removeAimTarget(targetId: string): void {
    this.aimTargets.delete(targetId);
    if (this.lockedTarget?.id === targetId) {
      this.lockedTarget = null;
    }
  }

  /**
   * Clear all aim targets
   */
  clearAimTargets(): void {
    this.aimTargets.clear();
    this.lockedTarget = null;
  }

  /**
   * Configure aim assist
   */
  configureAimAssist(config: Partial<AimAssistConfig>): void {
    this.aimAssistConfig = { ...this.aimAssistConfig, ...config };
  }

  /**
   * Add gaming zone
   */
  addZone(zone: GamingZone): void {
    this.zones.set(zone.id, zone);
  }

  /**
   * Remove gaming zone
   */
  removeZone(zoneId: string): void {
    this.zones.delete(zoneId);
  }

  /**
   * Check if gaze is in any gaming zone
   */
  private checkZones(point: GazePoint): void {
    for (const zone of this.zones.values()) {
      const bounds = zone.bounds;
      if (
        point.x >= bounds.x &&
        point.x <= bounds.x + bounds.width &&
        point.y >= bounds.y &&
        point.y <= bounds.y + bounds.height
      ) {
        this.handleZoneAction(zone, point);
      }
    }
  }

  /**
   * Handle action for a gaming zone
   */
  private handleZoneAction(zone: GamingZone, point: GazePoint): void {
    switch (zone.action.type) {
      case 'move':
        // Emit key press for movement
        break;

      case 'look':
        // Emit mouse movement for camera
        break;

      case 'button':
        // Emit key press
        break;

      case 'scroll':
        this.emitMouseEvent({
          type: 'scroll',
          x: point.x * this.screenWidth,
          y: point.y * this.screenHeight,
          scrollDelta: { x: 0, y: zone.action.speed },
          timestamp: Date.now(),
        });
        break;

      case 'aim':
        // Aim mode
        break;
    }
  }

  /**
   * Trigger dwell click
   */
  triggerClick(button: 'left' | 'right' | 'middle' = 'left'): void {
    if (!this.currentGaze) return;

    const x = this.currentGaze.x * this.screenWidth;
    const y = this.currentGaze.y * this.screenHeight;

    this.emitMouseEvent({ type: 'down', x, y, button, timestamp: Date.now() });
    this.emitMouseEvent({ type: 'up', x, y, button, timestamp: Date.now() });
    this.emitMouseEvent({ type: 'click', x, y, button, timestamp: Date.now() });
  }

  /**
   * Get XR integration status
   */
  get xrGazeIntegration(): XRGazeIntegration {
    return { ...this.xrIntegration };
  }

  /**
   * Initialize XR integration
   */
  initializeXR(platform: 'openXR' | 'steamVR' | 'metaQuest'): boolean {
    // In real implementation: Initialize platform-specific XR SDK
    this.xrIntegration[platform] = true;
    console.log(`Initialized ${platform} gaze integration`);
    return true;
  }

  // Event callbacks

  onMouseEvent(callback: (event: EmulatedMouseEvent) => void): void {
    this.mouseCallbacks.push(callback);
  }

  onGamepadState(callback: (state: GamepadState) => void): void {
    this.gamepadCallbacks.push(callback);
  }

  private emitMouseEvent(event: EmulatedMouseEvent): void {
    for (const callback of this.mouseCallbacks) {
      callback(event);
    }
  }

  /**
   * Emit gamepad state (would be called periodically)
   */
  emitGamepadState(): void {
    if (!this.currentGaze) return;

    const leftStick = this.gazeToAnalogStick(this.currentGaze);

    const state: GamepadState = {
      leftStick,
      rightStick: { x: 0, y: 0, magnitude: 0, angle: 0 },
      leftTrigger: 0,
      rightTrigger: 0,
      dpad: { up: false, down: false, left: false, right: false },
      buttons: {
        a: false, b: false, x: false, y: false,
        lb: false, rb: false,
        back: false, start: false,
        leftStickPress: false, rightStickPress: false,
      },
    };

    for (const callback of this.gamepadCallbacks) {
      callback(state);
    }
  }

  /**
   * Set screen dimensions
   */
  setScreenSize(width: number, height: number): void {
    this.screenWidth = width;
    this.screenHeight = height;
  }

  /**
   * Cleanup
   */
  dispose(): void {
    this.mouseCallbacks = [];
    this.gamepadCallbacks = [];
    this.aimTargets.clear();
    this.zones.clear();
    this.gazeBuffer = [];
  }
}

/**
 * Create default gaming zones for common layouts
 */
export function createDefaultZones(): GamingZone[] {
  return [
    // Movement zone (left side)
    {
      id: 'movement',
      type: 'movement',
      bounds: { x: 0, y: 0.3, width: 0.3, height: 0.4 },
      action: { type: 'move', direction: 'up' },
    },
    // Aim zone (center-right)
    {
      id: 'aim',
      type: 'aim',
      bounds: { x: 0.3, y: 0.2, width: 0.7, height: 0.6 },
      action: { type: 'aim' },
    },
    // Scroll zones (edges)
    {
      id: 'scroll-up',
      type: 'scroll',
      bounds: { x: 0, y: 0, width: 1, height: 0.1 },
      action: { type: 'scroll', speed: -10 },
    },
    {
      id: 'scroll-down',
      type: 'scroll',
      bounds: { x: 0, y: 0.9, width: 1, height: 0.1 },
      action: { type: 'scroll', speed: 10 },
    },
  ];
}

export default GamingIntegration;
