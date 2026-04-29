/**
 * WIA-SEMI-017 3D Touch SDK
 * 
 * Main SDK for force sensing and haptic feedback implementation.
 */

import {
  Force3DConfig,
  ForceMeasurement,
  ForceGestureEvent,
  ForceGestureType,
  ForceEventListener,
  HapticWaveform,
  HapticCallback,
  ForceGrams,
  TimestampMs,
  ForceSensorType,
  HapticPattern,
  HapticIntensity
} from './types';

/**
 * Main 3D Touch controller class
 */
export class Force3DController {
  private config: Force3DConfig;
  private listeners: Map<ForceGestureType, Set<ForceEventListener>>;
  private currentForce: number = 0;
  private gestureState: ForceGestureType = ForceGestureType.RELEASE;
  private gestureStartTime: TimestampMs = 0;
  
  constructor(config: Force3DConfig) {
    this.config = config;
    this.listeners = new Map();
  }
  
  /**
   * Initialize force sensing
   */
  public async initialize(): Promise<void> {
    // Initialize hardware, load calibration
    console.log('Initializing 3D Touch with config:', this.config);
    // Implementation would connect to actual hardware
  }
  
  /**
   * Update force measurement
   */
  public updateForce(measurement: ForceMeasurement): void {
    this.currentForce = measurement.force;
    const gesture = this.detectGesture(measurement);
    
    if (gesture && gesture.type !== this.gestureState) {
      this.gestureState = gesture.type;
      this.gestureStartTime = measurement.timestamp;
      this.emitGestureEvent(gesture);
    }
  }
  
  /**
   * Detect force gesture from measurement
   */
  private detectGesture(measurement: ForceMeasurement): ForceGestureEvent | null {
    const { force, timestamp } = measurement;
    const { thresholds } = this.config;
    
    let gestureType: ForceGestureType;
    
    if (force < thresholds.touch) {
      gestureType = ForceGestureType.HOVER;
    } else if (force >= thresholds.pop) {
      gestureType = ForceGestureType.POP;
    } else if (force >= thresholds.peek) {
      gestureType = ForceGestureType.PEEK;
    } else {
      gestureType = ForceGestureType.HOVER;
    }
    
    if (force < thresholds.touch - thresholds.hysteresis) {
      gestureType = ForceGestureType.RELEASE;
    }
    
    return {
      type: gestureType,
      force: measurement,
      startTime: this.gestureStartTime,
      duration: timestamp - this.gestureStartTime
    };
  }
  
  /**
   * Register force gesture event listener
   */
  public on(gestureType: ForceGestureType, listener: ForceEventListener): void {
    if (!this.listeners.has(gestureType)) {
      this.listeners.set(gestureType, new Set());
    }
    this.listeners.get(gestureType)!.add(listener);
  }
  
  /**
   * Unregister event listener
   */
  public off(gestureType: ForceGestureType, listener: ForceEventListener): void {
    const listeners = this.listeners.get(gestureType);
    if (listeners) {
      listeners.delete(listener);
    }
  }
  
  /**
   * Emit gesture event to listeners
   */
  private emitGestureEvent(event: ForceGestureEvent): void {
    const listeners = this.listeners.get(event.type);
    if (listeners) {
      listeners.forEach(listener => listener(event));
    }
  }
  
  /**
   * Trigger haptic feedback
   */
  public async triggerHaptic(
    waveform: HapticWaveform,
    callback?: HapticCallback
  ): Promise<void> {
    console.log('Triggering haptic:', waveform);
    // Implementation would drive actual haptic hardware
    if (callback) {
      setTimeout(callback, waveform.duration);
    }
  }
  
  /**
   * Get current force measurement
   */
  public getCurrentForce(): ForceGrams {
    return this.currentForce;
  }
  
  /**
   * Update force thresholds
   */
  public updateThresholds(thresholds: Partial<typeof this.config.thresholds>): void {
    this.config.thresholds = { ...this.config.thresholds, ...thresholds };
  }
  
  /**
   * Calibrate force sensor
   */
  public async calibrate(calibrationPoints: typeof this.config.sensor.calibration): Promise<void> {
    this.config.sensor.calibration = calibrationPoints;
    console.log('Calibration updated with', calibrationPoints.length, 'points');
  }
}

/**
 * Utility: Create default 3D Touch configuration
 */
export function createDefaultConfig(): Force3DConfig {
  return {
    sensor: {
      type: ForceSensorType.CAPACITIVE,
      calibration: [
        { force: 0, rawValue: 0 },
        { force: 50, rawValue: 512 },
        { force: 150, rawValue: 1536 },
        { force: 250, rawValue: 2560 },
        { force: 400, rawValue: 4096 }
      ],
      maxForce: 500,
      minForce: 1,
      samplingRate: 120,
      resolution: 1024,
      temperatureCoefficient: -0.003
    },
    thresholds: {
      touch: 30,
      peek: 200,
      pop: 400,
      max: 500,
      hysteresis: 15
    },
    haptic: {
      type: 'lra' as any,
      resonantFrequency: 175,
      maxAcceleration: 3,
      responseTime: 12,
      powerConsumption: 150
    },
    adaptiveThresholds: true,
    temperatureCompensation: true
  };
}

/**
 * Utility: Create haptic waveform presets
 */
export const HapticPresets = {
  click: (): HapticWaveform => ({
    pattern: HapticPattern.CLICK,
    intensity: HapticIntensity.MEDIUM,
    duration: 12,
    riseTime: 3,
    decayTime: 9,
    amplitude: 3.5
  }),
  
  thud: (): HapticWaveform => ({
    pattern: HapticPattern.THUD,
    intensity: HapticIntensity.HEAVY,
    duration: 30,
    riseTime: 8,
    decayTime: 22,
    amplitude: 2.5
  }),
  
  tick: (): HapticWaveform => ({
    pattern: HapticPattern.TICK,
    intensity: HapticIntensity.LIGHT,
    duration: 8,
    riseTime: 2,
    decayTime: 6,
    amplitude: 1.5
  })
};

// Export all types
export * from './types';
