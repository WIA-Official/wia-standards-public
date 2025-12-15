/**
 * WIA Eye Gaze Standard - Mock Adapter
 *
 * Mock adapter for testing and development without physical hardware.
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

import { IEyeTrackerAdapter } from '../IEyeTracker';
import {
  GazePoint,
  GazeCallback,
  EyeTrackerCapabilities,
  CalibrationPoint,
  CalibrationResult,
} from '../../types';

export interface MockAdapterOptions {
  /** Simulated sampling rate (Hz) */
  samplingRate?: number;
  /** Simulate random noise in gaze data */
  addNoise?: boolean;
  /** Noise level (0-1) */
  noiseLevel?: number;
  /** Simulate periodic blinks */
  simulateBlinks?: boolean;
  /** Blink interval (ms) */
  blinkInterval?: number;
  /** Simulate mouse following (for web) */
  followMouse?: boolean;
}

/**
 * Mock Eye Tracker Adapter
 *
 * Generates simulated gaze data for testing and development.
 * Can optionally follow mouse position in browser environments.
 */
export class MockAdapter implements IEyeTrackerAdapter {
  readonly name = 'mock';

  private options: Required<MockAdapterOptions>;
  private connected: boolean = false;
  private streaming: boolean = false;
  private streamInterval: ReturnType<typeof setInterval> | null = null;
  private callback: GazeCallback | null = null;
  private currentX: number = 0.5;
  private currentY: number = 0.5;
  private blinkState: boolean = false;
  private lastBlinkTime: number = 0;

  constructor(options?: MockAdapterOptions) {
    this.options = {
      samplingRate: options?.samplingRate ?? 60,
      addNoise: options?.addNoise ?? true,
      noiseLevel: options?.noiseLevel ?? 0.01,
      simulateBlinks: options?.simulateBlinks ?? true,
      blinkInterval: options?.blinkInterval ?? 4000,
      followMouse: options?.followMouse ?? false,
    };

    if (this.options.followMouse && typeof window !== 'undefined') {
      this.setupMouseTracking();
    }
  }

  private setupMouseTracking(): void {
    if (typeof window !== 'undefined') {
      window.addEventListener('mousemove', (e: MouseEvent) => {
        this.currentX = e.clientX / window.innerWidth;
        this.currentY = e.clientY / window.innerHeight;
      });
    }
  }

  canHandle(_deviceInfo: unknown): boolean {
    return true; // Mock can handle anything
  }

  async initialize(_options?: unknown): Promise<void> {
    // Nothing to initialize for mock
  }

  async connectNative(): Promise<void> {
    await this.simulateDelay(100);
    this.connected = true;
  }

  async disconnectNative(): Promise<void> {
    this.stopNativeStream();
    this.connected = false;
  }

  startNativeStream(callback: GazeCallback): void {
    if (this.streaming) return;

    this.callback = callback;
    this.streaming = true;

    const intervalMs = 1000 / this.options.samplingRate;
    this.streamInterval = setInterval(() => {
      if (this.callback) {
        this.callback(this.generateGazePoint());
      }
    }, intervalMs);
  }

  stopNativeStream(): void {
    if (this.streamInterval) {
      clearInterval(this.streamInterval);
      this.streamInterval = null;
    }
    this.streaming = false;
    this.callback = null;
  }

  getCapabilities(): EyeTrackerCapabilities {
    return {
      device: {
        deviceId: 'mock-tracker-001',
        vendor: 'WIA',
        model: 'Mock Eye Tracker',
        firmwareVersion: '1.0.0',
        protocolVersion: '1.0.0',
        deviceType: 'webcam_based',
      },
      tracking: {
        binocular: true,
        headTracking: false,
        gaze3D: false,
        samplingRate: {
          supported: [30, 60, 120],
          default: 60,
          current: this.options.samplingRate,
        },
        accuracy: { typical: 1.0 },
        precision: { typical: 0.5 },
        latency: { average: 20 },
        operatingDistance: { min: 40, max: 90 },
        trackingArea: { horizontal: 40, vertical: 30 },
      },
      data: {
        gazePoint: true,
        eyeData: true,
        pupilDiameter: true,
        pupilPosition: false,
        eyeOpenness: true,
        eyeImages: false,
        gazeOrigin3D: false,
        gazeDirection3D: false,
        builtInFixationDetection: false,
        builtInSaccadeDetection: false,
        builtInBlinkDetection: false,
        deviceTimestamp: true,
        systemTimestamp: true,
        externalSync: false,
      },
      calibration: {
        required: false,
        types: ['quick'],
        pointOptions: [1, 5],
        defaultPoints: 5,
        autoCalibration: true,
        profileManagement: false,
        qualityAssessment: true,
        adaptiveCalibration: false,
      },
      accessibility: {
        dwellSelection: false,
        blinkInput: false,
        winkInput: false,
        switchEmulation: false,
        adaptiveDwellTime: false,
        adaptiveTargetSize: false,
        errorSmoothing: true,
        tremorCompensation: false,
        aacOptimizedMode: false,
        longSessionOptimization: false,
        fatigueDetection: false,
        breakReminder: false,
      },
      connectivity: {
        connectionTypes: ['usb'],
        apiProtocols: ['wia_standard'],
        multiClient: true,
        remoteConnection: false,
        mobileConnection: false,
      },
      supportedFeatures: [
        'GAZE_POINT',
        'BINOCULAR',
        'PUPIL_DIAMETER',
        'EYE_OPENNESS',
        'CALIBRATION',
        'DEVICE_TIMESTAMP',
      ],
    };
  }

  async calibrate(_points: CalibrationPoint[]): Promise<CalibrationResult> {
    await this.simulateDelay(500);
    return {
      success: true,
      averageAccuracy: 0.8 + Math.random() * 0.4,
      averagePrecision: 0.1 + Math.random() * 0.1,
      timestamp: Date.now(),
      pointResults: _points.map((p, i) => ({
        pointIndex: i,
        position: { x: p.x, y: p.y },
        accuracy: 0.5 + Math.random() * 0.5,
        precision: 0.05 + Math.random() * 0.1,
        valid: true,
      })),
    };
  }

  convertGazeData(nativeData: unknown): GazePoint {
    return nativeData as GazePoint;
  }

  dispose(): void {
    this.disconnectNative();
  }

  // ============================================
  // Private Methods
  // ============================================

  private generateGazePoint(): GazePoint {
    const now = Date.now();

    // Check for blink
    if (this.options.simulateBlinks) {
      if (now - this.lastBlinkTime > this.options.blinkInterval) {
        this.blinkState = true;
        this.lastBlinkTime = now;
        setTimeout(() => { this.blinkState = false; }, 150);
      }
    }

    if (this.blinkState) {
      return this.createBlinkPoint(now);
    }

    let x = this.currentX;
    let y = this.currentY;

    // Add noise
    if (this.options.addNoise) {
      x += (Math.random() - 0.5) * this.options.noiseLevel * 2;
      y += (Math.random() - 0.5) * this.options.noiseLevel * 2;
    }

    // Add smooth movement simulation if not following mouse
    if (!this.options.followMouse) {
      this.currentX += (Math.random() - 0.5) * 0.02;
      this.currentY += (Math.random() - 0.5) * 0.02;
      this.currentX = Math.max(0.1, Math.min(0.9, this.currentX));
      this.currentY = Math.max(0.1, Math.min(0.9, this.currentY));
    }

    const pupilDiameter = 3.5 + Math.random() * 1.0;
    const eyeOpenness = 0.85 + Math.random() * 0.1;

    return {
      timestamp: now,
      x: Math.max(0, Math.min(1, x)),
      y: Math.max(0, Math.min(1, y)),
      confidence: 0.9 + Math.random() * 0.1,
      valid: true,
      leftEye: {
        gaze: { x: x - 0.005, y },
        valid: true,
        pupilDiameter,
        eyeOpenness,
      },
      rightEye: {
        gaze: { x: x + 0.005, y },
        valid: true,
        pupilDiameter: pupilDiameter + 0.1,
        eyeOpenness,
      },
      deviceTimestamp: now,
    };
  }

  private createBlinkPoint(timestamp: number): GazePoint {
    return {
      timestamp,
      x: this.currentX,
      y: this.currentY,
      confidence: 0,
      valid: false,
      leftEye: {
        gaze: { x: this.currentX, y: this.currentY },
        valid: false,
        eyeOpenness: 0,
      },
      rightEye: {
        gaze: { x: this.currentX, y: this.currentY },
        valid: false,
        eyeOpenness: 0,
      },
      deviceTimestamp: timestamp,
      metadata: { invalidReason: 'blink' },
    };
  }

  private simulateDelay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

/**
 * Create a mock adapter for testing
 */
export function createMockAdapter(options?: MockAdapterOptions): MockAdapter {
  return new MockAdapter(options);
}
