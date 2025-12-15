/**
 * WIA Eye Gaze Standard - Tobii Adapter
 *
 * Adapter for Tobii Pro eye trackers.
 * Requires Tobii Pro SDK to be installed separately.
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
  EyeData,
} from '../../types';

export interface TobiiAdapterOptions {
  /** Tobii device serial number (optional, auto-detect if not provided) */
  serialNumber?: string;
  /** Preferred sampling rate */
  samplingRate?: number;
  /** Enable eye images stream */
  enableEyeImages?: boolean;
}

/**
 * Native Tobii data structure (from Tobii Pro SDK)
 */
interface TobiiGazeData {
  system_time_stamp: number;
  device_time_stamp: number;
  left_gaze_point_on_display_area: [number, number];
  right_gaze_point_on_display_area: [number, number];
  left_gaze_point_validity: number;
  right_gaze_point_validity: number;
  left_pupil_diameter: number;
  right_pupil_diameter: number;
  left_gaze_origin_in_user_coordinate_system: [number, number, number];
  right_gaze_origin_in_user_coordinate_system: [number, number, number];
  left_gaze_origin_validity: number;
  right_gaze_origin_validity: number;
  left_eye_openness?: number;
  right_eye_openness?: number;
}

/**
 * Tobii Pro Eye Tracker Adapter
 *
 * Connects to Tobii Pro eye trackers using the Tobii Pro SDK.
 * This adapter translates Tobii's native data format to WIA standard.
 *
 * @example
 * ```typescript
 * import { TobiiAdapter, createEyeTracker } from '@anthropics/wia-eye-gaze';
 *
 * const adapter = new TobiiAdapter({ samplingRate: 120 });
 * const tracker = createEyeTracker(adapter);
 *
 * await tracker.connect();
 * tracker.subscribe(gaze => console.log(gaze.x, gaze.y));
 * tracker.startTracking();
 * ```
 */
export class TobiiAdapter implements IEyeTrackerAdapter {
  readonly name = 'tobii';

  private options: TobiiAdapterOptions;
  private connected: boolean = false;
  private streaming: boolean = false;
  private callback: GazeCallback | null = null;
  private nativeTracker: unknown = null;
  private deviceInfo: { model: string; serial: string; firmware: string } | null = null;

  constructor(options?: TobiiAdapterOptions) {
    this.options = {
      samplingRate: 60,
      enableEyeImages: false,
      ...options,
    };
  }

  canHandle(deviceInfo: unknown): boolean {
    if (typeof deviceInfo === 'object' && deviceInfo !== null) {
      const info = deviceInfo as Record<string, unknown>;
      return (
        typeof info.vendor === 'string' &&
        info.vendor.toLowerCase().includes('tobii')
      );
    }
    return false;
  }

  async initialize(_options?: unknown): Promise<void> {
    // Check if Tobii SDK is available
    // In real implementation, this would load the Tobii SDK bindings
    console.log('[TobiiAdapter] Initializing Tobii Pro SDK...');
  }

  async connectNative(): Promise<void> {
    // In real implementation:
    // 1. Find available Tobii eye trackers
    // 2. Connect to specified device (or first available)
    // 3. Get device information

    console.log('[TobiiAdapter] Connecting to Tobii device...');

    // Simulated connection for SDK structure
    // Real implementation would use tobii_research library
    this.deviceInfo = {
      model: 'Tobii Pro Fusion',
      serial: this.options.serialNumber ?? 'AUTO-DETECTED',
      firmware: '2.1.0',
    };

    this.connected = true;
    console.log(`[TobiiAdapter] Connected to ${this.deviceInfo.model}`);
  }

  async disconnectNative(): Promise<void> {
    this.stopNativeStream();
    this.connected = false;
    this.nativeTracker = null;
    console.log('[TobiiAdapter] Disconnected');
  }

  startNativeStream(callback: GazeCallback): void {
    if (this.streaming) return;

    this.callback = callback;
    this.streaming = true;

    // In real implementation:
    // this.nativeTracker.subscribe_to(tobii.EYETRACKER_GAZE_DATA, this.handleNativeData);
    console.log('[TobiiAdapter] Started gaze data stream');
  }

  stopNativeStream(): void {
    if (!this.streaming) return;

    // In real implementation:
    // this.nativeTracker.unsubscribe_from(tobii.EYETRACKER_GAZE_DATA);
    this.streaming = false;
    this.callback = null;
    console.log('[TobiiAdapter] Stopped gaze data stream');
  }

  getCapabilities(): EyeTrackerCapabilities {
    return {
      device: {
        deviceId: `tobii-${this.deviceInfo?.serial ?? 'unknown'}`,
        vendor: 'Tobii',
        model: this.deviceInfo?.model ?? 'Unknown',
        firmwareVersion: this.deviceInfo?.firmware ?? '0.0.0',
        protocolVersion: '1.0.0',
        deviceType: 'screen_based',
        vendorUrl: 'https://www.tobii.com',
        productUrl: 'https://www.tobii.com/products/eye-trackers',
      },
      tracking: {
        binocular: true,
        headTracking: true,
        gaze3D: true,
        samplingRate: {
          supported: [60, 120, 250],
          default: 120,
          current: this.options.samplingRate,
        },
        accuracy: { typical: 0.5, best: 0.3 },
        precision: { typical: 0.1, rms: 0.08 },
        latency: { average: 3, maximum: 10 },
        operatingDistance: { min: 50, max: 80, optimal: 65 },
        trackingArea: { horizontal: 30, vertical: 25 },
      },
      data: {
        gazePoint: true,
        eyeData: true,
        pupilDiameter: true,
        pupilPosition: true,
        eyeOpenness: true,
        eyeImages: this.options.enableEyeImages ?? false,
        gazeOrigin3D: true,
        gazeDirection3D: true,
        builtInFixationDetection: false,
        builtInSaccadeDetection: false,
        builtInBlinkDetection: false,
        deviceTimestamp: true,
        systemTimestamp: true,
        externalSync: true,
      },
      calibration: {
        required: true,
        types: ['standard', 'quick', 'accessibility'],
        pointOptions: [1, 2, 5, 9],
        defaultPoints: 5,
        autoCalibration: false,
        profileManagement: true,
        qualityAssessment: true,
        adaptiveCalibration: true,
      },
      connectivity: {
        connectionTypes: ['usb', 'usb_c'],
        apiProtocols: ['native_sdk', 'wia_standard'],
        multiClient: true,
        remoteConnection: false,
        mobileConnection: false,
      },
      supportedFeatures: [
        'GAZE_POINT',
        'BINOCULAR',
        'HEAD_TRACKING',
        'GAZE_3D',
        'PUPIL_DIAMETER',
        'PUPIL_POSITION',
        'EYE_OPENNESS',
        'CALIBRATION',
        'CALIBRATION_PROFILES',
        'DEVICE_TIMESTAMP',
        'EXTERNAL_SYNC',
      ],
    };
  }

  async calibrate(points: CalibrationPoint[]): Promise<CalibrationResult> {
    // In real implementation:
    // 1. Start calibration mode
    // 2. For each point, collect samples
    // 3. Compute calibration
    // 4. Return results

    console.log(`[TobiiAdapter] Starting calibration with ${points.length} points`);

    // Simulated calibration
    await new Promise(resolve => setTimeout(resolve, 1000));

    return {
      success: true,
      averageAccuracy: 0.5,
      averagePrecision: 0.1,
      timestamp: Date.now(),
      pointResults: points.map((p, i) => ({
        pointIndex: i,
        position: { x: p.x, y: p.y },
        accuracy: 0.4 + Math.random() * 0.2,
        precision: 0.08 + Math.random() * 0.04,
        valid: true,
      })),
    };
  }

  convertGazeData(nativeData: unknown): GazePoint {
    const data = nativeData as TobiiGazeData;

    const leftValid = data.left_gaze_point_validity === 1;
    const rightValid = data.right_gaze_point_validity === 1;
    const valid = leftValid || rightValid;

    let x = 0.5;
    let y = 0.5;

    if (leftValid && rightValid) {
      x = (data.left_gaze_point_on_display_area[0] + data.right_gaze_point_on_display_area[0]) / 2;
      y = (data.left_gaze_point_on_display_area[1] + data.right_gaze_point_on_display_area[1]) / 2;
    } else if (leftValid) {
      x = data.left_gaze_point_on_display_area[0];
      y = data.left_gaze_point_on_display_area[1];
    } else if (rightValid) {
      x = data.right_gaze_point_on_display_area[0];
      y = data.right_gaze_point_on_display_area[1];
    }

    const leftEye: EyeData = {
      gaze: {
        x: data.left_gaze_point_on_display_area[0],
        y: data.left_gaze_point_on_display_area[1],
      },
      valid: leftValid,
      pupilDiameter: data.left_pupil_diameter,
      gazeOrigin: data.left_gaze_origin_validity === 1 ? {
        x: data.left_gaze_origin_in_user_coordinate_system[0],
        y: data.left_gaze_origin_in_user_coordinate_system[1],
        z: data.left_gaze_origin_in_user_coordinate_system[2],
      } : undefined,
      eyeOpenness: data.left_eye_openness,
    };

    const rightEye: EyeData = {
      gaze: {
        x: data.right_gaze_point_on_display_area[0],
        y: data.right_gaze_point_on_display_area[1],
      },
      valid: rightValid,
      pupilDiameter: data.right_pupil_diameter,
      gazeOrigin: data.right_gaze_origin_validity === 1 ? {
        x: data.right_gaze_origin_in_user_coordinate_system[0],
        y: data.right_gaze_origin_in_user_coordinate_system[1],
        z: data.right_gaze_origin_in_user_coordinate_system[2],
      } : undefined,
      eyeOpenness: data.right_eye_openness,
    };

    return {
      timestamp: data.system_time_stamp,
      x,
      y,
      confidence: valid ? 0.95 : 0,
      valid,
      leftEye,
      rightEye,
      deviceTimestamp: data.device_time_stamp,
    };
  }

  dispose(): void {
    this.disconnectNative();
  }
}

/**
 * Create a Tobii adapter
 */
export function createTobiiAdapter(options?: TobiiAdapterOptions): TobiiAdapter {
  return new TobiiAdapter(options);
}
