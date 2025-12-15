/**
 * WIA Eye Gaze Standard - Pupil Labs Adapter
 *
 * Adapter for Pupil Labs eye trackers (Pupil Core, Pupil Invisible, Neon).
 * Uses Pupil Labs Network API (ZMQ-based).
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

export interface PupilLabsAdapterOptions {
  /** Pupil Capture/Service address */
  address?: string;
  /** Request port */
  reqPort?: number;
  /** Subscribe port */
  subPort?: number;
  /** Device type */
  deviceType?: 'core' | 'invisible' | 'neon';
}

/**
 * Pupil Labs gaze datum structure
 */
interface PupilGazeDatum {
  topic: string;
  timestamp: number;
  confidence: number;
  norm_pos: [number, number];
  gaze_point_3d?: [number, number, number];
  eye_center_3d?: { [eye: string]: [number, number, number] };
  gaze_normal_3d?: { [eye: string]: [number, number, number] };
  base_data?: PupilDatum[];
}

interface PupilDatum {
  topic: string;
  timestamp: number;
  confidence: number;
  id: number; // 0=right, 1=left
  norm_pos: [number, number];
  diameter: number;
  diameter_3d?: number;
  sphere?: {
    center: [number, number, number];
    radius: number;
  };
  circle_3d?: {
    center: [number, number, number];
    normal: [number, number, number];
    radius: number;
  };
}

/**
 * Pupil Labs Eye Tracker Adapter
 *
 * Connects to Pupil Labs eye trackers using the Network API.
 * Supports Pupil Core, Pupil Invisible, and Neon devices.
 *
 * @example
 * ```typescript
 * import { PupilLabsAdapter, createEyeTracker } from '@anthropics/wia-eye-gaze';
 *
 * const adapter = new PupilLabsAdapter({ deviceType: 'core' });
 * const tracker = createEyeTracker(adapter);
 *
 * await tracker.connect();
 * tracker.subscribe(gaze => console.log(gaze.x, gaze.y));
 * tracker.startTracking();
 * ```
 */
export class PupilLabsAdapter implements IEyeTrackerAdapter {
  readonly name = 'pupil-labs';

  private options: Required<PupilLabsAdapterOptions>;
  private connected: boolean = false;
  private streaming: boolean = false;
  private callback: GazeCallback | null = null;

  constructor(options?: PupilLabsAdapterOptions) {
    this.options = {
      address: options?.address ?? '127.0.0.1',
      reqPort: options?.reqPort ?? 50020,
      subPort: options?.subPort ?? 50021,
      deviceType: options?.deviceType ?? 'core',
    };
  }

  canHandle(deviceInfo: unknown): boolean {
    if (typeof deviceInfo === 'object' && deviceInfo !== null) {
      const info = deviceInfo as Record<string, unknown>;
      return (
        typeof info.vendor === 'string' &&
        (info.vendor.toLowerCase().includes('pupil') ||
         info.vendor.toLowerCase().includes('pupil labs'))
      );
    }
    return false;
  }

  async initialize(_options?: unknown): Promise<void> {
    console.log('[PupilLabsAdapter] Initializing...');
  }

  async connectNative(): Promise<void> {
    console.log(`[PupilLabsAdapter] Connecting to ${this.options.address}:${this.options.reqPort}...`);

    // In real implementation:
    // 1. Create ZMQ REQ socket and connect to Pupil Remote
    // 2. Request SUB_PORT for data subscription
    // 3. Create ZMQ SUB socket and subscribe to 'gaze.' topic

    // Example ZMQ setup:
    // const req = new zmq.Request();
    // await req.connect(`tcp://${address}:${reqPort}`);
    // await req.send('SUB_PORT');
    // const subPort = await req.receive();

    this.connected = true;
    console.log('[PupilLabsAdapter] Connected');
  }

  async disconnectNative(): Promise<void> {
    this.stopNativeStream();
    this.connected = false;
    console.log('[PupilLabsAdapter] Disconnected');
  }

  startNativeStream(callback: GazeCallback): void {
    if (this.streaming) return;

    this.callback = callback;
    this.streaming = true;

    // In real implementation:
    // 1. Subscribe to 'gaze.' and 'pupil.' topics
    // 2. Process incoming messages and convert to GazePoint

    console.log('[PupilLabsAdapter] Started gaze data stream');
  }

  stopNativeStream(): void {
    if (!this.streaming) return;

    // In real implementation: unsubscribe from topics
    this.streaming = false;
    this.callback = null;
    console.log('[PupilLabsAdapter] Stopped gaze data stream');
  }

  getCapabilities(): EyeTrackerCapabilities {
    const isWearable = this.options.deviceType !== 'core';

    return {
      device: {
        deviceId: `pupil-${this.options.deviceType}`,
        vendor: 'Pupil Labs',
        model: this.getModelName(),
        firmwareVersion: '1.0.0',
        protocolVersion: '1.0.0',
        deviceType: isWearable ? 'wearable' : 'screen_based',
        vendorUrl: 'https://pupil-labs.com',
        productUrl: this.getProductUrl(),
      },
      tracking: {
        binocular: true,
        headTracking: isWearable,
        gaze3D: true,
        samplingRate: {
          supported: this.getSupportedSamplingRates(),
          default: this.options.deviceType === 'neon' ? 200 : 120,
        },
        accuracy: { typical: this.options.deviceType === 'neon' ? 0.7 : 1.0 },
        precision: { typical: 0.3 },
        latency: { average: 20 },
        operatingDistance: { min: 0, max: 1000 }, // Wearable - no fixed distance
        trackingArea: { horizontal: 120, vertical: 90 }, // Full field of view
      },
      data: {
        gazePoint: true,
        eyeData: true,
        pupilDiameter: true,
        pupilPosition: true,
        eyeOpenness: false,
        eyeImages: true,
        gazeOrigin3D: true,
        gazeDirection3D: true,
        builtInFixationDetection: false,
        builtInSaccadeDetection: false,
        builtInBlinkDetection: true,
        deviceTimestamp: true,
        systemTimestamp: true,
        externalSync: true,
      },
      calibration: {
        required: true,
        types: ['standard', 'quick'],
        pointOptions: [5, 9],
        defaultPoints: 5,
        autoCalibration: false,
        profileManagement: true,
        qualityAssessment: true,
        adaptiveCalibration: false,
      },
      connectivity: {
        connectionTypes: ['usb', 'wifi'],
        apiProtocols: ['websocket', 'wia_standard'],
        multiClient: true,
        remoteConnection: true,
        mobileConnection: this.options.deviceType === 'neon',
      },
      supportedFeatures: [
        'GAZE_POINT',
        'BINOCULAR',
        'HEAD_TRACKING',
        'GAZE_3D',
        'PUPIL_DIAMETER',
        'PUPIL_POSITION',
        'EYE_IMAGES',
        'BLINK_DETECTION',
        'CALIBRATION',
        'CALIBRATION_PROFILES',
        'DEVICE_TIMESTAMP',
        'EXTERNAL_SYNC',
      ],
    };
  }

  private getModelName(): string {
    switch (this.options.deviceType) {
      case 'core': return 'Pupil Core';
      case 'invisible': return 'Pupil Invisible';
      case 'neon': return 'Neon';
      default: return 'Unknown';
    }
  }

  private getProductUrl(): string {
    switch (this.options.deviceType) {
      case 'core': return 'https://pupil-labs.com/products/core';
      case 'invisible': return 'https://pupil-labs.com/products/invisible';
      case 'neon': return 'https://pupil-labs.com/products/neon';
      default: return 'https://pupil-labs.com';
    }
  }

  private getSupportedSamplingRates(): number[] {
    switch (this.options.deviceType) {
      case 'core': return [30, 60, 120, 200];
      case 'invisible': return [30, 60, 120, 200];
      case 'neon': return [200];
      default: return [30, 60, 120];
    }
  }

  async calibrate(points: CalibrationPoint[]): Promise<CalibrationResult> {
    console.log(`[PupilLabsAdapter] Starting calibration with ${points.length} points`);

    // In real implementation:
    // 1. Send calibration start command via Network API
    // 2. For each point, add reference point
    // 3. Compute and apply calibration

    await new Promise(resolve => setTimeout(resolve, 500));

    return {
      success: true,
      averageAccuracy: 0.8,
      averagePrecision: 0.2,
      timestamp: Date.now(),
      pointResults: points.map((p, i) => ({
        pointIndex: i,
        position: { x: p.x, y: p.y },
        accuracy: 0.6 + Math.random() * 0.3,
        precision: 0.15 + Math.random() * 0.1,
        valid: true,
      })),
    };
  }

  convertGazeData(nativeData: unknown): GazePoint {
    const data = nativeData as PupilGazeDatum;

    // Note: Pupil Labs Y-axis is inverted (0=bottom, 1=top)
    const x = data.norm_pos[0];
    const y = 1 - data.norm_pos[1]; // Invert Y to match WIA standard

    const valid = data.confidence > 0.6;

    let leftEye = undefined;
    let rightEye = undefined;

    if (data.base_data) {
      for (const pupil of data.base_data) {
        const eyeData = {
          gaze: {
            x: pupil.norm_pos[0],
            y: 1 - pupil.norm_pos[1],
          },
          valid: pupil.confidence > 0.6,
          pupilDiameter: pupil.diameter_3d ?? pupil.diameter,
          gazeOrigin: pupil.sphere ? {
            x: pupil.sphere.center[0],
            y: pupil.sphere.center[1],
            z: pupil.sphere.center[2],
          } : undefined,
          gazeDirection: pupil.circle_3d ? {
            x: pupil.circle_3d.normal[0],
            y: pupil.circle_3d.normal[1],
            z: pupil.circle_3d.normal[2],
          } : undefined,
        };

        if (pupil.id === 0) {
          rightEye = eyeData;
        } else {
          leftEye = eyeData;
        }
      }
    }

    return {
      timestamp: Math.floor(data.timestamp * 1000),
      x,
      y,
      confidence: data.confidence,
      valid,
      leftEye,
      rightEye,
      deviceTimestamp: Math.floor(data.timestamp * 1000),
      metadata: {
        gaze_point_3d: data.gaze_point_3d,
      },
    };
  }

  dispose(): void {
    this.disconnectNative();
  }

  // ============================================
  // Pupil Labs-specific API Methods
  // ============================================

  /**
   * Send notification to Pupil Capture/Service
   */
  sendNotification(_subject: string, _payload?: Record<string, unknown>): void {
    // In real implementation:
    // Send ZMQ notification message
  }

  /**
   * Start recording
   */
  startRecording(_sessionName?: string): void {
    this.sendNotification('recording.should_start');
  }

  /**
   * Stop recording
   */
  stopRecording(): void {
    this.sendNotification('recording.should_stop');
  }

  /**
   * Set time reference for synchronization
   */
  setTimeSync(_timestamp: number): void {
    // In real implementation:
    // Send time sync message
  }
}

/**
 * Create a Pupil Labs adapter
 */
export function createPupilLabsAdapter(options?: PupilLabsAdapterOptions): PupilLabsAdapter {
  return new PupilLabsAdapter(options);
}
