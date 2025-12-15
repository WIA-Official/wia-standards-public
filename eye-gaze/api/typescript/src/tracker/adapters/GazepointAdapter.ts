/**
 * WIA Eye Gaze Standard - Gazepoint Adapter
 *
 * Adapter for Gazepoint GP3/GP3 HD eye trackers.
 * Uses Gazepoint's Open Gaze API over TCP/IP.
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

export interface GazepointAdapterOptions {
  /** Gazepoint server host */
  host?: string;
  /** Gazepoint server port */
  port?: number;
  /** Sampling rate (30, 60, or 150 Hz) */
  samplingRate?: number;
}

/**
 * Gazepoint Open Gaze API data record
 */
interface GazepointRecord {
  TIME: number;
  FPOGX: number;
  FPOGY: number;
  FPOGV: number;
  LPOGX: number;
  LPOGY: number;
  LPOGV: number;
  RPOGX: number;
  RPOGY: number;
  RPOGV: number;
  LPCX: number;
  LPCY: number;
  LPD: number;
  LPS: number;
  RPCX: number;
  RPCY: number;
  RPD: number;
  RPS: number;
  LEYEX: number;
  LEYEY: number;
  LEYEZ: number;
  REYEX: number;
  REYEY: number;
  REYEZ: number;
}

/**
 * Gazepoint Eye Tracker Adapter
 *
 * Connects to Gazepoint eye trackers using the Open Gaze API.
 * Communication is over TCP/IP with XML-formatted messages.
 *
 * @example
 * ```typescript
 * import { GazepointAdapter, createEyeTracker } from '@anthropics/wia-eye-gaze';
 *
 * const adapter = new GazepointAdapter({ port: 4242 });
 * const tracker = createEyeTracker(adapter);
 *
 * await tracker.connect();
 * tracker.subscribe(gaze => console.log(gaze.x, gaze.y));
 * tracker.startTracking();
 * ```
 */
export class GazepointAdapter implements IEyeTrackerAdapter {
  readonly name = 'gazepoint';

  private options: Required<GazepointAdapterOptions>;
  private connected: boolean = false;
  private streaming: boolean = false;
  private callback: GazeCallback | null = null;
  private socket: unknown = null;

  constructor(options?: GazepointAdapterOptions) {
    this.options = {
      host: options?.host ?? '127.0.0.1',
      port: options?.port ?? 4242,
      samplingRate: options?.samplingRate ?? 60,
    };
  }

  canHandle(deviceInfo: unknown): boolean {
    if (typeof deviceInfo === 'object' && deviceInfo !== null) {
      const info = deviceInfo as Record<string, unknown>;
      return (
        typeof info.vendor === 'string' &&
        info.vendor.toLowerCase().includes('gazepoint')
      );
    }
    return false;
  }

  async initialize(_options?: unknown): Promise<void> {
    console.log('[GazepointAdapter] Initializing...');
  }

  async connectNative(): Promise<void> {
    console.log(`[GazepointAdapter] Connecting to ${this.options.host}:${this.options.port}...`);

    // In real implementation:
    // 1. Create TCP socket connection
    // 2. Send initial commands to configure data streams
    // 3. Enable required data channels

    // Example commands:
    // <SET ID="ENABLE_SEND_POG_FIX" STATE="1" />
    // <SET ID="ENABLE_SEND_EYE_LEFT" STATE="1" />
    // <SET ID="ENABLE_SEND_EYE_RIGHT" STATE="1" />
    // <SET ID="ENABLE_SEND_PUPIL_LEFT" STATE="1" />
    // <SET ID="ENABLE_SEND_PUPIL_RIGHT" STATE="1" />
    // <SET ID="ENABLE_SEND_TIME" STATE="1" />

    this.connected = true;
    console.log('[GazepointAdapter] Connected');
  }

  async disconnectNative(): Promise<void> {
    this.stopNativeStream();

    // In real implementation: close TCP socket
    this.socket = null;
    this.connected = false;
    console.log('[GazepointAdapter] Disconnected');
  }

  startNativeStream(callback: GazeCallback): void {
    if (this.streaming) return;

    this.callback = callback;
    this.streaming = true;

    // In real implementation:
    // 1. Send <SET ID="ENABLE_SEND_DATA" STATE="1" />
    // 2. Listen for incoming REC messages
    // 3. Parse XML and convert to GazePoint

    console.log('[GazepointAdapter] Started data stream');
  }

  stopNativeStream(): void {
    if (!this.streaming) return;

    // In real implementation:
    // Send <SET ID="ENABLE_SEND_DATA" STATE="0" />

    this.streaming = false;
    this.callback = null;
    console.log('[GazepointAdapter] Stopped data stream');
  }

  getCapabilities(): EyeTrackerCapabilities {
    return {
      device: {
        deviceId: 'gazepoint-gp3',
        vendor: 'Gazepoint',
        model: 'GP3 HD',
        firmwareVersion: '1.0.0',
        protocolVersion: '1.0.0',
        deviceType: 'screen_based',
        vendorUrl: 'https://www.gazept.com',
        productUrl: 'https://www.gazept.com/product/gp3hd/',
      },
      tracking: {
        binocular: true,
        headTracking: false,
        gaze3D: true,
        samplingRate: {
          supported: [30, 60, 150],
          default: 60,
          current: this.options.samplingRate,
        },
        accuracy: { typical: 0.5 },
        precision: { typical: 0.1 },
        latency: { average: 20 },
        operatingDistance: { min: 50, max: 80 },
        trackingArea: { horizontal: 35, vertical: 22 },
      },
      data: {
        gazePoint: true,
        eyeData: true,
        pupilDiameter: true,
        pupilPosition: true,
        eyeOpenness: false,
        eyeImages: false,
        gazeOrigin3D: true,
        gazeDirection3D: false,
        builtInFixationDetection: true,
        builtInSaccadeDetection: false,
        builtInBlinkDetection: false,
        deviceTimestamp: true,
        systemTimestamp: true,
        externalSync: false,
      },
      calibration: {
        required: true,
        types: ['standard', 'quick'],
        pointOptions: [5, 9],
        defaultPoints: 5,
        autoCalibration: false,
        profileManagement: false,
        qualityAssessment: false,
        adaptiveCalibration: false,
      },
      connectivity: {
        connectionTypes: ['usb'],
        apiProtocols: ['tcp_ip', 'wia_standard'],
        multiClient: false,
        remoteConnection: true,
        mobileConnection: false,
      },
      supportedFeatures: [
        'GAZE_POINT',
        'BINOCULAR',
        'GAZE_3D',
        'PUPIL_DIAMETER',
        'PUPIL_POSITION',
        'FIXATION_DETECTION',
        'CALIBRATION',
        'DEVICE_TIMESTAMP',
      ],
    };
  }

  async calibrate(points: CalibrationPoint[]): Promise<CalibrationResult> {
    console.log(`[GazepointAdapter] Starting calibration with ${points.length} points`);

    // In real implementation:
    // 1. Send <SET ID="CALIBRATE_START" STATE="1" />
    // 2. For each point:
    //    - Send <SET ID="CALIBRATE_SHOW" X="0.5" Y="0.5" />
    //    - Wait for user to fixate
    //    - Send <SET ID="CALIBRATE_ADDPOINT" />
    // 3. Compute calibration
    // 4. Send <SET ID="CALIBRATE_RESULT_SUMMARY" />

    await new Promise(resolve => setTimeout(resolve, 500));

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
    const data = nativeData as GazepointRecord;

    const leftValid = data.LPOGV > 0;
    const rightValid = data.RPOGV > 0;
    const fixationValid = data.FPOGV > 0;
    const valid = fixationValid || leftValid || rightValid;

    let x = data.FPOGX;
    let y = data.FPOGY;

    if (!fixationValid) {
      if (leftValid && rightValid) {
        x = (data.LPOGX + data.RPOGX) / 2;
        y = (data.LPOGY + data.RPOGY) / 2;
      } else if (leftValid) {
        x = data.LPOGX;
        y = data.LPOGY;
      } else if (rightValid) {
        x = data.RPOGX;
        y = data.RPOGY;
      }
    }

    return {
      timestamp: Math.floor(data.TIME * 1000),
      x,
      y,
      confidence: valid ? 0.9 : 0,
      valid,
      fixation: fixationValid,
      leftEye: {
        gaze: { x: data.LPOGX, y: data.LPOGY },
        valid: leftValid,
        pupilDiameter: data.LPD * 1000, // Convert to mm
        pupilCenter: { x: data.LPCX, y: data.LPCY },
        gazeOrigin: {
          x: data.LEYEX,
          y: data.LEYEY,
          z: data.LEYEZ,
        },
      },
      rightEye: {
        gaze: { x: data.RPOGX, y: data.RPOGY },
        valid: rightValid,
        pupilDiameter: data.RPD * 1000,
        pupilCenter: { x: data.RPCX, y: data.RPCY },
        gazeOrigin: {
          x: data.REYEX,
          y: data.REYEY,
          z: data.REYEZ,
        },
      },
      deviceTimestamp: Math.floor(data.TIME * 1000),
    };
  }

  dispose(): void {
    this.disconnectNative();
  }

  // ============================================
  // Gazepoint-specific API Commands
  // ============================================

  /**
   * Send a GET command to the Gazepoint server
   */
  sendGetCommand(_id: string): void {
    // In real implementation:
    // const cmd = `<GET ID="${id}" />\r\n`;
    // this.socket.write(cmd);
  }

  /**
   * Send a SET command to the Gazepoint server
   */
  sendSetCommand(_id: string, _state: string | number): void {
    // In real implementation:
    // const cmd = `<SET ID="${id}" STATE="${state}" />\r\n`;
    // this.socket.write(cmd);
  }
}

/**
 * Create a Gazepoint adapter
 */
export function createGazepointAdapter(options?: GazepointAdapterOptions): GazepointAdapter {
  return new GazepointAdapter(options);
}
