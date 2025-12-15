/**
 * WIA Eye Gaze Standard - Eye Tracker Implementation
 *
 * Main implementation of the IEyeTracker interface.
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

import { IEyeTracker, IEyeTrackerAdapter } from './IEyeTracker';
import {
  GazePoint,
  GazeEvent,
  GazeEventType,
  EyeTrackerCapabilities,
  TrackerStatus,
  TrackerState,
  TrackerConfig,
  CalibrationPoint,
  CalibrationResult,
  CalibrationQuality,
  Subscription,
  GazeCallback,
  EventCallback,
} from '../types';

type EventHandler = EventCallback<GazeEvent>;

/**
 * WIA Eye Tracker - Standard implementation
 *
 * This class provides a unified interface to eye tracking devices
 * through device-specific adapters.
 */
export class WiaEyeTracker implements IEyeTracker {
  private adapter: IEyeTrackerAdapter;
  private config: TrackerConfig;
  private status: TrackerStatus;
  private subscriptions: Map<string, GazeCallback>;
  private eventHandlers: Map<GazeEventType, Set<EventHandler>>;
  private latestGaze: GazePoint | null;
  private calibrationResult: CalibrationResult | null;
  private subscriptionCounter: number;

  constructor(adapter: IEyeTrackerAdapter, config?: TrackerConfig) {
    this.adapter = adapter;
    this.config = {
      samplingRate: 60,
      smoothing: false,
      smoothingFactor: 0.3,
      autoReconnect: true,
      reconnectInterval: 5000,
      ...config,
    };

    this.status = {
      state: 'disconnected',
      connected: false,
      tracking: false,
      calibrated: false,
    };

    this.subscriptions = new Map();
    this.eventHandlers = new Map();
    this.latestGaze = null;
    this.calibrationResult = null;
    this.subscriptionCounter = 0;

    this.initializeEventHandlers();
  }

  private initializeEventHandlers(): void {
    for (const eventType of Object.values(GazeEventType)) {
      this.eventHandlers.set(eventType, new Set());
    }
  }

  // ============================================
  // Connection Management
  // ============================================

  async connect(): Promise<void> {
    if (this.status.connected) {
      return;
    }

    this.updateStatus({ state: 'connecting' });

    try {
      await this.adapter.connectNative();
      this.updateStatus({
        state: 'connected',
        connected: true,
      });
      this.emit(GazeEventType.DEVICE_CONNECTED, this.createSystemEvent(GazeEventType.DEVICE_CONNECTED));
    } catch (error) {
      this.updateStatus({
        state: 'error',
        error: error instanceof Error ? error.message : 'Connection failed',
      });
      throw error;
    }
  }

  async disconnect(): Promise<void> {
    if (!this.status.connected) {
      return;
    }

    this.stopTracking();
    await this.adapter.disconnectNative();

    this.updateStatus({
      state: 'disconnected',
      connected: false,
      tracking: false,
    });

    this.emit(GazeEventType.DEVICE_DISCONNECTED, this.createSystemEvent(GazeEventType.DEVICE_DISCONNECTED));
  }

  isConnected(): boolean {
    return this.status.connected;
  }

  // ============================================
  // Configuration
  // ============================================

  configure(config: TrackerConfig): void {
    this.config = { ...this.config, ...config };
  }

  getConfig(): TrackerConfig {
    return { ...this.config };
  }

  // ============================================
  // Calibration
  // ============================================

  async startCalibration(points?: CalibrationPoint[]): Promise<CalibrationResult> {
    if (!this.status.connected) {
      throw new Error('Tracker not connected');
    }

    this.updateStatus({ state: 'calibrating' });
    this.emit(GazeEventType.CALIBRATION_START, this.createSystemEvent(GazeEventType.CALIBRATION_START));

    const calibrationPoints = points ?? this.generateDefaultCalibrationPoints();

    try {
      this.calibrationResult = await this.adapter.calibrate(calibrationPoints);

      this.updateStatus({
        state: 'connected',
        calibrated: this.calibrationResult.success,
      });

      this.emit(GazeEventType.CALIBRATION_END, {
        ...this.createSystemEvent(GazeEventType.CALIBRATION_END),
        metadata: { result: this.calibrationResult },
      });

      return this.calibrationResult;
    } catch (error) {
      this.updateStatus({ state: 'connected' });
      throw error;
    }
  }

  async addCalibrationPoint(point: CalibrationPoint): Promise<void> {
    this.emit(GazeEventType.CALIBRATION_POINT, {
      ...this.createSystemEvent(GazeEventType.CALIBRATION_POINT),
      position: { x: point.x, y: point.y },
      metadata: { pointIndex: point.index },
    });
  }

  async completeCalibration(): Promise<CalibrationResult> {
    if (!this.calibrationResult) {
      throw new Error('No calibration in progress');
    }
    return this.calibrationResult;
  }

  cancelCalibration(): void {
    this.updateStatus({ state: 'connected' });
  }

  getCalibrationQuality(): CalibrationQuality | null {
    if (!this.calibrationResult?.success) {
      return null;
    }

    const accuracy = this.calibrationResult.averageAccuracy ?? 1.0;
    let overall: CalibrationQuality['overall'];

    if (accuracy < 0.5) overall = 'excellent';
    else if (accuracy < 1.0) overall = 'good';
    else if (accuracy < 2.0) overall = 'fair';
    else overall = 'poor';

    return {
      overall,
      accuracy,
      precision: this.calibrationResult.averagePrecision ?? 0.1,
      coverage: this.calibrationResult.pointResults
        ? this.calibrationResult.pointResults.filter(p => p.valid).length /
          this.calibrationResult.pointResults.length
        : 1.0,
    };
  }

  isCalibrated(): boolean {
    return this.status.calibrated;
  }

  private generateDefaultCalibrationPoints(): CalibrationPoint[] {
    // 5-point calibration pattern
    return [
      { x: 0.5, y: 0.5, index: 0 },   // Center
      { x: 0.1, y: 0.1, index: 1 },   // Top-left
      { x: 0.9, y: 0.1, index: 2 },   // Top-right
      { x: 0.1, y: 0.9, index: 3 },   // Bottom-left
      { x: 0.9, y: 0.9, index: 4 },   // Bottom-right
    ];
  }

  // ============================================
  // Data Streaming
  // ============================================

  subscribe(callback: GazeCallback): Subscription {
    const id = `sub_${++this.subscriptionCounter}`;
    this.subscriptions.set(id, callback);

    return {
      id,
      unsubscribe: () => this.unsubscribe({ id, unsubscribe: () => {} }),
    };
  }

  unsubscribe(subscription: Subscription): void {
    this.subscriptions.delete(subscription.id);
  }

  startTracking(): void {
    if (!this.status.connected) {
      throw new Error('Tracker not connected');
    }

    if (this.status.tracking) {
      return;
    }

    this.adapter.startNativeStream((gazePoint: GazePoint) => {
      this.handleGazeData(gazePoint);
    });

    this.updateStatus({
      state: 'tracking',
      tracking: true,
    });
  }

  stopTracking(): void {
    if (!this.status.tracking) {
      return;
    }

    this.adapter.stopNativeStream();
    this.updateStatus({
      state: 'connected',
      tracking: false,
    });
  }

  isTracking(): boolean {
    return this.status.tracking;
  }

  private handleGazeData(gazePoint: GazePoint): void {
    // Apply smoothing if enabled
    const processedPoint = this.config.smoothing
      ? this.applySmoothing(gazePoint)
      : gazePoint;

    this.latestGaze = processedPoint;

    // Notify all subscribers
    for (const callback of this.subscriptions.values()) {
      try {
        callback(processedPoint);
      } catch (error) {
        console.error('Error in gaze subscriber:', error);
      }
    }

    // Handle tracking lost/recovered
    if (!processedPoint.valid && this.status.tracking) {
      this.emit(GazeEventType.TRACKING_LOST, this.createSystemEvent(GazeEventType.TRACKING_LOST));
    }
  }

  private applySmoothing(current: GazePoint): GazePoint {
    if (!this.latestGaze || !current.valid) {
      return current;
    }

    const factor = this.config.smoothingFactor ?? 0.3;
    return {
      ...current,
      x: this.latestGaze.x * (1 - factor) + current.x * factor,
      y: this.latestGaze.y * (1 - factor) + current.y * factor,
    };
  }

  // ============================================
  // Event Handling
  // ============================================

  on<T extends GazeEvent>(event: GazeEventType, handler: EventCallback<T>): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.add(handler as EventHandler);
    }
  }

  off<T extends GazeEvent>(event: GazeEventType, handler: EventCallback<T>): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      handlers.delete(handler as EventHandler);
    }
  }

  once<T extends GazeEvent>(event: GazeEventType, handler: EventCallback<T>): void {
    const onceHandler: EventCallback<T> = (e: T) => {
      this.off(event, onceHandler);
      handler(e);
    };
    this.on(event, onceHandler);
  }

  private emit(eventType: GazeEventType, event: GazeEvent): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      for (const handler of handlers) {
        try {
          handler(event);
        } catch (error) {
          console.error(`Error in event handler for ${eventType}:`, error);
        }
      }
    }
  }

  private createSystemEvent(type: GazeEventType): GazeEvent {
    return {
      type,
      timestamp: Date.now(),
      eventId: `evt_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`,
    };
  }

  // ============================================
  // Device Information
  // ============================================

  getCapabilities(): EyeTrackerCapabilities {
    return this.adapter.getCapabilities();
  }

  getStatus(): TrackerStatus {
    return { ...this.status };
  }

  getLatestGaze(): GazePoint | null {
    return this.latestGaze;
  }

  // ============================================
  // Lifecycle
  // ============================================

  private updateStatus(update: Partial<TrackerStatus>): void {
    this.status = { ...this.status, ...update };
  }

  dispose(): void {
    this.stopTracking();
    if (this.status.connected) {
      this.adapter.disconnectNative().catch(() => {});
    }
    this.adapter.dispose();
    this.subscriptions.clear();
    this.eventHandlers.clear();
  }
}

/**
 * Factory function to create a WiaEyeTracker with a specific adapter
 */
export function createEyeTracker(
  adapter: IEyeTrackerAdapter,
  config?: TrackerConfig
): IEyeTracker {
  return new WiaEyeTracker(adapter, config);
}
