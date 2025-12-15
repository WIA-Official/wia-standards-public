/**
 * WIA Eye Gaze Standard - Eye Tracker Interface
 *
 * Core interface that all eye tracker implementations must follow.
 *
 * 弘益人間 - 널리 인간을 이롭게
 */

import {
  GazePoint,
  GazeEvent,
  GazeEventType,
  EyeTrackerCapabilities,
  TrackerStatus,
  TrackerConfig,
  CalibrationPoint,
  CalibrationResult,
  CalibrationQuality,
  Subscription,
  GazeCallback,
  EventCallback,
} from '../types';

/**
 * Core Eye Tracker Interface
 *
 * This interface defines the standard API for all eye tracking devices.
 * Implementations should extend this interface to provide device-specific functionality.
 */
export interface IEyeTracker {
  // ============================================
  // Connection Management
  // ============================================

  /**
   * Connect to the eye tracker device
   * @throws Error if connection fails
   */
  connect(): Promise<void>;

  /**
   * Disconnect from the eye tracker device
   */
  disconnect(): Promise<void>;

  /**
   * Check if currently connected to the device
   */
  isConnected(): boolean;

  // ============================================
  // Configuration
  // ============================================

  /**
   * Configure the tracker settings
   * @param config - Configuration options
   */
  configure(config: TrackerConfig): void;

  /**
   * Get current configuration
   */
  getConfig(): TrackerConfig;

  // ============================================
  // Calibration
  // ============================================

  /**
   * Start calibration process
   * @param points - Calibration points to use
   * @returns Calibration result
   */
  startCalibration(points?: CalibrationPoint[]): Promise<CalibrationResult>;

  /**
   * Add a calibration point during calibration
   * @param point - Point being calibrated
   */
  addCalibrationPoint(point: CalibrationPoint): Promise<void>;

  /**
   * Complete the calibration process
   */
  completeCalibration(): Promise<CalibrationResult>;

  /**
   * Cancel ongoing calibration
   */
  cancelCalibration(): void;

  /**
   * Get the current calibration quality
   */
  getCalibrationQuality(): CalibrationQuality | null;

  /**
   * Check if the tracker is calibrated
   */
  isCalibrated(): boolean;

  // ============================================
  // Data Streaming
  // ============================================

  /**
   * Subscribe to gaze data stream
   * @param callback - Function to call with each gaze point
   * @returns Subscription object with unsubscribe method
   */
  subscribe(callback: GazeCallback): Subscription;

  /**
   * Unsubscribe from gaze data
   * @param subscription - Subscription to cancel
   */
  unsubscribe(subscription: Subscription): void;

  /**
   * Start data streaming
   */
  startTracking(): void;

  /**
   * Stop data streaming
   */
  stopTracking(): void;

  /**
   * Check if currently tracking
   */
  isTracking(): boolean;

  // ============================================
  // Event Handling
  // ============================================

  /**
   * Register an event handler
   * @param event - Event type to listen for
   * @param handler - Handler function
   */
  on<T extends GazeEvent>(event: GazeEventType, handler: EventCallback<T>): void;

  /**
   * Remove an event handler
   * @param event - Event type
   * @param handler - Handler to remove
   */
  off<T extends GazeEvent>(event: GazeEventType, handler: EventCallback<T>): void;

  /**
   * Register a one-time event handler
   * @param event - Event type
   * @param handler - Handler function
   */
  once<T extends GazeEvent>(event: GazeEventType, handler: EventCallback<T>): void;

  // ============================================
  // Device Information
  // ============================================

  /**
   * Get device capabilities
   */
  getCapabilities(): EyeTrackerCapabilities;

  /**
   * Get current device status
   */
  getStatus(): TrackerStatus;

  /**
   * Get the latest gaze point (polling mode)
   */
  getLatestGaze(): GazePoint | null;

  // ============================================
  // Lifecycle
  // ============================================

  /**
   * Dispose of resources
   */
  dispose(): void;
}

/**
 * Eye Tracker Adapter Interface
 *
 * Device-specific adapters implement this interface to translate
 * between native device APIs and the WIA standard.
 */
export interface IEyeTrackerAdapter {
  /**
   * Adapter name (e.g., 'tobii', 'gazepoint', 'pupil-labs')
   */
  readonly name: string;

  /**
   * Check if this adapter can handle the given device
   */
  canHandle(deviceInfo: unknown): boolean;

  /**
   * Initialize the adapter with device-specific settings
   */
  initialize(options?: unknown): Promise<void>;

  /**
   * Connect to the native device
   */
  connectNative(): Promise<void>;

  /**
   * Disconnect from the native device
   */
  disconnectNative(): Promise<void>;

  /**
   * Start native data streaming
   * @param callback - Callback for converted gaze data
   */
  startNativeStream(callback: GazeCallback): void;

  /**
   * Stop native data streaming
   */
  stopNativeStream(): void;

  /**
   * Get device capabilities in WIA format
   */
  getCapabilities(): EyeTrackerCapabilities;

  /**
   * Perform native calibration
   */
  calibrate(points: CalibrationPoint[]): Promise<CalibrationResult>;

  /**
   * Convert native gaze data to WIA format
   */
  convertGazeData(nativeData: unknown): GazePoint;

  /**
   * Dispose adapter resources
   */
  dispose(): void;
}
