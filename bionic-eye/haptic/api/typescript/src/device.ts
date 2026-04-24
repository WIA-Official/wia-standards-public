/**
 * WIA Haptic Standard - Device Abstraction Layer
 * @version 1.0.0
 */

import {
  BodyLocation,
  ActuatorType,
  DeviceType,
  ConnectionState,
  HapticDeviceEvent,
  HapticEventListener,
} from './types';

import {
  HapticPattern,
  HapticPrimitive,
  HapticSequence,
  SpatialPattern,
} from './patterns';

// ============================================================================
// Device Capabilities
// ============================================================================

export interface HapticCapabilities {
  /** Type of haptic actuator */
  actuatorType: ActuatorType;

  /** Supported frequency range in Hz */
  frequencyRange: {
    min: number;
    max: number;
  };

  /** Available body locations */
  locations: BodyLocation[];

  /** Maximum supported intensity (0.0-1.0) */
  maxIntensity: number;

  /** Response latency in milliseconds */
  latency: number;

  /** Number of independent actuators */
  actuatorCount: number;

  /** Whether device supports custom waveforms */
  supportsCustomWaveforms: boolean;

  /** Whether device supports amplitude modulation */
  supportsAmplitudeModulation: boolean;

  /** Whether device supports frequency modulation */
  supportsFrequencyModulation: boolean;

  /** Battery level (0-100) or null if not available */
  batteryLevel: number | null;
}

// ============================================================================
// Device Configuration
// ============================================================================

export interface DeviceConfig {
  /** Device identifier */
  deviceId?: string;

  /** Device name for display */
  name?: string;

  /** Connection timeout in ms */
  connectionTimeout?: number;

  /** Auto-reconnect on disconnect */
  autoReconnect?: boolean;

  /** Global intensity scaling (0.0-2.0) */
  intensityScale?: number;

  /** User-specific calibration */
  calibration?: UserCalibration;
}

export interface UserCalibration {
  /** Per-location intensity thresholds */
  perceptionThresholds?: Partial<Record<BodyLocation, number>>;

  /** Per-location maximum comfort levels */
  comfortMaximum?: Partial<Record<BodyLocation, number>>;

  /** Locations to avoid */
  unavailableLocations?: BodyLocation[];
}

// ============================================================================
// Haptic Device Interface
// ============================================================================

export interface IHapticDevice {
  // -------------------------------------------------------------------------
  // Properties
  // -------------------------------------------------------------------------

  /** Current connection state */
  readonly state: ConnectionState;

  /** Device capabilities (available after connection) */
  readonly capabilities: HapticCapabilities | null;

  /** Device type */
  readonly deviceType: DeviceType;

  /** Device configuration */
  readonly config: DeviceConfig;

  // -------------------------------------------------------------------------
  // Connection Management
  // -------------------------------------------------------------------------

  /**
   * Connect to the haptic device
   * @throws HapticError on connection failure
   */
  connect(): Promise<void>;

  /**
   * Disconnect from the haptic device
   */
  disconnect(): Promise<void>;

  /**
   * Check if device is connected
   */
  isConnected(): boolean;

  // -------------------------------------------------------------------------
  // Capability Query
  // -------------------------------------------------------------------------

  /**
   * Get device capabilities
   * @throws HapticError if not connected
   */
  getCapabilities(): HapticCapabilities;

  /**
   * Check if device supports a specific location
   */
  supportsLocation(location: BodyLocation): boolean;

  /**
   * Check if device supports a specific frequency
   */
  supportsFrequency(frequency: number): boolean;

  // -------------------------------------------------------------------------
  // Pattern Playback
  // -------------------------------------------------------------------------

  /**
   * Play a haptic pattern
   * @param pattern - Primitive, sequence, or spatial pattern
   */
  play(pattern: HapticPattern): Promise<void>;

  /**
   * Play multiple patterns in sequence
   * @param patterns - Array of patterns to play
   */
  playSequence(patterns: HapticPattern[]): Promise<void>;

  /**
   * Stop all haptic playback
   */
  stop(): void;

  /**
   * Pause current playback
   */
  pause(): void;

  /**
   * Resume paused playback
   */
  resume(): void;

  // -------------------------------------------------------------------------
  // Real-time Control
  // -------------------------------------------------------------------------

  /**
   * Set intensity for a specific location
   * @param location - Body location
   * @param intensity - Intensity value (0.0-1.0)
   */
  setIntensity(location: BodyLocation, intensity: number): void;

  /**
   * Trigger a quick pulse at a location
   * @param location - Body location
   * @param duration - Pulse duration in ms
   * @param intensity - Optional intensity (0.0-1.0)
   */
  pulse(location: BodyLocation, duration: number, intensity?: number): void;

  /**
   * Set continuous vibration at a location
   * @param location - Body location
   * @param frequency - Frequency in Hz
   * @param intensity - Intensity (0.0-1.0)
   */
  vibrate(location: BodyLocation, frequency: number, intensity: number): void;

  // -------------------------------------------------------------------------
  // Event Handling
  // -------------------------------------------------------------------------

  /**
   * Add event listener
   */
  addEventListener(listener: HapticEventListener): void;

  /**
   * Remove event listener
   */
  removeEventListener(listener: HapticEventListener): void;
}

// ============================================================================
// Haptic Error
// ============================================================================

export class HapticError extends Error {
  constructor(
    message: string,
    public readonly code: HapticErrorCode,
    public readonly details?: unknown
  ) {
    super(message);
    this.name = 'HapticError';
  }
}

export type HapticErrorCode =
  | 'CONNECTION_FAILED'
  | 'CONNECTION_TIMEOUT'
  | 'DEVICE_NOT_FOUND'
  | 'NOT_CONNECTED'
  | 'UNSUPPORTED_FEATURE'
  | 'INVALID_PATTERN'
  | 'PLAYBACK_ERROR'
  | 'PERMISSION_DENIED'
  | 'BLUETOOTH_DISABLED'
  | 'UNKNOWN';

// ============================================================================
// Abstract Base Device
// ============================================================================

export abstract class BaseHapticDevice implements IHapticDevice {
  protected _state: ConnectionState = 'disconnected';
  protected _capabilities: HapticCapabilities | null = null;
  protected _config: DeviceConfig;
  protected _listeners: Set<HapticEventListener> = new Set();
  protected _isPlaying: boolean = false;
  protected _isPaused: boolean = false;

  constructor(
    public readonly deviceType: DeviceType,
    config: DeviceConfig = {}
  ) {
    this._config = {
      connectionTimeout: 10000,
      autoReconnect: true,
      intensityScale: 1.0,
      ...config,
    };
  }

  // -------------------------------------------------------------------------
  // Properties
  // -------------------------------------------------------------------------

  get state(): ConnectionState {
    return this._state;
  }

  get capabilities(): HapticCapabilities | null {
    return this._capabilities;
  }

  get config(): DeviceConfig {
    return this._config;
  }

  // -------------------------------------------------------------------------
  // Connection (to be implemented by adapters)
  // -------------------------------------------------------------------------

  abstract connect(): Promise<void>;
  abstract disconnect(): Promise<void>;

  isConnected(): boolean {
    return this._state === 'connected';
  }

  // -------------------------------------------------------------------------
  // Capabilities
  // -------------------------------------------------------------------------

  getCapabilities(): HapticCapabilities {
    if (!this._capabilities) {
      throw new HapticError(
        'Device not connected',
        'NOT_CONNECTED'
      );
    }
    return this._capabilities;
  }

  supportsLocation(location: BodyLocation): boolean {
    return this._capabilities?.locations.includes(location) ?? false;
  }

  supportsFrequency(frequency: number): boolean {
    if (!this._capabilities) return false;
    const { min, max } = this._capabilities.frequencyRange;
    return frequency >= min && frequency <= max;
  }

  // -------------------------------------------------------------------------
  // Playback (to be implemented by adapters)
  // -------------------------------------------------------------------------

  abstract play(pattern: HapticPattern): Promise<void>;

  async playSequence(patterns: HapticPattern[]): Promise<void> {
    for (const pattern of patterns) {
      await this.play(pattern);
    }
  }

  abstract stop(): void;

  pause(): void {
    if (this._isPlaying) {
      this._isPaused = true;
    }
  }

  resume(): void {
    this._isPaused = false;
  }

  // -------------------------------------------------------------------------
  // Real-time Control (to be implemented by adapters)
  // -------------------------------------------------------------------------

  abstract setIntensity(location: BodyLocation, intensity: number): void;
  abstract pulse(location: BodyLocation, duration: number, intensity?: number): void;
  abstract vibrate(location: BodyLocation, frequency: number, intensity: number): void;

  // -------------------------------------------------------------------------
  // Event Handling
  // -------------------------------------------------------------------------

  addEventListener(listener: HapticEventListener): void {
    this._listeners.add(listener);
  }

  removeEventListener(listener: HapticEventListener): void {
    this._listeners.delete(listener);
  }

  protected emit(event: HapticDeviceEvent): void {
    for (const listener of this._listeners) {
      try {
        listener(event);
      } catch (e) {
        console.error('Error in haptic event listener:', e);
      }
    }
  }

  // -------------------------------------------------------------------------
  // Utility Methods
  // -------------------------------------------------------------------------

  protected assertConnected(): void {
    if (!this.isConnected()) {
      throw new HapticError('Device not connected', 'NOT_CONNECTED');
    }
  }

  protected scaleIntensity(intensity: number): number {
    const scale = this._config.intensityScale ?? 1.0;
    return Math.min(1.0, Math.max(0.0, intensity * scale));
  }

  protected applyCalibration(
    location: BodyLocation,
    intensity: number
  ): number {
    const calibration = this._config.calibration;
    if (!calibration) return intensity;

    // Check if location is unavailable
    if (calibration.unavailableLocations?.includes(location)) {
      return 0;
    }

    // Apply perception threshold
    const threshold = calibration.perceptionThresholds?.[location] ?? 0;
    if (intensity < threshold) {
      return 0;
    }

    // Apply comfort maximum
    const maximum = calibration.comfortMaximum?.[location] ?? 1.0;
    return Math.min(intensity, maximum);
  }
}

// ============================================================================
// Device Manager
// ============================================================================

export class HapticDeviceManager {
  private devices: Map<string, IHapticDevice> = new Map();
  private defaultDevice: IHapticDevice | null = null;

  /**
   * Register a device
   */
  register(id: string, device: IHapticDevice): void {
    this.devices.set(id, device);
    if (!this.defaultDevice) {
      this.defaultDevice = device;
    }
  }

  /**
   * Unregister a device
   */
  unregister(id: string): void {
    const device = this.devices.get(id);
    if (device) {
      if (device.isConnected()) {
        device.disconnect();
      }
      this.devices.delete(id);
      if (this.defaultDevice === device) {
        this.defaultDevice = this.devices.values().next().value ?? null;
      }
    }
  }

  /**
   * Get device by ID
   */
  get(id: string): IHapticDevice | undefined {
    return this.devices.get(id);
  }

  /**
   * Get default device
   */
  getDefault(): IHapticDevice | null {
    return this.defaultDevice;
  }

  /**
   * Set default device
   */
  setDefault(id: string): void {
    const device = this.devices.get(id);
    if (device) {
      this.defaultDevice = device;
    }
  }

  /**
   * Get all registered devices
   */
  getAll(): IHapticDevice[] {
    return Array.from(this.devices.values());
  }

  /**
   * Get all connected devices
   */
  getConnected(): IHapticDevice[] {
    return this.getAll().filter(d => d.isConnected());
  }

  /**
   * Connect all devices
   */
  async connectAll(): Promise<void> {
    await Promise.all(
      this.getAll().map(d => d.connect().catch(e => console.error(e)))
    );
  }

  /**
   * Disconnect all devices
   */
  async disconnectAll(): Promise<void> {
    await Promise.all(
      this.getConnected().map(d => d.disconnect())
    );
  }

  /**
   * Play pattern on all connected devices
   */
  async playOnAll(pattern: HapticPattern): Promise<void> {
    await Promise.all(
      this.getConnected().map(d => d.play(pattern))
    );
  }

  /**
   * Stop all devices
   */
  stopAll(): void {
    for (const device of this.getConnected()) {
      device.stop();
    }
  }
}

// ============================================================================
// Singleton Manager Instance
// ============================================================================

export const hapticManager = new HapticDeviceManager();
