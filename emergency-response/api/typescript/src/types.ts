/**
 * WIA-SOC-005: Emergency Response Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * UUID v4 string
 */
export type UUID = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  x: number;
  y: number;
  theta?: number;
}

// ============================================================================
// Robot Types
// ============================================================================

/**
 * Robot operating modes
 */
export enum RobotMode {
  IDLE = 'idle',
  CLEANING = 'cleaning',
  CHARGING = 'charging',
  RETURNING = 'returning',
  PAUSED = 'paused',
  ERROR = 'error'
}

/**
 * Cleaning modes
 */
export enum CleaningMode {
  AUTO = 'auto',
  SPOT = 'spot',
  EDGE = 'edge',
  ZIGZAG = 'zigzag',
  SPIRAL = 'spiral',
  MANUAL = 'manual'
}

/**
 * Surface types
 */
export enum SurfaceType {
  HARDWOOD = 'hardwood',
  TILE = 'tile',
  CARPET_LOW = 'carpet_low',
  CARPET_HIGH = 'carpet_high',
  VINYL = 'vinyl',
  MARBLE = 'marble',
  LAMINATE = 'laminate',
  CONCRETE = 'concrete',
  RUG = 'rug',
  UNKNOWN = 'unknown'
}

/**
 * Robot identity information
 */
export interface RobotIdentity {
  robotId: UUID;
  manufacturer: string;
  model: string;
  serialNumber: string;
  firmwareVersion: string;
  hardwareVersion: string;
  manufactureDate: Timestamp;
  capabilities: string[];
}

/**
 * Battery state
 */
export interface BatteryState {
  percentage: number;        // 0-100
  voltage: number;           // Volts
  current: number;           // Amperes
  temperature: number;       // Celsius
  health: number;            // 0-100
  cycleCount: number;
  charging: boolean;
  timeToFull?: number;       // seconds
  timeToEmpty?: number;      // seconds
}

/**
 * Robot pose (position and orientation)
 */
export interface Pose {
  x: number;                 // meters
  y: number;                 // meters
  theta: number;             // radians
  floor: number;
  confidence: number;        // 0-1
}

/**
 * Sensor readings
 */
export interface SensorReadings {
  lidar?: LidarScan;
  camera?: CameraImage;
  cliff: boolean[];
  bumper: boolean;
  dirtLevel: number;         // 0-100
  imu?: ImuData;
  wheelEncoders?: WheelEncoders;
}

/**
 * Complete robot state
 */
export interface RobotState {
  timestamp: Timestamp;
  battery: BatteryState;
  pose: Pose;
  mode: RobotMode;
  cleaningMode?: CleaningMode;
  suctionPower: number;      // 0-100
  brushSpeed: number;        // 0-100
  waterFlow?: number;        // 0-100
  sensors: SensorReadings;
  errors?: RobotError[];
}

/**
 * Robot error
 */
export interface RobotError {
  code: string;
  message: string;
  severity: 'info' | 'warning' | 'error' | 'critical';
  timestamp: Timestamp;
  details?: Record<string, any>;
}

// ============================================================================
// Map Types
// ============================================================================

/**
 * Occupancy grid map
 */
export interface OccupancyGrid {
  resolution: number;        // meters per cell
  width: number;             // cells
  height: number;            // cells
  origin: Coordinates;
  data: Uint8Array;          // 0=free, 100=occupied, -1=unknown
  rooms: Room[];
  noGoZones: Zone[];
  dockLocation: Coordinates;
  lastUpdated: Timestamp;
}

/**
 * Room definition
 */
export interface Room {
  id: UUID;
  name: string;
  polygon: Coordinates[];
  surfaceType: SurfaceType;
  lastCleaned?: Timestamp;
  cleanCount: number;
  cleaningPriority: number;  // 1-10
}

/**
 * No-go zone
 */
export interface Zone {
  id: UUID;
  name?: string;
  polygon: Coordinates[];
  type: 'no_go' | 'restricted' | 'virtual_wall';
}

// ============================================================================
// Cleaning Session
// ============================================================================

/**
 * Cleaning session log
 */
export interface CleaningSession {
  sessionId: UUID;
  startTime: Timestamp;
  endTime?: Timestamp;
  duration: number;          // seconds
  areaCleaned: number;       // m²
  distanceTraveled: number;  // meters
  batteryConsumed: number;   // Wh
  cleaningMode: CleaningMode;
  rooms: UUID[];
  interruptions: Interruption[];
  statistics: SessionStatistics;
}

/**
 * Cleaning interruption
 */
export interface Interruption {
  timestamp: Timestamp;
  reason: 'stuck' | 'battery_low' | 'manual_stop' | 'error' | 'obstacle';
  duration: number;          // seconds
  location?: Coordinates;
}

/**
 * Session statistics
 */
export interface SessionStatistics {
  avgSpeed: number;          // m/s
  avgSuction: number;        // %
  dirtCollected: number;     // grams
  coverage: number;          // %
  efficiency: number;        // %
}

// ============================================================================
// Sensor Data
// ============================================================================

/**
 * LiDAR scan data
 */
export interface LidarScan {
  timestamp: Timestamp;
  angleMin: number;          // radians
  angleMax: number;          // radians
  angleIncrement: number;    // radians
  rangeMin: number;          // meters
  rangeMax: number;          // meters
  ranges: number[];          // meters
  intensities?: number[];    // optional
}

/**
 * Camera image data
 */
export interface CameraImage {
  timestamp: Timestamp;
  width: number;
  height: number;
  encoding: 'rgb8' | 'bgr8' | 'jpeg' | 'png';
  data: string;              // base64
  detectedObjects?: DetectedObject[];
}

/**
 * Detected object
 */
export interface DetectedObject {
  class: string;
  confidence: number;        // 0-1
  boundingBox: {
    x: number;
    y: number;
    w: number;
    h: number;
  };
}

/**
 * IMU data
 */
export interface ImuData {
  timestamp: Timestamp;
  accelerometer: {
    x: number;
    y: number;
    z: number;
  };
  gyroscope: {
    x: number;
    y: number;
    z: number;
  };
  magnetometer?: {
    x: number;
    y: number;
    z: number;
  };
}

/**
 * Wheel encoder data
 */
export interface WheelEncoders {
  timestamp: Timestamp;
  left: number;              // ticks
  right: number;             // ticks
  leftVelocity: number;      // rad/s
  rightVelocity: number;     // rad/s
}

// ============================================================================
// Commands
// ============================================================================

/**
 * Start cleaning command parameters
 */
export interface StartCleaningParams {
  mode: CleaningMode;
  rooms?: UUID[];
  powerLevel?: number;       // 0-100
  waterLevel?: number;       // 0-100
  repeat?: number;           // number of passes
}

/**
 * Robot command
 */
export interface RobotCommand {
  commandId: UUID;
  timestamp: Timestamp;
  action: 'start' | 'stop' | 'pause' | 'resume' | 'dock' | 'set_mode';
  parameters?: Record<string, any>;
}

/**
 * Command response
 */
export interface CommandResponse {
  commandId: UUID;
  status: 'accepted' | 'rejected' | 'completed' | 'failed';
  message?: string;
  data?: Record<string, any>;
}

// ============================================================================
// Events
// ============================================================================

/**
 * Robot event
 */
export interface RobotEvent {
  eventId: UUID;
  timestamp: Timestamp;
  type: 'info' | 'warning' | 'error' | 'critical';
  category: 'navigation' | 'cleaning' | 'battery' | 'sensor' | 'communication';
  message: string;
  details?: Record<string, any>;
}

/**
 * Event listener callback
 */
export type EventCallback = (event: RobotEvent) => void;

// ============================================================================
// Scheduling
// ============================================================================

/**
 * Cleaning schedule
 */
export interface CleaningSchedule {
  scheduleId: UUID;
  enabled: boolean;
  entries: ScheduleEntry[];
}

/**
 * Schedule entry
 */
export interface ScheduleEntry {
  day: 'monday' | 'tuesday' | 'wednesday' | 'thursday' | 'friday' | 'saturday' | 'sunday';
  time: string;              // HH:MM format
  mode: CleaningMode;
  rooms?: UUID[];
  powerLevel?: number;
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Robot configuration
 */
export interface RobotConfig {
  robotId: UUID;
  name: string;
  location?: string;
  timezone: string;
  language: string;
  units: 'metric' | 'imperial';
  preferences: RobotPreferences;
  network: NetworkConfig;
}

/**
 * Robot preferences
 */
export interface RobotPreferences {
  defaultCleaningMode: CleaningMode;
  defaultPowerLevel: number;
  quietHours?: {
    enabled: boolean;
    start: string;           // HH:MM
    end: string;             // HH:MM
  };
  voiceControl: boolean;
  ledBrightness: number;     // 0-100
  soundVolume: number;       // 0-100
}

/**
 * Network configuration
 */
export interface NetworkConfig {
  wifiSsid?: string;
  wifiConnected: boolean;
  ipAddress?: string;
  macAddress: string;
  cloudEnabled: boolean;
  cloudConnected: boolean;
}

// ============================================================================
// API Client Options
// ============================================================================

/**
 * API client configuration options
 */
export interface ApiClientOptions {
  host: string;
  port?: number;
  protocol?: 'http' | 'https';
  token?: string;
  timeout?: number;          // milliseconds
  retries?: number;
}

/**
 * WebSocket options
 */
export interface WebSocketOptions {
  reconnect?: boolean;
  reconnectInterval?: number; // milliseconds
  maxReconnectAttempts?: number;
}
