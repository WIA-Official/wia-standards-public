/**
 * WIA-AUTO-008: Connected Car - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Vehicle Types
// ============================================================================

/**
 * Vehicle Identification Number (VIN) - 17 characters
 */
export type VIN = string;

/**
 * Geographic coordinates
 */
export interface Location {
  /** Latitude in decimal degrees (-90 to 90) */
  latitude: number;

  /** Longitude in decimal degrees (-180 to 180) */
  longitude: number;

  /** Altitude in meters above sea level */
  altitude?: number;

  /** Heading/direction in degrees (0-360, 0=North) */
  heading?: number;

  /** Speed in km/h */
  speed?: number;

  /** Horizontal accuracy in meters */
  accuracy?: number;

  /** Timestamp of location reading */
  timestamp?: Date;
}

/**
 * Vehicle status information
 */
export interface VehicleStatus {
  /** Ignition state */
  ignition: boolean;

  /** Vehicle locked state */
  locked: boolean;

  /** Odometer reading in kilometers */
  odometer: number;

  /** Fuel level percentage (0-100) */
  fuelLevel?: number;

  /** Battery state of charge percentage (0-100) for EVs */
  batteryStateOfCharge?: number;

  /** Battery voltage in volts */
  batteryVoltage: number;

  /** Engine RPM */
  engineRpm?: number;

  /** Engine temperature in Celsius */
  engineTemperature?: number;

  /** Transmission gear */
  gear?: 'P' | 'R' | 'N' | 'D' | '1' | '2' | '3' | '4' | '5' | '6';

  /** Tire pressure readings */
  tirePressure?: TirePressure;

  /** Active warning lights */
  warningLights?: string[];
}

/**
 * Tire pressure and temperature information
 */
export interface TirePressure {
  frontLeft: TireInfo;
  frontRight: TireInfo;
  rearLeft: TireInfo;
  rearRight: TireInfo;
  spare?: TireInfo;
}

/**
 * Individual tire information
 */
export interface TireInfo {
  /** Pressure in PSI or kPa */
  pressure: number;

  /** Temperature in Celsius */
  temperature?: number;

  /** Pressure unit */
  unit: 'psi' | 'kpa' | 'bar';

  /** Tire status */
  status: 'ok' | 'low' | 'high' | 'critical';
}

// ============================================================================
// Telemetry Types
// ============================================================================

/**
 * Complete telemetry data packet
 */
export interface VehicleTelemetry {
  /** Vehicle Identification Number */
  vehicleId: VIN;

  /** Timestamp of data collection (ISO 8601) */
  timestamp: Date | string;

  /** GPS location data */
  location: Location;

  /** Vehicle status */
  status: VehicleStatus;

  /** Diagnostic information */
  diagnostics?: DiagnosticsData;

  /** Environmental data */
  environment?: EnvironmentalData;

  /** Driver behavior metrics */
  driverBehavior?: DriverBehavior;

  /** Metadata about the data packet */
  metadata?: TelemetryMetadata;
}

/**
 * Telemetry collection configuration
 */
export interface TelemetryConfig {
  /** Collection interval in milliseconds */
  interval: number;

  /** Include location data */
  includeLocation?: boolean;

  /** Include vehicle status */
  includeStatus?: boolean;

  /** Include diagnostics */
  includeDiagnostics?: boolean;

  /** Include environmental data */
  includeEnvironment?: boolean;

  /** Include driver behavior */
  includeDriverBehavior?: boolean;

  /** Callback for telemetry data */
  onData?: (data: VehicleTelemetry) => void | Promise<void>;

  /** Callback for errors */
  onError?: (error: Error) => void;

  /** Batch size before transmission */
  batchSize?: number;

  /** Compression enabled */
  compress?: boolean;
}

/**
 * Metadata about telemetry packet
 */
export interface TelemetryMetadata {
  /** Firmware version */
  firmwareVersion: string;

  /** TCU serial number */
  tcuSerial: string;

  /** Data schema version */
  dataVersion: string;

  /** Signal strength (RSSI) */
  signalStrength?: number;

  /** Connection type */
  connectionType?: 'cellular' | 'wifi' | 'bluetooth' | 'satellite';

  /** Battery level of TCU */
  tcuBatteryLevel?: number;
}

/**
 * Environmental sensor data
 */
export interface EnvironmentalData {
  /** Outside temperature in Celsius */
  outsideTemperature?: number;

  /** Inside temperature in Celsius */
  insideTemperature?: number;

  /** Humidity percentage */
  humidity?: number;

  /** Atmospheric pressure in hPa */
  pressure?: number;

  /** Rain detection */
  rainDetected?: boolean;

  /** Light level (lux) */
  ambientLight?: number;

  /** Road surface condition */
  roadCondition?: 'dry' | 'wet' | 'snow' | 'ice' | 'unknown';
}

/**
 * Driver behavior metrics
 */
export interface DriverBehavior {
  /** Average speed in km/h */
  averageSpeed: number;

  /** Maximum speed in km/h */
  maxSpeed: number;

  /** Number of harsh acceleration events */
  harshAccelerations: number;

  /** Number of harsh braking events */
  harshBraking: number;

  /** Number of harsh cornering events */
  harshCornering: number;

  /** Idle time in seconds */
  idleTime: number;

  /** Driving time in seconds */
  drivingTime: number;

  /** Distance driven in kilometers */
  distanceDriven: number;

  /** Safety score (0-100) */
  safetyScore?: number;

  /** Eco-driving score (0-100) */
  ecoScore?: number;
}

// ============================================================================
// Diagnostics Types
// ============================================================================

/**
 * Diagnostic data
 */
export interface DiagnosticsData {
  /** Diagnostic Trouble Codes */
  dtcs: DiagnosticTroubleCode[];

  /** System health scores */
  systemHealth?: SystemHealth;

  /** Sensor readings */
  sensors?: SensorReadings;

  /** Fluid levels */
  fluids?: FluidLevels;

  /** Overall health score (0-100) */
  healthScore?: number;
}

/**
 * Diagnostic Trouble Code (DTC)
 */
export interface DiagnosticTroubleCode {
  /** DTC code (e.g., P0420) */
  code: string;

  /** Human-readable description */
  description: string;

  /** Severity level */
  severity: 'critical' | 'high' | 'medium' | 'low';

  /** System affected */
  system: 'powertrain' | 'chassis' | 'body' | 'network';

  /** When the code was first detected */
  firstDetected: Date | string;

  /** Number of occurrences */
  occurrences: number;

  /** Is code currently active */
  active: boolean;

  /** Recommended action */
  recommendation?: string;
}

/**
 * System health scores
 */
export interface SystemHealth {
  /** Engine health (0-100) */
  engine: number;

  /** Transmission health (0-100) */
  transmission: number;

  /** Brake system health (0-100) */
  brakes: number;

  /** Suspension health (0-100) */
  suspension: number;

  /** Electrical system health (0-100) */
  electrical: number;

  /** Battery health (0-100) */
  battery: number;

  /** Cooling system health (0-100) */
  cooling: number;
}

/**
 * Sensor readings
 */
export interface SensorReadings {
  /** Mass Air Flow in g/s */
  maf?: number;

  /** Oxygen sensor (lambda) */
  o2?: number;

  /** Manifold Absolute Pressure in kPa */
  map?: number;

  /** Intake Air Temperature in Celsius */
  iat?: number;

  /** Engine Coolant Temperature in Celsius */
  ect?: number;

  /** Throttle Position percentage (0-100) */
  throttlePosition?: number;

  /** Fuel pressure in kPa */
  fuelPressure?: number;

  /** Boost pressure in kPa (turbo/supercharged) */
  boostPressure?: number;
}

/**
 * Fluid levels
 */
export interface FluidLevels {
  engineOil: FluidStatus;
  coolant: FluidStatus;
  brakeFluid: FluidStatus;
  washerFluid: FluidStatus;
  transmissionFluid?: FluidStatus;
  powerSteeringFluid?: FluidStatus;
}

/**
 * Fluid status
 */
export interface FluidStatus {
  level: 'ok' | 'low' | 'critical' | 'empty';
  percentage?: number;
  lastChanged?: Date | string;
  changeRecommendedAt?: number; // kilometers
}

/**
 * Diagnostic request configuration
 */
export interface DiagnosticRequest {
  /** Diagnostic level */
  level: 'basic' | 'standard' | 'comprehensive';

  /** Systems to diagnose */
  systems?: ('engine' | 'transmission' | 'brakes' | 'battery' | 'all')[];

  /** Include predictive maintenance */
  includePredictive?: boolean;

  /** Clear DTCs after reading */
  clearDTCs?: boolean;
}

/**
 * Diagnostic result
 */
export interface DiagnosticResult {
  /** Job ID for tracking */
  jobId: string;

  /** Job status */
  status: 'pending' | 'running' | 'completed' | 'failed';

  /** Diagnostics data */
  results?: DiagnosticsData;

  /** Predictive maintenance results */
  predictiveMaintenance?: PredictiveMaintenance[];

  /** Recommendations */
  recommendations?: string[];

  /** Errors encountered */
  errors?: string[];

  /** Timestamp of completion */
  completedAt?: Date | string;
}

/**
 * Predictive maintenance item
 */
export interface PredictiveMaintenance {
  /** Component name */
  component: string;

  /** Current condition percentage (100 = new) */
  currentCondition: number;

  /** Estimated remaining life in days */
  estimatedLife: number;

  /** Estimated remaining life in kilometers */
  estimatedLifeKm?: number;

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Recommendation */
  recommendation: string;

  /** Confidence level (0-1) */
  confidence: number;
}

// ============================================================================
// OTA Update Types
// ============================================================================

/**
 * OTA update package information
 */
export interface OTAUpdatePackage {
  /** Unique package ID */
  packageId: string;

  /** Version number */
  version: string;

  /** Release date */
  releaseDate: Date | string;

  /** Target component */
  targetComponent: string;

  /** Package type */
  packageType: 'software' | 'firmware' | 'configuration' | 'map';

  /** Package size in bytes */
  size: number;

  /** Digital signature */
  signature: PackageSignature;

  /** Encryption details */
  encryption: PackageEncryption;

  /** Integrity checksum */
  integrity: PackageIntegrity;

  /** Compatibility information */
  compatibility: PackageCompatibility;

  /** Download URL */
  downloadUrl: string;

  /** Release notes */
  releaseNotes: string;

  /** Update criticality */
  criticality: 'critical' | 'recommended' | 'optional';

  /** Estimated installation time in minutes */
  estimatedInstallTime?: number;
}

/**
 * Package digital signature
 */
export interface PackageSignature {
  /** Signature algorithm (e.g., RSA-4096) */
  algorithm: string;

  /** Base64-encoded signature value */
  value: string;

  /** Base64-encoded certificate */
  certificate: string;

  /** Certificate chain */
  certificateChain?: string[];
}

/**
 * Package encryption details
 */
export interface PackageEncryption {
  /** Encryption algorithm (e.g., AES-256-GCM) */
  algorithm: string;

  /** Key fingerprint (SHA-256 hash) */
  keyFingerprint: string;

  /** Initialization vector (if applicable) */
  iv?: string;
}

/**
 * Package integrity information
 */
export interface PackageIntegrity {
  /** Hash algorithm (e.g., SHA-256) */
  algorithm: string;

  /** Hex-encoded checksum */
  checksum: string;
}

/**
 * Package compatibility requirements
 */
export interface PackageCompatibility {
  /** Compatible vehicle models */
  vehicleModels: string[];

  /** Minimum hardware version */
  minHardwareVersion: string;

  /** Maximum hardware version */
  maxHardwareVersion?: string;

  /** Required dependencies */
  dependencies: string[];

  /** Conflicting packages */
  conflicts?: string[];
}

/**
 * OTA installation request
 */
export interface OTAInstallRequest {
  /** Package ID to install */
  packageId: string;

  /** Schedule installation at specific time */
  scheduleAt?: Date | string;

  /** Enable automatic rollback on failure */
  autoRollback?: boolean;

  /** Notify user on completion */
  notifyUser?: boolean;

  /** Installation priority */
  priority?: 'high' | 'normal' | 'low';
}

/**
 * OTA installation job
 */
export interface OTAInstallJob {
  /** Job ID for tracking */
  jobId: string;

  /** Package being installed */
  packageId: string;

  /** Job status */
  status: 'scheduled' | 'downloading' | 'verifying' | 'installing' | 'completed' | 'failed' | 'rolled-back';

  /** Progress percentage (0-100) */
  progress: number;

  /** Current phase */
  phase?: 'download' | 'verify' | 'install' | 'validate';

  /** Scheduled installation time */
  scheduledAt?: Date | string;

  /** Started at */
  startedAt?: Date | string;

  /** Completed at */
  completedAt?: Date | string;

  /** Error message if failed */
  error?: string;

  /** Rollback performed */
  rolledBack?: boolean;

  /** Previous version (for rollback) */
  previousVersion?: string;
}

/**
 * OTA update check response
 */
export interface OTAUpdateCheck {
  /** Updates available */
  updates: OTAUpdatePackage[];

  /** Last check timestamp */
  lastChecked: Date | string;

  /** Next scheduled check */
  nextCheck?: Date | string;
}

// ============================================================================
// Connectivity Types
// ============================================================================

/**
 * Connectivity status
 */
export interface ConnectivityStatus {
  /** Overall connectivity state */
  connected: boolean;

  /** Active connection type */
  connectionType: 'cellular' | 'wifi' | 'bluetooth' | 'satellite' | 'none';

  /** Cellular connection details */
  cellular?: CellularConnection;

  /** WiFi connection details */
  wifi?: WiFiConnection;

  /** Bluetooth connection details */
  bluetooth?: BluetoothConnection;

  /** Signal strength (RSSI in dBm) */
  signalStrength: number;

  /** Data usage statistics */
  dataUsage?: DataUsage;

  /** Last connected timestamp */
  lastConnected?: Date | string;
}

/**
 * Cellular connection information
 */
export interface CellularConnection {
  /** Network type */
  networkType: '2G' | '3G' | '4G' | '5G';

  /** Carrier/operator name */
  carrier: string;

  /** Signal strength (RSSI) */
  signalStrength: number;

  /** Signal quality (RSRQ for LTE/5G) */
  signalQuality?: number;

  /** Cell ID */
  cellId?: string;

  /** MCC (Mobile Country Code) */
  mcc?: string;

  /** MNC (Mobile Network Code) */
  mnc?: string;

  /** IP address */
  ipAddress?: string;

  /** Roaming status */
  roaming: boolean;
}

/**
 * WiFi connection information
 */
export interface WiFiConnection {
  /** SSID (network name) */
  ssid: string;

  /** BSSID (MAC address of access point) */
  bssid?: string;

  /** Signal strength (RSSI) */
  signalStrength: number;

  /** WiFi channel */
  channel?: number;

  /** WiFi band */
  band: '2.4GHz' | '5GHz' | '6GHz';

  /** Security type */
  security: 'open' | 'wep' | 'wpa' | 'wpa2' | 'wpa3';

  /** IP address */
  ipAddress?: string;
}

/**
 * Bluetooth connection information
 */
export interface BluetoothConnection {
  /** Paired devices */
  pairedDevices: BluetoothDevice[];

  /** Connected devices */
  connectedDevices: BluetoothDevice[];

  /** Bluetooth version */
  version: string;
}

/**
 * Bluetooth device
 */
export interface BluetoothDevice {
  /** Device name */
  name: string;

  /** Device MAC address */
  address: string;

  /** Device type */
  type: 'phone' | 'headset' | 'key' | 'other';

  /** Connected status */
  connected: boolean;

  /** Signal strength */
  rssi?: number;
}

/**
 * Data usage statistics
 */
export interface DataUsage {
  /** Total data transmitted (bytes) */
  transmitted: number;

  /** Total data received (bytes) */
  received: number;

  /** Current billing period usage */
  periodUsage?: number;

  /** Data limit for period */
  dataLimit?: number;

  /** Period start date */
  periodStart?: Date | string;

  /** Period end date */
  periodEnd?: Date | string;
}

// ============================================================================
// Cloud Integration Types
// ============================================================================

/**
 * Cloud platform configuration
 */
export interface CloudConfig {
  /** Cloud platform type */
  platform: 'aws' | 'azure' | 'gcp' | 'custom';

  /** API endpoint */
  endpoint: string;

  /** Authentication credentials */
  credentials: CloudCredentials;

  /** Connection protocol */
  protocol: 'mqtt' | 'https' | 'websocket' | 'amqp';

  /** Region */
  region?: string;

  /** Custom headers */
  headers?: Record<string, string>;

  /** Connection timeout (ms) */
  timeout?: number;

  /** Retry configuration */
  retry?: RetryConfig;
}

/**
 * Cloud authentication credentials
 */
export interface CloudCredentials {
  /** API key */
  apiKey?: string;

  /** Access token */
  accessToken?: string;

  /** Client certificate */
  certificate?: string;

  /** Private key */
  privateKey?: string;

  /** CA certificate */
  caCert?: string;

  /** OAuth configuration */
  oauth?: {
    clientId: string;
    clientSecret: string;
    tokenUrl: string;
  };
}

/**
 * Retry configuration
 */
export interface RetryConfig {
  /** Maximum retry attempts */
  maxAttempts: number;

  /** Initial retry delay (ms) */
  initialDelay: number;

  /** Maximum retry delay (ms) */
  maxDelay: number;

  /** Backoff multiplier */
  backoffMultiplier: number;

  /** Retry on these error types */
  retryableErrors?: string[];
}

// ============================================================================
// Security and Privacy Types
// ============================================================================

/**
 * User consent preferences
 */
export interface ConsentPreferences {
  /** Vehicle ID */
  vehicleId: VIN;

  /** User ID */
  userId: string;

  /** Consent grants */
  consents: ConsentGrant[];

  /** Last updated timestamp */
  lastUpdated: Date | string;

  /** IP address of last update */
  updateIpAddress?: string;
}

/**
 * Individual consent grant
 */
export interface ConsentGrant {
  /** Consent category */
  category: 'required' | 'optional' | 'analytics' | 'marketing' | 'third_party';

  /** Description of what is consented to */
  description: string;

  /** Consent granted */
  granted: boolean;

  /** Timestamp of grant/revocation */
  timestamp: Date | string;

  /** Expiration date (if applicable) */
  expiresAt?: Date | string;

  /** Version of terms */
  termsVersion?: string;
}

/**
 * Data anonymization configuration
 */
export interface AnonymizationConfig {
  /** Anonymization technique */
  technique: 'k-anonymity' | 'differential-privacy' | 'pseudonymization';

  /** K-anonymity parameter (if applicable) */
  kValue?: number;

  /** Differential privacy epsilon (if applicable) */
  epsilon?: number;

  /** Fields to anonymize */
  fieldsToAnonymize: string[];

  /** Retain original for duration (days) */
  retentionDays?: number;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * Connected Car SDK configuration
 */
export interface ConnectedCarConfig {
  /** Vehicle Identification Number */
  vehicleId: VIN;

  /** API key for authentication */
  apiKey: string;

  /** Cloud endpoint URL */
  cloudEndpoint?: string;

  /** Cloud platform configuration */
  cloudConfig?: CloudConfig;

  /** Enable telemetry collection */
  enableTelemetry?: boolean;

  /** Telemetry configuration */
  telemetryConfig?: TelemetryConfig;

  /** Enable diagnostics */
  enableDiagnostics?: boolean;

  /** Enable OTA updates */
  enableOTA?: boolean;

  /** Debug mode */
  debug?: boolean;

  /** Custom logger */
  logger?: Logger;
}

/**
 * Logger interface
 */
export interface Logger {
  debug(message: string, ...args: unknown[]): void;
  info(message: string, ...args: unknown[]): void;
  warn(message: string, ...args: unknown[]): void;
  error(message: string, ...args: unknown[]): void;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-008 error codes
 */
export enum AutoErrorCode {
  // Connection errors (A001-A099)
  CONNECTION_FAILED = 'A001',
  AUTHENTICATION_FAILED = 'A002',
  NETWORK_TIMEOUT = 'A003',
  INVALID_CREDENTIALS = 'A004',

  // Telemetry errors (A100-A199)
  TELEMETRY_COLLECTION_FAILED = 'A100',
  INVALID_TELEMETRY_DATA = 'A101',
  TELEMETRY_TRANSMISSION_FAILED = 'A102',

  // Diagnostics errors (A200-A299)
  DIAGNOSTICS_FAILED = 'A200',
  DTC_READ_FAILED = 'A201',
  SENSOR_READ_FAILED = 'A202',

  // OTA errors (A300-A399)
  OTA_UPDATE_FAILED = 'A300',
  OTA_DOWNLOAD_FAILED = 'A301',
  OTA_VERIFICATION_FAILED = 'A302',
  OTA_INSTALLATION_FAILED = 'A303',
  OTA_ROLLBACK_FAILED = 'A304',

  // Security errors (A400-A499)
  SIGNATURE_VERIFICATION_FAILED = 'A400',
  ENCRYPTION_FAILED = 'A401',
  CERTIFICATE_INVALID = 'A402',
  UNAUTHORIZED_ACCESS = 'A403',

  // General errors (A900-A999)
  INVALID_VEHICLE_ID = 'A900',
  INVALID_CONFIGURATION = 'A901',
  OPERATION_NOT_SUPPORTED = 'A902',
  UNKNOWN_ERROR = 'A999',
}

/**
 * Connected Car error
 */
export class ConnectedCarError extends Error {
  constructor(
    public code: AutoErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ConnectedCarError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Event handler type
 */
export type EventHandler<T = unknown> = (event: T) => void | Promise<void>;

/**
 * Subscription type
 */
export interface Subscription {
  /** Unsubscribe from events */
  unsubscribe(): void;
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  Location,
  VehicleStatus,
  TirePressure,
  TireInfo,

  // Telemetry
  VehicleTelemetry,
  TelemetryConfig,
  TelemetryMetadata,
  EnvironmentalData,
  DriverBehavior,

  // Diagnostics
  DiagnosticsData,
  DiagnosticTroubleCode,
  SystemHealth,
  SensorReadings,
  FluidLevels,
  FluidStatus,
  DiagnosticRequest,
  DiagnosticResult,
  PredictiveMaintenance,

  // OTA Updates
  OTAUpdatePackage,
  PackageSignature,
  PackageEncryption,
  PackageIntegrity,
  PackageCompatibility,
  OTAInstallRequest,
  OTAInstallJob,
  OTAUpdateCheck,

  // Connectivity
  ConnectivityStatus,
  CellularConnection,
  WiFiConnection,
  BluetoothConnection,
  BluetoothDevice,
  DataUsage,

  // Cloud
  CloudConfig,
  CloudCredentials,
  RetryConfig,

  // Security & Privacy
  ConsentPreferences,
  ConsentGrant,
  AnonymizationConfig,

  // SDK
  ConnectedCarConfig,
  Logger,

  // Utilities
  EventHandler,
  Subscription,
};

export { AutoErrorCode, ConnectedCarError };
