/**
 * WIA-CRYO-009 TypeScript Type Definitions
 *
 * @packageDocumentation
 */

/**
 * Standard compliance levels
 */
export enum ComplianceLevel {
  LEVEL_1_BASIC = 'level1',
  LEVEL_2_STANDARD = 'level2',
  LEVEL_3_PREMIUM = 'level3'
}

/**
 * Transport modes
 */
export enum TransportMode {
  GROUND = 'ground',
  AIR = 'air',
  SEA = 'sea',
  MULTIMODAL = 'multimodal'
}

/**
 * Container types
 */
export enum ContainerType {
  DEWAR_50L = 'dewar-50L',
  DEWAR_100L = 'dewar-100L',
  DEWAR_250L = 'dewar-250L',
  CUSTOM = 'custom'
}

/**
 * Temperature zones
 */
export enum TemperatureZone {
  OPTIMAL = 'optimal',     // -196°C to -185°C
  ACCEPTABLE = 'acceptable', // -185°C to -140°C
  WARNING = 'warning',      // -140°C to -130°C
  CRITICAL = 'critical'     // Above -130°C
}

/**
 * Emergency levels
 */
export enum EmergencyLevel {
  NONE = 0,
  MINOR = 1,
  MODERATE = 2,
  SEVERE = 3,
  CRITICAL = 4
}

/**
 * GPS position data
 */
export interface GPSPosition {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy: number;
  timestamp: Date;
  satellites?: number;
}

/**
 * Temperature reading
 */
export interface TemperatureReading {
  celsius: number;
  zone: TemperatureZone;
  sensorId: string;
  timestamp: Date;
}

/**
 * Accelerometer reading (6-axis)
 */
export interface AccelerometerReading {
  x: number;          // m/s² (translational)
  y: number;
  z: number;
  roll: number;       // rad/s (rotational)
  pitch: number;
  yaw: number;
  magnitude: number;  // Combined g-force
  timestamp: Date;
}

/**
 * LN2 level measurement
 */
export interface LN2Level {
  liters: number;
  percentage: number;  // 0-100
  capacity: number;    // Total capacity in liters
  timestamp: Date;
}

/**
 * Transport container
 */
export interface Container {
  id: string;
  serialNumber: string;
  type: ContainerType;
  capacity: number;         // Liters
  holdTime: number;         // Days (static)
  lastInspection: Date;
  certifications: string[];
  vacuumPressure?: number;  // Torr
}

/**
 * Patient information
 */
export interface Patient {
  id: string;
  preservationType: 'whole-body' | 'neuro';
  preservationDate: Date;
  cryoprotectant?: string;
}

/**
 * Facility information
 */
export interface Facility {
  id: string;
  name: string;
  address: string;
  location: GPSPosition;
  contactPhone: string;
  contactEmail: string;
  operatingLicense: string;
}

/**
 * Transport personnel
 */
export interface Personnel {
  id: string;
  name: string;
  role: 'coordinator' | 'driver' | 'technician' | 'other';
  certifications: string[];
  biometricId?: string;
}

/**
 * Chain of custody transfer
 */
export interface CustodyTransfer {
  id: string;
  timestamp: Date;
  location: GPSPosition;
  fromPersonnel: Personnel;
  toPersonnel: Personnel;
  containerSealNumbers: string[];
  temperature: TemperatureReading;
  ln2Level: LN2Level;
  photos?: string[];           // URLs or base64
  digitalSignatures: {
    from: string;
    to: string;
  };
  blockchainTxHash?: string;   // If using blockchain
}

/**
 * Alert configuration
 */
export interface AlertConfig {
  temperatureWarning: number;   // °C
  temperatureCritical: number;  // °C
  gForceWarning: number;        // g
  gForceCritical: number;       // g
  ln2LevelWarning: number;      // percentage
  ln2LevelCritical: number;     // percentage
  geofenceDeviation: number;    // meters
  notificationEmails: string[];
  notificationPhones: string[];
}

/**
 * Transport event
 */
export interface TransportEvent {
  id: string;
  type: 'departure' | 'arrival' | 'transfer' | 'refill' | 'inspection' | 'alert' | 'incident' | 'other';
  timestamp: Date;
  location: GPSPosition;
  description: string;
  personnel?: Personnel;
  severity?: EmergencyLevel;
  resolved?: boolean;
  metadata?: Record<string, any>;
}

/**
 * Geofence definition
 */
export interface Geofence {
  id: string;
  name: string;
  type: 'corridor' | 'facility' | 'exclusion' | 'speed-zone';
  coordinates: GPSPosition[];  // Polygon vertices
  radius?: number;             // meters (for circular fences)
  maxSpeed?: number;           // km/h (for speed zones)
  active: boolean;
}

/**
 * Transport plan
 */
export interface TransportPlan {
  id: string;
  patient: Patient;
  container: Container;
  origin: Facility;
  destination: Facility;
  mode: TransportMode;
  complianceLevel: ComplianceLevel;
  plannedRoute: GPSPosition[];
  estimatedDuration: number;    // hours
  plannedDeparture: Date;
  plannedArrival: Date;
  personnel: Personnel[];
  geofences: Geofence[];
  alertConfig: AlertConfig;
  documentation: {
    permits: string[];
    insurance: string;
    certifications: string[];
  };
}

/**
 * Real-time transport status
 */
export interface TransportStatus {
  transportId: string;
  status: 'preparing' | 'in-transit' | 'stopped' | 'delivered' | 'emergency';
  currentLocation: GPSPosition;
  latestTemperature: TemperatureReading;
  latestAcceleration: AccelerometerReading;
  latestLN2Level: LN2Level;
  currentPersonnel: Personnel;
  distanceTraveled: number;     // km
  distanceRemaining: number;    // km
  estimatedArrival: Date;
  alerts: TransportEvent[];
  lastUpdate: Date;
}

/**
 * Sensor data package
 */
export interface SensorDataPackage {
  transportId: string;
  timestamp: Date;
  location: GPSPosition;
  temperature: TemperatureReading[];   // Multiple sensors
  acceleration: AccelerometerReading;
  ln2Level: LN2Level;
  batteryLevel: number;                // percentage
  signalStrength: number;              // dBm
}

/**
 * Transport summary (post-completion)
 */
export interface TransportSummary {
  transportId: string;
  plan: TransportPlan;
  actualDeparture: Date;
  actualArrival: Date;
  actualDuration: number;           // hours
  actualDistance: number;           // km
  custodyTransfers: CustodyTransfer[];
  events: TransportEvent[];
  temperatureStats: {
    min: number;
    max: number;
    avg: number;
    timeInOptimal: number;          // hours
    timeInWarning: number;
    timeInCritical: number;
  };
  accelerationStats: {
    maxGForce: number;
    events: number;                 // Count of >2g events
    criticalEvents: number;         // Count of >5g events
  };
  ln2Stats: {
    initialLevel: number;
    finalLevel: number;
    consumed: number;
    refills: number;
  };
  complianceScore: number;          // 0-100
  incidents: TransportEvent[];
  recommendations: string[];
}

/**
 * Blockchain configuration
 */
export interface BlockchainConfig {
  enabled: boolean;
  network: 'ethereum' | 'polygon' | 'arbitrum' | 'private';
  contractAddress: string;
  privateKey?: string;
  rpcUrl: string;
}

/**
 * API configuration
 */
export interface CryoTransportConfig {
  apiEndpoint: string;
  apiKey: string;
  blockchain?: BlockchainConfig;
  autoRetry?: boolean;
  retryAttempts?: number;
  timeout?: number;             // ms
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Date;
}

/**
 * Statistics for temperature predictions (AI)
 */
export interface TemperaturePrediction {
  currentTemperature: number;
  predictedTemperature: number;  // 1 hour ahead
  confidence: number;            // 0-1
  riskLevel: 'low' | 'medium' | 'high';
  recommendations: string[];
  timestamp: Date;
}

/**
 * Route optimization result
 */
export interface RouteOptimization {
  route: GPSPosition[];
  distance: number;              // km
  estimatedDuration: number;     // hours
  riskScore: number;             // 0-100 (lower is better)
  refillStops: {
    location: GPSPosition;
    facilityName: string;
    estimatedTime: Date;
  }[];
  weatherWarnings: string[];
  trafficWarnings: string[];
}

/**
 * Export all types as a namespace for convenience
 */
export namespace WIACryo {
  export type Position = GPSPosition;
  export type Temp = TemperatureReading;
  export type Accel = AccelerometerReading;
  export type Level = LN2Level;
  export type Transfer = CustodyTransfer;
  export type Event = TransportEvent;
  export type Plan = TransportPlan;
  export type Status = TransportStatus;
  export type Summary = TransportSummary;
  export type Response<T> = APIResponse<T>;
}
