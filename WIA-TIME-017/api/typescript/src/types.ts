/**
 * WIA-TIME-017: Chronosphere Chamber - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Chamber Types
// ============================================================================

/**
 * Chamber identification and metadata
 */
export interface ChamberIdentity {
  /** Unique chamber identifier */
  id: string;

  /** Chamber model designation */
  model: string;

  /** Serial number */
  serialNumber: string;

  /** Manufacturing date */
  manufactureDate: Date;

  /** Last certification date */
  certificationDate: Date;

  /** Next certification due */
  nextCertificationDue: Date;

  /** Total displacements performed */
  totalDisplacements: number;

  /** Current operational status */
  status: ChamberStatus;
}

/**
 * Operational status of the chamber
 */
export enum ChamberStatus {
  STANDBY = 'STANDBY',
  CALIBRATING = 'CALIBRATING',
  READY = 'READY',
  LOADING = 'LOADING',
  DISPLACING = 'DISPLACING',
  EMERGENCY = 'EMERGENCY',
  MAINTENANCE = 'MAINTENANCE',
  OFFLINE = 'OFFLINE',
}

/**
 * Complete chamber configuration
 */
export interface ChamberConfig {
  /** Chamber identity */
  identity: ChamberIdentity;

  /** Physical specifications */
  specifications: ChamberSpecifications;

  /** Operational parameters */
  parameters: OperationalParameters;

  /** Safety configuration */
  safety: SafetyConfig;
}

/**
 * Physical specifications of the chamber
 */
export interface ChamberSpecifications {
  /** External dimensions (meters) */
  external: {
    equatorialRadius: number; // 'a' in oblate spheroid
    polarRadius: number; // 'b' in oblate spheroid
  };

  /** Internal dimensions (meters) */
  internal: {
    equatorialRadius: number;
    polarRadius: number;
  };

  /** Weight specifications (kg) */
  weight: {
    empty: number;
    maxLoaded: number;
    currentLoad: number;
  };

  /** Passenger capacity */
  capacity: {
    passengers: number;
    cargoMass: number; // kg
  };

  /** Layer specifications */
  layers: LayerSpecification[];
}

/**
 * Layer specification
 */
export interface LayerSpecification {
  /** Layer number (1-6, outside to inside) */
  number: 1 | 2 | 3 | 4 | 5 | 6;

  /** Layer name */
  name: string;

  /** Material composition */
  material: string;

  /** Thickness in mm */
  thickness: number;

  /** Primary functions */
  functions: string[];

  /** Current condition */
  condition: LayerCondition;
}

/**
 * Layer condition assessment
 */
export interface LayerCondition {
  /** Integrity percentage (0-100) */
  integrity: number;

  /** Temperature (Celsius) */
  temperature: number;

  /** Stress level (MPa) */
  stress: number;

  /** Wear percentage (0-100) */
  wear: number;

  /** Requires replacement? */
  replacementNeeded: boolean;
}

// ============================================================================
// Temporal Field Types
// ============================================================================

/**
 * Temporal field configuration
 */
export interface TemporalFieldConfig {
  /** Field strength in Tesla */
  strength: number;

  /** Field geometry */
  geometry: 'spherical' | 'toroidal' | 'ellipsoidal';

  /** Operating frequency (Hz) */
  frequency: number;

  /** Harmonic content */
  harmonics: number[];

  /** Stabilization method */
  stabilization: 'passive' | 'active-feedback' | 'predictive';

  /** Containment settings */
  containment: ContainmentConfig;
}

/**
 * Field containment configuration
 */
export interface ContainmentConfig {
  /** Enable inner shielding */
  innerShield: boolean;

  /** Enable outer shielding */
  outerShield: boolean;

  /** Eddy current suppression */
  eddySuppression: boolean;

  /** Target containment efficiency (0-1) */
  efficiency: number;

  /** Maximum allowable leakage (Tesla/second) */
  maxLeakage: number;
}

/**
 * Temporal field status
 */
export interface TemporalFieldStatus {
  /** Is field currently active? */
  active: boolean;

  /** Current field strength (Tesla) */
  currentStrength: number;

  /** Field uniformity (0-1, higher is better) */
  uniformity: number;

  /** Containment efficiency (0-1) */
  containmentEfficiency: number;

  /** Current leakage rate (Tesla/second) */
  leakageRate: number;

  /** Field stability (0-1) */
  stability: number;

  /** Sensor readings */
  sensorReadings: FieldSensorReading[];
}

/**
 * Individual field sensor reading
 */
export interface FieldSensorReading {
  /** Sensor identifier */
  sensorId: string;

  /** Sensor location [x, y, z] in meters */
  location: [number, number, number];

  /** Measured field strength (Tesla) */
  strength: number;

  /** Measured field direction (unit vector) */
  direction: [number, number, number];

  /** Measurement timestamp */
  timestamp: Date;
}

// ============================================================================
// Life Support Types
// ============================================================================

/**
 * Life support system status
 */
export interface LifeSupportStatus {
  /** Oxygen generation */
  oxygen: OxygenSystemStatus;

  /** CO2 removal */
  co2Removal: CO2RemovalStatus;

  /** Atmospheric composition */
  atmosphere: AtmosphericComposition;

  /** Air circulation */
  circulation: CirculationStatus;

  /** Overall life support health */
  overallHealth: number; // 0-100 percentage
}

/**
 * Oxygen generation system status
 */
export interface OxygenSystemStatus {
  /** Is system active? */
  active: boolean;

  /** Production rate (grams/hour) */
  productionRate: number;

  /** Target production rate (grams/hour) */
  targetRate: number;

  /** Water consumption (liters/day) */
  waterConsumption: number;

  /** Remaining water supply (liters) */
  waterRemaining: number;

  /** Power consumption (watts) */
  powerConsumption: number;

  /** System efficiency (0-1) */
  efficiency: number;

  /** Backup O2 tank pressure (bar) */
  backupPressure: number;
}

/**
 * CO2 removal system status
 */
export interface CO2RemovalStatus {
  /** Primary scrubbing method */
  method: 'chemical' | 'biological' | 'hybrid';

  /** Is system active? */
  active: boolean;

  /** Removal rate (grams/hour) */
  removalRate: number;

  /** LiOH cartridge remaining (kg) */
  liohRemaining: number;

  /** Algae bioreactor health (0-1) */
  bioreactorHealth: number;

  /** System effectiveness (0-1) */
  effectiveness: number;
}

/**
 * Atmospheric composition
 */
export interface AtmosphericComposition {
  /** Oxygen percentage */
  o2: number;

  /** Nitrogen percentage */
  n2: number;

  /** Argon percentage */
  ar: number;

  /** Carbon dioxide (ppm) */
  co2: number;

  /** Water vapor relative humidity (percentage) */
  humidity: number;

  /** Total pressure (kPa) */
  pressure: number;

  /** Temperature (Celsius) */
  temperature: number;
}

/**
 * Air circulation status
 */
export interface CirculationStatus {
  /** Fan speeds (RPM) */
  fanSpeeds: number[];

  /** Airflow rate (CFM) */
  airflowRate: number;

  /** Air velocity (m/s) */
  velocity: number;

  /** Filter pressure drop (Pa) */
  filterPressureDrop: number;

  /** HEPA filter life remaining (hours) */
  hepaLifeRemaining: number;

  /** Carbon filter life remaining (hours) */
  carbonLifeRemaining: number;
}

// ============================================================================
// Environmental Control Types
// ============================================================================

/**
 * Environmental control settings
 */
export interface EnvironmentalSettings {
  /** Target temperature (Celsius) */
  targetTemperature: number;

  /** Acceptable temperature range */
  temperatureRange: [number, number];

  /** Target humidity (percentage) */
  targetHumidity: number;

  /** Acceptable humidity range */
  humidityRange: [number, number];

  /** Target pressure (kPa) */
  targetPressure: number;

  /** Lighting settings */
  lighting: LightingSettings;

  /** Climate zones */
  zones: ClimateZone[];
}

/**
 * Lighting settings
 */
export interface LightingSettings {
  /** Intensity (lux) */
  intensity: number;

  /** Color temperature (Kelvin) */
  colorTemperature: number;

  /** Dimming level (0-1) */
  dimmingLevel: number;

  /** Circadian rhythm mode */
  circadianMode: boolean;

  /** Current time of day simulation */
  simulatedTime?: 'morning' | 'day' | 'evening' | 'night';
}

/**
 * Climate zone configuration
 */
export interface ClimateZone {
  /** Zone identifier */
  zoneId: number;

  /** Zone name */
  name: string;

  /** Target temperature */
  temperature: number;

  /** Temperature tolerance */
  tolerance: number;

  /** Sensors assigned to zone */
  sensors: string[];
}

/**
 * Environmental status
 */
export interface EnvironmentalStatus {
  /** Current settings */
  settings: EnvironmentalSettings;

  /** Measured conditions */
  measured: {
    temperature: number;
    humidity: number;
    pressure: number;
    lighting: number; // lux
  };

  /** Control status */
  control: {
    heatingActive: boolean;
    coolingActive: boolean;
    humidifierActive: boolean;
    dehumidifierActive: boolean;
  };

  /** Power consumption (watts) */
  powerConsumption: number;
}

// ============================================================================
// Passenger Safety Types
// ============================================================================

/**
 * Passenger manifest entry
 */
export interface Passenger {
  /** Passenger identifier */
  id: string;

  /** Full name */
  name: string;

  /** Age */
  age: number;

  /** Weight (kg) */
  weight: number;

  /** Seat assignment */
  seatId: number;

  /** Medical clearance */
  medicalClearance: boolean;

  /** Emergency contact */
  emergencyContact: string;

  /** Special needs */
  specialNeeds?: string[];

  /** Current vital signs */
  vitals?: VitalSigns;
}

/**
 * Vital signs monitoring
 */
export interface VitalSigns {
  /** Heart rate (BPM) */
  heartRate: number;

  /** Blood pressure [systolic, diastolic] (mmHg) */
  bloodPressure: [number, number];

  /** Oxygen saturation (percentage) */
  spo2: number;

  /** Body temperature (Celsius) */
  temperature: number;

  /** Respiration rate (breaths/min) */
  respirationRate: number;

  /** Measurement timestamp */
  timestamp: Date;

  /** Alert status */
  alert: VitalSignAlert | null;
}

/**
 * Vital sign alert
 */
export interface VitalSignAlert {
  /** Alert severity */
  severity: 'info' | 'warning' | 'urgent' | 'critical';

  /** Which vital triggered alert */
  parameter: 'heartRate' | 'bloodPressure' | 'spo2' | 'temperature' | 'respiration';

  /** Alert message */
  message: string;

  /** Recommended action */
  action: string;
}

/**
 * Inertial dampening status
 */
export interface InertialDampeningStatus {
  /** Is system active? */
  active: boolean;

  /** Current attenuation factor (0-1) */
  attenuation: number;

  /** Response time (milliseconds) */
  responseTime: number;

  /** External acceleration measured (g) */
  externalAcceleration: [number, number, number]; // [x, y, z]

  /** Felt by passengers (g) */
  feltAcceleration: [number, number, number];

  /** Power consumption (watts) */
  powerConsumption: number;
}

/**
 * Medical system status
 */
export interface MedicalSystemStatus {
  /** Defibrillator status */
  defibrillator: {
    ready: boolean;
    batteryLevel: number; // 0-1
    padsAttached: boolean[];
    lastTest: Date;
  };

  /** First aid kit */
  firstAidKit: {
    inventoryComplete: boolean;
    expiringMedications: string[];
    missingItems: string[];
  };

  /** Medications available */
  medications: MedicationInventory[];
}

/**
 * Medication inventory item
 */
export interface MedicationInventory {
  /** Medication name */
  name: string;

  /** Quantity available */
  quantity: number;

  /** Unit (tablets, vials, etc.) */
  unit: string;

  /** Expiration date */
  expirationDate: Date;

  /** Is expired? */
  expired: boolean;
}

// ============================================================================
// Airlock Types
// ============================================================================

/**
 * Airlock status
 */
export interface AirlockStatus {
  /** Current state */
  state: AirlockState;

  /** Occupancy */
  occupied: boolean;

  /** Number of people in airlock */
  occupantCount: number;

  /** Current pressure (kPa) */
  pressure: number;

  /** Door status */
  doors: {
    outer: DoorStatus;
    inner: DoorStatus;
  };

  /** Cycle progress (0-1) */
  cycleProgress: number;

  /** Estimated time remaining (seconds) */
  timeRemaining: number;
}

/**
 * Airlock state machine
 */
export enum AirlockState {
  IDLE = 'IDLE',
  ENTRY_SEQUENCE = 'ENTRY_SEQUENCE',
  PRESSURIZING = 'PRESSURIZING',
  DEPRESSURIZING = 'DEPRESSURIZING',
  DECONTAMINATING = 'DECONTAMINATING',
  EXIT_SEQUENCE = 'EXIT_SEQUENCE',
  EMERGENCY = 'EMERGENCY',
}

/**
 * Door status
 */
export interface DoorStatus {
  /** Is door open? */
  open: boolean;

  /** Is door locked? */
  locked: boolean;

  /** Is door sealed? */
  sealed: boolean;

  /** Seal integrity (0-1) */
  sealIntegrity: number;
}

/**
 * Entry request
 */
export interface EntryRequest {
  /** Passengers entering */
  passengers: Passenger[];

  /** Luggage items */
  luggage: LuggageItem[];

  /** Wait for temporal synchronization? */
  temporalSync: boolean;

  /** Safety checks to perform */
  safety: {
    pressurizationRate: number; // kPa/min
    biometricVerification: boolean;
    contaminationScan: boolean;
  };
}

/**
 * Luggage item
 */
export interface LuggageItem {
  /** Item identifier */
  id: string;

  /** Description */
  description: string;

  /** Weight (kg) */
  weight: number;

  /** Has been scanned? */
  scanned: boolean;

  /** Scan results */
  scanResults?: {
    safe: boolean;
    warnings: string[];
  };
}

/**
 * Exit request
 */
export interface ExitRequest {
  /** Destination time */
  destinationTime: Date;

  /** Verify timeline before exit? */
  verifyTimeline: boolean;

  /** Perform decontamination? */
  decontamination: boolean;

  /** Perform medical check? */
  medicalCheck: boolean;
}

// ============================================================================
// Calibration Types
// ============================================================================

/**
 * Calibration request
 */
export interface CalibrationRequest {
  /** Temporal field calibration */
  temporalField: TemporalFieldCalibration;

  /** Spatial geometry calibration */
  spatialGeometry: SpatialCalibration;

  /** Sensor calibration */
  sensors: SensorCalibration;

  /** Systems check */
  systems: SystemsCheck;
}

/**
 * Temporal field calibration parameters
 */
export interface TemporalFieldCalibration {
  /** Baseline frequency (Hz) */
  frequency: number;

  /** Field amplitude (Tesla) */
  amplitude: number;

  /** Phase offset (radians) */
  phase: number;

  /** Enable harmonic tuning */
  harmonics: boolean;

  /** Automatic tuning */
  autoTune: boolean;
}

/**
 * Spatial calibration parameters
 */
export interface SpatialCalibration {
  /** Center of mass [x, y, z] relative to chamber center */
  centerOfMass: [number, number, number];

  /** Moment of inertia calculation */
  momentOfInertia: 'auto-calculate' | 'manual';

  /** Alignment reference */
  alignment: 'magnetic-north' | 'true-north' | 'custom';

  /** Tilt compensation */
  tilt: 'gravity-compensated' | 'manual';
}

/**
 * Sensor calibration settings
 */
export interface SensorCalibration {
  /** Zero point calibration */
  zeroPoint: boolean;

  /** Cross-calibration between sensors */
  crossCalibration: boolean;

  /** Temperature compensation */
  temperatureCompensation: boolean;

  /** Automatic drift correction */
  drift: 'auto-correct' | 'manual' | 'none';
}

/**
 * Systems check configuration
 */
export interface SystemsCheck {
  /** Life support check */
  lifeSupport: 'full-diagnostic' | 'quick-check' | 'skip';

  /** Power systems check */
  power: 'load-test' | 'voltage-check' | 'skip';

  /** Communications check */
  communications: 'signal-strength' | 'connectivity' | 'skip';

  /** Emergency systems check */
  emergency: 'all-systems' | 'critical-only' | 'skip';
}

/**
 * Calibration result
 */
export interface CalibrationResult {
  /** Overall calibration score (0-100) */
  score: number;

  /** Is chamber ready for displacement? */
  ready: boolean;

  /** Individual test results */
  results: {
    temporalField: TestResult;
    spatialGeometry: TestResult;
    sensors: TestResult;
    systems: TestResult;
  };

  /** Warnings */
  warnings: string[];

  /** Errors */
  errors: string[];

  /** Timestamp */
  timestamp: Date;
}

/**
 * Test result
 */
export interface TestResult {
  /** Test passed? */
  passed: boolean;

  /** Score (0-100) */
  score: number;

  /** Measured values */
  measurements: Record<string, number>;

  /** Notes */
  notes: string[];
}

// ============================================================================
// Emergency Types
// ============================================================================

/**
 * Emergency protocols
 */
export enum EmergencyType {
  ABORT = 'ABORT',
  LIFE_SUPPORT_FAILURE = 'LIFE_SUPPORT_FAILURE',
  FIELD_BREACH = 'FIELD_BREACH',
  MEDICAL_EMERGENCY = 'MEDICAL_EMERGENCY',
  FIRE = 'FIRE',
  DEPRESSURIZATION = 'DEPRESSURIZATION',
  POWER_LOSS = 'POWER_LOSS',
  COLLISION = 'COLLISION',
  TEMPORAL_ANOMALY = 'TEMPORAL_ANOMALY',
}

/**
 * Emergency severity
 */
export enum EmergencySeverity {
  ADVISORY = 1,
  CAUTION = 2,
  WARNING = 3,
  URGENT = 4,
  EMERGENCY = 5,
}

/**
 * Emergency abort request
 */
export interface AbortRequest {
  /** Reason for abort */
  reason: string;

  /** Return to origin timeline? */
  returnToOrigin: boolean;

  /** Notify authorities? */
  notifyAuthorities: boolean;

  /** Preserve data logs? */
  preserveData: boolean;
}

/**
 * Abort result
 */
export interface AbortResult {
  /** Was abort successful? */
  success: boolean;

  /** Abort report */
  report: {
    reason: string;
    triggerTime: number; // timestamp
    returnDuration: number; // milliseconds
    passengerStatus: PassengerStatus[];
    systemStatus: SystemStatus[];
    recommendations: string[];
  };

  /** Error if failed */
  error?: string;
}

/**
 * Passenger status
 */
export interface PassengerStatus {
  /** Passenger ID */
  passengerId: string;

  /** Medical condition */
  condition: 'stable' | 'injured' | 'critical';

  /** Vital signs */
  vitals: VitalSigns;

  /** Notes */
  notes: string;
}

/**
 * System status
 */
export interface SystemStatus {
  /** System name */
  system: string;

  /** Operational status */
  operational: boolean;

  /** Health percentage (0-100) */
  health: number;

  /** Issues */
  issues: string[];
}

// ============================================================================
// Monitoring Types
// ============================================================================

/**
 * Comprehensive chamber status
 */
export interface ChamberStatusReport {
  /** Structural status */
  structural: StructuralStatus;

  /** Temporal field status */
  temporal: TemporalFieldStatus;

  /** Environmental status */
  environmental: EnvironmentalStatus;

  /** Passenger status */
  passengers: PassengerSystemStatus;

  /** Power status */
  power: PowerStatus;

  /** Overall health score (0-100) */
  overallHealth: number;

  /** Active alerts */
  alerts: Alert[];

  /** Timestamp */
  timestamp: Date;
}

/**
 * Structural status
 */
export interface StructuralStatus {
  /** Integrity percentage (0-100) */
  integrity: number;

  /** Stress level (MPa) */
  stress: number;

  /** Fatigue cycles remaining */
  fatigue: number;

  /** Temperature (Celsius) */
  temperature: number;
}

/**
 * Passenger system status
 */
export interface PassengerSystemStatus {
  /** Number of passengers */
  count: number;

  /** Passenger vitals */
  vitals: VitalSigns[];

  /** Comfort score (0-100) */
  comfort: number;

  /** Active alerts */
  alerts: VitalSignAlert[];
}

/**
 * Power status
 */
export interface PowerStatus {
  /** Main power (kW) */
  main: number;

  /** Backup battery level (0-1) */
  backup: number;

  /** Current consumption (kW) */
  consumption: number;

  /** Reserve duration (hours) */
  reserve: number;
}

/**
 * Alert
 */
export interface Alert {
  /** Alert identifier */
  id: string;

  /** Severity level */
  level: number; // 1-5

  /** Alert message */
  message: string;

  /** Alert source system */
  source: string;

  /** Timestamp */
  timestamp: Date;

  /** Has been acknowledged? */
  acknowledged: boolean;

  /** Recommended action */
  recommendedAction?: string;
}

// ============================================================================
// Mission Types
// ============================================================================

/**
 * Mission configuration
 */
export interface MissionConfig {
  /** Mission identifier */
  id: string;

  /** Destination time */
  destination: Date;

  /** Mission duration (seconds) */
  duration: number;

  /** Number of passengers */
  passengers: number;

  /** Cargo mass (kg) */
  cargoMass: number;

  /** Mission type */
  type: 'tourism' | 'research' | 'rescue' | 'transport';

  /** Safety requirements */
  safety: {
    preFlightCheck: boolean;
    emergencyPrepared: boolean;
    medicalClearance: boolean;
    insuranceVerified: boolean;
  };
}

/**
 * Monitoring configuration
 */
export interface MonitoringConfig {
  /** Sampling interval (milliseconds) */
  interval: number;

  /** Sensors to monitor */
  sensors: 'all' | 'critical' | 'custom';

  /** Enable logging */
  logging: boolean;

  /** Alert configuration */
  alerts: {
    console: boolean;
    email?: string;
    sms?: string;
  };
}

// ============================================================================
// Operational Parameters
// ============================================================================

/**
 * Operational parameters
 */
export interface OperationalParameters {
  /** Maximum displacement range (years) */
  maxDisplacementRange: number;

  /** Maximum acceleration (g) */
  maxAcceleration: number;

  /** Cruise temporal field strength (Tesla) */
  cruiseFieldStrength: number;

  /** Operating temperature range (Celsius) */
  temperatureRange: [number, number];

  /** Operating pressure range (kPa) */
  pressureRange: [number, number];
}

/**
 * Safety configuration
 */
export interface SafetyConfig {
  /** Safety level */
  level: 'minimum' | 'standard' | 'maximum';

  /** Auto-abort enabled */
  autoAbort: boolean;

  /** Emergency beacon always active */
  beaconAlwaysOn: boolean;

  /** Pre-flight check required */
  preFlightCheckRequired: boolean;

  /** Paradox prevention integration */
  paradoxPrevention: boolean;
}

// ============================================================================
// Temporal Types
// ============================================================================

/**
 * Temporal marker
 */
export interface TemporalMarker {
  /** Marker identifier */
  id: string;

  /** Placement timestamp */
  placement: Date;

  /** Location [lat, lon, alt] */
  location: [number, number, number];

  /** Chamber ID that placed marker */
  chamberId: string;

  /** Marker properties */
  properties: {
    beaconFrequency: number; // Hz
    signalStrength: number; // dBm
    battery: number; // hours remaining
    range: number; // meters
  };

  /** Purpose */
  purpose: 'return_reference' | 'emergency_beacon' | 'navigation';
}

/**
 * Timeline verification result
 */
export interface TimelineVerification {
  /** Destination timeline ID */
  timelineId: string;

  /** Target time */
  targetTime: Date;

  /** Target location [lat, lon, alt] */
  targetLocation: [number, number, number];

  /** Verification checks */
  checks: {
    timelineStability: number; // percentage
    historicalConsistency: boolean;
    physicalLaws: boolean;
    safeToExit: boolean;
  };

  /** Risk assessment */
  risks: {
    paradoxProbability: number;
    timelineCorruption: number;
    environmentalHazards: string[];
  };
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  ChamberIdentity,
  ChamberConfig,
  ChamberSpecifications,
  LayerSpecification,
  LayerCondition,
  TemporalFieldConfig,
  ContainmentConfig,
  TemporalFieldStatus,
  FieldSensorReading,
  LifeSupportStatus,
  OxygenSystemStatus,
  CO2RemovalStatus,
  AtmosphericComposition,
  CirculationStatus,
  EnvironmentalSettings,
  LightingSettings,
  ClimateZone,
  EnvironmentalStatus,
  Passenger,
  VitalSigns,
  VitalSignAlert,
  InertialDampeningStatus,
  MedicalSystemStatus,
  MedicationInventory,
  AirlockStatus,
  DoorStatus,
  EntryRequest,
  LuggageItem,
  ExitRequest,
  CalibrationRequest,
  TemporalFieldCalibration,
  SpatialCalibration,
  SensorCalibration,
  SystemsCheck,
  CalibrationResult,
  TestResult,
  AbortRequest,
  AbortResult,
  PassengerStatus,
  SystemStatus,
  ChamberStatusReport,
  StructuralStatus,
  PassengerSystemStatus,
  PowerStatus,
  Alert,
  MissionConfig,
  MonitoringConfig,
  OperationalParameters,
  SafetyConfig,
  TemporalMarker,
  TimelineVerification,
};

export {
  ChamberStatus,
  AirlockState,
  EmergencyType,
  EmergencySeverity,
};

// ============================================================================
// Constants
// ============================================================================

/**
 * Chronosphere Chamber constants
 */
export const CHRONOSPHERE_CONSTANTS = {
  /** Standard capacity */
  STANDARD_CAPACITY: 4, // passengers

  /** Standard external radius */
  STANDARD_RADIUS: 2.0, // meters

  /** Standard empty weight */
  STANDARD_WEIGHT: 2500, // kg

  /** Life support duration */
  LIFE_SUPPORT_DURATION: 168, // hours (7 days)

  /** Standard field strength */
  STANDARD_FIELD_STRENGTH: 1e8, // Tesla

  /** Maximum displacement range */
  MAX_DISPLACEMENT: 1e9, // years

  /** Calibration interval */
  CALIBRATION_INTERVAL: 30, // days

  /** Certification validity */
  CERTIFICATION_VALIDITY: 365, // days

  /** Emergency power reserve */
  EMERGENCY_POWER: 72, // hours
} as const;
