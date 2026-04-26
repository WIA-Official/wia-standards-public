/**
 * WIA-AUTO-005: EV Charging - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Geographic location
 */
export interface Location {
  latitude: number;
  longitude: number;
  address?: string;
  city?: string;
  state?: string;
  country?: string;
  postalCode?: string;
}

/**
 * Date-time range
 */
export interface TimeRange {
  start: Date | string;
  end: Date | string;
}

// ============================================================================
// Charging Levels and Connectors
// ============================================================================

/**
 * Charging level (1, 2, or 3)
 */
export type ChargingLevel = 1 | 2 | 3;

/**
 * Connector type
 */
export type ConnectorType =
  | 'CCS1'           // Combined Charging System Type 1 (North America)
  | 'CCS2'           // Combined Charging System Type 2 (Europe)
  | 'CHAdeMO'        // Japanese DC fast charging
  | 'Tesla'          // Tesla Supercharger
  | 'J1772'          // SAE J1772 (Level 1/2 AC)
  | 'Type2'          // IEC 62196 Type 2 (Mennekes)
  | 'GBT'            // GB/T (China)
  | 'NACS';          // North American Charging Standard (Tesla)

/**
 * Charging mode
 */
export type ChargingMode = 'AC' | 'DC';

/**
 * Connector standard details
 */
export interface ConnectorStandard {
  type: ConnectorType;
  mode: ChargingMode;
  maxPower: number;        // kW
  maxVoltage: number;      // V
  maxCurrent: number;      // A
  region: string[];
  v2gCapable: boolean;
  description: string;
}

// ============================================================================
// Vehicle Information
// ============================================================================

/**
 * Vehicle type
 */
export type VehicleType = 'BEV' | 'PHEV';

/**
 * Battery chemistry
 */
export type BatteryChemistry = 'NMC' | 'LFP' | 'NCA' | 'LTO' | 'Solid-State';

/**
 * Battery information
 */
export interface BatteryInfo {
  /** Total battery capacity in kWh */
  capacity: number;

  /** Usable battery capacity in kWh */
  usableCapacity: number;

  /** Battery chemistry type */
  chemistry: BatteryChemistry;

  /** Warranty mileage */
  warrantyMiles: number;

  /** Warranty years */
  warrantyYears: number;

  /** Current state of health (0-1) */
  soh?: number;

  /** Number of charge cycles */
  cycles?: number;
}

/**
 * Charging capabilities
 */
export interface ChargingCapabilities {
  /** Supported connector types */
  connectors: ConnectorType[];

  /** Maximum AC charging power in kW */
  maxACPower: number;

  /** Maximum DC charging power in kW */
  maxDCPower: number;

  /** Maximum charging current in A */
  maxCurrent: number;

  /** Maximum charging voltage in V */
  maxVoltage: number;

  /** V2G capable */
  v2gCapable?: boolean;

  /** Supports Plug & Charge (ISO 15118) */
  plugAndChargeSupport?: boolean;
}

/**
 * Vehicle efficiency
 */
export interface VehicleEfficiency {
  /** Watt-hours per mile */
  whPerMile: number;

  /** EPA range in miles */
  range: number;

  /** City efficiency in miles/kWh */
  cityEfficiency?: number;

  /** Highway efficiency in miles/kWh */
  highwayEfficiency?: number;
}

/**
 * Complete vehicle information
 */
export interface VehicleInfo {
  vehicleId: string;
  vin?: string;
  make: string;
  model: string;
  year: number;
  type: VehicleType;
  battery: BatteryInfo;
  charging: ChargingCapabilities;
  efficiency: VehicleEfficiency;
  owner?: {
    userId: string;
    subscriptions?: string[];
  };
}

// ============================================================================
// Charging Station
// ============================================================================

/**
 * Connector status
 */
export type ConnectorStatus =
  | 'Available'
  | 'Charging'
  | 'Reserved'
  | 'Faulted'
  | 'Unavailable'
  | 'Preparing';

/**
 * Station amenities
 */
export type Amenity = 'WiFi' | 'Restroom' | 'Coffee' | 'Restaurant' | 'Shopping' | 'Covered';

/**
 * Access type
 */
export type AccessType = '24/7' | 'Business Hours' | 'Restricted' | 'Private';

/**
 * Pricing information
 */
export interface Pricing {
  currency: string;
  energyPrice?: number;      // $/kWh
  timePrice?: number;        // $/minute
  sessionFee?: number;       // $
  idleFee?: number;          // $/minute after charging
  parkingFee?: number;       // $/hour
}

/**
 * Current charging session info
 */
export interface CurrentSession {
  sessionId: string;
  startTime: Date | string;
  currentSOC?: number;       // 0-100
  currentPower?: number;     // kW
  energyDelivered?: number;  // kWh
  estimatedEndTime?: Date | string;
  userId?: string;
}

/**
 * Connector information
 */
export interface Connector {
  connectorId: number;
  type: ConnectorType;
  maxPower: number;          // kW
  status: ConnectorStatus;
  pricing?: Pricing;
  currentSession?: CurrentSession;
}

/**
 * Charging station information
 */
export interface ChargingStation {
  stationId: string;
  name?: string;
  location: Location;
  operator: string;
  connectors: Connector[];
  amenities?: Amenity[];
  access: AccessType;
  accessInstructions?: string;
  parkingInfo?: string;
  lastUpdated: Date | string;

  /** Network/roaming memberships */
  networks?: string[];

  /** Station photos */
  photos?: string[];

  /** User ratings */
  rating?: number;

  /** Number of reviews */
  reviewCount?: number;
}

// ============================================================================
// Charging Session
// ============================================================================

/**
 * Charging curve data point
 */
export interface ChargingCurvePoint {
  time: string;              // HH:MM format
  power: number;             // kW
  soc: number;               // 0-100
  voltage?: number;          // V
  current?: number;          // A
  temperature?: number;      // °C
}

/**
 * Payment information
 */
export interface PaymentInfo {
  method: PaymentMethod;
  cardLast4?: string;
  transactionId: string;
  status: PaymentStatus;
  authorization?: string;
}

/**
 * Payment method
 */
export type PaymentMethod =
  | 'credit_card'
  | 'debit_card'
  | 'rfid'
  | 'mobile_app'
  | 'plug_and_charge'
  | 'subscription';

/**
 * Payment status
 */
export type PaymentStatus =
  | 'pending'
  | 'authorized'
  | 'completed'
  | 'failed'
  | 'refunded';

/**
 * Complete charging session record
 */
export interface ChargingSession {
  sessionId: string;
  stationId: string;
  connectorId: number;
  connectorType: ConnectorType;

  /** Vehicle information */
  vehicleId?: string;
  vin?: string;

  /** User information */
  userId: string;

  /** Session timing */
  startTime: Date | string;
  endTime?: Date | string;
  duration?: number;         // seconds

  /** Energy and state of charge */
  startSOC: number;          // 0-100
  endSOC?: number;           // 0-100
  energyDelivered: number;   // kWh

  /** Power metrics */
  averagePower: number;      // kW
  peakPower: number;         // kW
  minPower?: number;         // kW

  /** Detailed charging curve */
  chargingCurve?: ChargingCurvePoint[];

  /** Pricing and payment */
  pricing: {
    energyPrice: number;     // $/kWh
    timePrice?: number;      // $/minute
    sessionFee?: number;     // $
    idleFee?: number;        // $
    totalCost: number;       // $
  };

  payment: PaymentInfo;

  /** Session metadata */
  status: SessionStatus;
  stopReason?: StopReason;
  errors?: string[];
}

/**
 * Session status
 */
export type SessionStatus =
  | 'pending'
  | 'authorized'
  | 'charging'
  | 'suspended'
  | 'completed'
  | 'failed';

/**
 * Stop reason
 */
export type StopReason =
  | 'user_stopped'
  | 'target_reached'
  | 'departure_time'
  | 'error'
  | 'emergency_stop'
  | 'remote_stop'
  | 'power_outage'
  | 'overheat';

// ============================================================================
// Charging Calculations
// ============================================================================

/**
 * Charging time calculation request
 */
export interface ChargingTimeRequest {
  /** Battery capacity in kWh */
  batteryCapacity: number;

  /** Current state of charge (0-1) */
  currentSOC: number;

  /** Target state of charge (0-1) */
  targetSOC: number;

  /** Charging power in kW */
  chargingPower: number;

  /** Charging efficiency (0-1), default 0.9 */
  efficiency?: number;

  /** Account for charging curve taper */
  useChargingCurve?: boolean;

  /** Battery chemistry for curve calculation */
  batteryChemistry?: BatteryChemistry;
}

/**
 * Charging time calculation response
 */
export interface ChargingTimeResponse {
  /** Charging time in hours */
  hours: number;

  /** Charging time in minutes */
  minutes: number;

  /** Energy to be delivered in kWh */
  energyDelivered: number;

  /** Final state of charge (0-1) */
  finalSOC: number;

  /** Estimated cost if pricing available */
  estimatedCost?: number;

  /** Range added in miles */
  rangeAdded?: number;

  /** Average power during session */
  averagePower?: number;
}

/**
 * Charging cost estimation request
 */
export interface ChargingCostRequest {
  /** Energy delivered in kWh */
  energy: number;

  /** Pricing information */
  pricing: Pricing;

  /** Session duration in minutes */
  duration?: number;

  /** Idle time in minutes */
  idleTime?: number;

  /** Parking duration in hours */
  parkingDuration?: number;
}

/**
 * Charging cost estimation response
 */
export interface ChargingCostResponse {
  /** Energy cost */
  energyCost: number;

  /** Time-based cost */
  timeCost?: number;

  /** Session fee */
  sessionFee?: number;

  /** Idle fee */
  idleFee?: number;

  /** Parking fee */
  parkingFee?: number;

  /** Total cost */
  totalCost: number;

  /** Cost breakdown */
  breakdown: {
    component: string;
    cost: number;
    unit?: string;
  }[];
}

// ============================================================================
// Session Validation
// ============================================================================

/**
 * Charging session validation request
 */
export interface SessionValidation {
  connectorType: ConnectorType;
  vehicleType: VehicleType;
  maxPower: number;          // kW
  batteryCapacity: number;   // kWh
  currentSOC?: number;       // 0-1
  vehicleConnectors?: ConnectorType[];
}

/**
 * Compatibility check result
 */
export interface CompatibilityCheck {
  /** Connector compatible */
  connector: boolean;

  /** Power level compatible */
  power: boolean;

  /** Communication protocol compatible */
  communication: boolean;

  /** Overall compatibility */
  overall: boolean;
}

/**
 * Validation result
 */
export interface ValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
  compatibility: CompatibilityCheck;
  estimatedTime?: ChargingTimeResponse;
  recommendations?: string[];
}

// ============================================================================
// Smart Charging
// ============================================================================

/**
 * Time-of-use rate
 */
export interface TouRate {
  period: string;            // e.g., "Off-Peak", "Mid-Peak", "On-Peak"
  start: string;             // HH:MM
  end: string;               // HH:MM
  price: number;             // $/kWh
  daysOfWeek?: number[];     // 0=Sunday, 6=Saturday
}

/**
 * Smart charging schedule
 */
export interface ChargingSchedule {
  scheduleId: string;
  vehicleId: string;

  /** Target departure time */
  departureTime: Date | string;

  /** Current and target SOC */
  currentSOC: number;        // 0-1
  targetSOC: number;         // 0-1

  /** Available charging power */
  maxPower: number;          // kW

  /** TOU rate schedule */
  touRates?: TouRate[];

  /** Calculated charging periods */
  chargingPeriods: ChargingPeriod[];

  /** Total estimated cost */
  estimatedCost: number;

  /** Total estimated energy */
  estimatedEnergy: number;   // kWh
}

/**
 * Charging period in schedule
 */
export interface ChargingPeriod {
  start: Date | string;
  end: Date | string;
  power: number;             // kW
  energy: number;            // kWh
  cost: number;              // $
  rate: number;              // $/kWh
}

/**
 * Load balancing configuration
 */
export interface LoadBalancing {
  /** Total available power */
  totalPower: number;        // kW

  /** Current building load */
  buildingLoad: number;      // kW

  /** Active charging sessions */
  activeSessions: number;

  /** Power allocation per session */
  powerPerSession: number;   // kW

  /** Priority levels */
  priorities?: {
    sessionId: string;
    priority: 'critical' | 'high' | 'medium' | 'low';
    allocatedPower: number;  // kW
  }[];
}

// ============================================================================
// V2G (Vehicle-to-Grid)
// ============================================================================

/**
 * V2G mode
 */
export type V2GMode = 'G2V' | 'V2G' | 'V2H' | 'V2B';

/**
 * V2G configuration
 */
export interface V2GConfig {
  mode: V2GMode;

  /** Enable V2G */
  enabled: boolean;

  /** Maximum discharge power */
  maxDischargePower: number; // kW

  /** Minimum battery SOC to maintain */
  minSOC: number;            // 0-1

  /** Maximum daily cycles */
  maxDailyCycles: number;

  /** Degradation compensation rate */
  degradationRate: number;   // $/kWh

  /** Grid frequency regulation */
  frequencyRegulation?: boolean;

  /** Peak shaving */
  peakShaving?: boolean;
}

/**
 * V2G transaction
 */
export interface V2GTransaction {
  transactionId: string;
  sessionId: string;
  mode: V2GMode;

  /** Energy discharged from vehicle */
  energyDischarged: number;  // kWh

  /** Energy charged to vehicle */
  energyCharged: number;     // kWh

  /** Net energy (negative = discharged) */
  netEnergy: number;         // kWh

  /** Revenue from discharge */
  dischargeRevenue: number;  // $

  /** Cost of charging */
  chargeCost: number;        // $

  /** Degradation cost */
  degradationCost: number;   // $

  /** Net revenue (can be negative) */
  netRevenue: number;        // $

  /** Battery cycles consumed */
  cyclesConsumed: number;

  timestamp: Date | string;
}

// ============================================================================
// OCPP Protocol
// ============================================================================

/**
 * OCPP message type
 */
export type OCPPMessageType =
  | 'StatusNotification'
  | 'StartTransaction'
  | 'StopTransaction'
  | 'MeterValues'
  | 'Authorize'
  | 'BootNotification'
  | 'Heartbeat'
  | 'RemoteStartTransaction'
  | 'RemoteStopTransaction';

/**
 * OCPP status
 */
export type OCPPStatus =
  | 'Available'
  | 'Preparing'
  | 'Charging'
  | 'SuspendedEVSE'
  | 'SuspendedEV'
  | 'Finishing'
  | 'Reserved'
  | 'Unavailable'
  | 'Faulted';

/**
 * OCPP error code
 */
export type OCPPErrorCode =
  | 'NoError'
  | 'ConnectorLockFailure'
  | 'EVCommunicationError'
  | 'GroundFailure'
  | 'HighTemperature'
  | 'InternalError'
  | 'LocalListConflict'
  | 'OtherError'
  | 'OverCurrentFailure'
  | 'PowerMeterFailure'
  | 'PowerSwitchFailure'
  | 'ReaderFailure'
  | 'ResetFailure'
  | 'UnderVoltage'
  | 'OverVoltage'
  | 'WeakSignal';

/**
 * OCPP message
 */
export interface OCPPMessage {
  messageType: OCPPMessageType;
  connectorId?: number;
  status?: OCPPStatus;
  errorCode?: OCPPErrorCode;
  timestamp: Date | string;
  transactionId?: number;
  meterValue?: number;
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Safety and Monitoring
// ============================================================================

/**
 * Safety check type
 */
export type SafetyCheckType =
  | 'ground_fault'
  | 'overcurrent'
  | 'overvoltage'
  | 'undervoltage'
  | 'overtemperature'
  | 'communication'
  | 'emergency_stop';

/**
 * Safety check result
 */
export interface SafetyCheck {
  type: SafetyCheckType;
  status: 'pass' | 'fail' | 'warning';
  value?: number;
  threshold?: number;
  message: string;
  timestamp: Date | string;
}

/**
 * Monitoring data point
 */
export interface MonitoringData {
  timestamp: Date | string;
  voltage: number;           // V
  current: number;           // A
  power: number;             // kW
  energy: number;            // kWh (cumulative)
  temperature: number;       // °C
  soc?: number;              // 0-100
  frequency?: number;        // Hz
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical constants and limits
 */
export const CHARGING_CONSTANTS = {
  /** Standard AC frequencies */
  AC_FREQUENCY_NA: 60,       // Hz (North America)
  AC_FREQUENCY_EU: 50,       // Hz (Europe)

  /** Voltage levels */
  LEVEL1_VOLTAGE_NA: 120,    // V
  LEVEL1_VOLTAGE_EU: 230,    // V
  LEVEL2_VOLTAGE: 240,       // V
  DC_VOLTAGE_MIN: 200,       // V
  DC_VOLTAGE_MAX: 1000,      // V

  /** Current limits */
  LEVEL1_CURRENT_MAX: 16,    // A
  LEVEL2_CURRENT_MAX: 80,    // A
  DC_CURRENT_MAX: 500,       // A

  /** Power limits */
  LEVEL1_POWER_MAX: 1.9,     // kW
  LEVEL2_POWER_MAX: 19.2,    // kW
  DC_FAST_POWER_MIN: 50,     // kW
  DC_FAST_POWER_MAX: 350,    // kW

  /** Efficiency */
  DEFAULT_EFFICIENCY: 0.9,
  AC_EFFICIENCY: 0.88,
  DC_EFFICIENCY: 0.93,

  /** Safety thresholds */
  MAX_CABLE_TEMP: 85,        // °C
  MAX_CONNECTOR_TEMP: 90,    // °C
  GROUND_FAULT_LIMIT: 20,    // mA
  VOLTAGE_TOLERANCE: 0.10,   // ±10%

  /** SOC limits */
  MIN_SOC: 0.0,
  MAX_SOC: 1.0,
  FAST_CHARGE_SOC_LIMIT: 0.8,  // Typically slow down after 80%

  /** Battery degradation */
  DEGRADATION_COST_PER_KWH: 0.05,  // $/kWh
  MAX_DAILY_CYCLES: 2.0,

  /** Time limits */
  IDLE_GRACE_PERIOD: 10,     // minutes
  MAX_SESSION_DURATION: 240, // minutes (4 hours)
} as const;

/**
 * Connector standards reference
 */
export const CONNECTOR_STANDARDS: Record<ConnectorType, ConnectorStandard> = {
  CCS1: {
    type: 'CCS1',
    mode: 'DC',
    maxPower: 350,
    maxVoltage: 920,
    maxCurrent: 500,
    region: ['North America'],
    v2gCapable: true,
    description: 'Combined Charging System Type 1 (SAE J1772 + DC)',
  },
  CCS2: {
    type: 'CCS2',
    mode: 'DC',
    maxPower: 350,
    maxVoltage: 920,
    maxCurrent: 500,
    region: ['Europe', 'Asia', 'Rest of World'],
    v2gCapable: true,
    description: 'Combined Charging System Type 2 (IEC 62196 Type 2 + DC)',
  },
  CHAdeMO: {
    type: 'CHAdeMO',
    mode: 'DC',
    maxPower: 400,
    maxVoltage: 1000,
    maxCurrent: 400,
    region: ['Japan', 'Asia'],
    v2gCapable: true,
    description: 'Japanese DC fast charging standard',
  },
  Tesla: {
    type: 'Tesla',
    mode: 'DC',
    maxPower: 250,
    maxVoltage: 500,
    maxCurrent: 500,
    region: ['Global'],
    v2gCapable: false,
    description: 'Tesla Supercharger (proprietary)',
  },
  J1772: {
    type: 'J1772',
    mode: 'AC',
    maxPower: 19.2,
    maxVoltage: 240,
    maxCurrent: 80,
    region: ['North America', 'Japan'],
    v2gCapable: false,
    description: 'SAE J1772 Level 1/2 AC charging',
  },
  Type2: {
    type: 'Type2',
    mode: 'AC',
    maxPower: 43,
    maxVoltage: 400,
    maxCurrent: 63,
    region: ['Europe'],
    v2gCapable: false,
    description: 'IEC 62196 Type 2 (Mennekes) AC charging',
  },
  GBT: {
    type: 'GBT',
    mode: 'DC',
    maxPower: 237.5,
    maxVoltage: 950,
    maxCurrent: 250,
    region: ['China'],
    v2gCapable: true,
    description: 'GB/T Chinese standard',
  },
  NACS: {
    type: 'NACS',
    mode: 'DC',
    maxPower: 250,
    maxVoltage: 500,
    maxCurrent: 500,
    region: ['North America'],
    v2gCapable: false,
    description: 'North American Charging Standard (Tesla connector becoming industry standard)',
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-005 error codes
 */
export enum ChargingErrorCode {
  INVALID_PARAMETERS = 'AUTO005-001',
  INCOMPATIBLE_CONNECTOR = 'AUTO005-002',
  INSUFFICIENT_POWER = 'AUTO005-003',
  BATTERY_OVERHEAT = 'AUTO005-004',
  COMMUNICATION_FAILURE = 'AUTO005-005',
  PAYMENT_FAILED = 'AUTO005-006',
  GROUND_FAULT = 'AUTO005-007',
  OVERCURRENT = 'AUTO005-008',
  OVERVOLTAGE = 'AUTO005-009',
  UNDERVOLTAGE = 'AUTO005-010',
  EMERGENCY_STOP = 'AUTO005-011',
  STATION_UNAVAILABLE = 'AUTO005-012',
}

/**
 * Charging error
 */
export class ChargingError extends Error {
  constructor(
    public code: ChargingErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ChargingError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Location,
  TimeRange,

  // Connectors
  ConnectorStandard,

  // Vehicle
  VehicleInfo,
  BatteryInfo,
  ChargingCapabilities,
  VehicleEfficiency,

  // Station
  ChargingStation,
  Connector,
  Pricing,
  CurrentSession,

  // Session
  ChargingSession,
  ChargingCurvePoint,
  PaymentInfo,

  // Calculations
  ChargingTimeRequest,
  ChargingTimeResponse,
  ChargingCostRequest,
  ChargingCostResponse,

  // Validation
  SessionValidation,
  ValidationResult,
  CompatibilityCheck,

  // Smart Charging
  ChargingSchedule,
  ChargingPeriod,
  TouRate,
  LoadBalancing,

  // V2G
  V2GConfig,
  V2GTransaction,

  // OCPP
  OCPPMessage,

  // Safety
  SafetyCheck,
  MonitoringData,
};

export {
  CHARGING_CONSTANTS,
  CONNECTOR_STANDARDS,
  ChargingErrorCode,
  ChargingError,
};
