/**
 * WIA-AUTO-029: Vehicle-to-Grid (V2G) - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive & Energy Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Vehicle Types
// ============================================================================

/**
 * Vehicle identification and basic information
 */
export interface Vehicle {
  /** Unique vehicle identifier */
  vehicleId: string;

  /** Vehicle manufacturer */
  make: string;

  /** Vehicle model */
  model: string;

  /** Model year */
  year: number;

  /** Vehicle VIN (optional) */
  vin?: string;

  /** Owner/operator information */
  owner?: {
    id: string;
    name: string;
    email?: string;
  };
}

/**
 * Battery specifications and state
 */
export interface BatteryState {
  /** Total battery capacity in kWh */
  capacity: number;

  /** Current state of charge (0-100%) */
  soc: number;

  /** State of health (0-100%) */
  soh: number;

  /** Battery chemistry type */
  chemistry: 'NMC' | 'NCA' | 'LFP' | 'LTO' | 'NCM811' | 'other';

  /** Current battery temperature in °C */
  temperature: number;

  /** Current battery voltage in V */
  voltage: number;

  /** Current battery current in A (positive = charging) */
  current: number;

  /** Cumulative charge/discharge cycles */
  cycleCount?: number;

  /** Estimated remaining useful life (%) */
  remainingLife?: number;
}

/**
 * Power capabilities and current power flow
 */
export interface PowerCapability {
  /** Current power flow in kW (positive = charging, negative = discharging) */
  currentPower: number;

  /** Maximum charging power in kW */
  maxChargePower: number;

  /** Maximum discharging power in kW */
  maxDischargePower: number;

  /** Current operating mode */
  mode: 'idle' | 'charging' | 'discharging' | 'frequency-regulation' | 'error';

  /** Connector type */
  connector?: 'Type1' | 'Type2' | 'CCS1' | 'CCS2' | 'CHAdeMO' | 'Tesla';

  /** Supported communication protocols */
  protocols?: ('ISO15118' | 'OCPP' | 'OpenADR')[];
}

// ============================================================================
// Connection and Session
// ============================================================================

/**
 * EVSE (charging station) connection information
 */
export interface ConnectionState {
  /** Is vehicle currently connected to EVSE? */
  connected: boolean;

  /** EVSE identifier */
  evseId?: string;

  /** Connection timestamp */
  connectionTime?: Date | string;

  /** Planned departure time */
  plannedDeparture?: Date | string;

  /** Physical location */
  location?: {
    latitude: number;
    longitude: number;
    address?: string;
  };

  /** EVSE capabilities */
  evseCapabilities?: {
    maxPower: number;
    supportsV2G: boolean;
    protocols: string[];
  };
}

/**
 * V2G session information
 */
export interface V2GSession {
  /** Unique session identifier */
  sessionId: string;

  /** Vehicle ID */
  vehicleId: string;

  /** EVSE ID */
  evseId: string;

  /** Session start time */
  startTime: Date | string;

  /** Session end time (null if ongoing) */
  endTime?: Date | string | null;

  /** Session status */
  status: 'pending' | 'active' | 'paused' | 'completed' | 'error' | 'cancelled';

  /** Active grid service type */
  serviceType?: GridServiceType;

  /** Energy delivered to vehicle (kWh) */
  energyCharged: number;

  /** Energy delivered to grid (kWh) */
  energyDischarged: number;

  /** Revenue earned during session */
  revenue: number;

  /** Session preferences */
  preferences?: V2GPreferences;
}

/**
 * User preferences for V2G operation
 */
export interface V2GPreferences {
  /** Target SoC at end of session (%) */
  targetSoC: number;

  /** Minimum SoC to maintain (%) */
  minSoC: number;

  /** Maximum SoC allowed (%) */
  maxSoC: number;

  /** Maximum depth of discharge allowed (%) */
  maxDoD: number;

  /** Allow bidirectional power flow? */
  allowV2G: boolean;

  /** Maximum battery degradation per day (% SoH) */
  degradationLimit: number;

  /** Priority: revenue, battery-health, convenience */
  priority: 'revenue' | 'battery-health' | 'convenience' | 'balanced';

  /** Participation in specific grid services */
  allowedServices?: GridServiceType[];
}

// ============================================================================
// Grid Services
// ============================================================================

/**
 * Types of grid services available
 */
export type GridServiceType =
  | 'frequency-regulation'
  | 'peak-shaving'
  | 'load-balancing'
  | 'spinning-reserve'
  | 'voltage-support'
  | 'demand-response'
  | 'energy-arbitrage';

/**
 * Grid service request from operator
 */
export interface GridServiceRequest {
  /** Unique request identifier */
  requestId: string;

  /** Timestamp of request */
  timestamp: Date | string;

  /** Type of service requested */
  serviceType: GridServiceType;

  /** Duration of service in seconds */
  duration: number;

  /** Service start time */
  startTime: Date | string;

  /** Service end time */
  endTime: Date | string;

  /** Power profile requirements */
  powerProfile: {
    baseline: number;
    maxCharge: number;
    maxDischarge: number;
    rampRate: number; // kW/s
  };

  /** Compensation structure */
  compensation: {
    capacityRate: number; // $/kW/h
    energyRate: number; // $/kWh
    performanceBonus?: number;
    currency: string;
  };

  /** Service requirements */
  requirements: {
    responseTime: number; // seconds
    availability: number; // 0-1
    accuracyTolerance: number; // 0-1
  };
}

/**
 * Grid service response from vehicle
 */
export interface GridServiceResponse {
  /** Request ID being responded to */
  requestId: string;

  /** Vehicle ID */
  vehicleId: string;

  /** Accept or decline service */
  accepted: boolean;

  /** Reason for declining (if applicable) */
  reason?: string;

  /** Available capacity */
  availableCapacity?: {
    chargeCapacity: number; // kW
    dischargeCapacity: number; // kW
  };

  /** Estimated revenue */
  estimatedRevenue?: number;

  /** Estimated battery impact */
  estimatedDegradation?: number;
}

// ============================================================================
// Energy Management
// ============================================================================

/**
 * Energy pricing information
 */
export interface EnergyPrice {
  /** Timestamp */
  timestamp: Date | string;

  /** Energy price in $/kWh */
  price: number;

  /** Price type */
  priceType: 'real-time' | 'day-ahead' | 'time-of-use' | 'fixed';

  /** Pricing tier (if applicable) */
  tier?: 'off-peak' | 'mid-peak' | 'on-peak' | 'super-peak';

  /** Forecasted next price (if available) */
  forecast?: {
    nextHourPrice: number;
    nextDayAveragePrice: number;
  };
}

/**
 * Energy arbitrage opportunity
 */
export interface ArbitrageOpportunity {
  /** Opportunity ID */
  id: string;

  /** Buy time window */
  buyWindow: {
    start: Date | string;
    end: Date | string;
    price: number; // $/kWh
  };

  /** Sell time window */
  sellWindow: {
    start: Date | string;
    end: Date | string;
    price: number; // $/kWh
  };

  /** Energy amount in kWh */
  energyAmount: number;

  /** Estimated revenue */
  estimatedRevenue: number;

  /** Estimated battery cycles consumed */
  cyclesCost: number;

  /** Net profit after degradation */
  netProfit: number;

  /** Confidence level (0-1) */
  confidence: number;
}

/**
 * Charging schedule optimization
 */
export interface ChargingSchedule {
  /** Schedule ID */
  scheduleId: string;

  /** Vehicle ID */
  vehicleId: string;

  /** Schedule time periods */
  periods: ChargingPeriod[];

  /** Total energy planned (kWh) */
  totalEnergy: number;

  /** Total estimated cost */
  totalCost: number;

  /** Total estimated revenue */
  totalRevenue: number;

  /** Net cost/revenue */
  netAmount: number;

  /** Final SoC at departure */
  finalSoC: number;

  /** Optimization objective used */
  objective: 'minimize-cost' | 'maximize-revenue' | 'maximize-battery-life';
}

/**
 * Individual charging period in schedule
 */
export interface ChargingPeriod {
  /** Period start time */
  startTime: Date | string;

  /** Period end time */
  endTime: Date | string;

  /** Power level (kW, positive = charge, negative = discharge) */
  power: number;

  /** Energy in period (kWh) */
  energy: number;

  /** Energy price ($/kWh) */
  price: number;

  /** Cost or revenue for period */
  amount: number;

  /** Purpose of period */
  purpose: 'charging' | 'discharging' | 'idle' | 'grid-service';
}

// ============================================================================
// Battery Health and Degradation
// ============================================================================

/**
 * Battery degradation model parameters
 */
export interface DegradationModel {
  /** Calendar aging parameters */
  calendarAging: {
    alphaFactor: number;
    activationEnergy: number; // J/mol
    referenceTemperature: number; // °C
  };

  /** Cycle aging parameters */
  cycleAging: {
    betaFactor: number;
    dodExponent: number;
    rateExponent: number;
  };

  /** Current degradation state */
  currentDegradation: {
    calendarLoss: number; // %
    cycleLoss: number; // %
    totalLoss: number; // %
  };

  /** Projected degradation */
  projection?: {
    months: number;
    expectedSoH: number;
  };
}

/**
 * Battery health assessment
 */
export interface BatteryHealth {
  /** Current state of health (%) */
  stateOfHealth: number;

  /** Total cycle count */
  cycleCount: number;

  /** Equivalent full cycles */
  equivalentFullCycles: number;

  /** Average depth of discharge */
  averageDoD: number;

  /** Average temperature during operation */
  averageTemperature: number;

  /** Estimated remaining cycles */
  remainingCycles: number;

  /** Estimated remaining years */
  remainingYears: number;

  /** Degradation rate (%/year) */
  degradationRate: number;

  /** Health status */
  status: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Recommendations */
  recommendations?: string[];
}

/**
 * Battery protection configuration
 */
export interface BatteryProtection {
  /** Minimum allowed SoC (%) */
  minSoC: number;

  /** Maximum allowed SoC (%) */
  maxSoC: number;

  /** Maximum depth of discharge per day (%) */
  maxDailyDoD: number;

  /** Maximum cycles per day */
  maxCyclesPerDay: number;

  /** Temperature limits */
  temperatureLimits: {
    min: number; // °C
    max: number; // °C
    optimal: number; // °C
  };

  /** Power limits */
  powerLimits: {
    maxChargeRate: number; // C-rate
    maxDischargeRate: number; // C-rate
  };

  /** Degradation threshold for alerts */
  degradationAlert: number; // % per month
}

// ============================================================================
// Revenue and Billing
// ============================================================================

/**
 * Revenue breakdown
 */
export interface RevenueBreakdown {
  /** Energy arbitrage revenue */
  energyArbitrage: {
    revenue: number;
    transactions: number;
  };

  /** Frequency regulation revenue */
  frequencyRegulation: {
    revenue: number;
    hours: number;
  };

  /** Demand response revenue */
  demandResponse: {
    revenue: number;
    events: number;
  };

  /** Peak shaving revenue */
  peakShaving: {
    revenue: number;
    savings: number;
  };

  /** Other grid services */
  otherServices?: {
    [serviceName: string]: {
      revenue: number;
      units: number;
    };
  };
}

/**
 * Cost breakdown
 */
export interface CostBreakdown {
  /** Energy purchase cost */
  energyPurchase: number;

  /** Battery degradation cost */
  degradation: number;

  /** Service/platform fees */
  serviceFee: number;

  /** Other costs */
  otherCosts?: {
    [costType: string]: number;
  };
}

/**
 * Revenue report
 */
export interface RevenueReport {
  /** Vehicle ID */
  vehicleId: string;

  /** Report period */
  reportPeriod: {
    start: Date | string;
    end: Date | string;
  };

  /** Summary totals */
  summary: {
    totalRevenue: number;
    totalCost: number;
    netRevenue: number;
    energyDelivered: number; // kWh to grid
    energyConsumed: number; // kWh from grid
  };

  /** Detailed breakdown */
  breakdown: RevenueBreakdown;

  /** Cost details */
  costs: CostBreakdown;

  /** Battery health impact */
  batteryHealth: {
    cyclesThisPeriod: number;
    averageDoD: number;
    estimatedDegradation: number; // % SoH
  };

  /** Performance metrics */
  performance?: {
    availability: number; // %
    responseAccuracy: number; // %
    serviceUptime: number; // hours
  };
}

// ============================================================================
// Grid Integration
// ============================================================================

/**
 * Grid status and parameters
 */
export interface GridStatus {
  /** Current grid frequency in Hz */
  frequency: number;

  /** Grid voltage in V */
  voltage: number;

  /** Grid phase (for 3-phase) */
  phase?: 'A' | 'B' | 'C';

  /** Current grid load in MW */
  totalLoad?: number;

  /** Renewable generation percentage */
  renewablePct?: number;

  /** Grid stability indicator */
  stability: 'stable' | 'unstable' | 'critical';

  /** Current marginal price ($/MWh) */
  marginalPrice?: number;
}

/**
 * Aggregator platform information
 */
export interface Aggregator {
  /** Aggregator ID */
  aggregatorId: string;

  /** Aggregator name */
  name: string;

  /** Total fleet capacity */
  fleetCapacity: {
    chargeCapacity: number; // MW
    dischargeCapacity: number; // MW
    connectedVehicles: number;
  };

  /** Revenue share model */
  revenueShare: {
    vehicleOwner: number; // %
    aggregator: number; // %
  };

  /** Supported grid services */
  supportedServices: GridServiceType[];

  /** Platform status */
  status: 'active' | 'maintenance' | 'offline';
}

// ============================================================================
// ISO 15118 Protocol Types
// ============================================================================

/**
 * ISO 15118 charge parameters
 */
export interface ISO15118ChargeParameters {
  /** Energy transfer mode */
  energyTransferMode:
    | 'AC_single_phase_core'
    | 'AC_three_phase_core'
    | 'AC_single_phase_core_BPT'
    | 'AC_three_phase_core_BPT'
    | 'DC_core'
    | 'DC_extended'
    | 'DC_combo_core'
    | 'DC_unique';

  /** Maximum power limit (W) */
  maxPowerLimit: number;

  /** Maximum current limit (A) */
  maxCurrentLimit: number;

  /** Maximum voltage limit (V) */
  maxVoltageLimit: number;

  /** Minimum current limit (A) - negative for discharge */
  minCurrentLimit?: number;

  /** Minimum voltage limit (V) */
  minVoltageLimit?: number;

  /** Power ramp limitation (W/s) */
  powerRampLimitation?: number;

  /** Departure time */
  departureTime?: Date | string;

  /** Target SoC (%) */
  targetSoC?: number;
}

/**
 * ISO 15118 power delivery request
 */
export interface PowerDeliveryRequest {
  /** Charge progress */
  chargeProgress: 'Start' | 'Stop' | 'Standby' | 'Renegotiate';

  /** Current power demand (W) */
  currentPower?: number;

  /** Target voltage (V) */
  targetVoltage?: number;

  /** Target current (A) */
  targetCurrent?: number;

  /** Bulk charging complete? */
  bulkChargingComplete?: boolean;

  /** Charging complete? */
  chargingComplete?: boolean;
}

// ============================================================================
// Events and Notifications
// ============================================================================

/**
 * V2G event types
 */
export type V2GEventType =
  | 'session-start'
  | 'session-end'
  | 'connection-established'
  | 'connection-lost'
  | 'service-request'
  | 'service-start'
  | 'service-end'
  | 'power-update'
  | 'soc-update'
  | 'revenue-update'
  | 'battery-alert'
  | 'grid-alert'
  | 'error';

/**
 * V2G event
 */
export interface V2GEvent {
  /** Event ID */
  eventId: string;

  /** Event type */
  type: V2GEventType;

  /** Timestamp */
  timestamp: Date | string;

  /** Vehicle ID */
  vehicleId: string;

  /** Session ID (if applicable) */
  sessionId?: string;

  /** Event data */
  data: Record<string, unknown>;

  /** Severity level */
  severity: 'info' | 'warning' | 'error' | 'critical';

  /** Human-readable message */
  message: string;
}

// ============================================================================
// Configuration and Settings
// ============================================================================

/**
 * V2G controller configuration
 */
export interface V2GConfig {
  /** Vehicle information */
  vehicleId: string;

  /** Battery capacity (kWh) */
  batteryCapacity: number;

  /** Maximum charge power (kW) */
  maxChargePower: number;

  /** Maximum discharge power (kW) */
  maxDischargePower: number;

  /** Default preferences */
  defaultPreferences: V2GPreferences;

  /** Battery protection settings */
  batteryProtection: BatteryProtection;

  /** Aggregator settings */
  aggregator?: {
    aggregatorId: string;
    apiEndpoint: string;
    apiKey: string;
  };

  /** Communication settings */
  communication?: {
    protocol: 'ISO15118' | 'OCPP' | 'OpenADR';
    endpoint: string;
    certificatePath?: string;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Standard API response wrapper
 */
export interface ApiResponse<T> {
  /** Success status */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error information */
  error?: {
    code: string;
    message: string;
    details?: Record<string, unknown>;
  };

  /** Response timestamp */
  timestamp: Date | string;

  /** Request ID for tracking */
  requestId?: string;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Array of items */
  items: T[];

  /** Total count of items */
  total: number;

  /** Current page */
  page: number;

  /** Items per page */
  pageSize: number;

  /** Has more pages? */
  hasMore: boolean;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Time series data point
 */
export interface TimeSeriesPoint<T = number> {
  timestamp: Date | string;
  value: T;
}

/**
 * Time series data
 */
export type TimeSeries<T = number> = TimeSeriesPoint<T>[];

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; value: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Constants
// ============================================================================

/**
 * Physical constants for V2G calculations
 */
export const V2G_CONSTANTS = {
  /** Standard grid frequency (Hz) */
  GRID_FREQUENCY_60HZ: 60,
  GRID_FREQUENCY_50HZ: 50,

  /** Standard grid voltages (V) */
  GRID_VOLTAGE_120V: 120,
  GRID_VOLTAGE_240V: 240,
  GRID_VOLTAGE_400V: 400,

  /** Typical efficiency values */
  CHARGE_EFFICIENCY: 0.92,
  DISCHARGE_EFFICIENCY: 0.88,
  ROUND_TRIP_EFFICIENCY: 0.85,

  /** Battery temperature limits (°C) */
  BATTERY_TEMP_MIN: 10,
  BATTERY_TEMP_MAX: 40,
  BATTERY_TEMP_OPTIMAL: 25,

  /** SoC operating windows (%) */
  SOC_MIN_CONSERVATIVE: 30,
  SOC_MAX_CONSERVATIVE: 80,
  SOC_MIN_AGGRESSIVE: 20,
  SOC_MAX_AGGRESSIVE: 90,

  /** C-rate limits */
  C_RATE_MAX_CHARGE: 1.0,
  C_RATE_MAX_DISCHARGE: 1.0,

  /** Degradation coefficients (example values) */
  DEGRADATION_ALPHA: 0.0001,
  DEGRADATION_BETA: 0.00005,
  DEGRADATION_GAMMA: 0.000001,

  /** Revenue sharing defaults */
  DEFAULT_OWNER_SHARE: 0.70,
  DEFAULT_AGGREGATOR_SHARE: 0.30,
} as const;

/**
 * Error codes for V2G operations
 */
export enum V2GErrorCode {
  // Connection errors
  CONNECTION_FAILED = 'V2G_001',
  CONNECTION_LOST = 'V2G_002',
  EVSE_NOT_COMPATIBLE = 'V2G_003',

  // Battery errors
  BATTERY_TEMPERATURE_HIGH = 'V2G_010',
  BATTERY_TEMPERATURE_LOW = 'V2G_011',
  SOC_TOO_LOW = 'V2G_012',
  SOC_TOO_HIGH = 'V2G_013',
  BATTERY_FAULT = 'V2G_014',

  // Grid errors
  GRID_FREQUENCY_OUT_OF_RANGE = 'V2G_020',
  GRID_VOLTAGE_OUT_OF_RANGE = 'V2G_021',
  GRID_FAULT = 'V2G_022',

  // Service errors
  SERVICE_REQUEST_REJECTED = 'V2G_030',
  SERVICE_CAPACITY_INSUFFICIENT = 'V2G_031',
  SERVICE_TIMEOUT = 'V2G_032',

  // Communication errors
  PROTOCOL_ERROR = 'V2G_040',
  AUTHENTICATION_FAILED = 'V2G_041',
  MESSAGE_TIMEOUT = 'V2G_042',

  // Safety errors
  OVER_CURRENT = 'V2G_050',
  OVER_VOLTAGE = 'V2G_051',
  GROUND_FAULT = 'V2G_052',
  ISLANDING_DETECTED = 'V2G_053',
}

/**
 * V2G error class
 */
export class V2GError extends Error {
  constructor(
    public code: V2GErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'V2GError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  Vehicle,
  BatteryState,
  PowerCapability,
  ConnectionState,
  V2GSession,
  V2GPreferences,

  // Grid services
  GridServiceRequest,
  GridServiceResponse,

  // Energy management
  EnergyPrice,
  ArbitrageOpportunity,
  ChargingSchedule,
  ChargingPeriod,

  // Battery health
  DegradationModel,
  BatteryHealth,
  BatteryProtection,

  // Revenue
  RevenueBreakdown,
  CostBreakdown,
  RevenueReport,

  // Grid integration
  GridStatus,
  Aggregator,

  // ISO 15118
  ISO15118ChargeParameters,
  PowerDeliveryRequest,

  // Events
  V2GEvent,

  // Configuration
  V2GConfig,

  // API types
  ApiResponse,
  PaginatedResponse,

  // Utility types
  TimeSeriesPoint,
  TimeSeries,
  Result,
  AsyncResult,
};

export { V2G_CONSTANTS, V2GErrorCode, V2GError };
