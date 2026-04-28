/**
 * WIA-ENERGY-004: Smart Grid Standard - TypeScript Type Definitions
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
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
  elevation?: number;
}

/**
 * Address information
 */
export interface Address {
  street?: string;
  city: string;
  state: string;
  country: string;
  postalCode: string;
  coordinates: Coordinates;
  timezone: string;
}

// ============================================================================
// Power Grid Topology Types
// ============================================================================

/**
 * Grid node types
 */
export enum GridNodeType {
  SUBSTATION = 'substation',
  TRANSFORMER = 'transformer',
  DISTRIBUTION_NODE = 'distribution-node',
  TRANSMISSION_LINE = 'transmission-line',
  CIRCUIT_BREAKER = 'circuit-breaker',
  LOAD_CENTER = 'load-center',
  GENERATION_POINT = 'generation-point',
}

/**
 * Voltage levels
 */
export enum VoltageLevel {
  TRANSMISSION_765KV = '765kV',      // Extra High Voltage
  TRANSMISSION_500KV = '500kV',      // Extra High Voltage
  TRANSMISSION_345KV = '345kV',      // High Voltage
  TRANSMISSION_230KV = '230kV',      // High Voltage
  TRANSMISSION_138KV = '138kV',      // High Voltage
  DISTRIBUTION_69KV = '69kV',        // Medium Voltage
  DISTRIBUTION_34_5KV = '34.5kV',    // Medium Voltage
  DISTRIBUTION_13_8KV = '13.8kV',    // Medium Voltage
  DISTRIBUTION_4_16KV = '4.16kV',    // Low Voltage
  SERVICE_480V = '480V',             // Low Voltage
  SERVICE_240V = '240V',             // Low Voltage
  SERVICE_120V = '120V',             // Low Voltage
}

/**
 * Grid topology node
 */
export interface GridNode {
  nodeId: string;
  name: string;
  type: GridNodeType;
  location: Address;
  voltageLevel: VoltageLevel;
  capacity: {
    rated: number;                   // MVA
    current: number;                 // MVA
    utilization: number;             // %
  };
  connectedNodes: string[];          // nodeIds
  operationalStatus: OperationalStatus;
  lastMaintenanceDate?: Timestamp;
  nextMaintenanceDate?: Timestamp;
}

/**
 * Transmission line
 */
export interface TransmissionLine {
  lineId: string;
  name: string;
  fromNode: string;                  // nodeId
  toNode: string;                    // nodeId
  voltageLevel: VoltageLevel;
  length: number;                    // km
  capacity: number;                  // MW
  impedance: {
    resistance: number;              // ohms
    reactance: number;               // ohms
  };
  lineType: 'overhead' | 'underground' | 'submarine';
  operationalStatus: OperationalStatus;
}

/**
 * Operational status
 */
export enum OperationalStatus {
  OPERATIONAL = 'operational',
  MAINTENANCE = 'maintenance',
  FAULT = 'fault',
  OFFLINE = 'offline',
  STANDBY = 'standby',
}

// ============================================================================
// Energy Generation and Distribution Types
// ============================================================================

/**
 * Generation source types
 */
export enum GenerationType {
  COAL = 'coal',
  NATURAL_GAS = 'natural-gas',
  NUCLEAR = 'nuclear',
  HYDRO = 'hydro',
  SOLAR_PV = 'solar-pv',
  SOLAR_THERMAL = 'solar-thermal',
  WIND_ONSHORE = 'wind-onshore',
  WIND_OFFSHORE = 'wind-offshore',
  BIOMASS = 'biomass',
  GEOTHERMAL = 'geothermal',
  TIDAL = 'tidal',
  BATTERY_STORAGE = 'battery-storage',
}

/**
 * Power plant information
 */
export interface PowerPlant {
  plantId: string;
  name: string;
  type: GenerationType;
  location: Address;
  capacity: {
    installed: number;               // MW
    available: number;               // MW
    current: number;                 // MW
  };
  efficiency: number;                // %
  emissionRate?: number;             // kg CO₂/MWh
  isRenewable: boolean;
  operationalStatus: OperationalStatus;
  rampRate: {
    up: number;                      // MW/min
    down: number;                    // MW/min
  };
}

/**
 * Real-time generation data
 */
export interface GenerationData {
  plantId: string;
  timestamp: Timestamp;
  activePower: number;               // MW
  reactivePower: number;             // MVAr
  voltage: number;                   // kV
  frequency: number;                 // Hz
  powerFactor: number;               // 0-1
  fuelConsumption?: number;          // units vary by type
  heatRate?: number;                 // BTU/kWh
  emissions?: {
    co2: number;                     // kg/h
    nox: number;                     // kg/h
    so2: number;                     // kg/h
  };
}

/**
 * Load distribution
 */
export interface LoadDistribution {
  zoneId: string;
  zoneName: string;
  timestamp: Timestamp;
  totalLoad: number;                 // MW
  breakdown: {
    residential: number;             // MW
    commercial: number;              // MW
    industrial: number;              // MW
    agricultural: number;            // MW
    other: number;                   // MW
  };
  peakLoad: {
    value: number;                   // MW
    timeOfDay: string;               // HH:MM
  };
}

// ============================================================================
// Smart Meter Types
// ============================================================================

/**
 * Smart meter types
 */
export enum SmartMeterType {
  RESIDENTIAL = 'residential',
  COMMERCIAL = 'commercial',
  INDUSTRIAL = 'industrial',
  AGRICULTURAL = 'agricultural',
}

/**
 * Smart meter information
 */
export interface SmartMeter {
  meterId: string;
  type: SmartMeterType;
  customerId: string;
  location: Address;
  installDate: Timestamp;
  firmwareVersion: string;
  communicationProtocol: 'zigbee' | 'lora' | 'cellular' | 'plc' | 'wifi';
  readingInterval: number;           // seconds
  dataRetention: number;             // days
  isActive: boolean;
}

/**
 * Smart meter reading
 */
export interface SmartMeterReading {
  meterId: string;
  timestamp: Timestamp;
  energy: {
    activeImport: number;            // kWh
    activeExport: number;            // kWh
    reactiveImport: number;          // kVArh
    reactiveExport: number;          // kVArh
  };
  power: {
    activePower: number;             // kW
    reactivePower: number;           // kVAr
    apparentPower: number;           // kVA
    powerFactor: number;             // 0-1
  };
  voltage: {
    phaseA: number;                  // V
    phaseB: number;                  // V
    phaseC: number;                  // V
    average: number;                 // V
  };
  current: {
    phaseA: number;                  // A
    phaseB: number;                  // A
    phaseC: number;                  // A
  };
  frequency: number;                 // Hz
  powerQuality: {
    thd: number;                     // % Total Harmonic Distortion
    voltageImbalance: number;        // %
    currentImbalance: number;        // %
  };
  temperature?: number;              // °C (meter temperature)
}

// ============================================================================
// Demand Response Types
// ============================================================================

/**
 * Demand response event types
 */
export enum DREventType {
  SCHEDULED = 'scheduled',
  EMERGENCY = 'emergency',
  ECONOMIC = 'economic',
  ANCILLARY_SERVICES = 'ancillary-services',
}

/**
 * Demand response signal levels
 */
export enum DRSignalLevel {
  NORMAL = 'normal',
  MODERATE = 'moderate',           // 5-10% reduction
  HIGH = 'high',                   // 10-20% reduction
  CRITICAL = 'critical',           // 20-30% reduction
  EMERGENCY = 'emergency',         // 30%+ reduction
}

/**
 * Demand response event
 */
export interface DemandResponseEvent {
  eventId: string;
  type: DREventType;
  signalLevel: DRSignalLevel;
  startTime: Timestamp;
  endTime: Timestamp;
  affectedZones: string[];           // zoneIds
  requestedReduction: {
    absolute: number;                // MW
    percentage: number;              // %
  };
  baselineLoad: number;              // MW
  incentive?: {
    type: 'payment' | 'credit' | 'rebate';
    amount: number;                  // currency per MW
    currency: string;
  };
  participation: 'mandatory' | 'voluntary';
  status: 'scheduled' | 'active' | 'completed' | 'cancelled';
}

/**
 * Demand response participant
 */
export interface DRParticipant {
  participantId: string;
  name: string;
  type: 'aggregator' | 'direct-customer' | 'utility';
  enrolledCapacity: number;          // MW
  availableCapacity: number;         // MW
  responseTime: number;              // minutes
  minimumDuration: number;           // minutes
  maximumEvents: {
    perDay: number;
    perMonth: number;
    perYear: number;
  };
  participationHistory: {
    eventsParticipated: number;
    totalReduction: number;          // MWh
    reliabilityScore: number;        // 0-100
  };
}

// ============================================================================
// Grid Stability and Frequency Types
// ============================================================================

/**
 * Grid frequency status
 */
export interface GridFrequencyStatus {
  zoneId: string;
  timestamp: Timestamp;
  frequency: number;                 // Hz
  nominalFrequency: number;          // Hz (50 or 60)
  deviation: number;                 // Hz
  rateOfChange: number;              // Hz/s (ROCOF)
  status: 'normal' | 'warning' | 'critical' | 'emergency';
  stabilityMargin: number;           // %
}

/**
 * Voltage stability metrics
 */
export interface VoltageStability {
  nodeId: string;
  timestamp: Timestamp;
  voltage: number;                   // kV
  nominalVoltage: number;            // kV
  deviation: number;                 // %
  status: 'normal' | 'undervoltage' | 'overvoltage' | 'critical';
  varSupport: {
    available: number;               // MVAr
    required: number;                // MVAr
    deficit: number;                 // MVAr
  };
}

/**
 * Grid inertia metrics
 */
export interface GridInertia {
  zoneId: string;
  timestamp: Timestamp;
  systemInertia: number;             // GW·s
  minimumRequired: number;           // GW·s
  margin: number;                    // %
  syntheticInertia: {
    available: number;               // GW·s
    sources: {
      batteries: number;             // GW·s
      windTurbines: number;          // GW·s
      hvdc: number;                  // GW·s
    };
  };
}

/**
 * Power system stabilizer
 */
export interface PowerSystemStabilizer {
  pssId: string;
  nodeId: string;
  type: 'conventional' | 'advanced' | 'adaptive';
  status: OperationalStatus;
  parameters: {
    gain: number;
    washoutTimeConstant: number;     // seconds
    leadTimeConstant: number;        // seconds
    lagTimeConstant: number;         // seconds
  };
  performance: {
    dampingRatio: number;            // 0-1
    oscillationFrequency: number;    // Hz
    settlingTime: number;            // seconds
  };
}

// ============================================================================
// Renewable Energy Integration Types
// ============================================================================

/**
 * Renewable forecast
 */
export interface RenewableForecast {
  sourceId: string;
  sourceType: GenerationType;
  timestamp: Timestamp;
  horizon: '15min' | '1hour' | '6hour' | '24hour' | '7day';
  predictions: {
    timestamp: Timestamp;
    expectedGeneration: number;      // MW
    confidence: number;              // %
    upperBound: number;              // MW
    lowerBound: number;              // MW
  }[];
  weatherData?: {
    solarIrradiance?: number[];      // W/m²
    windSpeed?: number[];            // m/s
    cloudCover?: number[];           // %
    temperature?: number[];          // °C
  };
}

/**
 * Curtailment event
 */
export interface CurtailmentEvent {
  eventId: string;
  sourceId: string;
  sourceType: GenerationType;
  timestamp: Timestamp;
  reason: 'grid-congestion' | 'overgeneration' | 'frequency-control' | 'voltage-control';
  curtailedPower: number;            // MW
  duration: number;                  // minutes
  energyLost: number;                // MWh
  compensation?: number;             // currency
}

/**
 * Grid integration metrics
 */
export interface GridIntegrationMetrics {
  zoneId: string;
  timestamp: Timestamp;
  renewablePenetration: number;      // % of total generation
  instantaneousRenewable: number;    // MW
  totalGeneration: number;           // MW
  variability: {
    ramping: number;                 // MW/min
    volatility: number;              // standard deviation
  };
  flexibility: {
    upwardReserve: number;           // MW
    downwardReserve: number;         // MW
    responseTime: number;            // seconds
  };
}

// ============================================================================
// Outage Management Types
// ============================================================================

/**
 * Outage types
 */
export enum OutageType {
  PLANNED = 'planned',
  UNPLANNED = 'unplanned',
  STORM = 'storm',
  EQUIPMENT_FAILURE = 'equipment-failure',
  OVERLOAD = 'overload',
  CYBERATTACK = 'cyberattack',
}

/**
 * Outage severity
 */
export enum OutageSeverity {
  MINOR = 'minor',                   // < 100 customers
  MODERATE = 'moderate',             // 100-1000 customers
  MAJOR = 'major',                   // 1000-10000 customers
  CRITICAL = 'critical',             // > 10000 customers
}

/**
 * Outage event
 */
export interface OutageEvent {
  outageId: string;
  type: OutageType;
  severity: OutageSeverity;
  startTime: Timestamp;
  estimatedRestorationTime?: Timestamp;
  actualRestorationTime?: Timestamp;
  affectedArea: {
    zoneId: string;
    nodes: string[];                 // nodeIds
    coordinates: Coordinates[];      // polygon boundary
  };
  customersAffected: {
    residential: number;
    commercial: number;
    industrial: number;
    total: number;
  };
  loadLost: number;                  // MW
  energyNotServed: number;           // MWh
  cause?: string;
  status: 'detected' | 'diagnosed' | 'dispatched' | 'restoring' | 'restored';
  crewsAssigned: string[];           // crew IDs
}

/**
 * Outage detection
 */
export interface OutageDetection {
  detectionId: string;
  timestamp: Timestamp;
  method: 'ami' | 'scada' | 'customer-call' | 'sensor' | 'patrol';
  location: Coordinates;
  affectedMeters: string[];          // meterIds
  predictedCause?: OutageType;
  confidence: number;                // %
  priority: 'low' | 'medium' | 'high' | 'critical';
}

/**
 * Restoration plan
 */
export interface RestorationPlan {
  planId: string;
  outageId: string;
  createdAt: Timestamp;
  priority: number;                  // 1-10
  steps: {
    stepNumber: number;
    action: string;
    nodeId?: string;
    estimatedDuration: number;       // minutes
    status: 'pending' | 'in-progress' | 'completed' | 'failed';
    assignedCrew?: string;
  }[];
  estimatedCustomersRestored: {
    byStep: number[];
  };
  safetyConsiderations: string[];
}

// ============================================================================
// Energy Storage Types
// ============================================================================

/**
 * Battery storage system
 */
export interface BatteryStorageSystem {
  storageId: string;
  name: string;
  location: Address;
  capacity: {
    energy: number;                  // MWh
    power: number;                   // MW
  };
  batteryType: 'lithium-ion' | 'flow-battery' | 'lead-acid' | 'sodium-sulfur';
  stateOfCharge: number;             // %
  efficiency: number;                // % round-trip
  cycleLife: number;                 // cycles
  currentCycles: number;
  operationalMode: 'charging' | 'discharging' | 'idle' | 'standby';
  services: ('energy-arbitrage' | 'frequency-regulation' | 'peak-shaving' | 'voltage-support')[];
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
    requestId?: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

/**
 * Date range filter
 */
export interface DateRangeFilter {
  startDate: Timestamp;
  endDate: Timestamp;
}

/**
 * Time series data point
 */
export interface TimeSeriesDataPoint {
  timestamp: Timestamp;
  value: number;
  quality?: 'good' | 'uncertain' | 'bad';
}

// ============================================================================
// Event Handling Types
// ============================================================================

/**
 * Grid event types
 */
export enum GridEventType {
  FREQUENCY_DEVIATION = 'frequency-deviation',
  VOLTAGE_DEVIATION = 'voltage-deviation',
  OVERLOAD = 'overload',
  OUTAGE = 'outage',
  GENERATION_CHANGE = 'generation-change',
  DEMAND_RESPONSE = 'demand-response',
  RENEWABLE_CURTAILMENT = 'renewable-curtailment',
}

/**
 * Grid event
 */
export interface GridEvent {
  eventId: string;
  type: GridEventType;
  timestamp: Timestamp;
  severity: 'info' | 'warning' | 'error' | 'critical';
  zoneId?: string;
  nodeId?: string;
  message: string;
  data?: any;
  acknowledged: boolean;
  acknowledgedBy?: string;
  acknowledgedAt?: Timestamp;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Address,
};
