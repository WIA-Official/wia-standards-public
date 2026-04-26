/**
 * WIA-CITY-011: Building Energy Management Standard - TypeScript Type Definitions
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
}

/**
 * Address information
 */
export interface Address {
  street: string;
  city: string;
  state: string;
  country: string;
  postalCode: string;
  coordinates: Coordinates;
  timezone: string;
  climateZone: string;
}

// ============================================================================
// Building Information
// ============================================================================

/**
 * Building types
 */
export enum BuildingType {
  COMMERCIAL_OFFICE = 'commercial-office',
  COMMERCIAL_RETAIL = 'commercial-retail',
  COMMERCIAL_HOTEL = 'commercial-hotel',
  COMMERCIAL_HOSPITAL = 'commercial-hospital',
  RESIDENTIAL_APARTMENT = 'residential-apartment',
  RESIDENTIAL_HOUSE = 'residential-house',
  INDUSTRIAL_FACTORY = 'industrial-factory',
  INDUSTRIAL_WAREHOUSE = 'industrial-warehouse',
  PUBLIC_GOVERNMENT = 'public-government',
  PUBLIC_SCHOOL = 'public-school',
  PUBLIC_LIBRARY = 'public-library',
  SPECIAL_DATACENTER = 'special-datacenter',
}

/**
 * Building specifications
 */
export interface BuildingSpecifications {
  totalFloorArea: number;         // m²
  grossFloorArea: number;         // m²
  floors: {
    aboveGround: number;
    underground: number;
  };
  occupancy: number;              // maximum occupants
  constructionYear: number;
  certifications: string[];       // LEED, BREEAM, G-SEED, etc.
}

/**
 * Energy systems in building
 */
export interface EnergySystems {
  hvac: HVACSystem;
  lighting: LightingSystem;
  renewable: RenewableEnergySystems;
  waterHeating: WaterHeatingSystem;
}

/**
 * HVAC system information
 */
export interface HVACSystem {
  heatingSource: 'gas-boiler' | 'electric-boiler' | 'heat-pump' | 'district-heating' | 'geothermal';
  coolingSource: 'electric-chiller' | 'absorption-chiller' | 'heat-pump' | 'district-cooling' | 'geothermal';
  ventilationType: 'natural' | 'mechanical' | 'mechanical-with-hrv' | 'mechanical-with-erv';
  controlType: 'manual' | 'thermostat' | 'zone-control' | 'bems';
}

/**
 * Lighting system information
 */
export interface LightingSystem {
  type: 'LED' | 'fluorescent' | 'incandescent' | 'halogen';
  controlSystem: 'manual' | 'occupancy-sensor' | 'daylight-harvesting' | 'intelligent';
  dimmingCapable: boolean;
}

/**
 * Renewable energy systems
 */
export interface RenewableEnergySystems {
  solar?: SolarPVSystem;
  ess?: EnergyStorageSystem;
  geothermal?: GeothermalSystem;
  wind?: WindSystem;
}

/**
 * Water heating system
 */
export interface WaterHeatingSystem {
  type: 'gas' | 'electric' | 'heat-pump' | 'solar-thermal' | 'district';
  capacity: number;               // L
  efficiency: number;             // %
}

/**
 * Building information
 */
export interface BuildingInfo {
  buildingId: string;
  name: string;
  type: BuildingType;
  location: Address;
  specifications: BuildingSpecifications;
  energySystems: EnergySystems;
}

// ============================================================================
// Energy Metering
// ============================================================================

/**
 * Meter types
 */
export enum MeterType {
  ELECTRICITY = 'electricity',
  GAS = 'gas',
  WATER = 'water',
  DISTRICT_HEATING = 'district-heating',
  DISTRICT_COOLING = 'district-cooling',
  CHILLED_WATER = 'chilled-water',
  HOT_WATER = 'hot-water',
}

/**
 * Electrical readings
 */
export interface ElectricalReadings {
  activePower: number;            // kW
  reactivePower: number;          // kvar
  apparentPower: number;          // kVA
  powerFactor: number;            // 0-1
  voltage: {
    phaseA: number;               // V
    phaseB: number;               // V
    phaseC: number;               // V
  };
  current: {
    phaseA: number;               // A
    phaseB: number;               // A
    phaseC: number;               // A
  };
  frequency: number;              // Hz
  energy: {
    cumulative: number;           // kWh (total)
    interval: number;             // kWh (interval)
  };
}

/**
 * Energy breakdown by end-use
 */
export interface EnergyBreakdown {
  hvac: number;                   // kW
  lighting: number;               // kW
  plugLoad: number;               // kW
  elevators: number;              // kW
  waterHeating: number;           // kW
  others: number;                 // kW
}

/**
 * Data quality indicators
 */
export interface DataQuality {
  accuracy: number;               // % (meter accuracy class)
  dataValid: boolean;
  estimatedData: boolean;
  lastCalibration?: Timestamp;
}

/**
 * Energy meter reading
 */
export interface EnergyMeterReading {
  meterId: string;
  buildingId: string;
  meterType: MeterType;
  timestamp: Timestamp;
  interval: '1min' | '5min' | '15min' | '30min' | '60min';
  readings: ElectricalReadings | ThermalReadings | FlowReadings;
  breakdown?: EnergyBreakdown;
  quality: DataQuality;
}

/**
 * Thermal readings (heating/cooling)
 */
export interface ThermalReadings {
  supplyTemperature: number;      // °C
  returnTemperature: number;      // °C
  flowRate: number;               // m³/h
  thermalPower: number;           // kW
  thermalEnergy: {
    cumulative: number;           // kWh
    interval: number;             // kWh
  };
}

/**
 * Flow readings (gas, water)
 */
export interface FlowReadings {
  instantFlow: number;            // m³/h
  volume: {
    cumulative: number;           // m³
    interval: number;             // m³
  };
  pressure?: number;              // kPa
  temperature?: number;           // °C
}

// ============================================================================
// BEMS Components
// ============================================================================

/**
 * System operational status
 */
export enum OperationalStatus {
  RUNNING = 'running',
  STOPPED = 'stopped',
  STANDBY = 'standby',
  FAULT = 'fault',
  MAINTENANCE = 'maintenance',
}

/**
 * Control mode
 */
export enum ControlMode {
  AUTO = 'auto',
  MANUAL = 'manual',
  SCHEDULE = 'schedule',
  OPTIMIZED = 'optimized',
  EMERGENCY = 'emergency',
}

/**
 * HVAC unit types
 */
export enum HVACUnitType {
  AIR_HANDLING_UNIT = 'air-handling-unit',
  FAN_COIL_UNIT = 'fan-coil-unit',
  VAV_BOX = 'vav-box',
  CHILLER = 'chiller',
  BOILER = 'boiler',
  HEAT_PUMP = 'heat-pump',
  COOLING_TOWER = 'cooling-tower',
}

/**
 * Airflow data
 */
export interface AirflowData {
  supplyAirflow: number;          // m³/h
  returnAirflow: number;          // m³/h
  outdoorAirflow: number;         // m³/h
  exhaustAirflow: number;         // m³/h
}

/**
 * Temperature data
 */
export interface TemperatureData {
  supplyAir: number;              // °C
  returnAir: number;              // °C
  outdoorAir: number;             // °C
  mixedAir: number;               // °C
  setpoint?: number;              // °C
}

/**
 * Humidity data
 */
export interface HumidityData {
  supplyAir: number;              // %
  returnAir: number;              // %
  outdoorAir: number;             // %
  setpoint?: number;              // %
}

/**
 * Pressure data
 */
export interface PressureData {
  staticPressure: number;         // Pa
  differentialPressure: number;   // Pa
}

/**
 * Fan component
 */
export interface FanComponent {
  status: OperationalStatus;
  speed: number;                  // % (0-100)
  power: number;                  // kW
  vfd: boolean;                   // Variable Frequency Drive
}

/**
 * Coil component
 */
export interface CoilComponent {
  status: OperationalStatus;
  valvePosition: number;          // % (0-100)
  output: number;                 // kW
  setpoint?: number;              // °C
}

/**
 * Filter component
 */
export interface FilterComponent {
  status: 'normal' | 'warning' | 'critical';
  pressureDrop: number;           // Pa
  replacementDue: Timestamp;
  efficiency: number;             // % (MERV rating)
}

/**
 * HVAC components
 */
export interface HVACComponents {
  fan?: FanComponent;
  heatingCoil?: CoilComponent;
  coolingCoil?: CoilComponent;
  filter?: FilterComponent;
  dampers?: {
    outdoor: number;              // % open
    return: number;               // % open
    exhaust: number;              // % open
  };
}

/**
 * Energy consumption
 */
export interface EnergyConsumption {
  current: number;                // kW
  daily: number;                  // kWh
  monthly: number;                // kWh
  annual?: number;                // kWh
}

/**
 * HVAC operational data
 */
export interface HVACOperationalData {
  systemId: string;
  buildingId: string;
  systemType: HVACUnitType;
  timestamp: Timestamp;
  operationalStatus: OperationalStatus;
  controlMode: ControlMode;
  airflow?: AirflowData;
  temperature: TemperatureData;
  humidity?: HumidityData;
  pressure?: PressureData;
  components: HVACComponents;
  energyConsumption: EnergyConsumption;
}

// ============================================================================
// Load Management
// ============================================================================

/**
 * Load priority levels
 */
export enum LoadPriority {
  CRITICAL = 'critical',          // Must never be shed
  HIGH = 'high',                  // Shed only in emergency
  MEDIUM = 'medium',              // Can be shed during peak
  LOW = 'low',                    // First to be shed
  FLEXIBLE = 'flexible',          // Can be shifted in time
}

/**
 * Load information
 */
export interface LoadInfo {
  loadId: string;
  name: string;
  type: 'hvac' | 'lighting' | 'plug-load' | 'elevator' | 'pump' | 'other';
  priority: LoadPriority;
  ratedPower: number;             // kW
  controllable: boolean;
  sheddable: boolean;
  shiftable: boolean;
}

/**
 * Peak shaving configuration
 */
export interface PeakShavingConfig {
  enabled: boolean;
  targetPeakDemand: number;       // kW
  peakThreshold: number;          // kW (trigger level)
  strategy: 'ess-discharge' | 'load-shedding' | 'setpoint-adjustment' | 'combined';
  loadSheddingSequence: string[]; // loadIds in order
  hvacAdjustment: {
    coolingOffset: number;        // °C (increase setpoint)
    heatingOffset: number;        // °C (decrease setpoint)
  };
  lightingReduction: number;      // % (dim to this level)
}

/**
 * Peak shaving event
 */
export interface PeakShavingEvent {
  eventId: string;
  buildingId: string;
  timestamp: Timestamp;
  trigger: {
    currentDemand: number;        // kW
    threshold: number;            // kW
  };
  actions: {
    essDischargePower?: number;   // kW
    loadsShed: string[];          // loadIds
    hvacAdjusted: boolean;
    lightingDimmed: boolean;
  };
  result: {
    peakReduction: number;        // kW
    targetAchieved: boolean;
  };
}

// ============================================================================
// Demand Response
// ============================================================================

/**
 * Demand response signal
 */
export enum DRSignal {
  NORMAL = 'normal',
  MODERATE = 'moderate',          // 5-10% reduction
  HIGH = 'high',                  // 10-20% reduction
  CRITICAL = 'critical',          // 20-30% reduction
}

/**
 * Demand response event
 */
export interface DemandResponseEvent {
  eventId: string;
  buildingId: string;
  startTime: Timestamp;
  endTime: Timestamp;
  signal: DRSignal;
  requestedReduction: number;     // kW or %
  baselineLoad: number;           // kW
  actualReduction?: number;       // kW
  compensation?: number;          // currency
  participation: 'mandatory' | 'voluntary';
}

/**
 * DR strategy
 */
export interface DRStrategy {
  signal: DRSignal;
  actions: {
    hvacSetpointAdjustment: number;     // °C
    lightingReduction: number;          // %
    essDischargePower: number;          // kW
    nonCriticalLoadsShutdown: boolean;
  };
}

// ============================================================================
// Renewable Energy Integration
// ============================================================================

/**
 * Solar PV system
 */
export interface SolarPVSystem {
  pvSystemId: string;
  capacity: number;               // kWp
  installationType: 'rooftop' | 'ground-mount' | 'facade' | 'carport';
  panelCount: number;
  panelWattage: number;           // Wp
  inverterType: 'string-inverter' | 'micro-inverter' | 'central-inverter';
  inverterCount: number;
  inverterCapacity: number;       // kW
  azimuth: number;                // degrees (0-360)
  tilt: number;                   // degrees (0-90)
}

/**
 * Solar PV real-time data
 */
export interface SolarPVData {
  pvSystemId: string;
  buildingId: string;
  timestamp: Timestamp;
  generation: {
    current: number;              // kW
    daily: number;                // kWh
    monthly: number;              // kWh
    annual: number;               // kWh
    lifetime: number;             // kWh
  };
  irradiance: number;             // W/m²
  moduleTemperature: number;      // °C
  ambientTemperature: number;     // °C
  inverterEfficiency: number;     // %
  performanceRatio: number;       // % (PR)
  status: OperationalStatus;
}

/**
 * Energy Storage System (ESS)
 */
export interface EnergyStorageSystem {
  essId: string;
  capacity: number;               // kWh
  power: number;                  // kW (max charge/discharge)
  batteryType: 'lithium-ion' | 'lead-acid' | 'flow-battery' | 'sodium-sulfur';
  cycleLife: number;              // cycles
  efficiency: number;             // % (round-trip)
  dod: number;                    // % (depth of discharge)
}

/**
 * ESS operational mode
 */
export enum ESSMode {
  IDLE = 'idle',
  CHARGING = 'charging',
  DISCHARGING = 'discharging',
  STANDBY = 'standby',
  FAULT = 'fault',
}

/**
 * ESS real-time data
 */
export interface ESSData {
  essId: string;
  buildingId: string;
  timestamp: Timestamp;
  mode: ESSMode;
  stateOfCharge: number;          // % (0-100)
  power: number;                  // kW (positive = charging, negative = discharging)
  voltage: number;                // V
  current: number;                // A
  temperature: number;            // °C
  cycleCount: number;
  energyThroughput: {
    charged: number;              // kWh (lifetime)
    discharged: number;           // kWh (lifetime)
  };
  health: {
    stateOfHealth: number;        // % (capacity retention)
    remainingCycleLife: number;
    warningFlags: string[];
  };
}

/**
 * Geothermal system
 */
export interface GeothermalSystem {
  ghpId: string;
  capacity: number;               // kW
  heatPumpType: 'water-to-water' | 'water-to-air' | 'ground-to-water';
  groundLoopType: 'vertical-borehole' | 'horizontal-trench' | 'pond' | 'well';
  boreholeCount?: number;
  boreholeDepth?: number;         // m
  copHeating: number;             // COP for heating
  copCooling: number;             // COP for cooling
}

/**
 * Geothermal operational data
 */
export interface GeothermalData {
  ghpId: string;
  buildingId: string;
  timestamp: Timestamp;
  mode: 'heating' | 'cooling' | 'off';
  temperatures: {
    enteringWater: number;        // °C
    leavingWater: number;         // °C
    groundLoop: number;           // °C
  };
  flowRate: number;               // m³/h
  thermalPower: number;           // kW
  electricalPower: number;        // kW
  cop: number;                    // actual COP
  energyConsumption: EnergyConsumption;
}

// ============================================================================
// Carbon Footprint
// ============================================================================

/**
 * Emission factors
 */
export interface EmissionFactors {
  electricity: number;            // kg CO₂/kWh
  gas: number;                    // kg CO₂/m³
  districtHeating: number;        // kg CO₂/MJ
  districtCooling: number;        // kg CO₂/MJ
}

/**
 * Carbon emissions
 */
export interface CarbonEmissions {
  electricity: number;            // kg CO₂
  gas: number;                    // kg CO₂
  districtHeating: number;        // kg CO₂
  districtCooling: number;        // kg CO₂
  total: number;                  // kg CO₂
}

/**
 * Carbon offset by renewable
 */
export interface CarbonOffset {
  solar: number;                  // kg CO₂
  geothermal: number;             // kg CO₂
  total: number;                  // kg CO₂
}

/**
 * Carbon intensity metrics
 */
export interface CarbonIntensity {
  perSquareMeter: number;         // kg CO₂/m²
  perOccupant: number;            // kg CO₂/person
  perEnergyUnit: number;          // kg CO₂/kWh
}

/**
 * Carbon footprint report
 */
export interface CarbonFootprintReport {
  reportId: string;
  buildingId: string;
  reportDate: Timestamp;
  period: 'daily' | 'weekly' | 'monthly' | 'annual';
  emissions: CarbonEmissions;
  offsetByRenewable: CarbonOffset;
  netEmissions: number;           // kg CO₂
  intensity: CarbonIntensity;
  target?: {
    targetEmissions: number;      // kg CO₂
    achievement: number;          // %
  };
  trends: {
    previousPeriod: number;       // kg CO₂
    percentageChange: number;     // %
  };
}

// ============================================================================
// Energy Benchmarking
// ============================================================================

/**
 * Energy performance metrics
 */
export interface EnergyPerformanceMetrics {
  eui: number;                    // kWh/m²/year (Energy Use Intensity)
  peakDemand: number;             // kW
  loadFactor: number;             // (0-1)
  carbonIntensity: number;        // kg CO₂/m²/year
  renewablePercentage: number;    // %
}

/**
 * Benchmarking comparison
 */
export interface BenchmarkingComparison {
  buildingType: BuildingType;
  floorArea: number;              // m²
  yourEUI: number;                // kWh/m²/year
  benchmarkEUI: {
    median: number;               // kWh/m²/year
    top25: number;                // kWh/m²/year (best performers)
    bottom25: number;             // kWh/m²/year (worst performers)
  };
  percentile: number;             // 0-100
  rank: 'Excellent' | 'Good' | 'Average' | 'Below Average' | 'Poor';
  potentialSavings: {
    energy: number;               // kWh/year
    cost: number;                 // currency
    carbon: number;               // kg CO₂/year
  };
}

/**
 * Benchmarking recommendations
 */
export interface BenchmarkingRecommendation {
  category: 'hvac' | 'lighting' | 'envelope' | 'renewable' | 'controls';
  priority: 'high' | 'medium' | 'low';
  title: string;
  description: string;
  estimatedSavings: number;       // % or kWh/year
  implementationCost: number;     // currency
  paybackPeriod: number;          // years
}

// ============================================================================
// Green Building Certifications
// ============================================================================

/**
 * LEED certification levels
 */
export enum LEEDLevel {
  CERTIFIED = 'certified',
  SILVER = 'silver',
  GOLD = 'gold',
  PLATINUM = 'platinum',
}

/**
 * BREEAM certification levels
 */
export enum BREEAMLevel {
  PASS = 'pass',
  GOOD = 'good',
  VERY_GOOD = 'very-good',
  EXCELLENT = 'excellent',
  OUTSTANDING = 'outstanding',
}

/**
 * LEED energy performance data
 */
export interface LEEDEnergyData {
  proposedEUI: number;            // kWh/m²/year
  baselineEUI: number;            // kWh/m²/year
  percentImprovement: number;     // %
  pointsEarned: number;           // 0-18 points
  meteringPoints: number;         // 0-1 points
  renewablePoints: number;        // 0-3 points
  certificationLevel: LEEDLevel;
}

/**
 * BREEAM energy data
 */
export interface BREEAMEnergyData {
  energyEfficiencyRating: number; // %
  energyMonitoringCompliance: boolean;
  lowCarbonDesign: boolean;
  totalScore: number;             // %
  certificationLevel: BREEAMLevel;
}

/**
 * Certification status
 */
export interface CertificationStatus {
  certificationId: string;
  buildingId: string;
  certificationType: 'LEED' | 'BREEAM' | 'G-SEED' | 'WELL' | 'Living Building Challenge';
  version: string;
  status: 'registered' | 'in-progress' | 'certified' | 'expired';
  certificationDate?: Timestamp;
  expiryDate?: Timestamp;
  level?: LEEDLevel | BREEAMLevel | string;
  score?: number;
  data: LEEDEnergyData | BREEAMEnergyData | any;
}

// ============================================================================
// Monitoring & Analytics
// ============================================================================

/**
 * Real-time dashboard
 */
export interface RealtimeDashboard {
  buildingId: string;
  timestamp: Timestamp;
  energy: {
    currentDemand: number;        // kW
    todayConsumption: number;     // kWh
    monthToDate: number;          // kWh
    yearToDate: number;           // kWh
  };
  cost: {
    todayCost: number;            // currency
    monthToDate: number;          // currency
    yearToDate: number;           // currency
  };
  carbon: {
    todayEmissions: number;       // kg CO₂
    monthToDate: number;          // kg CO₂
    yearToDate: number;           // kg CO₂
  };
  renewable: {
    solarGeneration: number;      // kW
    essChargePower: number;       // kW (positive = charging)
    selfConsumption: number;      // %
  };
  comfort: {
    averageTemperature: number;   // °C
    averageHumidity: number;      // %
    co2Level: number;             // ppm
  };
  alerts: AlertInfo[];
}

/**
 * Alert types
 */
export enum AlertType {
  HIGH_ENERGY = 'high-energy',
  EQUIPMENT_FAULT = 'equipment-fault',
  SENSOR_ANOMALY = 'sensor-anomaly',
  TARGET_EXCEEDED = 'target-exceeded',
  MAINTENANCE_DUE = 'maintenance-due',
  POOR_IAQ = 'poor-indoor-air-quality',
}

/**
 * Alert severity
 */
export enum AlertSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical',
}

/**
 * Alert information
 */
export interface AlertInfo {
  alertId: string;
  buildingId: string;
  timestamp: Timestamp;
  type: AlertType;
  severity: AlertSeverity;
  systemId?: string;
  message: string;
  details?: any;
  acknowledged: boolean;
  resolvedAt?: Timestamp;
}

/**
 * Energy forecast
 */
export interface EnergyForecast {
  buildingId: string;
  forecastDate: Timestamp;
  horizon: '1day' | '7day' | '30day';
  predictions: {
    timestamp: Timestamp;
    predictedLoad: number;        // kW
    confidence: number;           // %
    lower95: number;              // kW (95% confidence lower bound)
    upper95: number;              // kW (95% confidence upper bound)
  }[];
  variables: {
    weather: {
      temperature: number[];      // °C
      humidity: number[];         // %
      solarIrradiance: number[];  // W/m²
    };
    occupancy: number[];          // persons
  };
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
 * Aggregation types
 */
export type AggregationType = '15min' | '1hour' | 'daily' | 'weekly' | 'monthly' | 'annual';

// ============================================================================
// Control Commands
// ============================================================================

/**
 * HVAC setpoint command
 */
export interface HVACSetpointCommand {
  buildingId: string;
  systemId: string;
  zoneId?: string;
  mode: 'heating' | 'cooling' | 'auto';
  setpoint: number;               // °C
  duration?: number;              // seconds (0 = permanent)
}

/**
 * Lighting control command
 */
export interface LightingControlCommand {
  buildingId: string;
  lightingGroupId: string;
  action: 'on' | 'off' | 'dim';
  dimLevel?: number;              // % (0-100)
  schedule?: {
    startTime: string;            // HH:MM
    endTime: string;              // HH:MM
  };
}

/**
 * ESS control command
 */
export interface ESSControlCommand {
  buildingId: string;
  essId: string;
  mode: 'charge' | 'discharge' | 'idle';
  power?: number;                 // kW
  targetSOC?: number;             // %
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  Coordinates,
  Address,
};
