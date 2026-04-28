/**
 * Zero Energy Building City Standard - TypeScript Type Definitions
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
 * District identifier
 */
export type DistrictId = string;

/**
 * Building identifier
 */
export type BuildingId = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  lat: number;
  lon: number;
  elevation_m?: number;
}

/**
 * Location information
 */
export interface Location {
  lat: number;
  lon: number;
  address: string;
  city: string;
  state?: string;
  country: string;
  postalCode?: string;
}

// ============================================================================
// District Types
// ============================================================================

/**
 * Energy district classification
 */
export enum DistrictClass {
  RESIDENTIAL = 'RESIDENTIAL',           // 주거 지구
  COMMERCIAL = 'COMMERCIAL',             // 상업 지구
  INDUSTRIAL = 'INDUSTRIAL',             // 산업 지구
  MIXED_USE = 'MIXED_USE',               // 복합 용도
  CAMPUS = 'CAMPUS',                     // 캠퍼스
  GOVERNMENT = 'GOVERNMENT',             // 정부 시설
}

/**
 * Energy district
 */
export interface EnergyDistrict {
  districtId: DistrictId;
  name: string;
  class: DistrictClass;
  location: Location;

  // Area & population
  area_km2: number;
  population?: number;
  buildings_count: number;

  // Energy profile
  energy_profile: DistrictEnergyProfile;

  // Grid connection
  grid_connection: GridConnection;

  // Renewable sources
  renewable_sources: RenewableSource[];

  // Storage systems
  storage_systems: StorageSystem[];

  // Smart city integration
  smart_meters: number;
  data_hub: DataHub;

  // Status
  status: 'ACTIVE' | 'PLANNING' | 'CONSTRUCTION' | 'OFFLINE';
  certified: boolean;
  certification?: Certification;

  created_at: Timestamp;
  updated_at: Timestamp;
}

/**
 * Building cluster within a district
 */
export interface BuildingCluster {
  clusterId: string;
  districtId: DistrictId;
  name: string;

  buildings: ZEBBuilding[];

  // Shared infrastructure
  shared_microgrid?: Microgrid;
  shared_storage?: StorageSystem;

  // Aggregate metrics
  total_energy_demand_kWh: number;
  total_energy_production_kWh: number;
  net_energy_balance_kWh: number;
}

/**
 * District energy profile
 */
export interface DistrictEnergyProfile {
  // Annual energy (kWh/year)
  annual_demand_kWh: number;
  annual_production_kWh: number;
  annual_net_balance_kWh: number;

  // Peak demand (kW)
  peak_demand_kW: number;
  peak_production_kW: number;

  // Energy sources breakdown
  sources: {
    solar_percent: number;
    wind_percent: number;
    geothermal_percent: number;
    grid_import_percent: number;
    other_percent: number;
  };

  // Load profile (24-hour average)
  load_profile_kW: number[];

  // Production profile (24-hour average)
  production_profile_kW: number[];
}

// ============================================================================
// Energy Grid
// ============================================================================

/**
 * City-wide energy grid
 */
export interface CityGrid {
  gridId: string;
  city: string;
  operator: string;

  // Capacity
  total_capacity_MW: number;
  available_capacity_MW: number;

  // Districts connected
  districts: DistrictId[];

  // Real-time metrics
  current_load_MW: number;
  current_production_MW: number;
  frequency_Hz: number;
  voltage_kV: number;

  // Grid status
  status: 'STABLE' | 'STRESSED' | 'CRITICAL' | 'BLACKOUT';
  reliability_score: number; // 0-100

  last_update: Timestamp;
}

/**
 * Microgrid system
 */
export interface Microgrid {
  microgridId: string;
  districtId?: DistrictId;
  name: string;

  // Configuration
  islanding_capable: boolean; // Can operate independently
  grid_connected: boolean;

  // Capacity
  capacity_kW: number;

  // Connected assets
  renewable_sources: string[]; // IDs
  storage_systems: string[]; // IDs
  loads: string[]; // Building IDs

  // Real-time status
  current_load_kW: number;
  current_production_kW: number;
  current_storage_kWh: number;

  // Operation mode
  mode: 'GRID_CONNECTED' | 'ISLANDED' | 'STANDBY' | 'TRANSITION';

  status: 'OPERATIONAL' | 'MAINTENANCE' | 'FAULT';
}

/**
 * Grid connection point
 */
export interface GridConnection {
  connectionId: string;
  districtId: DistrictId;

  // Connection details
  connection_point: string;
  voltage_level_kV: number;
  max_import_kW: number;
  max_export_kW: number;

  // Metering
  import_meter_id: string;
  export_meter_id: string;

  // Current flow
  current_flow_kW: number; // Positive = import, negative = export
  power_factor: number;

  // Billing
  import_rate_USD_per_kWh: number;
  export_rate_USD_per_kWh: number;

  status: 'CONNECTED' | 'DISCONNECTED' | 'FAULT';
}

/**
 * Energy flow measurement
 */
export interface EnergyFlow {
  timestamp: Timestamp;
  source: string;
  destination: string;

  // Power flow
  active_power_kW: number;
  reactive_power_kVAR: number;
  apparent_power_kVA: number;

  // Energy (cumulative)
  energy_kWh: number;

  // Quality
  voltage_V: number;
  current_A: number;
  frequency_Hz: number;
  power_factor: number;
}

// ============================================================================
// Building Types
// ============================================================================

/**
 * Building energy class
 */
export enum BuildingClass {
  ZEB = 'ZEB',                           // Zero Energy Building
  NZEB = 'NZEB',                         // Nearly Zero Energy Building
  PLUS_ENERGY = 'PLUS_ENERGY',           // Plus Energy Building (produces more than consumes)
  PASSIVE = 'PASSIVE',                   // Passive House
  CONVENTIONAL = 'CONVENTIONAL',         // Conventional building
}

/**
 * Zero Energy Building
 */
export interface ZEBBuilding {
  buildingId: BuildingId;
  name: string;
  districtId?: DistrictId;
  location: Location;

  // Classification
  class: BuildingClass;
  building_type: 'RESIDENTIAL' | 'COMMERCIAL' | 'INDUSTRIAL' | 'INSTITUTIONAL';

  // Physical characteristics
  floor_area_m2: number;
  floors: number;
  occupancy?: number;
  year_built: number;

  // Energy profile
  energy_profile: BuildingEnergyProfile;

  // Renewable systems
  solar_pv_kW?: number;
  solar_thermal_kW?: number;
  geothermal_kW?: number;

  // Storage
  battery_storage_kWh?: number;
  thermal_storage_kWh?: number;

  // Smart systems
  smart_meter?: SmartMeter;
  bms_connected: boolean; // Building Management System

  // Certification
  certification?: Certification;

  status: 'OPERATIONAL' | 'CONSTRUCTION' | 'PLANNING' | 'DECOMMISSIONED';
}

/**
 * Building energy profile
 */
export interface BuildingEnergyProfile {
  // Annual metrics (kWh/year)
  annual_consumption_kWh: number;
  annual_production_kWh: number;
  annual_net_balance_kWh: number;

  // Normalized metrics (kWh/m²/year)
  eui_kWh_m2: number; // Energy Use Intensity
  epi_kWh_m2: number; // Energy Production Intensity

  // Breakdown
  consumption_breakdown: {
    heating_percent: number;
    cooling_percent: number;
    hot_water_percent: number;
    lighting_percent: number;
    appliances_percent: number;
    other_percent: number;
  };

  // Performance
  zero_energy_balance_achieved: boolean;
  self_sufficiency_percent: number;
}

// ============================================================================
// Renewable Energy Sources
// ============================================================================

/**
 * Renewable energy source type
 */
export enum RenewableSourceType {
  SOLAR_PV = 'SOLAR_PV',
  SOLAR_THERMAL = 'SOLAR_THERMAL',
  WIND = 'WIND',
  GEOTHERMAL = 'GEOTHERMAL',
  BIOMASS = 'BIOMASS',
  HYDROELECTRIC = 'HYDROELECTRIC',
}

/**
 * Base renewable source
 */
export interface RenewableSource {
  sourceId: string;
  type: RenewableSourceType;
  name: string;
  location: Coordinates;

  // Capacity
  rated_capacity_kW: number;
  current_output_kW: number;

  // Performance
  capacity_factor: number; // 0-1
  annual_production_kWh: number;

  // Status
  status: 'OPERATIONAL' | 'MAINTENANCE' | 'FAULT' | 'OFFLINE';
  commission_date: Timestamp;

  last_update: Timestamp;
}

/**
 * Solar photovoltaic farm
 */
export interface SolarFarm extends RenewableSource {
  type: RenewableSourceType.SOLAR_PV;

  // Array specs
  panel_count: number;
  panel_wattage_W: number;
  array_area_m2: number;

  // Performance
  current_irradiance_W_m2: number;
  current_efficiency_percent: number;
  panel_temperature_C: number;

  // Orientation
  tilt_degrees: number;
  azimuth_degrees: number;

  // Tracking
  tracking_system?: 'FIXED' | 'SINGLE_AXIS' | 'DUAL_AXIS';
}

/**
 * Wind turbine installation
 */
export interface WindTurbine extends RenewableSource {
  type: RenewableSourceType.WIND;

  // Turbine specs
  turbine_count: number;
  hub_height_m: number;
  rotor_diameter_m: number;

  // Wind conditions
  current_wind_speed_m_s: number;
  current_wind_direction_deg: number;

  // Performance
  cut_in_speed_m_s: number;
  rated_speed_m_s: number;
  cut_out_speed_m_s: number;

  // Status
  rotor_rpm: number;
  nacelle_temperature_C: number;
}

/**
 * Geothermal plant
 */
export interface GeothermalPlant extends RenewableSource {
  type: RenewableSourceType.GEOTHERMAL;

  // System type
  system_type: 'HEAT_PUMP' | 'DIRECT_USE' | 'POWER_GENERATION';

  // Wells
  well_count: number;
  well_depth_m: number;

  // Thermal properties
  ground_temperature_C: number;
  heat_extraction_rate_kW: number;

  // Heat pump (if applicable)
  cop?: number; // Coefficient of Performance
  eer?: number; // Energy Efficiency Ratio

  // Status
  flow_rate_L_s: number;
  supply_temperature_C: number;
  return_temperature_C: number;
}

// ============================================================================
// Energy Storage
// ============================================================================

/**
 * Storage system type
 */
export enum StorageType {
  BATTERY = 'BATTERY',
  THERMAL = 'THERMAL',
  PUMPED_HYDRO = 'PUMPED_HYDRO',
  FLYWHEEL = 'FLYWHEEL',
  COMPRESSED_AIR = 'COMPRESSED_AIR',
}

/**
 * Base storage system
 */
export interface StorageSystem {
  storageId: string;
  type: StorageType;
  name: string;
  location: Coordinates;

  // Capacity
  energy_capacity_kWh: number;
  power_capacity_kW: number;

  // State
  current_charge_kWh: number;
  state_of_charge_percent: number;

  // Performance
  round_trip_efficiency_percent: number;
  cycle_count: number;
  max_cycles: number;

  // Status
  status: 'CHARGING' | 'DISCHARGING' | 'IDLE' | 'MAINTENANCE' | 'FAULT';
  health_percent: number;

  commission_date: Timestamp;
  last_update: Timestamp;
}

/**
 * Battery storage bank
 */
export interface BatteryBank extends StorageSystem {
  type: StorageType.BATTERY;

  // Battery specs
  chemistry: 'LITHIUM_ION' | 'LITHIUM_IRON_PHOSPHATE' | 'LEAD_ACID' | 'FLOW_BATTERY' | 'SODIUM_ION';
  module_count: number;
  nominal_voltage_V: number;

  // Current state
  current_voltage_V: number;
  current_current_A: number;
  current_power_kW: number;

  // Temperature
  temperature_C: number;
  max_temperature_C: number;

  // Safety
  bms_status: 'NORMAL' | 'WARNING' | 'ALARM'; // Battery Management System
  cell_balancing_active: boolean;
}

/**
 * Thermal energy storage
 */
export interface ThermalStorage extends StorageSystem {
  type: StorageType.THERMAL;

  // Storage medium
  medium: 'WATER' | 'PHASE_CHANGE_MATERIAL' | 'MOLTEN_SALT' | 'ROCK';
  volume_m3: number;

  // Temperature
  current_temperature_C: number;
  min_temperature_C: number;
  max_temperature_C: number;

  // Flow
  flow_rate_L_s: number;
  inlet_temperature_C: number;
  outlet_temperature_C: number;

  // Insulation
  heat_loss_rate_kW: number;
}

/**
 * City-level storage aggregation
 */
export interface CityStorage {
  districtId: DistrictId;

  // Aggregate capacity
  total_battery_capacity_kWh: number;
  total_thermal_capacity_kWh: number;
  total_capacity_kWh: number;

  // Current state
  total_charge_kWh: number;
  average_soc_percent: number;

  // Storage systems
  battery_banks: BatteryBank[];
  thermal_storage: ThermalStorage[];

  // Performance
  total_charging_power_kW: number;
  total_discharging_power_kW: number;

  last_update: Timestamp;
}

// ============================================================================
// Smart City Integration
// ============================================================================

/**
 * Smart meter device
 */
export interface SmartMeter {
  meterId: string;
  buildingId?: BuildingId;
  location: string;

  // Meter type
  type: 'ELECTRICITY' | 'GAS' | 'WATER' | 'HEAT';

  // Real-time readings
  current_power_kW?: number;
  current_flow_rate?: number;

  // Cumulative readings
  total_consumption_kWh: number;
  total_production_kWh?: number; // For bidirectional meters

  // Communication
  communication_protocol: 'ZIGBEE' | 'LORA' | 'NB_IOT' | 'WIFI' | 'ETHERNET';
  last_reading: Timestamp;
  reading_interval_min: number;

  // Data
  daily_data_points: number;
  data_retention_days: number;

  status: 'ONLINE' | 'OFFLINE' | 'FAULT';
}

/**
 * District data hub
 */
export interface DataHub {
  hubId: string;
  districtId: DistrictId;

  // Connected devices
  smart_meters_count: number;
  sensors_count: number;
  controllers_count: number;

  // Data flow
  messages_per_hour: number;
  data_volume_MB_per_day: number;

  // Processing
  analytics_enabled: boolean;
  ml_models_deployed: string[];

  // Storage
  database_size_GB: number;
  retention_period_days: number;

  // API access
  api_endpoint: string;
  api_version: string;

  status: 'OPERATIONAL' | 'DEGRADED' | 'OFFLINE';
  last_update: Timestamp;
}

/**
 * Central control center
 */
export interface ControlCenter {
  centerId: string;
  city: string;
  operator: string;

  // Monitored districts
  districts: DistrictId[];

  // Capabilities
  automated_control: boolean;
  predictive_optimization: boolean;
  demand_response_enabled: boolean;

  // Operations
  active_operators: number;
  active_alarms: number;
  pending_commands: number;

  // Performance
  uptime_percent: number;
  response_time_ms: number;

  status: 'OPERATIONAL' | 'BACKUP_MODE' | 'OFFLINE';
}

// ============================================================================
// Sustainability Metrics
// ============================================================================

/**
 * Carbon footprint tracking
 */
export interface CarbonFootprint {
  entityId: string; // District or building ID
  entity_type: 'DISTRICT' | 'BUILDING';

  // Emissions (kg CO2e)
  annual_emissions_kg_CO2e: number;
  monthly_emissions_kg_CO2e: number[];

  // Breakdown
  emissions_by_source: {
    electricity_kg_CO2e: number;
    heating_kg_CO2e: number;
    cooling_kg_CO2e: number;
    transportation_kg_CO2e?: number;
    other_kg_CO2e: number;
  };

  // Grid emission factor
  grid_emission_factor_kg_CO2e_per_kWh: number;

  // Offsets
  renewable_offset_kg_CO2e: number;
  carbon_credits_kg_CO2e?: number;

  // Net emissions
  net_emissions_kg_CO2e: number;

  calculation_date: Timestamp;
}

/**
 * Net-zero energy score
 */
export interface NetZeroScore {
  entityId: string;
  entity_type: 'DISTRICT' | 'BUILDING';

  // Energy balance (kWh/year)
  annual_consumption_kWh: number;
  annual_production_kWh: number;
  annual_net_balance_kWh: number;

  // Performance metrics
  zero_energy_achieved: boolean;
  energy_autonomy_percent: number;
  self_consumption_percent: number;

  // Score (0-100)
  overall_score: number;

  // Grade
  grade: 'A+++' | 'A++' | 'A+' | 'A' | 'B' | 'C' | 'D' | 'E' | 'F';

  // Comparison
  better_than_baseline_percent: number;
  peer_ranking_percentile: number;

  evaluated_at: Timestamp;
}

/**
 * Zero energy certification
 */
export interface Certification {
  certificationId: string;
  entityId: string;
  entity_type: 'DISTRICT' | 'BUILDING';

  // Certification body
  certifying_organization: string;
  standard: string; // e.g., "LEED Zero", "Living Building Challenge", "Passive House"

  // Level
  certification_level: 'PLATINUM' | 'GOLD' | 'SILVER' | 'BRONZE' | 'CERTIFIED';

  // Requirements
  requirements_met: string[];
  requirements_pending?: string[];

  // Energy verification
  verified_net_zero: boolean;
  verification_period_years: number;
  verification_method: 'MEASURED' | 'MODELED' | 'HYBRID';

  // Dates
  application_date: Timestamp;
  certification_date?: Timestamp;
  expiry_date?: Timestamp;
  renewal_required: boolean;

  // Documentation
  certificate_url?: string;
  verification_report_url?: string;

  status: 'PENDING' | 'CERTIFIED' | 'EXPIRED' | 'REVOKED';
}

// ============================================================================
// Forecast & Optimization
// ============================================================================

/**
 * Energy demand forecast
 */
export interface DemandForecast {
  districtId: DistrictId;
  forecast_date: Timestamp;

  // Time series forecast (kW)
  hourly_forecast_kW: number[];

  // Confidence intervals
  upper_bound_kW: number[];
  lower_bound_kW: number[];

  // Model info
  model_name: string;
  model_accuracy_percent: number;

  // Forecast horizon
  horizon_hours: number;

  generated_at: Timestamp;
}

/**
 * Energy production forecast
 */
export interface ProductionForecast {
  sourceId: string;
  forecast_date: Timestamp;

  // Time series forecast (kW)
  hourly_forecast_kW: number[];

  // Weather dependency
  weather_dependent: boolean;
  weather_forecast?: {
    temperature_C: number[];
    wind_speed_m_s?: number[];
    solar_irradiance_W_m2?: number[];
  };

  generated_at: Timestamp;
}

/**
 * Grid optimization result
 */
export interface OptimizationResult {
  districtId: DistrictId;
  optimization_timestamp: Timestamp;

  // Objective
  objective: 'MINIMIZE_COST' | 'MINIMIZE_EMISSIONS' | 'MAXIMIZE_RELIABILITY' | 'BALANCED';

  // Recommendations
  battery_charge_schedule_kW: number[];
  grid_import_schedule_kW: number[];
  demand_response_events?: Array<{
    start_time: Timestamp;
    duration_hours: number;
    load_reduction_kW: number;
  }>;

  // Expected outcomes
  expected_cost_savings_USD: number;
  expected_emission_reduction_kg_CO2e: number;
  expected_reliability_improvement_percent: number;

  // Confidence
  confidence_level_percent: number;

  validity_period_hours: number;
}

// ============================================================================
// Error Classes
// ============================================================================

/**
 * Base error class for ZEB City SDK
 */
export class ZEBCityError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ZEBCityError';
  }
}

/**
 * District not found error
 */
export class DistrictNotFoundError extends ZEBCityError {
  constructor(districtId: string) {
    super(
      `District not found: ${districtId}`,
      'DISTRICT_NOT_FOUND',
      { districtId }
    );
    this.name = 'DistrictNotFoundError';
  }
}

/**
 * Grid balance error
 */
export class GridBalanceError extends ZEBCityError {
  constructor(message: string, details?: any) {
    super(message, 'GRID_BALANCE_ERROR', details);
    this.name = 'GridBalanceError';
  }
}

/**
 * Storage capacity error
 */
export class StorageCapacityError extends ZEBCityError {
  constructor(message: string, details?: any) {
    super(message, 'STORAGE_CAPACITY_ERROR', details);
    this.name = 'StorageCapacityError';
  }
}

/**
 * Certification error
 */
export class CertificationError extends ZEBCityError {
  constructor(message: string, details?: any) {
    super(message, 'CERTIFICATION_ERROR', details);
    this.name = 'CertificationError';
  }
}

// ============================================================================
// API Response Types
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
