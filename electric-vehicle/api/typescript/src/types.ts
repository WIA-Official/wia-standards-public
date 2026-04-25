/**
 * WIA-AUTO-004: Electric Vehicle Standard - TypeScript Type Definitions
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
 * Electric vehicle type classification
 */
export type EVType = 'BEV' | 'PHEV' | 'EREV' | 'HEV';

/**
 * Drivetrain configuration
 */
export type DrivetrainType = 'FWD' | 'RWD' | 'AWD' | '4WD';

/**
 * Drive profile type
 */
export type DriveProfile = 'urban' | 'highway' | 'mixed' | 'sport' | 'eco';

/**
 * Terrain type
 */
export type TerrainType = 'flat' | 'rolling' | 'mountainous';

// ============================================================================
// Battery Types
// ============================================================================

/**
 * Battery chemistry types
 */
export type BatteryChemistry =
  | 'NMC111'
  | 'NMC532'
  | 'NMC622'
  | 'NMC811'
  | 'LFP'
  | 'NCA'
  | 'LTO'
  | 'SolidState'
  | 'LiS'
  | 'LiAir';

/**
 * Battery cell format
 */
export type CellFormat =
  | 'cylindrical_18650'
  | 'cylindrical_21700'
  | 'cylindrical_4680'
  | 'pouch'
  | 'prismatic';

/**
 * Battery cooling type
 */
export type BatteryCoolingType = 'passive_air' | 'active_air' | 'liquid' | 'refrigerant' | 'pcm';

/**
 * Battery configuration
 */
export interface BatteryConfiguration {
  /** Total battery capacity in kWh */
  capacity_kwh: number;

  /** Usable capacity in kWh */
  usable_capacity_kwh: number;

  /** Battery chemistry type */
  chemistry: BatteryChemistry;

  /** Nominal voltage in V */
  voltage_nominal: number;

  /** Maximum voltage in V */
  voltage_max: number;

  /** Minimum voltage in V */
  voltage_min: number;

  /** Cell format type */
  cell_type: CellFormat;

  /** Configuration string (e.g., "96s4p") */
  configuration: string;

  /** Cooling system type */
  cooling_type: BatteryCoolingType;

  /** Number of cells in series */
  cells_series?: number;

  /** Number of cells in parallel */
  cells_parallel?: number;

  /** Total number of cells */
  total_cells?: number;

  /** Pack weight in kg */
  weight_kg?: number;

  /** Energy density at pack level in Wh/kg */
  energy_density_wh_kg?: number;
}

/**
 * Battery state
 */
export interface BatteryState {
  /** State of Charge (0-100%) */
  soc_percent: number;

  /** State of Health (0-100%) */
  soh_percent: number;

  /** Pack voltage in V */
  voltage_v: number;

  /** Pack current in A (positive = discharge) */
  current_a: number;

  /** Average temperature in °C */
  temperature_c: number;

  /** Current power in kW (positive = discharge) */
  power_kw: number;

  /** Maximum charge power available in kW */
  max_charge_power_kw: number;

  /** Maximum discharge power available in kW */
  max_discharge_power_kw: number;

  /** Cell voltage array (optional) */
  cell_voltages_v?: number[];

  /** Cell temperature array (optional) */
  cell_temperatures_c?: number[];

  /** Total cycle count */
  cycle_count?: number;

  /** Estimated remaining calendar life in years */
  estimated_life_years?: number;
}

/**
 * Battery limits
 */
export interface BatteryLimits {
  /** Maximum continuous discharge current in A */
  max_discharge_current_a: number;

  /** Peak discharge current in A (10s) */
  peak_discharge_current_a: number;

  /** Maximum continuous charge current in A */
  max_charge_current_a: number;

  /** Peak charge current in A (fast charging) */
  peak_charge_current_a: number;

  /** Maximum cell voltage in V */
  max_cell_voltage_v: number;

  /** Minimum cell voltage in V */
  min_cell_voltage_v: number;

  /** Maximum temperature in °C */
  max_temperature_c: number;

  /** Minimum temperature in °C */
  min_temperature_c: number;
}

/**
 * Battery degradation prediction
 */
export interface BatteryDegradation {
  /** Current State of Health (%) */
  current_soh_percent: number;

  /** Predicted SoH over time */
  predictions: Array<{
    years: number;
    soh_percent: number;
    confidence: number; // 0-1
  }>;

  /** Degradation rate (% per year) */
  degradation_rate_per_year: number;

  /** Factors influencing health */
  factors: {
    temperature_stress: number; // 0-1
    cycle_depth_stress: number; // 0-1
    fast_charge_frequency: number; // 0-1
    calendar_aging: number; // 0-1
  };

  /** Recommendations to improve battery life */
  recommendations: string[];
}

// ============================================================================
// Motor Types
// ============================================================================

/**
 * Electric motor types
 */
export type MotorType = 'PMSM' | 'IM' | 'SRM' | 'BLDC' | 'SynRM';

/**
 * Motor location
 */
export type MotorLocation = 'front' | 'rear' | 'front_rear' | 'in_wheel';

/**
 * Motor configuration
 */
export interface MotorConfiguration {
  /** Motor type */
  type: MotorType;

  /** Number of motors */
  count: number;

  /** Motor location(s) */
  location: MotorLocation;

  /** Rated continuous power in kW */
  power_rated_kw: number;

  /** Peak power in kW (10s) */
  power_peak_kw: number;

  /** Rated continuous torque in N·m */
  torque_rated_nm: number;

  /** Peak torque in N·m */
  torque_peak_nm: number;

  /** Maximum RPM */
  max_rpm: number;

  /** Motor efficiency at rated point (%) */
  efficiency_percent: number;

  /** Motor weight in kg */
  weight_kg?: number;

  /** Power density in kW/kg */
  power_density_kw_kg?: number;

  /** Cooling type */
  cooling_type?: 'air' | 'liquid' | 'oil';
}

/**
 * Motor state
 */
export interface MotorState {
  /** Actual torque output in N·m */
  torque_nm: number;

  /** Rotational speed in RPM */
  speed_rpm: number;

  /** Mechanical power output in kW */
  power_kw: number;

  /** Current efficiency (%) */
  efficiency_percent: number;

  /** Motor temperatures */
  temperature: {
    stator_c: number;
    rotor_c: number;
    windings_c: number;
  };

  /** d-q axis currents (for PMSM/IM) */
  current?: {
    d_axis_a: number;
    q_axis_a: number;
  };

  /** d-q axis voltages (for PMSM/IM) */
  voltage?: {
    d_axis_v: number;
    q_axis_v: number;
  };
}

/**
 * Motor control command
 */
export interface MotorCommand {
  /** Requested torque in N·m */
  torque_request_nm: number;

  /** Speed limit in RPM */
  speed_limit_rpm: number;

  /** Efficiency mode */
  efficiency_mode: 'eco' | 'normal' | 'sport';

  /** Regenerative braking enabled */
  regen_enabled?: boolean;
}

// ============================================================================
// Power Electronics Types
// ============================================================================

/**
 * Inverter type
 */
export type InverterType = 'Si_IGBT' | 'SiC_MOSFET' | 'GaN_HEMT';

/**
 * Inverter configuration
 */
export interface InverterConfiguration {
  /** Inverter semiconductor type */
  type: InverterType;

  /** Inverter efficiency (%) */
  efficiency_percent: number;

  /** Maximum current in A */
  max_current_a: number;

  /** Switching frequency in kHz */
  switching_frequency_khz: number;

  /** Cooling type */
  cooling_type: 'air' | 'liquid';
}

/**
 * Power electronics state
 */
export interface PowerElectronicsState {
  /** DC bus voltage in V */
  dc_bus_voltage_v: number;

  /** DC bus current in A */
  dc_bus_current_a: number;

  /** Inverter temperature in °C */
  inverter_temp_c: number;

  /** Junction temperature in °C */
  junction_temp_c: number;

  /** Current efficiency (%) */
  efficiency_percent: number;

  /** Power loss in kW */
  power_loss_kw: number;
}

// ============================================================================
// Charging Types
// ============================================================================

/**
 * Charging type
 */
export type ChargingType = 'AC_L1' | 'AC_L2' | 'DC_FAST' | 'WIRELESS';

/**
 * Charging connector standard
 */
export type ChargingConnector =
  | 'J1772' // North America AC
  | 'Type2' // Europe AC (Mennekes)
  | 'CCS1' // North America DC
  | 'CCS2' // Europe DC
  | 'CHAdeMO' // Japanese DC
  | 'Tesla' // Tesla proprietary
  | 'NACS' // North American Charging Standard
  | 'GBT'; // Chinese standard

/**
 * Charging configuration
 */
export interface ChargingConfiguration {
  /** Maximum AC charging power in kW */
  ac_max_kw: number;

  /** Maximum DC charging power in kW */
  dc_max_kw: number;

  /** AC connector type */
  ac_connector: ChargingConnector;

  /** DC connector type */
  dc_connector: ChargingConnector;

  /** On-board charger power in kW */
  obc_power_kw?: number;

  /** On-board charger efficiency (%) */
  obc_efficiency_percent?: number;

  /** Supports bidirectional charging (V2G/V2L) */
  bidirectional?: boolean;
}

/**
 * Charging session
 */
export interface ChargingSession {
  /** Session ID */
  session_id: string;

  /** Start timestamp */
  start_time: Date | string;

  /** End timestamp */
  end_time?: Date | string;

  /** Duration in minutes */
  duration_minutes?: number;

  /** Charger ID */
  charger_id: string;

  /** Charger type */
  charger_type: ChargingType;

  /** Maximum power available in kW */
  max_power_kw: number;

  /** Charging standard */
  standard: ChargingConnector;

  /** Energy delivered in kWh */
  energy_delivered_kwh: number;

  /** Total cost */
  cost_total?: number;

  /** Currency */
  cost_currency?: string;

  /** Starting SoC (%) */
  soc_start_percent: number;

  /** Ending SoC (%) */
  soc_end_percent: number;

  /** Battery temperature at start (°C) */
  battery_temp_start_c: number;

  /** Battery temperature at end (°C) */
  battery_temp_end_c: number;

  /** Power profile over time */
  power_profile?: Array<{
    time: string;
    power_kw: number;
    soc_percent: number;
    temp_c: number;
  }>;

  /** Average power in kW */
  average_power_kw?: number;

  /** Peak power in kW */
  peak_power_kw?: number;

  /** Charging efficiency (%) */
  efficiency_percent?: number;
}

/**
 * Charging time request
 */
export interface ChargingTimeRequest {
  /** Battery capacity in kWh */
  battery_capacity_kwh: number;

  /** Current State of Charge (%) */
  current_soc_percent: number;

  /** Target State of Charge (%) */
  target_soc_percent: number;

  /** Charger power in kW */
  charger_power_kw: number;

  /** Charger type */
  charger_type: ChargingType;

  /** Battery temperature in °C */
  battery_temp_c: number;

  /** Ambient temperature in °C */
  ambient_temp_c?: number;
}

/**
 * Charging time response
 */
export interface ChargingTimeResponse {
  /** Estimated time in minutes */
  estimated_time_minutes: number;

  /** Energy to deliver in kWh */
  energy_to_deliver_kwh: number;

  /** Average charging power in kW */
  average_power_kw: number;

  /** Charging curve */
  charging_curve: Array<{
    time_minutes: number;
    soc_percent: number;
    power_kw: number;
  }>;

  /** Preconditioning time if needed (minutes) */
  precondition_time_minutes?: number;

  /** Warnings or limitations */
  warnings?: string[];
}

// ============================================================================
// Vehicle Specifications
// ============================================================================

/**
 * Vehicle physical specifications
 */
export interface VehicleSpecifications {
  /** Vehicle mass in kg */
  mass_kg: number;

  /** Aerodynamic drag coefficient (Cd) */
  drag_coefficient: number;

  /** Frontal area in m² */
  frontal_area_m2: number;

  /** Rolling resistance coefficient */
  rolling_resistance: number;

  /** Tire radius in meters */
  tire_radius_m: number;

  /** Wheelbase in meters */
  wheelbase_m?: number;

  /** Track width in meters */
  track_width_m?: number;

  /** Center of gravity height in meters */
  cog_height_m?: number;

  /** Rotational inertia factor */
  rotational_inertia_factor?: number;
}

/**
 * Complete vehicle configuration
 */
export interface VehicleConfiguration {
  /** Vehicle identification */
  id: string;
  manufacturer: string;
  model: string;
  year: number;
  type: EVType;

  /** Battery system */
  battery: BatteryConfiguration;

  /** Motor system */
  motor: MotorConfiguration;

  /** Power electronics */
  inverter: InverterConfiguration;

  /** Powertrain */
  powertrain: {
    gearbox_type: 'single_speed' | 'multi_speed';
    gearbox_ratio: number;
    gearbox_efficiency_percent: number;
    drivetrain: DrivetrainType;
  };

  /** Vehicle specifications */
  vehicle_specs: VehicleSpecifications;

  /** Charging capabilities */
  charging: ChargingConfiguration;

  /** Thermal management */
  thermal?: {
    battery_cooling: BatteryCoolingType;
    motor_cooling: 'air' | 'liquid' | 'oil';
    cabin_hvac: 'resistive' | 'heat_pump';
  };
}

// ============================================================================
// Range and Energy Consumption
// ============================================================================

/**
 * Range calculation request
 */
export interface RangeCalculationRequest {
  /** Battery capacity in kWh */
  battery_capacity_kwh: number;

  /** Current State of Charge (%) */
  current_soc_percent: number;

  /** Average speed in km/h */
  average_speed_kmh: number;

  /** Ambient temperature in °C */
  ambient_temp_c: number;

  /** Drive profile type */
  drive_profile: DriveProfile;

  /** HVAC enabled */
  hvac_enabled: boolean;

  /** Terrain type */
  terrain_type: TerrainType;

  /** Driving style factor (0.8 = eco, 1.0 = normal, 1.3 = sport) */
  driving_style_factor?: number;

  /** Vehicle specifications (if not using defaults) */
  vehicle_specs?: VehicleSpecifications;

  /** Usable depth of discharge (0-1) */
  usable_dod?: number;

  /** Overall powertrain efficiency (0-1) */
  powertrain_efficiency?: number;
}

/**
 * Range calculation response
 */
export interface RangeCalculationResponse {
  /** Estimated range in km */
  estimated_range_km: number;

  /** Minimum range in km (worst case) */
  range_min_km: number;

  /** Maximum range in km (best case) */
  range_max_km: number;

  /** Energy consumption in kWh/100km */
  energy_consumption_kwh_per_100km: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Breakdown of factors */
  factors: {
    base_consumption_kwh_per_100km: number;
    temperature_impact_percent: number;
    hvac_impact_percent: number;
    terrain_impact_percent: number;
    style_impact_percent: number;
  };

  /** Available energy in kWh */
  available_energy_kwh: number;
}

/**
 * Energy consumption request
 */
export interface EnergyConsumptionRequest {
  /** Distance to travel in km */
  distance_km: number;

  /** Average speed in km/h */
  average_speed_kmh: number;

  /** Vehicle mass in kg */
  vehicle_mass_kg: number;

  /** Drag coefficient */
  drag_coefficient: number;

  /** Frontal area in m² */
  frontal_area_m2: number;

  /** Rolling resistance coefficient */
  rolling_resistance: number;

  /** Road grade in % (positive = uphill) */
  grade_percent?: number;

  /** Average acceleration in m/s² */
  acceleration_mps2?: number;

  /** Ambient temperature in °C */
  ambient_temp_c?: number;

  /** HVAC power in kW */
  hvac_power_kw?: number;

  /** Powertrain efficiency (0-1) */
  powertrain_efficiency?: number;
}

/**
 * Energy consumption response
 */
export interface EnergyConsumptionResponse {
  /** Total energy required in kWh */
  total_energy_kwh: number;

  /** Energy per km in kWh/km */
  energy_per_km: number;

  /** Energy per 100km in kWh/100km */
  energy_per_100km: number;

  /** Energy breakdown */
  breakdown: {
    aerodynamic_kwh: number;
    rolling_kwh: number;
    grade_kwh: number;
    acceleration_kwh: number;
    hvac_kwh: number;
    accessories_kwh: number;
    drivetrain_loss_kwh: number;
  };

  /** Overall efficiency (%) */
  efficiency_percent: number;
}

// ============================================================================
// Regenerative Braking
// ============================================================================

/**
 * Regenerative braking request
 */
export interface RegenerativeBrakingRequest {
  /** Vehicle mass in kg */
  vehicle_mass_kg: number;

  /** Initial speed in km/h */
  initial_speed_kmh: number;

  /** Final speed in km/h */
  final_speed_kmh: number;

  /** Deceleration rate in m/s² */
  deceleration_mps2: number;

  /** Motor efficiency as generator (0-1) */
  motor_efficiency: number;

  /** Inverter efficiency (0-1) */
  inverter_efficiency: number;

  /** Battery charging efficiency (0-1) */
  battery_efficiency: number;

  /** Current State of Charge (%) */
  current_soc_percent: number;

  /** Battery temperature in °C */
  battery_temp_c: number;

  /** Maximum regenerative power in kW */
  max_regen_power_kw?: number;
}

/**
 * Regenerative braking response
 */
export interface RegenerativeBrakingResponse {
  /** Kinetic energy available in kWh */
  kinetic_energy_kwh: number;

  /** Theoretically recoverable energy in kWh */
  recoverable_energy_kwh: number;

  /** Actually recovered energy in kWh */
  actual_recovered_kwh: number;

  /** Recovery efficiency (%) */
  recovery_efficiency_percent: number;

  /** Limitations encountered */
  limitations: string[];

  /** Range extension in km */
  range_extension_km: number;

  /** Average regenerative power in kW */
  average_regen_power_kw: number;
}

// ============================================================================
// Drive Cycle and Simulation
// ============================================================================

/**
 * Drive cycle segment
 */
export interface DriveCycleSegment {
  /** Segment type */
  type: 'urban' | 'highway' | 'rural' | 'stop' | 'acceleration';

  /** Distance in km */
  distance_km: number;

  /** Average speed in km/h */
  average_speed_kmh: number;

  /** Energy consumed in kWh */
  energy_kwh: number;

  /** Duration in seconds */
  duration_seconds?: number;
}

/**
 * Drive profile data
 */
export interface DriveProfileData {
  /** Profile timestamp */
  timestamp: Date | string;

  /** Total duration in seconds */
  duration_seconds: number;

  /** Total distance in km */
  distance_km: number;

  /** Average speed in km/h */
  average_speed_kmh: number;

  /** Total energy consumed in kWh */
  energy_consumed_kwh: number;

  /** Energy efficiency in kWh/100km */
  efficiency_kwh_per_100km: number;

  /** Conditions during drive */
  conditions: {
    ambient_temp_c: number;
    hvac_power_kw: number;
    terrain: TerrainType;
    weather: string;
  };

  /** Starting SoC (%) */
  soc_start_percent: number;

  /** Ending SoC (%) */
  soc_end_percent: number;

  /** Regenerative energy recovered in kWh */
  regenerative_energy_kwh: number;

  /** Regenerative efficiency (%) */
  regenerative_efficiency_percent: number;

  /** Detailed segments */
  segments?: DriveCycleSegment[];
}

// ============================================================================
// Performance Metrics
// ============================================================================

/**
 * Vehicle performance metrics
 */
export interface PerformanceMetrics {
  /** 0-100 km/h acceleration time in seconds */
  acceleration_0_100_kmh_s: number;

  /** Top speed in km/h */
  top_speed_kmh: number;

  /** EPA combined range in km */
  epa_range_km: number;

  /** WLTP range in km */
  wltp_range_km: number;

  /** EPA combined efficiency in kWh/100km */
  epa_efficiency_kwh_per_100km: number;

  /** WLTP efficiency in kWh/100km */
  wltp_efficiency_kwh_per_100km: number;

  /** DC fast charging time (10-80%) in minutes */
  dc_fast_charge_10_80_minutes: number;

  /** Maximum payload in kg */
  max_payload_kg?: number;

  /** Towing capacity in kg */
  towing_capacity_kg?: number;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Physical constants for EV calculations
 */
export const EV_CONSTANTS = {
  /** Gravitational acceleration in m/s² */
  GRAVITY: 9.81,

  /** Air density at sea level, 15°C in kg/m³ */
  AIR_DENSITY: 1.225,

  /** Conversion factor: kW to HP */
  KW_TO_HP: 1.341,

  /** Conversion factor: km/h to m/s */
  KMH_TO_MS: 1 / 3.6,

  /** Conversion factor: RPM to rad/s */
  RPM_TO_RADS: 2 * Math.PI / 60,

  /** Conversion factor: J to kWh */
  J_TO_KWH: 1 / 3600000,

  /** Typical usable depth of discharge for Li-ion */
  TYPICAL_DOD: 0.9,

  /** Typical powertrain efficiency */
  TYPICAL_POWERTRAIN_EFFICIENCY: 0.90,

  /** Typical regenerative braking efficiency */
  TYPICAL_REGEN_EFFICIENCY: 0.70,
} as const;

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
 * Simulation result
 */
export interface SimulationResult {
  /** Simulation ID */
  id: string;

  /** Vehicle configuration used */
  vehicle: VehicleConfiguration;

  /** Drive profile simulated */
  drive_profile: DriveProfileData;

  /** Range estimate */
  range_estimate: RangeCalculationResponse;

  /** Performance metrics */
  performance?: PerformanceMetrics;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;

  /** Simulation timestamp */
  timestamp: Date | string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-004 error codes
 */
export enum EVErrorCode {
  BATTERY_OVERVOLTAGE = 'EV001',
  BATTERY_UNDERVOLTAGE = 'EV002',
  BATTERY_OVERCURRENT = 'EV003',
  BATTERY_OVERTEMP = 'EV004',
  MOTOR_OVERTEMP = 'EV005',
  INVERTER_FAULT = 'EV006',
  ISOLATION_FAULT = 'EV007',
  CHARGING_FAULT = 'EV008',
  THERMAL_RUNAWAY = 'EV009',
  HV_DISCONNECT = 'EV010',
  INVALID_PARAMETERS = 'EV011',
  SENSOR_FAULT = 'EV012',
  COMMUNICATION_ERROR = 'EV013',
}

/**
 * Electric vehicle error
 */
export class ElectricVehicleError extends Error {
  constructor(
    public code: EVErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'ElectricVehicleError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Vehicle types
  EVType,
  DrivetrainType,
  DriveProfile,
  TerrainType,

  // Battery types
  BatteryChemistry,
  CellFormat,
  BatteryCoolingType,
  BatteryConfiguration,
  BatteryState,
  BatteryLimits,
  BatteryDegradation,

  // Motor types
  MotorType,
  MotorLocation,
  MotorConfiguration,
  MotorState,
  MotorCommand,

  // Power electronics
  InverterType,
  InverterConfiguration,
  PowerElectronicsState,

  // Charging types
  ChargingType,
  ChargingConnector,
  ChargingConfiguration,
  ChargingSession,
  ChargingTimeRequest,
  ChargingTimeResponse,

  // Vehicle specs
  VehicleSpecifications,
  VehicleConfiguration,

  // Range and consumption
  RangeCalculationRequest,
  RangeCalculationResponse,
  EnergyConsumptionRequest,
  EnergyConsumptionResponse,

  // Regenerative braking
  RegenerativeBrakingRequest,
  RegenerativeBrakingResponse,

  // Drive cycle
  DriveCycleSegment,
  DriveProfileData,

  // Performance
  PerformanceMetrics,

  // Simulation
  SimulationResult,
};

export { EV_CONSTANTS, EVErrorCode, ElectricVehicleError };
