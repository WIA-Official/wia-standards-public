/**
 * WIA-IND-002: Smart Textile Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industrial Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This module provides comprehensive type definitions for smart textile technology,
 * including conductive fibers, sensors, temperature regulation, e-textiles,
 * health monitoring, energy harvesting, and communication systems.
 */

// ============================================================================
// Core Textile Types
// ============================================================================

/**
 * Base textile material types
 */
export type BaseFabricType =
  | 'cotton'
  | 'polyester'
  | 'nylon'
  | 'spandex'
  | 'wool'
  | 'silk'
  | 'linen'
  | 'rayon'
  | 'polyester_cotton_blend'
  | 'cotton_spandex'
  | 'nylon_spandex';

/**
 * Textile construction methods
 */
export type TextileConstruction = 'woven' | 'knitted' | 'nonwoven' | 'braided' | 'embroidered';

/**
 * Garment types
 */
export type GarmentType =
  | 'shirt'
  | 't_shirt'
  | 'pants'
  | 'jacket'
  | 'vest'
  | 'glove'
  | 'sock'
  | 'hat'
  | 'bandage'
  | 'strap'
  | 'patch'
  | 'full_body_suit';

// ============================================================================
// Conductive Fiber Types
// ============================================================================

/**
 * Conductive fiber material types
 */
export type ConductiveMaterial =
  | 'silver_plated_nylon'
  | 'silver_plated_polyester'
  | 'copper_wire'
  | 'stainless_steel'
  | 'graphene'
  | 'carbon_nanotube'
  | 'conductive_polymer_PEDOT'
  | 'conductive_polymer_PANI'
  | 'silver_ink'
  | 'carbon_ink'
  | 'graphene_ink'
  | 'hybrid_composite';

/**
 * Conductive fiber configuration
 */
export interface ConductiveFiberConfig {
  /** Conductive material type */
  material: ConductiveMaterial;

  /** Electrical conductivity in S/m */
  conductivity_sm: number;

  /** Volume fraction of conductive material (0-1) */
  fiber_fraction: number;

  /** Fiber alignment factor (0-1, 1 = perfect alignment) */
  alignment_factor: number;

  /** Inter-fiber contact factor (0-1) */
  contact_factor: number;

  /** Diameter in micrometers */
  diameter_um: number;

  /** Linear resistance in Ω/m */
  resistance_per_meter: number;

  /** Current carrying capacity in mA */
  max_current_ma: number;
}

/**
 * Conductive ink properties
 */
export interface ConductiveInkProperties {
  /** Ink type */
  type: 'silver_nanoparticle' | 'graphene' | 'carbon_black' | 'CNT';

  /** Conductivity in S/m */
  conductivity_sm: number;

  /** Sheet resistance in Ω/sq */
  sheet_resistance_ohm_sq: number;

  /** Sintering temperature in °C */
  sintering_temp_c: number;

  /** Particle size in nm */
  particle_size_nm: number;

  /** Viscosity in cP */
  viscosity_cp: number;

  /** Solid content percentage */
  solid_content_pct: number;
}

// ============================================================================
// Sensor Types
// ============================================================================

/**
 * Sensor categories
 */
export type SensorCategory = 'physiological' | 'physical' | 'chemical' | 'environmental';

/**
 * Physiological sensor types
 */
export type PhysiologicalSensorType = 'ECG' | 'EMG' | 'temperature' | 'respiration' | 'SpO2' | 'blood_pressure';

/**
 * Physical sensor types
 */
export type PhysicalSensorType = 'strain' | 'pressure' | 'accelerometer' | 'gyroscope' | 'magnetometer';

/**
 * Chemical sensor types
 */
export type ChemicalSensorType = 'pH' | 'lactate' | 'glucose' | 'sodium' | 'potassium' | 'cortisol';

/**
 * Environmental sensor types
 */
export type EnvironmentalSensorType = 'UV' | 'air_quality' | 'humidity' | 'ambient_temperature' | 'light';

/**
 * All sensor types union
 */
export type SensorType =
  | PhysiologicalSensorType
  | PhysicalSensorType
  | ChemicalSensorType
  | EnvironmentalSensorType;

/**
 * ECG sensor configuration
 */
export interface ECGSensorConfig {
  /** Sensor type */
  type: 'ECG';

  /** Number of leads (1, 3, 5, or 12) */
  lead_count: 1 | 3 | 5 | 12;

  /** Electrode area in cm² */
  electrode_area_cm2: number;

  /** Skin-electrode impedance in kΩ */
  electrode_impedance_kohm: number;

  /** Sampling rate in Hz */
  sampling_rate_hz: number;

  /** ADC resolution in bits */
  adc_resolution_bits: number;

  /** Common mode rejection ratio in dB */
  cmrr_db: number;

  /** Input impedance in MΩ */
  input_impedance_mohm: number;

  /** Bandwidth in Hz [low, high] */
  bandwidth_hz: [number, number];
}

/**
 * Strain sensor configuration
 */
export interface StrainSensorConfig {
  /** Sensor type */
  type: 'strain';

  /** Gauge factor (sensitivity) */
  gauge_factor: number;

  /** Maximum strain percentage */
  max_strain_pct: number;

  /** Base resistance in Ω */
  base_resistance_ohm: number;

  /** Sensor length in mm */
  length_mm: number;

  /** Sensor width in mm */
  width_mm: number;

  /** Hysteresis percentage */
  hysteresis_pct: number;

  /** Linearity error percentage */
  linearity_error_pct: number;

  /** Response time in ms */
  response_time_ms: number;
}

/**
 * Pressure sensor configuration
 */
export interface PressureSensorConfig {
  /** Sensor type */
  type: 'pressure';

  /** Technology (resistive, capacitive, piezoelectric) */
  technology: 'resistive' | 'capacitive' | 'piezoelectric';

  /** Pressure range in kPa [min, max] */
  pressure_range_kpa: [number, number];

  /** Sensitivity in kPa⁻¹ */
  sensitivity_kpa_inv: number;

  /** Spatial resolution in mm */
  spatial_resolution_mm: number;

  /** Array size [rows, columns] */
  array_size?: [number, number];

  /** Response time in ms */
  response_time_ms: number;
}

/**
 * Temperature sensor configuration
 */
export interface TemperatureSensorConfig {
  /** Sensor type */
  type: 'temperature';

  /** Technology */
  technology: 'thermistor' | 'thermocouple' | 'RTD' | 'conductive_polymer';

  /** Temperature range in °C [min, max] */
  temp_range_c: [number, number];

  /** Accuracy in °C */
  accuracy_c: number;

  /** Resolution in °C */
  resolution_c: number;

  /** Response time in seconds */
  response_time_s: number;

  /** Beta constant for thermistors (K) */
  beta_constant_k?: number;

  /** Thermocouple type (K, T, J, etc.) */
  tc_type?: 'K' | 'T' | 'J' | 'E' | 'N';
}

/**
 * Biosensor configuration for chemical sensing
 */
export interface BiosensorConfig {
  /** Sensor type */
  type: ChemicalSensorType;

  /** Detection method */
  detection_method: 'enzymatic' | 'electrochemical' | 'optical' | 'potentiometric';

  /** Measurement range [min, max] in appropriate units */
  measurement_range: [number, number];

  /** Unit of measurement */
  unit: string;

  /** Sensitivity */
  sensitivity: number;

  /** Selectivity (interference level) */
  selectivity: string;

  /** Response time in seconds */
  response_time_s: number;

  /** Enzyme type (if enzymatic) */
  enzyme?: string;

  /** Lifetime in days */
  lifetime_days: number;
}

/**
 * Generic sensor configuration union
 */
export type SensorConfig =
  | ECGSensorConfig
  | StrainSensorConfig
  | PressureSensorConfig
  | TemperatureSensorConfig
  | BiosensorConfig;

/**
 * Sensor placement on garment
 */
export interface SensorPlacement {
  /** Sensor identifier */
  sensor_id: string;

  /** Sensor configuration */
  config: SensorConfig;

  /** Position on garment */
  position: {
    /** X coordinate (0-1, normalized) */
    x: number;
    /** Y coordinate (0-1, normalized) */
    y: number;
    /** Body location description */
    location: BodyLocation;
  };

  /** Contact pressure in kPa (for electrode sensors) */
  contact_pressure_kpa?: number;

  /** Orientation angle in degrees */
  orientation_deg?: number;
}

/**
 * Body location for sensor placement
 */
export type BodyLocation =
  | 'chest_left'
  | 'chest_center'
  | 'chest_right'
  | 'abdomen'
  | 'back_upper'
  | 'back_lower'
  | 'shoulder_left'
  | 'shoulder_right'
  | 'arm_left'
  | 'arm_right'
  | 'forearm_left'
  | 'forearm_right'
  | 'wrist_left'
  | 'wrist_right'
  | 'thigh_left'
  | 'thigh_right'
  | 'calf_left'
  | 'calf_right'
  | 'ankle_left'
  | 'ankle_right'
  | 'foot_left'
  | 'foot_right'
  | 'neck'
  | 'head'
  | 'waist';

// ============================================================================
// Temperature Regulation Types
// ============================================================================

/**
 * Temperature regulation method
 */
export type TemperatureRegulationMethod =
  | 'PCM'
  | 'resistive_heating'
  | 'thermoelectric'
  | 'evaporative_cooling'
  | 'radiative_cooling'
  | 'passive';

/**
 * Phase Change Material (PCM) configuration
 */
export interface PCMConfig {
  /** PCM type */
  type: 'paraffin' | 'salt_hydrate' | 'fatty_acid' | 'polyethylene_glycol';

  /** Melting point in °C */
  melting_point_c: number;

  /** Latent heat of fusion in J/g */
  latent_heat_j_per_g: number;

  /** Specific heat capacity in J/(g·K) */
  specific_heat_j_per_g_k: number;

  /** Thermal conductivity in W/(m·K) */
  thermal_conductivity_w_per_mk: number;

  /** PCM mass in grams */
  mass_g: number;

  /** Encapsulation method */
  encapsulation: 'microencapsulated' | 'macroencapsulated' | 'fiber_integrated' | 'laminated';

  /** Particle size for microencapsulation in μm */
  particle_size_um?: number;

  /** Loading percentage in textile */
  loading_pct: number;
}

/**
 * Resistive heating configuration
 */
export interface ResistiveHeatingConfig {
  /** Heating element material */
  material: ConductiveMaterial;

  /** Resistance in Ω */
  resistance_ohm: number;

  /** Operating voltage in V */
  voltage_v: number;

  /** Target power density in W/m² */
  power_density_w_per_m2: number;

  /** Heated area in cm² */
  heated_area_cm2: number;

  /** Maximum temperature in °C */
  max_temperature_c: number;

  /** Temperature control method */
  control_method: 'on_off' | 'PWM' | 'PID' | 'PTC_self_regulating';

  /** Thermal cutoff temperature in °C */
  cutoff_temperature_c: number;
}

/**
 * Thermoelectric (Peltier) configuration
 */
export interface ThermoelectricConfig {
  /** Module count */
  module_count: number;

  /** Module dimensions in mm [length, width, thickness] */
  module_dimensions_mm: [number, number, number];

  /** Maximum current in A */
  max_current_a: number;

  /** Maximum voltage in V */
  max_voltage_v: number;

  /** Coefficient of performance (COP) */
  cop: number;

  /** Maximum temperature difference in K */
  max_delta_t_k: number;

  /** Operating mode */
  mode: 'heating' | 'cooling' | 'bidirectional';
}

/**
 * Thermal performance metrics
 */
export interface ThermalPerformance {
  /** Cooling/heating power in W */
  power_w: number;

  /** Energy storage capacity in J (for PCM) */
  energy_storage_j?: number;

  /** Thermal buffering time in minutes (for PCM) */
  buffering_time_min?: number;

  /** Temperature regulation range in °C [min, max] */
  regulation_range_c: [number, number];

  /** Response time in seconds */
  response_time_s: number;

  /** Efficiency (0-1) */
  efficiency: number;
}

/**
 * Breathability metrics
 */
export interface BreathabilityMetrics {
  /** Moisture Vapor Transmission Rate in g/m²/day */
  mvtr_g_per_m2_day: number;

  /** Air permeability in cm³/s/cm² */
  air_permeability_cm3_per_s_cm2: number;

  /** Water resistance in mm H₂O */
  water_resistance_mm_h2o: number;

  /** Wicking rate in cm/10s */
  wicking_rate_cm_per_10s: number;

  /** Drying time in minutes */
  drying_time_min: number;
}

// ============================================================================
// E-Textile Architecture Types
// ============================================================================

/**
 * Circuit trace method
 */
export type CircuitTraceMethod = 'woven' | 'embroidered' | 'screen_printed' | 'inkjet_printed' | 'spray_coated';

/**
 * Circuit trace configuration
 */
export interface CircuitTrace {
  /** Trace ID */
  trace_id: string;

  /** Manufacturing method */
  method: CircuitTraceMethod;

  /** Conductive material */
  material: ConductiveMaterial;

  /** Line width in μm */
  line_width_um: number;

  /** Trace thickness in μm */
  thickness_um: number;

  /** Resistance per unit length in Ω/cm */
  resistance_per_cm: number;

  /** Maximum current in mA */
  max_current_ma: number;

  /** Start and end points (normalized coordinates) */
  path: Array<{ x: number; y: number }>;

  /** Layer number (for multilayer textiles) */
  layer: number;

  /** Insulation type */
  insulation?: 'coating' | 'fabric_layer' | 'none';
}

/**
 * Component attachment method
 */
export type AttachmentMethod = 'snap_fastener' | 'conductive_adhesive' | 'solder' | 'ACF' | 'magnetic';

/**
 * Electronic component integration
 */
export interface ElectronicComponent {
  /** Component ID */
  component_id: string;

  /** Component type */
  type: 'microcontroller' | 'sensor_ic' | 'led' | 'battery' | 'wireless_module' | 'passive' | 'other';

  /** Position on textile */
  position: { x: number; y: number };

  /** Attachment method */
  attachment: AttachmentMethod;

  /** Dimensions in mm [length, width, height] */
  dimensions_mm: [number, number, number];

  /** Mass in grams */
  mass_g: number;

  /** Encapsulation rating */
  ip_rating: string;

  /** Power consumption in mW */
  power_consumption_mw: number;

  /** Operating voltage in V */
  operating_voltage_v: number;

  /** Removable for washing */
  removable: boolean;
}

/**
 * Power source configuration
 */
export interface PowerSourceConfig {
  /** Power source type */
  type: 'lithium_polymer' | 'lithium_ion' | 'flexible_battery' | 'printed_battery' | 'supercapacitor' | 'hybrid';

  /** Nominal voltage in V */
  voltage_nominal_v: number;

  /** Voltage range in V [min, max] */
  voltage_range_v: [number, number];

  /** Capacity in mAh */
  capacity_mah: number;

  /** Energy density in Wh/kg */
  energy_density_wh_per_kg: number;

  /** Dimensions in mm [length, width, thickness] */
  dimensions_mm: [number, number, number];

  /** Mass in grams */
  mass_g: number;

  /** Charging method */
  charging: 'wired' | 'wireless' | 'solar' | 'energy_harvesting';

  /** Battery protection features */
  protection: {
    overcharge: boolean;
    over_discharge: boolean;
    overcurrent: boolean;
    thermal_cutoff: boolean;
    short_circuit: boolean;
  };
}

/**
 * Power budget calculation
 */
export interface PowerBudget {
  /** Microcontroller power in mW */
  mcu_power_mw: number;

  /** Sensor power in mW */
  sensor_power_mw: number;

  /** Wireless communication power in mW */
  wireless_power_mw: number;

  /** Actuator power (heating, LED, etc.) in mW */
  actuator_power_mw: number;

  /** Other power consumption in mW */
  other_power_mw: number;

  /** Total power in mW */
  total_power_mw: number;

  /** Battery life in hours */
  battery_life_hours: number;
}

// ============================================================================
// Energy Harvesting Types
// ============================================================================

/**
 * Energy harvesting technology
 */
export type EnergyHarvestingTechnology =
  | 'piezoelectric'
  | 'thermoelectric'
  | 'solar'
  | 'triboelectric'
  | 'electromagnetic'
  | 'hybrid';

/**
 * Piezoelectric harvester configuration
 */
export interface PiezoelectricHarvesterConfig {
  /** Technology type */
  type: 'piezoelectric';

  /** Piezoelectric material */
  material: 'PVDF' | 'PZT' | 'ZnO_nanowire' | 'BaTiO3';

  /** Piezoelectric coefficient d₃₃ in pC/N */
  d33_pc_per_n: number;

  /** Active area in cm² */
  active_area_cm2: number;

  /** Thickness in μm */
  thickness_um: number;

  /** Mechanical frequency in Hz */
  frequency_hz: number;

  /** Applied force in N */
  force_n: number;

  /** Power output in μW */
  power_output_uw: number;

  /** Placement location */
  placement: BodyLocation;
}

/**
 * Thermoelectric harvester configuration
 */
export interface ThermoelectricHarvesterConfig {
  /** Technology type */
  type: 'thermoelectric';

  /** Thermoelectric material */
  material: 'Bi2Te3' | 'conductive_polymer' | 'CNT_composite';

  /** Seebeck coefficient in μV/K */
  seebeck_coefficient_uv_per_k: number;

  /** Figure of merit ZT */
  zt_figure_of_merit: number;

  /** Active area in cm² */
  active_area_cm2: number;

  /** Temperature difference in K */
  delta_t_k: number;

  /** Internal resistance in Ω */
  internal_resistance_ohm: number;

  /** Power output in μW */
  power_output_uw: number;

  /** Thermal insulation thickness in mm */
  insulation_thickness_mm: number;
}

/**
 * Solar harvester configuration
 */
export interface SolarHarvesterConfig {
  /** Technology type */
  type: 'solar';

  /** Solar cell technology */
  technology: 'silicon_thin_film' | 'organic_PV' | 'dye_sensitized' | 'perovskite';

  /** Efficiency percentage */
  efficiency_pct: number;

  /** Active area in cm² */
  active_area_cm2: number;

  /** Incident light intensity in W/m² */
  light_intensity_w_per_m2: number;

  /** Power output in mW */
  power_output_mw: number;

  /** Flexible design */
  flexible: boolean;

  /** Placement location */
  placement: 'shoulder' | 'back' | 'chest' | 'hat' | 'arm';
}

/**
 * Triboelectric harvester configuration
 */
export interface TriboelectricHarvesterConfig {
  /** Technology type */
  type: 'triboelectric';

  /** Operating mode */
  mode: 'contact_separation' | 'sliding' | 'single_electrode' | 'freestanding';

  /** Positive triboelectric material */
  material_positive: 'nylon' | 'polyester' | 'silk';

  /** Negative triboelectric material */
  material_negative: 'PTFE' | 'silicone' | 'Kapton';

  /** Active area in cm² */
  active_area_cm2: number;

  /** Contact frequency in Hz */
  contact_frequency_hz: number;

  /** Open circuit voltage in V */
  open_circuit_voltage_v: number;

  /** Short circuit current in μA */
  short_circuit_current_ua: number;

  /** Average power output in μW */
  power_output_uw: number;
}

/**
 * Energy harvesting system configuration
 */
export type EnergyHarvesterConfig =
  | PiezoelectricHarvesterConfig
  | ThermoelectricHarvesterConfig
  | SolarHarvesterConfig
  | TriboelectricHarvesterConfig;

/**
 * Energy storage configuration
 */
export interface EnergyStorageConfig {
  /** Storage type */
  type: 'supercapacitor' | 'rechargeable_battery' | 'hybrid';

  /** Capacity in mAh (battery) or F (supercapacitor) */
  capacity: number;

  /** Voltage in V */
  voltage_v: number;

  /** Energy in Wh or J */
  energy: number;

  /** Charge/discharge cycles */
  cycle_life: number;

  /** Self-discharge rate in %/day */
  self_discharge_pct_per_day: number;
}

// ============================================================================
// Health Monitoring Types
// ============================================================================

/**
 * Vital signs measurement
 */
export interface VitalSigns {
  /** Heart rate in bpm */
  heart_rate_bpm?: number;

  /** Heart rate variability (SDNN) in ms */
  hrv_sdnn_ms?: number;

  /** Respiration rate in breaths/min */
  respiration_rate_bpm?: number;

  /** Skin temperature in °C */
  skin_temperature_c?: number;

  /** Core temperature in °C (estimated) */
  core_temperature_c?: number;

  /** SpO₂ percentage */
  spo2_pct?: number;

  /** Blood pressure [systolic, diastolic] in mmHg */
  blood_pressure_mmhg?: [number, number];

  /** Activity level (METs) */
  activity_mets?: number;

  /** Timestamp */
  timestamp: Date;

  /** Data quality indicator (0-1, 1 = excellent) */
  quality: number;
}

/**
 * ECG waveform data
 */
export interface ECGWaveform {
  /** Sampling rate in Hz */
  sampling_rate_hz: number;

  /** Lead name */
  lead: string;

  /** Voltage values in mV */
  voltage_mv: number[];

  /** Timestamp for first sample */
  timestamp_start: Date;

  /** QRS complex detections (R-peak indices) */
  r_peaks?: number[];

  /** Heart rate in bpm */
  heart_rate_bpm?: number;

  /** Arrhythmia detection */
  arrhythmia?: ArrhythmiaType;
}

/**
 * Arrhythmia types
 */
export type ArrhythmiaType =
  | 'normal'
  | 'atrial_fibrillation'
  | 'atrial_flutter'
  | 'ventricular_tachycardia'
  | 'ventricular_fibrillation'
  | 'bradycardia'
  | 'tachycardia'
  | 'pvc'
  | 'pac'
  | 'unknown';

/**
 * Activity classification
 */
export type ActivityType =
  | 'resting'
  | 'sitting'
  | 'standing'
  | 'walking'
  | 'running'
  | 'cycling'
  | 'stair_climbing'
  | 'sleeping'
  | 'unknown';

/**
 * Activity data
 */
export interface ActivityData {
  /** Activity type */
  activity: ActivityType;

  /** Confidence level (0-1) */
  confidence: number;

  /** Duration in seconds */
  duration_s: number;

  /** Energy expenditure in calories */
  calories: number;

  /** Step count (for walking/running) */
  steps?: number;

  /** Distance in meters */
  distance_m?: number;

  /** Average speed in m/s */
  avg_speed_m_per_s?: number;

  /** Timestamp */
  timestamp: Date;
}

/**
 * Sleep analysis
 */
export interface SleepAnalysis {
  /** Sleep stage */
  stage: 'awake' | 'light' | 'deep' | 'REM';

  /** Stage duration in minutes */
  duration_min: number;

  /** Sleep quality score (0-100) */
  quality_score: number;

  /** Movement count */
  movement_count: number;

  /** Average heart rate during stage in bpm */
  avg_heart_rate_bpm: number;

  /** Average respiration rate in breaths/min */
  avg_respiration_rate_bpm: number;

  /** Timestamp */
  timestamp: Date;
}

// ============================================================================
// Washability and Durability Types
// ============================================================================

/**
 * Wash test protocol
 */
export interface WashTestProtocol {
  /** Standard (ISO 6330, IEC 61340-4-9, etc.) */
  standard: string;

  /** Wash temperature in °C */
  temperature_c: number;

  /** Detergent type */
  detergent: 'anionic' | 'cationic' | 'nonionic' | 'enzymatic' | 'neutral';

  /** Cycle type */
  cycle_type: 'normal' | 'delicate' | 'hand_wash';

  /** Number of cycles */
  cycles: number;

  /** Drying method */
  drying: 'air_dry' | 'tumble_dry_low' | 'tumble_dry_high';

  /** Spin speed in RPM */
  spin_speed_rpm?: number;
}

/**
 * Durability test results
 */
export interface DurabilityTestResults {
  /** Resistance retention percentage after washing */
  resistance_retention_pct: number;

  /** Sensor accuracy degradation percentage */
  sensor_accuracy_degradation_pct: number;

  /** Tensile strength retention percentage */
  tensile_strength_retention_pct: number;

  /** Abrasion resistance grade (1-5) */
  abrasion_grade: 1 | 2 | 3 | 4 | 5;

  /** Flex cycles to failure */
  flex_cycles_to_failure: number;

  /** Visual defects */
  visual_defects: string[];

  /** Pass/Fail */
  pass: boolean;
}

/**
 * IP (Ingress Protection) rating
 */
export interface IPRating {
  /** IP code (e.g., "IP67") */
  code: string;

  /** Dust protection level (0-6) */
  dust_protection: 0 | 1 | 2 | 3 | 4 | 5 | 6;

  /** Water protection level (0-9) */
  water_protection: 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9;

  /** Description */
  description: string;
}

// ============================================================================
// Communication Types
// ============================================================================

/**
 * Wireless protocol
 */
export type WirelessProtocol = 'BLE' | 'WiFi' | 'NFC' | 'Zigbee' | 'LoRa' | 'cellular';

/**
 * BLE configuration
 */
export interface BLEConfig {
  /** Protocol type */
  protocol: 'BLE';

  /** BLE version */
  version: '4.0' | '4.1' | '4.2' | '5.0' | '5.1' | '5.2' | '5.3';

  /** Advertising interval in ms */
  advertising_interval_ms: number;

  /** Connection interval in ms */
  connection_interval_ms: number;

  /** MTU size in bytes */
  mtu_bytes: number;

  /** TX power in dBm */
  tx_power_dbm: number;

  /** Services */
  services: BLEService[];

  /** Pairing required */
  pairing_required: boolean;

  /** Encryption enabled */
  encryption: boolean;
}

/**
 * BLE service definition
 */
export interface BLEService {
  /** Service UUID */
  uuid: string;

  /** Service name */
  name: string;

  /** Characteristics */
  characteristics: BLECharacteristic[];
}

/**
 * BLE characteristic definition
 */
export interface BLECharacteristic {
  /** Characteristic UUID */
  uuid: string;

  /** Characteristic name */
  name: string;

  /** Properties */
  properties: {
    read: boolean;
    write: boolean;
    notify: boolean;
    indicate: boolean;
  };

  /** Data type */
  data_type: string;

  /** Unit */
  unit?: string;
}

/**
 * Data transmission configuration
 */
export interface DataTransmissionConfig {
  /** Wireless protocol */
  protocol: WirelessProtocol;

  /** Transmission interval in seconds */
  interval_s: number;

  /** Data format */
  format: 'JSON' | 'binary' | 'CSV' | 'protobuf';

  /** Compression enabled */
  compression: boolean;

  /** Encryption enabled */
  encryption: boolean;

  /** Buffer size in KB */
  buffer_size_kb: number;

  /** Retry attempts */
  retry_attempts: number;
}

// ============================================================================
// Smart Textile System Types
// ============================================================================

/**
 * Complete smart textile system configuration
 */
export interface SmartTextileSystem {
  /** System ID */
  system_id: string;

  /** System name */
  name: string;

  /** Version */
  version: string;

  /** Garment type */
  garment_type: GarmentType;

  /** Base fabric */
  base_fabric: BaseFabricType;

  /** Textile construction */
  construction: TextileConstruction;

  /** Size (S, M, L, XL, etc.) */
  size: string;

  /** Conductive fibers */
  conductive_fibers: ConductiveFiberConfig[];

  /** Sensors */
  sensors: SensorPlacement[];

  /** Temperature regulation */
  temperature_regulation?: {
    method: TemperatureRegulationMethod;
    config: PCMConfig | ResistiveHeatingConfig | ThermoelectricConfig;
    performance: ThermalPerformance;
  };

  /** Circuit traces */
  circuit_traces: CircuitTrace[];

  /** Electronic components */
  components: ElectronicComponent[];

  /** Power source */
  power_source: PowerSourceConfig;

  /** Power budget */
  power_budget: PowerBudget;

  /** Energy harvesting */
  energy_harvesting?: EnergyHarvesterConfig[];

  /** Energy storage */
  energy_storage?: EnergyStorageConfig;

  /** Communication */
  communication: DataTransmissionConfig;

  /** Breathability metrics */
  breathability: BreathabilityMetrics;

  /** Washability rating (number of cycles) */
  washability_cycles: number;

  /** IP rating */
  ip_rating: IPRating;

  /** Certifications */
  certifications: string[];

  /** Manufacturing date */
  manufacturing_date?: Date;

  /** Warranty period in months */
  warranty_months: number;
}

/**
 * Manufacturing specifications
 */
export interface ManufacturingSpecs {
  /** Circuit trace method */
  trace_method: CircuitTraceMethod;

  /** Printing parameters (if applicable) */
  printing?: {
    method: 'screen' | 'inkjet' | 'spray';
    ink: ConductiveInkProperties;
    layer_thickness_um: number;
    curing_temp_c: number;
    curing_time_min: number;
  };

  /** Embroidery parameters (if applicable) */
  embroidery?: {
    thread_type: ConductiveMaterial;
    stitch_density_per_mm: number;
    pattern_file: string;
  };

  /** Component attachment */
  attachment_methods: AttachmentMethod[];

  /** Encapsulation */
  encapsulation: {
    material: string;
    thickness_um: number;
    method: 'dip_coating' | 'spray_coating' | 'lamination' | 'overmolding';
  };

  /** Quality control checkpoints */
  qc_checkpoints: string[];
}

// ============================================================================
// Calculation and Analysis Types
// ============================================================================

/**
 * Conductivity calculation parameters
 */
export interface ConductivityCalculationParams {
  /** Base conductivity of material in S/m */
  base_conductivity_sm: number;

  /** Volume fraction of conductive material */
  fiber_fraction: number;

  /** Alignment factor (0-1) */
  alignment_factor: number;

  /** Contact factor (0-1) */
  contact_factor?: number;
}

/**
 * Sensor performance analysis
 */
export interface SensorPerformanceAnalysis {
  /** Sensor type */
  sensor_type: SensorType;

  /** Sensitivity */
  sensitivity: number;

  /** Signal-to-noise ratio in dB */
  snr_db: number;

  /** Accuracy */
  accuracy: number;

  /** Precision (repeatability) */
  precision: number;

  /** Drift over time */
  drift_per_hour: number;

  /** Linearity error percentage */
  linearity_error_pct: number;

  /** Cross-sensitivity to other parameters */
  cross_sensitivity: Record<string, number>;
}

/**
 * Energy harvesting performance
 */
export interface EnergyHarvestingPerformance {
  /** Harvester type */
  type: EnergyHarvestingTechnology;

  /** Average power output in μW */
  avg_power_output_uw: number;

  /** Peak power output in μW */
  peak_power_output_uw: number;

  /** Daily energy in mWh */
  daily_energy_mwh: number;

  /** Efficiency percentage */
  efficiency_pct: number;

  /** Power conditioning required */
  power_conditioning_required: boolean;
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  BaseFabricType,
  TextileConstruction,
  GarmentType,

  // Conductive materials
  ConductiveMaterial,
  ConductiveFiberConfig,
  ConductiveInkProperties,

  // Sensors
  SensorCategory,
  SensorType,
  SensorConfig,
  SensorPlacement,
  BodyLocation,

  // Temperature regulation
  TemperatureRegulationMethod,
  PCMConfig,
  ResistiveHeatingConfig,
  ThermoelectricConfig,
  ThermalPerformance,
  BreathabilityMetrics,

  // E-textile architecture
  CircuitTraceMethod,
  CircuitTrace,
  AttachmentMethod,
  ElectronicComponent,
  PowerSourceConfig,
  PowerBudget,

  // Energy harvesting
  EnergyHarvestingTechnology,
  EnergyHarvesterConfig,
  EnergyStorageConfig,

  // Health monitoring
  VitalSigns,
  ECGWaveform,
  ArrhythmiaType,
  ActivityType,
  ActivityData,
  SleepAnalysis,

  // Durability
  WashTestProtocol,
  DurabilityTestResults,
  IPRating,

  // Communication
  WirelessProtocol,
  BLEConfig,
  BLEService,
  BLECharacteristic,
  DataTransmissionConfig,

  // System
  SmartTextileSystem,
  ManufacturingSpecs,

  // Analysis
  ConductivityCalculationParams,
  SensorPerformanceAnalysis,
  EnergyHarvestingPerformance,
};
