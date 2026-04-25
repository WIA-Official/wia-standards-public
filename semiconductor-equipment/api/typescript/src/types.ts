/**
 * WIA-SEMI-019 Semiconductor Equipment Standard - TypeScript Types
 * Version: 1.0.0
 *
 * 弘益人間 · Benefit All Humanity
 */

// ==================== Equipment Specification ====================

export interface EquipmentSpecification {
  standard: 'WIA-SEMI-019';
  version: string;
  last_updated: string;
  equipment: EquipmentInfo;
  capabilities: EquipmentCapabilities;
  parameters: ParameterCounts;
  interfaces: EquipmentInterfaces;
  certification?: CertificationInfo;
}

export interface EquipmentInfo {
  manufacturer: string;
  model: string;
  serial_number?: string;
  type: EquipmentType;
  subtype?: string;
  installation_date?: string;
  wafer_size_mm: number[];
  process_node_nm: number[];
  chambers: number;
  load_ports: number;
}

export type EquipmentType =
  | 'lithography'
  | 'etch'
  | 'deposition'
  | 'inspection'
  | 'cmp'
  | 'implant'
  | 'cleaning'
  | 'metrology'
  | 'anneal'
  | 'photolithography';

export interface EquipmentCapabilities {
  technology?: string;
  throughput_wph: number;
  max_wafer_size_mm?: number;
  min_feature_size_nm?: number;
  uniformity_percent?: number;
  process_temperature_range_celsius?: {
    min: number;
    max: number;
  };
  materials_supported?: string[];
  automation_level?: string;
}

export interface ParameterCounts {
  process: number;
  sensor: number;
  control: number;
  metrology?: number;
  status: number;
}

export interface EquipmentInterfaces {
  secs_gem: boolean;
  rest_api: boolean;
  websocket: boolean;
  e84: boolean;
}

export interface CertificationInfo {
  wia_level: 'Bronze' | 'Silver' | 'Gold' | 'Platinum';
  certified_date: string;
  certificate_id: string;
  valid_until?: string;
}

// ==================== Equipment Status ====================

export interface EquipmentStatus {
  equipment_id: string;
  timestamp: string;
  state: EquipmentState;
  substate?: string;
  previous_state?: EquipmentState;
  state_duration_seconds: number;
  control_mode: 'LOCAL' | 'REMOTE' | 'OFFLINE';
  processing?: ProcessingInfo;
  health: HealthInfo;
}

export type EquipmentState =
  | 'IDLE'
  | 'SETUP'
  | 'READY'
  | 'EXECUTING'
  | 'PAUSED'
  | 'ALARM'
  | 'MAINTENANCE'
  | 'OFFLINE';

export interface ProcessingInfo {
  wafer_id: string;
  lot_id: string;
  recipe_id: string;
  step: number;
  total_steps: number;
  estimated_completion: string;
}

export interface HealthInfo {
  status: 'NORMAL' | 'WARNING' | 'ALARM' | 'ERROR';
  warnings: number;
  alarms: number;
  uptime_hours: number;
}

// ==================== Parameters ====================

export interface Parameter {
  parameter_id: string;
  value: number | string | boolean;
  timestamp: string;
  quality: QualityInfo;
}

export interface QualityInfo {
  status: 'GOOD' | 'UNCERTAIN' | 'BAD' | 'UNKNOWN';
  confidence: number;
  sensor_health: 'NORMAL' | 'DEGRADED' | 'FAILED';
  calibration_date?: string;
  calibration_due?: string;
  out_of_spec: boolean;
  error_code?: string;
}

export interface ParameterDefinition {
  parameter_id: string;
  description: string;
  unit: string;
  data_type: 'float' | 'integer' | 'boolean' | 'string' | 'timestamp';
  precision?: number;
  range?: {
    min: number;
    max: number;
    normal_min?: number;
    normal_max?: number;
  };
  category: 'process' | 'sensor' | 'control' | 'metrology' | 'status';
  update_frequency_hz?: number;
  secs_variable_id?: number;
}

// ==================== Alarms ====================

export interface Alarm {
  alarm_id: string;
  code: string;
  severity: 'INFO' | 'WARNING' | 'ALARM' | 'ERROR' | 'FATAL';
  message: string;
  timestamp: string;
  parameter?: string;
  current_value?: number;
  setpoint?: number;
  acknowledged: boolean;
  acknowledged_by?: string;
  acknowledged_at?: string;
}

// ==================== Events ====================

export interface Event {
  event_id: string;
  type: string;
  timestamp: string;
  wafer_id?: string;
  lot_id?: string;
  recipe_id?: string;
  result?: 'SUCCESS' | 'FAILURE' | 'ABORTED';
  duration_seconds?: number;
  metadata?: Record<string, unknown>;
}

// ==================== Recipes ====================

export interface Recipe {
  recipe_id: string;
  version: string;
  description?: string;
  steps: RecipeStep[];
  checksum: string;
}

export interface RecipeStep {
  step_number: number;
  step_name: string;
  duration_seconds: number;
  parameters: Record<string, number | string | boolean>;
}

// ==================== Commands ====================

export interface Command {
  command: CommandType;
  parameters?: Record<string, unknown>;
  correlation_id?: string;
}

export type CommandType =
  | 'START'
  | 'STOP'
  | 'PAUSE'
  | 'RESUME'
  | 'ABORT'
  | 'RESET'
  | 'HOME';

export interface CommandResponse {
  command_id: string;
  status: 'ACCEPTED' | 'REJECTED' | 'EXECUTING' | 'COMPLETED' | 'FAILED';
  estimated_completion?: string;
  message?: string;
}

// ==================== Lithography Specific ====================

export interface LithographyCapabilities {
  exposure: {
    wavelength_nm: number;
    dose_mj_cm2: number;
    exposure_time_ms: number;
    numerical_aperture: number;
    slit_width_mm: number;
    scan_speed_mm_s: number;
  };
  overlay: {
    overlay_x_nm: number;
    overlay_y_nm: number;
    overlay_3sigma_nm: number;
    alignment_marks: number;
    alignment_residual_nm: number;
  };
  focus: {
    focus_offset_nm: number;
    focus_depth_nm: number;
    tilt_x_nm: number;
    tilt_y_nm: number;
    leveling_residual_nm: number;
  };
  reticle: {
    reticle_id: string;
    magnification: number;
    pellicle_installed: boolean;
    inspection_date?: string;
  };
}

// ==================== Etch Specific ====================

export interface EtchCapabilities {
  plasma: {
    rf_power_top_watts: number;
    rf_power_bottom_watts: number;
    rf_frequency_mhz: number;
    dc_bias_volts: number;
    plasma_density_cm3: number;
  };
  process: {
    etch_rate_nm_min: number;
    selectivity_ratio: number;
    uniformity_percent: number;
    aspect_ratio_max: number;
    sidewall_angle_degrees: number;
  };
  gases: {
    gas_chemistry: string;
    total_flow_sccm: number;
  };
}

// ==================== Deposition Specific ====================

export interface DepositionCapabilities {
  film: {
    material: string;
    target_thickness_nm: number;
    measured_thickness_nm: number;
    uniformity_percent: number;
    step_coverage_percent: number;
    stress_mpa: number;
    refractive_index?: number;
  };
  process: {
    deposition_rate_nm_min: number;
    temperature_celsius: number;
    pressure_pascal: number;
    method: 'CVD' | 'ALD' | 'PVD' | 'PECVD' | 'LPCVD' | 'MOCVD';
  };
}

// ==================== Inspection Specific ====================

export interface InspectionCapabilities {
  method: 'ebeam' | 'optical' | 'xray' | 'afm';
  resolution_nm: number;
  defect_sensitivity_nm: number;
  throughput_wph: number;
  inspection_area_percent: number;
  defects: {
    total_count: number;
    critical_count: number;
    defect_types: Record<string, number>;
  };
  classification: {
    nuisance_percent: number;
    systematic_percent: number;
    random_percent: number;
  };
}

// ==================== WebSocket Messages ====================

export interface WebSocketSubscribeMessage {
  action: 'subscribe' | 'unsubscribe' | 'close';
  parameters?: string[];
  frequency_hz?: number;
  format?: 'json' | 'binary' | 'msgpack';
  compression?: 'gzip' | 'deflate';
}

export interface WebSocketDataMessage {
  timestamp: string;
  sequence: number;
  data: Record<string, number | string | boolean>;
}

// ==================== API Configuration ====================

export interface WIAConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
  retries?: number;
}

// ==================== Error Types ====================

export interface WIAError {
  error: {
    code: string;
    message: string;
    timestamp: string;
    request_id?: string;
  };
}
