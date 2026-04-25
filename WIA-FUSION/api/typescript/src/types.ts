/**
 * WIA-FUSION TypeScript Type Definitions
 * Nuclear Fusion Energy Standard
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 * © 2025 WIA - World Certification Industry Association
 */

// ============================================================================
// Core Types
// ============================================================================

export type ReactorType = 'ITER' | 'KSTAR' | 'JET' | 'EAST' | 'SPARC' | 'W7X' | 'NIF' | 'custom';
export type QualityFlag = 'good' | 'suspect' | 'bad' | 'missing';
export type HeatingType = 'ohmic' | 'nbi' | 'icrh' | 'ecrh' | 'lhcd';
export type ProtocolType = 'startup' | 'steady' | 'shutdown' | 'emergency';
export type MitigationType = 'mgi' | 'spi' | 'runaway_suppression';

// ============================================================================
// Plasma State Schema
// ============================================================================

export interface Temperature {
  ion: number;      // keV
  electron: number; // keV
}

export interface Density {
  value: number;
  unit: '1e20/m3';
}

export interface TripleProduct {
  value: number;
  unit: 'keV·s·1e20/m3';
}

export interface CoreParameters {
  temperature_keV: Temperature;
  density_m3: Density;
  confinement_time_s: number;
  triple_product: TripleProduct;
}

export interface Performance {
  q_factor: number;
  fusion_power_mw: number;
  plasma_current_ma: number;
  beta_percent: number;
  bootstrap_fraction?: number;
}

export interface DisruptionRisk {
  value: number;     // 0-1
  confidence: number; // 0-1
}

export interface Stability {
  disruption_risk: DisruptionRisk;
  elm_frequency_hz: number;
  mhd_activity: string[]; // e.g., ["m=2,n=1", "m=3,n=2"]
  locked_mode?: boolean;
  vertical_displacement?: number;
}

export interface HeatingPower {
  ohmic: number;
  nbi: number;    // Neutral Beam Injection
  icrh: number;   // Ion Cyclotron Resonance Heating
  ecrh: number;   // Electron Cyclotron Resonance Heating
  lhcd?: number;  // Lower Hybrid Current Drive
}

export interface FuelMix {
  deuterium: number; // 0-1
  tritium: number;   // 0-1
}

export interface Control {
  heating_power_mw: HeatingPower;
  magnetic_field_t: number;
  divertor_heat_mw_m2: number;
  fuel_mix?: FuelMix;
}

export interface PlasmaState {
  shot_id: string;
  timestamp: string; // ISO8601
  reactor: ReactorType;
  core_parameters: CoreParameters;
  performance?: Performance;
  stability?: Stability;
  control?: Control;
}

// ============================================================================
// Energy Output Schema
// ============================================================================

export interface NeutronWallLoading {
  value: number;
  unit: 'MW/m2';
}

export interface EnergyOutput {
  gross_fusion_power_mw: number;
  thermal_power_mw: number;
  net_electric_power_mw: number;
  plant_efficiency_percent: number;
  availability_factor: number;
  tritium_breeding_ratio: number;
  neutron_wall_loading?: NeutronWallLoading;
}

// ============================================================================
// Diagnostic Data Schema
// ============================================================================

export interface DiagnosticData {
  diagnostic_id: string;
  timestamp: string;
  sampling_rate_hz: number;
  data: number[];
  radial_positions?: number[];
  units: string;
  calibration_version?: string;
  quality_flag: QualityFlag;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface RecordStateRequest {
  plasma_state: PlasmaState;
}

export interface RecordStateResponse {
  success: boolean;
  shot_id: string;
  recorded_at: string;
}

export interface DisruptionPredictionRequest {
  plasma_state: {
    temperature_keV: number;
    density_m3: number;
    plasma_current_ma: number;
    beta_percent: number;
    li?: number;
    q95?: number;
  };
  diagnostics?: {
    mhd_amplitude?: number;
    locked_mode_indicator?: boolean;
    radiation_peaking?: number;
  };
}

export interface RiskFactor {
  factor: string;
  contribution: number;
}

export interface RecommendedAction {
  action: string;
  priority: 'low' | 'medium' | 'high' | 'critical';
}

export interface DisruptionPredictionResponse {
  prediction: {
    disruption_probability: number;
    time_to_disruption_s: number | null;
    confidence: number;
    risk_factors: RiskFactor[];
    recommended_actions: RecommendedAction[];
  };
}

export interface ControlOptimizeRequest {
  current_state: {
    temperature_keV: number;
    density_m3?: number;
    confinement_time_s?: number;
    heating_power_mw: Partial<HeatingPower>;
  };
  target: {
    q_factor: number;
    steady_state_duration_s?: number;
  };
  constraints?: {
    max_heating_power_mw?: number;
    max_divertor_heat_mw_m2?: number;
  };
}

export interface ControlOptimizeResponse {
  optimized_control: {
    heating_power_mw: HeatingPower;
    plasma_shape?: {
      elongation: number;
      triangularity: number;
    };
    predicted_performance: {
      q_factor: number;
      confinement_improvement?: string;
    };
    confidence: number;
  };
}

export interface EnergyBalanceResponse {
  energy_balance: {
    input_power_mw: HeatingPower & { total: number };
    output_power_mw: {
      fusion: number;
      radiation: number;
      conduction: number;
      convection: number;
    };
    q_factor: number;
    energy_confinement_time_s: number;
    h_factor: number;
  };
}

// ============================================================================
// WebSocket Types
// ============================================================================

export interface StreamSubscription {
  action: 'subscribe' | 'unsubscribe';
  channels: ('plasma_state' | 'stability' | 'control' | 'diagnostics')[];
  reactor?: ReactorType;
  sample_rate_hz?: number;
}

export interface StreamMessage {
  channel: string;
  timestamp: string;
  data: Partial<PlasmaState>;
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface FusionClientConfig {
  apiKey: string;
  baseUrl?: string;
  reactor?: ReactorType;
  timeout?: number;
}

// ============================================================================
// Error Types
// ============================================================================

export interface FusionError {
  code: string;
  message: string;
  details?: { field: string; error: string }[];
  request_id?: string;
}

export class FusionAPIError extends Error {
  code: string;
  details?: { field: string; error: string }[];
  request_id?: string;

  constructor(error: FusionError) {
    super(error.message);
    this.name = 'FusionAPIError';
    this.code = error.code;
    this.details = error.details;
    this.request_id = error.request_id;
  }
}
