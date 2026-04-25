/**
 * WIA Satellite Technology Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type OrbitType = 'LEO' | 'MEO' | 'GEO' | 'HEO' | 'SSO';
export type SatelliteStatus = 'active' | 'standby' | 'safe_mode' | 'decommissioned';
export type SubsystemStatus = 'nominal' | 'degraded' | 'failed';

export interface OrbitalParameters {
  semi_major_axis_km: number;
  eccentricity: number;
  inclination_deg: number;
  raan_deg: number;
  argument_of_perigee_deg: number;
  true_anomaly_deg: number;
}

export interface SatellitePosition {
  latitude_deg: number;
  longitude_deg: number;
  altitude_km: number;
  velocity_km_s: number;
  timestamp: string;
}

export interface PowerSubsystem {
  solar_array_output_w: number;
  battery_capacity_wh: number;
  battery_charge_percent: number;
  power_consumption_w: number;
  status: SubsystemStatus;
}

export interface ThermalSubsystem {
  temperature_c: number;
  heater_status: boolean;
  radiator_efficiency_percent: number;
  status: SubsystemStatus;
}

export interface CommunicationSubsystem {
  frequency_band: string;
  uplink_rate_mbps: number;
  downlink_rate_mbps: number;
  signal_strength_dbm: number;
  status: SubsystemStatus;
}

export interface SatelliteData {
  id: string;
  name: string;
  orbit_type: OrbitType;
  status: SatelliteStatus;
  orbital_parameters: OrbitalParameters;
  position?: SatellitePosition;
  power?: PowerSubsystem;
  thermal?: ThermalSubsystem;
  communication?: CommunicationSubsystem;
  metadata?: Record<string, any>;
}

export interface WIAConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
}

export interface LinkBudget {
  transmit_power_dbm: number;
  transmit_gain_dbi: number;
  path_loss_db: number;
  receive_gain_dbi: number;
  received_power_dbm: number;
  noise_power_dbm: number;
  snr_db: number;
}
