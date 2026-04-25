/**
 * WIA Satellite Communication Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type FrequencyBand = 'C-band' | 'Ku-band' | 'Ka-band' | 'X-band' | 'S-band';
export type ModulationType = 'QPSK' | '8PSK' | '16APSK' | '32APSK';
export type AccessMethod = 'FDMA' | 'TDMA' | 'CDMA' | 'MF-TDMA';

export interface LinkBudget {
  transmit_power_dbm: number;
  transmit_gain_dbi: number;
  path_loss_db: number;
  receive_gain_dbi: number;
  received_power_dbm: number;
  noise_power_dbm: number;
  cnr_db: number;
}

export interface CommunicationParameters {
  frequency_band: FrequencyBand;
  uplink_frequency_ghz: number;
  downlink_frequency_ghz: number;
  data_rate_mbps: number;
  modulation: ModulationType;
  access_method: AccessMethod;
}

export interface SatelliteLink {
  id: string;
  parameters: CommunicationParameters;
  link_budget: LinkBudget;
  quality_of_service: {
    latency_ms: number;
    packet_loss_percent: number;
    availability_percent: number;
  };
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
