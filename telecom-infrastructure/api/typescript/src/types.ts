// WIA-SOC-012 Telecommunications Infrastructure Standard - TypeScript Types
// Version: 1.0.0

export interface Infrastructure {
  infra_id: string;
  timestamp: string;
  version: string;
  type: InfrastructureType;
  location: Location;
  specifications: any;
  telemetry?: Telemetry;
  metadata?: Metadata;
}

export type InfrastructureType = 'cell_tower' | 'fiber_node' | 'data_center' | 'edge_node' | 'backhaul';

export interface Location {
  latitude: number;
  longitude: number;
  altitude?: number;
  address?: string;
  country_code?: string;
}

export interface Telemetry {
  performance?: Performance;
  power?: Power;
  environmental?: Environmental;
  signal_quality?: SignalQuality;
}

export interface Performance {
  throughput_mbps?: number;
  latency_ms?: number;
  packet_loss_percent?: number;
  active_users?: number;
}

export interface Power {
  consumption_watts?: number;
  battery_level_percent?: number;
  generator_status?: 'off' | 'standby' | 'running';
}

export interface Environmental {
  temperature_celsius?: number;
  humidity_percent?: number;
  wind_speed_mps?: number;
}

export interface SignalQuality {
  rsrp_dbm?: number;
  rsrq_db?: number;
  sinr_db?: number;
}

export interface Metadata {
  owner?: string;
  operator_id?: string;
  deployment_date?: string;
  status?: 'operational' | 'maintenance' | 'offline' | 'error';
}

export interface ClientConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}
