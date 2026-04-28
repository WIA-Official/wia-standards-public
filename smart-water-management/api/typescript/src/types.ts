/**
 * WIA-CITY-020: Smart Water Management Types
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

export type CertificationLevel = 'bronze' | 'silver' | 'gold' | 'platinum';
export type ControlMode = 'auto' | 'manual' | 'scheduled' | 'occupancy';
export type SystemStatus = 'active' | 'standby' | 'fault' | 'offline';

export interface LightingConfig {
  mode: ControlMode;
  brightness: number;
  colorTemperature?: number;
  certificationLevel: CertificationLevel;
  zones?: ZoneConfig[];
}

export interface ZoneConfig {
  id?: string;
  name: string;
  brightness: number;
  occupancySensing: boolean;
  daylightHarvesting: boolean;
}

export interface SystemMetrics {
  energySavings: number;
  efficacy: number;
  brightness: number;
  colorTemperature: number;
  occupancy: number;
  illuminance: number;
}
