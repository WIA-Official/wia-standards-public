/**
 * WIA-ENE-005 Solar Energy Standard - TypeScript Type Definitions
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (Benefit All Humanity)
 */

export interface Location {
  latitude: number;
  longitude: number;
  altitude: number;
  timezone: string;
}

export interface SiteProfile {
  siteId: string;
  name: string;
  location: Location;
  installedCapacity: number; // Watts
  commissionDate: string; // ISO 8601
  standard: 'WIA-ENE-005';
  complianceLevel: 1 | 2 | 3;
}

export interface ProductionData {
  timestamp: string; // ISO 8601
  power: number; // Watts
  energyToday: number; // kWh
  energyMonth: number; // kWh
  energyYear: number; // kWh
  energyLifetime: number; // kWh
}

export interface InverterMetrics {
  inverterId: string;
  timestamp: string;
  dcInput: {
    voltage: number; // Volts
    current: number; // Amps
    power: number; // Watts
  };
  acOutput: {
    voltage: number; // Volts
    current: number; // Amps
    power: number; // Watts
    frequency: number; // Hz
    powerFactor: number;
  };
  efficiency: number; // Percentage
  temperature: number; // Celsius
  status: 'PRODUCING' | 'IDLE' | 'FAULT' | 'MAINTENANCE';
  faultCode?: string;
}

export interface BatteryStatus {
  batteryId: string;
  timestamp: string;
  stateOfCharge: number; // Percentage
  capacity: number; // kWh
  usableCapacity: number; // kWh
  power: number; // kW (positive = discharging, negative = charging)
  voltage: number; // Volts
  current: number; // Amps
  temperature: number; // Celsius
  cycleCount: number;
  health: number; // Percentage (State of Health)
  mode: 'CHARGING' | 'DISCHARGING' | 'IDLE' | 'SELF_CONSUMPTION' | 'BACKUP';
}

export interface EnvironmentalData {
  timestamp: string;
  irradiance: number; // W/m²
  panelTemperature: number; // Celsius
  ambientTemperature: number; // Celsius
  windSpeed?: number; // m/s
  humidity?: number; // Percentage
}

export interface SystemStatus {
  systemId: string;
  timestamp: string;
  status: 'OPERATIONAL' | 'DEGRADED' | 'OFFLINE' | 'MAINTENANCE';
  availability: number; // Percentage
  faults: Array<{
    code: string;
    severity: 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
    message: string;
    timestamp: string;
  }>;
}

export interface PerformanceMetrics {
  efficiency: number; // Percentage
  performanceRatio: number; // Percentage
  capacityFactor: number; // Percentage
  degradationRate: number; // Percentage per year
  specificYield: number; // kWh/kWp/year
}

export interface APIResponse<T> {
  status: 'success' | 'error';
  timestamp: string;
  data?: T;
  error?: {
    code: string;
    message: string;
  };
  metadata: {
    responseTime: number; // milliseconds
    standard: 'WIA-ENE-005';
    version: string;
  };
}

export interface ClientConfig {
  apiKey: string;
  baseUrl: string;
  timeout?: number; // milliseconds
  standard?: 'WIA-ENE-005';
}

export interface ProductionHistory {
  systemId: string;
  start: string; // ISO 8601
  end: string; // ISO 8601
  resolution: '1m' | '5m' | '15m' | '1h' | '1d';
  data: Array<{
    timestamp: string;
    power?: number;
    energy?: number;
  }>;
}
