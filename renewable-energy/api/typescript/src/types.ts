/**
 * WIA-ENE-004 Renewable Energy Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @copyright 2025 SmileStory Inc. / WIA
 *
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ==================== Energy Source Types ====================

export type EnergySourceType =
  | 'SOLAR-PV'      // Solar Photovoltaic
  | 'SOLAR-TH'      // Solar Thermal
  | 'WIND-ON'       // Wind Onshore
  | 'WIND-OFF'      // Wind Offshore
  | 'HYDRO'         // Hydroelectric
  | 'GEO'           // Geothermal
  | 'BIO'           // Biomass
  | 'TIDAL';        // Tidal Energy

export type OperationalState =
  | 'ACTIVE'
  | 'INACTIVE'
  | 'MAINTENANCE'
  | 'FAULT'
  | 'OFFLINE';

export type HealthStatus =
  | 'HEALTHY'
  | 'DEGRADED'
  | 'CRITICAL'
  | 'UNKNOWN';

export type AlertSeverity =
  | 'CRITICAL'
  | 'HIGH'
  | 'MEDIUM'
  | 'LOW'
  | 'INFO';

// ==================== Data Models ====================

export interface Location {
  latitude: number;
  longitude: number;
  elevation: number;
  address?: string;
  timezone: string;
}

export interface Capacity {
  value: number;
  unit: 'kW' | 'MW' | 'GW';
}

export interface EnergySourceMetadata {
  manufacturer: string;
  model: string;
  serialNumber: string;
  installDate: string; // ISO 8601
  warrantyExpiry?: string;
  certifications: string[];
}

export interface EnergySource {
  sourceId: string;
  sourceType: EnergySourceType;
  name: string;
  ratedCapacity: Capacity;
  location: Location;
  operator: string;
  metadata: EnergySourceMetadata;
  status: OperationalState;
  healthStatus: HealthStatus;
  createdAt: string;
  updatedAt: string;
}

export interface ProductionValue {
  value: number;
  unit: string;
}

export interface ProductionData {
  timestamp: string;
  sourceId: string;
  production: {
    instantaneous: ProductionValue;
    cumulative: {
      today: number;
      thisMonth: number;
      lifetime: number;
      unit: string;
    };
  };
  efficiency?: {
    current: number;
    average24h: number;
    unit: 'percent';
  };
  environmentalConditions?: {
    temperature?: number;
    humidity?: number;
    windSpeed?: number;
    solarIrradiance?: number;
  };
  quality?: {
    voltage: number;
    frequency: number;
    powerFactor: number;
    harmonicDistortion: number;
  };
}

export interface Alert {
  alertId: string;
  sourceId: string;
  severity: AlertSeverity;
  type: string;
  title: string;
  description: string;
  timestamp: string;
  metadata?: Record<string, any>;
  actions?: AlertAction[];
  status: 'OPEN' | 'ACKNOWLEDGED' | 'RESOLVED' | 'CLOSED';
  acknowledgedBy?: string;
  acknowledgedAt?: string;
  resolvedAt?: string;
}

export interface AlertAction {
  action: string;
  priority: 'HIGH' | 'MEDIUM' | 'LOW';
  estimatedDuration?: string;
}

export interface SystemStatus {
  sourceId: string;
  timestamp: string;
  operationalState: OperationalState;
  healthStatus: HealthStatus;
  alerts: Alert[];
  components?: ComponentStatus[];
  maintenanceStatus?: MaintenanceStatus;
}

export interface ComponentStatus {
  componentId: string;
  status: 'OPERATIONAL' | 'DEGRADED' | 'FAULT';
  metrics: Record<string, number>;
}

export interface MaintenanceStatus {
  lastMaintenance: string;
  nextScheduled: string;
  hoursUntilService: number;
}

// ==================== API Request/Response Types ====================

export interface ListSourcesParams {
  type?: EnergySourceType;
  status?: OperationalState;
  limit?: number;
  offset?: number;
  sortBy?: 'name' | 'capacity' | 'installDate';
  sortOrder?: 'asc' | 'desc';
}

export interface ListSourcesResponse {
  sources: EnergySource[];
  total: number;
  limit: number;
  offset: number;
}

export interface GetProductionParams {
  period?: string; // e.g., '24h', '7d', '30d'
  resolution?: string; // e.g., '1min', '15min', '1hour'
  start?: string; // ISO 8601
  end?: string; // ISO 8601
}

export interface GetProductionResponse {
  sourceId: string;
  period: {
    start: string;
    end: string;
    resolution: string;
  };
  dataPoints: ProductionData[];
  summary: {
    total: number;
    average: number;
    peak: number;
    efficiency?: number;
  };
}

export interface CreateAlertRuleParams {
  sourceId: string;
  name: string;
  condition: {
    metric: string;
    operator: 'greater_than' | 'less_than' | 'equals' | 'not_equals';
    threshold: number;
    duration?: string;
  };
  actions: AlertAction[];
}

// ==================== Authentication ====================

export interface AuthToken {
  access_token: string;
  token_type: 'Bearer';
  expires_in: number;
  refresh_token?: string;
  scope?: string;
}

export interface AuthCredentials {
  clientId: string;
  clientSecret: string;
  grantType?: 'client_credentials' | 'password' | 'refresh_token';
  scope?: string;
}

// ==================== Configuration ====================

export interface ClientConfig {
  apiEndpoint: string;
  apiKey?: string;
  apiSecret?: string;
  version?: string;
  timeout?: number;
  retries?: number;
  logLevel?: 'debug' | 'info' | 'warn' | 'error';
}

// ==================== Errors ====================

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
  documentation?: string;
  requestId?: string;
}

// ==================== Utility Types ====================

export type TimeRange = {
  start: string;
  end: string;
};

export type Pagination = {
  limit: number;
  offset: number;
  total: number;
};

export type SortOptions = {
  sortBy: string;
  sortOrder: 'asc' | 'desc';
};
