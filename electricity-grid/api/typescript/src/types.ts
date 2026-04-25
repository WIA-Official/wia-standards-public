/**
 * WIA-SOC-010 Electricity Grid Standard - TypeScript Type Definitions
 * Version: 1.0.0
 *
 * © 2025 SmileStory Inc. / WIA
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Configuration and Client Options
// ============================================================================

export interface WiaElectricityGridConfig {
  /** API host (e.g., 'api.grid-operator.com') */
  host: string;
  /** Bearer token for OAuth 2.0 authentication */
  bearerToken?: string;
  /** API key for simple authentication */
  apiKey?: string;
  /** Use HTTPS (default: true) */
  secure?: boolean;
  /** API version (default: 'v1') */
  version?: string;
  /** Request timeout in milliseconds (default: 30000) */
  timeout?: number;
}

export interface WebSocketOptions {
  /** Channels to subscribe to */
  channels: string[];
  /** Reconnect on disconnect (default: true) */
  autoReconnect?: boolean;
  /** Reconnect delay in milliseconds (default: 5000) */
  reconnectDelay?: number;
}

// ============================================================================
// System Information
// ============================================================================

export interface SystemInfo {
  '@context'?: string;
  '@type': 'SystemInfo';
  gridOperator: {
    name: string;
    id: string;
    region: string;
    timezone: string;
  };
  capabilities: {
    renewableIntegration: boolean;
    energyStorage: boolean;
    demandResponse: boolean;
    synchrophasors: boolean;
  };
  version: string;
  status: 'operational' | 'degraded' | 'outage';
}

// ============================================================================
// Grid Status
// ============================================================================

export interface GridStatus {
  '@type': 'GridStatus';
  id?: string;
  timestamp: string;
  region?: string;
  frequency: {
    value: number;
    unit: 'Hz';
    deviation: number;
  };
  voltage?: {
    nominal: number;
    actual: number;
    unit: 'kV';
  };
  currentLoad: {
    value: number;
    unit: 'MW';
  };
  capacity: {
    total: number;
    available: number;
    unit: 'MW';
  };
  loadFactor: number;
  status: 'normal' | 'warning' | 'critical' | 'emergency';
}

// ============================================================================
// Renewable Energy
// ============================================================================

export type RenewableSourceType = 'solar' | 'wind' | 'hydro' | 'biomass' | 'geothermal';

export interface RenewableSource {
  type: RenewableSourceType;
  capacity: number;
  generation: number;
  availabilityFactor: number;
  location?: {
    latitude: number;
    longitude: number;
    altitude?: number;
  };
  forecast?: RenewableForecast;
}

export interface RenewableGeneration {
  '@type': 'RenewableGeneration';
  id?: string;
  timestamp: string;
  sources: RenewableSource[];
  totalGeneration: number;
  penetrationRate: number;
}

export interface RenewableForecast {
  horizon: string;
  predictions: Array<{
    timestamp: string;
    expectedGeneration: number;
    confidenceInterval?: {
      lower: number;
      upper: number;
      level: number;
    };
  }>;
}

// ============================================================================
// Energy Storage
// ============================================================================

export type StorageTechnology =
  | 'lithium-ion'
  | 'flow-battery'
  | 'pumped-hydro'
  | 'compressed-air'
  | 'other';

export interface EnergyStorage {
  '@type': 'EnergyStorage';
  id: string;
  timestamp: string;
  technology: StorageTechnology;
  capacity: {
    power: number;
    energy: number;
    duration: number;
  };
  stateOfCharge: number;
  powerFlow: {
    value: number;
    direction: 'charging' | 'discharging' | 'idle';
  };
  efficiency: {
    roundTrip: number;
    charging: number;
    discharging: number;
  };
  cycleCount: number;
  healthStatus: {
    stateOfHealth: number;
    estimatedRemainingLife?: string;
  };
}

export interface StorageDispatch {
  storageId: string;
  power: number;
  duration: string;
  priority: 'low' | 'medium' | 'high' | 'critical';
}

export interface StorageDispatchResponse {
  dispatchId: string;
  status: 'accepted' | 'rejected' | 'pending';
  scheduledStart: string;
  estimatedRevenue?: number;
  message?: string;
}

// ============================================================================
// Demand Response
// ============================================================================

export type DRProgramType =
  | 'dynamic-pricing'
  | 'direct-control'
  | 'interruptible'
  | 'emergency';

export interface DemandResponseEvent {
  '@type': 'DemandResponseEvent';
  id: string;
  programType: DRProgramType;
  startTime: string;
  endTime: string;
  targetLoadReduction: number;
  actualLoadReduction?: number;
  participants: {
    enrolled: number;
    active: number;
    optedOut?: number;
  };
  pricing: {
    baseline: number;
    event: number;
    currency: string;
  };
  incentives?: {
    total: number;
    perParticipant: number;
    currency: string;
  };
  status: 'scheduled' | 'active' | 'completed' | 'canceled';
}

export interface CreateDREventRequest {
  programType: DRProgramType;
  startTime: string;
  endTime: string;
  targetReduction: number;
  pricing: {
    event: number;
    currency: string;
  };
}

// ============================================================================
// Power Quality
// ============================================================================

export interface PowerQualityMetrics {
  '@type': 'PowerQualityMetrics';
  id?: string;
  timestamp: string;
  location: string;
  frequency: {
    nominal: number;
    actual: number;
    deviation: number;
    stability?: number;
  };
  voltage: {
    nominal: number;
    actual: number;
    sags?: number;
    swells?: number;
    interruptions?: number;
  };
  harmonics: {
    thd: number;
    individual?: Array<{
      harmonic: number;
      magnitude: number;
    }>;
  };
  powerFactor: number;
  flicker?: {
    pst: number;
    plt: number;
  };
}

// ============================================================================
// Smart Meters
// ============================================================================

export interface MeterReading {
  '@type': 'MeterReading';
  meterId: string;
  timestamp: string;
  customerId?: string;
  location?: {
    address?: string;
    coordinates?: {
      latitude: number;
      longitude: number;
    };
  };
  consumption: {
    energy: number;
    interval: string;
    imported: number;
    exported?: number;
  };
  demand: {
    peak: number;
    average: number;
  };
  voltage?: {
    min: number;
    max: number;
    average: number;
  };
  events?: Array<{
    type: string;
    timestamp: string;
    duration?: string;
    severity: 'info' | 'warning' | 'critical';
  }>;
}

export interface MeterConsumptionQuery {
  from: string;
  to: string;
  interval?: '15min' | '1h' | '1d';
}

// ============================================================================
// Alerts and Notifications
// ============================================================================

export type AlertSeverity = 'info' | 'warning' | 'critical';
export type AlertStatus = 'new' | 'acknowledged' | 'resolved';

export interface Alert {
  id: string;
  severity: AlertSeverity;
  category: string;
  message: string;
  timestamp: string;
  status: AlertStatus;
  details?: Record<string, any>;
}

export interface AlertList {
  '@type': 'AlertList';
  alerts: Alert[];
  total?: number;
}

// ============================================================================
// WebSocket Events
// ============================================================================

export interface WebSocketMessage {
  channel: string;
  timestamp: string;
  data: any;
}

export interface WebSocketAuth {
  type: 'auth';
  token: string;
}

export interface WebSocketSubscribe {
  type: 'subscribe';
  channels: string[];
}

// ============================================================================
// API Response Types
// ============================================================================

export interface ApiResponse<T> {
  data: T;
  metadata?: {
    timestamp: string;
    requestId?: string;
  };
}

export interface ApiError {
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
    timestamp: string;
    requestId?: string;
  };
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    limit: number;
    offset: number;
    total: number;
    hasMore: boolean;
  };
}

// ============================================================================
// Query Parameters
// ============================================================================

export interface GridStatusQuery {
  zone?: string;
  detail?: 'basic' | 'full';
}

export interface RenewablesQuery {
  source?: RenewableSourceType | 'all';
  zone?: string;
}

export interface RenewableForecastQuery {
  horizon: '1h' | '6h' | '24h' | '7d';
  source?: RenewableSourceType;
  resolution?: '15min' | '1h';
}

export interface PowerQualityQuery {
  location?: string;
  metrics?: 'frequency' | 'voltage' | 'harmonics' | 'all';
}

export interface AlertQuery {
  severity?: AlertSeverity;
  category?: string;
  status?: AlertStatus;
}
