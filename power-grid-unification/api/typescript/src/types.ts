/**
 * WIA-UNI-007 Power Grid Unification Standard
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @description Type definitions for Phase 1 data formats
 */

/**
 * Base WIA entity with JSON-LD context
 */
export interface WIAEntity {
  '@context': string;
  '@type': string;
  id: string;
  metadata?: Metadata;
}

/**
 * Metadata for tracking data provenance
 */
export interface Metadata {
  createdAt: string;
  updatedAt?: string;
  version?: string;
}

/**
 * Geographic location with coordinates
 */
export interface Location {
  latitude: number;
  longitude: number;
  elevation?: number;
  address?: string;
  region: 'north' | 'south' | 'dmz';
}

/**
 * Power capacity specification
 */
export interface Capacity {
  value: number;
  unit: 'MW' | 'GW';
  ratedVoltage?: {
    value: number;
    unit: 'kV';
  };
}

/**
 * Grid node types
 */
export type NodeType =
  | 'substation'
  | 'hvdc-terminal'
  | 'solar-farm'
  | 'wind-farm'
  | 'hydro-plant'
  | 'storage'
  | 'transmission-line';

/**
 * Operational status
 */
export type OperationalStatus =
  | 'operational'
  | 'planned'
  | 'under-construction'
  | 'maintenance'
  | 'decommissioned';

/**
 * Certification status
 */
export type CertificationStatus = 'certified' | 'pending' | 'expired';

/**
 * Power Grid Node
 */
export interface PowerGridNode extends WIAEntity {
  '@type': 'PowerGridNode';
  nodeType: NodeType;
  name: string;
  location: Location;
  capacity: Capacity;
  operationalStatus: OperationalStatus;
  grid: {
    operator: string;
    region: string;
    connectionType: 'AC' | 'HVDC';
  };
  standards: string[];
  certificationStatus: CertificationStatus;
}

/**
 * Power Flow Measurement
 */
export interface PowerFlowMeasurement extends WIAEntity {
  '@type': 'PowerFlowMeasurement';
  nodeId: string;
  timestamp: string;
  activePower: {
    value: number;
    unit: 'MW';
    direction: 'import' | 'export' | 'balanced';
  };
  reactivePower: {
    value: number;
    unit: 'MVAr';
  };
  voltage: {
    value: number;
    unit: 'kV';
  };
  frequency: {
    value: number;
    unit: 'Hz';
  };
  lineLoading?: {
    percentage: number;
    temperature: number;
  };
}

/**
 * Renewable energy source types
 */
export type RenewableSourceType = 'solar' | 'wind' | 'hydro' | 'tidal' | 'geothermal';

/**
 * Renewable Energy Metrics
 */
export interface RenewableEnergyMetrics extends WIAEntity {
  '@type': 'RenewableEnergyMetrics';
  sourceId: string;
  sourceType: RenewableSourceType;
  timestamp: string;
  generation: {
    current: number;
    forecast24h?: number[];
    capacityFactor: number;
  };
  environmental: {
    carbonAvoided: number;
    renewablePercentage: number;
  };
  weather?: {
    windSpeed?: number;
    solarIrradiance?: number;
    temperature?: number;
  };
}

/**
 * Energy Trading Order
 */
export interface TradingOrder {
  orderType: 'buy' | 'sell';
  quantity: number;
  unit: 'MWh' | 'GWh';
  price: number;
  currency: string;
  deliveryTime: string;
  source?: 'renewable' | 'conventional';
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL: string;
  accessToken: string;
  region: 'kr-south' | 'kr-north' | 'unified';
  timeout?: number;
}

/**
 * API Response wrapper
 */
export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
  };
}
