/**
 * WIA-MED-011 Smart Pill - Type Definitions
 * @version 1.0.0
 */

export type PillStatus = 'active' | 'dissolved' | 'excreted';
export type GILocation = 'stomach' | 'small_intestine' | 'large_intestine' | 'unknown';

export interface SmartPill {
  id: string;
  patientId: string;
  ingestedAt: string;
  status: PillStatus;
  sensors: SensorData[];
  location: GILocation;
  batteryLevel?: number;
}

export interface SensorData {
  timestamp: string;
  pH: number;
  temperature: number;
  pressure?: number;
  location: GILocation;
}

export interface TransitMetrics {
  gastricEmptyingTime: number;
  smallIntestineTransit: number;
  largeIntestineTransit: number;
  totalTransitTime: number;
}

export interface SDKConfig {
  apiKey: string;
  environment: 'production' | 'staging' | 'development';
  baseUrl?: string;
}
