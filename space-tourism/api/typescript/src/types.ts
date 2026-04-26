/**
 * WIA-SPACE-014: Space Tourism Standard - TypeScript Types
 */

export enum FlightType {
  SUBORBITAL = 'SUBORBITAL',
  ORBITAL = 'ORBITAL',
  SPACE_STATION = 'SPACE_STATION',
  LUNAR_FLYBY = 'LUNAR_FLYBY'
}

export enum OperatorName {
  VIRGIN_GALACTIC = 'VIRGIN_GALACTIC',
  BLUE_ORIGIN = 'BLUE_ORIGIN',
  SPACEX = 'SPACEX',
  AXIOM_SPACE = 'AXIOM_SPACE'
}

export interface Tourist {
  name: string;
  age: number;
  medical_clearance: boolean;
  country: string;
}

export interface Flight {
  type: FlightType;
  operator: OperatorName | string;
  date: string;
  duration_hours: number;
  altitude_km: number;
}

export interface Training {
  duration_days: number;
  completed: boolean;
  g_force_tolerance: number;
}

export interface Booking {
  id: string;
  tourist: Tourist;
  flight: Flight;
  training: Training;
  cost_usd: number;
}

export interface APIResponse<T> {
  status: 'success' | 'error';
  data?: T;
  error?: string;
  timestamp: string;
}
