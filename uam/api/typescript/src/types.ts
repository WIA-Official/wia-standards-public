/**
 * WIA Urban Air Mobility Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘익人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type VehicleType =
  | 'multicopter'
  | 'lift-cruise'
  | 'vectored-thrust'
  | 'powered-lift'
  | 'tilt-wing'
  | 'tilt-rotor';

export type VehicleStatus =
  | 'parked'
  | 'preflight'
  | 'takeoff'
  | 'cruise'
  | 'approach'
  | 'landing'
  | 'charging'
  | 'maintenance';

export type VertiportStatus =
  | 'operational'
  | 'busy'
  | 'closed'
  | 'emergency';

export interface Position {
  latitude: number;
  longitude: number;
  altitude: number;
}

export interface Capacity {
  passengers: number;
  payload: number;        // kg
  maxTakeoffWeight: number;  // kg
}

export interface BatterySystem {
  charge: number;         // percentage
  voltage: number;        // volts
  current: number;        // amps
  temperature: number;    // celsius
  capacity: number;       // kWh
  remainingRange: number; // km
}

export interface Telemetry {
  position: Position;
  speed: number;          // km/h
  heading: number;        // degrees
  verticalSpeed: number;  // m/s
  battery: BatterySystem;
  timestamp: string;
}

export interface Vertiport {
  id: string;
  name: string;
  position: Position;
  status: VertiportStatus;
  padsAvailable: number;
  padsTotal: number;
  chargingStations: number;
  facilities: {
    passengerWaiting: boolean;
    weatherStation: boolean;
    emergencyServices: boolean;
  };
}

export interface Route {
  origin: Vertiport;
  destination: Vertiport;
  waypoints: Position[];
  distance: number;          // km
  estimatedTime: number;     // minutes
  energyRequired: number;    // kWh
  corridor: string;
}

export interface UAMVehicle {
  id: string;
  type: VehicleType;
  status: VehicleStatus;
  manufacturer: string;
  model: string;
  registrationNumber: string;
  capacity: Capacity;
  telemetry?: Telemetry;
  currentRoute?: Route;
  operator: string;
  certification: {
    standard: string;
    authority: string;
    validUntil: string;
  };
  metadata?: Record<string, any>;
}

export interface FlightPlan {
  vehicleId: string;
  route: Route;
  departureTime: string;
  passengers: number;
  approved: boolean;
  flightNumber?: string;
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
