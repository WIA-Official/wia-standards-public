/**
 * WIA Drone (UAV) Standard - TypeScript Types
 * Version: 1.0
 * Philosophy: 弘益人間 - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

export type DroneType =
  | 'quadcopter'
  | 'hexacopter'
  | 'octocopter'
  | 'fixed-wing'
  | 'vtol'
  | 'hybrid';

export type DroneStatus =
  | 'ready'
  | 'preflight'
  | 'flying'
  | 'landing'
  | 'charging'
  | 'maintenance'
  | 'emergency';

export type FlightMode =
  | 'MANUAL'
  | 'STABILIZE'
  | 'AUTO'
  | 'GUIDED'
  | 'LOITER'
  | 'RTL'
  | 'LAND';

export type GPSFixType =
  | 'NO_GPS'
  | 'NO_FIX'
  | '2D_FIX'
  | '3D_FIX'
  | 'DGPS'
  | 'RTK_FLOAT'
  | 'RTK_FIXED';

export interface Position {
  latitude: number;   // degrees
  longitude: number;  // degrees
  altitude: number;   // meters (MSL)
  relativeAltitude?: number;  // meters (AGL)
}

export interface Attitude {
  roll: number;   // degrees
  pitch: number;  // degrees
  yaw: number;    // degrees
}

export interface Velocity {
  groundSpeed: number;    // m/s
  airSpeed?: number;      // m/s
  verticalSpeed: number;  // m/s
}

export interface BatteryStatus {
  voltage: number;      // volts
  current: number;      // amps
  remaining: number;    // percentage (0-100)
  capacity: number;     // mAh
  temperature?: number; // celsius
}

export interface GPSStatus {
  fixType: GPSFixType;
  satellites: number;
  hdop: number;        // Horizontal Dilution of Precision
  vdop: number;        // Vertical Dilution of Precision
}

export interface Telemetry {
  timestamp: string;
  position: Position;
  attitude: Attitude;
  velocity: Velocity;
  battery: BatteryStatus;
  gps: GPSStatus;
}

export interface FlightController {
  type: string;          // e.g., "Pixhawk", "ArduPilot", "DJI"
  version: string;
  mode: FlightMode;
  armed: boolean;
  ekfOk: boolean;       // Extended Kalman Filter status
  homePosition?: Position;
}

export interface Payload {
  type: string;         // e.g., "camera", "lidar", "thermal"
  weight: number;       // kg
  powerConsumption: number;  // watts
  active: boolean;
  data?: any;
}

export interface Mission {
  id: string;
  name: string;
  waypoints: Waypoint[];
  totalDistance: number;     // km
  estimatedDuration: number; // minutes
  status: 'planned' | 'active' | 'completed' | 'aborted';
}

export interface Waypoint {
  sequence: number;
  latitude: number;
  longitude: number;
  altitude: number;
  command: string;      // e.g., "WAYPOINT", "TAKEOFF", "LAND"
  param1?: number;
  param2?: number;
  param3?: number;
  param4?: number;
}

export interface Drone {
  id: string;
  type: DroneType;
  status: DroneStatus;
  manufacturer: string;
  model: string;
  serialNumber: string;
  registrationNumber: string;
  maxTakeoffWeight: number;  // kg
  maxFlightTime: number;     // minutes
  maxRange: number;          // km
  telemetry?: Telemetry;
  flightController: FlightController;
  payloads: Payload[];
  currentMission?: Mission;
  operator: string;
  metadata?: Record<string, any>;
}

export interface FlightPlan {
  droneId: string;
  departureTime: string;
  arrivalTime: string;
  route: Position[];
  maxAltitude: number;
  minAltitude: number;
  purpose: string;
  approved: boolean;
  approvalAuthority?: string;
}

export interface RemoteID {
  uasId: string;
  operatorId: string;
  operatorLocation: Position;
  timestamp: string;
  emergencyStatus: boolean;
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
