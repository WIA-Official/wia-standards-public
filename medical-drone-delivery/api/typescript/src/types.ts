/**
 * WIA-MED-017: Medical Drone Delivery Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum DroneStatus {
  IDLE = 'idle',
  LOADING = 'loading',
  IN_FLIGHT = 'in_flight',
  DELIVERING = 'delivering',
  RETURNING = 'returning',
  CHARGING = 'charging',
  MAINTENANCE = 'maintenance',
}

export enum DeliveryPriority {
  EMERGENCY = 'emergency',
  URGENT = 'urgent',
  STANDARD = 'standard',
  SCHEDULED = 'scheduled',
}

export enum CargoType {
  BLOOD = 'blood',
  MEDICATION = 'medication',
  VACCINE = 'vaccine',
  ORGAN = 'organ',
  SAMPLES = 'samples',
  EQUIPMENT = 'equipment',
}

export enum DeliveryStatus {
  PENDING = 'pending',
  DISPATCHED = 'dispatched',
  IN_TRANSIT = 'in_transit',
  DELIVERED = 'delivered',
  FAILED = 'failed',
  CANCELLED = 'cancelled',
}

// ============================================================================
// Drone Types
// ============================================================================

export interface MedicalDrone {
  droneId: string;
  model: string;
  manufacturer: string;
  status: DroneStatus;
  battery: BatteryStatus;
  location: GeoLocation;
  payloadCapacityKg: number;
  rangeKm: number;
  maxSpeedKmh: number;
  temperatureControl: boolean;
  lastMaintenance: string;
}

export interface BatteryStatus {
  percentage: number;
  voltageV: number;
  healthPercent: number;
  estimatedRangeKm: number;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitudeM: number;
  heading?: number;
  speedKmh?: number;
}

// ============================================================================
// Delivery Types
// ============================================================================

export interface Delivery {
  deliveryId: string;
  droneId: string;
  priority: DeliveryPriority;
  status: DeliveryStatus;
  cargo: Cargo;
  origin: Waypoint;
  destination: Waypoint;
  scheduledTime: string;
  dispatchedAt?: string;
  deliveredAt?: string;
  estimatedArrival?: string;
  route: Waypoint[];
}

export interface Cargo {
  cargoId: string;
  type: CargoType;
  description: string;
  weightKg: number;
  temperatureRange?: TemperatureRange;
  hazardous: boolean;
  fragile: boolean;
}

export interface TemperatureRange {
  minC: number;
  maxC: number;
  currentC?: number;
}

export interface Waypoint {
  waypointId: string;
  name: string;
  location: GeoLocation;
  type: 'origin' | 'destination' | 'waypoint' | 'landing_zone';
  facilityId?: string;
}

// ============================================================================
// Telemetry Types
// ============================================================================

export interface DroneTelemetry {
  droneId: string;
  timestamp: string;
  location: GeoLocation;
  battery: BatteryStatus;
  cargoTemperature?: number;
  windSpeedKmh?: number;
  visibility?: number;
  status: DroneStatus;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: string;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
