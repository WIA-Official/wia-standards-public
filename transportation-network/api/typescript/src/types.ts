/**
 * WIA-UNI-008 Transportation Network Standard
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
  createdBy: string;
  createdAt: string;
  modifiedBy?: string;
  modifiedAt?: string;
  version?: string;
}

/**
 * Geographic location with coordinates
 */
export interface Location {
  latitude: number;
  longitude: number;
  elevation?: number;
  name?: string;
}

/**
 * Station or terminal information
 */
export interface Station {
  name: string;
  code: string;
  location: Location;
  stopDuration?: number;
}

/**
 * Operator information
 */
export interface Operator {
  name: string;
  operatorCode?: string;
  country: string;
  license?: string;
}

/**
 * Distance measurement
 */
export interface Distance {
  value: number;
  unit: 'km' | 'miles';
}

/**
 * Duration measurement
 */
export interface Duration {
  value: number;
  unit: 'hours' | 'minutes';
}

/**
 * Schedule information
 */
export interface Schedule {
  frequency: 'daily' | 'weekly' | 'on-demand';
  departureTimes: string[];
  validFrom?: string;
  validUntil?: string;
}

/**
 * Vehicle/vessel specifications
 */
export interface VehicleSpecifications {
  maxSpeed: number;
  capacity: {
    passengers?: number;
    cargo?: number;
  };
  gauge?: number;
  electrification?: string;
  signaling?: string;
}

/**
 * Transportation route types
 */
export type RouteType =
  | 'high-speed-rail'
  | 'conventional-rail'
  | 'highway'
  | 'air-route'
  | 'maritime';

/**
 * Certification status
 */
export type CertificationStatus =
  | 'pending'
  | 'in-progress'
  | 'certified'
  | 'expired';

/**
 * Booking status
 */
export type BookingStatus =
  | 'pending'
  | 'confirmed'
  | 'cancelled'
  | 'completed';

/**
 * Payment status
 */
export type PaymentStatus =
  | 'pending'
  | 'completed'
  | 'refunded';

/**
 * Travel class
 */
export type TravelClass =
  | 'economy'
  | 'business'
  | 'first';

/**
 * Transportation route
 */
export interface TransportationRoute extends WIAEntity {
  '@type': 'TransportationRoute';
  routeType: RouteType;
  name: string;
  description?: string;
  operator: Operator;
  origin: Station;
  destination: Station;
  waypoints?: Station[];
  distance: Distance;
  estimatedDuration: Duration;
  schedule: Schedule;
  specifications?: VehicleSpecifications;
  standards?: string[];
  certificationStatus: CertificationStatus;
}

/**
 * Passenger information
 */
export interface Passenger {
  name: string;
  passportNumber: string;
  nationality: string;
  dateOfBirth?: string;
  contactInfo?: {
    email?: string;
    phone?: string;
  };
}

/**
 * Journey details
 */
export interface Journey {
  routeId: string;
  origin: string;
  destination: string;
  departureDate: string;
  departureTime: string;
  arrivalTime?: string;
  class: TravelClass;
  seatNumber?: string;
}

/**
 * Pricing information
 */
export interface Pricing {
  basePrice?: number;
  taxes?: number;
  fees?: number;
  totalAmount: number;
  currency: string;
  discounts?: Array<{
    type: string;
    amount: number;
  }>;
}

/**
 * Payment information
 */
export interface Payment {
  method: 'credit-card' | 'bank-transfer' | 'digital-wallet';
  status: PaymentStatus;
  transactionId?: string;
}

/**
 * Transportation booking
 */
export interface TransportationBooking extends WIAEntity {
  '@type': 'TransportationBooking';
  bookingReference: string;
  status: BookingStatus;
  bookingDate?: string;
  passenger: Passenger;
  journey: Journey;
  pricing: Pricing;
  payment?: Payment;
  additionalServices?: Array<{
    type: string;
    description: string;
    price: number;
  }>;
  specialRequirements?: string[];
  verifiableCredential?: {
    id: string;
    issued: boolean;
  };
}

/**
 * Vehicle/vessel types
 */
export type VehicleType =
  | 'train'
  | 'bus'
  | 'truck'
  | 'aircraft'
  | 'ship';

/**
 * Transport vehicle
 */
export interface TransportVehicle extends WIAEntity {
  '@type': 'TransportVehicle';
  vehicleType: VehicleType;
  registrationNumber: string;
  operator: Operator;
  specifications: {
    manufacturer: string;
    model: string;
    yearBuilt: number;
    capacity: {
      passengers?: number;
      cargo?: number;
    };
    fuelType?: string;
    maxSpeed?: number;
  };
  certifications?: Array<{
    type: string;
    issuedBy: string;
    validUntil: string;
  }>;
  currentStatus: 'active' | 'maintenance' | 'retired';
}

/**
 * Cargo type
 */
export type CargoType =
  | 'container'
  | 'bulk'
  | 'perishable'
  | 'hazardous';

/**
 * Cargo shipment status
 */
export type CargoStatus =
  | 'booked'
  | 'in-transit'
  | 'customs'
  | 'delivered';

/**
 * Cargo details
 */
export interface Cargo {
  description: string;
  type: CargoType;
  weight: {
    value: number;
    unit: 'kg' | 'tons';
  };
  volume?: {
    value: number;
    unit: 'm3';
  };
  containerNumber?: string;
  hsCode?: string;
}

/**
 * Route segment
 */
export interface RouteSegment {
  mode: 'rail' | 'road' | 'air' | 'sea';
  from: string;
  to: string;
  vehicleId?: string;
  estimatedDeparture?: string;
  estimatedArrival?: string;
}

/**
 * Customs information
 */
export interface CustomsInfo {
  status: 'pending' | 'cleared' | 'inspection';
  declarationNumber?: string;
  estimatedClearance?: string;
}

/**
 * Cargo shipment
 */
export interface CargoShipment extends WIAEntity {
  '@type': 'CargoShipment';
  trackingNumber: string;
  status: CargoStatus;
  shipper: {
    name: string;
    address: Record<string, any>;
    contact: Record<string, any>;
  };
  consignee: {
    name: string;
    address: Record<string, any>;
    contact: Record<string, any>;
  };
  cargo: Cargo;
  route: {
    origin: string;
    destination: string;
    currentLocation?: {
      name: string;
      coordinates: Location;
      timestamp: string;
    };
    segments?: RouteSegment[];
  };
  customs?: CustomsInfo;
}

/**
 * Vehicle tracking status
 */
export type VehicleStatus =
  | 'in-transit'
  | 'stopped'
  | 'delayed'
  | 'emergency';

/**
 * Speed information
 */
export interface Speed {
  value: number;
  unit: 'km/h' | 'mph' | 'knots';
}

/**
 * Next stop information
 */
export interface NextStop {
  name: string;
  code: string;
  estimatedArrival: string;
  distance?: number;
}

/**
 * Alert information
 */
export interface Alert {
  type: 'delay' | 'emergency' | 'maintenance';
  severity: 'info' | 'warning' | 'critical';
  message: string;
  timestamp: string;
}

/**
 * Vehicle tracking data
 */
export interface VehicleTracking {
  vehicleId: string;
  trackingId?: string;
  timestamp: string;
  location: {
    latitude: number;
    longitude: number;
    altitude?: number;
    heading?: number;
    accuracy?: number;
  };
  speed: Speed;
  status: VehicleStatus;
  nextStop?: NextStop;
  occupancy?: {
    passengerCount?: number;
    cargoLoad?: number;
  };
  onTimePerformance?: string;
  alerts?: Alert[];
}

/**
 * API response wrapper
 */
export interface APIResponse<T> {
  data: T;
  metadata?: {
    timestamp: string;
    requestId: string;
  };
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  totalResults: number;
  page: number;
  limit: number;
  totalPages?: number;
  data: T[];
  links?: {
    first?: string;
    prev?: string;
    next?: string;
    last?: string;
  };
}

/**
 * Query parameters for list operations
 */
export interface QueryParams {
  page?: number;
  limit?: number;
  type?: RouteType | VehicleType;
  origin?: string;
  destination?: string;
  operator?: string;
  status?: string;
}

/**
 * API error response
 */
export interface APIError {
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
    timestamp: string;
    requestId?: string;
  };
}

/**
 * Webhook event types
 */
export type WebhookEvent =
  | 'booking.confirmed'
  | 'booking.cancelled'
  | 'booking.modified'
  | 'vehicle.delayed'
  | 'vehicle.emergency'
  | 'cargo.status_change'
  | 'customs.cleared';

/**
 * Webhook configuration
 */
export interface WebhookConfig {
  url: string;
  events: WebhookEvent[];
  secret?: string;
}

/**
 * Webhook registration response
 */
export interface Webhook {
  id: string;
  url: string;
  events: WebhookEvent[];
  active: boolean;
  createdAt?: string;
}

/**
 * Availability search parameters
 */
export interface AvailabilityParams {
  origin: string;
  destination: string;
  date: string;
  passengers?: number;
  class?: TravelClass;
}

/**
 * Availability result
 */
export interface AvailabilityResult {
  searchDate: string;
  origin: string;
  destination: string;
  available: Array<{
    routeId: string;
    departureTime: string;
    arrivalTime: string;
    price: Record<TravelClass, number>;
    seatsAvailable: Record<TravelClass, number>;
  }>;
}
