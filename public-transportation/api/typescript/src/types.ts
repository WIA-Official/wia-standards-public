/**
 * WIA-SOC-007: Public Transportation Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * UUID v4 string
 */
export type UUID = string;

/**
 * Geographic coordinates
 */
export interface Coordinates {
  lat: number;
  lon: number;
}

// ============================================================================
// Transit Types
// ============================================================================

/**
 * Route types per GTFS specification
 */
export enum RouteType {
  TRAM = 0,
  METRO = 1,
  RAIL = 2,
  BUS = 3,
  FERRY = 4,
  CABLE_TRAM = 5,
  AERIAL_LIFT = 6,
  FUNICULAR = 7,
  TROLLEYBUS = 11,
  MONORAIL = 12
}

/**
 * Transit agency information
 */
export interface TransitAgency {
  agencyId: UUID;
  agencyName: string;
  agencyUrl: string;
  agencyTimezone: string;
  agencyLang: string;
  agencyPhone?: string;
  agencyFareUrl?: string;
  agencyEmail?: string;
}

/**
 * Transit route information
 */
export interface TransitRoute {
  routeId: UUID;
  agencyId: UUID;
  routeShortName: string;
  routeLongName: string;
  routeDesc?: string;
  routeType: RouteType;
  routeUrl?: string;
  routeColor?: string;
  routeTextColor?: string;
  routeSortOrder?: number;
  continuousPickup?: number;
  continuousDropOff?: number;
}

/**
 * Wheelchair boarding status
 */
export enum WheelchairBoarding {
  UNKNOWN = 0,
  ACCESSIBLE = 1,
  NOT_ACCESSIBLE = 2
}

/**
 * Stop/Station information
 */
export interface TransitStop {
  stopId: UUID;
  stopCode?: string;
  stopName: string;
  stopDesc?: string;
  stopLat: number;
  stopLon: number;
  zoneId?: string;
  stopUrl?: string;
  locationType?: number;
  parentStation?: UUID;
  stopTimezone?: string;
  wheelchairBoarding?: WheelchairBoarding;
  levelId?: string;
  platformCode?: string;
}

/**
 * Trip information
 */
export interface TransitTrip {
  tripId: UUID;
  routeId: UUID;
  serviceId: UUID;
  tripHeadsign?: string;
  tripShortName?: string;
  directionId?: number;
  blockId?: string;
  shapeId?: string;
  wheelchairAccessible?: number;
  bikesAllowed?: number;
}

/**
 * Stop time information
 */
export interface StopTime {
  tripId: UUID;
  arrivalTime: string;
  departureTime: string;
  stopId: UUID;
  stopSequence: number;
  stopHeadsign?: string;
  pickupType?: number;
  dropOffType?: number;
  continuousPickup?: number;
  continuousDropOff?: number;
  shapeDistTraveled?: number;
  timepoint?: number;
}

// ============================================================================
// Real-time Types
// ============================================================================

/**
 * Vehicle current status
 */
export enum VehicleStatus {
  INCOMING_AT = 'INCOMING_AT',
  STOPPED_AT = 'STOPPED_AT',
  IN_TRANSIT_TO = 'IN_TRANSIT_TO'
}

/**
 * Congestion levels
 */
export enum CongestionLevel {
  UNKNOWN = 'UNKNOWN_CONGESTION_LEVEL',
  SMOOTH = 'RUNNING_SMOOTHLY',
  STOP_AND_GO = 'STOP_AND_GO',
  CONGESTION = 'CONGESTION',
  SEVERE = 'SEVERE_CONGESTION'
}

/**
 * Occupancy status
 */
export enum OccupancyStatus {
  EMPTY = 'EMPTY',
  MANY_SEATS = 'MANY_SEATS_AVAILABLE',
  FEW_SEATS = 'FEW_SEATS_AVAILABLE',
  STANDING_ONLY = 'STANDING_ROOM_ONLY',
  CRUSHED = 'CRUSHED_STANDING_ROOM_ONLY',
  FULL = 'FULL',
  NOT_ACCEPTING = 'NOT_ACCEPTING_PASSENGERS'
}

/**
 * Vehicle position
 */
export interface VehiclePosition {
  vehicleId: UUID;
  tripId?: UUID;
  routeId: UUID;
  position: {
    latitude: number;
    longitude: number;
    bearing?: number;
    speed?: number;
  };
  timestamp: Timestamp;
  currentStopSequence?: number;
  currentStatus?: VehicleStatus;
  congestionLevel?: CongestionLevel;
  occupancyStatus?: OccupancyStatus;
}

/**
 * Trip update
 */
export interface TripUpdate {
  tripId: UUID;
  routeId: UUID;
  vehicleId?: UUID;
  timestamp: Timestamp;
  delay?: number;
  stopTimeUpdates?: StopTimeUpdate[];
}

/**
 * Stop time update
 */
export interface StopTimeUpdate {
  stopSequence: number;
  stopId: UUID;
  arrival?: TimeEvent;
  departure?: TimeEvent;
  scheduleRelationship?: string;
}

/**
 * Time event (arrival/departure)
 */
export interface TimeEvent {
  delay?: number;
  time?: Timestamp;
  uncertainty?: number;
}

/**
 * Service alert
 */
export interface ServiceAlert {
  alertId: UUID;
  cause?: string;
  effect?: string;
  url?: string;
  headerText?: Record<string, string>;
  descriptionText?: Record<string, string>;
  activePeriod?: TimeRange[];
  informedEntity?: InformedEntity[];
}

/**
 * Time range
 */
export interface TimeRange {
  start?: Timestamp;
  end?: Timestamp;
}

/**
 * Informed entity
 */
export interface InformedEntity {
  agencyId?: UUID;
  routeId?: UUID;
  routeType?: RouteType;
  trip?: TripDescriptor;
  stopId?: UUID;
}

/**
 * Trip descriptor
 */
export interface TripDescriptor {
  tripId?: UUID;
  routeId?: UUID;
  directionId?: number;
  startTime?: string;
  startDate?: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Arrival prediction
 */
export interface ArrivalPrediction {
  tripId: UUID;
  routeId: UUID;
  routeShortName: string;
  headsign?: string;
  scheduledArrival: Timestamp;
  predictedArrival?: Timestamp;
  delay?: number;
  occupancyStatus?: OccupancyStatus;
  wheelchairAccessible?: boolean;
  realtime: boolean;
}

/**
 * Stop arrivals response
 */
export interface StopArrivalsResponse {
  stopId: UUID;
  stopName: string;
  arrivals: ArrivalPrediction[];
  timestamp: Timestamp;
}

/**
 * Trip planning request
 */
export interface TripPlanRequest {
  origin: Coordinates;
  destination: Coordinates;
  time?: Timestamp;
  arriveBy?: boolean;
  modes?: string[];
  preferences?: TripPreferences;
}

/**
 * Trip preferences
 */
export interface TripPreferences {
  optimize?: 'fastest' | 'cheapest' | 'fewest_transfers' | 'accessible';
  maxWalkDistance?: number;
  wheelchairAccessible?: boolean;
  maxTransfers?: number;
  avoidRoutes?: UUID[];
}

/**
 * Itinerary
 */
export interface Itinerary {
  duration: number;
  walkDistance: number;
  walkTime: number;
  transitTime: number;
  waitingTime: number;
  transfers: number;
  fare?: Fare;
  carbonFootprint?: CarbonFootprint;
  legs: Leg[];
}

/**
 * Journey leg
 */
export interface Leg {
  mode: string;
  from: string;
  to: string;
  startTime: Timestamp;
  endTime: Timestamp;
  distance?: number;
  routeId?: UUID;
  routeShortName?: string;
  tripId?: UUID;
  headsign?: string;
  stops?: number;
  path?: Coordinates[];
}

/**
 * Fare information
 */
export interface Fare {
  currency: string;
  amount: number;
  breakdown?: FareBreakdown[];
}

/**
 * Fare breakdown
 */
export interface FareBreakdown {
  leg: number;
  fare: number;
  description: string;
}

/**
 * Carbon footprint
 */
export interface CarbonFootprint {
  emissions: number;
  savingsVsCar: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * API error response
 */
export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
  requestId?: string;
}
