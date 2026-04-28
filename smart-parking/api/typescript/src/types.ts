/**
 * WIA-AUTO-013: Smart Parking - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Location Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  /** Latitude in decimal degrees */
  lat: number;

  /** Longitude in decimal degrees */
  lng: number;

  /** Altitude in meters (optional) */
  altitude?: number;
}

/**
 * Address information
 */
export interface Address {
  /** Street address */
  street: string;

  /** City name */
  city: string;

  /** State/province code */
  state: string;

  /** Postal/ZIP code */
  zip: string;

  /** Country code (ISO 3166-1 alpha-2) */
  country: string;
}

/**
 * Physical location with coordinates and address
 */
export interface Location {
  /** Geographic coordinates */
  coordinates: Coordinates;

  /** Street address */
  address?: Address;

  /** Floor number (for multi-level facilities) */
  floor?: number;

  /** Section/zone identifier */
  section?: string;

  /** Row identifier */
  row?: number;

  /** Space number within row */
  number?: number;
}

// ============================================================================
// Parking Space Types
// ============================================================================

/**
 * Parking space occupancy status
 */
export type OccupancyStatus =
  | 'AVAILABLE'
  | 'OCCUPIED'
  | 'RESERVED'
  | 'CHARGING'
  | 'DISABLED'
  | 'MAINTENANCE'
  | 'UNKNOWN';

/**
 * Parking space type classification
 */
export type SpaceType =
  | 'standard'
  | 'compact'
  | 'oversized'
  | 'motorcycle'
  | 'disabled'
  | 'family'
  | 'ev'
  | 'valet';

/**
 * Detection sensor type
 */
export type SensorType =
  | 'ultrasonic'
  | 'camera'
  | 'magnetic'
  | 'lidar'
  | 'radar'
  | 'pressure'
  | 'hybrid';

/**
 * Sensor information
 */
export interface SensorInfo {
  /** Sensor type */
  type: SensorType;

  /** Unique sensor identifier */
  id: string;

  /** Sensor health status */
  health: 'normal' | 'warning' | 'critical' | 'offline';

  /** Detection confidence (0-1) */
  confidence?: number;

  /** Last maintenance date */
  lastMaintenance?: Date;
}

/**
 * Space dimensions
 */
export interface SpaceDimensions {
  /** Length in meters */
  length: number;

  /** Width in meters */
  width: number;

  /** Height clearance in meters */
  height: number;

  /** Unit of measurement */
  unit: 'meters' | 'feet';
}

/**
 * EV charging capability
 */
export interface EVCharger {
  /** Charger available */
  available: boolean;

  /** Charging level */
  level: 'Level1' | 'Level2' | 'DCFast';

  /** Connector type */
  connector: 'Type1' | 'Type2' | 'CCS1' | 'CCS2' | 'CHAdeMO' | 'Tesla';

  /** Power output in kW */
  power: number;

  /** Charger ID */
  chargerId?: string;

  /** Charger status */
  status?: 'available' | 'in-use' | 'reserved' | 'maintenance' | 'offline';
}

/**
 * Accessibility features
 */
export interface AccessibilityFeatures {
  /** Designated for disabled parking */
  disabled: boolean;

  /** Family parking space */
  family: boolean;

  /** Motorcycle parking */
  motorcycle: boolean;

  /** Wider space */
  wider?: boolean;

  /** Closer to entrance */
  proximity?: 'entrance' | 'elevator' | 'stairs';
}

/**
 * Space features
 */
export interface SpaceFeatures {
  /** Covered/indoor parking */
  covered: boolean;

  /** EV charging capability */
  evCharger?: EVCharger;

  /** Accessibility features */
  accessibility: AccessibilityFeatures;

  /** Security camera coverage */
  cameraCoverage?: boolean;

  /** Lighting quality */
  lighting?: 'excellent' | 'good' | 'moderate' | 'poor';

  /** Reserved for specific use */
  reserved?: string;
}

/**
 * Current space status
 */
export interface SpaceStatus {
  /** Current occupancy status */
  current: OccupancyStatus;

  /** Detection confidence (0-1) */
  confidence: number;

  /** Last status update timestamp */
  lastUpdated: Date;

  /** Sensor information */
  sensor: SensorInfo;

  /** Vehicle information (if occupied) */
  vehicle?: VehicleInfo;
}

/**
 * Pricing information
 */
export interface Pricing {
  /** Hourly parking rate */
  hourlyRate: number;

  /** Daily maximum charge */
  dailyMax?: number;

  /** Currency code (ISO 4217) */
  currency: string;

  /** EV charging rate (per kWh) */
  evChargingRate?: number;

  /** EV charging unit */
  evChargingUnit?: 'kWh' | 'minute';

  /** Dynamic pricing enabled */
  dynamicPricing?: boolean;

  /** Current pricing multiplier */
  multiplier?: number;
}

/**
 * Geometric representation of parking space
 */
export interface SpaceGeometry {
  /** GeoJSON type */
  type: 'Polygon' | 'Point';

  /** Coordinates array */
  coordinates: number[][];
}

/**
 * Complete parking space object
 */
export interface ParkingSpace {
  /** Unique space identifier */
  id: string;

  /** Parent lot identifier */
  lotId: string;

  /** Physical location */
  location: Location;

  /** Geometric representation */
  geometry?: SpaceGeometry;

  /** Physical dimensions */
  dimensions: SpaceDimensions;

  /** Space type */
  type: SpaceType;

  /** Space features */
  features: SpaceFeatures;

  /** Current status */
  status: SpaceStatus;

  /** Pricing information */
  pricing: Pricing;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Vehicle Types
// ============================================================================

/**
 * Vehicle type classification
 */
export type VehicleType =
  | 'sedan'
  | 'suv'
  | 'truck'
  | 'van'
  | 'motorcycle'
  | 'bicycle'
  | 'compact'
  | 'electric'
  | 'other';

/**
 * Vehicle information
 */
export interface VehicleInfo {
  /** License plate number */
  licensePlate?: string;

  /** Vehicle type */
  type: VehicleType;

  /** Vehicle make */
  make?: string;

  /** Vehicle model */
  model?: string;

  /** Vehicle color */
  color?: string;

  /** Vehicle length in meters */
  length?: number;

  /** Electric vehicle */
  isElectric?: boolean;

  /** Entry timestamp */
  entryTime?: Date;

  /** Duration in minutes */
  durationMinutes?: number;
}

// ============================================================================
// Parking Lot Types
// ============================================================================

/**
 * Lot operator information
 */
export interface OperatorInfo {
  /** Operator identifier */
  id: string;

  /** Operator name */
  name: string;

  /** Contact information */
  contact: {
    phone: string;
    email: string;
    website?: string;
  };
}

/**
 * Entrance/exit point
 */
export interface EntranceExit {
  /** Entrance identifier */
  id: string;

  /** Type of access point */
  type: 'vehicle' | 'pedestrian' | 'both';

  /** Location coordinates */
  location: Coordinates;

  /** Access direction */
  access: ('entry' | 'exit')[];

  /** Operating hours */
  operatingHours?: OperatingHours;
}

/**
 * Capacity breakdown
 */
export interface LotCapacity {
  /** Total spaces */
  total: number;

  /** Standard spaces */
  standard: number;

  /** Disabled/accessible spaces */
  disabled: number;

  /** EV charging spaces */
  ev: number;

  /** Motorcycle spaces */
  motorcycle?: number;

  /** Compact spaces */
  compact?: number;

  /** Oversized spaces */
  oversized?: number;
}

/**
 * Security features
 */
export interface SecurityFeatures {
  /** CCTV cameras */
  cameras: boolean;

  /** Security guards */
  guards: boolean;

  /** Gated access */
  gated: boolean;

  /** Lighting level */
  lighting: 'full' | 'partial' | 'minimal' | 'none';

  /** Emergency call boxes */
  emergencyCallBoxes?: boolean;
}

/**
 * Amenities available
 */
export interface Amenities {
  /** Restrooms */
  restrooms: boolean;

  /** Elevator */
  elevator: boolean;

  /** Car wash */
  carWash: boolean;

  /** Valet service */
  valet: boolean;

  /** EV charging stations */
  evCharging?: boolean;

  /** WiFi */
  wifi?: boolean;

  /** ATM */
  atm?: boolean;
}

/**
 * Operating hours for a single day
 */
export interface DayHours {
  /** Opening time (24-hour format) */
  open: string;

  /** Closing time (24-hour format) */
  close: string;

  /** Closed all day */
  closed?: boolean;
}

/**
 * Weekly operating hours
 */
export interface OperatingHours {
  monday: DayHours;
  tuesday: DayHours;
  wednesday: DayHours;
  thursday: DayHours;
  friday: DayHours;
  saturday: DayHours;
  sunday: DayHours;

  /** 24/7 operation */
  is24_7: boolean;

  /** Holiday hours */
  holidays?: Record<string, DayHours>;
}

/**
 * Lot features
 */
export interface LotFeatures {
  /** Covered/indoor */
  covered: boolean;

  /** Security features */
  security: SecurityFeatures;

  /** Amenities */
  amenities: Amenities;

  /** Payment methods accepted */
  paymentMethods: PaymentMethod[];
}

/**
 * Complete parking lot object
 */
export interface ParkingLot {
  /** Unique lot identifier */
  id: string;

  /** Lot name */
  name: string;

  /** Operator information */
  operator: OperatorInfo;

  /** Location details */
  location: {
    address: Address;
    coordinates: Coordinates;
    entrances: EntranceExit[];
  };

  /** Capacity information */
  capacity: LotCapacity;

  /** Features and amenities */
  features: LotFeatures;

  /** Operating hours */
  operatingHours: OperatingHours;

  /** Base pricing */
  pricing: Pricing;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Occupancy Types
// ============================================================================

/**
 * Floor-level occupancy summary
 */
export interface FloorOccupancy {
  /** Floor number/identifier */
  floor: number | string;

  /** Total spaces on floor */
  total: number;

  /** Available spaces */
  available: number;

  /** Occupancy rate (0-1) */
  occupancyRate: number;
}

/**
 * Occupancy by space type
 */
export interface OccupancyByType {
  /** Space type */
  type: SpaceType;

  /** Total spaces of this type */
  total: number;

  /** Available spaces of this type */
  available: number;
}

/**
 * EV charging occupancy
 */
export interface EVChargingOccupancy {
  /** Total chargers */
  totalChargers: number;

  /** Chargers in use */
  inUse: number;

  /** Available chargers */
  available: number;

  /** Reserved chargers */
  reserved?: number;
}

/**
 * Occupancy prediction
 */
export interface OccupancyPrediction {
  /** Predicted occupancy rate for next hour */
  nextHour: number;

  /** Trend direction */
  trend: 'increasing' | 'decreasing' | 'stable';

  /** Confidence in prediction (0-1) */
  confidence?: number;
}

/**
 * Real-time lot occupancy summary
 */
export interface LotOccupancy {
  /** Lot identifier */
  lotId: string;

  /** Timestamp of data */
  timestamp: Date;

  /** Total spaces */
  total: number;

  /** Available spaces */
  available: number;

  /** Occupied spaces */
  occupied: number;

  /** Reserved spaces */
  reserved: number;

  /** Maintenance spaces */
  maintenance: number;

  /** Occupancy rate (0-1) */
  rate: number;

  /** Occupancy by floor */
  byFloor?: FloorOccupancy[];

  /** Occupancy by type */
  byType?: OccupancyByType[];

  /** EV charging status */
  evCharging?: EVChargingOccupancy;

  /** Occupancy prediction */
  prediction?: OccupancyPrediction;
}

// ============================================================================
// Search and Guidance Types
// ============================================================================

/**
 * Search preferences
 */
export interface SearchPreferences {
  /** Price range filter [min, max] */
  priceRange?: [number, number];

  /** Requires EV charging */
  requiresEV?: boolean;

  /** Requires accessibility features */
  requiresAccessibility?: boolean;

  /** Covered parking preferred */
  preferCovered?: boolean;

  /** Proximity preference */
  proximity?: 'entrance' | 'elevator' | 'stairs';

  /** Minimum space size */
  minimumSize?: SpaceDimensions;
}

/**
 * Space search parameters
 */
export interface SpaceSearchParams {
  /** Search center location */
  location: Coordinates;

  /** Search radius in meters */
  radius: number;

  /** Vehicle type */
  vehicleType?: VehicleType;

  /** Vehicle dimensions */
  vehicleDimensions?: SpaceDimensions;

  /** Requires EV charging */
  requiresEV?: boolean;

  /** Requires accessibility */
  requiresAccessibility?: boolean;

  /** User preferences */
  preferences?: SearchPreferences;

  /** Maximum results to return */
  limit?: number;
}

/**
 * Space search result
 */
export interface SpaceSearchResult {
  /** Parking space */
  space: ParkingSpace;

  /** Lot information */
  lot: ParkingLot;

  /** Distance from search location in meters */
  distance: number;

  /** Suitability score (0-100) */
  score: number;

  /** Estimated walking time in minutes */
  walkingTime?: number;

  /** Estimated driving time in minutes */
  drivingTime?: number;
}

// ============================================================================
// Reservation Types
// ============================================================================

/**
 * Reservation status
 */
export type ReservationStatus =
  | 'pending'
  | 'confirmed'
  | 'active'
  | 'completed'
  | 'cancelled'
  | 'expired'
  | 'no-show';

/**
 * Reservation request
 */
export interface ReservationRequest {
  /** Lot identifier */
  lotId: string;

  /** Specific space ID (optional) */
  spaceId?: string;

  /** Start time */
  startTime: Date;

  /** Duration in minutes */
  duration: number;

  /** Vehicle information */
  vehicle: VehicleInfo;

  /** User information */
  user: {
    userId?: string;
    email: string;
    phone: string;
    name?: string;
  };

  /** Preferences */
  preferences?: SearchPreferences;

  /** Payment method ID */
  paymentMethod?: string;
}

/**
 * Cost breakdown
 */
export interface CostBreakdown {
  /** Parking cost */
  parking: number;

  /** EV charging cost */
  evCharging?: number;

  /** Service fees */
  fees?: number;

  /** Tax amount */
  tax: number;

  /** Subtotal */
  subtotal: number;

  /** Total amount */
  total: number;

  /** Currency */
  currency: string;
}

/**
 * Payment information
 */
export interface PaymentInfo {
  /** Payment method type */
  method: PaymentMethod;

  /** Last 4 digits of card */
  last4?: string;

  /** Payment status */
  status: 'pending' | 'authorized' | 'captured' | 'failed' | 'refunded';

  /** Transaction ID */
  transactionId?: string;
}

/**
 * Access credentials
 */
export interface AccessCredentials {
  /** Access code */
  code?: string;

  /** QR code URL */
  qrCode?: string;

  /** RFID token */
  rfidToken?: string;

  /** License plate (for LPR) */
  licensePlate?: string;
}

/**
 * Reservation object
 */
export interface Reservation {
  /** Unique reservation ID */
  id: string;

  /** Human-readable confirmation code */
  confirmationCode: string;

  /** Space ID */
  spaceId: string;

  /** Lot ID */
  lotId: string;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Grace period in minutes */
  gracePeriod: number;

  /** Reservation status */
  status: ReservationStatus;

  /** Vehicle information */
  vehicle: VehicleInfo;

  /** Cost breakdown */
  cost: CostBreakdown;

  /** Payment information */
  payment: PaymentInfo;

  /** Access credentials */
  access: AccessCredentials;

  /** Creation timestamp */
  createdAt: Date;

  /** Last update timestamp */
  updatedAt?: Date;

  /** Special instructions */
  instructions?: string;
}

// ============================================================================
// Payment Types
// ============================================================================

/**
 * Payment method type
 */
export type PaymentMethod =
  | 'credit_card'
  | 'debit_card'
  | 'mobile_wallet'
  | 'license_plate'
  | 'qr_code'
  | 'rfid'
  | 'account_balance'
  | 'cash';

/**
 * Payment session
 */
export interface PaymentSession {
  /** Session ID */
  id: string;

  /** Amount to charge */
  amount: number;

  /** Currency */
  currency: string;

  /** Payment method */
  method: PaymentMethod;

  /** Parking session ID */
  parkingSessionId: string;

  /** Status */
  status: 'pending' | 'processing' | 'completed' | 'failed';

  /** Created timestamp */
  createdAt: Date;

  /** Completed timestamp */
  completedAt?: Date;

  /** Receipt URL */
  receiptUrl?: string;
}

// ============================================================================
// Parking Session Types
// ============================================================================

/**
 * Parking session status
 */
export type SessionStatus = 'active' | 'completed' | 'overstayed' | 'violation';

/**
 * Active parking session
 */
export interface ParkingSession {
  /** Session ID */
  id: string;

  /** Space ID */
  spaceId: string;

  /** Lot ID */
  lotId: string;

  /** Vehicle information */
  vehicle: VehicleInfo;

  /** Entry timestamp */
  entryTime: Date;

  /** Exit timestamp (if completed) */
  exitTime?: Date;

  /** Duration in minutes */
  duration: number;

  /** Session status */
  status: SessionStatus;

  /** Linked reservation ID */
  reservationId?: string;

  /** Current charges */
  currentCharges: number;

  /** Final charges (if completed) */
  finalCharges?: number;

  /** Currency */
  currency: string;

  /** Payment status */
  paymentStatus: 'unpaid' | 'paid' | 'pending';

  /** EV charging session */
  chargingSession?: ChargingSession;
}

// ============================================================================
// EV Charging Types
// ============================================================================

/**
 * Charging session status
 */
export type ChargingStatus =
  | 'idle'
  | 'preparing'
  | 'charging'
  | 'paused'
  | 'completed'
  | 'error';

/**
 * EV charging session
 */
export interface ChargingSession {
  /** Session ID */
  id: string;

  /** Charger ID */
  chargerId: string;

  /** Vehicle information */
  vehicle: {
    licensePlate?: string;
    batteryCapacity: number;
    currentSOC: number;
    targetSOC: number;
    connectorType: string;
  };

  /** Start time */
  startTime: Date;

  /** End time (if completed) */
  endTime?: Date;

  /** Estimated completion time */
  estimatedCompletion?: Date;

  /** Current charging power in kW */
  currentPower: number;

  /** Total energy delivered in kWh */
  energyDelivered: number;

  /** Current state of charge (%) */
  currentSOC: number;

  /** Charging status */
  status: ChargingStatus;

  /** Cost so far */
  costSoFar: number;

  /** Final cost (if completed) */
  finalCost?: number;

  /** Currency */
  currency: string;

  /** Pricing rate per kWh */
  ratePerKWh: number;

  /** Elapsed time in minutes */
  elapsedTime: number;
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Time period for analytics
 */
export type TimePeriod = 'hour' | 'day' | 'week' | 'month' | 'year';

/**
 * Occupancy statistics
 */
export interface OccupancyStats {
  /** Time period */
  period: TimePeriod;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Average occupancy rate */
  averageRate: number;

  /** Peak occupancy rate */
  peakRate: number;

  /** Lowest occupancy rate */
  lowestRate: number;

  /** Average dwell time in minutes */
  averageDwellTime: number;

  /** Turnover rate (vehicles per space per period) */
  turnoverRate: number;

  /** Hourly breakdown */
  hourlyBreakdown?: Array<{
    hour: number;
    rate: number;
    available: number;
  }>;
}

/**
 * Revenue statistics
 */
export interface RevenueStats {
  /** Time period */
  period: TimePeriod;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Total revenue */
  totalRevenue: number;

  /** Parking revenue */
  parkingRevenue: number;

  /** EV charging revenue */
  chargingRevenue?: number;

  /** Overstay penalties */
  penalties?: number;

  /** Currency */
  currency: string;

  /** Number of transactions */
  transactionCount: number;

  /** Average transaction value */
  averageTransactionValue: number;

  /** Payment method breakdown */
  byPaymentMethod?: Record<PaymentMethod, number>;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Real-time event type
 */
export type EventType =
  | 'occupancy_change'
  | 'space_status'
  | 'reservation_update'
  | 'charging_status'
  | 'payment_complete'
  | 'violation_detected';

/**
 * Real-time event
 */
export interface ParkingEvent {
  /** Event type */
  event: EventType;

  /** Timestamp */
  timestamp: Date;

  /** Lot ID */
  lotId: string;

  /** Space ID (if applicable) */
  spaceId?: string;

  /** Event data */
  data: Record<string, unknown>;

  /** Previous state (for status changes) */
  previousState?: unknown;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Standard API response
 */
export interface APIResponse<T> {
  /** Response status */
  status: 'success' | 'error';

  /** Response data */
  data?: T;

  /** Error message */
  error?: string;

  /** Error code */
  errorCode?: string;

  /** Timestamp */
  timestamp: Date;

  /** Request ID */
  requestId?: string;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Results array */
  results: T[];

  /** Total count */
  total: number;

  /** Page number */
  page: number;

  /** Page size */
  pageSize: number;

  /** Has more pages */
  hasMore: boolean;

  /** Next page URL */
  nextPage?: string;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Standard parking dimensions and constants
 */
export const PARKING_CONSTANTS = {
  /** Standard parking space dimensions in meters */
  STANDARD_SPACE: {
    length: 5.5,
    width: 2.5,
    height: 2.2,
  },

  /** Compact space dimensions in meters */
  COMPACT_SPACE: {
    length: 4.8,
    width: 2.3,
    height: 2.0,
  },

  /** Disabled space dimensions in meters */
  DISABLED_SPACE: {
    length: 5.5,
    width: 3.7,
    height: 2.5,
  },

  /** Default search radius in meters */
  DEFAULT_SEARCH_RADIUS: 1000,

  /** Maximum search radius in meters */
  MAX_SEARCH_RADIUS: 5000,

  /** Default reservation grace period in minutes */
  DEFAULT_GRACE_PERIOD: 15,

  /** Minimum detection confidence threshold */
  MIN_CONFIDENCE: 0.90,

  /** Default WebSocket heartbeat interval in seconds */
  WEBSOCKET_HEARTBEAT: 30,
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Location types
  Coordinates,
  Address,
  Location,

  // Space types
  OccupancyStatus,
  SpaceType,
  SensorType,
  SensorInfo,
  SpaceDimensions,
  EVCharger,
  AccessibilityFeatures,
  SpaceFeatures,
  SpaceStatus,
  Pricing,
  SpaceGeometry,
  ParkingSpace,

  // Vehicle types
  VehicleType,
  VehicleInfo,

  // Lot types
  OperatorInfo,
  EntranceExit,
  LotCapacity,
  SecurityFeatures,
  Amenities,
  DayHours,
  OperatingHours,
  LotFeatures,
  ParkingLot,

  // Occupancy types
  FloorOccupancy,
  OccupancyByType,
  EVChargingOccupancy,
  OccupancyPrediction,
  LotOccupancy,

  // Search types
  SearchPreferences,
  SpaceSearchParams,
  SpaceSearchResult,

  // Reservation types
  ReservationStatus,
  ReservationRequest,
  CostBreakdown,
  PaymentInfo,
  AccessCredentials,
  Reservation,

  // Payment types
  PaymentMethod,
  PaymentSession,

  // Session types
  SessionStatus,
  ParkingSession,

  // Charging types
  ChargingStatus,
  ChargingSession,

  // Analytics types
  TimePeriod,
  OccupancyStats,
  RevenueStats,

  // Event types
  EventType,
  ParkingEvent,

  // API types
  APIResponse,
  PaginatedResponse,
};

export { PARKING_CONSTANTS };
