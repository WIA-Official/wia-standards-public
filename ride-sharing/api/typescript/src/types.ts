/**
 * WIA-AUTO-014: Ride Sharing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Mobility Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geography Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  latitude: number;
  longitude: number;
  accuracy?: number;
  altitude?: number;
  bearing?: number;
  speed?: number;
}

/**
 * Physical address information
 */
export interface Address {
  street?: string;
  city: string;
  state: string;
  postalCode?: string;
  country: string;
  formattedAddress: string;
}

/**
 * Location with coordinates and address
 */
export interface Location {
  location: Coordinates;
  address?: Address | string;
  notes?: string;
  landmark?: string;
}

// ============================================================================
// Vehicle Types
// ============================================================================

/**
 * Vehicle type categories
 */
export type VehicleType =
  | 'economy'
  | 'sedan'
  | 'premium'
  | 'suv'
  | 'luxury'
  | 'van'
  | 'xl'
  | 'green'; // Electric/hybrid

/**
 * Vehicle accessibility features
 */
export type AccessibilityFeature =
  | 'wheelchair'
  | 'assistive_devices'
  | 'service_animal'
  | 'extra_space';

/**
 * Vehicle features
 */
export type VehicleFeature =
  | 'ac'
  | 'heating'
  | 'bluetooth'
  | 'usb_charging'
  | 'wifi'
  | 'child_seat'
  | 'pet_friendly'
  | 'quiet_mode';

/**
 * Vehicle information
 */
export interface Vehicle {
  make: string;
  model: string;
  year: number;
  color: string;
  licensePlate: string;
  capacity: number;
  type: VehicleType;
  features: VehicleFeature[];
  accessibility: AccessibilityFeature[];
  photoUrl?: string;
  isElectric?: boolean;
  isHybrid?: boolean;
}

// ============================================================================
// User Types
// ============================================================================

/**
 * User role
 */
export type UserRole = 'rider' | 'driver' | 'admin';

/**
 * Account verification status
 */
export interface VerificationStatus {
  idVerified: boolean;
  phoneVerified: boolean;
  emailVerified: boolean;
  backgroundCheckPassed?: boolean;
  backgroundCheckDate?: Date | string;
  licenseVerified?: boolean;
  licenseNumber?: string;
  licenseExpiry?: Date | string;
  licenseState?: string;
}

/**
 * User rating information
 */
export interface Rating {
  average: number; // 0-5
  totalTrips: number;
  recentRating?: number; // Last 50 trips
  last50Trips?: number;
  breakdown?: {
    5: number;
    4: number;
    3: number;
    2: number;
    1: number;
  };
}

/**
 * Driver status
 */
export interface DriverStatus {
  online: boolean;
  available: boolean;
  currentTrip: string | null;
  location?: Coordinates;
  heading?: number;
  batteryLevel?: number; // For EVs
}

/**
 * Driver profile
 */
export interface DriverProfile {
  driverId: string;
  personalInfo: {
    firstName: string;
    lastName: string;
    phone: string;
    email: string;
    photoUrl?: string;
    languages?: string[];
  };
  verification: VerificationStatus;
  vehicle: Vehicle;
  rating: Rating;
  status: DriverStatus;
  earnings?: {
    totalLifetime: number;
    currentWeek: number;
    pendingBalance: number;
  };
  joinDate?: Date | string;
  totalTripsCompleted?: number;
}

/**
 * Rider profile
 */
export interface RiderProfile {
  riderId: string;
  personalInfo: {
    firstName: string;
    lastName: string;
    phone: string;
    email: string;
    photoUrl?: string;
  };
  verification: VerificationStatus;
  rating: Rating;
  preferences?: RidePreferences;
  paymentMethods?: PaymentMethod[];
  emergencyContacts?: EmergencyContact[];
}

/**
 * Emergency contact
 */
export interface EmergencyContact {
  name: string;
  phone: string;
  relationship: string;
  notifyOnTrips?: boolean;
}

// ============================================================================
// Ride Request and Preferences
// ============================================================================

/**
 * Ride preferences
 */
export interface RidePreferences {
  maxWaitTime?: number; // seconds
  vehicleType?: VehicleType;
  accessibility?: AccessibilityFeature[];
  petFriendly?: boolean;
  quietMode?: boolean;
  temperature?: number; // Celsius
  musicPreference?: 'off' | 'driver_choice' | 'passenger_choice';
  conversationLevel?: 'chatty' | 'normal' | 'quiet';
}

/**
 * Ride request
 */
export interface RideRequest {
  requestId?: string;
  riderId: string;
  timestamp?: Date | string;
  pickup: Location;
  destination: Location;
  passengers: number;
  vehicleType?: VehicleType;
  preferences?: RidePreferences;
  scheduled?: Date | string | null;
  paymentMethod?: string;
  promoCode?: string;
  shareTrip?: boolean; // Willing to carpool
  notes?: string;
}

/**
 * Fare estimate for different vehicle types
 */
export interface FareEstimate {
  estimateId: string;
  pickup: Coordinates;
  destination: Coordinates;
  vehicleTypes: VehicleTypeEstimate[];
  timestamp: Date | string;
  expiresAt: Date | string;
}

/**
 * Fare estimate for a specific vehicle type
 */
export interface VehicleTypeEstimate {
  type: VehicleType;
  displayName: string;
  capacity: number;
  eta: number; // minutes
  fare: {
    minimum: number;
    maximum: number;
    estimate: number;
    currency: string;
    surgeMultiplier: number;
  };
  duration: number; // minutes
  distance: number; // km
  available: boolean;
}

// ============================================================================
// Trip and Route
// ============================================================================

/**
 * Trip status
 */
export type TripStatus =
  | 'requested'
  | 'searching'
  | 'matched'
  | 'accepted'
  | 'driver_arriving'
  | 'arrived'
  | 'in_progress'
  | 'completed'
  | 'cancelled'
  | 'failed';

/**
 * Trip cancellation reasons
 */
export type CancellationReason =
  | 'rider_cancelled'
  | 'driver_cancelled'
  | 'no_drivers_available'
  | 'payment_failed'
  | 'safety_concern'
  | 'other';

/**
 * Route information
 */
export interface Route {
  distance: number; // km
  duration: number; // seconds
  polyline?: string; // Encoded polyline
  waypoints?: Coordinates[];
  trafficCondition?: 'light' | 'moderate' | 'heavy' | 'severe';
  alternativeRoutes?: Route[];
}

/**
 * Trip timestamps
 */
export interface TripTimestamps {
  requested: Date | string;
  accepted?: Date | string;
  driverArrived?: Date | string;
  pickupComplete?: Date | string;
  dropoffComplete?: Date | string;
  cancelled?: Date | string;
}

/**
 * Trip fare breakdown
 */
export interface TripFare {
  currency: string;
  baseFare: number;
  distanceFare: number;
  timeFare: number;
  surgeMultiplier: number;
  subtotal: number;
  fees: number;
  discount?: number;
  tip?: number;
  total: number;
  driverEarnings: number;
  breakdown?: {
    baseRate: number;
    perKmRate: number;
    perMinuteRate: number;
    serviceFee: number;
    taxes: number;
    tolls?: number;
  };
}

/**
 * Trip ratings
 */
export interface TripRatings {
  riderRating?: number;
  driverRating?: number;
  riderComment?: string;
  driverComment?: string;
  riderTags?: string[];
  driverTags?: string[];
}

/**
 * Complete trip record
 */
export interface Trip {
  tripId: string;
  riderId: string;
  driverId?: string;
  status: TripStatus;
  timestamps: TripTimestamps;
  locations: {
    pickup: Location;
    destination: Location;
  };
  route?: Route;
  fare?: TripFare;
  ratings?: TripRatings;
  vehicleType: VehicleType;
  passengers: number;
  cancellationReason?: CancellationReason;
  cancellationFee?: number;
  receipt?: string; // Receipt URL
  shareUrl?: string; // Trip sharing URL
  emergencyActivated?: boolean;
}

// ============================================================================
// Matching and Scoring
// ============================================================================

/**
 * Matching parameters
 */
export interface MatchingParams {
  riderId: string;
  pickup: Coordinates;
  destination: Coordinates;
  vehicleType?: VehicleType;
  maxWaitTime?: number;
  preferences?: RidePreferences;
}

/**
 * Driver match result
 */
export interface DriverMatch {
  driverId: string;
  driver: {
    name: string;
    photoUrl?: string;
    rating: number;
    totalTrips: number;
  };
  vehicle: Vehicle;
  eta: number; // minutes
  distance: number; // km to pickup
  matchScore: number; // 0-1
  location: Coordinates;
  fareEstimate: {
    min: number;
    max: number;
    estimate: number;
    currency: string;
  };
}

/**
 * Matching score components
 */
export interface MatchingScore {
  total: number; // 0-1
  components: {
    distanceFactor: number;
    timeFactor: number;
    ratingFactor: number;
    preferenceFactor: number;
    vehicleFactor: number;
  };
  weights: {
    distance: number;
    time: number;
    rating: number;
    preference: number;
    vehicle: number;
  };
}

// ============================================================================
// Pricing and Surge
// ============================================================================

/**
 * Fare calculation parameters
 */
export interface FareCalculationParams {
  distance: number; // km
  duration: number; // minutes
  vehicleType: VehicleType;
  surgeMultiplier?: number;
  timeOfDay?: Date;
  pickupLocation?: Coordinates;
  destinationLocation?: Coordinates;
  promoCode?: string;
}

/**
 * Fare calculation result
 */
export interface FareResult {
  total: number;
  currency: string;
  breakdown: {
    baseFare: number;
    distanceFare: number;
    timeFare: number;
    subtotal: number;
    surgeAmount: number;
    fees: number;
    discount: number;
  };
  surge: number; // Multiplier
  displayPrice: string; // Formatted price
  driverEarnings: number;
  platformFee: number;
}

/**
 * Surge pricing data
 */
export interface SurgePricing {
  area: string;
  geofence?: Coordinates[];
  multiplier: number;
  demand: number; // Current requests
  supply: number; // Available drivers
  demandSupplyRatio: number;
  validUntil: Date | string;
  level: 'none' | 'low' | 'medium' | 'high' | 'extreme';
}

/**
 * Demand heatmap data
 */
export interface DemandHeatmap {
  region: string;
  timestamp: Date | string;
  cells: DemandCell[];
}

/**
 * Demand cell in heatmap
 */
export interface DemandCell {
  center: Coordinates;
  radius: number; // meters
  demand: number;
  supply: number;
  surgeMultiplier: number;
  intensity: number; // 0-1
}

// ============================================================================
// Payment
// ============================================================================

/**
 * Payment method types
 */
export type PaymentMethodType =
  | 'card'
  | 'bank_account'
  | 'digital_wallet'
  | 'cash'
  | 'corporate'
  | 'credits';

/**
 * Payment method
 */
export interface PaymentMethod {
  id: string;
  type: PaymentMethodType;
  isDefault: boolean;
  displayName: string; // e.g., "Visa ending in 1234"
  details?: {
    last4?: string;
    brand?: string;
    expiryMonth?: number;
    expiryYear?: number;
    billingAddress?: Address;
  };
  status: 'active' | 'expired' | 'failed' | 'pending';
}

/**
 * Payment transaction
 */
export interface PaymentTransaction {
  transactionId: string;
  tripId: string;
  amount: number;
  currency: string;
  paymentMethod: PaymentMethodType;
  status: 'pending' | 'completed' | 'failed' | 'refunded';
  timestamp: Date | string;
  receipt?: string;
  refundAmount?: number;
  refundReason?: string;
}

/**
 * Split payment
 */
export interface SplitPayment {
  tripId: string;
  participants: SplitParticipant[];
  status: 'pending' | 'all_confirmed' | 'processing' | 'completed' | 'failed';
  totalAmount: number;
  currency: string;
}

/**
 * Split payment participant
 */
export interface SplitParticipant {
  riderId: string;
  amount: number;
  percentage?: number;
  confirmed: boolean;
  paymentMethod?: string;
  status: 'pending' | 'confirmed' | 'paid' | 'failed';
}

// ============================================================================
// Safety and Security
// ============================================================================

/**
 * Safety check types
 */
export type SafetyCheckType =
  | 'idle_trip'
  | 'route_deviation'
  | 'extended_trip'
  | 'late_night'
  | 'unusual_activity';

/**
 * Safety check
 */
export interface SafetyCheck {
  checkId: string;
  tripId: string;
  type: SafetyCheckType;
  timestamp: Date | string;
  severity: 'info' | 'warning' | 'critical';
  message: string;
  requiresResponse: boolean;
  response?: {
    timestamp: Date | string;
    status: 'ok' | 'need_help' | 'emergency';
    message?: string;
  };
}

/**
 * Emergency alert
 */
export interface EmergencyAlert {
  alertId: string;
  tripId: string;
  userId: string;
  userType: 'rider' | 'driver';
  timestamp: Date | string;
  location: Coordinates;
  type: 'discreet' | 'emergency_call';
  status: 'active' | 'responded' | 'resolved' | 'false_alarm';
  responders?: string[];
  notes?: string;
}

/**
 * Incident report
 */
export interface IncidentReport {
  reportId: string;
  tripId: string;
  reporterId: string;
  reporterType: 'rider' | 'driver';
  type:
    | 'safety_concern'
    | 'accident'
    | 'harassment'
    | 'lost_item'
    | 'dispute'
    | 'vehicle_condition'
    | 'behavior'
    | 'other';
  severity: 'low' | 'medium' | 'high' | 'critical';
  description: string;
  timestamp: Date | string;
  status: 'submitted' | 'under_review' | 'resolved' | 'closed';
  resolution?: string;
  actionTaken?: string;
}

// ============================================================================
// Route Optimization
// ============================================================================

/**
 * Route optimization options
 */
export type OptimizationGoal = 'time' | 'distance' | 'cost' | 'eco';

/**
 * Route optimization request
 */
export interface RouteOptimizationRequest {
  waypoints: Coordinates[];
  optimize: OptimizationGoal;
  avoidHighways?: boolean;
  avoidTolls?: boolean;
  avoidFerries?: boolean;
  vehicleType?: VehicleType;
  departureTime?: Date | string;
}

/**
 * Optimized route result
 */
export interface OptimizedRoute {
  route: Route;
  optimizationGoal: OptimizationGoal;
  efficiencyScore: number; // 0-1
  carbonEmissions: number; // kg CO₂
  estimatedCost: number;
  alternatives?: Route[];
  warnings?: string[];
}

/**
 * Multi-stop trip for carpooling
 */
export interface MultiStopTrip {
  tripId: string;
  driverId: string;
  passengers: PassengerSegment[];
  route: Route;
  optimizedWaypoints: Coordinates[];
  totalDistance: number;
  totalDuration: number;
  efficiency: number;
}

/**
 * Passenger segment in multi-stop trip
 */
export interface PassengerSegment {
  riderId: string;
  pickup: Location;
  destination: Location;
  pickupIndex: number; // Index in waypoints
  dropoffIndex: number;
  detourTime: number; // Extra time due to carpooling
  discount: number; // Discount percentage
  fare: number;
}

// ============================================================================
// Environmental Impact
// ============================================================================

/**
 * Carbon footprint calculation
 */
export interface CarbonFootprint {
  tripId: string;
  totalEmissions: number; // kg CO₂
  emissionsPerKm: number;
  comparisonToAlternatives: {
    personalCar: number;
    publicTransit: number;
    taxi: number;
  };
  offset?: {
    purchased: boolean;
    amount: number;
    cost: number;
    certificate?: string;
  };
}

/**
 * Environmental impact metrics
 */
export interface EnvironmentalMetrics {
  totalTrips: number;
  totalDistance: number; // km
  totalEmissions: number; // kg CO₂
  emissionsSaved: number; // vs personal car
  treesEquivalent: number; // Trees needed to offset
  carpoolingRate: number; // Percentage
  evUsageRate: number; // Percentage of trips in EVs
}

// ============================================================================
// Analytics and Reporting
// ============================================================================

/**
 * Driver analytics
 */
export interface DriverAnalytics {
  driverId: string;
  period: {
    start: Date | string;
    end: Date | string;
  };
  trips: {
    total: number;
    completed: number;
    cancelled: number;
    acceptanceRate: number;
    completionRate: number;
  };
  earnings: {
    gross: number;
    net: number;
    tips: number;
    bonuses: number;
    averagePerTrip: number;
    averagePerHour: number;
  };
  performance: {
    rating: number;
    onTimeRate: number;
    cancellationRate: number;
    efficiencyScore: number;
  };
  drivingBehavior: {
    safetyScore: number;
    hardBrakingEvents: number;
    harshAccelerationEvents: number;
    speedingEvents: number;
  };
}

/**
 * Platform analytics
 */
export interface PlatformAnalytics {
  period: {
    start: Date | string;
    end: Date | string;
  };
  trips: {
    total: number;
    completed: number;
    cancelled: number;
    averageDuration: number;
    averageDistance: number;
  };
  revenue: {
    gross: number;
    driverEarnings: number;
    platformFees: number;
    averageFare: number;
  };
  users: {
    activeRiders: number;
    activeDrivers: number;
    newSignups: number;
    retention: number;
  };
  performance: {
    averageWaitTime: number;
    matchSuccessRate: number;
    averageRating: number;
    peakHours: string[];
  };
}

// ============================================================================
// Physical Constants and Configuration
// ============================================================================

/**
 * Configuration constants for ride sharing
 */
export const RIDE_SHARING_CONSTANTS = {
  /** Maximum pickup distance (km) */
  MAX_PICKUP_DISTANCE: 10,

  /** Maximum wait time (seconds) */
  MAX_WAIT_TIME: 900, // 15 minutes

  /** Minimum driver rating */
  MIN_DRIVER_RATING: 3.0,

  /** Minimum rider rating */
  MIN_RIDER_RATING: 3.0,

  /** Maximum surge multiplier */
  MAX_SURGE_MULTIPLIER: 5.0,

  /** Surge sensitivity constant */
  SURGE_SENSITIVITY: 1.0,

  /** Maximum continuous driving (hours) */
  MAX_DRIVING_HOURS: 8,

  /** Mandatory break duration (minutes) */
  MANDATORY_BREAK: 30,

  /** GPS update interval during trip (seconds) */
  GPS_UPDATE_INTERVAL_TRIP: 5,

  /** GPS update interval waiting (seconds) */
  GPS_UPDATE_INTERVAL_WAITING: 30,

  /** Maximum detour time for carpooling (minutes) */
  MAX_CARPOOL_DETOUR: 15,

  /** Commission rate (percentage) */
  PLATFORM_COMMISSION: 0.20, // 20%

  /** Service fee (percentage) */
  SERVICE_FEE: 0.15, // 15%
} as const;

/**
 * Matching weight defaults
 */
export const DEFAULT_MATCHING_WEIGHTS = {
  distance: 0.35,
  time: 0.25,
  rating: 0.20,
  preference: 0.10,
  vehicle: 0.10,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  page: number;
  pageSize: number;
  total: number;
  hasMore: boolean;
}

/**
 * API error
 */
export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, unknown>;
  timestamp: Date | string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-014 error codes
 */
export enum RideErrorCode {
  // General
  INVALID_PARAMETERS = 'R001',
  UNAUTHORIZED = 'R002',
  NOT_FOUND = 'R003',

  // Ride Request
  NO_DRIVERS_AVAILABLE = 'R101',
  PICKUP_TOO_FAR = 'R102',
  INVALID_DESTINATION = 'R103',
  SERVICE_UNAVAILABLE = 'R104',

  // Matching
  MATCH_TIMEOUT = 'R201',
  DRIVER_DECLINED = 'R202',
  CAPACITY_EXCEEDED = 'R203',

  // Payment
  PAYMENT_FAILED = 'R301',
  INSUFFICIENT_FUNDS = 'R302',
  INVALID_PAYMENT_METHOD = 'R303',

  // Safety
  DRIVER_SUSPENDED = 'R401',
  RIDER_SUSPENDED = 'R402',
  SAFETY_CONCERN = 'R403',

  // Trip
  TRIP_CANCELLED = 'R501',
  TRIP_EXPIRED = 'R502',
  INVALID_STATUS = 'R503',
}

/**
 * Ride sharing error
 */
export class RideSharingError extends Error {
  constructor(
    public code: RideErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'RideSharingError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Geography
  Coordinates,
  Address,
  Location,

  // Vehicle
  Vehicle,

  // Users
  DriverProfile,
  RiderProfile,
  Rating,
  VerificationStatus,

  // Ride Request
  RideRequest,
  RidePreferences,
  FareEstimate,
  VehicleTypeEstimate,

  // Trip
  Trip,
  Route,
  TripFare,
  TripRatings,

  // Matching
  MatchingParams,
  DriverMatch,
  MatchingScore,

  // Pricing
  FareCalculationParams,
  FareResult,
  SurgePricing,
  DemandHeatmap,

  // Payment
  PaymentMethod,
  PaymentTransaction,
  SplitPayment,

  // Safety
  SafetyCheck,
  EmergencyAlert,
  IncidentReport,

  // Route Optimization
  RouteOptimizationRequest,
  OptimizedRoute,
  MultiStopTrip,

  // Environmental
  CarbonFootprint,
  EnvironmentalMetrics,

  // Analytics
  DriverAnalytics,
  PlatformAnalytics,
};

export {
  RIDE_SHARING_CONSTANTS,
  DEFAULT_MATCHING_WEIGHTS,
  RideErrorCode,
  RideSharingError,
};
