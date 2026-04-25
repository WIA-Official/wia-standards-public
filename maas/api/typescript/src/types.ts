/**
 * WIA-AUTO-025: Mobility-as-a-Service - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive & Mobility Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic Types
// ============================================================================

/**
 * Geographic coordinates (WGS84)
 */
export interface Location {
  lat: number; // Latitude (-90 to 90)
  lng: number; // Longitude (-180 to 180)
  altitude?: number; // meters above sea level
  accuracy?: number; // meters
}

/**
 * Bounding box for geographic area
 */
export interface BoundingBox {
  north: number;
  south: number;
  east: number;
  west: number;
}

/**
 * Address information
 */
export interface Address {
  street?: string;
  number?: string;
  city: string;
  state?: string;
  country: string;
  postalCode?: string;
  formattedAddress?: string;
}

/**
 * Place (location with metadata)
 */
export interface Place {
  location: Location;
  address?: Address;
  name?: string;
  type?: 'home' | 'work' | 'poi' | 'transit-station' | 'custom';
  placeId?: string;
}

// ============================================================================
// Transport Modes
// ============================================================================

/**
 * All supported transport modes
 */
export type TransportMode =
  | 'walking'
  | 'cycling'
  | 'e-bike'
  | 'e-scooter'
  | 'bike-share'
  | 'scooter-share'
  | 'moped-share'
  | 'bus'
  | 'metro'
  | 'subway'
  | 'tram'
  | 'light-rail'
  | 'train'
  | 'commuter-rail'
  | 'intercity-rail'
  | 'ferry'
  | 'cable-car'
  | 'funicular'
  | 'ride-hail'
  | 'ride-share'
  | 'taxi'
  | 'car-share'
  | 'car-pool'
  | 'private-car'
  | 'motorcycle'
  | 'flight';

/**
 * Mode category
 */
export type ModeCategory =
  | 'active' // walking, cycling
  | 'micromobility' // e-scooter, e-bike, bike-share
  | 'public-transit' // bus, metro, train
  | 'shared-vehicle' // car-share, ride-hail
  | 'private-vehicle' // private car
  | 'other';

/**
 * Mode characteristics
 */
export interface ModeInfo {
  mode: TransportMode;
  category: ModeCategory;
  isShared: boolean;
  requiresBooking: boolean;
  isElectric: boolean;
  typicalSpeed: number; // km/h
  carbonIntensity: number; // kg CO2 per passenger-km
  costPerKm?: number; // average cost
  accessibility: {
    wheelchairAccessible: boolean;
    requiresPhysicalAbility: 'none' | 'low' | 'medium' | 'high';
  };
}

// ============================================================================
// MaaS Platform
// ============================================================================

/**
 * MaaS integration level (Sochor et al. 2018)
 */
export type IntegrationLevel = 0 | 1 | 2 | 3 | 4;

/**
 * MaaS platform configuration
 */
export interface MaaSConfig {
  apiKey: string;
  region: string;
  integrationLevel: IntegrationLevel;
  baseUrl?: string;
  language?: string;
  currency?: string;
}

/**
 * MaaS platform capabilities
 */
export interface PlatformCapabilities {
  integrationLevel: IntegrationLevel;
  features: {
    informationIntegration: boolean;
    bookingIntegration: boolean;
    paymentIntegration: boolean;
    subscriptionServices: boolean;
    policyCoordination: boolean;
    realTimeUpdates: boolean;
    carbonTracking: boolean;
    accessibility: boolean;
  };
  supportedModes: TransportMode[];
  supportedPaymentMethods: PaymentMethod[];
  dataFormats: DataFormat[];
}

// ============================================================================
// Journey Planning
// ============================================================================

/**
 * Journey planning request
 */
export interface JourneyPlanRequest {
  origin: Place | Location;
  destination: Place | Location;
  time: {
    type: 'departure' | 'arrival';
    datetime: Date | string;
  };
  preferences: JourneyPreferences;
  user?: {
    id: string;
    subscription?: SubscriptionPlan;
    mobilityProfile?: MobilityProfile;
  };
}

/**
 * Journey preferences
 */
export interface JourneyPreferences {
  optimize: 'time' | 'cost' | 'carbon' | 'comfort' | 'balanced';
  modes?: TransportMode[];
  avoidModes?: TransportMode[];
  maxWalkingDistance?: number; // meters
  maxCyclingDistance?: number; // meters
  maxTransfers?: number;
  minTransferTime?: number; // seconds
  accessibility?: AccessibilityRequirements;
  avoidTolls?: boolean;
  avoidHighways?: boolean;
  carbonPriority?: 'low' | 'medium' | 'high';
  requireSeating?: boolean;
  requireQuiet?: boolean;
}

/**
 * Journey planning response
 */
export interface JourneyPlanResponse {
  journeys: Journey[];
  metadata: {
    searchId: string;
    searchTime: number; // milliseconds
    alternativesCount: number;
    dataFreshness: Date;
    query: JourneyPlanRequest;
  };
}

/**
 * Journey (multimodal trip)
 */
export interface Journey {
  id: string;
  legs: Leg[];
  summary: JourneySummary;
  accessibility: AccessibilityInfo;
  realTimeUpdates: boolean;
  bookable: boolean;
  pricing: PricingInfo;
  carbonFootprint: CarbonFootprint;
  alternatives?: Journey[];
}

/**
 * Journey summary
 */
export interface JourneySummary {
  duration: number; // seconds
  distance: number; // meters
  walkingDistance: number; // meters
  cyclingDistance: number; // meters
  transfers: number;
  departureTime: Date;
  arrivalTime: Date;
  modes: TransportMode[];
  comfortScore: number; // 0-100
  reliabilityScore: number; // 0-100
  sustainabilityScore: number; // 0-100
}

/**
 * Journey leg (single mode segment)
 */
export interface Leg {
  id: string;
  mode: TransportMode;
  from: Stop;
  to: Stop;
  departureTime: Date;
  arrivalTime: Date;
  duration: number; // seconds
  distance: number; // meters
  operator?: Operator;
  route?: Route;
  vehicle?: Vehicle;
  fare?: Fare;
  booking?: BookingInfo;
  realTime: boolean;
  delay?: number; // seconds (positive = late, negative = early)
  instructions?: string[];
  intermediateStops?: Stop[];
  geometry?: GeoJSONLineString;
  accessibility: AccessibilityInfo;
  occupancy?: OccupancyLevel;
}

/**
 * Stop or station
 */
export interface Stop {
  id: string;
  name: string;
  location: Location;
  platform?: string;
  zone?: string;
  stopType: 'stop' | 'station' | 'dock' | 'parking' | 'pickup-point';
  amenities?: Amenity[];
  accessibility: AccessibilityInfo;
}

/**
 * Transport operator
 */
export interface Operator {
  id: string;
  name: string;
  logo?: string;
  website?: string;
  phone?: string;
  email?: string;
  type: 'public' | 'private' | 'shared-mobility' | 'other';
}

/**
 * Route information
 */
export interface Route {
  id: string;
  name: string;
  shortName?: string;
  color?: string;
  textColor?: string;
  type: TransportMode;
  headsign?: string;
}

/**
 * Vehicle information
 */
export interface Vehicle {
  id: string;
  type: TransportMode;
  registration?: string;
  model?: string;
  capacity: {
    seats?: number;
    standing?: number;
    bicycles?: number;
    wheelchairs?: number;
  };
  amenities?: Amenity[];
  fuelType?: 'gasoline' | 'diesel' | 'electric' | 'hybrid' | 'hydrogen' | 'human';
  emissionClass?: string;
  accessibility: AccessibilityInfo;
}

/**
 * Amenity
 */
export type Amenity =
  | 'wifi'
  | 'power-outlet'
  | 'restroom'
  | 'bike-rack'
  | 'luggage-storage'
  | 'air-conditioning'
  | 'elevator'
  | 'escalator'
  | 'waiting-area'
  | 'ticket-machine'
  | 'help-point'
  | 'food-beverage';

/**
 * Occupancy level
 */
export type OccupancyLevel =
  | 'empty'
  | 'many-seats-available'
  | 'few-seats-available'
  | 'standing-room-only'
  | 'crushed-standing-room-only'
  | 'full'
  | 'not-accepting-passengers';

// ============================================================================
// Accessibility
// ============================================================================

/**
 * Accessibility requirements
 */
export interface AccessibilityRequirements {
  mobilityAids?: {
    wheelchair?: boolean;
    walker?: boolean;
    cane?: boolean;
    serviceAnimal?: boolean;
  };
  sensoryNeeds?: {
    visualImpairment?: 'none' | 'low-vision' | 'blind';
    hearingImpairment?: 'none' | 'hard-of-hearing' | 'deaf';
    speechImpairment?: boolean;
  };
  cognitiveSupport?: {
    simplifiedInstructions?: boolean;
    extraTime?: boolean;
    companionRequired?: boolean;
  };
  preferences?: {
    maxWalkingDistance?: number;
    avoidStairs?: boolean;
    requireSeating?: boolean;
    preferQuietCarriage?: boolean;
    requireAssistance?: boolean;
  };
}

/**
 * Accessibility information
 */
export interface AccessibilityInfo {
  wheelchairAccessible: boolean;
  stepFreeAccess: boolean;
  audioGuidanceAvailable: boolean;
  visualGuidanceAvailable: boolean;
  tactilePathsAvailable: boolean;
  assistanceAvailable: boolean;
  elevatorAvailable: boolean;
  escalatorAvailable: boolean;
  rampAvailable: boolean;
  automaticDoorsAvailable: boolean;
  serviceAnimalAllowed: boolean;
  hearingLoopAvailable: boolean;
  accessibilityScore: number; // 0-100
  limitations?: string[];
}

// ============================================================================
// Booking and Ticketing
// ============================================================================

/**
 * Booking request
 */
export interface BookingRequest {
  journeyId: string;
  legs?: string[]; // Specific leg IDs to book (if not all)
  passengers: number;
  paymentMethod: PaymentMethod | PaymentDetails;
  specialRequirements?: SpecialRequirements;
  contactInfo?: ContactInfo;
}

/**
 * Booking response
 */
export interface BookingResponse {
  bookingId: string;
  status: BookingStatus;
  journey: Journey;
  tickets: Ticket[];
  totalCost: Money;
  validFrom: Date;
  validUntil: Date;
  cancellationPolicy?: CancellationPolicy;
  confirmationCode?: string;
  qrCode?: string;
  deepLink?: string;
}

/**
 * Booking status
 */
export type BookingStatus =
  | 'pending'
  | 'confirmed'
  | 'in-progress'
  | 'completed'
  | 'cancelled'
  | 'failed'
  | 'refunded';

/**
 * Ticket
 */
export interface Ticket {
  id: string;
  type: TicketType;
  leg?: Leg;
  validFrom: Date;
  validUntil: Date;
  qrCode?: string;
  barcode?: string;
  nfcToken?: string;
  deepLink?: string;
  status: 'valid' | 'used' | 'expired' | 'cancelled';
  fare: Fare;
}

/**
 * Ticket type
 */
export type TicketType =
  | 'single-journey'
  | 'return-journey'
  | 'day-pass'
  | 'multi-ride'
  | 'subscription'
  | 'free';

/**
 * Fare information
 */
export interface Fare {
  amount: Money;
  fareClass?: 'standard' | 'premium' | 'economy' | 'first-class';
  discounts?: Discount[];
  includedInSubscription: boolean;
  refundable: boolean;
}

/**
 * Discount
 */
export interface Discount {
  type: 'student' | 'senior' | 'child' | 'subscription' | 'loyalty' | 'promotional';
  amount: Money;
  percentage?: number;
  description: string;
}

/**
 * Special requirements
 */
export interface SpecialRequirements {
  accessibility?: AccessibilityRequirements;
  luggage?: {
    count: number;
    oversized: boolean;
  };
  bicycle?: boolean;
  pet?: {
    allowed: boolean;
    type?: 'service-animal' | 'pet';
  };
  assistance?: {
    required: boolean;
    type?: string;
  };
  notes?: string;
}

/**
 * Contact information
 */
export interface ContactInfo {
  name: string;
  email: string;
  phone: string;
  preferredLanguage?: string;
}

/**
 * Cancellation policy
 */
export interface CancellationPolicy {
  refundable: boolean;
  feeStructure: {
    timeBeforeDeparture: number; // seconds
    feePercentage: number; // 0-100
  }[];
  deadline?: Date; // absolute deadline for cancellation
}

// ============================================================================
// Payment
// ============================================================================

/**
 * Payment method
 */
export type PaymentMethod =
  | 'credit-card'
  | 'debit-card'
  | 'mobile-wallet'
  | 'account-balance'
  | 'subscription'
  | 'invoice'
  | 'external';

/**
 * Payment details
 */
export interface PaymentDetails {
  method: PaymentMethod;
  cardToken?: string; // Tokenized card
  walletType?: 'apple-pay' | 'google-pay' | 'samsung-pay';
  accountId?: string;
  externalProvider?: string;
}

/**
 * Money
 */
export interface Money {
  amount: number;
  currency: string; // ISO 4217 code
}

/**
 * Pricing information
 */
export interface PricingInfo {
  total: Money;
  breakdown: PriceBreakdown[];
  subscriptionDiscount?: Money;
  dynamicPricing?: {
    applied: boolean;
    multiplier: number; // 1.0 = base price
    reason?: string;
  };
  carbonOffset?: Money;
}

/**
 * Price breakdown
 */
export interface PriceBreakdown {
  description: string;
  amount: Money;
  quantity?: number;
  legId?: string;
}

// ============================================================================
// Subscriptions
// ============================================================================

/**
 * Subscription tier
 */
export type SubscriptionTier = 'basic' | 'light' | 'standard' | 'premium' | 'family';

/**
 * Subscription plan
 */
export interface SubscriptionPlan {
  id: string;
  tier: SubscriptionTier;
  name: string;
  description: string;
  price: Money;
  billingPeriod: 'monthly' | 'quarterly' | 'annual';
  credits: {
    total: number;
    remaining: number;
    rollover: boolean;
    maxRollover?: number;
  };
  includedModes: TransportMode[];
  benefits: Benefit[];
  validFrom: Date;
  validUntil: Date;
  autoRenew: boolean;
  cancellationPolicy: CancellationPolicy;
}

/**
 * Benefit
 */
export interface Benefit {
  type: 'unlimited-transit' | 'discount' | 'priority-booking' | 'carbon-offset' | 'support' | 'other';
  description: string;
  value?: string;
  modes?: TransportMode[];
}

/**
 * Mobility credits
 */
export interface MobilityCredits {
  total: number;
  used: number;
  remaining: number;
  expires?: Date;
  history: CreditTransaction[];
}

/**
 * Credit transaction
 */
export interface CreditTransaction {
  id: string;
  type: 'allocation' | 'usage' | 'bonus' | 'rollover' | 'expiration' | 'purchase';
  amount: number;
  balance: number;
  timestamp: Date;
  description: string;
  relatedBooking?: string;
}

// ============================================================================
// User Profile
// ============================================================================

/**
 * User profile
 */
export interface UserProfile {
  id: string;
  name: string;
  email: string;
  phone?: string;
  language: string;
  currency: string;
  subscription?: SubscriptionPlan;
  mobilityProfile: MobilityProfile;
  preferences: UserPreferences;
  paymentMethods: PaymentDetails[];
  savedPlaces: Place[];
  accessibility?: AccessibilityRequirements;
}

/**
 * Mobility profile
 */
export interface MobilityProfile {
  preferredModes: TransportMode[];
  avoidedModes: TransportMode[];
  typicalOrigins: Place[];
  typicalDestinations: Place[];
  commutePattern?: {
    home: Place;
    work: Place;
    frequency: 'daily' | 'weekly' | 'occasional';
  };
  averageTripsPerMonth: number;
  sustainabilityPriority: 'low' | 'medium' | 'high';
}

/**
 * User preferences
 */
export interface UserPreferences {
  defaultOptimization: 'time' | 'cost' | 'carbon' | 'comfort' | 'balanced';
  maxWalkingDistance: number;
  notifications: {
    bookingConfirmations: boolean;
    realTimeUpdates: boolean;
    delays: boolean;
    promotions: boolean;
    carbonReports: boolean;
  };
  privacy: {
    shareLocationData: boolean;
    shareTravelHistory: boolean;
    anonymousAnalytics: boolean;
  };
}

// ============================================================================
// Real-Time Updates
// ============================================================================

/**
 * Real-time update
 */
export interface RealTimeUpdate {
  id: string;
  type: UpdateType;
  timestamp: Date;
  severity: 'info' | 'warning' | 'critical';
  affectedLegs?: string[];
  affectedStops?: string[];
  message: string;
  action?: UpdateAction;
}

/**
 * Update type
 */
export type UpdateType =
  | 'delay'
  | 'cancellation'
  | 'platform-change'
  | 'route-diversion'
  | 'service-alert'
  | 'occupancy-update'
  | 'vehicle-change'
  | 'weather-alert';

/**
 * Update action
 */
export interface UpdateAction {
  type: 'reroute' | 'wait' | 'cancel' | 'no-action';
  alternatives?: Journey[];
  estimatedImpact: {
    delayMinutes: number;
    additionalCost?: Money;
  };
  userDecisionRequired: boolean;
  deadline?: Date;
}

/**
 * Service alert
 */
export interface ServiceAlert {
  id: string;
  title: string;
  description: string;
  severity: 'info' | 'warning' | 'critical';
  affectedModes: TransportMode[];
  affectedRoutes?: string[];
  affectedArea?: BoundingBox;
  validFrom: Date;
  validUntil?: Date;
  url?: string;
}

// ============================================================================
// Mobility Options
// ============================================================================

/**
 * Mobility options request
 */
export interface MobilityOptionsRequest {
  location: Location;
  radius: number; // meters
  modes?: TransportMode[];
  includeAvailability?: boolean;
  includeRealTime?: boolean;
}

/**
 * Mobility option
 */
export interface MobilityOption {
  mode: TransportMode;
  provider: Operator;
  location: Location;
  distance: number; // meters from query location
  available: number; // number of vehicles/bikes available
  capacity?: number; // total capacity at this location
  battery?: number; // percentage for electric vehicles
  pricing: {
    unlockFee?: Money;
    perMinute?: Money;
    perKm?: Money;
    perHour?: Money;
  };
  bookable: boolean;
  deepLink?: string;
}

// ============================================================================
// Carbon Footprint
// ============================================================================

/**
 * Carbon footprint
 */
export interface CarbonFootprint {
  totalEmissions: number; // kg CO2
  emissionsByMode: {
    mode: TransportMode;
    emissions: number; // kg CO2
    distance: number; // km
  }[];
  baseline: number; // kg CO2 (private car equivalent)
  saved: number; // kg CO2 saved vs baseline
  savingsPercentage: number; // 0-100
  offset?: CarbonOffset;
}

/**
 * Carbon offset
 */
export interface CarbonOffset {
  amount: number; // kg CO2
  cost: Money;
  project: {
    name: string;
    type: 'renewable-energy' | 'reforestation' | 'carbon-capture' | 'other';
    certification: string;
    url?: string;
  };
  certificateUrl?: string;
}

/**
 * Carbon report
 */
export interface CarbonReport {
  userId: string;
  period: {
    start: Date;
    end: Date;
  };
  totalEmissions: number; // kg CO2
  totalDistance: number; // km
  totalTrips: number;
  emissionsByMode: {
    mode: TransportMode;
    emissions: number;
    trips: number;
    distance: number;
  }[];
  baseline: number; // kg CO2 if all trips by car
  totalSaved: number; // kg CO2
  offsetPurchased?: number; // kg CO2
  trends: {
    comparedToPrevious: number; // percentage change
    averagePerTrip: number; // kg CO2 per trip
  };
  recommendations: string[];
}

// ============================================================================
// Trip History
// ============================================================================

/**
 * Trip
 */
export interface Trip {
  id: string;
  booking: BookingResponse;
  journey: Journey;
  startTime: Date;
  endTime?: Date;
  status: TripStatus;
  feedback?: TripFeedback;
  cost: Money;
  carbonEmissions: number; // kg CO2
}

/**
 * Trip status
 */
export type TripStatus =
  | 'upcoming'
  | 'in-progress'
  | 'completed'
  | 'cancelled'
  | 'missed';

/**
 * Trip feedback
 */
export interface TripFeedback {
  rating: number; // 1-5
  comfort: number; // 1-5
  punctuality: number; // 1-5
  cleanliness: number; // 1-5
  value: number; // 1-5
  comments?: string;
  issues?: TripIssue[];
  timestamp: Date;
}

/**
 * Trip issue
 */
export interface TripIssue {
  type: 'delay' | 'cancellation' | 'overcrowding' | 'cleanliness' | 'safety' | 'accessibility' | 'other';
  severity: 'minor' | 'moderate' | 'severe';
  description: string;
  legId?: string;
}

// ============================================================================
// Data Formats
// ============================================================================

/**
 * Supported data formats
 */
export type DataFormat =
  | 'GTFS'
  | 'GTFS-RT'
  | 'NeTEx'
  | 'SIRI'
  | 'MDS'
  | 'GBFS'
  | 'TOMP-API';

/**
 * GeoJSON LineString
 */
export interface GeoJSONLineString {
  type: 'LineString';
  coordinates: [number, number][]; // [lng, lat]
}

// ============================================================================
// Error Handling
// ============================================================================

/**
 * WIA-AUTO-025 error codes
 */
export enum MaaSErrorCode {
  INVALID_REQUEST = 'MAAS001',
  INVALID_LOCATION = 'MAAS002',
  NO_ROUTES_FOUND = 'MAAS003',
  BOOKING_FAILED = 'MAAS004',
  PAYMENT_DECLINED = 'MAAS005',
  INSUFFICIENT_CREDITS = 'MAAS006',
  SERVICE_UNAVAILABLE = 'MAAS007',
  OPERATOR_ERROR = 'MAAS008',
  REAL_TIME_DATA_UNAVAILABLE = 'MAAS009',
  SUBSCRIPTION_EXPIRED = 'MAAS010',
  INVALID_TICKET = 'MAAS011',
  CANCELLATION_NOT_ALLOWED = 'MAAS012',
  ACCESSIBILITY_NOT_AVAILABLE = 'MAAS013',
  RATE_LIMIT_EXCEEDED = 'MAAS014',
  AUTHENTICATION_FAILED = 'MAAS015',
}

/**
 * MaaS error
 */
export class MaaSError extends Error {
  constructor(
    public code: MaaSErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MaaSError';
  }
}

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
 * Pagination
 */
export interface Pagination {
  page: number;
  pageSize: number;
  totalPages: number;
  totalItems: number;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  pagination: Pagination;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * MaaS constants
 */
export const MAAS_CONSTANTS = {
  /** Default maximum walking distance in meters */
  DEFAULT_MAX_WALKING_DISTANCE: 1000,

  /** Default maximum cycling distance in meters */
  DEFAULT_MAX_CYCLING_DISTANCE: 5000,

  /** Default maximum number of transfers */
  DEFAULT_MAX_TRANSFERS: 3,

  /** Minimum transfer time in seconds */
  MIN_TRANSFER_TIME: 120,

  /** Maximum journey duration in seconds (24 hours) */
  MAX_JOURNEY_DURATION: 86400,

  /** Real-time data freshness threshold in seconds */
  REAL_TIME_FRESHNESS_THRESHOLD: 300,

  /** Carbon emission factors (kg CO2 per passenger-km) */
  CARBON_FACTORS: {
    walking: 0.0,
    cycling: 0.0,
    'e-bike': 0.005,
    'e-scooter': 0.008,
    'bus-electric': 0.027,
    'bus-diesel': 0.089,
    metro: 0.041,
    'train-electric': 0.041,
    'train-diesel': 0.085,
    'ride-hail-solo': 0.192,
    'ride-share-pooled': 0.096,
    'car-share-electric': 0.053,
    'car-share-gas': 0.171,
    'private-car': 0.192,
  },

  /** API rate limits */
  RATE_LIMITS: {
    journey_planning: 60, // requests per minute
    booking: 30,
    real_time_updates: 120,
  },
} as const;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Geographic
  Location,
  BoundingBox,
  Address,
  Place,

  // Modes
  TransportMode,
  ModeCategory,
  ModeInfo,

  // Platform
  IntegrationLevel,
  MaaSConfig,
  PlatformCapabilities,

  // Journey Planning
  JourneyPlanRequest,
  JourneyPreferences,
  JourneyPlanResponse,
  Journey,
  JourneySummary,
  Leg,
  Stop,
  Operator,
  Route,
  Vehicle,
  Amenity,
  OccupancyLevel,

  // Accessibility
  AccessibilityRequirements,
  AccessibilityInfo,

  // Booking
  BookingRequest,
  BookingResponse,
  BookingStatus,
  Ticket,
  TicketType,
  Fare,
  Discount,
  SpecialRequirements,
  ContactInfo,
  CancellationPolicy,

  // Payment
  PaymentMethod,
  PaymentDetails,
  Money,
  PricingInfo,
  PriceBreakdown,

  // Subscriptions
  SubscriptionTier,
  SubscriptionPlan,
  Benefit,
  MobilityCredits,
  CreditTransaction,

  // User
  UserProfile,
  MobilityProfile,
  UserPreferences,

  // Real-Time
  RealTimeUpdate,
  UpdateType,
  UpdateAction,
  ServiceAlert,

  // Mobility Options
  MobilityOptionsRequest,
  MobilityOption,

  // Carbon
  CarbonFootprint,
  CarbonOffset,
  CarbonReport,

  // Trips
  Trip,
  TripStatus,
  TripFeedback,
  TripIssue,

  // Data
  DataFormat,
  GeoJSONLineString,

  // Utility
  Pagination,
  PaginatedResponse,
};

export { MAAS_CONSTANTS, MaaSErrorCode, MaaSError };
