/**
 * WIA-IND-016: Hotel Tech - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Hotel Types
// ============================================================================

/**
 * Hotel property types
 */
export type PropertyType =
  | 'luxury'
  | 'business'
  | 'budget'
  | 'boutique'
  | 'resort'
  | 'serviced-apartment'
  | 'hostel'
  | 'vacation-rental';

/**
 * Room types
 */
export type RoomType =
  | 'standard-single'
  | 'standard-double'
  | 'deluxe-king'
  | 'deluxe-queen'
  | 'suite-junior'
  | 'suite-executive'
  | 'suite-presidential'
  | 'connecting'
  | 'accessible';

/**
 * Room status
 */
export type RoomStatus =
  | 'available'
  | 'reserved'
  | 'occupied'
  | 'dirty'
  | 'cleaning'
  | 'clean'
  | 'inspected'
  | 'out-of-order'
  | 'out-of-service';

/**
 * Reservation status
 */
export type ReservationStatus =
  | 'confirmed'
  | 'pending'
  | 'checked-in'
  | 'checked-out'
  | 'cancelled'
  | 'no-show'
  | 'waitlist';

/**
 * Payment methods
 */
export type PaymentMethod = 'credit-card' | 'debit-card' | 'cash' | 'mobile-pay' | 'crypto' | 'invoice';

/**
 * Rate codes
 */
export type RateCode = 'BAR' | 'CORP' | 'GOV' | 'AAA' | 'PROMO' | 'PKG' | 'GROUP' | 'LRA';

// ============================================================================
// Property Management
// ============================================================================

/**
 * Hotel property configuration
 */
export interface PropertyConfig {
  /** Property unique identifier */
  propertyId: string;

  /** Property name */
  name: string;

  /** Property type */
  type: PropertyType;

  /** Property address */
  address: Address;

  /** Contact information */
  contact: ContactInfo;

  /** Total number of rooms */
  totalRooms: number;

  /** Room types available */
  roomTypes: RoomTypeConfig[];

  /** Property amenities */
  amenities: string[];

  /** Check-in time */
  checkInTime: string;

  /** Check-out time */
  checkOutTime: string;

  /** Default currency */
  currency: string;

  /** Timezone */
  timezone: string;

  /** PMS provider */
  pmsProvider?: string;
}

/**
 * Address information
 */
export interface Address {
  /** Street address */
  street: string;

  /** City */
  city: string;

  /** State/Province */
  state: string;

  /** Postal code */
  postalCode: string;

  /** Country */
  country: string;

  /** Latitude */
  latitude?: number;

  /** Longitude */
  longitude?: number;
}

/**
 * Contact information
 */
export interface ContactInfo {
  /** Phone number */
  phone: string;

  /** Email */
  email: string;

  /** Website */
  website?: string;

  /** Social media */
  socialMedia?: {
    facebook?: string;
    instagram?: string;
    twitter?: string;
  };
}

/**
 * Room type configuration
 */
export interface RoomTypeConfig {
  /** Room type code */
  code: string;

  /** Room type name */
  name: string;

  /** Room type */
  type: RoomType;

  /** Number of this room type */
  quantity: number;

  /** Maximum occupancy */
  maxOccupancy: number;

  /** Bed configuration */
  beds: BedConfig[];

  /** Room size in sq meters */
  size: number;

  /** Room amenities */
  amenities: string[];

  /** Base rate */
  baseRate: number;

  /** Room images */
  images?: string[];
}

/**
 * Bed configuration
 */
export interface BedConfig {
  /** Bed type */
  type: 'king' | 'queen' | 'double' | 'single' | 'twin' | 'sofa-bed';

  /** Quantity */
  quantity: number;
}

// ============================================================================
// Reservations
// ============================================================================

/**
 * Guest reservation
 */
export interface Reservation {
  /** Confirmation number */
  confirmationNumber: string;

  /** Property ID */
  propertyId: string;

  /** Guest information */
  guest: GuestProfile;

  /** Check-in date (ISO 8601) */
  checkIn: string;

  /** Check-out date (ISO 8601) */
  checkOut: string;

  /** Number of nights */
  nights: number;

  /** Room type */
  roomType: string;

  /** Room number (assigned at check-in) */
  roomNumber?: string;

  /** Rate code */
  rateCode: RateCode;

  /** Room rate per night */
  roomRate: number;

  /** Number of adults */
  adults: number;

  /** Number of children */
  children: number;

  /** Special requests */
  specialRequests?: string;

  /** Status */
  status: ReservationStatus;

  /** Source of booking */
  source: BookingSource;

  /** Total amount */
  totalAmount: number;

  /** Taxes and fees */
  taxesAndFees: number;

  /** Deposit amount */
  depositAmount?: number;

  /** Payment method */
  paymentMethod?: PaymentMethod;

  /** Cancellation policy */
  cancellationPolicy: CancellationPolicy;

  /** Created timestamp */
  createdAt: Date;

  /** Last updated timestamp */
  updatedAt: Date;

  /** Additional services */
  services?: AdditionalService[];

  /** Guest preferences */
  preferences?: GuestPreferences;
}

/**
 * Guest profile
 */
export interface GuestProfile {
  /** Guest ID */
  guestId: string;

  /** First name */
  firstName: string;

  /** Last name */
  lastName: string;

  /** Email */
  email: string;

  /** Phone */
  phone: string;

  /** Date of birth */
  dateOfBirth?: string;

  /** Nationality */
  nationality?: string;

  /** Passport number */
  passportNumber?: string;

  /** Address */
  address?: Address;

  /** Loyalty membership */
  loyaltyMembership?: LoyaltyMembership;

  /** VIP status */
  vipStatus?: 'standard' | 'silver' | 'gold' | 'platinum';

  /** Guest preferences */
  preferences?: GuestPreferences;

  /** Stay history */
  stayHistory?: StayHistory[];

  /** Created date */
  createdAt: Date;

  /** Last visit */
  lastVisit?: Date;
}

/**
 * Guest preferences
 */
export interface GuestPreferences {
  /** Room location */
  roomLocation?: 'low-floor' | 'high-floor' | 'quiet' | 'view';

  /** Bed type */
  bedType?: 'king' | 'queen' | 'twin';

  /** Pillow type */
  pillowType?: 'soft' | 'medium' | 'firm' | 'hypoallergenic';

  /** Room temperature */
  temperature?: number;

  /** Newspaper */
  newspaper?: string;

  /** Dietary restrictions */
  dietaryRestrictions?: string[];

  /** Language */
  language?: string;

  /** Communication preference */
  communicationPreference?: 'email' | 'sms' | 'phone' | 'app';
}

/**
 * Stay history
 */
export interface StayHistory {
  /** Property ID */
  propertyId: string;

  /** Check-in date */
  checkIn: string;

  /** Check-out date */
  checkOut: string;

  /** Room type */
  roomType: string;

  /** Total spent */
  totalSpent: number;

  /** Feedback rating */
  rating?: number;
}

/**
 * Loyalty membership
 */
export interface LoyaltyMembership {
  /** Program name */
  program: string;

  /** Membership number */
  memberNumber: string;

  /** Tier level */
  tier: string;

  /** Points balance */
  points: number;
}

/**
 * Booking source
 */
export interface BookingSource {
  /** Channel */
  channel: 'direct' | 'phone' | 'ota' | 'gds' | 'corporate' | 'agent';

  /** Specific source */
  source?: string;

  /** Commission percentage */
  commission?: number;
}

/**
 * Cancellation policy
 */
export interface CancellationPolicy {
  /** Policy type */
  type: 'flexible' | 'moderate' | 'strict' | 'non-refundable';

  /** Free cancellation until */
  freeCancellationUntil?: string;

  /** Penalty percentage */
  penaltyPercentage?: number;

  /** Penalty amount */
  penaltyAmount?: number;
}

/**
 * Additional service
 */
export interface AdditionalService {
  /** Service code */
  code: string;

  /** Service name */
  name: string;

  /** Quantity */
  quantity: number;

  /** Price per unit */
  price: number;

  /** Total price */
  total: number;

  /** Date of service */
  date?: string;
}

// ============================================================================
// Room Management
// ============================================================================

/**
 * Room information
 */
export interface Room {
  /** Room number */
  roomNumber: string;

  /** Property ID */
  propertyId: string;

  /** Room type */
  roomType: string;

  /** Floor */
  floor: number;

  /** Building/Wing */
  building?: string;

  /** Current status */
  status: RoomStatus;

  /** Assigned to reservation */
  reservationId?: string;

  /** Current guest */
  guestId?: string;

  /** Housekeeping status */
  housekeepingStatus: HousekeepingStatus;

  /** Last cleaned */
  lastCleaned?: Date;

  /** Maintenance issues */
  maintenanceIssues?: MaintenanceIssue[];

  /** Room features */
  features?: RoomFeatures;

  /** IoT devices */
  iotDevices?: IoTDevice[];
}

/**
 * Housekeeping status
 */
export interface HousekeepingStatus {
  /** Status */
  status: 'clean' | 'dirty' | 'cleaning' | 'inspected';

  /** Assigned to staff */
  assignedTo?: string;

  /** Started at */
  startedAt?: Date;

  /** Completed at */
  completedAt?: Date;

  /** Inspected by */
  inspectedBy?: string;

  /** Inspection notes */
  inspectionNotes?: string;

  /** Items needed */
  itemsNeeded?: string[];
}

/**
 * Maintenance issue
 */
export interface MaintenanceIssue {
  /** Issue ID */
  issueId: string;

  /** Issue type */
  type: 'electrical' | 'plumbing' | 'hvac' | 'furniture' | 'other';

  /** Description */
  description: string;

  /** Priority */
  priority: 'low' | 'medium' | 'high' | 'urgent';

  /** Status */
  status: 'reported' | 'assigned' | 'in-progress' | 'resolved';

  /** Reported by */
  reportedBy: string;

  /** Reported at */
  reportedAt: Date;

  /** Assigned to */
  assignedTo?: string;

  /** Resolved at */
  resolvedAt?: Date;
}

/**
 * Room features
 */
export interface RoomFeatures {
  /** Smart TV */
  smartTV?: boolean;

  /** Mini bar */
  miniBar?: boolean;

  /** Coffee maker */
  coffeeMaker?: boolean;

  /** Safe */
  safe?: boolean;

  /** Balcony */
  balcony?: boolean;

  /** Bathtub */
  bathtub?: boolean;

  /** Shower */
  shower?: boolean;

  /** Air conditioning */
  airConditioning?: boolean;

  /** Heating */
  heating?: boolean;

  /** WiFi */
  wifi?: boolean;
}

// ============================================================================
// Smart Room & IoT
// ============================================================================

/**
 * IoT device in room
 */
export interface IoTDevice {
  /** Device ID */
  deviceId: string;

  /** Device type */
  type: 'thermostat' | 'lock' | 'light' | 'curtain' | 'tv' | 'sensor' | 'speaker';

  /** Device name */
  name: string;

  /** Device status */
  status: 'online' | 'offline' | 'error';

  /** Current state */
  state?: Record<string, unknown>;

  /** Last updated */
  lastUpdated: Date;
}

/**
 * Smart room controls
 */
export interface SmartRoomControls {
  /** Room number */
  roomNumber: string;

  /** Climate control */
  climate?: ClimateControl;

  /** Lighting */
  lighting?: LightingControl;

  /** Curtains */
  curtains?: CurtainControl;

  /** Entertainment */
  entertainment?: EntertainmentControl;

  /** Voice assistant */
  voiceAssistant?: VoiceAssistantConfig;
}

/**
 * Climate control
 */
export interface ClimateControl {
  /** Mode */
  mode: 'heat' | 'cool' | 'auto' | 'off';

  /** Target temperature (Celsius) */
  targetTemperature: number;

  /** Current temperature */
  currentTemperature: number;

  /** Humidity */
  humidity?: number;

  /** Fan speed */
  fanSpeed?: 'low' | 'medium' | 'high' | 'auto';
}

/**
 * Lighting control
 */
export interface LightingControl {
  /** Lighting scenes */
  scenes: LightingScene[];

  /** Current scene */
  currentScene?: string;

  /** Individual lights */
  lights?: Light[];
}

/**
 * Lighting scene
 */
export interface LightingScene {
  /** Scene name */
  name: string;

  /** Scene ID */
  sceneId: string;

  /** Light settings */
  settings: Record<string, { brightness: number; color?: string }>;
}

/**
 * Individual light
 */
export interface Light {
  /** Light ID */
  lightId: string;

  /** Light name */
  name: string;

  /** Is on */
  isOn: boolean;

  /** Brightness (0-100) */
  brightness: number;

  /** Color temperature */
  colorTemperature?: number;

  /** RGB color */
  color?: string;
}

/**
 * Curtain control
 */
export interface CurtainControl {
  /** Position (0-100, 0=closed) */
  position: number;

  /** Is automated */
  automated: boolean;

  /** Schedule */
  schedule?: {
    openTime?: string;
    closeTime?: string;
  };
}

/**
 * Entertainment control
 */
export interface EntertainmentControl {
  /** TV power */
  tvPower: boolean;

  /** Current channel */
  channel?: number;

  /** Volume */
  volume: number;

  /** Streaming services */
  streamingServices?: string[];

  /** Current input */
  input?: 'tv' | 'hdmi1' | 'hdmi2' | 'streaming';
}

/**
 * Voice assistant configuration
 */
export interface VoiceAssistantConfig {
  /** Enabled */
  enabled: boolean;

  /** Assistant type */
  type: 'alexa' | 'google-assistant' | 'siri' | 'custom';

  /** Language */
  language: string;

  /** Wake word */
  wakeWord?: string;
}

// ============================================================================
// Keyless Entry
// ============================================================================

/**
 * Mobile key
 */
export interface MobileKey {
  /** Key ID */
  keyId: string;

  /** Guest ID */
  guestId: string;

  /** Room number */
  roomNumber: string;

  /** Valid from */
  validFrom: Date;

  /** Valid until */
  validUntil: Date;

  /** Key type */
  keyType: 'mobile' | 'rfid' | 'pin';

  /** Access level */
  accessLevel: AccessLevel;

  /** Key status */
  status: 'active' | 'expired' | 'revoked';

  /** Usage log */
  usageLog?: KeyUsageLog[];
}

/**
 * Access level
 */
export interface AccessLevel {
  /** Room access */
  room: boolean;

  /** Elevator */
  elevator?: boolean;

  /** Pool */
  pool?: boolean;

  /** Gym */
  gym?: boolean;

  /** Spa */
  spa?: boolean;

  /** Executive lounge */
  executiveLounge?: boolean;
}

/**
 * Key usage log
 */
export interface KeyUsageLog {
  /** Timestamp */
  timestamp: Date;

  /** Location */
  location: string;

  /** Access granted */
  accessGranted: boolean;

  /** Denied reason */
  deniedReason?: string;
}

// ============================================================================
// Revenue Management
// ============================================================================

/**
 * Revenue management configuration
 */
export interface RevenueManagementConfig {
  /** Property ID */
  propertyId: string;

  /** Pricing strategy */
  strategy: 'dynamic' | 'fixed' | 'competitive' | 'value-based';

  /** Minimum rate */
  minRate: number;

  /** Maximum rate */
  maxRate: number;

  /** Competitor monitoring */
  competitorMonitoring: boolean;

  /** Event detection */
  eventDetection: boolean;

  /** Forecasting enabled */
  forecasting: boolean;
}

/**
 * Dynamic pricing result
 */
export interface DynamicPricingResult {
  /** Room type */
  roomType: string;

  /** Date */
  date: string;

  /** Base rate */
  baseRate: number;

  /** Recommended rate */
  recommendedRate: number;

  /** Occupancy forecast */
  occupancyForecast: number;

  /** Demand level */
  demandLevel: 'low' | 'medium' | 'high' | 'very-high';

  /** Price adjustment factors */
  factors: PricingFactor[];

  /** Competitor rates */
  competitorRates?: number[];

  /** Confidence score */
  confidence: number;
}

/**
 * Pricing factor
 */
export interface PricingFactor {
  /** Factor name */
  name: string;

  /** Impact percentage */
  impact: number;

  /** Description */
  description: string;
}

/**
 * Occupancy forecast
 */
export interface OccupancyForecast {
  /** Date */
  date: string;

  /** Forecasted occupancy (0-1) */
  forecastedOccupancy: number;

  /** Actual occupancy (0-1) */
  actualOccupancy?: number;

  /** Confidence level */
  confidence: number;

  /** Contributing factors */
  factors: string[];
}

// ============================================================================
// Channel Manager
// ============================================================================

/**
 * Channel configuration
 */
export interface ChannelConfig {
  /** Channel ID */
  channelId: string;

  /** Channel name */
  channelName: string;

  /** Channel type */
  type: 'ota' | 'gds' | 'wholesaler' | 'meta-search';

  /** Enabled */
  enabled: boolean;

  /** API endpoint */
  endpoint: string;

  /** Credentials */
  credentials: {
    username?: string;
    password?: string;
    apiKey?: string;
  };

  /** Commission percentage */
  commission: number;

  /** Rate parity */
  rateParity: boolean;

  /** Last sync */
  lastSync?: Date;
}

/**
 * Channel sync result
 */
export interface ChannelSyncResult {
  /** Channel ID */
  channelId: string;

  /** Sync timestamp */
  timestamp: Date;

  /** Success */
  success: boolean;

  /** Rooms updated */
  roomsUpdated: number;

  /** Rates updated */
  ratesUpdated: number;

  /** Errors */
  errors?: string[];

  /** Warnings */
  warnings?: string[];
}

/**
 * Rate distribution
 */
export interface RateDistribution {
  /** Room type */
  roomType: string;

  /** Date */
  date: string;

  /** Rate plan */
  ratePlan: string;

  /** Rate amount */
  rate: number;

  /** Availability */
  availability: number;

  /** Restrictions */
  restrictions?: RateRestrictions;

  /** Channels */
  channels: string[];
}

/**
 * Rate restrictions
 */
export interface RateRestrictions {
  /** Minimum length of stay */
  minLOS?: number;

  /** Maximum length of stay */
  maxLOS?: number;

  /** Closed to arrival */
  closedToArrival?: boolean;

  /** Closed to departure */
  closedToDeparture?: boolean;

  /** Stop sell */
  stopSell?: boolean;
}

// ============================================================================
// Guest Feedback & Reviews
// ============================================================================

/**
 * Guest feedback
 */
export interface GuestFeedback {
  /** Feedback ID */
  feedbackId: string;

  /** Property ID */
  propertyId: string;

  /** Guest ID */
  guestId: string;

  /** Reservation ID */
  reservationId: string;

  /** Overall rating (1-5) */
  overallRating: number;

  /** Category ratings */
  categoryRatings: CategoryRating[];

  /** Review text */
  reviewText?: string;

  /** Review title */
  reviewTitle?: string;

  /** Would recommend */
  wouldRecommend: boolean;

  /** Stay date */
  stayDate: string;

  /** Submitted date */
  submittedAt: Date;

  /** Source */
  source: 'email' | 'app' | 'website' | 'ota';

  /** Public response */
  publicResponse?: string;

  /** Internal notes */
  internalNotes?: string;
}

/**
 * Category rating
 */
export interface CategoryRating {
  /** Category */
  category: 'cleanliness' | 'staff' | 'location' | 'value' | 'amenities' | 'comfort';

  /** Rating (1-5) */
  rating: number;
}

/**
 * Review analytics
 */
export interface ReviewAnalytics {
  /** Property ID */
  propertyId: string;

  /** Time period */
  period: 'week' | 'month' | 'quarter' | 'year';

  /** Average rating */
  averageRating: number;

  /** Total reviews */
  totalReviews: number;

  /** Rating distribution */
  ratingDistribution: Record<number, number>;

  /** Category averages */
  categoryAverages: Record<string, number>;

  /** Sentiment analysis */
  sentiment?: {
    positive: number;
    neutral: number;
    negative: number;
  };

  /** Common keywords */
  keywords?: { word: string; count: number }[];
}

// ============================================================================
// Concierge Services
// ============================================================================

/**
 * Concierge request
 */
export interface ConciergeRequest {
  /** Request ID */
  requestId: string;

  /** Guest ID */
  guestId: string;

  /** Room number */
  roomNumber: string;

  /** Request type */
  type: 'restaurant' | 'transportation' | 'tour' | 'tickets' | 'spa' | 'other';

  /** Description */
  description: string;

  /** Priority */
  priority: 'low' | 'normal' | 'high' | 'urgent';

  /** Status */
  status: 'pending' | 'in-progress' | 'completed' | 'cancelled';

  /** Assigned to */
  assignedTo?: string;

  /** Created at */
  createdAt: Date;

  /** Completed at */
  completedAt?: Date;

  /** Notes */
  notes?: string;
}

/**
 * Recommendation
 */
export interface Recommendation {
  /** Recommendation ID */
  recommendationId: string;

  /** Type */
  type: 'restaurant' | 'attraction' | 'activity' | 'event';

  /** Name */
  name: string;

  /** Description */
  description: string;

  /** Category */
  category: string;

  /** Distance from hotel (meters) */
  distance: number;

  /** Rating */
  rating?: number;

  /** Price level (1-4) */
  priceLevel?: number;

  /** Address */
  address: Address;

  /** Hours */
  hours?: string;

  /** Reservation link */
  reservationLink?: string;

  /** Images */
  images?: string[];
}

// ============================================================================
// Physical Constants and Limits
// ============================================================================

/**
 * Hotel technology constants
 */
export const HOTEL_CONSTANTS = {
  /** Standard check-in time */
  STANDARD_CHECKIN: '15:00',

  /** Standard check-out time */
  STANDARD_CHECKOUT: '11:00',

  /** Late checkout fee percentage */
  LATE_CHECKOUT_FEE: 0.5,

  /** Early checkin fee percentage */
  EARLY_CHECKIN_FEE: 0.5,

  /** Maximum advance booking days */
  MAX_ADVANCE_BOOKING: 365,

  /** Minimum advance booking hours */
  MIN_ADVANCE_BOOKING: 2,

  /** Default cancellation hours */
  DEFAULT_CANCELLATION_HOURS: 24,

  /** Maximum occupancy per room */
  MAX_OCCUPANCY: 4,

  /** Typical cleaning time (minutes) */
  CLEANING_TIME: 30,

  /** Temperature range (Celsius) */
  TEMPERATURE_RANGE: { min: 16, max: 30 },

  /** Mobile key expiry buffer (hours) */
  KEY_EXPIRY_BUFFER: 2,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-IND-016 error codes
 */
export enum HotelErrorCode {
  ROOM_NOT_AVAILABLE = 'H001',
  RESERVATION_NOT_FOUND = 'H002',
  INVALID_DATES = 'H003',
  PAYMENT_FAILED = 'H004',
  CANCELLATION_NOT_ALLOWED = 'H005',
  OCCUPANCY_EXCEEDED = 'H006',
  RATE_NOT_FOUND = 'H007',
  CHANNEL_SYNC_FAILED = 'H008',
  MOBILE_KEY_FAILED = 'H009',
  PMS_CONNECTION_ERROR = 'H010',
}

/**
 * Hotel technology error
 */
export class HotelTechError extends Error {
  constructor(
    public code: HotelErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'HotelTechError';
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
 * Date range
 */
export interface DateRange {
  /** Start date */
  start: string;

  /** End date */
  end: string;
}

/**
 * Pagination
 */
export interface Pagination {
  /** Page number */
  page: number;

  /** Page size */
  pageSize: number;

  /** Total items */
  total: number;

  /** Total pages */
  totalPages: number;
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Property
  PropertyConfig,
  Address,
  ContactInfo,
  RoomTypeConfig,
  BedConfig,

  // Reservations
  Reservation,
  GuestProfile,
  GuestPreferences,
  StayHistory,
  LoyaltyMembership,
  BookingSource,
  CancellationPolicy,
  AdditionalService,

  // Rooms
  Room,
  HousekeepingStatus,
  MaintenanceIssue,
  RoomFeatures,

  // Smart Room
  IoTDevice,
  SmartRoomControls,
  ClimateControl,
  LightingControl,
  LightingScene,
  Light,
  CurtainControl,
  EntertainmentControl,
  VoiceAssistantConfig,

  // Keyless Entry
  MobileKey,
  AccessLevel,
  KeyUsageLog,

  // Revenue
  RevenueManagementConfig,
  DynamicPricingResult,
  PricingFactor,
  OccupancyForecast,

  // Channels
  ChannelConfig,
  ChannelSyncResult,
  RateDistribution,
  RateRestrictions,

  // Feedback
  GuestFeedback,
  CategoryRating,
  ReviewAnalytics,

  // Concierge
  ConciergeRequest,
  Recommendation,

  // Utilities
  DateRange,
  Pagination,
};

export { HOTEL_CONSTANTS, HotelErrorCode, HotelTechError };

**弘益人間 (홍익인간) · Benefit All Humanity**
