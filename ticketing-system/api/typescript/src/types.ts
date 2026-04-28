/**
 * WIA-IND-019: Ticketing System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Geographic Types
// ============================================================================

/**
 * Geographic coordinates
 */
export interface Coordinates {
  lat: number;
  lon: number;
}

/**
 * Physical location
 */
export interface Location {
  address: string;
  city?: string;
  state?: string;
  country?: string;
  postalCode?: string;
  coordinates?: Coordinates;
  timezone?: string;
}

// ============================================================================
// Venue Types
// ============================================================================

/**
 * Venue type classification
 */
export type VenueType =
  | 'stadium'
  | 'arena'
  | 'theater'
  | 'festival'
  | 'cinema'
  | 'museum'
  | 'conference-center'
  | 'virtual';

/**
 * Venue section capacity
 */
export interface VenueSection {
  capacity: number;
  type: 'seated' | 'standing' | 'accessible';
  priceMultiplier?: number;
}

/**
 * Venue accessibility features
 */
export interface VenueAccessibility {
  wheelchairAccess: boolean;
  elevators: boolean;
  assistiveListening: boolean;
  signLanguage: boolean;
  accessibleParking: boolean;
  serviceAnimals: boolean;
}

/**
 * Venue information
 */
export interface Venue {
  id: string;
  name: string;
  type: VenueType;
  location: Location;
  capacity: {
    total: number;
    sections: Record<string, VenueSection>;
  };
  amenities: string[];
  accessibility: VenueAccessibility;
  contactInfo: {
    phone: string;
    email: string;
    website: string;
  };
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Seat Types
// ============================================================================

/**
 * Seat type classification
 */
export type SeatType = 'standard' | 'VIP' | 'accessible' | 'premium' | 'standing';

/**
 * Seat information
 */
export interface Seat {
  section: string;
  row: string;
  seat: string;
  type: SeatType;
  accessible: boolean;
  companion?: string;
  features?: string[];
}

// ============================================================================
// Ticket Holder Types
// ============================================================================

/**
 * Biometric type
 */
export type BiometricType = 'fingerprint' | 'face' | 'iris';

/**
 * Biometric binding information
 */
export interface BiometricBinding {
  type: BiometricType;
  hash: string;
  verified: boolean;
}

/**
 * Ticket holder information
 */
export interface TicketHolder {
  name: string;
  email?: string;
  phone?: string;
  verified: boolean;
  biometric?: BiometricBinding;
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Pricing Types
// ============================================================================

/**
 * Discount type
 */
export type DiscountType =
  | 'early-bird'
  | 'student'
  | 'senior'
  | 'group'
  | 'promo'
  | 'loyalty'
  | 'coupon';

/**
 * Discount information
 */
export interface Discount {
  type: DiscountType;
  amount: number;
  code?: string;
  description?: string;
}

/**
 * Pricing information
 */
export interface Pricing {
  originalPrice: number;
  finalPrice: number;
  currency: string;
  taxes: number;
  fees: number;
  discounts: Discount[];
}

// ============================================================================
// Validity Types
// ============================================================================

/**
 * Entry window time range
 */
export interface EntryWindow {
  start: Date | string;
  end: Date | string;
}

/**
 * Ticket validity period
 */
export interface Validity {
  issueDate: Date | string;
  eventDate: Date | string;
  expiryDate: Date | string;
  timezone: string;
  entryWindow?: EntryWindow;
}

// ============================================================================
// Security Types
// ============================================================================

/**
 * Blockchain network
 */
export type BlockchainNetwork = 'ethereum' | 'polygon' | 'solana' | 'avalanche';

/**
 * Blockchain verification data
 */
export interface BlockchainData {
  hash: string;
  network: BlockchainNetwork;
  contractAddress: string;
  tokenId: string;
  verified: boolean;
}

/**
 * NFC chip data
 */
export interface NFCData {
  uid: string;
  ndef: string;
  encrypted?: boolean;
}

/**
 * Time-based OTP configuration
 */
export interface TOTPConfig {
  secret: string;
  interval: number;
  algorithm?: 'SHA1' | 'SHA256' | 'SHA512';
}

/**
 * Security information
 */
export interface SecurityData {
  qrCode: string;
  barcode?: string;
  nfc?: NFCData;
  blockchain?: BlockchainData;
  signature: string;
  totp?: TOTPConfig;
}

// ============================================================================
// Ticket Status Types
// ============================================================================

/**
 * Ticket status
 */
export type TicketStatus =
  | 'active'
  | 'pending'
  | 'used'
  | 'expired'
  | 'cancelled'
  | 'suspended'
  | 'transferred';

/**
 * Check-in status
 */
export type CheckInStatus = 'not-checked-in' | 'checked-in' | 'rejected';

/**
 * Check-in result
 */
export type CheckInResult = 'success' | 'failure' | 'duplicate';

/**
 * Check-in history entry
 */
export interface CheckInHistory {
  timestamp: Date | string;
  location: string;
  result: CheckInResult;
  validator: string;
  coordinates?: Coordinates;
}

// ============================================================================
// Ticket Core Type
// ============================================================================

/**
 * Complete ticket structure
 */
export interface Ticket {
  ticketId: string;
  eventId: string;
  eventName: string;
  venue: Venue;
  seat?: Seat;
  holder: TicketHolder;
  pricing: Pricing;
  validity: Validity;
  security: SecurityData;
  status: TicketStatus;
  transferable: boolean;
  resellable: boolean;
  checkInStatus: CheckInStatus;
  checkInHistory?: CheckInHistory[];
  metadata: {
    issuer: string;
    platform: string;
    version: string;
    customFields?: Record<string, unknown>;
  };
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Event category
 */
export type EventCategory =
  | 'concert'
  | 'sports'
  | 'theater'
  | 'conference'
  | 'festival'
  | 'cinema'
  | 'museum'
  | 'virtual'
  | 'other';

/**
 * Event information
 */
export interface Event {
  id: string;
  name: string;
  category: EventCategory;
  description?: string;
  venueId: string;
  startDate: Date | string;
  endDate: Date | string;
  timezone: string;
  capacity: number;
  ticketsAvailable: number;
  ticketsSold: number;
  pricing: {
    basePrice: number;
    currency: string;
    dynamicPricing: boolean;
  };
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Dynamic Pricing Types
// ============================================================================

/**
 * Market conditions
 */
export type MarketCondition = 'low' | 'normal' | 'high' | 'very-high';

/**
 * Demand metrics
 */
export interface DemandMetrics {
  pageViews: number;
  searchVolume: number;
  socialMentions: number;
  historyRate: number;
}

/**
 * Market conditions data
 */
export interface MarketConditions {
  economicIndex: number;
  seasonality: number;
  competitorPrices: number[];
  weatherForecast?: 'excellent' | 'good' | 'fair' | 'poor';
}

/**
 * Dynamic pricing parameters
 */
export interface DynamicPricingParams {
  basePrice: number;
  demandFactor?: number;
  daysUntilEvent: number;
  capacityRemaining: number;
  marketConditions?: MarketCondition;
  competitorPrices?: number[];
}

/**
 * Dynamic pricing result
 */
export interface DynamicPricingResult {
  calculatedPrice: number;
  basePrice: number;
  demandFactor: number;
  timeDecayFactor: number;
  scarcityFactor: number;
  marketMultiplier: number;
  breakdown: {
    demandAdjustment: number;
    timeAdjustment: number;
    scarcityAdjustment: number;
    marketAdjustment: number;
  };
}

// ============================================================================
// Validation Types
// ============================================================================

/**
 * Validation method
 */
export type ValidationMethod = 'qr' | 'barcode' | 'nfc' | 'biometric';

/**
 * Validation request
 */
export interface ValidationRequest {
  ticketId: string;
  validationMethod: ValidationMethod;
  validationData: string;
  location: {
    venueId: string;
    gate: string;
    coordinates?: Coordinates;
  };
  timestamp: Date | string;
  deviceId: string;
  validatorId?: string;
}

/**
 * Validation action
 */
export type ValidationAction = 'allow' | 'deny' | 'manual-check';

/**
 * Access level
 */
export type AccessLevel = 'standard' | 'vip' | 'staff' | 'accessible';

/**
 * Validation result
 */
export interface ValidationResult {
  isValid: boolean;
  ticketId: string;
  holder: {
    name: string;
    verified: boolean;
  };
  seat?: Seat;
  warnings: string[];
  errors: string[];
  action: ValidationAction;
  accessLevel: AccessLevel;
  checkInTime?: Date;
}

// ============================================================================
// Transfer Types
// ============================================================================

/**
 * Transfer status
 */
export type TransferStatus = 'pending' | 'auto-approved' | 'approved' | 'rejected' | 'completed' | 'expired';

/**
 * Ticket transfer request
 */
export interface TicketTransfer {
  ticketId: string;
  fromEmail: string;
  toEmail: string;
  toName?: string;
  requireApproval: boolean;
  message?: string;
}

/**
 * Transfer request record
 */
export interface TransferRequest {
  id: string;
  ticketId: string;
  from: string;
  to: string;
  toName?: string;
  status: TransferStatus;
  createdAt: Date;
  completedAt?: Date;
  message?: string;
}

/**
 * Transfer result
 */
export interface TransferResult {
  success: boolean;
  transferId: string;
  status: TransferStatus;
  message?: string;
}

// ============================================================================
// Resale Marketplace Types
// ============================================================================

/**
 * Resale listing status
 */
export type ListingStatus = 'active' | 'pending' | 'sold' | 'expired' | 'cancelled';

/**
 * Resale rules
 */
export interface ResaleRules {
  enabled: boolean;
  maxMarkup: number;
  minPriceRatio: number;
  transferFee: number;
  maxListingsPerUser: number;
  verificationRequired: boolean;
  cooldownPeriod: number;
}

/**
 * Resale listing
 */
export interface ResaleListing {
  ticketId: string;
  askingPrice: number;
  sellerId: string;
  expiryDate: Date | string;
}

/**
 * Marketplace listing record
 */
export interface MarketplaceListing {
  id: string;
  ticketId: string;
  sellerId: string;
  sellerEmail: string;
  askingPrice: number;
  originalPrice: number;
  listedAt: Date;
  expiryDate: Date;
  status: ListingStatus;
  transferFee: number;
  soldAt?: Date;
  buyerId?: string;
  finalPrice?: number;
}

/**
 * Listing result
 */
export interface ListingResult {
  success: boolean;
  listingId: string;
  url: string;
}

/**
 * Purchase result
 */
export interface PurchaseResult {
  success: boolean;
  ticketId: string;
  transferId: string;
  amountPaid: number;
}

// ============================================================================
// Season Pass Types
// ============================================================================

/**
 * Season pass type
 */
export type SeasonPassType = 'full-season' | 'partial-season' | 'flex-pass' | 'unlimited';

/**
 * Payment plan
 */
export interface PaymentPlan {
  installments: number;
  frequency: 'monthly' | 'quarterly';
  amount: number;
}

/**
 * Season pass benefits
 */
export interface SeasonPassBenefits {
  priorityAccess: boolean;
  discounts: Discount[];
  exclusiveContent: boolean;
  transferable: boolean;
}

/**
 * Season pass allocation
 */
export interface SeasonPassAllocation {
  totalEvents: number;
  usedEvents: number;
  remainingEvents: number;
  includedEvents: string[];
}

/**
 * Season pass
 */
export interface SeasonPass {
  passId: string;
  passType: SeasonPassType;
  holder: TicketHolder;
  validity: {
    startDate: Date | string;
    endDate: Date | string;
    timezone: string;
  };
  allocation: SeasonPassAllocation;
  benefits: SeasonPassBenefits;
  pricing: {
    totalPrice: number;
    perEventPrice: number;
    currency: string;
    paymentPlan?: PaymentPlan;
  };
  status: 'active' | 'suspended' | 'expired' | 'cancelled';
  renewalDate?: Date | string;
  autoRenew: boolean;
}

// ============================================================================
// QR Code Types
// ============================================================================

/**
 * QR code format
 */
export type QRCodeFormat = 'png' | 'svg' | 'base64' | 'terminal';

/**
 * QR code error correction level
 */
export type QRErrorCorrectionLevel = 'L' | 'M' | 'Q' | 'H';

/**
 * QR code generation parameters
 */
export interface QRCodeParams {
  ticketId: string;
  eventId: string;
  holderEmail: string;
  expiryDate: Date | string;
  totpSecret?: string;
  format: QRCodeFormat;
  size?: number;
  errorCorrection?: QRErrorCorrectionLevel;
}

// ============================================================================
// Access Control Types
// ============================================================================

/**
 * Access control gate type
 */
export type GateType = 'turnstile' | 'scanner' | 'nfc' | 'biometric' | 'mobile';

/**
 * Gate configuration
 */
export interface GateConfig {
  id: string;
  type: GateType;
  location: {
    name: string;
    coordinates: Coordinates;
  };
  capacity: number;
  status: 'online' | 'offline' | 'maintenance';
}

/**
 * Validation rules
 */
export interface ValidationRules {
  requireQR: boolean;
  requireBiometric: boolean;
  geofenceRadius: number;
  allowOffline: boolean;
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Analytics metrics
 */
export type AnalyticsMetric = 'sales' | 'revenue' | 'attendance' | 'demographics' | 'conversion';

/**
 * Demographics data
 */
export interface Demographics {
  age: Record<string, number>;
  gender: Record<string, number>;
  location: Record<string, number>;
}

/**
 * Analytics result
 */
export interface AnalyticsResult {
  eventId: string;
  totalTicketsSold: number;
  revenue: number;
  currentAttendance: number;
  peakHour?: string;
  demographics?: Demographics;
  conversionRate?: number;
}

// ============================================================================
// Bot Detection Types
// ============================================================================

/**
 * Bot detection signals
 */
export interface BotSignals {
  userAgent: number;
  ipReputation: number;
  browserFingerprint: string;
  mouseMovements: number;
  keystrokeDynamics: number;
  pageNavigation: string;
  requestSpeed: number;
  formFillTime: number;
}

/**
 * Bot detection action
 */
export type BotAction = 'allow' | 'challenge' | 'block';

/**
 * Bot detection score
 */
export interface BotScore {
  probability: number;
  signals: BotSignals;
  action: BotAction;
}

// ============================================================================
// Capacity Management Types
// ============================================================================

/**
 * Capacity status
 */
export interface CapacityStatus {
  total: number;
  sold: number;
  reserved: number;
  available: number;
  blocked: number;
  utilization: number;
}

/**
 * Seat lock
 */
export interface SeatLock {
  id: string;
  seatId: string;
  userId: string;
  sessionId: string;
  duration: number;
  expiresAt: Date;
}

// ============================================================================
// Multi-Venue Types
// ============================================================================

/**
 * Tour event date
 */
export interface TourDate {
  venueId: string;
  date: Date | string;
  capacity?: number;
}

/**
 * Tour ticket type
 */
export interface TourTicketType {
  type: string;
  venues: 'any' | 'all' | string[];
  price: number;
}

/**
 * Tour event
 */
export interface TourEvent {
  tourId: string;
  name: string;
  artist?: string;
  dates: TourDate[];
  ticketTypes: TourTicketType[];
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-IND-019 error codes
 */
export enum TicketingErrorCode {
  INVALID_TICKET = 'IND019-001',
  TICKET_EXPIRED = 'IND019-002',
  TICKET_USED = 'IND019-003',
  DUPLICATE_SCAN = 'IND019-004',
  INVALID_QR = 'IND019-005',
  LOCATION_MISMATCH = 'IND019-006',
  TOTP_EXPIRED = 'IND019-007',
  BLOCKCHAIN_INVALID = 'IND019-008',
  NOT_TRANSFERABLE = 'IND019-009',
  NOT_RESELLABLE = 'IND019-010',
  PRICE_VIOLATION = 'IND019-011',
  CAPACITY_EXCEEDED = 'IND019-012',
  BOT_DETECTED = 'IND019-013',
  FRAUD_SUSPECTED = 'IND019-014',
  SEAT_UNAVAILABLE = 'IND019-015',
}

/**
 * Ticketing error
 */
export class TicketingError extends Error {
  constructor(
    public code: TicketingErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TicketingError';
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
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated result
 */
export interface PaginatedResult<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  totalPages: number;
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core
  Coordinates,
  Location,

  // Venue
  Venue,
  VenueType,
  VenueSection,
  VenueAccessibility,

  // Seat
  Seat,
  SeatType,

  // Holder
  TicketHolder,
  BiometricType,
  BiometricBinding,

  // Pricing
  Pricing,
  Discount,
  DiscountType,

  // Validity
  Validity,
  EntryWindow,

  // Security
  SecurityData,
  BlockchainData,
  BlockchainNetwork,
  NFCData,
  TOTPConfig,

  // Ticket
  Ticket,
  TicketStatus,
  CheckInStatus,
  CheckInResult,
  CheckInHistory,

  // Event
  Event,
  EventCategory,

  // Dynamic Pricing
  DynamicPricingParams,
  DynamicPricingResult,
  DemandMetrics,
  MarketConditions,
  MarketCondition,

  // Validation
  ValidationRequest,
  ValidationResult,
  ValidationMethod,
  ValidationAction,
  AccessLevel,

  // Transfer
  TicketTransfer,
  TransferRequest,
  TransferResult,
  TransferStatus,

  // Resale
  ResaleListing,
  MarketplaceListing,
  ListingResult,
  PurchaseResult,
  ResaleRules,
  ListingStatus,

  // Season Pass
  SeasonPass,
  SeasonPassType,
  SeasonPassAllocation,
  SeasonPassBenefits,
  PaymentPlan,

  // QR Code
  QRCodeParams,
  QRCodeFormat,
  QRErrorCorrectionLevel,

  // Access Control
  GateConfig,
  GateType,
  ValidationRules,

  // Analytics
  AnalyticsResult,
  AnalyticsMetric,
  Demographics,

  // Bot Detection
  BotScore,
  BotSignals,
  BotAction,

  // Capacity
  CapacityStatus,
  SeatLock,

  // Multi-Venue
  TourEvent,
  TourDate,
  TourTicketType,

  // Utility
  PaginationParams,
  PaginatedResult,
};

export { TicketingErrorCode, TicketingError };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
