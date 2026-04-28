/**
 * WIA-IND-015: Travel Tech - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * ISO 8601 date-time string
 */
export type DateTime = string;

/**
 * ISO 8601 date string (YYYY-MM-DD)
 */
export type DateString = string;

/**
 * ISO 4217 currency code
 */
export type CurrencyCode = string;

/**
 * IATA airport code (3 letters)
 */
export type AirportCode = string;

/**
 * IATA airline code (2 letters)
 */
export type AirlineCode = string;

/**
 * ISO 3166-1 alpha-2 country code
 */
export type CountryCode = string;

/**
 * Language code (ISO 639-1)
 */
export type LanguageCode = string;

/**
 * Geographic coordinates
 */
export interface GeoCoordinate {
  latitude: number;
  longitude: number;
  altitude?: number;
}

/**
 * Money amount with currency
 */
export interface MoneyAmount {
  amount: number;
  currency: CurrencyCode;
}

/**
 * Price breakdown
 */
export interface PriceBreakdown {
  basePrice: MoneyAmount;
  taxes: MoneyAmount[];
  fees: MoneyAmount[];
  total: MoneyAmount;
  discount?: MoneyAmount;
}

// ============================================================================
// Travel Classes and Service Levels
// ============================================================================

/**
 * Travel service class
 */
export type TravelClass = 'economy' | 'premium-economy' | 'business' | 'first' | 'private';

/**
 * Hotel star rating
 */
export type HotelStars = 1 | 2 | 3 | 4 | 5;

/**
 * Service tier
 */
export type ServiceTier = 'basic' | 'standard' | 'premium' | 'luxury';

// ============================================================================
// Passenger and Traveler Information
// ============================================================================

/**
 * Passenger type
 */
export type PassengerType = 'adult' | 'child' | 'infant' | 'senior' | 'student';

/**
 * Gender
 */
export type Gender = 'male' | 'female' | 'other' | 'prefer-not-to-say';

/**
 * Traveler information
 */
export interface Traveler {
  /** Unique traveler ID */
  id?: string;

  /** Title (Mr., Mrs., Ms., Dr., etc.) */
  title?: string;

  /** First/given name */
  firstName: string;

  /** Middle name */
  middleName?: string;

  /** Last/family name */
  lastName: string;

  /** Date of birth */
  dateOfBirth?: DateString;

  /** Gender */
  gender?: Gender;

  /** Passenger type */
  type: PassengerType;

  /** Contact information */
  contact: {
    email: string;
    phone: string;
    address?: Address;
  };

  /** Frequent flyer numbers */
  frequentFlyer?: FrequentFlyerInfo[];

  /** Special requirements */
  specialRequirements?: SpecialRequirements;

  /** Travel documents */
  documents?: TravelDocument[];
}

/**
 * Address information
 */
export interface Address {
  street1: string;
  street2?: string;
  city: string;
  state?: string;
  postalCode: string;
  country: CountryCode;
}

/**
 * Frequent flyer information
 */
export interface FrequentFlyerInfo {
  airline: AirlineCode;
  number: string;
  tier?: string;
}

/**
 * Special requirements and accessibility needs
 */
export interface SpecialRequirements {
  /** Wheelchair assistance */
  wheelchair?: boolean;

  /** Visual impairment assistance */
  visualImpairment?: boolean;

  /** Hearing impairment assistance */
  hearingImpairment?: boolean;

  /** Special meal requests */
  mealPreference?: MealType[];

  /** Medical conditions */
  medicalConditions?: string[];

  /** Traveling with service animal */
  serviceAnimal?: boolean;

  /** Additional assistance */
  additionalAssistance?: string;
}

/**
 * Meal type preferences
 */
export type MealType =
  | 'regular'
  | 'vegetarian'
  | 'vegan'
  | 'halal'
  | 'kosher'
  | 'gluten-free'
  | 'diabetic'
  | 'low-sodium'
  | 'low-fat'
  | 'child'
  | 'baby';

// ============================================================================
// Flight Types
// ============================================================================

/**
 * Flight search request
 */
export interface FlightSearchRequest {
  /** Origin airport code */
  origin: AirportCode;

  /** Destination airport code */
  destination: AirportCode;

  /** Departure date */
  departureDate: DateString;

  /** Return date (for round-trip) */
  returnDate?: DateString;

  /** Number of passengers by type */
  passengers: {
    adults: number;
    children?: number;
    infants?: number;
  };

  /** Travel class */
  class: TravelClass;

  /** Search preferences */
  preferences?: FlightSearchPreferences;
}

/**
 * Flight search preferences
 */
export interface FlightSearchPreferences {
  /** Nonstop flights only */
  nonstop?: boolean;

  /** Maximum number of stops */
  maxStops?: number;

  /** Preferred airlines */
  airlines?: AirlineCode[];

  /** Excluded airlines */
  excludeAirlines?: AirlineCode[];

  /** Preferred departure time range */
  departureTimeRange?: TimeRange;

  /** Preferred arrival time range */
  arrivalTimeRange?: TimeRange;

  /** Maximum price */
  maxPrice?: MoneyAmount;

  /** Flexible dates (+/- days) */
  flexibleDates?: number;

  /** Cabin baggage only */
  cabinBaggageOnly?: boolean;
}

/**
 * Time range
 */
export interface TimeRange {
  start: string; // HH:MM format
  end: string; // HH:MM format
}

/**
 * Flight search result
 */
export interface FlightSearchResult {
  /** Search ID for tracking */
  searchId: string;

  /** Available flight options */
  flights: FlightOption[];

  /** Search metadata */
  metadata: {
    searchDate: DateTime;
    validUntil: DateTime;
    currency: CurrencyCode;
    resultsCount: number;
  };
}

/**
 * Flight option
 */
export interface FlightOption {
  /** Unique option ID */
  id: string;

  /** Outbound journey */
  outbound: FlightJourney;

  /** Return journey (if round-trip) */
  return?: FlightJourney;

  /** Total price */
  price: PriceBreakdown;

  /** Availability */
  seatsAvailable: number;

  /** Booking class */
  bookingClass: string;

  /** Fare rules */
  fareRules: FareRules;

  /** Amenities */
  amenities?: FlightAmenities;
}

/**
 * Flight journey (one or more segments)
 */
export interface FlightJourney {
  /** Journey segments */
  segments: FlightSegment[];

  /** Total duration in minutes */
  totalDuration: number;

  /** Number of stops */
  stops: number;
}

/**
 * Flight segment
 */
export interface FlightSegment {
  /** Segment number */
  segmentNumber: number;

  /** Operating airline */
  airline: AirlineCode;

  /** Flight number */
  flightNumber: string;

  /** Aircraft type */
  aircraft: string;

  /** Departure */
  departure: {
    airport: AirportCode;
    terminal?: string;
    gate?: string;
    time: DateTime;
  };

  /** Arrival */
  arrival: {
    airport: AirportCode;
    terminal?: string;
    gate?: string;
    time: DateTime;
  };

  /** Duration in minutes */
  duration: number;

  /** Layover time to next segment (minutes) */
  layover?: number;

  /** Travel class */
  class: TravelClass;

  /** Baggage allowance */
  baggage: BaggageAllowance;
}

/**
 * Baggage allowance
 */
export interface BaggageAllowance {
  /** Cabin baggage */
  cabin: {
    pieces: number;
    weight?: number; // kg
    dimensions?: string; // e.g., "55x40x20cm"
  };

  /** Checked baggage */
  checked: {
    pieces: number;
    weight?: number; // kg
    dimensions?: string;
  };
}

/**
 * Fare rules
 */
export interface FareRules {
  /** Refundable */
  refundable: boolean;

  /** Changeable */
  changeable: boolean;

  /** Cancellation fee */
  cancellationFee?: MoneyAmount;

  /** Change fee */
  changeFee?: MoneyAmount;

  /** Advance purchase required (days) */
  advancePurchase?: number;

  /** Minimum stay (days) */
  minStay?: number;

  /** Maximum stay (days) */
  maxStay?: number;
}

/**
 * Flight amenities
 */
export interface FlightAmenities {
  wifi?: boolean;
  power?: boolean;
  entertainment?: boolean;
  meal?: boolean;
  beverage?: boolean;
  lounge?: boolean;
  priorityBoarding?: boolean;
  extraLegroom?: boolean;
}

// ============================================================================
// Hotel Types
// ============================================================================

/**
 * Hotel search request
 */
export interface HotelSearchRequest {
  /** Destination (city, address, or coordinates) */
  destination: string | GeoCoordinate;

  /** Check-in date */
  checkIn: DateString;

  /** Check-out date */
  checkOut: DateString;

  /** Number of rooms */
  rooms: number;

  /** Number of guests */
  guests: number;

  /** Search preferences */
  preferences?: HotelSearchPreferences;
}

/**
 * Hotel search preferences
 */
export interface HotelSearchPreferences {
  /** Minimum star rating */
  minStars?: HotelStars;

  /** Maximum star rating */
  maxStars?: HotelStars;

  /** Price range */
  priceRange?: {
    min: MoneyAmount;
    max: MoneyAmount;
  };

  /** Required amenities */
  amenities?: string[];

  /** Hotel chains */
  chains?: string[];

  /** Accessibility requirements */
  accessibility?: SpecialRequirements;

  /** Guest rating minimum (0-10) */
  minGuestRating?: number;

  /** Free cancellation only */
  freeCancellation?: boolean;

  /** Breakfast included */
  breakfastIncluded?: boolean;
}

/**
 * Hotel search result
 */
export interface HotelSearchResult {
  /** Search ID */
  searchId: string;

  /** Available hotels */
  hotels: HotelOption[];

  /** Metadata */
  metadata: {
    searchDate: DateTime;
    validUntil: DateTime;
    currency: CurrencyCode;
    resultsCount: number;
  };
}

/**
 * Hotel option
 */
export interface HotelOption {
  /** Hotel ID */
  id: string;

  /** Hotel name */
  name: string;

  /** Star rating */
  stars: HotelStars;

  /** Address */
  address: Address;

  /** Location */
  location: GeoCoordinate;

  /** Room types available */
  rooms: RoomOption[];

  /** Amenities */
  amenities: string[];

  /** Guest rating */
  guestRating?: {
    score: number; // 0-10
    reviews: number;
  };

  /** Photos */
  photos?: string[];

  /** Description */
  description?: string;

  /** Check-in/out times */
  checkInTime: string;
  checkOutTime: string;
}

/**
 * Room option
 */
export interface RoomOption {
  /** Room type ID */
  id: string;

  /** Room type name */
  name: string;

  /** Description */
  description?: string;

  /** Maximum occupancy */
  maxOccupancy: number;

  /** Bed configuration */
  beds: BedConfiguration[];

  /** Room size (sq meters) */
  size?: number;

  /** Price per night */
  pricePerNight: MoneyAmount;

  /** Total price */
  totalPrice: PriceBreakdown;

  /** Availability */
  available: number;

  /** Cancellation policy */
  cancellationPolicy: CancellationPolicy;

  /** Room amenities */
  amenities: string[];
}

/**
 * Bed configuration
 */
export interface BedConfiguration {
  type: 'single' | 'double' | 'queen' | 'king' | 'sofa-bed';
  count: number;
}

/**
 * Cancellation policy
 */
export interface CancellationPolicy {
  /** Free cancellation until */
  freeCancellationUntil?: DateTime;

  /** Non-refundable */
  nonRefundable: boolean;

  /** Cancellation fee schedule */
  fees?: {
    from: DateTime;
    to?: DateTime;
    fee: MoneyAmount | { percentage: number };
  }[];
}

// ============================================================================
// Transportation Types
// ============================================================================

/**
 * Transportation mode
 */
export type TransportMode = 'flight' | 'train' | 'bus' | 'car' | 'taxi' | 'rideshare' | 'bike' | 'walk' | 'ferry';

/**
 * Transportation search request
 */
export interface TransportSearchRequest {
  /** Origin location */
  origin: string | GeoCoordinate;

  /** Destination location */
  destination: string | GeoCoordinate;

  /** Departure date/time */
  departureTime: DateTime;

  /** Transportation modes to include */
  modes?: TransportMode[];

  /** Number of passengers */
  passengers?: number;

  /** Preferences */
  preferences?: {
    maxTransfers?: number;
    wheelchair?: boolean;
    bikeAllowed?: boolean;
    maxPrice?: MoneyAmount;
  };
}

/**
 * Transportation option
 */
export interface TransportOption {
  /** Option ID */
  id: string;

  /** Transport mode */
  mode: TransportMode;

  /** Journey legs */
  legs: TransportLeg[];

  /** Total duration (minutes) */
  duration: number;

  /** Total distance (km) */
  distance: number;

  /** Price */
  price: PriceBreakdown;

  /** Departure time */
  departureTime: DateTime;

  /** Arrival time */
  arrivalTime: DateTime;

  /** Carbon footprint (kg CO2) */
  carbonFootprint?: number;
}

/**
 * Transport leg
 */
export interface TransportLeg {
  /** Leg number */
  legNumber: number;

  /** Transport mode */
  mode: TransportMode;

  /** Provider/operator */
  provider: string;

  /** Route/line number */
  route?: string;

  /** Origin */
  origin: {
    location: string | GeoCoordinate;
    time: DateTime;
    platform?: string;
  };

  /** Destination */
  destination: {
    location: string | GeoCoordinate;
    time: DateTime;
    platform?: string;
  };

  /** Duration (minutes) */
  duration: number;

  /** Distance (km) */
  distance: number;

  /** Intermediate stops */
  stops?: string[];
}

// ============================================================================
// Travel Documents
// ============================================================================

/**
 * Travel document type
 */
export type DocumentType = 'passport' | 'visa' | 'id-card' | 'vaccine-certificate' | 'travel-insurance' | 'boarding-pass' | 'hotel-voucher';

/**
 * Travel document
 */
export interface TravelDocument {
  /** Document type */
  type: DocumentType;

  /** Document number */
  number: string;

  /** Issuing country */
  issuingCountry: CountryCode;

  /** Issue date */
  issueDate: DateString;

  /** Expiry date */
  expiryDate: DateString;

  /** Additional data */
  metadata?: Record<string, any>;
}

/**
 * Document verification request
 */
export interface DocumentVerificationRequest {
  /** Traveler's passport */
  passport: TravelDocument;

  /** Destination country */
  destination: CountryCode;

  /** Departure date */
  departureDate: DateString;

  /** Return date */
  returnDate?: DateString;

  /** Purpose of travel */
  purpose?: 'tourism' | 'business' | 'study' | 'work' | 'transit' | 'other';
}

/**
 * Document verification result
 */
export interface DocumentVerificationResult {
  /** Is valid for travel */
  valid: boolean;

  /** Requirements */
  requirements: TravelRequirement[];

  /** Warnings */
  warnings: string[];

  /** Errors */
  errors: string[];
}

/**
 * Travel requirement
 */
export interface TravelRequirement {
  /** Requirement type */
  type: 'visa' | 'vaccine' | 'covid-test' | 'insurance' | 'transit-visa' | 'other';

  /** Is required */
  required: boolean;

  /** Is met */
  met: boolean;

  /** Description */
  description: string;

  /** How to obtain */
  howToObtain?: string;

  /** Processing time */
  processingTime?: string;

  /** Cost */
  cost?: MoneyAmount;

  /** Validity period */
  validity?: string;
}

// ============================================================================
// Itinerary Types
// ============================================================================

/**
 * Travel itinerary
 */
export interface TravelItinerary {
  /** Itinerary ID */
  id: string;

  /** Title */
  title: string;

  /** Description */
  description?: string;

  /** Travelers */
  travelers: Traveler[];

  /** Itinerary items */
  items: ItineraryItem[];

  /** Start date */
  startDate: DateString;

  /** End date */
  endDate: DateString;

  /** Total budget */
  budget?: MoneyAmount;

  /** Total cost */
  totalCost?: PriceBreakdown;

  /** Status */
  status: 'draft' | 'confirmed' | 'completed' | 'cancelled';

  /** Created at */
  createdAt: DateTime;

  /** Updated at */
  updatedAt: DateTime;
}

/**
 * Itinerary item
 */
export interface ItineraryItem {
  /** Item ID */
  id: string;

  /** Item type */
  type: 'flight' | 'hotel' | 'transport' | 'activity' | 'restaurant' | 'other';

  /** Date/time */
  dateTime: DateTime;

  /** End date/time (for multi-day items) */
  endDateTime?: DateTime;

  /** Title */
  title: string;

  /** Description */
  description?: string;

  /** Location */
  location?: string | GeoCoordinate;

  /** Booking reference */
  bookingReference?: string;

  /** Confirmation number */
  confirmationNumber?: string;

  /** Price */
  price?: MoneyAmount;

  /** Status */
  status: 'pending' | 'confirmed' | 'completed' | 'cancelled';

  /** Additional data */
  details?: Record<string, any>;
}

// ============================================================================
// Currency and Payment
// ============================================================================

/**
 * Currency conversion request
 */
export interface CurrencyConversionRequest {
  /** Source currency */
  from: CurrencyCode;

  /** Target currency */
  to: CurrencyCode;

  /** Amount to convert */
  amount: number;

  /** Use historical rate for date */
  date?: DateString;
}

/**
 * Currency conversion result
 */
export interface CurrencyConversionResult {
  /** Original amount */
  from: MoneyAmount;

  /** Converted amount */
  to: MoneyAmount;

  /** Exchange rate */
  rate: number;

  /** Rate timestamp */
  rateTimestamp: DateTime;

  /** Conversion fee */
  fee?: MoneyAmount;
}

/**
 * Payment method
 */
export interface PaymentMethod {
  /** Payment method type */
  type: 'credit-card' | 'debit-card' | 'bank-transfer' | 'digital-wallet' | 'crypto' | 'points' | 'voucher';

  /** Card details (if applicable) */
  card?: {
    number: string; // Masked
    holder: string;
    expiry: string; // MM/YY
    cvv?: string;
    type: 'visa' | 'mastercard' | 'amex' | 'discover' | 'other';
  };

  /** Wallet details (if applicable) */
  wallet?: {
    provider: 'paypal' | 'apple-pay' | 'google-pay' | 'other';
    accountId: string;
  };

  /** Bank details (if applicable) */
  bank?: {
    accountNumber: string;
    routingNumber: string;
    swift?: string;
  };
}

// ============================================================================
// Insurance Types
// ============================================================================

/**
 * Travel insurance type
 */
export type InsuranceType = 'trip-cancellation' | 'medical' | 'baggage' | 'flight-delay' | 'comprehensive';

/**
 * Insurance policy
 */
export interface InsurancePolicy {
  /** Policy ID */
  id: string;

  /** Insurance type */
  type: InsuranceType;

  /** Provider */
  provider: string;

  /** Coverage amount */
  coverage: MoneyAmount;

  /** Premium (cost) */
  premium: MoneyAmount;

  /** Covered travelers */
  travelers: string[]; // Traveler IDs

  /** Coverage period */
  period: {
    start: DateString;
    end: DateString;
  };

  /** Covered destinations */
  destinations: CountryCode[];

  /** Policy details */
  details: {
    medicalExpenses?: MoneyAmount;
    emergencyEvacuation?: MoneyAmount;
    tripCancellation?: MoneyAmount;
    baggageLoss?: MoneyAmount;
    flightDelay?: {
      minHours: number;
      compensation: MoneyAmount;
    };
  };

  /** Exclusions */
  exclusions?: string[];

  /** Policy document URL */
  policyDocument?: string;
}

// ============================================================================
// Loyalty Programs
// ============================================================================

/**
 * Loyalty account
 */
export interface LoyaltyAccount {
  /** Program ID */
  programId: string;

  /** Program name */
  programName: string;

  /** Provider (airline, hotel chain, etc.) */
  provider: string;

  /** Member number */
  memberNumber: string;

  /** Member tier/status */
  tier?: string;

  /** Points/miles balance */
  balance: number;

  /** Points expiring soon */
  expiring?: {
    amount: number;
    date: DateString;
  };

  /** Lifetime points */
  lifetimePoints?: number;
}

/**
 * Loyalty transaction
 */
export interface LoyaltyTransaction {
  /** Transaction ID */
  id: string;

  /** Account */
  account: string; // Loyalty account ID

  /** Transaction type */
  type: 'earn' | 'redeem' | 'expire' | 'transfer';

  /** Points amount (positive for earn, negative for redeem) */
  points: number;

  /** Description */
  description: string;

  /** Related booking */
  bookingReference?: string;

  /** Transaction date */
  date: DateTime;
}

// ============================================================================
// Alerts and Notifications
// ============================================================================

/**
 * Travel alert type
 */
export type AlertType = 'flight-delay' | 'gate-change' | 'cancellation' | 'weather' | 'security' | 'health' | 'destination-advisory';

/**
 * Travel alert
 */
export interface TravelAlert {
  /** Alert ID */
  id: string;

  /** Alert type */
  type: AlertType;

  /** Severity */
  severity: 'low' | 'medium' | 'high' | 'critical';

  /** Title */
  title: string;

  /** Message */
  message: string;

  /** Affected locations */
  locations?: string[];

  /** Affected dates */
  dates?: {
    start: DateString;
    end?: DateString;
  };

  /** Related booking */
  bookingReference?: string;

  /** Action required */
  actionRequired?: boolean;

  /** Action URL */
  actionUrl?: string;

  /** Timestamp */
  timestamp: DateTime;

  /** Expiry */
  expiresAt?: DateTime;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration
 */
export interface TravelTechConfig {
  /** API key */
  apiKey: string;

  /** Environment */
  environment: 'production' | 'sandbox' | 'development';

  /** API base URL (optional, for custom endpoints) */
  baseUrl?: string;

  /** Default currency */
  defaultCurrency?: CurrencyCode;

  /** Default language */
  defaultLanguage?: LanguageCode;

  /** Timeout (ms) */
  timeout?: number;

  /** Retry configuration */
  retry?: {
    maxRetries: number;
    retryDelay: number;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * API success response
 */
export interface ApiResponse<T> {
  success: true;
  data: T;
  metadata?: Record<string, any>;
}

/**
 * API error response
 */
export interface ApiError {
  success: false;
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
  };
}

/**
 * API response (success or error)
 */
export type ApiResult<T> = ApiResponse<T> | ApiError;

/**
 * Pagination metadata
 */
export interface PaginationMeta {
  page: number;
  pageSize: number;
  total: number;
  hasMore: boolean;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  pagination: PaginationMeta;
}

**弘益人間 (홍익인간) · Benefit All Humanity**
