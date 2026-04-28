/**
 * WIA-IND-017: Tourism Data - TypeScript Type Definitions
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
 * Geographic coordinates (WGS84)
 */
export interface GeoCoordinates {
  /** Latitude in decimal degrees (-90 to 90) */
  lat: number;

  /** Longitude in decimal degrees (-180 to 180) */
  lng: number;

  /** Altitude in meters (optional) */
  altitude?: number;
}

/**
 * Bounding box for geographic area
 */
export interface GeoBounds {
  /** Northern boundary */
  north: number;

  /** Southern boundary */
  south: number;

  /** Eastern boundary */
  east: number;

  /** Western boundary */
  west: number;
}

/**
 * Address information
 */
export interface Address {
  /** Street address */
  street?: string;

  /** City name */
  city: string;

  /** State/Province/Region */
  region?: string;

  /** Postal/ZIP code */
  postalCode?: string;

  /** Country code (ISO 3166-1 alpha-2) */
  country: string;

  /** Full formatted address */
  formatted?: string;
}

// ============================================================================
// Multi-language Content
// ============================================================================

/**
 * Multi-language text content
 */
export type MultilingualText = Record<string, string>;

/**
 * Language code (ISO 639-1)
 */
export type LanguageCode =
  | 'en' | 'es' | 'fr' | 'de' | 'it' | 'pt' | 'ru' | 'ja' | 'ko' | 'zh'
  | 'ar' | 'hi' | 'nl' | 'pl' | 'tr' | 'sv' | 'no' | 'da' | 'fi' | 'th'
  | string;

// ============================================================================
// Tourism Categories
// ============================================================================

/**
 * Attraction category
 */
export type AttractionCategory =
  | 'natural'
  | 'cultural'
  | 'historical'
  | 'entertainment'
  | 'religious'
  | 'educational'
  | 'sports'
  | 'shopping'
  | 'nightlife'
  | 'wellness';

/**
 * Attraction type
 */
export type AttractionType =
  // Natural
  | 'beach' | 'mountain' | 'park' | 'garden' | 'lake' | 'waterfall' | 'forest' | 'canyon'
  // Cultural
  | 'museum' | 'gallery' | 'theater' | 'opera' | 'concert-hall' | 'library'
  // Historical
  | 'monument' | 'castle' | 'palace' | 'ruins' | 'heritage-site' | 'memorial'
  // Entertainment
  | 'theme-park' | 'zoo' | 'aquarium' | 'stadium' | 'cinema' | 'arcade'
  // Religious
  | 'church' | 'cathedral' | 'temple' | 'mosque' | 'synagogue' | 'shrine'
  // Other
  | 'landmark' | 'viewpoint' | 'square' | 'market' | 'neighborhood';

/**
 * POI (Point of Interest) type
 */
export type POIType =
  | 'restaurant' | 'cafe' | 'bar' | 'hotel' | 'hostel' | 'airport' | 'station'
  | 'hospital' | 'pharmacy' | 'atm' | 'bank' | 'parking' | 'gas-station'
  | 'tourist-info' | 'embassy' | 'police' | 'post-office';

// ============================================================================
// Attraction
// ============================================================================

/**
 * Tourist attraction
 */
export interface Attraction {
  /** Unique identifier */
  id: string;

  /** Attraction name */
  name: string;

  /** Multilingual names */
  names?: MultilingualText;

  /** Category */
  category: AttractionCategory;

  /** Specific type */
  type: AttractionType;

  /** Description */
  description: MultilingualText;

  /** Location */
  location: {
    coordinates: GeoCoordinates;
    address: Address;
  };

  /** Ratings and reviews */
  ratings?: AttractionRatings;

  /** Visit information */
  visitInfo?: VisitInfo;

  /** Accessibility information */
  accessibility?: AccessibilityInfo;

  /** Media (photos, videos) */
  media?: MediaGallery;

  /** Contact information */
  contact?: ContactInfo;

  /** Official website */
  website?: string;

  /** Social media links */
  socialMedia?: SocialMediaLinks;

  /** Tags and keywords */
  tags?: string[];

  /** UNESCO World Heritage status */
  unesco?: UNESCOInfo;

  /** Current crowd data */
  crowdData?: CrowdDensity;

  /** Last updated timestamp */
  updatedAt: Date;

  /** Data quality tier */
  quality: 'gold' | 'silver' | 'bronze' | 'basic';
}

/**
 * Attraction ratings and reviews
 */
export interface AttractionRatings {
  /** Overall rating (0-5) */
  overall: number;

  /** Total number of ratings */
  count: number;

  /** Rating breakdown */
  breakdown?: {
    5: number;
    4: number;
    3: number;
    2: number;
    1: number;
  };

  /** Category-specific ratings */
  categories?: {
    value?: number;
    atmosphere?: number;
    cleanliness?: number;
    service?: number;
    facilities?: number;
  };

  /** Recent reviews */
  recentReviews?: Review[];
}

/**
 * Review
 */
export interface Review {
  /** Review ID */
  id: string;

  /** Rating (0-5) */
  rating: number;

  /** Review title */
  title?: string;

  /** Review text */
  text: string;

  /** Language */
  language: LanguageCode;

  /** Reviewer name */
  author: string;

  /** Review date */
  date: Date;

  /** Helpful votes */
  helpful?: number;

  /** Photos */
  photos?: string[];
}

/**
 * Visit information
 */
export interface VisitInfo {
  /** Opening hours */
  openingHours?: OpeningHours;

  /** Ticket information */
  tickets?: TicketInfo;

  /** Average visit duration (minutes) */
  averageVisitDuration?: number;

  /** Best time to visit */
  bestTimeToVisit?: {
    seasons?: ('spring' | 'summer' | 'fall' | 'winter')[];
    months?: number[];
    daysOfWeek?: number[];
    timeOfDay?: ('morning' | 'afternoon' | 'evening' | 'night')[];
  };

  /** Guided tours available */
  guidedTours?: boolean;

  /** Audio guide languages */
  audioGuideLanguages?: LanguageCode[];

  /** Advance booking required */
  advanceBookingRequired?: boolean;

  /** Estimated wait time (minutes) */
  estimatedWaitTime?: number;
}

/**
 * Opening hours
 */
export interface OpeningHours {
  /** Is currently open */
  isOpen?: boolean;

  /** Regular hours */
  regular?: {
    monday?: string;
    tuesday?: string;
    wednesday?: string;
    thursday?: string;
    friday?: string;
    saturday?: string;
    sunday?: string;
  };

  /** Special hours (holidays, events) */
  special?: {
    date: Date;
    hours: string;
    reason?: string;
  }[];

  /** Seasonal hours */
  seasonal?: {
    season: string;
    startDate: Date;
    endDate: Date;
    hours: string;
  }[];
}

/**
 * Ticket information
 */
export interface TicketInfo {
  /** Free admission */
  free?: boolean;

  /** Pricing */
  pricing?: {
    adult?: Price;
    child?: Price;
    student?: Price;
    senior?: Price;
    family?: Price;
    group?: Price;
  };

  /** Online booking URL */
  bookingUrl?: string;

  /** Reservation required */
  reservationRequired?: boolean;
}

/**
 * Price
 */
export interface Price {
  /** Amount */
  amount: number;

  /** Currency code (ISO 4217) */
  currency: string;

  /** Notes (e.g., "under 12", "per person") */
  notes?: string;
}

// ============================================================================
// Accessibility
// ============================================================================

/**
 * Accessibility information
 */
export interface AccessibilityInfo {
  /** Wheelchair accessible */
  wheelchair?: boolean;

  /** Elevator/lift available */
  elevator?: boolean;

  /** Accessible parking */
  accessibleParking?: boolean;

  /** Accessible restrooms */
  accessibleRestrooms?: boolean;

  /** Audio guides */
  audioGuide?: LanguageCode[];

  /** Braille signage */
  braille?: boolean;

  /** Sign language interpretation */
  signLanguage?: string[];

  /** Service animals allowed */
  serviceAnimals?: boolean;

  /** Assistive listening devices */
  hearingAssistance?: boolean;

  /** Visual assistance */
  visualAssistance?: boolean;

  /** Detailed accessibility notes */
  notes?: MultilingualText;

  /** Accessibility rating (0-5) */
  rating?: number;
}

// ============================================================================
// Media
// ============================================================================

/**
 * Media gallery
 */
export interface MediaGallery {
  /** Photos */
  photos?: Photo[];

  /** Videos */
  videos?: Video[];

  /** 360° panoramas */
  panoramas?: Panorama[];

  /** Virtual tour URL */
  virtualTour?: string;
}

/**
 * Photo
 */
export interface Photo {
  /** Photo URL */
  url: string;

  /** Thumbnail URL */
  thumbnailUrl?: string;

  /** Caption */
  caption?: MultilingualText;

  /** Photographer credit */
  credit?: string;

  /** Dimensions */
  width?: number;
  height?: number;

  /** Tags */
  tags?: string[];
}

/**
 * Video
 */
export interface Video {
  /** Video URL */
  url: string;

  /** Thumbnail URL */
  thumbnailUrl?: string;

  /** Title */
  title?: MultilingualText;

  /** Duration (seconds) */
  duration?: number;

  /** Platform (youtube, vimeo, etc.) */
  platform?: string;
}

/**
 * Panorama (360° photo)
 */
export interface Panorama {
  /** Panorama URL */
  url: string;

  /** Thumbnail URL */
  thumbnailUrl?: string;

  /** Caption */
  caption?: MultilingualText;

  /** Coordinates where taken */
  location?: GeoCoordinates;
}

// ============================================================================
// Contact & Social Media
// ============================================================================

/**
 * Contact information
 */
export interface ContactInfo {
  /** Phone number */
  phone?: string;

  /** Email address */
  email?: string;

  /** WhatsApp number */
  whatsapp?: string;

  /** Fax number */
  fax?: string;
}

/**
 * Social media links
 */
export interface SocialMediaLinks {
  facebook?: string;
  instagram?: string;
  twitter?: string;
  youtube?: string;
  tiktok?: string;
  linkedin?: string;
  tripadvisor?: string;
}

// ============================================================================
// UNESCO World Heritage
// ============================================================================

/**
 * UNESCO World Heritage information
 */
export interface UNESCOInfo {
  /** UNESCO site ID */
  siteId: number;

  /** Inscription year */
  inscriptionYear: number;

  /** Heritage category */
  category: 'cultural' | 'natural' | 'mixed';

  /** Selection criteria */
  criteria: string[];

  /** Endangered status */
  endangered?: boolean;

  /** Official UNESCO URL */
  url?: string;
}

// ============================================================================
// Crowd Density
// ============================================================================

/**
 * Real-time crowd density
 */
export interface CrowdDensity {
  /** Current number of visitors */
  current: number;

  /** Maximum capacity */
  capacity: number;

  /** Occupancy percentage (0-100) */
  percentage: number;

  /** Crowd level */
  level: 'low' | 'moderate' | 'high' | 'very-high' | 'at-capacity';

  /** Estimated wait time (minutes) */
  estimatedWaitTime?: number;

  /** Timestamp */
  timestamp: Date;

  /** Forecast for next hours */
  forecast?: {
    time: Date;
    expectedLevel: 'low' | 'moderate' | 'high' | 'very-high';
  }[];
}

// ============================================================================
// Destination
// ============================================================================

/**
 * Destination (city, region, country)
 */
export interface Destination {
  /** Unique identifier */
  id: string;

  /** Destination name */
  name: string;

  /** Multilingual names */
  names?: MultilingualText;

  /** Destination type */
  type: 'country' | 'region' | 'city' | 'neighborhood' | 'area';

  /** Parent destination (e.g., Paris -> France) */
  parentId?: string;

  /** Description */
  description: MultilingualText;

  /** Location */
  location: {
    coordinates: GeoCoordinates;
    bounds?: GeoBounds;
  };

  /** Tagline/slogan */
  tagline?: MultilingualText;

  /** Population */
  population?: number;

  /** Area (square kilometers) */
  area?: number;

  /** Time zone(s) */
  timeZones?: string[];

  /** Currency */
  currency?: {
    code: string;
    name: string;
    symbol: string;
  };

  /** Languages spoken */
  languages?: LanguageCode[];

  /** Climate information */
  climate?: ClimateInfo;

  /** Best seasons to visit */
  bestSeasons?: ('spring' | 'summer' | 'fall' | 'winter')[];

  /** Average daily budget */
  averageBudget?: Price;

  /** Safety rating (0-5) */
  safetyRating?: number;

  /** Top attractions */
  topAttractions?: string[];

  /** Media */
  media?: MediaGallery;

  /** Statistics */
  statistics?: DestinationStatistics;

  /** Last updated */
  updatedAt: Date;
}

/**
 * Climate information
 */
export interface ClimateInfo {
  /** Climate type */
  type?: string;

  /** Average temperatures (Celsius) */
  temperatures?: {
    jan?: number; feb?: number; mar?: number; apr?: number;
    may?: number; jun?: number; jul?: number; aug?: number;
    sep?: number; oct?: number; nov?: number; dec?: number;
  };

  /** Average rainfall (mm) */
  rainfall?: {
    jan?: number; feb?: number; mar?: number; apr?: number;
    may?: number; jun?: number; jul?: number; aug?: number;
    sep?: number; oct?: number; nov?: number; dec?: number;
  };
}

/**
 * Destination statistics
 */
export interface DestinationStatistics {
  /** Annual visitors */
  annualVisitors?: number;

  /** Tourism revenue */
  tourismRevenue?: Price;

  /** Number of hotels */
  numberOfHotels?: number;

  /** Number of restaurants */
  numberOfRestaurants?: number;

  /** Number of attractions */
  numberOfAttractions?: number;
}

// ============================================================================
// Point of Interest (POI)
// ============================================================================

/**
 * Point of Interest
 */
export interface POI {
  /** Unique identifier */
  id: string;

  /** POI name */
  name: string;

  /** Multilingual names */
  names?: MultilingualText;

  /** POI type */
  type: POIType;

  /** Location */
  location: {
    coordinates: GeoCoordinates;
    address: Address;
  };

  /** Rating (0-5) */
  rating?: number;

  /** Price range */
  priceRange?: '$' | '$$' | '$$$' | '$$$$';

  /** Currently open */
  openNow?: boolean;

  /** Opening hours */
  openingHours?: OpeningHours;

  /** Contact info */
  contact?: ContactInfo;

  /** Website */
  website?: string;

  /** Features/amenities */
  features?: string[];

  /** Accessibility */
  accessibility?: AccessibilityInfo;

  /** Cuisine (for restaurants) */
  cuisine?: string[];

  /** Sustainable/eco-friendly */
  sustainable?: boolean;

  /** Distance from user (meters) - calculated */
  distance?: number;
}

// ============================================================================
// Visitor Statistics
// ============================================================================

/**
 * Visitor statistics
 */
export interface VisitorStatistics {
  /** Attraction/destination ID */
  id: string;

  /** Time period */
  period: {
    start: Date;
    end: Date;
  };

  /** Granularity */
  granularity: 'hourly' | 'daily' | 'weekly' | 'monthly' | 'yearly';

  /** Total visitors */
  totalVisitors: number;

  /** Visitor data points */
  dataPoints: VisitorDataPoint[];

  /** Demographics */
  demographics?: VisitorDemographics;

  /** Origin analysis */
  origins?: VisitorOrigins;

  /** Revenue data */
  revenue?: RevenueData;

  /** Satisfaction metrics */
  satisfaction?: SatisfactionMetrics;
}

/**
 * Visitor data point
 */
export interface VisitorDataPoint {
  /** Timestamp */
  timestamp: Date;

  /** Number of visitors */
  visitors: number;

  /** Revenue (optional) */
  revenue?: number;

  /** Average satisfaction (0-5) */
  satisfaction?: number;
}

/**
 * Visitor demographics
 */
export interface VisitorDemographics {
  /** Age distribution */
  ageGroups?: {
    '0-17'?: number;
    '18-24'?: number;
    '25-34'?: number;
    '35-44'?: number;
    '45-54'?: number;
    '55-64'?: number;
    '65+'?: number;
  };

  /** Gender distribution */
  gender?: {
    male?: number;
    female?: number;
    other?: number;
  };

  /** Travel type */
  travelType?: {
    solo?: number;
    couple?: number;
    family?: number;
    group?: number;
    business?: number;
  };
}

/**
 * Visitor origins
 */
export interface VisitorOrigins {
  /** By country (ISO codes) */
  countries?: Record<string, number>;

  /** By region */
  regions?: Record<string, number>;

  /** Domestic vs international */
  domesticVsInternational?: {
    domestic: number;
    international: number;
  };
}

/**
 * Revenue data
 */
export interface RevenueData {
  /** Total revenue */
  total: Price;

  /** Revenue per visitor */
  perVisitor?: Price;

  /** Revenue by source */
  bySource?: {
    tickets?: Price;
    merchandise?: Price;
    food?: Price;
    other?: Price;
  };
}

/**
 * Satisfaction metrics
 */
export interface SatisfactionMetrics {
  /** Overall satisfaction (0-5) */
  overall: number;

  /** Net Promoter Score (-100 to 100) */
  nps?: number;

  /** Repeat visit rate (0-1) */
  repeatVisitRate?: number;

  /** Would recommend (0-1) */
  wouldRecommend?: number;
}

// ============================================================================
// Search & Query
// ============================================================================

/**
 * Attraction search parameters
 */
export interface AttractionSearchParams {
  /** Location-based search */
  location?: GeoCoordinates;

  /** Search radius (meters) */
  radius?: number;

  /** Bounding box */
  bounds?: GeoBounds;

  /** Categories filter */
  categories?: AttractionCategory[];

  /** Types filter */
  types?: AttractionType[];

  /** Minimum rating */
  minRating?: number;

  /** Accessibility requirements */
  accessibility?: string[];

  /** Languages available */
  languages?: LanguageCode[];

  /** Free admission only */
  freeOnly?: boolean;

  /** Currently open */
  openNow?: boolean;

  /** UNESCO sites only */
  unescoOnly?: boolean;

  /** Text search query */
  query?: string;

  /** Sort by */
  sortBy?: 'relevance' | 'rating' | 'distance' | 'popularity' | 'name';

  /** Results limit */
  limit?: number;

  /** Results offset */
  offset?: number;
}

/**
 * POI search parameters
 */
export interface POISearchParams {
  /** Location */
  location?: GeoCoordinates;

  /** Search radius (meters) */
  radius?: number;

  /** Bounding box */
  bounds?: GeoBounds;

  /** POI types */
  types?: POIType[];

  /** Minimum rating */
  minRating?: number;

  /** Price range */
  priceRange?: ('$' | '$$' | '$$$' | '$$$$')[];

  /** Currently open */
  openNow?: boolean;

  /** Cuisine types (for restaurants) */
  cuisine?: string[];

  /** Sustainable/eco-friendly only */
  sustainable?: boolean;

  /** Text search */
  query?: string;

  /** Sort by */
  sortBy?: 'distance' | 'rating' | 'price' | 'relevance';

  /** Results limit */
  limit?: number;

  /** Results offset */
  offset?: number;
}

// ============================================================================
// Cultural Heritage
// ============================================================================

/**
 * Cultural heritage site
 */
export interface CulturalHeritageSite {
  /** Unique identifier */
  id: string;

  /** Site name */
  name: string;

  /** Multilingual names */
  names?: MultilingualText;

  /** Description */
  description: MultilingualText;

  /** Location */
  location: {
    coordinates: GeoCoordinates;
    address: Address;
  };

  /** Heritage type */
  type: 'monument' | 'building' | 'site' | 'landscape' | 'district';

  /** Historical period */
  historicalPeriod?: string;

  /** Year built/established */
  yearEstablished?: number;

  /** UNESCO World Heritage */
  unesco?: UNESCOInfo;

  /** Protection status */
  protectionStatus?: {
    level: 'national' | 'regional' | 'local' | 'unesco';
    designation: string;
  };

  /** Conservation state */
  conservationState?: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Threats */
  threats?: string[];

  /** Visit information */
  visitInfo?: VisitInfo;

  /** Media */
  media?: MediaGallery;

  /** Last updated */
  updatedAt: Date;
}

// ============================================================================
// Local Experiences
// ============================================================================

/**
 * Local experience/activity
 */
export interface LocalExperience {
  /** Unique identifier */
  id: string;

  /** Experience name */
  name: string;

  /** Multilingual names */
  names?: MultilingualText;

  /** Description */
  description: MultilingualText;

  /** Experience type */
  type: 'tour' | 'workshop' | 'class' | 'tasting' | 'performance' | 'adventure' | 'cultural';

  /** Location */
  location: {
    coordinates: GeoCoordinates;
    meetingPoint?: string;
  };

  /** Duration (minutes) */
  duration: number;

  /** Group size */
  groupSize?: {
    min?: number;
    max?: number;
  };

  /** Price */
  price?: Price;

  /** Languages offered */
  languages?: LanguageCode[];

  /** Rating */
  rating?: number;

  /** Includes */
  includes?: string[];

  /** Requirements */
  requirements?: string[];

  /** Difficulty level */
  difficulty?: 'easy' | 'moderate' | 'challenging' | 'difficult';

  /** Booking required */
  bookingRequired?: boolean;

  /** Booking URL */
  bookingUrl?: string;

  /** Sustainable/authentic */
  sustainable?: boolean;

  /** Provider */
  provider?: {
    name: string;
    contact?: ContactInfo;
  };
}

// ============================================================================
// Seasonality
// ============================================================================

/**
 * Tourism seasonality data
 */
export interface SeasonalityData {
  /** Location/attraction ID */
  id: string;

  /** Seasonal patterns */
  seasons: {
    /** Season name */
    name: 'spring' | 'summer' | 'fall' | 'winter' | 'peak' | 'shoulder' | 'low';

    /** Months */
    months: number[];

    /** Visitor volume (relative, 0-1) */
    visitorVolume: number;

    /** Price level (relative, 0-1) */
    priceLevel: number;

    /** Crowd level */
    crowdLevel: 'low' | 'moderate' | 'high' | 'very-high';

    /** Weather conditions */
    weather?: string;

    /** Special events */
    events?: string[];
  }[];

  /** Monthly breakdown */
  monthly?: {
    month: number;
    visitors: number;
    temperature: number;
    rainfall: number;
    crowdLevel: 'low' | 'moderate' | 'high' | 'very-high';
  }[];
}

// ============================================================================
// Safety & Health
// ============================================================================

/**
 * Safety information
 */
export interface SafetyInfo {
  /** Overall safety rating (0-5) */
  overall: number;

  /** Crime level */
  crimeLevel: 'very-low' | 'low' | 'moderate' | 'high' | 'very-high';

  /** Travel advisories */
  advisories?: {
    level: 1 | 2 | 3 | 4;
    message: string;
    source: string;
    date: Date;
  }[];

  /** Health risks */
  healthRisks?: string[];

  /** Vaccination requirements */
  vaccinations?: {
    required?: string[];
    recommended?: string[];
  };

  /** Emergency numbers */
  emergency?: {
    police?: string;
    ambulance?: string;
    fire?: string;
    tourist?: string;
  };

  /** Safe areas */
  safeAreas?: string[];

  /** Areas to avoid */
  areasToAvoid?: string[];

  /** Safety tips */
  tips?: MultilingualText;
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
 * Pagination metadata
 */
export interface PaginationMeta {
  /** Total results */
  total: number;

  /** Results limit */
  limit: number;

  /** Results offset */
  offset: number;

  /** Has more results */
  hasMore: boolean;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Results */
  results: T[];

  /** Pagination metadata */
  pagination: PaginationMeta;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Tourism data error codes
 */
export enum TourismErrorCode {
  INVALID_LOCATION = 'T001',
  ATTRACTION_NOT_FOUND = 'T002',
  DESTINATION_NOT_FOUND = 'T003',
  INVALID_SEARCH_PARAMS = 'T004',
  API_RATE_LIMIT = 'T005',
  UNAUTHORIZED = 'T006',
  INVALID_LANGUAGE = 'T007',
  DATA_QUALITY_LOW = 'T008',
  SERVICE_UNAVAILABLE = 'T009',
  INVALID_DATE_RANGE = 'T010',
}

/**
 * Tourism data error
 */
export class TourismDataError extends Error {
  constructor(
    public code: TourismErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'TourismDataError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  GeoCoordinates,
  GeoBounds,
  Address,
  MultilingualText,
  LanguageCode,
  AttractionCategory,
  AttractionType,
  POIType,
  Attraction,
  AttractionRatings,
  Review,
  VisitInfo,
  OpeningHours,
  TicketInfo,
  Price,
  AccessibilityInfo,
  MediaGallery,
  Photo,
  Video,
  Panorama,
  ContactInfo,
  SocialMediaLinks,
  UNESCOInfo,
  CrowdDensity,
  Destination,
  ClimateInfo,
  DestinationStatistics,
  POI,
  VisitorStatistics,
  VisitorDataPoint,
  VisitorDemographics,
  VisitorOrigins,
  RevenueData,
  SatisfactionMetrics,
  AttractionSearchParams,
  POISearchParams,
  CulturalHeritageSite,
  LocalExperience,
  SeasonalityData,
  SafetyInfo,
  PaginationMeta,
  PaginatedResponse,
};

export { TourismErrorCode, TourismDataError };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
