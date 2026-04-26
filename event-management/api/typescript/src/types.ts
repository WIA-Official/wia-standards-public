/**
 * WIA-IND-018: Event Management - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Event Types
// ============================================================================

/**
 * Address information
 */
export interface Address {
  street: string;
  city: string;
  state?: string;
  country: string;
  postalCode: string;
  coordinates?: {
    latitude: number;
    longitude: number;
  };
}

/**
 * Event types
 */
export type EventType =
  | 'conference'
  | 'workshop'
  | 'webinar'
  | 'exhibition'
  | 'networking'
  | 'hybrid'
  | 'seminar'
  | 'trade-show'
  | 'summit'
  | 'symposium';

/**
 * Event format
 */
export type EventFormat = 'in-person' | 'virtual' | 'hybrid';

/**
 * Event status
 */
export type EventStatus =
  | 'draft'
  | 'planning'
  | 'published'
  | 'registration-open'
  | 'registration-closed'
  | 'in-progress'
  | 'completed'
  | 'cancelled'
  | 'archived';

/**
 * Main event entity
 */
export interface Event {
  /** Unique event identifier */
  id: string;

  /** Event title */
  title: string;

  /** Event description */
  description: string;

  /** Event type */
  type: EventType;

  /** Event format */
  format: EventFormat;

  /** Event status */
  status: EventStatus;

  /** Start date and time */
  startTime: Date;

  /** End date and time */
  endTime: Date;

  /** Timezone (IANA format) */
  timezone: string;

  /** Venue information */
  venue: VenueInfo;

  /** Capacity limits */
  capacity: CapacityInfo;

  /** Event organizer */
  organizer: OrganizerInfo;

  /** Registration settings */
  registration: RegistrationSettings;

  /** Event branding */
  branding: BrandingInfo;

  /** Event tags/categories */
  tags: string[];

  /** Event metadata */
  metadata: EventMetadata;

  /** Created timestamp */
  createdAt: Date;

  /** Last updated timestamp */
  updatedAt: Date;
}

/**
 * Venue information
 */
export interface VenueInfo {
  /** Physical venue (if applicable) */
  physical?: PhysicalVenue;

  /** Virtual platform (if applicable) */
  virtual?: VirtualPlatform;
}

/**
 * Physical venue details
 */
export interface PhysicalVenue {
  /** Venue name */
  name: string;

  /** Venue address */
  address: Address;

  /** Total capacity */
  capacity: number;

  /** Available rooms */
  rooms?: VenueRoom[];

  /** Amenities */
  amenities: VenueAmenities;

  /** Contact information */
  contact?: ContactInfo;
}

/**
 * Venue room
 */
export interface VenueRoom {
  /** Room identifier */
  id: string;

  /** Room name */
  name: string;

  /** Room capacity */
  capacity: number;

  /** Room setup type */
  setup: 'theater' | 'classroom' | 'banquet' | 'boardroom' | 'u-shape' | 'custom';

  /** Room dimensions (in meters) */
  dimensions?: {
    length: number;
    width: number;
    height: number;
  };

  /** Available equipment */
  equipment: string[];
}

/**
 * Venue amenities
 */
export interface VenueAmenities {
  /** WiFi availability */
  wifi: {
    available: boolean;
    bandwidth?: string;
    guestNetwork?: boolean;
  };

  /** Audio-visual equipment */
  av: {
    projectors?: number;
    screens?: number;
    soundSystem?: boolean;
    recording?: boolean;
  };

  /** Catering options */
  catering: {
    onSite: boolean;
    vendors?: string[];
    dietary: string[];
  };

  /** Parking information */
  parking?: {
    spaces: number;
    cost?: number;
    validation?: boolean;
  };

  /** Accessibility features */
  accessibility: {
    wheelchairAccessible: boolean;
    elevators?: boolean;
    signLanguage?: boolean;
    hearingAssistance?: boolean;
  };
}

/**
 * Virtual platform details
 */
export interface VirtualPlatform {
  /** Platform name */
  platform: 'zoom' | 'hopin' | 'teams' | 'webex' | 'custom';

  /** Platform URL */
  url?: string;

  /** Virtual capacity */
  capacity: number;

  /** Platform features */
  features: {
    chat?: boolean;
    qna?: boolean;
    polls?: boolean;
    breakoutRooms?: boolean;
    recording?: boolean;
    streaming?: boolean;
  };
}

/**
 * Capacity information
 */
export interface CapacityInfo {
  /** Total capacity */
  total: number;

  /** Physical capacity (for hybrid) */
  physical?: number;

  /** Virtual capacity (for hybrid) */
  virtual?: number;

  /** Current registrations */
  registered: number;

  /** Checked-in count */
  checkedIn?: number;

  /** Waitlist enabled */
  waitlistEnabled: boolean;

  /** Current waitlist count */
  waitlistCount?: number;
}

/**
 * Organizer information
 */
export interface OrganizerInfo {
  /** Organization name */
  name: string;

  /** Organization description */
  description?: string;

  /** Organization logo */
  logo?: string;

  /** Organization website */
  website?: string;

  /** Contact information */
  contact: ContactInfo;
}

/**
 * Contact information
 */
export interface ContactInfo {
  /** Contact name */
  name: string;

  /** Email address */
  email: string;

  /** Phone number */
  phone?: string;
}

/**
 * Registration settings
 */
export interface RegistrationSettings {
  /** Registration opens at */
  opensAt: Date;

  /** Registration closes at */
  closesAt: Date;

  /** Requires approval */
  requiresApproval: boolean;

  /** Allow waitlist */
  allowWaitlist: boolean;

  /** Maximum capacity */
  maxCapacity: number;

  /** Early bird deadline */
  earlyBirdDeadline?: Date;

  /** Refund policy */
  refundPolicy?: string;
}

/**
 * Branding information
 */
export interface BrandingInfo {
  /** Primary color */
  primaryColor: string;

  /** Secondary color */
  secondaryColor?: string;

  /** Logo URL */
  logo: string;

  /** Banner image */
  banner?: string;

  /** Custom CSS */
  customCss?: string;
}

/**
 * Event metadata
 */
export interface EventMetadata {
  /** Target audience */
  targetAudience?: string;

  /** Expected attendees */
  expectedAttendees?: number;

  /** Budget information */
  budget?: {
    total: number;
    currency: string;
  };

  /** Custom fields */
  [key: string]: unknown;
}

// ============================================================================
// Registration Types
// ============================================================================

/**
 * Registration status
 */
export type RegistrationStatus =
  | 'pending'
  | 'confirmed'
  | 'cancelled'
  | 'refunded'
  | 'waitlist';

/**
 * Ticket type
 */
export type TicketType =
  | 'general'
  | 'vip'
  | 'student'
  | 'speaker'
  | 'sponsor'
  | 'media'
  | 'virtual'
  | 'custom';

/**
 * Attendee registration
 */
export interface Registration {
  /** Registration ID */
  id: string;

  /** Event ID */
  eventId: string;

  /** Registration status */
  status: RegistrationStatus;

  /** Attendee information */
  attendee: AttendeeInfo;

  /** Ticket information */
  ticket: TicketInfo;

  /** Attendee preferences */
  preferences: AttendeePreferences;

  /** Check-in information */
  checkIn?: CheckInInfo;

  /** Registration metadata */
  metadata: RegistrationMetadata;

  /** Registered timestamp */
  registeredAt: Date;

  /** Updated timestamp */
  updatedAt: Date;
}

/**
 * Attendee information
 */
export interface AttendeeInfo {
  /** Attendee ID */
  id: string;

  /** First name */
  firstName: string;

  /** Last name */
  lastName: string;

  /** Email address */
  email: string;

  /** Phone number */
  phone?: string;

  /** Company */
  company?: string;

  /** Job title */
  jobTitle?: string;

  /** Profile photo */
  profilePhoto?: string;

  /** Bio */
  bio?: string;

  /** Social media */
  social?: {
    twitter?: string;
    linkedin?: string;
    website?: string;
  };
}

/**
 * Ticket information
 */
export interface TicketInfo {
  /** Ticket type */
  type: TicketType;

  /** Base price */
  price: number;

  /** Currency code */
  currency: string;

  /** Add-ons */
  addOns?: TicketAddOn[];

  /** Discount code */
  discountCode?: string;

  /** Discount amount */
  discountAmount?: number;

  /** Total paid */
  totalPaid: number;

  /** Payment method */
  paymentMethod?: string;

  /** Payment status */
  paymentStatus: 'pending' | 'completed' | 'failed' | 'refunded';
}

/**
 * Ticket add-on
 */
export interface TicketAddOn {
  /** Add-on ID */
  id: string;

  /** Add-on name */
  name: string;

  /** Add-on price */
  price: number;

  /** Quantity */
  quantity: number;
}

/**
 * Attendee preferences
 */
export interface AttendeePreferences {
  /** Dietary restrictions */
  dietary?: string[];

  /** Accessibility needs */
  accessibility?: {
    wheelchairAccess?: boolean;
    signLanguage?: boolean;
    other?: string;
  };

  /** Interests/topics */
  interests?: string[];

  /** Preferred sessions */
  sessions?: string[];

  /** Networking preferences */
  networking?: {
    enabled: boolean;
    visibility: 'public' | 'connections' | 'private';
  };
}

/**
 * Check-in information
 */
export interface CheckInInfo {
  /** Checked in */
  checkedIn: boolean;

  /** Check-in timestamp */
  checkInTime?: Date;

  /** Check-in method */
  checkInMethod?: 'qr-code' | 'manual' | 'nfc' | 'facial-recognition';

  /** Badge printed */
  badgePrinted?: boolean;

  /** Badge number */
  badgeNumber?: string;
}

/**
 * Registration metadata
 */
export interface RegistrationMetadata {
  /** Source */
  source: 'direct' | 'social' | 'referral' | 'organic';

  /** Referrer */
  referrer?: string;

  /** UTM parameters */
  utm?: {
    source?: string;
    medium?: string;
    campaign?: string;
  };

  /** IP address */
  ipAddress?: string;

  /** User agent */
  userAgent?: string;
}

// ============================================================================
// Session Types
// ============================================================================

/**
 * Session type
 */
export type SessionType =
  | 'keynote'
  | 'talk'
  | 'panel'
  | 'workshop'
  | 'demo'
  | 'networking'
  | 'break'
  | 'lunch'
  | 'reception';

/**
 * Session level
 */
export type SessionLevel = 'beginner' | 'intermediate' | 'advanced' | 'all';

/**
 * Event session
 */
export interface Session {
  /** Session ID */
  id: string;

  /** Event ID */
  eventId: string;

  /** Session type */
  type: SessionType;

  /** Session title */
  title: string;

  /** Session description */
  description: string;

  /** Learning objectives */
  learningObjectives?: string[];

  /** Session level */
  level: SessionLevel;

  /** Track name */
  track?: string;

  /** Tags */
  tags: string[];

  /** Speakers */
  speakers: SessionSpeaker[];

  /** Schedule */
  schedule: SessionSchedule;

  /** Venue */
  venue: SessionVenue;

  /** Resources */
  resources?: SessionResources;

  /** Attendance */
  attendance: SessionAttendance;

  /** Engagement metrics */
  engagement?: SessionEngagement;

  /** Requirements */
  requirements?: SessionRequirements;

  /** Created timestamp */
  createdAt: Date;

  /** Updated timestamp */
  updatedAt: Date;
}

/**
 * Session speaker
 */
export interface SessionSpeaker {
  /** Speaker ID */
  speakerId: string;

  /** Speaker role */
  role: 'primary' | 'co-presenter' | 'moderator' | 'panelist';
}

/**
 * Session schedule
 */
export interface SessionSchedule {
  /** Date */
  date: Date;

  /** Start time */
  startTime: Date;

  /** End time */
  endTime: Date;

  /** Duration (minutes) */
  duration: number;

  /** Timezone */
  timezone: string;
}

/**
 * Session venue
 */
export interface SessionVenue {
  /** Room name/ID */
  room: string;

  /** Capacity */
  capacity: number;

  /** Setup type */
  setup?: string;

  /** Virtual info */
  virtual?: {
    enabled: boolean;
    url?: string;
    platform?: string;
  };
}

/**
 * Session resources
 */
export interface SessionResources {
  /** Slides URL */
  slides?: string;

  /** Handouts */
  handouts?: string[];

  /** Recording URL */
  recording?: string;

  /** Additional links */
  additionalLinks?: string[];
}

/**
 * Session attendance
 */
export interface SessionAttendance {
  /** Registered count */
  registered: number;

  /** Capacity */
  capacity: number;

  /** Waitlist count */
  waitlist: number;

  /** Attended count */
  attended: number;

  /** Completion rate */
  completionRate?: number;
}

/**
 * Session engagement
 */
export interface SessionEngagement {
  /** Questions asked */
  questions: number;

  /** Poll responses */
  pollResponses: number;

  /** Chat messages */
  chatMessages: number;

  /** Average rating */
  rating?: number;
}

/**
 * Session requirements
 */
export interface SessionRequirements {
  /** Registration required */
  registration: 'required' | 'optional' | 'walk-in';

  /** Prerequisites */
  prerequisites?: string[];

  /** Materials needed */
  materials?: string[];

  /** Additional cost */
  cost?: number;
}

// ============================================================================
// Speaker Types
// ============================================================================

/**
 * Speaker status
 */
export type SpeakerStatus =
  | 'invited'
  | 'confirmed'
  | 'declined'
  | 'tentative'
  | 'cancelled';

/**
 * Speaker type
 */
export type SpeakerType = 'keynote' | 'presenter' | 'panelist' | 'workshop-leader' | 'mc';

/**
 * Speaker
 */
export interface Speaker {
  /** Speaker ID */
  id: string;

  /** Speaker type */
  type: SpeakerType;

  /** Personal information */
  personal: SpeakerPersonal;

  /** Professional information */
  professional: SpeakerProfessional;

  /** Social media */
  social?: SpeakerSocial;

  /** Associated sessions */
  sessions: string[];

  /** Requirements */
  requirements?: SpeakerRequirements;

  /** Compensation */
  compensation?: SpeakerCompensation;

  /** Contract info */
  contract?: SpeakerContract;

  /** Status */
  status: SpeakerStatus;

  /** Created timestamp */
  createdAt: Date;

  /** Updated timestamp */
  updatedAt: Date;
}

/**
 * Speaker personal information
 */
export interface SpeakerPersonal {
  firstName: string;
  lastName: string;
  title?: string;
  company?: string;
  email: string;
  phone?: string;
  photo?: {
    url: string;
    highRes?: boolean;
  };
}

/**
 * Speaker professional information
 */
export interface SpeakerProfessional {
  bio: {
    short: string;
    long?: string;
  };
  expertise: string[];
  previousSpeaking?: Array<{
    event: string;
    date: Date;
    topic: string;
  }>;
  publications?: string[];
  awards?: string[];
}

/**
 * Speaker social media
 */
export interface SpeakerSocial {
  twitter?: string;
  linkedin?: string;
  website?: string;
  instagram?: string;
}

/**
 * Speaker requirements
 */
export interface SpeakerRequirements {
  av?: {
    laptop?: 'own' | 'provided';
    connectors?: string[];
    microphone?: 'lapel' | 'handheld' | 'headset';
    clicker?: boolean;
    internet?: 'required' | 'preferred' | 'not-needed';
  };
  room?: {
    greenRoom?: boolean;
    privateArea?: boolean;
    secureStorage?: boolean;
  };
  travel?: {
    flight?: {
      required: boolean;
      class?: 'economy' | 'business' | 'first';
    };
    hotel?: {
      required: boolean;
      nights?: number;
    };
    ground?: {
      pickup?: boolean;
      rental?: boolean;
    };
  };
  dietary?: string[];
  accessibility?: string[];
}

/**
 * Speaker compensation
 */
export interface SpeakerCompensation {
  type: 'none' | 'honorarium' | 'fee' | 'expenses-only';
  amount?: number;
  currency?: string;
  paid?: boolean;
  paidDate?: Date;
}

/**
 * Speaker contract
 */
export interface SpeakerContract {
  signed: boolean;
  signedDate?: Date;
  documentUrl?: string;
  terms?: {
    recordingPermission?: boolean;
    photoPermission?: boolean;
    materialSharing?: boolean;
    exclusivity?: boolean;
  };
}

// ============================================================================
// Sponsor Types
// ============================================================================

/**
 * Sponsor tier
 */
export type SponsorTier =
  | 'diamond'
  | 'platinum'
  | 'gold'
  | 'silver'
  | 'bronze'
  | 'custom';

/**
 * Sponsor
 */
export interface Sponsor {
  /** Sponsor ID */
  id: string;

  /** Event ID */
  eventId: string;

  /** Company information */
  company: SponsorCompany;

  /** Sponsor tier */
  tier: SponsorTier;

  /** Investment details */
  investment: SponsorInvestment;

  /** Benefits */
  benefits: SponsorBenefit[];

  /** Booth information */
  booth?: SponsorBooth;

  /** Lead information */
  leads?: SponsorLeads;

  /** Marketing metrics */
  marketing?: SponsorMarketing;

  /** Contacts */
  contacts: SponsorContact[];

  /** Contract */
  contract?: {
    signed: boolean;
    signedDate?: Date;
    documentUrl?: string;
  };

  /** Satisfaction */
  satisfaction?: SponsorSatisfaction;

  /** Created timestamp */
  createdAt: Date;

  /** Updated timestamp */
  updatedAt: Date;
}

/**
 * Sponsor company
 */
export interface SponsorCompany {
  name: string;
  logo: {
    url: string;
    formats?: string[];
  };
  description?: string;
  website?: string;
  industry?: string;
  size?: 'startup' | 'small' | 'medium' | 'enterprise';
}

/**
 * Sponsor investment
 */
export interface SponsorInvestment {
  amount: number;
  currency: string;
  paymentTerms: 'upfront' | 'installments' | 'post-event';
  paid: boolean;
  paidDate?: Date;
  invoiceNumber?: string;
}

/**
 * Sponsor benefit
 */
export interface SponsorBenefit {
  type: 'booth' | 'speaking-slot' | 'tickets' | 'branding' | 'leads' | 'email' | 'social';
  description: string;
  quantity?: number;
  delivered: boolean;
}

/**
 * Sponsor booth
 */
export interface SponsorBooth {
  number: string;
  location: string;
  size: '10x10' | '20x20' | '30x30' | 'custom';
  electrical: boolean;
  internet: boolean;
  furniture?: string[];
  staff?: Array<{
    name: string;
    email: string;
    tickets: number;
  }>;
}

/**
 * Sponsor leads
 */
export interface SponsorLeads {
  systemProvided: boolean;
  captured: number;
  exported: boolean;
  exportDate?: Date;
}

/**
 * Sponsor marketing
 */
export interface SponsorMarketing {
  emailsSent: number;
  emailOpens: number;
  emailClicks: number;
  socialPosts: number;
  socialReach: number;
  socialEngagement: number;
  websiteImpressions: number;
}

/**
 * Sponsor contact
 */
export interface SponsorContact {
  name: string;
  role: 'primary' | 'billing' | 'marketing' | 'logistics';
  email: string;
  phone?: string;
}

/**
 * Sponsor satisfaction
 */
export interface SponsorSatisfaction {
  rating: number;
  feedback?: string;
  wouldSponsorAgain: boolean;
  roi: 'positive' | 'neutral' | 'negative';
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Event analytics
 */
export interface EventAnalytics {
  /** Event ID */
  eventId: string;

  /** Attendance metrics */
  attendance: AttendanceMetrics;

  /** Engagement metrics */
  engagement: EngagementMetrics;

  /** Revenue metrics */
  revenue: RevenueMetrics;

  /** Satisfaction metrics */
  satisfaction?: SatisfactionMetrics;

  /** Generated timestamp */
  generatedAt: Date;
}

/**
 * Attendance metrics
 */
export interface AttendanceMetrics {
  totalRegistrations: number;
  checkedIn: number;
  noShows: number;
  attendanceRate: number;
  byTicketType: Record<string, number>;
  byDay?: Record<string, number>;
  peakAttendance?: {
    count: number;
    time: Date;
  };
}

/**
 * Engagement metrics
 */
export interface EngagementMetrics {
  chat: {
    messages: number;
    participants: number;
    messagesPerUser: number;
  };
  qna: {
    questions: number;
    answered: number;
    answerRate: number;
  };
  polls: {
    conducted: number;
    responses: number;
    participationRate: number;
  };
  networking: {
    connections: number;
    meetings: number;
    cardsExchanged: number;
  };
}

/**
 * Revenue metrics
 */
export interface RevenueMetrics {
  tickets: {
    sold: number;
    revenue: number;
    byType: Record<string, number>;
  };
  sponsorships: {
    secured: number;
    revenue: number;
    byTier: Record<string, number>;
  };
  total: number;
  expenses?: number;
  profit?: number;
  roi?: number;
}

/**
 * Satisfaction metrics
 */
export interface SatisfactionMetrics {
  nps: number;
  averageRating: number;
  wouldReturn: number;
  topThemes: string[];
  improvements: string[];
}

// ============================================================================
// Survey Types
// ============================================================================

/**
 * Survey question type
 */
export type QuestionType =
  | 'nps'
  | 'rating'
  | 'multiple-choice'
  | 'checkbox'
  | 'text'
  | 'scale';

/**
 * Survey question
 */
export interface SurveyQuestion {
  id: string;
  type: QuestionType;
  text: string;
  required: boolean;
  options?: string[];
  scale?: number;
}

/**
 * Survey
 */
export interface Survey {
  id: string;
  eventId: string;
  title: string;
  timing: 'during' | 'immediately-after' | 'one-day-after' | 'one-week-after';
  questions: SurveyQuestion[];
  responses?: number;
  createdAt: Date;
}

/**
 * Survey response
 */
export interface SurveyResponse {
  id: string;
  surveyId: string;
  attendeeId: string;
  answers: Record<string, unknown>;
  submittedAt: Date;
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
 * Pagination options
 */
export interface PaginationOptions {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    total: number;
    totalPages: number;
  };
}

/**
 * Filter options
 */
export interface FilterOptions {
  status?: EventStatus[];
  type?: EventType[];
  startDate?: Date;
  endDate?: Date;
  search?: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-IND-018 error codes
 */
export enum EventErrorCode {
  EVENT_NOT_FOUND = 'E001',
  REGISTRATION_CLOSED = 'E002',
  CAPACITY_EXCEEDED = 'E003',
  INVALID_TICKET = 'E004',
  PAYMENT_FAILED = 'E005',
  SESSION_CONFLICT = 'E006',
  SPEAKER_UNAVAILABLE = 'E007',
  INVALID_PARAMETERS = 'E008',
  UNAUTHORIZED = 'E009',
  DUPLICATE_REGISTRATION = 'E010',
}

/**
 * Event management error
 */
export class EventManagementError extends Error {
  constructor(
    public code: EventErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'EventManagementError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core
  Event,
  VenueInfo,
  PhysicalVenue,
  VirtualPlatform,

  // Registration
  Registration,
  AttendeeInfo,
  TicketInfo,

  // Session
  Session,
  SessionSchedule,

  // Speaker
  Speaker,
  SpeakerPersonal,
  SpeakerProfessional,

  // Sponsor
  Sponsor,
  SponsorCompany,

  // Analytics
  EventAnalytics,
  AttendanceMetrics,
  EngagementMetrics,
  RevenueMetrics,

  // Survey
  Survey,
  SurveyQuestion,
  SurveyResponse,

  // Utility
  PaginatedResponse,
  FilterOptions,
};

export { EventErrorCode, EventManagementError };

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
