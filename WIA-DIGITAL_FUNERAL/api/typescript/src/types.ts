/**
 * WIA Digital Funeral Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADigitalFuneral {
  standard: 'WIA-DIGITAL-FUNERAL';
  version: string;
  service: FuneralService;
  deceased: DeceasedInfo;
  ceremony: CeremonyDetails;
  streaming: StreamingConfiguration;
  memorial: MemorialConfiguration;
  guestManagement: GuestManagement;
  condolences: CondolenceConfig;
  extensions?: Record<string, unknown>;
}

export interface FuneralService {
  id: string;
  type: ServiceType;
  status: ServiceStatus;
  provider: ServiceProvider;
  scheduledDate: string;
  timezone: string;
  createdAt: string;
  updatedAt?: string;
}

export type ServiceType = 'traditional' | 'celebration-of-life' | 'memorial' | 'graveside' | 'virtual-only' | 'hybrid';
export type ServiceStatus = 'planning' | 'scheduled' | 'live' | 'completed' | 'archived' | 'cancelled';

export interface ServiceProvider {
  id: string;
  name: string;
  type: 'funeral-home' | 'crematorium' | 'cemetery' | 'virtual-platform' | 'independent';
  contact: ContactInfo;
  license?: string;
}

export interface ContactInfo {
  email: string;
  phone?: string;
  address?: Address;
}

export interface Address {
  street: string;
  city: string;
  state?: string;
  country: string;
  postalCode: string;
}

export interface DeceasedInfo {
  id: string;
  name: string;
  dateOfBirth: string;
  dateOfDeath: string;
  biography?: string;
  photos: MediaAsset[];
  familyMessage?: string;
  culturalBackground?: string;
  religion?: string;
}

export interface MediaAsset {
  id: string;
  type: 'photo' | 'video' | 'audio' | 'document';
  url: string;
  caption?: string;
  uploadedAt: string;
  uploadedBy?: string;
}

// ============================================================================
// Ceremony Types
// ============================================================================

export interface CeremonyDetails {
  venue: VenueInfo;
  program: ProgramElement[];
  officiant?: OfficiantInfo;
  music: MusicSelection[];
  readings: Reading[];
  tributes: Tribute[];
  rituals?: Ritual[];
}

export interface VenueInfo {
  type: 'physical' | 'virtual' | 'hybrid';
  name: string;
  address?: Address;
  virtualUrl?: string;
  capacity?: number;
  accessibility?: AccessibilityFeature[];
}

export type AccessibilityFeature = 'wheelchair' | 'hearing-loop' | 'sign-language' | 'closed-captions' | 'audio-description';

export interface ProgramElement {
  order: number;
  type: ProgramType;
  title: string;
  duration: number;
  presenter?: string;
  notes?: string;
}

export type ProgramType = 'welcome' | 'prayer' | 'eulogy' | 'reading' | 'music' | 'tribute' | 'moment-of-silence' | 'committal' | 'closing';

export interface OfficiantInfo {
  name: string;
  title: string;
  organization?: string;
  contact?: ContactInfo;
}

export interface MusicSelection {
  title: string;
  artist?: string;
  type: 'prelude' | 'processional' | 'during-service' | 'recessional';
  format: 'live' | 'recorded';
  url?: string;
}

export interface Reading {
  title: string;
  source?: string;
  reader: string;
  text?: string;
}

export interface Tribute {
  id: string;
  contributor: string;
  type: 'speech' | 'video' | 'photo-slideshow' | 'written';
  content: string;
  duration?: number;
  approved: boolean;
}

export interface Ritual {
  name: string;
  tradition: string;
  description: string;
  requirements?: string[];
}

// ============================================================================
// Streaming Types
// ============================================================================

export interface StreamingConfiguration {
  enabled: boolean;
  platform: StreamingPlatform;
  quality: StreamQuality;
  access: AccessControl;
  recording: RecordingConfig;
  interactivity: InteractivityConfig;
}

export interface StreamingPlatform {
  provider: 'zoom' | 'youtube' | 'vimeo' | 'facebook' | 'custom';
  streamUrl?: string;
  embedCode?: string;
  backupUrl?: string;
}

export interface StreamQuality {
  resolution: '720p' | '1080p' | '4k';
  bitrate: number;
  adaptiveBitrate: boolean;
}

export interface AccessControl {
  type: 'public' | 'password' | 'invitation-only';
  password?: string;
  requireRegistration: boolean;
  geoRestrictions?: string[];
}

export interface RecordingConfig {
  enabled: boolean;
  retentionDays: number;
  downloadable: boolean;
  editingAllowed: boolean;
}

export interface InteractivityConfig {
  chat: boolean;
  reactions: boolean;
  virtualCandles: boolean;
  virtualFlowers: boolean;
  guestbook: boolean;
}

// ============================================================================
// Memorial Types
// ============================================================================

export interface MemorialConfiguration {
  enabled: boolean;
  type: MemorialType;
  visibility: MemorialVisibility;
  duration: MemorialDuration;
  features: MemorialFeature[];
  customization: MemorialCustomization;
}

export type MemorialType = 'webpage' | 'social-media' | 'qr-memorial' | 'virtual-space' | 'nft';
export type MemorialVisibility = 'public' | 'unlisted' | 'private';
export type MemorialDuration = 'permanent' | '1-year' | '5-years' | '10-years' | 'renewable';

export interface MemorialFeature {
  type: 'timeline' | 'photo-gallery' | 'video-gallery' | 'guestbook' | 'donations' | 'tree-planting' | 'star-naming';
  enabled: boolean;
  config?: Record<string, unknown>;
}

export interface MemorialCustomization {
  theme: string;
  primaryColor?: string;
  backgroundImage?: string;
  font?: string;
  layout?: 'classic' | 'modern' | 'minimal';
}

// ============================================================================
// Guest Management Types
// ============================================================================

export interface GuestManagement {
  invitations: InvitationConfig;
  rsvp: RSVPConfig;
  attendance: AttendanceTracking;
  notifications: NotificationConfig;
}

export interface InvitationConfig {
  method: ('email' | 'sms' | 'mail' | 'social')[];
  template: string;
  sendDate?: string;
  reminder: boolean;
  reminderDays?: number;
}

export interface RSVPConfig {
  enabled: boolean;
  deadline?: string;
  options: RSVPOption[];
  collectDietaryInfo: boolean;
  collectAccessibilityNeeds: boolean;
}

export interface RSVPOption {
  type: 'attending-in-person' | 'attending-virtually' | 'not-attending' | 'undecided';
  guestCount?: number;
}

export interface AttendanceTracking {
  enabled: boolean;
  checkInMethod: 'manual' | 'qr-code' | 'automatic';
  reportGeneration: boolean;
}

export interface NotificationConfig {
  types: ('service-reminder' | 'stream-starting' | 'recording-available' | 'memorial-update')[];
  channels: ('email' | 'sms' | 'push')[];
}

// ============================================================================
// Condolence Types
// ============================================================================

export interface CondolenceConfig {
  enabled: boolean;
  moderation: ModerationConfig;
  types: CondolenceType[];
  donations?: DonationConfig;
}

export interface ModerationConfig {
  autoApprove: boolean;
  filterProfanity: boolean;
  reviewQueue: boolean;
  moderators: string[];
}

export type CondolenceType = 'message' | 'memory' | 'photo' | 'video' | 'virtual-flower' | 'virtual-candle';

export interface DonationConfig {
  enabled: boolean;
  charities: Charity[];
  displayOnMemorial: boolean;
  anonymous: boolean;
}

export interface Charity {
  id: string;
  name: string;
  url: string;
  ein?: string;
  description?: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface FuneralResponse {
  id: string;
  type: ServiceType;
  status: ServiceStatus;
  deceasedName: string;
  scheduledDate: string;
  streamUrl?: string;
  links: { self: string; memorial?: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
