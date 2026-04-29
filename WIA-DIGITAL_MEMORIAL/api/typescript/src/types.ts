/**
 * WIA Digital Memorial Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADigitalMemorial {
  standard: 'WIA-DIGITAL-MEMORIAL';
  version: string;
  memorial: MemorialProfile;
  subject: SubjectInfo;
  content: MemorialContent;
  access: AccessConfiguration;
  interactions: InteractionConfig;
  preservation: PreservationConfig;
  extensions?: Record<string, unknown>;
}

export interface MemorialProfile {
  id: string;
  type: MemorialType;
  status: MemorialStatus;
  visibility: Visibility;
  createdAt: string;
  updatedAt?: string;
  expiresAt?: string;
  curator: CuratorInfo;
  customDomain?: string;
}

export type MemorialType = 'personal' | 'public-figure' | 'veteran' | 'group' | 'pet' | 'historical';
export type MemorialStatus = 'draft' | 'published' | 'archived' | 'suspended';
export type Visibility = 'public' | 'unlisted' | 'private' | 'family-only';

export interface CuratorInfo {
  id: string;
  name: string;
  email: string;
  relationship: string;
  verified: boolean;
}

export interface SubjectInfo {
  id: string;
  fullName: string;
  displayName?: string;
  birthDate?: string;
  deathDate?: string;
  birthPlace?: string;
  restingPlace?: string;
  epitaph?: string;
  biography?: string;
  profilePhoto?: MediaAsset;
}

// ============================================================================
// Content Types
// ============================================================================

export interface MemorialContent {
  timeline: TimelineEntry[];
  gallery: MediaGallery;
  stories: Story[];
  tributes: TributeMessage[];
  milestones: Milestone[];
  relationships: Relationship[];
  legacy: LegacyItem[];
}

export interface TimelineEntry {
  id: string;
  date: string;
  title: string;
  description?: string;
  type: TimelineType;
  media?: MediaAsset[];
  location?: Location;
  tags?: string[];
}

export type TimelineType = 'birth' | 'education' | 'career' | 'marriage' | 'achievement' | 'travel' | 'family' | 'death' | 'custom';

export interface MediaAsset {
  id: string;
  type: 'photo' | 'video' | 'audio' | 'document' | '3d-model';
  url: string;
  thumbnailUrl?: string;
  caption?: string;
  date?: string;
  uploadedBy: string;
  uploadedAt: string;
  metadata?: MediaMetadata;
}

export interface MediaMetadata {
  width?: number;
  height?: number;
  duration?: number;
  format?: string;
  size?: number;
  location?: Location;
}

export interface MediaGallery {
  albums: Album[];
  totalItems: number;
  storageUsed: number;
}

export interface Album {
  id: string;
  name: string;
  description?: string;
  coverPhoto?: string;
  items: MediaAsset[];
  createdAt: string;
}

export interface Story {
  id: string;
  title: string;
  content: string;
  author: string;
  authorRelationship?: string;
  publishedAt: string;
  featured: boolean;
  reactions: Reaction[];
}

export interface Reaction {
  type: 'love' | 'candle' | 'flower' | 'memory' | 'prayer';
  count: number;
}

export interface TributeMessage {
  id: string;
  author: string;
  authorEmail?: string;
  content: string;
  type: 'condolence' | 'memory' | 'gratitude' | 'poem';
  createdAt: string;
  approved: boolean;
  pinned: boolean;
}

export interface Milestone {
  id: string;
  title: string;
  date: string;
  description?: string;
  significance: 'major' | 'minor';
}

export interface Relationship {
  id: string;
  personId: string;
  personName: string;
  relationshipType: string;
  profileLink?: string;
}

export interface LegacyItem {
  id: string;
  type: 'quote' | 'achievement' | 'contribution' | 'value' | 'lesson';
  title: string;
  content: string;
  source?: string;
}

export interface Location {
  name: string;
  address?: string;
  latitude?: number;
  longitude?: number;
  country?: string;
}

// ============================================================================
// Access & Interaction Types
// ============================================================================

export interface AccessConfiguration {
  password?: string;
  invitationOnly: boolean;
  allowedEmails?: string[];
  familyCode?: string;
  geoRestrictions?: string[];
  ageRestriction?: number;
}

export interface InteractionConfig {
  allowTributes: boolean;
  allowStories: boolean;
  allowPhotoUploads: boolean;
  allowVideoUploads: boolean;
  virtualCandles: boolean;
  virtualFlowers: boolean;
  guestbook: boolean;
  donations: DonationConfig;
  anniversaryReminders: boolean;
}

export interface DonationConfig {
  enabled: boolean;
  charities: Charity[];
  goal?: number;
  raised?: number;
}

export interface Charity {
  id: string;
  name: string;
  url: string;
  description?: string;
}

// ============================================================================
// Preservation Types
// ============================================================================

export interface PreservationConfig {
  duration: PreservationDuration;
  backup: BackupConfig;
  succession: SuccessionPlan;
  archiveFormat?: string;
}

export type PreservationDuration = 'indefinite' | '10-years' | '50-years' | '100-years' | 'renewable';

export interface BackupConfig {
  frequency: 'daily' | 'weekly' | 'monthly';
  locations: string[];
  encryption: boolean;
}

export interface SuccessionPlan {
  successors: Successor[];
  inactivityPeriod: number;
  autoArchive: boolean;
}

export interface Successor {
  name: string;
  email: string;
  priority: number;
  verified: boolean;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface MemorialResponse {
  id: string;
  subjectName: string;
  type: MemorialType;
  status: MemorialStatus;
  url: string;
  createdAt: string;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
