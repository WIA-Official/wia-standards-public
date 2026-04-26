/**
 * WIA-UNI-011 Cultural Exchange Data Standard
 * TypeScript Type Definitions v2.0
 */

export type EventType = 'arts' | 'sports' | 'music' | 'festival' | 'heritage' | 'education';
export type UNESCOCategory = 'intangible' | 'tangible' | 'contemporary' | 'traditional';
export type Region = 'south' | 'north' | 'both' | 'international';
export type PreservationStatus = 'excellent' | 'good' | 'fair' | 'endangered' | 'critical';
export type ParticipantRole = 'artist' | 'athlete' | 'organizer' | 'audience' | 'expert' | 'official';

/** Multilingual text object */
export interface I18nString {
  ko: string;
  en: string;
  'ko-north'?: string;
  hanja?: string;
  [key: string]: string | undefined;
}

/** Geographic coordinates */
export interface Coordinates {
  lat: number;
  lng: number;
}

/** Location information */
export interface Location {
  name: I18nString;
  address?: string;
  coordinates?: Coordinates;
}

/** Organization reference */
export interface Organization {
  id: string;
  name: I18nString;
  type?: string;
  region?: Region;
}

/** Cultural Event */
export interface CulturalEvent {
  standard: 'WIA-UNI-011';
  version: string;
  eventId: string;
  eventType: EventType;
  title: I18nString;
  description: I18nString;
  startDate: string; // ISO 8601
  endDate: string; // ISO 8601
  location: Location;
  organizers: Organization[];
  participants?: Participant[];
  media?: MediaAsset[];
  unescoCategory?: UNESCOCategory;
  expectedParticipants?: number;
  tags?: string[];
  metadata?: {
    created: string;
    modified?: string;
    language: string[];
    region: Region;
  };
}

/** Participant/Artist Profile */
export interface Participant {
  participantId: string;
  name: I18nString;
  role: ParticipantRole;
  organization?: Organization;
  region: Region;
  specialty?: string[];
  biography?: I18nString;
  consentLevel?: 'public' | 'controlled' | 'private';
}

/** Heritage Item */
export interface HeritageItem {
  heritageId: string;
  name: I18nString;
  category: string;
  unescoStatus?: 'registered' | 'nominated' | 'candidate' | 'local';
  description: I18nString;
  historicalContext?: I18nString;
  regionalVariations?: RegionalVariation[];
  preservationStatus: PreservationStatus;
  media?: MediaAsset[];
}

/** Regional variation of heritage item */
export interface RegionalVariation {
  region: Region;
  localName: string;
  characteristics: string;
  preservationStatus: PreservationStatus;
  authority?: string;
  lastUpdated?: string;
}

/** Media Asset */
export interface MediaAsset {
  assetId: string;
  type: 'photo' | 'video' | 'audio' | 'document' | '3d';
  title: I18nString;
  description?: I18nString;
  creator: string;
  created: string; // ISO 8601
  license: License;
  file: FileInfo;
  relatedEvent?: string;
  tags?: string[];
}

/** File information */
export interface FileInfo {
  url: string;
  format: string;
  size: number;
  checksum: string;
  width?: number;
  height?: number;
  duration?: number;
}

/** License information */
export interface License {
  type: 'CC0' | 'CC-BY' | 'CC-BY-SA' | 'CC-BY-NC' | 'CC-BY-ND' | 'rights-reserved' | 'public-domain';
  url?: string;
  restrictions?: string[];
}

/** API Response wrapper */
export interface APIResponse<T> {
  data: T;
  pagination?: {
    total: number;
    page: number;
    limit: number;
    pages: number;
  };
  meta?: Record<string, any>;
}

/** API Error response */
export interface APIError {
  error: {
    code: string;
    message: string;
    details?: Record<string, any>;
    timestamp: string;
    requestId: string;
  };
}

/** Event filter options */
export interface EventFilter {
  type?: EventType;
  startDate?: string;
  endDate?: string;
  region?: Region;
  organization?: string;
  tags?: string[];
  limit?: number;
  offset?: number;
}

/** Heritage search options */
export interface HeritageSearchOptions {
  query?: string;
  category?: string;
  region?: Region;
  unescoStatus?: string;
  limit?: number;
  offset?: number;
}

/** Client configuration */
export interface ClientConfig {
  apiKey: string;
  baseURL?: string;
  timeout?: number;
  headers?: Record<string, string>;
}
