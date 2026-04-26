/**
 * WIA-EDU-019 Digital Content Standard - TypeScript Type Definitions
 * Version: 2.0.0
 *
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export type ContentType = 'video' | 'audio' | 'image' | 'interactive' | 'document' | '3d';
export type LanguageCode = 'en' | 'ko' | 'es' | 'fr' | 'de' | 'zh' | 'ja' | 'ar';
export type WCAGLevel = 'A' | 'AA' | 'AAA';
export type LicenseType = 'CC0' | 'CC BY' | 'CC BY-SA' | 'CC BY-ND' | 'CC BY-NC' | 'CC BY-NC-SA' | 'proprietary';

// ============================================================================
// Content Interfaces
// ============================================================================

export interface Content {
  id: string;
  title: string;
  description?: string;
  type: ContentType;
  format: string;
  language: LanguageCode | LanguageCode[];
  duration?: number;
  fileSize?: number;
  url?: string;
  thumbnail?: string;
  metadata: ContentMetadata;
  accessibility: AccessibilityFeatures;
  license: License;
  createdAt: string;
  updatedAt: string;
}

export interface ContentMetadata {
  creator: string;
  subject: string[];
  publisher?: string;
  date?: string;
  wcagLevel: WCAGLevel;
  keywords?: string[];
  educationalLevel?: string;
  learningResourceType?: string;
  [key: string]: any;
}

export interface AccessibilityFeatures {
  captions?: LanguageCode[];
  audioDescription?: boolean;
  transcript?: boolean;
  signLanguage?: boolean;
  keyboardNavigation?: boolean;
  screenReaderCompatible?: boolean;
  highContrast?: boolean;
}

export interface License {
  type: LicenseType;
  url?: string;
  text?: string;
  requiresAttribution?: boolean;
  allowsCommercial?: boolean;
  allowsDerivatives?: boolean;
  requiresShareAlike?: boolean;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface ListContentRequest {
  limit?: number;
  offset?: number;
  type?: ContentType;
  language?: LanguageCode;
  search?: string;
  wcagLevel?: WCAGLevel;
}

export interface ListContentResponse {
  total: number;
  limit: number;
  offset: number;
  items: Content[];
}

export interface CreateContentRequest {
  title: string;
  description?: string;
  type: ContentType;
  language: LanguageCode | LanguageCode[];
  file?: string | Buffer;
  fileUrl?: string;
  metadata: ContentMetadata;
  accessibility: AccessibilityFeatures;
  license: License;
}

export interface UpdateContentRequest {
  title?: string;
  description?: string;
  metadata?: Partial<ContentMetadata>;
  accessibility?: Partial<AccessibilityFeatures>;
  license?: Partial<License>;
}

// ============================================================================
// Search Types
// ============================================================================

export interface SearchRequest {
  query: string;
  filters?: SearchFilters;
  sort?: SortOptions;
  limit?: number;
  offset?: number;
}

export interface SearchFilters {
  type?: ContentType[];
  language?: LanguageCode[];
  wcagLevel?: WCAGLevel[];
  license?: LicenseType[];
  certified?: boolean;
}

export interface SortOptions {
  field: 'relevance' | 'date' | 'title' | 'popularity';
  order: 'asc' | 'desc';
}

export interface SearchResult {
  id: string;
  title: string;
  description?: string;
  score: number;
  highlights?: {
    title?: string;
    description?: string;
  };
}

export interface SearchResponse {
  total: number;
  results: SearchResult[];
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface TrackingEvent {
  contentId: string;
  userId?: string;
  event: 'view' | 'play' | 'pause' | 'complete' | 'download';
  timestamp: string;
  metadata?: {
    duration?: number;
    completion?: number;
    device?: string;
    location?: string;
    [key: string]: any;
  };
}

export interface AnalyticsData {
  contentId: string;
  period: string;
  views: number;
  uniqueUsers: number;
  averageDuration: number;
  completionRate: number;
  demographics?: {
    countries?: Record<string, number>;
    devices?: Record<string, number>;
    languages?: Record<string, number>;
  };
}

// ============================================================================
// Package Types
// ============================================================================

export interface ContentPackage {
  manifest: PackageManifest;
  metadata: ContentMetadata;
  files: PackageFile[];
  signatures?: PackageSignature[];
}

export interface PackageManifest {
  wia: {
    standard: 'WIA-EDU-019';
    version: string;
    packageVersion: string;
  };
  content: {
    id: string;
    title: string;
    type: ContentType;
    language: LanguageCode[];
    duration?: number;
    size: number;
  };
  requirements?: {
    platform?: string[];
    minBrowser?: Record<string, string>;
  };
}

export interface PackageFile {
  path: string;
  type: string;
  role: 'main' | 'primary' | 'secondary' | 'accessibility';
  size: number;
  checksum?: string;
}

export interface PackageSignature {
  algorithm: 'SHA-256' | 'SHA-512';
  value: string;
  timestamp: string;
  signer?: string;
}

// ============================================================================
// Certification Types
// ============================================================================

export interface Certification {
  id: string;
  standard: 'WIA-EDU-019';
  level: 'Phase 1' | 'Phases 1-2' | 'Phases 1-3' | 'Full Compliance';
  issueDate: string;
  expirationDate: string;
  certifier: string;
  badgeUrl?: string;
  verificationUrl?: string;
}

export interface CertificationRequest {
  contentId: string;
  targetLevel: Certification['level'];
  applicant: {
    organization: string;
    contact: string;
  };
  documentation: {
    technicalSpecs?: string;
    accessibilityReport?: string;
    testResults?: string;
  };
}

// ============================================================================
// Sync Types
// ============================================================================

export interface SyncRequest {
  deviceId: string;
  lastSyncTimestamp: string;
  localContent: LocalContentInfo[];
}

export interface LocalContentInfo {
  id: string;
  version: string;
  checksum: string;
}

export interface SyncResponse {
  syncId: string;
  timestamp: string;
  updates: SyncUpdate[];
}

export interface SyncUpdate {
  id: string;
  version: string;
  action: 'update' | 'delete';
  url?: string;
  checksum?: string;
}

// ============================================================================
// Error Types
// ============================================================================

export interface APIError {
  code: string;
  message: string;
  details?: Array<{
    field?: string;
    message: string;
  }>;
}

// ============================================================================
// Client Configuration
// ============================================================================

export interface ClientConfig {
  baseURL: string;
  accessToken?: string;
  apiKey?: string;
  timeout?: number;
  retries?: number;
  enableLogging?: boolean;
}

// ============================================================================
// Webhook Types
// ============================================================================

export interface WebhookConfig {
  url: string;
  events: WebhookEvent[];
  secret?: string;
}

export type WebhookEvent =
  | 'content.created'
  | 'content.updated'
  | 'content.deleted'
  | 'content.certified'
  | 'analytics.milestone';

export interface WebhookPayload {
  event: WebhookEvent;
  timestamp: string;
  data: any;
  signature?: string;
}
