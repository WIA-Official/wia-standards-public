/**
 * WIA-LEG-007 Social Media Legacy Standard v2.0
 * TypeScript Type Definitions
 *
 * Philosophy: 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

/**
 * Legacy Contact Permission Types
 */
export type LegacyPermission =
  | 'view'
  | 'download'
  | 'manage_tribute'
  | 'post_memorial'
  | 'manage_settings'
  | 'delete_account'
  | 'manage_aibot'
  | 'access_messages';

/**
 * Memorialization Action Types
 */
export type MemorializationAction = 'memorialize' | 'delete' | 'archive';

/**
 * Export Format Types
 */
export type ExportFormat = 'json' | 'csv' | 'xml';

/**
 * Visibility Levels
 */
export type Visibility = 'public' | 'friends' | 'family' | 'custom' | 'private';

/**
 * Legacy Contact Interface
 */
export interface LegacyContact {
  contactId: string;
  name: string;
  email: string;
  phone?: string; // E.164 format
  permissions: LegacyPermission[];
  priority: 1 | 2 | 3;
  verifiedAt?: string; // ISO 8601 timestamp
}

/**
 * Tribute Page Configuration
 */
export interface TributePageConfig {
  visibility: Visibility;
  allowTributes: boolean;
  moderateTributes: boolean;
  featuredPhoto?: string;
  coverPhoto?: string;
  memorialStatement?: string;
  charitableGiving?: {
    enabled: boolean;
    organization: string;
    donationUrl: string;
  };
  anniversaryNotifications?: boolean;
}

/**
 * Data Export Configuration
 */
export interface DataExportConfig {
  enabled: boolean;
  formats: ExportFormat[];
  includeMessages: boolean;
  includeMetadata: boolean;
  deliverTo: 'legacyContacts' | 'email' | 'cloudStorage';
  cloudStorageProvider?: string;
  scheduledExports?: {
    frequency: 'monthly' | 'quarterly' | 'annually';
    enabled: boolean;
  };
}

/**
 * AI Memorial Bot Configuration
 */
export interface AIBotConfig {
  enabled: boolean;
  trainingSources: ('posts' | 'comments' | 'messages' | 'photos' | 'videos')[];
  model: 'gpt4' | 'claude' | 'palm' | 'custom';
  accessControl: Visibility;
  sunsetDate?: string; // ISO 8601 date
  personalityParameters?: {
    formality: number; // 0-1
    humor: number; // 0-1
    emotionalTone: number; // 0-1
  };
}

/**
 * Account Deletion Trigger
 */
export interface DeletionTrigger {
  type: 'time_based' | 'contact_based' | 'immediate' | 'never';
  years?: number; // For time_based
  condition?: string; // For contact_based
}

/**
 * Legacy Preferences
 */
export interface LegacyPreferences {
  action: MemorializationAction;
  coolingOffPeriod: number; // days: 30-90
  tributePage?: TributePageConfig;
  dataExport: DataExportConfig;
  aiBot?: AIBotConfig;
  deletionTrigger: DeletionTrigger;
}

/**
 * Main Legacy Configuration
 */
export interface LegacyConfiguration {
  userId: string;
  version: '2.0';
  lastUpdated: string; // ISO 8601 timestamp
  legacyContacts: LegacyContact[];
  preferences: LegacyPreferences;
}

/**
 * Death Verification Request
 */
export interface DeathVerificationRequest {
  userId: string;
  requestorId: string;
  deathCertificate: File | Blob;
  additionalDocuments?: (File | Blob)[];
  relationship: 'spouse' | 'child' | 'parent' | 'sibling' | 'executor' | 'other';
  dateOfDeath: string; // ISO 8601 date
}

/**
 * Verification Status
 */
export type VerificationStatus =
  | 'pending'
  | 'in_review'
  | 'cooling_off'
  | 'approved'
  | 'rejected';

/**
 * Death Verification Response
 */
export interface DeathVerificationResponse {
  verificationId: string;
  status: VerificationStatus;
  submittedAt: string;
  estimatedCompletionDate: string;
  coolingOffEnds?: string;
  message: string;
}

/**
 * Data Export Request
 */
export interface DataExportRequest {
  userId: string;
  format: ExportFormat;
  scope: 'all' | 'dateRange' | 'contentType';
  dateRange?: {
    start: string; // ISO 8601 date
    end: string; // ISO 8601 date
  };
  contentTypes?: string[];
}

/**
 * Data Export Response
 */
export interface DataExportResponse {
  exportId: string;
  status: 'queued' | 'processing' | 'ready' | 'expired';
  estimatedCompletionTime?: string;
  downloadUrl?: string;
  expiresAt?: string;
  totalSize?: number; // bytes
  checksum?: string; // SHA-256
}

/**
 * Memorial Tribute
 */
export interface MemorialTribute {
  memorialId: string;
  content: string;
  mediaIds?: string[];
  visibility: Visibility;
  author?: {
    id: string;
    name: string;
  };
  timestamp?: string;
}

/**
 * Tribute Response
 */
export interface TributeResponse {
  tributeId: string;
  status: 'posted' | 'pending_moderation' | 'rejected';
  postedAt?: string;
  moderationMessage?: string;
}

/**
 * AI Bot Query
 */
export interface AIBotQuery {
  memorialId: string;
  query: string;
  context?: string;
  maxLength?: number;
}

/**
 * AI Bot Response
 */
export interface AIBotResponse {
  response: string;
  confidence: number; // 0-1
  sources?: string[]; // Reference IDs to original content
  timestamp: string;
  model: string;
}

/**
 * Export Metadata
 */
export interface ExportMetadata {
  version: '2.0';
  exportDate: string;
  userId: string;
  platform: string;
  dataScope: 'all' | 'partial';
  checksum: string;
  totalItems: number;
}

/**
 * API Error Response
 */
export interface APIError {
  error: {
    code: string;
    message: string;
    details?: Record<string, unknown>;
  };
  timestamp: string;
  requestId: string;
}

/**
 * API Success Response
 */
export interface APISuccess<T = unknown> {
  data: T;
  timestamp: string;
  requestId: string;
}

/**
 * Webhook Event Types
 */
export type WebhookEventType =
  | 'legacy.verification.started'
  | 'legacy.verification.completed'
  | 'legacy.export.ready'
  | 'legacy.memorial.activated'
  | 'legacy.aibot.trained'
  | 'legacy.tribute.posted';

/**
 * Webhook Payload
 */
export interface WebhookPayload {
  event: WebhookEventType;
  timestamp: string;
  data: Record<string, unknown>;
  signature: string; // HMAC-SHA256
}
