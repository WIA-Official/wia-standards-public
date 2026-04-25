/**
 * WIA-CORE-008: Universal Metadata - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Metadata Types
// ============================================================================

/**
 * Metadata domain types
 */
export type MetadataDomain =
  | 'general'
  | 'healthcare'
  | 'finance'
  | 'science'
  | 'iot'
  | 'media'
  | 'government'
  | 'education'
  | 'geospatial'
  | 'biomedical'
  | 'manufacturing'
  | 'transportation'
  | 'energy'
  | 'agriculture'
  | 'custom';

/**
 * Data type categories
 */
export type DataType =
  | 'dataset'
  | 'document'
  | 'image'
  | 'video'
  | 'audio'
  | 'model'
  | 'software'
  | 'service'
  | 'sensor-data'
  | 'event'
  | 'transaction'
  | 'record'
  | 'collection';

/**
 * Privacy/access classification
 */
export type PrivacyLevel =
  | 'public'
  | 'internal'
  | 'confidential'
  | 'restricted'
  | 'secret'
  | 'top-secret';

/**
 * Quality validation status
 */
export type ValidationStatus =
  | 'valid'
  | 'invalid'
  | 'warning'
  | 'incomplete'
  | 'not-validated';

/**
 * Agent type (creator, contributor)
 */
export interface Agent {
  /** Agent name */
  name: string;

  /** Agent identifier (ORCID, email, URI) */
  id?: string;

  /** Agent type */
  type?: 'person' | 'organization' | 'software';

  /** Contact email */
  email?: string;

  /** Affiliation */
  affiliation?: string;
}

/**
 * Temporal coverage/range
 */
export interface TemporalCoverage {
  /** Start date/time */
  start?: string;

  /** End date/time */
  end?: string;

  /** Duration (ISO 8601) */
  duration?: string;

  /** Time zone */
  timezone?: string;
}

/**
 * Spatial/geographic information
 */
export interface SpatialCoverage {
  /** Location name */
  location?: string;

  /** Coordinates (lat, lon) */
  coordinates?: {
    lat: number;
    lon: number;
  };

  /** Bounding box */
  boundingBox?: {
    north: number;
    south: number;
    east: number;
    west: number;
  };

  /** Elevation (meters) */
  elevation?: number;

  /** Coordinate reference system */
  crs?: string;
}

/**
 * License information
 */
export interface License {
  /** License name */
  name: string;

  /** License URL */
  url?: string;

  /** License identifier (SPDX) */
  spdx?: string;

  /** Custom terms */
  terms?: string;
}

/**
 * Version information
 */
export interface Version {
  /** Version number */
  number: string;

  /** Release date */
  date: string;

  /** Change description */
  changes?: string;

  /** Previous version */
  previousVersion?: string;
}

// ============================================================================
// Universal Metadata Schema
// ============================================================================

/**
 * Core universal metadata
 */
export interface UniversalMetadata {
  // Identification
  /** Unique identifier */
  id: string;

  /** Primary title */
  title: string;

  /** Alternate titles */
  alternateTitle?: string[];

  /** External identifiers (DOI, ISBN, etc.) */
  identifiers?: Record<string, string>;

  // Description
  /** Brief description */
  description?: string;

  /** Detailed abstract */
  abstract?: string;

  /** Keywords */
  keywords?: string[];

  /** Classification tags */
  tags?: string[];

  /** Subject categories */
  subjects?: string[];

  // Provenance
  /** Creator(s) */
  creator?: Agent | Agent[];

  /** Contributors */
  contributors?: Agent[];

  /** Publisher */
  publisher?: Agent;

  /** Data source */
  source?: string;

  // Temporal
  /** Creation timestamp */
  created?: string;

  /** Last modified timestamp */
  modified?: string;

  /** Publication date */
  published?: string;

  /** Temporal coverage */
  temporal?: TemporalCoverage;

  // Spatial
  /** Spatial coverage */
  spatial?: SpatialCoverage;

  // Rights
  /** License */
  license?: License | string;

  /** Copyright statement */
  copyright?: string;

  /** Access rights */
  accessRights?: string;

  /** Privacy classification */
  privacy?: PrivacyLevel;

  // Technical
  /** Data type */
  dataType?: DataType;

  /** Format/MIME type */
  format?: string;

  /** Size in bytes */
  size?: number;

  /** Version */
  version?: Version | string;

  /** Checksum (SHA-256) */
  checksum?: string;

  /** Language (ISO 639) */
  language?: string;

  // Domain-specific
  /** Metadata domain */
  domain?: MetadataDomain;

  /** Domain-specific fields */
  domainMetadata?: Record<string, any>;

  // Relationships
  /** Related metadata IDs */
  relatedTo?: string[];

  /** Parent metadata ID */
  partOf?: string;

  /** Derived from metadata ID */
  derivedFrom?: string;

  /** References */
  references?: string[];

  // Quality
  /** Quality score (0-100) */
  qualityScore?: number;

  /** Validation status */
  validationStatus?: ValidationStatus;

  /** Completeness (0-100) */
  completeness?: number;

  // Extensibility
  /** Custom fields */
  custom?: Record<string, any>;

  // System
  /** Schema version */
  schemaVersion?: string;

  /** Last validated */
  lastValidated?: string;
}

// ============================================================================
// Domain-Specific Metadata
// ============================================================================

/**
 * Healthcare domain metadata
 */
export interface HealthcareMetadata {
  /** Patient identifier */
  patientId?: string;

  /** Medical record number */
  mrn?: string;

  /** Diagnosis codes (ICD-10) */
  diagnosisCodes?: string[];

  /** Procedure codes (CPT) */
  procedureCodes?: string[];

  /** Healthcare facility */
  facility?: string;

  /** Treating physician */
  physician?: Agent;

  /** PHI classification */
  phi?: boolean;

  /** HIPAA compliance */
  hipaaCompliant?: boolean;
}

/**
 * Finance domain metadata
 */
export interface FinanceMetadata {
  /** Transaction ID */
  transactionId?: string;

  /** Account number */
  accountNumber?: string;

  /** Amount */
  amount?: number;

  /** Currency (ISO 4217) */
  currency?: string;

  /** Transaction type */
  transactionType?: string;

  /** Financial institution */
  institution?: string;

  /** Regulatory compliance */
  compliance?: string[];
}

/**
 * Science domain metadata
 */
export interface ScienceMetadata {
  /** Experiment ID */
  experimentId?: string;

  /** Research method */
  method?: string;

  /** Hypothesis */
  hypothesis?: string;

  /** Variables */
  variables?: string[];

  /** Sample size */
  sampleSize?: number;

  /** Instruments/equipment */
  instruments?: string[];

  /** Funding source */
  funding?: string;

  /** Publication DOI */
  publicationDoi?: string;
}

/**
 * IoT domain metadata
 */
export interface IoTMetadata {
  /** Device ID */
  deviceId?: string;

  /** Device type */
  deviceType?: string;

  /** Sensor type */
  sensorType?: string;

  /** Measurement unit */
  unit?: string;

  /** Sampling rate */
  samplingRate?: number;

  /** Calibration date */
  calibrationDate?: string;

  /** Device location */
  deviceLocation?: SpatialCoverage;

  /** Network ID */
  networkId?: string;
}

/**
 * Media domain metadata
 */
export interface MediaMetadata {
  /** Media type */
  mediaType?: 'image' | 'video' | 'audio' | 'document';

  /** MIME type */
  mimeType?: string;

  /** Dimensions (width x height) */
  dimensions?: { width: number; height: number };

  /** Duration (seconds) */
  duration?: number;

  /** Codec */
  codec?: string;

  /** Bitrate */
  bitrate?: number;

  /** Color space */
  colorSpace?: string;

  /** Camera/capture device */
  captureDevice?: string;

  /** Photographer/creator */
  photographer?: Agent;
}

// ============================================================================
// Validation and Quality
// ============================================================================

/**
 * Validation result
 */
export interface ValidationResult {
  /** Is valid */
  valid: boolean;

  /** Quality score (0-100) */
  qualityScore: number;

  /** Completeness score (0-100) */
  completeness: number;

  /** Discoverability score */
  discoverabilityScore: number;

  /** Validation errors */
  errors: ValidationError[];

  /** Warnings */
  warnings: string[];

  /** Suggestions */
  suggestions: string[];

  /** Field completeness */
  fieldCompleteness: Record<string, boolean>;
}

/**
 * Validation error
 */
export interface ValidationError {
  /** Field name */
  field: string;

  /** Error code */
  code: string;

  /** Error message */
  message: string;

  /** Severity */
  severity: 'error' | 'warning' | 'info';
}

/**
 * Quality metrics
 */
export interface QualityMetrics {
  /** Overall quality score (0-100) */
  overallScore: number;

  /** Completeness (0-100) */
  completeness: number;

  /** Accuracy (0-100) */
  accuracy: number;

  /** Consistency (0-100) */
  consistency: number;

  /** Timeliness (0-100) */
  timeliness: number;

  /** Discoverability */
  discoverability: number;

  /** Missing required fields */
  missingFields: string[];

  /** Invalid fields */
  invalidFields: string[];
}

// ============================================================================
// Search and Discovery
// ============================================================================

/**
 * Search query
 */
export interface MetadataQuery {
  /** Search text */
  query?: string;

  /** Domain filter */
  domain?: MetadataDomain;

  /** Data type filter */
  dataType?: DataType;

  /** Creator filter */
  creator?: string;

  /** Keyword filter */
  keywords?: string[];

  /** Date range */
  dateRange?: {
    start: string;
    end: string;
  };

  /** Privacy filter */
  privacy?: PrivacyLevel[];

  /** Custom filters */
  filters?: Record<string, any>;

  /** Sort field */
  sortBy?: string;

  /** Sort order */
  sortOrder?: 'asc' | 'desc';

  /** Results limit */
  limit?: number;

  /** Results offset */
  offset?: number;
}

/**
 * Search result
 */
export interface MetadataSearchResult {
  /** Total results */
  total: number;

  /** Results */
  results: UniversalMetadata[];

  /** Facets */
  facets?: Record<string, Record<string, number>>;

  /** Query time (ms) */
  queryTime: number;
}

// ============================================================================
// Enrichment
// ============================================================================

/**
 * Enrichment options
 */
export interface EnrichmentOptions {
  /** Auto-classify data */
  autoClassify?: boolean;

  /** Extract keywords */
  extractKeywords?: boolean;

  /** Generate tags */
  generateTags?: boolean;

  /** Enhance description */
  enhanceDescription?: boolean;

  /** Link related metadata */
  linkRelated?: boolean;

  /** Calculate quality */
  calculateQuality?: boolean;
}

/**
 * Enrichment result
 */
export interface EnrichmentResult {
  /** Enriched metadata */
  metadata: UniversalMetadata;

  /** Added fields */
  addedFields: string[];

  /** Enhanced fields */
  enhancedFields: string[];

  /** Quality improvement */
  qualityImprovement: number;

  /** Confidence score (0-1) */
  confidence: number;
}

// ============================================================================
// Templates
// ============================================================================

/**
 * Metadata template
 */
export interface MetadataTemplate {
  /** Template ID */
  id: string;

  /** Template name */
  name: string;

  /** Description */
  description: string;

  /** Domain */
  domain: MetadataDomain;

  /** Required fields */
  requiredFields: string[];

  /** Optional fields */
  optionalFields: string[];

  /** Field defaults */
  defaults?: Record<string, any>;

  /** Field constraints */
  constraints?: Record<string, any>;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Universal metadata constants
 */
export const METADATA_CONSTANTS = {
  /** Schema version */
  SCHEMA_VERSION: '1.0.0',

  /** Required core fields */
  REQUIRED_FIELDS: ['id', 'title'],

  /** Recommended fields */
  RECOMMENDED_FIELDS: ['description', 'creator', 'created', 'domain', 'dataType'],

  /** Quality thresholds */
  QUALITY_THRESHOLDS: {
    EXCELLENT: 90,
    GOOD: 75,
    ACCEPTABLE: 60,
    POOR: 40,
  },

  /** Discoverability factors */
  DISCOVERABILITY_WEIGHTS: {
    keywords: 0.3,
    tags: 0.2,
    description: 0.25,
    subjects: 0.15,
    abstract: 0.1,
  },

  /** Default privacy levels */
  DEFAULT_PRIVACY: 'internal' as PrivacyLevel,

  /** Max field lengths */
  MAX_LENGTHS: {
    title: 500,
    description: 5000,
    abstract: 10000,
    keyword: 100,
  },

  /** Supported date formats */
  DATE_FORMATS: [
    'ISO8601',
    'YYYY-MM-DD',
    'YYYY-MM-DDTHH:mm:ssZ',
  ],
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Metadata error codes
 */
export enum MetadataErrorCode {
  INVALID_SCHEMA = 'M001',
  MISSING_REQUIRED_FIELD = 'M002',
  INVALID_FIELD_VALUE = 'M003',
  INVALID_FORMAT = 'M004',
  CHECKSUM_MISMATCH = 'M005',
  DOMAIN_MISMATCH = 'M006',
  PRIVACY_VIOLATION = 'M007',
  VERSION_CONFLICT = 'M008',
  RELATIONSHIP_BROKEN = 'M009',
  VALIDATION_FAILED = 'M010',
}

/**
 * Metadata error
 */
export class MetadataError extends Error {
  constructor(
    public code: MetadataErrorCode,
    message: string,
    public field?: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MetadataError';
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
 * Partial metadata update
 */
export type MetadataUpdate = Partial<UniversalMetadata> & { id: string };

/**
 * Metadata with domain-specific fields
 */
export type DomainMetadata<T = Record<string, any>> = UniversalMetadata & {
  domainMetadata: T;
};

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  MetadataDomain,
  DataType,
  PrivacyLevel,
  ValidationStatus,
  Agent,
  TemporalCoverage,
  SpatialCoverage,
  License,
  Version,

  // Main metadata
  UniversalMetadata,

  // Domain-specific
  HealthcareMetadata,
  FinanceMetadata,
  ScienceMetadata,
  IoTMetadata,
  MediaMetadata,

  // Validation
  ValidationResult,
  ValidationError,
  QualityMetrics,

  // Search
  MetadataQuery,
  MetadataSearchResult,

  // Enrichment
  EnrichmentOptions,
  EnrichmentResult,

  // Templates
  MetadataTemplate,

  // Updates
  MetadataUpdate,
  DomainMetadata,
};

export { METADATA_CONSTANTS, MetadataErrorCode, MetadataError };
