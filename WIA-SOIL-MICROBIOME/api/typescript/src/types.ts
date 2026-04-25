/**
 * WIA-SOIL-MICROBIOME TypeScript Type Definitions
 * @version 1.0.0
 * @license MIT
 * @description Type definitions for the WIA-SOIL-MICROBIOME Standard
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * Soil sample type
 */
export type SoilType =
  | 'agricultural'
  | 'forest'
  | 'grassland'
  | 'wetland'
  | 'urban'
  | 'desert'
  | 'tundra';

/**
 * Sampling depth category
 */
export type SamplingDepth =
  | 'topsoil-0-15cm'
  | 'subsoil-15-30cm'
  | 'deep-30-60cm'
  | 'very-deep-60cm+';

/**
 * Analysis methods for microbiome profiling
 */
export type AnalysisMethod =
  | '16s-rrna-sequencing'
  | 'shotgun-metagenomics'
  | 'metatranscriptomics'
  | 'amplicon-sequencing'
  | 'qpcr'
  | 'plate-culture';

/**
 * Microbial domain classification
 */
export type MicrobialDomain = 'bacteria' | 'archaea' | 'fungi' | 'virus' | 'protist';

/**
 * Functional group categories
 */
export type FunctionalGroup =
  | 'nitrogen-fixers'
  | 'nitrifiers'
  | 'denitrifiers'
  | 'decomposers'
  | 'mycorrhizal'
  | 'pathogens'
  | 'phosphate-solubilizers'
  | 'methanotrophs'
  | 'methanogens';

/**
 * Soil health status
 */
export type HealthStatus = 'excellent' | 'good' | 'fair' | 'poor' | 'degraded';

/**
 * Carbon cycling process type
 */
export type CarbonProcess =
  | 'decomposition'
  | 'respiration'
  | 'sequestration'
  | 'mineralization'
  | 'humification';

/**
 * Message types for real-time protocol
 */
export type MessageType =
  | 'auth.request'
  | 'auth.response'
  | 'sample.submit'
  | 'sample.ack'
  | 'analysis.request'
  | 'analysis.result'
  | 'subscription.add'
  | 'subscription.remove'
  | 'alert.notify'
  | 'heartbeat.ping'
  | 'heartbeat.pong'
  | 'error';

/**
 * Alert levels
 */
export type AlertLevel = 'info' | 'warning' | 'critical';

// ============================================================================
// Sample & Location Types
// ============================================================================

/**
 * Geographic location
 */
export interface GeoLocation {
  /** Latitude */
  latitude: number;
  /** Longitude */
  longitude: number;
  /** Elevation in meters */
  elevation?: number;
  /** Location description */
  description?: string;
}

/**
 * Soil properties
 */
export interface SoilProperties {
  /** pH level */
  pH?: number;
  /** Organic matter percentage */
  organicMatter?: number;
  /** Moisture content percentage */
  moisture?: number;
  /** Temperature in Celsius */
  temperature?: number;
  /** Electrical conductivity (dS/m) */
  conductivity?: number;
  /** Clay percentage */
  clay?: number;
  /** Silt percentage */
  silt?: number;
  /** Sand percentage */
  sand?: number;
  /** Bulk density (g/cm³) */
  bulkDensity?: number;
}

/**
 * Soil sample
 */
export interface SoilSample {
  /** Unique sample identifier */
  id: string;
  /** Sample collection date */
  collectionDate: string;
  /** Geographic location */
  location: GeoLocation;
  /** Soil type */
  soilType: SoilType;
  /** Sampling depth */
  depth: SamplingDepth;
  /** Soil properties */
  properties?: SoilProperties;
  /** Laboratory that performed analysis */
  laboratory?: Laboratory;
  /** Sample metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Laboratory information
 */
export interface Laboratory {
  /** Laboratory identifier */
  id: string;
  /** Laboratory name */
  name: string;
  /** Certification information */
  certification?: string;
  /** Laboratory location */
  location?: string;
}

// ============================================================================
// Microbiome Profile Types
// ============================================================================

/**
 * Taxonomic classification
 */
export interface TaxonomicUnit {
  /** Taxonomic identifier (e.g., NCBI taxid) */
  taxId?: string;
  /** Domain */
  domain?: MicrobialDomain;
  /** Phylum */
  phylum?: string;
  /** Class */
  class?: string;
  /** Order */
  order?: string;
  /** Family */
  family?: string;
  /** Genus */
  genus?: string;
  /** Species */
  species?: string;
  /** Relative abundance (0-100%) */
  abundance: number;
  /** Number of reads */
  readCount?: number;
}

/**
 * Diversity indices
 */
export interface DiversityIndex {
  /** Shannon diversity index */
  shannon?: number;
  /** Simpson diversity index */
  simpson?: number;
  /** Observed species richness */
  observedSpecies?: number;
  /** Chao1 richness estimator */
  chao1?: number;
  /** Pielou's evenness */
  evenness?: number;
  /** Faith's phylogenetic diversity */
  faithsPD?: number;
}

/**
 * Functional group profile
 */
export interface FunctionalGroupProfile {
  /** Functional group type */
  group: FunctionalGroup;
  /** Relative abundance percentage */
  abundance: number;
  /** Gene markers detected */
  geneMarkers?: string[];
  /** Activity level (0-1) */
  activityLevel?: number;
}

/**
 * Complete microbiome profile
 */
export interface MicrobiomeProfile {
  /** Profile identifier */
  id: string;
  /** Associated sample ID */
  sampleId: string;
  /** Analysis method used */
  analysisMethod: AnalysisMethod;
  /** Analysis date */
  analysisDate: string;
  /** Taxonomic composition */
  taxonomy: TaxonomicUnit[];
  /** Diversity metrics */
  diversity: DiversityIndex;
  /** Functional groups */
  functionalGroups?: FunctionalGroupProfile[];
  /** Total number of sequences */
  totalSequences?: number;
  /** Sequencing depth */
  sequencingDepth?: number;
  /** Quality score */
  qualityScore?: number;
}

// ============================================================================
// Soil Health & Carbon Types
// ============================================================================

/**
 * Carbon metrics
 */
export interface CarbonMetrics {
  /** Total organic carbon (g/kg) */
  totalOrganicCarbon?: number;
  /** Microbial biomass carbon (mg/kg) */
  microbialBiomassCarbon?: number;
  /** Dissolved organic carbon (mg/kg) */
  dissolvedOrganicCarbon?: number;
  /** Carbon to nitrogen ratio */
  carbonNitrogenRatio?: number;
  /** Soil respiration rate (mg CO2/kg/day) */
  respirationRate?: number;
  /** Carbon sequestration potential (tons/ha/year) */
  sequestrationPotential?: number;
}

/**
 * Nutrient cycling metrics
 */
export interface NutrientCycling {
  /** Nitrogen mineralization rate (mg/kg/day) */
  nitrogenMineralization?: number;
  /** Nitrification rate (mg/kg/day) */
  nitrification?: number;
  /** Denitrification rate (mg/kg/day) */
  denitrification?: number;
  /** Phosphorus availability (mg/kg) */
  phosphorusAvailability?: number;
  /** Enzyme activity levels */
  enzymeActivities?: Record<string, number>;
}

/**
 * Soil health index
 */
export interface SoilHealthIndex {
  /** Overall health score (0-100) */
  score: number;
  /** Health status classification */
  status: HealthStatus;
  /** Biological indicator score */
  biologicalScore?: number;
  /** Chemical indicator score */
  chemicalScore?: number;
  /** Physical indicator score */
  physicalScore?: number;
  /** Carbon metrics */
  carbonMetrics?: CarbonMetrics;
  /** Nutrient cycling metrics */
  nutrientCycling?: NutrientCycling;
  /** Assessment date */
  assessmentDate: string;
  /** Recommendations */
  recommendations?: string[];
}

// ============================================================================
// Intervention Types
// ============================================================================

/**
 * Intervention type
 */
export type InterventionType =
  | 'compost-amendment'
  | 'cover-cropping'
  | 'reduced-tillage'
  | 'crop-rotation'
  | 'biochar-application'
  | 'microbial-inoculant'
  | 'organic-fertilizer'
  | 'lime-application';

/**
 * Soil intervention record
 */
export interface Intervention {
  /** Intervention identifier */
  id: string;
  /** Type of intervention */
  type: InterventionType;
  /** Intervention name */
  name: string;
  /** Application rate */
  applicationRate?: string;
  /** Application date */
  applicationDate: string;
  /** Expected duration */
  duration?: string;
  /** Target outcomes */
  targetOutcomes?: string[];
  /** Notes */
  notes?: string;
  /** Active status */
  active: boolean;
}

// ============================================================================
// Complete Soil Report
// ============================================================================

/**
 * Metadata for soil reports
 */
export interface Metadata {
  /** Standard name */
  standard: string;
  /** Standard version */
  version: string;
  /** Philosophy */
  philosophy: string;
  /** Created timestamp */
  createdAt?: string;
  /** Updated timestamp */
  updatedAt?: string;
  /** Additional custom fields */
  [key: string]: unknown;
}

/**
 * Complete soil microbiome report
 */
export interface SoilMicrobiomeReport {
  /** JSON-LD context */
  '@context': string[];
  /** Report types */
  type: string[];
  /** Unique identifier (URN) */
  id: string;
  /** Issuer DID */
  issuer?: string;
  /** Issuance date */
  issuanceDate?: string;
  /** Soil sample */
  sample: SoilSample;
  /** Microbiome profile */
  microbiomeProfile: MicrobiomeProfile;
  /** Soil health index */
  healthIndex?: SoilHealthIndex;
  /** Interventions applied */
  interventions?: Intervention[];
  /** Carbon sequestration data */
  carbonSequestration?: CarbonSequestration;
  /** Metadata */
  metadata?: Metadata;
}

/**
 * Carbon sequestration tracking
 */
export interface CarbonSequestration {
  /** Measurement period start */
  periodStart: string;
  /** Measurement period end */
  periodEnd: string;
  /** Baseline carbon level (tons/ha) */
  baseline: number;
  /** Current carbon level (tons/ha) */
  current: number;
  /** Net change (tons/ha) */
  netChange: number;
  /** Annual sequestration rate (tons/ha/year) */
  annualRate: number;
  /** Confidence level (0-1) */
  confidence?: number;
  /** Measurement method */
  method?: string;
}

// ============================================================================
// API Types
// ============================================================================

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Number of items per page */
  limit?: number;
  /** Offset for pagination */
  offset?: number;
}

/**
 * Pagination response
 */
export interface PaginationResponse {
  /** Total number of items */
  total: number;
  /** Current limit */
  limit: number;
  /** Current offset */
  offset: number;
  /** Whether there are more items */
  hasMore: boolean;
}

/**
 * Paginated response wrapper
 */
export interface PaginatedResponse<T> {
  /** Data items */
  data: T[];
  /** Pagination info */
  pagination: PaginationResponse;
}

/**
 * API error details
 */
export interface ApiErrorDetails {
  /** Field that caused the error */
  field?: string;
  /** Invalid value */
  value?: unknown;
  /** Constraint that was violated */
  constraint?: string;
}

/**
 * API error response
 */
export interface ApiError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Error details */
  details?: ApiErrorDetails;
  /** Request ID for tracking */
  requestId: string;
}

/**
 * Sample submission request
 */
export interface SubmitSampleRequest {
  /** Sample information */
  sample: Omit<SoilSample, 'id'>;
  /** Requested analysis methods */
  analysisMethods: AnalysisMethod[];
}

/**
 * Sample summary response
 */
export interface SampleSummary {
  /** Sample identifier */
  id: string;
  /** Collection date */
  collectionDate: string;
  /** Soil type */
  soilType: SoilType;
  /** Analysis status */
  status: 'pending' | 'processing' | 'completed' | 'failed';
  /** Created timestamp */
  createdAt: string;
  /** Last updated timestamp */
  updatedAt: string;
}

/**
 * Analysis request
 */
export interface AnalysisRequest {
  /** Sample ID to analyze */
  sampleId: string;
  /** Analysis method */
  method: AnalysisMethod;
  /** Analysis parameters */
  parameters?: Record<string, unknown>;
}

/**
 * Health index calculation request
 */
export interface CalculateHealthIndexRequest {
  /** Profile ID */
  profileId: string;
  /** Include carbon metrics */
  includeCarbon?: boolean;
  /** Include nutrient cycling */
  includeNutrients?: boolean;
}

// ============================================================================
// Protocol Types
// ============================================================================

/**
 * Message header
 */
export interface MessageHeader {
  /** Protocol version */
  version: string;
  /** Unique message identifier */
  messageId: string;
  /** Message type */
  type: MessageType;
  /** Timestamp */
  timestamp: string;
  /** Message source */
  source?: {
    id: string;
    type: string;
  };
  /** Message destination */
  destination?: {
    id: string;
    type: string;
  };
}

/**
 * Security information for messages
 */
export interface MessageSecurity {
  /** Signature */
  signature?: string;
  /** Encryption method */
  encryption?: 'none' | 'AES-256-GCM';
  /** Key identifier */
  keyId?: string;
  /** Initialization vector */
  iv?: string;
}

/**
 * Protocol message
 */
export interface ProtocolMessage<T = unknown> {
  /** Message header */
  header: MessageHeader;
  /** Message payload */
  payload: T;
  /** Security information */
  security?: MessageSecurity;
}

/**
 * Alert notification
 */
export interface Alert {
  /** Alert level */
  level: AlertLevel;
  /** Alert message */
  message: string;
  /** Related sample ID */
  sampleId?: string;
  /** Alert timestamp */
  timestamp: string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * SDK configuration options
 */
export interface WiaSoilMicrobiomeConfig {
  /** API key or access token */
  apiKey: string;
  /** Environment (production or sandbox) */
  environment?: 'production' | 'sandbox';
  /** Base URL override */
  baseUrl?: string;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Retry configuration */
  retry?: {
    /** Maximum number of retries */
    maxRetries: number;
    /** Base delay between retries */
    baseDelay: number;
  };
}

/**
 * Webhook event types
 */
export type WebhookEventType =
  | 'sample.completed'
  | 'analysis.completed'
  | 'health.alert'
  | 'carbon.milestone';

/**
 * Webhook payload
 */
export interface WebhookPayload<T = unknown> {
  /** Event identifier */
  id: string;
  /** Event type */
  type: WebhookEventType;
  /** Event timestamp */
  timestamp: string;
  /** Event data */
  data: T;
  /** Signature for verification */
  signature: string;
}

// ============================================================================
// Exports
// ============================================================================

export default {
  // Re-export all types for convenience
};
