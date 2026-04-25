/**
 * WIA-BIO-021: Synthetic Biology Registry - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Part Types
// ============================================================================

/**
 * Part type classification
 */
export type PartType =
  | 'promoter'
  | 'rbs'
  | 'cds'
  | 'terminator'
  | 'plasmid'
  | 'composite'
  | 'regulatory'
  | 'primer'
  | 'scar'
  | 'tag'
  | 'other';

/**
 * Part subtype for finer classification
 */
export type PartSubtype =
  | 'constitutive'
  | 'inducible'
  | 'repressible'
  | 'reporter'
  | 'selection'
  | 'origin'
  | 'enzyme'
  | 'structural'
  | 'device';

/**
 * Part status in registry lifecycle
 */
export type PartStatus =
  | 'planning'
  | 'ordered'
  | 'available'
  | 'characterized'
  | 'validated'
  | 'deprecated'
  | 'deleted';

/**
 * Biosafety level classification
 */
export type SafetyLevel = 'BSL-1' | 'BSL-2' | 'BSL-3' | 'BSL-4';

/**
 * Sequence format types
 */
export type SequenceFormat = 'raw' | 'fasta' | 'genbank' | 'sbol' | 'snapgene';

/**
 * Sequence type
 */
export type SequenceType = 'dna' | 'rna' | 'protein';

// ============================================================================
// Biological Part
// ============================================================================

/**
 * Core biological part definition
 */
export interface BiologicalPart {
  /** Unique part identifier (e.g., BBa_K123456) */
  partId: string;

  /** Human-readable part name */
  partName: string;

  /** Primary part type */
  type: PartType;

  /** Secondary classification */
  subtype?: PartSubtype;

  /** Current status */
  status: PartStatus;

  /** Biosafety classification */
  safetyLevel: SafetyLevel;

  /** Sequence information */
  sequence: SequenceData;

  /** Biological context */
  organism?: string;
  biologicalContext?: BiologicalContext;

  /** Author/creator information */
  author: AuthorInfo;

  /** Version information */
  version: string;

  /** License */
  license: string;

  /** Timestamps */
  created: Date | string;
  modified: Date | string;

  /** Description and documentation */
  description?: string;
  designNotes?: string;

  /** External references */
  references?: Reference[];
  externalLinks?: ExternalLink[];

  /** Characterization data */
  characterization?: CharacterizationData;

  /** Access control */
  accessControl?: AccessControl;

  /** Material transfer */
  materialTransfer?: MaterialTransfer;

  /** Deprecation info */
  deprecation?: DeprecationInfo;

  /** Reusability metrics */
  reusability?: ReusabilityMetrics;
}

/**
 * Sequence data and metadata
 */
export interface SequenceData {
  /** Nucleotide or amino acid sequence */
  nucleotides: string;

  /** Sequence length in bp/aa */
  length: number;

  /** Format of the sequence */
  format: SequenceFormat;

  /** Type of sequence */
  sequenceType: SequenceType;

  /** Checksum for integrity verification */
  checksum?: string;

  /** Annotated features */
  features?: SequenceFeature[];

  /** Circular or linear */
  topology?: 'circular' | 'linear';
}

/**
 * Annotated sequence feature
 */
export interface SequenceFeature {
  /** Feature type (promoter, CDS, etc.) */
  type: string;

  /** Start position (1-indexed) */
  start: number;

  /** End position (1-indexed) */
  end: number;

  /** Strand orientation */
  strand: '+' | '-';

  /** Feature name/label */
  name?: string;

  /** Qualifiers */
  qualifiers?: Record<string, string>;

  /** Color for visualization */
  color?: string;
}

/**
 * Biological context information
 */
export interface BiologicalContext {
  /** Expression host organism */
  expressionHost?: string;

  /** Strain information */
  strain?: string;

  /** Cellular compartment */
  compartment?: 'cytoplasm' | 'nucleus' | 'membrane' | 'periplasm' | 'extracellular';

  /** Growth conditions */
  growthConditions?: GrowthConditions;

  /** Expression details */
  expression?: ExpressionDetails;
}

/**
 * Growth conditions
 */
export interface GrowthConditions {
  /** Temperature in Celsius */
  temperature?: number;

  /** Growth medium */
  medium?: string;

  /** Antibiotics used */
  antibiotics?: string[];

  /** Inducers/supplements */
  inducers?: string[];

  /** pH */
  pH?: number;

  /** Oxygen level */
  oxygen?: 'aerobic' | 'anaerobic' | 'microaerobic';
}

/**
 * Expression details
 */
export interface ExpressionDetails {
  /** Expression host */
  expressionHost?: string;

  /** Cellular compartment */
  compartment?: string;

  /** Protein tags */
  tags?: string[];

  /** Fusion partners */
  fusionPartners?: string[];

  /** Codon optimization */
  codonOptimized?: boolean;

  /** Optimization organism */
  optimizedFor?: string;
}

/**
 * Author/creator information
 */
export interface AuthorInfo {
  /** Author name or team */
  name: string;

  /** Institution/affiliation */
  affiliation?: string;

  /** Email contact */
  email?: string;

  /** ORCID */
  orcid?: string;

  /** Team members */
  contributors?: string[];
}

/**
 * Reference to publication or resource
 */
export interface Reference {
  /** Reference type */
  type: 'publication' | 'patent' | 'thesis' | 'preprint' | 'website';

  /** Document title */
  title: string;

  /** Authors */
  authors?: string[];

  /** Publication year */
  year?: number;

  /** DOI */
  doi?: string;

  /** PubMed ID */
  pmid?: string;

  /** URL */
  url?: string;
}

/**
 * External database link
 */
export interface ExternalLink {
  /** Database name */
  database: 'GenBank' | 'iGEM' | 'Addgene' | 'SynBioHub' | 'UniProt' | 'PDB' | 'Other';

  /** Accession number or ID */
  accession?: string;

  /** Direct URL */
  url?: string;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

// ============================================================================
// Characterization Data
// ============================================================================

/**
 * Part characterization data
 */
export interface CharacterizationData {
  /** Quantitative measurements */
  measurements?: Measurement[];

  /** Growth curves */
  growthCurve?: GrowthCurveData;

  /** Expression levels */
  expression?: ExpressionData;

  /** Activity assays */
  activity?: ActivityData;

  /** Additional data */
  additionalData?: Record<string, unknown>;
}

/**
 * Single measurement
 */
export interface Measurement {
  /** Measurement type */
  type: string;

  /** Measured value */
  value: number;

  /** Unit of measurement */
  unit: string;

  /** Standard deviation */
  stdDev?: number;

  /** Number of replicates */
  replicates?: number;

  /** Experimental conditions */
  conditions?: Record<string, unknown>;

  /** Measurement method/protocol */
  method?: string;

  /** Who performed the measurement */
  performedBy?: string;

  /** When performed */
  date?: Date | string;
}

/**
 * Growth curve data
 */
export interface GrowthCurveData {
  /** Time series data points */
  dataPoints: Array<{
    /** Time in hours */
    time: number;
    /** Optical density at 600nm */
    od600: number;
  }>;

  /** Maximum growth rate */
  maxGrowthRate?: number;

  /** Unit for growth rate */
  unit?: string;

  /** Strain used */
  strain?: string;

  /** Growth medium */
  medium?: string;

  /** Temperature */
  temperature?: number;
}

/**
 * Expression level data
 */
export interface ExpressionData {
  /** Transcript level */
  transcriptLevel?: {
    value: number;
    unit: string;
    method?: string;
  };

  /** Protein level */
  proteinLevel?: {
    value: number;
    unit: string;
    method?: string;
  };

  /** Activity level */
  activity?: {
    specific: number;
    unit: string;
    substrate?: string;
  };
}

/**
 * Activity assay data
 */
export interface ActivityData {
  /** Enzyme activity */
  enzymeActivity?: number;

  /** Specific activity */
  specificActivity?: number;

  /** Unit */
  unit?: string;

  /** Substrate */
  substrate?: string;

  /** Km (Michaelis constant) */
  km?: number;

  /** Vmax (maximum velocity) */
  vmax?: number;

  /** Optimal pH */
  optimalPH?: number;

  /** Optimal temperature */
  optimalTemperature?: number;
}

// ============================================================================
// Version Control
// ============================================================================

/**
 * Version history entry
 */
export interface VersionHistory {
  /** Version number */
  version: string;

  /** Timestamp */
  date: Date | string;

  /** Author of this version */
  author: string;

  /** Description of changes */
  changes: string;

  /** Sequence hash for this version */
  sequenceHash: string;

  /** Tags */
  tags?: string[];
}

/**
 * Deprecation information
 */
export interface DeprecationInfo {
  /** Is part deprecated? */
  deprecated: boolean;

  /** Deprecation date */
  date?: Date | string;

  /** Reason for deprecation */
  reason?: string;

  /** Replacement part ID */
  replacementPart?: string;

  /** Still physically available? */
  stillAvailable?: boolean;
}

// ============================================================================
// Access Control
// ============================================================================

/**
 * License types
 */
export type LicenseType =
  | 'CC-BY-4.0'
  | 'CC-BY-SA-4.0'
  | 'CC0-1.0'
  | 'MIT'
  | 'Apache-2.0'
  | 'GPL-3.0'
  | 'Proprietary'
  | 'OpenMTA';

/**
 * Visibility levels
 */
export type Visibility = 'public' | 'private' | 'restricted';

/**
 * Access control settings
 */
export interface AccessControl {
  /** Visibility level */
  visibility: Visibility;

  /** Permissions by role */
  permissions?: {
    view?: string[];
    download?: string[];
    modify?: string[];
    characterize?: string[];
  };

  /** Embargo date (if applicable) */
  embargoUntil?: Date | string;

  /** Requires agreement to access? */
  requiresAgreement?: boolean;

  /** URL to agreement/MTA */
  agreementUrl?: string;
}

/**
 * Material transfer information
 */
export interface MaterialTransfer {
  /** Is physical material available? */
  available: boolean;

  /** Provider (Addgene, ATCC, etc.) */
  provider?: string;

  /** Catalog number */
  catalogNumber?: string;

  /** Cost */
  cost?: number;

  /** Currency */
  currency?: string;

  /** Shipping restrictions */
  shippingRestrictions?: string[];

  /** Requires MTA? */
  requiresMTA?: boolean;

  /** MTA document URL */
  mtaUrl?: string;
}

// ============================================================================
// Registry Operations
// ============================================================================

/**
 * Part registration request
 */
export interface PartRegistrationRequest {
  /** Part ID (optional, can be auto-generated) */
  partId?: string;

  /** Part name */
  partName: string;

  /** Part type */
  type: PartType;

  /** Sequence (raw or FASTA format) */
  sequence: string;

  /** Author information */
  author: AuthorInfo;

  /** License */
  license: LicenseType;

  /** Optional fields */
  subtype?: PartSubtype;
  organism?: string;
  description?: string;
  safetyLevel?: SafetyLevel;
  biologicalContext?: BiologicalContext;
}

/**
 * Part registration response
 */
export interface PartRegistrationResponse {
  /** Assigned part ID */
  partId: string;

  /** Version */
  version: string;

  /** Registry URL */
  url: string;

  /** Creation timestamp */
  created: Date | string;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

/**
 * Part search query
 */
export interface PartSearchQuery {
  /** Part type filter */
  type?: PartType;

  /** Organism filter */
  organism?: string;

  /** Keyword search */
  keywords?: string;

  /** Author filter */
  author?: string;

  /** Safety level filter */
  safetyLevel?: SafetyLevel;

  /** Status filter */
  status?: PartStatus;

  /** Sequence search (exact match) */
  sequence?: string;

  /** Part ID prefix */
  partIdPrefix?: string;

  /** Pagination */
  page?: number;
  pageSize?: number;

  /** Sort order */
  sortBy?: 'relevance' | 'date' | 'name' | 'popularity';
  sortOrder?: 'asc' | 'desc';
}

/**
 * Part search result
 */
export interface PartSearchResult {
  /** Matching parts */
  results: PartSearchResultItem[];

  /** Total number of results */
  total: number;

  /** Current page */
  page: number;

  /** Page size */
  pageSize: number;

  /** Total pages */
  totalPages: number;
}

/**
 * Single search result item
 */
export interface PartSearchResultItem {
  /** Part ID */
  partId: string;

  /** Part name */
  name: string;

  /** Part type */
  type: PartType;

  /** Brief description */
  description?: string;

  /** Author */
  author: string;

  /** Version */
  version: string;

  /** Relevance score (0-1) */
  relevance?: number;

  /** Preview snippet */
  snippet?: string;
}

/**
 * Part update request
 */
export interface PartUpdateRequest {
  /** Part ID */
  partId: string;

  /** Fields to update */
  updates: Partial<BiologicalPart>;

  /** Version increment type */
  versionIncrement: 'major' | 'minor' | 'patch';

  /** Change description */
  changeDescription: string;
}

/**
 * Characterization update request
 */
export interface CharacterizationUpdateRequest {
  /** Part ID */
  partId: string;

  /** New measurements */
  measurements?: Measurement[];

  /** Growth curve data */
  growthCurve?: GrowthCurveData;

  /** Expression data */
  expression?: ExpressionData;

  /** Activity data */
  activity?: ActivityData;
}

// ============================================================================
// Reusability & Community
// ============================================================================

/**
 * Reusability metrics
 */
export interface ReusabilityMetrics {
  /** Number of times used by others */
  timesUsed: number;

  /** Success rate (0-1) */
  successRate: number;

  /** Average user rating (1-5) */
  averageRating: number;

  /** User reviews */
  reviews?: UserReview[];

  /** Number of citations */
  citations?: number;

  /** Favorite count */
  favorites?: number;
}

/**
 * User review
 */
export interface UserReview {
  /** User ID or name */
  user: string;

  /** Rating (1-5) */
  rating: number;

  /** Comment */
  comment?: string;

  /** Date of review */
  date: Date | string;

  /** Did it work as expected? */
  workedAsExpected?: boolean;
}

// ============================================================================
// Composite Parts
// ============================================================================

/**
 * Component in composite part
 */
export interface PartComponent {
  /** Component part ID */
  partId: string;

  /** Component name */
  name?: string;

  /** Position in assembly */
  position: number;

  /** Direction */
  direction: 'forward' | 'reverse';

  /** Start position in composite sequence */
  start?: number;

  /** End position in composite sequence */
  end?: number;
}

/**
 * Composite part definition
 */
export interface CompositePart extends BiologicalPart {
  /** Component parts */
  components: PartComponent[];

  /** Assembly method */
  assemblyMethod?: 'BioBrick' | 'Gibson' | 'Golden Gate' | 'SLIC' | 'Other';

  /** Assembly standard */
  assemblyStandard?: 'RFC10' | 'RFC25' | 'RFC23' | 'Other';

  /** Expected function */
  expectedFunction?: string;

  /** Validation status */
  validationStatus?: 'unvalidated' | 'partial' | 'validated' | 'failed';
}

// ============================================================================
// Format Conversion
// ============================================================================

/**
 * Format conversion request
 */
export interface FormatConversionRequest {
  /** Input sequence data */
  input: string;

  /** Input format */
  inputFormat: SequenceFormat;

  /** Output format */
  outputFormat: SequenceFormat;

  /** Part metadata (for SBOL, GenBank) */
  metadata?: Partial<BiologicalPart>;
}

/**
 * Format conversion response
 */
export interface FormatConversionResponse {
  /** Converted sequence */
  output: string;

  /** Output format */
  format: SequenceFormat;

  /** Success status */
  success: boolean;

  /** Error message if failed */
  error?: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-021 error codes
 */
export enum BioErrorCode {
  INVALID_PART_ID = 'BIO001',
  PART_EXISTS = 'BIO002',
  INVALID_SEQUENCE = 'BIO003',
  MISSING_METADATA = 'BIO004',
  VERSION_CONFLICT = 'BIO005',
  SAFETY_VIOLATION = 'BIO006',
  LICENSE_INCOMPATIBLE = 'BIO007',
  FORMAT_CONVERSION_FAILED = 'BIO008',
  PART_NOT_FOUND = 'BIO009',
  UNAUTHORIZED_ACCESS = 'BIO010',
}

/**
 * Biological registry error
 */
export class BioRegistryError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BioRegistryError';
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

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  BiologicalPart,
  SequenceData,
  SequenceFeature,
  BiologicalContext,
  GrowthConditions,
  ExpressionDetails,
  AuthorInfo,
  Reference,
  ExternalLink,

  // Characterization
  CharacterizationData,
  Measurement,
  GrowthCurveData,
  ExpressionData,
  ActivityData,

  // Version control
  VersionHistory,
  DeprecationInfo,

  // Access control
  AccessControl,
  MaterialTransfer,

  // Operations
  PartRegistrationRequest,
  PartRegistrationResponse,
  PartSearchQuery,
  PartSearchResult,
  PartSearchResultItem,
  PartUpdateRequest,
  CharacterizationUpdateRequest,

  // Community
  ReusabilityMetrics,
  UserReview,

  // Composite parts
  PartComponent,
  CompositePart,

  // Format conversion
  FormatConversionRequest,
  FormatConversionResponse,
};

export {
  BioErrorCode,
  BioRegistryError,
};
