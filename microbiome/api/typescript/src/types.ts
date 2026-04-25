/**
 * WIA-BIO-013: Microbiome - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Microbiome Types
// ============================================================================

/**
 * Sample type and body site
 */
export type SampleType =
  | 'gut'
  | 'oral'
  | 'skin'
  | 'vaginal'
  | 'respiratory'
  | 'environmental'
  | 'other';

/**
 * Sequencing method
 */
export type SequencingMethod =
  | '16S'
  | 'ITS'
  | 'shotgun'
  | 'metatranscriptomics'
  | 'targeted';

/**
 * 16S variable region
 */
export type VariableRegion =
  | 'V1-V2'
  | 'V3-V4'
  | 'V4'
  | 'V4-V5'
  | 'full-length';

/**
 * Taxonomic rank
 */
export type TaxonomicRank =
  | 'domain'
  | 'phylum'
  | 'class'
  | 'order'
  | 'family'
  | 'genus'
  | 'species';

/**
 * Diversity metric type
 */
export type DiversityMetric =
  | 'shannon'
  | 'simpson'
  | 'observed'
  | 'chao1'
  | 'ace';

/**
 * Beta diversity metric
 */
export type BetaDiversityMetric =
  | 'bray-curtis'
  | 'jaccard'
  | 'unifrac-weighted'
  | 'unifrac-unweighted'
  | 'aitchison';

// ============================================================================
// Sample and Metadata
// ============================================================================

/**
 * Microbiome sample with metadata
 */
export interface MicrobiomeSample {
  /** Unique sample identifier */
  sampleId: string;

  /** Sample type and body site */
  sampleType: SampleType;

  /** Collection date and time */
  collectionDate: Date | string;

  /** Subject demographics */
  subject?: {
    subjectId?: string;
    age?: number;
    sex?: 'male' | 'female' | 'other';
    bmi?: number;
    ethnicity?: string;
  };

  /** Health status */
  health?: {
    conditions?: string[];
    medications?: string[];
    antibiotics?: {
      used: boolean;
      lastUse?: Date | string;
      duration?: number; // days
    };
    probiotics?: boolean;
    diet?: string;
  };

  /** Sample collection details */
  collection?: {
    method?: string;
    volume?: number; // mL or mg
    stabilizer?: string;
    storage?: {
      temperature: number; // Celsius
      duration: number; // hours
    };
  };

  /** Sequencing information */
  sequencing: {
    method: SequencingMethod;
    region?: VariableRegion;
    platform?: string;
    runDate?: Date | string;
    reads?: number;
  };

  /** Custom metadata */
  metadata?: Record<string, any>;
}

/**
 * Sequence read data
 */
export interface SequenceData {
  /** Read identifier */
  readId: string;

  /** Forward read sequence */
  sequence: string;

  /** Reverse read sequence (if paired-end) */
  reverseSequence?: string;

  /** Quality scores (Phred33) */
  quality?: string;

  /** Reverse quality scores */
  reverseQuality?: string;

  /** Read length */
  length: number;
}

// ============================================================================
// Taxonomic Classification
// ============================================================================

/**
 * Taxonomic assignment
 */
export interface TaxonomicAssignment {
  /** Feature ID (ASV, OTU, or contig) */
  featureId: string;

  /** Full taxonomic lineage */
  taxonomy: string;

  /** Taxonomic ranks */
  ranks: {
    domain?: string;
    phylum?: string;
    class?: string;
    order?: string;
    family?: string;
    genus?: string;
    species?: string;
  };

  /** Classification confidence (0-1) */
  confidence: number;

  /** Classification method */
  method?: string;

  /** Database used */
  database?: string;
}

/**
 * Abundance data for a single taxon
 */
export interface TaxonAbundance {
  /** Taxon name */
  taxon: string;

  /** Taxonomic rank */
  rank: TaxonomicRank;

  /** Absolute abundance (read count) */
  count: number;

  /** Relative abundance (proportion) */
  relativeAbundance: number;

  /** Presence in samples */
  prevalence?: number;

  /** Full taxonomic lineage */
  lineage?: string;
}

/**
 * Abundance table across samples
 */
export interface AbundanceTable {
  /** Sample IDs */
  samples: string[];

  /** Taxa (rows) */
  taxa: TaxonAbundance[];

  /** Abundance matrix [taxa × samples] */
  matrix: number[][];

  /** Taxonomic rank of table */
  rank: TaxonomicRank;

  /** Total reads per sample */
  totalReads: number[];
}

// ============================================================================
// Diversity Analysis
// ============================================================================

/**
 * Alpha diversity metrics
 */
export interface AlphaDiversity {
  /** Sample ID */
  sampleId: string;

  /** Shannon diversity index */
  shannon: number;

  /** Simpson diversity index */
  simpson: number;

  /** Observed species (richness) */
  observed: number;

  /** Chao1 estimator */
  chao1?: number;

  /** ACE estimator */
  ace?: number;

  /** Evenness (Pielou's J) */
  evenness?: number;

  /** Total reads */
  totalReads: number;

  /** Rarefaction depth */
  rarefactionDepth?: number;
}

/**
 * Beta diversity distance matrix
 */
export interface BetaDiversity {
  /** Metric used */
  metric: BetaDiversityMetric;

  /** Sample IDs */
  samples: string[];

  /** Distance matrix (symmetric) */
  distanceMatrix: number[][];

  /** Phylogenetic tree (for UniFrac) */
  phylogeneticTree?: string;
}

/**
 * Rarefaction curve data
 */
export interface RarefactionCurve {
  /** Sample ID */
  sampleId: string;

  /** Sampling depths */
  depths: number[];

  /** Observed species at each depth */
  observedSpecies: number[];

  /** Shannon index at each depth */
  shannon?: number[];

  /** 95% confidence intervals */
  confidenceIntervals?: {
    lower: number[];
    upper: number[];
  };
}

// ============================================================================
// Functional Analysis
// ============================================================================

/**
 * Functional annotation
 */
export interface FunctionalAnnotation {
  /** Gene or protein ID */
  id: string;

  /** Functional category */
  category: 'KEGG' | 'COG' | 'Pfam' | 'GO' | 'MetaCyc';

  /** Annotation ID */
  annotationId: string;

  /** Annotation description */
  description: string;

  /** Abundance or coverage */
  abundance: number;

  /** E-value or confidence */
  confidence?: number;
}

/**
 * Metabolic pathway
 */
export interface MetabolicPathway {
  /** Pathway ID (e.g., KEGG ko00010) */
  pathwayId: string;

  /** Pathway name */
  name: string;

  /** Pathway category */
  category?: string;

  /** Abundance or completeness */
  abundance: number;

  /** Genes in pathway */
  genes?: string[];

  /** Pathway completeness (0-1) */
  completeness?: number;
}

/**
 * Short-chain fatty acid production potential
 */
export interface SCFAProduction {
  /** Sample ID */
  sampleId: string;

  /** Acetate production potential */
  acetate: number;

  /** Propionate production potential */
  propionate: number;

  /** Butyrate production potential */
  butyrate: number;

  /** Key butyrate-producing genera */
  butyrateProducers?: {
    genus: string;
    abundance: number;
  }[];
}

// ============================================================================
// Analysis Results
// ============================================================================

/**
 * Quality control metrics
 */
export interface QualityMetrics {
  /** Sample ID */
  sampleId: string;

  /** Total raw reads */
  rawReads: number;

  /** Reads after QC */
  filteredReads: number;

  /** Pass rate */
  passRate: number;

  /** Average quality score */
  averageQuality: number;

  /** Q30 percentage */
  q30Percentage: number;

  /** GC content */
  gcContent?: number;

  /** Host contamination (if removed) */
  hostContamination?: number;

  /** QC status */
  status: 'pass' | 'warning' | 'fail';

  /** QC warnings or errors */
  messages?: string[];
}

/**
 * Complete microbiome analysis result
 */
export interface MicrobiomeAnalysis {
  /** Analysis ID */
  analysisId: string;

  /** Sample information */
  sample: MicrobiomeSample;

  /** Quality metrics */
  quality: QualityMetrics;

  /** Taxonomic composition */
  taxonomy: {
    assignments: TaxonomicAssignment[];
    abundanceTable: AbundanceTable;
    topTaxa: TaxonAbundance[];
  };

  /** Diversity metrics */
  diversity: {
    alpha: AlphaDiversity;
    rarefaction?: RarefactionCurve;
  };

  /** Functional analysis (if available) */
  functional?: {
    annotations: FunctionalAnnotation[];
    pathways: MetabolicPathway[];
    scfa?: SCFAProduction;
  };

  /** Clinical interpretation */
  clinical?: ClinicalInterpretation;

  /** Analysis metadata */
  metadata: {
    pipeline: string;
    version: string;
    database: string;
    analysisDate: Date | string;
    parameters?: Record<string, any>;
  };
}

// ============================================================================
// Clinical Interpretation
// ============================================================================

/**
 * Dysbiosis assessment
 */
export interface DysbiosisIndex {
  /** Sample ID */
  sampleId: string;

  /** Dysbiosis index value */
  index: number;

  /** Interpretation */
  interpretation: 'healthy' | 'balanced' | 'mild-dysbiosis' | 'moderate-dysbiosis' | 'severe-dysbiosis';

  /** Pathobiont abundance */
  pathobionts: number;

  /** Commensal abundance */
  commensals: number;

  /** Key pathobionts detected */
  keyPathobionts?: TaxonAbundance[];

  /** Depleted beneficial taxa */
  depletedBeneficial?: TaxonAbundance[];
}

/**
 * Disease association
 */
export interface DiseaseAssociation {
  /** Disease or condition */
  disease: string;

  /** Risk score (0-1) */
  riskScore: number;

  /** Confidence level */
  confidence: 'low' | 'medium' | 'high';

  /** Associated taxa */
  associatedTaxa: {
    taxon: string;
    direction: 'increased' | 'decreased';
    foldChange: number;
  }[];

  /** References */
  references?: string[];
}

/**
 * Clinical interpretation
 */
export interface ClinicalInterpretation {
  /** Sample ID */
  sampleId: string;

  /** Overall health score (0-100) */
  healthScore: number;

  /** Dysbiosis assessment */
  dysbiosis: DysbiosisIndex;

  /** Disease associations */
  diseaseRisks?: DiseaseAssociation[];

  /** Key findings */
  findings: {
    category: 'positive' | 'negative' | 'neutral';
    description: string;
    severity?: 'low' | 'medium' | 'high';
  }[];

  /** Recommendations */
  recommendations: {
    type: 'dietary' | 'probiotic' | 'lifestyle' | 'medical';
    recommendation: string;
    priority: 'low' | 'medium' | 'high';
  }[];

  /** Reference ranges */
  referenceRanges?: {
    metric: string;
    value: number;
    referenceRange: [number, number];
    status: 'low' | 'normal' | 'high';
  }[];
}

// ============================================================================
// FMT (Fecal Microbiota Transplant)
// ============================================================================

/**
 * FMT donor screening
 */
export interface FMTDonor {
  /** Donor ID */
  donorId: string;

  /** Demographics */
  age: number;
  sex: 'male' | 'female';
  bmi: number;

  /** Screening results */
  screening: {
    antibioticFree: boolean;
    giHealthy: boolean;
    bloodTests: {
      hiv: 'negative' | 'positive';
      hepatitisB: 'negative' | 'positive';
      hepatitisC: 'negative' | 'positive';
    };
    stoolTests: {
      parasites: 'negative' | 'positive';
      cdiff: 'negative' | 'positive';
      pathogens: 'negative' | 'positive';
    };
  };

  /** Microbiome profile */
  microbiome: {
    diversity: AlphaDiversity;
    composition: TaxonAbundance[];
    dysbiosis: DysbiosisIndex;
  };

  /** Donor suitability */
  suitability: 'excellent' | 'good' | 'fair' | 'unsuitable';

  /** Approval status */
  approved: boolean;
}

/**
 * FMT efficacy prediction
 */
export interface FMTEfficacy {
  /** Recipient sample ID */
  recipientId: string;

  /** Donor ID */
  donorId: string;

  /** Pre-FMT diversity */
  diversityPre: AlphaDiversity;

  /** Donor diversity */
  diversityDonor: AlphaDiversity;

  /** Post-FMT diversity (if available) */
  diversityPost?: AlphaDiversity;

  /** FMT score */
  fmtScore: number;

  /** Predicted engraftment rate */
  predictedEngraftment: number;

  /** Efficacy prediction */
  prediction: 'excellent' | 'good' | 'fair' | 'poor';
}

// ============================================================================
// Batch Analysis
// ============================================================================

/**
 * Comparative analysis across samples
 */
export interface ComparativeAnalysis {
  /** Analysis ID */
  analysisId: string;

  /** Sample groups */
  groups: {
    groupId: string;
    name: string;
    samples: string[];
  }[];

  /** Beta diversity */
  betaDiversity: BetaDiversity;

  /** Differential abundance */
  differentialAbundance?: {
    taxon: string;
    log2FoldChange: number;
    pValue: number;
    adjustedPValue: number;
    significant: boolean;
  }[];

  /** PERMANOVA results */
  permanova?: {
    rSquared: number;
    pValue: number;
    significant: boolean;
  };

  /** Principal coordinates (PCoA) */
  pcoa?: {
    samples: string[];
    pc1: number[];
    pc2: number[];
    pc3?: number[];
    varianceExplained: number[];
  };
}

// ============================================================================
// Analysis Parameters
// ============================================================================

/**
 * Pipeline parameters for analysis
 */
export interface AnalysisParameters {
  /** Quality filtering */
  qualityFilter?: {
    minQuality: number;
    minLength: number;
    maxExpectedErrors: number;
  };

  /** Denoising (DADA2/Deblur) */
  denoising?: {
    method: 'dada2' | 'deblur' | 'none';
    trimLeft?: number;
    truncLen?: number;
  };

  /** Taxonomic classification */
  classification?: {
    database: 'silva' | 'greengenes' | 'rdp' | 'custom';
    confidenceThreshold: number;
    method: 'naive-bayes' | 'blast' | 'vsearch';
  };

  /** Diversity analysis */
  diversity?: {
    rarefactionDepth?: number;
    betaMetric: BetaDiversityMetric;
  };

  /** Functional analysis */
  functional?: {
    enable: boolean;
    databases: ('KEGG' | 'COG' | 'Pfam')[];
  };
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Reference values for microbiome metrics
 */
export const MICROBIOME_CONSTANTS = {
  /** Healthy gut Shannon diversity range */
  HEALTHY_GUT_SHANNON: [3.5, 5.5],

  /** Dysbiosis threshold */
  DYSBIOSIS_THRESHOLD: 0.0,

  /** Minimum read depth for analysis */
  MIN_READ_DEPTH: 1000,

  /** Recommended rarefaction depth */
  RECOMMENDED_RAREFACTION: 10000,

  /** Minimum Q30 percentage */
  MIN_Q30_PERCENTAGE: 75,

  /** Firmicutes/Bacteroidetes ratio (healthy) */
  HEALTHY_FB_RATIO: [1.0, 3.0],

  /** Minimum donor diversity for FMT */
  MIN_DONOR_DIVERSITY: 4.0,
} as const;

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
// Error Types
// ============================================================================

/**
 * WIA-BIO-013 error codes
 */
export enum MicrobiomeErrorCode {
  INVALID_SAMPLE = 'BIO013-001',
  INSUFFICIENT_READS = 'BIO013-002',
  LOW_QUALITY = 'BIO013-003',
  CLASSIFICATION_FAILED = 'BIO013-004',
  DIVERSITY_CALCULATION_FAILED = 'BIO013-005',
  INVALID_PARAMETERS = 'BIO013-006',
  DATABASE_ERROR = 'BIO013-007',
  ANALYSIS_FAILED = 'BIO013-008',
}

/**
 * Microbiome analysis error
 */
export class MicrobiomeError extends Error {
  constructor(
    public code: MicrobiomeErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'MicrobiomeError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  SampleType,
  SequencingMethod,
  VariableRegion,
  TaxonomicRank,
  DiversityMetric,
  BetaDiversityMetric,

  // Sample
  MicrobiomeSample,
  SequenceData,

  // Taxonomy
  TaxonomicAssignment,
  TaxonAbundance,
  AbundanceTable,

  // Diversity
  AlphaDiversity,
  BetaDiversity,
  RarefactionCurve,

  // Functional
  FunctionalAnnotation,
  MetabolicPathway,
  SCFAProduction,

  // Analysis
  QualityMetrics,
  MicrobiomeAnalysis,

  // Clinical
  DysbiosisIndex,
  DiseaseAssociation,
  ClinicalInterpretation,

  // FMT
  FMTDonor,
  FMTEfficacy,

  // Comparative
  ComparativeAnalysis,

  // Parameters
  AnalysisParameters,
};

export { MICROBIOME_CONSTANTS, MicrobiomeErrorCode, MicrobiomeError };
