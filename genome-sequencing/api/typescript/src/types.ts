/**
 * WIA-BIO-002: Genome Sequencing - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bioinformatics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Sequencing Types
// ============================================================================

/**
 * Sequencing platform types
 */
export type SequencingPlatform =
  | 'illumina-novaseq'
  | 'illumina-nextseq'
  | 'illumina-miseq'
  | 'pacbio-sequel'
  | 'oxford-nanopore'
  | '10x-genomics'
  | 'other';

/**
 * Reference genome builds
 */
export type ReferenceGenome = 'hg38' | 'hg19' | 'GRCh38' | 'GRCh37' | 'mm10' | 'mm39';

/**
 * Sequencing read
 */
export interface SequenceRead {
  /** Read identifier */
  id: string;

  /** DNA/RNA sequence */
  sequence: string;

  /** Quality scores (Phred+33 ASCII) */
  quality: string;

  /** Read length in base pairs */
  length: number;

  /** Read number (1 or 2 for paired-end) */
  readNumber?: 1 | 2;

  /** Is this read filtered? */
  filtered?: boolean;
}

/**
 * Quality score metrics
 */
export interface QualityScore {
  /** Phred quality score */
  phred: number;

  /** Probability of error (0-1) */
  errorProbability: number;

  /** Accuracy percentage */
  accuracy: number;

  /** Position in read (0-based) */
  position?: number;
}

/**
 * Genome assembly information
 */
export interface GenomeAssembly {
  /** Assembly name/version */
  name: string;

  /** Species */
  species: string;

  /** Total genome size in base pairs */
  size: number;

  /** Number of chromosomes */
  chromosomes: number;

  /** Assembly accession */
  accession?: string;

  /** Assembly date */
  releaseDate?: Date;
}

// ============================================================================
// Coverage and Depth
// ============================================================================

/**
 * Coverage calculation parameters
 */
export interface CoverageParameters {
  /** Total number of reads */
  totalReads: number;

  /** Read length in base pairs */
  readLength: number;

  /** Genome size in base pairs */
  genomeSize: number;

  /** Is paired-end sequencing? */
  pairedEnd?: boolean;

  /** Expected duplication rate (0-1) */
  duplicationRate?: number;
}

/**
 * Coverage statistics
 */
export interface CoverageStats {
  /** Average (mean) coverage depth */
  averageDepth: number;

  /** Median coverage depth */
  medianDepth: number;

  /** Standard deviation of coverage */
  stdDeviation: number;

  /** Coverage uniformity (0-1) */
  uniformity: number;

  /** Total bases sequenced */
  totalBases: number;

  /** Percentage of genome covered at ≥1× */
  coverage1x: number;

  /** Percentage of genome covered at ≥10× */
  coverage10x: number;

  /** Percentage of genome covered at ≥20× */
  coverage20x: number;

  /** Percentage of genome covered at ≥30× */
  coverage30x: number;

  /** Feasibility assessment */
  feasibility: 'excellent' | 'good' | 'acceptable' | 'insufficient';
}

/**
 * Regional coverage information
 */
export interface RegionalCoverage {
  /** Chromosome/contig name */
  chromosome: string;

  /** Start position (1-based) */
  start: number;

  /** End position (1-based, inclusive) */
  end: number;

  /** Mean coverage in region */
  meanDepth: number;

  /** Minimum coverage in region */
  minDepth: number;

  /** Maximum coverage in region */
  maxDepth: number;

  /** Percentage of region with coverage ≥threshold */
  percentCovered: number;
}

// ============================================================================
// Variant Types
// ============================================================================

/**
 * Variant type classification
 */
export type VariantType = 'SNP' | 'insertion' | 'deletion' | 'indel' | 'MNP' | 'CNV' | 'SV';

/**
 * Variant zygosity
 */
export type Zygosity = 'homozygous' | 'heterozygous' | 'hemizygous';

/**
 * Variant filter status
 */
export type FilterStatus = 'PASS' | 'FAIL' | 'LOW_QUALITY' | 'LOW_DEPTH' | 'STRAND_BIAS';

/**
 * Variant call
 */
export interface VariantCall {
  /** Chromosome/contig */
  chromosome: string;

  /** Position (1-based) */
  position: number;

  /** Reference allele */
  reference: string;

  /** Alternative allele(s) */
  alternative: string | string[];

  /** Variant type */
  type: VariantType;

  /** Variant quality score */
  quality: number;

  /** Filter status */
  filter: FilterStatus;

  /** Total read depth at position */
  depth: number;

  /** Variant allele frequency (0-1) */
  vaf: number;

  /** Genotype (e.g., "0/1", "1/1") */
  genotype: string;

  /** Zygosity */
  zygosity: Zygosity;

  /** Genotype quality */
  genotypeQuality?: number;

  /** dbSNP rs identifier */
  rsId?: string;

  /** Additional INFO fields */
  info?: Record<string, unknown>;
}

/**
 * Structural variant
 */
export interface StructuralVariant {
  /** Variant identifier */
  id: string;

  /** Type of structural variant */
  type: 'deletion' | 'insertion' | 'duplication' | 'inversion' | 'translocation';

  /** Chromosome 1 */
  chromosome1: string;

  /** Position 1 */
  position1: number;

  /** Chromosome 2 (for translocations) */
  chromosome2?: string;

  /** Position 2 */
  position2?: number;

  /** Size of variant in base pairs */
  size: number;

  /** Supporting read count */
  supportingReads: number;

  /** Quality score */
  quality: number;

  /** Filter status */
  filter: FilterStatus;
}

// ============================================================================
// Annotation
// ============================================================================

/**
 * Gene annotation
 */
export interface GeneAnnotation {
  /** Gene symbol */
  symbol: string;

  /** Gene identifier (e.g., ENSG00000...) */
  geneId: string;

  /** Transcript identifier (e.g., ENST00000...) */
  transcriptId?: string;

  /** Chromosome */
  chromosome: string;

  /** Gene start position */
  start: number;

  /** Gene end position */
  end: number;

  /** Strand (+ or -) */
  strand: '+' | '-';

  /** Gene biotype */
  biotype?: string;

  /** Gene description */
  description?: string;
}

/**
 * Variant consequence types (VEP-style)
 */
export type ConsequenceType =
  | 'stop_gained'
  | 'stop_lost'
  | 'start_lost'
  | 'frameshift_variant'
  | 'missense_variant'
  | 'synonymous_variant'
  | 'splice_donor_variant'
  | 'splice_acceptor_variant'
  | 'inframe_insertion'
  | 'inframe_deletion'
  | 'intron_variant'
  | 'upstream_variant'
  | 'downstream_variant'
  | '5_prime_UTR_variant'
  | '3_prime_UTR_variant'
  | 'intergenic_variant';

/**
 * Impact severity
 */
export type ImpactSeverity = 'HIGH' | 'MODERATE' | 'LOW' | 'MODIFIER';

/**
 * Variant annotation
 */
export interface VariantAnnotation {
  /** Gene information */
  gene: GeneAnnotation;

  /** Consequence type(s) */
  consequences: ConsequenceType[];

  /** Impact severity */
  impact: ImpactSeverity;

  /** Amino acid change (e.g., "p.Arg123Gln") */
  proteinChange?: string;

  /** cDNA position */
  cdnaPosition?: number;

  /** Coding sequence position */
  cdsPosition?: number;

  /** Protein position */
  proteinPosition?: number;

  /** SIFT prediction */
  sift?: {
    score: number;
    prediction: 'tolerated' | 'deleterious';
  };

  /** PolyPhen-2 prediction */
  polyphen?: {
    score: number;
    prediction: 'benign' | 'possibly_damaging' | 'probably_damaging';
  };

  /** CADD score */
  caddScore?: number;

  /** Population frequency (gnomAD) */
  gnomadFrequency?: number;

  /** ClinVar significance */
  clinvarSignificance?: ClinicalSignificance;

  /** COSMIC identifier */
  cosmicId?: string;
}

/**
 * Clinical significance (ClinVar)
 */
export type ClinicalSignificance =
  | 'pathogenic'
  | 'likely_pathogenic'
  | 'uncertain_significance'
  | 'likely_benign'
  | 'benign';

// ============================================================================
// Quality Control
// ============================================================================

/**
 * Quality validation parameters
 */
export interface QualityValidation {
  /** Percentage of bases with Q≥30 */
  q30Percentage: number;

  /** Mean sequencing coverage */
  meanCoverage: number;

  /** Coverage uniformity (0-1) */
  coverageUniformity: number;

  /** Mapping rate (0-1) */
  mappingRate: number;

  /** Duplication rate (0-1) */
  duplicationRate: number;

  /** Contamination rate (0-1) */
  contaminationRate: number;

  /** GC bias coefficient */
  gcBias?: number;

  /** Insert size mean (paired-end) */
  insertSizeMean?: number;

  /** Insert size standard deviation */
  insertSizeStd?: number;
}

/**
 * Quality assessment result
 */
export interface QualityResult {
  /** Is quality acceptable? */
  isValid: boolean;

  /** Quality grade */
  grade: 'clinical' | 'research' | 'failed';

  /** Errors (blocking issues) */
  errors: string[];

  /** Warnings (non-blocking issues) */
  warnings: string[];

  /** Recommendations for improvement */
  recommendations: string[];

  /** Individual metric checks */
  checks: QualityCheck[];

  /** Overall quality score (0-100) */
  overallScore: number;
}

/**
 * Individual quality check
 */
export interface QualityCheck {
  /** Metric name */
  name: string;

  /** Check status */
  status: 'pass' | 'warn' | 'fail';

  /** Measured value */
  value: number;

  /** Expected/threshold value */
  threshold: number;

  /** Description */
  description: string;

  /** Corrective action if failed */
  correctiveAction?: string;
}

// ============================================================================
// Sequencing Run
// ============================================================================

/**
 * Sequencing run metadata
 */
export interface SequencingRun {
  /** Run identifier */
  runId: string;

  /** Sequencing platform */
  platform: SequencingPlatform;

  /** Flow cell identifier */
  flowCellId: string;

  /** Run date */
  runDate: Date;

  /** Read configuration (e.g., "2x150") */
  readConfiguration: string;

  /** Chemistry/kit version */
  chemistry: string;

  /** Operator/technician */
  operator?: string;

  /** Number of samples in run */
  sampleCount: number;

  /** Run status */
  status: 'pending' | 'running' | 'completed' | 'failed';

  /** QC metrics */
  qcMetrics?: {
    clusterDensity?: number;
    clustersPF?: number;
    q30?: number;
    errorRate?: number;
  };
}

/**
 * Sample metadata
 */
export interface SampleMetadata {
  /** Sample identifier */
  sampleId: string;

  /** Patient/subject identifier (hashed/anonymized) */
  patientId: string;

  /** Sample type */
  sampleType: 'blood' | 'saliva' | 'tissue' | 'ffpe' | 'cell-line' | 'other';

  /** Tissue type */
  tissueType: 'germline' | 'tumor' | 'normal' | 'other';

  /** Collection date */
  collectionDate: Date;

  /** Library preparation kit */
  libraryPrep: string;

  /** Target coverage depth */
  targetCoverage: number;

  /** Reference genome */
  referenceGenome: ReferenceGenome;

  /** Sequencing type */
  sequencingType: 'WGS' | 'WES' | 'targeted' | 'RNA-seq' | 'ChIP-seq' | 'other';

  /** Target regions (for targeted sequencing) */
  targetRegions?: string;

  /** Clinical indication */
  clinicalIndication?: string;
}

// ============================================================================
// Analysis Pipeline
// ============================================================================

/**
 * Variant calling parameters
 */
export interface VariantCallingParams {
  /** Input BAM file path */
  bamFile: string;

  /** Reference genome */
  referenceGenome: ReferenceGenome;

  /** Variant caller to use */
  caller: 'gatk' | 'freebayes' | 'mutect2' | 'strelka2' | 'varscan2';

  /** Minimum read depth */
  minDepth: number;

  /** Minimum variant quality */
  minQuality: number;

  /** Minimum variant allele frequency */
  minVAF: number;

  /** Minimum mapping quality */
  minMapQuality?: number;

  /** Minimum base quality */
  minBaseQuality?: number;

  /** Target regions BED file */
  targetRegions?: string;

  /** Matched normal BAM (for somatic calling) */
  normalBam?: string;

  /** Output VCF file path */
  outputVcf: string;
}

/**
 * Annotation parameters
 */
export interface AnnotationParams {
  /** Input VCF file */
  vcfFile: string;

  /** Annotation databases to use */
  databases: ('clinvar' | 'dbsnp' | 'gnomad' | 'cosmic' | 'omim')[];

  /** Prediction tools to use */
  predictors?: ('sift' | 'polyphen' | 'cadd')[];

  /** Include population frequencies */
  includeFrequencies?: boolean;

  /** Output annotated VCF */
  outputVcf: string;
}

/**
 * Analysis report
 */
export interface AnalysisReport {
  /** Report identifier */
  reportId: string;

  /** Sample information */
  sample: SampleMetadata;

  /** Sequencing run information */
  run: SequencingRun;

  /** Quality metrics */
  quality: QualityResult;

  /** Coverage statistics */
  coverage: CoverageStats;

  /** Total variants called */
  totalVariants: number;

  /** Variants by type */
  variantsByType: {
    snp: number;
    insertion: number;
    deletion: number;
    indel: number;
    cnv?: number;
    sv?: number;
  };

  /** Clinically significant variants */
  clinicalVariants: VariantCall[];

  /** Ti/Tv ratio (transition/transversion) */
  tiTvRatio: number;

  /** Het/Hom ratio (heterozygous/homozygous) */
  hetHomRatio: number;

  /** Novel variants (not in dbSNP) */
  novelVariants: number;

  /** Report generation date */
  generatedDate: Date;

  /** Analysis pipeline version */
  pipelineVersion: string;
}

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Sequencing constants and thresholds
 */
export const SEQUENCING_CONSTANTS = {
  /** Human genome size (hg38) in base pairs */
  HUMAN_GENOME_SIZE: 3_088_269_832,

  /** Minimum Phred score for high-quality base */
  MIN_PHRED_QUALITY: 30,

  /** Clinical-grade minimum coverage */
  CLINICAL_MIN_COVERAGE: 30,

  /** Research-grade minimum coverage */
  RESEARCH_MIN_COVERAGE: 20,

  /** Minimum Q30 percentage for clinical */
  CLINICAL_MIN_Q30: 90,

  /** Minimum Q30 percentage for research */
  RESEARCH_MIN_Q30: 85,

  /** Maximum acceptable duplication rate */
  MAX_DUPLICATION_RATE: 0.2,

  /** Minimum mapping rate */
  MIN_MAPPING_RATE: 0.95,

  /** Minimum coverage uniformity */
  MIN_UNIFORMITY: 0.9,

  /** Maximum contamination rate */
  MAX_CONTAMINATION: 0.01,

  /** Expected Ti/Tv ratio for WGS */
  EXPECTED_TITV_WGS: 2.0,

  /** Expected Ti/Tv ratio for WES */
  EXPECTED_TITV_WES: 3.0,

  /** Expected Het/Hom ratio */
  EXPECTED_HET_HOM: 1.5,

  /** Minimum variant quality score */
  MIN_VARIANT_QUALITY: 30,

  /** Minimum variant depth */
  MIN_VARIANT_DEPTH: 10,

  /** Population frequency threshold (rare variant) */
  RARE_VARIANT_FREQ: 0.01,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-002 error codes
 */
export enum BioErrorCode {
  INSUFFICIENT_COVERAGE = 'B001',
  LOW_QUALITY = 'B002',
  HIGH_DUPLICATION = 'B003',
  POOR_UNIFORMITY = 'B004',
  CONTAMINATION_DETECTED = 'B005',
  MAPPING_FAILURE = 'B006',
  INVALID_PARAMETERS = 'B007',
  FILE_NOT_FOUND = 'B008',
  PARSE_ERROR = 'B009',
  ANNOTATION_FAILED = 'B010',
}

/**
 * Genome sequencing error
 */
export class GenomeSequencingError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'GenomeSequencingError';
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
 * Genomic coordinate
 */
export interface GenomicCoordinate {
  chromosome: string;
  position: number;
  referenceGenome: ReferenceGenome;
}

/**
 * Genomic region
 */
export interface GenomicRegion {
  chromosome: string;
  start: number;
  end: number;
  name?: string;
  strand?: '+' | '-';
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  SequencingPlatform,
  ReferenceGenome,
  SequenceRead,
  QualityScore,
  GenomeAssembly,

  // Coverage
  CoverageParameters,
  CoverageStats,
  RegionalCoverage,

  // Variants
  VariantType,
  Zygosity,
  FilterStatus,
  VariantCall,
  StructuralVariant,

  // Annotation
  GeneAnnotation,
  ConsequenceType,
  ImpactSeverity,
  VariantAnnotation,
  ClinicalSignificance,

  // Quality
  QualityValidation,
  QualityResult,
  QualityCheck,

  // Sequencing
  SequencingRun,
  SampleMetadata,

  // Analysis
  VariantCallingParams,
  AnnotationParams,
  AnalysisReport,

  // Utility
  GenomicCoordinate,
  GenomicRegion,
};

export { SEQUENCING_CONSTANTS, BioErrorCode, GenomeSequencingError };
