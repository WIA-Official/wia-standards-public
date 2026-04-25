/**
 * WIA-BIO-007: Bioinformatics - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bioinformatics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Sequence Types
// ============================================================================

/**
 * Biological sequence with metadata
 */
export interface Sequence {
  /** Unique sequence identifier */
  id: string;

  /** Sequence header/description */
  header?: string;

  /** Sequence string (amino acids or nucleotides) */
  sequence: string;

  /** Sequence type */
  type: 'protein' | 'dna' | 'rna';

  /** Sequence length */
  length: number;

  /** Optional metadata */
  metadata?: {
    organism?: string;
    gene?: string;
    accession?: string;
    database?: string;
    [key: string]: unknown;
  };
}

/**
 * Alignment between two sequences
 */
export interface Alignment {
  /** Query sequence */
  query: Sequence;

  /** Subject sequence */
  subject: Sequence;

  /** Aligned query sequence (with gaps) */
  aligned1: string;

  /** Aligned subject sequence (with gaps) */
  aligned2: string;

  /** Alignment score */
  score: number;

  /** Percent identity (0-100) */
  identity: number;

  /** Percent similarity (0-100) */
  similarity: number;

  /** Number of gaps */
  gaps: number;

  /** Alignment length */
  length: number;

  /** Query coverage (0-100) */
  queryCoverage: number;

  /** Subject coverage (0-100) */
  subjectCoverage: number;

  /** Start position in query */
  queryStart: number;

  /** End position in query */
  queryEnd: number;

  /** Start position in subject */
  subjectStart: number;

  /** End position in subject */
  subjectEnd: number;

  /** Algorithm used */
  algorithm: 'needleman-wunsch' | 'smith-waterman' | 'blast';
}

/**
 * Alignment score configuration
 */
export interface AlignmentScore {
  /** Match score */
  match: number;

  /** Mismatch penalty */
  mismatch: number;

  /** Gap opening penalty */
  gapOpen: number;

  /** Gap extension penalty */
  gapExtension: number;

  /** Scoring matrix (e.g., BLOSUM62, PAM250) */
  matrix?: 'BLOSUM62' | 'PAM250' | 'IDENTITY' | string;
}

// ============================================================================
// Database Search Types
// ============================================================================

/**
 * Database search parameters
 */
export interface DatabaseSearch {
  /** Query sequence */
  query: Sequence | string;

  /** Target database */
  database: 'UniProt' | 'NCBI-nr' | 'PDB' | 'Ensembl' | 'Custom' | string;

  /** Search algorithm */
  algorithm: 'BLAST' | 'FASTA' | 'HMMER' | string;

  /** E-value threshold */
  eValueThreshold?: number;

  /** Maximum number of hits to return */
  maxHits?: number;

  /** Minimum identity threshold (0-100) */
  minIdentity?: number;

  /** Minimum coverage threshold (0-100) */
  minCoverage?: number;

  /** Alignment scoring parameters */
  scoring?: AlignmentScore;
}

/**
 * Database search result hit
 */
export interface SearchHit {
  /** Hit identifier */
  id: string;

  /** Hit description */
  description: string;

  /** Hit sequence */
  sequence?: Sequence;

  /** Alignment to query */
  alignment: Alignment;

  /** E-value (statistical significance) */
  eValue: number;

  /** Bit score */
  bitScore: number;

  /** Percent identity */
  identity: number;

  /** Query coverage */
  coverage: number;

  /** Database source */
  database: string;
}

/**
 * Database search results
 */
export interface SearchResults {
  /** Search query */
  query: Sequence;

  /** Database searched */
  database: string;

  /** Algorithm used */
  algorithm: string;

  /** List of hits */
  hits: SearchHit[];

  /** Total number of hits found */
  totalHits: number;

  /** Search execution time (ms) */
  executionTime: number;

  /** Search parameters */
  parameters: DatabaseSearch;
}

// ============================================================================
// Phylogenetic Analysis Types
// ============================================================================

/**
 * Phylogenetic tree node
 */
export interface PhylogeneticNode {
  /** Node identifier */
  id: string;

  /** Node label (taxon name for leaves) */
  label?: string;

  /** Branch length to parent */
  branchLength?: number;

  /** Bootstrap support value (0-100) */
  bootstrap?: number;

  /** Children nodes */
  children?: PhylogeneticNode[];

  /** Is this a leaf node? */
  isLeaf: boolean;

  /** Sequence associated with leaf */
  sequence?: Sequence;
}

/**
 * Phylogenetic tree
 */
export interface PhylogeneticTree {
  /** Tree identifier */
  id: string;

  /** Root node */
  root: PhylogeneticNode;

  /** Tree construction method */
  method: 'neighbor-joining' | 'maximum-likelihood' | 'bayesian' | 'upgma' | string;

  /** Substitution model used */
  model?: 'JC69' | 'K2P' | 'HKY85' | 'GTR' | string;

  /** Number of taxa (leaves) */
  numTaxa: number;

  /** Tree in Newick format */
  newick: string;

  /** Total tree length */
  totalLength?: number;

  /** Log-likelihood (for ML trees) */
  logLikelihood?: number;

  /** Bootstrap replicates */
  bootstrapReplicates?: number;
}

/**
 * Distance matrix for phylogenetic analysis
 */
export interface DistanceMatrix {
  /** Taxon labels */
  labels: string[];

  /** Pairwise distance matrix */
  distances: number[][];

  /** Distance calculation method */
  method: 'p-distance' | 'jukes-cantor' | 'kimura' | string;
}

// ============================================================================
// Gene Ontology and Pathway Types
// ============================================================================

/**
 * Gene Ontology term
 */
export interface GeneOntology {
  /** GO term ID (e.g., GO:0008150) */
  id: string;

  /** GO term name */
  name: string;

  /** GO namespace */
  namespace: 'biological_process' | 'molecular_function' | 'cellular_component';

  /** Definition */
  definition: string;

  /** Evidence code */
  evidence?: string[];

  /** Parent terms */
  parents?: string[];

  /** Child terms */
  children?: string[];
}

/**
 * Biological pathway
 */
export interface Pathway {
  /** Pathway identifier */
  id: string;

  /** Pathway name */
  name: string;

  /** Pathway database source */
  source: 'KEGG' | 'Reactome' | 'WikiPathways' | 'GO' | string;

  /** Genes in pathway */
  genes: string[];

  /** Pathway description */
  description?: string;

  /** Pathway category */
  category?: string;

  /** Organism */
  organism?: string;
}

/**
 * Pathway enrichment result
 */
export interface PathwayEnrichment {
  /** Pathway */
  pathway: Pathway;

  /** Number of genes from query in pathway */
  queryGenesInPathway: number;

  /** Total query genes */
  totalQueryGenes: number;

  /** Total genes in pathway */
  totalPathwayGenes: number;

  /** Total background genes */
  totalBackgroundGenes: number;

  /** P-value */
  pValue: number;

  /** Adjusted p-value (FDR) */
  adjustedPValue: number;

  /** Fold enrichment */
  foldEnrichment: number;

  /** Genes from query in this pathway */
  genes: string[];
}

// ============================================================================
// Machine Learning Types
// ============================================================================

/**
 * Protein structure prediction result
 */
export interface StructurePrediction {
  /** Sequence */
  sequence: Sequence;

  /** Predicted structure in PDB format */
  pdb: string;

  /** Confidence scores per residue (pLDDT) */
  confidenceScores: number[];

  /** Overall confidence */
  overallConfidence: number;

  /** Model used */
  model: 'AlphaFold2' | 'ESMFold' | 'RoseTTAFold' | string;

  /** Predicted secondary structure */
  secondaryStructure?: string;

  /** Predicted solvent accessibility */
  solventAccessibility?: number[];
}

/**
 * Variant effect prediction
 */
export interface VariantEffect {
  /** Chromosome */
  chromosome: string;

  /** Position */
  position: number;

  /** Reference allele */
  reference: string;

  /** Alternative allele */
  alternative: string;

  /** Gene symbol */
  gene?: string;

  /** Protein change */
  proteinChange?: string;

  /** SIFT score (0-1) */
  siftScore?: number;

  /** SIFT prediction */
  siftPrediction?: 'deleterious' | 'tolerated';

  /** PolyPhen-2 score (0-1) */
  polyphenScore?: number;

  /** PolyPhen-2 prediction */
  polyphenPrediction?: 'probably-damaging' | 'possibly-damaging' | 'benign';

  /** CADD score */
  caddScore?: number;

  /** Clinical significance */
  clinicalSignificance?: 'pathogenic' | 'likely-pathogenic' | 'uncertain' | 'likely-benign' | 'benign';
}

// ============================================================================
// Pipeline Types
// ============================================================================

/**
 * Analysis pipeline step
 */
export interface PipelineStep {
  /** Step identifier */
  id: string;

  /** Step name */
  name: string;

  /** Tool/command to execute */
  command: string;

  /** Input files */
  inputs: string[];

  /** Output files */
  outputs: string[];

  /** Parameters */
  parameters?: Record<string, unknown>;

  /** Dependencies (step IDs that must complete first) */
  dependencies?: string[];

  /** Container image */
  container?: string;

  /** CPU cores */
  cpus?: number;

  /** Memory (GB) */
  memory?: number;

  /** Timeout (seconds) */
  timeout?: number;
}

/**
 * Analysis pipeline
 */
export interface AnalysisPipeline {
  /** Pipeline identifier */
  id: string;

  /** Pipeline name */
  name: string;

  /** Pipeline version */
  version: string;

  /** Pipeline description */
  description?: string;

  /** Pipeline steps */
  steps: PipelineStep[];

  /** Global parameters */
  globalParameters?: Record<string, unknown>;

  /** Workflow engine */
  engine?: 'nextflow' | 'snakemake' | 'cwl' | 'manual';
}

/**
 * Compute job
 */
export interface ComputeJob {
  /** Job identifier */
  id: string;

  /** Job name */
  name: string;

  /** Pipeline step */
  step: PipelineStep;

  /** Job status */
  status: 'pending' | 'running' | 'completed' | 'failed' | 'cancelled';

  /** Start time */
  startTime?: Date;

  /** End time */
  endTime?: Date;

  /** Execution time (seconds) */
  executionTime?: number;

  /** Exit code */
  exitCode?: number;

  /** Standard output */
  stdout?: string;

  /** Standard error */
  stderr?: string;

  /** Resource usage */
  resources?: {
    cpuUsage?: number;
    memoryUsage?: number;
    diskUsage?: number;
  };
}

/**
 * Analysis result set
 */
export interface ResultSet {
  /** Analysis identifier */
  analysisId: string;

  /** Pipeline used */
  pipeline: AnalysisPipeline;

  /** Jobs executed */
  jobs: ComputeJob[];

  /** Output files */
  outputs: {
    path: string;
    type: string;
    size: number;
    checksum?: string;
  }[];

  /** Metadata */
  metadata: {
    startTime: Date;
    endTime: Date;
    executionTime: number;
    author?: string;
    version?: string;
    [key: string]: unknown;
  };

  /** Success status */
  success: boolean;

  /** Error messages */
  errors?: string[];
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Bioinformatics constants
 */
export const BIOINFORMATICS_CONSTANTS = {
  /** Default gap opening penalty */
  GAP_OPEN_PENALTY: -10,

  /** Default gap extension penalty */
  GAP_EXTENSION_PENALTY: -0.5,

  /** Default BLAST E-value threshold */
  DEFAULT_EVALUE: 0.001,

  /** Default minimum identity for homology */
  MIN_IDENTITY_HOMOLOGY: 30,

  /** Default minimum coverage */
  MIN_COVERAGE: 50,

  /** Default bootstrap replicates */
  BOOTSTRAP_REPLICATES: 1000,

  /** Minimum bootstrap support for reliable branch */
  MIN_BOOTSTRAP_SUPPORT: 70,

  /** P-value significance threshold */
  PVALUE_THRESHOLD: 0.05,

  /** FDR threshold for multiple testing */
  FDR_THRESHOLD: 0.05,

  /** pLDDT confidence thresholds */
  PLDDT_VERY_HIGH: 90,
  PLDDT_CONFIDENT: 70,
  PLDDT_LOW: 50,

  /** Standard genetic code (codon table 1) */
  GENETIC_CODE: {
    'TTT': 'F', 'TTC': 'F', 'TTA': 'L', 'TTG': 'L',
    'TCT': 'S', 'TCC': 'S', 'TCA': 'S', 'TCG': 'S',
    'TAT': 'Y', 'TAC': 'Y', 'TAA': '*', 'TAG': '*',
    'TGT': 'C', 'TGC': 'C', 'TGA': '*', 'TGG': 'W',
    'CTT': 'L', 'CTC': 'L', 'CTA': 'L', 'CTG': 'L',
    'CCT': 'P', 'CCC': 'P', 'CCA': 'P', 'CCG': 'P',
    'CAT': 'H', 'CAC': 'H', 'CAA': 'Q', 'CAG': 'Q',
    'CGT': 'R', 'CGC': 'R', 'CGA': 'R', 'CGG': 'R',
    'ATT': 'I', 'ATC': 'I', 'ATA': 'I', 'ATG': 'M',
    'ACT': 'T', 'ACC': 'T', 'ACA': 'T', 'ACG': 'T',
    'AAT': 'N', 'AAC': 'N', 'AAA': 'K', 'AAG': 'K',
    'AGT': 'S', 'AGC': 'S', 'AGA': 'R', 'AGG': 'R',
    'GTT': 'V', 'GTC': 'V', 'GTA': 'V', 'GTG': 'V',
    'GCT': 'A', 'GCC': 'A', 'GCA': 'A', 'GCG': 'A',
    'GAT': 'D', 'GAC': 'D', 'GAA': 'E', 'GAG': 'E',
    'GGT': 'G', 'GGC': 'G', 'GGA': 'G', 'GGG': 'G',
  } as const,

  /** Amino acid molecular weights (Da) */
  AMINO_ACID_WEIGHTS: {
    'A': 89.1, 'R': 174.2, 'N': 132.1, 'D': 133.1, 'C': 121.2,
    'Q': 146.2, 'E': 147.1, 'G': 75.1, 'H': 155.2, 'I': 131.2,
    'L': 131.2, 'K': 146.2, 'M': 149.2, 'F': 165.2, 'P': 115.1,
    'S': 105.1, 'T': 119.1, 'W': 204.2, 'Y': 181.2, 'V': 117.1,
  } as const,

  /** BLOSUM62 matrix (simplified - full matrix would be 20x20) */
  BLOSUM62_MATCH: 4,
  BLOSUM62_MISMATCH: -3,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-BIO-007 error codes
 */
export enum BioErrorCode {
  INVALID_SEQUENCE = 'B001',
  INVALID_FORMAT = 'B002',
  ALIGNMENT_FAILED = 'B003',
  DATABASE_ERROR = 'B004',
  INVALID_PARAMETERS = 'B005',
  COMPUTATION_ERROR = 'B006',
  FILE_NOT_FOUND = 'B007',
  PIPELINE_FAILED = 'B008',
}

/**
 * Bioinformatics error
 */
export class BioinformaticsError extends Error {
  constructor(
    public code: BioErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'BioinformaticsError';
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
 * Sequence format type
 */
export type SequenceFormat = 'fasta' | 'genbank' | 'pdb' | 'fastq' | 'vcf';

/**
 * Analysis type
 */
export type AnalysisType =
  | 'alignment'
  | 'blast-search'
  | 'phylogeny'
  | 'pathway-enrichment'
  | 'structure-prediction'
  | 'variant-calling';

// ============================================================================
// Export All
// ============================================================================

export type {
  // Sequences
  Sequence,
  Alignment,
  AlignmentScore,

  // Database search
  DatabaseSearch,
  SearchHit,
  SearchResults,

  // Phylogenetics
  PhylogeneticNode,
  PhylogeneticTree,
  DistanceMatrix,

  // Pathways and GO
  GeneOntology,
  Pathway,
  PathwayEnrichment,

  // Machine learning
  StructurePrediction,
  VariantEffect,

  // Pipelines
  PipelineStep,
  AnalysisPipeline,
  ComputeJob,
  ResultSet,

  // Formats
  SequenceFormat,
  AnalysisType,
};

export { BIOINFORMATICS_CONSTANTS, BioErrorCode, BioinformaticsError };
