/**
 * WIA Bio Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-bio
 */

export enum SequenceType {
  DNA = 'dna',
  RNA = 'rna',
  Protein = 'protein',
  Peptide = 'peptide'
}

export enum OrganismType {
  Human = 'human',
  Animal = 'animal',
  Plant = 'plant',
  Bacteria = 'bacteria',
  Virus = 'virus',
  Fungi = 'fungi',
  Archaea = 'archaea'
}

export enum ExperimentType {
  WGS = 'whole_genome_sequencing',
  RNASeq = 'rna_sequencing',
  Proteomics = 'proteomics',
  Metabolomics = 'metabolomics',
  Epigenomics = 'epigenomics',
  CRISPR = 'crispr',
  SingleCell = 'single_cell'
}

export enum DataFormat {
  FASTA = 'fasta',
  FASTQ = 'fastq',
  SAM = 'sam',
  BAM = 'bam',
  VCF = 'vcf',
  GFF = 'gff',
  BED = 'bed',
  GenBank = 'genbank'
}

export enum QualityLevel {
  Raw = 'raw',
  Filtered = 'filtered',
  Validated = 'validated',
  Curated = 'curated'
}

export interface Organism {
  id: string;
  taxonId: number;
  scientificName: string;
  commonName?: string;
  type: OrganismType;
  strain?: string;
  lineage: string[];
}

export interface Sample {
  id: string;
  name: string;
  organism: Organism;
  sampleType: string;
  collectionDate: Date;
  location?: string;
  metadata: Record<string, unknown>;
}

export interface Sequence {
  id: string;
  accession?: string;
  type: SequenceType;
  sequence: string;
  length: number;
  organism: Organism;
  description?: string;
  quality?: SequenceQuality;
  annotations: Annotation[];
  features: Feature[];
  metadata: Record<string, unknown>;
}

export interface SequenceQuality {
  level: QualityLevel;
  phredScores?: number[];
  meanQuality: number;
  gcContent: number;
  nContent: number;
}

export interface Annotation {
  id: string;
  type: string;
  start: number;
  end: number;
  strand: '+' | '-';
  attributes: Record<string, unknown>;
}

export interface Feature {
  id: string;
  type: 'gene' | 'exon' | 'intron' | 'promoter' | 'cds' | 'utr' | 'repeat' | 'binding_site';
  name: string;
  start: number;
  end: number;
  strand: '+' | '-';
  product?: string;
  function?: string;
}

export interface Variant {
  id: string;
  chromosome: string;
  position: number;
  reference: string;
  alternate: string[];
  type: 'snv' | 'insertion' | 'deletion' | 'mnv' | 'sv';
  quality: number;
  filter: string;
  info: Record<string, unknown>;
  clinicalSignificance?: ClinicalSignificance;
}

export interface ClinicalSignificance {
  significance: 'pathogenic' | 'likely_pathogenic' | 'uncertain' | 'likely_benign' | 'benign';
  condition?: string;
  inheritance?: string;
  evidence: string[];
}

export interface Alignment {
  id: string;
  queryId: string;
  targetId: string;
  queryStart: number;
  queryEnd: number;
  targetStart: number;
  targetEnd: number;
  score: number;
  identity: number;
  coverage: number;
  eValue: number;
  cigar?: string;
}

export interface Protein {
  id: string;
  accession: string;
  name: string;
  sequence: string;
  length: number;
  organism: Organism;
  function?: string;
  domains: ProteinDomain[];
  structure?: ProteinStructure;
}

export interface ProteinDomain {
  id: string;
  name: string;
  accession: string;
  start: number;
  end: number;
  score: number;
}

export interface ProteinStructure {
  pdbId?: string;
  method: 'xray' | 'nmr' | 'cryo_em' | 'predicted';
  resolution?: number;
  coordinates?: string;
}

export interface Experiment {
  id: string;
  name: string;
  type: ExperimentType;
  samples: Sample[];
  protocol: string;
  platform?: string;
  results: ExperimentResult[];
  status: 'pending' | 'running' | 'completed' | 'failed';
  startDate: Date;
  endDate?: Date;
}

export interface ExperimentResult {
  id: string;
  type: string;
  data: unknown;
  format: DataFormat;
  qualityMetrics: Record<string, number>;
  timestamp: Date;
}

export interface Gene {
  id: string;
  symbol: string;
  name: string;
  description?: string;
  chromosome: string;
  start: number;
  end: number;
  strand: '+' | '-';
  organism: Organism;
  transcripts: Transcript[];
  ontology?: GeneOntology[];
}

export interface Transcript {
  id: string;
  geneId: string;
  type: 'mrna' | 'ncrna' | 'trna' | 'rrna';
  exons: Exon[];
  cdsStart?: number;
  cdsEnd?: number;
}

export interface Exon {
  number: number;
  start: number;
  end: number;
}

export interface GeneOntology {
  id: string;
  term: string;
  namespace: 'biological_process' | 'molecular_function' | 'cellular_component';
  evidence: string;
}

export interface Pathway {
  id: string;
  name: string;
  description?: string;
  genes: string[];
  database: 'kegg' | 'reactome' | 'wikipathways';
  organism: Organism;
}

export interface BioConfig {
  apiEndpoint: string;
  apiKey?: string;
  defaultOrganism?: Organism;
  enableQualityChecks: boolean;
  maxSequenceLength: number;
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-BIO';
  testDate: string;
  config: BioConfig;
  targetLevel: CertificationLevel;
  tests: TestResult[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

export interface TestResult {
  testName: string;
  passed: boolean;
  notes?: string;
}
