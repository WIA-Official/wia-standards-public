/**
 * WIA Smart Breeding Data Format Standard - TypeScript Types
 * @version 1.0.0
 * @license MIT
 */

/**
 * Species supported by the breeding system
 */
export type Species = 'CATTLE' | 'PIG' | 'CHICKEN' | 'RICE' | 'CORN' | 'WHEAT';

/**
 * Sex of the individual
 */
export type Sex = 'MALE' | 'FEMALE';

/**
 * Status of the individual
 */
export type Status = 'ALIVE' | 'DECEASED' | 'CULLED';

/**
 * SNP genotype encoding
 * 0: Homozygous for major allele (AA)
 * 1: Heterozygous (AB)
 * 2: Homozygous for minor allele (BB)
 * -9: Missing genotype
 */
export type Genotype = 0 | 1 | 2 | -9 | null;

/**
 * Data provider information
 */
export interface DataProvider {
  organization: string;
  contact: string;
  country: string;
}

/**
 * Pedigree information
 */
export interface Pedigree {
  sire_id?: string;
  dam_id?: string;
  paternal_grandsire?: string;
  paternal_granddam?: string;
  maternal_grandsire?: string;
  maternal_granddam?: string;
  inbreeding_coefficient: number;
}

/**
 * Pedigree completeness metrics
 */
export interface PedigreeCompleteness {
  generation_1: number;
  generation_2: number;
  generation_3: number;
  max_generation: number;
}

/**
 * SNP marker information
 */
export interface Marker {
  marker_id: string;
  rsid?: string;
  chromosome: number;
  position: number;
  genotype: Genotype;
  allele_A: string;
  allele_B: string;
  allele_frequency?: number;
  imputation_quality?: number;
}

/**
 * Genotyping data
 */
export interface GenotypingData {
  individual_id: string;
  platform: string;
  chip_version?: string;
  total_markers: number;
  call_rate: number;
  genotyping_date: string;
  markers: Marker[];
}

/**
 * Genomic relationship matrix (G-Matrix)
 */
export interface GMatrix {
  type: string;
  dimension: number;
  individuals: string[];
  matrix: number[][];
  computed_date: string;
  snp_count: number;
  maf_threshold: number;
}

/**
 * Sequence data reference
 */
export interface SequenceData {
  individual_id: string;
  sequencing_platform: string;
  coverage: string;
  file_format: 'VCF' | 'BAM' | 'CRAM';
  file_url: string;
  reference_genome: string;
  variant_count: number;
  quality_score: number;
}

/**
 * Trait measurement
 */
export interface TraitMeasurement {
  trait_code: string;
  trait_name?: string;
  value: number;
  unit: string;
  measurement_date: string;
  lactation_number?: number;
  contemporary_group?: string;
}

/**
 * Environmental effects
 */
export interface EnvironmentalEffects {
  herd?: string;
  season?: string;
  year?: number;
  management_group?: string;
}

/**
 * Phenotype data
 */
export interface PhenotypeData {
  individual_id: string;
  measurements: TraitMeasurement[];
  environmental_effects?: EnvironmentalEffects;
}

/**
 * Breeding value trait
 */
export interface BreedingValueTrait {
  trait_code: string;
  ebv?: number;
  gebv?: number;
  accuracy: number;
  reliability?: number;
  percentile?: number;
  genetic_trend?: string;
}

/**
 * Traditional Estimated Breeding Value (EBV)
 */
export interface BreedingValue {
  individual_id: string;
  method: 'BLUP' | 'GBLUP';
  evaluation_date: string;
  traits: BreedingValueTrait[];
}

/**
 * SNP effects for genomic breeding value
 */
export interface SNPEffects {
  total_markers_used: number;
  significant_qtl: number;
  explained_variance: number;
}

/**
 * Genomic Estimated Breeding Value (GEBV)
 */
export interface GenomicBreedingValue extends BreedingValue {
  method: 'GBLUP';
  reference_population_size: number;
  snp_effects?: SNPEffects;
}

/**
 * Selection index component
 */
export interface SelectionIndexComponent {
  trait: string;
  weight: number;
  contribution: number;
}

/**
 * Selection index
 */
export interface SelectionIndex {
  index_name: string;
  individual_id: string;
  total_index: number;
  components: SelectionIndexComponent[];
  economic_value?: string;
}

/**
 * Genetic diversity metrics
 */
export interface GeneticDiversityMetrics {
  observed_heterozygosity: number;
  expected_heterozygosity: number;
  inbreeding_rate_per_generation: number;
  genetic_drift: number;
}

/**
 * Population genetic diversity
 */
export interface GeneticDiversity {
  population_name: string;
  population_size: number;
  effective_population_size: number;
  metrics: GeneticDiversityMetrics;
  diversity_status: string;
  recommendation?: string;
}

/**
 * Genetic distance matrix
 */
export interface GeneticDistance {
  method: string;
  breeds: string[];
  distance_matrix: number[][];
}

/**
 * Individual breeding record
 */
export interface Individual {
  individual_id: string;
  species: Species;
  breed: string;
  sex: Sex;
  birth_date: string;
  status: Status;
  pedigree?: Pedigree;
  genomic_data?: GenotypingData;
  phenotype_records?: PhenotypeData[];
  breeding_values?: BreedingValue;
}

/**
 * Root breeding data structure
 */
export interface BreedingData {
  '@context': string;
  '@type': 'BreedingData';
  version: string;
  species: Species;
  breed_variety: string;
  data_provider: DataProvider;
  timestamp: string;
  data: {
    genomic?: GenotypingData[];
    pedigree?: Pedigree[];
    phenotype?: PhenotypeData[];
    breeding_values?: BreedingValue[];
  };
}

/**
 * API Response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

/**
 * Query parameters for fetching breeding data
 */
export interface BreedingDataQuery {
  species?: Species;
  breed?: string;
  individual_id?: string;
  limit?: number;
  offset?: number;
}
