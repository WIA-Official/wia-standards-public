/**
 * WIA-BIO-002: Genome Sequencing SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bioinformatics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for genome sequencing including:
 * - Coverage calculation and analysis
 * - Quality control and validation
 * - Variant calling and annotation
 * - Clinical report generation
 */

import {
  CoverageParameters,
  CoverageStats,
  QualityValidation,
  QualityResult,
  QualityCheck,
  VariantCall,
  VariantAnnotation,
  AnalysisReport,
  SampleMetadata,
  SequencingRun,
  SEQUENCING_CONSTANTS,
  BioErrorCode,
  GenomeSequencingError,
  SequenceRead,
  QualityScore,
  VariantCallingParams,
  AnnotationParams,
  ReferenceGenome,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-002 Genome Sequencing SDK
 */
export class GenomeSequencingSDK {
  private version = '1.0.0';
  private initialized = false;

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Calculate sequencing coverage statistics
   *
   * @param params - Coverage calculation parameters
   * @returns Coverage statistics and quality assessment
   */
  calculateCoverage(params: CoverageParameters): CoverageStats {
    const {
      totalReads,
      readLength,
      genomeSize,
      pairedEnd = false,
      duplicationRate = 0.15,
    } = params;

    // Validate inputs
    if (totalReads <= 0) {
      throw new GenomeSequencingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Total reads must be positive'
      );
    }

    if (readLength <= 0) {
      throw new GenomeSequencingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Read length must be positive'
      );
    }

    if (genomeSize <= 0) {
      throw new GenomeSequencingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Genome size must be positive'
      );
    }

    // Calculate effective read length
    const effectiveReadLength = pairedEnd ? readLength * 2 : readLength;

    // Calculate total bases sequenced
    const totalBases = totalReads * effectiveReadLength;

    // Account for duplicates
    const uniqueBases = totalBases * (1 - duplicationRate);

    // Calculate average depth
    const averageDepth = uniqueBases / genomeSize;

    // Estimate median (typically ~0.9 of mean for good uniformity)
    const medianDepth = averageDepth * 0.9;

    // Estimate standard deviation (CV ~0.2-0.3 for good runs)
    const coefficientOfVariation = 0.25;
    const stdDeviation = averageDepth * coefficientOfVariation;

    // Calculate uniformity (bases at ≥0.2× mean coverage)
    // Higher coverage = better uniformity
    const uniformity = Math.min(0.98, 0.7 + averageDepth / 100);

    // Calculate coverage at different thresholds
    // These are estimates based on Poisson distribution
    const coverage1x = this.calculatePoissonCoverage(averageDepth, 1);
    const coverage10x = this.calculatePoissonCoverage(averageDepth, 10);
    const coverage20x = this.calculatePoissonCoverage(averageDepth, 20);
    const coverage30x = this.calculatePoissonCoverage(averageDepth, 30);

    // Determine feasibility
    let feasibility: CoverageStats['feasibility'];
    if (averageDepth >= 40) {
      feasibility = 'excellent';
    } else if (averageDepth >= 30) {
      feasibility = 'good';
    } else if (averageDepth >= 20) {
      feasibility = 'acceptable';
    } else {
      feasibility = 'insufficient';
    }

    return {
      averageDepth,
      medianDepth,
      stdDeviation,
      uniformity,
      totalBases: uniqueBases,
      coverage1x,
      coverage10x,
      coverage20x,
      coverage30x,
      feasibility,
    };
  }

  /**
   * Validate sequencing quality metrics
   *
   * @param validation - Quality metrics to validate
   * @returns Quality assessment with pass/fail status
   */
  validateQuality(validation: QualityValidation): QualityResult {
    const {
      q30Percentage,
      meanCoverage,
      coverageUniformity,
      mappingRate,
      duplicationRate,
      contaminationRate,
      gcBias = 0,
      insertSizeMean,
      insertSizeStd,
    } = validation;

    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];
    const checks: QualityCheck[] = [];

    // Q30 Score Check
    const q30Check: QualityCheck = {
      name: 'Q30 Percentage',
      status: 'pass',
      value: q30Percentage,
      threshold: SEQUENCING_CONSTANTS.CLINICAL_MIN_Q30,
      description: 'Percentage of bases with Phred score ≥30',
    };

    if (q30Percentage < SEQUENCING_CONSTANTS.RESEARCH_MIN_Q30) {
      q30Check.status = 'fail';
      errors.push(
        `Q30 percentage too low: ${q30Percentage.toFixed(1)}% (minimum: ${SEQUENCING_CONSTANTS.RESEARCH_MIN_Q30}%)`
      );
      q30Check.correctiveAction = 'Check sequencing chemistry, cluster density, or library quality';
    } else if (q30Percentage < SEQUENCING_CONSTANTS.CLINICAL_MIN_Q30) {
      q30Check.status = 'warn';
      warnings.push(
        `Q30 percentage below clinical grade: ${q30Percentage.toFixed(1)}% (recommended: ≥${SEQUENCING_CONSTANTS.CLINICAL_MIN_Q30}%)`
      );
      recommendations.push('Consider re-sequencing for clinical applications');
    }
    checks.push(q30Check);

    // Coverage Check
    const coverageCheck: QualityCheck = {
      name: 'Mean Coverage',
      status: 'pass',
      value: meanCoverage,
      threshold: SEQUENCING_CONSTANTS.CLINICAL_MIN_COVERAGE,
      description: 'Average sequencing depth across genome',
    };

    if (meanCoverage < SEQUENCING_CONSTANTS.RESEARCH_MIN_COVERAGE) {
      coverageCheck.status = 'fail';
      errors.push(
        `Insufficient coverage: ${meanCoverage.toFixed(1)}× (minimum: ${SEQUENCING_CONSTANTS.RESEARCH_MIN_COVERAGE}×)`
      );
      coverageCheck.correctiveAction = 'Increase sequencing depth or pool more reads';
    } else if (meanCoverage < SEQUENCING_CONSTANTS.CLINICAL_MIN_COVERAGE) {
      coverageCheck.status = 'warn';
      warnings.push(
        `Coverage below clinical grade: ${meanCoverage.toFixed(1)}× (recommended: ≥${SEQUENCING_CONSTANTS.CLINICAL_MIN_COVERAGE}×)`
      );
    }
    checks.push(coverageCheck);

    // Uniformity Check
    const uniformityCheck: QualityCheck = {
      name: 'Coverage Uniformity',
      status: 'pass',
      value: coverageUniformity,
      threshold: SEQUENCING_CONSTANTS.MIN_UNIFORMITY,
      description: 'Evenness of coverage across genome',
    };

    if (coverageUniformity < 0.85) {
      uniformityCheck.status = 'fail';
      errors.push(
        `Poor coverage uniformity: ${(coverageUniformity * 100).toFixed(1)}% (minimum: 85%)`
      );
      uniformityCheck.correctiveAction = 'Check library preparation or GC bias';
    } else if (coverageUniformity < SEQUENCING_CONSTANTS.MIN_UNIFORMITY) {
      uniformityCheck.status = 'warn';
      warnings.push(
        `Coverage uniformity below optimal: ${(coverageUniformity * 100).toFixed(1)}% (recommended: ≥${SEQUENCING_CONSTANTS.MIN_UNIFORMITY * 100}%)`
      );
    }
    checks.push(uniformityCheck);

    // Mapping Rate Check
    const mappingCheck: QualityCheck = {
      name: 'Mapping Rate',
      status: 'pass',
      value: mappingRate,
      threshold: SEQUENCING_CONSTANTS.MIN_MAPPING_RATE,
      description: 'Percentage of reads successfully mapped',
    };

    if (mappingRate < 0.9) {
      mappingCheck.status = 'fail';
      errors.push(
        `Low mapping rate: ${(mappingRate * 100).toFixed(1)}% (minimum: 90%)`
      );
      mappingCheck.correctiveAction = 'Check reference genome or sample contamination';
    } else if (mappingRate < SEQUENCING_CONSTANTS.MIN_MAPPING_RATE) {
      mappingCheck.status = 'warn';
      warnings.push(
        `Mapping rate below optimal: ${(mappingRate * 100).toFixed(1)}% (recommended: ≥${SEQUENCING_CONSTANTS.MIN_MAPPING_RATE * 100}%)`
      );
    }
    checks.push(mappingCheck);

    // Duplication Rate Check
    const duplicationCheck: QualityCheck = {
      name: 'Duplication Rate',
      status: 'pass',
      value: duplicationRate,
      threshold: SEQUENCING_CONSTANTS.MAX_DUPLICATION_RATE,
      description: 'Percentage of duplicate reads',
    };

    if (duplicationRate > 0.3) {
      duplicationCheck.status = 'fail';
      errors.push(
        `High duplication rate: ${(duplicationRate * 100).toFixed(1)}% (maximum: 30%)`
      );
      duplicationCheck.correctiveAction = 'Increase DNA input amount or reduce PCR cycles';
    } else if (duplicationRate > SEQUENCING_CONSTANTS.MAX_DUPLICATION_RATE) {
      duplicationCheck.status = 'warn';
      warnings.push(
        `Duplication rate elevated: ${(duplicationRate * 100).toFixed(1)}% (recommended: <${SEQUENCING_CONSTANTS.MAX_DUPLICATION_RATE * 100}%)`
      );
    }
    checks.push(duplicationCheck);

    // Contamination Check
    const contaminationCheck: QualityCheck = {
      name: 'Contamination Rate',
      status: 'pass',
      value: contaminationRate,
      threshold: SEQUENCING_CONSTANTS.MAX_CONTAMINATION,
      description: 'Estimated sample contamination',
    };

    if (contaminationRate > 0.02) {
      contaminationCheck.status = 'fail';
      errors.push(
        `High contamination: ${(contaminationRate * 100).toFixed(2)}% (maximum: 2%)`
      );
      contaminationCheck.correctiveAction = 'Validate sample identity and check for cross-contamination';
    } else if (contaminationRate > SEQUENCING_CONSTANTS.MAX_CONTAMINATION) {
      contaminationCheck.status = 'warn';
      warnings.push(
        `Contamination detected: ${(contaminationRate * 100).toFixed(2)}% (recommended: <${SEQUENCING_CONSTANTS.MAX_CONTAMINATION * 100}%)`
      );
    }
    checks.push(contaminationCheck);

    // GC Bias Check (if provided)
    if (gcBias !== undefined && gcBias !== 0) {
      const gcBiasCheck: QualityCheck = {
        name: 'GC Bias',
        status: 'pass',
        value: Math.abs(gcBias),
        threshold: 0.05,
        description: 'GC content bias in coverage',
      };

      if (Math.abs(gcBias) > 0.1) {
        gcBiasCheck.status = 'fail';
        errors.push(`High GC bias: ${(gcBias * 100).toFixed(1)}% (maximum: 10%)`);
        gcBiasCheck.correctiveAction = 'Optimize library preparation or use PCR-free method';
      } else if (Math.abs(gcBias) > 0.05) {
        gcBiasCheck.status = 'warn';
        warnings.push(`GC bias detected: ${(gcBias * 100).toFixed(1)}%`);
      }
      checks.push(gcBiasCheck);
    }

    // Insert Size Check (if paired-end)
    if (insertSizeMean !== undefined && insertSizeStd !== undefined) {
      const expectedMean = 350; // Typical insert size
      const insertSizeCheck: QualityCheck = {
        name: 'Insert Size',
        status: 'pass',
        value: insertSizeMean,
        threshold: expectedMean,
        description: 'Mean fragment insert size',
      };

      if (insertSizeMean < 200 || insertSizeMean > 600) {
        insertSizeCheck.status = 'warn';
        warnings.push(
          `Unusual insert size: ${insertSizeMean.toFixed(0)} bp (typical: 300-400 bp)`
        );
      }

      if (insertSizeStd > 100) {
        warnings.push(
          `High insert size variation: σ=${insertSizeStd.toFixed(0)} bp`
        );
      }
      checks.push(insertSizeCheck);
    }

    // Determine overall grade
    let grade: QualityResult['grade'];
    if (errors.length > 0) {
      grade = 'failed';
    } else if (
      q30Percentage >= SEQUENCING_CONSTANTS.CLINICAL_MIN_Q30 &&
      meanCoverage >= SEQUENCING_CONSTANTS.CLINICAL_MIN_COVERAGE &&
      coverageUniformity >= SEQUENCING_CONSTANTS.MIN_UNIFORMITY &&
      mappingRate >= SEQUENCING_CONSTANTS.MIN_MAPPING_RATE &&
      duplicationRate <= SEQUENCING_CONSTANTS.MAX_DUPLICATION_RATE &&
      contaminationRate <= SEQUENCING_CONSTANTS.MAX_CONTAMINATION
    ) {
      grade = 'clinical';
    } else {
      grade = 'research';
    }

    // Calculate overall score (0-100)
    const overallScore = this.calculateQualityScore({
      q30Percentage,
      meanCoverage,
      coverageUniformity,
      mappingRate,
      duplicationRate,
      contaminationRate,
    });

    // Generate recommendations
    if (grade === 'research' && warnings.length > 0) {
      recommendations.push('Quality meets research standards but not clinical grade');
      recommendations.push('Consider optimization for clinical applications');
    }

    if (duplicationRate > 0.15) {
      recommendations.push('High duplication - consider increasing DNA input');
    }

    if (meanCoverage < 30) {
      recommendations.push('Increase sequencing depth for better variant detection');
    }

    return {
      isValid: errors.length === 0,
      grade,
      errors,
      warnings,
      recommendations,
      checks,
      overallScore,
    };
  }

  /**
   * Call variants from sequencing data (simulation)
   *
   * @param params - Variant calling parameters
   * @returns Array of called variants
   */
  callVariants(params: VariantCallingParams): VariantCall[] {
    // In a real implementation, this would interface with GATK, FreeBayes, etc.
    // For this SDK, we'll provide a simulation/demonstration

    const { minDepth, minQuality, minVAF } = params;

    // Validate parameters
    if (minDepth < 1) {
      throw new GenomeSequencingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Minimum depth must be at least 1'
      );
    }

    // Simulate some example variants
    const variants: VariantCall[] = [
      {
        chromosome: 'chr1',
        position: 123456,
        reference: 'A',
        alternative: 'G',
        type: 'SNP',
        quality: 99.5,
        filter: 'PASS',
        depth: 45,
        vaf: 0.52,
        genotype: '0/1',
        zygosity: 'heterozygous',
        genotypeQuality: 99,
        rsId: 'rs123456',
      },
      {
        chromosome: 'chr7',
        position: 140753336,
        reference: 'A',
        alternative: 'T',
        type: 'SNP',
        quality: 100,
        filter: 'PASS',
        depth: 85,
        vaf: 0.98,
        genotype: '1/1',
        zygosity: 'homozygous',
        genotypeQuality: 99,
        rsId: 'rs113488022', // BRAF V600E region
      },
    ];

    // Filter by parameters
    return variants.filter(
      (v) =>
        v.depth >= minDepth && v.quality >= minQuality && v.vaf >= minVAF
    );
  }

  /**
   * Annotate variants with functional information (simulation)
   *
   * @param params - Annotation parameters
   * @returns Array of annotated variants
   */
  annotateVariants(params: AnnotationParams): VariantAnnotation[] {
    // In a real implementation, this would interface with VEP, ANNOVAR, etc.
    // For this SDK, we'll provide example annotations

    const annotations: VariantAnnotation[] = [
      {
        gene: {
          symbol: 'BRAF',
          geneId: 'ENSG00000157764',
          transcriptId: 'ENST00000288602',
          chromosome: 'chr7',
          start: 140719327,
          end: 140924929,
          strand: '-',
          biotype: 'protein_coding',
          description: 'B-Raf proto-oncogene, serine/threonine kinase',
        },
        consequences: ['missense_variant'],
        impact: 'MODERATE',
        proteinChange: 'p.Val600Glu',
        cdnaPosition: 1799,
        cdsPosition: 1799,
        proteinPosition: 600,
        sift: {
          score: 0.01,
          prediction: 'deleterious',
        },
        polyphen: {
          score: 0.99,
          prediction: 'probably_damaging',
        },
        caddScore: 35.0,
        gnomadFrequency: 0.00001,
        clinvarSignificance: 'pathogenic',
        cosmicId: 'COSV57014428',
      },
    ];

    return annotations;
  }

  /**
   * Generate a comprehensive analysis report
   *
   * @param sample - Sample metadata
   * @param run - Sequencing run information
   * @param coverage - Coverage statistics
   * @param quality - Quality metrics
   * @param variants - Called variants
   * @returns Complete analysis report
   */
  generateReport(
    sample: SampleMetadata,
    run: SequencingRun,
    coverage: CoverageStats,
    quality: QualityResult,
    variants: VariantCall[]
  ): AnalysisReport {
    // Count variants by type
    const variantsByType = {
      snp: variants.filter((v) => v.type === 'SNP').length,
      insertion: variants.filter((v) => v.type === 'insertion').length,
      deletion: variants.filter((v) => v.type === 'deletion').length,
      indel: variants.filter((v) => v.type === 'indel').length,
    };

    // Calculate Ti/Tv ratio (transitions/transversions)
    const tiTvRatio = this.calculateTiTvRatio(variants);

    // Calculate Het/Hom ratio
    const hetHomRatio = this.calculateHetHomRatio(variants);

    // Count novel variants (those without rsId)
    const novelVariants = variants.filter((v) => !v.rsId).length;

    // Identify clinically significant variants (high quality, good depth)
    const clinicalVariants = variants.filter(
      (v) =>
        v.quality >= 30 &&
        v.depth >= 20 &&
        v.filter === 'PASS'
    );

    return {
      reportId: `RPT-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      sample,
      run,
      quality,
      coverage,
      totalVariants: variants.length,
      variantsByType,
      clinicalVariants,
      tiTvRatio,
      hetHomRatio,
      novelVariants,
      generatedDate: new Date(),
      pipelineVersion: this.version,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Calculate Poisson probability of coverage at threshold
   */
  private calculatePoissonCoverage(meanDepth: number, threshold: number): number {
    // P(X >= k) = 1 - P(X < k)
    // Using Poisson cumulative distribution
    let prob = 0;
    for (let i = 0; i < threshold; i++) {
      prob += this.poissonPMF(meanDepth, i);
    }
    return (1 - prob) * 100; // Return as percentage
  }

  /**
   * Poisson probability mass function
   */
  private poissonPMF(lambda: number, k: number): number {
    return (Math.exp(-lambda) * Math.pow(lambda, k)) / this.factorial(k);
  }

  /**
   * Calculate factorial
   */
  private factorial(n: number): number {
    if (n <= 1) return 1;
    let result = 1;
    for (let i = 2; i <= n; i++) {
      result *= i;
    }
    return result;
  }

  /**
   * Calculate overall quality score (0-100)
   */
  private calculateQualityScore(metrics: QualityValidation): number {
    const weights = {
      q30: 0.25,
      coverage: 0.25,
      uniformity: 0.20,
      mapping: 0.15,
      duplication: 0.10,
      contamination: 0.05,
    };

    const q30Score = Math.min(100, (metrics.q30Percentage / 95) * 100);
    const coverageScore = Math.min(100, (metrics.meanCoverage / 50) * 100);
    const uniformityScore = metrics.coverageUniformity * 100;
    const mappingScore = metrics.mappingRate * 100;
    const duplicationScore = Math.max(0, (1 - metrics.duplicationRate / 0.3) * 100);
    const contaminationScore = Math.max(0, (1 - metrics.contaminationRate / 0.02) * 100);

    return (
      q30Score * weights.q30 +
      coverageScore * weights.coverage +
      uniformityScore * weights.uniformity +
      mappingScore * weights.mapping +
      duplicationScore * weights.duplication +
      contaminationScore * weights.contamination
    );
  }

  /**
   * Calculate Ti/Tv ratio (transition/transversion)
   */
  private calculateTiTvRatio(variants: VariantCall[]): number {
    const snps = variants.filter((v) => v.type === 'SNP');

    let transitions = 0;
    let transversions = 0;

    for (const snp of snps) {
      const ref = snp.reference;
      const alt = typeof snp.alternative === 'string' ? snp.alternative : snp.alternative[0];

      const isTransition =
        (ref === 'A' && alt === 'G') ||
        (ref === 'G' && alt === 'A') ||
        (ref === 'C' && alt === 'T') ||
        (ref === 'T' && alt === 'C');

      if (isTransition) {
        transitions++;
      } else {
        transversions++;
      }
    }

    return transversions > 0 ? transitions / transversions : 0;
  }

  /**
   * Calculate Het/Hom ratio (heterozygous/homozygous)
   */
  private calculateHetHomRatio(variants: VariantCall[]): number {
    const heterozygous = variants.filter((v) => v.zygosity === 'heterozygous').length;
    const homozygous = variants.filter((v) => v.zygosity === 'homozygous').length;

    return homozygous > 0 ? heterozygous / homozygous : 0;
  }

  /**
   * Convert Phred score to error probability
   */
  phredToErrorProbability(phred: number): number {
    return Math.pow(10, -phred / 10);
  }

  /**
   * Convert error probability to Phred score
   */
  errorProbabilityToPhred(errorProb: number): number {
    return -10 * Math.log10(errorProb);
  }

  /**
   * Calculate Phred score from accuracy
   */
  accuracyToPhred(accuracy: number): number {
    const errorProb = 1 - accuracy;
    return this.errorProbabilityToPhred(errorProb);
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate sequencing coverage (standalone function)
 */
export function calculateCoverage(params: CoverageParameters): CoverageStats {
  const sdk = new GenomeSequencingSDK();
  return sdk.calculateCoverage(params);
}

/**
 * Validate quality metrics (standalone function)
 */
export function validateQuality(validation: QualityValidation): QualityResult {
  const sdk = new GenomeSequencingSDK();
  return sdk.validateQuality(validation);
}

/**
 * Call variants (standalone function)
 */
export function callVariants(params: VariantCallingParams): VariantCall[] {
  const sdk = new GenomeSequencingSDK();
  return sdk.callVariants(params);
}

/**
 * Annotate variants (standalone function)
 */
export function annotateVariants(params: AnnotationParams): VariantAnnotation[] {
  const sdk = new GenomeSequencingSDK();
  return sdk.annotateVariants(params);
}

/**
 * Generate analysis report (standalone function)
 */
export function generateReport(
  sample: SampleMetadata,
  run: SequencingRun,
  coverage: CoverageStats,
  quality: QualityResult,
  variants: VariantCall[]
): AnalysisReport {
  const sdk = new GenomeSequencingSDK();
  return sdk.generateReport(sample, run, coverage, quality, variants);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { GenomeSequencingSDK };
export default GenomeSequencingSDK;
