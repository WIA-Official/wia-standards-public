/**
 * WIA-BIO-013: Microbiome SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for microbiome analysis including:
 * - Sample quality control
 * - Taxonomic classification
 * - Diversity analysis (alpha and beta)
 * - Functional annotation
 * - Clinical interpretation
 */

import {
  MicrobiomeSample,
  SequenceData,
  MicrobiomeAnalysis,
  AlphaDiversity,
  BetaDiversity,
  TaxonAbundance,
  AbundanceTable,
  QualityMetrics,
  DysbiosisIndex,
  ClinicalInterpretation,
  FMTDonor,
  FMTEfficacy,
  ComparativeAnalysis,
  AnalysisParameters,
  DiversityMetric,
  BetaDiversityMetric,
  TaxonomicRank,
  MICROBIOME_CONSTANTS,
  MicrobiomeErrorCode,
  MicrobiomeError,
  SCFAProduction,
  MetabolicPathway,
  RarefactionCurve,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-013 Microbiome SDK
 */
export class MicrobiomeSDK {
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
   * Analyze microbiome sample
   *
   * @param sample - Sample metadata and sequencing information
   * @param sequences - Sequence read data
   * @param parameters - Analysis parameters
   * @returns Complete microbiome analysis
   */
  async analyzeSample(
    sample: MicrobiomeSample,
    sequences: SequenceData[],
    parameters?: AnalysisParameters
  ): Promise<MicrobiomeAnalysis> {
    // Quality control
    const quality = this.performQualityControl(sample, sequences, parameters);

    if (quality.status === 'fail') {
      throw new MicrobiomeError(
        MicrobiomeErrorCode.LOW_QUALITY,
        'Sample failed quality control',
        { quality }
      );
    }

    // Taxonomic classification
    const taxonomy = this.classifyTaxonomy(sequences, parameters);

    // Calculate diversity
    const alpha = this.calculateAlphaDiversity(taxonomy.abundanceTable);
    const rarefaction = this.generateRarefactionCurve(
      sample.sampleId,
      taxonomy.abundanceTable
    );

    // Clinical interpretation
    const clinical = this.interpretClinical(sample, taxonomy.abundanceTable, alpha);

    return {
      analysisId: `ANALYSIS-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      sample,
      quality,
      taxonomy: {
        assignments: [],
        abundanceTable: taxonomy.abundanceTable,
        topTaxa: taxonomy.topTaxa,
      },
      diversity: {
        alpha,
        rarefaction,
      },
      clinical,
      metadata: {
        pipeline: 'WIA-BIO-013',
        version: this.version,
        database: parameters?.classification?.database || 'silva',
        analysisDate: new Date(),
        parameters,
      },
    };
  }

  /**
   * Calculate alpha diversity metrics
   *
   * @param abundanceTable - Abundance table
   * @param metric - Diversity metric to calculate
   * @returns Alpha diversity metrics
   */
  calculateAlphaDiversity(
    abundanceTable: AbundanceTable,
    metric?: DiversityMetric
  ): AlphaDiversity {
    if (abundanceTable.samples.length === 0) {
      throw new MicrobiomeError(
        MicrobiomeErrorCode.INVALID_SAMPLE,
        'Abundance table has no samples'
      );
    }

    // Use first sample for single-sample analysis
    const sampleIndex = 0;
    const sampleId = abundanceTable.samples[sampleIndex];
    const abundances = abundanceTable.matrix.map((row) => row[sampleIndex]);

    const totalReads = abundances.reduce((sum, count) => sum + count, 0);

    if (totalReads < MICROBIOME_CONSTANTS.MIN_READ_DEPTH) {
      throw new MicrobiomeError(
        MicrobiomeErrorCode.INSUFFICIENT_READS,
        `Insufficient reads: ${totalReads} < ${MICROBIOME_CONSTANTS.MIN_READ_DEPTH}`
      );
    }

    // Calculate metrics
    const shannon = this.shannonIndex(abundances);
    const simpson = this.simpsonIndex(abundances);
    const observed = abundances.filter((count) => count > 0).length;
    const chao1 = this.chao1Estimator(abundances);
    const evenness = shannon / Math.log(observed);

    return {
      sampleId,
      shannon,
      simpson,
      observed,
      chao1,
      evenness,
      totalReads,
    };
  }

  /**
   * Calculate beta diversity between samples
   *
   * @param abundanceTable - Abundance table
   * @param metric - Beta diversity metric
   * @returns Beta diversity distance matrix
   */
  calculateBetaDiversity(
    abundanceTable: AbundanceTable,
    metric: BetaDiversityMetric = 'bray-curtis'
  ): BetaDiversity {
    const numSamples = abundanceTable.samples.length;
    const distanceMatrix: number[][] = Array(numSamples)
      .fill(0)
      .map(() => Array(numSamples).fill(0));

    // Calculate pairwise distances
    for (let i = 0; i < numSamples; i++) {
      for (let j = i + 1; j < numSamples; j++) {
        const sample1 = abundanceTable.matrix.map((row) => row[i]);
        const sample2 = abundanceTable.matrix.map((row) => row[j]);

        let distance: number;
        switch (metric) {
          case 'bray-curtis':
            distance = this.brayCurtisDistance(sample1, sample2);
            break;
          case 'jaccard':
            distance = this.jaccardDistance(sample1, sample2);
            break;
          default:
            distance = this.brayCurtisDistance(sample1, sample2);
        }

        distanceMatrix[i][j] = distance;
        distanceMatrix[j][i] = distance; // Symmetric
      }
    }

    return {
      metric,
      samples: abundanceTable.samples,
      distanceMatrix,
    };
  }

  /**
   * Predict SCFA (short-chain fatty acid) production potential
   *
   * @param abundanceTable - Abundance table at genus level
   * @returns SCFA production potential
   */
  predictSCFAProduction(abundanceTable: AbundanceTable): SCFAProduction {
    if (abundanceTable.samples.length === 0) {
      throw new MicrobiomeError(
        MicrobiomeErrorCode.INVALID_SAMPLE,
        'Abundance table has no samples'
      );
    }

    const sampleId = abundanceTable.samples[0];

    // Butyrate producers
    const butyrateProducers = [
      'Faecalibacterium',
      'Roseburia',
      'Eubacterium',
      'Anaerostipes',
      'Coprococcus',
    ];

    // Acetate producers
    const acetateProducers = ['Bacteroides', 'Bifidobacterium', 'Akkermansia'];

    // Propionate producers
    const propionateProducers = [
      'Bacteroides',
      'Phascolarctobacterium',
      'Dialister',
    ];

    let butyrate = 0;
    let acetate = 0;
    let propionate = 0;

    const butyrateProducersList: { genus: string; abundance: number }[] = [];

    abundanceTable.taxa.forEach((taxon, index) => {
      const genus = this.extractGenus(taxon.taxon);
      const abundance = taxon.relativeAbundance;

      if (butyrateProducers.includes(genus)) {
        butyrate += abundance;
        butyrateProducersList.push({ genus, abundance });
      }

      if (acetateProducers.includes(genus)) {
        acetate += abundance;
      }

      if (propionateProducers.includes(genus)) {
        propionate += abundance;
      }
    });

    return {
      sampleId,
      butyrate,
      acetate,
      propionate,
      butyrateProducers: butyrateProducersList.sort((a, b) => b.abundance - a.abundance),
    };
  }

  /**
   * Assess dysbiosis index
   *
   * @param abundanceTable - Abundance table
   * @returns Dysbiosis assessment
   */
  assessDysbiosis(abundanceTable: AbundanceTable): DysbiosisIndex {
    const sampleId = abundanceTable.samples[0];

    // Pathobionts
    const pathobionts = ['Escherichia', 'Klebsiella', 'Enterococcus', 'Clostridium'];

    // Commensals
    const commensals = [
      'Faecalibacterium',
      'Akkermansia',
      'Roseburia',
      'Bifidobacterium',
      'Lactobacillus',
    ];

    let pathobiontAbundance = 0;
    let commensalAbundance = 0;

    const keyPathobionts: TaxonAbundance[] = [];
    const depletedBeneficial: TaxonAbundance[] = [];

    abundanceTable.taxa.forEach((taxon) => {
      const genus = this.extractGenus(taxon.taxon);
      const abundance = taxon.relativeAbundance;

      if (pathobionts.includes(genus)) {
        pathobiontAbundance += abundance;
        if (abundance > 0.01) {
          keyPathobionts.push(taxon);
        }
      }

      if (commensals.includes(genus)) {
        commensalAbundance += abundance;
        if (abundance < 0.02) {
          depletedBeneficial.push(taxon);
        }
      }
    });

    // Dysbiosis index: log10(pathobionts / commensals)
    const index =
      commensalAbundance > 0
        ? Math.log10(pathobiontAbundance / commensalAbundance)
        : 1.0;

    let interpretation: DysbiosisIndex['interpretation'];
    if (index < -0.5) {
      interpretation = 'healthy';
    } else if (index < 0) {
      interpretation = 'balanced';
    } else if (index < 0.3) {
      interpretation = 'mild-dysbiosis';
    } else if (index < 0.7) {
      interpretation = 'moderate-dysbiosis';
    } else {
      interpretation = 'severe-dysbiosis';
    }

    return {
      sampleId,
      index,
      interpretation,
      pathobionts: pathobiontAbundance,
      commensals: commensalAbundance,
      keyPathobionts,
      depletedBeneficial,
    };
  }

  /**
   * Predict FMT efficacy
   *
   * @param recipientPre - Recipient pre-FMT diversity
   * @param donor - Donor diversity
   * @param recipientPost - Recipient post-FMT diversity (optional)
   * @returns FMT efficacy prediction
   */
  predictFMTEfficacy(
    recipientPre: AlphaDiversity,
    donor: AlphaDiversity,
    recipientPost?: AlphaDiversity
  ): FMTEfficacy {
    // FMT score = (D_post - D_pre) / D_donor
    let fmtScore: number;
    if (recipientPost) {
      fmtScore =
        (recipientPost.shannon - recipientPre.shannon) / donor.shannon;
    } else {
      // Predict based on donor-recipient difference
      fmtScore = (donor.shannon - recipientPre.shannon) / donor.shannon;
    }

    // Predicted engraftment rate
    const predictedEngraftment = Math.min(1.0, Math.max(0, fmtScore * 0.8));

    // Efficacy prediction
    let prediction: FMTEfficacy['prediction'];
    if (fmtScore > 0.5) {
      prediction = 'excellent';
    } else if (fmtScore > 0.3) {
      prediction = 'good';
    } else if (fmtScore > 0.1) {
      prediction = 'fair';
    } else {
      prediction = 'poor';
    }

    return {
      recipientId: recipientPre.sampleId,
      donorId: donor.sampleId,
      diversityPre: recipientPre,
      diversityDonor: donor,
      diversityPost: recipientPost,
      fmtScore,
      predictedEngraftment,
      prediction,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Shannon diversity index
   */
  private shannonIndex(abundances: number[]): number {
    const total = abundances.reduce((sum, count) => sum + count, 0);
    if (total === 0) return 0;

    let shannon = 0;
    for (const count of abundances) {
      if (count > 0) {
        const proportion = count / total;
        shannon -= proportion * Math.log(proportion);
      }
    }
    return shannon;
  }

  /**
   * Simpson diversity index
   */
  private simpsonIndex(abundances: number[]): number {
    const total = abundances.reduce((sum, count) => sum + count, 0);
    if (total === 0) return 0;

    let simpson = 0;
    for (const count of abundances) {
      if (count > 0) {
        const proportion = count / total;
        simpson += proportion * proportion;
      }
    }
    return 1 - simpson;
  }

  /**
   * Chao1 richness estimator
   */
  private chao1Estimator(abundances: number[]): number {
    const observed = abundances.filter((count) => count > 0).length;
    const singletons = abundances.filter((count) => count === 1).length;
    const doubletons = abundances.filter((count) => count === 2).length;

    if (doubletons === 0) {
      return observed + (singletons * (singletons - 1)) / 2;
    }

    return observed + (singletons * singletons) / (2 * doubletons);
  }

  /**
   * Bray-Curtis dissimilarity
   */
  private brayCurtisDistance(sample1: number[], sample2: number[]): number {
    let numerator = 0;
    let denominator = 0;

    for (let i = 0; i < sample1.length; i++) {
      numerator += Math.abs(sample1[i] - sample2[i]);
      denominator += sample1[i] + sample2[i];
    }

    return denominator > 0 ? numerator / denominator : 0;
  }

  /**
   * Jaccard distance
   */
  private jaccardDistance(sample1: number[], sample2: number[]): number {
    let intersection = 0;
    let union = 0;

    for (let i = 0; i < sample1.length; i++) {
      const present1 = sample1[i] > 0;
      const present2 = sample2[i] > 0;

      if (present1 && present2) intersection++;
      if (present1 || present2) union++;
    }

    return union > 0 ? 1 - intersection / union : 0;
  }

  /**
   * Perform quality control on sequences
   */
  private performQualityControl(
    sample: MicrobiomeSample,
    sequences: SequenceData[],
    parameters?: AnalysisParameters
  ): QualityMetrics {
    const rawReads = sequences.length;
    const minQuality = parameters?.qualityFilter?.minQuality || 30;
    const minLength = parameters?.qualityFilter?.minLength || 250;

    // Filter sequences
    const filteredSequences = sequences.filter((seq) => {
      const avgQuality = this.calculateAverageQuality(seq.quality || '');
      return avgQuality >= minQuality && seq.length >= minLength;
    });

    const filteredReads = filteredSequences.length;
    const passRate = rawReads > 0 ? filteredReads / rawReads : 0;

    // Calculate average quality
    const totalQuality = filteredSequences.reduce((sum, seq) => {
      return sum + this.calculateAverageQuality(seq.quality || '');
    }, 0);
    const averageQuality =
      filteredSequences.length > 0 ? totalQuality / filteredSequences.length : 0;

    // Q30 percentage
    const q30Count = filteredSequences.filter((seq) => {
      const avgQuality = this.calculateAverageQuality(seq.quality || '');
      return avgQuality >= 30;
    }).length;
    const q30Percentage =
      filteredSequences.length > 0 ? (q30Count / filteredSequences.length) * 100 : 0;

    // Determine status
    let status: QualityMetrics['status'];
    const messages: string[] = [];

    if (
      filteredReads < MICROBIOME_CONSTANTS.MIN_READ_DEPTH ||
      q30Percentage < MICROBIOME_CONSTANTS.MIN_Q30_PERCENTAGE
    ) {
      status = 'fail';
      messages.push('Insufficient reads or low quality');
    } else if (passRate < 0.7 || q30Percentage < 85) {
      status = 'warning';
      messages.push('Quality metrics below optimal thresholds');
    } else {
      status = 'pass';
    }

    return {
      sampleId: sample.sampleId,
      rawReads,
      filteredReads,
      passRate,
      averageQuality,
      q30Percentage,
      status,
      messages,
    };
  }

  /**
   * Classify taxonomy (mock implementation)
   */
  private classifyTaxonomy(
    sequences: SequenceData[],
    parameters?: AnalysisParameters
  ): { abundanceTable: AbundanceTable; topTaxa: TaxonAbundance[] } {
    // Mock taxonomic classification
    const mockTaxa: TaxonAbundance[] = [
      {
        taxon: 'Faecalibacterium prausnitzii',
        rank: 'species',
        count: Math.floor(sequences.length * 0.08),
        relativeAbundance: 0.08,
      },
      {
        taxon: 'Bacteroides uniformis',
        rank: 'species',
        count: Math.floor(sequences.length * 0.12),
        relativeAbundance: 0.12,
      },
      {
        taxon: 'Akkermansia muciniphila',
        rank: 'species',
        count: Math.floor(sequences.length * 0.05),
        relativeAbundance: 0.05,
      },
      {
        taxon: 'Roseburia intestinalis',
        rank: 'species',
        count: Math.floor(sequences.length * 0.06),
        relativeAbundance: 0.06,
      },
      {
        taxon: 'Bifidobacterium longum',
        rank: 'species',
        count: Math.floor(sequences.length * 0.04),
        relativeAbundance: 0.04,
      },
    ];

    const abundanceTable: AbundanceTable = {
      samples: ['SAMPLE001'],
      taxa: mockTaxa,
      matrix: mockTaxa.map((t) => [t.count]),
      rank: 'species',
      totalReads: [sequences.length],
    };

    return {
      abundanceTable,
      topTaxa: mockTaxa.slice(0, 10),
    };
  }

  /**
   * Generate rarefaction curve
   */
  private generateRarefactionCurve(
    sampleId: string,
    abundanceTable: AbundanceTable
  ): RarefactionCurve {
    const maxDepth = abundanceTable.totalReads[0];
    const depths: number[] = [];
    const observedSpecies: number[] = [];

    // Generate curve points
    for (let d = 1000; d <= maxDepth; d += Math.floor(maxDepth / 10)) {
      depths.push(d);
      // Simplified: actual implementation would use rarefaction
      const observed = Math.min(
        abundanceTable.taxa.length,
        Math.floor(Math.log(d) * 5)
      );
      observedSpecies.push(observed);
    }

    return {
      sampleId,
      depths,
      observedSpecies,
    };
  }

  /**
   * Clinical interpretation
   */
  private interpretClinical(
    sample: MicrobiomeSample,
    abundanceTable: AbundanceTable,
    diversity: AlphaDiversity
  ): ClinicalInterpretation {
    const dysbiosis = this.assessDysbiosis(abundanceTable);
    const scfa = this.predictSCFAProduction(abundanceTable);

    // Health score (0-100)
    let healthScore = 50;
    healthScore += Math.min(25, (diversity.shannon / 5) * 25);
    healthScore -= Math.abs(dysbiosis.index) * 20;
    healthScore = Math.max(0, Math.min(100, healthScore));

    const findings = [];
    const recommendations = [];

    // Diversity findings
    if (diversity.shannon < MICROBIOME_CONSTANTS.HEALTHY_GUT_SHANNON[0]) {
      findings.push({
        category: 'negative' as const,
        description: `Low diversity (Shannon: ${diversity.shannon.toFixed(2)})`,
        severity: 'medium' as const,
      });
      recommendations.push({
        type: 'dietary' as const,
        recommendation: 'Increase dietary fiber intake (25-30g/day)',
        priority: 'high' as const,
      });
    }

    // Dysbiosis findings
    if (dysbiosis.interpretation !== 'healthy') {
      findings.push({
        category: 'negative' as const,
        description: `${dysbiosis.interpretation} detected`,
        severity: 'medium' as const,
      });
    }

    // Butyrate findings
    if (scfa.butyrate < 0.05) {
      findings.push({
        category: 'negative' as const,
        description: 'Low butyrate production potential',
        severity: 'low' as const,
      });
      recommendations.push({
        type: 'probiotic' as const,
        recommendation: 'Consider Faecalibacterium prausnitzii supplementation',
        priority: 'medium' as const,
      });
    }

    return {
      sampleId: sample.sampleId,
      healthScore,
      dysbiosis,
      findings,
      recommendations,
    };
  }

  /**
   * Calculate average quality score from Phred33 string
   */
  private calculateAverageQuality(qualityString: string): number {
    if (!qualityString || qualityString.length === 0) return 0;

    let totalQuality = 0;
    for (const char of qualityString) {
      totalQuality += char.charCodeAt(0) - 33; // Phred33 encoding
    }
    return totalQuality / qualityString.length;
  }

  /**
   * Extract genus from taxonomic string
   */
  private extractGenus(taxonomy: string): string {
    const parts = taxonomy.split(/[; ]/);
    for (const part of parts) {
      if (part.startsWith('g__')) {
        return part.substring(3);
      }
    }
    // If no prefix, assume it's genus or species
    const words = taxonomy.split(' ');
    return words[0];
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate alpha diversity (standalone function)
 */
export function calculateAlphaDiversity(
  abundanceTable: AbundanceTable,
  metric?: DiversityMetric
): AlphaDiversity {
  const sdk = new MicrobiomeSDK();
  return sdk.calculateAlphaDiversity(abundanceTable, metric);
}

/**
 * Calculate beta diversity (standalone function)
 */
export function calculateBetaDiversity(
  abundanceTable: AbundanceTable,
  metric?: BetaDiversityMetric
): BetaDiversity {
  const sdk = new MicrobiomeSDK();
  return sdk.calculateBetaDiversity(abundanceTable, metric);
}

/**
 * Assess dysbiosis (standalone function)
 */
export function assessDysbiosis(abundanceTable: AbundanceTable): DysbiosisIndex {
  const sdk = new MicrobiomeSDK();
  return sdk.assessDysbiosis(abundanceTable);
}

/**
 * Predict SCFA production (standalone function)
 */
export function predictSCFAProduction(
  abundanceTable: AbundanceTable
): SCFAProduction {
  const sdk = new MicrobiomeSDK();
  return sdk.predictSCFAProduction(abundanceTable);
}

/**
 * Predict FMT efficacy (standalone function)
 */
export function predictFMTEfficacy(
  recipientPre: AlphaDiversity,
  donor: AlphaDiversity,
  recipientPost?: AlphaDiversity
): FMTEfficacy {
  const sdk = new MicrobiomeSDK();
  return sdk.predictFMTEfficacy(recipientPre, donor, recipientPost);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MicrobiomeSDK };
export default MicrobiomeSDK;
