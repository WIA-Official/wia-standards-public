/**
 * WIA-BIO-014: CRISPR Protocol SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for CRISPR genome editing including:
 * - Guide RNA design with on-target scoring
 * - Off-target prediction and analysis
 * - Editing efficiency calculations
 * - Protocol validation and safety checks
 * - Comprehensive CRISPR workflow support
 */

import {
  GuideRNARequest,
  GuideRNA,
  OffTargetRequest,
  OffTargetSite,
  EfficiencyRequest,
  EfficiencyResponse,
  CRISPRProtocol,
  ProtocolValidation,
  ValidationResult,
  EditingOutcomes,
  IndelAllele,
  EditingSimulation,
  PredictedOutcomes,
  CRISPR_CONSTANTS,
  CRISPRErrorCode,
  CRISPRProtocolError,
  GenomicCoordinates,
  OffTargetAnnotation,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-014 CRISPR Protocol SDK
 */
export class CRISPRProtocolSDK {
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
   * Design guide RNA for a target sequence
   *
   * @param request - Guide RNA design parameters
   * @returns Designed guide RNA with scores
   */
  designGuideRNA(request: GuideRNARequest): GuideRNA {
    const { targetSequence, pamType, organism } = request;

    // Validate input
    if (!targetSequence || targetSequence.length < 20) {
      throw new CRISPRProtocolError(
        CRISPRErrorCode.INVALID_SEQUENCE,
        'Target sequence must be at least 20bp'
      );
    }

    // Extract PAM and protospacer based on PAM type
    const { sequence, pam, fullTarget } = this.extractSequenceAndPAM(
      targetSequence,
      pamType
    );

    // Calculate GC content
    const gcContent = this.calculateGCContent(sequence);

    // Check sequence constraints
    const warnings = this.checkSequenceConstraints(sequence, gcContent);

    // Calculate on-target score (Doench-like algorithm)
    const onTargetScore = this.calculateOnTargetScore(sequence, gcContent);

    // Predict editing efficiency
    const predictedEfficiency = this.predictEditingEfficiency(
      onTargetScore,
      gcContent
    );

    // Calculate specificity score
    const specificityScore = this.calculateSpecificityScore(sequence);

    // Generate genomic coordinates if provided
    let coordinates: GenomicCoordinates | undefined;
    if (request.chromosome && request.position !== undefined) {
      coordinates = {
        chromosome: request.chromosome,
        start: request.position,
        end: request.position + fullTarget.length,
        strand: request.strand || '+',
        assembly: this.getAssemblyForOrganism(organism),
      };
    }

    return {
      sequence,
      pam,
      fullTarget,
      onTargetScore,
      gcContent,
      predictedEfficiency,
      specificityScore,
      warnings,
      coordinates,
    };
  }

  /**
   * Predict off-target sites for a guide RNA
   *
   * @param request - Off-target prediction parameters
   * @returns List of predicted off-target sites
   */
  predictOffTargets(request: OffTargetRequest): OffTargetSite[] {
    const {
      guideRNA,
      genome,
      maxMismatches,
      includeBulges = false,
      filterByChromatin = false,
    } = request;

    // Validate input
    if (!guideRNA || guideRNA.length !== 20) {
      throw new CRISPRProtocolError(
        CRISPRErrorCode.INVALID_SEQUENCE,
        'Guide RNA must be exactly 20bp'
      );
    }

    // In a real implementation, this would search a genome database
    // For demonstration, we'll generate mock off-target sites
    const offTargets: OffTargetSite[] = [];

    // Simulate off-target search
    const numOffTargets = Math.min(maxMismatches * 2, 10);
    for (let i = 0; i < numOffTargets; i++) {
      const mismatches = Math.floor(Math.random() * (maxMismatches + 1));
      const mismatchPositions = this.generateMismatchPositions(
        mismatches,
        20
      );

      const cfdScore = this.calculateCFDScore(mismatchPositions);
      const mitScore = this.calculateMITScore(mismatches, mismatchPositions);

      // Skip if filtering by chromatin and site is closed
      const chromatinState = Math.random() > 0.3 ? 'open' : 'closed';
      if (filterByChromatin && chromatinState === 'closed') {
        continue;
      }

      const offTarget: OffTargetSite = {
        coordinates: this.generateMockCoordinates(genome),
        sequence: this.generateOffTargetSequence(guideRNA, mismatchPositions),
        mismatches,
        mismatchPositions,
        cfdScore,
        mitScore,
        annotation: this.generateMockAnnotation(),
        chromatinState,
        predictedActivity: cfdScore * 100,
      };

      offTargets.push(offTarget);
    }

    // Sort by CFD score (descending)
    return offTargets.sort((a, b) => b.cfdScore - a.cfdScore);
  }

  /**
   * Calculate editing efficiency from sequencing data
   *
   * @param request - Efficiency calculation parameters
   * @returns Editing efficiency with statistics
   */
  calculateEditingEfficiency(request: EfficiencyRequest): EfficiencyResponse {
    const { editedReads, totalReads, editType } = request;

    // Validate input
    if (editedReads > totalReads) {
      throw new CRISPRProtocolError(
        CRISPRErrorCode.INVALID_PARAMETERS,
        'Edited reads cannot exceed total reads'
      );
    }

    if (totalReads === 0) {
      throw new CRISPRProtocolError(
        CRISPRErrorCode.INVALID_PARAMETERS,
        'Total reads must be greater than 0'
      );
    }

    // Calculate efficiency
    const efficiency = (editedReads / totalReads) * 100;

    // Calculate standard error
    const p = editedReads / totalReads;
    const standardError = Math.sqrt((p * (1 - p)) / totalReads) * 100;

    // Calculate 95% confidence interval
    const z = 1.96; // 95% confidence
    const margin = z * standardError;
    const confidenceInterval: [number, number] = [
      Math.max(0, efficiency - margin),
      Math.min(100, efficiency + margin),
    ];

    // Assess quality based on total reads
    let quality: 'high' | 'medium' | 'low';
    if (totalReads >= 1000) {
      quality = 'high';
    } else if (totalReads >= 100) {
      quality = 'medium';
    } else {
      quality = 'low';
    }

    const adequateSampleSize = totalReads >= 500;

    // Generate recommendation
    let recommendation = '';
    if (!adequateSampleSize) {
      recommendation =
        'Sample size is low. Recommend ≥500 reads for reliable estimation. ';
    }

    // Compare to typical efficiency for edit type
    const typicalEfficiency = this.getTypicalEfficiency(editType);
    if (efficiency < typicalEfficiency * 0.5) {
      recommendation +=
        `Efficiency is below typical range for ${editType}. Consider optimizing delivery or gRNA design.`;
    } else if (efficiency >= typicalEfficiency * 0.8) {
      recommendation += `Efficiency is within expected range for ${editType}.`;
    }

    return {
      efficiency,
      confidenceInterval,
      quality,
      adequateSampleSize,
      recommendation,
      statistics: {
        standardError,
      },
    };
  }

  /**
   * Validate a complete CRISPR protocol
   *
   * @param validation - Protocol validation parameters
   * @returns Validation result with errors and warnings
   */
  validateProtocol(validation: ProtocolValidation): ValidationResult {
    const {
      protocol,
      checkEthics = true,
      checkRegulatory = true,
      performOffTargetAnalysis = true,
    } = validation;

    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Validate guide RNA
    const gRNA = protocol.guideRNA;
    if (gRNA.onTargetScore < CRISPR_CONSTANTS.MIN_ON_TARGET_SCORE) {
      warnings.push(
        `Low on-target score (${gRNA.onTargetScore.toFixed(2)}). Consider redesigning gRNA.`
      );
    }

    if (
      gRNA.gcContent < CRISPR_CONSTANTS.OPTIMAL_GC_MIN ||
      gRNA.gcContent > CRISPR_CONSTANTS.OPTIMAL_GC_MAX
    ) {
      warnings.push(
        `GC content (${gRNA.gcContent.toFixed(1)}%) outside optimal range (40-60%).`
      );
    }

    if (gRNA.specificityScore < CRISPR_CONSTANTS.MIN_SPECIFICITY_SCORE) {
      errors.push(
        `Specificity score too low (${gRNA.specificityScore}). High off-target risk.`
      );
      recommendations.push('Redesign gRNA with higher specificity.');
    }

    // Check off-targets
    if (performOffTargetAnalysis && gRNA.offTargets) {
      if (gRNA.offTargets.length > CRISPR_CONSTANTS.MAX_OFF_TARGETS) {
        warnings.push(
          `High number of predicted off-targets (${gRNA.offTargets.length}).`
        );
        recommendations.push(
          'Perform experimental off-target validation (GUIDE-seq, CIRCLE-seq).'
        );
      }

      const highActivityOffTargets = gRNA.offTargets.filter(
        (ot) => ot.predictedActivity > 50
      );
      if (highActivityOffTargets.length > 0) {
        warnings.push(
          `${highActivityOffTargets.length} off-targets with high predicted activity.`
        );
      }
    }

    // Validate delivery method
    const delivery = protocol.delivery;
    if (delivery.method.includes('virus') && !protocol.safety.ethicsApproval) {
      errors.push('Viral delivery requires ethics approval.');
    }

    // Safety checks
    const safety = protocol.safety;
    if (!safety.offTargetAnalysis) {
      errors.push('Off-target analysis is required for all protocols.');
    }

    if (checkEthics && !safety.ethicsApproval) {
      if (protocol.target.purpose === 'knockout') {
        warnings.push('Ethics approval recommended for gene knockout studies.');
      } else {
        errors.push('Ethics approval required for this application.');
      }
    }

    if (checkRegulatory && safety.regulatory.length === 0) {
      warnings.push('No regulatory compliance documented.');
    }

    // Risk assessment
    const offTargetRisk =
      gRNA.specificityScore > 70
        ? 'low'
        : gRNA.specificityScore > 50
          ? 'medium'
          : 'high';

    const overallRisk =
      safety.riskLevel === 'high' || offTargetRisk === 'high'
        ? 'high'
        : safety.riskLevel === 'medium' || offTargetRisk === 'medium'
          ? 'medium'
          : 'low';

    // Quality scores
    const gRNAQuality = Math.min(
      100,
      gRNA.onTargetScore * 50 + gRNA.specificityScore * 0.5
    );

    const protocolCompleteness =
      (Number(Boolean(protocol.target)) * 20 +
        Number(Boolean(protocol.guideRNA)) * 20 +
        Number(Boolean(protocol.casSystem)) * 20 +
        Number(Boolean(protocol.delivery)) * 20 +
        Number(Boolean(protocol.validation.length > 0)) * 20) /
      1;

    // Generate final recommendations
    if (overallRisk === 'high') {
      recommendations.push(
        'High risk protocol. Consider using high-fidelity Cas9 variants.'
      );
    }

    if (protocol.strategy.type === 'HDR' && !protocol.strategy.template) {
      errors.push('HDR strategy requires repair template.');
    }

    if (protocol.validation.length === 0) {
      warnings.push('No validation methods specified.');
      recommendations.push(
        'Add validation methods (at minimum: T7E1 or Sanger sequencing).'
      );
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      safety: {
        offTargetRisk,
        overallRisk,
        ethicsCompliant: safety.ethicsApproval || !checkEthics,
        regulatoryCompliant: safety.regulatory.length > 0 || !checkRegulatory,
      },
      quality: {
        gRNAQuality,
        specificityScore: gRNA.specificityScore,
        protocolCompleteness,
      },
      recommendations,
      validated: new Date(),
    };
  }

  /**
   * Simulate CRISPR editing experiment
   *
   * @param protocol - CRISPR protocol to simulate
   * @returns Simulation results with predictions
   */
  simulateEditing(protocol: CRISPRProtocol): EditingSimulation {
    const gRNA = protocol.guideRNA;

    // Predict outcomes based on strategy
    const outcomes: PredictedOutcomes = {
      nhejEfficiency: this.predictNHEJEfficiency(gRNA.onTargetScore),
      onTargetScore: gRNA.onTargetScore,
      specificityScore: gRNA.specificityScore,
      predictedIndels: {
        insertions: 30,
        deletions: 60,
        avgSize: 5,
      },
    };

    if (protocol.strategy.type === 'HDR' && protocol.strategy.template) {
      outcomes.hdrEfficiency = this.predictHDREfficiency(
        gRNA.onTargetScore,
        protocol.strategy.template
      );
    }

    if (protocol.casSystem.type === 'BaseEditor') {
      outcomes.baseEditEfficiency = this.predictBaseEditEfficiency(
        gRNA.onTargetScore
      );
    }

    // Calculate success probability
    const successProbability = this.calculateSuccessProbability(
      gRNA.onTargetScore,
      gRNA.specificityScore,
      protocol.delivery.expectedEfficiency || 50
    );

    // Estimate timeline
    const timeline = {
      design: '1-2 days',
      preparation: '3-5 days',
      execution: '1-3 days',
      validation: '3-7 days',
      total: '1-3 weeks',
    };

    return {
      id: `SIM-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      protocol,
      outcomes,
      offTargets: gRNA.offTargets || [],
      successProbability,
      timeline,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Extract gRNA sequence and PAM from target
   */
  private extractSequenceAndPAM(
    targetSequence: string,
    pamType: string
  ): { sequence: string; pam: string; fullTarget: string } {
    const seq = targetSequence.toUpperCase();

    // For SpCas9 (NGG), PAM is at 3' end
    if (pamType === 'NGG' || pamType === 'NAG' || pamType === 'NGA') {
      const sequence = seq.slice(0, 20);
      const pam = seq.slice(20, 23) || 'NGG';
      return { sequence, pam, fullTarget: sequence + pam };
    }

    // For Cas12a (TTTV), PAM is at 5' end
    if (pamType === 'TTTV') {
      const pam = seq.slice(0, 4);
      const sequence = seq.slice(4, 27);
      return { sequence, pam, fullTarget: pam + sequence };
    }

    // For SaCas9 (NNGRRT)
    if (pamType === 'NNGRRT') {
      const sequence = seq.slice(0, 21);
      const pam = seq.slice(21, 27) || 'NNGRRT';
      return { sequence, pam, fullTarget: sequence + pam };
    }

    // Default
    const sequence = seq.slice(0, 20);
    const pam = seq.slice(20, 23);
    return { sequence, pam, fullTarget: sequence + pam };
  }

  /**
   * Calculate GC content
   */
  private calculateGCContent(sequence: string): number {
    const gc = (sequence.match(/[GC]/gi) || []).length;
    return (gc / sequence.length) * 100;
  }

  /**
   * Check sequence constraints
   */
  private checkSequenceConstraints(
    sequence: string,
    gcContent: number
  ): string[] {
    const warnings: string[] = [];

    // Check for poly-T
    if (/TTTT/.test(sequence)) {
      warnings.push('Contains poly-T tract (≥4 T\'s) - may cause transcriptional termination');
    }

    // Check GC content
    if (gcContent < 30) {
      warnings.push('Low GC content (<30%) - may reduce activity');
    } else if (gcContent > 70) {
      warnings.push('High GC content (>70%) - may reduce synthesis quality');
    }

    // Check for homopolymer runs
    if (/AAAA|CCCC|GGGG/.test(sequence)) {
      warnings.push('Contains homopolymer run - may reduce activity');
    }

    return warnings;
  }

  /**
   * Calculate on-target score (simplified Doench algorithm)
   */
  private calculateOnTargetScore(sequence: string, gcContent: number): number {
    // Simplified scoring based on GC content and position weights
    let score = 0.5; // Base score

    // GC content factor
    const gcOptimal = 50;
    const gcDeviation = Math.abs(gcContent - gcOptimal);
    const gcFactor = 1 - gcDeviation / 100;
    score += gcFactor * 0.2;

    // Position-specific nucleotide preferences (simplified)
    const positionWeights = [
      0.01, 0.01, 0.02, 0.02, 0.03, 0.03, 0.04, 0.04, 0.05, 0.05, 0.05, 0.05,
      0.04, 0.04, 0.03, 0.03, 0.02, 0.02, 0.01, 0.01,
    ];

    for (let i = 0; i < sequence.length && i < positionWeights.length; i++) {
      const nucleotide = sequence[i];
      if (nucleotide === 'G' || nucleotide === 'C') {
        score += positionWeights[i];
      }
    }

    return Math.min(1, Math.max(0, score));
  }

  /**
   * Calculate specificity score
   */
  private calculateSpecificityScore(sequence: string): number {
    // Simplified specificity based on sequence uniqueness
    // Higher complexity = higher specificity
    const uniqueNucleotides = new Set(sequence).size;
    const baseScore = (uniqueNucleotides / 4) * 50;

    // Penalize simple repeats
    const hasRepeats = /(.)\1{2,}/.test(sequence);
    const repeatPenalty = hasRepeats ? 20 : 0;

    return Math.max(0, Math.min(100, baseScore + 30 - repeatPenalty));
  }

  /**
   * Predict editing efficiency
   */
  private predictEditingEfficiency(
    onTargetScore: number,
    gcContent: number
  ): number {
    const gcOptimal = 50;
    const gcDeviation = Math.abs(gcContent - gcOptimal) / 50;
    const gcPenalty = gcDeviation * 20;

    const efficiency = onTargetScore * 100 - gcPenalty;
    return Math.max(0, Math.min(100, efficiency));
  }

  /**
   * Generate mismatch positions
   */
  private generateMismatchPositions(count: number, length: number): number[] {
    const positions: number[] = [];
    while (positions.length < count) {
      const pos = Math.floor(Math.random() * length);
      if (!positions.includes(pos)) {
        positions.push(pos);
      }
    }
    return positions.sort((a, b) => a - b);
  }

  /**
   * Calculate CFD score
   */
  private calculateCFDScore(mismatchPositions: number[]): number {
    if (mismatchPositions.length === 0) return 1.0;

    let score = 1.0;
    for (const pos of mismatchPositions) {
      // PAM-proximal positions (16-20) have higher penalty
      const penalty = pos >= 16 ? 0.9 : pos >= 12 ? 0.6 : pos >= 6 ? 0.3 : 0.1;
      score *= 1 - penalty;
    }
    return Math.max(0, score);
  }

  /**
   * Calculate MIT score
   */
  private calculateMITScore(
    mismatches: number,
    mismatchPositions: number[]
  ): number {
    if (mismatches === 0) return 100;

    const offTargetWeight = mismatchPositions.reduce((sum, pos) => {
      const weight = pos >= 16 ? 10 : pos >= 12 ? 5 : pos >= 6 ? 2 : 1;
      return sum + weight;
    }, 0);

    const mitScore = 100 / (100 + offTargetWeight * mismatches);
    return mitScore * 100;
  }

  /**
   * Generate off-target sequence with mismatches
   */
  private generateOffTargetSequence(
    original: string,
    mismatchPositions: number[]
  ): string {
    const nucleotides = ['A', 'T', 'C', 'G'];
    const sequence = original.split('');

    for (const pos of mismatchPositions) {
      const current = sequence[pos];
      const alternatives = nucleotides.filter((n) => n !== current);
      sequence[pos] = alternatives[Math.floor(Math.random() * alternatives.length)];
    }

    return sequence.join('');
  }

  /**
   * Generate mock genomic coordinates
   */
  private generateMockCoordinates(genome: string): GenomicCoordinates {
    const chromosomes = [
      'chr1',
      'chr2',
      'chr3',
      'chr7',
      'chr12',
      'chr17',
      'chrX',
    ];
    const chr =
      chromosomes[Math.floor(Math.random() * chromosomes.length)];
    const start = Math.floor(Math.random() * 100000000);

    return {
      chromosome: chr,
      start,
      end: start + 23,
      strand: Math.random() > 0.5 ? '+' : '-',
      assembly: genome,
    };
  }

  /**
   * Generate mock annotation
   */
  private generateMockAnnotation(): OffTargetAnnotation {
    const regions: Array<OffTargetAnnotation['region']> = [
      'intergenic',
      'intron',
      'exon',
      'promoter',
    ];
    const region = regions[Math.floor(Math.random() * regions.length)];

    return {
      region,
      gene: region !== 'intergenic' ? `GENE${Math.floor(Math.random() * 1000)}` : undefined,
    };
  }

  /**
   * Get genome assembly for organism
   */
  private getAssemblyForOrganism(organism: string): string {
    const assemblies: Record<string, string> = {
      human: 'hg38',
      mouse: 'mm10',
      rat: 'rn6',
      zebrafish: 'danRer11',
      fly: 'dm6',
      worm: 'ce11',
    };
    return assemblies[organism] || 'unknown';
  }

  /**
   * Get typical efficiency for edit type
   */
  private getTypicalEfficiency(editType: string): number {
    const efficiencies: Record<string, number> = {
      NHEJ: CRISPR_CONSTANTS.TYPICAL_NHEJ,
      HDR: CRISPR_CONSTANTS.TYPICAL_HDR,
      base_edit: CRISPR_CONSTANTS.TYPICAL_BASE_EDIT,
      prime_edit: CRISPR_CONSTANTS.TYPICAL_PRIME_EDIT,
    };
    return efficiencies[editType] || 50;
  }

  /**
   * Predict NHEJ efficiency
   */
  private predictNHEJEfficiency(onTargetScore: number): number {
    return Math.min(90, onTargetScore * 100 * 1.2);
  }

  /**
   * Predict HDR efficiency
   */
  private predictHDREfficiency(
    onTargetScore: number,
    template: any
  ): number {
    const baseEfficiency = onTargetScore * 20;
    const haBonus = template.leftHA > 500 ? 5 : 0;
    return Math.min(30, baseEfficiency + haBonus);
  }

  /**
   * Predict base editing efficiency
   */
  private predictBaseEditEfficiency(onTargetScore: number): number {
    return Math.min(70, onTargetScore * 60);
  }

  /**
   * Calculate success probability
   */
  private calculateSuccessProbability(
    onTargetScore: number,
    specificityScore: number,
    deliveryEfficiency: number
  ): number {
    return (
      (onTargetScore * 0.4 +
        (specificityScore / 100) * 0.3 +
        (deliveryEfficiency / 100) * 0.3)
    );
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Design guide RNA (standalone function)
 */
export function designGuideRNA(request: GuideRNARequest): GuideRNA {
  const sdk = new CRISPRProtocolSDK();
  return sdk.designGuideRNA(request);
}

/**
 * Predict off-targets (standalone function)
 */
export function predictOffTargets(request: OffTargetRequest): OffTargetSite[] {
  const sdk = new CRISPRProtocolSDK();
  return sdk.predictOffTargets(request);
}

/**
 * Calculate editing efficiency (standalone function)
 */
export function calculateEditingEfficiency(
  request: EfficiencyRequest
): EfficiencyResponse {
  const sdk = new CRISPRProtocolSDK();
  return sdk.calculateEditingEfficiency(request);
}

/**
 * Validate protocol (standalone function)
 */
export function validateCRISPRProtocol(
  validation: ProtocolValidation
): ValidationResult {
  const sdk = new CRISPRProtocolSDK();
  return sdk.validateProtocol(validation);
}

/**
 * Simulate editing (standalone function)
 */
export function simulateCRISPREditing(
  protocol: CRISPRProtocol
): EditingSimulation {
  const sdk = new CRISPRProtocolSDK();
  return sdk.simulateEditing(protocol);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { CRISPRProtocolSDK };
export default CRISPRProtocolSDK;
