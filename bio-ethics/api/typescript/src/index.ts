/**
 * WIA-BIO-018: Bio-Ethics SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bioethics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for ethical oversight in biotechnology including:
 * - Informed consent validation
 * - Ethical risk assessment
 * - IRB protocol submission
 * - Gene editing ethics evaluation
 * - Vulnerable population protection
 */

import {
  ConsentValidation,
  ConsentValidationResult,
  ConsentFormElements,
  EthicalRiskAssessment,
  EthicalRiskResult,
  IRBProtocol,
  IRBSubmission,
  IRBReviewLevel,
  GeneEditingEvaluation,
  GeneEditingDecision,
  GeneEditingType,
  GeneEditingPurpose,
  RiskBenefitAnalysis,
  Risk,
  Benefit,
  RiskLevel,
  VulnerablePopulation,
  EnhancedProtection,
  IncidentalFinding,
  AdverseEvent,
  BioEthicsErrorCode,
  BioEthicsError,
  PrincipleAssessment,
  EthicalPrinciple,
  RiskCategory,
  BenefitCategory,
  ParticipantCapacity,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-018 Bio-Ethics SDK
 */
export class BioEthicsSDK {
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
   * Validate informed consent
   *
   * @param validation - Consent validation parameters
   * @returns Validation result with compliance score
   */
  validateInformedConsent(validation: ConsentValidation): ConsentValidationResult {
    const {
      participantId,
      studyId,
      consentForm,
      participantCapacity = 'full',
      comprehensionScore = 1.0,
      witnessPresent,
      signature,
    } = validation;

    const violations: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];
    const requiredActions: string[] = [];

    // Check all consent form elements
    const formElements = Object.entries(consentForm);
    const completedElements = formElements.filter(([_, value]) => value === true).length;
    const totalElements = formElements.length;
    const formCompleteness = completedElements / totalElements;

    if (formCompleteness < 1.0) {
      const missingElements = formElements
        .filter(([_, value]) => value === false)
        .map(([key]) => key);
      violations.push(`Incomplete consent form. Missing: ${missingElements.join(', ')}`);
      requiredActions.push('Complete all required consent form elements');
    }

    // Check comprehension
    const minComprehension = 0.8;
    if (comprehensionScore < minComprehension) {
      violations.push(
        `Insufficient comprehension (${(comprehensionScore * 100).toFixed(1)}%). Minimum required: ${minComprehension * 100}%`
      );
      requiredActions.push('Re-educate participant and reassess comprehension');
    } else if (comprehensionScore < 0.9) {
      warnings.push('Comprehension is adequate but could be improved');
      recommendations.push('Consider additional educational materials');
    }

    // Check participant capacity
    const capacityScores: Record<ParticipantCapacity, number> = {
      full: 1.0,
      borderline: 0.85,
      diminished: 0.65,
      'severely-impaired': 0.4,
    };

    const capacityScore = capacityScores[participantCapacity];

    if (participantCapacity === 'diminished' || participantCapacity === 'severely-impaired') {
      if (!signature.lar) {
        violations.push(
          'Participant has diminished capacity but no Legally Authorized Representative (LAR) consent'
        );
        requiredActions.push('Obtain consent from Legally Authorized Representative');
      } else {
        warnings.push(
          'LAR consent obtained for participant with diminished capacity (appropriate)'
        );
      }
    }

    if (participantCapacity === 'borderline') {
      warnings.push('Participant has borderline capacity - enhanced monitoring recommended');
      recommendations.push('Provide additional support and reassess capacity periodically');
    }

    // Check witness requirement
    if (!witnessPresent && (participantCapacity !== 'full' || comprehensionScore < 0.85)) {
      violations.push('Witness required for vulnerable participant or low comprehension');
      requiredActions.push('Obtain witness signature');
    }

    // Check signatures
    if (!signature.participant && !signature.lar) {
      violations.push('No participant or LAR signature');
      requiredActions.push('Obtain required signatures');
    }

    if (
      witnessPresent &&
      !signature.witness &&
      (participantCapacity !== 'full' || comprehensionScore < 0.9)
    ) {
      violations.push('Witness present but signature missing');
      requiredActions.push('Obtain witness signature');
    }

    // Calculate compliance score
    const elementScore = formCompleteness;
    const comprehensionWeight = 0.3;
    const capacityWeight = 0.2;
    const signatureScore = signature.participant || signature.lar ? 1.0 : 0.0;
    const witnessScore =
      witnessPresent && signature.witness
        ? 1.0
        : !witnessPresent &&
          participantCapacity === 'full' &&
          comprehensionScore >= 0.85
        ? 1.0
        : 0.5;

    const complianceScore =
      elementScore * 0.3 +
      comprehensionScore * comprehensionWeight +
      capacityScore * capacityWeight +
      signatureScore * 0.15 +
      witnessScore * 0.05;

    // Final recommendations
    if (complianceScore >= 0.95) {
      recommendations.push('Excellent consent process - maintain standards');
    } else if (complianceScore >= 0.8) {
      recommendations.push('Good consent process - address minor issues noted');
    } else if (complianceScore >= 0.7) {
      recommendations.push('Consent process needs improvement - address all warnings');
    } else {
      recommendations.push('Consent process inadequate - do not proceed until violations resolved');
    }

    return {
      isValid: violations.length === 0 && complianceScore >= 0.8,
      complianceScore,
      violations,
      warnings,
      recommendations,
      requiredActions: requiredActions.length > 0 ? requiredActions : undefined,
    };
  }

  /**
   * Assess ethical risk of research study
   *
   * @param assessment - Risk assessment parameters
   * @returns Ethical risk evaluation
   */
  assessEthicalRisk(assessment: EthicalRiskAssessment): EthicalRiskResult {
    const {
      studyType,
      targetPopulation,
      interventionType,
      potentialBenefits,
      potentialRisks,
      riskLevel,
    } = assessment;

    const warnings: string[] = [];
    const requiredProtections: string[] = [];

    // Assess population vulnerability
    let populationRiskMultiplier = 1.0;
    if (targetPopulation !== 'general-adults') {
      populationRiskMultiplier = 1.5;
      warnings.push(`Study involves vulnerable population: ${targetPopulation}`);
      requiredProtections.push(...this.getEnhancedProtections(targetPopulation).protections);
    }

    // Assess intervention risk
    const interventionRiskScores: Record<string, number> = {
      observational: 0.1,
      behavioral: 0.2,
      drug: 0.6,
      device: 0.5,
      surgical: 0.8,
      'gene-therapy': 0.9,
      other: 0.5,
    };

    const interventionRisk = interventionRiskScores[interventionType] || 0.5;

    // Calculate risk score
    const riskLevelScores: Record<RiskLevel, number> = {
      minimal: 0.2,
      'minor-increase': 0.4,
      'greater-than-minimal': 0.7,
      high: 0.9,
    };

    const baseRiskScore = riskLevelScores[riskLevel] || 0.5;
    const totalRiskScore =
      (baseRiskScore + interventionRisk) / 2 * populationRiskMultiplier +
      potentialRisks.length * 0.05;

    // Calculate benefit score
    const benefitScore = Math.min(1.0, potentialBenefits.length * 0.2);

    // Calculate risk-benefit ratio
    const riskBenefitRatio = benefitScore / Math.max(totalRiskScore, 0.1);

    // Calculate ethical score
    // Higher is better (combines low risk with high benefit)
    const ethicalScore = Math.min(1.0, riskBenefitRatio * 0.5);

    // Determine recommendation
    let recommendation: 'approve' | 'revise' | 'reject';
    let rationale: string;

    if (ethicalScore >= 0.7 && riskBenefitRatio >= 1.5) {
      recommendation = 'approve';
      rationale = 'Favorable risk-benefit profile with adequate protections';
    } else if (ethicalScore >= 0.5 && riskBenefitRatio >= 1.0) {
      recommendation = 'approve';
      rationale = 'Acceptable risk-benefit profile - implement all required protections';
      warnings.push('Risk-benefit ratio is modest - enhanced monitoring recommended');
    } else if (ethicalScore >= 0.3 || riskBenefitRatio >= 0.7) {
      recommendation = 'revise';
      rationale = 'Risk-benefit profile needs improvement';
      warnings.push('Consider reducing risks or increasing benefits');
      requiredProtections.push('Enhanced safety monitoring');
      requiredProtections.push('Data Safety Monitoring Board');
    } else {
      recommendation = 'reject';
      rationale = 'Unacceptable risk-benefit profile';
      warnings.push('Risks substantially outweigh benefits');
    }

    // Additional protections based on risk level
    if (riskLevel === 'greater-than-minimal' || riskLevel === 'high') {
      requiredProtections.push('Full IRB review required');
      requiredProtections.push('Independent data monitoring');
      requiredProtections.push('Regular safety reviews');
    }

    if (interventionType === 'gene-therapy') {
      requiredProtections.push('Long-term follow-up (minimum 5 years)');
      requiredProtections.push('Genetic counseling');
      requiredProtections.push('Family member notification protocol');
    }

    // Ensure IRB review
    if (targetPopulation !== 'general-adults' || riskLevel !== 'minimal') {
      requiredProtections.push('IRB approval with continuing review');
    }

    return {
      ethicalScore,
      riskBenefitRatio,
      recommendation,
      requiredProtections: [...new Set(requiredProtections)], // Remove duplicates
      warnings,
      rationale,
    };
  }

  /**
   * Submit protocol to IRB
   *
   * @param protocol - IRB protocol
   * @returns IRB submission record
   */
  submitIRBProtocol(protocol: IRBProtocol): IRBSubmission {
    // Determine appropriate review level
    let reviewLevel: IRBReviewLevel;

    if (protocol.riskLevel === 'minimal' && !this.hasVulnerablePopulations(protocol)) {
      // Check if eligible for exempt
      if (this.isExemptEligible(protocol)) {
        reviewLevel = 'exempt';
      } else {
        reviewLevel = 'expedited';
      }
    } else if (
      protocol.riskLevel === 'minor-increase' &&
      !this.hasVulnerablePopulations(protocol)
    ) {
      reviewLevel = 'expedited';
    } else {
      reviewLevel = 'full-board';
    }

    // Generate submission ID
    const submissionId = `IRB-${Date.now()}-${Math.random().toString(36).substr(2, 9).toUpperCase()}`;

    return {
      submissionId,
      protocol,
      reviewLevel,
      status: 'submitted',
      // Review date and decision will be determined by IRB
    };
  }

  /**
   * Evaluate gene editing ethics
   *
   * @param evaluation - Gene editing evaluation parameters
   * @returns Ethical decision
   */
  evaluateGeneEditingEthics(evaluation: GeneEditingEvaluation): GeneEditingDecision {
    const { editingType, purpose, scientificValidity, risks, ethical } = evaluation;

    const conditions: string[] = [];
    const oversight: string[] = [];
    const monitoring: string[] = [];

    // Germline editing - highly restricted
    if (editingType === 'germline') {
      if (purpose === 'enhancement') {
        return {
          decision: 'reject',
          rationale:
            'Germline editing for enhancement is ethically unacceptable and prohibited in most jurisdictions',
          oversight: [],
          monitoring: [],
        };
      }

      if (purpose === 'therapeutic') {
        return {
          decision: 'defer',
          rationale:
            'Germline editing for therapeutic purposes requires international consensus and is currently under moratorium',
          conditions: [
            'Await international regulatory framework',
            'Demonstrate compelling medical need',
            'Prove long-term safety across generations',
            'Obtain broad public consultation',
          ],
          oversight: [
            'International ethics committee',
            'National regulatory authority',
            'Institutional ethics board',
            'Independent safety monitoring',
          ],
          monitoring: [
            'Multi-generational follow-up',
            'Comprehensive genetic analysis',
            'Developmental monitoring',
            'Psychosocial assessment',
          ],
        };
      }

      // Research only
      conditions.push('Laboratory research only - no clinical application');
      conditions.push('No implantation of edited embryos');
      conditions.push('Embryo destruction within 14 days');
    }

    // Somatic editing - more permissive but regulated
    if (editingType === 'somatic') {
      if (purpose === 'enhancement') {
        return {
          decision: 'reject',
          rationale:
            'Somatic editing for enhancement raises significant ethical concerns and is not currently acceptable',
          oversight: [],
          monitoring: [],
        };
      }

      // Check scientific validity
      if (!scientificValidity.preclinicalEvidence) {
        return {
          decision: 'reject',
          rationale: 'Insufficient preclinical evidence',
          conditions: ['Complete preclinical studies'],
          oversight: [],
          monitoring: [],
        };
      }

      if (!scientificValidity.mechanismUnderstood) {
        conditions.push('Further mechanistic studies required');
      }

      if (!scientificValidity.safetyDemonstrated) {
        return {
          decision: 'defer',
          rationale: 'Safety not adequately demonstrated',
          conditions: ['Complete safety studies', 'Demonstrate acceptable off-target profile'],
          oversight: ['IRB', 'Data Safety Monitoring Board'],
          monitoring: [],
        };
      }

      // Check ethical requirements
      if (!ethical.consentObtained) {
        return {
          decision: 'reject',
          rationale: 'Informed consent required',
          oversight: [],
          monitoring: [],
        };
      }

      if (!ethical.benefitsOutweighRisks) {
        return {
          decision: 'revise',
          rationale: 'Risk-benefit profile unfavorable',
          conditions: ['Reduce risks or demonstrate greater benefits'],
          oversight: [],
          monitoring: [],
        };
      }

      // Calculate risk score
      const totalRisk =
        risks.offTargetEffects +
        risks.immuneResponse +
        risks.mosaicism +
        risks.longTermUnknown;
      const avgRisk = totalRisk / 4;

      if (avgRisk > 0.5) {
        conditions.push('Enhanced risk mitigation strategies required');
        monitoring.push('Intensive safety monitoring (weekly for first month)');
      }

      // Standard oversight for somatic therapeutic
      oversight.push('IRB approval with continuing review');
      oversight.push('Institutional biosafety committee');

      if (purpose === 'therapeutic') {
        oversight.push('FDA IND approval (or equivalent)');
      }

      // Standard monitoring
      monitoring.push('Long-term follow-up (minimum 5 years)');
      monitoring.push('Off-target effect surveillance');
      monitoring.push('Immune response monitoring');
      monitoring.push('Clinical efficacy assessment');

      // Additional conditions
      if (!ethical.noAlternatives) {
        conditions.push(
          'Justify why conventional therapies are inadequate or unavailable'
        );
      }

      if (!ethical.equitableAccess) {
        conditions.push('Develop plan for equitable access to therapy');
      }

      // Determine final decision
      if (
        purpose === 'therapeutic' &&
        scientificValidity.preclinicalEvidence &&
        scientificValidity.safetyDemonstrated &&
        ethical.consentObtained &&
        ethical.benefitsOutweighRisks &&
        avgRisk <= 0.5
      ) {
        return {
          decision: 'approve',
          rationale:
            'Somatic gene therapy meets ethical and scientific criteria for clinical trial',
          conditions,
          oversight,
          monitoring,
        };
      } else if (purpose === 'research') {
        return {
          decision: 'approve-research-only',
          rationale: 'Approved for research purposes with appropriate oversight',
          conditions: ['No clinical application without separate approval', ...conditions],
          oversight,
          monitoring,
        };
      } else {
        return {
          decision: 'defer',
          rationale: 'Additional evidence or safeguards needed',
          conditions,
          oversight,
          monitoring,
        };
      }
    }

    // Default fallback
    return {
      decision: 'defer',
      rationale: 'Insufficient information for decision',
      oversight: ['Ethics committee review'],
      monitoring: [],
    };
  }

  /**
   * Perform risk-benefit analysis
   *
   * @param studyId - Study identifier
   * @param risks - Identified risks
   * @param benefits - Identified benefits
   * @param justiceFactor - Justice distribution factor (0-1)
   * @returns Risk-benefit analysis result
   */
  performRiskBenefitAnalysis(
    studyId: string,
    risks: Risk[],
    benefits: Benefit[],
    justiceFactor: number = 1.0
  ): RiskBenefitAnalysis {
    // Calculate total risk score
    const totalRiskScore = risks.reduce((sum, risk) => sum + risk.score, 0);

    // Calculate total benefit score
    const totalBenefitScore = benefits.reduce((sum, benefit) => sum + benefit.score, 0);

    // Calculate risk-benefit ratio
    const riskBenefitRatio = totalBenefitScore / Math.max(totalRiskScore, 0.1);

    // Adjust for justice
    const adjustedRatio = riskBenefitRatio * justiceFactor;

    // Determine ethical acceptability
    const isEthicallyAcceptable = adjustedRatio >= 1.0 && justiceFactor >= 0.8;

    // Determine recommendation
    let recommendation: RiskBenefitAnalysis['recommendation'];
    let rationale: string;

    if (adjustedRatio >= 2.0 && justiceFactor >= 0.9) {
      recommendation = 'approve';
      rationale = 'Excellent risk-benefit profile with equitable distribution';
    } else if (adjustedRatio >= 1.5 && justiceFactor >= 0.85) {
      recommendation = 'approve';
      rationale = 'Favorable risk-benefit profile';
    } else if (adjustedRatio >= 1.0 && justiceFactor >= 0.8) {
      recommendation = 'approve-with-modifications';
      rationale = 'Acceptable risk-benefit profile with minor concerns';
    } else if (adjustedRatio >= 0.7 || justiceFactor >= 0.7) {
      recommendation = 'defer';
      rationale = 'Risk-benefit profile or justice concerns require revision';
    } else {
      recommendation = 'reject';
      rationale = 'Unacceptable risk-benefit profile';
    }

    return {
      analysisId: `RBA-${Date.now()}`,
      studyId,
      risks,
      totalRiskScore,
      benefits,
      totalBenefitScore,
      riskBenefitRatio: adjustedRatio,
      justiceFactor,
      isEthicallyAcceptable,
      recommendation,
      rationale,
    };
  }

  /**
   * Assess ethical principle compliance
   *
   * @param principle - Ethical principle to assess
   * @param evidence - Supporting evidence
   * @param concerns - Potential violations
   * @returns Principle assessment
   */
  assessEthicalPrinciple(
    principle: EthicalPrinciple,
    evidence: string[],
    concerns: string[] = []
  ): PrincipleAssessment {
    // Calculate score based on evidence vs concerns
    const evidenceScore = Math.min(1.0, evidence.length * 0.25);
    const concernPenalty = concerns.length * 0.2;
    const score = Math.max(0, Math.min(1.0, evidenceScore - concernPenalty));

    return {
      principle,
      score,
      evidence,
      concerns: concerns.length > 0 ? concerns : undefined,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Get enhanced protections for vulnerable population
   */
  private getEnhancedProtections(population: VulnerablePopulation): EnhancedProtection {
    const protections: Record<VulnerablePopulation, EnhancedProtection> = {
      children: {
        population: 'children',
        protections: [
          'Parental permission required',
          'Child assent for ages 7+',
          'Age-appropriate education materials',
          'Limited risk categories (45 CFR 46.404-407)',
        ],
        consentRequirements: [
          'Both parents when feasible',
          'Child assent unless direct benefit',
          'Respect for dissent',
        ],
        irbRequirements: ['Full board review', 'Pediatric expert on IRB'],
        riskLimitations: [
          'Minimal risk preferred',
          'Minor increase only if direct benefit or vital knowledge',
        ],
      },
      'pregnant-women': {
        population: 'pregnant-women',
        protections: [
          'Preclinical reproductive toxicity studies',
          'Fetal risk assessment',
          "Father's consent when feasible",
          'No incentives related to pregnancy outcome',
        ],
        consentRequirements: [
          'Informed of fetal risks',
          "Father's consent (unless rape/incest)",
          'No influence on pregnancy decisions',
        ],
        irbRequirements: ['Full board review', 'Fetal protection assessment'],
        riskLimitations: ['Minimal risk to fetus preferred', 'Risk justified by benefit'],
      },
      prisoners: {
        population: 'prisoners',
        protections: [
          'Prisoner representative on IRB',
          'No parole advantage',
          'Enhanced voluntariness monitoring',
          'Purpose must benefit prisoners as class',
        ],
        consentRequirements: [
          'Enhanced coercion screening',
          'Independent consent monitor',
          'Regular re-consent',
        ],
        irbRequirements: [
          'Full board review',
          'Prisoner or prisoner advocate on IRB',
          'Secretary HHS review (US)',
        ],
        riskLimitations: [
          'Minimal risk only (with exceptions)',
          'No cosmetic or convenience studies',
        ],
      },
      'cognitively-impaired': {
        population: 'cognitively-impaired',
        protections: [
          'Capacity assessment',
          'LAR consent when needed',
          'Seek assent when possible',
          'Respect for dissent',
          'Periodic capacity reassessment',
        ],
        consentRequirements: [
          'Formal capacity evaluation',
          'LAR consent if capacity < 0.75',
          'Simplified consent forms',
          'Extended education time',
        ],
        irbRequirements: ['Full board review', 'Enhanced protection plan'],
        riskLimitations: ['Minimal risk preferred', 'Benefit must be significant'],
      },
      'economically-disadvantaged': {
        population: 'economically-disadvantaged',
        protections: [
          'Reasonable compensation (not undue inducement)',
          'Access to results/benefits',
          'Cultural sensitivity',
          'Language accessibility',
          'Community engagement',
        ],
        consentRequirements: [
          'Verify understanding',
          'Assess for undue inducement',
          'Provide in native language',
        ],
        irbRequirements: ['Assessment of exploitation risk', 'Community consultation'],
        riskLimitations: ['Equitable risk distribution', 'Fair benefit sharing'],
      },
      'educationally-disadvantaged': {
        population: 'educationally-disadvantaged',
        protections: [
          'Simplified consent materials',
          'Visual aids',
          'Extended education time',
          'Teach-back assessment',
          'Witness present',
        ],
        consentRequirements: [
          'Enhanced comprehension assessment',
          'Multiple teaching sessions',
          'Witness signature',
        ],
        irbRequirements: ['Consent form readability review', 'Education plan approval'],
        riskLimitations: ['Clear risk communication', 'Enhanced monitoring'],
      },
      'terminally-ill': {
        population: 'terminally-ill',
        protections: [
          'Avoid therapeutic misconception',
          'Realistic benefit communication',
          'Palliative care access maintained',
          'Family support',
        ],
        consentRequirements: [
          'Capacity assessment',
          'Avoid desperation exploitation',
          'Right to withdraw emphasized',
        ],
        irbRequirements: ['Palliative care review', 'Benefit-risk scrutiny'],
        riskLimitations: ['Proportionate to remaining quality of life'],
      },
      'ethnic-minorities': {
        population: 'ethnic-minorities',
        protections: [
          'Cultural sensitivity',
          'Community engagement',
          'Translation services',
          'Historical context awareness',
          'Equitable benefit sharing',
        ],
        consentRequirements: [
          'Culturally appropriate materials',
          'Native language consent',
          'Community leader involvement',
        ],
        irbRequirements: ['Community consultation', 'Cultural competency review'],
        riskLimitations: ['Fair inclusion', 'No exploitation'],
      },
      refugees: {
        population: 'refugees',
        protections: [
          'Trauma-informed approach',
          'Language services',
          'Cultural mediators',
          'Legal protections',
          'No immigration consequences',
        ],
        consentRequirements: [
          'Enhanced voluntariness assessment',
          'Native language materials',
          'Legal advisor available',
        ],
        irbRequirements: ['Vulnerability assessment', 'Special protections plan'],
        riskLimitations: ['Minimal risk preferred', 'Clear benefit required'],
      },
    };

    return protections[population];
  }

  /**
   * Check if protocol involves vulnerable populations
   */
  private hasVulnerablePopulations(protocol: IRBProtocol): boolean {
    return (
      protocol.participants.vulnerablePopulations !== undefined &&
      protocol.participants.vulnerablePopulations.length > 0
    );
  }

  /**
   * Check if protocol is eligible for exempt review
   */
  private isExemptEligible(protocol: IRBProtocol): boolean {
    // Simplified eligibility check
    const exemptCategories = [
      'educational research',
      'anonymous surveys',
      'observation of public behavior',
      'existing de-identified data',
    ];

    const isExemptType = exemptCategories.some((cat) =>
      protocol.studyType.toLowerCase().includes(cat)
    );

    const noVulnerablePopulations = !this.hasVulnerablePopulations(protocol);
    const minimalRisk = protocol.riskLevel === 'minimal';

    return isExemptType && noVulnerablePopulations && minimalRisk;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Validate informed consent (standalone function)
 */
export function validateInformedConsent(
  validation: ConsentValidation
): ConsentValidationResult {
  const sdk = new BioEthicsSDK();
  return sdk.validateInformedConsent(validation);
}

/**
 * Assess ethical risk (standalone function)
 */
export function assessEthicalRisk(
  assessment: EthicalRiskAssessment
): EthicalRiskResult {
  const sdk = new BioEthicsSDK();
  return sdk.assessEthicalRisk(assessment);
}

/**
 * Submit IRB protocol (standalone function)
 */
export function submitIRBProtocol(protocol: IRBProtocol): IRBSubmission {
  const sdk = new BioEthicsSDK();
  return sdk.submitIRBProtocol(protocol);
}

/**
 * Evaluate gene editing ethics (standalone function)
 */
export function evaluateGeneEditingEthics(
  evaluation: GeneEditingEvaluation
): GeneEditingDecision {
  const sdk = new BioEthicsSDK();
  return sdk.evaluateGeneEditingEthics(evaluation);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BioEthicsSDK };
export default BioEthicsSDK;
