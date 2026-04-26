/**
 * WIA-AUG-012: Augmentation Ethics SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Ethics Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides tools for ethical assessment of human augmentation including:
 * - Ethical principle evaluation
 * - Informed consent validation
 * - Coercion detection and prevention
 * - Equity and access assessment
 * - Identity impact analysis
 * - Reversibility review
 * - Vulnerable population protection
 */

import {
  EthicalPrinciple,
  PrincipleScores,
  EthicalConcern,
  ConsentLevel,
  ConsentDocumentation,
  AugmentationType,
  AugmentationDetails,
  EquityAssessment,
  EquityContext,
  AccessBarriers,
  DistributionMetric,
  CoercionContext,
  CoercionIndicators,
  CoercionRisk,
  CoercionCheckResult,
  IdentityDimensions,
  IdentityImpact,
  AuthenticityAssessment,
  ReversibilityLevel,
  ReversibilityProfile,
  VulnerablePopulation,
  SubjectProfile,
  VulnerableProtections,
  DecisionContext,
  EthicalAssessmentRequest,
  EthicalAssessment,
  RequiredAction,
  ConsentValidationRequest,
  ConsentValidationResult,
  EthicsReport,
  ETHICS_CONSTANTS,
  EthicsErrorCode,
  EthicsError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-012 Augmentation Ethics SDK
 */
export class AugmentationEthicsSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Perform comprehensive ethical compliance assessment
   */
  assessEthicalCompliance(request: EthicalAssessmentRequest): EthicalAssessment {
    // Assess each ethical principle
    const principleScores = this.evaluatePrinciples(request);

    // Identify satisfied principles
    const principlesSatisfied = this.getatisfiedPrinciples(principleScores);

    // Identify concerns
    const concerns = this.identifyConcerns(principleScores, request);

    // Generate recommendations
    const recommendations = this.generateRecommendations(concerns, request);

    // Determine required actions
    const requiredActions = this.determineRequiredActions(concerns, request);

    // Determine approval level
    const approvalLevel = this.determineApprovalLevel(principleScores, request);

    // Overall compliance
    const compliant = this.isCompliant(principleScores);

    // Additional assessments
    const consentValid = request.consent ? this.validateConsentQuick(request.consent, request.augmentation.type) : false;
    const equityAssessed = true;
    const coercionChecked = true;
    const identityEvaluated = true;
    const reversibilityReviewed = true;
    const vulnerableProtected = request.subject.isVulnerable ? this.checkVulnerableProtection(request) : true;

    return {
      compliant,
      principleScores,
      principlesSatisfied,
      concerns,
      recommendations,
      requiredActions,
      approvalLevel,
      consentValid,
      equityAssessed,
      coercionChecked,
      identityEvaluated,
      reversibilityReviewed,
      vulnerableProtected,
    };
  }

  /**
   * Validate informed consent documentation
   */
  validateConsent(request: ConsentValidationRequest): ConsentValidationResult {
    const { augmentationType, requiredLevel, providedConsent } = request;

    const gaps: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Check if consent level matches requirement
    if (this.getConsentLevelOrder(providedConsent.level) < this.getConsentLevelOrder(requiredLevel)) {
      gaps.push(`Consent level ${providedConsent.level} insufficient; ${requiredLevel} required`);
    }

    // Check disclosures
    const requiredDisclosures = this.getRequiredDisclosures(requiredLevel);
    for (const [key, required] of Object.entries(requiredDisclosures)) {
      if (required && !providedConsent.disclosures[key as keyof typeof providedConsent.disclosures]) {
        gaps.push(`Missing disclosure: ${key}`);
      }
    }

    // Check comprehension
    if (!providedConsent.comprehension.assessed) {
      gaps.push('Comprehension not assessed');
    } else if (providedConsent.comprehension.score < 7.0) {
      warnings.push(`Low comprehension score: ${providedConsent.comprehension.score}/10`);
    }

    // Check voluntariness
    if (!providedConsent.voluntariness.coercionScreened) {
      gaps.push('Coercion not screened');
    }
    if (providedConsent.voluntariness.coercionDetected) {
      gaps.push('Coercion detected - consent invalid');
    }
    if (providedConsent.voluntariness.pressure !== 'none' && providedConsent.voluntariness.pressure !== 'minimal') {
      warnings.push(`Elevated pressure detected: ${providedConsent.voluntariness.pressure}`);
    }

    // Check capacity
    if (!providedConsent.capacity.evaluated) {
      gaps.push('Decision-making capacity not evaluated');
    }
    if (!providedConsent.capacity.hasCapacity) {
      gaps.push('Subject lacks decision-making capacity');
    }

    // Check cooling-off period
    const requiredCoolingOff = this.getRequiredCoolingOffPeriod(requiredLevel);
    if (providedConsent.coolingOff.required) {
      if (providedConsent.coolingOff.duration < requiredCoolingOff) {
        gaps.push(`Cooling-off period ${providedConsent.coolingOff.duration} days < required ${requiredCoolingOff} days`);
      }
      if (!providedConsent.coolingOff.completed) {
        gaps.push('Cooling-off period not completed');
      }
    }

    // Check sessions for comprehensive/experimental
    if (requiredLevel === ConsentLevel.COMPREHENSIVE || requiredLevel === ConsentLevel.EXPERIMENTAL) {
      if (!providedConsent.sessions || providedConsent.sessions.length < 3) {
        gaps.push('Insufficient consent sessions (minimum 3 required)');
      }
    }

    // Generate recommendations
    if (gaps.length > 0) {
      recommendations.push('Address all gaps before proceeding');
    }
    if (warnings.length > 0) {
      recommendations.push('Review warnings and consider additional safeguards');
    }
    if (providedConsent.comprehension.score < 8.0) {
      recommendations.push('Consider additional education to improve comprehension');
    }

    const valid = gaps.length === 0;
    const compliant = valid && warnings.length === 0;

    return {
      valid,
      level: providedConsent.level,
      gaps,
      warnings,
      recommendations,
      compliant,
    };
  }

  /**
   * Evaluate equity and access considerations
   */
  evaluateEquity(context: EquityContext): EquityAssessment {
    // Assess access barriers
    const accessBarriers = this.assessAccessBarriers(context);

    // Assess demographic distribution (simulated for now)
    const demographicDistribution = {
      income: this.calculateDistributionMetric(0.3),
      race: this.calculateDistributionMetric(0.2),
      geography: this.calculateDistributionMetric(0.4),
      disability: this.calculateDistributionMetric(0.35),
    };

    // Calculate equity score
    const barrierScore = (accessBarriers.economic + accessBarriers.geographic +
                         accessBarriers.social + accessBarriers.systemic) / 4;

    const inequalityScore = (
      demographicDistribution.income.giniCoefficient * 25 +
      demographicDistribution.race.giniCoefficient * 25 +
      demographicDistribution.geography.giniCoefficient * 25 +
      demographicDistribution.disability.giniCoefficient * 25
    );

    const equityScore = Math.max(0, 100 - (barrierScore * 5 + inequalityScore));

    // Identify concerns
    const concerns: string[] = [];
    if (accessBarriers.economic > 7) concerns.push('High economic barriers to access');
    if (accessBarriers.geographic > 7) concerns.push('Significant geographic access limitations');
    if (accessBarriers.social > 7) concerns.push('Major social barriers present');
    if (accessBarriers.systemic > 7) concerns.push('Systemic barriers impede equitable access');

    for (const [group, metric] of Object.entries(demographicDistribution)) {
      if (metric.giniCoefficient > 0.4) {
        concerns.push(`High inequality in ${group} distribution`);
      }
    }

    // Generate mitigations
    const mitigations: string[] = [];
    if (accessBarriers.economic > 5) {
      mitigations.push('Implement sliding scale pricing or subsidies');
    }
    if (accessBarriers.geographic > 5) {
      mitigations.push('Expand geographic availability or telemedicine options');
    }
    if (accessBarriers.social > 5) {
      mitigations.push('Address social barriers through education and outreach');
    }
    if (accessBarriers.systemic > 5) {
      mitigations.push('Advocate for policy changes to reduce systemic barriers');
    }

    const compliant = equityScore >= ETHICS_CONSTANTS.EQUITY_THRESHOLDS.EQUITABLE;

    return {
      accessBarriers,
      demographicDistribution,
      equityScore,
      concerns,
      mitigations,
      compliant,
    };
  }

  /**
   * Check for coercion indicators
   */
  checkCoercion(context: CoercionContext, indicators: CoercionIndicators): CoercionCheckResult {
    // Calculate risk score
    const weights = {
      mandatoryRequirement: 3.0,
      employmentConsequence: 2.5,
      authorityPressure: 2.0,
      powerImbalance: 2.0,
      financialIncentive: 1.5,
      limitedAlternatives: 1.5,
      peerPressure: 1.0,
      timeConstraint: 1.0,
    };

    let score = 0;
    for (const [key, value] of Object.entries(indicators)) {
      if (value && key in weights) {
        score += weights[key as keyof typeof weights];
      }
    }

    // Apply context multiplier
    const contextMultipliers: Record<CoercionContext, number> = {
      military: 1.5,
      occupational: 1.3,
      educational: 1.2,
      medical: 1.0,
      research: 1.1,
      social: 0.9,
      familial: 1.1,
    };
    score *= contextMultipliers[context];

    // Determine risk level
    let riskLevel: CoercionCheckResult['riskLevel'];
    if (score === 0) riskLevel = 'none';
    else if (score <= ETHICS_CONSTANTS.COERCION_THRESHOLDS.LOW) riskLevel = 'low';
    else if (score <= ETHICS_CONSTANTS.COERCION_THRESHOLDS.MODERATE) riskLevel = 'moderate';
    else if (score <= ETHICS_CONSTANTS.COERCION_THRESHOLDS.HIGH) riskLevel = 'high';
    else riskLevel = 'severe';

    // Identify concerns
    const concerns: string[] = [];
    if (indicators.mandatoryRequirement) concerns.push('Augmentation is mandatory requirement');
    if (indicators.employmentConsequence) concerns.push('Employment consequences for non-augmentation');
    if (indicators.authorityPressure) concerns.push('Pressure from authority figures');
    if (indicators.powerImbalance) concerns.push('Power imbalance in decision context');
    if (indicators.limitedAlternatives) concerns.push('Limited alternatives available');
    if (indicators.financialIncentive) concerns.push('Financial incentives may constitute undue inducement');
    if (indicators.timeConstraint) concerns.push('Time pressure limiting deliberation');

    // Determine interventions
    const interventions: string[] = [];
    if (riskLevel === 'severe') {
      interventions.push('HALT: Do not proceed - severe coercion detected');
      interventions.push('Conduct investigation of coercive practices');
      interventions.push('Report to ethics board and regulatory authorities');
    } else if (riskLevel === 'high') {
      interventions.push('Require ethics committee review');
      interventions.push('Provide independent counseling');
      interventions.push('Extended cooling-off period (minimum 30 days)');
      interventions.push('Ensure viable alternatives available');
      interventions.push('External advocacy required');
    } else if (riskLevel === 'moderate') {
      interventions.push('Enhanced informed consent process');
      interventions.push('Independent counseling recommended');
      interventions.push('Extended cooling-off period (minimum 14 days)');
      interventions.push('Document alternatives offered');
    } else if (riskLevel === 'low') {
      interventions.push('Standard consent process with coercion screening');
      interventions.push('Document voluntary nature of decision');
    }

    const coercionDetected = riskLevel !== 'none';
    const proceedRecommendation = riskLevel !== 'severe' && riskLevel !== 'high';

    return {
      coercionDetected,
      riskLevel,
      concerns,
      interventions,
      proceedRecommendation,
    };
  }

  /**
   * Assess identity impact of augmentation
   */
  assessIdentityImpact(augmentation: AugmentationDetails): IdentityImpact {
    // Estimate identity dimensions impact based on augmentation type
    const dimensions = this.estimateIdentityDimensions(augmentation);

    // Calculate overall impact score
    const overallScore = this.calculateIdentityScore(dimensions);

    // Determine impact level
    let level: IdentityImpact['level'];
    if (overallScore < ETHICS_CONSTANTS.IDENTITY_THRESHOLDS.MINIMAL) {
      level = 'minimal';
    } else if (overallScore < ETHICS_CONSTANTS.IDENTITY_THRESHOLDS.MODERATE) {
      level = 'moderate';
    } else if (overallScore < ETHICS_CONSTANTS.IDENTITY_THRESHOLDS.SUBSTANTIAL) {
      level = 'substantial';
    } else {
      level = 'severe';
    }

    // Identify concerns
    const concerns: string[] = [];
    if (dimensions.psychological.memory > 6) concerns.push('Significant memory alteration risk');
    if (dimensions.psychological.personality > 6) concerns.push('Personality changes likely');
    if (dimensions.psychological.consciousness > 6) concerns.push('Consciousness alteration risk');
    if (dimensions.narrative.continuity > 6) concerns.push('Narrative discontinuity risk');
    if (dimensions.values.beliefs > 6) concerns.push('Core beliefs may be affected');
    if (level === 'severe') concerns.push('SEVERE identity impact - comprehensive review required');

    // Generate recommendations
    const recommendations: string[] = [];
    if (level === 'severe') {
      recommendations.push('Extensive psychological evaluation required');
      recommendations.push('Identity preservation strategies essential');
      recommendations.push('Consider less invasive alternatives');
      recommendations.push('Long-term identity monitoring mandatory');
    } else if (level === 'substantial') {
      recommendations.push('Psychological support throughout process');
      recommendations.push('Gradual implementation if possible');
      recommendations.push('Regular identity assessments');
    } else if (level === 'moderate') {
      recommendations.push('Counseling to prepare for changes');
      recommendations.push('Monitoring for identity disruption');
    }

    const authenticityPreserved = level !== 'severe' && dimensions.values.beliefs < 7;

    return {
      dimensions,
      overallScore,
      level,
      concerns,
      recommendations,
      authenticityPreserved,
    };
  }

  /**
   * Review reversibility of augmentation
   */
  reviewReversibility(augmentation: AugmentationDetails): ReversibilityProfile {
    const restorationPercentage = augmentation.reversibility * 100;

    // Determine reversibility level
    let level: ReversibilityLevel;
    if (restorationPercentage >= ETHICS_CONSTANTS.REVERSIBILITY_THRESHOLDS.FULLY) {
      level = ReversibilityLevel.FULLY_REVERSIBLE;
    } else if (restorationPercentage >= ETHICS_CONSTANTS.REVERSIBILITY_THRESHOLDS.LARGELY) {
      level = ReversibilityLevel.LARGELY_REVERSIBLE;
    } else if (restorationPercentage >= ETHICS_CONSTANTS.REVERSIBILITY_THRESHOLDS.PARTIALLY) {
      level = ReversibilityLevel.PARTIALLY_REVERSIBLE;
    } else if (restorationPercentage >= ETHICS_CONSTANTS.REVERSIBILITY_THRESHOLDS.MINIMALLY) {
      level = ReversibilityLevel.MINIMALLY_REVERSIBLE;
    } else {
      level = ReversibilityLevel.IRREVERSIBLE;
    }

    // Estimate reversal process
    const reversalProcess = {
      surgical: augmentation.invasiveness !== 'none',
      duration: this.estimateReversalDuration(augmentation),
      risk: this.estimateReversalRisk(augmentation),
      cost: this.estimateReversalCost(augmentation),
      availability: restorationPercentage > 40,
    };

    // Identify permanent changes
    const permanentChanges: string[] = [];
    if (restorationPercentage < 100) {
      permanentChanges.push(`${100 - restorationPercentage}% of changes are permanent`);
    }
    if (augmentation.invasiveness === 'high') {
      permanentChanges.push('Surgical scars and tissue modification');
    }

    // Estimate recovery periods
    const recovery = {
      physical: this.estimatePhysicalRecovery(augmentation),
      psychological: this.estimatePsychologicalRecovery(augmentation),
      functional: this.estimateFunctionalRecovery(augmentation),
    };

    // Determine if justification required
    const justificationRequired =
      (augmentation.type === AugmentationType.ENHANCEMENT && restorationPercentage < 70) ||
      (augmentation.type === AugmentationType.EXPERIMENTAL && restorationPercentage < 90);

    return {
      level,
      restorationPercentage,
      reversalProcess,
      permanentChanges,
      recovery,
      justificationRequired,
    };
  }

  /**
   * Apply protections for vulnerable populations
   */
  protectVulnerable(subject: SubjectProfile): VulnerableProtections {
    const required = subject.isVulnerable;
    const categories = subject.vulnerableCategories;

    // Determine additional requirements
    const additionalRequirements = {
      independentAdvocacy: categories.some(c =>
        ['cognitive_impairment', 'children', 'institutionalized', 'prisoners'].includes(c)
      ),
      ethicsReview: required,
      judicialOversight: categories.some(c =>
        ['prisoners', 'cognitive_impairment'].includes(c)
      ),
      extendedCoolingOff: required,
      communityConsultation: categories.includes('minority' as VulnerablePopulation),
      surrogateConsent: categories.includes('cognitive_impairment' as VulnerablePopulation) && !subject.hasDecisionCapacity,
    };

    // Determine restrictions
    const restrictions = {
      enhancementProhibited: categories.some(c =>
        ['children', 'cognitive_impairment', 'prisoners'].includes(c)
      ),
      experimentalProhibited: categories.some(c =>
        ['children', 'prisoners'].includes(c)
      ),
      irreversibleProhibited: categories.some(c =>
        ['children', 'cognitive_impairment'].includes(c)
      ),
      therapeuticOnly: categories.some(c =>
        ['children', 'cognitive_impairment', 'prisoners'].includes(c)
      ),
    };

    // Identify support services
    const supportServices: string[] = [];
    if (required) supportServices.push('Counseling and psychological support');
    if (additionalRequirements.independentAdvocacy) supportServices.push('Independent patient advocacy');
    if (subject.socioeconomicStatus === 'low') supportServices.push('Financial assistance programs');
    if (subject.primaryLanguage !== 'English') supportServices.push('Language interpretation services');
    if (categories.includes('minority' as VulnerablePopulation)) supportServices.push('Culturally appropriate care');

    return {
      required,
      categories,
      additionalRequirements,
      restrictions,
      supportServices,
    };
  }

  /**
   * Generate comprehensive ethics report
   */
  generateEthicsReport(request: EthicalAssessmentRequest): EthicsReport {
    const ethicalAssessment = this.assessEthicalCompliance(request);

    const consentValidation = request.consent
      ? this.validateConsent({
          subjectId: request.subject.id,
          augmentationType: request.augmentation.type,
          requiredLevel: this.getRequiredConsentLevel(request.augmentation.type),
          providedConsent: request.consent,
        })
      : {
          valid: false,
          level: ConsentLevel.BASIC,
          gaps: ['No consent provided'],
          warnings: [],
          recommendations: ['Obtain appropriate consent'],
          compliant: false,
        };

    const equityContext: EquityContext = {
      augmentationType: request.augmentation.type,
      cost: 50000,  // Default estimate
      availability: 'limited',
      insuranceCoverage: request.augmentation.type === AugmentationType.THERAPEUTIC,
      subsidiesAvailable: false,
      geographicRestrictions: [],
    };
    const equityAssessment = this.evaluateEquity(equityContext);

    const coercionIndicators: CoercionIndicators = {
      mandatoryRequirement: false,
      employmentConsequence: false,
      peerPressure: request.context.isPressured,
      authorityPressure: false,
      financialIncentive: false,
      limitedAlternatives: !request.context.hasAlternatives,
      powerImbalance: false,
      timeConstraint: request.context.timeConstrained,
    };
    const coercionCheck = this.checkCoercion(
      request.context.coercionContext || 'medical',
      coercionIndicators
    );

    const identityImpact = this.assessIdentityImpact(request.augmentation);
    const reversibilityProfile = this.reviewReversibility(request.augmentation);

    const vulnerableProtections = request.subject.isVulnerable
      ? this.protectVulnerable(request.subject)
      : undefined;

    // Determine overall recommendation
    let overallRecommendation: 'proceed' | 'conditional' | 'do_not_proceed';
    if (!ethicalAssessment.compliant || coercionCheck.riskLevel === 'severe') {
      overallRecommendation = 'do_not_proceed';
    } else if (
      ethicalAssessment.concerns.some(c => c.severity === 'high' || c.severity === 'critical') ||
      coercionCheck.riskLevel === 'high' ||
      identityImpact.level === 'severe'
    ) {
      overallRecommendation = 'conditional';
    } else {
      overallRecommendation = 'proceed';
    }

    // Generate summary
    const summary = this.generateReportSummary(
      ethicalAssessment,
      consentValidation,
      equityAssessment,
      coercionCheck,
      identityImpact,
      reversibilityProfile,
      overallRecommendation
    );

    return {
      id: `ETH-${Date.now()}`,
      generatedAt: new Date(),
      augmentation: request.augmentation,
      subject: request.subject,
      ethicalAssessment,
      consentValidation,
      equityAssessment,
      coercionCheck,
      identityImpact,
      reversibilityProfile,
      vulnerableProtections,
      overallRecommendation,
      summary,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private evaluatePrinciples(request: EthicalAssessmentRequest): PrincipleScores {
    return {
      autonomy: this.evaluateAutonomy(request),
      beneficence: this.evaluateBeneficence(request),
      nonMaleficence: this.evaluateNonMaleficence(request),
      justice: this.evaluateJustice(request),
      dignity: this.evaluateDignity(request),
      authenticity: this.evaluateAuthenticity(request),
    };
  }

  private evaluateAutonomy(request: EthicalAssessmentRequest): number {
    let score = 10;

    // Informed consent
    if (!request.consent) score -= 3;
    else {
      if (!request.consent.comprehension.assessed) score -= 1;
      if (request.consent.comprehension.score < 8) score -= 1;
    }

    // Decision-making capacity
    if (!request.subject.hasDecisionCapacity) score -= 4;

    // Coercion
    if (request.context.isPressured) score -= 2;
    if (request.context.timeConstrained) score -= 1;

    // Vulnerable populations
    if (request.subject.isVulnerable &&
        request.subject.vulnerableCategories.includes('cognitive_impairment' as VulnerablePopulation)) {
      score -= 2;
    }

    return Math.max(0, score);
  }

  private evaluateBeneficence(request: EthicalAssessmentRequest): number {
    let score = 8;  // Default moderate

    // Therapeutic has clear benefit
    if (request.augmentation.type === AugmentationType.THERAPEUTIC) score += 2;

    // Enhancement may have questionable benefit
    if (request.augmentation.type === AugmentationType.ENHANCEMENT) score -= 1;

    // Experimental uncertain
    if (request.augmentation.type === AugmentationType.EXPERIMENTAL) score -= 2;

    // Risk level
    if (request.augmentation.riskLevel === 'minimal' || request.augmentation.riskLevel === 'low') score += 1;
    if (request.augmentation.riskLevel === 'high' || request.augmentation.riskLevel === 'severe') score -= 2;

    return Math.max(0, Math.min(10, score));
  }

  private evaluateNonMaleficence(request: EthicalAssessmentRequest): number {
    let score = 10;

    // Risk level
    const riskPenalties: Record<string, number> = {
      minimal: 0,
      low: 1,
      moderate: 2,
      high: 4,
      severe: 6,
    };
    score -= riskPenalties[request.augmentation.riskLevel] || 0;

    // Reversibility (lower reversibility = higher harm risk)
    if (request.augmentation.reversibility < 0.5) score -= 2;
    if (request.augmentation.reversibility < 0.3) score -= 2;

    // Duration
    if (request.augmentation.duration === 'permanent') score -= 1;

    return Math.max(0, score);
  }

  private evaluateJustice(request: EthicalAssessmentRequest): number {
    let score = 8;  // Default moderate

    // Therapeutic augmentation should be accessible
    if (request.augmentation.type === AugmentationType.THERAPEUTIC) {
      if (request.subject.hasInsurance) score += 2;
      else score -= 1;
    }

    // Vulnerable populations
    if (request.subject.isVulnerable) {
      if (request.subject.vulnerableCategories.includes('economically_disadvantaged' as VulnerablePopulation)) {
        score -= 1;  // Access concerns
      }
    }

    return Math.max(0, Math.min(10, score));
  }

  private evaluateDignity(request: EthicalAssessmentRequest): number {
    let score = 9;  // Generally high unless specific concerns

    // Enhancement for employment may compromise dignity
    if (request.context.coercionContext === 'occupational' &&
        request.augmentation.type === AugmentationType.ENHANCEMENT) {
      score -= 2;
    }

    // Vulnerable populations more at risk
    if (request.subject.isVulnerable &&
        request.subject.vulnerableCategories.some(c => ['prisoners', 'institutionalized'].includes(c))) {
      score -= 1;
    }

    return Math.max(0, score);
  }

  private evaluateAuthenticity(request: EthicalAssessmentRequest): number {
    const identityImpact = this.assessIdentityImpact(request.augmentation);

    let score = 10;

    // Identity impact
    if (identityImpact.level === 'severe') score -= 6;
    else if (identityImpact.level === 'substantial') score -= 4;
    else if (identityImpact.level === 'moderate') score -= 2;

    // Cognitive/emotional augmentation higher risk
    if (request.augmentation.category === 'cognitive' || request.augmentation.category === 'emotional') {
      score -= 1;
    }

    // Irreversibility threatens authenticity
    if (request.augmentation.reversibility < 0.5) score -= 1;

    return Math.max(0, score);
  }

  private getSatisfiedPrinciples(scores: PrincipleScores): EthicalPrinciple[] {
    const satisfied: EthicalPrinciple[] = [];
    const threshold = ETHICS_CONSTANTS.PRINCIPLE_THRESHOLDS.SATISFIED;

    if (scores.autonomy >= threshold) satisfied.push(EthicalPrinciple.AUTONOMY);
    if (scores.beneficence >= threshold) satisfied.push(EthicalPrinciple.BENEFICENCE);
    if (scores.nonMaleficence >= threshold) satisfied.push(EthicalPrinciple.NON_MALEFICENCE);
    if (scores.justice >= threshold) satisfied.push(EthicalPrinciple.JUSTICE);
    if (scores.dignity >= threshold) satisfied.push(EthicalPrinciple.DIGNITY);
    if (scores.authenticity >= threshold) satisfied.push(EthicalPrinciple.AUTHENTICITY);

    return satisfied;
  }

  private identifyConcerns(scores: PrincipleScores, request: EthicalAssessmentRequest): EthicalConcern[] {
    const concerns: EthicalConcern[] = [];
    const threshold = ETHICS_CONSTANTS.PRINCIPLE_THRESHOLDS.SATISFIED;
    const warningThreshold = ETHICS_CONSTANTS.PRINCIPLE_THRESHOLDS.WARNING;

    for (const [principle, score] of Object.entries(scores)) {
      if (score < warningThreshold) {
        concerns.push({
          principle: principle.toUpperCase() as EthicalPrinciple,
          severity: 'critical',
          description: `${principle} score critically low: ${score}/10`,
          mitigation: `Address ${principle} concerns before proceeding`,
        });
      } else if (score < threshold) {
        concerns.push({
          principle: principle.toUpperCase() as EthicalPrinciple,
          severity: score < 6 ? 'high' : 'moderate',
          description: `${principle} score below threshold: ${score}/10`,
          mitigation: `Improve ${principle} score through additional safeguards`,
        });
      }
    }

    return concerns;
  }

  private generateRecommendations(concerns: EthicalConcern[], request: EthicalAssessmentRequest): string[] {
    const recommendations: string[] = [];

    // Address critical concerns first
    const critical = concerns.filter(c => c.severity === 'critical');
    if (critical.length > 0) {
      recommendations.push('CRITICAL: Address all critical concerns before proceeding');
      critical.forEach(c => {
        if (c.mitigation) recommendations.push(c.mitigation);
      });
    }

    // High severity concerns
    const high = concerns.filter(c => c.severity === 'high');
    if (high.length > 0) {
      recommendations.push('Implement additional safeguards for high-severity concerns');
    }

    // Consent recommendations
    if (!request.consent) {
      recommendations.push('Obtain appropriate informed consent');
    }

    // Vulnerable population recommendations
    if (request.subject.isVulnerable) {
      recommendations.push('Apply enhanced protections for vulnerable population');
    }

    // Reversibility recommendations
    if (request.augmentation.reversibility < 0.7) {
      recommendations.push('Consider more reversible alternatives if available');
    }

    return recommendations;
  }

  private determineRequiredActions(concerns: EthicalConcern[], request: EthicalAssessmentRequest): RequiredAction[] {
    const actions: RequiredAction[] = [];

    // Critical concerns require immediate action
    concerns.filter(c => c.severity === 'critical').forEach(c => {
      actions.push({
        action: c.mitigation || `Resolve ${c.principle} violation`,
        priority: 'critical',
        responsible: 'Ethics Committee',
        status: 'pending',
      });
    });

    // Consent required
    if (!request.consent || !request.consent.capacity.evaluated) {
      actions.push({
        action: 'Obtain comprehensive informed consent',
        priority: 'high',
        responsible: 'Provider',
        status: 'pending',
      });
    }

    // Vulnerable population protections
    if (request.subject.isVulnerable) {
      actions.push({
        action: 'Implement vulnerable population protections',
        priority: 'high',
        responsible: 'Ethics Committee',
        status: 'pending',
      });
    }

    return actions;
  }

  private determineApprovalLevel(scores: PrincipleScores, request: EthicalAssessmentRequest): 'automatic' | 'standard' | 'enhanced' | 'prohibited' {
    // Prohibited if any principle critically low
    if (Object.values(scores).some(s => s < ETHICS_CONSTANTS.PRINCIPLE_THRESHOLDS.WARNING)) {
      return 'prohibited';
    }

    // Enhanced review for enhancement, experimental, or vulnerable
    if (
      request.augmentation.type === AugmentationType.ENHANCEMENT ||
      request.augmentation.type === AugmentationType.EXPERIMENTAL ||
      request.subject.isVulnerable ||
      request.augmentation.reversibility < 0.5
    ) {
      return 'enhanced';
    }

    // Standard review for restorative or moderate concerns
    if (
      request.augmentation.type === AugmentationType.RESTORATIVE ||
      Object.values(scores).some(s => s < ETHICS_CONSTANTS.PRINCIPLE_THRESHOLDS.SATISFIED)
    ) {
      return 'standard';
    }

    // Automatic approval if all principles satisfied and therapeutic
    if (request.augmentation.type === AugmentationType.THERAPEUTIC && this.isCompliant(scores)) {
      return 'automatic';
    }

    return 'standard';
  }

  private isCompliant(scores: PrincipleScores): boolean {
    const values = Object.values(scores);
    const allSatisfied = values.every(s => s >= ETHICS_CONSTANTS.PRINCIPLE_THRESHOLDS.SATISFIED);
    const averageHigh = values.reduce((a, b) => a + b, 0) / values.length >= ETHICS_CONSTANTS.OVERALL_COMPLIANCE_THRESHOLD;
    return allSatisfied && averageHigh;
  }

  private validateConsentQuick(consent: ConsentDocumentation, type: AugmentationType): boolean {
    const requiredLevel = this.getRequiredConsentLevel(type);
    return this.getConsentLevelOrder(consent.level) >= this.getConsentLevelOrder(requiredLevel) &&
           consent.capacity.hasCapacity &&
           !consent.voluntariness.coercionDetected;
  }

  private checkVulnerableProtection(request: EthicalAssessmentRequest): boolean {
    const protections = this.protectVulnerable(request.subject);

    // Check if enhancement is prohibited but requested
    if (protections.restrictions.enhancementProhibited &&
        request.augmentation.type === AugmentationType.ENHANCEMENT) {
      return false;
    }

    // Check if experimental is prohibited but requested
    if (protections.restrictions.experimentalProhibited &&
        request.augmentation.type === AugmentationType.EXPERIMENTAL) {
      return false;
    }

    return true;
  }

  private getRequiredConsentLevel(type: AugmentationType): ConsentLevel {
    const mapping: Record<AugmentationType, ConsentLevel> = {
      [AugmentationType.THERAPEUTIC]: ConsentLevel.BASIC,
      [AugmentationType.RESTORATIVE]: ConsentLevel.ENHANCED,
      [AugmentationType.ENHANCEMENT]: ConsentLevel.COMPREHENSIVE,
      [AugmentationType.EXPERIMENTAL]: ConsentLevel.EXPERIMENTAL,
    };
    return mapping[type];
  }

  private getConsentLevelOrder(level: ConsentLevel): number {
    const order: Record<ConsentLevel, number> = {
      [ConsentLevel.BASIC]: 1,
      [ConsentLevel.ENHANCED]: 2,
      [ConsentLevel.COMPREHENSIVE]: 3,
      [ConsentLevel.EXPERIMENTAL]: 4,
    };
    return order[level];
  }

  private getRequiredDisclosures(level: ConsentLevel): Record<string, boolean> {
    const base = {
      nature: true,
      benefits: true,
      risks: true,
      alternatives: true,
      longTerm: false,
      identity: false,
      social: false,
      reversibility: true,
    };

    if (level === ConsentLevel.ENHANCED || level === ConsentLevel.COMPREHENSIVE || level === ConsentLevel.EXPERIMENTAL) {
      base.longTerm = true;
      base.reversibility = true;
    }

    if (level === ConsentLevel.COMPREHENSIVE || level === ConsentLevel.EXPERIMENTAL) {
      base.identity = true;
      base.social = true;
    }

    return base;
  }

  private getRequiredCoolingOffPeriod(level: ConsentLevel): number {
    return ETHICS_CONSTANTS.COOLING_OFF_PERIODS[level];
  }

  private assessAccessBarriers(context: EquityContext): AccessBarriers {
    return {
      economic: context.cost > 100000 ? 9 : context.cost > 50000 ? 7 : context.cost > 10000 ? 5 : 3,
      geographic: context.availability === 'rare' ? 8 : context.availability === 'limited' ? 6 : 3,
      social: context.augmentationType === AugmentationType.ENHANCEMENT ? 5 : 3,
      systemic: !context.insuranceCoverage && context.augmentationType === AugmentationType.THERAPEUTIC ? 7 : 4,
    };
  }

  private calculateDistributionMetric(gini: number): DistributionMetric {
    return {
      giniCoefficient: gini,
      representationRatio: 1 - gini,
      disparityScore: gini * 100,
    };
  }

  private estimateIdentityDimensions(augmentation: AugmentationDetails): IdentityDimensions {
    const base = {
      psychological: { memory: 2, personality: 2, consciousness: 1, emotions: 2 },
      physical: { embodiment: 3, appearance: 2, capabilities: 4, sensorimotor: 3 },
      narrative: { continuity: 2, meaning: 2, autobiography: 1 },
      social: { relationships: 2, roles: 2, community: 2, identity: 2 },
      values: { beliefs: 1, commitments: 1, goals: 2, worldview: 1 },
    };

    // Adjust based on augmentation type and category
    if (augmentation.category === 'cognitive') {
      base.psychological.memory += 4;
      base.psychological.personality += 3;
      base.psychological.consciousness += 5;
      base.values.beliefs += 3;
    }

    if (augmentation.category === 'physical') {
      base.physical.embodiment += 3;
      base.physical.capabilities += 2;
      base.physical.sensorimotor += 3;
    }

    if (augmentation.category === 'emotional') {
      base.psychological.emotions += 5;
      base.psychological.personality += 4;
      base.values.beliefs += 2;
    }

    // Reversibility affects all dimensions
    const irreversibilityFactor = (1 - augmentation.reversibility) * 2;
    Object.values(base).forEach(dimension => {
      Object.keys(dimension).forEach(key => {
        dimension[key as keyof typeof dimension] += irreversibilityFactor;
        dimension[key as keyof typeof dimension] = Math.min(10, dimension[key as keyof typeof dimension]);
      });
    });

    return base;
  }

  private calculateIdentityScore(dimensions: IdentityDimensions): number {
    const weights = {
      psychological: 0.25,
      physical: 0.15,
      narrative: 0.20,
      social: 0.20,
      values: 0.20,
    };

    let score = 0;
    for (const [dim, weight] of Object.entries(weights)) {
      const dimValues = Object.values(dimensions[dim as keyof IdentityDimensions]);
      const dimAvg = dimValues.reduce((a, b) => a + b, 0) / dimValues.length;
      score += dimAvg * weight * 10;
    }

    return score;
  }

  private estimateReversalDuration(augmentation: AugmentationDetails): number {
    const baseDays = augmentation.invasiveness === 'high' ? 30 : augmentation.invasiveness === 'moderate' ? 14 : 7;
    return baseDays;
  }

  private estimateReversalRisk(augmentation: AugmentationDetails): 'low' | 'moderate' | 'high' {
    if (augmentation.invasiveness === 'high' || augmentation.reversibility < 0.5) return 'high';
    if (augmentation.invasiveness === 'moderate' || augmentation.reversibility < 0.7) return 'moderate';
    return 'low';
  }

  private estimateReversalCost(augmentation: AugmentationDetails): number {
    // Estimate as 50-75% of original cost
    const baseCost = 50000;
    return baseCost * (augmentation.invasiveness === 'high' ? 0.75 : 0.5);
  }

  private estimatePhysicalRecovery(augmentation: AugmentationDetails): number {
    const days: Record<string, number> = { none: 0, minimal: 3, moderate: 14, high: 30 };
    return days[augmentation.invasiveness] || 7;
  }

  private estimatePsychologicalRecovery(augmentation: AugmentationDetails): number {
    return augmentation.category === 'cognitive' || augmentation.category === 'emotional' ? 60 : 30;
  }

  private estimateFunctionalRecovery(augmentation: AugmentationDetails): number {
    return this.estimatePhysicalRecovery(augmentation) + 7;
  }

  private generateReportSummary(
    ethical: EthicalAssessment,
    consent: ConsentValidationResult,
    equity: EquityAssessment,
    coercion: CoercionCheckResult,
    identity: IdentityImpact,
    reversibility: ReversibilityProfile,
    recommendation: 'proceed' | 'conditional' | 'do_not_proceed'
  ): string {
    const parts: string[] = [];

    parts.push(`Ethical Assessment: ${ethical.compliant ? 'COMPLIANT' : 'NON-COMPLIANT'}`);
    parts.push(`Principles Satisfied: ${ethical.principlesSatisfied.length}/6`);

    if (!consent.valid) parts.push('Consent: INVALID');
    if (coercion.coercionDetected) parts.push(`Coercion: ${coercion.riskLevel.toUpperCase()} RISK`);
    if (identity.level === 'severe' || identity.level === 'substantial') {
      parts.push(`Identity Impact: ${identity.level.toUpperCase()}`);
    }
    if (reversibility.level === ReversibilityLevel.IRREVERSIBLE) {
      parts.push('Reversibility: IRREVERSIBLE');
    }

    parts.push(`Recommendation: ${recommendation.toUpperCase().replace('_', ' ')}`);

    return parts.join(' | ');
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Assess ethical compliance (standalone)
 */
export function assessEthicalCompliance(request: EthicalAssessmentRequest): EthicalAssessment {
  const sdk = new AugmentationEthicsSDK();
  return sdk.assessEthicalCompliance(request);
}

/**
 * Validate consent (standalone)
 */
export function validateConsent(request: ConsentValidationRequest): ConsentValidationResult {
  const sdk = new AugmentationEthicsSDK();
  return sdk.validateConsent(request);
}

/**
 * Evaluate equity (standalone)
 */
export function evaluateEquity(context: EquityContext): EquityAssessment {
  const sdk = new AugmentationEthicsSDK();
  return sdk.evaluateEquity(context);
}

/**
 * Check coercion (standalone)
 */
export function checkCoercion(context: CoercionContext, indicators: CoercionIndicators): CoercionCheckResult {
  const sdk = new AugmentationEthicsSDK();
  return sdk.checkCoercion(context, indicators);
}

/**
 * Assess identity impact (standalone)
 */
export function assessIdentityImpact(augmentation: AugmentationDetails): IdentityImpact {
  const sdk = new AugmentationEthicsSDK();
  return sdk.assessIdentityImpact(augmentation);
}

/**
 * Review reversibility (standalone)
 */
export function reviewReversibility(augmentation: AugmentationDetails): ReversibilityProfile {
  const sdk = new AugmentationEthicsSDK();
  return sdk.reviewReversibility(augmentation);
}

/**
 * Protect vulnerable populations (standalone)
 */
export function protectVulnerable(subject: SubjectProfile): VulnerableProtections {
  const sdk = new AugmentationEthicsSDK();
  return sdk.protectVulnerable(subject);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { AugmentationEthicsSDK };
export default AugmentationEthicsSDK;
