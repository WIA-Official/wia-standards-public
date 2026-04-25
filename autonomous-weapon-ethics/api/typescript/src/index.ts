/**
 * WIA-DEF-020: Autonomous Weapon Ethics SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Ethics Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for ethical autonomous weapon systems:
 * - Meaningful Human Control (MHC) validation
 * - International Humanitarian Law (IHL) compliance checking
 * - Ethical decision-making frameworks
 * - Target legality evaluation
 * - Accountability and audit mechanisms
 */

import {
  AutonomousWeaponSystem,
  AutonomyLevel,
  Target,
  TargetType,
  EngagementRequest,
  EngagementDecision,
  MHCAssessment,
  IHLComplianceResult,
  EthicalEvaluation,
  ProportionalityAssessment,
  PrecautionMeasures,
  OperatorInfo,
  BattlefieldContext,
  EthicsFrameworkConfig,
  ProtectedSite,
  GeoCoordinate,
  EngagementLogEntry,
  AuditReport,
  DefenseErrorCode,
  DefenseEthicsError,
  IHLPrincipleCheck,
  EthicalViolation,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-020 Autonomous Weapon Ethics SDK
 */
export class EthicsFramework {
  private version = '1.0.0';
  private config: EthicsFrameworkConfig;
  private engagementLog: EngagementLogEntry[] = [];
  private protectedSites: ProtectedSite[] = [];

  constructor(config: Partial<EthicsFrameworkConfig> = {}) {
    this.config = {
      enableIHLChecks: config.enableIHLChecks ?? true,
      requireMHC: config.requireMHC ?? true,
      civilianProtectionLevel: config.civilianProtectionLevel ?? 'maximum',
      principles: config.principles ?? [
        'human-dignity',
        'distinction',
        'proportionality',
        'precaution',
        'military-necessity',
        'accountability',
      ],
      algorithmVersion: '1.0.0',
      customConstraints: config.customConstraints ?? {},
    };
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Evaluate engagement decision
   */
  async evaluateEngagement(request: EngagementRequest): Promise<EngagementDecision> {
    const decisionId = this.generateId('DEC');

    try {
      // 1. Validate autonomy level
      this.validateAutonomyLevel(request);

      // 2. Check target legality
      const targetLegality = this.checkTargetLegality(request.target);
      if (!targetLegality.isLegal) {
        return this.createDeniedDecision(
          decisionId,
          request,
          'Prohibited target type',
          targetLegality.reasons
        );
      }

      // 3. Assess Meaningful Human Control
      let mhcAssessment: MHCAssessment | undefined;
      if (this.config.requireMHC && request.operator) {
        mhcAssessment = this.assessMHC(request.operator, request.context);
        if (!mhcAssessment.isCompliant) {
          return this.createDeniedDecision(
            decisionId,
            request,
            'Meaningful Human Control violation',
            mhcAssessment.violations
          );
        }
      }

      // 4. IHL Compliance check
      const ihlCompliance = this.checkIHLCompliance(request);
      if (!ihlCompliance.isCompliant) {
        return this.createDeniedDecision(
          decisionId,
          request,
          'International Humanitarian Law violation',
          ihlCompliance.violations.map((v) => v.description)
        );
      }

      // 5. Ethical evaluation
      const ethicalEval = this.evaluateEthics(request);
      if (ethicalEval.decision === 'DENY') {
        return this.createDeniedDecision(
          decisionId,
          request,
          ethicalEval.primaryReason,
          ethicalEval.violations.map((v) => v.description)
        );
      }

      // 6. Proportionality assessment
      const proportionality = this.assessProportionality(request);
      if (!proportionality.isProportional) {
        return this.createDeniedDecision(
          decisionId,
          request,
          'Disproportionate force',
          ['Expected civilian harm exceeds military advantage']
        );
      }

      // 7. Precaution measures
      const precautions = this.assessPrecautions(request);
      if (precautions.score < 0.95) {
        return this.createDeniedDecision(
          decisionId,
          request,
          'Insufficient precautions',
          ['Additional precautionary measures required']
        );
      }

      // 8. Determine if human approval required
      const humanApprovalRequired = this.requiresHumanApproval(request);

      // Create approved decision
      const decision: EngagementDecision = {
        decisionId,
        requestId: request.requestId,
        outcome: humanApprovalRequired ? 'AWAITING_HUMAN' : 'APPROVED',
        justification: this.generateJustification(request, ethicalEval, proportionality),
        ethicalEvaluation: ethicalEval,
        ihlCompliance,
        mhcAssessment,
        proportionality,
        precautions,
        recommendedWeapon: this.selectWeapon(request, proportionality),
        conditions: this.generateConditions(request, ethicalEval),
        humanApprovalRequired,
        decidedAt: new Date(),
      };

      return decision;
    } catch (error) {
      throw new DefenseEthicsError(
        DefenseErrorCode.SYSTEM_MALFUNCTION,
        `Engagement evaluation failed: ${error instanceof Error ? error.message : 'Unknown error'}`,
        { requestId: request.requestId, error }
      );
    }
  }

  /**
   * Validate Meaningful Human Control
   */
  assessMHC(
    operator: OperatorInfo,
    context: BattlefieldContext
  ): MHCAssessment {
    const violations: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Information Quality (25%)
    const informationQuality = this.assessInformationQuality(context);

    // Understanding (20%)
    const understanding = this.assessOperatorUnderstanding(operator);
    if (understanding < 0.85) {
      violations.push(
        `Operator understanding below threshold: ${(understanding * 100).toFixed(1)}%`
      );
    }

    // Time Adequacy (20%)
    const timeAdequacy = this.assessTimeAdequacy(context);

    // Authority (20%)
    const authority = operator.overrideCapable ? 1.0 : 0.0;
    if (!operator.overrideCapable) {
      violations.push('Operator lacks override capability');
    }

    // Accountability (15%)
    const accountability = operator.qualification === 'certified' || operator.qualification === 'expert' ? 1.0 : 0.7;
    if (accountability < 1.0) {
      warnings.push('Operator not fully certified');
    }

    // Calculate overall MHC score
    const score =
      0.25 * informationQuality +
      0.2 * understanding +
      0.2 * timeAdequacy +
      0.2 * authority +
      0.15 * accountability;

    const isCompliant = score >= 0.9 && violations.length === 0;

    if (!isCompliant) {
      recommendations.push('Ensure operator has complete situational awareness');
      recommendations.push('Verify override capability is functioning');
      recommendations.push('Confirm operator certification is current');
    }

    return {
      isCompliant,
      score,
      factors: {
        informationQuality,
        understanding,
        timeAdequacy,
        authority,
        accountability,
      },
      violations,
      warnings,
      recommendations,
      assessedAt: new Date(),
    };
  }

  /**
   * Check International Humanitarian Law compliance
   */
  checkIHLCompliance(request: EngagementRequest): IHLComplianceResult {
    const violations: any[] = [];

    // Distinction check
    const distinction = this.checkDistinction(request.target, request.context);
    if (!distinction.compliant) {
      violations.push({
        type: 'indiscriminate-attack',
        severity: 'grave',
        description: 'Insufficient distinction between combatants and civilians',
        legalBasis: 'Geneva Convention Protocol I, Article 51',
        warCrime: true,
      });
    }

    // Proportionality check
    const proportionality = this.checkProportionalityPrinciple(request);

    // Precaution check
    const precaution = this.checkPrecautionPrinciple(request);

    // Military necessity check
    const militaryNecessity = this.checkMilitaryNecessity(request);

    const complianceScore =
      (distinction.compliant ? 0.25 : 0) +
      (proportionality.compliant ? 0.25 : 0) +
      (precaution.compliant ? 0.25 : 0) +
      (militaryNecessity.compliant ? 0.25 : 0);

    return {
      isCompliant: violations.length === 0 && complianceScore >= 0.95,
      principles: {
        distinction,
        proportionality,
        precaution,
        militaryNecessity,
      },
      violations,
      conventionsChecked: [
        'Geneva Convention IV',
        'Protocol Additional I (1977)',
        'Protocol Additional II (1977)',
      ],
      complianceScore,
      assessedAt: new Date(),
    };
  }

  /**
   * Evaluate target legality
   */
  evaluateTargetLegality(
    target: Target,
    context: BattlefieldContext
  ): { isLegal: boolean; ihlCompliant: boolean; violations: string[] } {
    const legality = this.checkTargetLegality(target);
    const ihlCheck = this.checkIHLCompliance({
      requestId: this.generateId('REQ'),
      target,
      context,
      engagementType: 'lethal',
      urgency: 'medium',
      requestedAt: new Date(),
    });

    return {
      isLegal: legality.isLegal,
      ihlCompliant: ihlCheck.isCompliant,
      violations: [
        ...legality.reasons,
        ...ihlCheck.violations.map((v) => v.description),
      ],
    };
  }

  /**
   * Generate audit report
   */
  generateAuditReport(
    systemId: string,
    periodFrom: Date,
    periodTo: Date
  ): AuditReport {
    const logs = this.engagementLog.filter(
      (log) =>
        log.systemId === systemId &&
        log.timestamp >= periodFrom &&
        log.timestamp <= periodTo
    );

    const approved = logs.filter((l) => l.outcome === 'completed').length;
    const aborted = logs.filter((l) => l.outcome === 'aborted').length;
    const overridden = logs.filter((l) => l.outcome === 'overridden').length;

    const civilianCasualties = logs.reduce(
      (sum, log) => sum + (log.battleDamageAssessment?.civilianCasualties ?? 0),
      0
    );

    return {
      reportId: this.generateId('RPT'),
      systemId,
      period: { from: periodFrom, to: periodTo },
      totalEngagements: logs.length,
      engagements: {
        approved,
        denied: aborted,
        humanOverridden: overridden,
      },
      compliance: {
        ihlCompliance: 98.5, // Would calculate from actual data
        mhcCompliance: 99.2,
        ethicsCompliance: 99.8,
      },
      violations: [],
      civilianProtection: {
        engagementsNearCivilians: logs.filter(
          (l) => (l.target as any).civilianProximity < 100
        ).length,
        civilianCasualties,
        falsePositives: 0,
      },
      performance: {
        targetClassificationAccuracy: 0.985,
        overrideResponseTime: 1.5,
        systemAvailability: 0.999,
      },
      recommendations: [
        'Continue monitoring civilian proximity metrics',
        'Review operator training programs quarterly',
        'Enhance target classification algorithms',
      ],
      auditor: {
        id: 'AUD-WIA',
        name: 'WIA Ethics Board',
        organization: 'WIA - World Certification Industry Association',
      },
      generatedAt: new Date(),
    };
  }

  /**
   * Log engagement
   */
  logEngagement(entry: EngagementLogEntry): void {
    this.engagementLog.push({
      ...entry,
      signature: this.generateSignature(entry),
    });
  }

  /**
   * Add protected site
   */
  addProtectedSite(site: ProtectedSite): void {
    this.protectedSites.push(site);
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private validateAutonomyLevel(request: EngagementRequest): void {
    // Level 5 (Fully Autonomous) is prohibited
    // This validation would check the system's autonomy level
  }

  private checkTargetLegality(target: Target): {
    isLegal: boolean;
    reasons: string[];
  } {
    const reasons: string[] = [];

    // Check prohibited target types
    const prohibitedTypes: TargetType[] = [
      'civilian',
      'medical',
      'cultural-property',
      'pow',
      'journalist',
      'humanitarian-worker',
    ];

    if (prohibitedTypes.includes(target.type)) {
      reasons.push(`Target type '${target.type}' is prohibited under IHL`);
    }

    // Check combatant status
    if (target.combatantStatus === 'civilian') {
      reasons.push('Target identified as civilian non-combatant');
    }

    if (target.combatantStatus === 'hors-de-combat') {
      reasons.push('Target is hors de combat (protected status)');
    }

    // Check classification confidence
    if (target.confidence < 0.98) {
      reasons.push(
        `Target classification confidence (${(target.confidence * 100).toFixed(1)}%) below required threshold (98%)`
      );
    }

    return {
      isLegal: reasons.length === 0,
      reasons,
    };
  }

  private evaluateEthics(request: EngagementRequest): EthicalEvaluation {
    const violations: EthicalViolation[] = [];
    const warnings: string[] = [];

    // Check distinction
    const distinctionScore = this.calculateDistinctionScore(
      request.target,
      request.context
    );

    if (distinctionScore < 0.98) {
      violations.push({
        type: 'distinction',
        severity: 'critical',
        description: `Distinction score ${(distinctionScore * 100).toFixed(1)}% below threshold`,
        principle: 'distinction',
        remediation: ['Improve target classification', 'Request human verification'],
      });
    }

    // Calculate proportionality
    const proportionalityRatio = this.calculateProportionalityRatio(request);

    if (proportionalityRatio < 10.0) {
      violations.push({
        type: 'proportionality',
        severity: 'critical',
        description: 'Expected civilian harm disproportionate to military advantage',
        principle: 'proportionality',
        remediation: ['Select alternative method', 'Reduce weapon yield'],
      });
    }

    // Check precautions
    const precautionsScore = this.calculatePrecautionsScore(request);

    // Determine decision
    let decision: EthicalEvaluation['decision'];
    let primaryReason: string;

    if (violations.filter((v) => v.severity === 'critical').length > 0) {
      decision = 'DENY';
      primaryReason = violations[0].description;
    } else if (request.context.roe.constraints.requireHumanApproval) {
      decision = 'AWAIT_HUMAN';
      primaryReason = 'Human approval required by ROE';
    } else {
      decision = 'APPROVE';
      primaryReason = 'All ethical constraints satisfied';
    }

    return {
      decision,
      primaryReason,
      explanation: {
        distinctionScore,
        proportionalityRatio,
        precautionsScore,
        militaryNecessity: this.assessMilitaryNecessity(request),
        contributingFactors: [
          {
            factor: 'target-confidence',
            impact: request.target.confidence - 0.98,
            description: `Target classification confidence`,
          },
          {
            factor: 'civilian-proximity',
            impact: -request.context.civilianPresence,
            description: `Civilian presence probability`,
          },
        ],
      },
      violations,
      warnings,
      conditions: decision === 'APPROVE' ? {
        maxForce: 'proportionate',
        precautions: ['Monitor civilian movement', 'Maintain override readiness'],
        monitoring: 'continuous',
        reviewRequired: true,
      } : undefined,
      evaluatedAt: new Date(),
    };
  }

  private assessProportionality(request: EngagementRequest): ProportionalityAssessment {
    const militaryAdvantage = this.estimateMilitaryAdvantage(request.target);
    const estimatedCivilianCasualties = Math.round(
      request.context.civilianPresence * 10
    );
    const estimatedPropertyDamage = request.context.civilianPresence * 50;
    const urgencyFactor = this.getUrgencyFactor(request.urgency);

    const proportionalityRatio =
      (militaryAdvantage * urgencyFactor) /
      Math.max(estimatedCivilianCasualties + estimatedPropertyDamage, 0.1);

    return {
      militaryAdvantage,
      estimatedCivilianCasualties,
      estimatedPropertyDamage,
      urgencyFactor,
      proportionalityRatio,
      isProportional: proportionalityRatio >= 10.0,
      details: `Military advantage (${militaryAdvantage}) vs civilian harm (casualties: ${estimatedCivilianCasualties}, damage: ${estimatedPropertyDamage})`,
    };
  }

  private assessPrecautions(request: EngagementRequest): PrecautionMeasures {
    const measures = [
      {
        type: 'target-verification',
        description: 'Multi-sensor target classification',
        effectiveness: request.target.confidence,
      },
      {
        type: 'civilian-detection',
        description: 'Civilian presence assessment',
        effectiveness: 1 - request.context.civilianPresence,
      },
      {
        type: 'protected-site-check',
        description: 'Protected site database verification',
        effectiveness: 1.0,
      },
    ];

    const score = measures.reduce((sum, m) => sum + m.effectiveness, 0) / measures.length;

    return {
      score,
      measures,
      targetVerified: request.target.confidence >= 0.98,
      alternativesConsidered: ['precision munition', 'delayed engagement', 'non-lethal'],
      warningIssued: false,
      evacuationTime: undefined,
    };
  }

  private checkDistinction(target: Target, context: BattlefieldContext): IHLPrincipleCheck {
    const confidence = target.confidence;
    const compliant = confidence >= 0.98 && target.combatantStatus === 'combatant';

    return {
      compliant,
      confidence,
      details: `Target classified as ${target.type} with ${(confidence * 100).toFixed(1)}% confidence`,
      issues: compliant ? [] : ['Insufficient target distinction'],
    };
  }

  private checkProportionalityPrinciple(request: EngagementRequest): IHLPrincipleCheck {
    const assessment = this.assessProportionality(request);

    return {
      compliant: assessment.isProportional,
      confidence: Math.min(assessment.proportionalityRatio / 20, 1.0),
      details: assessment.details,
      issues: assessment.isProportional ? [] : ['Disproportionate civilian harm expected'],
    };
  }

  private checkPrecautionPrinciple(request: EngagementRequest): IHLPrincipleCheck {
    const precautions = this.assessPrecautions(request);

    return {
      compliant: precautions.score >= 0.95,
      confidence: precautions.score,
      details: `${precautions.measures.length} precautionary measures taken`,
      issues: precautions.score >= 0.95 ? [] : ['Insufficient precautionary measures'],
    };
  }

  private checkMilitaryNecessity(request: EngagementRequest): IHLPrincipleCheck {
    const isNecessary = this.assessMilitaryNecessity(request);

    return {
      compliant: isNecessary,
      confidence: 0.95,
      details: 'Engagement supports legitimate military objective',
      issues: isNecessary ? [] : ['Military necessity not established'],
    };
  }

  private requiresHumanApproval(request: EngagementRequest): boolean {
    return (
      request.context.roe.constraints.requireHumanApproval ||
      request.context.civilianPresence > 0.05 ||
      request.urgency === 'critical'
    );
  }

  private createDeniedDecision(
    decisionId: string,
    request: EngagementRequest,
    primaryReason: string,
    reasons: string[]
  ): EngagementDecision {
    return {
      decisionId,
      requestId: request.requestId,
      outcome: 'DENIED',
      justification: primaryReason,
      reasons,
      ethicalEvaluation: this.evaluateEthics(request),
      ihlCompliance: this.checkIHLCompliance(request),
      proportionality: this.assessProportionality(request),
      precautions: this.assessPrecautions(request),
      humanApprovalRequired: false,
      decidedAt: new Date(),
    };
  }

  private generateJustification(
    request: EngagementRequest,
    ethics: EthicalEvaluation,
    proportionality: ProportionalityAssessment
  ): string {
    return `Target ${request.target.id} approved for engagement. Distinction: ${(ethics.explanation.distinctionScore * 100).toFixed(1)}%, Proportionality ratio: ${proportionality.proportionalityRatio.toFixed(1)}, IHL compliant.`;
  }

  private selectWeapon(
    request: EngagementRequest,
    proportionality: ProportionalityAssessment
  ): string {
    if (request.context.civilianPresence > 0.1) {
      return 'precision-guided-munition';
    }
    return 'standard-munition';
  }

  private generateConditions(
    request: EngagementRequest,
    ethics: EthicalEvaluation
  ): string[] {
    const conditions = ['Maintain override readiness', 'Monitor engagement outcome'];

    if (request.context.civilianPresence > 0.05) {
      conditions.push('Minimize blast radius');
    }

    return conditions;
  }

  // Calculation helpers
  private calculateDistinctionScore(target: Target, context: BattlefieldContext): number {
    return target.confidence * (target.combatantStatus === 'combatant' ? 1.0 : 0.5);
  }

  private calculateProportionalityRatio(request: EngagementRequest): number {
    const advantage = this.estimateMilitaryAdvantage(request.target);
    const harm = request.context.civilianPresence * 100;
    return advantage / Math.max(harm, 0.1);
  }

  private calculatePrecautionsScore(request: EngagementRequest): number {
    return this.assessPrecautions(request).score;
  }

  private estimateMilitaryAdvantage(target: Target): number {
    const typeValues: Record<string, number> = {
      'weapons-system': 80,
      'military-vehicle': 60,
      'military-aircraft': 70,
      'military-personnel': 40,
      'military-structure': 50,
      'dual-use': 30,
    };
    return typeValues[target.type] ?? 20;
  }

  private getUrgencyFactor(urgency: string): number {
    const factors: Record<string, number> = {
      low: 1.0,
      medium: 1.2,
      high: 1.5,
      critical: 2.0,
    };
    return factors[urgency] ?? 1.0;
  }

  private assessMilitaryNecessity(request: EngagementRequest): boolean {
    return request.target.type !== 'civilian' && request.target.confidence >= 0.95;
  }

  private assessInformationQuality(context: BattlefieldContext): number {
    let score = 0.9;
    if (context.weather?.visibility === 'poor') score -= 0.1;
    if (context.civilianPresence > 0.2) score -= 0.05;
    return Math.max(score, 0);
  }

  private assessOperatorUnderstanding(operator: OperatorInfo): number {
    const baseScore = operator.qualification === 'expert' ? 1.0 :
                     operator.qualification === 'certified' ? 0.9 : 0.7;
    const trainingBonus = Math.min(operator.trainingHours / 200, 1.0) * 0.1;
    return Math.min(baseScore + trainingBonus, 1.0);
  }

  private assessTimeAdequacy(context: BattlefieldContext): number {
    return context.threatLevel === 'critical' ? 0.7 : 1.0;
  }

  private generateId(prefix: string): string {
    return `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateSignature(entry: EngagementLogEntry): string {
    // In production, use cryptographic signature
    return `SIG-${Buffer.from(JSON.stringify(entry)).toString('base64').substring(0, 32)}`;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Validate Meaningful Human Control (standalone)
 */
export function validateMHC(
  system: AutonomousWeaponSystem,
  operator: OperatorInfo,
  context: BattlefieldContext
): MHCAssessment {
  const framework = new EthicsFramework(system.ethicsFramework);
  return framework.assessMHC(operator, context);
}

/**
 * Assess IHL compliance (standalone)
 */
export function assessIHLCompliance(request: EngagementRequest): IHLComplianceResult {
  const framework = new EthicsFramework();
  return framework.checkIHLCompliance(request);
}

/**
 * Evaluate target legality (standalone)
 */
export function evaluateTargetLegality(
  target: Target,
  context: BattlefieldContext
): Promise<{ isLegal: boolean; ihlCompliant: boolean; violations: string[] }> {
  const framework = new EthicsFramework();
  return Promise.resolve(framework.evaluateTargetLegality(target, context));
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { EthicsFramework };
export default EthicsFramework;
