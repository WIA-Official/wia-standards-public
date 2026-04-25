/**
 * WIA-CORE-005: Hongik Impact Metric SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Core Standards Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive Hongik Impact assessment functionality:
 * - Multi-dimensional impact scoring
 * - Stakeholder reach analysis
 * - Temporal impact tracking
 * - Project comparison and ranking
 * - Certification status evaluation
 */

import {
  ImpactAssessment,
  ImpactDimension,
  ImpactClassification,
  CertificationLevel,
  DimensionScores,
  QuickAssessmentInput,
  QuickAssessmentResult,
  StakeholderAnalysis,
  TemporalFactors,
  ProjectDuration,
  ProjectComparison,
  ProjectRank,
  DimensionComparison,
  ImpactReport,
  Recommendation,
  HongikErrorCode,
  HongikError,
  DIMENSION_WEIGHTS,
  TEMPORAL_FACTORS,
  GLOBAL_POPULATION,
  WCAGLevel,
  WCAG_SCORES,
  AgeGroup,
  VULNERABILITY_FACTORS,
  StakeholderGroup,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-CORE-005 Hongik Impact Metric SDK
 *
 * 弘益人間 - Benefit All Humanity
 */
export class HongikImpactSDK {
  private version = '1.0.0';

  constructor() {
    // Initialize SDK
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Quick Assessment Methods
  // ==========================================================================

  /**
   * Calculate quick Hongik Impact Score from basic inputs
   *
   * @param input - Quick assessment input
   * @returns Quick assessment result
   */
  calculateQuickScore(input: QuickAssessmentInput): QuickAssessmentResult {
    // Validate scores
    this.validateDimensionScores({
      socialGood: input.socialGood,
      accessibility: input.accessibility,
      sustainability: input.sustainability,
      healthWellbeing: input.healthWellbeing,
      economicEquity: input.economicEquity,
      education: input.education,
      innovation: input.innovation,
    });

    // Calculate weighted scores
    const weightedScores = {
      socialGood: input.socialGood * DIMENSION_WEIGHTS[ImpactDimension.SOCIAL_GOOD],
      accessibility:
        input.accessibility * DIMENSION_WEIGHTS[ImpactDimension.ACCESSIBILITY],
      sustainability:
        input.sustainability * DIMENSION_WEIGHTS[ImpactDimension.SUSTAINABILITY],
      healthWellbeing:
        input.healthWellbeing * DIMENSION_WEIGHTS[ImpactDimension.HEALTH_WELLBEING],
      economicEquity:
        input.economicEquity * DIMENSION_WEIGHTS[ImpactDimension.ECONOMIC_EQUITY],
      education: input.education * DIMENSION_WEIGHTS[ImpactDimension.EDUCATION],
      innovation: input.innovation * DIMENSION_WEIGHTS[ImpactDimension.INNOVATION],
    };

    // Sum weighted scores
    const baseScore =
      weightedScores.socialGood +
      weightedScores.accessibility +
      weightedScores.sustainability +
      weightedScores.healthWellbeing +
      weightedScores.economicEquity +
      weightedScores.education +
      weightedScores.innovation;

    // Calculate reach factor
    const reachFactor = this.calculateReachFactor(input.beneficiaries);

    // Calculate temporal factor
    const temporalFactor = this.calculateTemporalFactor(
      input.duration || ProjectDuration.MEDIUM_TERM
    );

    // Calculate total HIS
    const total = baseScore * reachFactor * temporalFactor * 1000;

    // Calculate per-capita impact
    const perCapita = total / input.beneficiaries;

    // Classify
    const classification = this.classifyScore(total);

    return {
      total,
      perCapita,
      classification,
      weightedScores,
      reachFactor,
      temporalFactor,
    };
  }

  /**
   * Assess a project with full details
   *
   * @param params - Assessment parameters
   * @returns Complete impact assessment
   */
  assessProject(params: {
    projectId: string;
    projectName: string;
    dimensions: DimensionScores;
    stakeholders: {
      directBeneficiaries: number;
      indirectBeneficiaries: number;
      futureBeneficiaries?: number;
      groups?: StakeholderGroup[];
    };
    duration?: ProjectDuration;
    lifespanYears?: number;
  }): ImpactAssessment {
    // Validate dimension scores
    this.validateDimensionScores(params.dimensions);

    // Calculate stakeholder analysis
    const stakeholderAnalysis = this.analyzeStakeholders({
      directBeneficiaries: params.stakeholders.directBeneficiaries,
      indirectBeneficiaries: params.stakeholders.indirectBeneficiaries,
      futureBeneficiaries: params.stakeholders.futureBeneficiaries || 0,
      groups: params.stakeholders.groups || [],
    });

    // Calculate temporal factors
    const temporalFactors = this.calculateTemporalFactors({
      duration: params.duration || ProjectDuration.MEDIUM_TERM,
      lifespanYears: params.lifespanYears || 10,
    });

    // Calculate base score
    const baseScore = this.calculateBaseScore(params.dimensions);

    // Calculate HIS
    const hongikScore =
      baseScore * stakeholderAnalysis.reachFactor * temporalFactors.temporalFactor * 1000;

    // Calculate per-capita impact
    const perCapitaImpact = hongikScore / stakeholderAnalysis.totalBeneficiaries;

    // Classify
    const classification = this.classifyScore(hongikScore);

    // Determine certification level
    const certificationLevel = this.determineCertificationLevel(hongikScore);

    // Calculate confidence interval (simplified)
    const confidenceInterval = {
      lower: hongikScore * 0.95,
      upper: hongikScore * 1.05,
    };

    const now = new Date();
    const nextReview = new Date(now);
    nextReview.setMonth(nextReview.getMonth() + 6);

    return {
      projectId: params.projectId,
      projectName: params.projectName,
      hongikScore,
      confidenceInterval,
      classification,
      certificationLevel,
      dimensions: params.dimensions,
      stakeholders: stakeholderAnalysis,
      temporal: temporalFactors,
      perCapitaImpact,
      assessmentDate: now,
      nextReviewDate: nextReview,
      metadata: {
        method: 'self_reported' as any,
        dataSources: [],
        verificationLevel: 'self_assessment' as any,
        assessor: 'Self-Assessment',
        confidence: 0.8,
        dataQuality: 0.75,
      },
    };
  }

  // ==========================================================================
  // Calculation Methods
  // ==========================================================================

  /**
   * Calculate base score from dimension scores
   */
  private calculateBaseScore(dimensions: DimensionScores): number {
    return (
      dimensions.socialGood * DIMENSION_WEIGHTS[ImpactDimension.SOCIAL_GOOD] +
      dimensions.accessibility * DIMENSION_WEIGHTS[ImpactDimension.ACCESSIBILITY] +
      dimensions.sustainability * DIMENSION_WEIGHTS[ImpactDimension.SUSTAINABILITY] +
      dimensions.healthWellbeing *
        DIMENSION_WEIGHTS[ImpactDimension.HEALTH_WELLBEING] +
      dimensions.economicEquity * DIMENSION_WEIGHTS[ImpactDimension.ECONOMIC_EQUITY] +
      dimensions.education * DIMENSION_WEIGHTS[ImpactDimension.EDUCATION] +
      dimensions.innovation * DIMENSION_WEIGHTS[ImpactDimension.INNOVATION]
    );
  }

  /**
   * Calculate stakeholder reach factor (1-10)
   *
   * SRF = 1 + 9 × [log10(AB) / log10(8B)]
   */
  calculateReachFactor(affectedBeneficiaries: number): number {
    if (affectedBeneficiaries <= 0) {
      throw new HongikError(
        HongikErrorCode.INSUFFICIENT_BENEFICIARIES,
        'Beneficiaries must be greater than 0'
      );
    }

    const logAB = Math.log10(affectedBeneficiaries);
    const logGlobal = Math.log10(GLOBAL_POPULATION);
    const srf = 1 + 9 * (logAB / logGlobal);

    return Math.max(1, Math.min(10, srf));
  }

  /**
   * Calculate temporal factor from project duration
   */
  calculateTemporalFactor(duration: ProjectDuration): number {
    return TEMPORAL_FACTORS[duration];
  }

  /**
   * Calculate temporal factors with trajectory
   */
  private calculateTemporalFactors(params: {
    duration: ProjectDuration;
    lifespanYears: number;
  }): TemporalFactors {
    const temporalFactor = this.calculateTemporalFactor(params.duration);

    return {
      duration: params.duration,
      lifespanYears: params.lifespanYears,
      temporalFactor,
    };
  }

  /**
   * Analyze stakeholders
   */
  private analyzeStakeholders(params: {
    directBeneficiaries: number;
    indirectBeneficiaries: number;
    futureBeneficiaries: number;
    groups: StakeholderGroup[];
  }): StakeholderAnalysis {
    const totalBeneficiaries =
      params.directBeneficiaries +
      params.indirectBeneficiaries +
      params.futureBeneficiaries;

    // Calculate weighted beneficiaries
    let weightedBeneficiaries = totalBeneficiaries;
    if (params.groups.length > 0) {
      weightedBeneficiaries = params.groups.reduce(
        (sum, group) => sum + group.count * group.vulnerabilityFactor,
        0
      );
    }

    const reachFactor = this.calculateReachFactor(totalBeneficiaries);

    return {
      directBeneficiaries: params.directBeneficiaries,
      indirectBeneficiaries: params.indirectBeneficiaries,
      futureBeneficiaries: params.futureBeneficiaries,
      totalBeneficiaries,
      weightedBeneficiaries,
      groups: params.groups,
      reachFactor,
    };
  }

  /**
   * Classify score into impact level
   */
  classifyScore(score: number): ImpactClassification {
    if (score >= 900) return ImpactClassification.EXCEPTIONAL;
    if (score >= 800) return ImpactClassification.ELITE;
    if (score >= 700) return ImpactClassification.HIGH;
    if (score >= 600) return ImpactClassification.GOOD;
    if (score >= 500) return ImpactClassification.MODERATE;
    if (score >= 400) return ImpactClassification.FAIR;
    if (score >= 300) return ImpactClassification.LIMITED;
    if (score >= 200) return ImpactClassification.MINIMAL;
    if (score >= 100) return ImpactClassification.POOR;
    return ImpactClassification.HARMFUL;
  }

  /**
   * Determine certification level from score
   */
  determineCertificationLevel(score: number): CertificationLevel {
    if (score >= 900) return CertificationLevel.DIAMOND;
    if (score >= 800) return CertificationLevel.PLATINUM;
    if (score >= 700) return CertificationLevel.GOLD;
    if (score >= 600) return CertificationLevel.SILVER;
    if (score >= 400) return CertificationLevel.BRONZE;
    return CertificationLevel.NONE;
  }

  // ==========================================================================
  // Accessibility Calculations
  // ==========================================================================

  /**
   * Calculate accessibility score from WCAG level and other factors
   */
  calculateAccessibilityScore(params: {
    wcagLevel: WCAGLevel;
    languagesSupported: number;
    offlineSupport: boolean;
    lowBandwidth: boolean;
    economicAccessibility: number; // 0-1
  }): number {
    // Digital accessibility (WCAG) - 25%
    const wcagScore = WCAG_SCORES[params.wcagLevel];
    const digitalScore = wcagScore * 0.25;

    // Language accessibility - 15%
    const languageScore = Math.min(params.languagesSupported / 50, 1.0) * 0.15;

    // Economic accessibility - 20%
    const economicScore = params.economicAccessibility * 0.2;

    // Geographic accessibility - 20%
    let geoScore = 0.5; // Base score
    if (params.lowBandwidth) geoScore += 0.25;
    if (params.offlineSupport) geoScore += 0.25;
    geoScore *= 0.2;

    // Physical accessibility (assumed average) - 20%
    const physicalScore = 0.7 * 0.2;

    return digitalScore + languageScore + economicScore + geoScore + physicalScore;
  }

  /**
   * Calculate language coverage score
   */
  calculateLanguageCoverage(supportedLanguages: string[]): number {
    // Top 20 languages cover ~80% of global population
    const top20Languages = [
      'en',
      'zh',
      'hi',
      'es',
      'ar',
      'bn',
      'pt',
      'ru',
      'ja',
      'pa',
      'de',
      'jv',
      'ko',
      'fr',
      'te',
      'mr',
      'tr',
      'ta',
      'vi',
      'ur',
    ];

    const coverage = supportedLanguages.filter((lang) =>
      top20Languages.includes(lang.toLowerCase())
    ).length;

    return Math.min(coverage / 20, 1.0);
  }

  // ==========================================================================
  // Comparison Methods
  // ==========================================================================

  /**
   * Compare multiple projects
   */
  compareProjects(projects: ImpactAssessment[]): ProjectComparison {
    if (projects.length === 0) {
      throw new HongikError(
        HongikErrorCode.MISSING_REQUIRED_DATA,
        'At least one project required for comparison'
      );
    }

    // Rank projects by HIS
    const ranking: ProjectRank[] = projects
      .map((p) => ({
        rank: 0,
        projectId: p.projectId,
        projectName: p.projectName,
        score: p.hongikScore,
        classification: p.classification,
      }))
      .sort((a, b) => b.score - a.score)
      .map((r, index) => ({ ...r, rank: index + 1 }));

    // Dimension-by-dimension comparison
    const dimensions = Object.values(ImpactDimension);
    const dimensionComparison: DimensionComparison[] = dimensions.map((dim) => {
      const scores = projects.map((p) => ({
        projectId: p.projectId,
        score: this.getDimensionScore(p.dimensions, dim),
      }));

      const sortedScores = [...scores].sort((a, b) => b.score - a.score);
      const leader = sortedScores[0].projectId;
      const average =
        scores.reduce((sum, s) => sum + s.score, 0) / scores.length;

      return {
        dimension: dim,
        scores,
        leader,
        average,
      };
    });

    // Generate insights
    const insights = this.generateComparisonInsights(projects, ranking);

    return {
      projects,
      ranking,
      dimensionComparison,
      insights,
    };
  }

  /**
   * Rank projects by impact
   */
  rankByImpact(projects: ImpactAssessment[]): ProjectRank[] {
    return this.compareProjects(projects).ranking;
  }

  /**
   * Get dimension score from DimensionScores
   */
  private getDimensionScore(scores: DimensionScores, dimension: ImpactDimension): number {
    switch (dimension) {
      case ImpactDimension.SOCIAL_GOOD:
        return scores.socialGood;
      case ImpactDimension.ACCESSIBILITY:
        return scores.accessibility;
      case ImpactDimension.SUSTAINABILITY:
        return scores.sustainability;
      case ImpactDimension.HEALTH_WELLBEING:
        return scores.healthWellbeing;
      case ImpactDimension.ECONOMIC_EQUITY:
        return scores.economicEquity;
      case ImpactDimension.EDUCATION:
        return scores.education;
      case ImpactDimension.INNOVATION:
        return scores.innovation;
      default:
        return 0;
    }
  }

  /**
   * Generate comparison insights
   */
  private generateComparisonInsights(
    projects: ImpactAssessment[],
    ranking: ProjectRank[]
  ): string[] {
    const insights: string[] = [];

    // Overall leader
    if (ranking.length > 0) {
      const leader = ranking[0];
      insights.push(
        `${leader.projectName} leads with HIS of ${leader.score.toFixed(1)}/1000 (${leader.classification})`
      );
    }

    // Score distribution
    const scores = projects.map((p) => p.hongikScore);
    const avgScore = scores.reduce((a, b) => a + b, 0) / scores.length;
    const maxScore = Math.max(...scores);
    const minScore = Math.min(...scores);
    insights.push(
      `Score range: ${minScore.toFixed(1)} - ${maxScore.toFixed(1)} (avg: ${avgScore.toFixed(1)})`
    );

    // Beneficiaries
    const totalBeneficiaries = projects.reduce(
      (sum, p) => sum + p.stakeholders.totalBeneficiaries,
      0
    );
    insights.push(
      `Collective impact: ${totalBeneficiaries.toLocaleString()} beneficiaries`
    );

    return insights;
  }

  // ==========================================================================
  // Report Generation
  // ==========================================================================

  /**
   * Generate impact report
   */
  generateReport(assessment: ImpactAssessment): ImpactReport {
    const recommendations = this.generateRecommendations(assessment);

    return {
      reportId: `RPT-${Date.now()}`,
      generatedAt: new Date(),
      period: {
        start: assessment.assessmentDate,
        end: assessment.nextReviewDate,
      },
      assessment,
      detailedMetrics: {
        socialGood: {
          communityEngagement: 0.8,
          socialEquity: 0.85,
          publicBenefit: 0.9,
          culturalImpact: 0.75,
          overall: assessment.dimensions.socialGood,
        },
        accessibility: {
          physical: 0.7,
          digital: 0.85,
          language: 0.8,
          economic: 0.75,
          geographic: 0.7,
          wcagLevel: WCAGLevel.AA_FULL,
          languagesSupported: 10,
          offlineSupport: true,
          lowBandwidth: true,
          overall: assessment.dimensions.accessibility,
        },
        sustainability: {
          carbonFootprint: 0.85,
          netEmissions: -100,
          resourceEfficiency: 0.8,
          ecosystemImpact: 0.75,
          longTermViability: 0.9,
          adaptability: 0.85,
          renewableEnergyPercent: 80,
          overall: assessment.dimensions.sustainability,
        },
        health: {
          diseasePrevention: 0.9,
          mentalHealth: 0.8,
          nutrition: 0.75,
          safety: 0.85,
          qualityOfLife: 0.88,
          overall: assessment.dimensions.healthWellbeing,
        },
        economicEquity: {
          incomeEquality: 0.7,
          wealthDistribution: 0.65,
          economicOpportunity: 0.8,
          povertyReduction: 0.85,
          livingWage: true,
          overall: assessment.dimensions.economicEquity,
        },
        education: {
          access: 0.85,
          quality: 0.8,
          knowledgeSharing: 0.9,
          capacityBuilding: 0.75,
          learnersReached: assessment.stakeholders.totalBeneficiaries,
          openResources: true,
          overall: assessment.dimensions.education,
        },
        innovation: {
          technologicalBreakthrough: 0.85,
          scientificContribution: 0.8,
          creativeInnovation: 0.75,
          problemSolving: 0.9,
          futurePotential: 0.88,
          overall: assessment.dimensions.innovation,
        },
      },
      recommendations,
      certificationStatus: {
        certified: assessment.hongikScore >= 400,
        level: assessment.certificationLevel,
        certifiedAt: assessment.hongikScore >= 400 ? new Date() : undefined,
        expiresAt:
          assessment.hongikScore >= 400
            ? new Date(Date.now() + 2 * 365 * 24 * 60 * 60 * 1000)
            : undefined,
        nextLevelRequirements: this.getNextLevelRequirements(
          assessment.certificationLevel
        ),
      },
    };
  }

  /**
   * Generate recommendations for improvement
   */
  private generateRecommendations(assessment: ImpactAssessment): Recommendation[] {
    const recommendations: Recommendation[] = [];

    // Find weakest dimensions
    const dimensionScores = [
      { dim: ImpactDimension.SOCIAL_GOOD, score: assessment.dimensions.socialGood },
      {
        dim: ImpactDimension.ACCESSIBILITY,
        score: assessment.dimensions.accessibility,
      },
      {
        dim: ImpactDimension.SUSTAINABILITY,
        score: assessment.dimensions.sustainability,
      },
      {
        dim: ImpactDimension.HEALTH_WELLBEING,
        score: assessment.dimensions.healthWellbeing,
      },
      {
        dim: ImpactDimension.ECONOMIC_EQUITY,
        score: assessment.dimensions.economicEquity,
      },
      { dim: ImpactDimension.EDUCATION, score: assessment.dimensions.education },
      { dim: ImpactDimension.INNOVATION, score: assessment.dimensions.innovation },
    ].sort((a, b) => a.score - b.score);

    // Generate recommendations for bottom 3
    dimensionScores.slice(0, 3).forEach((item, index) => {
      const actions = this.getImprovementActions(item.dim);
      const potentialScore = Math.min(item.score + 0.15, 1.0);
      const impactIncrease =
        (potentialScore - item.score) * DIMENSION_WEIGHTS[item.dim] * 100;

      recommendations.push({
        id: `REC-${index + 1}`,
        dimension: item.dim,
        currentScore: item.score,
        potentialScore,
        actions,
        effort: item.score < 0.5 ? 'high' : 'medium',
        impactIncrease,
        priority: index === 0 ? 'high' : index === 1 ? 'medium' : 'low',
      });
    });

    return recommendations;
  }

  /**
   * Get improvement actions for a dimension
   */
  private getImprovementActions(dimension: ImpactDimension): string[] {
    const actions: Record<ImpactDimension, string[]> = {
      [ImpactDimension.SOCIAL_GOOD]: [
        'Increase community engagement through participatory design',
        'Strengthen partnerships with local organizations',
        'Implement feedback mechanisms for beneficiaries',
      ],
      [ImpactDimension.ACCESSIBILITY]: [
        'Achieve WCAG 2.1 AAA compliance',
        'Add support for 10 additional languages',
        'Implement offline-first architecture',
        'Offer sliding scale pricing or free tier',
      ],
      [ImpactDimension.SUSTAINABILITY]: [
        'Transition to 100% renewable energy',
        'Implement circular economy principles',
        'Achieve carbon neutrality or negativity',
        'Conduct lifecycle assessment',
      ],
      [ImpactDimension.HEALTH_WELLBEING]: [
        'Expand preventive care programs',
        'Add mental health support services',
        'Improve safety protocols',
        'Track and improve QALY outcomes',
      ],
      [ImpactDimension.ECONOMIC_EQUITY]: [
        'Implement living wage policies',
        'Create job opportunities for marginalized groups',
        'Offer equity sharing or cooperative ownership',
        'Measure and reduce Gini coefficient',
      ],
      [ImpactDimension.EDUCATION]: [
        'Develop open educational resources',
        'Create peer learning communities',
        'Improve learning outcomes measurement',
        'Expand access to underserved populations',
      ],
      [ImpactDimension.INNOVATION]: [
        'Invest in R&D and breakthrough technologies',
        'Publish research findings openly',
        'File patents and share knowledge',
        'Build scalable solutions',
      ],
    };

    return actions[dimension] || [];
  }

  /**
   * Get next level certification requirements
   */
  private getNextLevelRequirements(currentLevel: CertificationLevel): string[] {
    const requirements: Record<CertificationLevel, string[]> = {
      [CertificationLevel.NONE]: [
        'Achieve HIS of 400+ for Bronze certification',
        'Complete self-assessment',
        'Submit for peer review',
      ],
      [CertificationLevel.BRONZE]: [
        'Achieve HIS of 600+ for Silver certification',
        'Obtain stakeholder validation',
        'Provide impact evidence',
      ],
      [CertificationLevel.SILVER]: [
        'Achieve HIS of 700+ for Gold certification',
        'Complete independent audit',
        'Demonstrate continuous improvement',
      ],
      [CertificationLevel.GOLD]: [
        'Achieve HIS of 800+ for Platinum certification',
        'Provide RCT or quasi-experimental evidence',
        'Show multi-year impact sustainability',
      ],
      [CertificationLevel.PLATINUM]: [
        'Achieve HIS of 900+ for Diamond certification',
        'Provide longitudinal impact data (3+ years)',
        'Demonstrate exceptional humanitarian benefit',
      ],
      [CertificationLevel.DIAMOND]: ['Maintain exceptional standards', 'Lead by example'],
    };

    return requirements[currentLevel] || [];
  }

  // ==========================================================================
  // Validation Methods
  // ==========================================================================

  /**
   * Validate dimension scores (must be 0-1)
   */
  private validateDimensionScores(scores: DimensionScores): void {
    const dimensions = Object.entries(scores);
    for (const [key, value] of dimensions) {
      if (typeof value !== 'number' || value < 0 || value > 1) {
        throw new HongikError(
          HongikErrorCode.INVALID_SCORE,
          `Invalid score for ${key}: ${value}. Must be between 0 and 1.`
        );
      }
    }
  }

  // ==========================================================================
  // Helper Methods
  // ==========================================================================

  /**
   * Calculate Hongik Impact Score from individual components
   */
  calculateHongikScore(params: {
    baseScore: number;
    reachFactor: number;
    temporalFactor: number;
  }): number {
    return params.baseScore * params.reachFactor * params.temporalFactor * 1000;
  }

  /**
   * Get emoji badge for classification
   */
  getClassificationBadge(classification: ImpactClassification): string {
    const badges: Record<ImpactClassification, string> = {
      [ImpactClassification.EXCEPTIONAL]: '🌟 Exceptional',
      [ImpactClassification.ELITE]: '⭐⭐⭐ Elite',
      [ImpactClassification.HIGH]: '⭐⭐ High',
      [ImpactClassification.GOOD]: '⭐ Good',
      [ImpactClassification.MODERATE]: '✓✓ Moderate',
      [ImpactClassification.FAIR]: '✓ Fair',
      [ImpactClassification.LIMITED]: '~ Limited',
      [ImpactClassification.MINIMAL]: '⚠ Minimal',
      [ImpactClassification.POOR]: '⚠⚠ Poor',
      [ImpactClassification.HARMFUL]: '❌ Harmful',
    };

    return badges[classification];
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate Hongik Impact Score
 */
export function calculateHongikScore(
  dimensions: DimensionScores,
  beneficiaries: number,
  duration: ProjectDuration = ProjectDuration.MEDIUM_TERM
): number {
  const sdk = new HongikImpactSDK();
  return sdk.calculateQuickScore({
    ...dimensions,
    beneficiaries,
    duration,
  }).total;
}

/**
 * Calculate accessibility score
 */
export function assessAccessibility(params: {
  wcagLevel: WCAGLevel;
  languagesSupported: number;
  offlineSupport: boolean;
  lowBandwidth: boolean;
  economicAccessibility: number;
}): number {
  const sdk = new HongikImpactSDK();
  return sdk.calculateAccessibilityScore(params);
}

/**
 * Calculate stakeholder reach factor
 */
export function calculateReachFactor(beneficiaries: number): number {
  const sdk = new HongikImpactSDK();
  return sdk.calculateReachFactor(beneficiaries);
}

/**
 * Classify impact score
 */
export function classifyImpact(score: number): ImpactClassification {
  const sdk = new HongikImpactSDK();
  return sdk.classifyScore(score);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';
export { HongikImpactSDK as default };
