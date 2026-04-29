/**
 * WIA-TIME-029: Time Adaptation SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Time Adaptation Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  AdaptationProgram,
  TravelerProfile,
  TimePeriod,
  CultureShockAssessment,
  CultureShockRisk,
  AdaptationMetrics,
  ProgressReport,
  LanguageAdaptationPlan,
  HistoricalContext,
  ReadinessAssessment,
  TrainingModule,
  AdaptationCategory,
  AdaptationPhase,
  TrendAnalysis,
  ADAPTATION_THRESHOLDS,
  AdaptationErrorCode,
  TimeAdaptationError,
  Result,
  AsyncResult,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

export class TimeAdaptationSDK {
  private version = '1.0.0';

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Create a comprehensive adaptation program
   */
  async createAdaptationProgram(params: {
    targetEra: string | Date;
    originEra?: string | Date;
    duration: number;
    travelerProfile: TravelerProfile;
    location?: string;
  }): AsyncResult<AdaptationProgram> {
    try {
      const originEra: TimePeriod = {
        date: params.originEra || new Date(),
        eraName: 'Origin Timeline',
      };

      const targetEra: TimePeriod = {
        date: params.targetEra,
        eraName: this.getEraName(params.targetEra),
      };

      // Calculate temporal displacement
      const displacement = this.calculateDisplacement(originEra.date, targetEra.date);

      // Assess culture shock risk
      const cultureShockRisk = this.assessCultureShockRisk({
        temporalDisplacement: displacement,
        culturalDistance: this.estimateCulturalDistance(displacement),
        technologicalGap: this.estimateTechGap(displacement),
        travelerAdaptability: params.travelerProfile.psychologicalProfile?.adaptability || 50,
      });

      // Generate training modules
      const modules = this.generateTrainingModules(
        originEra,
        targetEra,
        params.travelerProfile,
        cultureShockRisk
      );

      const program: AdaptationProgram = {
        id: this.generateId('PROG'),
        traveler: params.travelerProfile,
        originEra,
        targetEra,
        plannedDuration: params.duration,
        trainingModules: modules,
        preDeparture: this.createPreDeparturePhase(modules, cultureShockRisk),
        inEraSupport: this.createInEraSupport(targetEra),
        reAdaptation: this.createReAdaptationPlan(params.duration),
        status: 'planning',
        created: new Date(),
        updated: new Date(),
      };

      return { success: true, data: program };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  /**
   * Assess culture shock risk
   */
  assessCultureShockRisk(params: CultureShockAssessment): CultureShockRisk {
    const {
      temporalDisplacement,
      culturalDistance,
      technologicalGap,
      languageBarrier = 0.5,
      socialDifference = 0.5,
      travelerAdaptability = 50,
    } = params;

    // Calculate base risk score
    const timeYears = Math.abs(temporalDisplacement / (365.25 * 24 * 3600));
    const timeFactor = Math.min(timeYears / 100, 1); // Normalize to 0-1

    const baseRisk =
      timeFactor * 0.25 +
      culturalDistance * 0.25 +
      technologicalGap * 0.2 +
      languageBarrier * 0.15 +
      socialDifference * 0.15;

    // Adjust for traveler adaptability
    const adaptabilityFactor = (100 - travelerAdaptability) / 100;
    const riskScore = Math.min(baseRisk * (1 + adaptabilityFactor), 1) * 100;

    // Determine risk level
    let riskLevel: CultureShockRisk['riskLevel'];
    if (riskScore < 20) riskLevel = 'minimal';
    else if (riskScore < 40) riskLevel = 'low';
    else if (riskScore < 60) riskLevel = 'moderate';
    else if (riskScore < 75) riskLevel = 'high';
    else if (riskScore < 90) riskLevel = 'severe';
    else riskLevel = 'extreme';

    // Calculate adaptation difficulty (1-10 scale)
    const adaptationDifficulty = Math.ceil((riskScore / 100) * 10);

    // Calculate training duration
    const baseTraining = 4; // weeks
    const trainingDuration = Math.ceil(baseTraining * (1 + riskScore / 50));

    // Identify critical challenges
    const challenges: string[] = [];
    if (technologicalGap > 0.7) challenges.push('Significant technology gap');
    if (culturalDistance > 0.7) challenges.push('Major cultural differences');
    if (languageBarrier > 0.7) challenges.push('Substantial language barrier');
    if (socialDifference > 0.7) challenges.push('Vastly different social structures');
    if (timeYears > 100) challenges.push('Extreme temporal displacement');

    // Generate mitigation strategies
    const strategies = this.generateMitigationStrategies(riskLevel, challenges);

    // Calculate success probability
    const successProbability = Math.max(0.2, 1 - riskScore / 150);

    return {
      riskLevel,
      riskScore,
      adaptationDifficulty,
      trainingDuration,
      criticalChallenges: challenges,
      mitigationStrategies: strategies,
      successProbability,
    };
  }

  /**
   * Generate language adaptation plan
   */
  generateLanguagePlan(params: {
    targetEra: string | Date;
    nativeLanguage: string;
    currentProficiency?: number;
  }): LanguageAdaptationPlan {
    const targetYear = this.extractYear(params.targetEra);
    const targetLanguage = 'English'; // Simplified for example
    const historicalVariant = this.getHistoricalLanguageVariant(targetYear);

    const modules = this.generateLanguageModules(
      params.nativeLanguage,
      targetLanguage,
      historicalVariant,
      params.currentProficiency || 0
    );

    const estimatedHours = modules.reduce((sum, m) => sum + m.duration, 0);

    return {
      targetLanguage,
      historicalVariant,
      nativeLanguage: params.nativeLanguage,
      learningPath: modules,
      dialectFocus: [historicalVariant],
      slangTraining: true,
      writtenFormTraining: true,
      pronunciationCoaching: true,
      estimatedHours,
    };
  }

  /**
   * Get historical context for an era
   */
  async getHistoricalContext(params: {
    era: string | Date;
    location?: string;
  }): AsyncResult<HistoricalContext> {
    try {
      const year = this.extractYear(params.era);
      const context = this.generateHistoricalContext(year, params.location);
      return { success: true, data: context };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error : new Error('Unknown error'),
      };
    }
  }

  /**
   * Track adaptation progress
   */
  trackProgress(params: {
    travelerId: string;
    daysInEra: number;
    metrics: Partial<AdaptationMetrics>;
  }): ProgressReport {
    // Calculate overall score
    const metrics: AdaptationMetrics = {
      date: new Date(),
      daysInEra: params.daysInEra,
      culturalCompetency: params.metrics.culturalCompetency || 0,
      languageProficiency: params.metrics.languageProficiency || 0,
      socialIntegration: params.metrics.socialIntegration || 0,
      psychologicalWellbeing: params.metrics.psychologicalWellbeing || 0,
      dailyFunctioning: params.metrics.dailyFunctioning || 0,
      technologyAdaptation: params.metrics.technologyAdaptation || 0,
      overallScore: 0,
      currentPhase: this.determineAdaptationPhase(params.daysInEra),
      incidents: params.metrics.incidents || [],
      achievements: params.metrics.achievements || [],
    };

    // Calculate weighted overall score
    metrics.overallScore =
      metrics.culturalCompetency * 0.3 +
      metrics.languageProficiency * 0.25 +
      metrics.socialIntegration * 0.2 +
      metrics.psychologicalWellbeing * 0.15 +
      metrics.dailyFunctioning * 0.1;

    // Analyze trends
    const trends = this.analyzeTrends(metrics, params.daysInEra);

    // Generate recommendations
    const recommendations = this.generateRecommendations(metrics, trends);

    // Identify concerns
    const concerns = this.identifyConcerns(metrics);

    const report: ProgressReport = {
      id: this.generateId('RPT'),
      travelerId: params.travelerId,
      date: new Date(),
      period: {
        start: new Date(Date.now() - params.daysInEra * 24 * 3600 * 1000),
        end: new Date(),
      },
      metrics,
      trends,
      recommendations,
      concerns,
      nextSteps: this.generateNextSteps(metrics, trends),
    };

    return report;
  }

  /**
   * Quick assessment for travel feasibility
   */
  quickAssessment(params: {
    targetYear: number;
    currentYear: number;
    travelDuration: number;
  }): {
    difficulty: number;
    trainingWeeks: number;
    languageFocus: string[];
    culturalPriorities: string[];
    keyRisks: string[];
  } {
    const displacement = (params.targetYear - params.currentYear) * 365.25 * 24 * 3600;
    const culturalDistance = this.estimateCulturalDistance(displacement);
    const techGap = this.estimateTechGap(displacement);

    const risk = this.assessCultureShockRisk({
      temporalDisplacement: displacement,
      culturalDistance,
      technologicalGap: techGap,
    });

    const languageFocus = this.getLanguageFocus(params.targetYear);
    const culturalPriorities = this.getCulturalPriorities(params.targetYear);
    const keyRisks = risk.criticalChallenges;

    return {
      difficulty: risk.adaptationDifficulty,
      trainingWeeks: risk.trainingDuration,
      languageFocus,
      culturalPriorities,
      keyRisks,
    };
  }

  /**
   * Conduct readiness assessment
   */
  conductReadinessAssessment(
    program: AdaptationProgram
  ): ReadinessAssessment {
    const categoryScores: Record<AdaptationCategory, number> = {
      language: 0,
      technology: 0,
      'social-norms': 0,
      'daily-living': 0,
      'legal-political': 0,
      'health-medical': 0,
      economics: 0,
      'religion-philosophy': 0,
      history: 0,
      geography: 0,
      'arts-culture': 0,
      science: 0,
    };

    // Calculate scores from completed modules
    program.trainingModules.forEach((module) => {
      if (categoryScores[module.category] !== undefined) {
        categoryScores[module.category] = Math.max(
          categoryScores[module.category],
          module.progress || 0
        );
      }
    });

    // Calculate overall score
    const scores = Object.values(categoryScores);
    const overallScore = scores.reduce((sum, score) => sum + score, 0) / scores.length;

    // Determine recommendation
    let recommendation: ReadinessAssessment['recommendation'];
    const gaps: string[] = [];
    const strengths: string[] = [];

    Object.entries(categoryScores).forEach(([category, score]) => {
      if (score < 60) gaps.push(category);
      if (score >= 80) strengths.push(category);
    });

    if (overallScore >= 75 && gaps.length === 0) {
      recommendation = 'ready';
    } else if (overallScore >= 60 && gaps.length <= 2) {
      recommendation = 'conditional';
    } else if (overallScore >= 40) {
      recommendation = 'extended-training';
    } else {
      recommendation = 'not-ready';
    }

    const additionalTraining = gaps.length * 10; // hours per gap

    return {
      date: new Date(),
      overallScore,
      categoryScores,
      recommendation,
      gaps,
      strengths,
      additionalTraining: additionalTraining > 0 ? additionalTraining : undefined,
    };
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private generateId(prefix: string): string {
    return `${prefix}-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private getEraName(date: string | Date): string {
    const year = this.extractYear(date);

    if (year < 500) return 'Ancient Era';
    if (year < 1500) return 'Medieval Era';
    if (year < 1800) return 'Early Modern Era';
    if (year < 1900) return 'Industrial Age';
    if (year < 1950) return 'Early 20th Century';
    if (year < 2000) return 'Late 20th Century';
    if (year < 2050) return 'Early 21st Century';
    if (year < 2100) return 'Late 21st Century';
    return 'Far Future';
  }

  private extractYear(date: string | Date): number {
    if (date instanceof Date) return date.getFullYear();
    const parsed = new Date(date);
    if (!isNaN(parsed.getTime())) return parsed.getFullYear();
    // Try to extract year number from string
    const match = date.toString().match(/\d{4}/);
    return match ? parseInt(match[0]) : new Date().getFullYear();
  }

  private calculateDisplacement(origin: string | Date, target: string | Date): number {
    const originTime = origin instanceof Date ? origin.getTime() : new Date(origin).getTime();
    const targetTime = target instanceof Date ? target.getTime() : new Date(target).getTime();
    return (targetTime - originTime) / 1000; // seconds
  }

  private estimateCulturalDistance(displacement: number): number {
    const years = Math.abs(displacement / (365.25 * 24 * 3600));
    // Cultural distance increases with time, but not linearly
    return Math.min(Math.pow(years / 100, 0.7), 1);
  }

  private estimateTechGap(displacement: number): number {
    const years = Math.abs(displacement / (365.25 * 24 * 3600));
    // Technology gap grows faster than cultural distance
    return Math.min(Math.pow(years / 50, 0.8), 1);
  }

  private generateMitigationStrategies(
    riskLevel: CultureShockRisk['riskLevel'],
    challenges: string[]
  ): string[] {
    const strategies: string[] = [
      'Complete comprehensive pre-departure training',
      'Establish regular check-ins with support coordinator',
      'Utilize real-time translation and cultural advisory services',
    ];

    if (riskLevel === 'high' || riskLevel === 'severe' || riskLevel === 'extreme') {
      strategies.push('Assign dedicated in-era mentor');
      strategies.push('Implement daily psychological monitoring');
      strategies.push('Establish emergency extraction protocols');
    }

    if (challenges.includes('Significant technology gap')) {
      strategies.push('Extended technology familiarization training');
      strategies.push('Provide era-appropriate device operation guides');
    }

    if (challenges.includes('Substantial language barrier')) {
      strategies.push('Intensive language immersion program');
      strategies.push('Deploy neural language translation implant');
    }

    return strategies;
  }

  private generateTrainingModules(
    origin: TimePeriod,
    target: TimePeriod,
    profile: TravelerProfile,
    risk: CultureShockRisk
  ): TrainingModule[] {
    const modules: TrainingModule[] = [];
    const categories: AdaptationCategory[] = [
      'language',
      'technology',
      'social-norms',
      'daily-living',
      'history',
    ];

    categories.forEach((category, index) => {
      const duration = this.calculateModuleDuration(category, risk);
      modules.push({
        id: `MOD-${category.toUpperCase()}-${index}`,
        name: `${category.charAt(0).toUpperCase() + category.slice(1)} Adaptation`,
        category,
        description: `Comprehensive training for ${category} in target era`,
        duration,
        requiredCompletion: 80,
        progress: 0,
        materials: [],
        status: 'not-started',
      });
    });

    return modules;
  }

  private calculateModuleDuration(
    category: AdaptationCategory,
    risk: CultureShockRisk
  ): number {
    const baseDurations: Record<AdaptationCategory, number> = {
      language: 40,
      technology: 20,
      'social-norms': 30,
      'daily-living': 15,
      'legal-political': 10,
      'health-medical': 15,
      economics: 10,
      'religion-philosophy': 20,
      history: 25,
      geography: 10,
      'arts-culture': 15,
      science: 20,
    };

    const base = baseDurations[category] || 15;
    const multiplier = 1 + risk.riskScore / 200;
    return Math.ceil(base * multiplier);
  }

  private createPreDeparturePhase(
    modules: TrainingModule[],
    risk: CultureShockRisk
  ): AdaptationProgram['preDeparture'] {
    const totalHours = modules.reduce((sum, m) => sum + m.duration, 0);
    const startDate = new Date();
    const endDate = new Date(startDate.getTime() + risk.trainingDuration * 7 * 24 * 3600 * 1000);

    return {
      startDate,
      endDate,
      totalHours,
      completedHours: 0,
      requiredModules: modules.map((m) => m.id),
      status: 'scheduled',
    };
  }

  private createInEraSupport(target: TimePeriod): AdaptationProgram['inEraSupport'] {
    return {
      emergencyContacts: [
        {
          name: 'Temporal Support Center',
          role: 'Emergency Coordinator',
          method: 'Temporal Communication',
          details: 'TSC-EMERGENCY-001',
        },
      ],
      checkInSchedule: {
        frequency: 'weekly',
        method: 'temporal-comm',
      },
      translationSupport: true,
      culturalAdvisor: true,
      medicalSupport: {
        facilities: [],
        emergencyProtocol: 'Contact temporal support immediately',
      },
    };
  }

  private createReAdaptationPlan(duration: number): AdaptationProgram['reAdaptation'] {
    const decompressionDays = Math.min(Math.ceil(duration / 30), 30);
    const debriefingSessions = Math.min(Math.ceil(duration / 60), 10);

    return {
      decompressionPeriod: decompressionDays,
      debriefingSessions,
      counselingAvailable: true,
      reverseCultureShockSupport: true,
    };
  }

  private determineAdaptationPhase(daysInEra: number): AdaptationPhase {
    if (daysInEra <= 7) return 'honeymoon';
    if (daysInEra <= 30) return 'culture-shock';
    if (daysInEra <= 90) return 'adjustment';
    if (daysInEra <= 180) return 'adaptation';
    return 'mastery';
  }

  private analyzeTrends(
    metrics: AdaptationMetrics,
    daysInEra: number
  ): TrendAnalysis {
    // Simplified trend analysis
    const expectedScore = this.calculateExpectedScore(daysInEra);
    const delta = metrics.overallScore - expectedScore;

    let vsExpected: TrendAnalysis['vsExpected'];
    if (delta > 10) vsExpected = 'ahead';
    else if (delta < -20) vsExpected = 'significantly-behind';
    else if (delta < -10) vsExpected = 'behind';
    else vsExpected = 'on-track';

    const trajectory: TrendAnalysis['trajectory'] =
      metrics.overallScore > 70 ? 'improving' : 'stable';

    const improvementRate = metrics.overallScore / Math.max(daysInEra / 7, 1);

    const strengths: string[] = [];
    const weaknesses: string[] = [];

    if (metrics.languageProficiency > 75) strengths.push('Language');
    if (metrics.culturalCompetency > 75) strengths.push('Cultural Understanding');
    if (metrics.socialIntegration > 75) strengths.push('Social Integration');

    if (metrics.languageProficiency < 50) weaknesses.push('Language');
    if (metrics.culturalCompetency < 50) weaknesses.push('Cultural Understanding');
    if (metrics.technologyAdaptation < 50) weaknesses.push('Technology');

    const projectedMastery =
      improvementRate > 0
        ? new Date(Date.now() + ((90 - metrics.overallScore) / improvementRate) * 7 * 24 * 3600 * 1000)
        : undefined;

    return {
      trajectory,
      improvementRate,
      projectedMastery,
      strengths,
      weaknesses,
      vsExpected,
    };
  }

  private calculateExpectedScore(daysInEra: number): number {
    // Expected progression curve
    if (daysInEra <= 7) return 30;
    if (daysInEra <= 30) return 40;
    if (daysInEra <= 90) return 60;
    if (daysInEra <= 180) return 75;
    return 85;
  }

  private generateRecommendations(
    metrics: AdaptationMetrics,
    trends: TrendAnalysis
  ): string[] {
    const recommendations: string[] = [];

    if (metrics.languageProficiency < 60) {
      recommendations.push('Increase language practice - aim for daily conversations');
    }

    if (metrics.socialIntegration < 60) {
      recommendations.push('Participate in more social activities and community events');
    }

    if (metrics.psychologicalWellbeing < 70) {
      recommendations.push('Schedule additional counseling sessions');
    }

    if (trends.vsExpected === 'behind' || trends.vsExpected === 'significantly-behind') {
      recommendations.push('Consider additional training modules');
      recommendations.push('Increase check-in frequency with mentor');
    }

    return recommendations;
  }

  private identifyConcerns(metrics: AdaptationMetrics): string[] {
    const concerns: string[] = [];

    if (metrics.psychologicalWellbeing < 50) {
      concerns.push('CRITICAL: Low psychological wellbeing requires immediate attention');
    }

    if (metrics.dailyFunctioning < 40) {
      concerns.push('WARNING: Difficulty with daily functioning');
    }

    if (metrics.overallScore < ADAPTATION_THRESHOLDS.minimal) {
      concerns.push('ALERT: Overall adaptation below minimal threshold');
    }

    return concerns;
  }

  private generateNextSteps(
    metrics: AdaptationMetrics,
    trends: TrendAnalysis
  ): string[] {
    const steps: string[] = ['Continue regular check-ins'];

    if (trends.weaknesses.length > 0) {
      steps.push(`Focus on improving: ${trends.weaknesses.join(', ')}`);
    }

    if (metrics.overallScore >= ADAPTATION_THRESHOLDS.proficient) {
      steps.push('Begin advanced integration activities');
    }

    return steps;
  }

  private generateLanguageModules(
    native: string,
    target: string,
    variant: string,
    proficiency: number
  ): LanguageAdaptationPlan['learningPath'] {
    const modules: LanguageAdaptationPlan['learningPath'] = [];
    const focuses: Array<'vocabulary' | 'grammar' | 'pronunciation' | 'conversation'> = [
      'vocabulary',
      'grammar',
      'pronunciation',
      'conversation',
    ];

    focuses.forEach((focus, i) => {
      modules.push({
        id: `LANG-${focus.toUpperCase()}`,
        name: `${focus.charAt(0).toUpperCase() + focus.slice(1)} Training`,
        focus,
        level: proficiency < 30 ? 'beginner' : proficiency < 70 ? 'intermediate' : 'advanced',
        duration: 20 + i * 5,
        exercises: 50,
        progress: 0,
      });
    });

    return modules;
  }

  private getHistoricalLanguageVariant(year: number): string {
    if (year < 1500) return 'Middle English';
    if (year < 1800) return 'Early Modern English';
    if (year < 1950) return 'Modern English';
    if (year < 2000) return 'Late 20th Century English';
    if (year < 2100) return 'Contemporary English';
    return 'Future English Variant';
  }

  private getLanguageFocus(year: number): string[] {
    if (year < 1800) return ['Formal grammar', 'Archaic vocabulary', 'Historical pronunciation'];
    if (year < 1950) return ['Period slang', 'Formal etiquette', 'Written correspondence'];
    if (year < 2000) return ['Modern idioms', 'Cultural references', 'Media literacy'];
    return ['Digital communication', 'Contemporary slang', 'Global variants'];
  }

  private getCulturalPriorities(year: number): string[] {
    if (year < 1800) {
      return ['Social hierarchy', 'Religious customs', 'Court etiquette', 'Guild systems'];
    }
    if (year < 1950) {
      return ['Industrial workplace', 'Urbanization', 'Class distinctions', 'Gender roles'];
    }
    if (year < 2000) {
      return ['Mass media', 'Consumer culture', 'Civil rights', 'Technology adoption'];
    }
    return ['Digital society', 'Globalization', 'Environmental awareness', 'Diversity'];
  }

  private generateHistoricalContext(year: number, location?: string): HistoricalContext {
    // Simplified historical context generation
    return {
      period: {
        date: new Date(year, 0, 1),
        eraName: this.getEraName(new Date(year, 0, 1)),
      },
      majorEvents: [],
      socialStructure: {
        classSystem: 'Varies by era',
        mobility: 'moderate',
        genderRoles: 'Period-specific',
        familyStructure: 'Traditional for era',
        etiquette: ['Formal greetings', 'Respectful address'],
        taboos: ['Era-specific'],
      },
      technologyLevel: {
        era: this.getEraName(new Date(year, 0, 1)),
        communication: this.getTechForEra(year, 'communication'),
        transportation: this.getTechForEra(year, 'transportation'),
        energy: this.getTechForEra(year, 'energy'),
        medical: this.getTechForEra(year, 'medical'),
        devices: this.getTechForEra(year, 'devices'),
        gapFromOrigin: this.estimateTechGap((year - new Date().getFullYear()) * 365.25 * 24 * 3600),
      },
      culturalNorms: [],
      politicalSituation: {
        governmentType: 'Period-specific',
        keyPolicies: [],
        citizenRights: [],
        restrictions: [],
        stability: 'moderate',
      },
      economicConditions: {
        system: 'Period-specific',
        currency: 'Local currency',
        occupations: [],
        tradePractices: [],
      },
      dailyLife: {
        schedule: 'Typical for era',
        foodCulture: 'Regional cuisine',
        clothingNorms: 'Period-appropriate',
        housing: 'Era-typical',
        hygiene: 'Period standards',
        entertainment: ['Era-appropriate activities'],
        challenges: ['Adaptation to period norms'],
      },
    };
  }

  private getTechForEra(year: number, category: string): string[] {
    const tech: Record<string, Record<number, string[]>> = {
      communication: {
        1800: ['Letters', 'Messengers'],
        1900: ['Telegraph', 'Telephone', 'Newspapers'],
        1950: ['Radio', 'Television', 'Postal service'],
        2000: ['Internet', 'Mobile phones', 'Email'],
        2025: ['Smartphones', 'Social media', 'Video calls', 'Instant messaging'],
      },
      transportation: {
        1800: ['Horse', 'Carriage', 'Ship'],
        1900: ['Train', 'Automobile', 'Steamship'],
        1950: ['Car', 'Airplane', 'Bus'],
        2000: ['Car', 'Airplane', 'Train', 'Subway'],
        2025: ['Electric vehicles', 'High-speed rail', 'Aircraft', 'Ride-sharing'],
      },
    };

    const ranges = Object.keys(tech[category] || {})
      .map(Number)
      .sort((a, b) => a - b);
    const closestYear = ranges.reduce((prev, curr) =>
      Math.abs(curr - year) < Math.abs(prev - year) ? curr : prev
    );

    return tech[category]?.[closestYear] || ['Unknown'];
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate culture shock risk quickly
 */
export function calculateCultureShockRisk(
  yearsDisplacement: number,
  travelerAdaptability: number = 50
): CultureShockRisk {
  const sdk = new TimeAdaptationSDK();
  const displacement = yearsDisplacement * 365.25 * 24 * 3600;

  return sdk.assessCultureShockRisk({
    temporalDisplacement: displacement,
    culturalDistance: Math.min(Math.abs(yearsDisplacement) / 100, 1),
    technologicalGap: Math.min(Math.abs(yearsDisplacement) / 50, 1),
    travelerAdaptability,
  });
}

/**
 * Estimate training duration needed
 */
export function estimateTrainingDuration(
  originYear: number,
  targetYear: number
): number {
  const displacement = Math.abs(targetYear - originYear);
  const baseWeeks = 4;
  return Math.ceil(baseWeeks * (1 + displacement / 100));
}

/**
 * Check if traveler is ready
 */
export function checkReadiness(
  completionScores: Record<string, number>
): 'ready' | 'conditional' | 'not-ready' {
  const scores = Object.values(completionScores);
  const average = scores.reduce((a, b) => a + b, 0) / scores.length;
  const lowScores = scores.filter(s => s < 60).length;

  if (average >= 75 && lowScores === 0) return 'ready';
  if (average >= 60 && lowScores <= 2) return 'conditional';
  return 'not-ready';
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export {
  TimeAdaptationSDK,
  calculateCultureShockRisk,
  estimateTrainingDuration,
  checkReadiness,
};

export default TimeAdaptationSDK;
