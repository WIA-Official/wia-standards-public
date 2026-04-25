/**
 * WIA-AUG-005: Cognitive Enhancement SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Cognitive Enhancement Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for cognitive enhancement including:
 * - Baseline cognitive assessment
 * - Domain-specific enhancement
 * - Performance measurement and tracking
 * - Cognitive load and fatigue management
 * - Decision support integration
 */

import {
  CognitiveDomain,
  EnhancementMethod,
  CognitiveMetrics,
  DomainScore,
  CognitiveAssessment,
  CognitiveLoadStatus,
  FatigueAssessment,
  FatigueIndicators,
  FatigueLevel,
  FatigueAction,
  EnhancementRequest,
  EnhancementSession,
  EnhancementResult,
  PerformanceIndicators,
  PerformanceMeasurement,
  BaselineRequest,
  BaselineAssessment,
  DecisionSupportRequest,
  DecisionSupport,
  Recommendation,
  SafetyThresholds,
  EnhancementProtocol,
  COGNITIVE_CONSTANTS,
  CognitiveErrorCode,
  CognitiveError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-005 Cognitive Enhancement SDK
 */
export class CognitiveEnhancementSDK {
  private version = '1.0.0';
  private activeSessions: Map<string, EnhancementSession> = new Map();
  private baselineCache: Map<string, BaselineAssessment> = new Map();

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ============================================================================
  // Baseline Assessment
  // ============================================================================

  /**
   * Perform baseline cognitive assessment
   */
  async assessBaseline(request: BaselineRequest): Promise<BaselineAssessment> {
    const { userId, domains = this.getAllDomains(), type = 'comprehensive' } = request;

    // Simulate assessment (in production, this would run actual cognitive tests)
    const domainScores: Record<CognitiveDomain, DomainScore> = {} as any;

    for (const domain of domains) {
      domainScores[domain] = await this.assessDomain(userId, domain);
    }

    const cognitiveIndex = this.calculateCognitiveIndex(domainScores);

    const assessment: BaselineAssessment = {
      assessmentId: `BA-${Date.now()}`,
      userId,
      date: new Date(),
      demographics: request.includeDemographics ? {
        age: 0, // Would be provided by user
        education: 0,
      } : undefined,
      cognitive: {
        assessmentId: `CA-${Date.now()}`,
        userId,
        date: new Date(),
        cognitiveIndex,
        domainScores,
        type: 'baseline',
        duration: type === 'comprehensive' ? 300 : 60,
      },
      enhancementPotential: this.calculateEnhancementPotential(domainScores),
      recommendedMethods: this.recommendMethods(domainScores),
      contraindications: [],
      medicalClearanceRequired: false,
    };

    // Cache baseline
    this.baselineCache.set(userId, assessment);

    return assessment;
  }

  /**
   * Assess a single cognitive domain
   */
  private async assessDomain(userId: string, domain: CognitiveDomain): Promise<DomainScore> {
    // Simulate domain assessment
    // In production, this would run actual cognitive tests

    const baselineScore = 85 + Math.random() * 30; // 85-115 range
    const rawScore = baselineScore;
    const normalizedScore = rawScore;

    const metrics: CognitiveMetrics = {
      baseline: baselineScore,
      current: baselineScore,
      enhancementRatio: 0.0,
      percentile: this.scoreToPercentile(baselineScore),
      standardScore: baselineScore,
      confidence95: [baselineScore - 5, baselineScore + 5],
    };

    return {
      domain,
      rawScore,
      normalizedScore,
      metrics,
      timestamp: new Date(),
    };
  }

  // ============================================================================
  // Enhancement Functions
  // ============================================================================

  /**
   * Initiate an enhancement session
   */
  async initiateEnhancement(request: EnhancementRequest): Promise<EnhancementSession> {
    const { userId, targetDomains, method, targetRatio, duration, intensity = 0.5 } = request;

    // Validate request
    this.validateEnhancementRequest(request);

    // Check for baseline
    const baseline = this.baselineCache.get(userId);
    if (!baseline) {
      throw new CognitiveError(
        CognitiveErrorCode.BASELINE_REQUIRED,
        'Baseline assessment required before enhancement'
      );
    }

    // Validate target ratio
    const primaryDomain = targetDomains[0];
    const maxRatio = COGNITIVE_CONSTANTS.MAX_ENHANCEMENT_RATIOS[primaryDomain];
    if (targetRatio > maxRatio) {
      throw new CognitiveError(
        CognitiveErrorCode.ENHANCEMENT_LIMIT_EXCEEDED,
        `Target ratio ${targetRatio} exceeds maximum ${maxRatio} for ${primaryDomain}`
      );
    }

    const session: EnhancementSession = {
      sessionId: `ES-${Date.now()}`,
      userId,
      startTime: new Date(),
      method,
      targetDomain: primaryDomain,
      targetRatio,
      currentRatio: 0.0,
      duration,
      intensity,
      status: 'active',
      performanceData: [],
      loadHistory: [],
      fatigueAssessments: [],
    };

    this.activeSessions.set(session.sessionId, session);

    return session;
  }

  /**
   * Enhance a specific cognitive domain
   */
  async enhanceDomain(
    sessionId: string,
    domain: CognitiveDomain,
    intensity: number
  ): Promise<EnhancementResult> {
    const session = this.activeSessions.get(sessionId);
    if (!session) {
      throw new CognitiveError(
        CognitiveErrorCode.SESSION_EXPIRED,
        'Session not found or expired'
      );
    }

    if (session.status !== 'active') {
      throw new CognitiveError(
        CognitiveErrorCode.INVALID_PARAMETERS,
        'Session is not active'
      );
    }

    // Simulate enhancement process
    const baseline = this.baselineCache.get(session.userId);
    if (!baseline) {
      throw new CognitiveError(
        CognitiveErrorCode.BASELINE_REQUIRED,
        'Baseline assessment required'
      );
    }

    const baselineScore = baseline.cognitive.domainScores[domain].metrics.baseline;

    // Calculate enhancement based on method and intensity
    const methodEffectiveness = this.getMethodEffectiveness(session.method, domain);
    const achievedRatio = Math.min(
      session.targetRatio,
      methodEffectiveness * intensity * 0.8 // 80% of theoretical max
    );

    // Update session
    session.currentRatio = achievedRatio;

    const improvements: Record<CognitiveDomain, number> = {} as any;
    improvements[domain] = achievedRatio;

    // Check for transfer effects
    const transferDomains = this.getTransferDomains(domain);
    for (const transferDomain of transferDomains) {
      improvements[transferDomain] = achievedRatio * 0.3; // 30% transfer
    }

    const result: EnhancementResult = {
      sessionId,
      success: true,
      achievedRatio,
      improvements,
      summary: {
        duration: session.duration,
        averageLoad: this.calculateAverageLoad(session),
        peakLoad: this.calculatePeakLoad(session),
        fatigueLevel: this.getCurrentFatigueLevel(session),
        adverseEvents: [],
      },
      recommendations: this.generateRecommendations(session, achievedRatio),
      timestamp: new Date(),
    };

    return result;
  }

  // ============================================================================
  // Performance Measurement
  // ============================================================================

  /**
   * Measure current cognitive performance
   */
  async measurePerformance(
    userId: string,
    domain: CognitiveDomain,
    sessionId?: string
  ): Promise<PerformanceMeasurement> {
    const baseline = this.baselineCache.get(userId);
    if (!baseline) {
      throw new CognitiveError(
        CognitiveErrorCode.BASELINE_REQUIRED,
        'Baseline assessment required'
      );
    }

    const baselineScore = baseline.cognitive.domainScores[domain].metrics.baseline;

    // Simulate performance measurement
    const session = sessionId ? this.activeSessions.get(sessionId) : null;
    const enhancementBonus = session ? session.currentRatio : 0;

    const currentScore = baselineScore * (1 + enhancementBonus);
    const improvement = currentScore - baselineScore;
    const enhancementRatio = improvement / baselineScore;

    const indicators: PerformanceIndicators = {
      timestamp: new Date(),
      accuracyRate: 0.85 + enhancementRatio * 0.15,
      responseTime: 1000 - enhancementRatio * 200,
      taskCompletionRate: 0.80 + enhancementRatio * 0.2,
      errorRate: 0.15 - enhancementRatio * 0.1,
      perseverationIndex: 0.05,
      noveltyScore: domain === 'CREATIVITY' ? 0.6 + enhancementRatio * 0.3 : 0.3,
      subjective: {
        perceivedDifficulty: 5 - enhancementRatio * 2,
        perceivedPerformance: 6 + enhancementRatio * 3,
        mentalEffort: 6 - enhancementRatio * 1.5,
      },
    };

    // Add to session if active
    if (session) {
      session.performanceData.push(indicators);
    }

    const measurement: PerformanceMeasurement = {
      measurementId: `PM-${Date.now()}`,
      userId,
      sessionId,
      domain,
      indicators,
      baselineComparison: {
        baseline: baselineScore,
        current: currentScore,
        improvement,
        enhancementRatio,
      },
      cognitiveLoad: session ? this.calculateCurrentLoad(session) : 0.3,
      timestamp: new Date(),
    };

    return measurement;
  }

  // ============================================================================
  // Cognitive Load Management
  // ============================================================================

  /**
   * Monitor cognitive load
   */
  async monitorCognitiveLoad(sessionId: string): Promise<CognitiveLoadStatus> {
    const session = this.activeSessions.get(sessionId);
    if (!session) {
      throw new CognitiveError(
        CognitiveErrorCode.SESSION_EXPIRED,
        'Session not found'
      );
    }

    const currentLoad = this.calculateCurrentLoad(session);

    let category: 'low' | 'moderate' | 'high' | 'critical';
    let recommendation: 'continue' | 'reduce_load' | 'take_break' | 'stop_session';

    if (currentLoad < 0.3) {
      category = 'low';
      recommendation = 'continue';
    } else if (currentLoad < 0.7) {
      category = 'moderate';
      recommendation = 'continue';
    } else if (currentLoad < 0.9) {
      category = 'high';
      recommendation = 'reduce_load';
    } else {
      category = 'critical';
      recommendation = 'stop_session';
    }

    const status: CognitiveLoadStatus = {
      currentLoad,
      category,
      components: {
        taskDemand: session.intensity * 0.8,
        attentionAllocation: 0.7,
        memoryLoad: 0.6,
        processingSpeed: 0.65,
      },
      availableResources: 1.0 - currentLoad,
      recommendation,
      timestamp: new Date(),
    };

    session.loadHistory.push(status);

    // Check for critical load
    if (currentLoad > COGNITIVE_CONSTANTS.LOAD_THRESHOLDS.CRITICAL) {
      throw new CognitiveError(
        CognitiveErrorCode.COGNITIVE_LOAD_CRITICAL,
        'Critical cognitive load detected - session must be paused',
        { currentLoad, status }
      );
    }

    return status;
  }

  /**
   * Calculate current cognitive load
   */
  private calculateCurrentLoad(session: EnhancementSession): number {
    const sessionDuration = Date.now() - session.startTime.getTime();
    const durationMinutes = sessionDuration / (1000 * 60);

    // Load increases with time and intensity
    const baseLoad = session.intensity * 0.5;
    const temporalLoad = Math.min(durationMinutes / 60, 0.5); // Max 0.5 from time
    const fatigue = this.estimateFatigue(session);

    return Math.min(baseLoad + temporalLoad + fatigue * 0.3, 1.0);
  }

  /**
   * Calculate average load
   */
  private calculateAverageLoad(session: EnhancementSession): number {
    if (session.loadHistory.length === 0) return 0.5;
    const sum = session.loadHistory.reduce((acc, status) => acc + status.currentLoad, 0);
    return sum / session.loadHistory.length;
  }

  /**
   * Calculate peak load
   */
  private calculatePeakLoad(session: EnhancementSession): number {
    if (session.loadHistory.length === 0) return 0.5;
    return Math.max(...session.loadHistory.map(status => status.currentLoad));
  }

  // ============================================================================
  // Fatigue Management
  // ============================================================================

  /**
   * Detect cognitive fatigue
   */
  async detectFatigue(sessionId: string): Promise<FatigueAssessment> {
    const session = this.activeSessions.get(sessionId);
    if (!session) {
      throw new CognitiveError(
        CognitiveErrorCode.SESSION_EXPIRED,
        'Session not found'
      );
    }

    const fatigueScore = this.estimateFatigue(session) * 100;

    let level: FatigueLevel;
    let action: FatigueAction;
    let recommendedBreak: number;

    if (fatigueScore < 30) {
      level = 'low';
      action = 'continue';
      recommendedBreak = 0;
    } else if (fatigueScore < 60) {
      level = 'moderate';
      action = 'suggest_break';
      recommendedBreak = 10;
    } else if (fatigueScore < 80) {
      level = 'high';
      action = 'mandatory_break';
      recommendedBreak = 30;
    } else {
      level = 'critical';
      action = 'end_session';
      recommendedBreak = 240; // 4 hours
    }

    const performanceDecline = this.calculatePerformanceDecline(session);

    const indicators: FatigueIndicators = {
      performanceDecline,
      errorRateIncrease: performanceDecline * 1.5,
      responseTimeIncrease: performanceDecline * 2,
      selfReportedFatigue: fatigueScore / 10,
      motivationLevel: 10 - fatigueScore / 10,
      fatigueScore,
    };

    const assessment: FatigueAssessment = {
      level,
      indicators,
      action,
      recommendedBreak,
      nextCheckIn: level === 'low' ? 30 : 10,
      timestamp: new Date(),
    };

    session.fatigueAssessments.push(assessment);

    // Throw error if critical
    if (level === 'critical') {
      throw new CognitiveError(
        CognitiveErrorCode.FATIGUE_CRITICAL,
        'Critical fatigue detected - session must end',
        { assessment }
      );
    }

    return assessment;
  }

  /**
   * Manage cognitive fatigue
   */
  async manageFatigue(
    sessionId: string,
    action: 'break' | 'reduce_intensity' | 'end_session'
  ): Promise<{ success: boolean; message: string }> {
    const session = this.activeSessions.get(sessionId);
    if (!session) {
      throw new CognitiveError(
        CognitiveErrorCode.SESSION_EXPIRED,
        'Session not found'
      );
    }

    switch (action) {
      case 'break':
        session.status = 'paused';
        return { success: true, message: 'Session paused for break' };

      case 'reduce_intensity':
        session.intensity = Math.max(0.3, session.intensity - 0.2);
        return { success: true, message: `Intensity reduced to ${session.intensity}` };

      case 'end_session':
        session.status = 'completed';
        session.endTime = new Date();
        return { success: true, message: 'Session ended' };

      default:
        return { success: false, message: 'Invalid action' };
    }
  }

  /**
   * Estimate current fatigue level (0-1)
   */
  private estimateFatigue(session: EnhancementSession): number {
    const sessionDuration = Date.now() - session.startTime.getTime();
    const durationMinutes = sessionDuration / (1000 * 60);

    // Fatigue increases with time and intensity
    const temporalFatigue = Math.min(durationMinutes / 120, 1.0); // 120 min = full fatigue
    const intensityFatigue = session.intensity * 0.5;
    const loadFatigue = this.calculateAverageLoad(session) * 0.3;

    return Math.min(temporalFatigue * 0.5 + intensityFatigue * 0.3 + loadFatigue * 0.2, 1.0);
  }

  /**
   * Calculate performance decline
   */
  private calculatePerformanceDecline(session: EnhancementSession): number {
    if (session.performanceData.length < 2) return 0;

    const recent = session.performanceData.slice(-5);
    const initial = session.performanceData.slice(0, 5);

    const recentAvg = recent.reduce((sum, p) => sum + p.accuracyRate, 0) / recent.length;
    const initialAvg = initial.reduce((sum, p) => sum + p.accuracyRate, 0) / initial.length;

    return Math.max(0, (initialAvg - recentAvg) / initialAvg * 100);
  }

  /**
   * Get current fatigue level
   */
  private getCurrentFatigueLevel(session: EnhancementSession): FatigueLevel {
    const fatigue = this.estimateFatigue(session) * 100;

    if (fatigue < 30) return 'low';
    if (fatigue < 60) return 'moderate';
    if (fatigue < 80) return 'high';
    return 'critical';
  }

  // ============================================================================
  // Decision Support
  // ============================================================================

  /**
   * Integrate decision support
   */
  async integrateDecisionSupport(request: DecisionSupportRequest): Promise<DecisionSupport> {
    const { sessionId, problem, supportLevel, autonomy } = request;

    const session = this.activeSessions.get(sessionId);
    if (!session) {
      throw new CognitiveError(
        CognitiveErrorCode.SESSION_EXPIRED,
        'Session not found'
      );
    }

    // Analyze problem
    const analysis = {
      decomposition: [
        'Identify key decision factors',
        'Evaluate alternatives',
        'Assess risks and benefits',
        'Consider stakeholder impacts',
      ],
      keyFactors: ['Time constraints', 'Resource availability', 'Risk tolerance'],
      uncertainties: ['Market conditions', 'External dependencies'],
    };

    // Generate recommendations
    const recommendations: Recommendation[] = [
      {
        id: 'R1',
        option: 'Option A: Conservative approach',
        confidence: 0.75,
        expectedOutcome: 'Lower risk, moderate reward',
        risks: ['Opportunity cost', 'Slower progress'],
        benefits: ['Higher certainty', 'Easier implementation'],
        rationale: 'Based on risk tolerance and available resources',
      },
      {
        id: 'R2',
        option: 'Option B: Aggressive approach',
        confidence: 0.60,
        expectedOutcome: 'Higher risk, higher reward',
        risks: ['Resource constraints', 'Execution difficulty'],
        benefits: ['Faster progress', 'Greater impact'],
        rationale: 'Maximizes potential outcomes but requires more resources',
      },
    ];

    // Determine augmentation level based on current enhancement
    const enhancementLevel = session.currentRatio;

    const result: DecisionSupport = {
      requestId: `DS-${Date.now()}`,
      analysis,
      recommendations,
      augmentation: {
        domains: [session.targetDomain, 'REASONING', 'EXECUTIVE'],
        methods: ['Memory augmentation', 'Reasoning assistance'],
        enhancementLevel,
      },
      qualityEstimate: {
        accuracy: 0.75 + enhancementLevel * 0.2,
        completeness: 0.70 + enhancementLevel * 0.25,
        robustness: 0.65 + enhancementLevel * 0.3,
        confidence: 0.70 + enhancementLevel * 0.2,
      },
      timestamp: new Date(),
    };

    return result;
  }

  // ============================================================================
  // Utility Functions
  // ============================================================================

  /**
   * Get all cognitive domains
   */
  private getAllDomains(): CognitiveDomain[] {
    return ['MEMORY', 'ATTENTION', 'REASONING', 'CREATIVITY', 'LANGUAGE', 'EXECUTIVE', 'SPATIAL'];
  }

  /**
   * Calculate cognitive index
   */
  private calculateCognitiveIndex(domainScores: Record<CognitiveDomain, DomainScore>): number {
    let weightedSum = 0;
    let totalWeight = 0;

    for (const domain of this.getAllDomains()) {
      const score = domainScores[domain];
      const weight = COGNITIVE_CONSTANTS.DOMAIN_WEIGHTS[domain];
      weightedSum += score.normalizedScore * weight;
      totalWeight += weight;
    }

    return weightedSum / totalWeight;
  }

  /**
   * Calculate enhancement potential
   */
  private calculateEnhancementPotential(
    domainScores: Record<CognitiveDomain, DomainScore>
  ): Record<CognitiveDomain, number> {
    const potential: Record<CognitiveDomain, number> = {} as any;

    for (const domain of this.getAllDomains()) {
      const score = domainScores[domain].normalizedScore;
      const maxRatio = COGNITIVE_CONSTANTS.MAX_ENHANCEMENT_RATIOS[domain];

      // Lower baseline scores have higher potential
      const relativePotential = (115 - score) / 30; // 0-1 scale
      potential[domain] = Math.min(maxRatio, relativePotential * maxRatio);
    }

    return potential;
  }

  /**
   * Recommend enhancement methods
   */
  private recommendMethods(
    domainScores: Record<CognitiveDomain, DomainScore>
  ): EnhancementMethod[] {
    const methods: EnhancementMethod[] = ['COMPUTATIONAL', 'TRAINING'];

    // All users can benefit from computational and training
    // Add others based on profile (simplified)
    const avgScore = this.getAllDomains()
      .reduce((sum, d) => sum + domainScores[d].normalizedScore, 0) / 7;

    if (avgScore < 95) {
      methods.push('ELECTRICAL');
    }

    return methods;
  }

  /**
   * Get method effectiveness for domain
   */
  private getMethodEffectiveness(method: EnhancementMethod, domain: CognitiveDomain): number {
    const effectiveness: Record<EnhancementMethod, Partial<Record<CognitiveDomain, number>>> = {
      PHARMACOLOGICAL: {
        ATTENTION: 0.5,
        MEMORY: 0.4,
        EXECUTIVE: 0.4,
      },
      ELECTRICAL: {
        ATTENTION: 0.3,
        MEMORY: 0.25,
        EXECUTIVE: 0.3,
      },
      COMPUTATIONAL: {
        MEMORY: 0.6,
        ATTENTION: 0.6,
        REASONING: 0.6,
        CREATIVITY: 0.6,
        LANGUAGE: 0.5,
        EXECUTIVE: 0.7,
        SPATIAL: 0.5,
      },
      TRAINING: {
        MEMORY: 0.4,
        ATTENTION: 0.4,
        REASONING: 0.3,
        CREATIVITY: 0.4,
        EXECUTIVE: 0.4,
      },
      HYBRID: {
        MEMORY: 0.6,
        ATTENTION: 0.7,
        REASONING: 0.5,
        CREATIVITY: 0.6,
        LANGUAGE: 0.5,
        EXECUTIVE: 0.6,
        SPATIAL: 0.5,
      },
    };

    return effectiveness[method][domain] || 0.3;
  }

  /**
   * Get domains that benefit from transfer effects
   */
  private getTransferDomains(domain: CognitiveDomain): CognitiveDomain[] {
    const transfers: Record<CognitiveDomain, CognitiveDomain[]> = {
      MEMORY: ['ATTENTION', 'EXECUTIVE'],
      ATTENTION: ['EXECUTIVE', 'MEMORY'],
      REASONING: ['EXECUTIVE', 'CREATIVITY'],
      CREATIVITY: ['REASONING', 'EXECUTIVE'],
      LANGUAGE: ['MEMORY', 'REASONING'],
      EXECUTIVE: ['ATTENTION', 'REASONING', 'MEMORY'],
      SPATIAL: ['REASONING'],
    };

    return transfers[domain] || [];
  }

  /**
   * Convert score to percentile
   */
  private scoreToPercentile(score: number): number {
    // Assuming normal distribution, mean=100, SD=15
    const z = (score - 100) / 15;
    // Simplified percentile calculation
    return Math.max(0, Math.min(100, 50 + z * 34));
  }

  /**
   * Validate enhancement request
   */
  private validateEnhancementRequest(request: EnhancementRequest): void {
    if (request.targetRatio < 0 || request.targetRatio > 0.8) {
      throw new CognitiveError(
        CognitiveErrorCode.INVALID_PARAMETERS,
        'Target ratio must be between 0 and 0.8'
      );
    }

    if (request.duration < 10 || request.duration > 120) {
      throw new CognitiveError(
        CognitiveErrorCode.INVALID_PARAMETERS,
        'Duration must be between 10 and 120 minutes'
      );
    }

    if (request.targetDomains.length === 0) {
      throw new CognitiveError(
        CognitiveErrorCode.INVALID_PARAMETERS,
        'At least one target domain required'
      );
    }
  }

  /**
   * Generate recommendations
   */
  private generateRecommendations(session: EnhancementSession, achievedRatio: number): string[] {
    const recommendations: string[] = [];

    if (achievedRatio < session.targetRatio * 0.8) {
      recommendations.push('Consider increasing session duration or intensity');
      recommendations.push('Ensure adequate rest before sessions');
    } else if (achievedRatio >= session.targetRatio) {
      recommendations.push('Target enhancement achieved successfully');
      recommendations.push('Monitor for sustainability over time');
    }

    const avgLoad = this.calculateAverageLoad(session);
    if (avgLoad > 0.7) {
      recommendations.push('Cognitive load was high - consider breaks in future sessions');
    }

    const fatigueLevel = this.getCurrentFatigueLevel(session);
    if (fatigueLevel === 'high' || fatigueLevel === 'critical') {
      recommendations.push('Significant fatigue detected - ensure adequate recovery time');
    }

    return recommendations;
  }

  /**
   * End an enhancement session
   */
  async endSession(sessionId: string): Promise<EnhancementResult> {
    const session = this.activeSessions.get(sessionId);
    if (!session) {
      throw new CognitiveError(
        CognitiveErrorCode.SESSION_EXPIRED,
        'Session not found'
      );
    }

    session.status = 'completed';
    session.endTime = new Date();

    // Generate final result
    const result = await this.enhanceDomain(sessionId, session.targetDomain, session.intensity);

    // Remove from active sessions
    this.activeSessions.delete(sessionId);

    return result;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Assess baseline (standalone)
 */
export async function assessBaseline(request: BaselineRequest): Promise<BaselineAssessment> {
  const sdk = new CognitiveEnhancementSDK();
  return sdk.assessBaseline(request);
}

/**
 * Enhance domain (standalone)
 */
export async function enhanceDomain(
  sessionId: string,
  domain: CognitiveDomain,
  intensity: number
): Promise<EnhancementResult> {
  const sdk = new CognitiveEnhancementSDK();
  return sdk.enhanceDomain(sessionId, domain, intensity);
}

/**
 * Measure performance (standalone)
 */
export async function measurePerformance(
  userId: string,
  domain: CognitiveDomain,
  sessionId?: string
): Promise<PerformanceMeasurement> {
  const sdk = new CognitiveEnhancementSDK();
  return sdk.measurePerformance(userId, domain, sessionId);
}

/**
 * Manage fatigue (standalone)
 */
export async function manageFatigue(
  sessionId: string,
  action: 'break' | 'reduce_intensity' | 'end_session'
): Promise<{ success: boolean; message: string }> {
  const sdk = new CognitiveEnhancementSDK();
  return sdk.manageFatigue(sessionId, action);
}

/**
 * Optimize attention (convenience function)
 */
export async function optimizeAttention(
  userId: string,
  duration: number = 45
): Promise<EnhancementSession> {
  const sdk = new CognitiveEnhancementSDK();
  return sdk.initiateEnhancement({
    userId,
    targetDomains: ['ATTENTION'],
    method: 'COMPUTATIONAL',
    targetRatio: 0.5,
    duration,
  });
}

/**
 * Integrate decision support (standalone)
 */
export async function integrateDecisionSupport(
  request: DecisionSupportRequest
): Promise<DecisionSupport> {
  const sdk = new CognitiveEnhancementSDK();
  return sdk.integrateDecisionSupport(request);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { CognitiveEnhancementSDK };
export default CognitiveEnhancementSDK;
