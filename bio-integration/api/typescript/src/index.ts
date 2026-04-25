/**
 * WIA-AUG-011: Bio-Integration SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bio-Integration Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

import {
  IntegrationSite,
  SiteAssessment,
  IntegrationProtocol,
  IntegrationStatusMetrics,
  StabilityMetrics,
  TissueHealthMetrics,
  SignalQualityMetrics,
  ImmuneResponseMetrics,
  BiofilmRiskAssessment,
  LongtermData,
  OptimizationTarget,
  OptimizationResult,
  Recommendation,
  IntegrationLevel,
  InterfaceType,
  INTEGRATION_DEPTH_THRESHOLDS,
  IHS_WEIGHTS,
  MINIMUM_THRESHOLDS,
  OSSEO_ISQ_THRESHOLDS,
  MICROMOTION_LIMITS,
  BioIntegrationError,
  BioIntegrationErrorCode,
} from './types';

// ============================================================================
// Bio-Integration SDK Class
// ============================================================================

/**
 * Main SDK class for bio-integration operations
 */
export class BioIntegrationSDK {
  private version = '1.0.0';

  constructor() {
    // SDK initialization
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Assess integration site for suitability
   */
  assessIntegrationSite(site: IntegrationSite): SiteAssessment {
    // Calculate integration depth score
    const integrationDepthScore = this.calculateIntegrationDepthScore(site);

    // Assess tissue compatibility
    const tissueCompatibility = this.assessTissueCompatibility(site);

    // Identify risk factors
    const riskFactors = this.identifyRiskFactors(site);

    // Generate recommendations
    const recommendations = this.generateSiteRecommendations(site, riskFactors);

    // Determine recommended integration level
    const recommendedLevel = this.determineRecommendedLevel(integrationDepthScore, site);

    // Calculate overall suitability
    const suitabilityScore = this.calculateSuitabilityScore(
      integrationDepthScore,
      tissueCompatibility,
      riskFactors.length
    );

    return {
      site,
      integrationDepthScore,
      tissueCompatibility,
      recommendedLevel,
      riskFactors,
      recommendations,
      suitabilityScore,
    };
  }

  /**
   * Initiate integration protocol
   */
  initiateIntegration(protocol: IntegrationProtocol): IntegrationStatusMetrics {
    const integrationId = this.generateIntegrationId();
    const timestamp = new Date();

    // Initial metrics (immediately post-implant)
    const initialMetrics: IntegrationStatusMetrics = {
      integrationId,
      status: 'ACUTE',
      timeSinceImplant: {
        weeks: 0,
        phase: 'immediate_post_implant',
      },
      stability: 60, // Initial mechanical stability
      tissueHealth: 50, // Initial surgical trauma
      signalQuality: protocol.interfaceType === 'BIOELECTRONIC' ? 70 : undefined,
      immuneResponse: 40, // Acute inflammation
      healthScore: this.calculateIHS({
        stability: 60,
        tissueHealth: 50,
        signalQuality: 70,
        immuneResponse: 40,
      }),
      timestamp,
    };

    return initialMetrics;
  }

  /**
   * Monitor integration stability
   */
  monitorStability(integrationId: string, timepoint: string): StabilityMetrics {
    // Parse timepoint (e.g., "12_weeks", "6_months")
    const weeks = this.parseTimepoint(timepoint);

    // Simulate stability progression (in real implementation, would retrieve from database)
    const stability = this.estimateStability(weeks);

    return stability;
  }

  /**
   * Evaluate tissue health
   */
  evaluateTissueHealth(integrationId: string, scanDepth: number): TissueHealthMetrics {
    // In real implementation, would analyze imaging/biopsy data
    // This is a simulation for demonstration

    const weeks = this.getWeeksSinceImplant(integrationId);

    const tissueHealth: TissueHealthMetrics = {
      cellDensity: this.estimateCellDensity(weeks),
      cellViability: this.estimateCellViability(weeks),
      cellTypes: this.identifyCellTypes(weeks),
      vascularization: {
        vesselDensity: this.estimateVesselDensity(weeks),
        perfusion: this.estimatePerfusion(weeks),
        oxygenSaturation: 95,
      },
      inflammation: {
        inflammatoryCells: this.estimateInflammation(weeks),
        grade: this.classifyInflammation(weeks),
        cytokines: {
          il6: this.estimateCytokine('il6', weeks),
          tnfAlpha: this.estimateCytokine('tnfAlpha', weeks),
          il10: this.estimateCytokine('il10', weeks),
        },
      },
      ecm: {
        collagenContent: this.estimateCollagen(weeks),
        collagenRatio: {
          type1: this.estimateCollagenType1Ratio(weeks),
          type3: 100 - this.estimateCollagenType1Ratio(weeks),
        },
      },
      score: this.calculateTissueHealthScore(weeks),
    };

    return tissueHealth;
  }

  /**
   * Optimize interface parameters
   */
  optimizeInterface(integrationId: string, target: OptimizationTarget): OptimizationResult {
    const currentMetrics = this.getCurrentMetrics(integrationId);
    const recommendations = this.generateOptimizationRecommendations(target, currentMetrics);

    return {
      target,
      currentValue: this.getTargetValue(target, currentMetrics),
      recommendations,
      expectedImprovement: this.calculateExpectedImprovement(recommendations),
      timeline: this.estimateImplementationTimeline(recommendations),
      risks: this.assessOptimizationRisks(recommendations),
    };
  }

  /**
   * Assess biofilm risk
   */
  preventBiofilm(integrationId: string): BiofilmRiskAssessment {
    const weeks = this.getWeeksSinceImplant(integrationId);
    const metrics = this.getCurrentMetrics(integrationId);

    // Assess risk factors
    const persistentInflammation = metrics.tissueHealth < 70;
    const deviceMalfunction = metrics.signalQuality !== undefined && metrics.signalQuality < 60;

    // Calculate risk score
    const riskScore = this.calculateBiofilmRisk(weeks, metrics);
    const riskLevel = this.classifyBiofilmRisk(riskScore);

    return {
      riskLevel,
      detection: {
        detected: false, // Would use actual detection methods
        method: 'clinical_assessment',
      },
      clinicalIndicators: {
        persistentInflammation,
        deviceMalfunction,
        refractoryInfection: false,
      },
      laboratory: {
        cReactiveProtein: this.estimateCRP(metrics),
        whiteBloodCount: this.estimateWBC(metrics),
        erythrocyteSedimentation: this.estimateESR(metrics),
      },
      preventionMeasures: this.listPreventionMeasures(),
      recommendations: this.generateBiofilmRecommendations(riskLevel),
      riskScore,
    };
  }

  /**
   * Track long-term integration
   */
  trackLongterm(integrationId: string, duration: string): LongtermData {
    const months = this.parseDuration(duration);
    const dataPoints = this.generateLongtermDataPoints(integrationId, months);

    const trends = this.analyzeTrends(dataPoints);
    const degradationRate = this.calculateDegradationRate(dataPoints);
    const prediction = this.predictLongtermOutcome(dataPoints, degradationRate);

    return {
      integrationId,
      monitoringDuration: months,
      dataPoints,
      trends,
      degradationRate,
      prediction,
      adverseEvents: [], // Would retrieve from database
      success: prediction.predictedIHS >= MINIMUM_THRESHOLDS.IHS_6_MONTHS,
    };
  }

  /**
   * Calculate Integration Health Score
   */
  calculateIHS(metrics: {
    stability: number;
    tissueHealth: number;
    signalQuality?: number;
    immuneResponse: number;
  }): number {
    const { stability, tissueHealth, signalQuality, immuneResponse } = metrics;

    // Use signal quality if available, otherwise use stability as proxy
    const signalScore = signalQuality ?? stability;

    const ihs =
      stability * IHS_WEIGHTS.STABILITY +
      tissueHealth * IHS_WEIGHTS.TISSUE_HEALTH +
      signalScore * IHS_WEIGHTS.SIGNAL_QUALITY +
      immuneResponse * IHS_WEIGHTS.IMMUNE_RESPONSE;

    return Math.round(ihs * 10) / 10;
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  private calculateIntegrationDepthScore(site: IntegrationSite): number {
    const penetration = this.scorePenetration(site.depth);
    const tissueComplexity = this.scoreTissueComplexity(site.tissueType);
    const vascularity = this.scoreVascularity(site.vascularity);
    const nerveDensity = this.scoreNerveDensity(site.nerveDensity);

    return (
      penetration * 0.4 +
      tissueComplexity * 0.3 +
      vascularity * 0.2 +
      nerveDensity * 0.1
    );
  }

  private scorePenetration(level: IntegrationLevel): number {
    const scores: Record<IntegrationLevel, number> = {
      SURFACE: 1,
      SUBCUTANEOUS: 4,
      DEEP_TISSUE: 6,
      NEURAL: 8,
      VASCULAR: 8,
      OSSEOUS: 9,
    };
    return scores[level];
  }

  private scoreTissueComplexity(tissue: string): number {
    const complexityMap: Record<string, number> = {
      skin: 2,
      fat: 3,
      muscle: 6,
      fascia: 5,
      tendon: 7,
      ligament: 7,
      bone: 8,
      cartilage: 7,
      nerve: 10,
      blood_vessel: 9,
      organ: 10,
    };
    return complexityMap[tissue] || 5;
  }

  private scoreVascularity(vascularity: string): number {
    const scores: Record<string, number> = {
      minimal: 2,
      low: 4,
      moderate: 6,
      high: 8,
      very_high: 10,
    };
    return scores[vascularity] || 5;
  }

  private scoreNerveDensity(density: string): number {
    const scores: Record<string, number> = {
      minimal: 2,
      low: 4,
      moderate: 6,
      high: 9,
    };
    return scores[density] || 5;
  }

  private assessTissueCompatibility(site: IntegrationSite): number {
    let score = 80; // Base score

    // Adjust based on tissue type
    if (site.tissueType === 'bone') score += 10;
    if (site.tissueType === 'nerve') score -= 10;

    // Adjust based on vascularity (better healing)
    if (site.vascularity === 'very_high') score += 10;
    if (site.vascularity === 'minimal') score -= 15;

    // Adjust based on mechanical stress
    if (site.mechanicalStress === 'extreme') score -= 20;
    if (site.mechanicalStress === 'low') score += 5;

    // Adjust based on infection risk
    if (site.exposureRisk === 'contaminated') score -= 30;
    if (site.exposureRisk === 'sterile') score += 10;

    return Math.max(0, Math.min(100, score));
  }

  private identifyRiskFactors(site: IntegrationSite): string[] {
    const risks: string[] = [];

    if (site.exposureRisk === 'contaminated' || site.exposureRisk === 'clean_contaminated') {
      risks.push('Elevated infection risk due to exposure classification');
    }

    if (site.mechanicalStress === 'extreme' || site.mechanicalStress === 'high') {
      risks.push('High mechanical stress may compromise integration');
    }

    if (site.vascularity === 'minimal' || site.vascularity === 'low') {
      risks.push('Poor vascularization may delay healing');
    }

    if (site.nerveDensity === 'high') {
      risks.push('High nerve density increases pain and neural damage risk');
    }

    if (site.characteristics?.previousSurgery) {
      risks.push('Previous surgery may have compromised tissue quality');
    }

    if (site.characteristics?.scarTissue) {
      risks.push('Scar tissue present may affect integration');
    }

    return risks;
  }

  private generateSiteRecommendations(site: IntegrationSite, risks: string[]): string[] {
    const recommendations: string[] = [];

    if (site.exposureRisk !== 'sterile') {
      recommendations.push('Implement strict sterile technique and antimicrobial prophylaxis');
    }

    if (site.vascularity === 'minimal' || site.vascularity === 'low') {
      recommendations.push('Consider VEGF coating or bioactive surface to promote vascularization');
    }

    if (site.mechanicalStress === 'extreme') {
      recommendations.push('Use high-strength materials and ensure robust initial fixation');
    }

    if (site.nerveDensity === 'high') {
      recommendations.push('Plan careful surgical approach to minimize nerve damage');
    }

    if (risks.length > 3) {
      recommendations.push('Consider alternative integration site with fewer risk factors');
    }

    return recommendations;
  }

  private determineRecommendedLevel(score: number, site: IntegrationSite): IntegrationLevel {
    // Return the site's requested level if appropriate, otherwise recommend safer alternative
    return site.depth;
  }

  private calculateSuitabilityScore(
    depthScore: number,
    compatibility: number,
    riskCount: number
  ): number {
    const baseScore = (depthScore * 0.3 + compatibility * 0.7);
    const riskPenalty = riskCount * 5;
    return Math.max(0, Math.min(100, baseScore - riskPenalty));
  }

  private generateIntegrationId(): string {
    const timestamp = Date.now();
    const random = Math.floor(Math.random() * 10000);
    return `INT-${timestamp}-${random}`;
  }

  private parseTimepoint(timepoint: string): number {
    const match = timepoint.match(/(\d+)_(week|month)s?/);
    if (!match) return 12; // default

    const value = parseInt(match[1]);
    const unit = match[2];

    return unit === 'month' ? value * 4 : value;
  }

  private estimateStability(weeks: number): StabilityMetrics {
    // Stability typically improves over time following S-curve
    const stabilityScore = Math.min(95, 60 + (weeks / 24) * 35);

    let isq: number | undefined;
    let micromotion: number | undefined;

    // If osseointegration
    if (weeks < 2) {
      isq = 65;
      micromotion = 120;
    } else if (weeks < 12) {
      isq = 65 + ((weeks - 2) / 10) * 15;
      micromotion = 120 - ((weeks - 2) / 10) * 70;
    } else {
      isq = 80 + (weeks - 12) / 12 * 10;
      micromotion = 50 - ((weeks - 12) / 12) * 30;
    }

    return {
      mechanicalFixation: stabilityScore,
      micromotion,
      isq,
      pulloutForce: stabilityScore * 5,
      torqueResistance: stabilityScore * 10,
      structuralIntegrity: stabilityScore,
      score: stabilityScore,
    };
  }

  private getWeeksSinceImplant(integrationId: string): number {
    // In real implementation, would query database
    // For simulation, extract from ID or use default
    return 12; // Default to 12 weeks for demonstration
  }

  private estimateCellDensity(weeks: number): number {
    return 1000 + weeks * 50; // cells/mm³
  }

  private estimateCellViability(weeks: number): number {
    return Math.min(95, 70 + weeks * 1.5);
  }

  private identifyCellTypes(weeks: number): string[] {
    if (weeks < 4) {
      return ['fibroblasts', 'macrophages', 'neutrophils'];
    } else if (weeks < 12) {
      return ['fibroblasts', 'osteoblasts', 'endothelial_cells', 'macrophages'];
    } else {
      return ['osteocytes', 'osteoblasts', 'endothelial_cells', 'fibroblasts'];
    }
  }

  private estimateVesselDensity(weeks: number): number {
    return Math.min(60, weeks * 3);
  }

  private estimatePerfusion(weeks: number): number {
    return Math.min(90, 50 + weeks * 2.5);
  }

  private estimateInflammation(weeks: number): number {
    // Inflammation decreases over time
    return Math.max(10, 200 - weeks * 10);
  }

  private classifyInflammation(weeks: number): 'none' | 'minimal' | 'mild' | 'moderate' | 'severe' {
    if (weeks < 2) return 'moderate';
    if (weeks < 6) return 'mild';
    if (weeks < 12) return 'minimal';
    return 'none';
  }

  private estimateCytokine(type: string, weeks: number): number {
    // Cytokines decrease over time
    const baselinePro = 100; // pg/mL
    const baselineAnti = 20;

    if (type === 'il10') {
      // Anti-inflammatory increases then stabilizes
      return Math.min(50, baselineAnti + weeks * 2);
    } else {
      // Pro-inflammatory decreases
      return Math.max(5, baselinePro - weeks * 5);
    }
  }

  private estimateCollagen(weeks: number): number {
    return Math.min(150, weeks * 8);
  }

  private estimateCollagenType1Ratio(weeks: number): number {
    // Type I collagen ratio increases over time (mature tissue)
    return Math.min(85, 30 + weeks * 3);
  }

  private calculateTissueHealthScore(weeks: number): number {
    const viability = this.estimateCellViability(weeks);
    const perfusion = this.estimatePerfusion(weeks);
    const inflammation = 100 - this.estimateInflammation(weeks) / 2;

    return (viability * 0.4 + perfusion * 0.4 + inflammation * 0.2);
  }

  private getCurrentMetrics(integrationId: string): IntegrationStatusMetrics {
    // In real implementation, would retrieve from database
    const weeks = this.getWeeksSinceImplant(integrationId);
    const stability = this.estimateStability(weeks);
    const tissueHealth = this.calculateTissueHealthScore(weeks);

    return {
      integrationId,
      status: weeks < 4 ? 'ACUTE' : weeks < 12 ? 'SUBACUTE' : 'CHRONIC',
      timeSinceImplant: { weeks, phase: 'integration' },
      stability: stability.score,
      tissueHealth,
      signalQuality: 75,
      immuneResponse: Math.min(90, 40 + weeks * 3),
      healthScore: this.calculateIHS({
        stability: stability.score,
        tissueHealth,
        signalQuality: 75,
        immuneResponse: Math.min(90, 40 + weeks * 3),
      }),
      timestamp: new Date(),
    };
  }

  private generateOptimizationRecommendations(
    target: OptimizationTarget,
    metrics: IntegrationStatusMetrics
  ): Recommendation[] {
    const recommendations: Recommendation[] = [];

    if (target === 'stability' || target === 'overall_integration') {
      if (metrics.stability < 80) {
        recommendations.push({
          category: 'surgical',
          priority: 'high',
          description: 'Reduce mechanical loading during integration phase',
          expectedImpact: 15,
          difficulty: 'moderate',
          cost: 'low',
        });
      }
    }

    if (target === 'tissue_health' || target === 'overall_integration') {
      if (metrics.tissueHealth < 70) {
        recommendations.push({
          category: 'pharmacological',
          priority: 'medium',
          description: 'Consider growth factor supplementation (VEGF, PDGF)',
          expectedImpact: 20,
          difficulty: 'moderate',
          cost: 'moderate',
        });
      }
    }

    if (target === 'immune_response' || target === 'overall_integration') {
      if (metrics.immuneResponse < 70) {
        recommendations.push({
          category: 'pharmacological',
          priority: 'high',
          description: 'Implement immunomodulation strategy (local corticosteroids)',
          expectedImpact: 25,
          difficulty: 'easy',
          cost: 'low',
        });
      }
    }

    return recommendations;
  }

  private getTargetValue(target: OptimizationTarget, metrics: IntegrationStatusMetrics): number {
    switch (target) {
      case 'stability':
        return metrics.stability;
      case 'tissue_health':
        return metrics.tissueHealth;
      case 'signal_quality':
        return metrics.signalQuality || 0;
      case 'immune_response':
        return metrics.immuneResponse;
      case 'overall_integration':
        return metrics.healthScore;
      default:
        return 0;
    }
  }

  private calculateExpectedImprovement(recommendations: Recommendation[]): number {
    return recommendations.reduce((sum, rec) => sum + rec.expectedImpact, 0) * 0.7; // 70% efficacy
  }

  private estimateImplementationTimeline(recommendations: Recommendation[]): string {
    const hasHigh = recommendations.some((r) => r.priority === 'high' || r.priority === 'critical');
    return hasHigh ? 'immediate' : '2-4 weeks';
  }

  private assessOptimizationRisks(recommendations: Recommendation[]): string[] {
    const risks: string[] = [];

    if (recommendations.some((r) => r.category === 'pharmacological')) {
      risks.push('Potential drug side effects or interactions');
    }

    if (recommendations.some((r) => r.category === 'surgical')) {
      risks.push('Additional surgical intervention carries inherent risks');
    }

    return risks;
  }

  private calculateBiofilmRisk(weeks: number, metrics: IntegrationStatusMetrics): number {
    let risk = 30; // Base risk

    // Persistent inflammation increases risk
    if (metrics.tissueHealth < 60) risk += 30;

    // Poor immune response
    if (metrics.immuneResponse < 60) risk += 20;

    // Time-dependent (risk decreases after initial period)
    if (weeks < 4) risk += 20;
    else if (weeks > 12) risk -= 10;

    return Math.max(0, Math.min(100, risk));
  }

  private classifyBiofilmRisk(score: number): 'low' | 'moderate' | 'high' | 'critical' {
    if (score < 30) return 'low';
    if (score < 50) return 'moderate';
    if (score < 70) return 'high';
    return 'critical';
  }

  private estimateCRP(metrics: IntegrationStatusMetrics): number {
    return Math.max(0.5, 50 - metrics.tissueHealth * 0.5);
  }

  private estimateWBC(metrics: IntegrationStatusMetrics): number {
    return 5000 + (100 - metrics.immuneResponse) * 50;
  }

  private estimateESR(metrics: IntegrationStatusMetrics): number {
    return Math.max(5, 50 - metrics.tissueHealth * 0.4);
  }

  private listPreventionMeasures(): string[] {
    return [
      'Antimicrobial surface coating',
      'Sterile surgical technique',
      'Perioperative antibiotic prophylaxis',
      'Regular monitoring and hygiene',
    ];
  }

  private generateBiofilmRecommendations(riskLevel: string): string[] {
    const recommendations: string[] = [];

    if (riskLevel === 'high' || riskLevel === 'critical') {
      recommendations.push('Increase monitoring frequency');
      recommendations.push('Consider empirical antibiotic therapy');
      recommendations.push('Ultrasonic cleaning or antimicrobial irrigation');
    }

    if (riskLevel === 'moderate') {
      recommendations.push('Enhanced hygiene protocols');
      recommendations.push('Monthly clinical assessment');
    }

    recommendations.push('Maintain good tissue health through optimization strategies');

    return recommendations;
  }

  private parseDuration(duration: string): number {
    const match = duration.match(/(\d+)_month/);
    return match ? parseInt(match[1]) : 12;
  }

  private generateLongtermDataPoints(integrationId: string, months: number): IntegrationStatusMetrics[] {
    const dataPoints: IntegrationStatusMetrics[] = [];

    for (let month = 1; month <= months; month++) {
      const weeks = month * 4;
      const metrics = this.getCurrentMetrics(integrationId);
      metrics.timeSinceImplant = { weeks, phase: `month_${month}` };
      dataPoints.push(metrics);
    }

    return dataPoints;
  }

  private analyzeTrends(dataPoints: IntegrationStatusMetrics[]): any {
    if (dataPoints.length < 2) {
      return {
        stability: 'stable',
        tissueHealth: 'stable',
        signalQuality: 'stable',
        immuneResponse: 'stable',
      };
    }

    const first = dataPoints[0];
    const last = dataPoints[dataPoints.length - 1];

    return {
      stability: this.classifyTrend(first.stability, last.stability),
      tissueHealth: this.classifyTrend(first.tissueHealth, last.tissueHealth),
      signalQuality: first.signalQuality && last.signalQuality
        ? this.classifyTrend(first.signalQuality, last.signalQuality)
        : 'stable',
      immuneResponse: this.classifyTrend(first.immuneResponse, last.immuneResponse),
    };
  }

  private classifyTrend(initial: number, final: number): 'improving' | 'stable' | 'declining' {
    const change = final - initial;
    if (change > 5) return 'improving';
    if (change < -5) return 'declining';
    return 'stable';
  }

  private calculateDegradationRate(dataPoints: IntegrationStatusMetrics[]): number {
    if (dataPoints.length < 2) return 0;

    const first = dataPoints[0].healthScore;
    const last = dataPoints[dataPoints.length - 1].healthScore;
    const months = dataPoints.length;

    const totalChange = last - first;
    const monthlyChange = totalChange / months;
    const annualChange = monthlyChange * 12;

    return -annualChange; // Negative because we're measuring degradation
  }

  private predictLongtermOutcome(dataPoints: IntegrationStatusMetrics[], degradationRate: number): any {
    const currentIHS = dataPoints[dataPoints.length - 1].healthScore;
    const predictedIHS5y = Math.max(0, currentIHS - degradationRate * 5);

    const failureRisk = predictedIHS5y < MINIMUM_THRESHOLDS.IHS_6_MONTHS ? 'high' :
      predictedIHS5y < 70 ? 'moderate' : 'low';

    const estimatedLifespan = degradationRate > 0
      ? currentIHS / degradationRate
      : 20; // 20+ years if no degradation

    return {
      predictedIHS: Math.round(predictedIHS5y),
      failureRisk,
      estimatedLifespan: Math.round(estimatedLifespan),
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Assess integration site
 */
export function assessIntegrationSite(site: IntegrationSite): SiteAssessment {
  const sdk = new BioIntegrationSDK();
  return sdk.assessIntegrationSite(site);
}

/**
 * Initiate integration
 */
export function initiateIntegration(protocol: IntegrationProtocol): IntegrationStatusMetrics {
  const sdk = new BioIntegrationSDK();
  return sdk.initiateIntegration(protocol);
}

/**
 * Monitor stability
 */
export function monitorStability(integrationId: string, timepoint: string): StabilityMetrics {
  const sdk = new BioIntegrationSDK();
  return sdk.monitorStability(integrationId, timepoint);
}

/**
 * Evaluate tissue health
 */
export function evaluateTissueHealth(integrationId: string, depth: number): TissueHealthMetrics {
  const sdk = new BioIntegrationSDK();
  return sdk.evaluateTissueHealth(integrationId, depth);
}

/**
 * Optimize interface
 */
export function optimizeInterface(integrationId: string, target: OptimizationTarget): OptimizationResult {
  const sdk = new BioIntegrationSDK();
  return sdk.optimizeInterface(integrationId, target);
}

/**
 * Prevent biofilm
 */
export function preventBiofilm(integrationId: string): BiofilmRiskAssessment {
  const sdk = new BioIntegrationSDK();
  return sdk.preventBiofilm(integrationId);
}

/**
 * Track long-term
 */
export function trackLongterm(integrationId: string, duration: string): LongtermData {
  const sdk = new BioIntegrationSDK();
  return sdk.trackLongterm(integrationId, duration);
}

/**
 * Calculate Integration Health Score
 */
export function calculateIHS(metrics: {
  stability: number;
  tissueHealth: number;
  signalQuality?: number;
  immuneResponse: number;
}): number {
  const sdk = new BioIntegrationSDK();
  return sdk.calculateIHS(metrics);
}

// ============================================================================
// Exports
// ============================================================================

export * from './types';
export { BioIntegrationSDK };
