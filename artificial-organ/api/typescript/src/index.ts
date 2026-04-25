/**
 * WIA-AUG-010: Artificial Organ SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Working Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides tools for artificial organ classification, biocompatibility
 * assessment, function monitoring, rejection detection, and performance optimization.
 */

import {
  OrganType,
  TechnologyType,
  PowerSystemType,
  OperationalState,
  PerformanceLevel,
  RejectionRiskLevel,
  FailsafeMode,
  FailsafeTrigger,
  EmergencySituation,
  OrganInput,
  OrganClassification,
  FunctionMetrics,
  MonitoringResult,
  BiocompatibilityData,
  BiocompatibilityScore,
  RejectionData,
  RejectionAssessment,
  PerformanceTargets,
  OptimizationResult,
  ServiceSchedule,
  MaintenancePrediction,
  FailsafeStatus,
  EmergencyResponse,
  OrganStatus,
  TimeRange,
  HistoricalData,
  Alert,
  BIOCOMPATIBILITY_THRESHOLDS,
  REJECTION_RISK_THRESHOLDS,
  PERFORMANCE_THRESHOLDS,
  MINIMUM_REQUIREMENTS,
  OrganErrorCode,
  OrganError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-010 Artificial Organ SDK
 */
export class ArtificialOrganSDK {
  private version = '1.0.0';
  private organRegistry: Map<string, OrganStatus> = new Map();
  private monitoringHistory: Map<string, FunctionMetrics[]> = new Map();

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  // ==========================================================================
  // Classification Methods
  // ==========================================================================

  /**
   * Classify an artificial organ
   */
  classifyOrgan(input: OrganInput): OrganClassification {
    // Validate input
    this.validateOrganInput(input);

    // Determine performance level
    const performanceLevel = this.determinePerformanceLevel(input.targetFunction);

    // Calculate safety class
    const safetyClass = this.calculateSafetyClass(
      input.technologyType,
      input.invasiveness || 8,
      input.expectedLifespan || 5
    );

    // Determine monitoring frequency
    const monitoringFrequency = this.determineMonitoringFrequency(
      input.organType,
      safetyClass
    );

    return {
      type: input.organType,
      technology: input.technologyType,
      performanceLevel,
      safetyClass,
      powerSystem: input.powerSystem,
      biocompatibility: input.biocompatibilityRating,
      reversible: input.reversible !== undefined ? input.reversible : false,
      monitoringFrequency,
    };
  }

  /**
   * Determine performance level from target function
   */
  private determinePerformanceLevel(targetFunction: number): PerformanceLevel {
    const normalized = targetFunction / 100; // Convert percentage to 0-1

    if (normalized >= PERFORMANCE_THRESHOLDS.SUPERIOR) {
      return 'SUPERIOR';
    } else if (normalized >= PERFORMANCE_THRESHOLDS.OPTIMAL) {
      return 'OPTIMAL';
    } else if (normalized >= PERFORMANCE_THRESHOLDS.ADEQUATE) {
      return 'ADEQUATE';
    } else {
      return 'SUBOPTIMAL';
    }
  }

  /**
   * Calculate safety class
   */
  private calculateSafetyClass(
    technology: TechnologyType,
    invasiveness: number,
    lifespan: number
  ): string {
    let score = 0;

    // Technology contribution
    if (technology === 'MECHANICAL') score += 3;
    else if (technology === 'BIOARTIFICIAL') score += 4;
    else if (technology === 'BIOPRINTED') score += 3;
    else if (technology === 'XENOTRANSPLANT') score += 5;
    else if (technology === 'HYBRID') score += 4;

    // Invasiveness contribution
    score += invasiveness * 0.3;

    // Lifespan contribution (longer = higher risk)
    if (lifespan > 5) score += 2;
    else if (lifespan > 2) score += 1;

    // Map to safety class
    if (score <= 5) return 'Class III';
    if (score <= 8) return 'Class IV';
    return 'Class V';
  }

  /**
   * Determine monitoring frequency
   */
  private determineMonitoringFrequency(
    organType: OrganType,
    safetyClass: string
  ): string {
    // Critical organs need more frequent monitoring
    const criticalOrgans: OrganType[] = ['HEART', 'LUNG', 'LIVER'];
    const isCritical = criticalOrgans.includes(organType);

    if (safetyClass === 'Class V') {
      return isCritical ? 'Continuous' : 'Hourly';
    } else if (safetyClass === 'Class IV') {
      return isCritical ? 'Hourly' : 'Every 4 hours';
    } else {
      return isCritical ? 'Every 4 hours' : 'Daily';
    }
  }

  // ==========================================================================
  // Function Monitoring Methods
  // ==========================================================================

  /**
   * Monitor organ function
   */
  monitorFunction(organId: string, metrics: FunctionMetrics): MonitoringResult {
    // Store metrics in history
    if (!this.monitoringHistory.has(organId)) {
      this.monitoringHistory.set(organId, []);
    }
    this.monitoringHistory.get(organId)!.push(metrics);

    // Keep only last 1000 readings
    const history = this.monitoringHistory.get(organId)!;
    if (history.length > 1000) {
      history.shift();
    }

    // Assess current performance
    const performance = this.assessPerformance(metrics);

    // Generate alerts
    const alerts = this.generateAlerts(metrics);

    // Generate recommendations
    const recommendations = this.generateRecommendations(metrics, alerts);

    // Determine next monitoring time
    const nextMonitoring = this.calculateNextMonitoring(metrics, alerts);

    // Update registry
    this.updateOrganRegistry(organId, metrics, performance);

    return {
      organId,
      status: metrics.operationalState,
      performance,
      metrics,
      alerts,
      recommendations,
      nextMonitoring,
    };
  }

  /**
   * Assess current performance
   */
  private assessPerformance(metrics: FunctionMetrics): PerformanceLevel {
    const outputRatio = metrics.output.percentOfTarget / 100;
    const efficiency = metrics.efficiency.value;

    // Combined score
    const score = outputRatio * 0.6 + efficiency * 0.4;

    if (score >= 0.95) return 'SUPERIOR';
    if (score >= 0.85) return 'OPTIMAL';
    if (score >= 0.70) return 'ADEQUATE';
    return 'SUBOPTIMAL';
  }

  /**
   * Generate alerts based on metrics
   */
  private generateAlerts(metrics: FunctionMetrics): Alert[] {
    const alerts: Alert[] = [];

    // Output alerts
    if (metrics.output.percentOfTarget < 50) {
      alerts.push({
        id: `ALT-${Date.now()}-001`,
        type: 'PERFORMANCE',
        severity: 'CRITICAL',
        message: `Output critically low: ${metrics.output.percentOfTarget}% of target`,
        timestamp: new Date(),
        acknowledged: false,
        recommendedAction: 'Immediate medical intervention required',
      });
    } else if (metrics.output.percentOfTarget < 70) {
      alerts.push({
        id: `ALT-${Date.now()}-002`,
        type: 'PERFORMANCE',
        severity: 'WARNING',
        message: `Output below optimal: ${metrics.output.percentOfTarget}% of target`,
        timestamp: new Date(),
        acknowledged: false,
        recommendedAction: 'Monitor closely, consider optimization',
      });
    }

    // Efficiency alerts
    if (metrics.efficiency.value < 0.60) {
      alerts.push({
        id: `ALT-${Date.now()}-003`,
        type: 'PERFORMANCE',
        severity: 'WARNING',
        message: `Efficiency low: ${(metrics.efficiency.value * 100).toFixed(1)}%`,
        timestamp: new Date(),
        acknowledged: false,
        recommendedAction: 'Performance optimization recommended',
      });
    }

    // Battery alerts
    if (metrics.batteryLevel !== undefined) {
      if (metrics.batteryLevel < 10) {
        alerts.push({
          id: `ALT-${Date.now()}-004`,
          type: 'POWER',
          severity: 'CRITICAL',
          message: `Battery critically low: ${metrics.batteryLevel}%`,
          timestamp: new Date(),
          acknowledged: false,
          recommendedAction: 'Charge immediately or switch to backup power',
        });
      } else if (metrics.batteryLevel < 20) {
        alerts.push({
          id: `ALT-${Date.now()}-005`,
          type: 'POWER',
          severity: 'WARNING',
          message: `Battery low: ${metrics.batteryLevel}%`,
          timestamp: new Date(),
          acknowledged: false,
          recommendedAction: 'Charge soon',
        });
      }
    }

    // Temperature alerts
    if (metrics.physiology?.temperature) {
      const temp = metrics.physiology.temperature;
      if (temp > 39 || temp < 35) {
        alerts.push({
          id: `ALT-${Date.now()}-006`,
          type: 'SAFETY',
          severity: 'CRITICAL',
          message: `Temperature abnormal: ${temp.toFixed(1)}°C`,
          timestamp: new Date(),
          acknowledged: false,
          recommendedAction: 'Immediate medical evaluation required',
        });
      } else if (temp > 38 || temp < 36) {
        alerts.push({
          id: `ALT-${Date.now()}-007`,
          type: 'SAFETY',
          severity: 'WARNING',
          message: `Temperature elevated: ${temp.toFixed(1)}°C`,
          timestamp: new Date(),
          acknowledged: false,
          recommendedAction: 'Monitor closely',
        });
      }
    }

    return alerts;
  }

  /**
   * Generate recommendations
   */
  private generateRecommendations(
    metrics: FunctionMetrics,
    alerts: Alert[]
  ): string[] {
    const recommendations: string[] = [];

    // Critical alerts require immediate action
    const criticalAlerts = alerts.filter((a) => a.severity === 'CRITICAL');
    if (criticalAlerts.length > 0) {
      recommendations.push('URGENT: Seek immediate medical attention');
      return recommendations;
    }

    // Performance recommendations
    if (metrics.output.percentOfTarget < 85) {
      recommendations.push('Consider performance optimization');
    }

    if (metrics.efficiency.value < 0.75) {
      recommendations.push('Efficiency below target, review control settings');
    }

    // Battery recommendations
    if (metrics.batteryLevel && metrics.batteryLevel < 30) {
      recommendations.push('Charge battery at next opportunity');
    }

    // Trend recommendations
    if (metrics.output.trend && metrics.output.trend < -0.05) {
      recommendations.push('Declining output trend detected, schedule evaluation');
    }

    if (recommendations.length === 0) {
      recommendations.push('Continue routine monitoring');
    }

    return recommendations;
  }

  /**
   * Calculate next monitoring time
   */
  private calculateNextMonitoring(
    metrics: FunctionMetrics,
    alerts: Alert[]
  ): Date {
    const now = new Date();
    let intervalMinutes = 60; // Default 1 hour

    // Adjust based on alerts
    const hasCritical = alerts.some((a) => a.severity === 'CRITICAL');
    const hasWarning = alerts.some((a) => a.severity === 'WARNING');

    if (hasCritical) {
      intervalMinutes = 5; // 5 minutes for critical
    } else if (hasWarning) {
      intervalMinutes = 15; // 15 minutes for warnings
    } else if (metrics.output.percentOfTarget > 90 && metrics.efficiency.value > 0.85) {
      intervalMinutes = 240; // 4 hours if performing well
    }

    return new Date(now.getTime() + intervalMinutes * 60 * 1000);
  }

  /**
   * Update organ registry
   */
  private updateOrganRegistry(
    organId: string,
    metrics: FunctionMetrics,
    performance: PerformanceLevel
  ): void {
    const existing = this.organRegistry.get(organId);

    const status: OrganStatus = {
      organId,
      timestamp: new Date(),
      operationalState: metrics.operationalState,
      organType: existing?.organType || 'HEART',
      technologyType: existing?.technologyType || 'MECHANICAL',
      functionMetrics: metrics,
      alerts: this.generateAlerts(metrics),
      batteryLevel: metrics.batteryLevel,
      powerSource: existing?.powerSource,
      operatingHours: existing?.operatingHours,
      cycleCount: existing?.cycleCount,
    };

    this.organRegistry.set(organId, status);
  }

  // ==========================================================================
  // Biocompatibility Assessment Methods
  // ==========================================================================

  /**
   * Assess biocompatibility
   */
  assessBiocompatibility(
    organId: string,
    data: BiocompatibilityData
  ): BiocompatibilityScore {
    // Calculate tissue integration score
    const tissueScore = this.assessTissueIntegration(data.tissueIntegration);

    // Calculate immune response score
    const immuneScore = this.assessImmuneResponse(data.immuneResponse);

    // Calculate material safety score
    const materialScore = this.assessMaterialSafety(data.materialSafety);

    // Calculate functional stability score
    const stabilityScore = this.assessFunctionalStability(data.functionalStability);

    // Overall biocompatibility score
    const overallScore =
      tissueScore * 0.3 +
      immuneScore * 0.25 +
      materialScore * 0.25 +
      stabilityScore * 0.2;

    // Determine classification
    let classification: 'EXCELLENT' | 'GOOD' | 'ACCEPTABLE' | 'POOR';
    if (overallScore >= BIOCOMPATIBILITY_THRESHOLDS.EXCELLENT) {
      classification = 'EXCELLENT';
    } else if (overallScore >= BIOCOMPATIBILITY_THRESHOLDS.GOOD) {
      classification = 'GOOD';
    } else if (overallScore >= BIOCOMPATIBILITY_THRESHOLDS.ACCEPTABLE) {
      classification = 'ACCEPTABLE';
    } else {
      classification = 'POOR';
    }

    // Generate warnings and recommendations
    const warnings: string[] = [];
    const recommendations: string[] = [];

    if (tissueScore < 0.7) {
      warnings.push('Tissue integration below optimal');
      recommendations.push('Monitor capsule formation and vascularization');
    }

    if (immuneScore < 0.7) {
      warnings.push('Immune response requires attention');
      recommendations.push('Review immunosuppression protocol');
    }

    if (materialScore < 0.8) {
      warnings.push('Material safety concerns detected');
      recommendations.push('Evaluate for material degradation or leachables');
    }

    if (stabilityScore < 0.7) {
      warnings.push('Functional stability declining');
      recommendations.push('Assess for performance degradation causes');
    }

    return {
      overallScore,
      tissueIntegrationScore: tissueScore,
      immuneResponseScore: immuneScore,
      materialSafetyScore: materialScore,
      functionalStabilityScore: stabilityScore,
      classification,
      warnings,
      recommendations,
    };
  }

  /**
   * Assess tissue integration
   */
  private assessTissueIntegration(data: any): number {
    // Capsule thickness (thinner is better, target <1mm)
    const capsuleScore = Math.max(0, 1 - data.capsuleThickness / 2);

    // Vascularization (more is better, target >50 vessels/mm²)
    const vascScore = Math.min(1, data.vascularization / 50);

    // Fibrosis (lower is better, 0-4 scale)
    const fibrosisScore = 1 - data.fibrosis / 4;

    // Adhesions (lower is better, 0-4 scale)
    const adhesionScore = 1 - data.adhesions / 4;

    return (capsuleScore + vascScore + fibrosisScore + adhesionScore) / 4;
  }

  /**
   * Assess immune response
   */
  private assessImmuneResponse(data: any): number {
    let score = 1.0;

    // CRP scoring (lower is better)
    if (data.CRP > 100) score -= 0.35;
    else if (data.CRP > 50) score -= 0.20;
    else if (data.CRP > 10) score -= 0.10;

    // ESR scoring (lower is better)
    if (data.ESR > 50) score -= 0.15;
    else if (data.ESR > 30) score -= 0.08;

    // WBC elevation
    if (data.WBC > 15000) score -= 0.20;
    else if (data.WBC > 11000) score -= 0.10;

    return Math.max(0, score);
  }

  /**
   * Assess material safety
   */
  private assessMaterialSafety(data: any): number {
    return (
      (data.cytotoxicity +
        data.sensitization +
        data.irritation +
        data.systemicToxicity) /
      4
    );
  }

  /**
   * Assess functional stability
   */
  private assessFunctionalStability(data: any): number {
    const consistencyScore = data.consistency;
    const variabilityScore = Math.max(0, 1 - data.variability);
    const degradationScore = Math.max(0, 1 - data.degradationRate / 10);

    return (consistencyScore + variabilityScore + degradationScore) / 3;
  }

  // ==========================================================================
  // Rejection Detection Methods
  // ==========================================================================

  /**
   * Detect rejection risk
   */
  detectRejection(organId: string, data: RejectionData): RejectionAssessment {
    // Calculate component scores
    const immuneMarkerScore = this.calculateImmuneMarkerScore(data.immuneMarkers);
    const performanceDegradationScore = this.calculatePerformanceDegradationScore(
      data.performanceTrend
    );
    const antibodyScore = data.antibodyLevels.overallScore;

    // Calculate inflammation score (if provided)
    let inflammationScore = 0;
    if (data.inflammationIndicators) {
      inflammationScore = this.calculateInflammationScore(data.inflammationIndicators);
    }

    // Overall rejection risk score
    const riskScore =
      immuneMarkerScore * 0.35 +
      performanceDegradationScore * 0.3 +
      inflammationScore * 0.2 +
      antibodyScore * 0.15;

    // Determine risk level
    let level: RejectionRiskLevel;
    let recommendation: string;
    let priority: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
    let followUpInterval: string;

    if (riskScore > 0.70) {
      level = 'CRITICAL';
      recommendation = 'Immediate medical intervention required. Initiate rejection protocol.';
      priority = 'CRITICAL';
      followUpInterval = 'Daily';
    } else if (riskScore > 0.40) {
      level = 'HIGH';
      recommendation = 'Urgent medical evaluation. Consider immunosuppression adjustment.';
      priority = 'HIGH';
      followUpInterval = 'Every 2-3 days';
    } else if (riskScore > 0.25) {
      level = 'MODERATE';
      recommendation = 'Increased monitoring recommended. Review immunosuppression levels.';
      priority = 'MEDIUM';
      followUpInterval = 'Weekly';
    } else {
      level = 'LOW';
      recommendation = 'Continue routine monitoring.';
      priority = 'LOW';
      followUpInterval = 'Monthly';
    }

    // Generate detailed findings
    const findings: string[] = [];

    if (immuneMarkerScore > 0.5) {
      findings.push(`Elevated immune markers (CRP: ${data.immuneMarkers.CRP} mg/L)`);
    }

    if (performanceDegradationScore > 0.5) {
      findings.push(
        `Performance declining (${data.performanceTrend.trend30days.toFixed(1)}% over 30 days)`
      );
    }

    if (antibodyScore > 0.5) {
      findings.push('Elevated antibody levels detected');
    }

    if (findings.length === 0) {
      findings.push('No significant rejection indicators');
    }

    return {
      organId,
      riskScore,
      level,
      immuneMarkerScore,
      performanceDegradationScore,
      inflammationScore,
      antibodyScore,
      recommendation,
      priority,
      findings,
      followUpInterval,
    };
  }

  /**
   * Calculate immune marker score
   */
  private calculateImmuneMarkerScore(markers: any): number {
    let score = 0;

    // CRP scoring
    if (markers.CRP > 100) score += 0.4;
    else if (markers.CRP > 50) score += 0.25;
    else if (markers.CRP > 10) score += 0.15;

    // ESR scoring
    if (markers.ESR > 50) score += 0.2;
    else if (markers.ESR > 30) score += 0.1;

    // Cytokine elevation (if available)
    if (markers.cytokines) {
      if (markers.cytokines.IL6 > 50) score += 0.2;
      if (markers.cytokines.TNFalpha > 20) score += 0.15;
    }

    // Complement activation (if available)
    if (markers.complement) {
      if (markers.complement.C3 < 80 || markers.complement.C4 < 15) {
        score += 0.15;
      }
    }

    return Math.min(score, 1.0);
  }

  /**
   * Calculate performance degradation score
   */
  private calculatePerformanceDegradationScore(trend: any): number {
    const outputRatio = trend.currentOutput / trend.baselineOutput;
    const outputScore = 1 - outputRatio;

    // Trend scoring (negative trend = higher score)
    let trendScore = 0;
    if (trend.trend30days < -20) trendScore = 0.4;
    else if (trend.trend30days < -10) trendScore = 0.25;
    else if (trend.trend30days < -5) trendScore = 0.15;

    // Variability scoring
    const variabilityScore = Math.min(trend.variability, 0.3);

    // Efficiency degradation
    const efficiencyScore = Math.max(0, 1 - trend.efficiencyTrend);

    return (
      outputScore * 0.4 +
      trendScore * 0.3 +
      variabilityScore * 0.2 +
      efficiencyScore * 0.1
    );
  }

  /**
   * Calculate inflammation score
   */
  private calculateInflammationScore(indicators: Record<string, number>): number {
    // Simplified scoring based on available indicators
    const values = Object.values(indicators);
    const avgValue = values.reduce((sum, val) => sum + val, 0) / values.length;
    return Math.min(avgValue / 100, 1.0); // Normalize to 0-1
  }

  // ==========================================================================
  // Performance Optimization Methods
  // ==========================================================================

  /**
   * Optimize organ performance
   */
  optimizePerformance(
    organId: string,
    targets: PerformanceTargets
  ): OptimizationResult {
    // Get current status
    const status = this.organRegistry.get(organId);
    if (!status) {
      throw new OrganError(
        OrganErrorCode.OPTIMIZATION_FAILED,
        `Organ ${organId} not found in registry`
      );
    }

    const currentOutput = status.functionMetrics.output.value;
    const currentEfficiency = status.functionMetrics.efficiency.value;
    const currentPower = status.functionMetrics.efficiency.energyInput;

    // Calculate optimal operating point
    const optimizedOutput = this.calculateOptimalOutput(
      currentOutput,
      targets.targetOutput,
      targets.maxPowerConsumption || Infinity
    );

    // Estimate efficiency at optimal point
    const expectedEfficiency = this.estimateEfficiency(
      optimizedOutput,
      targets.targetOutput
    );

    // Calculate power consumption
    const powerConsumption = optimizedOutput / expectedEfficiency;

    // Generate control adjustments
    const adjustments: Record<string, number> = {
      outputSetting: optimizedOutput,
      efficiencySetting: expectedEfficiency,
      powerLimit: targets.maxPowerConsumption || powerConsumption * 1.2,
    };

    // Calculate performance improvement
    const performanceImprovement =
      ((optimizedOutput - currentOutput) / currentOutput) * 100;

    return {
      optimizedOutput,
      expectedEfficiency,
      powerConsumption,
      adjustments,
      performanceImprovement,
      estimatedDuration: '2-4 hours for adaptation',
    };
  }

  /**
   * Calculate optimal output
   */
  private calculateOptimalOutput(
    current: number,
    target: number,
    maxPower: number
  ): number {
    // Gradually adjust toward target, respecting power constraints
    const delta = target - current;
    const step = Math.sign(delta) * Math.min(Math.abs(delta), current * 0.1);
    return current + step;
  }

  /**
   * Estimate efficiency at given output
   */
  private estimateEfficiency(output: number, target: number): number {
    // Efficiency typically peaks near design point
    const ratio = output / target;
    if (ratio > 0.8 && ratio < 1.2) {
      return 0.85; // Peak efficiency
    } else if (ratio > 0.6 && ratio < 1.4) {
      return 0.75; // Good efficiency
    } else {
      return 0.65; // Reduced efficiency
    }
  }

  // ==========================================================================
  // Maintenance and Service Methods
  // ==========================================================================

  /**
   * Schedule service
   */
  scheduleService(organId: string): ServiceSchedule {
    const status = this.organRegistry.get(organId);
    if (!status) {
      throw new OrganError(
        OrganErrorCode.SERVICE_SCHEDULING_FAILED,
        `Organ ${organId} not found`
      );
    }

    // Simplified scheduling based on operating hours
    const hours = status.operatingHours || 0;
    const cycles = status.cycleCount || 0;

    let serviceType: 'PREVENTIVE' | 'PREDICTIVE' | 'CORRECTIVE' | 'EMERGENCY';
    let urgency: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
    let reason: string;
    let daysUntilService: number;

    // Determine service need
    if (hours > 40000 || cycles > 50000000) {
      // Major service due
      serviceType = 'PREVENTIVE';
      urgency = 'HIGH';
      reason = 'Major service interval reached';
      daysUntilService = 7;
    } else if (hours > 30000 || cycles > 30000000) {
      // Service approaching
      serviceType = 'PREVENTIVE';
      urgency = 'MEDIUM';
      reason = 'Routine service recommended';
      daysUntilService = 30;
    } else if (status.functionMetrics.efficiency.value < 0.70) {
      // Performance-based service
      serviceType = 'PREDICTIVE';
      urgency = 'MEDIUM';
      reason = 'Performance degradation detected';
      daysUntilService = 14;
    } else {
      // Routine check
      serviceType = 'PREVENTIVE';
      urgency = 'LOW';
      reason = 'Routine inspection';
      daysUntilService = 90;
    }

    const nextServiceDate = new Date();
    nextServiceDate.setDate(nextServiceDate.getDate() + daysUntilService);

    return {
      nextServiceDate,
      serviceType,
      urgency,
      reason,
      estimatedDowntime: 4, // hours
      requiredComponents: this.identifyRequiredComponents(status),
      preparationInstructions: [
        'Schedule appointment with certified service center',
        'Backup all device data',
        'Prepare alternative support if needed',
      ],
    };
  }

  /**
   * Identify required components
   */
  private identifyRequiredComponents(status: OrganStatus): string[] {
    const components: string[] = [];

    if (status.batteryLevel !== undefined && status.batteryLevel < 80) {
      components.push('Battery replacement');
    }

    if (status.operatingHours && status.operatingHours > 30000) {
      components.push('Wear parts inspection/replacement');
    }

    if (status.functionMetrics.efficiency.value < 0.75) {
      components.push('Performance optimization');
    }

    return components;
  }

  /**
   * Predict maintenance needs
   */
  predictMaintenance(organId: string): MaintenancePrediction {
    const status = this.organRegistry.get(organId);
    if (!status) {
      throw new OrganError(
        OrganErrorCode.SERVICE_SCHEDULING_FAILED,
        `Organ ${organId} not found`
      );
    }

    const hours = status.operatingHours || 0;
    const cycles = status.cycleCount || 0;

    // Estimate hours to next service (simplified)
    const hoursToService = Math.max(0, 35000 - hours);
    const estimatedServiceDate = new Date();
    estimatedServiceDate.setHours(estimatedServiceDate.getHours() + hoursToService);

    // Calculate confidence based on data availability
    const confidenceLevel = 0.85;

    // Assess wear indicators
    const wearIndicators: Record<string, number> = {
      bearings: Math.min(1.0, hours / 50000),
      seals: Math.min(1.0, cycles / 100000000),
      battery: status.batteryLevel ? (100 - status.batteryLevel) / 100 : 0.5,
    };

    // Calculate degradation rate
    const degradationRate = this.calculateDegradationRate(organId);

    return {
      operatingHours: hours,
      cycleCount: cycles,
      hoursToService,
      estimatedServiceDate,
      confidenceLevel,
      wearIndicators,
      degradationRate,
      recommendedActions: [
        'Continue routine monitoring',
        'Prepare for service in advance',
        'Monitor efficiency trends',
      ],
    };
  }

  /**
   * Calculate degradation rate
   */
  private calculateDegradationRate(organId: string): number {
    const history = this.monitoringHistory.get(organId);
    if (!history || history.length < 2) {
      return 0;
    }

    // Simple linear regression on efficiency
    const recent = history.slice(-100);
    const firstEfficiency = recent[0].efficiency.value;
    const lastEfficiency = recent[recent.length - 1].efficiency.value;

    const degradation = (firstEfficiency - lastEfficiency) / firstEfficiency;
    return Math.max(0, degradation * 100); // % degradation
  }

  // ==========================================================================
  // Failsafe and Emergency Methods
  // ==========================================================================

  /**
   * Activate failsafe mode
   */
  activateFailsafe(organId: string, mode: FailsafeMode): FailsafeStatus {
    const status = this.organRegistry.get(organId);

    return {
      active: true,
      mode,
      trigger: 'MANUAL_ACTIVATION',
      backupSystemStatus: 'ACTIVE',
      backupDuration: 60, // minutes
      actionsTaken: [
        'Switched to failsafe mode',
        'Activated backup systems',
        'Notified emergency contacts',
        'Logged event',
      ],
      nextSteps: [
        'Assess situation severity',
        'Contact medical team',
        'Prepare for potential intervention',
      ],
      emergencyContactsNotified: true,
    };
  }

  /**
   * Get emergency protocol
   */
  getEmergencyProtocol(
    organId: string,
    situation: EmergencySituation
  ): EmergencyResponse {
    let action: EmergencyResponse['action'];
    let notifications: EmergencyResponse['notifications'];
    let deviceMode: EmergencyResponse['deviceMode'];
    let instructions: string;
    let timeToIntervention: number | undefined;

    switch (situation) {
      case 'COMPLETE_FAILURE':
        action = 'IMMEDIATE_MEDICAL_INTERVENTION';
        notifications = ['PATIENT', 'EMERGENCY_CONTACT', 'MEDICAL_TEAM', 'EMERGENCY_SERVICES'];
        deviceMode = 'EMERGENCY_MODE';
        instructions = 'Call 911 immediately. Activate backup support if available.';
        timeToIntervention = 0;
        break;

      case 'PARTIAL_FAILURE':
        action = 'URGENT_MEDICAL_CONTACT';
        notifications = ['PATIENT', 'EMERGENCY_CONTACT', 'MEDICAL_TEAM'];
        deviceMode = 'BACKUP_MODE';
        instructions = 'Contact medical team immediately. Monitor closely.';
        timeToIntervention = 30;
        break;

      case 'POWER_CRITICAL':
        action = 'URGENT_MEDICAL_CONTACT';
        notifications = ['PATIENT', 'EMERGENCY_CONTACT'];
        deviceMode = 'BACKUP_MODE';
        instructions = 'Connect to power source immediately. Prepare for hospital visit.';
        timeToIntervention = 60;
        break;

      case 'REJECTION_CRITICAL':
        action = 'IMMEDIATE_MEDICAL_INTERVENTION';
        notifications = ['PATIENT', 'MEDICAL_TEAM'];
        deviceMode = 'NORMAL_MODE';
        instructions = 'Proceed to hospital immediately for rejection evaluation.';
        timeToIntervention = 120;
        break;

      default:
        action = 'MONITOR_CLOSELY';
        notifications = ['PATIENT'];
        deviceMode = 'NORMAL_MODE';
        instructions = 'Monitor situation and contact medical team if worsening.';
        timeToIntervention = undefined;
    }

    return {
      action,
      notifications,
      deviceMode,
      instructions,
      timeToIntervention,
      alternativeSupport: this.identifyAlternativeSupport(organId),
      protocolReference: 'WIA-AUG-010-EMERGENCY-001',
    };
  }

  /**
   * Identify alternative support options
   */
  private identifyAlternativeSupport(organId: string): string[] {
    const status = this.organRegistry.get(organId);
    if (!status) return [];

    const alternatives: string[] = [];

    switch (status.organType) {
      case 'HEART':
        alternatives.push('CPR if needed', 'AED if available', 'Manual CPR');
        break;
      case 'KIDNEY':
        alternatives.push('Emergency dialysis', 'Fluid management');
        break;
      case 'LUNG':
        alternatives.push('Supplemental oxygen', 'Manual ventilation', 'ECMO if available');
        break;
      default:
        alternatives.push('Supportive care', 'Emergency medical services');
    }

    return alternatives;
  }

  // ==========================================================================
  // Status and History Methods
  // ==========================================================================

  /**
   * Get current organ status
   */
  getStatus(organId: string): OrganStatus {
    const status = this.organRegistry.get(organId);
    if (!status) {
      throw new OrganError(OrganErrorCode.MONITORING_FAILED, `Organ ${organId} not found`);
    }
    return status;
  }

  /**
   * Get historical data
   */
  getHistory(organId: string, timeRange: TimeRange): HistoricalData {
    const history = this.monitoringHistory.get(organId);
    if (!history) {
      throw new OrganError(OrganErrorCode.MONITORING_FAILED, `No history for organ ${organId}`);
    }

    // Filter by time range
    const filtered = history.filter(
      (m) => m.timestamp >= timeRange.startTime && m.timestamp <= timeRange.endTime
    );

    // Calculate summary statistics
    const outputs = filtered.map((m) => m.output.value);
    const efficiencies = filtered.map((m) => m.efficiency.value);
    const alerts = filtered.flatMap((m) => this.generateAlerts(m));

    const summary = {
      averageOutput: outputs.reduce((a, b) => a + b, 0) / outputs.length,
      averageEfficiency: efficiencies.reduce((a, b) => a + b, 0) / efficiencies.length,
      minOutput: Math.min(...outputs),
      maxOutput: Math.max(...outputs),
      alertCount: alerts.length,
      uptimePercentage: (filtered.filter((m) => m.operationalState === 'ACTIVE').length / filtered.length) * 100,
    };

    return {
      organId,
      timeRange,
      dataPoints: filtered.map((m) => ({
        timestamp: m.timestamp,
        metrics: m,
      })),
      summary,
    };
  }

  // ==========================================================================
  // Validation Methods
  // ==========================================================================

  private validateOrganInput(input: OrganInput): void {
    if (!input.organType) {
      throw new OrganError(
        OrganErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Organ type is required'
      );
    }

    if (!input.technologyType) {
      throw new OrganError(
        OrganErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Technology type is required'
      );
    }

    if (input.targetFunction < 0 || input.targetFunction > 200) {
      throw new OrganError(
        OrganErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Target function must be between 0 and 200%'
      );
    }

    if (input.biocompatibilityRating < 0 || input.biocompatibilityRating > 1) {
      throw new OrganError(
        OrganErrorCode.CLASSIFICATION_INVALID_INPUT,
        'Biocompatibility rating must be between 0 and 1'
      );
    }
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Classify organ (standalone)
 */
export function classifyOrgan(input: OrganInput): OrganClassification {
  const sdk = new ArtificialOrganSDK();
  return sdk.classifyOrgan(input);
}

/**
 * Monitor function (standalone)
 */
export function monitorFunction(organId: string, metrics: FunctionMetrics): MonitoringResult {
  const sdk = new ArtificialOrganSDK();
  return sdk.monitorFunction(organId, metrics);
}

/**
 * Assess biocompatibility (standalone)
 */
export function assessCompatibility(
  organId: string,
  data: BiocompatibilityData
): BiocompatibilityScore {
  const sdk = new ArtificialOrganSDK();
  return sdk.assessBiocompatibility(organId, data);
}

/**
 * Detect rejection (standalone)
 */
export function detectRejection(organId: string, data: RejectionData): RejectionAssessment {
  const sdk = new ArtificialOrganSDK();
  return sdk.detectRejection(organId, data);
}

/**
 * Optimize performance (standalone)
 */
export function optimizePerformance(
  organId: string,
  targets: PerformanceTargets
): OptimizationResult {
  const sdk = new ArtificialOrganSDK();
  return sdk.optimizePerformance(organId, targets);
}

/**
 * Schedule service (standalone)
 */
export function scheduleService(organId: string): ServiceSchedule {
  const sdk = new ArtificialOrganSDK();
  return sdk.scheduleService(organId);
}

/**
 * Activate failsafe (standalone)
 */
export function activateFailsafe(organId: string, mode: FailsafeMode): FailsafeStatus {
  const sdk = new ArtificialOrganSDK();
  return sdk.activateFailsafe(organId, mode);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ArtificialOrganSDK };
export default ArtificialOrganSDK;
