/**
 * WIA-DEF-018: Military AI SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense AI Working Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for ethical military AI systems including:
 * - Autonomous system management
 * - Human oversight mechanisms
 * - ISR analysis
 * - Predictive maintenance
 * - Ethical compliance validation
 */

import {
  AutonomousSystem,
  AutonomyConfig,
  AIModel,
  SafetyFeatures,
  HumanOversight,
  MissionConfig,
  SystemStatus,
  ISRAnalyzer,
  ISRAnalysis,
  EquipmentHealth,
  EthicalAssessment,
  MissionValidation,
  ValidationMetrics,
  AuditReport,
  ModelPrediction,
  Explanation,
  DefenseAIError,
  DefenseAIErrorCode,
  GeoCoordinate,
  GeoPolygon,
  SafetyCheck,
  Alert,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-DEF-018 Military AI SDK
 */
export class MilitaryAISDK {
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
   * Create an autonomous vehicle system
   *
   * @param config - System configuration
   * @returns Configured autonomous system
   */
  createAutonomousVehicle(config: {
    type: 'UAV' | 'UGV' | 'USV' | 'UUV';
    autonomyLevel: 2 | 3 | 4;
    killSwitch: boolean;
    geofence: GeoPolygon;
    maxAutonomyDuration?: number;
  }): AutonomousSystem {
    // Validate configuration
    this.validateAutonomyConfig(config);

    // Create safety features
    const safety: SafetyFeatures = {
      killSwitch: {
        enabled: true,
        activationMethods: ['remote', 'automatic'],
        activationTime: 0.5, // <1 second
        failureMode: 'return-to-base',
      },
      geofence: {
        enabled: true,
        boundary: config.geofence,
        violationAction: 'stop',
      },
      emergencyStop: true,
      failSafeDefaults: true,
      auditLogging: true,
      adversarialRobustness: true,
    };

    // Create human oversight
    const oversight: HumanOversight = {
      level: config.autonomyLevel === 2 ? 'operator' : 'supervisor',
      operatorRequired: true,
      approvalRequiredFor: ['mission-start', 'geofence-change'],
      monitoringInterface: {
        updateRate: 1,
        videoFeed: true,
        telemetry: true,
        aiDecisionsVisible: true,
      },
      interventionCapabilities: {
        emergencyStop: true,
        manualControlTakeover: true,
        parameterAdjustment: true,
        missionModification: true,
      },
    };

    // Create autonomy configuration
    const autonomy: AutonomyConfig = {
      level: config.autonomyLevel,
      maxDuration: config.maxAutonomyDuration || 3600,
      checkinInterval: 300,
      lethalForcePermitted: false, // Never for levels 2-4
      requiresHumanApproval: config.autonomyLevel === 2,
    };

    return {
      id: `${config.type}-${Date.now()}`,
      type: config.type,
      designation: `Autonomous ${config.type}`,
      autonomy,
      safety,
      oversight,
      aiSystems: [],
      sensors: [],
      status: this.createInitialStatus(),
    };
  }

  /**
   * Create an ISR analyzer
   *
   * @param config - Analyzer configuration
   * @returns ISR analyzer instance
   */
  createISRAnalyzer(config: {
    dataTypes: ('IMINT' | 'SIGINT' | 'HUMINT' | 'MASINT' | 'OSINT')[];
    confidenceThreshold?: number;
    humanReview?: boolean;
  }): ISRAnalyzer {
    return {
      dataTypes: config.dataTypes,
      confidenceThreshold: config.confidenceThreshold || 0.85,
      humanReviewRequired: config.humanReview !== false,
      multiIntFusion: true,
      privacyProtection: true,
    };
  }

  /**
   * Analyze ISR data
   *
   * @param analyzer - ISR analyzer configuration
   * @param data - Intelligence data
   * @returns Analysis result
   */
  async analyzeISRData(
    analyzer: ISRAnalyzer,
    data: {
      type: 'IMINT' | 'SIGINT' | 'HUMINT' | 'MASINT' | 'OSINT';
      payload: any;
      location?: GeoCoordinate;
      timestamp?: Date;
    }
  ): Promise<ISRAnalysis> {
    // Simulate analysis (in real implementation, call actual AI models)
    const detections = await this.performDetection(data.payload);
    const classifications = await this.performClassification(detections);

    // Calculate overall confidence
    const confidence = this.calculateOverallConfidence(
      detections,
      classifications
    );

    // Determine if human review required
    const requiresHumanReview =
      confidence < analyzer.confidenceThreshold ||
      analyzer.humanReviewRequired ||
      this.detectAmbiguity(detections, classifications);

    // Generate explanation
    const explanation = this.generateExplanation(
      detections,
      classifications,
      confidence
    );

    return {
      id: `ISR-${Date.now()}`,
      dataType: data.type,
      detections,
      classifications,
      confidence,
      requiresHumanReview,
      explanation,
      timestamp: data.timestamp || new Date(),
      location: data.location,
      source: {
        type: 'other',
        id: 'simulated',
        collectedAt: new Date(),
      },
    };
  }

  /**
   * Assess equipment health for predictive maintenance
   *
   * @param equipmentId - Equipment identifier
   * @param sensorData - Sensor readings
   * @returns Health assessment
   */
  assessEquipmentHealth(
    equipmentId: string,
    sensorData: {
      vibration?: number[];
      temperature?: number[];
      acoustics?: number[];
      performance?: Record<string, number>;
    }
  ): EquipmentHealth {
    // Detect anomalies
    const anomalies = this.detectAnomalies(sensorData);

    // Predict failures
    const predictedFailures = this.predictFailures(
      equipmentId,
      sensorData,
      anomalies
    );

    // Estimate remaining useful life
    const remainingUsefulLife = this.estimateRUL(
      sensorData,
      predictedFailures
    );

    // Calculate health score
    const healthScore = this.calculateHealthScore(
      anomalies,
      predictedFailures,
      remainingUsefulLife
    );

    // Generate recommendations
    const recommendations = this.generateMaintenanceRecommendations(
      healthScore,
      anomalies,
      predictedFailures,
      remainingUsefulLife
    );

    // Generate explanation
    const explanation = this.generateHealthExplanation(
      healthScore,
      anomalies,
      predictedFailures
    );

    return {
      equipmentId,
      healthScore,
      anomalies,
      predictedFailures,
      remainingUsefulLife,
      recommendations,
      timestamp: new Date(),
      explanation,
    };
  }

  /**
   * Validate mission for ethical and legal compliance
   *
   * @param mission - Mission configuration
   * @returns Validation result
   */
  validateMission(mission: MissionConfig): MissionValidation {
    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];
    const safetyChecks: SafetyCheck[] = [];

    // Ethical compliance check
    const ethicalCompliance = this.assessEthicalCompliance(mission);

    safetyChecks.push({
      name: 'Ethical Compliance',
      status: ethicalCompliance.isEthical ? 'pass' : 'fail',
      description: 'Verify ethical compliance with IHL and LOAC',
    });

    if (!ethicalCompliance.isEthical) {
      errors.push('Mission violates ethical guidelines');
      errors.push(...ethicalCompliance.violations);
    }

    if (ethicalCompliance.warnings.length > 0) {
      warnings.push(...ethicalCompliance.warnings);
    }

    // Civilian risk check
    safetyChecks.push({
      name: 'Civilian Risk Assessment',
      status: mission.civilianRisk === 'high' ? 'warning' : 'pass',
      description: 'Assess risk to civilian population',
    });

    if (mission.civilianRisk === 'high') {
      warnings.push('High civilian risk - extra precautions required');
      recommendations.push('Implement additional human oversight');
      recommendations.push('Reduce autonomy level if possible');
    }

    // Authorization check
    safetyChecks.push({
      name: 'Authorization',
      status: mission.approvedBy ? 'pass' : 'fail',
      description: 'Verify proper authorization',
    });

    if (!mission.approvedBy) {
      errors.push('Mission requires human authorization');
    }

    // Duration check
    safetyChecks.push({
      name: 'Duration Limits',
      status: mission.maxDuration <= 14400 ? 'pass' : 'warning',
      description: 'Check mission duration within limits',
    });

    if (mission.maxDuration > 14400) {
      warnings.push('Mission duration exceeds recommended 4-hour limit');
    }

    // Legal compliance
    const legalCompliance = {
      compliant: ethicalCompliance.ihlCompliance,
      issues: ethicalCompliance.ihlCompliance
        ? []
        : ['Does not comply with International Humanitarian Law'],
    };

    return {
      isValid: errors.length === 0,
      ethicalCompliance,
      safetyChecks,
      legalCompliance,
      errors,
      warnings,
      recommendations,
    };
  }

  /**
   * Assess ethical compliance
   */
  assessEthicalCompliance(mission: MissionConfig): EthicalAssessment {
    const violations: string[] = [];
    const warnings: string[] = [];

    // Check lethal force (not allowed in this standard for autonomous systems)
    if (mission.type === 'combat') {
      violations.push('Autonomous lethal force is prohibited');
    }

    // Check proportionality
    const isProportional = mission.civilianRisk !== 'high';
    if (!isProportional) {
      warnings.push('Mission may not meet proportionality requirements');
    }

    // Check distinction capability
    const canDistinguish = mission.civilianRisk !== 'high';

    // IHL compliance
    const ihlCompliance = violations.length === 0;

    // Determine recommendation
    let recommendation: EthicalAssessment['recommendation'];
    if (violations.length > 0) {
      recommendation = 'prohibited';
    } else if (warnings.length > 0) {
      recommendation = 'requires-review';
    } else if (mission.civilianRisk === 'medium') {
      recommendation = 'proceed-with-caution';
    } else {
      recommendation = 'proceed';
    }

    return {
      isEthical: violations.length === 0,
      ihlCompliance,
      proportionality: {
        militaryAdvantage: mission.objective,
        civilianHarm: `${mission.civilianRisk} risk`,
        isProportional,
      },
      distinction: {
        canDistinguish,
        confidence: canDistinguish ? 0.95 : 0.6,
      },
      violations,
      warnings,
      recommendation,
      requiredOversight: ['operator', 'supervisor'],
    };
  }

  /**
   * Perform compliance audit
   */
  performAudit(system: AutonomousSystem): AuditReport {
    const report: AuditReport = {
      id: `AUDIT-${Date.now()}`,
      systemId: system.id,
      timestamp: new Date(),
      auditor: 'WIA-DEF-018 SDK',
      safety: this.auditSafety(system),
      oversight: this.auditOversight(system),
      explainability: this.auditExplainability(system),
      security: this.auditSecurity(system),
      performance: this.auditPerformance(system),
      legal: this.auditLegalCompliance(system),
      compliant: false,
      recommendations: [],
      requiredActions: [],
    };

    // Determine overall compliance
    report.compliant =
      report.safety.passed &&
      report.oversight.passed &&
      report.explainability.passed &&
      report.security.passed &&
      report.performance.passed &&
      report.legal.passed;

    // Generate recommendations
    if (!report.compliant) {
      report.requiredActions.push('Address all audit findings before deployment');
    }

    return report;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private validateAutonomyConfig(config: any): void {
    if (!config.killSwitch && (config.autonomyLevel === 3 || config.autonomyLevel === 4)) {
      throw new DefenseAIError(
        DefenseAIErrorCode.INVALID_CONFIGURATION,
        'Kill switch required for autonomy levels 3-4'
      );
    }

    if (!config.geofence && (config.autonomyLevel === 3 || config.autonomyLevel === 4)) {
      throw new DefenseAIError(
        DefenseAIErrorCode.INVALID_CONFIGURATION,
        'Geofence required for autonomy levels 3-4'
      );
    }
  }

  private createInitialStatus(): SystemStatus {
    return {
      state: 'idle',
      powerLevel: 1.0,
      healthScore: 1.0,
      commLinkQuality: 1.0,
      alerts: [],
      lastUpdate: new Date(),
    };
  }

  private async performDetection(payload: any): Promise<any[]> {
    // Simulated detection - in real implementation, call actual AI model
    return [
      {
        type: 'vehicle',
        confidence: 0.92,
        boundingBox: { x: 100, y: 100, width: 50, height: 50 },
      },
    ];
  }

  private async performClassification(detections: any[]): Promise<any[]> {
    // Simulated classification
    return detections.map((d) => ({
      class: d.type,
      confidence: d.confidence,
    }));
  }

  private calculateOverallConfidence(
    detections: any[],
    classifications: any[]
  ): number {
    if (detections.length === 0) return 0;
    const avg =
      detections.reduce((sum, d) => sum + d.confidence, 0) / detections.length;
    return avg;
  }

  private detectAmbiguity(detections: any[], classifications: any[]): boolean {
    // Check for low confidence or conflicting classifications
    return detections.some((d) => d.confidence < 0.90);
  }

  private generateExplanation(
    detections: any[],
    classifications: any[],
    confidence: number
  ): Explanation {
    return {
      featureImportance: [
        { feature: 'shape', importance: 0.4 },
        { feature: 'texture', importance: 0.3 },
        { feature: 'context', importance: 0.3 },
      ],
      confidenceFactors: [
        'High-quality imagery',
        'Clear visibility',
        'Known object type',
      ],
    };
  }

  private detectAnomalies(sensorData: any): any[] {
    // Simulated anomaly detection
    const anomalies: any[] = [];

    if (sensorData.temperature && Math.max(...sensorData.temperature) > 80) {
      anomalies.push({
        type: 'high-temperature',
        severity: 'medium' as const,
        description: 'Temperature exceeds normal operating range',
        confidence: 0.9,
        detectedAt: new Date(),
        affectedComponents: ['engine'],
      });
    }

    return anomalies;
  }

  private predictFailures(
    equipmentId: string,
    sensorData: any,
    anomalies: any[]
  ): any[] {
    // Simulated failure prediction
    const predictions: any[] = [];

    if (anomalies.length > 0) {
      predictions.push({
        type: 'component-degradation',
        component: 'engine',
        probability: 0.3,
        timeToFailure: 168, // 1 week
        severity: 'medium' as const,
        impact: 'Reduced performance, potential mission failure',
      });
    }

    return predictions;
  }

  private estimateRUL(sensorData: any, predictions: any[]): number {
    // Simulated remaining useful life estimation
    if (predictions.length > 0) {
      return Math.min(...predictions.map((p: any) => p.timeToFailure));
    }
    return 1000; // Default: 1000 hours
  }

  private calculateHealthScore(
    anomalies: any[],
    predictions: any[],
    rul: number
  ): number {
    // Calculate health score (0-100)
    let score = 100;

    // Deduct for anomalies
    score -= anomalies.length * 10;

    // Deduct for predictions
    score -= predictions.filter((p: any) => p.severity === 'critical').length * 20;
    score -= predictions.filter((p: any) => p.severity === 'high').length * 15;
    score -= predictions.filter((p: any) => p.severity === 'medium').length * 10;

    // Deduct based on RUL
    if (rul < 24) score -= 30;
    else if (rul < 168) score -= 15;

    return Math.max(0, Math.min(100, score));
  }

  private generateMaintenanceRecommendations(
    healthScore: number,
    anomalies: any[],
    predictions: any[],
    rul: number
  ): any[] {
    const recommendations: any[] = [];

    if (healthScore < 40) {
      recommendations.push({
        priority: 'critical' as const,
        action: 'Immediate inspection and repair required',
        timeframe: 'Within 24 hours',
        reason: 'Critical failure imminent',
      });
    } else if (healthScore < 70 || rul < 168) {
      recommendations.push({
        priority: 'high' as const,
        action: 'Schedule maintenance within 1 week',
        timeframe: 'Within 7 days',
        reason: `Health score ${healthScore}, RUL ${rul} hours`,
      });
    } else if (healthScore < 85) {
      recommendations.push({
        priority: 'medium' as const,
        action: 'Increased monitoring recommended',
        timeframe: 'Ongoing',
        reason: 'Degraded performance detected',
      });
    } else {
      recommendations.push({
        priority: 'low' as const,
        action: 'Continue normal operations',
        timeframe: 'Next scheduled maintenance',
        reason: 'Equipment operating normally',
      });
    }

    return recommendations;
  }

  private generateHealthExplanation(
    healthScore: number,
    anomalies: any[],
    predictions: any[]
  ): Explanation {
    return {
      featureImportance: [
        { feature: 'anomalies', importance: 0.4 },
        { feature: 'predictions', importance: 0.4 },
        { feature: 'historical-data', importance: 0.2 },
      ],
      confidenceFactors: [
        `${anomalies.length} anomalies detected`,
        `${predictions.length} failures predicted`,
        `Health score: ${healthScore}`,
      ],
    };
  }

  private auditSafety(system: AutonomousSystem): { passed: boolean; findings: string[] } {
    const findings: string[] = [];

    if (!system.safety.killSwitch.enabled) {
      findings.push('Kill switch not enabled');
    }

    if (!system.safety.geofence.enabled) {
      findings.push('Geofence not enabled');
    }

    if (!system.safety.emergencyStop) {
      findings.push('Emergency stop not configured');
    }

    return {
      passed: findings.length === 0,
      findings,
    };
  }

  private auditOversight(system: AutonomousSystem): { passed: boolean; findings: string[] } {
    const findings: string[] = [];

    if (!system.oversight.operatorRequired && system.autonomy.level > 1) {
      findings.push('Human operator should be required for autonomy level > 1');
    }

    return {
      passed: findings.length === 0,
      findings,
    };
  }

  private auditExplainability(system: AutonomousSystem): { passed: boolean; findings: string[] } {
    const findings: string[] = [];

    const modelsWithoutExplainability = system.aiSystems.filter(
      (m) => !m.explainabilityEnabled
    );

    if (modelsWithoutExplainability.length > 0) {
      findings.push(`${modelsWithoutExplainability.length} models lack explainability`);
    }

    return {
      passed: findings.length === 0,
      findings,
    };
  }

  private auditSecurity(system: AutonomousSystem): { passed: boolean; findings: string[] } {
    const findings: string[] = [];

    if (!system.safety.adversarialRobustness) {
      findings.push('Adversarial robustness not validated');
    }

    return {
      passed: findings.length === 0,
      findings,
    };
  }

  private auditPerformance(system: AutonomousSystem): { passed: boolean; findings: string[] } {
    const findings: string[] = [];

    const lowPerformanceModels = system.aiSystems.filter(
      (m) => m.performance.accuracy < 0.95
    );

    if (lowPerformanceModels.length > 0) {
      findings.push(`${lowPerformanceModels.length} models below 95% accuracy`);
    }

    return {
      passed: findings.length === 0,
      findings,
    };
  }

  private auditLegalCompliance(system: AutonomousSystem): { passed: boolean; findings: string[] } {
    const findings: string[] = [];

    // Check if lethal force is permitted (should not be for autonomy > 1)
    if (system.autonomy.lethalForcePermitted && system.autonomy.level > 1) {
      findings.push('Lethal force not permitted for autonomy levels > 1');
    }

    return {
      passed: findings.length === 0,
      findings,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create autonomous vehicle (standalone function)
 */
export function createAutonomousVehicle(config: {
  type: 'UAV' | 'UGV' | 'USV' | 'UUV';
  autonomyLevel: 2 | 3 | 4;
  killSwitch: boolean;
  geofence: GeoPolygon;
  maxAutonomyDuration?: number;
}): AutonomousSystem {
  const sdk = new MilitaryAISDK();
  return sdk.createAutonomousVehicle(config);
}

/**
 * Validate mission (standalone function)
 */
export function validateMission(mission: MissionConfig): MissionValidation {
  const sdk = new MilitaryAISDK();
  return sdk.validateMission(mission);
}

/**
 * Assess ethical compliance (standalone function)
 */
export function assessEthicalCompliance(mission: MissionConfig): EthicalAssessment {
  const sdk = new MilitaryAISDK();
  return sdk.assessEthicalCompliance(mission);
}

/**
 * Perform audit (standalone function)
 */
export function performAudit(system: AutonomousSystem): AuditReport {
  const sdk = new MilitaryAISDK();
  return sdk.performAudit(system);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { MilitaryAISDK };
export default MilitaryAISDK;
