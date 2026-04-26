/**
 * WIA-AUG-013: Augmentation Safety SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Safety Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for augmentation safety assessment including:
 * - Device safety classification
 * - Risk assessment and FMEA
 * - Biocompatibility evaluation
 * - Long-term monitoring
 * - Emergency protocols
 */

import {
  SafetyLevel,
  ClassificationInput,
  ClassificationResult,
  RPNComponents,
  RiskLevel,
  FailureMode,
  RiskAssessment,
  BiocompatibilityTest,
  BioTestResult,
  MaterialClassification,
  BiocompatibilityResult,
  MonitoringData,
  Alert,
  DeviceInfo,
  PatientInfo,
  SafetyReport,
  SafeMode,
  SAFETY_CONSTANTS,
  SafetyErrorCode,
  SafetyError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-013 Augmentation Safety SDK
 */
export class AugmentationSafetySDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Classify a device's safety level
   */
  classifyDevice(input: ClassificationInput): ClassificationResult {
    // Validate inputs
    this.validateClassificationInput(input);

    const weights = SAFETY_CONSTANTS.CLASSIFICATION_WEIGHTS;

    // Calculate weighted score (normalized to 0-100)
    const breakdown = {
      invasiveness: input.invasiveness * weights.invasiveness * 10,
      vitalProximity: input.vitalProximity * weights.vitalProximity * 10,
      reversibility: input.reversibility * weights.reversibility * 10,
      failureConsequence: input.failureConsequence * weights.failureConsequence * 10,
      biologicalInteraction: input.biologicalInteraction * weights.biologicalInteraction * 10,
    };

    const score =
      breakdown.invasiveness +
      breakdown.vitalProximity +
      breakdown.reversibility +
      breakdown.failureConsequence +
      breakdown.biologicalInteraction;

    // Determine level
    const thresholds = SAFETY_CONSTANTS.CLASSIFICATION_THRESHOLDS;
    let level: SafetyLevel;
    if (score <= thresholds.LEVEL1_MAX) {
      level = 'Level1';
    } else if (score <= thresholds.LEVEL2_MAX) {
      level = 'Level2';
    } else if (score <= thresholds.LEVEL3_MAX) {
      level = 'Level3';
    } else if (score <= thresholds.LEVEL4_MAX) {
      level = 'Level4';
    } else {
      level = 'Level5';
    }

    // Generate requirements based on level
    const requirements = this.getRequirementsForLevel(level);

    return {
      level,
      score,
      breakdown,
      weights,
      requirements,
    };
  }

  /**
   * Calculate Risk Priority Number
   */
  calculateRPN(components: RPNComponents): { rpn: number; level: RiskLevel } {
    const { severity, occurrence, detection } = components;

    // Validate
    if (severity < 1 || severity > 10) {
      throw new SafetyError(SafetyErrorCode.CLASSIFICATION_INVALID_INPUT, 'Severity must be 1-10');
    }
    if (occurrence < 1 || occurrence > 10) {
      throw new SafetyError(SafetyErrorCode.CLASSIFICATION_INVALID_INPUT, 'Occurrence must be 1-10');
    }
    if (detection < 1 || detection > 10) {
      throw new SafetyError(SafetyErrorCode.CLASSIFICATION_INVALID_INPUT, 'Detection must be 1-10');
    }

    const rpn = severity * occurrence * detection;
    const thresholds = SAFETY_CONSTANTS.RPN_THRESHOLDS;

    let level: RiskLevel;
    if (rpn <= thresholds.LOW_MAX) {
      level = 'Low';
    } else if (rpn <= thresholds.MODERATE_MAX) {
      level = 'Moderate';
    } else if (rpn <= thresholds.HIGH_MAX) {
      level = 'High';
    } else if (rpn <= thresholds.CRITICAL_MAX) {
      level = 'Critical';
    } else {
      level = 'Unacceptable';
    }

    return { rpn, level };
  }

  /**
   * Perform risk assessment for a device
   */
  assessRisk(deviceId: string, failureModes: FailureMode[]): RiskAssessment {
    const assessedModes = failureModes.map((fm) => {
      const { rpn, level } = this.calculateRPN(fm.rpn);
      return { ...fm, rpnValue: rpn };
    });

    const totalRPN = assessedModes.reduce((sum, fm) => sum + fm.rpnValue, 0);
    const maxRPN = Math.max(...assessedModes.map((fm) => fm.rpnValue));
    const unacceptableCount = assessedModes.filter(
      (fm) => fm.rpnValue > SAFETY_CONSTANTS.RPN_THRESHOLDS.CRITICAL_MAX
    ).length;

    // Determine overall risk
    let overallRisk: RiskLevel;
    if (unacceptableCount > 0) {
      overallRisk = 'Unacceptable';
    } else if (maxRPN > SAFETY_CONSTANTS.RPN_THRESHOLDS.HIGH_MAX) {
      overallRisk = 'Critical';
    } else if (maxRPN > SAFETY_CONSTANTS.RPN_THRESHOLDS.MODERATE_MAX) {
      overallRisk = 'High';
    } else if (maxRPN > SAFETY_CONSTANTS.RPN_THRESHOLDS.LOW_MAX) {
      overallRisk = 'Moderate';
    } else {
      overallRisk = 'Low';
    }

    const passed = overallRisk !== 'Unacceptable' && overallRisk !== 'Critical';

    const recommendations = this.generateRiskRecommendations(overallRisk, assessedModes);

    return {
      id: `RA-${Date.now()}`,
      deviceId,
      date: new Date(),
      failureModes: assessedModes,
      overallRisk,
      totalRPN,
      maxRPN,
      unacceptableCount,
      passed,
      recommendations,
    };
  }

  /**
   * Evaluate biocompatibility requirements
   */
  evaluateBiocompatibility(
    deviceId: string,
    material: MaterialClassification,
    testResults: BioTestResult[]
  ): BiocompatibilityResult {
    const requiredTests = this.getRequiredBioTests(material);

    const passedTests = testResults.filter((t) => t.status === 'Pass');
    const failedTests = testResults.filter((t) => t.status === 'Fail');
    const pendingTests = requiredTests.filter(
      (rt) => !testResults.find((t) => t.test === rt)
    );

    const compliant =
      failedTests.length === 0 &&
      pendingTests.length === 0 &&
      requiredTests.every((rt) => passedTests.find((t) => t.test === rt));

    return {
      deviceId,
      material,
      requiredTests,
      testResults,
      compliant,
      pendingTests,
      failedTests: failedTests.map((t) => t.test),
    };
  }

  /**
   * Check monitoring data for alerts
   */
  checkMonitoringAlerts(data: MonitoringData): Alert[] {
    const alerts: Alert[] = [];
    const thresholds = SAFETY_CONSTANTS.ALERT_THRESHOLDS;

    // Power level alerts
    if (data.metrics.powerLevel < thresholds.POWER_CRITICAL) {
      alerts.push({
        id: `ALT-${Date.now()}-PWR-C`,
        type: 'power',
        severity: 'critical',
        message: `Critical: Battery level at ${data.metrics.powerLevel}%`,
        timestamp: new Date(),
        acknowledged: false,
      });
    } else if (data.metrics.powerLevel < thresholds.POWER_WARNING) {
      alerts.push({
        id: `ALT-${Date.now()}-PWR-W`,
        type: 'power',
        severity: 'warning',
        message: `Warning: Battery level at ${data.metrics.powerLevel}%`,
        timestamp: new Date(),
        acknowledged: false,
      });
    }

    // Signal quality alerts
    if (data.metrics.signalQuality < thresholds.SIGNAL_CRITICAL) {
      alerts.push({
        id: `ALT-${Date.now()}-SIG-C`,
        type: 'signal',
        severity: 'critical',
        message: `Critical: Signal quality at ${data.metrics.signalQuality}%`,
        timestamp: new Date(),
        acknowledged: false,
      });
    } else if (data.metrics.signalQuality < thresholds.SIGNAL_WARNING) {
      alerts.push({
        id: `ALT-${Date.now()}-SIG-W`,
        type: 'signal',
        severity: 'warning',
        message: `Warning: Signal quality at ${data.metrics.signalQuality}%`,
        timestamp: new Date(),
        acknowledged: false,
      });
    }

    // Temperature alerts
    const temp = data.metrics.operatingTemperature;
    if (temp > thresholds.TEMP_CRITICAL_HIGH || temp < thresholds.TEMP_CRITICAL_LOW) {
      alerts.push({
        id: `ALT-${Date.now()}-TMP-C`,
        type: 'temperature',
        severity: 'critical',
        message: `Critical: Temperature at ${temp}°C`,
        timestamp: new Date(),
        acknowledged: false,
      });
    } else if (temp > thresholds.TEMP_WARNING_HIGH || temp < thresholds.TEMP_WARNING_LOW) {
      alerts.push({
        id: `ALT-${Date.now()}-TMP-W`,
        type: 'temperature',
        severity: 'warning',
        message: `Warning: Temperature at ${temp}°C`,
        timestamp: new Date(),
        acknowledged: false,
      });
    }

    // Check biomarkers
    for (const biomarker of data.metrics.biomarkers) {
      if (biomarker.status === 'critical') {
        alerts.push({
          id: `ALT-${Date.now()}-BIO-C`,
          type: 'biomarker',
          severity: 'critical',
          message: `Critical: ${biomarker.name} at ${biomarker.value} ${biomarker.unit}`,
          timestamp: new Date(),
          acknowledged: false,
        });
      } else if (biomarker.status === 'warning') {
        alerts.push({
          id: `ALT-${Date.now()}-BIO-W`,
          type: 'biomarker',
          severity: 'warning',
          message: `Warning: ${biomarker.name} at ${biomarker.value} ${biomarker.unit}`,
          timestamp: new Date(),
          acknowledged: false,
        });
      }
    }

    return alerts;
  }

  /**
   * Generate default safe mode configuration
   */
  getDefaultSafeMode(): SafeMode {
    return {
      trigger: 'automatic',
      actions: ['cease_active_functions', 'maintain_vital_support', 'enable_beacon'],
      powerState: 'minimal',
      communication: 'beacon',
      reversible: true,
      timeout: 3600,
    };
  }

  /**
   * Generate comprehensive safety report
   */
  generateSafetyReport(
    device: DeviceInfo,
    classification: ClassificationResult,
    riskAssessment: RiskAssessment,
    biocompatibility: BiocompatibilityResult
  ): SafetyReport {
    const compliant =
      riskAssessment.passed &&
      biocompatibility.compliant &&
      classification.level !== 'Level5';

    return {
      id: `SR-${Date.now()}`,
      generatedAt: new Date(),
      device,
      classification,
      riskAssessment,
      biocompatibility,
      clinicalTrials: [],
      monitoringSummary: {
        duration: 0,
        dataPoints: 0,
        averageMetrics: {
          powerLevel: 100,
          signalQuality: 100,
          operatingTemperature: 37,
          impedance: [],
          functionality: 100,
          biomarkers: [],
        },
        alertCount: 0,
        criticalAlerts: 0,
      },
      compliant,
      certificationStatus: compliant ? 'pending' : 'rejected',
      recommendations: [
        ...classification.requirements,
        ...riskAssessment.recommendations,
      ],
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private validateClassificationInput(input: ClassificationInput): void {
    const fields = [
      'invasiveness',
      'vitalProximity',
      'reversibility',
      'failureConsequence',
      'biologicalInteraction',
    ];

    for (const field of fields) {
      const value = input[field as keyof ClassificationInput];
      if (value < 1 || value > 10) {
        throw new SafetyError(
          SafetyErrorCode.CLASSIFICATION_INVALID_INPUT,
          `${field} must be between 1 and 10`
        );
      }
    }
  }

  private getRequirementsForLevel(level: SafetyLevel): string[] {
    const base = ['Basic documentation', 'Quality management system'];

    switch (level) {
      case 'Level1':
        return [...base, 'Self-declaration'];
      case 'Level2':
        return [...base, 'Biocompatibility testing (ISO 10993-5, 10)', 'EMC testing'];
      case 'Level3':
        return [
          ...base,
          'Full biocompatibility testing',
          'Pre-clinical studies',
          'Clinical trial (Phase I/II)',
        ];
      case 'Level4':
        return [
          ...base,
          'Comprehensive biocompatibility',
          'Extended animal studies',
          'Clinical trials (Phase I/II/III)',
          'Long-term follow-up',
        ];
      case 'Level5':
        return [
          ...base,
          'Full ISO 10993 compliance',
          'Extensive pre-clinical studies',
          'Multi-phase clinical trials',
          'Lifetime monitoring',
          'Ethics board approval',
          'Regulatory pre-approval',
        ];
      default:
        return base;
    }
  }

  private getRequiredBioTests(material: MaterialClassification): BiocompatibilityTest[] {
    const tests: BiocompatibilityTest[] = ['Cytotoxicity'];

    if (material.contactType !== 'Surface') {
      tests.push('Sensitization', 'Irritation');
    }

    if (material.contactType === 'Implant') {
      tests.push('SystemicToxicity', 'Genotoxicity', 'Implantation');
    }

    if (material.tissueContact === 'Blood' || material.tissueContact === 'Neural') {
      tests.push('Carcinogenicity');
    }

    if (material.contactDuration === 'Permanent' && material.contactType === 'Implant') {
      tests.push('ReproductiveToxicity');
    }

    return tests;
  }

  private generateRiskRecommendations(
    overallRisk: RiskLevel,
    failureModes: FailureMode[]
  ): string[] {
    const recommendations: string[] = [];

    if (overallRisk === 'Unacceptable') {
      recommendations.push('STOP: Do not proceed without major redesign');
      recommendations.push('Address all failure modes with RPN > 500 immediately');
    } else if (overallRisk === 'Critical') {
      recommendations.push('Major risk mitigation required before proceeding');
      recommendations.push('Focus on failure modes with highest RPN values');
    } else if (overallRisk === 'High') {
      recommendations.push('Implement risk mitigation measures');
      recommendations.push('Conduct additional testing');
    } else if (overallRisk === 'Moderate') {
      recommendations.push('Continue with standard risk monitoring');
      recommendations.push('Document all mitigation efforts');
    } else {
      recommendations.push('Maintain current safety protocols');
      recommendations.push('Regular monitoring recommended');
    }

    // Add specific recommendations for high-RPN failure modes
    const highRPN = failureModes.filter(
      (fm) => fm.rpnValue > SAFETY_CONSTANTS.RPN_THRESHOLDS.MODERATE_MAX
    );
    for (const fm of highRPN.slice(0, 3)) {
      recommendations.push(`Address ${fm.component}: ${fm.failureType} (RPN: ${fm.rpnValue})`);
    }

    return recommendations;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Classify device safety level (standalone)
 */
export function classifyDevice(input: ClassificationInput): ClassificationResult {
  const sdk = new AugmentationSafetySDK();
  return sdk.classifyDevice(input);
}

/**
 * Calculate RPN (standalone)
 */
export function calculateRPN(components: RPNComponents): { rpn: number; level: RiskLevel } {
  const sdk = new AugmentationSafetySDK();
  return sdk.calculateRPN(components);
}

/**
 * Assess risk (standalone)
 */
export function assessRisk(deviceId: string, failureModes: FailureMode[]): RiskAssessment {
  const sdk = new AugmentationSafetySDK();
  return sdk.assessRisk(deviceId, failureModes);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { AugmentationSafetySDK };
export default AugmentationSafetySDK;
