/**
 * WIA-AUG-002: Cybernetic Implant SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Human Augmentation Cybernetics Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for cybernetic implant management including:
 * - Implant classification and categorization
 * - Biocompatibility assessment
 * - Power management and monitoring
 * - Communication protocol handling
 * - Rejection monitoring and detection
 * - Firmware update management
 * - End-of-life and explantation procedures
 */

import {
  ImplantType,
  BiocompatibilityClass,
  PowerSource,
  IntegrationLevel,
  ImplantClassificationInput,
  ImplantClassification,
  BioAssessmentParams,
  BiocompatibilityResult,
  BiocompatibilityTest,
  PowerConfig,
  PowerMetrics,
  PowerAlert,
  ConnectionConfig,
  Connection,
  CommProtocol,
  ImplantData,
  TransmissionResult,
  RejectionMonitoringData,
  RejectionStatus,
  UpdateInfo,
  UpdateResult,
  RollbackResult,
  ExplantParams,
  ExplantSchedule,
  DeactivationResult,
  ImplantAssessmentInput,
  ImplantAssessment,
  VitalMetrics,
  IMPLANT_CONSTANTS,
  ImplantErrorCode,
  ImplantError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-AUG-002 Cybernetic Implant SDK
 */
export class CyberneticImplantSDK {
  private version = '1.0.0';

  constructor() {}

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Classify an implant based on its characteristics
   */
  classifyImplant(input: ImplantClassificationInput): ImplantClassification {
    // Determine implant type
    const type = this.determineImplantType(input);

    // Determine biocompatibility class
    const bioClass = this.determineBioClass(input.integrationLevel);

    // Determine risk level
    const riskLevel = this.determineRiskLevel(type, input.integrationLevel);

    // Get requirements
    const requirements = this.getRequirements(type, bioClass, input.integrationLevel);

    // Get monitoring frequency
    const monitoringFrequency = this.getMonitoringFrequency(input.integrationLevel);

    return {
      type,
      integrationLevel: input.integrationLevel,
      bioClass,
      riskLevel,
      requirements,
      monitoringFrequency,
    };
  }

  /**
   * Assess biocompatibility of an implant
   */
  assessBiocompatibility(params: BioAssessmentParams): BiocompatibilityResult {
    // Determine biocompatibility class
    const bioClass = this.getBioClassFromDuration(params.contactDuration, params.tissueType);

    // Get required tests
    const requiredTests = this.getRequiredBioTests(bioClass, params.tissueType);

    // Process existing test results
    const testResults = params.testResults || [];
    const passedTests = testResults.filter((t) => t.result === 'PASS');
    const failedTests = testResults.filter((t) => t.result === 'FAIL');
    const pendingTests = requiredTests.filter(
      (rt) => !testResults.find((t) => t.test === rt)
    );

    // Determine compliance
    const compliant =
      failedTests.length === 0 &&
      pendingTests.length === 0 &&
      requiredTests.every((rt) => passedTests.find((t) => t.test === rt));

    // Determine assessment
    let assessment: 'approved' | 'conditional' | 'rejected';
    if (compliant) {
      assessment = 'approved';
    } else if (failedTests.length > 0) {
      assessment = 'rejected';
    } else {
      assessment = 'conditional';
    }

    // Generate recommendations
    const recommendations = this.generateBioRecommendations(
      compliant,
      pendingTests,
      failedTests.map((t) => t.test)
    );

    return {
      implantId: params.implantId,
      class: bioClass,
      requiredTests,
      testResults,
      compliant,
      pendingTests,
      failedTests: failedTests.map((t) => t.test),
      assessment,
      recommendations,
    };
  }

  /**
   * Configure power management for an implant
   */
  managePower(config: PowerConfig): PowerMetrics {
    // Simulate power metrics based on configuration
    const estimatedRuntime = this.calculateRuntime(
      config.primarySource,
      config.chargingSchedule.frequency
    );

    return {
      batteryLevel: 100,
      chargeCycles: 0,
      powerConsumption: this.estimatePowerConsumption(config.primarySource),
      estimatedRuntime,
      chargingStatus: 'full',
      activeSource: config.primarySource,
      sourceAvailability: {
        battery: config.primarySource === 'BATTERY' || config.primarySource === 'HYBRID',
        wireless: config.primarySource === 'WIRELESS' || config.primarySource === 'HYBRID',
        bioHarvest: config.primarySource === 'BIO_HARVEST' || config.primarySource === 'HYBRID',
      },
    };
  }

  /**
   * Monitor for rejection signs
   */
  monitorRejection(data: RejectionMonitoringData): RejectionStatus {
    // Calculate individual risk scores
    const biomarkerRisk = this.assessBiomarkerRisk(data.biomarkers);
    const clinicalRisk = this.assessClinicalRisk(data.clinicalSigns);
    const deviceRisk = this.assessDeviceRisk(data.deviceMetrics);

    // Calculate overall risk score (0-100)
    const riskScore = (biomarkerRisk + clinicalRisk + deviceRisk) / 3;

    // Determine risk level
    let riskLevel: 'green' | 'yellow' | 'orange' | 'red';
    if (riskScore < 25) riskLevel = 'green';
    else if (riskScore < 50) riskLevel = 'yellow';
    else if (riskScore < 75) riskLevel = 'orange';
    else riskLevel = 'red';

    // Generate alerts
    const alerts = this.generateRejectionAlerts(data);

    // Determine urgency
    const urgency = this.determineUrgency(riskLevel, alerts.length);

    // Generate recommendations
    const recommendations = this.generateRejectionRecommendations(riskLevel, alerts);

    // Calculate next monitoring date
    const nextMonitoring = this.calculateNextMonitoring(riskLevel);

    return {
      riskLevel,
      riskScore,
      alerts,
      clinicalAssessmentNeeded: riskLevel === 'orange' || riskLevel === 'red',
      urgency,
      recommendations,
      nextMonitoring,
    };
  }

  /**
   * Check for firmware updates
   */
  checkUpdates(): UpdateInfo {
    // This would typically query a server
    // Returning mock data for demonstration
    return {
      id: `UPD-${Date.now()}`,
      version: '2.1.0',
      type: 'optimization',
      urgency: 'recommended',
      mandatory: false,
      releaseDate: new Date(),
      description: 'Performance optimization and bug fixes',
      size: 524288, // 512 KB
      releaseNotes: 'Improved power efficiency and signal processing',
      compatibility: {
        minVersion: '2.0.0',
        maxVersion: '2.0.9',
        hardwareRevision: ['Rev-A', 'Rev-B'],
      },
    };
  }

  /**
   * Apply a firmware update
   */
  updateFirmware(updateId: string): UpdateResult {
    // Simulate update process
    const success = true;
    const previousVersion = '2.0.5';
    const newVersion = '2.1.0';
    const duration = 120; // seconds

    return {
      success,
      previousVersion,
      newVersion,
      duration,
      verified: success,
      error: success ? undefined : 'Update verification failed',
      rollbackAvailable: true,
    };
  }

  /**
   * Rollback firmware to previous version
   */
  rollback(): RollbackResult {
    // Simulate rollback process
    const success = true;

    return {
      success,
      currentVersion: '2.1.0',
      rolledBackToVersion: '2.0.5',
      duration: 60, // seconds
      error: success ? undefined : 'Rollback failed',
    };
  }

  /**
   * Schedule explantation
   */
  scheduleExplant(params: ExplantParams): ExplantSchedule {
    // Validate parameters
    if (!params.patientConsent) {
      throw new ImplantError(
        ImplantErrorCode.SAFETY_THRESHOLD_EXCEEDED,
        'Patient consent required for explantation'
      );
    }

    // Determine urgency-based scheduling
    const scheduledDate = params.scheduledDate || this.calculateExplantDate(params.urgency);

    // Get estimated duration
    const estimatedDuration = this.estimateExplantDuration(params.category);

    // Generate pre-op requirements
    const preOpRequirements = this.getPreOpRequirements(params.category);

    return {
      id: `EXP-${Date.now()}`,
      implantId: params.implantId,
      scheduledDate,
      category: params.category,
      status: 'scheduled',
      preOpRequirements,
      estimatedDuration,
    };
  }

  /**
   * Deactivate an implant
   */
  deactivateImplant(implantId: string): DeactivationResult {
    // Simulate deactivation process
    const success = true;

    return {
      success,
      implantId,
      timestamp: new Date(),
      dataExtracted: true,
      safeModeActivated: true,
      powerStatus: 'safe_mode',
      error: success ? undefined : 'Deactivation failed',
    };
  }

  /**
   * Perform comprehensive implant assessment
   */
  assessImplant(input: ImplantAssessmentInput): ImplantAssessment {
    // Classify the implant
    const classificationInput: ImplantClassificationInput = {
      hasPowerSource: input.device.powerSource !== 'BATTERY',
      hasProcessing: input.device.type !== 'PASSIVE',
      hasNeuralConnection: input.device.type === 'NEURAL_INTERFACE',
      adaptiveBehavior: input.device.type === 'SMART' || input.device.type === 'NEURAL_INTERFACE',
      aiCapability: input.device.type === 'SMART' || input.device.type === 'NEURAL_INTERFACE',
      location: input.location.anatomicalSite,
      integrationLevel: input.location.integrationLevel,
    };

    const classification = this.classifyImplant(classificationInput);

    // Assess biocompatibility
    const biocompatibility = {
      class: classification.bioClass,
      requiredTests: this.getRequiredBioTests(
        classification.bioClass,
        input.location.tissueType
      ),
      assessment: `${classification.bioClass} biocompatibility required`,
    };

    // Create power plan
    const powerPlan = {
      strategy: this.determinePowerStrategy(input.device.powerSource),
      estimatedLifetime: this.estimateDeviceLifetime(input.device.powerSource),
      chargingFrequency: this.determineChargingFrequency(input.device.powerSource),
    };

    // Create monitoring plan
    const monitoringPlan = {
      frequency: classification.monitoringFrequency,
      duration: this.determineMonitoringDuration(classification.integrationLevel),
      biomarkers: ['CRP', 'IL-6', 'Temperature', 'Antibodies', 'Impedance'],
    };

    // Calculate overall risk
    const overallRisk = this.calculateOverallRisk(
      classification,
      input.patient.immuneStatus,
      input.location.integrationLevel
    );

    // Determine approval status
    const approvalStatus =
      overallRisk.level === 'critical'
        ? 'rejected'
        : overallRisk.level === 'high'
        ? 'conditional'
        : 'approved';

    // Generate recommendations
    const recommendations = [
      ...classification.requirements,
      ...overallRisk.recommendations,
    ];

    return {
      classification,
      biocompatibility,
      powerPlan,
      monitoringPlan,
      overallRisk,
      approvalStatus,
      recommendations,
    };
  }

  /**
   * Get current vital metrics
   */
  getVitals(implantId: string): VitalMetrics {
    // This would typically query the device
    // Returning mock data for demonstration
    return {
      timestamp: new Date(),
      implantId,
      functionality: 98,
      powerLevel: 85,
      signalQuality: 92,
      temperature: 36.8,
      communicationStatus: 'active',
      alerts: [],
      status: 'normal',
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private determineImplantType(input: ImplantClassificationInput): ImplantType {
    if (!input.hasPowerSource) return 'PASSIVE';
    if (!input.hasProcessing) return 'ACTIVE';
    if (!input.hasNeuralConnection && !input.aiCapability) return 'ACTIVE';
    if (input.hasNeuralConnection) return 'NEURAL_INTERFACE';
    if (input.adaptiveBehavior || input.aiCapability) return 'SMART';
    return 'ACTIVE';
  }

  private determineBioClass(level: IntegrationLevel): BiocompatibilityClass {
    if (level <= 1) return 'CLASS_I';
    if (level <= 2) return 'CLASS_II';
    return 'CLASS_III';
  }

  private determineRiskLevel(
    type: ImplantType,
    level: IntegrationLevel
  ): 'low' | 'moderate' | 'high' | 'critical' {
    const score = this.calculateRiskScore(type, level);
    if (score < 25) return 'low';
    if (score < 50) return 'moderate';
    if (score < 75) return 'high';
    return 'critical';
  }

  private calculateRiskScore(type: ImplantType, level: IntegrationLevel): number {
    const typeScores = {
      PASSIVE: 10,
      ACTIVE: 30,
      SMART: 50,
      NEURAL_INTERFACE: 70,
    };

    const levelScore = level * 15;
    const typeScore = typeScores[type];

    return (typeScore + levelScore) / 2;
  }

  private getRequirements(
    type: ImplantType,
    bioClass: BiocompatibilityClass,
    level: IntegrationLevel
  ): string[] {
    const requirements = ['Device Master File', 'Risk Management File'];

    if (bioClass === 'CLASS_II' || bioClass === 'CLASS_III') {
      requirements.push('ISO 10993 biocompatibility testing');
    }

    if (level >= 3) {
      requirements.push('Pre-clinical animal studies', 'Clinical trials');
    }

    if (level >= 4) {
      requirements.push('Long-term monitoring plan', 'Neurological assessment');
    }

    if (type === 'NEURAL_INTERFACE') {
      requirements.push('Neural interface validation', 'EMC compliance', 'Cybersecurity assessment');
    }

    return requirements;
  }

  private getMonitoringFrequency(
    level: IntegrationLevel
  ): 'continuous' | 'daily' | 'weekly' | 'biweekly' | 'monthly' {
    if (level >= 5) return 'continuous';
    if (level >= 4) return 'daily';
    if (level >= 3) return 'weekly';
    if (level >= 2) return 'biweekly';
    return 'monthly';
  }

  private getBioClassFromDuration(
    duration: string,
    tissueType: string
  ): BiocompatibilityClass {
    if (duration === 'permanent' || tissueType === 'neural' || tissueType === 'bone') {
      return 'CLASS_III';
    }
    if (duration === 'prolonged') return 'CLASS_II';
    return 'CLASS_I';
  }

  private getRequiredBioTests(
    bioClass: BiocompatibilityClass,
    tissueType: string
  ): BiocompatibilityTest[] {
    const tests: BiocompatibilityTest[] = ['cytotoxicity'];

    if (bioClass === 'CLASS_II' || bioClass === 'CLASS_III') {
      tests.push('sensitization', 'irritation', 'systemic_toxicity');
    }

    if (bioClass === 'CLASS_III') {
      tests.push('genotoxicity', 'implantation');

      if (tissueType === 'blood' || tissueType === 'neural') {
        tests.push('hemocompatibility', 'carcinogenicity');
      }

      if (tissueType === 'neural') {
        tests.push('reproductive_toxicity');
      }
    }

    return tests;
  }

  private generateBioRecommendations(
    compliant: boolean,
    pendingTests: BiocompatibilityTest[],
    failedTests: BiocompatibilityTest[]
  ): string[] {
    const recommendations: string[] = [];

    if (compliant) {
      recommendations.push('All biocompatibility requirements met');
      recommendations.push('Proceed with clinical evaluation');
    } else if (failedTests.length > 0) {
      recommendations.push('CRITICAL: Biocompatibility tests failed');
      recommendations.push(`Failed tests: ${failedTests.join(', ')}`);
      recommendations.push('Material reformulation required');
    } else if (pendingTests.length > 0) {
      recommendations.push('Complete pending biocompatibility tests');
      recommendations.push(`Pending: ${pendingTests.join(', ')}`);
    }

    return recommendations;
  }

  private calculateRuntime(source: PowerSource, frequency: string): number {
    const baseHours = {
      BATTERY: 168, // 1 week
      WIRELESS: 999999, // continuous
      BIO_HARVEST: 720, // 30 days
      HYBRID: 336, // 2 weeks
    };

    return baseHours[source];
  }

  private estimatePowerConsumption(source: PowerSource): number {
    const consumption = {
      BATTERY: 50, // mW
      WIRELESS: 30,
      BIO_HARVEST: 10,
      HYBRID: 40,
    };

    return consumption[source];
  }

  private assessBiomarkerRisk(biomarkers: RejectionMonitoringData['biomarkers']): number {
    let risk = 0;

    // CRP assessment
    if (biomarkers.crp > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.CRP.critical) risk += 30;
    else if (biomarkers.crp > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.CRP.warning) risk += 15;

    // IL-6 assessment
    if (biomarkers.il6 > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.IL6.critical) risk += 25;
    else if (biomarkers.il6 > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.IL6.warning) risk += 12;

    // Temperature assessment
    if (biomarkers.temperature > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.TEMPERATURE.critical)
      risk += 20;
    else if (biomarkers.temperature > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.TEMPERATURE.warning)
      risk += 10;

    // Antibodies assessment
    if (biomarkers.antibodies > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.ANTIBODIES.critical)
      risk += 25;
    else if (biomarkers.antibodies > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.ANTIBODIES.warning)
      risk += 12;

    return Math.min(risk, 100);
  }

  private assessClinicalRisk(signs: RejectionMonitoringData['clinicalSigns']): number {
    let risk = 0;

    if (signs.pain >= 7) risk += 30;
    else if (signs.pain >= 4) risk += 15;

    if (signs.swelling) risk += 15;
    if (signs.redness) risk += 10;
    if (signs.warmth) risk += 10;
    if (signs.discharge) risk += 25;

    return Math.min(risk, 100);
  }

  private assessDeviceRisk(metrics: RejectionMonitoringData['deviceMetrics']): number {
    let risk = 0;

    if (metrics.functionality < 70) risk += 30;
    else if (metrics.functionality < 85) risk += 15;

    if (metrics.communicationQuality < 50) risk += 25;
    else if (metrics.communicationQuality < 70) risk += 12;

    if (metrics.impedance && metrics.impedance > 1000) risk += 20;
    if (metrics.signalQuality && metrics.signalQuality < 5) risk += 25;

    return Math.min(risk, 100);
  }

  private generateRejectionAlerts(data: RejectionMonitoringData): string[] {
    const alerts: string[] = [];

    if (data.biomarkers.crp > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.CRP.warning) {
      alerts.push(`Elevated CRP: ${data.biomarkers.crp} mg/L`);
    }

    if (data.biomarkers.il6 > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.IL6.warning) {
      alerts.push(`Elevated IL-6: ${data.biomarkers.il6} pg/mL`);
    }

    if (data.biomarkers.temperature > IMPLANT_CONSTANTS.BIOMARKER_THRESHOLDS.TEMPERATURE.warning) {
      alerts.push(`Elevated temperature: ${data.biomarkers.temperature}°C`);
    }

    if (data.clinicalSigns.pain >= 4) {
      alerts.push(`Significant pain reported: ${data.clinicalSigns.pain}/10`);
    }

    if (data.clinicalSigns.discharge) {
      alerts.push('Discharge present - infection risk');
    }

    if (data.deviceMetrics.functionality < 85) {
      alerts.push(`Reduced functionality: ${data.deviceMetrics.functionality}%`);
    }

    return alerts;
  }

  private determineUrgency(
    riskLevel: string,
    alertCount: number
  ): 'routine' | 'soon' | 'urgent' | 'emergency' {
    if (riskLevel === 'red') return 'emergency';
    if (riskLevel === 'orange') return 'urgent';
    if (riskLevel === 'yellow' && alertCount > 2) return 'soon';
    return 'routine';
  }

  private generateRejectionRecommendations(riskLevel: string, alerts: string[]): string[] {
    const recommendations: string[] = [];

    switch (riskLevel) {
      case 'red':
        recommendations.push('EMERGENCY: Immediate medical intervention required');
        recommendations.push('Consider explantation');
        recommendations.push('Daily clinical monitoring');
        break;
      case 'orange':
        recommendations.push('Urgent clinical assessment required');
        recommendations.push('Increase monitoring frequency to daily');
        recommendations.push('Consider anti-rejection therapy');
        break;
      case 'yellow':
        recommendations.push('Clinical review within 1 week');
        recommendations.push('Increase monitoring to bi-weekly');
        recommendations.push('Review medication and lifestyle factors');
        break;
      case 'green':
        recommendations.push('Continue routine monitoring');
        recommendations.push('No immediate intervention needed');
        break;
    }

    if (alerts.length > 0) {
      recommendations.push(`Address ${alerts.length} active alert(s)`);
    }

    return recommendations;
  }

  private calculateNextMonitoring(riskLevel: string): Date {
    const now = new Date();
    const daysToAdd = {
      green: 30,
      yellow: 14,
      orange: 1,
      red: 0,
    }[riskLevel];

    return new Date(now.getTime() + daysToAdd * 24 * 60 * 60 * 1000);
  }

  private calculateExplantDate(urgency: string): Date {
    const now = new Date();
    const daysToAdd = {
      emergency: 0,
      urgent: 3,
      elective: 30,
      scheduled: 90,
    }[urgency] || 90;

    return new Date(now.getTime() + daysToAdd * 24 * 60 * 60 * 1000);
  }

  private estimateExplantDuration(category: string): number {
    // Duration in minutes
    const durations = {
      device_failure: 90,
      medical_necessity: 120,
      upgrade: 60,
      end_of_service: 75,
    };

    return durations[category as keyof typeof durations] || 90;
  }

  private getPreOpRequirements(category: string): string[] {
    const base = [
      'Pre-operative physical examination',
      'Blood work (CBC, metabolic panel)',
      'Imaging studies',
      'Patient consent',
    ];

    if (category === 'medical_necessity') {
      base.push('Infection screening', 'Immunological assessment');
    }

    return base;
  }

  private determinePowerStrategy(
    source: PowerSource
  ): 'battery' | 'wireless' | 'hybrid' {
    if (source === 'HYBRID') return 'hybrid';
    if (source === 'WIRELESS') return 'wireless';
    return 'battery';
  }

  private estimateDeviceLifetime(source: PowerSource): number {
    // Years
    const lifetimes = {
      BATTERY: 7,
      WIRELESS: 15,
      BIO_HARVEST: 10,
      HYBRID: 12,
    };

    return lifetimes[source];
  }

  private determineChargingFrequency(source: PowerSource): string {
    const frequencies = {
      BATTERY: 'Daily or weekly',
      WIRELESS: 'Continuous',
      BIO_HARVEST: 'Not required',
      HYBRID: 'As needed',
    };

    return frequencies[source];
  }

  private determineMonitoringDuration(level: IntegrationLevel): number {
    // Months
    const durations = {
      1: 6,
      2: 12,
      3: 24,
      4: 60,
      5: 120,
    };

    return durations[level];
  }

  private calculateOverallRisk(
    classification: ImplantClassification,
    immuneStatus: string,
    level: IntegrationLevel
  ): {
    score: number;
    level: 'low' | 'moderate' | 'high' | 'critical';
    factors: { surgical: number; biological: number; technical: number; longTerm: number };
    recommendations: string[];
  } {
    // Calculate risk factors
    const surgical = level * 15;
    const biological = immuneStatus === 'compromised' ? 30 : immuneStatus === 'enhanced' ? 10 : 20;
    const technical =
      classification.type === 'NEURAL_INTERFACE'
        ? 30
        : classification.type === 'SMART'
        ? 20
        : 10;
    const longTerm = level >= 4 ? 25 : level >= 3 ? 15 : 10;

    const score = (surgical + biological + technical + longTerm) / 4;

    let riskLevel: 'low' | 'moderate' | 'high' | 'critical';
    if (score < 25) riskLevel = 'low';
    else if (score < 50) riskLevel = 'moderate';
    else if (score < 75) riskLevel = 'high';
    else riskLevel = 'critical';

    const recommendations = [];
    if (surgical > 50) recommendations.push('Experienced surgeon required');
    if (biological > 25) recommendations.push('Enhanced immune monitoring');
    if (technical > 25) recommendations.push('Advanced technical support needed');
    if (longTerm > 20) recommendations.push('Long-term follow-up essential');

    return {
      score,
      level: riskLevel,
      factors: { surgical, biological, technical, longTerm },
      recommendations,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Classify implant (standalone)
 */
export function classifyImplant(input: ImplantClassificationInput): ImplantClassification {
  const sdk = new CyberneticImplantSDK();
  return sdk.classifyImplant(input);
}

/**
 * Assess biocompatibility (standalone)
 */
export function assessBiocompatibility(params: BioAssessmentParams): BiocompatibilityResult {
  const sdk = new CyberneticImplantSDK();
  return sdk.assessBiocompatibility(params);
}

/**
 * Manage power (standalone)
 */
export function managePower(config: PowerConfig): PowerMetrics {
  const sdk = new CyberneticImplantSDK();
  return sdk.managePower(config);
}

/**
 * Monitor rejection (standalone)
 */
export function monitorRejection(data: RejectionMonitoringData): RejectionStatus {
  const sdk = new CyberneticImplantSDK();
  return sdk.monitorRejection(data);
}

/**
 * Update firmware (standalone)
 */
export function updateFirmware(updateId: string): UpdateResult {
  const sdk = new CyberneticImplantSDK();
  return sdk.updateFirmware(updateId);
}

/**
 * Schedule explant (standalone)
 */
export function scheduleExplant(params: ExplantParams): ExplantSchedule {
  const sdk = new CyberneticImplantSDK();
  return sdk.scheduleExplant(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { CyberneticImplantSDK };
export default CyberneticImplantSDK;
