/**
 * WIA-BIO-017: Bio-Safety SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biosafety Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for biosafety including:
 * - Risk score calculations
 * - BSL compliance validation
 * - PPE requirement generation
 * - Exposure assessment
 * - Incident management
 */

import {
  RiskAssessment,
  RiskScore,
  RiskLevel,
  BiosaftyLevel,
  BSLValidation,
  ComplianceResult,
  PPERequest,
  PPERequirements,
  ExposureAssessment,
  ExposureResult,
  ContainmentEffectiveness,
  IncidentReport,
  BiologicalAgent,
  BIOSAFETY_CONSTANTS,
  BiosaftyErrorCode,
  BiosaftyError,
  GloveRequirement,
  ExposureRoute,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-017 Bio-Safety SDK
 */
export class BioSafetySDK {
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
   * Calculate biosafety risk score
   *
   * @param assessment - Risk assessment parameters
   * @returns Risk score and recommendations
   */
  calculateRiskScore(assessment: RiskAssessment): RiskScore {
    const {
      hazardLevel,
      exposureProbability,
      impactExtent,
      containmentEffectiveness,
    } = assessment;

    // Validate inputs
    if (hazardLevel < 1 || hazardLevel > 4) {
      throw new BiosaftyError(
        BiosaftyErrorCode.INVALID_PARAMETERS,
        'Hazard level must be between 1 and 4'
      );
    }

    if (exposureProbability < 0 || exposureProbability > 1) {
      throw new BiosaftyError(
        BiosaftyErrorCode.INVALID_PARAMETERS,
        'Exposure probability must be between 0 and 1'
      );
    }

    if (impactExtent < 1 || impactExtent > 10) {
      throw new BiosaftyError(
        BiosaftyErrorCode.INVALID_PARAMETERS,
        'Impact extent must be between 1 and 10'
      );
    }

    if (containmentEffectiveness < 0.1 || containmentEffectiveness > 1) {
      throw new BiosaftyError(
        BiosaftyErrorCode.INVALID_PARAMETERS,
        'Containment effectiveness must be between 0.1 and 1'
      );
    }

    // Calculate risk score: R = (H × P × E) / C
    const riskScore =
      (hazardLevel * exposureProbability * impactExtent) / containmentEffectiveness;

    // Determine risk level
    let riskLevel: RiskLevel;
    if (riskScore < BIOSAFETY_CONSTANTS.RISK_THRESHOLDS.LOW) {
      riskLevel = 'low';
    } else if (riskScore < BIOSAFETY_CONSTANTS.RISK_THRESHOLDS.MEDIUM) {
      riskLevel = 'medium';
    } else if (riskScore < BIOSAFETY_CONSTANTS.RISK_THRESHOLDS.HIGH) {
      riskLevel = 'high';
    } else {
      riskLevel = 'extreme';
    }

    // Determine recommended BSL
    let bslRecommended: BiosaftyLevel;
    if (riskScore < 10 || hazardLevel === 1) {
      bslRecommended = 'BSL-1';
    } else if (riskScore < 30 || hazardLevel === 2) {
      bslRecommended = 'BSL-2';
    } else if (riskScore < 60 || hazardLevel === 3) {
      bslRecommended = 'BSL-3';
    } else {
      bslRecommended = 'BSL-4';
    }

    // Generate recommendations
    const recommendations: string[] = [];

    if (riskLevel === 'extreme') {
      recommendations.push('CRITICAL: Immediate review of safety procedures required');
      recommendations.push('Consider upgrading to BSL-4 or abandoning procedure');
      recommendations.push('Implement maximum containment and PPE');
    } else if (riskLevel === 'high') {
      recommendations.push('High risk detected - enhanced safety measures required');
      recommendations.push('Upgrade to BSL-3 or implement additional engineering controls');
      recommendations.push('Require respiratory protection and enhanced PPE');
    } else if (riskLevel === 'medium') {
      recommendations.push('Moderate risk - standard BSL-2 practices should be followed');
      recommendations.push('Ensure proper PPE and biosafety cabinet use');
      recommendations.push('Consider additional training for personnel');
    } else {
      recommendations.push('Low risk - standard microbiological practices sufficient');
      recommendations.push('Maintain good laboratory practices');
    }

    if (exposureProbability > 0.5) {
      recommendations.push('High exposure probability - review procedure to minimize contact');
    }

    if (containmentEffectiveness < 0.7) {
      recommendations.push('Low containment effectiveness - upgrade engineering controls');
      recommendations.push('Verify BSC certification and proper use');
    }

    return {
      riskScore,
      riskLevel,
      bslRecommended,
      recommendations,
      components: {
        hazard: hazardLevel,
        probability: exposureProbability,
        impact: impactExtent,
        containment: containmentEffectiveness,
      },
    };
  }

  /**
   * Validate BSL compliance
   *
   * @param validation - BSL validation parameters
   * @returns Compliance result with violations and recommendations
   */
  validateBSLLevel(validation: BSLValidation): ComplianceResult {
    const {
      pathogenClass,
      facilityLevel,
      procedureType,
      ppeAvailable,
      engineeringControls = [],
      administrativeControls = [],
      aerosolGenerating = false,
    } = validation;

    const violations: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // BSL level mapping
    const bslLevels: Record<BiosaftyLevel, number> = {
      'BSL-1': 1,
      'BSL-2': 2,
      'BSL-3': 3,
      'BSL-4': 4,
    };

    // Check facility level matches pathogen requirement
    if (bslLevels[facilityLevel] < bslLevels[pathogenClass]) {
      violations.push(
        `Facility level (${facilityLevel}) insufficient for pathogen class (${pathogenClass})`
      );
      violations.push(`Upgrade facility to ${pathogenClass} or higher`);
    }

    // BSL-specific requirements
    switch (pathogenClass) {
      case 'BSL-4':
        // BSL-4 requirements
        if (!ppeAvailable.includes('positive-pressure-suit') && !ppeAvailable.includes('class-III-BSC')) {
          violations.push('BSL-4 requires positive pressure suit or Class III BSC');
        }
        if (!engineeringControls.includes('isolated-zone')) {
          violations.push('BSL-4 requires isolated zone');
        }
        if (!engineeringControls.includes('double-door-airlock')) {
          violations.push('BSL-4 requires double-door airlock entry');
        }
        if (!engineeringControls.includes('effluent-decontamination')) {
          violations.push('BSL-4 requires effluent decontamination system');
        }
        recommendations.push('Verify personnel competency for BSL-4 operations');
        recommendations.push('Ensure buddy system protocol in place');
        break;

      case 'BSL-3':
        // BSL-3 requirements
        if (!ppeAvailable.includes('N95') && !ppeAvailable.includes('PAPR')) {
          violations.push('BSL-3 requires respiratory protection (N95 or PAPR)');
        }
        if (!ppeAvailable.includes('solid-front-gown')) {
          warnings.push('BSL-3 recommends solid-front gowns');
        }
        if (!engineeringControls.includes('BSC')) {
          violations.push('BSL-3 requires Biological Safety Cabinet for all procedures');
        }
        if (!engineeringControls.includes('directional-airflow')) {
          violations.push('BSL-3 requires directional inward airflow');
        }
        if (!engineeringControls.includes('autoclave-in-room')) {
          warnings.push('BSL-3 recommends autoclave within the laboratory');
        }
        if (!administrativeControls.includes('controlled-access')) {
          violations.push('BSL-3 requires controlled access (authorized personnel only)');
        }
        recommendations.push('Conduct annual respiratory fit testing');
        recommendations.push('Establish baseline serum samples for personnel');
        break;

      case 'BSL-2':
        // BSL-2 requirements
        if (aerosolGenerating && !engineeringControls.includes('BSC')) {
          violations.push('BSL-2 requires BSC for aerosol-generating procedures');
        }
        if (!ppeAvailable.includes('gloves')) {
          violations.push('BSL-2 requires gloves');
        }
        if (!ppeAvailable.includes('lab-coat') && !ppeAvailable.includes('gown')) {
          violations.push('BSL-2 requires lab coat or gown');
        }
        if (!ppeAvailable.includes('eye-protection')) {
          warnings.push('BSL-2 requires eye protection for splash hazards');
        }
        if (!engineeringControls.includes('autoclave-available')) {
          violations.push('BSL-2 requires autoclave availability');
        }
        if (!administrativeControls.includes('limited-access')) {
          warnings.push('BSL-2 recommends limited access during work');
        }
        recommendations.push('Implement sharps safety program');
        recommendations.push('Establish waste decontamination procedures');
        break;

      case 'BSL-1':
        // BSL-1 requirements
        if (!ppeAvailable.includes('lab-coat')) {
          warnings.push('BSL-1 recommends lab coat');
        }
        recommendations.push('Maintain standard microbiological practices');
        recommendations.push('Ensure hand washing facilities available');
        break;
    }

    // Calculate compliance score
    const totalChecks =
      (violations.length > 0 ? violations.length : 1) +
      (warnings.length > 0 ? warnings.length : 1);
    const passedChecks = totalChecks - violations.length - warnings.length * 0.5;
    const complianceScore = Math.max(0, Math.round((passedChecks / totalChecks) * 100));

    const isCompliant = violations.length === 0;

    return {
      isCompliant,
      violations,
      warnings,
      recommendations,
      complianceScore,
      requiredUpgrades: isCompliant ? undefined : violations,
    };
  }

  /**
   * Generate PPE requirements
   *
   * @param request - PPE request parameters
   * @returns Complete PPE requirements
   */
  generatePPERequirements(request: PPERequest): PPERequirements {
    const {
      bslLevel,
      procedureType,
      aerosolGenerating,
      chemicalExposure = false,
      sharpsInvolved = false,
    } = request;

    const additional: string[] = [];

    // Determine gloves
    let gloves: GloveRequirement;
    if (bslLevel >= 3) {
      gloves = {
        material: chemicalExposure ? 'nitrile' : 'nitrile',
        layers: 2,
        extendedCuff: true,
        chemicalResistance: chemicalExposure ? 8 : 6,
        punctureResistance: sharpsInvolved ? 8 : 6,
      };
    } else if (bslLevel === 2) {
      gloves = {
        material: 'nitrile',
        layers: sharpsInvolved ? 2 : 1,
        extendedCuff: false,
        chemicalResistance: 6,
        punctureResistance: sharpsInvolved ? 7 : 5,
      };
    } else {
      gloves = {
        material: 'latex',
        layers: 1,
        extendedCuff: false,
        chemicalResistance: 4,
        punctureResistance: 3,
      };
    }

    // Determine respiratory protection
    let respiratoryProtection: PPERequirements['respiratoryProtection'];
    if (bslLevel === 4) {
      respiratoryProtection = 'supplied-air';
    } else if (bslLevel === 3 || (bslLevel === 2 && aerosolGenerating)) {
      respiratoryProtection = 'N95';
      additional.push('Annual fit testing required for N95');
    } else {
      respiratoryProtection = 'none';
    }

    // Determine eye protection
    let eyeProtection: PPERequirements['eyeProtection'];
    if (bslLevel === 4) {
      eyeProtection = 'full-face-respirator';
    } else if (bslLevel === 3 || aerosolGenerating) {
      eyeProtection = 'face-shield';
      additional.push('Use face shield with safety glasses');
    } else if (bslLevel === 2) {
      eyeProtection = 'safety-glasses';
    } else {
      eyeProtection = 'safety-glasses';
    }

    // Determine body protection
    let bodyProtection: PPERequirements['bodyProtection'];
    if (bslLevel === 4) {
      bodyProtection = 'positive-pressure-suit';
    } else if (bslLevel === 3) {
      bodyProtection = 'solid-front-gown';
      additional.push('Decontaminate gown before removal from laboratory');
    } else if (bslLevel === 2) {
      bodyProtection = 'lab-coat';
    } else {
      bodyProtection = 'lab-coat';
    }

    // Determine footwear
    let footwear: PPERequirements['footwear'];
    if (bslLevel === 4) {
      footwear = 'dedicated-boots';
    } else if (bslLevel === 3) {
      footwear = 'dedicated-shoes';
      additional.push('Shoes must remain in laboratory');
    } else {
      footwear = 'closed-toe-shoes';
    }

    // Additional requirements
    if (sharpsInvolved) {
      additional.push('Use safety-engineered sharps devices');
      additional.push('Never recap needles');
    }

    if (chemicalExposure) {
      additional.push('Verify glove chemical compatibility');
      additional.push('Change gloves if chemical exposure suspected');
    }

    // Donning and doffing sequences
    const donningSequence = [
      'Wash hands',
      'Put on first pair of gloves',
      'Put on gown/suit',
      'Put on second pair of gloves (if required)',
      'Put on respiratory protection',
      'Put on eye protection',
      'Verify all PPE properly fitted',
    ];

    const doffingSequence = [
      'Remove outer gloves',
      'Remove gown/suit (inside-out)',
      'Remove eye protection',
      'Remove respiratory protection',
      'Remove inner gloves',
      'Wash hands thoroughly',
    ];

    return {
      gloves,
      respiratoryProtection,
      eyeProtection,
      bodyProtection,
      footwear,
      additional,
      donningSequence,
      doffingSequence,
    };
  }

  /**
   * Assess exposure incident
   *
   * @param assessment - Exposure assessment parameters
   * @returns Exposure result with risk and recommendations
   */
  assessExposure(assessment: ExposureAssessment): ExposureResult {
    const {
      doseConcentration,
      frequency,
      duration,
      safetyFactor = 0.1,
      route = 'inhalation',
    } = assessment;

    // Calculate exposure value: EA = (D × F × T) × SF
    const exposureValue = doseConcentration * frequency * duration * safetyFactor;

    // Apply route-specific multiplier
    const routeMultiplier = BIOSAFETY_CONSTANTS.ROUTE_MULTIPLIERS[route];
    const adjustedExposure = exposureValue * routeMultiplier;

    // Estimate infectious dose based on typical ID50 ranges
    let infectiousDose: number;
    if (adjustedExposure < 10) {
      infectiousDose = adjustedExposure;
    } else if (adjustedExposure < 1000) {
      infectiousDose = adjustedExposure * 0.1;
    } else {
      infectiousDose = adjustedExposure * 0.01;
    }

    // Calculate infection risk (simplified model)
    const infectionRisk = Math.min(1, infectiousDose / 1000);

    // Generate recommendations
    const recommendations: string[] = [];
    let medicalEvaluationRequired = false;

    if (infectionRisk > 0.5) {
      recommendations.push('HIGH RISK: Immediate medical evaluation required');
      recommendations.push('Consider post-exposure prophylaxis');
      recommendations.push('Implement strict quarantine protocols');
      recommendations.push('Daily symptom monitoring for 14 days minimum');
      medicalEvaluationRequired = true;
    } else if (infectionRisk > 0.1) {
      recommendations.push('MODERATE RISK: Medical evaluation recommended');
      recommendations.push('Monitor for symptoms');
      recommendations.push('Consider prophylactic measures');
      recommendations.push('Follow-up testing as appropriate');
      medicalEvaluationRequired = true;
    } else if (infectionRisk > 0.01) {
      recommendations.push('LOW RISK: Document exposure and monitor');
      recommendations.push('Be alert for symptoms');
      recommendations.push('Report any unusual symptoms immediately');
      medicalEvaluationRequired = false;
    } else {
      recommendations.push('MINIMAL RISK: Standard precautions sufficient');
      recommendations.push('Document incident for records');
      medicalEvaluationRequired = false;
    }

    // Route-specific recommendations
    if (route === 'percutaneous') {
      recommendations.push('Wash wound thoroughly with soap and water');
      recommendations.push('Do not squeeze or manipulate wound');
      recommendations.push('Apply antiseptic');
    } else if (route === 'mucous-membrane') {
      recommendations.push('Irrigate affected area for 15 minutes');
      recommendations.push('Use eyewash station or safety shower');
    } else if (route === 'inhalation') {
      recommendations.push('Move to fresh air immediately');
      recommendations.push('Monitor respiratory symptoms closely');
    }

    return {
      exposureValue,
      infectiousDose,
      infectionRisk,
      recommendations,
      medicalEvaluationRequired,
    };
  }

  /**
   * Calculate containment effectiveness
   *
   * @param params - Containment parameters
   * @returns Containment effectiveness assessment
   */
  calculateContainmentEffectiveness(params: {
    bscEffectiveness: number;
    ppeEffectiveness: number;
    proceduralAdherence: number;
    breaches?: number;
    totalOperations?: number;
  }): ContainmentEffectiveness {
    const {
      bscEffectiveness,
      ppeEffectiveness,
      proceduralAdherence,
      breaches = 0,
      totalOperations = 1,
    } = params;

    // Validate inputs
    if (bscEffectiveness < 0.8 || bscEffectiveness > 0.999) {
      throw new BiosaftyError(
        BiosaftyErrorCode.INVALID_PARAMETERS,
        'BSC effectiveness must be between 0.8 and 0.999'
      );
    }

    if (ppeEffectiveness < 0.5 || ppeEffectiveness > 0.99) {
      throw new BiosaftyError(
        BiosaftyErrorCode.INVALID_PARAMETERS,
        'PPE effectiveness must be between 0.5 and 0.99'
      );
    }

    if (proceduralAdherence < 0.6 || proceduralAdherence > 0.95) {
      throw new BiosaftyError(
        BiosaftyErrorCode.INVALID_PARAMETERS,
        'Procedural adherence must be between 0.6 and 0.95'
      );
    }

    // Calculate overall effectiveness: CE = (BSC × PPE × Proc)^(1/3)
    const overallEffectiveness = Math.pow(
      bscEffectiveness * ppeEffectiveness * proceduralAdherence,
      1 / 3
    );

    // Calculate containment efficiency: CE = (1 - L/T) × 100%
    const containmentEfficiency = ((1 - breaches / totalOperations) * 100);

    return {
      bscEffectiveness,
      ppeEffectiveness,
      proceduralAdherence,
      overallEffectiveness,
      breaches,
      totalOperations,
      containmentEfficiency,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate risk score (standalone function)
 */
export function calculateRiskScore(assessment: RiskAssessment): RiskScore {
  const sdk = new BioSafetySDK();
  return sdk.calculateRiskScore(assessment);
}

/**
 * Validate BSL level (standalone function)
 */
export function validateBSLLevel(validation: BSLValidation): ComplianceResult {
  const sdk = new BioSafetySDK();
  return sdk.validateBSLLevel(validation);
}

/**
 * Generate PPE requirements (standalone function)
 */
export function generatePPERequirements(request: PPERequest): PPERequirements {
  const sdk = new BioSafetySDK();
  return sdk.generatePPERequirements(request);
}

/**
 * Assess exposure (standalone function)
 */
export function assessExposure(assessment: ExposureAssessment): ExposureResult {
  const sdk = new BioSafetySDK();
  return sdk.assessExposure(assessment);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BioSafetySDK };
export default BioSafetySDK;
