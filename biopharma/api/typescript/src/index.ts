/**
 * WIA-BIO-016: Biopharma SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biopharmaceutical Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for biopharmaceutical development including:
 * - Binding affinity calculations
 * - Pharmacokinetic/pharmacodynamic modeling
 * - Immunogenicity risk assessment
 * - Drug candidate validation
 * - Quality attribute analysis
 */

import {
  BindingAffinity,
  BindingAffinityResult,
  PKParameters,
  PKResult,
  PDParameters,
  PDResult,
  ImmunogenicityParams,
  ImmunogenicityRisk,
  DrugCandidate,
  DrugValidationResult,
  QualityAttributes,
  StabilityResult,
  StabilityConditions,
  BiosimilarComparison,
  BiosimilarAssessment,
  BIOPHARMA_CONSTANTS,
  BiopharmaErrorCode,
  BiopharmaError,
  KineticParameters,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-016 Biopharma SDK
 */
export class BiopharmaSDK {
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
   * Calculate binding affinity (Kd) from equilibrium concentrations
   *
   * @param params - Binding affinity parameters
   * @returns Binding affinity result with Kd and classification
   */
  calculateBindingAffinity(params: BindingAffinity): BindingAffinityResult {
    const { antibodyConc, antigenConc, complexConc, temperature } = params;

    // Validate inputs
    if (antibodyConc <= 0 || antigenConc <= 0 || complexConc < 0) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'Concentrations must be positive'
      );
    }

    if (complexConc > antibodyConc || complexConc > antigenConc) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'Complex concentration cannot exceed reactant concentrations'
      );
    }

    // Calculate free concentrations
    const freeAntibody = antibodyConc - complexConc;
    const freeAntigen = antigenConc - complexConc;

    // Calculate Kd: Kd = [Ab][Ag] / [Ab-Ag]
    const kd = complexConc > 0 ? (freeAntibody * freeAntigen) / complexConc : Infinity;

    // Calculate fraction bound
    const fractionBound = complexConc / antibodyConc;

    // Classify binding strength
    let bindingStrength: BindingAffinityResult['bindingStrength'];
    if (kd < 1e-12) {
      bindingStrength = 'ultra-high';
    } else if (kd < 100e-12) {
      bindingStrength = 'very-high';
    } else if (kd < 1e-9) {
      bindingStrength = 'high';
    } else if (kd < 10e-9) {
      bindingStrength = 'moderate';
    } else if (kd < 100e-9) {
      bindingStrength = 'low';
    } else {
      bindingStrength = 'very-low';
    }

    // Determine therapeutic suitability
    const therapeuticSuitability = kd <= BIOPHARMA_CONSTANTS.MIN_THERAPEUTIC_KD;

    return {
      kd,
      freeAntibody,
      freeAntigen,
      bindingStrength,
      fractionBound,
      therapeuticSuitability,
    };
  }

  /**
   * Calculate binding kinetics from association and dissociation rates
   *
   * @param kon - Association rate constant (M⁻¹s⁻¹)
   * @param koff - Dissociation rate constant (s⁻¹)
   * @returns Kinetic parameters including Kd
   */
  calculateKinetics(kon: number, koff: number): KineticParameters {
    if (kon <= 0 || koff <= 0) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'Rate constants must be positive'
      );
    }

    const kd = koff / kon;
    const complexHalfLife = Math.log(2) / koff;

    return {
      kon,
      koff,
      kd,
      complexHalfLife,
      method: 'spr',
      temperature: 25,
    };
  }

  /**
   * Calculate pharmacokinetic parameters
   *
   * @param params - PK parameters
   * @returns PK results including AUC, half-life, and dosing recommendations
   */
  calculatePharmacokinetics(params: PKParameters): PKResult {
    const { dose, bioavailability, clearance, volumeOfDistribution } = params;

    // Validate inputs
    if (dose <= 0 || clearance <= 0 || volumeOfDistribution <= 0) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'Dose, clearance, and volume must be positive'
      );
    }

    if (bioavailability < 0 || bioavailability > 1) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'Bioavailability must be between 0 and 1'
      );
    }

    // Calculate AUC: AUC = (Dose × F) / CL
    const auc = (dose * bioavailability) / clearance;

    // Calculate elimination rate constant: ke = CL / Vd
    const ke = clearance / volumeOfDistribution;

    // Calculate half-life: t½ = 0.693 / ke
    const halfLife = 0.693 / ke;

    // Calculate initial concentration: C0 = Dose / Vd
    const c0 = dose / volumeOfDistribution;

    // Recommend dosing interval (typically 1-2 half-lives for maintenance)
    let dosingInterval: number;
    if (halfLife < 12) {
      dosingInterval = Math.max(4, Math.ceil(halfLife / 2));
    } else if (halfLife < 48) {
      dosingInterval = 24;
    } else {
      dosingInterval = Math.ceil(halfLife);
    }

    return {
      auc,
      halfLife,
      c0,
      ke,
      dosingInterval,
    };
  }

  /**
   * Calculate pharmacodynamic effect
   *
   * @param params - PD parameters
   * @returns PD results including effect and receptor occupancy
   */
  calculatePharmacodynamics(params: PDParameters): PDResult {
    const {
      concentration,
      emax,
      ec50,
      hillCoefficient = 1,
      baseline = 0,
    } = params;

    // Validate inputs
    if (concentration < 0 || emax < 0 || ec50 <= 0) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'Invalid PD parameters'
      );
    }

    // Calculate effect using Hill equation: E = Emax × C^n / (EC50^n + C^n)
    const cPower = Math.pow(concentration, hillCoefficient);
    const ec50Power = Math.pow(ec50, hillCoefficient);
    const effect = baseline + (emax * cPower) / (ec50Power + cPower);

    // Calculate receptor occupancy (simple binding)
    const receptorOccupancy = concentration / (ec50 + concentration);

    // Effect percentage
    const effectPercentage = (effect / emax) * 100;

    // Therapeutic range (typically 50-90% of Emax)
    const therapeuticRange = {
      min: ec50 * 0.5,
      max: ec50 * 9, // ~90% occupancy
    };

    const withinRange =
      concentration >= therapeuticRange.min && concentration <= therapeuticRange.max;

    return {
      effect,
      receptorOccupancy,
      effectPercentage,
      therapeuticRange,
      withinRange,
    };
  }

  /**
   * Assess immunogenicity risk
   *
   * @param params - Immunogenicity assessment parameters
   * @returns Immunogenicity risk assessment
   */
  assessImmunogenicity(params: ImmunogenicityParams): ImmunogenicityRisk {
    const { adaRate, severity, durationFactor, toleranceFactor } = params;

    // Validate inputs
    if (adaRate < 0 || adaRate > 1) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'ADA rate must be between 0 and 1'
      );
    }

    if (severity < 1 || severity > 10) {
      throw new BiopharmaError(
        BiopharmaErrorCode.INVALID_PARAMETERS,
        'Severity must be between 1 and 10'
      );
    }

    // Calculate immunogenicity risk score
    // IRS = (ADA_rate × Severity × Duration) / Tolerance
    const riskScore = (adaRate * severity * durationFactor) / toleranceFactor;

    // Classify risk level
    let riskLevel: ImmunogenicityRisk['riskLevel'];
    if (riskScore < 10) {
      riskLevel = 'low';
    } else if (riskScore < 30) {
      riskLevel = 'moderate';
    } else if (riskScore < 60) {
      riskLevel = 'high';
    } else {
      riskLevel = 'very-high';
    }

    // Predict ADA incidence
    const predictedADA = Math.min(1, adaRate * (1 + riskScore / 100));

    // Determine monitoring frequency
    let monitoringFrequency: ImmunogenicityRisk['monitoringFrequency'];
    if (riskLevel === 'low') {
      monitoringFrequency = 'standard';
    } else if (riskLevel === 'moderate') {
      monitoringFrequency = 'frequent';
    } else {
      monitoringFrequency = 'intensive';
    }

    // Mitigation strategies
    const mitigationStrategies: string[] = [];
    if (riskScore > 20) {
      mitigationStrategies.push('Sequence optimization / humanization');
      mitigationStrategies.push('Deimmunization of T-cell epitopes');
    }
    if (riskScore > 40) {
      mitigationStrategies.push('Consider alternative format (e.g., Fab, scFv)');
      mitigationStrategies.push('Immunosuppression protocol');
    }
    if (adaRate > 0.3) {
      mitigationStrategies.push('Combination therapy to reduce immunogenicity');
    }

    // Clinical recommendations
    const recommendations: string[] = [];
    recommendations.push(`Monitor ADA levels ${monitoringFrequency}ly during treatment`);
    if (riskLevel !== 'low') {
      recommendations.push('Baseline and periodic PK/PD assessments');
    }
    if (riskLevel === 'high' || riskLevel === 'very-high') {
      recommendations.push('Consider premedication or dose adjustment');
      recommendations.push('Have alternative therapy available');
    }

    return {
      riskScore,
      riskLevel,
      predictedADA,
      monitoringFrequency,
      mitigationStrategies,
      recommendations,
    };
  }

  /**
   * Validate drug candidate
   *
   * @param candidate - Drug candidate parameters
   * @returns Validation result with feasibility assessment
   */
  validateDrugCandidate(candidate: DrugCandidate): DrugValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    const risks: string[] = [];
    const recommendations: string[] = [];

    // Validate affinity
    if (candidate.targetAffinity > BIOPHARMA_CONSTANTS.MIN_THERAPEUTIC_KD) {
      warnings.push(
        `Target affinity (${candidate.targetAffinity.toExponential()} M) is weaker than typical therapeutic threshold (${BIOPHARMA_CONSTANTS.MIN_THERAPEUTIC_KD.toExponential()} M)`
      );
      risks.push('Suboptimal target affinity may reduce efficacy');
    }

    if (candidate.targetAffinity > 100e-9) {
      errors.push('Affinity too weak for therapeutic application');
    }

    // Validate stability
    if (candidate.stability < 0.9) {
      warnings.push('Stability is below optimal threshold (90%)');
      recommendations.push('Optimize formulation for improved stability');
    }

    if (candidate.stability < 0.7) {
      errors.push('Insufficient stability for development');
      risks.push('Stability issues may prevent successful formulation');
    }

    // Assess immunogenicity
    if (candidate.immunogenicityRisk === 'high') {
      warnings.push('High immunogenicity risk detected');
      recommendations.push('Consider humanization or deimmunization strategies');
      risks.push('High ADA incidence may limit clinical utility');
    }

    // Validate molecular weight (for antibodies)
    if (candidate.drugType === 'monoclonal-antibody') {
      if (
        candidate.molecularWeight < 145000 ||
        candidate.molecularWeight > 155000
      ) {
        warnings.push(
          'Molecular weight outside typical IgG range (145-155 kDa)'
        );
      }
    }

    // Manufacturing assessment
    if (candidate.manufacturingComplexity === 'high') {
      warnings.push('High manufacturing complexity may increase costs');
      risks.push('Complex manufacturing may delay development timeline');
    }

    // Expression system validation
    if (
      candidate.drugType === 'monoclonal-antibody' &&
      (candidate.expressionSystem === 'e-coli' ||
        candidate.expressionSystem === 'yeast')
    ) {
      warnings.push(
        'Expression system may not provide appropriate glycosylation for antibodies'
      );
      recommendations.push('Consider CHO or HEK293 expression system');
    }

    // Determine feasibility
    let feasibility: DrugValidationResult['feasibility'];
    if (errors.length > 0) {
      feasibility = 'low';
    } else if (warnings.length > 2 || risks.length > 2) {
      feasibility = 'medium';
    } else {
      feasibility = 'high';
    }

    // Estimate development timeline
    let estimatedTimeline: number;
    switch (candidate.developmentStage) {
      case 'discovery':
        estimatedTimeline = 10;
        break;
      case 'preclinical':
        estimatedTimeline = 8;
        break;
      case 'phase1':
        estimatedTimeline = 6;
        break;
      case 'phase2':
        estimatedTimeline = 4;
        break;
      case 'phase3':
        estimatedTimeline = 2;
        break;
      case 'approved':
        estimatedTimeline = 0;
        break;
      default:
        estimatedTimeline = 10;
    }

    // Adjust timeline based on complexity
    if (candidate.manufacturingComplexity === 'high') {
      estimatedTimeline += 1;
    }
    if (candidate.immunogenicityRisk === 'high') {
      estimatedTimeline += 0.5;
    }

    // Estimate development cost (millions USD)
    let estimatedCost: number;
    if (candidate.drugType === 'monoclonal-antibody') {
      estimatedCost = 1500; // Typical mAb development cost
    } else if (candidate.drugType === 'vaccine') {
      estimatedCost = 800;
    } else {
      estimatedCost = 1000;
    }

    // Adjust cost based on complexity
    if (candidate.manufacturingComplexity === 'high') {
      estimatedCost *= 1.3;
    }
    if (feasibility === 'medium') {
      estimatedCost *= 1.2;
    } else if (feasibility === 'low') {
      estimatedCost *= 1.5;
    }

    // Calculate success probability
    let successProbability = 0.9;
    successProbability -= warnings.length * 0.05;
    successProbability -= errors.length * 0.2;
    successProbability -= risks.length * 0.08;
    if (candidate.immunogenicityRisk === 'high') {
      successProbability -= 0.15;
    }
    successProbability = Math.max(0.1, Math.min(0.95, successProbability));

    // General recommendations
    if (feasibility === 'high') {
      recommendations.push('Strong candidate for advancement');
      recommendations.push('Proceed with lead optimization and CMC development');
    } else if (feasibility === 'medium') {
      recommendations.push('Address identified risks before advancing');
      recommendations.push('Consider backup candidates in parallel');
    } else {
      recommendations.push('Significant issues require resolution');
      recommendations.push('May need to return to discovery phase');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      feasibility,
      estimatedTimeline,
      estimatedCost,
      risks,
      recommendations,
      successProbability,
    };
  }

  /**
   * Assess biosimilarity between reference and biosimilar products
   *
   * @param reference - Reference product quality attributes
   * @param biosimilar - Biosimilar product quality attributes
   * @returns Biosimilarity assessment
   */
  assessBiosimilarity(
    reference: QualityAttributes,
    biosimilar: QualityAttributes
  ): BiosimilarAssessment {
    const comparisons: BiosimilarComparison[] = [];

    // Helper function to create comparison
    const createComparison = (
      attribute: string,
      refValue: number,
      bioValue: number,
      tier: 1 | 2 | 3,
      threshold: number = 5
    ): BiosimilarComparison => {
      const similarity = 100 - (Math.abs(bioValue - refValue) / refValue) * 100;
      const acceptable = Math.abs(bioValue - refValue) / refValue <= threshold / 100;

      return {
        attribute,
        referenceValue: refValue,
        biosimilarValue: bioValue,
        similarity: Math.max(0, similarity),
        tier,
        acceptable,
      };
    };

    // Tier 1 comparisons (most critical)
    comparisons.push(
      createComparison('Potency', reference.potency, biosimilar.potency, 1, 20)
    );
    comparisons.push(
      createComparison('Purity', reference.purity, biosimilar.purity, 1, 5)
    );

    // Tier 2 comparisons
    comparisons.push(
      createComparison('Monomer', reference.monomer, biosimilar.monomer, 2, 5)
    );
    comparisons.push(
      createComparison(
        'Aggregates',
        reference.aggregates,
        biosimilar.aggregates,
        2,
        50
      )
    );
    comparisons.push(
      createComparison('Main Peak', reference.mainPeak, biosimilar.mainPeak, 2, 10)
    );

    // Tier 3 comparisons
    comparisons.push(
      createComparison(
        'Acidic Variants',
        reference.acidicVariants,
        biosimilar.acidicVariants,
        3,
        20
      )
    );
    comparisons.push(
      createComparison(
        'Basic Variants',
        reference.basicVariants,
        biosimilar.basicVariants,
        3,
        20
      )
    );

    // Calculate overall similarity
    const tier1Avg =
      comparisons
        .filter((c) => c.tier === 1)
        .reduce((sum, c) => sum + c.similarity, 0) /
      comparisons.filter((c) => c.tier === 1).length;

    const tier2Avg =
      comparisons
        .filter((c) => c.tier === 2)
        .reduce((sum, c) => sum + c.similarity, 0) /
      comparisons.filter((c) => c.tier === 2).length;

    const tier3Avg =
      comparisons
        .filter((c) => c.tier === 3)
        .reduce((sum, c) => sum + c.similarity, 0) /
      comparisons.filter((c) => c.tier === 3).length;

    // Weighted average (Tier 1: 50%, Tier 2: 35%, Tier 3: 15%)
    const overallSimilarity = tier1Avg * 0.5 + tier2Avg * 0.35 + tier3Avg * 0.15;

    // Analytical similarity
    const tier1Pass = comparisons
      .filter((c) => c.tier === 1)
      .every((c) => c.acceptable);
    const tier2Pass = comparisons
      .filter((c) => c.tier === 2)
      .every((c) => c.acceptable);

    const analyticalSimilarity =
      tier1Pass && tier2Pass && overallSimilarity >= 95;

    // Determine conclusion
    let conclusion: BiosimilarAssessment['conclusion'];
    if (analyticalSimilarity) {
      conclusion = 'similar';
    } else if (overallSimilarity >= 90 && tier1Pass) {
      conclusion = 'further-testing-required';
    } else {
      conclusion = 'not-similar';
    }

    // Recommendations
    const recommendations: string[] = [];
    if (conclusion === 'similar') {
      recommendations.push('Analytical similarity demonstrated');
      recommendations.push('Proceed with PK similarity studies');
    } else if (conclusion === 'further-testing-required') {
      recommendations.push('Additional analytical characterization needed');
      recommendations.push('Investigate differences in Tier 2 attributes');
    } else {
      recommendations.push('Significant differences detected');
      recommendations.push('Manufacturing process optimization required');
      recommendations.push('May need to reformulate or modify production');
    }

    // Check specific attributes
    const failedTier1 = comparisons.filter((c) => c.tier === 1 && !c.acceptable);
    if (failedTier1.length > 0) {
      recommendations.push(
        `Critical: Address ${failedTier1.map((c) => c.attribute).join(', ')}`
      );
    }

    return {
      overallSimilarity,
      comparisons,
      analyticalSimilarity,
      conclusion,
      recommendations,
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Convert concentration units
   */
  convertConcentration(
    value: number,
    fromUnit: string,
    toUnit: string,
    molecularWeight?: number
  ): number {
    // Simplified conversion (full implementation would handle all unit combinations)
    const molarUnits = ['M', 'mM', 'μM', 'nM', 'pM'];
    const massUnits = ['mg/mL', 'μg/mL'];

    if (molarUnits.includes(fromUnit) && molarUnits.includes(toUnit)) {
      // Molar to molar conversion
      const toM: Record<string, number> = {
        M: 1,
        mM: 1e-3,
        μM: 1e-6,
        nM: 1e-9,
        pM: 1e-12,
      };
      return (value * toM[fromUnit]) / toM[toUnit];
    }

    throw new BiopharmaError(
      BiopharmaErrorCode.INVALID_PARAMETERS,
      'Unit conversion not implemented for these units'
    );
  }

  /**
   * Calculate stability prediction using Arrhenius equation
   */
  predictShelfLife(
    k25C: number,
    activationEnergy: number,
    targetTemp: number = 5
  ): number {
    const R = 8.314; // Gas constant J/mol·K
    const T1 = 273.15 + 25; // 25°C in Kelvin
    const T2 = 273.15 + targetTemp;

    // k2 = k1 × exp[(Ea/R) × (1/T1 - 1/T2)]
    const k2 =
      k25C * Math.exp((activationEnergy / R) * (1 / T1 - 1 / T2));

    // t90 = 0.105 / k (time to 90% potency)
    const t90Days = 0.105 / k2 / (24 * 3600); // Convert seconds to days
    const t90Months = t90Days / 30;

    return t90Months;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate binding affinity (standalone function)
 */
export function calculateBindingAffinity(
  params: BindingAffinity
): BindingAffinityResult {
  const sdk = new BiopharmaSDK();
  return sdk.calculateBindingAffinity(params);
}

/**
 * Calculate pharmacokinetics (standalone function)
 */
export function calculatePharmacokinetics(params: PKParameters): PKResult {
  const sdk = new BiopharmaSDK();
  return sdk.calculatePharmacokinetics(params);
}

/**
 * Calculate pharmacodynamics (standalone function)
 */
export function calculatePharmacodynamics(params: PDParameters): PDResult {
  const sdk = new BiopharmaSDK();
  return sdk.calculatePharmacodynamics(params);
}

/**
 * Assess immunogenicity (standalone function)
 */
export function assessImmunogenicity(
  params: ImmunogenicityParams
): ImmunogenicityRisk {
  const sdk = new BiopharmaSDK();
  return sdk.assessImmunogenicity(params);
}

/**
 * Validate drug candidate (standalone function)
 */
export function validateDrugCandidate(
  candidate: DrugCandidate
): DrugValidationResult {
  const sdk = new BiopharmaSDK();
  return sdk.validateDrugCandidate(candidate);
}

/**
 * Assess biosimilarity (standalone function)
 */
export function assessBiosimilarity(
  reference: QualityAttributes,
  biosimilar: QualityAttributes
): BiosimilarAssessment {
  const sdk = new BiopharmaSDK();
  return sdk.assessBiosimilarity(reference, biosimilar);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BiopharmaSDK };
export default BiopharmaSDK;
