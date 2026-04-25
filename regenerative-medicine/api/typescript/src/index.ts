/**
 * WIA-BIO-020: Regenerative Medicine SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Regenerative Medicine Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for regenerative medicine including:
 * - Stem cell characterization and analysis
 * - Tissue regeneration rate calculations
 * - Scaffold design and optimization
 * - Growth factor delivery systems
 * - Clinical protocol management
 */

import {
  TissueType,
  StemCellType,
  GrowthFactorType,
  ScaffoldMaterial,
  RegenerationRequest,
  RegenerationResponse,
  SurvivalRequest,
  SurvivalResponse,
  ScaffoldRequest,
  ScaffoldResponse,
  GrowthFactorRequest,
  GrowthFactorResponse,
  CellCharacterization,
  TreatmentProtocol,
  TreatmentOutcome,
  BIO_CONSTANTS,
  BioErrorCode,
  RegenerativeMedicineError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-020 Regenerative Medicine SDK
 */
export class RegenerativeMedicineSDK {
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
   * Calculate tissue regeneration rate
   *
   * @param params - Regeneration parameters
   * @returns Regeneration rate and feasibility assessment
   */
  calculateRegenerationRate(params: RegenerationRequest): RegenerationResponse {
    const {
      tissueType,
      cellDensity,
      timeFrame,
      growthFactors,
      initialVolume = 1,
      targetVolume = 10,
      patientAge = 40,
    } = params;

    // Validate inputs
    if (cellDensity <= 0) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Cell density must be positive'
      );
    }

    if (timeFrame <= 0) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Time frame must be positive'
      );
    }

    // Tissue-specific regeneration coefficients
    const tissueCoefficients: Record<TissueType, { base: number; efficiency: number }> = {
      cardiac: { base: 0.1, efficiency: 0.3 },
      neural: { base: 0.05, efficiency: 0.2 },
      bone: { base: 0.5, efficiency: 0.8 },
      cartilage: { base: 0.15, efficiency: 0.4 },
      skin: { base: 0.8, efficiency: 0.9 },
      muscle: { base: 0.3, efficiency: 0.6 },
      liver: { base: 0.6, efficiency: 0.7 },
      kidney: { base: 0.2, efficiency: 0.4 },
      vascular: { base: 0.4, efficiency: 0.7 },
    };

    const coeff = tissueCoefficients[tissueType];

    // Age factor (younger patients regenerate faster)
    const ageFactor = 1 - (patientAge - 20) / 200;
    const ageMultiplier = Math.max(0.5, Math.min(1.5, ageFactor));

    // Growth factor bonus
    const growthFactorBonus = 1 + (growthFactors.length * 0.15);

    // Calculate regeneration rate (cells/day)
    const baseRate = cellDensity * coeff.base;
    const rate = baseRate * ageMultiplier * growthFactorBonus;

    // Calculate required cell dose
    const volumeIncrease = targetVolume - initialVolume;
    const requiredCells = volumeIncrease * 1e8; // Assume 100M cells per cm³
    const requiredCellDose = Math.max(requiredCells, cellDensity * timeFrame);

    // Calculate time to complete regeneration
    const totalCellsNeeded = requiredCells;
    const cellsPerDay = rate * initialVolume;
    const timeToComplete = totalCellsNeeded / cellsPerDay;

    // Calculate recovery percentage
    const achievableVolume = initialVolume + (rate * timeFrame * initialVolume) / 1e8;
    const recoveryPercentage = Math.min(
      100,
      ((achievableVolume - initialVolume) / (targetVolume - initialVolume)) * 100
    );

    // Feasibility assessment
    let feasibility: RegenerationResponse['feasibility'];
    if (recoveryPercentage >= 80 && timeToComplete <= timeFrame * 1.5) {
      feasibility = 'high';
    } else if (recoveryPercentage >= 50 && timeToComplete <= timeFrame * 3) {
      feasibility = 'medium';
    } else if (recoveryPercentage >= 20) {
      feasibility = 'low';
    } else {
      feasibility = 'infeasible';
    }

    // Integration score
    const integrationScore = coeff.efficiency * ageMultiplier;

    // Confidence level
    const confidence = Math.min(
      1.0,
      (coeff.efficiency + growthFactors.length * 0.05) * ageMultiplier
    );

    // Generate recommendations
    const recommendations: string[] = [];

    if (feasibility === 'low' || feasibility === 'infeasible') {
      recommendations.push('Consider increasing cell dose or extending treatment duration');
    }

    if (growthFactors.length < 2) {
      recommendations.push('Add complementary growth factors for synergistic effects');
    }

    if (patientAge > 60) {
      recommendations.push('Optimize culture conditions to compensate for age-related factors');
    }

    if (tissueType === 'cardiac' || tissueType === 'neural') {
      recommendations.push(
        'Use iPSC-derived cells for better differentiation and integration'
      );
    }

    if (integrationScore < 0.7) {
      recommendations.push('Consider scaffold-based delivery to improve integration');
    }

    return {
      rate,
      recoveryPercentage,
      timeToComplete,
      feasibility,
      requiredCellDose,
      confidence,
      recommendations,
      integrationScore,
    };
  }

  /**
   * Assess cell survival and viability
   *
   * @param params - Survival assessment parameters
   * @returns Survival rate and quality assessment
   */
  assessCellSurvival(params: SurvivalRequest): SurvivalResponse {
    const {
      cellType,
      viableCells,
      totalCells,
      cultureConditions,
      temperature = 37,
      co2Level = 5,
      cultureDuration = 24,
    } = params;

    // Validate inputs
    if (viableCells < 0 || totalCells <= 0) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Cell counts must be non-negative and total must be positive'
      );
    }

    if (viableCells > totalCells) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Viable cells cannot exceed total cells'
      );
    }

    // Calculate survival rate
    const survivalRate = (viableCells / totalCells) * 100;

    // Classify viability
    let viability: SurvivalResponse['viability'];
    let qualityGrade: SurvivalResponse['qualityGrade'];

    if (survivalRate >= 90) {
      viability = 'excellent';
      qualityGrade = 'A';
    } else if (survivalRate >= 80) {
      viability = 'good';
      qualityGrade = 'B';
    } else if (survivalRate >= 70) {
      viability = 'fair';
      qualityGrade = 'C';
    } else if (survivalRate >= 50) {
      viability = 'poor';
      qualityGrade = 'D';
    } else {
      viability = 'poor';
      qualityGrade = 'F';
    }

    // Generate recommendations
    const recommendations: string[] = [];

    if (survivalRate < BIO_CONSTANTS.MIN_VIABILITY * 100) {
      recommendations.push(
        `Survival rate below minimum threshold (${BIO_CONSTANTS.MIN_VIABILITY * 100}%)`
      );
      recommendations.push('Review culture conditions and media composition');
    }

    if (temperature < 36 || temperature > 38) {
      recommendations.push('Temperature is outside optimal range (37°C ±1°C)');
    }

    if (co2Level < 4 || co2Level > 6) {
      recommendations.push('CO₂ level is outside optimal range (5% ±1%)');
    }

    if (cultureDuration > 72) {
      recommendations.push('Consider passaging cells to prevent over-confluence');
    }

    if (survivalRate < 80) {
      recommendations.push('Add survival-enhancing supplements (e.g., Y-27632 for iPSCs)');
      recommendations.push('Optimize seeding density');
    }

    if (cultureConditions.toLowerCase().includes('serum-free')) {
      recommendations.push('Ensure growth factors are supplemented in serum-free conditions');
    }

    // Estimate cryopreservation survival
    const cryoSurvivalFactor = cellType === 'iPSC' ? 0.7 : cellType === 'MSC' ? 0.85 : 0.8;
    const cryoSurvival = survivalRate * cryoSurvivalFactor;

    return {
      survivalRate,
      viability,
      recommendations,
      cryoSurvival,
      qualityGrade,
    };
  }

  /**
   * Design tissue engineering scaffold
   *
   * @param params - Scaffold design parameters
   * @returns Optimized scaffold design
   */
  designScaffold(params: ScaffoldRequest): ScaffoldResponse {
    const {
      tissueType,
      material,
      porosity,
      size,
      mechanicalRequirements = {},
      poreSize,
      degradationTime,
      cellSeeding,
    } = params;

    // Validate inputs
    if (porosity < 0 || porosity > 1) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Porosity must be between 0 and 1'
      );
    }

    if (size <= 0) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Size must be positive'
      );
    }

    // Tissue-specific defaults
    const tissueDefaults: Record<
      TissueType,
      { poreSize: number; degradation: number; youngModulus: number }
    > = {
      bone: { poreSize: 300, degradation: 12, youngModulus: 15000 },
      cartilage: { poreSize: 100, degradation: 6, youngModulus: 1.5 },
      skin: { poreSize: 80, degradation: 4, youngModulus: 0.5 },
      cardiac: { poreSize: 100, degradation: 8, youngModulus: 10 },
      neural: { poreSize: 50, degradation: 6, youngModulus: 0.1 },
      muscle: { poreSize: 150, degradation: 8, youngModulus: 50 },
      liver: { poreSize: 100, degradation: 6, youngModulus: 5 },
      kidney: { poreSize: 100, degradation: 8, youngModulus: 5 },
      vascular: { poreSize: 80, degradation: 6, youngModulus: 5 },
    };

    const defaults = tissueDefaults[tissueType];

    // Calculate dimensions (assume cubic for simplicity)
    const sideLength = Math.cbrt(size * 1000); // Convert cm³ to mm³
    const dimensions = {
      width: sideLength,
      height: sideLength,
      depth: sideLength,
      volume: size,
      surfaceArea: 6 * Math.pow(sideLength / 10, 2), // cm²
    };

    // Determine pore size
    const finalPoreSize = poreSize || defaults.poreSize;

    // Determine degradation time
    const finalDegradation = degradationTime || defaults.degradation;

    // Material-specific properties
    const materialProperties: Record<
      ScaffoldMaterial,
      { biocompat: number; youngModulus: number; tensile: number; compressive: number }
    > = {
      collagen: { biocompat: 1.0, youngModulus: 5, tensile: 2, compressive: 0.5 },
      gelatin: { biocompat: 0.95, youngModulus: 3, tensile: 1.5, compressive: 0.3 },
      chitosan: { biocompat: 0.9, youngModulus: 10, tensile: 40, compressive: 5 },
      'hyaluronic-acid': { biocompat: 1.0, youngModulus: 0.1, tensile: 0.05, compressive: 0.02 },
      fibrin: { biocompat: 1.0, youngModulus: 0.5, tensile: 0.3, compressive: 0.1 },
      alginate: { biocompat: 0.85, youngModulus: 0.2, tensile: 0.1, compressive: 0.05 },
      PLA: { biocompat: 0.8, youngModulus: 3000, tensile: 60, compressive: 80 },
      PLGA: { biocompat: 0.85, youngModulus: 2000, tensile: 50, compressive: 60 },
      PCL: { biocompat: 0.85, youngModulus: 400, tensile: 20, compressive: 25 },
      PEG: { biocompat: 0.9, youngModulus: 0.01, tensile: 0.005, compressive: 0.002 },
      hydroxyapatite: { biocompat: 0.95, youngModulus: 100000, tensile: 50, compressive: 500 },
      'beta-TCP': { biocompat: 0.95, youngModulus: 50000, tensile: 30, compressive: 300 },
    };

    const matProps = materialProperties[material];

    // Adjust for porosity (higher porosity = lower mechanical strength)
    const porosityFactor = 1 - porosity * 0.7;

    const mechanical = {
      youngModulus: matProps.youngModulus * porosityFactor,
      tensileStrength: matProps.tensile * porosityFactor,
      compressiveStrength: matProps.compressive * porosityFactor,
      matchesNative:
        Math.abs(matProps.youngModulus * porosityFactor - defaults.youngModulus) <
        defaults.youngModulus * 0.5,
    };

    // Biocompatibility rating
    const biocompatScore = matProps.biocompat;
    let biocompatibility: ScaffoldResponse['biocompatibility'];
    if (biocompatScore >= 0.95) biocompatibility = 'excellent';
    else if (biocompatScore >= 0.85) biocompatibility = 'good';
    else if (biocompatScore >= 0.75) biocompatibility = 'acceptable';
    else biocompatibility = 'poor';

    // Fabrication method recommendation
    let fabricationMethod: ScaffoldResponse['fabricationMethod'];
    if (material === 'PLA' || material === 'PLGA' || material === 'PCL') {
      fabricationMethod = '3D-printing';
    } else if (
      material === 'collagen' ||
      material === 'gelatin' ||
      material === 'chitosan'
    ) {
      fabricationMethod = 'freeze-drying';
    } else if (material === 'alginate' || material === 'hyaluronic-acid') {
      fabricationMethod = 'molding';
    } else {
      fabricationMethod = 'electrospinning';
    }

    // Cell seeding density
    const recommendedSeeding =
      cellSeeding || (tissueType === 'bone' ? 5e6 : tissueType === 'cartilage' ? 1e7 : 5e6);

    // Cost and time estimates
    const costEstimate = size * 1000 * (material.includes('beta') ? 50 : 100); // USD
    const manufacturingTime = fabricationMethod === '3D-printing' ? 2 : 5; // days

    return {
      design: {
        material,
        dimensions,
        poreSize: finalPoreSize,
        porosity,
        degradationTime: finalDegradation,
        cellSeeding: recommendedSeeding,
      },
      mechanical,
      biocompatibility,
      fabricationMethod,
      costEstimate,
      manufacturingTime,
    };
  }

  /**
   * Optimize growth factor delivery
   *
   * @param params - Growth factor parameters
   * @returns Optimized delivery protocol
   */
  optimizeGrowthFactors(params: GrowthFactorRequest): GrowthFactorResponse {
    const {
      type,
      targetConcentration,
      duration,
      deliveryMethod,
      tissueVolume = 1,
      combination = [],
    } = params;

    // Validate inputs
    if (targetConcentration <= 0) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Target concentration must be positive'
      );
    }

    if (duration <= 0) {
      throw new RegenerativeMedicineError(
        BioErrorCode.INVALID_PARAMETERS,
        'Duration must be positive'
      );
    }

    // Growth factor half-lives (hours)
    const halfLives: Record<GrowthFactorType, number> = {
      VEGF: 3,
      FGF: 5,
      'TGF-β': 0.05,
      BMP: 0.12,
      PDGF: 3,
      IGF: 13,
      EGF: 9,
      BDNF: 0.02,
      NGF: 2.5,
      HGF: 0.08,
    };

    const halfLife = halfLives[type];

    // Calculate total dose
    let totalDoseMultiplier = 1;

    if (deliveryMethod === 'bolus') {
      // Need to account for rapid degradation
      totalDoseMultiplier = (duration * 24) / halfLife;
    } else if (deliveryMethod === 'scaffold') {
      // Sustained release, more efficient
      totalDoseMultiplier = duration * 0.5;
    } else if (deliveryMethod === 'microsphere') {
      // Controlled release, most efficient
      totalDoseMultiplier = duration * 0.3;
    } else {
      // Gene delivery, continuous production
      totalDoseMultiplier = 0.1;
    }

    const totalDose = (targetConcentration * tissueVolume * totalDoseMultiplier) / 1000; // Convert to μg

    // Dosing schedule
    const schedule =
      deliveryMethod === 'bolus'
        ? {
            frequency: `Every ${Math.ceil(halfLife * 3)} hours`,
            dose: targetConcentration * tissueVolume,
            unit: 'ng',
          }
        : deliveryMethod === 'gene-delivery'
          ? {
              frequency: 'Single administration',
              dose: totalDose,
              unit: 'μg plasmid/vector',
            }
          : {
              frequency: 'Sustained release',
              dose: totalDose,
              unit: 'μg total',
            };

    // Release kinetics
    const releaseKinetics = {
      half_life: halfLife,
      peakConcentration:
        deliveryMethod === 'bolus' ? targetConcentration * 2 : targetConcentration * 1.2,
      sustainedRelease: deliveryMethod !== 'bolus',
    };

    // Efficacy
    let efficacy = 0.6;
    if (deliveryMethod === 'scaffold' || deliveryMethod === 'microsphere') efficacy = 0.8;
    if (deliveryMethod === 'gene-delivery') efficacy = 0.9;

    // Synergy bonus
    const synergyBonus = combination.length > 0 ? combination.length * 0.15 : 0;

    // Cost estimate ($/μg)
    const costPerMicrogram: Record<GrowthFactorType, number> = {
      VEGF: 50,
      FGF: 40,
      'TGF-β': 60,
      BMP: 100,
      PDGF: 55,
      IGF: 45,
      EGF: 30,
      BDNF: 80,
      NGF: 90,
      HGF: 70,
    };

    const costEstimate = totalDose * costPerMicrogram[type];

    return {
      totalDose,
      schedule,
      releaseKinetics,
      efficacy,
      synergyBonus,
      costEstimate,
    };
  }

  /**
   * Validate cell characterization for clinical use
   *
   * @param characterization - Cell characterization data
   * @returns Validation result with recommendations
   */
  validateCellCharacterization(characterization: CellCharacterization): {
    valid: boolean;
    errors: string[];
    warnings: string[];
  } {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Viability check
    if (characterization.viability < BIO_CONSTANTS.MIN_VIABILITY) {
      errors.push(
        `Viability too low: ${(characterization.viability * 100).toFixed(1)}% (minimum: ${BIO_CONSTANTS.MIN_VIABILITY * 100}%)`
      );
    } else if (characterization.viability < 0.8) {
      warnings.push('Viability is acceptable but could be improved');
    }

    // Mycoplasma check
    if (characterization.mycoplasma === 'positive') {
      errors.push('Mycoplasma contamination detected - cells cannot be used');
    } else if (characterization.mycoplasma === 'pending') {
      warnings.push('Mycoplasma test pending - do not use until confirmed negative');
    }

    // Sterility check
    if (characterization.sterility === 'fail') {
      errors.push('Sterility test failed - contamination detected');
    }

    // Endotoxin check
    if (characterization.endotoxin && characterization.endotoxin > BIO_CONSTANTS.MAX_ENDOTOXIN) {
      errors.push(
        `Endotoxin level too high: ${characterization.endotoxin} EU/ml (maximum: ${BIO_CONSTANTS.MAX_ENDOTOXIN})`
      );
    }

    // Marker expression check
    const positiveMarkers = characterization.markers.filter(
      (m) => m.expectedPositive && m.expression >= 0.9
    );
    const negativeMarkers = characterization.markers.filter(
      (m) => !m.expectedPositive && m.expression < 0.02
    );

    if (positiveMarkers.length < 3) {
      warnings.push('Insufficient positive marker expression');
    }

    if (characterization.cellType === 'iPSC' || characterization.cellType === 'ESC') {
      const pluripotencyMarkers = characterization.markers.filter((m) =>
        BIO_CONSTANTS.PLURIPOTENCY_MARKERS.includes(m.name)
      );
      if (pluripotencyMarkers.length < 3) {
        errors.push('Insufficient pluripotency markers for stem cells');
      }
    }

    // Passage number check
    if (characterization.passage > 30 && characterization.cellType.includes('SC')) {
      warnings.push('High passage number may affect cell quality');
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate regeneration rate (standalone function)
 */
export function calculateRegenerationRate(
  params: RegenerationRequest
): RegenerationResponse {
  const sdk = new RegenerativeMedicineSDK();
  return sdk.calculateRegenerationRate(params);
}

/**
 * Assess cell survival (standalone function)
 */
export function assessCellSurvival(params: SurvivalRequest): SurvivalResponse {
  const sdk = new RegenerativeMedicineSDK();
  return sdk.assessCellSurvival(params);
}

/**
 * Design scaffold (standalone function)
 */
export function designScaffold(params: ScaffoldRequest): ScaffoldResponse {
  const sdk = new RegenerativeMedicineSDK();
  return sdk.designScaffold(params);
}

/**
 * Optimize growth factors (standalone function)
 */
export function optimizeGrowthFactors(params: GrowthFactorRequest): GrowthFactorResponse {
  const sdk = new RegenerativeMedicineSDK();
  return sdk.optimizeGrowthFactors(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { RegenerativeMedicineSDK };
export default RegenerativeMedicineSDK;
