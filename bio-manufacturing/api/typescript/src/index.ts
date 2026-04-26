/**
 * WIA-BIO-015: Bio-Manufacturing SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bio-Manufacturing Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for bio-manufacturing including:
 * - Yield and productivity calculations
 * - Bioreactor monitoring
 * - Fermentation optimization
 * - Purification design
 * - Quality control
 */

import {
  YieldParameters,
  YieldResult,
  ProductivityParameters,
  ProductivityResult,
  BioreactorStatus,
  MonitoringResult,
  PurificationDesign,
  PurificationProtocol,
  ChromatographyStep,
  GrowthKinetics,
  ValidationParameters,
  ValidationResult,
  CellLine,
  FermentationParameters,
  OptimizationResult,
  BIO_CONSTANTS,
  BioErrorCode,
  BioManufacturingError,
  ParameterTrend,
  ProductType,
  CriticalQualityAttribute,
  ProcessCapability,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-015 Bio-Manufacturing SDK
 */
export class BioManufacturingSDK {
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
   * Calculate product yield
   *
   * @param params - Yield calculation parameters
   * @returns Yield result with efficiency assessment
   */
  calculateYield(params: YieldParameters): YieldResult {
    const {
      productConcentration,
      finalVolume,
      substrateConcentration,
      initialVolume,
      feedVolume = 0,
      feedSubstrate = 0,
    } = params;

    // Validate inputs
    if (productConcentration < 0 || finalVolume <= 0) {
      throw new BioManufacturingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Product concentration and volume must be positive'
      );
    }

    if (substrateConcentration <= 0 || initialVolume <= 0) {
      throw new BioManufacturingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Substrate concentration and volume must be positive'
      );
    }

    // Calculate total product
    const totalProduct = productConcentration * finalVolume;

    // Calculate total substrate (initial + feed)
    const totalSubstrate =
      substrateConcentration * initialVolume + feedSubstrate * feedVolume;

    // Calculate yield
    const yieldValue = totalProduct / totalSubstrate;

    // Calculate efficiency percentage
    const efficiency = yieldValue * 100;

    // Assess performance
    let performance: YieldResult['performance'];
    if (yieldValue > 0.15) {
      performance = 'excellent';
    } else if (yieldValue > 0.10) {
      performance = 'good';
    } else if (yieldValue > 0.05) {
      performance = 'fair';
    } else {
      performance = 'poor';
    }

    return {
      yield: yieldValue,
      totalProduct,
      totalSubstrate,
      efficiency,
      performance,
      yieldFormatted: `${(efficiency).toFixed(2)}% (${yieldValue.toFixed(4)})`,
    };
  }

  /**
   * Calculate volumetric productivity
   *
   * @param params - Productivity calculation parameters
   * @returns Productivity result with performance assessment
   */
  calculateProductivity(params: ProductivityParameters): ProductivityResult {
    const {
      finalConcentration,
      initialConcentration,
      cultureTime,
      volume,
    } = params;

    // Validate inputs
    if (cultureTime <= 0) {
      throw new BioManufacturingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Culture time must be positive'
      );
    }

    if (finalConcentration < initialConcentration) {
      throw new BioManufacturingError(
        BioErrorCode.INVALID_PARAMETERS,
        'Final concentration cannot be less than initial'
      );
    }

    // Calculate volumetric productivity (g/L/h)
    const value = (finalConcentration - initialConcentration) / cultureTime;

    // Convert to per day
    const perDay = value * 24;

    // Calculate total product if volume provided
    const totalProduct = volume ? finalConcentration * volume : undefined;

    // Assess performance for mAb (typical benchmark)
    let performance: ProductivityResult['performance'];
    if (perDay > 0.8) {
      performance = 'excellent';
    } else if (perDay > 0.5) {
      performance = 'good';
    } else if (perDay > 0.3) {
      performance = 'fair';
    } else {
      performance = 'poor';
    }

    return {
      value,
      perDay,
      totalProduct,
      performance,
      formatted: `${perDay.toFixed(3)} g/L/day (${value.toFixed(4)} g/L/h)`,
    };
  }

  /**
   * Monitor bioreactor status
   *
   * @param status - Current bioreactor status
   * @returns Monitoring result with warnings and recommendations
   */
  monitorBioreactor(status: BioreactorStatus): MonitoringResult {
    const warnings: string[] = [];
    const criticalAlarms: string[] = [];
    const recommendations: string[] = [];
    const trends: ParameterTrend[] = [];

    const { conditions, metrics } = status;

    // Check viability
    if (metrics.viability < BIO_CONSTANTS.MIN_VIABILITY) {
      criticalAlarms.push(
        `Low viability: ${metrics.viability.toFixed(1)}% (target: >${BIO_CONSTANTS.MIN_VIABILITY}%)`
      );
      recommendations.push('Consider harvesting soon or investigating culture conditions');
    } else if (metrics.viability < 85) {
      warnings.push(`Viability declining: ${metrics.viability.toFixed(1)}%`);
    }

    // Check pH
    if (conditions.pH < 6.8 || conditions.pH > 7.4) {
      criticalAlarms.push(
        `pH out of range: ${conditions.pH.toFixed(2)} (target: 6.9-7.4)`
      );
      recommendations.push('Check pH control and calibration');
    } else if (conditions.pH < 6.9 || conditions.pH > 7.3) {
      warnings.push(`pH near limits: ${conditions.pH.toFixed(2)}`);
    }

    // Check temperature
    if (conditions.temperature < 35 || conditions.temperature > 38) {
      criticalAlarms.push(
        `Temperature excursion: ${conditions.temperature.toFixed(1)}°C (target: 36-37°C)`
      );
      recommendations.push('Check temperature control system');
    } else if (conditions.temperature < 36 || conditions.temperature > 37.5) {
      warnings.push(`Temperature suboptimal: ${conditions.temperature.toFixed(1)}°C`);
    }

    // Check dissolved oxygen
    if (conditions.dissolvedOxygen < 20 || conditions.dissolvedOxygen > 60) {
      criticalAlarms.push(
        `DO out of range: ${conditions.dissolvedOxygen.toFixed(1)}% (target: 30-50%)`
      );
      recommendations.push('Adjust agitation or aeration rate');
    } else if (conditions.dissolvedOxygen < 30 || conditions.dissolvedOxygen > 50) {
      warnings.push(`DO suboptimal: ${conditions.dissolvedOxygen.toFixed(1)}%`);
    }

    // Check lactate if available
    if (metrics.lactate !== undefined) {
      if (metrics.lactate > 40) {
        criticalAlarms.push(
          `High lactate: ${metrics.lactate.toFixed(1)} mM (limit: <40 mM)`
        );
        recommendations.push('Reduce glucose feed rate or implement feeding strategy');
      } else if (metrics.lactate > 30) {
        warnings.push(`Lactate accumulating: ${metrics.lactate.toFixed(1)} mM`);
      }
    }

    // Check ammonia if available
    if (metrics.ammonia !== undefined) {
      if (metrics.ammonia > 8) {
        warnings.push(`Elevated ammonia: ${metrics.ammonia.toFixed(1)} mM (caution: >8 mM)`);
        recommendations.push('Monitor closely; may affect product quality');
      }
    }

    // Generate trends
    trends.push({
      parameter: 'Cell Density',
      current: metrics.cellDensity,
      trend: 'stable',
      rateOfChange: 0,
      alert: 'none',
    });

    trends.push({
      parameter: 'Viability',
      current: metrics.viability,
      trend: metrics.viability < 90 ? 'decreasing' : 'stable',
      rateOfChange: metrics.viability < 90 ? -0.5 : 0,
      alert: metrics.viability < BIO_CONSTANTS.MIN_VIABILITY ? 'critical' : 'none',
    });

    // Determine overall status
    let overallStatus: MonitoringResult['status'];
    if (criticalAlarms.length > 0) {
      overallStatus = 'critical';
    } else if (warnings.length > 2) {
      overallStatus = 'warning';
    } else if (warnings.length > 0) {
      overallStatus = 'acceptable';
    } else {
      overallStatus = 'optimal';
    }

    return {
      reactorId: status.reactorId,
      timestamp: status.timestamp,
      status: overallStatus,
      warnings,
      criticalAlarms,
      recommendations,
      trends,
    };
  }

  /**
   * Design purification protocol
   *
   * @param design - Purification design parameters
   * @returns Purification protocol with steps and estimates
   */
  designPurification(design: PurificationDesign): PurificationProtocol {
    const { productType, scale, startingConcentration, targetPurity, targetRecovery } =
      design;

    const steps: ChromatographyStep[] = [];
    let cumulativeRecovery = 1.0;
    let cumulativePurity = 30; // Assume 30% starting purity

    // Step 1: Capture (Protein A for antibodies, IMAC for His-tagged)
    if (productType === 'antibody') {
      const proteinARecovery = 0.95;
      const proteinAPurity = 95;

      steps.push({
        stepNumber: 1,
        name: 'Protein A Affinity Chromatography',
        type: 'protein-a',
        resinVolume: scale * startingConcentration / 40, // 40 g/L DBC
        bindingCapacity: 40,
        loadVolume: scale,
        recovery: proteinARecovery * 100,
        purity: proteinAPurity,
        residenceTime: 6,
        buffers: {
          equilibration: 'PBS pH 7.4',
          load: 'Harvest (clarified)',
          wash: 'PBS pH 7.4',
          elution: 'Glycine pH 3.5',
        },
      });

      cumulativeRecovery *= proteinARecovery;
      cumulativePurity = proteinAPurity;
    } else {
      // Generic capture for other proteins
      steps.push({
        stepNumber: 1,
        name: 'Cation Exchange Chromatography (Capture)',
        type: 'cation-exchange',
        resinVolume: scale * startingConcentration / 50,
        bindingCapacity: 50,
        loadVolume: scale,
        recovery: 90,
        purity: 80,
        residenceTime: 4,
      });

      cumulativeRecovery *= 0.90;
      cumulativePurity = 80;
    }

    // Step 2: Intermediate purification (Ion Exchange)
    if (productType === 'antibody') {
      const iexRecovery = 0.92;
      const iexPurity = 98;

      steps.push({
        stepNumber: 2,
        name: 'Anion Exchange Chromatography (Polishing)',
        type: 'anion-exchange',
        resinVolume: scale * startingConcentration * 0.95 / 60,
        bindingCapacity: 60,
        loadVolume: scale * 0.95,
        recovery: iexRecovery * 100,
        purity: iexPurity,
        residenceTime: 4,
        buffers: {
          equilibration: 'Tris pH 8.0',
          load: 'Protein A eluate (neutralized)',
          wash: 'Tris pH 8.0',
          elution: 'Flow-through collection',
        },
      });

      cumulativeRecovery *= iexRecovery;
      cumulativePurity = iexPurity;
    }

    // Step 3: Polishing (HIC or Size Exclusion)
    const hicRecovery = 0.90;
    const hicPurity = 99;

    steps.push({
      stepNumber: steps.length + 1,
      name: 'Hydrophobic Interaction Chromatography (Polishing)',
      type: 'hydrophobic',
      resinVolume: scale * startingConcentration * cumulativeRecovery / 40,
      bindingCapacity: 40,
      loadVolume: scale * cumulativeRecovery,
      recovery: hicRecovery * 100,
      purity: hicPurity,
      residenceTime: 5,
      buffers: {
        equilibration: '1.0 M Ammonium Sulfate',
        load: 'Intermediate pool + salt',
        wash: '1.0 M Ammonium Sulfate',
        elution: 'Decreasing salt gradient',
      },
    });

    cumulativeRecovery *= hicRecovery;
    cumulativePurity = hicPurity;

    // Calculate duration (rough estimate)
    const duration =
      steps.length * 8 + // 8 hours per chromatography step
      2 + // Viral inactivation
      4; // Buffer exchange and formulation

    // Calculate cost (rough estimate)
    const costPerGram =
      productType === 'antibody'
        ? 50 + steps.length * 20 // Protein A is expensive
        : 30 + steps.length * 15;

    return {
      id: `PURIF-${Date.now()}`,
      name: `${productType} Purification Protocol`,
      productType,
      steps,
      viralInactivation: true,
      overallRecovery: cumulativeRecovery * 100,
      finalPurity: cumulativePurity,
      duration,
      costPerGram,
    };
  }

  /**
   * Calculate growth kinetics
   *
   * @param cellLine - Cell line information
   * @param conditions - Culture conditions
   * @returns Growth kinetics parameters
   */
  calculateGrowthKinetics(cellLine: CellLine): GrowthKinetics {
    const specificGrowthRate = cellLine.growthRate;
    const doublingTime = Math.log(2) / specificGrowthRate;

    return {
      specificGrowthRate,
      doublingTime,
      maxGrowthRate: specificGrowthRate * 1.2, // 20% higher under optimal conditions
      lagTime: cellLine.system === 'CHO' ? 12 : 2, // Mammalian vs microbial
    };
  }

  /**
   * Validate batch
   *
   * @param params - Validation parameters
   * @returns Validation result
   */
  validateBatch(params: ValidationParameters): ValidationResult {
    const {
      targetTiter,
      targetViability,
      maxLactate,
      maxAmmonia,
      gmpRequired,
    } = params;

    const errors: string[] = [];
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Titer check
    if (targetTiter < 1.0) {
      errors.push(`Titer too low: ${targetTiter.toFixed(2)} g/L (minimum: 1.0 g/L)`);
    } else if (targetTiter < 2.0) {
      warnings.push(`Low titer: ${targetTiter.toFixed(2)} g/L`);
      recommendations.push('Consider process optimization to improve titer');
    }

    // Viability check
    if (targetViability < BIO_CONSTANTS.MIN_VIABILITY) {
      errors.push(
        `Viability too low: ${targetViability.toFixed(1)}% (minimum: ${BIO_CONSTANTS.MIN_VIABILITY}%)`
      );
    }

    // Lactate check
    if (maxLactate > 40) {
      warnings.push(`High lactate: ${maxLactate.toFixed(1)} mM (limit: <40 mM)`);
      recommendations.push('Review feeding strategy to reduce lactate');
    }

    // Ammonia check
    if (maxAmmonia > 8) {
      warnings.push(`High ammonia: ${maxAmmonia.toFixed(1)} mM (caution: >8 mM)`);
    }

    // Compliance checks
    const compliance = {
      gmp: gmpRequired ? errors.length === 0 : true,
      cqas: errors.length === 0,
      sterility: true, // Would require actual sterility test
      documentation: gmpRequired,
    };

    // Quality score
    let qualityScore = 100;
    qualityScore -= errors.length * 20;
    qualityScore -= warnings.length * 5;
    qualityScore = Math.max(0, qualityScore);

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      compliance,
      qualityScore,
      recommendations,
    };
  }

  /**
   * Optimize fermentation
   *
   * @param params - Fermentation parameters
   * @returns Optimization recommendations
   */
  optimizeFermentation(params: FermentationParameters): OptimizationResult {
    const { initialCellDensity, cultureTime, volume } = params;

    // Simple optimization based on growth phase
    const optimalGrowthRate = 0.03; // h⁻¹ for CHO cells
    const optimalDensity = 5e6; // cells/mL target

    // Calculate improvement potential
    const currentFinalDensity = initialCellDensity * Math.exp(optimalGrowthRate * cultureTime);
    const improvement = ((optimalDensity - currentFinalDensity) / currentFinalDensity) * 100;

    return {
      id: `OPT-${Date.now()}`,
      parameters: {
        optimalInitialDensity: 3e5,
        optimalGlucose: 6.0, // g/L
        optimalGlutamine: 4.0, // mM
        optimalTemperature: 37.0, // °C
        optimalPH: 7.1,
      },
      predictedResponse: optimalDensity,
      confidenceInterval: [optimalDensity * 0.9, optimalDensity * 1.1],
      improvement: Math.max(0, improvement),
      recommendation:
        'Implement fed-batch feeding with exponential glucose delivery and temperature shift at day 7',
    };
  }

  /**
   * Calculate process capability
   *
   * @param data - Historical data points
   * @param lsl - Lower specification limit
   * @param usl - Upper specification limit
   * @returns Process capability metrics
   */
  calculateProcessCapability(
    attributeName: string,
    data: number[],
    lsl: number,
    usl: number
  ): ProcessCapability {
    // Calculate mean and standard deviation
    const mean = data.reduce((sum, val) => sum + val, 0) / data.length;
    const variance =
      data.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / (data.length - 1);
    const stdDev = Math.sqrt(variance);

    // Calculate Cp
    const cp = (usl - lsl) / (6 * stdDev);

    // Calculate Cpk
    const cpupper = (usl - mean) / (3 * stdDev);
    const cplower = (mean - lsl) / (3 * stdDev);
    const cpk = Math.min(cpupper, cplower);

    // Assess capability
    let assessment: ProcessCapability['assessment'];
    let recommendation: string | undefined;

    if (cpk >= 1.33) {
      assessment = 'capable';
    } else if (cpk >= 1.0) {
      assessment = 'marginally-capable';
      recommendation = 'Consider process improvement to increase capability';
    } else {
      assessment = 'not-capable';
      recommendation = 'Process improvement required - high risk of out-of-spec product';
    }

    return {
      attribute: attributeName,
      mean,
      stdDev,
      cp,
      cpk,
      assessment,
      recommendation,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate product yield (standalone function)
 */
export function calculateYield(params: YieldParameters): YieldResult {
  const sdk = new BioManufacturingSDK();
  return sdk.calculateYield(params);
}

/**
 * Calculate volumetric productivity (standalone function)
 */
export function calculateProductivity(
  params: ProductivityParameters
): ProductivityResult {
  const sdk = new BioManufacturingSDK();
  return sdk.calculateProductivity(params);
}

/**
 * Monitor bioreactor (standalone function)
 */
export function monitorBioreactor(status: BioreactorStatus): MonitoringResult {
  const sdk = new BioManufacturingSDK();
  return sdk.monitorBioreactor(status);
}

/**
 * Design purification protocol (standalone function)
 */
export function designPurification(design: PurificationDesign): PurificationProtocol {
  const sdk = new BioManufacturingSDK();
  return sdk.designPurification(design);
}

/**
 * Validate batch (standalone function)
 */
export function validateBatch(params: ValidationParameters): ValidationResult {
  const sdk = new BioManufacturingSDK();
  return sdk.validateBatch(params);
}

/**
 * Optimize fermentation (standalone function)
 */
export function optimizeFermentation(
  params: FermentationParameters
): OptimizationResult {
  const sdk = new BioManufacturingSDK();
  return sdk.optimizeFermentation(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BioManufacturingSDK };
export default BioManufacturingSDK;
