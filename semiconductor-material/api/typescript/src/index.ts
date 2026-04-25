/**
 * WIA-SEMI-018 Semiconductor Material Standard SDK
 * @packageDocumentation
 */

export * from './types';

import {
  WaferSpecification,
  PhotoresistSpecification,
  GasSpecification,
  TestResult,
  MaterialLot,
  Supplier,
  SPCData,
  Defect,
  ValidationResult,
  SDKConfig,
} from './types';

/**
 * Main SDK class for WIA-SEMI-018 Semiconductor Material Standard
 */
export class SemiconductorMaterialSDK {
  private config: SDKConfig;

  /**
   * Initialize SDK with configuration
   * @param config - SDK configuration options
   */
  constructor(config: SDKConfig = {}) {
    this.config = {
      debug: false,
      timeout: 30000,
      ...config,
    };

    if (this.config.debug) {
      console.log('[WIA-SEMI-018] SDK initialized with config:', this.config);
    }
  }

  /**
   * Validate silicon wafer against WIA-SEMI-018 specifications
   * @param wafer - Wafer specification to validate
   * @returns Validation result
   */
  validateWafer(wafer: WaferSpecification): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Diameter validation (300mm standard)
    if (Math.abs(wafer.diameter - 300) > 0.2) {
      errors.push(`Diameter ${wafer.diameter}mm exceeds tolerance (300 ± 0.2mm)`);
    }

    // Thickness validation
    if (wafer.thickness < 765 || wafer.thickness > 785) {
      errors.push(`Thickness ${wafer.thickness}µm out of range (775 ± 10µm)`);
    }

    // TTV validation
    if (wafer.ttv >= 2.0) {
      errors.push(`TTV ${wafer.ttv}µm exceeds limit (<2.0µm)`);
    }

    // Bow validation
    if (wafer.bow >= 40) {
      errors.push(`Bow ${wafer.bow}µm exceeds limit (<40µm)`);
    }

    // Warp validation
    if (wafer.warp >= 50) {
      errors.push(`Warp ${wafer.warp}µm exceeds limit (<50µm)`);
    }

    // Defect density validation
    if (wafer.defectDensity >= 0.1) {
      errors.push(
        `Defect density ${wafer.defectDensity}/cm² exceeds limit (<0.1/cm²)`
      );
    } else if (wafer.defectDensity >= 0.08) {
      warnings.push(
        `Defect density ${wafer.defectDensity}/cm² approaching limit (<0.1/cm²)`
      );
    }

    // Surface roughness validation
    if (wafer.surfaceRoughness >= 0.2) {
      errors.push(
        `Surface roughness Ra ${wafer.surfaceRoughness}nm exceeds limit (<0.2nm)`
      );
    }

    // Purity validation (11-9s minimum)
    if (wafer.purity < 11) {
      errors.push(`Silicon purity ${wafer.purity}-9s below minimum (11-9s)`);
    }

    // Orientation validation
    const validOrientations = ['<100>', '<111>', '<110>'];
    if (!validOrientations.includes(wafer.orientation)) {
      errors.push(
        `Invalid crystal orientation "${wafer.orientation}". Must be one of: ${validOrientations.join(', ')}`
      );
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Validate photoresist against WIA-SEMI-018 specifications
   * @param resist - Photoresist specification to validate
   * @returns Validation result
   */
  validatePhotoresist(resist: PhotoresistSpecification): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Type-specific validations
    switch (resist.type) {
      case 'EUV':
        // EUV specific validations
        if (resist.sensitivity < 15 || resist.sensitivity > 25) {
          errors.push(
            `EUV sensitivity ${resist.sensitivity}mJ/cm² out of range (15-25mJ/cm²)`
          );
        }
        if (resist.ler >= 1.5) {
          errors.push(`EUV LER ${resist.ler}nm exceeds limit (<1.5nm)`);
        }
        if (resist.resolution > 8) {
          warnings.push(
            `EUV resolution ${resist.resolution}nm may not meet High-NA requirements (≤8nm)`
          );
        }
        break;

      case 'ArF':
        // ArF specific validations
        if (resist.sensitivity < 25 || resist.sensitivity > 35) {
          errors.push(
            `ArF sensitivity ${resist.sensitivity}mJ/cm² out of range (25-35mJ/cm²)`
          );
        }
        if (resist.ler >= 3.0) {
          errors.push(`ArF LER ${resist.ler}nm exceeds limit (<3.0nm)`);
        }
        break;

      case 'KrF':
        // KrF specific validations
        if (resist.sensitivity < 15 || resist.sensitivity > 30) {
          errors.push(
            `KrF sensitivity ${resist.sensitivity}mJ/cm² out of range (15-30mJ/cm²)`
          );
        }
        break;
    }

    // Contrast validation
    if (resist.contrast < 5.0) {
      errors.push(`Contrast ${resist.contrast} below minimum (≥5.0)`);
    }

    // Defect density validation
    if (resist.defectDensity >= 0.01) {
      errors.push(
        `Defect density ${resist.defectDensity}/cm² exceeds limit (<0.01/cm²)`
      );
    }

    // Particle count validation
    if (resist.particleCount >= 10) {
      errors.push(
        `Particle count ${resist.particleCount}/mL exceeds limit (<10/mL)`
      );
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Validate specialty gas against WIA-SEMI-018 specifications
   * @param gas - Gas specification to validate
   * @returns Validation result
   */
  validateGas(gas: GasSpecification): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Purity validation (minimum 5N for most, 6N for critical)
    const minPurity = ['PH₃', 'AsH₃', 'SiH₄', 'N₂', 'H₂', 'Ar'].includes(
      gas.formula
    )
      ? 6
      : 5;

    if (gas.purity < minPurity) {
      errors.push(
        `Gas purity ${gas.purity}N below minimum (${minPurity}N for ${gas.name})`
      );
    }

    // Impurity validation
    const criticalImpurities: Record<string, number> = {
      PH₃: { AsH₃: 1, H₂O: 1, O₂: 0.5 },
      SiH₄: { B₂H₆: 0.1, PH₃: 0.1, AsH₃: 0.01, H₂O: 0.5 },
      B₂H₆: { SiH₄: 50, PH₃: 10, H₂O: 1, O₂: 0.5 },
    };

    if (criticalImpurities[gas.formula]) {
      const limits = criticalImpurities[gas.formula];
      for (const [impurity, limit] of Object.entries(limits)) {
        if (gas.impurities[impurity] && gas.impurities[impurity] > limit) {
          errors.push(
            `${impurity} impurity ${gas.impurities[impurity]}ppm exceeds limit (<${limit}ppm)`
          );
        }
      }
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Calculate process capability index (Cpk)
   * @param data - Array of measurement values
   * @param lsl - Lower specification limit
   * @param usl - Upper specification limit
   * @returns Cpk value
   */
  calculateCpk(data: number[], lsl: number, usl: number): number {
    if (data.length === 0) {
      throw new Error('Data array cannot be empty');
    }

    // Calculate mean
    const mean = data.reduce((sum, val) => sum + val, 0) / data.length;

    // Calculate standard deviation
    const variance =
      data.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) /
      (data.length - 1);
    const stdDev = Math.sqrt(variance);

    // Calculate Cpk
    const cpkUpper = (usl - mean) / (3 * stdDev);
    const cpkLower = (mean - lsl) / (3 * stdDev);

    return Math.min(cpkUpper, cpkLower);
  }

  /**
   * Calculate defect density from particle count
   * @param waferDiameter - Wafer diameter in mm
   * @param totalDefects - Total number of defects
   * @param inspectionArea - Percentage of wafer inspected (default 100%)
   * @returns Defect density in defects/cm²
   */
  calculateDefectDensity(
    waferDiameter: number,
    totalDefects: number,
    inspectionArea: number = 100
  ): number {
    // Calculate wafer area in cm²
    const radiusCm = (waferDiameter / 2) / 10; // Convert mm to cm
    const totalAreaCm2 = Math.PI * radiusCm * radiusCm;

    // Calculate inspected area
    const inspectedAreaCm2 = totalAreaCm2 * (inspectionArea / 100);

    // Calculate defect density
    return totalDefects / inspectedAreaCm2;
  }

  /**
   * Evaluate test result against specification
   * @param result - Test result to evaluate
   * @returns Whether test passed
   */
  evaluateTestResult(result: TestResult): boolean {
    return (
      result.measuredValue >= result.lowerLimit &&
      result.measuredValue <= result.upperLimit
    );
  }

  /**
   * Calculate supplier scorecard
   * @param defectRatePpm - Defect rate in parts per million
   * @param ontimeDeliveryPercent - On-time delivery percentage
   * @param responsiveness - Responsiveness rating (1-5)
   * @param priceCompetitiveness - Price competitiveness rating (1-5)
   * @returns Overall score (0-100)
   */
  calculateSupplierScore(
    defectRatePpm: number,
    ontimeDeliveryPercent: number,
    responsiveness: number,
    priceCompetitiveness: number
  ): number {
    // Quality score (40 points)
    let qualityScore = 0;
    if (defectRatePpm < 10) qualityScore = 40;
    else if (defectRatePpm < 50) qualityScore = 30;
    else if (defectRatePpm < 100) qualityScore = 20;
    else qualityScore = 10;

    // Delivery score (30 points)
    let deliveryScore = 0;
    if (ontimeDeliveryPercent >= 99) deliveryScore = 30;
    else if (ontimeDeliveryPercent >= 98) deliveryScore = 25;
    else if (ontimeDeliveryPercent >= 95) deliveryScore = 20;
    else deliveryScore = 10;

    // Service score (20 points) - based on responsiveness
    const serviceScore = (responsiveness / 5) * 20;

    // Cost score (10 points) - based on price competitiveness
    const costScore = (priceCompetitiveness / 5) * 10;

    return qualityScore + deliveryScore + serviceScore + costScore;
  }

  /**
   * Generate SPC control limits
   * @param data - Historical data points
   * @returns SPC data with control limits
   */
  generateSPCLimits(data: number[]): Omit<SPCData, 'parameter' | 'timestamps'> {
    if (data.length < 25) {
      throw new Error('Minimum 25 data points required for SPC');
    }

    // Calculate mean
    const mean = data.reduce((sum, val) => sum + val, 0) / data.length;

    // Calculate standard deviation
    const variance =
      data.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) /
      (data.length - 1);
    const stdDev = Math.sqrt(variance);

    // Calculate control limits (±3σ)
    const ucl = mean + 3 * stdDev;
    const lcl = mean - 3 * stdDev;

    // Check if process is in control
    const outOfControl = data.some((val) => val > ucl || val < lcl);

    return {
      mean,
      stdDev,
      ucl,
      lcl,
      cpk: 0, // Calculate separately with LSL/USL
      inControl: !outOfControl,
      dataPoints: data,
    };
  }

  /**
   * Convert purity percentage to number of nines
   * @param purityPercent - Purity as percentage (e.g., 99.999999999)
   * @returns Number of nines (e.g., 11 for 11-9s)
   */
  percentageToNines(purityPercent: number): number {
    if (purityPercent >= 100 || purityPercent < 0) {
      throw new Error('Purity percentage must be between 0 and 100');
    }

    const impurityFraction = 1 - purityPercent / 100;
    const nines = Math.floor(-Math.log10(impurityFraction));

    return nines;
  }

  /**
   * Convert number of nines to purity percentage
   * @param nines - Number of nines (e.g., 11 for 11-9s)
   * @returns Purity percentage
   */
  ninesToPercentage(nines: number): number {
    if (nines < 0) {
      throw new Error('Number of nines must be positive');
    }

    const impurityFraction = Math.pow(10, -nines);
    const purityPercent = (1 - impurityFraction) * 100;

    return purityPercent;
  }

  /**
   * Get SDK version
   * @returns SDK version string
   */
  getVersion(): string {
    return '1.0.0';
  }

  /**
   * Get standard version
   * @returns WIA-SEMI-018 standard version
   */
  getStandardVersion(): string {
    return 'WIA-SEMI-018 v1.0';
  }
}

// Export singleton instance
export const semiconductorMaterial = new SemiconductorMaterialSDK();

// Export class for custom instances
export default SemiconductorMaterialSDK;
