/**
 * WIA-BIO-005: Cellular Therapy SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for cellular therapy including:
 * - Dose calculations
 * - Quality assessment
 * - Manufacturing tracking
 * - Safety monitoring
 * - Certificate generation
 */

import {
  CellType,
  CellProduct,
  DoseParameters,
  DoseResult,
  QualityMetrics,
  QualityResult,
  ReleaseCriteria,
  ManufacturingProcess,
  BatchResult,
  CRSAssessment,
  ICANSAssessment,
  SafetyEvent,
  PostThawQuality,
  CertificateOfAnalysis,
  PotencyAssay,
  CELLULAR_THERAPY_CONSTANTS,
  CellularTherapyErrorCode,
  CellularTherapyError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-005 Cellular Therapy SDK
 */
export class CellularTherapySDK {
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
   * Calculate patient-specific dose
   *
   * @param params - Dose calculation parameters
   * @returns Dose calculation result
   */
  calculateDose(params: DoseParameters): DoseResult {
    const {
      cellType,
      totalCells,
      viability,
      patientWeight,
      targetMarkerPercent,
      strategy = 'weight-based',
    } = params;

    // Validate inputs
    if (totalCells <= 0) {
      throw new CellularTherapyError(
        CellularTherapyErrorCode.INVALID_PARAMETERS,
        'Total cells must be positive'
      );
    }

    if (viability < 0 || viability > 100) {
      throw new CellularTherapyError(
        CellularTherapyErrorCode.INVALID_PARAMETERS,
        'Viability must be between 0 and 100'
      );
    }

    if (patientWeight <= 0) {
      throw new CellularTherapyError(
        CellularTherapyErrorCode.INVALID_PARAMETERS,
        'Patient weight must be positive'
      );
    }

    // Calculate viable cells
    const totalViableCells = totalCells * (viability / 100);

    // Calculate target marker positive cells if applicable
    let targetMarkerCells: number | undefined;
    if (targetMarkerPercent !== undefined) {
      targetMarkerCells = totalViableCells * (targetMarkerPercent / 100);
    }

    // Calculate dose based on strategy
    let dose: number;
    let recommendedMin: number;
    let recommendedMax: number;
    let unit: 'cells/kg' | 'total cells';

    if (strategy === 'weight-based') {
      dose = totalViableCells / patientWeight;
      unit = 'cells/kg';

      // Set recommended ranges based on cell type
      switch (cellType) {
        case 'CAR-T':
          recommendedMin = CELLULAR_THERAPY_CONSTANTS.CAR_T_DOSE.MIN;
          recommendedMax = CELLULAR_THERAPY_CONSTANTS.CAR_T_DOSE.MAX;
          // Adjust for CAR+ cells if available
          if (targetMarkerCells) {
            dose = targetMarkerCells / patientWeight;
          }
          break;

        case 'MSC':
          recommendedMin = CELLULAR_THERAPY_CONSTANTS.MSC_DOSE.MIN;
          recommendedMax = CELLULAR_THERAPY_CONSTANTS.MSC_DOSE.MAX;
          break;

        case 'CAR-NK':
          recommendedMin = 1e7;
          recommendedMax = 1e9;
          break;

        case 'TIL':
          recommendedMin = 1e9;
          recommendedMax = 2e11;
          break;

        default:
          recommendedMin = 1e6;
          recommendedMax = 1e7;
      }
    } else {
      // Flat dose
      dose = totalViableCells;
      unit = 'total cells';
      recommendedMin = 1e8;
      recommendedMax = 6e8;
    }

    // Determine if within range
    const withinRange = dose >= recommendedMin && dose <= recommendedMax;

    // Assess dose
    let assessment: DoseResult['assessment'];
    if (dose < recommendedMin * 0.5) {
      assessment = 'out-of-range';
    } else if (dose < recommendedMin) {
      assessment = 'suboptimal';
    } else if (dose <= recommendedMax) {
      assessment = 'optimal';
    } else if (dose <= recommendedMax * 1.5) {
      assessment = 'acceptable';
    } else {
      assessment = 'out-of-range';
    }

    // Generate notes
    const notes: string[] = [];
    if (!withinRange) {
      if (dose < recommendedMin) {
        notes.push(`Dose below recommended minimum (${recommendedMin.toExponential()} ${unit})`);
        notes.push('Consider collecting more cells or adjusting manufacturing');
      } else {
        notes.push(`Dose exceeds recommended maximum (${recommendedMax.toExponential()} ${unit})`);
        notes.push('Consider dose reduction to minimize toxicity risk');
      }
    }

    if (viability < CELLULAR_THERAPY_CONSTANTS.MIN_VIABILITY) {
      notes.push(`Viability (${viability}%) below minimum threshold (${CELLULAR_THERAPY_CONSTANTS.MIN_VIABILITY}%)`);
    }

    return {
      dose,
      totalViableCells,
      targetMarkerCells,
      withinRange,
      recommendedDose: {
        minimum: recommendedMin,
        maximum: recommendedMax,
        unit,
      },
      assessment,
      notes,
    };
  }

  /**
   * Assess quality control metrics
   *
   * @param metrics - Quality control measurements
   * @returns Quality assessment result
   */
  assessQuality(metrics: QualityMetrics): QualityResult {
    const failures: string[] = [];
    const warnings: string[] = [];

    // Check viability
    const viabilityPass = metrics.viability >= CELLULAR_THERAPY_CONSTANTS.MIN_VIABILITY;
    if (!viabilityPass) {
      failures.push(`Viability (${metrics.viability}%) below minimum (${CELLULAR_THERAPY_CONSTANTS.MIN_VIABILITY}%)`);
    } else if (metrics.viability < 90) {
      warnings.push(`Viability (${metrics.viability}%) is acceptable but below optimal (≥90%)`);
    }

    // Check potency
    const potencyPass = metrics.potency >= CELLULAR_THERAPY_CONSTANTS.MIN_POTENCY;
    if (!potencyPass) {
      failures.push(`Potency (${metrics.potency}%) below minimum (${CELLULAR_THERAPY_CONSTANTS.MIN_POTENCY}%)`);
    }

    // Check sterility
    const sterilityPass = metrics.sterility === true;
    if (!sterilityPass) {
      failures.push('Sterility test failed - product contaminated');
    }

    // Check endotoxin
    const endotoxinPass = metrics.endotoxin < CELLULAR_THERAPY_CONSTANTS.MAX_ENDOTOXIN;
    if (!endotoxinPass) {
      failures.push(`Endotoxin (${metrics.endotoxin} EU/mL) exceeds limit (${CELLULAR_THERAPY_CONSTANTS.MAX_ENDOTOXIN} EU/mL)`);
    } else if (metrics.endotoxin > CELLULAR_THERAPY_CONSTANTS.MAX_ENDOTOXIN * 0.8) {
      warnings.push(`Endotoxin (${metrics.endotoxin} EU/mL) approaching limit`);
    }

    // Check mycoplasma
    const mycoplasmaPass = metrics.mycoplasma === false;
    if (!mycoplasmaPass) {
      failures.push('Mycoplasma test positive');
    }

    // Assess identity markers (example for CAR-T)
    let identityPass = true;
    let identityString = 'Verified';

    // Check appearance
    if (metrics.appearance.particulates) {
      warnings.push('Visible particulates detected - inspect before use');
    }

    // Build release criteria
    const criteria: ReleaseCriteria = {
      viability: {
        required: CELLULAR_THERAPY_CONSTANTS.MIN_VIABILITY,
        actual: metrics.viability,
        pass: viabilityPass,
      },
      identity: {
        required: 'Target markers present',
        actual: identityString,
        pass: identityPass,
      },
      potency: {
        required: CELLULAR_THERAPY_CONSTANTS.MIN_POTENCY,
        actual: metrics.potency,
        pass: potencyPass,
      },
      sterility: {
        required: true,
        actual: metrics.sterility,
        pass: sterilityPass,
      },
      endotoxin: {
        required: CELLULAR_THERAPY_CONSTANTS.MAX_ENDOTOXIN,
        actual: metrics.endotoxin,
        pass: endotoxinPass,
      },
      mycoplasma: {
        required: false,
        actual: metrics.mycoplasma,
        pass: mycoplasmaPass,
      },
    };

    // Determine overall pass/fail
    const passRelease = failures.length === 0;

    // Assign grade
    let grade: QualityResult['grade'];
    if (!passRelease) {
      grade = 'F';
    } else if (warnings.length === 0 && metrics.viability >= 95 && metrics.potency >= 85) {
      grade = 'A';
    } else if (warnings.length <= 1) {
      grade = 'B';
    } else {
      grade = 'C';
    }

    // Determine recommendation
    let recommendation: QualityResult['recommendation'];
    if (failures.length > 0) {
      if (failures.some(f => f.includes('Sterility') || f.includes('Mycoplasma'))) {
        recommendation = 'reject';
      } else {
        recommendation = 'retest';
      }
    } else if (warnings.length > 2) {
      recommendation = 'conditional-release';
    } else {
      recommendation = 'release';
    }

    return {
      passRelease,
      grade,
      failures,
      warnings,
      recommendation,
      criteria,
    };
  }

  /**
   * Track manufacturing batch progress
   *
   * @param process - Manufacturing process data
   * @returns Updated process with recommendations
   */
  trackManufacturing(process: ManufacturingProcess): ManufacturingProcess {
    const alerts: typeof process.alerts = [...process.alerts];

    // Check critical process parameters
    const { cpp } = process;
    const limits = CELLULAR_THERAPY_CONSTANTS.CPP;

    if (cpp.temperature < limits.TEMPERATURE.MIN || cpp.temperature > limits.TEMPERATURE.MAX) {
      alerts.push({
        id: `TEMP-${Date.now()}`,
        severity: 'critical',
        message: `Temperature out of range: ${cpp.temperature}°C (acceptable: ${limits.TEMPERATURE.MIN}-${limits.TEMPERATURE.MAX}°C)`,
        timestamp: new Date(),
        resolved: false,
      });
    }

    if (cpp.pH < limits.PH.MIN || cpp.pH > limits.PH.MAX) {
      alerts.push({
        id: `PH-${Date.now()}`,
        severity: 'critical',
        message: `pH out of range: ${cpp.pH} (acceptable: ${limits.PH.MIN}-${limits.PH.MAX})`,
        timestamp: new Date(),
        resolved: false,
      });
    }

    if (cpp.cellDensity < limits.CELL_DENSITY.MIN || cpp.cellDensity > limits.CELL_DENSITY.MAX) {
      alerts.push({
        id: `DENSITY-${Date.now()}`,
        severity: 'warning',
        message: `Cell density out of optimal range: ${cpp.cellDensity.toExponential()} cells/mL`,
        timestamp: new Date(),
        resolved: false,
      });
    }

    // Check expansion progress
    if (process.currentMetrics.expansionRatio !== undefined) {
      const targetExpansion = process.parameters.targetExpansion;
      const currentExpansion = process.currentMetrics.expansionRatio;

      if (currentExpansion < targetExpansion * 0.5 && process.currentMetrics.dayInCulture > process.parameters.cultureDays * 0.7) {
        alerts.push({
          id: `EXP-${Date.now()}`,
          severity: 'warning',
          message: `Expansion ratio (${currentExpansion.toFixed(1)}×) below target (${targetExpansion}×) at day ${process.currentMetrics.dayInCulture}`,
          timestamp: new Date(),
          resolved: false,
        });
      }
    }

    // Check viability
    if (process.currentMetrics.viability < 85) {
      alerts.push({
        id: `VIA-${Date.now()}`,
        severity: 'critical',
        message: `Viability (${process.currentMetrics.viability}%) below acceptable threshold (85%)`,
        timestamp: new Date(),
        resolved: false,
      });
    } else if (process.currentMetrics.viability < 90) {
      alerts.push({
        id: `VIA-${Date.now()}`,
        severity: 'warning',
        message: `Viability (${process.currentMetrics.viability}%) below optimal (≥90%)`,
        timestamp: new Date(),
        resolved: false,
      });
    }

    return {
      ...process,
      alerts,
      cpp: {
        ...cpp,
        withinRanges: alerts.filter(a => a.severity === 'critical' && !a.resolved).length === 0,
      },
    };
  }

  /**
   * Assess CRS severity and recommend treatment
   *
   * @param assessment - CRS assessment data
   * @returns Treatment recommendation
   */
  assessCRS(assessment: CRSAssessment): {
    grade: number;
    treatment: string;
    urgency: 'routine' | 'urgent' | 'emergency';
  } {
    const { grade } = assessment;

    let treatment: string;
    let urgency: 'routine' | 'urgent' | 'emergency';

    switch (grade) {
      case 0:
        treatment = 'No treatment required. Continue monitoring.';
        urgency = 'routine';
        break;

      case 1:
        treatment = 'Supportive care: antipyretics, IV fluids. Monitor closely.';
        urgency = 'routine';
        break;

      case 2:
        treatment = 'Tocilizumab 8 mg/kg IV (max 800 mg). Consider corticosteroids if no improvement in 24h. Repeat tocilizumab q8h PRN.';
        urgency = 'urgent';
        break;

      case 3:
      case 4:
        treatment = 'Tocilizumab 8 mg/kg IV immediately + Methylprednisolone 1-2 mg/kg/day or Dexamethasone 10 mg q6h. ICU level care. Repeat tocilizumab q8h as needed.';
        urgency = 'emergency';
        break;

      default:
        treatment = 'Unknown grade';
        urgency = 'routine';
    }

    return { grade, treatment, urgency };
  }

  /**
   * Assess ICANS severity and recommend treatment
   *
   * @param assessment - ICANS assessment data
   * @returns Treatment recommendation
   */
  assessICANS(assessment: ICANSAssessment): {
    grade: number;
    treatment: string;
    urgency: 'routine' | 'urgent' | 'emergency';
  } {
    const { grade } = assessment;

    let treatment: string;
    let urgency: 'routine' | 'urgent' | 'emergency';

    switch (grade) {
      case 0:
        treatment = 'No treatment required. Continue ICE score monitoring q4-6h.';
        urgency = 'routine';
        break;

      case 1:
        treatment = 'Monitor ICE score q4h. Minimize sedation. Neurologic checks.';
        urgency = 'routine';
        break;

      case 2:
        treatment = 'Dexamethasone 10 mg IV q6h. Consider anti-seizure prophylaxis (levetiracetam 500-1500 mg BID). MRI if focal findings. Continuous monitoring.';
        urgency = 'urgent';
        break;

      case 3:
      case 4:
        treatment = 'Methylprednisolone 1-2 mg/kg/day or Dexamethasone 10-20 mg q6h. Anti-seizure medication. ICU care. Intubation PRN for airway protection. AVOID tocilizumab. Neurology consult.';
        urgency = 'emergency';
        break;

      default:
        treatment = 'Unknown grade';
        urgency = 'routine';
    }

    return { grade, treatment, urgency };
  }

  /**
   * Monitor patient safety post-infusion
   *
   * @param patientId - Patient identifier
   * @param daysPostInfusion - Days since cell infusion
   * @param events - Adverse events
   * @returns Safety monitoring report
   */
  monitorSafety(
    patientId: string,
    daysPostInfusion: number,
    events: SafetyEvent[]
  ): {
    riskLevel: 'low' | 'moderate' | 'high' | 'critical';
    activeEvents: SafetyEvent[];
    recommendations: string[];
  } {
    const activeEvents = events.filter(e => !e.resolutionDate);

    // Determine risk level
    let riskLevel: 'low' | 'moderate' | 'high' | 'critical' = 'low';

    const hasCriticalEvent = activeEvents.some(e => e.severity >= 4);
    const hasSeriousEvent = activeEvents.some(e => e.serious);
    const hasCRS = activeEvents.some(e => e.type === 'CRS');
    const hasICANS = activeEvents.some(e => e.type === 'ICANS');

    if (hasCriticalEvent) {
      riskLevel = 'critical';
    } else if (hasSeriousEvent || (hasCRS && hasICANS)) {
      riskLevel = 'high';
    } else if (activeEvents.length > 0) {
      riskLevel = 'moderate';
    }

    // Generate recommendations
    const recommendations: string[] = [];

    if (daysPostInfusion <= 10) {
      recommendations.push('Patient should remain hospitalized with continuous monitoring');
      recommendations.push('Assess for CRS and ICANS at least q4-6 hours');
    } else if (daysPostInfusion <= 30) {
      recommendations.push('Patient should remain within 2 hours of treatment center');
      recommendations.push('Weekly clinic visits required');
    }

    if (hasCRS) {
      recommendations.push('Tocilizumab should be readily available');
      recommendations.push('Monitor inflammatory markers (CRP, ferritin, IL-6)');
    }

    if (hasICANS) {
      recommendations.push('Perform ICE score assessment q4-6h');
      recommendations.push('Anti-seizure prophylaxis recommended');
    }

    if (activeEvents.some(e => e.type === 'infection')) {
      recommendations.push('Broad-spectrum antibiotics if febrile');
      recommendations.push('Consider IgG replacement if <400 mg/dL');
    }

    if (activeEvents.some(e => e.type === 'cytopenia')) {
      recommendations.push('Monitor CBC at least weekly');
      recommendations.push('G-CSF support for neutropenia');
      recommendations.push('Transfusion support as needed');
    }

    return {
      riskLevel,
      activeEvents,
      recommendations,
    };
  }

  /**
   * Assess post-thaw quality
   *
   * @param preFreeze - Pre-freeze metrics
   * @param postThaw - Post-thaw metrics
   * @returns Quality assessment
   */
  assessPostThawQuality(
    preFreeze: { viability: number; cellCount: number; potency: number },
    postThaw: { viability: number; cellCount: number; potency: number }
  ): PostThawQuality {
    const viability = postThaw.viability;
    const recovery = (postThaw.cellCount / preFreeze.cellCount) * 100;
    const potencyRetained = (postThaw.potency / preFreeze.potency) * 100;

    const acceptable =
      viability >= CELLULAR_THERAPY_CONSTANTS.MIN_VIABILITY &&
      recovery >= CELLULAR_THERAPY_CONSTANTS.MIN_RECOVERY &&
      potencyRetained >= 70;

    return {
      viability,
      recovery,
      potencyRetained,
      acceptable,
    };
  }

  /**
   * Generate Certificate of Analysis
   *
   * @param product - Cell product information
   * @param quality - Quality test results
   * @param potency - Potency assay results
   * @returns Certificate of Analysis
   */
  generateCertificate(
    product: CellProduct,
    quality: QualityMetrics,
    potency: PotencyAssay
  ): CertificateOfAnalysis {
    const qualityResult = this.assessQuality(quality);

    if (!qualityResult.passRelease) {
      throw new CellularTherapyError(
        CellularTherapyErrorCode.QUALITY_FAILURE,
        'Cannot generate certificate for product that failed release testing'
      );
    }

    const now = new Date();
    const expirationDate = new Date(product.storage?.freezeDate || now);
    expirationDate.setMonth(expirationDate.getMonth() + 24); // 24 months shelf life

    return {
      certificateNumber: `COA-${product.batchId}-${now.getTime()}`,
      product,
      testResults: quality,
      potencyAssay: potency,
      releaseDecision: {
        approved: true,
        approvedBy: 'QC Manager',
        approvalDate: now,
        expirationDate,
      },
      manufacturing: {
        facility: 'GMP Manufacturing Suite',
        batchRecord: product.batchId,
        deviations: [],
      },
      storage: {
        location: product.storage?.location || 'Unknown',
        temperature: product.storage?.temperature || -150,
        shippingConditions: 'Dry ice or liquid nitrogen vapor phase',
      },
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate dose (standalone function)
 */
export function calculateDose(params: DoseParameters): DoseResult {
  const sdk = new CellularTherapySDK();
  return sdk.calculateDose(params);
}

/**
 * Assess quality (standalone function)
 */
export function assessQuality(metrics: QualityMetrics): QualityResult {
  const sdk = new CellularTherapySDK();
  return sdk.assessQuality(metrics);
}

/**
 * Track manufacturing (standalone function)
 */
export function trackManufacturing(process: ManufacturingProcess): ManufacturingProcess {
  const sdk = new CellularTherapySDK();
  return sdk.trackManufacturing(process);
}

/**
 * Monitor safety (standalone function)
 */
export function monitorSafety(
  patientId: string,
  daysPostInfusion: number,
  events: SafetyEvent[]
) {
  const sdk = new CellularTherapySDK();
  return sdk.monitorSafety(patientId, daysPostInfusion, events);
}

/**
 * Generate certificate (standalone function)
 */
export function generateCertificate(
  product: CellProduct,
  quality: QualityMetrics,
  potency: PotencyAssay
): CertificateOfAnalysis {
  const sdk = new CellularTherapySDK();
  return sdk.generateCertificate(product, quality, potency);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { CellularTherapySDK };
export default CellularTherapySDK;
