/**
 * WIA-BIO-003: Gene Therapy SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for gene therapy including:
 * - Dosage calculations for viral vectors
 * - Safety assessment and monitoring
 * - Expression level prediction
 * - Clinical protocol generation
 * - CRISPR/Cas9 delivery optimization
 */

import {
  DosageParameters,
  DosageResult,
  SafetyAssessment,
  SafetyResult,
  ExpressionMonitoring,
  ExpressionResult,
  TransductionResult,
  ClinicalProtocol,
  GeneTherapyVector,
  MonitoringPlan,
  AdministrationProtocol,
  TherapeuticOutcome,
  PatientProfile,
  SimulationResult,
  GENE_THERAPY_CONSTANTS,
  GeneTherapyErrorCode,
  GeneTherapyError,
  VectorType,
  TargetTissue,
  ExpressionMeasurement,
  AdverseEvent,
  LabTest,
  FollowUpSchedule,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-003 Gene Therapy SDK
 */
export class GeneTherapySDK {
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
   * Calculate optimal dosage for gene therapy
   *
   * @param params - Dosage calculation parameters
   * @returns Calculated dosage and administration details
   */
  calculateDosage(params: DosageParameters): DosageResult {
    const {
      patientWeight,
      targetTissue,
      vectorType,
      therapeuticGene,
      serotype,
      targetCellCount,
      vgPerCell,
      route,
    } = params;

    // Validate inputs
    if (patientWeight <= 0) {
      throw new GeneTherapyError(
        GeneTherapyErrorCode.INVALID_PARAMETERS,
        'Patient weight must be positive'
      );
    }

    // Determine tissue-specific parameters
    const tissueParams = this.getTissueParameters(targetTissue);
    const effectiveVgPerCell = vgPerCell || tissueParams.vgPerCell;
    const effectiveTargetCells = targetCellCount || tissueParams.cellCount;

    // Calculate total viral genomes needed
    const totalVG = effectiveTargetCells * effectiveVgPerCell;

    // Calculate dose per kg
    const vgPerKg = totalVG / patientWeight;

    // Determine vector titer and calculate volume
    const vectorTiter = this.getVectorTiter(vectorType);
    const volumeMl = totalVG / vectorTiter;

    // Get expected efficiency
    const expectedEfficiency = this.getTransductionEfficiency(vectorType, targetTissue);

    // Assess feasibility
    const feasibility = this.assessDoseFeasibility(vgPerKg);
    const doseLevel = this.classifyDoseLevel(vgPerKg);

    // Generate administration protocol
    const protocol = this.generateAdministrationProtocol(
      vectorType,
      targetTissue,
      volumeMl,
      route
    );

    // Generate warnings
    const warnings = this.generateDosageWarnings(vgPerKg, volumeMl, doseLevel);

    return {
      viralGenomes: totalVG,
      vgPerKg,
      volumeMl,
      expectedEfficiency,
      feasibility,
      doseLevel,
      protocol,
      warnings,
    };
  }

  /**
   * Assess safety profile for gene therapy
   *
   * @param assessment - Safety assessment parameters
   * @returns Safety evaluation and recommendations
   */
  assessSafety(assessment: SafetyAssessment): SafetyResult {
    const {
      vectorDose,
      immuneStatus,
      preexistingAntibodies,
      nabTiter,
      liverFunction,
      kidneyFunction,
      age,
      previousTherapy,
      medications,
    } = assessment;

    const warnings: string[] = [];
    const recommendations: string[] = [];
    let score = 100;
    const predictedAdverseEvents: AdverseEvent[] = [];

    // Check neutralizing antibodies
    if (preexistingAntibodies) {
      score -= 30;
      warnings.push('Pre-existing neutralizing antibodies detected');
      recommendations.push('Consider alternative AAV serotype or immunosuppression');

      if (nabTiter && nabTiter > GENE_THERAPY_CONSTANTS.SAFETY_THRESHOLDS.NAB_TITER_MAX) {
        score -= 20;
        warnings.push(`NAb titer (1:${nabTiter}) exceeds safe threshold`);
        recommendations.push('Patient may not be eligible - consult medical director');
      }
    }

    // Check liver function
    const altRatio = liverFunction.ALT / GENE_THERAPY_CONSTANTS.SAFETY_THRESHOLDS.ALT_ULN;
    const astRatio = liverFunction.AST / GENE_THERAPY_CONSTANTS.SAFETY_THRESHOLDS.AST_ULN;

    if (altRatio > 1.5 || astRatio > 1.5) {
      score -= 25;
      warnings.push('Elevated liver enzymes detected');
      recommendations.push('Defer treatment until liver function normalizes');

      predictedAdverseEvents.push({
        type: 'Hepatotoxicity',
        probability: 0.4,
        grade: 3,
        onset: '2-4 weeks',
        duration: '4-8 weeks',
        management: ['Monitor ALT/AST weekly', 'Consider corticosteroids', 'Reduce immunosuppression if needed'],
      });
    }

    if (liverFunction.totalBilirubin > GENE_THERAPY_CONSTANTS.SAFETY_THRESHOLDS.BILIRUBIN_MAX) {
      score -= 30;
      warnings.push('Elevated bilirubin indicates liver dysfunction');
      recommendations.push('Treatment not recommended - investigate liver disease');
    }

    // Check dose level
    if (vectorDose > GENE_THERAPY_CONSTANTS.DOSE_RANGES.HIGH) {
      score -= 15;
      warnings.push('High-dose regimen increases adverse event risk');
      recommendations.push('Enhanced monitoring protocol required');

      predictedAdverseEvents.push({
        type: 'Thrombotic Microangiopathy',
        probability: vectorDose > GENE_THERAPY_CONSTANTS.DOSE_RANGES.MTD ? 0.15 : 0.05,
        grade: 4,
        onset: '1-2 weeks',
        duration: '2-6 weeks',
        management: ['Daily platelet monitoring', 'LDH/haptoglobin tracking', 'Nephrology consult on standby'],
      });
    }

    // Check immune status
    if (immuneStatus === 'autoimmune') {
      score -= 20;
      warnings.push('Autoimmune condition may increase immune response risk');
      recommendations.push('Prophylactic immunosuppression strongly recommended');
    } else if (immuneStatus === 'suppressed') {
      score -= 10;
      warnings.push('Immunosuppressed state may affect efficacy');
    }

    // Age considerations
    if (age < 2) {
      score -= 10;
      warnings.push('Pediatric dosing requires special consideration');
      recommendations.push('Use weight-based dosing with pediatric safety margins');
    } else if (age > 70) {
      score -= 10;
      warnings.push('Advanced age may affect treatment tolerance');
      recommendations.push('Enhanced safety monitoring recommended');
    }

    // Previous therapy check
    if (previousTherapy) {
      score -= 25;
      warnings.push('Previous gene therapy increases immunogenicity risk');
      recommendations.push('Different serotype required; immunosuppression essential');
    }

    // Add standard adverse events
    predictedAdverseEvents.push(
      {
        type: 'Infusion reaction',
        probability: 0.2,
        grade: 1,
        onset: 'During infusion',
        duration: '1-4 hours',
        management: ['Slow infusion rate', 'Antihistamines', 'Corticosteroids if needed'],
      },
      {
        type: 'Flu-like symptoms',
        probability: 0.3,
        grade: 1,
        onset: '24-48 hours',
        duration: '2-5 days',
        management: ['Supportive care', 'Acetaminophen', 'Rest'],
      }
    );

    // Determine risk level
    let riskLevel: SafetyResult['riskLevel'];
    if (score >= 80) riskLevel = 'low';
    else if (score >= 60) riskLevel = 'medium';
    else if (score >= 40) riskLevel = 'high';
    else riskLevel = 'extreme';

    // Determine eligibility
    const eligible = score >= 40 && liverFunction.totalBilirubin <= 2.0;
    const ineligibilityReasons = eligible ? undefined : warnings.filter(w =>
      w.includes('not recommended') || w.includes('not be eligible')
    );

    // Generate monitoring plan
    const monitoringPlan = this.generateMonitoringPlan(riskLevel, vectorDose);

    return {
      score,
      riskLevel,
      warnings,
      recommendations,
      monitoringPlan,
      predictedAdverseEvents,
      eligible,
      ineligibilityReasons,
    };
  }

  /**
   * Predict transduction efficiency
   *
   * @param vectorType - Type of viral vector
   * @param targetTissue - Target tissue
   * @param dose - Dose in vg/kg (optional)
   * @returns Predicted efficiency percentage
   */
  predictEfficiency(
    vectorType: VectorType,
    targetTissue: TargetTissue,
    dose?: number
  ): number {
    const baseEfficiency = this.getTransductionEfficiency(vectorType, targetTissue);

    // Adjust for dose if provided
    if (dose) {
      const doseFactor = this.calculateDoseFactor(dose);
      return Math.min(100, baseEfficiency * doseFactor);
    }

    return baseEfficiency;
  }

  /**
   * Monitor gene expression over time
   *
   * @param monitoring - Expression monitoring parameters
   * @returns Expression analysis results
   */
  monitorExpression(monitoring: ExpressionMonitoring): ExpressionResult {
    const { gene, timePoints, method, baselineValue, targetLevel, unit } = monitoring;

    // Generate simulated measurements (in real implementation, this would use actual data)
    const measurements: ExpressionMeasurement[] = timePoints.map((timepoint) => {
      const value = this.simulateExpression(gene, timepoint, method);
      const percentOfNormal = targetLevel ? (value / targetLevel) * 100 : 0;

      return {
        timepoint,
        value,
        unit,
        percentOfNormal,
        standardDeviation: value * 0.1,
        qcStatus: 'pass',
      };
    });

    // Analyze trend
    const trend = this.analyzeTrend(measurements);

    // Find peak expression
    const peakMeasurement = measurements.reduce((max, m) =>
      m.value > max.value ? m : max
    );

    const peakExpression = {
      value: peakMeasurement.value,
      timepoint: peakMeasurement.timepoint,
      percentOfNormal: peakMeasurement.percentOfNormal,
    };

    // Current level (last measurement)
    const lastMeasurement = measurements[measurements.length - 1];
    const currentLevel = {
      value: lastMeasurement.value,
      percentOfNormal: lastMeasurement.percentOfNormal,
      therapeutic: targetLevel
        ? lastMeasurement.value >= targetLevel * 0.4 && lastMeasurement.value <= targetLevel * 1.5
        : false,
    };

    // Check therapeutic range
    const therapeuticRange = currentLevel.therapeutic;

    return {
      measurements,
      trend,
      therapeuticRange,
      peakExpression,
      currentLevel,
    };
  }

  /**
   * Generate clinical trial protocol
   *
   * @param condition - Disease/condition
   * @param vectorType - Vector type to use
   * @param phase - Trial phase
   * @returns Clinical protocol specification
   */
  generateProtocol(
    condition: string,
    vectorType: VectorType,
    phase: 'Phase-I' | 'Phase-I/II' | 'Phase-II' | 'Phase-III' = 'Phase-I/II'
  ): ClinicalProtocol {
    // Generate protocol ID
    const protocolId = `WIA-BIO-003-${Date.now()}-${vectorType}`;

    // Create therapy vector (simplified)
    const therapy: GeneTherapyVector = {
      id: `${vectorType}-001`,
      type: vectorType,
      gene: this.getGeneForCondition(condition),
      geneName: `${this.getGeneForCondition(condition)} gene`,
      promoter: 'Liver-specific',
      genomeSize: 4.5,
      titer: 1e13,
      tropism: ['liver'],
      expectedEfficiency: 70,
      immunogenicity: 'low',
      duration: 'long-term',
    };

    // Standard inclusion criteria
    const inclusionCriteria = [
      `Confirmed diagnosis of ${condition}`,
      'Age ≥18 years',
      'AAV neutralizing antibody titer <1:5',
      'Normal liver function (ALT/AST <1.5× ULN)',
      'Normal kidney function (eGFR >60 mL/min/1.73m²)',
      'Adequate bone marrow function',
      'Willing to use contraception for 6 months post-treatment',
    ];

    // Standard exclusion criteria
    const exclusionCriteria = [
      'Active hepatitis B or C infection',
      'HIV positive',
      'Severe immunodeficiency',
      'Active malignancy within past 5 years',
      'Pregnancy or nursing',
      'Prior gene therapy with same serotype',
      'Significant cardiac, pulmonary, or renal disease',
      'Concurrent immunosuppressive therapy',
    ];

    // Dose escalation design
    const doseLevels = [1e12, 3e12, 1e13, 3e13, 1e14];

    const dosingRegimen = {
      doseLevels,
      escalationDesign: '3+3' as const,
      patientsPerCohort: 3,
      dltObservationPeriod: '28 days',
      mtdTarget: 1e14,
    };

    // Primary and secondary endpoints
    const primaryEndpoints = [
      'Safety and tolerability',
      'Incidence of dose-limiting toxicities',
      'Maximum tolerated dose',
    ];

    const secondaryEndpoints = [
      'Transgene expression levels',
      `Clinical improvement in ${condition} phenotype`,
      'Quality of life scores',
      'Immunogenicity (NAb and T-cell responses)',
      'Vector biodistribution',
    ];

    // Safety monitoring
    const safetyMonitoring = this.generateMonitoringPlan('medium', 1e13);

    return {
      protocolId,
      phase,
      condition,
      therapy,
      inclusionCriteria,
      exclusionCriteria,
      dosingRegimen,
      primaryEndpoints,
      secondaryEndpoints,
      safetyMonitoring,
      duration: '12 months primary, 5 years long-term follow-up',
      sampleSize: doseLevels.length * 3, // Minimum for 3+3 design
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Get tissue-specific parameters
   */
  private getTissueParameters(tissue: TargetTissue): {
    cellCount: number;
    vgPerCell: number;
  } {
    const params: Record<TargetTissue, { cellCount: number; vgPerCell: number }> = {
      liver: { cellCount: 2e11, vgPerCell: 1e5 },
      muscle: { cellCount: 5e11, vgPerCell: 1e4 },
      CNS: { cellCount: 1e11, vgPerCell: 1e5 },
      heart: { cellCount: 3e10, vgPerCell: 1e5 },
      lung: { cellCount: 1e11, vgPerCell: 1e4 },
      retina: { cellCount: 1e8, vgPerCell: 1e5 },
      blood: { cellCount: 5e12, vgPerCell: 10 },
      'bone-marrow': { cellCount: 1e12, vgPerCell: 100 },
      other: { cellCount: 1e10, vgPerCell: 1e4 },
    };

    return params[tissue] || params.other;
  }

  /**
   * Get vector titer
   */
  private getVectorTiter(vectorType: VectorType): number {
    const titers: Record<string, number> = {
      AAV1: 1e13,
      AAV2: 1e13,
      AAV5: 1e13,
      AAV8: 5e13,
      AAV9: 1e13,
      AAVrh10: 1e13,
      lentivirus: 1e9,
      adenovirus: 1e12,
      LNP: 1e12,
    };

    return titers[vectorType] || 1e13;
  }

  /**
   * Get transduction efficiency
   */
  private getTransductionEfficiency(vectorType: VectorType, tissue: TargetTissue): number {
    const key = `${tissue}_${vectorType}`;
    const efficiencies: Record<string, number> = {
      liver_AAV8: 80,
      liver_AAV9: 70,
      muscle_AAV1: 60,
      muscle_AAV9: 50,
      CNS_AAV9: 40,
      retina_AAV2: 70,
    };

    return efficiencies[key] || 50;
  }

  /**
   * Assess dose feasibility
   */
  private assessDoseFeasibility(vgPerKg: number): DosageResult['feasibility'] {
    if (vgPerKg <= GENE_THERAPY_CONSTANTS.DOSE_RANGES.MEDIUM) {
      return 'standard';
    } else if (vgPerKg <= GENE_THERAPY_CONSTANTS.DOSE_RANGES.HIGH) {
      return 'high-dose';
    } else if (vgPerKg <= GENE_THERAPY_CONSTANTS.DOSE_RANGES.MTD) {
      return 'experimental';
    } else {
      return 'not-recommended';
    }
  }

  /**
   * Classify dose level
   */
  private classifyDoseLevel(vgPerKg: number): DosageResult['doseLevel'] {
    if (vgPerKg <= GENE_THERAPY_CONSTANTS.DOSE_RANGES.LOW) return 'low';
    if (vgPerKg <= GENE_THERAPY_CONSTANTS.DOSE_RANGES.MEDIUM) return 'medium';
    if (vgPerKg <= GENE_THERAPY_CONSTANTS.DOSE_RANGES.HIGH) return 'high';
    return 'MTD';
  }

  /**
   * Generate administration protocol
   */
  private generateAdministrationProtocol(
    vectorType: VectorType,
    tissue: TargetTissue,
    volumeMl: number,
    route?: string
  ): AdministrationProtocol {
    const defaultRoute = route || (tissue === 'CNS' ? 'intrathecal' : 'IV');

    return {
      route: defaultRoute as any,
      duration: volumeMl > 100 ? '2-4 hours' : '1-2 hours',
      premedications: [
        'Methylprednisolone 500mg IV',
        'Diphenhydramine 50mg IV',
        'Acetaminophen 650mg PO',
      ],
      observationPeriod: '24 hours',
      instructions: [
        'Administer through peripheral IV or central line',
        'Monitor vital signs every 15 minutes during infusion',
        'Have emergency medications available',
        'Patient should fast 4 hours before infusion',
      ],
    };
  }

  /**
   * Generate dosage warnings
   */
  private generateDosageWarnings(
    vgPerKg: number,
    volumeMl: number,
    doseLevel: string
  ): string[] {
    const warnings: string[] = [];

    if (vgPerKg > GENE_THERAPY_CONSTANTS.DOSE_RANGES.HIGH) {
      warnings.push('High-dose regimen: Enhanced safety monitoring required');
    }

    if (volumeMl > 500) {
      warnings.push('Large volume infusion: Consider split dosing or concentration');
    }

    if (doseLevel === 'MTD') {
      warnings.push('Dose near maximum tolerated dose: Use only in approved clinical trials');
    }

    return warnings;
  }

  /**
   * Calculate dose factor for efficiency adjustment
   */
  private calculateDoseFactor(dose: number): number {
    // Simple logarithmic relationship
    const referenceDose = GENE_THERAPY_CONSTANTS.DOSE_RANGES.MEDIUM;
    const factor = 1 + 0.2 * Math.log10(dose / referenceDose);
    return Math.max(0.5, Math.min(1.5, factor));
  }

  /**
   * Generate monitoring plan
   */
  private generateMonitoringPlan(riskLevel: string, dose: number): MonitoringPlan {
    const labTests: LabTest[] = [
      {
        name: 'Complete Blood Count',
        frequency: riskLevel === 'high' ? 'Daily × 7d, then weekly × 4w' : 'Weekly × 4w',
        normalRange: 'Age-appropriate',
        actionThreshold: 'Grade 3 hematologic toxicity',
      },
      {
        name: 'Liver Function (ALT/AST)',
        frequency: riskLevel === 'high' ? 'Daily × 14d, then weekly × 8w' : 'Weekly × 12w',
        normalRange: '<40 U/L',
        actionThreshold: '>3× ULN',
      },
      {
        name: 'Coagulation (PT/INR)',
        frequency: 'Weekly × 12w',
        normalRange: 'INR 0.9-1.1',
        actionThreshold: 'INR >1.5',
      },
      {
        name: 'Kidney Function (Cr/eGFR)',
        frequency: 'Weekly × 4w, then monthly',
        normalRange: 'eGFR >60',
        actionThreshold: 'eGFR <30',
      },
    ];

    if (dose > GENE_THERAPY_CONSTANTS.DOSE_RANGES.HIGH) {
      labTests.push({
        name: 'TMA Panel (Platelets/LDH/Haptoglobin)',
        frequency: 'Daily × 14d',
        normalRange: 'Platelets >150k, LDH <250, Haptoglobin >30',
        actionThreshold: 'Platelets <100k or rising LDH',
      });
    }

    const schedule: FollowUpSchedule[] = [
      {
        timepoint: 'Day 1',
        assessments: ['Vital signs q15min', 'Adverse event monitoring'],
        critical: true,
      },
      {
        timepoint: 'Week 1',
        assessments: ['CBC', 'CMP', 'LFTs', 'Transgene expression'],
        critical: true,
      },
      {
        timepoint: 'Week 4',
        assessments: ['CBC', 'CMP', 'LFTs', 'Transgene expression', 'NAb titer'],
        critical: true,
      },
      {
        timepoint: 'Month 3',
        assessments: ['CBC', 'CMP', 'LFTs', 'Functional assays', 'QoL survey'],
        critical: false,
      },
      {
        timepoint: 'Month 6',
        assessments: ['Full panel', 'Imaging', 'Efficacy assessment'],
        critical: false,
      },
    ];

    return {
      frequency: riskLevel === 'high' ? 'Daily → Weekly → Monthly' : 'Weekly → Monthly',
      biomarkers: ['ALT', 'AST', 'Platelets', 'Transgene mRNA', 'Protein expression'],
      labTests,
      imaging: dose > GENE_THERAPY_CONSTANTS.DOSE_RANGES.HIGH ? ['Liver ultrasound'] : undefined,
      duration: '12 months primary, 5 years long-term',
      schedule,
    };
  }

  /**
   * Simulate gene expression (for demonstration)
   */
  private simulateExpression(gene: string, timepoint: number, method: string): number {
    // Simplified AAV expression kinetics
    const onset = GENE_THERAPY_CONSTANTS.EXPRESSION_KINETICS.AAV_ONSET;
    const peak = GENE_THERAPY_CONSTANTS.EXPRESSION_KINETICS.AAV_PEAK;
    const plateau = GENE_THERAPY_CONSTANTS.EXPRESSION_KINETICS.AAV_PLATEAU;

    if (timepoint < onset) return 0.1;
    if (timepoint < peak) {
      // Rising phase
      return 10 + (90 * (timepoint - onset)) / (peak - onset);
    }
    if (timepoint < plateau) {
      // Peak to plateau
      return 100 - (10 * (timepoint - peak)) / (plateau - peak);
    }
    // Plateau phase
    return 90 + Math.random() * 10;
  }

  /**
   * Analyze expression trend
   */
  private analyzeTrend(measurements: ExpressionMeasurement[]): ExpressionResult['trend'] {
    if (measurements.length < 2) return 'stable';

    const values = measurements.map((m) => m.value);
    const firstHalf = values.slice(0, Math.floor(values.length / 2));
    const secondHalf = values.slice(Math.floor(values.length / 2));

    const avgFirst = firstHalf.reduce((a, b) => a + b, 0) / firstHalf.length;
    const avgSecond = secondHalf.reduce((a, b) => a + b, 0) / secondHalf.length;

    const change = (avgSecond - avgFirst) / avgFirst;

    if (change > 0.2) return 'increasing';
    if (change < -0.2) return 'decreasing';
    return 'stable';
  }

  /**
   * Get gene for condition (simplified mapping)
   */
  private getGeneForCondition(condition: string): string {
    const mapping: Record<string, string> = {
      hemophilia: 'F8',
      'hemophilia A': 'F8',
      'hemophilia B': 'F9',
      SMA: 'SMN1',
      DMD: 'DMD',
      'sickle cell': 'HBB',
    };

    return mapping[condition.toLowerCase()] || 'THERAPEUTIC_GENE';
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Calculate dosage (standalone function)
 */
export function calculateDosage(params: DosageParameters): DosageResult {
  const sdk = new GeneTherapySDK();
  return sdk.calculateDosage(params);
}

/**
 * Assess safety (standalone function)
 */
export function assessSafety(assessment: SafetyAssessment): SafetyResult {
  const sdk = new GeneTherapySDK();
  return sdk.assessSafety(assessment);
}

/**
 * Predict efficiency (standalone function)
 */
export function predictEfficiency(
  vectorType: VectorType,
  targetTissue: TargetTissue,
  dose?: number
): number {
  const sdk = new GeneTherapySDK();
  return sdk.predictEfficiency(vectorType, targetTissue, dose);
}

/**
 * Monitor expression (standalone function)
 */
export function monitorExpression(monitoring: ExpressionMonitoring): ExpressionResult {
  const sdk = new GeneTherapySDK();
  return sdk.monitorExpression(monitoring);
}

/**
 * Generate protocol (standalone function)
 */
export function generateProtocol(
  condition: string,
  vectorType: VectorType,
  phase?: 'Phase-I' | 'Phase-I/II' | 'Phase-II' | 'Phase-III'
): ClinicalProtocol {
  const sdk = new GeneTherapySDK();
  return sdk.generateProtocol(condition, vectorType, phase);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { GeneTherapySDK };
export default GeneTherapySDK;
