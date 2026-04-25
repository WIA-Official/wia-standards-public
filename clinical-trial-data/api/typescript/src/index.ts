/**
 * WIA-BIO-010: Clinical Trial Data SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive tools for clinical trial data management including:
 * - Trial registration and management
 * - Patient data collection (CDISC-compliant)
 * - Adverse event reporting (MedDRA coding)
 * - Statistical analysis (sample size, power, hypothesis testing)
 * - Regulatory submission preparation
 */

import {
  ClinicalTrial,
  TrialPhase,
  TrialStatus,
  PatientEnrollment,
  PatientVisit,
  AdverseEvent,
  SampleSizeCalculation,
  SampleSizeResult,
  PowerCalculation,
  PowerResult,
  HypothesisTestResult,
  DescriptiveStats,
  DataCollectionRequest,
  DataCollectionResponse,
  RegulatorySubmission,
  RegulatoryAgency,
  SubmissionFormat,
  CLINICAL_CONSTANTS,
  VITAL_SIGNS_RANGES,
  LAB_RANGES,
  ClinicalTrialErrorCode,
  ClinicalTrialError,
  Investigator,
  Endpoint,
  Demographics,
  ClinicalMeasurements,
  LabResults,
  VitalSigns,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-010 Clinical Trial Data SDK
 */
export class ClinicalTrialSDK {
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
   * Register a new clinical trial
   *
   * @param params - Trial registration parameters
   * @returns Registered clinical trial
   */
  registerTrial(params: {
    title: string;
    phase: TrialPhase;
    indication: string;
    primaryEndpoint: string;
    sampleSize: number;
    duration?: number;
    sponsor?: string;
    investigator?: Investigator;
  }): ClinicalTrial {
    const {
      title,
      phase,
      indication,
      primaryEndpoint,
      sampleSize,
      duration = 52,
      sponsor = 'Not specified',
      investigator = {
        name: 'Principal Investigator',
        institution: 'Research Center',
      },
    } = params;

    // Generate trial ID (NCT format)
    const trialId = `NCT${Date.now().toString().slice(-8)}`;

    const trial: ClinicalTrial = {
      id: trialId,
      title,
      phase,
      indication,
      sponsor,
      principalInvestigator: investigator,
      primaryEndpoint: {
        parameter: primaryEndpoint,
        metric: 'Change from baseline',
        timepoint: `Week ${duration}`,
      },
      sampleSize: {
        planned: sampleSize,
        enrolled: 0,
        randomized: 0,
        completed: 0,
        withdrawn: 0,
        dropoutRate: CLINICAL_CONSTANTS.DROPOUT_RATE,
      },
      startDate: new Date(),
      estimatedCompletionDate: new Date(
        Date.now() + duration * 7 * 24 * 60 * 60 * 1000
      ),
      status: 'planned',
      protocolVersion: '1.0',
      registrationDate: new Date(),
      duration,
    };

    return trial;
  }

  /**
   * Enroll a patient in a clinical trial
   *
   * @param params - Enrollment parameters
   * @returns Patient enrollment record
   */
  enrollPatient(params: {
    trialId: string;
    demographics: Demographics;
    consentDate: Date | string;
  }): PatientEnrollment {
    const { trialId, demographics, consentDate } = params;

    // Generate subject ID
    const subjectId = `${trialId}-${String(Math.floor(Math.random() * 1000)).padStart(
      3,
      '0'
    )}`;

    const enrollment: PatientEnrollment = {
      subjectId,
      trialId,
      demographics,
      enrollmentDate: new Date(),
      consentDate,
      status: 'enrolled',
    };

    return enrollment;
  }

  /**
   * Collect patient data for a visit
   *
   * @param request - Data collection request
   * @returns Data collection response with validation
   */
  collectPatientData(request: DataCollectionRequest): DataCollectionResponse {
    const { trialId, patientId, visit, measurements, laboratoryResults, vitalSigns } =
      request;

    const validationErrors: string[] = [];
    const warnings: string[] = [];

    // Validate measurements
    if (measurements) {
      Object.entries(measurements).forEach(([key, value]) => {
        if (typeof value !== 'number' || isNaN(value)) {
          validationErrors.push(`Invalid measurement value for ${key}`);
        }
      });
    }

    // Validate vital signs
    if (vitalSigns) {
      if (
        vitalSigns.systolicBP &&
        (vitalSigns.systolicBP < VITAL_SIGNS_RANGES.systolicBP.min ||
          vitalSigns.systolicBP > 250)
      ) {
        if (vitalSigns.systolicBP > 180) {
          warnings.push(
            `Systolic BP (${vitalSigns.systolicBP}) is critically high`
          );
        } else {
          warnings.push(`Systolic BP (${vitalSigns.systolicBP}) is out of normal range`);
        }
      }

      if (
        vitalSigns.heartRate &&
        (vitalSigns.heartRate < 30 || vitalSigns.heartRate > 200)
      ) {
        warnings.push(`Heart rate (${vitalSigns.heartRate}) is out of valid range`);
      }
    }

    // Validate laboratory results
    if (laboratoryResults?.chemistry) {
      const { hba1c, glucose, alt, ast } = laboratoryResults.chemistry;

      if (hba1c && (hba1c < 3 || hba1c > 20)) {
        warnings.push(`HbA1c (${hba1c}%) is out of expected range`);
      }

      if (glucose && (glucose < 20 || glucose > 600)) {
        warnings.push(`Glucose (${glucose} mg/dL) is out of valid range`);
      }

      if (alt && alt > 200) {
        warnings.push(`ALT (${alt} U/L) is significantly elevated (Grade 2+)`);
      }

      if (ast && ast > 200) {
        warnings.push(`AST (${ast} U/L) is significantly elevated (Grade 2+)`);
      }
    }

    // Generate data ID
    const dataId = `DATA-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    return {
      success: validationErrors.length === 0,
      dataId: validationErrors.length === 0 ? dataId : undefined,
      validationErrors,
      warnings,
      timestamp: new Date(),
    };
  }

  /**
   * Record an adverse event
   *
   * @param ae - Adverse event information
   * @returns Recorded adverse event with ID
   */
  recordAdverseEvent(
    ae: Omit<AdverseEvent, 'aeId'>
  ): AdverseEvent & { reportingRequired: boolean; reportingDeadline?: Date } {
    // Generate AE ID
    const aeId = `AE-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    const adverseEvent: AdverseEvent = {
      aeId,
      ...ae,
    };

    // Determine reporting requirements
    const reportingRequired = ae.seriousness.serious;
    let reportingDeadline: Date | undefined;

    if (reportingRequired) {
      const daysToReport = ae.seriousness.death || ae.seriousness.lifeThreatening
        ? CLINICAL_CONSTANTS.FATAL_SAE_REPORTING_DAYS
        : CLINICAL_CONSTANTS.SAE_REPORTING_DAYS;

      reportingDeadline = new Date(
        new Date(ae.startDate).getTime() + daysToReport * 24 * 60 * 60 * 1000
      );
    }

    return {
      ...adverseEvent,
      reportingRequired,
      reportingDeadline,
    };
  }

  /**
   * Calculate required sample size for clinical trial
   *
   * @param params - Sample size calculation parameters
   * @returns Sample size calculation result
   */
  calculateSampleSize(params: SampleSizeCalculation): SampleSizeResult {
    const {
      alpha,
      power,
      effectSize,
      standardDeviation,
      controlRate,
      treatmentRate,
      testType,
      dropoutRate = CLINICAL_CONSTANTS.DROPOUT_RATE,
    } = params;

    // Validate inputs
    if (alpha <= 0 || alpha >= 1) {
      throw new ClinicalTrialError(
        ClinicalTrialErrorCode.STATISTICAL_VIOLATION,
        'Alpha must be between 0 and 1'
      );
    }

    if (power <= 0 || power >= 1) {
      throw new ClinicalTrialError(
        ClinicalTrialErrorCode.STATISTICAL_VIOLATION,
        'Power must be between 0 and 1'
      );
    }

    // Get critical values
    const zAlpha = testType === 'two-tailed' ? 1.96 : 1.645; // for α=0.05
    const zBeta = power >= 0.9 ? 1.28 : 0.84; // 90% or 80% power

    let sampleSizePerGroup: number;
    let method: string;

    // Calculate based on outcome type
    if (standardDeviation !== undefined) {
      // Continuous outcome
      method = 'Continuous outcome (two-sample t-test)';
      const numerator = Math.pow(zAlpha + zBeta, 2) * 2 * Math.pow(standardDeviation, 2);
      const denominator = Math.pow(effectSize, 2);
      sampleSizePerGroup = Math.ceil(numerator / denominator);
    } else if (controlRate !== undefined && treatmentRate !== undefined) {
      // Binary outcome
      method = 'Binary outcome (two-proportion test)';
      const p1 = treatmentRate;
      const p2 = controlRate;
      const pBar = (p1 + p2) / 2;

      const numerator =
        Math.pow(zAlpha * Math.sqrt(2 * pBar * (1 - pBar)), 2) +
        Math.pow(zBeta * Math.sqrt(p1 * (1 - p1) + p2 * (1 - p2)), 2);
      const denominator = Math.pow(p1 - p2, 2);

      sampleSizePerGroup = Math.ceil(numerator / denominator);
    } else {
      throw new ClinicalTrialError(
        ClinicalTrialErrorCode.STATISTICAL_VIOLATION,
        'Must provide either standardDeviation (continuous) or controlRate/treatmentRate (binary)'
      );
    }

    const totalSampleSize = sampleSizePerGroup * 2;

    // Adjust for dropout
    const adjustedSampleSize = Math.ceil(totalSampleSize / (1 - dropoutRate));

    return {
      sampleSizePerGroup,
      totalSampleSize,
      adjustedSampleSize,
      power,
      effectSize,
      method,
    };
  }

  /**
   * Calculate statistical power
   *
   * @param params - Power calculation parameters
   * @returns Power calculation result
   */
  calculatePower(params: PowerCalculation): PowerResult {
    const { sampleSize, alpha, effectSize, standardDeviation, testType } = params;

    // Simplified power calculation using effect size
    const zAlpha = testType === 'two-tailed' ? 1.96 : 1.645;

    // For continuous outcomes
    if (standardDeviation) {
      const delta = effectSize;
      const se = standardDeviation * Math.sqrt(2 / sampleSize);
      const noncentrality = delta / se;
      const zBeta = noncentrality - zAlpha;

      // Convert to power using normal CDF approximation
      const power = this.normalCDF(zBeta);

      return {
        power: Math.max(0, Math.min(1, power)),
        sampleSize,
        effectSize,
        alpha,
      };
    }

    // Simplified calculation if no SD provided
    const estimatedPower = Math.min(
      0.99,
      0.5 + 0.4 * (sampleSize / 100) * effectSize
    );

    return {
      power: estimatedPower,
      sampleSize,
      effectSize,
      alpha,
    };
  }

  /**
   * Perform hypothesis test (two-sample t-test)
   *
   * @param group1 - First group data
   * @param group2 - Second group data
   * @param alpha - Significance level
   * @returns Hypothesis test result
   */
  performHypothesisTest(
    group1: number[],
    group2: number[],
    alpha: number = 0.05
  ): HypothesisTestResult {
    // Calculate descriptive statistics for each group
    const stats1 = this.calculateDescriptiveStats(group1);
    const stats2 = this.calculateDescriptiveStats(group2);

    const n1 = group1.length;
    const n2 = group2.length;

    // Calculate pooled variance
    const pooledVariance =
      ((n1 - 1) * Math.pow(stats1.sd, 2) + (n2 - 1) * Math.pow(stats2.sd, 2)) /
      (n1 + n2 - 2);

    // Calculate standard error
    const standardError = Math.sqrt(pooledVariance * (1 / n1 + 1 / n2));

    // Calculate t-statistic
    const meanDifference = stats1.mean - stats2.mean;
    const testStatistic = meanDifference / standardError;

    // Degrees of freedom
    const df = n1 + n2 - 2;

    // Calculate p-value (two-tailed approximation)
    const pValue = 2 * (1 - this.tCDF(Math.abs(testStatistic), df));

    // Calculate 95% confidence interval
    const tCritical = 1.96; // Approximation for large samples
    const margin = tCritical * standardError;
    const confidenceInterval: [number, number] = [
      meanDifference - margin,
      meanDifference + margin,
    ];

    return {
      testStatistic,
      pValue,
      degreesOfFreedom: df,
      confidenceInterval,
      significant: pValue < alpha,
      testName: 'Two-sample t-test',
      effectEstimate: meanDifference,
      standardError,
    };
  }

  /**
   * Calculate descriptive statistics
   *
   * @param data - Numeric array
   * @returns Descriptive statistics
   */
  calculateDescriptiveStats(data: number[]): DescriptiveStats {
    if (data.length === 0) {
      throw new ClinicalTrialError(
        ClinicalTrialErrorCode.STATISTICAL_VIOLATION,
        'Cannot calculate statistics for empty dataset'
      );
    }

    const n = data.length;
    const sorted = [...data].sort((a, b) => a - b);

    // Mean
    const mean = data.reduce((sum, val) => sum + val, 0) / n;

    // Standard deviation
    const variance =
      data.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / (n - 1);
    const sd = Math.sqrt(variance);

    // Standard error
    const se = sd / Math.sqrt(n);

    // Median
    const median =
      n % 2 === 0
        ? (sorted[n / 2 - 1] + sorted[n / 2]) / 2
        : sorted[Math.floor(n / 2)];

    // Quartiles
    const q1Index = Math.floor(n * 0.25);
    const q3Index = Math.floor(n * 0.75);
    const q1 = sorted[q1Index];
    const q3 = sorted[q3Index];

    // Min/Max
    const min = sorted[0];
    const max = sorted[n - 1];

    // 95% CI for mean
    const tCritical = 1.96; // Approximation
    const margin = tCritical * se;
    const ci95: [number, number] = [mean - margin, mean + margin];

    return {
      n,
      mean,
      sd,
      se,
      median,
      min,
      max,
      q1,
      q3,
      ci95,
    };
  }

  /**
   * Generate regulatory submission package
   *
   * @param params - Submission parameters
   * @returns Regulatory submission record
   */
  generateSubmission(params: {
    trialId: string;
    agency: RegulatoryAgency;
    format: SubmissionFormat;
    type: 'IND' | 'NDA' | 'BLA' | 'CTA' | 'MAA' | 'OTHER';
    datasets: string[];
  }): RegulatorySubmission {
    const { trialId, agency, format, type, datasets } = params;

    const submissionId = `SUB-${Date.now()}-${Math.random()
      .toString(36)
      .substr(2, 9)}`;

    const submission: RegulatorySubmission = {
      id: submissionId,
      trialId,
      agency,
      format,
      type,
      submissionDate: new Date(),
      datasets,
      status: 'draft',
      comments: [],
    };

    return submission;
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  /**
   * Normal CDF approximation
   */
  private normalCDF(x: number): number {
    // Approximation of standard normal CDF using error function
    const t = 1 / (1 + 0.2316419 * Math.abs(x));
    const d = 0.3989423 * Math.exp((-x * x) / 2);
    const p =
      d *
      t *
      (0.3193815 +
        t * (-0.3565638 + t * (1.781478 + t * (-1.821256 + t * 1.330274))));

    return x > 0 ? 1 - p : p;
  }

  /**
   * Student's t CDF approximation
   */
  private tCDF(t: number, df: number): number {
    // For large df, approximate with normal
    if (df > 30) {
      return this.normalCDF(t);
    }

    // Simple approximation for smaller df
    const x = df / (df + t * t);
    return 1 - 0.5 * Math.pow(x, df / 2);
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Register a trial (standalone function)
 */
export function registerTrial(params: {
  title: string;
  phase: TrialPhase;
  indication: string;
  primaryEndpoint: string;
  sampleSize: number;
  duration?: number;
}): ClinicalTrial {
  const sdk = new ClinicalTrialSDK();
  return sdk.registerTrial(params);
}

/**
 * Collect patient data (standalone function)
 */
export function collectPatientData(
  request: DataCollectionRequest
): DataCollectionResponse {
  const sdk = new ClinicalTrialSDK();
  return sdk.collectPatientData(request);
}

/**
 * Record adverse event (standalone function)
 */
export function recordAdverseEvent(ae: Omit<AdverseEvent, 'aeId'>): AdverseEvent {
  const sdk = new ClinicalTrialSDK();
  return sdk.recordAdverseEvent(ae);
}

/**
 * Calculate sample size (standalone function)
 */
export function calculateSampleSize(
  params: SampleSizeCalculation
): SampleSizeResult {
  const sdk = new ClinicalTrialSDK();
  return sdk.calculateSampleSize(params);
}

/**
 * Calculate statistical power (standalone function)
 */
export function calculatePower(params: PowerCalculation): PowerResult {
  const sdk = new ClinicalTrialSDK();
  return sdk.calculatePower(params);
}

/**
 * Calculate statistics (standalone function)
 */
export function calculateStatistics(params: {
  alpha: number;
  power?: number;
  effectSize: number;
  sampleSize?: number;
  testType: 'one-tailed' | 'two-tailed';
}): PowerResult | SampleSizeResult {
  const sdk = new ClinicalTrialSDK();

  if (params.sampleSize !== undefined && params.power === undefined) {
    // Calculate power
    return sdk.calculatePower({
      sampleSize: params.sampleSize,
      alpha: params.alpha,
      effectSize: params.effectSize,
      testType: params.testType,
    });
  } else if (params.power !== undefined && params.sampleSize === undefined) {
    // Calculate sample size
    return sdk.calculateSampleSize({
      alpha: params.alpha,
      power: params.power,
      effectSize: params.effectSize,
      testType: params.testType,
    });
  } else {
    throw new ClinicalTrialError(
      ClinicalTrialErrorCode.STATISTICAL_VIOLATION,
      'Must specify either sampleSize (to calculate power) or power (to calculate sample size)'
    );
  }
}

/**
 * Generate regulatory submission (standalone function)
 */
export function generateSubmission(params: {
  trialId: string;
  agency: RegulatoryAgency;
  format: SubmissionFormat;
}): RegulatorySubmission {
  const sdk = new ClinicalTrialSDK();
  return sdk.generateSubmission({
    ...params,
    type: 'OTHER',
    datasets: [],
  });
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { ClinicalTrialSDK };
export default ClinicalTrialSDK;
