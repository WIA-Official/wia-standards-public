/**
 * WIA-ALZHEIMERS TypeScript SDK
 * NAD+ Homeostasis Treatment Standard for Alzheimer's Disease
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA - World Certification Industry Association
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

import { v4 as uuidv4 } from 'uuid';
import * as Types from './types';

export * from './types';

// ============================================================================
// Constants
// ============================================================================

export const WIA_ALZHEIMERS_VERSION = '1.0.0';
export const SCHEMA_URL = 'https://wia.live/schemas/alzheimers/v1.0.0';
export const API_BASE_URL = 'https://api.wia.live/v1/alzheimers';

export const NAD_REFERENCE_RANGES = {
  blood: { low: 20, optimal_low: 25, optimal_high: 35, high: 45 },
  csf: { low: 0.5, optimal_low: 1, optimal_high: 2, high: 3 },
  brain: { low: 200, optimal_low: 400, optimal_high: 700, high: 900 },
};

export const NADH_NAD_RATIO_OPTIMAL = { min: 0.1, max: 0.3 };

export const COGNITIVE_CUTOFFS = {
  mmse: { normal: 27, mci: 24, mild: 20, moderate: 10 },
  moca: { normal: 26, mci: 22, mild: 17 },
  cdr_sb: { normal: 0, mci: 0.5, mild: 4.5, moderate: 9.5, severe: 16 },
};

export const BIOMARKER_CUTOFFS = {
  abeta42_40_ratio: { positive: 0.067, borderline: 0.08 },
  plasma_ptau181: { elevated: 20, high: 35 },
  plasma_ptau217: { elevated: 0.4, high: 0.8 },
  plasma_nfl: { elevated: 30 },
};

// ============================================================================
// NAD+ Homeostasis Calculator
// ============================================================================

/**
 * Calculate NAD+ Homeostasis Index
 *
 * Formula: NHI = 0.35×L + 0.25×R + 0.25×S - 0.15×P
 * Where:
 *   L = NAD+ level normalized (0-1)
 *   R = NADH/NAD+ ratio score (1 if optimal, 0 otherwise)
 *   S = Sirtuin activity (0-1)
 *   P = PARP activity (0-1)
 *
 * @param nadLevel - NAD+ concentration in μM
 * @param nadhNadRatio - NADH/NAD+ ratio
 * @param sirtuinActivity - Sirtuin activity (0-1)
 * @param parpActivity - PARP activity (0-1)
 * @param tissue - Tissue type for reference ranges
 * @returns NAD+ Homeostasis Index (0-1)
 */
export function calculateNADHomeostasisIndex(
  nadLevel: number,
  nadhNadRatio: number,
  sirtuinActivity: number,
  parpActivity: number,
  tissue: Types.TissueType = 'blood'
): number {
  const ref = NAD_REFERENCE_RANGES[tissue] || NAD_REFERENCE_RANGES.blood;

  // NAD+ level score (normalized to optimal range)
  const nadScore = Math.min(nadLevel / ref.optimal_high, 1) * 0.35;

  // NADH/NAD+ ratio score (optimal is 0.1-0.3)
  const ratioOptimal = nadhNadRatio >= NADH_NAD_RATIO_OPTIMAL.min &&
                       nadhNadRatio <= NADH_NAD_RATIO_OPTIMAL.max;
  const ratioScore = ratioOptimal
    ? 0.25
    : Math.max(0, 0.25 - Math.abs(nadhNadRatio - 0.2) * 0.5);

  // Sirtuin activity contribution
  const sirtuinScore = sirtuinActivity * 0.25;

  // PARP activity (lower is better for NAD+ preservation)
  const parpScore = Math.max(0, 0.15 - parpActivity * 0.15);

  return Math.min(1, nadScore + ratioScore + sirtuinScore + parpScore);
}

/**
 * Get interpretation of NAD+ Homeostasis Index
 */
export function interpretHomeostasisIndex(
  index: number
): Types.HomeostasisInterpretation {
  if (index >= 0.85) return 'excellent';
  if (index >= 0.70) return 'good';
  if (index >= 0.50) return 'moderate';
  if (index >= 0.30) return 'poor';
  return 'critical';
}

/**
 * Calculate percentile from homeostasis index
 * Based on population distribution data
 */
export function calculatePercentile(index: number): number {
  // Simplified percentile calculation
  return Math.round(index * 100);
}

// ============================================================================
// Cognitive Stage Assessment
// ============================================================================

/**
 * Determine cognitive stage based on assessment scores
 */
export function determineCognitiveStage(
  assessment: Partial<Types.CognitiveAssessment>
): Types.CognitiveStage {
  const { mmse, moca, cdr_sb } = assessment;

  // Check CDR-SB first (most reliable)
  if (cdr_sb?.score !== undefined) {
    if (cdr_sb.score >= COGNITIVE_CUTOFFS.cdr_sb.severe) return 'severe';
    if (cdr_sb.score >= COGNITIVE_CUTOFFS.cdr_sb.moderate) return 'moderate';
    if (cdr_sb.score >= COGNITIVE_CUTOFFS.cdr_sb.mild) return 'mild';
    if (cdr_sb.score >= COGNITIVE_CUTOFFS.cdr_sb.mci) return 'mci';
    return 'preclinical';
  }

  // Check MMSE
  if (mmse?.score !== undefined) {
    if (mmse.score < COGNITIVE_CUTOFFS.mmse.moderate) return 'severe';
    if (mmse.score < COGNITIVE_CUTOFFS.mmse.mild) return 'moderate';
    if (mmse.score < COGNITIVE_CUTOFFS.mmse.mci) return 'mild';
    if (mmse.score < COGNITIVE_CUTOFFS.mmse.normal) return 'mci';
    return 'preclinical';
  }

  // Check MoCA
  if (moca?.score !== undefined) {
    if (moca.score < COGNITIVE_CUTOFFS.moca.mild) return 'moderate';
    if (moca.score < COGNITIVE_CUTOFFS.moca.mci) return 'mild';
    if (moca.score < COGNITIVE_CUTOFFS.moca.normal) return 'mci';
    return 'preclinical';
  }

  return 'preclinical';
}

// ============================================================================
// Biomarker Analysis
// ============================================================================

/**
 * Check if amyloid markers indicate positivity
 */
export function isAmyloidPositive(markers: Types.AmyloidMarkers): boolean {
  if (markers.abeta42_40_ratio?.positive !== undefined) {
    return markers.abeta42_40_ratio.positive;
  }
  if (markers.abeta42_40_ratio?.value !== undefined) {
    return markers.abeta42_40_ratio.value < BIOMARKER_CUTOFFS.abeta42_40_ratio.positive;
  }
  if (markers.pet_amyloid?.positive !== undefined) {
    return markers.pet_amyloid.positive;
  }
  return false;
}

/**
 * Check if tau markers are elevated
 */
export function isTauElevated(markers: Types.TauMarkers): boolean {
  if (markers.plasma_ptau181?.value !== undefined) {
    return markers.plasma_ptau181.value > BIOMARKER_CUTOFFS.plasma_ptau181.elevated;
  }
  if (markers.plasma_ptau217?.value !== undefined) {
    return markers.plasma_ptau217.value > BIOMARKER_CUTOFFS.plasma_ptau217.elevated;
  }
  if (markers.pet_tau?.positive !== undefined) {
    return markers.pet_tau.positive;
  }
  return false;
}

// ============================================================================
// Document Generators
// ============================================================================

/**
 * Create a new NAD+ Homeostasis Index document
 */
export function createNADHomeostasisIndex(
  subjectId: string,
  metabolism: Types.NADMetabolism,
  options?: {
    synthesizingEnzymes?: Types.NADSynthesizingEnzymes;
    consumingEnzymes?: Types.NADConsumingEnzymes;
    mitochondrialFunction?: Types.MitochondrialFunction;
  }
): Types.NADHomeostasisIndex {
  // Calculate homeostasis index
  const sirtuinActivity = options?.consumingEnzymes?.sirt1?.activity ?? 0.7;
  const parpActivity = options?.consumingEnzymes?.parp1?.activity ?? 0.4;

  const index = calculateNADHomeostasisIndex(
    metabolism.nad_plus.value,
    metabolism.nadh_nad_ratio.value,
    sirtuinActivity,
    parpActivity,
    metabolism.nad_plus.tissue
  );

  return {
    version: WIA_ALZHEIMERS_VERSION,
    subject_id: subjectId,
    timestamp: new Date().toISOString(),
    nad_metabolism: metabolism,
    nad_synthesizing_enzymes: options?.synthesizingEnzymes,
    nad_consuming_enzymes: options?.consumingEnzymes,
    mitochondrial_function: options?.mitochondrialFunction,
    composite_score: {
      homeostasis_index: parseFloat(index.toFixed(3)),
      percentile: calculatePercentile(index),
      interpretation: interpretHomeostasisIndex(index),
    },
  };
}

/**
 * Create a comprehensive assessment document
 */
export function createAssessment(
  subject: Types.SubjectInfo,
  data: {
    nadHomeostasis?: Types.NADHomeostasisIndex;
    pathology?: Types.PathologyProfile;
    treatment?: Types.TreatmentResponse;
  }
): Types.AlzheimersAssessment {
  let documentType: Types.AlzheimersAssessment['document_type'] = 'comprehensive_assessment';

  if (data.nadHomeostasis && !data.pathology && !data.treatment) {
    documentType = 'nad_homeostasis_index';
  } else if (data.pathology && !data.nadHomeostasis && !data.treatment) {
    documentType = 'pathology_profile';
  } else if (data.treatment && !data.nadHomeostasis && !data.pathology) {
    documentType = 'treatment_response';
  }

  return {
    schema_version: WIA_ALZHEIMERS_VERSION,
    document_type: documentType,
    subject,
    timestamp: new Date().toISOString(),
    nad_homeostasis: data.nadHomeostasis,
    pathology: data.pathology,
    treatment: data.treatment,
  };
}

// ============================================================================
// Treatment Recommendations
// ============================================================================

/**
 * Generate intervention recommendations based on assessment
 */
export function generateRecommendations(
  assessment: Types.AlzheimersAssessment
): Types.Recommendation[] {
  const recommendations: Types.Recommendation[] = [];

  const nadIndex = assessment.nad_homeostasis?.composite_score.homeostasis_index;
  const stage = assessment.pathology?.cognitive_assessment.stage;

  // NAD+ supplementation recommendation
  if (nadIndex !== undefined && nadIndex < 0.7) {
    recommendations.push({
      type: 'intervention',
      priority: nadIndex < 0.5 ? 'high' : 'medium',
      description: `Consider NAD+ precursor supplementation (NR 500mg/day or NMN 250mg/day). Current homeostasis index: ${nadIndex.toFixed(2)}`,
      evidence_level: 'moderate',
    });
  }

  // Monitoring recommendation
  if (stage && stage !== 'preclinical') {
    recommendations.push({
      type: 'monitoring',
      priority: 'high',
      description: `Regular NAD+ and cognitive monitoring recommended for ${stage} stage. Follow-up interval: ${stage === 'mci' ? 12 : 8} weeks`,
    });
  }

  // Lifestyle recommendation
  recommendations.push({
    type: 'lifestyle',
    priority: 'medium',
    description: 'Aerobic exercise 150 min/week, Mediterranean diet, 7-9 hours sleep to support NAD+ levels',
    evidence_level: 'high',
  });

  // Anti-amyloid consideration
  const amyloidPositive = assessment.pathology?.amyloid_markers &&
    isAmyloidPositive(assessment.pathology.amyloid_markers);

  if (amyloidPositive && (stage === 'mci' || stage === 'mild')) {
    recommendations.push({
      type: 'referral',
      priority: 'high',
      description: 'Consider referral for anti-amyloid therapy evaluation (Lecanemab or Donanemab)',
      evidence_level: 'moderate',
    });
  }

  return recommendations;
}

// ============================================================================
// FHIR Conversion
// ============================================================================

/**
 * Convert NAD+ Homeostasis Index to FHIR Observation
 */
export function toFHIRObservation(
  nadIndex: Types.NADHomeostasisIndex
): Types.FHIRObservation {
  return {
    resourceType: 'Observation',
    id: uuidv4(),
    status: 'final',
    code: {
      coding: [{
        system: 'https://wia.live/codes/alzheimers',
        code: 'NAD-HOMEOSTASIS-INDEX',
        display: 'NAD+ Homeostasis Index',
      }],
    },
    subject: {
      reference: `Patient/${nadIndex.subject_id}`,
    },
    effectiveDateTime: nadIndex.timestamp,
    valueQuantity: {
      value: nadIndex.composite_score.homeostasis_index,
      unit: 'index',
      system: 'https://wia.live/units',
      code: 'homeostasis-index',
    },
    interpretation: [{
      coding: [{
        system: 'https://wia.live/codes/interpretation',
        code: nadIndex.composite_score.interpretation,
        display: nadIndex.composite_score.interpretation.charAt(0).toUpperCase() +
                 nadIndex.composite_score.interpretation.slice(1),
      }],
    }],
    component: [
      {
        code: {
          coding: [{
            system: 'https://wia.live/codes/alzheimers',
            code: 'NAD-PLUS',
            display: 'NAD+ Concentration',
          }],
        },
        valueQuantity: {
          value: nadIndex.nad_metabolism.nad_plus.value,
          unit: nadIndex.nad_metabolism.nad_plus.unit,
        },
      },
      {
        code: {
          coding: [{
            system: 'https://wia.live/codes/alzheimers',
            code: 'NADH-NAD-RATIO',
            display: 'NADH/NAD+ Ratio',
          }],
        },
        valueQuantity: {
          value: nadIndex.nad_metabolism.nadh_nad_ratio.value,
          unit: 'ratio',
        },
      },
    ],
  };
}

// ============================================================================
// Validation
// ============================================================================

/**
 * Validate NAD+ Homeostasis Index data
 */
export function validateNADHomeostasisIndex(
  data: Types.NADHomeostasisIndex
): Types.WIAError[] {
  const errors: Types.WIAError[] = [];

  // Validate NAD+ level
  if (data.nad_metabolism.nad_plus.value <= 0) {
    errors.push({
      code: 'INVALID_NAD_LEVEL',
      message: 'NAD+ level must be positive',
      field: 'nad_metabolism.nad_plus.value',
    });
  }

  // Validate NADH/NAD+ ratio
  const ratio = data.nad_metabolism.nadh_nad_ratio.value;
  if (ratio < 0 || ratio > 2) {
    errors.push({
      code: 'INVALID_RATIO',
      message: 'NADH/NAD+ ratio must be between 0 and 2',
      field: 'nad_metabolism.nadh_nad_ratio.value',
    });
  }

  // Validate homeostasis index
  if (data.composite_score.homeostasis_index < 0 ||
      data.composite_score.homeostasis_index > 1) {
    errors.push({
      code: 'INVALID_INDEX',
      message: 'Homeostasis index must be between 0 and 1',
      field: 'composite_score.homeostasis_index',
    });
  }

  return errors;
}

// ============================================================================
// Default Export
// ============================================================================

export default {
  // Version
  WIA_ALZHEIMERS_VERSION,
  SCHEMA_URL,
  API_BASE_URL,

  // Constants
  NAD_REFERENCE_RANGES,
  NADH_NAD_RATIO_OPTIMAL,
  COGNITIVE_CUTOFFS,
  BIOMARKER_CUTOFFS,

  // Functions
  calculateNADHomeostasisIndex,
  interpretHomeostasisIndex,
  calculatePercentile,
  determineCognitiveStage,
  isAmyloidPositive,
  isTauElevated,
  createNADHomeostasisIndex,
  createAssessment,
  generateRecommendations,
  toFHIRObservation,
  validateNADHomeostasisIndex,
};
