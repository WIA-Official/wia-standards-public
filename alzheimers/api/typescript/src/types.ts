/**
 * WIA-ALZHEIMERS TypeScript Types
 * NAD+ Homeostasis Treatment Standard for Alzheimer's Disease
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA - World Certification Industry Association
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 */

// ============================================================================
// Core Types
// ============================================================================

export type UUID = string;
export type ISO8601 = string;

export type TissueType = 'blood' | 'csf' | 'brain' | 'plasma';
export type CognitiveStage = 'preclinical' | 'mci' | 'mild' | 'moderate' | 'severe';
export type HomeostasisInterpretation = 'excellent' | 'good' | 'moderate' | 'poor' | 'critical';
export type ResponderStatus = 'full' | 'partial' | 'non' | 'worsened';
export type InterventionType = 'nad_precursor' | 'anti_amyloid' | 'lifestyle' | 'combination';
export type NADPrecursor = 'NR' | 'NMN' | 'niacin' | 'P7C3-A20';
export type AntiAmyloidAgent = 'lecanemab' | 'donanemab' | 'aducanumab';

// ============================================================================
// Subject Information
// ============================================================================

export interface SubjectInfo {
  id: UUID;
  age: number;
  sex: 'male' | 'female' | 'other';
  apoe_genotype?: 'e2/e2' | 'e2/e3' | 'e2/e4' | 'e3/e3' | 'e3/e4' | 'e4/e4';
  ethnicity?: string;
  clinical_site?: string;
}

// ============================================================================
// Measurement Types
// ============================================================================

export interface Measurement {
  value: number;
  unit: string;
  tissue?: TissueType;
  method?: string;
  timestamp?: ISO8601;
}

export interface BiomarkerMeasurement {
  value: number;
  unit: string;
  reference_range?: {
    low: number;
    high: number;
  };
  interpretation?: 'normal' | 'borderline' | 'abnormal' | 'critical';
}

export interface EnzymeProfile {
  activity: number; // 0-1 normalized
  expression?: number | 'low' | 'normal' | 'elevated' | 'high';
  protein_level?: number;
}

// ============================================================================
// NAD+ Homeostasis Index
// ============================================================================

export interface NADMetabolism {
  nad_plus: Measurement;
  nadh: Measurement;
  nadh_nad_ratio: {
    value: number;
    optimal_range: [number, number];
  };
  nadp_plus?: Measurement;
  nadph?: Measurement;
  nicotinamide?: Measurement;
  nmn?: Measurement;
  nr?: Measurement;
}

export interface NADSynthesizingEnzymes {
  nampt?: EnzymeProfile;
  nadsyn1?: EnzymeProfile;
  nmnat1?: EnzymeProfile;
  nmnat2?: EnzymeProfile;
  nmnat3?: EnzymeProfile;
}

export interface NADConsumingEnzymes {
  sirt1?: EnzymeProfile;
  sirt2?: EnzymeProfile;
  sirt3?: EnzymeProfile;
  sirt4?: EnzymeProfile;
  sirt5?: EnzymeProfile;
  sirt6?: EnzymeProfile;
  sirt7?: EnzymeProfile;
  parp1?: EnzymeProfile;
  parp2?: EnzymeProfile;
  parp4?: EnzymeProfile;
  nadk2?: EnzymeProfile;
  cd38?: EnzymeProfile;
}

export interface MitochondrialFunction {
  membrane_potential_mv: number;
  atp_production?: Measurement;
  ros_level?: {
    value: number;
    max: number;
  };
  oxidative_stress?: {
    value: number;
    max: number;
  };
  upmt_activation?: {
    atf4: number;
    hsp60: number;
    lonp1: number;
  };
}

export interface CompositeScore {
  homeostasis_index: number;
  percentile?: number;
  interpretation: HomeostasisInterpretation;
}

export interface NADHomeostasisIndex {
  version: string;
  subject_id: UUID;
  timestamp: ISO8601;
  nad_metabolism: NADMetabolism;
  nad_synthesizing_enzymes?: NADSynthesizingEnzymes;
  nad_consuming_enzymes?: NADConsumingEnzymes;
  mitochondrial_function?: MitochondrialFunction;
  composite_score: CompositeScore;
}

// ============================================================================
// Cognitive Assessment
// ============================================================================

export interface CognitiveAssessment {
  mmse?: {
    score: number;
    max: 30;
  };
  moca?: {
    score: number;
    max: 30;
  };
  cdr_sb?: {
    score: number;
    baseline?: number;
  };
  adas_cog?: {
    score: number;
  };
  stage: CognitiveStage;
}

// ============================================================================
// Biomarkers
// ============================================================================

export interface AmyloidMarkers {
  plasma_abeta42?: BiomarkerMeasurement;
  plasma_abeta40?: BiomarkerMeasurement;
  abeta42_40_ratio?: {
    value: number;
    cutoff: number;
    positive: boolean;
  };
  csf_abeta42?: BiomarkerMeasurement;
  pet_amyloid?: {
    centiloid: number;
    positive: boolean;
  };
}

export interface TauMarkers {
  plasma_ptau181?: BiomarkerMeasurement;
  plasma_ptau217?: BiomarkerMeasurement;
  csf_total_tau?: BiomarkerMeasurement;
  csf_ptau181?: BiomarkerMeasurement;
  pet_tau?: {
    suvr: number;
    positive: boolean;
  };
}

export interface NeurodegenerationMarkers {
  plasma_nfl?: BiomarkerMeasurement;
  plasma_gfap?: BiomarkerMeasurement;
  csf_nfl?: BiomarkerMeasurement;
  hippocampal_volume?: {
    value: number;
    unit: string;
    atrophy_percent: number;
  };
  cortical_thickness?: Measurement;
}

export interface Neuroinflammation {
  il6?: BiomarkerMeasurement;
  tnf_alpha?: BiomarkerMeasurement;
  ykl40?: BiomarkerMeasurement;
  trem2?: BiomarkerMeasurement;
  reactive_astrocytes?: number;
  activated_microglia?: number;
}

export interface BloodBrainBarrier {
  integrity_score: number;
  albumin_ratio?: number;
  tight_junction_proteins?: {
    claudin5: number;
    occludin: number;
  };
}

export interface SynapticFunction {
  neurogranin?: BiomarkerMeasurement;
  snap25?: BiomarkerMeasurement;
  synaptic_density?: number;
  ltp_capacity?: number;
}

// ============================================================================
// Pathology Profile
// ============================================================================

export interface PathologyProfile {
  cognitive_assessment: CognitiveAssessment;
  amyloid_markers?: AmyloidMarkers;
  tau_markers?: TauMarkers;
  neurodegeneration_markers?: NeurodegenerationMarkers;
  neuroinflammation?: Neuroinflammation;
  blood_brain_barrier?: BloodBrainBarrier;
  synaptic_function?: SynapticFunction;
}

// ============================================================================
// Treatment Response
// ============================================================================

export interface Intervention {
  type: InterventionType;
  agent: string;
  dosage: {
    value: number;
    unit: string;
    frequency: string;
  };
  duration_days?: number;
  started_at: ISO8601;
  ended_at?: ISO8601;
}

export interface FollowUp {
  timepoint_days: number;
  nad_homeostasis_index?: number;
  cognitive_change?: number;
  biomarker_change?: Record<string, number>;
  adverse_events?: string[];
}

export interface TreatmentOutcomes {
  responder_status: ResponderStatus;
  cognitive_improvement_percent?: number;
  nad_recovery_percent?: number;
  time_to_response_days?: number;
}

export interface TreatmentResponse {
  intervention: Intervention;
  baseline: {
    nad_homeostasis_index: number;
    cognitive_score: number;
    biomarkers: Record<string, number>;
  };
  follow_up: FollowUp[];
  outcomes?: TreatmentOutcomes;
}

// ============================================================================
// Complete Assessment Document
// ============================================================================

export interface AlzheimersAssessment {
  schema_version: string;
  document_type: 'nad_homeostasis_index' | 'pathology_profile' | 'treatment_response' | 'comprehensive_assessment';
  subject: SubjectInfo;
  timestamp: ISO8601;
  nad_homeostasis?: NADHomeostasisIndex;
  pathology?: PathologyProfile;
  treatment?: TreatmentResponse;
}

// ============================================================================
// API Response Types
// ============================================================================

export interface AssessmentResponse {
  assessment_id: string;
  subject_id: UUID;
  timestamp: ISO8601;
  nad_homeostasis_index: CompositeScore;
  cognitive_stage: CognitiveStage;
  risk_assessment: {
    progression_risk: 'low' | 'moderate' | 'high';
    recommended_follow_up_days: number;
  };
  recommendations: Recommendation[];
}

export interface Recommendation {
  type: 'intervention' | 'monitoring' | 'lifestyle' | 'referral';
  priority: 'low' | 'medium' | 'high';
  description: string;
  evidence_level?: 'high' | 'moderate' | 'low';
}

export interface InterventionRecommendation {
  recommendation_id: string;
  subject_id: UUID;
  timestamp: ISO8601;
  primary_intervention: {
    type: InterventionType;
    agent: string;
    dosage: {
      value: number;
      unit: string;
      frequency: string;
    };
    duration: string;
    expected_outcomes: {
      nad_increase_percent: number;
      cognitive_improvement_percent: number;
      time_to_response_weeks: number;
    };
    evidence_level: 'high' | 'moderate' | 'low';
    references: string[];
  };
  secondary_interventions: Array<{
    type: string;
    description: string;
    expected_benefit: string;
  }>;
  monitoring_protocol: {
    initial_assessment_weeks: number;
    follow_up_interval_weeks: number;
    required_tests: string[];
  };
}

// ============================================================================
// FHIR Compatibility Types
// ============================================================================

export interface FHIRObservation {
  resourceType: 'Observation';
  id?: string;
  status: 'final' | 'preliminary' | 'registered' | 'cancelled';
  code: {
    coding: Array<{
      system: string;
      code: string;
      display: string;
    }>;
  };
  subject?: {
    reference: string;
  };
  effectiveDateTime?: ISO8601;
  valueQuantity?: {
    value: number;
    unit: string;
    system?: string;
    code?: string;
  };
  interpretation?: Array<{
    coding: Array<{
      system: string;
      code: string;
      display: string;
    }>;
  }>;
  component?: Array<{
    code: {
      coding: Array<{
        system: string;
        code: string;
        display: string;
      }>;
    };
    valueQuantity?: {
      value: number;
      unit: string;
    };
  }>;
}

// ============================================================================
// Certification Types
// ============================================================================

export type CertificationLevel = 1 | 2 | 3 | 4;

export interface CertificationInfo {
  level: CertificationLevel;
  level_name: 'Basic Screening' | 'Comprehensive Assessment' | 'Precision Diagnosis' | 'Research Grade';
  granted_date: ISO8601;
  valid_until: ISO8601;
  organization: string;
  capabilities: string[];
}

// ============================================================================
// Error Types
// ============================================================================

export interface WIAError {
  code: string;
  message: string;
  field?: string;
  details?: Record<string, unknown>;
}

export interface APIErrorResponse {
  error: WIAError;
}
