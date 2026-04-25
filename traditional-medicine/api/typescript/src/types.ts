/**
 * WIA-TRADITIONAL-MEDICINE TypeScript Types
 * 弘益人間 - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================
// Constitutional Profile Types
// ============================================

export type TCMConstitutionType =
  | 'qi_deficiency'
  | 'yang_deficiency'
  | 'yin_deficiency'
  | 'phlegm_dampness'
  | 'damp_heat'
  | 'blood_stasis'
  | 'qi_stagnation'
  | 'special_diathesis'
  | 'balanced';

export type SasangConstitutionType =
  | 'taeyang'
  | 'taeeum'
  | 'soyang'
  | 'soeum';

export type AyurvedaDoshaType =
  | 'vata'
  | 'pitta'
  | 'kapha'
  | 'vata-pitta'
  | 'pitta-kapha'
  | 'vata-kapha'
  | 'tridoshic';

export interface TCMConstitutionScore {
  qi_deficiency: number;
  yang_deficiency: number;
  yin_deficiency: number;
  phlegm_dampness: number;
  damp_heat: number;
  blood_stasis: number;
  qi_stagnation: number;
  special_diathesis: number;
  balanced: number;
}

export interface TCMConstitution {
  primary_type: TCMConstitutionType;
  secondary_types: TCMConstitutionType[];
  assessment_method: 'CCMQ' | 'clinical' | 'ai_assisted';
  score: TCMConstitutionScore;
  confidence: number;
}

export interface AyurvedaPrakriti {
  vata: { score: number; subdoshas?: Record<string, number> };
  pitta: { score: number; subdoshas?: Record<string, number> };
  kapha: { score: number; subdoshas?: Record<string, number> };
  dominant: AyurvedaDoshaType;
  assessment_method?: string;
}

export interface SasangConstitution {
  type: SasangConstitutionType;
  confidence: number;
  questionnaire_score?: Record<string, number>;
  facial_analysis?: { ai_confidence: number };
  voice_analysis?: { ai_confidence: number };
}

export interface GenomicCorrelates {
  metabolic_genes?: Record<string, string>;
  immune_genes?: Record<string, string>;
  inflammation_genes?: Record<string, string>;
  pharmacogenomics?: Record<string, string>;
}

export interface MicrobiomeProfile {
  enterotype: 'Bacteroides' | 'Prevotella' | 'Ruminococcus';
  diversity_index: number;
  key_species?: Record<string, number>;
}

export interface ConstitutionalProfile {
  profile_id: string;
  patient_id: string;
  timestamp: string;
  practitioner_id?: string;
  tcm_constitution?: TCMConstitution;
  ayurveda_prakriti?: AyurvedaPrakriti;
  sasang_constitution?: SasangConstitution;
  genomic_correlates?: GenomicCorrelates;
  metabolomic_profile?: Record<string, any>;
  microbiome_profile?: MicrobiomeProfile;
}

// ============================================
// Traditional Diagnosis Types
// ============================================

export type TongueBodyColor = 'pale' | 'light_red' | 'red' | 'dark_red' | 'purple';
export type TongueBodyShape = 'thin' | 'normal' | 'swollen' | 'teeth_marked';
export type TongueCoatingColor = 'white' | 'yellow' | 'gray' | 'black';
export type TongueCoatingThickness = 'thin' | 'normal' | 'thick' | 'peeled';
export type TongueMoisture = 'dry' | 'normal' | 'wet';

export interface TongueDiagnosis {
  body_color: TongueBodyColor;
  body_shape: TongueBodyShape;
  coating_color: TongueCoatingColor;
  coating_thickness: TongueCoatingThickness;
  moisture: TongueMoisture;
  image_url?: string;
  ai_analysis?: {
    model: string;
    confidence: number;
    findings: string[];
  };
}

export interface PulseDiagnosis {
  rate: number;
  rhythm: 'regular' | 'irregular';
  quality: string[];
  positions?: {
    left_cun: { quality: string };
    left_guan: { quality: string };
    left_chi: { quality: string };
    right_cun: { quality: string };
    right_guan: { quality: string };
    right_chi: { quality: string };
  };
  sensor_data?: Record<string, any>;
}

export interface FourExaminations {
  inspection: {
    complexion?: string;
    spirit?: string;
    tongue?: TongueDiagnosis;
    eyes?: Record<string, any>;
    nails?: Record<string, any>;
    skin?: Record<string, any>;
  };
  auscultation_olfaction?: {
    voice?: { quality: string; volume?: string; analysis?: Record<string, any> };
    breathing?: { pattern: string };
    body_odor?: string;
  };
  inquiry: {
    chief_complaint: string;
    onset?: string;
    history?: string[];
    ten_questions?: {
      cold_heat?: string;
      perspiration?: string;
      appetite_thirst?: string;
      urination_defecation?: string;
      sleep?: string;
      pain?: string;
      menstruation?: string;
      emotions?: string;
    };
  };
  palpation?: {
    pulse?: PulseDiagnosis;
    abdominal?: Record<string, any>;
    acupoints?: Record<string, any>;
  };
}

export interface PatternDiagnosis {
  icd11_tm_code: string;
  pattern_name: {
    en: string;
    zh?: string;
    ko?: string;
  };
  zang_fu_involved?: string[];
  pathogenic_factors?: string[];
  qi_blood_status?: Record<string, any>;
  yin_yang_status?: Record<string, any>;
}

export interface TraditionalDiagnosis {
  diagnosis_id: string;
  patient_id: string;
  practitioner_id?: string;
  timestamp: string;
  four_examinations: FourExaminations;
  pattern_diagnosis: PatternDiagnosis;
}

// ============================================
// Herbal Medicine Types
// ============================================

export type HerbTaste = 'sweet' | 'bitter' | 'sour' | 'pungent' | 'salty' | 'bland' | 'astringent';
export type HerbTemperature = 'hot' | 'warm' | 'neutral' | 'cool' | 'cold';
export type ToxicityClass = 'non_toxic' | 'low' | 'moderate' | 'high';
export type PregnancyCategory = 'A' | 'B' | 'C' | 'D' | 'X';

export interface ActiveCompound {
  compound_id?: string;
  name: string;
  concentration_percent?: number;
  targets?: string[];
  pathways?: string[];
}

export interface DrugInteraction {
  drug: string;
  effect: string;
  severity: 'minor' | 'moderate' | 'major';
  mechanism?: string;
  recommendation?: string;
  references?: string[];
}

export interface HerbalMedicine {
  herb_id: string;
  names: {
    scientific: string;
    chinese?: string;
    pinyin?: string;
    korean?: string;
    japanese?: string;
    sanskrit?: string;
    common_en?: string;
  };
  taxonomy?: {
    kingdom: string;
    family: string;
    genus: string;
    species: string;
    dna_barcode?: string;
  };
  traditional_properties: {
    taste: HerbTaste[];
    temperature: HerbTemperature;
    meridians: string[];
    actions: string[];
    indications?: string[];
    contraindications?: string[];
  };
  modern_pharmacology?: {
    active_compounds: ActiveCompound[];
    network_pharmacology?: {
      targets: string[];
      pathways: string[];
      diseases: string[];
    };
  };
  safety?: {
    toxicity_class: ToxicityClass;
    max_daily_dose_g: number;
    pregnancy_category: PregnancyCategory;
    drug_interactions: DrugInteraction[];
    adverse_reactions?: string[];
    heavy_metals?: Record<string, number>;
    pesticide_residues?: Record<string, number>;
  };
  quality_standards?: {
    pharmacopoeia: string[];
    marker_compounds: string[];
    authentication_method: string[];
  };
}

export interface FormulaComposition {
  herb: string;
  pinyin?: string;
  dose_g: number;
  role?: 'monarch' | 'minister' | 'assistant' | 'guide';
}

export interface HerbalFormula {
  formula_id?: string;
  name_zh: string;
  name_pinyin: string;
  name_en: string;
  composition: FormulaComposition[];
  actions?: string[];
  indications?: string[];
  modifications?: string[];
}

// ============================================
// API Response Types
// ============================================

export interface ConstitutionAssessmentRequest {
  patient_id: string;
  system: 'tcm' | 'ayurveda' | 'sasang';
  questionnaire_type: 'CCMQ' | 'prakriti' | 'QSCCII';
  responses: Record<string, number>;
  include_recommendations?: boolean;
}

export interface ConstitutionAssessmentResponse {
  profile_id: string;
  timestamp: string;
  primary_constitution: string;
  secondary_constitutions?: string[];
  scores: Record<string, number>;
  confidence: number;
  recommendations?: {
    diet: string[];
    lifestyle: string[];
    herbs_to_consider?: string[];
  };
}

export interface TongueAnalysisResponse {
  analysis_id: string;
  image_quality?: { score: number; issues: string[] };
  findings: TongueDiagnosis;
  patterns_suggested: Array<{
    pattern: string;
    confidence: number;
    evidence?: string[];
  }>;
  icd11_tm_codes: string[];
}

export interface FormulaRecommendation {
  formula: HerbalFormula;
  match_score: number;
  modifications_suggested?: string[];
  safety_check: {
    status: 'safe' | 'caution' | 'contraindicated';
    interactions: DrugInteraction[];
    warnings: string[];
  };
}

export interface InteractionCheckResponse {
  interactions: Array<{
    type: 'herb_drug' | 'herb_herb';
    herb: string;
    drug?: string;
    other_herb?: string;
    severity: 'minor' | 'moderate' | 'major';
    effect: string;
    mechanism?: string;
    recommendation: string;
    evidence_level?: string;
    references?: string[];
  }>;
  overall_risk: 'low' | 'moderate' | 'high';
  summary?: string;
  alternatives?: Array<{
    replace: string;
    with: string;
    reason: string;
  }>;
}

// ============================================
// FHIR Integration Types
// ============================================

export interface FHIRCondition {
  resourceType: 'Condition';
  id: string;
  meta?: {
    profile: string[];
  };
  clinicalStatus: {
    coding: Array<{ system: string; code: string }>;
  };
  code: {
    coding: Array<{
      system: string;
      code: string;
      display: string;
    }>;
    text?: string;
  };
  subject: { reference: string };
  recordedDate?: string;
}

export interface FHIRObservation {
  resourceType: 'Observation';
  id: string;
  status: 'final' | 'preliminary';
  code: {
    coding: Array<{
      system: string;
      code: string;
      display: string;
    }>;
  };
  subject: { reference: string };
  valueCodeableConcept?: {
    coding: Array<{ system: string; code: string; display: string }>;
  };
  component?: Array<{
    code: { coding: Array<{ system: string; code: string }> };
    valueQuantity?: { value: number; unit: string };
    valueString?: string;
  }>;
}
