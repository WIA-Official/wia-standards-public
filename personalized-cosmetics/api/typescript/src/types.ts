/**
 * WIA-IND-006: Personalized Cosmetics Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry & Biotech Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Skin Type Classifications
// ============================================================================

/**
 * Baumann Skin Type System (16 types)
 */
export type BaumannOiliness = 'O' | 'D'; // Oily or Dry
export type BaumannSensitivity = 'S' | 'R'; // Sensitive or Resistant
export type BaumannPigmentation = 'P' | 'N'; // Pigmented or Non-pigmented
export type BaumannWrinkles = 'W' | 'T'; // Wrinkle-prone or Tight

export type BaumannType =
  | 'DSNW' | 'DSNT' | 'DSPW' | 'DSPT'
  | 'DRNW' | 'DRNT' | 'DRPW' | 'DRPT'
  | 'OSNW' | 'OSNT' | 'OSPW' | 'OSPT'
  | 'ORNW' | 'ORNT' | 'ORPW' | 'ORPT';

/**
 * Fitzpatrick Phototype (6 types)
 */
export type FitzpatrickType = 'I' | 'II' | 'III' | 'IV' | 'V' | 'VI';

/**
 * Skin classification categories
 */
export type SkinClassification = 'very-dry' | 'dry' | 'normal' | 'combination' | 'oily' | 'very-oily';
export type SensitivityLevel = 'resistant' | 'mildly-sensitive' | 'moderately-sensitive' | 'highly-sensitive';
export type PoreSize = 'micro' | 'small' | 'medium' | 'large' | 'macro';

// ============================================================================
// Measurement Types
// ============================================================================

/**
 * Hydration measurement (Corneometer)
 */
export interface HydrationMeasurement {
  /** Forehead measurement in arbitrary units */
  forehead: number;
  /** Left cheek measurement */
  cheekLeft: number;
  /** Right cheek measurement */
  cheekRight: number;
  /** Jawline measurement (optional) */
  jawline?: number;
  /** Average hydration across all sites */
  average: number;
  /** Unit of measurement */
  unit: 'AU'; // Arbitrary Units
  /** Classification based on measurement */
  classification: 'severely-dehydrated' | 'dehydrated' | 'normal' | 'well-hydrated' | 'optimally-hydrated';
}

/**
 * Elasticity measurement (Cutometer)
 */
export interface ElasticityMeasurement {
  /** Cheek elasticity score (0-100) */
  cheek: number;
  /** Crow's feet area score */
  crowsFeet: number;
  /** Forehead score (optional) */
  forehead?: number;
  /** Average elasticity */
  average: number;
  /** Unit of measurement */
  unit: 'score';
  /** Classification */
  classification: 'poor' | 'fair' | 'good' | 'excellent';
  /** Immediate elastic recovery (mm) */
  immediateRecovery?: number;
  /** Final distension (mm) */
  finalDistension?: number;
}

/**
 * Sebum/Oil measurement (Sebumeter)
 */
export interface SebumMeasurement {
  /** T-zone (forehead, nose) measurement */
  tZone: number;
  /** Cheek measurement */
  cheek: number;
  /** Chin measurement (optional) */
  chin?: number;
  /** Unit of measurement */
  unit: 'μg/cm²';
  /** Skin type classification */
  classification: SkinClassification;
}

/**
 * Melanin measurement (Mexameter)
 */
export interface MelaninMeasurement {
  /** Average cheek melanin */
  cheek: number;
  /** Dark spot measurements (if present) */
  spots?: number[];
  /** Melanin uniformity score (0-100, higher = more uniform) */
  uniformity: number;
  /** Unit of measurement */
  unit: 'MI'; // Melanin Index
}

/**
 * Erythema/Redness measurement (Mexameter)
 */
export interface ErythemaMeasurement {
  /** Average cheek erythema */
  cheek: number;
  /** Redness in specific areas */
  tZone?: number;
  nose?: number;
  chin?: number;
  /** Unit of measurement */
  unit: 'EI'; // Erythema Index
  /** Classification */
  classification: 'minimal' | 'mild' | 'moderate' | 'severe';
}

/**
 * Trans-Epidermal Water Loss (TEWL)
 */
export interface TEWLMeasurement {
  /** Cheek TEWL */
  cheek: number;
  /** Forehead TEWL (optional) */
  forehead?: number;
  /** Unit of measurement */
  unit: 'g/m²/h';
  /** Barrier function status */
  barrierStatus: 'excellent' | 'good' | 'compromised' | 'severely-compromised';
}

/**
 * Skin pH measurement
 */
export interface PHMeasurement {
  /** Cheek pH */
  cheek: number;
  /** Forehead pH (optional) */
  forehead?: number;
  /** Status relative to optimal (4.5-5.5) */
  status: 'too-acidic' | 'optimal-acidic' | 'neutral' | 'alkaline';
}

/**
 * Complete instrumental measurements
 */
export interface InstrumentalMeasurements {
  hydration: HydrationMeasurement;
  elasticity: ElasticityMeasurement;
  sebum: SebumMeasurement;
  melanin: MelaninMeasurement;
  erythema: ErythemaMeasurement;
  tewl: TEWLMeasurement;
  pH: PHMeasurement;
  /** Measurement timestamp */
  timestamp: string; // ISO 8601
}

// ============================================================================
// AI Analysis Types
// ============================================================================

/**
 * Wrinkle analysis from AI
 */
export interface WrinkleAnalysis {
  /** Total wrinkle count */
  count: number;
  /** Average wrinkle depth (0-1 scale) */
  averageDepth: number;
  /** Maximum wrinkle depth */
  maxDepth?: number;
  /** Wrinkle locations */
  locations: Array<'forehead' | 'crowsFeet' | 'nasolabialFolds' | 'marionette' | 'underEye' | 'lips'>;
  /** Overall severity */
  severity: 'minimal' | 'mild' | 'moderate' | 'severe';
  /** Confidence score (0-1) */
  confidence: number;
}

/**
 * Pigmentation analysis from AI
 */
export interface PigmentationAnalysis {
  /** Number of dark spots detected */
  spotCount: number;
  /** Total area coverage (%) */
  coverage: number;
  /** Skin tone uniformity (0-100, higher = more uniform) */
  uniformity: number;
  /** Spot locations */
  spotLocations?: Array<{ x: number; y: number; size: number; intensity: number }>;
  /** Melasma detection */
  melasmaDetected: boolean;
  /** Freckles detection */
  frecklesDetected: boolean;
}

/**
 * Pore analysis from AI
 */
export interface PoreAnalysis {
  /** Average pore size */
  size: PoreSize;
  /** Visibility level */
  visibility: 'minimal' | 'low' | 'moderate' | 'high' | 'very-high';
  /** Pore density (per cm²) */
  density: number;
  /** T-zone pore status */
  tZone: 'normal' | 'enlarged' | 'clogged';
  /** Cheek pore status */
  cheeks: 'normal' | 'enlarged' | 'clogged';
}

/**
 * Texture analysis from AI
 */
export interface TextureAnalysis {
  /** Roughness score (0-100, lower = smoother) */
  roughness: number;
  /** Smoothness score (0-100, higher = smoother) */
  smoothness: number;
  /** Skin surface uniformity */
  uniformity: number;
  /** Acne scars detected */
  acneScars: boolean;
  /** Acne scar severity */
  scarSeverity?: 'mild' | 'moderate' | 'severe';
}

/**
 * Acne analysis from AI
 */
export interface AcneAnalysis {
  /** Active lesion count */
  activeLesions: number;
  /** Comedone (blackhead/whitehead) count */
  comedones: number;
  /** Inflammatory lesions (papules/pustules) */
  inflammatory: number;
  /** Acne severity grade */
  severity: 'clear' | 'mild' | 'moderate' | 'severe' | 'very-severe';
  /** Post-inflammatory hyperpigmentation (PIH) */
  pihPresent: boolean;
}

/**
 * Complete AI-powered visual analysis
 */
export interface AIVisualAnalysis {
  wrinkles: WrinkleAnalysis;
  pigmentation: PigmentationAnalysis;
  pores: PoreAnalysis;
  texture: TextureAnalysis;
  acne?: AcneAnalysis;
  /** Overall skin quality score (0-100) */
  overallQuality: number;
  /** Analysis timestamp */
  timestamp: string;
}

// ============================================================================
// Questionnaire Types
// ============================================================================

/**
 * User demographics
 */
export interface Demographics {
  age: number;
  gender: 'female' | 'male' | 'non-binary' | 'prefer-not-to-say';
  ethnicity?: string;
  location: {
    country: string;
    city?: string;
    climate: ClimateZone;
  };
}

/**
 * Climate zones
 */
export type ClimateZone =
  | 'tropical-humid'
  | 'tropical-dry'
  | 'desert-arid'
  | 'temperate'
  | 'continental'
  | 'arctic-cold'
  | 'subtropical';

/**
 * Lifestyle factors
 */
export interface LifestyleFactors {
  /** Sun exposure hours per week */
  sunExposure: number;
  /** Smoking status */
  smoking: boolean;
  /** Alcohol consumption (drinks per week) */
  alcoholConsumption: number;
  /** Sleep hours per night */
  sleep: number;
  /** Stress level (1-10) */
  stress: number;
  /** Diet quality (1-10) */
  dietQuality: number;
  /** Water intake (liters per day) */
  waterIntake: number;
  /** Exercise frequency (hours per week) */
  exercise: number;
  /** Occupation type */
  occupation: 'indoor' | 'outdoor' | 'mixed';
  /** Pollution exposure */
  pollution: 'low' | 'moderate' | 'high' | 'very-high';
}

/**
 * Skin concerns (user-reported)
 */
export type SkinConcern =
  | 'anti-aging'
  | 'wrinkles'
  | 'fine-lines'
  | 'sagging'
  | 'hyperpigmentation'
  | 'dark-spots'
  | 'melasma'
  | 'uneven-tone'
  | 'acne'
  | 'acne-scars'
  | 'blackheads'
  | 'whiteheads'
  | 'dryness'
  | 'dehydration'
  | 'sensitivity'
  | 'redness'
  | 'rosacea'
  | 'irritation'
  | 'large-pores'
  | 'rough-texture'
  | 'oiliness'
  | 'dullness'
  | 'dark-circles'
  | 'puffiness';

/**
 * Medical history
 */
export interface MedicalHistory {
  /** Skin conditions */
  skinConditions: Array<'eczema' | 'psoriasis' | 'rosacea' | 'acne' | 'melasma' | 'vitiligo'>;
  /** Known allergies to ingredients */
  allergies: string[];
  /** Current medications affecting skin */
  medications: Array<{
    name: string;
    type: 'oral-retinoid' | 'topical-retinoid' | 'antibiotic' | 'hormonal' | 'other';
  }>;
  /** Pregnancy status */
  pregnant: boolean;
  /** Breastfeeding status */
  breastfeeding: boolean;
}

/**
 * Current skincare routine
 */
export interface CurrentSkincare {
  /** Products currently used */
  products: Array<{
    category: ProductType;
    brand?: string;
    activeIngredients?: string[];
    frequency: 'daily-am' | 'daily-pm' | 'daily-both' | 'alternate-days' | 'weekly';
  }>;
  /** Satisfaction level (1-10) */
  satisfaction: number;
  /** Duration of current routine (months) */
  durationMonths: number;
  /** Adverse reactions experienced */
  adverseReactions: string[];
}

/**
 * Complete questionnaire data
 */
export interface QuestionnaireData {
  demographics: Demographics;
  lifestyle: LifestyleFactors;
  skinConcerns: SkinConcern[];
  /** Priority ranking of concerns (1 = highest priority) */
  concernPriority: { [key in SkinConcern]?: number };
  medicalHistory: MedicalHistory;
  currentSkincare: CurrentSkincare;
}

// ============================================================================
// Genetic Analysis Types
// ============================================================================

/**
 * Genetic variant types
 */
export type GeneticVariant = string; // e.g., "GG", "GT", "TT", "wild-type"

/**
 * Genetic markers for skin aging
 */
export interface AgingGenes {
  /** Collagen Type I Alpha 1 (collagen production) */
  COL1A1?: GeneticVariant;
  /** Collagen Type III Alpha 1 */
  COL3A1?: GeneticVariant;
  /** Matrix Metalloproteinase 1 (collagen degradation) */
  MMP1?: GeneticVariant;
  /** Matrix Metalloproteinase 3 */
  MMP3?: GeneticVariant;
  /** Elastin gene */
  ELN?: GeneticVariant;
  /** Superoxide Dismutase 2 (antioxidant) */
  SOD2?: GeneticVariant;
}

/**
 * Genetic markers for pigmentation
 */
export interface PigmentationGenes {
  /** Melanocortin 1 Receptor (melanin production, UV sensitivity) */
  MC1R?: GeneticVariant;
  /** Tyrosinase (melanin synthesis enzyme) */
  TYR?: GeneticVariant;
  /** Tyrosinase-related protein 1 */
  TYRP1?: GeneticVariant;
  /** Solute Carrier Family 24 Member 5 (skin color) */
  SLC24A5?: GeneticVariant;
  /** Solute Carrier Family 45 Member 2 */
  SLC45A2?: GeneticVariant;
}

/**
 * Genetic markers for skin barrier
 */
export interface BarrierGenes {
  /** Filaggrin (skin barrier protein, NMF) */
  FLG?: GeneticVariant;
  /** Interleukin 1 Alpha (inflammation) */
  IL1A?: GeneticVariant;
  /** Glutathione S-Transferase P1 (detoxification) */
  GSTP1?: GeneticVariant;
  /** Vitamin D Receptor (sun sensitivity) */
  VDR?: GeneticVariant;
}

/**
 * Genetic risk scores
 */
export interface GeneticRiskScores {
  /** Aging risk score (0-100) */
  agingRisk: {
    score: number;
    level: 'low' | 'moderate' | 'high' | 'very-high';
    biologicalSkinAge: number;
  };
  /** Pigmentation risk score */
  pigmentationRisk: {
    score: number;
    level: 'low' | 'moderate' | 'high' | 'very-high';
  };
  /** Sensitivity risk score */
  sensitivityRisk: {
    score: number;
    level: 'low' | 'moderate' | 'high' | 'very-high';
  };
  /** Barrier health score */
  barrierHealth: {
    score: number;
    level: 'excellent' | 'good' | 'fair' | 'poor';
  };
}

/**
 * Complete genetic profile
 */
export interface GeneticProfile {
  agingGenes: AgingGenes;
  pigmentationGenes: PigmentationGenes;
  barrierGenes: BarrierGenes;
  riskScores: GeneticRiskScores;
  /** Raw genetic data file reference */
  rawDataFile?: string;
  /** Analysis date */
  analysisDate: string;
  /** Lab/provider */
  provider?: string;
}

// ============================================================================
// Skin Profile Types
// ============================================================================

/**
 * Complete skin profile
 */
export interface SkinProfile {
  /** Unique profile ID */
  skinProfileID: string;
  /** User ID */
  userID: string;
  /** Profile creation date */
  createdDate: string;
  /** Last updated date */
  updatedDate?: string;
  /** Baumann classification */
  baumann: {
    type: BaumannType;
    oiliness: BaumannOiliness;
    sensitivity: BaumannSensitivity;
    pigmentation: BaumannPigmentation;
    wrinkles: BaumannWrinkles;
    /** Classification confidence (0-1) */
    confidence: number;
  };
  /** Fitzpatrick classification */
  fitzpatrick: {
    type: FitzpatrickType;
    melaninIndex: number;
    uvSensitivity: 'very-high' | 'high' | 'moderate' | 'low';
  };
  /** Instrumental measurements */
  measurements: InstrumentalMeasurements;
  /** AI visual analysis */
  aiAnalysis: AIVisualAnalysis;
  /** Questionnaire data */
  questionnaire: QuestionnaireData;
  /** Genetic data (optional) */
  geneticData?: GeneticProfile;
  /** Overall skin health score (0-100) */
  skinHealthScore: number;
}

// ============================================================================
// Ingredient Types
// ============================================================================

/**
 * INCI ingredient categories
 */
export type IngredientCategory =
  | 'active'
  | 'emollient'
  | 'humectant'
  | 'emulsifier'
  | 'preservative'
  | 'antioxidant'
  | 'thickener'
  | 'solvent'
  | 'ph-adjuster'
  | 'fragrance'
  | 'colorant'
  | 'other';

/**
 * Active ingredient types
 */
export type ActiveType =
  | 'retinoid'
  | 'vitamin-c'
  | 'aha'
  | 'bha'
  | 'peptide'
  | 'niacinamide'
  | 'antioxidant'
  | 'brightening'
  | 'exfoliant'
  | 'soothing'
  | 'anti-inflammatory'
  | 'hydrating';

/**
 * Ingredient definition
 */
export interface Ingredient {
  /** INCI name */
  inci: string;
  /** Common name */
  commonName?: string;
  /** CAS number */
  cas?: string;
  /** Concentration percentage */
  percentage: number;
  /** Functional category */
  category: IngredientCategory;
  /** Active type (if active) */
  activeType?: ActiveType;
  /** Target skin concerns */
  targetConcerns?: SkinConcern[];
  /** Phase in formulation */
  phase: 'A' | 'B' | 'C' | 'D'; // A=water, B=oil, C=heat-sensitive, D=post-emulsion
  /** Known allergen */
  isAllergen: boolean;
  /** Pregnancy safety */
  pregnancySafe: boolean;
  /** Stability notes */
  stability?: string;
}

// ============================================================================
// Formulation Types
// ============================================================================

/**
 * Product types
 */
export type ProductType =
  | 'cleanser'
  | 'toner'
  | 'essence'
  | 'serum'
  | 'ampoule'
  | 'moisturizer'
  | 'cream'
  | 'lotion'
  | 'gel'
  | 'oil'
  | 'balm'
  | 'mask'
  | 'eye-cream'
  | 'sunscreen'
  | 'exfoliant'
  | 'peel';

/**
 * Texture preferences
 */
export type TexturePreference =
  | 'gel'
  | 'lightweight'
  | 'medium'
  | 'rich'
  | 'balm'
  | 'oil'
  | 'cream'
  | 'lotion';

/**
 * Preservative system types
 */
export type PreservativeSystem =
  | 'phenoxyethanol'
  | 'paraben-free'
  | 'ecocert'
  | 'natural'
  | 'conventional';

/**
 * Budget tiers
 */
export type BudgetTier = 'economy' | 'mid-range' | 'premium' | 'luxury';

/**
 * Formulation preferences
 */
export interface FormulationPreferences {
  /** Texture preference */
  texture: TexturePreference;
  /** Scent preference */
  scent: 'none' | 'light' | 'moderate';
  /** Preservative system */
  preservativeSystem: PreservativeSystem;
  /** Budget tier */
  budgetTier: BudgetTier;
  /** Vegan requirement */
  vegan?: boolean;
  /** Cruelty-free requirement */
  crueltyFree?: boolean;
  /** Organic preference */
  organic?: boolean;
  /** Fragrance-free requirement */
  fragranceFree?: boolean;
}

/**
 * Custom formulation
 */
export interface Formulation {
  /** Unique formulation ID */
  formulationID: string;
  /** Associated skin profile */
  skinProfileID: string;
  /** Product type */
  productType: ProductType;
  /** Target concerns */
  targetConcerns: SkinConcern[];
  /** Formulation phases */
  phaseA: Ingredient[]; // Water phase
  phaseB: Ingredient[]; // Oil phase
  phaseC: Ingredient[]; // Heat-sensitive actives
  phaseD?: Ingredient[]; // Post-emulsion additions
  /** Complete ingredient list (all phases combined) */
  fullIngredientList: Ingredient[];
  /** Formulation properties */
  properties: {
    /** Target pH */
    targetPH: number;
    /** Texture description */
    texture: string;
    /** Scent description */
    scent: string;
    /** Color */
    color: string;
    /** Viscosity (cP) */
    viscosity?: number;
  };
  /** Predicted scores */
  predictions: {
    /** Efficacy score (0-100) */
    efficacyScore: number;
    /** Ingredient compatibility (0-100) */
    compatibilityScore: number;
    /** Stability score (0-100) */
    stabilityScore: number;
    /** Safety score (0-100) */
    safetyScore: number;
  };
  /** Production details */
  production: {
    /** Estimated production time (hours) */
    estimatedTime: number;
    /** Shelf life (months) */
    shelfLife: number;
    /** Packaging type */
    packaging: string;
    /** Batch size (mL) */
    batchSize?: number;
  };
  /** Creation timestamp */
  createdDate: string;
}

// ============================================================================
// Manufacturing Types
// ============================================================================

/**
 * Batch status
 */
export type BatchStatus =
  | 'pending'
  | 'in-production'
  | 'quality-control'
  | 'passed'
  | 'failed'
  | 'shipped'
  | 'delivered';

/**
 * Quality control test result
 */
export interface QCTestResult {
  testName: string;
  result: 'pass' | 'fail';
  value?: number | string;
  specification?: string;
  notes?: string;
}

/**
 * Batch record
 */
export interface BatchRecord {
  /** Batch number (e.g., WIA-IND006-20251227-0042) */
  batchNumber: string;
  /** Customer/User ID */
  customerID: string;
  /** Formulation ID */
  formulationID: string;
  /** Product type */
  productType: ProductType;
  /** Production date */
  productionDate: string;
  /** Expiration date */
  expirationDate: string;
  /** Shelf life (months) */
  shelfLife: number;
  /** Batch size (mL) */
  batchSize: number;
  /** Ingredients with lot numbers */
  ingredients: Array<{
    inci: string;
    lotNumber: string;
    weight: number;
    supplier: string;
  }>;
  /** Production details */
  production: {
    operator: string;
    equipment: string;
    phaseATemp: number;
    phaseBTemp: number;
    emulsificationTime: number;
    finalTemp: number;
    finalPH: number;
  };
  /** Quality control results */
  qualityControl: {
    visualInspection: QCTestResult;
    microscopicEval: QCTestResult;
    phVerification: QCTestResult;
    microbialTest: QCTestResult;
    tests: QCTestResult[];
    releaseDate: string;
    approvedBy: string;
  };
  /** Distribution info */
  distribution?: {
    shippedDate: string;
    carrier: string;
    trackingNumber: string;
  };
  /** Batch status */
  status: BatchStatus;
}

// ============================================================================
// Efficacy Tracking Types
// ============================================================================

/**
 * Efficacy measurement interval
 */
export type EfficacyInterval = 'baseline' | 'week-2' | 'week-4' | 'week-8' | 'week-12' | 'week-24';

/**
 * Efficacy assessment
 */
export interface EfficacyAssessment {
  /** Assessment ID */
  assessmentID: string;
  /** User ID */
  userID: string;
  /** Formulation being assessed */
  formulationID: string;
  /** Batch number */
  batchNumber: string;
  /** Assessment interval */
  interval: EfficacyInterval;
  /** Assessment date */
  date: string;
  /** Instrumental measurements */
  measurements: InstrumentalMeasurements;
  /** AI visual analysis */
  aiAnalysis: AIVisualAnalysis;
  /** Subjective user feedback */
  userFeedback: {
    /** Overall satisfaction (1-10) */
    satisfaction: number;
    /** Perceived improvement (1-10) */
    improvement: number;
    /** Tolerability (1-10, higher = better tolerated) */
    tolerability: number;
    /** Texture satisfaction (1-10) */
    textureSatisfaction: number;
    /** Would recommend (yes/no) */
    wouldRecommend: boolean;
    /** Free-text comments */
    comments?: string;
  };
  /** Compliance data */
  compliance: {
    /** Expected applications since baseline */
    expectedApplications: number;
    /** Actual applications */
    actualApplications: number;
    /** Compliance percentage */
    complianceRate: number;
  };
  /** Adverse events (if any) */
  adverseEvents?: Array<{
    date: string;
    severity: 'mild' | 'moderate' | 'severe';
    description: string;
  }>;
}

/**
 * Efficacy comparison (baseline vs. current)
 */
export interface EfficacyComparison {
  /** Baseline assessment */
  baseline: EfficacyAssessment;
  /** Current assessment */
  current: EfficacyAssessment;
  /** Improvements */
  improvements: {
    /** Hydration improvement (%) */
    hydration: number;
    /** Elasticity improvement (%) */
    elasticity: number;
    /** Pigmentation reduction (%) */
    pigmentation: number;
    /** Wrinkle reduction (%) */
    wrinkles: number;
    /** Texture improvement (%) */
    texture: number;
    /** Overall skin quality improvement (%) */
    overallQuality: number;
  };
  /** Composite efficacy score (0-100) */
  compositeEfficacyScore: number;
  /** Statistical significance (if applicable) */
  pValue?: number;
}

// ============================================================================
// Environmental Adaptation Types
// ============================================================================

/**
 * Pollution severity
 */
export type PollutionSeverity = 'low' | 'moderate' | 'high' | 'very-high' | 'hazardous';

/**
 * Environmental conditions
 */
export interface EnvironmentalConditions {
  /** Climate zone */
  climate: ClimateZone;
  /** Current season (for temperate climates) */
  season?: 'spring' | 'summer' | 'fall' | 'winter';
  /** Temperature (°C) */
  temperature: number;
  /** Humidity (%) */
  humidity: number;
  /** UV index (0-11+) */
  uvIndex: number;
  /** Pollution level */
  pollution: {
    severity: PollutionSeverity;
    /** PM2.5 level (μg/m³) */
    pm25?: number;
    /** PM10 level (μg/m³) */
    pm10?: number;
    /** Air Quality Index */
    aqi?: number;
  };
}

/**
 * Environmental formulation modifiers
 */
export interface EnvironmentalModifiers {
  /** Hydration adjustment (-1.0 to +1.0) */
  hydration: number;
  /** Oil control adjustment */
  oilControl: number;
  /** Antioxidant boost */
  antioxidants: number;
  /** Occlusive boost */
  occlusives: number;
  /** Soothing agents boost */
  soothing: number;
  /** SPF recommendation */
  spfRecommendation: number;
  /** Barrier support boost */
  barrierSupport: number;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Skin analysis request
 */
export interface SkinAnalysisRequest {
  userID: string;
  /** Base64 encoded images */
  images?: string[];
  /** Manual measurements (if available) */
  measurements?: Partial<InstrumentalMeasurements>;
  /** Questionnaire responses */
  questionnaire: QuestionnaireData;
  /** Genetic data (optional) */
  geneticData?: GeneticProfile;
}

/**
 * Skin analysis response
 */
export interface SkinAnalysisResponse {
  skinProfile: SkinProfile;
  recommendations: string[];
  success: boolean;
  message?: string;
}

/**
 * Formulation generation request
 */
export interface FormulationGenerationRequest {
  skinProfileID: string;
  productType: ProductType;
  targetConcerns: SkinConcern[];
  preferences: FormulationPreferences;
  /** Environmental conditions (for adaptation) */
  environment?: EnvironmentalConditions;
}

/**
 * Formulation generation response
 */
export interface FormulationGenerationResponse {
  formulation: Formulation;
  alternatives?: Formulation[]; // Alternative formulations
  success: boolean;
  message?: string;
}

/**
 * Efficacy tracking request
 */
export interface EfficacyTrackingRequest {
  userID: string;
  formulationID: string;
  batchNumber: string;
  interval: EfficacyInterval;
  measurements: Partial<InstrumentalMeasurements>;
  images?: string[];
  userFeedback: EfficacyAssessment['userFeedback'];
  compliance: EfficacyAssessment['compliance'];
}

/**
 * Efficacy tracking response
 */
export interface EfficacyTrackingResponse {
  assessment: EfficacyAssessment;
  comparison?: EfficacyComparison; // If not baseline
  adjustmentRecommendations?: string[];
  success: boolean;
  message?: string;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Ingredient compatibility check
 */
export interface IngredientCompatibility {
  ingredientA: string;
  ingredientB: string;
  compatible: boolean;
  compatibilityScore: number; // 0-100
  warnings?: string[];
  recommendations?: string[];
}

/**
 * Allergen screening result
 */
export interface AllergenScreeningResult {
  formulation: Formulation;
  userAllergies: string[];
  detectedAllergens: string[];
  safe: boolean;
  warnings: string[];
}

/**
 * Genetic recommendation
 */
export interface GeneticRecommendation {
  gene: string;
  variant: GeneticVariant;
  impact: 'protective' | 'neutral' | 'risk';
  recommendations: string[];
  ingredientBoosts?: { [ingredient: string]: number }; // Percentage adjustment
}

/**
 * Skin factor modifiers (for formulation algorithm)
 */
export interface SkinFactorModifiers {
  /** Hydration factor (-1.0 to +1.0) */
  hydrationFactor: number;
  /** Oil balance factor */
  oilBalanceFactor: number;
  /** Sensitivity factor */
  sensitivityFactor: number;
  /** Aging factor (0 to +1.0) */
  agingFactor: number;
  /** Pigmentation factor */
  pigmentationFactor: number;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * API Error response
 */
export interface APIError {
  success: false;
  error: {
    code: string;
    message: string;
    details?: any;
  };
}

/**
 * Validation error
 */
export interface ValidationError {
  field: string;
  message: string;
  value?: any;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Regulatory concentration limits (EU)
 */
export const CONCENTRATION_LIMITS = {
  'Retinol': 0.3,
  'Salicylic Acid': 2.0,
  'Glycolic Acid': 15.0,
  'Benzoyl Peroxide': 5.0,
  'Hydroquinone': 0, // Banned in EU
  'Phenoxyethanol': 1.0,
} as const;

/**
 * Target pH ranges by product type
 */
export const TARGET_PH_RANGES: { [key in ProductType]?: { min: number; max: number } } = {
  'cleanser': { min: 5.0, max: 6.5 },
  'toner': { min: 4.5, max: 5.5 },
  'serum': { min: 4.0, max: 6.0 },
  'essence': { min: 5.0, max: 6.0 },
  'moisturizer': { min: 5.0, max: 6.5 },
  'eye-cream': { min: 6.0, max: 7.0 },
  'sunscreen': { min: 6.0, max: 7.5 },
};

/**
 * Standard measurement units
 */
export const MEASUREMENT_UNITS = {
  hydration: 'AU', // Arbitrary Units
  elasticity: 'score',
  sebum: 'μg/cm²',
  melanin: 'MI', // Melanin Index
  erythema: 'EI', // Erythema Index
  tewl: 'g/m²/h',
  pH: 'pH',
} as const;

// ============================================================================
// Export all types
// ============================================================================

export type {
  // Core classifications
  BaumannType,
  FitzpatrickType,
  SkinClassification,

  // Measurements
  InstrumentalMeasurements,
  AIVisualAnalysis,

  // Profile
  SkinProfile,
  QuestionnaireData,
  GeneticProfile,

  // Formulation
  Ingredient,
  Formulation,
  ProductType,

  // Manufacturing
  BatchRecord,
  BatchStatus,

  // Efficacy
  EfficacyAssessment,
  EfficacyComparison,

  // API
  SkinAnalysisRequest,
  SkinAnalysisResponse,
  FormulationGenerationRequest,
  FormulationGenerationResponse,
  EfficacyTrackingRequest,
  EfficacyTrackingResponse,
};
