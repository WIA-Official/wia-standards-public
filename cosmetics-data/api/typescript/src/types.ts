/**
 * WIA-IND-005: Cosmetics Data Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Product Types
// ============================================================================

/**
 * Product type classification
 */
export type ProductType =
  | 'cleanser'
  | 'toner'
  | 'serum'
  | 'essence'
  | 'moisturizer'
  | 'cream'
  | 'lotion'
  | 'sunscreen'
  | 'mask'
  | 'eye_cream'
  | 'shampoo'
  | 'conditioner'
  | 'hair_treatment'
  | 'hair_styling'
  | 'foundation'
  | 'concealer'
  | 'powder'
  | 'lipstick'
  | 'lip_gloss'
  | 'mascara'
  | 'eyeliner'
  | 'eyeshadow'
  | 'nail_polish'
  | 'perfume'
  | 'body_wash'
  | 'body_lotion'
  | 'deodorant'
  | 'other';

/**
 * Product category classification
 */
export type ProductCategory =
  | 'skincare'
  | 'haircare'
  | 'makeup'
  | 'bodycare'
  | 'fragrance'
  | 'suncare'
  | 'babycare';

/**
 * Application area
 */
export type ApplicationArea =
  | 'face'
  | 'eyes'
  | 'lips'
  | 'body'
  | 'hands'
  | 'feet'
  | 'hair'
  | 'scalp'
  | 'nails';

/**
 * Product format
 */
export type ProductFormat =
  | 'cream'
  | 'gel'
  | 'lotion'
  | 'serum'
  | 'oil'
  | 'foam'
  | 'powder'
  | 'stick'
  | 'spray'
  | 'aerosol'
  | 'wipes'
  | 'sheet_mask'
  | 'solid'
  | 'liquid';

/**
 * Skin type classification
 */
export type SkinType =
  | 'normal'
  | 'dry'
  | 'oily'
  | 'combination'
  | 'sensitive'
  | 'mature'
  | 'acne_prone'
  | 'all_types';

/**
 * Hair type classification
 */
export type HairType =
  | 'straight'
  | 'wavy'
  | 'curly'
  | 'coily'
  | 'fine'
  | 'thick'
  | 'damaged'
  | 'colored'
  | 'all_types';

// ============================================================================
// Ingredient Types
// ============================================================================

/**
 * INCI ingredient identification
 */
export interface IngredientIdentification {
  /** INCI (International Nomenclature of Cosmetic Ingredients) name */
  inci_name: string;

  /** CAS (Chemical Abstracts Service) registry number */
  cas_number?: string;

  /** EC (European Commission) number */
  ec_number?: string;

  /** EINECS number */
  einecs_number?: string;

  /** IUPAC chemical name */
  iupac_name?: string;

  /** Common names and trade names */
  common_names?: string[];

  /** Molecular formula */
  molecular_formula?: string;

  /** Molecular weight in g/mol */
  molecular_weight?: number;

  /** Chemical structure (SMILES or InChI notation) */
  chemical_structure?: string;
}

/**
 * Ingredient function categories
 */
export type IngredientFunction =
  | 'Active'
  | 'Emollient'
  | 'Humectant'
  | 'Preservative'
  | 'Emulsifier'
  | 'Surfactant'
  | 'Thickener'
  | 'Colorant'
  | 'Fragrance'
  | 'pH_Adjuster'
  | 'Antioxidant'
  | 'Chelating_Agent'
  | 'Solvent'
  | 'Film_Former'
  | 'Opacifier'
  | 'UV_Filter'
  | 'Exfoliant'
  | 'Absorbent'
  | 'Conditioning_Agent'
  | 'Viscosity_Controller';

/**
 * Solubility levels
 */
export type SolubilityLevel =
  | 'insoluble'
  | 'slightly_soluble'
  | 'moderately_soluble'
  | 'soluble'
  | 'very_soluble'
  | 'miscible';

/**
 * Ingredient functional properties
 */
export interface IngredientProperties {
  /** Primary function */
  primary_function: IngredientFunction;

  /** Secondary functions */
  secondary_functions?: IngredientFunction[];

  /** Mechanism of action */
  mechanism_of_action?: string;

  /** Efficacy data references */
  efficacy_data?: EfficacyData[];

  /** Typical usage range */
  typical_usage_range: {
    min_percent: number;
    max_percent: number;
    optimal_percent: number;
  };

  /** Solubility characteristics */
  solubility: {
    water: SolubilityLevel;
    oil: SolubilityLevel;
    alcohol: SolubilityLevel;
  };

  /** Stability parameters */
  stability: {
    ph_range: { min: number; max: number };
    temperature_range: { min: number; max: number };
    light_sensitivity: 'none' | 'low' | 'moderate' | 'high';
    oxidation_sensitivity: 'none' | 'low' | 'moderate' | 'high';
  };

  /** Physical state */
  physical_state: 'solid' | 'liquid' | 'gas' | 'semi-solid';

  /** Appearance */
  appearance: {
    color: string;
    odor: string;
  };
}

/**
 * Battery chemistry for cosmetic ingredient sourcing
 */
export type IngredientOrigin =
  | 'natural'
  | 'naturally_derived'
  | 'synthetic'
  | 'biotechnology'
  | 'mineral';

/**
 * Botanical ingredient details
 */
export interface BotanicalIngredient {
  /** Scientific name */
  scientific_name: {
    genus: string;
    species: string;
    variety?: string;
    author?: string;
  };

  /** Common names by locale */
  common_names: Record<string, string>;

  /** Plant part used */
  plant_part:
    | 'leaf'
    | 'root'
    | 'flower'
    | 'seed'
    | 'bark'
    | 'fruit'
    | 'stem'
    | 'whole_plant'
    | 'resin';

  /** Extraction method */
  extraction_method: ExtractionMethod;

  /** Solvent used in extraction */
  solvent_used?: string;

  /** Standardization marker */
  standardization?: {
    marker_compound: string;
    concentration: number;
    unit: string;
  };

  /** Geographical origin */
  geographical_origin?: string[];

  /** Harvest season */
  harvest_season?: string;

  /** Cultivation method */
  cultivation_method?: 'wild_crafted' | 'organic' | 'conventional';

  /** Active constituents */
  active_constituents?: Array<{
    compound_name: string;
    chemical_class: string;
    concentration_range: { min: number; max: number; unit: string };
    bioactivity: string[];
  }>;
}

/**
 * Extraction methods for botanical ingredients
 */
export type ExtractionMethod =
  | 'aqueous_extract'
  | 'hydroalcoholic_extract'
  | 'glycolic_extract'
  | 'oil_infusion'
  | 'co2_extract'
  | 'steam_distillation'
  | 'cold_press'
  | 'maceration'
  | 'percolation'
  | 'enfleurage';

/**
 * Complete ingredient data structure
 */
export interface Ingredient {
  /** Identification */
  identification: IngredientIdentification;

  /** Functional properties */
  properties: IngredientProperties;

  /** Origin type */
  origin: IngredientOrigin;

  /** Botanical details (if applicable) */
  botanical?: BotanicalIngredient;

  /** Safety profile */
  safety: SafetyProfile;

  /** Regulatory status */
  regulatory: RegulatoryStatus;

  /** Quality grade */
  grade: 'cosmetic' | 'pharmaceutical' | 'food' | 'technical';

  /** Sustainability metrics */
  sustainability?: SustainabilityMetrics;
}

// ============================================================================
// Safety and Toxicology Types
// ============================================================================

/**
 * Safety assessment scores
 */
export interface SafetyProfile {
  /** Overall safety score (0-100, 100 = safest) */
  safety_score: number;

  /** Toxicity data */
  toxicity: {
    ld50_oral?: { value: number; unit: 'mg/kg'; species: string };
    ld50_dermal?: { value: number; unit: 'mg/kg'; species: string };
    lc50_inhalation?: { value: number; unit: 'mg/L'; species: string };
  };

  /** Irritation potential */
  irritation: {
    skin: 'none' | 'negligible' | 'slight' | 'moderate' | 'severe';
    eye: 'none' | 'negligible' | 'slight' | 'moderate' | 'severe';
    respiratory?: 'none' | 'negligible' | 'slight' | 'moderate' | 'severe';
  };

  /** Sensitization potential */
  sensitization: {
    skin: 'very_low' | 'low' | 'moderate' | 'high' | 'very_high';
    llna_ec3?: number; // Local Lymph Node Assay EC3 value
  };

  /** Allergenicity classification */
  allergenicity: 'none' | 'low' | 'moderate' | 'high' | 'known_allergen';

  /** Known allergens */
  allergen_type?: string[];

  /** Phototoxicity */
  phototoxicity: 'none' | 'low' | 'moderate' | 'high';

  /** Photoallergenicity */
  photoallergenicity: 'none' | 'low' | 'moderate' | 'high';

  /** Comedogenicity rating (0-5) */
  comedogenicity: 0 | 1 | 2 | 3 | 4 | 5;

  /** Genotoxicity */
  genotoxicity: {
    ames_test: 'positive' | 'negative' | 'not_tested';
    micronucleus: 'positive' | 'negative' | 'not_tested';
  };

  /** Carcinogenicity classification */
  carcinogenicity?: 'group_1' | 'group_2A' | 'group_2B' | 'group_3' | 'group_4';

  /** Reproductive toxicity */
  reproductive_toxicity: 'none' | 'category_1A' | 'category_1B' | 'category_2';

  /** Endocrine disruption potential */
  endocrine_disruption: 'none' | 'suspected' | 'confirmed';

  /** Margin of Safety (MoS) */
  margin_of_safety?: {
    noael: number; // mg/kg bw/day
    sed: number; // Systemic Exposure Dose mg/kg bw/day
    mos: number; // NOAEL / SED
  };
}

/**
 * Adverse event data
 */
export interface AdverseEvent {
  /** Event ID */
  event_id: string;

  /** Date reported */
  report_date: string;

  /** Product involved */
  product_id: string;

  /** Batch number */
  batch_number?: string;

  /** Event description */
  description: string;

  /** Severity level */
  severity: 'mild' | 'moderate' | 'severe';

  /** Outcome */
  outcome:
    | 'recovered'
    | 'recovering'
    | 'not_recovered'
    | 'recovered_with_sequelae'
    | 'fatal'
    | 'unknown';

  /** Causality assessment */
  causality: 'certain' | 'probable' | 'possible' | 'unlikely' | 'unrelated';

  /** Patient demographics */
  patient?: {
    age?: number;
    gender?: 'male' | 'female' | 'other';
    medical_history?: string[];
    concurrent_medications?: string[];
  };

  /** Reported by */
  reporter: {
    type: 'consumer' | 'healthcare_professional' | 'company';
    contact?: string;
  };

  /** Actions taken */
  actions: string[];

  /** Follow-up required */
  follow_up_required: boolean;
}

// ============================================================================
// Regulatory Compliance Types
// ============================================================================

/**
 * Market regulatory status
 */
export type MarketRegion = 'US' | 'EU' | 'KR' | 'JP' | 'CN' | 'CA' | 'AU' | 'BR' | 'IN';

/**
 * Regulatory approval status
 */
export interface RegulatoryStatus {
  /** US FDA status */
  us_fda?: {
    status: 'approved' | 'restricted' | 'prohibited' | 'otc_drug';
    restrictions?: string;
    max_concentration?: number;
    required_warnings?: string[];
  };

  /** EU status */
  eu?: {
    status: 'approved' | 'restricted' | 'prohibited';
    annex?: 'II' | 'III' | 'IV' | 'V' | 'VI'; // Prohibited, Restricted, Colorants, Preservatives, UV Filters
    restrictions?: string;
    max_concentration?: number;
    required_warnings?: string[];
  };

  /** South Korea status */
  korea?: {
    status: 'approved' | 'restricted' | 'prohibited' | 'functional_cosmetic';
    restrictions?: string;
    max_concentration?: number;
  };

  /** Japan status */
  japan?: {
    status: 'approved' | 'restricted' | 'prohibited' | 'quasi_drug';
    jsci_listed: boolean; // Japanese Standards of Cosmetic Ingredients
    restrictions?: string;
  };

  /** China status */
  china?: {
    status: 'approved' | 'restricted' | 'prohibited';
    iecic_listed: boolean; // Inventory of Existing Cosmetic Ingredients in China
    animal_testing_required?: boolean;
    restrictions?: string;
  };

  /** Other markets */
  other_markets?: Record<
    MarketRegion,
    {
      status: 'approved' | 'restricted' | 'prohibited';
      restrictions?: string;
    }
  >;
}

/**
 * Product regulatory compliance
 */
export interface ProductCompliance {
  /** Product classification */
  classification: {
    us: 'cosmetic' | 'drug' | 'cosmetic_drug';
    eu: 'cosmetic' | 'quasi_drug';
    korea: 'general_cosmetic' | 'functional_cosmetic' | 'quasi_drug';
    japan: 'cosmetic' | 'quasi_drug' | 'pharmaceutical';
    china: 'general_cosmetic' | 'special_cosmetic';
  };

  /** Required registrations/notifications */
  registrations: {
    us_fda_facility?: string; // Facility registration number
    eu_cpnp?: string; // CPNP notification number
    korea_mfds?: string; // MFDS registration/notification number
    japan_pmda?: string; // PMDA notification number
    china_nmpa?: string; // NMPA registration number
  };

  /** Safety reports */
  safety_reports: {
    eu_cpsr?: { // Cosmetic Product Safety Report
      report_id: string;
      assessor: string;
      assessment_date: string;
      validity: string;
    };
    korea_safety_dossier?: {
      dossier_id: string;
      approval_date?: string;
    };
  };

  /** Compliant markets */
  compliant_markets: MarketRegion[];

  /** Non-compliant markets with reasons */
  non_compliant_markets?: Array<{
    market: MarketRegion;
    reason: string;
    prohibited_ingredients?: string[];
  }>;

  /** Labeling requirements met */
  labeling_compliant: boolean;

  /** GMP certification */
  gmp_certified: boolean;
}

// ============================================================================
// Formulation Types
// ============================================================================

/**
 * Formulation phase
 */
export type FormulationPhase = 'A' | 'B' | 'C' | 'D';

/**
 * Ingredient in formulation
 */
export interface FormulationIngredient {
  /** INCI name */
  inci_name: string;

  /** Percentage by weight */
  percentage: number;

  /** Primary function in formulation */
  function: IngredientFunction;

  /** Formulation phase */
  phase: FormulationPhase;

  /** Supplier information */
  supplier?: {
    name: string;
    grade: string;
    lot_number?: string;
  };

  /** Cost per kg */
  cost_per_kg?: number;
}

/**
 * Product formulation
 */
export interface Formulation {
  /** Formulation metadata */
  metadata: {
    product_id: string;
    product_name: string;
    product_type: ProductType;
    formula_version: string;
    created_date: string;
    created_by: string;
    last_modified?: string;
    status: 'draft' | 'validated' | 'approved' | 'archived';
  };

  /** Target product characteristics */
  target_properties: {
    ph_target: number;
    ph_range: { min: number; max: number };
    viscosity_target?: number;
    viscosity_range?: { min: number; max: number };
    color?: string;
    odor?: string;
    texture?: string;
  };

  /** Ingredient list */
  ingredients: FormulationIngredient[];

  /** Total percentage (should be 100) */
  total_percentage: number;

  /** Manufacturing instructions */
  manufacturing: {
    phases: Array<{
      phase: FormulationPhase;
      temperature?: number;
      mixing_time?: number;
      mixing_speed?: number;
      instructions: string;
    }>;
    equipment_required?: string[];
    critical_parameters?: Record<string, any>;
  };

  /** Stability data */
  stability?: {
    tested: boolean;
    accelerated_passed?: boolean;
    long_term_passed?: boolean;
    shelf_life_months?: number;
  };

  /** Safety assessment */
  safety_assessment?: SafetyAssessment;

  /** Regulatory compliance */
  regulatory_compliance?: ProductCompliance;

  /** Cost analysis */
  cost_analysis?: {
    raw_material_cost: number;
    packaging_cost?: number;
    manufacturing_cost?: number;
    total_cost?: number;
    currency: string;
  };
}

/**
 * Formulation validation results
 */
export interface FormulationValidation {
  /** Valid formulation */
  is_valid: boolean;

  /** Validation errors */
  errors: Array<{
    severity: 'error' | 'warning' | 'info';
    code: string;
    message: string;
    ingredient?: string;
  }>;

  /** Total percentage check */
  percentage_check: {
    total: number;
    deviation: number;
    acceptable: boolean;
  };

  /** pH compatibility */
  ph_compatibility: {
    compatible: boolean;
    conflicts?: string[];
  };

  /** Ingredient compatibility */
  ingredient_compatibility: {
    compatible: boolean;
    incompatibilities?: Array<{
      ingredient1: string;
      ingredient2: string;
      issue: string;
    }>;
  };

  /** Stability prediction */
  stability_prediction: {
    rating: 'excellent' | 'good' | 'fair' | 'poor';
    risk_factors?: string[];
  };
}

// ============================================================================
// Efficacy and Claims Types
// ============================================================================

/**
 * Clinical study design
 */
export type StudyDesign =
  | 'open_label'
  | 'single_blind'
  | 'double_blind'
  | 'split_face'
  | 'randomized_controlled';

/**
 * Efficacy data from studies
 */
export interface EfficacyData {
  /** Study metadata */
  study_id: string;
  study_type: 'in_vitro' | 'ex_vivo' | 'in_vivo' | 'consumer_perception';
  study_design?: StudyDesign;

  /** Sample information */
  sample_size?: number;
  duration?: { value: number; unit: 'days' | 'weeks' | 'months' };

  /** Study endpoints */
  endpoints: Array<{
    parameter: string;
    measurement_method: string;
    baseline_value?: number;
    endpoint_value?: number;
    change_from_baseline?: {
      absolute: number;
      percent: number;
    };
    p_value?: number;
    statistically_significant: boolean;
  }>;

  /** Instrumental measurements */
  instrumental_data?: {
    hydration?: { value: number; unit: 'AU'; device: string };
    tewl?: { value: number; unit: 'g/h/m²'; device: string };
    elasticity?: { r2: number; r5: number; r7: number; device: string };
    roughness?: { ra: number; rz: number; device: string };
    melanin_index?: { value: number; device: string };
    erythema_index?: { value: number; device: string };
  };

  /** Consumer perception */
  consumer_perception?: Array<{
    question: string;
    response_type: 'VAS' | 'Likert' | 'binary';
    positive_response_rate?: number;
  }>;

  /** Study reference */
  reference?: string;
  publication_date?: string;
}

/**
 * Product claim substantiation
 */
export interface ClaimSubstantiation {
  /** Claim text */
  claim: string;

  /** Claim category */
  category:
    | 'moisturizing'
    | 'anti_aging'
    | 'whitening'
    | 'firming'
    | 'smoothing'
    | 'soothing'
    | 'acne'
    | 'other';

  /** Evidence level */
  evidence_level: 'A' | 'B' | 'C' | 'D';

  /** Supporting studies */
  supporting_studies: EfficacyData[];

  /** Claim approval status */
  approved_for_markets: MarketRegion[];

  /** Restrictions */
  restrictions?: string[];
}

// ============================================================================
// Quality Control Types
// ============================================================================

/**
 * Quality specification
 */
export interface QualitySpecification {
  /** Parameter name */
  parameter: string;

  /** Test method */
  method: string;

  /** Acceptance criteria */
  specification: string;

  /** Is critical parameter */
  critical: boolean;
}

/**
 * Batch record
 */
export interface BatchRecord {
  /** Batch identification */
  batch_number: string;
  product_id: string;
  product_name: string;

  /** Manufacturing dates */
  manufacturing_date: string;
  expiry_date: string;

  /** Batch size */
  batch_size: { value: number; unit: string };

  /** Formula version */
  formula_version: string;

  /** Raw materials used */
  raw_materials: Array<{
    ingredient_name: string;
    supplier: string;
    lot_number: string;
    quantity_used: { value: number; unit: string };
    coa_verified: boolean;
  }>;

  /** Manufacturing steps */
  manufacturing_steps: Array<{
    step_number: number;
    description: string;
    operator_id: string;
    start_time: string;
    end_time: string;
    parameters?: Record<string, any>;
    verification: boolean;
  }>;

  /** Quality control results */
  qc_results: Array<{
    test_name: string;
    method: string;
    specification: string;
    result: string;
    pass: boolean;
    tested_by: string;
    test_date: string;
  }>;

  /** Release status */
  release_status: 'released' | 'rejected' | 'on_hold' | 'quarantine';
  release_date?: string;
  released_by?: string;

  /** Deviations */
  deviations?: Array<{
    deviation_number: string;
    description: string;
    impact: string;
    capa_status: 'open' | 'closed';
  }>;
}

/**
 * Microbiological test results
 */
export interface MicrobiologicalTest {
  /** Test date */
  test_date: string;

  /** Batch tested */
  batch_number: string;

  /** Total aerobic count */
  tamc: {
    result: number;
    unit: 'CFU/g' | 'CFU/mL';
    limit: number;
    pass: boolean;
  };

  /** Total yeast and mold count */
  tymc: {
    result: number;
    unit: 'CFU/g' | 'CFU/mL';
    limit: number;
    pass: boolean;
  };

  /** Pathogen screening */
  pathogens: {
    s_aureus: 'absent' | 'present';
    p_aeruginosa: 'absent' | 'present';
    e_coli: 'absent' | 'present';
    c_albicans: 'absent' | 'present';
  };

  /** All pathogens absent */
  pathogens_pass: boolean;

  /** Overall pass status */
  overall_pass: boolean;
}

/**
 * Stability test data
 */
export interface StabilityTest {
  /** Study ID */
  study_id: string;

  /** Product tested */
  product_id: string;
  batch_number: string;

  /** Storage condition */
  condition: {
    temperature: number;
    humidity?: number;
    light_exposure?: boolean;
  };

  /** Timepoints tested */
  timepoints: Array<{
    time: { value: number; unit: 'days' | 'weeks' | 'months' };
    tests: Array<{
      parameter: string;
      result: string;
      specification: string;
      pass: boolean;
    }>;
    overall_pass: boolean;
  }>;

  /** Study conclusion */
  conclusion: {
    stable: boolean;
    shelf_life_estimate?: { value: number; unit: 'months' };
    notes?: string;
  };
}

// ============================================================================
// Supply Chain Types
// ============================================================================

/**
 * Ingredient source tracking
 */
export interface IngredientSource {
  /** Supplier information */
  supplier: {
    name: string;
    country: string;
    certifications: string[];
    audit_score?: number;
    audit_date?: string;
  };

  /** Origin details */
  origin: {
    country_of_origin: string;
    region?: string;
    farm_or_facility?: string;
    gps_coordinates?: { latitude: number; longitude: number };
  };

  /** Harvest/production information */
  production: {
    date: string;
    batch_number: string;
    method: 'wild_crafted' | 'organic' | 'conventional' | 'synthetic';
    certifications?: string[];
  };

  /** Transportation */
  transportation: {
    route: string[];
    method: 'air' | 'sea' | 'land';
    temperature_controlled: boolean;
    arrival_date: string;
  };

  /** Quality verification */
  quality: {
    coa_number: string;
    testing_lab: string;
    release_date: string;
  };
}

/**
 * Sustainability metrics
 */
export interface SustainabilityMetrics {
  /** Carbon footprint */
  carbon_footprint?: {
    value: number;
    unit: 'kg CO2e';
    scope: 'ingredient' | 'product' | 'full_lifecycle';
  };

  /** Water usage */
  water_usage?: {
    value: number;
    unit: 'L';
  };

  /** Biodegradability */
  biodegradability: {
    readily_biodegradable: boolean;
    oecd_301_test?: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';
    degradation_percent?: number;
  };

  /** Bioaccumulation potential */
  bioaccumulation?: {
    log_kow: number;
    bcf?: number; // Bioconcentration Factor
    potential: 'low' | 'moderate' | 'high';
  };

  /** Certifications */
  certifications: Array<{
    type:
      | 'organic'
      | 'fair_trade'
      | 'rainforest_alliance'
      | 'cosmos'
      | 'ecocert'
      | 'usda_organic'
      | 'cruelty_free'
      | 'vegan'
      | 'other';
    certifying_body: string;
    certificate_number?: string;
    valid_until?: string;
  }>;

  /** Renewable content */
  renewable_content_percent?: number;

  /** Packaging recyclability */
  packaging_recyclable?: boolean;
  packaging_recycled_content_percent?: number;
}

// ============================================================================
// Safety Assessment Types
// ============================================================================

/**
 * Comprehensive safety assessment
 */
export interface SafetyAssessment {
  /** Assessment ID */
  assessment_id: string;

  /** Product assessed */
  product_id: string;
  formula_version: string;

  /** Assessment date */
  assessment_date: string;

  /** Assessor information */
  assessor: {
    name: string;
    qualifications: string[];
    signature?: string;
  };

  /** Overall safety conclusion */
  conclusion: {
    safe_for_use: boolean;
    target_population: string[];
    restrictions?: string[];
    warnings_required?: string[];
  };

  /** Individual ingredient assessments */
  ingredient_assessments: Array<{
    inci_name: string;
    concentration_percent: number;
    safety_profile: SafetyProfile;
    acceptable: boolean;
    notes?: string;
  }>;

  /** Exposure assessment */
  exposure: {
    application_area: ApplicationArea[];
    frequency_of_use: 'daily' | 'weekly' | 'occasional';
    amount_per_application: { value: number; unit: 'g' | 'mL' };
    retention_factor: number; // 0-1, for rinse-off vs leave-on
    dermal_absorption_percent: number;
    systemic_exposure_dose: number; // mg/kg bw/day
  };

  /** Risk assessment */
  risk_assessment: {
    margin_of_safety: number;
    acceptable_mos: number;
    risk_acceptable: boolean;
  };

  /** Special populations */
  special_populations?: {
    children: { safe: boolean; age_restriction?: string };
    pregnant_women: { safe: boolean; warnings?: string[] };
    elderly: { safe: boolean; warnings?: string[] };
  };
}

// ============================================================================
// Analytics and Reporting Types
// ============================================================================

/**
 * Product performance analytics
 */
export interface ProductAnalytics {
  /** Product ID */
  product_id: string;

  /** Sales data */
  sales?: {
    units_sold: number;
    revenue: number;
    currency: string;
    period: { start: string; end: string };
  };

  /** Consumer reviews */
  reviews?: {
    average_rating: number;
    total_reviews: number;
    rating_distribution: { 1: number; 2: number; 3: number; 4: number; 5: number };
  };

  /** Return/complaint rate */
  quality_metrics?: {
    return_rate_percent: number;
    complaint_rate_percent: number;
    adverse_event_count: number;
  };

  /** Efficacy metrics */
  efficacy_metrics?: {
    claimed_benefit: string;
    consumer_satisfaction_percent: number;
    repeat_purchase_rate_percent: number;
  };
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Measurement with unit
 */
export interface Measurement {
  value: number;
  unit: string;
}

/**
 * Date range
 */
export interface DateRange {
  start: string;
  end: string;
}

/**
 * Pagination
 */
export interface Pagination {
  page: number;
  page_size: number;
  total_items: number;
  total_pages: number;
}

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: string;
    version: string;
    pagination?: Pagination;
  };
}

// ============================================================================
// Search and Filter Types
// ============================================================================

/**
 * Ingredient search filters
 */
export interface IngredientSearchFilters {
  /** Text search */
  query?: string;

  /** Function filter */
  functions?: IngredientFunction[];

  /** Origin filter */
  origins?: IngredientOrigin[];

  /** Safety score range */
  safety_score_min?: number;
  safety_score_max?: number;

  /** Regulatory approval */
  approved_in_markets?: MarketRegion[];

  /** Allergen-free */
  allergen_free?: boolean;

  /** Comedogenicity max */
  max_comedogenicity?: 0 | 1 | 2 | 3 | 4 | 5;

  /** Certifications */
  certifications?: string[];

  /** Price range */
  price_min?: number;
  price_max?: number;
}

/**
 * Product search filters
 */
export interface ProductSearchFilters {
  /** Category */
  category?: ProductCategory;

  /** Product type */
  product_type?: ProductType;

  /** Skin type */
  skin_type?: SkinType;

  /** Contains ingredients */
  contains_ingredients?: string[];

  /** Excludes ingredients */
  excludes_ingredients?: string[];

  /** Certifications */
  certifications?: string[];

  /** Price range */
  price_min?: number;
  price_max?: number;

  /** Safety score minimum */
  safety_score_min?: number;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './index'; // Re-export main SDK functions
