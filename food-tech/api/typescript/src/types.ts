/**
 * WIA-IND-007: Food Tech Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Food Technology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Food Types
// ============================================================================

/**
 * Food category classification
 */
export type FoodCategory =
  | 'meat_seafood'
  | 'dairy_eggs'
  | 'fruits_vegetables'
  | 'grains_cereals'
  | 'legumes_nuts'
  | 'beverages'
  | 'processed_foods'
  | 'alternative_protein'
  | 'supplements'
  | 'condiments_sauces';

/**
 * Measurement units
 */
export type MeasurementUnit = 'g' | 'mg' | 'μg' | 'kg' | 'L' | 'mL' | 'oz' | 'lb' | 'cup' | 'tbsp' | 'tsp';

/**
 * Temperature units
 */
export type TemperatureUnit = 'celsius' | 'fahrenheit' | 'kelvin';

// ============================================================================
// Alternative Protein Types
// ============================================================================

/**
 * Protein source types
 */
export type ProteinSourceType =
  // Plant-based
  | 'soy_protein'
  | 'pea_protein'
  | 'wheat_gluten'
  | 'rice_protein'
  | 'hemp_protein'
  | 'lentil_protein'
  | 'chickpea_protein'
  | 'quinoa_protein'
  | 'potato_protein'
  // Cultured meat
  | 'cultured_beef'
  | 'cultured_chicken'
  | 'cultured_pork'
  | 'cultured_fish'
  | 'cultured_seafood'
  // Fermentation
  | 'precision_fermentation_whey'
  | 'precision_fermentation_casein'
  | 'precision_fermentation_egg'
  | 'mycoprotein'
  | 'algae_spirulina'
  | 'algae_chlorella'
  // Insects
  | 'cricket_protein'
  | 'mealworm_protein'
  | 'black_soldier_fly'
  | 'silkworm';

/**
 * Battery chemistry for cultured meat (cell types)
 */
export type CellLineType =
  | 'satellite_cells'
  | 'embryonic_stem_cells'
  | 'induced_pluripotent_stem_cells'
  | 'mesenchymal_stem_cells'
  | 'fibroblasts';

/**
 * Protein extraction method
 */
export type ExtractionMethod =
  | 'wet_fractionation'
  | 'dry_fractionation'
  | 'enzyme_assisted'
  | 'membrane_filtration'
  | 'ultrafiltration'
  | 'isoelectric_precipitation';

/**
 * Texturization technology
 */
export type TexturizationMethod =
  | 'high_moisture_extrusion'
  | 'low_moisture_extrusion'
  | 'shear_cell'
  | '3d_printing'
  | 'spinning'
  | 'emulsion';

/**
 * Protein efficiency metrics
 */
export interface ProteinEfficiencyMetrics {
  /** Feed Conversion Ratio (Feed Input / Edible Output) */
  fcr: number;

  /** Protein Efficiency Ratio (Weight Gain / Protein Intake) */
  per: number;

  /** Protein Digestibility-Corrected Amino Acid Score (0-1.0) */
  pdcaas: number;

  /** Digestible Indispensable Amino Acid Score */
  diaas?: number;

  /** Energy efficiency (MJ input / kg protein output) */
  energy_efficiency: number;

  /** Production yield (kg output / kg input) */
  yield: number;
}

/**
 * Alternative protein product
 */
export interface AlternativeProteinProduct {
  /** Product identifier */
  id: string;

  /** Product name */
  name: string;

  /** Protein source type */
  protein_source: ProteinSourceType;

  /** Protein content (g per 100g) */
  protein_content: number;

  /** Complete amino acid profile */
  amino_acid_profile: AminoAcidProfile;

  /** Processing methods used */
  processing: {
    extraction?: ExtractionMethod;
    texturization?: TexturizationMethod;
    additional_processing: string[];
  };

  /** Efficiency metrics */
  efficiency: ProteinEfficiencyMetrics;

  /** Nutritional fortification */
  fortification?: FortificationProfile;
}

/**
 * Amino acid profile (g per 100g protein)
 */
export interface AminoAcidProfile {
  // Essential amino acids
  histidine: number;
  isoleucine: number;
  leucine: number;
  lysine: number;
  methionine: number;
  phenylalanine: number;
  threonine: number;
  tryptophan: number;
  valine: number;

  // Conditionally essential
  arginine?: number;
  cysteine?: number;
  glutamine?: number;
  glycine?: number;
  proline?: number;
  tyrosine?: number;
}

// ============================================================================
// Nutrition Types
// ============================================================================

/**
 * Vitamin types
 */
export type VitaminType =
  | 'A' | 'B1' | 'B2' | 'B3' | 'B5' | 'B6' | 'B7' | 'B9' | 'B12'
  | 'C' | 'D' | 'E' | 'K';

/**
 * Mineral types
 */
export type MineralType =
  | 'calcium' | 'phosphorus' | 'magnesium' | 'sodium' | 'potassium'
  | 'iron' | 'zinc' | 'copper' | 'selenium' | 'iodine' | 'manganese'
  | 'chromium' | 'molybdenum' | 'fluoride' | 'chloride';

/**
 * Macronutrient distribution
 */
export interface MacronutrientProfile {
  /** Total calories (kcal) */
  calories: number;

  /** Protein (grams) */
  protein: {
    grams: number;
    calories: number;
    percent_of_total: number;
  };

  /** Carbohydrates (grams) */
  carbohydrates: {
    total_grams: number;
    fiber_grams: number;
    sugars_grams: number;
    added_sugars_grams?: number;
    net_carbs_grams: number;  // total - fiber
    calories: number;
    percent_of_total: number;
  };

  /** Fat (grams) */
  fat: {
    total_grams: number;
    saturated_grams: number;
    trans_grams: number;
    monounsaturated_grams?: number;
    polyunsaturated_grams?: number;
    omega3_grams?: number;
    omega6_grams?: number;
    calories: number;
    percent_of_total: number;
  };

  /** Cholesterol (mg) */
  cholesterol?: number;

  /** Sodium (mg) */
  sodium: number;
}

/**
 * Micronutrient profile
 */
export interface MicronutrientProfile {
  /** Vitamins (amount and % RDI) */
  vitamins: Record<VitaminType, {
    amount: number;
    unit: 'mg' | 'μg' | 'IU';
    percent_rdi: number;
  }>;

  /** Minerals (amount and % RDI) */
  minerals: Record<MineralType, {
    amount: number;
    unit: 'mg' | 'μg';
    percent_rdi: number;
  }>;
}

/**
 * Nutritional Density Score calculation
 */
export interface NutritionalDensityScore {
  /** Overall NDS (higher = more nutrient-dense) */
  score: number;

  /** Individual nutrient contributions */
  breakdown: {
    vitamin_score: number;
    mineral_score: number;
    protein_score: number;
    fiber_score: number;
  };

  /** Classification */
  classification: 'superfood' | 'nutrient_dense' | 'moderate' | 'low' | 'empty_calories';

  /** Calories per serving */
  calories: number;
}

/**
 * Fortification profile
 */
export interface FortificationProfile {
  /** Added vitamins */
  vitamins?: Array<{
    type: VitaminType;
    amount: number;
    unit: 'mg' | 'μg';
    source: string;
  }>;

  /** Added minerals */
  minerals?: Array<{
    type: MineralType;
    amount: number;
    unit: 'mg' | 'μg';
    source: string;
  }>;

  /** Other functional ingredients */
  other?: Array<{
    name: string;
    amount: number;
    unit: string;
    purpose: string;
  }>;
}

/**
 * Personalized nutrition profile
 */
export interface PersonalNutritionProfile {
  /** User demographics */
  demographics: {
    age: number;
    gender: 'male' | 'female' | 'other';
    weight_kg: number;
    height_cm: number;
  };

  /** Activity level */
  activity_level: 'sedentary' | 'light' | 'moderate' | 'active' | 'very_active';

  /** Dietary goal */
  goal: 'loss' | 'maintenance' | 'gain' | 'recomposition';

  /** Dietary restrictions */
  restrictions?: Array<'vegan' | 'vegetarian' | 'gluten_free' | 'dairy_free' | 'low_carb' | 'keto'>;

  /** Allergies */
  allergies?: AllergenType[];

  /** Health conditions */
  health_conditions?: Array<'diabetes' | 'hypertension' | 'high_cholesterol' | 'kidney_disease' | 'celiac'>;
}

/**
 * Personalized macronutrient recommendations
 */
export interface PersonalizedMacros {
  /** Basal Metabolic Rate (kcal/day) */
  bmr: number;

  /** Total Daily Energy Expenditure (kcal/day) */
  tdee: number;

  /** Target calories for goal */
  target_calories: number;

  /** Recommended macronutrients */
  macros: MacronutrientProfile;

  /** Meal timing recommendations */
  meal_timing?: {
    meals_per_day: number;
    pre_workout?: MacronutrientProfile;
    post_workout?: MacronutrientProfile;
  };
}

// ============================================================================
// Food Safety Types
// ============================================================================

/**
 * Allergen types (FDA Big 9 + common)
 */
export type AllergenType =
  | 'milk'
  | 'eggs'
  | 'fish'
  | 'shellfish'
  | 'tree_nuts'
  | 'peanuts'
  | 'wheat'
  | 'soybeans'
  | 'sesame'
  | 'gluten'
  | 'sulfites'
  | 'mustard'
  | 'celery'
  | 'lupin';

/**
 * Pathogen types
 */
export type PathogenType =
  | 'salmonella'
  | 'listeria_monocytogenes'
  | 'e_coli_o157'
  | 'e_coli_generic'
  | 'campylobacter'
  | 'staphylococcus_aureus'
  | 'bacillus_cereus'
  | 'clostridium_perfringens'
  | 'clostridium_botulinum'
  | 'vibrio'
  | 'yersinia'
  | 'shigella';

/**
 * Microbial test result
 */
export interface MicrobialTest {
  /** Test type */
  test_type: 'pathogen' | 'indicator' | 'spoilage';

  /** Microorganism tested */
  organism: PathogenType | 'total_plate_count' | 'coliform' | 'yeast_mold';

  /** Result (CFU/g or CFU/mL) */
  result: number | 'not_detected';

  /** Detection limit (CFU) */
  detection_limit: number;

  /** Specification limit */
  specification_limit: number | 'zero_tolerance';

  /** Pass/Fail */
  status: 'pass' | 'fail' | 'presumptive';

  /** Test method */
  method: 'culture' | 'pcr' | 'qpcr' | 'elisa' | 'atp' | 'rapid';

  /** Test date */
  test_date: string;

  /** Lab name */
  lab: string;
}

/**
 * Chemical test result
 */
export interface ChemicalTest {
  /** Analyte tested */
  analyte: string;

  /** Category */
  category: 'pesticide' | 'heavy_metal' | 'mycotoxin' | 'veterinary_drug' | 'additive' | 'contaminant';

  /** Result (ppm, ppb, or mg/kg) */
  result: number;

  /** Unit */
  unit: 'ppm' | 'ppb' | 'mg/kg' | 'μg/kg';

  /** Regulatory limit */
  regulatory_limit: number;

  /** Pass/Fail */
  status: 'pass' | 'fail';

  /** Test method */
  method: 'icp_ms' | 'gc_ms' | 'lc_ms' | 'elisa' | 'hplc';

  /** Test date */
  test_date: string;
}

/**
 * Physical inspection result
 */
export interface PhysicalInspection {
  /** Foreign material detection */
  foreign_material: {
    detected: boolean;
    type?: 'metal' | 'glass' | 'plastic' | 'stone' | 'wood' | 'other';
    size_mm?: number;
  };

  /** Visual defects */
  defects: Array<{
    type: 'discoloration' | 'bruising' | 'mold' | 'insect' | 'damage';
    severity: 'minor' | 'moderate' | 'major' | 'critical';
    location?: string;
  }>;

  /** Overall appearance rating (0-10) */
  appearance_rating: number;

  /** Inspector */
  inspector: string;

  /** Inspection date */
  inspection_date: string;
}

/**
 * Food Safety Index
 */
export interface FoodSafetyIndex {
  /** Overall FSI score (0-100) */
  overall_score: number;

  /** Component scores */
  scores: {
    microbial_score: number;  // 0-100
    chemical_score: number;   // 0-100
    physical_score: number;   // 0-100
  };

  /** Classification */
  classification: 'safe' | 'acceptable' | 'marginal' | 'unsafe';

  /** Risk level */
  risk_level: 'low' | 'medium' | 'high' | 'critical';

  /** Recommendations */
  recommendations: string[];

  /** Test summary */
  test_summary: {
    microbial_tests: MicrobialTest[];
    chemical_tests: ChemicalTest[];
    physical_inspection: PhysicalInspection;
  };
}

/**
 * HACCP Critical Control Point
 */
export interface CriticalControlPoint {
  /** CCP identifier */
  ccp_id: string;

  /** Process step */
  step: string;

  /** Hazard being controlled */
  hazard: {
    type: 'biological' | 'chemical' | 'physical';
    description: string;
  };

  /** Critical limit */
  critical_limit: {
    parameter: 'temperature' | 'time' | 'ph' | 'aw' | 'pressure' | 'other';
    value: number;
    unit: string;
    operator: '>=' | '<=' | '=' | '>' | '<' | 'range';
  };

  /** Monitoring procedure */
  monitoring: {
    method: string;
    frequency: string;
    responsibility: string;
  };

  /** Corrective action */
  corrective_action: string;

  /** Verification */
  verification: {
    method: string;
    frequency: string;
  };

  /** Records */
  records: string[];
}

// ============================================================================
// Fermentation Types
// ============================================================================

/**
 * Microorganism types
 */
export type MicroorganismType =
  // Yeast
  | 'saccharomyces_cerevisiae'
  | 'saccharomyces_pastorianus'
  | 'pichia_pastoris'
  | 'kluyveromyces_lactis'
  | 'yarrowia_lipolytica'
  // Bacteria
  | 'escherichia_coli'
  | 'bacillus_subtilis'
  | 'corynebacterium_glutamicum'
  | 'lactobacillus_bulgaricus'
  | 'lactobacillus_plantarum'
  | 'streptococcus_thermophilus'
  // Fungi
  | 'aspergillus_oryzae'
  | 'aspergillus_niger'
  | 'trichoderma_reesei'
  | 'fusarium_venenatum'  // Mycoprotein
  // Algae
  | 'spirulina_platensis'
  | 'chlorella_vulgaris';

/**
 * Substrate type for fermentation
 */
export type SubstrateType =
  | 'glucose'
  | 'sucrose'
  | 'lactose'
  | 'maltose'
  | 'starch'
  | 'cellulose'
  | 'glycerol'
  | 'methanol'
  | 'ethanol'
  | 'acetate'
  | 'whey'
  | 'molasses'
  | 'corn_steep_liquor';

/**
 * Fermentation mode
 */
export type FermentationMode =
  | 'batch'
  | 'fed_batch'
  | 'continuous'
  | 'perfusion';

/**
 * Fermentation parameters
 */
export interface FermentationParameters {
  /** Fermentation mode */
  mode: FermentationMode;

  /** Microorganism */
  organism: MicroorganismType;

  /** Substrate */
  substrate: SubstrateType;

  /** Operating conditions */
  conditions: {
    temperature_celsius: number;
    ph: number;
    dissolved_oxygen_percent: number;
    agitation_rpm: number;
    pressure_bar: number;
  };

  /** Kinetic parameters */
  kinetics: {
    specific_growth_rate: number;  // h^-1
    doubling_time_hours: number;
    yield_coefficient: number;  // g biomass / g substrate
    productivity: number;  // g/L/h
  };

  /** Media composition */
  media: {
    substrate_concentration_g_per_L: number;
    nitrogen_source: string;
    vitamins: string[];
    minerals: string[];
    growth_factors?: string[];
  };

  /** Target metrics */
  targets: {
    biomass_concentration_g_per_L?: number;
    product_titer_g_per_L?: number;
    fermentation_time_hours: number;
    final_yield_percent: number;
  };
}

/**
 * Fermentation result
 */
export interface FermentationResult {
  /** Batch identifier */
  batch_id: string;

  /** Start and end times */
  start_time: string;
  end_time: string;

  /** Achieved metrics */
  achieved: {
    biomass_concentration_g_per_L: number;
    product_titer_g_per_L: number;
    yield_on_substrate_g_per_g: number;
    productivity_g_per_L_per_h: number;
  };

  /** Process data */
  process_data: {
    temperature_log: Array<{ timestamp: string; value: number }>;
    ph_log: Array<{ timestamp: string; value: number }>;
    do_log: Array<{ timestamp: string; value: number }>;
  };

  /** Quality assessment */
  quality: {
    contamination_check: 'pass' | 'fail';
    viability_percent: number;
    product_purity_percent: number;
  };

  /** Cost analysis */
  cost: {
    raw_materials_usd: number;
    utilities_usd: number;
    labor_usd: number;
    total_usd: number;
    cost_per_kg_usd: number;
  };
}

// ============================================================================
// Processing Types
// ============================================================================

/**
 * Processing methods
 */
export type ProcessingMethod =
  // Thermal
  | 'pasteurization'
  | 'sterilization'
  | 'uhp'  // Ultra-high temperature
  | 'blanching'
  | 'cooking'
  | 'roasting'
  | 'baking'
  | 'frying'
  // Non-thermal
  | 'high_pressure_processing'
  | 'pulsed_electric_field'
  | 'ultrasound'
  | 'uv_light'
  | 'irradiation'
  | 'ozonation'
  // Preservation
  | 'freezing'
  | 'refrigeration'
  | 'dehydration'
  | 'freeze_drying'
  | 'spray_drying'
  | 'canning'
  | 'vacuum_packaging'
  | 'modified_atmosphere_packaging'
  // Mechanical
  | 'homogenization'
  | 'emulsification'
  | 'grinding'
  | 'extrusion'
  | 'centrifugation'
  | 'filtration'
  // Chemical
  | 'acidification'
  | 'fermentation'
  | 'enzymatic_treatment'
  | 'smoking'
  | 'curing';

/**
 * Packaging type
 */
export type PackagingType =
  | 'glass_bottle'
  | 'plastic_bottle'
  | 'metal_can'
  | 'carton'
  | 'pouch'
  | 'vacuum_sealed'
  | 'modified_atmosphere'
  | 'aseptic'
  | 'bulk';

/**
 * Shelf life prediction
 */
export interface ShelfLifePrediction {
  /** Predicted shelf life (days) */
  shelf_life_days: number;

  /** Confidence interval */
  confidence_interval: {
    lower_days: number;
    upper_days: number;
    confidence_level: number;  // e.g., 0.95 for 95%
  };

  /** Limiting factor */
  limiting_factor: 'microbial' | 'chemical' | 'sensory' | 'nutritional';

  /** Storage conditions */
  storage: {
    temperature_celsius: number;
    humidity_percent: number;
    light_exposure: 'dark' | 'ambient' | 'direct';
    packaging: PackagingType;
  };

  /** Q10 temperature coefficient */
  q10: number;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Sustainability Types
// ============================================================================

/**
 * Lifecycle stages for LCA
 */
export type LifecycleStage =
  | 'agriculture'
  | 'processing'
  | 'packaging'
  | 'transportation'
  | 'retail'
  | 'consumer_use'
  | 'end_of_life';

/**
 * Carbon footprint result
 */
export interface CarbonFootprintResult {
  /** Total GHG emissions (kg CO2e per kg product) */
  total_kg_co2e: number;

  /** Breakdown by lifecycle stage */
  breakdown: Record<LifecycleStage, number>;

  /** Breakdown by emission source */
  by_source: {
    scope_1_direct: number;
    scope_2_indirect_energy: number;
    scope_3_supply_chain: number;
  };

  /** Comparison to conventional product */
  comparison?: {
    conventional_product: string;
    conventional_emissions_kg_co2e: number;
    reduction_percent: number;
  };

  /** Carbon offset recommendations */
  offset_recommendations?: string[];
}

/**
 * Water footprint result
 */
export interface WaterFootprintResult {
  /** Total water (liters per kg product) */
  total_liters: number;

  /** Breakdown by water type */
  breakdown: {
    blue_water_liters: number;    // Irrigation, processing
    green_water_liters: number;   // Rainwater
    grey_water_liters: number;    // Pollution dilution
  };

  /** Water stress indicator */
  water_stress: 'low' | 'medium' | 'high' | 'extremely_high';
}

/**
 * Land use result
 */
export interface LandUseResult {
  /** Land area (m² per kg product per year) */
  area_m2: number;

  /** Land use type */
  land_type: 'cropland' | 'pasture' | 'forest' | 'urban' | 'mixed';

  /** Biodiversity impact score (0-100, lower is better) */
  biodiversity_impact: number;

  /** Soil health indicators */
  soil_health?: {
    organic_matter_percent: number;
    erosion_rate_t_per_ha_per_year: number;
    compaction_level: 'low' | 'medium' | 'high';
  };
}

/**
 * Comprehensive sustainability metrics
 */
export interface SustainabilityMetrics {
  /** Carbon footprint */
  carbon: CarbonFootprintResult;

  /** Water footprint */
  water: WaterFootprintResult;

  /** Land use */
  land: LandUseResult;

  /** Energy consumption (MJ per kg) */
  energy_mj: number;

  /** Waste generation (kg per kg product) */
  waste_kg: number;

  /** Recyclability score (0-100) */
  recyclability: number;

  /** Overall sustainability score (0-100, higher is better) */
  overall_score: number;

  /** Sustainability rating */
  rating: 'A+' | 'A' | 'B' | 'C' | 'D' | 'F';
}

// ============================================================================
// Traceability Types
// ============================================================================

/**
 * Certification types
 */
export type CertificationType =
  | 'usda_organic'
  | 'eu_organic'
  | 'non_gmo_project'
  | 'fair_trade'
  | 'rainforest_alliance'
  | 'msc_certified'  // Marine Stewardship Council
  | 'asc_certified'  // Aquaculture Stewardship Council
  | 'animal_welfare_approved'
  | 'grass_fed'
  | 'halal'
  | 'kosher'
  | 'gluten_free'
  | 'vegan'
  | 'paleo'
  | 'keto_certified';

/**
 * Certification record
 */
export interface Certification {
  /** Certification type */
  type: CertificationType;

  /** Certifying body */
  certifier: string;

  /** Certificate number */
  certificate_number: string;

  /** Issue date */
  issue_date: string;

  /** Expiration date */
  expiration_date: string;

  /** Verification URL */
  verification_url?: string;

  /** Blockchain hash */
  blockchain_hash?: string;
}

/**
 * Ingredient information
 */
export interface Ingredient {
  /** Ingredient name */
  name: string;

  /** Amount */
  amount: number;

  /** Unit */
  unit: MeasurementUnit;

  /** Percentage of total */
  percent: number;

  /** Origin */
  origin?: {
    country: string;
    region?: string;
    farm?: string;
    coordinates?: { lat: number; lon: number };
  };

  /** Allergen classification */
  is_allergen: boolean;
  allergen_type?: AllergenType;

  /** Certifications */
  certifications?: CertificationType[];

  /** Traceability ID */
  lot_number?: string;
  blockchain_hash?: string;
}

/**
 * Traceability record for blockchain
 */
export interface TraceabilityRecord {
  /** Block identifier */
  block_id: string;

  /** Timestamp */
  timestamp: number;

  /** Product information */
  product: {
    id: string;
    name: string;
    gtin: string;
    lot_number: string;
  };

  /** Origin information */
  origin: {
    farm?: string;
    location?: { lat: number; lon: number };
    certifications: CertificationType[];
  };

  /** Processing history */
  processing: Array<{
    facility: string;
    date: string;
    processes: ProcessingMethod[];
    haccp_verified: boolean;
    temperature_logs?: string;  // IPFS hash
  }>;

  /** Testing records */
  testing: {
    pathogen_screen: 'negative' | 'positive';
    chemical_tests: 'pass' | 'fail';
    lab: string;
    certificate: string;  // IPFS hash
  };

  /** Distribution */
  distribution: {
    shipper: string;
    temperature_range: string;
    transit_time_hours: number;
  };

  /** Blockchain linkage */
  previous_hash: string;
  hash: string;
}

// ============================================================================
// Complete Food Product Type
// ============================================================================

/**
 * Complete food product specification
 */
export interface FoodProduct {
  // Identification
  id: string;
  gtin: string;
  name: string;
  brand?: string;
  category: FoodCategory;

  // Nutritional information
  nutrition: {
    serving_size: { amount: number; unit: MeasurementUnit };
    servings_per_container?: number;
    macros: MacronutrientProfile;
    micros: MicronutrientProfile;
    nutritional_density_score?: NutritionalDensityScore;
  };

  // Ingredients
  ingredients: Ingredient[];
  allergens: AllergenType[];

  // Processing
  processing: ProcessingMethod[];
  packaging: PackagingType;

  // Safety and quality
  haccp_certified: boolean;
  critical_control_points?: CriticalControlPoint[];
  food_safety_index?: FoodSafetyIndex;

  // Certifications
  certifications: Certification[];

  // Sustainability
  sustainability?: SustainabilityMetrics;

  // Traceability
  traceability: {
    lot_number: string;
    production_date: string;
    expiration_date: string;
    origin: {
      country: string;
      region?: string;
      facility?: string;
      coordinates?: { lat: number; lon: number };
    };
    blockchain_hash?: string;
  };

  // Alternative protein specific
  alternative_protein?: AlternativeProteinProduct;

  // Storage and shelf life
  storage: {
    temperature_celsius: number;
    humidity_percent?: number;
    shelf_life_days: number;
    shelf_life_prediction?: ShelfLifePrediction;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Generic API response wrapper
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
    request_id: string;
  };
}

/**
 * Batch processing result
 */
export interface BatchProcessingResult {
  batch_id: string;
  total_items: number;
  processed: number;
  failed: number;
  results: Array<{
    item_id: string;
    status: 'success' | 'failed';
    data?: any;
    error?: string;
  }>;
}

// ============================================================================
// Export all types
// ============================================================================

export default {
  // Re-export all interfaces and types for convenience
};
