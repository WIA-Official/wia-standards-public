/**
 * WIA-IND-004: Beauty Tech Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Beauty Technology Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Beauty Tech Types
// ============================================================================

/**
 * Fitzpatrick skin type classification (I-VI)
 */
export type FitzpatrickType = 'I' | 'II' | 'III' | 'IV' | 'V' | 'VI';

/**
 * Baumann skin type classification (16 types)
 * O/D: Oily vs Dry
 * S/R: Sensitive vs Resistant
 * P/N: Pigmented vs Non-pigmented
 * W/T: Wrinkle-prone vs Tight
 */
export type BaumannType =
  | 'OSNW' | 'OSNP' | 'OSPT' | 'OSPW'
  | 'ORNT' | 'ORNP' | 'ORPT' | 'ORPW'
  | 'DSNW' | 'DSNP' | 'DSPT' | 'DSPW'
  | 'DRNT' | 'DRNP' | 'DRPT' | 'DRPW';

/**
 * Skin undertone classification
 */
export type SkinUndertone = 'warm' | 'cool' | 'neutral' | 'olive';

/**
 * Gender classification (for product recommendations)
 */
export type Gender = 'male' | 'female' | 'non-binary' | 'other';

/**
 * Ethnicity classification (optional, for research)
 */
export type Ethnicity =
  | 'caucasian'
  | 'african'
  | 'asian'
  | 'hispanic'
  | 'middle_eastern'
  | 'mixed'
  | 'other';

// ============================================================================
// Image and Analysis Types
// ============================================================================

/**
 * Image metadata for skin analysis
 */
export interface ImageMetadata {
  /** Image resolution (e.g., "4032x3024") */
  resolution: string;

  /** Camera or device used */
  camera: string;

  /** Lighting condition */
  lighting: 'natural_daylight' | 'studio' | 'indoor' | 'outdoor' | 'mixed';

  /** Distance from camera in cm */
  distance_cm: number;

  /** Camera angle */
  angle: 'frontal' | 'left_45' | 'right_45' | 'left_90' | 'right_90';

  /** Timestamp of capture */
  timestamp: string;

  /** Image file format */
  format: 'jpeg' | 'png' | 'raw' | 'heic';

  /** Color space */
  color_space: 'sRGB' | 'Adobe_RGB' | 'ProPhoto_RGB';
}

/**
 * Facial landmark point (2D or 3D)
 */
export interface FacialLandmark {
  /** Landmark ID */
  id: number;

  /** X coordinate (pixel) */
  x: number;

  /** Y coordinate (pixel) */
  y: number;

  /** Z coordinate (depth, optional) */
  z?: number;

  /** Detection confidence (0-1) */
  confidence?: number;
}

/**
 * Facial landmark set
 */
export interface FacialLandmarks {
  /** Detection method used */
  method: 'mediapipe_468' | 'dlib_68' | 'custom_cnn' | '3dmm';

  /** Overall detection confidence */
  confidence: number;

  /** Array of landmark points */
  points: FacialLandmark[];

  /** Face bounding box */
  bounding_box?: {
    x: number;
    y: number;
    width: number;
    height: number;
  };
}

/**
 * Region of interest on face
 */
export interface FaceRegion {
  /** Region name */
  name: 'forehead' | 't_zone' | 'cheeks' | 'nose' | 'chin' | 'eye_area' | 'mouth_area' | 'jawline';

  /** Polygon points defining region */
  polygon: Array<{ x: number; y: number }>;

  /** Area in pixels */
  area_pixels: number;
}

// ============================================================================
// Skin Analysis Types
// ============================================================================

/**
 * Hydration measurement
 */
export interface HydrationMeasurement {
  /** Overall hydration value (0-100) */
  value: number;

  /** Measurement method */
  method: 'capacitance' | 'tewl' | 'image_based' | 'corneometer';

  /** Unit of measurement */
  unit: 'AU' | 'g/m2/h' | 'score';

  /** Breakdown by face zone */
  zone_breakdown?: {
    forehead?: number;
    cheeks?: number;
    t_zone?: number;
    chin?: number;
  };

  /** Classification */
  classification: 'very_dry' | 'dry' | 'normal' | 'well_hydrated';

  /** Trans-epidermal water loss (if measured) */
  tewl?: number;
}

/**
 * Elasticity and firmness measurement
 */
export interface ElasticityMeasurement {
  /** Elasticity value (0-1 for R2, or 0-100 score) */
  value: number;

  /** Measurement method */
  method: 'cutometer_R2' | 'cutometer_R5' | 'cutometer_R7' | 'suction' | 'image_based';

  /** Age-adjusted score (0-100) */
  age_adjusted: number;

  /** Firmness value (N/mm if physical measurement) */
  firmness?: number;

  /** Classification */
  classification: 'excellent' | 'good' | 'fair' | 'poor';

  /** Detailed Cutometer parameters (if available) */
  cutometer_params?: {
    R0: number; // Immediate deformation
    R2: number; // Gross elasticity
    R5: number; // Net elasticity
    R7: number; // Biological elasticity
  };
}

/**
 * Pore analysis
 */
export interface PoreAnalysis {
  /** Number of pores per cm² */
  count_per_cm2: number;

  /** Average pore diameter in micrometers */
  average_diameter_um: number;

  /** Pore quality score (0-100) */
  quality_score: number;

  /** Size distribution */
  size_distribution?: {
    small: number; // Count of pores < 250 μm
    medium: number; // Count of pores 250-400 μm
    large: number; // Count of pores > 400 μm
  };

  /** Pore density (% of skin surface) */
  density_percent: number;

  /** Classification */
  classification: 'small' | 'medium' | 'large' | 'very_large';

  /** Most affected zones */
  affected_zones: string[];
}

/**
 * Wrinkle assessment
 */
export interface WrinkleAssessment {
  /** Overall severity score (0-10) */
  severity_score: number;

  /** Maximum wrinkle depth in mm */
  depth_max_mm: number;

  /** Total wrinkle length in mm */
  total_length_mm: number;

  /** Wrinkle classification */
  classification: 'minimal' | 'mild' | 'moderate' | 'significant' | 'severe';

  /** Breakdown by facial zone */
  zones: {
    forehead?: number;
    crows_feet?: number;
    smile_lines?: number;
    frown_lines?: number;
    marionette_lines?: number;
  };

  /** Surface roughness metrics (if 3D profiling) */
  surface_metrics?: {
    Ra: number; // Average roughness (μm)
    Rz: number; // Max peak-to-valley (μm)
    Rmax: number; // Maximum depth (μm)
  };

  /** Wrinkle types detected */
  types: Array<'fine_lines' | 'expression_lines' | 'static_wrinkles' | 'deep_furrows'>;
}

/**
 * Pigmentation analysis
 */
export interface PigmentationAnalysis {
  /** Melanin index */
  melanin_index: number;

  /** Tone evenness score (0-100) */
  evenness_score: number;

  /** Number of pigmentation spots */
  spot_count: number;

  /** Hyperpigmentation area as % of face */
  hyperpigmentation_area_percent: number;

  /** Hemoglobin index */
  hemoglobin_index?: number;

  /** Individual spots detected */
  spots?: Array<{
    type: 'freckle' | 'age_spot' | 'melasma' | 'post_inflammatory';
    location: { x: number; y: number };
    size_mm2: number;
    color_intensity: number; // ΔE from surrounding skin
  }>;

  /** Distribution uniformity (0-100) */
  uniformity: number;

  /** Classification */
  classification: 'excellent' | 'good' | 'fair' | 'poor';
}

/**
 * Redness and inflammation assessment
 */
export interface RednessAssessment {
  /** Erythema index */
  erythema_index: number;

  /** Severity classification */
  severity: 'normal' | 'mild' | 'moderate' | 'severe';

  /** Vessel density (% of skin area) */
  vessel_density_percent: number;

  /** Average vessel diameter in micrometers */
  average_vessel_diameter_um?: number;

  /** Conditions detected */
  conditions: Array<'rosacea' | 'spider_veins' | 'sensitive_skin' | 'inflammation' | 'none'>;

  /** Most affected zones */
  affected_zones: string[];

  /** Capillary pattern analysis */
  capillary_pattern?: {
    density: number;
    tortuosity_index: number;
    distribution: 'focal' | 'diffuse' | 'patchy';
  };
}

/**
 * Comprehensive skin analysis result
 */
export interface SkinAnalysisResult {
  /** Unique report ID */
  report_id: string;

  /** User ID */
  user_id: string;

  /** Analysis timestamp */
  timestamp: string;

  /** API version */
  version: string;

  /** Image metadata */
  image_metadata: ImageMetadata;

  /** Skin classification */
  classification: {
    fitzpatrick_type: FitzpatrickType;
    baumann_type: BaumannType;
    ita_value: number; // Individual Typology Angle
    undertone: SkinUndertone;
  };

  /** Quantitative measurements */
  measurements: {
    hydration: HydrationMeasurement;
    elasticity: ElasticityMeasurement;
    pores: PoreAnalysis;
    wrinkles: WrinkleAssessment;
    pigmentation: PigmentationAnalysis;
    redness: RednessAssessment;
  };

  /** Overall scores */
  overall_scores: {
    skin_health: number; // 0-100 composite score
    skin_age: number; // Estimated skin age
    chronological_age?: number; // Actual age if provided
  };

  /** Detected concerns */
  concerns_detected: Array<{
    concern: SkinConcern;
    severity: 'mild' | 'moderate' | 'severe';
    affected_area: string;
    confidence: number;
  }>;

  /** Recommendations */
  recommendations: {
    immediate: string[];
    long_term: string[];
    products?: ProductRecommendation[];
  };
}

/**
 * Skin concerns enumeration
 */
export type SkinConcern =
  | 'acne'
  | 'large_pores'
  | 'oily_skin'
  | 'dry_skin'
  | 'dehydration'
  | 'fine_lines'
  | 'wrinkles'
  | 'sagging'
  | 'dark_spots'
  | 'hyperpigmentation'
  | 'melasma'
  | 'uneven_tone'
  | 'dullness'
  | 'redness'
  | 'rosacea'
  | 'sensitivity'
  | 'dark_circles'
  | 'puffiness'
  | 'blackheads'
  | 'whiteheads'
  | 'scarring'
  | 'enlarged_pores';

// ============================================================================
// Virtual Makeup Types
// ============================================================================

/**
 * Makeup product type
 */
export type MakeupProductType =
  | 'foundation'
  | 'concealer'
  | 'powder'
  | 'blush'
  | 'bronzer'
  | 'highlighter'
  | 'contour'
  | 'eyeshadow'
  | 'eyeliner'
  | 'mascara'
  | 'eyebrow'
  | 'lipstick'
  | 'lip_gloss'
  | 'lip_liner';

/**
 * Makeup finish type
 */
export type MakeupFinish = 'matte' | 'satin' | 'dewy' | 'glossy' | 'metallic' | 'shimmer' | 'pearl';

/**
 * Coverage level
 */
export type CoverageLevel = 'sheer' | 'light' | 'medium' | 'full' | 'buildable';

/**
 * Color representation in LAB color space
 */
export interface ColorLAB {
  /** Lightness (0-100) */
  L: number;

  /** Green-Red axis (-128 to +127) */
  a: number;

  /** Blue-Yellow axis (-128 to +127) */
  b: number;
}

/**
 * Makeup product definition
 */
export interface MakeupProduct {
  /** Product type */
  type: MakeupProductType;

  /** Brand name */
  brand?: string;

  /** Product name */
  name?: string;

  /** Shade name */
  shade?: string;

  /** Color in hex */
  color_hex?: string;

  /** Color in LAB (more accurate) */
  color_lab?: ColorLAB;

  /** Coverage level */
  coverage?: CoverageLevel;

  /** Finish type */
  finish: MakeupFinish;

  /** Opacity (0-1) */
  opacity?: number;

  /** Intensity (0-1) */
  intensity?: number;

  /** Application zone (for targeted products) */
  placement?: string;

  /** Blend radius in pixels */
  blend_radius?: number;
}

/**
 * Virtual makeup try-on request
 */
export interface VirtualMakeupRequest {
  /** Base image URL or data */
  image: string | Blob;

  /** Products to apply */
  products: MakeupProduct[];

  /** Try-on options */
  options?: {
    /** Auto-adjust for lighting */
    auto_adjust?: boolean;

    /** Target lighting condition */
    lighting?: 'natural' | 'studio' | 'indoor' | 'outdoor';

    /** Quality level */
    quality?: 'low' | 'medium' | 'high' | 'ultra';

    /** Enable skin retouching */
    skin_retouch?: boolean;

    /** Retouch intensity (0-1) */
    retouch_intensity?: number;
  };
}

/**
 * Virtual makeup try-on result
 */
export interface VirtualMakeupResult {
  /** Unique session ID */
  session_id: string;

  /** Result image URL */
  result_url: string;

  /** Thumbnail URL */
  thumbnail_url?: string;

  /** Facial landmarks detected */
  landmarks: FacialLandmarks;

  /** Products successfully applied */
  products_applied: MakeupProduct[];

  /** Processing metadata */
  metadata: {
    processing_time_ms: number;
    rendering_quality: string;
    lighting_adjusted: boolean;
  };
}

/**
 * Foundation shade match
 */
export interface FoundationMatch {
  /** Product ID */
  product_id: string;

  /** Brand name */
  brand: string;

  /** Product name */
  product_name: string;

  /** Shade name */
  shade_name: string;

  /** Match score (0-100) */
  match_score: number;

  /** Color in LAB */
  color_lab: ColorLAB;

  /** Delta E (color difference) */
  delta_e: number;

  /** Undertone match */
  undertone: SkinUndertone;

  /** Oxidation consideration */
  oxidation_adjusted: boolean;

  /** Coverage options */
  coverage_options: CoverageLevel[];

  /** Finish options */
  finish_options: MakeupFinish[];
}

// ============================================================================
// Beauty Device IoT Types
// ============================================================================

/**
 * Beauty device category
 */
export type BeautyDeviceCategory =
  | 'cleansing'
  | 'led_therapy'
  | 'microcurrent'
  | 'skin_scanner'
  | 'massage'
  | 'hair_care'
  | 'ipl_laser'
  | 'heat_therapy'
  | 'cryo_therapy';

/**
 * Device connectivity type
 */
export type DeviceConnectivity = 'bluetooth_le' | 'wifi' | 'usb' | 'zigbee' | 'proprietary';

/**
 * Beauty device information
 */
export interface BeautyDevice {
  /** Unique device ID */
  device_id: string;

  /** Device category */
  category: BeautyDeviceCategory;

  /** Manufacturer */
  manufacturer: string;

  /** Model name */
  model: string;

  /** Firmware version */
  firmware_version: string;

  /** Hardware version */
  hardware_version?: string;

  /** Connectivity type */
  connectivity: DeviceConnectivity;

  /** Device capabilities */
  capabilities: {
    modes: string[];
    intensity_levels: number;
    timer_range: [number, number]; // [min, max] in seconds
    sensors: string[];
  };

  /** Battery information */
  battery?: {
    level_percent: number;
    charging: boolean;
    type: 'lithium_ion' | 'lithium_polymer' | 'nimh';
  };

  /** Last connection timestamp */
  last_connected?: string;
}

/**
 * Device treatment session
 */
export interface DeviceSession {
  /** Unique session ID */
  session_id: string;

  /** Device ID */
  device_id: string;

  /** User ID */
  user_id: string;

  /** Session start time */
  start_time: string;

  /** Session duration in seconds */
  duration: number;

  /** Treatment mode used */
  mode: string;

  /** Device settings */
  settings: {
    intensity?: number;
    temperature?: number;
    frequency?: number;
    [key: string]: any;
  };

  /** Measurements taken during session */
  measurements?: {
    avg_pressure?: number;
    coverage_percent?: number;
    temperature_readings?: number[];
    [key: string]: any;
  };

  /** User feedback */
  feedback?: {
    effectiveness: number; // 1-5 rating
    comfort: number; // 1-5 rating
    notes?: string;
  };
}

/**
 * LED therapy device settings
 */
export interface LEDTherapySettings {
  /** Wavelength selection */
  wavelength: 'red' | 'blue' | 'yellow' | 'green' | 'infrared' | 'multi';

  /** Specific wavelengths in nm (if multi) */
  wavelengths_nm?: number[];

  /** Power density in mW/cm² */
  power_density: number;

  /** Treatment duration in minutes */
  duration_minutes: number;

  /** Pulsed mode settings */
  pulsed?: {
    enabled: boolean;
    on_seconds: number;
    off_seconds: number;
  };

  /** Intensity level (0-100) */
  intensity: number;
}

/**
 * Skin scanner measurement
 */
export interface SkinScannerMeasurement {
  /** Measurement timestamp */
  timestamp: string;

  /** Hydration reading (0-100) */
  hydration?: number;

  /** Oil level reading (0-100) */
  oil_level?: number;

  /** Skin temperature in Celsius */
  temperature?: number;

  /** Melanin index */
  melanin_index?: number;

  /** Pore count in scanned area */
  pore_count?: number;

  /** Image captured */
  image_url?: string;

  /** Location on face */
  location: 'forehead' | 'cheek_left' | 'cheek_right' | 't_zone' | 'chin';
}

// ============================================================================
// Personalized Skincare Types
// ============================================================================

/**
 * Skin profile for personalization
 */
export interface SkinProfile {
  /** User ID */
  user_id: string;

  /** Age */
  age: number;

  /** Gender */
  gender: Gender;

  /** Ethnicity (optional) */
  ethnicity?: Ethnicity;

  /** Fitzpatrick type */
  fitzpatrick_type: FitzpatrickType;

  /** Baumann type */
  baumann_type: BaumannType;

  /** Current measurements */
  measurements: {
    hydration: number;
    oil_level: number;
    elasticity: number;
    pore_quality: number;
    pigmentation_evenness: number;
    wrinkle_score: number;
    redness_index: number;
    skin_age: number;
  };

  /** Main skin concerns */
  concerns: SkinConcern[];

  /** Skincare goals */
  goals: string[];

  /** User preferences */
  preferences: {
    natural_ingredients?: boolean;
    fragrance_free?: boolean;
    cruelty_free?: boolean;
    vegan?: boolean;
    budget?: 'budget' | 'moderate' | 'premium' | 'luxury';
    routine_complexity?: 'simple' | 'moderate' | 'extensive';
  };

  /** Known allergies or sensitivities */
  allergies: string[];

  /** Environmental factors */
  environment?: {
    climate: 'tropical' | 'subtropical' | 'temperate' | 'cold' | 'arid';
    pollution_level: 'low' | 'moderate' | 'high';
    uv_index_avg: number;
    indoor_heating?: boolean;
    air_conditioning?: boolean;
  };

  /** Current routine (if any) */
  current_routine?: SkincareRoutine;
}

/**
 * Skincare routine
 */
export interface SkincareRoutine {
  /** Morning routine steps */
  morning: RoutineStep[];

  /** Evening routine steps */
  evening: RoutineStep[];

  /** Weekly treatments */
  weekly?: RoutineStep[];

  /** As-needed treatments */
  as_needed?: RoutineStep[];
}

/**
 * Individual routine step
 */
export interface RoutineStep {
  /** Step order (1-10) */
  order: number;

  /** Product category */
  category: ProductCategory;

  /** Specific product (if recommended) */
  product?: ProductRecommendation;

  /** Usage instructions */
  instructions: string;

  /** Frequency */
  frequency: 'daily' | 'twice_daily' | 'alternate_days' | '2-3x_week' | 'weekly' | 'as_needed';

  /** Wait time before next step (minutes) */
  wait_time_minutes?: number;

  /** Optional step */
  optional: boolean;
}

/**
 * Product category
 */
export type ProductCategory =
  | 'oil_cleanser'
  | 'cleanser'
  | 'exfoliant'
  | 'toner'
  | 'essence'
  | 'serum'
  | 'treatment'
  | 'eye_cream'
  | 'moisturizer'
  | 'sunscreen'
  | 'night_cream'
  | 'sleeping_mask'
  | 'sheet_mask'
  | 'spot_treatment';

/**
 * Product recommendation
 */
export interface ProductRecommendation {
  /** Product ID */
  product_id: string;

  /** Product name */
  name: string;

  /** Brand */
  brand: string;

  /** Category */
  category: ProductCategory;

  /** Recommendation score (0-100) */
  score: number;

  /** Why this product is recommended */
  reasoning: string;

  /** Key ingredients */
  key_ingredients: string[];

  /** Addresses these concerns */
  addresses_concerns: SkinConcern[];

  /** Priority (1 = highest) */
  priority: number;

  /** Price */
  price?: {
    amount: number;
    currency: string;
  };

  /** Where to buy */
  purchase_links?: Array<{
    retailer: string;
    url: string;
    price: number;
  }>;
}

/**
 * Ingredient information
 */
export interface IngredientInfo {
  /** INCI name (International Nomenclature Cosmetic Ingredient) */
  inci_name: string;

  /** Common name */
  common_name?: string;

  /** Chemical name */
  chemical_name?: string;

  /** Ingredient functions */
  functions: IngredientFunction[];

  /** Typical concentration range */
  concentration_range?: [number, number]; // [min, max] in %

  /** pH compatibility */
  ph_range?: [number, number];

  /** Safety information */
  safety: {
    ewg_rating: number; // 0-10 scale
    cir_approved: boolean;
    eu_approved: boolean;
    allergen_potential: 'low' | 'moderate' | 'high';
    pregnancy_safe: boolean;
  };

  /** Efficacy data */
  efficacy?: {
    clinical_studies_count: number;
    proven_effective: boolean;
    time_to_results_weeks: number;
  };

  /** Interactions */
  interactions?: {
    synergies: string[]; // Works well with
    conflicts: string[]; // Avoid combining with
  };
}

/**
 * Ingredient function
 */
export type IngredientFunction =
  | 'humectant'
  | 'emollient'
  | 'occlusive'
  | 'antioxidant'
  | 'anti_aging'
  | 'brightening'
  | 'exfoliant'
  | 'anti_inflammatory'
  | 'antibacterial'
  | 'uv_filter'
  | 'preservative'
  | 'fragrance'
  | 'colorant'
  | 'emulsifier'
  | 'thickener'
  | 'ph_adjuster'
  | 'penetration_enhancer';

// ============================================================================
// Hair Analysis Types
// ============================================================================

/**
 * Hair type classification
 */
export type HairType =
  | '1A' | '1B' | '1C' // Straight
  | '2A' | '2B' | '2C' // Wavy
  | '3A' | '3B' | '3C' // Curly
  | '4A' | '4B' | '4C'; // Coily/Kinky

/**
 * Hair porosity
 */
export type HairPorosity = 'low' | 'medium' | 'high';

/**
 * Hair density
 */
export type HairDensity = 'low' | 'medium' | 'high';

/**
 * Hair thickness
 */
export type HairThickness = 'fine' | 'medium' | 'thick';

/**
 * Scalp condition
 */
export interface ScalpCondition {
  /** Scalp type */
  type: 'oily' | 'dry' | 'balanced' | 'sensitive';

  /** Sebum level (μg/cm²) */
  sebum_level: number;

  /** Hydration level (0-100) */
  hydration: number;

  /** pH level */
  ph?: number;

  /** Inflammation/redness (0-100) */
  inflammation: number;

  /** Flakiness score (0-100) */
  flakiness: number;

  /** Conditions detected */
  conditions: Array<'dandruff' | 'seborrheic_dermatitis' | 'psoriasis' | 'folliculitis' | 'none'>;
}

/**
 * Hair structure analysis
 */
export interface HairStructureAnalysis {
  /** Hair type */
  type: HairType;

  /** Fiber thickness in micrometers */
  thickness_um: number;

  /** Thickness classification */
  thickness_class: HairThickness;

  /** Hair density (hairs per cm²) */
  density_per_cm2: number;

  /** Density classification */
  density_class: HairDensity;

  /** Porosity level */
  porosity: HairPorosity;

  /** Elasticity (% stretch before break) */
  elasticity_percent: number;

  /** Damage assessment (0-100, 0=healthy) */
  damage_score: number;

  /** Specific damage types */
  damage_types: Array<'split_ends' | 'breakage' | 'cuticle_damage' | 'color_damage' | 'chemical_damage'>;

  /** Split ends percentage */
  split_ends_percent?: number;
}

/**
 * Hair color analysis
 */
export interface HairColorAnalysis {
  /** Natural hair level (1-10 scale) */
  natural_level: number;

  /** Undertone */
  undertone: 'warm' | 'cool' | 'neutral';

  /** Dominant pigment */
  dominant_pigment: 'eumelanin' | 'pheomelanin' | 'mixed';

  /** Color in LAB */
  color_lab: ColorLAB;

  /** Gray coverage percentage */
  gray_percent?: number;

  /** Previously colored */
  colored: boolean;

  /** Color fade assessment (if colored) */
  fade_level?: 'none' | 'mild' | 'moderate' | 'significant';
}

/**
 * Hair loss assessment
 */
export interface HairLossAssessment {
  /** Hair count in measured area */
  hair_count: number;

  /** Percentage change from baseline */
  change_percent?: number;

  /** Growth phase ratio */
  phase_ratio?: {
    anagen: number; // Growth phase (%)
    catagen: number; // Transition phase (%)
    telogen: number; // Resting phase (%)
  };

  /** Daily shedding estimate */
  daily_shedding?: number;

  /** Miniaturization detected */
  miniaturization: boolean;

  /** Pattern baldness scale */
  pattern_scale?: {
    male_norwood?: number; // 1-7
    female_ludwig?: number; // 1-3
  };

  /** Classification */
  classification: 'normal' | 'mild_thinning' | 'moderate_loss' | 'significant_loss';
}

/**
 * Complete hair analysis result
 */
export interface HairAnalysisResult {
  /** Analysis ID */
  analysis_id: string;

  /** User ID */
  user_id: string;

  /** Timestamp */
  timestamp: string;

  /** Scalp condition */
  scalp: ScalpCondition;

  /** Hair structure */
  structure: HairStructureAnalysis;

  /** Hair color */
  color: HairColorAnalysis;

  /** Hair loss assessment */
  loss_assessment?: HairLossAssessment;

  /** Concerns detected */
  concerns: string[];

  /** Product recommendations */
  recommendations: ProductRecommendation[];
}

// ============================================================================
// Progress Tracking Types
// ============================================================================

/**
 * Progress comparison
 */
export interface ProgressComparison {
  /** Comparison ID */
  comparison_id: string;

  /** User ID */
  user_id: string;

  /** Baseline analysis */
  baseline: {
    analysis_id: string;
    date: string;
    image_url: string;
  };

  /** Current analysis */
  current: {
    analysis_id: string;
    date: string;
    image_url: string;
  };

  /** Time between analyses (days) */
  days_elapsed: number;

  /** Quantified improvements */
  improvements: {
    wrinkle_reduction_percent?: number;
    pigmentation_improvement_percent?: number;
    hydration_increase_percent?: number;
    elasticity_increase_percent?: number;
    pore_size_reduction_percent?: number;
    overall_skin_health_change?: number;
  };

  /** Visual comparison */
  comparison_image_url?: string;

  /** Statistical significance */
  significance?: {
    wrinkles_significant: boolean;
    pigmentation_significant: boolean;
    p_value: number;
  };

  /** Summary */
  summary: string;
}

/**
 * Progress tracking timeline
 */
export interface ProgressTimeline {
  /** User ID */
  user_id: string;

  /** Timeline start date */
  start_date: string;

  /** All analysis points */
  timeline: Array<{
    date: string;
    analysis_id: string;
    skin_health_score: number;
    key_metrics: {
      hydration: number;
      elasticity: number;
      pigmentation: number;
      wrinkles: number;
    };
    notes?: string;
  }>;

  /** Overall trend */
  trend: {
    direction: 'improving' | 'stable' | 'declining';
    improvement_rate_percent_per_month: number;
  };
}

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Generic API response
 */
export interface APIResponse<T> {
  /** Response status */
  status: 'success' | 'error';

  /** Response data (if successful) */
  data?: T;

  /** Error message (if error) */
  error?: {
    code: string;
    message: string;
    details?: any;
  };

  /** Response metadata */
  metadata?: {
    timestamp: string;
    request_id: string;
    processing_time_ms: number;
  };
}

/**
 * Skin analysis request
 */
export interface SkinAnalysisRequest {
  /** Image data (base64 or URL) */
  image: string | Blob;

  /** Analysis options */
  options: {
    /** Analysis depth */
    depth: 'quick' | 'standard' | 'comprehensive';

    /** Enable specific detections */
    detect_pores?: boolean;
    detect_wrinkles?: boolean;
    detect_pigmentation?: boolean;
    detect_redness?: boolean;

    /** Include recommendations */
    include_recommendations?: boolean;

    /** User context (for better recommendations) */
    user_context?: Partial<SkinProfile>;
  };
}

/**
 * Product recommendation request
 */
export interface ProductRecommendationRequest {
  /** Skin profile */
  skin_profile: Partial<SkinProfile>;

  /** Specific concerns to address */
  concerns?: SkinConcern[];

  /** User preferences */
  preferences?: {
    budget?: 'budget' | 'moderate' | 'premium' | 'luxury';
    natural?: boolean;
    fragrance_free?: boolean;
    vegan?: boolean;
    cruelty_free?: boolean;
  };

  /** Number of recommendations to return */
  limit?: number;

  /** Category filter */
  category?: ProductCategory;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Color match score
 */
export interface ColorMatchScore {
  /** Match score (0-100) */
  score: number;

  /** Delta E (color difference) */
  delta_e: number;

  /** Color difference components */
  components: {
    delta_L: number;
    delta_a: number;
    delta_b: number;
  };

  /** Perceptual classification */
  classification: 'exact_match' | 'very_close' | 'close' | 'similar' | 'different';
}

/**
 * Date range
 */
export interface DateRange {
  /** Start date (ISO 8601) */
  start: string;

  /** End date (ISO 8601) */
  end: string;
}

/**
 * Pagination
 */
export interface Pagination {
  /** Current page (1-indexed) */
  page: number;

  /** Items per page */
  per_page: number;

  /** Total items */
  total: number;

  /** Total pages */
  total_pages: number;
}

/**
 * Geolocation
 */
export interface Geolocation {
  /** Latitude */
  latitude: number;

  /** Longitude */
  longitude: number;

  /** City */
  city?: string;

  /** Country */
  country?: string;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  // Re-export for convenience
  FitzpatrickType,
  BaumannType,
  SkinUndertone,
  MakeupProductType,
  MakeupFinish,
  BeautyDeviceCategory,
  SkinConcern,
  ProductCategory,
  HairType,
  HairPorosity,
  HairDensity,
  HairThickness,
};
