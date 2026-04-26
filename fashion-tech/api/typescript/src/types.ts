/**
 * WIA-IND-001: Fashion Tech Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Fashion Technology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Fashion Types
// ============================================================================

/**
 * Garment category classification
 */
export type GarmentCategory =
  | 'tops'
  | 'bottoms'
  | 'dresses'
  | 'outerwear'
  | 'suits'
  | 'activewear'
  | 'intimate'
  | 'accessories'
  | 'shoes'
  | 'bags';

/**
 * Gender classification
 */
export type Gender = 'womens' | 'mens' | 'unisex' | 'kids' | 'baby';

/**
 * Season classification
 */
export type Season = 'spring' | 'summer' | 'fall' | 'winter' | 'all_season';

/**
 * Fashion style categories
 */
export type FashionStyle =
  | 'minimalist'
  | 'bohemian'
  | 'preppy'
  | 'streetwear'
  | 'vintage'
  | 'classic'
  | 'romantic'
  | 'edgy'
  | 'sporty'
  | 'business'
  | 'casual'
  | 'formal';

/**
 * Occasion types
 */
export type Occasion =
  | 'work'
  | 'casual'
  | 'formal'
  | 'evening'
  | 'party'
  | 'sport'
  | 'beach'
  | 'travel'
  | 'wedding'
  | 'date';

// ============================================================================
// Material & Fabric Types
// ============================================================================

/**
 * Material types
 */
export type MaterialType =
  // Natural fibers
  | 'cotton'
  | 'organic_cotton'
  | 'linen'
  | 'silk'
  | 'wool'
  | 'cashmere'
  | 'hemp'
  | 'bamboo'
  // Synthetic fibers
  | 'polyester'
  | 'recycled_polyester'
  | 'nylon'
  | 'spandex'
  | 'acrylic'
  | 'rayon'
  // Innovative materials
  | 'tencel'
  | 'modal'
  | 'lyocell'
  | 'pinatex'
  | 'mushroom_leather'
  | 'vegan_leather'
  // Blends
  | 'cotton_blend'
  | 'poly_blend';

/**
 * Fabric construction types
 */
export type FabricConstruction =
  | 'woven'
  | 'knit'
  | 'jersey'
  | 'denim'
  | 'fleece'
  | 'lace'
  | 'mesh'
  | 'twill'
  | 'satin'
  | 'chiffon';

/**
 * Material composition
 */
export interface MaterialComposition {
  /** Material type */
  type: MaterialType;

  /** Percentage of total (0-100) */
  percentage: number;

  /** Weight in g/m² */
  weight_gsm?: number;

  /** Country of origin */
  origin?: string;

  /** Certification (GOTS, OEKO-TEX, etc.) */
  certification?: string;

  /** Sustainability score (0-100) */
  sustainability_score?: number;
}

/**
 * Fabric properties
 */
export interface FabricProperties {
  /** Construction type */
  construction: FabricConstruction;

  /** Weight in g/m² */
  weight_gsm: number;

  /** Fabric density (g/cm³) */
  density?: number;

  /** Stretch factor (1.0 = no stretch, 2.0 = 100% stretch) */
  stretch_factor: number;

  /** Recovery after stretch (0-1) */
  recovery: number;

  /** Drape coefficient (0 = stiff, 1 = fluid) */
  drape: number;

  /** Breathability (0-1) */
  breathability?: number;

  /** Moisture wicking (0-1) */
  moisture_wicking?: number;

  /** Insulation factor (0-1) */
  insulation?: number;

  /** UV protection factor */
  upf?: number;
}

/**
 * Physical material properties for simulation
 */
export interface MaterialPhysics {
  /** Spring stiffness (N/m) */
  stiffness: number;

  /** Damping coefficient (0-1) */
  damping: number;

  /** Stretch factors [warp, weft] */
  stretch: [number, number];

  /** Friction coefficient */
  friction: number;

  /** Air permeability */
  air_permeability?: number;
}

// ============================================================================
// Color & Pattern Types
// ============================================================================

/**
 * Color representation
 */
export interface Color {
  /** Hex color code */
  hex: string;

  /** Color name */
  name: string;

  /** sRGB values [0-255, 0-255, 0-255] */
  sRGB?: [number, number, number];

  /** Pantone code */
  pantone?: string;

  /** Spectral reflectance curve (optional) */
  spectral?: number[];

  /** Color category */
  category?: 'warm' | 'cool' | 'neutral' | 'pastel' | 'neon';

  /** Suitable seasons */
  seasons?: Season[];
}

/**
 * Pattern type
 */
export type PatternType =
  | 'solid'
  | 'striped'
  | 'checkered'
  | 'polka_dot'
  | 'floral'
  | 'geometric'
  | 'abstract'
  | 'animal_print'
  | 'paisley'
  | 'plaid'
  | 'houndstooth';

/**
 * Pattern information
 */
export interface Pattern {
  /** Pattern type */
  type: PatternType;

  /** Pattern scale (small, medium, large) */
  scale?: 'small' | 'medium' | 'large';

  /** Colors in pattern */
  colors: Color[];

  /** Pattern repeat size in cm */
  repeat_size?: number;
}

// ============================================================================
// Size & Measurement Types
// ============================================================================

/**
 * Standard size classification
 */
export type StandardSize =
  | 'XXXS'
  | 'XXS'
  | 'XS'
  | 'S'
  | 'M'
  | 'L'
  | 'XL'
  | 'XXL'
  | 'XXXL'
  | '4XL'
  | '5XL';

/**
 * Size modifier for height variations
 */
export type SizeModifier = 'petite' | 'regular' | 'tall' | 'plus';

/**
 * Body type classification
 */
export type BodyType = 'pear' | 'hourglass' | 'apple' | 'rectangle' | 'inverted_triangle';

/**
 * Body measurements in centimeters
 */
export interface BodyMeasurements {
  /** Height in cm */
  height: number;

  /** Weight in kg */
  weight?: number;

  /** Chest/bust circumference in cm */
  chest: number;

  /** Under-bust circumference in cm */
  under_bust?: number;

  /** Waist circumference in cm */
  waist: number;

  /** Hip circumference in cm */
  hips: number;

  /** High hip circumference in cm */
  high_hip?: number;

  /** Inseam length in cm */
  inseam?: number;

  /** Arm length (shoulder to wrist) in cm */
  arm_length?: number;

  /** Shoulder width in cm */
  shoulder?: number;

  /** Neck circumference in cm */
  neck?: number;

  /** Thigh circumference in cm */
  thigh?: number;

  /** Calf circumference in cm */
  calf?: number;

  /** Shoe size (region-specific) */
  shoe_size?: number;

  /** Shoe size region */
  shoe_region?: 'US' | 'EU' | 'UK' | 'JP' | 'CN';
}

/**
 * Garment measurements
 */
export interface GarmentMeasurements {
  /** Chest/bust width in cm */
  chest?: number;

  /** Waist width in cm */
  waist?: number;

  /** Hip width in cm */
  hips?: number;

  /** Total length in cm */
  length?: number;

  /** Sleeve length in cm */
  sleeve_length?: number;

  /** Shoulder width in cm */
  shoulder?: number;

  /** Inseam in cm */
  inseam?: number;

  /** Rise in cm (pants) */
  rise?: number;

  /** Hem width in cm */
  hem_width?: number;
}

/**
 * Size recommendation
 */
export interface SizeRecommendation {
  /** Recommended size */
  size: StandardSize;

  /** Confidence level (0-1) */
  confidence: number;

  /** Fit prediction */
  fit: 'tight' | 'fitted' | 'true_to_size' | 'relaxed' | 'oversized';

  /** Specific fit details */
  details: {
    chest?: 'tight' | 'comfortable' | 'loose';
    waist?: 'tight' | 'comfortable' | 'loose';
    hips?: 'tight' | 'comfortable' | 'loose';
    length?: 'short' | 'perfect' | 'long';
    sleeves?: 'short' | 'perfect' | 'long';
  };

  /** Alternative sizes */
  alternatives?: Array<{
    size: StandardSize;
    confidence: number;
    note: string;
  }>;
}

// ============================================================================
// 3D Model Types
// ============================================================================

/**
 * 3D model format
 */
export type ModelFormat = 'glTF' | 'GLB' | 'FBX' | 'OBJ' | 'USD' | 'USDZ' | 'Alembic';

/**
 * Level of detail
 */
export type LOD = 'ultra_low' | 'low' | 'medium' | 'high' | 'ultra_high';

/**
 * 3D model specification
 */
export interface Model3D {
  /** Model format */
  format: ModelFormat;

  /** Level of detail */
  lod: LOD;

  /** URL to model file */
  url: string;

  /** Polygon/triangle count */
  polygon_count: number;

  /** Vertex count */
  vertex_count?: number;

  /** File size in bytes */
  file_size?: number;

  /** Rigged for animation */
  rigged?: boolean;

  /** Animation clips included */
  animations?: string[];
}

/**
 * Texture map types
 */
export interface TextureMaps {
  /** Base color/albedo texture */
  base_color: string;

  /** Normal map */
  normal?: string;

  /** Roughness map */
  roughness?: string;

  /** Metallic map */
  metallic?: string;

  /** Ambient occlusion map */
  ao?: string;

  /** Emission map */
  emission?: string;

  /** Opacity/transparency map */
  opacity?: string;

  /** Height/displacement map */
  height?: string;
}

/**
 * PBR material properties
 */
export interface PBRMaterial {
  /** Material name */
  name: string;

  /** Base color (if no texture) */
  base_color?: Color;

  /** Metallic value (0-1) */
  metallic: number;

  /** Roughness value (0-1) */
  roughness: number;

  /** Texture maps */
  textures?: TextureMaps;

  /** Double-sided rendering */
  double_sided?: boolean;

  /** Alpha mode */
  alpha_mode?: 'opaque' | 'mask' | 'blend';

  /** Alpha cutoff (for mask mode) */
  alpha_cutoff?: number;
}

/**
 * Complete 3D garment assets
 */
export interface Garment3DAssets {
  /** Multiple LOD models */
  models: Model3D[];

  /** Materials */
  materials: PBRMaterial[];

  /** Physics properties */
  physics: MaterialPhysics;

  /** Thumbnail image */
  thumbnail?: string;
}

// ============================================================================
// Sustainability Types
// ============================================================================

/**
 * Environmental impact
 */
export interface EnvironmentalImpact {
  /** Carbon footprint in kg CO₂e */
  carbon_footprint: number;

  /** Water usage in liters */
  water_usage: number;

  /** Chemical impact score (0-100, lower is better) */
  chemical_impact: number;

  /** Waste generated in kg */
  waste_generated?: number;

  /** Microplastic release in grams */
  microplastic_release?: number;

  /** Environmental score (0-100) */
  environmental_score: number;
}

/**
 * Social impact
 */
export interface SocialImpact {
  /** Fair labor practices (0-100) */
  fair_labor: number;

  /** Safe working conditions (0-100) */
  safe_conditions: number;

  /** Living wage provided (0-100) */
  living_wage: number;

  /** Community impact (0-100) */
  community_impact: number;

  /** Certifications */
  certifications?: string[];

  /** Social score (0-100) */
  social_score: number;
}

/**
 * Circular fashion metrics
 */
export interface CircularMetrics {
  /** Recyclability (0-1) */
  recyclability: number;

  /** Durability score (0-100) */
  durability: number;

  /** Repairability score (0-100) */
  repairability: number;

  /** Biodegradability (0-1) */
  biodegradability: number;

  /** Circular score (0-100) */
  circular_score: number;

  /** Expected lifespan in years */
  expected_lifespan?: number;

  /** Take-back program available */
  take_back_program?: boolean;
}

/**
 * Complete sustainability assessment
 */
export interface SustainabilityScore {
  /** Total sustainability score (0-100) */
  total_score: number;

  /** Letter grade (A+ to F) */
  grade: 'A+' | 'A' | 'B' | 'C' | 'D' | 'F';

  /** Environmental metrics */
  environmental: EnvironmentalImpact;

  /** Social metrics */
  social: SocialImpact;

  /** Circular metrics */
  circular: CircularMetrics;

  /** Certifications */
  certifications?: string[];

  /** Last updated */
  updated_at: Date | string;
}

// ============================================================================
// Garment Design Types
// ============================================================================

/**
 * Garment silhouette
 */
export type Silhouette =
  | 'fitted'
  | 'relaxed'
  | 'oversized'
  | 'a-line'
  | 'straight'
  | 'flared'
  | 'bodycon'
  | 'shift'
  | 'empire';

/**
 * Neckline types
 */
export type Neckline =
  | 'crew'
  | 'v-neck'
  | 'scoop'
  | 'boat'
  | 'square'
  | 'halter'
  | 'off-shoulder'
  | 'cowl'
  | 'turtleneck'
  | 'strapless';

/**
 * Sleeve types
 */
export type SleeveType =
  | 'sleeveless'
  | 'short'
  | 'elbow'
  | 'three_quarter'
  | 'long'
  | 'cap'
  | 'flutter'
  | 'bell'
  | 'bishop'
  | 'puff'
  | 'raglan';

/**
 * Closure types
 */
export type ClosureType =
  | 'zipper'
  | 'buttons'
  | 'snaps'
  | 'velcro'
  | 'laces'
  | 'ties'
  | 'pullover'
  | 'magnetic';

/**
 * Garment design specification
 */
export interface GarmentDesign {
  /** Overall style */
  style: string;

  /** Silhouette */
  silhouette: Silhouette;

  /** Neckline type */
  neckline?: Neckline;

  /** Sleeve type */
  sleeves?: SleeveType;

  /** Length (for dresses, skirts, shorts, etc.) */
  length?: 'mini' | 'knee' | 'midi' | 'maxi' | 'ankle' | 'floor';

  /** Closure type */
  closure?: ClosureType;

  /** Additional details */
  details?: string[];

  /** Special features */
  features?: string[];
}

// ============================================================================
// Complete Garment Type
// ============================================================================

/**
 * Complete garment specification
 */
export interface Garment {
  /** Unique garment ID */
  id: string;

  /** Brand name */
  brand: string;

  /** Product name */
  name: string;

  /** SKU */
  sku?: string;

  /** Garment category */
  category: GarmentCategory;

  /** Subcategory (dress, shirt, etc.) */
  subcategory: string;

  /** Gender */
  gender: Gender;

  /** Season */
  season: Season;

  /** Year/collection */
  year?: number;

  /** Design specification */
  design: GarmentDesign;

  /** Material composition */
  materials: MaterialComposition[];

  /** Fabric properties */
  fabric?: FabricProperties;

  /** Colors available */
  colors: Color[];

  /** Pattern (if any) */
  pattern?: Pattern;

  /** Available sizes */
  sizes: {
    system: 'WIA' | 'US' | 'EU' | 'UK' | 'JP' | 'CN';
    available: StandardSize[];
    measurements: Record<StandardSize, GarmentMeasurements>;
  };

  /** 3D assets */
  assets_3d?: Garment3DAssets;

  /** Images (URLs) */
  images?: string[];

  /** Pricing */
  pricing: {
    retail_price: number;
    currency: string;
    market?: string;
    sale_price?: number;
  };

  /** Sustainability */
  sustainability?: SustainabilityScore;

  /** Blockchain/NFT data */
  blockchain?: {
    nft_enabled: boolean;
    contract_address?: string;
    token_id?: number;
    material_passport?: string;
  };

  /** Care instructions */
  care_instructions?: {
    washing?: string;
    drying?: string;
    ironing?: string;
    dry_cleaning?: boolean;
    bleaching?: boolean;
  };

  /** Tags/keywords */
  tags?: string[];

  /** Description */
  description?: string;
}

// ============================================================================
// Virtual Try-On Types
// ============================================================================

/**
 * Virtual try-on mode
 */
export type TryOnMode = 'AR_camera' | '3D_avatar' | 'photo_overlay';

/**
 * Virtual try-on request
 */
export interface VirtualTryOnRequest {
  /** Garment ID */
  garment_id: string;

  /** Try-on mode */
  mode: TryOnMode;

  /** Body measurements (for 3D avatar) */
  body_measurements?: BodyMeasurements;

  /** Photo URL (for photo overlay) */
  photo_url?: string;

  /** Render quality */
  quality?: 'low' | 'medium' | 'high' | 'ultra';

  /** Viewing angles (for 3D) */
  angles?: number[];

  /** Background */
  background?: 'transparent' | 'white' | 'custom';
}

/**
 * Virtual try-on response
 */
export interface VirtualTryOnResponse {
  /** Result image/video URL */
  result_url: string;

  /** Fit confidence (0-1) */
  fit_confidence: number;

  /** Size recommendation */
  size_recommendation: SizeRecommendation;

  /** Processing time in ms */
  processing_time_ms: number;

  /** Warnings or notes */
  warnings?: string[];
}

// ============================================================================
// AI & ML Types
// ============================================================================

/**
 * Trend prediction request
 */
export interface TrendPredictionRequest {
  /** Season to predict */
  season: string;

  /** Category */
  category: GarmentCategory;

  /** Region */
  region: 'global' | 'north_america' | 'europe' | 'asia' | 'africa' | 'oceania';

  /** Time horizon in months */
  timeframe_months?: number;

  /** Data sources to use */
  data_sources?: ('social_media' | 'runway' | 'retail' | 'expert')[];
}

/**
 * Single trend prediction
 */
export interface TrendPrediction {
  /** Trend name/description */
  name: string;

  /** Style keywords */
  keywords: string[];

  /** Confidence score (0-1) */
  confidence: number;

  /** Trend strength (0-100) */
  strength: number;

  /** Expected peak month */
  peak_month?: string;

  /** Recommended colors */
  colors?: Color[];

  /** Recommended materials */
  materials?: MaterialType[];

  /** Target demographic */
  demographic?: string[];

  /** Supporting data points */
  data_points?: number;
}

/**
 * Trend prediction response
 */
export interface TrendPredictionResponse {
  /** Season predicted for */
  season: string;

  /** Category */
  category: GarmentCategory;

  /** Region */
  region: string;

  /** Predictions */
  predictions: TrendPrediction[];

  /** Overall confidence (0-1) */
  overall_confidence: number;

  /** Generated at */
  generated_at: Date | string;

  /** Data sources used */
  sources_used: string[];
}

/**
 * Style recommendation request
 */
export interface StyleRecommendationRequest {
  /** User body measurements */
  body_measurements: BodyMeasurements;

  /** Style preferences */
  style_preferences: FashionStyle[];

  /** Occasion */
  occasion?: Occasion;

  /** Budget range */
  budget?: {
    min: number;
    max: number;
    currency: string;
  };

  /** Sustainability minimum score */
  sustainability_min?: number;

  /** Preferred colors */
  preferred_colors?: string[];

  /** Number of recommendations */
  limit?: number;
}

/**
 * Single recommendation
 */
export interface Recommendation {
  /** Garment */
  garment: Garment;

  /** Recommendation score (0-100) */
  score: number;

  /** Confidence (0-1) */
  confidence: number;

  /** Reasons for recommendation */
  reasons: string[];

  /** Outfit suggestions (IDs of complementary items) */
  outfit_suggestions?: string[];
}

/**
 * Style recommendation response
 */
export interface StyleRecommendationResponse {
  /** Recommendations */
  recommendations: Recommendation[];

  /** Personalization score (how well we know user) */
  personalization_score: number;

  /** Alternative styles to explore */
  explore_styles?: FashionStyle[];
}

/**
 * Wardrobe optimization request
 */
export interface WardrobeOptimizationRequest {
  /** Current wardrobe items */
  items: Garment[];

  /** Budget for new items */
  budget?: number;

  /** Occasions to optimize for */
  occasions: Record<Occasion, number>; // percentage allocation

  /** Style preferences */
  style_preferences: FashionStyle[];

  /** Sustainability preference (0-1) */
  sustainability_weight?: number;
}

/**
 * Wardrobe gap
 */
export interface WardrobeGap {
  /** Gap type */
  type: 'missing_category' | 'low_versatility' | 'occasion_gap' | 'color_gap';

  /** Description */
  description: string;

  /** Severity (0-1) */
  severity: number;

  /** Recommended additions */
  recommendations: Recommendation[];
}

/**
 * Wardrobe optimization response
 */
export interface WardrobeOptimizationResponse {
  /** Current wardrobe score (0-100) */
  current_score: number;

  /** Potential score with additions (0-100) */
  potential_score: number;

  /** Identified gaps */
  gaps: WardrobeGap[];

  /** Recommended additions */
  recommended_additions: Recommendation[];

  /** Items to consider removing/donating */
  consider_removing?: Garment[];

  /** Overall utility scores */
  utility_scores: Record<string, number>; // garment_id -> score
}

// ============================================================================
// Blockchain & NFT Types
// ============================================================================

/**
 * Metaverse platform
 */
export type MetaversePlatform =
  | 'decentraland'
  | 'sandbox'
  | 'roblox'
  | 'fortnite'
  | 'minecraft'
  | 'vrchat'
  | 'horizon_worlds';

/**
 * NFT fashion metadata
 */
export interface NFTFashionMetadata {
  /** NFT name */
  name: string;

  /** Description */
  description: string;

  /** Display image */
  image: string;

  /** 3D model animation */
  animation_url?: string;

  /** Attributes */
  attributes: Array<{
    trait_type: string;
    value: string | number;
  }>;

  /** External URL */
  external_url?: string;

  /** WIA-specific data */
  wia_fashion_data: {
    standard: 'WIA-IND-001';
    version: string;
    models_3d: Record<MetaversePlatform, string>;
    sustainability: SustainabilityScore;
    physical_twin?: boolean;
    unlockable_content?: Record<string, string>;
  };
}

/**
 * Supply chain stage
 */
export interface SupplyChainStage {
  /** Stage name */
  stage: string;

  /** Timestamp */
  timestamp: Date | string;

  /** Location */
  location: string;

  /** Certification */
  certification?: string;

  /** Carbon footprint for this stage */
  carbon_footprint?: number;

  /** Water usage for this stage */
  water_usage?: number;

  /** Verified by */
  verified_by?: string;

  /** Additional data */
  [key: string]: unknown;
}

/**
 * Material passport (blockchain-based)
 */
export interface MaterialPassport {
  /** Garment ID */
  garment_id: string;

  /** NFT token ID */
  nft_token?: string;

  /** Supply chain stages */
  stages: SupplyChainStage[];

  /** Total carbon footprint */
  total_carbon_footprint: number;

  /** Total water usage */
  total_water_usage: number;

  /** Sustainability score */
  sustainability_score: number;

  /** Blockchain hash */
  blockchain_hash?: string;
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Items in current page */
  items: T[];

  /** Total number of items */
  total: number;

  /** Current page (0-indexed) */
  page: number;

  /** Page size */
  page_size: number;

  /** Total pages */
  total_pages: number;

  /** Has next page */
  has_next: boolean;

  /** Has previous page */
  has_previous: boolean;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Fashion technology constants
 */
export const FASHION_CONSTANTS = {
  /** Maximum carbon footprint (kg CO₂e/kg) */
  MAX_CARBON: 17.0, // Leather

  /** Maximum water usage (L/kg) */
  MAX_WATER: 125000, // Wool

  /** Average garment lifespan (years) */
  AVG_LIFESPAN: 3.0,

  /** Typical number of wears before disposal */
  TYPICAL_WEARS: 75,

  /** Carbon per wash cycle (kg CO₂e) */
  CARBON_PER_WASH: {
    cold_water_line_dry: 0.15,
    cold_water_tumble_dry: 0.75,
    hot_water_tumble_dry: 1.0,
  },

  /** Water per wash (liters) */
  WATER_PER_WASH: {
    standard: 50,
    high_efficiency: 25,
  },

  /** Sustainability score thresholds */
  SUSTAINABILITY_GRADES: {
    'A+': 90,
    A: 80,
    B: 70,
    C: 60,
    D: 50,
    F: 0,
  },

  /** Size confidence thresholds */
  CONFIDENCE_THRESHOLDS: {
    high: 0.85,
    medium: 0.7,
    low: 0.5,
  },

  /** Trend prediction accuracy by timeframe */
  TREND_ACCURACY: {
    '1_month': 0.9,
    '3_months': 0.82,
    '6_months': 0.78,
    '12_months': 0.65,
  },
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-IND-001 error codes
 */
export enum FashionErrorCode {
  INVALID_MEASUREMENTS = 'FASHION001',
  GARMENT_NOT_FOUND = 'FASHION002',
  SIZE_NOT_AVAILABLE = 'FASHION003',
  INVALID_MATERIAL = 'FASHION004',
  MODEL_GENERATION_FAILED = 'FASHION005',
  TRYON_FAILED = 'FASHION006',
  SUSTAINABILITY_CALC_FAILED = 'FASHION007',
  TREND_PREDICTION_FAILED = 'FASHION008',
  INSUFFICIENT_DATA = 'FASHION009',
  BLOCKCHAIN_ERROR = 'FASHION010',
  AUTHENTICATION_FAILED = 'FASHION011',
}

/**
 * Fashion technology error
 */
export class FashionTechError extends Error {
  constructor(
    public code: FashionErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'FashionTechError';
  }
}

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  GarmentCategory,
  Gender,
  Season,
  FashionStyle,
  Occasion,

  // Material types
  MaterialType,
  FabricConstruction,
  MaterialComposition,
  FabricProperties,
  MaterialPhysics,

  // Color & pattern
  Color,
  PatternType,
  Pattern,

  // Size & measurements
  StandardSize,
  SizeModifier,
  BodyType,
  BodyMeasurements,
  GarmentMeasurements,
  SizeRecommendation,

  // 3D model types
  ModelFormat,
  LOD,
  Model3D,
  TextureMaps,
  PBRMaterial,
  Garment3DAssets,

  // Sustainability
  EnvironmentalImpact,
  SocialImpact,
  CircularMetrics,
  SustainabilityScore,

  // Garment design
  Silhouette,
  Neckline,
  SleeveType,
  ClosureType,
  GarmentDesign,

  // Complete garment
  Garment,

  // Virtual try-on
  TryOnMode,
  VirtualTryOnRequest,
  VirtualTryOnResponse,

  // AI & ML
  TrendPredictionRequest,
  TrendPrediction,
  TrendPredictionResponse,
  StyleRecommendationRequest,
  Recommendation,
  StyleRecommendationResponse,
  WardrobeOptimizationRequest,
  WardrobeGap,
  WardrobeOptimizationResponse,

  // Blockchain & NFT
  MetaversePlatform,
  NFTFashionMetadata,
  SupplyChainStage,
  MaterialPassport,

  // Utility types
  PaginatedResponse,
};

export { FASHION_CONSTANTS, FashionErrorCode, FashionTechError };
