/**
 * WIA-IND-008: Smart Kitchen Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry 4.0 Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Appliance Types
// ============================================================================

/**
 * Appliance type classification
 */
export type ApplianceType =
  | 'oven'
  | 'cooktop'
  | 'induction-cooktop'
  | 'gas-cooktop'
  | 'refrigerator'
  | 'freezer'
  | 'dishwasher'
  | 'microwave'
  | 'rice-cooker'
  | 'slow-cooker'
  | 'pressure-cooker'
  | 'air-fryer'
  | 'coffee-maker'
  | 'blender'
  | 'food-processor'
  | 'toaster'
  | 'range-hood'
  | 'water-purifier'
  | 'wine-cooler'
  | 'kimchi-refrigerator';

/**
 * Appliance status
 */
export type ApplianceStatus =
  | 'off'
  | 'standby'
  | 'preheating'
  | 'cooking'
  | 'cooling'
  | 'paused'
  | 'error'
  | 'maintenance';

/**
 * Energy efficiency class
 */
export type EnergyClass = 'A+++' | 'A++' | 'A+' | 'A' | 'B' | 'C' | 'D' | 'E' | 'F' | 'G';

/**
 * Connectivity type
 */
export type ConnectivityType = 'wifi' | 'bluetooth' | 'zigbee' | 'z-wave' | 'thread' | 'matter';

/**
 * Base appliance configuration
 */
export interface ApplianceConfiguration {
  /** Unique appliance identifier */
  appliance_id: string;

  /** Type of appliance */
  type: ApplianceType;

  /** Manufacturer name */
  manufacturer: string;

  /** Model number */
  model: string;

  /** Firmware version */
  firmware_version: string;

  /** Power rating in watts */
  power_rating_w: number;

  /** Voltage requirement */
  voltage: number;

  /** Energy efficiency class */
  energy_class: EnergyClass;

  /** Connectivity options */
  connectivity: ConnectivityType[];

  /** Installation date */
  installation_date: string;

  /** Warranty expiration */
  warranty_expires: string;

  /** Current status */
  status: ApplianceStatus;
}

// ============================================================================
// Oven Types
// ============================================================================

/**
 * Oven cooking mode
 */
export type OvenMode =
  | 'conventional'
  | 'convection'
  | 'fan-assisted'
  | 'top-heat'
  | 'bottom-heat'
  | 'grill'
  | 'steam'
  | 'steam-assist'
  | 'air-fry'
  | 'dehydrate'
  | 'proof'
  | 'self-clean'
  | 'pyrolytic-clean';

/**
 * Oven configuration
 */
export interface OvenConfiguration extends ApplianceConfiguration {
  type: 'oven';

  /** Internal capacity in liters */
  capacity_liters: number;

  /** Temperature range */
  temperature_range_celsius: {
    min: number;
    max: number;
  };

  /** Temperature accuracy in ±°C */
  temperature_accuracy: number;

  /** Available cooking modes */
  modes: OvenMode[];

  /** Heating elements */
  heating_elements: {
    upper?: number;
    lower?: number;
    convection_fan?: number;
    grill?: number;
  };

  /** Number of racks */
  rack_positions: number;

  /** Has internal light */
  has_light: boolean;

  /** Has temperature probe */
  has_probe: boolean;

  /** Self-cleaning capability */
  self_cleaning: boolean;

  /** Preheat times in seconds */
  preheat_times: {
    to_180C: number;
    to_220C: number;
    to_260C: number;
  };
}

/**
 * Oven state
 */
export interface OvenState {
  appliance_id: string;
  status: ApplianceStatus;
  mode: OvenMode | null;
  current_temperature: number;
  target_temperature: number;
  remaining_time_seconds: number;
  door_open: boolean;
  light_on: boolean;
  probe_temperature?: number;
  power_consumption_w: number;
  errors: string[];
}

// ============================================================================
// Cooktop Types
// ============================================================================

/**
 * Cooktop type
 */
export type CooktopType = 'induction' | 'gas' | 'electric-coil' | 'ceramic-glass' | 'hybrid';

/**
 * Heat level (1-17 for induction, typically 1-9 for others)
 */
export type HeatLevel = number;

/**
 * Cooking zone configuration
 */
export interface CookingZone {
  /** Zone identifier */
  zone_id: string;

  /** Zone diameter in cm */
  diameter_cm: number;

  /** Normal power rating in watts */
  power_w: number;

  /** Boost power rating in watts */
  boost_w?: number;

  /** Current heat level */
  current_level: HeatLevel;

  /** Maximum heat level */
  max_level: HeatLevel;

  /** Zone is active */
  active: boolean;

  /** Pan detected (for induction) */
  pan_detected?: boolean;

  /** Current temperature estimate */
  temperature_estimate?: number;

  /** Remaining time if timer set */
  timer_seconds?: number;
}

/**
 * Cooktop configuration
 */
export interface CooktopConfiguration extends ApplianceConfiguration {
  type: 'cooktop' | 'induction-cooktop' | 'gas-cooktop';

  /** Cooktop technology type */
  cooktop_type: CooktopType;

  /** Number of cooking zones */
  num_zones: number;

  /** Zone configurations */
  zones: CookingZone[];

  /** Total power rating */
  total_power_w: number;

  /** Efficiency factor */
  efficiency: number;

  /** Features */
  features: {
    pan_detection?: boolean;
    power_boost?: boolean;
    keep_warm?: boolean;
    power_slide?: boolean;
    bridge_zone?: boolean;
    child_lock?: boolean;
    overflow_detection?: boolean;
    auto_shutoff?: boolean;
  };

  /** Response time in seconds */
  response_time_seconds: number;
}

/**
 * Cooktop state
 */
export interface CooktopState {
  appliance_id: string;
  status: ApplianceStatus;
  zones: CookingZone[];
  child_lock: boolean;
  total_power_consumption_w: number;
  errors: string[];
}

// ============================================================================
// Refrigerator Types
// ============================================================================

/**
 * Refrigerator zone type
 */
export type RefrigeratorZoneType =
  | 'refrigerator-main'
  | 'freezer'
  | 'flex-zone'
  | 'crisper'
  | 'dairy'
  | 'door-bins'
  | 'wine-zone'
  | 'kimchi-zone';

/**
 * Refrigerator zone configuration
 */
export interface RefrigeratorZone {
  /** Zone identifier */
  zone_id: string;

  /** Zone name */
  name: string;

  /** Zone type */
  type: RefrigeratorZoneType;

  /** Capacity in liters */
  capacity_liters: number;

  /** Temperature range */
  temp_range_celsius: {
    min: number;
    max: number;
  };

  /** Current temperature */
  current_temp: number;

  /** Target temperature */
  target_temp: number;

  /** Has humidity control */
  humidity_control: boolean;

  /** Current humidity percentage */
  humidity_percent?: number;

  /** Is convertible (fridge ↔ freezer) */
  convertible?: boolean;

  /** Door open status */
  door_open: boolean;
}

/**
 * Refrigerator configuration
 */
export interface RefrigeratorConfiguration extends ApplianceConfiguration {
  type: 'refrigerator';

  /** Total capacity in liters */
  total_capacity_liters: number;

  /** Annual energy consumption in kWh */
  annual_consumption_kwh: number;

  /** Climate class */
  climate_class: 'SN' | 'N' | 'ST' | 'T';

  /** Zones configuration */
  zones: RefrigeratorZone[];

  /** Smart features */
  smart_features: {
    internal_cameras?: boolean;
    inventory_tracking?: boolean;
    expiry_monitoring?: boolean;
    door_open_alerts?: boolean;
    temperature_alerts?: boolean;
    recipe_suggestions?: boolean;
    shopping_list?: boolean;
  };

  /** Number of internal cameras */
  num_cameras?: number;

  /** Number of door sensors */
  num_door_sensors: number;

  /** Has water dispenser */
  water_dispenser?: boolean;

  /** Has ice maker */
  ice_maker?: boolean;
}

/**
 * Refrigerator state
 */
export interface RefrigeratorState {
  appliance_id: string;
  status: ApplianceStatus;
  zones: RefrigeratorZone[];
  power_consumption_w: number;
  ice_maker_status?: 'idle' | 'making' | 'full' | 'error';
  water_filter_life_percent?: number;
  defrost_cycle?: boolean;
  errors: string[];
}

// ============================================================================
// Recipe Types
// ============================================================================

/**
 * Cuisine type
 */
export type CuisineType =
  | 'korean'
  | 'japanese'
  | 'chinese'
  | 'italian'
  | 'french'
  | 'american'
  | 'mexican'
  | 'indian'
  | 'thai'
  | 'vietnamese'
  | 'mediterranean'
  | 'middle-eastern'
  | 'fusion'
  | 'international';

/**
 * Recipe category
 */
export type RecipeCategory =
  | 'appetizer'
  | 'soup_stew'
  | 'salad'
  | 'main_course'
  | 'side_dish'
  | 'dessert'
  | 'beverage'
  | 'breakfast'
  | 'brunch'
  | 'snack'
  | 'sauce_condiment';

/**
 * Difficulty level
 */
export type DifficultyLevel = 'easy' | 'medium' | 'hard' | 'expert';

/**
 * Cooking technique
 */
export type CookingTechnique =
  | 'boiling'
  | 'simmering'
  | 'steaming'
  | 'poaching'
  | 'blanching'
  | 'sauteing'
  | 'stir-frying'
  | 'pan-frying'
  | 'deep-frying'
  | 'grilling'
  | 'broiling'
  | 'roasting'
  | 'baking'
  | 'braising'
  | 'stewing'
  | 'slow-cooking'
  | 'pressure-cooking'
  | 'air-frying'
  | 'sous-vide'
  | 'smoking'
  | 'fermentation'
  | 'cutting'
  | 'mixing'
  | 'marinating';

/**
 * Heat level
 */
export type HeatLevelName = 'low' | 'medium-low' | 'medium' | 'medium-high' | 'high';

/**
 * Recipe ingredient
 */
export interface Ingredient {
  /** Ingredient ID */
  id: string;

  /** Ingredient name */
  name: string;

  /** Amount/quantity */
  amount: number;

  /** Unit of measurement */
  unit: string;

  /** Preparation notes */
  notes?: string;

  /** Optional ingredient */
  optional: boolean;

  /** Possible substitutes */
  substitutes?: string[];

  /** Allergen information */
  allergens?: string[];
}

/**
 * Cooking step
 */
export interface CookingStep {
  /** Step number */
  step_number: number;

  /** Instruction text */
  instruction: string;

  /** Duration in minutes */
  duration_minutes: number;

  /** Required appliances */
  appliances: ApplianceType[];

  /** Target temperature (if applicable) */
  temperature?: number;

  /** Cooking technique */
  technique: CookingTechnique;

  /** Heat level (if using cooktop) */
  heat_level?: HeatLevelName;

  /** Visual aid (image/video URL) */
  media_url?: string;

  /** Timer should alert */
  alert?: boolean;
}

/**
 * Nutritional information
 */
export interface NutritionInfo {
  /** Calories (kcal) */
  calories: number;

  /** Protein (grams) */
  protein_g: number;

  /** Carbohydrates (grams) */
  carbs_g: number;

  /** Fat (grams) */
  fat_g: number;

  /** Fiber (grams) */
  fiber_g: number;

  /** Sugar (grams) */
  sugar_g: number;

  /** Sodium (mg) */
  sodium_mg: number;

  /** Cholesterol (mg) */
  cholesterol_mg?: number;

  /** Saturated fat (grams) */
  saturated_fat_g?: number;

  /** Trans fat (grams) */
  trans_fat_g?: number;

  /** Vitamins */
  vitamins?: VitaminProfile;

  /** Minerals */
  minerals?: MineralProfile;
}

/**
 * Vitamin profile
 */
export interface VitaminProfile {
  vitamin_a_iu?: number;
  vitamin_c_mg?: number;
  vitamin_d_iu?: number;
  vitamin_e_mg?: number;
  vitamin_k_mcg?: number;
  vitamin_b1_mg?: number;
  vitamin_b2_mg?: number;
  vitamin_b3_mg?: number;
  vitamin_b6_mg?: number;
  vitamin_b12_mcg?: number;
  folate_mcg?: number;
}

/**
 * Mineral profile
 */
export interface MineralProfile {
  calcium_mg?: number;
  iron_mg?: number;
  magnesium_mg?: number;
  phosphorus_mg?: number;
  potassium_mg?: number;
  zinc_mg?: number;
  copper_mg?: number;
  manganese_mg?: number;
  selenium_mcg?: number;
}

/**
 * Multilingual text
 */
export interface MultilingualText {
  en: string;
  ko?: string;
  ja?: string;
  zh?: string;
  es?: string;
  fr?: string;
  de?: string;
}

/**
 * Recipe
 */
export interface Recipe {
  /** Unique recipe identifier */
  recipe_id: string;

  /** Recipe name (multilingual) */
  name: MultilingualText;

  /** Cuisine type */
  cuisine: CuisineType;

  /** Recipe category */
  category: RecipeCategory;

  /** Difficulty level */
  difficulty: DifficultyLevel;

  /** Number of servings */
  servings: number;

  /** Preparation time (minutes) */
  prep_time_minutes: number;

  /** Cooking time (minutes) */
  cook_time_minutes: number;

  /** Total time (minutes) */
  total_time_minutes: number;

  /** Ingredients list */
  ingredients: Ingredient[];

  /** Cooking steps */
  steps: CookingStep[];

  /** Nutritional information (per serving) */
  nutrition: {
    per_serving: NutritionInfo;
    per_100g?: NutritionInfo;
  };

  /** Required appliances */
  appliances_required: ApplianceType[];

  /** Tags for categorization */
  tags: string[];

  /** Allergens present */
  allergens: string[];

  /** Dietary flags */
  dietary_flags: string[];

  /** Author/source */
  author?: string;

  /** Creation date */
  created_at: string;

  /** Last updated */
  updated_at: string;

  /** User rating (0-5) */
  rating?: number;

  /** Number of times cooked */
  times_cooked?: number;

  /** Recipe image URL */
  image_url?: string;

  /** Recipe video URL */
  video_url?: string;
}

// ============================================================================
// Inventory Types
// ============================================================================

/**
 * Food category
 */
export type FoodCategory =
  | 'dairy'
  | 'meat'
  | 'poultry'
  | 'fish_seafood'
  | 'eggs'
  | 'vegetables'
  | 'fruits'
  | 'grains'
  | 'legumes'
  | 'nuts_seeds'
  | 'oils_fats'
  | 'condiments'
  | 'spices'
  | 'beverages'
  | 'frozen'
  | 'canned'
  | 'baking'
  | 'snacks'
  | 'prepared_foods'
  | 'other';

/**
 * Storage location
 */
export type StorageLocation =
  | 'refrigerator'
  | 'freezer'
  | 'pantry'
  | 'counter'
  | 'wine-cooler'
  | 'kimchi-refrigerator';

/**
 * Freshness category
 */
export type FreshnessCategory = 'fresh' | 'good' | 'fair' | 'use-soon' | 'expired';

/**
 * Inventory item
 */
export interface InventoryItem {
  /** Unique item identifier */
  item_id: string;

  /** Item name */
  name: string;

  /** Food category */
  category: FoodCategory;

  /** Quantity */
  quantity: number;

  /** Unit of measurement */
  unit: string;

  /** Purchase date */
  purchase_date: string;

  /** Expiration date */
  expiry_date: string;

  /** Days until expiry */
  days_until_expiry: number;

  /** Freshness category */
  freshness: FreshnessCategory;

  /** Freshness index (0-100) */
  freshness_index: number;

  /** Storage location */
  location: StorageLocation;

  /** Specific zone/shelf */
  zone?: string;

  /** Barcode */
  barcode?: string;

  /** Price */
  price?: number;

  /** Currency */
  currency?: string;

  /** Nutritional info per 100g/100ml */
  nutritional_info?: NutritionInfo;

  /** Storage temperature (°C) */
  storage_temp_celsius?: number;

  /** Has been opened */
  opened: boolean;

  /** Date opened */
  opened_date?: string;

  /** Use within X days after opening */
  use_within_days_after_opening?: number;

  /** Item image (from camera) */
  image_url?: string;
}

/**
 * Shopping list item
 */
export interface ShoppingListItem {
  /** Item name */
  name: string;

  /** Quantity needed */
  quantity: number;

  /** Unit */
  unit: string;

  /** Category for store organization */
  category: FoodCategory;

  /** Priority (1-5, 5 being highest) */
  priority: number;

  /** For specific recipe */
  for_recipe?: string;

  /** Estimated price */
  estimated_price?: number;

  /** Purchased */
  purchased: boolean;
}

/**
 * Shopping list
 */
export interface ShoppingList {
  /** List identifier */
  list_id: string;

  /** List name */
  name: string;

  /** Creation date */
  created_at: string;

  /** Items */
  items: ShoppingListItem[];

  /** Total estimated cost */
  total_estimated_cost?: number;

  /** Currency */
  currency?: string;
}

// ============================================================================
// Energy Management Types
// ============================================================================

/**
 * Energy mode
 */
export type EnergyMode = 'eco' | 'balanced' | 'performance' | 'scheduled';

/**
 * Time of use period
 */
export type TimeOfUsePeriod = 'off-peak' | 'mid-peak' | 'peak';

/**
 * Power status
 */
export interface PowerStatus {
  /** Current power consumption (W) */
  current_power_w: number;

  /** Today's total consumption (kWh) */
  today_kwh: number;

  /** This month's total (kWh) */
  month_kwh: number;

  /** Projected monthly (kWh) */
  projected_monthly_kwh: number;

  /** Active appliances */
  active_appliances: string[];

  /** Current time-of-use period */
  tou_period: TimeOfUsePeriod;

  /** Current energy rate (currency/kWh) */
  current_rate?: number;
}

/**
 * Appliance energy usage
 */
export interface ApplianceEnergyUsage {
  /** Appliance ID */
  appliance_id: string;

  /** Appliance type */
  type: ApplianceType;

  /** Energy consumed (kWh) */
  energy_kwh: number;

  /** Percentage of total */
  percent_of_total: number;

  /** Operating hours */
  operating_hours: number;

  /** Cost (in currency) */
  cost?: number;
}

/**
 * Energy breakdown
 */
export interface EnergyBreakdown {
  /** Period start */
  period_start: string;

  /** Period end */
  period_end: string;

  /** Total energy (kWh) */
  total_kwh: number;

  /** Total cost */
  total_cost?: number;

  /** Currency */
  currency?: string;

  /** Breakdown by appliance */
  by_appliance: ApplianceEnergyUsage[];

  /** Comparison to previous period */
  vs_previous_period?: number; // percentage change
}

/**
 * Energy schedule
 */
export interface EnergySchedule {
  /** Schedule name */
  name: string;

  /** Enabled */
  enabled: boolean;

  /** Time-of-use pricing */
  tou_pricing?: {
    off_peak: { start: string; end: string; rate: number }[];
    mid_peak: { start: string; end: string; rate: number }[];
    peak: { start: string; end: string; rate: number }[];
  };

  /** Preferred cooking times */
  preferred_times: string[];

  /** Avoid times */
  avoid_times: string[];

  /** Maximum concurrent power (W) */
  max_concurrent_power?: number;
}

// ============================================================================
// Meal Planning Types
// ============================================================================

/**
 * Meal type
 */
export type MealType = 'breakfast' | 'lunch' | 'dinner' | 'snack' | 'dessert';

/**
 * Planned meal
 */
export interface PlannedMeal {
  /** Date */
  date: string;

  /** Meal type */
  meal_type: MealType;

  /** Recipe ID */
  recipe_id: string;

  /** Number of servings */
  servings: number;

  /** Scheduled cooking start time */
  scheduled_time?: string;

  /** Estimated prep time */
  estimated_prep_minutes: number;

  /** Estimated cook time */
  estimated_cook_minutes: number;

  /** Required ingredients */
  required_ingredients: Ingredient[];

  /** Missing ingredients */
  missing_ingredients?: Ingredient[];

  /** Completed */
  completed: boolean;
}

/**
 * Weekly meal plan
 */
export interface WeeklyMealPlan {
  /** Plan ID */
  plan_id: string;

  /** Week start date */
  week_start: string;

  /** Week end date */
  week_end: string;

  /** Planned meals */
  meals: PlannedMeal[];

  /** Number of people */
  num_people: number;

  /** Dietary restrictions */
  dietary_restrictions?: string[];

  /** Budget target */
  budget_target?: number;

  /** Total estimated cost */
  total_estimated_cost?: number;

  /** Shopping list ID */
  shopping_list_id?: string;
}

// ============================================================================
// Cooking Automation Types
// ============================================================================

/**
 * Cooking event type
 */
export type CookingEventType =
  | 'started'
  | 'step_started'
  | 'step_completed'
  | 'temperature_reached'
  | 'timer_alert'
  | 'completed'
  | 'paused'
  | 'resumed'
  | 'cancelled'
  | 'error';

/**
 * Cooking event
 */
export interface CookingEvent {
  /** Event type */
  type: CookingEventType;

  /** Timestamp */
  timestamp: string;

  /** Current step number */
  step?: number;

  /** Total steps */
  total_steps?: number;

  /** Message */
  message: string;

  /** Additional data */
  data?: Record<string, any>;
}

/**
 * Cooking session
 */
export interface CookingSession {
  /** Session ID */
  session_id: string;

  /** Recipe being cooked */
  recipe: Recipe;

  /** Start time */
  start_time: string;

  /** End time */
  end_time?: string;

  /** Current step */
  current_step: number;

  /** Session status */
  status: 'in-progress' | 'paused' | 'completed' | 'cancelled' | 'error';

  /** Appliances in use */
  appliances_in_use: string[];

  /** Events log */
  events: CookingEvent[];

  /** Actual energy used */
  energy_used_kwh?: number;

  /** User rating after completion */
  user_rating?: number;

  /** User notes */
  notes?: string;
}

// ============================================================================
// Safety and Monitoring Types
// ============================================================================

/**
 * Alert level
 */
export type AlertLevel = 'info' | 'warning' | 'critical' | 'emergency';

/**
 * Alert type
 */
export type AlertType =
  | 'temperature_exceeded'
  | 'door_open_too_long'
  | 'food_expiring'
  | 'appliance_error'
  | 'fire_risk'
  | 'electrical_fault'
  | 'maintenance_required'
  | 'cooking_complete'
  | 'timer_alert';

/**
 * Kitchen alert
 */
export interface KitchenAlert {
  /** Alert ID */
  alert_id: string;

  /** Alert type */
  type: AlertType;

  /** Alert level */
  level: AlertLevel;

  /** Timestamp */
  timestamp: string;

  /** Message */
  message: string;

  /** Affected appliance */
  appliance_id?: string;

  /** Recommended action */
  recommended_action?: string;

  /** Acknowledged */
  acknowledged: boolean;

  /** Resolved */
  resolved: boolean;
}

/**
 * Safety limits
 */
export interface SafetyLimits {
  /** Maximum cooking time per appliance (minutes) */
  max_cooking_time: {
    oven: number;
    cooktop: number;
    microwave: number;
  };

  /** Maximum temperatures (°C) */
  max_temperatures: {
    oven: number;
    cooktop: number;
    oil_deep_fry: number;
  };

  /** Auto-shutoff enabled */
  auto_shutoff_enabled: boolean;

  /** Child lock enabled */
  child_lock_enabled: boolean;

  /** Allowed appliances when child lock active */
  child_lock_allowed?: ApplianceType[];
}

// ============================================================================
// User and Preferences Types
// ============================================================================

/**
 * Dietary preference
 */
export type DietaryPreference =
  | 'omnivore'
  | 'vegetarian'
  | 'vegan'
  | 'pescatarian'
  | 'keto'
  | 'paleo'
  | 'low-carb'
  | 'low-fat'
  | 'gluten-free'
  | 'dairy-free'
  | 'halal'
  | 'kosher';

/**
 * User profile
 */
export interface UserProfile {
  /** User ID */
  user_id: string;

  /** Name */
  name: string;

  /** Email */
  email: string;

  /** Dietary preferences */
  dietary_preferences: DietaryPreference[];

  /** Allergens to avoid */
  allergens: string[];

  /** Favorite cuisines */
  favorite_cuisines: CuisineType[];

  /** Cooking skill level */
  skill_level: DifficultyLevel;

  /** Daily calorie target */
  calorie_target?: number;

  /** Macro targets */
  macro_targets?: {
    protein_g: number;
    carbs_g: number;
    fat_g: number;
  };

  /** Household size */
  household_size: number;

  /** Timezone */
  timezone: string;

  /** Language preference */
  language: string;

  /** Energy mode preference */
  energy_mode: EnergyMode;
}

/**
 * Kitchen configuration
 */
export interface KitchenConfiguration {
  /** Kitchen ID */
  kitchen_id: string;

  /** Owner user ID */
  owner_id: string;

  /** Kitchen name */
  name: string;

  /** Registered appliances */
  appliances: ApplianceConfiguration[];

  /** Energy schedule */
  energy_schedule?: EnergySchedule;

  /** Safety limits */
  safety_limits: SafetyLimits;

  /** Shared with users */
  shared_users?: string[];

  /** Creation date */
  created_at: string;

  /** Last updated */
  updated_at: string;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Generic API response
 */
export interface ApiResponse<T> {
  /** Success status */
  success: boolean;

  /** Response data */
  data?: T;

  /** Error message */
  error?: string;

  /** Error code */
  error_code?: string;

  /** Timestamp */
  timestamp: string;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Items */
  items: T[];

  /** Total count */
  total: number;

  /** Page number */
  page: number;

  /** Page size */
  page_size: number;

  /** Has more pages */
  has_more: boolean;
}

// ============================================================================
// Calculation and Analysis Types
// ============================================================================

/**
 * Energy calculation result
 */
export interface EnergyCa lculation {
  /** Appliance type */
  appliance: ApplianceType;

  /** Power rating (W) */
  power_w: number;

  /** Duration (hours) */
  duration_hours: number;

  /** Efficiency factor */
  efficiency: number;

  /** Calculated energy (kWh) */
  energy_kwh: number;

  /** Estimated cost */
  cost?: number;

  /** Currency */
  currency?: string;
}

/**
 * Recipe scaling result
 */
export interface RecipeScalingResult {
  /** Original recipe */
  original_recipe: Recipe;

  /** Scaled recipe */
  scaled_recipe: Recipe;

  /** Scaling factor */
  scaling_factor: number;

  /** Adjustments made */
  adjustments: {
    ingredients_scaled: boolean;
    time_adjusted: boolean;
    temperature_adjusted: boolean;
  };
}

/**
 * Cooking timeline
 */
export interface CookingTimeline {
  /** Recipe ID */
  recipe_id: string;

  /** Target completion time */
  target_time: string;

  /** Start time */
  start_time: string;

  /** Timeline steps */
  steps: {
    time: string;
    actions: string[];
    appliances: ApplianceType[];
  }[];

  /** Total duration minutes */
  total_duration_minutes: number;
}

/**
 * Food waste analysis
 */
export interface FoodWasteAnalysis {
  /** Period start */
  period_start: string;

  /** Period end */
  period_end: string;

  /** Items wasted */
  items_wasted: number;

  /** Total value wasted */
  total_value?: number;

  /** Currency */
  currency?: string;

  /** Most wasted categories */
  top_wasted_categories: {
    category: FoodCategory;
    count: number;
    value?: number;
  }[];

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
