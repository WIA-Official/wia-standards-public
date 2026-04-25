/**
 * WIA-IND-008: Smart Kitchen Standard - Type Definitions
 * @module wia-ind-008
 */

export enum ApplianceType {
  Oven = 'oven', Refrigerator = 'refrigerator', Dishwasher = 'dishwasher',
  CoffeeMaker = 'coffee_maker', Microwave = 'microwave', Blender = 'blender',
  AirFryer = 'air_fryer', InstantPot = 'instant_pot', SmartScale = 'smart_scale',
  WaterPurifier = 'water_purifier', RiceCooker = 'rice_cooker', SousVide = 'sous_vide'
}

export enum CookingMethod {
  Bake = 'bake', Roast = 'roast', Broil = 'broil', Grill = 'grill',
  Steam = 'steam', Boil = 'boil', Fry = 'fry', AirFry = 'air_fry',
  SlowCook = 'slow_cook', PressureCook = 'pressure_cook', SousVide = 'sous_vide'
}

export enum TemperatureUnit { Celsius = 'celsius', Fahrenheit = 'fahrenheit' }

export interface ApplianceSpec {
  standard: 'WIA-IND-008';
  version: string;
  applianceId: string;
  type: ApplianceType;
  brand: string;
  model: string;
  connectivity: ('wifi' | 'bluetooth' | 'zigbee')[];
  voiceAssistants: ('alexa' | 'google' | 'siri')[];
  sensors: string[];
  capacity?: { value: number; unit: string };
  powerRating: number;
  energyClass?: string;
}

export interface CookingProgram {
  id: string;
  name: string;
  method: CookingMethod;
  temperature: number;
  temperatureUnit: TemperatureUnit;
  duration: number;
  steps: CookingStep[];
  preheating: boolean;
  notifications: boolean;
}

export interface CookingStep {
  order: number;
  action: string;
  duration: number;
  temperature?: number;
  settings?: Record<string, unknown>;
}

export interface Recipe {
  id: string;
  name: string;
  cuisine: string;
  servings: number;
  prepTime: number;
  cookTime: number;
  ingredients: Ingredient[];
  steps: string[];
  nutritionInfo?: NutritionInfo;
  appliances: ApplianceType[];
  difficulty: 'easy' | 'medium' | 'hard';
}

export interface Ingredient {
  name: string;
  amount: number;
  unit: string;
  optional?: boolean;
  alternatives?: string[];
}

export interface NutritionInfo {
  calories: number;
  protein: number;
  carbohydrates: number;
  fat: number;
  fiber: number;
  sodium: number;
  servingSize: string;
}

export interface InventoryItem {
  id: string;
  name: string;
  quantity: number;
  unit: string;
  expirationDate?: string;
  location: 'refrigerator' | 'freezer' | 'pantry';
  category: string;
  lowThreshold?: number;
}

export interface ApplianceStatus {
  applianceId: string;
  online: boolean;
  currentState: 'idle' | 'running' | 'paused' | 'complete' | 'error';
  currentProgram?: string;
  temperature?: number;
  progress?: number;
  remainingTime?: number;
  errorCode?: string;
}

export interface CookingSession {
  sessionId: string;
  applianceId: string;
  recipe?: string;
  program: CookingProgram;
  startTime: number;
  endTime?: number;
  status: 'in_progress' | 'completed' | 'cancelled' | 'failed';
}

export interface MealPlan {
  id: string;
  date: string;
  meals: { type: 'breakfast' | 'lunch' | 'dinner' | 'snack'; recipe: Recipe }[];
  groceryList?: Ingredient[];
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-IND-008';
  testDate: string;
  applianceId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type SmartKitchenEventType = 'status-update' | 'cooking-complete' | 'inventory-low' | 'timer-alert' | 'error';
export type EventCallback<T = unknown> = (data: T) => void;
