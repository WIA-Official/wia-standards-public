/**
 * WIA-MED-030: Nutrition Tracking Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Type Aliases
// ============================================================================

export type Timestamp = string;
export type UserID = string;
export type MealID = string;
export type FoodID = string;

// ============================================================================
// Enums
// ============================================================================

export enum MealType {
  BREAKFAST = 'breakfast',
  LUNCH = 'lunch',
  DINNER = 'dinner',
  SNACK = 'snack',
  BEVERAGE = 'beverage',
}

export enum NutrientUnit {
  GRAMS = 'g',
  MILLIGRAMS = 'mg',
  MICROGRAMS = 'mcg',
  CALORIES = 'kcal',
  MILLILITERS = 'ml',
}

export enum DietaryGoal {
  WEIGHT_LOSS = 'weight_loss',
  MUSCLE_GAIN = 'muscle_gain',
  MAINTENANCE = 'maintenance',
  MEDICAL = 'medical',
  ATHLETIC = 'athletic',
}

export enum AllergenType {
  GLUTEN = 'gluten',
  DAIRY = 'dairy',
  NUTS = 'nuts',
  SOY = 'soy',
  EGGS = 'eggs',
  SHELLFISH = 'shellfish',
  FISH = 'fish',
}

// ============================================================================
// Food Types
// ============================================================================

export interface FoodItem {
  foodId: FoodID;
  name: string;
  brand?: string;
  servingSize: number;
  servingUnit: string;
  nutrients: NutrientProfile;
  allergens: AllergenType[];
  barcode?: string;
  verified: boolean;
}

export interface NutrientProfile {
  calories: number;
  protein: number;
  carbohydrates: number;
  fat: number;
  fiber?: number;
  sugar?: number;
  sodium?: number;
  saturatedFat?: number;
  cholesterol?: number;
  potassium?: number;
  vitaminA?: number;
  vitaminC?: number;
  calcium?: number;
  iron?: number;
}

// ============================================================================
// Meal Types
// ============================================================================

export interface Meal {
  mealId: MealID;
  userId: UserID;
  type: MealType;
  timestamp: Timestamp;
  foods: MealFood[];
  totalNutrients: NutrientProfile;
  notes?: string;
  photoUrl?: string;
  location?: string;
}

export interface MealFood {
  foodId: FoodID;
  quantity: number;
  unit: string;
  nutrients: NutrientProfile;
}

// ============================================================================
// Goals and Tracking Types
// ============================================================================

export interface DailyGoals {
  userId: UserID;
  goalType: DietaryGoal;
  calories: number;
  protein: number;
  carbohydrates: number;
  fat: number;
  fiber?: number;
  water?: number;
}

export interface DailyProgress {
  userId: UserID;
  date: Timestamp;
  consumed: NutrientProfile;
  goals: DailyGoals;
  remainingCalories: number;
  waterIntakeMl: number;
  mealsLogged: number;
}

export interface NutritionInsight {
  insightId: string;
  userId: UserID;
  type: 'achievement' | 'warning' | 'suggestion';
  title: string;
  message: string;
  metric: string;
  value: number;
  createdAt: Timestamp;
}

export interface WeeklyReport {
  userId: UserID;
  weekStart: Timestamp;
  weekEnd: Timestamp;
  averageCalories: number;
  averageProtein: number;
  averageCarbs: number;
  averageFat: number;
  daysLogged: number;
  goalsMet: number;
  insights: NutritionInsight[];
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: Timestamp;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
