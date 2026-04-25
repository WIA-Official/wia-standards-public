/**
 * WIA-IND-010: Personalized Nutrition Standard - Type Definitions
 * @module wia-ind-010
 */

export enum DietaryGoal {
  WeightLoss = 'weight_loss', WeightGain = 'weight_gain', Maintenance = 'maintenance',
  MuscleBuilding = 'muscle_building', HeartHealth = 'heart_health',
  DiabetesManagement = 'diabetes_management', GutHealth = 'gut_health'
}

export enum DietaryRestriction {
  Vegan = 'vegan', Vegetarian = 'vegetarian', Pescatarian = 'pescatarian',
  Keto = 'keto', Paleo = 'paleo', GlutenFree = 'gluten_free',
  DairyFree = 'dairy_free', NutFree = 'nut_free', Halal = 'halal', Kosher = 'kosher'
}

export enum ActivityLevel {
  Sedentary = 'sedentary', LightlyActive = 'lightly_active',
  ModeratelyActive = 'moderately_active', VeryActive = 'very_active', ExtremelyActive = 'extremely_active'
}

export interface UserProfile {
  id: string;
  age: number;
  gender: 'male' | 'female' | 'other';
  height: number;
  weight: number;
  activityLevel: ActivityLevel;
  goals: DietaryGoal[];
  restrictions: DietaryRestriction[];
  allergies: string[];
  medicalConditions?: string[];
  preferences: { likes: string[]; dislikes: string[] };
}

export interface NutrientTarget {
  nutrient: string;
  target: number;
  unit: string;
  min?: number;
  max?: number;
}

export interface NutritionPlan {
  id: string;
  userId: string;
  dailyCalories: number;
  macros: { protein: number; carbs: number; fat: number };
  micronutrients: NutrientTarget[];
  mealDistribution: { meal: string; caloriePercentage: number }[];
  startDate: string;
  endDate: string;
  recommendations: string[];
}

export interface Food {
  id: string;
  name: string;
  brand?: string;
  servingSize: number;
  servingUnit: string;
  calories: number;
  protein: number;
  carbs: number;
  fat: number;
  fiber: number;
  sugar: number;
  sodium: number;
  micronutrients: { name: string; amount: number; unit: string }[];
  allergens: string[];
  category: string;
}

export interface Meal {
  id: string;
  name: string;
  foods: { food: Food; servings: number }[];
  totalCalories: number;
  totalProtein: number;
  totalCarbs: number;
  totalFat: number;
  mealType: 'breakfast' | 'lunch' | 'dinner' | 'snack';
  timestamp: number;
}

export interface DailyLog {
  date: string;
  userId: string;
  meals: Meal[];
  waterIntake: number;
  exercise: { type: string; duration: number; caloriesBurned: number }[];
  notes?: string;
  mood?: 'great' | 'good' | 'okay' | 'poor';
}

export interface ProgressReport {
  userId: string;
  period: { start: string; end: string };
  weightChange: number;
  averageCalories: number;
  macroAdherence: { protein: number; carbs: number; fat: number };
  topFoods: string[];
  recommendations: string[];
  goals: { goal: string; progress: number }[];
}

export interface GeneticInsight {
  gene: string;
  variant: string;
  impact: string;
  nutritionRecommendation: string;
  foodsToFavor: string[];
  foodsToAvoid: string[];
}

export enum CertificationLevel { Bronze = 'bronze', Silver = 'silver', Gold = 'gold' }

export interface ComplianceReport {
  standard: 'WIA-IND-010';
  testDate: string;
  platformId: string;
  certificationLevel: CertificationLevel;
  tests: { name: string; passed: boolean }[];
  compliant: boolean;
}

export type NutritionEventType = 'meal-logged' | 'goal-achieved' | 'plan-updated' | 'reminder' | 'insight';
export type EventCallback<T = unknown> = (data: T) => void;
