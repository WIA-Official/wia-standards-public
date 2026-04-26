/**
 * WIA-MED-030: Nutrition Tracking Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

import EventEmitter from 'eventemitter3';
import {
  WIAConfig,
  APIResponse,
  PaginatedResponse,
  FoodItem,
  Meal,
  MealFood,
  NutrientProfile,
  DailyGoals,
  DailyProgress,
  NutritionInsight,
  WeeklyReport,
  MealType,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WIANutritionConfig extends WIAConfig {
  userId?: string;
}

// ============================================================================
// Main SDK Client
// ============================================================================

export class WIANutritionClient {
  private config: Required<WIANutritionConfig>;
  private eventEmitter = new EventEmitter();

  constructor(config: WIANutritionConfig) {
    this.config = {
      endpoint: 'https://api.wia-standards.org/v1/nutrition',
      timeout: 30000,
      debug: false,
      userId: '',
      ...config,
    };

    if (!this.config.apiKey) {
      throw new Error('API key is required');
    }
  }

  // ==========================================================================
  // Food Operations
  // ==========================================================================

  async searchFoods(query: string): Promise<PaginatedResponse<FoodItem>> {
    return this.makeRequest('GET', `/foods/search?q=${encodeURIComponent(query)}`);
  }

  async getFood(foodId: string): Promise<APIResponse<FoodItem>> {
    return this.makeRequest('GET', `/foods/${foodId}`);
  }

  async getFoodByBarcode(barcode: string): Promise<APIResponse<FoodItem>> {
    return this.makeRequest('GET', `/foods/barcode/${barcode}`);
  }

  async createCustomFood(food: Omit<FoodItem, 'foodId'>): Promise<APIResponse<FoodItem>> {
    return this.makeRequest('POST', '/foods', food);
  }

  // ==========================================================================
  // Meal Operations
  // ==========================================================================

  async logMeal(meal: Omit<Meal, 'mealId' | 'timestamp' | 'totalNutrients'>): Promise<APIResponse<Meal>> {
    return this.makeRequest('POST', '/meals', meal);
  }

  async getMeal(mealId: string): Promise<APIResponse<Meal>> {
    return this.makeRequest('GET', `/meals/${mealId}`);
  }

  async updateMeal(mealId: string, updates: Partial<Meal>): Promise<APIResponse<Meal>> {
    return this.makeRequest('PATCH', `/meals/${mealId}`, updates);
  }

  async deleteMeal(mealId: string): Promise<APIResponse<void>> {
    return this.makeRequest('DELETE', `/meals/${mealId}`);
  }

  async getMealHistory(userId: string, filters?: {
    startDate?: string;
    endDate?: string;
    mealType?: MealType;
  }): Promise<PaginatedResponse<Meal>> {
    const params = new URLSearchParams({ userId, ...filters as Record<string, string> });
    return this.makeRequest('GET', `/meals?${params}`);
  }

  async addFoodToMeal(mealId: string, food: MealFood): Promise<APIResponse<Meal>> {
    return this.makeRequest('POST', `/meals/${mealId}/foods`, food);
  }

  async removeFoodFromMeal(mealId: string, foodId: string): Promise<APIResponse<Meal>> {
    return this.makeRequest('DELETE', `/meals/${mealId}/foods/${foodId}`);
  }

  // ==========================================================================
  // Daily Tracking
  // ==========================================================================

  async getDailyIntake(userId: string, date: string): Promise<APIResponse<NutrientProfile>> {
    return this.makeRequest('GET', `/users/${userId}/intake/${date}`);
  }

  async getDailyProgress(userId: string, date: string): Promise<APIResponse<DailyProgress>> {
    return this.makeRequest('GET', `/users/${userId}/progress/${date}`);
  }

  async logWater(userId: string, amountMl: number): Promise<APIResponse<{ totalMl: number }>> {
    return this.makeRequest('POST', `/users/${userId}/water`, { amountMl });
  }

  // ==========================================================================
  // Goals
  // ==========================================================================

  async getGoals(userId: string): Promise<APIResponse<DailyGoals>> {
    return this.makeRequest('GET', `/users/${userId}/goals`);
  }

  async setGoals(userId: string, goals: Omit<DailyGoals, 'userId'>): Promise<APIResponse<DailyGoals>> {
    return this.makeRequest('PUT', `/users/${userId}/goals`, goals);
  }

  async updateGoals(userId: string, updates: Partial<DailyGoals>): Promise<APIResponse<DailyGoals>> {
    return this.makeRequest('PATCH', `/users/${userId}/goals`, updates);
  }

  // ==========================================================================
  // Insights & Reports
  // ==========================================================================

  async getInsights(userId: string): Promise<PaginatedResponse<NutritionInsight>> {
    return this.makeRequest('GET', `/users/${userId}/insights`);
  }

  async getWeeklyReport(userId: string, weekStart: string): Promise<APIResponse<WeeklyReport>> {
    return this.makeRequest('GET', `/users/${userId}/reports/weekly?weekStart=${weekStart}`);
  }

  async getNutrientTrends(userId: string, nutrient: string, days: number): Promise<APIResponse<{
    dates: string[];
    values: number[];
    average: number;
    goal: number;
  }>> {
    return this.makeRequest('GET', `/users/${userId}/trends/${nutrient}?days=${days}`);
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  on(event: 'mealLogged' | 'goalMet' | 'insightGenerated' | 'error', callback: (...args: unknown[]) => void): void {
    this.eventEmitter.on(event, callback);
  }

  // ==========================================================================
  // Private Methods
  // ==========================================================================

  private async makeRequest<T>(method: string, path: string, body?: unknown): Promise<T> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA Nutrition] ${method} ${url}`, body);
    }

    try {
      const response = await fetch(url, {
        method,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.config.apiKey}`,
          'X-WIA-Standard': 'MED-030',
          'X-WIA-Version': '1.0.0',
          ...(this.config.userId && { 'X-User-ID': this.config.userId }),
        },
        body: body ? JSON.stringify(body) : undefined,
      });

      return await response.json();
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Unknown error';
      this.eventEmitter.emit('error', { code: 'REQUEST_FAILED', message });
      throw error;
    }
  }
}

export function createClient(config: WIANutritionConfig): WIANutritionClient {
  return new WIANutritionClient(config);
}

export default WIANutritionClient;
