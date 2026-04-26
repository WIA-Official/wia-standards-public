/**
 * WIA-AGRI-029 Food Security Standard - TypeScript SDK
 * @module @wia/food-security
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Food Security Client
 */
export class FoodSecurityClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-foodsecurity.io/v1',
      timeout: 30000,
      retryAttempts: 3,
      ...config,
    };

    this.api = axios.create({
      baseURL: this.config.baseURL,
      timeout: this.config.timeout,
      headers: {
        'X-API-Key': this.config.apiKey,
        'Content-Type': 'application/json',
      },
    });
  }

  // ========================================================================
  // Food Security Assessment Methods
  // ========================================================================

  /**
   * Get food security assessment for a region
   */
  async getAssessment(regionId: string): Promise<types.FoodSecurityAssessment> {
    const response = await this.api.get(`/assessments/region/${regionId}`);
    return response.data;
  }

  /**
   * Create new food security assessment
   */
  async createAssessment(
    assessment: Partial<types.FoodSecurityAssessment>
  ): Promise<types.FoodSecurityAssessment> {
    const response = await this.api.post('/assessments', assessment);
    return response.data;
  }

  /**
   * Get assessment history for region
   */
  async getAssessmentHistory(
    regionId: string,
    startDate?: string,
    endDate?: string
  ): Promise<types.FoodSecurityAssessment[]> {
    const response = await this.api.get(`/assessments/region/${regionId}/history`, {
      params: { startDate, endDate },
    });
    return response.data;
  }

  /**
   * Get regions by security level
   */
  async getRegionsBySecurityLevel(
    level: types.SecurityLevel
  ): Promise<types.Region[]> {
    const response = await this.api.get('/regions', {
      params: { securityLevel: level },
    });
    return response.data;
  }

  // ========================================================================
  // Supply Chain Methods
  // ========================================================================

  /**
   * Get supply chain information
   */
  async getSupplyChain(chainId: string): Promise<types.SupplyChain> {
    const response = await this.api.get(`/supply-chains/${chainId}`);
    return response.data;
  }

  /**
   * Get all supply chains
   */
  async getSupplyChains(commodity?: string): Promise<types.SupplyChain[]> {
    const response = await this.api.get('/supply-chains', {
      params: { commodity },
    });
    return response.data;
  }

  /**
   * Track commodity through supply chain
   */
  async trackCommodity(
    commodityId: string,
    batchId: string
  ): Promise<types.SupplyChainStage[]> {
    const response = await this.api.get(`/supply-chains/track/${commodityId}/${batchId}`);
    return response.data;
  }

  /**
   * Update supply chain stage
   */
  async updateSupplyChainStage(
    stageId: string,
    updates: Partial<types.SupplyChainStage>
  ): Promise<types.SupplyChainStage> {
    const response = await this.api.patch(`/supply-chains/stages/${stageId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Food Stock & Inventory Methods
  // ========================================================================

  /**
   * Get food stocks
   */
  async getFoodStocks(
    facilityId?: string,
    commodity?: string
  ): Promise<types.FoodStock[]> {
    const response = await this.api.get('/stocks', {
      params: { facilityId, commodity },
    });
    return response.data;
  }

  /**
   * Add food stock
   */
  async addFoodStock(stock: Partial<types.FoodStock>): Promise<types.FoodStock> {
    const response = await this.api.post('/stocks', stock);
    return response.data;
  }

  /**
   * Update food stock
   */
  async updateFoodStock(
    stockId: string,
    updates: Partial<types.FoodStock>
  ): Promise<types.FoodStock> {
    const response = await this.api.patch(`/stocks/${stockId}`, updates);
    return response.data;
  }

  /**
   * Get storage facilities
   */
  async getStorageFacilities(regionId?: string): Promise<types.StorageFacility[]> {
    const response = await this.api.get('/facilities', {
      params: { regionId },
    });
    return response.data;
  }

  /**
   * Get total stock by commodity
   */
  async getStockSummary(regionId?: string): Promise<Record<string, number>> {
    const response = await this.api.get('/stocks/summary', {
      params: { regionId },
    });
    return response.data;
  }

  // ========================================================================
  // Market & Price Methods
  // ========================================================================

  /**
   * Get current market prices
   */
  async getMarketPrices(
    marketId?: string,
    commodity?: string
  ): Promise<types.MarketPrice[]> {
    const response = await this.api.get('/market-prices', {
      params: { marketId, commodity },
    });
    return response.data;
  }

  /**
   * Get price history
   */
  async getPriceHistory(
    commodity: string,
    marketId: string,
    startDate: string,
    endDate: string
  ): Promise<types.MarketPrice[]> {
    const response = await this.api.get(`/market-prices/history`, {
      params: { commodity, marketId, startDate, endDate },
    });
    return response.data;
  }

  /**
   * Get price analysis
   */
  async analyzePrices(
    commodity: string,
    startDate: string,
    endDate: string
  ): Promise<types.PriceAnalysis> {
    const response = await this.api.post('/market-prices/analyze', {
      commodity,
      startDate,
      endDate,
    });
    return response.data;
  }

  /**
   * Get markets
   */
  async getMarkets(regionId?: string): Promise<types.Market[]> {
    const response = await this.api.get('/markets', {
      params: { regionId },
    });
    return response.data;
  }

  // ========================================================================
  // Nutrition & Diet Methods
  // ========================================================================

  /**
   * Get nutritional requirements
   */
  async getNutritionalRequirements(
    populationGroup: string
  ): Promise<types.NutritionalRequirement> {
    const response = await this.api.get(`/nutrition/requirements/${populationGroup}`);
    return response.data;
  }

  /**
   * Record dietary intake
   */
  async recordDietaryIntake(
    intake: Partial<types.DietaryIntake>
  ): Promise<types.DietaryIntake> {
    const response = await this.api.post('/nutrition/intake', intake);
    return response.data;
  }

  /**
   * Analyze dietary adequacy
   */
  async analyzeDiet(
    householdId: string,
    period: { start: string; end: string }
  ): Promise<{ averageAdequacy: number; deficiencies: string[] }> {
    const response = await this.api.post('/nutrition/analyze', {
      householdId,
      period,
    });
    return response.data;
  }

  // ========================================================================
  // Food Assistance Methods
  // ========================================================================

  /**
   * Get food assistance programs
   */
  async getFoodAssistancePrograms(
    regionId?: string,
    type?: types.FoodAssistance['type']
  ): Promise<types.FoodAssistance[]> {
    const response = await this.api.get('/assistance-programs', {
      params: { regionId, type },
    });
    return response.data;
  }

  /**
   * Create food assistance program
   */
  async createFoodAssistanceProgram(
    program: Partial<types.FoodAssistance>
  ): Promise<types.FoodAssistance> {
    const response = await this.api.post('/assistance-programs', program);
    return response.data;
  }

  /**
   * Update food assistance program
   */
  async updateFoodAssistanceProgram(
    programId: string,
    updates: Partial<types.FoodAssistance>
  ): Promise<types.FoodAssistance> {
    const response = await this.api.patch(`/assistance-programs/${programId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Early Warning & Monitoring Methods
  // ========================================================================

  /**
   * Get early warning alerts
   */
  async getEarlyWarningAlerts(
    regionId?: string,
    severity?: types.EarlyWarningAlert['severity'],
    status?: types.EarlyWarningAlert['status']
  ): Promise<types.EarlyWarningAlert[]> {
    const response = await this.api.get('/early-warning/alerts', {
      params: { regionId, severity, status },
    });
    return response.data;
  }

  /**
   * Create early warning alert
   */
  async createEarlyWarningAlert(
    alert: Partial<types.EarlyWarningAlert>
  ): Promise<types.EarlyWarningAlert> {
    const response = await this.api.post('/early-warning/alerts', alert);
    return response.data;
  }

  /**
   * Update alert status
   */
  async updateAlertStatus(
    alertId: string,
    status: types.EarlyWarningAlert['status']
  ): Promise<types.EarlyWarningAlert> {
    const response = await this.api.patch(`/early-warning/alerts/${alertId}`, { status });
    return response.data;
  }

  /**
   * Get monitoring indicators
   */
  async getMonitoringIndicators(
    regionId: string,
    category?: string
  ): Promise<types.MonitoringIndicator[]> {
    const response = await this.api.get('/monitoring/indicators', {
      params: { regionId, category },
    });
    return response.data;
  }

  // ========================================================================
  // Intervention Methods
  // ========================================================================

  /**
   * Get interventions
   */
  async getInterventions(
    regionId?: string,
    type?: types.Intervention['type'],
    status?: types.Intervention['status']
  ): Promise<types.Intervention[]> {
    const response = await this.api.get('/interventions', {
      params: { regionId, type, status },
    });
    return response.data;
  }

  /**
   * Create intervention
   */
  async createIntervention(
    intervention: Partial<types.Intervention>
  ): Promise<types.Intervention> {
    const response = await this.api.post('/interventions', intervention);
    return response.data;
  }

  /**
   * Update intervention
   */
  async updateIntervention(
    interventionId: string,
    updates: Partial<types.Intervention>
  ): Promise<types.Intervention> {
    const response = await this.api.patch(`/interventions/${interventionId}`, updates);
    return response.data;
  }

  /**
   * Get intervention impact
   */
  async getInterventionImpact(interventionId: string): Promise<types.InterventionImpact> {
    const response = await this.api.get(`/interventions/${interventionId}/impact`);
    return response.data;
  }

  // ========================================================================
  // Climate & Environmental Methods
  // ========================================================================

  /**
   * Get climate data
   */
  async getClimateData(
    regionId: string,
    startTime: number,
    endTime: number
  ): Promise<types.ClimateData[]> {
    const response = await this.api.get('/climate/data', {
      params: { regionId, startTime, endTime },
    });
    return response.data;
  }

  /**
   * Get seasonal forecast
   */
  async getSeasonalForecast(
    regionId: string,
    season: string,
    year: number
  ): Promise<types.SeasonalForecast> {
    const response = await this.api.get('/climate/forecast', {
      params: { regionId, season, year },
    });
    return response.data;
  }

  // ========================================================================
  // Region Methods
  // ========================================================================

  /**
   * Get region information
   */
  async getRegion(regionId: string): Promise<types.Region> {
    const response = await this.api.get(`/regions/${regionId}`);
    return response.data;
  }

  /**
   * Get all regions
   */
  async getRegions(country?: string): Promise<types.Region[]> {
    const response = await this.api.get('/regions', {
      params: { country },
    });
    return response.data;
  }

  /**
   * Search regions by location
   */
  async searchRegionsByLocation(
    latitude: number,
    longitude: number,
    radius: number
  ): Promise<types.Region[]> {
    const response = await this.api.get('/regions/search', {
      params: { latitude, longitude, radius },
    });
    return response.data;
  }
}

/**
 * Utility functions for food security analysis
 */
export class FoodSecurityUtils {
  /**
   * Calculate food security score
   */
  static calculateSecurityScore(indicators: types.SecurityIndicators): number {
    const weights = {
      availability: 0.25,
      access: 0.25,
      utilization: 0.25,
      stability: 0.25,
    };

    const availabilityScore =
      (indicators.availability.productionPerCapita / 500) * 50 +
      indicators.availability.cropDiversityIndex * 50;

    const accessScore =
      indicators.access.marketAccessScore * 0.4 +
      (1 - indicators.access.affordabilityRatio) * 60;

    const utilizationScore =
      indicators.utilization.nutritionAdequacy * 0.5 +
      indicators.utilization.dietDiversity * 30 +
      (indicators.utilization.waterQualityIndex / 100) * 20;

    const stabilityScore =
      (1 - indicators.stability.priceVolatility) * 30 +
      (1 - indicators.stability.productionVariability) * 30 +
      (1 - indicators.stability.climateVulnerability) * 20 +
      (indicators.stability.politicalStability / 100) * 20;

    return (
      availabilityScore * weights.availability +
      accessScore * weights.access +
      utilizationScore * weights.utilization +
      stabilityScore * weights.stability
    );
  }

  /**
   * Determine security level from score
   */
  static determineSecurityLevel(score: number): types.SecurityLevel {
    if (score >= 80) return 'secure';
    if (score >= 60) return 'moderately_secure';
    if (score >= 40) return 'moderately_insecure';
    if (score >= 20) return 'severely_insecure';
    return 'famine';
  }

  /**
   * Calculate food gap
   */
  static calculateFoodGap(
    population: number,
    currentSupply: number,
    dailyRequirement: number = 2100
  ): number {
    const totalDailyNeeds = (population * dailyRequirement) / 1000; // in tons
    const annualNeeds = totalDailyNeeds * 365;
    return Math.max(0, annualNeeds - currentSupply);
  }

  /**
   * Estimate intervention cost
   */
  static estimateInterventionCost(
    beneficiaries: number,
    duration: number,
    costPerPersonPerDay: number = 0.6
  ): number {
    return beneficiaries * duration * costPerPersonPerDay;
  }

  /**
   * Calculate diet diversity score
   */
  static calculateDietDiversityScore(
    foodGroups: Record<types.FoodCategory, number>
  ): number {
    const consumedGroups = Object.values(foodGroups).filter((servings) => servings > 0).length;
    const totalGroups = Object.keys(foodGroups).length;
    return consumedGroups / totalGroups;
  }

  /**
   * Assess price volatility risk
   */
  static assessPriceVolatilityRisk(volatility: number): 'low' | 'medium' | 'high' | 'severe' {
    if (volatility < 0.1) return 'low';
    if (volatility < 0.2) return 'medium';
    if (volatility < 0.35) return 'high';
    return 'severe';
  }
}

/**
 * Export default client
 */
export default FoodSecurityClient;
