/**
 * WIA-AGRI-031 Food Crisis Response Standard - TypeScript SDK
 * @module @wia/food-crisis-response
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Food Crisis Response Client
 */
export class FoodCrisisResponseClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-crisisresponse.io/v1',
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
  // Crisis Management Methods
  // ========================================================================

  /**
   * Get active food crises
   */
  async getCrises(status?: types.CrisisStatus, severity?: types.CrisisSeverity): Promise<types.FoodCrisis[]> {
    const response = await this.api.get('/crises', { params: { status, severity } });
    return response.data;
  }

  /**
   * Get crisis by ID
   */
  async getCrisis(crisisId: string): Promise<types.FoodCrisis> {
    const response = await this.api.get(`/crises/${crisisId}`);
    return response.data;
  }

  /**
   * Create new crisis
   */
  async createCrisis(crisis: Partial<types.FoodCrisis>): Promise<types.FoodCrisis> {
    const response = await this.api.post('/crises', crisis);
    return response.data;
  }

  /**
   * Update crisis information
   */
  async updateCrisis(crisisId: string, updates: Partial<types.FoodCrisis>): Promise<types.FoodCrisis> {
    const response = await this.api.patch(`/crises/${crisisId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Emergency Response Methods
  // ========================================================================

  /**
   * Get emergency responses
   */
  async getResponses(crisisId?: string, status?: string): Promise<types.EmergencyResponse[]> {
    const response = await this.api.get('/responses', { params: { crisisId, status } });
    return response.data;
  }

  /**
   * Create emergency response
   */
  async createResponse(response: Partial<types.EmergencyResponse>): Promise<types.EmergencyResponse> {
    const res = await this.api.post('/responses', response);
    return res.data;
  }

  /**
   * Update emergency response
   */
  async updateResponse(responseId: string, updates: Partial<types.EmergencyResponse>): Promise<types.EmergencyResponse> {
    const res = await this.api.patch(`/responses/${responseId}`, updates);
    return res.data;
  }

  // ========================================================================
  // Distribution Methods
  // ========================================================================

  /**
   * Get distributions
   */
  async getDistributions(operationId?: string, status?: string): Promise<types.Distribution[]> {
    const response = await this.api.get('/distributions', { params: { operationId, status } });
    return response.data;
  }

  /**
   * Create distribution
   */
  async createDistribution(distribution: Partial<types.Distribution>): Promise<types.Distribution> {
    const response = await this.api.post('/distributions', distribution);
    return response.data;
  }

  /**
   * Record distribution completion
   */
  async completeDistribution(distributionId: string, data: { beneficiaries: number; commodities: types.CommodityDistribution[] }): Promise<types.Distribution> {
    const response = await this.api.post(`/distributions/${distributionId}/complete`, data);
    return response.data;
  }

  /**
   * Get beneficiaries
   */
  async getBeneficiaries(householdId?: string): Promise<types.Beneficiary[]> {
    const response = await this.api.get('/beneficiaries', { params: { householdId } });
    return response.data;
  }

  /**
   * Register beneficiary
   */
  async registerBeneficiary(beneficiary: Partial<types.Beneficiary>): Promise<types.Beneficiary> {
    const response = await this.api.post('/beneficiaries', beneficiary);
    return response.data;
  }

  // ========================================================================
  // Logistics Methods
  // ========================================================================

  /**
   * Get logistics hubs
   */
  async getLogisticsHubs(type?: string): Promise<types.LogisticsHub[]> {
    const response = await this.api.get('/logistics/hubs', { params: { type } });
    return response.data;
  }

  /**
   * Get distribution routes
   */
  async getDistributionRoutes(from?: string, to?: string): Promise<types.DistributionRoute[]> {
    const response = await this.api.get('/logistics/routes', { params: { from, to } });
    return response.data;
  }

  /**
   * Get vehicle fleet
   */
  async getVehicles(status?: string): Promise<types.Vehicle[]> {
    const response = await this.api.get('/logistics/vehicles', { params: { status } });
    return response.data;
  }

  /**
   * Get warehouse inventory
   */
  async getWarehouses(regionId?: string): Promise<types.Warehouse[]> {
    const response = await this.api.get('/logistics/warehouses', { params: { regionId } });
    return response.data;
  }

  // ========================================================================
  // Nutrition Program Methods
  // ========================================================================

  /**
   * Get nutrition programs
   */
  async getNutritionPrograms(type?: types.NutritionProgram['type']): Promise<types.NutritionProgram[]> {
    const response = await this.api.get('/nutrition/programs', { params: { type } });
    return response.data;
  }

  /**
   * Create nutrition program
   */
  async createNutritionProgram(program: Partial<types.NutritionProgram>): Promise<types.NutritionProgram> {
    const response = await this.api.post('/nutrition/programs', program);
    return response.data;
  }

  /**
   * Get nutrition facilities
   */
  async getNutritionFacilities(type?: string): Promise<types.NutritionFacility[]> {
    const response = await this.api.get('/nutrition/facilities', { params: { type } });
    return response.data;
  }

  // ========================================================================
  // Assessment Methods
  // ========================================================================

  /**
   * Create rapid assessment
   */
  async createRapidAssessment(assessment: Partial<types.RapidAssessment>): Promise<types.RapidAssessment> {
    const response = await this.api.post('/assessments/rapid', assessment);
    return response.data;
  }

  /**
   * Get assessments
   */
  async getAssessments(crisisId?: string): Promise<types.RapidAssessment[]> {
    const response = await this.api.get('/assessments', { params: { crisisId } });
    return response.data;
  }

  /**
   * Create monitoring report
   */
  async createMonitoringReport(report: Partial<types.MonitoringReport>): Promise<types.MonitoringReport> {
    const response = await this.api.post('/monitoring/reports', report);
    return response.data;
  }

  /**
   * Get monitoring reports
   */
  async getMonitoringReports(responseId: string): Promise<types.MonitoringReport[]> {
    const response = await this.api.get('/monitoring/reports', { params: { responseId } });
    return response.data;
  }

  // ========================================================================
  // Coordination Methods
  // ========================================================================

  /**
   * Create coordination meeting
   */
  async createMeeting(meeting: Partial<types.CoordinationMeeting>): Promise<types.CoordinationMeeting> {
    const response = await this.api.post('/coordination/meetings', meeting);
    return response.data;
  }

  /**
   * Get meetings
   */
  async getMeetings(type?: string, startDate?: string): Promise<types.CoordinationMeeting[]> {
    const response = await this.api.get('/coordination/meetings', { params: { type, startDate } });
    return response.data;
  }

  /**
   * Update action point
   */
  async updateActionPoint(actionId: string, status: types.ActionPoint['status']): Promise<types.ActionPoint> {
    const response = await this.api.patch(`/coordination/actions/${actionId}`, { status });
    return response.data;
  }

  // ========================================================================
  // Funding Methods
  // ========================================================================

  /**
   * Get funding appeals
   */
  async getFundingAppeals(crisisId?: string): Promise<types.FundingAppeal[]> {
    const response = await this.api.get('/funding/appeals', { params: { crisisId } });
    return response.data;
  }

  /**
   * Create funding appeal
   */
  async createFundingAppeal(appeal: Partial<types.FundingAppeal>): Promise<types.FundingAppeal> {
    const response = await this.api.post('/funding/appeals', appeal);
    return response.data;
  }

  /**
   * Record donation
   */
  async recordDonation(appealId: string, donation: { donorId: string; amount: number; type: 'pledge' | 'disbursement' }): Promise<types.FundingAppeal> {
    const response = await this.api.post(`/funding/appeals/${appealId}/donations`, donation);
    return response.data;
  }
}

/**
 * Utility functions for crisis response
 */
export class CrisisResponseUtils {
  /**
   * Calculate ration requirements
   */
  static calculateRationRequirements(beneficiaries: number, days: number, caloriesPerPerson: number = 2100): number {
    return (beneficiaries * days * caloriesPerPerson) / 1000; // in kcal (thousands)
  }

  /**
   * Estimate response cost
   */
  static estimateResponseCost(beneficiaries: number, duration: number, costPerPersonPerDay: number = 0.6): number {
    return beneficiaries * duration * costPerPersonPerDay;
  }

  /**
   * Calculate IPC phase from indicators
   */
  static calculateIPCPhase(malnutritionRate: number, mortalityRate: number, foodConsumptionScore: number): types.CrisisSeverity {
    if (mortalityRate >= 2 || malnutritionRate >= 30) return 'phase_5';
    if (mortalityRate >= 1 || malnutritionRate >= 15) return 'phase_4';
    if (malnutritionRate >= 10 || foodConsumptionScore < 28) return 'phase_3';
    if (malnutritionRate >= 5 || foodConsumptionScore < 42) return 'phase_2';
    return 'phase_1';
  }
}

export default FoodCrisisResponseClient;
