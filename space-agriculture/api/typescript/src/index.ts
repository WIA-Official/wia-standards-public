/**
 * WIA-AGRI-035 Space Agriculture Standard - TypeScript SDK
 * @module @wia/space-agriculture
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Space Agriculture Client
 */
export class SpaceAgricultureClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-spaceag.io/v1',
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
  // Space Farm Methods
  // ========================================================================

  async getSpaceFarms(type?: types.FarmType): Promise<types.SpaceFarm[]> {
    const response = await this.api.get('/farms', { params: { type } });
    return response.data;
  }

  async getSpaceFarm(farmId: string): Promise<types.SpaceFarm> {
    const response = await this.api.get(`/farms/${farmId}`);
    return response.data;
  }

  async createSpaceFarm(farm: Partial<types.SpaceFarm>): Promise<types.SpaceFarm> {
    const response = await this.api.post('/farms', farm);
    return response.data;
  }

  async updateSpaceFarm(farmId: string, updates: Partial<types.SpaceFarm>): Promise<types.SpaceFarm> {
    const response = await this.api.patch(`/farms/${farmId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Crop Module Methods
  // ========================================================================

  async getCropModules(farmId: string): Promise<types.CropModule[]> {
    const response = await this.api.get(`/farms/${farmId}/modules`);
    return response.data;
  }

  async getCropModule(moduleId: string): Promise<types.CropModule> {
    const response = await this.api.get(`/modules/${moduleId}`);
    return response.data;
  }

  async updateCropModule(moduleId: string, updates: Partial<types.CropModule>): Promise<types.CropModule> {
    const response = await this.api.patch(`/modules/${moduleId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Environmental Control Methods
  // ========================================================================

  async getEnvironmentalControl(moduleId: string): Promise<types.EnvironmentalControl> {
    const response = await this.api.get(`/modules/${moduleId}/environment`);
    return response.data;
  }

  async updateEnvironmentalControl(moduleId: string, control: Partial<types.EnvironmentalControl>): Promise<types.EnvironmentalControl> {
    const response = await this.api.patch(`/modules/${moduleId}/environment`, control);
    return response.data;
  }

  async getAtmosphericConditions(moduleId: string): Promise<types.AtmosphericConditions> {
    const response = await this.api.get(`/modules/${moduleId}/atmosphere`);
    return response.data;
  }

  async getLightingSystem(moduleId: string): Promise<types.LightingSystem> {
    const response = await this.api.get(`/modules/${moduleId}/lighting`);
    return response.data;
  }

  async updateLightingSystem(moduleId: string, lighting: Partial<types.LightingSystem>): Promise<types.LightingSystem> {
    const response = await this.api.patch(`/modules/${moduleId}/lighting`, lighting);
    return response.data;
  }

  // ========================================================================
  // Resource Management Methods
  // ========================================================================

  async getResourceBudget(farmId: string, period?: { start: string; end: string }): Promise<types.ResourceBudget> {
    const response = await this.api.get(`/farms/${farmId}/resources`, { params: period });
    return response.data;
  }

  async getWaterBudget(farmId: string): Promise<types.WaterBudget> {
    const response = await this.api.get(`/farms/${farmId}/water`);
    return response.data;
  }

  async getPowerBudget(farmId: string): Promise<types.PowerBudget> {
    const response = await this.api.get(`/farms/${farmId}/power`);
    return response.data;
  }

  async getNutrientBudget(farmId: string): Promise<types.NutrientBudget> {
    const response = await this.api.get(`/farms/${farmId}/nutrients`);
    return response.data;
  }

  // ========================================================================
  // Harvest Methods
  // ========================================================================

  async recordHarvest(harvest: Partial<types.SpaceHarvest>): Promise<types.SpaceHarvest> {
    const response = await this.api.post('/harvests', harvest);
    return response.data;
  }

  async getHarvests(moduleId?: string, startDate?: string, endDate?: string): Promise<types.SpaceHarvest[]> {
    const response = await this.api.get('/harvests', { params: { moduleId, startDate, endDate } });
    return response.data;
  }

  async getHarvest(harvestId: string): Promise<types.SpaceHarvest> {
    const response = await this.api.get(`/harvests/${harvestId}`);
    return response.data;
  }

  async submitCrewFeedback(harvestId: string, feedback: types.CrewFeedback): Promise<types.SpaceHarvest> {
    const response = await this.api.post(`/harvests/${harvestId}/feedback`, feedback);
    return response.data;
  }

  // ========================================================================
  // Life Support Integration Methods
  // ========================================================================

  async getLifeSupportMetrics(farmId: string): Promise<types.LifeSupportMetrics> {
    const response = await this.api.get(`/farms/${farmId}/life-support`);
    return response.data;
  }

  async getBioregenerativeRatio(farmId: string): Promise<types.BioregenerativeRatio> {
    const response = await this.api.get(`/farms/${farmId}/closure-ratio`);
    return response.data;
  }

  // ========================================================================
  // Automation Methods
  // ========================================================================

  async getAutomationSystem(moduleId: string): Promise<types.AutomationSystem> {
    const response = await this.api.get(`/modules/${moduleId}/automation`);
    return response.data;
  }

  async getSensors(moduleId: string, type?: string): Promise<types.Sensor[]> {
    const response = await this.api.get(`/modules/${moduleId}/sensors`, { params: { type } });
    return response.data;
  }

  async getActuators(moduleId: string): Promise<types.Actuator[]> {
    const response = await this.api.get(`/modules/${moduleId}/actuators`);
    return response.data;
  }

  async getRoboticSystems(farmId: string): Promise<types.RoboticSystem[]> {
    const response = await this.api.get(`/farms/${farmId}/robotics`);
    return response.data;
  }

  async updateRoboticSystem(robotId: string, updates: Partial<types.RoboticSystem>): Promise<types.RoboticSystem> {
    const response = await this.api.patch(`/robotics/${robotId}`, updates);
    return response.data;
  }

  // ========================================================================
  // Research Methods
  // ========================================================================

  async getExperiments(status?: string): Promise<types.SpaceAgExperiment[]> {
    const response = await this.api.get('/experiments', { params: { status } });
    return response.data;
  }

  async getExperiment(experimentId: string): Promise<types.SpaceAgExperiment> {
    const response = await this.api.get(`/experiments/${experimentId}`);
    return response.data;
  }

  async createExperiment(experiment: Partial<types.SpaceAgExperiment>): Promise<types.SpaceAgExperiment> {
    const response = await this.api.post('/experiments', experiment);
    return response.data;
  }

  async updateExperiment(experimentId: string, updates: Partial<types.SpaceAgExperiment>): Promise<types.SpaceAgExperiment> {
    const response = await this.api.patch(`/experiments/${experimentId}`, updates);
    return response.data;
  }

  async submitExperimentResults(experimentId: string, results: types.ExperimentResults): Promise<types.SpaceAgExperiment> {
    const response = await this.api.post(`/experiments/${experimentId}/results`, results);
    return response.data;
  }
}

/**
 * Utility functions for space agriculture
 */
export class SpaceAgUtils {
  static calculateEquivalentSystemMass(farmArea: number, powerConsumption: number): number {
    // ESM = area (m²) * 150 kg/m² + power (kW) * 50 kg/kW
    return farmArea * 150 + powerConsumption * 50;
  }

  static calculateClosureRatio(produced: number, consumed: number): number {
    return (produced / consumed) * 100;
  }

  static estimateCrewTimeSavings(autonomyLevel: number, baseHours: number): number {
    return baseHours * (1 - autonomyLevel / 100);
  }

  static calculateHarvestIndex(edibleBiomass: number, totalBiomass: number): number {
    return edibleBiomass / totalBiomass;
  }

  static estimateLaunchCost(mass: number, costPerKg: number = 10000): number {
    return mass * costPerKg; // Default $10k/kg to LEO
  }
}

export default SpaceAgricultureClient;
