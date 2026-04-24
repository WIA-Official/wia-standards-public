/**
 * WIA-AGRI-027 Aeroponics Standard - TypeScript SDK
 * @module @wia/aeroponics
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

/**
 * Main Aeroponics Client
 */
export class AeroponicsClient {
  private api: AxiosInstance;
  private config: types.ClientConfig;

  constructor(config: types.ClientConfig) {
    this.config = {
      baseURL: 'https://api.wia-aeroponics.io/v1',
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
  // System Management Methods
  // ========================================================================

  /**
   * Get all aeroponics systems
   */
  async getSystems(page: number = 1, pageSize: number = 20): Promise<types.PaginatedResponse<types.AeroponicsSystem>> {
    const response = await this.api.get('/systems', {
      params: { page, pageSize },
    });
    return response.data;
  }

  /**
   * Get system by ID
   */
  async getSystem(systemId: string): Promise<types.AeroponicsSystem> {
    const response = await this.api.get(`/systems/${systemId}`);
    return response.data;
  }

  /**
   * Create a new system
   */
  async createSystem(system: Partial<types.AeroponicsSystem>): Promise<types.AeroponicsSystem> {
    const response = await this.api.post('/systems', system);
    return response.data;
  }

  /**
   * Update system configuration
   */
  async updateSystem(
    systemId: string,
    updates: Partial<types.AeroponicsSystem>
  ): Promise<types.AeroponicsSystem> {
    const response = await this.api.patch(`/systems/${systemId}`, updates);
    return response.data;
  }

  /**
   * Delete a system
   */
  async deleteSystem(systemId: string): Promise<void> {
    await this.api.delete(`/systems/${systemId}`);
  }

  // ========================================================================
  // Misting System Methods
  // ========================================================================

  /**
   * Get misting system configuration
   */
  async getMistingSystem(systemId: string): Promise<types.MistingSystem> {
    const response = await this.api.get(`/systems/${systemId}/misting`);
    return response.data;
  }

  /**
   * Update misting system configuration
   */
  async updateMistingSystem(
    systemId: string,
    config: Partial<types.MistingSystem>
  ): Promise<types.MistingSystem> {
    const response = await this.api.patch(`/systems/${systemId}/misting`, config);
    return response.data;
  }

  /**
   * Start misting cycle
   */
  async startMisting(systemId: string): Promise<types.MistingSystem> {
    const response = await this.api.post(`/systems/${systemId}/misting/start`);
    return response.data;
  }

  /**
   * Stop misting cycle
   */
  async stopMisting(systemId: string): Promise<types.MistingSystem> {
    const response = await this.api.post(`/systems/${systemId}/misting/stop`);
    return response.data;
  }

  // ========================================================================
  // Nutrient Solution Methods
  // ========================================================================

  /**
   * Get current nutrient solution status
   */
  async getNutrientSolution(systemId: string): Promise<types.NutrientSolution> {
    const response = await this.api.get(`/systems/${systemId}/nutrients`);
    return response.data;
  }

  /**
   * Update nutrient solution
   */
  async updateNutrientSolution(
    systemId: string,
    solution: Partial<types.NutrientSolution>
  ): Promise<types.NutrientSolution> {
    const response = await this.api.post(`/systems/${systemId}/nutrients`, solution);
    return response.data;
  }

  /**
   * Get nutrient recipes
   */
  async getNutrientRecipes(cropType?: string): Promise<types.NutrientRecipe[]> {
    const response = await this.api.get('/recipes', {
      params: { cropType },
    });
    return response.data;
  }

  /**
   * Create nutrient recipe
   */
  async createNutrientRecipe(recipe: Partial<types.NutrientRecipe>): Promise<types.NutrientRecipe> {
    const response = await this.api.post('/recipes', recipe);
    return response.data;
  }

  /**
   * Apply nutrient recipe to system
   */
  async applyNutrientRecipe(systemId: string, recipeId: string): Promise<types.NutrientSolution> {
    const response = await this.api.post(`/systems/${systemId}/nutrients/apply-recipe`, {
      recipeId,
    });
    return response.data;
  }

  // ========================================================================
  // Plant Management Methods
  // ========================================================================

  /**
   * Get all plants in a system
   */
  async getPlants(systemId: string): Promise<types.Plant[]> {
    const response = await this.api.get(`/systems/${systemId}/plants`);
    return response.data;
  }

  /**
   * Get plant by ID
   */
  async getPlant(plantId: string): Promise<types.Plant> {
    const response = await this.api.get(`/plants/${plantId}`);
    return response.data;
  }

  /**
   * Add plant to system
   */
  async addPlant(plant: Partial<types.Plant>): Promise<types.Plant> {
    const response = await this.api.post('/plants', plant);
    return response.data;
  }

  /**
   * Update plant information
   */
  async updatePlant(plantId: string, updates: Partial<types.Plant>): Promise<types.Plant> {
    const response = await this.api.patch(`/plants/${plantId}`, updates);
    return response.data;
  }

  /**
   * Remove plant from system (harvest)
   */
  async removePlant(plantId: string): Promise<void> {
    await this.api.delete(`/plants/${plantId}`);
  }

  /**
   * Get plant health assessment
   */
  async getPlantHealth(plantId: string): Promise<types.PlantHealth> {
    const response = await this.api.get(`/plants/${plantId}/health`);
    return response.data;
  }

  // ========================================================================
  // Harvest & Yield Methods
  // ========================================================================

  /**
   * Record crop harvest
   */
  async recordHarvest(harvest: Partial<types.CropYield>): Promise<types.CropYield> {
    const response = await this.api.post('/harvests', harvest);
    return response.data;
  }

  /**
   * Get harvest history
   */
  async getHarvests(
    systemId?: string,
    startDate?: string,
    endDate?: string
  ): Promise<types.CropYield[]> {
    const response = await this.api.get('/harvests', {
      params: { systemId, startDate, endDate },
    });
    return response.data;
  }

  // ========================================================================
  // Environmental Monitoring Methods
  // ========================================================================

  /**
   * Get current environmental data
   */
  async getEnvironmentalData(systemId: string): Promise<types.EnvironmentalData> {
    const response = await this.api.get(`/systems/${systemId}/environment`);
    return response.data;
  }

  /**
   * Get environmental data history
   */
  async getEnvironmentalHistory(
    systemId: string,
    startTime: number,
    endTime: number
  ): Promise<types.EnvironmentalData[]> {
    const response = await this.api.get(`/systems/${systemId}/environment/history`, {
      params: { startTime, endTime },
    });
    return response.data;
  }

  /**
   * Get climate control settings
   */
  async getClimateControl(systemId: string): Promise<types.ClimateControl> {
    const response = await this.api.get(`/systems/${systemId}/climate`);
    return response.data;
  }

  /**
   * Update climate control settings
   */
  async updateClimateControl(
    systemId: string,
    config: Partial<types.ClimateControl>
  ): Promise<types.ClimateControl> {
    const response = await this.api.patch(`/systems/${systemId}/climate`, config);
    return response.data;
  }

  // ========================================================================
  // Sensor Methods
  // ========================================================================

  /**
   * Get sensor readings
   */
  async getSensorReadings(
    systemId: string,
    sensorType?: string
  ): Promise<types.SensorReading[]> {
    const response = await this.api.get(`/systems/${systemId}/sensors`, {
      params: { sensorType },
    });
    return response.data;
  }

  /**
   * Calibrate sensor
   */
  async calibrateSensor(sensorId: string, calibrationValue: number): Promise<types.SensorReading> {
    const response = await this.api.post(`/sensors/${sensorId}/calibrate`, {
      calibrationValue,
    });
    return response.data;
  }

  // ========================================================================
  // Alert Methods
  // ========================================================================

  /**
   * Get alerts for a system
   */
  async getAlerts(
    systemId?: string,
    level?: types.AlertLevel,
    acknowledged?: boolean
  ): Promise<types.Alert[]> {
    const response = await this.api.get('/alerts', {
      params: { systemId, level, acknowledged },
    });
    return response.data;
  }

  /**
   * Acknowledge an alert
   */
  async acknowledgeAlert(alertId: string): Promise<types.Alert> {
    const response = await this.api.patch(`/alerts/${alertId}/acknowledge`);
    return response.data;
  }

  /**
   * Resolve an alert
   */
  async resolveAlert(alertId: string, notes?: string): Promise<types.Alert> {
    const response = await this.api.patch(`/alerts/${alertId}/resolve`, { notes });
    return response.data;
  }

  // ========================================================================
  // Maintenance Methods
  // ========================================================================

  /**
   * Log maintenance activity
   */
  async logMaintenance(log: Partial<types.MaintenanceLog>): Promise<types.MaintenanceLog> {
    const response = await this.api.post('/maintenance', log);
    return response.data;
  }

  /**
   * Get maintenance history
   */
  async getMaintenanceHistory(
    systemId: string,
    type?: types.MaintenanceLog['type']
  ): Promise<types.MaintenanceLog[]> {
    const response = await this.api.get('/maintenance', {
      params: { systemId, type },
    });
    return response.data;
  }

  /**
   * Get upcoming maintenance schedule
   */
  async getMaintenanceSchedule(systemId: string): Promise<types.MaintenanceLog[]> {
    const response = await this.api.get(`/systems/${systemId}/maintenance/schedule`);
    return response.data;
  }

  // ========================================================================
  // Analytics & Metrics Methods
  // ========================================================================

  /**
   * Get system metrics
   */
  async getSystemMetrics(
    systemId: string,
    startTime: number,
    endTime: number
  ): Promise<types.SystemMetrics> {
    const response = await this.api.get(`/systems/${systemId}/metrics`, {
      params: { startTime, endTime },
    });
    return response.data;
  }

  /**
   * Generate system report
   */
  async generateReport(
    systemId: string,
    reportType: 'daily' | 'weekly' | 'monthly' | 'custom',
    options?: {
      startDate?: string;
      endDate?: string;
      includeCharts?: boolean;
    }
  ): Promise<Blob> {
    const response = await this.api.post(
      `/systems/${systemId}/reports`,
      { reportType, ...options },
      { responseType: 'blob' }
    );
    return response.data;
  }
}

/**
 * Utility functions for aeroponics calculations
 */
export class AeroponicsUtils {
  /**
   * Calculate optimal droplet size for crop type
   */
  static calculateOptimalDropletSize(cropType: string, growthStage: types.GrowthStage): number {
    // Optimal range is typically 5-50 microns
    const baseSize = 20; // microns
    const stageMultiplier = {
      seedling: 0.7,
      vegetative: 1.0,
      flowering: 1.2,
      fruiting: 1.3,
      harvest: 1.0,
    };
    return baseSize * stageMultiplier[growthStage];
  }

  /**
   * Calculate nutrient solution volume needed
   */
  static calculateSolutionVolume(
    numberOfPlants: number,
    growthStage: types.GrowthStage
  ): number {
    const baseVolumePerPlant = 0.5; // liters
    const stageMultiplier = {
      seedling: 0.5,
      vegetative: 1.0,
      flowering: 1.5,
      fruiting: 2.0,
      harvest: 1.0,
    };
    return numberOfPlants * baseVolumePerPlant * stageMultiplier[growthStage];
  }

  /**
   * Calculate misting frequency based on environmental conditions
   */
  static calculateMistingFrequency(
    temperature: number,
    humidity: number,
    growthStage: types.GrowthStage
  ): number {
    // Returns recommended seconds between misting cycles
    let baseFrequency = 300; // 5 minutes

    // Adjust for temperature
    if (temperature > 25) {
      baseFrequency *= 0.8;
    } else if (temperature < 18) {
      baseFrequency *= 1.2;
    }

    // Adjust for humidity
    if (humidity < 60) {
      baseFrequency *= 0.9;
    } else if (humidity > 80) {
      baseFrequency *= 1.1;
    }

    // Adjust for growth stage
    const stageMultiplier = {
      seedling: 0.8,
      vegetative: 1.0,
      flowering: 1.1,
      fruiting: 1.2,
      harvest: 1.0,
    };

    return Math.round(baseFrequency * stageMultiplier[growthStage]);
  }

  /**
   * Validate nutrient solution parameters
   */
  static validateNutrientSolution(solution: types.NutrientSolution): {
    valid: boolean;
    warnings: string[];
  } {
    const warnings: string[] = [];

    // Check pH range
    if (solution.pH < 5.5 || solution.pH > 6.5) {
      warnings.push(`pH ${solution.pH} is outside optimal range (5.5-6.5)`);
    }

    // Check EC range
    if (solution.ec < 1.2 || solution.ec > 2.5) {
      warnings.push(`EC ${solution.ec} mS/cm is outside optimal range (1.2-2.5)`);
    }

    // Check temperature
    if (solution.temperature < 18 || solution.temperature > 24) {
      warnings.push(`Temperature ${solution.temperature}°C is outside optimal range (18-24°C)`);
    }

    // Check dissolved oxygen
    if (solution.dissolvedOxygen < 6) {
      warnings.push(`Dissolved oxygen ${solution.dissolvedOxygen} mg/L is too low (minimum 6)`);
    }

    return {
      valid: warnings.length === 0,
      warnings,
    };
  }
}

/**
 * Export default client
 */
export default AeroponicsClient;
