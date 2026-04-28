/**
 * WIA Urban Forest Standard - TypeScript SDK
 * Version: 1.0.0
 *
 * This SDK provides convenient access to WIA-compliant urban forest
 * management APIs and utilities for working with urban tree inventory data.
 */

import axios, { AxiosInstance } from 'axios';
import * as types from './types';

export * from './types';

// ============================================================================
// WIA Urban Forest Client
// ============================================================================

export class WIAUrbanForest {
  private client: AxiosInstance;
  private apiKey?: string;
  private city?: string;

  constructor(config: types.ClientConfig = {}) {
    this.apiKey = config.apiKey;
    this.city = config.city;
    this.client = axios.create({
      baseURL: config.baseURL || 'https://api.urban-forest.wia.org/v1',
      timeout: config.timeout || 30000,
      headers: config.apiKey ? {
        'Authorization': `Bearer ${config.apiKey}`
      } : {}
    });
  }

  // ========================================================================
  // Tree Inventory Methods
  // ========================================================================

  /**
   * Query tree inventory records
   */
  async getTreeInventory(
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.TreeInventoryRecord>> {
    const response = await this.client.get('/inventory', {
      params: this.buildQueryParams(options)
    });
    return response.data;
  }

  /**
   * Get specific tree by ID
   */
  async getTree(treeId: string): Promise<types.TreeInventoryRecord> {
    const response = await this.client.get(`/inventory/${treeId}`);
    return response.data.data;
  }

  /**
   * Create new tree inventory record
   */
  async createTree(
    tree: types.TreeInventoryRecord
  ): Promise<types.APIResponse<types.TreeInventoryRecord>> {
    const response = await this.client.post('/inventory', tree);
    return response.data;
  }

  /**
   * Update existing tree record
   */
  async updateTree(
    treeId: string,
    updates: Partial<types.TreeInventoryRecord>
  ): Promise<types.APIResponse<types.TreeInventoryRecord>> {
    const response = await this.client.patch(`/inventory/${treeId}`, updates);
    return response.data;
  }

  /**
   * Delete tree record
   */
  async deleteTree(treeId: string): Promise<types.APIResponse<any>> {
    const response = await this.client.delete(`/inventory/${treeId}`);
    return response.data;
  }

  /**
   * Get trees by species
   */
  async getTreesBySpecies(
    scientificName: string,
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.TreeInventoryRecord>> {
    return this.getTreeInventory({
      ...options,
      species: scientificName
    });
  }

  /**
   * Get trees by condition
   */
  async getTreesByCondition(
    condition: types.TreeCondition,
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.TreeInventoryRecord>> {
    return this.getTreeInventory({
      ...options,
      condition
    });
  }

  // ========================================================================
  // Canopy Analysis Methods
  // ========================================================================

  /**
   * Get canopy analysis records
   */
  async getCanopyAnalyses(
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.CanopyAnalysis>> {
    const response = await this.client.get('/canopy', {
      params: this.buildQueryParams(options)
    });
    return response.data;
  }

  /**
   * Get specific canopy analysis by ID
   */
  async getCanopyAnalysis(analysisId: string): Promise<types.CanopyAnalysis> {
    const response = await this.client.get(`/canopy/${analysisId}`);
    return response.data.data;
  }

  /**
   * Create new canopy analysis
   */
  async createCanopyAnalysis(
    analysis: types.CanopyAnalysis
  ): Promise<types.APIResponse<types.CanopyAnalysis>> {
    const response = await this.client.post('/canopy', analysis);
    return response.data;
  }

  /**
   * Calculate canopy coverage for an area
   */
  async calculateCanopyCoverage(
    areaName: string,
    bbox: [number, number, number, number]
  ): Promise<types.CanopyMetrics> {
    const response = await this.client.post('/canopy/calculate', {
      area_name: areaName,
      bbox
    });
    return response.data.data;
  }

  // ========================================================================
  // Planting Planning Methods
  // ========================================================================

  /**
   * Get planting plans
   */
  async getPlantingPlans(
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.PlantingPlan>> {
    const response = await this.client.get('/planting-plans', {
      params: this.buildQueryParams(options)
    });
    return response.data;
  }

  /**
   * Get specific planting plan
   */
  async getPlantingPlan(planId: string): Promise<types.PlantingPlan> {
    const response = await this.client.get(`/planting-plans/${planId}`);
    return response.data.data;
  }

  /**
   * Create new planting plan
   */
  async createPlantingPlan(
    plan: types.PlantingPlan
  ): Promise<types.APIResponse<types.PlantingPlan>> {
    const response = await this.client.post('/planting-plans', plan);
    return response.data;
  }

  /**
   * Get species recommendations for a site
   */
  async getSpeciesRecommendations(
    site: types.PlantingSite
  ): Promise<types.SpeciesRecommendation[]> {
    const response = await this.client.post('/planting-plans/recommend', site);
    return response.data.data;
  }

  // ========================================================================
  // Maintenance Scheduling Methods
  // ========================================================================

  /**
   * Get maintenance schedules
   */
  async getMaintenanceSchedules(
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.MaintenanceSchedule>> {
    const response = await this.client.get('/maintenance', {
      params: this.buildQueryParams(options)
    });
    return response.data;
  }

  /**
   * Get specific maintenance schedule
   */
  async getMaintenanceSchedule(
    scheduleId: string
  ): Promise<types.MaintenanceSchedule> {
    const response = await this.client.get(`/maintenance/${scheduleId}`);
    return response.data.data;
  }

  /**
   * Create maintenance schedule
   */
  async createMaintenanceSchedule(
    schedule: types.MaintenanceSchedule
  ): Promise<types.APIResponse<types.MaintenanceSchedule>> {
    const response = await this.client.post('/maintenance', schedule);
    return response.data;
  }

  /**
   * Update maintenance task status
   */
  async updateMaintenanceTask(
    taskId: string,
    updates: Partial<types.MaintenanceTask>
  ): Promise<types.APIResponse<types.MaintenanceTask>> {
    const response = await this.client.patch(`/maintenance/tasks/${taskId}`, updates);
    return response.data;
  }

  /**
   * Get maintenance tasks by priority
   */
  async getMaintenanceTasksByPriority(
    priority: types.MaintenancePriority
  ): Promise<types.MaintenanceTask[]> {
    const response = await this.client.get('/maintenance/tasks', {
      params: { priority }
    });
    return response.data.data;
  }

  // ========================================================================
  // Health Monitoring Methods
  // ========================================================================

  /**
   * Get health assessments
   */
  async getHealthAssessments(
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.HealthAssessment>> {
    const response = await this.client.get('/health', {
      params: this.buildQueryParams(options)
    });
    return response.data;
  }

  /**
   * Get specific health assessment
   */
  async getHealthAssessment(
    assessmentId: string
  ): Promise<types.HealthAssessment> {
    const response = await this.client.get(`/health/${assessmentId}`);
    return response.data.data;
  }

  /**
   * Create health assessment
   */
  async createHealthAssessment(
    assessment: types.HealthAssessment
  ): Promise<types.APIResponse<types.HealthAssessment>> {
    const response = await this.client.post('/health', assessment);
    return response.data;
  }

  /**
   * Get trees with high risk ratings
   */
  async getHighRiskTrees(): Promise<types.HealthAssessment[]> {
    const response = await this.client.get('/health', {
      params: { risk_rating: 'extreme,high' }
    });
    return response.data.data;
  }

  /**
   * Get pest and disease alerts
   */
  async getPestDiseaseAlerts(
    city?: string
  ): Promise<Array<{
    type: 'pest' | 'disease';
    name: string;
    affected_trees: number;
    severity: string;
  }>> {
    const response = await this.client.get('/health/alerts', {
      params: city ? { city } : {}
    });
    return response.data.data;
  }

  // ========================================================================
  // Carbon Calculation Methods
  // ========================================================================

  /**
   * Get carbon calculations
   */
  async getCarbonCalculations(
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.CarbonCalculation>> {
    const response = await this.client.get('/carbon', {
      params: this.buildQueryParams(options)
    });
    return response.data;
  }

  /**
   * Get specific carbon calculation
   */
  async getCarbonCalculation(
    calculationId: string
  ): Promise<types.CarbonCalculation> {
    const response = await this.client.get(`/carbon/${calculationId}`);
    return response.data.data;
  }

  /**
   * Calculate carbon sequestration for trees
   */
  async calculateCarbon(
    treeIds: string[]
  ): Promise<types.CarbonCalculation> {
    const response = await this.client.post('/carbon/calculate', {
      tree_ids: treeIds
    });
    return response.data.data;
  }

  /**
   * Get total carbon metrics for a city or area
   */
  async getCarbonMetrics(
    areaName?: string
  ): Promise<{
    total_carbon_stored_tons: number;
    annual_sequestration_tons: number;
    co2_equivalent_tons: number;
    monetary_value_usd: number;
  }> {
    const response = await this.client.get('/carbon/metrics', {
      params: areaName ? { area: areaName } : {}
    });
    return response.data.data;
  }

  // ========================================================================
  // Ecosystem Services Methods
  // ========================================================================

  /**
   * Get ecosystem services assessments
   */
  async getEcosystemServices(
    options: types.QueryOptions = {}
  ): Promise<types.APIResponse<types.EcosystemServices>> {
    const response = await this.client.get('/ecosystem-services', {
      params: this.buildQueryParams(options)
    });
    return response.data;
  }

  /**
   * Calculate ecosystem services for an area
   */
  async calculateEcosystemServices(
    areaName: string,
    treeIds?: string[]
  ): Promise<types.EcosystemServices> {
    const response = await this.client.post('/ecosystem-services/calculate', {
      area_name: areaName,
      tree_ids: treeIds
    });
    return response.data.data;
  }

  // ========================================================================
  // Event Handling Methods
  // ========================================================================

  /**
   * Get urban forest events
   */
  async getEvents(
    options: {
      event_type?: types.EventType;
      start_date?: string;
      end_date?: string;
      severity?: 'info' | 'warning' | 'critical';
      limit?: number;
    } = {}
  ): Promise<types.APIResponse<types.UrbanForestEvent>> {
    const response = await this.client.get('/events', {
      params: options
    });
    return response.data;
  }

  /**
   * Create event
   */
  async createEvent(
    event: types.UrbanForestEvent
  ): Promise<types.APIResponse<types.UrbanForestEvent>> {
    const response = await this.client.post('/events', event);
    return response.data;
  }

  /**
   * Subscribe to real-time events (WebSocket)
   */
  async subscribeToEvents(
    eventTypes: types.EventType[],
    callback: (event: types.UrbanForestEvent) => void
  ): Promise<void> {
    // WebSocket implementation would go here
    // This is a placeholder for the actual WebSocket connection
    console.log('Event subscription for:', eventTypes);
  }

  // ========================================================================
  // Analytics and Metrics Methods
  // ========================================================================

  /**
   * Get urban forest metrics for a city or area
   */
  async getMetrics(
    areaName?: string
  ): Promise<types.UrbanForestMetrics> {
    const response = await this.client.get('/metrics', {
      params: areaName ? { area: areaName } : {}
    });
    return response.data.data;
  }

  /**
   * Get species diversity analysis
   */
  async getSpeciesDiversity(
    areaName?: string
  ): Promise<{
    total_species: number;
    diversity_index: number;
    species_distribution: Array<{
      species: string;
      count: number;
      percentage: number;
    }>;
  }> {
    const response = await this.client.get('/analytics/diversity', {
      params: areaName ? { area: areaName } : {}
    });
    return response.data.data;
  }

  /**
   * Get canopy coverage trends over time
   */
  async getCanopyTrends(
    areaName: string,
    startYear: number,
    endYear: number
  ): Promise<Array<{
    year: number;
    canopy_percentage: number;
    canopy_area_m2: number;
  }>> {
    const response = await this.client.get('/analytics/canopy-trends', {
      params: { area: areaName, start_year: startYear, end_year: endYear }
    });
    return response.data.data;
  }

  // ========================================================================
  // Utility Methods
  // ========================================================================

  /**
   * Build query parameters
   */
  private buildQueryParams(options: types.QueryOptions): Record<string, any> {
    const params: Record<string, any> = {};

    if (this.city && !options.city) params.city = this.city;
    if (options.city) params.city = options.city;
    if (options.district) params.district = options.district;
    if (options.species) params.species = options.species;
    if (options.condition) params.condition = options.condition;
    if (options.status) params.status = options.status;
    if (options.start_date) params.start_date = options.start_date;
    if (options.end_date) params.end_date = options.end_date;
    if (options.bbox) params.bbox = options.bbox.join(',');
    if (options.limit) params.limit = options.limit;
    if (options.offset) params.offset = options.offset;
    if (options.format) params.format = options.format;
    if (options.sort_by) params.sort_by = options.sort_by;
    if (options.sort_order) params.sort_order = options.sort_order;

    return params;
  }

  /**
   * Export tree inventory to GeoJSON
   */
  async exportToGeoJSON(
    options: types.QueryOptions = {}
  ): Promise<any> {
    const response = await this.getTreeInventory({
      ...options,
      format: 'geojson'
    });
    return response.data;
  }

  /**
   * Export tree inventory to CSV
   */
  async exportToCSV(
    options: types.QueryOptions = {}
  ): Promise<string> {
    const response = await this.client.get('/inventory', {
      params: this.buildQueryParams({ ...options, format: 'csv' }),
      responseType: 'text'
    });
    return response.data;
  }

  /**
   * Validate tree inventory record
   */
  validateTreeRecord(
    tree: types.TreeInventoryRecord
  ): types.ValidationResult {
    const errors: types.ValidationError[] = [];
    const warnings: types.ValidationWarning[] = [];

    // Required field validations
    if (!tree.tree_id) {
      errors.push({
        field: 'tree_id',
        message: 'tree_id is required',
        severity: 'error'
      });
    }

    if (!tree.species?.scientific_name) {
      errors.push({
        field: 'species.scientific_name',
        message: 'species scientific_name is required',
        severity: 'error'
      });
    }

    if (!tree.measurements?.diameter_at_breast_height_cm) {
      errors.push({
        field: 'measurements.diameter_at_breast_height_cm',
        message: 'DBH measurement is required',
        severity: 'error'
      });
    }

    // Location validation
    if (!tree.location?.latitude || !tree.location?.longitude) {
      errors.push({
        field: 'location',
        message: 'latitude and longitude are required',
        severity: 'error'
      });
    }

    // Logical validations (warnings)
    if (tree.measurements?.diameter_at_breast_height_cm > 300) {
      warnings.push({
        field: 'measurements.diameter_at_breast_height_cm',
        message: 'DBH exceeds typical maximum (300cm)',
        value: tree.measurements.diameter_at_breast_height_cm,
        severity: 'warning'
      });
    }

    if (tree.measurements?.height_m && tree.measurements.height_m > 100) {
      warnings.push({
        field: 'measurements.height_m',
        message: 'Height exceeds typical maximum (100m)',
        value: tree.measurements.height_m,
        severity: 'warning'
      });
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings
    };
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Urban Forest client instance
 */
export function createUrbanForestClient(
  config: types.ClientConfig = {}
): WIAUrbanForest {
  return new WIAUrbanForest(config);
}

// ============================================================================
// Export default client class
// ============================================================================

export default WIAUrbanForest;
