/**
 * WIA-ENE-XXX: Wetland Conservation Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  WetlandSite,
  WaterLevelMeasurement,
  WaterQuality,
  HydrologicData,
  VegetationSurvey,
  WetlandSpeciesObservation,
  AvianAssessment,
  WetlandCondition,
  FunctionalAssessment,
  EcosystemServiceValuation,
  RestorationProject,
  ManagementAction,
  WetlandThreat,
  MonitoringProgram,
  WetlandHealthReport,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  LocationFilter,
  WetlandType,
  ProtectionStatus,
  RestorationType,
  EcosystemServiceType,
  VegetationType,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface WetlandConservationSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// Event Types
// ============================================================================

export type WetlandEventType =
  | 'site.created'
  | 'site.updated'
  | 'site.deleted'
  | 'water.level.changed'
  | 'water.quality.alert'
  | 'vegetation.survey.completed'
  | 'species.observed'
  | 'condition.assessed'
  | 'threat.identified'
  | 'restoration.started'
  | 'restoration.completed'
  | 'report.generated';

export interface WetlandEvent {
  eventType: WetlandEventType;
  timestamp: string;
  siteId: string;
  data: any;
}

export type WetlandEventHandler = (event: WetlandEvent) => void | Promise<void>;

// ============================================================================
// SDK Client
// ============================================================================

export class WIAWetlandConservation {
  private config: Required<WetlandConservationSDKConfig>;
  private headers: Record<string, string>;
  private eventHandlers: Map<WetlandEventType, WetlandEventHandler[]>;

  constructor(config: WetlandConservationSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-XXX',
      'X-WIA-Version': '1.0.0',
    };

    this.eventHandlers = new Map();
  }

  // ==========================================================================
  // Wetland Site Management
  // ==========================================================================

  /**
   * Create a new wetland site
   */
  async createWetlandSite(
    site: Omit<WetlandSite, 'siteId' | 'metadata'>
  ): Promise<ApiResponse<WetlandSite>> {
    const response = await this.post<WetlandSite>('/api/v1/sites', site);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'site.created',
        timestamp: new Date().toISOString(),
        siteId: response.data.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get wetland site by ID
   */
  async getWetlandSite(siteId: string): Promise<ApiResponse<WetlandSite>> {
    return this.get<WetlandSite>(`/api/v1/sites/${siteId}`);
  }

  /**
   * Update wetland site
   */
  async updateWetlandSite(
    siteId: string,
    updates: Partial<WetlandSite>
  ): Promise<ApiResponse<WetlandSite>> {
    const response = await this.put<WetlandSite>(`/api/v1/sites/${siteId}`, updates);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'site.updated',
        timestamp: new Date().toISOString(),
        siteId: response.data.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Delete wetland site
   */
  async deleteWetlandSite(siteId: string): Promise<ApiResponse<void>> {
    const response = await this.delete<void>(`/api/v1/sites/${siteId}`);

    if (response.success) {
      await this.emitEvent({
        eventType: 'site.deleted',
        timestamp: new Date().toISOString(),
        siteId: siteId,
        data: { siteId },
      });
    }

    return response;
  }

  /**
   * List wetland sites with filters
   */
  async listWetlandSites(params?: {
    pagination?: PaginationParams;
    location?: LocationFilter;
    wetlandType?: WetlandType;
    protectionStatus?: ProtectionStatus;
  }): Promise<ApiResponse<PaginatedResponse<WetlandSite>>> {
    const queryParams = new URLSearchParams();

    if (params?.pagination) {
      Object.entries(params.pagination).forEach(([key, value]) => {
        if (value !== undefined) queryParams.append(key, String(value));
      });
    }

    if (params?.location) {
      Object.entries(params.location).forEach(([key, value]) => {
        if (value !== undefined && key !== 'boundingBox') {
          queryParams.append(key, String(value));
        }
      });
    }

    if (params?.wetlandType) {
      queryParams.append('wetlandType', params.wetlandType);
    }

    if (params?.protectionStatus) {
      queryParams.append('protectionStatus', params.protectionStatus);
    }

    return this.get<PaginatedResponse<WetlandSite>>(
      `/api/v1/sites?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Water Level & Hydrology Monitoring
  // ==========================================================================

  /**
   * Record water level measurement
   */
  async recordWaterLevel(
    measurement: Omit<WaterLevelMeasurement, 'measurementId'>
  ): Promise<ApiResponse<WaterLevelMeasurement>> {
    const response = await this.post<WaterLevelMeasurement>('/api/v1/hydrology/water-level', measurement);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'water.level.changed',
        timestamp: measurement.timestamp,
        siteId: measurement.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get water level data
   */
  async getWaterLevelData(params: {
    siteId: string;
    dateRange?: DateRangeFilter;
  }): Promise<ApiResponse<WaterLevelMeasurement[]>> {
    const queryParams = new URLSearchParams({ siteId: params.siteId });

    if (params.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    return this.get<WaterLevelMeasurement[]>(
      `/api/v1/hydrology/water-level?${queryParams.toString()}`
    );
  }

  /**
   * Record water quality measurement
   */
  async recordWaterQuality(
    quality: Omit<WaterQuality, 'measurementId'>
  ): Promise<ApiResponse<WaterQuality>> {
    const response = await this.post<WaterQuality>('/api/v1/hydrology/water-quality', quality);

    if (response.success && response.data) {
      // Alert if water quality is poor or very poor
      if (response.data.qualityGrade === 'POOR' || response.data.qualityGrade === 'VERY_POOR') {
        await this.emitEvent({
          eventType: 'water.quality.alert',
          timestamp: quality.timestamp,
          siteId: quality.siteId,
          data: response.data,
        });
      }
    }

    return response;
  }

  /**
   * Get water quality data
   */
  async getWaterQualityData(params: {
    siteId: string;
    dateRange?: DateRangeFilter;
  }): Promise<ApiResponse<WaterQuality[]>> {
    const queryParams = new URLSearchParams({ siteId: params.siteId });

    if (params.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    return this.get<WaterQuality[]>(
      `/api/v1/hydrology/water-quality?${queryParams.toString()}`
    );
  }

  /**
   * Submit hydrologic data
   */
  async submitHydrologicData(
    data: Omit<HydrologicData, 'monitoringId'>
  ): Promise<ApiResponse<HydrologicData>> {
    return this.post<HydrologicData>('/api/v1/hydrology/data', data);
  }

  /**
   * Get hydrologic analysis
   */
  async getHydrologicAnalysis(siteId: string, year?: number): Promise<ApiResponse<HydrologicData>> {
    const queryParams = new URLSearchParams({ siteId });
    if (year) queryParams.append('year', String(year));

    return this.get<HydrologicData>(
      `/api/v1/hydrology/analysis?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Vegetation Monitoring
  // ==========================================================================

  /**
   * Submit vegetation survey
   */
  async submitVegetationSurvey(
    survey: Omit<VegetationSurvey, 'surveyId'>
  ): Promise<ApiResponse<VegetationSurvey>> {
    const response = await this.post<VegetationSurvey>('/api/v1/vegetation/survey', survey);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'vegetation.survey.completed',
        timestamp: survey.timestamp,
        siteId: survey.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get vegetation surveys
   */
  async getVegetationSurveys(params: {
    siteId: string;
    dateRange?: DateRangeFilter;
  }): Promise<ApiResponse<VegetationSurvey[]>> {
    const queryParams = new URLSearchParams({ siteId: params.siteId });

    if (params.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    return this.get<VegetationSurvey[]>(
      `/api/v1/vegetation/surveys?${queryParams.toString()}`
    );
  }

  /**
   * Get vegetation composition analysis
   */
  async getVegetationComposition(siteId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/vegetation/${siteId}/composition`);
  }

  /**
   * Detect invasive species
   */
  async detectInvasiveSpecies(params: {
    siteId: string;
    vegetationType?: VegetationType;
  }): Promise<ApiResponse<any[]>> {
    const queryParams = new URLSearchParams({ siteId: params.siteId });
    if (params.vegetationType) {
      queryParams.append('vegetationType', params.vegetationType);
    }

    return this.get<any[]>(
      `/api/v1/vegetation/invasive?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Species Tracking
  // ==========================================================================

  /**
   * Record species observation
   */
  async recordSpeciesObservation(
    observation: Omit<WetlandSpeciesObservation, 'observationId'>
  ): Promise<ApiResponse<WetlandSpeciesObservation>> {
    const response = await this.post<WetlandSpeciesObservation>(
      '/api/v1/species/observation',
      observation
    );

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'species.observed',
        timestamp: observation.timestamp,
        siteId: observation.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get species observations
   */
  async getSpeciesObservations(params: {
    siteId?: string;
    scientificName?: string;
    taxonomicGroup?: string;
    dateRange?: DateRangeFilter;
    pagination?: PaginationParams;
  }): Promise<ApiResponse<PaginatedResponse<WetlandSpeciesObservation>>> {
    const queryParams = new URLSearchParams();

    if (params.siteId) queryParams.append('siteId', params.siteId);
    if (params.scientificName) queryParams.append('scientificName', params.scientificName);
    if (params.taxonomicGroup) queryParams.append('taxonomicGroup', params.taxonomicGroup);

    if (params.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    if (params.pagination) {
      Object.entries(params.pagination).forEach(([key, value]) => {
        if (value !== undefined) queryParams.append(key, String(value));
      });
    }

    return this.get<PaginatedResponse<WetlandSpeciesObservation>>(
      `/api/v1/species/observations?${queryParams.toString()}`
    );
  }

  /**
   * Submit avian assessment
   */
  async submitAvianAssessment(
    assessment: Omit<AvianAssessment, 'assessmentId'>
  ): Promise<ApiResponse<AvianAssessment>> {
    return this.post<AvianAssessment>('/api/v1/species/avian', assessment);
  }

  /**
   * Get avian community trends
   */
  async getAvianTrends(params: {
    siteId: string;
    dateRange: DateRangeFilter;
  }): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      siteId: params.siteId,
      startDate: params.dateRange.startDate,
      endDate: params.dateRange.endDate,
    });

    return this.get<any>(`/api/v1/species/avian/trends?${queryParams.toString()}`);
  }

  /**
   * List endangered species in wetland
   */
  async listEndangeredSpecies(siteId: string): Promise<ApiResponse<any[]>> {
    return this.get<any[]>(`/api/v1/species/${siteId}/endangered`);
  }

  // ==========================================================================
  // Habitat Assessment
  // ==========================================================================

  /**
   * Assess wetland condition
   */
  async assessWetlandCondition(
    condition: Omit<WetlandCondition, 'assessmentId'>
  ): Promise<ApiResponse<WetlandCondition>> {
    const response = await this.post<WetlandCondition>('/api/v1/habitat/condition', condition);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'condition.assessed',
        timestamp: condition.assessmentDate,
        siteId: condition.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get wetland condition history
   */
  async getConditionHistory(params: {
    siteId: string;
    dateRange?: DateRangeFilter;
  }): Promise<ApiResponse<WetlandCondition[]>> {
    const queryParams = new URLSearchParams({ siteId: params.siteId });

    if (params.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    return this.get<WetlandCondition[]>(
      `/api/v1/habitat/condition/history?${queryParams.toString()}`
    );
  }

  /**
   * Assess functional capacity
   */
  async assessFunctionalCapacity(
    assessment: Omit<FunctionalAssessment, 'assessmentId'>
  ): Promise<ApiResponse<FunctionalAssessment>> {
    return this.post<FunctionalAssessment>('/api/v1/habitat/functional', assessment);
  }

  /**
   * Get habitat quality index
   */
  async getHabitatQualityIndex(siteId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/habitat/${siteId}/quality-index`);
  }

  // ==========================================================================
  // Ecosystem Services Valuation
  // ==========================================================================

  /**
   * Value ecosystem services
   */
  async valueEcosystemServices(
    valuation: Omit<EcosystemServiceValuation, 'valuationId'>
  ): Promise<ApiResponse<EcosystemServiceValuation>> {
    return this.post<EcosystemServiceValuation>('/api/v1/services/valuation', valuation);
  }

  /**
   * Get ecosystem service value
   */
  async getServiceValuation(siteId: string): Promise<ApiResponse<EcosystemServiceValuation>> {
    return this.get<EcosystemServiceValuation>(`/api/v1/services/${siteId}/valuation`);
  }

  /**
   * List ecosystem services
   */
  async listEcosystemServices(params: {
    siteId: string;
    serviceType?: EcosystemServiceType;
  }): Promise<ApiResponse<any[]>> {
    const queryParams = new URLSearchParams({ siteId: params.siteId });
    if (params.serviceType) {
      queryParams.append('serviceType', params.serviceType);
    }

    return this.get<any[]>(`/api/v1/services/list?${queryParams.toString()}`);
  }

  /**
   * Compare ecosystem service values across sites
   */
  async compareServiceValues(siteIds: string[]): Promise<ApiResponse<any>> {
    return this.post<any>('/api/v1/services/compare', { siteIds });
  }

  // ==========================================================================
  // Restoration Planning & Management
  // ==========================================================================

  /**
   * Create restoration project
   */
  async createRestorationProject(
    project: Omit<RestorationProject, 'projectId'>
  ): Promise<ApiResponse<RestorationProject>> {
    const response = await this.post<RestorationProject>('/api/v1/restoration/projects', project);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'restoration.started',
        timestamp: new Date().toISOString(),
        siteId: project.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get restoration project
   */
  async getRestorationProject(projectId: string): Promise<ApiResponse<RestorationProject>> {
    return this.get<RestorationProject>(`/api/v1/restoration/projects/${projectId}`);
  }

  /**
   * Update restoration project
   */
  async updateRestorationProject(
    projectId: string,
    updates: Partial<RestorationProject>
  ): Promise<ApiResponse<RestorationProject>> {
    const response = await this.put<RestorationProject>(
      `/api/v1/restoration/projects/${projectId}`,
      updates
    );

    // Check if project was completed
    if (response.success && response.data && response.data.timeline.status === 'COMPLETED') {
      await this.emitEvent({
        eventType: 'restoration.completed',
        timestamp: new Date().toISOString(),
        siteId: response.data.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * List restoration projects
   */
  async listRestorationProjects(params?: {
    siteId?: string;
    restorationType?: RestorationType;
    status?: string;
  }): Promise<ApiResponse<RestorationProject[]>> {
    const queryParams = new URLSearchParams();

    if (params?.siteId) queryParams.append('siteId', params.siteId);
    if (params?.restorationType) queryParams.append('restorationType', params.restorationType);
    if (params?.status) queryParams.append('status', params.status);

    return this.get<RestorationProject[]>(
      `/api/v1/restoration/projects?${queryParams.toString()}`
    );
  }

  /**
   * Evaluate restoration success
   */
  async evaluateRestorationSuccess(projectId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/restoration/projects/${projectId}/success`);
  }

  /**
   * Record management action
   */
  async recordManagementAction(
    action: Omit<ManagementAction, 'actionId'>
  ): Promise<ApiResponse<ManagementAction>> {
    return this.post<ManagementAction>('/api/v1/management/actions', action);
  }

  /**
   * Get management actions
   */
  async getManagementActions(siteId: string): Promise<ApiResponse<ManagementAction[]>> {
    return this.get<ManagementAction[]>(`/api/v1/management/${siteId}/actions`);
  }

  // ==========================================================================
  // Threats & Risk Assessment
  // ==========================================================================

  /**
   * Identify threat
   */
  async identifyThreat(
    threat: Omit<WetlandThreat, 'threatId'>
  ): Promise<ApiResponse<WetlandThreat>> {
    const response = await this.post<WetlandThreat>('/api/v1/threats', threat);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'threat.identified',
        timestamp: threat.identifiedDate,
        siteId: threat.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get threats for site
   */
  async getThreats(siteId: string): Promise<ApiResponse<WetlandThreat[]>> {
    return this.get<WetlandThreat[]>(`/api/v1/threats/${siteId}`);
  }

  /**
   * Update threat mitigation
   */
  async updateThreatMitigation(
    threatId: string,
    mitigation: any
  ): Promise<ApiResponse<WetlandThreat>> {
    return this.put<WetlandThreat>(`/api/v1/threats/${threatId}/mitigation`, mitigation);
  }

  /**
   * Assess cumulative threats
   */
  async assessCumulativeThreats(siteId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/threats/${siteId}/cumulative`);
  }

  // ==========================================================================
  // Monitoring & Reporting
  // ==========================================================================

  /**
   * Create monitoring program
   */
  async createMonitoringProgram(
    program: Omit<MonitoringProgram, 'programId'>
  ): Promise<ApiResponse<MonitoringProgram>> {
    return this.post<MonitoringProgram>('/api/v1/monitoring/programs', program);
  }

  /**
   * Get monitoring program
   */
  async getMonitoringProgram(programId: string): Promise<ApiResponse<MonitoringProgram>> {
    return this.get<MonitoringProgram>(`/api/v1/monitoring/programs/${programId}`);
  }

  /**
   * Generate wetland health report
   */
  async generateHealthReport(params: {
    siteId: string;
    period: DateRangeFilter;
  }): Promise<ApiResponse<WetlandHealthReport>> {
    const response = await this.post<WetlandHealthReport>('/api/v1/reports/health', params);

    if (response.success && response.data) {
      await this.emitEvent({
        eventType: 'report.generated',
        timestamp: new Date().toISOString(),
        siteId: params.siteId,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get wetland health report
   */
  async getHealthReport(reportId: string): Promise<ApiResponse<WetlandHealthReport>> {
    return this.get<WetlandHealthReport>(`/api/v1/reports/${reportId}`);
  }

  /**
   * List reports for site
   */
  async listReports(siteId: string): Promise<ApiResponse<WetlandHealthReport[]>> {
    return this.get<WetlandHealthReport[]>(`/api/v1/reports/site/${siteId}`);
  }

  /**
   * Get wetland statistics dashboard
   */
  async getStatistics(params?: {
    siteId?: string;
    region?: string;
    dateRange?: DateRangeFilter;
  }): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams();

    if (params?.siteId) queryParams.append('siteId', params.siteId);
    if (params?.region) queryParams.append('region', params.region);
    if (params?.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    return this.get<any>(
      `/api/v1/analytics/statistics${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // Event Handling
  // ==========================================================================

  /**
   * Register event handler
   */
  on(eventType: WetlandEventType, handler: WetlandEventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  /**
   * Unregister event handler
   */
  off(eventType: WetlandEventType, handler: WetlandEventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  /**
   * Emit event to all registered handlers
   */
  private async emitEvent(event: WetlandEvent): Promise<void> {
    const handlers = this.eventHandlers.get(event.eventType);
    if (handlers) {
      for (const handler of handlers) {
        try {
          await handler(event);
        } catch (error) {
          if (this.config.debug) {
            console.error(`[WIA-WETLAND] Event handler error:`, error);
          }
        }
      }
    }
  }

  // ==========================================================================
  // HTTP Helper Methods
  // ==========================================================================

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<ApiResponse<T>> {
    const url = `${this.config.endpoint}${path}`;

    if (this.config.debug) {
      console.log(`[WIA-WETLAND] ${method} ${url}`, body || '');
    }

    try {
      const response = await fetch(url, {
        method,
        headers: this.headers,
        body: body ? JSON.stringify(body) : undefined,
        signal: AbortSignal.timeout(this.config.timeout),
      });

      const data = await response.json();

      if (!response.ok) {
        return {
          success: false,
          error: {
            code: `HTTP_${response.status}`,
            message: data.message || response.statusText,
            details: data,
          },
          metadata: {
            timestamp: new Date().toISOString(),
            version: '1.0.0',
          },
        };
      }

      return {
        success: true,
        data: data as T,
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };
    } catch (error) {
      if (this.config.debug) {
        console.error('[WIA-WETLAND] Request failed:', error);
      }

      return {
        success: false,
        error: {
          code: 'NETWORK_ERROR',
          message: error instanceof Error ? error.message : 'Unknown error',
          details: error,
        },
        metadata: {
          timestamp: new Date().toISOString(),
          version: '1.0.0',
        },
      };
    }
  }

  private async get<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('GET', path);
  }

  private async post<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('DELETE', path);
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate wetland health score
 */
export function calculateWetlandHealth(condition: WetlandCondition): number {
  const weights = {
    hydrology: 0.3,
    vegetation: 0.25,
    soil: 0.15,
    habitat: 0.3,
  };

  const scores = {
    hydrology: (
      condition.hydrology.waterRegimeIntegrity * 0.4 +
      condition.hydrology.surfaceWaterConnectivity * 0.3 +
      condition.hydrology.groundwaterConnectivity * 0.3
    ),
    vegetation: condition.vegetation.vegetationIntegrity,
    soil: condition.soil.hydricSoilIndicators ? 100 : 50,
    habitat: (
      condition.habitat.habitatDiversity * 0.3 +
      condition.habitat.refugiaAvailability * 0.3 +
      condition.habitat.connectivityScore * 0.4
    ),
  };

  return (
    scores.hydrology * weights.hydrology +
    scores.vegetation * weights.vegetation +
    scores.soil * weights.soil +
    scores.habitat * weights.habitat
  );
}

/**
 * Calculate restoration success rate
 */
export function calculateRestorationSuccess(project: RestorationProject): number {
  if (!project.outcomes || project.outcomes.length === 0) {
    return 0;
  }

  const totalSuccess = project.outcomes.reduce((sum, outcome) => {
    return sum + outcome.successRate;
  }, 0);

  return totalSuccess / project.outcomes.length;
}

/**
 * Classify wetland by Ramsar type
 */
export function getRamsarClassification(wetlandType: WetlandType): string {
  const ramsarCodes: Record<WetlandType, string> = {
    [WetlandType.MARSH]: 'Tp',
    [WetlandType.SWAMP]: 'Xf',
    [WetlandType.BOG]: 'U',
    [WetlandType.FEN]: 'U',
    [WetlandType.PEATLAND]: 'U',
    [WetlandType.FLOODPLAIN]: 'Ts',
    [WetlandType.RIPARIAN]: 'N',
    [WetlandType.LAKE]: 'O',
    [WetlandType.POND]: 'Tp',
    [WetlandType.SEASONAL_POOL]: 'Ts',
    [WetlandType.TIDAL_MARSH]: 'H',
    [WetlandType.SALT_MARSH]: 'H',
    [WetlandType.MANGROVE]: 'I',
    [WetlandType.MUDFLAT]: 'G',
    [WetlandType.ESTUARY]: 'F',
    [WetlandType.LAGOON]: 'J',
    [WetlandType.RICE_PADDY]: '3',
    [WetlandType.RESERVOIR]: '6',
    [WetlandType.AQUACULTURE_POND]: '5',
    [WetlandType.TREATMENT_WETLAND]: '9',
  };

  return ramsarCodes[wetlandType] || 'Unknown';
}

/**
 * Assess eutrophication level from nutrients
 */
export function assessEutrophication(waterQuality: WaterQuality): string {
  const totalP = waterQuality.nutrients.totalPhosphorus || 0;
  const totalN = waterQuality.nutrients.totalNitrogen || 0;

  if (totalP < 10 && totalN < 200) return 'OLIGOTROPHIC';
  if (totalP < 30 && totalN < 650) return 'MESOTROPHIC';
  if (totalP < 100 && totalN < 1500) return 'EUTROPHIC';
  return 'HYPEREUTROPHIC';
}

/**
 * Calculate vegetation diversity (Shannon index)
 */
export function calculateVegetationDiversity(survey: VegetationSurvey): number {
  const totalCover = survey.species.reduce((sum, sp) => sum + sp.cover, 0);
  if (totalCover === 0) return 0;

  return -1 * survey.species.reduce((sum, sp) => {
    const pi = sp.cover / totalCover;
    return sum + (pi > 0 ? pi * Math.log(pi) : 0);
  }, 0);
}

/**
 * Determine priority for restoration
 */
export function determineRestorationPriority(
  condition: WetlandCondition,
  threats: WetlandThreat[]
): 'CRITICAL' | 'HIGH' | 'MODERATE' | 'LOW' {
  const healthScore = calculateWetlandHealth(condition);
  const criticalThreats = threats.filter(t => t.assessment.severity === 'CRITICAL').length;
  const highThreats = threats.filter(t => t.assessment.severity === 'HIGH').length;

  if (healthScore < 40 || criticalThreats > 0) return 'CRITICAL';
  if (healthScore < 60 || highThreats > 1) return 'HIGH';
  if (healthScore < 75 || highThreats > 0) return 'MODERATE';
  return 'LOW';
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIAWetlandConservation SDK instance
 */
export function createWetlandConservationSDK(
  config: WetlandConservationSDKConfig
): WIAWetlandConservation {
  return new WIAWetlandConservation(config);
}

// ============================================================================
// Default Export
// ============================================================================

export default WIAWetlandConservation;
