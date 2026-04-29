/**
 * WIA-OCEAN_CONSERVATION SDK
 * Version 1.0
 * 弘益人間 (Benefit All Humanity)
 *
 * Comprehensive SDK for ocean conservation activities including:
 * - Marine species tracking and population monitoring
 * - Ecosystem health assessment (coral reefs, seagrass, mangroves)
 * - Marine Protected Area management
 * - Illegal fishing detection and enforcement
 * - Pollution tracking and response
 * - Conservation alerts and notifications
 */

import axios, { AxiosInstance, AxiosError } from 'axios';
import * as types from './types';

/**
 * Main WIA Ocean Conservation SDK Client
 */
export class OceanConservationSDK {
  private client: AxiosInstance;
  private config: types.WIAOceanConservationConfig;

  public species: SpeciesManager;
  public ecosystems: EcosystemManager;
  public mpas: MPAManager;
  public enforcement: EnforcementManager;
  public pollution: PollutionManager;
  public alerts: AlertManager;
  public reports: ReportManager;
  public citizen: CitizenScienceManager;

  constructor(config: types.WIAOceanConservationConfig) {
    this.config = {
      baseUrl: 'https://api.wia.org/ocean-conservation/v1',
      timeout: 30000,
      retries: 3,
      version: '1.0.0',
      ...config
    };

    this.client = axios.create({
      baseURL: this.config.baseUrl,
      timeout: this.config.timeout,
      headers: {
        'Authorization': `Bearer ${this.config.apiKey}`,
        'Content-Type': 'application/json',
        'X-WIA-Version': this.config.version || '1.0.0',
        'User-Agent': 'WIA-Ocean-Conservation-SDK/1.0.0'
      }
    });

    // Add request interceptor for retry logic
    this.setupInterceptors();

    // Initialize managers
    this.species = new SpeciesManager(this.client);
    this.ecosystems = new EcosystemManager(this.client);
    this.mpas = new MPAManager(this.client);
    this.enforcement = new EnforcementManager(this.client);
    this.pollution = new PollutionManager(this.client);
    this.alerts = new AlertManager(this.client);
    this.reports = new ReportManager(this.client);
    this.citizen = new CitizenScienceManager(this.client);
  }

  private setupInterceptors(): void {
    // Response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      async (error: AxiosError) => {
        const config: any = error.config;

        if (!config || !config.retries) {
          config.retries = 0;
        }

        if (config.retries < (this.config.retries || 3)) {
          config.retries += 1;
          const delay = Math.pow(2, config.retries) * 1000;
          await new Promise(resolve => setTimeout(resolve, delay));
          return this.client(config);
        }

        return Promise.reject(error);
      }
    );
  }

  /**
   * Test API connection
   */
  async healthCheck(): Promise<{ status: string; timestamp: string }> {
    const response = await this.client.get('/health');
    return response.data;
  }
}

/**
 * Species Tracking and Population Management
 */
export class SpeciesManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Submit a new species observation
   */
  async submitObservation(
    observation: Omit<types.SpeciesObservation, 'observationId'>
  ): Promise<types.SpeciesObservation> {
    const response = await this.client.post('/species/observations', observation);
    return response.data;
  }

  /**
   * Get species observations with filters
   */
  async getObservations(params?: {
    speciesCode?: string;
    startDate?: string;
    endDate?: string;
    bbox?: types.BoundingBox;
    verified?: boolean;
    limit?: number;
    offset?: number;
  }): Promise<types.PaginatedResponse<types.SpeciesObservation>> {
    const response = await this.client.get('/species/observations', { params });
    return response.data;
  }

  /**
   * Get a specific observation by ID
   */
  async getObservation(observationId: string): Promise<types.SpeciesObservation> {
    const response = await this.client.get(`/species/observations/${observationId}`);
    return response.data;
  }

  /**
   * Update an observation (for verified users)
   */
  async updateObservation(
    observationId: string,
    updates: Partial<types.SpeciesObservation>
  ): Promise<types.SpeciesObservation> {
    const response = await this.client.patch(`/species/observations/${observationId}`, updates);
    return response.data;
  }

  /**
   * Delete an observation
   */
  async deleteObservation(observationId: string): Promise<void> {
    await this.client.delete(`/species/observations/${observationId}`);
  }

  /**
   * Get population trends for a species
   */
  async getPopulationTrends(params: {
    speciesCode: string;
    region?: string;
    startYear?: number;
    endYear?: number;
  }): Promise<{
    speciesCode: string;
    region: string;
    currentPopulation: number;
    trend: string;
    trendRate: number;
    yearlyData: Array<{ year: number; population: number; confidence: number }>;
    threats: string[];
  }> {
    const response = await this.client.get(`/species/${params.speciesCode}/population`, {
      params: {
        region: params.region,
        startYear: params.startYear,
        endYear: params.endYear
      }
    });
    return response.data;
  }

  /**
   * Search species by name or code
   */
  async searchSpecies(query: string): Promise<Array<{
    speciesCode: string;
    commonName: string;
    scientificName: string;
    conservationStatus: types.ConservationStatus;
  }>> {
    const response = await this.client.get('/species/search', {
      params: { q: query }
    });
    return response.data;
  }

  /**
   * Get species by conservation status
   */
  async getByConservationStatus(
    status: types.ConservationStatus,
    region?: string
  ): Promise<types.PaginatedResponse<types.SpeciesObservation>> {
    const response = await this.client.get('/species/by-status', {
      params: { status, region }
    });
    return response.data;
  }

  /**
   * Verify an observation (expert only)
   */
  async verifyObservation(
    observationId: string,
    verified: boolean,
    notes?: string
  ): Promise<types.SpeciesObservation> {
    const response = await this.client.post(`/species/observations/${observationId}/verify`, {
      verified,
      notes
    });
    return response.data;
  }
}

/**
 * Ecosystem Health Monitoring
 */
export class EcosystemManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Submit coral reef assessment
   */
  async submitCoralAssessment(
    assessment: Omit<types.CoralReefAssessment, 'assessmentId'>
  ): Promise<types.CoralReefAssessment> {
    const response = await this.client.post('/ecosystems/coral-reefs/assessments', assessment);
    return response.data;
  }

  /**
   * Get reef health status
   */
  async getReefHealth(reefId: string): Promise<{
    reefId: string;
    reefName: string;
    currentHealth: {
      overallScore: number;
      coralCover: number;
      bleaching: types.BleachingLevel;
      diversity: number;
      trend: string;
    };
    historicalTrend: Array<{ date: string; score: number }>;
    alerts: Array<{
      type: string;
      severity: string;
      message: string;
    }>;
  }> {
    const response = await this.client.get(`/ecosystems/coral-reefs/${reefId}/health`);
    return response.data;
  }

  /**
   * Get all coral reef assessments in a region
   */
  async getCoralAssessments(params?: {
    region?: string;
    startDate?: string;
    endDate?: string;
    minCoralCover?: number;
    bleachingLevel?: types.BleachingLevel;
    limit?: number;
    offset?: number;
  }): Promise<types.PaginatedResponse<types.CoralReefAssessment>> {
    const response = await this.client.get('/ecosystems/coral-reefs/assessments', { params });
    return response.data;
  }

  /**
   * Monitor ocean acidification
   */
  async getAcidificationData(params?: {
    bbox?: types.BoundingBox;
    startDate?: string;
    endDate?: string;
    depth?: string; // e.g., "0-100"
  }): Promise<{
    measurements: types.OceanAcidificationData[];
    regionalSummary: {
      averagePH: number;
      trend: string;
      criticalAreas: number;
    };
  }> {
    const response = await this.client.get('/ecosystems/acidification', { params });
    return response.data;
  }

  /**
   * Submit ocean acidification measurement
   */
  async submitAcidificationData(
    data: Omit<types.OceanAcidificationData, 'measurementId'>
  ): Promise<types.OceanAcidificationData> {
    const response = await this.client.post('/ecosystems/acidification', data);
    return response.data;
  }

  /**
   * Assess coral bleaching risk
   */
  async assessBleachingRisk(params: {
    reefId: string;
    temperatureData: any;
    threshold: number;
  }): Promise<{
    reefId: string;
    riskLevel: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';
    dhw: number; // Degree Heating Weeks
    predictedBleaching: number; // percentage
    recommendedActions: string[];
  }> {
    const response = await this.client.post('/ecosystems/coral-reefs/bleaching-risk', params);
    return response.data;
  }

  /**
   * Submit seagrass assessment
   */
  async submitSeagrassAssessment(
    assessment: Omit<types.SeagrassAssessment, 'assessmentId'>
  ): Promise<types.SeagrassAssessment> {
    const response = await this.client.post('/ecosystems/seagrass/assessments', assessment);
    return response.data;
  }

  /**
   * Submit mangrove assessment
   */
  async submitMangroveAssessment(
    assessment: Omit<types.MangroveAssessment, 'assessmentId'>
  ): Promise<types.MangroveAssessment> {
    const response = await this.client.post('/ecosystems/mangroves/assessments', assessment);
    return response.data;
  }

  /**
   * Get blue carbon ecosystem data
   */
  async getBlueCarbon(params?: {
    region?: string;
    ecosystemType?: 'SEAGRASS' | 'MANGROVE' | 'SALT_MARSH';
  }): Promise<{
    totalArea: number; // hectares
    totalCarbonStock: number; // tonnes C
    annualSequestration: number; // tonnes C per year
    ecosystems: Array<{
      type: string;
      area: number;
      carbonStock: number;
    }>;
  }> {
    const response = await this.client.get('/ecosystems/blue-carbon', { params });
    return response.data;
  }
}

/**
 * Marine Protected Area Management
 */
export class MPAManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Get MPA information
   */
  async getMPA(mpaId: string): Promise<types.MarineProtectedArea> {
    const response = await this.client.get(`/mpas/${mpaId}`);
    return response.data;
  }

  /**
   * Get all MPAs with filters
   */
  async getAllMPAs(params?: {
    region?: string;
    protectionLevel?: types.ProtectionLevel;
    iucnCategory?: types.IUCNCategory;
    minArea?: number;
  }): Promise<types.PaginatedResponse<types.MarineProtectedArea>> {
    const response = await this.client.get('/mpas', { params });
    return response.data;
  }

  /**
   * Check if location is within MPA
   */
  async checkIntersection(location: types.Location): Promise<{
    result: boolean;
    mpaId?: string;
    mpaName?: string;
    protectionLevel?: types.ProtectionLevel;
    regulations?: types.MPARegulation[];
  }> {
    const response = await this.client.post('/mpas/check-intersection', { location });
    return response.data;
  }

  /**
   * Check MPA compliance for activity
   */
  async checkAuthorization(params: {
    mpaId: string;
    vesselId?: string;
    location: types.Location;
    activity: string;
    timestamp: string;
  }): Promise<{
    compliant: boolean;
    violation?: {
      type: string;
      severity: string;
      regulation: string;
      penalty: string;
    };
    enforcementNotified: boolean;
  }> {
    const response = await this.client.post(`/mpas/${params.mpaId}/check-compliance`, params);
    return response.data;
  }

  /**
   * Get MPA effectiveness metrics
   */
  async getEffectiveness(mpaId: string): Promise<types.MPAEffectiveness> {
    const response = await this.client.get(`/mpas/${mpaId}/effectiveness`);
    return response.data;
  }

  /**
   * Update MPA monitoring data
   */
  async updateMonitoring(mpaId: string, monitoring: {
    protocol?: string;
    reportingFrequency?: string;
    reportRecipients?: string[];
  }): Promise<types.MarineProtectedArea> {
    const response = await this.client.patch(`/mpas/${mpaId}/monitoring`, monitoring);
    return response.data;
  }

  /**
   * Submit patrol log
   */
  async submitPatrolLog(
    patrolLog: Omit<types.PatrolLog, 'patrolId'>
  ): Promise<types.PatrolLog> {
    const response = await this.client.post('/mpas/patrol-logs', patrolLog);
    return response.data;
  }
}

/**
 * Illegal Fishing Detection and Enforcement
 */
export class EnforcementManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Report suspicious fishing activity
   */
  async reportSuspiciousActivity(
    detection: Omit<types.IllegalFishingDetection, 'detectionId'>
  ): Promise<{
    detectionId: string;
    status: string;
    caseNumber: string;
    assignedTo: string;
    priority: string;
  }> {
    const response = await this.client.post('/enforcement/suspicious-activity', detection);
    return response.data;
  }

  /**
   * Track vessel activity
   */
  async trackVessel(vesselId: string, params?: {
    startDate?: string;
    endDate?: string;
    includeAISGaps?: boolean;
  }): Promise<{
    vesselId: string;
    trackingData: Array<{
      timestamp: string;
      location: types.Location;
      speed: number;
      heading: number;
    }>;
    suspiciousPatterns: Array<{
      pattern: string;
      duration?: number;
      location: string;
      riskScore: number;
    }>;
    mpaIntrusions: number;
    violations: number;
  }> {
    const response = await this.client.get(`/enforcement/vessels/${vesselId}/track`, { params });
    return response.data;
  }

  /**
   * Get vessel history
   */
  async getVesselHistory(vesselId: string): Promise<{
    vessel: types.Vessel;
    violationHistory: types.Violation[];
    riskScore: number;
    watchlist: boolean;
  }> {
    const response = await this.client.get(`/enforcement/vessels/${vesselId}/history`);
    return response.data;
  }

  /**
   * Report violation
   */
  async reportViolation(violation: {
    vesselId: string;
    violation: types.ViolationType;
    evidence: any;
  }): Promise<{
    caseNumber: string;
    status: string;
  }> {
    const response = await this.client.post('/enforcement/violations', violation);
    return response.data;
  }

  /**
   * Get active enforcement cases
   */
  async getActiveCases(params?: {
    region?: string;
    severity?: string;
    status?: string;
  }): Promise<types.PaginatedResponse<types.IllegalFishingDetection>> {
    const response = await this.client.get('/enforcement/cases', { params });
    return response.data;
  }

  /**
   * Link case to external enforcement system
   */
  async linkCase(params: {
    detectionId: string;
    externalCaseNumber: string;
    agency: string;
  }): Promise<void> {
    await this.client.post('/enforcement/link-case', params);
  }
}

/**
 * Pollution Tracking and Response
 */
export class PollutionManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Report pollution event
   */
  async reportEvent(
    event: Omit<types.PollutionEvent, 'pollutionEventId'>
  ): Promise<{
    pollutionEventId: string;
    status: string;
    cleanupTeamAssigned?: string;
    estimatedResponseTime?: string;
  }> {
    const response = await this.client.post('/pollution/events', event);
    return response.data;
  }

  /**
   * Get pollution events
   */
  async getEvents(params?: {
    eventType?: types.PollutionType;
    severity?: types.PollutionSeverity;
    region?: string;
    startDate?: string;
    endDate?: string;
  }): Promise<types.PaginatedResponse<types.PollutionEvent>> {
    const response = await this.client.get('/pollution/events', { params });
    return response.data;
  }

  /**
   * Get pollution hotspots
   */
  async getHotspots(params?: {
    type?: types.PollutionType;
    severity?: string;
    region?: string;
  }): Promise<{
    hotspots: Array<{
      location: types.Location;
      pollutionType: types.PollutionType;
      concentration: string;
      affectedArea: number;
      activeCleanup: boolean;
      wildlifeImpact: string;
    }>;
    totalHotspots: number;
    criticalHotspots: number;
  }> {
    const response = await this.client.get('/pollution/hotspots', { params });
    return response.data;
  }

  /**
   * Update cleanup response
   */
  async updateCleanup(
    eventId: string,
    cleanup: Partial<types.CleanupResponse>
  ): Promise<types.PollutionEvent> {
    const response = await this.client.patch(`/pollution/events/${eventId}/cleanup`, cleanup);
    return response.data;
  }

  /**
   * Track marine debris
   */
  async trackDebris(params: {
    location: types.Location;
    debrisType: string;
    quantity: number;
    photos?: string[];
  }): Promise<{
    debrisId: string;
    registered: boolean;
  }> {
    const response = await this.client.post('/pollution/debris/track', params);
    return response.data;
  }
}

/**
 * Conservation Alerts and Notifications
 */
export class AlertManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Create new alert
   */
  async create(
    alert: Omit<types.ConservationAlert, 'alertId' | 'issuedAt' | 'status'>
  ): Promise<types.ConservationAlert> {
    const response = await this.client.post('/alerts', alert);
    return response.data;
  }

  /**
   * Get active alerts
   */
  async getActive(params?: {
    region?: string;
    severity?: types.AlertSeverity;
    type?: types.AlertType;
  }): Promise<{
    activeAlerts: types.ConservationAlert[];
    totalAlerts: number;
    criticalAlerts: number;
  }> {
    const response = await this.client.get('/alerts/active', { params });
    return response.data;
  }

  /**
   * Subscribe to alerts
   */
  async subscribe(
    subscription: Omit<types.AlertSubscription, 'subscriptionId'>
  ): Promise<types.AlertSubscription> {
    const response = await this.client.post('/alerts/subscriptions', subscription);
    return response.data;
  }

  /**
   * Unsubscribe from alerts
   */
  async unsubscribe(subscriptionId: string): Promise<void> {
    await this.client.delete(`/alerts/subscriptions/${subscriptionId}`);
  }

  /**
   * Update alert status
   */
  async updateStatus(alertId: string, status: 'ACTIVE' | 'MONITORING' | 'RESOLVED' | 'EXPIRED'): Promise<types.ConservationAlert> {
    const response = await this.client.patch(`/alerts/${alertId}`, { status });
    return response.data;
  }

  /**
   * Add alert update
   */
  async addUpdate(alertId: string, message: string): Promise<types.ConservationAlert> {
    const response = await this.client.post(`/alerts/${alertId}/updates`, {
      message,
      timestamp: new Date().toISOString()
    });
    return response.data;
  }
}

/**
 * Reports and Analytics
 */
export class ReportManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Generate conservation report
   */
  async generate(request: types.ReportRequest): Promise<{
    reportId: string;
    status: string;
    estimatedCompletion: string;
    downloadUrl: string | null;
  }> {
    const response = await this.client.post('/reports/generate', request);
    return response.data;
  }

  /**
   * Get report status
   */
  async getStatus(reportId: string): Promise<types.GeneratedReport> {
    const response = await this.client.get(`/reports/${reportId}`);
    return response.data;
  }

  /**
   * Download report
   */
  async download(reportId: string): Promise<Blob> {
    const response = await this.client.get(`/reports/${reportId}/download`, {
      responseType: 'blob'
    });
    return response.data;
  }

  /**
   * List available reports
   */
  async list(params?: {
    reportType?: types.ReportType;
    startDate?: string;
    endDate?: string;
  }): Promise<types.PaginatedResponse<types.GeneratedReport>> {
    const response = await this.client.get('/reports', { params });
    return response.data;
  }
}

/**
 * Citizen Science Integration
 */
export class CitizenScienceManager {
  constructor(private client: AxiosInstance) {}

  /**
   * Register citizen scientist
   */
  async register(profile: {
    username: string;
    email: string;
    interests?: string[];
  }): Promise<types.CitizenScientist> {
    const response = await this.client.post('/citizen/register', profile);
    return response.data;
  }

  /**
   * Submit citizen observation
   */
  async submitObservation(
    observation: Omit<types.CitizenObservation, 'observationId' | 'needsVerification'>
  ): Promise<types.CitizenObservation> {
    const response = await this.client.post('/citizen/observations', observation);
    return response.data;
  }

  /**
   * Get citizen profile
   */
  async getProfile(userId: string): Promise<types.CitizenScientist> {
    const response = await this.client.get(`/citizen/profile/${userId}`);
    return response.data;
  }

  /**
   * Get leaderboard
   */
  async getLeaderboard(params?: {
    region?: string;
    period?: 'week' | 'month' | 'year' | 'all';
  }): Promise<types.CitizenScientist[]> {
    const response = await this.client.get('/citizen/leaderboard', { params });
    return response.data;
  }

  /**
   * Vote on observation quality
   */
  async voteObservation(observationId: string, vote: 'up' | 'down'): Promise<void> {
    await this.client.post(`/citizen/observations/${observationId}/vote`, { vote });
  }
}

// Export all types
export * from './types';

// Export SDK as default
export default OceanConservationSDK;

/**
 * Example Usage:
 *
 * ```typescript
 * import { OceanConservationSDK } from '@wia/ocean-conservation';
 *
 * const wia = new OceanConservationSDK({
 *   apiKey: 'your-api-key'
 * });
 *
 * // Submit species observation
 * const observation = await wia.species.submitObservation({
 *   speciesCode: 'IUCN-Chelonia-mydas',
 *   commonName: 'Green Sea Turtle',
 *   scientificName: 'Chelonia mydas',
 *   location: { latitude: -23.5505, longitude: -46.6333 },
 *   observationType: 'VISUAL',
 *   count: 1,
 *   timestamp: new Date().toISOString(),
 *   observer: {
 *     observerId: 'user-123',
 *     type: 'RESEARCHER'
 *   }
 * });
 *
 * // Check reef health
 * const health = await wia.ecosystems.getReefHealth('reef-uuid');
 * console.log(`Coral cover: ${health.currentHealth.coralCover}%`);
 *
 * // Monitor illegal fishing
 * const vessels = await wia.enforcement.trackVessel('IMO-1234567', {
 *   startDate: '2026-01-01',
 *   endDate: '2026-01-12'
 * });
 *
 * // Subscribe to alerts
 * await wia.alerts.subscribe({
 *   subscriberId: 'user-123',
 *   alertTypes: ['BLEACHING_EVENT', 'ILLEGAL_FISHING'],
 *   regions: ['CARIBBEAN_SEA'],
 *   severityThreshold: 'HIGH',
 *   deliveryMethod: 'EMAIL',
 *   email: 'user@example.com'
 * });
 * ```
 */
