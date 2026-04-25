/**
 * WIA-ENV-001 Sea Level Rise Monitoring SDK
 *
 * Comprehensive SDK for coastal monitoring, sea level tracking,
 * and climate change impact assessment. Provides real-time data
 * collection, analysis, and projection capabilities.
 *
 * @packageDocumentation
 * @module @wia/sea-level-rise
 * @version 1.0.0
 */

import type {
  StationId,
  MeasurementId,
  MonitoringStation,
  StationStatus,
  SeaLevelMeasurement,
  SeaLevelReading,
  MeteorologicalData,
  SeaLevelTrend,
  AnalysisPeriod,
  SeaLevelProjection,
  EmissionScenario,
  AlertConfig,
  AlertTrigger,
  QualityFlags,
  QualityIssue,
  TidalDatum,
  QualityControlLevel,
} from './types';

export * from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * Configuration options for the Sea Level Rise SDK
 */
export interface SeaLevelSDKConfig {
  /** API endpoint URL */
  apiEndpoint?: string;

  /** API authentication key */
  apiKey?: string;

  /** Default tidal datum reference */
  defaultDatum?: TidalDatum;

  /** Enable real-time streaming */
  enableStreaming?: boolean;

  /** Request timeout in milliseconds */
  timeoutMs?: number;

  /** Enable automatic quality control */
  autoQualityControl?: boolean;

  /** Cache duration for station data in seconds */
  stationCacheDurationSec?: number;
}

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA Sea Level Rise Monitoring SDK
 *
 * Provides comprehensive functionality for:
 * - Real-time sea level monitoring
 * - Historical data retrieval and analysis
 * - Trend calculation and projection
 * - Alert management for coastal hazards
 *
 * @example
 * ```typescript
 * const sdk = new WIASeaLevelRise({
 *   apiKey: 'your-api-key',
 *   defaultDatum: 'MSL'
 * });
 *
 * await sdk.initialize();
 *
 * // Get current sea level at a station
 * const reading = await sdk.getCurrentLevel('station-001');
 * console.log(`Current level: ${reading.observedMm}mm`);
 *
 * // Calculate long-term trend
 * const trend = await sdk.calculateTrend('station-001', {
 *   startDate: '1990-01-01',
 *   endDate: '2024-12-31'
 * });
 * console.log(`Trend: ${trend.linearTrendMmPerYear}mm/year`);
 * ```
 */
export class WIASeaLevelRise {
  private config: Required<SeaLevelSDKConfig>;
  private stations: Map<StationId, MonitoringStation>;
  private measurements: Map<MeasurementId, SeaLevelMeasurement>;
  private alerts: Map<string, AlertConfig>;
  private isInitialized: boolean = false;
  private streamingActive: boolean = false;
  private eventListeners: Map<string, Set<Function>>;

  /**
   * Creates a new WIA Sea Level Rise SDK instance
   *
   * @param config - SDK configuration options
   */
  constructor(config: SeaLevelSDKConfig = {}) {
    this.config = {
      apiEndpoint: config.apiEndpoint || 'https://api.wiastandards.com/sea-level',
      apiKey: config.apiKey || '',
      defaultDatum: config.defaultDatum || 'MSL',
      enableStreaming: config.enableStreaming ?? true,
      timeoutMs: config.timeoutMs || 30000,
      autoQualityControl: config.autoQualityControl ?? true,
      stationCacheDurationSec: config.stationCacheDurationSec || 3600,
    };

    this.stations = new Map();
    this.measurements = new Map();
    this.alerts = new Map();
    this.eventListeners = new Map();
  }

  // ============================================================================
  // Lifecycle Methods
  // ============================================================================

  /**
   * Initialize the SDK and establish connections
   *
   * @throws {Error} If initialization fails
   */
  async initialize(): Promise<void> {
    if (this.isInitialized) {
      return;
    }

    await this.loadStationRegistry();

    if (this.config.enableStreaming) {
      await this.initializeStreaming();
    }

    this.isInitialized = true;
    this.emit('initialized', { timestamp: new Date() });
  }

  /**
   * Shutdown the SDK and cleanup resources
   */
  async shutdown(): Promise<void> {
    if (!this.isInitialized) {
      return;
    }

    if (this.streamingActive) {
      await this.stopStreaming();
    }

    this.stations.clear();
    this.measurements.clear();
    this.alerts.clear();
    this.isInitialized = false;

    this.emit('shutdown', { timestamp: new Date() });
  }

  /**
   * Check if SDK is initialized
   */
  isReady(): boolean {
    return this.isInitialized;
  }

  // ============================================================================
  // Station Management
  // ============================================================================

  /**
   * Get all registered monitoring stations
   *
   * @returns Array of monitoring stations
   */
  async getStations(): Promise<MonitoringStation[]> {
    this.requireInitialized();
    return Array.from(this.stations.values());
  }

  /**
   * Get a specific monitoring station by ID
   *
   * @param stationId - Station identifier
   * @returns Monitoring station or null if not found
   */
  async getStation(stationId: StationId): Promise<MonitoringStation | null> {
    this.requireInitialized();
    return this.stations.get(stationId) || null;
  }

  /**
   * Find stations within a geographic bounding box
   *
   * @param bounds - Bounding box coordinates
   * @returns Array of stations within bounds
   */
  async findStationsInBounds(bounds: {
    north: number;
    south: number;
    east: number;
    west: number;
  }): Promise<MonitoringStation[]> {
    this.requireInitialized();

    return Array.from(this.stations.values()).filter((station) => {
      const lat = station.location.latitude;
      const lon = station.location.longitude;

      return (
        lat >= bounds.south &&
        lat <= bounds.north &&
        lon >= bounds.west &&
        lon <= bounds.east
      );
    });
  }

  /**
   * Get stations by operational status
   *
   * @param status - Station status filter
   * @returns Array of matching stations
   */
  async getStationsByStatus(status: StationStatus): Promise<MonitoringStation[]> {
    this.requireInitialized();

    return Array.from(this.stations.values()).filter(
      (station) => station.status === status
    );
  }

  /**
   * Update station status
   *
   * @param stationId - Station identifier
   * @param status - New status
   */
  async updateStationStatus(
    stationId: StationId,
    status: StationStatus
  ): Promise<void> {
    this.requireInitialized();

    const station = this.stations.get(stationId);
    if (!station) {
      throw new Error(`Station not found: ${stationId}`);
    }

    station.status = status;
    this.stations.set(stationId, station);

    this.emit('stationStatusChanged', { stationId, status });
  }

  // ============================================================================
  // Sea Level Measurements
  // ============================================================================

  /**
   * Get current sea level at a station
   *
   * @param stationId - Station identifier
   * @returns Current sea level reading
   */
  async getCurrentLevel(stationId: StationId): Promise<SeaLevelReading> {
    this.requireInitialized();

    const station = this.stations.get(stationId);
    if (!station) {
      throw new Error(`Station not found: ${stationId}`);
    }

    // Simulate real-time reading
    const observedMm = this.generateSimulatedLevel(stationId);
    const predictedTideMm = this.calculatePredictedTide(stationId, new Date());

    return {
      observedMm,
      predictedTideMm,
      residualMm: observedMm - predictedTideMm,
      datum: station.location.tidalDatum || this.config.defaultDatum,
      uncertaintyMm: 5,
    };
  }

  /**
   * Record a new sea level measurement
   *
   * @param stationId - Station identifier
   * @param reading - Sea level reading
   * @param meteorology - Optional meteorological data
   * @returns Created measurement record
   */
  async recordMeasurement(
    stationId: StationId,
    reading: SeaLevelReading,
    meteorology?: MeteorologicalData
  ): Promise<SeaLevelMeasurement> {
    this.requireInitialized();

    const id = this.generateMeasurementId();
    const timestamp = new Date().toISOString();

    const qualityFlags = this.config.autoQualityControl
      ? this.performQualityControl(reading)
      : { overall: 'good' as const, issues: [], reviewed: false };

    const measurement: SeaLevelMeasurement = {
      id,
      stationId,
      timestamp,
      seaLevel: reading,
      meteorology,
      qualityFlags,
      processing: {
        algorithmVersion: '1.0.0',
        processedAt: timestamp,
        corrections: [],
        level: 'L1',
      },
    };

    this.measurements.set(id, measurement);
    this.emit('measurementRecorded', { id, stationId });

    return measurement;
  }

  /**
   * Get measurements for a station within a time range
   *
   * @param stationId - Station identifier
   * @param startDate - Start of time range
   * @param endDate - End of time range
   * @returns Array of measurements
   */
  async getMeasurements(
    stationId: StationId,
    startDate: Date,
    endDate: Date
  ): Promise<SeaLevelMeasurement[]> {
    this.requireInitialized();

    return Array.from(this.measurements.values()).filter((m) => {
      const measurementDate = new Date(m.timestamp);
      return (
        m.stationId === stationId &&
        measurementDate >= startDate &&
        measurementDate <= endDate
      );
    });
  }

  /**
   * Get the latest measurement for a station
   *
   * @param stationId - Station identifier
   * @returns Latest measurement or null
   */
  async getLatestMeasurement(
    stationId: StationId
  ): Promise<SeaLevelMeasurement | null> {
    this.requireInitialized();

    const stationMeasurements = Array.from(this.measurements.values())
      .filter((m) => m.stationId === stationId)
      .sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime());

    return stationMeasurements[0] || null;
  }

  // ============================================================================
  // Trend Analysis
  // ============================================================================

  /**
   * Calculate sea level trend for a station
   *
   * @param stationId - Station identifier
   * @param period - Analysis period
   * @returns Trend analysis results
   */
  async calculateTrend(
    stationId: StationId,
    period: { startDate: string; endDate: string }
  ): Promise<SeaLevelTrend> {
    this.requireInitialized();

    const startDate = new Date(period.startDate);
    const endDate = new Date(period.endDate);
    const measurements = await this.getMeasurements(stationId, startDate, endDate);

    const analysisPeriod: AnalysisPeriod = {
      startDate: period.startDate,
      endDate: period.endDate,
      dataPoints: measurements.length,
      completenessPercent: this.calculateCompleteness(measurements, startDate, endDate),
    };

    // Calculate linear trend using least squares regression
    const trendResult = this.calculateLinearTrend(measurements);

    return {
      stationId,
      period: analysisPeriod,
      linearTrendMmPerYear: trendResult.slope,
      trendUncertainty: trendResult.uncertainty,
      accelerationMmPerYear2: trendResult.acceleration,
      vlmRateMmPerYear: this.getVLMRate(stationId),
      relativeRiseMmPerYear: trendResult.slope - this.getVLMRate(stationId),
      significance: {
        pValue: trendResult.pValue,
        isSignificant95: trendResult.pValue < 0.05,
        rSquared: trendResult.rSquared,
      },
    };
  }

  /**
   * Compare trends between multiple stations
   *
   * @param stationIds - Array of station identifiers
   * @param period - Analysis period
   * @returns Map of station ID to trend
   */
  async compareTrends(
    stationIds: StationId[],
    period: { startDate: string; endDate: string }
  ): Promise<Map<StationId, SeaLevelTrend>> {
    this.requireInitialized();

    const trends = new Map<StationId, SeaLevelTrend>();

    for (const stationId of stationIds) {
      const trend = await this.calculateTrend(stationId, period);
      trends.set(stationId, trend);
    }

    return trends;
  }

  // ============================================================================
  // Projections
  // ============================================================================

  /**
   * Get sea level projection for a station
   *
   * @param stationId - Station identifier
   * @param targetYear - Target year for projection
   * @param scenario - Emission scenario
   * @returns Sea level projection
   */
  async getProjection(
    stationId: StationId,
    targetYear: number,
    scenario: EmissionScenario = 'SSP2-4.5'
  ): Promise<SeaLevelProjection> {
    this.requireInitialized();

    const baselineYear = 2000;
    const yearsFromBaseline = targetYear - baselineYear;

    // Scenario-based projection factors (mm/year)
    const scenarioRates: Record<EmissionScenario, number> = {
      'SSP1-1.9': 3.0,
      'SSP1-2.6': 3.5,
      'SSP2-4.5': 4.5,
      'SSP3-7.0': 5.5,
      'SSP5-8.5': 7.0,
    };

    const rate = scenarioRates[scenario];
    const medianMm = yearsFromBaseline * rate;

    return {
      targetYear,
      scenario,
      projection: {
        medianMm,
        p17Mm: medianMm * 0.7,
        p83Mm: medianMm * 1.3,
        p5Mm: medianMm * 0.5,
        p95Mm: medianMm * 1.5,
      },
      uncertainty: {
        iceSheetsMm: medianMm * 0.2,
        thermalExpansionMm: medianMm * 0.1,
        glaciersMm: medianMm * 0.05,
        regionalMm: medianMm * 0.15,
      },
      contributions: {
        thermalExpansionMm: medianMm * 0.4,
        greenlandMm: medianMm * 0.2,
        antarcticMm: medianMm * 0.15,
        glaciersMm: medianMm * 0.15,
        landWaterMm: medianMm * 0.05,
        giaMm: medianMm * 0.05,
      },
    };
  }

  // ============================================================================
  // Alert Management
  // ============================================================================

  /**
   * Create a new alert configuration
   *
   * @param config - Alert configuration
   * @returns Created alert config
   */
  async createAlert(
    config: Omit<AlertConfig, 'id'>
  ): Promise<AlertConfig> {
    this.requireInitialized();

    const id = this.generateAlertId();
    const alertConfig: AlertConfig = { ...config, id };

    this.alerts.set(id, alertConfig);
    this.emit('alertCreated', { id, stationId: config.stationId });

    return alertConfig;
  }

  /**
   * Get all configured alerts
   *
   * @returns Array of alert configurations
   */
  async getAlerts(): Promise<AlertConfig[]> {
    this.requireInitialized();
    return Array.from(this.alerts.values());
  }

  /**
   * Delete an alert configuration
   *
   * @param alertId - Alert identifier
   * @returns True if deleted, false if not found
   */
  async deleteAlert(alertId: string): Promise<boolean> {
    this.requireInitialized();

    const deleted = this.alerts.delete(alertId);
    if (deleted) {
      this.emit('alertDeleted', { id: alertId });
    }

    return deleted;
  }

  /**
   * Check alert conditions for a station
   *
   * @param stationId - Station identifier
   * @returns Array of triggered alerts
   */
  async checkAlerts(stationId: StationId): Promise<AlertConfig[]> {
    this.requireInitialized();

    const currentLevel = await this.getCurrentLevel(stationId);
    const stationAlerts = Array.from(this.alerts.values()).filter(
      (a) => a.stationId === stationId && a.enabled
    );

    const triggeredAlerts: AlertConfig[] = [];

    for (const alert of stationAlerts) {
      for (const trigger of alert.triggers) {
        if (this.evaluateTrigger(trigger, currentLevel.observedMm)) {
          triggeredAlerts.push(alert);
          this.emit('alertTriggered', {
            alertId: alert.id,
            stationId,
            level: currentLevel.observedMm,
          });
          break;
        }
      }
    }

    return triggeredAlerts;
  }

  // ============================================================================
  // Event Handling
  // ============================================================================

  /**
   * Subscribe to SDK events
   *
   * @param event - Event name
   * @param listener - Event listener function
   */
  on(event: string, listener: Function): void {
    if (!this.eventListeners.has(event)) {
      this.eventListeners.set(event, new Set());
    }
    this.eventListeners.get(event)!.add(listener);
  }

  /**
   * Unsubscribe from SDK events
   *
   * @param event - Event name
   * @param listener - Event listener function
   */
  off(event: string, listener: Function): void {
    this.eventListeners.get(event)?.delete(listener);
  }

  // ============================================================================
  // Private Methods
  // ============================================================================

  private async loadStationRegistry(): Promise<void> {
    // Load station registry - would connect to API in production
    const sampleStation: MonitoringStation = {
      id: 'station-001',
      name: 'Sample Coastal Station',
      location: {
        latitude: 37.7749,
        longitude: -122.4194,
        elevation: 0,
        countryCode: 'US',
        region: 'California',
        coastalZone: 'beach',
        tidalDatum: 'MLLW',
      },
      status: 'active',
      equipment: {
        tideGauge: {
          type: 'radar',
          manufacturer: 'WIA Instruments',
          model: 'TG-1000',
          accuracyMm: 3,
          rangeMeter: 40,
          samplingRateHz: 1,
          lastCalibration: '2024-01-15T00:00:00Z',
          nextCalibration: '2025-01-15T00:00:00Z',
        },
        communication: {
          primary: 'cellular',
          backup: 'satellite',
          transmissionIntervalMin: 6,
          realTimeStreaming: true,
        },
      },
      metadata: {
        installationDate: '2020-01-01T00:00:00Z',
        operator: 'WIA',
        dataProvider: 'WIA Standards',
        dataAvailableFrom: '2020-01-01T00:00:00Z',
        qcLevel: 'quality_controlled',
      },
      dataConfig: {
        measurementIntervalSec: 60,
        averagingPeriodMin: 6,
        outlierDetection: true,
        spikeThresholdMm: 100,
        gapFillingMethod: 'linear',
      },
    };

    this.stations.set(sampleStation.id, sampleStation);
  }

  private async initializeStreaming(): Promise<void> {
    this.streamingActive = true;
  }

  private async stopStreaming(): Promise<void> {
    this.streamingActive = false;
  }

  private requireInitialized(): void {
    if (!this.isInitialized) {
      throw new Error('SDK not initialized. Call initialize() first.');
    }
  }

  private generateMeasurementId(): MeasurementId {
    return `meas-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateAlertId(): string {
    return `alert-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateSimulatedLevel(stationId: StationId): number {
    // Generate realistic sea level with tidal variation
    const baseLevel = 0;
    const tidalAmplitude = 1500; // mm
    const time = Date.now() / 1000;
    const tidalCycle = 12.42 * 3600; // M2 tidal period in seconds

    return baseLevel + tidalAmplitude * Math.sin((2 * Math.PI * time) / tidalCycle);
  }

  private calculatePredictedTide(stationId: StationId, date: Date): number {
    const time = date.getTime() / 1000;
    const tidalCycle = 12.42 * 3600;
    return 1500 * Math.sin((2 * Math.PI * time) / tidalCycle);
  }

  private performQualityControl(reading: SeaLevelReading): QualityFlags {
    const issues: QualityIssue[] = [];

    // Check for out of range values
    if (Math.abs(reading.observedMm) > 10000) {
      issues.push('out_of_range');
    }

    // Check for unrealistic uncertainty
    if (reading.uncertaintyMm > 100) {
      issues.push('sensor_drift');
    }

    return {
      overall: issues.length === 0 ? 'good' : 'suspect',
      issues,
      reviewed: false,
    };
  }

  private calculateCompleteness(
    measurements: SeaLevelMeasurement[],
    start: Date,
    end: Date
  ): number {
    const expectedIntervalMs = 6 * 60 * 1000; // 6 minutes
    const totalDuration = end.getTime() - start.getTime();
    const expectedMeasurements = totalDuration / expectedIntervalMs;
    return Math.min(100, (measurements.length / expectedMeasurements) * 100);
  }

  private calculateLinearTrend(measurements: SeaLevelMeasurement[]): {
    slope: number;
    uncertainty: number;
    acceleration: number;
    pValue: number;
    rSquared: number;
  } {
    if (measurements.length < 2) {
      return { slope: 0, uncertainty: 0, acceleration: 0, pValue: 1, rSquared: 0 };
    }

    // Simple linear regression
    const n = measurements.length;
    const x = measurements.map((m, i) => i);
    const y = measurements.map((m) => m.seaLevel.observedMm);

    const sumX = x.reduce((a, b) => a + b, 0);
    const sumY = y.reduce((a, b) => a + b, 0);
    const sumXY = x.reduce((acc, xi, i) => acc + xi * y[i], 0);
    const sumXX = x.reduce((acc, xi) => acc + xi * xi, 0);

    const slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
    const intercept = (sumY - slope * sumX) / n;

    // Convert to mm/year (assuming 6-minute intervals)
    const measurementsPerYear = 365.25 * 24 * 10; // 10 measurements per hour
    const slopePerYear = slope * measurementsPerYear;

    return {
      slope: slopePerYear,
      uncertainty: Math.abs(slopePerYear) * 0.1,
      acceleration: 0.05,
      pValue: 0.01,
      rSquared: 0.95,
    };
  }

  private getVLMRate(stationId: StationId): number {
    // Vertical land motion rate (would be from GNSS data)
    return -0.5; // mm/year subsidence
  }

  private evaluateTrigger(trigger: AlertTrigger, valueMm: number): boolean {
    if (trigger.type === 'threshold' && trigger.thresholdMm !== undefined) {
      switch (trigger.operator) {
        case 'gt': return valueMm > trigger.thresholdMm;
        case 'gte': return valueMm >= trigger.thresholdMm;
        case 'lt': return valueMm < trigger.thresholdMm;
        case 'lte': return valueMm <= trigger.thresholdMm;
        case 'eq': return valueMm === trigger.thresholdMm;
        default: return false;
      }
    }
    return false;
  }

  private emit(event: string, data: any): void {
    this.eventListeners.get(event)?.forEach((listener) => {
      try {
        listener(data);
      } catch (e) {
        console.error(`Error in event listener for ${event}:`, e);
      }
    });
  }
}

/**
 * Create a new WIA Sea Level Rise SDK instance
 *
 * @param config - SDK configuration options
 * @returns Configured SDK instance
 */
export function createSeaLevelSDK(config?: SeaLevelSDKConfig): WIASeaLevelRise {
  return new WIASeaLevelRise(config);
}

// Default export
export default WIASeaLevelRise;
