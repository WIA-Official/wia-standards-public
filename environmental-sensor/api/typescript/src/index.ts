/**
 * WIA-ENE-035: Environmental Sensor Network Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  SensorDevice,
  SensorReading,
  SensorNetwork,
  Gateway,
  CalibrationRecord,
  MaintenanceRecord,
  AlertEvent,
  AlertConfig,
  AggregatedData,
  EdgeProcessingRule,
  AnomalyDetection,
  DataValidation,
  NetworkReport,
  KPIDashboard,
  ApiResponse,
  PaginatedResponse,
  PaginationParams,
  DateRangeFilter,
  SensorFilter,
  SensorType,
  SensorStatus,
  IoTProtocol,
  DataQuality,
  AlertSeverity,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface EnvironmentalSensorSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
}

// ============================================================================
// SDK Client
// ============================================================================

export class EnvironmentalSensorSDK {
  private config: Required<EnvironmentalSensorSDKConfig>;
  private headers: Record<string, string>;

  constructor(config: EnvironmentalSensorSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'ENE-035',
      'X-WIA-Version': '1.0.0',
    };
  }

  // ==========================================================================
  // Sensor Device Management APIs
  // ==========================================================================

  /**
   * Register a new sensor device
   */
  async registerSensor(
    sensor: Omit<SensorDevice, 'status' | 'installationDate'>
  ): Promise<ApiResponse<SensorDevice>> {
    return this.post<SensorDevice>('/api/v1/sensor/register', sensor);
  }

  /**
   * Get sensor device by ID
   */
  async getSensor(sensorId: string): Promise<ApiResponse<SensorDevice>> {
    return this.get<SensorDevice>(`/api/v1/sensor/${sensorId}`);
  }

  /**
   * Update sensor configuration
   */
  async updateSensor(
    sensorId: string,
    updates: Partial<SensorDevice>
  ): Promise<ApiResponse<SensorDevice>> {
    return this.put<SensorDevice>(`/api/v1/sensor/${sensorId}`, updates);
  }

  /**
   * Delete sensor device
   */
  async deleteSensor(sensorId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/sensor/${sensorId}`);
  }

  /**
   * List sensors with filters
   */
  async listSensors(params?: {
    pagination?: PaginationParams;
    filter?: SensorFilter;
  }): Promise<ApiResponse<PaginatedResponse<SensorDevice>>> {
    const queryParams = new URLSearchParams();

    if (params?.pagination) {
      Object.entries(params.pagination).forEach(([key, value]) => {
        if (value !== undefined) queryParams.append(key, String(value));
      });
    }

    if (params?.filter) {
      if (params.filter.sensorIds) {
        queryParams.append('sensorIds', params.filter.sensorIds.join(','));
      }
      if (params.filter.sensorTypes) {
        queryParams.append('sensorTypes', params.filter.sensorTypes.join(','));
      }
      if (params.filter.status) {
        queryParams.append('status', params.filter.status.join(','));
      }
      if (params.filter.location?.boundingBox) {
        queryParams.append('bbox', JSON.stringify(params.filter.location.boundingBox));
      }
    }

    return this.get<PaginatedResponse<SensorDevice>>(
      `/api/v1/sensors?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Sensor Reading APIs
  // ==========================================================================

  /**
   * Submit sensor reading
   */
  async submitReading(
    reading: Omit<SensorReading, 'readingId' | 'timestamp'>
  ): Promise<ApiResponse<SensorReading>> {
    return this.post<SensorReading>('/api/v1/reading/submit', reading);
  }

  /**
   * Get reading by ID
   */
  async getReading(readingId: string): Promise<ApiResponse<SensorReading>> {
    return this.get<SensorReading>(`/api/v1/reading/${readingId}`);
  }

  /**
   * Get latest reading for sensor
   */
  async getLatestReading(sensorId: string): Promise<ApiResponse<SensorReading>> {
    return this.get<SensorReading>(`/api/v1/sensor/${sensorId}/latest`);
  }

  /**
   * List readings with filters
   */
  async listReadings(params?: {
    sensorId?: string;
    pagination?: PaginationParams;
    dateRange?: DateRangeFilter;
    dataQuality?: DataQuality;
  }): Promise<ApiResponse<PaginatedResponse<SensorReading>>> {
    const queryParams = new URLSearchParams();

    if (params?.sensorId) queryParams.append('sensorId', params.sensorId);

    if (params?.pagination) {
      Object.entries(params.pagination).forEach(([key, value]) => {
        if (value !== undefined) queryParams.append(key, String(value));
      });
    }

    if (params?.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }

    if (params?.dataQuality) {
      queryParams.append('dataQuality', params.dataQuality);
    }

    return this.get<PaginatedResponse<SensorReading>>(
      `/api/v1/readings?${queryParams.toString()}`
    );
  }

  /**
   * Get aggregated data
   */
  async getAggregatedData(params: {
    sensorId: string;
    parameter: string;
    interval: 'minute' | 'hour' | 'day' | 'week' | 'month';
    dateRange: DateRangeFilter;
  }): Promise<ApiResponse<AggregatedData>> {
    const queryParams = new URLSearchParams({
      sensorId: params.sensorId,
      parameter: params.parameter,
      interval: params.interval,
      startDate: params.dateRange.startDate,
      endDate: params.dateRange.endDate,
    });

    return this.get<AggregatedData>(
      `/api/v1/data/aggregate?${queryParams.toString()}`
    );
  }

  // ==========================================================================
  // Calibration APIs
  // ==========================================================================

  /**
   * Submit calibration record
   */
  async submitCalibration(
    calibration: Omit<CalibrationRecord, 'calibrationId' | 'calibrationDate'>
  ): Promise<ApiResponse<CalibrationRecord>> {
    return this.post<CalibrationRecord>('/api/v1/calibration/submit', calibration);
  }

  /**
   * Get calibration record
   */
  async getCalibration(calibrationId: string): Promise<ApiResponse<CalibrationRecord>> {
    return this.get<CalibrationRecord>(`/api/v1/calibration/${calibrationId}`);
  }

  /**
   * Get calibration history for sensor
   */
  async getCalibrationHistory(sensorId: string): Promise<ApiResponse<CalibrationRecord[]>> {
    return this.get<CalibrationRecord[]>(`/api/v1/sensor/${sensorId}/calibration-history`);
  }

  /**
   * Get sensors due for calibration
   */
  async getSensorsDueForCalibration(daysAhead?: number): Promise<ApiResponse<SensorDevice[]>> {
    const queryParams = new URLSearchParams();
    if (daysAhead) queryParams.append('daysAhead', String(daysAhead));

    return this.get<SensorDevice[]>(
      `/api/v1/calibration/due${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // Maintenance APIs
  // ==========================================================================

  /**
   * Schedule maintenance
   */
  async scheduleMaintenance(
    maintenance: Omit<MaintenanceRecord, 'maintenanceId' | 'status'>
  ): Promise<ApiResponse<MaintenanceRecord>> {
    return this.post<MaintenanceRecord>('/api/v1/maintenance/schedule', maintenance);
  }

  /**
   * Update maintenance record
   */
  async updateMaintenance(
    maintenanceId: string,
    updates: Partial<MaintenanceRecord>
  ): Promise<ApiResponse<MaintenanceRecord>> {
    return this.put<MaintenanceRecord>(`/api/v1/maintenance/${maintenanceId}`, updates);
  }

  /**
   * Get maintenance record
   */
  async getMaintenance(maintenanceId: string): Promise<ApiResponse<MaintenanceRecord>> {
    return this.get<MaintenanceRecord>(`/api/v1/maintenance/${maintenanceId}`);
  }

  /**
   * Get maintenance history for sensor
   */
  async getMaintenanceHistory(sensorId: string): Promise<ApiResponse<MaintenanceRecord[]>> {
    return this.get<MaintenanceRecord[]>(`/api/v1/sensor/${sensorId}/maintenance-history`);
  }

  /**
   * Get upcoming maintenance tasks
   */
  async getUpcomingMaintenance(daysAhead?: number): Promise<ApiResponse<MaintenanceRecord[]>> {
    const queryParams = new URLSearchParams();
    if (daysAhead) queryParams.append('daysAhead', String(daysAhead));

    return this.get<MaintenanceRecord[]>(
      `/api/v1/maintenance/upcoming${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // Alert & Threshold APIs
  // ==========================================================================

  /**
   * Configure alert for sensor
   */
  async configureAlert(
    alert: Omit<AlertConfig, 'alertId'>
  ): Promise<ApiResponse<AlertConfig>> {
    return this.post<AlertConfig>('/api/v1/alert/configure', alert);
  }

  /**
   * Update alert configuration
   */
  async updateAlert(
    alertId: string,
    updates: Partial<AlertConfig>
  ): Promise<ApiResponse<AlertConfig>> {
    return this.put<AlertConfig>(`/api/v1/alert/${alertId}`, updates);
  }

  /**
   * Get alert configuration
   */
  async getAlertConfig(alertId: string): Promise<ApiResponse<AlertConfig>> {
    return this.get<AlertConfig>(`/api/v1/alert/${alertId}`);
  }

  /**
   * List active alerts
   */
  async listActiveAlerts(params?: {
    sensorId?: string;
    severity?: AlertSeverity;
    acknowledged?: boolean;
  }): Promise<ApiResponse<AlertEvent[]>> {
    const queryParams = new URLSearchParams();
    if (params?.sensorId) queryParams.append('sensorId', params.sensorId);
    if (params?.severity) queryParams.append('severity', params.severity);
    if (params?.acknowledged !== undefined) {
      queryParams.append('acknowledged', String(params.acknowledged));
    }

    return this.get<AlertEvent[]>(
      `/api/v1/alerts/active${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Acknowledge alert
   */
  async acknowledgeAlert(
    eventId: string,
    acknowledgedBy: string
  ): Promise<ApiResponse<AlertEvent>> {
    return this.post<AlertEvent>(`/api/v1/alert/${eventId}/acknowledge`, { acknowledgedBy });
  }

  /**
   * Resolve alert
   */
  async resolveAlert(
    eventId: string,
    actions?: string[]
  ): Promise<ApiResponse<AlertEvent>> {
    return this.post<AlertEvent>(`/api/v1/alert/${eventId}/resolve`, { actions });
  }

  // ==========================================================================
  // Network Management APIs
  // ==========================================================================

  /**
   * Create sensor network
   */
  async createNetwork(
    network: Omit<SensorNetwork, 'networkId' | 'createdAt' | 'updatedAt'>
  ): Promise<ApiResponse<SensorNetwork>> {
    return this.post<SensorNetwork>('/api/v1/network/create', network);
  }

  /**
   * Get sensor network
   */
  async getNetwork(networkId: string): Promise<ApiResponse<SensorNetwork>> {
    return this.get<SensorNetwork>(`/api/v1/network/${networkId}`);
  }

  /**
   * Update network
   */
  async updateNetwork(
    networkId: string,
    updates: Partial<SensorNetwork>
  ): Promise<ApiResponse<SensorNetwork>> {
    return this.put<SensorNetwork>(`/api/v1/network/${networkId}`, updates);
  }

  /**
   * Add sensor to network
   */
  async addSensorToNetwork(
    networkId: string,
    sensorId: string
  ): Promise<ApiResponse<void>> {
    return this.post<void>(`/api/v1/network/${networkId}/sensor`, { sensorId });
  }

  /**
   * Remove sensor from network
   */
  async removeSensorFromNetwork(
    networkId: string,
    sensorId: string
  ): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/network/${networkId}/sensor/${sensorId}`);
  }

  // ==========================================================================
  // Gateway APIs
  // ==========================================================================

  /**
   * Register gateway
   */
  async registerGateway(
    gateway: Omit<Gateway, 'status' | 'lastSeen'>
  ): Promise<ApiResponse<Gateway>> {
    return this.post<Gateway>('/api/v1/gateway/register', gateway);
  }

  /**
   * Get gateway
   */
  async getGateway(gatewayId: string): Promise<ApiResponse<Gateway>> {
    return this.get<Gateway>(`/api/v1/gateway/${gatewayId}`);
  }

  /**
   * Update gateway
   */
  async updateGateway(
    gatewayId: string,
    updates: Partial<Gateway>
  ): Promise<ApiResponse<Gateway>> {
    return this.put<Gateway>(`/api/v1/gateway/${gatewayId}`, updates);
  }

  /**
   * Get gateway health
   */
  async getGatewayHealth(gatewayId: string): Promise<ApiResponse<any>> {
    return this.get<any>(`/api/v1/gateway/${gatewayId}/health`);
  }

  // ==========================================================================
  // Edge Computing APIs
  // ==========================================================================

  /**
   * Create edge processing rule
   */
  async createEdgeRule(
    rule: Omit<EdgeProcessingRule, 'ruleId'>
  ): Promise<ApiResponse<EdgeProcessingRule>> {
    return this.post<EdgeProcessingRule>('/api/v1/edge/rule', rule);
  }

  /**
   * Update edge processing rule
   */
  async updateEdgeRule(
    ruleId: string,
    updates: Partial<EdgeProcessingRule>
  ): Promise<ApiResponse<EdgeProcessingRule>> {
    return this.put<EdgeProcessingRule>(`/api/v1/edge/rule/${ruleId}`, updates);
  }

  /**
   * Delete edge processing rule
   */
  async deleteEdgeRule(ruleId: string): Promise<ApiResponse<void>> {
    return this.delete<void>(`/api/v1/edge/rule/${ruleId}`);
  }

  /**
   * Get anomaly detections
   */
  async getAnomalyDetections(params?: {
    sensorId?: string;
    dateRange?: DateRangeFilter;
    minScore?: number;
  }): Promise<ApiResponse<AnomalyDetection[]>> {
    const queryParams = new URLSearchParams();
    if (params?.sensorId) queryParams.append('sensorId', params.sensorId);
    if (params?.dateRange) {
      queryParams.append('startDate', params.dateRange.startDate);
      queryParams.append('endDate', params.dateRange.endDate);
    }
    if (params?.minScore) queryParams.append('minScore', String(params.minScore));

    return this.get<AnomalyDetection[]>(
      `/api/v1/edge/anomalies${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // Data Quality APIs
  // ==========================================================================

  /**
   * Validate sensor reading
   */
  async validateReading(readingId: string): Promise<ApiResponse<DataValidation>> {
    return this.post<DataValidation>(`/api/v1/data/validate/${readingId}`, {});
  }

  /**
   * Get data quality report
   */
  async getDataQualityReport(params: {
    sensorId?: string;
    dateRange: DateRangeFilter;
  }): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      startDate: params.dateRange.startDate,
      endDate: params.dateRange.endDate,
    });
    if (params.sensorId) queryParams.append('sensorId', params.sensorId);

    return this.get<any>(`/api/v1/data/quality-report?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Reporting & Analytics APIs
  // ==========================================================================

  /**
   * Generate network report
   */
  async generateReport(params: {
    networkId: string;
    reportType: 'daily' | 'weekly' | 'monthly' | 'annual' | 'custom';
    dateRange: DateRangeFilter;
  }): Promise<ApiResponse<NetworkReport>> {
    return this.post<NetworkReport>('/api/v1/report/generate', params);
  }

  /**
   * Get report
   */
  async getReport(reportId: string): Promise<ApiResponse<NetworkReport>> {
    return this.get<NetworkReport>(`/api/v1/report/${reportId}`);
  }

  /**
   * Get KPI dashboard
   */
  async getKPIDashboard(params?: {
    networkId?: string;
    date?: string;
  }): Promise<ApiResponse<KPIDashboard>> {
    const queryParams = new URLSearchParams();
    if (params?.networkId) queryParams.append('networkId', params.networkId);
    if (params?.date) queryParams.append('date', params.date);

    return this.get<KPIDashboard>(
      `/api/v1/analytics/kpi${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Get environmental trends
   */
  async getEnvironmentalTrends(params: {
    parameter: string;
    sensorIds?: string[];
    dateRange: DateRangeFilter;
  }): Promise<ApiResponse<any>> {
    const queryParams = new URLSearchParams({
      parameter: params.parameter,
      startDate: params.dateRange.startDate,
      endDate: params.dateRange.endDate,
    });
    if (params.sensorIds) {
      queryParams.append('sensorIds', params.sensorIds.join(','));
    }

    return this.get<any>(`/api/v1/analytics/trends?${queryParams.toString()}`);
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
      console.log(`[WIA-ENE-035] ${method} ${url}`, body || '');
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
        console.error('[WIA-ENE-035] Request failed:', error);
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
 * Calculate Air Quality Index (AQI) from PM2.5
 */
export function calculateAQI(pm25: number): number {
  // US EPA AQI calculation for PM2.5
  const breakpoints = [
    { cLow: 0, cHigh: 12.0, iLow: 0, iHigh: 50 },
    { cLow: 12.1, cHigh: 35.4, iLow: 51, iHigh: 100 },
    { cLow: 35.5, cHigh: 55.4, iLow: 101, iHigh: 150 },
    { cLow: 55.5, cHigh: 150.4, iLow: 151, iHigh: 200 },
    { cLow: 150.5, cHigh: 250.4, iLow: 201, iHigh: 300 },
    { cLow: 250.5, cHigh: 500.4, iLow: 301, iHigh: 500 },
  ];

  for (const bp of breakpoints) {
    if (pm25 >= bp.cLow && pm25 <= bp.cHigh) {
      return Math.round(
        ((bp.iHigh - bp.iLow) / (bp.cHigh - bp.cLow)) * (pm25 - bp.cLow) + bp.iLow
      );
    }
  }

  return 500; // Hazardous
}

/**
 * Calculate Water Quality Index (simple)
 */
export function calculateWQI(params: {
  ph?: number;
  dissolvedOxygen?: number;
  turbidity?: number;
  conductivity?: number;
}): number {
  let score = 100;
  let count = 0;

  if (params.ph !== undefined) {
    const phScore = 100 - Math.abs(7 - params.ph) * 10;
    score += Math.max(0, phScore);
    count++;
  }

  if (params.dissolvedOxygen !== undefined) {
    const doScore = Math.min(100, (params.dissolvedOxygen / 8) * 100);
    score += doScore;
    count++;
  }

  if (params.turbidity !== undefined) {
    const turbScore = Math.max(0, 100 - params.turbidity * 2);
    score += turbScore;
    count++;
  }

  if (params.conductivity !== undefined) {
    const condScore = Math.max(0, 100 - (params.conductivity / 1000) * 50);
    score += condScore;
    count++;
  }

  return count > 0 ? Math.round(score / (count + 1)) : 0;
}

/**
 * Validate sensor reading value against specs
 */
export function validateReadingValue(
  value: number,
  min: number,
  max: number
): { valid: boolean; error?: string } {
  if (value < min) {
    return { valid: false, error: `Value ${value} below minimum range ${min}` };
  }
  if (value > max) {
    return { valid: false, error: `Value ${value} above maximum range ${max}` };
  }
  return { valid: true };
}

/**
 * Check if sensor battery needs replacement
 */
export function needsBatteryReplacement(batteryLevel: number, threshold: number = 20): boolean {
  return batteryLevel < threshold;
}

/**
 * Calculate network uptime percentage
 */
export function calculateUptime(
  totalTime: number,
  downtime: number
): number {
  if (totalTime === 0) return 0;
  return Math.min(100, ((totalTime - downtime) / totalTime) * 100);
}

/**
 * Estimate battery life based on power consumption
 */
export function estimateBatteryLife(
  batteryCapacity: number,  // mAh
  powerConsumption: number,  // mW
  voltage: number = 3.7      // V
): number {
  // Battery life in hours
  const currentDraw = powerConsumption / voltage; // mA
  const batteryLifeHours = batteryCapacity / currentDraw;

  // Return days
  return batteryLifeHours / 24;
}

/**
 * Get sensor status from last seen time
 */
export function getSensorStatusFromLastSeen(
  lastSeen: string,
  thresholdMinutes: number = 30
): SensorStatus {
  const lastSeenTime = new Date(lastSeen).getTime();
  const now = new Date().getTime();
  const minutesSinceLastSeen = (now - lastSeenTime) / (1000 * 60);

  if (minutesSinceLastSeen > thresholdMinutes) {
    return SensorStatus.OFFLINE;
  }
  return SensorStatus.ONLINE;
}

/**
 * Format timestamp for display
 */
export function formatTimestamp(timestamp: string): string {
  return new Date(timestamp).toLocaleString('ko-KR', {
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    hour12: false,
  });
}

/**
 * Convert RSSI to signal quality percentage
 */
export function rssiToQuality(rssi: number): number {
  // RSSI typically ranges from -120 (worst) to -30 (best) dBm
  const min = -120;
  const max = -30;

  const quality = ((rssi - min) / (max - min)) * 100;
  return Math.max(0, Math.min(100, quality));
}

// ============================================================================
// Default Export
// ============================================================================

export default EnvironmentalSensorSDK;
