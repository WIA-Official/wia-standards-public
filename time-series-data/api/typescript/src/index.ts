/**
 * WIA Time Series Data Standard - TypeScript SDK
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// Export all types
export * from './types';

import {
  TimeSeriesDataPoint,
  TimeSeriesBatch,
  TimeRange,
  AggregationConfig,
  AggregationResult,
  DownsamplingConfig,
  DownsamplingResult,
  InterpolationConfig,
  InterpolationResult,
  AnomalyDetectionConfig,
  AnomalyDetectionResult,
  Anomaly,
  ForecastConfig,
  ForecastResult,
  TimeSeriesQuery,
  QueryResult,
  VisualizationData,
  VisualizationConfig,
  StatisticalSummary,
  EventType,
  TimeSeriesEvent,
  EventHandler,
  ApiResponse,
  Tags,
  AggregationType,
  DownsamplingStrategy,
  AnomalyDetectionMethod,
  ForecastMethod,
  ChartType,
} from './types';

// ============================================================================
// SDK Configuration
// ============================================================================

export interface TimeSeriesDataSDKConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  retries?: number;
  debug?: boolean;
  defaultTimeZone?: string;
}

// ============================================================================
// Main SDK Class
// ============================================================================

export class WIATimeSeriesData {
  private config: Required<TimeSeriesDataSDKConfig>;
  private headers: Record<string, string>;
  private eventHandlers: Map<EventType, Set<EventHandler>>;

  constructor(config: TimeSeriesDataSDKConfig) {
    this.config = {
      timeout: 30000,
      retries: 3,
      debug: false,
      defaultTimeZone: 'UTC',
      ...config,
    };

    this.headers = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.config.apiKey}`,
      'X-WIA-Standard': 'TIME-SERIES-DATA',
      'X-WIA-Version': '1.0.0',
    };

    this.eventHandlers = new Map();
  }

  // ==========================================================================
  // Data Ingestion Methods
  // ==========================================================================

  /**
   * Ingest a single data point
   */
  async ingestDataPoint(dataPoint: Omit<TimeSeriesDataPoint, 'timestamp'>): Promise<ApiResponse<TimeSeriesDataPoint>> {
    const point = {
      ...dataPoint,
      timestamp: new Date().toISOString(),
    };

    const response = await this.post<TimeSeriesDataPoint>('/api/v1/data/ingest', point);

    if (response.success) {
      this.emitEvent({
        eventId: this.generateId(),
        eventType: EventType.DATA_INGESTED,
        timestamp: new Date().toISOString(),
        metric: dataPoint.metric,
        data: point,
      });
    }

    return response;
  }

  /**
   * Ingest multiple data points in batch
   */
  async ingestBatch(dataPoints: Omit<TimeSeriesDataPoint, 'timestamp'>[]): Promise<ApiResponse<TimeSeriesBatch>> {
    const batch = {
      batchId: this.generateId(),
      points: dataPoints.map(point => ({
        ...point,
        timestamp: new Date().toISOString(),
      })),
      totalPoints: dataPoints.length,
      timestamp: new Date().toISOString(),
    };

    const response = await this.post<TimeSeriesBatch>('/api/v1/data/ingest/batch', batch);

    if (response.success) {
      this.emitEvent({
        eventId: this.generateId(),
        eventType: EventType.DATA_INGESTED,
        timestamp: new Date().toISOString(),
        data: { batchId: batch.batchId, count: batch.totalPoints },
      });
    }

    return response;
  }

  /**
   * Stream data points continuously
   */
  async streamDataPoints(
    dataPoints: AsyncIterable<Omit<TimeSeriesDataPoint, 'timestamp'>>
  ): Promise<ApiResponse<{ ingested: number; failed: number }>> {
    let ingested = 0;
    let failed = 0;

    for await (const point of dataPoints) {
      const response = await this.ingestDataPoint(point);
      if (response.success) {
        ingested++;
      } else {
        failed++;
      }
    }

    return {
      success: true,
      data: { ingested, failed },
      metadata: {
        timestamp: new Date().toISOString(),
        version: '1.0.0',
      },
    };
  }

  // ==========================================================================
  // Querying Methods
  // ==========================================================================

  /**
   * Query time series data
   */
  async query(query: TimeSeriesQuery): Promise<ApiResponse<QueryResult>> {
    const queryParams = new URLSearchParams();

    if (Array.isArray(query.metric)) {
      queryParams.append('metrics', query.metric.join(','));
    } else {
      queryParams.append('metric', query.metric);
    }

    queryParams.append('startTime', query.timeRange.startTime);
    queryParams.append('endTime', query.timeRange.endTime);

    if (query.tags) {
      queryParams.append('tags', JSON.stringify(query.tags));
    }

    if (query.aggregation) {
      queryParams.append('aggregation', JSON.stringify(query.aggregation));
    }

    if (query.downsampling) {
      queryParams.append('downsampling', JSON.stringify(query.downsampling));
    }

    if (query.limit) queryParams.append('limit', String(query.limit));
    if (query.offset) queryParams.append('offset', String(query.offset));
    if (query.orderBy) queryParams.append('orderBy', query.orderBy);

    return this.get<QueryResult>(`/api/v1/data/query?${queryParams.toString()}`);
  }

  /**
   * Get latest data point for a metric
   */
  async getLatest(metric: string, tags?: Tags): Promise<ApiResponse<TimeSeriesDataPoint>> {
    const queryParams = new URLSearchParams({ metric });
    if (tags) {
      queryParams.append('tags', JSON.stringify(tags));
    }

    return this.get<TimeSeriesDataPoint>(`/api/v1/data/latest?${queryParams.toString()}`);
  }

  /**
   * Get data point by timestamp
   */
  async getByTimestamp(
    metric: string,
    timestamp: string,
    tags?: Tags
  ): Promise<ApiResponse<TimeSeriesDataPoint>> {
    const queryParams = new URLSearchParams({ metric, timestamp });
    if (tags) {
      queryParams.append('tags', JSON.stringify(tags));
    }

    return this.get<TimeSeriesDataPoint>(`/api/v1/data/point?${queryParams.toString()}`);
  }

  /**
   * Get time range boundaries for a metric
   */
  async getTimeRangeBoundaries(metric: string, tags?: Tags): Promise<ApiResponse<TimeRange>> {
    const queryParams = new URLSearchParams({ metric });
    if (tags) {
      queryParams.append('tags', JSON.stringify(tags));
    }

    return this.get<TimeRange>(`/api/v1/data/boundaries?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Aggregation Methods
  // ==========================================================================

  /**
   * Aggregate time series data
   */
  async aggregate(
    metric: string,
    timeRange: TimeRange,
    config: AggregationConfig,
    tags?: Tags
  ): Promise<ApiResponse<AggregationResult>> {
    const params = {
      metric,
      timeRange,
      config,
      tags,
    };

    return this.post<AggregationResult>('/api/v1/data/aggregate', params);
  }

  /**
   * Perform multiple aggregations in parallel
   */
  async multiAggregate(
    metric: string,
    timeRange: TimeRange,
    configs: AggregationConfig[],
    tags?: Tags
  ): Promise<ApiResponse<AggregationResult[]>> {
    const params = {
      metric,
      timeRange,
      configs,
      tags,
    };

    return this.post<AggregationResult[]>('/api/v1/data/aggregate/multi', params);
  }

  /**
   * Calculate statistical summary
   */
  async calculateStatistics(
    metric: string,
    timeRange: TimeRange,
    tags?: Tags
  ): Promise<ApiResponse<StatisticalSummary>> {
    const params = { metric, timeRange, tags };
    return this.post<StatisticalSummary>('/api/v1/data/statistics', params);
  }

  // ==========================================================================
  // Downsampling Methods
  // ==========================================================================

  /**
   * Downsample time series data
   */
  async downsample(
    metric: string,
    timeRange: TimeRange,
    config: DownsamplingConfig,
    tags?: Tags
  ): Promise<ApiResponse<DownsamplingResult>> {
    const params = {
      metric,
      timeRange,
      config,
      tags,
    };

    return this.post<DownsamplingResult>('/api/v1/data/downsample', params);
  }

  /**
   * Auto-downsample based on target resolution
   */
  async autoDownsample(
    metric: string,
    timeRange: TimeRange,
    targetPoints: number,
    tags?: Tags
  ): Promise<ApiResponse<DownsamplingResult>> {
    const config: DownsamplingConfig = {
      strategy: DownsamplingStrategy.LTTB,
      targetPoints,
      preserveExtremes: true,
    };

    return this.downsample(metric, timeRange, config, tags);
  }

  // ==========================================================================
  // Anomaly Detection Methods
  // ==========================================================================

  /**
   * Detect anomalies in time series data
   */
  async detectAnomalies(
    metric: string,
    timeRange: TimeRange,
    config: AnomalyDetectionConfig,
    tags?: Tags
  ): Promise<ApiResponse<AnomalyDetectionResult>> {
    const params = {
      metric,
      timeRange,
      config,
      tags,
    };

    const response = await this.post<AnomalyDetectionResult>('/api/v1/anomaly/detect', params);

    if (response.success && response.data && response.data.anomalies.length > 0) {
      response.data.anomalies.forEach(anomaly => {
        this.emitEvent({
          eventId: this.generateId(),
          eventType: EventType.ANOMALY_DETECTED,
          timestamp: new Date().toISOString(),
          metric: anomaly.metric,
          data: anomaly,
        });
      });
    }

    return response;
  }

  /**
   * Get anomaly detection history
   */
  async getAnomalyHistory(
    metric: string,
    timeRange: TimeRange,
    tags?: Tags
  ): Promise<ApiResponse<Anomaly[]>> {
    const queryParams = new URLSearchParams({
      metric,
      startTime: timeRange.startTime,
      endTime: timeRange.endTime,
    });

    if (tags) {
      queryParams.append('tags', JSON.stringify(tags));
    }

    return this.get<Anomaly[]>(`/api/v1/anomaly/history?${queryParams.toString()}`);
  }

  /**
   * Configure real-time anomaly detection
   */
  async configureRealtimeAnomalyDetection(
    metric: string,
    config: AnomalyDetectionConfig,
    tags?: Tags
  ): Promise<ApiResponse<{ detectionId: string }>> {
    const params = { metric, config, tags };
    return this.post<{ detectionId: string }>('/api/v1/anomaly/configure', params);
  }

  // ==========================================================================
  // Forecasting Methods
  // ==========================================================================

  /**
   * Generate forecast for time series
   */
  async forecast(
    metric: string,
    config: ForecastConfig,
    tags?: Tags
  ): Promise<ApiResponse<ForecastResult>> {
    const params = { metric, config, tags };

    const response = await this.post<ForecastResult>('/api/v1/forecast/generate', params);

    if (response.success) {
      this.emitEvent({
        eventId: this.generateId(),
        eventType: EventType.FORECAST_GENERATED,
        timestamp: new Date().toISOString(),
        metric,
        data: response.data,
      });
    }

    return response;
  }

  /**
   * Get forecast by ID
   */
  async getForecast(forecastId: string): Promise<ApiResponse<ForecastResult>> {
    return this.get<ForecastResult>(`/api/v1/forecast/${forecastId}`);
  }

  /**
   * Evaluate forecast accuracy
   */
  async evaluateForecast(
    forecastId: string,
    actualData: TimeSeriesDataPoint[]
  ): Promise<ApiResponse<{ mae: number; rmse: number; mape: number; r2: number }>> {
    return this.post(`/api/v1/forecast/${forecastId}/evaluate`, { actualData });
  }

  // ==========================================================================
  // Visualization Data Preparation Methods
  // ==========================================================================

  /**
   * Prepare data for visualization
   */
  async prepareVisualization(
    metric: string | string[],
    timeRange: TimeRange,
    config: VisualizationConfig,
    tags?: Tags
  ): Promise<ApiResponse<VisualizationData>> {
    const params = {
      metric,
      timeRange,
      config,
      tags,
    };

    return this.post<VisualizationData>('/api/v1/visualization/prepare', params);
  }

  /**
   * Generate chart data
   */
  async generateChartData(
    metric: string,
    timeRange: TimeRange,
    chartType: ChartType,
    tags?: Tags
  ): Promise<ApiResponse<VisualizationData>> {
    const config: VisualizationConfig = {
      chartType,
      showLegend: true,
      showGrid: true,
    };

    return this.prepareVisualization(metric, timeRange, config, tags);
  }

  /**
   * Export data for external visualization tools
   */
  async exportForVisualization(
    metric: string,
    timeRange: TimeRange,
    format: 'csv' | 'json' | 'excel',
    tags?: Tags
  ): Promise<ApiResponse<Blob>> {
    const queryParams = new URLSearchParams({
      metric,
      startTime: timeRange.startTime,
      endTime: timeRange.endTime,
      format,
    });

    if (tags) {
      queryParams.append('tags', JSON.stringify(tags));
    }

    return this.get<Blob>(`/api/v1/export/visualization?${queryParams.toString()}`);
  }

  // ==========================================================================
  // Event Handling Methods
  // ==========================================================================

  /**
   * Register event handler
   */
  on(eventType: EventType, handler: EventHandler): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, new Set());
    }
    this.eventHandlers.get(eventType)!.add(handler);
  }

  /**
   * Unregister event handler
   */
  off(eventType: EventType, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(eventType);
    if (handlers) {
      handlers.delete(handler);
    }
  }

  /**
   * Emit event to all registered handlers
   */
  private async emitEvent(event: TimeSeriesEvent): Promise<void> {
    const handlers = this.eventHandlers.get(event.eventType);
    if (handlers && handlers.size > 0) {
      const promises = Array.from(handlers).map(handler => handler(event));
      await Promise.allSettled(promises);
    }
  }

  /**
   * Get event history
   */
  async getEventHistory(
    eventType?: EventType,
    timeRange?: TimeRange
  ): Promise<ApiResponse<TimeSeriesEvent[]>> {
    const queryParams = new URLSearchParams();
    if (eventType) queryParams.append('eventType', eventType);
    if (timeRange) {
      queryParams.append('startTime', timeRange.startTime);
      queryParams.append('endTime', timeRange.endTime);
    }

    return this.get<TimeSeriesEvent[]>(
      `/api/v1/events${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  // ==========================================================================
  // Metric Management Methods
  // ==========================================================================

  /**
   * List all available metrics
   */
  async listMetrics(tags?: Tags): Promise<ApiResponse<string[]>> {
    const queryParams = new URLSearchParams();
    if (tags) {
      queryParams.append('tags', JSON.stringify(tags));
    }

    return this.get<string[]>(
      `/api/v1/metrics${queryParams.toString() ? '?' + queryParams.toString() : ''}`
    );
  }

  /**
   * Get metric metadata
   */
  async getMetricMetadata(metric: string): Promise<ApiResponse<any>> {
    return this.get(`/api/v1/metrics/${metric}/metadata`);
  }

  /**
   * Delete metric data
   */
  async deleteMetric(metric: string, timeRange?: TimeRange): Promise<ApiResponse<void>> {
    const queryParams = new URLSearchParams({ metric });
    if (timeRange) {
      queryParams.append('startTime', timeRange.startTime);
      queryParams.append('endTime', timeRange.endTime);
    }

    return this.delete<void>(`/api/v1/metrics?${queryParams.toString()}`);
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
      console.log(`[WIA-TIME-SERIES] ${method} ${url}`, body || '');
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
        console.error('[WIA-TIME-SERIES] Request failed:', error);
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

  private async post<T>(path: string, body?: any): Promise<ApiResponse<T>> {
    return this.request<T>('POST', path, body);
  }

  private async put<T>(path: string, body: any): Promise<ApiResponse<T>> {
    return this.request<T>('PUT', path, body);
  }

  private async delete<T>(path: string): Promise<ApiResponse<T>> {
    return this.request<T>('DELETE', path);
  }

  private generateId(): string {
    return `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }
}

// ============================================================================
// Factory Function
// ============================================================================

/**
 * Create a new WIA Time Series Data SDK instance
 */
export function createTimeSeriesDataSDK(config: TimeSeriesDataSDKConfig): WIATimeSeriesData {
  return new WIATimeSeriesData(config);
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Parse time interval string to seconds
 */
export function parseInterval(interval: string): number {
  const match = interval.match(/^(\d+)([smhd])$/);
  if (!match) {
    throw new Error(`Invalid interval format: ${interval}`);
  }

  const value = parseInt(match[1], 10);
  const unit = match[2];

  const multipliers: Record<string, number> = {
    s: 1,
    m: 60,
    h: 3600,
    d: 86400,
  };

  return value * multipliers[unit];
}

/**
 * Format time range for display
 */
export function formatTimeRange(timeRange: TimeRange): string {
  const start = new Date(timeRange.startTime).toLocaleString();
  const end = new Date(timeRange.endTime).toLocaleString();
  return `${start} - ${end}`;
}

/**
 * Calculate time range duration in seconds
 */
export function getTimeRangeDuration(timeRange: TimeRange): number {
  const start = new Date(timeRange.startTime).getTime();
  const end = new Date(timeRange.endTime).getTime();
  return (end - start) / 1000;
}

/**
 * Create time range from now minus duration
 */
export function createTimeRangeFromDuration(durationSeconds: number): TimeRange {
  const endTime = new Date();
  const startTime = new Date(endTime.getTime() - durationSeconds * 1000);

  return {
    startTime: startTime.toISOString(),
    endTime: endTime.toISOString(),
  };
}

/**
 * Validate time range
 */
export function validateTimeRange(timeRange: TimeRange): { valid: boolean; error?: string } {
  const start = new Date(timeRange.startTime).getTime();
  const end = new Date(timeRange.endTime).getTime();

  if (isNaN(start)) {
    return { valid: false, error: 'Invalid start time' };
  }

  if (isNaN(end)) {
    return { valid: false, error: 'Invalid end time' };
  }

  if (start >= end) {
    return { valid: false, error: 'Start time must be before end time' };
  }

  return { valid: true };
}

/**
 * Calculate moving average
 */
export function calculateMovingAverage(values: number[], windowSize: number): number[] {
  const result: number[] = [];

  for (let i = 0; i < values.length; i++) {
    const start = Math.max(0, i - windowSize + 1);
    const window = values.slice(start, i + 1);
    const avg = window.reduce((sum, val) => sum + val, 0) / window.length;
    result.push(avg);
  }

  return result;
}

/**
 * Calculate exponential moving average
 */
export function calculateEMA(values: number[], alpha: number = 0.3): number[] {
  if (values.length === 0) return [];

  const result: number[] = [values[0]];

  for (let i = 1; i < values.length; i++) {
    const ema = alpha * values[i] + (1 - alpha) * result[i - 1];
    result.push(ema);
  }

  return result;
}

// ============================================================================
// Default Export
// ============================================================================

export default WIATimeSeriesData;
