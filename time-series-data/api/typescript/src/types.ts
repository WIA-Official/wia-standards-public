/**
 * WIA Time Series Data Standard - TypeScript Type Definitions
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Basic Types
// ============================================================================

/**
 * ISO 8601 timestamp string
 */
export type Timestamp = string;

/**
 * Time range for queries
 */
export interface TimeRange {
  startTime: Timestamp;
  endTime: Timestamp;
}

/**
 * Time zone identifier
 */
export type TimeZone = string; // IANA time zone (e.g., 'America/New_York', 'Asia/Seoul')

/**
 * Retention policy duration
 */
export interface RetentionPolicy {
  duration: number;        // Duration in seconds
  shardDuration?: number;  // Shard duration in seconds
  replication?: number;    // Replication factor
}

// ============================================================================
// Data Point Types
// ============================================================================

/**
 * Metric value types
 */
export type MetricValue = number | string | boolean | null;

/**
 * Tags for time series data
 */
export type Tags = Record<string, string>;

/**
 * Fields for time series data
 */
export type Fields = Record<string, MetricValue>;

/**
 * Time series data point
 */
export interface TimeSeriesDataPoint {
  timestamp: Timestamp;
  metric: string;          // Metric name
  value: number;           // Primary numeric value
  tags?: Tags;             // Key-value tags for grouping/filtering
  fields?: Fields;         // Additional field values
  quality?: DataQuality;   // Data quality indicator
  metadata?: Record<string, any>;
}

/**
 * Batch of time series data points
 */
export interface TimeSeriesBatch {
  batchId: string;
  points: TimeSeriesDataPoint[];
  totalPoints: number;
  timestamp: Timestamp;
}

/**
 * Data quality level
 */
export enum DataQuality {
  EXCELLENT = 'EXCELLENT',   // High quality, validated
  GOOD = 'GOOD',             // Good quality
  FAIR = 'FAIR',             // Acceptable quality
  POOR = 'POOR',             // Low quality
  INVALID = 'INVALID',       // Invalid data
  INTERPOLATED = 'INTERPOLATED', // Interpolated value
  ESTIMATED = 'ESTIMATED',   // Estimated value
}

// ============================================================================
// Aggregation Types
// ============================================================================

/**
 * Aggregation function types
 */
export enum AggregationType {
  SUM = 'SUM',               // Sum of values
  AVG = 'AVG',               // Average (mean)
  MIN = 'MIN',               // Minimum value
  MAX = 'MAX',               // Maximum value
  COUNT = 'COUNT',           // Count of data points
  FIRST = 'FIRST',           // First value in time range
  LAST = 'LAST',             // Last value in time range
  MEDIAN = 'MEDIAN',         // Median value
  STDDEV = 'STDDEV',         // Standard deviation
  VARIANCE = 'VARIANCE',     // Variance
  RANGE = 'RANGE',           // Max - Min
  PERCENTILE = 'PERCENTILE', // Percentile (requires parameter)
  MODE = 'MODE',             // Most frequent value
  DERIVATIVE = 'DERIVATIVE', // Rate of change
  INTEGRAL = 'INTEGRAL',     // Cumulative sum over time
}

/**
 * Aggregation configuration
 */
export interface AggregationConfig {
  type: AggregationType;
  interval: string;          // Time interval (e.g., '1m', '5m', '1h', '1d')
  alignTime?: 'start' | 'end' | 'center';
  fillPolicy?: FillPolicy;
  percentile?: number;       // For PERCENTILE aggregation (0-100)
}

/**
 * Fill policy for missing data
 */
export enum FillPolicy {
  NONE = 'NONE',             // Leave gaps
  NULL = 'NULL',             // Fill with null
  ZERO = 'ZERO',             // Fill with 0
  LINEAR = 'LINEAR',         // Linear interpolation
  PREVIOUS = 'PREVIOUS',     // Forward fill (repeat previous value)
  NEXT = 'NEXT',             // Backward fill (use next value)
  MEAN = 'MEAN',             // Fill with mean of surrounding values
}

/**
 * Aggregated data point
 */
export interface AggregatedDataPoint {
  timestamp: Timestamp;
  value: number;
  aggregationType: AggregationType;
  count?: number;            // Number of raw points aggregated
  min?: number;
  max?: number;
  metadata?: Record<string, any>;
}

/**
 * Aggregation result
 */
export interface AggregationResult {
  metric: string;
  tags?: Tags;
  timeRange: TimeRange;
  interval: string;
  aggregationType: AggregationType;
  dataPoints: AggregatedDataPoint[];
  totalPoints: number;
}

// ============================================================================
// Downsampling Types
// ============================================================================

/**
 * Downsampling strategy
 */
export enum DownsamplingStrategy {
  LTTB = 'LTTB',                   // Largest Triangle Three Buckets
  AVERAGE = 'AVERAGE',             // Average downsampling
  MIN_MAX = 'MIN_MAX',             // Min-Max downsampling
  RANDOM = 'RANDOM',               // Random sampling
  FIRST_LAST = 'FIRST_LAST',       // Keep first and last in each bucket
  SIGNIFICANT_POINTS = 'SIGNIFICANT_POINTS', // Keep significant points (peaks, valleys)
}

/**
 * Downsampling configuration
 */
export interface DownsamplingConfig {
  strategy: DownsamplingStrategy;
  targetPoints: number;      // Target number of points after downsampling
  preserveExtremes?: boolean; // Preserve min/max values
  threshold?: number;        // Threshold for significant point detection
}

/**
 * Downsampling result
 */
export interface DownsamplingResult {
  metric: string;
  originalPoints: number;
  downsampledPoints: number;
  strategy: DownsamplingStrategy;
  dataPoints: TimeSeriesDataPoint[];
}

// ============================================================================
// Interpolation Types
// ============================================================================

/**
 * Interpolation method
 */
export enum InterpolationMethod {
  LINEAR = 'LINEAR',         // Linear interpolation
  CUBIC = 'CUBIC',           // Cubic spline interpolation
  STEP = 'STEP',             // Step function (forward fill)
  STEP_AFTER = 'STEP_AFTER', // Step function (backward fill)
  POLYNOMIAL = 'POLYNOMIAL', // Polynomial interpolation
  AKIMA = 'AKIMA',           // Akima spline interpolation
  NEAREST = 'NEAREST',       // Nearest neighbor
}

/**
 * Interpolation configuration
 */
export interface InterpolationConfig {
  method: InterpolationMethod;
  fillGaps?: boolean;        // Whether to fill gaps in data
  maxGapSize?: number;       // Maximum gap size to fill (in seconds)
  extrapolate?: boolean;     // Allow extrapolation beyond data range
}

/**
 * Interpolation result
 */
export interface InterpolationResult {
  timestamps: Timestamp[];
  values: number[];
  method: InterpolationMethod;
  interpolatedCount: number;
}

// ============================================================================
// Anomaly Detection Types
// ============================================================================

/**
 * Anomaly detection method
 */
export enum AnomalyDetectionMethod {
  THRESHOLD = 'THRESHOLD',             // Simple threshold-based
  ZSCORE = 'ZSCORE',                   // Z-score based
  IQR = 'IQR',                         // Interquartile range
  MOVING_AVERAGE = 'MOVING_AVERAGE',   // Moving average deviation
  EXPONENTIAL_SMOOTHING = 'EXPONENTIAL_SMOOTHING', // Exponential smoothing
  SEASONAL_DECOMPOSITION = 'SEASONAL_DECOMPOSITION', // Seasonal decomposition
  ISOLATION_FOREST = 'ISOLATION_FOREST', // Isolation Forest (ML)
  LSTM_AUTOENCODER = 'LSTM_AUTOENCODER', // LSTM Autoencoder (ML)
}

/**
 * Anomaly severity
 */
export enum AnomalySeverity {
  LOW = 'LOW',               // Minor anomaly
  MEDIUM = 'MEDIUM',         // Moderate anomaly
  HIGH = 'HIGH',             // Significant anomaly
  CRITICAL = 'CRITICAL',     // Critical anomaly
}

/**
 * Anomaly detection configuration
 */
export interface AnomalyDetectionConfig {
  method: AnomalyDetectionMethod;
  sensitivity?: number;      // Sensitivity level (0-1)
  threshold?: number;        // Threshold value for threshold-based methods
  windowSize?: number;       // Window size for moving methods
  seasonalPeriod?: number;   // Seasonal period for seasonal methods
  zScoreThreshold?: number;  // Z-score threshold (default: 3)
  iqrMultiplier?: number;    // IQR multiplier (default: 1.5)
}

/**
 * Detected anomaly
 */
export interface Anomaly {
  anomalyId: string;
  timestamp: Timestamp;
  metric: string;
  value: number;
  expectedValue: number;
  deviation: number;         // Deviation from expected
  severity: AnomalySeverity;
  confidence: number;        // Confidence score (0-1)
  method: AnomalyDetectionMethod;
  tags?: Tags;
  context?: {
    windowMean?: number;
    windowStdDev?: number;
    seasonalComponent?: number;
    trendComponent?: number;
  };
  metadata?: Record<string, any>;
}

/**
 * Anomaly detection result
 */
export interface AnomalyDetectionResult {
  metric: string;
  timeRange: TimeRange;
  method: AnomalyDetectionMethod;
  anomalies: Anomaly[];
  totalAnomalies: number;
  analysisTime: number;      // Analysis duration in ms
}

// ============================================================================
// Forecast Types
// ============================================================================

/**
 * Forecasting method
 */
export enum ForecastMethod {
  LINEAR_REGRESSION = 'LINEAR_REGRESSION',
  EXPONENTIAL_SMOOTHING = 'EXPONENTIAL_SMOOTHING',
  HOLT_WINTERS = 'HOLT_WINTERS',   // Triple exponential smoothing
  ARIMA = 'ARIMA',                 // AutoRegressive Integrated Moving Average
  SARIMA = 'SARIMA',               // Seasonal ARIMA
  PROPHET = 'PROPHET',             // Facebook Prophet
  LSTM = 'LSTM',                   // Long Short-Term Memory (neural network)
  GRU = 'GRU',                     // Gated Recurrent Unit
}

/**
 * Forecast configuration
 */
export interface ForecastConfig {
  method: ForecastMethod;
  horizon: number;           // Forecast horizon (number of periods)
  interval: string;          // Forecast interval (e.g., '1h', '1d')
  confidenceLevel?: number;  // Confidence level for intervals (default: 0.95)
  seasonalPeriod?: number;   // Seasonal period for seasonal models
  trainSize?: number;        // Training data size (number of points or percentage)
}

/**
 * Forecast data point
 */
export interface ForecastDataPoint {
  timestamp: Timestamp;
  forecastValue: number;
  lowerBound?: number;       // Lower confidence bound
  upperBound?: number;       // Upper confidence bound
  confidence?: number;       // Confidence score (0-1)
}

/**
 * Forecast result
 */
export interface ForecastResult {
  metric: string;
  method: ForecastMethod;
  trainTimeRange: TimeRange;
  forecastTimeRange: TimeRange;
  forecasts: ForecastDataPoint[];
  accuracy?: {
    mae?: number;            // Mean Absolute Error
    rmse?: number;           // Root Mean Square Error
    mape?: number;           // Mean Absolute Percentage Error
    r2?: number;             // R-squared
  };
  modelMetadata?: Record<string, any>;
}

// ============================================================================
// Query Types
// ============================================================================

/**
 * Time series query
 */
export interface TimeSeriesQuery {
  metric: string | string[]; // Metric name(s)
  timeRange: TimeRange;
  tags?: Tags;               // Tag filters
  aggregation?: AggregationConfig;
  downsampling?: DownsamplingConfig;
  limit?: number;
  offset?: number;
  orderBy?: 'ASC' | 'DESC';
}

/**
 * Query result
 */
export interface QueryResult {
  query: TimeSeriesQuery;
  series: TimeSeriesSeries[];
  totalSeries: number;
  executionTime: number;     // Query execution time in ms
}

/**
 * Time series series (single metric with tags)
 */
export interface TimeSeriesSeries {
  metric: string;
  tags?: Tags;
  dataPoints: TimeSeriesDataPoint[];
  count: number;
}

// ============================================================================
// Storage Types
// ============================================================================

/**
 * Storage engine type
 */
export enum StorageEngine {
  MEMORY = 'MEMORY',         // In-memory storage
  DISK = 'DISK',             // Disk-based storage
  DISTRIBUTED = 'DISTRIBUTED', // Distributed storage
  HYBRID = 'HYBRID',         // Hybrid (memory + disk)
}

/**
 * Compression algorithm
 */
export enum CompressionAlgorithm {
  NONE = 'NONE',
  GZIP = 'GZIP',
  SNAPPY = 'SNAPPY',
  LZ4 = 'LZ4',
  ZSTD = 'ZSTD',
  GORILLA = 'GORILLA',       // Facebook Gorilla compression for time series
}

/**
 * Storage configuration
 */
export interface StorageConfig {
  engine: StorageEngine;
  compression?: CompressionAlgorithm;
  retentionPolicy?: RetentionPolicy;
  shardSize?: number;        // Shard size in MB
  indexing?: {
    enabled: boolean;
    fields: string[];        // Fields to index
  };
  caching?: {
    enabled: boolean;
    size: number;            // Cache size in MB
    ttl: number;             // Time to live in seconds
  };
}

// ============================================================================
// Visualization Types
// ============================================================================

/**
 * Chart type for visualization
 */
export enum ChartType {
  LINE = 'LINE',
  AREA = 'AREA',
  BAR = 'BAR',
  SCATTER = 'SCATTER',
  HEATMAP = 'HEATMAP',
  CANDLESTICK = 'CANDLESTICK',
  HISTOGRAM = 'HISTOGRAM',
}

/**
 * Visualization data configuration
 */
export interface VisualizationConfig {
  chartType: ChartType;
  width?: number;
  height?: number;
  timeFormat?: string;
  valueFormat?: string;
  theme?: 'light' | 'dark';
  showLegend?: boolean;
  showGrid?: boolean;
}

/**
 * Visualization data point
 */
export interface VisualizationDataPoint {
  x: number | string;        // Time or category
  y: number;                 // Value
  label?: string;
  color?: string;
  metadata?: Record<string, any>;
}

/**
 * Visualization series
 */
export interface VisualizationSeries {
  name: string;
  data: VisualizationDataPoint[];
  color?: string;
  type?: ChartType;
}

/**
 * Visualization data
 */
export interface VisualizationData {
  config: VisualizationConfig;
  series: VisualizationSeries[];
  xAxis?: {
    label?: string;
    type?: 'time' | 'category' | 'value';
  };
  yAxis?: {
    label?: string;
    min?: number;
    max?: number;
  };
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * Time series event type
 */
export enum EventType {
  DATA_INGESTED = 'DATA_INGESTED',
  ANOMALY_DETECTED = 'ANOMALY_DETECTED',
  THRESHOLD_EXCEEDED = 'THRESHOLD_EXCEEDED',
  FORECAST_GENERATED = 'FORECAST_GENERATED',
  RETENTION_APPLIED = 'RETENTION_APPLIED',
  ERROR = 'ERROR',
}

/**
 * Time series event
 */
export interface TimeSeriesEvent {
  eventId: string;
  eventType: EventType;
  timestamp: Timestamp;
  metric?: string;
  data?: any;
  metadata?: Record<string, any>;
}

/**
 * Event handler function
 */
export type EventHandler = (event: TimeSeriesEvent) => void | Promise<void>;

// ============================================================================
// Statistics Types
// ============================================================================

/**
 * Statistical summary
 */
export interface StatisticalSummary {
  count: number;
  sum: number;
  mean: number;
  median: number;
  min: number;
  max: number;
  stdDev: number;
  variance: number;
  skewness?: number;
  kurtosis?: number;
  percentiles?: {
    p10?: number;
    p25?: number;
    p50?: number;
    p75?: number;
    p90?: number;
    p95?: number;
    p99?: number;
  };
}

// ============================================================================
// API Types
// ============================================================================

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: Timestamp;
    version: string;
    requestId?: string;
  };
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  page: number;
  limit: number;
  sortBy?: string;
  sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  limit: number;
  hasMore: boolean;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  Timestamp,
  TimeRange,
  TimeZone,
  RetentionPolicy,
  MetricValue,
  Tags,
  Fields,
};
