/**
 * WIA-AGRI-028 Agricultural Data Exchange Standard - TypeScript Type Definitions
 * @version 1.0.0
 * @standard WIA-AGRI-028
 */

// ============================================================================
// Core Types
// ============================================================================

export type DataType = 'sensor' | 'weather' | 'soil' | 'crop' | 'livestock' | 'equipment' | 'market' | 'other';
export type DataFormat = 'json' | 'xml' | 'csv' | 'geojson' | 'binary';
export type DataQuality = 'excellent' | 'good' | 'fair' | 'poor' | 'unverified';
export type AccessLevel = 'public' | 'private' | 'restricted' | 'confidential';
export type ExchangeStatus = 'pending' | 'active' | 'completed' | 'failed' | 'cancelled';
export type SubscriptionType = 'real-time' | 'batch' | 'on-demand';

// ============================================================================
// Data Source
// ============================================================================

export interface DataSource {
  sourceId: string;
  name: string;
  type: DataType;
  provider: ProviderInfo;
  description: string;
  coverage: GeographicCoverage;
  updateFrequency: UpdateFrequency;
  dataFormat: DataFormat;
  schema: DataSchema;
  accessLevel: AccessLevel;
  pricing?: PricingInfo;
  qualityMetrics: QualityMetrics;
  metadata: Record<string, any>;
  createdAt: string; // ISO 8601 timestamp
  updatedAt: string; // ISO 8601 timestamp
  version: string;
  standard: string;
}

export interface ProviderInfo {
  providerId: string;
  name: string;
  organizationType: string;
  contactEmail: string;
  contactPhone?: string;
  website?: string;
  certifications?: string[];
  reputation?: ReputationScore;
}

export interface ReputationScore {
  rating: number; // 1-5
  reviews: number;
  reliability: number; // 0-100
  dataQuality: number; // 0-100
}

export interface GeographicCoverage {
  type: 'point' | 'area' | 'region' | 'global';
  coordinates?: GeoCoordinates;
  boundingBox?: BoundingBox;
  regions?: string[];
  countries?: string[];
}

export interface GeoCoordinates {
  latitude: number;
  longitude: number;
  elevation?: number;
}

export interface BoundingBox {
  north: number;
  south: number;
  east: number;
  west: number;
}

export interface UpdateFrequency {
  interval: number;
  unit: 'seconds' | 'minutes' | 'hours' | 'days' | 'weeks' | 'months';
  realtime: boolean;
}

export interface DataSchema {
  schemaId: string;
  version: string;
  format: DataFormat;
  fields: SchemaField[];
  sampleData?: any;
  documentation?: string;
}

export interface SchemaField {
  name: string;
  type: 'string' | 'number' | 'boolean' | 'date' | 'object' | 'array';
  required: boolean;
  description?: string;
  unit?: string;
  constraints?: FieldConstraints;
}

export interface FieldConstraints {
  min?: number;
  max?: number;
  pattern?: string;
  enum?: string[];
}

export interface PricingInfo {
  model: 'free' | 'subscription' | 'pay-per-use' | 'custom';
  currency: string;
  price?: number;
  billingPeriod?: string;
  freeQuota?: number;
}

export interface QualityMetrics {
  accuracy: number; // 0-100
  completeness: number; // 0-100
  timeliness: number; // 0-100
  consistency: number; // 0-100
  overall: DataQuality;
  lastAssessed: string; // ISO 8601 timestamp
}

// ============================================================================
// Data Exchange
// ============================================================================

export interface DataExchange {
  exchangeId: string;
  sourceId: string;
  consumer: ConsumerInfo;
  subscription: SubscriptionInfo;
  dataFilter?: DataFilter;
  transformations?: DataTransformation[];
  delivery: DeliveryConfig;
  status: ExchangeStatus;
  statistics: ExchangeStatistics;
  startedAt: string; // ISO 8601 timestamp
  lastDelivery?: string; // ISO 8601 timestamp
  expiresAt?: string; // ISO 8601 timestamp
  createdAt: string; // ISO 8601 timestamp
  updatedAt: string; // ISO 8601 timestamp
}

export interface ConsumerInfo {
  consumerId: string;
  name: string;
  organizationType: string;
  purpose: string;
  contactEmail: string;
  apiKey?: string;
}

export interface SubscriptionInfo {
  type: SubscriptionType;
  frequency?: UpdateFrequency;
  batchSize?: number;
  priority?: 'low' | 'medium' | 'high';
  retentionPeriod?: number; // days
}

export interface DataFilter {
  temporal?: TemporalFilter;
  spatial?: SpatialFilter;
  attribute?: AttributeFilter[];
}

export interface TemporalFilter {
  startDate?: string; // ISO 8601 date
  endDate?: string; // ISO 8601 date
  timeOfDay?: TimeRange;
  daysOfWeek?: number[]; // 0-6
}

export interface TimeRange {
  start: string; // HH:MM
  end: string; // HH:MM
}

export interface SpatialFilter {
  boundingBox?: BoundingBox;
  polygon?: GeoCoordinates[];
  radius?: {
    center: GeoCoordinates;
    distance: number;
    unit: 'meters' | 'kilometers' | 'miles';
  };
}

export interface AttributeFilter {
  field: string;
  operator: 'eq' | 'ne' | 'gt' | 'gte' | 'lt' | 'lte' | 'in' | 'contains';
  value: any;
}

export interface DataTransformation {
  type: 'filter' | 'map' | 'aggregate' | 'convert' | 'enrich';
  config: Record<string, any>;
  order: number;
}

export interface DeliveryConfig {
  method: 'push' | 'pull' | 'webhook' | 'queue';
  endpoint?: string;
  protocol?: 'http' | 'https' | 'mqtt' | 'amqp' | 'websocket';
  authentication?: AuthenticationConfig;
  retry?: RetryPolicy;
  compression?: boolean;
  encryption?: boolean;
}

export interface AuthenticationConfig {
  type: 'api-key' | 'oauth' | 'jwt' | 'basic' | 'certificate';
  credentials?: Record<string, string>;
}

export interface RetryPolicy {
  maxAttempts: number;
  backoffMultiplier: number;
  initialDelay: number; // milliseconds
  maxDelay: number; // milliseconds
}

export interface ExchangeStatistics {
  totalRecords: number;
  successfulDeliveries: number;
  failedDeliveries: number;
  averageLatency: number; // milliseconds
  dataVolume: number; // bytes
  lastError?: ErrorInfo;
}

export interface ErrorInfo {
  code: string;
  message: string;
  timestamp: string;
  retryCount: number;
}

// ============================================================================
// Data Record
// ============================================================================

export interface DataRecord {
  recordId: string;
  sourceId: string;
  timestamp: string; // ISO 8601 timestamp
  location?: GeoCoordinates;
  data: Record<string, any>;
  metadata: RecordMetadata;
  quality?: DataQuality;
  version: string;
}

export interface RecordMetadata {
  collectedAt: string; // ISO 8601 timestamp
  deviceId?: string;
  sensorId?: string;
  processingVersion?: string;
  calibration?: CalibrationInfo;
  tags?: string[];
}

export interface CalibrationInfo {
  calibratedAt: string; // ISO 8601 timestamp
  calibrationType: string;
  accuracy: number;
  nextCalibrationDue?: string; // ISO 8601 date
}

// ============================================================================
// Weather Data
// ============================================================================

export interface WeatherData {
  timestamp: string;
  location: GeoCoordinates;
  temperature: Measurement;
  humidity: Measurement;
  precipitation?: Measurement;
  windSpeed?: Measurement;
  windDirection?: number; // degrees
  pressure?: Measurement;
  solarRadiation?: Measurement;
  cloudCover?: number; // percentage
  forecast?: WeatherForecast[];
}

export interface Measurement {
  value: number;
  unit: string;
  quality?: DataQuality;
}

export interface WeatherForecast {
  timestamp: string;
  temperature: { min: number; max: number; unit: string };
  precipitation: { probability: number; amount?: number; unit?: string };
  conditions: string;
  confidence: number; // 0-100
}

// ============================================================================
// Soil Data
// ============================================================================

export interface SoilData {
  timestamp: string;
  location: GeoCoordinates;
  depth: Measurement;
  moisture: Measurement;
  temperature?: Measurement;
  pH?: number;
  nutrients?: NutrientLevel[];
  organicMatter?: Measurement;
  texture?: SoilTexture;
  compaction?: Measurement;
}

export interface NutrientLevel {
  nutrient: string;
  level: Measurement;
  status: 'deficient' | 'adequate' | 'excessive';
  recommendation?: string;
}

export interface SoilTexture {
  sand: number; // percentage
  silt: number; // percentage
  clay: number; // percentage
  classification: string;
}

// ============================================================================
// Crop Data
// ============================================================================

export interface CropData {
  timestamp: string;
  location: GeoCoordinates;
  cropType: string;
  growthStage: string;
  health: CropHealth;
  biomass?: Measurement;
  yieldEstimate?: Measurement;
  phenology?: PhenologyData;
}

export interface CropHealth {
  overallScore: number; // 0-100
  vigor: number; // 0-100
  stressLevel: number; // 0-100
  diseases?: DiseaseDetection[];
  pests?: PestDetection[];
}

export interface DiseaseDetection {
  disease: string;
  severity: 'low' | 'medium' | 'high';
  confidence: number; // 0-100
  affectedArea: number; // percentage
}

export interface PestDetection {
  pest: string;
  population: string;
  severity: 'low' | 'medium' | 'high';
  confidence: number; // 0-100
}

export interface PhenologyData {
  stage: string;
  daysAfterPlanting: number;
  expectedHarvestDate?: string;
  accumulatedGDD?: number; // Growing Degree Days
}

// ============================================================================
// Market Data
// ============================================================================

export interface MarketData {
  timestamp: string;
  commodity: string;
  market: string;
  price: PriceInfo;
  volume?: number;
  trends?: TrendAnalysis;
  forecast?: PriceForecast[];
}

export interface PriceInfo {
  value: number;
  currency: string;
  unit: string;
  change?: number; // percentage
  changeDirection?: 'up' | 'down' | 'stable';
}

export interface TrendAnalysis {
  shortTerm: 'bullish' | 'bearish' | 'neutral';
  mediumTerm: 'bullish' | 'bearish' | 'neutral';
  longTerm: 'bullish' | 'bearish' | 'neutral';
  volatility: number; // 0-100
}

export interface PriceForecast {
  date: string;
  price: PriceInfo;
  confidence: number; // 0-100
}

// ============================================================================
// API Client Configuration
// ============================================================================

export interface ClientConfig {
  apiKey?: string;
  baseURL: string;
  timeout?: number;
  retryAttempts?: number;
}

export interface AuthTokens {
  accessToken: string;
  refreshToken?: string;
  expiresAt: number;
}

// ============================================================================
// Error Types
// ============================================================================

export class WIAAgriDataExchangeError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIAAgriDataExchangeError';
  }
}
