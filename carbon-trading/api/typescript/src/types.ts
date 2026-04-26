/**
 * WIA-FIN-025 Carbon Trading Standard - Type Definitions
 * @version 2.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

export type CreditType = 'avoidance' | 'reduction' | 'removal';
export type ProjectType = 'renewable_energy' | 'redd+' | 'energy_efficiency' | 'methane_capture' | 'agriculture' | 'waste' | 'other';
export type CreditStatus = 'issued' | 'traded' | 'retired' | 'cancelled';
export type ProjectStatus = 'registered' | 'validated' | 'operational' | 'completed' | 'suspended' | 'cancelled';
export type Standard = 'VCS' | 'GS' | 'CAR' | 'ACR' | 'CDM' | 'Article6';
export type RemovalMethod = 'dac' | 'beccs' | 'biochar' | 'weathering' | 'ocean' | 'forestry';
export type OrderType = 'market' | 'limit' | 'stop';
export type OrderSide = 'buy' | 'sell';
export type OrderStatus = 'pending' | 'partial' | 'filled' | 'cancelled' | 'rejected';
export type TimeInForce = 'GTC' | 'IOC' | 'FOK' | 'GTD';

// ============================================================================
// Carbon Credit
// ============================================================================

export interface CarbonCredit {
  // Core Identity
  id: string;
  serialNumber: string;
  projectId: string;

  // Credit Details
  vintage: number;
  quantity: number;
  creditType: CreditType;

  // Verification
  methodology: string;
  standard: Standard;
  verificationBody: string;
  verificationDate: string;
  qualityScore: number;

  // Metadata
  issuanceDate: string;
  expiryDate?: string;
  status: CreditStatus;
  currentOwner: string;

  // Removal-Specific
  removalMethod?: RemovalMethod;
  permanence?: number;
  reversalRisk?: number;

  // Blockchain
  tokenAddress?: string;
  chainId?: number;

  // Article 6
  correspondingAdjustment?: CorrespondingAdjustment;

  // Tracking
  transactions: TransactionRecord[];
  retirementInfo?: RetirementInfo;
}

export interface CorrespondingAdjustment {
  hostCountry: string;
  acquiringCountry: string;
  adjustmentApplied: boolean;
  itmoId: string;
}

export interface RetirementInfo {
  retirementId: string;
  retirementDate: string;
  beneficiary: string;
  reason: string;
  certificateUrl?: string;
}

export interface TransactionRecord {
  id: string;
  type: 'issuance' | 'transfer' | 'retirement';
  from: string;
  to: string;
  quantity: number;
  price?: number;
  timestamp: string;
  blockchainHash?: string;
}

// ============================================================================
// Project
// ============================================================================

export interface CarbonProject {
  id: string;
  name: string;
  description: string;

  // Classification
  type: ProjectType;
  subType?: string;
  sector: 'energy' | 'forestry' | 'agriculture' | 'waste' | 'industrial' | 'other';

  // Location
  location: ProjectLocation;

  // Participants
  developer: Organization;
  verifier?: Organization;
  financiers?: Organization[];

  // Technical
  methodology: string;
  baselineScenario: string;
  monitoringPlan: MonitoringPlan;

  // Monitoring
  sensors?: Sensor[];

  // Performance
  estimatedAnnualReduction: number;
  creditsIssued: number;
  creditsRetired: number;

  // Dates
  startDate: string;
  validationDate?: string;
  verificationDates: string[];

  // Quality & Co-Benefits
  qualityIndicators: QualityIndicators;
  coBenefits?: CoBenefits;

  // Financial
  budget?: number;
  carbonRevenue?: number;

  // Status
  status: ProjectStatus;

  // AI Insights
  aiInsights?: AIInsights;
}

export interface ProjectLocation {
  country: string;
  region: string;
  coordinates?: GeoCoordinates;
  boundary?: GeoJSON.Geometry;
}

export interface GeoCoordinates {
  latitude: number;
  longitude: number;
}

export interface Organization {
  id?: string;
  name: string;
  type?: 'developer' | 'verifier' | 'investor' | 'other';
  contact: string;
  website?: string;
}

export interface MonitoringPlan {
  methodology: string;
  frequency: string;
  parameters: MonitoringParameter[];
  qaQcProcedures: string;
}

export interface MonitoringParameter {
  name: string;
  unit: string;
  measurementMethod: string;
  accuracy: number;
}

export interface Sensor {
  type: string;
  count: number;
  dataFrequency: string;
  endpoint: string;
}

export interface QualityIndicators {
  additionality: number;
  permanence: number;
  leakage: number;
  verification: number;
}

export interface CoBenefits {
  biodiversity: boolean;
  communityDevelopment: boolean;
  waterQuality: boolean;
  airQuality: boolean;
  sustainableDevelopment: boolean;
}

export interface AIInsights {
  performancePrediction: number;
  riskAssessment: string;
  recommendations: string[];
}

// ============================================================================
// Trading
// ============================================================================

export interface TradeOrder {
  id: string;

  // Order Details
  type: OrderType;
  side: OrderSide;
  quantity: number;

  // Credit Specifications
  creditType?: CreditType;
  vintage?: number;
  minQualityScore?: number;
  standard?: Standard[];
  projectType?: ProjectType[];

  // Pricing
  limitPrice?: number;
  stopPrice?: number;

  // Execution
  status: OrderStatus;
  filledQuantity: number;
  averagePrice: number;

  // Timing
  timeInForce: TimeInForce;
  goodTillDate?: string;

  // Participant
  userId: string;
  organizationId: string;

  // Settlement
  settlement: 'immediate' | 't+1' | 't+2';

  // Timestamps
  createdAt: string;
  updatedAt: string;
  executedAt?: string;
}

export interface MarketPrice {
  creditType: CreditType;
  vintage: number;
  standard?: Standard;
  bid: number;
  ask: number;
  last: number;
  volume24h: number;
  change24h: number;
  timestamp: string;
}

// ============================================================================
// Verification
// ============================================================================

export interface VerificationRequest {
  projectId: string;
  period: string;
  data: MonitoringData;
}

export interface MonitoringData {
  emissionReductions: number;
  sensorReadings?: any[];
  satelliteAnalysis?: any;
  fieldMeasurements?: any;
}

export interface VerificationResult {
  verificationId: string;
  result: 'approved' | 'rejected' | 'requires_revision';
  confidence: number;
  analysis: VerificationAnalysis;
  humanReviewRequired: boolean;
  issues?: string[];
}

export interface VerificationAnalysis {
  dataQuality: number;
  consistencyCheck: string;
  anomalyDetection: string;
  calculationVerification: string;
}

// ============================================================================
// API Request/Response Types
// ============================================================================

export interface ProjectCreateRequest {
  name: string;
  description: string;
  type: ProjectType;
  location: ProjectLocation;
  methodology: string;
  developer: Organization;
  estimatedAnnualReduction: number;
  sensors?: Sensor[];
}

export interface ProjectCreateResponse {
  projectId: string;
  status: ProjectStatus;
  aiAssessment?: {
    additionality: number;
    feasibility: number;
    risks: string[];
    recommendations: string[];
  };
}

export interface CreditIssueRequest {
  projectId: string;
  quantity: number;
  vintage: number;
  creditType: CreditType;
  verificationReport: string;
  calculateQuality?: boolean;
}

export interface CreditIssueResponse {
  credits: CarbonCredit[];
}

export interface CreditSearchParams {
  type?: CreditType;
  minQuality?: number;
  vintage?: number;
  standard?: Standard;
  projectType?: ProjectType;
  limit?: number;
  offset?: number;
}

export interface CreditSearchResponse {
  total: number;
  credits: CarbonCredit[];
  filters: {
    applied: string[];
    available: string[];
  };
}

export interface OrderCreateRequest {
  type: OrderType;
  side: OrderSide;
  quantity: number;
  limitPrice?: number;
  stopPrice?: number;
  minQualityScore?: number;
  creditType?: CreditType;
  timeInForce?: TimeInForce;
}

export interface OrderCreateResponse {
  orderId: string;
  status: OrderStatus;
  filledQuantity: number;
  averagePrice: number;
  totalCost: number;
  credits?: CarbonCredit[];
}

// ============================================================================
// SDK Configuration
// ============================================================================

export interface SDKConfig {
  apiKey: string;
  environment: 'production' | 'staging' | 'development';
  baseUrl?: string;
  wsUrl?: string;
  timeout?: number;
  retryAttempts?: number;
}

// ============================================================================
// WebSocket Events
// ============================================================================

export interface WSMessage {
  channel: string;
  timestamp: string;
  data: any;
}

export interface WSSubscribeMessage {
  action: 'subscribe' | 'unsubscribe';
  channels: string[];
}

export interface PriceUpdate {
  channel: string;
  timestamp: string;
  data: {
    [key: string]: number;
  };
}

export interface TradeUpdate {
  channel: string;
  timestamp: string;
  data: {
    orderId: string;
    status: OrderStatus;
    filledQuantity: number;
    price: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

export class CarbonTradingError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'CarbonTradingError';
  }
}

export interface APIErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}
