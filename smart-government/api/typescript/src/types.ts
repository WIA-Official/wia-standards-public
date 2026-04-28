/**
 * WIA Smart Government Standard (WIA-SOC-006)
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @standard WIA-SOC-006
 * @philosophy 弘益人間 (홍익인간) · Benefit All Humanity
 */

// ============================================================================
// Configuration Types
// ============================================================================

export interface SmartGovernmentConfig {
  apiKey: string;
  region: string;
  baseUrl?: string;
  timeout?: number;
  retryAttempts?: number;
  enableLogging?: boolean;
}

// ============================================================================
// Citizen & Identity Types
// ============================================================================

export interface PersonalInfo {
  firstName: string;
  lastName: string;
  middleName?: string;
  dateOfBirth: string;
  nationality: string;
  gender?: 'male' | 'female' | 'other' | 'prefer_not_to_say';
}

export interface Address {
  street: string;
  city: string;
  state: string;
  postalCode: string;
  country: string;
  apt?: string;
  coordinates?: {
    lat: number;
    lng: number;
  };
}

export interface ContactInfo {
  email: string;
  phone: string;
  alternatePhone?: string;
  address: Address;
}

export type VerificationMethod = 'biometric' | 'document' | 'blockchain' | 'email' | 'sms';
export type TrustLevel = 'low' | 'medium' | 'high' | 'verified';

export interface IdentityVerification {
  method: VerificationMethod;
  verifiedAt: string;
  trustLevel: TrustLevel;
  verificationId?: string;
}

export interface AccessibilityPreferences {
  screenReader: boolean;
  largeText: boolean;
  colorBlindMode?: 'none' | 'protanopia' | 'deuteranopia' | 'tritanopia';
  highContrast?: boolean;
  reducedMotion?: boolean;
}

export interface CitizenPreferences {
  language: string;
  notificationChannels: ('email' | 'sms' | 'push' | 'postal')[];
  accessibility: AccessibilityPreferences;
  timezone?: string;
}

export interface CitizenProfile {
  citizenId: string;
  personalInfo: PersonalInfo;
  contactInfo: ContactInfo;
  identityVerification: IdentityVerification;
  preferences: CitizenPreferences;
  createdAt: string;
  updatedAt: string;
}

// ============================================================================
// Service Types
// ============================================================================

export type ServiceType = 'permit' | 'license' | 'certificate' | 'benefit' | 'complaint' | 'inquiry';
export type ServiceCategory = 'building' | 'business' | 'tax' | 'health' | 'education' | 'transportation' | 'utilities' | 'other';
export type Priority = 'low' | 'normal' | 'high' | 'urgent';
export type RequestStatus = 'draft' | 'submitted' | 'under_review' | 'pending_payment' | 'approved' | 'rejected' | 'completed' | 'cancelled';

export interface ServiceDocument {
  documentId: string;
  type: string;
  name: string;
  url: string;
  uploadedAt: string;
  verified: boolean;
  verificationMethod?: 'ocr' | 'manual' | 'blockchain';
  size?: number;
  mimeType?: string;
}

export interface PaymentInfo {
  amount: number;
  currency: string;
  status: 'pending' | 'completed' | 'failed' | 'refunded';
  transactionId?: string;
  paymentMethod?: 'credit_card' | 'debit_card' | 'bank_transfer' | 'digital_wallet';
  paidAt?: string;
}

export interface ApprovalRecord {
  department: string;
  approvedBy: string;
  approvedAt: string;
  comments?: string;
  decision: 'approved' | 'rejected' | 'pending';
}

export interface WorkflowInfo {
  currentStep: string;
  totalSteps: number;
  progress: number;
  approvals: ApprovalRecord[];
  estimatedCompletionDate?: string;
}

export interface ServiceRequest {
  requestId: string;
  citizenId: string;
  serviceType: ServiceType;
  serviceCategory: ServiceCategory;
  priority: Priority;
  status: RequestStatus;
  submittedAt: string;
  updatedAt: string;
  completedAt?: string;
  documents: ServiceDocument[];
  formData: Record<string, any>;
  payment?: PaymentInfo;
  workflow: WorkflowInfo;
  assignedTo?: string;
  notes?: string[];
}

export interface ServiceCatalogItem {
  serviceId: string;
  name: string;
  description: string;
  category: ServiceCategory;
  requirements: string[];
  fee: number;
  currency: string;
  averageProcessingTime: string;
  averageProcessingDays: number;
  automationLevel: 'manual' | 'semi_automated' | 'fully_automated';
  availability: '24/7' | 'business_hours' | 'custom';
  requiredDocuments: string[];
  eligibilityCriteria?: string[];
}

// ============================================================================
// AI Assistant Types
// ============================================================================

export type MessageRole = 'user' | 'assistant' | 'system';
export type IntentType = string;

export interface Entity {
  type: string;
  value: string;
  confidence: number;
  startIndex?: number;
  endIndex?: number;
}

export interface Intent {
  detected: IntentType;
  confidence: number;
  entities: Entity[];
  alternativeIntents?: Array<{
    intent: string;
    confidence: number;
  }>;
}

export interface ChatMessage {
  messageId: string;
  role: MessageRole;
  content: string;
  timestamp: string;
  language: string;
  intent?: Intent;
  attachments?: Array<{
    type: string;
    url: string;
    name: string;
  }>;
}

export interface ConversationContext {
  department?: string;
  currentService?: string;
  history: string[];
  userData?: Record<string, any>;
  sessionVariables?: Record<string, any>;
}

export interface ChatSessionMetadata {
  channel: 'web' | 'mobile' | 'voice' | 'kiosk' | 'sms';
  device?: string;
  location?: string;
  ipAddress?: string;
  userAgent?: string;
}

export interface ChatSession {
  sessionId: string;
  citizenId: string;
  messages: ChatMessage[];
  context: ConversationContext;
  metadata: ChatSessionMetadata;
  createdAt: string;
  updatedAt: string;
  expiresAt: string;
  isActive: boolean;
}

export interface AIAssistantConfig {
  name: string;
  languages: string[];
  departments: string[];
  personality?: 'formal' | 'friendly' | 'professional';
  maxTokens?: number;
  temperature?: number;
  enableSuggestions?: boolean;
}

export interface AIResponse {
  messageId: string;
  response: string;
  suggestions?: string[];
  detectedIntent: IntentType;
  confidence: number;
  actions?: Array<{
    type: string;
    label: string;
    data: any;
  }>;
  timestamp: string;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface MetricValue {
  current: number;
  previous?: number;
  change?: number;
  changePercentage?: number;
}

export interface DashboardMetrics {
  totalRequests: MetricValue;
  automationRate: MetricValue;
  avgProcessingTime: MetricValue;
  citizenSatisfaction: MetricValue;
  breakdown: {
    byServiceType: Record<ServiceType, number>;
    byStatus: Record<RequestStatus, number>;
    byDepartment: Record<string, number>;
  };
  trends?: {
    daily: Array<{ date: string; value: number }>;
    weekly: Array<{ week: string; value: number }>;
    monthly: Array<{ month: string; value: number }>;
  };
}

export interface PredictionRequest {
  model: 'service_demand' | 'resource_allocation' | 'processing_time' | 'citizen_satisfaction';
  timeframe: string;
  parameters: Record<string, any>;
  granularity?: 'daily' | 'weekly' | 'monthly';
}

export interface PredictionResult {
  date: string;
  serviceType?: string;
  department?: string;
  expectedVolume?: number;
  expectedValue?: number;
  confidence: number;
  confidenceInterval?: {
    lower: number;
    upper: number;
  };
}

export interface AnalyticsEvent {
  eventId: string;
  eventType: 'service_request' | 'citizen_interaction' | 'system_event' | 'performance_metric';
  timestamp: string;
  source: {
    department: string;
    system: string;
    userId?: string;
  };
  data: {
    metric: string;
    value: number | string | object;
    dimensions: Record<string, any>;
  };
  tags: string[];
}

// ============================================================================
// Integration Types
// ============================================================================

export type IntegrationStatus = 'connected' | 'disconnected' | 'syncing' | 'error';
export type IntegrationType = 'smart_city' | 'payment_gateway' | 'identity_provider' | 'legacy_system' | 'third_party_api';

export interface Integration {
  integrationId: string;
  name: string;
  type: IntegrationType;
  status: IntegrationStatus;
  endpoint?: string;
  version?: string;
  lastSyncAt?: string;
  nextSyncAt?: string;
  config: Record<string, any>;
  healthCheck?: {
    lastCheck: string;
    status: 'healthy' | 'degraded' | 'unhealthy';
    responseTime?: number;
  };
}

export interface SmartCityEvent {
  eventId: string;
  sensorId: string;
  sensorType: 'traffic' | 'environmental' | 'infrastructure' | 'energy' | 'waste';
  location: {
    lat: number;
    lng: number;
    address?: string;
  };
  measurements: Array<{
    timestamp: string;
    metric: string;
    value: number;
    unit: string;
  }>;
  alert?: {
    level: 'info' | 'warning' | 'critical';
    message: string;
    action: string;
  };
}

export interface InterAgencyRequest {
  requestId: string;
  requestingAgency: string;
  targetAgency: string;
  dataRequest: {
    citizenId: string;
    purpose: string;
    dataFields: string[];
    legalBasis: string;
  };
  consent: {
    citizenConsent: boolean;
    consentTimestamp: string;
    consentMethod: 'explicit' | 'implied' | 'legal_requirement';
  };
  status: 'pending' | 'approved' | 'rejected' | 'completed';
  response?: {
    data: Record<string, any>;
    timestamp: string;
  };
}

// ============================================================================
// API Response Types
// ============================================================================

export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
    version: string;
  };
}

export interface PaginatedResponse<T> extends APIResponse<T[]> {
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalItems: number;
    hasNext: boolean;
    hasPrevious: boolean;
  };
}

// ============================================================================
// Event & Webhook Types
// ============================================================================

export type WebhookEvent =
  | 'service.created'
  | 'service.updated'
  | 'service.completed'
  | 'payment.received'
  | 'document.verified'
  | 'citizen.verified';

export interface Webhook {
  webhookId: string;
  url: string;
  events: WebhookEvent[];
  secret: string;
  active: boolean;
  retryPolicy: {
    maxAttempts: number;
    backoffMultiplier: number;
  };
}

export interface WebhookPayload {
  event: WebhookEvent;
  timestamp: string;
  data: any;
  signature: string;
}

// ============================================================================
// Error Types
// ============================================================================

export class SmartGovernmentError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'SmartGovernmentError';
  }
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  SmartGovernmentConfig,
  PersonalInfo,
  Address,
  ContactInfo,
  VerificationMethod,
  TrustLevel,
  IdentityVerification,
  AccessibilityPreferences,
  CitizenPreferences,
  CitizenProfile,
  ServiceType,
  ServiceCategory,
  Priority,
  RequestStatus,
  ServiceDocument,
  PaymentInfo,
  ApprovalRecord,
  WorkflowInfo,
  ServiceRequest,
  ServiceCatalogItem,
  MessageRole,
  IntentType,
  Entity,
  Intent,
  ChatMessage,
  ConversationContext,
  ChatSessionMetadata,
  ChatSession,
  AIAssistantConfig,
  AIResponse,
  MetricValue,
  DashboardMetrics,
  PredictionRequest,
  PredictionResult,
  AnalyticsEvent,
  IntegrationStatus,
  IntegrationType,
  Integration,
  SmartCityEvent,
  InterAgencyRequest,
  APIResponse,
  PaginatedResponse,
  WebhookEvent,
  Webhook,
  WebhookPayload,
};
