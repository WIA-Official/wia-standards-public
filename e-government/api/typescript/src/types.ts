/**
 * WIA-SOC-003: E-Government Standard - TypeScript Type Definitions
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
 * UUID v4 string
 */
export type UUID = string;

/**
 * ISO 3166-1 alpha-2 country code
 */
export type CountryCode = string;

/**
 * ISO 639-1 language code
 */
export type LanguageCode = string;

// ============================================================================
// Authentication Types
// ============================================================================

/**
 * Authentication methods
 */
export enum AuthenticationMethod {
  BIOMETRIC = 'biometric',
  OTP = 'otp',
  PASSWORD = 'password',
  CERTIFICATE = 'certificate',
  TOKEN = 'token'
}

/**
 * Biometric types
 */
export enum BiometricType {
  FACE = 'face',
  FINGERPRINT = 'fingerprint',
  IRIS = 'iris',
  VOICE = 'voice'
}

/**
 * Authentication request
 */
export interface AuthenticationRequest {
  identityNumber: string;
  method: AuthenticationMethod;
  biometricData?: {
    type: BiometricType;
    data: string;
    metadata?: Record<string, any>;
  };
  otp?: string;
  password?: string;
  certificate?: string;
}

/**
 * Authentication response
 */
export interface AuthenticationResponse {
  success: boolean;
  token: string;
  refreshToken: string;
  expiresIn: number;
  citizen: CitizenSummary;
}

/**
 * Verification level
 */
export enum VerificationLevel {
  LOW = 'low',
  MEDIUM = 'medium',
  HIGH = 'high'
}

// ============================================================================
// Citizen Types
// ============================================================================

/**
 * Citizen identity
 */
export interface CitizenIdentity {
  '@context': string;
  '@type': 'CitizenIdentity';
  citizenId: UUID;
  identityNumber: string;
  givenName: string;
  familyName: string;
  dateOfBirth: string;
  nationality: CountryCode;
  residency: Residency;
  contact: ContactInfo;
  verification: VerificationInfo;
  privacySettings: PrivacySettings;
}

/**
 * Citizen summary (limited info)
 */
export interface CitizenSummary {
  id: UUID;
  name: string;
  verificationLevel: VerificationLevel;
}

/**
 * Residency information
 */
export interface Residency {
  country: CountryCode;
  region: string;
  district?: string;
  postalCode?: string;
}

/**
 * Contact information
 */
export interface ContactInfo {
  email?: string;
  phone?: string;
  preferredLanguage: LanguageCode;
}

/**
 * Verification information
 */
export interface VerificationInfo {
  level: VerificationLevel;
  methods: AuthenticationMethod[];
  lastVerified: Timestamp;
}

/**
 * Privacy settings
 */
export interface PrivacySettings {
  dataSharing: 'none' | 'minimal' | 'standard' | 'full';
  thirdPartyAccess: boolean;
  retentionPeriod: string;
}

// ============================================================================
// Service Types
// ============================================================================

/**
 * Service categories
 */
export enum ServiceCategory {
  TAXATION = 'taxation',
  HEALTHCARE = 'healthcare',
  CIVIL_AFFAIRS = 'civil_affairs',
  LEGAL = 'legal',
  SOCIAL_WELFARE = 'social_welfare',
  EDUCATION = 'education',
  TRANSPORTATION = 'transportation',
  HOUSING = 'housing'
}

/**
 * Service priority
 */
export enum ServicePriority {
  NORMAL = 'normal',
  URGENT = 'urgent',
  CRITICAL = 'critical'
}

/**
 * Service request status
 */
export enum ServiceRequestStatus {
  SUBMITTED = 'submitted',
  VERIFIED = 'verified',
  IN_PROGRESS = 'in_progress',
  PENDING_APPROVAL = 'pending_approval',
  APPROVED = 'approved',
  COMPLETED = 'completed',
  REJECTED = 'rejected',
  CANCELLED = 'cancelled'
}

/**
 * Government service
 */
export interface GovernmentService {
  '@type': 'GovernmentService';
  serviceId: string;
  serviceName: Record<LanguageCode, string>;
  description: Record<LanguageCode, string>;
  category: ServiceCategory;
  department: string;
  availability: ServiceAvailability;
  requirements: ServiceRequirements;
  processing: ProcessingInfo;
  fees: FeeInfo;
  endpoints: ServiceEndpoints;
}

/**
 * Service availability
 */
export interface ServiceAvailability {
  hours: string;
  maintenanceWindow?: string;
}

/**
 * Service requirements
 */
export interface ServiceRequirements {
  authentication: VerificationLevel;
  documents: string[];
  eligibility?: EligibilityCriteria;
}

/**
 * Eligibility criteria
 */
export interface EligibilityCriteria {
  minAge?: number;
  maxAge?: number;
  citizenshipRequired?: boolean;
  residencyRequired?: boolean;
}

/**
 * Processing information
 */
export interface ProcessingInfo {
  averageTime: string;
  maxTime: string;
  autoApproval: boolean;
}

/**
 * Fee information
 */
export interface FeeInfo {
  serviceFee: number;
  currency: string;
  paymentMethods: string[];
}

/**
 * Service endpoints
 */
export interface ServiceEndpoints {
  submitRequest: string;
  checkStatus: string;
  uploadDocument: string;
}

/**
 * Service request
 */
export interface ServiceRequest {
  '@type': 'ServiceRequest';
  requestId: string;
  citizenId: UUID;
  serviceType: string;
  category: ServiceCategory;
  priority: ServicePriority;
  status: ServiceRequestStatus;
  createdAt: Timestamp;
  updatedAt: Timestamp;
  estimatedCompletion?: Timestamp;
  metadata: Record<string, any>;
  documents: string[];
  attachments: string[];
  notes?: string;
  assignedTo?: string;
  tracking: RequestTracking;
}

/**
 * Request tracking
 */
export interface RequestTracking {
  steps: RequestStep[];
}

/**
 * Request step
 */
export interface RequestStep {
  step: string;
  status: 'pending' | 'in_progress' | 'completed' | 'failed';
  completedAt?: Timestamp;
  estimatedCompletion?: Timestamp;
}

/**
 * Service request submission parameters
 */
export interface ServiceRequestParams {
  serviceId: string;
  priority?: ServicePriority;
  metadata?: Record<string, any>;
  documents?: string[];
  notes?: string;
}

// ============================================================================
// Document Types
// ============================================================================

/**
 * Document type
 */
export enum DocumentType {
  CERTIFICATE = 'certificate',
  FORM = 'form',
  RECEIPT = 'receipt',
  LICENSE = 'license',
  PERMIT = 'permit',
  IDENTITY = 'identity',
  PROOF = 'proof'
}

/**
 * Government document
 */
export interface GovernmentDocument {
  '@type': 'GovernmentDocument';
  documentId: string;
  type: DocumentType;
  subtype: string;
  issuedBy: IssuingAuthority;
  issuedTo: DocumentRecipient;
  issuedDate: string;
  validFrom: string;
  validUntil?: string;
  documentNumber: string;
  content: DocumentContent;
  verification: DocumentVerification;
  qrCode?: QRCode;
  blockchain?: BlockchainInfo;
}

/**
 * Issuing authority
 */
export interface IssuingAuthority {
  authority: string;
  department: string;
  officerName?: string;
  officerId?: string;
}

/**
 * Document recipient
 */
export interface DocumentRecipient {
  citizenId: UUID;
  name: string;
}

/**
 * Document content
 */
export interface DocumentContent {
  format: string;
  encoding: string;
  data: string;
  hash: string;
}

/**
 * Document verification
 */
export interface DocumentVerification {
  method: 'digital_signature' | 'blockchain' | 'qr_code';
  algorithm?: string;
  signature?: string;
  certificate?: string;
}

/**
 * QR Code
 */
export interface QRCode {
  data: string;
  format: string;
  version: number;
}

/**
 * Blockchain information
 */
export interface BlockchainInfo {
  network: string;
  txHash: string;
  blockNumber: number;
}

/**
 * Document upload parameters
 */
export interface DocumentUploadParams {
  file: File | Buffer;
  type: string;
  subtype?: string;
  metadata?: Record<string, any>;
  encryption?: boolean;
}

// ============================================================================
// Payment Types
// ============================================================================

/**
 * Payment method
 */
export enum PaymentMethod {
  CREDIT_CARD = 'credit_card',
  DEBIT_CARD = 'debit_card',
  BANK_TRANSFER = 'bank_transfer',
  DIGITAL_WALLET = 'digital_wallet',
  CRYPTOCURRENCY = 'cryptocurrency'
}

/**
 * Payment status
 */
export enum PaymentStatus {
  PENDING = 'pending',
  PROCESSING = 'processing',
  COMPLETED = 'completed',
  FAILED = 'failed',
  CANCELLED = 'cancelled',
  REFUNDED = 'refunded'
}

/**
 * Payment information
 */
export interface Payment {
  paymentId: string;
  requestId: string;
  amount: number;
  currency: string;
  status: PaymentStatus;
  method: PaymentMethod;
  transactionId?: string;
  paidAt?: Timestamp;
  receipt?: ReceiptInfo;
}

/**
 * Receipt information
 */
export interface ReceiptInfo {
  receiptId: string;
  downloadUrl: string;
}

/**
 * Payment initiation parameters
 */
export interface PaymentParams {
  requestId: string;
  amount: number;
  currency: string;
  method: PaymentMethod;
  returnUrl?: string;
}

// ============================================================================
// Notification Types
// ============================================================================

/**
 * Notification channel
 */
export enum NotificationChannel {
  EMAIL = 'email',
  SMS = 'sms',
  PUSH = 'push',
  WEBSOCKET = 'websocket'
}

/**
 * Notification event type
 */
export enum NotificationEventType {
  SERVICE_UPDATE = 'service_update',
  DOCUMENT_READY = 'document_ready',
  PAYMENT_RECEIVED = 'payment_received',
  REQUEST_APPROVED = 'request_approved',
  REQUEST_REJECTED = 'request_rejected'
}

/**
 * Notification
 */
export interface Notification {
  type: NotificationEventType;
  requestId?: string;
  status?: string;
  message: string;
  timestamp: Timestamp;
  data?: Record<string, any>;
}

/**
 * Notification subscription
 */
export interface NotificationSubscription {
  channels: NotificationChannel[];
  events: NotificationEventType[];
  preferences: NotificationPreferences;
}

/**
 * Notification preferences
 */
export interface NotificationPreferences {
  email?: string;
  phone?: string;
  pushToken?: string;
}

// ============================================================================
// API Client Types
// ============================================================================

/**
 * API client configuration
 */
export interface ApiClientOptions {
  countryCode: CountryCode;
  apiKey: string;
  endpoint?: string;
  timeout?: number;
  environment?: 'production' | 'sandbox';
}

/**
 * API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: ApiError;
  requestId?: string;
  timestamp?: Timestamp;
}

/**
 * API error
 */
export interface ApiError {
  code: string;
  message: string;
  details?: Record<string, any>;
  requestId?: string;
  timestamp?: Timestamp;
}

/**
 * Pagination info
 */
export interface PaginationInfo {
  page: number;
  limit: number;
  total: number;
  pages: number;
  hasNext: boolean;
  hasPrevious: boolean;
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  data: T[];
  pagination: PaginationInfo;
}

// ============================================================================
// Analytics Types
// ============================================================================

/**
 * Public analytics
 */
export interface PublicAnalytics {
  period: {
    start: string;
    end: string;
  };
  statistics: {
    totalRequests: number;
    completedRequests: number;
    averageProcessingTime: string;
    citizenSatisfaction: number;
    topServices: ServiceStats[];
  };
}

/**
 * Service statistics
 */
export interface ServiceStats {
  serviceId: string;
  name: string;
  requests: number;
  satisfaction?: number;
}

// ============================================================================
// Export all types
// ============================================================================

export * from './types';
