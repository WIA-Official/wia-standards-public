/**
 * WIA-FIN-018 RegTech Standard - TypeScript Type Definitions
 * @module @wia/regtech-sdk
 */

/**
 * Network/blockchain identifier
 */
export type Network = 'mainnet' | 'testnet' | 'devnet';

/**
 * Jurisdiction code (ISO 3166-1 alpha-2)
 */
export type Jurisdiction = string;

/**
 * Risk level classification
 */
export type RiskLevel = 'low' | 'medium' | 'high' | 'prohibited';

/**
 * Compliance status
 */
export type ComplianceStatus = 'compliant' | 'review' | 'flagged' | 'blocked';

/**
 * KYC verification status
 */
export type KYCStatus = 'pending' | 'verified' | 'rejected' | 'expired';

/**
 * Event type classification
 */
export type EventType = 'transaction' | 'customer_action' | 'system_event' | 'external_event';

/**
 * Report type classification
 */
export type ReportType = 'sar' | 'ctr' | 'kyc' | 'gdpr' | 'audit' | 'custom';

/**
 * Client configuration options
 */
export interface RegTechClientConfig {
  /** API key for authentication */
  apiKey: string;
  /** Base URL for API endpoints */
  baseUrl?: string;
  /** Network environment */
  network?: Network;
  /** Default jurisdiction for operations */
  jurisdiction?: Jurisdiction;
  /** Request timeout in milliseconds */
  timeout?: number;
  /** Enable debug logging */
  debug?: boolean;
}

/**
 * Compliance event data
 */
export interface ComplianceEvent {
  /** Unique event identifier */
  eventId: string;
  /** Event timestamp */
  timestamp: string;
  /** Type of event */
  eventType: EventType;
  /** Jurisdiction code */
  jurisdiction: Jurisdiction;
  /** Compliance flags and scores */
  complianceFlags: {
    riskScore: number;
    requiresReview: boolean;
    requiresSAR: boolean;
    requiresCTR: boolean;
  };
  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Customer personal information
 */
export interface CustomerPersonalInfo {
  /** Full legal name */
  fullName: string;
  /** Date of birth (ISO 8601) */
  dateOfBirth: string;
  /** Nationality (ISO 3166-1) */
  nationality: string;
  /** Tax identifier (encrypted) */
  taxId?: string;
  /** Residential address */
  address?: {
    street: string;
    city: string;
    state?: string;
    postalCode: string;
    country: string;
  };
}

/**
 * KYC verification details
 */
export interface KYCVerification {
  /** Verification status */
  kycStatus: KYCStatus;
  /** Verification date */
  verificationDate: string;
  /** Verification method used */
  verificationMethod: string;
  /** Documents provided for verification */
  documentsProvided: string[];
}

/**
 * Customer risk profile
 */
export interface CustomerRiskProfile {
  /** Risk level classification */
  riskLevel: RiskLevel;
  /** Politically Exposed Person flag */
  isPEP: boolean;
  /** Sanctions list match */
  sanctionsMatch: boolean;
  /** Adverse media findings */
  adverseMedia?: Array<{
    source: string;
    date: string;
    summary: string;
    severity: 'low' | 'medium' | 'high';
  }>;
  /** Last risk review date */
  lastReviewDate?: string;
}

/**
 * Customer data model
 */
export interface Customer {
  /** Unique customer identifier */
  customerId: string;
  /** Personal information */
  personalInfo: CustomerPersonalInfo;
  /** Verification details */
  verification: KYCVerification;
  /** Risk profile */
  riskProfile: CustomerRiskProfile;
  /** Compliance history */
  complianceHistory?: {
    sarFiled: number;
    flaggedTransactions: number;
    averageMonthlyVolume: number;
  };
}

/**
 * Transaction party information
 */
export interface TransactionParty {
  /** Customer reference or external identifier */
  customerId?: string;
  /** Party name */
  name: string;
  /** Account number */
  accountNumber: string;
  /** Bank/institution identifier */
  bank?: string;
  /** Country code */
  country: string;
}

/**
 * Screening results
 */
export interface ScreeningResults {
  /** Sanctions screening */
  sanctions: {
    checked: boolean;
    matches: number;
    lists: string[];
  };
  /** PEP screening */
  pep: {
    checked: boolean;
    isPEP: boolean;
    category?: string;
  };
  /** Adverse media screening */
  adverseMedia: {
    checked: boolean;
    findings: number;
    summary?: string;
  };
}

/**
 * Compliance alert
 */
export interface ComplianceAlert {
  /** Alert identifier */
  alertId: string;
  /** Alert type */
  type: string;
  /** Severity level */
  severity: 'low' | 'medium' | 'high' | 'critical';
  /** Alert message */
  message: string;
  /** Alert timestamp */
  timestamp: string;
  /** Recommended actions */
  recommendations?: string[];
}

/**
 * Transaction data model
 */
export interface Transaction {
  /** Unique transaction identifier */
  transactionId: string;
  /** Transaction timestamp */
  timestamp: string;
  /** Transaction type */
  type: string;
  /** Transaction amount */
  amount: number;
  /** Currency code (ISO 4217) */
  currency: string;
  /** Transaction parties */
  parties: {
    originator: TransactionParty;
    beneficiary: TransactionParty;
  };
  /** Compliance information */
  compliance: {
    riskScore: number;
    alerts: ComplianceAlert[];
    screening: ScreeningResults;
    status?: ComplianceStatus;
  };
}

/**
 * Compliance check request
 */
export interface ComplianceCheckRequest {
  /** Transaction identifier */
  transactionId: string;
  /** Transaction amount */
  amount: number;
  /** Currency code */
  currency: string;
  /** Jurisdiction */
  jurisdiction: Jurisdiction;
  /** Customer risk profile */
  customerRiskProfile: RiskLevel;
  /** Additional context */
  context?: Record<string, any>;
}

/**
 * Compliance check response
 */
export interface ComplianceCheckResponse {
  /** Risk score (0-100) */
  riskScore: number;
  /** Compliance status */
  status: ComplianceStatus;
  /** SAR filing required */
  requiresSAR: boolean;
  /** CTR filing required */
  requiresCTR: boolean;
  /** Compliance alerts */
  alerts: ComplianceAlert[];
  /** Recommendations */
  recommendations: string[];
  /** Processing timestamp */
  timestamp: string;
}

/**
 * Document for KYC verification
 */
export interface KYCDocument {
  /** Document type */
  type: 'passport' | 'drivers_license' | 'national_id' | 'utility_bill' | 'bank_statement';
  /** Document number */
  documentNumber: string;
  /** Issuing country */
  issuingCountry: string;
  /** Expiry date */
  expiryDate?: string;
  /** Document image (base64 encoded) */
  image: string;
}

/**
 * KYC verification request
 */
export interface KYCVerificationRequest {
  /** Customer identifier */
  customerId: string;
  /** Documents for verification */
  documents: KYCDocument[];
  /** Biometric data (optional) */
  biometric?: {
    faceImage: string;
    livenessVideo?: string;
  };
}

/**
 * Verification match
 */
export interface VerificationMatch {
  /** Field that was verified */
  field: string;
  /** Match confidence (0-1) */
  confidence: number;
  /** Verification source */
  source: string;
}

/**
 * KYC verification response
 */
export interface KYCVerificationResponse {
  /** Verification identifier */
  verificationId: string;
  /** Verification status */
  status: KYCStatus;
  /** Overall confidence score (0-1) */
  confidence: number;
  /** Verification matches */
  matches: VerificationMatch[];
  /** Issues found */
  issues: string[];
  /** Timestamp */
  timestamp: string;
}

/**
 * Regulatory report data
 */
export interface RegulatoryReport {
  /** Report identifier */
  reportId: string;
  /** Report type */
  reportType: ReportType;
  /** Reporting period */
  reportingPeriod: {
    start: string;
    end: string;
  };
  /** Report data (type-specific) */
  data: Record<string, any>;
  /** Jurisdiction */
  jurisdiction: Jurisdiction;
}

/**
 * Report submission response
 */
export interface ReportSubmissionResponse {
  /** Submission identifier */
  submissionId: string;
  /** Submission status */
  status: 'accepted' | 'rejected' | 'pending';
  /** Confirmation number */
  confirmationNumber?: string;
  /** Validation errors */
  errors: Array<{
    field: string;
    message: string;
    code: string;
  }>;
  /** Warnings */
  warnings: string[];
  /** Submission timestamp */
  timestamp: string;
}

/**
 * Sanctions screening request
 */
export interface SanctionsScreeningRequest {
  /** Entity to screen */
  entity: {
    name: string;
    type: 'person' | 'organization';
    country?: string;
    dateOfBirth?: string;
  };
  /** Lists to check against */
  lists?: string[];
}

/**
 * Sanctions match
 */
export interface SanctionsMatch {
  /** List name */
  list: string;
  /** Match confidence (0-1) */
  confidence: number;
  /** Matched entry details */
  entry: {
    name: string;
    aliases: string[];
    program: string;
    country?: string;
  };
}

/**
 * Sanctions screening response
 */
export interface SanctionsScreeningResponse {
  /** Screening identifier */
  screeningId: string;
  /** Match found */
  match: boolean;
  /** Matches details */
  matches: SanctionsMatch[];
  /** Screening timestamp */
  timestamp: string;
}

/**
 * API error response
 */
export interface APIError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Additional details */
  details?: Record<string, any>;
  /** HTTP status code */
  statusCode: number;
}

/**
 * Compliance rule definition
 */
export interface ComplianceRule {
  /** Rule identifier */
  ruleId: string;
  /** Rule name */
  name: string;
  /** Rule description */
  description: string;
  /** Rule type */
  type: 'transaction_limit' | 'velocity_check' | 'pattern_detection' | 'sanctions_screen' | 'custom';
  /** Rule severity */
  severity: 'low' | 'medium' | 'high' | 'critical';
  /** Rule enabled status */
  enabled: boolean;
  /** Rule parameters */
  parameters: Record<string, any>;
  /** Applicable jurisdictions */
  jurisdictions: Jurisdiction[];
  /** Rule actions */
  actions: Array<{
    type: 'alert' | 'block' | 'review' | 'report';
    configuration: Record<string, any>;
  }>;
}

/**
 * KYC record for a customer
 */
export interface KYCRecord {
  /** Record identifier */
  recordId: string;
  /** Customer identifier */
  customerId: string;
  /** Customer personal information */
  personalInfo: CustomerPersonalInfo;
  /** Verification status */
  status: KYCStatus;
  /** Verification date */
  verificationDate: string;
  /** Documents submitted */
  documents: KYCDocument[];
  /** Verification level */
  verificationLevel: 'basic' | 'standard' | 'enhanced';
  /** Risk assessment */
  riskAssessment: {
    level: RiskLevel;
    score: number;
    factors: string[];
  };
  /** Screening results */
  screening: {
    sanctionsChecked: boolean;
    pepChecked: boolean;
    adverseMediaChecked: boolean;
    results: ScreeningResults;
  };
  /** Last update timestamp */
  lastUpdated: string;
  /** Record expiry date */
  expiryDate?: string;
}

/**
 * AML alert for suspicious activity
 */
export interface AMLAlert {
  /** Alert identifier */
  alertId: string;
  /** Alert type */
  type: 'sanctions_match' | 'pep_detected' | 'unusual_activity' | 'high_risk_country' | 'structuring' | 'other';
  /** Alert severity */
  severity: 'low' | 'medium' | 'high' | 'critical';
  /** Alert status */
  status: 'open' | 'investigating' | 'closed' | 'escalated';
  /** Alert creation timestamp */
  createdAt: string;
  /** Associated transaction ID */
  transactionId?: string;
  /** Associated customer ID */
  customerId?: string;
  /** Alert description */
  description: string;
  /** Detected patterns or indicators */
  indicators: string[];
  /** Risk score (0-100) */
  riskScore: number;
  /** Recommended actions */
  recommendedActions: string[];
  /** SAR filing required */
  requiresSAR: boolean;
  /** Investigation notes */
  notes?: string;
  /** Assigned investigator */
  assignedTo?: string;
  /** Resolution details */
  resolution?: {
    decision: 'false_positive' | 'confirmed' | 'escalated';
    reason: string;
    resolvedAt: string;
    resolvedBy: string;
  };
}
