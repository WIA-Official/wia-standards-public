/**
 * WIA-FIN-020 Credit Scoring Standard - Type Definitions
 * Version: 2.0.0
 */

export interface CreditScoringConfig {
  apiKey: string;
  environment?: 'production' | 'sandbox' | 'development';
  baseUrl?: string;
  timeout?: number;
  retries?: number;
}

export interface Applicant {
  firstName: string;
  lastName: string;
  ssn: string; // Last 4 digits or full (encrypted)
  dateOfBirth: string; // ISO 8601 format
  email?: string;
  phone?: string;
}

export interface Address {
  street: string;
  city: string;
  state: string;
  zipCode: string;
  country?: string;
  residenceType?: 'own' | 'rent' | 'other';
  monthsAtAddress?: number;
}

export interface Employment {
  employer?: string;
  jobTitle?: string;
  annualIncome: number;
  employmentStatus: 'full-time' | 'part-time' | 'self-employed' | 'unemployed' | 'retired';
  monthsEmployed?: number;
  industry?: string;
}

export interface LoanRequest {
  amount: number;
  term?: number; // months
  purpose?: 'debt_consolidation' | 'home_improvement' | 'auto' | 'personal' | 'business' | 'other';
  type?: 'personal' | 'auto' | 'mortgage' | 'credit-card' | 'business';
}

export interface Application {
  applicant: Applicant;
  address?: Address;
  employment?: Employment;
  loanRequest: LoanRequest;
  alternativeData?: {
    includeBankData?: boolean;
    includeRentPayments?: boolean;
    includeUtilities?: boolean;
  };
}

export interface ScoreOptions {
  includeExplanation?: boolean;
  includeBureauData?: boolean;
  includeAlternativeData?: boolean;
  enableContinuousMonitoring?: boolean;
}

export interface ScoreFactor {
  name: string;
  value?: string | number;
  impact: 'high' | 'medium' | 'low';
  direction: 'positive' | 'negative' | 'neutral';
  description?: string;
}

export interface SuggestedTerms {
  apr: number;
  maxAmount: number;
  term?: number;
  monthlyPayment?: number;
}

export interface Decision {
  outcome: 'approve' | 'decline' | 'review';
  confidence: number; // 0-1
  reasons?: string[];
  alternativeScenarios?: Array<{
    condition: string;
    outcome: 'approve' | 'decline';
    probability: number;
  }>;
}

export interface ContinuousMonitoring {
  enabled: boolean;
  frequency: 'daily' | 'weekly' | 'monthly';
  nextUpdate?: string; // ISO 8601
  alertThresholds?: {
    scoreDecrease?: number;
    delinquency?: boolean;
    fraud?: boolean;
  };
}

export interface ScoreResult {
  id: string;
  version: string;
  timestamp: string;
  score: number;
  confidenceInterval?: [number, number];
  grade: 'excellent' | 'very-good' | 'good' | 'fair' | 'poor';
  decision: Decision;
  approvalProbability: number;
  defaultProbability: number;
  factors?: ScoreFactor[];
  suggestedTerms?: SuggestedTerms;
  continuousMonitoring?: ContinuousMonitoring;
  metadata?: Record<string, any>;
}

export interface AdverseActionNotice {
  applicantId: string;
  decisionDate: string;
  primaryReason: string;
  secondaryReasons: string[];
  creditBureaus: Array<{
    name: string;
    phone: string;
    website: string;
  }>;
  disputeProcess: string;
  fcraNotice: string;
}

export interface MonitoringAlert {
  accountId: string;
  alertType: 'score_decrease' | 'delinquency' | 'fraud' | 'credit_inquiry';
  severity: 'low' | 'medium' | 'high' | 'critical';
  timestamp: string;
  details: {
    previousScore?: number;
    currentScore?: number;
    change?: number;
    description: string;
  };
  actionRequired?: string;
}

export interface BatchScoreRequest {
  applications: Application[];
  callbackUrl?: string;
  priority?: 'low' | 'normal' | 'high';
}

export interface BatchScoreStatus {
  batchId: string;
  status: 'pending' | 'processing' | 'completed' | 'failed';
  totalApplications: number;
  processedApplications: number;
  estimatedCompletion?: string;
  results?: ScoreResult[];
  errors?: Array<{ applicationIndex: number; error: string }>;
}

export interface WebhookRegistration {
  url: string;
  events: string[];
  secret?: string;
  active?: boolean;
}

export interface WebhookEvent {
  id: string;
  type: string;
  timestamp: string;
  data: any;
  signature: string;
}

export interface PerformanceMetrics {
  period: string;
  auc: number;
  precision: number;
  recall: number;
  f1Score: number;
  approvalRate: number;
  defaultRate: number;
  psi: number; // Population Stability Index
}

export interface FairnessMetrics {
  period: string;
  disparateImpactRatio: number;
  demographicParity: number;
  equalOpportunity: number;
  calibrationByGroup: Record<string, {
    predicted: number;
    actual: number;
    difference: number;
  }>;
  pass: boolean;
  violations?: string[];
}
