/**
 * WIA-UNI-004 Economic Integration SDK - Type Definitions
 *
 * @module @wia/economic-integration-sdk/types
 * @version 1.0.0
 */

// ============================================================================
// Core Types
// ============================================================================

export type Currency = string; // ISO 4217 currency code
export type Country = string;  // ISO 3166-1 alpha-2 country code
export type Timestamp = string; // ISO 8601 timestamp

export type TradeType = 'GOODS_EXPORT' | 'GOODS_IMPORT' | 'SERVICE_TRADE' | 'TECHNOLOGY_TRANSFER';

export type TradeStatus =
  | 'DRAFT'
  | 'SUBMITTED'
  | 'UNDER_REVIEW'
  | 'CUSTOMS_PROCESSING'
  | 'APPROVED'
  | 'REJECTED'
  | 'IN_TRANSIT'
  | 'DELIVERED';

export type InvestmentType =
  | 'FOREIGN_DIRECT_INVESTMENT'
  | 'PORTFOLIO_INVESTMENT'
  | 'INFRASTRUCTURE_DEVELOPMENT'
  | 'TECHNOLOGY_TRANSFER'
  | 'REAL_ESTATE';

export type InvestmentStatus =
  | 'DRAFT'
  | 'SUBMITTED'
  | 'UNDER_REVIEW'
  | 'APPROVED'
  | 'CONDITIONAL_APPROVAL'
  | 'REJECTED'
  | 'ACTIVE'
  | 'COMPLETED';

export type JVType = 'EQUITY_JV' | 'CONTRACTUAL_JV' | 'COOPERATIVE_ENTERPRISE';

export type SEZType =
  | 'INDUSTRIAL_COMPLEX'
  | 'FREE_TRADE_ZONE'
  | 'TECHNOLOGY_PARK'
  | 'EXPORT_PROCESSING_ZONE'
  | 'COMPREHENSIVE_SEZ';

// ============================================================================
// Configuration
// ============================================================================

export interface ClientConfig {
  apiKey: string;
  environment: 'production' | 'sandbox';
  baseURL?: string;
  timeout?: number;
  retries?: number;
}

// ============================================================================
// Money & Amount
// ============================================================================

export interface Money {
  amount: number;
  currency: Currency;
}

// ============================================================================
// Party Information
// ============================================================================

export interface Address {
  street: string;
  city: string;
  postalCode?: string;
  country: Country;
}

export interface Party {
  name: string;
  country: Country;
  address: Address;
  taxId?: string;
  registrationId?: string;
  contactPerson?: string;
  email?: string;
  phone?: string;
}

// ============================================================================
// Trade Operations
// ============================================================================

export interface TradeItem {
  description: string;
  hsCode: string;
  quantity: number;
  unit: string;
  unitPrice: number;
  totalValue: number;
  currency: Currency;
  originCountry: Country;
}

export interface CreateTradeRequest {
  tradeType: TradeType;
  exporter: Party;
  importer: Party;
  items: TradeItem[];
  totalValue: Money;
  paymentTerms: string;
  incoterms: string;
  estimatedDeliveryDate?: string;
}

export interface Trade {
  id: string;
  tradeType: TradeType;
  status: TradeStatus;
  exporter: Party;
  importer: Party;
  items: TradeItem[];
  totalValue: Money;
  createdAt: Timestamp;
  updatedAt: Timestamp;
  estimatedDeliveryDate?: string;
}

// ============================================================================
// Investment Operations
// ============================================================================

export interface InvestmentProject {
  name: string;
  location: string;
  sector: string;
  description: string;
  investmentAmount: Money;
  cashContribution: number;
  inKindContribution: number;
  timeline: {
    startDate: string;
    duration: number;
    unit: 'months' | 'years';
  };
  employment: {
    estimatedJobs: number;
    localHires: number;
    expatriates: number;
  };
}

export interface CreateInvestmentRequest {
  investmentType: InvestmentType;
  investor: Party;
  project: InvestmentProject;
  requestedIncentives?: string[];
}

export interface Investment {
  id: string;
  investmentType: InvestmentType;
  status: InvestmentStatus;
  investor: Party;
  project: InvestmentProject;
  approvedIncentives?: string[];
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

// ============================================================================
// Joint Venture Operations
// ============================================================================

export interface JVPartner {
  party: Party;
  equityShare: number;
  capitalContribution: {
    amount: number;
    currency: Currency;
    type: 'CASH' | 'IN_KIND';
    description?: string;
  };
}

export interface JVGovernance {
  boardSize: number;
  chairmanAppointment: 'ROTATING' | 'FIXED' | 'ELECTED';
  majorDecisionThreshold: number;
  ordinaryDecisionThreshold: number;
}

export interface CreateJointVentureRequest {
  jvType: JVType;
  ventureName: string;
  businessType: string;
  partners: JVPartner[];
  governance: JVGovernance;
  profitDistribution: {
    method: 'PROPORTIONAL_TO_EQUITY' | 'FIXED_RATIO' | 'PERFORMANCE_BASED';
    frequency: 'MONTHLY' | 'QUARTERLY' | 'ANNUAL';
    retentionRatio?: number;
  };
  disputeResolution: {
    method: 'NEGOTIATION' | 'MEDIATION' | 'INTERNATIONAL_ARBITRATION';
    venue?: string;
    governingLaw?: string;
  };
}

export interface JointVenture {
  id: string;
  jvType: JVType;
  ventureName: string;
  status: string;
  partners: JVPartner[];
  registeredAt?: Timestamp;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

// ============================================================================
// SEZ Operations
// ============================================================================

export interface CreateSEZRequest {
  sezType: SEZType;
  name: string;
  location: string;
  size: {
    area: number;
    unit: 'sq_km' | 'hectares';
  };
  primaryIndustries: string[];
  infrastructure: {
    electricity: boolean;
    water: boolean;
    gas: boolean;
    telecommunications: boolean;
    transportation: boolean;
  };
  incentives: string[];
}

export interface SEZ {
  id: string;
  sezType: SEZType;
  name: string;
  location: string;
  status: 'PLANNING' | 'UNDER_CONSTRUCTION' | 'OPERATIONAL' | 'SUSPENDED';
  activeEnterprises: number;
  employment: number;
  annualOutput?: Money;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

// ============================================================================
// Compliance & Risk
// ============================================================================

export interface ComplianceCheck {
  sanctionsScreening: {
    passed: boolean;
    lists: string[];
    checkDate: Timestamp;
  };
  amlCheck: {
    riskLevel: 'LOW' | 'MEDIUM' | 'HIGH';
    requiresEDD: boolean;
  };
  regulatoryCompliance: {
    compliant: boolean;
    issues?: string[];
  };
}

export interface RiskAssessment {
  politicalRisk: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  economicRisk: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  regulatoryRisk: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  operationalRisk: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  overallRisk: 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  assessmentDate: Timestamp;
  recommendations?: string[];
}

// ============================================================================
// API Response
// ============================================================================

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  timestamp: Timestamp;
}
