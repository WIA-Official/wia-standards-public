/**
 * WIA-UNI-014: Legal System Harmonization
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 */

// Core Types
export type JurisdictionSystem = 'north' | 'south' | 'unified';
export type JurisdictionLevel = 'national' | 'provincial' | 'municipal';
export type DocumentType = 'law' | 'regulation' | 'judgment' | 'contract' | 'property' | 'certificate';
export type DocumentStatus = 'active' | 'superseded' | 'repealed' | 'draft';
export type Language = 'ko' | 'en';
export type OwnershipType = 'private' | 'collective' | 'state' | 'mixed';

// Core Document Interface
export interface LegalDocument {
  standard: 'WIA-UNI-014';
  version: string;
  document: {
    id: string;
    type: DocumentType;
    jurisdiction: Jurisdiction;
    metadata: Metadata;
    content: Content;
    relationships?: Relationships;
    harmonization?: Harmonization;
    verification?: Verification;
  };
}

// Jurisdiction
export interface Jurisdiction {
  system: JurisdictionSystem;
  level: JurisdictionLevel;
  authority: string;
}

// Metadata
export interface Metadata {
  title: string;
  shortTitle?: string;
  issueDate: string;
  effectiveDate?: string;
  status: DocumentStatus;
  language: Language;
  classification?: 'public' | 'restricted' | 'confidential';
  keywords?: string[];
  legalAreas?: string[];
}

// Content
export interface Content {
  text: string;
  structure?: DocumentStructure;
  translations?: Translation[];
  authoritative?: Language;
  attachments?: Attachment[];
}

export interface DocumentStructure {
  chapters?: Chapter[];
  articles?: Article[];
  sections?: Section[];
}

export interface Chapter {
  number: number;
  title: string;
  articles?: Article[];
}

export interface Article {
  number: number;
  title?: string;
  text: string;
  sections?: Section[];
}

export interface Section {
  number: number;
  title?: string;
  text: string;
}

export interface Translation {
  language: Language;
  text: string;
  official: boolean;
  translator?: string;
  note?: string;
}

export interface Attachment {
  id: string;
  type: string;
  filename: string;
  url: string;
}

// Relationships
export interface Relationships {
  amends?: string[];
  supersedes?: string[];
  references?: string[];
  implementedBy?: string[];
}

// Harmonization
export interface Harmonization {
  northCompliance: boolean;
  southCompliance: boolean;
  conflicts?: Conflict[];
  resolutionStatus: 'unresolved' | 'proposed' | 'resolved';
  harmonizedVersion?: string;
}

export interface Conflict {
  type: string;
  description: string;
  severity: 'high' | 'medium' | 'low';
}

// Verification
export interface Verification {
  hash: string;
  algorithm?: string;
  timestamp: string;
  authority: string;
  signature?: string;
  certificate?: string;
  blockchain?: BlockchainRecord;
}

export interface BlockchainRecord {
  network: string;
  txHash: string;
  blockNumber: number;
}

// Property-Specific Types
export interface PropertyDocument extends LegalDocument {
  document: LegalDocument['document'] & {
    propertySpecific: PropertySpecific;
  };
}

export interface PropertySpecific {
  propertyId: string;
  propertyType: 'land' | 'building' | 'apartment' | 'commercial';
  location: Location;
  ownership: Ownership;
  history?: PropertyHistoryEntry[];
  encumbrances?: Encumbrance[];
}

export interface Location {
  address: string;
  coordinates?: {
    latitude: number;
    longitude: number;
  };
  cadastral?: string;
}

export interface Ownership {
  currentOwner: string; // WIA-UNI-001 ID
  ownershipType: OwnershipType;
  acquisitionDate: string;
  acquisitionMethod: 'purchase' | 'inheritance' | 'grant' | 'allocation';
  transferAmount?: number;
  currency?: string;
}

export interface PropertyHistoryEntry {
  date: string;
  event: 'transfer' | 'encumbrance' | 'modification';
  fromOwner?: string;
  toOwner?: string;
  documentId?: string;
}

export interface Encumbrance {
  type: 'mortgage' | 'lien' | 'easement' | 'restriction';
  holder: string;
  amount?: number;
  currency?: string;
  priority: number;
}

// Contract-Specific Types
export interface ContractDocument extends LegalDocument {
  document: LegalDocument['document'] & {
    contractSpecific: ContractSpecific;
  };
}

export interface ContractSpecific {
  contractType: 'sales' | 'lease' | 'employment' | 'service' | 'partnership';
  parties: ContractParty[];
  terms: ContractTerms;
  signatures?: ContractSignature[];
}

export interface ContractParty {
  id: string; // WIA-UNI-001 ID
  role: string;
  name: string;
  jurisdiction?: JurisdictionSystem;
}

export interface ContractTerms {
  subject: string;
  consideration?: Consideration;
  duration?: Duration;
  conditions?: string[];
}

export interface Consideration {
  amount: number;
  currency: string;
  paymentTerms?: string;
}

export interface Duration {
  start: string;
  end: string;
}

export interface ContractSignature {
  party: string;
  timestamp: string;
  signature: string;
}

// Judgment-Specific Types
export interface JudgmentDocument extends LegalDocument {
  document: LegalDocument['document'] & {
    judgmentSpecific: JudgmentSpecific;
  };
}

export interface JudgmentSpecific {
  court: Court;
  case: Case;
  outcome: Outcome;
  precedential?: Precedential;
}

export interface Court {
  name: string;
  level: 'supreme' | 'appellate' | 'trial';
  jurisdiction: JurisdictionSystem;
}

export interface Case {
  caseNumber: string;
  parties: {
    plaintiff: string[];
    defendant: string[];
  };
  filingDate?: string;
  judgmentDate: string;
}

export interface Outcome {
  decision: 'granted' | 'denied' | 'partiallyGranted';
  reasoning?: string;
  remedies?: string[];
}

export interface Precedential {
  binding: boolean;
  scope?: string;
}

// API Response Types
export interface APIResponse<T> {
  data: T;
  success: boolean;
  timestamp: string;
}

export interface SearchResponse {
  results: LegalDocument[];
  total: number;
  pagination: Pagination;
}

export interface Pagination {
  limit: number;
  offset: number;
  total: number;
  hasNext: boolean;
  hasPrev: boolean;
  links?: {
    first?: string;
    prev?: string;
    next?: string;
    last?: string;
  };
}

export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any[];
    requestId?: string;
    timestamp: string;
  };
}

// Client Configuration
export interface ClientConfig {
  apiKey: string;
  environment?: 'production' | 'sandbox';
  baseURL?: string;
  timeout?: number;
}

// API Request Options
export interface SearchOptions {
  query?: string;
  type?: DocumentType;
  jurisdiction?: JurisdictionSystem;
  status?: DocumentStatus;
  limit?: number;
  offset?: number;
}

export interface PropertyTransferRequest {
  propertyId: string;
  buyer: string; // WIA-UNI-001 ID
  seller: string; // WIA-UNI-001 ID
  price: number;
  currency: string;
  effectiveDate?: string;
}

export interface ContractCreateRequest {
  type: ContractSpecific['contractType'];
  parties: ContractParty[];
  jurisdiction: Jurisdiction;
  terms: ContractTerms;
}
