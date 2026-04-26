/**
 * WIA-MED-025: Healthcare Blockchain Standard - TypeScript Types
 *
 * 弘益人間 (홍익인간) - Benefit All Humanity
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// Enums
// ============================================================================

export enum RecordStatus {
  ACTIVE = 'active',
  INACTIVE = 'inactive',
  PENDING = 'pending',
  REVOKED = 'revoked',
}

export enum TransactionType {
  CREATE = 'create',
  UPDATE = 'update',
  ACCESS = 'access',
  CONSENT = 'consent',
  REVOKE = 'revoke',
}

export enum ConsentType {
  FULL_ACCESS = 'full_access',
  READ_ONLY = 'read_only',
  EMERGENCY = 'emergency',
  RESEARCH = 'research',
}

export enum DataCategory {
  DEMOGRAPHICS = 'demographics',
  DIAGNOSES = 'diagnoses',
  MEDICATIONS = 'medications',
  PROCEDURES = 'procedures',
  LAB_RESULTS = 'lab_results',
  IMAGING = 'imaging',
}

// ============================================================================
// Blockchain Core Types
// ============================================================================

export interface Block {
  blockId: string;
  previousHash: string;
  hash: string;
  timestamp: string;
  transactions: Transaction[];
  nonce: number;
  merkleRoot: string;
}

export interface Transaction {
  transactionId: string;
  type: TransactionType;
  timestamp: string;
  sender: string;
  recipient?: string;
  dataHash: string;
  signature: string;
  status: 'pending' | 'confirmed' | 'failed';
}

// ============================================================================
// Health Record Types
// ============================================================================

export interface HealthRecord {
  recordId: string;
  patientId: string;
  providerId: string;
  dataCategory: DataCategory;
  encryptedData: string;
  dataHash: string;
  status: RecordStatus;
  createdAt: string;
  updatedAt: string;
  accessLog: AccessLogEntry[];
}

export interface AccessLogEntry {
  accessId: string;
  accessorId: string;
  accessType: 'read' | 'write';
  timestamp: string;
  purpose: string;
}

export interface PatientIdentity {
  patientId: string;
  publicKey: string;
  didDocument: DIDDocument;
  status: RecordStatus;
}

export interface DIDDocument {
  id: string;
  controller: string;
  verificationMethod: VerificationMethod[];
  authentication: string[];
}

export interface VerificationMethod {
  id: string;
  type: string;
  controller: string;
  publicKeyMultibase: string;
}

// ============================================================================
// Consent Types
// ============================================================================

export interface ConsentRecord {
  consentId: string;
  patientId: string;
  granteeId: string;
  consentType: ConsentType;
  dataCategories: DataCategory[];
  purpose: string;
  validFrom: string;
  validUntil?: string;
  status: RecordStatus;
  signature: string;
}

export interface ConsentRequest {
  requestId: string;
  requesterId: string;
  patientId: string;
  consentType: ConsentType;
  dataCategories: DataCategory[];
  purpose: string;
  status: 'pending' | 'approved' | 'denied';
}

// ============================================================================
// Smart Contract Types
// ============================================================================

export interface SmartContract {
  contractId: string;
  name: string;
  version: string;
  abi: ContractABI[];
  status: 'active' | 'deprecated';
}

export interface ContractABI {
  name: string;
  type: 'function' | 'event';
  inputs: ABIParameter[];
  outputs?: ABIParameter[];
}

export interface ABIParameter {
  name: string;
  type: string;
}

// ============================================================================
// API Types
// ============================================================================

export interface WIAConfig {
  apiKey: string;
  endpoint: string;
  timeout?: number;
  debug?: boolean;
}

export interface APIResponse<T = unknown> {
  success: boolean;
  data?: T;
  error?: APIError;
  timestamp: string;
}

export interface APIError {
  code: string;
  message: string;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalPages: number;
    totalCount: number;
  };
}
