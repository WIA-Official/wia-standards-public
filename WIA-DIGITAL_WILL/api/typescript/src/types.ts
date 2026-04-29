/**
 * WIA Digital Will Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADigitalWill {
  standard: 'WIA-DIGITAL-WILL';
  version: string;
  will: WillMetadata;
  testator: TestatorInfo;
  assets: DigitalAsset[];
  beneficiaries: Beneficiary[];
  instructions: WillInstruction[];
  witnesses: Witness[];
  executor: ExecutorDesignation;
  legal: LegalConfiguration;
  extensions?: Record<string, unknown>;
}

export interface WillMetadata {
  id: string;
  status: WillStatus;
  createdAt: string;
  updatedAt?: string;
  effectiveDate?: string;
  expiresAt?: string;
  revisionNumber: number;
  previousVersion?: string;
  jurisdiction: string;
  language: string;
}

export type WillStatus = 'draft' | 'pending-signatures' | 'signed' | 'notarized' | 'executed' | 'revoked';

export interface TestatorInfo {
  id: string;
  fullName: string;
  dateOfBirth: string;
  nationality: string;
  residenceAddress: Address;
  identificationDocuments: IdentityDocument[];
  mentalCapacity: CapacityDeclaration;
  status: 'alive' | 'deceased' | 'incapacitated';
}

export interface Address {
  street: string;
  city: string;
  state?: string;
  country: string;
  postalCode: string;
}

export interface IdentityDocument {
  type: 'passport' | 'national-id' | 'driver-license' | 'ssn';
  number: string;
  issuingCountry: string;
  expiryDate?: string;
  verified: boolean;
}

export interface CapacityDeclaration {
  declared: boolean;
  date: string;
  witnessedBy?: string;
  medicalCertificate?: string;
}

// ============================================================================
// Asset Types
// ============================================================================

export interface DigitalAsset {
  id: string;
  category: AssetCategory;
  name: string;
  description?: string;
  platform?: string;
  accountIdentifier?: string;
  estimatedValue?: MonetaryValue;
  accessInfo?: AccessInfo;
  disposition: AssetDisposition;
}

export type AssetCategory =
  | 'cryptocurrency'
  | 'nft'
  | 'social-media'
  | 'email'
  | 'cloud-storage'
  | 'domain'
  | 'digital-media'
  | 'gaming'
  | 'subscription'
  | 'business-account'
  | 'investment'
  | 'other';

export interface MonetaryValue {
  amount: number;
  currency: string;
  valuationDate: string;
}

export interface AccessInfo {
  method: 'password' | 'key-file' | 'hardware-key' | 'recovery-phrase' | 'biometric';
  encryptedCredentials?: string;
  storageLocation: string;
  lastVerified?: string;
}

export interface AssetDisposition {
  action: 'transfer' | 'liquidate' | 'delete' | 'memorialize' | 'archive';
  beneficiaryId?: string;
  conditions?: DispositionCondition[];
  specialInstructions?: string;
}

export interface DispositionCondition {
  type: 'age' | 'date' | 'event' | 'approval';
  parameters: Record<string, unknown>;
}

// ============================================================================
// Beneficiary Types
// ============================================================================

export interface Beneficiary {
  id: string;
  type: 'individual' | 'organization' | 'trust' | 'charity';
  name: string;
  relationship?: string;
  contactInfo: ContactInfo;
  identification?: IdentityDocument;
  share?: number;
  alternates?: AlternateBeneficiary[];
}

export interface ContactInfo {
  email: string;
  phone?: string;
  address?: Address;
}

export interface AlternateBeneficiary {
  beneficiaryId: string;
  condition: string;
  priority: number;
}

// ============================================================================
// Instruction Types
// ============================================================================

export interface WillInstruction {
  id: string;
  priority: number;
  type: InstructionType;
  description: string;
  details: Record<string, unknown>;
  assetIds?: string[];
  beneficiaryIds?: string[];
  conditions?: InstructionCondition[];
  executionDeadline?: string;
}

export type InstructionType =
  | 'asset-transfer'
  | 'account-closure'
  | 'data-deletion'
  | 'message-delivery'
  | 'memorial-creation'
  | 'donation'
  | 'notification'
  | 'custom';

export interface InstructionCondition {
  type: 'time-delay' | 'beneficiary-age' | 'event-trigger' | 'approval-required';
  parameters: Record<string, unknown>;
}

// ============================================================================
// Witness & Executor Types
// ============================================================================

export interface Witness {
  id: string;
  name: string;
  dateOfBirth?: string;
  address: Address;
  relationship: string;
  email: string;
  signatureDate?: string;
  signatureHash?: string;
  verified: boolean;
}

export interface ExecutorDesignation {
  primary: ExecutorInfo;
  alternates: ExecutorInfo[];
  powers: ExecutorPower[];
  compensation?: Compensation;
}

export interface ExecutorInfo {
  id: string;
  type: 'individual' | 'institution' | 'platform';
  name: string;
  contactInfo: ContactInfo;
  credentials?: string[];
  acceptanceStatus: 'pending' | 'accepted' | 'declined';
}

export type ExecutorPower =
  | 'access-accounts'
  | 'transfer-assets'
  | 'liquidate-assets'
  | 'close-accounts'
  | 'legal-representation'
  | 'dispute-resolution';

export interface Compensation {
  type: 'fixed' | 'percentage' | 'hourly';
  amount: number;
  currency?: string;
}

// ============================================================================
// Legal Types
// ============================================================================

export interface LegalConfiguration {
  governingLaw: string;
  notarization: NotarizationInfo;
  signatures: SignatureInfo[];
  revocationClause: string;
  disputeResolution: DisputeResolution;
  privacySettings: PrivacySettings;
}

export interface NotarizationInfo {
  required: boolean;
  notary?: NotaryInfo;
  date?: string;
  certificateNumber?: string;
}

export interface NotaryInfo {
  name: string;
  licenseNumber: string;
  jurisdiction: string;
  commission: string;
}

export interface SignatureInfo {
  signerId: string;
  signerType: 'testator' | 'witness' | 'executor' | 'notary';
  signatureHash: string;
  timestamp: string;
  method: 'digital' | 'electronic' | 'biometric';
  certificate?: string;
}

export interface DisputeResolution {
  method: 'arbitration' | 'mediation' | 'court';
  venue?: string;
  rules?: string;
}

export interface PrivacySettings {
  publicAfterExecution: boolean;
  redactedFields: string[];
  accessRestrictions: AccessRestriction[];
}

export interface AccessRestriction {
  partyType: 'beneficiary' | 'executor' | 'legal-counsel' | 'public';
  permissions: string[];
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface WillResponse {
  id: string;
  status: WillStatus;
  testatorName: string;
  assetCount: number;
  beneficiaryCount: number;
  createdAt: string;
  links: { self: string };
}

export interface ValidationResult {
  valid: boolean;
  errors?: { path: string; message: string }[];
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: { total: number; limit: number; offset: number; hasMore: boolean };
}
