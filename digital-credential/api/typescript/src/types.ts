/**
 * WIA-EDU-011 Digital Credential Standard - TypeScript Type Definitions
 * Version: 2.0.0
 * Philosophy: 弘益人間 (Benefit All Humanity)
 */

/**
 * WIA Digital Credential Standard Version
 */
export type WIAVersion = '1.0' | '1.1' | '1.2' | '2.0';

/**
 * Credential Types
 */
export type CredentialType =
  | 'BachelorDegree'
  | 'MasterDegree'
  | 'DoctoralDegree'
  | 'AssociateDegree'
  | 'ProfessionalCertificate'
  | 'CompletionCertificate'
  | 'TrainingCertificate'
  | 'MicroCredential'
  | 'HighSchoolDiploma'
  | 'UniversityDiploma'
  | 'AcademicTranscript';

/**
 * Credential Status
 */
export type CredentialStatus = 'active' | 'revoked' | 'suspended' | 'expired';

/**
 * Blockchain Networks
 */
export type BlockchainNetwork =
  | 'ethereum'
  | 'polygon'
  | 'arbitrum'
  | 'sepolia'
  | 'mumbai';

/**
 * W3C Verifiable Credential
 */
export interface VerifiableCredential {
  '@context': string[];
  id: string;
  type: string[];
  issuer: Issuer;
  issuanceDate: string;
  expirationDate?: string;
  credentialSubject: CredentialSubject;
  proof: Proof;
  credentialStatus?: CredentialStatusInfo;
}

/**
 * Credential Issuer
 */
export interface Issuer {
  id: string; // DID
  name: string;
  type?: string;
  url?: string;
  country?: string;
  accreditation?: Accreditation[];
}

/**
 * Accreditation Information
 */
export interface Accreditation {
  body: string;
  status: string;
  validUntil?: string;
}

/**
 * Credential Subject (Holder)
 */
export interface CredentialSubject {
  id: string; // DID
  type?: string;
  name?: string;
  email?: string;
  achievement?: Achievement;
}

/**
 * Educational Achievement
 */
export interface Achievement {
  type: CredentialType;
  name: string;
  field?: string;
  level?: string;
  gpa?: string;
  graduationDate?: string;
  honors?: string;
  iscedLevel?: string;
  eqfLevel?: number;
}

/**
 * Cryptographic Proof
 */
export interface Proof {
  type: string;
  created: string;
  verificationMethod: string;
  proofPurpose: string;
  proofValue: string;
  blockchainAnchor?: BlockchainAnchor;
}

/**
 * Blockchain Anchoring Information
 */
export interface BlockchainAnchor {
  network: BlockchainNetwork;
  txHash: string;
  blockNumber?: number;
  timestamp?: string;
  contractAddress?: string;
}

/**
 * Credential Status Information
 */
export interface CredentialStatusInfo {
  id: string;
  type: string;
  revocationListIndex?: number;
  revocationListCredential?: string;
}

/**
 * Credential Issuance Request
 */
export interface IssuanceRequest {
  credentialType: CredentialType;
  recipientDID?: string;
  recipientEmail: string;
  recipientName: string;
  achievement: Achievement;
  expirationDate?: string;
  deliveryMethod?: ('wallet' | 'email' | 'download')[];
  blockchainNetwork?: BlockchainNetwork;
}

/**
 * Credential Issuance Response
 */
export interface IssuanceResponse {
  credentialId: string;
  credential: VerifiableCredential;
  qrCode?: string;
  downloadUrl?: string;
  blockchainTxHash?: string;
}

/**
 * Credential Verification Request
 */
export interface VerificationRequest {
  credential: VerifiableCredential | string;
  checkRevocation?: boolean;
  checkExpiration?: boolean;
  trustRegistry?: string[];
}

/**
 * Verification Check Result
 */
export interface VerificationCheck {
  type: string;
  status: 'passed' | 'failed' | 'warning';
  message?: string;
}

/**
 * Credential Verification Response
 */
export interface VerificationResponse {
  verified: boolean;
  checks: VerificationCheck[];
  issuer?: Issuer;
  credentialSubject?: CredentialSubject;
  issuanceDate?: string;
  expirationDate?: string;
  status?: CredentialStatus;
}

/**
 * Credential Revocation Request
 */
export interface RevocationRequest {
  credentialId: string;
  reason: string;
  revocationDate?: string;
}

/**
 * Credential Revocation Response
 */
export interface RevocationResponse {
  credentialId: string;
  revoked: boolean;
  revocationDate: string;
  blockchainTxHash?: string;
}

/**
 * DID Document
 */
export interface DIDDocument {
  '@context': string[];
  id: string;
  verificationMethod: VerificationMethod[];
  authentication?: string[];
  assertionMethod?: string[];
  service?: Service[];
}

/**
 * Verification Method
 */
export interface VerificationMethod {
  id: string;
  type: string;
  controller: string;
  publicKeyMultibase?: string;
  publicKeyJwk?: any;
}

/**
 * DID Service
 */
export interface Service {
  id: string;
  type: string;
  serviceEndpoint: string;
}

/**
 * Credential Wallet Interface
 */
export interface CredentialWallet {
  addCredential(credential: VerifiableCredential): Promise<void>;
  getCredential(id: string): Promise<VerifiableCredential | null>;
  listCredentials(): Promise<VerifiableCredential[]>;
  removeCredential(id: string): Promise<void>;
  presentCredential(
    id: string,
    selectiveDisclosure?: string[]
  ): Promise<VerifiablePresentation>;
}

/**
 * Verifiable Presentation
 */
export interface VerifiablePresentation {
  '@context': string[];
  type: string[];
  verifiableCredential: VerifiableCredential[];
  proof: Proof;
  holder?: string;
}

/**
 * SDK Configuration
 */
export interface SDKConfig {
  baseURL?: string;
  apiKey?: string;
  did?: string;
  privateKey?: string;
  blockchain?: BlockchainNetwork;
  rpcUrl?: string;
  contractAddress?: string;
}

/**
 * Batch Issuance Request
 */
export interface BatchIssuanceRequest {
  credentials: IssuanceRequest[];
  blockchainBatch?: boolean;
}

/**
 * Batch Issuance Response
 */
export interface BatchIssuanceResponse {
  success: number;
  failed: number;
  results: IssuanceResponse[];
  blockchainTxHash?: string;
}

/**
 * Trust Registry Entry
 */
export interface TrustRegistryEntry {
  did: string;
  name: string;
  type: string;
  country: string;
  accredited: boolean;
  accreditation?: Accreditation[];
  registrationDate: string;
  status: 'active' | 'suspended' | 'revoked';
}

/**
 * Analytics Data
 */
export interface CredentialAnalytics {
  totalIssued: number;
  totalVerified: number;
  totalRevoked: number;
  verificationsByCountry: Record<string, number>;
  verificationsByIndustry: Record<string, number>;
  averageVerificationTime: number;
}

/**
 * Error Response
 */
export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    details?: any;
  };
}
