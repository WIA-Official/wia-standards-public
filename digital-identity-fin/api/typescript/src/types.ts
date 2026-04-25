/**
 * WIA Digital Identity Standard (WIA-FIN-010)
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 */

// ============================================================================
// DID Types
// ============================================================================

export type DIDMethod = 'ethr' | 'key' | 'web' | 'ion' | 'sov' | 'peer';
export type NetworkType = 'mainnet' | 'testnet' | 'local';
export type KeyType = 'Ed25519' | 'secp256k1' | 'RSA';

export interface DIDParams {
  method: DIDMethod;
  network?: NetworkType;
  keyType?: KeyType;
  serviceEndpoints?: ServiceEndpoint[];
}

export interface ServiceEndpoint {
  id?: string;
  type: string;
  serviceEndpoint: string | object;
}

export interface VerificationMethod {
  id: string;
  type: string;
  controller: string;
  publicKeyMultibase?: string;
  publicKeyJwk?: object;
}

export interface DIDDocument {
  '@context': string | string[];
  id: string;
  controller?: string | string[];
  verificationMethod?: VerificationMethod[];
  authentication?: (string | VerificationMethod)[];
  assertionMethod?: (string | VerificationMethod)[];
  keyAgreement?: (string | VerificationMethod)[];
  capabilityInvocation?: (string | VerificationMethod)[];
  capabilityDelegation?: (string | VerificationMethod)[];
  service?: ServiceEndpoint[];
}

export interface DIDResolutionResult {
  didDocument: DIDDocument;
  didDocumentMetadata: {
    created?: string;
    updated?: string;
    deactivated?: boolean;
  };
  didResolutionMetadata: {
    contentType?: string;
    retrieved?: string;
  };
}

// ============================================================================
// Verifiable Credential Types
// ============================================================================

export interface CredentialSubject {
  id: string;
  [key: string]: any;
}

export interface CredentialProof {
  type: string;
  created: string;
  verificationMethod: string;
  proofPurpose: string;
  proofValue: string;
  challenge?: string;
  domain?: string;
}

export interface CredentialStatus {
  id: string;
  type: string;
  statusPurpose?: string;
  statusListIndex?: string;
  statusListCredential?: string;
}

export interface VerifiableCredential {
  '@context': string | string[];
  id: string;
  type: string[];
  issuer: string | { id: string; [key: string]: any };
  issuanceDate: string;
  expirationDate?: string;
  credentialSubject: CredentialSubject | CredentialSubject[];
  credentialStatus?: CredentialStatus;
  proof: CredentialProof;
}

export interface IssueCredentialParams {
  issuer: string;
  credentialSubject: CredentialSubject;
  type: string[];
  expirationDate?: string;
  credentialStatus?: CredentialStatus;
}

export interface VerificationResult {
  verified: boolean;
  checks: {
    signature: 'valid' | 'invalid';
    expiration?: 'valid' | 'expired';
    revocation?: 'not_revoked' | 'revoked';
    issuerTrust?: 'trusted' | 'untrusted';
  };
  warnings?: string[];
  errors?: string[];
}

// ============================================================================
// Verifiable Presentation Types
// ============================================================================

export interface VerifiablePresentation {
  '@context': string | string[];
  type: string[];
  holder?: string;
  verifiableCredential: VerifiableCredential[];
  proof: CredentialProof;
}

export interface PresentationParams {
  holder: string;
  verifiableCredentials: VerifiableCredential[];
  challenge: string;
  domain?: string;
}

export interface PresentationRequest {
  id: string;
  type: string;
  from: string;
  created: string;
  challenge: string;
  domain?: string;
  query: PresentationQuery[];
}

export interface PresentationQuery {
  type: string;
  credentialQuery: {
    reason?: string;
    example: Partial<VerifiableCredential>;
  };
}

// ============================================================================
// Biometric Types
// ============================================================================

export type BiometricModality = 'facial' | 'fingerprint' | 'iris' | 'voice';
export type LivenessMethod = 'active' | 'passive';

export interface BiometricTemplate {
  type: 'BiometricTemplate';
  modality: BiometricModality;
  version: string;
  created: string;
  template: {
    algorithm: string;
    vector: number[];
    quality: number;
    metadata: {
      captureDevice: string;
      resolution: string;
      lighting?: string;
    };
  };
  protection: {
    method: 'cancelable' | 'irreversible';
    key: string;
    irreversible: boolean;
  };
}

export interface LivenessProof {
  type: 'LivenessProof';
  method: LivenessMethod;
  timestamp: string;
  confidence: number;
  indicators: {
    textureAnalysis?: boolean;
    depthSensing?: boolean;
    microExpressions?: boolean;
    bloodFlow?: boolean;
  };
  videoHash?: string;
  deviceAttestation?: {
    manufacturer: string;
    model: string;
    attestation: string;
  };
}

export interface BiometricEnrollParams {
  did: string;
  modality: BiometricModality;
  biometricData: string; // base64 encoded
  livenessProof: LivenessProof;
}

export interface BiometricVerifyParams {
  did: string;
  modality: BiometricModality;
  biometricData: string; // base64 encoded
  livenessProof: LivenessProof;
}

export interface BiometricVerifyResult {
  verified: boolean;
  confidence: number;
  matchScore: number;
}

// ============================================================================
// Zero-Knowledge Proof Types
// ============================================================================

export type ZKProofType = 'AgeVerification' | 'RangeProof' | 'MembershipProof';
export type ZKAlgorithm = 'groth16' | 'plonk' | 'stark' | 'bbs+';

export interface ZeroKnowledgeProof {
  type: 'ZeroKnowledgeProof';
  proofType: ZKProofType;
  algorithm: ZKAlgorithm;
  curve?: string;
  claim: string;
  proof: object;
  publicInputs: object;
  verificationKey?: object;
}

export interface AgeProofParams {
  birthdate: string;
  minimumAge: number;
  credentialId?: string;
}

export interface SelectiveDisclosureProof {
  type: 'SelectiveDisclosureProof';
  algorithm: 'BBS+';
  disclosedAttributes: string[];
  proof: {
    proofValue: string;
    nonce: string;
  };
  revealedMessages: Record<string, any>;
}

// ============================================================================
// eKYC Types
// ============================================================================

export type KYCLevel = 1 | 2 | 3 | 4;
export type KYCStatus = 'pending' | 'processing' | 'approved' | 'rejected';
export type DocumentType = 'passport' | 'national_id' | 'drivers_license';

export interface KYCParams {
  did: string;
  level: KYCLevel;
  documentType: DocumentType;
  countryCode: string;
}

export interface KYCSession {
  sessionId: string;
  uploadUrl: string;
  expiresAt: string;
}

export interface KYCStatusResult {
  status: KYCStatus;
  level?: KYCLevel;
  credential?: VerifiableCredential;
  verificationDetails?: {
    documentVerified: boolean;
    biometricMatched: boolean;
    livenessDetected: boolean;
    sanctionsCleared: boolean;
  };
}

// ============================================================================
// Wallet Types
// ============================================================================

export interface WalletParams {
  ownerDid: string;
  backupMethod?: 'encrypted_cloud' | 'local' | 'hardware';
}

export interface WalletInfo {
  walletId: string;
  recoveryPhrase: string;
  backupUrl?: string;
}

export interface StoredCredential {
  id: string;
  type: string[];
  issuer: string;
  issuanceDate: string;
  expirationDate?: string;
  tags: string[];
  credential: VerifiableCredential;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface WIAIdentityConfig {
  apiKey?: string;
  baseUrl?: string;
  networks?: Partial<Record<DIDMethod, NetworkType>>;
  providers?: Record<string, string>;
  resolver?: string;
  debug?: boolean;
}

// ============================================================================
// Error Types
// ============================================================================

export class WIAIdentityError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'WIAIdentityError';
  }
}

export interface APIError {
  error: string;
  error_description: string;
  error_code: string;
  timestamp: string;
}
