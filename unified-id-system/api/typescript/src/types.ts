/**
 * WIA-UNI-002 Unified ID System - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 */

/**
 * Unified ID format: UNI-KR-YYMMDD-NNNN
 */
export type UnifiedId = string;

/**
 * Decentralized Identifier (DID) format
 */
export type DID = string;

/**
 * ISO 8601 date string (YYYY-MM-DD)
 */
export type ISODate = string;

/**
 * ISO 8601 datetime string
 */
export type ISODateTime = string;

/**
 * Origin region enumeration
 */
export enum OriginRegion {
  SOUTH = "south",
  NORTH = "north",
  UNIFIED = "unified"
}

/**
 * Credential status enumeration
 */
export enum CredentialStatus {
  ACTIVE = "active",
  REVOKED = "revoked",
  SUSPENDED = "suspended",
  EXPIRED = "expired"
}

/**
 * Gender enumeration
 */
export enum Gender {
  MALE = "M",
  FEMALE = "F",
  OTHER = "X"
}

/**
 * Personal information
 */
export interface PersonalInfo {
  familyName: string;
  givenName: string;
  fullName: string;
  birthDate: ISODate;
  gender?: Gender;
}

/**
 * Origin information (encrypted in practice)
 */
export interface OriginInfo {
  region: OriginRegion;
  previousId?: string;
  registrationDate: ISODateTime;
}

/**
 * Biometric hashes
 */
export interface BiometricInfo {
  fingerprintHash?: string;
  facialHash?: string;
  irisHash?: string;
}

/**
 * Complete unified ID record
 */
export interface UnifiedIdRecord {
  version: string;
  standard: "WIA-UNI-002";
  unifiedId: UnifiedId;
  personalInfo: PersonalInfo;
  origin: OriginInfo;
  biometric: BiometricInfo;
  issuedAt: ISODateTime;
  expiresAt?: ISODateTime;
  issuer: DID;
  status: CredentialStatus;
  privacyLevel: "public" | "protected" | "encrypted";
}

/**
 * Verifiable Credential
 */
export interface VerifiableCredential {
  "@context": string[];
  type: string[];
  issuer: DID;
  issuanceDate: ISODateTime;
  expirationDate?: ISODateTime;
  credentialSubject: {
    id: DID;
    unifiedId: UnifiedId;
    [key: string]: any;
  };
  proof: {
    type: string;
    created: ISODateTime;
    proofPurpose: string;
    verificationMethod: string;
    proofValue: string;
  };
}

/**
 * Zero-knowledge proof types
 */
export enum ZKProofType {
  AGE_OVER_18 = "age-over-18",
  AGE_OVER_21 = "age-over-21",
  AGE_OVER_65 = "age-over-65",
  CITIZENSHIP = "citizenship",
  AUTHORIZATION = "authorization",
  FAMILY_RELATIONSHIP = "family-relationship"
}

/**
 * Zero-knowledge proof
 */
export interface ZKProof {
  type: "zk-SNARK" | "zk-STARK" | "bulletproof";
  proof: string;
  publicInputs: Record<string, any>;
  verificationKey?: string;
}

/**
 * Verification result
 */
export interface VerificationResult {
  verified: boolean;
  checks: {
    signature: "valid" | "invalid";
    expiration: "valid" | "expired";
    revocation: "not_revoked" | "revoked";
    issuer?: "trusted" | "untrusted";
  };
  timestamp: ISODateTime;
  message?: string;
}

/**
 * API client configuration
 */
export interface ClientConfig {
  apiKey: string;
  baseUrl?: string;
  environment?: "production" | "staging" | "development";
  timeout?: number;
}

/**
 * Error response
 */
export interface APIError {
  error: {
    code: string;
    message: string;
    timestamp: ISODateTime;
    requestId: string;
  };
}

/**
 * Credential issuance request
 */
export interface IssueCredentialRequest {
  personalInfo: PersonalInfo;
  origin: OriginInfo;
  biometric: BiometricInfo;
}

/**
 * Credential issuance response
 */
export interface IssueCredentialResponse {
  unifiedId: UnifiedId;
  credential: VerifiableCredential;
}

/**
 * Verification request
 */
export interface VerifyCredentialRequest {
  credential: VerifiableCredential;
}

/**
 * ZK proof generation request
 */
export interface GenerateZKProofRequest {
  unifiedId: UnifiedId;
  proofType: ZKProofType | string;
  parameters?: Record<string, any>;
}

/**
 * ZK proof generation response
 */
export interface GenerateZKProofResponse {
  proof: ZKProof;
  message?: string;
}

/**
 * Revocation request
 */
export interface RevokeCredentialRequest {
  unifiedId: UnifiedId;
  reason: "lost" | "stolen" | "compromised" | "voluntary" | "other";
  effectiveDate?: ISODateTime;
}

/**
 * Revocation response
 */
export interface RevokeCredentialResponse {
  revoked: boolean;
  revocationId: string;
  timestamp: ISODateTime;
}
