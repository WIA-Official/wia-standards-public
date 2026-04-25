/**
 * WIA-SOC-002: Digital ID Standard - TypeScript Type Definitions
 * 弘益人間 (Hongik Ingan) - Benefit All Humanity
 */

// DID Types
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
  alsoKnownAs?: string[];
  created?: string;
  updated?: string;
  proof?: Proof | Proof[];
}

export interface VerificationMethod {
  id: string;
  type: string;
  controller: string;
  publicKeyMultibase?: string;
  publicKeyJwk?: JsonWebKey;
  publicKeyHex?: string;
  publicKeyBase58?: string;
  blockchainAccountId?: string;
}

export interface ServiceEndpoint {
  id: string;
  type: string;
  serviceEndpoint: string | string[] | Record<string, unknown>;
  description?: string;
}

// Verifiable Credential Types
export interface VerifiableCredential {
  '@context': string | string[];
  id: string;
  type: string[];
  issuer: string | Issuer;
  issuanceDate: string;
  expirationDate?: string;
  credentialSubject: CredentialSubject | CredentialSubject[];
  credentialStatus?: CredentialStatus;
  credentialSchema?: CredentialSchema;
  evidence?: Evidence[];
  termsOfUse?: TermsOfUse[];
  proof: Proof | Proof[];
}

export interface Issuer {
  id: string;
  name?: string;
  url?: string;
  image?: string;
}

export interface CredentialSubject {
  id: string;
  [key: string]: any;
}

export interface CredentialStatus {
  id: string;
  type: string;
  statusPurpose?: string;
  statusListIndex?: string;
  statusListCredential?: string;
}

export interface CredentialSchema {
  id: string;
  type: string;
}

export interface Evidence {
  id?: string;
  type: string | string[];
  verifier?: string;
  evidenceDocument?: string;
  subjectPresence?: string;
  documentPresence?: string;
}

export interface TermsOfUse {
  type: string;
  id?: string;
  profile?: string;
  prohibition?: Prohibition[];
}

export interface Prohibition {
  assigner: string;
  assignee: string;
  target: string;
  action: string[];
}

// Verifiable Presentation Types
export interface VerifiablePresentation {
  '@context': string | string[];
  type: string | string[];
  id?: string;
  holder: string;
  verifiableCredential: VerifiableCredential | VerifiableCredential[];
  proof: Proof | Proof[];
}

// Proof Types
export interface Proof {
  type: string;
  created: string;
  verificationMethod: string;
  proofPurpose: string;
  proofValue?: string;
  jws?: string;
  challenge?: string;
  domain?: string;
  nonce?: string;
}

export interface MultiSignatureProof extends Proof {
  type: 'MultiSignature2021';
  threshold: number;
  signatures: Proof[];
}

// Zero-Knowledge Proof Types
export interface ZKProof {
  type: 'ZKProof';
  proofSystem: 'groth16' | 'plonk' | 'bulletproofs';
  curve: 'bn254' | 'bls12-381';
  statement: ProofStatement;
  proof: Groth16Proof | PlonkProof | BulletproofData;
  publicSignals?: string[];
  verificationKey?: VerificationKey;
}

export interface ProofStatement {
  type: 'RangeProof' | 'MembershipProof' | 'EqualityProof';
  claim: string;
  publicInputs: string[];
}

export interface Groth16Proof {
  pi_a: string[];
  pi_b: string[][];
  pi_c: string[];
}

export interface PlonkProof {
  commitments: string[];
  evaluations: string[];
  openingProof: string;
}

export interface BulletproofData {
  A: string;
  S: string;
  T1: string;
  T2: string;
  taux: string;
  mu: string;
  t: string;
  innerProductProof: InnerProductProof;
}

export interface InnerProductProof {
  L: string[];
  R: string[];
  a: string;
  b: string;
}

export interface VerificationKey {
  protocol: string;
  curve: string;
  nPublic: number;
  vk_alpha_1: string[];
  vk_beta_2: string[][];
  vk_gamma_2: string[][];
  vk_delta_2: string[][];
  IC: string[][];
}

// DID Method Types
export interface DIDMethodOptions {
  method: 'wia' | 'ethr' | 'polygon' | 'solana' | 'web';
  network?: string;
  keyType?: KeyType;
  controller?: string;
  service?: ServiceEndpoint[];
}

export type KeyType =
  | 'Ed25519VerificationKey2020'
  | 'EcdsaSecp256k1VerificationKey2019'
  | 'JsonWebKey2020'
  | 'X25519KeyAgreementKey2020';

// Credential Issuance Types
export interface CredentialRequest {
  type: string[];
  issuer: string | Issuer;
  credentialSubject: CredentialSubject;
  expirationDate?: string;
  options?: IssuanceOptions;
}

export interface IssuanceOptions {
  proofType?: string;
  revocable?: boolean;
  statusType?: string;
  evidence?: Evidence[];
  termsOfUse?: TermsOfUse[];
}

// Verification Types
export interface VerificationResult {
  verified: boolean;
  checks?: VerificationChecks;
  warnings?: string[];
  errors?: string[];
  timestamp?: string;
}

export interface VerificationChecks {
  signature?: SignatureCheck;
  expiration?: ExpirationCheck;
  revocation?: RevocationCheck;
  proof?: ProofCheck;
}

export interface SignatureCheck {
  valid: boolean;
  issuer?: string;
  verificationMethod?: string;
  error?: string;
}

export interface ExpirationCheck {
  valid: boolean;
  expirationDate?: string;
  error?: string;
}

export interface RevocationCheck {
  valid: boolean;
  revoked: boolean;
  statusListChecked?: string;
  error?: string;
}

export interface ProofCheck {
  valid: boolean;
  proofType?: string;
  error?: string;
}

// API Types
export interface WIAClientConfig {
  apiUrl: string;
  apiKey?: string;
  network?: string;
  timeout?: number;
}

export interface DIDCreationResult {
  did: string;
  didDocument: DIDDocument;
  keys: KeyPair;
  transactionHash?: string;
  blockNumber?: number;
}

export interface KeyPair {
  privateKey: string;
  publicKey: string;
  keyId: string;
}

// Selective Disclosure Types
export interface SelectiveDisclosureOptions {
  revealed: string[];
  hidden: string[];
  hashAlgorithm?: 'SHA-256' | 'SHA-3' | 'Blake2b';
  salt?: string;
}

export interface SelectiveDisclosureData {
  revealed: string[];
  hidden: string[];
  hiddenHashes: Record<string, HiddenFieldHash>;
  hashAlgorithm: string;
}

export interface HiddenFieldHash {
  hash: string;
  salt: string;
}

// Presentation Request Types
export interface PresentationRequest {
  holder: string;
  verifiableCredential: VerifiableCredential[];
  challenge?: string;
  domain?: string;
  selectiveDisclosure?: SelectiveDisclosureOptions;
}

// Authentication Types
export interface AuthenticationProof {
  type: 'DIDAuthentication';
  did: string;
  challenge: string;
  timestamp: number;
  proof: Proof;
}

// Blockchain Integration Types
export interface BlockchainConfig {
  network: string;
  rpcUrl: string;
  registryAddress?: string;
  privateKey?: string;
}

export interface TransactionReceipt {
  transactionHash: string;
  blockNumber: number;
  blockHash: string;
  gasUsed: string;
  status: boolean;
}

// Error Types
export interface WIAError {
  code: string;
  message: string;
  details?: Record<string, any>;
  timestamp: string;
  requestId?: string;
}

// Revocation List Types
export interface StatusList2021 {
  '@context': string[];
  id: string;
  type: string[];
  issuer: string;
  issuanceDate: string;
  credentialSubject: StatusListSubject;
  proof: Proof;
}

export interface StatusListSubject {
  id: string;
  type: 'StatusList2021';
  statusPurpose: 'revocation' | 'suspension';
  encodedList: string;
}

// Wallet Integration Types
export interface WalletProvider {
  name: string;
  version: string;
  request(args: RequestArguments): Promise<any>;
  on(event: string, handler: (...args: any[]) => void): void;
  removeListener(event: string, handler: (...args: any[]) => void): void;
}

export interface RequestArguments {
  method: string;
  params?: any[];
}

// Event Types
export interface DIDEvent {
  type: 'DIDCreated' | 'DIDUpdated' | 'DIDDeactivated';
  did: string;
  timestamp: string;
  transactionHash?: string;
}

export interface CredentialEvent {
  type: 'CredentialIssued' | 'CredentialRevoked' | 'CredentialExpired';
  credentialId: string;
  issuer: string;
  subject: string;
  timestamp: string;
}

// Cache Types
export interface CacheConfig {
  ttl: number; // Time to live in milliseconds
  maxSize: number; // Maximum cache size
}

export interface CachedItem<T> {
  data: T;
  timestamp: number;
  ttl: number;
}
