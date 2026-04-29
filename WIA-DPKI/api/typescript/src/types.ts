/**
 * WIA DPKI Standard - TypeScript Type Definitions
 * Decentralized Public Key Infrastructure
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

export interface WIADPKI {
  standard: 'WIA-DPKI';
  version: string;
  infrastructure: DPKIInfrastructure;
  identities: DecentralizedIdentity[];
  keys: KeyPair[];
  certificates: DistributedCertificate[];
  trustAnchors: TrustAnchor[];
  revocations: RevocationEntry[];
  governance: DPKIGovernance;
  extensions?: Record<string, unknown>;
}

export interface DPKIInfrastructure {
  id: string;
  name: string;
  type: InfrastructureType;
  status: InfrastructureStatus;
  network: NetworkConfig;
  consensus: ConsensusConfig;
  createdAt: string;
}

export type InfrastructureType = 'blockchain' | 'distributed-ledger' | 'federated' | 'hybrid' | 'mesh';
export type InfrastructureStatus = 'initializing' | 'operational' | 'syncing' | 'degraded' | 'offline';

export interface NetworkConfig {
  protocol: NetworkProtocol;
  nodes: NodeInfo[];
  replicationFactor: number;
  partitionTolerance: boolean;
}

export type NetworkProtocol = 'libp2p' | 'gossip' | 'kademlia' | 'chord' | 'custom';

export interface NodeInfo {
  id: string;
  address: string;
  role: NodeRole;
  status: 'active' | 'syncing' | 'offline';
  lastSeen: string;
}

export type NodeRole = 'validator' | 'observer' | 'relay' | 'archive' | 'light';

export interface ConsensusConfig {
  algorithm: ConsensusAlgorithm;
  threshold: number;
  timeout: number;
  finality: FinalityType;
}

export type ConsensusAlgorithm = 'pbft' | 'raft' | 'tendermint' | 'hotstuff' | 'avalanche';
export type FinalityType = 'instant' | 'probabilistic' | 'eventual';

// ============================================================================
// Identity Types
// ============================================================================

export interface DecentralizedIdentity {
  did: string;
  controller: string;
  verificationMethods: VerificationMethod[];
  authentication: string[];
  assertionMethod: string[];
  keyAgreement: string[];
  services: ServiceEndpoint[];
  created: string;
  updated: string;
}

export interface VerificationMethod {
  id: string;
  type: VerificationMethodType;
  controller: string;
  publicKeyMultibase?: string;
  publicKeyJwk?: JsonWebKey;
}

export type VerificationMethodType =
  | 'Ed25519VerificationKey2020'
  | 'EcdsaSecp256k1VerificationKey2019'
  | 'JsonWebKey2020'
  | 'X25519KeyAgreementKey2020';

export interface JsonWebKey {
  kty: string;
  crv?: string;
  x?: string;
  y?: string;
  n?: string;
  e?: string;
}

export interface ServiceEndpoint {
  id: string;
  type: string;
  serviceEndpoint: string;
  description?: string;
}

// ============================================================================
// Key Types
// ============================================================================

export interface KeyPair {
  id: string;
  owner: string;
  algorithm: KeyAlgorithm;
  purpose: KeyPurpose[];
  publicKey: string;
  fingerprint: string;
  created: string;
  expires?: string;
  status: KeyStatus;
  usage: KeyUsageStats;
}

export type KeyAlgorithm = 'ed25519' | 'secp256k1' | 'rsa-2048' | 'rsa-4096' | 'p-256' | 'p-384' | 'dilithium' | 'sphincs+';
export type KeyPurpose = 'signing' | 'encryption' | 'authentication' | 'key-agreement' | 'capability-delegation';
export type KeyStatus = 'active' | 'suspended' | 'revoked' | 'expired' | 'compromised';

export interface KeyUsageStats {
  signatureCount: number;
  lastUsed?: string;
  rotationSchedule?: string;
}

// ============================================================================
// Certificate Types
// ============================================================================

export interface DistributedCertificate {
  id: string;
  version: number;
  subject: CertificateSubject;
  issuer: CertificateIssuer;
  validity: ValidityPeriod;
  publicKey: string;
  signature: CertificateSignature;
  extensions: CertificateExtension[];
  anchors: string[];
  status: CertificateStatus;
}

export interface CertificateSubject {
  did: string;
  name?: string;
  organization?: string;
  attributes: Record<string, string>;
}

export interface CertificateIssuer {
  did: string;
  name: string;
  trustLevel: number;
}

export interface ValidityPeriod {
  notBefore: string;
  notAfter: string;
}

export interface CertificateSignature {
  algorithm: string;
  value: string;
  signers: string[];
}

export interface CertificateExtension {
  oid: string;
  critical: boolean;
  value: unknown;
}

export type CertificateStatus = 'valid' | 'expired' | 'revoked' | 'suspended' | 'pending';

// ============================================================================
// Trust Types
// ============================================================================

export interface TrustAnchor {
  id: string;
  name: string;
  type: TrustAnchorType;
  publicKey: string;
  trustLevel: number;
  jurisdiction?: string;
  endorsements: Endorsement[];
  created: string;
}

export type TrustAnchorType = 'root' | 'intermediate' | 'cross-certified' | 'web-of-trust' | 'delegated';

export interface Endorsement {
  endorser: string;
  level: number;
  signature: string;
  timestamp: string;
}

// ============================================================================
// Revocation Types
// ============================================================================

export interface RevocationEntry {
  id: string;
  type: RevocationType;
  targetId: string;
  reason: RevocationReason;
  revokedBy: string;
  timestamp: string;
  proof: RevocationProof;
}

export type RevocationType = 'key' | 'certificate' | 'identity' | 'delegation';
export type RevocationReason = 'key-compromise' | 'superseded' | 'cessation' | 'privilege-withdrawn' | 'unspecified';

export interface RevocationProof {
  method: 'accumulator' | 'bloom-filter' | 'merkle-tree' | 'status-list';
  value: string;
  lastUpdated: string;
}

// ============================================================================
// Governance Types
// ============================================================================

export interface DPKIGovernance {
  model: GovernanceModel;
  policies: GovernancePolicy[];
  participants: GovernanceParticipant[];
  proposals: Proposal[];
  voting: VotingConfig;
}

export type GovernanceModel = 'decentralized' | 'consortium' | 'federated' | 'hybrid';

export interface GovernancePolicy {
  id: string;
  type: PolicyType;
  description: string;
  parameters: Record<string, unknown>;
  active: boolean;
}

export type PolicyType = 'key-rotation' | 'revocation' | 'trust-anchor' | 'membership' | 'upgrade';

export interface GovernanceParticipant {
  id: string;
  did: string;
  role: ParticipantRole;
  votingPower: number;
  joined: string;
}

export type ParticipantRole = 'admin' | 'validator' | 'member' | 'observer';

export interface Proposal {
  id: string;
  type: string;
  proposer: string;
  description: string;
  status: ProposalStatus;
  votes: Vote[];
  created: string;
  deadline: string;
}

export type ProposalStatus = 'pending' | 'active' | 'passed' | 'rejected' | 'executed';

export interface Vote {
  voter: string;
  choice: 'yes' | 'no' | 'abstain';
  weight: number;
  timestamp: string;
}

export interface VotingConfig {
  quorum: number;
  threshold: number;
  duration: number;
  delegationAllowed: boolean;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface InfrastructureResponse {
  id: string;
  name: string;
  type: InfrastructureType;
  status: InfrastructureStatus;
  nodeCount: number;
  identityCount: number;
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
