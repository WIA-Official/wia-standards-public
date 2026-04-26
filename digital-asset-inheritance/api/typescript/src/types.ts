/**
 * WIA-LEG-006: Digital Asset Inheritance Standard
 * TypeScript Type Definitions
 *
 * @version 2.0.0
 * @license MIT
 * @author WIA - World Certification Industry Association
 */

// ========== Core Types ==========

export interface InheritancePlan {
  wiaStandard: 'LEG-006';
  version: string;
  inheritancePlan: InheritancePlanData;
  signatures: Signature[];
}

export interface InheritancePlanData {
  planId: string;
  metadata: Metadata;
  owner: Owner;
  assets: Asset[];
  beneficiaries: Beneficiary[];
  distribution: Distribution;
  triggers: Trigger[];
  legal: LegalCompliance;
  execution?: ExecutionData;
}

// ========== Metadata ==========

export interface Metadata {
  created: string; // ISO 8601
  lastModified: string; // ISO 8601
  version: string; // Semantic version
  previousVersionHash?: string; // SHA-256
  title: string;
  description: string;
  jurisdiction: string; // ISO 3166-2
  language: string; // IETF BCP 47
  currency: string; // ISO 4217
  tags: string[];
  confidentiality: 'public' | 'private' | 'confidential';
  encryption?: EncryptionConfig;
}

export interface EncryptionConfig {
  algorithm: 'AES-256-GCM' | 'RSA-4096';
  keyDerivation: 'PBKDF2-HMAC-SHA256' | 'Argon2id';
  encryptedFields: string[];
}

// ========== Owner ==========

export interface Owner {
  identity: OwnerIdentity;
  contact: ContactInfo;
  authentication: AuthenticationConfig;
  legal: OwnerLegalInfo;
}

export interface OwnerIdentity {
  did: string; // W3C DID
  legalName: string;
  dateOfBirth: string; // ISO 8601
  nationality: string; // ISO 3166-1
  taxId?: string; // Encrypted
  biometricHash?: string; // SHA-256 of biometric template
}

export interface ContactInfo {
  email: string;
  phone: string; // E.164 format
  address: Address;
}

export interface Address {
  street: string;
  city: string;
  state: string;
  postalCode: string;
  country: string; // ISO 3166-1
}

export interface AuthenticationConfig {
  methods: AuthMethod[];
  mfa: boolean;
  recoveryContacts: string[];
}

export type AuthMethod = 'biometric' | 'hardware-key' | 'password' | 'sms' | 'email';

export interface OwnerLegalInfo {
  capacity: 'full' | 'limited' | 'guardian-appointed';
  powerOfAttorney?: string; // DID
  guardian?: string; // DID
}

// ========== Assets ==========

export type Asset = CryptocurrencyAsset | NFTAsset | DigitalRealEstateAsset | DomainNameAsset | IntellectualPropertyAsset;

export interface BaseAsset {
  assetId: string;
  type: AssetType;
  classification: string;
  valuation: Valuation;
  metadata: AssetMetadata;
}

export type AssetType = 'cryptocurrency' | 'nft' | 'digital-real-estate' | 'domain-name' | 'intellectual-property';

export interface Valuation {
  acquisitionCost: MonetaryValue;
  currentValue: MonetaryValue;
  taxBasis: 'FIFO' | 'LIFO' | 'HIFO' | 'specific-identification';
}

export interface MonetaryValue {
  amount: number;
  currency: string; // ISO 4217
  date: string; // ISO 8601
  source?: string;
}

export interface AssetMetadata {
  description: string;
  priority: 'low' | 'medium' | 'high';
  notes?: string;
  attachments?: string[];
}

// ===== Cryptocurrency Assets =====

export interface CryptocurrencyAsset extends BaseAsset {
  type: 'cryptocurrency';
  cryptocurrency: CryptocurrencyDetails;
  access: AccessConfig;
}

export interface CryptocurrencyDetails {
  symbol: string; // BTC, ETH, etc.
  fullName: string;
  network: 'mainnet' | 'testnet';
  amount: string; // Decimal string for precision
  walletType: 'hardware' | 'software' | 'paper' | 'custodial';
  walletProvider: string;
  addresses: WalletAddress[];
}

export interface WalletAddress {
  address: string;
  type: string; // P2PKH, P2WPKH, etc.
  balance: string; // Decimal string
  derivationPath?: string; // BIP32/BIP44
}

export interface AccessConfig {
  method: 'single-sig' | 'multi-sig' | 'shamir-secret';
  requiredSignatures?: number;
  totalKeys?: number;
  keyShares?: KeyShare[];
  seedPhraseBackup?: SeedPhraseBackup;
}

export interface KeyShare {
  shareId: number;
  holder: string; // DID or description
  location: string;
}

export interface SeedPhraseBackup {
  method: 'shamir-secret-sharing' | 'encrypted-backup';
  threshold?: number;
  totalShares?: number;
  shareLocations?: string[];
}

// ===== NFT Assets =====

export interface NFTAsset extends BaseAsset {
  type: 'nft';
  nft: NFTDetails;
  rights: NFTRights;
}

export interface NFTDetails {
  blockchain: string; // Ethereum, Polygon, Solana, etc.
  standard: string; // ERC-721, ERC-1155, SPL, etc.
  contractAddress: string;
  tokenId: string;
  ownerAddress: string;
  metadata: NFTMetadata;
  provenance: ProvenanceRecord[];
}

export interface NFTMetadata {
  name: string;
  description: string;
  image: string; // IPFS or HTTP URL
  attributes: NFTAttribute[];
  externalUrl?: string;
}

export interface NFTAttribute {
  trait_type: string;
  value: string | number;
}

export interface ProvenanceRecord {
  from: string;
  to: string;
  date: string; // ISO 8601
  price?: string;
  transactionHash?: string;
}

export interface NFTRights {
  copyright: 'retained' | 'transferred' | 'shared';
  commercialUse: 'permitted' | 'restricted' | 'prohibited';
  transferable: boolean;
  royalties?: RoyaltyConfig;
}

export interface RoyaltyConfig {
  percentage: number;
  recipient: string; // Address
}

// ===== Digital Real Estate =====

export interface DigitalRealEstateAsset extends BaseAsset {
  type: 'digital-real-estate';
  metaverseProperty: MetaversePropertyDetails;
  access: PlatformAccess;
}

export interface MetaversePropertyDetails {
  platform: string; // Decentraland, The Sandbox, etc.
  parcels: LandParcel[];
  totalArea: string;
  blockchain: string;
  contractAddress: string;
  tokenIds: string[];
  developments?: Development[];
}

export interface LandParcel {
  coordinates: { x: number; y: number };
  size: string;
  landId: string;
}

export interface Development {
  type: string;
  name: string;
  description: string;
  builderCredit?: string;
  files?: string[];
}

export interface PlatformAccess {
  platformAccount: {
    username: string;
    email: string;
    wallet: string;
  };
  buildPermissions?: string;
}

// ===== Domain Names =====

export interface DomainNameAsset extends BaseAsset {
  type: 'domain-name';
  domain: DomainDetails;
}

export interface DomainDetails {
  name: string;
  tld: string; // .com, .eth, etc.
  registrar: string;
  expirationDate: string; // ISO 8601
  autoRenew: boolean;
  blockchain?: string; // For ENS, etc.
}

// ===== Intellectual Property =====

export interface IntellectualPropertyAsset extends BaseAsset {
  type: 'intellectual-property';
  intellectualProperty: IPDetails;
}

export interface IPDetails {
  ipType: 'copyright' | 'patent' | 'trademark' | 'trade-secret';
  title: string;
  registrationNumber?: string;
  registrationDate?: string;
  expirationDate?: string;
  royaltyStream?: RoyaltyStream;
}

export interface RoyaltyStream {
  frequency: 'monthly' | 'quarterly' | 'annual';
  averageAmount: number;
  currency: string;
}

// ========== Beneficiaries ==========

export type Beneficiary = IndividualBeneficiary | OrganizationalBeneficiary;

export interface BaseBeneficiary {
  beneficiaryId: string;
  allocation: Allocation;
  contingent: ContingentConfig;
}

export interface IndividualBeneficiary extends BaseBeneficiary {
  identity: BeneficiaryIdentity;
  conditions?: DistributionConditions;
}

export interface OrganizationalBeneficiary extends BaseBeneficiary {
  identity: OrganizationIdentity;
  purpose: string;
}

export interface BeneficiaryIdentity {
  did: string;
  legalName: string;
  relationship: 'spouse' | 'child' | 'sibling' | 'parent' | 'friend' | 'other';
  dateOfBirth: string; // ISO 8601
  contact: ContactInfo;
}

export interface OrganizationIdentity {
  type: 'charitable-organization' | 'trust' | 'foundation' | 'corporation';
  legalName: string;
  taxId: string;
  charityRegistration?: string;
  wallet: string;
}

export interface Allocation {
  percentage: number; // 0-100
  specificAssets: string[]; // Array of assetIds
  residualShare: number; // 0-100
}

export interface DistributionConditions {
  ageRequirement?: number;
  educationMilestone?: string;
  trustStructure?: TrustStructure;
}

export interface TrustStructure {
  type: 'spendthrift' | 'discretionary' | 'special-needs';
  trustee: string; // DID
  distributionSchedule: DistributionScheduleItem[];
}

export interface DistributionScheduleItem {
  age: number;
  percentage: number;
}

export interface ContingentConfig {
  primary: boolean;
  alternates: string[]; // beneficiaryIds
  failureConditions: FailureCondition[];
}

export type FailureCondition = 'predeceased' | 'disclaimed' | 'incapacitated';

// ========== Distribution ==========

export interface Distribution {
  method: 'equal' | 'percentage' | 'specific-bequest' | 'mixed';
  rules: DistributionRule[];
  simultaneousDeath: SimultaneousDeathProvision;
  taxOptimization?: TaxOptimization;
}

export interface DistributionRule {
  ruleId: string;
  type: 'specific-bequest' | 'percentage-split' | 'residual';
  asset?: string; // assetId or pattern
  beneficiary?: string; // beneficiaryId
  beneficiaries?: BeneficiaryAllocation[];
  condition?: string;
}

export interface BeneficiaryAllocation {
  id: string; // beneficiaryId
  percentage: number;
}

export interface SimultaneousDeathProvision {
  presumption: 'beneficiary-predeceased' | 'uniform-simultaneous-death-act';
  survivorshipPeriod: string; // e.g., "30 days"
}

export interface TaxOptimization {
  strategy: 'marital-deduction' | 'charitable-remainder' | 'generation-skipping';
  estateTaxExemption: number;
  jurisdiction: string;
}

// ========== Triggers ==========

export type Trigger = DeadManSwitchTrigger | OracleVerifiedTrigger | ManualExecutorTrigger;

export interface BaseTrigger {
  triggerId: string;
  type: TriggerType;
}

export type TriggerType = 'dead-mans-switch' | 'oracle-verified' | 'manual-executor';

export interface DeadManSwitchTrigger extends BaseTrigger {
  type: 'dead-mans-switch';
  inactivityPeriod: number;
  unit: 'days' | 'hours' | 'months';
  checkInMethod: 'biometric-app' | 'email' | 'sms' | 'multi-factor';
  notificationSchedule: NotificationScheduleItem[];
  gracePeriod: number;
  resetOnActivity: boolean;
}

export interface NotificationScheduleItem {
  beforeTrigger: number; // Days before trigger
  recipients: ('owner' | 'executor' | 'beneficiary')[];
  method: 'email' | 'sms' | 'push' | 'certified-mail';
}

export interface OracleVerifiedTrigger extends BaseTrigger {
  type: 'oracle-verified';
  oracle: OracleConfig;
}

export interface OracleConfig {
  type: 'death-certificate' | 'court-order' | 'medical-incapacity';
  provider: string;
  apiEndpoint: string;
  requiredConfirmations: number;
  verificationMethod: 'multi-jurisdiction-consensus' | 'single-authority';
}

export interface ManualExecutorTrigger extends BaseTrigger {
  type: 'manual-executor';
  authorizedParties: string[]; // DIDs
  requiredSignatures: number;
  signatories: ('executor' | 'attorney' | 'beneficiary')[];
  documentationRequired: DocumentationType[];
}

export type DocumentationType = 'death-certificate' | 'letters-testamentary' | 'court-order';

export interface TriggerLogic {
  operator: 'OR' | 'AND';
  minimumConfidence: number; // 0.0-1.0
  disputeResolution: DisputeResolution;
}

export interface DisputeResolution {
  arbitrator: string;
  appealPeriod: number; // Days
}

// ========== Legal Compliance ==========

export interface LegalCompliance {
  governingLaw: string; // ISO 3166-2
  testamentaryCapacity?: TestamentaryCapacity;
  witnesses: Witness[];
  noContest: boolean;
  arbitration?: ArbitrationClause;
}

export interface TestamentaryCapacity {
  certifiedBy: 'attorney' | 'physician';
  date: string; // ISO 8601
  documentHash: string; // SHA-256
}

export interface Witness {
  name: string;
  signature: string; // Digital signature
  timestamp: string; // ISO 8601
}

export interface ArbitrationClause {
  required: boolean;
  forum: string;
  rules: 'AAA' | 'JAMS' | 'ICC' | 'other';
}

// ========== Execution ==========

export interface ExecutionData {
  executionId: string;
  triggerDate: string; // ISO 8601
  triggerType: TriggerType;
  executor: string; // DID
  status: ExecutionStatus;
  transactions: Transaction[];
  completionDate?: string; // ISO 8601
}

export type ExecutionStatus = 'pending' | 'in-progress' | 'completed' | 'contested' | 'failed';

export interface Transaction {
  transactionId: string;
  asset: string; // assetId
  beneficiary: string; // beneficiaryId
  amount: string;
  blockchain?: string;
  transactionHash?: string;
  status: 'pending' | 'confirmed' | 'failed';
  timestamp: string; // ISO 8601
}

// ========== Signatures ==========

export interface Signature {
  signer: string; // DID
  role: 'owner' | 'witness' | 'notary';
  signature: string; // Digital signature
  algorithm: 'Ed25519' | 'ECDSA' | 'RSA';
  timestamp: string; // ISO 8601
}

// ========== API Types ==========

export interface APIResponse<T> {
  success: boolean;
  data?: T;
  error?: APIError;
  metadata: ResponseMetadata;
}

export interface APIError {
  code: string;
  message: string;
  details?: Record<string, any>;
  timestamp: string;
  requestId: string;
  documentation?: string;
}

export interface ResponseMetadata {
  timestamp: string;
  requestId: string;
  version: string;
}

// ========== Client Configuration ==========

export interface WIAClientConfig {
  apiKey?: string;
  baseUrl: string;
  timeout?: number;
  retryAttempts?: number;
  enableLogging?: boolean;
}

// ========== Validation Results ==========

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
}

export interface ValidationError {
  field: string;
  message: string;
  code: string;
}

export interface ValidationWarning {
  field: string;
  message: string;
  severity: 'low' | 'medium' | 'high';
}
