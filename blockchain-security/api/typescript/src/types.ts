/**
 * WIA Blockchain Security Standard - Type Definitions
 *
 * @packageDocumentation
 * @module wia-blockchain-security
 */

export enum BlockchainNetwork {
  Ethereum = 'ethereum',
  Polygon = 'polygon',
  BSC = 'bsc',
  Solana = 'solana',
  Avalanche = 'avalanche',
  Arbitrum = 'arbitrum',
  Optimism = 'optimism',
  Base = 'base'
}

export enum VulnerabilityType {
  Reentrancy = 'reentrancy',
  IntegerOverflow = 'integer_overflow',
  AccessControl = 'access_control',
  FrontRunning = 'front_running',
  FlashLoan = 'flash_loan',
  OracleManipulation = 'oracle_manipulation',
  Rugpull = 'rugpull',
  PhishingContract = 'phishing_contract',
  PrivateKeyLeak = 'private_key_leak',
  LogicError = 'logic_error'
}

export enum Severity {
  Critical = 'critical',
  High = 'high',
  Medium = 'medium',
  Low = 'low',
  Info = 'info'
}

export enum ContractType {
  ERC20 = 'erc20',
  ERC721 = 'erc721',
  ERC1155 = 'erc1155',
  DeFi = 'defi',
  DAO = 'dao',
  Bridge = 'bridge',
  DEX = 'dex',
  Lending = 'lending',
  Staking = 'staking',
  Custom = 'custom'
}

export interface ContractAddress {
  address: string;
  network: BlockchainNetwork;
  chainId: number;
}

export interface SmartContract {
  id: string;
  address: ContractAddress;
  name: string;
  type: ContractType;
  sourceCode?: string;
  bytecode: string;
  abi: ContractABI[];
  verified: boolean;
  deployer: string;
  deployedAt: Date;
  metadata: ContractMetadata;
}

export interface ContractABI {
  name: string;
  type: 'function' | 'event' | 'constructor' | 'fallback' | 'receive';
  inputs: ABIParameter[];
  outputs?: ABIParameter[];
  stateMutability?: 'pure' | 'view' | 'nonpayable' | 'payable';
}

export interface ABIParameter {
  name: string;
  type: string;
  indexed?: boolean;
}

export interface ContractMetadata {
  compiler: string;
  version: string;
  optimization: boolean;
  runs?: number;
  license?: string;
}

export interface SecurityAudit {
  id: string;
  contract: SmartContract;
  auditor: string;
  findings: Finding[];
  score: number;
  status: 'in_progress' | 'completed' | 'passed' | 'failed';
  startedAt: Date;
  completedAt?: Date;
  reportUrl?: string;
}

export interface Finding {
  id: string;
  type: VulnerabilityType;
  severity: Severity;
  title: string;
  description: string;
  location: CodeLocation;
  recommendation: string;
  status: 'open' | 'acknowledged' | 'fixed' | 'wont_fix';
  cwe?: string;
}

export interface CodeLocation {
  file?: string;
  line?: number;
  function?: string;
  startOffset?: number;
  endOffset?: number;
}

export interface ThreatAlert {
  id: string;
  type: ThreatType;
  severity: Severity;
  contract: ContractAddress;
  transaction?: string;
  description: string;
  indicators: ThreatIndicator[];
  timestamp: Date;
  status: 'active' | 'investigating' | 'resolved' | 'false_positive';
}

export enum ThreatType {
  SuspiciousTransaction = 'suspicious_transaction',
  ExploitAttempt = 'exploit_attempt',
  AbnormalActivity = 'abnormal_activity',
  KnownMalicious = 'known_malicious',
  RugpullWarning = 'rugpull_warning',
  FlashLoanAttack = 'flash_loan_attack'
}

export interface ThreatIndicator {
  type: string;
  value: string;
  confidence: number;
}

export interface TransactionAnalysis {
  hash: string;
  network: BlockchainNetwork;
  from: string;
  to: string;
  value: string;
  gasUsed: number;
  risk: RiskAssessment;
  decodedInput?: DecodedInput;
  internalCalls: InternalCall[];
  events: DecodedEvent[];
}

export interface RiskAssessment {
  score: number;
  level: 'safe' | 'low' | 'medium' | 'high' | 'critical';
  factors: RiskFactor[];
}

export interface RiskFactor {
  name: string;
  impact: number;
  description: string;
}

export interface DecodedInput {
  functionName: string;
  parameters: Record<string, unknown>;
}

export interface InternalCall {
  type: 'call' | 'delegatecall' | 'staticcall' | 'create' | 'create2';
  from: string;
  to: string;
  value: string;
  gasUsed: number;
}

export interface DecodedEvent {
  name: string;
  address: string;
  parameters: Record<string, unknown>;
}

export interface WalletRisk {
  address: string;
  riskScore: number;
  labels: string[];
  associations: WalletAssociation[];
  firstSeen: Date;
  lastActive: Date;
  transactionCount: number;
}

export interface WalletAssociation {
  address: string;
  type: 'funded_by' | 'funded' | 'interacted' | 'contract_created';
  riskLevel: string;
}

export interface SecurityPolicy {
  id: string;
  name: string;
  rules: SecurityRule[];
  actions: PolicyAction[];
  enabled: boolean;
}

export interface SecurityRule {
  type: 'transaction' | 'contract' | 'wallet' | 'event';
  condition: string;
  threshold?: number;
}

export interface PolicyAction {
  type: 'alert' | 'block' | 'pause' | 'notify';
  target: string;
  parameters?: Record<string, unknown>;
}

export interface BlockchainSecurityConfig {
  networks: BlockchainNetwork[];
  rpcEndpoints: Record<BlockchainNetwork, string>;
  apiKey?: string;
  alertWebhook?: string;
  enableRealTimeMonitoring: boolean;
  scanDepth: number;
}

export enum CertificationLevel {
  Bronze = 'bronze',
  Silver = 'silver',
  Gold = 'gold'
}

export interface ComplianceReport {
  standard: 'WIA-BLOCKCHAIN-SECURITY';
  testDate: string;
  config: BlockchainSecurityConfig;
  targetLevel: CertificationLevel;
  tests: TestResult[];
  passed: boolean;
  achievedLevel?: CertificationLevel;
}

export interface TestResult {
  testName: string;
  passed: boolean;
  notes?: string;
}
