/**
 * WIA-DEF-004: Cyber Weapon - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Threat Types
// ============================================================================

/**
 * Cyber weapon categories
 */
export type CyberWeaponType =
  | 'malware'
  | 'ransomware'
  | 'apt-tool'
  | 'zero-day-exploit'
  | 'ddos-tool'
  | 'spyware'
  | 'rootkit'
  | 'logic-bomb'
  | 'wiper'
  | 'backdoor';

/**
 * Malware classifications
 */
export type MalwareType =
  | 'virus'
  | 'worm'
  | 'trojan'
  | 'ransomware'
  | 'spyware'
  | 'adware'
  | 'rootkit'
  | 'keylogger'
  | 'botnet'
  | 'hybrid';

/**
 * Attack vector types
 */
export type AttackVector =
  | 'network'
  | 'email'
  | 'web'
  | 'supply-chain'
  | 'social-engineering'
  | 'physical'
  | 'insider'
  | 'wireless'
  | 'usb';

/**
 * Threat severity levels
 */
export type ThreatLevel = 'critical' | 'high' | 'medium' | 'low';

/**
 * Cyber weapon status
 */
export interface CyberWeapon {
  /** Weapon identifier */
  id: string;

  /** Weapon type */
  type: CyberWeaponType;

  /** Weapon name/family */
  name: string;

  /** Sophistication level (1-10) */
  sophistication: number;

  /** Threat level */
  threatLevel: ThreatLevel;

  /** Indicators of compromise */
  indicators: IndicatorsOfCompromise;

  /** Attribution information */
  attribution?: Attribution;

  /** First seen timestamp */
  firstSeen: Date;

  /** Last seen timestamp */
  lastSeen: Date;

  /** Active status */
  active: boolean;
}

// ============================================================================
// Malware Analysis
// ============================================================================

/**
 * Malware classification result
 */
export interface MalwareClassification {
  /** Malware hash (SHA-256) */
  hash: string;

  /** Malware type */
  type: MalwareType;

  /** Malware family */
  family?: string;

  /** Severity score (0-10) */
  severity: number;

  /** Behavior patterns */
  behavior: MalwareBehavior;

  /** Propagation method */
  propagation: PropagationMethod;

  /** Payload type */
  payload: PayloadType;

  /** Evasion techniques */
  evasion: string[];

  /** Detection signatures */
  signatures: MalwareSignature[];

  /** Classification confidence (0-1) */
  confidence: number;
}

/**
 * Malware behavior patterns
 */
export interface MalwareBehavior {
  /** File system modifications */
  fileSystem: string[];

  /** Registry modifications (Windows) */
  registry: string[];

  /** Network activity */
  network: NetworkBehavior;

  /** Process creation */
  processes: string[];

  /** Persistence mechanisms */
  persistence: string[];

  /** Privilege escalation */
  privilegeEscalation: boolean;

  /** Data exfiltration */
  dataExfiltration: boolean;
}

/**
 * Network behavior
 */
export interface NetworkBehavior {
  /** C2 (Command & Control) domains */
  c2Domains: string[];

  /** IP addresses contacted */
  ipAddresses: string[];

  /** Ports used */
  ports: number[];

  /** Protocols */
  protocols: string[];

  /** DNS queries */
  dnsQueries: string[];
}

/**
 * Propagation methods
 */
export type PropagationMethod =
  | 'none'
  | 'email'
  | 'network-worm'
  | 'usb'
  | 'exploit'
  | 'social-engineering'
  | 'download'
  | 'supply-chain';

/**
 * Payload types
 */
export type PayloadType =
  | 'ransomware'
  | 'backdoor'
  | 'data-theft'
  | 'keylogger'
  | 'cryptocurrency-miner'
  | 'ddos-bot'
  | 'wiper'
  | 'downloader'
  | 'none';

/**
 * Malware signature
 */
export interface MalwareSignature {
  /** Signature type */
  type: 'yara' | 'hash' | 'pattern' | 'behavioral';

  /** Signature value */
  value: string;

  /** Detection confidence (0-1) */
  confidence: number;
}

// ============================================================================
// Threat Analysis
// ============================================================================

/**
 * Threat analysis parameters
 */
export interface ThreatAnalysisParams {
  /** Threat type */
  type: CyberWeaponType;

  /** Indicators of compromise */
  indicators: Partial<IndicatorsOfCompromise>;

  /** Target information */
  targets?: string[];

  /** Sophistication level (1-10) */
  sophistication?: number;

  /** Observed behavior */
  behavior?: string[];
}

/**
 * Threat analysis result
 */
export interface ThreatAnalysis {
  /** Threat classification */
  classification: string;

  /** Threat level */
  level: ThreatLevel;

  /** Risk score (0-100) */
  riskScore: number;

  /** Impact assessment */
  impact: ImpactAssessment;

  /** Recommended actions */
  recommendations: string[];

  /** Defense priority */
  priority: 'immediate' | 'urgent' | 'high' | 'medium' | 'low';

  /** Estimated recovery time (hours) */
  recoveryEstimate: number;

  /** Affected systems count estimate */
  affectedSystemsEstimate: number;
}

/**
 * Impact assessment
 */
export interface ImpactAssessment {
  /** Confidentiality impact */
  confidentiality: 'none' | 'low' | 'medium' | 'high';

  /** Integrity impact */
  integrity: 'none' | 'low' | 'medium' | 'high';

  /** Availability impact */
  availability: 'none' | 'low' | 'medium' | 'high';

  /** Financial damage estimate */
  financialDamage?: number;

  /** Data loss potential */
  dataLoss: boolean;

  /** Service disruption */
  serviceDisruption: boolean;

  /** Reputational damage */
  reputationalDamage: boolean;
}

// ============================================================================
// Indicators of Compromise (IOC)
// ============================================================================

/**
 * Indicators of compromise
 */
export interface IndicatorsOfCompromise {
  /** File hashes */
  fileHashes?: FileHash[];

  /** Network indicators */
  network?: NetworkIndicators;

  /** System indicators */
  system?: SystemIndicators;

  /** Behavioral indicators */
  behavioral?: string[];

  /** Email indicators */
  email?: EmailIndicators;
}

/**
 * File hash
 */
export interface FileHash {
  /** Hash algorithm */
  algorithm: 'md5' | 'sha1' | 'sha256' | 'ssdeep';

  /** Hash value */
  value: string;

  /** File name */
  fileName?: string;

  /** File size */
  fileSize?: number;
}

/**
 * Network indicators
 */
export interface NetworkIndicators {
  /** Malicious domains */
  domains: string[];

  /** IP addresses */
  ipAddresses: string[];

  /** URLs */
  urls: string[];

  /** User agents */
  userAgents?: string[];

  /** Certificates */
  certificates?: string[];
}

/**
 * System indicators
 */
export interface SystemIndicators {
  /** Registry keys */
  registryKeys?: string[];

  /** File paths */
  filePaths: string[];

  /** Mutexes */
  mutexes?: string[];

  /** Services */
  services?: string[];

  /** Scheduled tasks */
  scheduledTasks?: string[];
}

/**
 * Email indicators
 */
export interface EmailIndicators {
  /** Sender addresses */
  senderAddresses: string[];

  /** Subject patterns */
  subjects: string[];

  /** Attachment names */
  attachmentNames?: string[];

  /** Email headers */
  headers?: Record<string, string>;
}

// ============================================================================
// Vulnerability Assessment
// ============================================================================

/**
 * Vulnerability assessment parameters
 */
export interface VulnerabilityAssessment {
  /** CVE identifier */
  cve: string;

  /** CVSS score */
  cvss: CVSSScore;

  /** Exploitability */
  exploitability: ExploitabilityInfo;

  /** Affected assets */
  affectedAssets: AffectedAssets;

  /** Patch availability */
  patchAvailable: boolean;

  /** Workarounds available */
  workarounds?: string[];
}

/**
 * CVSS scoring
 */
export interface CVSSScore {
  /** Base score (0-10) */
  baseScore: number;

  /** Vector string */
  vector: string;

  /** Attack vector */
  attackVector: 'network' | 'adjacent' | 'local' | 'physical';

  /** Attack complexity */
  attackComplexity: 'low' | 'high';

  /** Privileges required */
  privilegesRequired: 'none' | 'low' | 'high';

  /** User interaction */
  userInteraction: 'none' | 'required';
}

/**
 * Exploitability information
 */
export interface ExploitabilityInfo {
  /** Exploit code available */
  exploitAvailable: boolean;

  /** Exploit complexity */
  exploitComplexity: 'low' | 'medium' | 'high';

  /** Weaponized exploit exists */
  weaponized: boolean;

  /** Exploit maturity */
  maturity: 'unproven' | 'poc' | 'functional' | 'high';

  /** Active exploitation observed */
  activeExploitation: boolean;
}

/**
 * Affected assets
 */
export interface AffectedAssets {
  /** Number of affected systems */
  count: number;

  /** Asset criticality */
  criticality: 'low' | 'medium' | 'high' | 'critical';

  /** Internet exposure */
  exposure: 'internal' | 'dmz' | 'internet-facing';

  /** Asset types */
  assetTypes: string[];
}

/**
 * Vulnerability result
 */
export interface VulnerabilityResult {
  /** Remediation priority */
  priority: 'critical' | 'high' | 'medium' | 'low';

  /** Recommended patch deadline */
  patchDeadline: Date;

  /** Workarounds */
  workarounds: string[];

  /** Compensating controls */
  compensatingControls: string[];

  /** Risk level */
  riskLevel: 'extreme' | 'high' | 'medium' | 'low';
}

// ============================================================================
// Attribution
// ============================================================================

/**
 * Attribution parameters
 */
export interface AttributionParams {
  /** Technical indicators */
  technical: TechnicalIndicators;

  /** Operational patterns */
  operational: OperationalPatterns;

  /** Geopolitical context */
  geopolitical: GeopoliticalContext;

  /** Confidence threshold (0-1) */
  confidenceThreshold?: number;
}

/**
 * Technical indicators for attribution
 */
export interface TechnicalIndicators {
  /** Malware families used */
  malware: string[];

  /** Infrastructure characteristics */
  infrastructure: string[];

  /** Tools and frameworks */
  tools: string[];

  /** Code artifacts */
  codeArtifacts?: string[];

  /** Language indicators */
  language?: string[];
}

/**
 * Operational patterns
 */
export interface OperationalPatterns {
  /** Target industries/sectors */
  targets: string[];

  /** Campaign timing patterns */
  timing: string;

  /** Tactics, Techniques, Procedures */
  ttps: string[];

  /** Attack duration */
  duration?: string;

  /** Operational security level */
  opsecLevel?: 'low' | 'medium' | 'high';
}

/**
 * Geopolitical context
 */
export interface GeopoliticalContext {
  /** Apparent motivation */
  motivation: 'espionage' | 'financial' | 'disruption' | 'sabotage' | 'unknown';

  /** Likely beneficiary */
  beneficiary: 'nation-state' | 'criminal' | 'hacktivist' | 'terrorist' | 'unknown';

  /** Resource indicators */
  resources: 'low-budget' | 'moderate' | 'well-funded' | 'state-sponsored';

  /** Political context */
  politicalContext?: string;
}

/**
 * Attribution result
 */
export interface AttributionResult {
  /** Attribution confidence (0-1) */
  confidence: number;

  /** Likely threat actor */
  actor: string;

  /** Actor type */
  actorType: 'nation-state' | 'criminal-group' | 'hacktivist' | 'insider' | 'unknown';

  /** Supporting evidence */
  evidence: string[];

  /** Alternative hypotheses */
  alternatives: string[];

  /** False flag indicators */
  falseFlagPotential: 'low' | 'medium' | 'high';
}

// ============================================================================
// Defense Strategy
// ============================================================================

/**
 * Defense strategy parameters
 */
export interface DefenseStrategyParams {
  /** Threat type to defend against */
  threatType: CyberWeaponType;

  /** Asset types to protect */
  assets: string[];

  /** Current security posture */
  currentPosture?: SecurityPosture;

  /** Budget constraints */
  budgetLevel?: 'low' | 'medium' | 'high' | 'unlimited';
}

/**
 * Security posture
 */
export interface SecurityPosture {
  /** Existing controls */
  controls: string[];

  /** Security maturity level (1-5) */
  maturityLevel: number;

  /** Recent incidents */
  recentIncidents: number;

  /** Vulnerability count */
  vulnerabilities: number;
}

/**
 * Defense strategy
 */
export interface DefenseStrategy {
  /** Prevention measures */
  prevention: DefenseMeasure[];

  /** Detection capabilities */
  detection: DefenseMeasure[];

  /** Response procedures */
  response: DefenseMeasure[];

  /** Recovery plans */
  recovery: DefenseMeasure[];

  /** Estimated implementation time */
  implementationTime: string;

  /** Estimated cost */
  estimatedCost?: string;

  /** Effectiveness rating (0-10) */
  effectiveness: number;
}

/**
 * Defense measure
 */
export interface DefenseMeasure {
  /** Measure name */
  name: string;

  /** Description */
  description: string;

  /** Priority */
  priority: 'critical' | 'high' | 'medium' | 'low';

  /** Implementation complexity */
  complexity: 'low' | 'medium' | 'high';

  /** Estimated timeframe */
  timeframe: string;

  /** Dependencies */
  dependencies?: string[];
}

// ============================================================================
// Incident Response
// ============================================================================

/**
 * Incident classification
 */
export type IncidentSeverity = 'critical' | 'high' | 'medium' | 'low' | 'informational';

/**
 * Incident status
 */
export type IncidentStatus =
  | 'detected'
  | 'analyzing'
  | 'contained'
  | 'eradicated'
  | 'recovered'
  | 'closed';

/**
 * Security incident
 */
export interface SecurityIncident {
  /** Incident ID */
  id: string;

  /** Incident type */
  type: string;

  /** Severity */
  severity: IncidentSeverity;

  /** Current status */
  status: IncidentStatus;

  /** Detection time */
  detectedAt: Date;

  /** Affected systems */
  affectedSystems: string[];

  /** Indicators observed */
  indicators: IndicatorsOfCompromise;

  /** Incident timeline */
  timeline: IncidentEvent[];

  /** Response actions */
  actions: ResponseAction[];

  /** Lessons learned */
  lessonsLearned?: string[];
}

/**
 * Incident event
 */
export interface IncidentEvent {
  /** Timestamp */
  timestamp: Date;

  /** Event type */
  type: string;

  /** Description */
  description: string;

  /** System/user involved */
  source: string;
}

/**
 * Response action
 */
export interface ResponseAction {
  /** Action type */
  type: 'containment' | 'eradication' | 'recovery' | 'investigation';

  /** Action description */
  description: string;

  /** Timestamp */
  timestamp: Date;

  /** Assigned to */
  assignedTo: string;

  /** Status */
  status: 'pending' | 'in-progress' | 'completed';

  /** Outcome */
  outcome?: string;
}

// ============================================================================
// Threat Intelligence
// ============================================================================

/**
 * Threat intelligence feed
 */
export interface ThreatIntelligence {
  /** Feed ID */
  id: string;

  /** Feed source */
  source: string;

  /** Intelligence type */
  type: 'strategic' | 'tactical' | 'operational' | 'technical';

  /** Indicators */
  indicators: IndicatorsOfCompromise;

  /** TTPs */
  ttps: string[];

  /** Threat actors */
  actors: string[];

  /** Confidence level (0-1) */
  confidence: number;

  /** Publication date */
  published: Date;

  /** Expiration date */
  expires?: Date;
}

/**
 * Threat hunting query
 */
export interface ThreatHuntingQuery {
  /** Query ID */
  id: string;

  /** Hypothesis */
  hypothesis: string;

  /** Query type */
  type: 'network' | 'endpoint' | 'log' | 'behavioral';

  /** Query string */
  query: string;

  /** Expected indicators */
  expectedIndicators: string[];

  /** Data sources */
  dataSources: string[];

  /** MITRE ATT&CK techniques */
  mitreAttack?: string[];
}

// ============================================================================
// Monitoring & Alerts
// ============================================================================

/**
 * Security alert
 */
export interface SecurityAlert {
  /** Alert ID */
  id: string;

  /** Alert name */
  name: string;

  /** Severity */
  severity: ThreatLevel;

  /** Detection source */
  source: string;

  /** Timestamp */
  timestamp: Date;

  /** Affected assets */
  affectedAssets: string[];

  /** Alert details */
  details: Record<string, unknown>;

  /** Recommended actions */
  recommendations: string[];

  /** Status */
  status: 'new' | 'investigating' | 'confirmed' | 'false-positive' | 'resolved';
}

/**
 * Monitoring metrics
 */
export interface SecurityMetrics {
  /** Monitoring period */
  period: {
    start: Date;
    end: Date;
  };

  /** Total alerts */
  totalAlerts: number;

  /** Alerts by severity */
  alertsBySeverity: Record<ThreatLevel, number>;

  /** Incidents detected */
  incidentsDetected: number;

  /** Mean time to detect (MTTD) in seconds */
  mttd: number;

  /** Mean time to respond (MTTR) in seconds */
  mttr: number;

  /** Top threats */
  topThreats: string[];

  /** Coverage percentage */
  coverage: number;
}

// ============================================================================
// Constants & Enums
// ============================================================================

/**
 * MITRE ATT&CK tactics
 */
export const MITRE_TACTICS = [
  'reconnaissance',
  'resource-development',
  'initial-access',
  'execution',
  'persistence',
  'privilege-escalation',
  'defense-evasion',
  'credential-access',
  'discovery',
  'lateral-movement',
  'collection',
  'command-and-control',
  'exfiltration',
  'impact',
] as const;

/**
 * Cyber kill chain phases
 */
export const KILL_CHAIN_PHASES = [
  'reconnaissance',
  'weaponization',
  'delivery',
  'exploitation',
  'installation',
  'command-and-control',
  'actions-on-objectives',
] as const;

/**
 * Common ports
 */
export const COMMON_ATTACK_PORTS = {
  FTP: 21,
  SSH: 22,
  TELNET: 23,
  SMTP: 25,
  HTTP: 80,
  HTTPS: 443,
  SMB: 445,
  RDP: 3389,
  MYSQL: 3306,
  POSTGRESQL: 5432,
} as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * Error codes
 */
export enum CyberDefenseErrorCode {
  INVALID_PARAMETERS = 'E001',
  ANALYSIS_FAILED = 'E002',
  INSUFFICIENT_DATA = 'E003',
  ATTRIBUTION_UNCERTAIN = 'E004',
  NO_DEFENSE_STRATEGY = 'E005',
  THREAT_INTELLIGENCE_UNAVAILABLE = 'E006',
}

/**
 * Cyber defense error
 */
export class CyberDefenseError extends Error {
  constructor(
    public code: CyberDefenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CyberDefenseError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Export All Types
// ============================================================================

export type {
  // Core types
  CyberWeapon,

  // Malware
  MalwareClassification,
  MalwareBehavior,
  NetworkBehavior,
  MalwareSignature,

  // Threat analysis
  ThreatAnalysisParams,
  ThreatAnalysis,
  ImpactAssessment,

  // IOC
  IndicatorsOfCompromise,
  FileHash,
  NetworkIndicators,
  SystemIndicators,
  EmailIndicators,

  // Vulnerability
  VulnerabilityAssessment,
  CVSSScore,
  ExploitabilityInfo,
  AffectedAssets,
  VulnerabilityResult,

  // Attribution
  AttributionParams,
  TechnicalIndicators,
  OperationalPatterns,
  GeopoliticalContext,
  AttributionResult,

  // Defense
  DefenseStrategyParams,
  SecurityPosture,
  DefenseStrategy,
  DefenseMeasure,

  // Incident Response
  SecurityIncident,
  IncidentEvent,
  ResponseAction,

  // Threat Intelligence
  ThreatIntelligence,
  ThreatHuntingQuery,

  // Monitoring
  SecurityAlert,
  SecurityMetrics,
};

export { MITRE_TACTICS, KILL_CHAIN_PHASES, COMMON_ATTACK_PORTS };
export { CyberDefenseErrorCode, CyberDefenseError };
