/**
 * WIA-AUTO-023: Vehicle Cybersecurity - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Automotive Security Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Security Types
// ============================================================================

/**
 * Vehicle Identification Number
 */
export type VIN = string;

/**
 * Security severity levels
 */
export type SecuritySeverity = 'info' | 'low' | 'medium' | 'high' | 'critical';

/**
 * ISO/SAE 21434 Cybersecurity Assurance Levels
 */
export type CAL = 'CAL-1' | 'CAL-2' | 'CAL-3' | 'CAL-4';

/**
 * Security risk levels
 */
export type RiskLevel = 'low' | 'medium' | 'high' | 'critical';

/**
 * Attack feasibility levels (ISO 21434)
 */
export type AttackFeasibility = 'very-low' | 'low' | 'medium' | 'high' | 'very-high';

// ============================================================================
// ECU Security
// ============================================================================

/**
 * Electronic Control Unit information
 */
export interface ECU {
  /** ECU identifier */
  id: string;

  /** ECU name/type */
  name: string;

  /** Hardware version */
  hardwareVersion: string;

  /** Software/firmware version */
  softwareVersion: string;

  /** Security version (for rollback protection) */
  securityVersion: number;

  /** Has Hardware Security Module */
  hasHSM: boolean;

  /** Secure boot enabled */
  secureBootEnabled: boolean;

  /** Code signing enforced */
  codeSigningEnabled: boolean;

  /** Cybersecurity Assurance Level */
  cal: CAL;
}

/**
 * Secure boot verification result
 */
export interface SecureBootResult {
  /** Is boot chain valid */
  isValid: boolean;

  /** Verification stage */
  stage: 'bootloader' | 'os' | 'application';

  /** Signature valid */
  signatureValid: boolean;

  /** Hash/integrity valid */
  integrityValid: boolean;

  /** Version check passed */
  versionValid: boolean;

  /** Error messages if invalid */
  errors: string[];
}

/**
 * HSM (Hardware Security Module) operations
 */
export interface HSMOperation {
  /** Operation type */
  type: 'encrypt' | 'decrypt' | 'sign' | 'verify' | 'derive-key' | 'generate-random';

  /** Algorithm used */
  algorithm: string;

  /** Key identifier */
  keyId: string;

  /** Input data size in bytes */
  inputSize: number;

  /** Output data size in bytes */
  outputSize: number;

  /** Operation duration in milliseconds */
  duration: number;

  /** Success status */
  success: boolean;
}

/**
 * Code signature information
 */
export interface CodeSignature {
  /** Signature algorithm */
  algorithm: 'ECDSA-P256' | 'ECDSA-P384' | 'RSA-2048' | 'RSA-4096';

  /** Signature value (hex encoded) */
  value: string;

  /** Hash algorithm */
  hashAlgorithm: 'SHA-256' | 'SHA-384' | 'SHA-512';

  /** Hash value */
  hashValue: string;

  /** Signing certificate */
  certificate: string;

  /** Certificate chain */
  certificateChain: string[];

  /** Signature timestamp */
  timestamp: Date;

  /** Signer organization */
  signer: string;
}

// ============================================================================
// Network Security
// ============================================================================

/**
 * In-vehicle network types
 */
export type NetworkType = 'CAN' | 'CAN-FD' | 'LIN' | 'FlexRay' | 'Ethernet';

/**
 * CAN message with security
 */
export interface SecureCANMessage {
  /** CAN identifier */
  canId: number;

  /** Data payload (up to 8 bytes for CAN classic, 64 for CAN-FD) */
  data: Uint8Array;

  /** Message Authentication Code */
  mac?: Uint8Array;

  /** Freshness counter */
  counter?: number;

  /** Timestamp */
  timestamp: number;

  /** Source ECU */
  source?: string;

  /** DLC (Data Length Code) */
  dlc: number;
}

/**
 * CAN bus firewall rule
 */
export interface CANFirewallRule {
  /** Rule identifier */
  id: string;

  /** Rule action */
  action: 'allow' | 'block' | 'inspect' | 'throttle';

  /** CAN ID or range */
  canId: number | { min: number; max: number };

  /** Allowed source ECU */
  sourceECU?: string;

  /** Rate limit (messages per second) */
  rateLimit?: number;

  /** Require MAC */
  requireMAC?: boolean;

  /** Rule description */
  description: string;

  /** Rule enabled */
  enabled: boolean;
}

/**
 * Network security status
 */
export interface NetworkSecurityStatus {
  /** Network type */
  network: NetworkType;

  /** Firewall enabled */
  firewallEnabled: boolean;

  /** Encryption enabled */
  encryptionEnabled: boolean;

  /** Authentication enabled */
  authenticationEnabled: boolean;

  /** Active firewall rules */
  activeRules: number;

  /** Messages processed */
  messagesProcessed: number;

  /** Messages blocked */
  messagesBlocked: number;

  /** Anomalies detected */
  anomaliesDetected: number;

  /** Health status */
  health: 'healthy' | 'warning' | 'critical';
}

/**
 * Automotive Ethernet security configuration
 */
export interface EthernetSecurityConfig {
  /** MACsec enabled */
  macsecEnabled: boolean;

  /** IPsec enabled */
  ipsecEnabled: boolean;

  /** TLS version */
  tlsVersion?: 'TLS_1.2' | 'TLS_1.3';

  /** VLAN segmentation */
  vlanEnabled: boolean;

  /** VLAN IDs in use */
  vlans?: number[];

  /** Cipher suites */
  cipherSuites: string[];
}

// ============================================================================
// External Interfaces
// ============================================================================

/**
 * V2X (Vehicle-to-Everything) message types
 */
export type V2XMessageType = 'CAM' | 'DENM' | 'BSM' | 'SPAT' | 'MAP' | 'PSM';

/**
 * V2X secured message
 */
export interface V2XSecuredMessage {
  /** Message type */
  type: V2XMessageType;

  /** Protocol version */
  protocolVersion: number;

  /** Message payload */
  payload: Uint8Array;

  /** Signer certificate */
  certificate: Uint8Array;

  /** Digital signature (ECDSA) */
  signature: Uint8Array;

  /** Generation time */
  generationTime: Date;

  /** Generation location */
  generationLocation?: {
    latitude: number;
    longitude: number;
    elevation?: number;
  };

  /** Signature verified */
  signatureVerified?: boolean;

  /** Certificate valid */
  certificateValid?: boolean;
}

/**
 * OBD-II security access
 */
export interface OBDSecurityAccess {
  /** Security level */
  level: 0 | 1 | 2 | 3;

  /** Access granted */
  granted: boolean;

  /** Session start time */
  sessionStart: Date;

  /** Session timeout (seconds) */
  timeout: number;

  /** Failed attempts */
  failedAttempts: number;

  /** Locked due to failed attempts */
  locked: boolean;

  /** Unlock time (if locked) */
  unlockTime?: Date;
}

/**
 * Telematics connection security
 */
export interface TelematicsSecurityConfig {
  /** TLS enabled */
  tlsEnabled: boolean;

  /** TLS version */
  tlsVersion: 'TLS_1.2' | 'TLS_1.3';

  /** Mutual authentication */
  mutualAuth: boolean;

  /** Certificate pinning */
  certPinning: boolean;

  /** Pinned certificate hashes */
  pinnedCerts: string[];

  /** Session resumption enabled */
  sessionResumption: boolean;

  /** Connection timeout (seconds) */
  timeout: number;
}

// ============================================================================
// OTA Updates
// ============================================================================

/**
 * OTA update package
 */
export interface OTAUpdatePackage {
  /** Package identifier */
  id: string;

  /** Package version */
  version: string;

  /** Previous version */
  previousVersion: string;

  /** Release date */
  releaseDate: Date;

  /** Priority */
  priority: 'critical' | 'high' | 'normal' | 'low';

  /** Target ECUs */
  targetECUs: string[];

  /** Package size in bytes */
  sizeBytes: number;

  /** Compressed */
  compressed: boolean;

  /** Compression algorithm */
  compressionAlgorithm?: string;

  /** Encrypted */
  encrypted: boolean;

  /** Encryption algorithm */
  encryptionAlgorithm?: string;

  /** Package hash */
  hash: {
    algorithm: string;
    value: string;
  };

  /** Digital signature */
  signature: CodeSignature;

  /** Download URL */
  downloadUrl: string;

  /** Metadata */
  metadata: {
    description: string;
    rollbackAllowed: boolean;
    requiresIgnitionOff: boolean;
    requiresFullCharge: boolean;
    estimatedDurationSeconds: number;
    requiresUserConfirmation: boolean;
  };
}

/**
 * OTA update validation result
 */
export interface OTAValidationResult {
  /** Is package valid */
  isValid: boolean;

  /** Signature verification */
  signatureValid: boolean;

  /** Hash verification */
  hashValid: boolean;

  /** Certificate chain valid */
  certificateChainValid: boolean;

  /** Version compatibility */
  versionCompatible: boolean;

  /** No threats detected */
  noThreats: boolean;

  /** Validation errors */
  errors: string[];

  /** Validation warnings */
  warnings: string[];
}

/**
 * OTA update status
 */
export interface OTAUpdateStatus {
  /** Update ID */
  updateId: string;

  /** Current status */
  status: 'pending' | 'downloading' | 'verifying' | 'installing' | 'completed' | 'failed' | 'rolled-back';

  /** Progress percentage (0-100) */
  progressPercent: number;

  /** Current stage description */
  currentStage: string;

  /** Estimated time remaining (seconds) */
  estimatedTimeRemaining?: number;

  /** Affected ECUs */
  affectedECUs: string[];

  /** Error message (if failed) */
  errorMessage?: string;

  /** Rollback available */
  rollbackAvailable: boolean;

  /** Started at */
  startedAt: Date;

  /** Completed at */
  completedAt?: Date;
}

// ============================================================================
// Intrusion Detection
// ============================================================================

/**
 * IDS detection method
 */
export type IDSDetectionMethod = 'signature' | 'anomaly' | 'behavioral' | 'heuristic';

/**
 * Security threat
 */
export interface SecurityThreat {
  /** Threat identifier */
  id: string;

  /** Threat type */
  type: 'intrusion' | 'dos' | 'spoofing' | 'tampering' | 'data-breach' | 'malware' | 'other';

  /** Severity */
  severity: SecuritySeverity;

  /** Confidence level (0-1) */
  confidence: number;

  /** Detection method */
  detectionMethod: IDSDetectionMethod;

  /** Signature ID (if signature-based) */
  signatureId?: string;

  /** Detection timestamp */
  timestamp: Date;

  /** Source network/interface */
  source: {
    network: NetworkType | 'OBD' | 'Telematics' | 'V2X' | 'Bluetooth';
    identifier: string;
    location?: string;
  };

  /** Threat description */
  description: string;

  /** Evidence */
  evidence: {
    rawData?: string;
    decodedData?: any;
    context: Record<string, any>;
  };

  /** Recommended actions */
  recommendations: string[];
}

/**
 * IDS response action
 */
export type IDSResponseAction = 'log' | 'alert' | 'throttle' | 'isolate' | 'block' | 'shutdown' | 'safe-mode';

/**
 * IDS alert
 */
export interface IDSAlert {
  /** Alert identifier */
  id: string;

  /** Associated threat */
  threat: SecurityThreat;

  /** Response actions taken */
  actionsTaken: IDSResponseAction[];

  /** Action effectiveness */
  effectiveness: 'successful' | 'partial' | 'failed' | 'unknown';

  /** User action required */
  userActionRequired: boolean;

  /** Alert timestamp */
  timestamp: Date;

  /** Alert acknowledged */
  acknowledged: boolean;

  /** Acknowledged by */
  acknowledgedBy?: string;

  /** Notes */
  notes?: string;
}

/**
 * IDS configuration
 */
export interface IDSConfiguration {
  /** IDS enabled */
  enabled: boolean;

  /** Monitored networks */
  monitoredNetworks: {
    canHighSpeed: boolean;
    canLowSpeed: boolean;
    flexray: boolean;
    ethernet: boolean;
    v2x: boolean;
    telematics: boolean;
    obd: boolean;
    bluetooth: boolean;
  };

  /** Detection methods */
  detectionMethods: {
    signatureBased: boolean;
    anomalyBased: boolean;
    behavioralAnalysis: boolean;
  };

  /** Response actions */
  responseActions: {
    log: boolean;
    alertDriver: boolean;
    alertBackend: boolean;
    autoMitigate: boolean;
  };

  /** Sensitivity level */
  sensitivity: 'low' | 'medium' | 'high' | 'paranoid';

  /** Update threat signatures automatically */
  autoUpdateSignatures: boolean;
}

// ============================================================================
// Security Assessment
// ============================================================================

/**
 * Vulnerability information
 */
export interface Vulnerability {
  /** Vulnerability identifier */
  id: string;

  /** CVE ID (if applicable) */
  cveId?: string;

  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Severity */
  severity: SecuritySeverity;

  /** CVSS score */
  cvss?: {
    version: string;
    score: number;
    vector: string;
  };

  /** Affected components */
  affectedComponents: string[];

  /** Affected versions */
  affectedVersions: string[];

  /** Patch available */
  patchAvailable: boolean;

  /** Patch ID */
  patchId?: string;

  /** Workarounds */
  workarounds: string[];

  /** Discovered date */
  discovered?: Date;

  /** Disclosed date */
  disclosed?: Date;
}

/**
 * Security weakness (non-vulnerability issue)
 */
export interface SecurityWeakness {
  /** Weakness identifier */
  id: string;

  /** CWE ID (if applicable) */
  cweId?: string;

  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Risk level */
  riskLevel: RiskLevel;

  /** Affected components */
  affectedComponents: string[];

  /** Recommendations */
  recommendations: string[];
}

/**
 * Security recommendation
 */
export interface SecurityRecommendation {
  /** Recommendation identifier */
  id: string;

  /** Priority */
  priority: 'critical' | 'high' | 'medium' | 'low';

  /** Category */
  category: 'configuration' | 'update' | 'monitoring' | 'policy' | 'hardware';

  /** Title */
  title: string;

  /** Description */
  description: string;

  /** Implementation steps */
  steps: string[];

  /** Expected benefit */
  benefit: string;

  /** Estimated effort */
  effort: 'low' | 'medium' | 'high';
}

/**
 * Compliance status
 */
export interface ComplianceStatus {
  /** Standard/regulation name */
  standard: string;

  /** Compliant */
  compliant: boolean;

  /** Compliance score (0-100) */
  score: number;

  /** Requirements met */
  requirementsMet: number;

  /** Total requirements */
  totalRequirements: number;

  /** Non-compliant items */
  nonCompliantItems: string[];

  /** Assessment date */
  assessmentDate: Date;
}

/**
 * Security assessment result
 */
export interface SecurityAssessment {
  /** Assessment identifier */
  id: string;

  /** Assessment timestamp */
  timestamp: Date;

  /** Vehicle VIN */
  vin: VIN;

  /** Overall security score (0-100) */
  overallScore: number;

  /** Risk level */
  riskLevel: RiskLevel;

  /** Vulnerabilities found */
  vulnerabilities: Vulnerability[];

  /** Security weaknesses */
  weaknesses: SecurityWeakness[];

  /** Recommendations */
  recommendations: SecurityRecommendation[];

  /** Compliance status */
  compliance: {
    iso21434: ComplianceStatus;
    unece_wp29: ComplianceStatus;
    nhtsa: ComplianceStatus;
  };

  /** Network security status */
  networkSecurity: {
    can: NetworkSecurityStatus;
    flexray: NetworkSecurityStatus;
    ethernet: NetworkSecurityStatus;
  };

  /** ECU security status */
  ecuSecurity: {
    [ecuId: string]: {
      secureBootEnabled: boolean;
      codeSigningEnabled: boolean;
      hasHSM: boolean;
      lastUpdated: Date;
      vulnerabilities: number;
    };
  };

  /** External interface security */
  externalInterfaces: {
    v2x: { enabled: boolean; secure: boolean; score: number };
    telematics: { enabled: boolean; secure: boolean; score: number };
    obd: { enabled: boolean; secure: boolean; score: number };
    bluetooth: { enabled: boolean; secure: boolean; score: number };
  };
}

// ============================================================================
// ISO/SAE 21434 TARA
// ============================================================================

/**
 * Asset in threat analysis
 */
export interface Asset {
  /** Asset identifier */
  id: string;

  /** Asset name */
  name: string;

  /** Asset type */
  type: 'ECU' | 'network' | 'data' | 'function' | 'interface';

  /** Criticality */
  criticality: 'safety-critical' | 'security-critical' | 'non-critical';

  /** Asset value */
  value: 'very-high' | 'high' | 'medium' | 'low';

  /** Description */
  description: string;
}

/**
 * Threat scenario
 */
export interface ThreatScenario {
  /** Scenario identifier */
  id: string;

  /** Scenario name */
  name: string;

  /** Targeted asset */
  asset: string;

  /** Attack path */
  attackPath: string;

  /** Attack feasibility */
  attackFeasibility: AttackFeasibility;

  /** Impact level */
  impact: 'severe' | 'major' | 'moderate' | 'negligible';

  /** Risk value */
  risk: number;

  /** Mitigation measures */
  mitigations: Array<{
    control: string;
    effectiveness: 'high' | 'medium' | 'low';
  }>;

  /** Residual risk */
  residualRisk: number;
}

/**
 * TARA (Threat Analysis and Risk Assessment) result
 */
export interface TARAResult {
  /** Assessment identifier */
  id: string;

  /** Assessment date */
  date: Date;

  /** Vehicle/system */
  vehicle: string;

  /** Assets identified */
  assets: Asset[];

  /** Threat scenarios */
  threats: ThreatScenario[];

  /** High-risk scenarios */
  highRiskScenarios: ThreatScenario[];

  /** Cybersecurity goals */
  cybersecurityGoals: string[];

  /** Required CAL */
  requiredCAL: CAL;
}

// ============================================================================
// Incident Response
// ============================================================================

/**
 * Security incident
 */
export interface SecurityIncident {
  /** Incident identifier */
  id: string;

  /** Incident timestamp */
  timestamp: Date;

  /** Vehicle VIN */
  vin: VIN;

  /** Incident type */
  type: 'intrusion' | 'malware' | 'data-breach' | 'dos' | 'unauthorized-access' | 'other';

  /** Severity */
  severity: SecuritySeverity;

  /** Confidence */
  confidence: number;

  /** Affected systems */
  affectedSystems: string[];

  /** Safety impact */
  safetyImpact: boolean;

  /** Description */
  description: string;

  /** Evidence */
  evidence: {
    logs: string[];
    packetCapture?: string;
    screenshots?: string[];
    hash: string;
  };

  /** Response actions */
  response: {
    actionsTaken: IDSResponseAction[];
    effectiveness: string;
    timestamp: Date;
  };

  /** Status */
  status: 'open' | 'investigating' | 'contained' | 'resolved' | 'closed';

  /** Assigned to */
  assignedTo?: string;

  /** Resolution */
  resolution?: {
    description: string;
    rootCause: string;
    lessonsLearned: string[];
    timestamp: Date;
  };
}

// ============================================================================
// Monitoring and Metrics
// ============================================================================

/**
 * Security metrics
 */
export interface SecurityMetrics {
  /** Metric collection period */
  period: {
    start: Date;
    end: Date;
  };

  /** IDS metrics */
  ids: {
    threatsDetected: number;
    falsePositives: number;
    falseNegatives: number;
    detectionRate: number;
    averageDetectionLatencyMs: number;
  };

  /** Network metrics */
  network: {
    canMessagesProcessed: number;
    canMessagesBlocked: number;
    ethernetPacketsProcessed: number;
    ethernetPacketsBlocked: number;
    anomaliesDetected: number;
  };

  /** OTA metrics */
  ota: {
    updatesDeployed: number;
    updatesSuccessful: number;
    updatesFailed: number;
    averageUpdateDurationSeconds: number;
  };

  /** Incident metrics */
  incidents: {
    totalIncidents: number;
    criticalIncidents: number;
    averageResponseTimeMinutes: number;
    resolvedIncidents: number;
  };

  /** Compliance */
  compliance: {
    iso21434Score: number;
    unece_wp29Score: number;
    nhtsaScore: number;
  };
}

// ============================================================================
// Vehicle Security SDK
// ============================================================================

/**
 * Vehicle security configuration
 */
export interface VehicleSecurityConfig {
  /** Vehicle VIN */
  vin: VIN;

  /** Security profile */
  securityProfile: 'basic' | 'standard' | 'enhanced' | 'premium';

  /** Enable IDS */
  enableIDS: boolean;

  /** IDS configuration */
  idsConfig?: IDSConfiguration;

  /** Enable OTA security */
  enableOTASecurity: boolean;

  /** Enable network security */
  enableNetworkSecurity: boolean;

  /** Security logging level */
  logLevel: 'debug' | 'info' | 'warning' | 'error' | 'critical';

  /** Backend reporting */
  backendReporting: {
    enabled: boolean;
    endpoint?: string;
    apiKey?: string;
  };
}

// ============================================================================
// Physical Constants and Limits
// ============================================================================

/**
 * Security constants and limits
 */
export const SECURITY_CONSTANTS = {
  /** Maximum OBD requests per second */
  MAX_OBD_REQUESTS_PER_SECOND: 10,

  /** Maximum failed authentication attempts */
  MAX_AUTH_ATTEMPTS: 3,

  /** OBD lockout duration (seconds) */
  OBD_LOCKOUT_DURATION: 3600,

  /** CAN message maximum rate (messages/second) */
  MAX_CAN_MESSAGE_RATE: 10000,

  /** Session timeout (seconds) */
  SESSION_TIMEOUT: 3600,

  /** Certificate validity (days) */
  CERT_VALIDITY_DAYS: {
    enrollment: 3650,      // 10 years
    authorization: 1095,   // 3 years
    pseudonym: 7,          // 1 week
  },

  /** Key sizes (bits) */
  KEY_SIZES: {
    aes: [128, 256],
    rsa: [2048, 4096],
    ecc: [256, 384],
  },

  /** Signature algorithms */
  SIGNATURE_ALGORITHMS: [
    'ECDSA-P256',
    'ECDSA-P384',
    'RSA-2048',
    'RSA-4096',
  ] as const,

  /** Hash algorithms */
  HASH_ALGORITHMS: [
    'SHA-256',
    'SHA-384',
    'SHA-512',
  ] as const,
} as const;

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

/**
 * Event handler type
 */
export type EventHandler<T> = (event: T) => void | Promise<void>;

/**
 * Observable type
 */
export interface Observable<T> {
  subscribe(handler: EventHandler<T>): () => void;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-AUTO-023 error codes
 */
export enum SecurityErrorCode {
  INVALID_SIGNATURE = 'SEC001',
  AUTHENTICATION_FAILED = 'SEC002',
  AUTHORIZATION_FAILED = 'SEC003',
  ENCRYPTION_FAILED = 'SEC004',
  DECRYPTION_FAILED = 'SEC005',
  CERTIFICATE_INVALID = 'SEC006',
  THREAT_DETECTED = 'SEC007',
  IDS_FAILURE = 'SEC008',
  FIREWALL_BLOCKED = 'SEC009',
  RATE_LIMIT_EXCEEDED = 'SEC010',
  OTA_VERIFICATION_FAILED = 'SEC011',
  ROLLBACK_DETECTED = 'SEC012',
  HSM_ERROR = 'SEC013',
  NETWORK_SECURITY_FAILURE = 'SEC014',
  COMPLIANCE_VIOLATION = 'SEC015',
}

/**
 * Vehicle security error
 */
export class VehicleSecurityError extends Error {
  constructor(
    public code: SecurityErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'VehicleSecurityError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  VIN,
  SecuritySeverity,
  CAL,
  RiskLevel,
  AttackFeasibility,

  // ECU
  ECU,
  SecureBootResult,
  HSMOperation,
  CodeSignature,

  // Network
  NetworkType,
  SecureCANMessage,
  CANFirewallRule,
  NetworkSecurityStatus,
  EthernetSecurityConfig,

  // External
  V2XMessageType,
  V2XSecuredMessage,
  OBDSecurityAccess,
  TelematicsSecurityConfig,

  // OTA
  OTAUpdatePackage,
  OTAValidationResult,
  OTAUpdateStatus,

  // IDS
  IDSDetectionMethod,
  SecurityThreat,
  IDSResponseAction,
  IDSAlert,
  IDSConfiguration,

  // Assessment
  Vulnerability,
  SecurityWeakness,
  SecurityRecommendation,
  ComplianceStatus,
  SecurityAssessment,

  // TARA
  Asset,
  ThreatScenario,
  TARAResult,

  // Incident
  SecurityIncident,

  // Monitoring
  SecurityMetrics,

  // Configuration
  VehicleSecurityConfig,

  // Utility
  EventHandler,
  Observable,
};

export { SECURITY_CONSTANTS, SecurityErrorCode, VehicleSecurityError };
