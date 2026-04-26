/**
 * WIA-DEF-005: Cyber Defense - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Defense & Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Defense Types
// ============================================================================

/**
 * Threat severity levels
 */
export type ThreatSeverity = 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW' | 'INFO';

/**
 * Incident status
 */
export type IncidentStatus =
  | 'new'
  | 'investigating'
  | 'contained'
  | 'eradicated'
  | 'recovered'
  | 'closed';

/**
 * Defense layer types
 */
export type DefenseLayer =
  | 'application'
  | 'data'
  | 'endpoint'
  | 'network'
  | 'identity'
  | 'infrastructure'
  | 'physical';

/**
 * Asset criticality levels
 */
export type AssetCriticality = 'critical' | 'high' | 'medium' | 'low';

// ============================================================================
// Threat Detection
// ============================================================================

/**
 * Security threat information
 */
export interface Threat {
  /** Unique threat identifier */
  id: string;

  /** Threat type */
  type:
    | 'malware'
    | 'exploit'
    | 'phishing'
    | 'dos'
    | 'ddos'
    | 'data_exfiltration'
    | 'privilege_escalation'
    | 'reconnaissance'
    | 'lateral_movement'
    | 'persistence'
    | 'command_and_control'
    | 'credential_access'
    | 'other';

  /** Threat severity */
  severity: ThreatSeverity;

  /** Severity score (0-100) */
  severityScore: number;

  /** Confidence level (0-1) */
  confidence: number;

  /** Detection timestamp */
  detectedAt: Date;

  /** Source of detection */
  detectionSource: string;

  /** Affected asset */
  affectedAsset: string;

  /** Asset criticality */
  assetCriticality: AssetCriticality;

  /** Threat description */
  description: string;

  /** Indicators of Compromise */
  iocs: IOC[];

  /** MITRE ATT&CK techniques */
  mitreAttack?: string[];

  /** Recommended actions */
  recommendedActions: string[];

  /** Auto-response taken */
  autoResponseTaken?: boolean;

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Indicator of Compromise (IOC)
 */
export interface IOC {
  /** IOC type */
  type: 'ip' | 'domain' | 'url' | 'hash' | 'email' | 'registry_key' | 'mutex';

  /** IOC value */
  value: string;

  /** Confidence score (0-1) */
  confidence: number;

  /** First seen timestamp */
  firstSeen: Date;

  /** Last seen timestamp */
  lastSeen: Date;

  /** Source of IOC */
  source: string;

  /** Additional context */
  context?: string;
}

/**
 * Threat detection parameters
 */
export interface ThreatDetectionParams {
  /** Data source to monitor */
  source: 'network' | 'endpoint' | 'application' | 'logs' | 'all';

  /** Time window in seconds */
  timeWindow?: number;

  /** Minimum severity threshold */
  severityThreshold?: ThreatSeverity;

  /** Minimum confidence threshold (0-1) */
  confidenceThreshold?: number;

  /** Filter by asset criticality */
  assetCriticality?: AssetCriticality[];

  /** Include historical data */
  includeHistorical?: boolean;
}

/**
 * Threat detection result
 */
export interface ThreatDetectionResult {
  /** Detected threats */
  threats: Threat[];

  /** Total count */
  totalCount: number;

  /** Detection time range */
  timeRange: {
    start: Date;
    end: Date;
  };

  /** Summary statistics */
  summary: {
    bySeverity: Record<ThreatSeverity, number>;
    byType: Record<string, number>;
    byAsset: Record<string, number>;
  };
}

// ============================================================================
// Incident Response
// ============================================================================

/**
 * Security incident
 */
export interface Incident {
  /** Unique incident identifier */
  id: string;

  /** Incident title */
  title: string;

  /** Incident description */
  description: string;

  /** Incident severity */
  severity: ThreatSeverity;

  /** Current status */
  status: IncidentStatus;

  /** Related threats */
  threats: string[];

  /** Affected assets */
  affectedAssets: string[];

  /** Assigned analysts */
  assignedTo: string[];

  /** Created timestamp */
  createdAt: Date;

  /** Updated timestamp */
  updatedAt: Date;

  /** Incident timeline */
  timeline: IncidentTimelineEntry[];

  /** Estimated impact */
  impact?: {
    dataLoss?: string;
    downtime?: number;
    affectedUsers?: number;
    estimatedCost?: number;
  };

  /** Response SLA */
  responseSLA?: {
    acknowledgeBy: Date;
    respondBy: Date;
    containBy: Date;
    resolveBy: Date;
  };

  /** Playbook used */
  playbook?: string;

  /** Tags */
  tags?: string[];

  /** Additional metadata */
  metadata?: Record<string, unknown>;
}

/**
 * Incident timeline entry
 */
export interface IncidentTimelineEntry {
  /** Entry timestamp */
  timestamp: Date;

  /** Entry type */
  type: 'detection' | 'analysis' | 'containment' | 'eradication' | 'recovery' | 'note';

  /** Entry description */
  description: string;

  /** Analyst who made the entry */
  analyst?: string;

  /** Actions taken */
  actions?: string[];
}

/**
 * Incident analysis parameters
 */
export interface IncidentAnalysisParams {
  /** Incident ID to analyze */
  incidentId: string;

  /** Collect forensic data */
  collectForensics?: boolean;

  /** Auto-contain the incident */
  autoContain?: boolean;

  /** Notification recipients */
  notifyRecipients?: string[];

  /** Playbook to execute */
  playbook?: string;
}

/**
 * Incident analysis result
 */
export interface IncidentAnalysisResult {
  /** Incident details */
  incident: Incident;

  /** Root cause analysis */
  rootCause?: string;

  /** Attack vector */
  attackVector?: string;

  /** Forensic evidence */
  forensicEvidence?: ForensicEvidence[];

  /** Containment status */
  containmentStatus: {
    isContained: boolean;
    actions: string[];
    timestamp?: Date;
  };

  /** Recommendations */
  recommendations: string[];
}

/**
 * Forensic evidence
 */
export interface ForensicEvidence {
  /** Evidence type */
  type: 'log' | 'file' | 'memory' | 'network' | 'registry' | 'artifact';

  /** Evidence source */
  source: string;

  /** Collection timestamp */
  collectedAt: Date;

  /** Evidence hash (SHA256) */
  hash?: string;

  /** Evidence location */
  location: string;

  /** Evidence description */
  description: string;

  /** Evidence data (Base64 encoded) */
  data?: string;
}

// ============================================================================
// Vulnerability Management
// ============================================================================

/**
 * Vulnerability information
 */
export interface Vulnerability {
  /** Vulnerability ID (CVE, etc.) */
  id: string;

  /** Vulnerability title */
  title: string;

  /** Description */
  description: string;

  /** CVSS score (0-10) */
  cvssScore: number;

  /** Severity rating */
  severity: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';

  /** Affected systems */
  affectedSystems: string[];

  /** Exploit available */
  exploitAvailable: boolean;

  /** Patch available */
  patchAvailable: boolean;

  /** Remediation steps */
  remediation?: string[];

  /** Discovered date */
  discoveredAt: Date;

  /** Remediation deadline */
  remediationDeadline?: Date;

  /** References */
  references?: string[];
}

/**
 * Vulnerability assessment parameters
 */
export interface VulnerabilityAssessmentParams {
  /** Target system(s) */
  target: string | string[];

  /** Scan type */
  scanType: 'quick' | 'comprehensive' | 'compliance';

  /** Compliance frameworks to check */
  complianceFramework?: ('NIST' | 'ISO27001' | 'PCI-DSS' | 'HIPAA')[];

  /** Include network scan */
  networkScan?: boolean;

  /** Include web application scan */
  webAppScan?: boolean;

  /** Authentication credentials */
  credentials?: {
    username: string;
    password: string;
  };
}

/**
 * Vulnerability assessment result
 */
export interface VulnerabilityAssessmentResult {
  /** Assessment ID */
  id: string;

  /** Target system(s) */
  target: string[];

  /** Scan timestamp */
  scannedAt: Date;

  /** Vulnerabilities found */
  vulnerabilities: Vulnerability[];

  /** Summary statistics */
  summary: {
    total: number;
    critical: number;
    high: number;
    medium: number;
    low: number;
  };

  /** Compliance status */
  compliance?: {
    framework: string;
    compliant: boolean;
    failedControls: string[];
  }[];

  /** Remediation priority */
  remediationPriority: Vulnerability[];
}

// ============================================================================
// Network Security
// ============================================================================

/**
 * Network security configuration
 */
export interface NetworkSecurityConfig {
  /** Configuration ID */
  id: string;

  /** Network segment */
  segment: string;

  /** Firewall rules */
  firewallRules: FirewallRule[];

  /** IDS/IPS enabled */
  idsIpsEnabled: boolean;

  /** Network segmentation */
  segmentation: {
    vlanId?: number;
    subnet: string;
    isolation: 'none' | 'partial' | 'full';
  };

  /** Allowed protocols */
  allowedProtocols: string[];

  /** Geo-blocking enabled */
  geoBlockingEnabled: boolean;

  /** Blocked countries */
  blockedCountries?: string[];
}

/**
 * Firewall rule
 */
export interface FirewallRule {
  /** Rule ID */
  id: string;

  /** Rule name */
  name: string;

  /** Priority (lower = higher priority) */
  priority: number;

  /** Source address */
  source: string;

  /** Destination address */
  destination: string;

  /** Port */
  port: number | string;

  /** Protocol */
  protocol: 'TCP' | 'UDP' | 'ICMP' | 'ANY';

  /** Action */
  action: 'ALLOW' | 'DENY' | 'DROP';

  /** Logging enabled */
  logging: boolean;

  /** Rule enabled */
  enabled: boolean;

  /** Description */
  description?: string;
}

/**
 * Network hardening parameters
 */
export interface NetworkHardeningParams {
  /** Network profile */
  profile: 'critical-infrastructure' | 'enterprise' | 'dmz' | 'internal' | 'custom';

  /** Apply security baselines */
  applyBaselines?: boolean;

  /** Enable IDS/IPS */
  enableIdsIps?: boolean;

  /** Enable geo-blocking */
  enableGeoBlocking?: boolean;

  /** Custom firewall rules */
  customRules?: FirewallRule[];

  /** Dry-run mode (preview changes) */
  dryRun?: boolean;
}

/**
 * Network hardening result
 */
export interface NetworkHardeningResult {
  /** Applied successfully */
  success: boolean;

  /** Changes made */
  changes: {
    firewallRules: { added: number; modified: number; removed: number };
    networkConfig: string[];
    securityBaselines: string[];
  };

  /** Validation results */
  validation: {
    passed: boolean;
    issues: string[];
  };

  /** Rollback plan */
  rollbackPlan?: string;
}

// ============================================================================
// SOC Operations
// ============================================================================

/**
 * SOC alert
 */
export interface SOCAlert {
  /** Alert ID */
  id: string;

  /** Alert title */
  title: string;

  /** Alert severity */
  severity: ThreatSeverity;

  /** Alert status */
  status: 'new' | 'acknowledged' | 'investigating' | 'resolved' | 'false_positive';

  /** Source system */
  source: string;

  /** Triggered timestamp */
  triggeredAt: Date;

  /** Acknowledged timestamp */
  acknowledgedAt?: Date;

  /** Assigned analyst */
  assignedTo?: string;

  /** Related threat */
  threat?: Threat;

  /** Enrichment data */
  enrichment?: {
    geoip?: { country: string; city: string; latitude: number; longitude: number };
    assetInfo?: { hostname: string; ip: string; os: string; criticality: AssetCriticality };
    threatIntel?: { reputation: string; malicious: boolean; confidence: number };
  };

  /** Notes */
  notes?: string[];
}

/**
 * SOC monitoring parameters
 */
export interface SOCMonitoringParams {
  /** Real-time monitoring */
  realTime?: boolean;

  /** Alert filters */
  filters?: {
    severity?: ThreatSeverity[];
    source?: string[];
    status?: string[];
  };

  /** Dashboard refresh interval (seconds) */
  refreshInterval?: number;

  /** Enable notifications */
  enableNotifications?: boolean;
}

/**
 * SOC dashboard metrics
 */
export interface SOCDashboardMetrics {
  /** Timestamp */
  timestamp: Date;

  /** Active alerts */
  activeAlerts: {
    total: number;
    critical: number;
    high: number;
    medium: number;
    low: number;
  };

  /** Open incidents */
  openIncidents: {
    total: number;
    bySeverity: Record<ThreatSeverity, number>;
  };

  /** Response metrics */
  responseMetrics: {
    meanTimeToAcknowledge: number; // seconds
    meanTimeToDetect: number; // seconds
    meanTimeToRespond: number; // seconds
    meanTimeToContain: number; // seconds
  };

  /** Detection coverage */
  detectionCoverage: {
    mitreAttackCoverage: number; // percentage
    logSources: number;
    activeRules: number;
  };

  /** Analyst performance */
  analystPerformance?: {
    alertsHandled: number;
    incidentsResolved: number;
    falsePositiveRate: number;
    avgResolutionTime: number;
  };
}

// ============================================================================
// Defense Posture
// ============================================================================

/**
 * Defense layer score
 */
export interface DefenseLayerScore {
  /** Layer type */
  layer: DefenseLayer;

  /** Score (0-100) */
  score: number;

  /** Weight factor */
  weight: number;

  /** Status */
  status: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Issues found */
  issues?: string[];

  /** Recommendations */
  recommendations?: string[];
}

/**
 * Overall defense posture
 */
export interface DefensePosture {
  /** Overall score (0-100) */
  overallScore: number;

  /** Status */
  status: 'excellent' | 'good' | 'fair' | 'poor' | 'critical';

  /** Layer scores */
  layers: DefenseLayerScore[];

  /** Assessment timestamp */
  assessedAt: Date;

  /** Key strengths */
  strengths: string[];

  /** Key weaknesses */
  weaknesses: string[];

  /** Priority actions */
  priorityActions: string[];

  /** Compliance status */
  compliance?: {
    framework: string;
    score: number;
    compliant: boolean;
  }[];
}

// ============================================================================
// Configuration
// ============================================================================

/**
 * Cyber Defense SDK configuration
 */
export interface CyberDefenseConfig {
  /** API key for authentication */
  apiKey?: string;

  /** API endpoint URL */
  apiEndpoint?: string;

  /** Threat intelligence feeds */
  threatIntelFeeds?: ('misp' | 'otx' | 'crowdstrike' | 'recorded-future')[];

  /** SIEM integration */
  siemIntegration?: 'splunk' | 'elastic' | 'qradar' | 'sentinel' | 'sumo-logic';

  /** SOAR integration */
  soarIntegration?: 'phantom' | 'demisto' | 'swimlane' | 'siemplify';

  /** Auto-response enabled */
  autoResponseEnabled?: boolean;

  /** Alert retention days */
  alertRetentionDays?: number;

  /** Log retention days */
  logRetentionDays?: number;

  /** Enable logging */
  enableLogging?: boolean;

  /** Custom configuration */
  custom?: Record<string, unknown>;
}

// ============================================================================
// Constants
// ============================================================================

/**
 * Defense severity thresholds
 */
export const SEVERITY_THRESHOLDS = {
  CRITICAL: 90,
  HIGH: 70,
  MEDIUM: 40,
  LOW: 20,
  INFO: 0,
} as const;

/**
 * Defense layer weights
 */
export const LAYER_WEIGHTS: Record<DefenseLayer, number> = {
  application: 0.15,
  data: 0.20,
  endpoint: 0.15,
  network: 0.15,
  identity: 0.20,
  infrastructure: 0.10,
  physical: 0.05,
} as const;

/**
 * Response SLA (in minutes)
 */
export const RESPONSE_SLA = {
  CRITICAL: 15,
  HIGH: 60,
  MEDIUM: 240,
  LOW: 1440,
} as const;

/**
 * MITRE ATT&CK tactics
 */
export const MITRE_TACTICS = [
  'TA0001', // Initial Access
  'TA0002', // Execution
  'TA0003', // Persistence
  'TA0004', // Privilege Escalation
  'TA0005', // Defense Evasion
  'TA0006', // Credential Access
  'TA0007', // Discovery
  'TA0008', // Lateral Movement
  'TA0009', // Collection
  'TA0010', // Exfiltration
  'TA0011', // Command and Control
  'TA0040', // Impact
  'TA0042', // Resource Development
  'TA0043', // Reconnaissance
] as const;

// ============================================================================
// Error Types
// ============================================================================

/**
 * WIA-DEF-005 error codes
 */
export enum DefenseErrorCode {
  THREAT_DETECTION_FAILED = 'DEF001',
  INCIDENT_ANALYSIS_FAILED = 'DEF002',
  VULNERABILITY_SCAN_FAILED = 'DEF003',
  NETWORK_HARDENING_FAILED = 'DEF004',
  AUTHORIZATION_FAILED = 'DEF005',
  INVALID_PARAMETERS = 'DEF006',
  RESOURCE_NOT_FOUND = 'DEF007',
  SOC_CONNECTION_FAILED = 'DEF008',
  SIEM_INTEGRATION_ERROR = 'DEF009',
  AUTO_RESPONSE_FAILED = 'DEF010',
}

/**
 * Cyber defense error
 */
export class CyberDefenseError extends Error {
  constructor(
    public code: DefenseErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CyberDefenseError';
  }
}

// ============================================================================
// Export All
// ============================================================================

export type {
  // Core types
  Threat,
  IOC,
  ThreatDetectionParams,
  ThreatDetectionResult,

  // Incident response
  Incident,
  IncidentTimelineEntry,
  IncidentAnalysisParams,
  IncidentAnalysisResult,
  ForensicEvidence,

  // Vulnerability management
  Vulnerability,
  VulnerabilityAssessmentParams,
  VulnerabilityAssessmentResult,

  // Network security
  NetworkSecurityConfig,
  FirewallRule,
  NetworkHardeningParams,
  NetworkHardeningResult,

  // SOC operations
  SOCAlert,
  SOCMonitoringParams,
  SOCDashboardMetrics,

  // Defense posture
  DefenseLayerScore,
  DefensePosture,

  // Configuration
  CyberDefenseConfig,
};

export {
  SEVERITY_THRESHOLDS,
  LAYER_WEIGHTS,
  RESPONSE_SLA,
  MITRE_TACTICS,
  DefenseErrorCode,
  CyberDefenseError,
};
