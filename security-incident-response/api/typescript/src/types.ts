/**
 * WIA Security Incident Response - TypeScript Type Definitions
 * Based on NIST 800-61 and SANS Incident Response Framework
 *
 * @module @wia/security-incident-response/types
 * @version 1.0.0
 */

/**
 * Incident Severity Levels (NIST/SANS)
 */
export enum IncidentSeverity {
  CRITICAL = 'critical',      // Immediate threat to critical systems
  HIGH = 'high',              // Significant impact, urgent response needed
  MEDIUM = 'medium',          // Moderate impact, timely response needed
  LOW = 'low',                // Minimal impact, standard response
  INFORMATIONAL = 'informational' // For awareness only
}

/**
 * Incident Types/Categories
 */
export enum IncidentType {
  MALWARE = 'malware',
  PHISHING = 'phishing',
  DATA_BREACH = 'data_breach',
  DOS_DDOS = 'dos_ddos',
  UNAUTHORIZED_ACCESS = 'unauthorized_access',
  INSIDER_THREAT = 'insider_threat',
  RANSOMWARE = 'ransomware',
  APT = 'apt',                // Advanced Persistent Threat
  WEB_ATTACK = 'web_attack',
  SQL_INJECTION = 'sql_injection',
  XSS = 'xss',
  MISCONFIGURATION = 'misconfiguration',
  SOCIAL_ENGINEERING = 'social_engineering',
  PHYSICAL_BREACH = 'physical_breach',
  SUPPLY_CHAIN = 'supply_chain',
  CRYPTOJACKING = 'cryptojacking',
  OTHER = 'other'
}

/**
 * Incident Lifecycle States (NIST Framework)
 */
export enum IncidentState {
  PREPARATION = 'preparation',           // Pre-incident readiness
  DETECTED = 'detected',                 // Initial detection
  TRIAGED = 'triaged',                   // Initial assessment complete
  ANALYZING = 'analyzing',               // Deep analysis in progress
  CONTAINED = 'contained',               // Threat contained
  ERADICATING = 'eradicating',          // Removing threat
  RECOVERING = 'recovering',             // Restoring systems
  POST_INCIDENT = 'post_incident',       // Lessons learned phase
  CLOSED = 'closed'                      // Incident closed
}

/**
 * Incident Response Team Roles
 */
export enum ResponseRole {
  INCIDENT_MANAGER = 'incident_manager',
  SECURITY_ANALYST = 'security_analyst',
  FORENSIC_INVESTIGATOR = 'forensic_investigator',
  SYSTEM_ADMINISTRATOR = 'system_administrator',
  NETWORK_ENGINEER = 'network_engineer',
  LEGAL_COUNSEL = 'legal_counsel',
  PR_COMMUNICATIONS = 'pr_communications',
  EXECUTIVE_SPONSOR = 'executive_sponsor',
  TECHNICAL_LEAD = 'technical_lead'
}

/**
 * Containment Strategy Types
 */
export enum ContainmentStrategy {
  ISOLATION = 'isolation',               // Network isolation
  SHUTDOWN = 'shutdown',                 // System shutdown
  CREDENTIAL_RESET = 'credential_reset', // Password/key rotation
  FIREWALL_RULE = 'firewall_rule',      // Firewall modification
  TRAFFIC_BLOCK = 'traffic_block',       // Block malicious traffic
  ACCOUNT_DISABLE = 'account_disable',   // Disable compromised accounts
  PATCH_APPLY = 'patch_apply',           // Emergency patching
  SEGMENTATION = 'segmentation'          // Network segmentation
}

/**
 * Evidence Type for Forensics
 */
export enum EvidenceType {
  NETWORK_LOGS = 'network_logs',
  SYSTEM_LOGS = 'system_logs',
  APPLICATION_LOGS = 'application_logs',
  MEMORY_DUMP = 'memory_dump',
  DISK_IMAGE = 'disk_image',
  NETWORK_CAPTURE = 'network_capture',
  FILE_SAMPLE = 'file_sample',
  SCREENSHOT = 'screenshot',
  REGISTRY_HIVE = 'registry_hive',
  DATABASE_SNAPSHOT = 'database_snapshot'
}

/**
 * Communication Priority Levels
 */
export enum CommunicationPriority {
  IMMEDIATE = 'immediate',     // Within minutes
  URGENT = 'urgent',           // Within hours
  NORMAL = 'normal',           // Within 24 hours
  LOW = 'low'                  // As needed
}

/**
 * Core Incident Interface
 */
export interface SecurityIncident {
  id: string;
  type: IncidentType;
  severity: IncidentSeverity;
  state: IncidentState;
  title: string;
  description: string;
  detectedAt: Date;
  reportedBy: string;
  assignedTo?: ResponseRole[];
  affectedSystems: string[];
  affectedData?: string[];
  indicators?: IndicatorOfCompromise[];
  timeline: IncidentEvent[];
  metadata?: Record<string, any>;
}

/**
 * Indicator of Compromise (IOC)
 */
export interface IndicatorOfCompromise {
  type: 'ip' | 'domain' | 'url' | 'hash' | 'email' | 'file_path';
  value: string;
  source?: string;
  confidence: 'high' | 'medium' | 'low';
  firstSeen: Date;
  lastSeen?: Date;
}

/**
 * Incident Timeline Event
 */
export interface IncidentEvent {
  timestamp: Date;
  actor: string;
  action: string;
  description: string;
  state?: IncidentState;
  metadata?: Record<string, any>;
}

/**
 * Response Team Member
 */
export interface ResponseTeamMember {
  id: string;
  name: string;
  role: ResponseRole;
  contact: ContactInfo;
  availability: 'available' | 'busy' | 'offline';
  assignedIncidents?: string[];
}

/**
 * Contact Information
 */
export interface ContactInfo {
  email: string;
  phone?: string;
  slack?: string;
  pagerDuty?: string;
}

/**
 * Containment Action
 */
export interface ContainmentAction {
  id: string;
  incidentId: string;
  strategy: ContainmentStrategy;
  description: string;
  implementedBy: string;
  implementedAt: Date;
  affectedSystems: string[];
  success: boolean;
  notes?: string;
}

/**
 * Eradication Action
 */
export interface EradicationAction {
  id: string;
  incidentId: string;
  action: 'remove_malware' | 'patch_system' | 'rebuild_system' | 'rotate_credentials' | 'other';
  description: string;
  implementedBy: string;
  implementedAt: Date;
  verified: boolean;
  verifiedBy?: string;
  verifiedAt?: Date;
}

/**
 * Forensic Evidence
 */
export interface ForensicEvidence {
  id: string;
  incidentId: string;
  type: EvidenceType;
  description: string;
  collectedBy: string;
  collectedAt: Date;
  chainOfCustody: ChainOfCustodyEntry[];
  location: string;
  hash?: string;
  size?: number;
  metadata?: Record<string, any>;
}

/**
 * Chain of Custody Entry
 */
export interface ChainOfCustodyEntry {
  timestamp: Date;
  custodian: string;
  action: 'collected' | 'transferred' | 'analyzed' | 'stored' | 'destroyed';
  location: string;
  notes?: string;
}

/**
 * Communication Record
 */
export interface CommunicationRecord {
  id: string;
  incidentId: string;
  priority: CommunicationPriority;
  recipients: string[];
  channel: 'email' | 'slack' | 'phone' | 'sms' | 'meeting';
  subject: string;
  message: string;
  sentBy: string;
  sentAt: Date;
  escalation?: boolean;
}

/**
 * Post-Incident Review
 */
export interface PostIncidentReview {
  incidentId: string;
  conductedAt: Date;
  participants: string[];
  timeline: IncidentEvent[];
  rootCause: string;
  lessonsLearned: string[];
  improvements: ImprovementRecommendation[];
  successMetrics: {
    detectionTime?: number;    // Minutes to detect
    responseTime?: number;     // Minutes to respond
    containmentTime?: number;  // Minutes to contain
    recoveryTime?: number;     // Minutes to recover
  };
  costImpact?: {
    financial?: number;
    reputation?: string;
    operational?: string;
  };
}

/**
 * Improvement Recommendation
 */
export interface ImprovementRecommendation {
  category: 'process' | 'technology' | 'people' | 'documentation';
  priority: 'high' | 'medium' | 'low';
  description: string;
  owner?: string;
  dueDate?: Date;
  status: 'proposed' | 'approved' | 'in_progress' | 'completed' | 'rejected';
}

/**
 * Incident Response Configuration
 */
export interface IncidentResponseConfig {
  organizationName: string;
  incidentResponseTeam: ResponseTeamMember[];
  escalationMatrix: EscalationRule[];
  retentionPolicy: {
    incidentData: number;    // Days
    forensicEvidence: number; // Days
    logs: number;            // Days
  };
  automationEnabled: boolean;
  integrationsEnabled: boolean;
}

/**
 * Escalation Rule
 */
export interface EscalationRule {
  severity: IncidentSeverity;
  timeThreshold: number; // Minutes
  escalateTo: ResponseRole[];
  notificationChannels: ('email' | 'sms' | 'phone' | 'slack')[];
}

/**
 * Incident Statistics
 */
export interface IncidentStatistics {
  period: {
    start: Date;
    end: Date;
  };
  totalIncidents: number;
  bySeverity: Record<IncidentSeverity, number>;
  byType: Record<IncidentType, number>;
  byState: Record<IncidentState, number>;
  averageResponseTime: number; // Minutes
  averageContainmentTime: number; // Minutes
  averageRecoveryTime: number; // Minutes
}
