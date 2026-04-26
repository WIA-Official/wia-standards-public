/**
 * WIA-COMM-015: Network Security - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Network Security Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Network Security Types
// ============================================================================

/**
 * Security action types
 */
export type SecurityAction = 'allow' | 'deny' | 'reject' | 'drop' | 'inspect' | 'quarantine' | 'alert' | 'log';

/**
 * Severity levels for security events
 */
export type SeverityLevel = 'critical' | 'high' | 'medium' | 'low' | 'info';

/**
 * Network protocol types
 */
export type NetworkProtocol = 'tcp' | 'udp' | 'icmp' | 'ip' | 'http' | 'https' | 'dns' | 'ssh' | 'ftp' | 'smtp';

/**
 * Deployment modes for security devices
 */
export type DeploymentMode = 'inline' | 'passive' | 'hybrid';

// ============================================================================
// Next-Generation Firewall (NGFW)
// ============================================================================

/**
 * Firewall zone configuration
 */
export interface FirewallZone {
  /** Zone name */
  name: string;

  /** Zone description */
  description?: string;

  /** Network interfaces in this zone */
  interfaces: string[];

  /** IP address ranges */
  networks?: string[];

  /** Zone type */
  type: 'trusted' | 'untrusted' | 'dmz' | 'management' | 'guest';
}

/**
 * Source/destination specification for firewall rules
 */
export interface FirewallEndpoint {
  /** Zone names */
  zones?: string[];

  /** IP addresses or ranges (CIDR) */
  addresses?: string[];

  /** User identities */
  users?: string[];

  /** Geographic locations (ISO country codes) */
  geolocations?: string[];
}

/**
 * Deep packet inspection configuration
 */
export interface DPIConfiguration {
  /** Enable deep packet inspection */
  dpi?: boolean;

  /** Enable intrusion prevention */
  ips?: boolean;

  /** Enable antivirus scanning */
  antivirus?: boolean;

  /** Enable SSL/TLS inspection */
  tlsInspection?: boolean;

  /** Enable URL filtering */
  urlFiltering?: boolean;

  /** Enable data loss prevention */
  dataLossPrevention?: boolean;

  /** Enable malware sandboxing */
  sandboxing?: boolean;
}

/**
 * Logging configuration
 */
export interface LoggingConfiguration {
  /** Enable logging */
  enabled: boolean;

  /** Log level */
  level: 'none' | 'summary' | 'detailed';

  /** Log destinations */
  destination: string[];

  /** Include packet capture */
  packetCapture?: boolean;
}

/**
 * Schedule configuration for time-based policies
 */
export interface ScheduleConfiguration {
  /** Start time (HH:MM format) */
  startTime?: string;

  /** End time (HH:MM format) */
  endTime?: string;

  /** Days of week (0=Sunday, 6=Saturday) */
  daysOfWeek?: number[];

  /** Timezone */
  timezone?: string;
}

/**
 * Firewall policy rule
 */
export interface FirewallPolicy {
  /** Unique policy ID */
  id: string;

  /** Policy name */
  name: string;

  /** Policy description */
  description?: string;

  /** Enable/disable policy */
  enabled: boolean;

  /** Policy priority (lower = higher priority) */
  priority: number;

  /** Source specification */
  source: FirewallEndpoint;

  /** Destination specification */
  destination: {
    zones?: string[];
    addresses?: string[];
    services?: string[];
  };

  /** Application names */
  applications?: string[];

  /** Application categories */
  categories?: string[];

  /** Security action */
  action: SecurityAction;

  /** Inspection configuration */
  inspection?: DPIConfiguration;

  /** Logging configuration */
  logging: LoggingConfiguration;

  /** Schedule */
  schedule?: ScheduleConfiguration;

  /** Custom metadata */
  metadata?: Record<string, any>;
}

/**
 * NGFW configuration
 */
export interface NGFWConfiguration {
  /** Firewall name */
  name: string;

  /** Deployment mode */
  mode: DeploymentMode;

  /** Network interfaces */
  interfaces: {
    wan?: string;
    lan?: string;
    dmz?: string;
    [key: string]: string | undefined;
  };

  /** Firewall zones */
  zones?: FirewallZone[];

  /** Firewall policies */
  policies: FirewallPolicy[];

  /** Threat intelligence feeds */
  threatIntelligence?: {
    feeds: string[];
    updateInterval: number; // seconds
  };

  /** High availability configuration */
  ha?: {
    enabled: boolean;
    mode: 'active-passive' | 'active-active';
    peer?: string;
  };
}

// ============================================================================
// Intrusion Detection/Prevention System (IDS/IPS)
// ============================================================================

/**
 * IDS/IPS detection method
 */
export type DetectionMethod = 'signature' | 'anomaly' | 'protocol-analysis' | 'heuristic' | 'machine-learning';

/**
 * IDS engine type
 */
export type IDSEngine = 'suricata' | 'snort' | 'zeek' | 'proprietary';

/**
 * IDS/IPS rule
 */
export interface IDSRule {
  /** Rule ID */
  sid: number;

  /** Rule revision */
  rev: number;

  /** Rule message */
  msg: string;

  /** Rule action */
  action: 'alert' | 'log' | 'pass' | 'drop' | 'reject';

  /** Protocol */
  protocol: NetworkProtocol;

  /** Source specification */
  source: {
    address: string;
    port: string | number;
  };

  /** Destination specification */
  destination: {
    address: string;
    port: string | number;
  };

  /** Rule options */
  options?: Record<string, any>;

  /** Classification type */
  classtype?: string;

  /** Priority */
  priority?: number;
}

/**
 * IDS action configuration based on severity
 */
export interface IDSActionConfig {
  /** Action for critical severity */
  critical: SecurityAction;

  /** Action for high severity */
  high: SecurityAction;

  /** Action for medium severity */
  medium: SecurityAction;

  /** Action for low severity */
  low: SecurityAction;
}

/**
 * IDS/IPS configuration
 */
export interface IDSConfiguration {
  /** IDS name */
  name: string;

  /** Operating mode */
  mode: 'ids' | 'ips';

  /** Detection engine */
  engine: IDSEngine;

  /** Ruleset names */
  rulesets: string[];

  /** Custom rules */
  customRules?: string[];

  /** Detection methods */
  detectionMethods?: DetectionMethod[];

  /** Action configuration */
  actions: IDSActionConfig;

  /** Interfaces to monitor */
  interfaces?: string[];

  /** Performance tuning */
  performance?: {
    workers?: number;
    bufferSize?: number;
    maxPendingPackets?: number;
  };
}

/**
 * IDS/IPS alert
 */
export interface IDSAlert {
  /** Alert ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Rule that triggered */
  rule: {
    sid: number;
    msg: string;
    classtype?: string;
  };

  /** Severity */
  severity: SeverityLevel;

  /** Source information */
  source: {
    ip: string;
    port: number;
    geo?: string;
  };

  /** Destination information */
  destination: {
    ip: string;
    port: number;
  };

  /** Protocol */
  protocol: NetworkProtocol;

  /** Packet payload (if captured) */
  payload?: string;

  /** Action taken */
  action: SecurityAction;
}

// ============================================================================
// Zero Trust Network Access (ZTNA)
// ============================================================================

/**
 * Identity provider type
 */
export type IdentityProviderType = 'saml' | 'oauth2' | 'oidc' | 'ldap' | 'active-directory';

/**
 * Multi-factor authentication method
 */
export type MFAMethod = 'totp' | 'push' | 'sms' | 'email' | 'hardware-token' | 'biometric';

/**
 * Device compliance status
 */
export type ComplianceStatus = 'compliant' | 'non-compliant' | 'unknown';

/**
 * Identity provider configuration
 */
export interface IdentityProvider {
  /** Provider type */
  type: IdentityProviderType;

  /** Provider endpoint URL */
  endpoint: string;

  /** Client ID */
  clientId?: string;

  /** Client secret */
  clientSecret?: string;

  /** Certificate for SAML */
  certificate?: string;
}

/**
 * Device posture requirements
 */
export interface PostureRequirements {
  /** OS version requirements */
  osVersion?: {
    windows?: string;
    macos?: string;
    linux?: string;
    ios?: string;
    android?: string;
  };

  /** Antivirus requirements */
  antivirus?: {
    required: boolean;
    approved?: string[];
    updateWithin?: string; // e.g., "24h"
  };

  /** Firewall requirements */
  firewall?: {
    required: boolean;
    enabled: boolean;
  };

  /** Disk encryption requirements */
  encryption?: {
    required: boolean;
    methods?: string[];
  };

  /** Patch requirements */
  patches?: {
    osUpdates: 'current' | 'recent';
    criticalWithin?: string; // e.g., "7d"
  };

  /** MDM enrollment */
  mdm?: {
    required: boolean;
    compliant: boolean;
  };
}

/**
 * Risk scoring factors
 */
export interface RiskScoring {
  /** Maximum acceptable risk score */
  maxScore: number;

  /** Risk factors */
  factors: Array<{
    name: string;
    score: number;
    action: SecurityAction;
  }>;
}

/**
 * ZTNA access conditions
 */
export interface ZTNAConditions {
  /** Require device compliance */
  deviceCompliance?: boolean;

  /** Require MFA */
  mfa?: boolean;

  /** MFA methods */
  mfaMethods?: MFAMethod[];

  /** Allowed networks */
  networks?: string[];

  /** Allowed locations (geo) */
  locations?: string[];

  /** Time window (HH:MM-HH:MM) */
  timeWindow?: string;

  /** Risk score threshold */
  riskScore?: {
    max: number;
  };

  /** Required approval */
  approval?: string;
}

/**
 * ZTNA policy
 */
export interface ZTNAPolicy {
  /** Policy name */
  name: string;

  /** User groups */
  users: string[];

  /** Resource identifiers */
  resources: string[];

  /** Access conditions */
  conditions: ZTNAConditions;

  /** Access decision */
  access: 'allow' | 'deny';

  /** Session timeout (seconds) */
  sessionTimeout?: number;
}

/**
 * ZTNA configuration
 */
export interface ZTNAConfiguration {
  /** Controller name */
  name: string;

  /** Identity provider */
  identityProvider: IdentityProvider;

  /** Posture requirements */
  postureRequirements?: PostureRequirements;

  /** Risk scoring */
  riskScoring?: RiskScoring;

  /** Access policies */
  policies: ZTNAPolicy[];

  /** Continuous verification interval (seconds) */
  continuousVerification?: number;
}

// ============================================================================
// Network Segmentation
// ============================================================================

/**
 * Segmentation strategy
 */
export type SegmentationStrategy = 'vlan' | 'subnet' | 'micro-segmentation' | 'software-defined';

/**
 * Isolation level
 */
export type IsolationLevel = 'strict' | 'moderate' | 'minimal' | 'complete';

/**
 * Inter-zone traffic policy
 */
export interface InterZoneTraffic {
  /** Source zone */
  from: string;

  /** Destination zone */
  to: string;

  /** Allowed ports */
  ports?: number[];

  /** Allowed protocols */
  protocols?: NetworkProtocol[];

  /** Traffic justification */
  justification?: string;
}

/**
 * Network zone for segmentation
 */
export interface NetworkZone {
  /** Zone name */
  name: string;

  /** VLAN IDs */
  vlans?: number[];

  /** Subnets */
  subnets?: string[];

  /** Isolation level */
  isolation: IsolationLevel;

  /** Allowed traffic patterns */
  allowedTraffic?: InterZoneTraffic[];

  /** Default action for non-matching traffic */
  defaultAction?: SecurityAction;
}

/**
 * Network segmentation configuration
 */
export interface SegmentationConfiguration {
  /** Segmentation strategy */
  strategy: SegmentationStrategy;

  /** Network zones */
  zones: NetworkZone[];

  /** Global policies */
  globalPolicies?: {
    denyByDefault?: boolean;
    logAllTraffic?: boolean;
  };
}

// ============================================================================
// DDoS Mitigation
// ============================================================================

/**
 * DDoS attack type
 */
export type DDoSAttackType =
  | 'volumetric'
  | 'protocol'
  | 'application-layer'
  | 'udp-flood'
  | 'syn-flood'
  | 'http-flood'
  | 'dns-amplification'
  | 'ntp-amplification';

/**
 * DDoS detection thresholds
 */
export interface DDoSThresholds {
  /** Packets per second threshold */
  packetsPerSecond?: number;

  /** Bits per second threshold */
  bitsPerSecond?: number;

  /** Connections per second threshold */
  connectionsPerSecond?: number;

  /** Requests per second threshold (application layer) */
  requestsPerSecond?: number;
}

/**
 * CDN integration for DDoS mitigation
 */
export interface CDNIntegration {
  /** CDN provider */
  provider: string;

  /** Enable CDN mitigation */
  enabled: boolean;

  /** API key */
  apiKey?: string;

  /** Challenge mode */
  challengeMode?: boolean;
}

/**
 * DDoS mitigation configuration
 */
export interface DDoSMitigationConfig {
  /** Configuration name */
  name: string;

  /** Detection thresholds */
  thresholds: DDoSThresholds;

  /** Mitigation techniques */
  mitigation: {
    /** Enable traffic scrubbing */
    scrubbing?: boolean;

    /** Enable rate limiting */
    rateLimiting?: boolean;

    /** Enable BGP Flowspec */
    bgpFlowspec?: boolean;

    /** CDN integration */
    cdnIntegration?: CDNIntegration;

    /** SYN proxy */
    synProxy?: boolean;
  };

  /** Alert configuration */
  alerts?: {
    email?: string[];
    webhook?: string;
    sms?: string[];
  };
}

/**
 * DDoS attack event
 */
export interface DDoSAttackEvent {
  /** Event ID */
  id: string;

  /** Attack start time */
  startTime: Date;

  /** Attack end time */
  endTime?: Date;

  /** Attack type */
  type: DDoSAttackType;

  /** Attack metrics */
  metrics: {
    peakPPS?: number;
    peakBPS?: number;
    peakConnections?: number;
    duration?: number; // seconds
  };

  /** Source information */
  sources?: Array<{
    ip: string;
    country?: string;
    asn?: number;
    packets: number;
  }>;

  /** Mitigation status */
  mitigation: {
    active: boolean;
    method?: string[];
    effectiveness?: number; // percentage
  };

  /** Severity */
  severity: SeverityLevel;
}

// ============================================================================
// Network Access Control (NAC)
// ============================================================================

/**
 * NAC authentication method
 */
export type NACAuthMethod = '802.1x' | 'mac-auth' | 'web-auth' | 'hybrid';

/**
 * Device profile
 */
export interface DeviceProfile {
  /** Device MAC address */
  mac: string;

  /** Device IP address */
  ip?: string;

  /** Device hostname */
  hostname?: string;

  /** Device type */
  type?: string;

  /** Operating system */
  os?: string;

  /** Manufacturer */
  manufacturer?: string;

  /** Classification certainty (0-100) */
  certainty?: number;
}

/**
 * NAC authorization result
 */
export interface NACAuthorization {
  /** Assigned VLAN */
  vlan: number;

  /** Access control list */
  acl?: string;

  /** Bandwidth limit */
  bandwidth?: string;

  /** Session timeout */
  sessionTimeout?: number;
}

/**
 * NAC configuration
 */
export interface NACConfiguration {
  /** NAC controller name */
  name: string;

  /** Authentication method */
  authentication: {
    method: NACAuthMethod;
    radiusServers?: string[];
    sharedSecret?: string;
  };

  /** Enable device profiling */
  deviceProfiling: boolean;

  /** Posture assessment */
  postureAssessment?: PostureRequirements;

  /** Compliance actions */
  compliance: {
    pass: NACAuthorization;
    fail: NACAuthorization;
    guest: NACAuthorization;
  };
}

// ============================================================================
// SSL/TLS Inspection
// ============================================================================

/**
 * TLS inspection method
 */
export type TLSInspectionMethod = 'man-in-the-middle' | 'passive' | 'none';

/**
 * TLS inspection bypass rule
 */
export interface TLSBypassRule {
  /** Category name */
  category: string;

  /** Domain patterns */
  domains?: string[];

  /** Application names */
  apps?: string[];

  /** Reason for bypass */
  reason: string;
}

/**
 * SSL/TLS inspection configuration
 */
export interface TLSInspectionConfig {
  /** Inspection method */
  method: TLSInspectionMethod;

  /** Interfaces to inspect */
  interfaces?: string[];

  /** Inspection CA certificate */
  caCertificate?: {
    subject: string;
    pem: string;
  };

  /** Bypass rules */
  bypassList?: TLSBypassRule[];

  /** Minimum TLS version to allow */
  minTLSVersion?: '1.0' | '1.1' | '1.2' | '1.3';

  /** Allowed cipher suites */
  allowedCiphers?: string[];
}

// ============================================================================
// DNS Security
// ============================================================================

/**
 * DNS security features
 */
export interface DNSSecurityConfig {
  /** Enable DNSSEC validation */
  dnssec: boolean;

  /** Enable DNS over HTTPS */
  doh?: boolean;

  /** DoH endpoint URL */
  dohEndpoint?: string;

  /** Enable DNS over TLS */
  dot?: boolean;

  /** Enable DNS firewall */
  dnsFirewall?: boolean;

  /** DNS blocklists */
  blocklists?: string[];

  /** Response Policy Zones */
  rpz?: string[];

  /** Query logging */
  logging?: {
    enabled: boolean;
    destination: string;
  };
}

// ============================================================================
// SD-WAN Security
// ============================================================================

/**
 * Tunnel encryption type
 */
export type TunnelType = 'ipsec' | 'wireguard' | 'gre' | 'vxlan';

/**
 * SD-WAN tunnel configuration
 */
export interface SDWANTunnel {
  /** Tunnel name */
  name: string;

  /** Tunnel type */
  type: TunnelType;

  /** Local endpoint */
  local: string;

  /** Remote endpoint */
  remote: string;

  /** Encryption algorithm */
  encryption?: string;

  /** Authentication algorithm */
  authentication?: string;

  /** Pre-shared key */
  psk?: string;

  /** Tunnel priority */
  priority: number;

  /** Bandwidth limit */
  bandwidth?: string;
}

/**
 * SD-WAN traffic policy
 */
export interface SDWANPolicy {
  /** Application name */
  app: string;

  /** Tunnel or path to use */
  tunnel: string;

  /** QoS priority */
  qos?: 'high' | 'medium' | 'low';

  /** Local internet breakout */
  breakout?: 'local' | 'hub';
}

/**
 * SD-WAN configuration
 */
export interface SDWANConfiguration {
  /** Site name */
  site: string;

  /** Tunnel configurations */
  tunnels: SDWANTunnel[];

  /** Traffic policies */
  policies?: SDWANPolicy[];

  /** Health check */
  healthCheck?: {
    interval: number; // seconds
    timeout: number;
    retries: number;
  };
}

// ============================================================================
// SIEM Integration
// ============================================================================

/**
 * Log format type
 */
export type LogFormat = 'syslog' | 'cef' | 'leef' | 'json' | 'custom';

/**
 * SIEM configuration
 */
export interface SIEMConfiguration {
  /** SIEM endpoint URL */
  endpoint: string;

  /** API key */
  apiKey?: string;

  /** Log format */
  format: LogFormat;

  /** TLS configuration */
  tls?: {
    enabled: boolean;
    validateCert: boolean;
  };

  /** Batch settings */
  batch?: {
    size: number;
    interval: number; // seconds
  };
}

/**
 * Security event for SIEM
 */
export interface SecurityEvent {
  /** Event ID */
  id: string;

  /** Timestamp */
  timestamp: Date;

  /** Event source */
  source: string;

  /** Event type */
  type: string;

  /** Severity */
  severity: SeverityLevel;

  /** Event message */
  message: string;

  /** Source IP */
  sourceIP?: string;

  /** Destination IP */
  destinationIP?: string;

  /** User */
  user?: string;

  /** Action taken */
  action?: SecurityAction;

  /** Threat information */
  threat?: {
    name: string;
    category: string;
    confidence: number;
  };

  /** Additional metadata */
  metadata?: Record<string, any>;
}

// ============================================================================
// Compliance and Reporting
// ============================================================================

/**
 * Compliance framework
 */
export type ComplianceFramework =
  | 'pci-dss'
  | 'hipaa'
  | 'gdpr'
  | 'iso-27001'
  | 'nist-csf'
  | 'cis'
  | 'cmmc';

/**
 * Compliance report
 */
export interface ComplianceReport {
  /** Framework */
  framework: ComplianceFramework;

  /** Reporting period */
  period: string;

  /** Controls assessed */
  controls: string[];

  /** Overall compliance score (percentage) */
  score: number;

  /** Passing controls */
  passing: number;

  /** Total controls */
  total: number;

  /** Findings */
  findings?: Array<{
    control: string;
    status: 'pass' | 'fail' | 'partial';
    details: string;
  }>;

  /** Evidence references */
  evidence?: string[];

  /** Generated timestamp */
  generatedAt: Date;
}

// ============================================================================
// Network Security SDK Configuration
// ============================================================================

/**
 * Network Security SDK initialization options
 */
export interface NetworkSecuritySDKOptions {
  /** SIEM integration */
  siem?: SIEMConfiguration;

  /** Threat intelligence */
  threatIntel?: {
    feeds: string[];
    updateInterval: number;
    apiKeys?: Record<string, string>;
  };

  /** Default timeout for operations */
  timeout?: number;

  /** Enable debug logging */
  debug?: boolean;
}

/**
 * Rate limit configuration
 */
export interface RateLimitConfig {
  /** Rate limit name */
  name: string;

  /** Scope of rate limit */
  scope: 'source-ip' | 'destination-ip' | 'user' | 'global';

  /** Protocol */
  protocol?: NetworkProtocol;

  /** TCP flags (for protocol=tcp) */
  flags?: string;

  /** Rate limit value */
  limit: number | string;

  /** Time period */
  period: string;

  /** Action when limit exceeded */
  action: SecurityAction;

  /** Burst allowance */
  burst?: number;

  /** Priority for established connections */
  priority?: string;
}

/**
 * Threat intelligence feed
 */
export interface ThreatIntelligenceFeed {
  /** Feed name */
  name: string;

  /** Feed URL */
  url: string;

  /** Feed type */
  type: 'ip' | 'domain' | 'hash' | 'url';

  /** Update frequency (seconds) */
  updateInterval: number;

  /** Last update */
  lastUpdate?: Date;

  /** Number of indicators */
  indicators?: number;
}

/**
 * Threat intelligence result
 */
export interface ThreatIntelligenceResult {
  /** Queried indicator */
  indicator: string;

  /** Indicator type */
  type: 'ip' | 'domain' | 'hash' | 'url';

  /** Is malicious */
  malicious: boolean;

  /** Threat categories */
  categories?: string[];

  /** Risk score (0-100) */
  riskScore?: number;

  /** Sources that flagged this */
  sources?: string[];

  /** Additional context */
  context?: Record<string, any>;
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  FirewallZone,
  FirewallEndpoint,
  DPIConfiguration,
  LoggingConfiguration,
  ScheduleConfiguration,
  FirewallPolicy,
  NGFWConfiguration,
  IDSRule,
  IDSActionConfig,
  IDSConfiguration,
  IDSAlert,
  IdentityProvider,
  PostureRequirements,
  RiskScoring,
  ZTNAConditions,
  ZTNAPolicy,
  ZTNAConfiguration,
  InterZoneTraffic,
  NetworkZone,
  SegmentationConfiguration,
  DDoSThresholds,
  CDNIntegration,
  DDoSMitigationConfig,
  DDoSAttackEvent,
  DeviceProfile,
  NACAuthorization,
  NACConfiguration,
  TLSBypassRule,
  TLSInspectionConfig,
  DNSSecurityConfig,
  SDWANTunnel,
  SDWANPolicy,
  SDWANConfiguration,
  SIEMConfiguration,
  SecurityEvent,
  ComplianceReport,
  NetworkSecuritySDKOptions,
  RateLimitConfig,
  ThreatIntelligenceFeed,
  ThreatIntelligenceResult,
};
