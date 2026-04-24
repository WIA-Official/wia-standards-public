/**
 * WIA Security Event Types
 * Core type definitions for the WIA Security Standard
 */

// ============================================================================
// Enums
// ============================================================================

export type EventType =
  | 'alert'
  | 'threat_intel'
  | 'vulnerability'
  | 'incident'
  | 'network_event'
  | 'endpoint_event'
  | 'auth_event';

export type SourceType = 'siem' | 'edr' | 'ids' | 'firewall' | 'scanner' | 'custom';

export type Priority = 'critical' | 'high' | 'medium' | 'low' | 'info';

export type Direction = 'inbound' | 'outbound' | 'internal';

export type CloudProvider = 'aws' | 'azure' | 'gcp';

export type AlertCategory = 'malware' | 'intrusion' | 'policy' | 'reconnaissance' | 'other';

export type AlertStatus = 'new' | 'investigating' | 'resolved' | 'false_positive' | 'closed';

export type ThreatType = 'malware' | 'apt' | 'campaign' | 'botnet' | 'ransomware' | 'phishing';

export type ThreatStatus = 'active' | 'inactive' | 'unknown';

export type CvssSeverity = 'critical' | 'high' | 'medium' | 'low' | 'none';

export type IncidentCategory =
  | 'malware'
  | 'phishing'
  | 'ransomware'
  | 'data_breach'
  | 'ddos'
  | 'unauthorized_access'
  | 'insider_threat'
  | 'apt'
  | 'other';

export type IncidentStatus =
  | 'new'
  | 'triaging'
  | 'investigating'
  | 'containing'
  | 'eradicating'
  | 'recovering'
  | 'closed';

export type ImpactLevel = 'none' | 'low' | 'medium' | 'high' | 'critical';

export type NetworkEventType =
  | 'connection'
  | 'dns'
  | 'http'
  | 'tls'
  | 'smtp'
  | 'ftp'
  | 'ssh'
  | 'rdp'
  | 'custom';

export type NetworkProtocol = 'TCP' | 'UDP' | 'ICMP' | 'GRE' | 'ESP';

export type NetworkAction = 'allowed' | 'blocked' | 'dropped' | 'reset';

export type EndpointEventType =
  | 'process_creation'
  | 'process_termination'
  | 'file_create'
  | 'file_modify'
  | 'file_delete'
  | 'registry_create'
  | 'registry_modify'
  | 'registry_delete'
  | 'network_connection'
  | 'dll_load'
  | 'driver_load'
  | 'service_install';

export type AuthEventType =
  | 'login_success'
  | 'login_failure'
  | 'logout'
  | 'password_change'
  | 'account_locked'
  | 'account_unlocked'
  | 'mfa_success'
  | 'mfa_failure'
  | 'privilege_escalation';

export type AuthResult = 'success' | 'failure';

export type AuthMethod =
  | 'password'
  | 'sso'
  | 'certificate'
  | 'biometric'
  | 'mfa'
  | 'api_key'
  | 'oauth';

// ============================================================================
// Base Interfaces
// ============================================================================

export interface Source {
  type: SourceType;
  name: string;
  vendor?: string;
  version?: string;
}

export interface OperatingSystem {
  name?: string;
  version?: string;
  build?: string;
}

export interface Host {
  hostname?: string;
  ip?: string[];
  mac?: string[];
  os?: OperatingSystem;
  domain?: string;
  agent_id?: string;
}

export interface NetworkEndpoint {
  ip?: string;
  port?: number;
  hostname?: string;
}

export interface GeoLocation {
  country?: string;
  country_name?: string;
  city?: string;
  latitude?: number;
  longitude?: number;
  asn?: number;
  org?: string;
}

export interface User {
  name?: string;
  domain?: string;
  email?: string;
  employee_id?: string;
  groups?: string[];
  roles?: string[];
}

export interface Cloud {
  provider?: CloudProvider;
  account_id?: string;
  region?: string;
}

export interface NetworkContext {
  source?: NetworkEndpoint;
  destination?: NetworkEndpoint;
  protocol?: string;
  direction?: Direction;
}

export interface Context {
  host?: Host;
  network?: NetworkContext;
  user?: User;
  cloud?: Cloud;
}

export interface Mitre {
  tactic?: string;
  tactic_name?: string;
  technique?: string;
  technique_name?: string;
  sub_technique?: string;
  sub_technique_name?: string;
}

export interface Meta {
  confidence?: number;
  tags?: string[];
  labels?: Record<string, string>;
  raw?: string;
  correlation_id?: string;
  custom?: Record<string, unknown>;
}

// ============================================================================
// Event Data Interfaces
// ============================================================================

export interface Indicator {
  type: 'ip' | 'domain' | 'url' | 'hash' | 'email' | 'file' | 'file_hash_md5' | 'file_hash_sha1' | 'file_hash_sha256' | 'mutex' | 'registry';
  value: string;
  context?: string;
  confidence?: number;
  first_seen?: string;
  last_seen?: string;
}

export interface DetectionRule {
  id: string;
  name: string;
  version?: string;
}

export interface AlertData {
  alert_id: string;
  title: string;
  description?: string;
  category: AlertCategory;
  status: AlertStatus;
  priority: Priority;
  assignee?: string;
  detection_rule?: DetectionRule;
  indicators?: Indicator[];
  first_seen?: string;
  last_seen?: string;
  count?: number;
}

export interface TTP {
  tactic?: string;
  tactic_name?: string;
  technique?: string;
  technique_name?: string;
}

export interface ThreatIndicator extends Indicator {
  context?: {
    description?: string;
    kill_chain_phase?: string;
  };
}

export interface ThreatIntelData {
  threat_type: ThreatType;
  threat_name: string;
  threat_family?: string;
  aliases?: string[];
  first_seen?: string;
  last_seen?: string;
  status: ThreatStatus;
  indicators?: ThreatIndicator[];
  ttps?: TTP[];
  target_sectors?: string[];
  target_countries?: string[];
  references?: string[];
  report_id?: string;
}

export interface CVSS {
  version: '2.0' | '3.0' | '3.1' | '4.0';
  score: number;
  vector?: string;
  severity: CvssSeverity;
  base_score?: number;
  temporal_score?: number;
  environmental_score?: number;
}

export interface AffectedProduct {
  vendor: string;
  product: string;
  versions?: string[];
  cpe?: string;
}

export interface ExploitDetails {
  type: 'poc' | 'weaponized' | 'in_the_wild';
  url?: string;
  maturity?: 'proof_of_concept' | 'functional' | 'high';
}

export interface PatchDetails {
  vendor_url?: string;
  fixed_versions?: string[];
  workarounds?: string[];
}

export interface VulnerabilityData {
  vuln_id: string;
  title: string;
  description?: string;
  cvss: CVSS;
  cwe?: string[];
  affected_products?: AffectedProduct[];
  exploit_available?: boolean;
  exploit_details?: ExploitDetails;
  patch_available?: boolean;
  patch_details?: PatchDetails;
  references?: string[];
  published?: string;
  modified?: string;
}

export interface Impact {
  availability?: ImpactLevel;
  confidentiality?: ImpactLevel;
  integrity?: ImpactLevel;
  financial?: 'unknown' | 'minimal' | 'moderate' | 'significant' | 'severe';
  reputation?: 'unknown' | 'minimal' | 'moderate' | 'significant' | 'severe';
}

export interface TimelineEntry {
  timestamp: string;
  action: string;
  actor?: string;
  details?: string;
}

export interface AffectedAsset {
  type: 'host' | 'server' | 'network' | 'application' | 'database' | 'user';
  id: string;
  name?: string;
  ip?: string;
  criticality?: 'critical' | 'high' | 'medium' | 'low';
}

export interface IOC {
  type: 'ip' | 'domain' | 'url' | 'hash' | 'email';
  value: string;
  description?: string;
  first_seen?: string;
}

export interface ResponseAction {
  action: 'isolate_host' | 'block_ip' | 'disable_user' | 'quarantine_file' | 'collect_forensics';
  target: string;
  status: 'pending' | 'in_progress' | 'completed' | 'failed';
  timestamp?: string;
  actor?: string;
  notes?: string;
}

export interface IncidentData {
  incident_id: string;
  title: string;
  description?: string;
  category: IncidentCategory;
  status: IncidentStatus;
  priority: Priority;
  impact?: Impact;
  timeline?: TimelineEntry[];
  affected_assets?: AffectedAsset[];
  iocs?: IOC[];
  response_actions?: ResponseAction[];
  root_cause?: string;
  lessons_learned?: string;
  created_at?: string;
  updated_at?: string;
  closed_at?: string;
  lead_analyst?: string;
  team?: string[];
}

export interface NetworkHost {
  ip: string;
  port?: number;
  hostname?: string;
  mac?: string;
  geo?: GeoLocation;
}

export interface HttpInfo {
  method?: 'GET' | 'POST' | 'PUT' | 'DELETE' | 'PATCH' | 'HEAD' | 'OPTIONS' | 'CONNECT' | 'TRACE';
  status_code?: number;
  user_agent?: string;
  referer?: string;
  content_type?: string;
}

export interface DnsInfo {
  query?: string;
  query_type?: 'A' | 'AAAA' | 'CNAME' | 'MX' | 'NS' | 'PTR' | 'SOA' | 'SRV' | 'TXT';
  response?: string[];
  response_code?: 'NOERROR' | 'NXDOMAIN' | 'SERVFAIL' | 'REFUSED' | 'FORMERR';
}

export interface TlsInfo {
  version?: 'TLS 1.0' | 'TLS 1.1' | 'TLS 1.2' | 'TLS 1.3';
  cipher?: string;
  sni?: string;
  certificate?: {
    issuer?: string;
    subject?: string;
    not_before?: string;
    not_after?: string;
    fingerprint_sha256?: string;
  };
}

export interface NetworkEventData {
  event_type: NetworkEventType;
  protocol: NetworkProtocol;
  direction?: Direction;
  action?: NetworkAction;
  source: NetworkHost;
  destination: NetworkHost;
  bytes_sent?: number;
  bytes_received?: number;
  packets_sent?: number;
  packets_received?: number;
  duration_ms?: number;
  rule_matched?: string;
  application?: string;
  url?: string;
  http?: HttpInfo;
  dns?: DnsInfo;
  tls?: TlsInfo;
}

export interface FileHash {
  md5?: string;
  sha1?: string;
  sha256?: string;
}

export interface ProcessInfo {
  pid?: number;
  ppid?: number;
  name?: string;
  path?: string;
  command_line?: string;
  user?: string;
  integrity_level?: 'system' | 'high' | 'medium' | 'low';
  start_time?: string;
  hash?: FileHash;
  signature?: {
    signed?: boolean;
    valid?: boolean;
    issuer?: string;
    subject?: string;
  };
  parent?: {
    pid?: number;
    name?: string;
    path?: string;
    command_line?: string;
  };
}

export interface FileInfo {
  path?: string;
  name?: string;
  extension?: string;
  size?: number;
  hash?: FileHash;
  created?: string;
  modified?: string;
  accessed?: string;
  owner?: string;
  permissions?: string;
}

export interface RegistryInfo {
  key?: string;
  value_name?: string;
  value_data?: string;
  value_type?: 'REG_SZ' | 'REG_DWORD' | 'REG_QWORD' | 'REG_BINARY' | 'REG_MULTI_SZ' | 'REG_EXPAND_SZ';
  previous_data?: string;
}

export interface EndpointEventData {
  event_type: EndpointEventType;
  host: Host;
  process?: ProcessInfo;
  file?: FileInfo;
  registry?: RegistryInfo;
  network_connection?: {
    direction?: 'inbound' | 'outbound';
    protocol?: 'TCP' | 'UDP';
    local_ip?: string;
    local_port?: number;
    remote_ip?: string;
    remote_port?: number;
    state?: string;
  };
  dll?: {
    path?: string;
    name?: string;
    hash?: FileHash;
    signed?: boolean;
  };
}

export interface AuthSource {
  ip?: string;
  hostname?: string;
  geo?: {
    country?: string;
    city?: string;
  };
  user_agent?: string;
  device_type?: 'desktop' | 'mobile' | 'tablet' | 'unknown';
}

export interface AuthTarget {
  type: 'application' | 'host' | 'service' | 'network';
  name: string;
  ip?: string;
  url?: string;
}

export interface AuthEventData {
  event_type: AuthEventType;
  result: AuthResult;
  failure_reason?: 'invalid_credentials' | 'account_disabled' | 'account_locked' | 'expired_password' | 'mfa_failed' | 'unknown';
  user: User;
  source?: AuthSource;
  target: AuthTarget;
  auth_method?: AuthMethod;
  mfa_used?: boolean;
  mfa_method?: 'totp' | 'sms' | 'push' | 'hardware_token' | 'biometric';
  session_id?: string;
  logon_type?: 'interactive' | 'network' | 'batch' | 'service' | 'remote_interactive';
  attempt_count?: number;
  previous_login?: string;
  risk_score?: number;
  risk_factors?: string[];
}

// ============================================================================
// Main Event Types
// ============================================================================

export type EventData =
  | AlertData
  | ThreatIntelData
  | VulnerabilityData
  | IncidentData
  | NetworkEventData
  | EndpointEventData
  | AuthEventData;

export interface BaseEvent {
  $schema?: string;
  version: string;
  id: string;
  timestamp: string;
  severity: number;
  source: Source;
  context?: Context;
  mitre?: Mitre;
  meta?: Meta;
}

export interface AlertEvent extends BaseEvent {
  type: 'alert';
  data: AlertData;
}

export interface ThreatIntelEvent extends BaseEvent {
  type: 'threat_intel';
  data: ThreatIntelData;
}

export interface VulnerabilityEvent extends BaseEvent {
  type: 'vulnerability';
  data: VulnerabilityData;
}

export interface IncidentEvent extends BaseEvent {
  type: 'incident';
  data: IncidentData;
}

export interface NetworkEvent extends BaseEvent {
  type: 'network_event';
  data: NetworkEventData;
}

export interface EndpointEvent extends BaseEvent {
  type: 'endpoint_event';
  data: EndpointEventData;
}

export interface AuthEvent extends BaseEvent {
  type: 'auth_event';
  data: AuthEventData;
}

export type WiaSecurityEvent =
  | AlertEvent
  | ThreatIntelEvent
  | VulnerabilityEvent
  | IncidentEvent
  | NetworkEvent
  | EndpointEvent
  | AuthEvent;
