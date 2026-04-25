/**
 * WIA Cybersecurity Standard - TypeScript Type Definitions
 * Version 1.0.0
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Cybersecurity Types
// ============================================================================

export interface WIACybersecurityProject {
  standard: 'WIA-CYBERSECURITY';
  version: string;
  metadata: ProjectMetadata;
  riskManagement: RiskManagement;
  accessControl: AccessControl;
  networkSecurity: NetworkSecurity;
  applicationSecurity: ApplicationSecurity;
  dataSecurity: DataSecurity;
  incidentResponse: IncidentResponse;
  compliance: ComplianceFramework;
  monitoring: SecurityMonitoring;
  extensions?: Record<string, unknown>;
}

export interface ProjectMetadata {
  id: string;
  name: string;
  description?: string;
  organization: Organization;
  scope: SecurityScope;
  createdAt: string;
  updatedAt?: string;
  status: ProjectStatus;
}

export interface Organization {
  name: string;
  industry: string;
  size: 'small' | 'medium' | 'large' | 'enterprise';
  region: string;
  contact: ContactInfo;
}

export interface ContactInfo {
  name: string;
  email: string;
  phone?: string;
  role: string;
}

export interface SecurityScope {
  assets: AssetCategory[];
  networks: string[];
  applications: string[];
  dataClassifications: string[];
}

export type AssetCategory = 'hardware' | 'software' | 'data' | 'network' | 'personnel' | 'facilities';
export type ProjectStatus = 'active' | 'draft' | 'review' | 'archived';

// ============================================================================
// Risk Management Types
// ============================================================================

export interface RiskManagement {
  framework: RiskFramework;
  assessments: RiskAssessment[];
  registry: RiskRegistry;
  treatment: RiskTreatment;
}

export interface RiskFramework {
  methodology: 'NIST' | 'ISO27005' | 'FAIR' | 'OCTAVE' | 'custom';
  riskAppetite: 'low' | 'medium' | 'high';
  reviewFrequency: string;
  escalationCriteria: string[];
}

export interface RiskAssessment {
  id: string;
  name: string;
  scope: string;
  conductedDate: string;
  assessor: string;
  findings: RiskFinding[];
  status: 'planned' | 'in-progress' | 'completed' | 'reviewed';
}

export interface RiskFinding {
  id: string;
  asset: string;
  threat: string;
  vulnerability: string;
  likelihood: 1 | 2 | 3 | 4 | 5;
  impact: 1 | 2 | 3 | 4 | 5;
  riskScore: number;
  treatment: 'accept' | 'mitigate' | 'transfer' | 'avoid';
}

export interface RiskRegistry {
  risks: RegisteredRisk[];
  lastUpdated: string;
  owner: string;
}

export interface RegisteredRisk {
  id: string;
  category: string;
  description: string;
  inherentRisk: number;
  residualRisk: number;
  controls: string[];
  status: 'open' | 'mitigated' | 'accepted' | 'closed';
}

export interface RiskTreatment {
  plans: TreatmentPlan[];
  budget: number;
  timeline: string;
}

export interface TreatmentPlan {
  riskId: string;
  action: string;
  owner: string;
  deadline: string;
  status: 'planned' | 'in-progress' | 'completed';
  effectiveness?: number;
}

// ============================================================================
// Access Control Types
// ============================================================================

export interface AccessControl {
  policy: AccessPolicy;
  identityManagement: IdentityManagement;
  authentication: AuthenticationConfig;
  authorization: AuthorizationConfig;
  privilegedAccess: PrivilegedAccessManagement;
}

export interface AccessPolicy {
  model: 'RBAC' | 'ABAC' | 'MAC' | 'DAC' | 'hybrid';
  principles: string[];
  reviewFrequency: string;
  exceptions: PolicyException[];
}

export interface PolicyException {
  id: string;
  reason: string;
  approver: string;
  expiryDate: string;
  conditions: string[];
}

export interface IdentityManagement {
  provider: string;
  lifecycle: IdentityLifecycle;
  federation: FederationConfig;
  provisioning: ProvisioningConfig;
}

export interface IdentityLifecycle {
  onboarding: string[];
  changes: string[];
  offboarding: string[];
  reviewFrequency: string;
}

export interface FederationConfig {
  enabled: boolean;
  protocols: ('SAML' | 'OIDC' | 'OAuth2')[];
  trustedProviders: string[];
}

export interface ProvisioningConfig {
  automated: boolean;
  approvalWorkflow: boolean;
  defaultRoles: string[];
}

export interface AuthenticationConfig {
  methods: AuthMethod[];
  mfaRequired: boolean;
  passwordPolicy: PasswordPolicy;
  sessionManagement: SessionConfig;
}

export interface AuthMethod {
  type: 'password' | 'mfa' | 'biometric' | 'certificate' | 'sso';
  enabled: boolean;
  priority: number;
}

export interface PasswordPolicy {
  minLength: number;
  complexity: string[];
  expiryDays: number;
  historyCount: number;
  lockoutThreshold: number;
}

export interface SessionConfig {
  timeout: number;
  maxConcurrent: number;
  secure: boolean;
}

export interface AuthorizationConfig {
  engine: string;
  policies: AuthzPolicy[];
  enforcement: 'strict' | 'permissive';
}

export interface AuthzPolicy {
  id: string;
  name: string;
  resources: string[];
  actions: string[];
  conditions: string[];
}

export interface PrivilegedAccessManagement {
  enabled: boolean;
  vault: string;
  justInTime: boolean;
  sessionRecording: boolean;
  approvalRequired: boolean;
}

// ============================================================================
// Network Security Types
// ============================================================================

export interface NetworkSecurity {
  architecture: NetworkArchitecture;
  segmentation: NetworkSegmentation;
  perimeter: PerimeterSecurity;
  encryption: NetworkEncryption;
  monitoring: NetworkMonitoring;
}

export interface NetworkArchitecture {
  topology: 'flat' | 'segmented' | 'zero-trust';
  zones: NetworkZone[];
  dmz: boolean;
  cloudConnectivity: string[];
}

export interface NetworkZone {
  id: string;
  name: string;
  classification: 'public' | 'internal' | 'restricted' | 'confidential';
  assets: string[];
  accessRules: string[];
}

export interface NetworkSegmentation {
  vlans: VlanConfig[];
  microsegmentation: boolean;
  sdnEnabled: boolean;
}

export interface VlanConfig {
  id: number;
  name: string;
  purpose: string;
  subnet: string;
}

export interface PerimeterSecurity {
  firewalls: FirewallConfig[];
  ids: IntrusionDetection;
  waf: WebAppFirewall;
  ddosProtection: DdosConfig;
}

export interface FirewallConfig {
  id: string;
  type: 'network' | 'host' | 'ngfw';
  vendor: string;
  ruleCount: number;
  lastReview: string;
}

export interface IntrusionDetection {
  type: 'IDS' | 'IPS' | 'hybrid';
  mode: 'inline' | 'passive';
  signatureUpdates: string;
  coverage: string[];
}

export interface WebAppFirewall {
  enabled: boolean;
  mode: 'detect' | 'block';
  rulesets: string[];
  customRules: number;
}

export interface DdosConfig {
  enabled: boolean;
  provider: string;
  mitigationCapacity: string;
  autoMitigation: boolean;
}

export interface NetworkEncryption {
  inTransit: TransitEncryption;
  vpn: VpnConfig;
  tlsConfig: TlsConfig;
}

export interface TransitEncryption {
  required: boolean;
  protocols: string[];
  minimumVersion: string;
}

export interface VpnConfig {
  type: 'site-to-site' | 'remote-access' | 'both';
  protocol: 'IPSec' | 'OpenVPN' | 'WireGuard';
  encryption: string;
}

export interface TlsConfig {
  minimumVersion: '1.2' | '1.3';
  cipherSuites: string[];
  certificateManagement: string;
}

export interface NetworkMonitoring {
  netflow: boolean;
  packetCapture: boolean;
  anomalyDetection: boolean;
  retentionDays: number;
}

// ============================================================================
// Application & Data Security Types
// ============================================================================

export interface ApplicationSecurity {
  sdlc: SecureSdlc;
  testing: SecurityTesting;
  runtime: RuntimeProtection;
  apiSecurity: ApiSecurityConfig;
}

export interface SecureSdlc {
  methodology: string;
  securityGates: SecurityGate[];
  training: string;
  codeReview: CodeReviewConfig;
}

export interface SecurityGate {
  phase: 'design' | 'development' | 'testing' | 'deployment';
  requirements: string[];
  approver: string;
}

export interface CodeReviewConfig {
  required: boolean;
  tools: string[];
  coverage: number;
}

export interface SecurityTesting {
  sast: TestingConfig;
  dast: TestingConfig;
  penetrationTesting: PenTestConfig;
  bugBounty: BugBountyConfig;
}

export interface TestingConfig {
  enabled: boolean;
  tools: string[];
  frequency: string;
  coverage: number;
}

export interface PenTestConfig {
  frequency: string;
  scope: string[];
  vendor?: string;
  lastTest?: string;
}

export interface BugBountyConfig {
  enabled: boolean;
  platform?: string;
  scope: string[];
  rewards: { severity: string; amount: number }[];
}

export interface RuntimeProtection {
  rasp: boolean;
  containerSecurity: ContainerSecurityConfig;
  secrets: SecretsManagement;
}

export interface ContainerSecurityConfig {
  scanning: boolean;
  runtime: boolean;
  policies: string[];
}

export interface SecretsManagement {
  vault: string;
  rotation: boolean;
  scanning: boolean;
}

export interface ApiSecurityConfig {
  authentication: string[];
  rateLimit: boolean;
  inputValidation: boolean;
  logging: boolean;
}

export interface DataSecurity {
  classification: DataClassification;
  encryption: DataEncryption;
  dlp: DataLossPrevention;
  backup: BackupConfig;
  retention: RetentionPolicy;
}

export interface DataClassification {
  scheme: string;
  levels: ClassificationLevel[];
  labeling: boolean;
  automation: boolean;
}

export interface ClassificationLevel {
  name: string;
  description: string;
  controls: string[];
  handling: string;
}

export interface DataEncryption {
  atRest: EncryptionConfig;
  inUse: EncryptionConfig;
  keyManagement: KeyManagementConfig;
}

export interface EncryptionConfig {
  enabled: boolean;
  algorithm: string;
  keyLength: number;
}

export interface KeyManagementConfig {
  provider: string;
  rotation: boolean;
  rotationPeriod: string;
  hsm: boolean;
}

export interface DataLossPrevention {
  enabled: boolean;
  channels: ('email' | 'web' | 'endpoint' | 'cloud')[];
  policies: DlpPolicy[];
}

export interface DlpPolicy {
  id: string;
  name: string;
  patterns: string[];
  action: 'alert' | 'block' | 'encrypt';
}

export interface BackupConfig {
  frequency: string;
  retention: string;
  encryption: boolean;
  offsite: boolean;
  testing: string;
}

export interface RetentionPolicy {
  default: string;
  byClassification: { level: string; period: string }[];
  legalHold: boolean;
}

// ============================================================================
// Incident Response & Monitoring Types
// ============================================================================

export interface IncidentResponse {
  plan: IncidentPlan;
  team: ResponseTeam;
  playbooks: Playbook[];
  communication: CommunicationPlan;
  forensics: ForensicsCapability;
}

export interface IncidentPlan {
  version: string;
  lastReview: string;
  phases: IncidentPhase[];
  escalation: EscalationMatrix;
}

export interface IncidentPhase {
  name: 'preparation' | 'detection' | 'containment' | 'eradication' | 'recovery' | 'lessons-learned';
  procedures: string[];
  tools: string[];
  roles: string[];
}

export interface EscalationMatrix {
  levels: EscalationLevel[];
  timeframes: { severity: string; escalateWithin: string }[];
}

export interface EscalationLevel {
  level: number;
  criteria: string;
  contacts: string[];
  authority: string;
}

export interface ResponseTeam {
  structure: string;
  members: TeamMember[];
  onCall: string;
  training: string;
  exercises: string;
}

export interface TeamMember {
  role: string;
  name: string;
  contact: string;
  responsibilities: string[];
}

export interface Playbook {
  id: string;
  name: string;
  trigger: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  steps: PlaybookStep[];
  automation: number;
}

export interface PlaybookStep {
  order: number;
  action: string;
  responsible: string;
  tools?: string[];
  success?: string;
}

export interface CommunicationPlan {
  internal: { channels: string[]; templates: string[] };
  external: { stakeholders: string[]; protocols: string[] };
  regulatory: { authorities: string[]; timeframes: string[] };
}

export interface ForensicsCapability {
  tools: string[];
  chain: string;
  preservation: string;
  analysis: string[];
}

export interface SecurityMonitoring {
  siem: SiemConfig;
  logging: LoggingConfig;
  alerting: AlertingConfig;
  metrics: MetricsConfig;
}

export interface SiemConfig {
  platform: string;
  sources: string[];
  retentionDays: number;
  correlationRules: number;
}

export interface LoggingConfig {
  centralized: boolean;
  sources: string[];
  format: string;
  integrity: boolean;
}

export interface AlertingConfig {
  channels: string[];
  severity: { level: string; channel: string }[];
  suppressionRules: number;
}

export interface MetricsConfig {
  kpis: SecurityKpi[];
  dashboards: string[];
  reporting: string;
}

export interface SecurityKpi {
  name: string;
  target: number;
  current: number;
  trend: 'up' | 'down' | 'stable';
}

// ============================================================================
// Compliance Types
// ============================================================================

export interface ComplianceFramework {
  regulations: string[];
  standards: string[];
  controls: ComplianceControl[];
  audits: AuditConfig;
  reporting: ComplianceReporting;
}

export interface ComplianceControl {
  id: string;
  framework: string;
  requirement: string;
  implementation: string;
  evidence: string[];
  status: 'compliant' | 'partial' | 'non-compliant' | 'not-applicable';
}

export interface AuditConfig {
  internal: { frequency: string; scope: string[] };
  external: { frequency: string; auditor: string };
  certifications: string[];
}

export interface ComplianceReporting {
  frequency: string;
  recipients: string[];
  format: string;
  automation: boolean;
}

// ============================================================================
// API Types
// ============================================================================

export interface APIConfig {
  baseURL: string;
  apiKey?: string;
  timeout?: number;
}

export interface ProjectResponse {
  id: string;
  name: string;
  status: ProjectStatus;
  createdAt: string;
  updatedAt?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
}

export interface ValidationError {
  path: string;
  message: string;
  value?: unknown;
}

export interface PaginatedResponse<T> {
  data: T[];
  pagination: {
    total: number;
    limit: number;
    offset: number;
    hasMore: boolean;
  };
}
