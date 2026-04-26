# Chapter 7: Connected Car Security

## Cybersecurity Frameworks and Privacy Protection

This chapter establishes comprehensive security requirements for connected vehicle systems, addressing cyber threats, data protection, and regulatory compliance across global markets.

---

## Automotive Cybersecurity Framework

### Security Architecture Overview

```typescript
// WIA Connected Car Security Framework
// Comprehensive cybersecurity implementation

/**
 * Automotive Cybersecurity Architecture
 * Based on ISO/SAE 21434 and UNECE WP.29
 */
interface AutomotiveCybersecurityArchitecture {
  securityGovernance: SecurityGovernance;
  threatAnalysis: TARA;
  securityControls: SecurityControlSet;
  incidentResponse: IncidentResponsePlan;
  continuousMonitoring: MonitoringSystem;
}

interface SecurityGovernance {
  securityPolicy: SecurityPolicy;
  riskManagement: RiskManagementFramework;
  complianceRequirements: ComplianceRequirement[];
  securityOrganization: SecurityOrganization;
  auditProgram: AuditProgram;
}

interface SecurityPolicy {
  version: string;
  effectiveDate: Date;
  approvalAuthority: string;
  scope: PolicyScope[];
  principles: SecurityPrinciple[];
  requirements: PolicyRequirement[];
  exceptions: PolicyException[];
  reviewCycle: number;  // months
}

type PolicyScope =
  | "VEHICLE_SYSTEMS"
  | "BACKEND_INFRASTRUCTURE"
  | "MOBILE_APPLICATIONS"
  | "TELEMATICS_UNITS"
  | "THIRD_PARTY_INTEGRATIONS";

interface SecurityPrinciple {
  id: string;
  name: string;
  description: string;
  implementation: string[];
}

/**
 * Threat Analysis and Risk Assessment (TARA)
 * ISO/SAE 21434 compliant methodology
 */
interface TARA {
  assetIdentification: Asset[];
  threatScenarios: ThreatScenario[];
  vulnerabilityAssessment: Vulnerability[];
  riskEvaluation: RiskAssessment[];
  treatmentPlan: RiskTreatment[];
}

interface Asset {
  id: string;
  name: string;
  type: AssetType;
  location: AssetLocation;
  securityProperties: SecurityProperty[];
  owner: string;
  value: AssetValue;
  dependencies: string[];
}

type AssetType =
  | "DATA" | "FUNCTION" | "COMPONENT"
  | "COMMUNICATION" | "INTERFACE" | "KEY_MATERIAL";

type AssetLocation =
  | "ECU" | "TCU" | "GATEWAY" | "CLOUD"
  | "MOBILE_APP" | "EXTERNAL_INTERFACE";

interface SecurityProperty {
  property: "CONFIDENTIALITY" | "INTEGRITY" | "AVAILABILITY" | "AUTHENTICITY" | "NON_REPUDIATION";
  level: SecurityLevel;
  rationale: string;
}

type SecurityLevel = "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";

interface ThreatScenario {
  id: string;
  name: string;
  category: ThreatCategory;
  attackVector: AttackVector;
  targetAssets: string[];
  threatActors: ThreatActor[];
  likelihood: LikelihoodLevel;
  impact: ImpactAssessment;
  attackSteps: AttackStep[];
  mitigations: string[];
}

type ThreatCategory =
  | "SPOOFING" | "TAMPERING" | "REPUDIATION"
  | "INFORMATION_DISCLOSURE" | "DENIAL_OF_SERVICE"
  | "ELEVATION_OF_PRIVILEGE";

type AttackVector =
  | "NETWORK" | "PHYSICAL" | "LOCAL"
  | "ADJACENT_NETWORK" | "USER";

interface ThreatActor {
  type: ThreatActorType;
  motivation: string;
  capability: CapabilityLevel;
  resources: ResourceLevel;
}

type ThreatActorType =
  | "NATION_STATE" | "ORGANIZED_CRIME" | "HACKTIVIST"
  | "INSIDER" | "OPPORTUNISTIC" | "RESEARCHER";

type CapabilityLevel = "BASIC" | "INTERMEDIATE" | "ADVANCED" | "EXPERT";
type ResourceLevel = "LIMITED" | "MODERATE" | "SIGNIFICANT" | "EXTENSIVE";
type LikelihoodLevel = "RARE" | "UNLIKELY" | "POSSIBLE" | "LIKELY" | "ALMOST_CERTAIN";

interface ImpactAssessment {
  safety: ImpactLevel;
  financial: ImpactLevel;
  operational: ImpactLevel;
  privacy: ImpactLevel;
  reputation: ImpactLevel;
  overall: ImpactLevel;
}

type ImpactLevel = "NEGLIGIBLE" | "MINOR" | "MODERATE" | "MAJOR" | "SEVERE";

interface AttackStep {
  sequence: number;
  action: string;
  requiredAccess: string;
  tools: string[];
  detectability: DetectabilityLevel;
}

type DetectabilityLevel = "EASY" | "MODERATE" | "DIFFICULT" | "VERY_DIFFICULT";

interface Vulnerability {
  id: string;
  cveId?: string;
  description: string;
  affectedAssets: string[];
  severity: CVSSSeverity;
  cvssScore: number;
  exploitability: ExploitabilityMetrics;
  remediationStatus: RemediationStatus;
}

interface CVSSSeverity {
  baseScore: number;
  temporalScore: number;
  environmentalScore: number;
  overallSeverity: "NONE" | "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";
}

interface ExploitabilityMetrics {
  attackVector: string;
  attackComplexity: string;
  privilegesRequired: string;
  userInteraction: string;
  scope: string;
}

type RemediationStatus =
  | "OPEN" | "IN_PROGRESS" | "REMEDIATED"
  | "ACCEPTED" | "TRANSFERRED";

interface RiskAssessment {
  threatId: string;
  vulnerabilityId: string;
  riskLevel: RiskLevel;
  riskScore: number;
  treatmentRequired: boolean;
  residualRisk: RiskLevel;
}

type RiskLevel = "VERY_LOW" | "LOW" | "MEDIUM" | "HIGH" | "VERY_HIGH";

interface RiskTreatment {
  riskId: string;
  strategy: TreatmentStrategy;
  controls: string[];
  owner: string;
  deadline: Date;
  status: TreatmentStatus;
  effectiveness: EffectivenessLevel;
}

type TreatmentStrategy =
  | "AVOID" | "MITIGATE" | "TRANSFER" | "ACCEPT";

type TreatmentStatus =
  | "PLANNED" | "IN_PROGRESS" | "IMPLEMENTED" | "VERIFIED";

type EffectivenessLevel =
  | "NOT_EFFECTIVE" | "PARTIALLY_EFFECTIVE"
  | "EFFECTIVE" | "HIGHLY_EFFECTIVE";

/**
 * Security Control Implementation
 */
class VehicleSecurityController {
  private hsm: HardwareSecurityModule;
  private keyManager: KeyManagementService;
  private cryptoEngine: CryptoEngine;
  private ids: IntrusionDetectionSystem;
  private accessControl: AccessControlManager;

  constructor(config: SecurityConfig) {
    this.hsm = new HardwareSecurityModule(config.hsmConfig);
    this.keyManager = new KeyManagementService(this.hsm, config.keyConfig);
    this.cryptoEngine = new CryptoEngine(this.hsm);
    this.ids = new IntrusionDetectionSystem(config.idsConfig);
    this.accessControl = new AccessControlManager(config.accessConfig);
  }

  /**
   * Secure boot verification
   */
  async verifySecureBoot(): Promise<SecureBootResult> {
    const measurements: BootMeasurement[] = [];

    // Verify bootloader
    const bootloaderMeasurement = await this.hsm.measureComponent("BOOTLOADER");
    measurements.push(bootloaderMeasurement);

    if (!this.verifyMeasurement(bootloaderMeasurement, "BOOTLOADER")) {
      return {
        success: false,
        failedStage: "BOOTLOADER",
        measurements
      };
    }

    // Verify kernel
    const kernelMeasurement = await this.hsm.measureComponent("KERNEL");
    measurements.push(kernelMeasurement);

    if (!this.verifyMeasurement(kernelMeasurement, "KERNEL")) {
      return {
        success: false,
        failedStage: "KERNEL",
        measurements
      };
    }

    // Verify critical applications
    const appMeasurement = await this.hsm.measureComponent("CRITICAL_APPS");
    measurements.push(appMeasurement);

    if (!this.verifyMeasurement(appMeasurement, "CRITICAL_APPS")) {
      return {
        success: false,
        failedStage: "CRITICAL_APPS",
        measurements
      };
    }

    // Extend PCR with measurements
    await this.hsm.extendPCR(measurements);

    return {
      success: true,
      measurements,
      attestationReport: await this.generateAttestationReport(measurements)
    };
  }

  /**
   * Authenticate external entity
   */
  async authenticate(
    request: AuthenticationRequest
  ): Promise<AuthenticationResult> {
    // Validate request integrity
    if (!await this.validateRequestSignature(request)) {
      return {
        success: false,
        error: "Invalid request signature"
      };
    }

    // Check certificate validity
    const certValidation = await this.validateCertificate(request.certificate);
    if (!certValidation.valid) {
      return {
        success: false,
        error: `Certificate validation failed: ${certValidation.reason}`
      };
    }

    // Verify challenge response
    const challengeResult = await this.verifyChallengeResponse(
      request.challengeResponse,
      request.certificate.publicKey
    );

    if (!challengeResult.valid) {
      return {
        success: false,
        error: "Challenge verification failed"
      };
    }

    // Check authorization
    const authzResult = await this.accessControl.checkAuthorization(
      request.entityId,
      request.requestedPermissions
    );

    if (!authzResult.authorized) {
      return {
        success: false,
        error: `Not authorized: ${authzResult.reason}`
      };
    }

    // Generate session token
    const sessionToken = await this.createSecureSession(
      request.entityId,
      authzResult.grantedPermissions
    );

    return {
      success: true,
      sessionToken,
      permissions: authzResult.grantedPermissions,
      expiresAt: sessionToken.expiresAt
    };
  }

  /**
   * Encrypt data for secure transmission
   */
  async encryptData(
    plaintext: Buffer,
    recipient: string,
    context: EncryptionContext
  ): Promise<EncryptedData> {
    // Get recipient's public key
    const recipientKey = await this.keyManager.getPublicKey(recipient);

    // Generate ephemeral key pair for ECDH
    const ephemeralKeyPair = await this.cryptoEngine.generateECDHKeyPair();

    // Derive shared secret
    const sharedSecret = await this.cryptoEngine.deriveSharedSecret(
      ephemeralKeyPair.privateKey,
      recipientKey
    );

    // Derive encryption key using HKDF
    const encryptionKey = await this.cryptoEngine.hkdf(
      sharedSecret,
      context.salt,
      context.info,
      32
    );

    // Generate nonce
    const nonce = await this.cryptoEngine.generateSecureRandom(12);

    // Encrypt with AES-GCM
    const ciphertext = await this.cryptoEngine.aesGcmEncrypt(
      encryptionKey,
      nonce,
      plaintext,
      context.aad
    );

    return {
      ciphertext: ciphertext.encrypted,
      authTag: ciphertext.authTag,
      nonce,
      ephemeralPublicKey: ephemeralKeyPair.publicKey,
      algorithm: "ECDH-ES+A256GCM",
      keyId: recipientKey.keyId
    };
  }

  /**
   * Verify firmware update signature
   */
  async verifyFirmwareUpdate(
    update: FirmwareUpdate
  ): Promise<FirmwareVerificationResult> {
    // Verify metadata signature
    const metadataValid = await this.cryptoEngine.verifySignature(
      update.metadata,
      update.metadataSignature,
      await this.keyManager.getOEMSigningKey()
    );

    if (!metadataValid) {
      return {
        valid: false,
        error: "Invalid metadata signature"
      };
    }

    // Verify update package hash
    const packageHash = await this.cryptoEngine.sha256(update.package);
    if (!packageHash.equals(update.metadata.packageHash)) {
      return {
        valid: false,
        error: "Package hash mismatch"
      };
    }

    // Verify code signing
    const codeSignatureValid = await this.cryptoEngine.verifyCodeSignature(
      update.package,
      update.codeSignature,
      await this.keyManager.getCodeSigningCertificate()
    );

    if (!codeSignatureValid) {
      return {
        valid: false,
        error: "Invalid code signature"
      };
    }

    // Verify version is newer (anti-rollback)
    const currentVersion = await this.getCurrentFirmwareVersion();
    if (this.compareVersions(update.metadata.version, currentVersion) <= 0) {
      return {
        valid: false,
        error: "Firmware version rollback not allowed"
      };
    }

    return {
      valid: true,
      metadata: update.metadata,
      approvedForInstallation: true
    };
  }

  // Helper methods (stubs)
  private verifyMeasurement(measurement: BootMeasurement, component: string): boolean {
    return true;
  }
  private async generateAttestationReport(measurements: BootMeasurement[]): Promise<any> {
    return {};
  }
  private async validateRequestSignature(request: AuthenticationRequest): Promise<boolean> {
    return true;
  }
  private async validateCertificate(cert: any): Promise<{ valid: boolean; reason?: string }> {
    return { valid: true };
  }
  private async verifyChallengeResponse(response: any, publicKey: any): Promise<{ valid: boolean }> {
    return { valid: true };
  }
  private async createSecureSession(entityId: string, permissions: string[]): Promise<SessionToken> {
    return { token: "", expiresAt: new Date() };
  }
  private async getCurrentFirmwareVersion(): Promise<string> {
    return "1.0.0";
  }
  private compareVersions(a: string, b: string): number {
    return 1;
  }
}

interface SecurityConfig {
  hsmConfig: any;
  keyConfig: any;
  idsConfig: any;
  accessConfig: any;
}

interface SecureBootResult {
  success: boolean;
  failedStage?: string;
  measurements: BootMeasurement[];
  attestationReport?: any;
}

interface BootMeasurement {
  component: string;
  hash: Buffer;
  timestamp: Date;
}

interface AuthenticationRequest {
  entityId: string;
  certificate: any;
  challengeResponse: any;
  requestedPermissions: string[];
}

interface AuthenticationResult {
  success: boolean;
  error?: string;
  sessionToken?: SessionToken;
  permissions?: string[];
  expiresAt?: Date;
}

interface SessionToken {
  token: string;
  expiresAt: Date;
}

interface EncryptionContext {
  salt: Buffer;
  info: Buffer;
  aad: Buffer;
}

interface EncryptedData {
  ciphertext: Buffer;
  authTag: Buffer;
  nonce: Buffer;
  ephemeralPublicKey: Buffer;
  algorithm: string;
  keyId: string;
}

interface FirmwareUpdate {
  metadata: FirmwareMetadata;
  metadataSignature: Buffer;
  package: Buffer;
  codeSignature: Buffer;
}

interface FirmwareMetadata {
  version: string;
  packageHash: Buffer;
  releaseDate: Date;
  targetHardware: string[];
}

interface FirmwareVerificationResult {
  valid: boolean;
  error?: string;
  metadata?: FirmwareMetadata;
  approvedForInstallation?: boolean;
}

// Supporting classes (stubs)
class HardwareSecurityModule {
  constructor(config: any) {}
  async measureComponent(component: string): Promise<BootMeasurement> {
    return { component, hash: Buffer.alloc(32), timestamp: new Date() };
  }
  async extendPCR(measurements: BootMeasurement[]): Promise<void> {}
}

class KeyManagementService {
  constructor(hsm: HardwareSecurityModule, config: any) {}
  async getPublicKey(recipient: string): Promise<any> { return {}; }
  async getOEMSigningKey(): Promise<any> { return {}; }
  async getCodeSigningCertificate(): Promise<any> { return {}; }
}

class CryptoEngine {
  constructor(hsm: HardwareSecurityModule) {}
  async generateECDHKeyPair(): Promise<any> { return {}; }
  async deriveSharedSecret(privateKey: any, publicKey: any): Promise<Buffer> { return Buffer.alloc(32); }
  async hkdf(secret: Buffer, salt: Buffer, info: Buffer, length: number): Promise<Buffer> { return Buffer.alloc(length); }
  async generateSecureRandom(length: number): Promise<Buffer> { return Buffer.alloc(length); }
  async aesGcmEncrypt(key: Buffer, nonce: Buffer, plaintext: Buffer, aad: Buffer): Promise<any> {
    return { encrypted: Buffer.alloc(0), authTag: Buffer.alloc(16) };
  }
  async verifySignature(data: any, signature: Buffer, key: any): Promise<boolean> { return true; }
  async sha256(data: Buffer): Promise<Buffer> { return Buffer.alloc(32); }
  async verifyCodeSignature(data: Buffer, signature: Buffer, cert: any): Promise<boolean> { return true; }
}

class IntrusionDetectionSystem {
  constructor(config: any) {}
}

class AccessControlManager {
  constructor(config: any) {}
  async checkAuthorization(entityId: string, permissions: string[]): Promise<any> {
    return { authorized: true, grantedPermissions: permissions };
  }
}

interface AssetValue {
  criticality: string;
  sensitivity: string;
}

interface PolicyRequirement {
  id: string;
  requirement: string;
  controls: string[];
}

interface PolicyException {
  id: string;
  description: string;
  approver: string;
  expirationDate: Date;
}

interface RiskManagementFramework {
  methodology: string;
  riskAppetite: any;
  assessmentCycle: number;
}

interface ComplianceRequirement {
  regulation: string;
  requirements: string[];
  status: string;
}

interface SecurityOrganization {
  ciso: string;
  securityTeam: any[];
  responsibilities: any;
}

interface AuditProgram {
  internalAudits: any[];
  externalAudits: any[];
  certifications: string[];
}

interface SecurityControlSet {
  preventive: SecurityControl[];
  detective: SecurityControl[];
  corrective: SecurityControl[];
}

interface SecurityControl {
  id: string;
  name: string;
  description: string;
  type: string;
  implementation: string;
  effectiveness: string;
}

interface IncidentResponsePlan {
  phases: any[];
  contacts: any[];
  playbooks: any[];
}

interface MonitoringSystem {
  sources: any[];
  alerts: any[];
  dashboards: any[];
}
```

---

## Intrusion Detection System

```typescript
/**
 * Automotive Intrusion Detection System (IDS)
 * Multi-layer threat detection for connected vehicles
 */
class AutomotiveIDS {
  private networkMonitor: NetworkIDSModule;
  private canBusMonitor: CANBusIDSModule;
  private applicationMonitor: ApplicationIDSModule;
  private behavioralAnalyzer: BehavioralAnalysisEngine;
  private alertManager: AlertManager;
  private mlEngine: MLDetectionEngine;

  constructor(config: IDSConfig) {
    this.networkMonitor = new NetworkIDSModule(config.network);
    this.canBusMonitor = new CANBusIDSModule(config.canBus);
    this.applicationMonitor = new ApplicationIDSModule(config.application);
    this.behavioralAnalyzer = new BehavioralAnalysisEngine(config.behavioral);
    this.alertManager = new AlertManager(config.alerting);
    this.mlEngine = new MLDetectionEngine(config.ml);
  }

  /**
   * Start IDS monitoring
   */
  async start(): Promise<void> {
    // Start all monitoring modules
    await Promise.all([
      this.networkMonitor.start(),
      this.canBusMonitor.start(),
      this.applicationMonitor.start(),
      this.behavioralAnalyzer.start()
    ]);

    // Setup event correlation
    this.setupEventCorrelation();

    console.log("Automotive IDS started");
  }

  /**
   * Process network traffic
   */
  processNetworkTraffic(packet: NetworkPacket): DetectionResult[] {
    const results: DetectionResult[] = [];

    // Signature-based detection
    const signatureHits = this.networkMonitor.matchSignatures(packet);
    results.push(...signatureHits);

    // Protocol anomaly detection
    const protocolAnomalies = this.networkMonitor.detectProtocolAnomalies(packet);
    results.push(...protocolAnomalies);

    // ML-based detection
    const mlResults = this.mlEngine.analyzeNetworkTraffic(packet);
    results.push(...mlResults);

    // Generate alerts for detections
    for (const result of results) {
      if (result.confidence >= result.threshold) {
        this.alertManager.createAlert(result);
      }
    }

    return results;
  }

  /**
   * Process CAN bus message
   */
  processCANMessage(message: CANMessage): DetectionResult[] {
    const results: DetectionResult[] = [];

    // Message frequency analysis
    const frequencyAnomaly = this.canBusMonitor.checkMessageFrequency(message);
    if (frequencyAnomaly) {
      results.push(frequencyAnomaly);
    }

    // Message content validation
    const contentAnomaly = this.canBusMonitor.validateMessageContent(message);
    if (contentAnomaly) {
      results.push(contentAnomaly);
    }

    // Injection detection
    const injectionDetection = this.canBusMonitor.detectInjection(message);
    if (injectionDetection) {
      results.push(injectionDetection);
    }

    // Timing analysis
    const timingAnomaly = this.canBusMonitor.analyzeMessageTiming(message);
    if (timingAnomaly) {
      results.push(timingAnomaly);
    }

    return results;
  }

  private setupEventCorrelation(): void {
    // Correlate events from multiple sources
    this.networkMonitor.on("detection", (result) => {
      this.correlateEvent(result, "NETWORK");
    });

    this.canBusMonitor.on("detection", (result) => {
      this.correlateEvent(result, "CAN_BUS");
    });

    this.applicationMonitor.on("detection", (result) => {
      this.correlateEvent(result, "APPLICATION");
    });
  }

  private correlateEvent(result: DetectionResult, source: string): void {
    // Event correlation logic
    this.behavioralAnalyzer.addEvent({
      ...result,
      source,
      timestamp: new Date()
    });
  }
}

interface IDSConfig {
  network: NetworkIDSConfig;
  canBus: CANBusIDSConfig;
  application: ApplicationIDSConfig;
  behavioral: BehavioralConfig;
  alerting: AlertConfig;
  ml: MLConfig;
}

interface NetworkIDSConfig {
  interfaces: string[];
  signatures: SignatureSet;
  protocols: ProtocolConfig[];
}

interface CANBusIDSConfig {
  buses: string[];
  baselineProfile: CANBaselineProfile;
  injectionDetection: InjectionDetectionConfig;
}

interface CANBaselineProfile {
  messageFrequencies: Map<number, FrequencyRange>;
  expectedPayloads: Map<number, PayloadPattern>;
  messageTiming: Map<number, TimingProfile>;
}

interface FrequencyRange {
  min: number;  // messages per second
  max: number;
  mean: number;
  stdDev: number;
}

interface PayloadPattern {
  mask: Buffer;
  expectedValues: Buffer[];
  valueRanges?: ValueRange[];
}

interface ValueRange {
  startBit: number;
  length: number;
  min: number;
  max: number;
}

interface TimingProfile {
  expectedInterval: number;  // milliseconds
  jitterTolerance: number;
  burstThreshold: number;
}

interface DetectionResult {
  id: string;
  type: DetectionType;
  severity: AlertSeverity;
  confidence: number;
  threshold: number;
  description: string;
  evidence: Evidence;
  recommendation: string;
  timestamp: Date;
}

type DetectionType =
  | "SIGNATURE_MATCH"
  | "PROTOCOL_ANOMALY"
  | "FREQUENCY_ANOMALY"
  | "CONTENT_ANOMALY"
  | "TIMING_ANOMALY"
  | "INJECTION_ATTEMPT"
  | "BEHAVIORAL_ANOMALY"
  | "ML_DETECTION";

type AlertSeverity = "LOW" | "MEDIUM" | "HIGH" | "CRITICAL";

interface Evidence {
  rawData: Buffer;
  context: Record<string, any>;
  relatedEvents: string[];
}

interface SignatureSet {
  version: string;
  signatures: Signature[];
}

interface Signature {
  id: string;
  name: string;
  pattern: Buffer | RegExp;
  severity: AlertSeverity;
  category: string;
}

interface ProtocolConfig {
  protocol: string;
  enabled: boolean;
  strictMode: boolean;
}

interface ApplicationIDSConfig {
  processes: string[];
  apiMonitoring: boolean;
  fileIntegrity: boolean;
}

interface BehavioralConfig {
  learningPeriod: number;
  anomalyThreshold: number;
  features: string[];
}

interface AlertConfig {
  channels: AlertChannel[];
  escalationPolicy: EscalationPolicy;
}

interface AlertChannel {
  type: "LOG" | "SIEM" | "CLOUD" | "LOCAL_DISPLAY";
  config: any;
}

interface EscalationPolicy {
  levels: EscalationLevel[];
}

interface EscalationLevel {
  severity: AlertSeverity;
  notifyAfter: number;
  contacts: string[];
}

interface MLConfig {
  models: MLModel[];
  inferenceMode: "LOCAL" | "EDGE" | "CLOUD";
}

interface MLModel {
  name: string;
  type: "ANOMALY_DETECTION" | "CLASSIFICATION";
  path: string;
  threshold: number;
}

interface InjectionDetectionConfig {
  enabled: boolean;
  methods: string[];
}

interface NetworkPacket {
  timestamp: Date;
  sourceIP: string;
  destIP: string;
  protocol: string;
  payload: Buffer;
}

interface CANMessage {
  id: number;
  data: Buffer;
  timestamp: number;
}

// Module stubs
class NetworkIDSModule {
  constructor(config: NetworkIDSConfig) {}
  async start(): Promise<void> {}
  matchSignatures(packet: NetworkPacket): DetectionResult[] { return []; }
  detectProtocolAnomalies(packet: NetworkPacket): DetectionResult[] { return []; }
  on(event: string, callback: (result: DetectionResult) => void): void {}
}

class CANBusIDSModule {
  constructor(config: CANBusIDSConfig) {}
  async start(): Promise<void> {}
  checkMessageFrequency(msg: CANMessage): DetectionResult | null { return null; }
  validateMessageContent(msg: CANMessage): DetectionResult | null { return null; }
  detectInjection(msg: CANMessage): DetectionResult | null { return null; }
  analyzeMessageTiming(msg: CANMessage): DetectionResult | null { return null; }
  on(event: string, callback: (result: DetectionResult) => void): void {}
}

class ApplicationIDSModule {
  constructor(config: ApplicationIDSConfig) {}
  async start(): Promise<void> {}
  on(event: string, callback: (result: DetectionResult) => void): void {}
}

class BehavioralAnalysisEngine {
  constructor(config: BehavioralConfig) {}
  async start(): Promise<void> {}
  addEvent(event: any): void {}
}

class AlertManager {
  constructor(config: AlertConfig) {}
  createAlert(result: DetectionResult): void {}
}

class MLDetectionEngine {
  constructor(config: MLConfig) {}
  analyzeNetworkTraffic(packet: NetworkPacket): DetectionResult[] { return []; }
}
```

---

## Privacy Protection Framework

```typescript
/**
 * Privacy Protection Framework
 * GDPR, CCPA, and automotive-specific privacy requirements
 */
interface PrivacyFramework {
  dataClassification: DataClassificationScheme;
  consentManagement: ConsentManagementSystem;
  dataMinimization: DataMinimizationPolicy;
  anonymization: AnonymizationEngine;
  rightsFulfillment: DataSubjectRights;
  dataRetention: RetentionPolicy;
  crossBorderTransfer: TransferMechanisms;
}

/**
 * Data Classification for Vehicle Data
 */
interface DataClassificationScheme {
  categories: DataCategory[];
  classificationRules: ClassificationRule[];
  handlingRequirements: HandlingRequirement[];
}

interface DataCategory {
  id: string;
  name: string;
  description: string;
  sensitivityLevel: SensitivityLevel;
  legalBasis: LegalBasis[];
  retentionPeriod: RetentionPeriod;
  examples: string[];
}

type SensitivityLevel =
  | "PUBLIC"
  | "INTERNAL"
  | "CONFIDENTIAL"
  | "RESTRICTED"
  | "HIGHLY_RESTRICTED";

type LegalBasis =
  | "CONSENT"
  | "CONTRACT"
  | "LEGAL_OBLIGATION"
  | "VITAL_INTEREST"
  | "PUBLIC_TASK"
  | "LEGITIMATE_INTEREST";

const vehicleDataClassification: DataCategory[] = [
  {
    id: "PII_DIRECT",
    name: "Direct Personal Identifiers",
    description: "Data that directly identifies an individual",
    sensitivityLevel: "HIGHLY_RESTRICTED",
    legalBasis: ["CONSENT", "CONTRACT"],
    retentionPeriod: { duration: 36, unit: "MONTHS", basis: "LAST_ACTIVITY" },
    examples: ["Driver name", "Email", "Phone number", "License plate"]
  },
  {
    id: "LOCATION_PRECISE",
    name: "Precise Location Data",
    description: "GPS coordinates and precise location history",
    sensitivityLevel: "RESTRICTED",
    legalBasis: ["CONSENT", "LEGITIMATE_INTEREST"],
    retentionPeriod: { duration: 12, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["GPS coordinates", "Trip history", "Parking locations"]
  },
  {
    id: "DRIVING_BEHAVIOR",
    name: "Driving Behavior Data",
    description: "Data revealing driving patterns and behavior",
    sensitivityLevel: "CONFIDENTIAL",
    legalBasis: ["CONSENT", "LEGITIMATE_INTEREST"],
    retentionPeriod: { duration: 24, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["Speed patterns", "Braking behavior", "Acceleration profiles"]
  },
  {
    id: "VEHICLE_TELEMETRY",
    name: "Vehicle Telemetry",
    description: "Technical vehicle data without personal identifiers",
    sensitivityLevel: "INTERNAL",
    legalBasis: ["CONTRACT", "LEGITIMATE_INTEREST"],
    retentionPeriod: { duration: 60, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["Battery status", "Tire pressure", "Engine diagnostics"]
  },
  {
    id: "BIOMETRIC",
    name: "Biometric Data",
    description: "Biometric identifiers from driver monitoring",
    sensitivityLevel: "HIGHLY_RESTRICTED",
    legalBasis: ["CONSENT"],
    retentionPeriod: { duration: 6, unit: "MONTHS", basis: "COLLECTION" },
    examples: ["Face images", "Voice prints", "Driver attention data"]
  }
];

interface RetentionPeriod {
  duration: number;
  unit: "DAYS" | "MONTHS" | "YEARS";
  basis: "COLLECTION" | "LAST_ACTIVITY" | "CONTRACT_END" | "EVENT";
}

/**
 * Consent Management System
 */
class ConsentManagementService {
  private consentStore: ConsentStore;
  private purposeRegistry: PurposeRegistry;

  constructor(
    consentStore: ConsentStore,
    purposeRegistry: PurposeRegistry
  ) {
    this.consentStore = consentStore;
    this.purposeRegistry = purposeRegistry;
  }

  /**
   * Record consent from user
   */
  async recordConsent(
    userId: string,
    vehicleId: string,
    consents: ConsentGrant[]
  ): Promise<ConsentRecord> {
    // Validate purposes
    for (const consent of consents) {
      const purpose = await this.purposeRegistry.getPurpose(consent.purposeId);
      if (!purpose) {
        throw new Error(`Invalid purpose: ${consent.purposeId}`);
      }
    }

    // Create consent record
    const record: ConsentRecord = {
      id: crypto.randomUUID(),
      userId,
      vehicleId,
      consents,
      timestamp: new Date(),
      source: "USER_INTERFACE",
      version: "1.0",
      ipAddress: consents[0].metadata?.ipAddress,
      userAgent: consents[0].metadata?.userAgent
    };

    // Store consent
    await this.consentStore.store(record);

    // Update active consent state
    await this.updateActiveConsents(userId, vehicleId);

    return record;
  }

  /**
   * Check if consent exists for specific purpose
   */
  async hasConsent(
    userId: string,
    vehicleId: string,
    purposeId: string
  ): Promise<ConsentCheckResult> {
    const activeConsents = await this.consentStore.getActiveConsents(
      userId,
      vehicleId
    );

    const consent = activeConsents.find(c =>
      c.consents.some(g => g.purposeId === purposeId && g.granted)
    );

    if (!consent) {
      return { hasConsent: false, reason: "NO_CONSENT_RECORD" };
    }

    const grant = consent.consents.find(g => g.purposeId === purposeId);

    // Check expiration
    if (grant?.expiresAt && new Date() > grant.expiresAt) {
      return { hasConsent: false, reason: "CONSENT_EXPIRED" };
    }

    return {
      hasConsent: true,
      consentId: consent.id,
      grantedAt: consent.timestamp,
      expiresAt: grant?.expiresAt
    };
  }

  /**
   * Withdraw consent
   */
  async withdrawConsent(
    userId: string,
    vehicleId: string,
    purposeIds: string[]
  ): Promise<void> {
    const record: ConsentRecord = {
      id: crypto.randomUUID(),
      userId,
      vehicleId,
      consents: purposeIds.map(purposeId => ({
        purposeId,
        granted: false,
        withdrawnAt: new Date()
      })),
      timestamp: new Date(),
      source: "USER_INTERFACE",
      version: "1.0"
    };

    await this.consentStore.store(record);
    await this.updateActiveConsents(userId, vehicleId);

    // Trigger data deletion workflow if required
    await this.handleConsentWithdrawal(userId, vehicleId, purposeIds);
  }

  /**
   * Get consent preferences for user
   */
  async getConsentPreferences(
    userId: string,
    vehicleId: string
  ): Promise<ConsentPreferences> {
    const purposes = await this.purposeRegistry.getAllPurposes();
    const activeConsents = await this.consentStore.getActiveConsents(
      userId,
      vehicleId
    );

    return {
      userId,
      vehicleId,
      purposes: purposes.map(purpose => ({
        purposeId: purpose.id,
        name: purpose.name,
        description: purpose.description,
        required: purpose.required,
        granted: this.isConsentGranted(activeConsents, purpose.id),
        grantedAt: this.getConsentGrantDate(activeConsents, purpose.id),
        dataCategories: purpose.dataCategories
      })),
      lastUpdated: this.getLastUpdateDate(activeConsents)
    };
  }

  private async updateActiveConsents(
    userId: string,
    vehicleId: string
  ): Promise<void> {
    // Implementation
  }

  private async handleConsentWithdrawal(
    userId: string,
    vehicleId: string,
    purposeIds: string[]
  ): Promise<void> {
    // Trigger data deletion for withdrawn purposes
  }

  private isConsentGranted(
    consents: ConsentRecord[],
    purposeId: string
  ): boolean {
    return consents.some(c =>
      c.consents.some(g => g.purposeId === purposeId && g.granted)
    );
  }

  private getConsentGrantDate(
    consents: ConsentRecord[],
    purposeId: string
  ): Date | undefined {
    const record = consents.find(c =>
      c.consents.some(g => g.purposeId === purposeId && g.granted)
    );
    return record?.timestamp;
  }

  private getLastUpdateDate(consents: ConsentRecord[]): Date | undefined {
    if (consents.length === 0) return undefined;
    return consents.reduce((latest, c) =>
      c.timestamp > latest ? c.timestamp : latest,
      consents[0].timestamp
    );
  }
}

interface ConsentGrant {
  purposeId: string;
  granted: boolean;
  expiresAt?: Date;
  withdrawnAt?: Date;
  metadata?: Record<string, any>;
}

interface ConsentRecord {
  id: string;
  userId: string;
  vehicleId: string;
  consents: ConsentGrant[];
  timestamp: Date;
  source: ConsentSource;
  version: string;
  ipAddress?: string;
  userAgent?: string;
}

type ConsentSource =
  | "USER_INTERFACE"
  | "MOBILE_APP"
  | "VEHICLE_HMI"
  | "API"
  | "PAPER_FORM";

interface ConsentCheckResult {
  hasConsent: boolean;
  reason?: string;
  consentId?: string;
  grantedAt?: Date;
  expiresAt?: Date;
}

interface ConsentPreferences {
  userId: string;
  vehicleId: string;
  purposes: PurposePreference[];
  lastUpdated?: Date;
}

interface PurposePreference {
  purposeId: string;
  name: string;
  description: string;
  required: boolean;
  granted: boolean;
  grantedAt?: Date;
  dataCategories: string[];
}

interface ConsentStore {
  store(record: ConsentRecord): Promise<void>;
  getActiveConsents(userId: string, vehicleId: string): Promise<ConsentRecord[]>;
}

interface PurposeRegistry {
  getPurpose(purposeId: string): Promise<Purpose | null>;
  getAllPurposes(): Promise<Purpose[]>;
}

interface Purpose {
  id: string;
  name: string;
  description: string;
  required: boolean;
  dataCategories: string[];
}

interface ClassificationRule {
  pattern: string;
  category: string;
}

interface HandlingRequirement {
  category: string;
  encryption: boolean;
  accessControl: string;
  auditLogging: boolean;
}

interface DataMinimizationPolicy {
  rules: MinimizationRule[];
}

interface MinimizationRule {
  dataType: string;
  purpose: string;
  minimization: string;
}

interface AnonymizationEngine {
  anonymize(data: any, technique: string): any;
}

interface DataSubjectRights {
  access: boolean;
  rectification: boolean;
  erasure: boolean;
  portability: boolean;
  objection: boolean;
}

interface RetentionPolicy {
  categories: RetentionCategory[];
}

interface RetentionCategory {
  category: string;
  period: RetentionPeriod;
  archival: boolean;
}

interface TransferMechanisms {
  adequacyDecisions: string[];
  standardContractualClauses: boolean;
  bindingCorporateRules: boolean;
}
```

---

## Regulatory Compliance

```typescript
/**
 * Automotive Regulatory Compliance Framework
 * UNECE WP.29, ISO/SAE 21434, regional requirements
 */
interface RegulatoryComplianceFramework {
  uneceWP29: UNECEWP29Compliance;
  isoSae21434: ISO21434Compliance;
  regionalCompliance: RegionalRequirement[];
}

interface UNECEWP29Compliance {
  unr155: UNR155CybersecurityCompliance;
  unr156: UNR156SoftwareUpdateCompliance;
}

/**
 * UN Regulation 155 - Cybersecurity Compliance
 */
interface UNR155CybersecurityCompliance {
  csmsRequirements: CSMSRequirement[];
  vehicleTypeApproval: VTARequirements;
  threatMitigation: ThreatMitigationEvidence[];
}

interface CSMSRequirement {
  id: string;
  category: CSMSCategory;
  requirement: string;
  evidence: string[];
  status: ComplianceStatus;
}

type CSMSCategory =
  | "RISK_MANAGEMENT"
  | "VEHICLE_DEVELOPMENT"
  | "PRODUCTION"
  | "POST_PRODUCTION"
  | "INCIDENT_RESPONSE"
  | "SUPPLY_CHAIN";

type ComplianceStatus =
  | "COMPLIANT"
  | "PARTIALLY_COMPLIANT"
  | "NON_COMPLIANT"
  | "NOT_APPLICABLE";

const unr155Requirements: CSMSRequirement[] = [
  {
    id: "UNR155-7.2.2.2-a",
    category: "RISK_MANAGEMENT",
    requirement: "Processes to identify cyber risks to vehicle types",
    evidence: [
      "TARA documentation",
      "Risk assessment reports",
      "Threat intelligence integration"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.2-b",
    category: "RISK_MANAGEMENT",
    requirement: "Processes to assess identified risks",
    evidence: [
      "Risk scoring methodology",
      "Impact assessment criteria",
      "Risk prioritization framework"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.2-c",
    category: "RISK_MANAGEMENT",
    requirement: "Processes to categorize and treat identified risks",
    evidence: [
      "Risk treatment plans",
      "Control implementation records",
      "Residual risk acceptance"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.3",
    category: "VEHICLE_DEVELOPMENT",
    requirement: "Security by design in vehicle development",
    evidence: [
      "Security requirements specification",
      "Secure coding standards",
      "Security testing results"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.4",
    category: "SUPPLY_CHAIN",
    requirement: "Manage cybersecurity risks in supply chain",
    evidence: [
      "Supplier security assessments",
      "Security requirements in contracts",
      "Component security verification"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.5",
    category: "INCIDENT_RESPONSE",
    requirement: "Detect and respond to cyber attacks",
    evidence: [
      "IDS/SOC capabilities",
      "Incident response procedures",
      "Incident tracking records"
    ],
    status: "COMPLIANT"
  },
  {
    id: "UNR155-7.2.2.6",
    category: "POST_PRODUCTION",
    requirement: "Monitor and respond to new threats",
    evidence: [
      "Threat intelligence program",
      "Vulnerability monitoring",
      "Security patch management"
    ],
    status: "COMPLIANT"
  }
];

interface VTARequirements {
  typeApprovalNumber: string;
  approvalAuthority: string;
  approvalDate: Date;
  validUntil: Date;
  coveredVehicleTypes: string[];
  conditions: string[];
}

interface ThreatMitigationEvidence {
  threatCategory: string;
  threats: ThreatMitigation[];
}

interface ThreatMitigation {
  threatId: string;
  description: string;
  mitigation: string;
  verificationMethod: string;
  status: MitigationStatus;
}

type MitigationStatus = "IMPLEMENTED" | "PLANNED" | "NOT_REQUIRED";

/**
 * UN Regulation 156 - Software Update Compliance
 */
interface UNR156SoftwareUpdateCompliance {
  sumsRequirements: SUMSRequirement[];
  rxswinManagement: RXSWINManagement;
  updateProcessVerification: UpdateProcessEvidence[];
}

interface SUMSRequirement {
  id: string;
  requirement: string;
  evidence: string[];
  status: ComplianceStatus;
}

interface RXSWINManagement {
  currentRxswin: string;
  managementProcess: string;
  updateHistory: RXSWINUpdate[];
}

interface RXSWINUpdate {
  previousRxswin: string;
  newRxswin: string;
  updateDate: Date;
  reason: string;
  approvalReference: string;
}

interface UpdateProcessEvidence {
  processStep: string;
  description: string;
  verification: string;
  evidence: string[];
}

/**
 * ISO/SAE 21434 Compliance
 */
interface ISO21434Compliance {
  organizationalCybersecurity: OrganizationalRequirements;
  projectDependentCybersecurity: ProjectRequirements;
  continuousCybersecurity: ContinuousRequirements;
}

interface OrganizationalRequirements {
  cybersecurityGovernance: GovernanceEvidence;
  cybersecurityCulture: CultureEvidence;
  informationSharing: SharingEvidence;
  managementSystems: ManagementSystemEvidence;
}

interface GovernanceEvidence {
  policy: string;
  roles: string[];
  responsibilities: string[];
  resources: string[];
}

interface CultureEvidence {
  awarenessProgram: string;
  trainingRecords: string[];
  competencyAssessments: string[];
}

interface SharingEvidence {
  isacs: string[];
  sharingAgreements: string[];
  threatIntelligenceFeeds: string[];
}

interface ManagementSystemEvidence {
  qualityManagement: string;
  toolQualification: string;
  auditProgram: string;
}

interface ProjectRequirements {
  conceptPhase: PhaseEvidence;
  developmentPhase: PhaseEvidence;
  productionPhase: PhaseEvidence;
  operationsPhase: PhaseEvidence;
}

interface PhaseEvidence {
  activities: string[];
  workProducts: string[];
  verification: string[];
}

interface ContinuousRequirements {
  cybersecurityMonitoring: MonitoringEvidence;
  vulnerabilityManagement: VulnerabilityEvidence;
  incidentResponse: IncidentEvidence;
}

interface MonitoringEvidence {
  sources: string[];
  processes: string[];
  metrics: string[];
}

interface VulnerabilityEvidence {
  discoveryProcess: string;
  assessmentProcess: string;
  remediationProcess: string;
  disclosurePolicy: string;
}

interface IncidentEvidence {
  responseProcess: string;
  communicationPlan: string;
  lessonsLearned: string;
}

/**
 * Regional Compliance Requirements
 */
interface RegionalRequirement {
  region: string;
  regulations: RegulationDetail[];
  effectiveDate: Date;
  applicability: string[];
}

const regionalRequirements: RegionalRequirement[] = [
  {
    region: "EUROPEAN_UNION",
    regulations: [
      {
        name: "GDPR",
        requirements: [
          "Data protection impact assessment for vehicle data",
          "Privacy by design in connected car features",
          "Data subject rights implementation",
          "Data breach notification within 72 hours"
        ]
      },
      {
        name: "UNECE WP.29 (UN R155/R156)",
        requirements: [
          "CSMS certification for vehicle types",
          "SUMS certification for OTA updates",
          "Type approval for cybersecurity"
        ]
      },
      {
        name: "EU Cyber Resilience Act",
        requirements: [
          "CE marking for connected products",
          "Vulnerability handling process",
          "Security update obligations"
        ]
      }
    ],
    effectiveDate: new Date("2024-07-01"),
    applicability: ["New vehicle types from July 2024", "All vehicles from July 2026"]
  },
  {
    region: "UNITED_STATES",
    regulations: [
      {
        name: "CCPA/CPRA",
        requirements: [
          "Consumer privacy rights for vehicle data",
          "Opt-out for data sales",
          "Privacy policy disclosures"
        ]
      },
      {
        name: "NHTSA Cybersecurity Best Practices",
        requirements: [
          "Layered security approach",
          "Incident response capabilities",
          "Self-auditing program"
        ]
      },
      {
        name: "State-level regulations",
        requirements: [
          "Varying requirements by state",
          "Data breach notification laws",
          "Connected vehicle data rights"
        ]
      }
    ],
    effectiveDate: new Date("2023-01-01"),
    applicability: ["All vehicles sold in US market"]
  },
  {
    region: "CHINA",
    regulations: [
      {
        name: "GB/T Automotive Standards",
        requirements: [
          "Data localization requirements",
          "Cross-border data transfer restrictions",
          "Security assessment for data export"
        ]
      },
      {
        name: "Cybersecurity Law / PIPL",
        requirements: [
          "Critical information infrastructure protection",
          "Personal information protection",
          "Data classification and grading"
        ]
      }
    ],
    effectiveDate: new Date("2023-01-01"),
    applicability: ["All vehicles sold in Chinese market"]
  }
];

interface RegulationDetail {
  name: string;
  requirements: string[];
}
```

---

## Summary

| Security Domain | Key Controls | Standards |
|-----------------|--------------|-----------|
| **Secure Boot** | TPM, code signing, measured boot | ISO 21434 |
| **Communication** | TLS 1.3, mTLS, encryption | AUTOSAR SecOC |
| **Authentication** | PKI, challenge-response, MFA | IEEE 1609.2 |
| **Intrusion Detection** | CAN IDS, network monitoring | UN R155 |
| **Privacy** | Consent management, anonymization | GDPR, CCPA |
| **Compliance** | CSMS, SUMS, type approval | UN R155/R156 |

---

**Next Chapter:** [Chapter 8: Implementation](./08-implementation.md) - Deployment, testing, and certification.

---

© 2025 World Industry Association (WIA). All rights reserved.
