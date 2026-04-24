# Chapter 6: System Integration

## Overview

Cryogenic identity management must seamlessly integrate with multiple facility systems including specimen management, access control, consent management, and external regulatory systems. This chapter provides comprehensive integration patterns and implementations.

## Integration Architecture

### Core Integration Framework

```typescript
import { EventEmitter } from 'events';

// Integration types
type IntegrationType =
  | 'specimen-management'
  | 'access-control'
  | 'consent-management'
  | 'facility-monitoring'
  | 'billing'
  | 'regulatory-reporting'
  | 'external-registry';

type IntegrationStatus =
  | 'connected'
  | 'disconnected'
  | 'degraded'
  | 'error';

type SyncDirection =
  | 'inbound'
  | 'outbound'
  | 'bidirectional';

// Integration configuration
interface IntegrationConfig {
  id: string;
  name: string;
  type: IntegrationType;
  enabled: boolean;
  endpoint: EndpointConfig;
  authentication: AuthConfig;
  syncDirection: SyncDirection;
  syncInterval: number; // milliseconds
  retryPolicy: RetryPolicy;
  transformations: DataTransformation[];
  filters: DataFilter[];
}

interface EndpointConfig {
  protocol: 'http' | 'https' | 'grpc' | 'amqp' | 'kafka';
  host: string;
  port: number;
  path?: string;
  timeout: number;
}

interface AuthConfig {
  type: 'none' | 'api-key' | 'oauth2' | 'mtls' | 'jwt';
  credentials: Record<string, string>;
  tokenEndpoint?: string;
  refreshInterval?: number;
}

interface RetryPolicy {
  maxRetries: number;
  initialDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
}

interface DataTransformation {
  sourceField: string;
  targetField: string;
  transform?: (value: any) => any;
}

interface DataFilter {
  field: string;
  operator: 'eq' | 'neq' | 'gt' | 'lt' | 'contains' | 'in';
  value: any;
}

// Main integration hub
class IntegrationHub extends EventEmitter {
  private integrations: Map<string, Integration> = new Map();
  private healthMonitor: HealthMonitor;
  private messageQueue: MessageQueue;
  private transformEngine: TransformEngine;

  constructor(
    private config: IntegrationHubConfig,
    private identityService: IdentityService
  ) {
    super();
    this.healthMonitor = new HealthMonitor();
    this.messageQueue = new MessageQueue(config.queue);
    this.transformEngine = new TransformEngine();
    this.initializeIntegrations();
  }

  private initializeIntegrations(): void {
    for (const integrationConfig of this.config.integrations) {
      if (integrationConfig.enabled) {
        const integration = this.createIntegration(integrationConfig);
        this.integrations.set(integrationConfig.id, integration);
        this.setupHealthCheck(integration);
      }
    }
  }

  private createIntegration(config: IntegrationConfig): Integration {
    switch (config.type) {
      case 'specimen-management':
        return new SpecimenManagementIntegration(config, this.identityService);
      case 'access-control':
        return new AccessControlIntegration(config, this.identityService);
      case 'consent-management':
        return new ConsentManagementIntegration(config, this.identityService);
      case 'facility-monitoring':
        return new FacilityMonitoringIntegration(config, this.identityService);
      case 'billing':
        return new BillingIntegration(config, this.identityService);
      case 'regulatory-reporting':
        return new RegulatoryReportingIntegration(config, this.identityService);
      case 'external-registry':
        return new ExternalRegistryIntegration(config, this.identityService);
      default:
        throw new Error(`Unknown integration type: ${config.type}`);
    }
  }

  private setupHealthCheck(integration: Integration): void {
    this.healthMonitor.register(integration.config.id, async () => {
      return integration.healthCheck();
    });
  }

  async start(): Promise<void> {
    for (const [id, integration] of this.integrations) {
      try {
        await integration.connect();
        this.emit('integrationConnected', id);
      } catch (error) {
        this.emit('integrationError', id, error);
      }
    }

    this.healthMonitor.startMonitoring();
    this.startMessageProcessing();
  }

  async stop(): Promise<void> {
    this.healthMonitor.stopMonitoring();

    for (const [id, integration] of this.integrations) {
      try {
        await integration.disconnect();
      } catch (error) {
        console.error(`Error disconnecting ${id}:`, error);
      }
    }
  }

  private async startMessageProcessing(): Promise<void> {
    while (true) {
      const message = await this.messageQueue.dequeue();
      if (message) {
        await this.processMessage(message);
      }
      await this.sleep(100);
    }
  }

  private async processMessage(message: IntegrationMessage): Promise<void> {
    const integration = this.integrations.get(message.targetIntegration);
    if (!integration) {
      console.error(`Integration not found: ${message.targetIntegration}`);
      return;
    }

    try {
      const transformed = this.transformEngine.transform(
        message.payload,
        integration.config.transformations
      );

      await integration.send(transformed);
      this.emit('messageSent', message.id, message.targetIntegration);
    } catch (error) {
      await this.handleMessageError(message, error);
    }
  }

  private async handleMessageError(message: IntegrationMessage, error: unknown): Promise<void> {
    message.retryCount = (message.retryCount || 0) + 1;
    const integration = this.integrations.get(message.targetIntegration);

    if (integration && message.retryCount <= integration.config.retryPolicy.maxRetries) {
      const delay = this.calculateRetryDelay(message.retryCount, integration.config.retryPolicy);
      setTimeout(() => this.messageQueue.enqueue(message), delay);
    } else {
      this.emit('messageDeadLetter', message, error);
    }
  }

  private calculateRetryDelay(retryCount: number, policy: RetryPolicy): number {
    const delay = policy.initialDelay * Math.pow(policy.backoffMultiplier, retryCount - 1);
    return Math.min(delay, policy.maxDelay);
  }

  async sendToIntegration(integrationId: string, payload: any): Promise<void> {
    const message: IntegrationMessage = {
      id: `MSG-${Date.now()}`,
      targetIntegration: integrationId,
      payload,
      createdAt: new Date(),
      retryCount: 0
    };

    await this.messageQueue.enqueue(message);
  }

  getIntegrationStatus(): Map<string, IntegrationStatus> {
    const statuses = new Map<string, IntegrationStatus>();
    for (const [id, integration] of this.integrations) {
      statuses.set(id, integration.getStatus());
    }
    return statuses;
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

interface IntegrationHubConfig {
  integrations: IntegrationConfig[];
  queue: QueueConfig;
}

interface IntegrationMessage {
  id: string;
  targetIntegration: string;
  payload: any;
  createdAt: Date;
  retryCount: number;
}

// Base integration class
abstract class Integration {
  protected status: IntegrationStatus = 'disconnected';
  protected lastSync?: Date;

  constructor(
    public config: IntegrationConfig,
    protected identityService: IdentityService
  ) {}

  abstract connect(): Promise<void>;
  abstract disconnect(): Promise<void>;
  abstract send(data: any): Promise<void>;
  abstract receive(): Promise<any>;
  abstract healthCheck(): Promise<HealthCheckResult>;

  getStatus(): IntegrationStatus {
    return this.status;
  }

  getLastSync(): Date | undefined {
    return this.lastSync;
  }
}

interface HealthCheckResult {
  healthy: boolean;
  latency?: number;
  message?: string;
}
```

### Specimen Management Integration

```typescript
// Integration with specimen management systems
interface SpecimenData {
  specimenId: string;
  subjectId: string;
  type: string;
  collectionDate: Date;
  location: StorageLocation;
  status: string;
  metadata: Record<string, any>;
}

interface StorageLocation {
  facilityId: string;
  tankId: string;
  rackId: string;
  boxId: string;
  position: string;
}

interface SpecimenEvent {
  eventType: 'created' | 'moved' | 'accessed' | 'disposed' | 'transferred';
  specimenId: string;
  timestamp: Date;
  performedBy: string;
  details: Record<string, any>;
}

class SpecimenManagementIntegration extends Integration {
  private client: SpecimenManagementClient;
  private eventSubscription?: EventSubscription;

  async connect(): Promise<void> {
    this.client = new SpecimenManagementClient(this.config.endpoint);
    await this.client.authenticate(this.config.authentication);

    if (this.config.syncDirection !== 'outbound') {
      this.eventSubscription = await this.subscribeToEvents();
    }

    this.status = 'connected';
  }

  async disconnect(): Promise<void> {
    if (this.eventSubscription) {
      await this.eventSubscription.unsubscribe();
    }
    await this.client.close();
    this.status = 'disconnected';
  }

  private async subscribeToEvents(): Promise<EventSubscription> {
    return this.client.subscribe('specimen-events', async (event: SpecimenEvent) => {
      await this.handleSpecimenEvent(event);
    });
  }

  private async handleSpecimenEvent(event: SpecimenEvent): Promise<void> {
    switch (event.eventType) {
      case 'created':
        await this.handleSpecimenCreated(event);
        break;
      case 'accessed':
        await this.handleSpecimenAccessed(event);
        break;
      case 'transferred':
        await this.handleSpecimenTransferred(event);
        break;
    }
  }

  private async handleSpecimenCreated(event: SpecimenEvent): Promise<void> {
    // Link specimen to subject identity
    const specimen = await this.client.getSpecimen(event.specimenId);

    await this.identityService.linkSpecimen(specimen.subjectId, {
      specimenId: specimen.specimenId,
      type: specimen.type,
      collectionDate: specimen.collectionDate,
      linkedAt: new Date()
    });
  }

  private async handleSpecimenAccessed(event: SpecimenEvent): Promise<void> {
    // Verify identity before specimen access
    const specimen = await this.client.getSpecimen(event.specimenId);
    const subject = await this.identityService.getSubject(specimen.subjectId);

    if (subject) {
      await this.identityService.recordAccess({
        subjectId: subject.id,
        specimenId: event.specimenId,
        accessedBy: event.performedBy,
        accessedAt: event.timestamp,
        purpose: event.details.purpose
      });
    }
  }

  private async handleSpecimenTransferred(event: SpecimenEvent): Promise<void> {
    // Handle custody chain for identity continuity
    const specimen = await this.client.getSpecimen(event.specimenId);

    await this.identityService.recordCustodyChange({
      subjectId: specimen.subjectId,
      specimenId: event.specimenId,
      fromFacility: event.details.fromFacility,
      toFacility: event.details.toFacility,
      transferredAt: event.timestamp,
      transferredBy: event.performedBy
    });
  }

  async send(data: any): Promise<void> {
    // Send identity updates to specimen management
    if (data.type === 'identity-update') {
      await this.client.updateSubjectIdentity(data.subjectId, {
        name: data.name,
        identifiers: data.identifiers,
        status: data.status
      });
    }
  }

  async receive(): Promise<any> {
    // Pull specimen data for identity reconciliation
    return this.client.getRecentSpecimens({
      since: this.lastSync,
      limit: 100
    });
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const start = Date.now();
    try {
      await this.client.ping();
      return {
        healthy: true,
        latency: Date.now() - start
      };
    } catch (error) {
      return {
        healthy: false,
        message: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  }

  // Specimen-specific operations
  async verifySpecimenOwnership(
    specimenId: string,
    subjectId: string
  ): Promise<OwnershipVerification> {
    const specimen = await this.client.getSpecimen(specimenId);

    if (!specimen) {
      return { verified: false, reason: 'Specimen not found' };
    }

    if (specimen.subjectId !== subjectId) {
      // Check for identity linking
      const isLinked = await this.identityService.areIdentitiesLinked(
        specimen.subjectId,
        subjectId
      );

      if (!isLinked) {
        return { verified: false, reason: 'Subject does not own specimen' };
      }
    }

    return {
      verified: true,
      specimen: specimen,
      verifiedAt: new Date()
    };
  }

  async getSubjectSpecimens(subjectId: string): Promise<SpecimenSummary[]> {
    const linkedIdentities = await this.identityService.getLinkedIdentities(subjectId);
    const allIds = [subjectId, ...linkedIdentities.map(l => l.id)];

    const specimens: SpecimenSummary[] = [];
    for (const id of allIds) {
      const subjectSpecimens = await this.client.getSpecimensBySubject(id);
      specimens.push(...subjectSpecimens.map(s => ({
        specimenId: s.specimenId,
        type: s.type,
        status: s.status,
        location: s.location,
        linkedSubjectId: id
      })));
    }

    return specimens;
  }
}

interface OwnershipVerification {
  verified: boolean;
  reason?: string;
  specimen?: SpecimenData;
  verifiedAt?: Date;
}

interface SpecimenSummary {
  specimenId: string;
  type: string;
  status: string;
  location: StorageLocation;
  linkedSubjectId: string;
}
```

### Access Control Integration

```typescript
// Physical and logical access control integration
interface AccessControlConfig {
  zones: AccessZone[];
  roles: AccessRole[];
  schedules: AccessSchedule[];
}

interface AccessZone {
  id: string;
  name: string;
  securityLevel: 'low' | 'medium' | 'high' | 'critical';
  requiredVerification: VerificationLevel;
  parentZone?: string;
}

interface AccessRole {
  id: string;
  name: string;
  permissions: string[];
  allowedZones: string[];
  restrictions: AccessRestriction[];
}

interface AccessRestriction {
  type: 'time' | 'date' | 'frequency' | 'duration';
  parameters: Record<string, any>;
}

interface AccessSchedule {
  id: string;
  name: string;
  dayOfWeek: number[];
  startTime: string;
  endTime: string;
  exceptions: Date[];
}

interface AccessRequest {
  requestId: string;
  subjectId: string;
  zoneId: string;
  requestedAt: Date;
  method: 'card' | 'biometric' | 'pin' | 'mobile';
  deviceId: string;
}

interface AccessDecision {
  allowed: boolean;
  reason?: string;
  additionalVerificationRequired?: VerificationLevel;
  expiresAt?: Date;
}

class AccessControlIntegration extends Integration {
  private accessClient: AccessControlClient;
  private decisionCache: Map<string, CachedDecision> = new Map();

  async connect(): Promise<void> {
    this.accessClient = new AccessControlClient(this.config.endpoint);
    await this.accessClient.authenticate(this.config.authentication);
    await this.syncAccessPolicies();
    this.status = 'connected';
  }

  async disconnect(): Promise<void> {
    await this.accessClient.close();
    this.status = 'disconnected';
  }

  private async syncAccessPolicies(): Promise<void> {
    const policies = await this.accessClient.getPolicies();
    // Cache policies for fast access decisions
  }

  async send(data: any): Promise<void> {
    if (data.type === 'identity-status-change') {
      await this.handleIdentityStatusChange(data);
    } else if (data.type === 'verification-result') {
      await this.handleVerificationResult(data);
    }
  }

  private async handleIdentityStatusChange(data: any): Promise<void> {
    const { subjectId, newStatus } = data;

    if (newStatus === 'suspended' || newStatus === 'inactive') {
      // Revoke all access
      await this.accessClient.revokeAllAccess(subjectId);
    } else if (newStatus === 'active') {
      // Restore access based on roles
      const subject = await this.identityService.getSubject(subjectId);
      if (subject) {
        await this.restoreAccess(subject);
      }
    }
  }

  private async handleVerificationResult(data: any): Promise<void> {
    const { subjectId, level, result } = data;

    if (result === 'completed') {
      // Update access level based on verification
      await this.accessClient.updateVerificationLevel(subjectId, level);
    }
  }

  private async restoreAccess(subject: Subject): Promise<void> {
    // Restore access based on subject's roles and verification status
    const verification = await this.identityService.getCurrentVerification(subject.id);

    if (verification) {
      await this.accessClient.grantAccess(subject.id, {
        verificationLevel: verification.level,
        validUntil: verification.expiresAt
      });
    }
  }

  async receive(): Promise<any> {
    return this.accessClient.getAccessLogs({
      since: this.lastSync,
      limit: 1000
    });
  }

  async healthCheck(): Promise<HealthCheckResult> {
    const start = Date.now();
    try {
      await this.accessClient.ping();
      return { healthy: true, latency: Date.now() - start };
    } catch (error) {
      return { healthy: false, message: error instanceof Error ? error.message : 'Unknown' };
    }
  }

  // Access decision making
  async evaluateAccess(request: AccessRequest): Promise<AccessDecision> {
    // Check cache first
    const cacheKey = `${request.subjectId}-${request.zoneId}`;
    const cached = this.decisionCache.get(cacheKey);
    if (cached && cached.expiresAt > new Date()) {
      return cached.decision;
    }

    // Get subject and zone information
    const subject = await this.identityService.getSubject(request.subjectId);
    if (!subject) {
      return { allowed: false, reason: 'Subject not found' };
    }

    if (subject.status !== 'active') {
      return { allowed: false, reason: `Subject status: ${subject.status}` };
    }

    const zone = await this.accessClient.getZone(request.zoneId);
    if (!zone) {
      return { allowed: false, reason: 'Zone not found' };
    }

    // Check verification level
    const verification = await this.identityService.getCurrentVerification(subject.id);
    if (!verification || this.isVerificationInsufficient(verification, zone)) {
      return {
        allowed: false,
        reason: 'Insufficient verification level',
        additionalVerificationRequired: zone.requiredVerification
      };
    }

    // Check role permissions
    const hasPermission = await this.checkRolePermissions(subject.id, zone.id);
    if (!hasPermission) {
      return { allowed: false, reason: 'No permission for this zone' };
    }

    // Check schedule restrictions
    const withinSchedule = await this.checkScheduleRestrictions(subject.id, zone.id);
    if (!withinSchedule) {
      return { allowed: false, reason: 'Outside allowed schedule' };
    }

    const decision: AccessDecision = {
      allowed: true,
      expiresAt: this.calculateAccessExpiration(zone)
    };

    // Cache the decision
    this.decisionCache.set(cacheKey, {
      decision,
      expiresAt: new Date(Date.now() + 5 * 60 * 1000) // 5 minute cache
    });

    return decision;
  }

  private isVerificationInsufficient(
    verification: VerificationRecord,
    zone: AccessZone
  ): boolean {
    const levels: VerificationLevel[] = ['basic', 'standard', 'enhanced', 'maximum'];
    const subjectLevel = levels.indexOf(verification.level);
    const requiredLevel = levels.indexOf(zone.requiredVerification);
    return subjectLevel < requiredLevel;
  }

  private async checkRolePermissions(subjectId: string, zoneId: string): Promise<boolean> {
    const roles = await this.accessClient.getSubjectRoles(subjectId);
    return roles.some(role => role.allowedZones.includes(zoneId));
  }

  private async checkScheduleRestrictions(subjectId: string, zoneId: string): Promise<boolean> {
    const schedules = await this.accessClient.getSchedules(subjectId, zoneId);
    const now = new Date();

    return schedules.some(schedule => {
      const currentDay = now.getDay();
      if (!schedule.dayOfWeek.includes(currentDay)) return false;

      const currentTime = now.toTimeString().substring(0, 5);
      return currentTime >= schedule.startTime && currentTime <= schedule.endTime;
    });
  }

  private calculateAccessExpiration(zone: AccessZone): Date {
    const durations: Record<string, number> = {
      low: 8 * 60 * 60 * 1000,      // 8 hours
      medium: 4 * 60 * 60 * 1000,   // 4 hours
      high: 1 * 60 * 60 * 1000,     // 1 hour
      critical: 15 * 60 * 1000      // 15 minutes
    };

    return new Date(Date.now() + durations[zone.securityLevel]);
  }

  async logAccessAttempt(
    request: AccessRequest,
    decision: AccessDecision
  ): Promise<void> {
    await this.accessClient.logAccess({
      requestId: request.requestId,
      subjectId: request.subjectId,
      zoneId: request.zoneId,
      timestamp: request.requestedAt,
      method: request.method,
      deviceId: request.deviceId,
      allowed: decision.allowed,
      reason: decision.reason
    });

    // Also record in identity service
    await this.identityService.recordAccessAttempt({
      subjectId: request.subjectId,
      zoneId: request.zoneId,
      allowed: decision.allowed,
      timestamp: request.requestedAt
    });
  }
}

interface CachedDecision {
  decision: AccessDecision;
  expiresAt: Date;
}

interface VerificationRecord {
  level: VerificationLevel;
  completedAt: Date;
  expiresAt: Date;
}
```

### Consent Management Integration

```typescript
// Integration with consent management systems
interface ConsentRecord {
  id: string;
  subjectId: string;
  consentType: ConsentType;
  scope: string[];
  grantedAt: Date;
  expiresAt?: Date;
  revokedAt?: Date;
  witnessedBy?: string;
  documentHash?: string;
}

type ConsentType =
  | 'storage'
  | 'research'
  | 'transfer'
  | 'disposal'
  | 'data-sharing'
  | 'third-party-access';

interface ConsentRequest {
  subjectId: string;
  consentType: ConsentType;
  scope: string[];
  purpose: string;
  requestedBy: string;
}

interface ConsentValidation {
  valid: boolean;
  consent?: ConsentRecord;
  reason?: string;
}

class ConsentManagementIntegration extends Integration {
  private consentClient: ConsentManagementClient;

  async connect(): Promise<void> {
    this.consentClient = new ConsentManagementClient(this.config.endpoint);
    await this.consentClient.authenticate(this.config.authentication);
    this.status = 'connected';
  }

  async disconnect(): Promise<void> {
    await this.consentClient.close();
    this.status = 'disconnected';
  }

  async send(data: any): Promise<void> {
    if (data.type === 'identity-verification-completed') {
      // After verification, check for pending consent requests
      await this.processPendingConsents(data.subjectId);
    }
  }

  private async processPendingConsents(subjectId: string): Promise<void> {
    const pending = await this.consentClient.getPendingConsents(subjectId);

    for (const request of pending) {
      // Notify subject of pending consents
      await this.identityService.notifySubject(subjectId, {
        type: 'pending-consent',
        consentRequest: request
      });
    }
  }

  async receive(): Promise<any> {
    return this.consentClient.getRecentConsents({
      since: this.lastSync,
      limit: 100
    });
  }

  async healthCheck(): Promise<HealthCheckResult> {
    try {
      await this.consentClient.ping();
      return { healthy: true };
    } catch (error) {
      return { healthy: false, message: error instanceof Error ? error.message : 'Unknown' };
    }
  }

  // Consent validation
  async validateConsent(
    subjectId: string,
    consentType: ConsentType,
    scope: string[]
  ): Promise<ConsentValidation> {
    // Get subject and linked identities
    const linkedIds = await this.identityService.getLinkedIdentities(subjectId);
    const allIds = [subjectId, ...linkedIds.map(l => l.id)];

    // Check consent for all linked identities
    for (const id of allIds) {
      const consents = await this.consentClient.getActiveConsents(id, consentType);

      for (const consent of consents) {
        if (this.consentCoversScope(consent, scope)) {
          return {
            valid: true,
            consent
          };
        }
      }
    }

    return {
      valid: false,
      reason: 'No valid consent found for requested scope'
    };
  }

  private consentCoversScope(consent: ConsentRecord, requestedScope: string[]): boolean {
    return requestedScope.every(s => consent.scope.includes(s));
  }

  async requestConsent(request: ConsentRequest): Promise<ConsentRequestResult> {
    // Verify subject identity first
    const subject = await this.identityService.getSubject(request.subjectId);
    if (!subject) {
      return {
        success: false,
        error: 'Subject not found'
      };
    }

    // Check if subject can provide consent
    const canConsent = await this.canSubjectConsent(subject);
    if (!canConsent.allowed) {
      return {
        success: false,
        error: canConsent.reason,
        alternativeApprover: canConsent.alternativeApprover
      };
    }

    // Create consent request
    const consentRequest = await this.consentClient.createRequest({
      subjectId: request.subjectId,
      consentType: request.consentType,
      scope: request.scope,
      purpose: request.purpose,
      requestedBy: request.requestedBy,
      requestedAt: new Date()
    });

    return {
      success: true,
      requestId: consentRequest.id,
      status: 'pending'
    };
  }

  private async canSubjectConsent(subject: Subject): Promise<ConsentCapability> {
    // Check subject type
    if (subject.type === 'minor') {
      const guardian = await this.identityService.getGuardian(subject.id);
      if (guardian) {
        return {
          allowed: false,
          reason: 'Subject is a minor, guardian consent required',
          alternativeApprover: guardian.id
        };
      }
    }

    if (subject.type === 'incapacitated') {
      const legalRepresentative = await this.identityService.getLegalRepresentative(subject.id);
      if (legalRepresentative) {
        return {
          allowed: false,
          reason: 'Subject is incapacitated, legal representative consent required',
          alternativeApprover: legalRepresentative.id
        };
      }
    }

    if (subject.type === 'posthumous') {
      const executor = await this.identityService.getEstateExecutor(subject.id);
      if (executor) {
        return {
          allowed: false,
          reason: 'Subject is deceased, estate executor consent required',
          alternativeApprover: executor.id
        };
      }
    }

    // Check verification status
    const verification = await this.identityService.getCurrentVerification(subject.id);
    if (!verification) {
      return {
        allowed: false,
        reason: 'Identity verification required before consent'
      };
    }

    return { allowed: true };
  }

  async recordConsentWithVerification(
    subjectId: string,
    consentType: ConsentType,
    scope: string[],
    verificationSessionId: string
  ): Promise<ConsentRecord> {
    // Verify the session
    const session = await this.identityService.getVerificationSession(verificationSessionId);
    if (!session || session.status !== 'completed') {
      throw new Error('Valid verification session required');
    }

    // Record consent with verification evidence
    const consent = await this.consentClient.recordConsent({
      subjectId,
      consentType,
      scope,
      grantedAt: new Date(),
      verificationSessionId,
      verificationLevel: session.level
    });

    return consent;
  }
}

interface ConsentRequestResult {
  success: boolean;
  requestId?: string;
  status?: string;
  error?: string;
  alternativeApprover?: string;
}

interface ConsentCapability {
  allowed: boolean;
  reason?: string;
  alternativeApprover?: string;
}
```

### External Registry Integration

```typescript
// Integration with external identity registries
interface ExternalRegistryConfig {
  registryType: 'national' | 'medical' | 'donor' | 'research';
  country: string;
  apiVersion: string;
  supportedOperations: RegistryOperation[];
}

type RegistryOperation =
  | 'lookup'
  | 'verify'
  | 'register'
  | 'update'
  | 'notify';

interface RegistryLookupRequest {
  identifierType: string;
  identifierValue: string;
  includeHistory?: boolean;
}

interface RegistryLookupResult {
  found: boolean;
  identity?: ExternalIdentity;
  lastUpdated?: Date;
  registrySource: string;
}

interface ExternalIdentity {
  registryId: string;
  identifiers: { type: string; value: string }[];
  name: {
    given: string;
    family: string;
  };
  dateOfBirth?: Date;
  status: string;
  registeredAt: Date;
  metadata: Record<string, any>;
}

class ExternalRegistryIntegration extends Integration {
  private registryClient: ExternalRegistryClient;
  private rateLimiter: RateLimiter;

  constructor(
    config: IntegrationConfig,
    identityService: IdentityService,
    private registryConfig: ExternalRegistryConfig
  ) {
    super(config, identityService);
    this.rateLimiter = new RateLimiter({
      maxRequests: 100,
      windowMs: 60000 // 1 minute
    });
  }

  async connect(): Promise<void> {
    this.registryClient = new ExternalRegistryClient(
      this.config.endpoint,
      this.registryConfig
    );
    await this.registryClient.authenticate(this.config.authentication);
    this.status = 'connected';
  }

  async disconnect(): Promise<void> {
    await this.registryClient.close();
    this.status = 'disconnected';
  }

  async send(data: any): Promise<void> {
    if (!this.registryConfig.supportedOperations.includes('notify')) {
      return;
    }

    if (data.type === 'identity-registered') {
      await this.notifyRegistry(data);
    }
  }

  private async notifyRegistry(data: any): Promise<void> {
    await this.rateLimiter.acquire();

    try {
      await this.registryClient.notify({
        eventType: 'new-registration',
        subjectId: data.subjectId,
        timestamp: new Date(),
        facilityId: data.facilityId
      });
    } catch (error) {
      console.error('Failed to notify registry:', error);
    }
  }

  async receive(): Promise<any> {
    // Pull updates from registry
    return this.registryClient.getUpdates({
      since: this.lastSync,
      limit: 100
    });
  }

  async healthCheck(): Promise<HealthCheckResult> {
    try {
      const status = await this.registryClient.getStatus();
      return {
        healthy: status.available,
        message: status.message
      };
    } catch (error) {
      return {
        healthy: false,
        message: error instanceof Error ? error.message : 'Unknown'
      };
    }
  }

  // Registry lookup operations
  async lookupIdentity(request: RegistryLookupRequest): Promise<RegistryLookupResult> {
    if (!this.registryConfig.supportedOperations.includes('lookup')) {
      throw new Error('Lookup operation not supported by this registry');
    }

    await this.rateLimiter.acquire();

    try {
      const result = await this.registryClient.lookup({
        identifierType: request.identifierType,
        identifierValue: request.identifierValue,
        includeHistory: request.includeHistory
      });

      return {
        found: result !== null,
        identity: result,
        lastUpdated: result?.registeredAt,
        registrySource: this.registryConfig.registryType
      };
    } catch (error) {
      throw new RegistryLookupError(
        `Failed to lookup in ${this.registryConfig.registryType} registry`,
        error
      );
    }
  }

  async verifyAgainstRegistry(subjectId: string): Promise<RegistryVerification> {
    if (!this.registryConfig.supportedOperations.includes('verify')) {
      throw new Error('Verify operation not supported by this registry');
    }

    const subject = await this.identityService.getSubject(subjectId);
    if (!subject) {
      return {
        verified: false,
        reason: 'Subject not found'
      };
    }

    // Find matching identifier for registry
    const identifier = this.findMatchingIdentifier(subject);
    if (!identifier) {
      return {
        verified: false,
        reason: 'No matching identifier for registry lookup'
      };
    }

    const lookupResult = await this.lookupIdentity({
      identifierType: identifier.type,
      identifierValue: identifier.value
    });

    if (!lookupResult.found || !lookupResult.identity) {
      return {
        verified: false,
        reason: 'Identity not found in registry'
      };
    }

    // Compare identity data
    const matchScore = this.compareIdentities(subject, lookupResult.identity);

    return {
      verified: matchScore >= 0.9,
      matchScore,
      registryIdentity: lookupResult.identity,
      verifiedAt: new Date()
    };
  }

  private findMatchingIdentifier(subject: Subject): { type: string; value: string } | null {
    const identifierPriority: Record<string, string[]> = {
      national: ['national-id', 'passport', 'drivers-license'],
      medical: ['medical-record', 'national-id'],
      donor: ['donor-id', 'national-id'],
      research: ['internal', 'national-id']
    };

    const priority = identifierPriority[this.registryConfig.registryType] || [];

    for (const type of priority) {
      const identifier = subject.identifiers.find(id => id.type === type);
      if (identifier) {
        return identifier;
      }
    }

    return null;
  }

  private compareIdentities(subject: Subject, external: ExternalIdentity): number {
    let score = 0;
    let checks = 0;

    // Name comparison
    if (subject.profile.legalName && external.name) {
      checks++;
      const nameMatch = this.compareName(subject.profile.legalName, external.name);
      score += nameMatch;
    }

    // Date of birth comparison
    if (subject.profile.dateOfBirth && external.dateOfBirth) {
      checks++;
      if (this.isSameDate(subject.profile.dateOfBirth, external.dateOfBirth)) {
        score += 1;
      }
    }

    // Identifier comparison
    for (const extId of external.identifiers) {
      const localId = subject.identifiers.find(id => id.type === extId.type);
      if (localId) {
        checks++;
        if (localId.value === extId.value) {
          score += 1;
        }
      }
    }

    return checks > 0 ? score / checks : 0;
  }

  private compareName(local: any, external: { given: string; family: string }): number {
    const localGiven = (local.given || local.firstName || '').toLowerCase();
    const localFamily = (local.family || local.lastName || '').toLowerCase();
    const extGiven = external.given.toLowerCase();
    const extFamily = external.family.toLowerCase();

    let score = 0;
    if (localGiven === extGiven) score += 0.5;
    if (localFamily === extFamily) score += 0.5;

    return score;
  }

  private isSameDate(date1: Date, date2: Date): boolean {
    return date1.toDateString() === date2.toDateString();
  }
}

interface RegistryVerification {
  verified: boolean;
  matchScore?: number;
  registryIdentity?: ExternalIdentity;
  verifiedAt?: Date;
  reason?: string;
}

class RegistryLookupError extends Error {
  constructor(message: string, public cause?: unknown) {
    super(message);
    this.name = 'RegistryLookupError';
  }
}
```

## Summary

This chapter covered:

1. **Integration Architecture**: Hub-based framework for managing multiple system integrations
2. **Specimen Management**: Bidirectional sync with specimen tracking systems
3. **Access Control**: Physical and logical access with verification requirements
4. **Consent Management**: Consent validation and recording with identity verification
5. **External Registries**: National and medical registry lookups and verification

The next chapter covers security frameworks for protecting identity data.
