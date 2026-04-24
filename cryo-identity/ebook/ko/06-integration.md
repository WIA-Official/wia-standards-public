# 제6장: 시스템 통합

## 개요

냉동보존 신원 관리 시스템은 표본 관리, 접근 제어, 동의 관리, 외부 규제 시스템을 포함한 다양한 시설 시스템과 원활하게 통합되어야 합니다. 이 장에서는 포괄적인 통합 패턴과 구현을 다룹니다.

## 통합 아키텍처

### 핵심 통합 프레임워크

```typescript
import { EventEmitter } from 'events';

// 통합 유형
type IntegrationType =
  | 'specimen-management'   // 표본 관리
  | 'access-control'        // 접근 제어
  | 'consent-management'    // 동의 관리
  | 'facility-monitoring'   // 시설 모니터링
  | 'billing'               // 청구
  | 'regulatory-reporting'  // 규제 보고
  | 'external-registry';    // 외부 레지스트리

type IntegrationStatus =
  | 'connected'    // 연결됨
  | 'disconnected' // 연결 끊김
  | 'degraded'     // 성능 저하
  | 'error';       // 오류

type SyncDirection =
  | 'inbound'       // 수신
  | 'outbound'      // 발신
  | 'bidirectional'; // 양방향

// 통합 설정
interface IntegrationConfig {
  id: string;
  name: string;
  type: IntegrationType;
  enabled: boolean;
  endpoint: EndpointConfig;
  authentication: AuthConfig;
  syncDirection: SyncDirection;
  syncInterval: number; // 밀리초
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

// 메인 통합 허브
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
        throw new Error(`알 수 없는 통합 유형: ${config.type}`);
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
        console.error(`${id} 연결 해제 중 오류:`, error);
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
      console.error(`통합을 찾을 수 없습니다: ${message.targetIntegration}`);
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
```

### 표본 관리 통합

```typescript
// 표본 관리 시스템과의 통합
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
    // 표본을 대상자 신원에 연결
    const specimen = await this.client.getSpecimen(event.specimenId);

    await this.identityService.linkSpecimen(specimen.subjectId, {
      specimenId: specimen.specimenId,
      type: specimen.type,
      collectionDate: specimen.collectionDate,
      linkedAt: new Date()
    });
  }

  private async handleSpecimenAccessed(event: SpecimenEvent): Promise<void> {
    // 표본 접근 전 신원 확인
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
    // 신원 연속성을 위한 보관 체인 처리
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
    // 표본 관리로 신원 업데이트 전송
    if (data.type === 'identity-update') {
      await this.client.updateSubjectIdentity(data.subjectId, {
        name: data.name,
        identifiers: data.identifiers,
        status: data.status
      });
    }
  }

  async receive(): Promise<any> {
    // 신원 조정을 위한 표본 데이터 가져오기
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
        message: error instanceof Error ? error.message : '알 수 없는 오류'
      };
    }
  }

  // 표본별 작업
  async verifySpecimenOwnership(
    specimenId: string,
    subjectId: string
  ): Promise<OwnershipVerification> {
    const specimen = await this.client.getSpecimen(specimenId);

    if (!specimen) {
      return { verified: false, reason: '표본을 찾을 수 없습니다' };
    }

    if (specimen.subjectId !== subjectId) {
      // 신원 연결 확인
      const isLinked = await this.identityService.areIdentitiesLinked(
        specimen.subjectId,
        subjectId
      );

      if (!isLinked) {
        return { verified: false, reason: '대상자가 표본의 소유자가 아닙니다' };
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

### 접근 제어 통합

```typescript
// 물리적 및 논리적 접근 제어 통합
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
    // 빠른 접근 결정을 위해 정책 캐시
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
      // 모든 접근 권한 취소
      await this.accessClient.revokeAllAccess(subjectId);
    } else if (newStatus === 'active') {
      // 역할에 따라 접근 권한 복원
      const subject = await this.identityService.getSubject(subjectId);
      if (subject) {
        await this.restoreAccess(subject);
      }
    }
  }

  private async handleVerificationResult(data: any): Promise<void> {
    const { subjectId, level, result } = data;

    if (result === 'completed') {
      // 인증에 따라 접근 레벨 업데이트
      await this.accessClient.updateVerificationLevel(subjectId, level);
    }
  }

  private async restoreAccess(subject: Subject): Promise<void> {
    // 대상자의 역할 및 인증 상태에 따라 접근 권한 복원
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
      return { healthy: false, message: error instanceof Error ? error.message : '알 수 없음' };
    }
  }

  // 접근 결정 처리
  async evaluateAccess(request: AccessRequest): Promise<AccessDecision> {
    // 먼저 캐시 확인
    const cacheKey = `${request.subjectId}-${request.zoneId}`;
    const cached = this.decisionCache.get(cacheKey);
    if (cached && cached.expiresAt > new Date()) {
      return cached.decision;
    }

    // 대상자 및 구역 정보 조회
    const subject = await this.identityService.getSubject(request.subjectId);
    if (!subject) {
      return { allowed: false, reason: '대상자를 찾을 수 없습니다' };
    }

    if (subject.status !== 'active') {
      return { allowed: false, reason: `대상자 상태: ${subject.status}` };
    }

    const zone = await this.accessClient.getZone(request.zoneId);
    if (!zone) {
      return { allowed: false, reason: '구역을 찾을 수 없습니다' };
    }

    // 인증 레벨 확인
    const verification = await this.identityService.getCurrentVerification(subject.id);
    if (!verification || this.isVerificationInsufficient(verification, zone)) {
      return {
        allowed: false,
        reason: '인증 레벨 부족',
        additionalVerificationRequired: zone.requiredVerification
      };
    }

    // 역할 권한 확인
    const hasPermission = await this.checkRolePermissions(subject.id, zone.id);
    if (!hasPermission) {
      return { allowed: false, reason: '이 구역에 대한 권한이 없습니다' };
    }

    // 스케줄 제한 확인
    const withinSchedule = await this.checkScheduleRestrictions(subject.id, zone.id);
    if (!withinSchedule) {
      return { allowed: false, reason: '허용된 시간 외 접근 시도' };
    }

    const decision: AccessDecision = {
      allowed: true,
      expiresAt: this.calculateAccessExpiration(zone)
    };

    // 결정 캐시
    this.decisionCache.set(cacheKey, {
      decision,
      expiresAt: new Date(Date.now() + 5 * 60 * 1000) // 5분 캐시
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
      low: 8 * 60 * 60 * 1000,      // 8시간
      medium: 4 * 60 * 60 * 1000,   // 4시간
      high: 1 * 60 * 60 * 1000,     // 1시간
      critical: 15 * 60 * 1000      // 15분
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

    // 신원 서비스에도 기록
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

### 동의 관리 통합

```typescript
// 동의 관리 시스템과의 통합
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
  | 'storage'          // 보관
  | 'research'         // 연구
  | 'transfer'         // 이전
  | 'disposal'         // 폐기
  | 'data-sharing'     // 데이터 공유
  | 'third-party-access'; // 제3자 접근

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
      // 인증 후 보류 중인 동의 요청 확인
      await this.processPendingConsents(data.subjectId);
    }
  }

  private async processPendingConsents(subjectId: string): Promise<void> {
    const pending = await this.consentClient.getPendingConsents(subjectId);

    for (const request of pending) {
      // 보류 중인 동의에 대해 대상자에게 알림
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
      return { healthy: false, message: error instanceof Error ? error.message : '알 수 없음' };
    }
  }

  // 동의 검증
  async validateConsent(
    subjectId: string,
    consentType: ConsentType,
    scope: string[]
  ): Promise<ConsentValidation> {
    // 대상자 및 연결된 신원 조회
    const linkedIds = await this.identityService.getLinkedIdentities(subjectId);
    const allIds = [subjectId, ...linkedIds.map(l => l.id)];

    // 모든 연결된 신원에 대해 동의 확인
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
      reason: '요청된 범위에 대한 유효한 동의를 찾을 수 없습니다'
    };
  }

  private consentCoversScope(consent: ConsentRecord, requestedScope: string[]): boolean {
    return requestedScope.every(s => consent.scope.includes(s));
  }

  async requestConsent(request: ConsentRequest): Promise<ConsentRequestResult> {
    // 먼저 대상자 신원 확인
    const subject = await this.identityService.getSubject(request.subjectId);
    if (!subject) {
      return {
        success: false,
        error: '대상자를 찾을 수 없습니다'
      };
    }

    // 대상자가 동의할 수 있는지 확인
    const canConsent = await this.canSubjectConsent(subject);
    if (!canConsent.allowed) {
      return {
        success: false,
        error: canConsent.reason,
        alternativeApprover: canConsent.alternativeApprover
      };
    }

    // 동의 요청 생성
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
    // 대상자 유형 확인
    if (subject.type === 'minor') {
      const guardian = await this.identityService.getGuardian(subject.id);
      if (guardian) {
        return {
          allowed: false,
          reason: '대상자가 미성년자입니다. 보호자 동의가 필요합니다.',
          alternativeApprover: guardian.id
        };
      }
    }

    if (subject.type === 'incapacitated') {
      const legalRepresentative = await this.identityService.getLegalRepresentative(subject.id);
      if (legalRepresentative) {
        return {
          allowed: false,
          reason: '대상자가 의사결정 무능력 상태입니다. 법정 대리인 동의가 필요합니다.',
          alternativeApprover: legalRepresentative.id
        };
      }
    }

    if (subject.type === 'posthumous') {
      const executor = await this.identityService.getEstateExecutor(subject.id);
      if (executor) {
        return {
          allowed: false,
          reason: '대상자가 사망했습니다. 유산 관리자 동의가 필요합니다.',
          alternativeApprover: executor.id
        };
      }
    }

    // 인증 상태 확인
    const verification = await this.identityService.getCurrentVerification(subject.id);
    if (!verification) {
      return {
        allowed: false,
        reason: '동의 전 신원 인증이 필요합니다'
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
    // 세션 확인
    const session = await this.identityService.getVerificationSession(verificationSessionId);
    if (!session || session.status !== 'completed') {
      throw new Error('유효한 인증 세션이 필요합니다');
    }

    // 인증 증거와 함께 동의 기록
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

### 외부 레지스트리 통합

```typescript
// 외부 신원 레지스트리와의 통합
interface ExternalRegistryConfig {
  registryType: 'national' | 'medical' | 'donor' | 'research';
  country: string;
  apiVersion: string;
  supportedOperations: RegistryOperation[];
}

type RegistryOperation =
  | 'lookup'   // 조회
  | 'verify'   // 검증
  | 'register' // 등록
  | 'update'   // 업데이트
  | 'notify';  // 알림

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
      windowMs: 60000 // 1분
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
      console.error('레지스트리 알림 실패:', error);
    }
  }

  async receive(): Promise<any> {
    // 레지스트리에서 업데이트 가져오기
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
        message: error instanceof Error ? error.message : '알 수 없음'
      };
    }
  }

  // 레지스트리 조회 작업
  async lookupIdentity(request: RegistryLookupRequest): Promise<RegistryLookupResult> {
    if (!this.registryConfig.supportedOperations.includes('lookup')) {
      throw new Error('이 레지스트리는 조회 작업을 지원하지 않습니다');
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
        `${this.registryConfig.registryType} 레지스트리 조회 실패`,
        error
      );
    }
  }

  async verifyAgainstRegistry(subjectId: string): Promise<RegistryVerification> {
    if (!this.registryConfig.supportedOperations.includes('verify')) {
      throw new Error('이 레지스트리는 검증 작업을 지원하지 않습니다');
    }

    const subject = await this.identityService.getSubject(subjectId);
    if (!subject) {
      return {
        verified: false,
        reason: '대상자를 찾을 수 없습니다'
      };
    }

    // 레지스트리에 맞는 식별자 찾기
    const identifier = this.findMatchingIdentifier(subject);
    if (!identifier) {
      return {
        verified: false,
        reason: '레지스트리 조회에 맞는 식별자가 없습니다'
      };
    }

    const lookupResult = await this.lookupIdentity({
      identifierType: identifier.type,
      identifierValue: identifier.value
    });

    if (!lookupResult.found || !lookupResult.identity) {
      return {
        verified: false,
        reason: '레지스트리에서 신원을 찾을 수 없습니다'
      };
    }

    // 신원 데이터 비교
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

    // 이름 비교
    if (subject.profile.legalName && external.name) {
      checks++;
      const nameMatch = this.compareName(subject.profile.legalName, external.name);
      score += nameMatch;
    }

    // 생년월일 비교
    if (subject.profile.dateOfBirth && external.dateOfBirth) {
      checks++;
      if (this.isSameDate(subject.profile.dateOfBirth, external.dateOfBirth)) {
        score += 1;
      }
    }

    // 식별자 비교
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

## 요약

이 장에서 다룬 내용:

1. **통합 아키텍처**: 다중 시스템 통합 관리를 위한 허브 기반 프레임워크
2. **표본 관리**: 표본 추적 시스템과의 양방향 동기화
3. **접근 제어**: 인증 요구사항을 포함한 물리적/논리적 접근 관리
4. **동의 관리**: 신원 인증과 연계한 동의 검증 및 기록
5. **외부 레지스트리**: 국가 및 의료 레지스트리 조회와 검증

다음 장에서는 신원 데이터 보호를 위한 보안 프레임워크를 다룹니다.
