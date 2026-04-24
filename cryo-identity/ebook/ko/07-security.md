# 제7장: 보안 프레임워크

## 개요

냉동보존 시스템의 신원 데이터는 최고 수준의 보안 보호가 필요합니다. 이 장에서는 암호화, 접근 제어, 감사 로깅, 의료 데이터 보호 규정 준수를 포함한 포괄적인 보안 프레임워크를 상세히 다룹니다.

## 보안 아키텍처

### 핵심 보안 프레임워크

```typescript
import { createCipheriv, createDecipheriv, randomBytes, scrypt } from 'crypto';
import { promisify } from 'util';

const scryptAsync = promisify(scrypt);

// 보안 분류 레벨
type SecurityLevel = 'public' | 'internal' | 'confidential' | 'restricted' | 'top-secret';

type DataCategory =
  | 'identity-core'  // 핵심 신원
  | 'biometric'      // 생체정보
  | 'medical'        // 의료정보
  | 'genetic'        // 유전정보
  | 'consent'        // 동의정보
  | 'access-log'     // 접근 로그
  | 'financial';     // 재무정보

// 보안 정책 설정
interface SecurityPolicy {
  id: string;
  name: string;
  dataCategories: DataCategory[];
  securityLevel: SecurityLevel;
  encryption: EncryptionPolicy;
  access: AccessPolicy;
  retention: RetentionPolicy;
  audit: AuditPolicy;
}

interface EncryptionPolicy {
  algorithm: string;
  keyLength: number;
  keyRotationDays: number;
  atRest: boolean;
  inTransit: boolean;
  fieldLevel: FieldEncryptionRule[];
}

interface FieldEncryptionRule {
  field: string;
  algorithm: string;
  searchable: boolean;
  deterministicSearch?: boolean;
}

interface AccessPolicy {
  defaultDeny: boolean;
  requiredRoles: string[];
  requiredVerificationLevel: VerificationLevel;
  mfaRequired: boolean;
  ipWhitelist?: string[];
  timeRestrictions?: TimeRestriction[];
}

interface RetentionPolicy {
  minRetentionDays: number;
  maxRetentionDays: number;
  archiveAfterDays: number;
  purgeSchedule: string;
  legalHoldOverride: boolean;
}

interface AuditPolicy {
  logAllAccess: boolean;
  logAllModifications: boolean;
  realTimeAlerts: boolean;
  alertThresholds: AlertThreshold[];
  retentionDays: number;
}

// 메인 보안 관리자
class SecurityManager {
  private policies: Map<string, SecurityPolicy> = new Map();
  private encryptionService: EncryptionService;
  private accessController: AccessController;
  private auditLogger: SecurityAuditLogger;
  private keyManager: KeyManager;

  constructor(private config: SecurityConfig) {
    this.encryptionService = new EncryptionService(config.encryption);
    this.accessController = new AccessController(config.access);
    this.auditLogger = new SecurityAuditLogger(config.audit);
    this.keyManager = new KeyManager(config.keys);
    this.loadPolicies();
  }

  private loadPolicies(): void {
    // 기본 정책 로드
    this.policies.set('identity-core', {
      id: 'identity-core',
      name: '핵심 신원 데이터',
      dataCategories: ['identity-core'],
      securityLevel: 'restricted',
      encryption: {
        algorithm: 'aes-256-gcm',
        keyLength: 256,
        keyRotationDays: 90,
        atRest: true,
        inTransit: true,
        fieldLevel: [
          { field: 'nationalId', algorithm: 'aes-256-gcm', searchable: true, deterministicSearch: true },
          { field: 'passport', algorithm: 'aes-256-gcm', searchable: true, deterministicSearch: true },
          { field: 'dateOfBirth', algorithm: 'aes-256-gcm', searchable: false }
        ]
      },
      access: {
        defaultDeny: true,
        requiredRoles: ['identity-admin', 'medical-staff'],
        requiredVerificationLevel: 'enhanced',
        mfaRequired: true
      },
      retention: {
        minRetentionDays: 3650, // 최소 10년
        maxRetentionDays: 36500, // 최대 100년 (평생 보존)
        archiveAfterDays: 365,
        purgeSchedule: '0 0 1 * *', // 매월
        legalHoldOverride: true
      },
      audit: {
        logAllAccess: true,
        logAllModifications: true,
        realTimeAlerts: true,
        alertThresholds: [
          { metric: 'failed-access', threshold: 5, window: 300, severity: 'high' },
          { metric: 'bulk-export', threshold: 100, window: 3600, severity: 'critical' }
        ],
        retentionDays: 2555 // 7년
      }
    });

    this.policies.set('biometric', {
      id: 'biometric',
      name: '생체정보 데이터',
      dataCategories: ['biometric'],
      securityLevel: 'top-secret',
      encryption: {
        algorithm: 'aes-256-gcm',
        keyLength: 256,
        keyRotationDays: 30,
        atRest: true,
        inTransit: true,
        fieldLevel: [
          { field: 'template', algorithm: 'aes-256-gcm', searchable: false },
          { field: 'rawData', algorithm: 'aes-256-gcm', searchable: false }
        ]
      },
      access: {
        defaultDeny: true,
        requiredRoles: ['biometric-admin'],
        requiredVerificationLevel: 'maximum',
        mfaRequired: true,
        ipWhitelist: ['10.0.0.0/8', '192.168.0.0/16']
      },
      retention: {
        minRetentionDays: 365,
        maxRetentionDays: 3650,
        archiveAfterDays: 180,
        purgeSchedule: '0 0 1 * *',
        legalHoldOverride: false
      },
      audit: {
        logAllAccess: true,
        logAllModifications: true,
        realTimeAlerts: true,
        alertThresholds: [
          { metric: 'any-access', threshold: 1, window: 60, severity: 'medium' },
          { metric: 'failed-access', threshold: 3, window: 300, severity: 'critical' }
        ],
        retentionDays: 3650
      }
    });
  }

  async protectData<T>(
    data: T,
    category: DataCategory,
    context: SecurityContext
  ): Promise<ProtectedData<T>> {
    const policy = this.getPolicyForCategory(category);

    // 접근 검증
    await this.verifyAccess(context, policy);

    // 민감 필드 암호화
    const encrypted = await this.encryptionService.encryptFields(
      data,
      policy.encryption.fieldLevel
    );

    // 접근 로그
    await this.auditLogger.logDataProtection({
      category,
      action: 'protect',
      userId: context.userId,
      timestamp: new Date()
    });

    return {
      data: encrypted,
      metadata: {
        policy: policy.id,
        encryptedAt: new Date(),
        keyVersion: await this.keyManager.getCurrentKeyVersion()
      }
    };
  }

  async accessData<T>(
    protectedData: ProtectedData<T>,
    category: DataCategory,
    context: SecurityContext
  ): Promise<T> {
    const policy = this.getPolicyForCategory(category);

    // 접근 검증
    await this.verifyAccess(context, policy);

    // 데이터 복호화
    const decrypted = await this.encryptionService.decryptFields(
      protectedData.data,
      policy.encryption.fieldLevel,
      protectedData.metadata.keyVersion
    );

    // 접근 로그
    await this.auditLogger.logDataAccess({
      category,
      action: 'access',
      userId: context.userId,
      timestamp: new Date(),
      fields: Object.keys(decrypted as object)
    });

    return decrypted;
  }

  private async verifyAccess(
    context: SecurityContext,
    policy: SecurityPolicy
  ): Promise<void> {
    // 역할 확인
    const hasRequiredRole = policy.access.requiredRoles.some(
      role => context.roles.includes(role)
    );
    if (!hasRequiredRole) {
      throw new SecurityError('ACCESS_DENIED', '역할 권한이 부족합니다');
    }

    // 인증 레벨 확인
    if (context.verificationLevel) {
      const levels: VerificationLevel[] = ['basic', 'standard', 'enhanced', 'maximum'];
      const userLevel = levels.indexOf(context.verificationLevel);
      const requiredLevel = levels.indexOf(policy.access.requiredVerificationLevel);
      if (userLevel < requiredLevel) {
        throw new SecurityError('ACCESS_DENIED', '인증 레벨이 부족합니다');
      }
    }

    // MFA 확인
    if (policy.access.mfaRequired && !context.mfaVerified) {
      throw new SecurityError('MFA_REQUIRED', '다단계 인증이 필요합니다');
    }

    // IP 화이트리스트 확인
    if (policy.access.ipWhitelist && context.ipAddress) {
      const allowed = this.isIpAllowed(context.ipAddress, policy.access.ipWhitelist);
      if (!allowed) {
        throw new SecurityError('ACCESS_DENIED', 'IP 주소가 허용 목록에 없습니다');
      }
    }

    // 시간 제한 확인
    if (policy.access.timeRestrictions) {
      const withinWindow = this.isWithinTimeWindow(policy.access.timeRestrictions);
      if (!withinWindow) {
        throw new SecurityError('ACCESS_DENIED', '허용된 시간 외 접근입니다');
      }
    }
  }

  private getPolicyForCategory(category: DataCategory): SecurityPolicy {
    for (const policy of this.policies.values()) {
      if (policy.dataCategories.includes(category)) {
        return policy;
      }
    }
    throw new SecurityError('POLICY_NOT_FOUND', `정책을 찾을 수 없습니다: ${category}`);
  }

  private isIpAllowed(ip: string, whitelist: string[]): boolean {
    // 간소화된 IP 확인 - 실제 구현에서는 적절한 CIDR 매칭 사용
    return whitelist.some(cidr => ip.startsWith(cidr.split('/')[0].replace(/\.0$/, '')));
  }

  private isWithinTimeWindow(restrictions: TimeRestriction[]): boolean {
    const now = new Date();
    const currentDay = now.getDay();
    const currentTime = now.toTimeString().substring(0, 5);

    return restrictions.some(r =>
      r.dayOfWeek.includes(currentDay) &&
      currentTime >= r.startTime &&
      currentTime <= r.endTime
    );
  }
}

interface SecurityConfig {
  encryption: EncryptionConfig;
  access: AccessConfig;
  audit: AuditConfig;
  keys: KeyConfig;
}

interface SecurityContext {
  userId: string;
  roles: string[];
  verificationLevel?: VerificationLevel;
  mfaVerified: boolean;
  ipAddress?: string;
  sessionId: string;
}

interface ProtectedData<T> {
  data: T;
  metadata: {
    policy: string;
    encryptedAt: Date;
    keyVersion: string;
  };
}

class SecurityError extends Error {
  constructor(public code: string, message: string) {
    super(message);
    this.name = 'SecurityError';
  }
}
```

### 암호화 서비스

```typescript
// 신원 데이터를 위한 고급 암호화
interface EncryptionConfig {
  masterKeyId: string;
  defaultAlgorithm: string;
  keyDerivationIterations: number;
  saltLength: number;
  ivLength: number;
  tagLength: number;
}

interface EncryptedField {
  ciphertext: string;
  iv: string;
  tag: string;
  algorithm: string;
  keyVersion: string;
}

interface EncryptedSearchable {
  ciphertext: string;
  searchHash: string;
  iv: string;
  tag: string;
  algorithm: string;
  keyVersion: string;
}

class EncryptionService {
  private keyCache: Map<string, Buffer> = new Map();

  constructor(
    private config: EncryptionConfig,
    private keyManager: KeyManager
  ) {}

  async encryptFields<T extends object>(
    data: T,
    rules: FieldEncryptionRule[]
  ): Promise<T> {
    const result = { ...data };

    for (const rule of rules) {
      const value = this.getNestedValue(result, rule.field);
      if (value !== undefined) {
        const encrypted = rule.searchable
          ? await this.encryptSearchable(value, rule)
          : await this.encrypt(value, rule.algorithm);
        this.setNestedValue(result, rule.field, encrypted);
      }
    }

    return result;
  }

  async decryptFields<T extends object>(
    data: T,
    rules: FieldEncryptionRule[],
    keyVersion: string
  ): Promise<T> {
    const result = { ...data };

    for (const rule of rules) {
      const encrypted = this.getNestedValue(result, rule.field);
      if (encrypted && typeof encrypted === 'object') {
        const decrypted = rule.searchable
          ? await this.decryptSearchable(encrypted as EncryptedSearchable, keyVersion)
          : await this.decrypt(encrypted as EncryptedField, keyVersion);
        this.setNestedValue(result, rule.field, decrypted);
      }
    }

    return result;
  }

  async encrypt(value: any, algorithm: string = 'aes-256-gcm'): Promise<EncryptedField> {
    const key = await this.getEncryptionKey();
    const iv = randomBytes(this.config.ivLength);
    const valueStr = typeof value === 'string' ? value : JSON.stringify(value);

    const cipher = createCipheriv(algorithm, key, iv);
    let ciphertext = cipher.update(valueStr, 'utf8', 'base64');
    ciphertext += cipher.final('base64');
    const tag = cipher.getAuthTag();

    return {
      ciphertext,
      iv: iv.toString('base64'),
      tag: tag.toString('base64'),
      algorithm,
      keyVersion: await this.keyManager.getCurrentKeyVersion()
    };
  }

  async decrypt(encrypted: EncryptedField, keyVersion: string): Promise<any> {
    const key = await this.getEncryptionKey(keyVersion);
    const iv = Buffer.from(encrypted.iv, 'base64');
    const tag = Buffer.from(encrypted.tag, 'base64');

    const decipher = createDecipheriv(encrypted.algorithm, key, iv);
    decipher.setAuthTag(tag);

    let decrypted = decipher.update(encrypted.ciphertext, 'base64', 'utf8');
    decrypted += decipher.final('utf8');

    try {
      return JSON.parse(decrypted);
    } catch {
      return decrypted;
    }
  }

  async encryptSearchable(
    value: any,
    rule: FieldEncryptionRule
  ): Promise<EncryptedSearchable> {
    const encrypted = await this.encrypt(value, rule.algorithm);
    const searchHash = await this.generateSearchHash(value, rule.deterministicSearch);

    return {
      ...encrypted,
      searchHash
    };
  }

  async decryptSearchable(
    encrypted: EncryptedSearchable,
    keyVersion: string
  ): Promise<any> {
    return this.decrypt(encrypted, keyVersion);
  }

  async generateSearchHash(value: any, deterministic: boolean = false): Promise<string> {
    const valueStr = typeof value === 'string' ? value : JSON.stringify(value);

    if (deterministic) {
      // 정확한 일치 검색을 위한 결정적 해시
      const salt = await this.keyManager.getSearchSalt();
      const hash = await scryptAsync(valueStr, salt, 32);
      return (hash as Buffer).toString('base64');
    } else {
      // 각 해시에 대한 랜덤 솔트 (더 안전하지만 검색 불가)
      const salt = randomBytes(16);
      const hash = await scryptAsync(valueStr, salt, 32);
      return salt.toString('base64') + ':' + (hash as Buffer).toString('base64');
    }
  }

  async searchByHash(searchValue: any, encryptedRecords: EncryptedSearchable[]): Promise<number[]> {
    const searchHash = await this.generateSearchHash(searchValue, true);
    const matches: number[] = [];

    for (let i = 0; i < encryptedRecords.length; i++) {
      if (encryptedRecords[i].searchHash === searchHash) {
        matches.push(i);
      }
    }

    return matches;
  }

  private async getEncryptionKey(version?: string): Promise<Buffer> {
    const keyVersion = version || await this.keyManager.getCurrentKeyVersion();
    const cacheKey = `enc-${keyVersion}`;

    if (this.keyCache.has(cacheKey)) {
      return this.keyCache.get(cacheKey)!;
    }

    const masterKey = await this.keyManager.getMasterKey(keyVersion);
    const derivedKey = await scryptAsync(
      masterKey,
      Buffer.from(this.config.masterKeyId),
      32
    ) as Buffer;

    this.keyCache.set(cacheKey, derivedKey);
    return derivedKey;
  }

  private getNestedValue(obj: any, path: string): any {
    return path.split('.').reduce((current, key) => current?.[key], obj);
  }

  private setNestedValue(obj: any, path: string, value: any): void {
    const keys = path.split('.');
    const lastKey = keys.pop()!;
    const target = keys.reduce((current, key) => {
      if (!current[key]) current[key] = {};
      return current[key];
    }, obj);
    target[lastKey] = value;
  }
}

// 키 관리
interface KeyConfig {
  provider: 'aws-kms' | 'azure-keyvault' | 'hashicorp-vault' | 'local';
  masterKeyArn?: string;
  vaultAddress?: string;
  rotationSchedule: string;
  keyHistoryLimit: number;
}

class KeyManager {
  private currentVersion: string;
  private keyHistory: Map<string, Buffer> = new Map();

  constructor(private config: KeyConfig) {
    this.currentVersion = this.generateKeyVersion();
  }

  async getCurrentKeyVersion(): Promise<string> {
    return this.currentVersion;
  }

  async getMasterKey(version: string): Promise<Buffer> {
    if (this.keyHistory.has(version)) {
      return this.keyHistory.get(version)!;
    }

    // 실제 구현에서는 KMS/Vault에서 가져오기
    const key = await this.fetchKeyFromProvider(version);
    this.keyHistory.set(version, key);

    // 히스토리 크기 제한
    if (this.keyHistory.size > this.config.keyHistoryLimit) {
      const oldest = this.keyHistory.keys().next().value;
      this.keyHistory.delete(oldest);
    }

    return key;
  }

  async getSearchSalt(): Promise<Buffer> {
    // 결정적 검색 해싱을 위한 상수 솔트
    // 실제 구현에서는 안전하게 저장
    return Buffer.from('cryo-identity-search-salt-v1');
  }

  async rotateKey(): Promise<string> {
    const newVersion = this.generateKeyVersion();
    const newKey = randomBytes(32);

    // 프로바이더에 저장
    await this.storeKeyInProvider(newVersion, newKey);

    this.keyHistory.set(newVersion, newKey);
    this.currentVersion = newVersion;

    return newVersion;
  }

  private generateKeyVersion(): string {
    return `v${Date.now()}-${randomBytes(4).toString('hex')}`;
  }

  private async fetchKeyFromProvider(version: string): Promise<Buffer> {
    // 실제 KMS/Vault 연결 구현
    return randomBytes(32);
  }

  private async storeKeyInProvider(version: string, key: Buffer): Promise<void> {
    // 실제 KMS/Vault 저장 구현
  }
}
```

### 보안 감사 로깅

```typescript
// 포괄적인 보안 감사 로깅
interface AuditConfig {
  storage: 'database' | 'elasticsearch' | 'splunk' | 's3';
  endpoint: string;
  bufferSize: number;
  flushInterval: number;
  encryption: boolean;
  signing: boolean;
}

interface AuditEvent {
  id: string;
  timestamp: Date;
  eventType: AuditEventType;
  severity: 'info' | 'warning' | 'error' | 'critical';
  actor: AuditActor;
  target: AuditTarget;
  action: string;
  result: 'success' | 'failure' | 'partial';
  details: Record<string, any>;
  metadata: AuditMetadata;
}

type AuditEventType =
  | 'authentication'    // 인증
  | 'authorization'     // 권한 부여
  | 'data-access'       // 데이터 접근
  | 'data-modification' // 데이터 수정
  | 'data-deletion'     // 데이터 삭제
  | 'verification'      // 인증
  | 'consent'           // 동의
  | 'export'            // 내보내기
  | 'admin-action'      // 관리자 작업
  | 'security-alert';   // 보안 알림

interface AuditActor {
  type: 'user' | 'system' | 'integration';
  id: string;
  name?: string;
  roles: string[];
  ipAddress?: string;
  userAgent?: string;
  sessionId?: string;
}

interface AuditTarget {
  type: 'subject' | 'specimen' | 'document' | 'system';
  id: string;
  category?: DataCategory;
}

interface AuditMetadata {
  facility: string;
  environment: string;
  version: string;
  correlationId?: string;
  parentEventId?: string;
  duration?: number;
}

class SecurityAuditLogger {
  private buffer: AuditEvent[] = [];
  private flushTimer: NodeJS.Timer;
  private signer: AuditSigner;
  private storage: AuditStorage;

  constructor(private config: AuditConfig) {
    this.signer = new AuditSigner();
    this.storage = this.createStorage(config);
    this.startFlushTimer();
  }

  private createStorage(config: AuditConfig): AuditStorage {
    switch (config.storage) {
      case 'elasticsearch':
        return new ElasticsearchAuditStorage(config.endpoint);
      case 'splunk':
        return new SplunkAuditStorage(config.endpoint);
      case 's3':
        return new S3AuditStorage(config.endpoint);
      default:
        return new DatabaseAuditStorage(config.endpoint);
    }
  }

  private startFlushTimer(): void {
    this.flushTimer = setInterval(() => {
      this.flush();
    }, this.config.flushInterval);
  }

  async log(event: Partial<AuditEvent>): Promise<string> {
    const fullEvent: AuditEvent = {
      id: this.generateEventId(),
      timestamp: new Date(),
      severity: 'info',
      result: 'success',
      details: {},
      metadata: {
        facility: process.env.FACILITY_ID || 'unknown',
        environment: process.env.NODE_ENV || 'development',
        version: process.env.APP_VERSION || '1.0.0'
      },
      ...event
    } as AuditEvent;

    // 무결성을 위해 이벤트 서명
    if (this.config.signing) {
      (fullEvent as any).signature = await this.signer.sign(fullEvent);
    }

    this.buffer.push(fullEvent);

    // 알림 임계값 확인
    await this.checkAlertThresholds(fullEvent);

    // 버퍼가 가득 차면 플러시
    if (this.buffer.length >= this.config.bufferSize) {
      await this.flush();
    }

    return fullEvent.id;
  }

  async logDataAccess(params: {
    category: DataCategory;
    action: string;
    userId: string;
    timestamp: Date;
    fields?: string[];
  }): Promise<string> {
    return this.log({
      eventType: 'data-access',
      actor: {
        type: 'user',
        id: params.userId,
        roles: []
      },
      target: {
        type: 'subject',
        id: 'unknown',
        category: params.category
      },
      action: params.action,
      details: {
        fields: params.fields
      }
    });
  }

  async logSecurityAlert(params: {
    alertType: string;
    severity: 'warning' | 'error' | 'critical';
    description: string;
    affectedSubjects?: string[];
    remediationSteps?: string[];
  }): Promise<string> {
    return this.log({
      eventType: 'security-alert',
      severity: params.severity,
      actor: {
        type: 'system',
        id: 'security-monitor',
        roles: ['system']
      },
      target: {
        type: 'system',
        id: 'cryo-identity'
      },
      action: params.alertType,
      details: {
        description: params.description,
        affectedSubjects: params.affectedSubjects,
        remediationSteps: params.remediationSteps
      }
    });
  }

  async logAuthenticationEvent(params: {
    userId: string;
    method: string;
    success: boolean;
    ipAddress: string;
    userAgent: string;
    failureReason?: string;
  }): Promise<string> {
    return this.log({
      eventType: 'authentication',
      severity: params.success ? 'info' : 'warning',
      result: params.success ? 'success' : 'failure',
      actor: {
        type: 'user',
        id: params.userId,
        roles: [],
        ipAddress: params.ipAddress,
        userAgent: params.userAgent
      },
      target: {
        type: 'system',
        id: 'authentication-service'
      },
      action: params.method,
      details: {
        failureReason: params.failureReason
      }
    });
  }

  private async checkAlertThresholds(event: AuditEvent): Promise<void> {
    // 알림을 트리거해야 하는 패턴 확인
    if (event.eventType === 'authentication' && event.result === 'failure') {
      const recentFailures = await this.countRecentEvents({
        eventType: 'authentication',
        result: 'failure',
        actorId: event.actor.id,
        windowMinutes: 5
      });

      if (recentFailures >= 5) {
        await this.triggerAlert({
          type: 'brute-force-attempt',
          severity: 'high',
          actorId: event.actor.id,
          details: { failureCount: recentFailures }
        });
      }
    }

    if (event.eventType === 'export') {
      const recentExports = await this.countRecentEvents({
        eventType: 'export',
        actorId: event.actor.id,
        windowMinutes: 60
      });

      if (recentExports >= 10) {
        await this.triggerAlert({
          type: 'bulk-data-export',
          severity: 'critical',
          actorId: event.actor.id,
          details: { exportCount: recentExports }
        });
      }
    }
  }

  private async countRecentEvents(criteria: {
    eventType: AuditEventType;
    result?: string;
    actorId: string;
    windowMinutes: number;
  }): Promise<number> {
    // 실제 구현에서는 감사 저장소 쿼리
    return this.buffer.filter(e =>
      e.eventType === criteria.eventType &&
      e.actor.id === criteria.actorId &&
      (!criteria.result || e.result === criteria.result) &&
      e.timestamp > new Date(Date.now() - criteria.windowMinutes * 60 * 1000)
    ).length;
  }

  private async triggerAlert(alert: {
    type: string;
    severity: string;
    actorId: string;
    details: Record<string, any>;
  }): Promise<void> {
    // 모니터링 시스템에 알림 전송
    console.error('보안 알림:', alert);

    await this.logSecurityAlert({
      alertType: alert.type,
      severity: alert.severity as 'warning' | 'error' | 'critical',
      description: `${alert.actorId}에 대한 보안 알림 발생`,
      remediationSteps: this.getRemediationSteps(alert.type)
    });
  }

  private getRemediationSteps(alertType: string): string[] {
    const remediations: Record<string, string[]> = {
      'brute-force-attempt': [
        '사용자 계정 임시 잠금',
        '보조 채널을 통해 사용자에게 알림',
        '차단을 위한 IP 주소 검토'
      ],
      'bulk-data-export': [
        '내보내기 요청 검토',
        '사용자 권한 확인',
        '데이터 유출 지표 확인'
      ]
    };

    return remediations[alertType] || ['사건 조사 필요'];
  }

  async flush(): Promise<void> {
    if (this.buffer.length === 0) return;

    const events = [...this.buffer];
    this.buffer = [];

    try {
      await this.storage.store(events);
    } catch (error) {
      // 실패 시 버퍼에 다시 추가
      this.buffer.unshift(...events);
      console.error('감사 로그 플러시 실패:', error);
    }
  }

  private generateEventId(): string {
    return `AE-${Date.now()}-${randomBytes(4).toString('hex')}`;
  }

  async query(criteria: AuditQueryCriteria): Promise<AuditEvent[]> {
    return this.storage.query(criteria);
  }

  async verifyIntegrity(eventId: string): Promise<IntegrityVerification> {
    const event = await this.storage.getById(eventId);
    if (!event) {
      return { valid: false, reason: '이벤트를 찾을 수 없습니다' };
    }

    if (this.config.signing) {
      const signature = (event as any).signature;
      const eventWithoutSig = { ...event };
      delete (eventWithoutSig as any).signature;

      const isValid = await this.signer.verify(eventWithoutSig, signature);
      return {
        valid: isValid,
        reason: isValid ? '서명 확인됨' : '잘못된 서명'
      };
    }

    return { valid: true, reason: '서명 기능 비활성화됨' };
  }
}

interface AuditQueryCriteria {
  startDate?: Date;
  endDate?: Date;
  eventTypes?: AuditEventType[];
  actorId?: string;
  targetId?: string;
  severity?: string[];
  limit?: number;
  offset?: number;
}

interface IntegrityVerification {
  valid: boolean;
  reason: string;
}

// 변조 감지를 위한 감사 서명
class AuditSigner {
  private privateKey: Buffer;

  constructor() {
    // 실제 구현에서는 안전한 키 저장소에서 로드
    this.privateKey = randomBytes(32);
  }

  async sign(event: AuditEvent): Promise<string> {
    const crypto = await import('crypto');
    const hmac = crypto.createHmac('sha256', this.privateKey);
    hmac.update(JSON.stringify(event));
    return hmac.digest('base64');
  }

  async verify(event: AuditEvent, signature: string): Promise<boolean> {
    const expectedSignature = await this.sign(event);
    return signature === expectedSignature;
  }
}

interface AuditStorage {
  store(events: AuditEvent[]): Promise<void>;
  query(criteria: AuditQueryCriteria): Promise<AuditEvent[]>;
  getById(id: string): Promise<AuditEvent | null>;
}
```

### 위협 탐지

```typescript
// 신원 시스템을 위한 실시간 위협 탐지
interface ThreatDetectionConfig {
  rules: ThreatRule[];
  mlModelEndpoint?: string;
  alertWebhook: string;
  quarantineEnabled: boolean;
}

interface ThreatRule {
  id: string;
  name: string;
  description: string;
  condition: ThreatCondition;
  severity: 'low' | 'medium' | 'high' | 'critical';
  actions: ThreatAction[];
  enabled: boolean;
}

interface ThreatCondition {
  type: 'pattern' | 'threshold' | 'anomaly' | 'ml';
  parameters: Record<string, any>;
}

type ThreatAction =
  | 'alert'         // 알림
  | 'block'         // 차단
  | 'quarantine'    // 격리
  | 'require-mfa'   // MFA 요구
  | 'lock-account'  // 계정 잠금
  | 'notify-admin'; // 관리자 알림

interface ThreatEvent {
  id: string;
  ruleId: string;
  severity: string;
  detectedAt: Date;
  actor: AuditActor;
  indicators: ThreatIndicator[];
  status: 'detected' | 'investigating' | 'mitigated' | 'false-positive';
  actions: ActionResult[];
}

interface ThreatIndicator {
  type: string;
  value: any;
  confidence: number;
}

class ThreatDetectionService {
  private rules: Map<string, ThreatRule> = new Map();
  private eventBuffer: AuditEvent[] = [];
  private alertService: AlertService;

  constructor(
    private config: ThreatDetectionConfig,
    private auditLogger: SecurityAuditLogger
  ) {
    this.alertService = new AlertService(config.alertWebhook);
    this.loadRules();
  }

  private loadRules(): void {
    for (const rule of this.config.rules) {
      if (rule.enabled) {
        this.rules.set(rule.id, rule);
      }
    }

    // 기본 규칙 추가
    this.rules.set('impossible-travel', {
      id: 'impossible-travel',
      name: '불가능한 이동 탐지',
      description: '짧은 시간 내 지리적으로 먼 위치에서의 로그인 감지',
      condition: {
        type: 'pattern',
        parameters: {
          maxSpeedKmH: 1000,
          timeWindowMinutes: 60
        }
      },
      severity: 'high',
      actions: ['alert', 'require-mfa'],
      enabled: true
    });

    this.rules.set('credential-stuffing', {
      id: 'credential-stuffing',
      name: '자격 증명 스터핑 탐지',
      description: '동일 IP에서 다수의 로그인 실패 감지',
      condition: {
        type: 'threshold',
        parameters: {
          metric: 'failed-auth',
          threshold: 10,
          windowMinutes: 5,
          groupBy: 'ip'
        }
      },
      severity: 'critical',
      actions: ['block', 'alert', 'notify-admin'],
      enabled: true
    });

    this.rules.set('privilege-escalation', {
      id: 'privilege-escalation',
      name: '권한 상승 시도',
      description: '제한된 리소스에 대한 무단 접근 시도 감지',
      condition: {
        type: 'pattern',
        parameters: {
          eventType: 'authorization',
          result: 'failure',
          targetCategory: ['restricted', 'top-secret']
        }
      },
      severity: 'critical',
      actions: ['alert', 'lock-account', 'notify-admin'],
      enabled: true
    });

    this.rules.set('data-exfiltration', {
      id: 'data-exfiltration',
      name: '데이터 유출 탐지',
      description: '비정상적인 데이터 내보내기 패턴 감지',
      condition: {
        type: 'anomaly',
        parameters: {
          metric: 'export-volume',
          deviationThreshold: 3
        }
      },
      severity: 'critical',
      actions: ['quarantine', 'alert', 'notify-admin'],
      enabled: true
    });
  }

  async analyze(event: AuditEvent): Promise<ThreatEvent[]> {
    this.eventBuffer.push(event);

    // 버퍼 크기 관리
    if (this.eventBuffer.length > 10000) {
      this.eventBuffer = this.eventBuffer.slice(-5000);
    }

    const threats: ThreatEvent[] = [];

    for (const [ruleId, rule] of this.rules) {
      const detected = await this.evaluateRule(rule, event);
      if (detected) {
        const threat = await this.createThreatEvent(rule, event, detected);
        threats.push(threat);
        await this.executeActions(threat, rule.actions);
      }
    }

    return threats;
  }

  private async evaluateRule(
    rule: ThreatRule,
    event: AuditEvent
  ): Promise<ThreatIndicator[] | null> {
    switch (rule.condition.type) {
      case 'pattern':
        return this.evaluatePatternRule(rule, event);
      case 'threshold':
        return this.evaluateThresholdRule(rule, event);
      case 'anomaly':
        return this.evaluateAnomalyRule(rule, event);
      case 'ml':
        return this.evaluateMLRule(rule, event);
      default:
        return null;
    }
  }

  private async evaluateThresholdRule(
    rule: ThreatRule,
    event: AuditEvent
  ): Promise<ThreatIndicator[] | null> {
    const params = rule.condition.parameters;
    const windowStart = new Date(Date.now() - params.windowMinutes * 60 * 1000);

    let groupValue: string;
    switch (params.groupBy) {
      case 'ip':
        groupValue = event.actor.ipAddress || '';
        break;
      case 'user':
        groupValue = event.actor.id;
        break;
      default:
        groupValue = 'global';
    }

    const count = this.eventBuffer.filter(e => {
      if (e.timestamp < windowStart) return false;

      if (params.metric === 'failed-auth') {
        if (e.eventType !== 'authentication' || e.result !== 'failure') return false;
      }

      switch (params.groupBy) {
        case 'ip':
          return e.actor.ipAddress === groupValue;
        case 'user':
          return e.actor.id === groupValue;
        default:
          return true;
      }
    }).length;

    if (count >= params.threshold) {
      return [{
        type: 'threshold-exceeded',
        value: { count, threshold: params.threshold, groupBy: params.groupBy, groupValue },
        confidence: 0.95
      }];
    }

    return null;
  }

  private async executeActions(
    threat: ThreatEvent,
    actions: ThreatAction[]
  ): Promise<void> {
    for (const action of actions) {
      const result = await this.executeAction(threat, action);
      threat.actions.push(result);
    }
  }

  private async executeAction(
    threat: ThreatEvent,
    action: ThreatAction
  ): Promise<ActionResult> {
    const result: ActionResult = {
      action,
      executedAt: new Date(),
      success: true
    };

    try {
      switch (action) {
        case 'alert':
          await this.alertService.send(threat);
          break;
        case 'block':
          result.details = '접근 차단됨';
          break;
        case 'quarantine':
          if (this.config.quarantineEnabled) {
            result.details = '계정 격리됨';
          }
          break;
        case 'require-mfa':
          result.details = '이제 MFA 필수';
          break;
        case 'lock-account':
          result.details = '계정 잠김';
          break;
        case 'notify-admin':
          await this.alertService.notifyAdmin(threat);
          break;
      }
    } catch (error) {
      result.success = false;
      result.details = error instanceof Error ? error.message : '작업 실패';
    }

    return result;
  }

  private async evaluatePatternRule(
    rule: ThreatRule,
    event: AuditEvent
  ): Promise<ThreatIndicator[] | null> {
    const params = rule.condition.parameters;

    if (params.eventType && event.eventType !== params.eventType) {
      return null;
    }

    if (params.result && event.result !== params.result) {
      return null;
    }

    return [{
      type: 'pattern-match',
      value: rule.id,
      confidence: 0.9
    }];
  }

  private async evaluateAnomalyRule(
    rule: ThreatRule,
    event: AuditEvent
  ): Promise<ThreatIndicator[] | null> {
    // 간소화된 이상 탐지
    return null;
  }

  private async evaluateMLRule(
    rule: ThreatRule,
    event: AuditEvent
  ): Promise<ThreatIndicator[] | null> {
    if (!this.config.mlModelEndpoint) return null;
    return null;
  }

  private async createThreatEvent(
    rule: ThreatRule,
    trigger: AuditEvent,
    indicators: ThreatIndicator[]
  ): Promise<ThreatEvent> {
    return {
      id: `TE-${Date.now()}-${randomBytes(4).toString('hex')}`,
      ruleId: rule.id,
      severity: rule.severity,
      detectedAt: new Date(),
      actor: trigger.actor,
      indicators,
      status: 'detected',
      actions: []
    };
  }
}

interface ActionResult {
  action: ThreatAction;
  executedAt: Date;
  success: boolean;
  details?: string;
}
```

## 요약

이 장에서 다룬 내용:

1. **보안 아키텍처**: 포괄적인 정책 기반 보안 프레임워크
2. **암호화 서비스**: 검색 가능 암호화를 지원하는 필드 레벨 암호화
3. **키 관리**: 안전한 키 저장 및 로테이션
4. **감사 로깅**: 무결성 검증이 가능한 변조 방지 로깅
5. **위협 탐지**: 실시간 패턴 매칭 및 이상 탐지

다음 장에서는 구현 지침과 배포 전략을 다룹니다.
