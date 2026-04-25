# 제7장: 보안 및 컴플라이언스 - 데이터 보호 및 감사

## 종합 보안 프레임워크

이 장에서는 WIA 극저온 법률 표준의 완전한 보안 구현을 제공하며, 암호화, 접근 제어, 감사 로깅 및 위협 탐지를 포함합니다.

## 보안 관리자

```typescript
/**
 * WIA 극저온 법률 표준 - 보안 관리자
 * 종합 보안 프레임워크 구현
 */

import { z } from 'zod';
import crypto from 'crypto';

// ============================================================================
// 보안 설정 스키마
// ============================================================================

export const SecurityConfigSchema = z.object({
  encryption: z.object({
    algorithm: z.enum(['aes-256-gcm', 'aes-256-cbc', 'chacha20-poly1305']),
    keyRotationDays: z.number().min(30).max(365),
    keyDerivation: z.enum(['pbkdf2', 'argon2', 'scrypt']),
  }),
  accessControl: z.object({
    sessionTimeoutMinutes: z.number().min(5).max(480),
    maxFailedAttempts: z.number().min(3).max(10),
    lockoutDurationMinutes: z.number().min(5).max(1440),
    mfaRequired: z.boolean(),
    ipWhitelisting: z.boolean(),
  }),
  audit: z.object({
    retentionDays: z.number().min(365).max(3650),
    realTimeAlerts: z.boolean(),
    detailedLogging: z.boolean(),
  }),
  compliance: z.object({
    pipa: z.boolean(),           // 개인정보보호법
    bioethicsLaw: z.boolean(),   // 생명윤리법
    electronicSignature: z.boolean(),  // 전자서명법
    dataLocalization: z.boolean(),     // 데이터 국지화
  }),
});

export type SecurityConfig = z.infer<typeof SecurityConfigSchema>;

// ============================================================================
// 한국 규제 준수 보안 설정
// ============================================================================

export const koreanSecurityConfig: SecurityConfig = {
  encryption: {
    algorithm: 'aes-256-gcm',
    keyRotationDays: 90,
    keyDerivation: 'argon2',
  },
  accessControl: {
    sessionTimeoutMinutes: 30,
    maxFailedAttempts: 5,
    lockoutDurationMinutes: 30,
    mfaRequired: true,
    ipWhitelisting: true,
  },
  audit: {
    retentionDays: 1825,  // 5년 (개인정보보호법)
    realTimeAlerts: true,
    detailedLogging: true,
  },
  compliance: {
    pipa: true,
    bioethicsLaw: true,
    electronicSignature: true,
    dataLocalization: true,
  },
};

// ============================================================================
// 보안 관리자
// ============================================================================

export class SecurityManager {
  private encryptionService: EncryptionService;
  private accessControlService: AccessControlService;
  private auditLogger: SecurityAuditLogger;
  private threatDetection: ThreatDetectionService;

  constructor(private readonly config: SecurityConfig) {
    this.encryptionService = new EncryptionService(config.encryption);
    this.accessControlService = new AccessControlService(config.accessControl);
    this.auditLogger = new SecurityAuditLogger(config.audit);
    this.threatDetection = new ThreatDetectionService();
  }

  async initialize(): Promise<void> {
    await this.encryptionService.initialize();
    await this.accessControlService.initialize();
    await this.auditLogger.initialize();
    await this.threatDetection.initialize();

    console.log('보안 관리자 초기화 완료');
  }

  // 암호화 서비스 접근
  get encryption(): EncryptionService {
    return this.encryptionService;
  }

  // 접근 제어 서비스 접근
  get accessControl(): AccessControlService {
    return this.accessControlService;
  }

  // 감사 로거 접근
  get audit(): SecurityAuditLogger {
    return this.auditLogger;
  }

  // 위협 탐지 서비스 접근
  get threats(): ThreatDetectionService {
    return this.threatDetection;
  }

  // 보안 상태 확인
  async getSecurityStatus(): Promise<SecurityStatus> {
    return {
      encryption: {
        status: 'active',
        statusKr: '활성',
        algorithm: this.config.encryption.algorithm,
        lastKeyRotation: await this.encryptionService.getLastKeyRotation(),
        nextKeyRotation: await this.encryptionService.getNextKeyRotation(),
      },
      accessControl: {
        status: 'active',
        statusKr: '활성',
        activeSessions: await this.accessControlService.getActiveSessionCount(),
        blockedIps: await this.accessControlService.getBlockedIpCount(),
      },
      audit: {
        status: 'active',
        statusKr: '활성',
        totalEvents: await this.auditLogger.getTotalEventCount(),
        todayEvents: await this.auditLogger.getTodayEventCount(),
      },
      threats: {
        status: 'monitoring',
        statusKr: '모니터링 중',
        detectedThreats: await this.threatDetection.getDetectedThreatCount(),
        activeAlerts: await this.threatDetection.getActiveAlertCount(),
      },
      compliance: {
        pipa: await this.checkPipaCompliance(),
        pipaKr: '개인정보보호법',
        bioethicsLaw: await this.checkBioethicsCompliance(),
        bioethicsLawKr: '생명윤리법',
        overallStatus: 'compliant',
        overallStatusKr: '준수',
      },
    };
  }

  private async checkPipaCompliance(): Promise<boolean> {
    // 개인정보보호법 준수 확인
    return this.config.compliance.pipa;
  }

  private async checkBioethicsCompliance(): Promise<boolean> {
    // 생명윤리법 준수 확인
    return this.config.compliance.bioethicsLaw;
  }
}

export interface SecurityStatus {
  encryption: {
    status: string;
    statusKr: string;
    algorithm: string;
    lastKeyRotation: string;
    nextKeyRotation: string;
  };
  accessControl: {
    status: string;
    statusKr: string;
    activeSessions: number;
    blockedIps: number;
  };
  audit: {
    status: string;
    statusKr: string;
    totalEvents: number;
    todayEvents: number;
  };
  threats: {
    status: string;
    statusKr: string;
    detectedThreats: number;
    activeAlerts: number;
  };
  compliance: {
    pipa: boolean;
    pipaKr: string;
    bioethicsLaw: boolean;
    bioethicsLawKr: string;
    overallStatus: string;
    overallStatusKr: string;
  };
}
```

## 암호화 서비스

```typescript
/**
 * 암호화 서비스
 * AES-256-GCM 기반 데이터 암호화
 */

export class EncryptionService {
  private masterKey: Buffer | null = null;
  private keyHistory: Map<string, Buffer> = new Map();
  private lastKeyRotation: Date | null = null;

  constructor(
    private readonly config: SecurityConfig['encryption']
  ) {}

  async initialize(): Promise<void> {
    // 마스터 키 로드 또는 생성
    this.masterKey = await this.loadOrGenerateMasterKey();
    this.lastKeyRotation = new Date();
  }

  private async loadOrGenerateMasterKey(): Promise<Buffer> {
    // 실제 구현에서는 HSM 또는 KMS에서 키 로드
    return crypto.randomBytes(32);
  }

  // 데이터 암호화
  async encrypt(plaintext: string | Buffer, context?: EncryptionContext): Promise<EncryptedData> {
    if (!this.masterKey) {
      throw new Error('암호화 서비스가 초기화되지 않았습니다');
    }

    const iv = crypto.randomBytes(12);  // GCM용 96비트 IV
    const cipher = crypto.createCipheriv(
      this.config.algorithm as crypto.CipherGCMTypes,
      this.masterKey,
      iv
    );

    // AAD (추가 인증 데이터) 설정
    const aad = context ? Buffer.from(JSON.stringify(context)) : Buffer.from('');
    cipher.setAAD(aad);

    const plainBuffer = typeof plaintext === 'string' ? Buffer.from(plaintext, 'utf8') : plaintext;
    const encrypted = Buffer.concat([cipher.update(plainBuffer), cipher.final()]);
    const authTag = cipher.getAuthTag();

    return {
      ciphertext: encrypted.toString('base64'),
      iv: iv.toString('base64'),
      authTag: authTag.toString('base64'),
      algorithm: this.config.algorithm,
      keyId: await this.getCurrentKeyId(),
      encryptedAt: new Date().toISOString(),
      context: context ? this.hashContext(context) : undefined,
    };
  }

  // 데이터 복호화
  async decrypt(encryptedData: EncryptedData, context?: EncryptionContext): Promise<Buffer> {
    const key = await this.getKeyById(encryptedData.keyId);

    if (!key) {
      throw new Error('복호화 키를 찾을 수 없습니다');
    }

    const decipher = crypto.createDecipheriv(
      encryptedData.algorithm as crypto.CipherGCMTypes,
      key,
      Buffer.from(encryptedData.iv, 'base64')
    );

    decipher.setAuthTag(Buffer.from(encryptedData.authTag, 'base64'));

    // AAD 설정
    const aad = context ? Buffer.from(JSON.stringify(context)) : Buffer.from('');
    decipher.setAAD(aad);

    const ciphertext = Buffer.from(encryptedData.ciphertext, 'base64');
    return Buffer.concat([decipher.update(ciphertext), decipher.final()]);
  }

  // 필드 레벨 암호화
  async encryptField<T extends Record<string, unknown>>(
    data: T,
    fieldsToEncrypt: (keyof T)[]
  ): Promise<T & { _encrypted: string[] }> {
    const result = { ...data, _encrypted: [] as string[] };

    for (const field of fieldsToEncrypt) {
      if (data[field] !== undefined) {
        const encrypted = await this.encrypt(JSON.stringify(data[field]));
        (result as any)[field] = encrypted;
        result._encrypted.push(field as string);
      }
    }

    return result;
  }

  // 필드 레벨 복호화
  async decryptField<T extends Record<string, unknown>>(
    data: T & { _encrypted?: string[] }
  ): Promise<T> {
    const result = { ...data };

    if (data._encrypted) {
      for (const field of data._encrypted) {
        const encryptedData = (data as any)[field] as EncryptedData;
        const decrypted = await this.decrypt(encryptedData);
        (result as any)[field] = JSON.parse(decrypted.toString('utf8'));
      }
      delete (result as any)._encrypted;
    }

    return result;
  }

  // 키 순환
  async rotateKey(): Promise<void> {
    const oldKeyId = await this.getCurrentKeyId();

    // 현재 키 이력에 저장
    if (this.masterKey) {
      this.keyHistory.set(oldKeyId, this.masterKey);
    }

    // 새 키 생성
    this.masterKey = crypto.randomBytes(32);
    this.lastKeyRotation = new Date();

    console.log(`키 순환 완료: ${oldKeyId} -> ${await this.getCurrentKeyId()}`);
  }

  // 현재 키 ID 가져오기
  private async getCurrentKeyId(): Promise<string> {
    if (!this.masterKey) {
      throw new Error('마스터 키가 없습니다');
    }
    return crypto.createHash('sha256').update(this.masterKey).digest('hex').substring(0, 16);
  }

  // 키 ID로 키 가져오기
  private async getKeyById(keyId: string): Promise<Buffer | null> {
    const currentKeyId = await this.getCurrentKeyId();
    if (keyId === currentKeyId) {
      return this.masterKey;
    }
    return this.keyHistory.get(keyId) || null;
  }

  // 컨텍스트 해시
  private hashContext(context: EncryptionContext): string {
    return crypto.createHash('sha256').update(JSON.stringify(context)).digest('hex');
  }

  async getLastKeyRotation(): Promise<string> {
    return this.lastKeyRotation?.toISOString() || 'N/A';
  }

  async getNextKeyRotation(): Promise<string> {
    if (!this.lastKeyRotation) return 'N/A';
    const next = new Date(this.lastKeyRotation);
    next.setDate(next.getDate() + this.config.keyRotationDays);
    return next.toISOString();
  }
}

export interface EncryptedData {
  ciphertext: string;
  iv: string;
  authTag: string;
  algorithm: string;
  keyId: string;
  encryptedAt: string;
  context?: string;
}

export interface EncryptionContext {
  purpose: string;
  dataType: string;
  userId?: string;
  contractId?: string;
  disputeId?: string;
}
```

## 접근 제어 서비스

```typescript
/**
 * 접근 제어 서비스
 * 역할 기반 접근 제어 (RBAC) 구현
 */

export class AccessControlService {
  private sessions: Map<string, Session> = new Map();
  private blockedIps: Map<string, BlockedIpInfo> = new Map();
  private failedAttempts: Map<string, FailedAttemptInfo> = new Map();

  constructor(
    private readonly config: SecurityConfig['accessControl']
  ) {}

  async initialize(): Promise<void> {
    // 세션 정리 타이머 시작
    setInterval(() => this.cleanupExpiredSessions(), 60000);  // 1분마다
    setInterval(() => this.cleanupExpiredBlockedIps(), 60000);
  }

  // ============================================================================
  // 세션 관리
  // ============================================================================

  async createSession(user: User, ipAddress: string): Promise<Session> {
    // IP 차단 확인
    if (await this.isIpBlocked(ipAddress)) {
      throw new Error('IP 주소가 차단되었습니다');
    }

    // IP 화이트리스트 확인
    if (this.config.ipWhitelisting && !await this.isIpWhitelisted(ipAddress)) {
      throw new Error('IP 주소가 화이트리스트에 없습니다');
    }

    const sessionId = crypto.randomUUID();
    const now = new Date();
    const expiresAt = new Date(now.getTime() + this.config.sessionTimeoutMinutes * 60000);

    const session: Session = {
      id: sessionId,
      userId: user.id,
      userEmail: user.email,
      role: user.role,
      permissions: this.getRolePermissions(user.role),
      ipAddress,
      createdAt: now.toISOString(),
      expiresAt: expiresAt.toISOString(),
      lastActivity: now.toISOString(),
      mfaVerified: false,
    };

    this.sessions.set(sessionId, session);

    // 실패 시도 초기화
    this.failedAttempts.delete(user.id);

    return session;
  }

  async validateSession(sessionId: string): Promise<Session | null> {
    const session = this.sessions.get(sessionId);

    if (!session) {
      return null;
    }

    // 만료 확인
    if (new Date() > new Date(session.expiresAt)) {
      this.sessions.delete(sessionId);
      return null;
    }

    // MFA 요구 확인
    if (this.config.mfaRequired && !session.mfaVerified) {
      return null;
    }

    // 마지막 활동 업데이트
    session.lastActivity = new Date().toISOString();

    return session;
  }

  async verifyMfa(sessionId: string, mfaCode: string): Promise<boolean> {
    const session = this.sessions.get(sessionId);

    if (!session) {
      return false;
    }

    // MFA 코드 검증 (실제 구현에서는 TOTP 등 사용)
    const isValid = await this.validateMfaCode(session.userId, mfaCode);

    if (isValid) {
      session.mfaVerified = true;
      // 세션 연장
      const expiresAt = new Date(Date.now() + this.config.sessionTimeoutMinutes * 60000);
      session.expiresAt = expiresAt.toISOString();
    }

    return isValid;
  }

  async invalidateSession(sessionId: string): Promise<void> {
    this.sessions.delete(sessionId);
  }

  async invalidateUserSessions(userId: string): Promise<number> {
    let count = 0;
    for (const [id, session] of this.sessions) {
      if (session.userId === userId) {
        this.sessions.delete(id);
        count++;
      }
    }
    return count;
  }

  // ============================================================================
  // 권한 확인
  // ============================================================================

  async hasPermission(sessionId: string, permission: Permission): Promise<boolean> {
    const session = await this.validateSession(sessionId);

    if (!session) {
      return false;
    }

    return session.permissions.includes(permission);
  }

  async checkAccess(
    sessionId: string,
    resource: string,
    action: 'read' | 'write' | 'delete' | 'admin'
  ): Promise<AccessCheckResult> {
    const session = await this.validateSession(sessionId);

    if (!session) {
      return {
        allowed: false,
        reason: '유효하지 않은 세션입니다',
        reasonCode: 'INVALID_SESSION',
      };
    }

    const permission = `${resource}:${action}` as Permission;

    if (!session.permissions.includes(permission)) {
      return {
        allowed: false,
        reason: `${resource}에 대한 ${action} 권한이 없습니다`,
        reasonCode: 'INSUFFICIENT_PERMISSION',
      };
    }

    return {
      allowed: true,
      session,
    };
  }

  // ============================================================================
  // 역할 및 권한
  // ============================================================================

  private getRolePermissions(role: UserRole): Permission[] {
    const rolePermissions: Record<UserRole, Permission[]> = {
      'admin': [
        'contracts:read', 'contracts:write', 'contracts:delete', 'contracts:admin',
        'disputes:read', 'disputes:write', 'disputes:delete', 'disputes:admin',
        'compliance:read', 'compliance:write', 'compliance:admin',
        'users:read', 'users:write', 'users:delete', 'users:admin',
        'audit:read', 'audit:admin',
        'security:read', 'security:write', 'security:admin',
      ],
      'legal-officer': [
        'contracts:read', 'contracts:write',
        'disputes:read', 'disputes:write',
        'compliance:read', 'compliance:write',
        'audit:read',
      ],
      'compliance-officer': [
        'contracts:read',
        'disputes:read',
        'compliance:read', 'compliance:write', 'compliance:admin',
        'audit:read', 'audit:admin',
      ],
      'facility-manager': [
        'contracts:read', 'contracts:write',
        'disputes:read',
        'compliance:read',
      ],
      'viewer': [
        'contracts:read',
        'disputes:read',
        'compliance:read',
      ],
    };

    return rolePermissions[role] || [];
  }

  // ============================================================================
  // 로그인 실패 관리
  // ============================================================================

  async recordFailedAttempt(userId: string, ipAddress: string): Promise<void> {
    const key = `${userId}:${ipAddress}`;
    const existing = this.failedAttempts.get(key) || {
      count: 0,
      firstAttempt: new Date().toISOString(),
      lastAttempt: new Date().toISOString(),
    };

    existing.count++;
    existing.lastAttempt = new Date().toISOString();

    this.failedAttempts.set(key, existing);

    // 최대 실패 횟수 초과 시 IP 차단
    if (existing.count >= this.config.maxFailedAttempts) {
      await this.blockIp(ipAddress, '로그인 실패 횟수 초과');
    }
  }

  // ============================================================================
  // IP 차단 관리
  // ============================================================================

  async blockIp(ipAddress: string, reason: string): Promise<void> {
    const expiresAt = new Date(Date.now() + this.config.lockoutDurationMinutes * 60000);

    this.blockedIps.set(ipAddress, {
      ipAddress,
      reason,
      blockedAt: new Date().toISOString(),
      expiresAt: expiresAt.toISOString(),
    });
  }

  async isIpBlocked(ipAddress: string): Promise<boolean> {
    const blocked = this.blockedIps.get(ipAddress);

    if (!blocked) {
      return false;
    }

    if (new Date() > new Date(blocked.expiresAt)) {
      this.blockedIps.delete(ipAddress);
      return false;
    }

    return true;
  }

  async isIpWhitelisted(ipAddress: string): Promise<boolean> {
    // 실제 구현에서는 화이트리스트 DB 조회
    const whitelist = ['127.0.0.1', '::1', '10.0.0.0/8', '192.168.0.0/16'];
    return whitelist.some(ip => this.matchIp(ipAddress, ip));
  }

  private matchIp(ipAddress: string, pattern: string): boolean {
    // 간단한 IP 매칭 (CIDR 지원 필요)
    return ipAddress === pattern || pattern.includes('/');
  }

  // ============================================================================
  // 정리 작업
  // ============================================================================

  private cleanupExpiredSessions(): void {
    const now = new Date();
    for (const [id, session] of this.sessions) {
      if (new Date(session.expiresAt) < now) {
        this.sessions.delete(id);
      }
    }
  }

  private cleanupExpiredBlockedIps(): void {
    const now = new Date();
    for (const [ip, info] of this.blockedIps) {
      if (new Date(info.expiresAt) < now) {
        this.blockedIps.delete(ip);
      }
    }
  }

  private async validateMfaCode(userId: string, code: string): Promise<boolean> {
    // 실제 구현에서는 TOTP 검증
    return code.length === 6 && /^\d+$/.test(code);
  }

  async getActiveSessionCount(): Promise<number> {
    return this.sessions.size;
  }

  async getBlockedIpCount(): Promise<number> {
    return this.blockedIps.size;
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface Session {
  id: string;
  userId: string;
  userEmail: string;
  role: UserRole;
  permissions: Permission[];
  ipAddress: string;
  createdAt: string;
  expiresAt: string;
  lastActivity: string;
  mfaVerified: boolean;
}

export interface User {
  id: string;
  email: string;
  role: UserRole;
}

export type UserRole = 'admin' | 'legal-officer' | 'compliance-officer' | 'facility-manager' | 'viewer';

export type Permission =
  | 'contracts:read' | 'contracts:write' | 'contracts:delete' | 'contracts:admin'
  | 'disputes:read' | 'disputes:write' | 'disputes:delete' | 'disputes:admin'
  | 'compliance:read' | 'compliance:write' | 'compliance:admin'
  | 'users:read' | 'users:write' | 'users:delete' | 'users:admin'
  | 'audit:read' | 'audit:admin'
  | 'security:read' | 'security:write' | 'security:admin';

export interface AccessCheckResult {
  allowed: boolean;
  reason?: string;
  reasonCode?: string;
  session?: Session;
}

export interface BlockedIpInfo {
  ipAddress: string;
  reason: string;
  blockedAt: string;
  expiresAt: string;
}

export interface FailedAttemptInfo {
  count: number;
  firstAttempt: string;
  lastAttempt: string;
}
```

## 보안 감사 로거

```typescript
/**
 * 보안 감사 로거
 * 모든 보안 이벤트 기록 및 분석
 */

export class SecurityAuditLogger {
  private events: AuditEvent[] = [];
  private alertHandlers: AlertHandler[] = [];

  constructor(
    private readonly config: SecurityConfig['audit']
  ) {}

  async initialize(): Promise<void> {
    // 이벤트 정리 타이머 시작
    setInterval(() => this.cleanupOldEvents(), 86400000);  // 매일
  }

  // ============================================================================
  // 이벤트 로깅
  // ============================================================================

  async log(event: AuditEventInput): Promise<AuditEvent> {
    const fullEvent: AuditEvent = {
      id: crypto.randomUUID(),
      ...event,
      timestamp: new Date().toISOString(),
      severity: event.severity || this.determineSeverity(event.type),
      severityKr: this.getSeverityKr(event.severity || this.determineSeverity(event.type)),
    };

    this.events.push(fullEvent);

    // 실시간 알림 확인
    if (this.config.realTimeAlerts && this.shouldAlert(fullEvent)) {
      await this.sendAlert(fullEvent);
    }

    // 상세 로깅
    if (this.config.detailedLogging) {
      console.log(`[감사] ${fullEvent.timestamp} - ${fullEvent.type}: ${fullEvent.description}`);
    }

    return fullEvent;
  }

  // 인증 이벤트 로깅
  async logAuthentication(
    event: 'login_success' | 'login_failure' | 'logout' | 'mfa_success' | 'mfa_failure',
    userId: string,
    ipAddress: string,
    details?: Record<string, unknown>
  ): Promise<AuditEvent> {
    const typeKr: Record<string, string> = {
      'login_success': '로그인 성공',
      'login_failure': '로그인 실패',
      'logout': '로그아웃',
      'mfa_success': 'MFA 성공',
      'mfa_failure': 'MFA 실패',
    };

    return this.log({
      type: event,
      typeKr: typeKr[event],
      category: 'authentication',
      categoryKr: '인증',
      description: `사용자 ${userId} - ${typeKr[event]}`,
      userId,
      ipAddress,
      details,
    });
  }

  // 접근 이벤트 로깅
  async logAccess(
    event: 'access_granted' | 'access_denied',
    userId: string,
    resource: string,
    action: string,
    ipAddress: string,
    details?: Record<string, unknown>
  ): Promise<AuditEvent> {
    const typeKr: Record<string, string> = {
      'access_granted': '접근 허용',
      'access_denied': '접근 거부',
    };

    return this.log({
      type: event,
      typeKr: typeKr[event],
      category: 'access',
      categoryKr: '접근',
      description: `${resource}에 대한 ${action} ${typeKr[event]}`,
      userId,
      ipAddress,
      resource,
      action,
      details,
    });
  }

  // 데이터 변경 이벤트 로깅
  async logDataChange(
    event: 'data_created' | 'data_updated' | 'data_deleted',
    userId: string,
    resource: string,
    resourceId: string,
    changes?: Record<string, { old: unknown; new: unknown }>,
    ipAddress?: string
  ): Promise<AuditEvent> {
    const typeKr: Record<string, string> = {
      'data_created': '데이터 생성',
      'data_updated': '데이터 수정',
      'data_deleted': '데이터 삭제',
    };

    return this.log({
      type: event,
      typeKr: typeKr[event],
      category: 'data',
      categoryKr: '데이터',
      description: `${resource} ${resourceId} ${typeKr[event]}`,
      userId,
      ipAddress,
      resource,
      resourceId,
      details: { changes },
    });
  }

  // 보안 이벤트 로깅
  async logSecurityEvent(
    event: 'threat_detected' | 'key_rotated' | 'config_changed' | 'ip_blocked' | 'suspicious_activity',
    description: string,
    severity: AuditSeverity,
    details?: Record<string, unknown>
  ): Promise<AuditEvent> {
    const typeKr: Record<string, string> = {
      'threat_detected': '위협 탐지',
      'key_rotated': '키 순환',
      'config_changed': '설정 변경',
      'ip_blocked': 'IP 차단',
      'suspicious_activity': '의심스러운 활동',
    };

    return this.log({
      type: event,
      typeKr: typeKr[event],
      category: 'security',
      categoryKr: '보안',
      description,
      severity,
      details,
    });
  }

  // ============================================================================
  // 이벤트 조회
  // ============================================================================

  async getEvents(filter: AuditEventFilter): Promise<AuditEvent[]> {
    let filtered = [...this.events];

    if (filter.type) {
      filtered = filtered.filter(e => e.type === filter.type);
    }

    if (filter.category) {
      filtered = filtered.filter(e => e.category === filter.category);
    }

    if (filter.userId) {
      filtered = filtered.filter(e => e.userId === filter.userId);
    }

    if (filter.severity) {
      filtered = filtered.filter(e => e.severity === filter.severity);
    }

    if (filter.from) {
      filtered = filtered.filter(e => new Date(e.timestamp) >= new Date(filter.from!));
    }

    if (filter.to) {
      filtered = filtered.filter(e => new Date(e.timestamp) <= new Date(filter.to!));
    }

    // 정렬
    filtered.sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime());

    // 페이지네이션
    const offset = filter.offset || 0;
    const limit = filter.limit || 100;

    return filtered.slice(offset, offset + limit);
  }

  async getEventById(id: string): Promise<AuditEvent | null> {
    return this.events.find(e => e.id === id) || null;
  }

  // ============================================================================
  // 통계
  // ============================================================================

  async getStatistics(dateRange: { from: string; to: string }): Promise<AuditStatistics> {
    const events = await this.getEvents({
      from: dateRange.from,
      to: dateRange.to,
    });

    const byCategory = this.groupBy(events, 'category');
    const bySeverity = this.groupBy(events, 'severity');
    const byType = this.groupBy(events, 'type');

    return {
      total: events.length,
      totalKr: `총 ${events.length}건`,
      byCategory: Object.fromEntries(
        Object.entries(byCategory).map(([k, v]) => [k, v.length])
      ),
      bySeverity: Object.fromEntries(
        Object.entries(bySeverity).map(([k, v]) => [k, v.length])
      ),
      byType: Object.fromEntries(
        Object.entries(byType).map(([k, v]) => [k, v.length])
      ),
      criticalEvents: events.filter(e => e.severity === 'critical').length,
      criticalEventsKr: `심각 ${events.filter(e => e.severity === 'critical').length}건`,
    };
  }

  private groupBy<T>(array: T[], key: keyof T): Record<string, T[]> {
    return array.reduce((result, item) => {
      const k = String(item[key]);
      (result[k] = result[k] || []).push(item);
      return result;
    }, {} as Record<string, T[]>);
  }

  // ============================================================================
  // 알림
  // ============================================================================

  registerAlertHandler(handler: AlertHandler): void {
    this.alertHandlers.push(handler);
  }

  private shouldAlert(event: AuditEvent): boolean {
    // 심각도가 높거나 특정 이벤트 유형인 경우 알림
    return event.severity === 'critical' ||
           event.severity === 'high' ||
           event.type === 'threat_detected' ||
           event.type === 'access_denied';
  }

  private async sendAlert(event: AuditEvent): Promise<void> {
    const alert: SecurityAlert = {
      id: crypto.randomUUID(),
      eventId: event.id,
      type: event.type,
      typeKr: event.typeKr,
      severity: event.severity,
      severityKr: event.severityKr,
      description: event.description,
      timestamp: new Date().toISOString(),
      acknowledged: false,
    };

    for (const handler of this.alertHandlers) {
      try {
        await handler(alert);
      } catch (error) {
        console.error('알림 발송 실패:', error);
      }
    }
  }

  // ============================================================================
  // 유틸리티
  // ============================================================================

  private determineSeverity(eventType: string): AuditSeverity {
    const severityMap: Record<string, AuditSeverity> = {
      'login_failure': 'medium',
      'mfa_failure': 'medium',
      'access_denied': 'medium',
      'threat_detected': 'critical',
      'suspicious_activity': 'high',
      'ip_blocked': 'medium',
      'data_deleted': 'medium',
      'config_changed': 'medium',
      'key_rotated': 'low',
    };

    return severityMap[eventType] || 'low';
  }

  private getSeverityKr(severity: AuditSeverity): string {
    const map: Record<AuditSeverity, string> = {
      'critical': '심각',
      'high': '높음',
      'medium': '중간',
      'low': '낮음',
      'info': '정보',
    };
    return map[severity];
  }

  private cleanupOldEvents(): void {
    const cutoff = new Date();
    cutoff.setDate(cutoff.getDate() - this.config.retentionDays);

    this.events = this.events.filter(e => new Date(e.timestamp) >= cutoff);
  }

  async getTotalEventCount(): Promise<number> {
    return this.events.length;
  }

  async getTodayEventCount(): Promise<number> {
    const today = new Date();
    today.setHours(0, 0, 0, 0);

    return this.events.filter(e => new Date(e.timestamp) >= today).length;
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface AuditEvent {
  id: string;
  type: string;
  typeKr?: string;
  category: string;
  categoryKr?: string;
  description: string;
  timestamp: string;
  severity: AuditSeverity;
  severityKr?: string;
  userId?: string;
  ipAddress?: string;
  resource?: string;
  resourceId?: string;
  action?: string;
  details?: Record<string, unknown>;
}

export interface AuditEventInput {
  type: string;
  typeKr?: string;
  category: string;
  categoryKr?: string;
  description: string;
  severity?: AuditSeverity;
  userId?: string;
  ipAddress?: string;
  resource?: string;
  resourceId?: string;
  action?: string;
  details?: Record<string, unknown>;
}

export type AuditSeverity = 'critical' | 'high' | 'medium' | 'low' | 'info';

export interface AuditEventFilter {
  type?: string;
  category?: string;
  userId?: string;
  severity?: AuditSeverity;
  from?: string;
  to?: string;
  offset?: number;
  limit?: number;
}

export interface AuditStatistics {
  total: number;
  totalKr: string;
  byCategory: Record<string, number>;
  bySeverity: Record<string, number>;
  byType: Record<string, number>;
  criticalEvents: number;
  criticalEventsKr: string;
}

export interface SecurityAlert {
  id: string;
  eventId: string;
  type: string;
  typeKr?: string;
  severity: AuditSeverity;
  severityKr?: string;
  description: string;
  timestamp: string;
  acknowledged: boolean;
}

export type AlertHandler = (alert: SecurityAlert) => Promise<void>;
```

## 위협 탐지 서비스

```typescript
/**
 * 위협 탐지 서비스
 * 실시간 보안 위협 탐지 및 대응
 */

export class ThreatDetectionService {
  private threats: DetectedThreat[] = [];
  private rules: ThreatDetectionRule[] = [];
  private alerts: ThreatAlert[] = [];

  async initialize(): Promise<void> {
    this.loadDefaultRules();
    // 주기적 분석 시작
    setInterval(() => this.analyzePatterns(), 300000);  // 5분마다
  }

  private loadDefaultRules(): void {
    this.rules = [
      {
        id: 'rule-brute-force',
        name: '무차별 대입 공격 탐지',
        nameKr: '무차별 대입 공격',
        description: '짧은 시간 내 다수의 로그인 실패',
        pattern: {
          eventType: 'login_failure',
          threshold: 10,
          timeWindowMinutes: 5,
        },
        severity: 'high',
        action: 'block_ip',
      },
      {
        id: 'rule-unusual-access',
        name: '비정상적 접근 패턴',
        nameKr: '비정상 접근',
        description: '평소와 다른 시간대/위치에서 접근',
        pattern: {
          eventType: 'login_success',
          anomalyDetection: true,
        },
        severity: 'medium',
        action: 'alert',
      },
      {
        id: 'rule-data-exfiltration',
        name: '데이터 유출 시도',
        nameKr: '데이터 유출',
        description: '대량 데이터 조회/다운로드',
        pattern: {
          eventType: 'data_access',
          threshold: 1000,
          timeWindowMinutes: 60,
        },
        severity: 'critical',
        action: 'block_session',
      },
      {
        id: 'rule-privilege-escalation',
        name: '권한 상승 시도',
        nameKr: '권한 상승',
        description: '권한 없는 리소스 접근 시도',
        pattern: {
          eventType: 'access_denied',
          threshold: 5,
          timeWindowMinutes: 10,
        },
        severity: 'high',
        action: 'alert_and_block',
      },
      {
        id: 'rule-sql-injection',
        name: 'SQL 인젝션 시도',
        nameKr: 'SQL 인젝션',
        description: '악의적인 SQL 쿼리 패턴 탐지',
        pattern: {
          inputPattern: /('|--|;|\/\*|\*\/|xp_|exec|execute|insert|select|delete|update|drop|create|alter)/i,
        },
        severity: 'critical',
        action: 'block_and_alert',
      },
    ];
  }

  // 이벤트 분석
  async analyzeEvent(event: AuditEvent): Promise<ThreatAnalysisResult> {
    const matchedRules: ThreatDetectionRule[] = [];

    for (const rule of this.rules) {
      if (await this.matchesRule(event, rule)) {
        matchedRules.push(rule);
      }
    }

    if (matchedRules.length > 0) {
      const highestSeverityRule = matchedRules.reduce((a, b) =>
        this.severityLevel(a.severity) > this.severityLevel(b.severity) ? a : b
      );

      const threat = await this.recordThreat(event, highestSeverityRule);

      return {
        threatDetected: true,
        threatDetectedKr: '위협 탐지됨',
        threat,
        matchedRules,
        recommendedAction: highestSeverityRule.action,
      };
    }

    return {
      threatDetected: false,
      threatDetectedKr: '정상',
      matchedRules: [],
    };
  }

  private async matchesRule(event: AuditEvent, rule: ThreatDetectionRule): Promise<boolean> {
    const pattern = rule.pattern;

    // 이벤트 유형 매칭
    if (pattern.eventType && event.type !== pattern.eventType) {
      return false;
    }

    // 임계값 매칭
    if (pattern.threshold && pattern.timeWindowMinutes) {
      const recentEvents = await this.getRecentEvents(
        event.type,
        pattern.timeWindowMinutes,
        event.userId
      );
      return recentEvents.length >= pattern.threshold;
    }

    // 입력 패턴 매칭
    if (pattern.inputPattern && event.details?.input) {
      return pattern.inputPattern.test(String(event.details.input));
    }

    return false;
  }

  private async getRecentEvents(
    eventType: string,
    minutesAgo: number,
    userId?: string
  ): Promise<AuditEvent[]> {
    // 실제 구현에서는 감사 로그에서 조회
    return [];
  }

  private severityLevel(severity: string): number {
    const levels: Record<string, number> = {
      'critical': 4,
      'high': 3,
      'medium': 2,
      'low': 1,
      'info': 0,
    };
    return levels[severity] || 0;
  }

  private async recordThreat(event: AuditEvent, rule: ThreatDetectionRule): Promise<DetectedThreat> {
    const threat: DetectedThreat = {
      id: crypto.randomUUID(),
      ruleId: rule.id,
      ruleName: rule.name,
      ruleNameKr: rule.nameKr,
      eventId: event.id,
      severity: rule.severity,
      severityKr: this.getSeverityKr(rule.severity),
      description: rule.description,
      detectedAt: new Date().toISOString(),
      status: 'active',
      statusKr: '활성',
      userId: event.userId,
      ipAddress: event.ipAddress,
    };

    this.threats.push(threat);

    // 알림 생성
    await this.createAlert(threat);

    return threat;
  }

  private getSeverityKr(severity: string): string {
    const map: Record<string, string> = {
      'critical': '심각',
      'high': '높음',
      'medium': '중간',
      'low': '낮음',
    };
    return map[severity] || severity;
  }

  private async createAlert(threat: DetectedThreat): Promise<void> {
    const alert: ThreatAlert = {
      id: crypto.randomUUID(),
      threatId: threat.id,
      type: 'threat_detected',
      typeKr: '위협 탐지',
      severity: threat.severity,
      severityKr: threat.severityKr,
      message: `${threat.ruleNameKr}: ${threat.description}`,
      createdAt: new Date().toISOString(),
      acknowledged: false,
    };

    this.alerts.push(alert);
  }

  // 패턴 분석 (주기적 실행)
  private async analyzePatterns(): Promise<void> {
    // 이상 패턴 분석
    // 기계 학습 기반 이상 탐지 등 구현
  }

  // 위협 확인 처리
  async acknowledgeThreat(threatId: string, acknowledgedBy: string): Promise<DetectedThreat | null> {
    const threat = this.threats.find(t => t.id === threatId);

    if (threat) {
      threat.status = 'acknowledged';
      threat.statusKr = '확인됨';
      threat.acknowledgedBy = acknowledgedBy;
      threat.acknowledgedAt = new Date().toISOString();
    }

    return threat || null;
  }

  // 위협 해결 처리
  async resolveThreat(threatId: string, resolvedBy: string, resolution: string): Promise<DetectedThreat | null> {
    const threat = this.threats.find(t => t.id === threatId);

    if (threat) {
      threat.status = 'resolved';
      threat.statusKr = '해결됨';
      threat.resolvedBy = resolvedBy;
      threat.resolvedAt = new Date().toISOString();
      threat.resolution = resolution;
    }

    return threat || null;
  }

  async getDetectedThreatCount(): Promise<number> {
    return this.threats.filter(t => t.status === 'active').length;
  }

  async getActiveAlertCount(): Promise<number> {
    return this.alerts.filter(a => !a.acknowledged).length;
  }

  async getThreats(filter?: { status?: string; severity?: string }): Promise<DetectedThreat[]> {
    let filtered = [...this.threats];

    if (filter?.status) {
      filtered = filtered.filter(t => t.status === filter.status);
    }

    if (filter?.severity) {
      filtered = filtered.filter(t => t.severity === filter.severity);
    }

    return filtered.sort((a, b) =>
      new Date(b.detectedAt).getTime() - new Date(a.detectedAt).getTime()
    );
  }
}

// ============================================================================
// 타입 정의
// ============================================================================

export interface ThreatDetectionRule {
  id: string;
  name: string;
  nameKr: string;
  description: string;
  pattern: {
    eventType?: string;
    threshold?: number;
    timeWindowMinutes?: number;
    anomalyDetection?: boolean;
    inputPattern?: RegExp;
  };
  severity: 'critical' | 'high' | 'medium' | 'low';
  action: 'alert' | 'block_ip' | 'block_session' | 'alert_and_block' | 'block_and_alert';
}

export interface DetectedThreat {
  id: string;
  ruleId: string;
  ruleName: string;
  ruleNameKr: string;
  eventId: string;
  severity: string;
  severityKr: string;
  description: string;
  detectedAt: string;
  status: 'active' | 'acknowledged' | 'resolved';
  statusKr: string;
  userId?: string;
  ipAddress?: string;
  acknowledgedBy?: string;
  acknowledgedAt?: string;
  resolvedBy?: string;
  resolvedAt?: string;
  resolution?: string;
}

export interface ThreatAlert {
  id: string;
  threatId: string;
  type: string;
  typeKr: string;
  severity: string;
  severityKr: string;
  message: string;
  createdAt: string;
  acknowledged: boolean;
  acknowledgedAt?: string;
}

export interface ThreatAnalysisResult {
  threatDetected: boolean;
  threatDetectedKr: string;
  threat?: DetectedThreat;
  matchedRules: ThreatDetectionRule[];
  recommendedAction?: string;
}
```

---

## 장 요약

이 장에서는 종합 보안 프레임워크에 대해 다루었습니다:

- **암호화 서비스**: AES-256-GCM 기반 데이터 암호화 및 키 순환
- **접근 제어**: RBAC, 세션 관리, MFA, IP 차단
- **감사 로깅**: 모든 보안 이벤트 기록 및 분석
- **위협 탐지**: 실시간 위협 탐지 및 대응
- **한국 규제 준수**: 개인정보보호법, 생명윤리법, 전자서명법

---

**다음 장**: [구현 가이드 - 배포 및 운영](./08-implementation.md)
