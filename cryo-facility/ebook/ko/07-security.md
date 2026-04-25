# 제7장: 보안 아키텍처

## 7.1 개요

극저온 시설의 보안은 물리적 보안, 접근 제어, 네트워크 보안, 데이터 보호, 규정 준수를 포함하는 다층 접근 방식이 필요합니다. 이 장에서는 종합적인 보안 프레임워크를 제시합니다.

```typescript
// 보안 아키텍처 개요
const securityArchitecture = {
  layers: {
    physical: '물리적 보안',
    network: '네트워크 보안',
    application: '애플리케이션 보안',
    data: '데이터 보안'
  },
  components: {
    accessControl: '접근 제어 시스템',
    encryption: '암호화 서비스',
    auditLogging: '감사 로깅',
    threatDetection: '위협 탐지',
    incidentResponse: '사고 대응'
  },
  compliance: [
    'HIPAA',           // 의료정보보호
    'GDPR',            // 개인정보보호 (EU)
    'PIPA',            // 개인정보보호 (한국)
    'FDA 21 CFR Part 11', // 전자기록
    'ISO 27001'        // 정보보안관리
  ],
  principles: [
    '최소 권한 원칙',
    '심층 방어',
    '제로 트러스트',
    '보안 기본값'
  ]
};
```

## 7.2 종합 보안 시스템

### 7.2.1 보안 시스템 코어

```typescript
import { EventEmitter } from 'events';

// 보안 이벤트 타입
export enum SecurityEventType {
  LOGIN_SUCCESS = 'login.success',
  LOGIN_FAILURE = 'login.failure',
  LOGOUT = 'logout',
  ACCESS_GRANTED = 'access.granted',
  ACCESS_DENIED = 'access.denied',
  PRIVILEGE_ESCALATION = 'privilege.escalation',
  SUSPICIOUS_ACTIVITY = 'suspicious.activity',
  DATA_ACCESS = 'data.access',
  DATA_EXPORT = 'data.export',
  CONFIG_CHANGE = 'config.change',
  SECURITY_ALERT = 'security.alert'
}

// 보안 이벤트
interface SecurityEvent {
  id: string;
  type: SecurityEventType;
  timestamp: Date;
  userId?: string;
  sessionId?: string;
  ipAddress: string;
  userAgent?: string;
  resource?: string;
  action?: string;
  outcome: 'success' | 'failure' | 'blocked';
  riskScore: number;
  details: any;
}

// 보안 정책
interface SecurityPolicy {
  id: string;
  name: string;
  enabled: boolean;
  rules: SecurityRule[];
  actions: SecurityAction[];
}

// 보안 규칙
interface SecurityRule {
  id: string;
  condition: string;
  threshold?: number;
  timeWindow?: number;
}

// 보안 조치
interface SecurityAction {
  type: 'alert' | 'block' | 'lock' | 'notify' | 'escalate';
  config: any;
}

// 극저온 시설 보안 시스템
export class CryoFacilitySecuritySystem extends EventEmitter {
  private policies: Map<string, SecurityPolicy> = new Map();
  private activeSessions: Map<string, UserSession> = new Map();
  private eventHistory: SecurityEvent[] = [];
  private blockedEntities: Set<string> = new Set();

  constructor(
    private config: SecurityConfig,
    private authService: AuthenticationService,
    private accessControl: AccessControlService,
    private encryptionService: EncryptionService,
    private auditLogger: AuditLogger
  ) {
    super();
    this.loadPolicies();
    this.startMonitoring();
  }

  // 정책 로드
  private async loadPolicies(): Promise<void> {
    // 기본 보안 정책 로드
    const defaultPolicies: SecurityPolicy[] = [
      {
        id: 'brute-force-protection',
        name: '무차별 대입 공격 방지',
        enabled: true,
        rules: [
          {
            id: 'login-failure-limit',
            condition: 'loginFailures > threshold',
            threshold: 5,
            timeWindow: 300 // 5분
          }
        ],
        actions: [
          { type: 'lock', config: { duration: 900 } }, // 15분 잠금
          { type: 'alert', config: { severity: 'high' } }
        ]
      },
      {
        id: 'session-security',
        name: '세션 보안',
        enabled: true,
        rules: [
          {
            id: 'session-timeout',
            condition: 'sessionIdleTime > threshold',
            threshold: 1800 // 30분
          },
          {
            id: 'concurrent-sessions',
            condition: 'activeSessions > threshold',
            threshold: 3
          }
        ],
        actions: [
          { type: 'notify', config: { template: 'session-warning' } }
        ]
      },
      {
        id: 'anomaly-detection',
        name: '이상 탐지',
        enabled: true,
        rules: [
          {
            id: 'unusual-access-time',
            condition: 'accessTime outside normalHours'
          },
          {
            id: 'unusual-location',
            condition: 'geoLocation != normalLocation'
          },
          {
            id: 'rapid-data-access',
            condition: 'dataAccessCount > threshold',
            threshold: 100,
            timeWindow: 60
          }
        ],
        actions: [
          { type: 'alert', config: { severity: 'medium' } },
          { type: 'escalate', config: { to: 'security-team' } }
        ]
      }
    ];

    for (const policy of defaultPolicies) {
      this.policies.set(policy.id, policy);
    }
  }

  // 모니터링 시작
  private startMonitoring(): void {
    // 세션 타임아웃 체크
    setInterval(() => {
      this.checkSessionTimeouts();
    }, 60000); // 1분마다

    // 이상 탐지
    setInterval(() => {
      this.runAnomalyDetection();
    }, 300000); // 5분마다

    // 이벤트 정리
    setInterval(() => {
      this.cleanupEventHistory();
    }, 3600000); // 1시간마다
  }

  // 보안 이벤트 기록
  async recordEvent(event: Omit<SecurityEvent, 'id' | 'timestamp'>): Promise<void> {
    const fullEvent: SecurityEvent = {
      ...event,
      id: this.generateEventId(),
      timestamp: new Date()
    };

    this.eventHistory.push(fullEvent);

    // 감사 로그
    await this.auditLogger.log({
      action: 'SECURITY_EVENT',
      eventType: event.type,
      userId: event.userId,
      details: event.details
    });

    // 정책 평가
    await this.evaluatePolicies(fullEvent);

    // 이벤트 발행
    this.emit('security-event', fullEvent);
  }

  // 정책 평가
  private async evaluatePolicies(event: SecurityEvent): Promise<void> {
    for (const policy of this.policies.values()) {
      if (!policy.enabled) continue;

      for (const rule of policy.rules) {
        if (await this.evaluateRule(rule, event)) {
          await this.executeActions(policy.actions, event);
          break;
        }
      }
    }
  }

  // 규칙 평가
  private async evaluateRule(
    rule: SecurityRule,
    event: SecurityEvent
  ): Promise<boolean> {
    switch (rule.id) {
      case 'login-failure-limit':
        return this.checkLoginFailures(
          event.ipAddress,
          rule.threshold!,
          rule.timeWindow!
        );

      case 'session-timeout':
        return this.checkSessionTimeout(
          event.sessionId!,
          rule.threshold!
        );

      case 'rapid-data-access':
        return this.checkDataAccessRate(
          event.userId!,
          rule.threshold!,
          rule.timeWindow!
        );

      default:
        return false;
    }
  }

  // 조치 실행
  private async executeActions(
    actions: SecurityAction[],
    event: SecurityEvent
  ): Promise<void> {
    for (const action of actions) {
      switch (action.type) {
        case 'alert':
          await this.sendSecurityAlert(event, action.config);
          break;

        case 'block':
          await this.blockEntity(event.ipAddress, action.config);
          break;

        case 'lock':
          await this.lockAccount(event.userId!, action.config);
          break;

        case 'notify':
          await this.sendNotification(event, action.config);
          break;

        case 'escalate':
          await this.escalateIncident(event, action.config);
          break;
      }
    }
  }

  // 로그인 실패 체크
  private checkLoginFailures(
    ipAddress: string,
    threshold: number,
    timeWindow: number
  ): boolean {
    const cutoff = Date.now() - timeWindow * 1000;
    const failures = this.eventHistory.filter(e =>
      e.type === SecurityEventType.LOGIN_FAILURE &&
      e.ipAddress === ipAddress &&
      e.timestamp.getTime() > cutoff
    );

    return failures.length >= threshold;
  }

  // 세션 타임아웃 체크
  private checkSessionTimeout(sessionId: string, threshold: number): boolean {
    const session = this.activeSessions.get(sessionId);
    if (!session) return false;

    const idleTime = (Date.now() - session.lastActivityAt.getTime()) / 1000;
    return idleTime > threshold;
  }

  // 데이터 접근 속도 체크
  private checkDataAccessRate(
    userId: string,
    threshold: number,
    timeWindow: number
  ): boolean {
    const cutoff = Date.now() - timeWindow * 1000;
    const accesses = this.eventHistory.filter(e =>
      e.type === SecurityEventType.DATA_ACCESS &&
      e.userId === userId &&
      e.timestamp.getTime() > cutoff
    );

    return accesses.length > threshold;
  }

  // 세션 타임아웃 처리
  private async checkSessionTimeouts(): Promise<void> {
    const now = Date.now();
    const timeout = this.config.sessionTimeout * 1000;

    for (const [sessionId, session] of this.activeSessions) {
      const idleTime = now - session.lastActivityAt.getTime();

      if (idleTime > timeout) {
        await this.terminateSession(sessionId, 'timeout');
      }
    }
  }

  // 이상 탐지 실행
  private async runAnomalyDetection(): Promise<void> {
    // 비정상 접근 패턴 탐지
    for (const [sessionId, session] of this.activeSessions) {
      const anomalies = await this.detectAnomalies(session);

      if (anomalies.length > 0) {
        await this.recordEvent({
          type: SecurityEventType.SUSPICIOUS_ACTIVITY,
          sessionId,
          userId: session.userId,
          ipAddress: session.ipAddress,
          outcome: 'success',
          riskScore: this.calculateRiskScore(anomalies),
          details: { anomalies }
        });
      }
    }
  }

  // 이상 탐지
  private async detectAnomalies(session: UserSession): Promise<string[]> {
    const anomalies: string[] = [];

    // 비정상 시간 접근
    const hour = new Date().getHours();
    if (hour < 6 || hour > 22) {
      anomalies.push('unusual_access_time');
    }

    // 지리적 위치 변경
    if (session.location !== session.normalLocation) {
      anomalies.push('location_change');
    }

    // 빠른 위치 변경 (불가능한 이동)
    // 추가 검사 로직...

    return anomalies;
  }

  // 위험 점수 계산
  private calculateRiskScore(anomalies: string[]): number {
    const weights: Record<string, number> = {
      'unusual_access_time': 20,
      'location_change': 30,
      'rapid_data_access': 40,
      'privilege_escalation': 50,
      'unknown_device': 25
    };

    return Math.min(
      100,
      anomalies.reduce((score, a) => score + (weights[a] || 10), 0)
    );
  }

  // 보안 알림 전송
  private async sendSecurityAlert(
    event: SecurityEvent,
    config: any
  ): Promise<void> {
    await this.emit('security-alert', {
      event,
      severity: config.severity,
      timestamp: new Date()
    });
  }

  // 엔터티 차단
  private async blockEntity(entity: string, config: any): Promise<void> {
    this.blockedEntities.add(entity);

    if (config.duration) {
      setTimeout(() => {
        this.blockedEntities.delete(entity);
      }, config.duration * 1000);
    }
  }

  // 계정 잠금
  private async lockAccount(userId: string, config: any): Promise<void> {
    await this.authService.lockAccount(userId, config.duration);
  }

  // 알림 전송
  private async sendNotification(
    event: SecurityEvent,
    config: any
  ): Promise<void> {
    // 알림 전송 로직
  }

  // 사건 에스컬레이션
  private async escalateIncident(
    event: SecurityEvent,
    config: any
  ): Promise<void> {
    // 에스컬레이션 로직
  }

  // 세션 종료
  async terminateSession(sessionId: string, reason: string): Promise<void> {
    const session = this.activeSessions.get(sessionId);
    if (!session) return;

    await this.recordEvent({
      type: SecurityEventType.LOGOUT,
      sessionId,
      userId: session.userId,
      ipAddress: session.ipAddress,
      outcome: 'success',
      riskScore: 0,
      details: { reason }
    });

    this.activeSessions.delete(sessionId);
  }

  // 이벤트 이력 정리
  private cleanupEventHistory(): void {
    const cutoff = Date.now() - 24 * 60 * 60 * 1000; // 24시간 보존
    this.eventHistory = this.eventHistory.filter(e =>
      e.timestamp.getTime() > cutoff
    );
  }

  // 차단 여부 확인
  isBlocked(entity: string): boolean {
    return this.blockedEntities.has(entity);
  }

  private generateEventId(): string {
    return `sec_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// 사용자 세션
interface UserSession {
  id: string;
  userId: string;
  ipAddress: string;
  userAgent: string;
  location?: string;
  normalLocation?: string;
  createdAt: Date;
  lastActivityAt: Date;
  permissions: string[];
}

// 보안 설정
interface SecurityConfig {
  sessionTimeout: number;
  maxLoginAttempts: number;
  lockoutDuration: number;
  mfaRequired: boolean;
  passwordPolicy: PasswordPolicy;
}

// 비밀번호 정책
interface PasswordPolicy {
  minLength: number;
  requireUppercase: boolean;
  requireLowercase: boolean;
  requireNumbers: boolean;
  requireSpecialChars: boolean;
  maxAge: number;
  preventReuse: number;
}
```

## 7.3 접근 제어 시스템

### 7.3.1 역할 기반 접근 제어 (RBAC)

```typescript
// 역할 정의
export enum CryoFacilityRole {
  ADMIN = 'admin',
  FACILITY_MANAGER = 'facility-manager',
  EQUIPMENT_OPERATOR = 'equipment-operator',
  LAB_TECHNICIAN = 'lab-technician',
  MAINTENANCE_TECHNICIAN = 'maintenance-technician',
  QUALITY_MANAGER = 'quality-manager',
  SAFETY_OFFICER = 'safety-officer',
  AUDITOR = 'auditor',
  VIEWER = 'viewer'
}

// 권한 정의
export enum Permission {
  // 시설 권한
  FACILITY_VIEW = 'facility:view',
  FACILITY_CREATE = 'facility:create',
  FACILITY_UPDATE = 'facility:update',
  FACILITY_DELETE = 'facility:delete',

  // 장비 권한
  EQUIPMENT_VIEW = 'equipment:view',
  EQUIPMENT_CREATE = 'equipment:create',
  EQUIPMENT_UPDATE = 'equipment:update',
  EQUIPMENT_DELETE = 'equipment:delete',
  EQUIPMENT_OPERATE = 'equipment:operate',
  EQUIPMENT_EMERGENCY = 'equipment:emergency',

  // 검체 권한
  SPECIMEN_VIEW = 'specimen:view',
  SPECIMEN_CREATE = 'specimen:create',
  SPECIMEN_UPDATE = 'specimen:update',
  SPECIMEN_TRANSFER = 'specimen:transfer',
  SPECIMEN_DISPOSE = 'specimen:dispose',

  // 유지보수 권한
  MAINTENANCE_VIEW = 'maintenance:view',
  MAINTENANCE_CREATE = 'maintenance:create',
  MAINTENANCE_APPROVE = 'maintenance:approve',
  MAINTENANCE_EXECUTE = 'maintenance:execute',

  // 품질 권한
  QUALITY_VIEW = 'quality:view',
  QUALITY_AUDIT = 'quality:audit',
  QUALITY_APPROVE = 'quality:approve',

  // 보고서 권한
  REPORT_VIEW = 'report:view',
  REPORT_CREATE = 'report:create',
  REPORT_EXPORT = 'report:export',

  // 관리 권한
  USER_MANAGE = 'user:manage',
  ROLE_MANAGE = 'role:manage',
  SYSTEM_CONFIG = 'system:config',
  AUDIT_VIEW = 'audit:view'
}

// 역할-권한 매핑
const rolePermissions: Record<CryoFacilityRole, Permission[]> = {
  [CryoFacilityRole.ADMIN]: Object.values(Permission),

  [CryoFacilityRole.FACILITY_MANAGER]: [
    Permission.FACILITY_VIEW, Permission.FACILITY_UPDATE,
    Permission.EQUIPMENT_VIEW, Permission.EQUIPMENT_CREATE,
    Permission.EQUIPMENT_UPDATE, Permission.EQUIPMENT_DELETE,
    Permission.EQUIPMENT_OPERATE, Permission.EQUIPMENT_EMERGENCY,
    Permission.SPECIMEN_VIEW, Permission.SPECIMEN_CREATE,
    Permission.SPECIMEN_UPDATE, Permission.SPECIMEN_TRANSFER,
    Permission.MAINTENANCE_VIEW, Permission.MAINTENANCE_CREATE,
    Permission.MAINTENANCE_APPROVE,
    Permission.QUALITY_VIEW,
    Permission.REPORT_VIEW, Permission.REPORT_CREATE, Permission.REPORT_EXPORT,
    Permission.USER_MANAGE, Permission.AUDIT_VIEW
  ],

  [CryoFacilityRole.EQUIPMENT_OPERATOR]: [
    Permission.FACILITY_VIEW,
    Permission.EQUIPMENT_VIEW, Permission.EQUIPMENT_OPERATE,
    Permission.EQUIPMENT_EMERGENCY,
    Permission.SPECIMEN_VIEW,
    Permission.MAINTENANCE_VIEW,
    Permission.REPORT_VIEW
  ],

  [CryoFacilityRole.LAB_TECHNICIAN]: [
    Permission.FACILITY_VIEW,
    Permission.EQUIPMENT_VIEW,
    Permission.SPECIMEN_VIEW, Permission.SPECIMEN_CREATE,
    Permission.SPECIMEN_UPDATE, Permission.SPECIMEN_TRANSFER,
    Permission.REPORT_VIEW
  ],

  [CryoFacilityRole.MAINTENANCE_TECHNICIAN]: [
    Permission.FACILITY_VIEW,
    Permission.EQUIPMENT_VIEW, Permission.EQUIPMENT_UPDATE,
    Permission.MAINTENANCE_VIEW, Permission.MAINTENANCE_CREATE,
    Permission.MAINTENANCE_EXECUTE,
    Permission.REPORT_VIEW
  ],

  [CryoFacilityRole.QUALITY_MANAGER]: [
    Permission.FACILITY_VIEW,
    Permission.EQUIPMENT_VIEW,
    Permission.SPECIMEN_VIEW,
    Permission.MAINTENANCE_VIEW,
    Permission.QUALITY_VIEW, Permission.QUALITY_AUDIT, Permission.QUALITY_APPROVE,
    Permission.REPORT_VIEW, Permission.REPORT_CREATE, Permission.REPORT_EXPORT,
    Permission.AUDIT_VIEW
  ],

  [CryoFacilityRole.SAFETY_OFFICER]: [
    Permission.FACILITY_VIEW,
    Permission.EQUIPMENT_VIEW, Permission.EQUIPMENT_EMERGENCY,
    Permission.MAINTENANCE_VIEW,
    Permission.QUALITY_VIEW,
    Permission.REPORT_VIEW, Permission.REPORT_CREATE,
    Permission.AUDIT_VIEW
  ],

  [CryoFacilityRole.AUDITOR]: [
    Permission.FACILITY_VIEW,
    Permission.EQUIPMENT_VIEW,
    Permission.SPECIMEN_VIEW,
    Permission.MAINTENANCE_VIEW,
    Permission.QUALITY_VIEW, Permission.QUALITY_AUDIT,
    Permission.REPORT_VIEW, Permission.REPORT_EXPORT,
    Permission.AUDIT_VIEW
  ],

  [CryoFacilityRole.VIEWER]: [
    Permission.FACILITY_VIEW,
    Permission.EQUIPMENT_VIEW,
    Permission.REPORT_VIEW
  ]
};

// 접근 제어 서비스
export class CryoAccessControlService {
  private userRoles: Map<string, CryoFacilityRole[]> = new Map();
  private userPermissions: Map<string, Permission[]> = new Map();
  private resourcePolicies: Map<string, ResourcePolicy> = new Map();

  constructor(
    private auditLogger: AuditLogger
  ) {}

  // 역할 할당
  async assignRole(
    userId: string,
    role: CryoFacilityRole,
    assignedBy: string
  ): Promise<void> {
    const roles = this.userRoles.get(userId) || [];

    if (!roles.includes(role)) {
      roles.push(role);
      this.userRoles.set(userId, roles);

      // 권한 업데이트
      this.updateUserPermissions(userId);

      await this.auditLogger.log({
        action: 'ROLE_ASSIGNED',
        userId,
        role,
        assignedBy
      });
    }
  }

  // 역할 제거
  async removeRole(
    userId: string,
    role: CryoFacilityRole,
    removedBy: string
  ): Promise<void> {
    const roles = this.userRoles.get(userId) || [];
    const index = roles.indexOf(role);

    if (index !== -1) {
      roles.splice(index, 1);
      this.userRoles.set(userId, roles);

      // 권한 업데이트
      this.updateUserPermissions(userId);

      await this.auditLogger.log({
        action: 'ROLE_REMOVED',
        userId,
        role,
        removedBy
      });
    }
  }

  // 사용자 권한 업데이트
  private updateUserPermissions(userId: string): void {
    const roles = this.userRoles.get(userId) || [];
    const permissions = new Set<Permission>();

    for (const role of roles) {
      const rolePerms = rolePermissions[role] || [];
      rolePerms.forEach(p => permissions.add(p));
    }

    this.userPermissions.set(userId, Array.from(permissions));
  }

  // 권한 확인
  hasPermission(userId: string, permission: Permission): boolean {
    const permissions = this.userPermissions.get(userId) || [];
    return permissions.includes(permission);
  }

  // 다중 권한 확인
  hasAllPermissions(userId: string, permissions: Permission[]): boolean {
    return permissions.every(p => this.hasPermission(userId, p));
  }

  // 하나 이상의 권한 확인
  hasAnyPermission(userId: string, permissions: Permission[]): boolean {
    return permissions.some(p => this.hasPermission(userId, p));
  }

  // 리소스 접근 확인
  async checkResourceAccess(
    userId: string,
    resourceType: string,
    resourceId: string,
    action: string
  ): Promise<AccessDecision> {
    // 1. 기본 권한 확인
    const permission = `${resourceType}:${action}` as Permission;
    if (!this.hasPermission(userId, permission)) {
      return {
        allowed: false,
        reason: '필요한 권한이 없습니다'
      };
    }

    // 2. 리소스 정책 확인
    const policy = this.resourcePolicies.get(`${resourceType}:${resourceId}`);
    if (policy) {
      const policyResult = await this.evaluatePolicy(userId, policy, action);
      if (!policyResult.allowed) {
        return policyResult;
      }
    }

    // 3. 추가 조건 확인 (예: 시간 제한, 위치 제한)
    // ...

    return { allowed: true };
  }

  // 정책 평가
  private async evaluatePolicy(
    userId: string,
    policy: ResourcePolicy,
    action: string
  ): Promise<AccessDecision> {
    // 명시적 거부 확인
    if (policy.deniedUsers?.includes(userId)) {
      return {
        allowed: false,
        reason: '리소스에 대한 접근이 거부되었습니다'
      };
    }

    // 허용된 사용자 확인
    if (policy.allowedUsers && !policy.allowedUsers.includes(userId)) {
      return {
        allowed: false,
        reason: '리소스에 대한 접근 권한이 없습니다'
      };
    }

    // 허용된 역할 확인
    if (policy.allowedRoles) {
      const userRoles = this.userRoles.get(userId) || [];
      const hasRole = userRoles.some(r => policy.allowedRoles!.includes(r));
      if (!hasRole) {
        return {
          allowed: false,
          reason: '필요한 역할이 없습니다'
        };
      }
    }

    // 조건 평가
    if (policy.conditions) {
      for (const condition of policy.conditions) {
        if (!await this.evaluateCondition(userId, condition)) {
          return {
            allowed: false,
            reason: `조건을 충족하지 않습니다: ${condition.description}`
          };
        }
      }
    }

    return { allowed: true };
  }

  // 조건 평가
  private async evaluateCondition(
    userId: string,
    condition: PolicyCondition
  ): Promise<boolean> {
    switch (condition.type) {
      case 'time-restriction':
        return this.isWithinTimeRange(condition.config);

      case 'ip-restriction':
        return this.isAllowedIP(condition.config);

      case 'mfa-required':
        return await this.hasMFAVerified(userId);

      default:
        return true;
    }
  }

  private isWithinTimeRange(config: any): boolean {
    const now = new Date();
    const hour = now.getHours();
    return hour >= config.startHour && hour <= config.endHour;
  }

  private isAllowedIP(config: any): boolean {
    // IP 확인 로직
    return true;
  }

  private async hasMFAVerified(userId: string): Promise<boolean> {
    // MFA 확인 로직
    return true;
  }
}

// 리소스 정책
interface ResourcePolicy {
  resourceType: string;
  resourceId: string;
  allowedUsers?: string[];
  deniedUsers?: string[];
  allowedRoles?: CryoFacilityRole[];
  conditions?: PolicyCondition[];
}

// 정책 조건
interface PolicyCondition {
  type: string;
  description: string;
  config: any;
}

// 접근 결정
interface AccessDecision {
  allowed: boolean;
  reason?: string;
}
```

### 7.3.2 다단계 인증 (MFA)

```typescript
import * as speakeasy from 'speakeasy';
import * as qrcode from 'qrcode';

// MFA 방법
export enum MFAMethod {
  TOTP = 'totp',
  SMS = 'sms',
  EMAIL = 'email',
  HARDWARE_KEY = 'hardware-key',
  BIOMETRIC = 'biometric'
}

// MFA 서비스
export class CryoMFAService {
  private userMFAConfig: Map<string, UserMFAConfig> = new Map();

  constructor(
    private smsService: SMSService,
    private emailService: EmailService,
    private auditLogger: AuditLogger
  ) {}

  // TOTP 설정
  async setupTOTP(userId: string): Promise<TOTPSetupResult> {
    const secret = speakeasy.generateSecret({
      name: `CryoFacility:${userId}`,
      issuer: 'CryoFacility'
    });

    // QR 코드 생성
    const qrCodeUrl = await qrcode.toDataURL(secret.otpauth_url!);

    // 임시 저장 (검증 후 영구 저장)
    const config: UserMFAConfig = {
      userId,
      method: MFAMethod.TOTP,
      secret: secret.base32,
      verified: false,
      backupCodes: this.generateBackupCodes()
    };

    this.userMFAConfig.set(userId, config);

    return {
      secret: secret.base32,
      qrCode: qrCodeUrl,
      backupCodes: config.backupCodes
    };
  }

  // TOTP 검증
  async verifyTOTP(userId: string, token: string): Promise<boolean> {
    const config = this.userMFAConfig.get(userId);
    if (!config || config.method !== MFAMethod.TOTP) {
      return false;
    }

    const verified = speakeasy.totp.verify({
      secret: config.secret!,
      encoding: 'base32',
      token,
      window: 1 // 30초 전후 허용
    });

    if (verified && !config.verified) {
      config.verified = true;
      this.userMFAConfig.set(userId, config);
    }

    await this.auditLogger.log({
      action: verified ? 'MFA_VERIFIED' : 'MFA_FAILED',
      userId,
      method: MFAMethod.TOTP
    });

    return verified;
  }

  // SMS 인증 코드 발송
  async sendSMSCode(userId: string, phoneNumber: string): Promise<void> {
    const code = this.generateNumericCode(6);
    const expiresAt = new Date(Date.now() + 5 * 60 * 1000); // 5분 유효

    const config: UserMFAConfig = {
      userId,
      method: MFAMethod.SMS,
      code,
      codeExpiresAt: expiresAt,
      verified: false
    };

    this.userMFAConfig.set(userId, config);

    await this.smsService.send({
      to: phoneNumber,
      message: `CryoFacility 인증 코드: ${code}. 5분 내에 입력해주세요.`
    });

    await this.auditLogger.log({
      action: 'MFA_CODE_SENT',
      userId,
      method: MFAMethod.SMS
    });
  }

  // SMS 코드 검증
  async verifySMSCode(userId: string, code: string): Promise<boolean> {
    const config = this.userMFAConfig.get(userId);
    if (!config || config.method !== MFAMethod.SMS) {
      return false;
    }

    if (config.codeExpiresAt && new Date() > config.codeExpiresAt) {
      return false;
    }

    const verified = config.code === code;

    if (verified) {
      config.verified = true;
      this.userMFAConfig.set(userId, config);
    }

    await this.auditLogger.log({
      action: verified ? 'MFA_VERIFIED' : 'MFA_FAILED',
      userId,
      method: MFAMethod.SMS
    });

    return verified;
  }

  // 이메일 인증 코드 발송
  async sendEmailCode(userId: string, email: string): Promise<void> {
    const code = this.generateNumericCode(6);
    const expiresAt = new Date(Date.now() + 10 * 60 * 1000); // 10분 유효

    const config: UserMFAConfig = {
      userId,
      method: MFAMethod.EMAIL,
      code,
      codeExpiresAt: expiresAt,
      verified: false
    };

    this.userMFAConfig.set(userId, config);

    await this.emailService.send({
      to: email,
      subject: 'CryoFacility 인증 코드',
      body: `인증 코드: ${code}\n\n이 코드는 10분간 유효합니다.`
    });

    await this.auditLogger.log({
      action: 'MFA_CODE_SENT',
      userId,
      method: MFAMethod.EMAIL
    });
  }

  // 백업 코드 사용
  async useBackupCode(userId: string, code: string): Promise<boolean> {
    const config = this.userMFAConfig.get(userId);
    if (!config || !config.backupCodes) {
      return false;
    }

    const index = config.backupCodes.indexOf(code);
    if (index === -1) {
      return false;
    }

    // 사용된 코드 제거
    config.backupCodes.splice(index, 1);
    this.userMFAConfig.set(userId, config);

    await this.auditLogger.log({
      action: 'MFA_BACKUP_CODE_USED',
      userId,
      remainingCodes: config.backupCodes.length
    });

    return true;
  }

  // 백업 코드 생성
  private generateBackupCodes(count: number = 10): string[] {
    const codes: string[] = [];
    for (let i = 0; i < count; i++) {
      codes.push(this.generateAlphanumericCode(8));
    }
    return codes;
  }

  private generateNumericCode(length: number): string {
    return Array.from({ length }, () =>
      Math.floor(Math.random() * 10)
    ).join('');
  }

  private generateAlphanumericCode(length: number): string {
    const chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789';
    return Array.from({ length }, () =>
      chars.charAt(Math.floor(Math.random() * chars.length))
    ).join('');
  }
}

// 사용자 MFA 설정
interface UserMFAConfig {
  userId: string;
  method: MFAMethod;
  secret?: string;
  code?: string;
  codeExpiresAt?: Date;
  verified: boolean;
  backupCodes?: string[];
}

// TOTP 설정 결과
interface TOTPSetupResult {
  secret: string;
  qrCode: string;
  backupCodes: string[];
}
```

## 7.4 데이터 보호

### 7.4.1 암호화 서비스

```typescript
import * as crypto from 'crypto';

// 암호화 설정
interface EncryptionConfig {
  algorithm: string;
  keyLength: number;
  ivLength: number;
  saltLength: number;
  iterations: number;
  masterKeyId: string;
}

// 데이터 보호 서비스
export class CryoDataProtectionService {
  private masterKeys: Map<string, Buffer> = new Map();
  private dataKeys: Map<string, DataKey> = new Map();

  constructor(
    private config: EncryptionConfig,
    private keyManagementService: KeyManagementService,
    private auditLogger: AuditLogger
  ) {
    this.loadMasterKeys();
  }

  // 마스터 키 로드
  private async loadMasterKeys(): Promise<void> {
    const keys = await this.keyManagementService.getMasterKeys();
    for (const key of keys) {
      this.masterKeys.set(key.id, key.material);
    }
  }

  // 데이터 키 생성
  async generateDataKey(purpose: string): Promise<DataKey> {
    const keyMaterial = crypto.randomBytes(this.config.keyLength);
    const keyId = this.generateKeyId();

    // 마스터 키로 데이터 키 암호화
    const masterKey = this.masterKeys.get(this.config.masterKeyId);
    if (!masterKey) {
      throw new Error('마스터 키를 찾을 수 없습니다');
    }

    const encryptedKey = this.encryptWithKey(keyMaterial, masterKey);

    const dataKey: DataKey = {
      id: keyId,
      purpose,
      encryptedMaterial: encryptedKey,
      plaintextMaterial: keyMaterial,
      createdAt: new Date(),
      expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000) // 24시간
    };

    this.dataKeys.set(keyId, dataKey);

    await this.auditLogger.log({
      action: 'DATA_KEY_GENERATED',
      keyId,
      purpose
    });

    return dataKey;
  }

  // 데이터 암호화
  encrypt(data: Buffer | string, keyId?: string): EncryptedData {
    const dataKey = keyId
      ? this.dataKeys.get(keyId)
      : Array.from(this.dataKeys.values())[0];

    if (!dataKey || !dataKey.plaintextMaterial) {
      throw new Error('유효한 데이터 키가 없습니다');
    }

    const iv = crypto.randomBytes(this.config.ivLength);
    const cipher = crypto.createCipheriv(
      this.config.algorithm,
      dataKey.plaintextMaterial,
      iv
    );

    const inputBuffer = typeof data === 'string'
      ? Buffer.from(data, 'utf-8')
      : data;

    const encrypted = Buffer.concat([
      cipher.update(inputBuffer),
      cipher.final()
    ]);

    const authTag = cipher.getAuthTag();

    return {
      keyId: dataKey.id,
      iv: iv.toString('base64'),
      ciphertext: encrypted.toString('base64'),
      authTag: authTag.toString('base64'),
      algorithm: this.config.algorithm
    };
  }

  // 데이터 복호화
  decrypt(encryptedData: EncryptedData): Buffer {
    const dataKey = this.dataKeys.get(encryptedData.keyId);
    if (!dataKey || !dataKey.plaintextMaterial) {
      throw new Error('복호화 키를 찾을 수 없습니다');
    }

    const iv = Buffer.from(encryptedData.iv, 'base64');
    const ciphertext = Buffer.from(encryptedData.ciphertext, 'base64');
    const authTag = Buffer.from(encryptedData.authTag, 'base64');

    const decipher = crypto.createDecipheriv(
      this.config.algorithm,
      dataKey.plaintextMaterial,
      iv
    );

    decipher.setAuthTag(authTag);

    return Buffer.concat([
      decipher.update(ciphertext),
      decipher.final()
    ]);
  }

  // 필드 레벨 암호화
  encryptFields<T extends object>(
    data: T,
    fieldsToEncrypt: (keyof T)[]
  ): T & { _encrypted: EncryptedFieldInfo } {
    const result = { ...data } as T & { _encrypted: EncryptedFieldInfo };
    const encryptedFields: Record<string, EncryptedData> = {};

    for (const field of fieldsToEncrypt) {
      const value = data[field];
      if (value !== undefined && value !== null) {
        const encrypted = this.encrypt(JSON.stringify(value));
        encryptedFields[field as string] = encrypted;
        (result as any)[field] = '[ENCRYPTED]';
      }
    }

    result._encrypted = {
      fields: encryptedFields,
      encryptedAt: new Date()
    };

    return result;
  }

  // 필드 레벨 복호화
  decryptFields<T extends object>(
    data: T & { _encrypted: EncryptedFieldInfo }
  ): T {
    const result = { ...data } as any;
    delete result._encrypted;

    for (const [field, encrypted] of Object.entries(data._encrypted.fields)) {
      const decrypted = this.decrypt(encrypted);
      result[field] = JSON.parse(decrypted.toString('utf-8'));
    }

    return result as T;
  }

  // 데이터 해싱 (비가역)
  hash(data: string, salt?: string): HashedData {
    const useSalt = salt || crypto.randomBytes(this.config.saltLength).toString('hex');

    const hash = crypto.pbkdf2Sync(
      data,
      useSalt,
      this.config.iterations,
      64,
      'sha512'
    );

    return {
      hash: hash.toString('hex'),
      salt: useSalt,
      algorithm: 'pbkdf2-sha512',
      iterations: this.config.iterations
    };
  }

  // 해시 검증
  verifyHash(data: string, hashedData: HashedData): boolean {
    const computed = this.hash(data, hashedData.salt);
    return crypto.timingSafeEqual(
      Buffer.from(computed.hash, 'hex'),
      Buffer.from(hashedData.hash, 'hex')
    );
  }

  // 키로 암호화 (내부 사용)
  private encryptWithKey(data: Buffer, key: Buffer): Buffer {
    const iv = crypto.randomBytes(this.config.ivLength);
    const cipher = crypto.createCipheriv('aes-256-gcm', key, iv);

    const encrypted = Buffer.concat([
      cipher.update(data),
      cipher.final()
    ]);

    const authTag = cipher.getAuthTag();

    return Buffer.concat([iv, authTag, encrypted]);
  }

  private generateKeyId(): string {
    return `key_${Date.now()}_${crypto.randomBytes(8).toString('hex')}`;
  }
}

// 데이터 키
interface DataKey {
  id: string;
  purpose: string;
  encryptedMaterial: Buffer;
  plaintextMaterial?: Buffer;
  createdAt: Date;
  expiresAt: Date;
}

// 암호화된 데이터
interface EncryptedData {
  keyId: string;
  iv: string;
  ciphertext: string;
  authTag: string;
  algorithm: string;
}

// 암호화된 필드 정보
interface EncryptedFieldInfo {
  fields: Record<string, EncryptedData>;
  encryptedAt: Date;
}

// 해시된 데이터
interface HashedData {
  hash: string;
  salt: string;
  algorithm: string;
  iterations: number;
}
```

## 7.5 규정 준수

### 7.5.1 규정 준수 관리 시스템

```typescript
// 규정 프레임워크
export enum ComplianceFramework {
  HIPAA = 'hipaa',
  GDPR = 'gdpr',
  FDA_21_CFR_11 = 'fda-21-cfr-11',
  ISO_27001 = 'iso-27001',
  PIPA = 'pipa',
  KISA = 'kisa'
}

// 규정 요구사항
interface ComplianceRequirement {
  id: string;
  framework: ComplianceFramework;
  category: string;
  title: string;
  description: string;
  controls: ComplianceControl[];
}

// 규정 통제
interface ComplianceControl {
  id: string;
  name: string;
  description: string;
  implementation: string;
  evidence: string[];
  testProcedure: string;
}

// 규정 준수 관리 시스템
export class CryoComplianceManagementSystem {
  private requirements: Map<string, ComplianceRequirement> = new Map();
  private assessments: Map<string, ComplianceAssessment> = new Map();

  constructor(
    private auditLogger: AuditLogger,
    private documentService: DocumentService
  ) {
    this.loadRequirements();
  }

  // 요구사항 로드
  private async loadRequirements(): Promise<void> {
    // FDA 21 CFR Part 11 요구사항
    const fda21CFR11: ComplianceRequirement[] = [
      {
        id: 'fda-11.10',
        framework: ComplianceFramework.FDA_21_CFR_11,
        category: '전자기록',
        title: '전자기록 통제',
        description: '전자기록의 무결성, 정확성, 신뢰성 보장',
        controls: [
          {
            id: 'ctrl-11.10-a',
            name: '시스템 유효성 검증',
            description: '시스템이 의도된 기능을 수행하는지 검증',
            implementation: '유효성 검증 프로토콜 및 테스트',
            evidence: ['IQ/OQ/PQ 문서', '테스트 결과'],
            testProcedure: '연간 시스템 검증 수행'
          },
          {
            id: 'ctrl-11.10-b',
            name: '기록 생성 능력',
            description: '정확하고 완전한 기록 사본 생성 능력',
            implementation: '감사 추적 및 보고서 생성 기능',
            evidence: ['감사 추적 로그', '보고서 샘플'],
            testProcedure: '분기별 기록 생성 테스트'
          },
          {
            id: 'ctrl-11.10-c',
            name: '기록 보호',
            description: '기록의 무결성 및 기밀성 보호',
            implementation: '암호화 및 접근 제어',
            evidence: ['암호화 정책', '접근 로그'],
            testProcedure: '침투 테스트 및 암호화 검증'
          },
          {
            id: 'ctrl-11.10-d',
            name: '접근 제한',
            description: '권한 있는 사용자만 시스템 접근',
            implementation: 'RBAC 및 MFA',
            evidence: ['사용자 권한 목록', 'MFA 로그'],
            testProcedure: '월간 접근 권한 검토'
          },
          {
            id: 'ctrl-11.10-e',
            name: '감사 추적',
            description: '모든 생성, 수정, 삭제에 대한 감사 추적',
            implementation: '완전한 감사 로깅 시스템',
            evidence: ['감사 로그', '변경 이력'],
            testProcedure: '감사 추적 무결성 검증'
          }
        ]
      },
      {
        id: 'fda-11.50',
        framework: ComplianceFramework.FDA_21_CFR_11,
        category: '전자서명',
        title: '전자서명 요구사항',
        description: '전자서명의 고유성, 무결성 보장',
        controls: [
          {
            id: 'ctrl-11.50-a',
            name: '서명 고유성',
            description: '각 전자서명이 한 개인에게 고유',
            implementation: '개인 인증서 또는 인증 수단',
            evidence: ['인증서 발급 기록', '서명 로그'],
            testProcedure: '서명 고유성 검증'
          }
        ]
      }
    ];

    // HIPAA 요구사항
    const hipaaRequirements: ComplianceRequirement[] = [
      {
        id: 'hipaa-164.312',
        framework: ComplianceFramework.HIPAA,
        category: '기술적 보호조치',
        title: '기술적 보호조치',
        description: 'ePHI 보호를 위한 기술적 정책 및 절차',
        controls: [
          {
            id: 'ctrl-164.312-a',
            name: '접근 제어',
            description: 'ePHI에 대한 접근 허용 및 제한',
            implementation: '역할 기반 접근 제어',
            evidence: ['접근 정책', '권한 매트릭스'],
            testProcedure: '접근 제어 테스트'
          },
          {
            id: 'ctrl-164.312-b',
            name: '감사 통제',
            description: 'ePHI 접근 활동 기록 및 검사',
            implementation: '감사 로깅 시스템',
            evidence: ['감사 로그', '검토 보고서'],
            testProcedure: '감사 로그 검토'
          },
          {
            id: 'ctrl-164.312-c',
            name: '무결성 통제',
            description: 'ePHI의 부적절한 변경 방지',
            implementation: '무결성 검증 및 변경 추적',
            evidence: ['체크섬 로그', '변경 기록'],
            testProcedure: '무결성 검증 테스트'
          },
          {
            id: 'ctrl-164.312-d',
            name: '전송 보안',
            description: '네트워크 전송 시 ePHI 보호',
            implementation: 'TLS 암호화',
            evidence: ['SSL 인증서', '전송 로그'],
            testProcedure: '암호화 강도 테스트'
          }
        ]
      }
    ];

    // 요구사항 등록
    [...fda21CFR11, ...hipaaRequirements].forEach(req => {
      this.requirements.set(req.id, req);
    });
  }

  // 규정 준수 평가 수행
  async performAssessment(
    frameworks: ComplianceFramework[],
    assessedBy: string
  ): Promise<ComplianceAssessment> {
    const assessment: ComplianceAssessment = {
      id: this.generateAssessmentId(),
      frameworks,
      assessedBy,
      assessedAt: new Date(),
      status: 'in-progress',
      results: [],
      overallScore: 0
    };

    // 선택된 프레임워크의 요구사항 평가
    for (const req of this.requirements.values()) {
      if (frameworks.includes(req.framework)) {
        const result = await this.assessRequirement(req);
        assessment.results.push(result);
      }
    }

    // 전체 점수 계산
    const totalControls = assessment.results.reduce(
      (sum, r) => sum + r.controlResults.length, 0
    );
    const compliantControls = assessment.results.reduce(
      (sum, r) => sum + r.controlResults.filter(c => c.status === 'compliant').length, 0
    );

    assessment.overallScore = Math.round(
      (compliantControls / totalControls) * 100
    );
    assessment.status = 'completed';

    this.assessments.set(assessment.id, assessment);

    await this.auditLogger.log({
      action: 'COMPLIANCE_ASSESSMENT_COMPLETED',
      assessmentId: assessment.id,
      frameworks,
      score: assessment.overallScore,
      assessedBy
    });

    return assessment;
  }

  // 요구사항 평가
  private async assessRequirement(
    requirement: ComplianceRequirement
  ): Promise<RequirementAssessmentResult> {
    const controlResults: ControlAssessmentResult[] = [];

    for (const control of requirement.controls) {
      const result = await this.assessControl(control);
      controlResults.push(result);
    }

    const compliantCount = controlResults.filter(
      c => c.status === 'compliant'
    ).length;

    return {
      requirementId: requirement.id,
      title: requirement.title,
      status: compliantCount === controlResults.length
        ? 'compliant'
        : compliantCount === 0
          ? 'non-compliant'
          : 'partial',
      controlResults,
      score: Math.round(
        (compliantCount / controlResults.length) * 100
      )
    };
  }

  // 통제 평가
  private async assessControl(
    control: ComplianceControl
  ): Promise<ControlAssessmentResult> {
    // 증거 수집 및 검증 로직
    const evidenceFound = await this.collectEvidence(control.evidence);
    const testResult = await this.executeTestProcedure(control.testProcedure);

    const status: ComplianceStatus =
      evidenceFound && testResult.passed
        ? 'compliant'
        : !evidenceFound && !testResult.passed
          ? 'non-compliant'
          : 'partial';

    return {
      controlId: control.id,
      name: control.name,
      status,
      evidenceCollected: evidenceFound,
      testResult,
      findings: testResult.findings,
      recommendations: this.generateRecommendations(status, control)
    };
  }

  // 증거 수집
  private async collectEvidence(evidenceTypes: string[]): Promise<boolean> {
    // 실제 구현에서는 문서 시스템에서 증거 조회
    return true;
  }

  // 테스트 절차 실행
  private async executeTestProcedure(procedure: string): Promise<{
    passed: boolean;
    findings: string[];
  }> {
    // 실제 구현에서는 자동화된 테스트 수행
    return {
      passed: true,
      findings: []
    };
  }

  // 권고사항 생성
  private generateRecommendations(
    status: ComplianceStatus,
    control: ComplianceControl
  ): string[] {
    if (status === 'compliant') return [];

    return [
      `${control.name}에 대한 구현 검토 필요`,
      `필요한 증거 문서 업데이트`,
      `테스트 절차 재실행 권장`
    ];
  }

  // 규정 준수 보고서 생성
  async generateReport(assessmentId: string): Promise<ComplianceReport> {
    const assessment = this.assessments.get(assessmentId);
    if (!assessment) {
      throw new Error('평가를 찾을 수 없습니다');
    }

    const report: ComplianceReport = {
      id: this.generateReportId(),
      assessmentId,
      generatedAt: new Date(),
      summary: {
        frameworks: assessment.frameworks,
        overallScore: assessment.overallScore,
        status: assessment.overallScore >= 90
          ? 'compliant'
          : assessment.overallScore >= 70
            ? 'partial'
            : 'non-compliant',
        totalRequirements: assessment.results.length,
        compliantRequirements: assessment.results.filter(
          r => r.status === 'compliant'
        ).length
      },
      details: assessment.results,
      recommendations: this.aggregateRecommendations(assessment),
      nextSteps: this.generateNextSteps(assessment)
    };

    return report;
  }

  private aggregateRecommendations(
    assessment: ComplianceAssessment
  ): string[] {
    const recommendations: string[] = [];

    for (const result of assessment.results) {
      for (const control of result.controlResults) {
        recommendations.push(...control.recommendations);
      }
    }

    return [...new Set(recommendations)];
  }

  private generateNextSteps(assessment: ComplianceAssessment): string[] {
    const steps: string[] = [];

    if (assessment.overallScore < 100) {
      steps.push('비준수 항목에 대한 시정 조치 계획 수립');
      steps.push('30일 내 후속 평가 일정 수립');
    }

    steps.push('평가 결과 경영진 보고');
    steps.push('필요 시 외부 감사 일정 조정');

    return steps;
  }

  private generateAssessmentId(): string {
    return `assess_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateReportId(): string {
    return `report_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

// 규정 준수 상태
type ComplianceStatus = 'compliant' | 'partial' | 'non-compliant';

// 규정 준수 평가
interface ComplianceAssessment {
  id: string;
  frameworks: ComplianceFramework[];
  assessedBy: string;
  assessedAt: Date;
  status: 'in-progress' | 'completed';
  results: RequirementAssessmentResult[];
  overallScore: number;
}

// 요구사항 평가 결과
interface RequirementAssessmentResult {
  requirementId: string;
  title: string;
  status: ComplianceStatus;
  controlResults: ControlAssessmentResult[];
  score: number;
}

// 통제 평가 결과
interface ControlAssessmentResult {
  controlId: string;
  name: string;
  status: ComplianceStatus;
  evidenceCollected: boolean;
  testResult: {
    passed: boolean;
    findings: string[];
  };
  findings: string[];
  recommendations: string[];
}

// 규정 준수 보고서
interface ComplianceReport {
  id: string;
  assessmentId: string;
  generatedAt: Date;
  summary: {
    frameworks: ComplianceFramework[];
    overallScore: number;
    status: ComplianceStatus;
    totalRequirements: number;
    compliantRequirements: number;
  };
  details: RequirementAssessmentResult[];
  recommendations: string[];
  nextSteps: string[];
}
```

## 7.6 요약

이 장에서는 극저온 시설의 종합적인 보안 아키텍처를 다루었습니다:

| 보안 영역 | 주요 구성요소 | 기능 |
|----------|--------------|------|
| 접근 제어 | RBAC, MFA | 역할 기반 권한, 다단계 인증 |
| 데이터 보호 | 암호화, 해싱 | AES-GCM, PBKDF2 |
| 위협 탐지 | 보안 모니터링 | 이상 탐지, 정책 기반 대응 |
| 규정 준수 | 준수 관리 | FDA, HIPAA, GDPR 준수 |

핵심 보안 원칙:
- 최소 권한 원칙 적용
- 심층 방어 전략
- 제로 트러스트 아키텍처
- 완전한 감사 추적

다음 장에서는 구현 및 운영에 대해 살펴봅니다.

---

© 2025 WIA Standards. All rights reserved.
