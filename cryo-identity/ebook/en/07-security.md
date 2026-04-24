# Chapter 7: Security Framework

## Overview

Identity data in cryogenic preservation systems requires the highest levels of security protection. This chapter details comprehensive security frameworks including encryption, access control, audit logging, and compliance with healthcare data protection regulations.

## Security Architecture

### Core Security Framework

```typescript
import { createCipheriv, createDecipheriv, randomBytes, scrypt } from 'crypto';
import { promisify } from 'util';

const scryptAsync = promisify(scrypt);

// Security classification levels
type SecurityLevel = 'public' | 'internal' | 'confidential' | 'restricted' | 'top-secret';

type DataCategory =
  | 'identity-core'
  | 'biometric'
  | 'medical'
  | 'genetic'
  | 'consent'
  | 'access-log'
  | 'financial';

// Security policy configuration
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

interface TimeRestriction {
  dayOfWeek: number[];
  startTime: string;
  endTime: string;
  timezone: string;
}

interface AlertThreshold {
  metric: string;
  threshold: number;
  window: number;
  severity: 'low' | 'medium' | 'high' | 'critical';
}

// Main security manager
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
    // Load default policies
    this.policies.set('identity-core', {
      id: 'identity-core',
      name: 'Core Identity Data',
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
        minRetentionDays: 3650, // 10 years minimum
        maxRetentionDays: 36500, // 100 years max (lifetime preservation)
        archiveAfterDays: 365,
        purgeSchedule: '0 0 1 * *', // Monthly
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
        retentionDays: 2555 // 7 years
      }
    });

    this.policies.set('biometric', {
      id: 'biometric',
      name: 'Biometric Data',
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

    // Verify access
    await this.verifyAccess(context, policy);

    // Encrypt sensitive fields
    const encrypted = await this.encryptionService.encryptFields(
      data,
      policy.encryption.fieldLevel
    );

    // Log access
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

    // Verify access
    await this.verifyAccess(context, policy);

    // Decrypt data
    const decrypted = await this.encryptionService.decryptFields(
      protectedData.data,
      policy.encryption.fieldLevel,
      protectedData.metadata.keyVersion
    );

    // Log access
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
    // Check role
    const hasRequiredRole = policy.access.requiredRoles.some(
      role => context.roles.includes(role)
    );
    if (!hasRequiredRole) {
      throw new SecurityError('ACCESS_DENIED', 'Insufficient role permissions');
    }

    // Check verification level
    if (context.verificationLevel) {
      const levels: VerificationLevel[] = ['basic', 'standard', 'enhanced', 'maximum'];
      const userLevel = levels.indexOf(context.verificationLevel);
      const requiredLevel = levels.indexOf(policy.access.requiredVerificationLevel);
      if (userLevel < requiredLevel) {
        throw new SecurityError('ACCESS_DENIED', 'Insufficient verification level');
      }
    }

    // Check MFA
    if (policy.access.mfaRequired && !context.mfaVerified) {
      throw new SecurityError('MFA_REQUIRED', 'Multi-factor authentication required');
    }

    // Check IP whitelist
    if (policy.access.ipWhitelist && context.ipAddress) {
      const allowed = this.isIpAllowed(context.ipAddress, policy.access.ipWhitelist);
      if (!allowed) {
        throw new SecurityError('ACCESS_DENIED', 'IP address not whitelisted');
      }
    }

    // Check time restrictions
    if (policy.access.timeRestrictions) {
      const withinWindow = this.isWithinTimeWindow(policy.access.timeRestrictions);
      if (!withinWindow) {
        throw new SecurityError('ACCESS_DENIED', 'Access outside allowed time window');
      }
    }
  }

  private getPolicyForCategory(category: DataCategory): SecurityPolicy {
    for (const policy of this.policies.values()) {
      if (policy.dataCategories.includes(category)) {
        return policy;
      }
    }
    throw new SecurityError('POLICY_NOT_FOUND', `No policy for category: ${category}`);
  }

  private isIpAllowed(ip: string, whitelist: string[]): boolean {
    // Simplified IP check - production would use proper CIDR matching
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

### Encryption Service

```typescript
// Advanced encryption for identity data
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
      // Deterministic hash for exact match search
      const salt = await this.keyManager.getSearchSalt();
      const hash = await scryptAsync(valueStr, salt, 32);
      return (hash as Buffer).toString('base64');
    } else {
      // Random salt for each hash (more secure, but can't search)
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

// Key management
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

    // In production, fetch from KMS/Vault
    const key = await this.fetchKeyFromProvider(version);
    this.keyHistory.set(version, key);

    // Limit history size
    if (this.keyHistory.size > this.config.keyHistoryLimit) {
      const oldest = this.keyHistory.keys().next().value;
      this.keyHistory.delete(oldest);
    }

    return key;
  }

  async getSearchSalt(): Promise<Buffer> {
    // Constant salt for deterministic search hashing
    // In production, this should be securely stored
    return Buffer.from('cryo-identity-search-salt-v1');
  }

  async rotateKey(): Promise<string> {
    const newVersion = this.generateKeyVersion();
    const newKey = randomBytes(32);

    // Store in provider
    await this.storeKeyInProvider(newVersion, newKey);

    this.keyHistory.set(newVersion, newKey);
    this.currentVersion = newVersion;

    return newVersion;
  }

  private generateKeyVersion(): string {
    return `v${Date.now()}-${randomBytes(4).toString('hex')}`;
  }

  private async fetchKeyFromProvider(version: string): Promise<Buffer> {
    // Implementation would connect to actual KMS/Vault
    // For demo, return a derived key
    return randomBytes(32);
  }

  private async storeKeyInProvider(version: string, key: Buffer): Promise<void> {
    // Implementation would store in actual KMS/Vault
  }
}
```

### Security Audit Logging

```typescript
// Comprehensive security audit logging
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
  | 'authentication'
  | 'authorization'
  | 'data-access'
  | 'data-modification'
  | 'data-deletion'
  | 'verification'
  | 'consent'
  | 'export'
  | 'admin-action'
  | 'security-alert';

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

    // Sign the event for integrity
    if (this.config.signing) {
      (fullEvent as any).signature = await this.signer.sign(fullEvent);
    }

    this.buffer.push(fullEvent);

    // Check alert thresholds
    await this.checkAlertThresholds(fullEvent);

    // Flush if buffer is full
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

  async logDataProtection(params: {
    category: DataCategory;
    action: string;
    userId: string;
    timestamp: Date;
  }): Promise<string> {
    return this.log({
      eventType: 'data-modification',
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
      action: params.action
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
    // Check for patterns that should trigger alerts
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
    // In production, query the audit storage
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
    // Send alert to monitoring system
    console.error('SECURITY ALERT:', alert);

    await this.logSecurityAlert({
      alertType: alert.type,
      severity: alert.severity as 'warning' | 'error' | 'critical',
      description: `Security alert triggered for actor ${alert.actorId}`,
      remediationSteps: this.getRemediationSteps(alert.type)
    });
  }

  private getRemediationSteps(alertType: string): string[] {
    const remediations: Record<string, string[]> = {
      'brute-force-attempt': [
        'Lock the user account temporarily',
        'Notify the user via secondary channel',
        'Review IP address for blocking'
      ],
      'bulk-data-export': [
        'Review export requests',
        'Verify user authorization',
        'Check for data exfiltration indicators'
      ]
    };

    return remediations[alertType] || ['Investigate the incident'];
  }

  async flush(): Promise<void> {
    if (this.buffer.length === 0) return;

    const events = [...this.buffer];
    this.buffer = [];

    try {
      await this.storage.store(events);
    } catch (error) {
      // Re-add to buffer on failure
      this.buffer.unshift(...events);
      console.error('Failed to flush audit logs:', error);
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
      return { valid: false, reason: 'Event not found' };
    }

    if (this.config.signing) {
      const signature = (event as any).signature;
      const eventWithoutSig = { ...event };
      delete (eventWithoutSig as any).signature;

      const isValid = await this.signer.verify(eventWithoutSig, signature);
      return {
        valid: isValid,
        reason: isValid ? 'Signature verified' : 'Invalid signature'
      };
    }

    return { valid: true, reason: 'Signing not enabled' };
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

// Audit signing for tamper detection
class AuditSigner {
  private privateKey: Buffer;

  constructor() {
    // In production, load from secure key storage
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

### Threat Detection

```typescript
// Real-time threat detection for identity systems
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
  | 'alert'
  | 'block'
  | 'quarantine'
  | 'require-mfa'
  | 'lock-account'
  | 'notify-admin';

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

interface ActionResult {
  action: ThreatAction;
  executedAt: Date;
  success: boolean;
  details?: string;
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

    // Add default rules
    this.rules.set('impossible-travel', {
      id: 'impossible-travel',
      name: 'Impossible Travel Detection',
      description: 'Detects logins from geographically distant locations in short time',
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
      name: 'Credential Stuffing Detection',
      description: 'Detects multiple failed logins from same IP',
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
      name: 'Privilege Escalation Attempt',
      description: 'Detects unauthorized access to restricted resources',
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
      name: 'Data Exfiltration Detection',
      description: 'Detects unusual data export patterns',
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

    // Keep buffer size manageable
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

    if (params.targetCategory) {
      const category = event.target.category;
      if (!category || !params.targetCategory.includes(category)) {
        return null;
      }
    }

    // Check for impossible travel
    if (rule.id === 'impossible-travel') {
      return this.checkImpossibleTravel(event, params);
    }

    return [{
      type: 'pattern-match',
      value: rule.id,
      confidence: 0.9
    }];
  }

  private async checkImpossibleTravel(
    event: AuditEvent,
    params: Record<string, any>
  ): Promise<ThreatIndicator[] | null> {
    if (!event.actor.ipAddress) return null;

    const recentEvents = this.eventBuffer.filter(e =>
      e.actor.id === event.actor.id &&
      e.actor.ipAddress &&
      e.actor.ipAddress !== event.actor.ipAddress &&
      e.timestamp > new Date(Date.now() - params.timeWindowMinutes * 60 * 1000)
    );

    if (recentEvents.length === 0) return null;

    // In production, use geolocation API
    // For demo, we'll detect if IPs are from different /8 networks
    const currentPrefix = event.actor.ipAddress.split('.')[0];
    const differentLocation = recentEvents.some(e =>
      e.actor.ipAddress!.split('.')[0] !== currentPrefix
    );

    if (differentLocation) {
      return [{
        type: 'impossible-travel',
        value: {
          currentIp: event.actor.ipAddress,
          previousIps: recentEvents.map(e => e.actor.ipAddress)
        },
        confidence: 0.85
      }];
    }

    return null;
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

  private async evaluateAnomalyRule(
    rule: ThreatRule,
    event: AuditEvent
  ): Promise<ThreatIndicator[] | null> {
    // Simplified anomaly detection
    // Production would use proper statistical analysis
    const params = rule.condition.parameters;

    if (event.eventType === 'export') {
      const historicalAvg = await this.getHistoricalAverage(event.actor.id, 'export');
      const currentValue = event.details.recordCount || 1;

      const deviation = (currentValue - historicalAvg.mean) / historicalAvg.stdDev;

      if (deviation > params.deviationThreshold) {
        return [{
          type: 'anomaly',
          value: {
            metric: params.metric,
            currentValue,
            historicalMean: historicalAvg.mean,
            deviation
          },
          confidence: 0.8
        }];
      }
    }

    return null;
  }

  private async evaluateMLRule(
    rule: ThreatRule,
    event: AuditEvent
  ): Promise<ThreatIndicator[] | null> {
    if (!this.config.mlModelEndpoint) return null;

    try {
      const response = await fetch(this.config.mlModelEndpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ event, ruleId: rule.id })
      });

      const result = await response.json();

      if (result.threat && result.confidence > 0.7) {
        return [{
          type: 'ml-detection',
          value: result.details,
          confidence: result.confidence
        }];
      }
    } catch (error) {
      console.error('ML model evaluation failed:', error);
    }

    return null;
  }

  private async getHistoricalAverage(
    actorId: string,
    eventType: string
  ): Promise<{ mean: number; stdDev: number }> {
    // In production, query historical data
    return { mean: 10, stdDev: 5 };
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
          // Block IP or session
          result.details = 'Blocked access';
          break;
        case 'quarantine':
          if (this.config.quarantineEnabled) {
            // Move to quarantine state
            result.details = 'Account quarantined';
          }
          break;
        case 'require-mfa':
          // Flag for MFA requirement
          result.details = 'MFA now required';
          break;
        case 'lock-account':
          // Lock the account
          result.details = 'Account locked';
          break;
        case 'notify-admin':
          await this.alertService.notifyAdmin(threat);
          break;
      }
    } catch (error) {
      result.success = false;
      result.details = error instanceof Error ? error.message : 'Action failed';
    }

    return result;
  }
}
```

## Summary

This chapter covered:

1. **Security Architecture**: Comprehensive policy-based security framework
2. **Encryption Service**: Field-level encryption with searchable encryption support
3. **Key Management**: Secure key storage and rotation
4. **Audit Logging**: Tamper-evident logging with integrity verification
5. **Threat Detection**: Real-time pattern matching and anomaly detection

The next chapter covers implementation guidelines and deployment strategies.
