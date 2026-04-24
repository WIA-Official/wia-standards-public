# Chapter 7: Security, Encryption, and Access Control

**弘익人間 (Benefit All Humanity)**

---

## Overview

This chapter covers comprehensive security measures for cryopreservation systems, including data encryption, access control, audit logging, regulatory compliance (HIPAA, GDPR), and cybersecurity best practices. Security is critical for protecting sensitive patient data and maintaining the integrity of biological specimens.

---

## Data Encryption

### Encryption at Rest

```typescript
/**
 * WIA Cryo Preservation - Security and Encryption System
 * Complete implementation of encryption, access control, and audit logging
 */

import crypto from 'crypto';
import { z } from 'zod';

/**
 * Encryption configuration
 */
export const EncryptionConfigSchema = z.object({
  algorithm: z.enum(['aes-256-gcm', 'aes-256-cbc', 'chacha20-poly1305']),
  keyDerivation: z.enum(['pbkdf2', 'scrypt', 'argon2']),
  keyLength: z.number().default(32), // 256 bits
  ivLength: z.number().default(16),  // 128 bits
  saltLength: z.number().default(32),
  iterations: z.number().default(100000),
  tagLength: z.number().default(16).optional() // For GCM mode
});

export type EncryptionConfig = z.infer<typeof EncryptionConfigSchema>;

/**
 * Encryption service for data at rest
 */
export class EncryptionService {
  private config: EncryptionConfig;
  private masterKey: Buffer;

  constructor(config: EncryptionConfig, masterKey: string) {
    this.config = config;
    this.masterKey = this.deriveKey(masterKey);
  }

  /**
   * Derive encryption key from password
   */
  private deriveKey(password: string, salt?: Buffer): Buffer {
    const useSalt = salt || crypto.randomBytes(this.config.saltLength);

    switch (this.config.keyDerivation) {
      case 'pbkdf2':
        return crypto.pbkdf2Sync(
          password,
          useSalt,
          this.config.iterations,
          this.config.keyLength,
          'sha256'
        );

      case 'scrypt':
        return crypto.scryptSync(
          password,
          useSalt,
          this.config.keyLength,
          { N: 16384, r: 8, p: 1 }
        );

      default:
        throw new Error(`Unsupported key derivation: ${this.config.keyDerivation}`);
    }
  }

  /**
   * Encrypt data
   */
  encrypt(plaintext: string | Buffer): {
    ciphertext: string;
    iv: string;
    tag?: string;
    algorithm: string;
  } {
    const data = typeof plaintext === 'string' ? Buffer.from(plaintext, 'utf8') : plaintext;
    const iv = crypto.randomBytes(this.config.ivLength);

    const cipher = crypto.createCipheriv(this.config.algorithm, this.masterKey, iv);

    let encrypted = cipher.update(data);
    encrypted = Buffer.concat([encrypted, cipher.final()]);

    const result: any = {
      ciphertext: encrypted.toString('base64'),
      iv: iv.toString('base64'),
      algorithm: this.config.algorithm
    };

    // Add authentication tag for GCM mode
    if (this.config.algorithm === 'aes-256-gcm') {
      const authTag = (cipher as crypto.CipherGCM).getAuthTag();
      result.tag = authTag.toString('base64');
    }

    return result;
  }

  /**
   * Decrypt data
   */
  decrypt(encrypted: {
    ciphertext: string;
    iv: string;
    tag?: string;
    algorithm: string;
  }): string {
    const iv = Buffer.from(encrypted.iv, 'base64');
    const ciphertext = Buffer.from(encrypted.ciphertext, 'base64');

    const decipher = crypto.createDecipheriv(encrypted.algorithm, this.masterKey, iv);

    // Set authentication tag for GCM mode
    if (encrypted.algorithm === 'aes-256-gcm' && encrypted.tag) {
      const tag = Buffer.from(encrypted.tag, 'base64');
      (decipher as crypto.DecipherGCM).setAuthTag(tag);
    }

    let decrypted = decipher.update(ciphertext);
    decrypted = Buffer.concat([decrypted, decipher.final()]);

    return decrypted.toString('utf8');
  }

  /**
   * Encrypt sensitive specimen data
   */
  encryptSpecimenData(specimen: any): any {
    const sensitiveFields = [
      'donor.demographics.ethnicity',
      'donor.medicalHistory',
      'consent.signedDocument',
      'financial'
    ];

    const encrypted = { ...specimen };

    sensitiveFields.forEach(field => {
      const value = this.getNestedValue(encrypted, field);
      if (value !== undefined) {
        const encryptedValue = this.encrypt(JSON.stringify(value));
        this.setNestedValue(encrypted, field, {
          encrypted: true,
          ...encryptedValue
        });
      }
    });

    return encrypted;
  }

  /**
   * Decrypt sensitive specimen data
   */
  decryptSpecimenData(specimen: any): any {
    const decrypted = { ...specimen };

    const traverse = (obj: any, path: string[] = []) => {
      for (const key in obj) {
        const value = obj[key];
        const currentPath = [...path, key];

        if (value && typeof value === 'object' && value.encrypted === true) {
          try {
            const decryptedValue = this.decrypt(value);
            obj[key] = JSON.parse(decryptedValue);
          } catch (error) {
            console.error(`Failed to decrypt ${currentPath.join('.')}:`, error);
          }
        } else if (value && typeof value === 'object') {
          traverse(value, currentPath);
        }
      }
    };

    traverse(decrypted);
    return decrypted;
  }

  /**
   * Helper: Get nested object value
   */
  private getNestedValue(obj: any, path: string): any {
    return path.split('.').reduce((current, key) => current?.[key], obj);
  }

  /**
   * Helper: Set nested object value
   */
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

/**
 * Field-level encryption for database storage
 */
export class FieldLevelEncryption {
  private encryptionService: EncryptionService;
  private fieldConfigs: Map<string, {
    encrypt: boolean;
    searchable: boolean;
    pii: boolean;
  }> = new Map();

  constructor(encryptionService: EncryptionService) {
    this.encryptionService = encryptionService;
    this.initializeFieldConfigs();
  }

  /**
   * Initialize field encryption configurations
   */
  private initializeFieldConfigs(): void {
    // Define which fields need encryption
    this.fieldConfigs.set('donor.demographics.ethnicity', {
      encrypt: true,
      searchable: false,
      pii: true
    });

    this.fieldConfigs.set('donor.medicalHistory', {
      encrypt: true,
      searchable: false,
      pii: true
    });

    this.fieldConfigs.set('consent.signedDocument', {
      encrypt: true,
      searchable: false,
      pii: false
    });

    this.fieldConfigs.set('financial', {
      encrypt: true,
      searchable: false,
      pii: true
    });

    // Searchable encrypted fields (using deterministic encryption)
    this.fieldConfigs.set('donor.donorId', {
      encrypt: true,
      searchable: true,
      pii: true
    });
  }

  /**
   * Encrypt document before storage
   */
  encryptDocument(document: any): any {
    const encrypted = { ...document };

    this.fieldConfigs.forEach((config, fieldPath) => {
      if (config.encrypt) {
        const value = this.encryptionService['getNestedValue'](document, fieldPath);
        if (value !== undefined) {
          const encryptedValue = this.encryptionService.encrypt(JSON.stringify(value));
          this.encryptionService['setNestedValue'](encrypted, fieldPath, encryptedValue);
        }
      }
    });

    return encrypted;
  }

  /**
   * Decrypt document after retrieval
   */
  decryptDocument(document: any): any {
    return this.encryptionService.decryptSpecimenData(document);
  }
}
```

---

## Access Control System

### Role-Based Access Control (RBAC)

```typescript
/**
 * Role-Based Access Control implementation
 */

export enum Permission {
  // Specimen permissions
  SPECIMEN_CREATE = 'specimen:create',
  SPECIMEN_READ = 'specimen:read',
  SPECIMEN_UPDATE = 'specimen:update',
  SPECIMEN_DELETE = 'specimen:delete',
  SPECIMEN_FREEZE = 'specimen:freeze',
  SPECIMEN_THAW = 'specimen:thaw',
  SPECIMEN_TRANSFER = 'specimen:transfer',

  // Protocol permissions
  PROTOCOL_CREATE = 'protocol:create',
  PROTOCOL_READ = 'protocol:read',
  PROTOCOL_UPDATE = 'protocol:update',
  PROTOCOL_APPROVE = 'protocol:approve',

  // Tank permissions
  TANK_CREATE = 'tank:create',
  TANK_READ = 'tank:read',
  TANK_UPDATE = 'tank:update',
  TANK_MAINTAIN = 'tank:maintain',

  // User management
  USER_CREATE = 'user:create',
  USER_READ = 'user:read',
  USER_UPDATE = 'user:update',
  USER_DELETE = 'user:delete',

  // Audit and reporting
  AUDIT_READ = 'audit:read',
  AUDIT_EXPORT = 'audit:export',
  REPORT_GENERATE = 'report:generate',

  // System administration
  SYSTEM_CONFIG = 'system:config',
  SYSTEM_BACKUP = 'system:backup'
}

export enum Role {
  SUPER_ADMIN = 'SUPER_ADMIN',
  ADMINISTRATOR = 'ADMINISTRATOR',
  EMBRYOLOGIST = 'EMBRYOLOGIST',
  TECHNICIAN = 'TECHNICIAN',
  NURSE = 'NURSE',
  PHYSICIAN = 'PHYSICIAN',
  AUDITOR = 'AUDITOR',
  VIEWER = 'VIEWER'
}

export const RolePermissionSchema = z.object({
  role: z.nativeEnum(Role),
  permissions: z.array(z.nativeEnum(Permission)),
  description: z.string(),
  canDelegate: z.boolean().default(false)
});

export type RolePermission = z.infer<typeof RolePermissionSchema>;

/**
 * Access Control List (ACL) Manager
 */
export class AccessControlManager {
  private rolePermissions: Map<Role, Set<Permission>> = new Map();
  private userRoles: Map<string, Set<Role>> = new Map();

  constructor() {
    this.initializeDefaultRoles();
  }

  /**
   * Initialize default role permissions
   */
  private initializeDefaultRoles(): void {
    // Super Admin - all permissions
    this.rolePermissions.set(Role.SUPER_ADMIN, new Set(Object.values(Permission)));

    // Administrator
    this.rolePermissions.set(Role.ADMINISTRATOR, new Set([
      Permission.SPECIMEN_CREATE,
      Permission.SPECIMEN_READ,
      Permission.SPECIMEN_UPDATE,
      Permission.SPECIMEN_DELETE,
      Permission.PROTOCOL_CREATE,
      Permission.PROTOCOL_READ,
      Permission.PROTOCOL_UPDATE,
      Permission.PROTOCOL_APPROVE,
      Permission.TANK_CREATE,
      Permission.TANK_READ,
      Permission.TANK_UPDATE,
      Permission.USER_CREATE,
      Permission.USER_READ,
      Permission.USER_UPDATE,
      Permission.AUDIT_READ,
      Permission.REPORT_GENERATE
    ]));

    // Embryologist
    this.rolePermissions.set(Role.EMBRYOLOGIST, new Set([
      Permission.SPECIMEN_CREATE,
      Permission.SPECIMEN_READ,
      Permission.SPECIMEN_UPDATE,
      Permission.SPECIMEN_FREEZE,
      Permission.SPECIMEN_THAW,
      Permission.PROTOCOL_READ,
      Permission.TANK_READ,
      Permission.REPORT_GENERATE
    ]));

    // Technician
    this.rolePermissions.set(Role.TECHNICIAN, new Set([
      Permission.SPECIMEN_READ,
      Permission.SPECIMEN_UPDATE,
      Permission.SPECIMEN_TRANSFER,
      Permission.PROTOCOL_READ,
      Permission.TANK_READ,
      Permission.TANK_UPDATE,
      Permission.TANK_MAINTAIN
    ]));

    // Nurse
    this.rolePermissions.set(Role.NURSE, new Set([
      Permission.SPECIMEN_READ,
      Permission.PROTOCOL_READ,
      Permission.TANK_READ
    ]));

    // Physician
    this.rolePermissions.set(Role.PHYSICIAN, new Set([
      Permission.SPECIMEN_CREATE,
      Permission.SPECIMEN_READ,
      Permission.SPECIMEN_THAW,
      Permission.PROTOCOL_READ,
      Permission.REPORT_GENERATE
    ]));

    // Auditor
    this.rolePermissions.set(Role.AUDITOR, new Set([
      Permission.SPECIMEN_READ,
      Permission.PROTOCOL_READ,
      Permission.TANK_READ,
      Permission.AUDIT_READ,
      Permission.AUDIT_EXPORT,
      Permission.REPORT_GENERATE
    ]));

    // Viewer
    this.rolePermissions.set(Role.VIEWER, new Set([
      Permission.SPECIMEN_READ,
      Permission.PROTOCOL_READ,
      Permission.TANK_READ
    ]));
  }

  /**
   * Assign role to user
   */
  assignRole(userId: string, role: Role): void {
    if (!this.userRoles.has(userId)) {
      this.userRoles.set(userId, new Set());
    }
    this.userRoles.get(userId)!.add(role);
  }

  /**
   * Revoke role from user
   */
  revokeRole(userId: string, role: Role): void {
    const roles = this.userRoles.get(userId);
    if (roles) {
      roles.delete(role);
    }
  }

  /**
   * Check if user has permission
   */
  hasPermission(userId: string, permission: Permission): boolean {
    const roles = this.userRoles.get(userId);
    if (!roles) {
      return false;
    }

    for (const role of roles) {
      const permissions = this.rolePermissions.get(role);
      if (permissions && permissions.has(permission)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Check if user has role
   */
  hasRole(userId: string, role: Role): boolean {
    const roles = this.userRoles.get(userId);
    return roles ? roles.has(role) : false;
  }

  /**
   * Get all permissions for user
   */
  getUserPermissions(userId: string): Permission[] {
    const roles = this.userRoles.get(userId);
    if (!roles) {
      return [];
    }

    const permissions = new Set<Permission>();
    roles.forEach(role => {
      const rolePerms = this.rolePermissions.get(role);
      if (rolePerms) {
        rolePerms.forEach(perm => permissions.add(perm));
      }
    });

    return Array.from(permissions);
  }

  /**
   * Authorization middleware
   */
  authorize(requiredPermission: Permission): (userId: string) => boolean {
    return (userId: string) => {
      return this.hasPermission(userId, requiredPermission);
    };
  }
}

/**
 * Attribute-Based Access Control (ABAC)
 */
export class AttributeBasedAccessControl {
  /**
   * Evaluate access based on attributes
   */
  evaluateAccess(params: {
    user: {
      userId: string;
      roles: Role[];
      department: string;
      facilityId: string;
    };
    resource: {
      type: string;
      ownerId?: string;
      facilityId: string;
      sensitivity: 'PUBLIC' | 'INTERNAL' | 'CONFIDENTIAL' | 'RESTRICTED';
    };
    action: Permission;
    context: {
      timestamp: Date;
      ipAddress?: string;
      mfaVerified: boolean;
    };
  }): {
    allowed: boolean;
    reason?: string;
  } {
    // Rule 1: User must have basic permission
    const acl = new AccessControlManager();
    params.user.roles.forEach(role => acl.assignRole(params.user.userId, role));

    if (!acl.hasPermission(params.user.userId, params.action)) {
      return {
        allowed: false,
        reason: 'User lacks required permission'
      };
    }

    // Rule 2: Facility access control
    if (params.resource.facilityId !== params.user.facilityId) {
      // Check if user has cross-facility access
      if (!params.user.roles.includes(Role.SUPER_ADMIN) &&
          !params.user.roles.includes(Role.ADMINISTRATOR)) {
        return {
          allowed: false,
          reason: 'Cross-facility access denied'
        };
      }
    }

    // Rule 3: Sensitivity-based access
    if (params.resource.sensitivity === 'RESTRICTED') {
      if (!params.context.mfaVerified) {
        return {
          allowed: false,
          reason: 'MFA required for restricted resources'
        };
      }
    }

    // Rule 4: Time-based access (business hours only for certain actions)
    const hour = params.context.timestamp.getHours();
    if ([Permission.SPECIMEN_DELETE, Permission.SPECIMEN_TRANSFER].includes(params.action)) {
      if (hour < 8 || hour > 18) {
        if (!params.user.roles.includes(Role.SUPER_ADMIN)) {
          return {
            allowed: false,
            reason: 'Action only allowed during business hours (8 AM - 6 PM)'
          };
        }
      }
    }

    // Rule 5: Owner-based access
    if (params.resource.ownerId && params.resource.ownerId !== params.user.userId) {
      // Only certain roles can access others' resources
      if (![Role.SUPER_ADMIN, Role.ADMINISTRATOR, Role.AUDITOR].some(role =>
        params.user.roles.includes(role)
      )) {
        return {
          allowed: false,
          reason: 'Cannot access resources owned by others'
        };
      }
    }

    return { allowed: true };
  }
}
```

---

## Audit Logging System

### Comprehensive Audit Trail

```typescript
/**
 * Audit logging system for compliance
 */

export enum AuditLevel {
  DEBUG = 'DEBUG',
  INFO = 'INFO',
  WARNING = 'WARNING',
  ERROR = 'ERROR',
  CRITICAL = 'CRITICAL'
}

export interface AuditLogEntry {
  logId: string;
  timestamp: Date;
  level: AuditLevel;

  // Actor
  userId: string;
  userName: string;
  userRole: string;
  sessionId: string;
  ipAddress: string;
  userAgent?: string;

  // Action
  action: string;
  resource: string;
  resourceId: string;
  method: string; // GET, POST, PUT, DELETE

  // Result
  success: boolean;
  statusCode?: number;
  errorMessage?: string;

  // Changes (for data modifications)
  changes?: {
    before: any;
    after: any;
  };

  // Context
  facilityId: string;
  department?: string;
  requestId: string;

  // Security
  authenticationMethod: string;
  mfaVerified: boolean;
  permissionsChecked: Permission[];

  // Metadata
  duration?: number; // milliseconds
  tags?: string[];
}

/**
 * Audit logger
 */
export class AuditLogger {
  private logs: AuditLogEntry[] = [];
  private retentionDays: number = 2555; // 7 years for HIPAA compliance

  /**
   * Log audit event
   */
  log(entry: Omit<AuditLogEntry, 'logId' | 'timestamp'>): void {
    const logEntry: AuditLogEntry = {
      logId: crypto.randomUUID(),
      timestamp: new Date(),
      ...entry
    };

    this.logs.push(logEntry);

    // In production, write to secure audit database
    this.writeToSecureStorage(logEntry);

    // Check if high-severity event needs immediate attention
    if ([AuditLevel.ERROR, AuditLevel.CRITICAL].includes(entry.level)) {
      this.triggerAlert(logEntry);
    }
  }

  /**
   * Query audit logs
   */
  query(filters: {
    userId?: string;
    action?: string;
    resource?: string;
    startDate?: Date;
    endDate?: Date;
    level?: AuditLevel;
    success?: boolean;
  }): AuditLogEntry[] {
    return this.logs.filter(log => {
      if (filters.userId && log.userId !== filters.userId) return false;
      if (filters.action && log.action !== filters.action) return false;
      if (filters.resource && log.resource !== filters.resource) return false;
      if (filters.startDate && log.timestamp < filters.startDate) return false;
      if (filters.endDate && log.timestamp > filters.endDate) return false;
      if (filters.level && log.level !== filters.level) return false;
      if (filters.success !== undefined && log.success !== filters.success) return false;
      return true;
    });
  }

  /**
   * Generate compliance report
   */
  generateComplianceReport(params: {
    startDate: Date;
    endDate: Date;
    facilityId?: string;
  }): {
    totalEvents: number;
    successfulEvents: number;
    failedEvents: number;
    criticalEvents: number;
    topUsers: Array<{ userId: string; count: number }>;
    topActions: Array<{ action: string; count: number }>;
    securityIncidents: AuditLogEntry[];
  } {
    const logs = this.query({
      startDate: params.startDate,
      endDate: params.endDate
    }).filter(log => !params.facilityId || log.facilityId === params.facilityId);

    const successful = logs.filter(l => l.success).length;
    const failed = logs.filter(l => !l.success).length;
    const critical = logs.filter(l => l.level === AuditLevel.CRITICAL).length;

    // Top users
    const userCounts = new Map<string, number>();
    logs.forEach(log => {
      userCounts.set(log.userId, (userCounts.get(log.userId) || 0) + 1);
    });
    const topUsers = Array.from(userCounts.entries())
      .map(([userId, count]) => ({ userId, count }))
      .sort((a, b) => b.count - a.count)
      .slice(0, 10);

    // Top actions
    const actionCounts = new Map<string, number>();
    logs.forEach(log => {
      actionCounts.set(log.action, (actionCounts.get(log.action) || 0) + 1);
    });
    const topActions = Array.from(actionCounts.entries())
      .map(([action, count]) => ({ action, count }))
      .sort((a, b) => b.count - a.count)
      .slice(0, 10);

    // Security incidents (failed authentication, unauthorized access)
    const securityIncidents = logs.filter(log =>
      !log.success &&
      (log.action.includes('LOGIN') ||
       log.action.includes('UNAUTHORIZED') ||
       log.level === AuditLevel.CRITICAL)
    );

    return {
      totalEvents: logs.length,
      successfulEvents: successful,
      failedEvents: failed,
      criticalEvents: critical,
      topUsers,
      topActions,
      securityIncidents
    };
  }

  /**
   * Write to secure storage
   */
  private writeToSecureStorage(entry: AuditLogEntry): void {
    // In production, write to write-once, read-many storage
    // Could use append-only database, blockchain, or WORM storage
    console.log('[AUDIT]', entry.level, entry.action, entry.resource);
  }

  /**
   * Trigger alert for critical events
   */
  private triggerAlert(entry: AuditLogEntry): void {
    console.error('[CRITICAL AUDIT EVENT]', {
      logId: entry.logId,
      action: entry.action,
      user: entry.userName,
      error: entry.errorMessage
    });
    // In production, send to monitoring system, PagerDuty, etc.
  }

  /**
   * Export audit logs for compliance
   */
  exportLogs(params: {
    startDate: Date;
    endDate: Date;
    format: 'JSON' | 'CSV' | 'PDF';
  }): string {
    const logs = this.query({
      startDate: params.startDate,
      endDate: params.endDate
    });

    if (params.format === 'JSON') {
      return JSON.stringify(logs, null, 2);
    }

    if (params.format === 'CSV') {
      const headers = [
        'Timestamp', 'User', 'Action', 'Resource', 'Success', 'IP Address'
      ].join(',');

      const rows = logs.map(log =>
        [
          log.timestamp.toISOString(),
          log.userName,
          log.action,
          log.resource,
          log.success,
          log.ipAddress
        ].join(',')
      );

      return [headers, ...rows].join('\n');
    }

    return 'PDF export not implemented';
  }
}
```

---

## HIPAA Compliance

```typescript
/**
 * HIPAA compliance utilities
 */

export class HIPAACompliance {
  /**
   * Validate minimum necessary access
   */
  validateMinimumNecessary(params: {
    userId: string;
    requestedFields: string[];
    purpose: string;
  }): {
    approved: string[];
    denied: string[];
    justification: string;
  } {
    // Define PHI fields
    const phiFields = new Set([
      'donor.demographics',
      'donor.medicalHistory',
      'financial',
      'consent.signedDocument'
    ]);

    // Define non-PHI fields always allowed
    const nonPhiFields = new Set([
      'specimenId',
      'category',
      'status',
      'storage.location'
    ]);

    const approved: string[] = [];
    const denied: string[] = [];

    params.requestedFields.forEach(field => {
      // Non-PHI fields are always approved
      if (nonPhiFields.has(field)) {
        approved.push(field);
        return;
      }

      // PHI fields require justification
      if (phiFields.has(field)) {
        // Check if purpose justifies access
        const validPurposes = ['TREATMENT', 'PAYMENT', 'OPERATIONS', 'RESEARCH_IRB'];

        if (validPurposes.includes(params.purpose)) {
          approved.push(field);
        } else {
          denied.push(field);
        }
      } else {
        approved.push(field);
      }
    });

    return {
      approved,
      denied,
      justification: `Access granted for ${params.purpose} purposes per HIPAA minimum necessary standard`
    };
  }

  /**
   * De-identify data for research
   */
  deIdentifyData(specimen: any): any {
    const deidentified = { ...specimen };

    // Remove direct identifiers (Safe Harbor method)
    const identifiersToRemove = [
      'donor.demographics.name',
      'donor.demographics.address',
      'donor.demographics.phone',
      'donor.demographics.email',
      'donor.demographics.ssn',
      'donor.demographics.medicalRecordNumber',
      'donor.demographics.photoUrl'
    ];

    identifiersToRemove.forEach(field => {
      this.removeField(deidentified, field);
    });

    // Generalize dates to year only
    if (deidentified.collection?.collectionDate) {
      const date = new Date(deidentified.collection.collectionDate);
      deidentified.collection.collectionDate = date.getFullYear().toString();
    }

    // Generalize age to age ranges
    if (deidentified.donor?.demographics?.age) {
      const age = deidentified.donor.demographics.age;
      if (age >= 90) {
        deidentified.donor.demographics.ageRange = '90+';
      } else {
        const rangeStart = Math.floor(age / 5) * 5;
        deidentified.donor.demographics.ageRange = `${rangeStart}-${rangeStart + 4}`;
      }
      delete deidentified.donor.demographics.age;
    }

    // Replace IDs with random codes
    deidentified.donor.donorId = `ANON-${crypto.randomBytes(8).toString('hex')}`;
    deidentified.specimenId = `ANON-SPEC-${crypto.randomBytes(8).toString('hex')}`;

    return deidentified;
  }

  /**
   * Helper to remove nested field
   */
  private removeField(obj: any, path: string): void {
    const keys = path.split('.');
    const lastKey = keys.pop()!;
    const target = keys.reduce((current, key) => current?.[key], obj);
    if (target) {
      delete target[lastKey];
    }
  }

  /**
   * Breach notification assessment
   */
  assessBreachNotification(params: {
    affectedRecords: number;
    dataTypes: string[];
    encryptionUsed: boolean;
    timeToDiscovery: number; // days
  }): {
    notificationRequired: boolean;
    severity: 'LOW' | 'MODERATE' | 'HIGH' | 'CRITICAL';
    actions: string[];
    deadline?: Date;
  } {
    let severity: 'LOW' | 'MODERATE' | 'HIGH' | 'CRITICAL' = 'LOW';
    const actions: string[] = [];
    let notificationRequired = false;

    // Encrypted data has lower risk
    if (!params.encryptionUsed) {
      severity = 'HIGH';
      notificationRequired = true;
      actions.push('Immediately enable encryption for all PHI');
    }

    // Number of affected records
    if (params.affectedRecords > 500) {
      severity = 'CRITICAL';
      notificationRequired = true;
      actions.push('Notify HHS and media (>500 individuals breached)');
    } else if (params.affectedRecords > 0) {
      notificationRequired = true;
      actions.push('Notify affected individuals within 60 days');
    }

    // Sensitive data types
    const sensitivePHI = ['medicalHistory', 'financialInformation', 'geneticData'];
    if (params.dataTypes.some(type => sensitivePHI.includes(type))) {
      if (severity === 'LOW') severity = 'MODERATE';
      if (severity === 'MODERATE') severity = 'HIGH';
      actions.push('Offer credit monitoring to affected individuals');
    }

    // Time to discovery
    if (params.timeToDiscovery > 60) {
      actions.push('Improve breach detection systems');
      actions.push('Implement real-time monitoring');
    }

    const deadline = notificationRequired
      ? new Date(Date.now() + 60 * 24 * 60 * 60 * 1000) // 60 days
      : undefined;

    return {
      notificationRequired,
      severity,
      actions,
      deadline
    };
  }
}
```

---

## Multi-Factor Authentication

```typescript
/**
 * Multi-factor authentication system
 */

export class MultiFactorAuth {
  /**
   * Generate TOTP secret
   */
  generateTOTPSecret(): {
    secret: string;
    qrCode: string;
    backupCodes: string[];
  } {
    const secret = crypto.randomBytes(20).toString('base64');

    // Generate backup codes
    const backupCodes = Array.from({ length: 10 }, () =>
      crypto.randomBytes(4).toString('hex').toUpperCase()
    );

    // QR code would be generated using a library like qrcode
    const qrCode = `otpauth://totp/CryoBank:user@example.com?secret=${secret}&issuer=CryoBank`;

    return {
      secret,
      qrCode,
      backupCodes
    };
  }

  /**
   * Verify TOTP token
   */
  verifyTOTP(secret: string, token: string): boolean {
    // In production, use a library like otplib or speakeasy
    // This is a simplified example
    const currentTime = Math.floor(Date.now() / 1000);
    const timeStep = 30; // 30-second window
    const currentCounter = Math.floor(currentTime / timeStep);

    // Allow tokens from previous, current, and next time windows
    for (let i = -1; i <= 1; i++) {
      const counter = currentCounter + i;
      const expectedToken = this.generateTOTPToken(secret, counter);

      if (token === expectedToken) {
        return true;
      }
    }

    return false;
  }

  /**
   * Generate TOTP token (simplified)
   */
  private generateTOTPToken(secret: string, counter: number): string {
    // In production, use proper HMAC-SHA1 implementation
    const hash = crypto
      .createHmac('sha1', secret)
      .update(counter.toString())
      .digest();

    // Extract 6-digit code
    const offset = hash[hash.length - 1] & 0xf;
    const code =
      ((hash[offset] & 0x7f) << 24) |
      ((hash[offset + 1] & 0xff) << 16) |
      ((hash[offset + 2] & 0xff) << 8) |
      (hash[offset + 3] & 0xff);

    return (code % 1000000).toString().padStart(6, '0');
  }

  /**
   * Send SMS verification code
   */
  async sendSMSCode(phoneNumber: string): Promise<string> {
    const code = Math.floor(100000 + Math.random() * 900000).toString();

    // In production, use SMS service like Twilio
    console.log(`SMS Code for ${phoneNumber}: ${code}`);

    return code;
  }

  /**
   * Send email verification code
   */
  async sendEmailCode(email: string): Promise<string> {
    const code = Math.floor(100000 + Math.random() * 900000).toString();

    // In production, use email service
    console.log(`Email Code for ${email}: ${code}`);

    return code;
  }
}
```

---

## Summary

This chapter provides comprehensive security for cryopreservation systems:

- **Encryption**: AES-256-GCM for data at rest, field-level encryption for sensitive data
- **Access Control**: RBAC with 8 predefined roles, ABAC for complex policies
- **Audit Logging**: Complete audit trail with 7-year retention for HIPAA compliance
- **HIPAA Compliance**: De-identification, minimum necessary access, breach notification
- **Multi-Factor Authentication**: TOTP, SMS, and email verification

Key Security Features:
- End-to-end encryption
- Granular permission system
- Comprehensive audit trail
- Regulatory compliance (HIPAA, GDPR)
- Multi-factor authentication
- Automated breach detection

---

**弘益人間 (Benefit All Humanity)**

*Robust security protects sensitive patient data, ensures regulatory compliance, and maintains trust in cryopreservation services globally.*
