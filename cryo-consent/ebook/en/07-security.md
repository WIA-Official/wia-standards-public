# Chapter 7: Security

## Consent Data Security Framework

This chapter defines comprehensive security measures for protecting consent data, ensuring confidentiality, integrity, and availability across potentially indefinite time horizons.

---

## 7.1 Security Architecture

```typescript
// Security architecture configuration
interface ConsentSecurityArchitecture {
  layers: SecurityLayer[];
  components: SecurityComponent[];
  policies: SecurityPolicy[];
  monitoring: SecurityMonitoring;
}

const securityArchitecture: ConsentSecurityArchitecture = {
  layers: [
    {
      name: 'PERIMETER',
      controls: ['WAF', 'DDoS_PROTECTION', 'API_GATEWAY', 'RATE_LIMITING'],
    },
    {
      name: 'NETWORK',
      controls: ['SEGMENTATION', 'ENCRYPTION_IN_TRANSIT', 'FIREWALL', 'IDS_IPS'],
    },
    {
      name: 'APPLICATION',
      controls: ['AUTHENTICATION', 'AUTHORIZATION', 'INPUT_VALIDATION', 'SESSION_MANAGEMENT'],
    },
    {
      name: 'DATA',
      controls: ['ENCRYPTION_AT_REST', 'TOKENIZATION', 'ACCESS_CONTROL', 'AUDIT_LOGGING'],
    },
    {
      name: 'ENDPOINT',
      controls: ['DEVICE_SECURITY', 'ENDPOINT_DETECTION', 'SECURE_BOOT'],
    },
  ],

  components: [
    {
      name: 'CryptographicService',
      responsibilities: ['ENCRYPTION', 'KEY_MANAGEMENT', 'DIGITAL_SIGNATURES', 'HASHING'],
    },
    {
      name: 'AccessControlService',
      responsibilities: ['AUTHENTICATION', 'AUTHORIZATION', 'RBAC', 'ABAC'],
    },
    {
      name: 'AuditService',
      responsibilities: ['LOGGING', 'MONITORING', 'ALERTING', 'FORENSICS'],
    },
    {
      name: 'IntegrityService',
      responsibilities: ['DATA_VALIDATION', 'TAMPER_DETECTION', 'CHECKSUMS'],
    },
  ],

  policies: [
    {
      name: 'DATA_CLASSIFICATION',
      levels: ['PUBLIC', 'INTERNAL', 'CONFIDENTIAL', 'SECRET', 'TOP_SECRET'],
    },
    {
      name: 'RETENTION',
      defaultPeriod: 'INDEFINITE',
      minimumPeriod: '100_YEARS',
    },
    {
      name: 'ACCESS_CONTROL',
      model: 'RBAC_PLUS_ABAC',
      defaultDeny: true,
    },
  ],

  monitoring: {
    realTimeAlerts: true,
    anomalyDetection: true,
    intrusionDetection: true,
    complianceMonitoring: true,
  },
};

// Security service implementation
class ConsentSecurityService {
  private cryptoService: CryptographicService;
  private accessControl: AccessControlService;
  private auditService: SecurityAuditService;
  private integrityService: IntegrityService;
  private threatDetection: ThreatDetectionService;

  constructor(config: SecurityConfig) {
    this.cryptoService = new CryptographicService(config.crypto);
    this.accessControl = new AccessControlService(config.access);
    this.auditService = new SecurityAuditService(config.audit);
    this.integrityService = new IntegrityService(config.integrity);
    this.threatDetection = new ThreatDetectionService(config.threat);
  }

  // Secure consent data
  async secureConsent(consent: ConsentRecord): Promise<SecuredConsent> {
    // Classify data
    const classification = this.classifyConsentData(consent);

    // Encrypt sensitive fields
    const encrypted = await this.encryptSensitiveData(consent, classification);

    // Generate integrity proofs
    const integrityProofs = await this.generateIntegrityProofs(encrypted);

    // Create access control policy
    const accessPolicy = this.createAccessPolicy(consent, classification);

    return {
      ...encrypted,
      security: {
        classification,
        integrityProofs,
        accessPolicy,
        securedAt: new Date(),
        securedBy: this.getServiceIdentity(),
      },
    };
  }

  private classifyConsentData(consent: ConsentRecord): DataClassification {
    const classifications: FieldClassification[] = [];

    // Patient ID - Confidential
    classifications.push({
      field: 'patientId',
      level: 'CONFIDENTIAL',
      reason: 'PII - Patient identifier',
    });

    // Decisions - Secret (medical preferences)
    classifications.push({
      field: 'decisions',
      level: 'SECRET',
      reason: 'Sensitive medical decisions',
    });

    // Authority/witnesses - Confidential
    classifications.push({
      field: 'authority',
      level: 'CONFIDENTIAL',
      reason: 'Contains PII of witnesses and proxies',
    });

    // Documents - varies
    for (const doc of consent.documents || []) {
      classifications.push({
        field: `documents.${doc.documentId}`,
        level: this.classifyDocument(doc),
        reason: 'Document content classification',
      });
    }

    // Overall classification is highest of components
    const overallLevel = this.getHighestClassification(
      classifications.map(c => c.level)
    );

    return {
      overall: overallLevel,
      fields: classifications,
      classifiedAt: new Date(),
    };
  }

  private async encryptSensitiveData(
    consent: ConsentRecord,
    classification: DataClassification
  ): Promise<ConsentRecord> {
    const encrypted = { ...consent };

    // Encrypt based on classification
    for (const fieldClass of classification.fields) {
      if (['CONFIDENTIAL', 'SECRET', 'TOP_SECRET'].includes(fieldClass.level)) {
        const fieldValue = this.getFieldValue(consent, fieldClass.field);
        const encryptedValue = await this.cryptoService.encryptField(
          fieldValue,
          {
            algorithm: this.getAlgorithmForLevel(fieldClass.level),
            keyId: await this.cryptoService.getActiveKeyId(fieldClass.level),
          }
        );
        this.setFieldValue(encrypted, fieldClass.field, encryptedValue);
      }
    }

    return encrypted;
  }

  private getAlgorithmForLevel(level: string): string {
    const algorithms: Record<string, string> = {
      CONFIDENTIAL: 'AES-256-GCM',
      SECRET: 'AES-256-GCM',
      TOP_SECRET: 'CHACHA20-POLY1305', // Post-quantum considerations
    };
    return algorithms[level] || 'AES-256-GCM';
  }
}

interface SecuredConsent extends ConsentRecord {
  security: {
    classification: DataClassification;
    integrityProofs: IntegrityProof[];
    accessPolicy: AccessPolicy;
    securedAt: Date;
    securedBy: string;
  };
}

interface DataClassification {
  overall: string;
  fields: FieldClassification[];
  classifiedAt: Date;
}

interface FieldClassification {
  field: string;
  level: string;
  reason: string;
}
```

---

## 7.2 Cryptographic Services

```typescript
// Cryptographic service for consent data protection
class CryptographicService {
  private keyManagement: KeyManagementService;
  private hsmClient: HSMClient;
  private config: CryptoConfig;

  constructor(config: CryptoConfig) {
    this.config = config;
    this.keyManagement = new KeyManagementService(config.keyManagement);
    this.hsmClient = new HSMClient(config.hsm);
  }

  // Encrypt data with envelope encryption
  async encrypt(
    data: Buffer,
    options: EncryptionOptions
  ): Promise<EncryptedData> {
    // Generate data encryption key (DEK)
    const dek = await this.generateDEK();

    // Encrypt data with DEK
    const encryptedData = await this.symmetricEncrypt(data, dek, options.algorithm);

    // Wrap DEK with key encryption key (KEK)
    const wrappedDek = await this.wrapKey(dek, options.keyId);

    return {
      ciphertext: encryptedData.ciphertext,
      iv: encryptedData.iv,
      authTag: encryptedData.authTag,
      wrappedKey: wrappedDek,
      algorithm: options.algorithm,
      keyId: options.keyId,
      encryptedAt: new Date(),
    };
  }

  // Decrypt data
  async decrypt(encrypted: EncryptedData): Promise<Buffer> {
    // Unwrap DEK
    const dek = await this.unwrapKey(encrypted.wrappedKey, encrypted.keyId);

    // Decrypt data
    const plaintext = await this.symmetricDecrypt(
      encrypted.ciphertext,
      dek,
      encrypted.iv,
      encrypted.authTag,
      encrypted.algorithm
    );

    return plaintext;
  }

  // Encrypt individual field
  async encryptField(
    value: any,
    options: FieldEncryptionOptions
  ): Promise<EncryptedField> {
    const serialized = JSON.stringify(value);
    const buffer = Buffer.from(serialized, 'utf-8');

    const encrypted = await this.encrypt(buffer, {
      algorithm: options.algorithm,
      keyId: options.keyId,
    });

    return {
      encrypted: true,
      ...encrypted,
    };
  }

  // Decrypt field
  async decryptField(encrypted: EncryptedField): Promise<any> {
    const buffer = await this.decrypt(encrypted);
    const serialized = buffer.toString('utf-8');
    return JSON.parse(serialized);
  }

  // Symmetric encryption
  private async symmetricEncrypt(
    data: Buffer,
    key: Buffer,
    algorithm: string
  ): Promise<{ ciphertext: Buffer; iv: Buffer; authTag: Buffer }> {
    const iv = crypto.randomBytes(12); // 96-bit IV for GCM

    const cipher = crypto.createCipheriv(
      this.mapAlgorithm(algorithm),
      key,
      iv,
      { authTagLength: 16 }
    );

    const ciphertext = Buffer.concat([
      cipher.update(data),
      cipher.final(),
    ]);

    const authTag = cipher.getAuthTag();

    return { ciphertext, iv, authTag };
  }

  // Symmetric decryption
  private async symmetricDecrypt(
    ciphertext: Buffer,
    key: Buffer,
    iv: Buffer,
    authTag: Buffer,
    algorithm: string
  ): Promise<Buffer> {
    const decipher = crypto.createDecipheriv(
      this.mapAlgorithm(algorithm),
      key,
      iv,
      { authTagLength: 16 }
    );

    decipher.setAuthTag(authTag);

    const plaintext = Buffer.concat([
      decipher.update(ciphertext),
      decipher.final(),
    ]);

    return plaintext;
  }

  // Key wrapping using HSM
  private async wrapKey(dek: Buffer, kekId: string): Promise<Buffer> {
    return this.hsmClient.wrapKey(dek, kekId);
  }

  // Key unwrapping using HSM
  private async unwrapKey(wrappedKey: Buffer, kekId: string): Promise<Buffer> {
    return this.hsmClient.unwrapKey(wrappedKey, kekId);
  }

  // Generate data encryption key
  private async generateDEK(): Promise<Buffer> {
    return crypto.randomBytes(32); // 256-bit key
  }

  // Get active key ID for classification level
  async getActiveKeyId(level: string): Promise<string> {
    return this.keyManagement.getActiveKey(level);
  }

  // Digital signatures
  async sign(data: Buffer, keyId: string): Promise<DigitalSignature> {
    const signature = await this.hsmClient.sign(data, keyId);

    return {
      signature,
      keyId,
      algorithm: 'RSA-PSS-SHA256',
      signedAt: new Date(),
    };
  }

  async verify(
    data: Buffer,
    signature: DigitalSignature
  ): Promise<SignatureVerification> {
    const valid = await this.hsmClient.verify(
      data,
      signature.signature,
      signature.keyId
    );

    return {
      valid,
      keyId: signature.keyId,
      verifiedAt: new Date(),
    };
  }

  // Hash functions
  async hash(data: Buffer, algorithm: string = 'SHA-256'): Promise<string> {
    const hash = crypto.createHash(algorithm.toLowerCase().replace('-', ''));
    hash.update(data);
    return hash.digest('hex');
  }

  // Multiple hash algorithms for longevity
  async multiHash(data: Buffer): Promise<MultiHash> {
    const [sha256, sha3_256, blake2b] = await Promise.all([
      this.hash(data, 'SHA-256'),
      this.hash(data, 'SHA3-256'),
      this.hashBlake2b(data),
    ]);

    return {
      sha256,
      sha3_256,
      blake2b,
      computedAt: new Date(),
    };
  }

  private async hashBlake2b(data: Buffer): Promise<string> {
    // Using blake2b-256 for future-proofing
    const blake2b = await import('blake2b');
    return blake2b.default(32).update(data).digest('hex');
  }

  private mapAlgorithm(algorithm: string): string {
    const mapping: Record<string, string> = {
      'AES-256-GCM': 'aes-256-gcm',
      'AES-128-GCM': 'aes-128-gcm',
      'CHACHA20-POLY1305': 'chacha20-poly1305',
    };
    return mapping[algorithm] || 'aes-256-gcm';
  }
}

// Key management service
class KeyManagementService {
  private keyStore: KeyStore;
  private rotationScheduler: KeyRotationScheduler;

  constructor(config: KeyManagementConfig) {
    this.keyStore = new KeyStore(config.store);
    this.rotationScheduler = new KeyRotationScheduler(config.rotation);
  }

  // Get active key for classification level
  async getActiveKey(level: string): Promise<string> {
    const keys = await this.keyStore.getKeys({ level, status: 'ACTIVE' });
    if (keys.length === 0) {
      throw new Error(`No active key found for level ${level}`);
    }
    return keys[0].keyId;
  }

  // Create new key
  async createKey(options: CreateKeyOptions): Promise<KeyMetadata> {
    const keyId = generateKeyId();

    const keyMetadata: KeyMetadata = {
      keyId,
      purpose: options.purpose,
      algorithm: options.algorithm,
      level: options.level,
      status: 'ACTIVE',
      createdAt: new Date(),
      expiresAt: this.calculateExpiration(options),
      rotationPolicy: options.rotationPolicy,
    };

    // Generate key in HSM
    await this.hsmClient.generateKey(keyId, options);

    // Store metadata
    await this.keyStore.create(keyMetadata);

    // Schedule rotation
    if (options.rotationPolicy) {
      await this.rotationScheduler.schedule(keyId, options.rotationPolicy);
    }

    return keyMetadata;
  }

  // Rotate key
  async rotateKey(keyId: string): Promise<KeyRotationResult> {
    const oldKey = await this.keyStore.get(keyId);
    if (!oldKey) {
      throw new Error(`Key ${keyId} not found`);
    }

    // Create new key
    const newKey = await this.createKey({
      purpose: oldKey.purpose,
      algorithm: oldKey.algorithm,
      level: oldKey.level,
      rotationPolicy: oldKey.rotationPolicy,
    });

    // Mark old key as rotated
    await this.keyStore.update(keyId, {
      status: 'ROTATED',
      rotatedAt: new Date(),
      rotatedTo: newKey.keyId,
    });

    // Re-encrypt data with new key (async background process)
    await this.scheduleReEncryption(keyId, newKey.keyId);

    return {
      oldKeyId: keyId,
      newKeyId: newKey.keyId,
      rotatedAt: new Date(),
    };
  }

  // Archive key (for data encrypted with old keys)
  async archiveKey(keyId: string): Promise<void> {
    await this.keyStore.update(keyId, {
      status: 'ARCHIVED',
      archivedAt: new Date(),
    });

    // Ensure key material is preserved for decryption
    await this.hsmClient.archiveKey(keyId);
  }

  // Destroy key (only after all data re-encrypted)
  async destroyKey(keyId: string): Promise<void> {
    // Verify no data still encrypted with this key
    const usage = await this.getKeyUsage(keyId);
    if (usage.activeEncryptions > 0) {
      throw new Error(`Cannot destroy key ${keyId}: still has active encryptions`);
    }

    await this.keyStore.update(keyId, {
      status: 'DESTROYED',
      destroyedAt: new Date(),
    });

    // Securely destroy key material in HSM
    await this.hsmClient.destroyKey(keyId);
  }
}

interface EncryptedData {
  ciphertext: Buffer;
  iv: Buffer;
  authTag: Buffer;
  wrappedKey: Buffer;
  algorithm: string;
  keyId: string;
  encryptedAt: Date;
}

interface EncryptedField extends EncryptedData {
  encrypted: true;
}

interface DigitalSignature {
  signature: Buffer;
  keyId: string;
  algorithm: string;
  signedAt: Date;
}

interface MultiHash {
  sha256: string;
  sha3_256: string;
  blake2b: string;
  computedAt: Date;
}

interface KeyMetadata {
  keyId: string;
  purpose: string;
  algorithm: string;
  level: string;
  status: 'ACTIVE' | 'ROTATED' | 'ARCHIVED' | 'DESTROYED';
  createdAt: Date;
  expiresAt?: Date;
  rotatedAt?: Date;
  rotatedTo?: string;
  rotationPolicy?: KeyRotationPolicy;
}
```

---

## 7.3 Access Control

```typescript
// Access control service
class AccessControlService {
  private rbacService: RBACService;
  private abacService: ABACService;
  private sessionManager: SessionManager;
  private mfaService: MFAService;

  constructor(config: AccessControlConfig) {
    this.rbacService = new RBACService(config.rbac);
    this.abacService = new ABACService(config.abac);
    this.sessionManager = new SessionManager(config.session);
    this.mfaService = new MFAService(config.mfa);
  }

  // Check access permission
  async checkAccess(request: AccessRequest): Promise<AccessDecision> {
    // Validate session
    const session = await this.sessionManager.validate(request.sessionToken);
    if (!session) {
      return {
        allowed: false,
        reason: 'INVALID_SESSION',
      };
    }

    // Check MFA if required
    if (this.requiresMFA(request)) {
      const mfaValid = await this.mfaService.verify(request.mfaToken);
      if (!mfaValid) {
        return {
          allowed: false,
          reason: 'MFA_REQUIRED',
        };
      }
    }

    // RBAC check
    const rbacResult = await this.rbacService.checkPermission(
      session.userId,
      request.resource,
      request.action
    );

    if (!rbacResult.allowed) {
      return {
        allowed: false,
        reason: 'RBAC_DENIED',
        details: rbacResult.reason,
      };
    }

    // ABAC check for fine-grained control
    const abacResult = await this.abacService.evaluate({
      subject: {
        id: session.userId,
        roles: session.roles,
        attributes: session.attributes,
      },
      resource: {
        type: request.resourceType,
        id: request.resourceId,
        attributes: await this.getResourceAttributes(request),
      },
      action: request.action,
      context: {
        time: new Date(),
        ipAddress: request.ipAddress,
        location: request.location,
      },
    });

    if (!abacResult.allowed) {
      return {
        allowed: false,
        reason: 'ABAC_DENIED',
        details: abacResult.reason,
      };
    }

    // Access granted
    return {
      allowed: true,
      constraints: [...rbacResult.constraints, ...abacResult.constraints],
      auditId: await this.auditAccess(request, session, true),
    };
  }

  // RBAC service
  private rbacService = {
    roles: new Map<string, Role>(),
    permissions: new Map<string, Permission[]>(),

    async checkPermission(
      userId: string,
      resource: string,
      action: string
    ): Promise<RBACResult> {
      const userRoles = await this.getUserRoles(userId);

      for (const role of userRoles) {
        const permissions = this.permissions.get(role.id) || [];
        const hasPermission = permissions.some(
          p => this.matchesPermission(p, resource, action)
        );
        if (hasPermission) {
          return {
            allowed: true,
            role: role.id,
            constraints: role.constraints,
          };
        }
      }

      return {
        allowed: false,
        reason: 'No role grants this permission',
      };
    },

    matchesPermission(
      permission: Permission,
      resource: string,
      action: string
    ): boolean {
      const resourceMatch =
        permission.resource === '*' ||
        permission.resource === resource ||
        this.wildcardMatch(permission.resource, resource);

      const actionMatch =
        permission.actions.includes('*') ||
        permission.actions.includes(action);

      return resourceMatch && actionMatch;
    },
  };

  // Define consent-specific roles
  private defineRoles(): void {
    // Patient role
    this.rbacService.roles.set('PATIENT', {
      id: 'PATIENT',
      name: 'Patient',
      permissions: [
        { resource: 'consent:own', actions: ['read', 'create', 'update', 'revoke'] },
        { resource: 'proxy:own', actions: ['read', 'create', 'update', 'remove'] },
        { resource: 'document:own', actions: ['read', 'upload', 'download'] },
      ],
      constraints: ['own_data_only'],
    });

    // Proxy role
    this.rbacService.roles.set('PROXY', {
      id: 'PROXY',
      name: 'Designated Proxy',
      permissions: [
        { resource: 'consent:delegated', actions: ['read'] },
        { resource: 'decision:delegated', actions: ['read', 'make'] },
        { resource: 'document:delegated', actions: ['read'] },
      ],
      constraints: ['delegated_authority_only', 'active_proxy_only'],
    });

    // Medical staff role
    this.rbacService.roles.set('MEDICAL_STAFF', {
      id: 'MEDICAL_STAFF',
      name: 'Medical Staff',
      permissions: [
        { resource: 'consent:patient', actions: ['read'] },
        { resource: 'decision:patient', actions: ['query'] },
        { resource: 'document:patient', actions: ['read'] },
      ],
      constraints: ['treatment_relationship', 'need_to_know'],
    });

    // Administrator role
    this.rbacService.roles.set('ADMIN', {
      id: 'ADMIN',
      name: 'System Administrator',
      permissions: [
        { resource: 'consent:*', actions: ['read', 'audit'] },
        { resource: 'user:*', actions: ['read', 'create', 'update', 'disable'] },
        { resource: 'system:*', actions: ['configure', 'monitor'] },
      ],
      constraints: ['audit_logged', 'mfa_required'],
    });

    // Auditor role
    this.rbacService.roles.set('AUDITOR', {
      id: 'AUDITOR',
      name: 'Compliance Auditor',
      permissions: [
        { resource: 'audit:*', actions: ['read', 'export'] },
        { resource: 'consent:*', actions: ['read'] },
        { resource: 'report:*', actions: ['read', 'generate'] },
      ],
      constraints: ['read_only', 'audit_logged'],
    });
  }
}

// ABAC policy evaluation
class ABACService {
  private policyEngine: PolicyEngine;
  private attributeProvider: AttributeProvider;

  constructor(config: ABACConfig) {
    this.policyEngine = new PolicyEngine(config.policies);
    this.attributeProvider = new AttributeProvider(config.attributes);
  }

  async evaluate(request: ABACRequest): Promise<ABACResult> {
    // Enrich with additional attributes
    const enrichedRequest = await this.enrichRequest(request);

    // Evaluate policies
    const decision = await this.policyEngine.evaluate(enrichedRequest);

    return decision;
  }

  private async enrichRequest(request: ABACRequest): Promise<ABACRequest> {
    // Get additional subject attributes
    const subjectAttrs = await this.attributeProvider.getSubjectAttributes(
      request.subject.id
    );

    // Get resource attributes
    const resourceAttrs = await this.attributeProvider.getResourceAttributes(
      request.resource.type,
      request.resource.id
    );

    // Get environmental attributes
    const envAttrs = await this.attributeProvider.getEnvironmentalAttributes();

    return {
      ...request,
      subject: {
        ...request.subject,
        attributes: { ...request.subject.attributes, ...subjectAttrs },
      },
      resource: {
        ...request.resource,
        attributes: { ...request.resource.attributes, ...resourceAttrs },
      },
      context: {
        ...request.context,
        environment: envAttrs,
      },
    };
  }
}

// Policy definitions for consent access
const consentAccessPolicies: ABACPolicy[] = [
  {
    id: 'PATIENT_OWN_CONSENT',
    description: 'Patients can access their own consent records',
    target: {
      subjects: [{ attribute: 'role', value: 'PATIENT' }],
      resources: [{ attribute: 'type', value: 'consent' }],
      actions: ['read', 'create', 'update', 'revoke'],
    },
    condition: {
      expression: 'subject.id == resource.patientId',
    },
    effect: 'PERMIT',
  },
  {
    id: 'PROXY_DELEGATED_ACCESS',
    description: 'Active proxies can access delegated patient records',
    target: {
      subjects: [{ attribute: 'role', value: 'PROXY' }],
      resources: [{ attribute: 'type', value: 'consent' }],
      actions: ['read'],
    },
    condition: {
      expression: `
        subject.proxyStatus == 'ACTIVE' &&
        resource.patientId IN subject.delegatedPatients &&
        resource.category IN subject.authorizedCategories
      `,
    },
    effect: 'PERMIT',
  },
  {
    id: 'MEDICAL_EMERGENCY_ACCESS',
    description: 'Medical staff can access consent in emergencies',
    target: {
      subjects: [{ attribute: 'role', value: 'MEDICAL_STAFF' }],
      resources: [{ attribute: 'type', value: 'consent' }],
      actions: ['read', 'query_decision'],
    },
    condition: {
      expression: `
        context.emergencyDeclared == true &&
        subject.facility == resource.facility &&
        action == 'read' || action == 'query_decision'
      `,
    },
    effect: 'PERMIT',
    obligations: [
      { action: 'LOG_EMERGENCY_ACCESS' },
      { action: 'NOTIFY_PRIVACY_OFFICER' },
    ],
  },
  {
    id: 'TIME_BASED_RESTRICTION',
    description: 'Restrict access outside business hours for non-emergency',
    target: {
      subjects: [{ attribute: 'role', value: '*' }],
      resources: [{ attribute: 'type', value: 'consent' }],
      actions: ['*'],
    },
    condition: {
      expression: `
        context.emergencyDeclared == false &&
        (context.hour < 6 || context.hour > 22) &&
        subject.role != 'PATIENT' &&
        subject.role != 'PROXY'
      `,
    },
    effect: 'DENY',
  },
];

interface AccessRequest {
  sessionToken: string;
  mfaToken?: string;
  resource: string;
  resourceType: string;
  resourceId: string;
  action: string;
  ipAddress: string;
  location?: string;
}

interface AccessDecision {
  allowed: boolean;
  reason?: string;
  details?: string;
  constraints?: string[];
  auditId?: string;
}

interface ABACRequest {
  subject: {
    id: string;
    roles: string[];
    attributes: Record<string, any>;
  };
  resource: {
    type: string;
    id: string;
    attributes: Record<string, any>;
  };
  action: string;
  context: {
    time: Date;
    ipAddress: string;
    location?: string;
    environment?: Record<string, any>;
  };
}

interface ABACPolicy {
  id: string;
  description: string;
  target: PolicyTarget;
  condition: PolicyCondition;
  effect: 'PERMIT' | 'DENY';
  obligations?: PolicyObligation[];
}
```

---

## 7.4 Audit and Logging

```typescript
// Security audit service
class SecurityAuditService {
  private auditLog: AuditLogStore;
  private eventPublisher: EventPublisher;
  private alertService: AlertService;
  private integrityService: AuditIntegrityService;

  constructor(config: AuditConfig) {
    this.auditLog = new AuditLogStore(config.store);
    this.eventPublisher = new EventPublisher(config.events);
    this.alertService = new AlertService(config.alerts);
    this.integrityService = new AuditIntegrityService(config.integrity);
  }

  // Log security event
  async logSecurityEvent(event: SecurityEvent): Promise<AuditRecord> {
    // Create audit record
    const record: AuditRecord = {
      recordId: generateAuditId(),
      timestamp: new Date(),
      eventType: event.type,
      severity: event.severity,

      subject: {
        userId: event.userId,
        sessionId: event.sessionId,
        roles: event.roles,
        ipAddress: event.ipAddress,
        userAgent: event.userAgent,
      },

      resource: event.resource,
      action: event.action,
      outcome: event.outcome,

      details: event.details,
      context: event.context,

      // Integrity chain
      previousHash: await this.auditLog.getLastHash(),
    };

    // Calculate record hash
    record.recordHash = await this.calculateRecordHash(record);

    // Store record
    await this.auditLog.append(record);

    // Publish event for real-time monitoring
    await this.eventPublisher.publish('SECURITY_AUDIT', record);

    // Check for alerts
    await this.checkAlertConditions(record);

    return record;
  }

  // Log consent access
  async logConsentAccess(access: ConsentAccessEvent): Promise<AuditRecord> {
    return this.logSecurityEvent({
      type: 'CONSENT_ACCESS',
      severity: 'INFO',
      userId: access.userId,
      sessionId: access.sessionId,
      roles: access.roles,
      ipAddress: access.ipAddress,
      userAgent: access.userAgent,
      resource: {
        type: 'CONSENT',
        id: access.consentId,
        patientId: access.patientId,
      },
      action: access.action,
      outcome: access.outcome,
      details: {
        fieldsAccessed: access.fieldsAccessed,
        decisionQueried: access.decisionQueried,
        purpose: access.purpose,
      },
      context: access.context,
    });
  }

  // Log authentication event
  async logAuthentication(auth: AuthenticationEvent): Promise<AuditRecord> {
    return this.logSecurityEvent({
      type: 'AUTHENTICATION',
      severity: auth.success ? 'INFO' : 'WARNING',
      userId: auth.userId,
      sessionId: auth.sessionId,
      roles: [],
      ipAddress: auth.ipAddress,
      userAgent: auth.userAgent,
      resource: { type: 'AUTH_SYSTEM' },
      action: auth.method,
      outcome: auth.success ? 'SUCCESS' : 'FAILURE',
      details: {
        method: auth.method,
        mfaUsed: auth.mfaUsed,
        failureReason: auth.failureReason,
      },
      context: auth.context,
    });
  }

  // Log authorization failure
  async logAuthorizationFailure(failure: AuthorizationFailure): Promise<AuditRecord> {
    const record = await this.logSecurityEvent({
      type: 'AUTHORIZATION_FAILURE',
      severity: 'WARNING',
      userId: failure.userId,
      sessionId: failure.sessionId,
      roles: failure.roles,
      ipAddress: failure.ipAddress,
      userAgent: failure.userAgent,
      resource: failure.resource,
      action: failure.attemptedAction,
      outcome: 'DENIED',
      details: {
        reason: failure.reason,
        policyViolated: failure.policyId,
      },
      context: failure.context,
    });

    // Alert on repeated failures
    await this.checkRepeatedFailures(failure.userId, failure.ipAddress);

    return record;
  }

  // Log data modification
  async logDataModification(modification: DataModificationEvent): Promise<AuditRecord> {
    return this.logSecurityEvent({
      type: 'DATA_MODIFICATION',
      severity: 'INFO',
      userId: modification.userId,
      sessionId: modification.sessionId,
      roles: modification.roles,
      ipAddress: modification.ipAddress,
      userAgent: modification.userAgent,
      resource: modification.resource,
      action: modification.action,
      outcome: modification.outcome,
      details: {
        previousValue: modification.previousValueHash, // Hash only, not actual data
        newValue: modification.newValueHash,
        fieldsModified: modification.fieldsModified,
        changeReason: modification.reason,
      },
      context: modification.context,
    });
  }

  // Calculate integrity hash for audit record
  private async calculateRecordHash(record: Omit<AuditRecord, 'recordHash'>): Promise<string> {
    const canonical = this.canonicalize(record);
    return this.integrityService.hash(canonical);
  }

  // Verify audit log integrity
  async verifyIntegrity(
    startDate?: Date,
    endDate?: Date
  ): Promise<IntegrityVerificationResult> {
    const records = await this.auditLog.getRecords({ startDate, endDate });

    let previousHash = '';
    const violations: IntegrityViolation[] = [];

    for (let i = 0; i < records.length; i++) {
      const record = records[i];

      // Verify hash chain
      if (i > 0 && record.previousHash !== previousHash) {
        violations.push({
          recordId: record.recordId,
          type: 'CHAIN_BREAK',
          expected: previousHash,
          actual: record.previousHash,
        });
      }

      // Verify record hash
      const calculatedHash = await this.calculateRecordHash({
        ...record,
        recordHash: undefined,
      });
      if (calculatedHash !== record.recordHash) {
        violations.push({
          recordId: record.recordId,
          type: 'HASH_MISMATCH',
          expected: record.recordHash,
          actual: calculatedHash,
        });
      }

      previousHash = record.recordHash;
    }

    return {
      verified: violations.length === 0,
      recordsChecked: records.length,
      violations,
      verifiedAt: new Date(),
    };
  }

  // Check for alert conditions
  private async checkAlertConditions(record: AuditRecord): Promise<void> {
    // Multiple failed logins
    if (record.eventType === 'AUTHENTICATION' && record.outcome === 'FAILURE') {
      const recentFailures = await this.countRecentFailures(
        record.subject.userId,
        record.subject.ipAddress,
        15 * 60 * 1000 // 15 minutes
      );

      if (recentFailures >= 5) {
        await this.alertService.trigger({
          type: 'BRUTE_FORCE_ATTEMPT',
          severity: 'HIGH',
          subject: record.subject,
          details: { failureCount: recentFailures },
        });
      }
    }

    // Unusual access pattern
    if (record.eventType === 'CONSENT_ACCESS') {
      const isUnusual = await this.detectUnusualAccess(record);
      if (isUnusual) {
        await this.alertService.trigger({
          type: 'UNUSUAL_ACCESS_PATTERN',
          severity: 'MEDIUM',
          subject: record.subject,
          resource: record.resource,
          details: record.details,
        });
      }
    }

    // Data export
    if (record.action === 'EXPORT' || record.action === 'BULK_DOWNLOAD') {
      await this.alertService.trigger({
        type: 'DATA_EXPORT',
        severity: 'INFO',
        subject: record.subject,
        resource: record.resource,
        details: record.details,
      });
    }
  }
}

interface AuditRecord {
  recordId: string;
  timestamp: Date;
  eventType: string;
  severity: 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';

  subject: {
    userId: string;
    sessionId?: string;
    roles: string[];
    ipAddress: string;
    userAgent?: string;
  };

  resource: {
    type: string;
    id?: string;
    patientId?: string;
  };

  action: string;
  outcome: 'SUCCESS' | 'FAILURE' | 'DENIED' | 'ERROR';

  details: Record<string, any>;
  context?: Record<string, any>;

  previousHash?: string;
  recordHash?: string;
}

interface SecurityEvent {
  type: string;
  severity: 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
  userId: string;
  sessionId?: string;
  roles: string[];
  ipAddress: string;
  userAgent?: string;
  resource: { type: string; id?: string; patientId?: string };
  action: string;
  outcome: 'SUCCESS' | 'FAILURE' | 'DENIED' | 'ERROR';
  details: Record<string, any>;
  context?: Record<string, any>;
}

interface IntegrityVerificationResult {
  verified: boolean;
  recordsChecked: number;
  violations: IntegrityViolation[];
  verifiedAt: Date;
}

interface IntegrityViolation {
  recordId: string;
  type: 'CHAIN_BREAK' | 'HASH_MISMATCH' | 'MISSING_RECORD';
  expected: string;
  actual: string;
}
```

---

## 7.5 Threat Detection and Response

```typescript
// Threat detection service
class ThreatDetectionService {
  private anomalyDetector: AnomalyDetector;
  private intrusionDetector: IntrusionDetector;
  private behaviorAnalyzer: BehaviorAnalyzer;
  private responseOrchestrator: ThreatResponseOrchestrator;

  constructor(config: ThreatDetectionConfig) {
    this.anomalyDetector = new AnomalyDetector(config.anomaly);
    this.intrusionDetector = new IntrusionDetector(config.intrusion);
    this.behaviorAnalyzer = new BehaviorAnalyzer(config.behavior);
    this.responseOrchestrator = new ThreatResponseOrchestrator(config.response);
  }

  // Analyze request for threats
  async analyzeRequest(request: SecurityRequest): Promise<ThreatAnalysis> {
    const analyses = await Promise.all([
      this.anomalyDetector.analyze(request),
      this.intrusionDetector.analyze(request),
      this.behaviorAnalyzer.analyze(request),
    ]);

    const combinedThreatLevel = this.combineThreatLevels(analyses);
    const threats = this.extractThreats(analyses);

    // Trigger response if needed
    if (combinedThreatLevel >= ThreatLevel.HIGH) {
      await this.responseOrchestrator.respond(threats, request);
    }

    return {
      threatLevel: combinedThreatLevel,
      threats,
      analyses,
      timestamp: new Date(),
    };
  }

  private combineThreatLevels(analyses: any[]): ThreatLevel {
    const levels = analyses.map(a => a.threatLevel);
    return Math.max(...levels);
  }

  private extractThreats(analyses: any[]): DetectedThreat[] {
    const threats: DetectedThreat[] = [];
    for (const analysis of analyses) {
      threats.push(...(analysis.threats || []));
    }
    return threats;
  }
}

// Anomaly detection
class AnomalyDetector {
  private baselineService: BaselineService;
  private mlModel: AnomalyMLModel;

  async analyze(request: SecurityRequest): Promise<AnomalyAnalysis> {
    // Get user baseline
    const baseline = await this.baselineService.getUserBaseline(request.userId);

    const anomalies: Anomaly[] = [];

    // Check time-based anomaly
    if (this.isUnusualTime(request.timestamp, baseline.typicalHours)) {
      anomalies.push({
        type: 'UNUSUAL_TIME',
        severity: 'MEDIUM',
        details: {
          requestTime: request.timestamp,
          typicalHours: baseline.typicalHours,
        },
      });
    }

    // Check location anomaly
    if (this.isUnusualLocation(request.ipAddress, baseline.typicalLocations)) {
      anomalies.push({
        type: 'UNUSUAL_LOCATION',
        severity: 'HIGH',
        details: {
          requestLocation: request.ipAddress,
          typicalLocations: baseline.typicalLocations,
        },
      });
    }

    // Check volume anomaly
    const recentVolume = await this.getRecentRequestVolume(request.userId);
    if (this.isUnusualVolume(recentVolume, baseline.typicalVolume)) {
      anomalies.push({
        type: 'UNUSUAL_VOLUME',
        severity: 'MEDIUM',
        details: {
          currentVolume: recentVolume,
          typicalVolume: baseline.typicalVolume,
        },
      });
    }

    // Check access pattern anomaly
    if (this.isUnusualAccessPattern(request, baseline.typicalPatterns)) {
      anomalies.push({
        type: 'UNUSUAL_PATTERN',
        severity: 'HIGH',
        details: {
          currentPattern: this.extractPattern(request),
          typicalPatterns: baseline.typicalPatterns,
        },
      });
    }

    // ML-based anomaly detection
    const mlScore = await this.mlModel.predict(request);
    if (mlScore > 0.8) {
      anomalies.push({
        type: 'ML_DETECTED_ANOMALY',
        severity: 'HIGH',
        details: {
          score: mlScore,
          features: this.mlModel.getSignificantFeatures(request),
        },
      });
    }

    return {
      threatLevel: this.calculateThreatLevel(anomalies),
      anomalies,
      timestamp: new Date(),
    };
  }

  private calculateThreatLevel(anomalies: Anomaly[]): ThreatLevel {
    if (anomalies.length === 0) return ThreatLevel.NONE;

    const highSeverityCount = anomalies.filter(a => a.severity === 'HIGH').length;
    const mediumSeverityCount = anomalies.filter(a => a.severity === 'MEDIUM').length;

    if (highSeverityCount >= 2) return ThreatLevel.CRITICAL;
    if (highSeverityCount >= 1) return ThreatLevel.HIGH;
    if (mediumSeverityCount >= 2) return ThreatLevel.MEDIUM;
    return ThreatLevel.LOW;
  }
}

// Threat response orchestrator
class ThreatResponseOrchestrator {
  private alertService: AlertService;
  private accessControl: AccessControlService;
  private incidentManager: IncidentManager;

  async respond(threats: DetectedThreat[], request: SecurityRequest): Promise<void> {
    // Create incident
    const incident = await this.incidentManager.createIncident({
      type: 'SECURITY_THREAT',
      severity: this.getHighestSeverity(threats),
      threats,
      request,
      timestamp: new Date(),
    });

    // Determine response actions
    const actions = this.determineResponseActions(threats, request);

    // Execute responses
    for (const action of actions) {
      await this.executeAction(action, incident);
    }
  }

  private determineResponseActions(
    threats: DetectedThreat[],
    request: SecurityRequest
  ): ResponseAction[] {
    const actions: ResponseAction[] = [];

    for (const threat of threats) {
      switch (threat.type) {
        case 'BRUTE_FORCE':
          actions.push({
            type: 'BLOCK_IP',
            target: request.ipAddress,
            duration: 3600, // 1 hour
          });
          actions.push({
            type: 'LOCK_ACCOUNT',
            target: request.userId,
            duration: 1800, // 30 minutes
          });
          break;

        case 'CREDENTIAL_STUFFING':
          actions.push({
            type: 'REQUIRE_MFA',
            target: request.userId,
          });
          actions.push({
            type: 'NOTIFY_USER',
            target: request.userId,
            message: 'Suspicious login attempt detected',
          });
          break;

        case 'DATA_EXFILTRATION':
          actions.push({
            type: 'REVOKE_SESSION',
            target: request.sessionId,
          });
          actions.push({
            type: 'BLOCK_USER',
            target: request.userId,
          });
          actions.push({
            type: 'ALERT_SECURITY_TEAM',
            severity: 'CRITICAL',
          });
          break;

        case 'PRIVILEGE_ESCALATION':
          actions.push({
            type: 'REVOKE_SESSION',
            target: request.sessionId,
          });
          actions.push({
            type: 'AUDIT_USER_ACTIONS',
            target: request.userId,
            lookback: 86400, // 24 hours
          });
          break;

        case 'INSIDER_THREAT':
          actions.push({
            type: 'ALERT_SECURITY_TEAM',
            severity: 'HIGH',
          });
          actions.push({
            type: 'INCREASE_MONITORING',
            target: request.userId,
          });
          break;
      }
    }

    // Always alert on high severity
    if (this.getHighestSeverity(threats) >= ThreatLevel.HIGH) {
      actions.push({
        type: 'ALERT_SECURITY_TEAM',
        severity: 'HIGH',
      });
    }

    return actions;
  }

  private async executeAction(
    action: ResponseAction,
    incident: Incident
  ): Promise<void> {
    switch (action.type) {
      case 'BLOCK_IP':
        await this.firewall.blockIP(action.target, action.duration);
        break;

      case 'LOCK_ACCOUNT':
        await this.accessControl.lockAccount(action.target, action.duration);
        break;

      case 'REVOKE_SESSION':
        await this.sessionManager.revokeSession(action.target);
        break;

      case 'BLOCK_USER':
        await this.accessControl.blockUser(action.target);
        break;

      case 'REQUIRE_MFA':
        await this.accessControl.requireMFA(action.target);
        break;

      case 'NOTIFY_USER':
        await this.notificationService.notify(action.target, action.message);
        break;

      case 'ALERT_SECURITY_TEAM':
        await this.alertService.alertSecurityTeam(incident, action.severity);
        break;

      case 'AUDIT_USER_ACTIONS':
        await this.auditService.generateUserReport(action.target, action.lookback);
        break;

      case 'INCREASE_MONITORING':
        await this.monitoringService.increaseMonitoring(action.target);
        break;
    }

    // Log response action
    await this.incidentManager.logAction(incident.id, action);
  }
}

enum ThreatLevel {
  NONE = 0,
  LOW = 1,
  MEDIUM = 2,
  HIGH = 3,
  CRITICAL = 4,
}

interface DetectedThreat {
  type: string;
  severity: ThreatLevel;
  confidence: number;
  details: Record<string, any>;
  indicators: string[];
}

interface ResponseAction {
  type: string;
  target?: string;
  duration?: number;
  severity?: string;
  message?: string;
  lookback?: number;
}

interface Incident {
  id: string;
  type: string;
  severity: ThreatLevel;
  threats: DetectedThreat[];
  request: SecurityRequest;
  status: 'OPEN' | 'INVESTIGATING' | 'RESOLVED' | 'CLOSED';
  timestamp: Date;
  actions: ResponseAction[];
}
```

---

## 7.6 Compliance Framework

```typescript
// Compliance monitoring service
class ComplianceMonitoringService {
  private complianceChecks: Map<string, ComplianceCheck> = new Map();
  private reportGenerator: ComplianceReportGenerator;
  private alertService: AlertService;

  constructor(config: ComplianceConfig) {
    this.reportGenerator = new ComplianceReportGenerator(config.reports);
    this.alertService = new AlertService(config.alerts);
    this.registerComplianceChecks();
  }

  private registerComplianceChecks(): void {
    // HIPAA compliance checks
    this.complianceChecks.set('HIPAA_ACCESS_CONTROLS', {
      framework: 'HIPAA',
      requirement: '164.312(a)(1)',
      description: 'Unique user identification',
      check: this.checkUniqueUserIdentification.bind(this),
    });

    this.complianceChecks.set('HIPAA_AUDIT_CONTROLS', {
      framework: 'HIPAA',
      requirement: '164.312(b)',
      description: 'Audit controls',
      check: this.checkAuditControls.bind(this),
    });

    this.complianceChecks.set('HIPAA_INTEGRITY_CONTROLS', {
      framework: 'HIPAA',
      requirement: '164.312(c)(1)',
      description: 'Integrity controls',
      check: this.checkIntegrityControls.bind(this),
    });

    this.complianceChecks.set('HIPAA_TRANSMISSION_SECURITY', {
      framework: 'HIPAA',
      requirement: '164.312(e)(1)',
      description: 'Transmission security',
      check: this.checkTransmissionSecurity.bind(this),
    });

    // GDPR compliance checks
    this.complianceChecks.set('GDPR_CONSENT_VALIDITY', {
      framework: 'GDPR',
      requirement: 'Article 7',
      description: 'Conditions for consent',
      check: this.checkGDPRConsentValidity.bind(this),
    });

    this.complianceChecks.set('GDPR_DATA_MINIMIZATION', {
      framework: 'GDPR',
      requirement: 'Article 5(1)(c)',
      description: 'Data minimization',
      check: this.checkDataMinimization.bind(this),
    });

    this.complianceChecks.set('GDPR_RIGHT_TO_ERASURE', {
      framework: 'GDPR',
      requirement: 'Article 17',
      description: 'Right to erasure',
      check: this.checkRightToErasure.bind(this),
    });

    // SOC 2 compliance checks
    this.complianceChecks.set('SOC2_ENCRYPTION', {
      framework: 'SOC2',
      requirement: 'CC6.1',
      description: 'Encryption of data',
      check: this.checkEncryption.bind(this),
    });

    this.complianceChecks.set('SOC2_ACCESS_REVIEW', {
      framework: 'SOC2',
      requirement: 'CC6.2',
      description: 'Periodic access review',
      check: this.checkAccessReviews.bind(this),
    });

    // PIPA (Korean privacy law) compliance checks
    this.complianceChecks.set('PIPA_CONSENT', {
      framework: 'PIPA',
      requirement: 'Article 15',
      description: 'Collection and use of personal information',
      check: this.checkPIPAConsent.bind(this),
    });
  }

  // Run all compliance checks
  async runComplianceAudit(): Promise<ComplianceAuditResult> {
    const results: ComplianceCheckResult[] = [];

    for (const [checkId, check] of this.complianceChecks) {
      try {
        const result = await check.check();
        results.push({
          checkId,
          framework: check.framework,
          requirement: check.requirement,
          description: check.description,
          ...result,
        });

        // Alert on failures
        if (!result.passed) {
          await this.alertService.trigger({
            type: 'COMPLIANCE_FAILURE',
            severity: 'HIGH',
            details: {
              checkId,
              framework: check.framework,
              requirement: check.requirement,
              findings: result.findings,
            },
          });
        }
      } catch (error) {
        results.push({
          checkId,
          framework: check.framework,
          requirement: check.requirement,
          description: check.description,
          passed: false,
          error: error.message,
        });
      }
    }

    const auditResult: ComplianceAuditResult = {
      auditId: generateAuditId(),
      timestamp: new Date(),
      results,
      overallCompliant: results.every(r => r.passed),
      summary: this.generateSummary(results),
    };

    // Generate report
    await this.reportGenerator.generateReport(auditResult);

    return auditResult;
  }

  // Individual compliance checks
  private async checkUniqueUserIdentification(): Promise<CheckResult> {
    // Verify all users have unique identifiers
    const users = await this.userService.getAllUsers();
    const duplicates = this.findDuplicateIdentifiers(users);

    return {
      passed: duplicates.length === 0,
      findings: duplicates.length > 0
        ? [`Found ${duplicates.length} duplicate user identifiers`]
        : [],
      evidence: {
        totalUsers: users.length,
        uniqueIdentifiers: new Set(users.map(u => u.id)).size,
      },
    };
  }

  private async checkAuditControls(): Promise<CheckResult> {
    // Verify audit logging is enabled and functioning
    const auditConfig = await this.getAuditConfiguration();
    const recentLogs = await this.auditService.getRecentLogs(24 * 60 * 60 * 1000);
    const integrityCheck = await this.auditService.verifyIntegrity();

    const findings: string[] = [];

    if (!auditConfig.enabled) {
      findings.push('Audit logging is not enabled');
    }

    if (recentLogs.length === 0) {
      findings.push('No audit logs in the last 24 hours');
    }

    if (!integrityCheck.verified) {
      findings.push('Audit log integrity verification failed');
    }

    return {
      passed: findings.length === 0,
      findings,
      evidence: {
        auditEnabled: auditConfig.enabled,
        recentLogCount: recentLogs.length,
        integrityVerified: integrityCheck.verified,
      },
    };
  }

  private async checkIntegrityControls(): Promise<CheckResult> {
    // Verify data integrity controls
    const findings: string[] = [];

    // Check encryption at rest
    const encryptionStatus = await this.checkEncryptionAtRest();
    if (!encryptionStatus.enabled) {
      findings.push('Encryption at rest is not enabled');
    }

    // Check integrity verification
    const integrityStatus = await this.checkDataIntegrity();
    if (!integrityStatus.verified) {
      findings.push(`Data integrity issues detected: ${integrityStatus.issues}`);
    }

    return {
      passed: findings.length === 0,
      findings,
      evidence: {
        encryptionAtRest: encryptionStatus,
        integrityStatus,
      },
    };
  }

  private async checkGDPRConsentValidity(): Promise<CheckResult> {
    // Verify GDPR consent requirements
    const consents = await this.consentService.getAllActiveConsents();
    const findings: string[] = [];

    for (const consent of consents) {
      // Check consent is freely given
      if (!consent.metadata.freelyGiven) {
        findings.push(`Consent ${consent.id}: Not marked as freely given`);
      }

      // Check consent is specific
      if (!consent.scope.procedures || consent.scope.procedures.length === 0) {
        findings.push(`Consent ${consent.id}: Not specific enough`);
      }

      // Check consent is informed
      if (!consent.documents.some(d => d.documentType === 'INFORMATION_SHEET')) {
        findings.push(`Consent ${consent.id}: Missing information sheet`);
      }

      // Check consent can be withdrawn
      if (consent.validity.revocation?.revokedAt && !this.wasRevocationProcessed(consent)) {
        findings.push(`Consent ${consent.id}: Revocation not properly processed`);
      }
    }

    return {
      passed: findings.length === 0,
      findings,
      evidence: {
        totalConsents: consents.length,
        issuesFound: findings.length,
      },
    };
  }

  private async checkPIPAConsent(): Promise<CheckResult> {
    // Korean PIPA compliance check
    const consents = await this.consentService.getAllActiveConsents();
    const findings: string[] = [];

    for (const consent of consents) {
      // PIPA requires explicit consent for sensitive information
      if (this.containsSensitiveInfo(consent)) {
        if (!consent.metadata.explicitConsent) {
          findings.push(`Consent ${consent.id}: Sensitive info requires explicit consent under PIPA`);
        }
      }

      // Check retention period compliance
      if (!this.hasValidRetentionPeriod(consent)) {
        findings.push(`Consent ${consent.id}: Retention period not properly defined under PIPA`);
      }
    }

    return {
      passed: findings.length === 0,
      findings,
      evidence: {
        totalConsents: consents.length,
        sensitiveConsents: consents.filter(c => this.containsSensitiveInfo(c)).length,
      },
    };
  }

  // Generate compliance summary
  private generateSummary(results: ComplianceCheckResult[]): ComplianceSummary {
    const byFramework: Record<string, FrameworkSummary> = {};

    for (const result of results) {
      if (!byFramework[result.framework]) {
        byFramework[result.framework] = {
          total: 0,
          passed: 0,
          failed: 0,
          requirements: [],
        };
      }

      byFramework[result.framework].total++;
      if (result.passed) {
        byFramework[result.framework].passed++;
      } else {
        byFramework[result.framework].failed++;
        byFramework[result.framework].requirements.push(result.requirement);
      }
    }

    return {
      totalChecks: results.length,
      passedChecks: results.filter(r => r.passed).length,
      failedChecks: results.filter(r => !r.passed).length,
      byFramework,
    };
  }
}

interface ComplianceCheck {
  framework: string;
  requirement: string;
  description: string;
  check: () => Promise<CheckResult>;
}

interface CheckResult {
  passed: boolean;
  findings?: string[];
  evidence?: Record<string, any>;
  error?: string;
}

interface ComplianceCheckResult extends CheckResult {
  checkId: string;
  framework: string;
  requirement: string;
  description: string;
}

interface ComplianceAuditResult {
  auditId: string;
  timestamp: Date;
  results: ComplianceCheckResult[];
  overallCompliant: boolean;
  summary: ComplianceSummary;
}

interface ComplianceSummary {
  totalChecks: number;
  passedChecks: number;
  failedChecks: number;
  byFramework: Record<string, FrameworkSummary>;
}

interface FrameworkSummary {
  total: number;
  passed: number;
  failed: number;
  requirements: string[];
}
```

---

*Next Chapter: Implementation - Comprehensive deployment guide and best practices*
