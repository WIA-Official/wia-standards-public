# Chapter 7: Security

## Protection Mechanisms and Compliance Frameworks

### Introduction

Securing cryonics assets presents unique challenges that extend far beyond traditional financial security concerns. Assets must be protected not just against contemporary threats but against vulnerabilities that may emerge over decades or centuries. This chapter establishes comprehensive security frameworks covering cryptographic protection, access control, data integrity, regulatory compliance, and long-term preservation strategies that ensure assets remain secure and accessible across extended time horizons.

---

## 7.1 Security Architecture

### Multi-Layer Security Framework

```typescript
// Security architecture overview
interface SecurityArchitecture {
  layers: {
    perimeter: PerimeterSecurity;
    network: NetworkSecurity;
    application: ApplicationSecurity;
    data: DataSecurity;
    cryptographic: CryptographicSecurity;
  };

  principles: SecurityPrinciple[];
  threatModel: ThreatModel;
  controlFramework: ControlFramework;
  complianceRequirements: ComplianceRequirement[];
}

const securityPrinciples: SecurityPrinciple[] = [
  {
    id: 'DEFENSE_IN_DEPTH',
    name: 'Defense in Depth',
    description: 'Multiple layers of security controls',
    implementation: [
      'Perimeter firewalls and WAF',
      'Network segmentation',
      'Application-level controls',
      'Data encryption at rest and in transit',
      'Cryptographic integrity verification',
    ],
  },
  {
    id: 'LEAST_PRIVILEGE',
    name: 'Principle of Least Privilege',
    description: 'Minimum necessary access rights',
    implementation: [
      'Role-based access control (RBAC)',
      'Just-in-time access provisioning',
      'Regular access reviews',
      'Automatic privilege revocation',
    ],
  },
  {
    id: 'ZERO_TRUST',
    name: 'Zero Trust Architecture',
    description: 'Never trust, always verify',
    implementation: [
      'Continuous authentication',
      'Micro-segmentation',
      'Encrypted communications',
      'Device trust verification',
    ],
  },
  {
    id: 'CRYPTO_AGILITY',
    name: 'Cryptographic Agility',
    description: 'Ability to upgrade cryptographic primitives',
    implementation: [
      'Algorithm abstraction layers',
      'Key rotation mechanisms',
      'Post-quantum readiness',
      'Multiple algorithm support',
    ],
  },
  {
    id: 'TEMPORAL_SECURITY',
    name: 'Temporal Security',
    description: 'Security maintained across extended time periods',
    implementation: [
      'Long-term key management',
      'Format-preserving encryption',
      'Technology-agnostic storage',
      'Succession planning for security controls',
    ],
  },
];

// Security service implementation
class CryoAssetSecurityService {
  private cryptoService: CryptographicService;
  private accessControl: AccessControlService;
  private auditService: SecurityAuditService;
  private threatDetection: ThreatDetectionService;

  constructor(config: SecurityConfig) {
    this.cryptoService = new CryptographicService(config.crypto);
    this.accessControl = new AccessControlService(config.access);
    this.auditService = new SecurityAuditService(config.audit);
    this.threatDetection = new ThreatDetectionService(config.threatDetection);
  }

  // Initialize security for new asset
  async secureAsset(asset: CryoAsset): Promise<SecuredAsset> {
    // Generate encryption key for asset
    const encryptionKey = await this.cryptoService.generateAssetKey(asset.id);

    // Encrypt sensitive data
    const encryptedData = await this.encryptAssetData(asset, encryptionKey);

    // Create integrity proof
    const integrityProof = await this.createIntegrityProof(asset, encryptedData);

    // Set up access controls
    const accessPolicy = await this.accessControl.createAssetPolicy(asset);

    // Record in security audit
    await this.auditService.recordSecurityEvent({
      type: 'ASSET_SECURED',
      assetId: asset.id,
      encryptionKeyId: encryptionKey.keyId,
      timestamp: new Date(),
    });

    return {
      assetId: asset.id,
      encryptedData,
      encryptionKeyId: encryptionKey.keyId,
      integrityProof,
      accessPolicy,
      securedAt: new Date(),
    };
  }

  // Verify asset integrity
  async verifyIntegrity(assetId: string): Promise<IntegrityVerification> {
    const asset = await this.getAsset(assetId);
    const storedProof = await this.getIntegrityProof(assetId);

    // Recalculate current proof
    const currentProof = await this.calculateIntegrityProof(asset);

    // Compare proofs
    const proofMatch = await this.cryptoService.verifyProof(
      storedProof,
      currentProof
    );

    // Verify blockchain anchor if present
    let blockchainVerification = null;
    if (storedProof.blockchainAnchor) {
      blockchainVerification = await this.verifyBlockchainAnchor(
        storedProof.blockchainAnchor,
        currentProof.hash
      );
    }

    const result: IntegrityVerification = {
      assetId,
      verified: proofMatch && (!blockchainVerification || blockchainVerification.valid),
      proofMatch,
      blockchainVerification,
      verifiedAt: new Date(),
      details: {
        storedHash: storedProof.hash,
        currentHash: currentProof.hash,
        algorithm: storedProof.algorithm,
      },
    };

    // Record verification
    await this.auditService.recordSecurityEvent({
      type: 'INTEGRITY_VERIFICATION',
      assetId,
      result: result.verified ? 'PASS' : 'FAIL',
      timestamp: new Date(),
    });

    return result;
  }
}
```

---

## 7.2 Cryptographic Security

### Encryption and Key Management

```typescript
// Cryptographic service implementation
interface CryptographicConfig {
  primaryAlgorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  keyDerivation: 'Argon2id' | 'PBKDF2';
  signing: 'Ed25519' | 'ECDSA-P384';
  hashing: 'SHA-256' | 'SHA-384' | 'BLAKE3';
  postQuantum: boolean;  // Enable post-quantum algorithms
}

class CryptographicService {
  private config: CryptographicConfig;
  private keyStore: KeyStore;
  private hsmClient: HSMClient | null;

  constructor(config: CryptographicConfig) {
    this.config = config;
    this.keyStore = new KeyStore(config);

    // Initialize HSM if available
    if (config.hsmEnabled) {
      this.hsmClient = new HSMClient(config.hsmConfig);
    }
  }

  // Generate encryption key for asset
  async generateAssetKey(assetId: string): Promise<EncryptionKey> {
    // Generate key material
    const keyMaterial = await this.generateKeyMaterial(256);

    // Wrap with master key
    const wrappedKey = await this.wrapKey(keyMaterial);

    // Create key record
    const key: EncryptionKey = {
      keyId: generateKeyId(),
      assetId,
      algorithm: this.config.primaryAlgorithm,
      keyMaterial: wrappedKey,
      version: 1,
      status: 'ACTIVE',
      createdAt: new Date(),
      expiresAt: null,  // Asset keys don't expire
      rotationPolicy: {
        automatic: false,
        intervalDays: null,
        onCompromise: true,
      },
    };

    // Store key
    await this.keyStore.storeKey(key);

    // For HSM-backed keys
    if (this.hsmClient) {
      await this.hsmClient.importKey(key.keyId, keyMaterial);
    }

    return key;
  }

  // Encrypt data
  async encrypt(
    data: Buffer,
    keyId: string
  ): Promise<EncryptedData> {
    const key = await this.keyStore.getKey(keyId);

    // Get unwrapped key material
    const keyMaterial = await this.unwrapKey(key.keyMaterial);

    // Generate nonce
    const nonce = crypto.randomBytes(12);

    // Encrypt based on algorithm
    let ciphertext: Buffer;
    let authTag: Buffer;

    switch (this.config.primaryAlgorithm) {
      case 'AES-256-GCM':
        const cipher = crypto.createCipheriv(
          'aes-256-gcm',
          keyMaterial,
          nonce
        );
        ciphertext = Buffer.concat([
          cipher.update(data),
          cipher.final(),
        ]);
        authTag = cipher.getAuthTag();
        break;

      case 'ChaCha20-Poly1305':
        const chaChaResult = await this.chacha20poly1305Encrypt(
          keyMaterial,
          nonce,
          data
        );
        ciphertext = chaChaResult.ciphertext;
        authTag = chaChaResult.tag;
        break;

      default:
        throw new Error(`Unsupported algorithm: ${this.config.primaryAlgorithm}`);
    }

    // Clear sensitive data
    keyMaterial.fill(0);

    return {
      ciphertext: ciphertext.toString('base64'),
      nonce: nonce.toString('base64'),
      authTag: authTag.toString('base64'),
      algorithm: this.config.primaryAlgorithm,
      keyId,
      keyVersion: key.version,
      encryptedAt: new Date(),
    };
  }

  // Decrypt data
  async decrypt(
    encrypted: EncryptedData
  ): Promise<Buffer> {
    const key = await this.keyStore.getKey(encrypted.keyId);

    // Check key version matches
    if (encrypted.keyVersion !== key.version) {
      // Get historical key version
      const historicalKey = await this.keyStore.getKeyVersion(
        encrypted.keyId,
        encrypted.keyVersion
      );
      return this.decryptWithKey(encrypted, historicalKey);
    }

    return this.decryptWithKey(encrypted, key);
  }

  private async decryptWithKey(
    encrypted: EncryptedData,
    key: EncryptionKey
  ): Promise<Buffer> {
    const keyMaterial = await this.unwrapKey(key.keyMaterial);
    const nonce = Buffer.from(encrypted.nonce, 'base64');
    const ciphertext = Buffer.from(encrypted.ciphertext, 'base64');
    const authTag = Buffer.from(encrypted.authTag, 'base64');

    let plaintext: Buffer;

    switch (encrypted.algorithm) {
      case 'AES-256-GCM':
        const decipher = crypto.createDecipheriv(
          'aes-256-gcm',
          keyMaterial,
          nonce
        );
        decipher.setAuthTag(authTag);
        plaintext = Buffer.concat([
          decipher.update(ciphertext),
          decipher.final(),
        ]);
        break;

      case 'ChaCha20-Poly1305':
        plaintext = await this.chacha20poly1305Decrypt(
          keyMaterial,
          nonce,
          ciphertext,
          authTag
        );
        break;

      default:
        throw new Error(`Unsupported algorithm: ${encrypted.algorithm}`);
    }

    keyMaterial.fill(0);
    return plaintext;
  }

  // Key rotation
  async rotateKey(keyId: string): Promise<EncryptionKey> {
    const currentKey = await this.keyStore.getKey(keyId);

    // Generate new key material
    const newKeyMaterial = await this.generateKeyMaterial(256);
    const wrappedNewKey = await this.wrapKey(newKeyMaterial);

    // Create new key version
    const newKey: EncryptionKey = {
      ...currentKey,
      keyMaterial: wrappedNewKey,
      version: currentKey.version + 1,
      previousVersions: [
        ...(currentKey.previousVersions || []),
        {
          version: currentKey.version,
          keyMaterial: currentKey.keyMaterial,
          retiredAt: new Date(),
        },
      ],
      rotatedAt: new Date(),
    };

    // Store updated key
    await this.keyStore.updateKey(newKey);

    // Update HSM if applicable
    if (this.hsmClient) {
      await this.hsmClient.rotateKey(keyId, newKeyMaterial);
    }

    return newKey;
  }

  // Create digital signature
  async sign(
    data: Buffer,
    signingKeyId: string
  ): Promise<DigitalSignature> {
    const signingKey = await this.keyStore.getSigningKey(signingKeyId);

    let signature: Buffer;

    switch (this.config.signing) {
      case 'Ed25519':
        signature = nacl.sign.detached(
          data,
          Buffer.from(signingKey.privateKey, 'base64')
        );
        break;

      case 'ECDSA-P384':
        const sign = crypto.createSign('SHA384');
        sign.update(data);
        signature = sign.sign({
          key: signingKey.privateKey,
          dsaEncoding: 'ieee-p1363',
        });
        break;

      default:
        throw new Error(`Unsupported signing algorithm: ${this.config.signing}`);
    }

    return {
      signature: Buffer.from(signature).toString('base64'),
      algorithm: this.config.signing,
      keyId: signingKeyId,
      signedAt: new Date(),
    };
  }

  // Verify digital signature
  async verify(
    data: Buffer,
    signature: DigitalSignature
  ): Promise<boolean> {
    const signingKey = await this.keyStore.getSigningKey(signature.keyId);
    const signatureBytes = Buffer.from(signature.signature, 'base64');

    switch (signature.algorithm) {
      case 'Ed25519':
        return nacl.sign.detached.verify(
          data,
          signatureBytes,
          Buffer.from(signingKey.publicKey, 'base64')
        );

      case 'ECDSA-P384':
        const verify = crypto.createVerify('SHA384');
        verify.update(data);
        return verify.verify(
          {
            key: signingKey.publicKey,
            dsaEncoding: 'ieee-p1363',
          },
          signatureBytes
        );

      default:
        throw new Error(`Unsupported signing algorithm: ${signature.algorithm}`);
    }
  }

  // Post-quantum key encapsulation (future-proofing)
  async generatePostQuantumKeyPair(): Promise<PostQuantumKeyPair> {
    // Using CRYSTALS-Kyber for key encapsulation
    const kyber = await import('@pqcrypto/kyber');

    const keypair = kyber.keypair();

    return {
      algorithm: 'CRYSTALS-Kyber-1024',
      publicKey: Buffer.from(keypair.publicKey).toString('base64'),
      privateKey: Buffer.from(keypair.privateKey).toString('base64'),
      createdAt: new Date(),
    };
  }
}

// Key store implementation
class KeyStore {
  private db: Database;
  private masterKey: Buffer;

  constructor(config: KeyStoreConfig) {
    this.db = new Database(config.database);
    this.initializeMasterKey(config);
  }

  private async initializeMasterKey(config: KeyStoreConfig): Promise<void> {
    if (config.masterKeySource === 'HSM') {
      // Get master key from HSM
      this.masterKey = await this.getHSMMasterKey(config.hsmConfig);
    } else if (config.masterKeySource === 'KMS') {
      // Get master key from cloud KMS
      this.masterKey = await this.getKMSMasterKey(config.kmsConfig);
    } else {
      // Derive from environment (development only)
      this.masterKey = crypto.scryptSync(
        process.env.MASTER_KEY_PASSWORD!,
        process.env.MASTER_KEY_SALT!,
        32
      );
    }
  }

  async storeKey(key: EncryptionKey): Promise<void> {
    // Encrypt key material with master key
    const encryptedKeyMaterial = await this.encryptWithMasterKey(
      Buffer.from(key.keyMaterial, 'base64')
    );

    await this.db.insert('encryption_keys', {
      key_id: key.keyId,
      asset_id: key.assetId,
      algorithm: key.algorithm,
      key_material: encryptedKeyMaterial,
      version: key.version,
      status: key.status,
      created_at: key.createdAt,
      expires_at: key.expiresAt,
      rotation_policy: JSON.stringify(key.rotationPolicy),
    });
  }

  async getKey(keyId: string): Promise<EncryptionKey> {
    const row = await this.db.get('encryption_keys', { key_id: keyId });

    if (!row) {
      throw new Error(`Key not found: ${keyId}`);
    }

    // Decrypt key material
    const keyMaterial = await this.decryptWithMasterKey(row.key_material);

    return {
      keyId: row.key_id,
      assetId: row.asset_id,
      algorithm: row.algorithm,
      keyMaterial: keyMaterial.toString('base64'),
      version: row.version,
      status: row.status,
      createdAt: row.created_at,
      expiresAt: row.expires_at,
      rotationPolicy: JSON.parse(row.rotation_policy),
    };
  }
}
```

---

## 7.3 Access Control

### Identity and Authorization Management

```typescript
// Access control service implementation
interface AccessControlConfig {
  model: 'RBAC' | 'ABAC' | 'PBAC';  // Role/Attribute/Policy-based
  mfaRequired: boolean;
  sessionTimeout: number;  // Minutes
  maxConcurrentSessions: number;
}

class AccessControlService {
  private config: AccessControlConfig;
  private identityProvider: IdentityProvider;
  private policyEngine: PolicyEngine;

  constructor(config: AccessControlConfig) {
    this.config = config;
    this.identityProvider = new IdentityProvider(config.identity);
    this.policyEngine = new PolicyEngine(config.policies);
  }

  // Authenticate user
  async authenticate(
    credentials: AuthenticationCredentials
  ): Promise<AuthenticationResult> {
    // Validate primary credentials
    const primaryAuth = await this.identityProvider.validateCredentials(credentials);

    if (!primaryAuth.valid) {
      await this.recordFailedAuthentication(credentials.username);
      return { success: false, reason: primaryAuth.reason };
    }

    // Check MFA requirement
    if (this.config.mfaRequired) {
      if (!credentials.mfaToken) {
        return {
          success: false,
          reason: 'MFA_REQUIRED',
          mfaChallenge: await this.generateMFAChallenge(primaryAuth.userId),
        };
      }

      const mfaValid = await this.verifyMFA(primaryAuth.userId, credentials.mfaToken);
      if (!mfaValid) {
        return { success: false, reason: 'MFA_INVALID' };
      }
    }

    // Create session
    const session = await this.createSession(primaryAuth.userId);

    // Get user roles and permissions
    const authorization = await this.getAuthorization(primaryAuth.userId);

    return {
      success: true,
      userId: primaryAuth.userId,
      session,
      authorization,
    };
  }

  // Check authorization for action
  async authorize(
    context: AuthorizationContext
  ): Promise<AuthorizationResult> {
    const { userId, action, resource, resourceId } = context;

    // Get user's roles and attributes
    const user = await this.identityProvider.getUser(userId);
    const roles = await this.getUserRoles(userId);
    const attributes = await this.getUserAttributes(userId);

    // Get resource attributes
    const resourceAttrs = await this.getResourceAttributes(resource, resourceId);

    // Evaluate policies
    const policyResult = await this.policyEngine.evaluate({
      subject: {
        userId,
        roles,
        attributes,
      },
      action,
      resource: {
        type: resource,
        id: resourceId,
        attributes: resourceAttrs,
      },
      environment: {
        timestamp: new Date(),
        ipAddress: context.ipAddress,
        requestId: context.requestId,
      },
    });

    // Record authorization decision
    await this.recordAuthorizationDecision({
      userId,
      action,
      resource,
      resourceId,
      decision: policyResult.decision,
      reason: policyResult.reason,
      timestamp: new Date(),
    });

    return {
      authorized: policyResult.decision === 'PERMIT',
      reason: policyResult.reason,
      obligations: policyResult.obligations,
    };
  }

  // Create access policy for asset
  async createAssetPolicy(asset: CryoAsset): Promise<AccessPolicy> {
    const policy: AccessPolicy = {
      id: generatePolicyId(),
      resourceType: 'ASSET',
      resourceId: asset.id,
      rules: [
        // Owner has full access
        {
          id: 'owner-full-access',
          effect: 'PERMIT',
          principals: [{ type: 'OWNER', id: asset.patientId }],
          actions: ['*'],
          conditions: [],
        },
        // Trustees can view and manage
        {
          id: 'trustee-manage',
          effect: 'PERMIT',
          principals: [{ type: 'ROLE', id: 'TRUSTEE' }],
          actions: ['VIEW', 'MANAGE', 'TRANSFER'],
          conditions: [
            {
              type: 'RELATIONSHIP',
              operator: 'HAS_RELATIONSHIP',
              value: { relationship: 'TRUSTEE_OF', patientId: asset.patientId },
            },
          ],
        },
        // Organization admins can view
        {
          id: 'org-admin-view',
          effect: 'PERMIT',
          principals: [{ type: 'ROLE', id: 'ORG_ADMIN' }],
          actions: ['VIEW'],
          conditions: [
            {
              type: 'ORGANIZATION',
              operator: 'SAME_ORG',
              value: asset.organizationId,
            },
          ],
        },
        // Default deny
        {
          id: 'default-deny',
          effect: 'DENY',
          principals: [{ type: 'ANY' }],
          actions: ['*'],
          conditions: [],
        },
      ],
      createdAt: new Date(),
      createdBy: 'SYSTEM',
    };

    await this.policyEngine.registerPolicy(policy);
    return policy;
  }

  // Multi-factor authentication
  async setupMFA(
    userId: string,
    method: MFAMethod
  ): Promise<MFASetupResult> {
    switch (method) {
      case 'TOTP':
        const secret = this.generateTOTPSecret();
        const qrCode = await this.generateTOTPQRCode(userId, secret);

        await this.storeMFAConfig(userId, {
          method: 'TOTP',
          secret,
          verified: false,
        });

        return {
          method: 'TOTP',
          secret,
          qrCode,
          backupCodes: await this.generateBackupCodes(userId),
        };

      case 'WEBAUTHN':
        const challenge = await this.generateWebAuthnChallenge(userId);
        return {
          method: 'WEBAUTHN',
          challenge,
          rpId: this.config.rpId,
        };

      case 'SMS':
        await this.storeMFAConfig(userId, {
          method: 'SMS',
          phoneNumber: null,  // To be set
          verified: false,
        });
        return { method: 'SMS' };

      default:
        throw new Error(`Unsupported MFA method: ${method}`);
    }
  }

  // Session management
  async createSession(userId: string): Promise<Session> {
    // Check concurrent session limit
    const activeSessions = await this.getActiveSessions(userId);
    if (activeSessions.length >= this.config.maxConcurrentSessions) {
      // Revoke oldest session
      await this.revokeSession(activeSessions[0].id);
    }

    const session: Session = {
      id: generateSessionId(),
      userId,
      createdAt: new Date(),
      expiresAt: new Date(Date.now() + this.config.sessionTimeout * 60 * 1000),
      lastActivity: new Date(),
      ipAddress: null,  // Set by caller
      userAgent: null,  // Set by caller
      status: 'ACTIVE',
    };

    await this.storeSession(session);
    return session;
  }

  // Emergency access
  async grantEmergencyAccess(
    request: EmergencyAccessRequest
  ): Promise<EmergencyAccessGrant> {
    // Validate emergency conditions
    const validation = await this.validateEmergencyConditions(request);
    if (!validation.valid) {
      throw new Error(`Emergency access denied: ${validation.reason}`);
    }

    // Create time-limited emergency grant
    const grant: EmergencyAccessGrant = {
      id: generateGrantId(),
      requesterId: request.requesterId,
      resourceType: request.resourceType,
      resourceId: request.resourceId,
      reason: request.reason,
      grantedActions: request.requestedActions,
      grantedAt: new Date(),
      expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000),  // 24 hours
      conditions: validation.conditions,
      approvedBy: validation.approvers,
    };

    await this.storeEmergencyGrant(grant);

    // Notify relevant parties
    await this.notifyEmergencyAccess(grant);

    return grant;
  }
}

// Policy engine implementation
class PolicyEngine {
  private policies: Map<string, AccessPolicy>;

  async evaluate(request: PolicyRequest): Promise<PolicyDecision> {
    const applicablePolicies = await this.findApplicablePolicies(request);

    // Evaluate each policy
    const decisions: RuleDecision[] = [];

    for (const policy of applicablePolicies) {
      for (const rule of policy.rules) {
        const ruleDecision = await this.evaluateRule(rule, request);
        decisions.push(ruleDecision);

        // Short-circuit on explicit DENY
        if (ruleDecision.effect === 'DENY' && ruleDecision.matched) {
          return {
            decision: 'DENY',
            reason: ruleDecision.reason,
            matchedRule: rule.id,
          };
        }
      }
    }

    // Check for any PERMIT
    const permitDecision = decisions.find(d => d.effect === 'PERMIT' && d.matched);
    if (permitDecision) {
      return {
        decision: 'PERMIT',
        reason: permitDecision.reason,
        matchedRule: permitDecision.ruleId,
        obligations: permitDecision.obligations,
      };
    }

    // Default deny
    return {
      decision: 'DENY',
      reason: 'No matching permit rule',
    };
  }

  private async evaluateRule(
    rule: PolicyRule,
    request: PolicyRequest
  ): Promise<RuleDecision> {
    // Check principal match
    const principalMatch = this.matchPrincipal(rule.principals, request.subject);
    if (!principalMatch) {
      return { ruleId: rule.id, effect: rule.effect, matched: false };
    }

    // Check action match
    const actionMatch = this.matchAction(rule.actions, request.action);
    if (!actionMatch) {
      return { ruleId: rule.id, effect: rule.effect, matched: false };
    }

    // Check conditions
    for (const condition of rule.conditions) {
      const conditionMet = await this.evaluateCondition(condition, request);
      if (!conditionMet) {
        return {
          ruleId: rule.id,
          effect: rule.effect,
          matched: false,
          reason: `Condition not met: ${condition.type}`,
        };
      }
    }

    return {
      ruleId: rule.id,
      effect: rule.effect,
      matched: true,
      obligations: rule.obligations,
    };
  }
}
```

---

## 7.4 Data Protection

### Comprehensive Data Security

```typescript
// Data protection service
class DataProtectionService {
  private classifier: DataClassifier;
  private encryptor: DataEncryptor;
  private tokenizer: DataTokenizer;
  private dlpEngine: DLPEngine;

  constructor(config: DataProtectionConfig) {
    this.classifier = new DataClassifier(config.classification);
    this.encryptor = new DataEncryptor(config.encryption);
    this.tokenizer = new DataTokenizer(config.tokenization);
    this.dlpEngine = new DLPEngine(config.dlp);
  }

  // Classify and protect data
  async protectData(
    data: any,
    context: DataContext
  ): Promise<ProtectedData> {
    // Classify data
    const classification = await this.classifier.classify(data, context);

    // Apply appropriate protection based on classification
    const protectedData = await this.applyProtection(data, classification);

    return {
      data: protectedData.data,
      classification,
      protectionApplied: protectedData.methods,
      metadata: {
        classifiedAt: new Date(),
        protectedAt: new Date(),
        retentionPolicy: classification.retentionPolicy,
      },
    };
  }

  private async applyProtection(
    data: any,
    classification: DataClassification
  ): Promise<{ data: any; methods: string[] }> {
    const methods: string[] = [];
    let protectedData = data;

    switch (classification.level) {
      case 'HIGHLY_CONFIDENTIAL':
        // Full encryption + tokenization of PII
        protectedData = await this.encryptor.encrypt(protectedData);
        methods.push('ENCRYPTION');

        protectedData = await this.tokenizer.tokenizePII(protectedData);
        methods.push('TOKENIZATION');
        break;

      case 'CONFIDENTIAL':
        // Encryption of sensitive fields
        protectedData = await this.encryptor.encryptFields(
          protectedData,
          classification.sensitiveFields
        );
        methods.push('FIELD_ENCRYPTION');
        break;

      case 'INTERNAL':
        // Tokenize PII only
        protectedData = await this.tokenizer.tokenizePII(protectedData);
        methods.push('TOKENIZATION');
        break;

      case 'PUBLIC':
        // No protection needed
        break;
    }

    return { data: protectedData, methods };
  }

  // Data Loss Prevention
  async scanForLeakage(
    data: any,
    destination: string
  ): Promise<DLPScanResult> {
    const findings: DLPFinding[] = [];

    // Scan for PII
    const piiFindings = await this.dlpEngine.scanForPII(data);
    findings.push(...piiFindings);

    // Scan for financial data
    const financialFindings = await this.dlpEngine.scanForFinancialData(data);
    findings.push(...financialFindings);

    // Scan for credentials
    const credentialFindings = await this.dlpEngine.scanForCredentials(data);
    findings.push(...credentialFindings);

    // Evaluate against DLP policies
    const policyViolations = await this.dlpEngine.evaluatePolicies(
      findings,
      destination
    );

    return {
      scannedAt: new Date(),
      destination,
      findings,
      policyViolations,
      blocked: policyViolations.some(v => v.action === 'BLOCK'),
      action: this.determineAction(policyViolations),
    };
  }

  // Secure data destruction
  async secureDestroy(
    dataId: string,
    reason: string
  ): Promise<DestructionCertificate> {
    // Get data location
    const dataLocation = await this.getDataLocation(dataId);

    // Verify destruction authorization
    const authorized = await this.verifyDestructionAuthorization(dataId);
    if (!authorized) {
      throw new Error('Destruction not authorized');
    }

    // Perform secure deletion
    const deletionResult = await this.performSecureDeletion(dataLocation);

    // Delete encryption keys
    const keyDeletion = await this.deleteAssociatedKeys(dataId);

    // Remove from backups
    const backupRemoval = await this.removeFromBackups(dataId);

    // Create destruction certificate
    const certificate: DestructionCertificate = {
      certificateId: generateCertificateId(),
      dataId,
      reason,
      destroyedAt: new Date(),
      deletionMethod: deletionResult.method,
      keysDestroyed: keyDeletion.keysDeleted,
      backupsRemoved: backupRemoval.count,
      attestation: await this.generateAttestation(deletionResult),
      verifiedBy: 'SYSTEM',
    };

    // Record destruction
    await this.recordDestruction(certificate);

    return certificate;
  }

  // Data retention management
  async enforceRetention(): Promise<RetentionEnforcementResult> {
    const result: RetentionEnforcementResult = {
      scannedRecords: 0,
      expiredRecords: 0,
      archivedRecords: 0,
      deletedRecords: 0,
      errors: [],
    };

    // Get all records with retention policies
    const records = await this.getRecordsWithRetention();

    for (const record of records) {
      result.scannedRecords++;

      try {
        const retentionStatus = this.evaluateRetention(record);

        switch (retentionStatus.action) {
          case 'ARCHIVE':
            await this.archiveRecord(record);
            result.archivedRecords++;
            break;

          case 'DELETE':
            await this.secureDestroy(record.id, 'RETENTION_EXPIRED');
            result.deletedRecords++;
            break;

          case 'RETAIN':
            // No action needed
            break;
        }

        if (retentionStatus.expired) {
          result.expiredRecords++;
        }
      } catch (error) {
        result.errors.push({
          recordId: record.id,
          error: error.message,
        });
      }
    }

    return result;
  }
}

// Data classification service
class DataClassifier {
  private rules: ClassificationRule[];

  async classify(data: any, context: DataContext): Promise<DataClassification> {
    const detectedPatterns: DetectedPattern[] = [];

    // Detect PII patterns
    const piiPatterns = await this.detectPII(data);
    detectedPatterns.push(...piiPatterns);

    // Detect financial patterns
    const financialPatterns = await this.detectFinancialData(data);
    detectedPatterns.push(...financialPatterns);

    // Detect health information
    const healthPatterns = await this.detectHealthInfo(data);
    detectedPatterns.push(...healthPatterns);

    // Determine classification level
    const level = this.determineClassificationLevel(detectedPatterns, context);

    // Identify sensitive fields
    const sensitiveFields = this.identifySensitiveFields(detectedPatterns);

    return {
      level,
      categories: this.categorizePatterns(detectedPatterns),
      sensitiveFields,
      retentionPolicy: this.getRetentionPolicy(level, context),
      handlingInstructions: this.getHandlingInstructions(level),
    };
  }

  private async detectPII(data: any): Promise<DetectedPattern[]> {
    const patterns: DetectedPattern[] = [];

    const piiPatterns = {
      SSN: /\b\d{3}-\d{2}-\d{4}\b/,
      EMAIL: /\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b/,
      PHONE: /\b\d{3}[-.]?\d{3}[-.]?\d{4}\b/,
      DOB: /\b\d{4}-\d{2}-\d{2}\b/,  // ISO date format
      PASSPORT: /\b[A-Z]{1,2}\d{6,9}\b/,
    };

    const dataStr = JSON.stringify(data);

    for (const [type, pattern] of Object.entries(piiPatterns)) {
      const matches = dataStr.match(new RegExp(pattern, 'g'));
      if (matches) {
        patterns.push({
          type: 'PII',
          subtype: type,
          count: matches.length,
          severity: this.getPIISeverity(type),
        });
      }
    }

    return patterns;
  }
}
```

---

## 7.5 Threat Detection and Response

### Security Monitoring

```typescript
// Threat detection service
class ThreatDetectionService {
  private rules: DetectionRule[];
  private mlModels: ThreatMLModels;
  private alertService: AlertService;

  constructor(config: ThreatDetectionConfig) {
    this.rules = this.loadDetectionRules(config.rulesPath);
    this.mlModels = new ThreatMLModels(config.mlConfig);
    this.alertService = new AlertService(config.alertConfig);
  }

  // Analyze event for threats
  async analyzeEvent(event: SecurityEvent): Promise<ThreatAnalysis> {
    const findings: ThreatFinding[] = [];

    // Rule-based detection
    const ruleFindings = await this.applyDetectionRules(event);
    findings.push(...ruleFindings);

    // ML-based anomaly detection
    const mlFindings = await this.detectAnomalies(event);
    findings.push(...mlFindings);

    // Behavioral analysis
    const behavioralFindings = await this.analyzeBehavior(event);
    findings.push(...behavioralFindings);

    // Correlate findings
    const correlatedFindings = await this.correlateFindings(findings);

    // Determine overall threat level
    const threatLevel = this.calculateThreatLevel(correlatedFindings);

    // Generate response actions
    const responseActions = await this.generateResponseActions(
      correlatedFindings,
      threatLevel
    );

    return {
      eventId: event.id,
      analyzedAt: new Date(),
      findings: correlatedFindings,
      threatLevel,
      responseActions,
      requiresInvestigation: threatLevel >= ThreatLevel.MEDIUM,
    };
  }

  // Real-time monitoring
  async monitorActivity(): Promise<void> {
    const eventStream = await this.getSecurityEventStream();

    for await (const event of eventStream) {
      try {
        const analysis = await this.analyzeEvent(event);

        if (analysis.threatLevel >= ThreatLevel.HIGH) {
          // Immediate alert
          await this.alertService.sendCriticalAlert({
            type: 'THREAT_DETECTED',
            analysis,
            timestamp: new Date(),
          });

          // Execute automated response
          await this.executeAutomatedResponse(analysis.responseActions);
        } else if (analysis.requiresInvestigation) {
          // Queue for investigation
          await this.queueForInvestigation(analysis);
        }

        // Store analysis
        await this.storeAnalysis(analysis);
      } catch (error) {
        console.error('Error analyzing event:', error);
      }
    }
  }

  private async detectAnomalies(event: SecurityEvent): Promise<ThreatFinding[]> {
    const findings: ThreatFinding[] = [];

    // User behavior anomaly detection
    if (event.userId) {
      const userBaseline = await this.getUserBaseline(event.userId);
      const anomalyScore = await this.mlModels.userBehavior.predict({
        event,
        baseline: userBaseline,
      });

      if (anomalyScore > 0.8) {
        findings.push({
          type: 'ANOMALY',
          category: 'USER_BEHAVIOR',
          description: 'Unusual user behavior detected',
          confidence: anomalyScore,
          indicators: this.extractAnomalyIndicators(event, userBaseline),
        });
      }
    }

    // Access pattern anomaly
    const accessAnomaly = await this.mlModels.accessPattern.predict({
      resourceId: event.resourceId,
      accessorId: event.userId,
      timestamp: event.timestamp,
      action: event.action,
    });

    if (accessAnomaly.isAnomaly) {
      findings.push({
        type: 'ANOMALY',
        category: 'ACCESS_PATTERN',
        description: accessAnomaly.description,
        confidence: accessAnomaly.confidence,
      });
    }

    return findings;
  }

  // Incident response
  async handleIncident(incident: SecurityIncident): Promise<IncidentResponse> {
    // Create incident record
    const incidentRecord = await this.createIncidentRecord(incident);

    // Determine severity
    const severity = this.assessSeverity(incident);

    // Execute containment
    const containmentActions = await this.executeContainment(incident, severity);

    // Collect evidence
    const evidence = await this.collectEvidence(incident);

    // Notify stakeholders
    await this.notifyStakeholders(incident, severity);

    // Generate response plan
    const responsePlan = await this.generateResponsePlan(incident, severity);

    return {
      incidentId: incidentRecord.id,
      severity,
      containmentActions,
      evidence,
      responsePlan,
      status: 'CONTAINED',
      nextSteps: responsePlan.immediateActions,
    };
  }

  private async executeContainment(
    incident: SecurityIncident,
    severity: Severity
  ): Promise<ContainmentAction[]> {
    const actions: ContainmentAction[] = [];

    // Based on incident type
    switch (incident.type) {
      case 'UNAUTHORIZED_ACCESS':
        // Revoke session
        await this.revokeUserSessions(incident.userId);
        actions.push({
          type: 'SESSION_REVOKED',
          target: incident.userId,
          timestamp: new Date(),
        });

        // Lock account if severe
        if (severity >= Severity.HIGH) {
          await this.lockAccount(incident.userId);
          actions.push({
            type: 'ACCOUNT_LOCKED',
            target: incident.userId,
            timestamp: new Date(),
          });
        }
        break;

      case 'DATA_EXFILTRATION':
        // Block network access
        await this.blockNetworkAccess(incident.sourceIp);
        actions.push({
          type: 'NETWORK_BLOCKED',
          target: incident.sourceIp,
          timestamp: new Date(),
        });
        break;

      case 'MALWARE_DETECTED':
        // Isolate affected systems
        await this.isolateSystem(incident.affectedSystem);
        actions.push({
          type: 'SYSTEM_ISOLATED',
          target: incident.affectedSystem,
          timestamp: new Date(),
        });
        break;
    }

    return actions;
  }
}
```

---

## 7.6 Compliance Frameworks

### Regulatory Compliance

```typescript
// Compliance management service
interface ComplianceFramework {
  name: string;
  version: string;
  controls: ComplianceControl[];
  assessmentSchedule: string;
}

class ComplianceService {
  private frameworks: Map<string, ComplianceFramework>;
  private assessmentEngine: AssessmentEngine;
  private reportingService: ComplianceReportingService;

  constructor(config: ComplianceConfig) {
    this.frameworks = new Map();
    this.loadFrameworks(config.frameworks);
    this.assessmentEngine = new AssessmentEngine(config.assessment);
    this.reportingService = new ComplianceReportingService(config.reporting);
  }

  private loadFrameworks(frameworks: string[]): void {
    for (const framework of frameworks) {
      switch (framework) {
        case 'SOC2':
          this.frameworks.set('SOC2', this.getSOC2Framework());
          break;
        case 'GDPR':
          this.frameworks.set('GDPR', this.getGDPRFramework());
          break;
        case 'CCPA':
          this.frameworks.set('CCPA', this.getCCPAFramework());
          break;
        case 'HIPAA':
          this.frameworks.set('HIPAA', this.getHIPAAFramework());
          break;
        case 'FINRA':
          this.frameworks.set('FINRA', this.getFINRAFramework());
          break;
      }
    }
  }

  // Assess compliance against framework
  async assessCompliance(
    frameworkId: string
  ): Promise<ComplianceAssessment> {
    const framework = this.frameworks.get(frameworkId);
    if (!framework) {
      throw new Error(`Framework not found: ${frameworkId}`);
    }

    const controlResults: ControlAssessmentResult[] = [];

    for (const control of framework.controls) {
      const result = await this.assessmentEngine.assessControl(control);
      controlResults.push(result);
    }

    // Calculate overall compliance score
    const score = this.calculateComplianceScore(controlResults);

    // Identify gaps
    const gaps = controlResults.filter(r => r.status !== 'COMPLIANT');

    // Generate remediation plan
    const remediationPlan = await this.generateRemediationPlan(gaps);

    return {
      frameworkId,
      frameworkVersion: framework.version,
      assessmentDate: new Date(),
      overallScore: score,
      status: score >= 0.9 ? 'COMPLIANT' : score >= 0.7 ? 'PARTIAL' : 'NON_COMPLIANT',
      controlResults,
      gaps,
      remediationPlan,
    };
  }

  private getSOC2Framework(): ComplianceFramework {
    return {
      name: 'SOC 2 Type II',
      version: '2017',
      controls: [
        // Security
        {
          id: 'CC1.1',
          category: 'SECURITY',
          title: 'Control Environment',
          description: 'The entity demonstrates a commitment to integrity and ethical values',
          testProcedures: ['Review code of conduct', 'Interview employees'],
          evidenceRequired: ['Code of conduct document', 'Training records'],
        },
        {
          id: 'CC6.1',
          category: 'SECURITY',
          title: 'Logical and Physical Access Controls',
          description: 'The entity implements logical access security software',
          testProcedures: ['Review access control policies', 'Test access controls'],
          evidenceRequired: ['Access control policies', 'Access logs', 'User provisioning records'],
        },
        // Availability
        {
          id: 'A1.1',
          category: 'AVAILABILITY',
          title: 'System Availability',
          description: 'The entity maintains system availability commitments',
          testProcedures: ['Review SLAs', 'Analyze uptime metrics'],
          evidenceRequired: ['SLA documents', 'Uptime reports', 'Incident records'],
        },
        // Confidentiality
        {
          id: 'C1.1',
          category: 'CONFIDENTIALITY',
          title: 'Confidential Information Protection',
          description: 'The entity identifies and maintains confidential information',
          testProcedures: ['Review data classification', 'Test encryption'],
          evidenceRequired: ['Data classification policy', 'Encryption audit'],
        },
        // Processing Integrity
        {
          id: 'PI1.1',
          category: 'PROCESSING_INTEGRITY',
          title: 'Processing Accuracy',
          description: 'The entity uses quality assurance procedures',
          testProcedures: ['Review QA procedures', 'Test data integrity'],
          evidenceRequired: ['QA documentation', 'Integrity check logs'],
        },
        // Privacy
        {
          id: 'P1.1',
          category: 'PRIVACY',
          title: 'Privacy Notice',
          description: 'The entity provides notice about privacy practices',
          testProcedures: ['Review privacy policy', 'Verify disclosure'],
          evidenceRequired: ['Privacy policy', 'Consent records'],
        },
      ],
      assessmentSchedule: 'ANNUAL',
    };
  }

  // Generate compliance report
  async generateComplianceReport(
    assessments: ComplianceAssessment[],
    reportType: string
  ): Promise<ComplianceReport> {
    return this.reportingService.generateReport({
      assessments,
      reportType,
      includeEvidence: true,
      includeRemediation: true,
    });
  }

  // GDPR specific compliance
  async assessGDPRCompliance(): Promise<GDPRAssessment> {
    return {
      dataInventory: await this.assessDataInventory(),
      lawfulBasis: await this.assessLawfulBasis(),
      dataSubjectRights: await this.assessDataSubjectRights(),
      dataProtection: await this.assessDataProtection(),
      crossBorderTransfers: await this.assessCrossBorderTransfers(),
      breachNotification: await this.assessBreachReadiness(),
      dpo: await this.assessDPORequirements(),
    };
  }
}
```

---

## Chapter Summary

This chapter established comprehensive security frameworks for cryonics asset management:

1. **Security Architecture**: Multi-layer defense with temporal considerations
2. **Cryptographic Security**: Encryption, key management, and post-quantum readiness
3. **Access Control**: Identity management, authorization, and MFA
4. **Data Protection**: Classification, encryption, DLP, and retention
5. **Threat Detection**: Real-time monitoring and incident response
6. **Compliance**: SOC 2, GDPR, HIPAA, and financial regulatory frameworks

Security measures are designed to protect assets across extended time horizons while maintaining accessibility for authorized parties.

---

*Next Chapter: Implementation - Development guidelines and deployment strategies*
