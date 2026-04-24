# 제7장: 보안

## 포괄적 보안 프레임워크

본 장에서는 WIA Cryo-Consent 시스템의 종합적인 보안 아키텍처를 정의합니다. 동의 데이터의 기밀성, 무결성, 가용성을 잠재적으로 무한한 시간 동안 보호하기 위한 조치를 포함합니다.

---

## 7.1 보안 아키텍처

```typescript
// 핵심 보안 아키텍처
interface SecurityArchitecture {
  principles: {
    defenseInDepth: '다중 보안 계층';
    leastPrivilege: '최소 권한 원칙';
    zeroTrust: '절대 신뢰하지 않고 항상 검증';
    dataMinimization: '필요한 데이터만 수집';
    privacyByDesign: '설계 시점부터 프라이버시 고려';
  };

  layers: {
    network: '네트워크 보안';
    application: '애플리케이션 보안';
    data: '데이터 보안';
    identity: '신원 및 접근 관리';
    audit: '감사 및 모니터링';
  };

  compliance: {
    hipaa: '미국 의료정보보호법';
    gdpr: 'EU 일반개인정보보호법';
    pipa: '한국 개인정보보호법';
    soc2: 'SOC 2 Type II';
  };
}

// 보안 서비스 구현
class ConsentSecurityService {
  private cryptoService: CryptographicService;
  private accessControl: AccessControlService;
  private auditService: SecurityAuditService;
  private threatDetection: ThreatDetectionService;

  constructor(config: SecurityConfig) {
    this.cryptoService = new CryptographicService(config.crypto);
    this.accessControl = new AccessControlService(config.accessControl);
    this.auditService = new SecurityAuditService(config.audit);
    this.threatDetection = new ThreatDetectionService(config.threats);
  }

  // 동의 데이터 보안 적용
  async secureConsentData(
    consent: ConsentRecord,
    context: SecurityContext
  ): Promise<SecuredConsent> {
    // 민감 필드 분류
    const classification = this.classifyFields(consent);

    // 기밀성 수준에 따른 암호화
    const encryptedData = await this.encryptByClassification(
      consent,
      classification
    );

    // 무결성 보호 추가
    const integrityProtected = await this.addIntegrityProtection(
      encryptedData
    );

    // 감사 기록 생성
    await this.auditService.logDataSecuring(consent.id, context);

    return integrityProtected;
  }

  // 필드 분류
  private classifyFields(consent: ConsentRecord): DataClassification {
    return {
      highlyConfidential: [
        'decisions',
        'authority.grantor',
        'documents',
      ],
      confidential: [
        'scope',
        'authority.proxyChain',
        'validity',
      ],
      internal: [
        'metadata',
        'id',
      ],
      public: [
        'category',
        'consentType',
      ],
    };
  }

  // 분류에 따른 암호화
  private async encryptByClassification(
    consent: ConsentRecord,
    classification: DataClassification
  ): Promise<EncryptedConsent> {
    const encrypted: any = { ...consent };

    // 고기밀 필드 - AES-256-GCM + 봉투 암호화
    for (const field of classification.highlyConfidential) {
      const value = this.getFieldValue(consent, field);
      if (value) {
        const encryptedValue = await this.cryptoService.encryptWithEnvelope(
          value,
          {
            algorithm: 'AES-256-GCM',
            keyRotation: 'QUARTERLY',
          }
        );
        this.setFieldValue(encrypted, field, encryptedValue);
      }
    }

    // 기밀 필드 - AES-256-GCM
    for (const field of classification.confidential) {
      const value = this.getFieldValue(consent, field);
      if (value) {
        const encryptedValue = await this.cryptoService.encrypt(
          value,
          { algorithm: 'AES-256-GCM' }
        );
        this.setFieldValue(encrypted, field, encryptedValue);
      }
    }

    return encrypted;
  }

  // 무결성 보호 추가
  private async addIntegrityProtection(
    data: EncryptedConsent
  ): Promise<SecuredConsent> {
    // 디지털 서명 생성
    const signature = await this.cryptoService.sign(data);

    // 해시 체인 앵커
    const hashChainEntry = await this.cryptoService.addToHashChain(
      data,
      signature
    );

    // 블록체인 앵커 (선택적)
    const blockchainAnchor = await this.cryptoService.anchorToBlockchain(
      hashChainEntry.hash
    );

    return {
      data,
      integrity: {
        signature,
        hashChainEntry,
        blockchainAnchor,
        timestamp: new Date(),
      },
    };
  }
}

interface SecuredConsent {
  data: EncryptedConsent;
  integrity: {
    signature: DigitalSignature;
    hashChainEntry: HashChainEntry;
    blockchainAnchor?: BlockchainAnchor;
    timestamp: Date;
  };
}

interface DataClassification {
  highlyConfidential: string[];
  confidential: string[];
  internal: string[];
  public: string[];
}
```

---

## 7.2 암호화 서비스

```typescript
// 암호화 서비스 구현
class CryptographicService {
  private keyManagement: KeyManagementService;
  private hsmProvider: HSMProvider;

  constructor(config: CryptoConfig) {
    this.keyManagement = new KeyManagementService(config.keyManagement);
    this.hsmProvider = new HSMProvider(config.hsm);
  }

  // 봉투 암호화
  async encryptWithEnvelope(
    data: any,
    options: EncryptionOptions
  ): Promise<EnvelopeEncryptedData> {
    // 데이터 암호화 키(DEK) 생성
    const dek = await this.generateDataKey();

    // DEK로 데이터 암호화
    const serialized = JSON.stringify(data);
    const encrypted = await this.encryptWithKey(
      Buffer.from(serialized),
      dek,
      options.algorithm
    );

    // 키 암호화 키(KEK)로 DEK 래핑
    const kek = await this.keyManagement.getKeyEncryptionKey();
    const wrappedDek = await this.wrapKey(dek, kek);

    // DEK 메모리에서 안전하게 삭제
    this.secureErase(dek);

    return {
      ciphertext: encrypted.ciphertext,
      iv: encrypted.iv,
      authTag: encrypted.authTag,
      wrappedKey: wrappedDek,
      keyId: kek.id,
      algorithm: options.algorithm,
      encryptedAt: new Date(),
    };
  }

  // 복호화
  async decryptEnvelope(
    encrypted: EnvelopeEncryptedData
  ): Promise<any> {
    // KEK 조회
    const kek = await this.keyManagement.getKey(encrypted.keyId);

    // DEK 언래핑
    const dek = await this.unwrapKey(encrypted.wrappedKey, kek);

    // 데이터 복호화
    const decrypted = await this.decryptWithKey(
      encrypted.ciphertext,
      dek,
      encrypted.iv,
      encrypted.authTag,
      encrypted.algorithm
    );

    // DEK 메모리에서 안전하게 삭제
    this.secureErase(dek);

    return JSON.parse(decrypted.toString());
  }

  // 디지털 서명
  async sign(data: any): Promise<DigitalSignature> {
    const signingKey = await this.keyManagement.getSigningKey();
    const serialized = this.canonicalize(data);
    const hash = await this.hash(serialized, 'SHA-256');

    // HSM에서 서명 수행
    const signature = await this.hsmProvider.sign(hash, signingKey);

    return {
      signature,
      algorithm: 'ECDSA-P256',
      keyId: signingKey.id,
      signedAt: new Date(),
      dataHash: hash.toString('hex'),
    };
  }

  // 서명 검증
  async verifySignature(
    data: any,
    signature: DigitalSignature
  ): Promise<SignatureVerificationResult> {
    // 공개키 조회
    const publicKey = await this.keyManagement.getPublicKey(signature.keyId);

    // 데이터 해시 계산
    const serialized = this.canonicalize(data);
    const hash = await this.hash(serialized, 'SHA-256');

    // 해시 일치 확인
    if (hash.toString('hex') !== signature.dataHash) {
      return {
        valid: false,
        reason: '데이터 해시 불일치',
      };
    }

    // 서명 검증
    const valid = await this.hsmProvider.verify(
      hash,
      signature.signature,
      publicKey
    );

    return {
      valid,
      verifiedAt: new Date(),
      keyId: signature.keyId,
    };
  }

  // 해시 체인에 추가
  async addToHashChain(
    data: any,
    signature: DigitalSignature
  ): Promise<HashChainEntry> {
    // 현재 체인 헤드 가져오기
    const currentHead = await this.hashChainService.getHead();

    // 새 엔트리 해시 계산
    const entryData = {
      data: this.canonicalize(data),
      signature,
      previousHash: currentHead?.hash || 'GENESIS',
      timestamp: new Date().toISOString(),
    };

    const hash = await this.hash(JSON.stringify(entryData), 'SHA-256');

    // 새 엔트리 저장
    const entry: HashChainEntry = {
      entryId: generateEntryId(),
      hash: hash.toString('hex'),
      previousHash: currentHead?.hash || 'GENESIS',
      data: entryData,
      timestamp: new Date(),
    };

    await this.hashChainService.appendEntry(entry);

    return entry;
  }

  // 블록체인 앵커링
  async anchorToBlockchain(hash: string): Promise<BlockchainAnchor> {
    // 앵커 트랜잭션 생성
    const tx = await this.blockchainService.createAnchorTransaction({
      hash,
      timestamp: new Date().toISOString(),
      type: 'CONSENT_DATA',
    });

    // 트랜잭션 제출
    const receipt = await this.blockchainService.submitTransaction(tx);

    return {
      network: 'WIA-CONSENT-CHAIN',
      transactionId: receipt.txId,
      blockNumber: receipt.blockNumber,
      blockHash: receipt.blockHash,
      timestamp: receipt.timestamp,
      hash,
    };
  }

  // 해시 계산
  private async hash(data: string | Buffer, algorithm: string): Promise<Buffer> {
    const crypto = require('crypto');
    return crypto.createHash(algorithm.toLowerCase()).update(data).digest();
  }

  // 정규화 (일관된 해싱을 위해)
  private canonicalize(data: any): string {
    return JSON.stringify(data, Object.keys(data).sort());
  }

  // 안전한 메모리 삭제
  private secureErase(buffer: Buffer): void {
    buffer.fill(0);
  }
}

interface EnvelopeEncryptedData {
  ciphertext: Buffer;
  iv: Buffer;
  authTag: Buffer;
  wrappedKey: Buffer;
  keyId: string;
  algorithm: string;
  encryptedAt: Date;
}

interface DigitalSignature {
  signature: Buffer;
  algorithm: string;
  keyId: string;
  signedAt: Date;
  dataHash: string;
}

interface HashChainEntry {
  entryId: string;
  hash: string;
  previousHash: string;
  data: any;
  timestamp: Date;
}

interface BlockchainAnchor {
  network: string;
  transactionId: string;
  blockNumber: number;
  blockHash: string;
  timestamp: Date;
  hash: string;
}
```

---

## 7.3 키 관리

```typescript
// 키 관리 서비스
class KeyManagementService {
  private keyVault: KeyVaultProvider;
  private keyRotationScheduler: KeyRotationScheduler;
  private keyHistory: KeyHistoryService;

  constructor(config: KeyManagementConfig) {
    this.keyVault = new KeyVaultProvider(config.vault);
    this.keyRotationScheduler = new KeyRotationScheduler(config.rotation);
    this.keyHistory = new KeyHistoryService(config.history);
  }

  // 마스터 키 생성
  async createMasterKey(
    purpose: KeyPurpose,
    options: MasterKeyOptions
  ): Promise<MasterKeyInfo> {
    // HSM에서 키 생성
    const key = await this.keyVault.generateKey({
      algorithm: options.algorithm || 'AES-256',
      purpose,
      extractable: false,
      usage: options.usage || ['ENCRYPT', 'DECRYPT', 'WRAP', 'UNWRAP'],
    });

    // 키 메타데이터 저장
    const keyInfo: MasterKeyInfo = {
      keyId: key.id,
      version: 1,
      algorithm: key.algorithm,
      purpose,
      status: 'ACTIVE',
      createdAt: new Date(),
      expiresAt: this.calculateExpiration(options.rotationPeriod),
      rotationPeriod: options.rotationPeriod,
    };

    await this.keyHistory.recordKeyCreation(keyInfo);

    // 순환 예약
    await this.keyRotationScheduler.scheduleRotation(keyInfo);

    return keyInfo;
  }

  // 키 순환
  async rotateKey(keyId: string): Promise<KeyRotationResult> {
    // 현재 키 정보 가져오기
    const currentKey = await this.getKeyInfo(keyId);
    if (!currentKey) {
      throw new Error(`키 ${keyId}를 찾을 수 없습니다`);
    }

    // 새 버전 생성
    const newKey = await this.keyVault.generateKey({
      algorithm: currentKey.algorithm,
      purpose: currentKey.purpose,
      extractable: false,
    });

    // 현재 키를 비활성화로 표시
    await this.updateKeyStatus(keyId, 'ROTATING');

    // 새 키 정보
    const newKeyInfo: MasterKeyInfo = {
      keyId: newKey.id,
      version: currentKey.version + 1,
      algorithm: currentKey.algorithm,
      purpose: currentKey.purpose,
      status: 'ACTIVE',
      createdAt: new Date(),
      expiresAt: this.calculateExpiration(currentKey.rotationPeriod),
      rotationPeriod: currentKey.rotationPeriod,
      previousKeyId: keyId,
    };

    await this.keyHistory.recordKeyRotation(currentKey, newKeyInfo);

    // 이전 키를 비활성화로 표시 (복호화에는 계속 사용 가능)
    await this.updateKeyStatus(keyId, 'INACTIVE');

    // 새 키에 대한 순환 예약
    await this.keyRotationScheduler.scheduleRotation(newKeyInfo);

    return {
      previousKeyId: keyId,
      newKeyId: newKey.id,
      rotatedAt: new Date(),
    };
  }

  // 키 암호화 키 가져오기
  async getKeyEncryptionKey(): Promise<CryptoKey> {
    const activeKEK = await this.findActiveKey('KEY_ENCRYPTION');
    if (!activeKEK) {
      throw new Error('활성 키 암호화 키를 찾을 수 없습니다');
    }

    return this.keyVault.getKey(activeKEK.keyId);
  }

  // 서명 키 가져오기
  async getSigningKey(): Promise<CryptoKey> {
    const activeSigningKey = await this.findActiveKey('SIGNING');
    if (!activeSigningKey) {
      throw new Error('활성 서명 키를 찾을 수 없습니다');
    }

    return this.keyVault.getKey(activeSigningKey.keyId);
  }

  // 비활성 키로 복호화 (레거시 데이터용)
  async decryptWithHistoricalKey(
    encrypted: EnvelopeEncryptedData
  ): Promise<Buffer> {
    // 키 버전 체인 가져오기
    const keyChain = await this.keyHistory.getKeyChain(encrypted.keyId);

    // 올바른 키 찾기
    const decryptionKey = keyChain.find(k => k.keyId === encrypted.keyId);
    if (!decryptionKey) {
      throw new Error(`키 ${encrypted.keyId}를 찾을 수 없습니다`);
    }

    // 키가 복호화에 여전히 유효한지 확인
    if (decryptionKey.status === 'DESTROYED') {
      throw new Error('키가 파기되어 복호화할 수 없습니다');
    }

    const key = await this.keyVault.getKey(encrypted.keyId);
    return this.cryptoService.decryptWithKey(
      encrypted.ciphertext,
      key,
      encrypted.iv,
      encrypted.authTag,
      encrypted.algorithm
    );
  }

  // 키 파기 (주의해서 사용)
  async destroyKey(
    keyId: string,
    authorization: DestructionAuthorization
  ): Promise<KeyDestructionResult> {
    // 여러 승인 필요
    if (!await this.validateDestructionAuthorization(authorization)) {
      throw new Error('키 파기 승인이 유효하지 않습니다');
    }

    // 이 키로 암호화된 데이터가 있는지 확인
    const encryptedRecords = await this.findRecordsEncryptedWith(keyId);
    if (encryptedRecords.length > 0) {
      throw new Error(
        `이 키로 암호화된 레코드가 ${encryptedRecords.length}개 있습니다. 먼저 재암호화해야 합니다.`
      );
    }

    // 키 자료 파기
    await this.keyVault.destroyKey(keyId);

    // 파기 기록
    const result: KeyDestructionResult = {
      keyId,
      destroyedAt: new Date(),
      authorizedBy: authorization.authorizers,
      reason: authorization.reason,
    };

    await this.keyHistory.recordKeyDestruction(result);

    return result;
  }

  // 데이터 재암호화 (키 순환용)
  async reencryptData(
    consentId: string,
    fromKeyId: string,
    toKeyId: string
  ): Promise<ReencryptionResult> {
    // 이전 키로 복호화
    const encrypted = await this.storageService.getEncrypted(consentId);
    const decrypted = await this.decryptWithHistoricalKey({
      ...encrypted,
      keyId: fromKeyId,
    });

    // 새 키로 재암호화
    const newKey = await this.keyVault.getKey(toKeyId);
    const reencrypted = await this.cryptoService.encryptWithKey(
      decrypted,
      newKey,
      'AES-256-GCM'
    );

    // 저장
    await this.storageService.updateEncrypted(consentId, {
      ...reencrypted,
      keyId: toKeyId,
    });

    return {
      consentId,
      fromKeyId,
      toKeyId,
      reencryptedAt: new Date(),
    };
  }
}

enum KeyPurpose {
  MASTER = 'MASTER',
  KEY_ENCRYPTION = 'KEY_ENCRYPTION',
  DATA_ENCRYPTION = 'DATA_ENCRYPTION',
  SIGNING = 'SIGNING',
  VERIFICATION = 'VERIFICATION',
}

interface MasterKeyInfo {
  keyId: string;
  version: number;
  algorithm: string;
  purpose: KeyPurpose;
  status: 'ACTIVE' | 'INACTIVE' | 'ROTATING' | 'DESTROYED';
  createdAt: Date;
  expiresAt: Date;
  rotationPeriod: string;
  previousKeyId?: string;
}

interface KeyRotationResult {
  previousKeyId: string;
  newKeyId: string;
  rotatedAt: Date;
}

interface KeyDestructionResult {
  keyId: string;
  destroyedAt: Date;
  authorizedBy: string[];
  reason: string;
}
```

---

## 7.4 접근 제어

```typescript
// 접근 제어 서비스
class AccessControlService {
  private rbacService: RBACService;
  private abacService: ABACService;
  private sessionManager: SessionManager;

  constructor(config: AccessControlConfig) {
    this.rbacService = new RBACService(config.rbac);
    this.abacService = new ABACService(config.abac);
    this.sessionManager = new SessionManager(config.session);
  }

  // 권한 확인
  async checkPermission(
    subject: Subject,
    resource: Resource,
    action: Action,
    context: AccessContext
  ): Promise<AccessDecision> {
    // 1단계: RBAC 확인
    const rbacDecision = await this.rbacService.checkAccess(
      subject.roles,
      resource.type,
      action
    );

    if (rbacDecision.denied) {
      return {
        allowed: false,
        reason: `역할 권한 거부: ${rbacDecision.reason}`,
        auditId: await this.logAccessAttempt(subject, resource, action, false),
      };
    }

    // 2단계: ABAC 확인 (세분화된 제어)
    const abacDecision = await this.abacService.evaluate({
      subject,
      resource,
      action,
      environment: context,
    });

    if (!abacDecision.allowed) {
      return {
        allowed: false,
        reason: `속성 정책 거부: ${abacDecision.reason}`,
        auditId: await this.logAccessAttempt(subject, resource, action, false),
      };
    }

    // 3단계: 동의별 접근 제어 확인
    if (resource.type === 'CONSENT') {
      const consentAcl = await this.checkConsentACL(
        subject,
        resource.id,
        action
      );

      if (!consentAcl.allowed) {
        return {
          allowed: false,
          reason: `동의 접근 제어 거부: ${consentAcl.reason}`,
          auditId: await this.logAccessAttempt(subject, resource, action, false),
        };
      }
    }

    return {
      allowed: true,
      constraints: [...rbacDecision.constraints, ...abacDecision.constraints],
      auditId: await this.logAccessAttempt(subject, resource, action, true),
    };
  }

  // 동의별 ACL 확인
  private async checkConsentACL(
    subject: Subject,
    consentId: string,
    action: Action
  ): Promise<AccessDecision> {
    // 동의의 접근 제어 설정 가져오기
    const consent = await this.consentService.getConsent(consentId);
    if (!consent) {
      return { allowed: false, reason: '동의를 찾을 수 없습니다' };
    }

    // 환자는 항상 자신의 동의에 접근 가능
    if (subject.id === consent.patientId) {
      return { allowed: true };
    }

    // 권한 있는 대리인 확인
    const isAuthorizedProxy = await this.checkProxyAuthorization(
      subject.id,
      consent,
      action
    );
    if (isAuthorizedProxy) {
      return { allowed: true };
    }

    // 권한 있는 역할 확인
    const authorizedRoles = this.getAuthorizedRolesForAction(action);
    const hasAuthorizedRole = subject.roles.some(r =>
      authorizedRoles.includes(r)
    );
    if (hasAuthorizedRole) {
      return { allowed: true };
    }

    // 개별 접근 권한 확인
    const individualAccess = await this.checkIndividualAccess(
      subject.id,
      consent,
      action
    );
    if (individualAccess) {
      return { allowed: true };
    }

    return {
      allowed: false,
      reason: '이 동의에 대한 접근 권한이 없습니다',
    };
  }

  // 대리인 권한 확인
  private async checkProxyAuthorization(
    subjectId: string,
    consent: ConsentRecord,
    action: Action
  ): Promise<boolean> {
    const proxyChain = consent.authority.proxyChain || [];

    for (const proxyEntry of proxyChain) {
      const proxy = proxyEntry.proxy;

      // 주체가 이 대리인인지 확인
      let isThisProxy = false;
      if (proxy.individual && proxy.individual.name === subjectId) {
        isThisProxy = true;
      }
      if (proxy.organization) {
        const isRepresentative = proxy.organization.authorizedRepresentatives?.includes(subjectId);
        if (isRepresentative) {
          isThisProxy = true;
        }
      }

      if (isThisProxy) {
        // 대리인이 이 작업에 대한 권한이 있는지 확인
        const actionCategory = this.mapActionToCategory(action);
        if (proxy.authorityScope.categories.includes(actionCategory)) {
          // 제외 확인
          if (!proxy.authorityScope.exclusions.includes(action)) {
            return true;
          }
        }
      }
    }

    return false;
  }

  // 역할 정의
  getRoleDefinitions(): RoleDefinition[] {
    return [
      {
        role: 'PATIENT',
        description: '환자 - 자신의 동의 완전 제어',
        permissions: [
          'consent:read:own',
          'consent:write:own',
          'consent:delete:own',
          'proxy:manage:own',
        ],
      },
      {
        role: 'PROXY',
        description: '지정된 대리인',
        permissions: [
          'consent:read:delegated',
          'consent:write:delegated',
          'decision:make:delegated',
        ],
      },
      {
        role: 'MEDICAL_STAFF',
        description: '의료진',
        permissions: [
          'consent:read:assigned',
          'consent:query:emergency',
          'patient:view:basic',
        ],
      },
      {
        role: 'LEGAL_STAFF',
        description: '법무팀',
        permissions: [
          'consent:read:all',
          'consent:verify:all',
          'document:access:all',
        ],
      },
      {
        role: 'ADMIN',
        description: '시스템 관리자',
        permissions: [
          'system:configure',
          'user:manage',
          'audit:view',
          'report:generate',
        ],
      },
      {
        role: 'AUDITOR',
        description: '감사자',
        permissions: [
          'audit:view:all',
          'report:generate:compliance',
          'consent:view:metadata',
        ],
      },
    ];
  }
}

interface Subject {
  id: string;
  type: 'USER' | 'SERVICE' | 'SYSTEM';
  roles: string[];
  attributes: Record<string, any>;
}

interface Resource {
  type: 'CONSENT' | 'PATIENT' | 'DOCUMENT' | 'SYSTEM';
  id: string;
  attributes: Record<string, any>;
}

type Action = 'READ' | 'WRITE' | 'DELETE' | 'QUERY' | 'EXECUTE' | 'MANAGE';

interface AccessContext {
  timestamp: Date;
  ipAddress: string;
  userAgent: string;
  location?: string;
  sessionId?: string;
}

interface AccessDecision {
  allowed: boolean;
  reason?: string;
  constraints?: AccessConstraint[];
  auditId?: string;
}

interface RoleDefinition {
  role: string;
  description: string;
  permissions: string[];
}
```

---

## 7.5 감사 로깅

```typescript
// 보안 감사 서비스
class SecurityAuditService {
  private auditLogger: AuditLogger;
  private integrityService: IntegrityService;
  private alertService: AlertService;

  constructor(config: AuditConfig) {
    this.auditLogger = new AuditLogger(config.logging);
    this.integrityService = new IntegrityService(config.integrity);
    this.alertService = new AlertService(config.alerts);
  }

  // 감사 이벤트 기록
  async logEvent(event: AuditEvent): Promise<AuditRecord> {
    // 감사 레코드 생성
    const record: AuditRecord = {
      auditId: generateAuditId(),
      timestamp: new Date(),
      eventType: event.type,
      eventCategory: event.category,
      actor: {
        id: event.actorId,
        type: event.actorType,
        roles: event.actorRoles,
        ipAddress: event.ipAddress,
        userAgent: event.userAgent,
      },
      target: {
        type: event.targetType,
        id: event.targetId,
      },
      action: event.action,
      result: event.result,
      details: event.details,
      sensitivity: this.calculateSensitivity(event),
    };

    // 무결성 보호
    record.integrity = await this.integrityService.protect(record);

    // 저장
    await this.auditLogger.write(record);

    // 민감 이벤트에 대한 알림 확인
    await this.checkAlerts(record);

    return record;
  }

  // 접근 시도 기록
  async logAccessAttempt(
    subject: Subject,
    resource: Resource,
    action: Action,
    allowed: boolean,
    reason?: string
  ): Promise<string> {
    const event: AuditEvent = {
      type: allowed ? 'ACCESS_GRANTED' : 'ACCESS_DENIED',
      category: 'ACCESS_CONTROL',
      actorId: subject.id,
      actorType: subject.type,
      actorRoles: subject.roles,
      targetType: resource.type,
      targetId: resource.id,
      action,
      result: allowed ? 'SUCCESS' : 'DENIED',
      details: {
        reason,
        resourceAttributes: resource.attributes,
      },
    };

    const record = await this.logEvent(event);
    return record.auditId;
  }

  // 데이터 접근 기록
  async logDataAccess(
    userId: string,
    consentId: string,
    accessType: 'READ' | 'WRITE' | 'DELETE',
    fields?: string[]
  ): Promise<string> {
    const event: AuditEvent = {
      type: `DATA_${accessType}`,
      category: 'DATA_ACCESS',
      actorId: userId,
      actorType: 'USER',
      targetType: 'CONSENT',
      targetId: consentId,
      action: accessType,
      result: 'SUCCESS',
      details: {
        fieldsAccessed: fields,
      },
    };

    const record = await this.logEvent(event);
    return record.auditId;
  }

  // 보안 이벤트 기록
  async logSecurityEvent(
    eventType: SecurityEventType,
    details: SecurityEventDetails
  ): Promise<string> {
    const event: AuditEvent = {
      type: eventType,
      category: 'SECURITY',
      actorId: details.actorId || 'SYSTEM',
      actorType: details.actorType || 'SYSTEM',
      targetType: details.targetType,
      targetId: details.targetId,
      action: details.action,
      result: details.result,
      details: details.additionalInfo,
    };

    const record = await this.logEvent(event);

    // 보안 이벤트에 대한 즉시 알림 트리거
    if (this.isHighSeveritySecurityEvent(eventType)) {
      await this.alertService.triggerSecurityAlert(record);
    }

    return record.auditId;
  }

  // 감사 추적 조회
  async getAuditTrail(query: AuditQuery): Promise<AuditTrailResult> {
    // 권한 확인
    if (!await this.canViewAuditTrail(query.requestor, query)) {
      throw new UnauthorizedError('감사 로그를 볼 권한이 없습니다');
    }

    // 기록 조회
    const records = await this.auditLogger.query({
      startDate: query.startDate,
      endDate: query.endDate,
      actorId: query.actorId,
      targetId: query.targetId,
      eventTypes: query.eventTypes,
      categories: query.categories,
      offset: query.offset,
      limit: query.limit,
    });

    // 무결성 검증
    const verificationResults = await Promise.all(
      records.map(r => this.integrityService.verify(r))
    );

    return {
      records,
      total: await this.auditLogger.count(query),
      integrityStatus: {
        allValid: verificationResults.every(v => v.valid),
        invalidCount: verificationResults.filter(v => !v.valid).length,
      },
    };
  }

  // 무결성 체인 검증
  async verifyIntegrityChain(
    startDate: Date,
    endDate: Date
  ): Promise<IntegrityVerificationResult> {
    const records = await this.auditLogger.getRecordsByDateRange(
      startDate,
      endDate
    );

    let previousHash: string | null = null;
    const invalidRecords: string[] = [];

    for (const record of records) {
      // 각 레코드의 무결성 검증
      const recordValid = await this.integrityService.verify(record);
      if (!recordValid.valid) {
        invalidRecords.push(record.auditId);
        continue;
      }

      // 체인 연속성 검증
      if (previousHash !== null && record.integrity.previousHash !== previousHash) {
        invalidRecords.push(record.auditId);
      }

      previousHash = record.integrity.hash;
    }

    return {
      verified: invalidRecords.length === 0,
      totalRecords: records.length,
      invalidRecords,
      verifiedAt: new Date(),
    };
  }

  // 알림 확인
  private async checkAlerts(record: AuditRecord): Promise<void> {
    const alertRules = await this.alertService.getActiveRules();

    for (const rule of alertRules) {
      if (this.matchesRule(record, rule)) {
        await this.alertService.trigger(rule, record);
      }
    }
  }

  private matchesRule(record: AuditRecord, rule: AlertRule): boolean {
    // 이벤트 유형 일치 확인
    if (rule.eventTypes && !rule.eventTypes.includes(record.eventType)) {
      return false;
    }

    // 카테고리 일치 확인
    if (rule.categories && !rule.categories.includes(record.eventCategory)) {
      return false;
    }

    // 민감도 임계값 확인
    if (rule.minSensitivity && record.sensitivity < rule.minSensitivity) {
      return false;
    }

    // 사용자 정의 조건 확인
    if (rule.customCondition) {
      return this.evaluateCondition(rule.customCondition, record);
    }

    return true;
  }
}

interface AuditEvent {
  type: string;
  category: string;
  actorId: string;
  actorType: string;
  actorRoles?: string[];
  ipAddress?: string;
  userAgent?: string;
  targetType: string;
  targetId: string;
  action: string;
  result: string;
  details?: Record<string, any>;
}

interface AuditRecord {
  auditId: string;
  timestamp: Date;
  eventType: string;
  eventCategory: string;
  actor: {
    id: string;
    type: string;
    roles?: string[];
    ipAddress?: string;
    userAgent?: string;
  };
  target: {
    type: string;
    id: string;
  };
  action: string;
  result: string;
  details?: Record<string, any>;
  sensitivity: number;
  integrity?: {
    hash: string;
    previousHash: string;
    signature: string;
  };
}

enum SecurityEventType {
  AUTHENTICATION_FAILURE = 'AUTHENTICATION_FAILURE',
  MULTIPLE_AUTH_FAILURES = 'MULTIPLE_AUTH_FAILURES',
  UNAUTHORIZED_ACCESS_ATTEMPT = 'UNAUTHORIZED_ACCESS_ATTEMPT',
  PRIVILEGE_ESCALATION = 'PRIVILEGE_ESCALATION',
  DATA_EXFILTRATION_ATTEMPT = 'DATA_EXFILTRATION_ATTEMPT',
  ENCRYPTION_FAILURE = 'ENCRYPTION_FAILURE',
  KEY_COMPROMISE_SUSPECTED = 'KEY_COMPROMISE_SUSPECTED',
  INTEGRITY_VIOLATION = 'INTEGRITY_VIOLATION',
}
```

---

## 7.6 위협 탐지

```typescript
// 위협 탐지 서비스
class ThreatDetectionService {
  private anomalyDetector: AnomalyDetector;
  private patternMatcher: ThreatPatternMatcher;
  private responseService: ThreatResponseService;

  constructor(config: ThreatDetectionConfig) {
    this.anomalyDetector = new AnomalyDetector(config.anomaly);
    this.patternMatcher = new ThreatPatternMatcher(config.patterns);
    this.responseService = new ThreatResponseService(config.response);
  }

  // 활동 분석
  async analyzeActivity(activity: UserActivity): Promise<ThreatAnalysis> {
    // 이상 행위 탐지
    const anomalyScore = await this.anomalyDetector.score(activity);

    // 알려진 위협 패턴 매칭
    const patternMatches = await this.patternMatcher.match(activity);

    // 위협 분류
    const threat = this.classifyThreat(anomalyScore, patternMatches);

    if (threat.level !== 'NONE') {
      // 대응 트리거
      await this.responseService.respond(threat, activity);
    }

    return {
      activityId: activity.id,
      anomalyScore,
      patternMatches,
      threatLevel: threat.level,
      threatType: threat.type,
      recommendedActions: threat.actions,
      analyzedAt: new Date(),
    };
  }

  // 실시간 모니터링
  async startRealtimeMonitoring(): Promise<void> {
    const eventStream = await this.eventBus.subscribe('USER_ACTIVITY');

    eventStream.on('event', async (event: UserActivity) => {
      const analysis = await this.analyzeActivity(event);

      if (analysis.threatLevel === 'CRITICAL' || analysis.threatLevel === 'HIGH') {
        // 즉시 대응
        await this.handleImmediateThreat(analysis, event);
      }
    });
  }

  // 즉각적 위협 처리
  private async handleImmediateThreat(
    analysis: ThreatAnalysis,
    activity: UserActivity
  ): Promise<void> {
    switch (analysis.threatType) {
      case 'CREDENTIAL_STUFFING':
        await this.responseService.blockIP(activity.ipAddress);
        await this.responseService.notifySecurityTeam(analysis);
        break;

      case 'DATA_EXFILTRATION':
        await this.responseService.terminateSession(activity.sessionId);
        await this.responseService.lockAccount(activity.userId);
        await this.responseService.notifySecurityTeam(analysis);
        break;

      case 'PRIVILEGE_ESCALATION':
        await this.responseService.revokeTemporaryPermissions(activity.userId);
        await this.responseService.notifySecurityTeam(analysis);
        break;

      case 'INSIDER_THREAT':
        await this.responseService.increaseSurveillance(activity.userId);
        await this.responseService.notifySecurityTeam(analysis);
        break;

      default:
        await this.responseService.notifySecurityTeam(analysis);
    }
  }

  // 위협 분류
  private classifyThreat(
    anomalyScore: number,
    patternMatches: PatternMatch[]
  ): ThreatClassification {
    // 우선 패턴 매칭 확인
    const criticalPatterns = patternMatches.filter(p => p.severity === 'CRITICAL');
    if (criticalPatterns.length > 0) {
      return {
        level: 'CRITICAL',
        type: criticalPatterns[0].patternType,
        actions: criticalPatterns[0].recommendedActions,
      };
    }

    const highPatterns = patternMatches.filter(p => p.severity === 'HIGH');
    if (highPatterns.length > 0) {
      return {
        level: 'HIGH',
        type: highPatterns[0].patternType,
        actions: highPatterns[0].recommendedActions,
      };
    }

    // 이상 점수 기반 분류
    if (anomalyScore > 0.9) {
      return {
        level: 'HIGH',
        type: 'ANOMALOUS_BEHAVIOR',
        actions: ['REVIEW_ACTIVITY', 'NOTIFY_SECURITY'],
      };
    }

    if (anomalyScore > 0.7) {
      return {
        level: 'MEDIUM',
        type: 'SUSPICIOUS_BEHAVIOR',
        actions: ['INCREASE_MONITORING'],
      };
    }

    if (anomalyScore > 0.5) {
      return {
        level: 'LOW',
        type: 'UNUSUAL_BEHAVIOR',
        actions: ['LOG_FOR_REVIEW'],
      };
    }

    return {
      level: 'NONE',
      type: 'NORMAL',
      actions: [],
    };
  }
}

interface UserActivity {
  id: string;
  userId: string;
  sessionId: string;
  ipAddress: string;
  userAgent: string;
  action: string;
  resource: string;
  timestamp: Date;
  metadata: Record<string, any>;
}

interface ThreatAnalysis {
  activityId: string;
  anomalyScore: number;
  patternMatches: PatternMatch[];
  threatLevel: 'NONE' | 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  threatType: string;
  recommendedActions: string[];
  analyzedAt: Date;
}

interface ThreatClassification {
  level: 'NONE' | 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';
  type: string;
  actions: string[];
}

interface PatternMatch {
  patternId: string;
  patternType: string;
  severity: string;
  confidence: number;
  matchedIndicators: string[];
  recommendedActions: string[];
}
```

---

## 7.7 규정 준수 프레임워크

```typescript
// 규정 준수 모니터링 서비스
class ComplianceMonitoringService {
  private hipaaChecker: HIPAAComplianceChecker;
  private gdprChecker: GDPRComplianceChecker;
  private pipaChecker: PIPAComplianceChecker;

  constructor(config: ComplianceConfig) {
    this.hipaaChecker = new HIPAAComplianceChecker(config.hipaa);
    this.gdprChecker = new GDPRComplianceChecker(config.gdpr);
    this.pipaChecker = new PIPAComplianceChecker(config.pipa);
  }

  // 종합 규정 준수 평가
  async assessCompliance(): Promise<ComplianceAssessment> {
    const assessments = await Promise.all([
      this.hipaaChecker.assess(),
      this.gdprChecker.assess(),
      this.pipaChecker.assess(),
    ]);

    return {
      assessmentDate: new Date(),
      overallStatus: this.calculateOverallStatus(assessments),
      frameworks: {
        hipaa: assessments[0],
        gdpr: assessments[1],
        pipa: assessments[2],
      },
      criticalFindings: this.extractCriticalFindings(assessments),
      recommendations: this.generateRecommendations(assessments),
    };
  }

  // HIPAA 규정 준수 확인
  async checkHIPAACompliance(): Promise<HIPAAAssessment> {
    return {
      privacyRule: await this.hipaaChecker.checkPrivacyRule(),
      securityRule: await this.hipaaChecker.checkSecurityRule(),
      breachNotification: await this.hipaaChecker.checkBreachNotification(),
      administrativeSafeguards: await this.hipaaChecker.checkAdministrative(),
      physicalSafeguards: await this.hipaaChecker.checkPhysical(),
      technicalSafeguards: await this.hipaaChecker.checkTechnical(),
    };
  }

  // GDPR 규정 준수 확인
  async checkGDPRCompliance(): Promise<GDPRAssessment> {
    return {
      lawfulBasis: await this.gdprChecker.checkLawfulBasis(),
      dataSubjectRights: await this.gdprChecker.checkDataSubjectRights(),
      dataProtection: await this.gdprChecker.checkDataProtection(),
      dataTransfers: await this.gdprChecker.checkDataTransfers(),
      dataBreachProcedures: await this.gdprChecker.checkBreachProcedures(),
      dpo: await this.gdprChecker.checkDPO(),
      records: await this.gdprChecker.checkRecordsOfProcessing(),
    };
  }

  // 한국 개인정보보호법(PIPA) 규정 준수 확인
  async checkPIPACompliance(): Promise<PIPAAssessment> {
    return {
      consentRequirements: await this.pipaChecker.checkConsentRequirements(),
      purposeLimitation: await this.pipaChecker.checkPurposeLimitation(),
      minimization: await this.pipaChecker.checkMinimization(),
      safeguards: await this.pipaChecker.checkSafeguards(),
      dataSubjectRights: await this.pipaChecker.checkDataSubjectRights(),
      thirdPartyProvision: await this.pipaChecker.checkThirdPartyProvision(),
      crossBorderTransfer: await this.pipaChecker.checkCrossBorderTransfer(),
    };
  }

  // 규정 준수 보고서 생성
  async generateComplianceReport(
    options: ReportOptions
  ): Promise<ComplianceReport> {
    const assessment = await this.assessCompliance();

    return {
      reportId: generateReportId(),
      generatedAt: new Date(),
      period: options.period,
      assessment,
      controlStatus: await this.getControlStatus(),
      auditFindings: await this.getAuditFindings(options.period),
      incidentSummary: await this.getIncidentSummary(options.period),
      trainingStatus: await this.getTrainingStatus(),
      policyReviewStatus: await this.getPolicyReviewStatus(),
      recommendations: this.prioritizeRecommendations(assessment),
    };
  }
}

interface ComplianceAssessment {
  assessmentDate: Date;
  overallStatus: 'COMPLIANT' | 'PARTIALLY_COMPLIANT' | 'NON_COMPLIANT';
  frameworks: {
    hipaa: HIPAAAssessment;
    gdpr: GDPRAssessment;
    pipa: PIPAAssessment;
  };
  criticalFindings: ComplianceFinding[];
  recommendations: ComplianceRecommendation[];
}

interface HIPAAAssessment {
  privacyRule: RuleAssessment;
  securityRule: RuleAssessment;
  breachNotification: RuleAssessment;
  administrativeSafeguards: RuleAssessment;
  physicalSafeguards: RuleAssessment;
  technicalSafeguards: RuleAssessment;
}

interface GDPRAssessment {
  lawfulBasis: RuleAssessment;
  dataSubjectRights: RuleAssessment;
  dataProtection: RuleAssessment;
  dataTransfers: RuleAssessment;
  dataBreachProcedures: RuleAssessment;
  dpo: RuleAssessment;
  records: RuleAssessment;
}

interface PIPAAssessment {
  consentRequirements: RuleAssessment;
  purposeLimitation: RuleAssessment;
  minimization: RuleAssessment;
  safeguards: RuleAssessment;
  dataSubjectRights: RuleAssessment;
  thirdPartyProvision: RuleAssessment;
  crossBorderTransfer: RuleAssessment;
}

interface RuleAssessment {
  status: 'COMPLIANT' | 'PARTIALLY_COMPLIANT' | 'NON_COMPLIANT';
  score: number;
  findings: ComplianceFinding[];
  controlsImplemented: string[];
  controlsNeeded: string[];
}

interface ComplianceFinding {
  findingId: string;
  severity: 'CRITICAL' | 'HIGH' | 'MEDIUM' | 'LOW';
  framework: string;
  requirement: string;
  description: string;
  remediation: string;
  dueDate?: Date;
}
```

---

*다음 장: 구현 - 프로젝트 구조, 데이터베이스 스키마 및 배포*
