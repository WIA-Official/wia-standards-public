# 제7장: 보안

## 보호 메커니즘 및 규정 준수 프레임워크

### 소개

냉동 보존 자산을 보호하는 것은 기존 금융 보안 문제를 훨씬 넘어서는 고유한 과제를 제시합니다. 자산은 현대의 위협뿐만 아니라 수십 년 또는 수 세기에 걸쳐 나타날 수 있는 취약점으로부터도 보호되어야 합니다. 이 장에서는 암호화 보호, 접근 제어, 데이터 무결성, 규제 준수 및 장기 보존 전략을 포괄하는 포괄적인 보안 프레임워크를 수립하여 확장된 시간 범위에서 자산이 안전하고 접근 가능하게 유지되도록 합니다.

---

## 7.1 보안 아키텍처

### 다중 계층 보안 프레임워크

```typescript
// 보안 아키텍처 개요
interface SecurityArchitecture {
  layers: {
    perimeter: PerimeterSecurity;       // 경계 보안
    network: NetworkSecurity;           // 네트워크 보안
    application: ApplicationSecurity;   // 애플리케이션 보안
    data: DataSecurity;                 // 데이터 보안
    cryptographic: CryptographicSecurity; // 암호화 보안
  };

  principles: SecurityPrinciple[];      // 보안 원칙
  threatModel: ThreatModel;             // 위협 모델
  controlFramework: ControlFramework;   // 제어 프레임워크
  complianceRequirements: ComplianceRequirement[]; // 규정 준수 요구사항
}

// 보안 원칙 정의
const securityPrinciples: SecurityPrinciple[] = [
  {
    id: 'DEFENSE_IN_DEPTH',
    name: '심층 방어',
    description: '다중 계층의 보안 제어',
    implementation: [
      '경계 방화벽 및 WAF',
      '네트워크 세분화',
      '애플리케이션 수준 제어',
      '저장 및 전송 중 데이터 암호화',
      '암호화 무결성 검증',
    ],
  },
  {
    id: 'LEAST_PRIVILEGE',
    name: '최소 권한 원칙',
    description: '필요한 최소한의 접근 권한',
    implementation: [
      '역할 기반 접근 제어(RBAC)',
      '적시 접근 프로비저닝',
      '정기적인 접근 검토',
      '자동 권한 해제',
    ],
  },
  {
    id: 'ZERO_TRUST',
    name: '제로 트러스트 아키텍처',
    description: '절대 신뢰하지 않고 항상 검증',
    implementation: [
      '지속적인 인증',
      '마이크로 세분화',
      '암호화된 통신',
      '장치 신뢰 검증',
    ],
  },
  {
    id: 'CRYPTO_AGILITY',
    name: '암호화 민첩성',
    description: '암호화 기본 요소 업그레이드 능력',
    implementation: [
      '알고리즘 추상화 계층',
      '키 순환 메커니즘',
      '양자 내성 준비',
      '다중 알고리즘 지원',
    ],
  },
  {
    id: 'TEMPORAL_SECURITY',
    name: '시간적 보안',
    description: '확장된 기간에 걸쳐 유지되는 보안',
    implementation: [
      '장기 키 관리',
      '형식 보존 암호화',
      '기술 불가지론적 저장',
      '보안 제어를 위한 승계 계획',
    ],
  },
];

// 보안 서비스 구현
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

  // 새 자산에 대한 보안 초기화
  async secureAsset(asset: CryoAsset): Promise<SecuredAsset> {
    // 자산용 암호화 키 생성
    const encryptionKey = await this.cryptoService.generateAssetKey(asset.id);

    // 민감한 데이터 암호화
    const encryptedData = await this.encryptAssetData(asset, encryptionKey);

    // 무결성 증명 생성
    const integrityProof = await this.createIntegrityProof(asset, encryptedData);

    // 접근 제어 설정
    const accessPolicy = await this.accessControl.createAssetPolicy(asset);

    // 보안 감사에 기록
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

  // 자산 무결성 검증
  async verifyIntegrity(assetId: string): Promise<IntegrityVerification> {
    const asset = await this.getAsset(assetId);
    const storedProof = await this.getIntegrityProof(assetId);

    // 현재 증명 재계산
    const currentProof = await this.calculateIntegrityProof(asset);

    // 증명 비교
    const proofMatch = await this.cryptoService.verifyProof(
      storedProof,
      currentProof
    );

    // 있는 경우 블록체인 앵커 검증
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

    // 검증 기록
    await this.auditService.recordSecurityEvent({
      type: 'INTEGRITY_VERIFICATION',
      assetId,
      result: result.verified ? 'PASS' : 'FAIL',
      timestamp: new Date(),
    });

    return result;
  }

  // 자산 데이터 암호화
  private async encryptAssetData(
    asset: CryoAsset,
    key: EncryptionKey
  ): Promise<EncryptedData> {
    const sensitiveFields = this.identifySensitiveFields(asset);

    for (const field of sensitiveFields) {
      const value = this.getFieldValue(asset, field);
      const encrypted = await this.cryptoService.encrypt(
        Buffer.from(JSON.stringify(value)),
        key.keyId
      );
      this.setFieldValue(asset, field, encrypted);
    }

    return {
      encryptedFields: sensitiveFields,
      keyId: key.keyId,
      algorithm: key.algorithm,
    };
  }

  // 무결성 증명 생성
  private async createIntegrityProof(
    asset: CryoAsset,
    encryptedData: EncryptedData
  ): Promise<IntegrityProof> {
    const dataToHash = {
      assetId: asset.id,
      category: asset.category,
      ownership: asset.ownership,
      encryptedData: encryptedData,
      timestamp: new Date().toISOString(),
    };

    const hash = await this.cryptoService.hash(JSON.stringify(dataToHash));
    const signature = await this.cryptoService.sign(
      Buffer.from(hash),
      this.getSigningKeyId()
    );

    return {
      hash,
      algorithm: 'SHA-384',
      signature,
      createdAt: new Date(),
    };
  }
}
```

---

## 7.2 암호화 보안

### 암호화 및 키 관리

```typescript
// 암호화 서비스 구현
interface CryptographicConfig {
  primaryAlgorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  keyDerivation: 'Argon2id' | 'PBKDF2';
  signing: 'Ed25519' | 'ECDSA-P384';
  hashing: 'SHA-256' | 'SHA-384' | 'BLAKE3';
  postQuantum: boolean;  // 양자 내성 알고리즘 활성화
}

class CryptographicService {
  private config: CryptographicConfig;
  private keyStore: KeyStore;
  private hsmClient: HSMClient | null;

  constructor(config: CryptographicConfig) {
    this.config = config;
    this.keyStore = new KeyStore(config);

    // HSM 사용 가능시 초기화
    if (config.hsmEnabled) {
      this.hsmClient = new HSMClient(config.hsmConfig);
    }
  }

  // 자산용 암호화 키 생성
  async generateAssetKey(assetId: string): Promise<EncryptionKey> {
    // 키 자료 생성
    const keyMaterial = await this.generateKeyMaterial(256);

    // 마스터 키로 래핑
    const wrappedKey = await this.wrapKey(keyMaterial);

    // 키 레코드 생성
    const key: EncryptionKey = {
      keyId: generateKeyId(),
      assetId,
      algorithm: this.config.primaryAlgorithm,
      keyMaterial: wrappedKey,
      version: 1,
      status: 'ACTIVE',
      createdAt: new Date(),
      expiresAt: null,  // 자산 키는 만료되지 않음
      rotationPolicy: {
        automatic: false,
        intervalDays: null,
        onCompromise: true,
      },
    };

    // 키 저장
    await this.keyStore.storeKey(key);

    // HSM 지원 키의 경우
    if (this.hsmClient) {
      await this.hsmClient.importKey(key.keyId, keyMaterial);
    }

    return key;
  }

  // 데이터 암호화
  async encrypt(
    data: Buffer,
    keyId: string
  ): Promise<EncryptedData> {
    const key = await this.keyStore.getKey(keyId);

    // 래핑 해제된 키 자료 가져오기
    const keyMaterial = await this.unwrapKey(key.keyMaterial);

    // 논스 생성
    const nonce = crypto.randomBytes(12);

    // 알고리즘에 따른 암호화
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
        throw new Error(`지원되지 않는 알고리즘: ${this.config.primaryAlgorithm}`);
    }

    // 민감한 데이터 지우기
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

  // 데이터 복호화
  async decrypt(
    encrypted: EncryptedData
  ): Promise<Buffer> {
    const key = await this.keyStore.getKey(encrypted.keyId);

    // 키 버전 일치 확인
    if (encrypted.keyVersion !== key.version) {
      // 이전 키 버전 가져오기
      const historicalKey = await this.keyStore.getKeyVersion(
        encrypted.keyId,
        encrypted.keyVersion
      );
      return this.decryptWithKey(encrypted, historicalKey);
    }

    return this.decryptWithKey(encrypted, key);
  }

  // 키로 복호화
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
        throw new Error(`지원되지 않는 알고리즘: ${encrypted.algorithm}`);
    }

    keyMaterial.fill(0);
    return plaintext;
  }

  // 키 순환
  async rotateKey(keyId: string): Promise<EncryptionKey> {
    const currentKey = await this.keyStore.getKey(keyId);

    // 새 키 자료 생성
    const newKeyMaterial = await this.generateKeyMaterial(256);
    const wrappedNewKey = await this.wrapKey(newKeyMaterial);

    // 새 키 버전 생성
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

    // 업데이트된 키 저장
    await this.keyStore.updateKey(newKey);

    // 해당되는 경우 HSM 업데이트
    if (this.hsmClient) {
      await this.hsmClient.rotateKey(keyId, newKeyMaterial);
    }

    return newKey;
  }

  // 디지털 서명 생성
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
        throw new Error(`지원되지 않는 서명 알고리즘: ${this.config.signing}`);
    }

    return {
      signature: Buffer.from(signature).toString('base64'),
      algorithm: this.config.signing,
      keyId: signingKeyId,
      signedAt: new Date(),
    };
  }

  // 디지털 서명 검증
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
        throw new Error(`지원되지 않는 서명 알고리즘: ${signature.algorithm}`);
    }
  }

  // 양자 내성 키 캡슐화 (미래 대비)
  async generatePostQuantumKeyPair(): Promise<PostQuantumKeyPair> {
    // 키 캡슐화를 위한 CRYSTALS-Kyber 사용
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

// 키 저장소 구현
class KeyStore {
  private db: Database;
  private masterKey: Buffer;

  constructor(config: KeyStoreConfig) {
    this.db = new Database(config.database);
    this.initializeMasterKey(config);
  }

  // 마스터 키 초기화
  private async initializeMasterKey(config: KeyStoreConfig): Promise<void> {
    if (config.masterKeySource === 'HSM') {
      // HSM에서 마스터 키 가져오기
      this.masterKey = await this.getHSMMasterKey(config.hsmConfig);
    } else if (config.masterKeySource === 'KMS') {
      // 클라우드 KMS에서 마스터 키 가져오기
      this.masterKey = await this.getKMSMasterKey(config.kmsConfig);
    } else {
      // 환경에서 파생 (개발 전용)
      this.masterKey = crypto.scryptSync(
        process.env.MASTER_KEY_PASSWORD!,
        process.env.MASTER_KEY_SALT!,
        32
      );
    }
  }

  // 키 저장
  async storeKey(key: EncryptionKey): Promise<void> {
    // 마스터 키로 키 자료 암호화
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

  // 키 조회
  async getKey(keyId: string): Promise<EncryptionKey> {
    const row = await this.db.get('encryption_keys', { key_id: keyId });

    if (!row) {
      throw new Error(`키를 찾을 수 없음: ${keyId}`);
    }

    // 키 자료 복호화
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

## 7.3 접근 제어

### ID 및 권한 관리

```typescript
// 접근 제어 서비스 구현
interface AccessControlConfig {
  model: 'RBAC' | 'ABAC' | 'PBAC';  // 역할/속성/정책 기반
  mfaRequired: boolean;
  sessionTimeout: number;  // 분
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

  // 사용자 인증
  async authenticate(
    credentials: AuthenticationCredentials
  ): Promise<AuthenticationResult> {
    // 기본 자격 증명 검증
    const primaryAuth = await this.identityProvider.validateCredentials(credentials);

    if (!primaryAuth.valid) {
      await this.recordFailedAuthentication(credentials.username);
      return { success: false, reason: primaryAuth.reason };
    }

    // MFA 요구사항 확인
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

    // 세션 생성
    const session = await this.createSession(primaryAuth.userId);

    // 사용자 역할 및 권한 가져오기
    const authorization = await this.getAuthorization(primaryAuth.userId);

    return {
      success: true,
      userId: primaryAuth.userId,
      session,
      authorization,
    };
  }

  // 작업에 대한 권한 확인
  async authorize(
    context: AuthorizationContext
  ): Promise<AuthorizationResult> {
    const { userId, action, resource, resourceId } = context;

    // 사용자의 역할 및 속성 가져오기
    const user = await this.identityProvider.getUser(userId);
    const roles = await this.getUserRoles(userId);
    const attributes = await this.getUserAttributes(userId);

    // 리소스 속성 가져오기
    const resourceAttrs = await this.getResourceAttributes(resource, resourceId);

    // 정책 평가
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

    // 권한 결정 기록
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

  // 자산에 대한 접근 정책 생성
  async createAssetPolicy(asset: CryoAsset): Promise<AccessPolicy> {
    const policy: AccessPolicy = {
      id: generatePolicyId(),
      resourceType: 'ASSET',
      resourceId: asset.id,
      rules: [
        // 소유자는 전체 접근 권한
        {
          id: 'owner-full-access',
          effect: 'PERMIT',
          principals: [{ type: 'OWNER', id: asset.patientId }],
          actions: ['*'],
          conditions: [],
        },
        // 수탁자는 조회 및 관리 가능
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
        // 조직 관리자는 조회 가능
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
        // 기본 거부
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

  // 다단계 인증
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
          phoneNumber: null,  // 설정 예정
          verified: false,
        });
        return { method: 'SMS' };

      default:
        throw new Error(`지원되지 않는 MFA 방식: ${method}`);
    }
  }

  // 세션 관리
  async createSession(userId: string): Promise<Session> {
    // 동시 세션 제한 확인
    const activeSessions = await this.getActiveSessions(userId);
    if (activeSessions.length >= this.config.maxConcurrentSessions) {
      // 가장 오래된 세션 해지
      await this.revokeSession(activeSessions[0].id);
    }

    const session: Session = {
      id: generateSessionId(),
      userId,
      createdAt: new Date(),
      expiresAt: new Date(Date.now() + this.config.sessionTimeout * 60 * 1000),
      lastActivity: new Date(),
      ipAddress: null,  // 호출자가 설정
      userAgent: null,  // 호출자가 설정
      status: 'ACTIVE',
    };

    await this.storeSession(session);
    return session;
  }

  // 긴급 접근
  async grantEmergencyAccess(
    request: EmergencyAccessRequest
  ): Promise<EmergencyAccessGrant> {
    // 긴급 조건 검증
    const validation = await this.validateEmergencyConditions(request);
    if (!validation.valid) {
      throw new Error(`긴급 접근 거부: ${validation.reason}`);
    }

    // 시간 제한 긴급 권한 생성
    const grant: EmergencyAccessGrant = {
      id: generateGrantId(),
      requesterId: request.requesterId,
      resourceType: request.resourceType,
      resourceId: request.resourceId,
      reason: request.reason,
      grantedActions: request.requestedActions,
      grantedAt: new Date(),
      expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000),  // 24시간
      conditions: validation.conditions,
      approvedBy: validation.approvers,
    };

    await this.storeEmergencyGrant(grant);

    // 관련 당사자에게 알림
    await this.notifyEmergencyAccess(grant);

    return grant;
  }
}

// 정책 엔진 구현
class PolicyEngine {
  private policies: Map<string, AccessPolicy>;

  // 정책 평가
  async evaluate(request: PolicyRequest): Promise<PolicyDecision> {
    const applicablePolicies = await this.findApplicablePolicies(request);

    // 각 정책 평가
    const decisions: RuleDecision[] = [];

    for (const policy of applicablePolicies) {
      for (const rule of policy.rules) {
        const ruleDecision = await this.evaluateRule(rule, request);
        decisions.push(ruleDecision);

        // 명시적 DENY에서 단축
        if (ruleDecision.effect === 'DENY' && ruleDecision.matched) {
          return {
            decision: 'DENY',
            reason: ruleDecision.reason,
            matchedRule: rule.id,
          };
        }
      }
    }

    // PERMIT 확인
    const permitDecision = decisions.find(d => d.effect === 'PERMIT' && d.matched);
    if (permitDecision) {
      return {
        decision: 'PERMIT',
        reason: permitDecision.reason,
        matchedRule: permitDecision.ruleId,
        obligations: permitDecision.obligations,
      };
    }

    // 기본 거부
    return {
      decision: 'DENY',
      reason: '일치하는 허용 규칙 없음',
    };
  }

  // 규칙 평가
  private async evaluateRule(
    rule: PolicyRule,
    request: PolicyRequest
  ): Promise<RuleDecision> {
    // 주체 일치 확인
    const principalMatch = this.matchPrincipal(rule.principals, request.subject);
    if (!principalMatch) {
      return { ruleId: rule.id, effect: rule.effect, matched: false };
    }

    // 동작 일치 확인
    const actionMatch = this.matchAction(rule.actions, request.action);
    if (!actionMatch) {
      return { ruleId: rule.id, effect: rule.effect, matched: false };
    }

    // 조건 확인
    for (const condition of rule.conditions) {
      const conditionMet = await this.evaluateCondition(condition, request);
      if (!conditionMet) {
        return {
          ruleId: rule.id,
          effect: rule.effect,
          matched: false,
          reason: `조건 충족 안됨: ${condition.type}`,
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

## 7.4 데이터 보호

### 포괄적인 데이터 보안

```typescript
// 데이터 보호 서비스
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

  // 데이터 분류 및 보호
  async protectData(
    data: any,
    context: DataContext
  ): Promise<ProtectedData> {
    // 데이터 분류
    const classification = await this.classifier.classify(data, context);

    // 분류에 따른 적절한 보호 적용
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

  // 보호 적용
  private async applyProtection(
    data: any,
    classification: DataClassification
  ): Promise<{ data: any; methods: string[] }> {
    const methods: string[] = [];
    let protectedData = data;

    switch (classification.level) {
      case 'HIGHLY_CONFIDENTIAL':
        // 전체 암호화 + PII 토큰화
        protectedData = await this.encryptor.encrypt(protectedData);
        methods.push('ENCRYPTION');

        protectedData = await this.tokenizer.tokenizePII(protectedData);
        methods.push('TOKENIZATION');
        break;

      case 'CONFIDENTIAL':
        // 민감한 필드 암호화
        protectedData = await this.encryptor.encryptFields(
          protectedData,
          classification.sensitiveFields
        );
        methods.push('FIELD_ENCRYPTION');
        break;

      case 'INTERNAL':
        // PII만 토큰화
        protectedData = await this.tokenizer.tokenizePII(protectedData);
        methods.push('TOKENIZATION');
        break;

      case 'PUBLIC':
        // 보호 필요 없음
        break;
    }

    return { data: protectedData, methods };
  }

  // 데이터 유출 방지
  async scanForLeakage(
    data: any,
    destination: string
  ): Promise<DLPScanResult> {
    const findings: DLPFinding[] = [];

    // PII 스캔
    const piiFindings = await this.dlpEngine.scanForPII(data);
    findings.push(...piiFindings);

    // 금융 데이터 스캔
    const financialFindings = await this.dlpEngine.scanForFinancialData(data);
    findings.push(...financialFindings);

    // 자격 증명 스캔
    const credentialFindings = await this.dlpEngine.scanForCredentials(data);
    findings.push(...credentialFindings);

    // DLP 정책 평가
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

  // 안전한 데이터 파기
  async secureDestroy(
    dataId: string,
    reason: string
  ): Promise<DestructionCertificate> {
    // 데이터 위치 가져오기
    const dataLocation = await this.getDataLocation(dataId);

    // 파기 권한 검증
    const authorized = await this.verifyDestructionAuthorization(dataId);
    if (!authorized) {
      throw new Error('파기 권한이 없습니다');
    }

    // 안전한 삭제 수행
    const deletionResult = await this.performSecureDeletion(dataLocation);

    // 암호화 키 삭제
    const keyDeletion = await this.deleteAssociatedKeys(dataId);

    // 백업에서 제거
    const backupRemoval = await this.removeFromBackups(dataId);

    // 파기 인증서 생성
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

    // 파기 기록
    await this.recordDestruction(certificate);

    return certificate;
  }

  // 데이터 보존 관리
  async enforceRetention(): Promise<RetentionEnforcementResult> {
    const result: RetentionEnforcementResult = {
      scannedRecords: 0,
      expiredRecords: 0,
      archivedRecords: 0,
      deletedRecords: 0,
      errors: [],
    };

    // 보존 정책이 있는 모든 레코드 가져오기
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
            // 조치 필요 없음
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

// 데이터 분류 서비스
class DataClassifier {
  private rules: ClassificationRule[];

  // 데이터 분류
  async classify(data: any, context: DataContext): Promise<DataClassification> {
    const detectedPatterns: DetectedPattern[] = [];

    // PII 패턴 감지
    const piiPatterns = await this.detectPII(data);
    detectedPatterns.push(...piiPatterns);

    // 금융 패턴 감지
    const financialPatterns = await this.detectFinancialData(data);
    detectedPatterns.push(...financialPatterns);

    // 건강 정보 감지
    const healthPatterns = await this.detectHealthInfo(data);
    detectedPatterns.push(...healthPatterns);

    // 분류 수준 결정
    const level = this.determineClassificationLevel(detectedPatterns, context);

    // 민감한 필드 식별
    const sensitiveFields = this.identifySensitiveFields(detectedPatterns);

    return {
      level,
      categories: this.categorizePatterns(detectedPatterns),
      sensitiveFields,
      retentionPolicy: this.getRetentionPolicy(level, context),
      handlingInstructions: this.getHandlingInstructions(level),
    };
  }

  // PII 감지
  private async detectPII(data: any): Promise<DetectedPattern[]> {
    const patterns: DetectedPattern[] = [];

    const piiPatterns = {
      SSN: /\b\d{3}-\d{2}-\d{4}\b/,
      EMAIL: /\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b/,
      PHONE: /\b\d{3}[-.]?\d{3}[-.]?\d{4}\b/,
      DOB: /\b\d{4}-\d{2}-\d{2}\b/,  // ISO 날짜 형식
      PASSPORT: /\b[A-Z]{1,2}\d{6,9}\b/,
      KOR_RRN: /\b\d{6}-[1-4]\d{6}\b/,  // 한국 주민등록번호
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

  // PII 심각도 결정
  private getPIISeverity(type: string): string {
    const severities: Record<string, string> = {
      SSN: 'CRITICAL',
      KOR_RRN: 'CRITICAL',
      PASSPORT: 'HIGH',
      DOB: 'MEDIUM',
      EMAIL: 'LOW',
      PHONE: 'LOW',
    };
    return severities[type] || 'MEDIUM';
  }
}
```

---

## 7.5 위협 탐지 및 대응

### 보안 모니터링

```typescript
// 위협 탐지 서비스
class ThreatDetectionService {
  private rules: DetectionRule[];
  private mlModels: ThreatMLModels;
  private alertService: AlertService;

  constructor(config: ThreatDetectionConfig) {
    this.rules = this.loadDetectionRules(config.rulesPath);
    this.mlModels = new ThreatMLModels(config.mlConfig);
    this.alertService = new AlertService(config.alertConfig);
  }

  // 위협에 대한 이벤트 분석
  async analyzeEvent(event: SecurityEvent): Promise<ThreatAnalysis> {
    const findings: ThreatFinding[] = [];

    // 규칙 기반 탐지
    const ruleFindings = await this.applyDetectionRules(event);
    findings.push(...ruleFindings);

    // ML 기반 이상 탐지
    const mlFindings = await this.detectAnomalies(event);
    findings.push(...mlFindings);

    // 행동 분석
    const behavioralFindings = await this.analyzeBehavior(event);
    findings.push(...behavioralFindings);

    // 발견 사항 상관관계 분석
    const correlatedFindings = await this.correlateFindings(findings);

    // 전체 위협 수준 결정
    const threatLevel = this.calculateThreatLevel(correlatedFindings);

    // 대응 조치 생성
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

  // 실시간 모니터링
  async monitorActivity(): Promise<void> {
    const eventStream = await this.getSecurityEventStream();

    for await (const event of eventStream) {
      try {
        const analysis = await this.analyzeEvent(event);

        if (analysis.threatLevel >= ThreatLevel.HIGH) {
          // 즉시 경고
          await this.alertService.sendCriticalAlert({
            type: 'THREAT_DETECTED',
            analysis,
            timestamp: new Date(),
          });

          // 자동화된 대응 실행
          await this.executeAutomatedResponse(analysis.responseActions);
        } else if (analysis.requiresInvestigation) {
          // 조사 대기열에 추가
          await this.queueForInvestigation(analysis);
        }

        // 분석 저장
        await this.storeAnalysis(analysis);
      } catch (error) {
        console.error('이벤트 분석 오류:', error);
      }
    }
  }

  // 이상 탐지
  private async detectAnomalies(event: SecurityEvent): Promise<ThreatFinding[]> {
    const findings: ThreatFinding[] = [];

    // 사용자 행동 이상 탐지
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
          description: '비정상적인 사용자 행동 감지',
          confidence: anomalyScore,
          indicators: this.extractAnomalyIndicators(event, userBaseline),
        });
      }
    }

    // 접근 패턴 이상
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

  // 인시던트 대응
  async handleIncident(incident: SecurityIncident): Promise<IncidentResponse> {
    // 인시던트 기록 생성
    const incidentRecord = await this.createIncidentRecord(incident);

    // 심각도 결정
    const severity = this.assessSeverity(incident);

    // 격리 실행
    const containmentActions = await this.executeContainment(incident, severity);

    // 증거 수집
    const evidence = await this.collectEvidence(incident);

    // 이해관계자 알림
    await this.notifyStakeholders(incident, severity);

    // 대응 계획 생성
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

  // 격리 실행
  private async executeContainment(
    incident: SecurityIncident,
    severity: Severity
  ): Promise<ContainmentAction[]> {
    const actions: ContainmentAction[] = [];

    // 인시던트 유형에 따른 조치
    switch (incident.type) {
      case 'UNAUTHORIZED_ACCESS':
        // 세션 해지
        await this.revokeUserSessions(incident.userId);
        actions.push({
          type: 'SESSION_REVOKED',
          target: incident.userId,
          timestamp: new Date(),
        });

        // 심각한 경우 계정 잠금
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
        // 네트워크 접근 차단
        await this.blockNetworkAccess(incident.sourceIp);
        actions.push({
          type: 'NETWORK_BLOCKED',
          target: incident.sourceIp,
          timestamp: new Date(),
        });
        break;

      case 'MALWARE_DETECTED':
        // 영향받은 시스템 격리
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

## 7.6 규정 준수 프레임워크

### 규제 준수

```typescript
// 규정 준수 관리 서비스
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

  // 프레임워크 로드
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
        case 'PIPA':  // 한국 개인정보보호법
          this.frameworks.set('PIPA', this.getPIPAFramework());
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

  // 프레임워크에 대한 규정 준수 평가
  async assessCompliance(
    frameworkId: string
  ): Promise<ComplianceAssessment> {
    const framework = this.frameworks.get(frameworkId);
    if (!framework) {
      throw new Error(`프레임워크를 찾을 수 없음: ${frameworkId}`);
    }

    const controlResults: ControlAssessmentResult[] = [];

    for (const control of framework.controls) {
      const result = await this.assessmentEngine.assessControl(control);
      controlResults.push(result);
    }

    // 전체 준수 점수 계산
    const score = this.calculateComplianceScore(controlResults);

    // 갭 식별
    const gaps = controlResults.filter(r => r.status !== 'COMPLIANT');

    // 개선 계획 생성
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

  // SOC 2 프레임워크
  private getSOC2Framework(): ComplianceFramework {
    return {
      name: 'SOC 2 Type II',
      version: '2017',
      controls: [
        // 보안
        {
          id: 'CC1.1',
          category: 'SECURITY',
          title: '통제 환경',
          description: '조직이 무결성 및 윤리적 가치에 대한 약속을 보여줍니다',
          testProcedures: ['행동 강령 검토', '직원 인터뷰'],
          evidenceRequired: ['행동 강령 문서', '교육 기록'],
        },
        {
          id: 'CC6.1',
          category: 'SECURITY',
          title: '논리적 및 물리적 접근 제어',
          description: '조직이 논리적 접근 보안 소프트웨어를 구현합니다',
          testProcedures: ['접근 제어 정책 검토', '접근 제어 테스트'],
          evidenceRequired: ['접근 제어 정책', '접근 로그', '사용자 프로비저닝 기록'],
        },
        // 가용성
        {
          id: 'A1.1',
          category: 'AVAILABILITY',
          title: '시스템 가용성',
          description: '조직이 시스템 가용성 약속을 유지합니다',
          testProcedures: ['SLA 검토', '가동 시간 메트릭 분석'],
          evidenceRequired: ['SLA 문서', '가동 시간 보고서', '인시던트 기록'],
        },
        // 기밀성
        {
          id: 'C1.1',
          category: 'CONFIDENTIALITY',
          title: '기밀 정보 보호',
          description: '조직이 기밀 정보를 식별하고 유지합니다',
          testProcedures: ['데이터 분류 검토', '암호화 테스트'],
          evidenceRequired: ['데이터 분류 정책', '암호화 감사'],
        },
        // 처리 무결성
        {
          id: 'PI1.1',
          category: 'PROCESSING_INTEGRITY',
          title: '처리 정확성',
          description: '조직이 품질 보증 절차를 사용합니다',
          testProcedures: ['QA 절차 검토', '데이터 무결성 테스트'],
          evidenceRequired: ['QA 문서', '무결성 검사 로그'],
        },
        // 개인정보보호
        {
          id: 'P1.1',
          category: 'PRIVACY',
          title: '개인정보보호 고지',
          description: '조직이 개인정보보호 관행에 대한 고지를 제공합니다',
          testProcedures: ['개인정보보호 정책 검토', '공개 확인'],
          evidenceRequired: ['개인정보보호 정책', '동의 기록'],
        },
      ],
      assessmentSchedule: 'ANNUAL',
    };
  }

  // 한국 개인정보보호법(PIPA) 프레임워크
  private getPIPAFramework(): ComplianceFramework {
    return {
      name: '개인정보 보호법',
      version: '2023',
      controls: [
        {
          id: 'PIPA-1',
          category: 'CONSENT',
          title: '개인정보 수집 동의',
          description: '정보주체로부터 적법한 동의를 획득합니다',
          testProcedures: ['동의 프로세스 검토', '동의서 양식 검토'],
          evidenceRequired: ['동의 기록', '동의서 양식'],
        },
        {
          id: 'PIPA-2',
          category: 'DATA_MINIMIZATION',
          title: '최소 수집 원칙',
          description: '목적에 필요한 최소한의 개인정보만 수집합니다',
          testProcedures: ['수집 항목 검토', '필요성 평가'],
          evidenceRequired: ['수집 항목 목록', '필요성 근거 문서'],
        },
        {
          id: 'PIPA-3',
          category: 'SECURITY',
          title: '안전성 확보 조치',
          description: '기술적, 관리적 보호조치를 구현합니다',
          testProcedures: ['보안 조치 검토', '암호화 확인'],
          evidenceRequired: ['보안 정책', '암호화 기록'],
        },
        {
          id: 'PIPA-4',
          category: 'RIGHTS',
          title: '정보주체 권리 보장',
          description: '열람, 정정, 삭제 등 정보주체 권리를 보장합니다',
          testProcedures: ['권리 행사 프로세스 검토', '응답 시간 확인'],
          evidenceRequired: ['권리 행사 기록', '처리 내역'],
        },
        {
          id: 'PIPA-5',
          category: 'TRANSFER',
          title: '제3자 제공 및 위탁',
          description: '개인정보 제공 및 위탁 시 적법한 절차를 따릅니다',
          testProcedures: ['제공 동의 확인', '위탁 계약 검토'],
          evidenceRequired: ['제공 동의서', '위탁 계약서'],
        },
      ],
      assessmentSchedule: 'ANNUAL',
    };
  }

  // 규정 준수 보고서 생성
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

  // GDPR 특정 규정 준수
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

  // 준수 점수 계산
  private calculateComplianceScore(
    results: ControlAssessmentResult[]
  ): number {
    const compliant = results.filter(r => r.status === 'COMPLIANT').length;
    return compliant / results.length;
  }
}
```

---

## 장 요약

이 장에서는 냉동 보존 자산 관리를 위한 포괄적인 보안 프레임워크를 수립했습니다:

1. **보안 아키텍처**: 시간적 고려사항을 포함한 다중 계층 방어
2. **암호화 보안**: 암호화, 키 관리, 양자 내성 준비
3. **접근 제어**: ID 관리, 권한 부여, MFA
4. **데이터 보호**: 분류, 암호화, DLP, 보존
5. **위협 탐지**: 실시간 모니터링 및 인시던트 대응
6. **규정 준수**: SOC 2, GDPR, PIPA, HIPAA 및 금융 규제 프레임워크

보안 조치는 권한 있는 당사자의 접근성을 유지하면서 확장된 시간 범위에 걸쳐 자산을 보호하도록 설계되었습니다.

---

*다음 장: 구현 - 개발 지침 및 배포 전략*
