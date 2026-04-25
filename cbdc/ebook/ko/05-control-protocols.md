# 제5장: CBDC 암호화 프로토콜 및 보안 아키텍처

## 중앙은행 디지털 화폐 시스템을 위한 고급 보안 프레임워크

### 5.1 암호화 기반

CBDC 시스템의 보안은 견고한 암호화 기반 위에 구축됩니다. WIA-CBDC 표준은 포스트 양자 대비를 갖춘 심층 방어 보안을 구현합니다.

```typescript
// 암호화 아키텍처 개요
interface CBDCCryptographyArchitecture {
  version: '1.0.0';

  algorithms: {
    // 현재 운영 알고리즘
    production: {
      keyExchange: 'X25519' | 'P-384';
      digitalSignature: 'Ed25519' | 'ECDSA-P384';
      encryption: 'AES-256-GCM' | 'ChaCha20-Poly1305';
      hashing: 'SHA-3-256' | 'BLAKE3';
      kdf: 'HKDF-SHA256' | 'Argon2id';
    };

    // 포스트 양자 알고리즘 (하이브리드 배포)
    postQuantum: {
      keyEncapsulation: 'ML-KEM-768' | 'ML-KEM-1024';
      digitalSignature: 'ML-DSA-65' | 'SLH-DSA-SHA2-128f';
      hybrid: boolean;  // 클래식 + PQ 모두 사용
    };
  };

  keyManagement: {
    hierarchy: KeyHierarchy;      // 키 계층 구조
    hsm: HSMRequirements;         // HSM 요구사항
    rotation: KeyRotationPolicy;  // 키 교체 정책
    recovery: KeyRecoveryMechanism; // 키 복구 메커니즘
  };

  protocols: {
    tokenSigning: TokenSigningProtocol;        // 토큰 서명
    transactionAuth: TransactionAuthProtocol;  // 거래 인증
    channelSecurity: ChannelSecurityProtocol;  // 채널 보안
    offlineAuth: OfflineAuthProtocol;          // 오프라인 인증
  };
}

// 키 계층 구조 정의
interface KeyHierarchy {
  levels: {
    level0_root: {
      name: '루트 키';
      algorithm: 'ML-DSA-87';  // 포스트 양자
      storage: '오프라인 HSM, 에어갭';
      usage: 'L1 키 서명만';
      lifetime: '20년';
      recovery: 'Shamir 비밀 공유 (5-of-9)';
    };

    level1_issuing: {
      name: '발행 기관 키';
      algorithm: 'ML-DSA-65';
      storage: '온라인 HSM 클러스터';
      usage: '토큰 배치 서명, L2 키';
      lifetime: '5년';
      rotation: '연례 세레모니';
    };

    level2_operational: {
      name: '운영 키';
      algorithm: 'Ed25519 + ML-DSA-44';
      storage: '자동 교체 HSM';
      usage: '거래 서명, API 인증';
      lifetime: '1년';
      rotation: '분기별 자동';
    };

    level3_session: {
      name: '세션/임시 키';
      algorithm: 'X25519 + ML-KEM-768';
      storage: '메모리만';
      usage: '채널 암호화';
      lifetime: '최대 24시간';
      rotation: '세션당';
    };
  };
}
```

### 5.2 토큰 암호화

```typescript
// 토큰 서명 및 검증
class TokenCryptographyService {
  private hsmClient: HSMClient;
  private keyStore: KeyStore;

  constructor(config: CryptoConfig) {
    this.hsmClient = new HSMClient(config.hsmEndpoint);
    this.keyStore = new KeyStore(config.keyStoreConfig);
  }

  async signToken(token: CBDCToken): Promise<SignedToken> {
    // 활성 발행 키 가져오기
    const issuingKey = await this.keyStore.getActiveKey('ISSUING');

    // 정규 토큰 표현 생성
    const canonicalPayload = this.canonicalize(token);

    // 페이로드 해시
    const payloadHash = await this.hash(canonicalPayload);

    // HSM으로 서명 (하이브리드: 클래식 + 포스트퀀텀)
    const classicalSig = await this.hsmClient.sign(
      payloadHash,
      issuingKey.classicalKeyId,
      'Ed25519'
    );

    const pqSig = await this.hsmClient.sign(
      payloadHash,
      issuingKey.pqKeyId,
      'ML-DSA-65'
    );

    // 서명 결합 (하이브리드 서명)
    const hybridSignature = this.combineSignatures(classicalSig, pqSig);

    return {
      ...token,
      cryptography: {
        signatureAlgorithm: 'HYBRID-Ed25519-ML-DSA-65',
        issuerSignature: hybridSignature,
        keyId: issuingKey.id,
        signedAt: new Date().toISOString()
      }
    };
  }

  async verifyToken(token: SignedToken): Promise<VerificationResult> {
    // 서명 키 가져오기 (과거 키일 수 있음)
    const signingKey = await this.keyStore.getKey(token.cryptography.keyId);

    if (!signingKey) {
      return { valid: false, reason: '알 수 없는 서명 키' };
    }

    if (signingKey.status === 'REVOKED') {
      return { valid: false, reason: '서명 키 폐지됨' };
    }

    // 정규 페이로드 재구성
    const canonicalPayload = this.canonicalize(token);
    const payloadHash = await this.hash(canonicalPayload);

    // 하이브리드 서명 분리
    const { classicalSig, pqSig } = this.splitSignatures(
      token.cryptography.issuerSignature
    );

    // 두 서명 모두 검증
    const classicalValid = await this.verifySignature(
      payloadHash,
      classicalSig,
      signingKey.classicalPublicKey,
      'Ed25519'
    );

    const pqValid = await this.verifySignature(
      payloadHash,
      pqSig,
      signingKey.pqPublicKey,
      'ML-DSA-65'
    );

    // 하이브리드에서는 둘 다 유효해야 함
    if (!classicalValid || !pqValid) {
      return {
        valid: false,
        reason: `서명 검증 실패: classical=${classicalValid}, pq=${pqValid}`
      };
    }

    return {
      valid: true,
      signingKeyId: signingKey.id,
      signedAt: token.cryptography.signedAt
    };
  }

  private canonicalize(token: CBDCToken): Buffer {
    // 서명을 위한 결정론적 직렬화
    const orderedFields = {
      tokenId: token.tokenId,
      serialNumber: token.serialNumber,
      denomination: {
        value: token.denomination.value,
        currency: token.denomination.currency
      },
      tokenType: token.tokenType,
      ownership: {
        currentHolder: token.ownership.currentHolder
      },
      state: {
        status: token.state.status,
        issuedAt: token.state.issuedAt
      },
      conditions: token.conditions?.sort((a, b) =>
        a.conditionId.localeCompare(b.conditionId)
      )
    };

    return Buffer.from(JSON.stringify(orderedFields), 'utf-8');
  }
}

// 소유권 증명 생성
class OwnershipProofService {
  async generateOwnershipProof(
    token: CBDCToken,
    ownerPrivateKey: PrivateKey,
    challenge?: Buffer
  ): Promise<OwnershipProof> {
    // 증명 챌린지 생성 (또는 제공된 것 사용)
    const proofChallenge = challenge || crypto.randomBytes(32);

    // 증명 메시지 구성
    const proofMessage = Buffer.concat([
      Buffer.from(token.tokenId),
      proofChallenge,
      Buffer.from(Date.now().toString())
    ]);

    // 소유자 키로 서명
    const signature = await this.sign(proofMessage, ownerPrivateKey);

    return {
      tokenId: token.tokenId,
      challenge: proofChallenge.toString('base64'),
      signature: signature.toString('base64'),
      timestamp: Date.now(),
      algorithm: ownerPrivateKey.algorithm
    };
  }

  async verifyOwnershipProof(
    proof: OwnershipProof,
    expectedOwner: string,
    maxAge: number = 300000  // 5분
  ): Promise<boolean> {
    // 타임스탬프 신선도 확인
    if (Date.now() - proof.timestamp > maxAge) {
      return false;
    }

    // 소유자 공개 키 가져오기
    const ownerPublicKey = await this.keyStore.getPublicKey(expectedOwner);

    // 증명 메시지 재구성
    const proofMessage = Buffer.concat([
      Buffer.from(proof.tokenId),
      Buffer.from(proof.challenge, 'base64'),
      Buffer.from(proof.timestamp.toString())
    ]);

    // 서명 검증
    return this.verifySignature(
      proofMessage,
      Buffer.from(proof.signature, 'base64'),
      ownerPublicKey,
      proof.algorithm
    );
  }
}
```

### 5.3 거래 보안

```typescript
// 거래 서명 프로토콜
class TransactionSecurityService {
  private nonceStore: NonceStore;
  private rateLimiter: RateLimiter;

  async signTransaction(
    transaction: CBDCTransaction,
    signerKey: PrivateKey,
    additionalSigners?: PrivateKey[]
  ): Promise<SignedTransaction> {
    // 거래 논스 생성
    const nonce = await this.generateNonce(transaction.parties.sender!.walletId);

    // 재전송 방지 데이터 추가
    const protectedTransaction = {
      ...transaction,
      security: {
        nonce,
        timestamp: Date.now(),
        sequenceNumber: await this.getNextSequence(
          transaction.parties.sender!.walletId
        )
      }
    };

    // 거래 해시 생성
    const txHash = await this.hashTransaction(protectedTransaction);

    // 기본 서명
    const primarySignature = await this.signWithKey(txHash, signerKey);

    const signatures: TransactionSignature[] = [{
      signerId: signerKey.id,
      signerRole: 'PRIMARY',
      algorithm: signerKey.algorithm,
      signature: primarySignature,
      timestamp: new Date().toISOString(),
      publicKey: signerKey.publicKey
    }];

    // 다중 서명을 위한 추가 서명
    if (additionalSigners) {
      for (const signer of additionalSigners) {
        const sig = await this.signWithKey(txHash, signer);
        signatures.push({
          signerId: signer.id,
          signerRole: 'CO_SIGNER',
          algorithm: signer.algorithm,
          signature: sig,
          timestamp: new Date().toISOString(),
          publicKey: signer.publicKey
        });
      }
    }

    return {
      ...protectedTransaction,
      technical: {
        ...protectedTransaction.technical,
        signatures
      }
    };
  }

  async verifyTransaction(
    transaction: SignedTransaction
  ): Promise<TransactionVerificationResult> {
    const errors: string[] = [];

    // 1. 재전송 방지 검증
    const replayCheck = await this.checkReplay(transaction);
    if (!replayCheck.valid) {
      errors.push(replayCheck.reason);
    }

    // 2. 타임스탬프 신선도 검증
    const ageMs = Date.now() - transaction.security.timestamp;
    if (ageMs > this.config.maxTransactionAge) {
      errors.push('거래가 너무 오래됨');
    }

    // 3. 모든 서명 검증
    const txHash = await this.hashTransaction(transaction);

    for (const sig of transaction.technical.signatures) {
      const sigValid = await this.verifySignature(
        txHash,
        Buffer.from(sig.signature, 'base64'),
        sig.publicKey,
        sig.algorithm
      );

      if (!sigValid) {
        errors.push(`${sig.signerId}의 유효하지 않은 서명`);
      }
    }

    // 4. 다중 서명 임계값 확인
    const requiredSigs = await this.getRequiredSignatures(
      transaction.parties.sender!.walletId,
      transaction.amount
    );

    if (transaction.technical.signatures.length < requiredSigs) {
      errors.push(`서명 부족: ${transaction.technical.signatures.length}/${requiredSigs}`);
    }

    // 5. 논스 사용 표시 (모든 검사 통과 시에만)
    if (errors.length === 0) {
      await this.markNonceUsed(
        transaction.parties.sender!.walletId,
        transaction.security.nonce
      );
    }

    return {
      valid: errors.length === 0,
      errors,
      verifiedAt: new Date().toISOString()
    };
  }

  private async checkReplay(
    transaction: SignedTransaction
  ): Promise<{ valid: boolean; reason?: string }> {
    const walletId = transaction.parties.sender!.walletId;

    // 논스가 사용되지 않았는지 확인
    const nonceUsed = await this.nonceStore.isUsed(
      walletId,
      transaction.security.nonce
    );

    if (nonceUsed) {
      return { valid: false, reason: '논스가 이미 사용됨 (재전송 감지)' };
    }

    // 시퀀스 번호 확인
    const expectedSequence = await this.getNextSequence(walletId);
    if (transaction.security.sequenceNumber < expectedSequence) {
      return { valid: false, reason: '시퀀스 번호가 너무 낮음 (재전송 감지)' };
    }

    return { valid: true };
  }
}

// 이중 지출 방지
class DoubleSpendPrevention {
  private tokenLockManager: TokenLockManager;
  private consensusLayer: ConsensusLayer;

  async validateAndLockTokens(
    transaction: CBDCTransaction
  ): Promise<TokenLockResult> {
    const tokenIds = this.extractTokenIds(transaction);

    // 분산 잠금 획득
    const locks = await this.tokenLockManager.acquireLocks(
      tokenIds,
      transaction.transactionId,
      this.config.lockTimeout
    );

    if (!locks.success) {
      return {
        success: false,
        reason: '토큰 잠금을 획득할 수 없음',
        conflictingTransaction: locks.conflictingTxId
      };
    }

    try {
      // 토큰 상태 검증
      for (const tokenId of tokenIds) {
        const token = await this.tokenStore.get(tokenId);

        if (!token) {
          throw new Error(`토큰 ${tokenId}을(를) 찾을 수 없음`);
        }

        if (token.state.status !== 'ACTIVE') {
          throw new Error(`토큰 ${tokenId}이(가) 활성 상태가 아님: ${token.state.status}`);
        }

        // 소유권 검증
        if (token.ownership.currentHolder !== transaction.parties.sender!.walletId) {
          throw new Error(`발신자가 토큰 ${tokenId}의 소유자가 아님`);
        }
      }

      // 순서화를 위해 합의에 제출
      const consensusResult = await this.consensusLayer.submit(transaction);

      if (!consensusResult.accepted) {
        throw new Error('거래가 합의에 의해 거부됨');
      }

      return {
        success: true,
        lockIds: locks.lockIds,
        consensusTimestamp: consensusResult.timestamp,
        sequenceNumber: consensusResult.sequence
      };
    } catch (error) {
      // 실패 시 잠금 해제
      await this.tokenLockManager.releaseLocks(locks.lockIds);
      throw error;
    }
  }
}
```

### 5.4 오프라인 보안 프로토콜

```typescript
// 오프라인 거래 보안
class OfflineSecurityService {
  private secureElement: SecureElement;
  private offlineTokenManager: OfflineTokenManager;

  async generateOfflineTokens(
    wallet: CBDCWallet,
    amount: MonetaryAmount,
    duration: number
  ): Promise<OfflineToken[]> {
    // 지갑에 충분한 잔액 확인
    if (wallet.balance.available < amount) {
      throw new Error('잔액 부족');
    }

    // 오프라인 한도 확인
    const limits = await this.getOfflineLimits(wallet);
    if (amount > limits.maxOfflineAmount) {
      throw new Error('금액이 오프라인 한도 초과');
    }

    // 온라인 지갑에서 자금 잠금
    await this.lockFunds(wallet.walletId, amount);

    // 보안 요소에서 오프라인 토큰 생성
    const tokens = await this.secureElement.generateOfflineTokens({
      walletId: wallet.walletId,
      totalAmount: amount,
      validityPeriod: duration,
      maxTransactions: limits.maxOfflineTransactions,
      deviceCertificate: await this.getDeviceCertificate()
    });

    // 각 토큰은 암호화적으로 이 기기에 바인딩
    for (const token of tokens) {
      token.deviceBinding = await this.createDeviceBinding(token);
      token.monotunicCounterStart = await this.secureElement.getCounter();
    }

    return tokens;
  }

  async executeOfflineTransfer(
    senderSecureElement: SecureElement,
    receiverSecureElement: SecureElement,
    amount: MonetaryAmount
  ): Promise<OfflineTransferResult> {
    // 1단계: NFC/블루투스를 통한 상호 인증
    const sessionKey = await this.establishSecureChannel(
      senderSecureElement,
      receiverSecureElement
    );

    // 2단계: 발신자가 SE에서 오프라인 전송 생성
    const transfer = await senderSecureElement.createOfflineTransfer({
      amount,
      recipientPublicKey: receiverSecureElement.publicKey,
      timestamp: Date.now(),
      counter: await senderSecureElement.incrementCounter()
    });

    // 3단계: 하드웨어 바인딩 키로 서명
    const signature = await senderSecureElement.signOfflineTransfer(transfer);

    // 4단계: 수신자를 위해 전송 암호화
    const encryptedTransfer = await this.encryptWithSessionKey(
      { transfer, signature },
      sessionKey
    );

    // 5단계: 수신자가 검증 및 저장
    const decrypted = await receiverSecureElement.decrypt(encryptedTransfer, sessionKey);

    const validationResult = await receiverSecureElement.validateOfflineTransfer(
      decrypted.transfer,
      decrypted.signature
    );

    if (!validationResult.valid) {
      throw new Error(`오프라인 전송 검증 실패: ${validationResult.reason}`);
    }

    // 6단계: 수신자가 대기 중인 들어오는 전송 저장
    await receiverSecureElement.storePendingTransfer(decrypted.transfer);

    // 7단계: 발신자가 토큰을 사용됨으로 표시
    await senderSecureElement.markTokensSpent(transfer.tokenIds);

    return {
      success: true,
      transferId: transfer.transferId,
      senderNewBalance: await senderSecureElement.getOfflineBalance(),
      receiverNewBalance: await receiverSecureElement.getOfflineBalance()
    };
  }

  async syncOfflineTransactions(
    secureElement: SecureElement,
    wallet: CBDCWallet
  ): Promise<SyncResult> {
    // SE에서 대기 중인 모든 오프라인 거래 가져오기
    const pendingTransfers = await secureElement.getPendingTransfers();
    const spentTokens = await secureElement.getSpentTokens();

    const results: TransactionSyncResult[] = [];

    // 나가는 (사용된) 토큰 동기화
    for (const spent of spentTokens) {
      try {
        // 중앙 원장에 제출
        const result = await this.submitOfflineSpend(spent, wallet);
        results.push({ type: 'OUTGOING', success: true, id: spent.tokenId });

        // SE에서 지우기
        await secureElement.clearSpentToken(spent.tokenId);
      } catch (error) {
        results.push({
          type: 'OUTGOING',
          success: false,
          id: spent.tokenId,
          error: error.message
        });
      }
    }

    // 들어오는 전송 동기화
    for (const transfer of pendingTransfers) {
      try {
        // 중앙 원장에서 전송 검증
        const verified = await this.verifyOfflineTransfer(transfer);

        if (verified) {
          // 온라인 지갑에 입금
          await this.creditOfflineReceived(wallet, transfer);

          // SE에서 지우기
          await secureElement.clearPendingTransfer(transfer.transferId);

          results.push({ type: 'INCOMING', success: true, id: transfer.transferId });
        }
      } catch (error) {
        results.push({
          type: 'INCOMING',
          success: false,
          id: transfer.transferId,
          error: error.message
        });
      }
    }

    return {
      totalSynced: results.filter(r => r.success).length,
      failed: results.filter(r => !r.success).length,
      results
    };
  }
}
```

### 5.5 HSM 통합

```typescript
// 하드웨어 보안 모듈 통합
interface HSMConfiguration {
  provider: 'Thales Luna' | 'AWS CloudHSM' | 'Azure Dedicated HSM' | 'Utimaco';
  cluster: {
    nodes: HSMNode[];
    quorum: number;
    failover: 'AUTOMATIC' | 'MANUAL';
  };
  keyPolicy: {
    extractable: false;
    exportable: false;
    wrapKey: string;  // 백업용
  };
}

class HSMService {
  private hsmCluster: HSMCluster;
  private keyHandles: Map<string, HSMKeyHandle>;

  constructor(config: HSMConfiguration) {
    this.hsmCluster = new HSMCluster(config);
    this.keyHandles = new Map();
  }

  async generateKey(
    keySpec: KeySpecification
  ): Promise<HSMKeyInfo> {
    // HSM 내부에서 키 생성 (절대 내보내지 않음)
    const keyHandle = await this.hsmCluster.generateKey({
      algorithm: keySpec.algorithm,
      keySize: keySpec.size,
      keyUsage: keySpec.usage,
      extractable: false
    });

    // 핸들 저장
    this.keyHandles.set(keySpec.keyId, keyHandle);

    // 공개 키 가져오기 (내보내기 가능)
    const publicKey = await this.hsmCluster.exportPublicKey(keyHandle);

    return {
      keyId: keySpec.keyId,
      algorithm: keySpec.algorithm,
      publicKey,
      createdAt: new Date().toISOString(),
      status: 'ACTIVE'
    };
  }

  async sign(
    data: Buffer,
    keyId: string,
    algorithm: string
  ): Promise<Buffer> {
    const keyHandle = this.keyHandles.get(keyId);
    if (!keyHandle) {
      throw new Error(`키를 찾을 수 없음: ${keyId}`);
    }

    // HSM 내부에서 서명
    const signature = await this.hsmCluster.sign({
      keyHandle,
      data,
      algorithm,
      mechanism: this.getMechanism(algorithm)
    });

    return signature;
  }

  async multiPartySign(
    data: Buffer,
    keyId: string,
    operators: Operator[]
  ): Promise<MultiPartySignature> {
    // 민감한 작업에 대해 다수의 운영자 승인 필요
    const approvals: OperatorApproval[] = [];

    for (const operator of operators) {
      const approval = await this.requestOperatorApproval(
        operator,
        data,
        keyId
      );

      approvals.push(approval);
    }

    // 정족수 확인
    const requiredApprovals = await this.getRequiredApprovals(keyId);
    if (approvals.length < requiredApprovals) {
      throw new Error('운영자 승인 부족');
    }

    // 정족수로 HSM 작업 수행
    const signature = await this.hsmCluster.multiPartySign({
      keyId,
      data,
      approvals
    });

    return {
      signature,
      approvals,
      timestamp: new Date().toISOString()
    };
  }
}
```

### 5.6 요약

WIA-CBDC 보안 아키텍처가 제공하는 것:

1. **하이브리드 암호화**: 미래 대비를 위한 클래식 + 포스트퀀텀
2. **HSM 기반 키 관리**: 모든 중요 키의 하드웨어 보호
3. **다중 서명 지원**: 고가치 작업을 위한 임계값 서명
4. **영지식 증명**: 프라이버시 보존 거래
5. **오프라인 보안**: 보안 요소 기반 오프라인 결제
6. **재전송 방지 보호**: 논스, 타임스탬프, 시퀀스 번호

---

**WIA-CBDC 보안 아키텍처**
**버전**: 1.0.0
**최종 업데이트**: 2025

© 2025 WIA (World Interoperability Alliance)
