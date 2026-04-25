# Chapter 5: CBDC Cryptographic Protocols and Security Architecture

## Advanced Security Framework for Central Bank Digital Currency Systems

### 5.1 Cryptographic Foundation

The security of CBDC systems rests upon a robust cryptographic foundation. The WIA-CBDC standard implements defense-in-depth security with post-quantum readiness.

```typescript
// Cryptographic Architecture Overview
interface CBDCCryptographyArchitecture {
  version: '1.0.0';

  algorithms: {
    // Current production algorithms
    production: {
      keyExchange: 'X25519' | 'P-384';
      digitalSignature: 'Ed25519' | 'ECDSA-P384';
      encryption: 'AES-256-GCM' | 'ChaCha20-Poly1305';
      hashing: 'SHA-3-256' | 'BLAKE3';
      kdf: 'HKDF-SHA256' | 'Argon2id';
    };

    // Post-quantum algorithms (hybrid deployment)
    postQuantum: {
      keyEncapsulation: 'ML-KEM-768' | 'ML-KEM-1024';
      digitalSignature: 'ML-DSA-65' | 'SLH-DSA-SHA2-128f';
      hybrid: boolean;  // Use both classical + PQ
    };
  };

  keyManagement: {
    hierarchy: KeyHierarchy;
    hsm: HSMRequirements;
    rotation: KeyRotationPolicy;
    recovery: KeyRecoveryMechanism;
  };

  protocols: {
    tokenSigning: TokenSigningProtocol;
    transactionAuth: TransactionAuthProtocol;
    channelSecurity: ChannelSecurityProtocol;
    offlineAuth: OfflineAuthProtocol;
  };
}

// Key Hierarchy Definition
interface KeyHierarchy {
  levels: {
    level0_root: {
      name: 'Root Key';
      algorithm: 'ML-DSA-87';  // Post-quantum
      storage: 'Offline HSM, air-gapped';
      usage: 'Sign L1 keys only';
      lifetime: '20 years';
      recovery: 'Shamir Secret Sharing (5-of-9)';
    };

    level1_issuing: {
      name: 'Issuing Authority Key';
      algorithm: 'ML-DSA-65';
      storage: 'Online HSM cluster';
      usage: 'Sign token batches, L2 keys';
      lifetime: '5 years';
      rotation: 'Annual ceremony';
    };

    level2_operational: {
      name: 'Operational Keys';
      algorithm: 'Ed25519 + ML-DSA-44';
      storage: 'HSM with auto-rotation';
      usage: 'Transaction signing, API auth';
      lifetime: '1 year';
      rotation: 'Quarterly automated';
    };

    level3_session: {
      name: 'Session/Ephemeral Keys';
      algorithm: 'X25519 + ML-KEM-768';
      storage: 'Memory only';
      usage: 'Channel encryption';
      lifetime: '24 hours max';
      rotation: 'Per session';
    };
  };
}
```

### 5.2 Token Cryptography

```typescript
// Token Signing and Verification
interface TokenCryptography {
  // Token structure with cryptographic binding
  tokenStructure: {
    header: {
      version: number;
      algorithm: string;
      keyId: string;
      timestamp: number;
    };
    payload: {
      tokenId: string;
      denomination: MonetaryAmount;
      issuanceInfo: IssuanceInfo;
      conditions: TokenCondition[];
    };
    signatures: {
      issuerSignature: string;
      ownershipProof?: string;
    };
  };
}

class TokenCryptographyService {
  private hsmClient: HSMClient;
  private keyStore: KeyStore;

  constructor(config: CryptoConfig) {
    this.hsmClient = new HSMClient(config.hsmEndpoint);
    this.keyStore = new KeyStore(config.keyStoreConfig);
  }

  async signToken(token: CBDCToken): Promise<SignedToken> {
    // Get active issuing key
    const issuingKey = await this.keyStore.getActiveKey('ISSUING');

    // Create canonical token representation
    const canonicalPayload = this.canonicalize(token);

    // Hash the payload
    const payloadHash = await this.hash(canonicalPayload);

    // Sign using HSM (hybrid: classical + post-quantum)
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

    // Combine signatures (hybrid signature)
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
    // Get signing key (may be historical)
    const signingKey = await this.keyStore.getKey(token.cryptography.keyId);

    if (!signingKey) {
      return { valid: false, reason: 'Unknown signing key' };
    }

    if (signingKey.status === 'REVOKED') {
      return { valid: false, reason: 'Signing key revoked' };
    }

    // Reconstruct canonical payload
    const canonicalPayload = this.canonicalize(token);
    const payloadHash = await this.hash(canonicalPayload);

    // Split hybrid signature
    const { classicalSig, pqSig } = this.splitSignatures(
      token.cryptography.issuerSignature
    );

    // Verify both signatures
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

    // Both must be valid for hybrid
    if (!classicalValid || !pqValid) {
      return {
        valid: false,
        reason: `Signature verification failed: classical=${classicalValid}, pq=${pqValid}`
      };
    }

    return {
      valid: true,
      signingKeyId: signingKey.id,
      signedAt: token.cryptography.signedAt
    };
  }

  private canonicalize(token: CBDCToken): Buffer {
    // Deterministic serialization for signing
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

  private combineSignatures(classical: Buffer, pq: Buffer): string {
    // Format: length(2 bytes) + classical + pq
    const combined = Buffer.alloc(2 + classical.length + pq.length);
    combined.writeUInt16BE(classical.length, 0);
    classical.copy(combined, 2);
    pq.copy(combined, 2 + classical.length);
    return combined.toString('base64');
  }
}

// Ownership Proof Generation
class OwnershipProofService {
  async generateOwnershipProof(
    token: CBDCToken,
    ownerPrivateKey: PrivateKey,
    challenge?: Buffer
  ): Promise<OwnershipProof> {
    // Create proof challenge (or use provided)
    const proofChallenge = challenge || crypto.randomBytes(32);

    // Construct proof message
    const proofMessage = Buffer.concat([
      Buffer.from(token.tokenId),
      proofChallenge,
      Buffer.from(Date.now().toString())
    ]);

    // Sign with owner's key
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
    maxAge: number = 300000  // 5 minutes
  ): Promise<boolean> {
    // Check timestamp freshness
    if (Date.now() - proof.timestamp > maxAge) {
      return false;
    }

    // Get owner's public key
    const ownerPublicKey = await this.keyStore.getPublicKey(expectedOwner);

    // Reconstruct proof message
    const proofMessage = Buffer.concat([
      Buffer.from(proof.tokenId),
      Buffer.from(proof.challenge, 'base64'),
      Buffer.from(proof.timestamp.toString())
    ]);

    // Verify signature
    return this.verifySignature(
      proofMessage,
      Buffer.from(proof.signature, 'base64'),
      ownerPublicKey,
      proof.algorithm
    );
  }
}
```

### 5.3 Transaction Security

```typescript
// Transaction Signing Protocol
interface TransactionSecurityProtocol {
  // Multi-signature requirements
  multiSig: {
    enabled: boolean;
    threshold: number;
    signers: string[];
  };

  // Time-lock mechanisms
  timeLock: {
    minimumDelay?: number;
    maximumDelay?: number;
  };

  // Anti-replay protection
  antiReplay: {
    nonce: boolean;
    timestamp: boolean;
    sequenceNumber: boolean;
  };
}

class TransactionSecurityService {
  private nonceStore: NonceStore;
  private rateLimiter: RateLimiter;

  async signTransaction(
    transaction: CBDCTransaction,
    signerKey: PrivateKey,
    additionalSigners?: PrivateKey[]
  ): Promise<SignedTransaction> {
    // Generate transaction nonce
    const nonce = await this.generateNonce(transaction.parties.sender!.walletId);

    // Add anti-replay data
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

    // Create transaction hash
    const txHash = await this.hashTransaction(protectedTransaction);

    // Primary signature
    const primarySignature = await this.signWithKey(txHash, signerKey);

    const signatures: TransactionSignature[] = [{
      signerId: signerKey.id,
      signerRole: 'PRIMARY',
      algorithm: signerKey.algorithm,
      signature: primarySignature,
      timestamp: new Date().toISOString(),
      publicKey: signerKey.publicKey
    }];

    // Additional signatures for multi-sig
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

    // 1. Verify anti-replay
    const replayCheck = await this.checkReplay(transaction);
    if (!replayCheck.valid) {
      errors.push(replayCheck.reason);
    }

    // 2. Verify timestamp freshness
    const ageMs = Date.now() - transaction.security.timestamp;
    if (ageMs > this.config.maxTransactionAge) {
      errors.push('Transaction too old');
    }

    // 3. Verify all signatures
    const txHash = await this.hashTransaction(transaction);

    for (const sig of transaction.technical.signatures) {
      const sigValid = await this.verifySignature(
        txHash,
        Buffer.from(sig.signature, 'base64'),
        sig.publicKey,
        sig.algorithm
      );

      if (!sigValid) {
        errors.push(`Invalid signature from ${sig.signerId}`);
      }
    }

    // 4. Check multi-sig threshold
    const requiredSigs = await this.getRequiredSignatures(
      transaction.parties.sender!.walletId,
      transaction.amount
    );

    if (transaction.technical.signatures.length < requiredSigs) {
      errors.push(`Insufficient signatures: ${transaction.technical.signatures.length}/${requiredSigs}`);
    }

    // 5. Mark nonce as used (only if all checks pass)
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

  private async generateNonce(walletId: string): Promise<string> {
    // Cryptographically secure nonce
    const randomPart = crypto.randomBytes(16);
    const timestampPart = Buffer.alloc(8);
    timestampPart.writeBigInt64BE(BigInt(Date.now()), 0);

    const nonce = Buffer.concat([randomPart, timestampPart]);

    // Store for replay detection
    await this.nonceStore.store(walletId, nonce.toString('hex'));

    return nonce.toString('hex');
  }

  private async checkReplay(
    transaction: SignedTransaction
  ): Promise<{ valid: boolean; reason?: string }> {
    const walletId = transaction.parties.sender!.walletId;

    // Check nonce hasn't been used
    const nonceUsed = await this.nonceStore.isUsed(
      walletId,
      transaction.security.nonce
    );

    if (nonceUsed) {
      return { valid: false, reason: 'Nonce already used (replay detected)' };
    }

    // Check sequence number
    const expectedSequence = await this.getNextSequence(walletId);
    if (transaction.security.sequenceNumber < expectedSequence) {
      return { valid: false, reason: 'Sequence number too low (replay detected)' };
    }

    return { valid: true };
  }
}

// Double-Spend Prevention
class DoubleSpendPrevention {
  private tokenLockManager: TokenLockManager;
  private consensusLayer: ConsensusLayer;

  async validateAndLockTokens(
    transaction: CBDCTransaction
  ): Promise<TokenLockResult> {
    const tokenIds = this.extractTokenIds(transaction);

    // Acquire distributed locks
    const locks = await this.tokenLockManager.acquireLocks(
      tokenIds,
      transaction.transactionId,
      this.config.lockTimeout
    );

    if (!locks.success) {
      return {
        success: false,
        reason: 'Could not acquire token locks',
        conflictingTransaction: locks.conflictingTxId
      };
    }

    try {
      // Verify token states
      for (const tokenId of tokenIds) {
        const token = await this.tokenStore.get(tokenId);

        if (!token) {
          throw new Error(`Token ${tokenId} not found`);
        }

        if (token.state.status !== 'ACTIVE') {
          throw new Error(`Token ${tokenId} is not active: ${token.state.status}`);
        }

        // Verify ownership
        if (token.ownership.currentHolder !== transaction.parties.sender!.walletId) {
          throw new Error(`Sender does not own token ${tokenId}`);
        }
      }

      // Submit to consensus for ordering
      const consensusResult = await this.consensusLayer.submit(transaction);

      if (!consensusResult.accepted) {
        throw new Error('Transaction rejected by consensus');
      }

      return {
        success: true,
        lockIds: locks.lockIds,
        consensusTimestamp: consensusResult.timestamp,
        sequenceNumber: consensusResult.sequence
      };
    } catch (error) {
      // Release locks on failure
      await this.tokenLockManager.releaseLocks(locks.lockIds);
      throw error;
    }
  }

  async commitTokenTransfer(
    transaction: CBDCTransaction,
    locks: TokenLockResult
  ): Promise<void> {
    const tokenIds = this.extractTokenIds(transaction);

    // Atomic state update
    await this.tokenStore.batchUpdate(tokenIds, {
      'ownership.currentHolder': transaction.parties.receiver!.walletId,
      'ownership.ownershipHistory': {
        $push: {
          holder: transaction.parties.sender!.walletId,
          transferredAt: new Date().toISOString(),
          transactionId: transaction.transactionId
        }
      },
      'state.lastTransferAt': new Date().toISOString()
    });

    // Release locks after commit
    await this.tokenLockManager.releaseLocks(locks.lockIds);
  }
}
```

### 5.4 Zero-Knowledge Proofs for Privacy

```typescript
// Privacy-Preserving Transaction Verification
interface ZKProofSystem {
  // Prove transaction validity without revealing amounts
  balanceProof: {
    type: 'Bulletproofs' | 'PLONK' | 'Groth16';
    proves: [
      'input_sum >= output_sum',
      'all_amounts >= 0',
      'no_overflow'
    ];
  };

  // Prove identity attributes without revealing identity
  identityProof: {
    type: 'Anonymous Credentials' | 'ZK-SNARK';
    proves: [
      'holder_is_verified',
      'holder_meets_age_requirement',
      'holder_not_sanctioned'
    ];
  };
}

class ZeroKnowledgeProofService {
  private prover: ZKProver;
  private verifier: ZKVerifier;

  constructor(config: ZKConfig) {
    this.prover = new ZKProver(config.provingKey);
    this.verifier = new ZKVerifier(config.verificationKey);
  }

  // Generate range proof for transaction amount
  async generateRangeProof(
    amount: bigint,
    blinding: bigint
  ): Promise<RangeProof> {
    // Bulletproofs range proof: prove 0 <= amount < 2^64
    const commitment = this.pedersenCommit(amount, blinding);

    const proof = await this.prover.proveRange({
      value: amount,
      blinding,
      bitLength: 64
    });

    return {
      commitment: commitment.toString('hex'),
      proof: proof.toString('hex'),
      type: 'Bulletproofs'
    };
  }

  // Verify range proof
  async verifyRangeProof(rangeProof: RangeProof): Promise<boolean> {
    return this.verifier.verifyRange({
      commitment: Buffer.from(rangeProof.commitment, 'hex'),
      proof: Buffer.from(rangeProof.proof, 'hex')
    });
  }

  // Generate balance proof for confidential transaction
  async generateBalanceProof(
    inputs: ConfidentialInput[],
    outputs: ConfidentialOutput[]
  ): Promise<BalanceProof> {
    // Prove: sum(inputs) = sum(outputs) + fee
    // Without revealing any individual amounts

    const inputCommitments = inputs.map(i =>
      this.pedersenCommit(i.amount, i.blinding)
    );

    const outputCommitments = outputs.map(o =>
      this.pedersenCommit(o.amount, o.blinding)
    );

    // Sum of blindings must balance
    const inputBlindingSum = inputs.reduce(
      (sum, i) => sum + i.blinding,
      BigInt(0)
    );

    const outputBlindingSum = outputs.reduce(
      (sum, o) => sum + o.blinding,
      BigInt(0)
    );

    // Generate proof
    const proof = await this.prover.proveBalance({
      inputCommitments,
      outputCommitments,
      inputBlindingSum,
      outputBlindingSum
    });

    return {
      inputCommitments: inputCommitments.map(c => c.toString('hex')),
      outputCommitments: outputCommitments.map(c => c.toString('hex')),
      proof: proof.toString('hex'),
      type: 'Bulletproofs-Balance'
    };
  }

  // Anonymous credentials for identity proofs
  async generateIdentityProof(
    credential: AnonymousCredential,
    requiredAttributes: string[]
  ): Promise<IdentityProof> {
    // Prove possession of valid credential with certain attributes
    // Without revealing the credential itself

    const proofRequest = {
      credentialType: credential.type,
      revealedAttributes: [], // Don't reveal any
      predicates: requiredAttributes.map(attr => ({
        attribute: attr,
        type: 'EXISTS'
      }))
    };

    const proof = await this.prover.proveCredential(
      credential,
      proofRequest
    );

    return {
      proof: proof.toString('hex'),
      type: 'AnonymousCredential',
      predicates: proofRequest.predicates
    };
  }

  // Pedersen commitment: C = g^v * h^r
  private pedersenCommit(value: bigint, blinding: bigint): Buffer {
    const g = this.config.generatorG;
    const h = this.config.generatorH;

    // Use elliptic curve operations
    const gv = this.ecMultiply(g, value);
    const hr = this.ecMultiply(h, blinding);
    const commitment = this.ecAdd(gv, hr);

    return this.serializePoint(commitment);
  }
}

// Confidential Transaction Implementation
class ConfidentialTransactionService {
  private zkService: ZeroKnowledgeProofService;

  async createConfidentialTransaction(
    sender: Wallet,
    receiver: string,
    amount: bigint,
    fee: bigint
  ): Promise<ConfidentialTransaction> {
    // Generate random blinding factors
    const inputBlinding = this.generateBlinding();
    const outputBlinding = this.generateBlinding();
    const changeBlinding = this.generateBlinding();

    // Calculate change
    const change = sender.balance - amount - fee;

    // Create commitments
    const inputs: ConfidentialInput[] = [{
      commitment: await this.zkService.commit(sender.balance, inputBlinding),
      amount: sender.balance,
      blinding: inputBlinding
    }];

    const outputs: ConfidentialOutput[] = [
      {
        commitment: await this.zkService.commit(amount, outputBlinding),
        amount,
        blinding: outputBlinding,
        recipient: receiver
      },
      {
        commitment: await this.zkService.commit(change, changeBlinding),
        amount: change,
        blinding: changeBlinding,
        recipient: sender.address
      }
    ];

    // Generate range proofs for all outputs
    const rangeProofs = await Promise.all(
      outputs.map(o => this.zkService.generateRangeProof(o.amount, o.blinding))
    );

    // Generate balance proof
    const balanceProof = await this.zkService.generateBalanceProof(
      inputs,
      outputs
    );

    return {
      inputs: inputs.map(i => ({ commitment: i.commitment })),
      outputs: outputs.map((o, i) => ({
        commitment: o.commitment,
        recipient: o.recipient,
        rangeProof: rangeProofs[i]
      })),
      balanceProof,
      fee: {
        amount: fee,
        commitment: await this.zkService.commit(fee, BigInt(0)) // Known value
      }
    };
  }

  async verifyConfidentialTransaction(
    tx: ConfidentialTransaction
  ): Promise<boolean> {
    // Verify all range proofs
    for (const output of tx.outputs) {
      const validRange = await this.zkService.verifyRangeProof(output.rangeProof);
      if (!validRange) {
        return false;
      }
    }

    // Verify balance proof
    const validBalance = await this.zkService.verifyBalanceProof(tx.balanceProof);
    if (!validBalance) {
      return false;
    }

    return true;
  }
}
```

### 5.5 Offline Security Protocols

```typescript
// Offline Transaction Security
interface OfflineSecurityProtocol {
  // Secure element requirements
  secureElement: {
    type: 'TEE' | 'SE' | 'HSM';
    certification: 'CC EAL4+' | 'FIPS 140-3 Level 3';
    capabilities: string[];
  };

  // Offline token format
  offlineToken: {
    maxValue: MonetaryAmount;
    maxDuration: number;  // Hours
    maxTransactions: number;
    securityBinding: string;
  };

  // Collision prevention
  collisionPrevention: {
    deviceCertificate: boolean;
    hardwareBoundKeys: boolean;
    monotunicCounter: boolean;
    timeSync: boolean;
  };
}

class OfflineSecurityService {
  private secureElement: SecureElement;
  private offlineTokenManager: OfflineTokenManager;

  async generateOfflineTokens(
    wallet: CBDCWallet,
    amount: MonetaryAmount,
    duration: number
  ): Promise<OfflineToken[]> {
    // Verify wallet has sufficient balance
    if (wallet.balance.available < amount) {
      throw new Error('Insufficient balance');
    }

    // Check offline limits
    const limits = await this.getOfflineLimits(wallet);
    if (amount > limits.maxOfflineAmount) {
      throw new Error('Amount exceeds offline limit');
    }

    // Lock funds in online wallet
    await this.lockFunds(wallet.walletId, amount);

    // Generate offline tokens in secure element
    const tokens = await this.secureElement.generateOfflineTokens({
      walletId: wallet.walletId,
      totalAmount: amount,
      validityPeriod: duration,
      maxTransactions: limits.maxOfflineTransactions,
      deviceCertificate: await this.getDeviceCertificate()
    });

    // Each token is cryptographically bound to this device
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
    // Step 1: Mutual authentication via NFC/Bluetooth
    const sessionKey = await this.establishSecureChannel(
      senderSecureElement,
      receiverSecureElement
    );

    // Step 2: Sender creates offline transfer in SE
    const transfer = await senderSecureElement.createOfflineTransfer({
      amount,
      recipientPublicKey: receiverSecureElement.publicKey,
      timestamp: Date.now(),
      counter: await senderSecureElement.incrementCounter()
    });

    // Step 3: Sign with hardware-bound key
    const signature = await senderSecureElement.signOfflineTransfer(transfer);

    // Step 4: Encrypt transfer for recipient
    const encryptedTransfer = await this.encryptWithSessionKey(
      { transfer, signature },
      sessionKey
    );

    // Step 5: Receiver validates and stores
    const decrypted = await receiverSecureElement.decrypt(encryptedTransfer, sessionKey);

    const validationResult = await receiverSecureElement.validateOfflineTransfer(
      decrypted.transfer,
      decrypted.signature
    );

    if (!validationResult.valid) {
      throw new Error(`Offline transfer validation failed: ${validationResult.reason}`);
    }

    // Step 6: Receiver stores pending incoming transfer
    await receiverSecureElement.storePendingTransfer(decrypted.transfer);

    // Step 7: Sender marks tokens as spent
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
    // Get all pending offline transactions from SE
    const pendingTransfers = await secureElement.getPendingTransfers();
    const spentTokens = await secureElement.getSpentTokens();

    const results: TransactionSyncResult[] = [];

    // Sync outgoing (spent) tokens
    for (const spent of spentTokens) {
      try {
        // Submit to central ledger
        const result = await this.submitOfflineSpend(spent, wallet);
        results.push({ type: 'OUTGOING', success: true, id: spent.tokenId });

        // Clear from SE
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

    // Sync incoming transfers
    for (const transfer of pendingTransfers) {
      try {
        // Verify transfer on central ledger
        const verified = await this.verifyOfflineTransfer(transfer);

        if (verified) {
          // Credit to online wallet
          await this.creditOfflineReceived(wallet, transfer);

          // Clear from SE
          await secureElement.clearPendingTransfer(transfer.transferId);

          results.push({ type: 'INCOMING', success: true, id: transfer.transferId });
        } else {
          results.push({
            type: 'INCOMING',
            success: false,
            id: transfer.transferId,
            error: 'Transfer verification failed'
          });
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

    // Unlock any remaining locked funds
    const unspentOfflineBalance = await secureElement.getOfflineBalance();
    if (unspentOfflineBalance > 0) {
      await this.unlockFunds(wallet.walletId, unspentOfflineBalance);
    }

    // Clear offline tokens from SE
    await secureElement.clearOfflineTokens();

    return {
      totalSynced: results.filter(r => r.success).length,
      failed: results.filter(r => !r.success).length,
      results
    };
  }

  private async establishSecureChannel(
    sender: SecureElement,
    receiver: SecureElement
  ): Promise<Buffer> {
    // ECDH key exchange
    const senderEphemeral = await sender.generateEphemeralKeyPair();
    const receiverEphemeral = await receiver.generateEphemeralKeyPair();

    // Exchange public keys (via NFC)
    const sharedSecret1 = await sender.deriveSharedSecret(receiverEphemeral.publicKey);
    const sharedSecret2 = await receiver.deriveSharedSecret(senderEphemeral.publicKey);

    // Derive session key using HKDF
    const sessionKey = await this.hkdf(
      sharedSecret1,
      Buffer.from('CBDC-OFFLINE-V1'),
      32
    );

    return sessionKey;
  }
}
```

### 5.6 HSM Integration

```typescript
// Hardware Security Module Integration
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
    wrapKey: string;  // For backup
  };
}

class HSMService {
  private hsmCluster: HSMCluster;
  private keyHandles: Map<string, HSMKeyHandle>;

  constructor(config: HSMConfiguration) {
    this.hsmCluster = new HSMCluster(config);
    this.keyHandles = new Map();
  }

  async initialize(): Promise<void> {
    // Connect to HSM cluster
    await this.hsmCluster.connect();

    // Verify HSM health
    const health = await this.hsmCluster.healthCheck();
    if (!health.healthy) {
      throw new Error(`HSM cluster unhealthy: ${health.reason}`);
    }

    // Load key handles
    await this.loadKeyHandles();
  }

  async generateKey(
    keySpec: KeySpecification
  ): Promise<HSMKeyInfo> {
    // Generate key inside HSM (never exported)
    const keyHandle = await this.hsmCluster.generateKey({
      algorithm: keySpec.algorithm,
      keySize: keySpec.size,
      keyUsage: keySpec.usage,
      extractable: false
    });

    // Store handle
    this.keyHandles.set(keySpec.keyId, keyHandle);

    // Get public key (exportable)
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
      throw new Error(`Key not found: ${keyId}`);
    }

    // Sign inside HSM
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
    // Require multiple operator approvals for sensitive operations
    const approvals: OperatorApproval[] = [];

    for (const operator of operators) {
      const approval = await this.requestOperatorApproval(
        operator,
        data,
        keyId
      );

      approvals.push(approval);
    }

    // Verify quorum
    const requiredApprovals = await this.getRequiredApprovals(keyId);
    if (approvals.length < requiredApprovals) {
      throw new Error('Insufficient operator approvals');
    }

    // Perform HSM operation with quorum
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

  async backupKey(keyId: string, wrapKeyId: string): Promise<EncryptedKeyBackup> {
    // Wrap key for secure backup (key never leaves HSM unencrypted)
    const keyHandle = this.keyHandles.get(keyId);
    const wrapKeyHandle = this.keyHandles.get(wrapKeyId);

    const wrappedKey = await this.hsmCluster.wrapKey(keyHandle, wrapKeyHandle);

    return {
      keyId,
      wrappedKey,
      wrapKeyId,
      wrappedAt: new Date().toISOString(),
      checksum: await this.calculateChecksum(wrappedKey)
    };
  }
}
```

### 5.7 Summary

The WIA-CBDC security architecture provides:

1. **Hybrid Cryptography**: Classical + post-quantum for future-proofing
2. **HSM-Based Key Management**: Hardware protection for all critical keys
3. **Multi-Signature Support**: Threshold signatures for high-value operations
4. **Zero-Knowledge Proofs**: Privacy-preserving transactions
5. **Offline Security**: Secure element-based offline payments
6. **Anti-Replay Protection**: Nonces, timestamps, and sequence numbers

---

**WIA-CBDC Security Architecture**
**Version**: 1.0.0
**Last Updated**: 2025

© 2025 WIA (World Interoperability Alliance)
