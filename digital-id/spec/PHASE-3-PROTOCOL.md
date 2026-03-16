# WIA-SOC-002 PHASE 3: Protocol Specification

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

This document specifies the verification protocols, cryptographic methods, and security mechanisms for WIA-SOC-002 digital identity operations.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer (DID/VC)       │
├─────────────────────────────────────┤
│   Verification Protocol Layer       │
├─────────────────────────────────────┤
│   Cryptographic Layer (Signatures)  │
├─────────────────────────────────────┤
│   Network Layer (HTTPS/WSS)         │
├─────────────────────────────────────┤
│   Blockchain Layer (Ethereum/etc)   │
└─────────────────────────────────────┘
```

---

## 2. DID Resolution Protocol

### 2.1 Resolution Process

```
┌─────────┐         ┌──────────┐         ┌───────────┐
│ Resolver│────1───>│DID Method│────2───>│Blockchain │
│         │<───4────│ Driver   │<───3────│ / Registry│
└─────────┘         └──────────┘         └───────────┘
```

**Steps:**
1. Resolver receives DID
2. Method driver queries blockchain/registry
3. Registry returns DID Document
4. Method driver formats and returns document

### 2.2 Resolution Algorithm

```typescript
async function resolveDID(did: string): Promise<DIDDocument> {
  // 1. Parse DID
  const { method, methodSpecificId } = parseDID(did);

  // 2. Select method driver
  const driver = getMethodDriver(method);

  // 3. Query registry
  const document = await driver.resolve(methodSpecificId);

  // 4. Validate document
  if (!validateDIDDocument(document)) {
    throw new Error('Invalid DID Document');
  }

  // 5. Return document
  return document;
}
```

### 2.3 Caching

- **TTL**: 24 hours for active DIDs
- **Invalidation**: On DID update events
- **Cache-Control**: `max-age=86400, must-revalidate`

---

## 3. Credential Verification Protocol

### 3.1 Verification Flow

```
┌─────────┐         ┌──────────┐         ┌─────────┐
│ Holder  │────1───>│ Verifier │────2───>│ Issuer  │
│         │         │          │<───3────│ DID Doc │
│         │         │          │────4───>│Revoc.   │
│         │         │          │<───5────│List     │
└─────────┘         └──────────┘         └─────────┘
     │                   │
     └────────6──────────┘
        (verified)
```

**Steps:**
1. Holder presents credential to verifier
2. Verifier resolves issuer DID
3. Verifier gets issuer's public key
4. Verifier checks revocation status
5. Verifier receives revocation list
6. Verifier confirms validity to holder

### 3.2 Verification Algorithm

```typescript
async function verifyCredential(
  credential: VerifiableCredential
): Promise<VerificationResult> {
  // 1. Check credential structure
  if (!validateStructure(credential)) {
    return { verified: false, reason: 'Invalid structure' };
  }

  // 2. Resolve issuer DID
  const issuerDoc = await resolveDID(credential.issuer.id);

  // 3. Get verification method
  const verificationMethod = getVerificationMethod(
    issuerDoc,
    credential.proof.verificationMethod
  );

  // 4. Verify signature
  const signatureValid = await verifySignature(
    credential,
    verificationMethod.publicKey
  );

  if (!signatureValid) {
    return { verified: false, reason: 'Invalid signature' };
  }

  // 5. Check expiration
  if (isExpired(credential.expirationDate)) {
    return { verified: false, reason: 'Credential expired' };
  }

  // 6. Check revocation
  const isRevoked = await checkRevocationStatus(credential);

  if (isRevoked) {
    return { verified: false, reason: 'Credential revoked' };
  }

  // 7. All checks passed
  return { verified: true };
}
```

---

## 4. Signature Protocols

### 4.1 EdDSA (Ed25519)

**Algorithm:** EdDSA with Ed25519 curve

```typescript
// Sign
function signEd25519(message: Uint8Array, privateKey: Uint8Array): Uint8Array {
  return ed25519.sign(message, privateKey);
}

// Verify
function verifyEd25519(
  signature: Uint8Array,
  message: Uint8Array,
  publicKey: Uint8Array
): boolean {
  return ed25519.verify(signature, message, publicKey);
}
```

**Message Canonicalization:**
```typescript
function canonicalizeForSigning(credential: any): string {
  // 1. Remove proof
  const { proof, ...credentialWithoutProof } = credential;

  // 2. Canonicalize using JSON-LD
  const canonical = await jsonld.canonize(credentialWithoutProof, {
    algorithm: 'URDNA2015',
    format: 'application/n-quads'
  });

  return canonical;
}
```

### 4.2 ECDSA (secp256k1)

**Algorithm:** ECDSA with secp256k1 curve (Ethereum compatible)

```typescript
// Sign
function signECDSA(messageHash: Uint8Array, privateKey: Uint8Array): {
  r: Uint8Array;
  s: Uint8Array;
  v: number;
} {
  return secp256k1.sign(messageHash, privateKey);
}

// Verify
function verifyECDSA(
  signature: { r: Uint8Array; s: Uint8Array; v: number },
  messageHash: Uint8Array,
  publicKey: Uint8Array
): boolean {
  return secp256k1.verify(signature, messageHash, publicKey);
}
```

---

## 5. Zero-Knowledge Proof Protocol

### 5.1 ZK Proof Generation

```typescript
async function generateZKProof(
  credential: VerifiableCredential,
  statement: ProofStatement,
  witness: Witness
): Promise<ZKProof> {
  // 1. Load circuit
  const circuit = await loadCircuit(statement.type);

  // 2. Generate witness
  const fullWitness = circuit.calculateWitness(witness);

  // 3. Generate proof using Groth16
  const { proof, publicSignals } = await groth16.fullProve(
    witness,
    circuit.wasm,
    circuit.zkey
  );

  // 4. Format proof
  return {
    type: 'ZKProof',
    proofSystem: 'groth16',
    curve: 'bn254',
    statement,
    proof: {
      pi_a: proof.pi_a,
      pi_b: proof.pi_b,
      pi_c: proof.pi_c
    },
    publicSignals
  };
}
```

### 5.2 ZK Proof Verification

```typescript
async function verifyZKProof(
  zkProof: ZKProof,
  verificationKey: VerificationKey
): Promise<boolean> {
  // 1. Validate proof structure
  if (!validateZKProofStructure(zkProof)) {
    return false;
  }

  // 2. Verify proof using Groth16
  const isValid = await groth16.verify(
    verificationKey,
    zkProof.publicSignals,
    zkProof.proof
  );

  return isValid;
}
```

### 5.3 Selective Disclosure Protocol

```typescript
async function selectiveDisclose(
  credential: VerifiableCredential,
  revealed: string[],
  hidden: string[]
): Promise<VerifiablePresentation> {
  // 1. Create hashes for hidden fields
  const hiddenHashes = {};
  for (const field of hidden) {
    const value = credential.credentialSubject[field];
    const salt = generateSalt();
    hiddenHashes[field] = {
      hash: sha256(value + salt),
      salt
    };
  }

  // 2. Create partial credential
  const partialSubject = {};
  for (const field of revealed) {
    partialSubject[field] = credential.credentialSubject[field];
  }

  // 3. Create presentation
  return {
    '@context': credential['@context'],
    type: 'VerifiablePresentation',
    verifiableCredential: [{
      ...credential,
      credentialSubject: {
        id: credential.credentialSubject.id,
        ...partialSubject
      }
    }],
    selectiveDisclosure: {
      revealed,
      hiddenHashes,
      hashAlgorithm: 'SHA-256'
    }
  };
}
```

---

## 6. Revocation Protocol

### 6.1 Status List 2021

```typescript
class StatusList2021 {
  private bitstring: Uint8Array;

  // Set credential as revoked
  async revoke(index: number): Promise<void> {
    const byteIndex = Math.floor(index / 8);
    const bitIndex = index % 8;
    this.bitstring[byteIndex] |= (1 << bitIndex);

    // Update on-chain
    await this.updateOnChain();
  }

  // Check if credential is revoked
  isRevoked(index: number): boolean {
    const byteIndex = Math.floor(index / 8);
    const bitIndex = index % 8;
    return (this.bitstring[byteIndex] & (1 << bitIndex)) !== 0;
  }

  // Compress bitstring
  compress(): string {
    return gzip.compress(this.bitstring);
  }
}
```

### 6.2 Blockchain Anchoring

```solidity
// Smart contract for revocation registry
contract RevocationRegistry {
    mapping(bytes32 => bool) public revoked;
    mapping(address => bool) public issuers;

    event CredentialRevoked(
        bytes32 indexed credentialHash,
        address indexed issuer,
        uint256 timestamp
    );

    function revokeCredential(bytes32 credentialHash) external {
        require(issuers[msg.sender], "Not authorized issuer");
        revoked[credentialHash] = true;
        emit CredentialRevoked(credentialHash, msg.sender, block.timestamp);
    }

    function isRevoked(bytes32 credentialHash) external view returns (bool) {
        return revoked[credentialHash];
    }
}
```

---

## 7. Authentication Protocol

### 7.1 DID Authentication

```typescript
async function authenticateWithDID(
  challenge: string,
  did: string,
  privateKey: Uint8Array
): Promise<AuthenticationProof> {
  // 1. Create authentication message
  const message = {
    challenge,
    did,
    timestamp: Date.now()
  };

  // 2. Sign message
  const messageBytes = encodeToBytes(message);
  const signature = await signEd25519(messageBytes, privateKey);

  // 3. Create proof
  return {
    type: 'DIDAuthentication',
    did,
    challenge,
    timestamp: message.timestamp,
    proof: {
      type: 'Ed25519Signature2020',
      verificationMethod: `${did}#key-1`,
      proofValue: encodeBase64(signature)
    }
  };
}
```

### 7.2 Verifying Authentication

```typescript
async function verifyDIDAuthentication(
  authProof: AuthenticationProof
): Promise<boolean> {
  // 1. Check timestamp (within 5 minutes)
  const now = Date.now();
  if (Math.abs(now - authProof.timestamp) > 300000) {
    return false;
  }

  // 2. Resolve DID
  const didDoc = await resolveDID(authProof.did);

  // 3. Get public key
  const verificationMethod = getVerificationMethod(
    didDoc,
    authProof.proof.verificationMethod
  );

  // 4. Verify signature
  const message = {
    challenge: authProof.challenge,
    did: authProof.did,
    timestamp: authProof.timestamp
  };

  const messageBytes = encodeToBytes(message);
  const signature = decodeBase64(authProof.proof.proofValue);

  return verifyEd25519(signature, messageBytes, verificationMethod.publicKey);
}
```

---

## 8. Communication Protocols

### 8.1 DIDComm Messaging

```typescript
interface DIDCommMessage {
  id: string;
  type: string;
  from: string; // DID
  to: string[]; // DIDs
  created_time: number;
  expires_time?: number;
  body: any;
}

// Encrypt message
async function encryptDIDCommMessage(
  message: DIDCommMessage,
  recipientPublicKey: Uint8Array
): Promise<JWE> {
  const jwe = await jose.encrypt(
    JSON.stringify(message),
    recipientPublicKey,
    {
      alg: 'ECDH-ES+A256KW',
      enc: 'A256GCM'
    }
  );

  return jwe;
}

// Decrypt message
async function decryptDIDCommMessage(
  jwe: JWE,
  recipientPrivateKey: Uint8Array
): Promise<DIDCommMessage> {
  const decrypted = await jose.decrypt(jwe, recipientPrivateKey);
  return JSON.parse(decrypted);
}
```

### 8.2 Verifiable Presentation Protocol

```
┌─────────┐                           ┌──────────┐
│ Holder  │                           │ Verifier │
└────┬────┘                           └─────┬────┘
     │                                      │
     │  1. Request presentation             │
     │<─────────────────────────────────────┤
     │                                      │
     │  2. Create & sign presentation       │
     ├─────────────────────────────────────>│
     │                                      │
     │                3. Verify presentation │
     │                                      │
     │  4. Verification result              │
     │<─────────────────────────────────────┤
     │                                      │
```

---

## 9. Security Protocols

### 9.1 Key Rotation

```typescript
async function rotateDIDKeys(
  did: string,
  oldPrivateKey: Uint8Array,
  newPublicKey: Uint8Array
): Promise<void> {
  // 1. Resolve current DID document
  const didDoc = await resolveDID(did);

  // 2. Add new key
  didDoc.verificationMethod.push({
    id: `${did}#key-new`,
    type: 'Ed25519VerificationKey2020',
    controller: did,
    publicKeyMultibase: encodeMultibase(newPublicKey)
  });

  // 3. Update authentication
  didDoc.authentication.push(`${did}#key-new`);

  // 4. Sign update with old key
  const proof = await signDIDDocument(didDoc, oldPrivateKey);
  didDoc.proof = proof;

  // 5. Publish update
  await updateDIDDocument(did, didDoc);

  // 6. After grace period, remove old key
  setTimeout(async () => {
    await removeOldKey(did, `${did}#key-1`);
  }, 7 * 24 * 60 * 60 * 1000); // 7 days
}
```

### 9.2 Multi-Signature

```typescript
async function createMultiSigProof(
  credential: VerifiableCredential,
  signers: Array<{ did: string; privateKey: Uint8Array }>,
  threshold: number
): Promise<VerifiableCredential> {
  const signatures = [];

  for (const signer of signers) {
    const canonical = await canonicalizeForSigning(credential);
    const signature = await signEd25519(
      encodeToBytes(canonical),
      signer.privateKey
    );

    signatures.push({
      type: 'Ed25519Signature2020',
      verificationMethod: `${signer.did}#key-1`,
      proofValue: encodeBase64(signature)
    });
  }

  credential.proof = {
    type: 'MultiSignature2021',
    threshold,
    signatures
  };

  return credential;
}
```

---

## 10. Privacy Protocols

### 10.1 Unlinkability

```typescript
// Generate correlation-resistant identifier
function generatePairwiseDID(
  masterDID: string,
  relatedParty: string
): string {
  const seed = deriveKey(masterDID, relatedParty);
  const keypair = generateKeypairFromSeed(seed);
  return createDID(keypair.publicKey);
}

// Usage: Different DID for each relying party
const didForBankA = generatePairwiseDID(masterDID, 'bank-a.com');
const didForBankB = generatePairwiseDID(masterDID, 'bank-b.com');
// Banks cannot correlate these DIDs
```

### 10.2 Blinded Credentials

```typescript
async function issueBlindedCredential(
  credentialRequest: BlindedRequest,
  issuerPrivateKey: Uint8Array
): Promise<BlindedCredential> {
  // 1. Verify request signature
  if (!await verifyBlindedRequest(credentialRequest)) {
    throw new Error('Invalid request');
  }

  // 2. Sign blinded credential
  const blindSignature = await blindSign(
    credentialRequest.blindedMessage,
    issuerPrivateKey
  );

  // 3. Return blinded credential
  return {
    blindedCredential: credentialRequest.blindedMessage,
    blindSignature
  };
}

// Holder unblinds credential
async function unblindCredential(
  blindedCredential: BlindedCredential,
  blindingFactor: Uint8Array
): Promise<VerifiableCredential> {
  const signature = unblind(
    blindedCredential.blindSignature,
    blindingFactor
  );

  return {
    ...blindedCredential.credential,
    proof: {
      type: 'RsaSignature2018',
      proofValue: signature
    }
  };
}
```

---

## 11. Performance Optimization

### 11.1 Batch Verification

```typescript
async function batchVerifyCredentials(
  credentials: VerifiableCredential[]
): Promise<VerificationResult[]> {
  // 1. Group by issuer
  const groups = groupByIssuer(credentials);

  // 2. Resolve DIDs in parallel
  const didDocs = await Promise.all(
    Object.keys(groups).map(issuer => resolveDID(issuer))
  );

  // 3. Verify signatures in batch
  const results = await Promise.all(
    credentials.map(credential =>
      verifyCredential(credential)
    )
  );

  return results;
}
```

### 11.2 Caching Strategy

```typescript
class VerificationCache {
  private cache = new Map<string, CachedVerification>();
  private ttl = 3600000; // 1 hour

  async verify(credential: VerifiableCredential): Promise<boolean> {
    const cacheKey = this.getCacheKey(credential);
    const cached = this.cache.get(cacheKey);

    if (cached && Date.now() - cached.timestamp < this.ttl) {
      return cached.result;
    }

    const result = await verifyCredential(credential);

    this.cache.set(cacheKey, {
      result,
      timestamp: Date.now()
    });

    return result;
  }
}
```

---

**Document Status:** Complete ✅
**Next Phase:** [PHASE-4-INTEGRATION.md](PHASE-4-INTEGRATION.md)

弘益人間 · Benefit All Humanity
