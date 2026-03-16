# WIA-UNI-002: Phase 3 - Protocol Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This document specifies the cross-border verification protocol, privacy-preserving algorithms, and security mechanisms for the WIA-UNI-002 Unified ID System.

## 2. Cross-Border Verification Protocol

### 2.1 Protocol Flow

1. **Presentation**: Citizen presents unified ID credential
2. **Challenge**: Verifier issues cryptographic challenge
3. **Proof Generation**: Citizen's device generates ZK proof
4. **Verification**: Verifier checks proof against blockchain
5. **Authorization**: Access granted if verification successful
6. **Audit**: Transaction logged (privacy-preserved)

### 2.2 Challenge-Response Protocol

```
Verifier → Citizen: challenge = random_nonce()
Citizen → Verifier: proof = zkp_generate(credential, challenge)
Verifier: verify(proof, challenge, blockchain_registry)
```

## 3. Zero-Knowledge Proofs

### 3.1 Supported Proof Types

| Proof Type | Description | Use Case |
|------------|-------------|----------|
| `age-over-{N}` | Prove age > N | Age-restricted services |
| `citizenship` | Prove unified citizenship | Border crossing |
| `authorization-{type}` | Prove specific permission | Service access |
| `family-relationship` | Prove family tie | Family reunification |

### 3.2 ZK-SNARK Implementation

**Circuit:**
```
public input: current_date, age_threshold
private input: birth_date, signature
output: is_over_age = (current_date - birth_date) >= (age_threshold * 365)
```

**Proof Generation:**
```javascript
const proof = await snarkjs.groth16.fullProve(
  { birthDate, signature },
  { currentDate, ageThreshold },
  wasmFile,
  zkeyFile
);
```

## 4. Privacy-Preserving Algorithms

### 4.1 Blind Signatures

Used for issuing credentials without authority seeing content:

1. Citizen blinds credential: `blinded = blind(credential, r)`
2. Authority signs: `signature = sign(blinded, private_key)`
3. Citizen unblinds: `unblinded_sig = unblind(signature, r)`

### 4.2 Homomorphic Encryption

Enables computation on encrypted data:

```python
# Example: Count citizens by age range without decrypting
encrypted_ages = [encrypt(age) for age in ages]
age_range_count = sum([age >= 18 and age < 65 for age in encrypted_ages])
# Result computed on encrypted data
```

## 5. Blockchain Integration

### 5.1 Registry Structure

**Public Ledger Contents:**
- Citizen public keys (for signature verification)
- Revocation accumulator (cryptographic set)
- Issuer registry (trusted authorities)
- Audit hashes (privacy-preserving logs)

**NOT Stored:**
- Personal information
- Biometric data
- Origin region
- Family relationships

### 5.2 Consensus Mechanism

**Multi-Party Governance:**
- North Korea government nodes (33% voting power)
- South Korea government nodes (33% voting power)
- International observers (17% voting power)
- Civilian oversight (17% voting power)

**Transaction Approval:**
- Requires majority from each category
- Critical operations require 75% total approval

## 6. Offline Verification

### 6.1 Offline Mode

When network unavailable:

1. **Cached Revocation List**: Device stores last-known revocations
2. **Local Verification**: Check signature and expiration locally
3. **Risk Assessment**: Flag for online verification when connected
4. **Queue Audit**: Store transaction for later blockchain submission

### 6.2 Sync Protocol

```
When network available:
1. Download latest revocation list
2. Upload queued audit transactions
3. Verify no credentials were revoked while offline
4. Resolve conflicts if necessary
```

## 7. Security Mechanisms

### 7.1 Replay Attack Prevention

- Timestamp-based nonces (valid for 60 seconds)
- Challenge-response protocol
- Proof includes current timestamp
- Verifier checks timestamp freshness

### 7.2 Man-in-the-Middle Protection

- TLS 1.3 with certificate pinning
- End-to-end encryption of proof data
- Device attestation for mobile wallets
- Mutual authentication between parties

### 7.3 Denial of Service Mitigation

- Rate limiting per IP and client ID
- Proof-of-work for anonymous requests
- Distributed verification nodes
- DDoS protection at CDN layer

## 8. Cross-Border Checkpoints

### 8.1 DMZ Verification Stations

Special protocol for demilitarized zone:

1. **Dual Verification**: Both North and South systems verify
2. **Consensus Requirement**: Both must agree for passage
3. **Audit Trail**: Immutable record on blockchain
4. **Privacy Protection**: No origin region exposed

### 8.2 Authorized Crossing Reasons

| Reason | Authorization Level | Duration |
|--------|---------------------|----------|
| Family Reunion | Standard | 7 days |
| Business | Enhanced | 30 days |
| Emergency Aid | Priority | 14 days |
| Permanent Relocation | Full | Indefinite |

## 9. Emergency Protocols

### 9.1 System Outage

In case of blockchain or API outage:

1. Switch to offline mode
2. Use paper backup credentials
3. Biometric-only verification
4. Manual log keeping
5. Sync when system restored

### 9.2 Security Breach Response

If cryptographic compromise detected:

1. Immediate key rotation
2. Notify all credential holders
3. Issue replacement credentials
4. Audit all transactions
5. Public disclosure within 72 hours

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
