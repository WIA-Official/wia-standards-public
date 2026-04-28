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

---

## Z.1 Audit transport and observability hooks (Phase 3)

Every Phase 3 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `unified-id-system` and `wia.standard.phase` =
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 3)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-unified-id-system-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3)

Phase 3 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 3)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.

---

## Z.1 Audit transport and observability hooks (Phase 3 (variant 1))

Every Phase 3 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `unified-id-system` and `wia.standard.phase` =
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3 (variant 1))

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 3 (variant 1))

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-unified-id-system-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3 (variant 1))

Phase 3 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3 (variant 1))

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 3 (variant 1))

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
