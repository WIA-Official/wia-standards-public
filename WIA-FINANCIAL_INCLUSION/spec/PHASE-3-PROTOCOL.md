# WIA-FINANCIAL_INCLUSION: Phase 3 - Protocol Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the communication and operational protocols for FINANCIAL INCLUSION. All implementations MUST follow these protocols to ensure safety, consistency, and data integrity.

## 2. Communication Protocol

### 2.1 Message Structure
```
+------------------+------------------+------------------+
|  Header (64B)    |  Payload (var)   |  Signature (64B) |
+------------------+------------------+------------------+
```

### 2.2 Header Format
```json
{
  "version": "1.0",
  "messageType": "request|response|broadcast|alert",
  "messageId": "UUID",
  "source": "system-identifier",
  "destination": "system-identifier",
  "timestamp": "ISO 8601",
  "priority": "low|normal|high|critical"
}
```

### 2.3 Message Types

| Type | Code | Description |
|------|------|-------------|
| REQUEST | 0x01 | Request operation |
| RESPONSE | 0x02 | Response to request |
| BROADCAST | 0x03 | System-wide notification |
| ALERT | 0x04 | Priority notification |
| SYNC | 0x05 | Synchronization message |
| HEARTBEAT | 0x06 | Health check |

## 3. Operational Protocols

### 3.1 Initialization Protocol
```
1. CONNECT: Establish connection
2. AUTHENTICATE: Verify credentials
3. CONFIGURE: Exchange capabilities
4. SYNC: Synchronize state
5. READY: Begin operations
```

### 3.2 Data Exchange Protocol
```
1. REQUEST: Submit data request
2. VALIDATE: Check request validity
3. PROCESS: Execute operation
4. RESPOND: Return results
5. CONFIRM: Acknowledge receipt
```

### 3.3 Shutdown Protocol
```
1. NOTIFY: Announce shutdown
2. FLUSH: Complete pending operations
3. SYNC: Final state synchronization
4. DISCONNECT: Close connections
5. CLEANUP: Release resources
```

## 4. Safety Protocols

### 4.1 Error Handling Levels
```yaml
Level 1 - Warning:
  - Log event
  - Continue operation
  - Monitor for escalation

Level 2 - Caution:
  - Log with notification
  - Restrict operations
  - Prepare recovery

Level 3 - Alert:
  - Halt new operations
  - Notify administrators
  - Initiate recovery

Level 4 - Critical:
  - Emergency shutdown
  - Full system isolation
  - Immediate investigation
```

### 4.2 Recovery Protocol
```
TRIGGER CONDITIONS:
- Data corruption detected
- System failure
- Security breach
- Operator request

PROCEDURE:
1. Isolate affected systems
2. Assess damage scope
3. Initiate backup restoration
4. Verify data integrity
5. Resume operations
6. Document incident
```

## 5. Synchronization Protocol

### 5.1 State Sync
```json
{
  "protocol": "WIA_SYNC_V1",
  "participants": ["system-1", "system-2"],
  "method": "consensus",
  "timestamp": "ISO 8601",
  "checkpoints": [
    { "id": "checkpoint-1", "hash": "sha256", "verified": true }
  ]
}
```

### 5.2 Clock Synchronization
```
1. Exchange timestamps
2. Calculate round-trip time
3. Adjust for network latency
4. Verify with reference
5. Establish synchronized time
```

## 6. Security Protocol

### 6.1 Encryption
- Transport: TLS 1.3
- Data at rest: AES-256-GCM
- Key exchange: ECDHE
- Signatures: Ed25519

### 6.2 Authentication
```
Root CA → Intermediate CA → Service Certificate → Client Certificate
```

### 6.3 Access Control
| Level | Access |
|-------|--------|
| Observer | Read only |
| Operator | Read/Write |
| Administrator | Full access |
| Auditor | Read + Audit logs |

## 7. Logging Requirements

### 7.1 Required Logs
- All operations
- Error conditions
- Security events
- State changes
- Performance metrics

### 7.2 Log Format
```json
{
  "timestamp": "ISO 8601",
  "level": "DEBUG|INFO|WARN|ERROR|CRITICAL",
  "source": "component-id",
  "event": "event-type",
  "message": "description",
  "context": {}
}
```

### 7.3 Retention
- Active logs: 90 days
- Archive: 7 years
- Audit logs: Indefinite

---

**弘益人間 (Benefit All Humanity)**

## Reference Standards Alignment

| Concern | Reference |
|---------|-----------|
| Financial-services messaging | ISO 20022 |
| Card-originated transactions | ISO 8583 |
| Transport security | RFC 8446 (TLS 1.3); RFC 5246 (TLS 1.2 minimum) |
| Certificate format | RFC 5280 (X.509 v3) |
| Time-stamping | RFC 3161 / RFC 5816 |
| HTTP semantics | RFC 9110 |
| HTTP/2 | RFC 9113 |
| HTTP/3 over QUIC | RFC 9114 / RFC 9000 |
| WebSocket | RFC 6455 |
| Token formats | RFC 7515 (JWS), RFC 7519 (JWT), RFC 9052 (COSE), RFC 8949 (CBOR) |
| Provenance | W3C PROV-O |
| Identity | W3C DID Core 1.0, W3C VC Data Model 2.0 |
| Time encoding | ISO 8601:2019 |
| Information security | ISO/IEC 27001:2022 |
| Privacy | ISO/IEC 27701:2019 |
| Risk-data aggregation | BCBS 239 |
| AML/CFT | FATF Recommendations |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

## Conformance

A Phase 3 implementation is conformant when:

1. Wire payloads conform to ISO 20022 message types where the deployment integrates with traditional financial-services rails.
2. TLS 1.3 (or 1.2 minimum) is supported with RFC 5280 certificate verification.
3. Token formats follow JWT (RFC 7519) with explicit algorithm identifiers.
4. Provenance assertions follow W3C PROV-O.
5. Information-security controls map to a published ISO/IEC 27001 statement of applicability.

## Implementation Appendix

Settlement protocols are designed to interoperate with widely deployed real-time payment systems (the deploying jurisdiction's instant-payment scheme) and with traditional batch settlement (where instant payment is not available). The protocol layer abstracts the underlying scheme so that financial-inclusion applications target a uniform API rather than encoding scheme-specific details.

For offline scenarios common in underbanked deployments, the protocol supports store-and-forward semantics: transactions are signed locally on the device, queued for later synchronisation when connectivity is restored, and reconciled with the operating organisation's ledger through idempotent retry.

Recordkeeping retention follows the deploying jurisdiction's financial-services rules (e.g., BSA recordkeeping in the US, the EU AML directive's retention rules, Korea PIPA Article 21's retention provisions for sensitive personal information). The retention configuration is published as part of the deployment's compliance documentation.

## Wire Protocol Detail

### Message Authentication

Every wire message carries a signature by the originating party (RFC 8032 EdDSA by default, NIST FIPS 186-5 ECDSA acceptable for legacy compatibility). Signatures cover the canonicalised message including the message type, parties, amount, currency, and timestamp. The signing key resolves through the originator's W3C DID (DID Core 1.0).

### Settlement Finality

Settlement finality follows the deploying jurisdiction's payment-systems rule. The reference protocol distinguishes between three finality classes:

- **Provisional** — credited to the destination but reversible until clearing concludes.
- **Settlement-final** — credited and irreversible per the operating scheme's rules.
- **Legally final** — settlement-final and not subject to claw-back under the deploying jurisdiction's bankruptcy or insolvency rules.

Each transaction record carries the achieved finality class so downstream systems can plan accordingly.

### Reconciliation

Reconciliation between operating organisations follows the wire-protocol envelope augmented with cryptographic proofs of inclusion in each side's ledger. Differences detected during reconciliation are documented with the rule-defined dispute-resolution procedure.

### Offline Operations

For offline operations common in underbanked deployments, the protocol supports store-and-forward queues with idempotent retry. Offline transactions are signed locally on the device, queued in the local ledger, and synchronised with the operating organisation's central ledger when connectivity is restored.

### Cross-Border Wire Format

Cross-border transactions wrap the WIA-FINANCIAL_INCLUSION envelope in an ISO 20022 message frame (typically pacs.008 for credit transfers) so that traditional banking infrastructure can carry the message without translation. The wrapping carries the WIA envelope as a structured remittance information field.

### Resilience

Wire-protocol resilience properties:

- Out-of-order message delivery handled by sequence numbers.
- Duplicate detection by idempotency keys and message hashes.
- Disconnection handled by store-and-forward queues with idempotent retry.
- Partition handled by independent ledger operation with reconciliation on rejoin.

### Conformance Test Vectors

The conformance test suite covers wire interoperability between two conformant implementations under controlled network conditions. Each test vector documents the input message, the expected output message, the active finality class, and the active wire-protocol version so that any implementation can self-assess against the same baseline.

---



## Protocol Versioning

Protocol versions follow semantic versioning. Wire-compatible minor version increments allow producers and consumers to negotiate the maximum mutually supported version through capability discovery. Major-version increments coexist with the prior major for at least 18 months before deprecation.

## Diagnostic Logging

Diagnostic events are emitted in structured form per RFC 5424 syslog severity classes, with the W3C Trace Context `traceparent` embedded so that traces span the originating client, the operating organisation's API tier, and the downstream payment-processor or banking-rail.

## Privacy Considerations

The protocol carries personal data (party identifiers, transaction details). Minimum-necessary disclosure is enforced at the protocol layer:

- Wire messages exposing party identity outside the operating organisation are encrypted end-to-end.
- Aggregate analytics use aggregation primitives that protect individual-level privacy (e.g., differential-privacy noise addition for cross-organisation aggregations).
- Subject access, rectification, and deletion follow the deploying jurisdiction's privacy law.

## Test Vectors

The Phase 3 conformance test suite is published as an open package. Each test vector documents the input message, the expected output, and the active wire-protocol version. Implementations report their test-vector pass rate as part of the conformance evidence.

## Reconciliation Considerations

Cross-organisation reconciliation runs on a documented cadence (typically end-of-day for retail systems, near-real-time for instant-payment integrations). The reconciliation procedure produces a signed reconciliation report that lists matched records, unmatched records, and disputed records. Reconciliation artefacts are retained per the deploying jurisdiction's recordkeeping rule.

## Wire Telemetry

Wire-level telemetry is emitted in OpenTelemetry semantic-convention form so that distributed traces span the originating client, the operating organisation, and downstream payment processors. Trace identifiers are emitted in error responses to aid investigation when transactions fail or are delayed.

## Cryptographic Agility

Implementations support multiple signature algorithms simultaneously and select per-message based on the operating organisation's cryptographic policy. The reference policy defaults to Ed25519 (RFC 8032); ECDSA P-256 (NIST FIPS 186-5) is acceptable for legacy hardware. Algorithm identifiers travel with each signature, and verifiers refuse to silently substitute algorithms.

## Long-Term Verifiability

Wire messages signed today must remain verifiable for the duration of the deploying jurisdiction's recordkeeping retention. Long-term verifiability is achieved through explicit algorithm identifiers, open hash and signature primitives with public test vectors, content-addressed payloads, and the operating organisation's commitment to preserve revocation information for retired keys.

---

*© 2025 WIA - World Certification Industry Association*
*MIT License*
