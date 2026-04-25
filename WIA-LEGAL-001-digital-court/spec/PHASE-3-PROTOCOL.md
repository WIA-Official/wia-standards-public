# WIA-LEGAL-001: Digital Court - Phase 3: Protocol

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-27

## Overview

Phase 3 defines the communication protocols, security standards, and data exchange mechanisms for digital court systems, including virtual hearings, secure messaging, and encrypted document transfer.

## 1. Communication Protocols

### 1.1 Virtual Hearing Protocol

**WebRTC-based Video Conferencing**

```
Protocol: WIA-LEGAL-001-VH (Virtual Hearing)
Transport: WebRTC + DTLS-SRTP
Signaling: WebSocket Secure (WSS)
Media: VP9/H.265 (video), Opus (audio)
```

**Connection Flow:**

```
1. Authentication → Court Server
2. Room Creation → Secure Virtual Courtroom
3. Participant Join → Identity Verification
4. Media Negotiation → SDP Exchange
5. Encrypted Streaming → DTLS-SRTP
6. Recording → Server-side encrypted storage
7. Transcription → Real-time AI processing
```

### 1.2 Secure Messaging Protocol

```
Protocol: WIA-LEGAL-001-SM (Secure Messaging)
Encryption: End-to-End (E2EE)
Algorithm: Signal Protocol
Key Exchange: X3DH (Extended Triple Diffie-Hellman)
Ratcheting: Double Ratchet Algorithm
```

**Message Format:**

```json
{
  "messageId": "msg-uuid",
  "caseId": "case-uuid",
  "sender": "did:wia:legal:sender",
  "recipient": "did:wia:legal:recipient",
  "encrypted_payload": "base64-encrypted-data",
  "signature": "ed25519-signature",
  "timestamp": "ISO-8601",
  "ephemeralKey": "x25519-public-key"
}
```

### 1.3 Document Transfer Protocol

```
Protocol: WIA-LEGAL-001-DT (Document Transfer)
Transport: HTTPS with TLS 1.3
Encryption: AES-256-GCM
Integrity: SHA-256 hash + Ed25519 signature
Chunking: Multi-part upload for large files
```

**Transfer Flow:**

```
1. Document Preparation
   → Hash calculation (SHA-256)
   → Metadata extraction
   → Digital signature (Ed25519)

2. Encryption
   → Generate symmetric key (AES-256)
   → Encrypt document
   → Encrypt key with recipient's public key (RSA-OAEP)

3. Upload
   → Multi-part chunked upload
   → Integrity verification per chunk
   → Server-side virus scanning

4. Notification
   → Send secure notification to recipient
   → Access control verification
   → Audit log entry
```

## 2. Security Standards

### 2.1 Encryption Requirements

**Data at Rest:**
- Algorithm: AES-256-GCM
- Key Management: Hardware Security Module (HSM)
- Key Rotation: Every 90 days
- Backup: Encrypted with separate keys

**Data in Transit:**
- TLS 1.3 minimum
- Perfect Forward Secrecy (PFS) required
- Certificate Pinning for mobile apps
- HSTS with preload

**End-to-End Encryption:**
- Signal Protocol for messaging
- PGP/GPG for email
- Sealed sender for anonymity

### 2.2 Authentication Mechanisms

**Multi-Factor Authentication (MFA):**

```
Level 1: Username + Password
Level 2: TOTP (Time-based One-Time Password)
Level 3: Hardware Security Key (FIDO2/WebAuthn)
Level 4: Biometric (for high-security operations)
```

**Identity Verification:**

```json
{
  "verificationType": "multi-factor",
  "factors": [
    {
      "type": "password",
      "strength": "high",
      "lastChanged": "2025-12-01"
    },
    {
      "type": "totp",
      "provider": "authenticator-app",
      "verified": true
    },
    {
      "type": "webauthn",
      "device": "yubikey-5",
      "registered": "2025-11-15"
    }
  ],
  "riskScore": "low",
  "timestamp": "2025-12-27T14:00:00Z"
}
```

### 2.3 Access Control

**Role-Based Access Control (RBAC):**

```yaml
roles:
  judge:
    permissions:
      - case:full_access
      - document:read_all
      - document:seal
      - hearing:preside
      - order:issue

  attorney:
    permissions:
      - case:read_assigned
      - document:file
      - document:read_unsealed
      - hearing:participate
      - motion:file

  party:
    permissions:
      - case:read_own
      - document:read_own
      - hearing:attend

  clerk:
    permissions:
      - case:manage
      - document:process
      - hearing:schedule
```

## 3. Virtual Hearing Specifications

### 3.1 Video Specifications

```
Resolution: 1920x1080 (1080p) minimum
Frame Rate: 30 fps minimum
Codec: VP9, H.265, or AV1
Bitrate: Adaptive (500 Kbps - 5 Mbps)
Latency: < 200ms target
```

### 3.2 Audio Specifications

```
Codec: Opus
Sample Rate: 48 kHz
Bitrate: 64 Kbps (mono), 128 Kbps (stereo)
Audio Processing:
  - Noise Suppression
  - Echo Cancellation
  - Automatic Gain Control
  - Voice Activity Detection
```

### 3.3 Recording Specifications

```
Format: WebM (VP9 + Opus) or MP4 (H.265 + AAC)
Storage: Encrypted at rest (AES-256)
Retention: Per jurisdiction requirements
Access: Audit-logged, role-restricted
Transcription: Automated with 95%+ accuracy
```

### 3.4 Participant Management

```json
{
  "participantId": "participant-uuid",
  "role": "attorney",
  "name": "Jane Doe, Esq.",
  "did": "did:wia:legal:janedoe",
  "permissions": {
    "video": true,
    "audio": true,
    "screenShare": true,
    "chat": true,
    "recordingAccess": false
  },
  "status": {
    "connected": true,
    "muted": false,
    "videoEnabled": true,
    "handRaised": false
  },
  "quality": {
    "videoResolution": "1080p",
    "audioQuality": "excellent",
    "latency": "95ms",
    "packetLoss": "0.1%"
  }
}
```

## 4. Blockchain Integration

### 4.1 Immutable Record Storage

```
Blockchain: Ethereum-compatible (Polygon, Arbitrum)
Smart Contract: WIA-LEGAL-001 Court Records
Storage: IPFS + Filecoin for documents
Anchoring: Periodic merkle root anchoring
```

**Record Structure:**

```solidity
struct CourtRecord {
    bytes32 recordId;
    bytes32 caseId;
    string recordType; // case|document|hearing|order
    bytes32 contentHash; // SHA-256
    string ipfsHash;
    address submittedBy;
    uint256 timestamp;
    bytes signature;
}
```

### 4.2 Chain of Custody

```json
{
  "evidenceId": "evidence-uuid",
  "chainOfCustody": [
    {
      "custodian": "did:wia:legal:officer-smith",
      "action": "collected",
      "location": "Crime Scene A",
      "timestamp": "2025-12-20T10:00:00Z",
      "blockchainTx": "0x123abc...",
      "signature": "ed25519-sig"
    },
    {
      "custodian": "did:wia:legal:lab-technician",
      "action": "analyzed",
      "location": "Forensic Lab",
      "timestamp": "2025-12-21T14:30:00Z",
      "blockchainTx": "0x456def...",
      "signature": "ed25519-sig"
    },
    {
      "custodian": "did:wia:legal:evidence-clerk",
      "action": "stored",
      "location": "Evidence Locker 5",
      "timestamp": "2025-12-22T09:00:00Z",
      "blockchainTx": "0x789ghi...",
      "signature": "ed25519-sig"
    }
  ]
}
```

## 5. Event Streaming

### 5.1 Real-time Updates

```
Protocol: Server-Sent Events (SSE) / WebSocket
Format: JSON
Compression: gzip
Heartbeat: Every 30 seconds
```

**Event Stream Example:**

```
event: case.updated
data: {"caseId":"550e8400-...","status":"hearing_scheduled"}

event: document.filed
data: {"caseId":"550e8400-...","documentId":"doc-123"}

event: hearing.started
data: {"hearingId":"hearing-789","participantCount":5}
```

### 5.2 Event Types

```
case.created
case.updated
case.closed
document.filed
document.served
document.signed
hearing.scheduled
hearing.started
hearing.ended
evidence.submitted
evidence.verified
order.issued
notification.sent
```

## 6. Audit Logging

### 6.1 Audit Log Format

```json
{
  "logId": "log-uuid",
  "eventType": "document.accessed",
  "actor": {
    "did": "did:wia:legal:attorney",
    "role": "attorney",
    "ipAddress": "192.168.1.100",
    "userAgent": "Mozilla/5.0..."
  },
  "resource": {
    "type": "document",
    "id": "doc-123456",
    "caseId": "case-uuid"
  },
  "action": "read",
  "result": "success",
  "timestamp": "2025-12-27T15:00:00Z",
  "metadata": {
    "sessionId": "session-xyz",
    "duration": "120s"
  }
}
```

### 6.2 Retention Requirements

- **Court Records**: Permanent (per jurisdiction)
- **Audit Logs**: Minimum 7 years
- **Video Recordings**: Per case retention policy
- **Temporary Data**: 90 days maximum

## 7. Compliance Standards

### 7.1 Data Protection

- **GDPR** (EU): Full compliance
- **CCPA** (California): Full compliance
- **HIPAA** (Health): When medical records involved
- **SOC 2 Type II**: Annual certification

### 7.2 Legal Standards

- **E-SIGN Act** (US): Digital signature compliance
- **UETA** (US): Electronic transactions
- **eIDAS** (EU): Electronic identification
- **ISO 27001**: Information security management

### 7.3 Accessibility

- **WCAG 2.1 Level AA**: Web accessibility
- **Section 508**: US federal accessibility
- **EN 301 549**: EU accessibility standard

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 WIA - World Certification Industry Association | MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-LEGAL-001-digital-court is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-LEGAL-001-digital-court/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-LEGAL-001-digital-court/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-LEGAL-001-digital-court/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

