# WIA-EDU-006 — Phase 3: PROTOCOL

> Virtual Classroom canonical Phase 3 specification per the WIA Standards four-Phase architecture.

> Domain: 가상 교실 — 원격 교육 · LMS · LRS · LTI · WebRTC · 접근성.

## A.1 Scope

This Phase covers the canonical protocol layer of the WIA-EDU-006 standard. It composes with the Phase 4 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- IMS Global LTI Advantage 1.3
- IEEE 9274.1.1 xAPI
- ADL cmi5 v1.0
- SCORM 2004 4th Edition
- WCAG 2.2 AA
- ISO/IEC 19796-1 Quality management
- EU Web Accessibility Directive 2016/2102
- ADA Title II / III
- IETF RFC 8866 SDP / WebRTC

## 1. Introduction

### 1.1 Purpose

This standard provides:
- Unified APIs for virtual classroom components
- Video and audio streaming specifications
- Interactive whiteboard protocols
- Breakout room management standards
- Attendance and engagement tracking frameworks
- LMS integration specifications

### 1.2 Scope

This standard applies to:
- K-12 and higher education institutions
- Corporate training platforms
- Online tutoring services
- Massive Open Online Courses (MOOCs)
- Virtual workshops and webinars

### 1.3 Definitions

- **Virtual Classroom**: Online environment for synchronous learning
- **Breakout Room**: Smaller session for group discussions
- **Whiteboard**: Shared collaborative canvas
- **Attendance**: Real-time tracking of participant presence
- **Engagement**: Metrics measuring student interaction and attention


## 5. Security & Privacy

### 5.1 Encryption

- TLS 1.3 for all API communications
- End-to-end encryption for WebRTC streams (DTLS-SRTP)
- AES-256 encryption for recordings
- Encrypted storage for personal data

### 5.2 Authentication

**OAuth 2.0 / OIDC:**
```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=abc123&
client_id=your_client_id&
client_secret=your_client_secret
```

**JWT Token:**
```json
{
  "sub": "user_123",
  "name": "John Doe",
  "email": "john@university.edu",
  "role": "student",
  "sessionId": "session_xyz789",
  "iat": 1704067200,
  "exp": 1704153600
}
```

### 5.3 Compliance

- FERPA (Family Educational Rights and Privacy Act)
- GDPR (General Data Protection Regulation)
- COPPA (Children's Online Privacy Protection Act)
- CCPA (California Consumer Privacy Act)


## 9. Error Handling

### 9.1 Error Codes

| Code | Description |
|------|-------------|
| 1001 | Session not found |
| 1002 | Unauthorized access |
| 1003 | Session capacity reached |
| 1004 | Invalid session state |
| 2001 | Whiteboard error |
| 3001 | Recording failed |

### 9.2 Error Response Format

```json
{
  "error": {
    "code": 1003,
    "message": "Session capacity reached",
    "details": "Maximum 100 participants allowed",
    "timestamp": "2025-01-20T10:00:00Z"
  }
}
```


## What's New in v1.1

### Enhanced Collaboration Features
- **Multi-whiteboard support** - Create and switch between multiple whiteboards
- **Advanced annotation tools** - Shape recognition, ink-to-text conversion
- **File collaboration** - Real-time co-editing of documents (Google Docs, Office 365)
- **Interactive 3D models** - View and manipulate 3D objects in the classroom

### Mobile Optimization
- **Native mobile apps** - iOS and Android with full feature parity
- **Touch-optimized whiteboard** - Gesture support for zoom, pan, rotate
- **Offline mode** - Download materials for offline access
- **Reduced bandwidth mode** - Optimized for low-bandwidth connections

### Advanced Analytics
- **Heatmap visualization** - See where students focus attention
- **Participation analytics** - Track individual and group engagement
- **Predictive analytics** - Identify at-risk students early
- **Custom dashboards** - Create personalized analytics views

### Accessibility Improvements
- **Live captioning in 40+ languages** - Real-time transcription
- **Sign language interpretation** - Dedicated video tile for interpreters
- **Dyslexia-friendly fonts** - OpenDyslexic font support
- **Color blind modes** - Multiple color scheme options

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
with `wia.standard.slug` = `virtual-classroom` and `wia.standard.phase` =
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
`/.well-known/wia-virtual-classroom-capabilities` enumerating per-endpoint
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
with `wia.standard.slug` = `virtual-classroom` and `wia.standard.phase` =
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
`/.well-known/wia-virtual-classroom-capabilities` enumerating per-endpoint
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
