# WIA-EDU-006 — Phase 1: DATA-FORMAT

> Virtual Classroom canonical Phase 1 specification per the WIA Standards four-Phase architecture.

> Domain: 가상 교실 — 원격 교육 · LMS · LRS · LTI · WebRTC · 접근성.

## A.1 Scope

This Phase covers the canonical data-format layer of the WIA-EDU-006 standard. It composes with the Phase 2 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## # WIA-EDU-006: Virtual Classroom Standard v1.0

**Status:** Release Candidate
**Date:** 2025-01-15
**Authors:** WIA Education Committee
**Category:** Education (EDU)

---


## 3. Core APIs

### 3.1 Session Management API

**Create Session:**
```http
POST /api/v1/sessions
Content-Type: application/json

{
  "title": "Introduction to Machine Learning",
  "instructor": {
    "userId": "instructor_123",
    "name": "Dr. Sarah Johnson",
    "email": "sarah@university.edu"
  },
  "schedule": {
    "startTime": "2025-01-20T10:00:00Z",
    "duration": 90,
    "timezone": "America/Los_Angeles"
  },
  "settings": {
    "maxStudents": 100,
    "waitingRoom": true,
    "requirePassword": true,
    "password": "ML2025",
    "enableRecording": true,
    "muteOnEntry": true
  }
}
```

**Response:**
```json
{
  "sessionId": "session_xyz789",
  "title": "Introduction to Machine Learning",
  "joinUrl": "https://classroom.wia.org/session_xyz789",
  "instructorUrl": "https://classroom.wia.org/session_xyz789?role=instructor&token=abc123",
  "status": "scheduled",
  "createdAt": "2025-01-15T08:00:00Z"
}
```

### 3.2 Attendance API

**Get Attendance:**
```http
GET /api/v1/sessions/{sessionId}/attendance
Authorization: Bearer {token}
```

**Response:**
```json
{
  "sessionId": "session_xyz789",
  "totalEnrolled": 95,
  "present": 87,
  "late": 5,
  "absent": 3,
  "participants": [
    {
      "userId": "user_123",
      "name": "John Doe",
      "email": "john@university.edu",
      "joinTime": "2025-01-20T10:02:00Z",
      "status": "present",
      "duration": 88,
      "attentionScore": 92,
      "participationScore": 85
    }
  ]
}
```

### 3.3 Whiteboard API

**Create Drawing:**
```http
POST /api/v1/whiteboards/{whiteboardId}/draw
Content-Type: application/json

{
  "type": "path",
  "tool": "pen",
  "color": "#000000",
  "width": 2,
  "points": [[100, 200], [150, 250], [200, 220]]
}
```

### 3.4 Breakout Rooms API

**Create Rooms:**
```http
POST /api/v1/sessions/{sessionId}/breakout-rooms
Content-Type: application/json

{
  "count": 5,
  "duration": 15,
  "assignmentMethod": "automatic",
  "allowSelfSelect": false
}
```

**Response:**
```json
{
  "rooms": [
    {
      "roomId": "room_001",
      "name": "Group 1",
      "participants": ["user_123", "user_456", "user_789"]
    },
    {
      "roomId": "room_002",
      "name": "Group 2",
      "participants": ["user_234", "user_567", "user_890"]
    }
  ]
}
```


## 7. Integration

### 7.1 LMS Integration

**Canvas LTI Integration:**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<cartridge_basiclti_link xmlns="http://www.imsglobal.org/xsd/imslticc_v1p0">
  <title>WIA Virtual Classroom</title>
  <description>Virtual classroom integration</description>
  <launch_url>https://classroom.wia.org/lti/launch</launch_url>
  <secure_launch_url>https://classroom.wia.org/lti/launch</secure_launch_url>
</cartridge_basiclti_link>
```

### 7.2 Gradebook Sync

```http
POST /api/v1/lms/gradebook/sync
Content-Type: application/json

{
  "sessionId": "session_xyz789",
  "lmsType": "canvas",
  "courseId": "course_456",
  "assignmentId": "assignment_789",
  "grades": [
    {
      "userId": "user_123",
      "score": 95,
      "maxScore": 100
    }
  ]
}
```


## Appendix A: WebRTC Configuration

```javascript
const config = {
  iceServers: [
    { urls: 'stun:stun.wia.org:3478' },
    {
      urls: 'turn:turn.wia.org:3478',
      username: 'user',
      credential: 'pass'
    }
  ],
  iceTransportPolicy: 'all',
  bundlePolicy: 'max-bundle',
  rtcpMuxPolicy: 'require'
};
```

---

## Z.1 Audit transport and observability hooks (Phase 1)

Every Phase 1 envelope SHOULD emit a structured log line at the
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
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1)

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

## Z.3 Capabilities discovery and SemVer (Phase 1)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1)

Phase 1 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1)

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

## Z.6 Supply-chain envelope per SLSA (Phase 1)

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

## Z.1 Audit transport and observability hooks (Phase 1 (variant 1))

Every Phase 1 envelope SHOULD emit a structured log line at the
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
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 1 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1 (variant 1))

Phase 1 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 1 (variant 1))

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
