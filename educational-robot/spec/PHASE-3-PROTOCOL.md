# WIA-EDU-007: Educational Robot Standard
## Phase 3: Protocol Specification

**Version:** 1.2.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-25
**Category:** Education/Culture (EDU)

---

## 1. Overview

### 1.1 Purpose
This specification defines the communication protocols for real-time interaction between educational robots, students, teachers, and learning systems. It covers WebSocket streaming, message formats, security protocols, and state management.

### 1.2 Philosophy
**弘益人間 (Benefit All Humanity)** - Real-time, responsive educational interactions create engaging learning experiences that benefit all students, regardless of their learning pace or style.

### 1.3 Protocol Stack

```
┌─────────────────────────────────┐
│   Application Layer (JSON-LD)   │
├─────────────────────────────────┤
│   WebSocket Protocol (RFC 6455) │
├─────────────────────────────────┤
│   TLS 1.3 Encryption            │
├─────────────────────────────────┤
│   TCP/IP                        │
└─────────────────────────────────┘
```

---

## 2. Connection Establishment

### 2.1 WebSocket Handshake

#### Connection URL
```
wss://stream.wiastandards.com/edu-robot/v1
```

#### Handshake Request
```http
GET /edu-robot/v1 HTTP/1.1
Host: stream.wiastandards.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer <jwt_token>
WIA-Robot-ID: EDU-ROBOT-2025-001
WIA-Session-ID: SESSION-2025-12345
```

#### Handshake Response
```http
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
WIA-Protocol-Version: 1.2.0
```

### 2.2 Authentication
All WebSocket connections require a valid JWT token obtained via OAuth 2.0 (see Phase 2).

### 2.3 Connection Lifecycle

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│ Connect  │───▶│  Auth    │───▶│  Active  │───▶│  Close   │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
     │               │               │               │
     │               │               ▼               │
     │               │          ┌──────────┐        │
     │               └─────────▶│  Reject  │────────┘
     │                          └──────────┘
     ▼
  Timeout
```

---

## 3. Message Format

### 3.1 Base Message Structure

All messages use JSON-LD format:

```json
{
  "@context": "https://wiastandards.com/contexts/edu-robot/message/v1",
  "@type": "RobotMessage",
  "messageId": "msg-<uuid>",
  "timestamp": "ISO8601",
  "sessionId": "SESSION-2025-12345",
  "robotId": "EDU-ROBOT-2025-001",
  "messageType": "enum",
  "payload": {...},
  "metadata": {...}
}
```

### 3.2 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `instruction` | Robot → Student | Teaching content delivery |
| `question` | Robot → Student | Assessment query |
| `feedback` | Robot → Student | Performance feedback |
| `response` | Student → Robot | Student answer/input |
| `emotion` | Student → Robot | Emotional state update |
| `gesture` | Student → Robot | Physical gesture input |
| `event` | Bidirectional | System events |
| `heartbeat` | Bidirectional | Keep-alive ping |
| `error` | Bidirectional | Error notification |

---

## 4. Communication Flows

### 4.1 Instruction Delivery

#### Robot Sends Instruction
```json
{
  "@context": "https://wiastandards.com/contexts/edu-robot/message/v1",
  "@type": "RobotMessage",
  "messageId": "msg-abc123",
  "timestamp": "2025-12-25T10:15:00Z",
  "sessionId": "SESSION-2025-12345",
  "robotId": "EDU-ROBOT-2025-001",
  "messageType": "instruction",
  "payload": {
    "instructionType": "concept-explanation",
    "subjectArea": "mathematics",
    "topic": "multiplication",
    "content": {
      "text": "Let's learn about multiplication. When we multiply 3 × 4, we're making 3 groups of 4.",
      "audio": "https://cdn.wia.com/audio/mult-intro.mp3",
      "visual": {
        "type": "animation",
        "url": "https://cdn.wia.com/visuals/mult-groups.mp4",
        "duration": 15
      },
      "subtitles": [
        {"lang": "en", "text": "..."},
        {"lang": "es", "text": "..."}
      ]
    },
    "interactionPrompt": {
      "text": "Can you count how many objects are in all 3 groups?",
      "expectedResponseType": "number",
      "hints": ["Count each group", "Add them together"]
    }
  },
  "metadata": {
    "difficulty": "beginner",
    "bloomLevel": "BL2",
    "estimatedDuration": 30
  }
}
```

### 4.2 Student Response

#### Student Sends Response
```json
{
  "@context": "https://wiastandards.com/contexts/edu-robot/message/v1",
  "@type": "StudentMessage",
  "messageId": "msg-xyz789",
  "timestamp": "2025-12-25T10:15:45Z",
  "sessionId": "SESSION-2025-12345",
  "messageType": "response",
  "payload": {
    "responseType": "voice",
    "content": "I counted 12 objects!",
    "confidence": 0.95,
    "responseTime": 45,
    "emotion": {
      "primary": "excited",
      "secondary": ["confident"],
      "valence": 0.8,
      "arousal": 0.7
    }
  },
  "metadata": {
    "inputModality": "voice",
    "processingTime": 0.8
  }
}
```

#### Robot Sends Feedback
```json
{
  "@type": "RobotMessage",
  "messageType": "feedback",
  "payload": {
    "feedbackType": "positive-reinforcement",
    "isCorrect": true,
    "content": {
      "text": "Excellent work! You're absolutely right - 3 groups of 4 equals 12!",
      "audio": "https://cdn.wia.com/audio/praise-excellent.mp3",
      "gesture": "thumbs-up",
      "facialExpression": "proud-smile"
    },
    "nextAction": {
      "type": "progression",
      "difficulty": "intermediate",
      "nextTopic": "multiplication-tables"
    },
    "learningAnalytics": {
      "skillDemonstrated": "single-digit-multiplication",
      "masteryProgress": 0.75,
      "recommendedPractice": false
    }
  }
}
```

---

## 5. Real-Time State Synchronization

### 5.1 Session State

#### State Update Message
```json
{
  "@type": "RobotMessage",
  "messageType": "event",
  "payload": {
    "eventType": "state-update",
    "state": {
      "sessionStatus": "active",
      "currentActivity": "multiplication-practice",
      "progress": {
        "completedSteps": 3,
        "totalSteps": 8,
        "percentage": 37.5
      },
      "studentEngagement": {
        "score": 87,
        "trend": "increasing",
        "attentionLevel": "focused"
      },
      "adaptations": {
        "difficultyAdjusted": false,
        "paceModified": false,
        "modalityShifted": false
      }
    }
  }
}
```

### 5.2 Emotion Tracking

#### Emotion Update (Student → Robot)
```json
{
  "@type": "StudentMessage",
  "messageType": "emotion",
  "payload": {
    "emotionDetection": {
      "method": "facial-recognition",
      "emotions": [
        {"type": "frustrated", "confidence": 0.72},
        {"type": "confused", "confidence": 0.65}
      ],
      "valence": -0.3,
      "arousal": 0.6,
      "timestamp": "2025-12-25T10:20:15Z"
    },
    "triggers": [
      {"type": "difficulty-spike", "timestamp": "2025-12-25T10:19:45Z"}
    ]
  }
}
```

#### Adaptive Response (Robot → Student)
```json
{
  "@type": "RobotMessage",
  "messageType": "instruction",
  "payload": {
    "instructionType": "adaptive-intervention",
    "adaptation": {
      "trigger": "negative-emotion-detected",
      "response": "difficulty-reduction",
      "empathy": {
        "text": "I can see this is challenging. Let's take a step back and try an easier example together.",
        "tone": "supportive",
        "gesture": "reassuring-pat"
      }
    },
    "content": {
      "text": "Let's practice with smaller numbers first...",
      "difficulty": "beginner"
    }
  }
}
```

---

## 6. Multimodal Interaction Protocol

### 6.1 Supported Modalities

| Modality | Input | Output | Latency Target |
|----------|-------|--------|----------------|
| **Voice** | Speech-to-Text | Text-to-Speech | <500ms |
| **Touch** | Touch coordinates | Haptic feedback | <100ms |
| **Gesture** | Skeleton tracking | Visual cues | <200ms |
| **Visual** | Eye tracking | Display output | <50ms |
| **Emotion** | Facial analysis | Expressive response | <300ms |

### 6.2 Voice Interaction

#### Voice Input Message
```json
{
  "@type": "StudentMessage",
  "messageType": "response",
  "payload": {
    "responseType": "voice",
    "audio": {
      "format": "webm",
      "sampleRate": 16000,
      "channels": 1,
      "duration": 3.5,
      "base64Data": "data:audio/webm;base64,..."
    },
    "transcription": {
      "text": "The answer is twelve",
      "confidence": 0.92,
      "language": "en-US",
      "alternatives": [
        {"text": "The answer is 12", "confidence": 0.90}
      ]
    }
  }
}
```

### 6.3 Gesture Interaction

#### Gesture Input Message
```json
{
  "@type": "StudentMessage",
  "messageType": "gesture",
  "payload": {
    "gestureType": "hand-raise",
    "confidence": 0.88,
    "duration": 2.5,
    "timestamp": "2025-12-25T10:22:00Z",
    "interpretation": "student-has-question",
    "skeleton": {
      "format": "mediapipe",
      "landmarks": [...]
    }
  }
}
```

---

## 7. Safety Protocols

### 7.1 Content Filtering

#### Inappropriate Content Detection
```json
{
  "@type": "RobotMessage",
  "messageType": "error",
  "payload": {
    "errorType": "content-policy-violation",
    "severity": "high",
    "action": "block",
    "details": {
      "violationType": "inappropriate-language",
      "detectedContent": "[redacted]",
      "timestamp": "2025-12-25T10:25:00Z"
    },
    "response": {
      "text": "I'm sorry, but I can't respond to that. Let's focus on our lesson.",
      "tone": "gentle-redirect",
      "teacherNotification": true
    }
  }
}
```

### 7.2 Emergency Stop Protocol

#### Emergency Stop Command
```json
{
  "@type": "SystemMessage",
  "messageType": "event",
  "payload": {
    "eventType": "emergency-stop",
    "trigger": "teacher-intervention",
    "timestamp": "2025-12-25T10:30:00Z",
    "action": "immediate-halt",
    "sessionSave": true,
    "notification": {
      "recipients": ["teacher", "admin"],
      "urgency": "critical"
    }
  }
}
```

### 7.3 Privacy Protection

#### Data Anonymization
All student identifiers must be anonymized:
- Student ID → Hash
- Name → Pseudonym
- Voice → Anonymized transcription
- Face → Emotion vectors only (no images stored)

---

## 8. Error Handling

### 8.1 Connection Errors

| Error Code | Description | Recovery Action |
|------------|-------------|-----------------|
| `1000` | Normal closure | None |
| `1001` | Going away | Reconnect |
| `1002` | Protocol error | Check message format |
| `1003` | Unsupported data | Check data type |
| `1006` | Abnormal closure | Reconnect with backoff |
| `4000` | Auth failure | Re-authenticate |
| `4001` | Session expired | Start new session |
| `4002` | Rate limit | Wait & retry |

### 8.2 Error Message Format

```json
{
  "@type": "ErrorMessage",
  "messageType": "error",
  "payload": {
    "errorCode": "4001",
    "errorType": "session-expired",
    "message": "Learning session has expired",
    "details": "Session SESSION-2025-12345 exceeded maximum duration of 120 minutes",
    "timestamp": "2025-12-25T12:00:00Z",
    "recoveryAction": {
      "type": "restart-session",
      "autoRetry": false,
      "userPrompt": "Would you like to start a new session?"
    }
  }
}
```

---

## 9. Heartbeat & Keep-Alive

### 9.1 Heartbeat Interval
- Client → Server: Every 30 seconds
- Server → Client: Every 45 seconds

### 9.2 Heartbeat Message

#### Ping (Client → Server)
```json
{
  "@type": "HeartbeatMessage",
  "messageType": "heartbeat",
  "payload": {
    "action": "ping",
    "clientTime": "2025-12-25T10:30:00.123Z",
    "sessionActive": true
  }
}
```

#### Pong (Server → Client)
```json
{
  "@type": "HeartbeatMessage",
  "messageType": "heartbeat",
  "payload": {
    "action": "pong",
    "serverTime": "2025-12-25T10:30:00.125Z",
    "latency": 2,
    "sessionStatus": "active"
  }
}
```

---

## 10. Quality of Service (QoS)

### 10.1 Latency Requirements

| Message Type | Target Latency | Maximum Latency |
|--------------|----------------|-----------------|
| Voice response | <500ms | 1000ms |
| Touch feedback | <100ms | 200ms |
| Visual update | <50ms | 100ms |
| Emotion response | <300ms | 600ms |
| State sync | <200ms | 500ms |

### 10.2 Bandwidth Management

| Priority | Message Type | Bandwidth Allocation |
|----------|--------------|---------------------|
| **P0** | Emergency stop | Unlimited |
| **P1** | Real-time interaction | 80% |
| **P2** | State synchronization | 15% |
| **P3** | Analytics logging | 5% |

---

## 11. Versioning & Compatibility

### 11.1 Protocol Versioning

Header format:
```http
WIA-Protocol-Version: 1.2.0
```

### 11.2 Backward Compatibility

The protocol maintains backward compatibility for one major version:
- v1.2 clients can connect to v1.x servers
- v2.0 will support v1.x for 12 months

### 11.3 Feature Negotiation

```json
{
  "@type": "HandshakeMessage",
  "messageType": "event",
  "payload": {
    "eventType": "capability-negotiation",
    "clientCapabilities": [
      "voice-input",
      "gesture-recognition",
      "emotion-detection"
    ],
    "serverCapabilities": [
      "voice-input",
      "gesture-recognition",
      "emotion-detection",
      "haptic-feedback"
    ],
    "agreedCapabilities": [
      "voice-input",
      "gesture-recognition",
      "emotion-detection"
    ]
  }
}
```

---

## 12. Testing & Validation

### 12.1 Test Environment

```
wss://test-stream.wiastandards.com/edu-robot/v1
```

### 12.2 Test Credentials

```
Robot ID: EDU-ROBOT-TEST-001
Session ID: SESSION-TEST-12345
JWT Token: test_token_abc123xyz789
```

### 12.3 Message Validation

All messages are validated against JSON Schema:
```
https://wiastandards.com/schemas/edu-robot/message-v1.2.json
```

---

## 13. Monitoring & Diagnostics

### 13.1 Connection Metrics

```json
{
  "@type": "DiagnosticsMessage",
  "messageType": "event",
  "payload": {
    "eventType": "connection-metrics",
    "metrics": {
      "latency": {
        "current": 45,
        "average": 52,
        "p95": 78,
        "p99": 120
      },
      "throughput": {
        "sent": 1250,
        "received": 980,
        "unit": "bytes/sec"
      },
      "messageStats": {
        "sent": 145,
        "received": 132,
        "errors": 2
      },
      "uptime": 1850
    }
  }
}
```

---

## 14. Conformance

A system is conformant with WIA-EDU-007 Phase 3 if it:

✅ Implements WebSocket protocol (RFC 6455)
✅ Uses TLS 1.3 encryption
✅ Sends JSON-LD formatted messages
✅ Supports all core message types
✅ Implements heartbeat mechanism
✅ Handles errors gracefully
✅ Maintains latency SLAs
✅ Implements safety protocols
✅ Supports multimodal interaction
✅ Provides connection diagnostics

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*


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

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
