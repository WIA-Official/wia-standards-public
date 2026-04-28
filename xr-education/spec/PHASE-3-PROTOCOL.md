# WIA-EDU-013 — Phase 3: PROTOCOL

> XR Education canonical Phase 3 specification per the WIA Standards four-Phase architecture.

> Domain: XR 교육 — 확장현실 교육 · VR/AR/MR · OpenXR · WebXR · 접근성.

## A.1 Scope

This Phase covers the canonical protocol layer of the WIA-EDU-013 standard. It composes with the Phase 4 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- Khronos OpenXR 1.0
- W3C WebXR Device API
- Khronos glTF 2.0 / KTX 2.0
- ISO/IEC 18039 (Mixed Augmented Reality)
- IEEE 2888 (Cyber Real Interfaces)
- WCAG 2.2 + XR Accessibility User Requirements
- IEEE 9274.1.1 xAPI
- ANSI Z136 Laser Safety
- EN 50566 (Wearable display SAR)

## 1. Abstract

The WIA-EDU-013 XR Education Standard defines interfaces, protocols, and best practices for implementing Extended Reality (VR/AR/MR) systems in educational contexts. This standard covers immersive learning environments, virtual laboratories, 3D content creation, spatial computing, accessibility features, and learning analytics for XR experiences.

**弘益人間 (홍익인간)** - This standard embodies the principle of broadly benefiting humanity through accessible, immersive education technology that transcends physical and geographical boundaries.

---


## 5. Virtual Learning Environment

### 5.1 Virtual Classroom

```typescript
interface VirtualClassroom {
  classroomId: string;
  capacity: number;

  // Spatial configuration
  dimensions: {
    width: number;      // meters
    length: number;     // meters
    height: number;     // meters
  };

  // Features
  features: {
    spatialAudio: boolean;
    whiteboard3D: boolean;
    screenSharing: boolean;
    handRaising: boolean;
    breakoutRooms: boolean;
  };

  // Participants
  teacher: Participant;
  students: Participant[];

  // Content
  assets: LearningAsset[];
}

interface Participant {
  userId: string;
  avatar: AvatarConfig;
  position: Vector3;
  rotation: Quaternion;
  role: 'teacher' | 'student' | 'observer';
  permissions: Permission[];
}
```

### 5.2 Avatar System

- Customizable avatars with facial tracking (optional)
- Hand and finger tracking for gestures
- Lip sync for speech
- Accessibility options (simplified avatars, reduced motion)

---


## 9. Accessibility Requirements

### 9.1 Mandatory Accessibility Features

```typescript
interface XRAccessibility {
  // Visual accommodations
  visual: {
    colorblindModes: ('deuteranopia' | 'protanopia' | 'tritanopia')[];
    highContrast: boolean;
    textToSpeech: boolean;
    subtitles: boolean;
    fontSize: 'small' | 'medium' | 'large' | 'xlarge';
  };

  // Motor accommodations
  motor: {
    seatedMode: boolean;
    reduceReach: boolean;
    snapTurning: boolean;          // vs smooth rotation
    teleportMovement: boolean;      // vs locomotion
    oneHandedMode: boolean;
  };

  // Comfort settings
  comfort: {
    vignette: number;               // 0-1, reduce motion sickness
    reducedMotion: boolean;
    fieldOfViewReduction: number;   // 0-1
  };

  // Cognitive support
  cognitive: {
    simplifiedUI: boolean;
    guidedMode: boolean;
    pauseAnytime: boolean;
  };
}
```

### 9.2 WCAG XR Compliance

- Follow WCAG 2.1 AAA where applicable
- XR-specific guidelines for motion sickness prevention
- Alternative input methods for all interactions
- Adjustable difficulty and pacing

---


## 13. Platform Interoperability

### 13.1 Cross-Platform Support

The standard MUST enable content to run on:
- Desktop VR (PC-based headsets)
- Standalone VR (Quest, Pico)
- Mobile AR (iOS, Android)
- WebXR (browsers)

### 13.2 Export Formats

- **Universal Scene Description (USD)** for scene interchange
- **glTF 2.0** for 3D models
- **WebXR** for web deployment
- **OpenXR** for runtime compatibility

---

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
with `wia.standard.slug` = `xr-education` and `wia.standard.phase` =
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
`/.well-known/wia-xr-education-capabilities` enumerating per-endpoint
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
with `wia.standard.slug` = `xr-education` and `wia.standard.phase` =
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
`/.well-known/wia-xr-education-capabilities` enumerating per-endpoint
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
