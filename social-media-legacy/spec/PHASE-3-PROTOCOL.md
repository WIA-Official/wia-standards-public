# WIA-SOC-001 — Phase 3: PROTOCOL

> Social Media Legacy canonical Phase 3 specification per the WIA Standards four-Phase architecture.

> Domain: 소셜 미디어 레거시 — 디지털 유산 · 사후 계정 · DSA · GDPR Art 17/20 · 디지털 추모.

## A.1 Scope

This Phase covers the canonical protocol layer of the WIA-SOC-001 standard. It composes with the Phase 4 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- EU Digital Services Act (Reg 2022/2065)
- GDPR Art 17 (Right to erasure) + Art 20 (Portability)
- GDPR Art 89 (Archival in public interest)
- ePrivacy Directive 2002/58/EC
- Library of Congress Web Archive standards
- IPTC NewsML-G2 Section 5.5 archival
- ISO 14721:2012 OAIS Reference Model
- Estate-of-deceased per US Uniform Probate Code
- Korea Personal Information Protection Act (PIPA)

## Core Features

### Four-Phase Protocol

1. **Preparation:** Lifetime legacy planning
2. **Verification:** Death confirmation (30-60 days)
3. **Execution:** Preference implementation
4. **Maintenance:** Long-term preservation

### Legacy Contact System

- Designate up to 2 legacy contacts
- Basic permissions: view and download
- Email verification required

### Data Export

- JSON format only
- Includes: posts, photos, videos, basic metadata
- Manual request process

### Memorialization

- Account conversion to memorial status
- Basic tribute wall functionality
- Friends-only or public visibility


## Updates from v1.0

### Enhanced Legacy Contact Permissions

- Increased to 3 legacy contacts maximum
- Granular permissions: view, download, manage_tribute, post_memorial
- Permission levels per contact
- Successor contact designation

### CSV Export Format

Added CSV export alongside JSON for improved accessibility:
- posts.csv
- photos.csv
- friends.csv
- README.txt

### Improved Tribute Pages

- Moderation controls (auto-approve, review-required, restricted)
- Rich media support in tributes
- Charitable donation integration
- Anniversary notification system

### Third-Party Privacy Protection

- Explicit opt-in for message access
- Correspondent notification system
- Redaction options for sensitive content

---

**© 2023-2025 SmileStory Inc. / WIA**
**弘益人間 · Benefit All Humanity**


---

# WIA-LEG-007 Social Media Legacy Standard v1.2

**Version:** 1.2
**Release Date:** 2024-06-20
**Status:** Superseded by v2.0
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---


## 2. Scope

### 2.1 Covered Platforms

This standard applies to all social media platforms including but not limited to:
- Social networks (Facebook, Instagram, Twitter/X, LinkedIn)
- Video platforms (YouTube, TikTok, Vimeo)
- Photo sharing (Pinterest, Flickr)
- Messaging apps with social features (WhatsApp, Telegram)
- Professional networks (LinkedIn, GitHub)

### 2.2 Covered Content Types

- Text posts and status updates
- Photos and videos
- Comments and reactions
- Private messages (with consent)
- Relationships and connections
- Groups and communities
- Events and check-ins

### 2.3 Out of Scope

- Financial accounts and banking platforms
- E-commerce accounts (covered by WIA-ECOM-*)
- Healthcare platforms (covered by WIA-HEALTH-*)
- Government services
- Corporate/business accounts (separate guidelines apply)

---


## 6. Security Requirements

### 6.1 Authentication

- Multi-factor authentication for legacy contacts
- Biometric verification for high-value accounts
- Time-based one-time passwords (TOTP)
- Hardware security key support

### 6.2 Document Verification

- Cryptographic hashing (SHA-256 minimum)
- Government database API integration
- Optical character recognition (OCR) for validation
- Blockchain timestamping for immutability

### 6.3 Fraud Detection

- Machine learning anomaly detection
- Behavioral analysis of requestors
- Cross-reference with known fraud patterns
- Human review escalation for suspicious cases

### 6.4 Data Encryption

- TLS 1.3 for data in transit
- AES-256 for data at rest
- End-to-end encryption for sensitive exports
- Secure key management (HSM or cloud KMS)

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
with `wia.standard.slug` = `social-media-legacy` and `wia.standard.phase` =
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
`/.well-known/wia-social-media-legacy-capabilities` enumerating per-endpoint
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
with `wia.standard.slug` = `social-media-legacy` and `wia.standard.phase` =
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
`/.well-known/wia-social-media-legacy-capabilities` enumerating per-endpoint
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
