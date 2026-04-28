# WIA-SOC-001 — Phase 4: INTEGRATION

> Social Media Legacy canonical Phase 4 specification per the WIA Standards four-Phase architecture.

> Domain: 소셜 미디어 레거시 — 디지털 유산 · 사후 계정 · DSA · GDPR Art 17/20 · 디지털 추모.

## A.1 Scope

This Phase covers the canonical integration layer of the WIA-SOC-001 standard. It composes with the Phase 1 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## Security

- Death certificate verification
- Email-based authentication
- 30-day cooling-off period minimum


## Updates from v1.1

### Extended Cooling-Off Period

- Minimum 30 days, maximum 90 days (was 30-60)
- Risk-based extension for high-value accounts
- Family deliberation time consideration

### XML Archive Format

Added archival-standard XML export:
- METS compliance
- PREMIS metadata
- Dublin Core integration
- Designed for 50+ year preservation

### Enhanced Fraud Detection

- Machine learning anomaly detection
- Behavioral analysis of requestors
- Cross-platform verification hints
- Human review escalation protocols

### Platform Shutdown Protocols

- 90-day advance notification requirement
- Standard-format data export
- Migration assistance to surviving platforms
- URL preservation through redirects

### Accessibility Requirements

- Screen reader compatibility
- Multi-language interface support
- Keyboard navigation
- WCAG 2.1 AA compliance

---

**© 2024-2025 SmileStory Inc. / WIA**
**弘益人間 · Benefit All Humanity**


---

# WIA-LEG-007 Social Media Legacy Standard v2.0

**Version:** 2.0
**Release Date:** 2025-01-15
**Status:** Current
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---


## 3. Four-Phase Protocol

### 3.1 Phase 1: Preparation (Lifetime)

**Timeline:** Ongoing throughout user's life
**Actors:** Account holder, optional legacy contacts

#### 3.1.1 Required Features

Platforms MUST provide:

1. **Legacy Contact Designation**
   - Ability to designate 1-3 legacy contacts
   - Contact verification via email/SMS
   - Permission level configuration
   - Successor contact designation

2. **Memorialization Preferences**
   - Account action selection (memorialize/delete/archive)
   - Tribute page configuration
   - Visibility settings
   - Sunset date specification

3. **Data Export Configuration**
   - Format selection (JSON, CSV, XML)
   - Content scope definition
   - Delivery method specification
   - Scheduled automatic exports

4. **AI Memorial Bot Preferences**
   - Opt-in/opt-out toggle
   - Training data source selection
   - Access control configuration
   - Sunset date setting

#### 3.1.2 Configuration Data Format

```json
{
  "userId": "string",
  "version": "2.0",
  "lastUpdated": "ISO8601 timestamp",
  "legacyContacts": [
    {
      "contactId": "string",
      "name": "string",
      "email": "string",
      "phone": "string (E.164 format)",
      "permissions": ["view", "download", "manage_tribute", etc.],
      "priority": "integer (1-3)"
    }
  ],
  "preferences": {
    "action": "memorialize|delete|archive",
    "coolingOffPeriod": "integer (30-90 days)",
    "tributePage": {...},
    "dataExport": {...},
    "aiBot": {...},
    "deletionTrigger": {...}
  }
}
```

### 3.2 Phase 2: Verification (30-90 Days Post-Death)

**Timeline:** 30-90 days after death
**Actors:** Legacy contacts, platform verification teams

#### 3.2.1 Death Verification Requirements

Platforms MUST accept and verify:

1. **Primary Documentation**
   - Government-issued death certificate
   - Court order or probate documents
   - Executor authorization

2. **Verification Process**
   - Cryptographic hashing of submitted documents
   - Government database API queries where available
   - Multi-factor authentication of requestor
   - Cross-platform verification sharing

3. **Cooling-Off Period**
   - Minimum 30 days from death verification
   - Extendable to 90 days for complex cases
   - Fraud detection review during period
   - Family deliberation time

#### 3.2.2 Security Protocols

- Document authenticity checks
- Anomaly detection algorithms
- Human review for high-value accounts
- Appeal process for rejected requests

### 3.3 Phase 3: Execution (24-72 Hours Post-Verification)

**Timeline:** 24-72 hours after verification completes
**Actors:** Platform automation systems, legacy contacts

#### 3.3.1 Automated Actions

Platforms MUST execute:

1. Account status transition to memorialized/archived/deleted
2. Legacy contact notification with access credentials
3. Data export generation in specified formats
4. Tribute page creation (if memorialization chosen)
5. AI memorial bot training initiation (if opted-in)
6. Privacy settings update for memorial status

#### 3.3.2 Execution Log Format

```json
{
  "executionId": "string",
  "userId": "string",
  "timestamp": "ISO8601",
  "actions": [
    {
      "action": "string",
      "status": "completed|failed",
      "details": {...}
    }
  ],
  "errors": [],
  "completionStatus": "success|partial|failed"
}
```

### 3.4 Phase 4: Maintenance (10+ Years)

**Timeline:** Indefinite long-term preservation
**Actors:** Platform systems, legacy contacts, community

#### 3.4.1 Long-Term Requirements

- Legacy contact succession management
- Tribute page moderation
- AI bot interaction logging
- Data archive integrity verification
- Format migration as standards evolve
- Platform shutdown migration protocols

---


## 7. Privacy Compliance

### 7.1 GDPR Requirements

- Death exemption acknowledgment
- Third-party privacy protection
- Data minimization principles
- Right to be forgotten implementation

### 7.2 RUFADAA Compliance (US)

- User direction primacy
- Fiduciary access protocols
- Content vs communication distinction
- Terms of service harmonization

### 7.3 Cross-Jurisdictional Handling

**Priority Order:**
1. User's explicit jurisdiction choice
2. Primary residence at death
3. Citizenship/nationality
4. Platform headquarters jurisdiction
5. Most protective standard (if conflict)

---

---

## Z.1 Audit transport and observability hooks (Phase 4)

Every Phase 4 envelope SHOULD emit a structured log line at the
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
`4` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 4)

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

## Z.3 Capabilities discovery and SemVer (Phase 4)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 4)

Phase 4 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 4)

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

## Z.6 Supply-chain envelope per SLSA (Phase 4)

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

## Z.1 Audit transport and observability hooks (Phase 4 (variant 1))

Every Phase 4 envelope SHOULD emit a structured log line at the
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
`4` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 4 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 4 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 4 (variant 1))

Phase 4 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 4 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 4 (variant 1))

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
