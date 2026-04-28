# WIA-GOV-001 — Phase 2: API

> Smart Government canonical Phase 2 specification per the WIA Standards four-Phase architecture.

> Domain: 스마트 거버넌스 — 디지털 정부 · GovTech · eIDAS · 시민 디지털 신원 · 개방데이터.

## A.1 Scope

This Phase covers the canonical api layer of the WIA-GOV-001 standard. It composes with the Phase 3 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- ISO/IEC 38505 (Governance of data)
- ISO/IEC 27001:2022
- EU eIDAS Regulation (EU) 910/2014 + Reg 2024/1183 (eIDAS 2)
- UN E-Government Survey 2024
- World Bank GovTech Maturity Index 2024
- OECD Recommendation on Digital Government 2014
- Open Government Partnership (OGP) Action Plans
- W3C DCAT 3.0 + DCAT-AP 3.0
- OpenAPI Specification 3.1

## Abstract

The WIA Smart Government Standard (WIA-SOC-006) defines a comprehensive technical framework for implementing AI-powered government services. This specification establishes protocols, data formats, API interfaces, security requirements, and integration patterns to enable public institutions to deliver intelligent, efficient, and citizen-centric digital services.

**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

---


## 4. API Endpoints

### 4.1 Service Management API

**Base URL:** `https://api.gov.example/v1`

#### Create Service Request

```
POST /services/requests
Authorization: Bearer {token}
Content-Type: application/json

{
  "serviceType": "building_permit",
  "formData": {...},
  "documents": [...]
}

Response: 201 Created
{
  "requestId": "REQ-2025-123456",
  "status": "submitted",
  "estimatedCompletionDate": "2025-12-30"
}
```

#### Get Request Status

```
GET /services/requests/{requestId}
Authorization: Bearer {token}

Response: 200 OK
{
  "requestId": "REQ-2025-123456",
  "status": "under_review",
  "currentStep": "department_approval",
  "progress": 60,
  "updates": [...]
}
```

#### List Available Services

```
GET /services/catalog
Query: ?category=permits&language=en

Response: 200 OK
{
  "services": [
    {
      "serviceId": "building_permit",
      "name": "Building Permit Application",
      "category": "permits",
      "description": "...",
      "requirements": [...],
      "fee": 150.00,
      "averageProcessingTime": "5 days"
    }
  ]
}
```

### 4.2 AI Assistant API

#### Create Chat Session

```
POST /ai/sessions
Authorization: Bearer {token}

Response: 201 Created
{
  "sessionId": "SESSION-ABC123",
  "expiresAt": "2025-12-27T18:00:00Z"
}
```

#### Send Message

```
POST /ai/sessions/{sessionId}/messages
Content-Type: application/json

{
  "message": "How do I apply for a building permit?",
  "language": "en"
}

Response: 200 OK
{
  "messageId": "MSG-456",
  "response": "To apply for a building permit, you'll need...",
  "suggestions": [
    "Start building permit application",
    "View requirements",
    "Estimate processing time"
  ],
  "detectedIntent": "inquiry_building_permit",
  "confidence": 0.95
}
```

### 4.3 Analytics API

#### Get Dashboard Metrics

```
GET /analytics/metrics
Query: ?department=all&period=30d

Response: 200 OK
{
  "totalRequests": 12547,
  "automationRate": 0.94,
  "avgProcessingTime": 2.5,
  "citizenSatisfaction": 0.97,
  "breakdown": {...}
}
```

#### Predictive Analytics

```
POST /analytics/predict
Content-Type: application/json

{
  "model": "service_demand",
  "timeframe": "next_30_days",
  "parameters": {...}
}

Response: 200 OK
{
  "predictions": [
    {
      "date": "2025-01-15",
      "serviceType": "building_permit",
      "expectedVolume": 245,
      "confidence": 0.87
    }
  ]
}
```

---


## 8. Performance Requirements

### 8.1 System Performance

- **API Response Time:** < 200ms (p95)
- **Page Load Time:** < 2 seconds
- **Concurrent Users:** Support 100,000+ simultaneous users
- **Uptime:** 99.9% availability (8.76 hours downtime/year)
- **Scalability:** Auto-scaling to handle 10x normal load

### 8.2 AI Performance

- **Chatbot Response:** < 2 seconds for simple queries
- **Complex Analysis:** < 10 seconds
- **Document Processing:** < 30 seconds per document
- **Batch Processing:** 1000+ documents per hour

### 8.3 Data Processing

- **Database Queries:** < 100ms (p95)
- **Backup:** Daily incremental, weekly full
- **Recovery Time Objective (RTO):** < 4 hours
- **Recovery Point Objective (RPO):** < 15 minutes

---


## 12. Compliance & Certification

### 12.1 WIA Certification Levels

**Bronze:** Core API compliance, basic security
**Silver:** AI assistance, automation, enhanced security
**Gold:** Smart city integration, predictive analytics, blockchain
**Platinum:** Full compliance, innovation, best practices sharing

### 12.2 Audit Requirements

- **Annual compliance audit**
- **Quarterly security reviews**
- **Monthly performance reports**
- **Continuous monitoring dashboards**

---

---

## Z.1 Audit transport and observability hooks (Phase 2)

Every Phase 2 envelope SHOULD emit a structured log line at the
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
with `wia.standard.slug` = `smart-government` and `wia.standard.phase` =
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2)

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

## Z.3 Capabilities discovery and SemVer (Phase 2)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-smart-government-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2)

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2)

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

## Z.6 Supply-chain envelope per SLSA (Phase 2)

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

## Z.1 Audit transport and observability hooks (Phase 2 (variant 1))

Every Phase 2 envelope SHOULD emit a structured log line at the
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
with `wia.standard.slug` = `smart-government` and `wia.standard.phase` =
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 2 (variant 1))

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-smart-government-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2 (variant 1))

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 2 (variant 1))

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
