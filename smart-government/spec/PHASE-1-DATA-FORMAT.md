# WIA-GOV-001 — Phase 1: DATA-FORMAT

> Smart Government canonical Phase 1 specification per the WIA Standards four-Phase architecture.

> Domain: 스마트 거버넌스 — 디지털 정부 · GovTech · eIDAS · 시민 디지털 신원 · 개방데이터.

## A.1 Scope

This Phase covers the canonical data-format layer of the WIA-GOV-001 standard. It composes with the Phase 2 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## # WIA Smart Government Standard Specification

**Version:** 1.0.0
**Standard ID:** WIA-SOC-006
**Category:** Society (SOC)
**Status:** Active
**Published:** December 27, 2025

---


## 3. Data Formats

### 3.1 Citizen Profile

```json
{
  "citizenId": "string",
  "personalInfo": {
    "firstName": "string",
    "lastName": "string",
    "dateOfBirth": "ISO-8601",
    "nationality": "string"
  },
  "contactInfo": {
    "email": "string",
    "phone": "string",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "postalCode": "string",
      "country": "string"
    }
  },
  "identityVerification": {
    "method": "biometric|document|blockchain",
    "verifiedAt": "ISO-8601",
    "trustLevel": "low|medium|high|verified"
  },
  "preferences": {
    "language": "string",
    "notificationChannels": ["email", "sms", "push"],
    "accessibility": {
      "screenReader": "boolean",
      "largeText": "boolean",
      "colorBlindMode": "string"
    }
  }
}
```

### 3.2 Service Request

```json
{
  "requestId": "string",
  "citizenId": "string",
  "serviceType": "permit|license|certificate|benefit|complaint",
  "serviceCategory": "string",
  "priority": "low|normal|high|urgent",
  "status": "submitted|under_review|approved|rejected|completed",
  "submittedAt": "ISO-8601",
  "updatedAt": "ISO-8601",
  "completedAt": "ISO-8601",
  "documents": [
    {
      "documentId": "string",
      "type": "string",
      "name": "string",
      "url": "string",
      "uploadedAt": "ISO-8601",
      "verified": "boolean",
      "verificationMethod": "ocr|manual|blockchain"
    }
  ],
  "formData": {
    "field1": "value1",
    "field2": "value2"
  },
  "payment": {
    "amount": "number",
    "currency": "string",
    "status": "pending|completed|failed|refunded",
    "transactionId": "string"
  },
  "workflow": {
    "currentStep": "string",
    "totalSteps": "number",
    "approvals": [
      {
        "department": "string",
        "approvedBy": "string",
        "approvedAt": "ISO-8601",
        "comments": "string"
      }
    ]
  }
}
```

### 3.3 AI Assistant Interaction

```json
{
  "sessionId": "string",
  "citizenId": "string",
  "messages": [
    {
      "messageId": "string",
      "role": "user|assistant|system",
      "content": "string",
      "timestamp": "ISO-8601",
      "language": "string",
      "intent": {
        "detected": "string",
        "confidence": "number",
        "entities": [
          {
            "type": "string",
            "value": "string",
            "confidence": "number"
          }
        ]
      }
    }
  ],
  "context": {
    "department": "string",
    "currentService": "string",
    "history": []
  },
  "metadata": {
    "channel": "web|mobile|voice|kiosk",
    "device": "string",
    "location": "string"
  }
}
```

### 3.4 Analytics Event

```json
{
  "eventId": "string",
  "eventType": "service_request|citizen_interaction|system_event",
  "timestamp": "ISO-8601",
  "source": {
    "department": "string",
    "system": "string",
    "userId": "string"
  },
  "data": {
    "metric": "string",
    "value": "number|string|object",
    "dimensions": {}
  },
  "tags": ["string"]
}
```

---


## 7. AI Model Requirements

### 7.1 Natural Language Processing

**Minimum Requirements:**
- **Languages:** Support for at least 3 languages
- **Intent recognition:** 90%+ accuracy
- **Entity extraction:** 85%+ accuracy
- **Context understanding:** Multi-turn conversations
- **Sentiment analysis:** Detect citizen frustration

**Recommended Models:**
- BERT-based models for understanding
- GPT-based models for generation
- Custom fine-tuning on government domain

### 7.2 Document Processing

**OCR Requirements:**
- **Accuracy:** 98%+ for printed text
- **Accuracy:** 95%+ for handwritten text
- **Supported formats:** PDF, JPG, PNG, TIFF
- **Languages:** Multi-language support
- **Features:** Layout preservation, table extraction

**Classification:**
- **Document type classification:** 95%+ accuracy
- **Automated field extraction:** 90%+ accuracy
- **Signature detection:** 98%+ accuracy
- **Fraud detection:** Anomaly detection enabled

### 7.3 Predictive Analytics

**Capabilities:**
- Time series forecasting (service demand)
- Population trend prediction
- Resource optimization
- Risk assessment
- Anomaly detection (fraud, errors)

**Evaluation Metrics:**
- **Accuracy:** MAPE < 10% for forecasts
- **Precision/Recall:** > 90% for classification
- **Model explainability:** SHAP or LIME integration

---


## 11. Implementation Guidelines

### 11.1 Phased Rollout

**Phase 1: Foundation (Months 1-3)**
- Infrastructure setup
- Data migration from legacy systems
- Basic API implementation
- Staff training

**Phase 2: Pilot (Months 4-6)**
- Launch AI chatbot for 2-3 services
- Automated processing for high-volume applications
- Limited user testing
- Iterative improvements

**Phase 3: Expansion (Months 7-12)**
- Scale to all services
- Smart city integration
- Mobile app launch
- Public awareness campaign

**Phase 4: Optimization (Ongoing)**
- Continuous AI model improvement
- Feature additions based on feedback
- Performance optimization
- Inter-agency collaboration

### 11.2 Change Management

- **Stakeholder engagement:** Regular updates to leadership
- **Staff training:** Comprehensive training programs
- **Citizen education:** Tutorials, help guides, support
- **Feedback loops:** Continuous improvement based on usage data

---

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
with `wia.standard.slug` = `smart-government` and `wia.standard.phase` =
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
with `wia.standard.slug` = `smart-government` and `wia.standard.phase` =
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
