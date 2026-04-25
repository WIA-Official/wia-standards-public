# WIA-SEC-016: Intrusion Detection Standard

# PHASE 4: ADVANCED FEATURES

## 5. Threat Intelligence Integration

### 5.1 Threat Feed Ingestion
**Supported Formats:**
- STIX 2.1 (Structured Threat Information Expression)
- TAXII 2.1 (Trusted Automated eXchange of Indicator Information)
- OpenIOC (Open Indicators of Compromise)
- MISP (Malware Information Sharing Platform)

**STIX Indicator Example:**
```json
{
  "type": "indicator",
  "spec_version": "2.1",
  "id": "indicator--8e2e2d2b-17d4-4cbf-938f-98ee46b3cd3f",
  "created": "2025-12-25T10:00:00.000Z",
  "modified": "2025-12-25T10:00:00.000Z",
  "name": "Malicious IP - Emotet C2",
  "pattern": "[ipv4-addr:value = '203.0.113.45']",
  "pattern_type": "stix",
  "valid_from": "2025-12-25T10:00:00.000Z",
  "indicator_types": ["malicious-activity"]
}
```

### 5.2 IP Reputation
**Reputation Sources:**
- Commercial feeds (Talos, Proofpoint ET Intelligence)
- Open-source feeds (Abuse.ch, AlienVault OTX)
- Internal reputation (learned from past incidents)

**Scoring:**
- Score range: 0 (benign) - 100 (malicious)
- Thresholds:
  - 0-30: Low risk (log only)
  - 31-70: Medium risk (alert)
  - 71-100: High risk (block in IPS mode)

**Enrichment:**
```json
{
  "ip": "203.0.113.45",
  "reputation_score": 85,
  "categories": ["malware-c2", "phishing"],
  "first_seen": "2025-11-20T00:00:00Z",
  "last_seen": "2025-12-25T09:00:00Z",
  "geo": {
    "country": "RU",
    "asn": 12345,
    "org": "Suspicious Hosting Inc."
  }
}
```

### 5.3 MITRE ATT&CK Mapping
**Map Alerts to Tactics and Techniques:**
```json
{
  "alert_id": "a1b2c3d4",
  "signature": "SQL Injection Attempt",
  "mitre_attack": {
    "tactic": "TA0001 - Initial Access",
    "technique": "T1190 - Exploit Public-Facing Application",
    "sub_technique": "T1190.001 - SQL Injection"
  }
}
```

**Benefits:**
- Understand attacker TTPs (Tactics, Techniques, Procedures)
- Correlate alerts across kill chain
- Prioritize defensive measures

---

## 6. Incident Response Integration

### 6.1 Automated Playbooks
**SOAR (Security Orchestration, Automation, and Response):**

**Playbook Example - High Severity Alert:**
```yaml
trigger:
  event: "alert_created"
  condition: "severity == 'HIGH'"

actions:
  - step: 1
    action: "enrich_alert"
    params:
      - geo_lookup: true
      - threat_intel_check: true
      - whois_lookup: true

  - step: 2
    action: "notify_soc"
    params:
      channels: ["email", "slack", "pagerduty"]

  - step: 3
    action: "quarantine_host"
    condition: "threat_score > 90"
    params:
      method: "firewall_block"
      duration: "1h"

  - step: 4
    action: "create_ticket"
    params:
      system: "ServiceNow"
      priority: "P1"
      assignment_group: "Security Incident Response"
```

### 6.2 Notification Channels
**Email:**
- SMTP/SMTPS support
- HTML formatted alerts with embedded details
- Attachments: PCAP file, alert JSON

**SMS:**
- Twilio API integration
- Maximum 160 characters summary
- Critical alerts only (severity >= HIGH)

**Webhook:**
- HTTP POST to custom endpoint
- JSON payload
- Retry logic with exponential backoff

**SNMP Trap:**
- SNMPv3 for security
- Custom MIB for WIA-IDS alerts
- Integration with network management systems (NMS)

**Slack/Teams:**
- Rich message formatting
- Interactive buttons (acknowledge, escalate, dismiss)
- Channel routing based on severity

---

## 7. Compliance Reporting

### 7.1 PCI DSS Reports
**Requirement 11.4 - Intrusion Detection:**
- Alert summary by severity
- Top triggered signatures
- Response times (MTTD, MTTA, MTTR)
- Evidence of daily signature updates
- Quarterly review documentation

### 7.2 HIPAA Reports
**164.312(b) - Audit Controls:**
- Access logs to ePHI systems
- Failed authentication attempts
- Malware detection events
- Network anomalies near ePHI data stores

### 7.3 ISO 27001 Reports
**Control 8.16 - Monitoring Activities:**
- Security event trends
- Incident response metrics
- System availability and performance
- Policy violations

**Report Format:**
```json
{
  "report_id": "rpt-2025-12",
  "period": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2025-12-31T23:59:59Z"
  },
  "summary": {
    "total_alerts": 1247,
    "critical": 12,
    "high": 89,
    "medium": 456,
    "low": 690
  },
  "top_signatures": [
    {"id": 2100498, "name": "SQL Injection", "count": 234},
    {"id": 2100499, "name": "XSS Attempt", "count": 189}
  ],
  "response_times": {
    "mean_time_to_detect": "45s",
    "mean_time_to_alert": "8s",
    "mean_time_to_respond": "12m"
  },
  "compliance_status": "PASS"
}
```

---

## 8. Performance Optimization

### 8.1 Hardware Acceleration
**FPGA (Field-Programmable Gate Array):**
- Offload signature matching to FPGA
- 10-100x performance improvement
- Supports 100+ Gbps throughput

**GPU Acceleration:**
- Use CUDA for ML inference
- Real-time anomaly detection on GPU
- TensorRT optimization for neural networks

**DPDK (Data Plane Development Kit):**
- Bypass kernel for packet processing
- Zero-copy packet forwarding
- <1 microsecond latency

### 8.2 Multi-Threading
**Packet Processing Pipeline:**
1. Capture thread (dedicated CPU core)
2. Decoder threads (2-4 cores)
3. Detection threads (8-16 cores)
4. Output threads (2 cores)

**Thread Affinity:**
- Pin threads to specific CPU cores
- NUMA-aware memory allocation
- Minimize cache misses

### 8.3 Rule Optimization
**Signature Pruning:**
- Disable irrelevant signatures (e.g., Windows rules on Linux networks)
- Group rules by protocol
- Use fast pattern matching (Boyer-Moore)

**Threshold Tuning:**
- Suppress noisy signatures
- Aggregate similar alerts (event_filter)
- Rate-limit alerts per source IP

---

## Appendix: Advanced Configuration Examples

### Example 1: Machine Learning Model Deployment
```python
import joblib
import numpy as np

# Load pre-trained model
model = joblib.load('ids_random_forest.pkl')

# Extract features from packet
def extract_features(packet):
    return np.array([
        len(packet),
        packet.ttl,
        packet.protocol,
        packet.tcp_flags,
        calculate_entropy(packet.payload)
    ]).reshape(1, -1)

# Predict
features = extract_features(packet)
prediction = model.predict(features)
probability = model.predict_proba(features)

if prediction == 1 and probability[0][1] > 0.85:
    generate_alert("ML Model: Malicious traffic detected", confidence=probability[0][1])
```

### Example 2: SIEM Correlation Rule
```python
# Splunk SPL (Search Processing Language)
index="ids_alerts" severity="HIGH"
| stats count by source_ip
| where count > 5
| join source_ip [
    search index="firewall" action="allowed"
    | stats count by src_ip
    | rename src_ip as source_ip
]
| eval risk_score = count * 10
| where risk_score > 50
| alert
```

---

**Document Control:**
- Author: WIA Security Standards Committee
- Effective Date: 2025-12-25
- Review Cycle: Annual
- Next Review: 2026-12-25

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association (WIA)

---

## Annex A — Conformance Tier Matrix

WIA conformance for intrusion-detection is evaluated across three tiers:

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

- `wia-standards/standards/intrusion-detection/api/` — TypeScript SDK skeleton
- `wia-standards/standards/intrusion-detection/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/intrusion-detection/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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



---

## Annex F — Operations and lifecycle notes

This informative annex captures operational guidance that has emerged from reference implementations and is expected to migrate into the normative body in a future minor revision.

### F.1 Deprecation policy

When a member of a canonical schema is deprecated, the schema MUST continue to accept and emit the member for at least 12 months from the publication of the deprecation notice. During the deprecation window, the implementation SHOULD emit a `Deprecation` HTTP header per the IETF deprecation-header draft for any response that contains the deprecated member, and SHOULD provide a `Sunset` header indicating the planned removal date per IETF RFC 8594.

### F.2 Backwards-compatible extensions

Vendors who extend a canonical schema with their own members MUST namespace those members with a reverse-DNS prefix, MUST treat the extension as opt-in, and MUST NOT shadow any normative member name reserved for future minor revisions of the standard.

### F.3 Operational telemetry

Every conformant deployment SHOULD expose a small set of operational metrics aligned with the OpenTelemetry semantic conventions. The recommended metric names are `wia.<slug>.requests.duration`, `wia.<slug>.requests.errors`, and `wia.<slug>.records.in_flight`.
