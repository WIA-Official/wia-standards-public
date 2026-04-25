# WIA-SEC-016: Intrusion Detection Standard

# PHASE 3: INTEGRATION & DEPLOYMENT

## 3. SIEM Integration

### 3.1 Supported Protocols
**Syslog (RFC 5424):**
```
<Priority>Version Timestamp Hostname AppName ProcID MsgID [StructuredData] Message

Example:
<134>1 2025-12-25T10:30:15.123Z ids-01 wia-ids 1234 ID001 [ids alert_id="a1b2c3" severity="HIGH"] SQL Injection detected
```

**Common Event Format (CEF):**
```
CEF:Version|Device Vendor|Device Product|Device Version|Signature ID|Name|Severity|Extension

Example:
CEF:0|WIA|IDS|1.0|2100498|SQL Injection|8|src=203.0.113.45 dst=192.168.1.100 spt=54321 dpt=80
```

**Log Event Extended Format (LEEF):**
```
LEEF:Version|Vendor|Product|Version|EventID|Delimiter|Key-Value Pairs

Example:
LEEF:2.0|WIA|IDS|1.0|2100498|^|src=203.0.113.45^dst=192.168.1.100^severity=8
```

### 3.2 API Integration
**REST API Endpoints:**

**POST /api/v1/alerts** - Submit alert
```json
{
  "timestamp": "2025-12-25T10:30:15Z",
  "signature_id": 2100498,
  "severity": "HIGH",
  "source_ip": "203.0.113.45",
  "dest_ip": "192.168.1.100"
}
```

**GET /api/v1/alerts?start_time={ts}&severity={level}** - Query alerts

**GET /api/v1/sensors** - List active sensors

**POST /api/v1/rules** - Upload detection rules

**Authentication:**
- API Key in header: `X-API-Key: <key>`
- OAuth 2.0 client credentials flow
- JWT tokens with 1-hour expiration

### 3.3 Platform-Specific Integrations

#### Splunk Integration
**HTTP Event Collector (HEC):**
```bash
curl -k https://splunk-server:8088/services/collector \
  -H "Authorization: Splunk <HEC_TOKEN>" \
  -d '{
    "time": 1735123815,
    "source": "wia-ids",
    "sourcetype": "ids:alert",
    "event": {
      "signature_id": 2100498,
      "severity": "HIGH",
      "src_ip": "203.0.113.45"
    }
  }'
```

**Splunk Universal Forwarder:**
- Monitor IDS log files: `/var/log/wia-ids/alerts.log`
- Index: `security_ids`
- Source type: `ids:alert`

#### Elastic Stack (ELK)
**Filebeat Configuration:**
```yaml
filebeat.inputs:
- type: log
  enabled: true
  paths:
    - /var/log/wia-ids/alerts.json
  json.keys_under_root: true

output.elasticsearch:
  hosts: ["elasticsearch:9200"]
  index: "ids-alerts-%{+yyyy.MM.dd}"

processors:
  - add_host_metadata: ~
  - add_cloud_metadata: ~
```

**Elasticsearch Index Mapping:**
```json
{
  "mappings": {
    "properties": {
      "timestamp": { "type": "date" },
      "signature_id": { "type": "integer" },
      "severity": { "type": "keyword" },
      "source.ip": { "type": "ip" },
      "destination.ip": { "type": "ip" },
      "signature": { "type": "text" }
    }
  }
}
```

---

## 4. Deployment Models

### 4.1 Network Tap Deployment
**Passive Monitoring (IDS Mode):**
- Physical network tap or optical splitter
- No impact on network traffic
- Zero risk of network disruption
- Cannot block attacks (alert-only)

**Recommended for:**
- High-availability production networks
- Compliance monitoring
- Forensic analysis

**Architecture:**
```
Internet → Firewall → [Network Tap] → Switch → Internal Network
                            |
                        IDS Sensor
```

### 4.2 Inline Deployment
**Active Prevention (IPS Mode):**
- Installed in network path
- Inspects and can block traffic
- Single point of failure (requires HA setup)
- Minimal latency (<5ms)

**Recommended for:**
- Perimeter defense
- Critical asset protection
- Zero-trust networks

**Architecture:**
```
Internet → Firewall → IPS Sensor → Switch → Internal Network
                      (inline)
```

**High Availability (HA):**
- Active-Passive or Active-Active clustering
- Heartbeat monitoring (VRRP or proprietary)
- Synchronized rule sets and state tables
- Automatic failover <2 seconds

### 4.3 Hybrid Deployment
**Combination of IDS and IPS:**
- IPS at network perimeter (inline)
- IDS at internal segments (passive)
- Host-based IDS on critical servers

**Benefits:**
- Defense in depth
- Minimal performance impact on internal networks
- Comprehensive visibility

### 4.4 Cloud Deployment
**AWS Integration:**
- VPC Traffic Mirroring to IDS instance
- AWS Transit Gateway for centralized inspection
- CloudWatch Logs for SIEM integration

**Azure Integration:**
- Azure Network Watcher packet capture
- Virtual Network TAP
- Azure Sentinel integration

**GCP Integration:**
- VPC Packet Mirroring
- Cloud Logging export to IDS
- Chronicle SIEM integration

---

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



---

## Annex F — Operations and lifecycle notes

This informative annex captures operational guidance that has emerged from reference implementations and is expected to migrate into the normative body in a future minor revision.

### F.1 Deprecation policy

When a member of a canonical schema is deprecated, the schema MUST continue to accept and emit the member for at least 12 months from the publication of the deprecation notice. During the deprecation window, the implementation SHOULD emit a `Deprecation` HTTP header per the IETF deprecation-header draft for any response that contains the deprecated member, and SHOULD provide a `Sunset` header indicating the planned removal date per IETF RFC 8594.

### F.2 Backwards-compatible extensions

Vendors who extend a canonical schema with their own members MUST namespace those members with a reverse-DNS prefix, MUST treat the extension as opt-in, and MUST NOT shadow any normative member name reserved for future minor revisions of the standard.

### F.3 Operational telemetry

Every conformant deployment SHOULD expose a small set of operational metrics aligned with the OpenTelemetry semantic conventions. The recommended metric names are `wia.<slug>.requests.duration`, `wia.<slug>.requests.errors`, and `wia.<slug>.records.in_flight`.
