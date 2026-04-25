# WIA-SEC-016: Intrusion Detection Standard
## PHASE 1 - CORE SPECIFICATION

**Standard ID:** WIA-SEC-016
**Title:** Intrusion Detection and Prevention Systems
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

## 1. Introduction

### 1.1 Purpose
This specification defines international standards for Intrusion Detection Systems (IDS) and Intrusion Prevention Systems (IPS). It establishes protocols for network monitoring, threat detection, real-time alerting, and integration with Security Information and Event Management (SIEM) platforms.

### 1.2 Scope
This standard covers:
- Network-based and host-based intrusion detection
- Signature-based and anomaly-based detection methods
- IDS/IPS architecture and deployment models
- Alert formats and notification mechanisms
- SIEM integration protocols
- Performance metrics and benchmarking

### 1.3 Philosophy
**弘益人間 (Benefit All Humanity)** - This standard aims to protect digital infrastructure and safeguard users from cyber threats through comprehensive, interoperable intrusion detection systems.

---

## 2. Core Components

### 2.1 Detection Engine
The heart of any IDS/IPS system, responsible for analyzing network traffic and identifying threats.

#### 2.1.1 Signature-Based Detection
- **Pattern Matching**: Uses pre-defined signatures to identify known attack patterns
- **Rule Syntax**: Compatible with Snort/Suricata rule formats
- **Signature Database**: Maintains 100,000+ threat signatures, updated daily
- **Multi-Pattern Matching**: Aho-Corasick algorithm for efficient parallel matching

**Example Signature:**
```
alert tcp any any -> $HOME_NET 80 (
    msg:"SQL Injection Attempt";
    flow:to_server,established;
    content:"SELECT"; nocase; http_uri;
    content:"FROM"; nocase; http_uri; distance:0;
    classtype:web-application-attack;
    sid:2100498;
)
```

#### 2.1.2 Anomaly-Based Detection
- **Baseline Establishment**: Statistical modeling of normal network behavior
- **Deviation Detection**: Identifies traffic patterns that deviate from baseline
- **Machine Learning**: Uses ML algorithms (Random Forest, Neural Networks) for zero-day detection
- **Behavioral Analysis**: Monitors user and entity behavior analytics (UEBA)

**Key Metrics:**
- Packets per second (PPS)
- Bytes per second (BPS)
- Connection rate
- Unique source/destination IP counts
- Protocol distribution

#### 2.1.3 Protocol Analysis
- **Deep Packet Inspection (DPI)**: Examines packet headers and payload
- **Protocol Validation**: Ensures compliance with RFC standards
- **Reassembly**: TCP stream and IP fragment reassembly
- **Decoding**: Supports HTTP, DNS, TLS, SMB, FTP, SSH, and 50+ protocols

### 2.2 Sensor Architecture

#### 2.2.1 Network-based IDS (NIDS)
**Deployment:**
- Inline (IPS mode): Packets flow through the sensor
- Passive (IDS mode): Sensor monitors via network tap or SPAN port

**Components:**
- Packet capture interface (libpcap/PF_RING/AF_PACKET)
- Packet decoder and preprocessor
- Detection engine
- Alert output module

**Performance Requirements:**
- Minimum: 1 Gbps throughput
- Standard: 10 Gbps throughput
- Enterprise: 40-100 Gbps with hardware acceleration

#### 2.2.2 Host-based IDS (HIDS)
**Monitors:**
- File integrity (critical system files, configurations)
- System calls and process execution
- Log files (auth.log, syslog, Windows Event Log)
- Registry modifications (Windows)
- Rootkit detection

**Agent Requirements:**
- Minimal CPU overhead (<5%)
- Low memory footprint (<256MB)
- Encrypted communication with central server
- Auto-update capability

---

## 3. Data Formats

### 3.1 Packet Capture Format
**PCAP (Packet Capture)**
- File format: .pcap or .pcapng
- Header: Global header + packet headers
- Timestamp: Microsecond precision
- Maximum packet size: 65,535 bytes

**PCAP Structure:**
```
Global Header (24 bytes)
  - Magic number (0xa1b2c3d4)
  - Version (major.minor)
  - Timezone offset
  - Timestamp accuracy
  - Snapshot length
  - Link-layer type

Packet Record
  - Timestamp (seconds + microseconds)
  - Captured length
  - Original length
  - Packet data
```

### 3.2 Flow Data Format
**NetFlow v9 / IPFIX**
```json
{
  "version": 9,
  "flow_sequence": 12345,
  "source_id": 1,
  "flows": [
    {
      "src_ip": "192.168.1.10",
      "dst_ip": "203.0.113.50",
      "src_port": 54321,
      "dst_port": 443,
      "protocol": "TCP",
      "packets": 150,
      "bytes": 98304,
      "start_time": "2025-12-25T10:00:00Z",
      "end_time": "2025-12-25T10:05:00Z",
      "tcp_flags": "ACK"
    }
  ]
}
```

### 3.3 Alert Format
**Unified Alert Format (JSON)**
```json
{
  "timestamp": "2025-12-25T10:30:15.123Z",
  "sensor_id": "ids-sensor-01",
  "alert_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "signature_id": 2100498,
  "signature": "ET EXPLOIT SQL Injection SELECT FROM",
  "classification": "web-application-attack",
  "severity": "HIGH",
  "priority": 1,
  "source": {
    "ip": "203.0.113.45",
    "port": 54321,
    "mac": "00:11:22:33:44:55",
    "geo": {
      "country": "US",
      "city": "New York",
      "latitude": 40.7128,
      "longitude": -74.0060
    }
  },
  "destination": {
    "ip": "192.168.1.100",
    "port": 80,
    "mac": "AA:BB:CC:DD:EE:FF",
    "hostname": "web-server-01.local"
  },
  "protocol": "TCP",
  "packet": {
    "length": 512,
    "ttl": 64,
    "payload_preview": "GET /api/users?id=1' OR 1=1--"
  },
  "action": "alert",
  "threat_intel": {
    "reputation_score": 85,
    "known_malicious": true,
    "threat_category": "sql-injection",
    "first_seen": "2025-11-20T00:00:00Z"
  }
}
```

---

## 4. Detection Rules

### 4.1 Rule Structure
**Snort/Suricata Compatible Syntax:**
```
<action> <protocol> <src_ip> <src_port> <direction> <dst_ip> <dst_port> (
    <rule_options>
)
```

**Actions:**
- `alert`: Generate alert and log packet
- `log`: Log packet without alert
- `pass`: Ignore packet (whitelist)
- `drop`: Block packet (IPS mode)
- `reject`: Block and send TCP RST or ICMP unreachable

**Rule Options:**
- `msg`: Human-readable alert message
- `content`: Payload content match
- `pcre`: Perl-compatible regular expression
- `flow`: TCP/IP flow tracking
- `threshold`: Rate-limiting alerts
- `reference`: External threat information (CVE, URL)

### 4.2 Example Rules

**Port Scan Detection:**
```
alert tcp any any -> $HOME_NET any (
    msg:"Possible TCP Port Scan";
    flags:S;
    threshold:type both, track by_src, count 20, seconds 60;
    classtype:attempted-recon;
    sid:1000001;
)
```

**DNS Tunneling:**
```
alert udp any any -> any 53 (
    msg:"DNS Query Unusually Long";
    content:"|00 01 00 00 00 00 00|";
    depth:7;
    dsize:>512;
    classtype:protocol-command-decode;
    sid:1000002;
)
```

**TLS Certificate Validation:**
```
alert tcp any any -> any 443 (
    msg:"Self-Signed TLS Certificate Detected";
    flow:to_server,established;
    ssl_state:server_hello;
    content:"Self-Signed";
    classtype:bad-unknown;
    sid:1000003;
)
```

---

## 5. Performance Metrics

### 5.1 Detection Metrics
- **True Positive Rate (TPR)**: TP / (TP + FN) - Target: >95%
- **False Positive Rate (FPR)**: FP / (FP + TN) - Target: <1%
- **Precision**: TP / (TP + FP) - Target: >90%
- **Recall**: TP / (TP + FN) - Target: >95%
- **F1 Score**: 2 * (Precision * Recall) / (Precision + Recall) - Target: >92%

### 5.2 System Performance
- **Throughput**: Packets processed per second
  - Minimum: 100,000 PPS
  - Standard: 1,000,000 PPS
  - Enterprise: 10,000,000+ PPS
- **Latency**: Time from packet arrival to alert generation
  - Target: <10ms (95th percentile)
  - Maximum: <100ms (99th percentile)
- **Packet Loss**: Percentage of dropped packets
  - Target: <0.01%
- **CPU Utilization**: Under maximum load
  - Target: <80%
- **Memory Usage**: Per 1Gbps throughput
  - Target: <2GB RAM

### 5.3 Alert Metrics
- **Mean Time to Detect (MTTD)**: Average time to detect intrusion - Target: <60 seconds
- **Mean Time to Alert (MTTA)**: Average time to generate alert - Target: <10 seconds
- **Mean Time to Respond (MTTR)**: Average time to respond to alert - Target: <15 minutes
- **Alert Volume**: Alerts per day - Target: <500 (after tuning)
- **Alert-to-Incident Ratio**: Percentage of alerts requiring investigation - Target: >20%

---

## 6. Compliance & Standards

### 6.1 Regulatory Compliance
- **PCI DSS 4.0**: Requirement 11.4 (Intrusion Detection/Prevention)
- **HIPAA Security Rule**: 164.312(b) - Audit Controls
- **NIST Cybersecurity Framework**: PR.DS-5, DE.CM-1, DE.CM-7
- **ISO/IEC 27001:2022**: Control 8.16 (Monitoring Activities)
- **GDPR**: Article 32 (Security of Processing)

### 6.2 Technical Standards
- **RFC 3164**: BSD Syslog Protocol
- **RFC 5424**: The Syslog Protocol
- **RFC 7011**: IPFIX Protocol Specification
- **IEEE 802.1Q**: VLAN tagging for network segmentation
- **MITRE ATT&CK**: Framework for threat detection mapping

### 6.3 Certification Requirements
- **Common Criteria (CC)**: EAL3+ certification
- **FIPS 140-2**: For cryptographic modules
- **ICSA Labs**: Network IPS certification
- **NSS Labs**: Breach Prevention System (BPS) testing

---

## 7. Security Considerations

### 7.1 Sensor Security
- **Hardened OS**: Minimal services, latest patches
- **Network Isolation**: Management interface on separate VLAN
- **Authentication**: Multi-factor authentication for admin access
- **Encryption**: TLS 1.3 for all communications
- **Integrity Monitoring**: File integrity checks on sensor binaries

### 7.2 Alert Security
- **Digital Signatures**: Sign alerts to prevent tampering
- **Encrypted Transport**: Use TLS for alert transmission
- **Access Control**: Role-based access to alert console
- **Audit Logging**: Log all alert access and modifications

### 7.3 Evasion Prevention
- **Anti-Fragmentation**: Detect fragmentation attacks
- **Protocol Normalization**: Prevent protocol-level evasion
- **Stream Reassembly**: Handle out-of-order packets
- **Encoding Detection**: Detect Unicode, Base64, URL encoding tricks

---

## Appendix A: Glossary

**IDS (Intrusion Detection System)**: Passive monitoring system that detects and alerts on threats.

**IPS (Intrusion Prevention System)**: Active system that detects and blocks threats in real-time.

**NIDS (Network-based IDS)**: IDS that monitors network traffic.

**HIDS (Host-based IDS)**: IDS that monitors individual host systems.

**Signature**: Pre-defined pattern that identifies known attack.

**Anomaly Detection**: Identifying deviations from normal behavior baseline.

**False Positive**: Benign traffic incorrectly flagged as malicious.

**False Negative**: Malicious traffic that evades detection.

**SIEM**: Security Information and Event Management system.

**DPI (Deep Packet Inspection)**: Examination of packet payload and headers.

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


## Annex E — Implementation Notes for PHASE-1-CORE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-CORE.

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
