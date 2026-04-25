# WIA-SEC-016: Intrusion Detection Standard

# PHASE 2: ADVANCED DETECTION

## 1. Machine Learning-Based Detection

### 1.1 Supervised Learning
**Objective:** Train models on labeled attack datasets to classify traffic.

**Algorithms:**
- **Random Forest**: Ensemble decision trees for classification
  - Features: Packet size, inter-arrival time, protocol, flags
  - Training: 80/20 train-test split
  - Accuracy target: >92%

- **Support Vector Machines (SVM)**: Binary classification (benign/malicious)
  - Kernel: RBF (Radial Basis Function)
  - Feature normalization: Min-max scaling
  - Cross-validation: 5-fold

- **Neural Networks**: Deep learning for complex pattern recognition
  - Architecture: Input (50 features) → Dense(128) → Dense(64) → Dense(32) → Output(2)
  - Activation: ReLU hidden layers, Softmax output
  - Loss: Categorical crossentropy
  - Training epochs: 100 with early stopping

**Training Data Requirements:**
- Minimum 1 million labeled samples
- Balanced classes (50% benign, 50% malicious)
- Diverse attack types (DDoS, SQL injection, malware, port scans)
- Real-world traffic scenarios

### 1.2 Unsupervised Learning
**Objective:** Detect unknown attacks without labeled data.

**Techniques:**
- **K-Means Clustering**: Group similar traffic patterns
  - K selection: Elbow method or silhouette analysis
  - Distance metric: Euclidean or Mahalanobis
  - Outlier threshold: 3 standard deviations from cluster centroid

- **Isolation Forest**: Identify anomalous data points
  - Tree depth: log₂(sample size)
  - Contamination factor: 0.01-0.05
  - Use case: Zero-day attack detection

- **Autoencoder**: Neural network for anomaly detection
  - Architecture: Encoder-Bottleneck-Decoder
  - Reconstruction error threshold: 95th percentile of training error
  - Input: Traffic features normalized to [0,1]

**Performance Metrics:**
- Anomaly detection rate: >80%
- False positive rate: <10%
- Processing time: <50ms per sample

### 1.3 Feature Engineering
**Network Traffic Features (50+ total):**

**Packet-Level:**
- Packet size (bytes)
- Time-to-live (TTL)
- Protocol type (TCP/UDP/ICMP)
- TCP flags (SYN, ACK, FIN, RST, PSH, URG)
- Payload entropy (Shannon entropy)

**Flow-Level:**
- Flow duration
- Total packets (forward/backward)
- Total bytes (forward/backward)
- Packets per second
- Bytes per second
- Mean/min/max/std packet size
- Mean/min/max/std inter-arrival time

**Connection-Level:**
- Number of connections per IP (last 5 minutes)
- Unique destination IPs per source
- Port diversity (number of unique ports accessed)
- Protocol distribution

**Application-Level:**
- HTTP request method (GET/POST/PUT/DELETE)
- HTTP status code
- DNS query length
- TLS cipher suite
- SSL certificate validity

---

## 2. Behavioral Analysis

### 2.1 User and Entity Behavior Analytics (UEBA)
**Goal:** Detect insider threats and compromised accounts.

**Baseline Establishment:**
- Monitor user activity for 30-90 days
- Profile normal behaviors:
  - Login times (time of day, day of week)
  - Access patterns (resources accessed, frequency)
  - Geographic locations (IP geolocation)
  - Device fingerprints (user agent, OS)

**Anomaly Indicators:**
- Login from unusual location (geo-velocity analysis)
- Access to sensitive resources never accessed before
- Unusual data exfiltration volume
- Privilege escalation attempts
- Off-hours activity (statistically significant deviation)

**Scoring System:**
- Risk score: 0-100 (weighted sum of anomaly indicators)
- Threshold for alert: >70
- Threshold for automatic response: >90

### 2.2 Network Behavior Analysis
**Traffic Pattern Analysis:**
- Identify deviations in traffic volume, protocol distribution, connection patterns
- Detect lateral movement (east-west traffic anomalies)
- Identify command-and-control (C2) beaconing

**C2 Beaconing Detection:**
```python
def detect_beaconing(flows):
    # Analyze connection regularity
    intervals = calculate_intervals(flows)

    # Statistical tests
    mean_interval = np.mean(intervals)
    std_interval = np.std(intervals)
    coefficient_of_variation = std_interval / mean_interval

    # Beaconing indicators
    if coefficient_of_variation < 0.3:  # Regular intervals
        if len(flows) > 10:  # Persistent connections
            if mean_interval < 3600:  # Less than 1 hour
                return True, "High confidence C2 beaconing"

    return False, "Normal traffic"
```

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


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

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
