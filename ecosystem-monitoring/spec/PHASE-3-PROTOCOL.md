# WIA Ecosystem Monitoring Standard - Phase 3: Protocol Specification v1.0

**Status:** Official Release  
**Version:** 1.0.0  
**Date:** December 26, 2025  
**License:** CC BY 4.0

## 1. Introduction

Phase 3 defines communication protocols, quality assurance procedures, and calibration standards.

## 2. Sensor Communication Protocols

### 2.1 Hardware Interfaces

Supported standards:
- **SDI-12**: Serial Data Interface for environmental sensors
- **Modbus RTU/TCP**: Industrial sensor protocol
- **I2C/SPI**: Digital sensor buses
- **4-20mA**: Analog current loop

### 2.2 Network Transport

Protocols by use case:
- **MQTT**: Low-bandwidth IoT sensor networks
- **CoAP**: Constrained devices
- **HTTP/HTTPS**: Standard connectivity
- **WebSocket**: Real-time bidirectional
- **LoRaWAN**: Long-range low-power wireless

### 2.3 Data Packet Structure

```json
{
  "packet_version": "1.0",
  "packet_id": "UUID",
  "sensor_id": "string",
  "timestamp": "ISO 8601",
  "sequence_number": "integer",
  "readings": [ /* array of measurements */ ],
  "battery_voltage": "number",
  "signal_strength": "number",
  "checksum": "CRC32 hash"
}
```

### 2.4 Error Detection

Requirements:
- CRC32 checksums for all packets
- Sequence numbers for ordering/duplicate detection
- Acknowledgments with retry mechanism
- Local buffering until transmission confirmed
- NTP/GPS time synchronization (±1 second accuracy)

## 3. Calibration Protocols

### 3.1 Calibration Frequency

Minimum requirements:

| Sensor Type | Interval | Method |
|------------|----------|--------|
| Temperature | 6 months | Ice point check |
| pH | 2 weeks | Two-point buffer |
| Dissolved Oxygen | 1 month | Water-saturated air |
| Conductivity | 3 months | Standard solution |
| Turbidity | 1 month | Formazin standards |
| Nutrients | Each run | Standard curve |

### 3.2 Calibration Documentation

Required metadata:
```json
{
  "calibration_id": "string",
  "sensor_id": "string",
  "calibration_date": "ISO 8601",
  "technician": "string",
  "method": "string",
  "standards_used": [
    {
      "value": "number",
      "lot_number": "string",
      "expiry_date": "ISO 8601"
    }
  ],
  "pre_calibration_readings": "array",
  "post_calibration_readings": "array",
  "adjustment_applied": "boolean",
  "drift_detected": "number",
  "pass_fail": "enum",
  "next_calibration_due": "ISO 8601",
  "certificate_url": "URL"
}
```

## 4. Quality Assurance Protocols

### 4.1 Field QA/QC

Requirements:
- **Field blanks**: 5% of samples
- **Equipment blanks**: Before/after sampling events
- **Field replicates**: 10% of samples
- **Positive controls**: For presence/absence methods

### 4.2 Laboratory QA/QC

Requirements:
- **Method blanks**: Each analytical batch
- **Calibration verification**: Every 10 samples
- **Spike recovery**: 10% of samples
- **Duplicate analysis**: 10% of samples
- **Certified reference materials**: Each batch
- **Blind QC samples**: 5% of samples

Acceptance criteria:
- Blank contamination: < detection limit
- Spike recovery: 75-125%
- Duplicate RPD: < 20%
- Reference material: Within ±2 standard deviations

### 4.3 Automated Data Validation

Required checks:
1. **Range checks**: Values within physical limits
2. **Rate-of-change**: Maximum change rates
3. **Flatline detection**: Minimum variability thresholds
4. **Spike detection**: Isolated extreme values
5. **Statistical outliers**: Beyond 3-4 standard deviations
6. **Cross-parameter**: Consistency between related variables

## 5. Field Sampling Protocols

### 5.1 Species Observations

**Point Count Protocol**:
- Fixed location for 5-10 minutes
- Record all detections within specified radius
- Note distance bands for detectability
- Conduct during appropriate season/time
- Multiple visits for detection probability
- Document environmental conditions

**Transect Protocol**:
- Predetermined route or random walk
- Defined width or distance bands
- Perpendicular distance measurement
- Consistent speed and timing
- Multiple observers when possible

### 5.2 Water Sampling

Standard procedure:
1. Approach from downstream
2. Rinse bottle 3× with ambient water
3. Collect mid-stream, mid-depth
4. Avoid sediment disturbance
5. Fill completely (no headspace for DO)
6. Preserve immediately if required
7. Label with metadata
8. Document field parameters
9. Maintain chain of custody
10. Transport on ice

### 5.3 Soil Sampling

Standard procedure:
1. Clear surface debris
2. Sample to specified depth
3. Composite multiple cores if required
4. Use clean, dedicated equipment
5. Store in appropriate containers
6. Keep cool until analysis
7. Process within specified time
8. Document horizon/depth precisely

## 6. Data Management Protocols

### 6.1 Data Entry

Requirements:
- Double data entry for critical datasets
- Real-time validation during entry
- Immediate backup of raw data
- Version control for edits
- Separation of raw/processed data

### 6.2 Metadata Documentation

Minimum required metadata (following EML/ISO 19115):
- Dataset title, abstract, keywords
- Authors, contacts, funding
- Temporal/spatial/taxonomic coverage
- Detailed methods and protocols
- QA/QC procedures
- Known limitations
- Access restrictions and licenses
- Related publications/datasets

### 6.3 Long-term Preservation

Requirements:
- Format migration to current standards
- Geographic distribution of backups
- Checksum verification
- Deposit in discipline repositories
- DOI assignment for citation

## 7. Change Management

### 7.1 Protocol Modifications

When changing methods:
1. Document what changed, when, why
2. Version control protocol documents
3. Update metadata
4. Flag data with method used
5. Conduct overlap studies (≥1 year)
6. Develop conversion factors
7. Document precision/bias differences

## 8. Proficiency Testing

### 8.1 Laboratory Proficiency

Requirements:
- Quarterly proficiency samples
- External provider or internal blind samples
- Z-score calculation and tracking
- Corrective action for |Z| > 2

### 8.2 Observer Calibration

Requirements:
- Multiple observers, same area
- Annual or when observers change
- Expert verification of identifications
- Training for discrepancies
- Document inter-observer variability


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
