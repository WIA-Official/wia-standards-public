# WIA-SEMI-TEST-001 — Phase 2: API

> Semiconductor Testing canonical Phase 2 specification per the WIA Standards four-Phase architecture.

> Domain: 반도체 시험 — ATE · 웨이퍼 프로빙 · BIST · 신뢰성 시험 · JEDEC.

## A.1 Scope

This Phase covers the canonical api layer of the WIA-SEMI-TEST-001 standard. It composes with the Phase 3 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- JEDEC JESD22 series (Reliability Test Methods)
- JEDEC JESD47 (Stress-Test-Driven Qualification)
- JEDEC JESD89 (Single Event Effects)
- IEEE 1450 STIL (Standard Test Interface Language)
- IEEE 1149.x JTAG family
- IEEE 1500 (Embedded Core Test)
- IEEE 1687 IJTAG
- AEC-Q100 / Q104 (Automotive)
- MIL-STD-883 (Military / Aerospace)

## Technical Specification v1.0

**Status:** Draft
**Version:** 1.0.0
**Date:** January 2025
**Authors:** WIA Technical Committee on Semiconductor Testing

---


## 3. Normative References

The following standards are referenced in this document:

- **SEMI E5:** Standard Test Data Format (STDF) Specification
- **JEDEC Standard 51:** Methodology for Thermal Measurement
- **IPC/JEDEC J-STD-020:** Moisture/Reflow Sensitivity Classification
- **SEMI E10:** Standard for Definition and Measurement of Equipment Reliability
- **ISO 9001:** Quality Management Systems

---


## 7. Test Protocol Specification

### 7.1 Wafer Probe Testing

#### 7.1.1 Parametric Tests

Required parametric measurements:

1. **Threshold Voltage (Vth)**
   - Test condition: VDS = 0.1V, VGS sweep
   - Specification: ±10% of nominal
   - Measurement accuracy: ±1mV

2. **Leakage Current (Ileak)**
   - Test condition: VGS = 0V, VDS = nominal
   - Specification: < 100nA per device
   - Measurement accuracy: ±1nA

3. **Drive Current (Idrive)**
   - Test condition: VGS = VDS = nominal
   - Specification: ±15% of nominal
   - Measurement accuracy: ±0.1%

#### 7.1.2 Functional Tests

Functional test requirements:

- **Pattern Coverage:** Minimum 95% stuck-at fault coverage
- **At-Speed Testing:** ≥ 90% of nominal frequency
- **Scan Chain Testing:** Full scan chain integrity verification
- **Memory Testing:** March C- or equivalent algorithm

### 7.2 Package Testing

#### 7.2.1 Continuity Testing

All package pins SHALL be tested for:
- Pin-to-die connectivity
- Inter-pin isolation (> 1MΩ)
- Pin resistance (< 10Ω)

#### 7.2.2 Environmental Testing

Tests SHALL be performed at:
- **Cold:** -40°C or device minimum
- **Nominal:** 25°C
- **Hot:** 125°C or device maximum

Voltage corners:
- **Min:** VDD - 10%
- **Nominal:** VDD
- **Max:** VDD + 10%

#### 7.2.3 Burn-In Protocol

Standard burn-in conditions:
- **Temperature:** 125°C
- **Voltage:** VDD + 10%
- **Duration:** 48-168 hours
- **Activity:** Dynamic pattern (50% toggle rate)
- **Monitoring:** Continuous with < 1 hour intervals

### 7.3 Binning Specification

Standard bin codes:

| Bin | Description | Disposition |
|-----|-------------|-------------|
| 0 | Fail | Scrap |
| 1 | Pass - Full spec | Ship |
| 2 | Pass - Speed bin 2 | Ship |
| 3 | Pass - Speed bin 3 | Ship |
| 4-8 | Pass - Reduced spec | Ship with restrictions |
| 9 | Retest required | Retest |

---


## Appendix A: Reference Implementation

See TypeScript SDK in `/api/typescript/` for reference implementation.

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
with `wia.standard.slug` = `semiconductor-testing` and `wia.standard.phase` =
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
`/.well-known/wia-semiconductor-testing-capabilities` enumerating per-endpoint
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
with `wia.standard.slug` = `semiconductor-testing` and `wia.standard.phase` =
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
`/.well-known/wia-semiconductor-testing-capabilities` enumerating per-endpoint
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
