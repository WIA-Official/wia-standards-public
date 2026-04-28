# WIA-QUA-020 — Phase 4: Integration

> Scientific-instrument canonical Phase 4: ecosystem integration (ISO/IEC 17025 + GMP + clinical + environmental + metrology + references).

# WIA-QUA-020: Scientific Instrument Standard
# Version 1.0.0

**Standard ID:** WIA-QUA-020
**Title:** Scientific Instrument
**Category:** QUA (미래기술/양자/물리)
**Status:** Active
**Published:** 2025-01-01
**Updated:** 2025-01-01

---

## Abstract

This specification defines a comprehensive standard for advanced scientific instruments used in research, discovery, and innovation. The WIA-QUA-020 standard covers particle accelerators, mass spectrometers, electron microscopes, X-ray crystallography systems, NMR spectrometers, gravitational wave detectors, telescopes, spectrophotometers, chromatography systems, calorimeters, data acquisition systems, and calibration standards.

**弘益人間 (Benefit All Humanity)** - This standard facilitates global collaboration, data sharing, and reproducibility in scientific research.

---


## 6. Quality Assurance

### 6.1 Instrument Qualification

**IQ/OQ/PQ Protocols:**
- Installation Qualification (IQ)
- Operational Qualification (OQ)
- Performance Qualification (PQ)

### 6.2 Maintenance

**Preventive Maintenance:**
- Daily: Leak checks, calibration
- Weekly: Cleaning, alignment
- Monthly: Vacuum pump service
- Yearly: Full calibration, certification

### 6.3 Validation

**Method Validation Parameters:**
- Accuracy
- Precision (repeatability, reproducibility)
- Linearity
- Range
- Limit of detection (LOD)
- Limit of quantification (LOQ)
- Robustness

---



## 10. Compliance

### 10.1 Standards

This standard complies with:
- ISO/IEC 17025 (Testing and Calibration)
- ISO 9001 (Quality Management)
- FDA 21 CFR Part 11 (Electronic Records)
- GLP/GMP (Good Laboratory/Manufacturing Practice)

### 10.2 Certification

WIA certification available at: [cert.wiastandards.com](https://cert.wiastandards.com)

---



## 11. References

2. Dass, C. (2007). *Fundamentals of Contemporary Mass Spectrometry*. Wiley.
3. Williams, D.B. & Carter, C.B. (2009). *Transmission Electron Microscopy*. Springer.

---



## 12. Appendix

### A. Physical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Speed of light | c | 299,792,458 m/s |
| Planck constant | h | 6.62607015×10⁻³⁴ J·s |
| Elementary charge | e | 1.602176634×10⁻¹⁹ C |
| Electron mass | mₑ | 9.1093837015×10⁻³¹ kg |
| Proton mass | mₚ | 1.67262192369×10⁻²⁷ kg |

### B. Unit Conversions

| From | To | Factor |
|------|-----|--------|
| eV | J | 1.602176634×10⁻¹⁹ |
| Å | m | 1×10⁻¹⁰ |
| amu | kg | 1.66053906660×10⁻²⁷ |
| bar | Pa | 1×10⁵ |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 Standards cross-walk

| Concern                            | Standard                                       |
|------------------------------------|------------------------------------------------|
| Laboratory competence              | ISO/IEC 17025                                   |
| Calibration laboratories           | ISO/IEC 17025 + ILAC-G24                        |
| Reference materials                | ISO 17034                                       |
| Proficiency testing                | ISO/IEC 17043                                   |
| Measurement uncertainty            | ISO/IEC Guide 98-3 (GUM) + JCGM 100             |
| Metrological vocabulary            | ISO/IEC Guide 99 (VIM) + JCGM 200               |
| Method validation (pharma)         | ICH Q2(R2)                                      |
| Method validation (clinical)       | CLSI EP05 / EP06 / EP09 / EP17                  |
| Data integrity (pharma)            | 21 CFR Part 11 + EMA Annex 11 + WHO TRS 996     |
| Lab automation interfaces          | ASTM E1394 + SiLA 2 + HL7 LAW                   |
| Spectral data interchange          | JCAMP-DX + AnIML + mzML                         |
| Microscopy interchange             | OME-TIFF + Bio-Formats                          |
| Chromatography interchange         | netCDF + AnIML + AIA/ANDI                       |
| Laser safety                       | IEC 60825-1                                     |
| Ionising-radiation safety          | IAEA GSR Part 3 + ICRP 103                      |
| Electrical safety (lab)            | IEC 61010-1                                     |

## A.2 ISO/IEC 17025 integration envelope

Integration with ISO/IEC 17025 covers: the laboratory's quality-
management system (clauses §8.1-8.9) feeding into instrument-
management policy; technical requirements (clauses §6.1-6.6 for
personnel, facilities, equipment, metrological traceability,
externally-provided products and services, internally-issued
reference materials) feeding into per-instrument capability
records; and process requirements (clauses §7.1-7.11 covering
review of requests, selection and validation of methods, sampling,
handling of test items, technical records, evaluation of
measurement uncertainty, ensuring the validity of results,
reporting, complaints, nonconforming work, control of data and
information management) feeding into the per-measurement envelope.

## A.3 Pharmaceutical / GMP integration envelope

GMP integration follows ICH Q9 quality risk management + ICH Q10
pharmaceutical quality system + 21 CFR Part 211 + EU GMP Part I
+ WHO TRS 986. Instrument qualification follows the IQ/OQ/PQ
sequence per USP <1058>: Installation Qualification documents
correct installation; Operational Qualification documents that
the instrument operates per specification across the operational
range; Performance Qualification documents fitness for the
intended analytical use. Periodic requalification follows the
operator's risk-based schedule; out-of-tolerance events trigger
deviation per ICH Q10 §3.2.5 with CAPA per FDA Guidance.

## A.4 Clinical-laboratory integration envelope

Clinical integration follows CLIA per 42 CFR Part 493 (US) + CAP
checklists + ISO 15189 (international): personnel-qualification
envelope per CLIA §493.1407-1495; quality-management envelope per
ISO 15189 §4 + §5; pre-examination process envelope per ISO 15189
§5.4; examination-process envelope per ISO 15189 §5.5 with
internal QC per Westgard / external proficiency testing per
CMS-approved PT provider; post-examination process envelope per
ISO 15189 §5.7 with critical-result reporting envelope per CLSI
GP47.

## A.5 Environmental / metrological integration envelope

Environmental-laboratory integration follows TNI per US EPA-NELAP
+ ISO/IEC 17025: method-detection-limit per 40 CFR Part 136
Appendix B; method-blank envelope; matrix-spike + matrix-spike-
duplicate envelope per the analytical method; laboratory-fortified-
blank envelope; internal-standard envelope per the method;
holding-time envelope per 40 CFR 136.3 Table II; field-quality-
control envelope (trip blank, equipment blank, field duplicate)
per the sampling SOP. Metrology-laboratory integration adds the
NMI-traceability chain per BIPM CIPM-MRA + the laboratory's
participation in international comparisons per BIPM key-comparison
database.

## A.6 References

- ISO/IEC 17025: General requirements for the competence of testing and calibration laboratories
- ISO/IEC 17034: General requirements for the competence of reference material producers
- ISO/IEC 17043: Conformity assessment — General requirements for proficiency testing
- ISO 15189: Medical laboratories — Requirements for quality and competence
- ISO/IEC Guide 98-3 (GUM): Uncertainty of measurement
- ISO/IEC Guide 99 (VIM): International vocabulary of metrology
- ICH Q2(R2): Validation of analytical procedures
- ICH Q9: Quality risk management
- ICH Q10: Pharmaceutical quality system
- USP <1058>: Analytical instrument qualification
- USP <621>: Chromatography
- 21 CFR Part 11: Electronic records; electronic signatures
- 21 CFR Part 211: Current GMP for finished pharmaceuticals
- EMA Annex 11: Computerised systems
- WHO TRS 986 / 996: Quality assurance of pharmaceuticals
- IEC 61010-1: Safety requirements for electrical equipment
- IEC 60825-1: Safety of laser products
- IAEA GSR Part 3: Radiation protection
- ICRP 103: Recommendations of the International Commission on Radiological Protection
- ASTM E2077: AnIML — Analytical Information Markup Language
- ASTM E1947: Spectroscopy data interchange
- ASTM E2655: Preventive maintenance
- IUPAC 1995: Nomenclature in evaluation of analytical methods
- IUPAC 2001: Recommendations on transport properties
- BIPM CIPM-MRA: Mutual recognition of national measurement standards
- ILAC-G24: Guidelines for the determination of calibration intervals


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/scientific-instrument/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-scientific-instrument-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/scientific-instrument-host:1.0.0` ships every scientific-instrument envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/scientific-instrument.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Scientific-instrument deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-scientific-instrument-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

弘益人間 — Benefit All Humanity.
