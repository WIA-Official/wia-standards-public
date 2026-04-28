# WIA-QUA-020 — Phase 3: Protocol

> Scientific-instrument canonical Phase 3: protocols (calibration + QC + remote-operation + safety + maintenance + data-integrity).

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


## 4. Calibration Standards

### 4.1 Mass Calibration

**Standards:**
- Caffeine (C₈H₁₀N₄O₂): 194.0804 m/z
- Reserpine (C₃₃H₄₀N₂O₉): 609.2812 m/z
- Polyethylene glycol (PEG)
- Polystyrene

### 4.2 Wavelength Calibration

**Standards:**
- Mercury lamp: 253.65, 365.01, 435.83 nm
- Neon lamp: 585.25, 640.22, 703.24 nm
- Laser lines: 532 nm, 632.8 nm, 1064 nm

### 4.3 Temperature Calibration

**Standards:**
- Indium: 156.6°C
- Tin: 231.9°C
- Lead: 327.5°C
- Zinc: 419.5°C

---



## 7. Remote Operation

### 7.1 Remote Control

**Protocols:**
- SSH for command-line access
- VNC/RDP for graphical interfaces
- REST API for programmatic control
- WebSocket for real-time data

### 7.2 Data Transfer

**Methods:**
- FTP/SFTP for large files
- rsync for synchronization
- Cloud storage (S3, Azure, GCP)
- GridFTP for high-performance transfer

---



## 8. Safety

### 8.1 Radiation Safety

**Protection:**
- Shielding (lead, concrete)
- Distance (inverse square law)
- Time minimization
- Monitoring (dosimeters, badges)

### 8.2 Electrical Safety

**Requirements:**
- Grounding
- Emergency shut-off
- Interlocks
- Isolation transformers

### 8.3 Cryogenic Safety

**Hazards:**
- Frostbite
- Asphyxiation (nitrogen, helium)
- Pressure buildup
- Embrittlement

---




---

## A.1 Calibration protocol

The calibration protocol per ISO/IEC 17025 §6.4 covers: pre-
calibration verification (instrument warm-up time, environmental
stabilisation per the instrument manual, reference-standard
acclimatisation), reference-standard preparation (CRM dilution,
volumetric verification per ISO 4787, mass verification per OIML
R 111), the calibration sequence (zero / blank / span / mid-range /
high-range with a documented number of replicates and a documented
replicate spacing), regression analysis (linear / quadratic / per
the validated calibration model), residual analysis with outlier
treatment per ISO 5725-2, drift / repeatability / reproducibility
computation per ISO 5725-3, and the calibration-certificate
generation step with the as-found and as-left tables.

## A.2 Quality-control protocol

Per laboratory run, a QC protocol covers: bracketing controls
(blank + LCS + duplicate at run start; blank + LCS at every Nth
sample; duplicate at every Mth sample; bracket QC at run end),
control-chart evaluation per Westgard rules (1-3s, 2-2s, R-4s,
4-1s, 10-x, 12-x for clinical chemistry per CLSI C24, with
documented rule selection for the assay), out-of-control response
(immediate run hold; root-cause investigation; reanalysis with
new QC; notification to the laboratory director), and the QC-data
archival envelope per ISO/IEC 17025 §7.4.7. Proficiency-testing
participation per ISO/IEC 17043 covers blind external samples per
the laboratory's accreditation cycle.

## A.3 Remote-operation protocol

Remote-operation protocols cover: operator-authentication envelope
(MFA per NIST SP 800-63B AAL2 minimum for routine work; AAL3 for
high-consequence labs), session-encryption envelope (TLS 1.3 per
RFC 8446 mandated; mutual TLS for instruments operating in
accredited scope; client certificates rooted in the laboratory's
internal PKI), latency-budget envelope (real-time-class instruments
requiring round-trip <50ms; non-real-time instruments tolerating
<2s), and the failsafe envelope for connection loss (instrument
auto-pauses on heartbeat loss, recoverable run-state held in
instrument-local storage for 24h per ISO 11611).

## A.4 Safety protocol

Safety protocols cover instrument-specific hazards: ionising-
radiation instruments (X-ray fluorescence, X-ray diffraction)
follow IAEA Safety Standards GSR Part 3 + ICRP 103 with operator
TLD + interlock chain + shielding-integrity verification quarterly;
laser instruments follow IEC 60825-1 Class 3B/4 with key-switch
+ emission-indicator + remote-interlock + beam-stop chain; high-
voltage instruments follow IEC 61010-1 with insulation-coordination
table and pollution-degree envelope; pressurised-system instruments
follow ISO 4126-1 + EN 13445 for relief valves; biohazard-handling
instruments follow ISO 35001 + WHO Laboratory Biosafety Manual
4th ed. with BSL-2 / BSL-3 / BSL-4 envelope appropriate to the
sample-class.

## A.5 Maintenance protocol

Preventive-maintenance per ASTM E2655 covers: per-instrument PM
schedule (manufacturer-recommended interval +/- the laboratory's
operational adjustment based on instrument-event-log analysis),
PM-procedure envelope (cleaning, lubrication, sensor verification,
consumable replacement, alignment verification, performance
verification with the documented acceptance criterion), as-found
performance documentation, as-left performance documentation,
service-vendor envelope (vendor-credentialed, third-party-
accredited, or operator-internal), and the PM-completion envelope
that returns the instrument to service or escalates to corrective
maintenance if performance verification fails.

## A.6 Data-integrity protocol

Data-integrity per ALCOA+ (Attributable, Legible, Contemporaneous,
Original, Accurate + Complete, Consistent, Enduring, Available)
per US FDA 21 CFR Part 11 + EMA Annex 11 + WHO TRS 996 covers:
audit-trail immutability with per-record digital signature per
RFC 5280; user-account management with role-based access per ISO
27002 §9.2 + named-user enforcement; system-clock synchronisation
to UTC via NTP per RFC 5905 with documented offset bounds; backup
+ restore envelope verified quarterly; archive envelope per the
laboratory's retention schedule (clinical labs typically 25 years;
GMP labs typically 30 years; environmental labs per the
applicable regulatory minimum).


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
