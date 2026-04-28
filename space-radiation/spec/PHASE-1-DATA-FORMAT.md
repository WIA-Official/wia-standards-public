# WIA-SPACE-009 — Phase 1: Data Format

> Space-radiation canonical Phase 1: source + dose-record + risk-criterion + LET-spectrum + dosimeter + mission-budget envelopes.

# WIA-SPACE-009: Space Radiation Protection Standard
## Version 1.0

**Status:** Active
**Published:** 2025-01-26
**Category:** Space Safety / Radiation Protection
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---


## 1. Introduction

### 1.1 Purpose

WIA-SPACE-009 establishes comprehensive standards and protocols for protecting astronauts and spacecraft from space radiation during all phases of space missions, including:

- Low Earth Orbit (LEO) operations
- Lunar missions and surface operations
- Mars expeditions and deep space exploration
- Extravehicular activities (EVA)
- Emergency radiation events

### 1.2 Scope

This standard covers:

- Radiation sources (GCR, SPE, trapped radiation)
- Health effects and risk assessment
- Shielding technologies (passive and active)
- Radiation monitoring systems
- Operational protocols and procedures
- Medical countermeasures
- Mission planning considerations

### 1.3 Applicable Documents

- ICRP Publications on radiation protection
- NASA STD-3001 (Space Flight Human System Standard)
- ISO 15390 (Space environment - Galactic cosmic rays)
- NCRP Reports on space radiation

---



## 2. Radiation Sources

### 2.1 Galactic Cosmic Rays (GCR)

**Characteristics:**
- Composition: ~85% protons, ~14% helium, ~1% HZE particles
- Energy range: Hundreds of MeV to hundreds of GeV
- Continuous exposure throughout mission
- Modulated by solar cycle (30-50% variation)

**Risk Level:** High for long-duration missions (>6 months)

**Shielding Difficulty:** Very High - requires meters of shielding for complete protection

### 2.2 Solar Particle Events (SPE)

**Characteristics:**
- Composition: >90% protons
- Energy range: Tens to hundreds of MeV
- Duration: Hours to days
- Frequency: Correlated with solar cycle

**Risk Level:** Very High during events (acute radiation syndrome possible)

**Warning Time:** 10 minutes to several hours

### 2.3 Trapped Radiation (Van Allen Belts)

**Inner Belt:**
- Altitude: 1,000-6,000 km
- Primary particles: High-energy protons (10-hundreds of MeV)
- Stability: Very stable

**Outer Belt:**
- Altitude: 13,000-60,000 km
- Primary particles: Electrons (0.1-10 MeV)
- Stability: Highly variable

**South Atlantic Anomaly (SAA):**
- Location: ~30°S, 60°W
- Contributes 70-80% of ISS astronaut dose
- Transit frequency: 5-6 times/day on ISS orbit

---



## 3. Dose Limits and Risk Criteria

### 3.1 NASA Dose Limits

**Career Limits (3% Risk of Exposure-Induced Death):**
- Age 25 Female: 600 mSv
- Age 25 Male: 800 mSv
- Age 35 Female: 900 mSv
- Age 35 Male: 1,000 mSv
- Age 45 Female: 1,100 mSv
- Age 45 Male: 1,200 mSv

**Short-term Limits:**
- 30 days: 250 mSv
- Annual: 500 mSv

**Organ-specific Limits:**
- Eye lens (career): 2,000 mSv (cataract prevention)
- Skin (30 days): 6,000 mSv (acute skin damage prevention)

### 3.2 ALARA Principle

All radiation exposure shall be kept As Low As Reasonably Achievable through:
- Time: Minimize exposure duration
- Distance: Maximize distance from radiation sources
- Shielding: Use appropriate shielding materials

---




---

## A.1 Radiation-source-record envelope

The Phase 1 envelope groups space-radiation sources by physical
mechanism (galactic cosmic rays — GCR, ~85% protons + ~14% alpha
+ ~1% HZE per NCRP 132; solar particle events — SPE, primarily
protons during solar maximum per NASA-STD-3001 Vol 1; trapped
radiation — Van Allen belts inner proton + outer electron per AE-9
/ AP-9 models; secondary radiation — neutrons + spallation products
per HZETRN 2015; gamma-ray bursts — extragalactic per Fermi-GBM
catalogue) with the canonical fields: source identifier, energy
spectrum (LET — linear energy transfer, in keV/μm; particle-type
breakdown; flux-energy distribution), temporal envelope (steady-
state for GCR; event-driven for SPE with onset-time + peak-time
+ duration; orbital for trapped radiation depending on altitude
and inclination), and the spatial envelope (heliocentric distance,
geomagnetic-cutoff rigidity per IGRF-13, shielding-thickness
context).

## A.2 Dose-record envelope

A dose-record envelope MUST list: dose identifier (UUID v7 per RFC
9562), crew-member reference (with privacy-compliant identifier
per HIPAA + NASA Privacy Act), measurement-instrument reference
(linking to the §A.5 dosimeter envelope), absorbed-dose value in
gray (Gy) per ICRU 33, equivalent-dose value in sievert (Sv) using
the ICRP 103 radiation weighting factors w_R, effective-dose
value in Sv using ICRP 103 tissue weighting factors w_T, organ-
dose breakdown for high-w_T organs (lungs, colon, stomach, breast,
gonads, red marrow, thyroid), measurement-uncertainty envelope per
ISO/IEC Guide 98-3, and the audit envelope tied to the mission
operator (NASA, ESA, JAXA, CSA, Roscosmos, CNSA).

## A.3 Risk-criterion envelope

Risk-criterion descriptors follow NCRP 132 + NCRP 153 + NCRP 167:
career-limit envelope (NASA permissible-exposure-limit table by
age + sex from NASA-STD-3001 Vol 1, ranging from 0.4 Sv at age
25 female to 4.0 Sv at age 55 male per the 3% REID risk
threshold), 30-day organ-dose limits (BFO 250 mGy-eq; lens 1000
mGy-eq; skin 1500 mGy-eq), annual organ-dose limits (BFO 500
mGy-eq; lens 2000 mGy-eq; skin 3000 mGy-eq), the SPE acute-
exposure envelope, the central-nervous-system risk envelope per
NCRP 153, and the cardiovascular risk envelope per NCRP 167. The
ICRP-recommended career limits in ICRP 103 differ from NASA
limits and are explicitly cross-referenced.

## A.4 LET-spectrum envelope

LET-spectrum envelopes capture the full LET-energy distribution
crossing the dosimeter or simulating a tissue-volume target: bin
boundaries (0.1, 1, 10, 100, 1000 keV/μm at minimum, often 100+
log-spaced bins), particle-type-resolved flux (proton vs alpha vs
HZE separately), depth-resolved spectrum (entrance, target, exit
for each shielding configuration), Q-factor distribution per
ICRP 60 / ICRP 103 / NASA Q(LET) / NCRP-Q (these differ for HZE
above 100 keV/μm and the choice MUST be documented), and the
cumulative-distribution envelope for risk-relevant LET ranges.

## A.5 Dosimeter-record envelope

Dosimeter-record envelopes catalogue: passive integrating dosimeters
— TLD per IAEA-TECDOC-1126 (LiF:Mg,Ti and LiF:Mg,Cu,P), OSL per
NIST-traceable AlS2O3:C, CR-39 PNTD per NASA JSC-PADLES, Bubble
detectors per BD-PND for thermal neutrons; active dosimeters —
TEPC per NASA RAM, silicon-diode arrays per ESA EuTEF, scintillator
+ photomultiplier per ISS-RAD, plasma-magnetic-spectrometer per
GOES SEISS; spacecraft dosimeters — Liulin per BAS / Bulgarian
Academy on multiple ISS missions; phantoms — MATROSHKA-R / -K per
DLR ISS-EXPRESS plus tissue-equivalent phantoms per ICRP 110.
Each dosimeter record carries calibration envelope per ISO/IEC
17025, traceability chain to NIST or PTB primary standards, and
the deployment-position envelope (intra-vehicular, extra-vehicular,
behind-shielding designation).

## A.6 Mission-radiation-budget envelope

Mission-radiation-budget envelopes are computed pre-mission for
each crew member: trajectory envelope (orbital LEO at 400-km
altitude / cislunar / lunar surface / Mars-transit / Mars-surface
with the corresponding GCR + SPE + trapped-radiation context),
shielding-configuration envelope (spacecraft hull mass-thickness
per material; emergency-shelter mass-thickness; surface-habitat
regolith berming mass-thickness if applicable), GCR-dose projection
per HZETRN with the chosen solar-modulation scenario (solar min
worst-case for GCR), SPE-dose projection per the chosen reference
event (1972 August 4 or 1989 October worst-case, or 2003 Halloween
recent-history), trapped-radiation projection per AE-9/AP-9 with
the chosen confidence percentile (typically 95th percentile mean
for design), and the final mission-cumulative-dose envelope per
mission segment.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/space-radiation/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-space-radiation-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/space-radiation-host:1.0.0` ships every space-radiation envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/space-radiation.sh` ships sample envelope generators with no
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
ecosystem. Space-radiation deployments that follow this layering
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
`/.well-known/wia-space-radiation-capabilities` that enumerates which
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
