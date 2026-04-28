# WIA-SPACE-009 — Phase 3: Protocol

> Space-radiation canonical Phase 3: protocols (shielding-design + SPE-shelter + real-time-monitoring + medical-countermeasure + verification + deviation).

# WIA-SPACE-009: Space Radiation Protection Standard
## Version 1.0

**Status:** Active
**Published:** 2025-01-26
**Category:** Space Safety / Radiation Protection
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---


## 4. Shielding Requirements

### 4.1 Passive Shielding

**Habitat Shielding:**
- Minimum: 10 g/cm² aluminum equivalent
- Recommended: 20 g/cm² for sleeping quarters
- Storm shelter: 40 g/cm² polyethylene or water equivalent

**Materials Priority:**
1. Hydrogen-rich materials (polyethylene, water)
2. Multi-layer structures
3. Low-Z materials to minimize secondary radiation

**Material Specifications:**

| Material | Density (g/cm³) | Hydrogen Content | Application |
|----------|-----------------|------------------|-------------|
| Polyethylene | 0.92 | 14% | Dedicated shielding layers |
| Water | 1.0 | 11% | Multi-purpose (life support + shielding) |
| Aluminum | 2.7 | 0% | Structural (baseline) |
| Boron composite | 1.8-2.5 | Varies | Neutron capture |

### 4.2 Storm Shelter Design

**Requirements:**
- Shielding: 20-40 g/cm² effective thickness
- Capacity: All crew members
- Duration: Minimum 72 hours self-sufficient
- Location: Central area with maximum shielding
- Access: Reachable within 15 minutes from any location

**Life Support:**
- Oxygen supply
- CO₂ removal
- Temperature control
- Food and water
- Waste management
- Communications

### 4.3 EVA Suit Enhancement

**Current EMU Shielding:** ~0.1-0.2 g/cm²

**Enhancement Targets:**
- Torso: 0.5 g/cm² (critical organ protection)
- Helmet: Enhanced polycarbonate + hydrogen gel
- Flexible joints: Polyethylene fiber weave

---



## 6. Operational Protocols

### 6.1 Mission Planning

**Launch Window Selection:**
- Solar cycle consideration
- SPE probability assessment
- GCR flux prediction

**Route Optimization:**
- Van Allen belt transit: Fastest trajectory through thinnest regions
- Earth shadow utilization when possible
- SPE avoidance maneuvers (contingency)

**Mission Duration:**
- Minimize transit time (faster propulsion)
- Balance surface stay time with radiation risk
- Dose budget allocation

### 6.2 EVA Procedures

**Pre-EVA:**
1. Solar activity forecast review (48 hours)
2. Current radiation environment assessment
3. Go/No-Go decision based on:
   - No M-class or larger flares in past 24 hours
   - No active region with high flare probability
   - Background dose rate within normal range
   - SAA passage timing (ISS)

**During EVA:**
1. Continuous personal dosimeter monitoring
2. Ground-based environment monitoring
3. Time limit enforcement
4. Emergency abort criteria:
   - SPE warning received
   - Dose rate >5x background
   - Cumulative dose approaching daily limit

**Post-EVA:**
1. Dosimeter reading and recording
2. Crew health assessment
3. Cumulative dose update

**EVA Limits:**
- Single EVA: 8 hours maximum
- Weekly: 16 hours maximum
- Mission total: Based on dose budget

### 6.3 SPE Response Protocol

**Alert Levels:**

| Level | Condition | Action |
|-------|-----------|--------|
| Green | Normal | Continue operations |
| Yellow | M-class flare or dose rate 2x background | Increase monitoring, review EVA |
| Orange | X-class flare or SPE predicted | Abort EVA, prepare shelter |
| Red | SPE in progress, dose rate >10x | Immediate shelter evacuation |

**Shelter Procedures:**
1. Alert reception
2. Cease non-essential activities
3. Shelter entry (target: <15 minutes)
4. Hatch closure and verification
5. Continuous monitoring (internal/external dosimeters)
6. Ground communication
7. Duration: Until dose rate returns to <2x background

**Shelter Activities:**
- Minimize movement (energy conservation)
- Health monitoring
- Maintain communication
- Psychological support

### 6.4 Acute Radiation Syndrome (ARS) Response

**Diagnosis:**
- Symptom onset time (nausea, vomiting)
- Dose estimation (dosimeters)
- Lymphocyte count (blood test if available)

**Treatment:**
1. Antiemetics (Ondansetron)
2. Hydration (oral or IV)
3. G-CSF (if dose >2 Gy)
4. Antibiotics (prophylactic if dose >1 Gy)
5. Supportive care
6. Ground medical consultation
7. Consider early return if severe (>4 Gy)

---



## 7. Medical Countermeasures

### 7.1 Radioprotectors

**Definition:** Agents administered before exposure to prevent or reduce cellular damage

**Candidates:**
- Amifostine (WR-2721): FDA-approved, but significant side effects
- Antioxidants: Vitamin C, E, Selenium, N-acetylcysteine
- Administration: Prior to high-risk activities (EVA during elevated risk)

### 7.2 Radiation Mitigators

**Definition:** Agents administered after exposure to limit damage progression

**Primary:**
- G-CSF (Granulocyte Colony-Stimulating Factor): Bone marrow recovery
- EPO (Erythropoietin): Red blood cell production
- TPO (Thrombopoietin): Platelet production

**Administration Timeline:** Within 24-48 hours of significant exposure (>1 Gy)

### 7.3 Pharmaceutical Inventory

**Required Medications:**
- Antiemetics: Ondansetron, Metoclopramide
- Growth factors: G-CSF, EPO
- Antibiotics: Broad-spectrum
- Analgesics: Pain management
- IV fluids and electrolytes
- Antioxidants

**Storage:**
- Radiation-shielded packaging
- Temperature-controlled
- Expiration date monitoring (minimum 3-year shelf life for Mars missions)

---




---

## A.1 Shielding-design protocol

Shielding-design protocols cover: material-selection envelope
(polyethylene at 0.93 g/cm³ for hydrogenous shielding per HZETRN
analyses showing optimal GCR attenuation per gram of mass; aluminum
at 2.70 g/cm³ for primary structural shielding; water at 1.00 g/cm³
where the operator can repurpose stored-water mass; regolith
berming for surface habitats with a 1.7-2.5 g/cm³ density envelope
depending on lunar / Mars composition), mass-thickness optimisation
per HZETRN with the GCR-dose minimisation objective (typically
20-40 g/cm² for cislunar / Mars-transit cabin; >40 g/cm² for
storm-shelter), the secondary-radiation penalty assessment
(neutrons + spallation products), and the final-design verification
via Monte-Carlo transport (FLUKA, Geant4, PHITS, MCNP6 with HZE
extensions).

## A.2 SPE-shelter operational protocol

SPE-shelter operational protocols cover: alert-reception envelope
(NOAA + ESA + onboard particle-detector triggering with the
operator's threshold matrix), shelter-entry protocol (crew-
notification audio + visual; shelter-entry SOP; hatch-seal
verification; communications check), shelter-occupancy duration
envelope (typical 12-72 hours per the SPE temporal envelope; up
to 1 week for Carrington-class events), shelter-exit protocol
(post-event flux verification; cumulative-dose verification against
the SPE budget; medical-officer assessment), and the shelter-event
documentation envelope per ISS-IRD or operator equivalent.

## A.3 Real-time monitoring protocol

Real-time monitoring protocols cover: dosimeter-polling envelope
(active dosimeters polled at 1-60 s cadence; passive dosimeters
read at end-of-mission), dose-rate trending with anomaly detection
(z-score deviation from trailing 24-hour mean), correlation with
external particle-flux feeds (SWPC GOES proton + Antarctic
neutron monitor + onboard particle-detector), automatic alert
escalation envelope (operator-configurable threshold matrix), and
the medical-officer notification envelope on threshold crossing
or anomaly detection.

## A.4 Medical-countermeasure protocol

Medical-countermeasure protocols per NASA HRP-47065 cover:
pre-mission pharmaceutical envelope (vitamin D + folate + omega-3
+ retinol per the bone / cardiovascular / cognitive risk envelopes;
operator-MD-approved antioxidant supplements), in-mission
pharmaceutical envelope (anti-emetics for acute high-dose
exposures; granisetron / ondansetron per NASA medical kit;
cytokine support per filgrastim equivalent for high-dose marrow
suppression — currently research-only), and the post-mission
medical-monitoring envelope (lifetime cancer-screening commitment
per NASA Lifetime Surveillance of Astronaut Health programme; CV
+ CNS + cataract surveillance per NCRP 153 + 167).

## A.5 Verification protocol

Per-mission verification covers: pre-mission dosimeter-calibration
verification (TLD pre-irradiation + post-irradiation reference;
OSL bleach + dose-curve; CR-39 etch-rate calibration), in-mission
inter-comparison between active and passive dosimeters (TEPC vs
TLD vs CR-39 vs silicon-diode), post-mission dose reconstruction
combining all dosimeter classes, comparison of measured cumulative
dose against the §A.6 mission-budget envelope (deviations >20%
trigger root-cause investigation per ISO/IEC 17025 §7.10), and
the final crew-cumulative-career-dose update with full audit
trail per NASA NPR 1441.1G.

## A.6 Crew-protection-deviation protocol

Crew-protection-deviation protocols cover the rare cases where
mission requirements force exceedance of the §A.3 risk-criterion
envelope (e.g., emergency contingency operations, off-nominal
trajectory). The protocol requires: operator-medical-officer
hazard assessment; crew informed-consent envelope per ICMRA + the
operator's IRB; documented mission-management review and approval;
post-event individualised lifetime risk recalculation; crew-
member medical-monitoring intensification; and the lessons-
learned envelope feeding into the operator's flight-rules update
process per NASA SLDP / equivalent.


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
