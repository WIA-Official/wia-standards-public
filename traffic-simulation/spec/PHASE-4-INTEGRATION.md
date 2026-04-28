# WIA-CITY-017 (traffic-simulation) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 specifies how a WIA-CITY-017 traffic-simulation deployment integrates with the broader urban-mobility ecosystem: TMC operational tooling, V2X infrastructure deployments, mobility-as-a-service operators, transport-research labs, sustainable-city reporting, and regulatory-compliance flows. The integration is layered: SUMO / VISSIM / Aimsun / MATSim carry the simulation-engine interoperability; SAE J2735 and ETSI ITS-G5 carry the V2X interoperability; ISO 17572 carries location-reference interoperability; ISO 37120 carries sustainable-city reporting.

---

## 2. Bridge profiles

### 2.1 Bridge to simulation engines

The standard does not invent a new microsimulation engine; it provides a wire format that lets multiple engines coexist in a single deployment. The bridge profile maps Phase 1 scenario descriptors to engine-native input formats:

| Engine | Input format | Output format | Bridge artefact |
|--------|--------------|----------------|-----------------|
| Eclipse SUMO | net.xml + rou.xml + add.xml | floating-car data + summary | bridges/sumo.json |
| PTV VISSIM | INPX + signal-controller config | TRR result file | bridges/vissim.json |
| Aimsun Next | ANG project + demand | Aimsun result database | bridges/aimsun.json |
| MATSim | network.xml + plans.xml + config.xml | events.xml | bridges/matsim.json |
| Eclipse SUMO + TraCI | TraCI socket | TraCI subscription stream | bridges/sumo-traci.json |

Bridge containers ship at `https://github.com/WIA-Official/wia-traffic-simulation-bridges` with reference implementations validated against the conformance suite.

### 2.2 Bridge to TMC operational systems

TMC operations consume calibrated simulation results to evaluate signal-timing changes, work-zone plans, and incident-response strategies before deployment. The bridge profile maps Phase 2 run-result envelopes to common TMC platforms:

- **Iteris ClearGuide / iPeMS** — performance-measurement and analytics
- **TransSuite ATMS** — advanced traffic-management system (used by several US state DOTs)
- **Sumitomo Electric SmartTraffic** — Japan-domestic TMC platform
- **Korean ITS 통합관제센터** — Korea Road Traffic Authority integrated traffic-control centre
- **Siemens Mobility Sitraffic Concert** — European TMC platform

### 2.3 Bridge to V2X infrastructure

When the deployment connects to physical V2X infrastructure (RSUs, OBUs, central V2X cloud), the bridge profile maps Phase 3 V2X-injection envelopes to the physical infrastructure's V2X stack. Reference V2X stacks: 5GAA Cooperative Awareness, ETSI ITS-G5 RSUs, China C-V2X (LTE-V2X / NR-V2X) per ITU-R Recommendation M.2084.

### 2.4 Bridge to mobility-as-a-service (MaaS) platforms

MaaS platforms (Whim, Citymapper Pass, Korean 카카오모빌리티, Japanese MyRoute) consume traffic-state forecasts for trip-planning. The bridge maps Phase 2 OD-matrix and trajectory aggregates to MaaS-native trip-planning APIs without exposing per-individual data.

### 2.5 Bridge to transport-research labs

Transport-research labs publish papers based on simulation results. The bridge profile maps Phase 2 run-result envelopes plus the calibration-evidence chain (Phase 3 §4) to the lab's data-publication archive (typically Zenodo, Dryad, or a domain-specific repository like the Transport Research International Documentation database).

---

## 3. Sustainable-mobility reporting integration

### 3.1 ISO 37120 indicators

ISO 37120 defines indicators for sustainable cities and communities. Traffic-simulation deployments contribute to multiple transport indicators:

| ISO 37120 indicator | Traffic-simulation contribution |
|---------------------|----------------------------------|
| 18.1 Kilometres of high-capacity public-transport per 100 000 population | Network description from `/scenario/{id}/network` |
| 18.2 Kilometres of light passenger transit per 100 000 population | Same |
| 18.3 Annual public-transport trips per capita | OD-matrix-derived if mode-split is modelled |
| 18.4 Number of personal automobiles per capita | Demand-matrix derivation |
| 18.5 Percentage of commuters using a travel mode other than personal vehicle | Mode-split outputs |
| 18.6 Number of two-wheeled motorised vehicles per capita | Demand-matrix derivation |

The reporting bridge generates the indicator values from the operator's bulk-export feed.

### 3.2 GHG accounting integration

Transport is the largest contributor to most cities' Scope 1 emissions inventories. The bridge maps run-result envelopes to ISO 14064-1 organisational greenhouse-gas inventories, computing per-mode emissions from vehicle kilometres travelled (VKT) and per-mode emission factors. Reference emission factors: COPERT (European Environment Agency), MOVES (US EPA), HBEFA (Switzerland).

### 3.3 Vision Zero road-safety reporting

Vision Zero road-safety frameworks consume simulated-incident outputs (Phase 3 §5 incident injection) to evaluate intersection-redesign strategies before deployment. The bridge profile maps incident-injection results to Vision Zero standardised reporting templates.

---

## 4. Cross-standard composition

This Phase composes with adjacent WIA-family standards:

- **WIA-OMNI-API** — TMC-operator and research-partner identity
- **WIA-AIR-SHIELD** — runtime trust list and key rotation
- **WIA-SOCIAL Phase 3 §5** — federation receipt shape reused for inter-TMC handshakes
- **WIA-INTENT** — declaration of mobility-intent so regulators and MaaS operators can monitor at intent-level
- **WIA V2X (companion standard)** — composes when simulation drives real-world V2X infrastructure rather than only modelling it
- **WIA Smart Lighting (WIA-CITY-009)** — composes when traffic-event-driven lighting overrides need coordinated audit chains
- **WIA Security System City (WIA-CITY-014)** — composes when traffic incidents trigger public-safety SOC engagement

---

## 5. Operational deployment runbook

A first traffic-simulation deployment that reaches production typically follows the runbook:

| Phase | Activity | Duration |
|-------|----------|----------|
| Day 0 | Reference container stood up; conformance suite run | 1 day |
| Day 1-14 | Network imported from city's GIS / OpenStreetMap; scenario descriptor published | 2 weeks |
| Day 15-30 | Demand matrix calibrated against loop-detector counts; goodness-of-fit threshold met | 2 weeks |
| Day 31-45 | Signal-control plan replicated from TMC's actual operations | 2 weeks |
| Day 46-60 | First V2X-injection scenarios run against the calibrated baseline | 2 weeks |
| Day 61-90 | Inter-centre handshake with neighbouring TMC; first cross-jurisdiction scenario run | 4 weeks |
| Day 91+ | Production cutover with shadow operation through Day 90; legacy retained as fallback | open-ended |

Lighter deployments (single-corridor research projects) compress this to 30 days; metropolitan-scale deployments (full-city models) may take 6-12 months for full calibration coverage.

---

## 6. Compliance and certification

The standard maps to:

- **ISO 14813-1** — ITS service architecture
- **ISO 17572-1/-2/-3** — Location referencing
- **ISO 39001** — Road traffic safety management systems
- **ISO 37120 / 37122 / 37123** — sustainable-cities indicators
- **ISO 14064-1** — Greenhouse-gas accounting
- **ISO/IEC 27001:2022** — Information security management
- **ISO/IEC 29134:2023** — Privacy impact assessment (for OD data)
- **NTCIP 1202 / 2202** — National Transportation Communications for ITS Protocol
- **TMDD v3.x** — Traffic Management Data Dictionary
- **SAE J2735 / ETSI ITS-G5** — V2X message standards
- **NIST SP 800-53 Rev 5** — Security and Privacy Controls
- **EU GDPR Article 32** + **KR PIPA Article 29** — privacy-floor frameworks

Operators publish a signed conformance attestation envelope that names which compliance frames they claim and which audit evidence supports each claim.

---

## 7. Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: SUMO + VISSIM + Aimsun + MATSim bridges; SAE J2735 + ETSI ITS-G5 V2X bridges |
| 1.1.x | Additive: more simulation engines (e.g., Cube, TransModeler); deeper ASN.1 module coverage |
| 1.2.x | Additive: multi-modal extensions (rail, micro-mobility, walking) |
| 1.3.x | Additive: connected-and-autonomous-vehicle (CAV) interaction layer with WIA V2X composition |
| 2.0.0 (no earlier than 2028) | Possible breaking change: post-quantum signature suite migration |

The standard is maintained by the WIA Standards Committee. Change proposals follow the WIA RFC process; breaking changes require a two-thirds Committee vote plus a 12-month deprecation window per IETF RFC 8594 / 9745.

---

## 8. References

- Eclipse SUMO project documentation
- PTV VISSIM 2024 documentation
- Aimsun Next documentation
- MATSim — Multi-Agent Transport Simulation
- OpenDRIVE 1.7 — ASAM road-network description
- SAE J2735 (2024) — DSRC Message Set Dictionary
- ETSI EN 302 663 / 302 637 / TS 103 301 — ITS-G5
- ISO 14813-1 — ITS service architecture
- ISO 17572-1/-2/-3 — Location referencing
- ISO 39001 — Road traffic safety management
- ISO 37120 / 37122 / 37123 — Sustainable cities
- ISO 14064-1 — GHG accounting
- ISO/IEC 27001:2022
- ISO/IEC 29134:2023 — Privacy impact assessment
- NTCIP 1202 / 2202
- TMDD v3.x
- NIST SP 800-53 Rev 5
- COPERT — European Environment Agency emission factors
- MOVES — US EPA Motor Vehicle Emission Simulator
- HBEFA — Handbook of Emission Factors for Road Transport (Switzerland)
- IETF RFC 8446 — TLS 1.3
- IETF RFC 8594 — sunset HTTP header
- IETF RFC 9745 — deprecation HTTP header

---

## 9. Closing implementer note

Traffic simulation is the canonical "policy laboratory" for urban mobility: every signal-timing change, every road-redesign proposal, every V2X-deployment decision benefits from being evaluated in simulation before deployment. The wire-format discipline is what lets the simulation results be trusted by regulators, peer-reviewed by researchers, and consumed by downstream MaaS operators without each consumer having to re-implement engine-specific glue. Subsequent deployments to additional cities reuse the same machinery with per-city network and calibration overlays.

弘益人間 — Benefit All Humanity.


## 10. Glossary expansion

ASN.1: Abstract Syntax Notation One, used by SAE J2735 and ETSI ITS-G5 for binary message encoding. UPER: Unaligned Packed Encoding Rules, the canonical ASN.1 encoding for over-the-air V2X. RSU/OBU: Road-Side Unit / On-Board Unit, the V2X infrastructure endpoints. C-V2X: Cellular V2X (LTE-V2X / NR-V2X). PSID: Provider Service Identifier, the J2735 message-type identifier. TMDD: Traffic Management Data Dictionary. NTCIP: National Transportation Communications for ITS Protocol.

## 11. Implementer note — Vision Zero integration

Vision Zero road-safety frameworks treat traffic fatalities as preventable through systemic intervention rather than individual blame. The standard's incident-injection protocol (Phase 3 §5) feeds Vision Zero retrospective analyses by letting researchers re-simulate fatal-incident locations under counterfactual signal-timing or road-redesign assumptions. The audit log preserves the evidence chain so the analysis can be peer-reviewed without trusting the publisher's current state.


## 11. Glossary expansion for Phase 4

TMC: Traffic Management Centre, the operational hub for a metropolitan road network. MaaS: Mobility-as-a-Service, the integrated trip-planning and payment paradigm. RSU / OBU: Road-Side Unit / On-Board Unit, the V2X infrastructure endpoints. C-V2X: Cellular Vehicle-to-Everything (LTE-V2X / NR-V2X). DSRC: Dedicated Short-Range Communications, the legacy 5.9 GHz V2X technology being supplemented by C-V2X. COPERT / MOVES / HBEFA: emission-factor frameworks for European, US, and Swiss contexts respectively. Vision Zero: road-safety framework treating fatalities as preventable through systemic intervention.

## 12. Implementer note — policy laboratory discipline

Traffic simulation is the canonical "policy laboratory" for urban mobility decisions. Every signal-timing change, every road-redesign proposal, every V2X deployment plan benefits from being evaluated in simulation before deployment. The wire-format discipline ensures the policy-laboratory results are trustworthy: regulators verify the calibration evidence chain; researchers verify the goodness-of-fit metrics; downstream MaaS operators verify the privacy floor.
