# WIA-CITY-009 (smart-lighting) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 specifies how a WIA-CITY-009 deployment integrates with the broader municipal-services ecosystem: utility energy-billing systems, asset-management platforms, traffic-control coordination, public-safety SOC handover, and sustainable-city reporting frameworks. The integration is layered: DALI-2 / D4i / Zhaga carry device-level interoperability; ANSI C12 carries metering interoperability; ISO 55001 carries asset-management interoperability; ISO 37120 carries sustainable-city reporting.

---

## 2. Bridge profiles

### 2.1 Bridge to Central Management Systems

Most cities run a central management system (CMS) for their lighting estate — Telensa Urban-IQ, Citelum Connected Lighting, Signify Interact City, Lucy Zodion Vivacity, ams OSRAM Symphony. The bridge profile maps Phase 2 endpoints to CMS-native primitives without requiring the legacy stack to be replaced.

| Phase 2 endpoint | CMS primitive | Bridge mapping |
|------------------|---------------|----------------|
| `PUT /luminaire/{id}/level` | CMS dim command | translated to CMS-native dim primitive plus DALI-2 DAPC |
| `GET /luminaire/{id}/telemetry` | CMS asset state | telemetry packets unwrapped from D4i payloads |
| `POST /schedule` | CMS scheduling engine | schedule envelope translated to CMS time-of-day or astronomic-event rules |
| `GET /fault` | CMS work-order system | faults flow into CMS work orders for contractor dispatch |

The bridge container ships at `https://github.com/WIA-Official/wia-smart-lighting-bridges/cms-bridge` with reference implementations for the five major CMS platforms.

### 2.2 Bridge to utility billing systems

The utility bridge maps Phase 2 energy-reporting envelopes to the utility's billing-system intake format. ANSI C12.19:2012 is the lingua franca for North American utilities; IEC 61968-9:2013 for European utilities; KEPCO 검침 데이터 EDI for Korean utilities. The bridge documents the per-utility field mapping and the billing-cycle cadence.

### 2.3 Bridge to asset-management platforms

The asset-management bridge maps the Phase 1 luminaire and zone descriptors to the city's enterprise asset-management (EAM) platform. Reference bridges:

- **IBM Maximo** — used by many large cities; bridge exposes EAM asset records
- **Esri ArcGIS Indoors / Utility Network** — geospatial asset platform; bridge anchors luminaire location to ArcGIS
- **Cityworks** (Trimble) — public-works asset platform; bridge maps fault flow to work-order generation
- **Korean 서울시 도시기반시설관리시스템** — Seoul-metropolitan asset management

### 2.4 Bridge to traffic-control systems

Smart lighting interacts with traffic-control infrastructure (some street-light fixtures host traffic-monitoring sensors; some traffic events drive lighting overrides for emergency response). The bridge profile maps Phase 2 zone preset commands to traffic-management-system events:

- **NTCIP 1202** — National Transportation Communications for ITS Protocol; standard interface to traffic signals
- **ISO 14813-1** — ITS service architecture; declares the interaction model
- **TMDD 3.x** — Traffic Management Data Dictionary; data exchange between TM systems

### 2.5 Bridge to public-safety SOC

When a lighting zone falls under a public-safety incident (search-and-rescue, evacuation, large gathering), the public-safety SOC may request emergency lighting overrides. The bridge profile maps the SOC's incident envelope (from WIA-CITY-014 security-system-city) to a Phase 2 zone preset command, with the audit chain extending across both standards.

---

## 3. Sustainable-city reporting integration

### 3.1 ISO 37120 indicators

ISO 37120 defines indicators for sustainable cities and communities. Smart-lighting deployments contribute to multiple indicators:

| ISO 37120 indicator | Smart-lighting contribution |
|---------------------|------------------------------|
| 7.4 Energy consumption of public buildings per capita | Aggregate energy from `/energy/report` |
| 7.5 Percentage of city's energy from renewable sources | When the deployment integrates with municipal-renewable feeds |
| 9.1 Average annual hours of electrical service interruption | Lighting outage duration from fault data |
| 9.4 Percentage of city population with authorized electrical service | Coverage of street-lighting zones across the city footprint |
| 11.2 Number of public street lights per 100 000 population | Direct from `/luminaire` count |

The ISO 37120 reporting bridge generates the indicator values directly from the operator's bulk-export feed.

### 3.2 GHG accounting integration

GHG accounting follows ISO 14064-1 organisational greenhouse-gas inventories. The smart-lighting deployment contributes Scope 2 (purchased electricity) emissions; the bridge translates the energy-report envelope to an ISO 14064-1 conformant emissions record.

### 3.3 EnerLightSmart citiesdark-sky / IDA reporting

Cities with dark-sky ordinances (IDA International Dark-Sky Association recognition, EU Light Pollution Directive 2024) require fixture-by-fixture compliance reporting. The bridge maps fixture descriptors (colour temperature, full-cutoff classification, dimming profile after midnight) to the IDA conformance template.

---

## 4. Cross-standard composition

This Phase composes with adjacent WIA-family standards:

- **WIA-OMNI-API** — operator and contractor identity (X.509 + DID); Phase 2 X.509 chain anchors against WIA-OMNI-API records when applicable
- **WIA-AIR-SHIELD** — runtime trust list and key rotation for cross-vendor federation
- **WIA-SOCIAL Phase 3 §5** — federation receipt shape reused for cross-operator handovers (e.g., one contractor maintains, another contractor operates)
- **WIA-INFRA-MONITORING** — composes when the smart-lighting deployment is part of a broader smart-city infrastructure monitoring footprint
- **WIA-CITY-014 (security-system-city)** — composes for emergency-lighting overrides triggered by public-safety incidents
- **WIA-AGRI-008 (yield-prediction)** — composes when greenhouse lighting falls under the same operator (rare but real cross-domain pattern)

---

## 5. Operational deployment runbook

A first city-scale smart-lighting deployment that reaches production typically follows the runbook:

| Phase | Activity | Duration |
|-------|----------|----------|
| Day 0 | Reference container stood up; conformance suite run | 1 day |
| Day 1-7 | Asset register imported from city EAM | 1 week |
| Day 8-30 | First DALI-2 / D4i bus segments commissioned; CMS bridge verified | 3 weeks |
| Day 31-45 | Energy-reporting bridge to utility; first billing cycle reconciled | 2 weeks |
| Day 46-60 | Schedule cascade established (city policy → zone overrides → fixture exceptions) | 2 weeks |
| Day 61-90 | Fault-flow tested against contractor dispatch; ISO 37120 reporting verified | 4 weeks |
| Day 91+ | Production cutover with shadow operation through Day 90; legacy retained as fallback | open-ended |

Lighter deployments (small towns with hundreds of fixtures) compress this to 30 days; metropolitan deployments (hundreds of thousands of fixtures) may take 12-24 months for full asset coverage.

---

## 6. Compliance and certification

The standard maps to:

- **ISO 55001:2014** — asset management
- **ISO 37120 / 37122 / 37123** — sustainable-cities indicators
- **ISO 14064-1** — greenhouse-gas accounting
- **IEC 62386 series** — DALI-2 digital control
- **IEC 60598 series** — luminaires general safety
- **IEC 60529** — IP protection ratings
- **ANSI C12.19** — utility-industry data tables
- **IEEE 802.15.4** — low-rate wireless networks (D4i)
- **NTCIP 1202 / TMDD 3.x** — ITS traffic-management interfaces
- **EU Light Pollution Directive 2024**
- **IDA International Dark-Sky Association** dark-sky ordinances

Operators publish a signed conformance attestation envelope that names which compliance frames they claim and which audit evidence supports each claim.

---

## 7. Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: DALI-2 / D4i / Zhaga / IEC 62386 stable; CMS + utility + EAM bridges |
| 1.1.x | Additive: more CMS bridges, deeper sensor-payload taxonomy for D4i |
| 1.2.x | Additive: dark-sky and circadian-friendly lighting profile; EU Light Pollution Directive 2024 native support |
| 1.3.x | Additive: V2X-coordination patterns when smart-lighting fixtures host C-V2X RSUs |
| 2.0.0 (no earlier than 2028) | Possible breaking change: post-quantum signature suite migration |

The standard is maintained by the WIA Standards Committee. Change proposals follow the WIA RFC process; breaking changes require a two-thirds Committee vote plus a 12-month deprecation window per IETF RFC 8594 / 9745.

---

## 8. References

- IEC 62386-150/-151/-251/-252/-253:2018-2020 — DALI-2 / D4i digital interfaces
- IEC 60598-1 — Luminaires general safety
- IEC 60529 — IP protection ratings
- DALI-2 v1.x — Digital Illumination Interface Alliance
- D4i Programme — DALI-2 IoT extension
- Zhaga Consortium Books 11 + 18 — luminaire-internal control + outdoor socket
- ANSI C12.19:2012 — utility data tables
- IEC 61968-9:2013 — utility metering interface
- ISO 55001:2014 — Asset management
- ISO 37120 / 37122 / 37123 — Sustainable cities indicators
- ISO 14064-1 — Organisational greenhouse-gas inventories
- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 29134:2023 — Privacy impact assessment
- IEC 62443-3-3:2013 — System security
- IEEE 802.15.4 — Low-rate wireless networks
- NTCIP 1202 — Actuated traffic signal controller
- ISO 14813-1 — ITS service architecture
- TMDD 3.x — Traffic Management Data Dictionary
- IDA International Dark-Sky Association
- EU Light Pollution Directive 2024
- IETF RFC 8446 — TLS 1.3
- IETF RFC 8594 — sunset HTTP header
- IETF RFC 9745 — deprecation HTTP header

---

## 9. Closing implementer note

Smart-lighting is one of the highest-volume but lowest-glamour smart-city deployments: every street fixture is managed; the asset count crosses six figures in capital cities; the financial savings from intelligent dimming alone justify the deployment; and the public-safety value (lighting up neighbourhoods during incidents, dimming during astronomic-friendly hours) emerges as a co-benefit. The wire-format discipline is what lets the city own the operation across multi-decade asset lifecycles without locking into a single vendor. Subsequent deployments to additional zones reuse the same machinery with per-zone schedule and policy overlays.

弘益人間 — Benefit All Humanity.


## 10. Glossary expansion

DALI-2: Digital Addressable Lighting Interface, version 2. The current open-standard digital control protocol for in-fixture communication. D4i: DALI-2 IoT extension package; adds energy reporting, sensor packets, and luminaire-information memory banks. Zhaga: open-standard luminaire-component consortium publishing form-factor specifications. CMS: Central Management System for street lighting. EAM: Enterprise Asset Management. NTCIP: National Transportation Communications for ITS Protocol. TMDD: Traffic Management Data Dictionary.

## 11. Implementer note — privacy floor for occupancy sensors

Occupancy and people-counting sensors carried in D4i payloads are a privacy-impacted data class. The standard's privacy floor (Phase 2 §1.2) refuses endpoints that would expose per-fixture per-second counts unless the deployment's published DPIA documents the purpose limitation. Aggregate counts at zone level are typically privacy-neutral; per-fixture data may not be, depending on density and re-identification risk.


## 11. Glossary expansion for Phase 4

CMS: Central Management System for street lighting. EAM: Enterprise Asset Management. NTCIP: National Transportation Communications for ITS Protocol. TMDD: Traffic Management Data Dictionary. IDA: International Dark-Sky Association. ANSI C12: American National Standards Institute electricity-metering data tables. KEPCO: Korea Electric Power Corporation. SI base units conventions follow BIPM. The lighting-controls open-standard backbone (DALI-2 + D4i + Zhaga) is intentionally the technical baseline; this Phase 4 layer is the operational integration on top of that backbone.

## 12. Implementer note — multi-decade asset lifecycle

A street-lighting installation has a 15-25 year asset lifecycle. The wire formats and protocol disciplines that operate it must absorb that horizon without locking the city into a single luminaire vendor. The DALI-2 + D4i + Zhaga combination is the open-standard backbone the city can rely on; this standard's operational layer is a thin wrapper above that backbone that lets the city manage its lighting estate without per-vendor custom integration.
