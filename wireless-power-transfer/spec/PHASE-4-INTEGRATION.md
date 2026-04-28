# WIA-COMM-009 — Phase 4: Integration

> Wireless-power-transfer canonical Phase 4: ecosystem integration (WPC + AirFuel + SAE + IEC + ICNIRP + IEC 60825-1 + ITU-R).

# WIA-COMM-009: Wireless Power Transfer Specification v1.0

> **Standard ID:** WIA-COMM-009
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Fundamental Principles](#2-fundamental-principles)
3. [Inductive Power Transfer (IPT)](#3-inductive-power-transfer-ipt)
4. [Capacitive Power Transfer (CPT)](#4-capacitive-power-transfer-cpt)
5. [Resonant Wireless Power](#5-resonant-wireless-power)
6. [Microwave Power Transfer](#6-microwave-power-transfer)
7. [Laser Power Beaming](#7-laser-power-beaming)
8. [EV Wireless Charging](#8-ev-wireless-charging)
9. [Space Solar Power](#9-space-solar-power)
10. [Safety and Regulations](#10-safety-and-regulations)
11. [Implementation Guidelines](#11-implementation-guidelines)
12. [References](#12-references)

---


## 12. References

### 12.1 Standards

1. **Wireless Power Consortium**, "Qi Specification v1.3," 2021
2. **AirFuel Alliance**, "Resonant Wireless Power Transfer Specification," 2020
3. **SAE International**, "SAE J2954: Wireless Power Transfer for Light-Duty Plug-In/Electric Vehicles," 2020
4. **ISO**, "ISO 19363: Electrically propelled road vehicles - Magnetic field wireless power transfer - Safety and interoperability requirements," 2020
5. **IEC**, "IEC 61980: Electric vehicle wireless power transfer systems," 2015

### 12.2 Safety & EMC

6. **FCC**, "Part 15: Radio Frequency Devices," CFR Title 47
7. **ICNIRP**, "Guidelines for Limiting Exposure to Electromagnetic Fields (100 kHz to 300 GHz)," 2020
8. **IEEE**, "IEEE C95.1: Standard for Safety Levels with Respect to Human Exposure to Electric, Magnetic, and Electromagnetic Fields," 2019
9. **CISPR**, "CISPR 11: Industrial, scientific and medical equipment - Radio-frequency disturbance characteristics," 2015

### 12.3 Technical Literature

10. **Kurs, A. et al.**, "Wireless Power Transfer via Strongly Coupled Magnetic Resonances," *Science*, 317(5834), 2007
11. **Brown, W. C.**, "The History of Power Transmission by Radio Waves," *IEEE Trans. Microwave Theory Tech.*, 1984
12. **Shinohara, N.**, "Beam Control Technologies with a High-Efficiency Phased Array for Microwave Power Transmission," *IEEE Trans. Industrial Electronics*, 2013
13. **Landis, G. A.**, "Laser Power Beaming for Lunar Polar Exploration," *AIAA Space 2007*
14. **Covic, G. A., Boys, J. T.**, "Modern Trends in Inductive Power Transfer for Transportation Applications," *IEEE J. Emerging Technologies*, 2013

### 12.4 WIA Integration

15. **WIA-OMNI-API**: Universal energy transfer API integration
16. **WIA-INTENT**: Intent-based power request protocols
17. **WIA-IOT**: IoT device wireless charging standards

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*



---

## A.1 Standards cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| Low-power consumer            | WPC Qi v1.3 / v2.0 (cross-ref WIA-COMM-008)|
| Resonant consumer             | AirFuel Alliance specifications           |
| EV static WPT                 | SAE J2954                                 |
| EV WPT communications         | SAE J2847/6                               |
| EV WPT (EU)                   | IEC 61980-1 / -2 / -3                     |
| EMC                           | CISPR 11 / CISPR 25 / EN 301 489          |
| EMC immunity                  | IEC 61000-4 series                        |
| RF (US)                       | FCC 47 CFR Part 15 / Part 18              |
| RF (EU)                       | RED Directive 2014/53/EU                  |
| EMF — public exposure         | ICNIRP 2010 + 2020                        |
| EMF — RF exposure             | IEEE C95.1-2019                           |
| Implantable medical devices   | ISO 14117                                 |
| Laser safety                  | IEC 60825-1                               |
| Microwave / RF allocations    | ITU-R Radio Regulations                   |
| Atmospheric loss models       | ITU-R P.676 / P.838                       |
| Battery-charger safety        | IEC 62368-1                               |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 Microwave / laser regulatory integration

Microwave-beaming regulatory integration captures the operator's licence under the appropriate national regulator (FCC Experimental + Part 18 in the US for ISM-band; Ofcom 28 GHz industrial-test in the UK; equivalents in DE BNetzA / FR ANFR / KR KCC / JP MIC / CN MIIT). The integration envelope captures the licensed frequency, the licensed Tx power and beam-steering envelope, the deployment-site approval, the post-licensing compliance-monitoring envelope, and the renewal-and-reporting cadence. Laser-beaming integration adds the laser-class-licensing envelope per the operator's national laser-product safety regime (US FDA CDRH for laser products; EU Directive 2006/25/EC for occupational laser exposure; KR Radiation Health Act).

## A.3 EV charging-infrastructure integration

EV-charging integration captures the per-station envelope (street-side / parking-lot / depot installation), the meter-of-record envelope, the user-authentication envelope (RFID / NFC / app / vehicle-credential), the OCPP integration for back-office billing per Open Charge Alliance OCPP 2.0.1, the load-management envelope for grid-side coordination per IEC 61851 / SAE J3072 / ISO 15118 plug-and-charge, and the bidirectional V2G envelope where the deployment supports backflow to the grid. Cross-reference WIA-AUTO-021 vehicle-lightweight-material for the EV-side receiver-of-record envelope.

## A.4 Cross-jurisdiction safety integration

Cross-jurisdiction safety integration honours the per-jurisdiction EMF and laser-exposure rules (ICNIRP / FCC OET 65 / EU Directive 2013/35/EU on workplace EMF / KR RAC standards), the per-jurisdiction implantable-medical-device interaction policy (ISO 14117 cross-walk; FDA + KFDA + PMDA + NMPA equivalent guidance), and the per-jurisdiction occupational-exposure framework (ICNIRP basic-restrictions + reference-levels at the operating frequency). Public-facing systems carry the EMF-test report referencing the operator's accredited test laboratory per ISO/IEC 17025.

## A.5 Future directions

Active research tracks: in-room ambient power-delivery to multiple devices via beamformed-RF; multi-frequency cooperative WPT systems blending IPT for high-power and resonant for spatial freedom; laser-WPT for BVLOS UAS recharging in flight; bi-directional V2G WPT through resonant link; satellite-to-Earth power-beaming at multi-MW scale; ultra-thin metasurface receivers for embedded-WPT in walls and floors; cooperative-multipoint WPT where multiple ground transmitters collaborate to deliver power to a moving vehicle. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4. **The standard explicitly does NOT certify commercial space-solar-power, public laser-beaming above Class 1, or megawatt-class microwave-beaming services** — these remain research-track only.

## A.6 Reference list

- WPC Qi v1.3 / v2.0 — wireless power consumer specification (cross-ref WIA-COMM-008)
- AirFuel Alliance Resonant Wireless Power specifications
- SAE J2954 — light-duty EV WPT
- SAE J2847/6 — EV WPT communications
- IEC 61980-1 / -2 / -3 — EU-harmonised EV WPT
- IEC 61851 / SAE J1772 / ISO 15118 — conductive EV charging (cross-ref)
- CISPR 11 / CISPR 25 — EMC for ISM and vehicle
- ETSI EN 301 489 / EN 300 330 — EMC and short-range devices
- FCC 47 CFR Part 15 / Part 18 — radio frequency device rules
- EU RED Directive 2014/53/EU — radio equipment
- ICNIRP guidelines (2010 + 2020) — limiting exposure to time-varying fields
- IEEE C95.1-2019 — electromagnetic exposure safety levels
- ISO 14117 — Active implantable medical devices EMC
- IEC 60825-1 — Safety of laser products
- IEC 62368-1 — AV/IT/CT equipment safety
- ITU-R Radio Regulations
- ITU-R P.676 / P.838 — atmospheric attenuation models
- Open Charge Alliance OCPP 2.0.1 — Open Charge Point Protocol


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/wireless-power-transfer/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-wireless-power-transfer-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/wireless-power-transfer-host:1.0.0` ships every wireless-power-transfer envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/wireless-power-transfer.sh` ships sample envelope generators with no
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
ecosystem. Wireless-power-transfer deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
