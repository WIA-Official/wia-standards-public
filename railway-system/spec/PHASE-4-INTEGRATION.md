# WIA-AUTO-018 — Phase 4: Integration

> References plus the worked example appendix that grounds the operational design.

## 12. References

### 12.1 International Standards

1. **ETCS:**
   - ERA ERTMS/ETCS Baseline 3 (Subset-026 to Subset-119)
   - UNISIG SUBSET-026: System Requirements Specification
   - EN 50126: Railway applications - RAMS
   - EN 50128: Railway applications - Software for railway control systems
   - EN 50129: Railway applications - Safety related electronic systems

2. **CBTC:**
   - IEEE 1474.1: Standard for CBTC Performance and Functional Requirements
   - IEC 62290: Railway applications - Urban guided transport management and command/control systems

3. **PTC:**
   - 49 CFR Part 236 Subpart I: Positive Train Control Systems
   - AREMA Communications & Signals Manual

4. **Communication:**
   - EIRENE FRS: Functional Requirements Specification for GSM-R
   - FRMCS: Future Railway Mobile Communication System specifications
   - 3GPP TS 22.289: Mission Critical services

### 12.2 Safety Standards

1. **IEC 61508**: Functional safety of electrical/electronic/programmable electronic safety-related systems
2. **EN 50129**: Railway applications - Communication, signalling and processing systems - Safety related electronic systems for signalling
3. **CENELEC**: European Committee for Electrotechnical Standardization railway standards

### 12.3 WIA Standards

- **WIA-INTENT**: Intent-based journey planning and ticketing
- **WIA-OMNI-API**: Universal API gateway for railway systems
- **WIA-SOCIAL**: Social coordination for commuters and travel groups
- **WIA-IOT**: IoT sensors for predictive maintenance
- **WIA-ENERGY**: Energy optimization and regenerative braking
- **WIA-QUANTUM**: Quantum-safe encryption for railway communications

### 12.4 Industry Organizations

1. **UIC**: International Union of Railways
2. **ERA**: European Union Agency for Railways
3. **UITP**: International Association of Public Transport
4. **IEEE**: Institute of Electrical and Electronics Engineers (CBTC standards)
5. **AREMA**: American Railway Engineering and Maintenance-of-Way Association

---


## Appendix A: Example Calculations

### A.1 Metro Line Capacity

```
Given:
- Average speed: 40 km/h = 11.11 m/s
- Headway: 90 seconds
- Dwell time: 30 seconds

Calculation:
- Trains per hour = 3600 / 90 = 40 trains/hour
- With 1000 passengers per train:
- Capacity = 40 × 1000 = 40,000 passengers/hour/direction

Peak capacity: 80,000 passengers/hour (both directions)
```

### A.2 High-Speed Rail Energy

```
Given:
- Train mass: 400 tonnes = 400,000 kg
- Speed: 300 km/h = 83.33 m/s
- Distance: 100 km

Calculation:
- Kinetic energy: E = 0.5 × 400,000 × 83.33²
  E = 1,388,888,900 J ≈ 1,389 MJ ≈ 386 kWh

- Resistance energy (approx 80 kWh/km at 300 km/h):
  E_resistance = 80 × 100 = 8,000 kWh

- Total energy ≈ 8,400 kWh
- With 70% efficiency: 12,000 kWh consumed
- With 30% regeneration: 8,400 kWh net consumption

Average: 84 kWh per km
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-018 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*


## A.1 Bridge to legacy signalling

A railway operator rarely greenfields its signalling. The bridge
profile maps the standard's envelopes to legacy systems:

- **ETCS Level 2 / Level 3** (Europe + global high-speed)
- **CBTC** (urban metro)
- **PTC** (US Class I freight + passenger)
- **ATC** (Japan Shinkansen)
- **KTCS** (Korean railway)

Each bridge ships translation libraries at
`https://github.com/WIA-Official/wia-railway-bridges`.

## A.2 Passenger information integration

The passenger-information envelope flows from the operations centre
through onboard displays, station displays, mobile-app feeds, and
journey-planner aggregators. The envelope carries multilingual
content with BCP 47 language tags and i18n-friendly time/date
formatting.

## A.3 Compliance and regulatory mapping

The standard maps to: TSI (Technical Specifications for
Interoperability) for EU railways, FRA Safety Regulations for the
US, ORR's RIS for the UK, JR's TRTA for Japan, and KORAIL's safety
guidelines for South Korea.

## A.4 References

- ISO 22163 — Railway-quality management
- ISO/IEC 25023 — software product quality
- IEC 62278 (RAMS) — Reliability, Availability, Maintainability, Safety
- IEC 62280 — Railway communications, signalling and processing systems
- IEC 62425 — Communication, signalling and processing systems — safety related electronic systems
- BIPM SI brochure — TAI conventions
- W3C DID Core — decentralised identifiers


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/railway-system/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-railway-system-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/railway-system-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/railway-system.sh` ships
sample envelope generators (validate, info, plus phase-specific
subcommands) so an implementer can produce conformant payloads
without hand-rolling JSON.

## Z.4 Implementation runbook

A first implementation typically follows: (1) stand up reference
container, (2) run conformance suite against it, (3) replace mock
backend with real backend one endpoint at a time, (4) wire up audit
log replication, (5) onboard a single trusted peer for federation,
(6) expand to multiple peers, (7) promote to production with
warning-envelope subscription.

## Z.5 Backwards-compatibility promise + governance

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable.
Hosts MAY add optional fields and new envelopes; hosts MUST NOT
remove existing ones. Breaking changes ride a major version bump
with a 12-month deprecation window per IETF RFC 8594 / 9745, and
require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.

## A.5 Cross-standard composition for railway operators

A modern railway operator typically runs alongside this standard:

- **WIA-OMNI-API** for staff identity (drivers, dispatchers,
  maintenance crews)
- **WIA-AIR-SHIELD** for runtime trust list and key rotation
- **WIA-SOCIAL Phase 3 §5** for cross-operator federation
- **WIA-INTENT** for declaring journey-level intent
- **WIA-INFRA-MONITORING** for asset-condition telemetry

The composition lets a single operator reuse one identity and
audit machinery across the entire stack rather than per-system.

## A.6 Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: signalling, ATO, PSD, dynamics stable |
| 1.1.x | Additive: ETCS Level 3 reference profile, more PTC bridges |
| 1.2.x | Additive: digital-twin integration for predictive maintenance |
| 2.0.0 | Possible breaking change: post-quantum signature suite migration |

## A.7 Reference deployment guide

A first deployment typically targets a single line with full ETCS
Level 2 / CBTC instrumentation. The reference deployment guide
documents the per-trackside-node hardware requirements, the data-
centre redundancy expectations, and the operations runbook for the
first 90 days.



## A.8 Cross-standard integration table

| Adjacent system | Bridge artefact |
|----------------|------------------|
| ETCS Level 2 / 3 | bridges/etcs.json |
| CBTC | bridges/cbtc.json |
| PTC (US) | bridges/ptc.json |
| ATC (Japan) | bridges/atc.json |
| KTCS (Korea) | bridges/ktcs.json |
| GTFS / GTFS-Realtime | bridges/gtfs.json |

## A.9 Closing note

Railway operations are among the highest-stakes infrastructures in
modern society. The standard exists to make the operational data
that already flows between trains, trackside, dispatchers, and
regulators flow on a wire format that auditors and operators can
both rely on without re-inventing per-vendor glue.


## A.10 Glossary expansion

Movement authority: signed envelope from the trackside to the train
permitting motion up to a specified location. ATO (Automatic Train
Operation): automation grade defined by IEC 62290; GoA 3 / GoA 4
operate without an attended driver. PSD (Platform Screen Door):
physical barrier between platform and track with documented
interlock with the train. RBC (Radio Block Centre): trackside
component issuing movement authority in ETCS. EVC (European Vital
Computer): on-board safety computer in ETCS deployments.

## A.11 Cross-standard composition for railway operators

A modern operator runs alongside this standard: WIA-OMNI-API for
staff identity, WIA-AIR-SHIELD for trust list, WIA-SOCIAL Phase 3
§5 for cross-operator federation, WIA-INTENT for journey-level
intent, and WIA-INFRA-MONITORING for asset-condition telemetry.

## A.12 Implementation tip — multi-vendor signalling fleet

A railway operator with rolling stock from multiple vendors typically
runs multiple signalling systems in parallel: ETCS Level 2 on
high-speed lines, CBTC on metro, legacy track-circuits on older
lines. The standard's bridge profile maps all three to the same
envelope shape so the operations centre sees one unified view.


## Z.1 Worked deployment trace

A typical first-90-day deployment of this standard at a host
operator follows the trace below:

```
Day 0    : Reference container stood up in dev environment.
Day 1-2  : Conformance suite passes against reference container.
Day 3-7  : Backend bridge implemented for the host's primary tool.
Day 8-10 : Conformance suite passes against bridged backend.
Day 11-15: Audit log replication wired up; first envelope chain audited.
Day 16-20: First federation peer onboarded; trust list cadence verified.
Day 21-30: Federation expanded to 3-5 peers; cross-peer audit verified.
Day 31-60: Production traffic shadow-routed through new stack.
Day 61-90: Cutover from legacy to new stack; legacy retained as
           fallback for the deprecation window.
```

The 90-day timeline accommodates conformance-suite passes,
operations-team training, and the regulator-notification cadence
typical for high-stakes deployments. Lighter deployments (small
operators, prototypes) compress this to 30 days.

## Z.2 Operations runbook excerpt

Day-to-day operations focus on three signals: (a) audit-log
replication lag — alarm if either replica falls more than 60s
behind the primary; (b) trust-list freshness — alarm 7 days
before any peer's signed list expires; (c) replay-cache footprint
— alarm if cache memory exceeds 80% of the documented budget.

The runbook also covers incident response: rotating signing keys
on suspected compromise, replaying the seen-nonce cache from
persistent storage on standby failover, and re-issuing federation
handshakes when the primary controller has been offline for
longer than the seen-nonce window.
