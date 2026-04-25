# WIA-CITY-017 (traffic-simulation) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 covers the integration of WIA-CITY-017 traffic-simulation deployments with surrounding traffic-management, mobility-data, public-safety, and information-security ecosystems. The scope includes:

1. Traffic-management centre (TMC) integration.
2. Hardware-in-the-loop (HiL) and software-in-the-loop (SiL) integration with controllers and roadside modules.
3. Connected and cooperative ITS integration (V2I / I2V / V2N).
4. Mobility-data interoperability (open mobility data, multimodal feeds).
5. Information security and privacy regimes.
6. Cross-discipline integration with other WIA-CITY-* standards.
7. Conformance testing, calibration, and field validation.

---

## 2. Traffic-Management Centre (TMC) Integration

### 2.1 Read-only TMC view

A WIA-CITY-017 deployment intended to support TMC operations MUST expose a read-only view of:

- Live SPaT/MAP messages (Phase 3 §3).
- Current Phase-1 *Run* state with last-known *Result* metrics.
- Aggregated link-level vehicle counts and queue-length estimates.

The view is delivered through the Phase-2 HTTP surface with an audience claim restricted to the TMC's identity domain.

### 2.2 Operator hand-over

Operator actions in the TMC that affect the engine (manual override, scenario load, run launch) MUST trigger a Phase-1 *Run* update with the operator identity recorded in the audit log per IEC 62443-3-3 SR 6.1.

### 2.3 Latency budget

Live operations against a TMC MUST respect:

- 95th-percentile end-to-end latency from engine event to TMC view ≤ 200 ms.
- Critical events (preemption, lockdown, emergency-vehicle priority) ≤ 100 ms.

---

## 3. Hardware- and Software-in-the-Loop Integration

### 3.1 HiL signal-controller binding

The engine MAY be bound to a real signal controller via the ISO 19082:2025 data-frame interface. In this configuration:

- The engine emits *Phase request* and *Detector input* frames.
- The controller emits *Operational state* and *Phase state* frames.
- The engine consumes the controller's outputs as ground-truth for the bound intersection.

The binding MUST be authenticated per Phase 3 §7.

### 3.2 SiL controller binding

When the engine is bound to a software simulator of the controller, the simulator MUST present an ISO 19082-conformant interface so that swap-out to a real controller does not change the engine's behaviour.

### 3.3 Vehicle-in-the-loop

Driving-simulator integration (a real or simulated vehicle exchanging V2X messages with the engine) follows ISO/TS 19091 message semantics; the engine SHALL not assume a specific PHY/MAC layer.

---

## 4. Connected and Cooperative ITS Integration

### 4.1 V2I / I2V

The engine integrates as the I2V infrastructure side, emitting MAP and SPaT messages that real or simulated vehicles consume. The engine SHALL preserve the message schedules and content rules of ISO/TS 19091.

### 4.2 V2N (vehicle-to-network)

V2N integration relies on a cellular-bearer agnostic JSON or CBOR message exchange. The engine MAY emit aggregate intersection state to a cloud aggregator at a configurable cadence, and MAY consume cloud-provided demand updates as Phase-1 *Demand* objects.

### 4.3 Cooperative perception

Engines that participate in cooperative-perception studies (where vehicles share sensor observations through the engine) MUST treat the cooperative-perception payload as informative reference and MUST NOT use it to override authoritative simulation state.

---

## 5. Mobility Data Interoperability

### 5.1 Open mobility feeds

The engine MAY ingest open mobility feeds (multimodal route data, real-time transit feeds) for demand calibration. Ingest MUST treat external data as informative reference and MUST log provenance per ISO 19115-1.

### 5.2 Microtransit and shared mobility

Shared-mobility demand (carsharing, micromobility, on-demand transit) is represented as Phase-1 *Demand* objects with a `vehicleClass` set to the appropriate token. The engine MUST honour the vehicle-class permitted-classes constraint of the affected lanes (Phase 1 §2.2).

### 5.3 Freight and logistics

Freight demand is represented similarly with `vehicleClass=truck` and an optional ISO 6346 container reference for projects studying intermodal freight movement.

---

## 6. Information Security and Privacy

### 6.1 ISO/IEC 27001 alignment

A WIA-CITY-017 deployment that handles real-world personal data (vehicle traces, operator identities) MUST be operated within an ISO/IEC 27001:2022 ISMS. The Phase-1 *SimulationProject* descriptor records the ISMS scope.

### 6.2 ISO/IEC 27701 privacy

Engines that ingest real-world traces MUST extend the ISMS to ISO/IEC 27701:2019 PIM. The privacy-policy field of Phase 1 §5 MUST be set and enforced.

### 6.3 IEC 62443 alignment

Engines bound to operational technology (controllers, roadside modules) MUST operate at IEC 62443-3-3 SL ≥ 2 in the bound zone. SL-3 is recommended for projects bound to safety-critical controllers.

### 6.4 Cryptographic algorithms

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS / DTLS | TLS 1.3 | RFC 8446 / RFC 9147 |
| OSCORE | AES-CCM-16-64-128 | RFC 8613 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| Trace encryption | AES-256-GCM | ISO/IEC 18033-3 |
| Random number generation | Cryptographically secure | NIST SP 800-90A (informative) |

### 6.5 Identity provisioning

- **Engines and modules**: X.509 v3 (RFC 5280) certificates from the project PKI.
- **Operators**: OAuth 2.1 (RFC 9700) federated identity with multi-factor authentication.

---

## 7. Cross-Discipline Integration

### 7.1 WIA-CITY-009 (smart-lighting)

Coordinated lighting along simulated routes integrates with WIA-CITY-009 by exchanging a low-bandwidth feed of expected demand levels per zone. The engine emits demand levels as a Phase-1 *Sensor*-style feed; the lighting gateway consumes the feed as a control input.

### 7.2 WIA-CITY-014 (security-system-city)

Incident-driven traffic re-routing reads alarms from a federated WIA-CITY-014 site. When an alarm of `category=INTRUSION|FIRE` and `severity=CRITICAL` is reported, the engine MAY load a contingency scenario; loading is gated by a privileged operator action.

### 7.3 WIA-CITY-008 (smart-parking)

Parking-supply availability is consumed as a Phase-1 *Demand* attribute. The engine SHALL NOT command parking-system actions; it MAY recommend them through the TMC view.

### 7.4 WIA-CITY-006 (public-transit)

Real-time transit feed (bus locations, dwell times) is consumed as informative reference and used to calibrate dwell-time distributions in mesoscopic and microscopic simulations.

---

## 8. Calibration and Validation

### 8.1 Calibration

Engines MUST support calibration against observed traffic counts and travel times. The calibration workflow:

1. Ingest observation data with provenance per ISO 19115-1.
2. Optimise model parameters subject to a documented loss function.
3. Emit a Phase-1 *Result* object with the calibration metrics.
4. Sign the calibration result with COSE_Sign1 for audit.

### 8.2 Validation

Validation MUST use *held-out* observation data not used during calibration. The validation report MUST include:

- GEH statistic (a standard goodness-of-fit measure used in transportation-engineering practice).
- Mean absolute percentage error on counts.
- Travel-time RMSE per O/D pair.

The report is stored alongside the calibration result and surfaced through the Phase-2 endpoint `/projects/{id}/validation-reports`.

---

## 9. Field Commissioning Checklist

A WIA-CITY-017 deployment MUST pass each of the following before being declared *operational* in a real-world binding:

| # | Test | Reference |
|---|------|-----------|
| 1 | GDF round-trip fidelity ≥ 99% on canonical suite | Phase 3 §2.3 |
| 2 | ISO/TS 19091 MAP/SPaT export passes byte-wise conformance vectors | ISO/TS 19091 |
| 3 | ISO 19082:2025 data frames pass conformance | ISO 19082 |
| 4 | TLS 1.3 cipher inventory verified on all surfaces | RFC 8446 |
| 5 | OAuth 2.1 token introspection produces audit entries | RFC 9700, RFC 7662 |
| 6 | Multi-factor enforced for operator login | ISO/IEC 27002 A.8.5 |
| 7 | NTPv4 + NTS time-sync ≤ 50 ms vs. reference | RFC 5905, RFC 8915 |
| 8 | Real-time SPaT cadence 100 ms 95th percentile | Phase 3 §3.2 |
| 9 | ISO 19082 frame cadence 50 ms 95th percentile | Phase 3 §4.3 |
| 10 | Federation token-exchange test passes | RFC 8693 |
| 11 | Calibration GEH statistic meets project threshold | §8 |
| 12 | Validation report archived with detached COSE signature | §8.2 |
| 13 | ISO/IEC 27001 ISMS scope statement covers the deployment | ISO/IEC 27001 §4.3 |
| 14 | IEC 62443-3-3 SR 1, SR 2, SR 5, SR 6 implemented | IEC 62443-3-3 |
| 15 | Privacy policy enforced on every trace stream | ISO/IEC 27701 |

The checklist results MUST be stored as a signed Phase-1 *SimulationProject* attribute and surfaced under `/projects/{id}/health`.

---

## 10. Operational Risk Considerations

### 10.1 Real-world binding risk

Bindings between an engine and a real-world traffic controller pose physical-world risk. Operators MUST:

- Use a privileged operator role for any state change that propagates to the controller.
- Maintain a fall-back path that re-asserts the controller's autonomous state within 1 s of engine disconnection.
- Restrict binding scope to bounded geographic and temporal windows.

### 10.2 Privacy re-identification

Vehicle-trace data, even when pseudonymised, is susceptible to re-identification under spatial / temporal correlation. Operators MUST:

- Apply the privacy-policy controls of Phase 1 §5.
- Aggregate to link-level summaries when releasing data outside the project boundary.
- Document any release as a record under ISO/IEC 27701 PIM.

### 10.3 Engine bias

Calibration-induced bias is a known risk. The validation report (§8.2) MUST surface bias metrics and the project SHALL document the operational consequences in the Phase-1 *SimulationProject* notes.

---

## 11. City-Scale Considerations

### 11.1 Multi-city federation

City-scale studies that span multiple jurisdictions use the Phase 3 §6 federation protocol. The federation document MUST list partner cities, partner project URIs, and the data-handling policies that survive federation.

### 11.2 Continuous-replay observatory

Observatory-mode deployments (continuous replay against live infrastructure for dashboarding purposes) MUST:

- Disable any engine action that propagates to the controller.
- Enforce a strict read-only TMC view.
- Limit retention of vehicle traces to the retention horizon configured in the privacy policy.

### 11.3 Climate and mode-shift studies

Engines used for climate or mode-shift studies MUST report estimated greenhouse-gas emissions and mode-shift effects with a clear methodology reference. Estimates MUST be presented as model outputs, not as authoritative emissions accounting; the latter requires alignment with ISO 14064 series (informative) and the operator's environmental-management system.

---

## 12. Conformance Tags

A project descriptor MUST advertise a list of conformance tags that summarise the achieved profile:

| Tag | Meaning |
|-----|---------|
| `wia-city-017/v1/baseline` | Phase 3 §9.1 |
| `wia-city-017/v1/real-time` | Phase 3 §9.2 |
| `wia-city-017/v1/federated` | Phase 3 §9.3 |
| `wia-city-017/v1/iso27001` | ISMS-covered |
| `wia-city-017/v1/iso27701` | PIM-covered |
| `wia-city-017/v1/iec62443-sl2` | Security level 2 |
| `wia-city-017/v1/iec62443-sl3` | Security level 3 |
| `wia-city-017/v1/hil` | HiL-bound to a real controller |
| `wia-city-017/v1/observatory` | Continuous-replay observatory |

Conformance tags are normative for automated discovery and informative for human readers.

---

## 13. References

1. IEC 60529:2013 — *IP Code.*
2. IEC 62443-2-1; IEC 62443-3-2; IEC 62443-3-3.
3. ISO 14064 (all parts) — *Greenhouse gases* (informative reference).
4. ISO 14817-1:2015; ISO 14817-2:2015; ISO 14817-3:2017 — *ITS data dictionaries.*
5. ISO 14825:2011 — *Geographic Data Files (GDF).*
6. ISO 17572 (all parts) — *Location referencing.*
7. ISO 19082:2025 — *Roadside-module / signal-controller data frames.*
8. ISO 19107:2003; ISO 19115-1:2014 — *Geographic information.*
9. ISO 21217:2020 — *CALM architecture.*
10. ISO/TS 19091:2017/2019 — *V2I / I2V signalised intersections.*
11. ISO 39001:2012 — *Road-traffic safety management* (informative).
12. ISO 6346 — *Freight containers — Coding, identification and marking* (informative).
13. ISO/IEC 18033-3:2010 — *Block ciphers.*
14. ISO/IEC 27001:2022; ISO/IEC 27002:2022; ISO/IEC 27701:2019.
15. RFC 5280 — *X.509 PKI Certificate and CRL Profile.*
16. RFC 5905; RFC 8915 — *NTPv4, NTS.*
17. RFC 7252; RFC 7641 — *CoAP, OBSERVE.*
18. RFC 7662 — *OAuth Token Introspection.*
19. RFC 8259; RFC 8610; RFC 8615; RFC 8949 — *JSON, CDDL, well-known URIs, CBOR.*
20. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
21. RFC 8613 — *OSCORE.*
22. RFC 8693 — *OAuth 2.0 Token Exchange.*
23. RFC 9019; RFC 9124 — *SUIT manifests.*
24. RFC 9052; RFC 9053 — *COSE.*
25. RFC 9110; RFC 9111; RFC 9457 — *HTTP semantics, caching, problem details.*
26. RFC 9700 — *OAuth 2.1.*
27. FIPS 180-4; FIPS 197; FIPS 198-1.
