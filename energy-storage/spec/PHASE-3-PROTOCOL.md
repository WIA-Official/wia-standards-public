# WIA-energy-storage PHASE 3 — PROTOCOL Specification

**Standard:** WIA-energy-storage
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
energy-storage operator: the IEC 62933 series storage-
specific safety and performance discipline; the
UL 9540 + 9540A thermal-runaway test discipline; the
NFPA 855 site-permit + emergency-response discipline;
the IEC 62619 + UL 1973 industrial-Li-ion + UN 38.3
transport discipline; the IEEE 1547.9 + IEEE 1679
grid-interconnection discipline; the IEEE 2030.2.1
operations-and-maintenance discipline; the BMS
configuration-and-firmware discipline; the cycle-
life-and-degradation tracking discipline; the
warranty-claim discipline; the incident-response
and emergency-services cooperation discipline; the
second-life and recycling end-of-life discipline;
and the supervisory and AHJ correspondence
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 + 27019:2024
- IEC 62933-1:2018, 62933-2-1:2017, 62933-2-2:2024,
  62933-5-2:2020
- IEC 62619:2022, IEC 62620:2014, IEC 60086 series,
  IEC 61960-3:2017
- IEEE 1547-2018 + 1547.9-2022 + 1679-2020 +
  2030.2.1-2019 (cross-reference to WIA-distributed-
  energy)
- UL 9540 + UL 9540A + UL 1973
- NFPA 855 (Standard for the Installation of
  Stationary Energy Storage Systems)
- NFPA 68 (Standard on Explosion Protection by
  Deflagration Venting)
- NFPA 69 (Standard on Explosion Prevention Systems)
- NFPA 70 NEC Article 706 (Energy Storage Systems)
- UN ST/SG/AC.10/11/Rev.7 — UN 38.3 lithium-battery
  transport tests
- UN ECE Regulation R100.03
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- US OSHA 29 CFR Part 1910 + KR 산업안전보건법
  occupational-safety
- KR KEPIC + KR 전기사업법 + KR 한국전기안전공사
  + KR 소방청 + KR 신·재생에너지법
- US FERC Order 841 (storage market participation)
- EU Battery Regulation (Reg (EU) 2023/1542)

---

## §1 IEC 62933 Series Discipline

The IEC 62933 storage-specific discipline:

- IEC 62933-1 — terminology baseline used across all
  storage records.
- IEC 62933-2-1 — performance-test methodology
  applied at unit-acceptance test.
- IEC 62933-2-2 — application-and-performance
  testing at the integrated installation level.
- IEC 62933-5-2 — safety requirements for grid-
  integrated electrochemical EES systems —
  electrical-protection, thermal-protection,
  mechanical-protection, and the reliability-and-
  safety analysis.

## §2 UL 9540 / 9540A Discipline

The UL 9540 / 9540A discipline:

- UL 9540 certification of the integrated ESS
  product (battery + power-conversion + protection).
- UL 9540A test method exercised at the cell level,
  the module level, the unit level, and (where
  installation-design warrants) the installation
  level. Test reports inform the operator's NFPA
  855-aligned site permit.
- The cell-level UL 9540A report identifies the
  thermal-runaway-onset temperature, the off-gas
  composition, and the propagation behaviour to
  adjacent cells.

## §3 NFPA 855 Site-Permit and Emergency-Response
       Discipline

The NFPA 855 discipline at the installation site:

- Maximum stored energy per fire-area limits (varies
  by chemistry and enclosure kind).
- Separation distances between adjacent storage
  units and between storage and exposures (occupied
  buildings, lot lines, public ways).
- Required fire-suppression system per the chemistry
  and enclosure kind.
- Required ventilation and explosion-control per
  NFPA 68 / 69 where applicable.
- Pre-incident plan delivered to the local fire
  department covering the chemistry, the storage
  topology, the suppression-system characteristics,
  and the recommended fire-suppression tactics.
- Annual emergency-response drill with the local
  fire department.

## §4 IEC 62619 + UL 1973 + UN 38.3 Discipline

For lithium-ion industrial cells and batteries:

- IEC 62619:2022 — safety requirements for
  industrial Li-ion cells and batteries (over-
  charge, internal short, external short, mechanical
  abuse, thermal abuse, vibration, shock, drop).
- UL 1973 — battery cell / pack safety for
  stationary auxiliary power.
- UN 38.3 — transport-test certification including
  altitude simulation, thermal test, vibration,
  shock, external short circuit, impact, overcharge,
  forced discharge.

## §5 IEEE 1547.9 + 1679 + 2030.2.1 Discipline

The IEEE-grid-interconnection-and-operations
discipline:

- IEEE 1547.9-2022 — recommended practice for
  storage interconnection. Covers the storage-
  specific aspects of IEEE 1547-2018 — bidirectional
  operation, charge / discharge ride-through,
  protection coordination, and reactive-power
  capability with energy-management.
- IEEE 1679-2020 — characterization and evaluation
  of energy-storage technologies (round-trip
  efficiency, capacity-fade rate, calendar life,
  temperature dependence).
- IEEE 2030.2.1-2019 — operations and maintenance
  guide for stationary BESS.

## §6 BMS Configuration and Firmware Discipline

The BMS discipline:

- BMS firmware is signed by the manufacturer's
  signing key; the BMS verifies the signature at
  boot per the secure-boot policy.
- Firmware updates are exercised under the
  operator's change-management discipline; updates
  are audit-logged with the operator's approval
  trail.
- BMS configuration changes (protection-threshold
  updates, balancing-method changes) require the
  operator's risk-and-safety committee approval.
- BMS-detected anomalies (cell-imbalance, voltage
  drift, internal-resistance increase) trigger the
  operator's preventive-maintenance workflow.

## §7 Cycle-Life and Degradation Discipline

The degradation discipline:

- Periodic capacity-test cycles (per IEEE 1679-2020
  reference method) calibrate the operator's
  state-of-health estimate.
- Degradation models — empirical (Arrhenius +
  capacity-fade-as-power-of-cycles), semi-empirical
  (single-particle model with Butler-Volmer
  kinetics), or physics-based (P2D pseudo-2D model)
  — drive the operator's end-of-life forecast.
- Degradation-indicator drift (capacity-fade,
  internal-resistance increase, self-discharge
  increase) triggers the operator's review and
  potential warranty claim.

## §8 Warranty-Claim Discipline

The warranty-claim discipline:

- Manufacturer warranty terms (typically 10 years
  / X full-equivalent-cycles / Y% capacity-retention
  warranted) are recorded in the asset register.
- Performance-monitoring identifies departures from
  the warranted trajectory; the operator's claim
  is filed with the manufacturer with the supporting
  cycle-life and state-record evidence.
- Claim resolution (replacement, refurbishment,
  buy-out) is recorded against the asset.

## §9 Incident-Response and Emergency-Services
       Cooperation Discipline

The incident-response discipline:

- Detection — voltage-anomaly, temperature-anomaly,
  off-gas detection, smoke detection, fire-
  suppression trigger drives the BMS / facility-
  alarm into the operator's incident channel.
- Containment — the BMS isolates the affected
  module; cooling / ventilation is increased; the
  operator's site-staff secures the perimeter.
- Notification — the local fire department / AHJ /
  KR 한국전기안전공사 / 소방청 is notified per the
  jurisdictional notification timeline.
- Suppression — fire-suppression is activated per
  the pre-incident plan.
- Post-incident — the NFPA 855 + UL 9540A-aligned
  post-mortem narrative is prepared; lessons
  feedback into the operator's NFPA 855 site
  permit, the BMS configuration, and the operator's
  emergency-response plan.

## §10 Second-Life and Recycling Discipline

The end-of-life discipline:

- Second-life repurposing — the storage's residual
  capacity (typically 70-80% of beginning-of-life
  nominal) supports stationary applications with
  lower performance demands.
- Recycling — the operator engages a licensed
  recycler under the operating jurisdiction's
  battery-recycling regime (EU Battery Regulation
  (EU) 2023/1542 producer-responsibility scheme;
  US state-by-state recycling rules; KR 전기·
  전자제품 및 자동차의 자원순환에 관한 법률).
- Hazardous-waste classification — Li-ion at end-
  of-life is classified as hazardous waste (US RCRA;
  KR 폐기물관리법) until processed.

## §11 Identity, Time and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every BMS
firmware update, configuration change, market bid,
incident, AHJ notification, post-mortem update,
warranty claim, and end-of-life decision.

## §12 Off-Gas-Detection and Deflagration-Prevention
        Discipline

The off-gas discipline:

- Cell-vent off-gas detection via H2 / CO / CO2 /
  HF / electrolyte-vapour sensors at the module and
  unit levels (operator-published thresholds inform
  the BMS isolation logic).
- Deflagration prevention per NFPA 68 (explosion
  protection by deflagration venting) and NFPA 69
  (explosion-prevention systems) — engineered vent
  panels, inert-gas blanketing, or oxygen-depletion
  for high-density installations.
- Off-gas detection is the earliest reliable
  indicator of thermal runaway initiation; the
  operator's response automation (BMS isolation +
  ventilation increase + AHJ notification) fires
  before fire develops.

## §13 Calendar-Aging and Path-Dependent Degradation
        Discipline

The degradation modelling discipline:

- Calendar aging — Arrhenius-temperature-dependent
  capacity-fade models projected from accelerated
  shelf-life tests.
- Cycle aging — depth-of-discharge-dependent and
  C-rate-dependent capacity-fade models.
- Path-dependent effects — the degradation rate
  depends on the operating SoC distribution; high
  SoC dwell-time accelerates aging.
- Lithium-plating prevention — fast-charge profiles
  derate at low temperature to avoid lithium
  plating, the dominant safety-critical aging mode
  for graphite-anode Li-ion.
- The operator's BMS adjusts charge-current and SoC
  bounds based on the calendar-and-cycle aging
  trajectory.

## §14 Cybersecurity-and-Tamper Discipline

The cybersecurity discipline for storage:

- BMS firmware secure-boot per the manufacturer's
  signing-key chain; tamper detection feeds the
  operator's incident channel.
- IEC 62351 cryptographic profile applied to BMS-
  to-cloud and BMS-to-DERMS communication.
- Physical-tamper detection at the cabinet level
  (door sensors, accelerometers); unauthorised-
  access events are audit-logged and escalated.
- Supply-chain integrity — the operator's
  procurement contract requires the manufacturer
  to provide CycloneDX or SPDX SBOM declarations
  for the BMS firmware.

## §15 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
satisfy the IEC 62933 + UL 9540 + 9540A + NFPA 855
+ UN 38.3 safety baseline, exercise the IEEE 1547.9
storage-interconnection discipline, and exercise the
incident-response discipline integrated with the
local AHJ / fire department / KR 한국전기안전공사 /
소방청.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-energy-storage
- **Last Updated:** 2026-04-28
