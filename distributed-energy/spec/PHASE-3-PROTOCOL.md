# WIA-distributed-energy PHASE 3 — PROTOCOL Specification

**Standard:** WIA-distributed-energy
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
distributed-energy operator: the IEEE 1547-2018
interconnection-and-interoperability discipline
(Category I / II / III performance classes,
voltage / frequency ride-through, reactive-power
capability, voltage-regulation modes, anti-island
protection); the IEEE 1547.1-2020 conformance-test
discipline; the IEEE 2030.5 Common Smart Inverter
Profile discipline; the IEC 61850-7-420 DER object-
model discipline; the IEC 62351 cybersecurity
discipline; the NERC CIP-002~014 critical-
infrastructure discipline; the FERC Order 2222
DER-aggregation discipline; the demand-response
OpenADR discipline; the EV-supply OCPP / ISO 15118
discipline; the storage-specific UL 9540A thermal-
runaway discipline; and the supervisory and
balancing-authority cooperation discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 + 27019:2024 (information
  security in the energy industry)
- IEEE 1547-2018 + IEEE 1547.1-2020 + IEEE 1547.9
- IEEE 2030.5-2018 + Common Smart Inverter Profile
- IEC 61850-7-420:2021 + IEC 61850-90-7
- IEC 62351 series (Power systems management
  security)
- IEC 61968 + IEC 61970 CIM
- IEC 62933-1 + IEC 62933-5-2
- IEC 62619:2022
- UL 1741-SB + UL 9540 + UL 9540A
- SunSpec Modbus
- OpenADR 2.0a / 2.0b
- OCPP 2.0.1, ISO 15118-2:2014 + ISO 15118-20:2022
- NERC CIP-002 through CIP-014
- US FERC Order 2222 + FERC Order 2003-D
- NIST SP 800-82 Rev 3 (OT security)
- IEEE C37.111 (COMTRADE)
- KR 신·재생에너지법 + KEPCO 분산전원 계통연계 기준
  + KEPIC

---

## §1 IEEE 1547-2018 Interconnection Discipline

The IEEE 1547-2018 interconnection discipline:

- Category selection — Category I (default for the
  smallest DER, most lenient ride-through), Category
  II (intermediate, the typical large-rooftop /
  small-commercial), Category III (most stringent,
  utility-scale and storage-heavy installations).
- Voltage ride-through — the asset rides through
  voltage excursions per the category-specific
  envelope; outside the envelope the asset trips per
  the published trip thresholds.
- Frequency ride-through — same pattern with
  frequency excursions.
- Reactive-power capability — Category A or B
  reactive-power capability per IEEE 1547-2018
  §5.2.2 (Category B is the default for most
  installations).
- Voltage-regulation mode — constant power factor,
  voltage-reactive (volt-var), watt-power-factor
  (watt-pf), or watt-var (watt-var) per the
  utility's published activation policy.
- Anti-island protection — the inverter detects
  islanding and disconnects per the IEEE 1547-2018
  §8 timing requirements.

The interconnection settings are recorded in PHASE-1
§4 with the protection-setting reference; field
verification follows IEEE 1547.1-2020 commissioning
test.

## §2 IEEE 1547.1-2020 Conformance-Test Discipline

The conformance-test discipline:

- Type test — performed by the manufacturer at a
  Nationally Recognised Testing Laboratory (NRTL)
  per UL 1741-SB before product certification;
  IEEE 1547.1-2020 specifies the procedures.
- Production test — per-unit factory test against
  the type-test envelope.
- Commissioning test — at the field installation to
  verify that the asset's interconnection settings
  match the utility's approved protection setting.
- Periodic test — exercised on the operator's
  documented cadence to verify continued conformance.

## §3 IEEE 2030.5 + Common Smart Inverter Profile
       Discipline

The IEEE 2030.5 + CSIP discipline operationalises
utility-DER communication:

- The DER asset's IEEE 2030.5 client registers with
  the utility's IEEE 2030.5 server.
- The asset publishes its DERCapability,
  DERAvailability, DERSettings, and DERStatus on the
  CSIP-defined cadence.
- The utility issues DERControl objects (active-
  power dispatch, reactive-power dispatch, ride-
  through curve update) which the asset accepts or
  rejects per its policy.
- For California Rule 21 + similar regulations the
  asset's CSIP-conformance certificate is required
  for interconnection approval.

## §4 IEC 61850-7-420 DER Object-Model Discipline

For utility DERMS implementations using IEC 61850
the operator's discipline encodes:

- The IEC 61850-7-420:2021 logical-node hierarchy
  for DER (DPVM, DSTO, DCST, DGEN, DREN, DRCT, etc.).
- IEC 61850-90-7 inverter object models for solar
  PV and storage.
- The IEC 61850 SCD (Substation Configuration
  Description) document covering the operator's
  DER fleet at the substation level.

## §5 IEC 62351 Cybersecurity Discipline

The IEC 62351 cybersecurity profiles applied:

- IEC 62351-3 — TLS profile for IEC 60870-5-104,
  IEC 61850 application protocols, DNP3.
- IEC 62351-4 — secure profiles for MMS (the
  application protocol used by IEC 61850-8-1).
- IEC 62351-5 — security for IEC 60870-5 and
  derivatives (DNP3 Secure Authentication v6).
- IEC 62351-6 — security for IEC 61850 GOOSE and
  Sampled Values.
- IEC 62351-7 — network-and-system management.
- IEC 62351-8 — role-based access control.
- IEC 62351-9 — key management.

## §6 NERC CIP Discipline

For US bulk-electric-system DER operators with
NERC-CIP-applicable assets:

- CIP-002 — BES cyber-asset categorisation (Low /
  Medium / High impact).
- CIP-003 — security-management controls.
- CIP-004 — personnel and training.
- CIP-005 — electronic security perimeters.
- CIP-006 — physical security of BES cyber systems.
- CIP-007 — system-security management.
- CIP-008 — incident-reporting and response
  planning.
- CIP-009 — recovery plans.
- CIP-010 — configuration-change management and
  vulnerability assessments.
- CIP-011 — information-protection.
- CIP-013 — supply-chain risk management.
- CIP-014 — physical security (transmission stations
  and substations).

The operator's compliance is exercised through NERC
audits and self-reports.

## §7 FERC Order 2222 DER-Aggregation Discipline

For DER aggregators participating in FERC-
jurisdictional wholesale markets under Order 2222:

- The aggregator's enrolment with the relevant
  Regional Transmission Organisation (RTO) /
  Independent System Operator (ISO).
- The aggregator's distribution-utility coordination
  agreement.
- The aggregator's metering, telemetry, and
  settlement obligations per the RTO / ISO tariff.
- The double-counting prevention discipline that
  ensures the same DER is not enrolled in conflicting
  retail and wholesale programmes.

## §8 Demand-Response OpenADR Discipline

The OpenADR demand-response discipline:

- The Virtual Top Node (VTN) at the utility / ISO
  publishes events to the Virtual End Node (VEN) at
  the operator.
- The VEN responds with an opt-in / opt-out per the
  customer's enrolment.
- The VEN reports back-haul telemetry to the VTN per
  the OpenADR reporting profile.
- Settlement follows the operating tariff's published
  baseline-and-performance methodology.

## §9 EV-Supply OCPP / ISO 15118 Discipline

For EV-supply equipment operators:

- OCPP 2.0.1 connects the charging station to the
  Central System (CSMS) for transactions, status,
  authorisation, smart-charging, and metering.
- ISO 15118-2:2014 plug-and-charge enables the
  vehicle's automatic authorisation.
- ISO 15118-20:2022 extends to bidirectional (V2G)
  power transfer.
- Smart-charging profiles received via OCPP 2.0.1
  SetChargingProfile constrain the station's output
  per the utility's distribution-feeder constraints.

## §10 Storage-Specific Discipline (UL 9540A + IEC
       62619)

For energy-storage assets the discipline:

- UL 9540A thermal-runaway test — cell-, module-,
  unit-, and installation-level tests demonstrate
  the storage's response to thermal abuse and the
  operator's siting-and-fire-protection design
  references.
- IEC 62619:2022 industrial Li-ion safety
  requirements.
- IEEE 1547.9 storage-interconnection recommended
  practice for grid-connected storage.
- The operator's emergency-response plan covering
  thermal-runaway scenarios with the local fire
  authority.

## §11 Identity, Time and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline; IEEE 1588 PTP is exercised for sub-
millisecond precision. Audit-events are emitted for
every interconnection change, dispatch instruction,
ride-through event, demand-response event, EV-charge
transaction, and cybersecurity-posture update.

## §12 Forecasting and Dispatch-Optimisation Discipline

The operator's forecasting discipline:

- Solar-irradiance forecast — short-term (intraday)
  and day-ahead irradiance forecasts feed the solar-
  PV output forecast.
- Load forecast — building-level / circuit-level
  load forecast feeds net-export prediction.
- State-of-charge forecast — for storage assets the
  forecast horizon spans the operator's
  optimisation window (typically 24-48 hours).
- Dispatch optimisation — the operator's dispatch
  engine optimises the asset fleet against the
  utility's published price signal and the wholesale-
  market price-and-capacity opportunities, subject
  to interconnection constraints.

The forecasts feed the IEEE 2030.5 DERAvailability
publication and the RTO / ISO bid-and-offer surfaces.

## §13 Cyber-Physical Defence-in-Depth Discipline

Beyond IEC 62351 + NERC CIP the operator's defence-
in-depth discipline applies:

- Network segmentation — IT / OT segmentation per
  IEC 62443 (Industrial Automation and Control
  Systems Security) zone-and-conduit model.
- Firmware integrity — secure-boot and signed-
  firmware-update policies enforced at the asset
  level.
- Supply-chain assurance — NERC CIP-013 supply-
  chain risk-management combined with the asset's
  SBOM (CycloneDX) declaration.
- Physical security — site-level access control for
  utility-scale assets, sealed-cabinet-and-tamper-
  detection for behind-the-meter assets.

## §14 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
maintain the IEEE 1547-2018 + IEEE 1547.1-2020 +
IEEE 2030.5 / CSIP compliance posture for the
operator's asset mix, satisfy the IEC 62351 + NERC
CIP cybersecurity discipline, exercise the demand-
response and EV-charging coordination on the
operator's published cadence, and exercise the
storage-specific safety discipline where storage is
in scope.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-distributed-energy
- **Last Updated:** 2026-04-28
