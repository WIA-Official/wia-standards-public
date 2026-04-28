# WIA-distributed-energy PHASE 4 — INTEGRATION Specification

**Standard:** WIA-distributed-energy
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a distributed-energy
operator integrates with the systems that surround
the DER lifecycle: the local distribution utility
(DSO) and its distribution-management system; the
balancing authority and the wholesale Regional
Transmission Organisation / Independent System
Operator under FERC Order 2222; the utility's DERMS
(Distributed Energy Resources Management System);
the customer's energy-management system (EMS) and
home-energy-management system (HEMS); the OpenADR
VTN at the utility / curtailment-service provider;
the OCPP CSMS at the EV-charge-network operator; the
wholesale-market settlement infrastructure (the
operating jurisdiction's settlement engine, the
billing engine, the metering data-management agent);
the cybersecurity supervisory authority (NERC + CISA
in US, ENISA in EU, KISA / KrCERT-CC in KR); the
external auditor and the ISO/IEC 27001 + ISO/IEC
27019 certification body; and the long-term archive
that preserves operating records past the active
retention horizon.

References (CITATION-POLICY ALLOW only):

- IEEE 1547-2018 + IEEE 1547.1-2020 + IEEE 1547.9
- IEEE 2030.5-2018 + CSIP
- IEC 61850-7-420 + IEC 61968 + IEC 61970 CIM
- IEC 62351 + IEC 62933-1 + IEC 62933-5-2 + IEC
  62619
- UL 1741-SB + UL 9540 + UL 9540A
- SunSpec Modbus, OpenADR 2.0, OCPP 2.0.1, ISO
  15118-2 + 15118-20
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 27019:2024
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- ISO 8601
- W3C Verifiable Credentials Data Model 2.0
  (optional)
- NERC CIP-002 to CIP-014
- FERC Order 2222 + Order 2003-D
- US DOE OE GMI initiatives + DOE Cybersecurity
  Capability Maturity Model (C2M2)
- KR 신·재생에너지법 + KEPCO 분산전원 계통연계 +
  KR ETSC 전력시장운영규칙 + KEPIC

---

## §1 Distribution Utility (DSO) Integration

The operator's DSO integration covers:

- The interconnection-application portal under the
  utility's published Rule 21 (CA), MA SREC II,
  Australian AS/NZS 4777.2, KEPCO 분산전원 계통연계
  기준, or other jurisdictional interconnection-
  rule equivalent.
- The metering and telemetry feed (advanced-metering-
  infrastructure data) under the utility's data-
  exchange protocol (commonly IEEE 2030.5 + IEC
  61968 message envelopes).
- The protection-coordination dossier — the asset's
  protection settings recorded against the utility's
  feeder-protection scheme.
- The curtailment-and-restoration coordination —
  during distribution-feeder constraint events the
  utility issues curtailment via DERMS or via the
  IEEE 2030.5 DERControl resource.

## §2 DERMS Integration

The utility's DERMS is the integration boundary for
fleet-level operations:

- Asset registration and visibility — the DERMS
  carries the asset register and the controller
  binding (PHASE-1 §3 + §5).
- Forecasting — short-term DER-output forecasts
  feed the DSO's distribution-operating decisions.
- Dispatch — the DERMS issues dispatch instructions
  through IEEE 2030.5, OpenADR, or proprietary
  channels.
- Settlement — the DERMS reconciles dispatched
  output against actual telemetry for wholesale-
  market settlement.

## §3 Balancing-Authority and RTO / ISO Integration

For DER aggregators participating in FERC-
jurisdictional wholesale markets under FERC Order
2222:

- The aggregator's enrolment with the RTO / ISO
  (CAISO, MISO, ERCOT, PJM, NYISO, ISO-NE, SPP).
- The aggregator's compliance with the RTO / ISO's
  Tariff DER-aggregation provisions.
- The aggregator's bidding interface (RTO / ISO
  bid-and-offer-submission API).
- The aggregator's settlement reconciliation.

For KR-jurisdiction the equivalent KR ETSC 전력
시장운영규칙 + 전력거래소 (KPX) integration applies
under KR 전기사업법 + 신·재생에너지법 등록.

## §4 Customer EMS / HEMS Integration

For prosumer-class operators the customer's energy-
management system (EMS) for commercial / industrial
or home-energy-management system (HEMS) for
residential is the integration boundary for the
customer-facing experience:

- The EMS / HEMS receives the asset's telemetry
  (state-of-charge, output, ride-through events).
- The EMS / HEMS exposes the customer's control
  preferences (priority charging from solar,
  emergency-grid-export disable, etc.).
- The EMS / HEMS coordinates with the operator's
  DERMS for dispatch-and-settlement.

## §5 OpenADR VTN Integration

For demand-response programmes the operator's VEN
integrates with the curtailment-service provider's
or utility's VTN:

- Event notification per OpenADR 2.0b.
- Opt-in / opt-out response.
- Performance reporting via the OpenADR report
  profile.
- Settlement reconciliation under the programme's
  published baseline-and-performance methodology.

## §6 OCPP CSMS Integration

For EV-charge network operators:

- Each charging station maintains a WebSocket
  Secure connection to the CSMS.
- ISO 15118-2 / -20 plug-and-charge enrolment is
  recorded in the CSMS.
- Smart-charging profiles received via OCPP
  SetChargingProfile constrain station output.
- Roaming-network integration (e.g., Hubject /
  OCPI) extends customer-facing access across
  multi-CPO networks.

## §7 Cybersecurity Supervisory Integration

For US bulk-electric-system operators:

- NERC for CIP audits and self-reports.
- CISA for incident reporting under EO 14028 and
  CIRCIA.

For EU operators:

- ENISA for sector-specific cybersecurity guidance.
- NIS2 (Directive (EU) 2022/2555) for incident
  reporting where the operator is an essential or
  important entity in the energy sector.

For KR operators:

- KISA + KrCERT-CC for incident reporting under
  정보통신망법.
- KR FSC for OT-related supervisory oversight where
  applicable.

## §8 Settlement and Metering Data Management

The operator integrates with:

- The metering data-management agent (the utility's
  AMI head-end + MDMS) for verified metering data.
- The wholesale-market settlement engine for FERC
  Order 2222-aligned settlement.
- The retail-billing engine for customer-facing
  bill credits / charges.
- The renewable-energy-credit (REC) registry — the
  WREGIS / NEPOOL-GIS / M-RETS / K-RECS registry
  that issues per-MWh tradeable credits.

## §9 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the energy-sector ISO/IEC 27019:2024
extension applied. The certification body operates
under ISO/IEC 17021-1; the conformity-assessment body
for WIA-distributed-energy operates under ISO/IEC
17065. UL 1741-SB + UL 9540 + UL 9540A
certifications are held by the asset-supplier and
recorded in PHASE-1 §3.

## §10 Long-Term Archival Integration

Records governed by the operator's retention horizons
(NERC CIP-013-2 retention; FERC Order 2222 settlement-
record retention; KR 신·재생에너지법 + KEPCO
records-retention; ISO/IEC 27001 audit-log integrity)
are migrated to the long-term archive at the close
of the active retention window. The archive preserves
the asset register, the interconnection record, the
cybersecurity posture record, the event records (with
COMTRADE waveforms), and the audit-event trail.

## §11 V2G and Vehicle-to-Building Integration

For bidirectional EV interoperability:

- ISO 15118-20:2022 V2G messages enable bidirectional
  power flow.
- The vehicle-original-equipment-manufacturer's
  battery-warranty terms are reflected in the
  operator's V2G dispatch policy.
- The OEM's API for state-of-charge, state-of-
  health, and battery-management constraints feeds
  the operator's optimisation engine.

## §12 Climate-Disclosure Integration

For operators subject to ESG-disclosure regimes
the operator's GHG-inventory benefits from the
distributed-energy fleet's net contribution: the
operator's avoided-emissions calculation references
the local grid's carbon-intensity factor and the
asset's actual output, integrating with the entity's
WIA-esg-finance disclosure record (ISSB IFRS S2 +
ESRS E1) where the entity is a CSRD-or-ISSB-
reporting entity.

## §13 Wildfire-Risk and Public-Safety-Power-Shutoff
        Integration

For operators in wildfire-risk service territories
(California Investor-Owned Utilities, parts of
Australia, KR 산림청 wildfire-risk areas):

- Public-Safety Power Shutoff (PSPS) coordination —
  the operator integrates with the utility's PSPS
  programme so that islanded operation under PSPS
  preserves the customer's critical loads where the
  asset's storage is dimensioned for backup.
- Vehicle-and-electrification-load shedding under
  PSPS conditions per the operating utility's
  published policy.
- Black-start coordination — for utility-scale DER
  participating in distribution-grid black-start the
  operator maintains the grid-forming capability and
  participates in the utility's restoration drill.

## §14 Microgrid Integration

For operators participating in microgrids:

- The microgrid controller's grid-forming asset
  designation.
- The point-of-interconnection switch coordination
  with the utility for islanding-and-resynchronisation.
- The microgrid's per-IEEE-2030.7 (microgrid
  controller specification) and IEEE 2030.8 (test
  procedures) compliance.
- The community-microgrid governance arrangement
  where the microgrid serves multiple customers.

## §15 Conformance

Implementations claiming PHASE-4 conformance maintain
the DSO and DERMS integrations, exercise the FERC
Order 2222 (or jurisdictional equivalent)
participation where the operator participates in
wholesale markets, integrate with the OpenADR / OCPP
networks where DR / EV-charging are in scope, hold
the ISO/IEC 27001 + ISO/IEC 27019 certifications,
exercise the cybersecurity supervisory integration on
the operating jurisdiction's cadence, exercise the
PSPS and microgrid integration where the territory
or programme calls for it, and operate the long-term
archival integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-distributed-energy
- **Last Updated:** 2026-04-28
