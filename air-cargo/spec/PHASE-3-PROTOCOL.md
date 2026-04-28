# WIA-air-cargo PHASE 3 — PROTOCOL Specification

**Standard:** WIA-air-cargo
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
air-cargo operator: the IATA DGR + ICAO Annex 18 +
Doc 9284 dangerous-goods discipline; the IATA TACT
acceptance discipline; the IATA e-AWB issuance and
digital-signature discipline; the regulated-agent /
known-consignor security discipline (ICAO Annex 17 +
EU Reg 2015/1998 + TSA 49 CFR 1544/1546/1548/1549);
the WCO SAFE Framework + AEO discipline; the EU
ICS2 Pre-Loading Advance Cargo Information (PLACI)
discipline; the US ACAS pre-loading discipline; the
Cargo iQ shipment-progress discipline; the
specialised-cargo discipline (CEIV-Pharma / LAR /
PCR / TCR + IATA Lithium Battery Shipping
Guidelines); the post-flight reconciliation
discipline; and the supervisory cooperation
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022
- ICAO Annex 18 + Doc 9284 (TI) + Doc 9481
  emergency response
- ICAO Annex 17 (Security)
- IATA DGR + IATA TACT + IATA Cargo-IMP / Cargo-XML
- IATA Resolution 672 e-AWB + Multilateral Agreement
- IATA Resolution 833 / 600a / 600b
- IATA Cargo iQ Resolution 674 master operating
  plan v6
- IATA CEIV-Pharma + CEIV-Lithium + CEIV-Live-
  Animals + CEIV-Fresh
- IATA LAR + PCR + TCR + Lithium Battery Shipping
  Guidelines
- WCO SAFE Framework of Standards + WCO Data Model
  3.x
- US 19 CFR Part 122 + Part 4 + 19 CFR 122.49b
  ACAS
- US 49 CFR Parts 171-180 HMR
- US TSA 49 CFR 1544 + 1546 + 1548 + 1549
- US C-TPAT (Customs-Trade Partnership Against
  Terrorism) for AEO-equivalent in US
- EU Reg (EU) 2015/1998 + Reg (EU) 952/2013 UCC +
  ICS2 Reg (EU) 2024/1248 + AEO under UCC Art 38
- KR 관세법 + 항공보안법 + 종합인증우수업체 (AEO)
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)

---

## §1 IATA DGR + ICAO Annex 18 + Doc 9284 Discipline

The dangerous-goods discipline:

- The shipper's declaration follows IATA DGR §8
  shipper's-declaration-for-dangerous-goods format.
- The freight forwarder's acceptance check follows
  IATA DGR §9 acceptance-checklist procedure (the
  92-point checklist).
- Packing per the IATA DGR Packing Instruction (PI)
  applicable to the UN number.
- Marking and labelling per IATA DGR §7.
- Documentation — air waybill carries the dangerous-
  goods indication per IATA DGR §8.1.6.4 and the
  shipper's declaration accompanies the shipment.
- Loading per IATA DGR §9.3 — segregation rules,
  passenger-aircraft prohibitions, hidden-dangerous-
  goods awareness training.
- ICAO Doc 9481 emergency response procedures
  carried on the aircraft.

## §2 IATA TACT Acceptance Discipline

The TACT acceptance discipline:

- Conditions of carriage per IATA Resolution 600
  a / b — the air-waybill incorporates the
  conditions by reference.
- Rate application per IATA TACT Rate Manual.
- Acceptance criteria — packaging, marking,
  labelling, documentation completeness, security
  status.
- Refusal of carriage criteria — dangerous-goods
  prohibited on the routing, prohibited articles,
  embargoed destinations, missing documentation.

## §3 IATA e-AWB Discipline (Resolution 672)

The e-AWB discipline:

- The Multilateral e-AWB Agreement signatories
  (the freight forwarder, the airline, and the
  ground-handling agent) operate without a paper
  air waybill on enabled trade lanes.
- The e-AWB digital signature is verifiable per
  IATA Resolution 672 Annex.
- The e-AWB record's canonical digest is preserved
  for tamper detection.

## §4 Regulated-Agent / Known-Consignor Security
       Discipline

The security discipline aligns with the operating
jurisdiction's regime:

- ICAO Annex 17 baseline — every consignment loaded
  onto a passenger or all-cargo aircraft is subject
  to security screening.
- EU Reg (EU) 2015/1998 — regulated agent (RA),
  known consignor (KC), account consignor (AC),
  and the RA3 / KC3 third-country regime.
- US TSA — Indirect Air Carrier (IAC) under 49 CFR
  1548; Certified Cargo Screening Program (CCSP)
  under 49 CFR 1549; Air Carrier under 49 CFR
  1544 / 1546.
- KR 항공보안법 — 화물 보안 검색 + 화주 보안
  관리 + 알려진 화주 등록 + 알려진 운송사 등록.

The supply-chain security discipline maintains the
unbroken chain of custody from KC to RA to airline
cargo terminal to aircraft.

## §5 WCO SAFE Framework + AEO Discipline

The WCO SAFE Framework discipline:

- Pillar 1 — Customs-to-Customs information sharing.
- Pillar 2 — Customs-to-Business AEO partnership.
- Pillar 3 — Customs-to-Other-Government-Agencies
  cooperation.

The Authorised Economic Operator status (AEO under
EU UCC Art 38; C-TPAT in US; KR 종합인증우수업체)
is the operating jurisdiction's certification of
the operator's secure-and-compliant supply-chain
management. AEO benefits include reduced data-set
requirements, priority treatment, and mutual-
recognition with partner-country AEO programmes.

## §6 EU ICS2 PLACI Discipline

The EU ICS2 (Import Control System 2) Pre-Loading
Advance Cargo Information discipline:

- Pre-Loading data submission for express courier
  shipments (Release 1) and air-cargo (Release 2).
- Risk analysis at the EU border-customs level.
- High-risk shipments may receive a "do-not-load"
  decision before departure.
- Authorised Filer designation for shippers,
  forwarders, and carriers.

## §7 US ACAS Discipline

The US Air Cargo Advance Screening (ACAS) discipline
under 19 CFR 122.49b:

- Pre-loading data filing for shipments destined
  to or transiting the US.
- Risk-based targeting at the CBP National Targeting
  Center.
- The "do-not-load" message is delivered to the
  carrier within the screening timeframe.
- Acceptable Filer Identification — US-bound
  freight-forwarder or carrier identifier.

## §8 IATA Cargo iQ Discipline (Resolution 674)

The Cargo iQ master-operating-plan (MOP v6)
discipline:

- The shipment record key (SRK) ties together the
  multi-leg journey at the master AWB level.
- Per-leg milestone events publish actual times
  against the planned schedule.
- Performance measurement at the operator and
  airline levels feeds the IATA Cargo iQ
  membership performance benchmarking.

## §9 Specialised-Cargo Discipline

The specialised-cargo discipline:

- IATA CEIV Pharma — pharmaceutical cargo handling
  per IATA TCR + GDP; temperature-controlled chain
  attestation.
- IATA CEIV Lithium — lithium-battery shipping
  per the IATA Lithium Battery Shipping Guidelines.
- IATA CEIV Live Animals — live-animal shipping
  per the IATA Live Animals Regulations (LAR).
- IATA CEIV Fresh — perishable-cargo handling per
  IATA PCR.

## §10 Post-Flight Reconciliation Discipline

The post-flight reconciliation discipline:

- Manifest-to-load reconciliation — the loaded
  cargo against the manifest is verified at
  arrival.
- Discrepancy reporting (FNA notification of
  arrival, FBL booking-list adjustments,
  documentary discrepancies) feeds the operator's
  quality-management process.
- Damage / loss / pilferage reports trigger the
  operator's claim workflow under IATA TACT
  conditions of carriage.

## §11 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every
acceptance, screening, customs-filing, milestone,
DG declaration, and post-flight reconciliation
event.

## §12 Lithium-Battery Shipping Discipline

The IATA Lithium Battery Shipping Guidelines (LBSG)
+ IATA DGR Section II discipline:

- Battery type identification — UN3480 (lithium-
  ion shipped alone), UN3481 (lithium-ion contained
  in or packed with equipment), UN3090 (lithium-
  metal shipped alone), UN3091 (lithium-metal
  contained in or packed with equipment).
- State-of-charge limit — UN3480 lithium-ion at
  not more than 30% SoC for shipment by air.
- Section II shipper's exemptions for small
  cells/batteries, with weight and quantity
  thresholds and the Section II Lithium Battery
  Mark (CAO/PI requirements).
- Damaged / defective / recalled lithium battery
  prohibition for transport unless under specific
  approval.
- Operator variations (the airline's
  acceptance-policy specifically deviating from
  IATA DGR baseline) recorded against the routing.

## §13 Damage / Loss / Pilferage and Liability
        Discipline

The Montreal Convention 1999 (MC99) and Warsaw
Convention 1929 liability discipline:

- Carrier liability for cargo per MC99 Article 18 —
  17 SDR per kg unless special declaration of
  value made.
- Special declaration of value (SDV) carries
  additional charges and increases the liability
  ceiling.
- Notice-of-claim timeline — apparent damage at
  delivery; non-apparent damage within 14 days;
  delay within 21 days; loss declared within 21
  days of expected delivery.
- Documentation requirements — air-waybill
  remarks, irregularity reports, damage assessment
  by airline or its handler.

## §14 ULD Management Discipline

The Unit Load Device (ULD) management discipline:

- IATA ULD Care Code (ULDCC) compliance for ULD
  build-up, handling, and storage.
- IATA ULD Regulations (ULDR) document compliance
  for the operator handling ULDs across multiple
  airlines.
- ULD damage assessment and repair authorisation
  per the airline's published damage-limit policy.
- ULD inventory reconciliation and CASS-based
  cross-airline settlement under IATA Resolution
  830d.

## §15 Customs Bonded-Warehouse and Free-Trade-Zone
        Discipline

For operators with bonded-warehouse / FTZ
operations:

- US 19 CFR Part 19 (customs warehouses) + Part
  146 (foreign trade zones).
- EU UCC Articles 240-242 (customs warehousing)
  + 243-249 (free zones).
- KR 관세법 + 외국인투자촉진법 (free economic
  zones).
- Per-jurisdiction inventory-control software and
  bond-discharge procedures.

## §16 Documentary-Compliance Discipline

The documentary-compliance discipline:

- Air-waybill consistency check across master /
  house levels.
- Commercial-invoice + packing-list + certificate-
  of-origin alignment for customs filing.
- Specific-license / export-license verification
  for export-controlled commodities (US EAR / ITAR;
  EU Dual-Use Reg 2021/821; KR 대외무역법 + 전략
  물자관리원).
- Sanctions-screening integration with US OFAC SDN
  + EU Consolidated List + UN Security Council
  Sanctions Committee + KR 외교부 제재대상자 목록.

## §17 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
satisfy the IATA DGR + ICAO Annex 18 baseline,
exercise the regulated-agent / known-consignor
discipline per the operating jurisdiction's regime,
satisfy the WCO SAFE Framework + AEO discipline,
exercise the EU ICS2 / US ACAS pre-loading
discipline, and exercise the specialised-cargo
disciplines where the operator handles such cargo.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-air-cargo
- **Last Updated:** 2026-04-28
