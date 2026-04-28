# WIA-air-cargo PHASE 4 — INTEGRATION Specification

**Standard:** WIA-air-cargo
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an air-cargo operator
integrates with the systems that surround the air-
cargo lifecycle: the airline cargo division and its
yield-management and capacity systems; the freight
forwarder forwarding-management system; the
regulated-agent / known-consignor security regime;
the customs / border-protection authority's
electronic-filing system; the WCO SAFE Framework
mutual-recognition partner network; the airport
cargo terminal's terminal-operating-system (TOS); the
airline interline-cargo settlement engine; the
international-postal operator (where airmail is in
scope); the IATA ONE Record initiative for next-
generation cargo data sharing; the cargo-insurance
underwriter; the supervisory authority for the
operating jurisdiction; the external auditor and
ISO/IEC 27001 + IATA CEIV certification body; and
the long-term archive that preserves cargo records
past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- ICAO Annex 18 + Annex 17 + Doc 9284 + Doc 9481
- IATA DGR + IATA TACT + IATA Cargo-IMP / Cargo-XML
  + IATA ONE Record
- IATA Resolution 672 + Resolution 674 Cargo iQ
- IATA CEIV Pharma + Lithium + Live Animals + Fresh
- WCO SAFE Framework + WCO Data Model 3.x + WCO
  Mutual Recognition
- US 19 CFR Part 122 + 49 CFR Parts 171-180 + TSA
  49 CFR 1544/1546/1548/1549 + 19 CFR 122.49b ACAS
  + C-TPAT
- EU Reg (EU) 952/2013 UCC + Reg (EU) 2015/1998 +
  ICS2 Reg (EU) 2024/1248 + AEO under UCC Art 38
- KR 관세법 + 항공보안법 + 종합인증우수업체 (AEO-K)
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015, ISO/IEC
  17065:2012
- ISO 8601
- W3C Verifiable Credentials Data Model 2.0
- UN/EDIFACT IFTSTA / IFTMIN / IFTMCS

---

## §1 Airline Cargo Division Integration

The operator's airline integration:

- Capacity and allotment management — IATA Cargo-
  IMP FRA / FFA messages or Cargo-XML allotment-
  request / allotment-confirmation.
- Yield-management feed — booked capacity, accepted
  bookings, and revenue at the booking class level.
- Aircraft-load-planning — IATA AHM 350 / 355
  loadsheet integration; cargo loadsheet by ULD
  position.
- Inflight-cargo handling — temperature-controlled
  containers, live-animal cabin care, dangerous-
  goods loading restrictions.

## §2 Freight Forwarder Integration

The forwarding-management system integration:

- Booking submission via Cargo-XML or Cargo-IMP.
- House AWB issuance under the consolidated master
  AWB.
- e-AWB Multilateral Agreement participation.
- Cargo iQ shipment-record-key (SRK) propagation
  for end-to-end visibility.

## §3 Regulated-Agent / Known-Consignor Integration

The supply-chain-security integration:

- Bilateral contractual arrangement establishing
  the chain of custody.
- Per-shipment Consignment Security Declaration
  (CSD) — paper or e-CSD via IATA Cargo-XML.
- Drift-and-anomaly monitoring at the regulated
  agent's screening operation.
- Periodic regulator-led inspections of the RA / KC
  facility.

## §4 Customs / Border-Protection Integration

The customs integration:

- US CBP — Air Manifest Air AMS, ACAS pre-loading,
  Automated Commercial Environment (ACE) e-filing.
- EU Member-State customs — ICS2 entry summary
  declaration (ENS), import declaration (UCC),
  transit declaration (NCTS).
- KR 관세청 — UNI-PASS 통합 신고 시스템.
- WCO Data Model 3.x harmonised data set.

## §5 WCO SAFE Mutual Recognition Integration

For AEO operators participating in WCO Mutual
Recognition Arrangements (MRAs):

- AEO certificate cross-references in customs
  declarations enable the partner-country
  facilitation benefits.
- Per-MRA-pair benefits — reduced inspection rates,
  priority release, faster transit times.
- The operator's MRA-eligible data attributes
  are forwarded to the partner customs.

## §6 Airport Cargo-Terminal Integration

The cargo-terminal-operating-system integration:

- Booking capacity reservation at the cargo
  terminal.
- Freight On Hand (FOH) acknowledgement at receipt.
- ULD build-up and break-down operations.
- Inter-terminal transfer where multiple cargo
  terminals serve the airport.
- Coordination with airport-operations (cross-
  reference WIA-airport-operations) for ramp
  movements.

## §7 IATA ONE Record Integration

For operators participating in the IATA ONE Record
initiative:

- ONE Record API + data model (JSON-LD over
  HTTPS) replaces the EDI-style messaging with a
  resource-oriented data substrate.
- Linked-data references between the master AWB,
  house AWB, shipment events, and party records.
- Distribution to authorised consumers via OAuth
  2.1 fine-grained scopes.

## §8 Airline Interline-Cargo Settlement Integration

The interline-cargo settlement integration:

- IATA CASS (Cargo Account Settlement Systems) for
  airline-to-forwarder accounts settlement.
- IATA SIS (Simplified Invoicing and Settlement)
  for airline-to-airline interline settlement.
- The operator's settlement records reconcile
  against the CASS / SIS clearing data.

## §9 International Postal Operator Integration

For airmail handled by the operator:

- Universal Postal Union (UPU) UCC EDI messages
  (PREDES, RESDES, CARDIT, RESDIT) for postal-
  cargo manifests.
- Postal terminal handling per UPU requirements.
- Postal customs treatment per the operating
  jurisdiction's postal-customs regime.

## §10 Cargo-Insurance Underwriter Integration

The cargo-insurance integration:

- Shipment-level insurance declaration accompanying
  the e-AWB.
- Claim filing for damage / loss / theft per the
  Montreal Convention 1999 / Warsaw Convention
  1929 limits.
- Underwriter risk-engineering on specialised-
  cargo handling (CEIV Pharma compliance, lithium-
  battery handling).

## §11 External Audit and Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the scope explicitly extending to
the cargo-messaging, customs-filing, and security-
screening endpoints. The certification body operates
under ISO/IEC 17021-1; the conformity-assessment
body for WIA-air-cargo operates under ISO/IEC
17065. IATA CEIV Pharma / Lithium / Live Animals /
Fresh certifications are held where the operator
handles the relevant specialised cargo.

## §12 Long-Term Archival Integration

Records governed by the operator's retention horizons
(US 19 CFR Part 122 retention; EU UCC five-year
retention; KR 관세법 7-year retention; IATA TACT
three-year retention; IATA DGR shipment-record
retention) are migrated to the long-term archive at
the close of the active retention window. The archive
preserves the e-AWB record, the dangerous-goods
declarations, the security-screening records, the
customs declarations, the Cargo iQ milestone history,
and the audit-event trail.

## §13 Specialised-Carrier Integration

For specialised carriers:

- Express integrators (FedEx, UPS, DHL, EMS) —
  integration through the integrator's proprietary
  API plus ICAO / IATA / WCO baseline message
  formats.
- All-cargo airlines — direct integration via
  IATA Cargo-IMP / Cargo-XML.
- Combination carriers (passenger aircraft cargo
  capacity) — booking through belly-cargo allotments.

## §14 Sustainable Aviation and Carbon Reporting
        Integration

For air-cargo decarbonisation reporting:

- ICAO CORSIA participation — eligible-fuel
  attestation for the operator's CO2 inventory.
- Per-shipment CO2 emissions disclosure for
  sustainability-conscious shippers — per-leg
  great-circle distance + payload-share + aircraft-
  type-specific emission factor (Smart Freight
  Centre Global Logistics Emissions Council
  Framework).
- Cargo-IQ + ONE Record extensions for carbon-
  intensity attribution.
- Operator's WIA-esg-finance disclosure record
  integrates the air-cargo Scope 1 + Scope 3
  attribution.

## §15 Counterfeit-Goods and Wildlife-Trafficking
        Cooperation Integration

For supply-chain integrity:

- WCO + Interpol cooperation on counterfeit goods
  intercepted in the cargo channel.
- CITES (Convention on International Trade in
  Endangered Species of Wild Fauna and Flora)
  permit verification for trade in CITES-listed
  species.
- IATA Live Animals Regulations LAR Chapter 2
  CITES compliance.
- US Lacey Act + EU Wildlife Trade Regulation
  338/97 enforcement support.

## §16 Cybersecurity and Cargo-Data Protection
        Integration

For cargo-data integrity and confidentiality:

- ICAO Annex 17 + Doc 8973 cyber-security baseline
  applied to cargo information systems.
- For EU-jurisdiction NIS2 (Directive (EU) 2022/2555)
  Annex I sector-criticality covers air-transport
  operators.
- TSA Cybersecurity Directives for US air-cargo
  operators.
- KR 정보통신망법 + KISA + KrCERT-CC for KR-
  jurisdiction operators.
- Cargo-data classification — booking, AWB, DG
  declarations, security-screening, customs
  declarations are sensitive supply-chain
  information requiring confidentiality controls.

## §17 Express Carrier and E-Commerce Cross-Border
        Integration

For express-carrier-and-e-commerce flows:

- Express integrators (FedEx / UPS / DHL / USPS /
  EMS / Korea Post / 한진 / CJ Logistics) integrate
  via proprietary API + IATA Cargo-XML baseline.
- US Section 321 de minimis threshold ($800)
  filings via ACE Type 86.
- EU Import One-Stop Shop (IOSS) for B2C imports.
- KR 해외직구 통관시스템 + 목록통관 channels.
- E-commerce-platform-to-postal handovers under
  UPU EMS Cooperative arrangement.

## §18 Pharma Cold-Chain Integration

For pharmaceutical cargo:

- IATA CEIV Pharma certified facilities along the
  routing.
- Temperature-controlled containers (active or
  passive) per IATA TCR specifications.
- WHO Good Distribution Practice for Medical
  Products (GDP) compliance.
- US FDA + EU EMA + KR MFDS supply-chain
  attestation.
- Continuous temperature monitoring with deviation
  alerts to the shipper.

## §19 Conformance

Implementations claiming PHASE-4 conformance maintain
the airline, forwarder, RA / KC, customs, terminal,
and IATA ONE Record (where adopted) integrations,
exercise the WCO SAFE + AEO mutual-recognition where
the operator is AEO-certified, hold the ISO/IEC
27001 certification + IATA CEIV certifications where
applicable, and operate the long-term archival
integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-air-cargo
- **Last Updated:** 2026-04-28
