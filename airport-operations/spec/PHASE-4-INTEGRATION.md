# WIA-airport-operations PHASE 4 — INTEGRATION Specification

**Standard:** WIA-airport-operations
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an airport operator
integrates with the systems that surround the
aerodrome lifecycle: the air-navigation-service-
provider (ANSP) and its tower / approach / area-
control facilities; the airline ground-handlers; the
airline operations-control centre; the national civil
aviation authority and the international ICAO USOAP
audit programme; the airport-collaborative-decision-
making (A-CDM) network; the airport-emergency-services
ecosystem (local fire, EMS, police, military); the
ICAO IBIS wildlife-strike database; the EU eccairs /
European Central Repository for occurrence reporting;
the FAA NextGen / SESAR ATM-modernisation programmes;
the customs / immigration / border-protection (CIQ)
agencies; the airport-tenant ecosystem (concessions,
fixed-base operators, cargo handlers, fuel providers);
the external auditor and ISO/IEC 27001 + ICAO Doc
9774 certification body; and the long-term archive
that preserves operational records past the active
retention horizon.

References (CITATION-POLICY ALLOW only):

- ICAO Annex 14 + Annex 17 + Annex 19 + Doc 9774 +
  Doc 9859 + Doc 9981 + Doc 4444 + Doc 9870 +
  Doc 9137
- ICAO USOAP Continuous Monitoring Approach
- ICAO Doc 9303 (cross-domain to passenger-
  processing)
- IATA AHM 803/810/911/913 + Resolution 753 + RP
  1750 + IGOM
- ACI APEX in Safety / APEX in Security
- EUROCAE ED-99 + ED-87 + ED-133
- ICAO FIXM + AIXM 5.1.1 + WXXM (weather)
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015, ISO/IEC
  17065:2012
- ISO 8601
- US FAA 14 CFR Part 139 + AC 150/5210 series + AC
  150/5300 (Airport Design)
- EU Regulation (EU) 139/2014 + Reg (EU) 376/2014
  + Reg (EU) 2015/1998 (security implementing rules)
- KR 항공안전법 + 공항시설법 + 인천국제공항공사법 +
  한국공항공사법 + KR 국토교통부 항공안전종합계획

---

## §1 ANSP Integration

The operator's ANSP integration:

- The tower / approach / area-control facility
  operates under ICAO Doc 4444 PANS-ATM with the
  airport's published procedures.
- A-SMGCS surveillance feeds (multilateration,
  ASDE-X, primary surveillance, ADS-B) inform the
  airport's surface-movement awareness.
- Bilateral data-exchange agreements between the
  airport and the ANSP under SWIM (FIXM + AIXM).

## §2 Airline Ground-Handler Integration

Per-airline-and-handler integration:

- Standard Ground Handling Agreement (IATA AHM 810)
  is the contractual baseline.
- Per-station Annexes A and B specify the services
  performed.
- IATA AHM service-specification clauses (911 /
  913 / 920) govern the per-service performance.
- IATA RP 1750 A-CDM milestone exchange.
- IATA Resolution 753 bag-tracking event publication.
- IGOM 9th edition operational procedures.

## §3 Airline Operations-Control-Centre (OCC)
       Integration

The airline OCC integration:

- Real-time A-CDM milestone updates from the
  airport feed the airline's network operations.
- Disruption coordination for diversions, delays,
  and irregular operations.
- Slot-coordination interface for slot-controlled
  airports per IATA WSG (Worldwide Slot Guidelines).

## §4 National Civil Aviation Authority and ICAO
       USOAP Integration

The supervisory authority integration:

- US FAA Office of Airports for 14 CFR Part 139
  certification + AC 150/5300 design conformance.
- EU EASA + Member-State NCA for Reg (EU) 139/2014
  certification.
- KR 국토교통부 항공정책실 for 항공안전법 certification.
- ICAO USOAP CMA (Continuous Monitoring Approach)
  audits with the operating jurisdiction's
  authority.

## §5 Airport-Emergency-Services Integration

The operator's mutual-aid agreements with:

- Local fire / EMS / police authorities for AEP
  activations.
- Military rescue assets where available
  (helicopter SAR, dive teams).
- Hospitals for mass-casualty incident reception.
- Hazmat units for dangerous-goods incidents.

The annual full-scale drill exercises the mutual-
aid integration.

## §6 ICAO IBIS Wildlife-Strike Database Integration

For wildlife-strike reporting the integration:

- Strike reports submitted to FAA Form 5200-7 (US-
  jurisdiction) feed the FAA Wildlife Strike
  Database.
- Sanitised reports submitted to ICAO IBIS via the
  national wildlife-strike coordinator.
- For EU-jurisdiction the strike reports are
  forwarded through the eccairs system to the
  European Central Repository.

## §7 EU eccairs / ECR Integration

For EU-jurisdiction operators:

- Mandatory occurrence reports (MOR) under Reg (EU)
  376/2014 forwarded to the national authority's
  eccairs entry-point.
- The Member-State authority forwards to the ECR.
- Sanitised ECR data feeds EASA's annual safety
  review and EU member-state safety-program
  preparation.

## §8 NextGen / SESAR ATM-Modernisation Integration

For US-jurisdiction the FAA NextGen integration
covers:

- ADS-B Out / ADS-B In equipage and surveillance
  consumption.
- Data Comm CPDLC integration where the airport
  is in tower-data-link environment.
- TFM (Traffic Flow Management) integration for
  Ground Delay Programs and Airspace Flow Programs.

For EU-jurisdiction the SESAR integration covers:

- 4D Trajectory-Based Operations (4D-TBO)
  participation.
- Initial 4D (i4D) integration with the FOM (Flight
  Object Model).
- Free Route Airspace coordination at the
  airspace boundary.

## §9 CIQ (Customs / Immigration / Quarantine)
       Integration

The operator's CIQ integration:

- Border-control authority integration (US CBP /
  EU FRONTEX coordination / KR 출입국·외국인청).
- API (Advance Passenger Information) and PNR
  (Passenger Name Record) data exchange per ICAO
  Doc 9944 + Doc 9303.
- Customs integration for cargo and passenger
  baggage.
- Public-health screening integration where the
  operating regime requires (e.g., during
  pandemic response under WHO IHR 2005).

## §10 Airport-Tenant Ecosystem Integration

The operator's tenant-management integration:

- Concession agreements with retail / F&B / lounge
  operators.
- Fixed-base operator (FBO) agreements at general-
  aviation aprons.
- Cargo handler agreements for the cargo apron.
- Fuel-provider agreements with hydrant or truck-
  fuelling operators.
- Vehicle-permit programmes at airside operations
  areas.

## §11 External Audit and Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the scope explicitly extending to
the operational, A-CDM, AHM messaging, SWIM, and
passenger-data endpoints. The certification body
operates under ISO/IEC 17021-1; the conformity-
assessment body for WIA-airport-operations operates
under ISO/IEC 17065. The aerodrome-certification
body (national CAA) operates under ICAO Doc 9774
guidance.

## §12 Long-Term Archival Integration

Records governed by the operator's retention horizons
(FAA 14 CFR 139.301 recordkeeping; EU Reg
376/2014 occurrence-record retention; KR 항공안전법
보존) are migrated to the long-term archive at the
close of the active retention window. The archive
preserves the movement-area record snapshots, the
A-CDM milestone history, the ground-handling records,
the de-icing records, the runway-incursion / wildlife-
strike / FOD records, the AEP activations, the SMS
hazard-and-occurrence record, and the audit-event
trail.

## §13 Climate-Disclosure Integration

For airport operators subject to ESG-disclosure
regimes the operator's GHG inventory captures Scope
1 (own vehicles / GPU emissions), Scope 2
(electricity for terminals / lighting), and Scope
3 (aircraft LTO emissions, ground-access emissions,
tenant emissions where consolidated). The operator's
WIA-esg-finance disclosure record (ISSB IFRS S2 +
ESRS E1) integrates the disclosure where the
operator is a CSRD or ISSB reporting entity.

## §14 Slot-Coordination and Schedule-Coordinator
        Integration

For Level 3 slot-coordinated airports per IATA WSG:

- The slot coordinator is independent of the
  airport operator and the airlines.
- The airport operator declares the airport's
  capacity (the coordination parameters) to the
  coordinator before each season.
- The coordinator allocates slots per IATA WSG
  rules.
- For EU-jurisdiction the EU Reg (EEC) 95/93
  (Slot Regulation) applies; the EC's slot reform
  proposal would extend the regime.

## §15 Aircraft Noise and Environmental Integration

The operator's environmental integration:

- Aircraft-noise modelling per ICAO Annex 16 Volume
  I + Doc 9911 (Aerodrome Noise Mitigation).
- Local-air-quality monitoring under ICAO Annex 16
  Volume II + Doc 9889 (Local Air Quality).
- Carbon-emissions reporting per ICAO CORSIA where
  the airport hosts CORSIA-eligible operators.
- Land-use planning coordination with the local
  zoning authority.

## §16 EV-Charging and Sustainable Aviation Fuel (SAF)
        Integration

For airports decarbonising ground-support equipment:

- EV-charging-station integration for ground-
  support-equipment fleets (cross-reference WIA-
  distributed-energy + WIA-energy-storage).
- SAF infrastructure integration — fuel-receiving,
  storage, and delivery systems for SAF (Sustainable
  Aviation Fuel) per ASTM D7566 specifications.
- ICAO CORSIA-eligible-fuel attestation tracking
  for the supplied fuel.
- Airport-side hydrogen-readiness planning for
  long-term decarbonisation pathways.

## §17 Cyber-Resilience and ICAO Annex 17 Cyber
        Integration

For cybersecurity governance:

- ICAO Annex 17 + Doc 8973 (Aviation Security
  Manual) cyber-security baseline.
- ICAO Cyber Resilience Strategy + Cybersecurity
  Action Plan integration.
- For EU-jurisdiction NIS2 (Directive (EU) 2022/2555)
  Annex I sector-criticality covers airports above
  the published threshold.
- For US-jurisdiction TSA Cybersecurity Directives
  for airport operators.

## §18 Conformance

Implementations claiming PHASE-4 conformance maintain
the ANSP, ground-handler, OCC, national-CAA, ICAO
USOAP, emergency-services, IBIS, eccairs, NextGen /
SESAR, CIQ, and tenant integrations, hold the
ISO/IEC 27001 certification + the aerodrome-
certification (Doc 9774-aligned), and operate the
long-term archival integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-airport-operations
- **Last Updated:** 2026-04-28
