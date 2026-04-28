# WIA-energy-cloud PHASE 4 — INTEGRATION Specification

**Standard:** WIA-energy-cloud
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an energy-cloud operator
integrates with the systems that surround the
utility-cloud lifecycle: the substation-automation
substrate (IEC 61850 GOOSE / Sampled Values / MMS
networks); the AMI head-end and meter data-management
agent (IEC 62056 / ANSI C12); the wholesale-market
RTO / ISO / KPX; the FERC and Member-State NCA
supervisory authorities; the DSO and TSO neighbouring
utilities; the third-party DERMS / DR aggregator
platform; the customer-bill engine and CIS;
the building-energy-management ecosystem (Project
Haystack + ASHRAE BACnet + ISO 16484); the
cybersecurity supervisor (NERC + CISA + ENISA + KISA);
the external auditor and ISO/IEC 27001 + 27019
certification body; and the long-term archive that
preserves utility-operations records past the active
retention horizon.

References (CITATION-POLICY ALLOW only):

- IEC 61968, IEC 61970, IEC 61850, IEC 62325, IEC
  62351, IEC 62056, IEC 61400-25 series
- ANSI C12.18 / C12.19 / C12.22 (utility metering)
- IEEE 1547-2018 + IEEE 2030.5 + IEEE 1815 (DNP3) +
  IEEE 1815.1 (DNP3 / IEC 61850 mapping)
- IEEE 1588 PTP, IEEE 1366-2022 reliability indices
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 27019:2024, ISO/IEC
  17021-1:2015, ISO/IEC 17065:2012, ISO 50001:2018
- ISO 16484 series (BACnet building-automation
  systems)
- ASHRAE Standard 135 (BACnet)
- W3C Verifiable Credentials Data Model 2.0
- Project Haystack (semantic-tagging convention)
- US FERC Order 2222 + Order 2003-D
- US DOE OE GMI initiatives + DOE C2M2
- NERC CIP-002 ~ CIP-014 + BAL + IRO + COM + EOP
- KR 전기사업법 + 전력시장운영규칙 + KPX 운영규정 +
  KEPIC + KR 정보통신망법 + KISA + KrCERT-CC
- US OE-417 + EU NIS2 (Directive (EU) 2022/2555)

---

## §1 Substation-Automation Integration

The operator's energy-cloud integrates with the
substation-automation substrate:

- IEC 61850 SCD (Substation Configuration
  Description) is the canonical substation-engineering
  document; the operator's cloud-side ingest
  consumes the SCD for situational awareness.
- GOOSE and Sampled-Values traffic typically remains
  on-premise (LAN-class latency); the operator's
  cloud receives status snapshots via MMS or via
  IEEE C37.118 synchrophasor streams.
- Edge gateways translate IEC 61850 / DNP3 / Modbus
  to the cloud-side IEC 61968 / 61970 envelopes.

## §2 AMI and Meter Data-Management Integration

For utility-side metering data:

- IEC 62056 (DLMS / COSEM) and ANSI C12.18 / C12.19
  / C12.22 are the on-premise meter-protocol
  baselines.
- The AMI head-end aggregates meter reads and
  forwards to the cloud-side MDMS via IEC 61968-9
  meter-reading messages.
- The cloud-side VEE process produces validated
  reads for billing-and-settlement use.

## §3 Wholesale-Market Integration

The wholesale-market integration:

- US-jurisdiction — the operator's market-
  participation surface integrates with CAISO /
  PJM / MISO / ERCOT / NYISO / ISO-NE / SPP per
  the relevant tariff.
- EU-jurisdiction — the operator's market surface
  integrates with the relevant TSO and the
  ENTSO-E + EUPHEMIA day-ahead / intraday markets
  using IEC 62325 schemas.
- KR-jurisdiction — the operator integrates with
  KPX 전력거래소 under 전력시장운영규칙.

## §4 FERC / NERC / Member-State NCA Integration

For US-jurisdiction operators:

- NERC for CIP audits and self-reports.
- FERC for Reg SCI-equivalent reliability standards
  enforcement.
- US DOE OE-417 reporting for electric emergencies.
- US CISA for cybersecurity-incident reporting under
  CIRCIA.

For EU-jurisdiction operators:

- The Member-State NCA for energy-sector regulation.
- ENTSO-E for transmission-system coordination.
- NIS2 (Directive (EU) 2022/2555) — for essential
  energy entities the incident-reporting under
  Article 23 applies.

For KR-jurisdiction:

- KR FSC + 산업통상자원부 for sector-regulatory
  oversight.
- KR KEPCO + 한국전력거래소 for wholesale-market
  coordination.
- KR KISA + KrCERT-CC for cybersecurity-incident
  reporting under 정보통신망법.

## §5 DSO and TSO Neighbour Integration

The operator's TSO-DSO interface integration:

- Bilateral data-exchange agreements with
  neighbouring TSOs / DSOs.
- ENTSO-E Communication and Information Manager
  (CIM-aligned) for transmission-system data
  exchange in EU.
- TSO-DSO data exchange under EU Network Code on
  System Operation (Reg (EU) 2017/1485) and the
  Network Code on Demand Connection (Reg (EU)
  2016/1388).

## §6 Third-Party DERMS / DR Aggregator Integration

The operator's catalogue-and-app discipline supports
third-party platforms:

- Per-aggregator OAuth client registration with the
  operator's IdP.
- Per-aggregator approved-customer-list with bulk-
  customer-consent capture under the operating
  jurisdiction's data-protection regime.
- Bilateral data-exchange agreements covering
  customer-data scope, settlement processes, and
  liability allocation.

## §7 Customer-Bill Engine and CIS Integration

The operator's billing and customer-information-
system integration:

- Per-tenancy meter reads, program-enrolment, and
  rate-tier assignments are forwarded to the CIS
  for bill production.
- Energy efficiency program credits, demand-response
  program payments, and net-metering credits flow
  through the CIS.
- Bill-print and electronic-bill delivery are
  managed at the CIS.

## §8 Building-Energy-Management Integration

For commercial-building energy management:

- ISO 16484 + ASHRAE BACnet (Standard 135) is the
  on-premise building-automation baseline.
- Project Haystack semantic tagging is used by
  building-energy-management software to interpret
  point names and units.
- The operator's cloud exposes per-tenancy energy-
  data subscriptions (e.g., Green Button Connect My
  Data) for commercial customers.

## §9 Cybersecurity Supervisor Integration

For US bulk-electric-system operators NERC + CISA
+ DOE OE-417 reporting; for EU operators ENISA + NIS2
incident reporting; for KR operators KISA + KrCERT-CC
+ FSC + 한국에너지공단 reporting.

## §10 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the ISO/IEC 27019:2024 energy-sector
extension applied. The certification body operates
under ISO/IEC 17021-1; the conformity-assessment body
for WIA-energy-cloud operates under ISO/IEC 17065.
ISO 50001 energy-management certification is held by
the operator where applicable.

## §11 Long-Term Archival Integration

Records governed by the operator's retention horizons
(NERC CIP retention; FERC retention; KR 전기사업법
+ KPX 보존) are migrated to the long-term archive at
the close of the active retention window. The archive
preserves the CIM-model snapshots, the DER-fleet
aggregations, the forecast records, the market-
participation records, the engagement records, the
operations-event records, and the audit-event trail.

## §12 Climate-Disclosure Integration

For operators subject to ESG-disclosure regimes the
operator's energy-cloud benefits the entity's GHG-
inventory (avoided emissions through demand-response,
renewable integration, EV-charging optimisation)
recorded under the WIA-esg-finance disclosure record
(ISSB IFRS S2 + ESRS E1) where the entity is a CSRD
or ISSB-reporting entity.

## §13 EV-Charging Network Integration

For utility-side EV-charging integration:

- The OCPP 2.0.1 CSMS at the EV-charge-network
  operator publishes charging-station status and
  smart-charging acceptance to the operator's cloud.
- Bilateral OCPI (Open Charge Point Interface)
  agreements enable cross-network roaming.
- Vehicle-grid-integration analytics combine the
  charging-load forecast with the distribution-
  feeder hosting-capacity analysis to optimise
  charging schedules.

## §14 Distributed-Energy-Resource Hosting Capacity
        Integration

The hosting-capacity-analysis (HCA) integration:

- The ADMS / DERMS computes per-feeder hosting-
  capacity maps and exposes them to the
  interconnection-application portal.
- Customer-and-installer applications consult the
  HCA to confirm the feasibility of a proposed DER
  installation.
- The HCA is refreshed on the operator's published
  cadence as DER penetration evolves.

## §15 Wholesale-Market Settlement Engine Integration

The settlement-engine integration:

- The operator forwards meter-reads, dispatch
  instructions, and award-acknowledgements to the
  RTO / ISO settlement engine via the relevant
  market interface.
- Settlement statements are reconciled against the
  operator's internal accounting; reconciliation
  exceptions are tracked through the operator's
  settlement-dispute workflow.
- For KR-jurisdiction the KPX 정산시스템 reconciles
  against KEPCO retail tariffs and the operator's
  internal accounting.

## §16 Disaster-Recovery Geographic-Diversity
        Integration

The DR site's geographic-diversity integration:

- The DR site mirrors the production CIM model,
  the production AMI/SCADA telemetry, and the
  production audit log on the operator's published
  RPO.
- The DR-failover procedure is exercised on the
  DR-drill cadence (typically annual full-stack +
  quarterly tabletop).
- Recovery-time objective alignment with the
  operating jurisdiction's reliability expectations
  (NERC EOP-008-2 cyber-attack contingency planning;
  NIS2 Article 21 minimum cybersecurity measures).

## §17 OPC-UA / IEC 62541 Industrial-Telemetry Bridge

For industrial-customer telemetry consumers the
operator's bridge to OPC UA (IEC 62541) supports
direct integration with the customer's plant-level
historian:

- The operator's OPC-UA-aligned data model maps to
  IEC 61968 message envelopes.
- Per-customer signed certificates establish the
  trust between the operator and the customer's
  OPC-UA client.

## §18 Time-of-Use and Tariff-Engine Integration

The tariff-engine integration carries the operating
jurisdiction's published rate-structure into the
operator's billing pipeline:

- Time-of-use periods (peak, mid-peak, off-peak)
  with seasonal variation.
- Critical-peak-pricing event triggers and
  customer-facing notifications.
- Real-time-pricing pass-through where applicable.
- Per-customer-class rate selection and tariff-
  switching workflows.

## §19 Conformance

Implementations claiming PHASE-4 conformance maintain
the substation-automation, AMI, wholesale-market, and
neighbouring-utility integrations, exercise the
supervisory-authority filing obligations, hold the
ISO/IEC 27001 + 27019 certifications, exercise the
DOE C2M2 cybersecurity-maturity discipline, and
operate the long-term archival integration described
above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-energy-cloud
- **Last Updated:** 2026-04-28
