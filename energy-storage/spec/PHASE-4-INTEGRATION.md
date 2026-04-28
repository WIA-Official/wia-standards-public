# WIA-energy-storage PHASE 4 — INTEGRATION Specification

**Standard:** WIA-energy-storage
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an energy-storage operator
integrates with the systems that surround the
storage-asset lifecycle: the manufacturer's portal
for BMS firmware-and-warranty management; the local
fire department / AHJ for site-permit and emergency-
response coordination; the wholesale-market
RTO / ISO under FERC Order 841 storage-market rules;
the utility distribution-or-transmission operator
under IEEE 1547-2018 + 1547.9; the recycler / second-
life partner under the operating jurisdiction's
battery-recycling regime; the supervisory authority
for the operating jurisdiction; the external auditor
and ISO/IEC 27001 certification body; and the long-
term archive that preserves storage-operations
records past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- IEC 62933, IEC 62619, IEC 62620, IEC 60086, IEC
  61960-3
- IEEE 1547-2018 + 1547.9-2022 + 1679-2020 +
  2030.2.1-2019
- UL 9540 + UL 9540A + UL 1973
- NFPA 855 + NFPA 68 + NFPA 69 + NFPA 70 NEC
  Article 706
- UN 38.3 + UN ECE Regulation R100.03
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015, ISO/IEC
  17065:2012
- ISO 8601
- US FERC Order 841 + Order 2222
- US OE-417 + EU NIS2 + KR 정보통신망법
- EU Battery Regulation (Reg (EU) 2023/1542)
- US RCRA + KR 폐기물관리법 + KR 전기·전자제품 및
  자동차의 자원순환에 관한 법률
- US OSHA 29 CFR Part 1910 + KR 산업안전보건법

---

## §1 Manufacturer Portal Integration

The operator integrates with the manufacturer's
portal for:

- BMS firmware updates with secure-boot signed
  firmware.
- Warranty status and claim submission.
- Cell / module / pack-level analytics shared back
  to the manufacturer for fleet-learning.
- Recall notifications (where the manufacturer
  identifies a fleet-wide defect).

## §2 Local Fire Department / AHJ Integration

The integration with the AHJ:

- Pre-incident plan delivery — the operator delivers
  the NFPA 855 pre-incident plan to the local fire
  department covering chemistry, topology, suppression
  characteristics, and recommended fire-suppression
  tactics.
- Annual emergency-response drill — the operator
  exercises a tabletop or full-stack drill with the
  fire department on the published cadence.
- Site-permit renewal — the operator renews the
  fire-code site permit on the jurisdiction's
  cadence.
- For KR-jurisdiction the integration extends to KR
  소방청 + 한국전기안전공사.

## §3 Wholesale-Market Integration (FERC Order 841)

For US-jurisdiction storage operators participating
in FERC-jurisdictional wholesale markets:

- FERC Order 841 — storage-as-a-resource
  participation across the seven RTOs / ISOs
  (CAISO, PJM, MISO, ERCOT, NYISO, ISO-NE, SPP).
- Per-RTO storage-resource modelling (state-of-
  charge management, bidirectional bidding, energy-
  vs-ancillary co-optimisation).
- Settlement and performance reporting per the
  RTO / ISO tariff.

For KR-jurisdiction the equivalent KR KPX 전력거래소
+ 전력시장운영규칙 storage-resource integration.

## §4 Distribution / Transmission Utility Integration

For grid-connected storage:

- Interconnection-application portal under the
  utility's IEEE 1547-2018 + IEEE 1547.9 rules.
- Coordination with the utility's DERMS / ADMS for
  dispatch and protection-coordination updates.
- Hosting-capacity-analysis confirmation before
  permit issuance.
- For KR-jurisdiction the KEPCO 분산전원 계통연계
  기준 + KEPIC 적용.

## §5 Recycler and Second-Life Partner Integration

For end-of-life storage:

- Recycler engagement under the operating
  jurisdiction's battery-recycling regime — EU
  Battery Regulation 2023/1542 producer-
  responsibility scheme, US state-by-state
  recycling rules, KR 전기·전자제품 및 자동차의
  자원순환에 관한 법률.
- Second-life partner engagement for residual-
  capacity repurposing.
- Hazardous-waste manifest handling — US RCRA
  cradle-to-grave, KR 폐기물관리법 indication, EU
  Waste Framework Directive 2008/98/EC.
- Per-material recovery attestation captured for
  EU Battery Regulation 2023/1542 minimum recycled
  content compliance.

## §6 Supervisory-Authority Integration

For US-jurisdiction operators:

- NERC for CIP audits where the storage is part of
  the bulk-electric-system.
- FERC for Order 841 / 2222 enforcement.
- US OSHA for occupational-safety compliance.
- US EPA for hazardous-waste compliance under RCRA.

For EU-jurisdiction operators:

- The Member-State NCA for energy-sector regulation.
- The EU Battery Authority (under Reg 2023/1542)
  for due-diligence reporting on critical raw
  materials.
- NIS2 (Directive (EU) 2022/2555) for cybersecurity
  incident reporting where the operator is an
  essential or important entity.

For KR-jurisdiction operators:

- KR FSC + 산업통상자원부 + 한국에너지공단 + KEPCO
  + KPX.
- KR 한국전기안전공사 + 소방청.
- KR 산업안전보건공단 + 환경부.

## §7 EU Battery Regulation Due-Diligence Integration

For storage operators placing batteries on the EU
market under Reg (EU) 2023/1542:

- Carbon footprint declaration (Article 7).
- Recycled-content declaration (Article 8).
- Performance-and-durability declaration (Article 10).
- Removability and replaceability requirement
  (Article 11).
- Battery passport for industrial and EV batteries
  (Article 77).
- Due diligence for raw-material sourcing (Articles
  47 to 53) covering cobalt, lithium, natural
  graphite, and nickel.

## §8 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022. The certification body operates under
ISO/IEC 17021-1; the conformity-assessment body for
WIA-energy-storage operates under ISO/IEC 17065.
The UL 9540 + UL 9540A + IEC 62933-5-2 + IEC 62619
+ UL 1973 certifications are held by the
manufacturer and recorded against the asset
register.

## §9 Long-Term Archival Integration

Records governed by the operator's retention
horizons (NERC CIP retention; OSHA 30-year
exposure-record retention; EU Battery Regulation
10-year due-diligence retention; KR 전기사업법
보존) are migrated to the long-term archive at the
close of the active retention window. The archive
preserves the asset register, the certification
records, the BMS configuration history, the
state-record telemetry summaries, the cycle-life
records, the incident records, the warranty-claim
records, the end-of-life records, and the audit-
event trail.

## §10 EV-Fleet and Vehicle-to-Grid Integration

For EV-fleet operators with vehicle-storage
participation:

- Vehicle OEM API for state-of-charge / state-of-
  health / battery-management constraints.
- ISO 15118-20:2022 V2G messages for bidirectional
  power flow with the grid.
- Battery-warranty terms reflected in the operator's
  V2G dispatch policy so that battery-cycle wear
  remains within OEM-warranted bounds.
- Second-life repurposing of EV batteries as
  stationary storage.

## §11 Climate-Disclosure Integration

The operator's storage fleet contributes to
decarbonisation accounting through avoided-
emissions calculations (renewable integration,
peak-shifting, frequency-regulation). The operator's
WIA-esg-finance disclosure record (ISSB IFRS S2 +
ESRS E1) integrates the contribution where the
operator is a CSRD or ISSB reporting entity. The
EU Battery Regulation 2023/1542 carbon-footprint
declaration feeds the same disclosure record.

## §12 Insurance-Underwriting and Claim Integration

The operator's insurance integration covers:

- Property-and-business-interruption underwriting —
  the carrier's underwriting guidelines reference
  the UL 9540 + UL 9540A reports, the NFPA 855 site
  permit, and the operator's emergency-response
  programme.
- Claim integration — incident records (PHASE-1 §9)
  are forwarded to the carrier with the post-mortem
  narrative.
- Risk-engineering recommendations — the carrier's
  risk-engineer site visits feed the operator's
  remediation backlog.

## §13 Vendor-and-Supply-Chain Provenance Integration

For traceable raw-material provenance under EU
Battery Regulation Articles 47-53:

- Cobalt provenance — OECD Due Diligence Guidance
  for Responsible Supply Chains of Minerals from
  Conflict-Affected and High-Risk Areas + Cobalt
  Industry Responsible Assessment Framework (CIRAF).
- Lithium provenance — the operator captures the
  brine / mineral source per the supplier's
  declaration.
- Nickel provenance — sustainability programmes
  applied at the mine level.
- Natural graphite provenance — anode-grade
  graphite supply provenance.

## §14 Microgrid and Critical-Facility Backup
        Integration

For microgrid and critical-facility-backup
deployments:

- Microgrid-controller integration per IEEE 2030.7
  microgrid controller specification + IEEE 2030.8
  microgrid testing.
- Black-start support for facilities the storage
  is designated to start.
- Critical-load-priority list maintained at the
  facility owner's level; the storage's discharge
  schedule honours the priority during islanded
  operation.
- Coordination with local generation (diesel /
  natural-gas / fuel-cell) under the microgrid
  controller's economic-and-resilience optimiser.

## §15 Battery-Performance Test-Lab and Type-Approval
        Integration

For type-approval at storage-product introduction:

- The operator's selected test-lab integrates with
  the manufacturer for IEC 62933-2-1 / -2-2 +
  IEEE 1679-2020 performance characterisation.
- The lab's UL 9540A propagation-test report feeds
  the operator's NFPA 855 site-permit application.
- Type-approval is renewed at the operator's
  declared cadence (typically per major BMS
  firmware-class change or per cell-chemistry
  refresh).

## §16 Conformance

Implementations claiming PHASE-4 conformance maintain
the manufacturer-portal, AHJ, wholesale-market,
utility, recycler, insurance-carrier, and supervisory
integrations, exercise the EU Battery Regulation
2023/1542 due-diligence and battery-passport
discipline where the operator places batteries on
the EU market, hold the ISO/IEC 27001 certification,
exercise the type-approval lab integration on the
operator's cadence, and operate the long-term
archival integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-energy-storage
- **Last Updated:** 2026-04-28
