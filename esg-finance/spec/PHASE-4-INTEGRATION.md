# WIA-esg-finance PHASE 4 — INTEGRATION Specification

**Standard:** WIA-esg-finance
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an ESG-disclosure operator
integrates with the systems that surround the
sustainability-disclosure lifecycle: the operating
jurisdiction's officially appointed mechanism for
storage of regulated information (OAM in EU, EDGAR
in US, KIND / DART in KR); the supervisory authority
(ESMA + Member-State NCA in EU, SEC in US, KR FSC +
KCSE in KR); the assurance provider; the SBTi public
dashboard; the CDP scoring platform; the ratings-
agency feeds (MSCI ESG, Sustainalytics, ISS ESG,
Refinitiv); the financial-product distribution chain
(SFDR Article 8 / 9 product factories and
distributors); the investor-engagement channel; the
operator's chain-of-activities partner integration
under CSDDD; and the long-term archive that preserves
disclosure artefacts past the active retention
horizon.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015 (audit and certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 14064-1 + 14064-3 + 14067
- ISO 8601, ISO 17442 LEI
- W3C Verifiable Credentials Data Model 2.0
- IFRS Foundation ISSB IFRS S1 + S2
- EU CSRD (2022/2464) + ESRS (2023/2772) + ESEF
  Regulation (Reg 2018/815) for digital-tagging
- EU Taxonomy Regulation (2020/852) + the three
  delegated acts (2021/2139, 2021/2178, 2023/2486)
- EU SFDR (2019/2088) + RTS (2022/1288)
- EU CSDDD (2024/1760)
- EU Green Claims Directive (proposed) and
  Empowering Consumers (2024/825)
- ISAE 3000 + ISAE 3410 + ISSA 5000
- GHG Protocol Corporate + Scope 3
- SBTi Corporate Net-Zero Standard
- CDP Climate / Forests / Water Security
  questionnaires
- GRI Standards (Universal 2021 + Sector + Topic)
- SASB Industry Standards
- UN PRI Reporting Framework
- UNEP FI Principles for Responsible Banking + NZBA
- KR FSC ESG 공시 의무화 + KIND + DART + KCSE
- US SEC EDGAR + 17 CFR 229.1500 to 1506
- US California SB-253 + SB-261

---

## §1 Officially Appointed Mechanism (OAM) Integration

The entity's regulated information is filed with the
Member-State officially appointed mechanism (OAM) under
the EU Transparency Directive 2004/109/EC + ESEF
Regulation 2018/815. The integration carries:

- The OAM's filing endpoint and authentication
  credential.
- The ESEF + ESRS 2023/2772 digital-tagging XBRL
  payload (the entity's annual financial report and
  sustainability statement combined).
- The operator's filing acknowledgement and the
  OAM-published reference identifier.

For US-jurisdiction filers the equivalent EDGAR
filing channel + 17 CFR 232.405 retention; for KR-
jurisdiction filers the KIND (Korea Investor's
Network for Disclosure) / DART (Data Analysis,
Retrieval and Transfer) channels.

## §2 Supervisory-Authority Integration

For EU-regulated operators:

- ESMA — for EU-wide ESG-disclosure thematic
  reviews, Q&A guidance, common supervisory action
  on SFDR, and direct supervision of certain
  EU-level activities.
- Member-State NCA — for the operator's authorisation,
  ongoing supervision, CSRD enforcement (under the
  Member-State implementation of Directive (EU)
  2022/2464), and SFDR enforcement.

For US-regulated operators:

- SEC — for the climate-related disclosure rules
  (17 CFR 229.1500 to 229.1506), the SEC examination
  staff (Division of Corporation Finance), and the
  SEC's enforcement function.
- State attorneys-general — for state-level UDAAP
  and consumer-protection enforcement (notably
  state-level greenwashing complaints).

For KR-regulated operators:

- KR FSC — for the FSC's ESG disclosure mandate
  (phased coverage of KOSPI-listed companies).
- KR KCSE (Korea Council for Sustainability
  Standards) — for the KR-jurisdiction sustainability
  standard-setting and enforcement.

## §3 Assurance-Provider Integration

The operator's assurance provider integrates with:

- The materiality-assessment record (PHASE-1 §4)
  for the CSRD assurance scope.
- The ISSB / ESRS disclosure record (PHASE-1 §5 +
  §6) for the assurance opinion.
- The GHG inventory (PHASE-1 §9) for the ISAE 3410
  / ISO 14064-3 verification.
- The taxonomy alignment (PHASE-1 §7) for the
  taxonomy-alignment-eligibility opinion.

The assurance opinion is recorded in PHASE-1 §12
and published alongside the entity's annual report.

## §4 SBTi and CDP Integration

For SBTi-validated targets the operator integrates
with the SBTi public dashboard — target submission,
validation correspondence, annual progress disclosure,
and the SBTi Net-Zero recommitment workflow.

For CDP the operator integrates with the CDP
disclosure platform — the climate questionnaire,
the forests questionnaire, the water-security
questionnaire — and feeds the CDP scoring algorithm.

## §5 Ratings-Agency Integration

The operator's investor-facing data feed integrates
with the major ESG ratings agencies:

- MSCI ESG Ratings — the entity's MSCI ESG profile
  is updated through the entity's CDP submission,
  the entity's public-disclosure surface, and the
  entity's MSCI ESG manager engagement.
- Sustainalytics ESG Risk Ratings — the entity's
  Sustainalytics profile is updated through similar
  channels.
- ISS ESG Corporate Rating — through the ISS ESG
  data-collection process.
- Refinitiv ESG Scores — through the Refinitiv data
  ingestion.

The entity's investor-relations function manages
the bilateral data-quality dialogue with each agency.

## §6 SFDR Product-Distribution Integration

For financial-market participants distributing
SFDR Article 8 / 9 financial products:

- The pre-contractual disclosure template feeds the
  distribution channel (KID, prospectus).
- The website disclosure feeds the entity's product
  marketing pages.
- The periodic disclosure is published with the
  fund's annual report.
- The MiFID II suitability assessment uses the SFDR
  classification to match the product to the
  client's sustainability preferences.

## §7 CSDDD Chain-of-Activities Integration

For in-scope CSDDD entities:

- Tier-1 supplier engagement — the operator's
  supplier-onboarding process collects the supplier's
  human-rights and environmental due-diligence
  evidence.
- Tier-2-and-beyond engagement — the operator
  applies the CSDDD risk-prioritisation discipline
  to extend due-diligence beyond Tier 1 where
  identified risks warrant.
- Complaints channel — the CSDDD Article 14
  notification mechanism is integrated with the
  operator's whistleblower programme (under EU
  Whistleblower Directive 2019/1937 implementation).
- Remediation tracker — the operator's remediation
  plan execution is tracked across the chain-of-
  activities partners.

## §8 ICT Third-Party Provider Integration (DORA where
       applicable)

For ESG-disclosure operators that are also financial
entities subject to DORA Regulation (EU) 2022/2554:

- Map ICT third-party providers supporting the
  ESG-disclosure pipeline (sustainability-data
  vendors, GHG-calculation engines, XBRL-tagging
  tools).
- Apply the DORA contractual minima.
- Report critical ICT third-party providers to the
  Member-State NCA.

## §9 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the scope explicitly extending to
the disclosure-pipeline, public-disclosure, and
examination endpoints. The certification body
operates under ISO/IEC 17021-1; the conformity-
assessment body for WIA-esg-finance operates under
ISO/IEC 17065. ISO 14064-3 verification body
accreditation is held by the entity's GHG verifier.

## §10 Long-Term Archival Integration

Records governed by the operator's retention
horizons (EU CSRD ten-year archival; SFDR five-year;
US SEC 17 CFR 232.405; KR FSC 공시 보존) are
migrated to the long-term archive at the close of
the active retention window. The archive preserves
the materiality-assessment record, the ISSB / ESRS /
Taxonomy / SFDR / GHG / SBTi / CSDDD records, the
assurance reports, the supervisory correspondence,
and the audit-event trail.

## §11 Investor-Engagement Channel

The operator's investor-engagement channel covers:

- Annual investor presentation on the entity's
  sustainability strategy and progress.
- Q1 / Q3 update calls referencing the entity's
  transition-plan execution.
- Bilateral engagement on shareholder resolutions
  and proxy-vote outcomes (under UK Stewardship
  Code, EU Shareholder Rights Directive II, KR
  스튜어드십 코드).
- Climate-and-biodiversity transition-plan engagement
  under the UK Transition Plan Taskforce / EU
  Sustainable Finance Disclosure framework.

## §12 Public-Disclosure Surface Integration

The operator's public-disclosure surface (the
entity's investor-relations website, the entity's
annual report PDF, the entity's ESEF-tagged XBRL)
is integrated with:

- The European Single Access Point (ESAP) once in
  force under the EU Capital Markets Union initiative.
- The KR DART central disclosure repository.
- The US SEC EDGAR central repository.
- The entity's own corporate website.

## §13 Conformance

Implementations claiming PHASE-4 conformance maintain
the OAM and supervisory-authority integrations,
exercise the SBTi / CDP / ratings-agency feeds where
the operator participates, hold the ISO/IEC 27001
certification + ISO 14064-3 verifier accreditation,
and operate the long-term archival integration
described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-esg-finance
- **Last Updated:** 2026-04-28
