# WIA-esg-finance PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-esg-finance
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer
for WIA-esg-finance. The standard covers persistent
record shapes for the lifecycle of a corporate
sustainability disclosure programme — the reporting
entity and its consolidation perimeter; the materiality-
assessment record (the EU CSRD double-materiality
discipline); the ISSB IFRS S1 / S2 climate-related
disclosure record; the EU CSRD / ESRS disclosure record
across the environmental, social, and governance
topical standards; the EU Taxonomy alignment record;
the EU SFDR Article 6 / 8 / 9 product classification
record (where the entity is a financial-market
participant); the GHG Protocol Scope 1 / 2 / 3
inventory; the SBTi target and progress record; the
TCFD disclosure cross-walk; the GRI Universal +
Sector + Topic disclosure cross-walk; the SASB Industry
Standards disclosure cross-walk; the EU CSDDD
(Corporate Sustainability Due Diligence Directive)
due-diligence record; the assurance-and-audit record;
and the supervisory-correspondence record. Records
are consumed by the entity's board (including the
audit committee), the entity's external assurance
provider, the supervisory authority for the operating
jurisdiction (ESMA + Member-State NCA in EU; SEC and
state attorneys-general in the US; FSC + KCSE in KR),
investors and lenders, capital-markets analysts, and —
through the public-disclosure surface — civil society
and affected-rights holders.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO 14064-1:2018 (organisation-level GHG inventory)
- ISO 14064-3:2019 (GHG verification and validation)
- ISO 14067:2018 (carbon footprint of products)
- ISO 17442 (LEI)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- IFRS Foundation ISSB IFRS S1 (General Requirements
  for Disclosure of Sustainability-related Financial
  Information) and IFRS S2 (Climate-related
  Disclosures)
- TCFD (Task Force on Climate-related Financial
  Disclosures) Recommendations (now consolidated
  into IFRS S2)
- EU CSRD (Directive (EU) 2022/2464) amending the
  Accounting Directive 2013/34/EU
- EU ESRS (the European Sustainability Reporting
  Standards adopted by Commission Delegated Regulation
  (EU) 2023/2772 — ESRS 1 General requirements; ESRS
  2 General disclosures; ESRS E1 Climate change; ESRS
  E2 Pollution; ESRS E3 Water and marine resources;
  ESRS E4 Biodiversity and ecosystems; ESRS E5
  Resource use and circular economy; ESRS S1 Own
  workforce; ESRS S2 Workers in the value chain;
  ESRS S3 Affected communities; ESRS S4 Consumers
  and end-users; ESRS G1 Business conduct)
- EU Taxonomy Regulation (Regulation (EU) 2020/852)
  and the Climate Delegated Act (Reg 2021/2139),
  Disclosures Delegated Act (Reg 2021/2178), and
  Environmental Delegated Act (Reg 2023/2486)
- EU SFDR (Sustainable Finance Disclosure Regulation
  (EU) 2019/2088) Articles 3, 4, 6, 8, 9, 10, 11
- EU CSDDD (Corporate Sustainability Due Diligence
  Directive (EU) 2024/1760) Articles 5 to 16 (due-
  diligence obligations across the chain of
  activities)
- GHG Protocol Corporate Accounting and Reporting
  Standard (revised) and Corporate Value Chain
  (Scope 3) Accounting and Reporting Standard
- SBTi (Science Based Targets initiative) Corporate
  Net-Zero Standard v1.x and SBTi Financial Sector
  guidance
- CDP (Carbon Disclosure Project) Climate / Forests
  / Water Security questionnaires
- GRI Standards (the GRI Universal Standards 2021,
  the Sector Standards programme, and the Topic
  Standards)
- SASB Industry Standards (now under the IFRS
  Foundation umbrella as part of the ISSB)
- UN PRI (Principles for Responsible Investment)
  reporting framework
- UNEP FI Principles for Responsible Banking and
  Net-Zero Banking Alliance commitments
- KR ESG 공시 의무화 (the KR FSC's phased ESG
  disclosure mandate) and KR K-ESG 가이드라인 (the
  Ministry of Trade, Industry and Energy's published
  K-ESG framework)
- US SEC climate-related disclosure rules (17 CFR
  229.1500 to 229.1506; 17 CFR 232.405) where in
  force after the Commission's adoption and any
  judicial stay
- US California SB-253 Climate Corporate Data
  Accountability Act and SB-261 Climate-Related
  Financial Risk Act

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
an ESG-disclosure operator (a corporate reporting
entity, a financial-market participant subject to
SFDR, a credit institution under the Net-Zero Banking
Alliance, an asset manager subject to the UN PRI, or
an assurance provider) maintains:

- The reporting entity record and consolidation
  perimeter.
- The double-materiality assessment record.
- The ISSB / IFRS S1 / S2 disclosure record.
- The EU CSRD / ESRS disclosure record.
- The EU Taxonomy alignment record.
- The EU SFDR Article 6 / 8 / 9 product
  classification record.
- The GHG Protocol Scope 1 / 2 / 3 inventory.
- The SBTi target and progress record.
- The CSDDD due-diligence record.
- The assurance and audit record.
- The supervisory correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
entityName           : string (legal name)
entityRole           : enum ("reporting-entity-eu-
                       csrd" | "reporting-entity-issb"
                       | "reporting-entity-sec" |
                       "reporting-entity-kr" |
                       "financial-market-participant-
                       sfdr" | "asset-manager-pri" |
                       "credit-institution-nzba" |
                       "assurance-provider" |
                       "user-defined")
entityLei            : string (ISO 17442)
entityJurisdiction   : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("ISSB-IFRS-S1"
                       | "ISSB-IFRS-S2" |
                       "TCFD" | "EU-CSRD-2022-2464"
                       | "EU-ESRS-2023-2772" |
                       "EU-TAXONOMY-2020-852" |
                       "EU-TAXONOMY-CLIMATE-DEL-
                       2021-2139" | "EU-TAXONOMY-
                       DISCLOSURES-DEL-2021-2178" |
                       "EU-TAXONOMY-ENVIRONMENTAL-
                       DEL-2023-2486" |
                       "EU-SFDR-2019-2088" |
                       "EU-CSDDD-2024-1760" |
                       "GHG-PROTOCOL-CORPORATE" |
                       "GHG-PROTOCOL-SCOPE-3" |
                       "ISO-14064-1" | "ISO-14064-3"
                       | "ISO-14067" | "SBTi-NET-
                       ZERO" | "CDP-CLIMATE" |
                       "GRI-UNIVERSAL-2021" |
                       "SASB-INDUSTRY" | "UN-PRI" |
                       "UNEP-FI-PRB" |
                       "KR-FSC-ESG-공시-의무화" |
                       "KR-MOTIE-K-ESG-가이드라인" |
                       "US-SEC-17-CFR-229-1500" |
                       "US-CA-SB-253" |
                       "US-CA-SB-261" |
                       "user-defined")
reportingPeriod      : object (the fiscal year start
                       and end dates per the entity's
                       financial reporting calendar)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Consolidation Perimeter Record

```
consolidationPerimeter:
  perimeterId        : string (uuidv7)
  consolidationBasis : enum ("ifrs-control" |
                       "operational-control" |
                       "financial-control" |
                       "equity-share" | "user-
                       defined") (the GHG Protocol
                       allows operational, financial,
                       or equity-share consolidation;
                       ISSB IFRS S1 follows IFRS
                       Accounting Standards control)
  entitiesIncluded   : array of object (legal-entity
                       references with their LEI and
                       inclusion percentage)
  scopeBoundaryDescription : string (the entity's
                       documented narrative for the
                       scope of consolidation)
```

## §4 Double-Materiality Assessment Record (EU CSRD /
       ESRS 1)

The double-materiality discipline under ESRS 1 §3
requires the entity to assess both impact materiality
(the entity's impacts on people and the environment)
and financial materiality (sustainability matters
that affect the entity's financial performance,
position, cash flows, cost of capital, or access to
finance):

```
materialityAssessment:
  assessmentId       : string (uuidv7)
  conductedAt        : string (ISO 8601)
  assessmentScope    : array of object (per-topic
                       assessment — climate change,
                       pollution, water, biodiversity,
                       resource use, own workforce,
                       value-chain workers, affected
                       communities, consumers,
                       business conduct)
  impactMaterialityOutcome : array of object (per-
                       topic impact-materiality
                       conclusion with the supporting
                       evidence reference)
  financialMaterialityOutcome : array of object (per-
                       topic financial-materiality
                       conclusion with the supporting
                       evidence reference)
  stakeholderEngagementRef : string (URI of the
                       stakeholder-engagement
                       narrative; ESRS 1 requires the
                       entity to engage affected
                       stakeholders)
  approvedBy         : string (the entity's audit
                       committee or sustainability
                       committee reference)
```

## §5 ISSB IFRS S1 / S2 Disclosure Record

The ISSB disclosure record encodes the four-pillar
TCFD-aligned IFRS S2 architecture (governance,
strategy, risk management, metrics and targets) plus
the IFRS S1 general requirements:

```
issbDisclosureRecord:
  disclosureId       : string (uuidv7)
  reportingPeriodRef : string (PHASE-1 §2)
  governanceNarrativeRef : string (URI of the IFRS
                       S2 §6 governance disclosure
                       narrative — board oversight,
                       management's role)
  strategyNarrativeRef : string (URI of the IFRS S2
                       §9 strategy disclosure —
                       climate-related risks and
                       opportunities, business model
                       and value chain, transition
                       plan)
  riskManagementNarrativeRef : string (URI of the
                       IFRS S2 §25 risk-management
                       disclosure)
  metricsTargetsRef  : object (the IFRS S2 §29
                       metrics and targets — Scope 1,
                       2, 3 emissions; cross-industry
                       metrics; industry-specific
                       metrics from SASB Industry
                       Standards)
  scenarioAnalysisRef : string (URI of the climate-
                       resilience scenario-analysis
                       narrative under IFRS S2 §22)
```

## §6 EU CSRD / ESRS Disclosure Record

```
esrsDisclosureRecord:
  disclosureId       : string (uuidv7)
  reportingPeriodRef : string
  esrs1GeneralRequirementsRef : string
  esrs2GeneralDisclosuresRef : string
  topicalDisclosures : array of object (per-topical-
                       standard disclosure reference
                       — ESRS E1 climate, E2
                       pollution, E3 water and marine,
                       E4 biodiversity, E5 resource
                       use and circular economy, S1
                       own workforce, S2 value-chain
                       workers, S3 affected
                       communities, S4 consumers, G1
                       business conduct)
  digitalTaggingRef  : string (URI of the XBRL
                       tagged digital report — ESRS
                       2023/2772 mandates digital
                       tagging per the ESEF Regulation)
```

## §7 EU Taxonomy Alignment Record

```
taxonomyAlignment:
  alignmentId        : string (uuidv7)
  reportingPeriodRef : string
  economicActivities : array of object (per-NACE
                       activity reference; the
                       Taxonomy Climate Delegated Act
                       (Reg 2021/2139) and the
                       Environmental Delegated Act
                       (Reg 2023/2486) define the
                       eligible activities for the
                       six environmental objectives —
                       climate-change mitigation,
                       climate-change adaptation,
                       water and marine resources,
                       circular economy, pollution
                       prevention and control,
                       biodiversity protection)
  taxonomyEligibleKpi : object (per-KPI eligible
                       turnover, capex, opex
                       fractions)
  taxonomyAlignedKpi : object (per-KPI aligned
                       turnover, capex, opex fractions
                       — the substantial-contribution
                       criterion, the do-no-significant-
                       harm criterion, and the minimum
                       safeguards under Article 18 of
                       Regulation (EU) 2020/852 are
                       all satisfied)
  minSafeguardsAttestationRef : string (URI of the
                       minimum-safeguards attestation
                       narrative covering the OECD
                       Guidelines for Multinational
                       Enterprises, the UN Guiding
                       Principles on Business and
                       Human Rights, and the ILO
                       Declaration on Fundamental
                       Principles and Rights at Work)
```

## §8 EU SFDR Product-Classification Record

For financial-market participants:

```
sfdrProductRecord:
  productId          : string (uuidv7)
  productName        : string
  classificationKind : enum ("art-6-mainstream" |
                       "art-8-promoting-
                       environmental-or-social" |
                       "art-9-sustainable-
                       investment-objective")
  preContractualDisclosureRef : string (URI of the
                       SFDR Article 6 / 8 / 9 pre-
                       contractual disclosure)
  periodicDisclosureRef : string (URI of the SFDR
                       Article 11 periodic disclosure)
  paiStatementRef    : string (URI of the principal-
                       adverse-impacts statement under
                       SFDR Article 4 + RTS Annex I;
                       absent unless the FMP exceeds
                       the 500-employee threshold or
                       opts in)
```

## §9 GHG Protocol Inventory and Verification Record

```
ghgInventory:
  inventoryId        : string (uuidv7)
  reportingPeriodRef : string
  scope1Emissions    : object (tCO2e total + per
                       facility / activity breakdown
                       — direct emissions from owned
                       or controlled sources)
  scope2Emissions    : object (tCO2e total under
                       both location-based and
                       market-based methods — Scope
                       2 Guidance Amendment requires
                       both)
  scope3Emissions    : object (tCO2e total + per-
                       category-1-15 breakdown —
                       purchased goods, capital
                       goods, fuel-and-energy
                       activities, upstream
                       transportation, waste,
                       business travel, employee
                       commuting, upstream leased
                       assets, downstream
                       transportation, processing,
                       use of sold products, end-
                       of-life of sold products,
                       downstream leased assets,
                       franchises, investments)
  consolidationApproach : enum ("operational-control"
                       | "financial-control" |
                       "equity-share")
  verificationKind   : enum ("limited-assurance-iso-
                       14064-3" | "reasonable-
                       assurance-iso-14064-3" |
                       "self-reported-no-assurance")
  verificationProviderRef : string (the ISO 14064-3
                       accredited verifier; absent if
                       self-reported)
  verifiedAt         : string (ISO 8601)
```

## §10 SBTi Target and Progress Record

```
sbtiTarget:
  targetId           : string (uuidv7)
  targetKind         : enum ("near-term-1-5-2-c-
                       aligned" | "near-term-well-
                       below-2-c" | "long-term-net-
                       zero-corporate-net-zero-
                       standard" | "fls-financial-
                       sector-target" | "user-
                       defined")
  baseYear           : integer (calendar year)
  targetYear         : integer
  scopesCovered      : array of enum ("scope-1" |
                       "scope-2" | "scope-3")
  reductionPercentage : number (vs. base year)
  validationOutcome  : enum ("submitted" |
                       "validated" | "needs-revision"
                       | "expired")
  progressRecord     : array of object (per-year
                       actual emissions trajectory
                       vs. linear path to target)
```

## §11 CSDDD Due-Diligence Record

```
csdddDueDiligence:
  dueDiligenceId     : string (uuidv7)
  reportingPeriodRef : string
  chainOfActivitiesScope : object (the entity's chain-
                       of-activities scope per CSDDD
                       Article 3(g))
  riskIdentificationRef : string (URI of the adverse-
                       human-rights-and-environmental-
                       impact identification
                       narrative — CSDDD Article 8)
  preventionPlanRef  : string (URI of the prevention
                       and minimisation plan — CSDDD
                       Article 10)
  remediationPlanRef : string (URI of the actual-
                       impact remediation plan —
                       CSDDD Article 11)
  stakeholderEngagementRef : string (URI of the
                       affected-stakeholder engagement
                       — CSDDD Article 13)
  complaintsChannelRef : string (URI of the
                       complaints procedure — CSDDD
                       Article 14)
  monitoringRef      : string (URI of the periodic
                       monitoring narrative — CSDDD
                       Article 15)
```

## §12 Assurance and Audit Record

```
assuranceRecord:
  assuranceId        : string (uuidv7)
  reportingPeriodRef : string
  assuranceProviderRef : string (the assurance
                       provider's identity and ISAE
                       3000 / ISAE 3410 / ISO 14064-3
                       accreditation)
  assuranceLevel     : enum ("limited-assurance" |
                       "reasonable-assurance")
  scope              : array of enum ("issb-ifrs-s2"
                       | "esrs-environmental" |
                       "esrs-social" | "esrs-
                       governance" | "ghg-scope-1"
                       | "ghg-scope-2" | "ghg-scope-
                       3" | "taxonomy-alignment" |
                       "user-defined")
  reportRef          : string (URI of the assurance
                       report)
  issuedAt           : string (ISO 8601)
```

## §13 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for every reporting period,
exercise the double-materiality discipline annually,
emit the IFRS S1 / S2 / ESRS / Taxonomy / SFDR / GHG
Protocol / SBTi disclosures within the operating
jurisdiction's filing deadline, and preserve the
records under the operating jurisdiction's
recordkeeping discipline (EU CSRD ten-year archival;
US SEC 17 CFR 232.405 retention; KR FSC 공시 보존
discipline).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-esg-finance
- **Last Updated:** 2026-04-28
