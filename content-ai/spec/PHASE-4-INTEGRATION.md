# WIA-content-ai PHASE 4 — INTEGRATION Specification

**Standard:** WIA-content-ai
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an AI-content platform
operator integrates with the systems that surround
the AI-assisted content lifecycle: the upstream
generative-AI model provider (WIA-generative-ai-
governed); the C2PA Content Credentials trust list
and the public verification ecosystem; the EU DSA
Transparency Database (DSA-TDB); the EU Member-State
Digital Services Coordinator (DSC) and the Commission
for VLOP / VLOSE oversight; the certified out-of-
court dispute settlement bodies; the trusted-flagger
network; the rights-holder representative bodies for
copyright-takedown; the law-enforcement requests-
channel; the supervisory data-protection authority;
the FTC / OFCOM / KCC / KR PIPC sector regulators;
the accessibility certification body; the
operator's vetted-researcher data-access programme
under DSA Article 40; the external auditor for the
ISO/IEC 42001 + ISO/IEC 27001 certifications; and
the long-term archive that preserves transparency
artefacts past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- C2PA Content Credentials specification v1.4 +
  Trust List
- ETSI TS 104 224
- IPTC Photo Metadata 2024
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 42001:2023
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- ISO 8601
- W3C Verifiable Credentials Data Model 2.0
  (optional)
- EU AI Act Articles 50, 51-55, 71, 72, 73
- EU DSA Articles 14-28, 33-43, 70-88
- EU Empowering Consumers (EU) 2024/825
- US Section 230 (47 USC 230)
- US COPPA + 16 CFR 312
- US DMCA 17 USC 512
- US California SB-1001
- UK AADC + UK Online Safety Act 2023
- KR PIPA + 청소년보호법 + 정보통신망법 + 저작권법
  + KCC + 방송통신심의위원회 + KISA + KrCERT-CC
- W3C WCAG 2.2 + ATAG 2.0 + UAAG 2.0 + EU EAA +
  EN 301 549
- US ADA Title III + Section 508
- NIST AI 600-1
- NCMEC CyberTipline for US-jurisdiction CSAM
  reporting
- INHOPE network for international child-protection
  hotlines

---

## §1 Upstream Generative-AI Model Provider Integration

The operator's content-platform integrates with the
generative-AI model provider whose model produced
the AI-generated content:

- The provider's model card and intended-use
  declaration (per the WIA-generative-ai standard)
  drives the content-platform's deployer-obligations
  under EU AI Act Article 26.
- The provider's serious-incident reporting under
  Article 73 (where applicable to the deployed
  model) is forwarded to the operator's trust-and-
  safety function for impact assessment.
- The provider's transparency obligations to deployers
  under Article 25 feed the operator's content-
  generation pipeline configuration.

## §2 C2PA Trust List and Verification Ecosystem
       Integration

The operator integrates with:

- The C2PA trust list — the operator's signing key
  is registered and published.
- Public verification widgets — third-party
  verification consumers (browsers, social-network
  widgets, fact-checking organisations) consult the
  trust list at verification time.
- ETSI TS 104 224 ecosystem-aligned verifiers — for
  content distributed to the news-and-broadcasting
  ecosystem the ETSI-published manifest format
  applies.
- IPTC Photo Metadata 2024 — for news imagery the
  operator emits the IPTC Digital Source Type
  controlled-vocabulary value (trainedAlgorithmicMedia,
  digitalCapture, etc.) alongside the C2PA manifest.

## §3 EU DSA Transparency Database (DSA-TDB)
       Integration

For EU-jurisdiction operators in DSA scope:

- Article 17 statement-of-reasons records are
  submitted to the DSA-TDB on the operator's
  published cadence (real-time for VLOPs / VLOSEs;
  on a documented periodic basis for other
  operators).
- The DSA-TDB's published schema and submission
  endpoint govern the integration.
- The operator's periodic Article 24 transparency
  reports are linked from the DSA-TDB and published
  on the operator's website.

## §4 EU DSC and Commission Integration

For EU-jurisdiction operators:

- The Member-State DSC is the operator's primary
  supervisory authority for DSA compliance.
- For VLOPs / VLOSEs the European Commission has
  exclusive enforcement powers under DSA Article 66.
- The Article 33 designation is communicated by the
  Commission; the operator's compliance officer
  notifies the Commission of any change in user
  count that affects designation.

## §5 Out-of-Court Dispute Settlement Body Integration

The operator integrates with the certified DSA
Article 21 OODS bodies in the operating jurisdictions:

- The operator publishes the list of OODS bodies the
  operator is willing to engage with.
- When an appeal escalates the operator forwards the
  case file to the OODS body and accepts the OODS
  body's binding determination.
- The fee discipline follows Article 21(5) — winning
  party fees are reimbursed by the losing party.

## §6 Trusted-Flagger Network Integration

For DSA Article 22 trusted-flagger integration:

- The operator processes notices from certified
  trusted flaggers with priority and deals with them
  expeditiously.
- The operator engages with the DSC's trusted-
  flagger registry to validate the certification of
  flaggers it accepts notices from.
- Suspicious-flagging behaviour is escalated to the
  DSC for review under Article 22(6).

## §7 Rights-Holder and Copyright Integration

For copyright-bearing content the operator
integrates with:

- The DMCA notice-channel for US-jurisdiction
  takedowns under 17 USC 512.
- The EU CDSM Directive Article 17 representative
  rights-holder licensing channel.
- The operator's automated content-recognition
  (ACR) system for known-rights-holder content.
- The KR 저작권법 cooperation channel.

## §8 Law-Enforcement Request Channel Integration

For DSA Article 9 / 10 law-enforcement integration:

- The operator's law-enforcement portal accepts
  removal orders and information-requests from
  Member-State authorities.
- The operator notifies the affected user (where
  permitted) of received orders.
- The operator publishes statistics on law-
  enforcement requests as part of the Article 24
  transparency report.

## §9 Supervisory Data-Protection and Sector
       Regulator Integration

The operator integrates with:

- The lead data-protection authority (GDPR Article
  56) for personal-data-processing oversight.
- The FTC for COPPA / CalOPPA / UDAAP enforcement
  (US).
- Ofcom for UK Online Safety Act enforcement.
- KCC / 방송통신심의위원회 for KR content-deliberation
  enforcement.
- KR PIPC for KR-jurisdiction personal-data
  oversight.
- KR 산업안전보건공단 in incidental scope where
  occupational-context AI content is in scope.

## §10 NCMEC and INHOPE Integration

For child-safety content:

- The operator reports CSAM (Child Sexual Abuse
  Material) to NCMEC's CyberTipline (US-
  jurisdiction).
- The operator participates in the INHOPE network
  for international child-protection hotline
  cooperation.
- For UK-jurisdiction operators the IWF (Internet
  Watch Foundation) is the integration counterparty.

## §11 Vetted-Researcher Data-Access Integration
       (DSA Article 40)

For VLOP / VLOSE operators the data-access
integration:

- The operator processes data-access requests from
  vetted researchers per Article 40.
- The DSC coordinates the vetted-researcher
  certification.
- The data is shared in a privacy-preserving way
  (typically aggregated, pseudonymised, or
  differential-privacy-applied).

## §12 External Audit and ISMS / AIMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022; the AIMS is certified against ISO/IEC
42001:2023. Certification scope explicitly extends
to the publication, moderation, transparency, and
trust-and-safety endpoints. The certification body
operates under ISO/IEC 17021-1; the conformity-
assessment body for WIA-content-ai operates under
ISO/IEC 17065. For VLOPs / VLOSEs the DSA Article
37 independent-audit firm is contracted under the
audit framework specified in Commission Delegated
Regulation (EU) 2023/2828.

## §13 Long-Term Archival Integration

Records governed by the operator's retention
horizons (DSA Article 24 transparency-report
retention; KR PIPA retention; the operator's
internal three-to-five-year retention) are migrated
to the long-term archive at the close of the active
retention window. The archive preserves the content
transcripts, the C2PA manifests, the moderation
decisions, the notice-and-action records, the
appeals records, the trusted-flagger records, the
transparency reports, the risk-assessment reports,
the independent-audit reports, and the audit-event
trail.

## §14 Public-Disclosure and Researcher-Access Surface

The operator's public-disclosure surface exposes:

- The Article 24 transparency report.
- The Article 34 risk-assessment summary
  (proportionate to the operator's risk-disclosure
  obligations).
- The Article 37 independent-audit report summary.
- The Article 27 recommender-system parameter
  documentation.
- The Article 26 ad-library where the operator runs
  advertising.

## §15 News-Industry and Fact-Checking Network
        Integration

For news-publisher operators and operators that
distribute news content:

- The IPTC NewsCodes controlled vocabularies are
  applied to news content classification.
- The integration with the IFCN (International Fact-
  Checking Network) signatories enables the
  operator's third-party fact-check labelling and
  associated content downranking.
- The IPTC Digital Source Type controlled
  vocabulary discloses the AI-source classification
  alongside the C2PA manifest.
- For coordinated-inauthentic-behaviour reporting
  the operator participates in the EDMO (European
  Digital Media Observatory) network where
  applicable.

## §16 Conformance

Implementations claiming PHASE-4 conformance maintain
the C2PA, DSA-TDB, DSC, OODS, trusted-flagger, and
rights-holder integrations, exercise the law-
enforcement and child-safety reporting obligations,
hold the ISO/IEC 42001 + ISO/IEC 27001 certifications,
participate in the NCMEC / INHOPE / IWF networks
where the threat surface warrants, exercise the
vetted-researcher data-access programme where the
operator is a VLOP / VLOSE, and operate the long-
term archival integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-content-ai
- **Last Updated:** 2026-04-28
