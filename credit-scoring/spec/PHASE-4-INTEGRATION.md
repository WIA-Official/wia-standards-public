# WIA-credit-scoring PHASE 4 — INTEGRATION Specification

**Standard:** WIA-credit-scoring
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a credit-scoring operator
integrates with the systems that surround the credit-
decision lifecycle: the credit-bureau-supplied data
feed; the lender's loan-origination, loan-servicing,
and collections systems; the consumer-facing portal
and the consumer's right-to-explanation surface; the
supervisory authority for the operating jurisdiction;
the external auditor and the model-risk-validation
function; the EU AI Act conformity-assessment body
(for high-risk credit-scoring AI systems placed on
the EU market); and the long-term archive that
preserves model artefacts past the retention horizon.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 17021-1:2015 (audit and certification)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional
  for re-issuance of attestations)
- US ECOA + Reg B, FCRA + Reg V, FHA, TILA + Reg Z
- US CFPB Examination Manual
- EU CCD recast (Directive (EU) 2023/2225)
- EU Mortgage Credit Directive (Directive 2014/17/EU)
- EU AI Act 2024 (Regulation (EU) 2024/1689)
- EU GDPR Articles 9, 15, 20, 22, 33, 34, 46
- KR Credit Information Use and Protection Act
- KR 금융소비자보호법
- Basel Committee on Banking Supervision sound-
  practices guidance on model risk governance
- NIST AI RMF 1.0

---

## §1 Credit-Bureau Integration

The operator integrates with the credit-bureaux that
supply tradeline information. The integration carries:

- The bureau's identity and the bureau's contact
  information (carried into the adverse-action notice
  per FCRA 15 USC 1681m(a)(3)).
- The permissible-purpose certification per FCRA 15
  USC 1681e(a) — the operator certifies the
  permissible purpose for each inquiry.
- The bureau's API endpoint, authentication
  credential, and the wire-format declaration (the
  bureaux publish proprietary specifications; this
  standard prescribes the operator-side discipline,
  not the bureau-side wire format).
- The dispute relay channel — when the consumer
  disputes a tradeline through the operator's
  channel the operator forwards the dispute to the
  bureau under FCRA 15 USC 1681i(a).
- The reciprocity / furnisher feed — when the
  operator furnishes tradeline data back to the
  bureau the operator's furnisher discipline (FCRA
  15 USC 1681s-2 and Regulation V 12 CFR 1022.40 to
  1022.43) applies.

## §2 Loan-Origination System Integration

The operator's scoring pipeline integrates with the
lender's loan-origination system (LOS):

- The LOS produces the credit application; the
  operator records the application identifier and
  the consumer's identity binding.
- The operator returns the score and the
  creditworthiness-assessment outcome; the LOS
  applies the operator's policy decision to the
  application's lifecycle (approved, declined,
  counter-offer, manual review).
- The adverse-action notice is produced by the LOS
  using the operator's principal-reasons and bureau-
  reference fields.
- The decision and the supporting artefacts (the
  signed score record, the feature vector, the model
  reference) are preserved for the operating
  retention horizon.

## §3 Loan-Servicing and Collections Integration

For active-account review (FCRA 15 USC 1681b(a)(3)(B))
and collections:

- The servicing system periodically requests refreshed
  scores; the permissible purpose is the active-
  account review.
- For accounts that enter delinquency the collections
  system uses the score to triage outreach intensity
  (consistent with the operator's compliance
  obligations under the operating jurisdiction's
  consumer-credit collection statutes).
- For accounts that enter charge-off or settlement
  the operator updates the bureau-furnished tradeline
  per the operator's furnisher-duty discipline
  (Regulation V 12 CFR 1022.41 accuracy and integrity
  guidelines).

## §4 Consumer-Portal Integration

The consumer-portal exposes:

- The consumer's free credit-report copy under FCRA
  15 USC 1681j(a) where the operator is a covered
  entity.
- The consumer's adverse-action notices.
- The consumer's score history (where the operator
  is a credit-bureau or scoring provider that
  delivers a consumer-facing score subscription).
- The dispute channel for tradeline inaccuracies.
- The Article 22(3) right-to-explanation surface for
  GDPR-regulated operators.
- The KR Credit Information Use and Protection Act
  rejection-reasoning explanation surface for KR-
  regulated operators.

## §5 Supervisory-Authority Integration

For US-regulated operators the supervisory-authority
counterparty is the CFPB (consumer-credit
examinations), the operator's prudential regulator
(FRB, OCC, FDIC, NCUA — depending on charter), the
state attorney-general (state-law UDAAP and consumer-
protection enforcement), and the Federal Trade
Commission (non-bank entities). For EU-regulated
operators the counterparty is the Member-State
financial-supervisory authority and the lead data-
protection authority. For KR-regulated operators the
counterparty is the Financial Services Commission
(FSC), the Financial Supervisory Service (FSS), and
the Personal Information Protection Commission
(PIPC).

## §6 EU AI Act Conformity-Assessment Integration

For high-risk credit-scoring AI systems placed on the
EU market the operator integrates with:

- The Member-State notifying authority and the
  notified body that performs the conformity
  assessment under EU AI Act Article 43 where third-
  party assessment is required (Annex III §5(b) is
  high-risk; the standard internal-control conformity-
  assessment procedure of Annex VI applies for most
  credit-scoring systems).
- The EU database for high-risk AI systems under
  Article 71.
- The post-market-monitoring channel under Article 72
  for collecting field performance data.
- The serious-incident reporting channel under
  Article 73.
- The market-surveillance authority under Article 74.

## §7 External Audit and Model-Validation Integration

The operator's model-validation function operates
organisationally separate from model development per
Basel sound-practices for model risk governance. The
ISMS is certified against ISO/IEC 27001:2022 and the
AI management system against ISO/IEC 42001:2023.
The certification body operates under ISO/IEC 17021-1;
the conformity-assessment body operates under
ISO/IEC 17065. External-audit findings feed the
operator's remediation cycle on the operator's
declared cadence.

## §8 Cross-Border Data-Transfer Integration

For cross-border transfers of credit information the
operator's discipline applies the destination
jurisdiction's regulatory regime — GDPR Chapter V
(Article 45 adequacy, Article 46 SCC / BCR, Article
49 derogation) for EU-source transfers; KR Credit
Information Use and Protection Act Article 32-2
(third-party provision of personal credit
information) and KR PIPA Article 28-8 (cross-border
transfer) for KR-source transfers; the operator's
business-associate-agreement equivalent for US-
source transfers where consumer-credit data is
transferred to a service provider.

## §9 Long-Term Archival Integration

Records governed by the operator's retention horizons
(US ECOA Reg B 12 CFR 1002.12 retention discipline
twenty-five months; FCRA recordkeeping; EU GDPR
Article 5(1)(e) no-longer-than-necessary; KR Credit
Information Use and Protection Act Article 20-2
retention discipline) are migrated to the long-term
archive at the close of the active retention window.
The archive preserves the model registry (training
data manifest, validation report, model version),
the score records, the adverse-action notices, the
audit-event trail, and the dispute correspondence.

## §10 Open-Banking and Alternative-Data Integration

Where the operator uses open-banking-supplied or
alternative-data-supplied features (rent payments,
utility payments, telecom payments, payroll-direct-
deposit signals) the integration carries:

- The data-provider's identity and the consumer's
  explicit consent to the operator's collection and
  use of the data (PSD2 access-to-account in the EU,
  US CFPB §1033 personal financial data rights, KR
  마이데이터 서비스).
- The data-provider's API endpoint, authentication
  credential, and the wire-format declaration.
- The provenance record — the alternative-data signal
  is preserved alongside the consumer's feature
  vector so that disputes referencing alternative
  data can be traced to the source.

Alternative-data features pass through the operator's
fair-lending review (PHASE-3 §4) and the model-risk
discipline (PHASE-3 §3) before they enter production.

## §11 Identity-Verification and Anti-Fraud Integration

The operator integrates with the lender's identity-
verification (IDV) and anti-fraud systems:

- IDV checks confirm the applicant's identity against
  jurisdictional identity-document verification
  (US KYC, EU eIDAS-aligned IDV, KR 신원확인);
  failed IDV stops the credit-application before
  scoring.
- Anti-fraud signals (synthetic-identity flags,
  velocity flags, device-fingerprint flags) are
  surfaced to the operator's manual-review queue;
  fraud denials are reported to the bureau-furnished
  tradeline only where FCRA permissible-purpose
  conditions are met.

## §12 Conformance

Implementations claiming PHASE-4 conformance maintain
the credit-bureau integration with the FCRA
permissible-purpose discipline, integrate the LOS
and servicing systems with the operator's policy-
decision point, expose the consumer-portal surface
required by the operating jurisdiction's right-to-
explanation discipline, hold the EU AI Act
conformity-assessment record (where the operator
places a high-risk system on the EU market), and
operate the external-audit and supervisory-authority
integrations described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-credit-scoring
- **Last Updated:** 2026-04-28
