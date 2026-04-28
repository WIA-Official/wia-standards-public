# WIA-food-traceability PHASE 4 — INTEGRATION Specification

**Standard:** WIA-food-traceability
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a food-traceability
operator integrates with the systems that surround
the food-supply-chain lifecycle: the GS1 EPCIS 2.0
event-distribution network across supply-chain
partners; the GS1 Digital Link and Verified by GS1
brand-owner registry; the GFSI-recognised
certification body; the FDA / USDA / EFSA / Member-
State competent authorities and the KR 식약처 +
식품의약품안전평가원; the FDA Reportable Food
Registry (RFR); the EU RASFF (Rapid Alert System
for Food and Feed); the WHO INFOSAN (International
Food Safety Authorities Network); the operator's
laboratory-and-testing partners; the consumer-facing
SmartLabel and recall-information channels; the
external auditor and ISO 22000 + ISO/IEC 27001 +
GFSI-scheme certification body; and the long-term
archive that preserves food-supply-chain records
past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- Codex Alimentarius CAC/GL 60-2006 + CAC/RCP
  1-1969
- ISO 22000:2018, ISO/TS 22002 series, ISO 22005:2007
- GS1 EPCIS 2.0 + CBV 2.0 + GTS 2.0 + GDM + Digital
  Link + SmartLabel + Verified by GS1
- GFSI Benchmarking Requirements + BRCGS Food 9 +
  IFS Food 8 + FSSC 22000 v6 + SQF v9
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015, ISO/IEC
  17065:2012
- ISO 8601
- W3C Verifiable Credentials Data Model 2.0
- US FDA FSMA Section 204 + 21 CFR Part 1 Subpart
  S + Subpart L FSVP + Subpart J Bioterrorism + 21
  CFR Part 7 recalls + 21 CFR Part 101 + Part 117
  preventive controls
- US FDA RFR + US USDA FSIS + COOL 7 CFR 65 / 60
- EU Reg (EC) 178/2002 + Reg (EU) 1169/2011 + Reg
  (EU) 931/2011 + Reg (EU) 2017/625 + Reg (EU)
  2017/2470
- WHO INFOSAN
- EU RASFF (Rapid Alert System for Food and Feed)
- KR 식약처 (MFDS) + 식품의약품안전평가원 + 식품
  안전나라 + KR 식품위생법 + 식품안전기본법 + KR
  RASFF-K

---

## §1 GS1 EPCIS Event-Distribution Integration

The supply-chain-partner integration:

- Per-partner EPCIS subscription registration —
  partners subscribe to events matching the
  GTIN / lot scope they require visibility on.
- Per-partner authentication via the GS1 EPCIS 2.0
  OAuth profile with per-event-class scopes.
- Subscription delivery via webhook (push) or polling
  (pull) per the partner's preference.
- Cross-organisation event correlation — the
  operator's events are correlated with the
  upstream and downstream events to form the
  end-to-end traceability graph.

## §2 GS1 Digital Link and Verified by GS1
       Integration

For consumer-facing product disclosure:

- Brand owners register the operator's GS1 Digital
  Link domain with the Verified by GS1 service.
- Per-GTIN linkType-keyed resolver entries point
  to product information, recipe, allergen,
  nutrition, recall information, sustainability,
  and consumer-engagement assets.
- The Verified by GS1 service enables consumer
  apps to verify the brand-owner identity for the
  scanned GTIN.

## §3 GFSI Certification Body Integration

The operator's GFSI-recognised-scheme integration:

- Per-scheme certification body operating under
  ISO/IEC 17065 + ISO 22003 (food-safety-management
  certification bodies).
- Scheme-specific audit cadence (typically annual
  with surveillance audits in between).
- Per-scheme corrective-action workflow integrated
  with the operator's FSMS record.

## §4 FDA / USDA / EFSA / KR MFDS Integration

For US-jurisdiction operators:

- US FDA — FSMA Section 204 24-hour records request,
  FSVP records request, RFR submission, recall
  coordination through the recall coordinator.
- US USDA FSIS — for meat / poultry / processed-
  egg products under the Federal Meat Inspection
  Act + Poultry Products Inspection Act + Egg
  Products Inspection Act.

For EU-jurisdiction operators:

- EU EFSA + Member-State competent authorities
  for official-controls (Reg (EU) 2017/625).
- EU RASFF for cross-Member-State alert
  distribution.
- DG SANTE for EU-level food-safety policy.

For KR-jurisdiction operators:

- KR 식약처 (MFDS) + 식품의약품안전평가원 — official
  controls under 식품위생법.
- KR 식품안전나라 + RASFF-K — KR-equivalent rapid
  alert system.
- KR 농림축산식품부 (MAFRA) — for agricultural and
  livestock products.
- KR 해양수산부 (MOF) — for aquatic products.

## §5 WHO INFOSAN Integration

For international food-safety alerts the operator
participates in WHO INFOSAN through the Member-State
focal point — international notification of food-
safety events, cross-border recall coordination, and
public-health risk communication.

## §6 EU RASFF and KR RASFF-K Integration

The rapid-alert-system integration:

- EU RASFF — Member-State CP receives the
  notification, triages, and forwards to the EC.
- KR RASFF-K — KR MFDS operates the equivalent
  channel under 식품안전기본법.
- Cross-system referrals between RASFF and INFOSAN
  for international alerts.

## §7 Laboratory-and-Testing Partner Integration

The laboratory integration:

- Per-test-sample EPCIS event captured at sample
  draw.
- Per-result test report ingested into the
  operator's FSMS.
- ISO/IEC 17025 accredited laboratory selection.
- For UK-jurisdiction Public Analyst services
  cooperation.

## §8 Consumer-Facing Channel Integration

The consumer integration:

- GS1 SmartLabel and Powered by GS1 Digital Link
  resolver for product information, allergen
  details, nutrition, recipe, and sustainability
  attestations.
- Recall notification through the operator's
  website, retailer channels, mobile-app push, and
  high-risk media coverage.
- Allergen-and-dietary-preference search support
  for consumer-facing apps.

## §9 External Audit and Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the scope explicitly extending to
the EPCIS, KDE, and recall-coordination endpoints.
The food-safety-management system is certified
against ISO 22000:2018 + the operator's chosen
GFSI-recognised scheme. The certification body
operates under ISO/IEC 17021-1; the conformity-
assessment body for WIA-food-traceability operates
under ISO/IEC 17065. ISO/IEC 17025 accreditation
is held by the operator's testing laboratory.

## §10 Long-Term Archival Integration

Records governed by the operator's retention
horizons (US FDA FSMA 204 two-year retention; US
Bioterrorism Act one-up-one-down for two years;
US FDA Part 117 preventive-controls records two-
year retention; EU Reg 178/2002 traceability
without specific retention; KR 식품위생법 5-year
retention) are migrated to the long-term archive
at the close of the active retention window. The
archive preserves the GS1 product / location /
logistic-unit registry, the EPCIS event store, the
KDE records, the supply-chain-partner records, the
FSMS records, the recall records, and the audit-
event trail.

## §11 Climate-Disclosure and Sustainability
        Integration

For ESG-disclosure-subject operators:

- Carbon-footprint-of-product calculations per
  ISO 14067 + Smart Freight Centre GLEC for
  logistics emissions.
- WRI / WBCSD GHG Protocol Product Standard for
  product-level CO2.
- EU CSRD ESRS E5 (resource use and circular
  economy) + EU Empowering Consumers (EU)
  2024/825 for food-product green-claims.
- Operator's WIA-esg-finance disclosure record
  integrates the food-supply-chain attribution.

## §12 EU Battery Regulation and Packaging Regulation
        Integration

For food-package-and-container operators:

- EU PPWR (Packaging and Packaging Waste Regulation,
  Reg (EU) 2025/40) — packaging traceability
  obligations.
- EU Single-Use Plastics Directive (Directive (EU)
  2019/904).
- US state-level extended-producer-responsibility
  (EPR) packaging regimes.
- KR 자원의 절약과 재활용촉진에 관한 법률 (Resources
  Recycling Promotion Act).

## §13 Cold-Chain and Logistics Integration

For temperature-sensitive food:

- Cold-chain monitoring per ISO 22002-5 + ATP
  Agreement (Agreement on the International
  Carriage of Perishable Foodstuffs).
- Continuous temperature logging captured as EPCIS
  events with the time-series referenced from the
  ILMD.
- Excursion alerts to shipper / consignee for
  out-of-specification temperature events.
- Product-specific cold-chain SLAs (frozen,
  refrigerated, ambient).

## §14 Counterfeit / Anti-Tampering Integration

For brand-protection:

- Per-product authentication via GS1 Verified by
  GS1 + GS1 Digital Link.
- Tamper-evident packaging with embedded
  cryptographic signatures (GS1 Reseller
  Identifier, EPC + chip-level authentication).
- Consumer-app verification scan against the
  brand-owner registry.
- Coordination with INTERPOL Operation OPSON for
  counterfeit-food enforcement.

## §15 Animal Health and Veterinary Integration

For meat / poultry / fish / egg products:

- ICVCP (International Committee on Veterinary
  Certificates and Procedures) certification
  exchange via TRACES NT.
- WOAH (World Organisation for Animal Health,
  formerly OIE) Terrestrial Animal Health Code +
  Aquatic Animal Health Code compliance.
- US USDA FSIS export-eligibility list integration.
- KR 검역본부 + 동물검역검사관 시스템 for KR-
  jurisdiction animal-product imports / exports.

## §16 Allergen Cross-Contamination Integration

For allergen management across co-manufacturing
sites:

- Per-line allergen profile recorded; line-
  changeover sanitation verified before next
  production.
- "Contains" / "May contain" / "Free from"
  declarations cross-referenced with the actual
  formulation and the supplier-attested allergen
  status.
- VITAL (Voluntary Incidental Trace Allergen
  Labelling) reference levels for "may-contain"
  decisions.

## §17 Conformance

Implementations claiming PHASE-4 conformance maintain
the supply-chain-partner EPCIS integration, exercise
the recall-coordination integration with the
operating jurisdiction's recall channel, integrate
with INFOSAN / RASFF / RASFF-K where the operator's
products travel cross-border, hold the ISO 22000 +
ISO/IEC 27001 + GFSI-scheme certifications, and
operate the long-term archival integration described
above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-food-traceability
- **Last Updated:** 2026-04-28
