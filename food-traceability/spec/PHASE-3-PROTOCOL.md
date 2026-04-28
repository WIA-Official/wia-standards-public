# WIA-food-traceability PHASE 3 — PROTOCOL Specification

**Standard:** WIA-food-traceability
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
food-traceability operator: the Codex CAC/GL
60-2006 product-tracing-as-tool discipline; the
ISO 22005 traceability-system-design discipline; the
GS1 EPCIS 2.0 + CBV 2.0 + GTS 2.0 critical-tracking-
event discipline; the FSMA 204 KDE discipline for
the Food Traceability List commodities; the EU Reg
178/2002 Article 18 one-step-back / one-step-forward
discipline; the GFSI-recognised-scheme + ISO 22000
food-safety-management discipline; the recall-and-
withdrawal discipline; the allergen-and-labelling
discipline (EU Reg 1169/2011 + FDA 21 CFR 101 + KR
표시·광고법); the foreign-supplier-verification
discipline (US FSMA FSVP); and the supervisory and
recall-coordination discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015, ISO/IEC 27001:2022, ISO 22000:2018
- ISO/TS 22002 series, ISO 22005:2007
- Codex Alimentarius CAC/GL 60-2006 + CAC/RCP
  1-1969 + CAC/GL 81-2013
- GS1 EPCIS 2.0 + CBV 2.0 + GTS 2.0 + GDM + Digital
  Link 1.4
- GFSI Benchmarking Requirements + recognised
  schemes (BRCGS Food 9, IFS Food 8, FSSC 22000 v6,
  SQF v9, PrimusGFS)
- EU Reg (EC) 178/2002 General Food Law Art 18 +
  Reg (EU) 1169/2011 FIC + Reg (EU) 931/2011 + Reg
  (EC) 852/2004 + Reg (EC) 853/2004 + Reg (EU)
  2017/625 + Reg (EU) 2017/2470 + Reg (EC)
  1924/2006
- US FDA FSMA Section 204 + 21 CFR Part 1 Subpart
  S + Subpart L FSVP + Subpart J Bioterrorism
- US FDA 21 CFR Part 101 (food labelling)
- US USDA COOL 7 CFR Part 65 + Part 60
- US FDA Reportable Food Registry (RFR)
- KR 식품위생법 + 식품안전기본법 + 농수산물품질관리법
  + 식품 등의 표시·광고에 관한 법률 + 식품 등의 이물
  발견 보고 등에 관한 규정 + MFDS 식품안전나라 + KR
  RASFF-K 식품안전관리 시스템
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)

---

## §1 Codex CAC/GL 60-2006 Discipline

The Codex Alimentarius CAC/GL 60 discipline:

- Traceability / product tracing applied as a tool
  within food inspection and certification systems.
- The objectives of traceability — facilitating
  recalls, supporting consumer information,
  protecting consumers from unsafe products,
  facilitating verification.
- The principles — the system is fit for purpose,
  designed in accordance with risk, technically
  feasible, economically viable, transparent in
  operation.

## §2 ISO 22005 Traceability-System-Design Discipline

The ISO 22005:2007 discipline:

- Identification of products and product lots.
- Identification of involved parties.
- Identification of the lifecycle steps each
  product/lot has been through.
- Documentation of the linkages between these
  identifiers.
- Documentation of the procedures for
  identification, recording, transmission, and
  retrieval.

## §3 GS1 EPCIS 2.0 + CBV 2.0 Discipline

The EPCIS 2.0 + CBV 2.0 discipline:

- Five W's framing — what (EPC), when (eventTime),
  where (readPoint / bizLocation), why (bizStep /
  disposition), how (event-specific extensions).
- Five EPCIS event types — ObjectEvent (commissioning,
  observing, decommissioning), AggregationEvent
  (cases on pallets, items in cases), TransactionEvent
  (associating EPCs with business transactions),
  TransformationEvent (transforming inputs into
  outputs — e.g. milling wheat to flour),
  AssociationEvent (associating sub-objects with
  parent objects).
- Capture-and-query interface — capture exposes
  the per-event JSON-LD payload; query exposes
  filter-based retrieval.
- Subscription — push-style notification of events
  matching subscriber-defined criteria.

## §4 FSMA Section 204 KDE Discipline

For US-jurisdiction operators handling Food
Traceability List (FTL) commodities the FSMA 204
discipline:

- Critical Tracking Events (CTEs) per 21 CFR
  1.1305 — harvesting, cooling, initial-packing,
  first-land-based-receiver, shipping, receiving,
  transformation.
- Per-CTE Key Data Elements per 21 CFR 1.1330 to
  1.1340.
- Traceability Lot Code (TLC) — per 21 CFR 1.1320
  the TLC is assigned at growing, initial packing,
  first land-based receipt, or transformation.
- TLC Source — the entity that originated the
  lot code.
- 24-hour FDA-request response — operators provide
  electronic, sortable spreadsheet of records to
  FDA upon request within 24 hours.
- Compliance date — 20 January 2026 originally, with
  FDA proposing extension; the operator's
  compliance plan tracks the applicable date.

## §5 EU Reg 178/2002 Art 18 One-Up / One-Down
       Discipline

For EU-jurisdiction operators:

- Article 18(1) — the operator establishes the
  traceability of food at all stages of production,
  processing, and distribution.
- Article 18(2) — operators identify any person
  from whom they have been supplied with a food /
  feed / food-producing-animal / substance intended
  to be incorporated into a food / feed (one-step-
  back).
- Article 18(3) — operators identify the businesses
  to which their products have been supplied (one-
  step-forward); the requirement does not apply to
  final consumers.
- Article 18(5) — information available to
  authorities upon request.
- Reg (EU) 931/2011 imposes additional traceability
  for food of animal origin.

## §6 KR 식품위생법 추적 Discipline

For KR-jurisdiction operators:

- 식품위생법 Article 49 (식품 이력 추적관리 시스템)
  — selected food categories required to participate
  in the food-history traceability system operated
  by MFDS.
- 농수산물품질관리법 (Agricultural and Marine Products
  Quality Management Act) — agricultural and marine
  products traceability system.
- 식품안전기본법 — recall, withdrawal, and supervisory
  cooperation.
- 식품 등의 이물 발견 보고 등에 관한 규정 — foreign-
  matter detection reporting.

## §7 GFSI-Recognised Scheme Discipline

The GFSI Benchmarking Requirements + recognised
schemes:

- BRCGS Food Safety Issue 9 — site-level
  certification covering HACCP, food-safety-
  management, site standards, product control,
  process control, personnel.
- IFS Food version 8 — German / French retailer-
  driven scheme covering similar scope.
- FSSC 22000 v6.0 — ISO 22000 + ISO/TS 22002 + FSSC
  additional requirements.
- SQF Food Safety Code v9 — North American retailer-
  driven scheme.
- PrimusGFS — fresh produce-focused scheme.

## §8 Recall-and-Withdrawal Discipline

The recall-and-withdrawal discipline:

- US FDA — Class I / II / III recall classes per
  21 CFR Part 7. The operator's recall is
  voluntary unless FDA mandates per FSMA Section
  206 + FDCA 423.
- EU RASFF (Rapid Alert System for Food and Feed)
  — Member-State authority issues the notification;
  the EU-level RASFF window distributes to all
  Member-States.
- KR MFDS 회수·폐기 명령 — MFDS issues the recall /
  destruction order.
- Consumer-facing notice — the operator publishes
  the recall through its website, retailer
  channels, social media, and (for high-risk
  recalls) news media.

## §9 Allergen and Labelling Discipline

The allergen-and-labelling discipline:

- EU Reg 1169/2011 Annex II 14 allergens list —
  cereals containing gluten, crustaceans, eggs,
  fish, peanuts, soybeans, milk (including lactose),
  nuts, celery, mustard, sesame, sulphur-dioxide
  and sulphites, lupin, molluscs.
- US FDA major-food-allergens — 9 (the FALCPA 8
  plus sesame added per FASTER Act of 2021).
- KR 식품 등의 표시·광고법 — KR allergen labelling
  list (계란, 우유, 메밀, 땅콩, 대두, 밀, 고등어,
  게, 새우, 돼지고기, 복숭아, 토마토, 아황산류,
  호두, 닭고기, 쇠고기, 오징어, 조개류, 잣).
- Country-of-origin labelling — US COOL + EU Reg
  1169/2011 Article 9(1)(i) + KR 농수산물의 원산지
  표시에 관한 법률.
- Nutrition labelling — FDA 21 CFR 101.9 + EU Reg
  1169/2011 Annex XV + KR 식품등의 표시기준.

## §10 FSVP Discipline (US FSMA)

For US-importer operators the Foreign Supplier
Verification Program per 21 CFR Part 1 Subpart L:

- Hazard analysis of each foreign supplier's food.
- Evaluation of the foreign supplier's performance
  and food risk.
- Approval of the foreign supplier.
- Establishing and following written procedures.
- Verification activities (onsite audit, sampling
  and testing, review of supplier's food-safety
  records).

## §11 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every EPCIS
capture, KDE record, supply-chain-partner update,
FSMS record, recall step, and consumer-disclosure
amendment.

## §12 Sampling-and-Testing Discipline

The operator's sampling-and-testing discipline:

- Risk-based sampling plan covering microbiological
  hazards (Salmonella, Listeria monocytogenes,
  Campylobacter, E. coli O157:H7), chemical
  hazards (heavy metals, pesticide residues,
  mycotoxins), and physical hazards.
- ISO/IEC 17025 accredited laboratories selected
  for the analytical work.
- Sample chain-of-custody documented in EPCIS
  events.
- Out-of-specification results trigger the
  operator's release-or-hold decision under the
  HACCP plan.
- Trend monitoring across multiple sampling
  campaigns identifies process drift.

## §13 Cross-Border Trade Discipline

For exports / imports:

- US FDA Prior Notice (21 CFR 1.276 to 1.285) for
  imported food shipments.
- EU Reg (EU) 2017/625 official-controls plus the
  TRACES NT veterinary-and-phytosanitary
  certification system.
- KR 식품의 수입신고 system through KR MFDS for
  imported food.
- Codex Alimentarius export certification per
  CAC/GL 38-2001.
- Phytosanitary certification per IPPC ISPM 12.

## §14 Foreign-Material and Adulteration Discipline

The foreign-material and adulteration discipline:

- Metal-detection / X-ray inspection at processing
  CCPs.
- Glass / plastic / wood / stone foreign-material
  detection with documented control of the source.
- Adulteration prevention — economically motivated
  adulteration (EMA) under FDA FSMA Section 106 +
  21 CFR Part 121 (Mitigation Strategies to Protect
  Food Against Intentional Adulteration).
- Food fraud vulnerability assessment per the
  GFSI-recognised scheme (BRCGS clause 5.4 / FSSC
  22000 V6 PRP / SQF clause 2.7).
- Food defense plans where the operator is in
  scope of FSMA IA.

## §15 Date-Marking and Shelf-Life Discipline

The date-marking discipline:

- "Use by" date for highly-perishable products
  (microbiological food-safety risk).
- "Best before" / "Best before end" date for
  shelf-stable products (quality, not safety).
- KR 식품 등의 표시·광고법 — 유통기한 / 소비기한 /
  품질유지기한 distinction (소비기한 mandatory in
  KR from 2023).
- Shelf-life validation per the operator's challenge
  studies and predictive microbiology models
  (ComBase, Pathogen Modeling Program).
- Date code embedded in the FSMA 204 traceability
  lot code where possible.

## §16 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
satisfy the Codex CAC/GL 60 + ISO 22005 + EPCIS 2.0
+ CBV 2.0 baseline, exercise the FSMA 204 KDE
discipline (US) + EU Reg 178/2002 Art 18 one-up /
one-down + KR 식품위생법 추적 + GFSI-scheme
discipline applicable to the operator, and exercise
the recall-and-withdrawal discipline integrated with
the operating jurisdiction's recall channel.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-food-traceability
- **Last Updated:** 2026-04-28
