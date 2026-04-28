# WIA-food-traceability PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-food-traceability
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-food-traceability. The standard covers
persistent record shapes for the lifecycle of a food
traceability operator — the food-business identity
and its traceability scope; the GS1 GTIN / GLN /
SSCC product-and-location identifier record; the
EPCIS 2.0 critical-tracking-event (CTE) record at
each phase of the food chain (production, processing,
packing, distribution, retail, food-service); the
key-data-element (KDE) attribute record; the supply-
chain-partner record; the food-safety-management
record (HACCP / GFSI scheme attestation); the
recall-and-withdrawal record; the consumer-facing
disclosure record (allergens, country of origin,
nutrition labelling); and the supervisory-and-audit
correspondence record. Records are consumed by the
food-business itself, the supply-chain partners
(producer, processor, distributor, retailer,
food-service), the operating jurisdiction's food-
safety authority (US FDA + USDA + EU EFSA + KR
식약처), the GFSI-recognised certification body, the
consumer-facing channels, and the supervisory and
recall-management correspondents.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO 22000:2018 (food safety management systems —
  requirements for any organization in the food
  chain)
- ISO/TS 22002 series (prerequisite programmes for
  food safety — Part 1 food manufacturing, Part 2
  catering, Part 3 farming, Part 4 food packaging
  manufacturing, Part 5 transport and storage, Part
  6 feed production)
- ISO 22005:2007 (Traceability in the feed and food
  chain — General principles and basic requirements
  for system design and implementation)
- ISO 22300:2021 (Security and resilience —
  Vocabulary)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- Codex Alimentarius CAC/GL 60-2006 (Principles for
  Traceability / Product Tracing as a Tool Within a
  Food Inspection and Certification System)
- Codex Alimentarius CAC/RCP 1-1969 (General
  Principles of Food Hygiene, including HACCP)
- Codex Alimentarius CAC/GL 81-2013 (Risk
  Assessment of Microbiological Hazards in Foods)
- GS1 General Specifications (the GS1 system of
  global identification standards — GTIN, GLN,
  SSCC, GIAI, SGTIN)
- GS1 EPCIS 2.0 (Electronic Product Code Information
  Services — REST + JSON-LD)
- GS1 Core Business Vocabulary 2.0 (CBV) — the
  controlled-vocabulary for EPCIS event-data
- GS1 Global Traceability Standard (GTS) 2.0
- GS1 Global Data Model (GDM) for product
  attributes
- GS1 SmartLabel + Powered by GS1 Digital Link
- Global Food Safety Initiative (GFSI) recognised
  schemes — BRCGS Food, IFS Food, FSSC 22000, SQF
- BRCGS Global Standard Food Safety Issue 9
- IFS Food version 8
- FSSC 22000 v6.0
- SQF Food Safety Code v9
- EU Regulation (EC) 178/2002 (General Food Law) —
  Article 18 traceability obligations
- EU Reg (EU) 1169/2011 (Food Information to
  Consumers — FIC)
- EU Reg (EU) 931/2011 (traceability requirements
  for food of animal origin)
- EU Reg (EC) 852/2004 (food hygiene)
- EU Reg (EC) 853/2004 (specific hygiene rules for
  food of animal origin)
- EU Reg (EC) 854/2004 + Reg (EU) 2017/625 (official
  controls)
- EU Reg (EU) 2017/2470 (novel-food list)
- EU Reg (EC) 1924/2006 (nutrition and health
  claims)
- US FDA FSMA Section 204 (Food Traceability Final
  Rule, 21 CFR Part 1, Subpart S — the Food
  Traceability List + Critical Tracking Events +
  Key Data Elements)
- US FDA FSVP (Foreign Supplier Verification
  Program) under FSMA + 21 CFR Part 1 Subpart L
- US FDA Reportable Food Registry (RFR)
- US USDA Country of Origin Labeling (COOL) under
  7 CFR Part 65 + Part 60
- US Bioterrorism Act 2002 + 21 CFR Part 1 Subpart
  J (records)
- KR 식품위생법 + KR 식품안전기본법 + KR 농수산물
  품질관리법 + KR 식품 등의 표시·광고에 관한 법률
  + KR 식품 등의 이물 발견 보고 등에 관한 규정 +
  KR 식품의약품안전처 (MFDS) 식품안전나라

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts a food-traceability operator (a primary
producer / farmer, a processor, a manufacturer, a
packer, a distributor, a wholesaler, a retailer, a
food-service operator, an importer / exporter)
maintains:

- The food-business identification record.
- The GS1 GTIN / GLN / SSCC inventory record.
- The EPCIS 2.0 critical-tracking-event record.
- The key-data-element record.
- The supply-chain-partner record.
- The food-safety-management record.
- The recall-and-withdrawal record.
- The consumer-facing disclosure record.
- The supervisory-correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("primary-producer-
                       farmer" | "primary-producer-
                       fisherman" | "processor" |
                       "manufacturer" | "packer" |
                       "distributor-wholesaler" |
                       "retailer" | "food-service-
                       restaurant" | "food-service-
                       institutional" | "importer"
                       | "exporter" | "carrier" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
gs1CompanyPrefix     : string (the operator's GS1
                       company prefix — the leading
                       digits common to GTIN / GLN
                       / SSCC issued by the
                       operator)
governingFrameworks  : array of enum ("ISO-22000-
                       2018" | "ISO-TS-22002-1-2-3-
                       4-5-6" | "ISO-22005-2007" |
                       "CAC-GL-60-2006" |
                       "CAC-RCP-1-1969" |
                       "GS1-EPCIS-2-0" |
                       "GS1-CBV-2-0" |
                       "GS1-GTS-2-0" |
                       "GS1-GDM" |
                       "BRCGS-FOOD-9" |
                       "IFS-FOOD-8" |
                       "FSSC-22000-V6" |
                       "SQF-V9" |
                       "EU-REG-EC-178-2002-ART-18"
                       | "EU-REG-EU-1169-2011-FIC"
                       | "EU-REG-EU-931-2011" |
                       "EU-REG-EU-2017-625" |
                       "US-FSMA-204-21-CFR-1-S" |
                       "US-FSVP-21-CFR-1-L" |
                       "US-BIOTERRORISM-21-CFR-1-J"
                       | "US-COOL-7-CFR-65" |
                       "KR-식품위생법" |
                       "KR-식품안전기본법" |
                       "KR-식품 등의 표시·광고법" |
                       "user-defined")
gfsiScheme           : enum ("brcgs-food" |
                       "ifs-food" | "fssc-22000" |
                       "sqf" | "primusgfs" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 GS1 GTIN / GLN / SSCC Record

```
gs1ProductRecord:
  productId          : string (uuidv7)
  gtin               : string (GS1 GTIN-14)
  productName        : string
  netContent         : object (UoM-quantified;
                       units per ISO 80000-1)
  brandOwner         : string (the brand-owner's
                       GLN)
  globalProductCategory : string (the GS1 GPC code)
  productLifecycleStatus : enum ("active" |
                       "discontinued" | "withdrawn")

gs1LocationRecord:
  locationId         : string (uuidv7)
  gln                : string (GS1 GLN)
  locationKind       : enum ("farm" | "fishing-
                       vessel" | "processor-plant"
                       | "manufacturing-plant" |
                       "packing-house" | "warehouse"
                       | "distribution-centre" |
                       "retail-store" | "food-
                       service-outlet" | "user-
                       defined")
  legalEntityRef     : string (the GLN's legal-
                       entity owner reference)
  geoCoordinates     : object (latitude / longitude
                       in WGS 84)

gs1LogisticUnitRecord:
  unitId             : string (uuidv7)
  sscc               : string (GS1 SSCC-18)
  contentRef         : array of object (per-GTIN /
                       quantity contents)
  parentSsccRef      : string (the parent SSCC for
                       nested logistic units;
                       absent for top-level units)
```

## §4 EPCIS 2.0 Critical-Tracking-Event Record

The EPCIS 2.0 + CBV 2.0 event record is the
canonical traceability event:

```
epcisEvent:
  eventId            : string (uuidv7)
  eventTime          : string (ISO 8601 with at-
                       least second precision)
  eventTimeZoneOffset : string (ISO 8601 offset)
  recordTime         : string (ISO 8601; the time
                       the operator's system
                       recorded the event)
  eventKind          : enum ("ObjectEvent" |
                       "AggregationEvent" |
                       "TransactionEvent" |
                       "TransformationEvent" |
                       "AssociationEvent")
  bizStep            : enum ("commissioning" |
                       "harvesting" |
                       "receiving" | "shipping" |
                       "transporting" |
                       "transforming" | "packing"
                       | "storing" |
                       "decommissioning" |
                       "destroying" | "user-defined")
  disposition        : enum ("active" | "in-
                       progress" | "completed" |
                       "in-transit" | "expired" |
                       "destroyed" | "recalled" |
                       "user-defined")
  readPoint          : string (the GS1 GLN of the
                       location where the event
                       was observed)
  bizLocation        : string (the GS1 GLN of the
                       business location)
  epcs               : array of string (the EPC /
                       SGTIN / SSCC instance
                       identifiers involved)
  quantityList       : array of object (per-EPC-
                       class quantity for case-and-
                       pallet level events)
  bizTransactions    : array of object (purchase-
                       order, despatch-advice,
                       receiving-advice references)
  ilmd               : object (Instance / Lot
                       Master Data — the lot /
                       batch attributes and the
                       FSMA 204 KDEs)
```

## §5 Key Data Element Record (FSMA Section 204)

For US-jurisdiction operators handling Food
Traceability List (FTL) commodities:

```
kdeRecord:
  kdeId              : string (uuidv7)
  associatedEpcisEventRef : string
  ftlCommodity       : enum ("leafy-greens" |
                       "tomatoes" | "tropical-tree-
                       fruits" | "fresh-cucumbers"
                       | "fresh-herbs" | "melons"
                       | "peppers" | "sprouts" |
                       "shell-eggs" | "nut-butters"
                       | "ready-to-eat-deli-salads"
                       | "cheeses" | "finfish" |
                       "crustaceans" | "molluscan-
                       shellfish" | "user-defined")
  cteKind            : enum ("harvesting" |
                       "cooling" | "initial-packing"
                       | "first-land-based-receiver"
                       | "shipping" | "receiving"
                       | "transformation")
  kdes               : object (the per-CTE-kind KDE
                       set per 21 CFR 1.1330 to
                       1.1340 — traceability lot
                       code, traceability product
                       description, location
                       descriptions, dates / times,
                       quantity / unit-of-measure)
  traceabilityLotCode : string (the FSMA traceability
                       lot code)
  traceabilityLotCodeSource : string (the originator
                       of the lot code per 21 CFR
                       1.1320)
```

## §6 Supply-Chain-Partner Record

```
supplyChainPartner:
  partnerId          : string (uuidv7)
  partnerKind        : enum ("supplier" | "customer"
                       | "logistics-provider" |
                       "third-party-warehouse" |
                       "co-manufacturer" |
                       "co-packer" | "user-defined")
  legalName          : string
  jurisdiction       : string (ISO 3166-1)
  gln                : string
  brcgsIfsFssc22000SqfRef : string (GFSI-scheme
                       certification reference;
                       absent if not GFSI-certified)
  fsvpVerificationRef : string (FDA FSVP verification
                       record reference for foreign
                       suppliers; absent if the
                       supplier is domestic or the
                       FSMA-204 exemption applies)
```

## §7 Food-Safety-Management Record

```
fsmsRecord:
  recordId           : string (uuidv7)
  recordKind         : enum ("haccp-plan" |
                       "iso-22000-management-system
                       -review" | "gfsi-scheme-
                       internal-audit" | "supplier-
                       audit" | "external-audit" |
                       "regulatory-inspection" |
                       "consumer-complaint-record"
                       | "internal-deviation" |
                       "user-defined")
  conductedAt        : string (ISO 8601)
  conductorRef       : string
  reportRef          : string (URI of the report)
  correctiveActions  : array of object (planned
                       remediation, owner, due
                       date, completion status)
```

## §8 Recall-and-Withdrawal Record

```
recallRecord:
  recallId           : string (uuidv7)
  initiatedAt        : string (ISO 8601)
  recallClass        : enum ("class-i-serious-
                       health-consequences" |
                       "class-ii-temporary-or-
                       reversible" | "class-iii-
                       not-likely-to-cause-
                       adverse-health" | "withdrawal
                       -no-violation" | "user-
                       defined")
  affectedGtinSet    : array of string
  affectedLotCodeSet : array of string
  hazardKind         : enum ("microbiological-
                       contamination" | "chemical-
                       contamination" |
                       "physical-contamination" |
                       "undeclared-allergen" |
                       "mislabeling" | "off-spec-
                       product" | "tampering" |
                       "user-defined")
  rootCauseRef       : string (URI of the root-
                       cause analysis)
  fdaRecallNumber    : string (the FDA-assigned
                       recall number; absent if
                       not US)
  euRasffNotificationRef : string (the EU RASFF
                       notification number; absent
                       if not EU)
  krMfdsNotificationRef : string (the KR MFDS
                       회수·폐기 명령 reference;
                       absent if not KR)
  totalUnitsAffected : integer
  totalUnitsRecovered : integer
  consumerCommunicationRef : string (URI of the
                       consumer-facing recall
                       notice)
```

## §9 Consumer-Disclosure Record

```
consumerDisclosure:
  disclosureId       : string (uuidv7)
  productRef         : string (GTIN reference)
  allergenDisclosures : array of enum ("gluten-
                       containing-cereals" |
                       "crustaceans" | "eggs" |
                       "fish" | "peanuts" |
                       "soybeans" | "milk" |
                       "tree-nuts" | "celery" |
                       "mustard" | "sesame" |
                       "sulphur-dioxide-sulphites"
                       | "lupin" | "molluscs" |
                       "user-defined")
  countryOfOrigin    : string (ISO 3166-1)
  nutritionPanelRef  : string (URI of the
                       nutrition-facts panel per
                       EU Reg 1169/2011 Annex XV +
                       FDA 21 CFR 101.9 + KR
                       표시·광고법)
  smartLabelRef      : string (URI of the GS1
                       SmartLabel / Digital Link
                       resolver entry)
```

## §10 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for every food product
the operator handles, exercise the EPCIS 2.0 CTE
recording on the operator's published cadence,
satisfy the FSMA 204 KDE discipline for the FTL
commodities, and preserve the records under the
operating jurisdiction's recordkeeping discipline
(US FDA FSMA 204 two-year retention; US Bioterrorism
Act one-step-forward / one-step-back for two years;
EU Reg 178/2002 traceability — Article 18 obligations
without specific retention; KR 식품위생법 5-year
retention).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-food-traceability
- **Last Updated:** 2026-04-28
