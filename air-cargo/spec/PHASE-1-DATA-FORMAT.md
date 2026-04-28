# WIA-air-cargo PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-air-cargo
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-air-cargo. The standard covers
persistent record shapes for the lifecycle of an
air-cargo operator — the shipper, freight
forwarder, regulated agent / known consignor,
airline cargo division, ground-handling cargo
operator, and customs broker. Records cover the
shipment's entire chain — booking, build-up at the
forwarder, security screening, e-AWB and accompanying
documents, dangerous-goods declaration, regulated-
agent / known-consignor security regime, customs
declaration, ramp transfer, in-flight care
constraints, transshipment, delivery to consignee,
and post-flight reconciliation. Records are consumed
by the airline cargo system, the freight forwarder's
forwarding-management system, the regulated agent
and known consignor under the operating
jurisdiction's air-cargo security regime, the
operating jurisdiction's customs and border-
protection authority, the dangerous-goods regulator,
the airport cargo terminal, the international postal
operator (where airmail is in scope), and the
external auditors and supervisory authorities.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- ICAO Annex 18 (The Safe Transport of Dangerous
  Goods by Air) and ICAO Doc 9284 (Technical
  Instructions for the Safe Transport of Dangerous
  Goods by Air, "TI") + Doc 9481 (Emergency Response
  Guidance for Aircraft Incidents Involving
  Dangerous Goods)
- ICAO Annex 17 (Security)
- IATA Dangerous Goods Regulations (DGR), the
  industry-standard implementation of the ICAO
  Technical Instructions
- IATA TACT (The Air Cargo Tariff and Rules) Rules
  manual, the operational baseline for cargo
  acceptance, handling, and rate application
- IATA Cargo-IMP (Cargo Interchange Message
  Procedures) — the legacy cargo messaging
  baseline (FFM, FWB, FHL, FNA / FBL / FRC / FOH /
  FFA / FZB)
- IATA Cargo-XML — the XML messaging baseline that
  modernises Cargo-IMP
- IATA e-AWB (electronic Air Waybill) Resolution
  672 + Multilateral e-AWB Agreement
- IATA Resolution 833 (Master Air Waybill)
- IATA Resolution 600a / b (Master and House Air
  Waybill)
- IATA Resolution 674 (Cargo iQ)
- IATA Center of Excellence for Independent
  Validators (CEIV) for Pharma / Live Animals /
  Lithium Battery / Fresh
- IATA Live Animal Regulations (LAR)
- IATA Perishable Cargo Regulations (PCR)
- IATA Temperature Control Regulations (TCR)
- IATA Lithium Battery Shipping Guidelines
- World Customs Organization (WCO) SAFE Framework
  of Standards
- WCO Cargo XML / Cargo IMP-replacement messages
  CUSCAR / CUSRES / CUSDEC / CUSEXP
- UN/EDIFACT IFTSTA (Status Report) / IFTMIN
  (Instruction Message) / IFTMCS (Booking
  Confirmation) / IFTSTA / IFTMIN
- UN Recommendation 19 (UN/LOCODE) and UN/CEFACT
  Buy-Ship-Pay reference data model
- US 19 CFR Part 122 (Air Commerce Regulations) +
  Part 4 (Vessels in Foreign and Domestic Trades)
- US 49 CFR Parts 171-180 (Hazardous Materials
  Regulations, HMR)
- TSA 49 CFR 1544 + 1546 + 1548 + 1549 (Air Cargo
  Security)
- US Air Cargo Advance Screening (ACAS) under 19
  CFR 122.49b
- EU Regulation (EU) 2015/1998 (security
  implementing rules) + Reg (EU) 952/2013 (Union
  Customs Code) + Reg (EU) 2018/1808 (e-CMR
  cross-domain reference)
- EU Authorised Economic Operator (AEO) programme
  per UCC Article 38
- EU Pre-Loading Advance Cargo Information (PLACI)
  under EU Reg (EU) 2019/1715 + Reg (EU) 2024/1248
- EU Customs Pre-Arrival Security and Safety
  declaration ENS / Pre-Loading via ICS2
- KR 관세법 + KR 화물자동차 운수사업법 + KR
  국토교통부 항공안전 관계법령 + 항공보안법

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts an air-cargo operator (shipper, freight
forwarder, regulated agent, known consignor, airline
cargo division, ground-handler, customs broker)
maintains:

- The shipper-and-consignee record.
- The booking-and-rate record.
- The e-AWB record (master / house).
- The dangerous-goods declaration record.
- The CEIV-Pharma / LAR / PCR / TCR specialised
  record.
- The security-screening record.
- The regulated-agent / known-consignor record.
- The customs declaration record.
- The cargo-iQ shipment-progress record.
- The post-flight reconciliation record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("shipper" | "freight-
                       forwarder" | "regulated-agent
                       -ra" | "known-consignor-kc" |
                       "airline-cargo-division" |
                       "ground-handler-cargo-
                       terminal" | "customs-broker"
                       | "integrator" | "international
                       -postal-operator" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("ICAO-ANNEX-
                       18" | "ICAO-DOC-9284-TI" |
                       "ICAO-DOC-9481" |
                       "ICAO-ANNEX-17" |
                       "IATA-DGR" | "IATA-TACT" |
                       "IATA-CARGO-IMP" |
                       "IATA-CARGO-XML" |
                       "IATA-RES-672-E-AWB" |
                       "IATA-RES-833-MAWB" |
                       "IATA-RES-600A-B-MAWB-HAWB"
                       | "IATA-RES-674-CARGO-IQ" |
                       "IATA-CEIV-PHARMA" |
                       "IATA-CEIV-LITHIUM" |
                       "IATA-CEIV-LIVE-ANIMALS" |
                       "IATA-CEIV-FRESH" |
                       "IATA-LAR" | "IATA-PCR" |
                       "IATA-TCR" |
                       "WCO-SAFE-FRAMEWORK" |
                       "WCO-CARGO-XML" |
                       "UN-EDIFACT-IFTSTA-IFTMIN-
                       IFTMCS" |
                       "UN-LOCODE-REC-19" |
                       "US-19-CFR-122" |
                       "US-49-CFR-171-180-HMR" |
                       "US-TSA-49-CFR-1544-1546-
                       1548-1549" |
                       "US-ACAS-19-CFR-122-49B" |
                       "EU-REG-2015-1998-SEC" |
                       "EU-UCC-952-2013" |
                       "EU-AEO-UCC-38" |
                       "EU-ICS2-PRE-LOADING" |
                       "KR-관세법" |
                       "KR-항공보안법" |
                       "user-defined")
ceivCertifications   : array of enum ("ceiv-pharma"
                       | "ceiv-lithium" | "ceiv-
                       live-animals" | "ceiv-fresh"
                       | "user-defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Shipper-and-Consignee Record

```
partyRecord:
  partyId            : string (uuidv7)
  role               : enum ("shipper" | "consignee"
                       | "notify-party" | "agent")
  legalName          : string
  jurisdiction       : string (ISO 3166-1)
  address            : object (structured address;
                       UN/LOCODE city code where
                       available)
  taxIdentifier      : object (per-jurisdiction tax
                       identifier — VAT / EORI /
                       DUNS / 사업자등록번호)
  knownConsignorRef  : string (the regulated
                       authority-issued KC
                       identifier; absent if not a
                       known consignor)
```

## §4 Booking-and-Rate Record

```
bookingRecord:
  bookingId          : string (uuidv7)
  bookingMessageRef  : enum ("iata-cargo-imp-fbl-
                       booking-list" | "iata-cargo-
                       imp-fra-allotment-request" |
                       "iata-cargo-xml-bookingrequest"
                       | "user-defined")
  carrierAirlineCode : string (IATA / ICAO airline
                       code)
  routing            : array of object (per-leg
                       origin / destination per
                       UN/LOCODE airport code)
  bookedTonnage      : number (kg)
  bookedVolume       : number (m³)
  rateBasis          : enum ("iata-tact-general-
                       cargo-rate" | "iata-tact-
                       specific-commodity-rate" |
                       "iata-tact-class-rate" |
                       "iata-tact-special-handling-
                       rate" | "negotiated-rate" |
                       "user-defined")
  bookedAt           : string (ISO 8601)
```

## §5 e-AWB Record (Master / House)

```
eAwbRecord:
  awbId              : string (uuidv7)
  awbKind            : enum ("master-awb" | "house-
                       awb" | "neutral-awb")
  awbNumber          : string (3-digit airline
                       prefix + 8-digit serial,
                       e.g. "157-12345675")
  iataResolution672Compliant : boolean
  shipperRef         : string
  consigneeRef       : string
  weight             : number (kg)
  volume             : number (m³)
  pieces             : integer
  natureAndQuantity  : string (natural-language
                       description)
  hsCode             : array of string (Harmonized
                       System tariff classification)
  declaredValueCarriage : object (currency-quantified)
  declaredValueCustoms : object (currency-quantified)
  specialHandlingCodes : array of string (IATA TACT
                       three-letter codes — DGR,
                       PER, AVI, COL, RFL, etc.)
  awbIssuedAt        : string (ISO 8601)
  awbDigest          : string (SHA-256 of the
                       canonical e-AWB payload)
```

## §6 Dangerous-Goods Declaration Record

```
dgDeclarationRecord:
  declarationId      : string (uuidv7)
  awbRef             : string
  shipper            : object (the DGR shipper's
                       declaration)
  unNumber           : string (UN four-digit number,
                       e.g. UN1230 methanol)
  properShippingName : string (per IATA DGR §4.2)
  packingGroup       : enum ("packing-group-i" |
                       "packing-group-ii" |
                       "packing-group-iii" |
                       "n/a")
  hazardClass        : array of string (the IATA
                       DGR class / sub-class — 1
                       Explosives / 2 Gases / 3
                       Flammable liquids / 4
                       Flammable solids / 5
                       Oxidizers and organic
                       peroxides / 6 Toxic and
                       infectious / 7 Radioactive /
                       8 Corrosives / 9
                       Miscellaneous including
                       lithium batteries)
  packingInstruction : string (the IATA DGR PI per
                       §5)
  packageType        : string
  netQuantityPerPackage : object
  numberOfPackages   : integer
  freightForwarderApprovedDgrCheckRef : string (the
                       freight forwarder's DGR-
                       acceptance checklist
                       reference per IATA DGR §9)
  emergencyResponseRef : string (URI of the ICAO
                       Doc 9481 emergency response
                       procedures)
```

## §7 CEIV-Pharma / LAR / PCR / TCR Specialised Record

```
specialisedHandlingRecord:
  recordId           : string (uuidv7)
  awbRef             : string
  ceivCertificationRef : enum ("ceiv-pharma" |
                       "ceiv-lithium" | "ceiv-live-
                       animals" | "ceiv-fresh" |
                       "user-defined")
  pharmaTemperatureRange : object (target temperature
                       range per IATA TCR — for
                       example +2 °C to +8 °C
                       refrigerated; -20 °C to
                       -10 °C frozen)
  liveAnimalSpecies  : string (the LAR identification
                       — required for live-animal
                       shipments)
  iataLarContainerRequirement : string (the LAR
                       container-requirement code)
  perishableCommodityCode : string (the IATA PCR
                       commodity code)
  temperatureChartRef : string (URI of the
                       continuous temperature log
                       across the journey)
```

## §8 Security-Screening Record

```
securityScreening:
  screeningId        : string (uuidv7)
  awbRef             : string
  screeningKind      : enum ("x-ray" | "explosive-
                       trace-detection-etd" |
                       "explosives-detection-system
                       -eds" | "physical-search" |
                       "canine-explosive-detection
                       -eddt" | "electromagnetic-
                       detection-emd" |
                       "user-defined")
  screeningLocationRef : string
  screenedBy         : string (the regulated agent
                       / known consignor / airline
                       cargo screener identity)
  screenedAt         : string (ISO 8601)
  outcomeKind        : enum ("clear" | "alarm-
                       resolved-by-secondary-
                       screening" | "alarm-not-
                       resolved-rejected-shipment"
                       | "user-defined")
  iataE-CSDOnboardCertificate : string (URI of the
                       e-Consignment Security
                       Declaration where applicable)
```

## §9 Customs Declaration Record

```
customsDeclaration:
  declarationId      : string (uuidv7)
  awbRef             : string
  declarationKind    : enum ("export-declaration" |
                       "import-declaration" |
                       "transit-declaration" |
                       "ens-pre-arrival-eu" |
                       "us-acas-pre-loading" |
                       "wco-cuscar-cargo-report" |
                       "user-defined")
  customsAuthorityRef : string (the operating
                       jurisdiction's authority — US
                       CBP, EU Member-State customs,
                       KR 관세청)
  declaredAt         : string (ISO 8601)
  hsCodes            : array of string
  customsValue       : object (currency-quantified)
  edifactMessageRef  : string (URI of the UN/EDIFACT
                       CUSCAR / CUSREP / CUSDEC
                       message)
  cargoXmlMessageRef : string (URI of the WCO
                       Cargo-XML message; absent
                       unless XML used)
  outcomeKind        : enum ("released" | "selected-
                       for-physical-inspection" |
                       "held-for-clarification" |
                       "rejected" | "user-defined")
```

## §10 Cargo-iQ Shipment-Progress Record

The IATA Resolution 674 Cargo iQ event-driven
shipment-progress record:

```
cargoIqEvent:
  eventId            : string (uuidv7)
  awbRef             : string
  shipmentRecordKey  : string (the Cargo iQ SRK)
  milestoneCode      : enum ("BKD-booking-
                       confirmed" | "FOH-freight-
                       on-hand" | "RCS-received-
                       cargo-from-shipper" |
                       "DEP-departure" | "ARR-
                       arrival" | "RCT-received-
                       cargo-at-transshipment" |
                       "DLV-delivered-to-consignee"
                       | "NFD-notification-flight-
                       arrival" | "AWD-shipment-
                       awaiting-consignee" |
                       "user-defined")
  observedAt         : string (ISO 8601)
  locationRef        : string (UN/LOCODE)
  carrierRef         : string
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
each of the records defined above for every shipment
the operator handles, satisfy the security-screening
discipline at every regulated-agent / airline cargo
acceptance point, exercise the dangerous-goods
declaration discipline per IATA DGR, and preserve
records under the operating jurisdiction's
recordkeeping discipline (US 19 CFR 122 retention;
EU UCC five-year retention; KR 관세법 7-year
retention; IATA TACT three-year retention).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-air-cargo
- **Last Updated:** 2026-04-28
