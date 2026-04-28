# WIA-economic-integration PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-economic-integration
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-economic-integration. The
standard covers the persistent record shapes
that a customs administration, a trade-
facilitation single-window operator, a national
statistical office contributing to UN COMTRADE,
a chamber of commerce issuing certificates of
origin, an authorised economic operator, a
freight forwarder, a financial-institution
operator handling cross-border payments and
documentary credits, and an inter-state economic-
union secretariat (the EU Single Market
secretariat, the ASEAN Economic Community
secretariat, an USMCA / CPTPP / RCEP working
group) maintain when registering a trade
declaration, exchanging an electronic-trade
message under UN/EDIFACT or ISO 20022,
publishing a trade-statistics dataset to UN
COMTRADE, anchoring an Authorised Economic
Operator certificate, transmitting a
documentary credit under the ICC UCP 600
framework, and tracking the per-shipment
chain-of-custody trail. Records are consumed
by the destination customs authority, by the
banker financing the trade, by the freight
forwarder coordinating the multimodal shipment,
by the importer / exporter executing the
contract, and — where the trade involves a
controlled-goods category — by the supervisory
export-control authority overseeing the
licensing decision.

References (CITATION-POLICY ALLOW only):

- WTO General Agreement on Tariffs and Trade
  1994 (GATT 1994), the WTO Agreement on
  Trade Facilitation (TFA, 2014, in force
  2017), the WTO Agreement on Government
  Procurement, the WTO Agreement on Trade-
  Related Aspects of Intellectual Property
  Rights (TRIPS), the WTO Agreement on
  Customs Valuation
- World Customs Organization (WCO) Framework
  of Standards to Secure and Facilitate Global
  Trade (the WCO SAFE Framework, the 2018
  edition cited as the operative version)
- WCO Harmonized Commodity Description and
  Coding System (the Harmonized System or HS)
  — the 2022 edition cited as the operative
  version
- WCO Data Model version 3 — the cross-border
  customs message reference cited normatively
  for the per-declaration envelope in §4
- WCO Authorised Economic Operator (AEO)
  Programme guidance
- WCO SAFE Mutual Recognition Arrangement
  framework
- UN/EDIFACT (Electronic Data Interchange for
  Administration, Commerce and Transport) —
  the message family standardised under ISO
  9735:2002, with the directories from D.96A
  through D.21B; cited normatively for the
  ORDERS, INVOIC, DESADV, IFTMIN, IFTMBC, and
  CUSDEC messages in §5
- ISO 9735-1:2002 / -2:2002 / -3:2002 / -4:
  2002 / -5:2002 / -6:2002 / -7:2002 / -8:
  2002 / -9:2002 / -10:2014 (UN/EDIFACT —
  application-level syntax rules)
- UN/CEFACT (United Nations Centre for Trade
  Facilitation and Electronic Business)
  Recommendations — Recommendation 1 (UN
  Layout Key for Trade Documents),
  Recommendation 16 (UNLOCODE — Codes for
  Trade and Transport Locations),
  Recommendation 33 (Single Window Recommendation),
  and Recommendation 36 (Single Window
  Interoperability)
- UN Comtrade Database (the UN Commodity
  Trade Statistics Database) and its data-
  submission specification
- ISO 20022 (Universal Financial Industry
  Message Scheme) — the cross-border payments
  message family cited normatively for §6
- ISO 4217:2015 (currency codes)
- ISO 3166-1:2020 (country codes), ISO 3166-2
  (subdivision codes)
- ISO 9362:2022 (BIC — Business Identifier
  Code)
- ISO 13616:2020 (IBAN — International Bank
  Account Number)
- ISO 17442:2020 (LEI — Legal Entity
  Identifier)
- ICC Incoterms 2020 (the ICC Incoterms rules
  for the use of domestic and international
  trade terms — eleven rules: EXW, FCA, CPT,
  CIP, DAP, DPU, DDP, FAS, FOB, CFR, CIF)
- ICC Uniform Customs and Practice for
  Documentary Credits (UCP 600, in force
  2007) and ICC International Standard
  Banking Practice (ISBP 745)
- ICC Uniform Rules for Demand Guarantees
  (URDG 758)
- ICC Uniform Rules for Bank Payment
  Obligations (URBPO 750)
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)
- ISO/IEC 27001:2022 (information-security
  management — used for the chain-of-custody
  record discipline in §8)
- W3C Verifiable Credentials Data Model v2.0
  (cited where the operator binds an AEO
  certificate or a certificate of origin to
  a verifiable credential)
- W3C Open Digital Rights Language (ODRL) 2.2
- KR 관세법 (Customs Act), KR 대외무역법
  (Foreign Trade Act), KR 외국환거래법
  (Foreign Exchange Transactions Act)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a trader, a freight
forwarder, a customs administration, and a
financing institution coordinate a cross-
border trade transaction. Implementations
covered include:

- A national customs administration operating
  a single-window-of-trade per UN/CEFACT
  Recommendation 33.
- A trade-facilitation single-window operator
  ingesting UN/EDIFACT CUSDEC declarations.
- A national statistical office contributing
  to the UN COMTRADE database under the
  HS-2022 commodity coding.
- A chamber of commerce issuing electronic
  certificates of origin under the WTO Rules
  of Origin and the operator's national
  preferential-origin agreement.
- An authorised economic operator (AEO)
  operating under the WCO SAFE Framework
  Mutual Recognition Arrangement.
- A freight forwarder operating multimodal
  transport under the IFTMIN / IFTMBC /
  IFTSTA EDIFACT messages.
- A financial-institution operator
  transmitting cross-border payments under
  ISO 20022 pacs.008 / camt.054 / pain.001.
- An inter-state economic-union secretariat
  publishing the per-Member-State market-
  integration dataset.

The customs declaration, the multimodal-
transport message, the cross-border payment
message, and the certificate of origin
receive distinct encodings in this PHASE; the
additional safeguards required by each trade-
facilitation regime are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of
                       the operator — customs
                       administration, single-
                       window operator, national
                       statistical office,
                       chamber of commerce, AEO,
                       freight forwarder,
                       financial institution, or
                       economic-union
                       secretariat)
operatorRole         : enum ("customs-admin" |
                       "single-window-operator"
                       | "national-statistical-
                       office" | "chamber-of-
                       commerce" | "authorised-
                       economic-operator" |
                       "freight-forwarder" |
                       "financial-institution" |
                       "economic-union-
                       secretariat" | "user-
                       defined")
operatorJurisdiction : array of string (ISO
                       3166-1 country codes)
governingFrameworks  : array of enum ("WTO-GATT-
                       1994" | "WTO-TFA" |
                       "WTO-GPA" | "WTO-TRIPS" |
                       "WCO-SAFE-2018" |
                       "WCO-HS-2022" |
                       "WCO-DATA-MODEL-V3" |
                       "WCO-AEO-PROGRAMME" |
                       "UN-EDIFACT-D-21B" |
                       "ISO-9735-1" |
                       "UN-CEFACT-REC-1" |
                       "UN-CEFACT-REC-16-
                       UNLOCODE" |
                       "UN-CEFACT-REC-33-SW" |
                       "UN-CEFACT-REC-36-SW-
                       INTEROP" |
                       "UN-COMTRADE" |
                       "ISO-20022" | "ISO-4217"
                       | "ISO-3166-1" | "ISO-
                       9362-BIC" | "ISO-13616-
                       IBAN" | "ISO-17442-LEI" |
                       "ICC-INCOTERMS-2020" |
                       "ICC-UCP-600" | "ICC-
                       ISBP-745" | "ICC-URDG-
                       758" | "ICC-URBPO-750" |
                       "EU-UCC-952-2013" |
                       "ASEAN-AEC-BLUEPRINT" |
                       "USMCA" | "CPTPP" |
                       "RCEP" |
                       "KR-관세법" | "KR-대외
                       무역법" | "KR-외국환거래법"
                       | "user-defined")
aeoCertificate       : object (the AEO
                       certificate reference,
                       the issuing customs
                       authority, the AEO type
                       — Authorised Economic
                       Operator Customs (AEOC),
                       Authorised Economic
                       Operator Security and
                       Safety (AEOS), or
                       Authorised Economic
                       Operator Full (AEOF) —
                       and the certificate's
                       expiry date)
programmeStatus      : enum ("design" |
                       "operating" | "limited-
                       rollout" | "wind-down" |
                       "archived")
```

## §3 Trade Declaration Record (WCO Data Model)

```
declarationRecord:
  declarationId      : string (uuidv7)
  declarationType    : enum ("import-clearance"
                       | "export-clearance" |
                       "transit" | "transhipment"
                       | "warehousing" |
                       "temporary-admission" |
                       "user-defined")
  consigneeRef       : object (the consignee's
                       Legal Entity Identifier
                       per ISO 17442 and the
                       operator's customs-system
                       trader registration
                       number)
  consignorRef       : object (the consignor's
                       Legal Entity Identifier
                       and the operator's
                       customs-system trader
                       registration number)
  goodsDescription   : array of object (per-
                       commodity envelope —
                       HS-2022 ten-digit
                       classification, gross
                       weight in kg per ISO
                       80000-4, net weight,
                       package count, package
                       type per UN/CEFACT
                       Recommendation 21)
  declaredCustomsValue : object (the customs
                       value per WTO Customs
                       Valuation Agreement
                       Article VII methods —
                       Method 1 transaction
                       value, Methods 2-6
                       fallback methods; the
                       value is declared with
                       the ISO 4217 currency
                       code)
  incoterm           : enum (per ICC Incoterms
                       2020 — "EXW" | "FCA" |
                       "CPT" | "CIP" | "DAP" |
                       "DPU" | "DDP" | "FAS" |
                       "FOB" | "CFR" | "CIF")
  transportDetails   : object (per-leg modal
                       envelope — air, ocean,
                       road, rail, multimodal
                       — with the UN/CEFACT
                       Recommendation 19 modal
                       code, the carrier's
                       identifier, the bill of
                       lading or air waybill
                       number, and the
                       UNLOCODE per UN/CEFACT
                       Recommendation 16)
```

## §4 Electronic-Trade Message Record (UN/EDIFACT)

```
edifactMessage:
  messageId          : string (uuidv7)
  messageType        : enum ("ORDERS" |
                       "ORDRSP" | "INVOIC" |
                       "DESADV" | "RECADV" |
                       "REMADV" | "IFTMIN" |
                       "IFTMBC" | "IFTMBP" |
                       "IFTSTA" | "CUSDEC" |
                       "CUSREP" | "CUSCAR" |
                       "user-defined")
  edifactDirectory   : enum (per UN/EDIFACT
                       directory release — "D-
                       16A" | "D-17A" | "D-18A"
                       | "D-19A" | "D-20A" |
                       "D-21A" | "D-21B" |
                       "user-defined")
  syntaxRules        : enum ("ISO-9735-2002")
  envelopePayload    : string (URI of the
                       complete EDIFACT
                       message — the UNB / UNH
                       / UNT / UNZ envelope
                       with per-message
                       segments)
  partnerExchange    : object (the sender and
                       receiver UN/EDIFACT
                       identifier per UNB
                       segment, the test or
                       production indicator,
                       the syntax-version
                       indicator)
```

## §5 Cross-Border Payment Record (ISO 20022)

```
paymentRecord:
  paymentId          : string (uuidv7)
  iso20022Message    : enum ("pacs.008" |
                       "pacs.009" | "pacs.002"
                       | "camt.054" | "camt.029"
                       | "camt.056" | "pain.001"
                       | "pain.002" | "user-
                       defined")
  initiatingParty    : object (the initiating
                       party's BIC per ISO 9362
                       and LEI per ISO 17442)
  beneficiaryParty   : object (the beneficiary's
                       BIC and LEI; account
                       identifier per ISO 13616
                       IBAN where the
                       transaction is in scope
                       of the IBAN registry)
  instructedAmount   : object (amount in the
                       declared currency per
                       ISO 4217)
  paymentPurpose     : object (the ISO 20022
                       Purpose Code per the
                       External Code Set;
                       trade-purpose codes such
                       as TRAD or CRED carry
                       the underlying trade
                       declaration reference)
  uniqueEnd-toEndTransactionRef : string (the
                       UETR per the SWIFT GPI
                       discipline)
```

## §6 Documentary Credit Record (ICC UCP 600)

```
documentaryCredit:
  creditId           : string (uuidv7)
  creditType         : enum ("irrevocable" |
                       "transferable" |
                       "back-to-back" |
                       "standby" | "user-
                       defined")
  issuingBank        : object (BIC per ISO 9362)
  beneficiaryBank    : object (BIC per ISO 9362)
  applicantRef       : object (the importer's
                       LEI per ISO 17442)
  beneficiaryRef     : object (the exporter's
                       LEI per ISO 17442)
  creditAmount       : object (amount in the
                       declared currency per
                       ISO 4217 with the
                       allowed-tolerance per
                       UCP 600 Article 30)
  expirePlace        : object (the place of
                       expiry per UCP 600
                       Article 6)
  expiryDate         : string (ISO 8601)
  documentsRequired  : array of object (the
                       per-document required
                       set under UCP 600
                       Articles 19-28 — bill of
                       lading, air waybill,
                       commercial invoice,
                       insurance document,
                       packing list,
                       certificate of origin)
  isbpProfile        : enum ("ISBP-745" |
                       "user-defined")
```

## §7 Certificate of Origin Record

```
originCertificate:
  certificateId      : string (uuidv7)
  issuingChamberRef  : string (the chamber of
                       commerce or competent
                       authority issuing the
                       certificate)
  beneficiaryRef     : object (the exporter's
                       LEI)
  goodsDescription   : array of object (HS-2022
                       classification + WTO
                       Rules of Origin
                       qualification per
                       Annex II of the
                       applicable preferential-
                       trade agreement)
  agreementRef       : enum ("EU-GSP" |
                       "USMCA" | "CPTPP" |
                       "RCEP" | "ASEAN" |
                       "ICC-NON-PREFERENTIAL"
                       | "user-defined")
  validityPeriod     : object (per the per-
                       agreement validity rules)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the declaration,
                       EDIFACT message, payment,
                       documentary credit, or
                       origin certificate
                       identifier)
  custodyEvent       : enum ("declaration-
                       lodged" | "declaration-
                       cleared" | "edifact-
                       transmitted" |
                       "iso20022-instructed" |
                       "documentary-credit-
                       issued" | "documents-
                       presented" | "documents-
                       paid" | "origin-
                       certified" | "withdrawn"
                       | "user-defined")
  eventTimestamp     : string (ISO 8601)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex
                       digest)
```

## §9 Manifest

Implementations publish a signed manifest
carrying `standardSlug` (constant value
"economic-integration"), `version`,
`implementation`, the operator's `aeoCertificate`
envelope, and the `profile` declaration that
selects which of the optional records
(EDIFACT, ISO 20022, UCP 600, origin) the
implementation supports. The manifest is
signed using a key whose public part is
published on the operator's `.well-known/wia/
economic-integration/` discovery endpoint
declared in PHASE-2.
