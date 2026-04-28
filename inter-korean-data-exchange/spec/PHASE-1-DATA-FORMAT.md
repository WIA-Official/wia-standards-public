# WIA-inter-korean-data-exchange PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-inter-korean-data-exchange
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-inter-korean-data-exchange. The standard covers persistent
record shapes for cross-DMZ data exchange between authorised
South-Korean operators and their North-Korean counterparts in
the limited domains where such exchange is permitted under the
South Korean Inter-Korean Exchange and Cooperation Act and
the parties' bilateral arrangements: family-reunion records,
humanitarian-aid manifests, separated-family communication
logs, joint-venture inventories, and inter-Korean liaison
correspondence. The format is consumed by the Ministry of
Unification, the Inter-Korean Exchange and Cooperation
Bureau, designated humanitarian organisations (Korea Red
Cross, KOICA, registered NGOs), and the operator-side IT
systems that prepare exchange artefacts for review.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO 3166-1 (country codes; cited for KP, KR, and the
  reciprocal-jurisdiction designations)
- ISO 639-1 / 639-2 (language codes; both `ko` references the
  unified Korean language while north-side and south-side
  orthography differences are recorded separately)
- ISO 15924 (script codes; both sides use Hangul `Hang`,
  with limited Hanja `Hani` use on certain historical records)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 17025:2017 (testing and calibration laboratories;
  cited where humanitarian-aid manifest verification is
  performed at an accredited laboratory)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 5322 (internet message format; cited as envelope
  for liaison correspondence)
- W3C XML 1.1 (legacy import only)
- UN OCHA Common Operational Datasets (cited as the reference
  vocabulary for humanitarian-aid records)
- Sphere Standards 2018 (humanitarian charter and minimum
  standards in humanitarian response — cited normatively for
  humanitarian-aid manifest classifications)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts produced
and consumed by an inter-Korean exchange operator. Implementations
covered include:

- Family-reunion administration systems operated by the Korea
  Red Cross and the Ministry of Unification.
- Humanitarian-aid logistics systems operated by KOICA, the
  Korea Red Cross, and registered humanitarian NGOs.
- Inter-Korean liaison correspondence systems that exchange
  formal letters and protocol documents under the Inter-Korean
  Exchange and Cooperation Act.
- Joint-venture inventory systems for the few cross-border
  industrial and tourism cooperatives that have operated.
- Cultural-exchange records for heritage repatriation,
  scholarly cooperation, and sporting exchanges.

Direct citizen-to-citizen messaging across the DMZ, military
or intelligence communications, and any commercial activity
not authorised by the Inter-Korean Exchange and Cooperation
Act are out of scope.

## §2 Programme Identifier

```
programmeId         : string (uuidv7)
programmeOperator   : string (institutional identifier of the
                        South-Korean operator authorised under
                        the Inter-Korean Exchange and
                        Cooperation Act; e.g. Korea Red Cross,
                        KOICA, an registered NGO)
programmeRegistered : string (ISO 8601 / RFC 3339)
exchangeDomains     : array of enum ("family-reunion" |
                        "humanitarian-aid" |
                        "liaison-correspondence" |
                        "joint-venture-inventory" |
                        "cultural-exchange" |
                        "scholarly-exchange" |
                        "sport-exchange")
authorisationRef    : string (Inter-Korean Exchange and
                        Cooperation Act authorisation reference
                        issued by the Ministry of Unification)
counterpartRef      : string (north-side counterpart organisation
                        identifier, where the bilateral
                        arrangement names one)
programmeStatus     : enum ("draft" | "operating" |
                        "suspended" | "concluded" |
                        "archived")
```

## §3 Personal-Identity Token

The DATA-FORMAT layer never carries direct personal identity
across the DMZ. Each subject is represented by an opaque token
that the south-side operator's CRM maps to the subject's
identity records held under the operator's data-protection
policy.

```
identityToken:
  tokenId            : string (uuidv7)
  side               : enum ("south" | "north")
  jurisdiction       : enum ("KR" | "KP" | "third-country-
                       resident-of-korean-origin")
  ageBand            : enum ("minor" | "adult-working-age" |
                        "elder-65-79" | "elder-80-plus")
  reunionEligibility : enum ("registered" | "deceased-confirmed"
                        | "deceased-unconfirmed" |
                        "withdrawn" | "not-applicable")
```

Age-band representation is intentionally coarse so that
downstream consumers can prioritise reunion-list ordering
without exposing fine-grained personal data.

## §4 Family-Reunion Record

```
familyReunion:
  reunionId           : string (uuidv7)
  programmeId         : string (uuidv7)
  applicantTokenRef   : string (south-side identity token)
  counterpartTokenRef : string (north-side identity token; absent
                         until the north side confirms a match)
  registeredAt        : string (ISO 8601)
  matchConfirmedAt    : string (ISO 8601; absent until matched)
  reunionRound        : string (operator-internal round
                         identifier for batched reunion events)
  reunionMode         : enum ("in-person-mt-kumgang" |
                         "in-person-other-venue" |
                         "video-conference" |
                         "letter-exchange" |
                         "voice-call")
  outcomeAt           : string (ISO 8601; absent until reunion
                         occurred or was cancelled)
  outcome             : enum ("completed" | "deceased-before"
                         | "withdrawn" | "cancelled-political"
                         | "cancelled-other")
```

## §5 Humanitarian-Aid Manifest

Humanitarian-aid manifests follow UN OCHA Common Operational
Datasets vocabulary so that aid records flowing across the DMZ
can be reconciled against international humanitarian reporting
without translation loss.

```
aidManifest:
  manifestId          : string (uuidv7)
  programmeId         : string (uuidv7)
  authorisationRef    : string (Ministry of Unification
                         shipment authorisation)
  packagedAt          : string (ISO 8601)
  packagedAtFacility  : string (south-side facility identifier)
  shippedAt           : string (ISO 8601; absent until shipped)
  receivedAt          : string (ISO 8601; absent until receipt
                         confirmed by north-side counterpart)
  routeCode           : enum ("dorasan-kaesong-rail" |
                         "dorasan-kaesong-road" |
                         "ganghwa-haeju-sea" |
                         "wonsan-dprk-port" |
                         "third-country-via-china" |
                         "third-country-via-russia" |
                         "third-country-via-ngo-corridor")
  cargoLines          : array of CargoLine
  recipientCounterparty: string (north-side organisation
                         identifier; e.g. "KP-RED-CROSS")
  beneficiaryClassification : enum ("infants-under-5" |
                         "pregnant-and-lactating" |
                         "school-age-children" |
                         "elderly" | "general-population" |
                         "disaster-affected" | "mixed")

CargoLine:
  cargoCode           : string (UN OCHA cluster classification
                         e.g. "WASH-HYG", "FOOD-CER",
                         "HEALTH-MED", "SHELTER-NFI")
  description         : string (free text; redacted on export
                         when contains operator PII)
  quantity            : number
  unitCode            : string (UN/CEFACT recommendation 20
                         common code; e.g. "KGM" kilogram, "EA"
                         each, "MTQ" cubic metre)
  expiryDate          : string (ISO 8601 date; required for
                         consumables)
  hazardClass         : enum ("non-hazardous" |
                         "un-class-3-flammable-liquid" |
                         "un-class-6.1-toxic" |
                         "un-class-9-misc" |
                         "user-defined")
```

## §6 Liaison Correspondence Record

```
liaisonCorrespondence:
  correspondenceId    : string (uuidv7)
  programmeId         : string (uuidv7)
  direction           : enum ("south-to-north" |
                         "north-to-south")
  channel             : enum ("inter-korean-liaison-office" |
                         "panmunjom-exchange" |
                         "ministry-direct-courier" |
                         "third-country-conduit")
  sentAt              : string (ISO 8601)
  receivedAt          : string (ISO 8601; absent until receipt
                         acknowledged)
  classification      : enum ("administrative" | "humanitarian"
                         | "cultural" | "scholarly" |
                         "joint-venture" | "ceremonial")
  subjectRef          : string (operator-internal subject
                         classification code)
  bodyRef             : string (content-addressed URI of the
                         correspondence body, redacted of any
                         operator PII before storage)
  signatoryRef        : string (south-side signatory token;
                         the north-side signatory is recorded
                         as a free-text role rather than an
                         identity token)
```

## §7 Joint-Venture Inventory Record

```
jointVentureInventory:
  inventoryId         : string (uuidv7)
  programmeId         : string (uuidv7)
  ventureRef          : string (joint-venture registration
                         reference under the Inter-Korean
                         Exchange and Cooperation Act)
  capturedAt          : string (ISO 8601)
  inventoryClass      : enum ("raw-materials" |
                         "work-in-progress" | "finished-goods"
                         | "equipment-and-fixtures" |
                         "intellectual-property")
  itemLines           : array of object (per-item code,
                         description redacted of PII, quantity,
                         unit code, valuation in ISO 4217 KRW
                         or USD per the ventur's accounting
                         convention)
  custodyLocation     : enum ("kaesong-industrial-complex" |
                         "mt-kumgang-tourist-zone" |
                         "rason-special-economic-zone" |
                         "third-country-warehouse" |
                         "south-side-bonded-warehouse")
```

## §8 Cultural-Exchange Record

```
culturalExchange:
  exchangeId          : string (uuidv7)
  programmeId         : string (uuidv7)
  exchangeKind        : enum ("heritage-repatriation" |
                         "scholarly-conference" |
                         "performing-arts-troupe" |
                         "sport-team-exchange" |
                         "joint-exhibition" |
                         "language-training")
  startedAt           : string (ISO 8601)
  endedAt             : string (ISO 8601)
  participantCount    : integer (south-side total; north-side
                         total recorded separately under
                         operator's bilateral-reporting
                         arrangement)
  venueRef            : string (venue identifier; for in-person
                         exchanges, this is the negotiated
                         venue; for virtual exchanges, the
                         operator's relay platform)
  artefactCatalogueRef: string (URI of the catalogue of
                         artefacts exchanged; required for
                         heritage-repatriation kind)
```

## §9 Distribution-Evidence Record

Humanitarian-aid manifests that have completed shipment
require distribution evidence so that downstream auditors
(Ministry of Unification, the operator's funder, the public
disclosure register) can verify that the aid reached the
declared beneficiaries.

```
distributionEvidence:
  evidenceId          : string (uuidv7)
  manifestId          : string (uuidv7)
  capturedAt          : string (ISO 8601)
  distributorRef      : string (north-side counterpart that
                         distributed the aid; e.g.
                         "KP-RED-CROSS")
  beneficiaryClassification : enum (same enum as PHASE-1 §5;
                         MUST match the manifest's declared
                         classification)
  distributionLocations : array of object (north-side
                         distribution sites named in the
                         counterpart's distribution report)
  evidenceArtefactRefs : array of string (URIs of
                         distribution-report artefacts; image
                         references are subject to the
                         operator's image-disclosure policy)
  diversionFlag       : boolean (true when the operator's
                         monitoring identified diversion
                         concerns)
```

## §10 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every authorised exchange and honour
the identity-token rule in §3 (no direct personal identity
across the DMZ).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-inter-korean-data-exchange
- **Last Updated:** 2026-04-28
