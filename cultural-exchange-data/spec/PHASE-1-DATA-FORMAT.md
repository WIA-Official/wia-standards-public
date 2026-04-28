# WIA-cultural-exchange-data PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-cultural-exchange-data
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-cultural-exchange-data. The
standard covers the persistent record shapes
that a memory-institution operator (a national
museum, a national or municipal library, a
state or institutional archive, a heritage-site
custodian, an audiovisual archive, a digital
humanities research centre, an academic or
civil-society heritage observer), an
intergovernmental cultural-exchange programme
operator (the UNESCO secretariat operating the
2005 Convention reporting cycle, a regional
cultural-cooperation body), a publisher
contributing to a cross-institutional catalogue,
and a public-procurement authority running a
cultural-data interoperability programme
maintain when registering an item in a shared
catalogue, exposing the item through an
international image-and-presentation API,
publishing a finding aid for an archival
collection, exchanging the per-item record
under a Resource Description and Access
discipline, and tracking the per-item rights-
expression and provenance trail. Records are
consumed by a researcher discovering the item
in a federated portal, by a curator re-using
the item in an exhibition, by an educator
embedding the item in a learning resource, by a
journalist citing the item in an investigation,
and — where the item is in scope of an
intergovernmental restitution discipline — by
the supervisory cultural-heritage authority
overseeing the cross-border claim.

References (CITATION-POLICY ALLOW only):

- UNESCO Convention on the Protection and
  Promotion of the Diversity of Cultural
  Expressions (the 2005 Convention) and its
  Operational Guidelines — the framework
  reference for the cultural-exchange
  programme record carried in §3
- UNESCO Convention concerning the Protection
  of the World Cultural and Natural Heritage
  (the 1972 World Heritage Convention) — cited
  where the item is bound to a UNESCO World
  Heritage property
- UNESCO Convention for the Safeguarding of
  the Intangible Cultural Heritage (the 2003
  Convention) — cited where the item is bound
  to an Intangible Cultural Heritage element
- ISO 15836-1:2017 (Dublin Core Metadata
  Element Set, Part 1: Core Elements) and ISO
  15836-2:2019 (Dublin Core Metadata Element
  Set, Part 2: DCMI Properties and Classes) —
  the normative element set carried by the
  cataloguing record in §4
- ISO 21127:2014 (CIDOC Conceptual Reference
  Model — a reference ontology for the
  exchange of cultural heritage information)
- ISO 23081-1:2017, ISO 23081-2:2021, ISO
  23081-3:2024 (information and documentation
  — managing metadata for records)
- ISO 25964-1:2011, ISO 25964-2:2013
  (information and documentation — thesauri
  and interoperability with other vocabularies)
- ISO 28560-1:2014, ISO 28560-2:2014, ISO
  28560-3:2014 (RFID in libraries)
- ISO 23950:1998 (Z39.50 information retrieval)
- IIIF Image API 3.0 and IIIF Presentation API
  3.0 (the International Image Interoperability
  Framework specifications cited normatively
  for the image-delivery and presentation
  envelope in §5)
- IIIF Authentication API 2.0 and IIIF Search
  API 2.0
- Encoded Archival Description (EAD) version
  3, published by the Society of American
  Archivists' Technical Subcommittee on
  Encoded Archival Standards in cooperation
  with the Library of Congress, cited
  normatively for the finding-aid envelope in
  §6
- Resource Description and Access (RDA): Toolkit
  the international cataloguing standard
  published by the RDA Steering Committee — the
  normative cataloguing rules referenced in §4
- METS (Metadata Encoding and Transmission
  Standard) and MODS (Metadata Object
  Description Schema) maintained by the
  Library of Congress — cited where the
  operator publishes a METS-wrapped digital
  object
- LIDO (Lightweight Information Describing
  Objects) maintained by the ICOM CIDOC
  Documentation Standards Working Group —
  cited where the operator publishes a LIDO-
  formatted object record
- Europeana Data Model (EDM) — cited where the
  operator contributes to Europeana
- Open Archives Initiative Protocol for
  Metadata Harvesting (OAI-PMH) version 2.0 —
  cited for the harvest endpoint envelope
- W3C Linked Data Platform 1.0 and the W3C SKOS
  (Simple Knowledge Organization System)
  Reference — cited for the linked-data
  publication of catalogue records
- W3C Open Digital Rights Language (ODRL) 2.2
  Information Model — cited for the rights
  expression carried by every record
- W3C Verifiable Credentials Data Model v2.0 —
  optional, cited for the re-issuance of
  attribution attestations
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)
- ISO/IEC 27001:2022 (information-security
  management)
- EU Directive 2019/790 on copyright in the
  Digital Single Market (Article 14 and
  Article 17) — cited where the operator
  publishes works from the public domain
  alongside in-copyright works
- KR 박물관 및 미술관 진흥법 (Museum and Art
  Gallery Promotion Act) and KR 문화재보호법
  (Cultural Heritage Protection Act)

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a memory-institution
catalogues an item, exposes it through an
international image-and-presentation API,
publishes a finding aid for an archival
collection, exchanges the per-item record under
a cross-institutional catalogue, and tracks the
per-item provenance and rights envelope.
Implementations covered include:

- A national-museum cataloguing system
  publishing per-object records under the LIDO
  schema with images served through IIIF
  Image API 3.0.
- A national-library bibliographic catalogue
  exposing records under MODS and serving
  digital surrogates through IIIF Presentation
  API 3.0.
- A national-archive finding-aid system
  publishing per-collection EAD3 finding aids.
- An audiovisual archive publishing time-based
  media descriptions under EBUCore and
  PBCore — cited as user-defined extensions of
  the LIDO schema.
- A heritage-site custodian publishing the
  UNESCO World Heritage property record
  bound to a IIIF Presentation API manifest.
- A digital humanities research centre
  publishing a CIDOC CRM-aligned event-and-
  entity graph derived from the catalogued
  records.
- An intergovernmental cultural-exchange
  programme operator (the UNESCO secretariat
  operating the 2005 Convention quadrennial
  reporting cycle) publishing per-Member-State
  reports.

The cataloguing record, the IIIF presentation
manifest, the EAD3 finding aid, and the
intergovernmental programme report receive
distinct encodings in this PHASE; the
additional safeguards required by each cultural-
heritage discipline are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of
                       the operator — museum,
                       library, archive, heritage-
                       site custodian, audiovisual
                       archive, digital-humanities
                       centre, or
                       intergovernmental
                       programme)
operatorRole         : enum ("museum" | "library"
                       | "archive" | "heritage-
                       site-custodian" |
                       "audiovisual-archive" |
                       "digital-humanities-
                       centre" |
                       "intergovernmental-
                       programme" | "user-
                       defined")
governingFrameworks  : array of enum ("UNESCO-
                       2005-CONVENTION" |
                       "UNESCO-1972-WORLD-
                       HERITAGE" | "UNESCO-2003-
                       INTANGIBLE" | "ISO-15836-
                       1" | "ISO-15836-2" |
                       "ISO-21127-CIDOC-CRM" |
                       "ISO-23081" |
                       "ISO-25964" |
                       "IIIF-IMAGE-API-3" |
                       "IIIF-PRESENTATION-API-3"
                       | "IIIF-AUTH-API-2" |
                       "IIIF-SEARCH-API-2" |
                       "EAD3" | "RDA-TOOLKIT" |
                       "METS" | "MODS" |
                       "LIDO" | "EDM" |
                       "OAI-PMH-2" |
                       "W3C-SKOS" |
                       "W3C-LDP-1" |
                       "W3C-ODRL" |
                       "EU-COPYRIGHT-DSM-2019-
                       790" |
                       "KR-박물관및미술관진흥법" |
                       "KR-문화재보호법" |
                       "user-defined")
accreditationStatus  : object (the operator's
                       memberships and
                       accreditations — ICOM
                       museum membership, IFLA
                       library membership, ICA
                       archive membership, the
                       UNESCO Member-State
                       reporting status under
                       the 2005 Convention)
programmeStatus      : enum ("design" |
                       "operating" | "limited-
                       rollout" | "wind-down" |
                       "archived")
```

## §3 Cultural-Exchange Programme Record

```
programmeRecord:
  cepId              : string (uuidv7)
  publishingOperator : string (the operator's
                       legal-entity reference)
  programmeType      : enum ("unesco-2005-
                       quadrennial-report" |
                       "world-heritage-state-
                       of-conservation" |
                       "intangible-heritage-
                       state-of-the-element" |
                       "bilateral-cultural-
                       cooperation" |
                       "regional-cultural-fund"
                       | "user-defined")
  reportingPeriod    : object (start and end
                       dates per ISO 8601)
  subject            : object (the policy
                       question, the heritage
                       property reference, or
                       the intangible heritage
                       element under
                       deliberation)
  rightsExpression   : object (the W3C ODRL
                       2.2 policy expression
                       carrying the programme
                       record's permission to
                       redistribute, to use for
                       research, or to
                       anonymise)
```

## §4 Cataloguing Record

```
catalogueRecord:
  itemId             : string (uuidv7; the
                       operator's persistent
                       identifier for the item)
  identifierBindings : array of object (per-
                       authority identifiers —
                       for example the operator's
                       accession number, the
                       Library of Congress
                       Control Number, the
                       International Standard
                       Bibliographic Number, the
                       OCLC WorldCat
                       identifier — each
                       carrying the issuing
                       authority and the scope
                       of use)
  cataloguingScheme  : enum ("RDA-TOOLKIT" |
                       "MODS" | "LIDO" |
                       "EAD3" | "EDM" |
                       "CIDOC-CRM" |
                       "DUBLIN-CORE-15836-1" |
                       "DUBLIN-CORE-15836-2" |
                       "user-defined")
  itemType           : enum ("text" | "image" |
                       "moving-image" |
                       "sound" | "physical-
                       object" | "manuscript" |
                       "archival-collection" |
                       "monument" | "intangible-
                       practice" | "user-
                       defined")
  title              : object (per-language
                       title array, the
                       declared language code
                       per BCP 47)
  creator            : array of object (per-role
                       creator — author,
                       photographer,
                       sculptor, donor; each
                       creator carrying a name
                       and a per-authority
                       authority-record
                       reference such as the
                       VIAF identifier or the
                       operator's local
                       authority reference)
  date               : object (per-event date —
                       creation, publication,
                       acquisition; each event
                       carrying ISO 8601 with
                       partial-date support per
                       the cataloguing scheme's
                       partial-date convention)
  subjectClassification : array of object (per-
                       scheme subject heading —
                       LC Subject Headings,
                       Getty AAT, ICONCLASS,
                       UDC, MeSH; each
                       classification carrying
                       the scheme reference and
                       the per-term identifier)
  rightsExpression   : object (the W3C ODRL
                       2.2 policy expression
                       carrying the item's
                       copyright status, the
                       public-domain mark, the
                       Creative-Commons licence,
                       or the per-jurisdiction
                       rights statement)
```

## §5 IIIF Manifest Record

```
iiifManifest:
  manifestId         : string (uuidv7)
  itemRef            : string (PHASE-1 §4
                       record reference)
  iiifApiVersion     : enum ("Image-API-3" |
                       "Presentation-API-3" |
                       "Auth-API-2" |
                       "Search-API-2")
  manifestUri        : string (URI of the
                       published IIIF
                       Presentation API 3.0
                       manifest)
  imageServices      : array of object (per-
                       image IIIF Image API 3.0
                       service — the canonical
                       image service URI, the
                       maximum width and height,
                       and the supported image
                       feature set)
  rangesAndStructure : array of object (the IIIF
                       Range structure for
                       multi-page items, audio-
                       and-video time-based
                       segmentation)
```

## §6 Archival Finding-Aid Record (EAD3)

```
findingAidRecord:
  faId               : string (uuidv7)
  collectionRef      : string (PHASE-1 §4
                       collection reference)
  ead3Document       : string (URI of the
                       published EAD3 finding
                       aid; the document is
                       served as
                       application/xml under
                       the EAD3 schema)
  arrangementHierarchy : array of object (the
                       collection's
                       arrangement-and-
                       description hierarchy,
                       carrying per-component
                       title, scope-and-content
                       note, and date range)
  conditionsOfAccess : object (the access-
                       restriction declaration
                       per the operator's
                       jurisdiction)
```

## §7 Linked-Data Graph Record

```
graphRecord:
  graphId            : string (uuidv7)
  itemRef            : string (PHASE-1 §4
                       record reference)
  graphSerialisation : enum ("RDF-Turtle" |
                       "RDF-XML" | "JSON-LD" |
                       "N-Triples" | "user-
                       defined")
  ontologySet        : array of enum ("CIDOC-
                       CRM-7.1.3" | "EDM" |
                       "Schema-org" | "Dublin-
                       Core" | "FOAF" | "SKOS"
                       | "user-defined")
  triplesUri         : string (URI of the
                       published triples set)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the catalogue,
                       IIIF, finding-aid, graph,
                       or programme record
                       identifier)
  custodyEvent       : enum ("catalogued" |
                       "iiif-published" |
                       "ead3-published" |
                       "graph-published" |
                       "rights-statement-
                       updated" | "restitution-
                       claim-received" |
                       "withdrawn" | "user-
                       defined")
  eventTimestamp     : string (ISO 8601 date-
                       time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex
                       digest)
```

## §9 Manifest

Implementations publish a signed manifest
carrying `standardSlug` (constant value
"cultural-exchange-data"), `version`,
`implementation`, the operator's
`accreditationStatus`, and the `profile`
declaration that selects which of the optional
records (IIIF, EAD3, graph, programme) the
implementation supports. The manifest is signed
using a key whose public part is published on
the operator's `.well-known/wia/cultural-
exchange-data/` discovery endpoint declared in
PHASE-2.
