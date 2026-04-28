# WIA-museum-digital-archive PHASE 4 — Integration Specification

**Standard:** WIA-museum-digital-archive
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-museum-digital-archive
integrates with adjacent cultural-heritage, scholarly-
publication, public-discovery, and preservation
systems: Europeana and national aggregators, Linked
Art and Linked-Data communities, Wikidata / Wikibase,
Getty vocabularies (AAT / TGN / ULAN / IA),
Iconclass, scholarly catalogues raisonnés, the IIIF
ecosystem, ResourceSync harvesters, OAIS-conformant
trustworthy-digital-repositories, ICOM Red Lists,
sponsored / community trust frameworks, and downstream
analytics consumers. It also specifies the operational
binding to companion WIA standards.

References (CITATION-POLICY ALLOW only):
- Europeana Data Model (EDM) 5.2.8; Europeana Publishing Framework
- Linked Art (community spec); CIDOC-CRM (ISO 21127)
- Wikidata data model; Wikibase REST API
- Getty Research Institute vocabularies — AAT, TGN, ULAN, CONA
- Iconclass — iconographic classification system
- IIIF Image / Presentation / Auth / Search APIs
- ResourceSync (NISO Z39.99-2017)
- ISO 14721 (OAIS) — Open Archival Information System
- ISO 16363 — Trustworthy Digital Repositories
- ICOM Red Lists; ICOM Code of Ethics
- UNESCO 1970 Convention; UNIDROIT 1995 Convention
- Local Contexts — TK / BC labels and notices
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)

---

## §1 Europeana / national aggregator integration

| Aggregator              | Profile                                       |
|-------------------------|-----------------------------------------------|
| Europeana               | EDM 5.2.8 + Publishing Framework               |
| National aggregator (DE)| Deutsche Digitale Bibliothek (DDB)             |
| National aggregator (FR)| API Pop / Joconde                              |
| National aggregator (UK)| Art UK                                         |
| National aggregator (KR)| Cultural Heritage Administration / e-Museum    |
| National aggregator (NL)| Netwerk Digitaal Erfgoed (NDE)                 |
| National aggregator (US)| DPLA (Digital Public Library of America)        |

Aggregator submission uses the institution's chosen
metadata profile; LIDO and EDM are the most common.
Submissions emit ResourceSync change-list updates so
aggregators see incremental changes without polling.

## §2 Linked-data and Wikidata integration

| Target                | Binding                                          |
|-----------------------|--------------------------------------------------|
| Wikidata              | object QID + statements with property mapping    |
| Linked Art bridge     | Linked Art JSON-LD per Linked Art profile        |
| Linked Open Data Cloud | RDF endpoint at the institution                  |
| GLAM / Art Linked-Data| federated query via SPARQL                       |

Wikidata integration honours community editorial
practices; the institution may publish authoritative
statements via OpenRefine or the Wikidata API.

## §3 Getty Vocabularies and Iconclass integration

| Authority           | Use                                                |
|---------------------|----------------------------------------------------|
| Getty AAT           | concept terms (object types, materials, styles)    |
| Getty TGN           | place names                                        |
| Getty ULAN          | actor names                                        |
| Getty CONA          | cultural-object names                              |
| Iconclass           | iconographic subjects                               |
| LCSH / FAST         | subject vocabulary (Library of Congress)            |
| VIAF                | actor-name authority                               |
| ISNI                | actor-name authority                               |

Authority bindings emit re-mapping events on
authority releases; the WIA record cites the
authority release identifier in force at the binding
time.

## §4 IIIF ecosystem integration

| Component / consumer  | Profile                                         |
|-----------------------|-------------------------------------------------|
| IIIF Image API 3.0    | image-tile and region serving                   |
| IIIF Presentation 3.0 | manifest-driven viewers (Mirador, Universal     |
|                       | Viewer, Clover, etc.)                            |
| IIIF Auth 2.0         | restricted-imagery access tiers                 |
| IIIF Search 1.0       | search within annotation                        |
| IIIF Change Discovery | OAI-PMH / ResourceSync alignment                |

IIIF manifests sign with the institution's key so
downstream viewers see authentic manifests.

## §5 OAIS-conformant repository integration

| Target                | Profile                                         |
|-----------------------|-------------------------------------------------|
| Trustworthy digital   | ISO 16363 audit                                 |
| repository (TDR)      |                                                 |
| Geographic redundancy | replicate AIPs across geographic regions        |
| Peer-network          | replication to peer institutions                 |
| Cloud storage         | content-addressed object store                   |

The implementation submits AIPs to the TDR per OAIS
SIP / AIP / DIP; the TDR returns an archival
identifier that the WIA record stores as
`preservationRef`.

## §6 ICOM Red List and law-enforcement integration

```
new accession → ICOM Red List screen → match-found? →
                                          │
                                          ├─ yes → law-enforcement
                                          │         notice + accession hold
                                          └─ no → proceed to committee review
```

The institution maintains liaison with applicable
law-enforcement bodies (Interpol Stolen Works of Art
database, FBI Art Crime Team, Carabinieri TPC,
national equivalents). Suspicious matches emit a
restricted-access event on the audit chain pending
investigation.

## §7 Indigenous-community engagement

Where the institution holds material connected to
indigenous or community-of-origin contexts:

```
identification → community-engagement →
  consultation-events → community-recommendations →
  classification-update / restitution-pathway
```

The Local Contexts label / notice taxonomy publishes
on the relevant object's rights record; the
recommendations bind to the access-tier policy.

## §8 Cross-domain WIA bindings

| Companion standard          | Binding purpose                                |
|-----------------------------|------------------------------------------------|
| WIA-cultural-heritage-      | source-imaging pipeline                         |
| digitization                |                                                |
| WIA-cultural-exchange-data  | cross-border lending / exhibition exchange     |
| WIA-translation-data        | multilingual-label MT pipeline                 |
| WIA-digital-time-capsule    | long-term preservation                         |
| WIA-content-ai              | AI-generated descriptive metadata governance   |
| WIA-data-portability        | metadata export to community repositories      |
| WIA-master-data-management  | actor / place / concept master                 |

Each binding identifies the consumed PHASE.

## §9 Long-term archival

| Authority / context       | Retention                                  |
|---------------------------|--------------------------------------------|
| OAIS-conformant TDR       | indefinite (per OAIS designated-community  |
|                           | preservation policy)                        |
| Conservation records      | indefinite                                  |
| Loan / exhibition records | minimum institution-policy retention        |
| Provenance records        | indefinite                                  |
| Sensitive imagery         | per community-consent withdrawal            |

Personal data of donors, lenders, conservators is
retained per applicable privacy law and contractual
terms.

## §10 Conformance test suite

The reference test suite covers:

- LIDO 1.1 round-trip on a representative object
- Linked Art JSON-LD validation
- IIIF Image API 3.0 region request
- IIIF Presentation 3.0 manifest validation
- IIIF Auth 2.0 probe service for a restricted item
- OAI-PMH harvest of a metadata-prefix
- ResourceSync change-list discovery
- PREMIS fixity-check failure → preservation-event
- LCSH / Getty AAT authority binding
- ICOM Red List screening on a synthetic accession
- restitution event recorded as provenance E10 Transfer-of-Custody

## §11 Internationalisation

Object descriptive labels and exhibition narratives
carry BCP 47 language tags; institution metadata is
exposed in the institution's primary language with
public-API fallback to a configured default. Public
APIs honour `Accept-Language`.

## §12 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for sensitive
  exchanges (loan documentation, restitution records)
- Authentication: OAuth 2 with PKCE for staff; IIIF
  Auth 2.0 for restricted-imagery access
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-collection key wrapping
- Audit: tamper-evident chain (PHASE 3 §10) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: lender / donor / conservator records
  retained under contractual and privacy-law
  obligations; community-consent restrictions honoured

## §13 Operational metrics

Sponsors / institutions report (informationally) on
the WIA registry:

- objects published vs. catalogued vs. accessioned
- provenance-gap rate
- conservation-task throughput
- IIIF traffic (pages / unique objects)
- preservation-fixity-failure rate
- restitution claims (open / resolved)
- loan throughput

## §14 Recovery and continuity

- API outage — public IIIF endpoints remain available
  via CDN cache; write endpoints retry on reconnect
- TDR outage — local SIP queue holds pending
  ingestion; replays on recovery
- aggregator outage — ResourceSync change-list buffers
  until aggregator returns
- KMS outage — sealed back-up keys per sponsor's BCP

## Annex A — Worked end-to-end example (informative)

A regional museum digitises a collection of 4,800
ethnographic objects. Capture follows the institution's
SOP; surrogates produce IIIF manifests. Descriptive
metadata authors in LIDO and Linked Art; authority
bindings link to Getty AAT and ULAN. Sensitive items
receive Local Contexts BC and TK labels through
community engagement. The institution submits to
Europeana via EDM mapping; ResourceSync change-list
keeps Europeana current. Two objects identified on the
ICOM Red List for the region trigger law-enforcement
notification; one is subsequently restituted, the
other moves to community-only access. The
preservation cycle runs weekly fixity checks on
critical originals and quarterly on standard.

## Annex B — Conformance disclosure

Implementations declare the EDM mapping version, the
LIDO / Linked Art / CIDOC-CRM versions served, the
IIIF profiles, the OAI-PMH metadata prefixes, the
authority bindings (Getty / Iconclass / Wikidata /
LCSH), and the OAIS / ISO 16363 certification status.
Disclosure is machine-readable at `/.well-known/wia-
mu-conformance.json`.

## Annex C — Versioning

Adding a new authority binding is minor; changing
the canonical metadata profile is major.

## Annex D — Provenance research data sources

| Source                        | Use                                  |
|-------------------------------|--------------------------------------|
| Getty Provenance Index        | sales catalogues, dealer stock, etc. |
| Art Loss Register             | stolen-art due-diligence              |
| Interpol Stolen Works of Art  | law-enforcement diligence             |
| FBI National Stolen Art File  | US-context diligence                  |
| Carabinieri TPC database      | Italian-context diligence             |
| German Lost Art Foundation    | Nazi-era due-diligence                |
| Holocaust-era Looted Art      | per institution                      |
| commissions                   |                                      |

Provenance research events bind the consulted source
and the search outcome (positive / negative / further
inquiry); negative outcomes do not by themselves clear
title but record diligence.
