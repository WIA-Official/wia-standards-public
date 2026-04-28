# WIA-museum-digital-archive PHASE 1 — Data Format Specification

**Standard:** WIA-museum-digital-archive
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
museum digital-archive operations covering object
records, accession and provenance histories,
conservation reports, exhibition and loan events,
imagery and 3D / multimedia surrogates, multilingual
descriptive metadata, rights / licence bindings, and
preservation events. The format aligns with CIDOC-CRM,
LIDO, Linked Art, Spectrum 5.1 collections-management
procedures, OAIS / ISO 14721 preservation framework,
IIIF for image / presentation / search APIs, Europeana
EDM, and the W3C Web Annotation Data Model.

References (CITATION-POLICY ALLOW only):
- CIDOC-CRM (ISO 21127:2014) — A reference ontology for cultural-heritage data
- ISO 21127 — Information and documentation: a reference ontology
- LIDO XML 1.1 — Lightweight Information Describing Objects
- Linked Art (community reference) — Linked-data profile of CIDOC-CRM
- Dublin Core Metadata Initiative (DCMI Terms 1.1)
- Europeana EDM 5.2.8 — Europeana Data Model
- ICOM Code of Ethics for Museums; ICOM Object ID
- Spectrum 5.1 — Collections-Trust collections-management procedures
- ISO 14721:2012 — OAIS — Open Archival Information System
- ISO 16363:2012 — Trustworthy Digital Repositories
- IIIF Image API 3.0; IIIF Presentation API 3.0; IIIF Auth 2.0; IIIF Search API 1.0
- W3C Web Annotation Data Model
- W3C Verifiable Credentials Data Model 2.0 (where loan / endorsement attestations issue)
- ISO 19115-2:2019 — geographic-information metadata (geo-bound objects)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 7515 (JWS)
- PREMIS 3.0 (preservation metadata; Library of Congress)
- METS 1.12 (Metadata Encoding and Transmission Standard)
- TEI Lite (text encoding initiative for transcribed manuscripts / archives)
- ICOM Red Lists (illicit-trafficking risk categories)

---

## §1 Scope

This PHASE applies to museums, libraries, archives,
and cultural-heritage institutions managing digital
records of physical or born-digital objects (artworks,
artefacts, specimens, documents, manuscripts,
recordings, photographs, films, performance video, and
born-digital art).

In scope: object record, accession record, provenance
record, conservation record, exhibition record, loan
record, imagery / surrogate record, descriptive-
metadata record, multilingual-label record, rights /
licence record, preservation-event record, and the
cross-references binding records to public-presentation
manifests, scholarly catalogues, and external
authorities. Out of scope: contemporary commercial
publishing of cultural goods (handled by the
content-distribution standard) and live-performance
ticketing (handled by the ticketing-system standard).

## §2 Object record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `objectRef`          | UUID (RFC 4122)                                 |
| `localIdentifier`    | institution-local accession number (Spectrum)   |
| `type`               | controlled term (CIDOC-CRM E22 Man-Made-Object, |
|                      | E20 Biological-Object, E84 Information-Carrier) |
| `classification`     | per-collection taxonomy (e.g. Getty AAT)         |
| `culturalContext`    | era / period (Getty TGN / AAT references)        |
| `materials`          | controlled list (Getty AAT materials)            |
| `dimensions`         | per-axis with unit (CIDOC-CRM E54)               |
| `inscriptions`       | text + script + position                          |
| `creator`            | actor-reference (CIDOC-CRM E21 Person /          |
|                      | E74 Group); multiple actors allowed              |
| `dateCreated`        | bound to a CIDOC-CRM E52 Time-Span                |
| `subject`            | per-collection subject taxonomy (Getty AAT,      |
|                      | LCSH, Iconclass)                                  |
| `descriptiveMetadata`| free-text bound to multilingual-label record    |
| `accessionRef`       | accession event (this PHASE §3)                   |
| `currentLocationRef` | current physical / digital location              |

Born-digital objects record the technical-environment
(file format, software / hardware platform) per the
PREMIS environment-description.

## §3 Accession record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `accessionRef`       | UUID                                            |
| `objectRef`          | §2                                              |
| `mode`               | `purchase`, `gift`, `bequest`, `transfer`,      |
|                      | `loan`, `find`, `commission`                    |
| `accessionedAt`      | ISO 8601                                        |
| `donorRef`           | actor-reference                                  |
| `legalBasis`         | sale-contract / gift-letter / bequest-record /   |
|                      | repatriation-treaty                              |
| `acquisitionPolicyRef`| institution policy applied                      |
| `dueDiligenceRef`    | provenance / illicit-trade due-diligence record  |

Due-diligence references the ICOM Red Lists and
relevant national / international cultural-property
laws (e.g. UNESCO 1970 Convention; UNIDROIT 1995
Convention).

## §4 Provenance record

Provenance follows CIDOC-CRM E10 Transfer-of-Custody
and E8 Acquisition events:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `provenanceRef`      | UUID                                            |
| `objectRef`          | §2                                              |
| `eventKind`          | `creation`, `change-of-ownership`,               |
|                      | `change-of-custody`, `loss`, `recovery`         |
| `priorOwnerRef`      | actor-reference                                  |
| `subsequentOwnerRef` | actor-reference                                  |
| `place`              | Getty TGN reference                              |
| `eventTime`          | ISO 8601 / E52 Time-Span                         |
| `documentRef`        | source-document reference (catalogue, sale       |
|                      | record, archival ledger)                         |

## §5 Conservation record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `conservationRef`    | UUID                                            |
| `objectRef`          | §2                                              |
| `treatmentKind`      | `examination`, `cleaning`, `consolidation`,     |
|                      | `restoration`, `reconstruction`, `mount-rebuild`|
| `conservatorRef`     | conservator identity (with credential)           |
| `treatmentTime`      | ISO 8601                                        |
| `materialsUsed`      | per Getty AAT or institution catalogue           |
| `conditionBefore`    | structured condition assessment                  |
| `conditionAfter`     | structured condition assessment                  |
| `imageryRef[]`       | before / during / after surrogates              |

Conservation entries are append-only; the object's
condition timeline reconstructs from the chain.

## §6 Exhibition / loan record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `exhibitionRef`      | UUID                                            |
| `title`              | localised label                                  |
| `venue`              | institution / temporary venue                     |
| `startsAt`           | ISO 8601                                        |
| `endsAt`             | ISO 8601                                        |
| `objects[]`          | object-references displayed                      |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `loanRef`            | UUID                                            |
| `objectRef`          | §2                                              |
| `borrowerRef`        | borrowing institution                             |
| `loanPurpose`        | exhibition / research / conservation / teaching  |
| `loanStart`          | ISO 8601                                        |
| `loanEnd`            | ISO 8601                                        |
| `insuranceRef`       | bound insurance policy                           |
| `couriersRef[]`      | courier(s) accompanying the object               |
| `conditionReports[]` | pre / post loan condition reports                |

## §7 Imagery / surrogate record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `surrogateRef`       | UUID                                            |
| `objectRef`          | §2                                              |
| `kind`               | `still-image`, `video`, `audio`, `3d-mesh`,     |
|                      | `3d-pointcloud`, `gigapixel-zoom`, `spectral`,  |
|                      | `xrf`, `multispectral`, `tomography`            |
| `mediaUri`           | content-addressed URI                            |
| `iiifManifest`       | IIIF Presentation 3.0 manifest URI               |
| `mimeType`           | per IANA registry                               |
| `colorProfile`       | ICC profile reference                            |
| `dpi`                | for raster                                      |
| `dimensions`         | pixels / vertices / sample-rate                  |
| `captureTime`        | ISO 8601                                        |
| `captureMetadata`    | EXIF + camera / scanner profile                  |
| `licenceRef`         | rights / licence record (§9)                     |

## §8 Descriptive-metadata and multilingual-label record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `metadataRef`        | UUID                                            |
| `objectRef`          | §2                                              |
| `lidoXml`            | LIDO 1.1 record URI (where bound)                |
| `linkedArt`          | Linked Art JSON-LD URI                           |
| `cidocCrmGraph`      | CIDOC-CRM RDF graph URI                          |
| `dcTerms`            | Dublin Core triples                              |
| `tei`                | TEI Lite reference (for transcribed text)        |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `labelRef`           | UUID                                            |
| `objectRef`          | §2                                              |
| `language`           | BCP 47 tag                                       |
| `audience`           | `gallery-public`, `scholarly`, `accessible-easy-|
|                      | language`, `audio-description`, `sign-language` |
| `text`               | localised text                                   |
| `signLanguageMedia`  | sign-language-video reference                    |

## §9 Rights / licence record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `rightsRef`          | UUID                                            |
| `objectRef`          | §2                                              |
| `copyrightStatus`    | per Europeana Rights Statements (in-copyright,  |
|                      | out-of-copyright, no-known-copyright,            |
|                      | underlying-works-only)                           |
| `licence`            | SPDX or Europeana RS or local custom URI         |
| `restrictions`       | per-institution restriction policy               |
| `traditionalCulturalNotice`| Local Contexts label / notice (where         |
|                      | indigenous communities engaged)                  |

For sensitive material (sacred items, ancestral
remains, secret-sacred objects per ICOM ethics) the
restriction policy may limit imagery access or
imagery distribution.

## §10 Preservation-event record (PREMIS)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `preservationEventRef`| UUID                                            |
| `objectRef`          | §2                                              |
| `eventType`          | per PREMIS event vocabulary (ingestion,         |
|                      | fixity-check, migration, normalisation,         |
|                      | replication, deletion, validation)               |
| `eventTime`          | ISO 8601                                        |
| `agentRef`           | preservation system / archivist                  |
| `outcome`            | `success`, `failure`, `warning`                  |
| `fixityValueRef`     | content-addressed digest (SHA-256, BLAKE3)       |

## §11 Cross-domain references (informative)

- WIA-cultural-heritage-digitization — source-imaging
  pipeline
- WIA-cultural-exchange-data — international exchange
- WIA-translation-data — multilingual-label MT pipeline
- WIA-digital-time-capsule — long-term preservation

## Annex A — Worked Linked-Art binding (informative)

```json
{
  "@context": "https://linked.art/ns/v1/linked-art.json",
  "id": "https://museum.example/object/123",
  "type": "HumanMadeObject",
  "classified_as": [{"id":"http://vocab.getty.edu/aat/300033973","type":"Type","_label":"painting"}],
  "made_of": [{"id":"http://vocab.getty.edu/aat/300015012","_label":"oil paint"}],
  "produced_by": {"type":"Production","carried_out_by":[{"id":"https://museum.example/actor/57","_label":"Artist"}]}
}
```

## Annex B — Conformance disclosure

Implementations declare the LIDO / Linked Art / CIDOC-
CRM versions served, the IIIF profile (Image / Pres /
Auth / Search) versions exposed, the PREMIS / METS
revisions, and the Europeana EDM mapping version.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.
