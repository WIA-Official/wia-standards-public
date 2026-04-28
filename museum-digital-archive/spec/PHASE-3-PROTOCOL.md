# WIA-museum-digital-archive PHASE 3 — Protocol Specification

**Standard:** WIA-museum-digital-archive
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
records and API resources into auditable lifecycles:
accession workflow with due diligence, conservation
schedule, exhibition / loan operating discipline,
imagery / surrogate authoring with rights enforcement,
preservation cycle (OAIS / PREMIS), descriptive-
metadata QA, multilingual-label QA, and the audit-
event chain. The protocols are framed so an ICOM
ethics audit, a national heritage authority's
inspection, or a trustworthy-digital-repository (ISO
16363) certification can reconstruct any object from
the event log.

References (CITATION-POLICY ALLOW only):
- ICOM Code of Ethics for Museums (current edition)
- ICOM Code of Ethics for Natural-History Museums
- UNESCO 1970 Convention on the Means of Prohibiting and Preventing the Illicit Import, Export and Transfer of Ownership of Cultural Property
- UNIDROIT 1995 Convention on Stolen or Illegally Exported Cultural Objects
- ISO 21127 (CIDOC-CRM) — reference ontology
- ISO 14721 (OAIS) — Open Archival Information System
- ISO 16363 — Trustworthy Digital Repositories
- Spectrum 5.1 — collections-management procedures
- LIDO 1.1; Linked Art; PREMIS 3.0; METS 1.12
- IIIF Image / Presentation / Auth / Search APIs
- ISO/IEC 27037 — digital evidence preservation
- IETF RFC 5424 (Syslog), RFC 7515 (JWS), RFC 8785 (JCS)

---

## §1 Accession workflow

```
proposed → due-diligence → committee-reviewed → accessioned →
  catalogued → published
                                  │
                                  └→ rejected → returned-to-source
```

Due-diligence checks include UNESCO 1970 / UNIDROIT
1995 compliance, the ICOM Red List risk categorisation,
provenance research consistent with Spectrum 5.1, and
sale / gift / bequest documentation.

## §2 Provenance discipline

Provenance is an append-only chain of E10 / E8 events
per CIDOC-CRM. Gaps are recorded with explicit
acknowledgement; an unresolved provenance gap raises a
stewardship task and may delay public publication.

## §3 Conservation schedule

| Cadence kind        | Cadence                                       |
|---------------------|-----------------------------------------------|
| Routine inspection  | per institutional schedule (typ. annually for |
|                     | sensitive media; multi-year for stable media) |
| Pre-loan inspection | mandatory before any loan                     |
| Post-loan inspection| mandatory at return                           |
| Disaster-response   | event-driven (fire / flood / pest)            |

Conservator-credential is recorded; treatments without
an active credential are rejected.

## §4 Exhibition / loan operating discipline

```
loan: requested → reviewed → approved → packed → in-transit →
      installed → de-installed → returned-in-transit → received
                       │
                       └→ damaged → conservation-task
```

Loan reviews check the borrower's facilities-report,
courier credentials, insurance coverage, and the
object's stability in a touring environment. The
condition reports at packing and installation form the
contractual baseline for damage attribution.

## §5 Surrogate authoring

```
capture-planned → captured → quality-checked →
  metadata-bound → rights-classified → published / restricted
```

Capture metadata records the device, lighting, colour
target, and operator. Quality-check protocols include:
colour-target verification (per the institution's
colour-management SOP), focus / sharpness, exposure
range, geometric correction, and ICC profile binding.

## §6 Preservation cycle (OAIS / PREMIS)

```
ingest (SIP) → quality-control → ingest-confirmed →
  AIP-stored → fixity-monitored → migration / normalisation →
  dissemination (DIP)
```

Fixity-check cadence is risk-based:

| Risk tier        | Cadence                                       |
|------------------|-----------------------------------------------|
| Critical (signed  | weekly                                        |
| originals)       |                                               |
| High             | monthly                                       |
| Standard         | quarterly                                     |
| Low / large      | annually                                      |

Fixity failure raises a preservation-event with
outcome `failure`; recovery from replicas (with
chain-of-custody recorded) restores the canonical
copy.

## §7 Descriptive-metadata QA

| Check                      | Discipline                              |
|----------------------------|-----------------------------------------|
| Required fields per LIDO   | accession-number, type, title, creator   |
| Authority binding           | Getty AAT / TGN / ULAN linkage           |
| CIDOC-CRM consistency      | E22 / E84 type alignment                  |
| Language coverage          | minimum gallery-public label per           |
|                            | institutional default + accessible label  |
| Iconographic vocabulary    | Iconclass mapping where applicable        |

Validation runs on metadata write; violations gate
publication.

## §8 Multilingual-label QA

Per the institution's audience-tier policy:

| Tier                    | QA discipline                              |
|-------------------------|--------------------------------------------|
| Gallery-public          | curator review + readability metric        |
| Scholarly catalogue     | peer review per institution                |
| Accessible easy-language| communications-team review                  |
| Audio-description       | trained-describer review per accessibility |
|                         | guidelines                                 |
| Sign-language           | native-signer review                       |
| Machine-translated      | MT-with-human-post-edit; flag MT-only      |
|                         | warning to public consumers                |

Multilingual labels are versioned; corrections emit
audit events.

## §9 Rights / sensitivity enforcement

```
rights-recorded → access-tier-determined → published / restricted
                                                      │
                                                      └→ committee-review
                                                         (community / curator)
```

Sensitive material (sacred-secret, indigenous-
ancestral remains, depictions causing distress)
follows local communities' wishes; access tiers
include `public`, `community-only`, `restricted-
research`, and `not-publishable`. Indigenous-
community engagement records the consultation.

## §10 Audit event chain

| Field          | Meaning                                                 |
|----------------|---------------------------------------------------------|
| `eventId`      | UUID                                                    |
| `eventTime`    | ISO 8601 with timezone                                  |
| `actor`        | identity (curator / conservator / archivist / lender)   |
| `resourceRef`  | URI of the resource that changed                        |
| `action`       | accessioned / loaned / conserved / published / migrated |
| `priorHash`    | SHA-256 of the prior event payload                      |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)|

## §11 Repatriation and restitution flow

Per UNESCO 1970 / UNIDROIT 1995 and applicable
national laws, restitution flows:

```
claim-filed → evaluated → committee-reviewed → decision →
  decision-implemented (return / agreed-keep / mediated outcome)
```

The institution's audit chain records each step; the
return event is itself a provenance E10 Transfer-of-
Custody.

## §12 Reproducibility

A surrogate-publication is `reproducible-strong` when
capture parameters, colour profile, processing
pipeline (with container digest), and rights
classification are content-addressed; `weak` when any
is absent.

## Annex A — Worked accession example (informative)

A modern-art bequest comprising 47 paintings,
sketches, and personal papers enters the institution
under §1. Due-diligence research uses Getty Provenance
Index and the Art Loss Register; six items show
provenance gaps in the 1933-1945 period and move to
extended-due-diligence (Spoliation Advisory Panel
involvement). After 18 months of research, four items
are deemed clean and accessioned; two items resolve
through a restitution agreement with the original
owner's heirs. The accession event chain records each
step.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema
version, the JWS algorithm registry, the OAIS / PREMIS
revisions implemented, the IIIF profiles served, the
ICOM Code of Ethics edition followed, and the ISO
16363 certification status.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, KRISS, PTB).

## Annex E — Operator-credential binding

| Credential                | Source                              |
|---------------------------|-------------------------------------|
| Curator                   | institutional appointment           |
| Conservator               | per professional body (ICOM-CC,     |
|                           | AIC, IIC)                           |
| Registrar / collections    | institutional + RCAAA / equivalent  |
| Archivist                 | institutional + ICA / SAA           |
| IIIF / digital-asset       | sponsor                             |
| operator                  |                                     |
| Indigenous-community      | community-recognised                |
| representative            |                                     |

A signing event by an operator without an active
credential rejects.

## Annex F — De-accessioning protocol

Per ICOM Code of Ethics §2.13-2.16 and Spectrum 5.1
de-accessioning is a controlled procedure:

```
proposed → reviewed (committee) → community-consultation →
  approved → executed → recorded
```

De-accessioning artefacts include the rationale, the
mode (transfer / sale / destruction / repatriation),
the approving committee minutes, and the post-de-
accession provenance event. De-accessioning by sale
follows the institution's collection-trust principle
that proceeds reinvest in collection-care.

## Annex G — Inspector replay payload

For an audit (institutional, accreditation, or trust-
repository certification) the protocol exposes a
replay payload covering:

- accession + provenance chain
- conservation timeline
- exhibition / loan history
- imagery / surrogate manifests with content digests
- preservation cycle events
- rights / access-tier history
- audit-chain export

The payload signs with the institution's audit-chain
JWS key so the inspector verifies integrity end-to-end
without trusting the API operator at runtime.