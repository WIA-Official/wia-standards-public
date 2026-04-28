# WIA-legal-system-harmonization PHASE 1 — Data Format Specification

**Standard:** WIA-legal-system-harmonization
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-legal-system-harmonization, the cross-jurisdiction
legal document and case-law interoperability
standard. The records bind every legislative act,
court decision, regulatory rule, treaty, and
authoritative legal commentary to a documented
identifier scheme, a structural markup model, and a
provenance trail so that downstream registries,
courts, regulators, and citizen-information services
can resolve, cite, compare, and consume legal
artefacts across jurisdictions.

References (CITATION-POLICY ALLOW only):
- OASIS LegalDocML (Akoma Ntoso 1.0)
- OASIS LegalRuleML 1.0
- ELI (European Legislation Identifier) per Council Conclusion 2012/C 325/02
- ECLI (European Case Law Identifier) per Council Conclusion 2011/C 127/01
- IFLA FRBR / FRBRoo (functional requirements for bibliographic records)
- IFLA LRMoo, CIDOC CRM (cultural-heritage upper ontology)
- ISO 17442:2020 (Legal Entity Identifier — LEI)
- ISO 8601 (Date and time), ISO 3166-1 alpha-2
- Dublin Core Metadata Element Set (ISO 15836-1:2017)
- W3C RDF 1.1, RDF/XML, JSON-LD 1.1, SKOS, SHACL
- W3C ODRL (Open Digital Rights Language) 2.2
- W3C Web Annotation Model
- Hague Conference on Private International Law (HCCH) Apostille e-Register specification
- UN/CEFACT Cross-Border Trade Facilitation Reference Data Model (informative)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 7515 (JWS), RFC 4122 (UUID)
- IETF RFC 5005 (Atom paged feeds), RFC 9421 (HTTP Message Signatures)

---

## §1 Scope

This PHASE applies to records that describe a legal
artefact — primary legislation (constitutions, codes,
statutes, regulations), secondary legislation (orders,
directives, ordinances), court decisions, treaties
and conventions, administrative rulings, official
gazettes — and the cross-jurisdiction relationships
that bind them.

In scope: jurisdiction record, work record, expression
record, manifestation record, item record (FRBR
levels), authority record, citation record, treaty
record, harmonisation map record, and the cross-
references binding every artefact to its language
versions, amendment history, and authoritative
publisher.

Out of scope: substantive legal interpretation
(handled by sovereign courts and authorised
commentators); contract document automation (handled
by WIA-smart-contract); proprietary legal-research
products (commercial layer above this PHASE).

## §2 Jurisdiction record

Every artefact carries:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `jurisdictionRef`    | URI; per ELI: `eli/<country>` or extended       |
|                      | for sub-national entities                        |
| `country`            | ISO 3166-1 alpha-2                              |
| `subdivision`        | ISO 3166-2 if applicable                        |
| `legalSystem`        | civil, common, religious, mixed, customary,     |
|                      | hybrid                                          |
| `officialLanguage[]` | BCP 47 tags                                     |
| `treatyMember[]`     | LEI references for international organisations  |
|                      | (e.g. WTO, WIPO, Hague Conference)              |
| `apostilleAuthority` | URI to the Hague Apostille issuing authority    |

## §3 Work record (FRBR Level 1)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `workRef`            | URI per ELI / Akoma Ntoso `FRBRWork`            |
| `actType`            | `constitution`, `code`, `statute`, `regulation`,|
|                      | `directive`, `decree`, `ordinance`,             |
|                      | `court-decision`, `treaty`, `convention`,       |
|                      | `administrative-ruling`                          |
| `enactingAuthority`  | LEI of the enacting body                        |
| `subjectMatter[]`    | EuroVoc descriptors or sovereign equivalent     |
| `numberOfficial`     | the official number assigned by the publisher  |
| `dateAdopted`        | ISO 8601                                        |

A work is invariant under amendments; amendments
produce new expression records (this PHASE §4) but
keep `workRef` stable.

## §4 Expression record (FRBR Level 2)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `expressionRef`      | URI per ELI / Akoma Ntoso `FRBRExpression`      |
| `workRef`            | this PHASE §3                                   |
| `language`           | BCP 47 tag                                      |
| `validFrom`          | ISO 8601 (entry into force)                     |
| `validUntil`         | ISO 8601 (consolidation cut-off, if any)        |
| `state`              | `proposed`, `enacted`, `consolidated`,          |
|                      | `repealed`, `withdrawn`                         |
| `consolidationOf[]`  | prior expressionRef list when this expression   |
|                      | is a consolidated text                          |
| `amendsRef[]`        | expressionRef list this expression amends       |

Multiple language expressions of the same work share
`workRef`; consolidation is a derived expression that
references the prior expressions it consolidates.

## §5 Manifestation record (FRBR Level 3)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `manifestationRef`   | URI per ELI / Akoma Ntoso `FRBRManifestation`   |
| `expressionRef`      | this PHASE §4                                   |
| `format`             | `akn`, `pdf`, `html`, `epub`, `xml`, `rtf`      |
| `producedBy`         | LEI of the publisher                            |
| `producedAt`         | ISO 8601                                        |
| `digestRef`          | SHA-512 of the byte content                     |
| `signatureRef`       | URI to detached JWS over the canonical content  |

## §6 Item record (FRBR Level 4)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `itemRef`            | URI per ELI / Akoma Ntoso `FRBRItem`            |
| `manifestationRef`   | this PHASE §5                                   |
| `repository`         | URI of the holding repository                   |
| `accessPolicy`       | `open`, `restricted`, `paywalled`, `archived`   |
| `apostilleRef`       | optional Hague Apostille e-register reference   |

## §7 Authority record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `authorityRef`       | URI                                             |
| `lei`                | ISO 17442 LEI                                   |
| `name`               | localised authority name (BCP 47 keys)          |
| `kind`               | `legislature`, `court`, `executive`,            |
|                      | `regulatory-agency`, `treaty-body`              |
| `parent`             | optional parent authority for nested bodies     |
| `address`            | postal contact                                  |
| `signingKeyRef`      | JWKS URL for verifiable publications            |

## §8 Citation record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `citationRef`        | URI                                             |
| `sourceWorkRef`      | citing work or expression                       |
| `targetWorkRef`      | cited work or expression                        |
| `targetSection`      | Akoma Ntoso eId path (e.g. `art_5__para_2`)     |
| `relationKind`       | `amends`, `repeals`, `derogates`,               |
|                      | `cites-as-authority`, `interprets`,             |
|                      | `transposes`, `ratifies`                        |
| `pinpointDate`       | the ISO 8601 date the relation took effect      |

For court decisions, `targetWorkRef` MAY be an ECLI
identifier; the registry expands ECLI to the
corresponding Akoma Ntoso work URI when known.

## §9 Treaty record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `treatyRef`          | URI                                             |
| `name`               | official name (multilingual)                    |
| `parties[]`          | LEI references for state parties                |
| `signedAt`           | ISO 8601                                        |
| `enteredInForceAt`   | ISO 8601                                        |
| `depositary`         | LEI of the treaty depositary                    |
| `umbrellaConventions[]` | parent conventions if applicable             |
| `ratifications[]`    | per-party ratification records (date, scope,    |
|                      | reservations, declarations)                     |

## §10 Harmonisation map record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `harmonisationRef`   | URI                                             |
| `srcExpressionRef`   | expressionRef in source jurisdiction            |
| `tgtExpressionRef`   | expressionRef in target jurisdiction            |
| `mappingKind`        | `equivalent`, `narrower`, `broader`,            |
|                      | `partial`, `incompatible`                       |
| `conceptScheme`      | SKOS scheme (EuroVoc, FAOLEX, UNTERM, JuriCAF)  |
| `provenance`         | LEI of the harmonisation publisher              |
| `effectiveAt`        | ISO 8601 — when the mapping took effect         |

Harmonisation maps are informative; they do not
override the binding interpretation by sovereign
courts.

## §11 Cross-domain references (informative)

- WIA-legal-knowledge — case-law search
- WIA-policy-research — comparative legal study
- WIA-language-bridge — official translation flow
- WIA-pubscript — publication-time formatting

## Annex A — Conformance disclosure

Implementations declare the schema versions they
accept (Akoma Ntoso, ELI, ECLI, LegalRuleML), the
canonicalisation form (RFC 8785), and the JWS key
set used to sign work / expression / manifestation
records.

## Annex B — Akoma Ntoso eId addressing

Pinpoint references inside an expression use the
Akoma Ntoso `eId` attribute. Examples:

| eId path                                | Reference                                |
|-----------------------------------------|------------------------------------------|
| `chp_3__sec_12`                         | Chapter 3, Section 12                    |
| `art_5__para_2__lst_a`                  | Article 5, Paragraph 2, List item (a)    |
| `body__para_3`                          | Decision body, Paragraph 3               |
| `pmbl__lst_4`                           | Preamble, Recital 4                      |

## Annex C — Worked expression record (informative)

```json
{
  "expressionRef": "/eli/eu/dir/2016/679/oj",
  "workRef": "/eli/eu/dir/2016/679",
  "language": "en",
  "validFrom": "2018-05-25",
  "state": "consolidated",
  "consolidationOf": [
    "/eli/eu/dir/2016/679/oj/2016-04-27"
  ]
}
```

## Annex D — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with Akoma Ntoso, ELI, and ECLI
revisions.

## Annex E — Conformance level

Conformance is "Core" (jurisdiction + work +
expression + manifestation + item + authority +
citation) or "Full" (adds treaty and harmonisation
map records).

## Annex F — LegalRuleML binding

When normative content is exported as LegalRuleML
1.0 rules, each rule references its source
expression at the Akoma Ntoso eId level. Rule
provenance is signed jointly by the LegalRuleML
publisher and the source authority.

## Annex G — Privacy and personal data

Personal data appearing in legal artefacts (party
names, witness identifiers, judicial dossier IDs)
is processed under the publishing jurisdiction's
privacy regime (GDPR, CCPA, K-PIPA, LGPD). Aliasing
or pseudonymisation is the publisher's responsibility.

## Annex H — Multilingual concept schemes

The registry tracks SKOS concept schemes for legal
subject-matter (EuroVoc, FAOLEX, UNTERM JuriCAF,
NomCom). Each scheme version is recorded with a
SHA-512 digest so that historical mappings remain
reproducible.

## Annex I — Court hierarchy record

A jurisdiction's court hierarchy is recorded as a
parent-child graph with the apex court at the root.
Each node carries:

- the court's LEI;
- jurisdiction subdivision (ISO 3166-2);
- competence catalogue (`civil`, `criminal`,
  `administrative`, `constitutional`, `family`,
  `military`, `arbitral`);
- appeals route to the parent.

The hierarchy supports citation walks ascending
through the appeals route.

## Annex J — Reservations and declarations

Treaty ratifications may carry reservations
(narrowing the obligation) or declarations
(interpretation by the ratifying party). Each
reservation / declaration is a separate record:

| Field         | Source / Binding                             |
|---------------|----------------------------------------------|
| `kind`        | `reservation`, `declaration`                  |
| `articleRef`  | the treaty article reserved or interpreted   |
| `text`        | full text in the depositary's language       |
| `madeAt`      | ISO 8601                                     |
| `withdrawnAt` | optional ISO 8601                            |

## Annex K — Cross-domain authority overlap

When a single body acts in multiple capacities
(e.g. an executive that also issues binding
regulations), the authority record carries
multiple `kind` values. The registry surfaces the
overlap so that citation walks correctly attribute
publications to the relevant capacity.

弘益人間 (Hongik Ingan) — Benefit All Humanity
