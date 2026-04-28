# WIA-legal-system-harmonization PHASE 4 — Integration Specification

**Standard:** WIA-legal-system-harmonization
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-legal-system-
harmonization integrates with adjacent ecosystems
(official-publishing authorities, courts, treaty
depositaries, Hague Apostille e-registers, EUR-Lex,
WIPO, sovereign administrative-law systems, and
downstream WIA standards), how conformance evidence
is produced, and how deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO/IEC 27001:2022, ISO/IEC 27701:2019, ISO 17442:2020
- IETF RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)
- OASIS LegalDocML (Akoma Ntoso 1.0), OASIS LegalRuleML 1.0
- ELI / ECLI Council Conclusions
- IFLA FRBR, Dublin Core ISO 15836-1:2017
- W3C JSON-LD 1.1, W3C SHACL, W3C ODRL 2.2
- HCCH Apostille e-Register, Hague Conference

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-legal-system-harmonization
consumes upstream specifications (Akoma Ntoso, ELI,
ECLI), how downstream WIA standards reference legal
artefacts, and how conformance is assessed,
recorded, and audited.

## §2 Upstream integration

### 2.1 OASIS

Akoma Ntoso 1.0 and LegalRuleML 1.0 are consumed at
their published versions. Errata are absorbed
editorially.

### 2.2 EU institutional

ELI Council Conclusion 2012/C 325/02 and ECLI
Council Conclusion 2011/C 127/01 are the
authoritative identifier schemes for European
artefacts. National adoption variants are recorded
in the jurisdiction record (PHASE-1 §2).

### 2.3 IFLA

FRBR / FRBRoo levels (Work, Expression,
Manifestation, Item) underpin the record model.

### 2.4 ISO

ISO 17442 LEI is the canonical authority identifier.
ISO 8601 is the canonical date/time format.

### 2.5 Hague Conference (HCCH)

The Apostille e-Register specification is consumed
for international authentication of public documents.

## §3 EUR-Lex / national publication integration

National publication systems push their daily
gazette deltas into the registry via authenticated
Atom feeds. The registry mirrors EUR-Lex through a
read-only mirror to make EU acts available alongside
national counterparts. Mirror cadence is hourly for
the recent window (last 7 days) and daily for older
content.

## §4 Court system integration

Courts publish decisions with ECLI identifiers and
Akoma Ntoso markup. The registry mirrors decisions
into the citation graph, generating implicit
citations from the decision body's `<ref>` elements.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared schema versions (Akoma Ntoso, LegalRuleML,
  ELI, ECLI);
- declared authority register with LEI attestations;
- declared SKOS subject-matter scheme set;
- the test-vector matrix per Annex G of each PHASE;
- the JWS signing key set used for expressions and
  manifestations.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. Akoma Ntoso documents validate against schema 1.0.
2. LEI references resolve at GLEIF.
3. Citation walks return reproducible results for
   reference seed sets.
4. Consolidation history is replayable from the
   prior expression list.
5. ELI / ECLI resolution returns a single
   authoritative target for unambiguous queries.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                 | Integration point                       |
|--------------------------|-----------------------------------------|
| WIA-legal-knowledge      | citation graph search                   |
| WIA-policy-research      | comparative legal study                 |
| WIA-language-bridge      | official translation                    |
| WIA-pubscript            | legal publication formatting            |
| WIA-smart-contract       | regulatory machine-readable rules       |

## §7 Privacy

Personal data appearing in court decisions is
processed under the publishing court's privacy
regime. Anonymisation rules differ across
jurisdictions; the registry preserves the publisher's
form rather than re-anonymising.

## §8 Security

Authority signing key compromise triggers immediate
JWKS rotation and a public revocation list update.
Compromised manifestations are tombstoned and the
underlying work is republished under a new
`expressionRef`.

## §9 ODRL policy binding

When an expression's manifestation carries a license
or rights statement, the rights are expressed as a
W3C ODRL 2.2 policy bound to the manifestation. The
ODRL policy is informative; it does not override the
publishing authority's legal terms.

## §10 Localisation

Multilingual artefacts are catalogued as one
expression per language. The registry surfaces a
union view across languages so that researchers can
study harmonisation across linguistic versions of
the same work.

## §11 Accessibility

HTML manifestations conform to WCAG 2.2 AA. PDF
manifestations expose tagged structure (PDF/UA-1
ISO 14289-1) so that assistive technology can
navigate sections, articles, and footnotes.

## §12 Verifiable presentation of legal artefacts

Verifiable presentations of legal artefacts (e.g.
proof of an apostille) follow W3C VC 2.0 patterns.
The presentation links the apostille e-register
record, the underlying manifestation, and the
verifying authority's signature.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence URL
from their landing page and from the README under a
`## Conformance` heading.

## Annex B — Worked harmonisation map (informative)

```json
{
  "harmonisationRef": "/v1/harmonisation/gdpr-uk-dpa",
  "srcExpressionRef": "/eli/eu/dir/2016/679/oj/2018-05-25",
  "tgtExpressionRef": "/eli/uk/ukpga/2018/12",
  "mappingKind": "partial",
  "conceptScheme": "EuroVoc",
  "provenance": "529900T8BM49AURSDO55",
  "effectiveAt": "2020-12-31"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with Akoma Ntoso, ELI, ECLI revisions.

## Annex D — Open governance

Issues and proposals are tracked at
`github.com/WIA-Official/wia-standards/issues` with
the `legal-system-harmonization` label.

## Annex E — Withdrawal procedure

A deployment withdraws conformance by tombstoning
its evidence package. Tombstones are immutable.

## Annex F — Reproducibility

Evidence is reproducible from publicly available
inputs: authority register, LEI attestations, ELI /
ECLI resolution traces, citation graph snapshots,
and the registry's signing key set.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative vector
under `tests/phase-vectors/`.

## Annex H — National adoption catalogue

| Region | Authority                          | Standards in use         |
|--------|------------------------------------|--------------------------|
| EU     | Publications Office (EUR-Lex)      | ELI, ECLI, Akoma Ntoso   |
| UK     | TNA (legislation.gov.uk)           | CLML, Akoma Ntoso        |
| US     | OFR (govinfo)                      | USLM, Akoma Ntoso (pilot)|
| KR     | Ministry of Government Legislation | Akoma Ntoso (pilot)      |
| JP     | E-Gov                              | LAWS XML                 |
| BR     | Senado / LexML                     | LexML / Akoma Ntoso       |

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Authority key compromise              | Immediate JWKS rotation   |
| Akoma Ntoso schema drift              | Vector matrix gate        |
| Citation walk loop                    | 1024-hop cap              |
| LEI revocation                        | Authority record amend    |
| Apostille e-register downtime         | Cached binding fallback   |
| Withdrawal of treaty ratification     | Atom feed event           |

## Annex J — Sustainability

Registry deployments SHOULD declare their estimated
energy cost per published expression and per
resolved ELI / ECLI request. Declarations are
informative.

## Annex K — Researcher access programme

Comparative-law researchers request access to
restricted artefacts (e.g. unredacted court
dossiers, treaty negotiation drafts) via the
registry's researcher access programme. Each
request is reviewed by an authority-appointed
gatekeeper and a documented data-use agreement.

## Annex L — Continuous improvement programme

Each conformant deployment publishes an annual
improvement plan addressing schema drift, citation
gaps, harmonisation map stale-rate, and
accessibility regressions.

## Annex M — Multi-jurisdiction citation crosswalk

A citation that spans jurisdictions (e.g. an EU
directive cited by a UK statute) is surfaced both
in the source registry's citation graph and in any
peering registry that subscribes to the relevant
Atom feed. The crosswalk preserves the source
authority's declaration; the destination registry
adds derivative citations only as informational
overlays.

## Annex N — Long-term archival preservation

Manifestations are archived in a digital
preservation system (LOCKSS, Portico, or sovereign
national archives) with at least three geographic
copies. The archive records the SHA-512 digest, the
publishing authority's signature, and the archival
event timestamp.

## Annex O — Open data license

Where the publishing jurisdiction permits, the
registry expresses the artefact's license as a W3C
ODRL 2.2 policy. Common licenses (CC0, CC-BY,
Open Government License v3) are pre-cached as
named ODRL profiles.

## Annex P — Citation provenance chain

Each citation carries a provenance chain: the
publisher of the citing artefact, the date the
citation was first asserted, and any amendments to
the citation since. The chain is signed by each
authority that touches the citation so that
downstream consumers can verify the chain end-to-
end.

## Annex Q — Common Cartography for civil-common

A civil-law expression frequently lacks a direct
common-law analogue. The registry catalogues
common-cartography mappings under
`/v1/registry/cartography` so that researchers can
navigate concept gaps without misrepresenting
equivalence. Cartography entries carry
`mappingKind: cartographic` to distinguish them
from binding harmonisation maps.

## Annex R — Sovereign immunity and procedural overlay

Some artefacts (military justice acts, intelligence
oversight rulings) are subject to sovereign-immunity
overlays. The registry permits the publishing
authority to mark an artefact `restrictedAccess`
with a public summary while keeping the body
behind authentication. Auditors verify the
restriction's lawful basis but do not override the
publishing authority's decision.

## Annex S — Citizen information services

National citizen-information services (e.g.
data.gov.uk, EUR-Lex SUMMARY, BÉN public-info
portals) integrate via Atom feeds and a SHACL
filter declaring the topical scope of interest.
The registry surfaces aggregate counters of which
artefacts are republished by which services so that
publishing authorities can monitor reach.

弘益人間 (Hongik Ingan) — Benefit All Humanity
