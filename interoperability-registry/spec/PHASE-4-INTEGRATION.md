# WIA-interoperability-registry PHASE 4 — INTEGRATION Specification

**Standard:** WIA-interoperability-registry
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an interoperability-registry operator
integrates with the systems that surround it: source code-system
publishers (SNOMED International, Regenstrief LOINC, NLM RxNorm,
WHO ICD-11, NLM UMLS), peer registries in cross-registry
federations, runtime adapter binders that consume the registry
at boot, integration-engineering platforms (Talend, Apache
Camel, Mule, custom integration-platform-as-a-service),
governance review platforms, and long-term archives that hold
registry snapshots beyond programme wind-down.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421 / 6920
- IETF RFC 8785 (JSON Canonicalization Scheme)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- ISO/IEC 11179 (metadata registries)
- W3C SKOS / OWL 2 / SHACL
- W3C Verifiable Credentials Data Model 2.0 (optional)
- OASIS ebXML Registry Information Model 3.0

---

## §1 Source Code-System Publisher Integration

The registry's value sets (PHASE-1 §5) draw from authoritative
code systems published by external custodians. Integration with
each custodian carries:

- the custodian's identifier (e.g. SNOMED International for
  SNOMED CT, Regenstrief Institute for LOINC, NLM for RxNorm);
- the custodian's release schedule (typically biannual for
  SNOMED CT International Edition, semi-annual for LOINC,
  monthly for RxNorm);
- the operator's licence reference where the custodian
  requires licensing;
- the value-set expansion-refresh cadence the operator
  applies on the custodian's release.

Custodian releases that materially change a code's status
(retirement, re-mapping) emit a value-set expansion-refresh
event (PHASE-3 §4) that downstream binding consumers receive
through the streaming subscription.

## §2 Peer-Registry Federation Integration

Registry federation members (PHASE-3 §8) integrate through a
mutually-authenticated TLS channel with the federation's
shared discovery and authorisation rules. Integration carries
each peer's identifier, the per-class trust matrix (which
peer's artefacts are trusted under which registration
status), and the federation's joint naming-authority committee
reference for cross-registry conflict adjudication (PHASE-3
§6).

## §3 Runtime Adapter Binder Integration

Runtime adapter binders are integration-engineering tools that
consume adapter manifests (PHASE-1 §9) at boot and bind the
operator's runtime data flows against the registry's content-
addressed identifiers. Integration carries the binder's
identifier, the per-runtime authorisation scope, and the
content-cache rotation policy that the binder honours so that
upgraded artefacts propagate to runtimes within the operator's
declared change window.

## §4 Integration-Engineering Platform Integration

Integration platforms (Talend, Apache Camel, Mule, IPaaS
products) consume registry artefacts through the binder of §3
or directly through the API. The integration record carries
each platform's identifier, the per-pipeline authorisation,
and the pipeline-version pinning that lets the platform
re-resolve registry artefacts deterministically when re-
running a pipeline against historical data.

## §5 Governance Review Platform Integration

Governance review platforms (the operator's data-governance
council, an external standards-development organisation, an
industry consortium's interoperability working group) consume
artefacts at the candidate or qualified status and emit
review verdicts that the registrar registers as
registration-status transitions (PHASE-3 §3). Integration
carries the platform's identifier and the per-class review
SLA.

## §6 Evidence Package Format

```
evidence/
  manifest.json              — package manifest (signed)
  registry.json              — registry record
  artefacts/                 — per-artefact metadata at the
                                cited revision
  artefact-bodies/           — content-addressed body archive
                                (only artefacts whose retention
                                window covers the cited interval)
  expansions/                — value-set expansions at the cited
                                interval
  mappings/                  — mapping-set revisions
  harvests/                  — harvest summaries at the cited
                                interval
  audit/                     — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the registry's quality manager.

## §7 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:interoperability-registry:evidence-mismatch`.

## §8 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-interoperability-registry` that links to the
API root, the naming-authority reference, the published
quality dossier, the federation memberships, and the catalogue
of source code-system custodians the operator binds against.

## §9 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (registrar
qualification, ISO/IEC 27001 certification, federation
membership) to consumers of W3C Verifiable Credentials MAY
re-issue the attestations as Verifiable Credentials under the
Data Model 2.0 specification. Re-issuance is optional; the
canonical record remains the JSON evidence-package manifest.

## §10 Long-Term Archive Integration

Programmes designate a long-term archive that holds registry
snapshots beyond programme wind-down. Snapshots are taken at
the operator's declared cadence (typically annually) and
include every artefact at every revision; on wind-down,
remaining records transfer to the archive with content-
addresses preserved.

## §11 Cross-Standard Linkage

Operators that consume adjacent WIA standards (data-lineage,
master-data-management, schema-evolution) emit cross-standard
linkage records that name the consuming standard and the
version under which the linkage is claimed.

## §12 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support; subscribers that disconnect
during long expansion-refresh windows resume from the last
seen event identifier without losing visibility of priority-1
events (status retirements, supersession notices, federation
conflicts).

## §13 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a deprecation
window of at least one full federation review cycle so that
peer registries and consuming platforms have time to migrate.

## §14 Public Catalogue Integration

Registries that operate under public-catalogue obligations
(industry consortia, government-led interoperability
programmes) emit a JSON Feed listing artefacts at `standard`
and `preferred-standard` registration status with their
content-addresses and the registrar approvals. The feed is a
discovery mechanism for adopting consumers, not a primary
record.

## §15 Integration Migration from Pre-Standard Registries

Operators migrating from pre-standard registries (XML
Registries, ad-hoc spreadsheet-based registries, departmental
repositories) MAY emit synthetic artefact records with a
`legacyImport` flag. Synthetic artefacts begin at `candidate`
status; promotion to higher status requires the operator's
review process per PHASE-3 §3.

## §16 Reader Tooling for Browsable Registries

Operators MAY publish supplementary reader tools (browsable
artefact catalogues, dependency-graph visualisers, mapping-
set comparison views) alongside the canonical evidence
package; the tools are non-normative.

## §17 Joint Naming-Authority Committee Integration

Federation member registries that operate under a joint
naming-authority committee (PHASE-3 §6, §8) integrate with
the committee's adjudication endpoint. The integration carries
the committee's identifier, the per-conflict adjudication
intake reference, and the committee's published decision-
turnaround window so that operators can manage downstream
binding consumer expectations during conflict resolution.

## §18 Per-Class Reviewer Qualification Provider Integration

Reviewer qualification (PHASE-3 §15) is administered by
operator-internal training programmes or by external
qualification bodies (industry consortia, standards-
development organisations, ISO/IEC 11179-aligned training
providers). Integration carries each provider's identifier,
the per-class qualification reference per reviewer, and the
qualification expiry that drives reviewer-status updates in
the operator's IDP.

## §19 SNOMED CT National Release Centre Integration

Operators that bind value sets against SNOMED CT integrate
with the operating jurisdiction's National Release Centre
(NLM in the US, NHS Digital in the UK, KOSEN-NRC equivalents
in other jurisdictions). Integration carries the NRC's
identifier, the per-edition release schedule, and the
licence terms governing redistribution of the operator's
expansions.

## §20 LOINC and RxNorm Custodian Integration

LOINC custodianship rests with the Regenstrief Institute;
RxNorm custodianship rests with the US National Library of
Medicine. Integration with each custodian carries the
custodian's identifier, the release-schedule subscription,
and the per-release expansion-refresh trigger. Custodian
deprecations of codes (LOINC `STATUS=DEPRECATED`, RxNorm
RXCUI retirement) emit value-set expansion-refresh events
that downstream binding consumers receive.

## §21 ICD-11 / WHO Integration

Operators that bind to ICD-11 integrate with the WHO's
ICD-11 maintenance team. Integration carries the WHO's
identifier, the per-revision release schedule, and the
linearisation that the operator binds against (Mortality and
Morbidity Statistics, Primary Care, etc.).

## §22 UMLS Metathesaurus Integration (US-context registries)

US-context operators that bind to multiple biomedical
vocabularies through the UMLS Metathesaurus integrate with
the NLM UMLS distribution. Integration carries the UMLS
licence reference, the per-release subscription, and the
cross-vocabulary mapping inheritance that the operator's
mapping-set adjudication recognises.

## §23 Conformance and Sunset

A registry conformant with PHASE-4 has integrated successfully
with at least one source code-system custodian (where the
registry holds value sets), with at least one peer registry
(where the operator participates in federation), with at
least one runtime adapter binder, with the operator's
governance review platform, and with at least one long-term
archive, and has published at least one externally citable
evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-interoperability-registry
- **Last Updated:** 2026-04-28
