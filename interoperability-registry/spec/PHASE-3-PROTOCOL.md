# WIA-interoperability-registry PHASE 3 — PROTOCOL Specification

**Standard:** WIA-interoperability-registry
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
interoperability-registry operator: naming-authority
governance, ISO/IEC 11179 registration-status discipline,
artefact-review workflow, value-set expansion stewardship,
mapping-set adjudication, cross-registry conflict resolution,
content-addressing rules, registry federation, recordkeeping,
and registry wind-down.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 11179-1/2/3/4/5/6 (metadata registries)
- ISO/IEC 19763 (metamodel framework for interoperability)
- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- W3C SKOS (value-set encoding)
- W3C OWL 2 (ontology fragments)
- W3C SHACL (constraint shapes)
- OASIS ebXML Registry Information Model 3.0

---

## §1 Naming-Authority Governance

The registry operates under a single naming authority that
governs identifier uniqueness within the registry's namespace.
The naming authority is named in the registry record (PHASE-1
§2 `namingAuthorityRef`); its policies for slug formation,
namespace partitioning, and identifier deprecation are
recorded in the operator's quality dossier.

Naming-authority changes (operator merger, governance
restructuring) follow a change-control procedure that
preserves identifier stability: existing identifiers remain
addressable under the prior authority through a content-
addressable redirect, and new identifiers go through the
successor authority.

## §2 ISO/IEC 11179 Registration-Status Discipline

Artefacts move through the ISO/IEC 11179-6 registration-status
lifecycle (PHASE-1 §3) under registrar oversight. Each
transition requires:

- review against the operator's per-class quality bar
  (definition clarity for data elements, expansion
  completeness for value sets, conformance vector coverage
  for message schemas, mapping-set reviewer signoff);
- approval by a registrar with the per-class authorisation;
- record of the transition rationale in the artefact's
  audit log.

Artefacts that fail review are returned to `incomplete` with
a documented set of corrective actions. Artefacts marked
`retired` or `withdrawn` remain addressable but emit a
deprecation notice through the streaming subscription so that
binding consumers can plan migration.

## §3 Artefact Review Workflow

The review workflow is per-class:

- **Data elements**: ISO/IEC 11179-4 conformance check
  (definition clarity, distinguishability), value-domain
  reference resolution, data-type alignment.
- **Value sets**: SKOS-encoding well-formedness, expansion
  reproducibility from inclusion rules, code-system
  authority verification.
- **Message schemas**: schema validity against the encoding's
  meta-schema, conformance-vector coverage, binding-data-
  element resolution.
- **Mapping sets**: paired-vocabulary versioning, per-rule
  reviewer rationale, mapping-method coherence (no
  equivalence claim where rules are mostly broader/narrower).
- **Ontology fragments**: OWL 2 profile conformance,
  imports resolution, SHACL shape validity against the
  fragment's vocabulary.
- **Adapter manifests**: consumed-artefact resolution,
  produced-artefact resolution, runtime-binding
  reproducibility.

## §4 Value-Set Expansion Stewardship

Value sets bind to underlying code systems (SNOMED CT, LOINC,
RxNorm, ICD-11, ATC, UNECE recommendations, ISO 3166-2, etc.)
that the registry does not control. Code-system updates
trigger value-set expansion refreshes per the operator's
declared cadence (typically aligned with the source code
system's release schedule).

Expansion-refresh events emit cross-references when refreshed
expansions materially differ from the prior expansion (added
or removed concepts, prefLabel changes that affect display).
Binding consumers receive the cross-references through the
streaming subscription.

## §5 Mapping-Set Adjudication

Mapping sets are inherently subjective; the same source-target
vocabulary pair can support multiple mapping sets that reflect
different application contexts. The operator's mapping
governance records:

- the application context the mapping is intended for;
- the reviewing body that adjudicated the mapping;
- the per-rule reviewer rationale (PHASE-1 §7 MappingRule
  comment);
- the mapping's expected revision cadence.

Mapping disagreements between application contexts are
resolved by registering parallel mapping sets, not by
overwriting the existing mapping; consumers select the
mapping appropriate to their context.

## §6 Cross-Registry Conflict Resolution

Cross-registry harvesting (PHASE-1 §10) occasionally surfaces
conflicts: an artefact identifier exists locally with a
different content digest than the harvested artefact. The
conflict-resolution protocol:

- if both artefacts are in `candidate` or `recorded` status,
  the operator merges the artefacts under the local
  identifier and records the merge as the resolution;
- if either artefact is in `qualified` or higher status, the
  operator escalates to the joint naming-authority committee
  that the federation has registered for this purpose;
- if no joint committee exists, the operator records the
  conflict as `unresolved` and notifies binding consumers
  through the streaming subscription so that consumers can
  select a side.

## §7 Content-Addressing Rules

Artefact bodies are content-addressed by SHA-256 (PHASE-1 §3
`contentDigest`). The canonical serialisation rules per
encoding:

- JSON / JSON Schema: RFC 8785 JSON Canonicalization Scheme
  (JCS) before digest computation.
- XML / XML Schema: W3C Canonical XML 1.1 before digest
  computation.
- SKOS / OWL / SHACL Turtle: RDF normalisation per RDFC-1.0
  before digest computation.
- Avro / Protobuf: schema-canonical-form serialisation
  before digest.

Digest mismatches between submitted body and declared
`contentDigest` cause the API to reject the submission with
type `urn:wia:interoperability-registry:content-digest-
mismatch`.

## §8 Registry Federation

Registries may federate so that a query against one registry
can resolve artefacts that another registry holds. Federation
discipline:

- each member registry retains naming authority over its own
  namespace; federation does not merge namespaces;
- federation queries return the source registry's identifier
  alongside the artefact so that consumers know which
  registry holds the authoritative version;
- federation member changes (joins, departures) are recorded
  as audit events.

## §9 Records Retention

Registry records — every artefact at every revision, the API
audit logs, the harvest histories, the conflict-resolution
records, and the registrar approvals — retain indefinitely.
Retired and superseded artefacts remain addressable so that
historical bindings continue to resolve.

## §10 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) against a
national-metrological-laboratory stratum-1 service so that
artefact-registration timestamps and audit logs are
consistent across the registrar fleet.

## §11 Privacy

Artefact metadata is, by design, public (or operator-internal
for non-public artefacts). The registry holds no personal
data; the operator's policy is that data elements that
describe personal-data fields carry only the field-level
description, never instance-level personal data.

## §12 Quality Dossier

The operator's quality dossier records the naming authority,
the per-class review committees, the reviewer authorisation
register, the federation memberships, the harvest sources,
and the operator's incident history.

## §13 Registry Freeze and Wind-Down

Registries that freeze (no new artefacts accepted, existing
artefacts continue to resolve) record the freeze rationale
and the expected wind-down date. Wind-down archives the
registry to a long-term archive (PHASE-4 §10) and notifies
binding consumers so that they can redirect bindings to a
successor registry where one exists.

## §14 Identifier-Stability Discipline

Artefact identifiers (PHASE-1 §3) are stable for the artefact's
operational lifetime and beyond: identifiers do not move
between artefacts even after retirement or supersession,
because downstream systems pin against them in long-lived
bindings. Operator policy:

- never recycle a retired or superseded identifier under a
  new artefact;
- always carry the supersession chain so that consumers
  resolving the retired identifier discover the successor;
- treat identifier collisions across federation members
  through the conflict-resolution protocol of §6 rather than
  through silent overwrite.

## §15 Reviewer Qualification

Registrars and per-class reviewers carry qualification records
held in the operator's HR / IDP. Qualification covers:

- ISO/IEC 11179-4 definition-formulation training for
  data-element reviewers;
- SKOS / value-set-stewardship training for value-set
  reviewers;
- schema-language proficiency (XML Schema, JSON Schema,
  Avro, Protobuf) for message-schema reviewers;
- mapping-set adjudication training (W3C SKOS mapping
  relations, application-context reasoning) for mapping-set
  reviewers;
- OWL 2 profile expertise for ontology-fragment reviewers.

Qualifications expire on the operator's review cycle; expired
qualifications demote the reviewer to read-only access until
re-qualification.

## §16 Dependency-Graph Discipline

Artefacts depend on other artefacts (a data element binds a
value set; a message schema binds data elements; an adapter
manifest consumes message schemas; an ontology fragment
imports other fragments). The operator maintains a
dependency-graph view that surfaces:

- transitive dependency chains so that supersession of an
  upstream artefact propagates to all downstream artefacts;
- cyclic dependencies that the registrar must resolve before
  registering the cycle's artefacts at higher status;
- orphan artefacts that no consumer references.

Transitive supersession events emit per-affected-downstream
notifications through the streaming subscription so that
consuming systems can plan migration before the upstream
retirement is enforced.

## §17 Code-System Licensing Discipline

Source code systems (PHASE-1 §5 `codeSystemRef`) carry
licences that the consuming registry MUST honour. Licensing
discipline:

- record per-code-system licence terms in the operator's
  quality dossier;
- enforce per-licence access controls at the value-set
  expansion download endpoint;
- emit licensing notices on the per-class reader tooling so
  that downstream binding consumers see the licence terms
  alongside the artefact body.

Licences that lapse or are amended trigger an integration
review with the source publisher; affected value-set
expansions enter `frozen` status until the licence is
re-established.

## §18 Cross-Class Consistency Discipline

Artefacts that reference each other across classes (a message
schema binds data elements; an adapter manifest consumes
mappings, schemas, and value sets) follow consistency rules
that the registrar enforces during status promotion:

- a message schema cannot reach `qualified` until every
  bound data element is at `qualified` or higher;
- an adapter manifest cannot reach `standard` until every
  consumed artefact (schemas, mappings, value sets) is at
  `standard` or higher;
- mapping sets cannot reach `preferred-standard` until both
  the source and target vocabularies are at `standard` or
  higher.

Cross-class consistency violations during promotion attempts
trigger a Problem-Details response and the artefact remains
at its current status until the upstream artefacts are
promoted.

## §19 Conformance and Auditing

A registry conformant with WIA-interoperability-registry
publishes its naming-authority reference, its per-class
review-committee register, its federation memberships, and
the catalogue of artefacts at each registration status, and
answers an annual self-assessment that maps each clause of
this PHASE to the operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-interoperability-registry
- **Last Updated:** 2026-04-28
