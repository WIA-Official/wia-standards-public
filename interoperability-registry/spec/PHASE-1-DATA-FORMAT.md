# WIA-interoperability-registry PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-interoperability-registry
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-interoperability-registry. The standard covers persistent
record shapes for a metadata registry that catalogues
interoperability artefacts — data elements, value sets,
information models, message schemas, code systems, mapping
sets, ontology fragments, and adapter manifests — that systems
of record use to exchange data with each other. The registry
is consumed by data architects, integration engineers,
compliance reviewers, and the systems of record that bind
their exchanges to the registry's authoritative identifiers.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 11179-1/2/3/4/5/6 (metadata registries — framework,
  classification, registry metamodel and basic attributes,
  formulation of data definitions, naming principles,
  registration)
- ISO/IEC 19763-1/3/5/8/10/12 (metamodel framework for
  interoperability — MFI for ontology, business process,
  forms, models)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6920 (Naming Things with Hashes; cited for
  artefact content-addressing conventions)
- W3C SKOS (Simple Knowledge Organisation System) — used as
  the canonical representation for value-set entries
- W3C OWL 2 (Web Ontology Language) — used for ontology
  fragments
- W3C SHACL (Shapes Constraint Language) — used for binding
  constraints
- OASIS ebXML Registry Information Model 3.0 (cited as the
  reference registry information model from which this
  standard's metamodel derives)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts a
metadata-registry operator manages. Implementations covered
include:

- Data-element registries (ISO/IEC 11179 §4 metadata-element
  registries) holding canonical data-element definitions.
- Value-set registries holding code lists, code systems, and
  the SKOS-encoded value sets that bind them to applications.
- Message-schema registries holding XML, JSON Schema, and
  Avro schemas that bind information models to wire formats.
- Mapping registries holding mapping sets between paired
  vocabularies (HL7 v2 ↔ FHIR R5, OAGIS BOD ↔ XBRL, ACORD
  v2 ↔ ACORD v3, etc.).
- Adapter manifest registries holding integration-adapter
  metadata that consuming systems of record use to discover
  and bind adapters at runtime.
- Cross-registry harvesters that mirror remote registries and
  reconcile cross-registry conflicts.

Master-data-management instance records, transactional message
payloads, and physical-database catalogues are out of scope;
this registry holds metadata about the artefacts that govern
those records, not the records themselves.

## §2 Registry Identifier

```
registryId         : string (uuidv7)
registryOperator   : string (institutional identifier of the
                       operator that maintains the registry)
registryRegistered : string (ISO 8601 / RFC 3339)
registryDomains    : array of enum ("data-elements" |
                       "value-sets" | "message-schemas" |
                       "mapping-sets" | "ontology-fragments" |
                       "adapter-manifests" |
                       "user-defined")
namingAuthorityRef : string (URI of the naming authority that
                       governs the registry's namespace; the
                       registry's identifiers are unique under
                       this authority per ISO/IEC 11179-6
                       registration rules)
registryStatus     : enum ("draft" | "operating" |
                       "frozen" | "archived")
```

## §3 Artefact Identifier and Lifecycle

Every artefact in the registry carries a stable identifier that
is unique under the registry's naming authority. Artefact
revisions are versioned per ISO/IEC 11179-6 registration-
status semantics.

```
artefact:
  artefactId         : string (uuidv7)
  registryId         : string (uuidv7)
  artefactClass      : enum (matches registryDomains; one
                         class per artefact)
  artefactSlug       : string (operator's human-readable
                         identifier under the naming authority)
  registeredAt       : string (ISO 8601 / RFC 3339)
  registrationStatus : enum ("incomplete" | "candidate" |
                         "recorded" | "qualified" |
                         "standard" | "preferred-standard"
                         | "retired" | "withdrawn" |
                         "superseded")
  contentDigest      : string (SHA-256 of the canonical
                         serialisation of the artefact body)
  contentRef         : string (content-addressed URI of the
                         artefact body)
  supersededBy       : string (URI of the successor artefact;
                         absent unless registrationStatus =
                         "superseded")
  custodianRef       : string (institutional identifier of the
                         custodian responsible for the
                         artefact's content)
```

The registration-status transitions follow ISO/IEC 11179-6
state-machine conventions: a candidate may advance to
recorded; recorded to qualified; qualified to standard; and
standard to preferred-standard. Retirement, withdrawal, and
supersession are terminal with respect to consuming systems'
binding decisions.

## §4 Data-Element Record

Data elements follow the ISO/IEC 11179-3 metamodel: each
element has a definition, a value-domain reference, and a
data-type binding.

```
dataElement:
  artefactId         : string (uuidv7; matches §3)
  definition         : string (ISO/IEC 11179-4 conformant;
                         single-sentence definition,
                         distinguishable from related
                         elements, free of redundant
                         qualifiers)
  valueDomainRef     : string (URI of the value-set artefact
                         that the element binds against, where
                         applicable)
  dataType           : enum ("string" | "integer" | "decimal"
                         | "boolean" | "date" | "date-time" |
                         "duration" | "uri" | "uuid" |
                         "binary" | "user-defined")
  cardinality        : enum ("zero-or-one" | "exactly-one" |
                         "zero-or-more" | "one-or-more")
  permittedValuesRef : string (URI of the SKOS-encoded
                         permitted-values value set when the
                         dataType is constrained beyond its
                         primitive range)
  units              : string (UN/CEFACT recommendation 20
                         common code, where applicable)
```

## §5 Value-Set Record

Value sets are SKOS-encoded collections of concepts. Each
concept carries a notation (the code), prefLabel (the
preferred display label per language), and altLabels (the
synonyms used in source systems).

```
valueSet:
  artefactId         : string (uuidv7; matches §3)
  codeSystemRef      : string (URI of the underlying code
                         system the value set draws from;
                         e.g. "http://snomed.info/sct" for
                         SNOMED CT, "http://loinc.org" for
                         LOINC)
  conceptCount       : integer (number of concepts in the
                         value set at the artefact's content
                         digest)
  expansionRef       : string (URI of the expansion artefact;
                         the expansion lists each concept
                         with its notation, prefLabel, and
                         altLabels per BCP 47 language)
  inclusionRulesRef  : string (URI of the inclusion-rules
                         artefact; SKOS-encoded rules that
                         derive the expansion from the
                         underlying code system)
```

## §6 Message-Schema Record

```
messageSchema:
  artefactId         : string (uuidv7; matches §3)
  encoding           : enum ("xml-schema-1.1" |
                         "json-schema-2020-12" | "avro-1.11"
                         | "protobuf-3" | "asn-1" |
                         "user-defined")
  schemaRef          : string (content-addressed URI of the
                         schema body)
  conformanceRef     : string (URI of the conformance test
                         vectors that bind the schema to its
                         intended interpretation)
  bindingDataElements: array of string (URIs of the data
                         elements that the schema binds)
```

## §7 Mapping-Set Record

```
mappingSet:
  artefactId         : string (uuidv7; matches §3)
  sourceArtefactRef  : string (URI of the source vocabulary)
  targetArtefactRef  : string (URI of the target vocabulary)
  mappingRules       : array of MappingRule
  mappingMethod      : enum ("equivalence" | "broader-than" |
                         "narrower-than" | "related-to" |
                         "no-direct-mapping")
  reviewerRef        : string (institutional identifier of
                         the reviewing body that adjudicated
                         the mapping)

MappingRule:
  sourceCode         : string
  targetCode         : string
  relation           : enum (matches W3C SKOS mapping
                         relations: "skos:exactMatch" |
                         "skos:closeMatch" | "skos:broadMatch"
                         | "skos:narrowMatch" |
                         "skos:relatedMatch")
  comment            : string (free text; reviewer's
                         justification for the relation)
```

## §8 Ontology-Fragment Record

Ontology fragments encode classes, properties, and axioms in
W3C OWL 2.

```
ontologyFragment:
  artefactId         : string (uuidv7; matches §3)
  ontologyIri        : string (the IRI under which the
                         fragment is published)
  owl2Profile        : enum ("OWL2-EL" | "OWL2-QL" |
                         "OWL2-RL" | "OWL2-Full")
  importsArtefactRefs: array of string (URIs of imported
                         ontology fragments)
  shaclShapeRefs     : array of string (URIs of SHACL shapes
                         that constrain the fragment's
                         instance data; bindings consumers
                         honour the shapes)
```

## §9 Adapter-Manifest Record

Adapter manifests describe integration adapters that consuming
systems use to bind data elements to runtime data flows.

```
adapterManifest:
  artefactId         : string (uuidv7; matches §3)
  adapterKind        : enum ("file-extract-transform-load" |
                         "message-broker-bridge" |
                         "rest-api-adapter" |
                         "graphql-resolver" |
                         "event-stream-translator" |
                         "user-defined")
  consumedArtefactRefs : array of string (URIs of registry
                         artefacts that the adapter consumes
                         — data elements, value sets, message
                         schemas, mappings)
  producedArtefactRefs : array of string (URIs of registry
                         artefacts that the adapter produces)
  runtimeBindingRef  : string (URI of the operator's runtime
                         binding artefact that materialises
                         the adapter into a deployable
                         component)
```

## §10 Cross-Registry Harvest Record

```
harvest:
  harvestId          : string (uuidv7)
  registryId         : string (uuidv7)
  remoteRegistryRef  : string (URI of the remote registry's
                         well-known discovery document)
  harvestedAt        : string (ISO 8601)
  harvestedArtefactCount : integer
  conflictCount      : integer (number of artefacts whose
                         identifiers collide with existing
                         local artefacts; conflicts are
                         resolved through the protocol in
                         PHASE-3 §6)
  harvestArtefactRef : string (URI of the harvest archive)
```

## §11 Dependency-Graph Snapshot Record

```
dependencyGraphSnapshot:
  snapshotId         : string (uuidv7)
  registryId         : string (uuidv7)
  capturedAt         : string (ISO 8601)
  artefactCount      : integer (number of artefacts at the
                         snapshot's content digest)
  edgeCount          : integer (number of dependency edges)
  cyclesDetected     : integer (number of dependency cycles
                         detected; should be 0 for a healthy
                         registry)
  orphanCount        : integer (number of artefacts with no
                         inbound dependencies)
  snapshotArtefactRef: string (content-addressed URI of the
                         full snapshot graph in JSON-LD or
                         GraphML)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every artefact registered and
honour the ISO/IEC 11179 registration-status semantics in §3.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-interoperability-registry
- **Last Updated:** 2026-04-28
