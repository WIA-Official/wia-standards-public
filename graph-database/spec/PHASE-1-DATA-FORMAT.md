# WIA-DATA-015: Graph Database Standard
## PHASE 1: Data Format Specification

**Version:** 1.0  
**Status:** Draft  
**Last Updated:** 2025-01-15

---

## 1. Overview

This document defines the data format specifications for the WIA-DATA-015 Graph Database Standard. It establishes the foundational structures for representing graph data, including nodes, edges, properties, and metadata.

## 2. Graph Data Model

### 2.1 Property Graph Model

The WIA-DATA-015 standard adopts the property graph model as its primary data representation:

- **Nodes (Vertices):** Discrete entities with optional labels and properties
- **Edges (Relationships):** Directed connections between nodes with type and properties
- **Properties:** Key-value pairs attached to nodes and edges
- **Labels:** Categories or types for nodes (multi-label supported)

### 2.2 Data Types

#### Primitive Types
- `string`: UTF-8 encoded text
- `integer`: 64-bit signed integer
- `float`: 64-bit floating point
- `boolean`: true/false
- `null`: Absence of value

#### Temporal Types
- `date`: ISO 8601 date (YYYY-MM-DD)
- `time`: ISO 8601 time with timezone
- `datetime`: ISO 8601 combined date and time
- `duration`: ISO 8601 duration

#### Collection Types
- `list`: Ordered collection of values
- `map`: Key-value pairs (object/dictionary)

#### Spatial Types
- `point`: Geographic or cartesian coordinates
- `polygon`: Bounded region

## 3. Node Specification

### 3.1 Node Structure

```json
{
  "id": "node_001",
  "labels": ["Person", "Employee"],
  "properties": {
    "name": "Alice Johnson",
    "email": "alice@example.com",
    "age": 30,
    "joinedDate": "2020-01-15",
    "verified": true
  },
  "metadata": {
    "created": "2024-01-15T10:30:00Z",
    "updated": "2024-06-20T14:22:00Z",
    "version": 3
  }
}
```

### 3.2 Node Requirements

- **ID:** Unique identifier (string or integer, immutable)
- **Labels:** Array of zero or more label strings
- **Properties:** Object with key-value pairs
- **Metadata:** System-managed tracking information (optional)

### 3.3 Label Conventions

- Use PascalCase for labels: `Person`, `Product`, `Company`
- Labels should be singular nouns
- Multiple labels allowed: `["Person", "Employee", "Manager"]`
- Reserved labels start with underscore: `_System`, `_Temp`

## 4. Edge Specification

### 4.1 Edge Structure

```json
{
  "id": "edge_001",
  "type": "WORKS_FOR",
  "source": "node_001",
  "target": "node_002",
  "properties": {
    "since": "2020-01-15",
    "role": "Software Engineer",
    "department": "Engineering"
  },
  "metadata": {
    "created": "2024-01-15T10:35:00Z",
    "weight": 1.0
  }
}
```

### 4.2 Edge Requirements

- **ID:** Unique identifier (optional, can be auto-generated)
- **Type:** Single relationship type (required)
- **Source:** ID of source node (required)
- **Target:** ID of target node (required)
- **Direction:** Always directed (bidirectional traversal allowed)
- **Properties:** Object with key-value pairs (optional)
- **Metadata:** System-managed information (optional)

### 4.3 Relationship Type Conventions

- Use UPPER_SNAKE_CASE: `WORKS_FOR`, `KNOWS`, `PURCHASED`
- Should be verb phrases indicating the relationship
- Always directional: source→type→target
- Reserved types start with underscore: `_SYSTEM_LINK`

## 5. Property Specifications

### 5.1 Property Naming

- Use camelCase: `firstName`, `emailAddress`, `createdAt`
- Avoid reserved keywords: `id`, `type`, `labels`
- Length: 1-64 characters
- Characters: alphanumeric, underscore, no spaces

### 5.2 Property Constraints

```json
{
  "propertyName": "email",
  "dataType": "string",
  "required": false,
  "unique": true,
  "indexed": true,
  "validation": {
    "pattern": "^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}$",
    "maxLength": 255
  }
}
```

### 5.3 Property Validation Rules

- **String:** maxLength, minLength, pattern (regex)
- **Integer/Float:** min, max, precision
- **List:** maxItems, minItems, itemType
- **Map:** requiredKeys, allowedKeys

## 6. Serialization Formats

### 6.1 JSON Representation

Primary serialization format for data exchange:

```json
{
  "graph": {
    "nodes": [
      {
        "id": "1",
        "labels": ["Person"],
        "properties": {"name": "Alice", "age": 30}
      },
      {
        "id": "2",
        "labels": ["Company"],
        "properties": {"name": "Acme Corp"}
      }
    ],
    "edges": [
      {
        "id": "e1",
        "type": "WORKS_FOR",
        "source": "1",
        "target": "2",
        "properties": {"since": "2020-01-01"}
      }
    ]
  }
}
```

### 6.2 CSV Representation

For bulk import/export:

**Nodes CSV:**
```csv
:ID,name:string,age:int,:LABEL
1,Alice,30,Person
2,Acme Corp,,Company
```

**Edges CSV:**
```csv
:START_ID,:END_ID,:TYPE,since:date
1,2,WORKS_FOR,2020-01-01
```

### 6.3 GraphML Representation

XML-based format for tool interoperability:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<graphml xmlns="http://graphml.graphdrawing.org/xmlns">
  <graph id="G" edgedefault="directed">
    <node id="1">
      <data key="label">Person</data>
      <data key="name">Alice</data>
      <data key="age">30</data>
    </node>
    <edge id="e1" source="1" target="2">
      <data key="type">WORKS_FOR</data>
      <data key="since">2020-01-01</data>
    </edge>
  </graph>
</graphml>
```

## 7. Schema Definition

### 7.1 Schema Format

```json
{
  "schema": {
    "version": "1.0",
    "nodeTypes": [
      {
        "label": "Person",
        "properties": {
          "name": {"type": "string", "required": true},
          "email": {"type": "string", "unique": true},
          "age": {"type": "integer", "min": 0}
        }
      }
    ],
    "edgeTypes": [
      {
        "type": "WORKS_FOR",
        "sourceLabel": "Person",
        "targetLabel": "Company",
        "properties": {
          "since": {"type": "date"},
          "role": {"type": "string"}
        }
      }
    ],
    "constraints": [
      {
        "type": "unique",
        "nodeLabel": "Person",
        "property": "email"
      },
      {
        "type": "existence",
        "nodeLabel": "Person",
        "property": "name"
      }
    ]
  }
}
```

## 8. Metadata and Versioning

### 8.1 Graph Metadata

```json
{
  "metadata": {
    "graphId": "graph_001",
    "name": "Corporate Network",
    "version": "2.1.0",
    "created": "2024-01-01T00:00:00Z",
    "modified": "2024-06-15T10:30:00Z",
    "nodeCount": 1500,
    "edgeCount": 3200,
    "schema": "schema_v1.json",
    "description": "Employee and organizational relationships"
  }
}
```

### 8.2 Versioning Strategy

- **Semantic Versioning:** MAJOR.MINOR.PATCH
- **Change Tracking:** Track schema and data versions independently
- **Migration:** Provide upgrade paths between versions
- **Compatibility:** Maintain backward compatibility within major versions

## 9. Data Integrity

### 9.1 Referential Integrity

- All edges must reference existing nodes
- Node deletion requires orphan edge handling:
  - `CASCADE`: Delete dependent edges
  - `RESTRICT`: Prevent deletion if edges exist
  - `SET_NULL`: Invalid for graphs (edges require endpoints)

### 9.2 Constraints

```json
{
  "constraints": {
    "uniqueness": [
      {"nodeLabel": "User", "property": "username"},
      {"nodeLabel": "Product", "property": "sku"}
    ],
    "existence": [
      {"nodeLabel": "Person", "property": "name"},
      {"edgeType": "PURCHASED", "property": "date"}
    ],
    "nodeKey": [
      {"nodeLabel": "Person", "properties": ["firstName", "lastName", "birthDate"]}
    ]
  }
}
```

## 10. File Format Specifications

### 10.1 WIA Graph Format (.wgf)

Binary format for efficient storage and transmission:

```
Header (32 bytes):
- Magic number: 0x57474631 ('WGF1')
- Version: uint16
- Flags: uint16
- Node count: uint64
- Edge count: uint64
- Property count: uint64

Node Block:
- Node ID: varint
- Label count: uint8
- Labels: [string]
- Property count: uint16
- Properties: [key:string, type:uint8, value:bytes]

Edge Block:
- Edge ID: varint
- Type: string
- Source: varint
- Target: varint
- Property count: uint16
- Properties: [key:string, type:uint8, value:bytes]
```

## 11. Compliance

### 11.1 Required Features

Implementations must support:
- Property graph model
- All primitive data types
- JSON serialization
- Basic constraints (unique, existence)
- Referential integrity

### 11.2 Optional Features

- Binary format support (.wgf)
- GraphML import/export
- Advanced spatial types
- Custom validation rules
- Schema evolution tools

## 12. Security Considerations

### 12.1 Data Protection

- Property encryption support for sensitive fields
- Redaction of PII in exports
- Access control metadata on nodes/edges

### 12.2 Validation

- Input sanitization for injection prevention
- Schema validation before import
- Size limits to prevent DoS

---

## Appendix A: Examples

See `examples/data-format/` directory for complete examples of:
- Node and edge definitions
- Schema files
- Import/export samples
- Validation rules

## Appendix B: References

- ISO/IEC 39075 GQL Standard (draft)
- Property Graph Schema Working Group
- Neo4j Data Format Documentation
- Apache TinkerPop Data Model

---

**License:** CC BY 4.0  
**Maintained by:** WIA Standards Committee  
**Contact:** standards@wia-official.org


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
