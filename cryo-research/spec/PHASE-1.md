# WIA-CRYO-010 PHASE 1: DATA FORMATS

**Standard**: WIA-CRYO-010  
**Phase**: 1 - Data Formats and Schemas  
**Version**: 1.0.0  
**Date**: January 2025  
**Status**: Active  

## Overview

Phase 1 defines the core data formats, schemas, and structures for cryopreservation research data. All data interchange in WIA-CRYO-010 compliant systems must adhere to these specifications.

## 1.1 Core Data Format: JSON-LD

All WIA-CRYO-010 data MUST be represented as JSON-LD (JavaScript Object Notation for Linked Data).

### Context Definition

```json
{
  "@context": "https://wia.org/standards/cryo-research/v1",
  "@vocab": "https://wia.org/schemas/cryo-research#",
  "schema": "http://schema.org/",
  "xsd": "http://www.w3.org/2001/XMLSchema#",
  "unit": "http://qudt.org/schema/qudt/"
}
```

### Base Schema Structure

```json
{
  "@context": "https://wia.org/standards/cryo-research/v1",
  "@type": "CryoResearchData",
  "@id": "urn:uuid:{UUID}",
  "version": "1.0.0",
  "created": "ISO-8601 timestamp",
  "modified": "ISO-8601 timestamp",
  "data": {}
}
```

## 1.2 Experiment Record Schema

```json
{
  "@context": "https://wia.org/standards/cryo-research/v1",
  "@type": "CryoResearchExperiment",
  "experimentId": "string (UUID or unique identifier)",
  "title": "string (required)",
  "principalInvestigator": {
    "@type": "Person",
    "name": "string (required)",
    "orcid": "string (0000-0000-0000-0000 format)",
    "affiliation": "string"
  },
  "institution": {
    "@type": "Organization",
    "name": "string (required)",
    "ror": "string (Research Organization Registry ID)"
  },
  "lifecycle": {
    "status": "enum [PROPOSED, APPROVED, PREPARING, IN_PROGRESS, ANALYZING, COMPLETED, ARCHIVED]",
    "startDate": "ISO-8601 date-time",
    "expectedCompletion": "ISO-8601 date-time",
    "actualCompletion": "ISO-8601 date-time"
  },
  "scientific": {
    "researchType": "enum [CPA_OPTIMIZATION, COOLING_RATE, REVIVAL_PROTOCOL, VIABILITY_ASSESSMENT, LONG_TERM_STORAGE, OTHER]",
    "hypothesis": "string",
    "subjectType": "enum [CELL_CULTURE, TISSUE, ORGAN, ORGANISM, GAMETE, EMBRYO]",
    "species": "string (scientific name)",
    "sampleSize": "integer",
    "replicates": "integer",
    "controlGroups": "integer"
  },
  "technical": {
    "cpaFormulation": "object (reference to Formulation)",
    "coolingProtocol": "object",
    "storageDuration": "ISO-8601 duration",
    "warmingProtocol": "object",
    "assessmentMethods": "array of strings",
    "equipment": "array of objects"
  },
  "results": {
    "primaryEndpoint": "object",
    "secondaryEndpoints": "array of objects",
    "rawDataLinks": "array of URIs"
  },
  "metadata": {
    "irbApproval": "string",
    "fundingSource": "string",
    "keywords": "array of strings",
    "relatedPublications": "array of DOIs"
  }
}
```

## 1.3 CPA Formulation Schema

```json
{
  "@context": "https://wia.org/standards/cryo-research/v1",
  "@type": "CryoprotectantFormulation",
  "formulationId": "string (unique identifier)",
  "name": "string",
  "version": "string (semantic versioning)",
  "status": "enum [EXPERIMENTAL, VALIDATED, DEPRECATED]",
  "targetApplication": {
    "subjectType": "string",
    "species": "string",
    "tissueType": "string"
  },
  "baseSolution": {
    "name": "string",
    "composition": "string",
    "pH": "number",
    "osmolality": "number",
    "unit": "mOsm/kg"
  },
  "permeatingCPAs": [
    {
      "component": "string",
      "casNumber": "string",
      "concentration": "number",
      "unit": "string (%v/v, M, mM)",
      "manufacturer": "string",
      "catalogNumber": "string",
      "lotNumber": "string",
      "purity": "string"
    }
  ],
  "nonPermeatingCPAs": [
    {
      "component": "string",
      "casNumber": "string",
      "concentration": "number",
      "unit": "string (mM, mg/mL, %w/v)",
      "manufacturer": "string",
      "catalogNumber": "string",
      "lotNumber": "string"
    }
  ],
  "iceBlockers": "array (same structure as CPAs)",
  "additives": "array (same structure as CPAs)",
  "physicalProperties": {
    "finalOsmolality": "number",
    "osmolalityUnit": "mOsm/kg",
    "finalPH": "number",
    "viscosity": "string",
    "glassTransitionTemp": "number",
    "glassTempUnit": "°C"
  },
  "preparationProtocol": {
    "steps": "array of strings",
    "equipmentRequired": "array of strings",
    "safetyNotes": "array of strings"
  },
  "performanceMetrics": {
    "cellViability": "object with mean, sd, n, assay",
    "functionalAssessment": "object",
    "vqi": "object (Vitrification Quality Index)"
  },
  "validationStudies": "array of experiment references",
  "citations": "array of DOIs"
}
```

## 1.4 Viability Assessment Schema

```json
{
  "@context": "https://wia.org/standards/cryo-research/v1",
  "@type": "ViabilityAssessment",
  "assessmentId": "string (UUID)",
  "experimentId": "string (reference)",
  "sampleId": "string",
  "assessmentTime": "ISO-8601 timestamp",
  "timePostThaw": {
    "value": "number",
    "unit": "string (hours, days)"
  },
  "assays": [
    {
      "method": "enum [TRYPAN_BLUE, PROPIDIUM_IODIDE, MTT, ALAMAR_BLUE, ATP, ANNEXIN_V, FLOW_CYTOMETRY, OTHER]",
      "protocol": "string (description or reference)",
      "results": {
        "viabilityPercent": "number (0-100)",
        "confidence": "number (95 typical)",
        "standardError": "number",
        "totalCells": "integer",
        "viableCells": "integer",
        "deadCells": "integer"
      },
      "operator": "string",
      "equipment": "string",
      "quality": "enum [PASS, FAIL, UNCERTAIN]"
    }
  ],
  "recovery": {
    "cellsPreFreeze": "number",
    "cellsPostThaw": "number",
    "recoveryPercent": "number",
    "effectiveViability": "number"
  },
  "interpretation": {
    "overall": "enum [EXCELLENT, GOOD, FAIR, POOR]",
    "viabilityGrade": "string",
    "notes": "string"
  }
}
```

## 1.5 Long-term Outcome Study Schema

```json
{
  "@context": "https://wia.org/standards/cryo-research/v1",
  "@type": "LongTermOutcomeStudy",
  "studyId": "string (unique)",
  "title": "string",
  "registration": {
    "registry": "string (e.g., ClinicalTrials.gov)",
    "identifier": "string",
    "registrationDate": "ISO-8601 date"
  },
  "design": {
    "type": "enum [PROSPECTIVE_COHORT, RETROSPECTIVE, CASE_CONTROL, RCT]",
    "duration": "string",
    "enrollmentTarget": "integer",
    "centers": "integer",
    "randomization": "string",
    "blinding": "string"
  },
  "population": {
    "inclusion": "array of strings",
    "exclusion": "array of strings"
  },
  "endpoints": {
    "primary": {
      "name": "string",
      "target": "string",
      "assay": "string"
    },
    "secondary": "array of endpoint objects"
  },
  "assessmentSchedule": {
    "timepoints": "array of strings (e.g., '1 month', '6 months')",
    "samplesPerTimepoint": "integer"
  },
  "results": {
    "totalSamples": "integer",
    "successfulSamples": "integer",
    "successRate": "number (percentage)",
    "confidenceInterval": {
      "level": 95,
      "lowerBound": "number",
      "upperBound": "number"
    },
    "bySubgroup": "array of subgroup analysis objects"
  }
}
```

## 1.6 Data Quality Metadata

Every data record SHOULD include quality metadata:

```json
{
  "quality": {
    "completeness": "number (0-1, percentage of required fields)",
    "consistency": "boolean (passed consistency checks)",
    "accuracy": "string (assessment of measurement accuracy)",
    "timeliness": "ISO-8601 (when data was collected)",
    "validation": {
      "validated": "boolean",
      "validator": "string (name or ID)",
      "validationDate": "ISO-8601",
      "method": "string"
    }
  },
  "provenance": {
    "creator": "string or object",
    "created": "ISO-8601",
    "modified": "ISO-8601",
    "modifiedBy": "string",
    "changeLog": "array of change objects"
  }
}
```

## 1.7 Controlled Vocabularies

All enumerations MUST use standardized values:

### Research Types
- `CPA_OPTIMIZATION`
- `COOLING_RATE_OPTIMIZATION`
- `REVIVAL_PROTOCOL_DEVELOPMENT`
- `VIABILITY_ASSESSMENT`
- `LONG_TERM_STORAGE_STUDY`
- `COMPARATIVE_STUDY`
- `OTHER`

### Subject Types
- `CELL_CULTURE` (with subtypes: PRIMARY, IMMORTALIZED, STEM_CELL)
- `TISSUE`
- `ORGAN`
- `ORGANISM`
- `GAMETE` (OOCYTE, SPERM)
- `EMBRYO`

### Viability Assay Methods
- `TRYPAN_BLUE`
- `PROPIDIUM_IODIDE`
- `7AAD`
- `CALCEIN_AM`
- `MTT`
- `ALAMAR_BLUE`
- `ATP_ASSAY`
- `ANNEXIN_V_PI`
- `FLOW_CYTOMETRY`
- `LDH_RELEASE`
- `TUNEL`
- `OTHER`

### Lifecycle Status
- `PROPOSED`
- `APPROVED`
- `PREPARING`
- `IN_PROGRESS`
- `ANALYZING`
- `COMPLETED`
- `SUSPENDED`
- `FAILED`
- `ARCHIVED`

## 1.8 Units and Measurements

All numeric values MUST include units. Recommended unit vocabulary: QUDT (http://qudt.org/)

### Temperature
- Celsius: `°C`
- Kelvin: `K`

### Concentration
- Percentage volume/volume: `%v/v`
- Percentage weight/volume: `%w/v`
- Molar: `M`
- Millimolar: `mM`
- Micromolar: `μM`
- Milligrams per milliliter: `mg/mL`

### Time
- Seconds: `s`
- Minutes: `min`
- Hours: `h`
- Days: `d`
- ISO-8601 duration: `P1Y2M3DT4H5M6S`

### Volume
- Milliliter: `mL`
- Microliter: `μL`
- Liter: `L`

## 1.9 File Formats

### Primary Format
- **JSON-LD**: All structured data

### Supplementary Formats
- **CSV**: Tabular data export (with mandatory header row and data dictionary)
- **XML**: Alternative serialization if JSON not supported
- **HDF5**: Large time-series data (temperature logs)
- **TIFF**: Raw microscopy images (uncompressed)
- **PDF/A**: Archival documents

## 1.10 Versioning

All schemas use semantic versioning (MAJOR.MINOR.PATCH):
- MAJOR: Incompatible changes
- MINOR: Backward-compatible additions
- PATCH: Backward-compatible fixes

Current version: **1.0.0**

## 1.11 Validation

All JSON-LD documents MUST validate against the JSON Schema available at:
https://wia.org/schemas/cryo-research/v1/schema.json

Validation tools:
- Online: https://www.jsonschemavalidator.net/
- CLI: `ajv validate -s schema.json -d data.json`
- Python: `jsonschema` library
- JavaScript: `ajv` library

## 1.12 Extensions

Implementers MAY add custom fields prefixed with organization namespace:
```json
{
  "@context": "https://wia.org/standards/cryo-research/v1",
  "wia:standard": "value",
  "myorg:customField": "value"
}
```

## References

- JSON-LD: https://www.w3.org/TR/json-ld/
- JSON Schema: https://json-schema.org/
- ISO 8601: https://www.iso.org/iso-8601-date-and-time-format.html
- QUDT: http://qudt.org/

---

**Next Phase**: [PHASE-2: Algorithms](PHASE-2.md)

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for cryo-research is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/cryo-research/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cryo-research/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cryo-research/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-1

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1.

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
