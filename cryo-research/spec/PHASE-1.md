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
