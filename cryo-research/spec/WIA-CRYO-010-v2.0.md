# WIA-CRYO-010 Specification v2.0

**Cryopreservation Research Data Standard**

**Status:** Official Release
**Release Date:** 2025-01-01
**Supersedes:** WIA-CRYO-010 v1.2
**Maintained By:** WIA Standards Committee

---

## Abstract

WIA-CRYO-010 defines a comprehensive standard for documenting, sharing, and analyzing cryopreservation research data. This specification enables reproducible science, facilitates collaboration, and supports regulatory compliance across basic research and clinical applications.

**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Scope

This standard applies to:

- Experimental cryopreservation research (cells, tissues, organs)
- Clinical cryopreservation applications (reproductive medicine, tissue banking)
- Research data exchange and collaboration
- Regulatory submissions requiring cryopreservation data
- Long-term data archiving and preservation

**Out of Scope:**
- Food preservation and industrial freezing
- Geological and environmental cryobiology
- Cryogenic engineering (non-biological)

---

## 2. Normative References

- ISO 8601: Date and time format
- ISO 3166: Country codes
- ORCID: Researcher identification
- FAIR Data Principles: Findable, Accessible, Interoperable, Reusable
- CDISC SDTM: Clinical data standards
- HIPAA: Health insurance portability and accountability
- GDPR: General data protection regulation

---

## 3. Terms and Definitions

### 3.1 Cryopreservation
The process of preserving biological materials at ultra-low temperatures (typically -80°C to -196°C) to maintain viability.

### 3.2 Cryoprotectant Agent (CPA)
Chemical compounds that protect cells and tissues from cryoinjury during freezing and thawing.

### 3.3 Vitrification
A cryopreservation method achieving glass-like solidification without ice crystal formation.

### 3.4 Slow Freezing
Controlled-rate cooling method allowing gradual cellular dehydration.

### 3.5 Viability
The proportion of cells or tissues that retain membrane integrity and metabolic function after cryopreservation.

### 3.6 Revival Success Rate
The percentage of cryopreserved specimens that meet defined success criteria after thawing.

---

## 4. Data Format Specifications

### 4.1 Core Data Structure

All experimental data SHALL conform to the following hierarchical structure:

```json
{
  "standard": "WIA-CRYO-010",
  "version": "2.0",
  "experiment": {
    "id": "string (REQUIRED, UNIQUE, FORMAT: CRYO-YYYY-NNN)",
    "type": "enum (REQUIRED: cell|tissue|organ|embryo)",
    "title": "string (REQUIRED)",
    "date": "ISO8601 datetime (REQUIRED)",
    "researcher": {
      "name": "string (REQUIRED)",
      "orcid": "string (REQUIRED, FORMAT: 0000-0000-0000-0000)",
      "institution": "string (REQUIRED)",
      "email": "string (REQUIRED, FORMAT: email)"
    }
  },
  "sample": {
    "type": "string (REQUIRED, CONTROLLED_VOCABULARY)",
    "source": "string (REQUIRED)",
    "quantity": "number (REQUIRED)",
    "unit": "string (REQUIRED)"
  },
  "protocol": {
    "method": "enum (REQUIRED: slow_freeze|vitrification|directional|ultra_rapid)",
    "temperature_profile": "array (REQUIRED, MIN_LENGTH: 3)",
    "cryoprotectant": "object (REQUIRED)",
    "equipment_settings": "object (REQUIRED)"
  },
  "measurements": {
    "pre_freeze_viability": "number (REQUIRED, RANGE: 0-100)",
    "post_thaw_viability": "number (REQUIRED, RANGE: 0-100)",
    "assessment_timepoint": "number (REQUIRED)",
    "assessment_method": "string (REQUIRED)"
  },
  "quality_control": {
    "equipment_calibration": "ISO8601 date (REQUIRED)",
    "protocol_deviations": "array (OPTIONAL)"
  }
}
```

### 4.2 Temperature Profile Format

Temperature data MUST include:
- Temporal resolution: ≥1 measurement per minute during critical phases
- Spatial data: Sample temperature (distinct from chamber temperature)
- Unit specification: Celsius (default) or Kelvin
- Accuracy metadata: ±0.1°C minimum

```json
"temperature_profile": {
  "unit": "celsius|kelvin",
  "sampling_rate": "number (measurements_per_minute)",
  "data_points": [
    {
      "time": "number (minutes_from_start)",
      "temperature": "number",
      "chamber_temperature": "number (OPTIONAL)",
      "sample_temperature": "number (REQUIRED)"
    }
  ],
  "statistics": {
    "mean_cooling_rate": "number (degrees_per_minute)",
    "temperature_uniformity": "number (0-1 scale)"
  }
}
```

### 4.3 Cryoprotectant Documentation

All CPAs MUST be documented with:
- Chemical name (IUPAC or common name)
- Concentration (% v/v, % w/v, or molar)
- Grade/purity
- Manufacturer and lot number
- Total composition MUST sum to 100%

```json
"cryoprotectant": {
  "components": [
    {
      "name": "string (REQUIRED)",
      "concentration": "number (REQUIRED)",
      "unit": "enum (REQUIRED: percent_v/v|percent_w/v|molar)",
      "grade": "string (REQUIRED)",
      "manufacturer": "string (REQUIRED)",
      "lot_number": "string (RECOMMENDED)"
    }
  ],
  "osmolality": "number (OPTIONAL, UNIT: mOsm/kg)",
  "pH": "number (OPTIONAL)",
  "preparation_date": "ISO8601 date (REQUIRED)"
}
```

### 4.4 Viability Assessment

Viability data SHALL include:
- Assessment method (standardized terminology)
- Timepoint(s) post-thaw
- Sample size (n ≥ 3 biological replicates)
- Statistical measures (mean, SD, CI)

```json
"viability_assessment": {
  "method": "enum (REQUIRED: trypan_blue|flow_cytometry|MTT|ATP|live_dead_staining)",
  "timepoint": "number (REQUIRED, UNIT: hours_post_thaw)",
  "biological_replicates": "number (REQUIRED, MIN: 3)",
  "technical_replicates": "number (REQUIRED, MIN: 3)",

  "measurements": {
    "total_cells": "number (REQUIRED)",
    "viable_cells": "number (REQUIRED)",
    "viability_percentage": "number (REQUIRED, RANGE: 0-100)"
  },

  "statistics": {
    "mean": "number (REQUIRED)",
    "standard_deviation": "number (REQUIRED)",
    "standard_error": "number (RECOMMENDED)",
    "confidence_interval_95": "array [lower, upper] (RECOMMENDED)"
  }
}
```

---

## 5. Clinical Trial Data Extensions

For clinical applications, additional requirements apply:

### 5.1 Patient Privacy

- NO directly identifying information (PHI) in research databases
- Use study-specific patient IDs
- Geographic data limited to region level
- Age reported in ranges (5-year bins)

### 5.2 Informed Consent

All clinical data MUST include:
- Consent version and date
- Consent language
- Special permissions (research use, data sharing)
- Right to withdraw status

### 5.3 Regulatory Compliance

Clinical data SHALL support:
- FDA 21 CFR Part 1271 (US)
- EU Directive 2004/23/EC (Europe)
- HFEA Code of Practice (UK)
- Local regulatory requirements

---

## 6. Data Exchange and Interoperability

### 6.1 File Formats

**Primary Format:** JSON (UTF-8 encoding)

**Supported Formats:**
- XML (for regulatory submissions)
- CSV (for statistical analysis)
- HDF5 (for large time-series data)

### 6.2 API Specifications

RESTful API endpoints SHALL support:
- POST /experiments (submit new data)
- GET /experiments/{id} (retrieve specific experiment)
- GET /experiments/search (query with filters)
- PUT /experiments/{id} (update with version control)

Authentication: OAuth 2.0 bearer tokens

### 6.3 Controlled Vocabularies

Cell types SHALL use Cell Ontology (CL) terms.
Experimental conditions SHALL use Experimental Factor Ontology (EFO) terms.
Units SHALL use Units Ontology (UO) terms.

---

## 7. Quality Assurance

### 7.1 Validation Rules

All submitted data MUST pass:

1. **Schema Validation:** Conform to JSON schema
2. **Range Checks:** Values within defined ranges
3. **Logic Checks:** Post-thaw viability ≤ pre-freeze viability
4. **Completeness:** All REQUIRED fields present
5. **Format Checks:** Dates, ORCIDs, emails properly formatted

### 7.2 Quality Metrics

Data quality assessed on:
- **Completeness:** % of fields populated (target ≥ 98%)
- **Accuracy:** % passing validation (target ≥ 99%)
- **Timeliness:** Data entry within 24h of experiment (target ≥ 95%)

---

## 8. Metadata and Provenance

### 8.1 Required Metadata

Every dataset SHALL include:
- Standard version (e.g., "WIA-CRYO-010 v2.0")
- Creation timestamp
- Creator ORCID
- Institution
- Funding source (if applicable)
- Ethical approval (if human/animal subjects)
- Data license (recommend CC-BY-4.0)

### 8.2 Provenance Tracking

Changes to data MUST be logged:
- What changed
- Who changed it
- When (timestamp)
- Why (change rationale)
- Previous version reference

---

## 9. Security and Privacy

### 9.1 Encryption

- Data in transit: TLS 1.3 minimum
- Data at rest: AES-256 minimum
- Backups: Encrypted, geographically distributed

### 9.2 Access Control

- Role-based access control (RBAC)
- Audit logging of all access
- Regular access reviews

### 9.3 Data Retention

- Research data: Minimum 10 years post-publication
- Clinical data: Per regulatory requirements (often ≥25 years)
- Audit logs: Minimum 7 years

---

## 10. Versioning and Backward Compatibility

### 10.1 Version Numbering

Format: MAJOR.MINOR.PATCH

- MAJOR: Breaking changes, not backward compatible
- MINOR: New features, backward compatible
- PATCH: Bug fixes, backward compatible

### 10.2 Deprecation Policy

- Features deprecated: Minimum 2 years notice
- Old versions supported: Minimum 5 years after supersession
- Migration tools provided for breaking changes

---

## 11. Implementation Guidance

### 11.1 Conformance Levels

**Level 1 (Basic):** Core fields only, suitable for basic research
**Level 2 (Standard):** All required fields, recommended for publication
**Level 3 (Clinical):** All fields including clinical extensions, required for regulatory submission

### 11.2 Certification

Implementations MAY seek WIA certification:
- Software tools
- Data repositories
- Laboratory information management systems (LIMS)

---

## 12. Changes from Version 1.2

**Major Changes:**
- Added AI/ML model integration support
- Enhanced clinical trial data requirements
- New equipment calibration tracking
- Extended statistical reporting requirements

**Minor Changes:**
- Added support for magnetic nanoparticle protocols
- Improved temperature profile granularity
- Enhanced metadata fields

**Deprecations:**
- Legacy unit formats (use SI units exclusively)

---

## Appendix A: Example Dataset

See accompanying file: `example-dataset-v2.0.json`

---

## Appendix B: Validation Schema

JSON Schema available at: `https://wia.org/schemas/cryo-010/v2.0/schema.json`

---

## Appendix C: Migration Guide

For migrating from v1.2 to v2.0, see: `migration-guide-v1.2-to-v2.0.md`

---

## References

1. Polge, C., Smith, A. U., & Parkes, A. S. (1949). Revival of spermatozoa after vitrification and dehydration at low temperatures. Nature, 164(4172), 666-666.


3. WIA Standards Committee. (2024). Cryopreservation Data Best Practices. WIA Technical Report 2024-001.

---

## Contact and Support

**WIA Standards Committee**
Email: cryo-010@wia.org
Website: https://wia.org/standards/cryo-010
GitHub: https://github.com/WIA-Official/cryo-010

**Document History:**
- v2.0: 2025-01-01 (Current)
- v1.2: 2024-06-15
- v1.1: 2023-12-01
- v1.0: 2023-06-01

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

This specification is licensed under CC-BY-4.0.
