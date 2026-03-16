# WIA-CRYO-010 Specification v1.2

**Cryopreservation Research Data Standard**

**Status:** Stable (Superseded by v2.0)
**Release Date:** 2024-06-15
**Supersedes:** WIA-CRYO-010 v1.1
**Maintained By:** WIA Standards Committee

---

## Abstract

WIA-CRYO-010 v1.2 adds clinical trial support, enhanced privacy protections, and improved collaboration features.

**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Scope

Extended to include:
- Clinical trial data management
- Patient privacy protections (HIPAA, GDPR)
- Multi-center study coordination
- Regulatory submission support

---

## 2. Core Data Structure

```json
{
  "standard": "WIA-CRYO-010",
  "version": "1.2",
  "experiment": {
    "id": "string (REQUIRED, UNIQUE)",
    "type": "enum (cell|tissue|organ|embryo)",
    "title": "string",
    "researcher": {
      "name": "string (REQUIRED)",
      "orcid": "string (REQUIRED)",
      "institution": "string (REQUIRED)",
      "email": "string (REQUIRED)"
    },
    "date": "ISO8601 (REQUIRED)"
  },
  "sample": {
    "type": "string (REQUIRED)",
    "source": "string",
    "quantity": "number",
    "unit": "string"
  },
  "protocol": {
    "method": "enum (slow_freeze|vitrification|directional)",
    "temperature_profile": {
      "unit": "celsius",
      "data_points": "array (MIN_LENGTH: 3)",
      "statistics": {
        "mean_cooling_rate": "number",
        "temperature_uniformity": "number"
      }
    },
    "cryoprotectant": {
      "components": "array",
      "osmolality": "number",
      "pH": "number"
    }
  },
  "measurements": {
    "pre_freeze_viability": "number (0-100)",
    "post_thaw_viability": "number (0-100)",
    "assessment_timepoint": "number",
    "method": "string"
  },
  "quality_control": {
    "equipment_calibration": "ISO8601 date",
    "protocol_deviations": "array"
  }
}
```

---

## 3. Clinical Trial Extensions

### 3.1 Patient Data

For clinical applications:

```json
"clinical_trial": {
  "study_id": "string (REQUIRED)",
  "patient": {
    "study_id": "string (REQUIRED, NO PHI)",
    "age_range": "string (e.g., 30-35)",
    "sex": "M|F|O",
    "consent_version": "string",
    "consent_date": "ISO8601"
  },
  "irb_approval": "string",
  "data_classification": "public|restricted|confidential"
}
```

### 3.2 Privacy Protections

- NO directly identifying information (PHI/PII)
- De-identification required
- Encryption at rest and in transit
- Audit logging of all access

### 3.3 Informed Consent

- Consent version tracking
- Special permissions (research use, data sharing)
- Withdrawal tracking

---

## 4. Enhanced Statistical Requirements

Required statistical measures:
- Sample size (n ≥ 3 biological replicates)
- Mean and standard deviation
- Standard error (recommended)
- 95% confidence intervals (recommended)
- Statistical test used (for comparisons)
- P-values (for comparisons)

---

## 5. Multi-Center Collaboration

Support for:
- Centralized data repositories
- Site-specific adaptations
- Protocol harmonization
- Inter-laboratory comparisons

---

## 6. Data Exchange

### 6.1 API Support

RESTful API endpoints:
- POST /experiments
- GET /experiments/{id}
- GET /experiments/search
- PUT /experiments/{id}

### 6.2 Authentication

- OAuth 2.0 bearer tokens
- Role-based access control
- API key management

---

## 7. Quality Assurance Enhancements

### 7.1 Validation Rules

- Schema validation against JSON schema
- Range checks on numerical values
- Logic checks (e.g., post-thaw ≤ pre-freeze)
- Format validation (dates, emails, ORCIDs)

### 7.2 Quality Metrics

- Completeness: ≥98% of fields populated
- Accuracy: ≥99% passing validation
- Timeliness: Data entry within 24h

---

## 8. Metadata and Provenance

Enhanced metadata:
- Funding information
- Ethical approval details
- Data license (CC-BY-4.0 recommended)
- Related publications
- Version control and change logging

---

## 9. Security Requirements

- TLS 1.2+ for data in transit
- AES-256 for data at rest
- Regular security audits
- Incident response procedures

---

## 10. File Formats

Supported formats:
- JSON (primary, UTF-8)
- XML (regulatory submissions)
- CSV (statistical analysis)
- HDF5 (large time-series, new in v1.2)

---

## Changes from Version 1.1

**Major Additions:**
- Clinical trial data extensions
- Privacy and security enhancements
- Multi-center collaboration support
- RESTful API specifications

**Improvements:**
- Enhanced statistical requirements
- Better quality assurance
- Improved metadata tracking
- HDF5 format support

**Backward Compatibility:**
- v1.1 data fully compatible
- Migration guide available
- Validation tools updated

---

## Deprecation Notices

- Legacy date formats (use ISO8601 exclusively)
- Non-SI temperature units (use Celsius or Kelvin)

---

## Migration from v1.1

Key changes for migration:
1. Add ORCID to all researcher records
2. Add email addresses
3. Enhance statistical reporting
4. Add equipment calibration dates
5. Implement privacy controls for clinical data

Migration tool: `wia-cryo-migrate-v1.1-to-v1.2`

---

© 2024 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
