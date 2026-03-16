# WIA-CRYO-010 Specification v1.1

**Cryopreservation Research Data Standard**

**Status:** Stable (Superseded by v2.0)
**Release Date:** 2023-12-01
**Supersedes:** WIA-CRYO-010 v1.0
**Maintained By:** WIA Standards Committee

---

## Abstract

WIA-CRYO-010 v1.1 extends the foundational standard with enhanced statistical requirements and organ preservation support.

**Philosophy:** 弘익人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Scope

Extended to include:
- Cell, tissue, AND organ cryopreservation
- Statistical analysis requirements
- Equipment calibration tracking
- Multi-component cryoprotectant solutions

---

## 2. Core Data Structure

```json
{
  "standard": "WIA-CRYO-010",
  "version": "1.1",
  "experiment": {
    "id": "string (REQUIRED, FORMAT: CRYO-YYYY-NNN)",
    "type": "cell|tissue|organ|embryo",
    "researcher": {
      "name": "string (REQUIRED)",
      "orcid": "string (RECOMMENDED)",
      "institution": "string"
    },
    "date": "ISO8601 (REQUIRED)"
  },
  "temperature_profile": {
    "unit": "celsius",
    "data_points": [
      {
        "time": "number",
        "temperature": "number"
      }
    ],
    "cooling_rate": "number"
  },
  "cryoprotectant": {
    "components": [
      {
        "name": "string",
        "concentration": "number",
        "unit": "percent_v/v|percent_w/v"
      }
    ]
  },
  "viability_assessment": {
    "method": "string (REQUIRED)",
    "timepoint": "number (hours)",
    "measurements": {
      "pre_freeze": "number",
      "post_thaw": "number",
      "n": "number (REQUIRED, MIN: 3)"
    },
    "statistics": {
      "mean": "number",
      "std_dev": "number"
    }
  },
  "equipment": {
    "model": "string",
    "calibration_date": "ISO8601"
  }
}
```

---

## 3. New Features in v1.1

### 3.1 Statistical Requirements

- Sample size (n) now REQUIRED
- Mean and standard deviation REQUIRED for viability data
- Confidence intervals RECOMMENDED

### 3.2 Equipment Tracking

- Equipment model documentation
- Calibration date tracking
- Settings documentation

### 3.3 Multi-Component CPAs

- Support for complex cryoprotectant solutions
- Multiple components with individual concentrations
- Total must sum to 100%

### 3.4 ORCID Integration

- Researcher identification using ORCID
- Enables better attribution and tracking

---

## 4. Temperature Profile Enhancements

Enhanced requirements:
- Time-series data (not just rates)
- Minimum 5 data points (increased from 3)
- Unit specification required

---

## 5. Validation Rules

New validation requirements:
- Experiment ID format: CRYO-YYYY-NNN
- Sample size n ≥ 3
- Equipment calibration within 12 months
- CPA concentrations sum to 100%

---

## 6. File Formats

Enhanced format support:
- JSON (primary, with schema validation)
- XML (added for regulatory compatibility)
- CSV (statistical analysis)

---

## 7. Metadata Enhancements

Additional metadata fields:
- Funding source
- Keywords
- Related publications (DOI)

---

## Changes from Version 1.0

**Additions:**
- Statistical requirements
- Equipment calibration tracking
- Multi-component CPA support
- ORCID researcher identification
- Enhanced validation rules

**Improvements:**
- More detailed temperature profiles
- Structured researcher information
- Better documentation requirements

**Backward Compatibility:**
- v1.0 data can be migrated to v1.1 format
- Migration tool available

---

## Deprecation Notices

None in this version (fully backward compatible)

---

© 2023 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
