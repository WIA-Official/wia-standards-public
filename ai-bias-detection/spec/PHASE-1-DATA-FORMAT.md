# WIA-AI-013: AI Bias Detection Standard
## Phase 1: Data Format Specification

**Version:** 1.0
**Status:** Final
**Last Updated:** 2025-01-08
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the data formats for AI bias detection and fairness assessment. The format enables standardized exchange of bias detection results, fairness metrics, and model evaluation data across different systems and tools.

### 1.1 Goals

- **Interoperability**: Enable different bias detection tools to exchange data
- **Completeness**: Capture all relevant information for bias assessment
- **Extensibility**: Support future fairness metrics and evaluation methods
- **Human-Readable**: JSON-based format that humans can read and write
- **Machine-Readable**: Strict schema for automated processing

### 1.2 Philosophy

Following 弘益人間, this format emphasizes transparency and accessibility, ensuring that bias detection results can be understood and acted upon by all stakeholders.

---

## 2. Core Data Structures

### 2.1 Bias Detection Report

The primary data structure for bias detection results.

```json
{
  "reportMetadata": {
    "standardVersion": "WIA-AI-013-v1.0",
    "reportId": "string (UUID)",
    "generatedAt": "string (ISO 8601 timestamp)",
    "generatedBy": {
      "tool": "string",
      "version": "string",
      "organization": "string (optional)"
    }
  },
  "modelInfo": {
    "modelId": "string",
    "modelName": "string",
    "modelVersion": "string",
    "modelType": "classification | regression | ranking",
    "taskDescription": "string"
  },
  "datasetInfo": {
    "datasetId": "string",
    "datasetName": "string",
    "totalSamples": "number",
    "dateRange": {
      "start": "string (ISO 8601)",
      "end": "string (ISO 8601)"
    },
    "protectedAttributes": ["array of strings"]
  },
  "fairnessMetrics": {
    // See section 2.2
  },
  "groupMetrics": {
    // See section 2.3
  },
  "biasIndicators": {
    // See section 2.4
  },
  "recommendations": {
    // See section 2.5
  }
}
```

### 2.2 Fairness Metrics

Standardized fairness metrics across all protected attributes.

```json
{
  "demographicParity": {
    "metric": "demographic_parity_ratio",
    "value": "number (0-1)",
    "threshold": "number",
    "status": "pass | fail | warning",
    "byGroup": {
      "groupA": {
        "positiveRate": "number",
        "sampleSize": "number"
      },
      "groupB": {
        "positiveRate": "number",
        "sampleSize": "number"
      }
    }
  },
  "equalizedOdds": {
    "metric": "equalized_odds",
    "tprDisparity": "number",
    "fprDisparity": "number",
    "threshold": "number",
    "status": "pass | fail | warning",
    "byGroup": {
      "groupA": {
        "truePositiveRate": "number",
        "falsePositiveRate": "number",
        "trueNegativeRate": "number",
        "falseNegativeRate": "number"
      }
    }
  },
  "equalOpportunity": {
    "metric": "equal_opportunity",
    "value": "number",
    "threshold": "number",
    "status": "pass | fail | warning",
    "byGroup": {
      "groupA": {"truePositiveRate": "number"}
    }
  },
  "predictiveParity": {
    "metric": "predictive_parity",
    "value": "number",
    "threshold": "number",
    "status": "pass | fail | warning",
    "byGroup": {
      "groupA": {"precision": "number"}
    }
  },
  "disparateImpact": {
    "metric": "disparate_impact_ratio",
    "value": "number",
    "threshold": 0.8,
    "status": "pass | fail | warning",
    "referenceGroup": "string",
    "byGroup": {
      "groupA": {"ratio": "number"}
    }
  }
}
```

### 2.3 Group Performance Metrics

Disaggregated performance by demographic group.

```json
{
  "byProtectedAttribute": {
    "gender": {
      "male": {
        "sampleSize": "number",
        "accuracy": "number",
        "precision": "number",
        "recall": "number",
        "f1Score": "number",
        "confusionMatrix": {
          "truePositive": "number",
          "falsePositive": "number",
          "trueNegative": "number",
          "falseNegative": "number"
        }
      },
      "female": { /* same structure */ }
    },
    "race": { /* same structure per group */ }
  },
  "intersectional": {
    "female_black": {
      "sampleSize": "number",
      "accuracy": "number",
      /* other metrics */
    }
  }
}
```

### 2.4 Bias Indicators

Specific bias patterns detected in the model.

```json
{
  "biasTypes": [
    {
      "type": "selection | measurement | label | algorithmic | aggregation | temporal",
      "severity": "low | medium | high | critical",
      "confidence": "number (0-1)",
      "description": "string",
      "affectedGroups": ["array of group identifiers"],
      "evidence": {
        "metricName": "string",
        "observedValue": "number",
        "expectedValue": "number",
        "deviation": "number"
      }
    }
  ],
  "proxyFeatures": [
    {
      "featureName": "string",
      "protectedAttribute": "string",
      "correlation": "number",
      "importance": "number",
      "recommendation": "remove | review | retain"
    }
  ]
}
```

### 2.5 Recommendations

Actionable recommendations for bias mitigation.

```json
{
  "priority": "immediate | high | medium | low",
  "recommendations": [
    {
      "id": "string",
      "category": "data | model | deployment | monitoring",
      "phase": "pre-processing | in-processing | post-processing",
      "title": "string",
      "description": "string",
      "expectedImpact": "string",
      "implementationEffort": "low | medium | high",
      "resources": ["array of URLs or references"]
    }
  ]
}
```

---

## 3. Data Exchange Format

### 3.1 File Format

- **Primary Format**: JSON (.json)
- **Encoding**: UTF-8
- **Compression**: Optional gzip (.json.gz)
- **Schema Validation**: JSON Schema provided

### 3.2 API Response Format

When transmitted via API:

```json
{
  "success": true,
  "data": {
    /* BiasDetectionReport structure */
  },
  "errors": [],
  "warnings": []
}
```

### 3.3 Batch Format

For multiple reports:

```json
{
  "batchMetadata": {
    "batchId": "string",
    "createdAt": "string (ISO 8601)",
    "totalReports": "number"
  },
  "reports": [
    /* Array of BiasDetectionReport */
  ]
}
```

---

## 4. Metadata Standards

### 4.1 Protected Attributes

Standardized attribute names:

```json
{
  "protectedAttributes": {
    "demographic": ["age", "gender", "race", "ethnicity", "nationality"],
    "socioeconomic": ["income", "education", "occupation"],
    "geographic": ["zipcode", "region", "country"],
    "other": ["religion", "disability", "marital_status"]
  }
}
```

### 4.2 Metric Thresholds

Default thresholds for fairness metrics:

```json
{
  "thresholds": {
    "demographicParityRatio": 0.8,
    "equalizedOddsDisparity": 0.1,
    "disparateImpactRatio": 0.8,
    "calibrationError": 0.05
  }
}
```

---

## 5. Validation Rules

### 5.1 Required Fields

- reportMetadata.standardVersion
- reportMetadata.reportId
- reportMetadata.generatedAt
- modelInfo.modelId
- datasetInfo.totalSamples
- fairnessMetrics (at least one metric)

### 5.2 Value Constraints

- All ratios: 0.0 to 1.0
- All percentages: 0 to 100
- Sample sizes: positive integers
- Timestamps: ISO 8601 format
- UUIDs: RFC 4122 format

### 5.3 Consistency Rules

- Sum of confusion matrix = sample size
- Precision/recall calculable from confusion matrix
- Group sample sizes sum to total
- All referenced groups have corresponding data

---

## 6. Extensions

### 6.1 Custom Metrics

Support for organization-specific metrics:

```json
{
  "customMetrics": {
    "metricName": {
      "definition": "string",
      "value": "number",
      "threshold": "number",
      "status": "pass | fail | warning"
    }
  }
}
```

### 6.2 Audit Trail

Optional audit information:

```json
{
  "auditTrail": {
    "createdBy": "string",
    "reviewedBy": ["array of strings"],
    "approvedBy": "string",
    "reviewComments": ["array of comment objects"],
    "version": "number"
  }
}
```

---

## 7. Examples

### 7.1 Minimal Valid Report

```json
{
  "reportMetadata": {
    "standardVersion": "WIA-AI-013-v1.0",
    "reportId": "550e8400-e29b-41d4-a716-446655440000",
    "generatedAt": "2025-01-08T10:30:00Z",
    "generatedBy": {
      "tool": "BiasDetector",
      "version": "1.0.0"
    }
  },
  "modelInfo": {
    "modelId": "model-001",
    "modelName": "Credit Scoring Model",
    "modelVersion": "2.1",
    "modelType": "classification",
    "taskDescription": "Predict credit default risk"
  },
  "datasetInfo": {
    "datasetId": "dataset-001",
    "datasetName": "Credit Applications 2024",
    "totalSamples": 10000,
    "protectedAttributes": ["gender", "race"]
  },
  "fairnessMetrics": {
    "demographicParity": {
      "metric": "demographic_parity_ratio",
      "value": 0.75,
      "threshold": 0.8,
      "status": "fail",
      "byGroup": {
        "male": {"positiveRate": 0.45, "sampleSize": 6000},
        "female": {"positiveRate": 0.34, "sampleSize": 4000}
      }
    }
  }
}
```

---

## 8. Implementation Guidelines

### 8.1 Producers

Systems generating bias detection reports should:

1. Validate output against JSON schema
2. Include all required fields
3. Use standardized attribute names
4. Calculate metrics consistently
5. Provide clear status indicators

### 8.2 Consumers

Systems reading reports should:

1. Validate input against schema
2. Handle missing optional fields gracefully
3. Support all standard metrics
4. Respect status thresholds
5. Preserve audit trail information

---

## 9. Compliance

### 9.1 弘益人間 Requirements

Implementations must:

- Ensure transparency in all calculations
- Provide human-readable explanations
- Support accessibility (screen readers, etc.)
- Enable stakeholder review
- Maintain privacy of individual records

### 9.2 Certification

To be WIA-AI-013 Phase 1 certified:

- ✓ Implement all required data structures
- ✓ Support standard fairness metrics
- ✓ Validate against JSON schema
- ✓ Provide examples and documentation
- ✓ Pass interoperability test suite

---

**Document Version:** 1.0
**Effective Date:** 2025-01-08
**Maintained By:** WIA Standards Committee
**License:** CC BY 4.0

弘益人間 · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-bias-detection is evaluated across three tiers:

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

- `wia-standards/standards/ai-bias-detection/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-bias-detection/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-bias-detection/simulator/` — interactive browser-based simulator for the PHASE protocol

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
