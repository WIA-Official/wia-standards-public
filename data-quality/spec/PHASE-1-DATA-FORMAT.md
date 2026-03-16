# WIA-DATA-005: Data Quality - Phase 1 Data Format Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This document defines the data format specifications for the WIA-DATA-005 Data Quality standard. It establishes standardized formats for representing data quality metadata, rules, metrics, and validation results.

## Core Data Structures

### 1. Data Quality Profile

Represents comprehensive quality profile of a dataset.

```json
{
  "profileId": "string (UUID)",
  "datasetId": "string",
  "datasetName": "string",
  "createdAt": "ISO8601 timestamp",
  "profiledBy": "string (user/system ID)",
  "statistics": {
    "totalRows": "integer",
    "totalColumns": "integer",
    "dataSize": "integer (bytes)"
  },
  "dimensions": {
    "accuracy": "DimensionScore",
    "completeness": "DimensionScore",
    "consistency": "DimensionScore",
    "timeliness": "DimensionScore",
    "validity": "DimensionScore",
    "uniqueness": "DimensionScore"
  },
  "columns": ["ColumnProfile"]
}
```

### 2. Dimension Score

Represents quality score for a specific dimension.

```json
{
  "dimension": "string (accuracy|completeness|consistency|timeliness|validity|uniqueness)",
  "score": "float (0-100)",
  "status": "string (pass|warning|fail)",
  "threshold": "float (0-100)",
  "issues": ["Issue"],
  "measuredAt": "ISO8601 timestamp"
}
```

### 3. Column Profile

Detailed statistics for individual column.

```json
{
  "columnName": "string",
  "dataType": "string (string|integer|float|date|boolean)",
  "statistics": {
    "totalCount": "integer",
    "nullCount": "integer",
    "nullPercentage": "float",
    "uniqueCount": "integer",
    "uniquePercentage": "float",
    "minValue": "any",
    "maxValue": "any",
    "mean": "float (numeric only)",
    "median": "float (numeric only)",
    "stdDev": "float (numeric only)"
  },
  "patterns": ["PatternMatch"],
  "topValues": ["ValueFrequency"]
}
```

### 4. Validation Rule

Defines a data quality validation rule.

```json
{
  "ruleId": "string (UUID)",
  "ruleName": "string",
  "description": "string",
  "ruleType": "string (format|range|uniqueness|referential|cross-field|completeness)",
  "severity": "string (critical|high|medium|low)",
  "target": {
    "dataset": "string",
    "column": "string (optional)",
    "columns": ["string"] (for cross-field rules)
  },
  "condition": {
    "operator": "string (eq|ne|gt|lt|gte|lte|in|not_in|regex|null|not_null)",
    "value": "any",
    "pattern": "string (regex pattern)"
  },
  "createdAt": "ISO8601 timestamp",
  "createdBy": "string",
  "active": "boolean"
}
```

### 5. Validation Result

Result of executing validation rules.

```json
{
  "resultId": "string (UUID)",
  "validationId": "string (UUID)",
  "ruleId": "string (UUID)",
  "executedAt": "ISO8601 timestamp",
  "status": "string (pass|fail)",
  "recordsChecked": "integer",
  "recordsPassed": "integer",
  "recordsFailed": "integer",
  "passRate": "float (0-100)",
  "failedRecords": ["FailedRecord"],
  "executionTime": "integer (milliseconds)"
}
```

### 6. Quality Metric

Time-series quality metric.

```json
{
  "metricId": "string (UUID)",
  "metricName": "string",
  "datasetId": "string",
  "dimension": "string",
  "value": "float",
  "unit": "string (percentage|count|seconds)",
  "timestamp": "ISO8601 timestamp",
  "tags": {
    "key": "value"
  }
}
```

### 7. Issue

Represents a data quality issue.

```json
{
  "issueId": "string (UUID)",
  "issueType": "string (missing|duplicate|invalid|outlier|inconsistent)",
  "severity": "string (critical|high|medium|low)",
  "description": "string",
  "detectedAt": "ISO8601 timestamp",
  "location": {
    "dataset": "string",
    "column": "string (optional)",
    "row": "integer (optional)",
    "value": "any (optional)"
  },
  "status": "string (open|in_progress|resolved|ignored)",
  "assignedTo": "string (optional)",
  "resolvedAt": "ISO8601 timestamp (optional)",
  "resolution": "string (optional)"
}
```

## Data Exchange Formats

### CSV Format

For bulk import/export of validation rules:

```csv
ruleId,ruleName,ruleType,severity,dataset,column,operator,value
uuid-1,Email Format,format,high,customers,email,regex,^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$
uuid-2,Age Range,range,medium,customers,age,between,18-120
```

### JSON Lines Format

For streaming quality metrics:

```jsonl
{"metricId":"uuid-1","metricName":"completeness","datasetId":"customers","dimension":"completeness","value":95.5,"unit":"percentage","timestamp":"2025-12-26T10:00:00Z"}
{"metricId":"uuid-2","metricName":"accuracy","datasetId":"customers","dimension":"accuracy","value":92.3,"unit":"percentage","timestamp":"2025-12-26T10:00:00Z"}
```

## Naming Conventions

### Field Names
- Use camelCase for JSON fields
- Use snake_case for database columns
- Be descriptive and avoid abbreviations

### IDs
- Use UUID v4 for all identifiers
- Format: `xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx`

### Timestamps
- Use ISO 8601 format: `YYYY-MM-DDTHH:mm:ss.sssZ`
- Always UTC timezone

### Enumerations
- Use lowercase with hyphens for multi-word values
- Examples: `pass|fail|warning`, `data-format|api|protocol`

## Validation

All data formats MUST be validated against JSON Schema definitions provided in this specification.

Example JSON Schema for DataQualityProfile:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DataQualityProfile",
  "type": "object",
  "required": ["profileId", "datasetId", "createdAt"],
  "properties": {
    "profileId": {
      "type": "string",
      "format": "uuid"
    },
    "datasetId": {
      "type": "string"
    },
    "createdAt": {
      "type": "string",
      "format": "date-time"
    },
    "statistics": {
      "type": "object",
      "properties": {
        "totalRows": {"type": "integer", "minimum": 0},
        "totalColumns": {"type": "integer", "minimum": 0}
      }
    }
  }
}
```

## Versioning

Data format versions follow Semantic Versioning (SemVer):
- MAJOR.MINOR.PATCH
- Breaking changes increment MAJOR
- New features increment MINOR
- Bug fixes increment PATCH

## Compatibility

Implementations MUST support:
- JSON as primary format
- CSV for bulk operations
- JSON Lines for streaming

Implementations SHOULD support:
- Avro for big data pipelines
- Parquet for analytics workloads
- Protocol Buffers for high-performance scenarios

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
