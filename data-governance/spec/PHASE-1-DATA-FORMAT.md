# WIA-DATA-006: Data Governance - PHASE 1: DATA FORMAT

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-01-15

---

## Overview

Phase 1 establishes the foundational data formats and structures for implementing data governance across your organization. This phase defines how governance metadata, policies, and catalogs are represented and stored.

## Goals

- Define standard data formats for governance artifacts
- Establish metadata schemas for data assets
- Create policy document structures
- Enable interoperability between governance tools

## Core Data Formats

### 1. Data Asset Metadata Format

```json
{
  "assetId": "uuid-v4",
  "assetName": "string",
  "assetType": "database|table|file|api|stream",
  "classification": "public|internal|confidential|restricted",
  "domain": "string",
  "description": "string",
  "owner": {
    "email": "string",
    "name": "string",
    "department": "string"
  },
  "stewards": [
    {
      "email": "string",
      "name": "string",
      "role": "business|technical|executive"
    }
  ],
  "technicalMetadata": {
    "location": "string",
    "format": "string",
    "schema": {},
    "size": "number",
    "rowCount": "number",
    "lastUpdated": "ISO-8601 datetime"
  },
  "businessMetadata": {
    "businessTerms": ["string"],
    "tags": ["string"],
    "certificationStatus": "certified|draft|deprecated",
    "usageGuidelines": "string"
  },
  "qualityMetadata": {
    "qualityScore": "number (0-100)",
    "completeness": "number (0-100)",
    "accuracy": "number (0-100)",
    "consistency": "number (0-100)",
    "timeliness": "number (0-100)",
    "lastAssessed": "ISO-8601 datetime"
  },
  "lineage": {
    "upstreamAssets": ["assetId"],
    "downstreamAssets": ["assetId"],
    "transformations": [
      {
        "transformationId": "string",
        "description": "string",
        "logic": "string"
      }
    ]
  },
  "compliance": {
    "regulations": ["GDPR|CCPA|HIPAA|SOX"],
    "retentionPeriod": "ISO-8601 duration",
    "privacyLevel": "pii|sensitive|non-sensitive"
  },
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 2. Policy Document Format

```json
{
  "policyId": "uuid-v4",
  "policyName": "string",
  "policyType": "quality|security|privacy|retention|access",
  "version": "string (semver)",
  "status": "draft|active|deprecated",
  "effectiveDate": "ISO-8601 date",
  "expirationDate": "ISO-8601 date",
  "owner": {
    "email": "string",
    "name": "string"
  },
  "approvers": [
    {
      "email": "string",
      "name": "string",
      "approvedAt": "ISO-8601 datetime"
    }
  ],
  "scope": {
    "domains": ["string"],
    "assetTypes": ["string"],
    "applicability": "organization|domain|asset"
  },
  "statement": "string",
  "requirements": [
    {
      "requirementId": "string",
      "description": "string",
      "mandatory": "boolean",
      "controls": ["string"]
    }
  ],
  "exceptions": [
    {
      "exceptionId": "string",
      "description": "string",
      "approver": "string",
      "expiresAt": "ISO-8601 datetime"
    }
  ],
  "relatedPolicies": ["policyId"],
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 3. Business Glossary Term Format

```json
{
  "termId": "uuid-v4",
  "termName": "string",
  "definition": "string",
  "synonyms": ["string"],
  "relatedTerms": ["termId"],
  "domain": "string",
  "owner": {
    "email": "string",
    "name": "string"
  },
  "status": "draft|approved|deprecated",
  "examples": ["string"],
  "businessRules": [
    {
      "ruleId": "string",
      "description": "string",
      "expression": "string"
    }
  ],
  "dataAssets": ["assetId"],
  "approvedBy": {
    "email": "string",
    "name": "string",
    "approvedAt": "ISO-8601 datetime"
  },
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 4. Access Request Format

```json
{
  "requestId": "uuid-v4",
  "requestType": "access|modification|deletion",
  "requester": {
    "email": "string",
    "name": "string",
    "department": "string"
  },
  "assetId": "string",
  "accessLevel": "read|write|admin",
  "justification": "string",
  "duration": "ISO-8601 duration",
  "status": "pending|approved|rejected|expired",
  "workflow": [
    {
      "step": "number",
      "approver": "string",
      "status": "pending|approved|rejected",
      "comments": "string",
      "timestamp": "ISO-8601 datetime"
    }
  ],
  "grantedAt": "ISO-8601 datetime",
  "expiresAt": "ISO-8601 datetime",
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

### 5. Data Quality Rule Format

```json
{
  "ruleId": "uuid-v4",
  "ruleName": "string",
  "ruleType": "completeness|accuracy|consistency|timeliness|validity",
  "description": "string",
  "assetId": "string",
  "field": "string",
  "expression": "string",
  "threshold": {
    "operator": "gt|gte|lt|lte|eq|ne",
    "value": "number"
  },
  "severity": "critical|high|medium|low",
  "active": "boolean",
  "schedule": "cron expression",
  "lastExecuted": "ISO-8601 datetime",
  "lastResult": {
    "passed": "boolean",
    "score": "number",
    "failureCount": "number",
    "executedAt": "ISO-8601 datetime"
  },
  "notifications": [
    {
      "email": "string",
      "severity": ["critical|high|medium|low"]
    }
  ],
  "createdAt": "ISO-8601 datetime",
  "updatedAt": "ISO-8601 datetime"
}
```

## File Formats

### Supported Formats

1. **JSON** - Primary format for APIs and modern systems
2. **YAML** - Human-readable format for configuration
3. **XML** - Legacy system compatibility
4. **CSV** - Bulk imports and exports
5. **Parquet** - Large-scale data processing

### Naming Conventions

- **Metadata Files:** `{asset-type}_{asset-name}_metadata.json`
- **Policy Files:** `policy_{policy-type}_{version}.json`
- **Glossary Files:** `glossary_{domain}_{timestamp}.json`
- **Quality Rules:** `quality_rules_{asset-id}.json`

## Data Storage Requirements

### Metadata Repository

- **Technology:** JSON documents in document database (MongoDB, DynamoDB) or relational database
- **Versioning:** All metadata changes must be versioned
- **Retention:** Minimum 7 years for compliance
- **Backup:** Daily incremental, weekly full backups

### Policy Store

- **Technology:** Version-controlled repository (Git)
- **Format:** JSON or YAML with schema validation
- **Approval:** Digital signatures for policy approvals
- **History:** Complete audit trail of changes

## Validation

### Schema Validation

All data formats must be validated against JSON Schema:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["assetId", "assetName", "assetType", "owner"],
  "properties": {
    "assetId": {
      "type": "string",
      "format": "uuid"
    },
    "assetName": {
      "type": "string",
      "minLength": 1,
      "maxLength": 255
    }
  }
}
```

### Quality Checks

- **Completeness:** All required fields must be populated
- **Accuracy:** Data types and formats must match schema
- **Consistency:** Cross-references must be valid
- **Timeliness:** Timestamps must be current

## Migration Path

### From Legacy Systems

1. **Assessment:** Identify existing governance data
2. **Mapping:** Map legacy formats to WIA-DATA-006 formats
3. **Transformation:** Convert data using ETL processes
4. **Validation:** Verify data integrity
5. **Cutover:** Switch to new format

### Backward Compatibility

- Support legacy formats for 12 months
- Provide conversion utilities
- Document migration process
- Offer support during transition

## Next Steps

After implementing Phase 1 data formats:

1. **Test** data format implementations
2. **Validate** schema compliance
3. **Document** any customizations
4. **Proceed** to Phase 2: API implementation

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
