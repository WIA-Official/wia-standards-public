# WIA-DATA-009: Master Data Management Standard
## PHASE 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Draft  
**Last Updated:** 2025-12-26

---

## Overview

This phase defines the core data formats and structures for representing master data entities, relationships, and metadata according to the WIA-DATA-009 standard. It establishes the foundational data models that enable interoperability across MDM implementations.

## 1. Core Data Structures

### 1.1 Master Data Entity

Represents a master data record (customer, product, supplier, etc.).

```json
{
  "entityId": "string (uuid)",
  "entityType": "customer | product | supplier | location | asset | employee",
  "sourceSystem": "string",
  "sourceSystemId": "string",
  "goldenRecordId": "string (uuid)",
  "status": "active | inactive | pending | archived",
  "effectiveDate": "ISO8601 timestamp",
  "expirationDate": "ISO8601 timestamp",
  "createdAt": "ISO8601 timestamp",
  "updatedAt": "ISO8601 timestamp",
  "metadata": {
    "dataQualityScore": "integer (0-100)",
    "completenessScore": "integer (0-100)",
    "trustScore": "integer (0-100)",
    "lastVerifiedDate": "ISO8601 timestamp",
    "steward": "string",
    "businessOwner": "string",
    "tags": ["string"],
    "custom": {}
  },
  "attributes": {}
}
```

**Field Descriptions:**

- `entityId`: Unique identifier (UUID v4) for this entity instance
- `entityType`: Type of master data entity
- `sourceSystem`: Identifier of the source system
- `sourceSystemId`: Original identifier in source system
- `goldenRecordId`: Reference to the golden record this entity contributes to
- `status`: Current lifecycle status of the entity
- `effectiveDate`: When this version of the entity became effective
- `expirationDate`: When this version expires (null if current)
- `metadata`: Quality and governance metadata
- `attributes`: Domain-specific attributes (see domain models below)

### 1.2 Customer Entity Model

```json
{
  "entityId": "uuid",
  "entityType": "customer",
  "attributes": {
    "partyType": "person | organization",
    "person": {
      "firstName": "string",
      "middleName": "string",
      "lastName": "string",
      "suffix": "string",
      "gender": "male | female | other | unknown",
      "dateOfBirth": "ISO8601 date",
      "nationality": "ISO 3166-1 alpha-2",
      "maritalStatus": "string"
    },
    "organization": {
      "legalName": "string",
      "tradeName": "string",
      "taxId": "string",
      "registrationNumber": "string",
      "industry": "NAICS code",
      "numberOfEmployees": "integer",
      "annualRevenue": "decimal",
      "fiscalYearEnd": "MM-DD"
    },
    "contactMethods": [
      {
        "contactType": "email | phone | mobile | fax | social",
        "contactValue": "string",
        "isPrimary": "boolean",
        "isVerified": "boolean",
        "effectiveDate": "ISO8601 timestamp"
      }
    ],
    "addresses": [
      {
        "addressType": "billing | shipping | mailing | physical",
        "addressLine1": "string",
        "addressLine2": "string",
        "city": "string",
        "stateProvince": "string",
        "postalCode": "string",
        "country": "ISO 3166-1 alpha-2",
        "isPrimary": "boolean",
        "isVerified": "boolean",
        "effectiveDate": "ISO8601 timestamp"
      }
    ],
    "segments": ["string"],
    "preferences": {
      "language": "ISO 639-1",
      "currency": "ISO 4217",
      "timezone": "IANA timezone",
      "communicationPreferences": {
        "email": "boolean",
        "sms": "boolean",
        "phone": "boolean",
        "mail": "boolean"
      }
    },
    "relationships": [
      {
        "relatedEntityId": "uuid",
        "relationshipType": "parent | child | spouse | sibling | employee | subsidiary",
        "effectiveDate": "ISO8601 timestamp",
        "expirationDate": "ISO8601 timestamp"
      }
    ]
  }
}
```

### 1.3 Product Entity Model

```json
{
  "entityId": "uuid",
  "entityType": "product",
  "attributes": {
    "productIdentifiers": {
      "sku": "string",
      "upc": "string",
      "gtin": "string",
      "partNumber": "string",
      "manufacturerPartNumber": "string"
    },
    "productInfo": {
      "name": "string",
      "description": "string",
      "brand": "string",
      "manufacturer": "string",
      "category": "string",
      "productLine": "string",
      "productFamily": "string"
    },
    "specifications": {
      "dimensions": {
        "length": {"value": "decimal", "unit": "string"},
        "width": {"value": "decimal", "unit": "string"},
        "height": {"value": "decimal", "unit": "string"},
        "weight": {"value": "decimal", "unit": "string"}
      },
      "attributes": [
        {"name": "string", "value": "string", "unit": "string"}
      ]
    },
    "commercialData": {
      "listPrice": {"amount": "decimal", "currency": "ISO 4217"},
      "cost": {"amount": "decimal", "currency": "ISO 4217"},
      "taxCategory": "string",
      "isActive": "boolean",
      "availableDate": "ISO8601 date",
      "discontinuedDate": "ISO8601 date"
    },
    "operationalData": {
      "hazmatClass": "string",
      "shelfLife": {"value": "integer", "unit": "days"},
      "storageRequirements": "string",
      "handlingInstructions": "string"
    },
    "hierarchy": {
      "parentProductId": "uuid",
      "childProductIds": ["uuid"],
      "hierarchyType": "variant | bundle | kit | assembly"
    }
  }
}
```

### 1.4 Supplier Entity Model

```json
{
  "entityId": "uuid",
  "entityType": "supplier",
  "attributes": {
    "supplierInfo": {
      "legalName": "string",
      "tradeName": "string",
      "taxId": "string",
      "dunsNumber": "string",
      "supplierType": "manufacturer | distributor | service_provider | contractor"
    },
    "financialData": {
      "paymentTerms": "string",
      "creditLimit": {"amount": "decimal", "currency": "ISO 4217"},
      "bankAccounts": [
        {
          "accountNumber": "string",
          "routingNumber": "string",
          "bankName": "string",
          "accountType": "checking | savings | wire",
          "isPrimary": "boolean"
        }
      ]
    },
    "complianceData": {
      "certifications": [
        {"type": "string", "number": "string", "expiryDate": "ISO8601 date"}
      ],
      "auditResults": [
        {"auditDate": "ISO8601 date", "score": "integer", "findings": "string"}
      ],
      "riskRating": "low | medium | high | critical"
    },
    "performanceMetrics": {
      "onTimeDeliveryRate": "decimal (0-1)",
      "qualityScore": "integer (0-100)",
      "totalSpend": {"amount": "decimal", "currency": "ISO 4217"},
      "lastEvaluationDate": "ISO8601 date"
    },
    "contracts": [
      {
        "contractId": "string",
        "contractType": "master | purchase_order | service_agreement",
        "startDate": "ISO8601 date",
        "endDate": "ISO8601 date",
        "value": {"amount": "decimal", "currency": "ISO 4217"}
      }
    ]
  }
}
```

## 2. Matching and Linking Structures

### 2.1 Match Candidate

Represents a potential duplicate or related entity.

```json
{
  "matchId": "uuid",
  "entity1Id": "uuid",
  "entity2Id": "uuid",
  "matchScore": "decimal (0-1)",
  "matchType": "exact | probabilistic | fuzzy | ml_predicted",
  "matchAlgorithm": "string",
  "matchedAttributes": [
    {
      "attributeName": "string",
      "similarity": "decimal (0-1)",
      "entity1Value": "string",
      "entity2Value": "string"
    }
  ],
  "status": "pending | confirmed | rejected | auto_merged",
  "reviewedBy": "string",
  "reviewedAt": "ISO8601 timestamp",
  "createdAt": "ISO8601 timestamp"
}
```

### 2.2 Golden Record

Represents the merged, authoritative version of an entity.

```json
{
  "goldenRecordId": "uuid",
  "entityType": "string",
  "status": "active | inactive | pending_review",
  "attributes": {},
  "sourceEntities": [
    {
      "entityId": "uuid",
      "sourceSystem": "string",
      "contributionScore": "decimal (0-1)",
      "linkedAt": "ISO8601 timestamp"
    }
  ],
  "survivorshipRules": [
    {
      "attributeName": "string",
      "rule": "most_complete | most_recent | most_trusted | highest_quality",
      "selectedSourceEntityId": "uuid"
    }
  ],
  "qualityMetrics": {
    "overallScore": "integer (0-100)",
    "completeness": "integer (0-100)",
    "accuracy": "integer (0-100)",
    "consistency": "integer (0-100)",
    "timeliness": "integer (0-100)"
  },
  "createdAt": "ISO8601 timestamp",
  "updatedAt": "ISO8601 timestamp",
  "version": "integer"
}
```

## 3. Relationship Structures

### 3.1 Entity Relationship

```json
{
  "relationshipId": "uuid",
  "sourceEntityId": "uuid",
  "targetEntityId": "uuid",
  "relationshipType": "string",
  "relationshipStrength": "primary | secondary | tertiary",
  "effectiveDate": "ISO8601 timestamp",
  "expirationDate": "ISO8601 timestamp",
  "attributes": {},
  "metadata": {
    "sourceSystem": "string",
    "confidence": "decimal (0-1)",
    "lastVerified": "ISO8601 timestamp"
  }
}
```

### 3.2 Hierarchy Node

```json
{
  "nodeId": "uuid",
  "entityId": "uuid",
  "hierarchyType": "string",
  "parentNodeId": "uuid",
  "level": "integer",
  "path": "string",
  "isLeaf": "boolean",
  "effectiveDate": "ISO8601 timestamp",
  "expirationDate": "ISO8601 timestamp",
  "attributes": {}
}
```

## 4. Change Tracking Structures

### 4.1 Change Event

```json
{
  "eventId": "uuid",
  "eventType": "create | update | delete | merge | split | unlink",
  "entityId": "uuid",
  "entityType": "string",
  "timestamp": "ISO8601 timestamp",
  "userId": "string",
  "sourceSystem": "string",
  "changes": [
    {
      "attributeName": "string",
      "oldValue": "any",
      "newValue": "any",
      "changeReason": "string"
    }
  ],
  "metadata": {
    "ipAddress": "string",
    "userAgent": "string",
    "sessionId": "string"
  }
}
```

## 5. Data Quality Structures

### 5.1 Quality Rule

```json
{
  "ruleId": "uuid",
  "ruleName": "string",
  "ruleType": "validation | standardization | enrichment | deduplication",
  "entityType": "string",
  "attributeName": "string",
  "condition": "string",
  "severity": "error | warning | info",
  "isActive": "boolean",
  "createdAt": "ISO8601 timestamp",
  "updatedAt": "ISO8601 timestamp"
}
```

### 5.2 Quality Issue

```json
{
  "issueId": "uuid",
  "entityId": "uuid",
  "ruleId": "uuid",
  "severity": "critical | high | medium | low",
  "description": "string",
  "detectedAt": "ISO8601 timestamp",
  "status": "open | in_progress | resolved | suppressed",
  "assignedTo": "string",
  "resolution": "string",
  "resolvedAt": "ISO8601 timestamp"
}
```

## 6. Reference Data Structures

### 6.1 Reference Data Item

```json
{
  "code": "string",
  "domain": "string",
  "value": "string",
  "description": "string",
  "parentCode": "string",
  "sortOrder": "integer",
  "isActive": "boolean",
  "effectiveDate": "ISO8601 timestamp",
  "expirationDate": "ISO8601 timestamp",
  "attributes": {}
}
```

## 7. Data Exchange Formats

### 7.1 Supported Formats

- **JSON**: Primary format for API exchanges
- **XML**: Legacy system integration
- **CSV**: Batch data import/export
- **Parquet**: Big data analytics integration
- **Avro**: Streaming data integration

### 7.2 Batch Exchange Structure

```json
{
  "batchId": "uuid",
  "batchType": "full | incremental | delta",
  "entityType": "string",
  "sourceSystem": "string",
  "extractedAt": "ISO8601 timestamp",
  "recordCount": "integer",
  "checksum": "string",
  "compressionType": "none | gzip | zip",
  "entities": []
}
```

## 8. Validation Rules

All data must conform to:

- **Date/Time**: ISO 8601 format (YYYY-MM-DDTHH:mm:ssZ)
- **Country**: ISO 3166-1 alpha-2 codes
- **Currency**: ISO 4217 codes
- **Language**: ISO 639-1 codes
- **UUIDs**: RFC 4122 version 4
- **Phone Numbers**: E.164 format (recommended)
- **Email**: RFC 5322 format

## 9. Versioning and Compatibility

- **Schema Version**: Included in all messages via `schemaVersion` field
- **Backward Compatibility**: New fields are optional; existing fields maintain meaning
- **Deprecation Policy**: 6-month notice before removing fields

## 10. Summary

This data format specification provides a comprehensive foundation for master data representation, ensuring consistency, interoperability, and extensibility across MDM implementations.
