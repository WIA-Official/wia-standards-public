# WIA-SOC-020: Labor Market Data - Phase 1: Data Format Specification

**Version:** 1.0  
**Status:** Active  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 1 of the WIA-SOC-020 Labor Market Data Standard defines the fundamental data structures, schemas, and validation rules for representing employment information, wage data, skills taxonomies, and workforce analytics.

### 1.1 Objectives

- Establish standardized JSON schemas for all labor market data types
- Define comprehensive field specifications with data types and constraints
- Create unified taxonomies for industries, occupations, and skills
- Ensure interoperability across global labor market information systems
- Enable automated validation and quality assurance

### 1.2 Scope

This specification covers:
- Worker profiles and employment history
- Employer and organization data
- Job postings and vacancy information
- Wage and compensation statistics
- Skills taxonomy and proficiency levels
- Labor market aggregate statistics
- Geographic and demographic classifications

---

## 2. Core Data Schemas

### 2.1 Worker Profile Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wiastandards.com/schemas/soc-020/worker-profile/v1",
  "title": "WIA-SOC-020 Worker Profile",
  "type": "object",
  "properties": {
    "workerId": {
      "type": "string",
      "pattern": "^WKR-[0-9]{4}-[0-9]{6}$",
      "description": "Unique worker identifier in format WKR-YYYY-NNNNNN"
    },
    "personalInfo": {
      "type": "object",
      "properties": {
        "name": {"type": "string", "minLength": 1},
        "dateOfBirth": {"type": "string", "format": "date"},
        "gender": {"enum": ["male", "female", "non-binary", "prefer-not-to-say"]},
        "location": {"$ref": "#/definitions/Location"}
      },
      "required": ["name"]
    },
    "skills": {
      "type": "array",
      "items": {"$ref": "#/definitions/Skill"}
    },
    "experience": {
      "type": "array",
      "items": {"$ref": "#/definitions/Employment"}
    },
    "education": {
      "type": "array",
      "items": {"$ref": "#/definitions/Education"}
    },
    "certifications": {
      "type": "array",
      "items": {"$ref": "#/definitions/Certification"}
    }
  },
  "required": ["workerId", "personalInfo"]
}
```

### 2.2 Employment Record Schema

```json
{
  "employment": {
    "employmentId": "EMP-2025-789012",
    "employer": {
      "employerId": "ORG-543210",
      "name": "Tech Innovations Inc.",
      "industry": {
        "naics": "5415",
        "isic": "6201",
        "description": "Computer Systems Design Services"
      },
      "size": "1000-4999",
      "location": {"$ref": "#/definitions/Location"}
    },
    "position": {
      "title": "Senior Software Engineer",
      "occupationCode": {
        "soc": "15-1252",
        "isco": "2512",
        "onet": "15-1252.00"
      },
      "level": "L5",
      "department": "Engineering"
    },
    "period": {
      "startDate": "2022-03-15",
      "endDate": null,
      "isCurrent": true,
      "tenure": "3 years 9 months"
    },
    "employmentType": "full-time",
    "workArrangement": "hybrid",
    "hoursPerWeek": 40,
    "compensation": {
      "$ref": "#/definitions/Compensation"
    }
  }
}
```

### 2.3 Skill Definition Schema

```json
{
  "skill": {
    "skillId": "SKL-001234",
    "name": "Python Programming",
    "aliases": ["Python", "Python Development", "Python 3.x"],
    "category": "Technical",
    "subcategory": "Programming Languages",
    "proficiencyLevel": 4,
    "proficiencyScale": {
      "min": 1,
      "max": 5,
      "labels": ["Beginner", "Intermediate", "Advanced", "Expert", "Master"]
    },
    "verification": {
      "method": "certification",
      "source": "Python Institute",
      "date": "2024-06-15",
      "expiryDate": "2027-06-15"
    },
    "relatedSkills": ["SKL-001235", "SKL-001236"],
    "demandIndex": 92,
    "trendDirection": "increasing"
  }
}
```

### 2.4 Job Posting Schema

```json
{
  "jobPosting": {
    "postingId": "JOB-2025-456789",
    "employer": "ORG-543210",
    "title": "Machine Learning Engineer",
    "description": "Build and deploy ML models at scale for production systems",
    "requirements": {
      "education": {
        "level": "bachelor",
        "field": "Computer Science or related field"
      },
      "experience": {
        "minYears": 3,
        "maxYears": 5,
        "preferred": 4
      },
      "skills": {
        "required": ["SKL-001234", "SKL-009012", "SKL-012345"],
        "preferred": ["SKL-006789", "SKL-004567"]
      },
      "certifications": {
        "required": [],
        "preferred": ["AWS Certified ML Specialty"]
      }
    },
    "compensation": {
      "salaryRange": {
        "min": 120000,
        "max": 180000,
        "currency": "USD",
        "period": "annual"
      },
      "equity": true,
      "benefits": "comprehensive",
      "bonusStructure": "performance-based"
    },
    "location": {
      "city": "San Francisco",
      "state": "CA",
      "country": "US",
      "remote": "hybrid",
      "travelRequired": "10%"
    },
    "posted": "2025-12-15",
    "validUntil": "2026-01-15",
    "status": "open"
  }
}
```

### 2.5 Wage Statistics Schema

```json
{
  "wageStatistics": {
    "occupation": "SOC-15-1252",
    "region": "US-CA-SF",
    "period": "2025-Q4",
    "statistics": {
      "mean": 158000,
      "median": 152000,
      "mode": 145000,
      "percentile10": 105000,
      "percentile25": 128000,
      "percentile75": 185000,
      "percentile90": 220000,
      "standardDeviation": 35000
    },
    "currency": "USD",
    "period": "annual",
    "sampleSize": 2847,
    "confidenceLevel": 0.95,
    "marginOfError": 2500,
    "lastUpdated": "2025-12-01",
    "purchasingPowerParity": {
      "localValue": 158000,
      "normalizedUSD": 158000,
      "colIndex": 178,
      "pppFactor": 1.0
    }
  }
}
```

---

## 3. Taxonomy Definitions

### 3.1 Industry Classification

WIA-SOC-020 harmonizes multiple industry classification systems:

| System | Version | Coverage | Mapping |
|--------|---------|----------|---------|
| NAICS | 2022 | North America | Primary |
| ISIC | Rev. 4 | International | Cross-reference |
| NACE | Rev. 2 | European Union | Cross-reference |

Cross-reference mappings provided in separate taxonomy files.

### 3.2 Occupation Codes

Unified occupation taxonomy integrating:

- **SOC (Standard Occupational Classification)** - 2018 version
- **ISCO (International Standard Classification of Occupations)** - ISCO-08
- **O*NET** - Online database with 35,000+ skills

### 3.3 Skills Taxonomy

50,000+ standardized skills organized into:

- **Technical Skills**: Programming, tools, technologies
- **Professional Skills**: Project management, analysis, design
- **Soft Skills**: Communication, leadership, teamwork
- **Industry-Specific Skills**: Domain knowledge and certifications

---

## 4. Field Definitions

### 4.1 Core Fields

| Field | Type | Format | Required | Description |
|-------|------|--------|----------|-------------|
| `workerId` | string | `WKR-YYYY-NNNNNN` | Yes | Unique worker identifier |
| `employerId` | string | `ORG-NNNNNN` | Yes | Unique employer identifier |
| `postingId` | string | `JOB-YYYY-NNNNNN` | Yes | Unique job posting identifier |
| `skillId` | string | `SKL-NNNNNN` | Yes | Unique skill identifier |
| `occupationCode` | string | SOC/ISCO format | Yes | Standardized occupation code |
| `industry` | string | NAICS/ISIC code | Yes | Industry classification code |

### 4.2 Employment Type Enumeration

```
- full-time
- part-time
- contract
- freelance
- temporary
- seasonal
- internship
- apprenticeship
```

### 4.3 Work Arrangement Enumeration

```
- on-site
- remote
- hybrid
- distributed
- field-based
```

### 4.4 Education Level Classification (ISCED)

```
0: Early childhood education
1: Primary education
2: Lower secondary education
3: Upper secondary education
4: Post-secondary non-tertiary education
5: Short-cycle tertiary education
6: Bachelor's or equivalent
7: Master's or equivalent
8: Doctoral or equivalent
```

---

## 5. Validation Rules

### 5.1 Structural Validation

- All required fields must be present
- Field types must match schema definitions
- Enumerated values must be from allowed sets
- Date fields must be valid ISO 8601 dates
- Numeric fields must be within specified ranges

### 5.2 Semantic Validation

- Start dates must precede end dates
- Current employment cannot have end date
- Salary range minimum ≤ maximum
- Skill proficiency levels validated against scale
- Geographic codes validated against authoritative databases
- Industry/occupation codes must be current (not deprecated)

### 5.3 Business Rules

- Wages must fall within reasonable ranges for occupation and region
- Employment gaps >6 months should include reason code
- Skill proficiency claims should have supporting evidence
- Job posting validity period ≤ 90 days recommended
- Minimum sample size for statistical aggregation: 30 observations

---

## 6. Data Quality Indicators

Every dataset should include quality metadata:

```json
{
  "qualityMetrics": {
    "completeness": 0.95,
    "freshness": "2025-12-26T10:30:00Z",
    "coverage": {
      "populationSize": 100000,
      "sampleSize": 5000,
      "samplingMethod": "stratified random"
    },
    "accuracy": {
      "confidenceLevel": 0.95,
      "marginOfError": 0.02
    },
    "provenance": {
      "source": "National Statistical Agency",
      "trustScore": 0.98,
      "verificationMethod": "government certified"
    }
  }
}
```

---

## 7. Extensibility

Organizations may add custom extensions while maintaining core compatibility:

```json
{
  "standard": "WIA-SOC-020",
  "version": "1.0",
  "coreData": { ... },
  "extensions": {
    "healthcare": {
      "licenseNumber": "RN-123456",
      "specializations": ["ICU", "Emergency"],
      "boardCertified": true
    }
  }
}
```

---

## 8. Implementation Guidelines

### 8.1 JSON Schema Validation

Use JSON Schema validators (e.g., AJV, jsonschema) to ensure structural compliance.

### 8.2 Reference Data

Maintain up-to-date reference datasets for:
- Industry codes with descriptions
- Occupation codes with definitions
- Skills taxonomy with relationships
- Geographic codes with hierarchies

### 8.3 Version Management

- Schema version indicated in `$schema` field
- Breaking changes trigger major version increment
- Backward compatibility maintained for 18 months

---

## 9. Compliance

Organizations implementing Phase 1 must:

- ✅ Validate all data against official JSON schemas
- ✅ Use standardized taxonomies for industries, occupations, skills
- ✅ Include quality metadata with all datasets
- ✅ Document any custom extensions
- ✅ Maintain data freshness indicators

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association | MIT License
