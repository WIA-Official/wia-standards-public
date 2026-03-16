# WIA-CORE-008: Universal Metadata - PHASE 2 Specification

**Version:** 1.0.0
**Status:** Stable
**Last Updated:** December 2025
**Phase:** Recommended Fields & Quality Metrics

## Overview

PHASE 2 extends the foundation established in PHASE 1 by introducing recommended fields that enhance discoverability, quality metrics for assessing metadata completeness and accuracy, and mechanisms for multilingual support. While PHASE 1 fields are mandatory, PHASE 2 fields are strongly recommended for production systems.

## 2.1 Recommended Fields

### description (RECOMMENDED)
Provides detailed information about the resource.

- **Type:** String
- **Constraints:**
  - SHOULD be between 100 and 2000 characters
  - SHOULD be complete sentences
  - MUST NOT be only a repeat of the title
- **Purpose:** Enable users to understand resource content without accessing it
- **Search:** Indexed for full-text search

**Example:**
```json
"description": "This comprehensive course introduces fundamental concepts of computer science including algorithms, data structures, programming paradigms, and computational thinking. Students will learn through hands-on projects and real-world problem solving."
```

### keywords (RECOMMENDED)
Array of searchable terms and phrases.

- **Type:** Array of strings
- **Constraints:**
  - SHOULD contain 3-20 keywords
  - Each keyword SHOULD be 2-50 characters
  - SHOULD use controlled vocabularies when available
  - MAY include multi-word phrases
- **Purpose:** Enhance discovery and faceted search
- **Search:** Primary facet for subject-based filtering

**Example:**
```json
"keywords": [
  "computer science",
  "algorithms",
  "data structures",
  "programming",
  "education"
]
```

### creator (RECOMMENDED)
Individual or organization that created the resource.

- **Type:** String or Object
- **String Format:** Name of creator
- **Object Format:** Structured creator information

**Simple Example:**
```json
"creator": "Dr. Jane Smith"
```

**Structured Example:**
```json
"creator": {
  "type": "Person",
  "name": "Dr. Jane Smith",
  "email": "jane.smith@university.edu",
  "orcid": "https://orcid.org/0000-0001-2345-6789",
  "affiliation": "University of Example"
}
```

### contributors (RECOMMENDED)
Array of additional contributors.

- **Type:** Array of strings or objects
- **Format:** Same as creator
- **Purpose:** Credit all contributors appropriately

**Example:**
```json
"contributors": [
  {
    "type": "Person",
    "name": "John Doe",
    "role": "Editor"
  },
  {
    "type": "Person",
    "name": "Alice Johnson",
    "role": "Reviewer"
  }
]
```

### contentType (RECOMMENDED)
General category or MIME type of the resource.

- **Type:** String
- **Values:** MIME type or general category
- **Standard Categories:**
  - document
  - image
  - video
  - audio
  - dataset
  - software
  - website
  - email
  - presentation
  - spreadsheet
  - 3d-model
  - virtual-reality
  - augmented-reality

**Example:**
```json
"contentType": "application/pdf"
```
OR
```json
"contentType": "document"
```

### format (RECOMMENDED)
Specific file format or physical format.

- **Type:** String
- **Examples:** PDF, MP4, JPEG, CSV, JSON, ZIP
- **Purpose:** Technical format information for processing

**Example:**
```json
"format": "PDF/A-1b"
```

### license (RECOMMENDED)
Usage rights and licensing information.

- **Type:** String or Object
- **String Format:** License name or URI
- **Object Format:** Structured license information

**Simple Example:**
```json
"license": "CC BY 4.0"
```

**Structured Example:**
```json
"license": {
  "type": "CreativeCommons",
  "name": "Attribution 4.0 International",
  "uri": "https://creativecommons.org/licenses/by/4.0/",
  "permissions": ["distribute", "modify", "commercial-use"],
  "requirements": ["attribution", "notice"]
}
```

### version (RECOMMENDED)
Version identifier for the resource.

- **Type:** String
- **Format:** Semantic versioning recommended (MAJOR.MINOR.PATCH)
- **Purpose:** Track resource evolution

**Example:**
```json
"version": "2.1.0"
```

### relatedResources (RECOMMENDED)
Array of related resource identifiers.

- **Type:** Array of strings or objects
- **Purpose:** Establish relationships between resources

**Simple Example:**
```json
"relatedResources": [
  "https://doi.org/10.1234/related.001",
  "https://example.org/prerequisite"
]
```

**Structured Example:**
```json
"relatedResources": [
  {
    "id": "https://doi.org/10.1234/related.001",
    "relationship": "isPartOf"
  },
  {
    "id": "https://example.org/prerequisite",
    "relationship": "requires"
  }
]
```

### audience (RECOMMENDED)
Intended audience or user group.

- **Type:** String or array of strings
- **Values:** Free text or controlled vocabulary
- **Purpose:** Help users determine resource relevance

**Example:**
```json
"audience": ["undergraduate students", "computer science majors"]
```

## 2.2 Quality Metrics

Quality metrics provide objective assessment of metadata completeness and accuracy.

### qualityMetrics Object Structure

```json
"qualityMetrics": {
  "completeness": 0.95,
  "accuracy": 0.98,
  "consistency": 0.96,
  "timeliness": 1.0,
  "overall": 0.97,
  "assessedAt": "2025-01-20T10:00:00Z",
  "assessedBy": "automated-quality-analyzer-v2.1"
}
```

### Quality Dimensions

#### completeness (0.0 - 1.0)
Measures how thoroughly metadata describes the resource.

**Calculation:**
```
completeness = (requiredFieldsPresent / totalRequiredFields) * 0.5 +
               (recommendedFieldsPresent / totalRecommendedFields) * 0.3 +
               (extensionFieldsPresent / expectedExtensionFields) * 0.2
```

**Scoring Guidelines:**
- 0.9-1.0: Excellent - All required and recommended fields present, rich descriptions
- 0.7-0.89: Good - All required fields, most recommended fields
- 0.5-0.69: Adequate - All required fields, some recommended fields
- Below 0.5: Poor - Missing required fields or minimal content

#### accuracy (0.0 - 1.0)
Reflects how correctly metadata represents the actual resource.

**Assessment Factors:**
1. Controlled vocabulary compliance
2. Data type correctness
3. Format validity
4. Cross-reference consistency
5. Expert review scores (if available)

**Scoring Guidelines:**
- 0.9-1.0: Verified accurate, expert reviewed
- 0.7-0.89: Likely accurate, automated validation passed
- 0.5-0.69: Uncertain accuracy, some validation failures
- Below 0.5: Known inaccuracies or major validation failures

#### consistency (0.0 - 1.0)
Measures alignment with established patterns and conventions.

**Assessment Factors:**
1. Terminology standardization
2. Format consistency
3. Alignment with similar resources
4. Relationship coherence
5. Guideline compliance

**Scoring Guidelines:**
- 0.9-1.0: Highly consistent, follows all guidelines
- 0.7-0.89: Mostly consistent, minor variations
- 0.5-0.69: Inconsistent in some areas
- Below 0.5: Major inconsistencies

#### timeliness (0.0 - 1.0)
Indicates how current metadata is relative to the resource.

**Calculation:**
```
ageInDays = (currentDate - modifiedDate) / (1 day)
timeliness = max(0, 1 - (ageInDays / thresholdDays))
```

**Threshold Guidelines by Domain:**
- News/media: 7 days
- Technology: 90 days
- Research: 180 days
- Historical: 365+ days

**Scoring Guidelines:**
- 1.0: Recently created/updated
- 0.7-0.99: Moderately aged but acceptable
- 0.5-0.69: Aging, may need review
- Below 0.5: Outdated, needs update

#### overall
Simple average of the four dimensions:
```
overall = (completeness + accuracy + consistency + timeliness) / 4
```

## 2.3 Multilingual Support

### Translation Fields

For multilingual resources, provide translations using `_translations` suffix:

```json
{
  "title": "Introduction to Quantum Computing",
  "title_translations": {
    "ko": "양자 컴퓨팅 입문",
    "ja": "量子コンピューティング入門",
    "zh": "量子计算导论",
    "es": "Introducción a la Computación Cuántica",
    "fr": "Introduction à l'Informatique Quantique",
    "de": "Einführung in Quantencomputing"
  },
  "description": "A comprehensive guide to quantum computing principles...",
  "description_translations": {
    "ko": "양자 컴퓨팅 원리에 대한 포괄적인 가이드...",
    "ja": "量子コンピューティングの原理に関する包括的なガイド..."
  }
}
```

### Language-Specific Fields

Some fields may have language-specific values:

```json
{
  "language": "en",
  "alternativeLanguages": ["ko", "ja", "zh"],
  "languageNote": "Primary content in English, supplementary materials in Korean and Japanese"
}
```

## 2.4 Complete Example with PHASE 2 Fields

```json
{
  "standard": "WIA-CORE-008",
  "version": "1.0",

  "id": "https://university.edu/courses/quantum-101",
  "title": "Introduction to Quantum Computing",
  "language": "en",
  "domain": "education",
  "created": "2024-12-01T09:00:00Z",
  "modified": "2025-01-15T14:30:00Z",

  "description": "A comprehensive introductory course covering fundamental principles of quantum computing including qubits, quantum gates, quantum algorithms, and real-world applications. Suitable for advanced undergraduate and graduate students with background in linear algebra and computer science.",

  "keywords": [
    "quantum computing",
    "quantum mechanics",
    "quantum algorithms",
    "qubits",
    "quantum gates",
    "education",
    "computer science"
  ],

  "creator": {
    "type": "Person",
    "name": "Dr. Sarah Chen",
    "email": "sarah.chen@university.edu",
    "orcid": "https://orcid.org/0000-0002-3456-7890",
    "affiliation": "Department of Computer Science, Example University"
  },

  "contributors": [
    {
      "type": "Person",
      "name": "Prof. Michael Zhang",
      "role": "Course Designer"
    },
    {
      "type": "Person",
      "name": "Dr. Emily Rodriguez",
      "role": "Technical Reviewer"
    }
  ],

  "contentType": "course",
  "format": "online-interactive",

  "license": {
    "type": "CreativeCommons",
    "name": "Attribution-NonCommercial 4.0 International",
    "uri": "https://creativecommons.org/licenses/by-nc/4.0/"
  },

  "version": "3.0.0",

  "relatedResources": [
    {
      "id": "https://university.edu/courses/linear-algebra",
      "relationship": "requires"
    },
    {
      "id": "https://university.edu/courses/quantum-201",
      "relationship": "isPrerequisiteFor"
    }
  ],

  "audience": ["undergraduate students", "graduate students", "quantum computing beginners"],

  "qualityMetrics": {
    "completeness": 0.95,
    "accuracy": 0.98,
    "consistency": 0.97,
    "timeliness": 1.0,
    "overall": 0.975,
    "assessedAt": "2025-01-15T14:30:00Z",
    "assessedBy": "wia-quality-analyzer-v2.1"
  },

  "title_translations": {
    "ko": "양자 컴퓨팅 입문",
    "zh": "量子计算导论",
    "ja": "量子コンピューティング入門"
  }
}
```

## 2.5 Implementation Guidelines

### Field Population Priority
1. **Always include:** All PHASE 1 required fields
2. **High priority:** description, keywords, creator, contentType
3. **Medium priority:** license, version, audience
4. **When applicable:** contributors, relatedResources, format

### Quality Metric Calculation
1. Calculate immediately after metadata creation/modification
2. Store alongside metadata for quick access
3. Recalculate periodically (daily/weekly based on domain)
4. Update assessedAt timestamp after each calculation

### Multilingual Best Practices
1. Always specify primary language in `language` field
2. Provide translations for title at minimum
3. Include description translations when possible
4. Use consistent translation quality across languages
5. Indicate machine vs. human translation if relevant

## Next Phase

PHASE 3 introduces extension mechanisms, domain-specific fields, and advanced features. See [PHASE3.md](./PHASE3.md) for details.

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
