# WIA-CORE-008: Universal Metadata Specification v1.0

> **Standard ID:** WIA-CORE-008
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Category:** CORE (Universal Integration Standards)
> **Authors:** WIA Core Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Core Metadata Schema](#2-core-metadata-schema)
3. [Domain-Specific Extensions](#3-domain-specific-extensions)
4. [Quality Metrics](#4-quality-metrics)
5. [Validation Rules](#5-validation-rules)
6. [Enrichment Algorithms](#6-enrichment-algorithms)
7. [Search and Discovery](#7-search-and-discovery)
8. [Interoperability](#8-interoperability)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for universal metadata that can describe, enrich, and contextualize any type of data. WIA-CORE-008 enables consistent metadata management across all domains, systems, and data types.

### 1.2 Scope

The standard covers:
- Universal metadata schema
- Domain-specific extensions
- Quality assessment and validation
- Metadata enrichment
- Search and discovery mechanisms
- Cross-system interoperability

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to make all data universally understandable, discoverable, and interoperable, benefiting humanity through enhanced knowledge sharing and data accessibility.

### 1.4 Terminology

- **Metadata**: Data about data; descriptive information about a resource
- **Universal Schema**: Core fields applicable to all data types
- **Domain Extension**: Additional fields specific to a domain
- **Provenance**: History and origin of data
- **Discoverability**: Ease of finding data through search
- **Quality Score**: Numerical measure of metadata completeness and accuracy

---

## 2. Core Metadata Schema

### 2.1 Required Fields

All universal metadata MUST include:

| Field | Type | Description |
|-------|------|-------------|
| `id` | string | Unique identifier (UUID or custom) |
| `title` | string | Primary title (max 500 chars) |

### 2.2 Recommended Fields

Metadata SHOULD include:

| Field | Type | Description |
|-------|------|-------------|
| `description` | string | Brief description (max 5000 chars) |
| `creator` | Agent | Primary creator/author |
| `created` | ISO8601 | Creation timestamp |
| `domain` | MetadataDomain | Metadata domain classification |
| `dataType` | DataType | Type of data being described |

### 2.3 Identification Fields

```json
{
  "id": "wia-metadata-1234567890-abc123",
  "title": "Research Dataset: Gene Expression Analysis",
  "alternateTitle": ["Gene Expression Data", "RNA-seq Results"],
  "identifiers": {
    "doi": "10.1234/example.doi",
    "arxiv": "2401.12345",
    "orcid": "0000-0002-1825-0097"
  }
}
```

### 2.4 Descriptive Fields

```json
{
  "description": "Comprehensive RNA-seq analysis of cancer cell lines",
  "abstract": "This dataset contains RNA sequencing data from 100 cancer cell lines...",
  "keywords": ["genomics", "RNA-seq", "cancer", "transcriptomics"],
  "tags": ["biomedical", "dataset", "public"],
  "subjects": ["Molecular Biology", "Cancer Research"]
}
```

### 2.5 Provenance Fields

```json
{
  "creator": {
    "name": "Dr. Jane Smith",
    "id": "orcid:0000-0002-1825-0097",
    "type": "person",
    "email": "jane.smith@example.org",
    "affiliation": "University Research Lab"
  },
  "contributors": [
    {
      "name": "John Doe",
      "type": "person",
      "affiliation": "Research Institute"
    }
  ],
  "publisher": {
    "name": "Open Science Repository",
    "type": "organization"
  },
  "source": "https://example.org/original-data"
}
```

### 2.6 Temporal Fields

```json
{
  "created": "2025-01-15T10:30:00Z",
  "modified": "2025-01-20T14:45:00Z",
  "published": "2025-02-01T00:00:00Z",
  "temporal": {
    "start": "2024-01-01",
    "end": "2024-12-31",
    "duration": "P1Y",
    "timezone": "UTC"
  }
}
```

### 2.7 Spatial Fields

```json
{
  "spatial": {
    "location": "Jezero Crater, Mars",
    "coordinates": {
      "lat": 18.85,
      "lon": 77.52
    },
    "boundingBox": {
      "north": 19.0,
      "south": 18.7,
      "east": 77.7,
      "west": 77.3
    },
    "elevation": -2500,
    "crs": "EPSG:4326"
  }
}
```

### 2.8 Rights and Access Fields

```json
{
  "license": {
    "name": "Creative Commons Attribution 4.0",
    "url": "https://creativecommons.org/licenses/by/4.0/",
    "spdx": "CC-BY-4.0"
  },
  "copyright": "© 2025 Research Institute",
  "accessRights": "Open access with attribution",
  "privacy": "public"
}
```

Privacy levels:
- `public`: Freely accessible
- `internal`: Organization-only
- `confidential`: Restricted access
- `restricted`: Highly restricted
- `secret`: Classified
- `top-secret`: Highest classification

### 2.9 Technical Fields

```json
{
  "dataType": "dataset",
  "format": "text/csv",
  "size": 1073741824,
  "version": {
    "number": "2.1.0",
    "date": "2025-01-20",
    "changes": "Added validation and quality improvements"
  },
  "checksum": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
  "language": "en"
}
```

---

## 3. Domain-Specific Extensions

### 3.1 Healthcare Domain

```json
{
  "domain": "healthcare",
  "domainMetadata": {
    "patientId": "PT-12345",
    "mrn": "MRN-67890",
    "diagnosisCodes": ["C50.9", "Z85.3"],
    "procedureCodes": ["99213", "80053"],
    "facility": "General Hospital",
    "physician": {
      "name": "Dr. Medical",
      "id": "NPI-1234567890"
    },
    "phi": true,
    "hipaaCompliant": true
  }
}
```

### 3.2 Finance Domain

```json
{
  "domain": "finance",
  "domainMetadata": {
    "transactionId": "TXN-20250127-001",
    "accountNumber": "****1234",
    "amount": 1500.00,
    "currency": "USD",
    "transactionType": "payment",
    "institution": "Example Bank",
    "compliance": ["SOX", "PCI-DSS"]
  }
}
```

### 3.3 Science Domain

```json
{
  "domain": "science",
  "domainMetadata": {
    "experimentId": "EXP-2025-001",
    "method": "RNA sequencing",
    "hypothesis": "Gene expression differs between cell lines",
    "variables": ["cell_type", "treatment", "time_point"],
    "sampleSize": 100,
    "instruments": ["Illumina NovaSeq 6000"],
    "funding": "NIH Grant R01-123456",
    "publicationDoi": "10.1234/journal.2025.001"
  }
}
```

### 3.4 IoT Domain

```json
{
  "domain": "iot",
  "domainMetadata": {
    "deviceId": "DEV-SENSOR-001",
    "deviceType": "temperature-sensor",
    "sensorType": "thermistor",
    "unit": "celsius",
    "samplingRate": 60,
    "calibrationDate": "2025-01-01",
    "deviceLocation": {
      "location": "Building A, Room 101",
      "coordinates": { "lat": 37.7749, "lon": -122.4194 }
    },
    "networkId": "NET-001"
  }
}
```

### 3.5 Media Domain

```json
{
  "domain": "media",
  "domainMetadata": {
    "mediaType": "image",
    "mimeType": "image/jpeg",
    "dimensions": { "width": 4096, "height": 3072 },
    "codec": "JPEG",
    "colorSpace": "sRGB",
    "captureDevice": "Canon EOS R5",
    "photographer": {
      "name": "Jane Photographer",
      "id": "photographer-001"
    }
  }
}
```

---

## 4. Quality Metrics

### 4.1 Quality Score Calculation

```
Q = (C × 0.4) + (A × 0.3) + (Co × 0.2) + (T × 0.1)
```

Where:
- `Q` = Overall quality score (0-100)
- `C` = Completeness (0-100)
- `A` = Accuracy (0-100)
- `Co` = Consistency (0-100)
- `T` = Timeliness (0-100)

### 4.2 Completeness Score

```
C = (Present Fields / Total Expected Fields) × 100
```

Total expected fields:
- Required: 2 (id, title)
- Recommended: 5 (description, creator, created, domain, dataType)
- Optional: Varies by domain

Example:
```
Required fields: 2/2 = 100%
Recommended fields: 4/5 = 80%
Completeness = (2 + 4) / (2 + 5) × 100 = 85.7%
```

### 4.3 Accuracy Score

Validation checks:
- Date formats are valid ISO8601
- Size values are non-negative
- Email addresses are valid
- URLs are well-formed
- Checksums match content

Each validation failure reduces accuracy by 10 points.

### 4.4 Consistency Score

Cross-field validation:
- `created` ≤ `modified`
- `temporal.start` ≤ `temporal.end`
- `size` matches actual data size
- `checksum` matches calculated hash

### 4.5 Timeliness Score

```
T = 100 - (days_since_update × 0.274)
```

- Fresh (0-30 days): 100-92 points
- Recent (31-180 days): 91-51 points
- Aging (181-365 days): 50-0 points
- Stale (>365 days): 0 points

### 4.6 Discoverability Score

```
D = log₂(1 + K) × 8 + log₂(1 + T) × 6 + (Desc ? 20 : 0) + (Abs ? 15 : 0) + log₂(1 + S) × 5
```

Where:
- `K` = Number of keywords
- `T` = Number of tags
- `Desc` = Has description (boolean)
- `Abs` = Has abstract (boolean)
- `S` = Number of subjects

Target: D ≥ 25 for good discoverability

---

## 5. Validation Rules

### 5.1 Field Validation

#### 5.1.1 ID Validation
```
RULE: id MUST be unique
FORMAT: Any string, recommended UUID or prefixed identifier
EXAMPLE: "wia-metadata-1234567890-abc123"
```

#### 5.1.2 Title Validation
```
RULE: title MUST NOT be empty
LENGTH: 1-500 characters
ENCODING: UTF-8
```

#### 5.1.3 Date Validation
```
FORMAT: ISO8601 (YYYY-MM-DDTHH:mm:ssZ)
EXAMPLES:
  - "2025-01-27"
  - "2025-01-27T14:30:00Z"
  - "2025-01-27T14:30:00+00:00"
```

#### 5.1.4 Size Validation
```
TYPE: Non-negative integer
UNIT: Bytes
RANGE: 0 to 2^63-1
```

### 5.2 Structural Validation

Schema validation using JSON Schema:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["id", "title"],
  "properties": {
    "id": { "type": "string" },
    "title": { "type": "string", "maxLength": 500 },
    "description": { "type": "string", "maxLength": 5000 },
    "created": { "type": "string", "format": "date-time" },
    "size": { "type": "integer", "minimum": 0 }
  }
}
```

### 5.3 Semantic Validation

- Keywords should be relevant to content
- Tags should align with domain
- Creator information should be verifiable
- Temporal ranges should be logical
- Spatial coordinates should be valid

---

## 6. Enrichment Algorithms

### 6.1 Auto-Classification

Domain classification based on keyword matching:

```
ALGORITHM DomainClassification(metadata):
  text = concat(title, description, keywords)
  for each domain in DOMAINS:
    score = 0
    for each keyword in domain.keywords:
      if keyword in text:
        score += 1
    if score > threshold:
      return domain
  return "general"
```

Domain keyword sets:
- **Healthcare**: patient, medical, clinical, diagnosis, treatment
- **Finance**: transaction, payment, financial, currency, trading
- **Science**: research, experiment, hypothesis, analysis, study
- **IoT**: sensor, device, telemetry, monitoring, measurement

### 6.2 Keyword Extraction

```
ALGORITHM ExtractKeywords(text):
  words = tokenize(lowercase(text))
  words = filter_stopwords(words)
  words = filter_by_length(words, min=4)
  words = unique(words)
  keywords = top_n(words, n=10)
  return keywords
```

Stopwords: the, a, an, and, or, but, in, on, at, to, for, of, with, by

### 6.3 Tag Generation

```
ALGORITHM GenerateTags(metadata):
  tags = []
  if metadata.domain:
    tags.add(metadata.domain)
  if metadata.dataType:
    tags.add(metadata.dataType)
  if metadata.format:
    tags.add(metadata.format)
  if metadata.privacy:
    tags.add(metadata.privacy)
  return unique(tags)
```

### 6.4 Quality Enhancement

```
ALGORITHM EnhanceQuality(metadata):
  if not metadata.description and metadata.title:
    metadata.description = "Data resource: " + metadata.title

  if not metadata.keywords:
    metadata.keywords = ExtractKeywords(metadata.title + " " + metadata.description)

  if not metadata.tags:
    metadata.tags = GenerateTags(metadata)

  if not metadata.domain:
    metadata.domain = DomainClassification(metadata)

  metadata.qualityScore = CalculateQuality(metadata)
  metadata.modified = CurrentTimestamp()

  return metadata
```

---

## 7. Search and Discovery

### 7.1 Search Algorithm

```
ALGORITHM SearchMetadata(query, metadataList):
  results = []

  # Text search
  if query.text:
    for metadata in metadataList:
      score = 0
      if query.text in metadata.title:
        score += 10
      if query.text in metadata.description:
        score += 5
      for keyword in metadata.keywords:
        if query.text in keyword:
          score += 3
      if score > 0:
        results.add((metadata, score))

  # Filter by domain
  if query.domain:
    results = filter(r => r.metadata.domain == query.domain, results)

  # Filter by date range
  if query.dateRange:
    results = filter(r => r.metadata.created in query.dateRange, results)

  # Sort by score (descending)
  results = sort(results, by=score, order=desc)

  # Pagination
  results = paginate(results, offset=query.offset, limit=query.limit)

  return results
```

### 7.2 Faceted Search

Calculate facets for filtering:

```
ALGORITHM CalculateFacets(results):
  facets = {
    domain: {},
    dataType: {},
    privacy: {},
    format: {}
  }

  for result in results:
    facets.domain[result.domain] += 1
    facets.dataType[result.dataType] += 1
    facets.privacy[result.privacy] += 1
    facets.format[result.format] += 1

  return facets
```

### 7.3 Ranking Algorithm

```
Rank = Relevance × 0.6 + Quality × 0.3 + Recency × 0.1
```

Where:
- `Relevance` = Text match score (0-100)
- `Quality` = Quality score (0-100)
- `Recency` = 100 × e^(-days_since_update / 180)

---

## 8. Interoperability

### 8.1 Standard Mappings

WIA-CORE-008 maps to:

| Standard | Mapping |
|----------|---------|
| Dublin Core | Direct field mapping |
| DCAT (Data Catalog) | Compatible structure |
| Schema.org | Semantic alignment |
| CKAN | API compatibility |
| DataCite | DOI registration |

### 8.2 Dublin Core Mapping

```json
{
  "title": "dc:title",
  "description": "dc:description",
  "creator": "dc:creator",
  "publisher": "dc:publisher",
  "created": "dc:date",
  "subjects": "dc:subject",
  "format": "dc:format",
  "identifiers": "dc:identifier",
  "license": "dc:rights",
  "language": "dc:language"
}
```

### 8.3 Export Formats

Supported export formats:
- JSON (native)
- JSON-LD (with @context)
- XML (Dublin Core)
- RDF/Turtle
- CSV (flattened)

---

## 9. Implementation Guidelines

### 9.1 Creating Metadata

```typescript
import { createMetadata } from '@wia/core-008';

const metadata = createMetadata({
  title: 'Research Dataset',
  description: 'Comprehensive analysis of...',
  creator: { name: 'Dr. Smith', id: 'orcid:0000-0002-1825-0097' },
  domain: 'science',
  dataType: 'dataset',
  keywords: ['research', 'analysis', 'data']
});
```

### 9.2 Validating Metadata

```typescript
import { validateMetadata } from '@wia/core-008';

const result = validateMetadata(metadata);

if (result.valid) {
  console.log(`Quality: ${result.qualityScore}/100`);
} else {
  console.log('Errors:', result.errors);
}
```

### 9.3 Enriching Metadata

```typescript
import { enrichMetadata } from '@wia/core-008';

const enriched = enrichMetadata(metadata, {
  autoClassify: true,
  extractKeywords: true,
  generateTags: true,
  calculateQuality: true
});

console.log(`Quality improved by ${enriched.qualityImprovement} points`);
```

### 9.4 Searching Metadata

```typescript
import { searchMetadata } from '@wia/core-008';

const results = searchMetadata(
  {
    query: 'genomics cancer',
    domain: 'science',
    dateRange: { start: '2024-01-01', end: '2025-01-01' },
    limit: 10
  },
  metadataList
);

console.log(`Found ${results.total} results in ${results.queryTime}ms`);
```

### 9.5 Best Practices

1. **Always include required fields**: id, title
2. **Add description and keywords**: Improves discoverability
3. **Use standard identifiers**: DOI, ORCID, ISBN, etc.
4. **Update modification timestamp**: Track changes
5. **Validate before storing**: Ensure quality
6. **Enrich automatically**: Enhance with ML
7. **Version metadata**: Track evolution
8. **Link related data**: Build knowledge graphs

---

## 10. References

### 10.1 Standards

- Dublin Core Metadata Initiative (DCMI)
- ISO 15836-1:2017 (Dublin Core)
- DCAT Version 2 (Data Catalog Vocabulary)
- Schema.org Metadata
- DataCite Metadata Schema

### 10.2 Specifications

- RFC 3339: Date and Time on the Internet
- RFC 4122: UUID Specification
- JSON Schema Draft 7
- RDF 1.1 Concepts

### 10.3 WIA Integration

- WIA-INTENT: Natural language metadata queries
- WIA-OMNI-API: Universal data access
- WIA-DATA: Data management standards
- WIA-SEARCH: Search and discovery

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
