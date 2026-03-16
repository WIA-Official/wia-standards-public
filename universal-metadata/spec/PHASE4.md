# WIA-CORE-008: Universal Metadata - PHASE 4 Specification

**Version:** 1.0.0
**Status:** Stable
**Last Updated:** December 2025
**Phase:** Integration & Interoperability

## Overview

PHASE 4 focuses on integration with existing metadata standards, migration from legacy systems, API specifications, and advanced interoperability patterns. This phase ensures Universal Metadata works seamlessly with established metadata ecosystems.

## 4.1 Standard Mappings

### Dublin Core Mapping

Complete bidirectional mapping to all 15 Dublin Core elements:

```javascript
const dublinCoreMapping = {
  // Dublin Core -> Universal Metadata
  'dc:title': 'title',
  'dc:creator': 'creator',
  'dc:subject': 'keywords',
  'dc:description': 'description',
  'dc:publisher': 'publisher',
  'dc:contributor': 'contributors',
  'dc:date': 'created',
  'dc:type': 'contentType',
  'dc:format': 'format',
  'dc:identifier': 'id',
  'dc:source': 'provenance.wasDerivedFrom',
  'dc:language': 'language',
  'dc:relation': 'relatedResources',
  'dc:coverage': 'geospatial',
  'dc:rights': 'license'
};

// Example transformation
const dublinCoreRecord = {
  'dc:title': 'Introduction to Metadata',
  'dc:creator': 'Jane Smith',
  'dc:date': '2025-01-15',
  'dc:type': 'Text',
  'dc:identifier': 'https://example.org/doc123'
};

const universalMetadata = transformer.fromDublinCore(dublinCoreRecord);
// Returns WIA-CORE-008 compliant metadata
```

### Schema.org Mapping

Mapping to Schema.org types and properties:

```javascript
const schemaOrgMapping = {
  'CreativeWork': {
    'name': 'title',
    'description': 'description',
    'creator': 'creator',
    'dateCreated': 'created',
    'dateModified': 'modified',
    'inLanguage': 'language',
    'keywords': 'keywords',
    'license': 'license',
    'version': 'version'
  },
  'ScholarlyArticle': {
    'author': 'creator',
    'abstract': 'description',
    'datePublished': 'created',
    'identifier': 'id'
  }
};

// Example Schema.org JSON-LD output
const schemaOrgOutput = {
  "@context": "https://schema.org",
  "@type": "ScholarlyArticle",
  "name": metadata.title,
  "author": {
    "@type": "Person",
    "name": metadata.creator.name,
    "affiliation": metadata.creator.affiliation
  },
  "datePublished": metadata.created,
  "inLanguage": metadata.language,
  "keywords": metadata.keywords.join(', '),
  "abstract": metadata.description
};
```

### DCAT (Data Catalog Vocabulary) Mapping

For data catalogs and open data:

```javascript
const dcatMapping = {
  'dcat:Dataset': {
    'dct:title': 'title',
    'dct:description': 'description',
    'dcat:keyword': 'keywords',
    'dct:issued': 'created',
    'dct:modified': 'modified',
    'dct:language': 'language',
    'dcat:distribution': 'format',
    'dct:identifier': 'id'
  }
};
```

### MODS (Metadata Object Description Schema) Mapping

For bibliographic metadata:

```javascript
const modsMapping = {
  'mods:titleInfo/mods:title': 'title',
  'mods:name/mods:namePart': 'creator',
  'mods:abstract': 'description',
  'mods:subject/mods:topic': 'keywords',
  'mods:originInfo/mods:dateIssued': 'created',
  'mods:language/mods:languageTerm': 'language',
  'mods:identifier': 'id'
};
```

## 4.2 REST API Specification

### Core Endpoints

#### Create Metadata
```http
POST /api/v1/metadata
Content-Type: application/json

{
  "title": "New Resource",
  "language": "en",
  "domain": "research",
  ...
}

Response: 201 Created
{
  "id": "https://api.example.org/metadata/12345",
  "created": "2025-01-15T10:00:00Z",
  ...
}
```

#### Retrieve Metadata
```http
GET /api/v1/metadata/{id}
Accept: application/json

Response: 200 OK
{
  "standard": "WIA-CORE-008",
  "version": "1.0",
  ...
}
```

#### Update Metadata
```http
PUT /api/v1/metadata/{id}
Content-Type: application/json

{
  "title": "Updated Title",
  "modified": "2025-01-16T11:00:00Z",
  ...
}

Response: 200 OK
```

#### Delete Metadata
```http
DELETE /api/v1/metadata/{id}

Response: 204 No Content
```

#### Search Metadata
```http
GET /api/v1/metadata?q=quantum&domain=research&minQuality=0.8&page=1&pageSize=20

Response: 200 OK
{
  "total": 150,
  "page": 1,
  "pageSize": 20,
  "results": [...],
  "facets": {...}
}
```

### Content Negotiation

Support multiple formats via Accept header:

```http
GET /api/v1/metadata/{id}
Accept: application/json          # Universal Metadata JSON
Accept: application/ld+json       # Schema.org JSON-LD
Accept: application/xml           # Universal Metadata XML
Accept: application/rdf+xml       # RDF/XML
Accept: text/turtle               # RDF Turtle
```

### Batch Operations

```http
POST /api/v1/metadata/batch
Content-Type: application/json

{
  "operations": [
    {"action": "create", "data": {...}},
    {"action": "update", "id": "123", "data": {...}},
    {"action": "delete", "id": "456"}
  ]
}

Response: 200 OK
{
  "results": [
    {"operation": 0, "success": true, "id": "789"},
    {"operation": 1, "success": true},
    {"operation": 2, "success": false, "error": "Not found"}
  ]
}
```

### Validation Endpoint

```http
POST /api/v1/metadata/validate
Content-Type: application/json

{
  "title": "Test",
  "language": "en",
  ...
}

Response: 200 OK
{
  "valid": false,
  "errors": [
    {
      "field": "title",
      "code": "LENGTH_CONSTRAINT",
      "message": "Title must be between 10 and 200 characters"
    }
  ],
  "warnings": [
    {
      "field": "description",
      "message": "Recommended field 'description' is missing"
    }
  ]
}
```

## 4.3 Migration Strategies

### Legacy System Assessment

Before migration, assess your current metadata:

```javascript
const assessmentReport = {
  totalRecords: 50000,
  currentStandard: "Custom schema",
  fieldMapping: {
    "legacy_id": "id (95% populated)",
    "legacy_title": "title (100% populated)",
    "legacy_description": "description (60% populated)",
    "legacy_created_date": "created (100% populated)"
  },
  estimatedQuality: {
    averageCompleteness: 0.65,
    estimatedMigrationQuality: 0.75
  },
  migrationComplexity: "Medium",
  estimatedDuration: "4 weeks"
};
```

### Phased Migration Approach

**Phase 1: Pilot (Week 1-2)**
- Migrate 1% of records (500 records)
- Validate quality and accuracy
- Refine mapping and transformation rules
- Document issues and solutions

**Phase 2: Incremental (Week 3-6)**
- Migrate 10% per week
- Run parallel systems
- Validate each batch
- Allow rollback if needed

**Phase 3: Full Migration (Week 7-8)**
- Complete remaining records
- Decommission legacy system
- Monitor quality metrics
- Address any remaining issues

### Migration Code Example

```javascript
import { MetadataMigrator, QualityAnalyzer } from '@wia/universal-metadata';

const migrator = new MetadataMigrator({
  source: {
    type: 'postgresql',
    connection: process.env.LEGACY_DB_URL,
    table: 'legacy_metadata'
  },
  target: {
    type: 'postgresql',
    connection: process.env.NEW_DB_URL,
    table: 'universal_metadata'
  },
  mappings: {
    'legacy_id': (value) => `https://new-system.org/resources/${value}`,
    'legacy_title': 'title',
    'legacy_description': 'description',
    'legacy_subject': (value) => ({
      keywords: value.split(';').map(k => k.trim())
    }),
    'legacy_created': (value) => ({
      created: new Date(value).toISOString()
    })
  },
  enrichment: {
    // Automatically detect language
    detectLanguage: true,
    // Calculate quality metrics
    calculateQuality: true,
    // Generate missing IDs
    generateIds: true,
    // Set modified date to migration time
    setModifiedDate: true
  },
  validation: {
    strict: false,
    logErrors: true,
    continueOnError: true
  }
});

// Execute migration
const result = await migrator.migrate({
  batchSize: 1000,
  parallelJobs: 5,
  dryRun: false
});

console.log(result);
// {
//   total: 50000,
//   migrated: 48500,
//   failed: 1500,
//   duration: '2h 15m',
//   averageQuality: 0.76,
//   errors: [...]
// }
```

## 4.4 Federation and Harvesting

### OAI-PMH Support

Universal Metadata can be harvested via OAI-PMH:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<OAI-PMH xmlns="http://www.openarchives.org/OAI/2.0/">
  <responseDate>2025-01-15T10:00:00Z</responseDate>
  <request verb="GetRecord" identifier="oai:example.org:12345"
           metadataPrefix="wia_universal">
    https://repository.example.org/oai
  </request>
  <GetRecord>
    <record>
      <header>
        <identifier>oai:example.org:12345</identifier>
        <datestamp>2025-01-15</datestamp>
      </header>
      <metadata>
        <wia:universal xmlns:wia="http://wia.org/standards/core-008">
          <wia:id>https://example.org/resources/12345</wia:id>
          <wia:title>Resource Title</wia:title>
          ...
        </wia:universal>
      </metadata>
    </record>
  </GetRecord>
</OAI-PMH>
```

### Federated Search

Enable cross-repository search:

```javascript
const federatedSearch = new FederatedSearchEngine({
  repositories: [
    {
      url: 'https://repo1.org/api/search',
      protocol: 'wia-api',
      priority: 1
    },
    {
      url: 'https://repo2.org/oai',
      protocol: 'oai-pmh',
      metadataPrefix: 'wia_universal',
      priority: 2
    },
    {
      url: 'https://repo3.org/graphql',
      protocol: 'graphql',
      priority: 1
    }
  ]
});

const results = await federatedSearch.search({
  query: 'quantum computing',
  domain: 'research',
  minQuality: 0.8,
  maxResults: 100,
  sortBy: 'relevance'
});

// Merged results from all repositories
console.log(results);
// {
//   total: 287,
//   sources: {
//     'repo1.org': 150,
//     'repo2.org': 87,
//     'repo3.org': 50
//   },
//   results: [...]
// }
```

## 4.5 GraphQL API

For flexible querying:

```graphql
type Metadata {
  id: ID!
  standard: String!
  version: String!
  title: String!
  description: String
  language: String!
  domain: String!
  keywords: [String!]
  creator: Creator
  created: DateTime!
  modified: DateTime!
  qualityMetrics: QualityMetrics
}

type QualityMetrics {
  completeness: Float!
  accuracy: Float!
  consistency: Float!
  timeliness: Float!
  overall: Float!
}

type Query {
  metadata(id: ID!): Metadata
  searchMetadata(
    query: String
    domain: String
    language: String
    minQuality: Float
    limit: Int
    offset: Int
  ): MetadataSearchResult!
}

type Mutation {
  createMetadata(input: MetadataInput!): Metadata!
  updateMetadata(id: ID!, input: MetadataInput!): Metadata!
  deleteMetadata(id: ID!): Boolean!
}
```

## 4.6 Webhook Integration

Real-time notifications for metadata events:

```javascript
// Register webhook
POST /api/v1/webhooks
{
  "url": "https://your-app.com/webhooks/metadata",
  "events": ["metadata.created", "metadata.updated", "metadata.deleted"],
  "secret": "your-webhook-secret"
}

// Webhook payload
POST https://your-app.com/webhooks/metadata
X-WIA-Signature: sha256=...
X-WIA-Event: metadata.created

{
  "event": "metadata.created",
  "timestamp": "2025-01-15T10:00:00Z",
  "data": {
    "id": "https://example.org/metadata/12345",
    "title": "New Resource",
    "domain": "research",
    ...
  }
}
```

## 4.7 Implementation Checklist

### PHASE 4 Compliance

- [ ] Support at least one standard mapping (Dublin Core, Schema.org, or DCAT)
- [ ] Implement REST API with all core endpoints
- [ ] Support content negotiation for multiple formats
- [ ] Provide migration tools or documentation
- [ ] Implement validation endpoint
- [ ] Support batch operations
- [ ] Provide OpenAPI/Swagger documentation
- [ ] Implement proper error handling and status codes
- [ ] Support pagination for search results
- [ ] Implement rate limiting and authentication

## Conclusion

PHASE 4 ensures Universal Metadata integrates seamlessly with existing systems, standards, and workflows. By following these specifications, implementations can provide maximum interoperability while maintaining the power and flexibility of Universal Metadata.

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
