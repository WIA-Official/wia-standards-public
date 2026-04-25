# Chapter 4: API Interface

## Phase 2: RESTful APIs, GraphQL, and Integration Services

### Building Connected Biodiversity Data Systems

---

## Overview

Phase 2 of the WIA Biodiversity Index Standard provides computational services and APIs for biodiversity data analysis. Building on Phase 1's standardized data formats, this chapter defines RESTful and GraphQL interfaces for diversity calculation, validation, integration, and analytics.

---

## API Architecture

### Design Principles

The WIA Biodiversity API follows modern API design principles:

1. **RESTful**: Resource-oriented URLs with standard HTTP methods
2. **GraphQL**: Flexible queries for complex data relationships
3. **Real-time**: WebSocket streams for live data feeds
4. **Versioned**: URL-based versioning (v1, v2)
5. **Documented**: OpenAPI 3.0 specification

### Base Configuration

**Base URL:** `https://api.biodiversity.wia.org/v1/`

**Production Endpoints:**
| Service | URL | Purpose |
|---------|-----|---------|
| REST API | `api.biodiversity.wia.org` | CRUD operations |
| GraphQL | `api.biodiversity.wia.org/graphql` | Complex queries |
| WebSocket | `stream.biodiversity.wia.org` | Real-time feeds |
| Auth | `auth.biodiversity.wia.org` | Authentication |

**Content Types:**
- Request: `application/json`
- Response: `application/json`, `application/geo+json`
- Character Encoding: UTF-8

---

## Authentication

### OAuth 2.0 + JWT

The WIA API uses OAuth 2.0 with JWT tokens for authentication.

**Supported Grant Types:**
- Client Credentials (machine-to-machine)
- Authorization Code (user applications)
- Refresh Token (token renewal)

### Client Credentials Flow

```http
POST https://auth.biodiversity.wia.org/oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "read:occurrences write:occurrences calculate:indices"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read:occurrences write:occurrences calculate:indices",
  "issued_at": "2025-11-20T10:00:00Z"
}
```

### Using Access Tokens

Include the access token in the Authorization header:

```http
GET /v1/occurrences
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### Available Scopes

| Scope | Description |
|-------|-------------|
| `read:occurrences` | Read occurrence records |
| `write:occurrences` | Create/update occurrences |
| `delete:occurrences` | Delete occurrences |
| `read:species` | Access species information |
| `calculate:indices` | Run diversity calculations |
| `export:data` | Export datasets |
| `admin:datasets` | Manage datasets |

---

## Rate Limiting

### Tier Structure

| Tier | Requests/Hour | Burst/Minute | Monthly Limit | Use Case |
|------|---------------|--------------|---------------|----------|
| Free | 1,000 | 50 | 25,000 | Individual researchers |
| Research | 10,000 | 200 | 250,000 | Academic institutions |
| Enterprise | 100,000 | 1,000 | Unlimited | Government agencies |
| Custom | Negotiable | Negotiable | Negotiable | Global initiatives |

### Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640995200
X-RateLimit-Tier: free
```

### Handling Rate Limits

When rate limited, the API returns HTTP 429:

```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please wait before retrying.",
    "retry_after": 3600,
    "current_usage": 1000,
    "limit": 1000
  }
}
```

---

## RESTful Endpoints

### Occurrence Management

**List Occurrences:**
```http
GET /v1/occurrences
```

**Query Parameters:**

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `species` | string | Scientific name | `Panthera tigris` |
| `country` | string | ISO 3166-1 alpha-2 | `IN` |
| `start_date` | string | ISO 8601 date | `2025-01-01` |
| `end_date` | string | ISO 8601 date | `2025-12-31` |
| `bbox` | string | Bounding box | `-180,-90,180,90` |
| `habitat_type` | string | Habitat code | `forest` |
| `iucn_status` | string | IUCN category | `CR,EN,VU` |
| `limit` | integer | Results per page (max 1000) | `100` |
| `offset` | integer | Pagination offset | `0` |
| `sort` | string | Sort field | `observation_date` |
| `order` | string | Sort order | `desc` |

**Example Request:**
```http
GET /v1/occurrences?species=Panthera+tigris&country=IN&year=2025&limit=50
Authorization: Bearer eyJhbG...
```

**Example Response:**
```json
{
  "count": 127,
  "total": 127,
  "limit": 50,
  "offset": 0,
  "next": "/v1/occurrences?offset=50&species=Panthera+tigris&country=IN&year=2025&limit=50",
  "previous": null,
  "results": [
    {
      "occurrence_id": "OCC-2025-123456",
      "species": {
        "scientific_name": "Panthera tigris",
        "common_name": "Bengal Tiger",
        "iucn_status": "EN"
      },
      "location": {
        "latitude": 27.5142,
        "longitude": 88.7597,
        "country": "India",
        "protected_area": "Sundarbans National Park"
      },
      "temporal": {
        "observation_date": "2025-11-15T09:30:00Z"
      },
      "observation": {
        "individual_count": 1,
        "basis_of_record": "human_observation"
      }
    }
  ]
}
```

**Create Occurrence:**
```http
POST /v1/occurrences
Content-Type: application/json
Authorization: Bearer eyJhbG...

{
  "species": {
    "scientific_name": "Panthera tigris"
  },
  "location": {
    "latitude": 27.5142,
    "longitude": 88.7597
  },
  "temporal": {
    "observation_date": "2025-11-15T09:30:00Z"
  }
}
```

**Response:**
```json
{
  "occurrence_id": "OCC-2025-234567",
  "status": "created",
  "created_at": "2025-11-20T10:30:00Z",
  "validation": {
    "valid": true,
    "warnings": []
  }
}
```

**Get Single Occurrence:**
```http
GET /v1/occurrences/{occurrence_id}
```

**Update Occurrence:**
```http
PUT /v1/occurrences/{occurrence_id}
```

**Delete Occurrence:**
```http
DELETE /v1/occurrences/{occurrence_id}
```

### Diversity Index Calculation

**Calculate Indices:**
```http
POST /v1/indices/calculate
Content-Type: application/json
Authorization: Bearer eyJhbG...

{
  "dataset_id": "DS-AMAZON-2025",
  "spatial_filter": {
    "type": "polygon",
    "coordinates": [[[-60.5, -3.5], [-60.0, -3.5], [-60.0, -3.0], [-60.5, -3.0], [-60.5, -3.5]]]
  },
  "temporal_filter": {
    "start_date": "2025-01-01",
    "end_date": "2025-12-31"
  },
  "taxonomic_filter": {
    "taxon_group": "birds"
  },
  "indices": ["shannon_diversity", "simpson_index", "species_richness", "chao1"],
  "rarefaction": {
    "enabled": true,
    "target_n": 1000
  },
  "bootstrap": {
    "enabled": true,
    "iterations": 1000,
    "confidence_level": 0.95
  }
}
```

**Response (Synchronous for small datasets):**
```json
{
  "calculation_id": "CALC-2025-1234",
  "status": "completed",
  "execution_time_ms": 2847,
  "input_summary": {
    "occurrence_count": 15847,
    "species_count": 156
  },
  "results": {
    "species_richness": {
      "observed": 156,
      "rarefied": 142.7,
      "ci_lower": 138.2,
      "ci_upper": 147.3
    },
    "shannon_diversity": {
      "value": 4.127,
      "ci_lower": 3.982,
      "ci_upper": 4.268
    },
    "simpson_index": {
      "dominance_d": 0.0234,
      "diversity_1_minus_d": 0.9766
    },
    "chao1": {
      "estimate": 172.4,
      "ci_lower": 161.2,
      "ci_upper": 195.7
    }
  }
}
```

**Response (Asynchronous for large datasets):**
```json
{
  "calculation_id": "CALC-2025-1234",
  "status": "processing",
  "estimated_completion": "2025-11-20T10:35:00Z",
  "progress_url": "/v1/indices/CALC-2025-1234"
}
```

**Check Calculation Status:**
```http
GET /v1/indices/{calculation_id}
```

### Species Information

**List Species:**
```http
GET /v1/species?country=IN&iucn_status=EN,CR
```

**Get Species Details:**
```http
GET /v1/species/{taxon_id}
```

**Response:**
```json
{
  "taxon_id": "GBIF:9694",
  "scientific_name": "Panthera tigris",
  "common_names": {
    "en": "Tiger",
    "hi": "बाघ",
    "bn": "বাঘ"
  },
  "taxonomy": {
    "kingdom": "Animalia",
    "phylum": "Chordata",
    "class": "Mammalia",
    "order": "Carnivora",
    "family": "Felidae"
  },
  "conservation_status": {
    "iucn_category": "EN",
    "population_trend": "increasing",
    "assessment_date": "2022-07-21"
  },
  "distribution": {
    "countries": ["IN", "BD", "NP", "BT", "MM", "MY"],
    "habitat_types": ["tropical_forest", "mangrove", "grassland"]
  },
  "occurrence_count": 15847
}
```

### Spatial Queries

**Occurrences in Polygon:**
```http
POST /v1/spatial/region
Content-Type: application/json

{
  "type": "Polygon",
  "coordinates": [[[-60.5, -3.5], [-60.0, -3.5], [-60.0, -3.0], [-60.5, -3.0], [-60.5, -3.5]]]
}
```

**Proximity Search:**
```http
GET /v1/spatial/proximity?lat=27.5&lon=88.7&radius_km=50
```

**Identify Hotspots:**
```http
POST /v1/spatial/hotspots
Content-Type: application/json

{
  "region": "IN",
  "index": "shannon_diversity",
  "cell_size_km": 10,
  "threshold_percentile": 90
}
```

### Data Validation

**Validate Single Occurrence:**
```http
POST /v1/validate/occurrence
Content-Type: application/json

{
  "occurrence_id": "TEST-001",
  "species": {
    "scientific_name": "Panthera tigris"
  },
  "location": {
    "latitude": 27.5142,
    "longitude": 88.7597,
    "country": "India"
  },
  "temporal": {
    "observation_date": "2025-11-15T09:30:00Z"
  }
}
```

**Validation Response:**
```json
{
  "valid": true,
  "quality_score": 0.94,
  "checks": [
    {
      "check": "coordinate_validity",
      "status": "passed",
      "details": "Coordinates within valid range"
    },
    {
      "check": "country_match",
      "status": "passed",
      "details": "Coordinates fall within India boundaries"
    },
    {
      "check": "species_range",
      "status": "passed",
      "details": "Location within known range for Panthera tigris"
    },
    {
      "check": "taxonomy_valid",
      "status": "passed",
      "details": "Scientific name matches GBIF Backbone"
    },
    {
      "check": "temporal_valid",
      "status": "passed",
      "details": "Date is valid and not in future"
    }
  ],
  "warnings": [],
  "errors": [],
  "suggested_improvements": [
    "Add coordinate_uncertainty_m for better spatial precision"
  ]
}
```

### Export Services

**Export to Darwin Core Archive:**
```http
POST /v1/export/dwca
Content-Type: application/json

{
  "dataset_id": "DS-AMAZON-2025",
  "filters": {
    "year": 2025
  }
}
```

**Export to GeoJSON:**
```http
POST /v1/export/geojson
Content-Type: application/json

{
  "dataset_id": "DS-AMAZON-2025",
  "include_fields": ["occurrence_id", "scientific_name", "observation_date"]
}
```

---

## GraphQL Interface

### Endpoint

**URL:** `https://api.biodiversity.wia.org/graphql`
**Method:** POST
**Content-Type:** `application/json`

### Schema Overview

```graphql
type Query {
  species(scientificName: String, taxonId: String): Species
  occurrences(filter: OccurrenceFilter, limit: Int, offset: Int): OccurrenceConnection
  diversityIndices(region: String, filter: IndicesFilter): DiversityIndices
  datasets(owner: String): [Dataset]
}

type Mutation {
  createOccurrence(input: OccurrenceInput!): Occurrence
  updateOccurrence(id: ID!, input: OccurrenceInput!): Occurrence
  deleteOccurrence(id: ID!): Boolean
  calculateIndices(input: IndicesInput!): Calculation
}

type Subscription {
  occurrenceCreated(region: String): Occurrence
  alertTriggered(filter: AlertFilter): Alert
}
```

### Example Queries

**Species with Occurrences:**
```graphql
query GetSpeciesWithOccurrences {
  species(scientificName: "Panthera tigris") {
    taxonId
    scientificName
    commonNames {
      language
      name
    }
    conservationStatus {
      iucnCategory
      populationTrend
      assessmentDate
    }
    occurrences(limit: 10, year: 2025) {
      totalCount
      edges {
        node {
          occurrenceId
          location {
            latitude
            longitude
            protectedArea {
              name
              iucnCategory
            }
          }
          observationDate
          individualCount
        }
      }
    }
    diversityIndices(region: "India") {
      shannonIndex {
        value
        confidenceInterval
      }
      simpsonIndex {
        value
      }
    }
  }
}
```

**Biodiversity Summary:**
```graphql
query BiodiversitySummary($region: String!, $year: Int!) {
  region(code: $region) {
    name
    area_km2
    protectedAreaCoverage

    speciesSummary(year: $year) {
      totalSpecies
      threatenedSpecies
      endemicSpecies
      speciesByGroup {
        group
        count
      }
    }

    diversityTrend(years: 5) {
      year
      shannonIndex
      speciesRichness
    }

    topHotspots(limit: 5) {
      name
      coordinates
      shannonIndex
      keySpecies
    }
  }
}
```

**Variables:**
```json
{
  "region": "IN-WB",
  "year": 2025
}
```

### Mutations

**Create Occurrence:**
```graphql
mutation CreateOccurrence($input: OccurrenceInput!) {
  createOccurrence(input: $input) {
    occurrenceId
    status
    validation {
      valid
      warnings
    }
  }
}
```

**Variables:**
```json
{
  "input": {
    "species": {
      "scientificName": "Panthera tigris"
    },
    "location": {
      "latitude": 27.5142,
      "longitude": 88.7597
    },
    "observationDate": "2025-11-15T09:30:00Z"
  }
}
```

---

## WebSocket Streams

### Connection

```javascript
const ws = new WebSocket('wss://stream.biodiversity.wia.org/v1/live');

ws.onopen = () => {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('Received:', message);
};
```

### Subscribe to Channels

**Endangered Species Detections:**
```javascript
ws.send(JSON.stringify({
  action: 'subscribe',
  channels: ['endangered_detections'],
  filters: {
    region: 'Southeast_Asia',
    iucn_categories: ['CR', 'EN']
  }
}));
```

**Real-time Occurrence Feed:**
```javascript
ws.send(JSON.stringify({
  action: 'subscribe',
  channels: ['occurrence_feed'],
  filters: {
    species: ['Panthera tigris', 'Elephas maximus'],
    countries: ['IN', 'NP', 'BD']
  }
}));
```

### Message Types

**Detection Alert:**
```json
{
  "type": "endangered_detection",
  "timestamp": "2025-11-20T10:30:15Z",
  "data": {
    "occurrence_id": "OCC-2025-345678",
    "species": {
      "scientific_name": "Rhinoceros sondaicus",
      "iucn_status": "CR"
    },
    "location": {
      "latitude": -6.7563,
      "longitude": 105.4231,
      "protected_area": "Ujung Kulon National Park"
    },
    "detection_method": "camera_trap"
  }
}
```

**Threshold Alert:**
```json
{
  "type": "threshold_alert",
  "timestamp": "2025-11-20T10:30:15Z",
  "data": {
    "metric": "shannon_diversity",
    "region": "Yellowstone_Zone_A",
    "current_value": 3.15,
    "threshold": 3.20,
    "direction": "below",
    "consecutive_violations": 2
  }
}
```

---

## SDK Support

### Python SDK

**Installation:**
```bash
pip install wia-biodiversity
```

**Usage:**
```python
from wia_biodiversity import BiodiversityAPI

# Initialize client
client = BiodiversityAPI(api_key='your_api_key')

# Query occurrences
occurrences = client.occurrences.list(
    species='Panthera tigris',
    country='IN',
    year=2025,
    limit=100
)

for occ in occurrences:
    print(f"{occ.occurrence_id}: {occ.location.protected_area}")

# Calculate diversity indices
results = client.indices.calculate(
    dataset_id='DS-INDIA-2025',
    indices=['shannon', 'simpson', 'chao1'],
    spatial_filter={
        'type': 'country',
        'code': 'IN'
    },
    bootstrap_iterations=1000
)

print(f"Shannon Index: {results.shannon_diversity.value}")
print(f"95% CI: [{results.shannon_diversity.ci_lower}, {results.shannon_diversity.ci_upper}]")

# Create occurrence
new_occ = client.occurrences.create(
    species={'scientific_name': 'Panthera tigris'},
    location={'latitude': 27.5, 'longitude': 88.7},
    observation_date='2025-11-20T10:00:00Z'
)
```

### TypeScript/JavaScript SDK

**Installation:**
```bash
npm install @wia/biodiversity-sdk
```

**Usage:**
```typescript
import { BiodiversityAPI } from '@wia/biodiversity-sdk';

const client = new BiodiversityAPI({ apiKey: 'your_api_key' });

// Query occurrences
const occurrences = await client.occurrences.list({
  species: 'Panthera tigris',
  country: 'IN',
  year: 2025
});

// Calculate indices
const indices = await client.indices.calculate({
  datasetId: 'DS-INDIA-2025',
  indices: ['shannon', 'simpson'],
  bootstrap: { iterations: 1000 }
});

console.log(`Shannon: ${indices.shannonDiversity.value}`);

// Real-time subscriptions
const ws = client.streams.connect();
ws.subscribe('endangered_detections', {
  region: 'Southeast_Asia',
  iucnCategories: ['CR', 'EN']
}, (alert) => {
  console.log(`Alert: ${alert.species.scientificName} detected!`);
});
```

### R Package

**Installation:**
```r
install.packages("wiabiodiversity")
```

**Usage:**
```r
library(wiabiodiversity)

# Set API key
wia_set_key("your_api_key")

# Query occurrences
occurrences <- wia_occurrences(
  species = "Panthera tigris",
  country = "IN",
  year = 2025
)

# Calculate diversity indices
indices <- wia_diversity(
  dataset_id = "DS-INDIA-2025",
  indices = c("shannon", "simpson", "chao1"),
  bootstrap = 1000
)

# Plot rarefaction curve
wia_rarefaction_plot(indices)

# Export to sf for spatial analysis
library(sf)
occ_sf <- wia_to_sf(occurrences)
plot(occ_sf["iucn_status"])
```

---

## Error Handling

### HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Success |
| 201 | Created | Resource created |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Missing/invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 422 | Unprocessable Entity | Validation failed |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance mode |

### Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid scientific name format",
    "details": {
      "field": "species.scientific_name",
      "value": "panthera tigris",
      "expected": "Capitalized genus (e.g., 'Panthera tigris')"
    },
    "documentation_url": "https://docs.biodiversity.wia.org/errors/VALIDATION_ERROR",
    "request_id": "req_abc123def456"
  }
}
```

---

## Key Takeaways

1. **RESTful API** provides CRUD operations for occurrences, species, and datasets
2. **GraphQL interface** enables flexible queries for complex data relationships
3. **WebSocket streams** deliver real-time alerts for endangered species and threshold violations
4. **Authentication** uses OAuth 2.0 with JWT tokens and scope-based permissions
5. **SDKs** available for Python, TypeScript/JavaScript, and R

## Review Questions

1. What authentication method does the WIA API use?
2. How do the rate limiting tiers differ between Free and Enterprise?
3. What are the advantages of using GraphQL over REST for biodiversity queries?
4. How can you subscribe to real-time endangered species detection alerts?
5. Which SDK would you use for spatial analysis with sf package integration?

---

**Next Chapter Preview:** Chapter 5 explores Phase 3: Field Protocols, covering standardized survey methods for birds, mammals, vegetation, and eDNA collection.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity · Preserve All Life
