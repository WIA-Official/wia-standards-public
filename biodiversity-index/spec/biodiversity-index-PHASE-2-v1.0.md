# WIA Biodiversity Index Standard - Phase 2: API Interface
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity, Preserve All Life

## Overview

Phase 2 provides computational services and APIs for biodiversity data analysis. Building on Phase 1's standardized data formats, this phase defines RESTful and GraphQL interfaces for diversity calculation, validation, integration, and analytics.

## API Architecture

**Base URL:** `https://api.biodiversity.wia.org/v1/`

**Authentication:** OAuth 2.0 + JWT
**Rate Limiting:** Tiered (Free: 1000/hr, Research: 10000/hr, Enterprise: 100000/hr)
**Content Types:** application/json, application/geo+json
**Character Encoding:** UTF-8

## RESTful Endpoints

### Occurrence Management

```
GET    /occurrences                    # List occurrences (paginated)
POST   /occurrences                    # Create new occurrence
GET    /occurrences/{id}              # Get specific occurrence
PUT    /occurrences/{id}              # Update occurrence
DELETE /occurrences/{id}              # Delete occurrence
GET    /occurrences/search            # Search with filters
```

**Query Parameters:**
- `species` - Scientific name (string)
- `country` - ISO 3166-1 alpha-2 code
- `start_date`, `end_date` - ISO 8601 dates
- `bbox` - Bounding box (minLon,minLat,maxLon,maxLat)
- `habitat_type` - Habitat classification
- `limit` (default: 100, max: 1000)
- `offset` - Pagination offset

**Example Request:**
```http
GET /v1/occurrences?species=Panthera+tigris&country=IN&year=2025&limit=50
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Example Response:**
```json
{
  "count": 127,
  "next": "/v1/occurrences?offset=50&...",
  "previous": null,
  "results": [
    {
      "occurrence_id": "OCC-2025-123456",
      "species": {"scientific_name": "Panthera tigris"},
      "location": {"latitude": 27.5142, "longitude": 88.7597},
      "observation_date": "2025-11-15T09:30:00Z"
    }
  ]
}
```

### Diversity Index Calculation

```
POST   /indices/calculate              # Calculate diversity indices
GET    /indices/{calculation_id}       # Retrieve results
GET    /indices/trends                 # Temporal trend analysis
```

**Calculate Diversity Request:**
```json
POST /v1/indices/calculate

{
  "dataset_id": "DS-AMAZON-2025",
  "spatial_filter": {
    "type": "polygon",
    "coordinates": [[[-60.5, -3.5], [-60.0, -3.5], ...]]
  },
  "temporal_filter": {
    "start_date": "2025-01-01",
    "end_date": "2025-12-31"
  },
  "indices": ["shannon_diversity", "simpson_index", "species_richness"],
  "rarefaction": {"enabled": true, "target_n": 1000},
  "bootstrap": {"enabled": true, "iterations": 1000, "confidence_level": 0.95}
}
```

**Response:**
```json
{
  "calculation_id": "CALC-2025-1234",
  "status": "completed",
  "execution_time_ms": 2847,
  "results": {
    "species_richness": {"observed": 156, "rarefied": 142.7, "ci_lower": 138.2, "ci_upper": 147.3},
    "shannon_diversity": {"value": 4.127, "ci_lower": 3.982, "ci_upper": 4.268},
    "simpson_index": {"value": 0.0234, "diversity_1_minus_d": 0.9766}
  }
}
```

### Species Information

```
GET    /species                        # List species
GET    /species/{taxon_id}            # Species details
GET    /species/{taxon_id}/occurrences # Occurrences for species
GET    /species/search                # Search species
```

### Spatial Queries

```
GET    /spatial/region/{polygon}       # Occurrences in polygon
GET    /spatial/proximity             # Species near coordinates
POST   /spatial/hotspots              # Identify biodiversity hotspots
```

### Data Validation

```
POST   /validate/occurrence            # Validate single occurrence
POST   /validate/batch                # Batch validation
```

**Validation Request:**
```json
POST /v1/validate/occurrence

{
  "occurrence_id": "OCC-2025-TEST-001",
  "species": {"scientific_name": "Panthera tigris"},
  "location": {"latitude": 27.5142, "longitude": 88.7597, "country": "India"},
  "temporal": {"observation_date": "2025-11-15T09:30:00Z"}
}
```

**Validation Response:**
```json
{
  "valid": true,
  "quality_score": 0.94,
  "checks": [
    {"check": "coordinate_validity", "status": "passed"},
    {"check": "country_match", "status": "passed"},
    {"check": "species_range", "status": "passed"},
    {"check": "taxonomy_valid", "status": "passed"}
  ],
  "warnings": [],
  "errors": []
}
```

### Integration Services

```
POST   /export/gbif                    # Export to GBIF format
POST   /export/geojson                # Export as GeoJSON
GET    /taxonomy/resolve              # Resolve taxonomic names
GET    /integration/iucn/species/{name} # IUCN status lookup
```

## GraphQL Interface

**Endpoint:** `https://api.biodiversity.wia.org/graphql`

**Example Query:**
```graphql
{
  species(scientificName: "Panthera tigris") {
    taxonId
    commonNames
    conservationStatus {
      category
      populationTrend
    }
    occurrences(limit: 10, year: 2025) {
      occurrenceId
      location {
        latitude
        longitude
        protectedArea
      }
      observationDate
      individualCount
    }
    diversityIndices(region: "India") {
      shannonIndex
      simpsonIndex
    }
  }
}
```

## WebSocket Streams

**Endpoint:** `wss://stream.biodiversity.wia.org/v1/live`

**Real-time Alerts:**
```javascript
const ws = new WebSocket('wss://stream.biodiversity.wia.org/v1/live');

ws.send(JSON.stringify({
  "action": "subscribe",
  "channels": ["endangered_detections"],
  "filters": {
    "region": "Southeast_Asia",
    "iucn_categories": ["CR", "EN"]
  }
}));

ws.onmessage = (event) => {
  const alert = JSON.parse(event.data);
  console.log('Endangered species detected:', alert);
};
```

## Authentication

**OAuth 2.0 Flow:**
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
  "scope": "read:occurrences write:occurrences calculate:indices"
}
```

## Error Handling

**Standard HTTP Status Codes:**
- 200: Success
- 201: Created
- 400: Bad Request (validation error)
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Rate Limit Exceeded
- 500: Internal Server Error

**Error Response Format:**
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid species name format",
    "details": {
      "field": "species.scientific_name",
      "value": "panthera tigris",
      "expected": "Capitalized binomial nomenclature"
    },
    "documentation_url": "https://docs.biodiversity.wia.org/errors/VALIDATION_ERROR"
  },
  "request_id": "req_abc123def456"
}
```

## Rate Limiting

**Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640995200
```

**Tiers:**
| Tier | Requests/Hour | Burst | Use Case |
|------|---------------|-------|----------|
| Free | 1,000 | 50/min | Individual researchers |
| Research | 10,000 | 200/min | Academic institutions |
| Enterprise | 100,000 | 1,000/min | Government agencies |
| Custom | Negotiable | Negotiable | Global initiatives |

## SDK Support

**Official SDKs:**
- Python: `pip install wia-biodiversity`
- JavaScript/TypeScript: `npm install @wia/biodiversity-sdk`
- R: `install.packages("wiabiodiversity")`

**Python SDK Example:**
```python
from wia_biodiversity import BiodiversityAPI

client = BiodiversityAPI(api_key='your_api_key')

# Query occurrences
occurrences = client.occurrences.list(
    species='Panthera tigris',
    country='India',
    year=2025
)

# Calculate diversity
results = client.indices.calculate(
    dataset_id='DS-INDIA-2025',
    indices=['shannon', 'simpson'],
    bootstrap_iterations=1000
)
```

## Performance Considerations

**Caching:**
- CDN caching for static resources
- Redis caching for frequent queries
- Cache-Control headers respected

**Pagination:**
- Default limit: 100
- Maximum limit: 1000
- Cursor-based pagination for large datasets

**Async Operations:**
- Long-running calculations return job ID
- Poll `/indices/{calculation_id}` for status
- Webhook notifications supported

## Compliance Requirements

### Silver Certification
- Phase 2 API integration demonstrated
- Successful diversity index calculations
- Integration with at least one external system (GBIF/IUCN/GIS)
- Documented API usage examples

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity · Preserve All Life
