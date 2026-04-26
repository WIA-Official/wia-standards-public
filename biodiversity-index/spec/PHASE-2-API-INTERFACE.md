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


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.
