# WIA-SOIL-MICROBIOME Phase 2: API Interface Specification

**Version:** 1.0.0
**Date:** 2025-12-29
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

Phase 2 defines the RESTful API interface for WIA-SOIL-MICROBIOME data submission, retrieval, analysis, and integration. This specification enables standardized programmatic access to soil microbiome data across laboratories, research institutions, and agricultural technology platforms.

## 1. API Architecture

### Base URL Structure

```
Production:  https://api.wia-soil-microbiome.org/v1
Staging:     https://api-staging.wia-soil-microbiome.org/v1
Development: https://api-dev.wia-soil-microbiome.org/v1
```

### API Versioning

- Version is specified in URL path: `/v1`, `/v2`, etc.
- Current version: `v1`
- Backward compatibility maintained within major versions
- Deprecation notices provided 12 months in advance
- Legacy endpoint support: minimum 24 months

### Content Negotiation

```http
Accept: application/json                           # Default JSON response
Accept: application/vnd.wia.soil-microbiome+json  # WIA-specific JSON
Accept: application/xml                            # XML response
Accept: text/csv                                   # CSV export
Accept: application/vnd.wia.soil-microbiome.v1+json  # Version-specific
```

## 2. Authentication & Authorization

### Authentication Methods

#### API Key Authentication (Recommended for Services)

```http
GET /v1/samples HTTP/1.1
Host: api.wia-soil-microbiome.org
Authorization: ApiKey YOUR_API_KEY_HERE
Content-Type: application/json
```

#### OAuth 2.0 (Recommended for Applications)

```http
POST /v1/oauth/token HTTP/1.1
Host: api.wia-soil-microbiome.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=samples:read samples:write
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "samples:read samples:write"
}
```

#### JWT Bearer Token

```http
GET /v1/samples HTTP/1.1
Host: api.wia-soil-microbiome.org
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### API Key Management

```http
POST /v1/auth/keys HTTP/1.1
Authorization: Bearer {user_token}
Content-Type: application/json

{
  "name": "Production API Key",
  "description": "Key for automated sample submission",
  "scopes": ["samples:write", "analysis:read"],
  "expires_at": "2026-12-29T00:00:00Z",
  "ip_whitelist": ["192.168.1.0/24", "10.0.0.1"],
  "rate_limit_tier": "premium"
}
```

Response:
```json
{
  "key_id": "key_live_abc123xyz789",
  "api_key": "wia_sk_live_abc123xyz789def456ghi...",
  "created_at": "2025-12-29T10:30:00Z",
  "expires_at": "2026-12-29T00:00:00Z",
  "scopes": ["samples:write", "analysis:read"],
  "rate_limit": {
    "tier": "premium",
    "requests_per_minute": 1000,
    "requests_per_day": 100000
  }
}
```

### Authorization Scopes

| Scope | Description | Operations |
|-------|-------------|------------|
| `samples:read` | Read sample metadata and results | GET /samples, GET /samples/{id} |
| `samples:write` | Create and update samples | POST /samples, PUT /samples/{id} |
| `samples:delete` | Delete samples | DELETE /samples/{id} |
| `analysis:read` | Read analysis results | GET /analysis, GET /diversity |
| `analysis:write` | Submit analysis jobs | POST /analysis/jobs |
| `taxonomy:read` | Read taxonomic data | GET /taxonomy |
| `functional:read` | Read functional gene data | GET /functional |
| `export:read` | Export bulk data | GET /export |
| `admin:all` | Full administrative access | All operations |

### Security Headers

Required security headers for all API requests:

```http
X-API-Version: 1.0.0
X-Request-ID: uuid-v4-string
X-Client-Name: MyApp/1.0.0
X-Timestamp: 2025-12-29T10:30:00Z
```

## 3. Rate Limiting

### Rate Limit Tiers

| Tier | Requests/Minute | Requests/Hour | Requests/Day | Burst Limit |
|------|-----------------|---------------|--------------|-------------|
| Free | 60 | 1,000 | 10,000 | 10 |
| Basic | 300 | 10,000 | 100,000 | 50 |
| Professional | 1,000 | 50,000 | 500,000 | 200 |
| Enterprise | 5,000 | 250,000 | Unlimited | 1,000 |
| Research Institution | 2,000 | 100,000 | 1,000,000 | 500 |

### Rate Limit Headers

Every API response includes rate limit information:

```http
HTTP/1.1 200 OK
X-RateLimit-Tier: professional
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735467000
X-RateLimit-Reset-After: 45
Retry-After: 45
```

### Rate Limit Exceeded Response

```http
HTTP/1.1 429 Too Many Requests
Content-Type: application/json
Retry-After: 60

{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded for tier 'basic'",
    "tier": "basic",
    "limit": 300,
    "window": "per_minute",
    "retry_after": 60,
    "upgrade_url": "https://api.wia-soil-microbiome.org/pricing"
  }
}
```

### Rate Limit Best Practices

1. **Implement exponential backoff** when receiving 429 responses
2. **Cache responses** when appropriate
3. **Use webhooks** instead of polling for async operations
4. **Batch requests** when possible
5. **Monitor headers** and adjust request frequency

## 4. Core API Endpoints

### 4.1 Sample Management

#### Create Sample

```http
POST /v1/samples HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "sample": {
    "id": "WSM-US-20251229-0001",
    "type": "rhizosphere",
    "depth_cm": {"min": 0, "max": 15},
    "mass_g": 25.5
  },
  "location": {
    "coordinates": {
      "latitude": 40.7128,
      "longitude": -74.0060,
      "elevation_m": 125
    },
    "datum": "WGS84",
    "field_id": "FIELD-A-01"
  },
  "collection": {
    "timestamp": "2025-12-29T10:30:00Z",
    "collector": {
      "name": "Dr. Jane Smith",
      "organization": "Agricultural Research Institute"
    },
    "method": "manual_auger"
  }
}
```

Response (201 Created):
```json
{
  "id": "WSM-US-20251229-0001",
  "status": "created",
  "created_at": "2025-12-29T10:35:00Z",
  "urls": {
    "self": "https://api.wia-soil-microbiome.org/v1/samples/WSM-US-20251229-0001",
    "results": "https://api.wia-soil-microbiome.org/v1/samples/WSM-US-20251229-0001/results",
    "analysis": "https://api.wia-soil-microbiome.org/v1/samples/WSM-US-20251229-0001/analysis"
  },
  "qc_status": "pending"
}
```

#### Get Sample

```http
GET /v1/samples/{sample_id} HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
```

Response (200 OK):
```json
{
  "standard": "WIA-SOIL-MICROBIOME",
  "version": "1.0.0",
  "sample": { /* full sample metadata */ },
  "processing_status": {
    "dna_extraction": "completed",
    "sequencing": "in_progress",
    "analysis": "pending"
  },
  "results": {
    "taxonomy": {
      "status": "available",
      "url": "/v1/samples/WSM-US-20251229-0001/taxonomy"
    },
    "functional": {
      "status": "pending",
      "estimated_completion": "2025-12-30T10:00:00Z"
    }
  }
}
```

#### Update Sample

```http
PUT /v1/samples/{sample_id} HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "soil_properties": {
    "chemistry": {
      "ph": 6.8,
      "organic_matter_percent": 3.2
    }
  }
}
```

#### List Samples

```http
GET /v1/samples?page=1&limit=50&sort=created_at:desc HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
```

Query Parameters:
- `page`: Page number (default: 1)
- `limit`: Results per page (max: 100, default: 50)
- `sort`: Sort field and direction (e.g., `created_at:desc`)
- `filter[field_id]`: Filter by field ID
- `filter[date_from]`: Filter by collection date
- `filter[location]`: Geographic bounding box
- `filter[crop]`: Filter by crop type
- `filter[tillage]`: Filter by tillage practice

Response (200 OK):
```json
{
  "data": [
    { /* sample object */ },
    { /* sample object */ }
  ],
  "pagination": {
    "total": 1247,
    "count": 50,
    "page": 1,
    "pages": 25,
    "links": {
      "first": "/v1/samples?page=1&limit=50",
      "last": "/v1/samples?page=25&limit=50",
      "next": "/v1/samples?page=2&limit=50",
      "prev": null
    }
  }
}
```

#### Delete Sample

```http
DELETE /v1/samples/{sample_id} HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
X-Confirm-Delete: true
```

Response (204 No Content)

### 4.2 Taxonomic Data

#### Get Taxonomic Profile

```http
GET /v1/samples/{sample_id}/taxonomy HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Accept: application/json
```

Query Parameters:
- `level`: Taxonomic level (phylum, class, order, family, genus, species)
- `min_abundance`: Minimum abundance threshold (0-1)
- `top_n`: Return only top N taxa
- `format`: Response format (json, csv, biom)

Response (200 OK):
```json
{
  "sample_id": "WSM-US-20251229-0001",
  "sequencing": {
    "platform": "illumina_miseq",
    "target_gene": "16S_v3v4",
    "read_count": 45230
  },
  "taxonomy": {
    "database": "SILVA_138",
    "classifier": "naive_bayes",
    "taxa": [
      {
        "otu_id": "ASV_0001",
        "abundance": 0.082,
        "taxonomy": {
          "phylum": "Proteobacteria",
          "class": "Alphaproteobacteria",
          "order": "Rhizobiales",
          "family": "Bradyrhizobiaceae",
          "genus": "Bradyrhizobium"
        }
      }
    ]
  },
  "diversity_metrics": {
    "shannon_index": 6.82,
    "simpson_index": 0.98,
    "observed_otus": 1247
  }
}
```

#### Compare Taxonomic Profiles

```http
POST /v1/taxonomy/compare HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "sample_ids": [
    "WSM-US-20251229-0001",
    "WSM-US-20251229-0002",
    "WSM-US-20251229-0003"
  ],
  "metrics": ["bray_curtis", "jaccard", "weighted_unifrac"],
  "level": "genus"
}
```

Response (200 OK):
```json
{
  "comparison": {
    "samples": 3,
    "metrics": {
      "bray_curtis": {
        "matrix": [
          [0.0, 0.234, 0.456],
          [0.234, 0.0, 0.321],
          [0.456, 0.321, 0.0]
        ]
      },
      "jaccard": { /* similar structure */ }
    },
    "ordination": {
      "method": "PCoA",
      "axes": [
        {"sample": "WSM-US-20251229-0001", "pc1": 0.123, "pc2": -0.456},
        {"sample": "WSM-US-20251229-0002", "pc1": -0.234, "pc2": 0.123}
      ],
      "explained_variance": [0.42, 0.28, 0.15]
    }
  }
}
```

### 4.3 Functional Analysis

#### Get Functional Profile

```http
GET /v1/samples/{sample_id}/functional HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
```

Query Parameters:
- `database`: Annotation database (KEGG, COG, eggNOG)
- `category`: Functional category filter
- `pathway`: Specific pathway ID
- `min_abundance`: Minimum abundance threshold

Response (200 OK):
```json
{
  "sample_id": "WSM-US-20251229-0001",
  "functional_data": {
    "annotation_database": "KEGG",
    "pathways": [
      {
        "pathway_id": "ko00910",
        "pathway_name": "Nitrogen metabolism",
        "abundance": 45.2,
        "abundance_units": "rpk",
        "genes": [
          {
            "gene_id": "K02586",
            "gene_name": "nifH",
            "description": "nitrogenase iron protein",
            "ec_number": "1.18.6.1",
            "abundance": 12.3
          }
        ]
      }
    ],
    "functional_guilds": {
      "nitrogen_cycle": {
        "nitrogen_fixation": 12.3,
        "nitrification": 8.7,
        "denitrification": 15.2
      }
    }
  }
}
```

#### Predict Functional Capacity

```http
POST /v1/functional/predict HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "sample_id": "WSM-US-20251229-0001",
  "method": "picrust2",
  "pathway_database": "MetaCyc",
  "options": {
    "stratified": true,
    "per_sequence_contrib": false
  }
}
```

Response (202 Accepted):
```json
{
  "job_id": "job_abc123xyz789",
  "status": "queued",
  "estimated_completion": "2025-12-29T11:00:00Z",
  "status_url": "/v1/jobs/job_abc123xyz789",
  "webhook_url": "https://your-app.com/webhooks/job-complete"
}
```

### 4.4 Analysis Jobs

#### Submit Analysis Job

```http
POST /v1/analysis/jobs HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "job_type": "differential_abundance",
  "parameters": {
    "sample_groups": {
      "treatment": ["WSM-US-20251229-0001", "WSM-US-20251229-0002"],
      "control": ["WSM-US-20251229-0003", "WSM-US-20251229-0004"]
    },
    "method": "DESeq2",
    "p_value_threshold": 0.05,
    "fold_change_threshold": 2.0
  },
  "priority": "normal",
  "callback_url": "https://your-app.com/webhooks/analysis-complete"
}
```

Response (202 Accepted):
```json
{
  "job_id": "job_xyz789abc123",
  "status": "queued",
  "queue_position": 12,
  "estimated_start": "2025-12-29T10:45:00Z",
  "estimated_completion": "2025-12-29T11:15:00Z",
  "status_url": "/v1/analysis/jobs/job_xyz789abc123"
}
```

#### Get Job Status

```http
GET /v1/analysis/jobs/{job_id} HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
```

Response (200 OK):
```json
{
  "job_id": "job_xyz789abc123",
  "status": "completed",
  "progress": 100,
  "started_at": "2025-12-29T10:45:23Z",
  "completed_at": "2025-12-29T11:12:45Z",
  "results": {
    "url": "/v1/analysis/jobs/job_xyz789abc123/results",
    "format": "json",
    "size_bytes": 1048576
  },
  "statistics": {
    "significant_features": 127,
    "total_features_tested": 1247,
    "processing_time_seconds": 1642
  }
}
```

Job Status Values:
- `queued`: Job is waiting in queue
- `running`: Job is currently processing
- `completed`: Job finished successfully
- `failed`: Job encountered an error
- `cancelled`: Job was cancelled by user

### 4.5 Diversity Analysis

#### Calculate Alpha Diversity

```http
GET /v1/diversity/alpha?sample_ids=WSM-US-20251229-0001,WSM-US-20251229-0002 HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
```

Query Parameters:
- `sample_ids`: Comma-separated sample IDs
- `metrics`: Comma-separated metrics (shannon, simpson, chao1, ace)
- `rarefaction_depth`: Rarefaction depth for normalization

Response (200 OK):
```json
{
  "metrics": ["shannon", "simpson", "observed_otus"],
  "rarefaction_depth": 10000,
  "results": [
    {
      "sample_id": "WSM-US-20251229-0001",
      "shannon": 6.82,
      "simpson": 0.98,
      "observed_otus": 1247
    },
    {
      "sample_id": "WSM-US-20251229-0002",
      "shannon": 5.94,
      "simpson": 0.95,
      "observed_otus": 983
    }
  ]
}
```

#### Calculate Beta Diversity

```http
POST /v1/diversity/beta HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "sample_ids": [
    "WSM-US-20251229-0001",
    "WSM-US-20251229-0002",
    "WSM-US-20251229-0003"
  ],
  "metric": "bray_curtis",
  "ordination": "PCoA",
  "phylogenetic_tree": false
}
```

Response (200 OK):
```json
{
  "metric": "bray_curtis",
  "distance_matrix": [
    [0.0, 0.234, 0.456],
    [0.234, 0.0, 0.321],
    [0.456, 0.321, 0.0]
  ],
  "ordination": {
    "method": "PCoA",
    "coordinates": [
      {"sample_id": "WSM-US-20251229-0001", "pc1": 0.123, "pc2": -0.456},
      {"sample_id": "WSM-US-20251229-0002", "pc1": -0.234, "pc2": 0.123},
      {"sample_id": "WSM-US-20251229-0003", "pc1": 0.111, "pc2": 0.333}
    ],
    "explained_variance": [0.42, 0.28, 0.15, 0.09, 0.06]
  }
}
```

### 4.6 Biomarker Queries

#### Get Biomarker Values

```http
GET /v1/samples/{sample_id}/biomarkers HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
```

Query Parameters:
- `category`: Biomarker category (nitrogen_cycle, carbon_cycle, etc.)
- `codes`: Comma-separated WIA biomarker codes

Response (200 OK):
```json
{
  "sample_id": "WSM-US-20251229-0001",
  "biomarkers": [
    {
      "code": "WSM-BM-101",
      "name": "nifH gene (N-fixation)",
      "value": 2.3e7,
      "units": "copies/g soil",
      "target_range": {"min": 1e6, "max": 1e8},
      "status": "within_range",
      "percentile": 68,
      "measurement_method": "qPCR",
      "measurement_date": "2025-12-29"
    },
    {
      "code": "WSM-BM-501",
      "name": "Shannon diversity (bacteria)",
      "value": 6.82,
      "units": "index",
      "target_range": {"min": 5.0},
      "status": "above_target",
      "percentile": 85
    }
  ],
  "summary": {
    "total_biomarkers": 12,
    "within_range": 8,
    "above_target": 3,
    "below_target": 1
  }
}
```

### 4.7 Bulk Operations

#### Bulk Sample Upload

```http
POST /v1/samples/bulk HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "samples": [
    { /* sample 1 metadata */ },
    { /* sample 2 metadata */ },
    { /* sample 3 metadata */ }
  ],
  "validation": {
    "strict": true,
    "stop_on_error": false
  },
  "callback_url": "https://your-app.com/webhooks/bulk-upload-complete"
}
```

Response (202 Accepted):
```json
{
  "batch_id": "batch_abc123xyz789",
  "total_samples": 150,
  "status": "processing",
  "validation_results": {
    "valid": 148,
    "invalid": 2,
    "warnings": 5
  },
  "status_url": "/v1/samples/bulk/batch_abc123xyz789"
}
```

#### Export Bulk Data

```http
GET /v1/export?sample_ids=WSM-US-20251229-0001,WSM-US-20251229-0002&format=biom HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
```

Query Parameters:
- `sample_ids`: Comma-separated sample IDs (or use filters)
- `format`: Export format (json, csv, biom, xlsx)
- `include`: Data types to include (metadata, taxonomy, functional)
- `filter[*]`: Various filters

Response (200 OK):
```json
{
  "export_id": "export_xyz789abc123",
  "status": "generating",
  "format": "biom",
  "estimated_size_mb": 45.2,
  "estimated_completion": "2025-12-29T10:40:00Z",
  "download_url": null,
  "expires_at": null
}
```

When ready:
```json
{
  "export_id": "export_xyz789abc123",
  "status": "ready",
  "format": "biom",
  "size_mb": 42.8,
  "download_url": "https://downloads.wia-soil-microbiome.org/exports/export_xyz789abc123.biom",
  "expires_at": "2025-12-30T10:40:00Z"
}
```

### 4.8 Search and Discovery

#### Advanced Search

```http
POST /v1/search HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "query": {
    "location": {
      "bounding_box": {
        "min_lat": 40.0,
        "max_lat": 45.0,
        "min_lon": -75.0,
        "max_lon": -70.0
      }
    },
    "soil_properties": {
      "ph": {"min": 6.0, "max": 7.5},
      "organic_matter_percent": {"min": 2.0}
    },
    "land_use": {
      "tillage_practice": ["no_till", "reduced_till"],
      "organic_certified": true
    },
    "collection_date": {
      "from": "2025-01-01",
      "to": "2025-12-31"
    }
  },
  "include": ["metadata", "diversity_metrics"],
  "limit": 100
}
```

Response (200 OK):
```json
{
  "total_results": 247,
  "results": [
    {
      "sample_id": "WSM-US-20251229-0001",
      "score": 0.95,
      "metadata": { /* sample metadata */ },
      "diversity_metrics": { /* diversity data */ }
    }
  ],
  "facets": {
    "crop_type": {
      "corn": 120,
      "soybean": 89,
      "wheat": 38
    },
    "texture_class": {
      "loam": 95,
      "silt_loam": 78,
      "sandy_loam": 74
    }
  }
}
```

## 5. Error Handling

### Standard Error Response Format

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {
      "field": "sample.depth_cm.min",
      "issue": "Value must be between 0 and 500"
    },
    "request_id": "req_abc123xyz789",
    "timestamp": "2025-12-29T10:30:00Z",
    "documentation_url": "https://docs.wia-soil-microbiome.org/errors/ERROR_CODE"
  }
}
```

### HTTP Status Codes

| Code | Status | Description | Retry |
|------|--------|-------------|-------|
| 200 | OK | Request succeeded | N/A |
| 201 | Created | Resource created successfully | N/A |
| 202 | Accepted | Async request accepted | Poll status |
| 204 | No Content | Request succeeded, no content | N/A |
| 400 | Bad Request | Invalid request syntax/parameters | No |
| 401 | Unauthorized | Missing or invalid authentication | No |
| 403 | Forbidden | Authenticated but not authorized | No |
| 404 | Not Found | Resource not found | No |
| 409 | Conflict | Resource conflict (duplicate) | No |
| 422 | Unprocessable Entity | Valid syntax but invalid data | No |
| 429 | Too Many Requests | Rate limit exceeded | Yes |
| 500 | Internal Server Error | Server error | Yes |
| 502 | Bad Gateway | Gateway error | Yes |
| 503 | Service Unavailable | Service temporarily unavailable | Yes |
| 504 | Gateway Timeout | Gateway timeout | Yes |

### Error Codes

| Error Code | HTTP Status | Description | Resolution |
|------------|-------------|-------------|------------|
| `INVALID_API_KEY` | 401 | API key is invalid or expired | Check API key, generate new one |
| `INSUFFICIENT_PERMISSIONS` | 403 | API key lacks required scope | Request additional scopes |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests | Wait and retry, upgrade tier |
| `INVALID_SAMPLE_ID` | 400 | Sample ID format invalid | Use WSM-{CC}-{DATE}-{SEQ} format |
| `SAMPLE_NOT_FOUND` | 404 | Sample does not exist | Verify sample ID |
| `DUPLICATE_SAMPLE_ID` | 409 | Sample ID already exists | Use unique sample ID |
| `INVALID_COORDINATES` | 422 | GPS coordinates out of range | Check lat/lon values |
| `MISSING_REQUIRED_FIELD` | 422 | Required field missing | Add required field to request |
| `VALIDATION_FAILED` | 422 | Data validation failed | Check validation details |
| `SEQUENCING_INCOMPLETE` | 422 | Sequencing not yet complete | Wait for sequencing completion |
| `INSUFFICIENT_READ_COUNT` | 422 | Read count below minimum | Increase sequencing depth |
| `ANALYSIS_IN_PROGRESS` | 409 | Analysis already running | Wait for current analysis |
| `JOB_NOT_FOUND` | 404 | Job ID not found | Verify job ID |
| `EXPORT_EXPIRED` | 404 | Export download expired | Generate new export |
| `INTERNAL_ERROR` | 500 | Internal server error | Retry, contact support |
| `SERVICE_UNAVAILABLE` | 503 | Service temporarily down | Retry after delay |

### Validation Error Details

```json
{
  "error": {
    "code": "VALIDATION_FAILED",
    "message": "Request validation failed",
    "validation_errors": [
      {
        "field": "sample.depth_cm.max",
        "value": 600,
        "constraint": "maximum",
        "limit": 500,
        "message": "Depth maximum cannot exceed 500 cm"
      },
      {
        "field": "soil_properties.texture.sand_percent",
        "value": null,
        "constraint": "required",
        "message": "Sand percentage is required when texture is provided"
      },
      {
        "field": "location.coordinates.latitude",
        "value": 95.0,
        "constraint": "range",
        "range": {"min": -90, "max": 90},
        "message": "Latitude must be between -90 and 90"
      }
    ],
    "request_id": "req_abc123xyz789"
  }
}
```

## 6. Webhooks

### Webhook Configuration

```http
POST /v1/webhooks HTTP/1.1
Authorization: ApiKey YOUR_API_KEY
Content-Type: application/json

{
  "url": "https://your-app.com/webhooks/wia-soil-microbiome",
  "events": [
    "sample.created",
    "sample.updated",
    "analysis.completed",
    "job.completed",
    "job.failed"
  ],
  "secret": "your_webhook_secret",
  "enabled": true
}
```

Response (201 Created):
```json
{
  "webhook_id": "webhook_abc123xyz789",
  "url": "https://your-app.com/webhooks/wia-soil-microbiome",
  "events": ["sample.created", "sample.updated", "analysis.completed"],
  "secret": "whsec_abc123xyz789...",
  "enabled": true,
  "created_at": "2025-12-29T10:30:00Z"
}
```

### Webhook Events

| Event | Description | Payload |
|-------|-------------|---------|
| `sample.created` | New sample created | Sample object |
| `sample.updated` | Sample metadata updated | Sample object |
| `sample.deleted` | Sample deleted | Sample ID |
| `sequencing.completed` | Sequencing finished | Sample ID, read count |
| `analysis.completed` | Analysis job completed | Job ID, results URL |
| `job.started` | Background job started | Job ID |
| `job.completed` | Background job completed | Job ID, results |
| `job.failed` | Background job failed | Job ID, error |
| `export.ready` | Bulk export ready | Export ID, download URL |

### Webhook Payload Example

```json
{
  "event": "analysis.completed",
  "timestamp": "2025-12-29T11:15:00Z",
  "webhook_id": "webhook_abc123xyz789",
  "data": {
    "job_id": "job_xyz789abc123",
    "job_type": "differential_abundance",
    "status": "completed",
    "results_url": "/v1/analysis/jobs/job_xyz789abc123/results",
    "sample_count": 4,
    "significant_features": 127
  },
  "signature": "sha256=abc123xyz789..."
}
```

### Webhook Signature Verification

```python
import hmac
import hashlib

def verify_webhook_signature(payload, signature, secret):
    expected_signature = hmac.new(
        secret.encode('utf-8'),
        payload.encode('utf-8'),
        hashlib.sha256
    ).hexdigest()
    return hmac.compare_digest(f"sha256={expected_signature}", signature)
```

## 7. Pagination

### Cursor-Based Pagination (Recommended)

```http
GET /v1/samples?limit=50&cursor=eyJpZCI6IldTTS1VUy0yMDI1MTIyOS0wMDUwIn0 HTTP/1.1
```

Response:
```json
{
  "data": [ /* sample objects */ ],
  "pagination": {
    "limit": 50,
    "has_more": true,
    "next_cursor": "eyJpZCI6IldTTS1VUy0yMDI1MTIyOS0wMTAwIn0"
  }
}
```

### Offset-Based Pagination

```http
GET /v1/samples?offset=100&limit=50 HTTP/1.1
```

Response:
```json
{
  "data": [ /* sample objects */ ],
  "pagination": {
    "total": 1247,
    "offset": 100,
    "limit": 50,
    "has_more": true
  }
}
```

## 8. Filtering and Sorting

### Filter Syntax

```http
GET /v1/samples?filter[ph][gte]=6.0&filter[ph][lte]=7.5&filter[crop]=corn HTTP/1.1
```

Filter Operators:
- `eq`: Equal
- `ne`: Not equal
- `gt`: Greater than
- `gte`: Greater than or equal
- `lt`: Less than
- `lte`: Less than or equal
- `in`: In array
- `nin`: Not in array
- `contains`: Contains substring
- `startswith`: Starts with

### Sorting

```http
GET /v1/samples?sort=created_at:desc,ph:asc HTTP/1.1
```

## 9. API Monitoring and Health

### Health Check Endpoint

```http
GET /v1/health HTTP/1.1
```

Response (200 OK):
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-29T10:30:00Z",
  "services": {
    "database": "healthy",
    "sequencing_pipeline": "healthy",
    "analysis_queue": "healthy",
    "storage": "healthy"
  },
  "performance": {
    "avg_response_time_ms": 145,
    "requests_per_second": 234
  }
}
```

### API Status Page

Public status page: `https://status.wia-soil-microbiome.org`

## 10. SDK and Client Libraries

### Official SDKs

- **Python**: `pip install wia-soil-microbiome`
- **JavaScript/Node.js**: `npm install @wia/soil-microbiome`
- **R**: `install.packages("WIASoilMicrobiome")`
- **Java**: Maven/Gradle dependency available
- **Go**: `go get github.com/wia-official/soil-microbiome-go`

### Example Usage (Python)

```python
from wia_soil_microbiome import Client

client = Client(api_key="wia_sk_live_abc123...")

# Create sample
sample = client.samples.create({
    "sample": {
        "id": "WSM-US-20251229-0001",
        "type": "rhizosphere",
        "depth_cm": {"min": 0, "max": 15}
    },
    "location": {
        "coordinates": {"latitude": 40.7128, "longitude": -74.0060}
    }
})

# Get taxonomy
taxonomy = client.samples.get_taxonomy("WSM-US-20251229-0001")

# Calculate diversity
diversity = client.diversity.alpha(
    sample_ids=["WSM-US-20251229-0001", "WSM-US-20251229-0002"],
    metrics=["shannon", "simpson"]
)
```

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
**弘益人間 · Benefit All Humanity**
