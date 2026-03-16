# WIA Smart Breeding API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication & Authorization](#authentication--authorization)
4. [Genomic Data API](#genomic-data-api)
5. [Pedigree Management API](#pedigree-management-api)
6. [Phenotype Recording API](#phenotype-recording-api)
7. [Breeding Value API](#breeding-value-api)
8. [Mating Plan API](#mating-plan-api)
9. [Genetic Diversity API](#genetic-diversity-api)
10. [Error Handling](#error-handling)
11. [Rate Limiting](#rate-limiting)
12. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Smart Breeding API Interface Standard defines RESTful APIs for genomic breeding programs, enabling seamless data exchange between breeding systems, gene banks, research institutes, and breeding companies.

**API Objectives**:
- Provide standardized endpoints for genomic data upload/download
- Enable real-time breeding value calculations and queries
- Support pedigree verification and relationship queries
- Facilitate phenotype data collection from multiple sources
- Enable optimal mating plan generation
- Support genetic diversity monitoring and alerts

### 1.2 Base URL

```
Production:  https://api.wia-breeding.org/v1
Staging:     https://staging-api.wia-breeding.org/v1
Sandbox:     https://sandbox-api.wia-breeding.org/v1
```

### 1.3 API Design Principles

1. **RESTful**: Standard HTTP methods (GET, POST, PUT, DELETE, PATCH)
2. **JSON**: All requests and responses use JSON format
3. **Versioned**: API version in URL path (/v1, /v2, ...)
4. **Paginated**: Large result sets use cursor-based pagination
5. **Secure**: OAuth 2.0 + JWT authentication, HTTPS only
6. **Idempotent**: Safe retries using idempotency keys

---

## API Architecture

### 2.1 Architecture Overview

```
┌─────────────────────────────────────────────────┐
│         WIA Breeding API Gateway                │
│  (Authentication, Rate Limiting, Logging)       │
└─────────────────────────────────────────────────┘
                     │
      ┌──────────────┼──────────────┐
      │              │              │
┌─────▼─────┐  ┌────▼────┐  ┌──────▼──────┐
│ Genomic   │  │ Pedigree│  │  Phenotype  │
│  Service  │  │ Service │  │   Service   │
└───────────┘  └─────────┘  └─────────────┘
      │              │              │
┌─────▼─────┐  ┌────▼────┐  ┌──────▼──────┐
│ Breeding  │  │  Mating │  │  Diversity  │
│ Value Svc │  │ Plan Svc│  │   Service   │
└───────────┘  └─────────┘  └─────────────┘
```

### 2.2 HTTP Methods

| Method | Usage | Idempotent |
|--------|-------|------------|
| GET | Retrieve resources | ✅ Yes |
| POST | Create new resources | ❌ No |
| PUT | Replace entire resource | ✅ Yes |
| PATCH | Partial update | ❌ No |
| DELETE | Remove resource | ✅ Yes |

### 2.3 Standard Headers

```http
Content-Type: application/json
Authorization: Bearer {JWT_TOKEN}
X-API-Key: {API_KEY}
X-Request-ID: {UUID}
X-Idempotency-Key: {UUID}
Accept-Language: en-US, ko-KR
```

---

## Authentication & Authorization

### 3.1 OAuth 2.0 Flow

**Step 1: Get Access Token**

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={CLIENT_ID}
&client_secret={CLIENT_SECRET}
&scope=breeding:read breeding:write
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "breeding:read breeding:write"
}
```

**Step 2: Use Access Token**

```http
GET /v1/individuals/BULL-2025-001
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 3.2 API Scopes

| Scope | Description |
|-------|-------------|
| `breeding:read` | Read genomic, pedigree, phenotype data |
| `breeding:write` | Create/update breeding records |
| `breeding:admin` | Manage users, permissions, data deletion |
| `genomic:read` | Read SNP genotypes, sequences |
| `genomic:write` | Upload genomic data |
| `ebv:read` | Read breeding values (EBV/GEBV) |
| `ebv:calculate` | Trigger EBV calculations |

### 3.3 JWT Token Structure

```json
{
  "sub": "user@breeding-org.com",
  "iss": "https://auth.wia-breeding.org",
  "aud": "https://api.wia-breeding.org",
  "exp": 1735689600,
  "iat": 1735686000,
  "scope": "breeding:read breeding:write",
  "organization": "National Livestock Research Institute",
  "role": "BREEDER"
}
```

---

## Genomic Data API

### 4.1 Upload SNP Genotypes

```http
POST /v1/genomic/genotypes
Authorization: Bearer {TOKEN}
Content-Type: application/json
```

**Request Body:**

```json
{
  "individual_id": "BULL-2025-001",
  "platform": "Illumina BovineSNP50",
  "genotyping_date": "2025-01-10",
  "genotypes": [
    {
      "marker_id": "SNP_1",
      "chromosome": 1,
      "position": 135098,
      "genotype": 1,
      "allele_A": "C",
      "allele_B": "T"
    },
    {
      "marker_id": "SNP_2",
      "chromosome": 1,
      "position": 267891,
      "genotype": 2
    }
  ]
}
```

**Response (201 Created):**

```json
{
  "genotype_id": "GENO-2025-001",
  "individual_id": "BULL-2025-001",
  "total_markers": 54001,
  "call_rate": 0.985,
  "status": "PROCESSED",
  "created_at": "2025-01-15T10:30:00Z"
}
```

### 4.2 Get Genotype Data

```http
GET /v1/genomic/genotypes/{individual_id}
Authorization: Bearer {TOKEN}
```

**Query Parameters:**

- `format`: Output format (`json`, `vcf`, `plink`)
- `chromosome`: Filter by chromosome (1-30)
- `start_position`: Start position (bp)
- `end_position`: End position (bp)

**Response (200 OK):**

```json
{
  "individual_id": "BULL-2025-001",
  "platform": "Illumina BovineSNP50",
  "total_markers": 54001,
  "call_rate": 0.985,
  "heterozygosity": 0.352,
  "download_url": "https://api.wia-breeding.org/downloads/GENO-2025-001.vcf.gz",
  "expires_at": "2025-01-16T10:30:00Z"
}
```

### 4.3 Calculate Genomic Relationship Matrix

```http
POST /v1/genomic/g-matrix
Authorization: Bearer {TOKEN}
Content-Type: application/json
```

**Request Body:**

```json
{
  "individuals": ["BULL-2025-001", "BULL-2025-002", "COW-2025-003"],
  "method": "VanRaden_Method1",
  "maf_threshold": 0.01,
  "impute_missing": true
}
```

**Response (200 OK):**

```json
{
  "g_matrix_id": "GMAT-2025-001",
  "dimension": 3,
  "status": "COMPLETED",
  "download_url": "https://api.wia-breeding.org/downloads/GMAT-2025-001.csv",
  "computation_time_seconds": 12.5
}
```

---

## Pedigree Management API

### 5.1 Create Pedigree Record

```http
POST /v1/pedigree
Authorization: Bearer {TOKEN}
Content-Type: application/json
```

**Request Body:**

```json
{
  "individual_id": "BULL-2025-001",
  "sire_id": "BULL-2020-045",
  "dam_id": "COW-2018-123",
  "birth_date": "2025-01-15",
  "registration_number": "HOLKOR-2025-001234"
}
```

**Response (201 Created):**

```json
{
  "pedigree_id": "PED-2025-001",
  "individual_id": "BULL-2025-001",
  "sire_verified": true,
  "dam_verified": true,
  "inbreeding_coefficient": 0.045,
  "created_at": "2025-01-15T10:30:00Z"
}
```

### 5.2 Calculate Inbreeding Coefficient

```http
GET /v1/pedigree/{individual_id}/inbreeding
Authorization: Bearer {TOKEN}
```

**Query Parameters:**

- `generations`: Number of generations to trace (default: 5)
- `method`: Calculation method (`meuwissen_luo`, `sargolzaei`)

**Response (200 OK):**

```json
{
  "individual_id": "BULL-2025-001",
  "inbreeding_coefficient": 0.045,
  "method": "meuwissen_luo_1992",
  "generations_traced": 8,
  "common_ancestors": [
    {
      "ancestor_id": "BULL-2010-001",
      "contribution": 0.025
    }
  ]
}
```

### 5.3 Get Pedigree Tree

```http
GET /v1/pedigree/{individual_id}/tree
Authorization: Bearer {TOKEN}
```

**Query Parameters:**

- `depth`: Tree depth (1=parents, 2=grandparents, ...)
- `direction`: `ancestors` or `descendants`

**Response (200 OK):**

```json
{
  "individual_id": "BULL-2025-001",
  "tree": {
    "sire": {
      "id": "BULL-2020-045",
      "sire": {"id": "BULL-2015-012"},
      "dam": {"id": "COW-2016-089"}
    },
    "dam": {
      "id": "COW-2018-123",
      "sire": {"id": "BULL-2014-034"},
      "dam": {"id": "COW-2015-067"}
    }
  },
  "completeness": {
    "generation_1": 1.0,
    "generation_2": 1.0,
    "generation_3": 0.875
  }
}
```

---

## Phenotype Recording API

### 6.1 Submit Phenotype Record

```http
POST /v1/phenotype
Authorization: Bearer {TOKEN}
Content-Type: application/json
```

**Request Body:**

```json
{
  "individual_id": "BULL-2025-001",
  "measurements": [
    {
      "trait_code": "MILK_YIELD",
      "value": 9200,
      "unit": "kg",
      "measurement_date": "2026-11-15",
      "lactation_number": 1
    },
    {
      "trait_code": "FAT_PERCENTAGE",
      "value": 3.85,
      "unit": "%",
      "measurement_date": "2026-11-15"
    }
  ],
  "contemporary_group": "HERD_A_2026_WINTER"
}
```

**Response (201 Created):**

```json
{
  "phenotype_id": "PHENO-2025-001",
  "individual_id": "BULL-2025-001",
  "traits_recorded": 2,
  "status": "VALIDATED",
  "created_at": "2025-01-15T10:30:00Z"
}
```

### 6.2 Get Phenotype History

```http
GET /v1/phenotype/{individual_id}
Authorization: Bearer {TOKEN}
```

**Query Parameters:**

- `trait_code`: Filter by trait (MILK_YIELD, FAT_PERCENTAGE, ...)
- `start_date`: Start date (YYYY-MM-DD)
- `end_date`: End date (YYYY-MM-DD)

**Response (200 OK):**

```json
{
  "individual_id": "BULL-2025-001",
  "total_records": 15,
  "measurements": [
    {
      "trait_code": "MILK_YIELD",
      "value": 9200,
      "unit": "kg",
      "measurement_date": "2026-11-15",
      "standardized_value": 1.45,
      "percentile": 85
    }
  ]
}
```

---

## Breeding Value API

### 7.1 Calculate Breeding Value (EBV/GEBV)

```http
POST /v1/breeding-value/calculate
Authorization: Bearer {TOKEN}
Content-Type: application/json
```

**Request Body:**

```json
{
  "individuals": ["BULL-2025-001", "BULL-2025-002"],
  "traits": ["MILK_YIELD", "FAT_PERCENTAGE"],
  "method": "GBLUP",
  "reference_population": "HOLSTEIN_KR_2025",
  "heritability": {
    "MILK_YIELD": 0.35,
    "FAT_PERCENTAGE": 0.42
  }
}
```

**Response (202 Accepted):**

```json
{
  "job_id": "EBV-JOB-2025-001",
  "status": "PROCESSING",
  "estimated_completion": "2025-01-15T10:35:00Z",
  "status_url": "/v1/breeding-value/jobs/EBV-JOB-2025-001"
}
```

### 7.2 Get Breeding Value

```http
GET /v1/breeding-value/{individual_id}
Authorization: Bearer {TOKEN}
```

**Response (200 OK):**

```json
{
  "individual_id": "BULL-2025-001",
  "method": "GBLUP",
  "evaluation_date": "2025-01-15",
  "traits": [
    {
      "trait_code": "MILK_YIELD",
      "gebv": 920,
      "accuracy": 0.88,
      "reliability": 0.77,
      "percentile": 95,
      "breeding_category": "ELITE"
    }
  ],
  "selection_index": {
    "total_merit_index": 2450,
    "rank": 12,
    "percentile": 98
  }
}
```

### 7.3 Get Top Breeding Candidates

```http
GET /v1/breeding-value/top-candidates
Authorization: Bearer {TOKEN}
```

**Query Parameters:**

- `trait_code`: Trait to rank by
- `sex`: MALE or FEMALE
- `limit`: Number of top individuals (default: 100)
- `min_accuracy`: Minimum accuracy threshold (default: 0.70)

**Response (200 OK):**

```json
{
  "trait_code": "MILK_YIELD",
  "sex": "MALE",
  "total_candidates": 5420,
  "top_candidates": [
    {
      "rank": 1,
      "individual_id": "BULL-2024-789",
      "gebv": 1050,
      "accuracy": 0.92,
      "percentile": 99.5
    },
    {
      "rank": 2,
      "individual_id": "BULL-2025-001",
      "gebv": 920,
      "accuracy": 0.88,
      "percentile": 98.2
    }
  ]
}
```

---

## Mating Plan API

### 8.1 Generate Optimal Mating Plan

```http
POST /v1/mating/optimal-plan
Authorization: Bearer {TOKEN}
Content-Type: application/json
```

**Request Body:**

```json
{
  "sires": ["BULL-2024-789", "BULL-2025-001"],
  "dams": ["COW-2023-101", "COW-2023-102", "COW-2023-103"],
  "objectives": {
    "maximize_genetic_gain": 0.70,
    "minimize_inbreeding": 0.30
  },
  "constraints": {
    "max_inbreeding": 0.0625,
    "max_matings_per_sire": 50
  }
}
```

**Response (200 OK):**

```json
{
  "mating_plan_id": "MATE-2025-001",
  "total_matings": 3,
  "matings": [
    {
      "sire_id": "BULL-2024-789",
      "dam_id": "COW-2023-101",
      "expected_offspring_gebv": 485,
      "expected_inbreeding": 0.032,
      "recommendation_score": 0.95
    },
    {
      "sire_id": "BULL-2025-001",
      "dam_id": "COW-2023-102",
      "expected_offspring_gebv": 460,
      "expected_inbreeding": 0.045,
      "recommendation_score": 0.88
    }
  ],
  "expected_genetic_gain": "+125 kg milk per generation"
}
```

### 8.2 Check Mating Compatibility

```http
GET /v1/mating/compatibility
Authorization: Bearer {TOKEN}
```

**Query Parameters:**

- `sire_id`: Sire individual ID
- `dam_id`: Dam individual ID

**Response (200 OK):**

```json
{
  "sire_id": "BULL-2025-001",
  "dam_id": "COW-2023-101",
  "compatible": true,
  "expected_inbreeding": 0.032,
  "genetic_distance": 0.145,
  "warnings": [],
  "expected_offspring": {
    "gebv_milk_yield": 485,
    "accuracy": 0.65
  }
}
```

---

## Genetic Diversity API

### 9.1 Get Population Diversity

```http
GET /v1/diversity/population
Authorization: Bearer {TOKEN}
```

**Query Parameters:**

- `population`: Population name (HOLSTEIN_KR, DUROC_US, ...)
- `year`: Evaluation year (default: current year)

**Response (200 OK):**

```json
{
  "population": "HOLSTEIN_KR",
  "year": 2025,
  "metrics": {
    "population_size": 25000,
    "effective_population_size": 85,
    "observed_heterozygosity": 0.352,
    "expected_heterozygosity": 0.368,
    "inbreeding_rate": 0.0059,
    "genetic_drift": 0.0118
  },
  "status": "MODERATE_CONCERN",
  "recommendation": "Increase Ne to >100 through strategic mating"
}
```

### 9.2 Monitor Diversity Trends

```http
GET /v1/diversity/trends
Authorization: Bearer {TOKEN}
```

**Response (200 OK):**

```json
{
  "population": "HOLSTEIN_KR",
  "time_series": [
    {
      "year": 2020,
      "effective_population_size": 105,
      "inbreeding_rate": 0.0048
    },
    {
      "year": 2025,
      "effective_population_size": 85,
      "inbreeding_rate": 0.0059
    }
  ],
  "trend": "DECLINING",
  "alert_level": "WARNING"
}
```

---

## Error Handling

### 10.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_GENOTYPE",
    "message": "SNP genotype must be 0, 1, 2, or -9",
    "details": {
      "marker_id": "SNP_12345",
      "invalid_value": 5
    },
    "request_id": "req_abc123xyz",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 10.2 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET, PUT, PATCH, DELETE |
| 201 | Created | Successful POST |
| 202 | Accepted | Async operation started |
| 400 | Bad Request | Invalid input data |
| 401 | Unauthorized | Missing/invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Duplicate resource |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance mode |

---

## Rate Limiting

### 11.1 Rate Limits

| Tier | Requests/Hour | Requests/Day |
|------|---------------|--------------|
| Free | 100 | 1,000 |
| Basic | 1,000 | 10,000 |
| Pro | 10,000 | 100,000 |
| Enterprise | Unlimited | Unlimited |

### 11.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735689600
```

---

## Examples

### 12.1 Complete Workflow: Upload → Calculate EBV → Get Results

```bash
# 1. Upload genotype
curl -X POST https://api.wia-breeding.org/v1/genomic/genotypes \
  -H "Authorization: Bearer {TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{"individual_id": "BULL-2025-001", "genotypes": [...]}'

# 2. Calculate breeding value
curl -X POST https://api.wia-breeding.org/v1/breeding-value/calculate \
  -H "Authorization: Bearer {TOKEN}" \
  -d '{"individuals": ["BULL-2025-001"], "method": "GBLUP"}'

# 3. Get results
curl https://api.wia-breeding.org/v1/breeding-value/BULL-2025-001 \
  -H "Authorization: Bearer {TOKEN}"
```

---

**© 2025 WIA (World Certification Industry Association)**
**弘益人間 · Benefit All Humanity**
**License**: MIT
