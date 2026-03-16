# WIA-SOC-016 PHASE 2: API Specification

## Census Data Standard - REST API Design

**Version:** 1.0
**Status:** PUBLISHED
**Last Updated:** 2025-12-26

---

## 1. Overview

This document specifies the RESTful API for census data access under WIA-SOC-016 standard.

### 1.1 Base URL

```
https://api.census.wia.org/v1
```

### 1.2 Authentication

All API requests require authentication via API key:

```
Authorization: Bearer YOUR_API_KEY
```

---

## 2. Core Endpoints

### 2.1 Population Statistics

```
GET /population
GET /population/{geo_code}
GET /population/{geo_code}/{characteristic}
```

**Query Parameters:**
- `year` (integer): Census year
- `geo_level` (enum): nation, state, county, tract, block
- `age_group` (string): Filter by age range
- `sex` (enum): MALE, FEMALE, ALL
- `format` (enum): json, csv, xml

**Example Request:**
```bash
curl -H "Authorization: Bearer API_KEY" \
  "https://api.census.wia.org/v1/population/USA-CA?year=2025&format=json"
```

**Example Response:**
```json
{
  "data": {
    "geography": {
      "code": "USA-CA",
      "name": "California",
      "level": "state"
    },
    "population": {
      "total": 39538223,
      "male": 19639838,
      "female": 19898385,
      "median_age": 37.0
    },
    "quality": {
      "margin_of_error": 125000,
      "confidence_level": 0.95
    }
  },
  "metadata": {
    "source": "Census 2025",
    "privacy_method": "differential_privacy",
    "epsilon": 1.0
  }
}
```

### 2.2 Demographics

```
GET /demographics
GET /demographics/{geo_code}
```

**Dimensions:**
- Age distribution
- Education levels
- Marital status
- Citizenship
- Language spoken

### 2.3 Housing Data

```
GET /housing
GET /housing/{geo_code}
```

**Includes:**
- Housing units by type
- Tenure (owned/rented)
- Monthly housing costs
- Utilities and amenities

### 2.4 Economic Indicators

```
GET /economy
GET /economy/{geo_code}
```

**Includes:**
- Labor force participation
- Unemployment rate
- Occupation distribution
- Income distribution
- Commuting patterns

### 2.5 Time Series

```
GET /timeseries/{variable}
```

**Parameters:**
- `start_year`, `end_year`
- `geo_code`
- `frequency` (annual, intercensal)

---

## 3. Data Formats

### 3.1 JSON (Default)

Content-Type: `application/json`

### 3.2 CSV

Content-Type: `text/csv`

```
geo_code,geo_name,total_population,median_age
USA-CA,California,39538223,37.0
USA-TX,Texas,29145505,35.5
```

### 3.3 XML

Content-Type: `application/xml`

### 3.4 SDMX

Content-Type: `application/vnd.sdmx.data+xml`

---

## 4. Error Handling

```json
{
  "error": {
    "code": "INVALID_GEO_CODE",
    "message": "Geographic code 'XYZ' not found",
    "status": 404,
    "details": {
      "valid_codes": ["USA-CA", "USA-TX", ...]
    }
  }
}
```

**HTTP Status Codes:**
- 200: Success
- 400: Bad Request
- 401: Unauthorized
- 404: Not Found
- 429: Rate Limit Exceeded
- 500: Internal Server Error

---

## 5. Rate Limiting

**Public:** 100 requests/hour
**Registered:** 1,000 requests/hour
**Government:** 10,000 requests/hour

Headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640995200
```

---

## 6. Pagination

```
GET /population?page=1&per_page=100
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "page": 1,
    "per_page": 100,
    "total": 5432,
    "total_pages": 55
  },
  "links": {
    "first": "/population?page=1",
    "prev": null,
    "next": "/population?page=2",
    "last": "/population?page=55"
  }
}
```

---

## 7. Filtering and Querying

```
GET /population?filter[age]=25-34&filter[sex]=FEMALE&filter[education]=BACHELORS
```

---

## 8. GraphQL Alternative

```
POST /graphql
```

**Query:**
```graphql
query {
  population(geoCode: "USA-CA") {
    total
    byAge {
      ageGroup
      count
    }
    bySex {
      sex
      count
    }
  }
}
```

---

## 9. WebSocket for Real-time Updates

```
wss://api.census.wia.org/ws
```

Subscribe to live census processing updates.

---

## 10. SDK Support

Official SDKs available for:
- Python
- R
- JavaScript/TypeScript
- Java
- Go

---

**Document Version:** 1.0
© 2025 SmileStory Inc. / WIA
