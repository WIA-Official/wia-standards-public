# WIA-AGRI-025: Insect Protein Standard
## PHASE 2: API Interface Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-025

---

## 1. Overview

This specification defines REST and GraphQL API interfaces for insect protein data exchange, enabling seamless integration between farms, processors, certification bodies, and retail systems.

### 1.1 Base URL

**Production:** `https://api.wia-agri.org/v1`
**Staging:** `https://staging-api.wia-agri.org/v1`
**Development:** `https://dev-api.wia-agri.org/v1`

### 1.2 API Versions

- **v1**: Current stable version (this specification)
- Version format: `/v{major}`
- Breaking changes increment major version

---

## 2. Authentication

### 2.1 API Key Authentication

All API requests require an API key in the Authorization header.

```http
Authorization: Bearer YOUR_API_KEY
```

**Example:**
```bash
curl -X GET https://api.wia-agri.org/v1/products \
  -H "Authorization: Bearer wia_live_abc123xyz789" \
  -H "Content-Type: application/json"
```

### 2.2 OAuth 2.0

For user-authenticated actions, OAuth 2.0 is supported.

**Authorization Endpoint:** `/oauth/authorize`
**Token Endpoint:** `/oauth/token`

**Supported Grant Types:**
- `authorization_code`
- `client_credentials`
- `refresh_token`

**Example:**
```bash
curl -X POST https://api.wia-agri.org/oauth/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=client_credentials" \
  -d "client_id=YOUR_CLIENT_ID" \
  -d "client_secret=YOUR_CLIENT_SECRET"
```

---

## 3. REST API Endpoints

### 3.1 Products

#### GET /products

List all insect protein products with filtering and pagination.

**Query Parameters:**
- `species` (string): Filter by species (e.g., "cricket", "mealworm")
- `form` (string): Filter by product form (e.g., "powder", "whole")
- `certified` (boolean): Filter by certification status
- `min_protein` (number): Minimum protein content (g/100g)
- `farm_id` (string): Filter by farm ID
- `page` (integer): Page number (default: 1)
- `limit` (integer): Items per page (default: 20, max: 100)
- `sort` (string): Sort field (e.g., "productionDate", "protein")
- `order` (string): Sort order ("asc" or "desc")

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "id": "PRODUCT-001",
      "species": {
        "commonName": "House Cricket",
        "scientificName": "Acheta domesticus"
      },
      "productForm": "powder",
      "batchNumber": "BATCH-2025-001",
      "nutrition": {
        "protein": 65.5
      },
      "certified": true
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 150,
    "totalPages": 8
  }
}
```

**Status Codes:**
- `200 OK`: Success
- `400 Bad Request`: Invalid parameters
- `401 Unauthorized`: Invalid API key
- `429 Too Many Requests`: Rate limit exceeded

#### POST /products

Create a new insect protein product record.

**Request Body:**
```json
{
  "species": {
    "commonName": "House Cricket",
    "scientificName": "Acheta domesticus"
  },
  "productForm": "powder",
  "batchNumber": "BATCH-2025-002",
  "productionDate": "2025-12-20T10:00:00Z",
  "farmInfo": {
    "farmId": "FARM-KR-001",
    "farmName": "Seoul Cricket Farm"
  },
  "nutrition": {
    "macronutrients": {
      "protein": {
        "value": 65.5,
        "unit": "g/100g"
      }
    }
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "id": "PRODUCT-NEW-001",
    "message": "Product created successfully",
    "createdAt": "2025-12-26T12:00:00Z"
  }
}
```

**Status Codes:**
- `201 Created`: Product created successfully
- `400 Bad Request`: Invalid data
- `401 Unauthorized`: Invalid API key
- `409 Conflict`: Batch number already exists

#### GET /products/{id}

Retrieve a specific product by ID.

**Path Parameters:**
- `id` (string): Product ID

**Response:**
```json
{
  "success": true,
  "data": {
    "id": "PRODUCT-001",
    "version": "1.0",
    "standard": "WIA-AGRI-025",
    "species": {
      "commonName": "House Cricket",
      "scientificName": "Acheta domesticus"
    },
    "productForm": "powder",
    "nutrition": { /* Full nutrition profile */ },
    "safety": { /* Full safety profile */ },
    "sustainability": { /* Full sustainability metrics */ }
  }
}
```

**Status Codes:**
- `200 OK`: Success
- `404 Not Found`: Product not found

#### PUT /products/{id}

Update an existing product record.

**Path Parameters:**
- `id` (string): Product ID

**Request Body:**
```json
{
  "nutrition": {
    "macronutrients": {
      "protein": {
        "value": 66.0,
        "unit": "g/100g"
      }
    }
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "id": "PRODUCT-001",
    "message": "Product updated successfully",
    "updatedAt": "2025-12-26T12:30:00Z"
  }
}
```

**Status Codes:**
- `200 OK`: Update successful
- `400 Bad Request`: Invalid data
- `404 Not Found`: Product not found

#### DELETE /products/{id}

Delete a product record.

**Path Parameters:**
- `id` (string): Product ID

**Response:**
```json
{
  "success": true,
  "message": "Product deleted successfully"
}
```

**Status Codes:**
- `200 OK`: Deletion successful
- `404 Not Found`: Product not found
- `403 Forbidden`: Cannot delete certified product

### 3.2 Nutrition

#### GET /nutrition/{productId}

Get detailed nutrition profile for a product.

**Path Parameters:**
- `productId` (string): Product ID

**Response:**
```json
{
  "success": true,
  "data": {
    "productId": "PRODUCT-001",
    "macronutrients": { /* ... */ },
    "aminoAcids": { /* ... */ },
    "vitamins": { /* ... */ },
    "minerals": { /* ... */ },
    "energyContent": { /* ... */ }
  }
}
```

#### POST /nutrition/compare

Compare nutrition profiles of multiple products.

**Request Body:**
```json
{
  "productIds": ["PRODUCT-001", "PRODUCT-002", "PRODUCT-003"]
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "comparison": [
      {
        "productId": "PRODUCT-001",
        "protein": 65.5,
        "fat": 15.2
      },
      {
        "productId": "PRODUCT-002",
        "protein": 58.3,
        "fat": 18.5
      }
    ]
  }
}
```

### 3.3 Safety

#### GET /safety/{productId}

Get safety testing results for a product.

**Response:**
```json
{
  "success": true,
  "data": {
    "productId": "PRODUCT-001",
    "microbiological": { /* ... */ },
    "heavyMetals": { /* ... */ },
    "allergens": { /* ... */ },
    "overallStatus": "passed",
    "testDate": "2025-12-16"
  }
}
```

#### POST /safety/alerts

Report a safety issue or alert.

**Request Body:**
```json
{
  "productId": "PRODUCT-001",
  "alertType": "contamination|recall|quality-issue",
  "severity": "low|medium|high|critical",
  "description": "Batch contamination detected",
  "affectedBatches": ["BATCH-2025-001"],
  "reportedBy": "Lab-XYZ"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "alertId": "ALERT-001",
    "status": "investigating",
    "createdAt": "2025-12-26T14:00:00Z"
  }
}
```

### 3.4 Sustainability

#### GET /sustainability/{productId}

Get sustainability metrics for a product.

**Response:**
```json
{
  "success": true,
  "data": {
    "productId": "PRODUCT-001",
    "carbonFootprint": {
      "total": 0.5,
      "unit": "kg CO2e/kg product"
    },
    "waterUsage": {
      "total": 1.2,
      "unit": "L/kg product"
    },
    "feedConversionRatio": 1.7,
    "score": 95
  }
}
```

#### GET /sustainability/comparison

Compare sustainability metrics across different protein sources.

**Query Parameters:**
- `product_id` (string): Product ID to compare
- `compare_with` (string[]): Protein types (e.g., "beef", "chicken", "soy")

**Response:**
```json
{
  "success": true,
  "data": {
    "insectProtein": {
      "carbonFootprint": 0.5,
      "waterUsage": 1.2,
      "landUse": 15
    },
    "beef": {
      "carbonFootprint": 27.0,
      "waterUsage": 15400,
      "landUse": 326
    },
    "chicken": {
      "carbonFootprint": 6.9,
      "waterUsage": 4325,
      "landUse": 51
    }
  }
}
```

### 3.5 Certifications

#### GET /certifications

List all certifications.

**Query Parameters:**
- `status` (string): Filter by status ("active", "expired", "pending")
- `product_id` (string): Filter by product
- `farm_id` (string): Filter by farm

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "certificationId": "CERT-001",
      "productId": "PRODUCT-001",
      "standardVersion": "1.0",
      "status": "active",
      "issueDate": "2025-12-20",
      "expiryDate": "2026-12-20"
    }
  ]
}
```

#### POST /certifications

Request a new certification.

**Request Body:**
```json
{
  "productId": "PRODUCT-001",
  "certificationBody": "WIA Certification Authority",
  "requestedBy": {
    "name": "John Doe",
    "email": "john@example.com",
    "organization": "Seoul Cricket Farm"
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "requestId": "CERT-REQ-001",
    "status": "pending",
    "estimatedCompletionDate": "2026-01-15"
  }
}
```

#### GET /certifications/{id}/verify

Verify a certification's authenticity.

**Response:**
```json
{
  "success": true,
  "data": {
    "certificationId": "CERT-001",
    "valid": true,
    "productId": "PRODUCT-001",
    "issuer": "WIA Certification Authority",
    "issueDate": "2025-12-20",
    "expiryDate": "2026-12-20",
    "blockchain": {
      "verified": true,
      "transactionHash": "0x..."
    }
  }
}
```

### 3.6 Batches

#### GET /batches

List production batches.

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "batchNumber": "BATCH-2025-001",
      "productId": "PRODUCT-001",
      "productionDate": "2025-12-15",
      "quantity": {
        "value": 500,
        "unit": "kg"
      },
      "status": "in-production|completed|shipped|recalled"
    }
  ]
}
```

#### POST /batches

Create a new production batch.

**Request Body:**
```json
{
  "batchNumber": "BATCH-2025-003",
  "farmId": "FARM-KR-001",
  "species": "Acheta domesticus",
  "productionDate": "2025-12-26T08:00:00Z",
  "estimatedQuantity": {
    "value": 300,
    "unit": "kg"
  }
}
```

### 3.7 Farms

#### GET /farms

List registered farms.

**Response:**
```json
{
  "success": true,
  "data": [
    {
      "farmId": "FARM-KR-001",
      "farmName": "Seoul Cricket Farm",
      "location": {
        "country": "South Korea",
        "region": "Seoul"
      },
      "certifications": ["WIA-AGRI-025", "HACCP"],
      "species": ["Acheta domesticus", "Tenebrio molitor"]
    }
  ]
}
```

#### POST /farms

Register a new farm.

**Request Body:**
```json
{
  "farmName": "Seoul Cricket Farm",
  "location": {
    "country": "South Korea",
    "region": "Seoul",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "contactInfo": {
    "email": "info@seoulcricket.com",
    "phone": "+82-2-1234-5678"
  },
  "species": ["Acheta domesticus"]
}
```

---

## 4. GraphQL API

### 4.1 Endpoint

**URL:** `https://api.wia-agri.org/graphql`

### 4.2 Schema

```graphql
type Query {
  products(
    species: String
    form: ProductForm
    certified: Boolean
    page: Int
    limit: Int
  ): ProductConnection!

  product(id: ID!): Product

  nutrition(productId: ID!): NutritionProfile

  safety(productId: ID!): SafetyProfile

  sustainability(productId: ID!): SustainabilityMetrics

  certifications(
    status: CertificationStatus
    productId: ID
  ): [Certification!]!

  farms(country: String, certified: Boolean): [Farm!]!
}

type Mutation {
  createProduct(input: CreateProductInput!): Product!

  updateProduct(id: ID!, input: UpdateProductInput!): Product!

  deleteProduct(id: ID!): Boolean!

  createCertificationRequest(input: CertificationRequestInput!): CertificationRequest!

  reportSafetyAlert(input: SafetyAlertInput!): SafetyAlert!
}

type Product {
  id: ID!
  version: String!
  standard: String!
  species: Species!
  productForm: ProductForm!
  batchNumber: String!
  productionDate: DateTime!
  farmInfo: Farm!
  nutrition: NutritionProfile!
  safety: SafetyProfile!
  sustainability: SustainabilityMetrics!
  certification: Certification
  traceability: TraceabilityInfo!
}

type Species {
  commonName: String!
  scientificName: String!
  taxonomy: Taxonomy!
}

enum ProductForm {
  WHOLE
  POWDER
  PROTEIN_ISOLATE
  OIL
  PASTE
  FLOUR
}

type NutritionProfile {
  macronutrients: Macronutrients!
  aminoAcids: AminoAcids!
  vitamins: Vitamins!
  minerals: Minerals!
  energyContent: EnergyContent!
}

type Macronutrients {
  protein: NutrientValue!
  fat: FatProfile!
  carbohydrates: CarbohydrateProfile!
  moisture: NutrientValue!
  ash: NutrientValue!
}
```

### 4.3 Example Queries

**Get Product with Full Details:**
```graphql
query GetProduct($id: ID!) {
  product(id: $id) {
    id
    species {
      commonName
      scientificName
    }
    productForm
    nutrition {
      macronutrients {
        protein {
          value
          unit
        }
      }
    }
    certification {
      certificationId
      status
      issueDate
      expiryDate
    }
  }
}
```

**Create Product:**
```graphql
mutation CreateProduct($input: CreateProductInput!) {
  createProduct(input: $input) {
    id
    batchNumber
    productionDate
  }
}
```

**Variables:**
```json
{
  "input": {
    "species": {
      "commonName": "House Cricket",
      "scientificName": "Acheta domesticus"
    },
    "productForm": "POWDER",
    "batchNumber": "BATCH-2025-005"
  }
}
```

---

## 5. Webhooks

### 5.1 Webhook Events

Subscribe to events for real-time notifications.

**Available Events:**
- `product.created`
- `product.updated`
- `product.deleted`
- `certification.issued`
- `certification.expired`
- `batch.created`
- `batch.completed`
- `safety.alert`
- `sustainability.updated`

### 5.2 Webhook Payload

```json
{
  "event": "product.created",
  "timestamp": "2025-12-26T12:00:00Z",
  "data": {
    "productId": "PRODUCT-NEW-001",
    "batchNumber": "BATCH-2025-005",
    "species": "Acheta domesticus"
  },
  "signature": "sha256=..."
}
```

### 5.3 Webhook Configuration

**POST /webhooks**

```json
{
  "url": "https://your-domain.com/webhook",
  "events": ["product.created", "certification.issued"],
  "secret": "your-webhook-secret"
}
```

---

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "INVALID_REQUEST",
    "message": "The request is invalid",
    "details": [
      {
        "field": "productForm",
        "message": "Invalid product form. Must be one of: whole, powder, protein-isolate, oil"
      }
    ]
  }
}
```

### 6.2 Error Codes

- `INVALID_REQUEST`: Request validation failed
- `UNAUTHORIZED`: Invalid or missing API key
- `FORBIDDEN`: Insufficient permissions
- `NOT_FOUND`: Resource not found
- `CONFLICT`: Resource already exists
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `INTERNAL_ERROR`: Server error

---

## 7. Rate Limiting

### 7.1 Rate Limits

**Standard Tier:**
- 1,000 requests/hour
- 10,000 requests/day

**Premium Tier:**
- 10,000 requests/hour
- 100,000 requests/day

**Enterprise Tier:**
- Custom limits

### 7.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640530800
```

---

## 8. Pagination

### 8.1 Request Parameters

- `page`: Page number (default: 1)
- `limit`: Items per page (default: 20, max: 100)

### 8.2 Response Format

```json
{
  "success": true,
  "data": [ /* items */ ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 150,
    "totalPages": 8,
    "hasNext": true,
    "hasPrevious": false
  }
}
```

---

**Next Phase:** [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md)

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
