# WIA-HERITAGE-005: Phase 2 - API Interface Specification

> 弘益人間 (Benefit All Humanity)

## Overview

Phase 2 defines the RESTful API, GraphQL interface, and WebSocket protocols for accessing and managing cultural artifact data.

## 1. RESTful API

### 1.1 Base Configuration

```yaml
base_url: https://api.wia.org/heritage/v1
authentication: Bearer Token (JWT)
rate_limiting: 1000 requests/hour
content_type: application/json
encoding: UTF-8
```

### 1.2 Core Endpoints

#### Artifacts

```http
# List artifacts
GET /artifacts?page=1&limit=20&filter=period:Bronze+Age
Response: 200 OK

# Create artifact
POST /artifacts
Body: { "name": "...", "period": "...", ... }
Response: 201 Created

# Get artifact
GET /artifacts/{id}
Response: 200 OK

# Update artifact
PATCH /artifacts/{id}
Body: { "condition": "good", ... }
Response: 200 OK

# Delete artifact
DELETE /artifacts/{id}
Response: 204 No Content

# Search artifacts
POST /artifacts/search
Body: { "query": "pottery", "filters": {...} }
Response: 200 OK
```

#### 3D Models

```http
# Generate 3D model
POST /models/generate
Body: { "artifactId": "...", "resolution": "high" }
Response: 202 Accepted

# Get model
GET /models/{id}
Response: 200 OK

# Download model
GET /models/{id}/download?format=gltf
Response: 200 OK (binary)

# Get model status
GET /models/{id}/status
Response: 200 OK
```

#### Provenance

```http
# Record provenance
POST /provenance
Body: { "artifactId": "...", "owner": "...", ... }
Response: 201 Created

# Get provenance history
GET /provenance/{artifactId}
Response: 200 OK

# Verify provenance
POST /provenance/{id}/verify
Response: 200 OK
```

#### Virtual Exhibitions

```http
# Create exhibition
POST /exhibitions
Body: { "title": "...", "artifacts": [...], ... }
Response: 201 Created

# List exhibitions
GET /exhibitions?public=true
Response: 200 OK

# Get exhibition
GET /exhibitions/{id}
Response: 200 OK

# Update exhibition
PATCH /exhibitions/{id}
Response: 200 OK
```

### 1.3 Request/Response Examples

#### Create Artifact

```bash
curl -X POST https://api.wia.org/heritage/v1/artifacts \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Ancient Greek Amphora",
    "period": "Classical Period",
    "origin": {
      "country": "Greece",
      "region": "Attica"
    },
    "materials": [
      {"type": "terracotta", "percentage": 100}
    ],
    "dimensions": {
      "height": 425,
      "width": 280,
      "depth": 280
    }
  }'
```

Response:

```json
{
  "success": true,
  "data": {
    "id": "art-005",
    "name": "Ancient Greek Amphora",
    "created": "2025-01-15T14:30:00Z",
    "url": "/artifacts/art-005"
  },
  "timestamp": "2025-01-15T14:30:00Z"
}
```

## 2. GraphQL API

### 2.1 Schema Definition

```graphql
type Artifact {
  id: ID!
  name: String!
  description: String
  period: String!
  origin: Origin!
  materials: [Material!]!
  dimensions: Dimensions!
  dating: Dating
  condition: ConditionReport!
  models: [Model3D!]!
  provenance: [ProvenanceRecord!]!
  created: DateTime!
  updated: DateTime!
}

type Origin {
  country: String!
  region: String
  site: String
  coordinates: Coordinates
}

type Model3D {
  id: ID!
  format: String!
  url: String!
  quality: String!
  stats: ModelStats!
}

type Query {
  artifact(id: ID!): Artifact
  artifacts(
    page: Int = 1
    limit: Int = 20
    filter: ArtifactFilter
  ): ArtifactConnection!
  searchArtifacts(query: String!): [Artifact!]!

  exhibition(id: ID!): Exhibition
  exhibitions(public: Boolean): [Exhibition!]!
}

type Mutation {
  createArtifact(input: ArtifactInput!): Artifact!
  updateArtifact(id: ID!, input: ArtifactInput!): Artifact!
  deleteArtifact(id: ID!): Boolean!

  recordProvenance(input: ProvenanceInput!): ProvenanceRecord!
  createExhibition(input: ExhibitionInput!): Exhibition!
}

type Subscription {
  scanProgress(scanId: ID!): ScanStatus!
  modelGeneration(modelId: ID!): ModelStatus!
}
```

### 2.2 Query Examples

```graphql
# Get artifact with models
query GetArtifact($id: ID!) {
  artifact(id: $id) {
    id
    name
    period
    origin {
      country
      region
    }
    models {
      id
      format
      url
      quality
    }
    provenance {
      owner { name }
      location { institution }
      timestamp
    }
  }
}

# Search artifacts
query SearchArtifacts {
  searchArtifacts(query: "pottery") {
    id
    name
    period
    origin { country }
  }
}

# Create artifact with mutation
mutation CreateArtifact($input: ArtifactInput!) {
  createArtifact(input: $input) {
    id
    name
    created
  }
}
```

## 3. IIIF Image API

### 3.1 Image Request Pattern

```
{scheme}://{server}{/prefix}/{identifier}/{region}/{size}/{rotation}/{quality}.{format}

Example:
https://iiif.wia.org/heritage/art-005/full/1000,/0/default.jpg
```

### 3.2 Supported Parameters

```yaml
region:
  - full: Complete image
  - x,y,w,h: Rectangle (pixels)
  - pct:x,y,w,h: Percentage

size:
  - full: Full size
  - w,: Width specified
  - ,h: Height specified
  - pct:n: Percentage

rotation:
  - 0-360: Degrees

quality:
  - default: Platform default
  - color: 24-bit color
  - gray: Grayscale
  - bitonal: 1-bit

format:
  - jpg, png, tif, webp
```

### 3.3 Image Information (info.json)

```json
{
  "@context": "http://iiif.io/api/image/3/context.json",
  "id": "https://iiif.wia.org/heritage/art-005",
  "type": "ImageService3",
  "protocol": "http://iiif.io/api/image",
  "profile": "level2",
  "width": 8192,
  "height": 8192,
  "sizes": [
    {"width": 1024, "height": 1024},
    {"width": 2048, "height": 2048},
    {"width": 4096, "height": 4096}
  ],
  "tiles": [
    {"width": 512, "scaleFactors": [1, 2, 4, 8, 16]}
  ]
}
```

## 4. WebSocket Streaming

### 4.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia.org/heritage/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'subscribe',
    channel: 'scan_progress',
    scanId: 'scan-005',
    token: 'YOUR_API_KEY'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Progress:', data.progress);
};
```

### 4.2 Event Types

```json
{
  "scan_progress": {
    "scanId": "scan-005",
    "progress": 0.75,
    "stage": "texturing",
    "eta": 120
  },
  "model_ready": {
    "modelId": "model-005",
    "url": "https://...",
    "format": "gltf"
  },
  "provenance_verified": {
    "recordId": "prov-005",
    "valid": true,
    "txHash": "0x..."
  }
}
```

## 5. Authentication & Authorization

### 5.1 JWT Token Structure

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "user-123",
    "iss": "wia.org",
    "aud": "heritage-api",
    "exp": 1737043200,
    "iat": 1737039600,
    "scope": ["read:artifacts", "write:artifacts", "admin:all"]
  }
}
```

### 5.2 Scopes

```yaml
public:
  - read:artifacts (public artifacts)
  - read:exhibitions

authenticated:
  - read:artifacts:private
  - write:artifacts:own
  - create:models

curator:
  - write:exhibitions
  - manage:provenance

admin:
  - admin:all
  - delete:artifacts
  - manage:users
```

## 6. Rate Limiting

### 6.1 Limits

```yaml
anonymous: 100 requests/hour
authenticated: 1000 requests/hour
curator: 5000 requests/hour
admin: unlimited
```

### 6.2 Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1737043200
```

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "ARTIFACT_NOT_FOUND",
    "message": "Artifact with ID 'art-999' not found",
    "details": {
      "artifactId": "art-999"
    },
    "timestamp": "2025-01-15T14:30:00Z"
  }
}
```

### 7.2 Status Codes

```yaml
200: Success
201: Created
202: Accepted (async operation)
204: No Content
400: Bad Request
401: Unauthorized
403: Forbidden
404: Not Found
409: Conflict
429: Too Many Requests
500: Internal Server Error
503: Service Unavailable
```

## 8. Pagination

```json
{
  "data": [...],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 1523,
    "pages": 77,
    "hasNext": true,
    "hasPrev": false
  },
  "links": {
    "self": "/artifacts?page=1",
    "next": "/artifacts?page=2",
    "last": "/artifacts?page=77"
  }
}
```

## 9. Webhook Integration

### 9.1 Configuration

```http
POST /webhooks
Body: {
  "url": "https://your-server.com/wia-webhook",
  "events": ["artifact.created", "model.ready"],
  "secret": "your-webhook-secret"
}
```

### 9.2 Payload

```json
{
  "event": "model.ready",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "modelId": "model-005",
    "artifactId": "art-005",
    "format": "gltf",
    "url": "https://..."
  },
  "signature": "sha256=..."
}
```

---

**Phase 2 Compliance**: All implementations must support RESTful API with JWT authentication, IIIF Image API, and proper error handling.

**Next**: [Phase 3 - Protocol](PHASE-3-PROTOCOL.md)

---

弘益人間 (Benefit All Humanity)
© 2025 SmileStory Inc. / WIA
