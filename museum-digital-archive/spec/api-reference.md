# WIA-EDU-024: Museum Digital Archive Standard - API Reference

> **Philosophy:** 弘益人間 (Benefit All Humanity)

## Base URL

```
https://api.museum.org/v1
```

## Authentication

### API Key (Recommended for public access)

```http
GET /objects
X-API-Key: your-api-key-here
```

### OAuth 2.0 (For privileged operations)

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

Usage:
```http
GET /objects
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

## Rate Limiting

| Tier | Requests/Minute | Daily Limit |
|------|----------------|-------------|
| Anonymous | 10 | 1,000 |
| Free (API Key) | 100 | 10,000 |
| Standard | 1,000 | 100,000 |
| Premium | 10,000 | 1,000,000 |

Rate limit headers:
```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1640000000
```

## Common Headers

### Request Headers
```http
Accept: application/json
Content-Type: application/json
X-API-Key: your-api-key
User-Agent: MyApp/1.0
Accept-Language: en-US,en;q=0.9
```

### Response Headers
```http
Content-Type: application/json; charset=utf-8
X-Request-ID: 550e8400-e29b-41d4-a716-446655440000
Cache-Control: public, max-age=3600
ETag: "33a64df551425fcc55e4d42a148795d9f25f89d4"
Link: <https://api.museum.org/v1/objects?page=2>; rel="next"
```

## REST API Endpoints

### 1. Objects

#### List Objects

```http
GET /objects
```

Query parameters:
- `page` (integer): Page number (default: 1)
- `pageSize` (integer): Items per page (default: 20, max: 100)
- `department` (string): Filter by department
- `medium` (string): Filter by medium
- `hasImage` (boolean): Only objects with images
- `onView` (boolean): Only objects currently on view
- `q` (string): Search query
- `sort` (string): Sort field (name, date, accession)
- `order` (string): Sort order (asc, desc)

Response:
```json
{
  "data": [
    {
      "id": "2025.123.45",
      "name": "The Starry Night",
      "creator": "Vincent van Gogh",
      "date": "1889-06",
      "department": "European Paintings",
      "image": "https://iiif.museum.org/starry-night/full/400,/0/default.jpg",
      "url": "https://api.museum.org/v1/objects/2025.123.45"
    }
  ],
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalPages": 50,
    "totalResults": 1000
  },
  "links": {
    "self": "https://api.museum.org/v1/objects?page=1",
    "next": "https://api.museum.org/v1/objects?page=2",
    "last": "https://api.museum.org/v1/objects?page=50"
  }
}
```

#### Get Object Details

```http
GET /objects/{id}
```

Path parameters:
- `id` (string): Object identifier (accession number)

Query parameters:
- `include` (string): Additional fields (provenance,exhibitions,conservation)

Response:
```json
{
  "@context": "https://schema.org",
  "@type": "VisualArtwork",
  "wiaStandard": "WIA-EDU-024",
  "identifier": "2025.123.45",
  "name": "The Starry Night",
  "creator": {
    "@type": "Person",
    "name": "Vincent van Gogh",
    "birthDate": "1853",
    "deathDate": "1890"
  },
  "dateCreated": "1889-06",
  "artMedium": "Oil on canvas",
  "width": { "value": 73.7, "unitCode": "CMT" },
  "height": { "value": 92.1, "unitCode": "CMT" },
  "description": "Post-impressionist masterpiece...",
  "image": {
    "contentUrl": "https://iiif.museum.org/starry-night/full/max/0/default.jpg",
    "width": 15000,
    "height": 12000
  },
  "iiifManifest": "https://iiif.museum.org/starry-night/manifest.json",
  "department": "European Paintings",
  "classification": "Paintings",
  "period": "Modern Art",
  "license": "https://creativecommons.org/publicdomain/mark/1.0/",
  "keywords": ["Post-Impressionism", "landscape", "night sky"]
}
```

#### Search Objects

```http
POST /objects/search
```

Request body:
```json
{
  "query": "impressionist landscape",
  "filters": {
    "department": ["European Paintings"],
    "medium": ["painting"],
    "dateRange": {
      "start": "1860",
      "end": "1920"
    },
    "hasImage": true
  },
  "facets": ["department", "creator", "medium"],
  "sort": {
    "field": "relevance",
    "order": "desc"
  },
  "page": 1,
  "pageSize": 20
}
```

Response:
```json
{
  "query": {
    "text": "impressionist landscape",
    "processedQuery": "impressionism AND landscape"
  },
  "results": [
    {
      "id": "2025.123.45",
      "name": "The Starry Night",
      "creator": "Vincent van Gogh",
      "relevanceScore": 0.95,
      "highlights": {
        "description": "...Post-<em>impressionist</em> <em>landscape</em>..."
      }
    }
  ],
  "facets": {
    "department": {
      "European Paintings": 127,
      "Modern Art": 89
    },
    "creator": {
      "Claude Monet": 45,
      "Vincent van Gogh": 23
    }
  },
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalResults": 234
  }
}
```

### 2. IIIF Integration

#### Get IIIF Manifest

```http
GET /iiif/{id}/manifest.json
```

Response: IIIF Presentation API 3.0 manifest (see Technical Specification)

#### IIIF Image API

```http
GET /iiif/{id}/{region}/{size}/{rotation}/{quality}.{format}
```

Examples:
```
# Full image, 400px wide
/iiif/starry-night/full/400,/0/default.jpg

# Specific region, full size
/iiif/starry-night/1000,1000,2000,2000/full/0/default.jpg

# Full image, maximum size, PNG
/iiif/starry-night/full/max/0/default.png
```

### 3. Exhibitions

#### List Exhibitions

```http
GET /exhibitions
```

Query parameters:
- `status` (string): current, upcoming, past
- `type` (string): permanent, temporary, virtual
- `page` (integer)
- `pageSize` (integer)

Response:
```json
{
  "data": [
    {
      "id": "EXH-2025-001",
      "name": "Impressionism: Dawn of Modern Art",
      "curator": "Dr. Sarah Johnson",
      "startDate": "2025-03-01",
      "endDate": "2025-08-31",
      "status": "upcoming",
      "type": "temporary",
      "numberOfObjects": 75,
      "image": "https://museum.org/exhibitions/impressionism-2025/banner.jpg",
      "url": "https://api.museum.org/v1/exhibitions/EXH-2025-001"
    }
  ],
  "pagination": { ... }
}
```

#### Get Exhibition Details

```http
GET /exhibitions/{id}
```

Response:
```json
{
  "@type": "ExhibitionEvent",
  "wiaStandard": "WIA-EDU-024",
  "identifier": "EXH-2025-001",
  "name": "Impressionism: Dawn of Modern Art",
  "description": "Explore the revolutionary movement...",
  "curator": {
    "@type": "Person",
    "name": "Dr. Sarah Johnson"
  },
  "startDate": "2025-03-01",
  "endDate": "2025-08-31",
  "location": { ... },
  "virtualLocation": {
    "url": "https://museum.org/exhibitions/impressionism-2025"
  },
  "workFeatured": [
    { "@id": "https://api.museum.org/v1/objects/2025.123.45" }
  ],
  "numberOfObjects": 75,
  "sections": [
    {
      "name": "Early Impressionism",
      "description": "The beginnings...",
      "objects": ["2025.123.45", "2025.123.46"]
    }
  ]
}
```

#### Get Exhibition Objects

```http
GET /exhibitions/{id}/objects
```

Returns paginated list of objects in the exhibition

### 4. Departments

#### List Departments

```http
GET /departments
```

Response:
```json
{
  "data": [
    {
      "id": "european-paintings",
      "name": "European Paintings",
      "description": "European paintings from the 13th to early 20th century",
      "objectCount": 1234,
      "url": "https://api.museum.org/v1/departments/european-paintings"
    }
  ]
}
```

#### Get Department Objects

```http
GET /departments/{id}/objects
```

Returns paginated list of objects in the department

### 5. Creators/Artists

#### List Creators

```http
GET /creators
```

Query parameters:
- `q` (string): Search query
- `nationality` (string)
- `birthYear` (integer)
- `page`, `pageSize`

Response:
```json
{
  "data": [
    {
      "id": "vincent-van-gogh",
      "name": "Vincent van Gogh",
      "birthDate": "1853",
      "deathDate": "1890",
      "nationality": "Dutch",
      "objectCount": 23,
      "url": "https://api.museum.org/v1/creators/vincent-van-gogh"
    }
  ]
}
```

#### Get Creator Details

```http
GET /creators/{id}
```

Returns creator information and list of works

### 6. Educational Resources

#### List Educational Resources

```http
GET /education
```

Query parameters:
- `type` (string): lesson-plan, activity, video, quiz
- `gradeLevel` (string): K-2, 3-5, 6-8, 9-12, college
- `subject` (string): art, history, science

Response:
```json
{
  "data": [
    {
      "id": "lesson-001",
      "type": "lesson-plan",
      "title": "Exploring Impressionism",
      "description": "A comprehensive lesson plan...",
      "gradeLevel": "9-12",
      "subject": "art",
      "duration": "45 minutes",
      "objectives": ["Understand the key...", "Analyze..."],
      "materials": ["Projector", "Handouts"],
      "url": "https://api.museum.org/v1/education/lesson-001"
    }
  ]
}
```

### 7. Collections

#### Get Random Objects

```http
GET /objects/random
```

Query parameters:
- `count` (integer): Number of random objects (default: 1, max: 20)
- `department` (string): Filter by department
- `hasImage` (boolean)

Response:
```json
{
  "data": [
    { /* object */ }
  ]
}
```

#### Get Related Objects

```http
GET /objects/{id}/related
```

Returns objects related by artist, period, medium, or keywords

### 8. Statistics

#### Get Collection Statistics

```http
GET /stats
```

Response:
```json
{
  "totalObjects": 50000,
  "objectsWithImages": 35000,
  "departments": 12,
  "creators": 5000,
  "exhibitions": 150,
  "dateRange": {
    "earliest": "-3000",
    "latest": "2025"
  },
  "topDepartments": [
    { "name": "European Paintings", "count": 12000 }
  ],
  "recentlyAdded": 150,
  "lastUpdated": "2025-12-26T10:30:00Z"
}
```

## GraphQL API

### Endpoint

```
POST /graphql
```

### Sample Query

```graphql
query GetObject($id: ID!) {
  object(id: $id) {
    id
    name
    creator {
      name
      nationality
      birthDate
    }
    dateCreated
    artMedium
    department {
      name
    }
    image {
      url(size: MEDIUM)
      width
      height
    }
    exhibitions {
      name
      startDate
    }
  }
}
```

Variables:
```json
{
  "id": "2025.123.45"
}
```

### Sample Mutation (Authenticated)

```graphql
mutation CreateAnnotation($input: AnnotationInput!) {
  createAnnotation(input: $input) {
    id
    text
    createdAt
    user {
      name
    }
  }
}
```

## SPARQL Endpoint

### Endpoint

```
POST /sparql
Content-Type: application/sparql-query
```

### Sample Query

```sparql
PREFIX schema: <https://schema.org/>
PREFIX crm: <http://www.cidoc-crm.org/cidoc-crm/>
PREFIX wia: <https://wiastandards.org/edu-024/>

SELECT ?object ?title ?creator ?date
WHERE {
  ?object a schema:VisualArtwork ;
          schema:name ?title ;
          schema:creator ?creatorNode ;
          schema:dateCreated ?date .
  ?creatorNode schema:name ?creator .
  FILTER(CONTAINS(LCASE(?title), "starry"))
}
LIMIT 10
```

## Webhooks

### Register Webhook

```http
POST /webhooks
```

Request:
```json
{
  "url": "https://yourapp.com/webhook",
  "events": ["object.created", "object.updated", "exhibition.published"],
  "secret": "your-webhook-secret"
}
```

### Webhook Events

- `object.created` - New object added
- `object.updated` - Object metadata updated
- `object.deleted` - Object removed
- `exhibition.created` - New exhibition created
- `exhibition.published` - Exhibition made public
- `image.uploaded` - New image added

### Webhook Payload

```json
{
  "event": "object.created",
  "timestamp": "2025-12-26T10:30:00Z",
  "data": {
    "id": "2025.123.45",
    "name": "The Starry Night",
    "url": "https://api.museum.org/v1/objects/2025.123.45"
  },
  "signature": "sha256=..."
}
```

## Error Responses

### Error Format

```json
{
  "error": {
    "code": "NOT_FOUND",
    "message": "Object with ID '2025.123.99' not found",
    "details": {
      "id": "2025.123.99",
      "resource": "object"
    },
    "requestId": "550e8400-e29b-41d4-a716-446655440000"
  }
}
```

### HTTP Status Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 201 | Created |
| 204 | No Content |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 429 | Too Many Requests |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

## SDK Examples

### TypeScript

```typescript
import { MuseumArchiveClient } from '@wia/museum-digital-archive-sdk';

const client = new MuseumArchiveClient({
  baseURL: 'https://api.museum.org/v1',
  apiKey: 'your-api-key'
});

// Get object
const object = await client.objects.get('2025.123.45');

// Search
const results = await client.objects.search({
  query: 'impressionist landscape',
  filters: { department: 'European Paintings' }
});

// Get IIIF manifest
const manifest = await client.iiif.getManifest('2025.123.45');
```

### Python

```python
from wia_museum_archive import MuseumArchiveClient

client = MuseumArchiveClient(
    base_url='https://api.museum.org/v1',
    api_key='your-api-key'
)

# Get object
object = client.objects.get('2025.123.45')

# Search
results = client.objects.search(
    query='impressionist landscape',
    filters={'department': 'European Paintings'}
)

# List exhibitions
exhibitions = client.exhibitions.list(status='current')
```

---

**Philosophy:** 弘益人間 (Benefit All Humanity)

*WIA - World Certification Industry Association*

© 2025 MIT License
