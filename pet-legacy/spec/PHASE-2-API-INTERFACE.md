# WIA PET-LEGACY PHASE 2: API INTERFACE SPECIFICATION

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication & Authorization](#authentication--authorization)
4. [Memorial Profile Endpoints](#memorial-profile-endpoints)
5. [Media Management Endpoints](#media-management-endpoints)
6. [Timeline Event Endpoints](#timeline-event-endpoints)
7. [Family Member Endpoints](#family-member-endpoints)
8. [Search & Discovery Endpoints](#search--discovery-endpoints)
9. [AI & Analytics Endpoints](#ai--analytics-endpoints)
10. [Webhook Integration](#webhook-integration)
11. [Error Handling](#error-handling)
12. [Rate Limiting & Quotas](#rate-limiting--quotas)

---

## 1. Overview

### 1.1 Purpose

The WIA PET-LEGACY Phase 2 specification defines comprehensive REST API interfaces for managing digital pet memorials. These APIs enable seamless integration between memorial platforms, veterinary systems, pet cemeteries, and third-party applications while maintaining security, privacy, and data integrity.

### 1.2 API Design Principles

| Principle | Description | Implementation |
|-----------|-------------|----------------|
| **RESTful** | Follow REST architectural constraints | Resource-based URLs, HTTP verbs |
| **Versioned** | Support multiple API versions | URL path versioning (/v1/, /v2/) |
| **Stateless** | Each request contains all necessary information | JWT tokens, no server-side sessions |
| **Cacheable** | Enable efficient caching strategies | ETags, Cache-Control headers |
| **Secure** | Protect sensitive memorial data | OAuth 2.0, HTTPS only, encryption |
| **Documented** | Comprehensive OpenAPI documentation | Swagger/OpenAPI 3.0 specification |

### 1.3 Base URL Structure

```
Production:  https://api.petlegacy.wia.org/v1
Staging:     https://api-staging.petlegacy.wia.org/v1
Development: https://api-dev.petlegacy.wia.org/v1
```

### 1.4 Supported Formats

| Format | Content-Type | Usage |
|--------|--------------|-------|
| JSON | application/json | Primary request/response format |
| JSON-LD | application/ld+json | Linked data for semantic web |
| XML | application/xml | Legacy system compatibility |
| MessagePack | application/msgpack | High-performance binary format |

---

## 2. API Architecture

### 2.1 System Architecture

```
┌─────────────────┐
│  Client Apps   │
│  Web, Mobile   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────┐
│     API Gateway (Kong/AWS)          │
│  - Rate limiting                     │
│  - Authentication                    │
│  - Request routing                   │
└────────┬────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────┐
│     Load Balancer                    │
└────────┬────────────────────────────┘
         │
    ┌────┴────┐
    ▼         ▼
┌────────┐ ┌────────┐
│  API   │ │  API   │
│ Server │ │ Server │
│  (N)   │ │  (N)   │
└───┬────┘ └───┬────┘
    │          │
    └────┬─────┘
         ▼
┌─────────────────────────────────────┐
│         Microservices                │
│  ┌──────────┐  ┌─────────────┐      │
│  │ Memorial │  │   Media     │      │
│  │ Service  │  │   Service   │      │
│  └──────────┘  └─────────────┘      │
│  ┌──────────┐  ┌─────────────┐      │
│  │ Timeline │  │    User     │      │
│  │ Service  │  │   Service   │      │
│  └──────────┘  └─────────────┘      │
│  ┌──────────┐  ┌─────────────┐      │
│  │    AI    │  │  Analytics  │      │
│  │ Service  │  │   Service   │      │
│  └──────────┘  └─────────────┘      │
└─────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────┐
│       Data Layer                     │
│  ┌──────────┐  ┌─────────────┐      │
│  │PostgreSQL│  │   S3/CDN    │      │
│  │(Metadata)│  │   (Media)   │      │
│  └──────────┘  └─────────────┘      │
│  ┌──────────┐  ┌─────────────┐      │
│  │  Redis   │  │ Elasticsearch│      │
│  │ (Cache)  │  │  (Search)   │      │
│  └──────────┘  └─────────────┘      │
└─────────────────────────────────────┘
```

### 2.2 API Versioning Strategy

| Strategy | Format | Example | Status |
|----------|--------|---------|--------|
| URL Path | /v{version}/ | /v1/memorials | Current |
| Header | API-Version: 1.0 | API-Version: 2.0 | Supported |
| Query Parameter | ?version=1.0 | /memorials?version=1.0 | Deprecated |

### 2.3 Response Envelope

All API responses follow a consistent envelope structure:

```json
{
  "success": true,
  "data": {
    // Response payload
  },
  "metadata": {
    "requestId": "req_7x9kp3m2n5",
    "timestamp": "2024-12-18T14:22:00Z",
    "version": "1.0.0",
    "processingTime": 234
  },
  "pagination": {
    "page": 1,
    "perPage": 20,
    "total": 150,
    "totalPages": 8,
    "hasNext": true,
    "hasPrevious": false
  },
  "links": {
    "self": "/v1/memorials?page=1",
    "next": "/v1/memorials?page=2",
    "previous": null,
    "first": "/v1/memorials?page=1",
    "last": "/v1/memorials?page=8"
  }
}
```

---

## 3. Authentication & Authorization

### 3.1 OAuth 2.0 Flow

```
┌────────┐                                   ┌─────────────┐
│ Client │                                   │ Auth Server │
└───┬────┘                                   └──────┬──────┘
    │                                               │
    │  1. Authorization Request                     │
    │──────────────────────────────────────────────>│
    │                                               │
    │  2. User Login & Consent                      │
    │<──────────────────────────────────────────────│
    │                                               │
    │  3. Authorization Code                        │
    │<──────────────────────────────────────────────│
    │                                               │
    │  4. Exchange Code for Token                   │
    │──────────────────────────────────────────────>│
    │                                               │
    │  5. Access Token + Refresh Token              │
    │<──────────────────────────────────────────────│
    │                                               │
    │  6. API Request with Access Token             │
    │──────────────────────────────>                │
    │                               │                │
    │  7. Protected Resource        │                │
    │<──────────────────────────────│                │
```

### 3.2 Authentication Endpoints

#### 3.2.1 Obtain Access Token

```http
POST /v1/auth/token
Content-Type: application/json

{
  "grant_type": "authorization_code",
  "code": "AUTH_CODE_HERE",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "redirect_uri": "https://yourapp.com/callback"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def502004f8c7e8e3b2d1a9c...",
  "scope": "memorial:read memorial:write media:upload"
}
```

#### 3.2.2 Refresh Access Token

```http
POST /v1/auth/token
Content-Type: application/json

{
  "grant_type": "refresh_token",
  "refresh_token": "def502004f8c7e8e3b2d1a9c...",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret"
}
```

### 3.3 Authorization Scopes

| Scope | Description | Access Level |
|-------|-------------|--------------|
| `memorial:read` | View memorial profiles | Read-only |
| `memorial:write` | Create and update memorials | Read-Write |
| `memorial:delete` | Delete memorial profiles | Full |
| `media:read` | View media assets | Read-only |
| `media:upload` | Upload photos/videos | Write |
| `media:delete` | Delete media assets | Full |
| `timeline:read` | View timeline events | Read-only |
| `timeline:write` | Create/update events | Read-Write |
| `family:read` | View family members | Read-only |
| `family:manage` | Manage member access | Admin |
| `admin:all` | Full administrative access | Super Admin |

### 3.4 JWT Token Structure

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "key-id-2024"
  },
  "payload": {
    "iss": "https://auth.petlegacy.wia.org",
    "sub": "660e8400-e29b-41d4-a716-446655440001",
    "aud": "https://api.petlegacy.wia.org",
    "exp": 1703347200,
    "iat": 1703343600,
    "scope": "memorial:read memorial:write media:upload",
    "user": {
      "id": "660e8400-e29b-41d4-a716-446655440001",
      "email": "sarah.j@example.com",
      "name": "Sarah Johnson",
      "role": "guardian"
    }
  }
}
```

### 3.5 API Key Authentication (Legacy)

For server-to-server communication:

```http
GET /v1/memorials
X-API-Key: EXAMPLE_API_KEY_REPLACE_ME
```

---

## 4. Memorial Profile Endpoints

### 4.1 Create Memorial Profile

```http
POST /v1/memorials
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "petIdentity": {
    "name": "Max",
    "species": "dog",
    "breed": "Golden Retriever",
    "gender": "male",
    "birthDate": "2012-05-20",
    "passingDate": "2024-03-10"
  },
  "guardianInfo": {
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "name": "Sarah Johnson",
    "role": "owner"
  },
  "memorialStatus": {
    "status": "draft",
    "isPublic": false
  }
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "profileId": "550e8400-e29b-41d4-a716-446655440000",
    "petIdentity": {
      "name": "Max",
      "species": "dog",
      "breed": "Golden Retriever",
      "gender": "male",
      "birthDate": "2012-05-20",
      "passingDate": "2024-03-10"
    },
    "createdAt": "2024-12-18T14:22:00Z",
    "memorialUrl": "https://petlegacy.wia.org/memorial/550e8400"
  },
  "metadata": {
    "requestId": "req_7x9kp3m2n5",
    "timestamp": "2024-12-18T14:22:00Z"
  }
}
```

### 4.2 Get Memorial Profile

```http
GET /v1/memorials/{memorialId}
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `include` | string | Comma-separated relations (timeline,media,family) | null |
| `fields` | string | Specific fields to return | all |

**Example:**
```http
GET /v1/memorials/550e8400-e29b-41d4-a716-446655440000?include=timeline,media
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "profileId": "550e8400-e29b-41d4-a716-446655440000",
    "version": "1.0.0",
    "petIdentity": {
      "name": "Max",
      "nickname": ["Maxie", "Maxy Boy"],
      "species": "dog",
      "breed": "Golden Retriever",
      "gender": "male",
      "birthDate": "2012-05-20",
      "passingDate": "2024-03-10",
      "ageAtPassing": {
        "years": 11,
        "months": 9,
        "days": 20
      }
    },
    "memorialStatus": {
      "status": "active",
      "isPublic": true,
      "viewCount": 1247,
      "tributeCount": 89
    },
    "timeline": {
      "totalEvents": 52,
      "recentEvents": [...],
      "link": "/v1/memorials/550e8400/timeline"
    },
    "media": {
      "totalAssets": 418,
      "photos": 324,
      "videos": 47,
      "link": "/v1/memorials/550e8400/media"
    }
  }
}
```

### 4.3 Update Memorial Profile

```http
PATCH /v1/memorials/{memorialId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "memorialStatus": {
    "status": "active",
    "isPublic": true
  },
  "memorialCustomization": {
    "epitaph": "Forever in our hearts. The best friend anyone could ask for.",
    "themeColor": "#F59E0B"
  }
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "profileId": "550e8400-e29b-41d4-a716-446655440000",
    "lastModified": "2024-12-18T14:30:00Z",
    "changes": ["memorialStatus", "memorialCustomization"]
  }
}
```

### 4.4 Delete Memorial Profile

```http
DELETE /v1/memorials/{memorialId}
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `confirm` | boolean | Confirmation of deletion | Yes |
| `deleteMedia` | boolean | Also delete all media assets | No |

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "message": "Memorial profile scheduled for deletion",
    "profileId": "550e8400-e29b-41d4-a716-446655440000",
    "recoveryDeadline": "2025-01-17T14:30:00Z",
    "recoveryUrl": "/v1/memorials/550e8400/recover"
  }
}
```

### 4.5 List Memorial Profiles

```http
GET /v1/memorials
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `page` | integer | Page number | 1 |
| `perPage` | integer | Items per page (max 100) | 20 |
| `status` | string | Filter by status | all |
| `species` | string | Filter by species | all |
| `sort` | string | Sort field (createdAt, name, passingDate) | createdAt |
| `order` | string | Sort order (asc, desc) | desc |

**Response (200 OK):**
```json
{
  "success": true,
  "data": [
    {
      "profileId": "550e8400-e29b-41d4-a716-446655440000",
      "petIdentity": {
        "name": "Max",
        "species": "dog",
        "breed": "Golden Retriever"
      },
      "passingDate": "2024-03-10",
      "memorialStatus": {
        "status": "active",
        "isPublic": true
      }
    }
  ],
  "pagination": {
    "page": 1,
    "perPage": 20,
    "total": 3,
    "totalPages": 1
  }
}
```

### 4.6 Export Memorial Data

```http
POST /v1/memorials/{memorialId}/export
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "format": "json",
  "includeMedia": true,
  "compression": "zip",
  "password": "optional_password"
}
```

**Response (202 Accepted):**
```json
{
  "success": true,
  "data": {
    "exportId": "exp_9k2m5n7p8q",
    "status": "processing",
    "estimatedSize": "2.3 GB",
    "estimatedTime": "15 minutes",
    "statusUrl": "/v1/exports/exp_9k2m5n7p8q",
    "webhookUrl": "configured_webhook_url"
  }
}
```

---

## 5. Media Management Endpoints

### 5.1 Upload Media Asset

```http
POST /v1/memorials/{memorialId}/media
Authorization: Bearer {access_token}
Content-Type: multipart/form-data

--boundary
Content-Disposition: form-data; name="file"; filename="max-photo.jpg"
Content-Type: image/jpeg

[binary data]
--boundary
Content-Disposition: form-data; name="metadata"

{
  "title": "Max at the Beach",
  "description": "First time seeing the ocean",
  "capturedAt": "2015-07-20T14:30:00Z",
  "tags": ["beach", "summer", "happy"],
  "privacyLevel": "public"
}
--boundary--
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "type": "photo",
    "url": "https://cdn.petlegacy.wia.org/assets/max-beach.jpg",
    "thumbnailUrl": "https://cdn.petlegacy.wia.org/thumbs/max-beach-thumb.jpg",
    "processingStatus": "processing",
    "uploadedAt": "2024-12-18T14:35:00Z"
  }
}
```

### 5.2 Get Media Asset

```http
GET /v1/memorials/{memorialId}/media/{assetId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "type": "photo",
    "url": "https://cdn.petlegacy.wia.org/assets/max-beach.jpg",
    "thumbnailUrl": "https://cdn.petlegacy.wia.org/thumbs/max-beach-thumb.jpg",
    "title": "Max at the Beach",
    "description": "First time seeing the ocean",
    "capturedAt": "2015-07-20T14:30:00Z",
    "uploadedAt": "2024-12-18T14:35:00Z",
    "fileSize": 3457280,
    "mediaMetadata": {
      "width": 4032,
      "height": 3024,
      "orientation": 1
    },
    "processingStatus": "ready"
  }
}
```

### 5.3 List Media Assets

```http
GET /v1/memorials/{memorialId}/media
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `type` | string | Filter by type (photo, video, audio) | all |
| `page` | integer | Page number | 1 |
| `perPage` | integer | Items per page | 50 |
| `sort` | string | Sort by (capturedAt, uploadedAt, fileSize) | capturedAt |
| `order` | string | Sort order (asc, desc) | desc |
| `tags` | string | Filter by tags (comma-separated) | null |

### 5.4 Update Media Metadata

```http
PATCH /v1/memorials/{memorialId}/media/{assetId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "title": "Max's First Beach Adventure",
  "description": "Updated description with more details",
  "tags": ["beach", "summer", "happy", "california"],
  "privacyLevel": "public",
  "isFeatured": true
}
```

### 5.5 Delete Media Asset

```http
DELETE /v1/memorials/{memorialId}/media/{assetId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "message": "Media asset deleted successfully",
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "recoveryDeadline": "2025-01-17T14:40:00Z"
  }
}
```

### 5.6 Bulk Upload Media

```http
POST /v1/memorials/{memorialId}/media/bulk
Authorization: Bearer {access_token}
Content-Type: multipart/form-data

{
  "files": [
    {"file": "photo1.jpg", "capturedAt": "2020-01-15"},
    {"file": "photo2.jpg", "capturedAt": "2020-01-16"},
    {"file": "video1.mp4", "capturedAt": "2020-01-17"}
  ],
  "defaultPrivacy": "family_only",
  "defaultTags": ["vacation", "2020"]
}
```

**Response (202 Accepted):**
```json
{
  "success": true,
  "data": {
    "batchId": "batch_5x7k9m2n4p",
    "totalFiles": 3,
    "status": "processing",
    "statusUrl": "/v1/media/batches/batch_5x7k9m2n4p"
  }
}
```

---

## 6. Timeline Event Endpoints

### 6.1 Create Timeline Event

```http
POST /v1/memorials/{memorialId}/timeline
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "eventType": "celebration",
  "date": "2020-05-20T15:00:00Z",
  "title": "Max's 8th Birthday",
  "description": "Celebrated with a dog-friendly cake and lots of treats!",
  "location": {
    "placeName": "Our Backyard",
    "city": "Portland",
    "state": "OR"
  },
  "attachedMedia": [
    "770e8400-e29b-41d4-a716-446655440002",
    "770e8400-e29b-41d4-a716-446655440005"
  ],
  "celebrationInfo": {
    "occasionType": "birthday",
    "gifts": ["new toys", "treats"]
  }
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "eventId": "880e8400-e29b-41d4-a716-446655440003",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "eventType": "celebration",
    "date": "2020-05-20T15:00:00Z",
    "title": "Max's 8th Birthday",
    "createdAt": "2024-12-18T14:50:00Z"
  }
}
```

### 6.2 Get Timeline Event

```http
GET /v1/memorials/{memorialId}/timeline/{eventId}
Authorization: Bearer {access_token}
```

### 6.3 List Timeline Events

```http
GET /v1/memorials/{memorialId}/timeline
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `eventType` | string | Filter by event type | all |
| `startDate` | date | Events after this date | null |
| `endDate` | date | Events before this date | null |
| `sort` | string | Sort by date | desc |

**Response (200 OK):**
```json
{
  "success": true,
  "data": [
    {
      "eventId": "880e8400-e29b-41d4-a716-446655440003",
      "eventType": "celebration",
      "date": "2020-05-20T15:00:00Z",
      "title": "Max's 8th Birthday",
      "attachedMedia": 2,
      "createdBy": "Sarah Johnson"
    }
  ],
  "pagination": {
    "total": 52
  }
}
```

### 6.4 Update Timeline Event

```http
PATCH /v1/memorials/{memorialId}/timeline/{eventId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "description": "Updated description with more memories",
  "attachedMedia": [
    "770e8400-e29b-41d4-a716-446655440002",
    "770e8400-e29b-41d4-a716-446655440005",
    "770e8400-e29b-41d4-a716-446655440007"
  ]
}
```

### 6.5 Delete Timeline Event

```http
DELETE /v1/memorials/{memorialId}/timeline/{eventId}
Authorization: Bearer {access_token}
```

---

## 7. Family Member Endpoints

### 7.1 Invite Family Member

```http
POST /v1/memorials/{memorialId}/family/invite
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "email": "michael.j@example.com",
  "role": "contributor",
  "relationshipToPet": "Co-owner",
  "personalMessage": "Hi Mike, I'd like you to help build Max's memorial.",
  "permissions": {
    "canView": true,
    "canComment": true,
    "canUpload": true,
    "canEdit": true
  }
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "invitationId": "inv_2k5m7n9p3q",
    "email": "michael.j@example.com",
    "role": "contributor",
    "status": "pending",
    "invitedAt": "2024-12-18T15:00:00Z",
    "expiresAt": "2025-01-17T15:00:00Z",
    "invitationUrl": "https://petlegacy.wia.org/invite/inv_2k5m7n9p3q"
  }
}
```

### 7.2 List Family Members

```http
GET /v1/memorials/{memorialId}/family
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": [
    {
      "memberId": "990e8400-e29b-41d4-a716-446655440008",
      "userId": "660e8400-e29b-41d4-a716-446655440001",
      "displayName": "Sarah Johnson",
      "role": "guardian",
      "status": "active",
      "contributionStats": {
        "photosUploaded": 198,
        "storiesWritten": 8,
        "lastContribution": "2024-12-18T14:22:00Z"
      }
    },
    {
      "memberId": "990e8400-e29b-41d4-a716-446655440009",
      "userId": "660e8400-e29b-41d4-a716-446655440010",
      "displayName": "Michael Johnson",
      "role": "contributor",
      "status": "active",
      "contributionStats": {
        "photosUploaded": 126,
        "storiesWritten": 7
      }
    }
  ]
}
```

### 7.3 Update Family Member Permissions

```http
PATCH /v1/memorials/{memorialId}/family/{memberId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "role": "contributor",
  "permissions": {
    "canUpload": true,
    "canEdit": false,
    "canDelete": false
  }
}
```

### 7.4 Remove Family Member

```http
DELETE /v1/memorials/{memorialId}/family/{memberId}
Authorization: Bearer {access_token}
```

---

## 8. Search & Discovery Endpoints

### 8.1 Search Memorials

```http
GET /v1/search/memorials
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `q` | string | Search query |
| `species` | string | Filter by species |
| `breed` | string | Filter by breed |
| `location` | string | Geographic location |
| `yearPassed` | integer | Year of passing |
| `isPublic` | boolean | Public memorials only |

**Response (200 OK):**
```json
{
  "success": true,
  "data": [
    {
      "profileId": "550e8400-e29b-41d4-a716-446655440000",
      "petName": "Max",
      "species": "dog",
      "breed": "Golden Retriever",
      "profilePhoto": "https://cdn.petlegacy.wia.org/profiles/max.jpg",
      "epitaph": "Forever in our hearts",
      "relevanceScore": 0.95
    }
  ],
  "searchMetadata": {
    "query": "golden retriever",
    "totalResults": 1247,
    "searchTime": 0.034
  }
}
```

### 8.2 Discover Public Memorials

```http
GET /v1/discover/memorials
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `category` | string | trending, recent, popular |
| `timeframe` | string | day, week, month, year |

### 8.3 Search Media

```http
GET /v1/search/media
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `q` | string | Search in titles/descriptions |
| `tags` | string | Filter by tags |
| `type` | string | photo, video, audio |
| `dateRange` | string | Date range filter |

---

## 9. AI & Analytics Endpoints

### 9.1 Generate Memory Compilation

```http
POST /v1/memorials/{memorialId}/ai/compile
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "compilationType": "video_slideshow",
  "includePhotos": true,
  "includeVideos": false,
  "musicPreference": "gentle",
  "duration": 180,
  "theme": "celebration_of_life"
}
```

**Response (202 Accepted):**
```json
{
  "success": true,
  "data": {
    "compilationId": "comp_3k7m9n2p5q",
    "status": "processing",
    "estimatedTime": "5 minutes",
    "statusUrl": "/v1/ai/compilations/comp_3k7m9n2p5q"
  }
}
```

### 9.2 Generate Memorial Summary

```http
POST /v1/memorials/{memorialId}/ai/summarize
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "summaryType": "life_story",
  "tone": "warm_and_loving",
  "length": "medium",
  "includeMilestones": true,
  "includePersonality": true
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "summary": "Max was a beloved Golden Retriever who brought joy to everyone he met. Born in 2012, he spent 11 wonderful years as a cherished family member...",
    "wordCount": 487,
    "generatedAt": "2024-12-18T15:30:00Z",
    "confidence": 0.92
  }
}
```

### 9.3 Get Memorial Analytics

```http
GET /v1/memorials/{memorialId}/analytics
Authorization: Bearer {access_token}
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `metric` | string | views, tributes, shares, engagement |
| `period` | string | day, week, month, year, all |
| `granularity` | string | hourly, daily, weekly, monthly |

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "period": {
      "start": "2024-11-18",
      "end": "2024-12-18"
    },
    "metrics": {
      "totalViews": 1247,
      "uniqueVisitors": 892,
      "tributeCount": 89,
      "shareCount": 156,
      "averageTimeOnPage": 342,
      "topReferrers": [
        {"source": "facebook", "count": 45},
        {"source": "direct", "count": 38}
      ]
    },
    "timeline": [
      {"date": "2024-12-11", "views": 42, "tributes": 3},
      {"date": "2024-12-12", "views": 38, "tributes": 5}
    ]
  }
}
```

### 9.4 AI Photo Enhancement

```http
POST /v1/memorials/{memorialId}/media/{assetId}/enhance
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "enhancements": ["color_correction", "sharpening", "noise_reduction"],
  "preserveOriginal": true
}
```

### 9.5 AI Tag Suggestions

```http
POST /v1/memorials/{memorialId}/media/{assetId}/ai/tags
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "suggestedTags": [
      {"tag": "beach", "confidence": 0.98},
      {"tag": "ocean", "confidence": 0.95},
      {"tag": "summer", "confidence": 0.89},
      {"tag": "golden_retriever", "confidence": 0.99}
    ],
    "detectedObjects": ["dog", "water", "sand", "sky"],
    "sceneType": "outdoor_beach"
  }
}
```

---

## 10. Webhook Integration

### 10.1 Configure Webhook

```http
POST /v1/webhooks
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://yourapp.com/webhooks/petlegacy",
  "events": [
    "memorial.created",
    "memorial.updated",
    "media.uploaded",
    "timeline.event_created",
    "family.member_joined"
  ],
  "secret": "your_webhook_secret"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "webhookId": "wh_5k7m9n2p4q",
    "url": "https://yourapp.com/webhooks/petlegacy",
    "events": ["memorial.created", "memorial.updated"],
    "status": "active",
    "createdAt": "2024-12-18T16:00:00Z"
  }
}
```

### 10.2 Webhook Events

| Event | Description | Payload |
|-------|-------------|---------|
| `memorial.created` | New memorial created | Full memorial object |
| `memorial.updated` | Memorial modified | Changed fields + full object |
| `memorial.deleted` | Memorial deleted | Memorial ID only |
| `media.uploaded` | New media asset added | Media object |
| `media.processed` | Media processing complete | Media object with URLs |
| `timeline.event_created` | Timeline event added | Event object |
| `family.member_invited` | New member invited | Invitation object |
| `family.member_joined` | Member accepted invite | Member object |
| `export.completed` | Export package ready | Download URL |

### 10.3 Webhook Payload Example

```json
{
  "eventId": "evt_9k2m5n7p8q",
  "eventType": "media.uploaded",
  "timestamp": "2024-12-18T16:05:00Z",
  "apiVersion": "1.0.0",
  "data": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "type": "photo",
    "uploadedBy": "660e8400-e29b-41d4-a716-446655440001",
    "uploadedAt": "2024-12-18T16:05:00Z"
  }
}
```

---

## 11. Error Handling

### 11.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid request data",
    "details": [
      {
        "field": "petIdentity.birthDate",
        "issue": "Birth date cannot be in the future",
        "rejectedValue": "2025-12-25"
      }
    ],
    "requestId": "req_7x9kp3m2n5",
    "timestamp": "2024-12-18T16:10:00Z",
    "documentation": "https://docs.petlegacy.wia.org/errors/VALIDATION_ERROR"
  }
}
```

### 11.2 HTTP Status Codes

| Code | Status | Description |
|------|--------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created successfully |
| 202 | Accepted | Request accepted for processing |
| 204 | No Content | Success with no response body |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict (duplicate) |
| 422 | Unprocessable Entity | Validation failed |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary service outage |

### 11.3 Common Error Codes

| Error Code | HTTP Status | Description |
|------------|-------------|-------------|
| `AUTHENTICATION_REQUIRED` | 401 | No authentication provided |
| `INVALID_TOKEN` | 401 | Token expired or invalid |
| `INSUFFICIENT_PERMISSIONS` | 403 | Missing required permissions |
| `RESOURCE_NOT_FOUND` | 404 | Requested resource doesn't exist |
| `VALIDATION_ERROR` | 422 | Request validation failed |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `MEDIA_TOO_LARGE` | 413 | File exceeds size limit |
| `UNSUPPORTED_FORMAT` | 415 | File format not supported |
| `QUOTA_EXCEEDED` | 403 | Account quota limit reached |

---

## 12. Rate Limiting & Quotas

### 12.1 Rate Limits

| Tier | Requests/Minute | Requests/Hour | Requests/Day |
|------|-----------------|---------------|--------------|
| **Free** | 60 | 1,000 | 10,000 |
| **Basic** | 300 | 10,000 | 100,000 |
| **Pro** | 1,000 | 50,000 | 500,000 |
| **Enterprise** | Custom | Custom | Custom |

### 12.2 Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1703347260
Retry-After: 30
```

### 12.3 Storage Quotas

| Tier | Total Storage | Max File Size | Max Memorials |
|------|---------------|---------------|---------------|
| **Free** | 5 GB | 50 MB | 3 |
| **Basic** | 50 GB | 100 MB | 25 |
| **Pro** | 500 GB | 500 MB | Unlimited |
| **Enterprise** | Custom | Custom | Unlimited |

---

## Appendix A: Code Examples

### Example 1: Node.js SDK Usage

```javascript
const PetLegacy = require('@wia/pet-legacy');

const client = new PetLegacy({
  apiKey: 'your_api_key',
  environment: 'production'
});

// Create memorial
async function createMemorial() {
  try {
    const memorial = await client.memorials.create({
      petIdentity: {
        name: 'Max',
        species: 'dog',
        breed: 'Golden Retriever',
        birthDate: '2012-05-20',
        passingDate: '2024-03-10'
      },
      guardianInfo: {
        userId: 'user_123',
        name: 'Sarah Johnson'
      }
    });

    console.log('Memorial created:', memorial.profileId);
    return memorial;
  } catch (error) {
    console.error('Error:', error.message);
  }
}

// Upload photo
async function uploadPhoto(memorialId, filePath) {
  const photo = await client.media.upload(memorialId, {
    file: fs.createReadStream(filePath),
    title: 'Max at the Beach',
    tags: ['beach', 'summer']
  });

  return photo;
}
```

### Example 2: Python SDK Usage

```python
from wia_pet_legacy import PetLegacyClient

client = PetLegacyClient(api_key='your_api_key')

# Create memorial
memorial = client.memorials.create({
    'petIdentity': {
        'name': 'Max',
        'species': 'dog',
        'breed': 'Golden Retriever',
        'birthDate': '2012-05-20',
        'passingDate': '2024-03-10'
    },
    'guardianInfo': {
        'userId': 'user_123',
        'name': 'Sarah Johnson'
    }
})

print(f"Memorial created: {memorial.profile_id}")

# Upload media
with open('max-photo.jpg', 'rb') as photo_file:
    media = client.media.upload(
        memorial_id=memorial.profile_id,
        file=photo_file,
        metadata={
            'title': 'Max at the Beach',
            'tags': ['beach', 'summer']
        }
    )
```

### Example 3: cURL Examples

```bash
# Create memorial
curl -X POST https://api.petlegacy.wia.org/v1/memorials \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "petIdentity": {
      "name": "Max",
      "species": "dog",
      "breed": "Golden Retriever",
      "birthDate": "2012-05-20",
      "passingDate": "2024-03-10"
    }
  }'

# Upload photo
curl -X POST https://api.petlegacy.wia.org/v1/memorials/MEMORIAL_ID/media \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -F "file=@max-photo.jpg" \
  -F 'metadata={"title":"Max at the Beach","tags":["beach","summer"]}'

# Get memorial with timeline
curl -X GET "https://api.petlegacy.wia.org/v1/memorials/MEMORIAL_ID?include=timeline,media" \
  -H "Authorization: Bearer YOUR_TOKEN"
```

### Example 4: Webhook Handler

```javascript
const express = require('express');
const crypto = require('crypto');

const app = express();
app.use(express.json());

app.post('/webhooks/petlegacy', (req, res) => {
  // Verify webhook signature
  const signature = req.headers['x-webhook-signature'];
  const payload = JSON.stringify(req.body);
  const secret = process.env.WEBHOOK_SECRET;

  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');

  if (signature !== expectedSignature) {
    return res.status(401).send('Invalid signature');
  }

  // Process webhook event
  const { eventType, data } = req.body;

  switch (eventType) {
    case 'media.uploaded':
      console.log(`New media uploaded: ${data.assetId}`);
      // Process new media
      break;
    case 'memorial.created':
      console.log(`New memorial created: ${data.memorialId}`);
      // Send welcome email
      break;
  }

  res.status(200).send('OK');
});

app.listen(3000);
```

### Example 5: Pagination Helper

```javascript
async function getAllTimelineEvents(memorialId) {
  const allEvents = [];
  let page = 1;
  let hasMore = true;

  while (hasMore) {
    const response = await client.timeline.list(memorialId, {
      page: page,
      perPage: 100
    });

    allEvents.push(...response.data);
    hasMore = response.pagination.hasNext;
    page++;
  }

  return allEvents;
}
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
