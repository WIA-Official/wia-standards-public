# WIA Digital Funeral Standard - Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #64748B (Slate)
**Series:** Digital Death Services

---

## Table of Contents

1. [Introduction](#introduction)
2. [API Architecture](#api-architecture)
3. [Authentication and Authorization](#authentication-and-authorization)
4. [Deceased Person Management APIs](#deceased-person-management-apis)
5. [Funeral Service APIs](#funeral-service-apis)
6. [Guest and Attendee APIs](#guest-and-attendee-apis)
7. [Memorial and Tribute APIs](#memorial-and-tribute-apis)
8. [Financial Transaction APIs](#financial-transaction-apis)
9. [Cemetery and Interment APIs](#cemetery-and-interment-apis)
10. [Streaming and Media APIs](#streaming-and-media-apis)
11. [Notification APIs](#notification-apis)
12. [Error Handling](#error-handling)
13. [Rate Limiting and Quotas](#rate-limiting-and-quotas)
14. [Implementation Examples](#implementation-examples)

---

## 1. Introduction

### 1.1 Purpose

This specification defines RESTful API interfaces for digital funeral services, enabling standardized integration between funeral service providers, memorial platforms, streaming services, and third-party applications. The APIs support complete lifecycle management of funeral services from pre-need planning through post-service memorial maintenance.

### 1.2 API Design Principles

- **RESTful Architecture**: Resource-oriented URLs with standard HTTP methods
- **JSON Format**: All requests and responses use JSON encoding
- **Versioning**: API version included in URL path (e.g., `/v1/`)
- **Idempotency**: Safe retries for critical operations
- **Pagination**: Consistent pagination for list endpoints
- **Security**: OAuth 2.0 authentication with role-based access control
- **Privacy**: Data access controls respecting privacy preferences

### 1.3 Base URL Structure

```
Production: https://api.funeral-service.example.com/v1
Staging: https://api-staging.funeral-service.example.com/v1
Sandbox: https://api-sandbox.funeral-service.example.com/v1
```

---

## 2. API Architecture

### 2.1 HTTP Methods

| Method | Usage | Idempotent |
|--------|-------|------------|
| GET | Retrieve resources | Yes |
| POST | Create new resources | No |
| PUT | Replace entire resource | Yes |
| PATCH | Partial update of resource | No |
| DELETE | Remove resource | Yes |

### 2.2 Standard Response Format

```json
{
  "success": true,
  "data": {},
  "metadata": {
    "timestamp": "2025-12-18T10:30:00Z",
    "requestId": "req_1234567890",
    "version": "1.0.0"
  },
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalPages": 5,
    "totalCount": 98
  }
}
```

### 2.3 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid date format for dateOfDeath",
    "details": [
      {
        "field": "dateOfDeath",
        "issue": "Must be in ISO 8601 format"
      }
    ],
    "requestId": "req_1234567890",
    "timestamp": "2025-12-18T10:30:00Z"
  }
}
```

### 2.4 Common Query Parameters

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| page | integer | Page number (1-indexed) | `?page=2` |
| pageSize | integer | Items per page (max 100) | `?pageSize=50` |
| sort | string | Sort field and direction | `?sort=createdAt:desc` |
| filter | string | Filter criteria | `?filter=status:published` |
| include | string | Related resources to include | `?include=services,photos` |
| fields | string | Specific fields to return | `?fields=id,name,dateOfDeath` |

---

## 3. Authentication and Authorization

### 3.1 OAuth 2.0 Authentication

```http
POST /v1/auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "funeral.read funeral.write"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "funeral.read funeral.write"
}
```

### 3.2 Authorization Header

```http
GET /v1/deceased/550e8400-e29b-41d4-a716-446655440000
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 3.3 API Scopes

| Scope | Description | Access Level |
|-------|-------------|--------------|
| funeral.read | Read funeral service data | Read-only |
| funeral.write | Create and modify funeral data | Read/Write |
| funeral.admin | Administrative operations | Full access |
| memorial.read | Read memorial and tribute data | Read-only |
| memorial.write | Post tributes and condolences | Read/Write |
| financial.read | View financial transactions | Read-only |
| financial.write | Process payments and donations | Read/Write |
| streaming.manage | Control live streaming | Read/Write |

### 3.4 Role-Based Access Control

| Role | Permissions | Typical User |
|------|-------------|--------------|
| family_admin | Full access to deceased and services | Primary family contact |
| family_member | Read access, limited write | Family members |
| funeral_director | Manage services, view financials | Funeral home staff |
| guest | View public info, post condolences | Service attendees |
| service_provider | Access specific service data | Florists, caterers |

---

## 4. Deceased Person Management APIs

### 4.1 Create Deceased Person Record

```http
POST /v1/deceased
Authorization: Bearer {token}
Content-Type: application/json

{
  "legalName": {
    "prefix": "Dr.",
    "firstName": "Margaret",
    "middleName": "Anne",
    "lastName": "Johnson",
    "suffix": "PhD",
    "preferredName": "Maggie"
  },
  "dateOfBirth": "1945-03-15",
  "dateOfDeath": "2025-12-10T08:30:00Z",
  "placeOfBirth": {
    "city": "Boston",
    "state": "Massachusetts",
    "country": "US"
  },
  "placeOfDeath": {
    "facility": "St. Mary's Hospital",
    "city": "Portland",
    "state": "Oregon",
    "country": "US"
  },
  "privacy": {
    "level": "public",
    "showDateOfBirth": true,
    "showFullBiography": true
  }
}
```

**Response: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "legalName": {
      "prefix": "Dr.",
      "firstName": "Margaret",
      "middleName": "Anne",
      "lastName": "Johnson",
      "suffix": "PhD",
      "preferredName": "Maggie"
    },
    "dateOfBirth": "1945-03-15",
    "dateOfDeath": "2025-12-10T08:30:00Z",
    "metadata": {
      "createdAt": "2025-12-18T10:30:00Z",
      "verificationStatus": "pending"
    }
  },
  "metadata": {
    "timestamp": "2025-12-18T10:30:00Z",
    "requestId": "req_1234567890"
  }
}
```

### 4.2 Get Deceased Person Details

```http
GET /v1/deceased/{deceasedId}
Authorization: Bearer {token}
```

**Query Parameters:**
- `include`: Related resources (e.g., `services,photos,tributes`)
- `fields`: Specific fields to return

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "legalName": {
      "firstName": "Margaret",
      "lastName": "Johnson",
      "preferredName": "Maggie"
    },
    "dateOfBirth": "1945-03-15",
    "dateOfDeath": "2025-12-10T08:30:00Z",
    "biography": {
      "short": "Dr. Margaret Johnson, beloved mother and professor...",
      "full": "Dr. Margaret 'Maggie' Johnson dedicated her life..."
    },
    "photos": [
      {
        "url": "https://cdn.example.com/photos/maggie-primary.jpg",
        "isPrimary": true
      }
    ]
  }
}
```

### 4.3 Update Deceased Person Information

```http
PATCH /v1/deceased/{deceasedId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "biography": {
    "full": "Updated biography text...",
    "achievements": [
      "Professor Emerita, Portland State University",
      "Author of three books on American poetry"
    ]
  }
}
```

**Response: 200 OK**

### 4.4 List Deceased Persons

```http
GET /v1/deceased?page=1&pageSize=20&sort=dateOfDeath:desc
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "legalName": {
        "firstName": "Margaret",
        "lastName": "Johnson"
      },
      "dateOfDeath": "2025-12-10T08:30:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalPages": 3,
    "totalCount": 58
  }
}
```

### 4.5 Upload Deceased Person Photo

```http
POST /v1/deceased/{deceasedId}/photos
Authorization: Bearer {token}
Content-Type: multipart/form-data

file: [binary image data]
caption: "Dr. Johnson at her retirement celebration"
yearTaken: 2010
isPrimary: true
```

**Response: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "photo_123456",
    "url": "https://cdn.example.com/photos/550e8400/photo_123456.jpg",
    "caption": "Dr. Johnson at her retirement celebration",
    "yearTaken": 2010,
    "isPrimary": true,
    "uploadedAt": "2025-12-18T11:00:00Z"
  }
}
```

### 4.6 Delete Deceased Person Photo

```http
DELETE /v1/deceased/{deceasedId}/photos/{photoId}
Authorization: Bearer {token}
```

**Response: 204 No Content**

---

## 5. Funeral Service APIs

### 5.1 Create Funeral Service

```http
POST /v1/services
Authorization: Bearer {token}
Content-Type: application/json

{
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "memorial",
  "religiousTradition": "non-religious",
  "startDateTime": "2025-12-18T14:00:00-08:00",
  "endDateTime": "2025-12-18T16:00:00-08:00",
  "timezone": "America/Los_Angeles",
  "venue": {
    "type": "funeral-home",
    "name": "Peaceful Rest Memorial Chapel",
    "address": {
      "street": "1234 Oak Street",
      "city": "Portland",
      "state": "Oregon",
      "postalCode": "97201",
      "country": "US"
    },
    "capacity": 150
  },
  "virtualService": {
    "enabled": true,
    "platform": "custom",
    "recordingEnabled": true,
    "chatEnabled": true
  },
  "rsvpRequired": true,
  "rsvpDeadline": "2025-12-16T23:59:59-08:00"
}
```

**Response: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "660e8400-e29b-41d4-a716-446655440001",
    "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
    "serviceType": "memorial",
    "startDateTime": "2025-12-18T14:00:00-08:00",
    "status": "planned",
    "virtualService": {
      "enabled": true,
      "streamingUrl": "https://stream.example.com/service/660e8400",
      "accessCode": "MAGGIE2025"
    }
  }
}
```

### 5.2 Get Service Details

```http
GET /v1/services/{serviceId}
Authorization: Bearer {token}
```

**Response: 200 OK**

### 5.3 Update Service Information

```http
PATCH /v1/services/{serviceId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "status": "confirmed",
  "expectedAttendance": 120,
  "program": [
    {
      "order": 1,
      "type": "music",
      "title": "Prelude",
      "duration": 10,
      "performer": "Emily Chen, pianist"
    }
  ]
}
```

**Response: 200 OK**

### 5.4 List Services for Deceased

```http
GET /v1/deceased/{deceasedId}/services
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": [
    {
      "id": "660e8400-e29b-41d4-a716-446655440001",
      "serviceType": "memorial",
      "startDateTime": "2025-12-18T14:00:00-08:00",
      "venue": {
        "name": "Peaceful Rest Memorial Chapel"
      },
      "status": "confirmed"
    }
  ]
}
```

### 5.5 Cancel Service

```http
POST /v1/services/{serviceId}/cancel
Authorization: Bearer {token}
Content-Type: application/json

{
  "reason": "Weather conditions - rescheduling",
  "notifyAttendees": true
}
```

**Response: 200 OK**

### 5.6 Create Pre-Need Plan

```http
POST /v1/preneed-plans
Authorization: Bearer {token}
Content-Type: application/json

{
  "planHolderId": "bb0e8400-e29b-41d4-a716-446655440006",
  "servicePreferences": {
    "serviceType": ["memorial", "celebration-of-life"],
    "religiousTradition": "non-religious",
    "musicSelections": [
      "What a Wonderful World - Louis Armstrong"
    ]
  },
  "dispositionPreferences": {
    "method": "cremation",
    "cemetery": "Green Valley Cemetery"
  },
  "contacts": {
    "primaryContact": {
      "name": "Jessica Anderson",
      "relationship": "daughter",
      "phone": "+1-503-555-0234",
      "email": "jessica.anderson@email.com"
    }
  }
}
```

**Response: 201 Created**

### 5.7 Get Pre-Need Plan

```http
GET /v1/preneed-plans/{planId}
Authorization: Bearer {token}
```

**Response: 200 OK**

### 5.8 Update Pre-Need Plan

```http
PATCH /v1/preneed-plans/{planId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "financialPlan": {
    "amountPaid": 5100.00
  },
  "status": "active"
}
```

**Response: 200 OK**

---

## 6. Guest and Attendee APIs

### 6.1 Submit RSVP

```http
POST /v1/services/{serviceId}/rsvp
Content-Type: application/json

{
  "attendeeInfo": {
    "name": "David Miller",
    "email": "david.miller@email.com",
    "phone": "+1-503-555-0199",
    "relationship": "former student"
  },
  "response": "attending",
  "attendanceType": "in-person",
  "numberOfGuests": 2,
  "guestNames": ["David Miller", "Lisa Miller"],
  "dietaryRestrictions": ["vegetarian"]
}
```

**Response: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "770e8400-e29b-41d4-a716-446655440002",
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "response": "attending",
    "confirmationCode": "RSVP-DM-2025",
    "submittedAt": "2025-12-14T10:30:00Z"
  }
}
```

### 6.2 Update RSVP

```http
PATCH /v1/services/{serviceId}/rsvp/{rsvpId}
Content-Type: application/json

{
  "response": "not-attending",
  "numberOfGuests": 0
}
```

**Response: 200 OK**

### 6.3 Get RSVP Details

```http
GET /v1/services/{serviceId}/rsvp/{rsvpId}
```

**Response: 200 OK**

### 6.4 List Service RSVPs

```http
GET /v1/services/{serviceId}/rsvp?filter=response:attending
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": [
    {
      "id": "770e8400-e29b-41d4-a716-446655440002",
      "attendeeInfo": {
        "name": "David Miller",
        "relationship": "former student"
      },
      "response": "attending",
      "numberOfGuests": 2
    }
  ],
  "metadata": {
    "summary": {
      "attending": 45,
      "notAttending": 12,
      "maybe": 3,
      "virtualOnly": 18
    }
  }
}
```

### 6.5 Get Attendance Summary

```http
GET /v1/services/{serviceId}/attendance-summary
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "totalResponses": 78,
    "attending": 45,
    "notAttending": 12,
    "maybe": 3,
    "virtualOnly": 18,
    "totalGuests": 89,
    "inPerson": 71,
    "virtual": 18
  }
}
```

---

## 7. Memorial and Tribute APIs

### 7.1 Post Condolence Message

```http
POST /v1/deceased/{deceasedId}/condolences
Content-Type: application/json

{
  "authorName": "Rebecca Thompson",
  "authorEmail": "rebecca.t@email.com",
  "relationship": "former student",
  "message": "Dr. Johnson was the most inspiring professor I ever had...",
  "isPublic": true
}
```

**Response: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "880e8400-e29b-41d4-a716-446655440003",
    "authorName": "Rebecca Thompson",
    "message": "Dr. Johnson was the most inspiring professor...",
    "moderationStatus": "pending",
    "postedAt": "2025-12-12T16:45:00Z"
  }
}
```

### 7.2 List Condolence Messages

```http
GET /v1/deceased/{deceasedId}/condolences?page=1&pageSize=20&sort=postedAt:desc
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": [
    {
      "id": "880e8400-e29b-41d4-a716-446655440003",
      "authorName": "Rebecca Thompson",
      "relationship": "former student",
      "message": "Dr. Johnson was the most inspiring...",
      "postedAt": "2025-12-12T16:45:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalCount": 47
  }
}
```

### 7.3 Moderate Condolence Message

```http
PATCH /v1/condolences/{condolenceId}/moderate
Authorization: Bearer {token}
Content-Type: application/json

{
  "moderationStatus": "approved"
}
```

**Response: 200 OK**

### 7.4 Delete Condolence Message

```http
DELETE /v1/condolences/{condolenceId}
Authorization: Bearer {token}
```

**Response: 204 No Content**

### 7.5 Create Memorial Tribute

```http
POST /v1/deceased/{deceasedId}/tributes
Authorization: Bearer {token}
Content-Type: application/json

{
  "type": "photo-album",
  "title": "Memories Through the Years",
  "content": {
    "text": "A collection of photos celebrating Dr. Johnson's life",
    "mediaUrls": [
      "https://cdn.example.com/tributes/photo1.jpg",
      "https://cdn.example.com/tributes/photo2.jpg"
    ]
  },
  "author": {
    "name": "Johnson Family",
    "relationship": "family"
  },
  "isPublic": true
}
```

**Response: 201 Created**

### 7.6 Get Tribute Details

```http
GET /v1/tributes/{tributeId}
```

**Response: 200 OK**

### 7.7 List Tributes

```http
GET /v1/deceased/{deceasedId}/tributes?type=video-montage
```

**Response: 200 OK**

---

## 8. Financial Transaction APIs

### 8.1 Process Donation

```http
POST /v1/deceased/{deceasedId}/donations
Content-Type: application/json

{
  "amount": 100.00,
  "currency": "USD",
  "donorInfo": {
    "name": "John Smith",
    "email": "john.smith@email.com",
    "isAnonymous": false
  },
  "recipient": {
    "type": "charity",
    "name": "Portland Public Library Foundation",
    "taxId": "12-3456789"
  },
  "message": "In honor of Dr. Johnson's dedication to literacy",
  "paymentMethod": "credit-card",
  "paymentDetails": {
    "token": "tok_visa_1234567890"
  }
}
```

**Response: 201 Created**
```json
{
  "success": true,
  "data": {
    "id": "990e8400-e29b-41d4-a716-446655440004",
    "amount": 100.00,
    "currency": "USD",
    "status": "completed",
    "transactionId": "txn_1234567890",
    "receiptUrl": "https://receipts.example.com/990e8400",
    "donatedAt": "2025-12-13T09:15:00Z"
  }
}
```

### 8.2 Get Donation Details

```http
GET /v1/donations/{donationId}
Authorization: Bearer {token}
```

**Response: 200 OK**

### 8.3 List Donations

```http
GET /v1/deceased/{deceasedId}/donations?page=1&pageSize=20
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": [
    {
      "id": "990e8400-e29b-41d4-a716-446655440004",
      "amount": 100.00,
      "currency": "USD",
      "donorInfo": {
        "name": "John Smith",
        "isAnonymous": false
      },
      "donatedAt": "2025-12-13T09:15:00Z"
    }
  ],
  "metadata": {
    "totalDonations": 47,
    "totalAmount": 4750.00,
    "currency": "USD"
  }
}
```

### 8.4 Record Floral Arrangement

```http
POST /v1/services/{serviceId}/flowers
Authorization: Bearer {token}
Content-Type: application/json

{
  "type": "standing-spray",
  "description": "White roses and lilies",
  "florist": {
    "name": "Portland Flowers",
    "phone": "+1-503-555-0300",
    "orderNumber": "FLW-12345"
  },
  "sender": {
    "name": "ABC Corporation",
    "relationship": "employer"
  },
  "cardMessage": "In loving memory of Dr. Johnson",
  "deliveryDate": "2025-12-18",
  "deliveryLocation": "funeral-home"
}
```

**Response: 201 Created**

### 8.5 List Floral Arrangements

```http
GET /v1/services/{serviceId}/flowers
Authorization: Bearer {token}
```

**Response: 200 OK**

---

## 9. Cemetery and Interment APIs

### 9.1 Create Interment Record

```http
POST /v1/interments
Authorization: Bearer {token}
Content-Type: application/json

{
  "deceasedId": "550e8400-e29b-41d4-a716-446655440000",
  "cemetery": {
    "name": "River View Memorial Park",
    "address": {
      "street": "5678 Cemetery Lane",
      "city": "Portland",
      "state": "Oregon",
      "postalCode": "97202",
      "country": "US"
    }
  },
  "plotLocation": {
    "section": "Garden of Memories",
    "lot": "42",
    "grave": "3"
  },
  "intermentType": "cremains",
  "intermentDate": "2025-12-20T11:00:00-08:00",
  "perpetualCare": true
}
```

**Response: 201 Created**

### 9.2 Get Interment Details

```http
GET /v1/interments/{intermentId}
```

**Response: 200 OK**

### 9.3 Update Monument Information

```http
PATCH /v1/interments/{intermentId}/monument
Authorization: Bearer {token}
Content-Type: application/json

{
  "monument": {
    "type": "marker",
    "material": "granite",
    "inscription": "Dr. Margaret Anne Johnson\n1945 - 2025\nBeloved Wife, Mother, and Teacher",
    "unveilingDate": "2026-12-10"
  }
}
```

**Response: 200 OK**

### 9.4 Search Cemetery Records

```http
GET /v1/cemeteries/{cemeteryId}/search?lastName=Johnson&section=Garden
Authorization: Bearer {token}
```

**Response: 200 OK**

---

## 10. Streaming and Media APIs

### 10.1 Start Live Stream

```http
POST /v1/services/{serviceId}/stream/start
Authorization: Bearer {token}
Content-Type: application/json

{
  "platform": "custom",
  "quality": "1080p",
  "enableChat": true,
  "enableRecording": true
}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "streamId": "stream_660e8400",
    "streamUrl": "https://stream.example.com/service/660e8400",
    "streamKey": "live_sk_1234567890",
    "viewerUrl": "https://watch.example.com/660e8400",
    "status": "live",
    "startedAt": "2025-12-18T14:00:00Z"
  }
}
```

### 10.2 Stop Live Stream

```http
POST /v1/services/{serviceId}/stream/stop
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "streamId": "stream_660e8400",
    "status": "ended",
    "endedAt": "2025-12-18T16:00:00Z",
    "duration": 7200,
    "viewerCount": 156,
    "recordingUrl": "https://cdn.example.com/recordings/660e8400.mp4"
  }
}
```

### 10.3 Get Stream Status

```http
GET /v1/services/{serviceId}/stream/status
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "streamId": "stream_660e8400",
    "status": "live",
    "currentViewers": 78,
    "peakViewers": 92,
    "duration": 3600,
    "quality": "1080p"
  }
}
```

### 10.4 Get Stream Analytics

```http
GET /v1/services/{serviceId}/stream/analytics
Authorization: Bearer {token}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "totalViews": 156,
    "uniqueViewers": 134,
    "averageWatchTime": 5400,
    "peakViewers": 92,
    "viewersByLocation": {
      "US": 120,
      "CA": 15,
      "UK": 8,
      "other": 13
    },
    "chatMessages": 47
  }
}
```

### 10.5 Upload Service Recording

```http
POST /v1/services/{serviceId}/recordings
Authorization: Bearer {token}
Content-Type: multipart/form-data

file: [binary video data]
title: "Memorial Service for Dr. Margaret Johnson"
description: "Full recording of the memorial service"
```

**Response: 201 Created**

### 10.6 Get Recording Details

```http
GET /v1/recordings/{recordingId}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "id": "rec_660e8400",
    "serviceId": "660e8400-e29b-41d4-a716-446655440001",
    "title": "Memorial Service for Dr. Margaret Johnson",
    "url": "https://cdn.example.com/recordings/660e8400.mp4",
    "duration": 7200,
    "fileSize": 1073741824,
    "uploadedAt": "2025-12-18T16:30:00Z",
    "views": 234
  }
}
```

---

## 11. Notification APIs

### 11.1 Send Service Reminder

```http
POST /v1/services/{serviceId}/notifications/reminder
Authorization: Bearer {token}
Content-Type: application/json

{
  "recipients": "all-rsvp",
  "channel": ["email", "sms"],
  "message": {
    "subject": "Reminder: Memorial Service Tomorrow",
    "body": "This is a reminder about the memorial service for Dr. Margaret Johnson..."
  },
  "scheduledFor": "2025-12-17T09:00:00Z"
}
```

**Response: 200 OK**
```json
{
  "success": true,
  "data": {
    "notificationId": "notif_1234567890",
    "recipientCount": 67,
    "scheduledFor": "2025-12-17T09:00:00Z",
    "status": "scheduled"
  }
}
```

### 11.2 Send Service Update

```http
POST /v1/services/{serviceId}/notifications/update
Authorization: Bearer {token}
Content-Type: application/json

{
  "recipients": "all-rsvp",
  "updateType": "time-change",
  "message": {
    "subject": "Service Time Updated",
    "body": "The service time has been changed to 3:00 PM..."
  }
}
```

**Response: 200 OK**

### 11.3 Subscribe to Notifications

```http
POST /v1/deceased/{deceasedId}/notifications/subscribe
Content-Type: application/json

{
  "email": "user@example.com",
  "phone": "+1-503-555-0100",
  "preferences": {
    "serviceUpdates": true,
    "newTributes": true,
    "anniversaryReminders": true
  }
}
```

**Response: 201 Created**

### 11.4 Unsubscribe from Notifications

```http
DELETE /v1/notifications/unsubscribe/{subscriptionId}
```

**Response: 204 No Content**

---

## 12. Error Handling

### 12.1 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| VALIDATION_ERROR | 400 | Request validation failed |
| AUTHENTICATION_ERROR | 401 | Invalid or missing authentication |
| AUTHORIZATION_ERROR | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| CONFLICT | 409 | Resource conflict (e.g., duplicate) |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Temporary service outage |

### 12.2 Error Response Examples

**Validation Error:**
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid request data",
    "details": [
      {
        "field": "dateOfDeath",
        "issue": "Must be in ISO 8601 format"
      },
      {
        "field": "privacy.level",
        "issue": "Must be one of: public, family-only, private"
      }
    ],
    "requestId": "req_1234567890"
  }
}
```

**Authorization Error:**
```json
{
  "success": false,
  "error": {
    "code": "AUTHORIZATION_ERROR",
    "message": "Insufficient permissions to access this resource",
    "requiredScope": "funeral.admin",
    "currentScope": "funeral.read",
    "requestId": "req_1234567890"
  }
}
```

**Not Found Error:**
```json
{
  "success": false,
  "error": {
    "code": "NOT_FOUND",
    "message": "Deceased person not found",
    "resourceType": "deceased",
    "resourceId": "550e8400-e29b-41d4-a716-446655440000",
    "requestId": "req_1234567890"
  }
}
```

---

## 13. Rate Limiting and Quotas

### 13.1 Rate Limits

| Tier | Requests per Minute | Requests per Hour | Requests per Day |
|------|---------------------|-------------------|------------------|
| Free | 60 | 1,000 | 10,000 |
| Basic | 300 | 10,000 | 100,000 |
| Professional | 1,000 | 50,000 | 500,000 |
| Enterprise | Custom | Custom | Custom |

### 13.2 Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1734529200
Retry-After: 3600
```

### 13.3 Rate Limit Exceeded Response

```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded",
    "limit": 1000,
    "remaining": 0,
    "resetAt": "2025-12-18T12:00:00Z",
    "retryAfter": 3600
  }
}
```

---

## 14. Implementation Examples

### 14.1 Complete Workflow: Creating a Funeral Service

```javascript
// Step 1: Authenticate
const authResponse = await fetch('https://api.funeral-service.example.com/v1/auth/token', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    grant_type: 'client_credentials',
    client_id: 'your_client_id',
    client_secret: 'your_client_secret',
    scope: 'funeral.write'
  })
});
const { access_token } = await authResponse.json();

// Step 2: Create deceased person record
const deceasedResponse = await fetch('https://api.funeral-service.example.com/v1/deceased', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${access_token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    legalName: {
      firstName: 'Margaret',
      lastName: 'Johnson',
      preferredName: 'Maggie'
    },
    dateOfBirth: '1945-03-15',
    dateOfDeath: '2025-12-10T08:30:00Z',
    privacy: {
      level: 'public'
    }
  })
});
const { data: deceased } = await deceasedResponse.json();

// Step 3: Create funeral service
const serviceResponse = await fetch('https://api.funeral-service.example.com/v1/services', {
  method: 'POST',
  headers: {
    'Authorization': `Bearer ${access_token}`,
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    deceasedId: deceased.id,
    serviceType: 'memorial',
    startDateTime: '2025-12-18T14:00:00-08:00',
    endDateTime: '2025-12-18T16:00:00-08:00',
    venue: {
      type: 'funeral-home',
      name: 'Peaceful Rest Memorial Chapel',
      address: {
        street: '1234 Oak Street',
        city: 'Portland',
        state: 'Oregon',
        postalCode: '97201'
      }
    },
    virtualService: {
      enabled: true,
      recordingEnabled: true
    }
  })
});
const { data: service } = await serviceResponse.json();

console.log('Service created:', service.id);
console.log('Streaming URL:', service.virtualService.streamingUrl);
```

### 14.2 Public Guest RSVP Submission

```javascript
// No authentication required for public RSVP
const rsvpResponse = await fetch(
  `https://api.funeral-service.example.com/v1/services/${serviceId}/rsvp`,
  {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      attendeeInfo: {
        name: 'David Miller',
        email: 'david.miller@email.com',
        relationship: 'former student'
      },
      response: 'attending',
      attendanceType: 'in-person',
      numberOfGuests: 2,
      guestNames: ['David Miller', 'Lisa Miller']
    })
  }
);

const { data: rsvp } = await rsvpResponse.json();
console.log('RSVP confirmed:', rsvp.confirmationCode);
```

### 14.3 Processing a Memorial Donation

```javascript
const donationResponse = await fetch(
  `https://api.funeral-service.example.com/v1/deceased/${deceasedId}/donations`,
  {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      amount: 100.00,
      currency: 'USD',
      donorInfo: {
        name: 'John Smith',
        email: 'john.smith@email.com',
        isAnonymous: false
      },
      recipient: {
        type: 'charity',
        name: 'Portland Public Library Foundation',
        taxId: '12-3456789'
      },
      message: 'In honor of Dr. Johnson',
      paymentMethod: 'credit-card',
      paymentDetails: {
        token: 'tok_visa_1234567890'
      }
    })
  }
);

const { data: donation } = await donationResponse.json();
console.log('Donation processed:', donation.transactionId);
console.log('Receipt URL:', donation.receiptUrl);
```

### 14.4 Managing Live Stream

```javascript
// Start stream before service
const startResponse = await fetch(
  `https://api.funeral-service.example.com/v1/services/${serviceId}/stream/start`,
  {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${access_token}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      platform: 'custom',
      quality: '1080p',
      enableChat: true,
      enableRecording: true
    })
  }
);
const { data: stream } = await startResponse.json();
console.log('Stream started:', stream.viewerUrl);

// Monitor stream during service
const statusInterval = setInterval(async () => {
  const statusResponse = await fetch(
    `https://api.funeral-service.example.com/v1/services/${serviceId}/stream/status`,
    {
      headers: { 'Authorization': `Bearer ${access_token}` }
    }
  );
  const { data: status } = await statusResponse.json();
  console.log('Current viewers:', status.currentViewers);
}, 30000); // Check every 30 seconds

// Stop stream after service
const stopResponse = await fetch(
  `https://api.funeral-service.example.com/v1/services/${serviceId}/stream/stop`,
  {
    method: 'POST',
    headers: { 'Authorization': `Bearer ${access_token}` }
  }
);
const { data: endedStream } = await stopResponse.json();
console.log('Total viewers:', endedStream.viewerCount);
console.log('Recording:', endedStream.recordingUrl);
```

### 14.5 Retrieving and Displaying Condolences

```javascript
// Fetch condolence messages (public, no auth required)
const condolencesResponse = await fetch(
  `https://api.funeral-service.example.com/v1/deceased/${deceasedId}/condolences?page=1&pageSize=20&sort=postedAt:desc`
);
const { data: condolences, pagination } = await condolencesResponse.json();

// Display on memorial page
condolences.forEach(condolence => {
  console.log(`${condolence.authorName} (${condolence.relationship}):`);
  console.log(condolence.message);
  console.log(`Posted: ${new Date(condolence.postedAt).toLocaleDateString()}`);
  console.log('---');
});

console.log(`Showing ${condolences.length} of ${pagination.totalCount} messages`);
```

### 14.6 Error Handling Best Practices

```javascript
async function createFuneralService(deceasedId, serviceData, token) {
  try {
    const response = await fetch(
      'https://api.funeral-service.example.com/v1/services',
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ deceasedId, ...serviceData })
      }
    );

    if (!response.ok) {
      const error = await response.json();

      switch (error.error.code) {
        case 'VALIDATION_ERROR':
          console.error('Validation failed:', error.error.details);
          // Show user-friendly validation messages
          break;
        case 'AUTHORIZATION_ERROR':
          console.error('Permission denied:', error.error.message);
          // Redirect to login or show permission error
          break;
        case 'RATE_LIMIT_EXCEEDED':
          const retryAfter = error.error.retryAfter;
          console.log(`Rate limited. Retry after ${retryAfter} seconds`);
          // Implement exponential backoff
          break;
        default:
          console.error('API error:', error.error.message);
      }

      throw new Error(error.error.message);
    }

    const { data } = await response.json();
    return data;
  } catch (error) {
    console.error('Request failed:', error);
    throw error;
  }
}
```

---

## Conclusion

This Phase 2 specification defines comprehensive RESTful APIs for digital funeral services. The APIs provide complete functionality for managing deceased persons, funeral services, guest attendance, memorials, financial transactions, and live streaming. All endpoints follow consistent patterns for authentication, error handling, and data formatting.

**Next Phase:** Phase 3 will define the protocol specifications for real-time communication, webhooks, and event streaming.

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
