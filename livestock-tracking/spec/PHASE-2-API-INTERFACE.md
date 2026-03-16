# WIA Livestock Tracking API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**License**: MIT

---

## Table of Contents

1. [API Overview](#api-overview)
2. [Authentication](#authentication)
3. [Animal Management](#animal-management)
4. [Location Tracking](#location-tracking)
5. [Health Records](#health-records)
6. [Traceability](#traceability)
7. [Webhooks](#webhooks)
8. [Error Handling](#error-handling)

---

## API Overview

### 1.1 Base URL

```
Production: https://api.livestock.wia.com/v1
Sandbox: https://sandbox.livestock.wia.com/v1
```

### 1.2 Request Format

- Content-Type: `application/json`
- Character Encoding: UTF-8
- HTTP Methods: GET, POST, PUT, DELETE
- Rate Limit: 1000 requests/hour

---

## Authentication

### 2.1 API Key Authentication

```http
GET /animals HTTP/1.1
Host: api.livestock.wia.com
Authorization: Bearer YOUR_API_KEY
```

### 2.2 OAuth 2.0

```http
POST /oauth/token HTTP/1.1
Host: api.livestock.wia.com
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
```

---

## Animal Management

### 3.1 Register New Animal

```http
POST /animals
```

**Request:**
```json
{
  "rfid_tag": "982-000123456789",
  "species": "CATTLE",
  "breed": "HANWOO",
  "birth_date": "2025-01-15",
  "gender": "FEMALE",
  "farm_id": "FARM-KR-001"
}
```

**Response (201):**
```json
{
  "animal_id": "CATTLE-KR-2025-001234",
  "rfid_tag": "982-000123456789",
  "status": "REGISTERED",
  "created_at": "2025-01-15T08:30:00Z"
}
```

### 3.2 Get Animal Details

```http
GET /animals/{animal_id}
```

**Response (200):**
```json
{
  "animal_id": "CATTLE-KR-2025-001234",
  "rfid_tag": "982-000123456789",
  "species": "CATTLE",
  "breed": "HANWOO",
  "birth_date": "2025-01-15",
  "current_weight_kg": 450.5,
  "health_status": "HEALTHY",
  "location": {
    "lat": 37.566535,
    "lng": 126.977969,
    "last_update": "2025-01-15T14:22:00Z"
  }
}
```

### 3.3 Update Animal Information

```http
PUT /animals/{animal_id}
```

**Request:**
```json
{
  "current_weight_kg": 455.0,
  "health_status": "HEALTHY"
}
```

### 3.4 List Animals

```http
GET /animals?farm_id=FARM-KR-001&status=ACTIVE&page=1&limit=50
```

**Response (200):**
```json
{
  "animals": [...],
  "total": 125,
  "page": 1,
  "limit": 50
}
```

---

## Location Tracking

### 4.1 Update Location

```http
POST /animals/{animal_id}/location
```

**Request:**
```json
{
  "lat": 37.566535,
  "lng": 126.977969,
  "accuracy_m": 2.5,
  "device_id": "GPS-COLLAR-001",
  "timestamp": "2025-01-15T14:22:00Z"
}
```

### 4.2 Get Location History

```http
GET /animals/{animal_id}/location/history?start_date=2025-01-01&end_date=2025-01-15
```

**Response (200):**
```json
{
  "animal_id": "CATTLE-KR-2025-001234",
  "locations": [
    {
      "timestamp": "2025-01-15T08:00:00Z",
      "lat": 37.566535,
      "lng": 126.977969,
      "activity": "GRAZING"
    }
  ]
}
```

---

## Health Records

### 5.1 Add Health Record

```http
POST /animals/{animal_id}/health
```

**Request:**
```json
{
  "type": "VACCINATION",
  "date": "2025-01-15",
  "veterinarian": "Dr. Kim",
  "vaccination": {
    "vaccine_name": "FMD",
    "dose_ml": 2.0
  }
}
```

### 5.2 Get Health History

```http
GET /animals/{animal_id}/health
```

---

## Traceability

### 6.1 Get Blockchain Proof

```http
GET /animals/{animal_id}/traceability
```

**Response (200):**
```json
{
  "animal_id": "CATTLE-KR-2025-001234",
  "blockchain": {
    "network": "ETHEREUM",
    "hash": "0x1a2b3c...",
    "verified": true
  },
  "supply_chain": [...]
}
```

### 6.2 Generate QR Code

```http
GET /animals/{animal_id}/qr
```

**Response (200):**
```json
{
  "qr_code_url": "https://livestock.wia.com/verify/CATTLE-KR-2025-001234",
  "qr_image_base64": "data:image/png;base64,..."
}
```

---

## Webhooks

### 7.1 Register Webhook

```http
POST /webhooks
```

**Request:**
```json
{
  "url": "https://yourserver.com/webhook",
  "events": ["ANIMAL_MOVED", "HEALTH_ALERT", "GEOFENCE_EXIT"],
  "secret": "YOUR_WEBHOOK_SECRET"
}
```

### 7.2 Webhook Events

**GEOFENCE_EXIT:**
```json
{
  "event": "GEOFENCE_EXIT",
  "animal_id": "CATTLE-KR-2025-001234",
  "timestamp": "2025-01-15T14:22:00Z",
  "location": {
    "lat": 37.566535,
    "lng": 126.977969
  }
}
```

---

## Error Handling

### 8.1 Error Response Format

```json
{
  "error": {
    "code": "ANIMAL_NOT_FOUND",
    "message": "Animal with ID CATTLE-KR-2025-999999 not found",
    "request_id": "req_123456",
    "timestamp": "2025-01-15T14:22:00Z"
  }
}
```

### 8.2 HTTP Status Codes

| Code | Meaning |
|------|---------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request |
| 401 | Unauthorized |
| 404 | Not Found |
| 429 | Rate Limit Exceeded |
| 500 | Internal Server Error |

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA-AGRI-009 Livestock Tracking Standard*
*© 2025 WIA - MIT License*
