# WIA Ocean Plastic Track API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime)

---

## Table of Contents

1. [Overview](#overview)
2. [Authentication](#authentication)
3. [API Endpoints](#api-endpoints)
4. [Request/Response Format](#requestresponse-format)
5. [Error Handling](#error-handling)
6. [Rate Limiting](#rate-limiting)
7. [SDK Examples](#sdk-examples)
8. [Webhooks](#webhooks)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Ocean Plastic Track API Interface Standard defines RESTful API endpoints, authentication methods, and integration patterns for ocean plastic tracking systems, enabling seamless data exchange between collection devices, recycling facilities, and monitoring platforms.

**Core Objectives**:
- Provide standardized API for plastic tracking data
- Enable real-time IoT device integration
- Support third-party application development
- Facilitate data sharing across organizations

### 1.2 API Design Principles

1. **RESTful**: Standard HTTP methods (GET, POST, PUT, DELETE)
2. **JSON-first**: All data exchange in JSON format
3. **Secure**: OAuth 2.0 and API Key authentication
4. **Versioned**: API version in URL path
5. **Paginated**: Large result sets paginated by default

### 1.3 Base URL

```
Production:  https://api.wia.live/ocean-plastic-track/v1
Staging:     https://api-staging.wia.live/ocean-plastic-track/v1
Development: https://api-dev.wia.live/ocean-plastic-track/v1
```

---

## Authentication

### 2.1 Supported Methods

| Method | Use Case | Security Level |
|--------|----------|----------------|
| OAuth 2.0 | User applications | High |
| API Key | Server-to-server | Medium |
| JWT | Mobile/Web apps | High |
| Device Token | IoT devices | Medium |

### 2.2 OAuth 2.0 Authentication

#### Authorization Code Flow

```http
# Step 1: Authorization Request
GET /oauth/authorize?
  client_id=YOUR_CLIENT_ID&
  response_type=code&
  redirect_uri=https://yourapp.com/callback&
  scope=read:batches write:batches&
  state=random_state_string

# Step 2: Token Exchange
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "authorization_code",
  "code": "AUTH_CODE",
  "client_id": "YOUR_CLIENT_ID",
  "client_secret": "YOUR_CLIENT_SECRET",
  "redirect_uri": "https://yourapp.com/callback"
}

# Response
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh_token_here",
  "scope": "read:batches write:batches"
}
```

#### Client Credentials Flow (Server-to-Server)

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "YOUR_CLIENT_ID",
  "client_secret": "YOUR_CLIENT_SECRET",
  "scope": "read:batches"
}
```

### 2.3 API Key Authentication

```http
GET /v1/batches
Authorization: ApiKey YOUR_API_KEY
```

**Generating API Keys:**

```http
POST /v1/auth/api-keys
Authorization: Bearer YOUR_OAUTH_TOKEN
Content-Type: application/json

{
  "name": "Production API Key",
  "scopes": ["read:batches", "write:batches"],
  "expiresIn": "365d"
}

# Response
{
  "apiKey": "wia_live_abc123...",
  "keyId": "key-001",
  "created": "2025-01-15T10:00:00Z",
  "expiresAt": "2026-01-15T10:00:00Z"
}
```

### 2.4 Device Token Authentication

For IoT devices with limited resources:

```http
POST /v1/auth/device-token
Content-Type: application/json

{
  "deviceId": "IOT-DEV-001",
  "deviceSecret": "device_secret_here"
}

# Response
{
  "deviceToken": "dt_abc123...",
  "expiresIn": 86400
}

# Using device token
POST /v1/batches
Authorization: DeviceToken dt_abc123...
```

---

## API Endpoints

### 3.1 Batch Management

#### 3.1.1 Create Plastic Batch

```http
POST /v1/batches
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "collection": {
    "location": {
      "gps": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "accuracy": 5.0
      },
      "description": "Haeundae Beach, Busan"
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "collector": {
      "deviceId": "IOT-DEV-001",
      "organizationId": "ORG-CLEANUP-001"
    },
    "method": "beach_cleanup"
  },
  "material": {
    "totalWeight": {
      "value": 25.8,
      "unit": "kg"
    },
    "composition": [
      {
        "resinCode": "PET",
        "percentage": 50,
        "weight": {"value": 12.9, "unit": "kg"},
        "condition": "good"
      },
      {
        "resinCode": "PP",
        "percentage": 50,
        "weight": {"value": 12.9, "unit": "kg"},
        "condition": "good"
      }
    ]
  }
}

# Response: 201 Created
{
  "batchId": "BATCH-2025-000001",
  "status": "collected",
  "created": "2025-01-15T10:30:00Z",
  "qrCode": "https://api.wia.live/qr/BATCH-2025-000001",
  "trackingUrl": "https://track.wia.live/batch/BATCH-2025-000001"
}
```

#### 3.1.2 Get Batch Details

```http
GET /v1/batches/{batchId}
Authorization: Bearer YOUR_TOKEN

# Response: 200 OK
{
  "batchId": "BATCH-2025-000001",
  "status": "collected",
  "created": "2025-01-15T10:30:00Z",
  "collection": { ... },
  "material": { ... },
  "recycling": { ... },
  "environmental": { ... }
}
```

#### 3.1.3 Update Batch Status

```http
PUT /v1/batches/{batchId}/status
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "status": "transported",
  "notes": "Transported to sorting facility",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "timestamp": "2025-01-16T08:00:00Z"
}

# Response: 200 OK
{
  "batchId": "BATCH-2025-000001",
  "status": "transported",
  "updatedAt": "2025-01-16T08:00:00Z"
}
```

#### 3.1.4 List Batches

```http
GET /v1/batches?
  status=collected&
  startDate=2025-01-01&
  endDate=2025-01-31&
  organizationId=ORG-CLEANUP-001&
  limit=50&
  offset=0
Authorization: Bearer YOUR_TOKEN

# Response: 200 OK
{
  "data": [
    {
      "batchId": "BATCH-2025-000001",
      "status": "collected",
      "created": "2025-01-15T10:30:00Z",
      "totalWeight": {"value": 25.8, "unit": "kg"}
    }
  ],
  "pagination": {
    "total": 150,
    "limit": 50,
    "offset": 0,
    "hasMore": true
  }
}
```

#### 3.1.5 Delete Batch

```http
DELETE /v1/batches/{batchId}
Authorization: Bearer YOUR_TOKEN

# Response: 204 No Content
```

### 3.2 IoT Device Management

#### 3.2.1 Register Device

```http
POST /v1/devices
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "deviceId": "IOT-DEV-001",
  "deviceType": "vessel_tracker",
  "organizationId": "ORG-CLEANUP-001",
  "sensors": ["gps", "weight", "environmental"],
  "metadata": {
    "vesselName": "Ocean Guardian",
    "maxCapacity": 500
  }
}

# Response: 201 Created
{
  "deviceId": "IOT-DEV-001",
  "deviceSecret": "generated_secret_here",
  "registered": "2025-01-15T10:00:00Z"
}
```

#### 3.2.2 Send Sensor Data

```http
POST /v1/devices/{deviceId}/sensors
Authorization: DeviceToken dt_abc123...
Content-Type: application/json

{
  "timestamp": "2025-01-15T10:30:00Z",
  "sensors": [
    {
      "type": "gps",
      "data": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "speed": 15,
        "heading": 180
      }
    },
    {
      "type": "weight",
      "data": {
        "currentWeight": 45.5,
        "percentFull": 9.1
      }
    }
  ]
}

# Response: 202 Accepted
{
  "received": "2025-01-15T10:30:01Z",
  "processed": true
}
```

#### 3.2.3 Get Device Status

```http
GET /v1/devices/{deviceId}/status
Authorization: Bearer YOUR_TOKEN

# Response: 200 OK
{
  "deviceId": "IOT-DEV-001",
  "status": "active",
  "lastPing": "2025-01-15T10:30:00Z",
  "battery": {
    "level": 85,
    "estimatedHours": 48
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "firmware": "v2.1.0"
}
```

### 3.3 Recycling Facility Management

#### 3.3.1 Register Facility

```http
POST /v1/facilities
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "name": "Seoul Plastic Sorting Center",
  "type": "sorting",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "address": "123 Recycling St, Seoul, Korea"
  },
  "certifications": ["ISO-14001", "GREEN-CERT-KR"],
  "capacity": {
    "daily": {"value": 1000, "unit": "kg"}
  }
}

# Response: 201 Created
{
  "facilityId": "FAC-SORT-001",
  "created": "2025-01-15T10:00:00Z"
}
```

#### 3.3.2 Receive Batch at Facility

```http
POST /v1/facilities/{facilityId}/receive
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "batchId": "BATCH-2025-000001",
  "arrivalTime": "2025-01-16T08:00:00Z",
  "inspectionNotes": "Material quality: good"
}

# Response: 200 OK
{
  "received": true,
  "nextStage": "sorting",
  "estimatedProcessingTime": "2025-01-17T08:00:00Z"
}
```

### 3.4 Analytics & Reporting

#### 3.4.1 Get Collection Statistics

```http
GET /v1/analytics/collection?
  startDate=2025-01-01&
  endDate=2025-01-31&
  groupBy=day&
  organizationId=ORG-CLEANUP-001
Authorization: Bearer YOUR_TOKEN

# Response: 200 OK
{
  "period": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-01-31T23:59:59Z"
  },
  "statistics": {
    "totalBatches": 150,
    "totalWeight": {"value": 3875.5, "unit": "kg"},
    "byResinType": {
      "PET": {"batches": 90, "weight": {"value": 1550.2, "unit": "kg"}},
      "HDPE": {"batches": 45, "weight": {"value": 1162.6, "unit": "kg"}},
      "PP": {"batches": 60, "weight": {"value": 775.3, "unit": "kg"}}
    },
    "byMethod": {
      "beach_cleanup": 100,
      "vessel_net": 40,
      "underwater_collection": 10
    }
  },
  "daily": [
    {
      "date": "2025-01-01",
      "batches": 5,
      "weight": {"value": 129.2, "unit": "kg"}
    }
  ]
}
```

#### 3.4.2 Get Environmental Impact

```http
GET /v1/analytics/environmental?
  startDate=2025-01-01&
  endDate=2025-01-31
Authorization: Bearer YOUR_TOKEN

# Response: 200 OK
{
  "period": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-01-31T23:59:59Z"
  },
  "impact": {
    "totalPlasticCollected": {"value": 3875.5, "unit": "kg"},
    "marineLifeSaved": {
      "estimatedAnimals": 1250,
      "speciesTypes": ["fish", "turtle", "seabird", "whale"]
    },
    "carbonFootprint": {
      "collection": {"value": 450.0, "unit": "kg_co2"},
      "transportation": {"value": 320.0, "unit": "kg_co2"},
      "processing": {"value": 180.0, "unit": "kg_co2"},
      "total": {"value": 950.0, "unit": "kg_co2"},
      "netImpact": {"value": -3100.0, "unit": "kg_co2"}
    },
    "oceanCleanupArea": {"value": 125000, "unit": "m2"},
    "virginPlasticAvoided": {"value": 3100.2, "unit": "kg"}
  }
}
```

#### 3.4.3 Generate Report

```http
POST /v1/reports/generate
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "type": "monthly_impact",
  "period": {
    "start": "2025-01-01",
    "end": "2025-01-31"
  },
  "organizationId": "ORG-CLEANUP-001",
  "format": "pdf",
  "includeCharts": true
}

# Response: 202 Accepted
{
  "reportId": "RPT-2025-001",
  "status": "processing",
  "estimatedCompletion": "2025-01-15T10:35:00Z"
}

# Get Report Status
GET /v1/reports/{reportId}

# Response: 200 OK
{
  "reportId": "RPT-2025-001",
  "status": "completed",
  "downloadUrl": "https://api.wia.live/reports/RPT-2025-001/download",
  "expiresAt": "2025-01-22T10:30:00Z"
}
```

### 3.5 Search & Query

#### 3.5.1 Search Batches by Location

```http
POST /v1/search/batches/geo
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "center": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "radius": 50,
  "unit": "km",
  "filters": {
    "status": ["collected", "transported"],
    "dateRange": {
      "start": "2025-01-01",
      "end": "2025-01-31"
    }
  }
}

# Response: 200 OK
{
  "results": [
    {
      "batchId": "BATCH-2025-000001",
      "distance": {"value": 5.2, "unit": "km"},
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780
      }
    }
  ],
  "total": 25
}
```

#### 3.5.2 Track Batch Journey

```http
GET /v1/batches/{batchId}/journey
Authorization: Bearer YOUR_TOKEN

# Response: 200 OK
{
  "batchId": "BATCH-2025-000001",
  "journey": [
    {
      "stage": "collected",
      "timestamp": "2025-01-15T10:30:00Z",
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "description": "Haeundae Beach"
      },
      "verifiedBy": "ORG-CLEANUP-001"
    },
    {
      "stage": "transported",
      "timestamp": "2025-01-16T08:00:00Z",
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "description": "Seoul Sorting Center"
      },
      "distance": {"value": 50, "unit": "km"}
    },
    {
      "stage": "sorting",
      "timestamp": "2025-01-16T10:00:00Z",
      "facility": "FAC-SORT-001"
    }
  ],
  "currentStage": "sorting"
}
```

### 3.6 Notifications

#### 3.6.1 Subscribe to Notifications

```http
POST /v1/notifications/subscribe
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json

{
  "events": [
    "batch.created",
    "batch.status_changed",
    "device.offline"
  ],
  "endpoint": "https://yourapp.com/webhooks",
  "secret": "your_webhook_secret"
}

# Response: 201 Created
{
  "subscriptionId": "SUB-001",
  "created": "2025-01-15T10:00:00Z"
}
```

---

## Request/Response Format

### 4.1 Standard Request Headers

```http
Authorization: Bearer YOUR_TOKEN
Content-Type: application/json
Accept: application/json
X-Request-ID: unique-request-id
X-API-Version: 1.0.0
```

### 4.2 Standard Response Format

#### Success Response

```json
{
  "success": true,
  "data": { ... },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "1.0.0"
  }
}
```

#### Error Response

```json
{
  "success": false,
  "error": {
    "code": "ERR_INVALID_BATCH",
    "message": "Invalid batch format",
    "details": {
      "field": "collection.location.gps",
      "reason": "GPS coordinates required"
    }
  },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 4.3 Pagination

```json
{
  "data": [ ... ],
  "pagination": {
    "total": 150,
    "limit": 50,
    "offset": 0,
    "hasMore": true,
    "nextOffset": 50,
    "prevOffset": null
  }
}
```

---

## Error Handling

### 5.1 HTTP Status Codes

| Code | Name | Description |
|------|------|-------------|
| 200 | OK | Request succeeded |
| 201 | Created | Resource created |
| 202 | Accepted | Async processing started |
| 204 | No Content | Successful deletion |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict |
| 422 | Unprocessable Entity | Validation failed |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Service temporarily down |

### 5.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `ERR_INVALID_TOKEN` | 401 | Invalid or expired token |
| `ERR_INSUFFICIENT_SCOPE` | 403 | Missing required scope |
| `ERR_BATCH_NOT_FOUND` | 404 | Batch ID not found |
| `ERR_INVALID_GPS` | 422 | Invalid GPS coordinates |
| `ERR_WEIGHT_MISMATCH` | 422 | Weight calculation error |
| `ERR_DEVICE_OFFLINE` | 503 | IoT device offline |
| `ERR_RATE_LIMIT` | 429 | Rate limit exceeded |

---

## Rate Limiting

### 6.1 Rate Limit Tiers

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 1,000 | 100 |
| Standard | 10,000 | 500 |
| Premium | 100,000 | 2,000 |
| Enterprise | Unlimited | Custom |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9950
X-RateLimit-Reset: 1642253400
X-RateLimit-Tier: standard
```

### 6.3 Rate Limit Exceeded Response

```json
{
  "success": false,
  "error": {
    "code": "ERR_RATE_LIMIT",
    "message": "Rate limit exceeded",
    "details": {
      "limit": 10000,
      "resetAt": "2025-01-15T11:00:00Z"
    }
  }
}
```

---

## SDK Examples

### 7.1 Python SDK

```python
from wia_ocean_plastic import OceanPlasticClient

# Initialize client
client = OceanPlasticClient(
    api_key='your_api_key_here',
    environment='production'
)

# Create a batch
batch = client.batches.create(
    collection={
        'location': {
            'gps': {
                'latitude': 37.5665,
                'longitude': 126.9780,
                'accuracy': 5.0
            },
            'description': 'Haeundae Beach, Busan'
        },
        'timestamp': '2025-01-15T10:30:00Z',
        'collector': {
            'device_id': 'IOT-DEV-001',
            'organization_id': 'ORG-CLEANUP-001'
        },
        'method': 'beach_cleanup'
    },
    material={
        'total_weight': {'value': 25.8, 'unit': 'kg'},
        'composition': [
            {
                'resin_code': 'PET',
                'percentage': 50,
                'weight': {'value': 12.9, 'unit': 'kg'},
                'condition': 'good'
            }
        ]
    }
)

print(f"Created batch: {batch.batch_id}")

# Get batch details
batch = client.batches.get('BATCH-2025-000001')
print(f"Status: {batch.status}")

# Update batch status
batch.update_status('transported', notes='Moved to facility')

# List batches with filters
batches = client.batches.list(
    status='collected',
    start_date='2025-01-01',
    limit=50
)

for batch in batches:
    print(f"{batch.batch_id}: {batch.total_weight}")

# Send IoT sensor data
client.devices.send_sensor_data(
    device_id='IOT-DEV-001',
    sensors=[
        {
            'type': 'gps',
            'data': {
                'latitude': 37.5665,
                'longitude': 126.9780
            }
        }
    ]
)

# Get analytics
stats = client.analytics.collection_statistics(
    start_date='2025-01-01',
    end_date='2025-01-31',
    organization_id='ORG-CLEANUP-001'
)

print(f"Total collected: {stats.total_weight}")
```

### 7.2 TypeScript SDK

```typescript
import { OceanPlasticClient } from '@wia/ocean-plastic-track';

// Initialize client
const client = new OceanPlasticClient({
  apiKey: 'your_api_key_here',
  environment: 'production'
});

// Create a batch
const batch = await client.batches.create({
  collection: {
    location: {
      gps: {
        latitude: 37.5665,
        longitude: 126.9780,
        accuracy: 5.0
      },
      description: 'Haeundae Beach, Busan'
    },
    timestamp: '2025-01-15T10:30:00Z',
    collector: {
      deviceId: 'IOT-DEV-001',
      organizationId: 'ORG-CLEANUP-001'
    },
    method: 'beach_cleanup'
  },
  material: {
    totalWeight: { value: 25.8, unit: 'kg' },
    composition: [
      {
        resinCode: 'PET',
        percentage: 50,
        weight: { value: 12.9, unit: 'kg' },
        condition: 'good'
      }
    ]
  }
});

console.log(`Created batch: ${batch.batchId}`);

// Get batch details
const batchDetails = await client.batches.get('BATCH-2025-000001');
console.log(`Status: ${batchDetails.status}`);

// Update batch status
await batchDetails.updateStatus('transported', {
  notes: 'Moved to facility'
});

// List batches with filters
const batches = await client.batches.list({
  status: 'collected',
  startDate: '2025-01-01',
  limit: 50
});

batches.data.forEach(batch => {
  console.log(`${batch.batchId}: ${batch.totalWeight.value}kg`);
});

// Subscribe to real-time updates
client.batches.subscribe('BATCH-2025-000001', (update) => {
  console.log(`Status changed to: ${update.status}`);
});

// Get analytics
const stats = await client.analytics.collectionStatistics({
  startDate: '2025-01-01',
  endDate: '2025-01-31',
  organizationId: 'ORG-CLEANUP-001'
});

console.log(`Total collected: ${stats.totalWeight.value}kg`);
```

### 7.3 JavaScript (Node.js) SDK

```javascript
const { OceanPlasticClient } = require('@wia/ocean-plastic-track');

// Initialize client
const client = new OceanPlasticClient({
  apiKey: process.env.WIA_API_KEY
});

// Create batch
async function createBatch() {
  try {
    const batch = await client.batches.create({
      collection: {
        location: {
          gps: { latitude: 37.5665, longitude: 126.9780 }
        },
        timestamp: new Date().toISOString(),
        collector: {
          deviceId: 'IOT-DEV-001'
        },
        method: 'beach_cleanup'
      },
      material: {
        totalWeight: { value: 25.8, unit: 'kg' },
        composition: [
          { resinCode: 'PET', percentage: 100, weight: { value: 25.8, unit: 'kg' } }
        ]
      }
    });

    console.log('Batch created:', batch.batchId);
    return batch;
  } catch (error) {
    console.error('Error creating batch:', error.message);
  }
}

createBatch();
```

### 7.4 cURL Examples

```bash
# Create a batch
curl -X POST https://api.wia.live/ocean-plastic-track/v1/batches \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "collection": {
      "location": {
        "gps": {"latitude": 37.5665, "longitude": 126.9780}
      },
      "timestamp": "2025-01-15T10:30:00Z",
      "collector": {"deviceId": "IOT-DEV-001"},
      "method": "beach_cleanup"
    },
    "material": {
      "totalWeight": {"value": 25.8, "unit": "kg"},
      "composition": [
        {"resinCode": "PET", "percentage": 100, "weight": {"value": 25.8, "unit": "kg"}}
      ]
    }
  }'

# Get batch details
curl -X GET https://api.wia.live/ocean-plastic-track/v1/batches/BATCH-2025-000001 \
  -H "Authorization: Bearer YOUR_TOKEN"

# Update batch status
curl -X PUT https://api.wia.live/ocean-plastic-track/v1/batches/BATCH-2025-000001/status \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "status": "transported",
    "timestamp": "2025-01-16T08:00:00Z"
  }'
```

---

## Webhooks

### 8.1 Webhook Events

| Event | Description | Payload |
|-------|-------------|---------|
| `batch.created` | New batch created | Full batch object |
| `batch.status_changed` | Batch status updated | Batch with new status |
| `batch.completed` | Batch recycling completed | Final batch state |
| `device.online` | IoT device came online | Device status |
| `device.offline` | IoT device went offline | Device status |
| `facility.received` | Facility received batch | Batch + facility info |

### 8.2 Webhook Payload

```json
{
  "event": "batch.status_changed",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "batchId": "BATCH-2025-000001",
    "previousStatus": "collected",
    "currentStatus": "transported",
    "updatedAt": "2025-01-15T10:30:00Z"
  },
  "signature": "sha256=abc123..."
}
```

### 8.3 Webhook Verification

```python
import hmac
import hashlib

def verify_webhook(payload, signature, secret):
    expected = 'sha256=' + hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(expected, signature)

# Usage
if verify_webhook(request.body, request.headers['X-Signature'], webhook_secret):
    # Process webhook
    pass
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Ocean Plastic Track Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
