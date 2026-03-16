# WIA-AGRI-014: Agricultural Supply Chain Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines REST API endpoints for agricultural supply chain management, including shipment tracking, cold chain monitoring, and provenance verification.

### 1.1 Base URL

```
Production: https://api.wia-supply-chain.org/v1
Staging: https://staging-api.wia-supply-chain.org/v1
```

### 1.2 Authentication

All API requests require authentication using Bearer tokens:

```http
Authorization: Bearer <your_access_token>
```

---

## 2. Shipment Management APIs

### 2.1 Create Shipment

**Endpoint:** `POST /shipments`

**Request:**
```json
{
  "productInfo": {
    "productName": "Organic Tomatoes",
    "productType": "FRESH_PRODUCE",
    "quantity": 500,
    "unit": "kg",
    "harvestDate": "2025-01-01"
  },
  "origin": {
    "farmId": "FARM-KR-001",
    "location": {
      "latitude": 33.4996,
      "longitude": 126.5312
    }
  },
  "destination": {
    "facilityId": "RETAIL-001",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "coldChainRequired": true,
  "targetTemperature": {
    "min": 2,
    "max": 8
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "shipmentId": "SHIP-2025-001",
    "status": "PENDING",
    "estimatedArrival": "2025-01-01T18:00:00Z",
    "trackingUrl": "https://track.wia-supply-chain.org/SHIP-2025-001"
  }
}
```

### 2.2 Get Shipment Status

**Endpoint:** `GET /shipments/{shipmentId}`

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "shipmentId": "SHIP-2025-001",
    "status": "IN_TRANSIT",
    "currentLocation": {
      "latitude": 37.1234,
      "longitude": 127.5678,
      "address": "Highway 1, 120km from destination"
    },
    "coldChain": {
      "temperature": 4.2,
      "humidity": 75,
      "status": "NORMAL"
    },
    "estimatedArrival": "2025-01-01T18:00:00Z",
    "timeline": [
      {
        "stage": "ORIGIN",
        "location": "Green Valley Farm",
        "timestamp": "2025-01-01T10:00:00Z",
        "completed": true
      },
      {
        "stage": "IN_TRANSIT",
        "location": "Highway 1",
        "timestamp": "2025-01-01T12:00:00Z",
        "completed": false
      }
    ]
  }
}
```

### 2.3 Update Shipment Status

**Endpoint:** `PATCH /shipments/{shipmentId}`

**Request:**
```json
{
  "status": "DELIVERED",
  "deliveryNotes": "Received in good condition",
  "receivedBy": "John Store Manager",
  "timestamp": "2025-01-01T18:00:00Z"
}
```

**Response:** `200 OK`

### 2.4 List Shipments

**Endpoint:** `GET /shipments?status=IN_TRANSIT&origin=FARM-KR-001&limit=20`

**Query Parameters:**
- `status`: Filter by shipment status
- `origin`: Filter by origin farm ID
- `destination`: Filter by destination facility ID
- `startDate`: Filter by shipment start date
- `endDate`: Filter by shipment end date
- `limit`: Number of results (default 50, max 100)
- `offset`: Pagination offset

**Response:** `200 OK`

---

## 3. Cold Chain Monitoring APIs

### 3.1 Submit Sensor Data

**Endpoint:** `POST /cold-chain/data`

**Request:**
```json
{
  "shipmentId": "SHIP-2025-001",
  "deviceId": "SENSOR-TEMP-001",
  "timestamp": "2025-01-01T12:30:45Z",
  "temperature": 4.2,
  "humidity": 75,
  "location": {
    "latitude": 37.1234,
    "longitude": 127.5678
  }
}
```

**Response:** `201 Created`

### 3.2 Get Cold Chain History

**Endpoint:** `GET /cold-chain/history/{shipmentId}?interval=5min`

**Query Parameters:**
- `interval`: Data aggregation interval (1min, 5min, 15min, 1hour)
- `startTime`: Start of time range
- `endTime`: End of time range

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "shipmentId": "SHIP-2025-001",
    "dataPoints": [
      {
        "timestamp": "2025-01-01T12:00:00Z",
        "temperature": 4.1,
        "humidity": 74,
        "status": "NORMAL"
      },
      {
        "timestamp": "2025-01-01T12:05:00Z",
        "temperature": 4.2,
        "humidity": 75,
        "status": "NORMAL"
      }
    ],
    "statistics": {
      "avgTemperature": 4.15,
      "minTemperature": 3.8,
      "maxTemperature": 4.5,
      "violations": 0
    }
  }
}
```

### 3.3 Get Cold Chain Alerts

**Endpoint:** `GET /cold-chain/alerts?shipmentId=SHIP-2025-001&severity=HIGH`

**Response:** `200 OK`

---

## 4. Provenance & Blockchain APIs

### 4.1 Record Provenance Event

**Endpoint:** `POST /provenance/events`

**Request:**
```json
{
  "productId": "PROD-TOMATO-001",
  "stage": "HARVEST",
  "location": "Green Valley Farm",
  "operator": "Kim Farmer",
  "timestamp": "2025-01-01T08:00:00Z",
  "metadata": {
    "weatherConditions": "Sunny, 22°C",
    "harvestMethod": "Manual picking"
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "data": {
    "recordId": "PROV-2025-001",
    "blockchainTx": {
      "transactionId": "TX-0x1234567890abcdef",
      "blockNumber": 12345,
      "confirmed": true
    }
  }
}
```

### 4.2 Verify Provenance

**Endpoint:** `GET /provenance/verify/{productId}`

**Response:** `200 OK`
```json
{
  "success": true,
  "data": {
    "productId": "PROD-TOMATO-001",
    "verified": true,
    "traceability": [
      {
        "stage": "HARVEST",
        "timestamp": "2025-01-01T08:00:00Z",
        "verified": true,
        "blockchainTx": "TX-0x123..."
      },
      {
        "stage": "PROCESSING",
        "timestamp": "2025-01-01T09:00:00Z",
        "verified": true,
        "blockchainTx": "TX-0x456..."
      }
    ],
    "certifications": ["ORGANIC", "GAP"],
    "blockchainHash": "0xabcdef1234567890"
  }
}
```

---

## 5. Quality Control APIs

### 5.1 Submit Quality Check

**Endpoint:** `POST /quality-control/checks`

**Request:**
```json
{
  "shipmentId": "SHIP-2025-001",
  "stage": "PRE_SHIPMENT",
  "inspector": {
    "name": "Lee Inspector",
    "licenseNumber": "QC-KR-2025-001"
  },
  "visualInspection": {
    "appearance": "EXCELLENT",
    "defects": 0
  },
  "laboratoryTests": [
    {
      "testType": "PESTICIDE_RESIDUE",
      "result": "NEGATIVE"
    }
  ],
  "passed": true
}
```

**Response:** `201 Created`

### 5.2 Get Quality History

**Endpoint:** `GET /quality-control/history/{shipmentId}`

**Response:** `200 OK`

---

## 6. Integration APIs

### 6.1 Warehouse Management Integration

**Endpoint:** `POST /integrations/wms/inbound`

**Request:**
```json
{
  "shipmentId": "SHIP-2025-001",
  "warehouseId": "WH-001",
  "expectedArrival": "2025-01-01T14:00:00Z",
  "storageLocation": "Bay A-12",
  "temperatureZone": "REFRIGERATED_2_8C"
}
```

### 6.2 Retailer POS Integration

**Endpoint:** `POST /integrations/retail/pos`

**Request:**
```json
{
  "shipmentId": "SHIP-2025-001",
  "retailerId": "RETAIL-001",
  "receivedQuantity": 500,
  "condition": "EXCELLENT",
  "shelfLife": 6
}
```

---

## 7. Error Responses

### 7.1 Error Format

```json
{
  "success": false,
  "error": {
    "code": "SHIPMENT_NOT_FOUND",
    "message": "Shipment with ID SHIP-2025-999 not found",
    "details": {
      "requestId": "REQ-123456",
      "timestamp": "2025-01-01T12:00:00Z"
    }
  }
}
```

### 7.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `UNAUTHORIZED` | 401 | Invalid or missing authentication token |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `SHIPMENT_NOT_FOUND` | 404 | Shipment ID does not exist |
| `VALIDATION_ERROR` | 400 | Request data validation failed |
| `COLD_CHAIN_VIOLATION` | 400 | Temperature/humidity out of range |
| `BLOCKCHAIN_ERROR` | 500 | Blockchain transaction failed |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |

---

## 8. Rate Limiting

- **Standard tier:** 100 requests/minute
- **Premium tier:** 1000 requests/minute
- **Enterprise tier:** Unlimited

Rate limit headers:
```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1609459200
```

---

## 9. Webhooks

### 9.1 Register Webhook

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": ["shipment.status_changed", "cold_chain.alert", "quality.failed"],
  "secret": "your_webhook_secret"
}
```

### 9.2 Webhook Events

**Event:** `shipment.status_changed`
```json
{
  "event": "shipment.status_changed",
  "timestamp": "2025-01-01T12:00:00Z",
  "data": {
    "shipmentId": "SHIP-2025-001",
    "oldStatus": "IN_TRANSIT",
    "newStatus": "DELIVERED"
  }
}
```

---

**Next Phase:** [Phase 3: Protocol](PHASE-3-PROTOCOL.md)

---

**弘益人間 (Benefit All Humanity)**
*WIA - World Certification Industry Association*
*© 2025 MIT License*
