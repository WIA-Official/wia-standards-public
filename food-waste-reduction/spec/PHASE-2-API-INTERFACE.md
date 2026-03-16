# WIA-AGRI-032: Food Waste Reduction Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines RESTful API endpoints, WebSocket interfaces, and webhook mechanisms for food waste tracking, inventory management, donation coordination, and analytics.

### 1.1 API Design Principles

- **RESTful**: Standard HTTP methods (GET, POST, PUT, DELETE, PATCH)
- **JSON First**: All request/response bodies use JSON
- **Versioned**: URL-based versioning (`/api/v1/...`)
- **Stateless**: Each request contains all necessary information
- **Idempotent**: Safe to retry operations
- **Rate Limited**: Protect against abuse

### 1.2 Base URL

```
Production: https://api.wia-foodwaste.org/v1
Sandbox:    https://sandbox-api.wia-foodwaste.org/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

All API requests require an API key in the header:

```http
GET /api/v1/inventory/items HTTP/1.1
Host: api.wia-foodwaste.org
Authorization: Bearer wia_live_xxxxxxxxxxxxxxxxxxxxx
Content-Type: application/json
```

### 2.2 OAuth 2.0

For user-delegated access:

```http
POST /oauth/token HTTP/1.1
Host: api.wia-foodwaste.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=inventory:read inventory:write waste:track
```

Response:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "inventory:read inventory:write waste:track"
}
```

---

## 3. Core API Endpoints

### 3.1 Food Item Tracking

#### Track New Food Item

```http
POST /api/v1/inventory/items
```

**Request Body:**
```json
{
  "facilityId": "FACILITY-2025-001",
  "product": {
    "name": "Fresh Milk",
    "sku": "DAIRY-MILK-001",
    "barcode": "012345678901",
    "category": "dairy",
    "quantity": 100,
    "unit": "kg"
  },
  "dates": {
    "received": "2025-12-26T08:00:00.000Z",
    "expiration": "2025-12-31T23:59:59.000Z"
  },
  "economics": {
    "costPerUnit": 3.50,
    "currency": "USD"
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "trackingId": "TRACK-20251226-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "status": "fresh",
  "daysUntilExpiry": 5,
  "alerts": [],
  "recommendations": [
    "Monitor daily - expires in 5 days",
    "Consider promotion at 3 days remaining"
  ]
}
```

#### Get Food Item Status

```http
GET /api/v1/inventory/items/{trackingId}
```

**Response:** `200 OK`
```json
{
  "trackingId": "TRACK-20251226-001",
  "product": {
    "name": "Fresh Milk",
    "quantity": 100,
    "unit": "kg"
  },
  "status": "fresh",
  "daysUntilExpiry": 5,
  "expirationDate": "2025-12-31T23:59:59.000Z",
  "totalValue": 350.00,
  "lastUpdated": "2025-12-26T10:30:00.000Z"
}
```

#### Update Food Item Status

```http
PATCH /api/v1/inventory/items/{trackingId}
```

**Request Body:**
```json
{
  "status": "near_expiry",
  "quantity": 80,
  "notes": "Marked down for quick sale"
}
```

**Response:** `200 OK`
```json
{
  "success": true,
  "trackingId": "TRACK-20251226-001",
  "status": "near_expiry",
  "quantity": 80,
  "updatedAt": "2025-12-26T12:00:00.000Z"
}
```

#### Query Inventory

```http
GET /api/v1/inventory/items?status=near_expiry&category=dairy&limit=50
```

**Query Parameters:**
- `status`: Filter by status (fresh, near_expiry, expired, etc.)
- `category`: Filter by product category
- `facilityId`: Filter by facility
- `expiresWithin`: Days until expiration (e.g., `3` for items expiring within 3 days)
- `limit`: Maximum results (default: 100, max: 1000)
- `offset`: Pagination offset
- `sortBy`: Sort field (expiration, value, quantity)
- `sortOrder`: asc or desc

**Response:** `200 OK`
```json
{
  "total": 215,
  "limit": 50,
  "offset": 0,
  "items": [
    {
      "trackingId": "TRACK-20251226-001",
      "product": {
        "name": "Fresh Milk",
        "category": "dairy"
      },
      "status": "near_expiry",
      "daysUntilExpiry": 2,
      "quantity": 80,
      "totalValue": 280.00
    }
  ],
  "summary": {
    "totalValue": 6250.00,
    "totalItems": 215,
    "estimatedLoss": 6250.00
  }
}
```

### 3.2 Waste Tracking

#### Record Waste Event

```http
POST /api/v1/waste/track
```

**Request Body:**
```json
{
  "facilityId": "FACILITY-2025-001",
  "wasteType": "expired",
  "products": [
    {
      "trackingId": "TRACK-20251220-005",
      "name": "Fresh Milk",
      "quantity": 20,
      "unit": "kg",
      "reason": "passed_expiration_date"
    }
  ],
  "disposal": {
    "method": "composted",
    "disposalDate": "2025-12-26T14:00:00.000Z"
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "eventId": "WASTE-20251226-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "totals": {
    "totalWeight": 20,
    "totalValue": 70.00,
    "co2Emissions": 40
  },
  "analysis": {
    "preventable": true,
    "rootCause": "over_ordering",
    "recommendations": [
      "Reduce order quantity by 15%",
      "Improve demand forecasting"
    ]
  }
}
```

#### Get Waste Metrics

```http
GET /api/v1/waste/metrics?facilityId=FACILITY-2025-001&period=30days
```

**Response:** `200 OK`
```json
{
  "period": "30 days",
  "startDate": "2025-11-26",
  "endDate": "2025-12-26",
  "totals": {
    "wasteEvents": 47,
    "totalWeight": 450,
    "weightUnit": "kg",
    "totalValue": 1575.00,
    "currency": "USD"
  },
  "breakdown": {
    "byCategory": [
      {
        "category": "dairy",
        "weight": 120,
        "value": 420.00,
        "percentage": 26.7
      },
      {
        "category": "produce",
        "weight": 200,
        "value": 600.00,
        "percentage": 44.4
      }
    ],
    "byReason": [
      {
        "reason": "expired",
        "weight": 280,
        "value": 980.00,
        "percentage": 62.2
      },
      {
        "reason": "spoiled",
        "weight": 170,
        "value": 595.00,
        "percentage": 37.8
      }
    ]
  },
  "environmental": {
    "co2Emissions": 890,
    "co2Unit": "kg",
    "waterWaste": 45000,
    "waterUnit": "liters"
  },
  "trends": {
    "changeVsPreviousPeriod": -12.5,
    "trend": "improving"
  }
}
```

### 3.3 Donation Management

#### Create Donation

```http
POST /api/v1/donations/create
```

**Request Body:**
```json
{
  "donor": {
    "facilityId": "FACILITY-2025-001",
    "contactName": "John Smith",
    "contactEmail": "john@grocery.com"
  },
  "recipient": {
    "organizationId": "FOODBANK-2025-001",
    "name": "City Food Bank"
  },
  "products": [
    {
      "trackingId": "TRACK-20251226-002",
      "name": "Fresh Bread",
      "quantity": 50,
      "unit": "loaves"
    }
  ],
  "logistics": {
    "pickupScheduled": "2025-12-26T16:00:00.000Z",
    "transportMethod": "recipient_pickup"
  }
}
```

**Response:** `201 Created`
```json
{
  "success": true,
  "donationId": "DONATE-20251226-001",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "totals": {
    "totalWeight": 25,
    "totalValue": 150.00,
    "estimatedMeals": 120
  },
  "logistics": {
    "pickupScheduled": "2025-12-26T16:00:00.000Z",
    "confirmationCode": "PICKUP-ABC123"
  },
  "taxReceipt": {
    "receiptId": "RECEIPT-20251226-001",
    "deductibleAmount": 150.00
  }
}
```

#### Get Donation History

```http
GET /api/v1/donations?facilityId=FACILITY-2025-001&startDate=2025-12-01
```

**Response:** `200 OK`
```json
{
  "total": 23,
  "donations": [
    {
      "donationId": "DONATE-20251226-001",
      "timestamp": "2025-12-26T10:30:00.000Z",
      "recipient": "City Food Bank",
      "totalValue": 150.00,
      "estimatedMeals": 120,
      "status": "scheduled"
    }
  ],
  "summary": {
    "totalDonations": 23,
    "totalValue": 3450.00,
    "totalMeals": 2760,
    "peopleServed": 920
  }
}
```

#### Find Nearby Food Banks

```http
GET /api/v1/donations/nearby?latitude=37.7749&longitude=-122.4194&radius=10
```

**Response:** `200 OK`
```json
{
  "total": 5,
  "foodBanks": [
    {
      "organizationId": "FOODBANK-2025-001",
      "name": "City Food Bank",
      "distance": 2.3,
      "distanceUnit": "km",
      "acceptsCategories": ["dairy", "produce", "bakery", "meat"],
      "pickupAvailable": true,
      "operatingHours": "Mon-Sat 8AM-6PM",
      "contact": {
        "phone": "+1-555-0456",
        "email": "donations@foodbank.org"
      },
      "capacity": {
        "currentCapacity": "high",
        "preferredItems": ["dairy", "produce"]
      }
    }
  ]
}
```

### 3.4 Analytics & Reporting

#### Get Impact Dashboard

```http
GET /api/v1/analytics/impact?facilityId=FACILITY-2025-001&period=90days
```

**Response:** `200 OK`
```json
{
  "period": "90 days",
  "wasteReduction": {
    "currentPeriod": {
      "weight": 450,
      "value": 1575.00
    },
    "previousPeriod": {
      "weight": 612,
      "value": 2142.00
    },
    "improvement": {
      "weight": -26.5,
      "value": -26.5,
      "percentage": -26.5
    }
  },
  "donations": {
    "totalDonations": 67,
    "totalValue": 9870.00,
    "mealsProvided": 7920,
    "peopleServed": 2640
  },
  "costSavings": {
    "wasteAvoided": 567.00,
    "taxDeductions": 9870.00,
    "totalSavings": 10437.00
  },
  "environmental": {
    "co2Prevented": 1340,
    "waterSaved": 134000,
    "landfillDiverted": 1120
  },
  "roi": {
    "investmentInSystem": 5000.00,
    "totalSavings": 10437.00,
    "roi": 108.7,
    "roiPercentage": "108.7%"
  }
}
```

---

## 4. WebSocket Real-Time Interface

### 4.1 Connection

```javascript
const ws = new WebSocket('wss://api.wia-foodwaste.org/v1/stream?token=wia_live_xxx');

ws.onopen = () => {
  // Subscribe to events
  ws.send(JSON.stringify({
    action: 'subscribe',
    topics: ['expiration_alerts', 'waste_events', 'donations']
  }));
};
```

### 4.2 Event Types

#### Expiration Alert

```json
{
  "event": "expiration_alert",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "trackingId": "TRACK-20251226-001",
  "product": {
    "name": "Fresh Milk",
    "quantity": 80
  },
  "daysUntilExpiry": 1,
  "severity": "high",
  "recommendations": [
    "Mark down 30% for immediate sale",
    "Contact food bank for donation"
  ]
}
```

#### Waste Event

```json
{
  "event": "waste_recorded",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "eventId": "WASTE-20251226-001",
  "wasteType": "expired",
  "totalValue": 70.00,
  "preventable": true
}
```

#### Donation Confirmed

```json
{
  "event": "donation_confirmed",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "donationId": "DONATE-20251226-001",
  "recipient": "City Food Bank",
  "pickupTime": "2025-12-26T16:00:00.000Z",
  "estimatedMeals": 120
}
```

---

## 5. Webhook Notifications

### 5.1 Webhook Configuration

```http
POST /api/v1/webhooks
```

**Request Body:**
```json
{
  "url": "https://your-server.com/webhooks/wia",
  "events": ["expiration_alert", "waste_event", "donation_confirmed"],
  "secret": "your_webhook_secret"
}
```

**Response:** `201 Created`
```json
{
  "webhookId": "WEBHOOK-001",
  "url": "https://your-server.com/webhooks/wia",
  "events": ["expiration_alert", "waste_event", "donation_confirmed"],
  "active": true,
  "createdAt": "2025-12-26T10:30:00.000Z"
}
```

### 5.2 Webhook Payload

```http
POST /webhooks/wia HTTP/1.1
Host: your-server.com
Content-Type: application/json
X-WIA-Signature: sha256=xxxxxxxxxxxxx
X-WIA-Event: expiration_alert

{
  "webhookId": "WEBHOOK-001",
  "event": "expiration_alert",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "data": {
    "trackingId": "TRACK-20251226-001",
    "product": {
      "name": "Fresh Milk"
    },
    "daysUntilExpiry": 1
  }
}
```

---

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid product category",
    "details": {
      "field": "product.category",
      "value": "invalid_category",
      "allowedValues": ["dairy", "produce", "meat", "bakery", "prepared"]
    },
    "timestamp": "2025-12-26T10:30:00.000Z",
    "requestId": "req_xxxxxxxxxxxxx"
  }
}
```

### 6.2 HTTP Status Codes

- `200 OK`: Successful GET/PATCH request
- `201 Created`: Successful POST request
- `400 Bad Request`: Invalid request data
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary service disruption

---

## 7. Rate Limiting

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640528400
```

**Limits:**
- Free Tier: 100 requests/hour
- Standard: 1,000 requests/hour
- Premium: 10,000 requests/hour
- Enterprise: Custom limits

---

**Next Phase:** [Phase 3: Protocol](./PHASE-3-PROTOCOL.md)

---

© 2025 WIA Standards · MIT License
弘益人間 · Benefit All Humanity
