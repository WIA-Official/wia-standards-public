# Chapter 5: API Interface Specification

---

## 5.1 Overview

The WIA-IND-009 standard defines a comprehensive REST API with WebSocket support for real-time features. This chapter provides complete API specifications including endpoints, request/response formats, authentication, and error handling.

### API Design Principles

1. **RESTful**: Standard HTTP methods (GET, POST, PATCH, DELETE)
2. **JSON**: All data in JSON format
3. **Versioned**: URL-based versioning (/v1/)
4. **Stateless**: Each request contains all necessary information
5. **HATEOAS**: Hypermedia links for discoverability
6. **Idempotent**: Safe to retry requests

---

## 5.2 Base Configuration

### 5.2.1 Base URL

```
Production: https://api.wia-ind-009.com/v1
Sandbox: https://sandbox.wia-ind-009.com/v1
```

### 5.2.2 Request Headers

```
Required Headers:
  Authorization: Bearer <jwt_token>
  Content-Type: application/json
  Accept: application/json

Optional Headers:
  X-Request-ID: <uuid>           (for tracing)
  X-Client-Version: <version>    (app version)
  Accept-Language: en-US         (localization)
```

### 5.2.3 Response Format

**Success Response:**
```json
{
  "data": {
    ...response data...
  },
  "meta": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T18:00:00Z",
    "version": "1.0.0"
  }
}
```

**Error Response:**
```json
{
  "error": {
    "code": "INVALID_INPUT",
    "message": "Delivery location is required",
    "details": {
      "field": "deliveryLocation",
      "constraint": "required"
    }
  },
  "meta": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T18:00:00Z"
  }
}
```

---

## 5.3 Authentication

### 5.3.1 Login

**Endpoint:** `POST /auth/login`

**Request:**
```json
{
  "email": "customer@example.com",
  "password": "secure_password_123"
}
```

**Response:**
```json
{
  "data": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "refreshToken": "refresh_abc123xyz",
    "expiresIn": 86400,
    "user": {
      "id": "user_123",
      "email": "customer@example.com",
      "role": "customer",
      "name": "Jane Doe"
    }
  }
}
```

### 5.3.2 Token Refresh

**Endpoint:** `POST /auth/refresh`

**Request:**
```json
{
  "refreshToken": "refresh_abc123xyz"
}
```

**Response:**
```json
{
  "data": {
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "expiresIn": 86400
  }
}
```

### 5.3.3 JWT Structure

```json
{
  "sub": "user_123",
  "role": "customer",
  "permissions": [
    "order:create",
    "order:read",
    "order:update"
  ],
  "exp": 1737057600,
  "iat": 1736971200
}
```

---

## 5.4 Order Endpoints

### 5.4.1 Create Order

**Endpoint:** `POST /orders`

**Request:**
```json
{
  "restaurantId": "rest_789",
  "items": [
    {
      "itemId": "item_pizza_001",
      "quantity": 1,
      "modifiers": [
        {"id": "mod_extra_cheese", "quantity": 1}
      ]
    }
  ],
  "deliveryLocation": {
    "latitude": 37.7858,
    "longitude": -122.4068,
    "address": "456 Mission St, Apt 12B, San Francisco, CA 94105",
    "addressLine2": "Apt 12B",
    "deliveryInstructions": "Ring doorbell"
  },
  "paymentMethodId": "pm_card_visa_1234",
  "contactlessDelivery": true,
  "specialInstructions": "Extra napkins please",
  "scheduledFor": "2025-01-15T19:00:00Z",
  "tip": 500
}
```

**Response:** `201 Created`
```json
{
  "data": {
    "id": "order_123",
    "confirmationCode": "ABC123",
    "status": "pending",
    "items": [...],
    "subtotal": 2598,
    "tax": 234,
    "deliveryFee": 499,
    "serviceFee": 390,
    "tip": 500,
    "total": 4221,
    "estimatedDelivery": "2025-01-15T19:35:00Z",
    "createdAt": "2025-01-15T18:00:00Z"
  },
  "links": {
    "self": "/orders/order_123",
    "tracking": "/orders/order_123/tracking",
    "cancel": "/orders/order_123"
  }
}
```

### 5.4.2 Get Order

**Endpoint:** `GET /orders/:orderId`

**Response:** `200 OK`
```json
{
  "data": {
    "id": "order_123",
    "confirmationCode": "ABC123",
    "status": "in_transit",
    "restaurant": {
      "id": "rest_789",
      "name": "Luigi's Pizza",
      "phone": "+14155551234",
      "address": "123 Market St, San Francisco, CA"
    },
    "driver": {
      "id": "drv_321",
      "name": "John D.",
      "phone": "+14155555678",
      "photo": "https://cdn.example.com/drivers/drv_321.jpg",
      "rating": 4.87,
      "vehicleType": "ebike"
    },
    "items": [...],
    "subtotal": 2598,
    "total": 4221,
    "estimatedDelivery": "2025-01-15T19:35:00Z",
    "actualDelivery": null,
    "statusHistory": [...]
  }
}
```

### 5.4.3 List Orders

**Endpoint:** `GET /orders`

**Query Parameters:**
```
?status=in_transit          Filter by status
&restaurant=rest_789        Filter by restaurant
&customer=cust_456          Filter by customer
&from=2025-01-01            Date range start
&to=2025-01-31              Date range end
&limit=20                   Results per page (default: 20, max: 100)
&offset=0                   Pagination offset
&sort=-createdAt            Sort field (- for descending)
```

**Response:** `200 OK`
```json
{
  "data": [
    {...order 1...},
    {...order 2...}
  ],
  "meta": {
    "total": 150,
    "limit": 20,
    "offset": 0,
    "hasMore": true
  },
  "links": {
    "self": "/orders?limit=20&offset=0",
    "next": "/orders?limit=20&offset=20",
    "prev": null
  }
}
```

### 5.4.4 Update Order

**Endpoint:** `PATCH /orders/:orderId`

**Request:**
```json
{
  "specialInstructions": "Please leave at door",
  "tip": 700
}
```

**Response:** `200 OK`
```json
{
  "data": {
    "id": "order_123",
    "specialInstructions": "Please leave at door",
    "tip": 700,
    "total": 4421,
    "updatedAt": "2025-01-15T18:15:00Z"
  }
}
```

### 5.4.5 Cancel Order

**Endpoint:** `DELETE /orders/:orderId`

**Request:**
```json
{
  "reason": "Changed my mind",
  "cancelledBy": "customer"
}
```

**Response:** `200 OK`
```json
{
  "data": {
    "id": "order_123",
    "status": "cancelled",
    "cancelledAt": "2025-01-15T18:20:00Z",
    "refundAmount": 4221,
    "refundStatus": "pending"
  }
}
```

### 5.4.6 Get Order Tracking

**Endpoint:** `GET /orders/:orderId/tracking`

**Response:** `200 OK`
```json
{
  "data": {
    "orderId": "order_123",
    "status": "in_transit",
    "driver": {
      "location": {
        "latitude": 37.7820,
        "longitude": -122.4100,
        "heading": 45,
        "speed": 22,
        "accuracy": 10,
        "timestamp": "2025-01-15T18:30:00Z"
      }
    },
    "route": {
      "currentStop": 2,
      "totalStops": 3,
      "distanceRemaining": 1.5,
      "estimatedArrival": "2025-01-15T18:40:00Z"
    },
    "temperature": {
      "current": 62.5,
      "target": 60,
      "inCompliance": true,
      "lastReading": "2025-01-15T18:30:00Z"
    }
  }
}
```

### 5.4.7 Get Temperature History

**Endpoint:** `GET /orders/:orderId/temperature`

**Response:** `200 OK`
```json
{
  "data": {
    "orderId": "order_123",
    "readings": [
      {
        "timestamp": "2025-01-15T18:20:00Z",
        "temperature": 65.2,
        "humidity": 45,
        "inCompliance": true
      },
      {
        "timestamp": "2025-01-15T18:21:00Z",
        "temperature": 63.8,
        "humidity": 46,
        "inCompliance": true
      }
    ],
    "summary": {
      "avgTemperature": 63.5,
      "minTemperature": 60.1,
      "maxTemperature": 65.2,
      "complianceRate": 1.0,
      "alerts": []
    }
  }
}
```

---

## 5.5 Driver Endpoints

### 5.5.1 Register Driver

**Endpoint:** `POST /drivers`

**Request:**
```json
{
  "firstName": "John",
  "lastName": "Doe",
  "email": "john.doe@example.com",
  "phone": "+14155551234",
  "dateOfBirth": "1990-05-15",
  "vehicleType": "ebike",
  "vehicleMake": "Rad Power",
  "vehicleModel": "RadRunner",
  "vehicleYear": 2024,
  "licensePlate": "ABC1234",
  "licenseNumber": "D1234567",
  "licenseExpiration": "2028-05-15"
}
```

**Response:** `201 Created`
```json
{
  "data": {
    "id": "drv_321",
    "status": "pending_verification",
    "backgroundCheckStatus": "pending",
    "joinedAt": "2025-01-15T18:00:00Z"
  }
}
```

### 5.5.2 Get Driver Profile

**Endpoint:** `GET /drivers/:driverId`

**Response:** `200 OK`
```json
{
  "data": {
    "id": "drv_321",
    "firstName": "John",
    "lastName": "Doe",
    "rating": 4.87,
    "totalDeliveries": 1523,
    "completionRate": 0.984,
    "onTimeRate": 0.923,
    "vehicleType": "ebike",
    "equipment": {
      "hasHotBag": true,
      "hasColdBag": true,
      "hasTemperatureSensor": true
    },
    "tier": "gold"
  }
}
```

### 5.5.3 Update Driver Location

**Endpoint:** `POST /drivers/:driverId/location`

**Request:**
```json
{
  "latitude": 37.7820,
  "longitude": -122.4100,
  "accuracy": 10,
  "heading": 45,
  "speed": 22,
  "timestamp": "2025-01-15T18:30:00Z"
}
```

**Response:** `200 OK`
```json
{
  "data": {
    "driverId": "drv_321",
    "location": {
      "latitude": 37.7820,
      "longitude": -122.4100,
      "timestamp": "2025-01-15T18:30:00Z"
    },
    "nearbyOrders": [
      {
        "orderId": "order_456",
        "distance": 0.8,
        "estimatedEarnings": 12.50
      }
    ]
  }
}
```

### 5.5.4 Get Driver Metrics

**Endpoint:** `GET /drivers/:driverId/metrics`

**Query Parameters:**
```
?period=week                today|week|month|all
&from=2025-01-01
&to=2025-01-31
```

**Response:** `200 OK`
```json
{
  "data": {
    "period": {
      "start": "2025-01-08T00:00:00Z",
      "end": "2025-01-15T23:59:59Z"
    },
    "efficiency": {
      "ordersPerHour": 2.8,
      "avgDeliveryTime": 23,
      "avgDistancePerOrder": 3.2,
      "utilizationRate": 0.75
    },
    "quality": {
      "onTimeDeliveryRate": 0.923,
      "customerRating": 4.87,
      "orderAccuracy": 0.995,
      "temperatureCompliance": 0.978
    },
    "earnings": {
      "total": 85320,
      "deliveryFees": 65000,
      "tips": 18000,
      "bonuses": 2320,
      "avgPerHour": 24.50,
      "avgPerDelivery": 9.20
    }
  }
}
```

---

## 5.6 Route Endpoints

### 5.6.1 Calculate Single Route

**Endpoint:** `POST /routes/calculate`

**Request:**
```json
{
  "pickup": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "delivery": {
    "latitude": 37.7858,
    "longitude": -122.4068
  },
  "vehicleType": "ebike",
  "departureTime": "2025-01-15T18:30:00Z"
}
```

**Response:** `200 OK`
```json
{
  "data": {
    "distance": 2.3,
    "duration": 12,
    "eta": "2025-01-15T18:42:00Z",
    "waypoints": [
      {"latitude": 37.7749, "longitude": -122.4194},
      {"latitude": 37.7780, "longitude": -122.4150},
      {"latitude": 37.7858, "longitude": -122.4068}
    ],
    "encodedPolyline": "a~l~Fjk~uOwHJy@P",
    "instructions": [
      "Head north on Market St",
      "Turn right on Mission St",
      "Arrive at destination"
    ],
    "trafficConditions": "moderate"
  }
}
```

### 5.6.2 Optimize Multi-Stop Route

**Endpoint:** `POST /routes/optimize`

**Request:**
```json
{
  "driverId": "drv_321",
  "orders": [
    {
      "orderId": "order_123",
      "pickup": {"latitude": 37.7749, "longitude": -122.4194},
      "delivery": {"latitude": 37.7858, "longitude": -122.4068},
      "deliveryWindow": {
        "start": "2025-01-15T18:30:00Z",
        "end": "2025-01-15T19:00:00Z"
      }
    },
    {
      "orderId": "order_124",
      "pickup": {"latitude": 37.7755, "longitude": -122.4185},
      "delivery": {"latitude": 37.7865, "longitude": -122.4055},
      "deliveryWindow": {
        "start": "2025-01-15T18:45:00Z",
        "end": "2025-01-15T19:15:00Z"
      }
    }
  ],
  "constraints": {
    "maxDuration": 60,
    "maxDistance": 15
  }
}
```

**Response:** `200 OK`
```json
{
  "data": {
    "routeId": "route_555",
    "stops": [
      {
        "sequence": 1,
        "type": "pickup",
        "orderId": "order_123",
        "location": {...},
        "estimatedArrival": "2025-01-15T18:35:00Z",
        "serviceDuration": 5
      },
      {
        "sequence": 2,
        "type": "pickup",
        "orderId": "order_124",
        "location": {...},
        "estimatedArrival": "2025-01-15T18:42:00Z",
        "serviceDuration": 5
      },
      {
        "sequence": 3,
        "type": "delivery",
        "orderId": "order_123",
        "location": {...},
        "estimatedArrival": "2025-01-15T18:55:00Z",
        "serviceDuration": 3
      },
      {
        "sequence": 4,
        "type": "delivery",
        "orderId": "order_124",
        "location": {...},
        "estimatedArrival": "2025-01-15T19:00:00Z",
        "serviceDuration": 3
      }
    ],
    "totalDistance": 4.8,
    "totalDuration": 25,
    "optimizationTime": 342,
    "algorithm": "2-opt",
    "allConstraintsMet": true
  }
}
```

---

## 5.7 WebSocket Real-Time Tracking

### 5.7.1 Connection

**URL:** `wss://ws.wia-ind-009.com/tracking`

**Authentication:**
```javascript
const socket = new WebSocket('wss://ws.wia-ind-009.com/tracking');

socket.onopen = () => {
  socket.send(JSON.stringify({
    action: 'authenticate',
    token: 'Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};
```

### 5.7.2 Subscribe to Order Updates

**Client → Server:**
```json
{
  "action": "subscribe",
  "orderId": "order_123",
  "userId": "user_456"
}
```

**Server → Client (Confirmation):**
```json
{
  "event": "subscribed",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:30:00Z"
}
```

### 5.7.3 Location Updates

**Server → Client (every 10-30 seconds):**
```json
{
  "event": "location_update",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:30:15Z",
  "data": {
    "driver": {
      "location": {
        "latitude": 37.7820,
        "longitude": -122.4100,
        "heading": 45,
        "speed": 22
      }
    },
    "eta": "2025-01-15T18:45:00Z",
    "distanceRemaining": 1.8,
    "status": "in_transit"
  }
}
```

### 5.7.4 Status Updates

**Server → Client:**
```json
{
  "event": "status_changed",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:35:00Z",
  "data": {
    "oldStatus": "assigned",
    "newStatus": "picked_up",
    "message": "Your driver has picked up your order"
  }
}
```

### 5.7.5 Temperature Alerts

**Server → Client:**
```json
{
  "event": "temperature_alert",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:32:00Z",
  "data": {
    "level": "warning",
    "temperature": 57.5,
    "threshold": 60,
    "message": "Food temperature is approaching lower limit"
  }
}
```

### 5.7.6 ETA Updates

**Server → Client (every 2 minutes during transit):**
```json
{
  "event": "eta_updated",
  "orderId": "order_123",
  "timestamp": "2025-01-15T18:36:00Z",
  "data": {
    "previousEta": "2025-01-15T18:45:00Z",
    "currentEta": "2025-01-15T18:42:00Z",
    "reason": "traffic_improved",
    "confidence": 0.95
  }
}
```

---

## 5.8 Webhook Notifications

### 5.8.1 Webhook Registration

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-server.com/webhooks/wia-ind-009",
  "events": [
    "order.created",
    "order.confirmed",
    "order.delivered",
    "order.cancelled",
    "order.temperature_alert"
  ],
  "secret": "your_webhook_secret_key"
}
```

**Response:** `201 Created`
```json
{
  "data": {
    "webhookId": "webhook_999",
    "url": "https://your-server.com/webhooks/wia-ind-009",
    "events": [...],
    "status": "active",
    "createdAt": "2025-01-15T18:00:00Z"
  }
}
```

### 5.8.2 Webhook Payload

**POST to your server:**
```json
{
  "webhookId": "webhook_999",
  "event": "order.delivered",
  "timestamp": "2025-01-15T19:00:00Z",
  "data": {
    "orderId": "order_123",
    "status": "delivered",
    "deliveryTime": "2025-01-15T19:00:00Z",
    "proofOfDelivery": {
      "photo": "https://cdn.example.com/proof_123.jpg",
      "signature": null,
      "location": {
        "latitude": 37.7858,
        "longitude": -122.4068
      }
    }
  },
  "signature": "sha256=5d41402abc4b2a76b9719d911017c592"
}
```

**Headers:**
```
X-WIA-Signature: sha256=5d41402abc4b2a76b9719d911017c592
X-WIA-Event: order.delivered
X-WIA-Timestamp: 1737057600
```

### 5.8.3 Webhook Verification

```python
import hmac
import hashlib

def verify_webhook(payload, signature, secret):
    """Verify webhook signature"""

    expected_signature = hmac.new(
        secret.encode(),
        payload.encode(),
        hashlib.sha256
    ).hexdigest()

    return hmac.compare_digest(
        f"sha256={expected_signature}",
        signature
    )
```

---

## 5.9 Error Handling

### 5.9.1 HTTP Status Codes

```
200 OK                  Success
201 Created             Resource created
204 No Content          Success with no response body
400 Bad Request         Invalid input
401 Unauthorized        Missing or invalid authentication
403 Forbidden           Valid auth but insufficient permissions
404 Not Found           Resource doesn't exist
409 Conflict            Resource already exists or state conflict
422 Unprocessable       Valid input but business logic failure
429 Too Many Requests   Rate limit exceeded
500 Internal Error      Server error
503 Service Unavailable Temporary outage
```

### 5.9.2 Error Codes

```typescript
enum ErrorCode {
  // Authentication
  INVALID_TOKEN = 'INVALID_TOKEN',
  EXPIRED_TOKEN = 'EXPIRED_TOKEN',
  INSUFFICIENT_PERMISSIONS = 'INSUFFICIENT_PERMISSIONS',

  // Validation
  INVALID_INPUT = 'INVALID_INPUT',
  MISSING_FIELD = 'MISSING_FIELD',
  INVALID_FORMAT = 'INVALID_FORMAT',
  OUT_OF_RANGE = 'OUT_OF_RANGE',

  // Business Logic
  RESTAURANT_UNAVAILABLE = 'RESTAURANT_UNAVAILABLE',
  OUT_OF_DELIVERY_RANGE = 'OUT_OF_DELIVERY_RANGE',
  NO_DRIVERS_AVAILABLE = 'NO_DRIVERS_AVAILABLE',
  PAYMENT_FAILED = 'PAYMENT_FAILED',

  // Resources
  ORDER_NOT_FOUND = 'ORDER_NOT_FOUND',
  DRIVER_NOT_FOUND = 'DRIVER_NOT_FOUND',
  CANNOT_CANCEL = 'CANNOT_CANCEL',

  // System
  INTERNAL_ERROR = 'INTERNAL_ERROR',
  SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE',
  RATE_LIMIT_EXCEEDED = 'RATE_LIMIT_EXCEEDED'
}
```

### 5.9.3 Example Error Response

```json
{
  "error": {
    "code": "OUT_OF_DELIVERY_RANGE",
    "message": "Delivery address is outside service area",
    "details": {
      "address": "456 Mission St, San Francisco, CA",
      "distance": 25.5,
      "maxDistance": 15,
      "unit": "km"
    },
    "hint": "Try a closer restaurant or contact support"
  },
  "meta": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T18:00:00Z",
    "documentation": "https://docs.wia-ind-009.com/errors/OUT_OF_DELIVERY_RANGE"
  }
}
```

---

## 5.10 Rate Limiting

### 5.10.1 Rate Limits

```
Tier              Requests/Min    Burst    WebSocket Connections
─────────────────────────────────────────────────────────────────
Standard          100             200      10
Premium           1,000           2,000    50
Enterprise        10,000          20,000   500
```

### 5.10.2 Rate Limit Headers

**Response Headers:**
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1737057600
Retry-After: 60
```

### 5.10.3 Rate Limit Exceeded Response

**HTTP 429:**
```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Too many requests. Please try again in 60 seconds.",
    "details": {
      "limit": 100,
      "window": "minute",
      "retryAfter": 60
    }
  }
}
```

---

## 5.11 Summary

The WIA-IND-009 API provides:

- **RESTful Endpoints**: Complete CRUD operations for orders, drivers, routes
- **WebSocket**: Real-time tracking with sub-second updates
- **Webhooks**: Event-driven notifications
- **Authentication**: JWT-based secure access
- **Rate Limiting**: Tiered limits with burst allowance
- **Error Handling**: Comprehensive error codes and messages

---

**Next Chapter**: [Chapter 6: Protocols and Algorithms →](06-protocol.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
