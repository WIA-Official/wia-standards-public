# WIA-UNI-008 - Phase 2: API Interface

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

## 1. Overview

Phase 2 of the WIA-UNI-008 Transportation Network Standard defines RESTful API interfaces for transportation route management, booking systems, real-time tracking, and logistics operations. The API enables seamless integration between different transportation operators and platforms across the Korean Peninsula.

### 1.1 API Principles

- **RESTful design** - Standard HTTP methods (GET, POST, PUT, DELETE)
- **JSON-LD responses** - All responses follow Phase 1 data formats
- **OAuth 2.0 authentication** - Secure token-based authentication
- **Rate limiting** - Fair usage policies to ensure service availability
- **Versioning** - API version in URL path for backward compatibility

### 1.2 Base URL

```
Production:  https://api.transportation.wia/{region}/v1/
Sandbox:     https://sandbox-api.transportation.wia/{region}/v1/
```

**Regions:** `seoul`, `pyongyang`, `international`

## 2. Authentication

### 2.1 OAuth 2.0 Flow

```bash
# 1. Request access token
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=routes:read bookings:write tracking:read

# Response
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "routes:read bookings:write tracking:read"
}

# 2. Use access token in API requests
GET /v1/routes
Authorization: Bearer YOUR_ACCESS_TOKEN
```

### 2.2 API Scopes

| Scope | Description |
|-------|-------------|
| `routes:read` | Read transportation routes |
| `routes:write` | Create and modify routes |
| `bookings:read` | Read booking information |
| `bookings:write` | Create and modify bookings |
| `tracking:read` | Access real-time tracking data |
| `cargo:read` | Read cargo shipment information |
| `cargo:write` | Create and update cargo shipments |
| `admin:all` | Full administrative access |

## 3. API Endpoints

### 3.1 Routes API

#### List Routes

```http
GET /v1/routes
Authorization: Bearer {token}

Query Parameters:
  - type: string (high-speed-rail, conventional-rail, highway, etc.)
  - origin: string (station/airport code)
  - destination: string (station/airport code)
  - operator: string (operator code)
  - page: number (default: 1)
  - limit: number (default: 20, max: 100)

Response: 200 OK
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "totalResults": 25,
  "page": 1,
  "limit": 20,
  "routes": [
    {
      "@type": "TransportationRoute",
      "id": "wia:uni:route:tkr-001",
      "name": "Trans-Korean Express",
      ...
    }
  ]
}
```

#### Get Route Details

```http
GET /v1/routes/{routeId}
Authorization: Bearer {token}

Response: 200 OK
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationRoute",
  "id": "wia:uni:route:tkr-001",
  "routeType": "high-speed-rail",
  "name": "Trans-Korean Express",
  ...
}
```

#### Create Route

```http
POST /v1/routes
Authorization: Bearer {token}
Content-Type: application/json

{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationRoute",
  "routeType": "high-speed-rail",
  "name": "Seoul-Pyongyang Express",
  "operator": {
    "name": "Korea Railway Corporation",
    "operatorCode": "KORAIL",
    "country": "KR"
  },
  ...
}

Response: 201 Created
Location: /v1/routes/wia:uni:route:new-route-id
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationRoute",
  "id": "wia:uni:route:new-route-id",
  ...
}
```

#### Update Route

```http
PUT /v1/routes/{routeId}
Authorization: Bearer {token}
Content-Type: application/json

{
  "schedule": {
    "frequency": "daily",
    "departureTimes": ["06:00", "09:00", "13:00", "17:00", "20:00"]
  }
}

Response: 200 OK
```

#### Delete Route

```http
DELETE /v1/routes/{routeId}
Authorization: Bearer {token}

Response: 204 No Content
```

### 3.2 Bookings API

#### Create Booking

```http
POST /v1/bookings
Authorization: Bearer {token}
Content-Type: application/json

{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationBooking",
  "passenger": {
    "name": "Kim Minjun",
    "passportNumber": "M12345678",
    "nationality": "KR",
    "contactInfo": {
      "email": "minjun.kim@example.com",
      "phone": "+82-10-1234-5678"
    }
  },
  "journey": {
    "routeId": "wia:uni:route:tkr-001",
    "origin": "SEL",
    "destination": "PYO",
    "departureDate": "2026-06-15",
    "departureTime": "09:00",
    "class": "business"
  }
}

Response: 201 Created
Location: /v1/bookings/{bookingId}
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationBooking",
  "id": "wia:uni:booking:abc123",
  "bookingReference": "TKRXYZ789",
  "status": "confirmed",
  "pricing": {
    "totalAmount": 120.00,
    "currency": "USD"
  },
  ...
}
```

#### Get Booking

```http
GET /v1/bookings/{bookingId}
Authorization: Bearer {token}

Response: 200 OK
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationBooking",
  "id": "wia:uni:booking:abc123",
  "bookingReference": "TKRXYZ789",
  ...
}
```

#### Cancel Booking

```http
DELETE /v1/bookings/{bookingId}
Authorization: Bearer {token}

Response: 200 OK
{
  "status": "cancelled",
  "refund": {
    "amount": 100.00,
    "currency": "USD",
    "method": "original-payment-method",
    "estimatedDays": 7
  }
}
```

#### Search Availability

```http
GET /v1/bookings/availability
Authorization: Bearer {token}

Query Parameters:
  - origin: string (required)
  - destination: string (required)
  - date: ISO 8601 date (required)
  - passengers: number (default: 1)
  - class: string (economy|business|first)

Response: 200 OK
{
  "searchDate": "2026-06-15",
  "origin": "SEL",
  "destination": "PYO",
  "available": [
    {
      "routeId": "wia:uni:route:tkr-001",
      "departureTime": "09:00",
      "arrivalTime": "10:30",
      "price": {
        "economy": 50.00,
        "business": 120.00,
        "first": 200.00
      },
      "seatsAvailable": {
        "economy": 85,
        "business": 12,
        "first": 4
      }
    }
  ]
}
```

### 3.3 Tracking API

#### Track Vehicle

```http
GET /v1/tracking/vehicles/{vehicleId}
Authorization: Bearer {token}

Response: 200 OK
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "VehicleTracking",
  "vehicleId": "TKR-2025-001",
  "timestamp": "2025-12-25T09:15:30Z",
  "location": {
    "latitude": 37.8234,
    "longitude": 126.7123,
    "heading": 358,
    "accuracy": 5
  },
  "speed": {
    "value": 285,
    "unit": "km/h"
  },
  "status": "in-transit",
  "nextStop": {
    "name": "Kaesong Station",
    "code": "KAE",
    "estimatedArrival": "2025-12-25T09:45:00Z",
    "distance": 23.4
  },
  "onTimePerformance": "98%"
}
```

#### Track Booking

```http
GET /v1/tracking/bookings/{bookingId}
Authorization: Bearer {token}

Response: 200 OK
{
  "bookingReference": "TKRXYZ789",
  "status": "in-transit",
  "vehicle": {
    "vehicleId": "TKR-2025-001",
    "currentLocation": {
      "latitude": 37.8234,
      "longitude": 126.7123,
      "timestamp": "2025-12-25T09:15:30Z"
    }
  },
  "journey": {
    "origin": "SEL",
    "destination": "PYO",
    "progress": "45%",
    "estimatedArrival": "2025-12-25T10:30:00Z"
  }
}
```

#### Subscribe to Tracking Updates (WebSocket)

```javascript
const ws = new WebSocket('wss://api.transportation.wia/v1/tracking/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    vehicleId: 'TKR-2025-001',
    token: 'YOUR_ACCESS_TOKEN'
  }));
};

ws.onmessage = (event) => {
  const update = JSON.parse(event.data);
  console.log('Location update:', update);
};
```

### 3.4 Cargo API

#### Create Cargo Shipment

```http
POST /v1/cargo
Authorization: Bearer {token}
Content-Type: application/json

{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "CargoShipment",
  "shipper": {
    "name": "Seoul Trading Co.",
    "address": { ... },
    "contact": { ... }
  },
  "consignee": {
    "name": "Pyongyang Import Co.",
    "address": { ... },
    "contact": { ... }
  },
  "cargo": {
    "description": "Electronic components",
    "type": "container",
    "weight": { "value": 15000, "unit": "kg" },
    "containerNumber": "CONT123456"
  },
  "route": {
    "origin": "Busan Port",
    "destination": "Nampo Port"
  }
}

Response: 201 Created
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "CargoShipment",
  "id": "wia:uni:cargo:xyz789",
  "trackingNumber": "CARGO2025XYZ",
  "status": "booked",
  ...
}
```

#### Track Cargo

```http
GET /v1/cargo/{trackingNumber}
Authorization: Bearer {token}

Response: 200 OK
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "CargoShipment",
  "trackingNumber": "CARGO2025XYZ",
  "status": "in-transit",
  "route": {
    "currentLocation": {
      "name": "Seoul Logistics Hub",
      "coordinates": { "latitude": 37.5665, "longitude": 126.9780 },
      "timestamp": "2025-12-25T09:30:00Z"
    },
    "segments": [
      {
        "mode": "road",
        "from": "Busan Port",
        "to": "Seoul Logistics Hub",
        "status": "completed"
      },
      {
        "mode": "rail",
        "from": "Seoul Logistics Hub",
        "to": "Nampo Port",
        "status": "in-progress",
        "estimatedArrival": "2025-12-26T15:00:00Z"
      }
    ]
  },
  "customs": {
    "status": "cleared",
    "declarationNumber": "CUST2025-ABC"
  }
}
```

## 4. Webhooks

### 4.1 Webhook Registration

```http
POST /v1/webhooks
Authorization: Bearer {token}
Content-Type: application/json

{
  "url": "https://your-server.com/webhooks/wia-transportation",
  "events": ["booking.confirmed", "booking.cancelled", "vehicle.delayed"],
  "secret": "your-webhook-secret"
}

Response: 201 Created
{
  "id": "webhook-123",
  "url": "https://your-server.com/webhooks/wia-transportation",
  "events": ["booking.confirmed", "booking.cancelled", "vehicle.delayed"],
  "active": true
}
```

### 4.2 Webhook Events

| Event | Description |
|-------|-------------|
| `booking.confirmed` | Booking successfully confirmed |
| `booking.cancelled` | Booking cancelled |
| `booking.modified` | Booking details modified |
| `vehicle.delayed` | Vehicle experiencing delay |
| `vehicle.emergency` | Emergency situation |
| `cargo.status_change` | Cargo status updated |
| `customs.cleared` | Customs clearance completed |

### 4.3 Webhook Payload

```json
{
  "event": "booking.confirmed",
  "timestamp": "2025-12-25T09:00:00Z",
  "data": {
    "@context": "https://wiastandards.com/contexts/uni-008/v1",
    "@type": "TransportationBooking",
    "id": "wia:uni:booking:abc123",
    "bookingReference": "TKRXYZ789",
    ...
  },
  "signature": "sha256=..."
}
```

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Missing required field: passenger.passportNumber",
    "details": {
      "field": "passenger.passportNumber",
      "requirement": "Must be a valid passport number"
    },
    "timestamp": "2025-12-25T09:00:00Z",
    "requestId": "req-abc123"
  }
}
```

### 5.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request or missing required fields |
| `UNAUTHORIZED` | 401 | Invalid or missing authentication token |
| `FORBIDDEN` | 403 | Insufficient permissions for requested operation |
| `NOT_FOUND` | 404 | Requested resource not found |
| `CONFLICT` | 409 | Resource already exists or conflict |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `SERVER_ERROR` | 500 | Internal server error |
| `SERVICE_UNAVAILABLE` | 503 | Service temporarily unavailable |

## 6. Rate Limiting

### 6.1 Rate Limits

| Tier | Requests/Hour | Requests/Day |
|------|---------------|--------------|
| **Standard** | 1,000 | 10,000 |
| **Premium** | 10,000 | 100,000 |
| **Enterprise** | Custom | Custom |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1640419200
```

## 7. Pagination

```http
GET /v1/routes?page=2&limit=50

Response:
{
  "totalResults": 250,
  "page": 2,
  "limit": 50,
  "totalPages": 5,
  "routes": [ ... ],
  "links": {
    "first": "/v1/routes?page=1&limit=50",
    "prev": "/v1/routes?page=1&limit=50",
    "next": "/v1/routes?page=3&limit=50",
    "last": "/v1/routes?page=5&limit=50"
  }
}
```

## 8. API Versioning

- **URL versioning:** `/v1/`, `/v2/`
- **Version lifecycle:** 12 months support after new version release
- **Deprecation notice:** 6 months before end-of-life

## 9. SDK Examples

### 9.1 TypeScript

```typescript
import { WIATransportationSDK } from '@wia/transportation-network';

const sdk = new WIATransportationSDK({
  baseURL: 'https://api.transportation.wia/seoul',
  accessToken: 'YOUR_ACCESS_TOKEN'
});

// Create booking
const booking = await sdk.createBooking({
  passenger: { ... },
  journey: { ... }
});

// Track vehicle
const tracking = await sdk.trackVehicle('TKR-2025-001');
```

## 10. References

- **OAuth 2.0:** https://oauth.net/2/
- **REST API Design:** https://restfulapi.net/
- **JSON-LD:** https://www.w3.org/TR/json-ld11/
- **WebSocket:** https://tools.ietf.org/html/rfc6455

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
