# WIA-IND-009: PHASE 2 - API INTERFACE SPECIFICATION
## Food Delivery Platform Standard
### Version 1.0 | 弘益人間 (Benefit All Humanity)

---

## Table of Contents
1. [Overview](#overview)
2. [Authentication & Authorization](#authentication--authorization)
3. [Order Management APIs](#order-management-apis)
4. [Menu Management APIs](#menu-management-apis)
5. [Driver Management APIs](#driver-management-apis)
6. [Tracking APIs](#tracking-apis)
7. [Restaurant APIs](#restaurant-apis)
8. [Customer APIs](#customer-apis)
9. [Payment APIs](#payment-apis)
10. [Error Handling](#error-handling)

---

## Overview

Phase 2 defines RESTful HTTP APIs for food delivery platform operations. All APIs follow REST principles with JSON payloads.

**Base URL Format:** `https://api.{platform-domain}/v1/`

**Design Principles:**
- RESTful architecture with resource-oriented URLs
- Stateless communication
- Standard HTTP methods (GET, POST, PUT, PATCH, DELETE)
- JSON request/response bodies
- OAuth 2.0 authentication
- Rate limiting and throttling
- Comprehensive error responses

### HTTP Methods

- `GET`: Retrieve resources
- `POST`: Create new resources
- `PUT`: Replace entire resource
- `PATCH`: Partial resource update
- `DELETE`: Remove resource

### Standard Headers

**Request Headers:**
```
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-Request-ID: {unique-request-id}
X-API-Version: 1.0
```

**Response Headers:**
```
Content-Type: application/json
X-Request-ID: {echo-request-id}
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640000000
```

---

## Authentication & Authorization

### OAuth 2.0 Flow

WIA-IND-009 uses OAuth 2.0 for authentication with the following grant types:

**1. Authorization Code Grant** (for customer applications)
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
client_secret={client_secret}
```

**2. Client Credentials Grant** (for server-to-server)
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id={client_id}&
client_secret={client_secret}&
scope={requested_scopes}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def50200...",
  "scope": "orders:read orders:write"
}
```

### Scopes

- `orders:read` - Read order data
- `orders:write` - Create and update orders
- `menus:read` - Read menu data
- `menus:write` - Update menu data
- `drivers:read` - Read driver data
- `drivers:write` - Update driver information
- `customers:read` - Read customer data
- `customers:write` - Update customer profiles
- `payments:process` - Process payments

---

## Order Management APIs

### Create Order

```
POST /orders
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "customerId": "CUST-789456",
  "restaurantId": "REST-PIZZA-HUT-001",
  "orderType": "delivery",
  "items": [
    {
      "itemId": "ITEM-PIZZA-MARGHERITA",
      "quantity": 2,
      "customizations": [...]
    }
  ],
  "deliveryAddress": {...},
  "contactInfo": {...}
}
```

**Response (201 Created):**
```json
{
  "orderId": "ORD-2025-001234",
  "status": "PENDING",
  "estimatedDelivery": "2025-12-27T13:00:00Z",
  "pricing": {
    "total": 35.48,
    "currency": "USD"
  },
  "timestamps": {
    "placed": "2025-12-27T12:00:00Z"
  }
}
```

### Get Order

```
GET /orders/{orderId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "orderId": "ORD-2025-001234",
  "status": "IN_TRANSIT",
  "customer": {...},
  "restaurant": {...},
  "driver": {...},
  "items": [...],
  "pricing": {...},
  "timeline": [...]
}
```

### Update Order Status

```
PATCH /orders/{orderId}/status
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "status": "CONFIRMED",
  "estimatedPreparation": "2025-12-27T12:20:00Z"
}
```

### List Orders

```
GET /orders?status=IN_TRANSIT&page=1&limit=20
Authorization: Bearer {access_token}
```

**Query Parameters:**
- `status` - Filter by order status
- `customerId` - Filter by customer
- `restaurantId` - Filter by restaurant
- `driverId` - Filter by driver
- `startDate` - Filter orders after date
- `endDate` - Filter orders before date
- `page` - Page number (default: 1)
- `limit` - Items per page (default: 20, max: 100)

**Response (200 OK):**
```json
{
  "orders": [...],
  "pagination": {
    "currentPage": 1,
    "totalPages": 5,
    "totalItems": 95,
    "itemsPerPage": 20
  }
}
```

### Cancel Order

```
DELETE /orders/{orderId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "reason": "customer_request",
  "notes": "Customer changed plans"
}
```

---

## Menu Management APIs

### Get Restaurant Menu

```
GET /restaurants/{restaurantId}/menu
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "menuId": "MENU-001",
  "restaurantId": "REST-PIZZA-HUT-001",
  "categories": [...],
  "lastUpdated": "2025-12-27T10:00:00Z"
}
```

### Update Menu Item

```
PATCH /restaurants/{restaurantId}/menu/items/{itemId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "available": false,
  "reason": "out_of_stock"
}
```

### Bulk Menu Update

```
PUT /restaurants/{restaurantId}/menu
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "categories": [...full menu structure...]
}
```

---

## Driver Management APIs

### Get Driver Profile

```
GET /drivers/{driverId}
Authorization: Bearer {access_token}
```

### Update Driver Status

```
PATCH /drivers/{driverId}/status
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "status": "AVAILABLE",
  "location": {
    "latitude": 40.7589,
    "longitude": -73.9851
  }
}
```

### Assign Order to Driver

```
POST /drivers/{driverId}/orders
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "orderId": "ORD-2025-001234"
}
```

### Get Driver Orders

```
GET /drivers/{driverId}/orders?status=active
Authorization: Bearer {access_token}
```

---

## Tracking APIs

### Update Location

```
POST /tracking/locations
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "driverId": "DRV-45789",
  "orderId": "ORD-2025-001234",
  "location": {
    "latitude": 40.7589,
    "longitude": -73.9851,
    "accuracy": 12,
    "heading": 90,
    "speed": 25
  },
  "timestamp": "2025-12-27T12:25:30Z"
}
```

### Get Order Tracking

```
GET /tracking/orders/{orderId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "orderId": "ORD-2025-001234",
  "status": "IN_TRANSIT",
  "driver": {
    "driverId": "DRV-45789",
    "name": "John Doe",
    "currentLocation": {...},
    "vehicle": {...}
  },
  "estimatedArrival": "2025-12-27T12:40:00Z",
  "route": {...}
}
```

---

## Restaurant APIs

### Register Restaurant

```
POST /restaurants
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "name": "Pizza Hut Downtown",
  "address": {...},
  "cuisineTypes": ["Pizza", "Italian"],
  "operatingHours": {...}
}
```

### Update Restaurant Profile

```
PATCH /restaurants/{restaurantId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "operatingHours": {...}
}
```

### Get Restaurant Analytics

```
GET /restaurants/{restaurantId}/analytics?period=week
Authorization: Bearer {access_token}
```

---

## Customer APIs

### Create Customer

```
POST /customers
Content-Type: application/json

{
  "name": "Jane Smith",
  "email": "jane@example.com",
  "phone": "+1-555-123-4567"
}
```

### Update Customer Profile

```
PATCH /customers/{customerId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "savedAddresses": [...],
  "preferences": {...}
}
```

### Get Order History

```
GET /customers/{customerId}/orders?limit=10
Authorization: Bearer {access_token}
```

---

## Payment APIs

### Create Payment Intent

```
POST /payments/intents
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "orderId": "ORD-2025-001234",
  "amount": 35.48,
  "currency": "USD",
  "paymentMethod": "pm_card_visa"
}
```

### Capture Payment

```
POST /payments/intents/{intentId}/capture
Authorization: Bearer {access_token}
```

### Process Refund

```
POST /payments/refunds
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "orderId": "ORD-2025-001234",
  "amount": 35.48,
  "reason": "customer_request"
}
```

---

## Error Handling

### Error Response Format

```json
{
  "error": {
    "code": "INVALID_ORDER_STATUS",
    "message": "Cannot cancel order in DELIVERED status",
    "details": {
      "orderId": "ORD-2025-001234",
      "currentStatus": "DELIVERED"
    },
    "timestamp": "2025-12-27T12:00:00Z",
    "requestId": "req-abc123"
  }
}
```

### HTTP Status Codes

- `200 OK` - Successful GET, PATCH
- `201 Created` - Successful POST
- `204 No Content` - Successful DELETE
- `400 Bad Request` - Invalid request data
- `401 Unauthorized` - Missing or invalid authentication
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Resource doesn't exist
- `409 Conflict` - Resource conflict
- `422 Unprocessable Entity` - Validation errors
- `429 Too Many Requests` - Rate limit exceeded
- `500 Internal Server Error` - Server error
- `503 Service Unavailable` - Service temporarily unavailable

### Common Error Codes

- `AUTHENTICATION_FAILED` - Invalid credentials
- `INSUFFICIENT_PERMISSIONS` - Lacking required scope
- `RESOURCE_NOT_FOUND` - Requested resource doesn't exist
- `VALIDATION_ERROR` - Request data failed validation
- `INVALID_ORDER_STATUS` - Operation not allowed in current state
- `PAYMENT_FAILED` - Payment processing error
- `RATE_LIMIT_EXCEEDED` - Too many requests

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-009-food-delivery is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-IND-009-food-delivery/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-009-food-delivery/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-009-food-delivery/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
