# WIA-SOC-003 Phase 2: API Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the RESTful API specifications for e-government services, including authentication endpoints, service request management, document handling, and real-time notifications. All APIs follow OpenAPI 3.0 specification.

## 2. Base Configuration

### 2.1 Base URL

```
Production: https://api.egov.{country-code}/v1
Sandbox: https://sandbox-api.egov.{country-code}/v1
```

### 2.2 Authentication

All API requests MUST include authentication:

```http
Authorization: Bearer {jwt-token}
X-API-Key: {api-key}
X-Request-ID: {uuid}
```

### 2.3 Headers

| Header | Required | Description |
|--------|----------|-------------|
| Authorization | Yes | JWT Bearer token |
| X-API-Key | Yes | API key for rate limiting |
| X-Request-ID | Yes | UUID for request tracking |
| Content-Type | Yes | application/json |
| Accept-Language | No | Preferred language (default: en) |
| X-Country-Code | No | ISO 3166-1 alpha-2 (default: from API domain) |

## 3. Authentication APIs

### 3.1 Citizen Authentication

**Endpoint**: `POST /auth/citizen`

**Request**:
```json
{
  "identityNumber": "encrypted-id",
  "method": "biometric",
  "biometricData": {
    "type": "face",
    "data": "base64-encoded-image",
    "metadata": {
      "captureTime": "2025-12-26T10:30:00Z",
      "device": "iPhone 15 Pro"
    }
  },
  "mfaToken": "optional-second-factor"
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refreshToken": "refresh-token-uuid",
  "expiresIn": 3600,
  "citizen": {
    "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "name": "Kim MinJun",
    "verificationLevel": "high"
  }
}
```

### 3.2 Token Refresh

**Endpoint**: `POST /auth/refresh`

**Request**:
```json
{
  "refreshToken": "refresh-token-uuid"
}
```

**Response (200 OK)**:
```json
{
  "token": "new-jwt-token",
  "expiresIn": 3600
}
```

### 3.3 Logout

**Endpoint**: `POST /auth/logout`

**Request**:
```json
{
  "token": "current-jwt-token"
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "message": "Logged out successfully"
}
```

## 4. Citizen Management APIs

### 4.1 Get Citizen Profile

**Endpoint**: `GET /citizen/{citizenId}`

**Response (200 OK)**:
```json
{
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "identityNumber": "****-****-4567",
  "givenName": "MinJun",
  "familyName": "Kim",
  "dateOfBirth": "1990-05-15",
  "nationality": "KR",
  "residency": {
    "country": "KR",
    "region": "Seoul",
    "district": "Gangnam-gu"
  },
  "contact": {
    "email": "kim.****@example.com",
    "phone": "+82-10-****-5678",
    "preferredLanguage": "ko"
  },
  "verificationLevel": "high",
  "createdAt": "2020-01-01T00:00:00Z",
  "updatedAt": "2025-12-26T10:00:00Z"
}
```

### 4.2 Update Citizen Profile

**Endpoint**: `PUT /citizen/{citizenId}`

**Request**:
```json
{
  "contact": {
    "email": "new.email@example.com",
    "phone": "+82-10-9876-5432"
  },
  "residency": {
    "district": "Songpa-gu",
    "postalCode": "05500"
  },
  "verification": {
    "method": "otp",
    "code": "123456"
  }
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "citizen": { /* updated citizen object */ },
  "message": "Profile updated successfully"
}
```

## 5. Service Request APIs

### 5.1 List Available Services

**Endpoint**: `GET /services/available`

**Query Parameters**:
- `category` (optional): Filter by category
- `language` (optional): Service description language
- `page` (optional): Page number (default: 1)
- `limit` (optional): Items per page (default: 20, max: 100)

**Response (200 OK)**:
```json
{
  "services": [
    {
      "id": "SRV-TAX-001",
      "name": {
        "en": "Individual Tax Filing",
        "ko": "개인 세금 신고"
      },
      "category": "taxation",
      "averageProcessingTime": "P3D",
      "fee": 0,
      "available": true
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 142,
    "pages": 8
  }
}
```

### 5.2 Submit Service Request

**Endpoint**: `POST /services/request`

**Request**:
```json
{
  "serviceId": "SRV-TAX-001",
  "priority": "normal",
  "metadata": {
    "taxYear": 2024,
    "filingType": "individual"
  },
  "documents": ["DOC-2025-5678", "DOC-2025-5679"],
  "notes": "First time filing"
}
```

**Response (201 Created)**:
```json
{
  "success": true,
  "request": {
    "requestId": "REQ-2025-001234",
    "status": "submitted",
    "estimatedCompletion": "2025-12-30T17:00:00Z",
    "trackingUrl": "https://portal.egov.kr/track/REQ-2025-001234"
  }
}
```

### 5.3 Get Request Status

**Endpoint**: `GET /services/request/{requestId}`

**Response (200 OK)**:
```json
{
  "requestId": "REQ-2025-001234",
  "serviceType": "tax_filing",
  "status": "in_progress",
  "progress": 65,
  "currentStep": "verification",
  "steps": [
    {
      "name": "submission",
      "status": "completed",
      "completedAt": "2025-12-26T14:30:00Z"
    },
    {
      "name": "verification",
      "status": "in_progress",
      "estimatedCompletion": "2025-12-27T12:00:00Z"
    },
    {
      "name": "processing",
      "status": "pending"
    },
    {
      "name": "approval",
      "status": "pending"
    }
  ],
  "updates": [
    {
      "timestamp": "2025-12-26T14:30:00Z",
      "message": "Request submitted successfully",
      "type": "info"
    },
    {
      "timestamp": "2025-12-26T15:00:00Z",
      "message": "Documents verified",
      "type": "success"
    }
  ]
}
```

### 5.4 Cancel Service Request

**Endpoint**: `DELETE /services/request/{requestId}`

**Request**:
```json
{
  "reason": "Changed plans, no longer needed",
  "verification": {
    "method": "otp",
    "code": "123456"
  }
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "message": "Request cancelled successfully",
  "refundAmount": 0,
  "refundMethod": "none"
}
```

## 6. Document Management APIs

### 6.1 Upload Document

**Endpoint**: `POST /documents/upload`

**Request** (multipart/form-data):
```
Content-Type: multipart/form-data
---
file: [binary file data]
type: "tax_form"
subtype: "w2"
metadata: {
  "taxYear": 2024,
  "employer": "Example Corp"
}
encryption: true
```

**Response (201 Created)**:
```json
{
  "success": true,
  "document": {
    "documentId": "DOC-2025-5678",
    "type": "tax_form",
    "size": 2458624,
    "hash": "sha256:abc123...",
    "uploadedAt": "2025-12-26T14:00:00Z",
    "status": "uploaded",
    "virusScan": "clean"
  }
}
```

### 6.2 Get Document

**Endpoint**: `GET /documents/{documentId}`

**Response (200 OK)**:
```json
{
  "documentId": "DOC-2025-5678",
  "type": "certificate",
  "subtype": "birth_certificate",
  "name": "Birth Certificate - Kim MinJun",
  "issuedDate": "2025-12-26",
  "validUntil": "2035-12-26",
  "format": "application/pdf",
  "size": 1048576,
  "downloadUrl": "https://cdn.egov.kr/docs/secure/DOC-2025-5678?token=xyz",
  "qrCodeUrl": "https://verify.egov.kr/doc/DOC-2025-5678"
}
```

### 6.3 Download Document

**Endpoint**: `GET /documents/{documentId}/download`

**Response (200 OK)**:
```
Content-Type: application/pdf
Content-Disposition: attachment; filename="birth_certificate.pdf"
Content-Length: 1048576
X-Document-Hash: sha256:abc123...

[Binary PDF data]
```

### 6.4 Verify Document

**Endpoint**: `GET /documents/{documentId}/verify`

**Response (200 OK)**:
```json
{
  "documentId": "DOC-2025-5678",
  "valid": true,
  "issuedBy": "Seoul Metropolitan Government",
  "issuedDate": "2025-12-26",
  "verificationMethod": "digital_signature",
  "blockchainTx": "0x123abc...",
  "qrCode": "data:image/png;base64,..."
}
```

## 7. Payment APIs

### 7.1 Initiate Payment

**Endpoint**: `POST /payment`

**Request**:
```json
{
  "requestId": "REQ-2025-001234",
  "amount": 50000,
  "currency": "KRW",
  "method": "credit_card",
  "returnUrl": "https://example.com/payment/callback"
}
```

**Response (200 OK)**:
```json
{
  "paymentId": "PAY-2025-7890",
  "amount": 50000,
  "currency": "KRW",
  "status": "pending",
  "paymentUrl": "https://pay.egov.kr/PAY-2025-7890",
  "expiresAt": "2025-12-26T15:00:00Z"
}
```

### 7.2 Get Payment Status

**Endpoint**: `GET /payment/{paymentId}`

**Response (200 OK)**:
```json
{
  "paymentId": "PAY-2025-7890",
  "requestId": "REQ-2025-001234",
  "amount": 50000,
  "currency": "KRW",
  "status": "completed",
  "method": "credit_card",
  "transactionId": "TXN-BANK-123456",
  "paidAt": "2025-12-26T14:45:00Z",
  "receipt": {
    "receiptId": "RCP-2025-4567",
    "downloadUrl": "https://cdn.egov.kr/receipts/RCP-2025-4567.pdf"
  }
}
```

## 8. Notification APIs

### 8.1 WebSocket Connection

**Endpoint**: `WSS /ws`

**Connection**:
```javascript
const ws = new WebSocket('wss://api.egov.kr/v1/ws');
ws.addEventListener('open', () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'jwt-token'
  }));
});
```

**Message Types**:
```json
{
  "type": "service_update",
  "requestId": "REQ-2025-001234",
  "status": "in_progress",
  "message": "Your request is being processed",
  "timestamp": "2025-12-26T15:00:00Z"
}
```

### 8.2 Subscribe to Notifications

**Endpoint**: `POST /notifications/subscribe`

**Request**:
```json
{
  "channels": ["email", "sms", "push"],
  "events": ["service_update", "document_ready", "payment_received"],
  "preferences": {
    "email": "kim.minjun@example.com",
    "phone": "+82-10-1234-5678",
    "pushToken": "fcm-device-token"
  }
}
```

**Response (200 OK)**:
```json
{
  "success": true,
  "subscriptionId": "SUB-2025-1234",
  "channels": ["email", "sms", "push"]
}
```

## 9. Analytics APIs (Public)

### 9.1 Get Public Statistics

**Endpoint**: `GET /analytics/public`

**Response (200 OK)**:
```json
{
  "period": {
    "start": "2025-12-01",
    "end": "2025-12-26"
  },
  "statistics": {
    "totalRequests": 892567,
    "completedRequests": 845234,
    "averageProcessingTime": "P2D18H",
    "citizenSatisfaction": 4.8,
    "topServices": [
      {
        "serviceId": "SRV-TAX-001",
        "name": "Tax Filing",
        "requests": 142567
      },
      {
        "serviceId": "SRV-HEALTH-001",
        "name": "Healthcare",
        "requests": 98234
      }
    ]
  }
}
```

## 10. Error Responses

### 10.1 Error Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "The request is invalid",
    "details": {
      "field": "identityNumber",
      "issue": "Invalid format"
    },
    "requestId": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-12-26T15:00:00Z"
  }
}
```

### 10.2 Error Codes

| HTTP Status | Error Code | Description |
|-------------|------------|-------------|
| 400 | INVALID_REQUEST | Malformed request body |
| 401 | UNAUTHORIZED | Missing or invalid authentication |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | NOT_FOUND | Resource not found |
| 409 | CONFLICT | Resource already exists |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Service temporarily unavailable |

## 11. Rate Limiting

### 11.1 Limits

| Tier | Requests/Minute | Requests/Day |
|------|-----------------|--------------|
| Free | 60 | 5,000 |
| Standard | 600 | 50,000 |
| Premium | 6,000 | 500,000 |
| Enterprise | Custom | Custom |

### 11.2 Headers

```http
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 543
X-RateLimit-Reset: 1640554800
```

## 12. Pagination

### 12.1 Request

```http
GET /services/available?page=2&limit=50
```

### 12.2 Response

```json
{
  "data": [...],
  "pagination": {
    "page": 2,
    "limit": 50,
    "total": 142,
    "pages": 3,
    "hasNext": true,
    "hasPrevious": true
  }
}
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc.
