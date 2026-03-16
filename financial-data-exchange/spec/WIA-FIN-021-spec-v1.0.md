# WIA-FIN-021: Financial Data Exchange Standard
## Specification Version 1.0

**Status:** Published
**Date:** January 15, 2024
**Authors:** WIA Technical Committee
**Category:** Finance (FIN)

---

## Abstract

This specification defines the Financial Data Exchange Standard (WIA-FIN-021), providing a comprehensive framework for secure, efficient, and interoperable exchange of financial data across institutions, platforms, and jurisdictions.

## 1. Introduction

### 1.1 Purpose

The WIA-FIN-021 standard addresses the growing need for standardized financial data exchange in an increasingly interconnected global financial ecosystem. It provides:

- Standardized protocols for data transmission
- Common data formats and schemas
- Security and authentication requirements
- Compliance and regulatory considerations
- Implementation guidelines and best practices

### 1.2 Scope

This standard applies to:

- Account information exchange
- Payment initiation and processing
- Transaction data sharing
- Market data distribution
- Regulatory reporting
- Cross-border financial data flows

### 1.3 Normative References

- ISO 20022: Financial Services - Universal financial industry message scheme
- ISO 8583: Financial transaction card originated messages
- FIX Protocol 4.4: Financial Information eXchange Protocol
- OAuth 2.0 (RFC 6749): The OAuth 2.0 Authorization Framework
- TLS 1.3 (RFC 8446): The Transport Layer Security Protocol

## 2. Architecture

### 2.1 Layered Architecture

The WIA-FIN-021 architecture consists of four layers:

```
┌─────────────────────────────────────┐
│    Application Layer                │
│    (Business Logic & Services)      │
├─────────────────────────────────────┤
│    Data Exchange Layer              │
│    (Transformation & Routing)       │
├─────────────────────────────────────┤
│    Security Layer                   │
│    (Authentication & Encryption)    │
├─────────────────────────────────────┤
│    Transport Layer                  │
│    (HTTP/2, WebSocket, MQ)          │
└─────────────────────────────────────┘
```

### 2.2 Data Flow

```
Source System → Authentication → Validation →
Transformation → Routing → Delivery → Target System
```

## 3. Protocols

### 3.1 HTTP-based APIs

#### 3.1.1 RESTful APIs

**Base URL Structure:**
```
https://{host}/api/v{version}/{resource}
```

**HTTP Methods:**
- `GET`: Retrieve resources
- `POST`: Create new resources
- `PUT`: Update entire resources
- `PATCH`: Partial resource updates
- `DELETE`: Remove resources

**Example Request:**
```http
GET /api/v1/accounts/ACC-12345 HTTP/2
Host: api.bank.example.com
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
Accept: application/json
```

**Example Response:**
```http
HTTP/2 200 OK
Content-Type: application/json

{
  "accountId": "ACC-12345",
  "currency": "USD",
  "balance": 2500.00,
  "status": "ACTIVE"
}
```

#### 3.1.2 GraphQL

GraphQL endpoints SHOULD be provided at:
```
https://{host}/graphql
```

### 3.2 Real-time Protocols

#### 3.2.1 WebSocket

WebSocket connections MUST use secure WebSocket (wss://):
```
wss://api.exchange.com/v1/stream
```

#### 3.2.2 Server-Sent Events

SSE endpoints for server-to-client streaming:
```
GET /api/v1/events/stream HTTP/1.1
Accept: text/event-stream
```

## 4. Data Formats

### 4.1 JSON

JSON MUST be the primary format for RESTful APIs.

**Account Object Schema:**
```json
{
  "accountId": "string",
  "customerId": "string",
  "accountType": "CHECKING|SAVINGS|INVESTMENT",
  "currency": "string (ISO 4217)",
  "balance": {
    "available": "number",
    "current": "number",
    "pending": "number"
  },
  "status": "ACTIVE|SUSPENDED|CLOSED",
  "openDate": "string (ISO 8601)",
  "metadata": "object"
}
```

### 4.2 ISO 20022

ISO 20022 XML MUST be supported for:
- Cross-border payments (pacs.008)
- Customer credit transfers (pain.001)
- Account statements (camt.053)

### 4.3 FIX Protocol

FIX 4.4 or higher MUST be supported for:
- Securities trading
- Market data distribution
- Order management

## 5. Security

### 5.1 Authentication

#### 5.1.1 OAuth 2.0

All API endpoints MUST support OAuth 2.0 authorization.

**Required Flows:**
- Authorization Code Grant (for user consent)
- Client Credentials Grant (for server-to-server)

**Token Endpoint:**
```http
POST /oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTH_CODE&
redirect_uri=https://client.example.com/callback&
client_id=CLIENT_ID&
client_secret=CLIENT_SECRET
```

#### 5.1.2 JWT

Access tokens MUST be formatted as JSON Web Tokens (JWT) using RS256 or ES256 algorithms.

**JWT Structure:**
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "iss": "https://auth.bank.com",
    "sub": "user@example.com",
    "aud": "api.bank.com",
    "exp": 1735138200,
    "scope": "accounts:read transactions:read"
  }
}
```

### 5.2 Encryption

#### 5.2.1 Transport Layer

All communications MUST use TLS 1.3 or TLS 1.2 with approved cipher suites.

**Minimum TLS Configuration:**
- Protocol: TLS 1.2 or higher
- Cipher Suites: TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384 or stronger
- Certificate: Valid X.509 certificate from trusted CA

#### 5.2.2 Data at Rest

Sensitive data MUST be encrypted using AES-256-GCM or stronger.

### 5.3 Access Control

Role-Based Access Control (RBAC) MUST be implemented.

**Standard Roles:**
- `viewer`: Read-only access
- `editor`: Read and write access
- `admin`: Full access including user management

## 6. Data Exchange Patterns

### 6.1 Request-Response

Synchronous data retrieval:
```
Client → Request → Server
Client ← Response ← Server
```

### 6.2 Event-Driven

Asynchronous event processing:
```
Producer → Event → Message Queue → Consumer
```

### 6.3 Batch Processing

Scheduled bulk data transfers:
```
Source → Extract → Transform → Load → Destination
```

## 7. Error Handling

### 7.1 HTTP Status Codes

Standard HTTP status codes MUST be used:

- `200 OK`: Successful request
- `201 Created`: Resource created
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary unavailability

### 7.2 Error Response Format

```json
{
  "error": {
    "code": "INVALID_ACCOUNT",
    "message": "Account not found",
    "details": {
      "accountId": "ACC-99999",
      "timestamp": "2024-01-15T10:30:00Z"
    }
  }
}
```

## 8. Compliance

### 8.1 Regulatory Requirements

Implementations MUST comply with applicable regulations:

- **PSD2**: Strong Customer Authentication, Open Banking APIs
- **GDPR**: Data protection, consent management, right to erasure
- **PCI DSS**: Payment card data security (if applicable)
- **SOC 2**: Security, availability, confidentiality controls

### 8.2 Audit Logging

All data exchange operations MUST be logged with:
- Timestamp (ISO 8601)
- User/system identifier
- Operation performed
- Result (success/failure)
- IP address
- Data elements accessed (without sensitive values)

## 9. Performance Requirements

### 9.1 Latency

- API requests: < 500ms (p95)
- Real-time streaming: < 100ms (p95)
- Batch processing: Based on SLA

### 9.2 Throughput

Minimum throughput requirements:
- REST APIs: 1,000 requests/second per instance
- WebSocket: 10,000 messages/second per instance
- Message Queue: 50,000 messages/second per cluster

### 9.3 Availability

- Production systems: 99.9% uptime
- Critical systems: 99.95% uptime

## 10. Implementation Guidelines

### 10.1 API Versioning

Use URL versioning:
```
/api/v1/accounts
/api/v2/accounts
```

Version deprecation MUST provide:
- 12 months notice minimum
- Migration guide
- Parallel support during transition

### 10.2 Rate Limiting

Implement rate limiting to prevent abuse:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1735138800
```

### 10.3 Pagination

Use cursor-based pagination for large result sets:

```http
GET /api/v1/transactions?limit=50&cursor=eyJpZCI6MTIzNDU2fQ
```

Response:
```json
{
  "data": [...],
  "pagination": {
    "next_cursor": "eyJpZCI6MTIzNTA2fQ",
    "has_more": true
  }
}
```

## 11. Testing

### 11.1 Test Environments

Provide separate environments:
- **Sandbox**: For initial development and testing
- **Staging**: Production-like environment for final testing
- **Production**: Live environment

### 11.2 Test Data

Sandbox environments MUST provide:
- Sample accounts
- Test credentials
- Simulated transactions
- Error scenarios

## Appendix A: Examples

### A.1 Complete Payment Flow

```http
# 1. Initiate Payment
POST /api/v1/payments HTTP/1.1
Authorization: Bearer {token}
Content-Type: application/json

{
  "debtorAccount": "DE89370400440532013000",
  "creditorAccount": "GB29NWBK60161331926819",
  "amount": 1500.00,
  "currency": "EUR",
  "reference": "Invoice 12345"
}

# 2. Response
HTTP/1.1 201 Created
Location: /api/v1/payments/PMT-001

{
  "paymentId": "PMT-001",
  "status": "PENDING",
  "scaRedirectUrl": "https://bank.com/sca?id=PMT-001"
}

# 3. Get Payment Status
GET /api/v1/payments/PMT-001 HTTP/1.1
Authorization: Bearer {token}

# 4. Status Response
HTTP/1.1 200 OK

{
  "paymentId": "PMT-001",
  "status": "COMPLETED",
  "transactionDate": "2024-01-15T14:30:00Z"
}
```

## Appendix B: Security Checklist

- [ ] TLS 1.3 enabled
- [ ] OAuth 2.0 implemented
- [ ] JWT token validation
- [ ] Rate limiting configured
- [ ] Audit logging enabled
- [ ] Data encryption at rest
- [ ] Input validation
- [ ] CSRF protection
- [ ] CORS configured
- [ ] Security headers set

---

**Version:** 1.0
**Effective Date:** January 15, 2024
**Next Review:** January 15, 2025

© 2024 WIA (World Industry Association)
弘益人間 (Hongik Ingan) - Benefit All Humanity
