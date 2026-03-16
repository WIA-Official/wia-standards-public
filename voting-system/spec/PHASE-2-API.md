# WIA-SOC-015: Phase 2 - API Interface Specification

**Version:** 1.0  
**Status:** FINAL  
**Last Updated:** 2025-01-15  
**Standards Body:** World Certification Industry Association (WIA)

---

## 1. Overview

Phase 2 defines RESTful API interfaces enabling communication between voting system components. This specification ensures interoperability while maintaining security, scalability, and accessibility.

### 1.1 Architecture Principles

- RESTful design following HTTP standards
- Stateless communication
- OAuth 2.0 authentication
- JSON request/response bodies
- Comprehensive error handling
- Rate limiting for system protection

---

## 2. Authentication & Authorization

### 2.1 OAuth 2.0 Implementation

All API access requires OAuth 2.0 authentication.

**Supported Flows:**
- Authorization Code (voter-facing applications)
- Client Credentials (server-to-server)
- Device Code (voting terminals)

**Token Endpoint:**
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
client_secret={client_secret}
```

### 2.2 JWT Token Format

Access tokens use JSON Web Token (JWT) format:

```json
{
  "iss": "https://auth.voting.example.com",
  "sub": "voter-12345",
  "aud": "voting-api",
  "exp": 1735689600,
  "iat": 1735686000,
  "scope": "vote.cast ballot.view",
  "wia": {
    "standard": "WIA-SOC-015-v1.0",
    "jurisdiction": "CA-DIST-12",
    "election": "ELECTION-2025-001"
  }
}
```

---

## 3. Core API Endpoints

### 3.1 Voter Registration

#### Register New Voter

```
POST /v1/voters/register
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "personalInfo": {...},
  "address": {...},
  "contactInfo": {...},
  "credentials": {...}
}

Response: 201 Created
{
  "voterId": "VOTER-CA-2025-789456",
  "status": "pending-verification",
  "registrationDate": "2025-01-15T14:30:00Z"
}
```

#### Check Registration Status

```
GET /v1/voters/{voterId}/status
Authorization: Bearer {access_token}

Response: 200 OK
{
  "voterId": "VOTER-CA-2025-789456",
  "status": "active",
  "eligibility": {...}
}
```

### 3.2 Ballot Management

#### Get Voter's Ballot

```
GET /v1/elections/{electionId}/ballots/{voterId}
Authorization: Bearer {access_token}

Response: 200 OK
{
  "ballotId": "BALLOT-2025-001-VOTER-789456",
  "version": "WIA-SOC-015-v1.0",
  "contests": [...],
  "personalizations": {...}
}
```

#### Submit Completed Ballot

```
POST /v1/elections/{electionId}/votes
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "ballotId": "BALLOT-2025-001-VOTER-789456",
  "voterId": "VOTER-CA-2025-789456",
  "votes": [...],
  "metadata": {...},
  "signature": "cryptographic-signature"
}

Response: 201 Created
{
  "voteId": "VOTE-2025-001-987654",
  "status": "accepted",
  "receipt": {...},
  "timestamp": "2025-03-01T14:30:15Z"
}
```

### 3.3 Result Tabulation

#### Get Election Results

```
GET /v1/elections/{electionId}/results?contest={contestId}&level=district&format=json
Authorization: Bearer {access_token}

Response: 200 OK
{
  "electionId": "ELECTION-2025-001",
  "reportTimestamp": "2025-03-01T22:00:00Z",
  "status": "preliminary",
  "completeness": {...},
  "results": [...]
}
```

### 3.4 Audit Trail Access

#### Get Audit Events

```
GET /v1/elections/{electionId}/audit?startDate=2025-03-01&eventType=vote-cast&limit=100
Authorization: Bearer {access_token}

Response: 200 OK
{
  "events": [...],
  "pagination": {...}
}
```

#### Verify Ballot Inclusion

```
GET /v1/verification/{receiptId}

Response: 200 OK
{
  "receiptId": "RECEIPT-2025-XYZ789",
  "verificationCode": "A7F3E9",
  "status": "verified",
  "included": true,
  "blockchainVerification": {...}
}
```

---

## 4. Error Handling

### 4.1 Standard Error Response

```json
{
  "error": {
    "code": "AUTH_INVALID_TOKEN",
    "message": "The provided authentication token is invalid or expired",
    "details": "Token expired at 2025-01-15T10:00:00Z",
    "timestamp": "2025-01-15T10:05:30Z",
    "requestId": "req-abc123def456",
    "documentation": "https://docs.wia-soc-015.com/errors/AUTH_INVALID_TOKEN"
  }
}
```

### 4.2 HTTP Status Codes

| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful GET request |
| 201 | Created | Successful POST creating resource |
| 400 | Bad Request | Invalid request data |
| 401 | Unauthorized | Missing/invalid authentication |
| 403 | Forbidden | Authenticated but not authorized |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side failure |

---

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1735689600
```

### 5.2 Limit Tiers

- **Voter Operations:** 100 requests/hour/voter
- **Poll Worker Operations:** 1,000 requests/hour/worker
- **Election Official Operations:** 10,000 requests/hour
- **Audit Access:** 10,000 requests/hour (public data)

---

## 6. Security Requirements

### 6.1 Transport Security

- HTTPS/TLS 1.3 required for all API communication
- Certificate pinning recommended
- HSTS headers mandatory

### 6.2 Input Validation

- Validate all input parameters
- Sanitize user-provided data
- Enforce maximum request sizes
- Implement SQL injection protection

---

## 7. Conformance Requirements

Systems must implement:
- All required endpoints
- OAuth 2.0 authentication
- Standard error responses
- Rate limiting
- HTTPS/TLS 1.3

---

**© 2025 World Certification Industry Association (WIA)**  
**弘益人間 (Hongik Ingan) - Benefit All Humanity**
