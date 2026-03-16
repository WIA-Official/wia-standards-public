# WIA-SOC-013 PHASE 2: API SPECIFICATION

**Public Document Standard - RESTful API and SDK**

Version: 1.0
Date: 2025-01-15
Status: Final

---

## 1. API Endpoints

### Base URL

```
https://api.wiastandards.gov/v1/documents
```

### 1.1 Document Operations

#### Create Document
```http
POST /documents
Content-Type: application/json
Authorization: Bearer {token}

{
  "type": "birthCertificate",
  "data": {...},
  "language": "en"
}

Response: 201 Created
{
  "documentId": "doc:wia:soc013:birth:US:2025:abc123",
  "status": "created",
  "url": "/documents/doc:wia:soc013:birth:US:2025:abc123"
}
```

#### Get Document
```http
GET /documents/{documentId}
Authorization: Bearer {token}

Response: 200 OK
{
  "documentId": "...",
  "type": "...",
  "data": {...}
}
```

#### Verify Document
```http
POST /documents/{documentId}/verify
Content-Type: application/json

{
  "checkRevocation": true,
  "validateSignature": true,
  "verifyBlockchain": true
}

Response: 200 OK
{
  "valid": true,
  "signatureValid": true,
  "notRevoked": true,
  "blockchainValid": true
}
```

### 1.2 Signature Operations

```http
POST /documents/{documentId}/sign
Authorization: Bearer {token}

{
  "algorithm": "ECDSA-SHA256",
  "certificateChain": true
}

Response: 200 OK
{
  "signature": "...",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 1.3 Metadata Operations

```http
PUT /documents/{documentId}/metadata
Content-Type: application/json

{
  "dublinCore": {...},
  "premis": {...}
}

Response: 200 OK
```

## 2. SDK Libraries

### 2.1 JavaScript/TypeScript

```typescript
import { WiaPublicDocument } from 'wia-soc-013';

const document = await WiaPublicDocument.create({
  type: 'birthCertificate',
  data: birthData
});

await document.sign({ privateKey: key });
await document.publish();

const verification = await document.verify();
```

### 2.2 Python

```python
from wia_soc_013 import WiaPublicDocument

document = WiaPublicDocument.create(
    type='birthCertificate',
    data=birth_data
)

document.sign(private_key=key)
document.publish()

verification = document.verify()
```

## 3. Authentication

OAuth 2.0 with OpenID Connect:

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=documents:read documents:write

Response:
{
  "access_token": "...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

## 4. Rate Limiting

- 1000 requests per minute per API key
- 50 MB total upload size per minute
- Retry-After header on 429 responses

## 5. Error Codes

| Code | Description |
|------|-------------|
| 400 | Bad Request - Invalid parameters |
| 401 | Unauthorized - Invalid credentials |
| 403 | Forbidden - Insufficient permissions |
| 404 | Not Found - Document doesn't exist |
| 409 | Conflict - Document already exists |
| 429 | Too Many Requests - Rate limit exceeded |
| 500 | Internal Server Error |

## 6. Webhooks

Register webhooks for events:

```http
POST /webhooks
{
  "url": "https://your-app.com/webhook",
  "events": ["document.created", "document.verified", "document.revoked"]
}
```

Webhook payload:
```json
{
  "event": "document.created",
  "documentId": "...",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {...}
}
```

---

© 2025 SmileStory Inc. / WIA
