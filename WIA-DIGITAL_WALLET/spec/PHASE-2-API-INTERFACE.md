# WIA-DIGITAL_WALLET: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the API interface requirements for DIGITAL WALLET. All implementations MUST provide these endpoints to ensure interoperability.

## 2. REST API Endpoints

### 2.1 Base URL
```
https://api.wia.org/v1/wia-digital-wallet/
```

### 2.2 Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | /records | List all records |
| GET | /records/{id} | Get specific record |
| POST | /records | Create new record |
| PUT | /records/{id} | Update record |
| DELETE | /records/{id} | Delete record |
| GET | /status | Service health check |
| GET | /schema | Get data schema |

### 2.3 Request/Response Format

#### Create Record (POST /records)
```http
POST /v1/wia-digital-wallet/records HTTP/1.1
Content-Type: application/json
Authorization: Bearer <token>

{
  "category": "standard",
  "value": "data content",
  "metadata": {}
}
```

#### Response
```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "created",
  "timestamp": "2025-01-01T00:00:00Z"
}
```

## 3. TypeScript SDK

### 3.1 Installation
```bash
npm install @wia/wia-digital-wallet-sdk
```

### 3.2 Client Interface
```typescript
interface WIAClient {
  // Record operations
  createRecord(data: RecordInput): Promise<Record>;
  getRecord(id: string): Promise<Record>;
  updateRecord(id: string, data: Partial<RecordInput>): Promise<Record>;
  deleteRecord(id: string): Promise<void>;
  listRecords(options?: ListOptions): Promise<RecordList>;
  
  // Utility methods
  validate(data: unknown): ValidationResult;
  getSchema(): Promise<JSONSchema>;
  healthCheck(): Promise<HealthStatus>;
}

interface RecordInput {
  category: string;
  value: any;
  metadata?: Record<string, unknown>;
}

interface Record extends RecordInput {
  id: string;
  type: string;
  version: string;
  timestamp: string;
  signature?: string;
}
```

### 3.3 Usage Example
```typescript
import { createClient } from '@wia/wia-digital-wallet-sdk';

const client = createClient({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia.org/v1'
});

// Create a new record
const record = await client.createRecord({
  category: 'standard',
  value: 'example data',
  metadata: { source: 'application' }
});

// Get record by ID
const retrieved = await client.getRecord(record.id);
```

## 4. Authentication

### 4.1 API Key Authentication
```http
Authorization: Bearer <api_key>
```

### 4.2 OAuth 2.0
```http
Authorization: Bearer <access_token>
```

### 4.3 Token Scopes
| Scope | Description |
|-------|-------------|
| read | Read-only access |
| write | Create and update |
| delete | Delete records |
| admin | Full access |

## 5. Error Handling

### 5.1 Error Response Format
```json
{
  "error": {
    "code": "E001",
    "message": "Validation failed",
    "details": [
      { "field": "category", "issue": "required" }
    ]
  }
}
```

### 5.2 HTTP Status Codes
| Code | Meaning |
|------|---------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 429 | Rate Limited |
| 500 | Server Error |

## 6. Rate Limiting

### 6.1 Limits
| Tier | Requests/Minute | Requests/Day |
|------|----------------|--------------|
| Free | 60 | 1,000 |
| Standard | 600 | 50,000 |
| Enterprise | 6,000 | Unlimited |

### 6.2 Rate Limit Headers
```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1704067200
```

## 7. Webhooks

### 7.1 Event Types
| Event | Description |
|-------|-------------|
| record.created | New record created |
| record.updated | Record updated |
| record.deleted | Record deleted |

### 7.2 Webhook Payload
```json
{
  "event": "record.created",
  "timestamp": "2025-01-01T00:00:00Z",
  "data": { "id": "...", "type": "..." }
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
