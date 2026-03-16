# WIA-AI-012: Federated Learning - Phase 2: API Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-09

## Overview

This specification defines the RESTful API and gRPC interfaces for federated learning systems.

## Philosophy

弘益人間 (Hongik Ingan) - This API standard enables any organization to participate in privacy-preserving collaborative learning, benefiting all humanity.

## 1. REST API Endpoints

### 1.1 Server API

#### Register Client

```http
POST /api/v1/clients/register
Content-Type: application/json
Authorization: Bearer <api-key>

{
  "device_id": "hashed-device-identifier",
  "platform": "android|ios|linux",
  "capabilities": {
    "max_model_size_mb": 500,
    "supported_frameworks": ["tensorflow", "pytorch"],
    "compute_capability": "low|medium|high"
  }
}

Response 201:
{
  "client_id": "uuid",
  "registration_token": "jwt-token",
  "expires_at": "ISO-8601"
}
```

#### Get Training Round

```http
GET /api/v1/rounds/current
Authorization: Bearer <client-token>

Response 200:
{
  "round_number": 42,
  "status": "active|pending|completed",
  "global_model_url": "https://...",
  "training_config": {...},
  "deadline": "ISO-8601"
}
```

#### Submit Update

```http
POST /api/v1/rounds/{round_id}/updates
Content-Type: application/octet-stream
Authorization: Bearer <client-token>
X-Update-Checksum: sha256-hash
X-Client-Signature: ed25519-signature

<binary model update data>

Response 202:
{
  "update_id": "uuid",
  "status": "accepted",
  "estimated_aggregation_time": "ISO-8601"
}
```

#### Download Model

```http
GET /api/v1/models/{version}
Authorization: Bearer <client-token>
Accept-Encoding: gzip, br

Response 200:
Content-Type: application/octet-stream
Content-Length: 104857600
X-Model-Checksum: sha256-hash
X-Model-Version: 1.2.3

<binary model data>
```

### 1.2 Client API (for server callbacks)

#### Receive Training Invitation

```http
POST /client/v1/training/invite
Content-Type: application/json
Authorization: Bearer <server-token>

{
  "round_number": 42,
  "global_model_url": "https://...",
  "training_config": {...},
  "deadline": "ISO-8601"
}

Response 200:
{
  "accepted": true,
  "estimated_completion": "ISO-8601"
}
```

## 2. gRPC Service Definition

```protobuf
syntax = "proto3";

package wia.federatedlearning.v1;

// Federated Learning Service
service FederatedLearning {
  // Register a new client
  rpc RegisterClient(RegisterClientRequest) returns (RegisterClientResponse);

  // Get current training round information
  rpc GetTrainingRound(GetTrainingRoundRequest) returns (TrainingRound);

  // Submit model update
  rpc SubmitUpdate(stream UpdateChunk) returns (UpdateReceipt);

  // Download global model
  rpc DownloadModel(DownloadModelRequest) returns (stream ModelChunk);

  // Bidirectional streaming for real-time coordination
  rpc StreamUpdates(stream ClientUpdate) returns (stream ServerMessage);
}

message RegisterClientRequest {
  string device_id = 1;
  string platform = 2;
  ClientCapabilities capabilities = 3;
}

message RegisterClientResponse {
  string client_id = 1;
  string registration_token = 2;
  int64 expires_at = 3;
}

message TrainingRound {
  int32 round_number = 1;
  string status = 2;
  ModelReference global_model = 3;
  TrainingConfig config = 4;
  int64 deadline = 5;
}

message ModelReference {
  string model_id = 1;
  string version = 2;
  string download_url = 3;
  bytes checksum = 4;
  int64 size_bytes = 5;
}

message UpdateChunk {
  bytes data = 1;
  int32 chunk_index = 2;
  int32 total_chunks = 3;
}

message UpdateReceipt {
  string update_id = 1;
  string status = 2;
  int64 estimated_aggregation_time = 3;
}

message ModelChunk {
  bytes data = 1;
  int32 chunk_index = 2;
  int32 total_chunks = 3;
}
```

## 3. WebSocket API

For real-time bidirectional communication:

```javascript
// Connection
const ws = new WebSocket('wss://server/api/v1/ws');

// Client -> Server: Subscribe to round updates
ws.send(JSON.stringify({
  type: 'subscribe',
  topics: ['round_updates', 'model_updates']
}));

// Server -> Client: Training invitation
{
  type: 'training_invitation',
  round_number: 42,
  global_model_url: '...',
  deadline: '...'
}

// Client -> Server: Update progress
{
  type: 'training_progress',
  round_number: 42,
  progress_percent: 45,
  estimated_completion: '...'
}

// Server -> Client: New model available
{
  type: 'model_ready',
  version: '1.2.4',
  download_url: '...'
}
```

## 4. Authentication & Authorization

### 4.1 API Key Authentication

```http
Authorization: Bearer <api-key>
```

- Used for initial client registration
- Rate limited: 100 requests/minute
- Scope: `client:register`

### 4.2 JWT Token Authentication

```http
Authorization: Bearer <jwt-token>
```

Payload:
```json
{
  "sub": "client-uuid",
  "iat": 1234567890,
  "exp": 1234571490,
  "scope": "training:participate model:download"
}
```

### 4.3 OAuth 2.0 (for organizational clients)

```http
Authorization: Bearer <oauth-access-token>
```

Supported flows:
- Client Credentials (machine-to-machine)
- Authorization Code (with PKCE)

## 5. Rate Limiting

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 73
X-RateLimit-Reset: 1234567890
```

Rate limits per endpoint:
- `GET /models/*`: 10/minute
- `POST /updates`: 5/minute
- `GET /rounds/*`: 20/minute

## 6. Pagination

```http
GET /api/v1/rounds?page=2&per_page=50

Response:
{
  "data": [...],
  "pagination": {
    "page": 2,
    "per_page": 50,
    "total_pages": 10,
    "total_items": 500
  },
  "links": {
    "first": "/api/v1/rounds?page=1",
    "prev": "/api/v1/rounds?page=1",
    "next": "/api/v1/rounds?page=3",
    "last": "/api/v1/rounds?page=10"
  }
}
```

## 7. Error Responses

Standard error format:

```json
{
  "error": {
    "code": "INVALID_MODEL_FORMAT",
    "message": "Model update format validation failed",
    "details": {
      "field": "weights.encoding",
      "expected": "float32",
      "received": "float64"
    },
    "request_id": "uuid",
    "timestamp": "ISO-8601"
  }
}
```

HTTP Status Codes:
- `400 Bad Request`: Invalid input
- `401 Unauthorized`: Missing/invalid authentication
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource doesn't exist
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary unavailability

## 8. Monitoring & Observability

### 8.1 Health Check

```http
GET /health

Response 200:
{
  "status": "healthy",
  "version": "1.0.0",
  "uptime_seconds": 86400,
  "services": {
    "database": "healthy",
    "storage": "healthy",
    "aggregation": "healthy"
  }
}
```

### 8.2 Metrics Endpoint (Prometheus)

```http
GET /metrics

Response 200:
# HELP fl_active_clients Number of active clients
# TYPE fl_active_clients gauge
fl_active_clients 12453

# HELP fl_training_rounds_total Total training rounds
# TYPE fl_training_rounds_total counter
fl_training_rounds_total 142

# HELP fl_aggregation_duration_seconds Time to aggregate updates
# TYPE fl_aggregation_duration_seconds histogram
fl_aggregation_duration_seconds_bucket{le="10"} 45
fl_aggregation_duration_seconds_bucket{le="30"} 120
fl_aggregation_duration_seconds_sum 2456.7
fl_aggregation_duration_seconds_count 142
```

## 9. SDK Examples

### 9.1 Python Client

```python
from wia_fl import FederatedLearningClient

client = FederatedLearningClient(
    server_url='https://fl.example.com',
    api_key='your-api-key'
)

# Register
client.register(device_id='device-123')

# Participate in training
@client.on_training_invitation
async def train(round_info):
    model = await client.download_model(round_info.model_url)
    updated_model = train_locally(model, local_data)
    await client.submit_update(updated_model)

client.start()
```

### 9.2 TypeScript Client

```typescript
import { FederatedLearningClient } from '@wia/federated-learning';

const client = new FederatedLearningClient({
  serverUrl: 'https://fl.example.com',
  apiKey: 'your-api-key'
});

await client.register({ deviceId: 'device-123' });

client.on('training_invitation', async (round) => {
  const model = await client.downloadModel(round.modelUrl);
  const update = await trainLocally(model);
  await client.submitUpdate(update);
});

await client.start();
```

## 10. API Versioning

URL-based versioning:
- `/api/v1/...` - Current stable version
- `/api/v2/...` - Next major version (when available)

Header-based versioning (alternative):
```http
Accept: application/vnd.wia.fl.v1+json
```

---

**Copyright © 2025 SmileStory Inc. / World Certification Industry Association**
**弘益人間 (Hongik Ingan) · Benefit All Humanity**

**License:** CC BY 4.0
