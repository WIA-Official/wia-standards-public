# WIA-LLM-INTEROP: Phase 3 - Protocol Specification

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Philosophy**: 홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document defines the communication protocols for WIA-LLM-INTEROP:

| Protocol | Use Case | Transport |
|----------|----------|-----------|
| REST API | Request-Response | HTTP/HTTPS |
| WebSocket | Real-time, Streaming | WSS |
| gRPC | High-performance | HTTP/2 |

All protocols share the same message format (Phase 1) and algorithms (Phase 2).

---

## 2. REST API Protocol

### 2.1 Base URL Pattern

```
https://{host}/wia/llm-interop/v1/
```

### 2.2 Authentication

All requests MUST include authentication:

```http
Authorization: Bearer {api_key}
# OR
Authorization: DID-Auth {did_token}
# OR
X-WIA-API-Key: {api_key}
```

### 2.3 Common Headers

**Request Headers**:
```http
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0
X-WIA-Trace-ID: {uuid}
X-WIA-Model-ID: {sender_model_id}
Accept-Language: ko, en;q=0.9
```

**Response Headers**:
```http
Content-Type: application/json
X-WIA-Version: 1.0
X-WIA-Trace-ID: {uuid}
X-WIA-Request-ID: {uuid}
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1702723200
```

### 2.4 Capability Endpoints

#### GET /capability
Get own capability document.

```http
GET /wia/llm-interop/v1/capability
Authorization: Bearer {api_key}
```

Response:
```json
{
  "success": true,
  "data": {
    "wia_version": "1.0",
    "document_type": "capability",
    "model": {
      "model_id": "did:wia:llm:provider:model",
      "display_name": "Model Name"
    },
    ...
  }
}
```

#### GET /capability/{model_id}
Get capability of another AI.

```http
GET /wia/llm-interop/v1/capability/did:wia:llm:anthropic:claude-opus
```

#### POST /capability/query
Find AIs matching capability requirements.

```http
POST /wia/llm-interop/v1/capability/query
Content-Type: application/json

{
  "required_domains": ["medical", "legal"],
  "min_level": 3,
  "required_languages": ["ko", "en"],
  "limit": 10
}
```

Response:
```json
{
  "success": true,
  "data": {
    "matches": [
      {
        "model_id": "did:wia:llm:provider:model",
        "score": 0.95,
        "capability_uri": "https://..."
      }
    ],
    "total_count": 15
  }
}
```

### 2.5 Message Endpoints

#### POST /message
Send a message to another AI.

```http
POST /wia/llm-interop/v1/message
Content-Type: application/json

{
  "wia_version": "1.0",
  "document_type": "message",
  "message_id": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": "2025-12-16T10:30:00Z",
  "trace_id": "550e8400-e29b-41d4-a716-446655440000",

  "from": {
    "model_id": "did:wia:llm:client:my-ai"
  },
  "to": {
    "model_id": "did:wia:llm:anthropic:claude-opus"
  },

  "type": "query",
  "priority": "normal",

  "payload": {
    "intent": "question",
    "content": "What is the capital of Korea?",
    "constraints": {
      "response_format": "text",
      "max_tokens": 100
    }
  }
}
```

Response:
```json
{
  "success": true,
  "data": {
    "message_id": "550e8400-e29b-41d4-a716-446655440002",
    "type": "response",
    "payload": {
      "content": "The capital of Korea (South Korea) is Seoul.",
      "confidence": 0.99,
      "usage": {
        "input_tokens": 15,
        "output_tokens": 12,
        "processing_time_ms": 234
      }
    }
  }
}
```

#### POST /message/stream
Send a message and receive streaming response.

```http
POST /wia/llm-interop/v1/message/stream
Content-Type: application/json
Accept: text/event-stream

{
  "type": "query",
  "payload": {
    "content": "Explain quantum computing",
    "constraints": {
      "response_format": "markdown"
    }
  }
}
```

Response (SSE):
```
event: chunk
data: {"chunk_index": 0, "content": "Quantum computing is", "is_final": false}

event: chunk
data: {"chunk_index": 1, "content": " a type of computation", "is_final": false}

event: chunk
data: {"chunk_index": 2, "content": "...", "is_final": true, "final_metadata": {...}}

event: done
data: {"message_id": "...", "usage": {...}}
```

### 2.6 Federation Endpoints

#### POST /federation
Create a new federation.

```http
POST /wia/llm-interop/v1/federation
Content-Type: application/json

{
  "name": "Medical-Legal Analysis Federation",
  "topology": "star",
  "config": {
    "task_config": {
      "auto_decompose": true,
      "max_subtasks": 5
    },
    "consensus_config": {
      "algorithm": "weighted",
      "quorum_percentage": 0.6
    }
  }
}
```

#### POST /federation/{federation_id}/join
Join an existing federation.

```http
POST /wia/llm-interop/v1/federation/550e8400.../join
Content-Type: application/json

{
  "capability": {...},
  "requested_role": "specialist",
  "domains": ["medical"]
}
```

#### POST /federation/{federation_id}/task
Submit a task to the federation.

```http
POST /wia/llm-interop/v1/federation/550e8400.../task
Content-Type: application/json

{
  "original_query": "환자 A의 치료비 보험 청구 가능 여부를 분석해주세요",
  "priority": "high",
  "constraints": {
    "response_language": "ko",
    "timeout_seconds": 60
  }
}
```

Response:
```json
{
  "success": true,
  "data": {
    "task_id": "550e8400-...",
    "status": "queued",
    "estimated_completion_seconds": 30,
    "subtasks": [
      {"task_id": "...", "description": "의료 기록 분석", "assigned_to": "medical-ai"},
      {"task_id": "...", "description": "보험 약관 검토", "assigned_to": "legal-ai"}
    ]
  }
}
```

#### GET /federation/{federation_id}/task/{task_id}
Get task status and result.

```http
GET /wia/llm-interop/v1/federation/550e8400.../task/660e8400...
```

### 2.7 Consensus Endpoints

#### POST /federation/{federation_id}/consensus
Initiate consensus voting.

```http
POST /wia/llm-interop/v1/federation/550e8400.../consensus
Content-Type: application/json

{
  "question": {
    "type": "multiple_choice",
    "description": "환자에게 수술이 필요한가요?",
    "options": [
      {"option_id": "yes", "description": "예, 수술 필요"},
      {"option_id": "no", "description": "아니오, 보존적 치료"},
      {"option_id": "uncertain", "description": "추가 검사 필요"}
    ]
  },
  "deadline_seconds": 30
}
```

### 2.8 Error Responses

All errors follow this format:

```json
{
  "success": false,
  "error": {
    "code": "E300",
    "message": "Rate limit exceeded",
    "details": {
      "limit": 100,
      "remaining": 0,
      "reset_at": "2025-12-16T11:00:00Z"
    },
    "recoverable": true,
    "retry_after_seconds": 60
  }
}
```

HTTP Status Codes:
| Status | Meaning |
|--------|---------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request (E2xx) |
| 401 | Unauthorized (E4xx) |
| 403 | Forbidden (E402) |
| 404 | Not Found |
| 429 | Rate Limited (E300) |
| 500 | Internal Error (E5xx) |
| 503 | Service Unavailable (E500) |

---

## 3. WebSocket Protocol

### 3.1 Connection

```javascript
const ws = new WebSocket('wss://{host}/wia/llm-interop/v1/ws');

// Or with authentication in URL (not recommended for production)
const ws = new WebSocket('wss://{host}/wia/llm-interop/v1/ws?token={api_key}');
```

### 3.2 Handshake Sequence

```
Client                          Server
   |                               |
   |-------- [Connect] ----------->|
   |                               |
   |<----- [Welcome Message] ------|
   |                               |
   |-------- [Auth Message] ------>|
   |                               |
   |<------ [Auth Response] -------|
   |                               |
   |------ [Capability Msg] ------>|
   |                               |
   |<---- [Capability Response] ---|
   |                               |
   |====== [Session Active] =======|
```

### 3.3 Message Frame Format

All WebSocket messages are JSON:

```json
{
  "type": "message_type",
  "id": "unique_message_id",
  "timestamp": "2025-12-16T10:30:00Z",
  "payload": {}
}
```

### 3.4 Message Types

#### Welcome (Server → Client)
```json
{
  "type": "welcome",
  "id": "msg_001",
  "payload": {
    "server_id": "did:wia:llm:server:1",
    "wia_version": "1.0",
    "supported_features": ["streaming", "federation", "consensus"]
  }
}
```

#### Auth (Client → Server)
```json
{
  "type": "auth",
  "id": "msg_002",
  "payload": {
    "method": "api_key",
    "credentials": {
      "api_key": "sk-..."
    }
  }
}
```

#### Auth Response (Server → Client)
```json
{
  "type": "auth_response",
  "id": "msg_003",
  "payload": {
    "success": true,
    "session_id": "sess_123",
    "expires_at": "2025-12-16T12:00:00Z"
  }
}
```

#### Query (Client → Server)
```json
{
  "type": "query",
  "id": "msg_004",
  "payload": {
    "to": {
      "model_id": "did:wia:llm:anthropic:claude-opus"
    },
    "content": "Hello, how are you?",
    "stream": true
  }
}
```

#### Stream Chunk (Server → Client)
```json
{
  "type": "stream",
  "id": "msg_004",  // Same ID as query
  "payload": {
    "chunk_index": 0,
    "content": "I'm doing well",
    "is_final": false
  }
}
```

#### Stream End (Server → Client)
```json
{
  "type": "stream_end",
  "id": "msg_004",
  "payload": {
    "content": "I'm doing well, thank you for asking!",
    "confidence": 0.95,
    "usage": {
      "input_tokens": 8,
      "output_tokens": 12
    }
  }
}
```

#### Subscribe (Client → Server)
Subscribe to events:
```json
{
  "type": "subscribe",
  "id": "msg_005",
  "payload": {
    "events": ["federation.task.completed", "consensus.vote.needed"],
    "federation_id": "550e8400-..."
  }
}
```

#### Event (Server → Client)
```json
{
  "type": "event",
  "id": "evt_001",
  "payload": {
    "event_type": "federation.task.completed",
    "data": {
      "task_id": "...",
      "result": {...}
    }
  }
}
```

#### Ping/Pong (Keepalive)
```json
// Client → Server
{"type": "ping", "id": "ping_001"}

// Server → Client
{"type": "pong", "id": "ping_001"}
```

### 3.5 Error Handling

```json
{
  "type": "error",
  "id": "msg_004",  // Reference to failed message
  "payload": {
    "code": "E302",
    "message": "Request timeout",
    "recoverable": true
  }
}
```

### 3.6 Connection Close Codes

| Code | Meaning |
|------|---------|
| 1000 | Normal closure |
| 1001 | Going away |
| 1008 | Policy violation |
| 4000 | Authentication failed |
| 4001 | Rate limited |
| 4002 | Invalid message |

---

## 4. gRPC Protocol

### 4.1 Proto Definition

```protobuf
syntax = "proto3";

package wia.llm.interop.v1;

import "google/protobuf/timestamp.proto";
import "google/protobuf/struct.proto";

// ==================== Services ====================

service LLMInteropService {
  // Capability
  rpc GetCapability(GetCapabilityRequest) returns (CapabilityResponse);
  rpc QueryCapabilities(QueryCapabilitiesRequest) returns (QueryCapabilitiesResponse);

  // Messaging
  rpc SendMessage(MessageEnvelope) returns (MessageEnvelope);
  rpc StreamMessage(MessageEnvelope) returns (stream StreamChunk);

  // Federation
  rpc CreateFederation(CreateFederationRequest) returns (FederationResponse);
  rpc JoinFederation(JoinFederationRequest) returns (JoinFederationResponse);
  rpc SubmitTask(SubmitTaskRequest) returns (TaskResponse);
  rpc GetTaskResult(GetTaskResultRequest) returns (TaskResultResponse);

  // Bidirectional streaming for real-time
  rpc BiDirectionalStream(stream MessageEnvelope) returns (stream MessageEnvelope);
}

// ==================== Messages ====================

message GetCapabilityRequest {
  string model_id = 1;  // Optional, empty = self
}

message CapabilityResponse {
  bool success = 1;
  CapabilityDocument capability = 2;
  Error error = 3;
}

message CapabilityDocument {
  string wia_version = 1;
  string document_id = 2;
  ModelIdentity model = 3;
  int32 level = 4;
  repeated DomainExpertise domains = 5;
  repeated LanguageSupport languages = 6;
  ModalitySupport modalities = 7;
  ResourceLimits limits = 8;
  TrustMetadata trust = 9;
}

message ModelIdentity {
  string model_id = 1;
  string display_name = 2;
  string provider_id = 3;
  string version = 4;
}

message DomainExpertise {
  string domain = 1;
  float proficiency = 2;
  repeated string sub_domains = 3;
}

message LanguageSupport {
  string code = 1;
  float proficiency = 2;
}

message ModalitySupport {
  TextModality text = 1;
  ImageModality image = 2;
  AudioModality audio = 3;
}

message TextModality {
  bool input = 1;
  bool output = 2;
  int32 max_input_length = 3;
  int32 max_output_length = 4;
  bool streaming = 5;
}

message ImageModality {
  bool input = 1;
  bool output = 2;
  bool ocr = 3;
}

message AudioModality {
  bool input = 1;
  bool output = 2;
  bool transcription = 3;
}

message ResourceLimits {
  int32 max_input_tokens = 1;
  int32 max_output_tokens = 2;
  int32 max_context_window = 3;
  int32 requests_per_minute = 4;
  int32 tokens_per_minute = 5;
}

message TrustMetadata {
  float confidence_threshold = 1;
  google.protobuf.Timestamp knowledge_cutoff = 2;
}

message QueryCapabilitiesRequest {
  repeated string required_domains = 1;
  int32 min_level = 2;
  repeated string required_languages = 3;
  int32 limit = 4;
}

message QueryCapabilitiesResponse {
  bool success = 1;
  repeated CapabilityMatch matches = 2;
  int32 total_count = 3;
}

message CapabilityMatch {
  string model_id = 1;
  float score = 2;
  string capability_uri = 3;
}

message MessageEnvelope {
  string wia_version = 1;
  string message_id = 2;
  google.protobuf.Timestamp timestamp = 3;
  string trace_id = 4;
  string parent_message_id = 5;

  MessageParticipant from = 6;
  MessageTarget to = 7;

  string type = 8;  // "query", "response", "error", etc.
  string priority = 9;
  int32 ttl_seconds = 10;

  oneof payload {
    QueryPayload query = 11;
    ResponsePayload response = 12;
    ErrorPayload error = 13;
    HandshakePayload handshake = 14;
  }

  google.protobuf.Struct metadata = 15;
}

message MessageParticipant {
  string model_id = 1;
  string capability_uri = 2;
}

message MessageTarget {
  string model_id = 1;
  CapabilityQuery capability_query = 2;
}

message CapabilityQuery {
  repeated string required_domains = 1;
  int32 min_level = 2;
  repeated string required_languages = 3;
}

message QueryPayload {
  string intent = 1;
  string content = 2;
  repeated ConversationContext context = 3;
  QueryConstraints constraints = 4;
  repeated Attachment attachments = 5;
}

message ConversationContext {
  string role = 1;
  string content = 2;
  string model_id = 3;
}

message QueryConstraints {
  string response_format = 1;
  int32 max_tokens = 2;
  string response_language = 3;
  float confidence_min = 4;
  int32 timeout_seconds = 5;
}

message Attachment {
  string attachment_id = 1;
  string type = 2;
  bytes content = 3;
  string url = 4;
  string mime_type = 5;
}

message ResponsePayload {
  string content = 1;
  float confidence = 2;
  string reasoning = 3;
  repeated Source sources = 4;
  UsageStats usage = 5;
}

message Source {
  string title = 1;
  string url = 2;
  string excerpt = 3;
}

message UsageStats {
  int32 input_tokens = 1;
  int32 output_tokens = 2;
  int32 processing_time_ms = 3;
}

message ErrorPayload {
  string code = 1;
  string message = 2;
  bool recoverable = 3;
  int32 retry_after_seconds = 4;
}

message HandshakePayload {
  string phase = 1;
  CapabilityDocument capability = 2;
  repeated string supported_versions = 3;
}

message StreamChunk {
  int32 chunk_index = 1;
  int32 total_chunks = 2;
  string content = 3;
  bool is_final = 4;
  ResponsePayload final_metadata = 5;
}

message Error {
  string code = 1;
  string message = 2;
  google.protobuf.Struct details = 3;
}

// Federation messages
message CreateFederationRequest {
  string name = 1;
  string topology = 2;
  FederationConfig config = 3;
}

message FederationConfig {
  TaskConfig task_config = 1;
  ConsensusConfig consensus_config = 2;
}

message TaskConfig {
  bool auto_decompose = 1;
  int32 max_subtasks = 2;
  int32 max_parallel_tasks = 3;
}

message ConsensusConfig {
  string algorithm = 1;
  float quorum_percentage = 2;
  int32 voting_timeout_seconds = 3;
}

message FederationResponse {
  bool success = 1;
  string federation_id = 2;
  Error error = 3;
}

message JoinFederationRequest {
  string federation_id = 1;
  CapabilityDocument capability = 2;
  string requested_role = 3;
  repeated string domains = 4;
}

message JoinFederationResponse {
  bool success = 1;
  string member_id = 2;
  string assigned_role = 3;
  Error error = 4;
}

message SubmitTaskRequest {
  string federation_id = 1;
  string original_query = 2;
  string priority = 3;
  TaskConstraints constraints = 4;
}

message TaskConstraints {
  string response_language = 1;
  int32 timeout_seconds = 2;
}

message TaskResponse {
  bool success = 1;
  string task_id = 2;
  string status = 3;
  repeated SubtaskInfo subtasks = 4;
  Error error = 5;
}

message SubtaskInfo {
  string task_id = 1;
  string description = 2;
  string assigned_to = 3;
}

message GetTaskResultRequest {
  string federation_id = 1;
  string task_id = 2;
}

message TaskResultResponse {
  bool success = 1;
  string task_id = 2;
  string status = 3;
  TaskResult result = 4;
  Error error = 5;
}

message TaskResult {
  string content = 1;
  float confidence = 2;
  UsageStats usage = 3;
}
```

### 4.2 gRPC Client Example

```rust
use tonic::transport::Channel;

// Generated from proto
use wia_llm_interop::llm_interop_service_client::LlmInteropServiceClient;
use wia_llm_interop::{MessageEnvelope, QueryPayload};

async fn send_query(client: &mut LlmInteropServiceClient<Channel>) {
    let request = MessageEnvelope {
        wia_version: "1.0".to_string(),
        message_id: uuid::Uuid::now_v7().to_string(),
        r#type: "query".to_string(),
        payload: Some(message_envelope::Payload::Query(QueryPayload {
            intent: "question".to_string(),
            content: "What is AI?".to_string(),
            ..Default::default()
        })),
        ..Default::default()
    };

    let response = client.send_message(request).await?;
    println!("Response: {:?}", response);
}
```

---

## 5. Security Protocol

### 5.1 DID-Auth (Recommended)

WIA-LLM-INTEROP recommends DID-based authentication:

```
DID-Auth Flow:
1. Client generates challenge nonce
2. Client signs nonce with private key
3. Server verifies signature using DID document
4. Session token issued
```

#### DID-Auth Header
```http
Authorization: DID-Auth eyJhbGciOiJFZERTQSIsInR5cCI6IkpXVCJ9...
```

#### DID-Auth Token Structure
```json
{
  "alg": "EdDSA",
  "typ": "JWT"
}
.
{
  "iss": "did:wia:llm:provider:model",
  "sub": "did:wia:llm:provider:model",
  "aud": "https://api.example.com",
  "iat": 1702723200,
  "exp": 1702726800,
  "nonce": "abc123"
}
.
{signature}
```

### 5.2 Message Signing

All critical messages SHOULD be signed:

```json
{
  "wia_version": "1.0",
  "message_id": "...",
  "payload": {...},
  "signature": {
    "algorithm": "ed25519",
    "public_key_id": "did:wia:llm:provider:model#key-1",
    "signature": "base64_encoded_signature",
    "timestamp": "2025-12-16T10:30:00Z"
  }
}
```

### 5.3 Encryption

For sensitive data, use envelope encryption:

```json
{
  "encrypted": true,
  "encryption": {
    "algorithm": "aes-256-gcm",
    "key_id": "recipient_key_id",
    "encrypted_key": "base64_rsa_encrypted_aes_key",
    "iv": "base64_initialization_vector"
  },
  "ciphertext": "base64_encrypted_payload"
}
```

---

## 6. Rate Limiting

### 6.1 Rate Limit Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1702723200
X-RateLimit-Window: 60
```

### 6.2 Rate Limit Response

```http
HTTP/1.1 429 Too Many Requests
Retry-After: 45

{
  "error": {
    "code": "E300",
    "message": "Rate limit exceeded",
    "retry_after_seconds": 45
  }
}
```

### 6.3 Backoff Strategy

```
Exponential backoff with jitter:

attempt 1: wait 1s + random(0, 0.5)
attempt 2: wait 2s + random(0, 1)
attempt 3: wait 4s + random(0, 2)
attempt 4: wait 8s + random(0, 4)
max wait: 60s
max attempts: 5
```

---

## 7. Health Check Protocol

### 7.1 Health Endpoint

```http
GET /wia/llm-interop/v1/health
```

Response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "model_id": "did:wia:llm:provider:model",
  "uptime_seconds": 86400,
  "checks": {
    "model": "healthy",
    "database": "healthy",
    "external_services": "healthy"
  }
}
```

### 7.2 Federation Health

```http
GET /wia/llm-interop/v1/federation/{federation_id}/health
```

Response:
```json
{
  "status": "healthy",
  "federation_id": "550e8400-...",
  "topology": "star",
  "members": {
    "total": 5,
    "active": 4,
    "unavailable": 1
  },
  "orchestrator": {
    "model_id": "did:wia:llm:...",
    "status": "active"
  }
}
```

---

## 8. Discovery Protocol

### 8.1 Well-Known Endpoint

```http
GET /.well-known/wia-llm-interop
```

Response:
```json
{
  "wia_llm_interop_version": "1.0",
  "endpoints": {
    "capability": "https://api.example.com/wia/llm-interop/v1/capability",
    "message": "https://api.example.com/wia/llm-interop/v1/message",
    "websocket": "wss://api.example.com/wia/llm-interop/v1/ws",
    "grpc": "grpc://api.example.com:443"
  },
  "supported_auth": ["api_key", "did_auth", "oauth2"],
  "documentation": "https://docs.example.com/wia-llm-interop"
}
```

### 8.2 Service Registration

AIs can register with a WIA registry:

```http
POST https://registry.wia.org/v1/register
Content-Type: application/json

{
  "capability": {...},
  "endpoints": {
    "api": "https://api.example.com/wia/llm-interop/v1",
    "websocket": "wss://api.example.com/wia/llm-interop/v1/ws"
  }
}
```

---

## 9. Protocol Versioning

### 9.1 Version Header
```http
X-WIA-Version: 1.0
Accept-Version: 1.0, 1.1
```

### 9.2 Version Negotiation

If client requests unsupported version:

```http
HTTP/1.1 400 Bad Request

{
  "error": {
    "code": "E200",
    "message": "Unsupported version",
    "details": {
      "requested": "2.0",
      "supported": ["1.0", "1.1"]
    }
  }
}
```

---

**Document ID**: WIA-LLM-INTEROP-PHASE-3
**Version**: 1.0.0
**Date**: 2025-12-16
**Philosophy**: 홍익인간 (弘益人間) - 프로토콜로 AI들이 소통합니다
