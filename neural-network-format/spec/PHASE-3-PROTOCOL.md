# WIA-AI-014: Neural Network Format - Phase 3 Protocol Specification

**Version:** 1.0.0
**Philosophy:** 弘益人間 - Benefit All Humanity

## 1. Network Protocol

### 1.1 Model Serving Protocol

```protobuf
service ModelService {
    rpc Predict(PredictRequest) returns (PredictResponse);
    rpc PredictStream(stream PredictRequest) returns (stream PredictResponse);
    rpc GetModelMetadata(MetadataRequest) returns (MetadataResponse);
    rpc GetModelStatus(StatusRequest) returns (StatusResponse);
}

message PredictRequest {
    string model_name = 1;
    string version = 2;
    map<string, Tensor> inputs = 3;
    PredictOptions options = 4;
}

message PredictResponse {
    map<string, Tensor> outputs = 1;
    PredictMetrics metrics = 2;
}
```

### 1.2 Model Registry Protocol

```protobuf
service RegistryService {
    rpc RegisterModel(RegisterRequest) returns (RegisterResponse);
    rpc GetModel(GetModelRequest) returns (GetModelResponse);
    rpc ListModels(ListRequest) returns (ListResponse);
    rpc DeleteModel(DeleteRequest) returns (DeleteResponse);
}
```

## 2. HTTP REST API

### 2.1 Prediction Endpoint

```
POST /v1/models/{model_name}/versions/{version}:predict
Content-Type: application/json

{
  "inputs": {
    "input": [[1.0, 2.0, 3.0]]
  }
}

Response:
{
  "outputs": {
    "output": [[0.8, 0.1, 0.1]]
  },
  "latency_ms": 15.2
}
```

### 2.2 Model Management

```
GET    /v1/models
POST   /v1/models
GET    /v1/models/{name}
DELETE /v1/models/{name}/versions/{version}
PUT    /v1/models/{name}/versions/{version}
```

## 3. Model Repository Format

```
model_repository/
├── {model_name}/
│   ├── config.json
│   └── {version}/
│       ├── model.wia
│       └── metadata.json
```

## 4. Security

### 4.1 Authentication

- API Keys
- OAuth 2.0
- mTLS (mutual TLS)

### 4.2 Authorization

```json
{
  "permissions": {
    "model_name": {
      "read": ["user1", "user2"],
      "write": ["admin"],
      "execute": ["service_account"]
    }
  }
}
```

## 5. Wire Format & Transport

WIA-AI-014 model serving MUST be available over at least one of the following transports:

| Transport | Status | Use case |
|-----------|--------|----------|
| gRPC over HTTP/2 (TLS 1.3) | RECOMMENDED | Internal RPC, high throughput |
| HTTP/1.1 + JSON over TLS 1.3 | REQUIRED | Public API surface |
| gRPC-Web | OPTIONAL | Browser clients |
| Server-Sent Events | OPTIONAL | Streaming token output |

All payloads MUST be UTF-8 encoded JSON (RFC 8259) when surfaced over HTTP/1.1. Binary tensors SHOULD use the `application/x-wia-tensor` media type with the framing defined in §6.

## 6. Tensor Wire Framing

### 6.1 Header

```
+------+--------+----------+------------+--------+--------+
| ver  | dtype  |  rank    | dim[0..r]  | layout | length |
| u8   | u8     |  u8      |  u32...    | u8     | u64    |
+------+--------+----------+------------+--------+--------+
```

| Field | Width | Notes |
|-------|-------|-------|
| ver | 1 | 0x01 |
| dtype | 1 | 0x01=fp32 0x02=fp16 0x03=bf16 0x04=int8 0x05=uint8 0x06=int32 0x07=int64 0x08=fp8e4m3 0x09=fp8e5m2 |
| rank | 1 | 1–8 |
| dim[i] | 4 | u32 little-endian, rank entries |
| layout | 1 | 0=row-major NCHW 1=NHWC 2=NDHWC |
| length | 8 | bytes that follow |

### 6.2 Quantization Side Channel

When `dtype` ∈ {int8, uint8, fp8e4m3, fp8e5m2} a sidecar JSON object MUST accompany the request:

```json
{
  "quantization": {
    "scheme": "per-channel-symmetric",
    "scales": [0.0078125, 0.00625, ...],
    "zeroPoints": [0, 0, ...],
    "axis": 0
  }
}
```

## 7. Streaming Inference Protocol

### 7.1 Bi-directional gRPC Stream

`PredictStream(stream PredictRequest) returns (stream PredictResponse)` — used for autoregressive language models, online time-series prediction, and chunked vision pipelines. Servers MUST honour the following back-pressure rules:

1. Inbound queue depth MUST be advertised at handshake; clients MUST NOT exceed it.
2. Servers MUST emit a `PredictMetrics` summary every 16 outputs or 250 ms, whichever comes first.
3. Either side MAY end the stream cleanly with a `terminate` sentinel; abort SHOULD be reserved for fault paths.

### 7.2 SSE Stream Format

```
event: token
data: {"index":12,"text":"Hello","logprob":-0.42}

event: usage
data: {"promptTokens":18,"completionTokens":12,"latencyMs":94}

event: end
data: {"finishReason":"stop"}
```

## 8. Health, Liveness, and Drain Protocol

| Endpoint | Method | Semantics |
|----------|--------|-----------|
| `/healthz` | GET | Process-level liveness; 200 = OK |
| `/readyz` | GET | Model loaded and warm; 200 = ready |
| `/drain` | POST | Reject new requests, finish in-flight |
| `/v1/models/{name}:warmup` | POST | Pre-fill caches with sample tensors |

`/drain` MUST honour `Retry-After`. Orchestrators (Kubernetes, Nomad) MUST send `/drain` before terminating a pod and wait for `/readyz` to return 503 before issuing SIGTERM.

## 9. Error Model

Errors over HTTP MUST follow RFC 9457 (`application/problem+json`). gRPC errors MUST use canonical `google.rpc.Status` with the `details` array carrying `wia.ai-014.ErrorInfo`:

```json
{
  "type": "https://standards.wia.example/neural-network-format/errors/oom",
  "title": "Out of GPU memory",
  "status": 503,
  "modelName": "resnet50",
  "version": "2.1.0",
  "detail": "Requested batch exceeds device capacity (32 GiB).",
  "retryable": true,
  "retryAfterMs": 1000
}
```

| Code | Type slug | Retryable |
|------|-----------|-----------|
| 400 | `invalid-tensor-shape` | No |
| 401 | `unauthenticated` | No |
| 403 | `quota-exhausted` | After reset |
| 404 | `model-not-found` | No |
| 409 | `version-conflict` | No |
| 413 | `request-too-large` | No |
| 429 | `rate-limited` | Yes |
| 503 | `model-not-ready` / `oom` | Yes |
| 504 | `inference-timeout` | Yes |

## 10. Authentication & Trust

- mTLS SHOULD use SPIFFE-issued SVIDs for service-to-service traffic.
- OAuth 2.0 access tokens MUST follow RFC 6749 + RFC 6750. Bearer scope MUST include `model:predict:<name>` and SHOULD be granular per version.
- Long-running offline batch jobs MAY use a signed Job Manifest with detached JWS (RFC 7515).
- All tokens MUST be validated using a JWKS endpoint published at `/.well-known/jwks.json` of the issuer.

## 11. Batching Protocol

Servers SHOULD support dynamic batching to amortize device launch overhead. Batches MUST be assembled from concurrent requests of the same `(model_name, version, dtype, shape_signature)` tuple.

```
PendingQueue --[max_wait_ms]--> assemble_batch
                                  |
                                  v
                              run_inference --> split_outputs --> respond_individually
```

Batch policy parameters:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `max_batch_size` | 32 | per-model override |
| `max_wait_ms` | 4 | latency budget |
| `pad_strategy` | `none` / `right` / `bucket` | LM contexts |

Each batched response MUST carry per-input metrics: queue wait, batch wait, device time.

## 12. Compatibility Matrix

| Client SDK | Server | Status |
|-----------|--------|--------|
| 1.0.x | 1.0.x | Full |
| 1.0.x | 1.1.x | Full (new methods ignored) |
| 1.1.x | 1.0.x | `tools/list_changed` notifications dropped silently |
| 2.x | 1.x | INCOMPATIBLE — wire format break |

A connecting peer MUST exchange a `wia/handshake` envelope on every new transport so version mismatches surface before the first inference call.

```json
{
  "wiaProtocolVersion": "1.1.0",
  "supportedDtypes": ["fp32","fp16","bf16","int8","fp8e4m3"],
  "supportedTransports": ["grpc","http+sse"],
  "supportedAuth": ["mtls-spiffe","oauth2"],
  "maxBatchSize": 32,
  "maxConcurrentStreams": 256,
  "extensions": ["dynamic-batching","speculative-decoding"]
}
```

## 13. Quotas & Fairness

Servers MUST publish per-tenant quotas on the `/v1/quotas` endpoint:

```json
{
  "tenant": "acme",
  "qps": 200,
  "tokensPerSecond": 60000,
  "concurrentRequests": 32,
  "burstWindowMs": 1000
}
```

Throttle decisions MUST use a token-bucket algorithm with the published refill rate. Each rejected request MUST receive HTTP 429 with `Retry-After` ≥ 1 second and an `X-WIA-Quota` header pointing at the offending dimension.

## 14. Streaming Speculative Decoding Protocol

For text-generation models that ship draft+target speculative decoding:

```
client                     server
   |                          |
   |--PredictStream--------->|
   |   {prompt, draftK=4}    |
   |<-tokens (verified)------|
   |<-tokens (verified)------|
   |<-event: speculation_summary
   |    {acceptedRatio: 0.62, draftK: 4, ...}
```

Servers MUST honour the request-level `draftK` hint when present and MUST publish per-batch `acceptedRatio` so clients can adapt their strategy.

## 15. Model Pinning & Graceful Drain

Long-lived clients (RAG retrievers, agent harnesses) often pin a specific model version for reproducibility. Pinning is expressed via the `X-WIA-Model-Pin` header carrying a SHA-256 digest:

```
X-WIA-Model-Pin: sha256-7e1c...c0d3
```

Servers MUST refuse the request with 409 `model-pin-mismatch` if the active version digest no longer matches. During scheduled rotation operators MUST notify pinned clients via the OpenAPI `Sunset` header (RFC 8594) at least 24 hours in advance.

## 16. Logging Schema

Per-request access logs MUST include:

| Field | Type | Notes |
|-------|------|-------|
| `ts` | RFC3339 string | UTC, millisecond precision |
| `model` | string |  |
| `version` | semver |  |
| `requestId` | ULID |  |
| `tenant` | string | OAuth subject or bearer hash |
| `latencyMs` | float | end-to-end |
| `queueWaitMs` | float | scheduler wait |
| `deviceMs` | float | accelerator time |
| `inputBytes` | int64 |  |
| `outputBytes` | int64 |  |
| `status` | int | HTTP code |
| `errorType` | string | RFC 9457 type slug if status ≥ 400 |

Logs SHOULD be JSON-lines over stdout for container deployments and forwarded to the platform aggregator (Loki, Cloud Logging, Datadog) without intermediate parsing.

## 17. Compliance Conformance Trace

Phase 3 ships sample traces under `cli/test-vectors/phase-3/`:

- `tensor-framing.bin` — wire framing for fp32, fp16, int8 tensors
- `streaming-llm.jsonl` — full duplex SSE token stream
- `error-rfc9457.json` — every status code with the canonical body
- `quota-token-bucket.csv` — millisecond-resolution refill curve
- `pin-mismatch.http` — request/response capture of a pin failure

Implementations MUST replay each vector with the published expectations to claim WIA-AI-014 Phase 3 conformance.

## 18. Connection Multiplexing

A single TLS connection MAY carry multiple concurrent streams (HTTP/2 default). Servers MUST advertise their stream limits at handshake:

```json
{
  "maxConcurrentStreams": 256,
  "initialStreamWindow": 65536,
  "initialConnectionWindow": 1048576
}
```

Clients MUST honour the announced flow-control windows and MUST NOT open additional streams beyond the limit until previous streams are closed. A misbehaving client SHOULD be terminated with `GOAWAY` and `ENHANCE_YOUR_CALM`.

## 13. Normative References

- IETF RFC 8259 — JSON Data Interchange Format
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7515 — JSON Web Signature
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 / RFC 6750 — OAuth 2.0
- IETF RFC 9457 — Problem Details for HTTP APIs
- gRPC Specification — gRPC.io (`google.rpc.Status`)
- WHATWG HTML — Server-Sent Events
- ONNX Operator Set v18 (reference for tensor shape semantics)

---

**Copyright © 2025 WIA**
