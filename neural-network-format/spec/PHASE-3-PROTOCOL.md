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

---

**Copyright © 2025 WIA**
