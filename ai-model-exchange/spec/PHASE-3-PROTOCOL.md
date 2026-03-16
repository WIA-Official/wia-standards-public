# WIA-AI-008 PHASE 3: Exchange Protocol & Registry

**Standard ID**: WIA-AI-008-P3
**Version**: 1.0.0
**Status**: Active
**Last Updated**: 2025-01-15

## 弘益人間 - Benefit All Humanity

---

## 1. Overview

Phase 3 defines protocols for model distribution, registry services, authentication, and version management. This phase enables secure, scalable model sharing across organizations and teams.

## 2. Model Registry Architecture

### 2.1 Registry Components

A compliant model registry MUST provide:

1. **Storage Backend**: Persistent storage for model artifacts
2. **Metadata Service**: Database for model metadata and versions
3. **API Server**: REST/gRPC API for registry operations
4. **Authentication Service**: User and service authentication
5. **Authorization Service**: Role-based access control
6. **Search Service**: Model discovery and filtering

### 2.2 Registry API Specification

#### 2.2.1 REST API Endpoints

```yaml
# OpenAPI 3.0 Specification

openapi: 3.0.0
info:
  title: WIA-AI-008 Model Registry API
  version: 1.0.0
  description: Standard API for AI model registries

servers:
  - url: https://registry.example.com/api/v1

paths:
  /models:
    get:
      summary: List all models
      parameters:
        - name: task_type
          in: query
          schema:
            type: string
        - name: framework
          in: query
          schema:
            type: string
        - name: limit
          in: query
          schema:
            type: integer
            default: 50
        - name: offset
          in: query
          schema:
            type: integer
            default: 0
      responses:
        '200':
          description: List of models
          content:
            application/json:
              schema:
                type: object
                properties:
                  models:
                    type: array
                    items:
                      $ref: '#/components/schemas/ModelSummary'
                  total:
                    type: integer
                  limit:
                    type: integer
                  offset:
                    type: integer

    post:
      summary: Register new model
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ModelRegistration'
      responses:
        '201':
          description: Model registered successfully
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Model'
        '400':
          description: Invalid request
        '401':
          description: Unauthorized

  /models/{model_name}:
    get:
      summary: Get model details
      parameters:
        - name: model_name
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Model details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Model'
        '404':
          description: Model not found

  /models/{model_name}/versions:
    get:
      summary: List model versions
      parameters:
        - name: model_name
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: List of versions
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/ModelVersion'

    post:
      summary: Create new version
      parameters:
        - name: model_name
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              type: object
              properties:
                version:
                  type: string
                model_file:
                  type: string
                  format: binary
                metadata:
                  type: object
      responses:
        '201':
          description: Version created
        '400':
          description: Invalid version

  /models/{model_name}/versions/{version}:
    get:
      summary: Get specific version
      parameters:
        - name: model_name
          in: path
          required: true
          schema:
            type: string
        - name: version
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Version details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ModelVersion'

  /models/{model_name}/versions/{version}/download:
    get:
      summary: Download model
      parameters:
        - name: model_name
          in: path
          required: true
          schema:
            type: string
        - name: version
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Model file
          content:
            application/octet-stream:
              schema:
                type: string
                format: binary
        '404':
          description: Version not found

components:
  schemas:
    ModelSummary:
      type: object
      properties:
        name:
          type: string
        latest_version:
          type: string
        task_type:
          type: string
        framework:
          type: string
        created_at:
          type: string
          format: date-time
        updated_at:
          type: string
          format: date-time

    Model:
      type: object
      properties:
        name:
          type: string
        description:
          type: string
        owner:
          type: string
        license:
          type: string
        versions:
          type: array
          items:
            $ref: '#/components/schemas/ModelVersion'

    ModelVersion:
      type: object
      properties:
        version:
          type: string
        created_at:
          type: string
          format: date-time
        size_bytes:
          type: integer
        checksum:
          type: string
        metadata:
          type: object
        stage:
          type: string
          enum: [development, staging, production, archived]
```

## 3. Authentication & Authorization

### 3.1 Authentication Methods

Registries MUST support at least one of:

1. **API Keys**: Simple key-based authentication
2. **OAuth 2.0**: Industry-standard authorization
3. **JWT Tokens**: Stateless token-based auth
4. **mTLS**: Mutual TLS for service-to-service

#### 3.1.1 API Key Authentication

```python
import requests

# Register with API key
headers = {
    'Authorization': 'Bearer YOUR_API_KEY',
    'Content-Type': 'application/json'
}

response = requests.post(
    'https://registry.example.com/api/v1/models',
    headers=headers,
    json=model_metadata
)
```

### 3.2 Role-Based Access Control (RBAC)

#### 3.2.1 Standard Roles

| Role | Permissions |
|------|-------------|
| `viewer` | Read access to public and authorized models |
| `contributor` | Read + Write (register models, create versions) |
| `maintainer` | Read + Write + Manage (promote versions, set tags) |
| `admin` | Full access (delete models, manage users, permissions) |

#### 3.2.2 Permission Matrix

| Action | Viewer | Contributor | Maintainer | Admin |
|--------|--------|-------------|------------|-------|
| List models | ✓ | ✓ | ✓ | ✓ |
| Download model | ✓ | ✓ | ✓ | ✓ |
| Register model | ✗ | ✓ | ✓ | ✓ |
| Create version | ✗ | ✓ | ✓ | ✓ |
| Promote to staging | ✗ | ✗ | ✓ | ✓ |
| Promote to production | ✗ | ✗ | ✓ | ✓ |
| Delete model | ✗ | ✗ | ✗ | ✓ |
| Manage permissions | ✗ | ✗ | ✗ | ✓ |

## 4. Model Lifecycle Management

### 4.1 Stage Definitions

Models progress through defined stages:

1. **Development**: Active development, frequent changes
2. **Staging**: Testing and validation
3. **Production**: Deployed and serving traffic
4. **Archived**: Deprecated, retained for history

### 4.2 Stage Transitions

```python
# Python SDK example
from wia_registry import RegistryClient

client = RegistryClient('https://registry.example.com', api_key='...')

# Transition to staging
client.transition_model_version_stage(
    name='sentiment-classifier',
    version='1.2.0',
    stage='staging'
)

# Promote to production (with approval)
client.transition_model_version_stage(
    name='sentiment-classifier',
    version='1.2.0',
    stage='production',
    archive_existing_production=True
)
```

## 5. Distribution Protocols

### 5.1 Supported Protocols

Registries SHOULD support multiple distribution methods:

| Protocol | Use Case | Speed |
|----------|----------|-------|
| HTTPS | Standard downloads | Medium |
| S3 | Cloud storage integration | Fast |
| GCS | Google Cloud integration | Fast |
| Azure Blob | Azure integration | Fast |
| Git LFS | Version control integration | Medium |
| BitTorrent | Peer-to-peer distribution | Fast (distributed) |

### 5.2 Download URLs

```bash
# Direct HTTPS download
curl -H "Authorization: Bearer $API_KEY" \
  https://registry.example.com/api/v1/models/resnet50/versions/1.0.0/download \
  -o resnet50-1.0.0.onnx

# S3 pre-signed URL
# Registry returns temporary S3 URL
curl https://s3.amazonaws.com/models/resnet50-1.0.0.onnx?...

# Git LFS
git lfs pull
```

## 6. Model Discovery

### 6.1 Search API

```http
GET /api/v1/models/search?q=sentiment+analysis&task_type=text-classification&framework=pytorch&limit=10

Response:
{
  "results": [
    {
      "name": "bert-sentiment-analysis",
      "version": "2.1.0",
      "task_type": "text-classification",
      "framework": "pytorch",
      "accuracy": 0.934,
      "downloads": 15234,
      "stars": 427
    }
  ],
  "total": 1,
  "query": "sentiment analysis"
}
```

### 6.2 Tags and Filters

Models SHOULD support tagging:

```json
{
  "tags": [
    "nlp",
    "sentiment-analysis",
    "production-ready",
    "quantized",
    "bert-base"
  ]
}
```

## 7. Versioning Protocol

### 7.1 Version Aliasing

Registries MUST support version aliases:

- `latest`: Most recent version
- `production`: Current production version
- `staging`: Current staging version
- Custom tags: `v1`, `stable`, etc.

```python
# Download by version
client.download('sentiment-classifier', version='1.2.0')

# Download by alias
client.download('sentiment-classifier', version='latest')
client.download('sentiment-classifier', version='production')
```

## 8. Model Signing and Verification

### 8.1 Cryptographic Signatures

Models SHOULD be signed for integrity:

```python
import hashlib
import hmac

def sign_model(model_path: str, secret_key: str) -> str:
    """Generate HMAC-SHA256 signature"""
    with open(model_path, 'rb') as f:
        data = f.read()

    signature = hmac.new(
        secret_key.encode(),
        data,
        hashlib.sha256
    ).hexdigest()

    return signature

def verify_model(model_path: str, signature: str, secret_key: str) -> bool:
    """Verify model signature"""
    expected = sign_model(model_path, secret_key)
    return hmac.compare_digest(signature, expected)
```

### 8.2 Checksums

All model downloads MUST include SHA256 checksums:

```json
{
  "version": "1.0.0",
  "download_url": "https://...",
  "checksum": {
    "algorithm": "SHA256",
    "value": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
  }
}
```

## 9. Compliance

Phase 3 compliant registries MUST:
- Implement standard REST API
- Support semantic versioning
- Provide authentication
- Enable RBAC
- Include model signatures/checksums
- Support stage transitions
- Provide search functionality

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)
Licensed under Apache 2.0
