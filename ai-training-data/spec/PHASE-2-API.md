# WIA-AI-007 PHASE 2: API & SDK Specification

**Version:** 1.0.0
**Status:** Stable
**Philosophy:** 弘益人間 (Benefit All Humanity)

## Overview

Phase 2 defines the API and SDK for dataset operations including upload, download, versioning, transformation, and quality assessment. The API is RESTful, language-agnostic, and designed for seamless integration with ML pipelines.

## API Endpoints

### Dataset Management

#### Create Dataset
```http
POST /api/v1/datasets
Content-Type: application/json

{
  "name": "medical-images-v1",
  "type": "image",
  "format": "hdf5",
  "description": "Medical imaging dataset",
  "philosophy": "弘益人間"
}

Response: 201 Created
{
  "dataset_id": "ds-abc123",
  "version": "1.0.0",
  "created_at": "2025-01-15T10:30:00Z"
}
```

#### Get Dataset Metadata
```http
GET /api/v1/datasets/{dataset_id}

Response: 200 OK
{
  "wia-ai-007": "1.0",
  "dataset": { ... },
  "schema": { ... },
  "provenance": { ... },
  "quality": { ... }
}
```

#### Update Dataset
```http
PATCH /api/v1/datasets/{dataset_id}
Content-Type: application/json

{
  "description": "Updated description",
  "version": "1.1.0"
}

Response: 200 OK
```

#### Delete Dataset
```http
DELETE /api/v1/datasets/{dataset_id}

Response: 204 No Content
```

### Data Operations

#### Upload Data
```http
POST /api/v1/datasets/{dataset_id}/upload
Content-Type: multipart/form-data

Response: 202 Accepted
{
  "upload_id": "upl-xyz789",
  "status": "processing"
}
```

#### Download Data
```http
GET /api/v1/datasets/{dataset_id}/download?version=1.0.0&format=parquet

Response: 200 OK
Content-Type: application/octet-stream
```

### Versioning

#### List Versions
```http
GET /api/v1/datasets/{dataset_id}/versions

Response: 200 OK
{
  "versions": [
    {"version": "1.0.0", "created": "2025-01-15T10:30:00Z"},
    {"version": "1.1.0", "created": "2025-02-01T14:20:00Z"}
  ]
}
```

#### Create Version
```http
POST /api/v1/datasets/{dataset_id}/versions
{
  "version": "1.2.0",
  "changes": ["Added 5000 new samples"],
  "breaking_changes": false
}

Response: 201 Created
```

### Quality Assessment

#### Run Quality Checks
```http
POST /api/v1/datasets/{dataset_id}/quality/check
{
  "checks": ["completeness", "consistency", "validity"],
  "thresholds": {
    "completeness": 0.95,
    "consistency": 0.98
  }
}

Response: 200 OK
{
  "quality_score": 0.96,
  "checks": {
    "completeness": {"score": 0.985, "passed": true},
    "consistency": {"score": 0.992, "passed": true}
  }
}
```

### Transformation

#### Apply Transformation
```http
POST /api/v1/datasets/{dataset_id}/transform
{
  "operations": [
    {"type": "normalization", "method": "z-score"},
    {"type": "augmentation", "factor": 2}
  ],
  "output_version": "2.0.0"
}

Response: 202 Accepted
{
  "job_id": "job-123",
  "status": "queued"
}
```

## TypeScript SDK

### Installation
```bash
npm install @wia/ai-007
```

### Basic Usage

```typescript
import { WIA_AI_007, Dataset } from '@wia/ai-007';

// Initialize client
const client = new WIA_AI_007({
  apiKey: process.env.WIA_API_KEY,
  endpoint: 'https://api.wia.org/v1'
});

// Create dataset
const dataset = await client.datasets.create({
  name: 'medical-images-v1',
  type: 'image',
  format: 'hdf5',
  description: 'Medical imaging dataset for diagnosis',
  philosophy: '弘益人間'
});

// Upload data
await dataset.upload({
  files: ['./data/*.png'],
  labels: './labels.json',
  metadata: './metadata.json'
});

// Run quality checks
const quality = await dataset.checkQuality({
  checks: ['completeness', 'consistency', 'validity'],
  thresholds: {
    completeness: 0.95,
    consistency: 0.98
  }
});

console.log(`Quality score: ${quality.overallScore}`);

// Create new version
const newVersion = await dataset.createVersion({
  version: '1.1.0',
  changes: ['Added 1000 new samples'],
  breakingChanges: false
});

// Download dataset
await dataset.download({
  version: '1.1.0',
  format: 'parquet',
  outputPath: './downloads'
});
```

### Advanced Features

```typescript
// Versioning and branching
const branch = await dataset.createBranch('experimental');
await branch.applyTransformations([
  { type: 'augmentation', factor: 3 },
  { type: 'normalization' }
]);

// Quality monitoring
dataset.on('quality:below-threshold', (event) => {
  console.warn(`Quality dropped: ${event.metric} = ${event.value}`);
});

// Bias detection
const biasReport = await dataset.detectBias({
  protectedAttributes: ['gender', 'race', 'age'],
  fairnessMetrics: ['demographic_parity', 'equal_opportunity']
});

// Privacy preservation
const anonymized = await dataset.anonymize({
  method: 'k-anonymity',
  k: 5,
  quasiIdentifiers: ['age', 'zip_code', 'gender']
});

// Export for ML frameworks
await dataset.export({
  framework: 'pytorch',
  format: 'hdf5',
  splits: { train: 0.8, val: 0.1, test: 0.1 }
});
```

## Python SDK

```python
from wia_ai_007 import WIA_AI_007, Dataset

# Initialize client
client = WIA_AI_007(api_key=os.getenv('WIA_API_KEY'))

# Create dataset
dataset = client.datasets.create(
    name='medical-images-v1',
    type='image',
    format='hdf5',
    description='Medical imaging dataset',
    philosophy='弘益人間'
)

# Upload data
dataset.upload(
    files='./data/*.png',
    labels='./labels.json',
    metadata='./metadata.json'
)

# Quality checks
quality = dataset.check_quality(
    checks=['completeness', 'consistency'],
    thresholds={'completeness': 0.95}
)

print(f"Quality score: {quality.overall_score}")

# Version management
new_version = dataset.create_version(
    version='1.1.0',
    changes=['Added 1000 new samples']
)

# Download
dataset.download(
    version='1.1.0',
    format='parquet',
    output_path='./downloads'
)
```

## Authentication

### API Key Authentication
```http
GET /api/v1/datasets
Authorization: Bearer {api_key}
```

### OAuth 2.0
```http
POST /oauth/token
{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret"
}
```

## Rate Limiting

- **Standard tier**: 1000 requests/hour
- **Premium tier**: 10000 requests/hour
- **Enterprise**: Unlimited

Rate limit headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1642345678
```

## Error Handling

### Standard Error Response
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid dataset format",
    "details": {
      "field": "format",
      "expected": ["json", "parquet", "hdf5"],
      "received": "invalid_format"
    }
  }
}
```

### HTTP Status Codes
- 200: Success
- 201: Created
- 202: Accepted (async operation)
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 422: Validation Error
- 429: Rate Limit Exceeded
- 500: Internal Server Error

## Webhooks

### Register Webhook
```http
POST /api/v1/webhooks
{
  "url": "https://your-app.com/webhook",
  "events": ["dataset.created", "quality.below_threshold"],
  "secret": "webhook_secret"
}
```

### Webhook Event
```json
{
  "event": "quality.below_threshold",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "dataset_id": "ds-abc123",
    "metric": "completeness",
    "value": 0.92,
    "threshold": 0.95
  }
}
```

## Best Practices

1. **Use versioning**: Always specify dataset versions
2. **Implement retries**: Handle transient failures with exponential backoff
3. **Validate input**: Validate data before upload
4. **Monitor quality**: Set up webhooks for quality alerts
5. **Secure API keys**: Never commit API keys to version control
6. **Handle rate limits**: Implement rate limiting in your client
7. **Use batch operations**: Upload/download in batches for efficiency

---

**弘益人間** · Benefit All Humanity
© 2025 SmileStory Inc. / WIA
WIA-AI-007 AI Training Data Standard v1.0
