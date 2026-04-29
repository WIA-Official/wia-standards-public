# WIA-MICROPLASTIC_DETECTION
## PHASE 2: API Interface Specification v1.0

**Status**: FULL Implementation
**Philosophy**: 弘益人間 (Benefit All Humanity)
**Last Updated**: 2026-01-12

---

## 1. Overview

This specification defines the RESTful API interfaces for microplastic detection systems, including sample analysis endpoints, spectroscopy data services, particle identification, and environmental monitoring integration.

### 1.1 API Design Principles

- **RESTful Architecture**: Standard HTTP methods and status codes
- **Authentication**: OAuth 2.0 and API key support
- **Versioning**: URL-based versioning (/v1/, /v2/)
- **Rate Limiting**: Fair usage policies
- **Real-time Updates**: WebSocket support for live monitoring
- **Batch Processing**: Efficient bulk operations

### 1.2 Base URL

```
Production:  https://api.wia.org/microplastic-detection/v1
Staging:     https://staging-api.wia.org/microplastic-detection/v1
Development: https://dev-api.wia.org/microplastic-detection/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

```http
GET /samples
Authorization: Bearer YOUR_API_KEY
```

### 2.2 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "samples:read samples:write analysis:execute"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "samples:read samples:write analysis:execute"
}
```

---

## 3. Sample Management APIs

### 3.1 Create Sample

```http
POST /samples
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "sampleName": "Santa Monica Bay - Station A1",
  "collectionId": "SMB-2026-Q1",
  "collectedAt": "2026-01-12T10:30:00Z",
  "collectedBy": "researcher-001",
  "collectionMethod": "MANTA_TRAWL",
  "location": {
    "latitude": 34.0522,
    "longitude": -118.2437,
    "siteName": "Santa Monica Bay",
    "waterbody": "Pacific Ocean"
  },
  "environmentType": "MARINE_SURFACE",
  "sampleVolume": 1000,
  "temperature": 18.5,
  "pH": 8.1,
  "salinity": 33.5
}
```

**Response (201 Created):**
```json
{
  "sampleId": "sample-2026-001",
  "sampleName": "Santa Monica Bay - Station A1",
  "status": "PENDING_ANALYSIS",
  "createdAt": "2026-01-12T15:45:00Z",
  "qrCode": "https://api.wia.org/samples/sample-2026-001/qr",
  "links": {
    "self": "/samples/sample-2026-001",
    "analyze": "/samples/sample-2026-001/analyze",
    "results": "/samples/sample-2026-001/results"
  }
}
```

### 3.2 Get Sample

```http
GET /samples/{sampleId}
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "sampleId": "sample-2026-001",
  "sampleName": "Santa Monica Bay - Station A1",
  "collectionId": "SMB-2026-Q1",
  "collectedAt": "2026-01-12T10:30:00Z",
  "location": {
    "latitude": 34.0522,
    "longitude": -118.2437,
    "siteName": "Santa Monica Bay"
  },
  "environmentType": "MARINE_SURFACE",
  "status": "ANALYZED",
  "totalParticleCount": 247,
  "particleConcentration": 0.247,
  "concentrationUnit": "particles/L",
  "dominantPolymer": "PE",
  "analyzedAt": "2026-01-12T16:30:00Z"
}
```

### 3.3 List Samples

```http
GET /samples?limit=50&offset=0&status=ANALYZED&collectionId=SMB-2026-Q1
Authorization: Bearer YOUR_API_KEY
```

**Query Parameters:**
- `limit` (integer): Number of results (default: 50, max: 100)
- `offset` (integer): Pagination offset
- `status` (string): Filter by status
- `collectionId` (string): Filter by collection
- `environmentType` (string): Filter by environment
- `startDate` (ISO8601): Collected after date
- `endDate` (ISO8601): Collected before date

**Response (200 OK):**
```json
{
  "samples": [...],
  "total": 127,
  "limit": 50,
  "offset": 0,
  "links": {
    "next": "/samples?limit=50&offset=50",
    "prev": null
  }
}
```

### 3.4 Update Sample

```http
PATCH /samples/{sampleId}
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "processingDate": "2026-01-12T14:00:00Z",
  "processingMethod": "DENSITY_SEPARATION",
  "filtrationSize": 20,
  "qcStatus": "PASSED"
}
```

### 3.5 Delete Sample

```http
DELETE /samples/{sampleId}
Authorization: Bearer YOUR_API_KEY
```

**Response (204 No Content)**

---

## 4. Analysis APIs

### 4.1 Submit Analysis Job

```http
POST /samples/{sampleId}/analyze
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "analysisMethod": "AUTOMATED_IMAGING",
  "techniques": ["RAMAN", "FTIR"],
  "minParticleSize": 10,
  "maxParticleSize": 5000,
  "minPolymerConfidence": 0.80,
  "priority": "NORMAL",
  "notifications": {
    "email": "researcher@example.com",
    "webhook": "https://your-server.com/webhook/analysis-complete"
  }
}
```

**Response (202 Accepted):**
```json
{
  "jobId": "job-2026-abc123",
  "sampleId": "sample-2026-001",
  "status": "QUEUED",
  "estimatedDuration": 3600,
  "queuePosition": 3,
  "createdAt": "2026-01-12T15:45:00Z",
  "links": {
    "status": "/jobs/job-2026-abc123",
    "cancel": "/jobs/job-2026-abc123/cancel"
  }
}
```

### 4.2 Get Analysis Job Status

```http
GET /jobs/{jobId}
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "jobId": "job-2026-abc123",
  "status": "IN_PROGRESS",
  "progress": 65,
  "startedAt": "2026-01-12T15:50:00Z",
  "currentStage": "POLYMER_IDENTIFICATION",
  "particlesAnalyzed": 161,
  "totalParticles": 247,
  "estimatedCompletion": "2026-01-12T16:30:00Z"
}
```

**Job Status Values:**
- `QUEUED`: Job is in queue
- `IN_PROGRESS`: Analysis running
- `COMPLETED`: Analysis finished successfully
- `FAILED`: Analysis failed
- `CANCELLED`: Job cancelled by user

### 4.3 Get Analysis Results

```http
GET /samples/{sampleId}/results
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "resultId": "result-2026-xyz789",
  "sampleId": "sample-2026-001",
  "analyzedAt": "2026-01-12T16:30:00Z",
  "analyzedBy": "instrument-raman-001",
  "totalParticles": 247,
  "concentration": {
    "particlesPerUnit": 0.247,
    "unit": "particles/L",
    "confidenceInterval": [0.221, 0.273],
    "confidenceLevel": 0.95
  },
  "sizeDistribution": {
    "meanSize": 152.3,
    "medianSize": 98.5,
    "sizeRange": [12.5, 987.3]
  },
  "polymerDistribution": {
    "polymers": [
      {"polymerType": "PE", "count": 98, "percentage": 39.7},
      {"polymerType": "PP", "count": 72, "percentage": 29.1},
      {"polymerType": "PS", "count": 45, "percentage": 18.2},
      {"polymerType": "PET", "count": 32, "percentage": 13.0}
    ]
  },
  "shapeDistribution": {
    "shapes": [
      {"shapeType": "FRAGMENT", "count": 142, "percentage": 57.5},
      {"shapeType": "FIBER", "count": 78, "percentage": 31.6},
      {"shapeType": "FILM", "count": 27, "percentage": 10.9}
    ]
  },
  "links": {
    "particles": "/samples/sample-2026-001/particles",
    "download": "/samples/sample-2026-001/results/download",
    "report": "/samples/sample-2026-001/report"
  }
}
```

---

## 5. Particle APIs

### 5.1 Get Particles

```http
GET /samples/{sampleId}/particles?limit=100&polymerType=PE&minSize=50
Authorization: Bearer YOUR_API_KEY
```

**Query Parameters:**
- `limit` (integer): Number of results
- `offset` (integer): Pagination offset
- `polymerType` (string): Filter by polymer
- `shapeType` (string): Filter by shape
- `minSize` (number): Minimum size in μm
- `maxSize` (number): Maximum size in μm
- `verified` (boolean): Only verified particles

**Response (200 OK):**
```json
{
  "particles": [
    {
      "particleId": "particle-001",
      "sampleId": "sample-2026-001",
      "size": {
        "length": 125.5,
        "width": 87.2,
        "equivalentSphericalDiameter": 98.3,
        "sizeClass": "LARGE_MICROPLASTIC"
      },
      "shape": {
        "primaryShape": "FRAGMENT",
        "aspectRatio": 1.44,
        "circularity": 0.72
      },
      "polymerType": "PE",
      "polymerConfidence": 0.92,
      "color": {
        "primaryColor": "BLUE",
        "transparency": "OPAQUE"
      },
      "detectionMethod": "RAMAN_SPECTROSCOPY",
      "verified": true,
      "imageUrl": "/particles/particle-001/image"
    }
  ],
  "total": 98,
  "limit": 100,
  "offset": 0
}
```

### 5.2 Get Single Particle

```http
GET /particles/{particleId}
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "particleId": "particle-001",
  "sampleId": "sample-2026-001",
  "size": {...},
  "shape": {...},
  "polymerType": "PE",
  "polymerConfidence": 0.92,
  "spectra": {
    "raman": "/particles/particle-001/spectra/raman",
    "ftir": "/particles/particle-001/spectra/ftir"
  },
  "images": {
    "optical": "/particles/particle-001/images/optical",
    "fluorescence": "/particles/particle-001/images/fluorescence"
  },
  "detectedAt": "2026-01-12T16:15:23Z",
  "verified": true
}
```

### 5.3 Update Particle

```http
PATCH /particles/{particleId}
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "verified": true,
  "verifiedBy": "analyst-smith",
  "verificationNotes": "Confirmed PE fragment with strong Raman signal",
  "polymerType": "PE",
  "polymerConfidence": 0.95
}
```

---

## 6. Spectroscopy APIs

### 6.1 Get Raman Spectrum

```http
GET /particles/{particleId}/spectra/raman
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "spectrumId": "raman-001",
  "particleId": "particle-001",
  "wavenumbers": [100, 101, 102, ...],
  "intensities": [1250, 1340, 1420, ...],
  "laserWavelength": 532,
  "laserPower": 5.0,
  "exposureTime": 10,
  "accumulations": 3,
  "peakPositions": [1128, 1295, 1440, 2850, 2883, 2930],
  "matchedPolymer": "PE",
  "matchConfidence": 0.92,
  "signalToNoiseRatio": 45.3,
  "spectrumQuality": "EXCELLENT"
}
```

### 6.2 Upload Spectrum

```http
POST /particles/{particleId}/spectra/raman
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "wavenumbers": [100, 101, ...],
  "intensities": [1250, 1340, ...],
  "laserWavelength": 532,
  "laserPower": 5.0,
  "exposureTime": 10
}
```

### 6.3 Identify Polymer from Spectrum

```http
POST /spectra/identify
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "spectrumType": "RAMAN",
  "wavenumbers": [100, 101, ...],
  "intensities": [1250, 1340, ...],
  "spectralLibrary": "WILEY_KNOWITALL"
}
```

**Response (200 OK):**
```json
{
  "matches": [
    {
      "polymerType": "PE",
      "confidence": 0.92,
      "hitQuality": 0.95,
      "libraryEntry": "Polyethylene (HDPE)"
    },
    {
      "polymerType": "PP",
      "confidence": 0.15,
      "hitQuality": 0.45,
      "libraryEntry": "Polypropylene (isotactic)"
    }
  ],
  "bestMatch": {
    "polymerType": "PE",
    "confidence": 0.92
  }
}
```

---

## 7. Image Analysis APIs

### 7.1 Upload Image for Analysis

```http
POST /images/analyze
Content-Type: multipart/form-data
Authorization: Bearer YOUR_API_KEY
```

**Form Data:**
- `image`: Image file (JPEG, PNG, TIFF)
- `sampleId`: Sample identifier
- `imageType`: optical, fluorescence, sem
- `magnification`: Magnification level
- `pixelSize`: Micrometers per pixel

**Response (202 Accepted):**
```json
{
  "jobId": "img-job-001",
  "status": "PROCESSING",
  "estimatedDuration": 120
}
```

### 7.2 Get Image Analysis Results

```http
GET /images/analyze/{jobId}/results
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "jobId": "img-job-001",
  "status": "COMPLETED",
  "particlesDetected": 247,
  "particles": [
    {
      "boundingBox": {"x": 120, "y": 340, "width": 85, "height": 62},
      "size": 98.3,
      "shape": "FRAGMENT",
      "confidence": 0.88
    }
  ],
  "annotatedImageUrl": "/images/img-job-001/annotated"
}
```

---

## 8. Environmental Monitoring APIs

### 8.1 Submit Sensor Reading

```http
POST /sensors/{sensorId}/readings
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "timestamp": "2026-01-12T10:30:00Z",
  "location": {
    "latitude": 34.0522,
    "longitude": -118.2437
  },
  "particleCount": 12,
  "sizeDistribution": {
    "nano": 3,
    "micro": 9
  },
  "waterQuality": {
    "temperature": 18.5,
    "pH": 8.1,
    "turbidity": 2.3
  }
}
```

### 8.2 Get Sensor Data

```http
GET /sensors/{sensorId}/readings?startDate=2026-01-01&endDate=2026-01-31
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
```json
{
  "sensorId": "sensor-smb-001",
  "sensorName": "Santa Monica Bay - Buoy A",
  "readings": [
    {
      "timestamp": "2026-01-12T10:30:00Z",
      "particleCount": 12,
      "temperature": 18.5
    }
  ],
  "summary": {
    "totalReadings": 720,
    "averageParticleCount": 15.3,
    "maxParticleCount": 47,
    "trendAnalysis": "INCREASING"
  }
}
```

---

## 9. Reporting APIs

### 9.1 Generate Report

```http
POST /reports/generate
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "reportType": "SAMPLE_ANALYSIS",
  "sampleIds": ["sample-2026-001", "sample-2026-002"],
  "format": "PDF",
  "includeCharts": true,
  "includeRawData": false,
  "language": "en"
}
```

**Response (202 Accepted):**
```json
{
  "reportId": "report-2026-001",
  "status": "GENERATING",
  "estimatedDuration": 30
}
```

### 9.2 Download Report

```http
GET /reports/{reportId}/download
Authorization: Bearer YOUR_API_KEY
```

**Response (200 OK):**
- Content-Type: application/pdf
- Binary PDF data

---

## 10. Batch Operations

### 10.1 Batch Sample Creation

```http
POST /samples/batch
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "samples": [
    {...},
    {...}
  ]
}
```

### 10.2 Batch Analysis

```http
POST /analyze/batch
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

---

## 11. WebSocket APIs

### 11.1 Real-time Analysis Updates

```javascript
const ws = new WebSocket('wss://api.wia.org/microplastic-detection/v1/ws');

ws.on('open', () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    channel: 'analysis-updates',
    jobId: 'job-2026-abc123',
    token: 'YOUR_API_KEY'
  }));
});

ws.on('message', (data) => {
  const update = JSON.parse(data);
  console.log(`Progress: ${update.progress}%`);
});
```

---

## 12. Error Handling

### 12.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_SAMPLE_ID",
    "message": "Sample not found",
    "details": "Sample 'sample-999' does not exist",
    "timestamp": "2026-01-12T15:45:00Z",
    "requestId": "req-abc123"
  }
}
```

### 12.2 HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `202 Accepted`: Job queued
- `204 No Content`: Success, no response body
- `400 Bad Request`: Invalid input
- `401 Unauthorized`: Missing/invalid auth
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Maintenance

---

## 13. Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1642001234
```

**Limits:**
- Free tier: 100 requests/hour
- Standard tier: 1,000 requests/hour
- Professional tier: 10,000 requests/hour
- Enterprise: Custom limits

---

## 14. Webhooks

### 14.1 Register Webhook

```http
POST /webhooks
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY
```

**Request Body:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": ["analysis.completed", "sample.created"],
  "secret": "your_webhook_secret"
}
```

### 14.2 Webhook Payload

```json
{
  "event": "analysis.completed",
  "timestamp": "2026-01-12T16:30:00Z",
  "data": {
    "jobId": "job-2026-abc123",
    "sampleId": "sample-2026-001",
    "status": "COMPLETED"
  }
}
```

---

**Document Version**: 1.0
**Last Updated**: 2026-01-12
**Status**: FULL Implementation
**Philosophy**: 弘益人間 - Clean oceans for all humanity

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
