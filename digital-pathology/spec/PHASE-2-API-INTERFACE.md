# WIA-MED-008 Phase 2: API Interface Specification

## Version 1.0.0

**Status:** ✅ Complete  
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the standard API interfaces for accessing and manipulating digital pathology images in the WIA-MED-008 ecosystem.

---

## 2. Core APIs

### 2.1 Image Access API

#### 2.1.1 Slide Information

```http
GET /api/v1/slides/{slide_id}

Response:
{
  "slide_id": "S25-00123-A1",
  "accession_number": "S25-00123",
  "dimensions": {
    "width": 100000,
    "height": 100000,
    "levels": 6
  },
  "properties": {
    "magnification": 40,
    "mpp": 0.25,
    "scanner": "Aperio GT450"
  }
}
```

#### 2.1.2 Tile Access (IIIF Compatible)

```http
GET /api/v1/slides/{slide_id}/{level}/{x},{y}/{width},{height}

Parameters:
  - level: Pyramid level (0 = highest resolution)
  - x, y: Top-left corner coordinates
  - width, height: Region size

Response: Image tile (JPEG/PNG)
```

### 2.2 Metadata API

```http
# Get metadata
GET /api/v1/slides/{slide_id}/metadata

# Update metadata (authorized only)
PUT /api/v1/slides/{slide_id}/metadata
Body: { metadata JSON }
```

### 2.3 Annotation API

```http
# List annotations
GET /api/v1/slides/{slide_id}/annotations

# Create annotation
POST /api/v1/slides/{slide_id}/annotations
Body: { GeoJSON annotation }

# Update annotation
PUT /api/v1/slides/{slide_id}/annotations/{annotation_id}

# Delete annotation
DELETE /api/v1/slides/{slide_id}/annotations/{annotation_id}
```

---

## 3. Authentication & Authorization

### 3.1 OAuth 2.0

```http
# Get access token
POST /oauth/token
Body: {
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_secret"
}

Response: {
  "access_token": "eyJ...",
  "token_type": "Bearer",
  "expires_in": 3600
}

# Use token
GET /api/v1/slides/{slide_id}
Authorization: Bearer eyJ...
```

### 3.2 Role-Based Access Control

| Role | Permissions |
|------|-------------|
| Pathologist | Read, Annotate, Diagnose |
| Technologist | Read, Upload, QC |
| Researcher | Read (anonymized only) |
| AI System | Read, Write AI Results |

---

## 4. SDK Examples

### 4.1 Python SDK

```python
from wia_pathology import WIAClient

# Initialize client
client = WIAClient(
    base_url='https://api.hospital.com',
    api_key='your_api_key'
)

# Get slide
slide = client.get_slide('S25-00123-A1')

# Read region
region = slide.read_region(
    location=(10000, 15000),
    level=0,
    size=(512, 512)
)

# Add annotation
annotation = {
    "type": "Feature",
    "geometry": {
        "type": "Polygon",
        "coordinates": [[[x1, y1], [x2, y2], ...]]
    },
    "properties": {
        "classification": "tumor",
        "confidence": 0.95
    }
}
slide.add_annotation(annotation)
```

### 4.2 JavaScript SDK

```javascript
import { WIAClient } from '@wia/pathology-sdk';

const client = new WIAClient({
  baseURL: 'https://api.hospital.com',
  apiKey: 'your_api_key'
});

// Get slide info
const slide = await client.getSlide('S25-00123-A1');

// Get tile URL
const tileUrl = client.getTileURL(
  'S25-00123-A1',
  level=0,
  x=10000,
  y=15000,
  width=256,
  height=256
);
```

---

## 5. Performance Requirements

| Metric | Requirement |
|--------|-------------|
| Tile Response Time | < 200ms (p95) |
| Metadata Response | < 50ms |
| Throughput | > 1000 tiles/sec |
| Availability | > 99.9% |

---

**© 2025 WIA**  
**License:** MIT License
