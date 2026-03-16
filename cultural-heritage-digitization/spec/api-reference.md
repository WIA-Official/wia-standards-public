# WIA-EDU-023: API Reference
## Cultural Heritage Digitization Standard

**Version:** 1.0.0
**Last Updated:** 2025-01-26

---

## 1. Overview

This document specifies the RESTful API for cultural heritage digitization systems complying with WIA-EDU-023.

### Base URL
```
https://api.example.com/wia/v1/heritage
```

### Authentication
```http
Authorization: Bearer <access_token>
```

---

## 2. Core Resources

### 2.1 Artifacts

#### Get Artifact
```http
GET /artifacts/{id}
```

**Response:**
```json
{
  "id": "artifact-001",
  "wiaId": "WIA-EDU-023-000001",
  "title": "Tang Dynasty Vase",
  "culture": "Tang Dynasty China",
  "period": "618-907 CE",
  "materials": ["Glazed ceramic"],
  "dimensions": {"height": 30, "width": 20, "depth": 20, "unit": "cm"},
  "location": {"current": "Xi'an Museum", "origin": "Xi'an, Shaanxi"},
  "3dModels": {
    "glb": "https://cdn.example.com/models/artifact-001.glb",
    "usdz": "https://cdn.example.com/models/artifact-001.usdz"
  },
  "metadata": {
    "dublinCore": {...},
    "cidocCrm": {...},
    "technical": {...}
  }
}
```

#### List Artifacts
```http
GET /artifacts?culture=Tang&limit=20&offset=0
```

#### Create Artifact
```http
POST /artifacts
Content-Type: application/json

{
  "title": "Ancient Scroll",
  "culture": "Song Dynasty",
  ...
}
```

### 2.2 3D Models

#### Upload 3D Model
```http
POST /artifacts/{id}/models
Content-Type: multipart/form-data

file: artifact.glb
format: glTF-2.0
resolution: high
```

#### Get Model Metadata
```http
GET /artifacts/{id}/models/{modelId}
```

### 2.3 Virtual Tours

#### Create Tour
```http
POST /tours

{
  "name": "Ancient Egypt Gallery",
  "type": "vr",
  "artifacts": ["artifact-001", "artifact-002"],
  "scenes": [...]
}
```

---

## 3. Metadata Operations

### 3.1 Dublin Core

```http
GET /artifacts/{id}/metadata/dublincore
PUT /artifacts/{id}/metadata/dublincore
```

### 3.2 CIDOC-CRM

```http
GET /artifacts/{id}/metadata/cidoc
```

---

## 4. Search & Discovery

```http
GET /search?q=pottery&culture=Roman&period=100BCE-400CE
```

---

## 5. Analytics

```http
GET /artifacts/{id}/analytics
```

---

© 2025 WIA - MIT License
