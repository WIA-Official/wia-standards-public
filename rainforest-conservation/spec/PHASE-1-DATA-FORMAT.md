# WIA Rainforest Conservation Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Rainforest Conservation Data Format Standard defines a unified format for monitoring rainforest ecosystems, tracking deforestation, preserving biodiversity, and protecting indigenous rights, enabling global coordination of forest conservation efforts.

**Core Objectives**:
- Enable precise monitoring of forest coverage and canopy health
- Track biodiversity and species preservation
- Integrate satellite and ground sensor data
- Support indigenous community rights and traditional knowledge
- Facilitate carbon credit and REDD+ programs

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Forest Monitoring | Tree coverage, canopy density, deforestation tracking |
| Biodiversity | Species counts, endemic species, threatened species |
| Carbon Storage | CO₂ sequestration, carbon credits, climate impact |
| Indigenous Rights | Traditional territories, community management, benefit-sharing |
| Satellite Integration | Remote sensing, NDVI analysis, fire detection |

### 1.3 Out of Scope

- Timber trade management (covered by WIA-FORESTRY)
- Urban forestry (covered by WIA-URBAN-FOREST)
- Agricultural land use (covered by WIA-AGRICULTURE)

---

## Terminology

| Term | Definition |
|------|------------|
| **Rainforest** | Dense forest area with high rainfall (>2000mm/year) |
| **Canopy Cover** | Percentage of ground covered by tree crowns |
| **Deforestation** | Permanent removal of forest for non-forest use |
| **Biodiversity Index** | Metric measuring species diversity (0-100) |
| **Endemic Species** | Species found only in specific geographic location |
| **REDD+** | Reducing Emissions from Deforestation and Degradation |
| **NDVI** | Normalized Difference Vegetation Index |
| **Indigenous Territory** | Land traditionally owned/managed by indigenous peoples |

---

## Base Structure

### 3.1 Core Data Model

```json
{
  "id": "string",
  "version": "string",
  "timestamp": "ISO8601",
  "forestArea": {
    "coverage": "number",
    "canopyCover": "number",
    "location": "GeoJSON",
    "forestType": "enum"
  },
  "biodiversity": {
    "speciesCount": "number",
    "endemicSpecies": "number",
    "threatenedSpecies": "number",
    "biodiversityIndex": "number"
  },
  "deforestation": {
    "rate": "number",
    "causes": ["string"],
    "alertLevel": "enum"
  },
  "carbonStorage": {
    "totalCarbon": "number",
    "sequestrationRate": "number",
    "carbonCredits": "number"
  },
  "indigenousRights": {
    "territoryName": "string",
    "community": "string",
    "managementAgreement": "boolean"
  },
  "metadata": {
    "source": "string",
    "accuracy": "number",
    "verifiedBy": "string"
  }
}
```

---

## Data Schema

### 4.1 Forest Area Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string | Yes | Unique identifier (FOREST-YYYY-XXXXX) |
| `coverage` | number | Yes | Forest area in hectares |
| `canopyCover` | number | Yes | Canopy coverage percentage (0-100) |
| `location` | GeoJSON | Yes | Geographic coordinates |
| `forestType` | enum | Yes | tropical, amazon, congo, southeast, atlantic |
| `elevation` | number | No | Average elevation in meters |
| `rainfall` | number | No | Annual rainfall in mm |

### 4.2 Biodiversity Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `speciesCount` | number | Yes | Total number of identified species |
| `endemicSpecies` | number | No | Number of endemic species |
| `threatenedSpecies` | number | No | Number of IUCN Red List species |
| `biodiversityIndex` | number | Yes | Calculated index (0-100) |
| `keystone` | array | No | Keystone species list |
| `invasive` | array | No | Invasive species detected |

### 4.3 Deforestation Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `rate` | number | Yes | Hectares lost per year |
| `causes` | array | No | logging, agriculture, mining, fire, etc. |
| `alertLevel` | enum | Yes | normal, warning, critical |
| `trend` | number | No | Year-over-year change percentage |
| `illegalActivity` | boolean | No | Illegal deforestation detected |

### 4.4 Carbon Storage Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `totalCarbon` | number | Yes | Total carbon stored in tons CO₂ |
| `sequestrationRate` | number | No | Annual CO₂ capture rate |
| `carbonCredits` | number | No | Verified carbon credits issued |
| `biomass` | number | No | Above-ground biomass in tons |
| `soilCarbon` | number | No | Soil organic carbon in tons |

### 4.5 Indigenous Rights Object

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `territoryName` | string | No | Name of indigenous territory |
| `community` | string | No | Indigenous community name |
| `managementAgreement` | boolean | No | Community-led management in place |
| `landRights` | enum | No | recognized, pending, disputed |
| `traditionalKnowledge` | boolean | No | Traditional practices integrated |

---

## Field Specifications

### 5.1 ID Format

**Pattern**: `FOREST-{YEAR}-{REGION}-{SEQUENCE}`

Examples:
- `FOREST-2025-AMZ-001` (Amazon)
- `FOREST-2025-CGO-042` (Congo Basin)
- `FOREST-2025-SEA-123` (Southeast Asia)

### 5.2 GeoJSON Location

```json
{
  "type": "Polygon",
  "coordinates": [
    [
      [-60.123, -3.456],
      [-60.456, -3.456],
      [-60.456, -3.789],
      [-60.123, -3.789],
      [-60.123, -3.456]
    ]
  ]
}
```

### 5.3 Timestamp Format

ISO 8601: `2025-01-15T14:30:00Z`

---

## Data Types

### 6.1 Enumerations

**Forest Type**:
- `tropical` - General tropical rainforest
- `amazon` - Amazon Basin
- `congo` - Congo Basin
- `southeast` - Southeast Asian rainforest
- `atlantic` - Atlantic Forest

**Alert Level**:
- `normal` - Deforestation < 100 ha/year
- `warning` - Deforestation 100-500 ha/year
- `critical` - Deforestation > 500 ha/year

**Land Rights**:
- `recognized` - Legally recognized indigenous territory
- `pending` - Recognition in process
- `disputed` - Land rights under dispute

---

## Validation Rules

### 7.1 Required Fields

✅ Must have: `id`, `coverage`, `canopyCover`, `location`, `forestType`, `biodiversityIndex`, `deforestationRate`, `alertLevel`

### 7.2 Value Ranges

| Field | Min | Max | Unit |
|-------|-----|-----|------|
| canopyCover | 0 | 100 | % |
| biodiversityIndex | 0 | 100 | score |
| coverage | 1 | 10000000 | hectares |
| deforestationRate | 0 | - | ha/year |

### 7.3 Data Integrity

- `speciesCount` ≥ `endemicSpecies` + `threatenedSpecies`
- `totalCarbon` must be positive if forest exists
- `timestamp` cannot be in the future
- `location` must be valid GeoJSON

---

## Examples

### 8.1 Complete Forest Record

```json
{
  "id": "FOREST-2025-AMZ-001",
  "version": "1.0.0",
  "timestamp": "2025-01-15T12:00:00Z",
  "forestArea": {
    "coverage": 15000,
    "canopyCover": 85,
    "location": {
      "type": "Polygon",
      "coordinates": [[[-60.1, -3.4], [-60.5, -3.4], [-60.5, -3.8], [-60.1, -3.8], [-60.1, -3.4]]]
    },
    "forestType": "amazon",
    "elevation": 120,
    "rainfall": 2800
  },
  "biodiversity": {
    "speciesCount": 523,
    "endemicSpecies": 68,
    "threatenedSpecies": 12,
    "biodiversityIndex": 88.5,
    "keystone": ["jaguar", "harpy eagle", "brazil nut tree"]
  },
  "deforestation": {
    "rate": 120,
    "causes": ["logging", "agriculture"],
    "alertLevel": "warning",
    "trend": -15,
    "illegalActivity": true
  },
  "carbonStorage": {
    "totalCarbon": 450000,
    "sequestrationRate": 3200,
    "carbonCredits": 15000,
    "biomass": 380000,
    "soilCarbon": 70000
  },
  "indigenousRights": {
    "territoryName": "Yanomami Territory",
    "community": "Yanomami People",
    "managementAgreement": true,
    "landRights": "recognized",
    "traditionalKnowledge": true
  },
  "metadata": {
    "source": "satellite-sentinel-2",
    "accuracy": 0.95,
    "verifiedBy": "WIA-Forest-Monitor-System"
  }
}
```

### 8.2 Minimal Record

```json
{
  "id": "FOREST-2025-CGO-042",
  "version": "1.0.0",
  "timestamp": "2025-01-15T12:00:00Z",
  "forestArea": {
    "coverage": 5000,
    "canopyCover": 78,
    "location": {
      "type": "Point",
      "coordinates": [15.123, -2.456]
    },
    "forestType": "congo"
  },
  "biodiversity": {
    "speciesCount": 280,
    "biodiversityIndex": 72.3
  },
  "deforestation": {
    "rate": 45,
    "alertLevel": "normal"
  },
  "carbonStorage": {
    "totalCarbon": 180000
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

**© 2025 WIA (World Certification Industry Association)**
**License**: MIT
**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity
