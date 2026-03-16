# WIA Precision Agriculture Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - Agriculture)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Field Data Schema](#field-data-schema)
5. [Zone Management](#zone-management)
6. [GPS Coordinate System](#gps-coordinate-system)
7. [Soil Data Format](#soil-data-format)
8. [Crop Information](#crop-information)
9. [Variable Rate Technology (VRT)](#variable-rate-technology)
10. [Yield Mapping](#yield-mapping)
11. [Validation Rules](#validation-rules)
12. [Examples](#examples)
13. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Precision Agriculture Data Format Standard defines a unified framework for digital farming data, enabling variable rate technology, GPS-guided operations, zone management, and data-driven decision making across all agricultural systems.

**Core Objectives**:
- Standardize field zone data with GPS coordinates
- Enable variable rate application (VRT) for fertilizers, seeds, pesticides
- Support GPS-guided farming operations with cm-level accuracy
- Facilitate zone-based management and prescription mapping
- Enable yield mapping and performance analysis
- Support ISOBUS and farm management system integration
- Promote sustainable and efficient farming practices

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Field Identity | Unique identification, boundaries, GPS coordinates |
| Zone Management | Field subdivision, zone classification, management zones |
| Soil Data | Soil type, moisture, pH, organic matter, nutrient levels |
| Crop Information | Crop type, variety, planting date, growth stage |
| VRT Prescriptions | Variable rate maps for fertilizer, seed, pesticide |
| GPS Guidance | Auto-steer paths, AB lines, headland management |
| Yield Mapping | Harvest data, yield variability, quality metrics |
| Weather Integration | Precipitation, temperature, growing degree days |

### 1.3 Design Principles

1. **Precision**: Support cm-level GPS accuracy for field operations
2. **Variability**: Capture spatial variability within fields
3. **Interoperability**: Compatible with ISOBUS, ADAPT, AgGateway standards
4. **Scalability**: Support farms from 1 hectare to 10,000+ hectares
5. **Real-time**: Enable in-field decision making during operations
6. **Sustainability**: Track inputs, reduce waste, optimize resource use

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Precision Agriculture** | Farm management strategy using data to optimize productivity |
| **Variable Rate Technology (VRT)** | Applying inputs at variable rates based on field zones |
| **Management Zone** | Sub-field area with similar characteristics |
| **GPS Guidance** | Auto-steering system using GPS/GNSS signals |
| **ISOBUS** | ISO 11783 standard for agricultural equipment communication |
| **NDVI** | Normalized Difference Vegetation Index (crop health) |
| **Prescription Map** | Spatial map defining input rates per zone |
| **Yield Monitor** | System measuring crop yield during harvest |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `field_id` | Unique field identifier | `"FLD-2025-001"` |
| `zone_id` | Management zone identifier | `"Z1"`, `"Z2"` |
| `gps_coordinate` | WGS84 coordinates | `{"lat": 37.5665, "lng": 126.9780}` |
| `soil_type` | Soil classification | `"LOAM"`, `"CLAY"` |
| `crop_code` | Crop type identifier | `"CORN"`, `"WHEAT"` |
| `application_rate` | VRT rate (kg/ha or seeds/ha) | `150.5` |
| `ndvi_value` | Vegetation index (0-1) | `0.75` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Root Object

```json
{
  "standard": "WIA-AGRI-002",
  "version": "1.0.0",
  "field": { /* Field object */ },
  "zones": [ /* Zone array */ ],
  "prescriptions": [ /* VRT prescription array */ ],
  "operations": [ /* Field operation array */ ],
  "metadata": { /* Metadata object */ }
}
```

### 3.2 Field Object

```json
{
  "field_id": "FLD-2025-001",
  "field_name": "North Field",
  "farm_id": "FARM-001",
  "farm_name": "Green Valley Farms",
  "total_area_ha": 50.5,
  "boundary": {
    "type": "Polygon",
    "coordinates": [ /* GeoJSON polygon */ ]
  },
  "center_point": {
    "lat": 37.566535,
    "lng": 126.977969
  },
  "elevation_m": 125.5,
  "slope_percent": 2.3,
  "created_at": "2025-01-15T08:00:00Z",
  "updated_at": "2025-01-15T08:00:00Z"
}
```

**Field Requirements**:
- `field_id` - REQUIRED - Unique identifier
- `field_name` - REQUIRED - Human-readable name
- `total_area_ha` - REQUIRED - Area in hectares
- `boundary` - REQUIRED - GeoJSON polygon (WGS84)
- `center_point` - REQUIRED - Field center GPS coordinate
- `elevation_m` - OPTIONAL - Average elevation in meters
- `slope_percent` - OPTIONAL - Average slope percentage

---

## Field Data Schema

### 4.1 Complete Field Data

```json
{
  "field_id": "FLD-2025-001",
  "field_name": "North Field",
  "farm_id": "FARM-001",
  "farm_name": "Green Valley Farms",
  "total_area_ha": 50.5,
  "boundary": {
    "type": "Polygon",
    "coordinates": [[
      [126.977, 37.566],
      [126.980, 37.566],
      [126.980, 37.568],
      [126.977, 37.568],
      [126.977, 37.566]
    ]]
  },
  "center_point": { "lat": 37.567, "lng": 126.9785 },
  "elevation_m": 125.5,
  "slope_percent": 2.3,
  "soil_type_primary": "LOAM",
  "drainage_class": "WELL_DRAINED",
  "irrigation_available": true,
  "previous_crop": "SOYBEAN",
  "current_crop": {
    "crop_type": "CORN",
    "variety": "Pioneer P1197",
    "planting_date": "2025-04-15",
    "expected_harvest": "2025-10-20"
  },
  "created_at": "2025-01-15T08:00:00Z",
  "updated_at": "2025-01-15T08:00:00Z"
}
```

---

## Zone Management

### 5.1 Management Zone Structure

```json
{
  "zone_id": "Z1",
  "field_id": "FLD-2025-001",
  "zone_name": "High Productivity Zone",
  "area_ha": 12.5,
  "boundary": {
    "type": "Polygon",
    "coordinates": [ /* GeoJSON polygon */ ]
  },
  "center_point": { "lat": 37.5668, "lng": 126.9782 },
  "classification": "HIGH_YIELD",
  "soil_characteristics": {
    "soil_type": "LOAM",
    "organic_matter_percent": 3.2,
    "ph": 6.5,
    "cec_meq_100g": 18.5,
    "moisture_percent": 22.0
  },
  "yield_history": {
    "2024": 10.5,
    "2023": 9.8,
    "2022": 10.2,
    "avg_tons_ha": 10.17
  },
  "ndvi_index": 0.75,
  "elevation_m": 127.0,
  "slope_percent": 1.5
}
```

### 5.2 Zone Classification

| Classification | Description | Typical Yield |
|----------------|-------------|---------------|
| `HIGH_YIELD` | Most productive areas | >110% of field average |
| `MEDIUM_YIELD` | Average productivity | 90-110% of field average |
| `LOW_YIELD` | Below average areas | <90% of field average |
| `VARIABLE` | High variability within zone | Mixed performance |
| `PROBLEM_AREA` | Persistent issues | Consistent underperformance |

---

## GPS Coordinate System

### 6.1 Coordinate Standard

**All GPS coordinates MUST use WGS84 (EPSG:4326)**

```json
{
  "lat": 37.566535,
  "lng": 126.977969,
  "altitude_m": 125.5,
  "accuracy_cm": 2.0,
  "fix_type": "RTK_FIXED",
  "satellites": 24,
  "hdop": 0.8,
  "timestamp": "2025-01-15T08:30:45Z"
}
```

### 6.2 GPS Accuracy Levels

| Accuracy | Type | Use Case |
|----------|------|----------|
| 2-5 cm | RTK (Real-Time Kinematic) | Precision operations, auto-steer |
| 10-30 cm | DGPS (Differential GPS) | General field operations |
| 1-3 m | Standard GPS | Field mapping, basic navigation |
| 5-10 m | Uncorrected GPS | Not recommended for precision ag |

### 6.3 AB Line (Guidance Line)

```json
{
  "ab_line_id": "AB-001",
  "field_id": "FLD-2025-001",
  "point_a": { "lat": 37.566, "lng": 126.977 },
  "point_b": { "lat": 37.568, "lng": 126.980 },
  "heading_degrees": 45.5,
  "swath_width_m": 12.0,
  "offset_m": 0.0,
  "created_at": "2025-04-15T07:00:00Z"
}
```

---

## Soil Data Format

### 7.1 Soil Sample

```json
{
  "sample_id": "SOIL-2025-001",
  "field_id": "FLD-2025-001",
  "zone_id": "Z1",
  "gps_location": { "lat": 37.5668, "lng": 126.9782 },
  "sample_date": "2025-03-01",
  "depth_cm": 30,
  "soil_type": "LOAM",
  "ph": 6.5,
  "organic_matter_percent": 3.2,
  "cec_meq_100g": 18.5,
  "nutrients": {
    "nitrogen_ppm": 25.5,
    "phosphorus_ppm": 42.0,
    "potassium_ppm": 185.0,
    "sulfur_ppm": 12.5,
    "calcium_ppm": 1850.0,
    "magnesium_ppm": 280.0
  },
  "moisture_percent": 22.0,
  "texture": {
    "sand_percent": 40,
    "silt_percent": 40,
    "clay_percent": 20
  },
  "lab_id": "LAB-2025-XYZ",
  "lab_name": "AgriTest Labs"
}
```

### 7.2 Soil Types

| Code | Description | Characteristics |
|------|-------------|-----------------|
| `SAND` | Sandy soil | Fast drainage, low nutrients |
| `LOAM` | Loamy soil | Ideal balance, best for crops |
| `CLAY` | Clay soil | Slow drainage, high nutrients |
| `SILT` | Silty soil | Medium drainage, fertile |
| `PEAT` | Organic peat | High organic matter |

---

## Crop Information

### 8.1 Crop Data

```json
{
  "crop_id": "CROP-2025-001",
  "field_id": "FLD-2025-001",
  "crop_type": "CORN",
  "variety": "Pioneer P1197",
  "planting_date": "2025-04-15",
  "expected_harvest": "2025-10-20",
  "growth_stage": "V6",
  "plant_population_per_ha": 75000,
  "row_spacing_cm": 76,
  "planting_depth_cm": 5,
  "seed_rate_kg_ha": 22.5,
  "target_yield_tons_ha": 10.5,
  "current_health": {
    "ndvi": 0.75,
    "status": "HEALTHY",
    "diseases": [],
    "pests": []
  }
}
```

### 8.2 Common Crop Types

| Code | Crop Name | Typical Yield (tons/ha) |
|------|-----------|------------------------|
| `CORN` | Maize/Corn | 8-12 |
| `WHEAT` | Wheat | 5-8 |
| `SOYBEAN` | Soybeans | 3-4 |
| `RICE` | Rice (paddy) | 6-9 |
| `COTTON` | Cotton | 1.5-2.5 (fiber) |
| `BARLEY` | Barley | 4-6 |

---

## Variable Rate Technology

### 9.1 VRT Prescription Map

```json
{
  "prescription_id": "VRT-2025-001",
  "field_id": "FLD-2025-001",
  "application_type": "FERTILIZER",
  "product_name": "Urea 46-0-0",
  "created_date": "2025-04-10",
  "application_date": "2025-04-15",
  "base_rate_kg_ha": 150.0,
  "zones": [
    {
      "zone_id": "Z1",
      "rate_kg_ha": 180.0,
      "adjustment_percent": 20.0,
      "rationale": "High yield potential, increased N requirement"
    },
    {
      "zone_id": "Z2",
      "rate_kg_ha": 150.0,
      "adjustment_percent": 0.0,
      "rationale": "Medium productivity, base rate applied"
    },
    {
      "zone_id": "Z3",
      "rate_kg_ha": 120.0,
      "adjustment_percent": -20.0,
      "rationale": "Lower yield zone, reduced N to prevent lodging"
    }
  ],
  "total_product_kg": 7750.0,
  "savings_vs_uniform": {
    "product_saved_kg": 450.0,
    "cost_saved": 225.0,
    "currency": "USD"
  }
}
```

### 9.2 Application Types

| Type | Product Examples | Unit |
|------|------------------|------|
| `FERTILIZER` | Urea, DAP, MAP | kg/ha |
| `SEED` | Corn, Soybean, Wheat | seeds/ha or kg/ha |
| `PESTICIDE` | Herbicides, Insecticides | L/ha or kg/ha |
| `LIME` | Agricultural lime | tons/ha |
| `MANURE` | Organic fertilizers | tons/ha |

---

## Yield Mapping

### 10.1 Yield Data Point

```json
{
  "yield_point_id": "YP-2025-001",
  "field_id": "FLD-2025-001",
  "harvest_date": "2025-10-22",
  "gps_location": { "lat": 37.5668, "lng": 126.9782 },
  "yield_kg_ha": 10500,
  "moisture_percent": 15.5,
  "dry_yield_kg_ha": 8870,
  "timestamp": "2025-10-22T14:23:45Z",
  "combine_speed_km_h": 6.5,
  "header_width_m": 7.6,
  "grain_quality": {
    "test_weight_kg_hl": 75.5,
    "protein_percent": 8.5,
    "oil_percent": 3.8
  }
}
```

### 10.2 Harvest Summary

```json
{
  "harvest_id": "HARVEST-2025-001",
  "field_id": "FLD-2025-001",
  "crop_type": "CORN",
  "harvest_date": "2025-10-22",
  "total_area_ha": 50.5,
  "total_yield_kg": 530250,
  "avg_yield_kg_ha": 10500,
  "moisture_percent": 15.5,
  "yield_variability": {
    "min_kg_ha": 8200,
    "max_kg_ha": 12400,
    "std_deviation": 850,
    "coefficient_variation": 8.1
  },
  "zones": [
    { "zone_id": "Z1", "avg_yield_kg_ha": 11800 },
    { "zone_id": "Z2", "avg_yield_kg_ha": 10500 },
    { "zone_id": "Z3", "avg_yield_kg_ha": 9200 }
  ]
}
```

---

## Validation Rules

### 11.1 Field Validation

| Rule | Description | Error Code |
|------|-------------|------------|
| Area Range | 0.1 ha ≤ area ≤ 100,000 ha | `E-FIELD-001` |
| Boundary Valid | GeoJSON polygon must be valid | `E-FIELD-002` |
| GPS Range | -90 ≤ lat ≤ 90, -180 ≤ lng ≤ 180 | `E-FIELD-003` |

### 11.2 Zone Validation

| Rule | Description | Error Code |
|------|-------------|------------|
| Zone Coverage | Sum of zone areas ≈ field area (±2%) | `E-ZONE-001` |
| Zone Overlap | Zones must not overlap | `E-ZONE-002` |
| NDVI Range | 0 ≤ NDVI ≤ 1 | `E-ZONE-003` |

### 11.3 VRT Validation

| Rule | Description | Error Code |
|------|-------------|------------|
| Rate Range | 0 ≤ rate ≤ 1000 kg/ha | `E-VRT-001` |
| Product Valid | Product name must be specified | `E-VRT-002` |
| Zone Coverage | All zones must have rates | `E-VRT-003` |

---

## Examples

### 12.1 Complete Field Example

```json
{
  "standard": "WIA-AGRI-002",
  "version": "1.0.0",
  "field": {
    "field_id": "FLD-2025-001",
    "field_name": "North Field",
    "farm_id": "FARM-001",
    "farm_name": "Green Valley Farms",
    "total_area_ha": 50.5,
    "boundary": {
      "type": "Polygon",
      "coordinates": [[
        [126.977, 37.566], [126.980, 37.566],
        [126.980, 37.568], [126.977, 37.568],
        [126.977, 37.566]
      ]]
    },
    "center_point": { "lat": 37.567, "lng": 126.9785 }
  },
  "zones": [
    {
      "zone_id": "Z1",
      "area_ha": 15.5,
      "classification": "HIGH_YIELD",
      "ndvi_index": 0.78,
      "soil_type": "LOAM"
    },
    {
      "zone_id": "Z2",
      "area_ha": 20.0,
      "classification": "MEDIUM_YIELD",
      "ndvi_index": 0.65,
      "soil_type": "CLAY_LOAM"
    },
    {
      "zone_id": "Z3",
      "area_ha": 15.0,
      "classification": "LOW_YIELD",
      "ndvi_index": 0.52,
      "soil_type": "SANDY_LOAM"
    }
  ],
  "prescriptions": [
    {
      "prescription_id": "VRT-2025-001",
      "application_type": "FERTILIZER",
      "product_name": "Urea 46-0-0",
      "zones": [
        { "zone_id": "Z1", "rate_kg_ha": 180.0 },
        { "zone_id": "Z2", "rate_kg_ha": 150.0 },
        { "zone_id": "Z3", "rate_kg_ha": 120.0 }
      ]
    }
  ],
  "metadata": {
    "created_at": "2025-01-15T08:00:00Z",
    "updated_at": "2025-01-15T08:00:00Z",
    "created_by": "agronomist@greenfarms.com"
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

**© 2025 WIA Standards - MIT License**
**弘益人間 (Hongik Ingan) - Benefit All Humanity**
