# Chapter 4: Data Format Specification

## Learning Objectives

After completing this chapter, you will be able to:

1. Implement WIA JSON schemas for observations and measurements
2. Apply validation rules to ensure data quality
3. Use controlled vocabularies for categorical data
4. Design databases compatible with WIA format
5. Convert existing data to WIA format

---

## 4.1 Base Schema Structure

### 4.1.1 Common Fields

All WIA ecosystem monitoring records share a common base schema:

```json
{
  "wia_version": "1.0",
  "schema_type": "string (required)",
  "record_id": "UUID or unique identifier (required)",
  "timestamp": "ISO 8601 datetime (required)",

  "location": {
    "latitude": "number (required)",
    "longitude": "number (required)",
    "elevation": "number (optional)",
    "datum": "string (default: WGS84)",
    "precision": "number in meters (optional)",
    "location_name": "string (optional)"
  },

  "observer": {
    "id": "string (required)",
    "name": "string (optional)",
    "organization": "string (optional)",
    "email": "string (optional)"
  },

  "quality": {
    "validation_status": "enum (required)",
    "quality_flags": "array of strings (optional)",
    "confidence_level": "number 0-1 (optional)"
  }
}
```

### 4.1.2 Required vs. Optional Fields

**Required Fields** (must be present and non-null):
- `wia_version`: Standard version ("1.0")
- `schema_type`: Type of record
- `record_id`: Globally unique identifier
- `timestamp`: ISO 8601 datetime with timezone
- `location.latitude`: -90 to 90 decimal degrees
- `location.longitude`: -180 to 180 decimal degrees
- `observer.id`: Unique observer/sensor identifier
- `quality.validation_status`: One of controlled vocabulary values

**Optional Fields** (may be omitted or null):
- `location.elevation`: Meters above sea level
- `location.datum`: Defaults to WGS84
- `location.precision`: Location accuracy in meters
- `observer.name`, `observer.organization`, `observer.email`
- `quality.confidence_level`: 0.0 to 1.0

### 4.1.3 Validation Status Values

```typescript
enum ValidationStatus {
  unvalidated = "Newly submitted, not yet reviewed",
  in_review = "Under expert review",
  validated = "Passed automated checks and basic review",
  expert_verified = "Confirmed by taxonomic expert or calibrated sensor",
  questionable = "Flags raised, use with caution",
  invalid = "Failed validation, exclude from analysis"
}
```

---

## 4.2 Species Observation Schema

### 4.2.1 Complete Schema

```json
{
  "wia_version": "1.0",
  "schema_type": "species-observation",
  "record_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-06-15T14:30:00-07:00",

  "location": {
    "latitude": 47.6062,
    "longitude": -122.3321,
    "elevation": 15.5,
    "datum": "WGS84",
    "precision": 10,
    "location_name": "Discovery Park, Seattle, WA"
  },

  "observer": {
    "id": "orcid:0000-0002-1825-0097",
    "name": "Jane Smith",
    "organization": "Seattle Audubon Society",
    "email": "jane.smith@seattleaudubon.org"
  },

  "taxon": {
    "scientific_name": "Haliaeetus leucocephalus",
    "common_name": "Bald Eagle",
    "kingdom": "Animalia",
    "phylum": "Chordata",
    "class": "Aves",
    "order": "Accipitriformes",
    "family": "Accipitridae",
    "genus": "Haliaeetus",
    "species": "leucocephalus",
    "subspecies": null,
    "taxon_authority": "Catalogue of Life 2025",
    "taxon_id": "COL:7MDXZ"
  },

  "detection_method": "visual_survey",
  "occurrence_status": "present",
  "abundance": 2,
  "life_stage": "adult",
  "sex": "unknown",
  "behavior": "Perched in snag, vocalizing",
  "reproductive_condition": null,
  "habitat_type": "ENVO:00000447",

  "associated_taxa": [
    {
      "scientific_name": "Corvus corax",
      "relationship": "Mobbing eagle"
    }
  ],

  "environmental_conditions": {
    "temperature_celsius": 18.5,
    "weather": "Partly cloudy",
    "wind_speed_ms": 3.2
  },

  "quality": {
    "validation_status": "expert_verified",
    "quality_flags": [],
    "confidence_level": 1.0
  }
}
```

### 4.2.2 Detection Methods

Controlled vocabulary for `detection_method`:

| Value | Description | Typical Use |
|-------|-------------|-------------|
| `visual_survey` | Direct observation by observer | Birds, mammals, herps, plants |
| `camera_trap` | Motion-activated camera | Terrestrial mammals, ground birds |
| `acoustic_monitoring` | Audio recording and analysis | Birds, bats, amphibians, insects |
| `edna` | Environmental DNA sampling | Aquatic organisms, rare species |
| `telemetry` | Radio/GPS tracking device | Movement ecology, home ranges |
| `mark_recapture` | Capture-mark-recapture study | Population estimates |
| `remote_sensing` | Satellite/aerial imagery analysis | Large animals, landscape scale |
| `specimen` | Physical specimen collection | Taxonomic verification |

### 4.2.3 Occurrence Status

```typescript
enum OccurrenceStatus {
  present = "Organism detected at location",
  absent = "Target survey conducted, organism not found",
  trace = "Evidence of organism (tracks, scat, burrow) but no direct observation"
}
```

**Important:** `absent` is only valid when a targeted survey was conducted. Opportunistic observations should only record `present` or `trace`.

### 4.2.4 Life Stage Values

```typescript
enum LifeStage {
  // Animals
  egg = "Egg stage",
  larva = "Larval stage (insects, amphibians)",
  juvenile = "Juvenile/immature",
  adult = "Adult/mature",

  // Plants
  seed = "Seed",
  seedling = "Seedling (< 1 year)",
  sapling = "Sapling (young tree)",
  mature = "Mature plant",

  // General
  unknown = "Life stage could not be determined"
}
```

---

## 4.3 Environmental Sensor Data Schema

### 4.3.1 Sensor Time Series Schema

```json
{
  "wia_version": "1.0",
  "schema_type": "sensor-timeseries",
  "record_id": "650e8400-e29b-41d4-a716-446655440111",
  "timestamp": "2025-06-15T00:00:00Z",

  "sensor_id": "WEATHER-STATION-042",

  "sensor_metadata": {
    "sensor_type": "Temperature",
    "manufacturer": "Campbell Scientific",
    "model": "CS107",
    "serial_number": "SN12345678",
    "measurement_parameter": "Air temperature",
    "measurement_unit": "celsius",
    "precision": 0.1,
    "accuracy": 0.2,
    "detection_limit": null,
    "calibration_date": "2025-01-15",
    "calibration_certificate": "https://example.org/certs/CAL-2025-042.pdf"
  },

  "deployment": {
    "deployment_date": "2025-01-20T10:00:00Z",
    "location": {
      "latitude": 47.6815,
      "longitude": -121.7453,
      "elevation": 850,
      "datum": "WGS84"
    },
    "height_above_ground": 2.0,
    "depth_below_surface": null,
    "environment": "Forest canopy gap"
  },

  "data": {
    "start_time": "2025-06-15T00:00:00Z",
    "end_time": "2025-06-15T23:59:00Z",
    "interval_seconds": 900,
    "readings": [
      {
        "timestamp": "2025-06-15T00:00:00Z",
        "value": 12.3,
        "qc_flag": "good",
        "qc_notes": null
      },
      {
        "timestamp": "2025-06-15T00:15:00Z",
        "value": 12.1,
        "qc_flag": "good",
        "qc_notes": null
      },
      {
        "timestamp": "2025-06-15T00:30:00Z",
        "value": 11.9,
        "qc_flag": "good",
        "qc_notes": null
      }
      // ... 96 readings per day at 15-minute intervals
    ]
  },

  "observer": {
    "id": "org:neon",
    "name": "National Ecological Observatory Network",
    "organization": "Battelle"
  },

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 0.98
  }
}
```

### 4.3.2 QC Flag Values

```typescript
enum QCFlag {
  good = "Passes all QC checks, suitable for all analyses",
  questionable = "Marginally acceptable, use with caution",
  bad = "Fails QC checks, exclude from analysis",
  missing = "Expected reading but not collected (sensor offline, power failure)",
  estimated = "Value interpolated or modeled, not directly measured"
}
```

### 4.3.3 Measurement Units (QUDT)

All units should reference the QUDT ontology:

| Measurement | Unit | QUDT Code |
|-------------|------|-----------|
| Temperature | celsius | qudt:DEG_C |
| Temperature | fahrenheit | qudt:DEG_F |
| Pressure | millibar | qudt:MilliBAR |
| Wind speed | meters per second | qudt:M-PER-SEC |
| Precipitation | millimeter | qudt:MilliM |
| Humidity | percent | qudt:PERCENT |

---

## 4.4 Water Quality Schema

### 4.4.1 Complete Schema

```json
{
  "wia_version": "1.0",
  "schema_type": "water-quality-sample",
  "record_id": "750e8400-e29b-41d4-a716-446655440222",
  "timestamp": "2025-06-15T10:30:00-07:00",

  "location": {
    "latitude": 47.6201,
    "longitude": -122.3493,
    "elevation": 5.0,
    "datum": "WGS84",
    "precision": 5,
    "location_name": "Green River at Auburn"
  },

  "sample_id": "GR-2025-06-15-001",
  "waterbody_name": "Green River",
  "site_id": "USGS-12113000",
  "sampling_depth_meters": 0.5,
  "sampling_method": "grab_sample",

  "parameters": {
    "temperature_c": 16.5,
    "ph": 7.2,
    "dissolved_oxygen_mgl": 9.8,
    "turbidity_ntu": 12.3,
    "conductivity_uscm": 145,
    "total_nitrogen_mgl": 0.85,
    "total_phosphorus_mgl": 0.042,
    "chlorophyll_a_ugl": 3.2,
    "salinity_ppt": 0.1
  },

  "laboratory": {
    "lab_name": "King County Environmental Lab",
    "analysis_date": "2025-06-16",
    "methods": {
      "total_nitrogen": "EPA 351.2",
      "total_phosphorus": "EPA 365.1"
    },
    "detection_limits": {
      "total_nitrogen_mgl": 0.01,
      "total_phosphorus_mgl": 0.002
    }
  },

  "observer": {
    "id": "kcenvlab:tech042",
    "name": "Robert Johnson",
    "organization": "King County Environmental Laboratory"
  },

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 0.95
  }
}
```

### 4.4.2 Sampling Methods

```typescript
enum SamplingMethod {
  grab_sample = "Single discrete sample at point in time",
  composite = "Multiple samples combined (time or space composite)",
  in_situ = "Measured in place with sensor (not removed)",
  continuous = "Automated continuous monitoring system"
}
```

### 4.4.3 Standard Parameters

| Parameter | Unit | Typical Range | Method |
|-----------|------|---------------|--------|
| Temperature | °C | 0-40 | In situ thermometer/sensor |
| pH | - | 4-10 | Glass electrode |
| Dissolved Oxygen | mg/L | 0-20 | Optical or electrode sensor |
| Turbidity | NTU | 0-1000 | Nephelometer |
| Conductivity | μS/cm | 0-5000 | Conductivity probe |
| Total Nitrogen | mg/L | 0-10 | EPA 351.2 or equivalent |
| Total Phosphorus | mg/L | 0-1 | EPA 365.1 or equivalent |
| Chlorophyll-a | μg/L | 0-100 | Fluorometry or extraction |
| Salinity | ppt | 0-40 | Calculated from conductivity |

---

## 4.5 Air Quality Schema

### 4.5.1 Complete Schema

```json
{
  "wia_version": "1.0",
  "schema_type": "air-quality-measurement",
  "record_id": "850e8400-e29b-41d4-a716-446655440333",
  "timestamp": "2025-06-15T14:00:00-07:00",

  "location": {
    "latitude": 47.6062,
    "longitude": -122.3321,
    "elevation": 50,
    "datum": "WGS84",
    "precision": 10,
    "location_name": "Seattle - Duwamish Valley"
  },

  "station_id": "EPA-AQS-530330030",

  "parameters": {
    "pm25_ugm3": 8.5,
    "pm10_ugm3": 15.2,
    "o3_ppb": 42,
    "no2_ppb": 18,
    "so2_ppb": 2,
    "co_ppm": 0.4,
    "co2_ppm": 415,
    "temperature_c": 18.5,
    "relative_humidity_percent": 65,
    "pressure_mb": 1013.25,
    "wind_speed_ms": 3.2,
    "wind_direction_degrees": 225
  },

  "aqi_value": 35,
  "aqi_category": "good",

  "observer": {
    "id": "epa:pscaa",
    "name": "Puget Sound Clean Air Agency",
    "organization": "EPA AQS Network"
  },

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 0.98
  }
}
```

### 4.5.2 AQI Categories

```typescript
enum AQICategory {
  good = "0-50: Air quality satisfactory",
  moderate = "51-100: Acceptable for most, sensitive groups may be affected",
  unhealthy_sensitive = "101-150: Sensitive groups may experience health effects",
  unhealthy = "151-200: Everyone may experience health effects",
  very_unhealthy = "201-300: Health alert, everyone may experience serious effects",
  hazardous = "301+: Health warnings of emergency conditions"
}
```

### 4.5.3 Standard Pollutants

| Pollutant | Unit | Health Concern | EPA Standard |
|-----------|------|----------------|--------------|
| PM2.5 | μg/m³ | Respiratory, cardiovascular | 12 μg/m³ annual |
| PM10 | μg/m³ | Respiratory | 150 μg/m³ 24-hour |
| Ozone (O3) | ppb | Respiratory irritant | 70 ppb 8-hour |
| NO2 | ppb | Respiratory | 100 ppb 1-hour |
| SO2 | ppb | Respiratory | 75 ppb 1-hour |
| CO | ppm | Cardiovascular | 9 ppm 8-hour |

---

## 4.6 Metadata Standards

### 4.6.1 Dataset-Level Metadata

Every dataset (collection of observations) MUST include comprehensive metadata:

```json
{
  "title": "Green River Watershed Biodiversity Monitoring 2020-2025",
  "abstract": "Long-term monitoring of aquatic and terrestrial biodiversity in the Green River watershed, focusing on salmon recovery indicators and riparian habitat quality.",
  "keywords": ["biodiversity", "salmon", "riparian", "Green River", "monitoring"],

  "authors": [
    {
      "name": "Jane Smith",
      "orcid": "0000-0002-1825-0097",
      "affiliation": "University of Washington",
      "role": "Principal Investigator"
    },
    {
      "name": "Robert Johnson",
      "orcid": "0000-0003-9876-5432",
      "affiliation": "King County",
      "role": "Data Manager"
    }
  ],

  "contacts": [
    {
      "name": "Jane Smith",
      "email": "jsmith@uw.edu",
      "role": "Data custodian"
    }
  ],

  "funding_sources": [
    "EPA Puget Sound Partnership Grant #PS-2020-042",
    "Washington Department of Fish and Wildlife"
  ],

  "temporal_coverage": {
    "start_date": "2020-01-01",
    "end_date": "2025-12-31"
  },

  "geographic_coverage": {
    "bounding_box": [-122.5, 47.3, -121.8, 47.8],
    "description": "Green River watershed from headwaters in Cascade Range to mouth at Elliott Bay"
  },

  "taxonomic_coverage": [
    {
      "scientific_name": "Oncorhynchus",
      "common_name": "Pacific salmon",
      "rank": "genus"
    },
    {
      "scientific_name": "Aves",
      "common_name": "Birds",
      "rank": "class"
    }
  ],

  "methods": {
    "description": "Point count surveys (birds), electrofishing (fish), pitfall traps (invertebrates), water quality grab samples",
    "protocol_url": "https://example.org/protocols/green-river-2020.pdf",
    "sampling_design": "Stratified random, 20 sites revisited monthly"
  },

  "quality_assurance": {
    "description": "All species IDs verified by taxonomic experts. Water samples analyzed by certified lab. 10% field replicates.",
    "validation_procedures": "Automated range checks, expert review of flagged records"
  },

  "access": {
    "license": "CC BY 4.0",
    "restrictions": "Sensitive species locations (eagles, bats) obscured to 1km precision",
    "citation": "선행 연구. Green River Watershed Biodiversity Monitoring 2020-2025. doi:10.1234/example"
  },

  "related_resources": {
    "publications": [
      "https://doi.org/10.1234/paper1",
      "https://doi.org/10.1234/paper2"
    ],
    "related_datasets": [
      "https://doi.org/10.1234/related-dataset"
    ]
  }
}
```

---

## 4.7 Validation Rules

### 4.7.1 Automated Validation Checks

All WIA-compliant data MUST pass these validation checks:

```typescript
interface ValidationRules {
  schemaValidation: {
    check: "Data structure matches JSON schema";
    errorExample: "Field 'latitude' is required but missing";
  };

  dataTypes: {
    check: "Values match specified types";
    errorExample: "Field 'abundance' expects number, got string '5'";
  };

  rangeChecks: {
    latitude: "Must be between -90 and 90",
    longitude: "Must be between -180 and 180",
    temperature_c: "Must be between -100 and 100",
    ph: "Must be between 0 and 14",
    confidence_level: "Must be between 0 and 1"
  };

  temporalValidation: {
    futureDate: "Timestamp cannot be in the future",
    endAfterStart: "End time must be >= start time"
  };

  taxonomicValidation: {
    scientificName: "Must resolve to recognized taxon in authority",
    authority: "Must be one of: Catalogue of Life, GBIF Backbone, ITIS, WoRMS"
  };

  spatialValidation: {
    landOcean: "Latitude/longitude must be on land for terrestrial species",
    precision: "Precision must be > 0 if provided"
  };

  qualityFlags: {
    validationStatus: "Must be one of: unvalidated, in_review, validated, expert_verified, questionable, invalid",
    qcFlag: "Must be one of: good, questionable, bad, missing, estimated"
  };
}
```

### 4.7.2 Cross-Field Validation

```typescript
interface CrossFieldValidation {
  detectionMethodVsOccurrence: {
    rule: "If occurrence_status = 'absent', detection_method cannot be 'remote_sensing' or 'camera_trap'",
    reason: "These methods cannot confirm absence"
  };

  abundanceVsOccurrence: {
    rule: "If occurrence_status = 'absent', abundance must be 0 or null",
    reason: "Cannot have abundance for absent species"
  };

  elevationVsLocation: {
    rule: "Elevation should be consistent with latitude/longitude (DEM lookup)",
    warning: "Flag if elevation differs by >100m from expected"
  };

  temperatureVsLocation: {
    rule: "Temperature should be plausible for location and season",
    warning: "Flag if outside expected range for location/date"
  };
}
```

---

## 4.8 File Format Conventions

### 4.8.1 Single Record Files

**Format:**
- Encoding: UTF-8
- Format: JSON
- Extension: `.json`
- Compression: gzip optional (`.json.gz`)

**Example filename:** `observation-550e8400-e29b-41d4-a716-446655440000.json`

### 4.8.2 Multi-Record Files

**Newline-Delimited JSON (NDJSON):**
```
{"wia_version":"1.0","schema_type":"species-observation",...}
{"wia_version":"1.0","schema_type":"species-observation",...}
{"wia_version":"1.0","schema_type":"species-observation",...}
```

- One complete JSON object per line
- Extension: `.ndjson` or `.jsonl`
- Compression: gzip recommended for large files (`.ndjson.gz`)

**Advantages:**
- Streamable (process one line at a time)
- Appendable (add records without parsing entire file)
- Parallel processing (split file by lines)

### 4.8.3 Batch Submission

**JSON Array:**
```json
[
  {"wia_version":"1.0", "schema_type":"species-observation", ...},
  {"wia_version":"1.0", "schema_type":"species-observation", ...},
  {"wia_version":"1.0", "schema_type":"species-observation", ...}
]
```

- Maximum 10,000 records per file recommended
- Use NDJSON for larger datasets

---

## 4.9 Review Questions

### Question 1
Design a validation algorithm that detects whether a temperature reading of 35°C in a mountain stream is likely an error. What fields would you check?

### Question 2
A researcher has bird observations with common names only ("Robin", "Sparrow"). How should they convert to WIA format? What challenges might arise?

### Question 3
Explain the difference between `occurrence_status = "absent"` and simply not having an observation record. When should each be used?

### Question 4
A sensor produces 96 readings per day (15-minute intervals). Should these be stored as 96 separate WIA records or one record with 96 readings? Justify your answer.

### Question 5
Convert this legacy water quality record to WIA format:
```
Site: Green River
Date: 6/15/25 10:30 AM
Temp: 61.7 F
DO: 9.8
pH: 7.2
Notes: Slightly turbid
```

---

## 4.10 Key Takeaways

| Component | Key Points |
|-----------|------------|
| **Base Schema** | Common fields shared by all record types |
| **Species Observations** | Taxonomic info, detection method, abundance |
| **Sensor Data** | Metadata-rich time series with QC flags |
| **Water Quality** | Lab methods, detection limits, parameters |
| **Air Quality** | Pollutants, meteorology, AQI calculations |
| **Metadata** | Comprehensive dataset documentation |
| **Validation** | Automated checks, range validation, cross-field rules |

### Design Decisions
- **JSON**: Human-readable, widely supported
- **ISO 8601**: Unambiguous date/time with timezone
- **WGS84**: Standard datum for global interoperability
- **QUDT**: Standard units vocabulary
- **Controlled vocabularies**: Prevent misspellings, enable filtering

### Next Chapter Preview

Chapter 5 details Phase 2 (API Interface), showing how to programmatically submit observations, query data, and stream real-time sensor readings using REST, WebSocket, and MQTT protocols.

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
