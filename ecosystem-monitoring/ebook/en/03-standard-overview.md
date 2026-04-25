# Chapter 3: WIA Ecosystem Monitoring Standard Overview

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain the WIA 4-phase architecture and rationale
2. Understand how each phase addresses specific challenges
3. Identify the core components of the standard
4. Compare WIA approach with existing standards
5. Evaluate adoption pathways for different use cases

---

## 3.1 The WIA 4-Phase Architecture

### 3.1.1 Overview of Phases

The WIA Ecosystem Monitoring Standard follows a proven 4-phase architecture used across all WIA standards:

```
WIA 4-Phase Architecture:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  PHASE 1: DATA FORMAT                                               │
│  ├─ What: JSON schemas, controlled vocabularies, validation rules  │
│  ├─ Why: Ensure consistent, machine-readable data structure        │
│  ├─ For: Data collectors, database designers                       │
│  └─ Output: Standardized observation and measurement records       │
│                                                                     │
│                        ⬇                                           │
│                                                                     │
│  PHASE 2: API INTERFACE                                             │
│  ├─ What: RESTful endpoints, WebSocket streaming, bulk access      │
│  ├─ Why: Enable programmatic data submission and retrieval         │
│  ├─ For: Software developers, data managers                        │
│  └─ Output: Interoperable data access across systems               │
│                                                                     │
│                        ⬇                                           │
│                                                                     │
│  PHASE 3: PROTOCOL                                                  │
│  ├─ What: QA/QC procedures, calibration standards, field methods   │
│  ├─ Why: Ensure data quality and scientific rigor                  │
│  ├─ For: Field researchers, lab technicians                        │
│  └─ Output: Publication-quality datasets with provenance           │
│                                                                     │
│                        ⬇                                           │
│                                                                     │
│  PHASE 4: SYSTEM INTEGRATION                                        │
│  ├─ What: GIS connections, cloud platforms, conservation databases │
│  ├─ Why: Connect WIA systems to existing infrastructure            │
│  ├─ For: IT professionals, system administrators                   │
│  └─ Output: Seamless data flow across organizational boundaries    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.1.2 Why Four Phases?

```typescript
// Rationale for phased approach
interface PhasedApproachRationale {
  separation_of_concerns: {
    benefit: "Each phase addresses distinct technical domain";
    example: "Data scientists focus on format, DevOps on integration";
  };

  incremental_adoption: {
    benefit: "Organizations can implement phases progressively";
    example: "Start with Phase 1 (data format), add API later";
  };

  expertise_matching: {
    benefit: "Different phases require different skills";
    example: "Ecologists design protocols, engineers build APIs";
  };

  versioning_independence: {
    benefit: "Phases can evolve at different rates";
    example: "Add new data types without changing API";
  };

  clear_responsibilities: {
    benefit: "Stakeholders know what applies to them";
    example: "Field crews follow Phase 3, ignore Phases 2 and 4";
  };
}
```

---

## 3.2 Phase 1: Data Format Specification

### 3.2.1 Core Components

**Species Observation Schema**
- Taxonomic information with authority
- Detection method and occurrence status
- Life stage, sex, abundance
- Associated environmental conditions

**Sensor Time Series Schema**
- Sensor metadata (type, calibration, deployment)
- Readings with timestamps and QC flags
- Battery voltage and signal strength
- Measurement units (QUDT standard)

**Water Quality Schema**
- Sample ID and site information
- Physical parameters (temp, pH, DO, turbidity)
- Chemical parameters (nutrients, salinity)
- Laboratory methods and detection limits

**Air Quality Schema**
- Station ID and location
- Pollutant concentrations (PM2.5, O3, NO2, etc.)
- Meteorological conditions
- AQI values and categories

**Base Schema (Common to All)**
- WIA version identifier
- Unique record ID (UUID)
- Timestamp (ISO 8601)
- Location (lat/lon, elevation, datum)
- Observer/sensor identification
- Quality metadata (validation status, flags, confidence)

### 3.2.2 Design Principles

```
Data Format Design Principles:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  1. JSON-NATIVE                                                     │
│     ├─ Human-readable text format                                  │
│     ├─ Widely supported by all programming languages               │
│     ├─ Easy to validate with JSON Schema                           │
│     └─ Web API friendly                                            │
│                                                                     │
│  2. SELF-DESCRIBING                                                 │
│     ├─ Field names are explicit ("temperature_celsius" not "temp") │
│     ├─ Units included in field names or metadata                   │
│     ├─ Schema version embedded in each record                      │
│     └─ References to external standards (ENVO, QUDT)               │
│                                                                     │
│  3. EXTENSIBLE                                                      │
│     ├─ Additional fields allowed (marked as "optional")            │
│     ├─ New schema types can be added                               │
│     ├─ Backward compatible versioning                              │
│     └─ Custom extensions via namespacing                           │
│                                                                     │
│  4. VALIDATION-READY                                                │
│     ├─ Required vs. optional fields explicit                       │
│     ├─ Data types specified (number, string, enum)                 │
│     ├─ Range constraints for numeric values                        │
│     ├─ Controlled vocabularies for categorical data                │
│     └─ Cross-field validation rules                                │
│                                                                     │
│  5. INTEROPERABILITY-FIRST                                          │
│     ├─ Mappings to Darwin Core provided                            │
│     ├─ Compatible with GeoJSON for spatial data                    │
│     ├─ ISO standards referenced (8601, 19115)                      │
│     └─ Alignment with domain ontologies (ENVO, QUDT)               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2.3 Example: Species Observation

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
    "location_name": "Discovery Park, Seattle"
  },

  "observer": {
    "id": "orcid:0000-0002-1825-0097",
    "name": "Jane Smith",
    "organization": "Seattle Audubon Society"
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
    "taxon_authority": "Catalogue of Life 2025"
  },

  "detection_method": "visual_survey",
  "occurrence_status": "present",
  "abundance": 2,
  "life_stage": "adult",
  "behavior": "Perched in snag, vocalizing",

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 1.0
  }
}
```

---

## 3.3 Phase 2: API Interface Specification

### 3.3.1 RESTful API Design

**Core Endpoints:**

```
GET    /v1/observations          # Query observations
POST   /v1/observations          # Submit new observation(s)
GET    /v1/observations/{id}     # Retrieve specific observation
GET    /v1/sensors               # List sensors
GET    /v1/sensors/{id}/data     # Retrieve sensor time series
GET    /v1/sites                 # Query monitoring sites
GET    /v1/datasets              # Discover datasets
POST   /v1/bulk-query            # Asynchronous large queries
GET    /v1/jobs/{id}             # Check job status
```

**Query Parameters:**
- `taxon`: Filter by scientific name
- `start_date`, `end_date`: Date range
- `bbox`: Bounding box (minLon,minLat,maxLon,maxLat)
- `limit`, `offset`: Pagination
- `format`: Response format (json, csv, geojson)

**Authentication:**
- API keys for simple access
- OAuth 2.0 for user-delegated access
- JWT tokens for service-to-service

**Rate Limiting:**
- Anonymous: 100 requests/hour
- Authenticated: 1,000 requests/hour
- Premium: 10,000 requests/hour

### 3.3.2 Real-Time Streaming

**WebSocket Protocol:**
```javascript
// Connect to real-time sensor stream
const ws = new WebSocket('wss://api.example.org/stream');

ws.send(JSON.stringify({
  action: 'subscribe',
  sensors: ['WEATHER-001', 'WATER-QUALITY-042'],
  filters: { quality_min: 0.8 }
}));

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(`${data.sensor_id}: ${data.value} ${data.unit}`);
};
```

**MQTT Topics:**
```
sensors/SENSOR-001/data       # Sensor readings
sensors/SENSOR-001/status     # Online/offline, battery
sensors/SENSOR-001/alerts     # Threshold exceedances
observations/SITE-042/species # New species detections
```

### 3.3.3 Bulk Data Access

For large datasets (millions of records), asynchronous queries:

```typescript
// Submit bulk query
const response = await fetch('/v1/bulk-query', {
  method: 'POST',
  body: JSON.stringify({
    type: 'observations',
    filters: {
      taxon: 'Ursus arctos',
      start_date: '2020-01-01',
      end_date: '2024-12-31',
      bbox: [-125, 40, -110, 50]
    },
    format: 'csv'
  })
});

const job = await response.json();
// Returns: { job_id: "abc123", status: "processing" }

// Poll for completion
const result = await fetch(`/v1/jobs/${job.job_id}`);
// When complete: { status: "complete", download_url: "..." }
```

---

## 3.4 Phase 3: Protocol Specification

### 3.4.1 Quality Assurance Requirements

**Field QA/QC:**
- Field blanks: 5% of samples
- Equipment blanks: Before/after events
- Field replicates: 10% of samples
- Positive controls: For detection methods

**Laboratory QA/QC:**
- Method blanks: Each analytical batch
- Calibration verification: Every 10 samples
- Spike recovery: 10% (acceptance: 75-125%)
- Duplicate analysis: 10% (RPD < 20%)
- Certified reference materials: Each batch

**Automated Validation:**
- Range checks (physical limits)
- Rate-of-change detection
- Flatline detection
- Spike/outlier detection
- Cross-parameter consistency

### 3.4.2 Calibration Standards

Minimum calibration frequency by sensor type:

| Sensor Type | Interval | Method |
|-------------|----------|--------|
| Temperature | 6 months | Ice point check (0°C) |
| pH | 2 weeks | Two-point buffer calibration |
| Dissolved Oxygen | 1 month | Water-saturated air |
| Conductivity | 3 months | Standard solution |
| Turbidity | 1 month | Formazin standards |
| Air quality (PM) | 6 months | Co-location with reference |
| Flow meters | 1 year | Known volume method |

### 3.4.3 Field Sampling Protocols

**Point Count Protocol (Birds):**
1. Fixed location for 5-10 minutes
2. Record all detections within specified radius (often 50m)
3. Note distance bands for detectability correction
4. Conduct during appropriate season (breeding) and time (dawn)
5. Multiple visits (3-5) to estimate detection probability
6. Document weather, noise, observer

**Water Sampling Standard Procedure:**
1. Approach sampling location from downstream
2. Rinse bottle 3× with ambient water
3. Collect mid-stream, mid-depth (avoid surface and sediment)
4. Fill completely (no headspace for DO samples)
5. Preserve immediately if required (acid for metals, freeze for organics)
6. Label with waterproof marker (site, date, time, initials)
7. Document field parameters (temp, pH, DO in situ)
8. Maintain chain of custody
9. Transport on ice, process within holding time

---

## 3.5 Phase 4: System Integration Specification

### 3.5.1 GIS Platform Integration

**QGIS Plugin:**
- Load WIA data directly into QGIS
- Query by spatial extent (current map view)
- Style observations by taxon, quality, date
- Export to Shapefile, GeoPackage

**ArcGIS Integration:**
- WIA REST Feature Service connector
- Real-time dashboard widgets
- ArcGIS Field Maps data collection
- Geoprocessing tools for import/export

**OGC Web Services:**
- WFS (Web Feature Service) for vector data
- WMS (Web Map Service) for visualization
- CSW (Catalog Service) for metadata discovery

### 3.5.2 Conservation Database Integration

**GBIF (Global Biodiversity Information Facility):**
```typescript
// WIA to Darwin Core Archive export
function exportToGBIF(wiaObservations) {
  return {
    'meta.xml': generateMetaXML(),
    'occurrence.txt': wiaObservations.map(obs => ({
      occurrenceID: obs.record_id,
      eventDate: obs.timestamp,
      decimalLatitude: obs.location.latitude,
      decimalLongitude: obs.location.longitude,
      scientificName: obs.taxon.scientific_name,
      individualCount: obs.abundance,
      samplingProtocol: obs.detection_method,
      // ... additional mappings
    })),
    'eml.xml': generateEML(metadata)
  };
}
```

**iNaturalist Bidirectional Sync:**
- Import research-grade observations via API
- Export WIA observations to iNaturalist
- Leverage community ID and AI suggestions
- Sync taxonomic updates

**eBird Integration:**
- Import eBird checklists as WIA observations
- Maintain observer effort metadata
- Preserve complete vs. incomplete checklist status

### 3.5.3 Cloud Platform Deployment

**AWS Architecture:**
```
WIA Ecosystem Monitoring on AWS:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  ┌──────────────┐         ┌──────────────┐         ┌─────────────┐ │
│  │ IoT Core     │────────►│ Lambda       │────────►│ S3          │ │
│  │ (MQTT)       │         │ (Validation) │         │ (Raw data)  │ │
│  └──────────────┘         └──────────────┘         └─────────────┘ │
│         │                         │                                │
│         │                         ▼                                │
│         │                 ┌──────────────┐                         │
│         │                 │ RDS/PostGIS  │                         │
│         │                 │ (Validated)  │                         │
│         │                 └──────────────┘                         │
│         │                         │                                │
│         ▼                         ▼                                │
│  ┌──────────────┐         ┌──────────────┐                         │
│  │ SageMaker    │         │ API Gateway  │                         │
│  │ (ML models)  │         │ (REST/WS)    │                         │
│  └──────────────┘         └──────────────┘                         │
│                                   │                                │
│                                   ▼                                │
│                           ┌──────────────┐                         │
│                           │ CloudFront   │                         │
│                           │ (CDN)        │                         │
│                           └──────────────┘                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3.6 Comparison with Existing Standards

### 3.6.1 How WIA Differs

| Aspect | Darwin Core | NetCDF-CF | WIA Ecosystem |
|--------|-------------|-----------|---------------|
| **Primary Focus** | Biodiversity specimens/obs | Climate/sensor data | Integrated ecosystem |
| **Data Model** | Flat table | Multidimensional arrays | JSON hierarchical |
| **Quality Metadata** | Limited | Extensive | Comprehensive |
| **API Specification** | None (data format only) | None | RESTful + streaming |
| **Real-time Support** | No | No | Yes (WebSocket, MQTT) |
| **Sensor Integration** | No | Yes | Yes |
| **Species Observations** | Yes | No | Yes |
| **Water/Air Quality** | No | Limited | Yes |
| **Adoption Barrier** | Low | Medium | Low-Medium |

### 3.6.2 Complementary Relationship

WIA does not replace existing standards—it bridges them:

```
WIA as Integration Layer:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  Legacy & Specialized Standards                                     │
│  ├─ Darwin Core (GBIF, museum collections)                         │
│  ├─ NetCDF (climate models, ocean data)                            │
│  ├─ EML (LTER metadata)                                            │
│  ├─ NEON data products                                             │
│  └─ eBird, iNaturalist formats                                     │
│                                                                     │
│                        ⬇ (Converters provided)                     │
│                                                                     │
│  ┌────────────────────────────────────────────────────────────┐    │
│  │                                                            │    │
│  │          WIA ECOSYSTEM MONITORING STANDARD                 │    │
│  │                                                            │    │
│  │  Unified API · Common JSON Format · QA/QC Protocol         │    │
│  │                                                            │    │
│  └────────────────────────────────────────────────────────────┘    │
│                                                                     │
│                        ⬇ (Standard interface)                      │
│                                                                     │
│  Modern Analysis & Visualization Tools                              │
│  ├─ GIS platforms (ArcGIS, QGIS)                                   │
│  ├─ Statistical software (R, Python)                               │
│  ├─ Cloud platforms (AWS, Google Earth Engine)                     │
│  ├─ Decision support (Marxan, InVEST)                              │
│  └─ Custom applications                                            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3.7 Adoption Pathways

### 3.7.1 Minimal Implementation

**For individual researchers or small organizations:**

```typescript
interface MinimalImplementation {
  phase1: {
    action: "Export data to WIA JSON format";
    effort: "1-2 days with provided tools";
    benefit: "Data discoverable and interoperable";
  };

  phase2: {
    action: "Optional - can skip if not sharing real-time";
    effort: "N/A";
  };

  phase3: {
    action: "Document existing QA/QC in WIA template";
    effort: "Half day";
    benefit: "Data quality transparent to users";
  };

  phase4: {
    action: "Optional - can skip if not integrating systems";
    effort: "N/A";
  };

  totalEffort: "2-3 days";
  outcome: "WIA-compliant dataset ready for publication";
}
```

### 3.7.2 Full Implementation

**For organizations building monitoring infrastructure:**

```typescript
interface FullImplementation {
  phase1: {
    action: "Design database schema based on WIA format";
    effort: "1 week";
    benefit: "Native WIA support from start";
  };

  phase2: {
    action: "Implement API endpoints, authentication, rate limiting";
    effort: "2-4 weeks";
    benefit: "Programmatic data access for partners";
  };

  phase3: {
    action: "Develop QA/QC pipeline, calibration tracking";
    effort: "2-3 weeks";
    benefit: "Automated quality control, reduced manual review";
  };

  phase4: {
    action: "Build connectors to GIS, cloud platforms, repositories";
    effort: "3-4 weeks";
    benefit: "Seamless workflow, reduced manual export";
  };

  totalEffort: "8-12 weeks";
  outcome: "Enterprise-grade WIA monitoring platform";
}
```

### 3.7.3 Incremental Adoption

Organizations can adopt phases incrementally:

```
Year 1: Implement Phase 1 (data format)
  → Gain: Standardized internal data, easier analysis

Year 2: Add Phase 3 (protocols)
  → Gain: Improved data quality, documented methods

Year 3: Implement Phase 2 (API)
  → Gain: Enable partner access, reduce manual requests

Year 4: Complete Phase 4 (integration)
  → Gain: Full ecosystem connectivity
```

---

## 3.8 Review Questions

### Question 1
Explain why WIA uses a 4-phase architecture instead of a single monolithic specification. What are the advantages for different types of adopters?

### Question 2
Compare WIA Ecosystem Monitoring Standard with Darwin Core. In what scenarios would you use each? Can they be used together?

### Question 3
A research station has 10 years of bird survey data in Excel spreadsheets. Design an adoption plan to make this data WIA-compliant with minimal effort.

### Question 4
Explain the difference between the RESTful API (Phase 2) and the MQTT streaming protocol. When would you use each?

### Question 5
An organization wants to implement only Phases 1 and 3, skipping Phases 2 and 4. Is this a valid approach? What would they gain and what would they lose?

---

## 3.9 Key Takeaways

| Component | Purpose | Key Feature |
|-----------|---------|-------------|
| **Phase 1: Data Format** | Standardize data structure | JSON schemas with validation |
| **Phase 2: API Interface** | Enable programmatic access | REST + WebSocket + MQTT |
| **Phase 3: Protocol** | Ensure data quality | QA/QC, calibration, sampling methods |
| **Phase 4: Integration** | Connect to ecosystem | GIS, databases, cloud platforms |

### Design Principles
- **JSON-native**: Human-readable, widely supported
- **Self-describing**: Units and metadata embedded
- **Extensible**: Room for future additions
- **Validation-ready**: Automated quality checks
- **Interoperable**: Works with existing standards

### Adoption Flexibility
- **Minimal**: 2-3 days (export existing data)
- **Full**: 8-12 weeks (new infrastructure)
- **Incremental**: Adopt phases progressively

### Next Chapter Preview

Chapter 4 provides detailed coverage of Phase 1 (Data Format), including complete JSON schemas, validation rules, and examples for all observation and measurement types.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
