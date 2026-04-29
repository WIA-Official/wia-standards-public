# WIA-OCEAN_CONSERVATION: Phase 1 - Data Format Specification
## Version 1.0 | 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the standardized data formats for ocean conservation activities, including marine species tracking, ecosystem health monitoring, Marine Protected Areas (MPAs) management, illegal fishing detection, pollution tracking, and coral reef restoration. All data formats support interoperability across conservation organizations, research institutions, enforcement agencies, and citizen science platforms.

**Philosophy**: 弘益人間 - Protecting our oceans benefits all humanity and future generations.

---

## 2. Core Data Structures

### 2.1 Marine Species Data

#### 2.1.1 Species Observation Record

```json
{
  "observationId": "uuid-v4",
  "speciesCode": "IUCN-species-code",
  "commonName": "Green Sea Turtle",
  "scientificName": "Chelonia mydas",
  "conservationStatus": "ENDANGERED",
  "timestamp": "2026-01-12T14:30:00Z",
  "location": {
    "latitude": -23.5505,
    "longitude": -46.6333,
    "depth": 15.5,
    "coordinateSystem": "WGS84",
    "accuracy": 10
  },
  "observationType": "VISUAL|ACOUSTIC|SATELLITE|CAMERA_TRAP|DNA",
  "count": 1,
  "behavior": "FEEDING|BREEDING|MIGRATING|RESTING",
  "ageClass": "JUVENILE|ADULT|UNKNOWN",
  "health": {
    "condition": "HEALTHY|INJURED|DISEASED",
    "injuries": ["FISHING_NET_ENTANGLEMENT"],
    "parasites": false
  },
  "observer": {
    "observerId": "uuid-v4",
    "type": "RESEARCHER|CITIZEN_SCIENTIST|AUTOMATED_SYSTEM",
    "credentials": "marine-biologist-cert-12345"
  },
  "media": [{
    "type": "PHOTO|VIDEO|AUDIO",
    "url": "https://storage.wia.org/obs/12345.jpg",
    "timestamp": "2026-01-12T14:30:00Z"
  }],
  "environmentalConditions": {
    "waterTemperature": 24.5,
    "salinity": 35.2,
    "visibility": 20,
    "currentSpeed": 0.5
  },
  "metadata": {
    "verified": true,
    "verifiedBy": "expert-uuid",
    "confidence": 0.95,
    "tags": ["migration-study", "health-assessment"]
  }
}
```

#### 2.1.2 Species Population Data

```json
{
  "populationId": "uuid-v4",
  "speciesCode": "IUCN-species-code",
  "region": "MEDITERRANEAN_SEA",
  "estimatedPopulation": 15000,
  "estimationMethod": "MARK_RECAPTURE|AERIAL_SURVEY|ACOUSTIC_MONITORING",
  "confidenceInterval": {
    "lower": 12000,
    "upper": 18000,
    "confidence": 0.95
  },
  "surveyPeriod": {
    "start": "2025-06-01T00:00:00Z",
    "end": "2025-08-31T23:59:59Z"
  },
  "populationTrend": "INCREASING|DECREASING|STABLE|UNKNOWN",
  "trendRate": 0.03,
  "threats": [
    {
      "threatType": "BYCATCH",
      "severity": "HIGH|MEDIUM|LOW",
      "affectedPercentage": 0.15
    }
  ]
}
```

### 2.2 Ecosystem Health Metrics

#### 2.2.1 Coral Reef Health Assessment

```json
{
  "assessmentId": "uuid-v4",
  "reefId": "reef-uuid",
  "reefName": "Great Barrier Reef - Section A12",
  "location": {
    "latitude": -18.2871,
    "longitude": 147.6992,
    "area": 5000
  },
  "assessmentDate": "2026-01-12T00:00:00Z",
  "coralCoverPercentage": 45.2,
  "bleachingLevel": "NONE|MILD|MODERATE|SEVERE",
  "bleachedPercentage": 12.5,
  "diversityIndex": {
    "shannonIndex": 2.3,
    "simpsonIndex": 0.85
  },
  "dominantSpecies": [
    {
      "species": "Acropora cervicornis",
      "coverage": 18.5
    }
  ],
  "healthIndicators": {
    "algaeCoverage": 15.2,
    "diseasePrevalence": 3.5,
    "recruitmentRate": 25.0,
    "mortality": 8.5
  },
  "waterQuality": {
    "temperature": 28.5,
    "pH": 8.15,
    "dissolvedOxygen": 6.8,
    "turbidity": 2.5,
    "nutrientLevels": {
      "nitrate": 0.15,
      "phosphate": 0.08
    }
  },
  "threats": ["OCEAN_WARMING", "ACIDIFICATION", "CROWN_OF_THORNS"],
  "restorationActivities": [{
    "type": "CORAL_TRANSPLANTATION",
    "area": 500,
    "date": "2025-12-01T00:00:00Z",
    "survivorship": 0.85
  }]
}
```

#### 2.2.2 Ocean Acidification Data

```json
{
  "measurementId": "uuid-v4",
  "location": {
    "latitude": 35.6762,
    "longitude": 139.6503,
    "depth": 50
  },
  "timestamp": "2026-01-12T12:00:00Z",
  "pH": 8.05,
  "pCO2": 450,
  "totalAlkalinity": 2300,
  "dissolvedInorganicCarbon": 2100,
  "aragoniteSaturation": 2.5,
  "calciteSaturation": 3.8,
  "temperature": 18.5,
  "salinity": 34.8,
  "trend": {
    "pHChangePerDecade": -0.018,
    "projectedPH2050": 7.95
  }
}
```

### 2.3 Marine Protected Area (MPA) Data

```json
{
  "mpaId": "uuid-v4",
  "name": "Papahānaumokuākea Marine National Monument",
  "designation": "NATIONAL_MONUMENT|MARINE_RESERVE|SANCTUARY",
  "iucnCategory": "Ia|Ib|II|III|IV|V|VI",
  "boundary": {
    "type": "Polygon",
    "coordinates": [/* GeoJSON coordinates */]
  },
  "area": 1508870,
  "establishedDate": "2006-06-15",
  "managementAuthority": "NOAA|State|International",
  "protectionLevel": "NO_TAKE|RESTRICTED|MULTIPLE_USE",
  "regulations": [{
    "type": "FISHING_BAN",
    "description": "All extractive activities prohibited",
    "penalties": "Fine up to $100,000"
  }],
  "zoning": [{
    "zone": "CORE_PROTECTION",
    "area": 1000000,
    "allowedActivities": ["RESEARCH"]
  }],
  "biodiversity": {
    "speciesCount": 7000,
    "endemicSpecies": 1400,
    "threatenedSpecies": 23
  },
  "monitoring": {
    "frequency": "QUARTERLY",
    "lastSurvey": "2026-01-01T00:00:00Z",
    "complianceRate": 0.98
  }
}
```

### 2.4 Illegal Fishing Detection Data

```json
{
  "detectionId": "uuid-v4",
  "timestamp": "2026-01-12T03:45:00Z",
  "detectionMethod": "SATELLITE|AIS|RADAR|PATROL|DRONE",
  "vessel": {
    "vesselId": "IMO-1234567",
    "name": "Suspicious Vessel",
    "flag": "UNKNOWN",
    "type": "FISHING_VESSEL",
    "mmsi": "123456789",
    "length": 45.5
  },
  "location": {
    "latitude": -10.5678,
    "longitude": 142.3456,
    "eez": "AUSTRALIA"
  },
  "violation": {
    "type": "UNAUTHORIZED_FISHING|GEAR_VIOLATION|QUOTA_EXCEED|MPA_INTRUSION",
    "severity": "CRITICAL|HIGH|MEDIUM|LOW",
    "evidence": ["SATELLITE_IMAGE", "AIS_PATTERN"],
    "confidence": 0.92
  },
  "aisPatternsuspicious": {
    "gapDuration": 48,
    "speedInconsistency": true,
    "locationSpoofing": false
  },
  "enforcement": {
    "reported": true,
    "reportedTo": ["COAST_GUARD", "FISHERIES_AUTHORITY"],
    "actionTaken": "VESSEL_INTERCEPTED",
    "outcomeDate": "2026-01-13T00:00:00Z"
  }
}
```

### 2.5 Pollution Tracking Data

```json
{
  "pollutionEventId": "uuid-v4",
  "eventType": "PLASTIC_DEBRIS|OIL_SPILL|CHEMICAL|NUTRIENT_RUNOFF",
  "severity": "CATASTROPHIC|MAJOR|MODERATE|MINOR",
  "detectedDate": "2026-01-12T00:00:00Z",
  "location": {
    "latitude": 29.9511,
    "longitude": -90.0715,
    "affectedArea": 250
  },
  "pollutantDetails": {
    "type": "MICROPLASTICS",
    "concentration": 150000,
    "unit": "particles_per_cubic_meter",
    "composition": ["PET", "POLYPROPYLENE", "POLYETHYLENE"]
  },
  "source": {
    "type": "LAND_BASED|MARINE_BASED|ATMOSPHERIC",
    "identified": true,
    "responsible": "Industrial facility XYZ"
  },
  "impact": {
    "wildlifeAffected": 150,
    "habitatDamage": "MODERATE",
    "economicLoss": 5000000
  },
  "response": {
    "cleanupInitiated": "2026-01-13T00:00:00Z",
    "method": "MECHANICAL|CHEMICAL|BIOLOGICAL",
    "percentageRemoved": 65,
    "status": "IN_PROGRESS|COMPLETED"
  }
}
```

---

## 3. Data Quality Standards

### 3.1 Accuracy Requirements
- Geospatial data: ±10m accuracy minimum
- Species identification: 90% confidence threshold
- Sensor data: Calibration every 6 months
- Time synchronization: UTC with millisecond precision

### 3.2 Validation Rules
- All coordinates must be within valid ocean boundaries
- Species codes must match IUCN Red List
- Timestamps must not be in the future
- Required fields must not be null

### 3.3 Metadata Requirements
- Data provenance tracking
- Version control for all datasets
- License information (CC-BY-SA-4.0 recommended)
- Citation guidelines

---

## 4. Interoperability

### 4.1 Supported Standards
- OBIS (Ocean Biogeographic Information System)
- Darwin Core for biodiversity data
- UNCLOS zones and boundaries
- GeoJSON for spatial data
- ISO 19115 for metadata

### 4.2 Data Exchange Formats
- Primary: JSON, GeoJSON
- Secondary: CSV, NetCDF, HDF5
- APIs: REST, GraphQL

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Protecting Oceans for All Humanity
