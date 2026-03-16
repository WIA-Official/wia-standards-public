# WIA Coral Reef Restoration - Phase 1: Data Format

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

This specification defines the standardized data format for coral reef restoration monitoring, health assessment, and biodiversity tracking. The format supports real-time monitoring, bleaching detection, restoration progress tracking, and marine ecosystem protection.

### 1.1 Purpose

- Enable standardized coral reef health data exchange
- Support bleaching prediction and early warning systems
- Track restoration efforts and success rates
- Monitor marine biodiversity and ecosystem recovery
- Facilitate data sharing among marine parks, researchers, and conservation organizations

### 1.2 Scope

This data format covers:
- Coral coverage and health metrics
- Water quality parameters
- Species diversity tracking
- Bleaching status and risk factors
- Restoration activity logging
- Marine biodiversity indices

---

## 2. Core Data Schema

### 2.1 Reef Health Record

```json
{
  "reefId": "string",
  "location": {
    "name": "string",
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    },
    "zone": "string",
    "depth": "number"
  },
  "timestamp": "ISO8601",
  "observer": {
    "id": "string",
    "name": "string",
    "organization": "string",
    "certification": "string"
  },
  "metrics": {
    "coralCoverage": "number",
    "speciesDiversity": "integer",
    "waterTemperature": "number",
    "bleachingStatus": "enum",
    "phLevel": "number",
    "salinity": "number",
    "turbidity": "number",
    "dissolvedOxygen": "number"
  },
  "healthScore": "number",
  "alerts": ["array"]
}
```

### 2.2 Restoration Activity Record

```json
{
  "activityId": "string",
  "reefId": "string",
  "timestamp": "ISO8601",
  "organization": {
    "id": "string",
    "name": "string",
    "type": "enum"
  },
  "restoration": {
    "type": "enum",
    "areaRestored": "number",
    "coralsPlanted": "integer",
    "speciesPlanted": ["array"],
    "method": "string",
    "cost": "number"
  },
  "success": {
    "survivalRate": "number",
    "growthRate": "number",
    "monthsTracked": "integer"
  }
}
```

### 2.3 Biodiversity Assessment

```json
{
  "assessmentId": "string",
  "reefId": "string",
  "timestamp": "ISO8601",
  "survey": {
    "method": "string",
    "duration": "integer",
    "areasurveyed": "number"
  },
  "species": {
    "coralSpecies": ["array"],
    "fishSpecies": ["array"],
    "invertebrateSpecies": ["array"],
    "totalCount": "integer"
  },
  "indices": {
    "shannon": "number",
    "simpson": "number",
    "evenness": "number"
  },
  "keyIndicatorSpecies": ["array"]
}
```

---

## 3. Field Definitions

### 3.1 Reef Identifiers

| Field | Type | Format | Required | Description |
|-------|------|--------|----------|-------------|
| reefId | string | `REEF-[A-Z0-9]{9}` | Yes | Unique reef identifier |
| location.name | string | UTF-8 | Yes | Human-readable location name |
| location.coordinates.latitude | number | -90 to 90 | Yes | Decimal degrees |
| location.coordinates.longitude | number | -180 to 180 | Yes | Decimal degrees |
| location.zone | string | UTF-8 | No | Management zone designation |
| location.depth | number | meters | Yes | Average depth of monitored area |

### 3.2 Health Metrics

| Field | Type | Range | Unit | Description |
|-------|------|-------|------|-------------|
| coralCoverage | number | 0-100 | % | Percentage of substrate covered by live coral |
| speciesDiversity | integer | 0+ | count | Number of distinct coral species |
| waterTemperature | number | 0-50 | °C | Surface water temperature |
| phLevel | number | 6-9 | pH | Acidity/alkalinity level |
| salinity | number | 0-50 | ppt | Parts per thousand |
| turbidity | number | 0+ | NTU | Nephelometric Turbidity Units |
| dissolvedOxygen | number | 0-20 | mg/L | Dissolved oxygen concentration |

### 3.3 Bleaching Status Enum

| Value | Description | Action Required |
|-------|-------------|-----------------|
| `none` | No bleaching observed | Routine monitoring |
| `mild` | <10% of corals showing bleaching | Increase observation frequency |
| `moderate` | 10-30% bleaching | Daily monitoring, alert stakeholders |
| `severe` | >30% bleaching | Emergency response, mitigation measures |
| `recovering` | Post-bleaching recovery phase | Track recovery progress |

### 3.4 Health Score Calculation

```
healthScore = (coralCoverage * 0.4) +
              (min(speciesDiversity, 50)) +
              bleachingPenalty

bleachingPenalty:
  - none: 0
  - mild: -15
  - moderate: -30
  - severe: -50

Range: 0-100 (higher is better)
```

---

## 4. Restoration Activity Types

### 4.1 Restoration Method Enum

| Type | Description |
|------|-------------|
| `coral_gardening` | Growing coral fragments in nurseries |
| `direct_transplant` | Direct transplantation from healthy reefs |
| `artificial_structure` | Installation of artificial reef structures |
| `larval_seeding` | Coral larvae propagation and seeding |
| `substrate_stabilization` | Reef substrate repair and stabilization |
| `invasive_removal` | Removal of invasive species |

### 4.2 Organization Types

- `marine_park`: Marine protected area management
- `research_institution`: Academic or research organization
- `conservation_ngo`: Non-profit conservation group
- `diving_operator`: Commercial diving operation
- `government_agency`: Government environmental agency
- `community_group`: Local community organization

---

## 5. Biodiversity Metrics

### 5.1 Shannon Diversity Index

```
H = -Σ(pi * ln(pi))

where:
  pi = proportion of individuals of species i
  H > 3.0: Excellent diversity
  H 2.0-3.0: Good diversity
  H < 2.0: Poor diversity
```

### 5.2 Simpson's Index

```
D = 1 - Σ(ni(ni-1)) / (N(N-1))

where:
  ni = number of individuals of species i
  N = total number of individuals
  D → 1: High diversity
  D → 0: Low diversity
```

### 5.3 Key Indicator Species

Species that indicate ecosystem health:
- Staghorn coral (Acropora cervicornis)
- Elkhorn coral (Acropora palmata)
- Brain coral (Diploria spp.)
- Parrotfish (Scaridae family)
- Grouper (Epinephelus spp.)
- Sea urchins (Diadema spp.)

---

## 6. Validation Rules

### 6.1 Data Quality Requirements

1. **Temporal Accuracy**
   - Timestamp must be in ISO 8601 format
   - Cannot be in the future
   - Must include timezone information

2. **Spatial Accuracy**
   - Coordinates must be valid GPS coordinates
   - Depth must be positive
   - Location must be in marine environment

3. **Metric Validity**
   - All percentages: 0-100
   - Temperature: realistic range for marine environment
   - pH: 6.0-9.0 (outside indicates sensor error)
   - All counts: non-negative integers

4. **Observer Credentials**
   - Observer must have valid certification
   - Organization must be registered
   - Contact information required

### 6.2 Data Completeness

**Mandatory fields:**
- reefId
- location (name + coordinates)
- timestamp
- observer information
- At least 3 health metrics

**Optional but recommended:**
- Full water quality parameters
- Biodiversity assessment
- Photo/video documentation
- Weather conditions

---

## 7. Examples

### 7.1 Basic Health Monitoring

```json
{
  "reefId": "REEF-GBR-A0123",
  "location": {
    "name": "Great Barrier Reef, Zone A",
    "coordinates": {
      "latitude": -16.2456,
      "longitude": 145.7823
    },
    "zone": "Marine Park Zone A",
    "depth": 12.5
  },
  "timestamp": "2025-12-25T08:30:00Z",
  "observer": {
    "id": "OBS-001",
    "name": "Dr. Sarah Ocean",
    "organization": "Marine Research Institute",
    "certification": "Coral Reef Monitoring Specialist"
  },
  "metrics": {
    "coralCoverage": 65.5,
    "speciesDiversity": 42,
    "waterTemperature": 28.5,
    "bleachingStatus": "none",
    "phLevel": 8.1,
    "salinity": 35.2,
    "turbidity": 2.1,
    "dissolvedOxygen": 6.8
  },
  "healthScore": 92.5,
  "alerts": []
}
```

### 7.2 Bleaching Alert

```json
{
  "reefId": "REEF-CAR-B0456",
  "location": {
    "name": "Caribbean Reef, Belize",
    "coordinates": {
      "latitude": 17.2512,
      "longitude": -87.5344
    },
    "depth": 8.0
  },
  "timestamp": "2025-12-25T14:15:00Z",
  "observer": {
    "id": "OBS-089",
    "name": "Maria Coral",
    "organization": "Belize Marine Conservation",
    "certification": "Advanced Reef Monitor"
  },
  "metrics": {
    "coralCoverage": 48.2,
    "speciesDiversity": 28,
    "waterTemperature": 31.2,
    "bleachingStatus": "moderate",
    "phLevel": 8.0,
    "salinity": 34.8,
    "turbidity": 4.5,
    "dissolvedOxygen": 5.2
  },
  "healthScore": 56.2,
  "alerts": [
    {
      "type": "BLEACHING_ALERT",
      "severity": "MODERATE",
      "message": "Temperature elevated 3.2°C above seasonal average for 14 days",
      "recommendation": "Increase monitoring to daily, prepare mitigation measures"
    }
  ]
}
```

### 7.3 Restoration Activity

```json
{
  "activityId": "ACT-REST-2025-1234",
  "reefId": "REEF-INDO-C0789",
  "timestamp": "2025-12-20T10:00:00Z",
  "organization": {
    "id": "ORG-CORAL-456",
    "name": "Coral Triangle Restoration Project",
    "type": "conservation_ngo"
  },
  "restoration": {
    "type": "coral_gardening",
    "areaRestored": 2.5,
    "coralsPlanted": 1500,
    "speciesPlanted": [
      "Acropora millepora",
      "Pocillopora damicornis",
      "Montipora digitata"
    ],
    "method": "Mid-water rope nursery transplantation",
    "cost": 15000
  },
  "success": {
    "survivalRate": 87.2,
    "growthRate": 2.3,
    "monthsTracked": 18
  }
}
```

---

## 8. Integration Points

### 8.1 WIA Ecosystem Integration

- **WIA-CLIMATE**: Share ocean temperature and pH data for climate monitoring
- **WIA-BIO**: Integrate with biodiversity tracking systems
- **WIA-OCEAN**: Connect with broader ocean health monitoring
- **WIA-SATELLITE**: Receive satellite imagery for bleaching detection
- **WIA-BLOCKCHAIN**: Immutable restoration activity logging

### 8.2 External Systems

- NOAA Coral Reef Watch
- Global Coral Reef Monitoring Network
- ReefBase Database
- Ocean Health Index
- IUCN Red List of Ecosystems

---

## 9. Versioning

**Current Version:** 1.0.0

### Version History
- 1.0.0 (2025-12-25): Initial release

### Future Enhancements
- Support for 3D reef mapping
- Integration with underwater drone data
- AI-powered species identification
- Genetic diversity tracking
- Ocean acidification metrics

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
