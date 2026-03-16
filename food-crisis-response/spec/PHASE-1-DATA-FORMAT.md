# WIA-AGRI-030: Food Crisis Response
## PHASE 1 - DATA FORMAT SPECIFICATION

**Version:** 1.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

The WIA-AGRI-030 Food Crisis Response standard defines comprehensive data formats for managing food security emergencies, coordinating relief efforts, and tracking resource distribution across global networks.

### 1.1 Purpose

- Standardize crisis reporting and assessment data
- Enable real-time monitoring of food security indicators
- Facilitate coordination among governments, NGOs, and international organizations
- Support AI-driven resource allocation and prediction

### 1.2 Scope

This specification covers:
- Crisis event data structures
- Food security metrics
- Resource inventory and distribution tracking
- Stakeholder coordination messages
- Emergency alert formats

---

## 2. Core Data Structures

### 2.1 Crisis Event Record

```json
{
  "crisisId": "CRISIS-2025-AF-001",
  "type": "drought|flood|conflict|economic|pest|supply",
  "severity": 1-3,
  "region": {
    "country": "Ethiopia",
    "province": "Tigray",
    "coordinates": {
      "lat": 14.2633,
      "lng": 38.2775
    },
    "affectedArea": 45000
  },
  "population": {
    "total": 5200000,
    "affected": 3800000,
    "displaced": 450000,
    "children": 1200000,
    "elderly": 280000
  },
  "timeline": {
    "detectedAt": "2025-01-15T08:30:00Z",
    "reportedAt": "2025-01-15T09:45:00Z",
    "peakExpected": "2025-03-01T00:00:00Z",
    "estimatedDuration": 180
  },
  "triggers": [
    {
      "type": "rainfall-deficit",
      "value": -65,
      "unit": "percent",
      "period": "90-days"
    },
    {
      "type": "crop-failure",
      "value": 78,
      "unit": "percent",
      "crops": ["wheat", "teff", "barley"]
    }
  ],
  "status": "active|monitoring|resolved",
  "confidence": 0.94
}
```

### 2.2 Food Security Assessment

```json
{
  "assessmentId": "FSA-2025-001",
  "crisisId": "CRISIS-2025-AF-001",
  "timestamp": "2025-01-16T12:00:00Z",
  "region": "Tigray, Ethiopia",
  "ipcPhase": {
    "current": 3,
    "projected": 4,
    "projectionDate": "2025-03-01"
  },
  "indicators": {
    "foodConsumptionScore": {
      "poor": 45,
      "borderline": 32,
      "acceptable": 23
    },
    "copingStrategyIndex": {
      "value": 42,
      "threshold": 18,
      "status": "severe"
    },
    "malnutrition": {
      "gam": 18.5,
      "sam": 4.2,
      "unit": "percent"
    },
    "mortality": {
      "crude": 1.8,
      "under5": 3.2,
      "unit": "per-10000-per-day"
    }
  },
  "needs": {
    "food": {
      "cereals": 45000,
      "pulses": 8000,
      "oil": 2500,
      "unit": "metric-tons",
      "duration": 180
    },
    "nutrition": {
      "rutf": 850000,
      "micronutrients": 1200000,
      "unit": "sachets"
    },
    "water": {
      "drinking": 75000000,
      "agriculture": 250000000,
      "unit": "liters-per-day"
    }
  },
  "assessmentMethod": "household-survey|satellite|mixed",
  "sampleSize": 2400,
  "confidence": 0.95
}
```

### 2.3 Resource Inventory

```json
{
  "inventoryId": "INV-ETH-2025-001",
  "location": {
    "facilityId": "WFP-ADDIS-01",
    "name": "WFP Addis Ababa Regional Hub",
    "coordinates": { "lat": 9.0320, "lng": 38.7469 },
    "capacity": 50000
  },
  "timestamp": "2025-01-16T14:30:00Z",
  "items": [
    {
      "category": "food",
      "subcategory": "cereals",
      "type": "wheat-flour",
      "quantity": 15000,
      "unit": "metric-tons",
      "expiryDate": "2026-06-30",
      "condition": "good|fair|poor",
      "reserved": 5000,
      "available": 10000
    },
    {
      "category": "nutrition",
      "type": "RUTF",
      "quantity": 500000,
      "unit": "sachets",
      "expiryDate": "2025-12-31",
      "condition": "good",
      "reserved": 200000,
      "available": 300000
    }
  ],
  "replenishment": {
    "scheduled": "2025-02-01T00:00:00Z",
    "quantity": 20000,
    "source": "global-reserve"
  }
}
```

### 2.4 Distribution Tracking

```json
{
  "shipmentId": "SHIP-2025-001",
  "crisisId": "CRISIS-2025-AF-001",
  "status": "planned|in-transit|delivered|delayed|cancelled",
  "priority": "critical|high|medium|low",
  "origin": {
    "facilityId": "WFP-ADDIS-01",
    "location": "Addis Ababa, Ethiopia",
    "coordinates": { "lat": 9.0320, "lng": 38.7469 }
  },
  "destination": {
    "distributionPointId": "DP-TIG-045",
    "location": "Mekelle Distribution Center",
    "coordinates": { "lat": 13.4967, "lng": 39.4753 },
    "beneficiaries": 45000
  },
  "cargo": [
    {
      "type": "wheat-flour",
      "quantity": 500,
      "unit": "metric-tons",
      "packaging": "50kg-bags",
      "count": 10000
    },
    {
      "type": "vegetable-oil",
      "quantity": 50,
      "unit": "metric-tons",
      "packaging": "1L-bottles",
      "count": 50000
    }
  ],
  "timeline": {
    "scheduledDeparture": "2025-01-18T06:00:00Z",
    "actualDeparture": "2025-01-18T07:30:00Z",
    "eta": "2025-01-20T16:00:00Z",
    "actualArrival": null
  },
  "tracking": {
    "currentLocation": { "lat": 10.5631, "lng": 38.9847 },
    "lastUpdate": "2025-01-19T08:15:00Z",
    "progress": 42,
    "estimatedDelay": 0
  },
  "verification": {
    "blockchainHash": "0x7a8f3e...",
    "signature": "WFP-DIST-SIG",
    "witnesses": ["WFP-Staff-001", "Local-Official-012"]
  }
}
```

### 2.5 Early Warning Alert

```json
{
  "alertId": "ALERT-2025-SA-003",
  "type": "food-crisis-warning",
  "severity": "advisory|watch|warning|emergency",
  "region": {
    "countries": ["Somalia", "Kenya", "Ethiopia"],
    "coordinates": [
      { "lat": 2.0469, "lng": 45.3182 },
      { "lat": -1.2864, "lng": 36.8172 },
      { "lat": 9.1450, "lng": 40.4897 }
    ]
  },
  "issuedAt": "2025-01-10T00:00:00Z",
  "validUntil": "2025-04-30T23:59:59Z",
  "triggers": [
    {
      "indicator": "rainfall-forecast",
      "value": -55,
      "threshold": -40,
      "confidence": 0.87
    },
    {
      "indicator": "vegetation-health",
      "value": 0.35,
      "threshold": 0.50,
      "confidence": 0.92
    }
  ],
  "predictions": {
    "affectedPopulation": 8500000,
    "ipcPhase": 3,
    "timeline": "2025-03-01 to 2025-06-30",
    "confidence": 0.84
  },
  "recommendations": [
    "Preposition food stocks in regional hubs",
    "Activate emergency procurement protocols",
    "Increase humanitarian funding appeals",
    "Strengthen early action partnerships"
  ],
  "stakeholders": [
    "government-ethiopia",
    "government-kenya",
    "government-somalia",
    "wfp",
    "fao",
    "unicef",
    "red-cross"
  ]
}
```

---

## 3. Data Types and Constraints

### 3.1 Severity Levels

| Level | Name | Description | Response |
|-------|------|-------------|----------|
| 1 | Stable | Minimal food insecurity | Monitoring |
| 2 | Moderate Risk | Stressed/Crisis IPC phases | Active preparation |
| 3 | Critical Emergency | Emergency/Famine IPC phases | Full mobilization |

### 3.2 Crisis Types

- **drought**: Prolonged rainfall deficit affecting crops
- **flood**: Excessive water destroying crops/infrastructure
- **conflict**: Armed conflict disrupting food production/distribution
- **economic**: Economic collapse affecting food access
- **pest**: Locust or other pest outbreaks
- **supply**: Supply chain disruptions (e.g., transport, market)

### 3.3 IPC (Integrated Food Security Phase Classification)

| Phase | Name | Characteristics |
|-------|------|----------------|
| 1 | Minimal | Food secure, minimal acute malnutrition |
| 2 | Stressed | Borderline food consumption, acute malnutrition 5-10% |
| 3 | Crisis | Food consumption gaps, acute malnutrition 10-15% |
| 4 | Emergency | Large food gaps, acute malnutrition >15%, excess mortality |
| 5 | Famine | Extreme food shortage, starvation, >30% malnutrition, >2/10,000/day mortality |

### 3.4 Units and Standards

- **Mass**: metric tons (MT)
- **Volume**: liters (L)
- **Area**: square kilometers (km²)
- **Population**: persons
- **Coordinates**: WGS84 decimal degrees
- **Timestamps**: ISO 8601 (UTC)
- **Malnutrition**: GAM (Global Acute Malnutrition), SAM (Severe Acute Malnutrition) as percentage
- **Mortality**: per 10,000 persons per day

---

## 4. Validation Rules

### 4.1 Required Fields

All crisis records MUST include:
- crisisId (unique identifier)
- type (valid crisis type)
- severity (1-3)
- region (with coordinates)
- population.affected
- timeline.detectedAt
- status

### 4.2 Data Constraints

- Severity: integer 1-3
- IPC Phase: integer 1-5
- Confidence: float 0.0-1.0
- Coordinates: lat (-90 to 90), lng (-180 to 180)
- Timestamps: ISO 8601 format, UTC timezone
- Quantities: positive numbers
- Percentages: 0-100

### 4.3 Referential Integrity

- crisisId must be unique per event
- assessmentId references valid crisisId
- shipmentId references valid crisisId
- facilityId must exist in facility registry
- beneficiaries must match distribution point capacity

---

## 5. Interoperability

### 5.1 Compatible Standards

- **WIA-AGRI-001**: Smart Farm (crop production data)
- **WIA-AGRI-015**: Food Traceability (supply chain tracking)
- **WIA-AGRI-021**: Agricultural Supply Chain (logistics)
- **ISO 8601**: Date/time formats
- **WGS84**: Geographic coordinates
- **IPC**: Food security classification

### 5.2 External System Integration

- UN OCHA HDX (Humanitarian Data Exchange)
- WFP VAM (Vulnerability Analysis and Mapping)
- FAO GIEWS (Global Information and Early Warning System)
- FEWS NET (Famine Early Warning Systems Network)

---

## 6. Security and Privacy

### 6.1 Data Classification

- **Public**: Aggregated statistics, general alerts
- **Restricted**: Detailed assessments, resource inventories
- **Confidential**: Individual beneficiary data, security-sensitive locations

### 6.2 Access Control

- Crisis data: Read access for all stakeholders, write access for authorized organizations
- Assessment data: Restricted to humanitarian organizations and governments
- Distribution tracking: Blockchain-verified, publicly auditable
- Beneficiary data: Encrypted, access limited to implementing partners

### 6.3 Data Retention

- Active crisis data: Retained for duration + 5 years
- Assessment data: 10 years minimum
- Distribution records: Permanent (blockchain)
- Alert history: 10 years

---

## 7. Version Control

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-26 | Initial specification release |

---

**Next Phase**: [PHASE-2-API-INTERFACE.md](PHASE-2-API-INTERFACE.md)

---

弘益人間 · Benefit All Humanity
© 2025 WIA Standards - MIT License
