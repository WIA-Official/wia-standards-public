# WIA Mangrove Restoration Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Blue Carbon Markets](#blue-carbon-markets)
3. [Coastal Management](#coastal-management)
4. [Marine Conservation](#marine-conservation)
5. [Fisheries Support](#fisheries-support)
6. [Satellite Integration](#satellite-integration)
7. [Community Systems](#community-systems)

---

## Overview

### 1.1 Purpose

The WIA Mangrove Restoration Integration Standard enables seamless interoperability between mangrove monitoring systems and external platforms including blue carbon markets, coastal management systems, marine conservation databases, and community applications.

### 1.2 Integration Architecture

```
┌─────────────────────────────────────┐
│   WIA Mangrove Restoration System   │
└──────────────┬──────────────────────┘
               │
    ┌──────────┼──────────┐
    │          │          │
┌───▼───┐  ┌──▼───┐  ┌──▼────┐
│ Blue  │  │Coast │  │Marine │
│Carbon │  │Mgmt  │  │ Cons  │
└───────┘  └──────┘  └───────┘
```

---

## Blue Carbon Markets

### 2.1 Verra VCS Integration

**Carbon Credit Issuance:**

```json
{
  "project": {
    "id": "VCS-2025-MANG-001",
    "title": "Philippines Mangrove Restoration",
    "methodology": "VM0033",
    "siteId": "MANG-2025-000001"
  },
  "carbonData": {
    "baselineEmissions": {"value": 0, "unit": "tons_co2_year"},
    "projectSequestration": {"value": 125.5, "unit": "tons_co2_year"},
    "netReduction": {"value": 125.5, "unit": "tons_co2_year"},
    "creditsIssued": 125,
    "vintage": 2025
  },
  "verification": {
    "verifier": "SCS Global Services",
    "date": "2025-01-15",
    "status": "verified"
  }
}
```

**API Integration:**

```javascript
// Submit carbon data to Verra Registry
const verraAPI = new VerraAPI({
  apiKey: process.env.VERRA_API_KEY,
  project: 'VCS-2025-MANG-001'
});

const carbonReport = await verraAPI.submitMonitoringReport({
  period: { start: '2024-01-01', end: '2024-12-31' },
  sequestration: { value: 125.5, unit: 'tons_co2_year' },
  methodology: 'VM0033',
  verificationDocuments: [
    's3://verra/reports/2024-annual.pdf'
  ]
});
```

### 2.2 Gold Standard Integration

```json
{
  "project": {
    "gsId": "GS-12345",
    "siteId": "MANG-2025-000001",
    "sdgImpacts": [
      {"sdg": 13, "impact": "Climate Action - Carbon Sequestration"},
      {"sdg": 14, "impact": "Life Below Water - Marine Habitat"},
      {"sdg": 15, "impact": "Life on Land - Coastal Ecosystems"}
    ]
  },
  "carbonCredits": {
    "issued": 125,
    "retired": 0,
    "available": 125,
    "price": {"amount": 20, "currency": "USD"}
  }
}
```

---

## Coastal Management

### 3.1 Coastal Zone Management Integration

**Erosion Monitoring:**

```json
{
  "siteId": "MANG-2025-000001",
  "coastalProtection": {
    "shorelineChange": {
      "baseline": "2020-01-01",
      "current": "2025-01-15",
      "changeRate": {"value": -0.5, "unit": "m_per_year"},
      "trend": "accretion",
      "mangroveEffect": {
        "waveAttenuation": {"value": 70, "unit": "percent"},
        "sedimentTrapping": {"value": 150, "unit": "kg_per_m2_year"}
      }
    },
    "floodProtection": {
      "returnPeriod": "10-year",
      "floodDepthReduction": {"value": 60, "unit": "percent"},
      "benefitedPopulation": 5000
    }
  }
}
```

**GIS Integration:**

```python
import geopandas as gpd
from wia_mangrove import MangroveAPI

# Load mangrove coverage data
api = MangroveAPI(api_key=os.environ['WIA_API_KEY'])
coverage = api.get_site_coverage('MANG-2025-000001')

# Create GIS layer
gdf = gpd.GeoDataFrame(
    coverage,
    geometry=gpd.points_from_xy(
        coverage['longitude'],
        coverage['latitude']
    ),
    crs='EPSG:4326'
)

# Export to Shapefile for coastal management system
gdf.to_file('mangrove_coverage.shp')
```

### 3.2 Storm Surge Modeling

```json
{
  "siteId": "MANG-2025-000001",
  "stormScenario": {
    "category": "Category 3 Typhoon",
    "windSpeed": {"value": 185, "unit": "km_h"},
    "surgeHeight": {"value": 3.5, "unit": "m"}
  },
  "mangroveProtection": {
    "surgeDamping": {"value": 40, "unit": "percent"},
    "waveHeightReduction": {"value": 1.4, "unit": "m"},
    "infrastructureProtected": [
      {"type": "residential", "units": 250},
      {"type": "commercial", "units": 15}
    ],
    "economicBenefit": {"amount": 2500000, "currency": "USD"}
  }
}
```

---

## Marine Conservation

### 3.1 OBIS Integration

**Ocean Biodiversity Information System:**

```json
{
  "siteId": "MANG-2025-000001",
  "biodiversity": {
    "marineSpecies": [
      {
        "scientificName": "Lutjanus argentimaculatus",
        "commonName": "Mangrove Red Snapper",
        "aphiaId": 218450,
        "abundance": "common",
        "lifeStage": "juvenile",
        "habitatUse": "nursery"
      },
      {
        "scientificName": "Scylla serrata",
        "commonName": "Mud Crab",
        "aphiaId": 107423,
        "abundance": "abundant",
        "commercialValue": "high"
      }
    ],
    "samplingDate": "2025-01-15",
    "samplingMethod": "visual survey",
    "effort": "2 person-hours"
  }
}
```

**API Integration:**

```python
from pyobis import occurrences

# Upload mangrove biodiversity data to OBIS
obis_data = {
    'scientificName': 'Lutjanus argentimaculatus',
    'decimalLatitude': 10.4806,
    'decimalLongitude': 123.3050,
    'eventDate': '2025-01-15',
    'habitat': 'mangrove',
    'basisOfRecord': 'HumanObservation'
}

occurrences.upload(obis_data)
```

### 3.2 Marine Protected Areas (MPA)

```json
{
  "mpaId": "MPA-PHIL-001",
  "siteId": "MANG-2025-000001",
  "designation": {
    "type": "Marine Sanctuary",
    "status": "established",
    "establishedDate": "2020-01-01"
  },
  "mangroveRole": {
    "connectivity": "juvenile fish migration corridor",
    "nurseryFunction": {
      "speciesCount": 25,
      "abundanceIndex": 88,
      "recruitmentRate": "high"
    },
    "carbonStorage": {
      "totalStock": {"value": 16867, "unit": "tons_c"},
      "contributionToMPA": "35%"
    }
  }
}
```

---

## Fisheries Support

### 4.1 Stock Assessment Integration

```json
{
  "siteId": "MANG-2025-000001",
  "fisheries": {
    "nurseryHabitat": {
      "juvenileDensity": {"value": 150, "unit": "fish_per_m2"},
      "speciesDiversity": 25,
      "growthRate": {
        "species": "Lutjanus argentimaculatus",
        "rate": {"value": 2.5, "unit": "cm_per_month"}
      }
    },
    "commercialValue": {
      "annualCatch": {"value": 5000, "unit": "kg"},
      "marketValue": {"amount": 75000, "currency": "USD"},
      "dependentFishers": 45
    },
    "sustainability": {
      "stockStatus": "healthy",
      "harvestRecommendation": "sustainable current levels",
      "closedSeasons": ["June-August"]
    }
  }
}
```

### 4.2 Community-Based Management

```json
{
  "siteId": "MANG-2025-000001",
  "community": {
    "name": "Barangay Manglar",
    "population": 1200,
    "households": 250,
    "fisherFamilies": 45,
    "managementModel": "Community-Based Coastal Resource Management",
    "activities": [
      {
        "type": "mangrove_planting",
        "participants": 80,
        "frequency": "quarterly",
        "treesPlanted": 5000
      },
      {
        "type": "fish_sanctuary_patrol",
        "participants": 12,
        "frequency": "weekly"
      }
    ],
    "economicBenefits": {
      "fisheriesIncrease": {"value": 25, "unit": "percent"},
      "carbonRevenue": {"amount": 15000, "currency": "USD", "period": "annual"},
      "ecotourism": {"amount": 8000, "currency": "USD", "period": "annual"}
    }
  }
}
```

---

## Satellite Integration

### 5.1 Sentinel-2 Analysis

**Mangrove Coverage Detection:**

```python
import ee
from wia_mangrove import SatelliteIntegration

# Initialize Earth Engine
ee.Initialize()

# Define mangrove site
site = ee.Geometry.Point([123.3050, 10.4806]).buffer(500)

# Get Sentinel-2 imagery
s2 = ee.ImageCollection('COPERNICUS/S2_SR') \
    .filterBounds(site) \
    .filterDate('2025-01-01', '2025-01-31') \
    .sort('CLOUDY_PIXEL_PERCENTAGE') \
    .first()

# Calculate NDVI for mangrove detection
ndvi = s2.normalizedDifference(['B8', 'B4'])

# Calculate mangrove area
mangrove_mask = ndvi.gt(0.3)
area = mangrove_mask.multiply(ee.Image.pixelArea()).reduceRegion(
    reducer=ee.Reducer.sum(),
    geometry=site,
    scale=10
)

print(f"Mangrove coverage: {area.getInfo()['nd']/10000} hectares")
```

### 5.2 Planet Labs Integration

```json
{
  "siteId": "MANG-2025-000001",
  "satelliteMonitoring": {
    "provider": "Planet Labs",
    "constellation": "PlanetScope",
    "resolution": {"value": 3, "unit": "m"},
    "frequency": "daily",
    "analysis": {
      "coverageChange": {
        "baseline": {"date": "2024-01-01", "area": 22.5},
        "current": {"date": "2025-01-15", "area": 25.5},
        "change": {"value": 3.0, "unit": "hectares", "percent": 13.3}
      },
      "healthIndex": {
        "ndvi": 0.78,
        "status": "healthy",
        "trend": "stable"
      }
    }
  }
}
```

---

## Community Systems

### 6.1 Mobile App Integration

**Field Data Collection:**

```json
{
  "app": "WIA Mangrove Field",
  "version": "1.0.0",
  "observation": {
    "observerId": "user-123",
    "timestamp": "2025-01-15T10:30:00Z",
    "location": {"lat": 10.4806, "lon": 123.3050},
    "siteId": "MANG-2025-000001",
    "observations": [
      {
        "type": "species_count",
        "species": "Rhizophora apiculata",
        "count": 150,
        "photo": "photo-uuid-123.jpg"
      },
      {
        "type": "water_quality",
        "salinity": {"value": 15.5, "unit": "ppt", "method": "refractometer"}
      },
      {
        "type": "wildlife",
        "species": "Egretta sacra",
        "count": 3,
        "behavior": "foraging"
      }
    ],
    "sync": {
      "status": "pending",
      "queuePosition": 1
    }
  }
}
```

### 6.2 Payment Systems

**Carbon Credit Revenue Distribution:**

```json
{
  "siteId": "MANG-2025-000001",
  "carbonSales": {
    "period": "2024-Q4",
    "credits": 125,
    "revenue": {"amount": 50000, "currency": "USD"}
  },
  "distribution": {
    "community": {
      "amount": 30000,
      "currency": "USD",
      "beneficiaries": 250,
      "paymentMethod": "mobile_wallet",
      "recipients": [
        {"id": "user-001", "share": 120, "amount": 120},
        {"id": "user-002", "share": 120, "amount": 120}
      ]
    },
    "maintenance": {"amount": 10000, "currency": "USD"},
    "monitoring": {"amount": 5000, "currency": "USD"},
    "reserve": {"amount": 5000, "currency": "USD"}
  }
}
```

---

## Integration Examples

### 7.1 Complete Workflow

```javascript
// 1. Collect field data
const fieldData = await mobileApp.collectData({
  siteId: 'MANG-2025-000001',
  observations: [...]
});

// 2. Update WIA system
await wiaAPI.updateSite('MANG-2025-000001', fieldData);

// 3. Calculate carbon sequestration
const carbon = await wiaAPI.calculateCarbon('MANG-2025-000001');

// 4. Submit to Verra
const verraReport = await verraAPI.submitReport({
  projectId: 'VCS-2025-MANG-001',
  carbonData: carbon
});

// 5. Generate credits
const credits = await verraAPI.issueCredits(verraReport.id);

// 6. Distribute revenue
await paymentSystem.distributeRevenue({
  amount: credits.value,
  beneficiaries: community.members
});
```

---

<div align="center">

**WIA Mangrove Restoration Integration v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
