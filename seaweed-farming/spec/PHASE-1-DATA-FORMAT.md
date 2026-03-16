# WIA-AGRI-024: Seaweed Farming Standard
## Phase 1 - Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines standardized data formats for seaweed farming systems, including cultivation monitoring, water quality tracking, growth optimization, harvest automation, and carbon sequestration measurement.

### 1.1 Design Principles

- **Marine-Optimized**: Designed for saltwater and coastal aquaculture environments
- **Species-Diverse**: Supports kelp, nori, wakame, dulse, and other macroalgae species
- **Real-time Monitoring**: Sub-minute latency for critical environmental parameters
- **Sustainability-First**: Carbon capture, nutrient recycling, and ecosystem benefits tracking
- **Traceability**: Full supply chain tracking from spore to consumer

---

## 2. Core Data Structures

### 2.1 Seaweed Farm Entity

```json
{
  "farmId": "string (UUID)",
  "farmName": "string",
  "farmType": "string (enum: longline, raft, net_cage, tank, integrated)",
  "location": {
    "latitude": "number (decimal degrees)",
    "longitude": "number (decimal degrees)",
    "waterBody": "string (ocean, sea, bay, estuary)",
    "depth": "number (meters, cultivation depth)",
    "coastalRegion": "string",
    "country": "string (ISO 3166-1 alpha-2)",
    "timezone": "string (IANA timezone)"
  },
  "capacity": {
    "totalArea": "number (hectares)",
    "cultivationLines": "number",
    "ropeLength": "number (meters)",
    "maxBiomass": "number (metric tons)"
  },
  "certifications": ["ASC", "Organic", "WIA-AGRI-024", "Blue Carbon"],
  "establishedDate": "string (ISO 8601)",
  "operator": {
    "name": "string",
    "contact": "string",
    "did": "string (W3C DID)",
    "license": "string (aquaculture license number)"
  }
}
```

### 2.2 Seaweed Species Profile

```json
{
  "speciesId": "string (UUID)",
  "scientificName": "string (e.g., Saccharina latissima)",
  "commonName": "string (e.g., Sugar Kelp)",
  "category": "string (enum: brown_algae, red_algae, green_algae)",
  "optimalConditions": {
    "temperatureRange": {
      "min": "number (Celsius)",
      "max": "number (Celsius)",
      "optimal": "number (Celsius)"
    },
    "salinityRange": {
      "min": "number (PSU)",
      "max": "number (PSU)",
      "optimal": "number (PSU)"
    },
    "lightRequirement": {
      "min": "number (μmol/m²/s)",
      "max": "number (μmol/m²/s)",
      "optimal": "number (μmol/m²/s)"
    },
    "nutrientRequirements": {
      "nitrate": {
        "min": "number (mg/L)",
        "optimal": "number (mg/L)"
      },
      "phosphate": {
        "min": "number (mg/L)",
        "optimal": "number (mg/L)"
      },
      "iron": {
        "min": "number (μg/L)",
        "optimal": "number (μg/L)"
      }
    }
  },
  "growthCharacteristics": {
    "averageGrowthRate": "number (cm/day)",
    "harvestCycle": "number (days)",
    "maxLength": "number (meters)",
    "carbonSequestrationRate": "number (kg CO2/kg biomass)"
  },
  "uses": ["food", "feed", "biofuel", "cosmetics", "fertilizer", "carbon_credits"]
}
```

### 2.3 Water Quality Sensor Data

```json
{
  "sensorId": "string (UUID)",
  "farmId": "string (UUID)",
  "zoneId": "string (cultivation zone identifier)",
  "timestamp": "string (ISO 8601 with timezone)",
  "sensorType": "string (enum: multiparameter, ctd, nutrient_probe)",
  "depth": "number (meters, sensor depth)",
  "measurements": {
    "temperature": {
      "value": "number (Celsius)",
      "unit": "°C",
      "status": "string (optimal, suboptimal, critical)"
    },
    "salinity": {
      "value": "number (PSU)",
      "unit": "PSU",
      "status": "string"
    },
    "dissolvedOxygen": {
      "value": "number (mg/L)",
      "unit": "mg/L",
      "saturation": "number (percentage)",
      "status": "string"
    },
    "pH": {
      "value": "number (0-14)",
      "status": "string"
    },
    "nitrate": {
      "value": "number (mg/L)",
      "unit": "mg/L",
      "status": "string"
    },
    "phosphate": {
      "value": "number (mg/L)",
      "unit": "mg/L",
      "status": "string"
    },
    "iron": {
      "value": "number (μg/L)",
      "unit": "μg/L",
      "status": "string"
    },
    "turbidity": {
      "value": "number (NTU)",
      "unit": "NTU",
      "status": "string"
    },
    "currentSpeed": {
      "value": "number (m/s)",
      "direction": "number (degrees)",
      "status": "string"
    },
    "waveHeight": {
      "value": "number (meters)",
      "period": "number (seconds)",
      "status": "string"
    }
  },
  "location": {
    "latitude": "number",
    "longitude": "number",
    "depth": "number (meters)"
  }
}
```

### 2.4 Growth Monitoring Data

```json
{
  "measurementId": "string (UUID)",
  "farmId": "string (UUID)",
  "speciesId": "string (UUID)",
  "lineId": "string (cultivation line identifier)",
  "timestamp": "string (ISO 8601)",
  "measurementMethod": "string (enum: manual, camera, sonar, drone)",
  "growthMetrics": {
    "biomass": {
      "density": "number (kg/m²)",
      "totalWeight": "number (kg)",
      "unit": "kg"
    },
    "dimensions": {
      "averageLength": "number (cm)",
      "averageWidth": "number (cm)",
      "thicknessIndex": "number (mm)",
      "unit": "cm"
    },
    "growthRate": {
      "daily": "number (cm/day)",
      "weekly": "number (cm/week)",
      "monthly": "number (cm/month)"
    },
    "healthScore": {
      "overall": "number (0-100)",
      "color": "string (vibrant, normal, faded)",
      "texture": "string (firm, normal, soft)",
      "diseasePresence": "boolean",
      "epiphyteLevel": "string (none, low, moderate, high)"
    },
    "maturityIndex": {
      "score": "number (0-100)",
      "harvestReadiness": "boolean",
      "estimatedDaysToHarvest": "number (days)"
    }
  },
  "environmentalFactors": {
    "waterTemperature": "number (°C)",
    "nutrientAvailability": "string (high, medium, low)",
    "lightExposure": "number (hours/day)",
    "currentFlow": "number (m/s)"
  }
}
```

### 2.5 Harvest Data

```json
{
  "harvestId": "string (UUID)",
  "farmId": "string (UUID)",
  "speciesId": "string (UUID)",
  "harvestDate": "string (ISO 8601)",
  "harvestMethod": "string (enum: manual, mechanical, automated, hybrid)",
  "harvestZone": {
    "zoneId": "string",
    "lineIds": ["string (array of cultivation line IDs)"],
    "areaHarvested": "number (hectares)"
  },
  "yield": {
    "totalWetWeight": "number (kg)",
    "totalDryWeight": "number (kg)",
    "moistureContent": "number (percentage)",
    "yieldPerArea": "number (kg/hectare)",
    "qualityGrade": "string (premium, standard, feed-grade)"
  },
  "composition": {
    "protein": "number (percentage)",
    "carbohydrates": "number (percentage)",
    "fiber": "number (percentage)",
    "minerals": {
      "iodine": "number (mg/kg)",
      "calcium": "number (mg/kg)",
      "iron": "number (mg/kg)",
      "magnesium": "number (mg/kg)"
    },
    "vitamins": {
      "vitaminA": "number (IU/100g)",
      "vitaminC": "number (mg/100g)",
      "vitaminK": "number (μg/100g)"
    }
  },
  "destination": {
    "purpose": "string (enum: food, feed, biofuel, cosmetics, fertilizer)",
    "buyer": "string",
    "processingFacility": "string",
    "traceabilityCode": "string"
  },
  "harvestConditions": {
    "weatherConditions": "string",
    "seaState": "number (Beaufort scale)",
    "crewSize": "number",
    "durationHours": "number"
  }
}
```

### 2.6 Carbon Sequestration Data

```json
{
  "carbonReportId": "string (UUID)",
  "farmId": "string (UUID)",
  "reportingPeriod": {
    "startDate": "string (ISO 8601)",
    "endDate": "string (ISO 8601)",
    "duration": "number (days)"
  },
  "carbonMetrics": {
    "biomassCarbonContent": {
      "totalCarbon": "number (kg C)",
      "carbonPercentage": "number (% of dry weight)"
    },
    "co2Sequestered": {
      "totalCO2": "number (kg CO2)",
      "co2PerHectare": "number (kg CO2/hectare)",
      "co2PerDay": "number (kg CO2/day)"
    },
    "carbonCredits": {
      "totalCredits": "number (verified carbon units)",
      "creditPrice": "number (USD per credit)",
      "totalValue": "number (USD)",
      "verificationStatus": "string (pending, verified, issued)"
    },
    "ecosystemBenefits": {
      "nitrogenRemoval": "number (kg N)",
      "phosphorusRemoval": "number (kg P)",
      "oxygenProduction": "number (kg O2)",
      "habitatProvided": "number (m² for marine species)"
    }
  },
  "methodology": {
    "calculationStandard": "string (e.g., Verra VCS, Gold Standard)",
    "carbonFactor": "number (kg CO2/kg biomass)",
    "verifiedBy": "string (certification body)",
    "verificationDate": "string (ISO 8601)"
  }
}
```

---

## 3. Data Validation Rules

### 3.1 Required Fields

All entities MUST include:
- Unique identifier (UUID v4)
- Timestamp (ISO 8601 format with timezone)
- Farm ID reference
- Status field

### 3.2 Value Constraints

```typescript
interface ValidationRules {
  temperature: {
    min: -2,      // Freezing point of seawater
    max: 35,      // Tropical maximum
    unit: "°C"
  },
  salinity: {
    min: 15,      // Brackish minimum
    max: 40,      // Hypersaline maximum
    unit: "PSU"
  },
  pH: {
    min: 7.0,
    max: 9.0,
    optimal: 8.1
  },
  biomass: {
    min: 0,
    max: 50,      // kg/m² maximum realistic density
    unit: "kg/m²"
  }
}
```

### 3.3 Temporal Consistency

- Growth measurements: Maximum once per hour
- Water quality: Minimum once per 15 minutes
- Harvest data: Must align with growth cycle (30-180 days)
- Carbon reports: Quarterly or annual reporting

---

## 4. Data Encoding

### 4.1 Supported Formats

1. **JSON** (Primary): Human-readable, web-friendly
2. **Protocol Buffers**: Efficient binary serialization
3. **MessagePack**: Compact binary format for constrained networks
4. **CSV/TSV**: Bulk data export and analysis

### 4.2 Character Encoding

- UTF-8 for all text fields
- Support for multilingual species names (Latin, Chinese, Japanese, Korean)

### 4.3 Compression

- GZIP compression for HTTP payloads
- Brotli for static data files
- LZ4 for real-time sensor streams

---

## 5. Units and Standards

### 5.1 Measurement Units

| Parameter | Unit | Standard |
|-----------|------|----------|
| Temperature | °C | SI |
| Salinity | PSU (Practical Salinity Unit) | UNESCO |
| Distance | meters, kilometers | SI |
| Weight | kilograms, metric tons | SI |
| Area | hectares | SI |
| Light | μmol/m²/s (PAR) | Photosynthetically Active Radiation |
| Nutrients | mg/L, μg/L | ISO 17025 |
| Carbon | kg CO2-equivalent | IPCC AR6 |

### 5.2 Geographic Coordinates

- WGS84 datum
- Decimal degrees format
- Precision: 6 decimal places (~0.1m accuracy)

### 5.3 Time Zones

- All timestamps in ISO 8601 format
- UTC offset included
- Support for DST transitions

---

## 6. Metadata

### 6.1 Data Provenance

```json
{
  "provenance": {
    "source": "string (sensor ID, operator ID, system ID)",
    "generatedBy": "string (hardware/software version)",
    "generatedAt": "string (ISO 8601)",
    "accuracy": "number (confidence level 0-1)",
    "calibrationDate": "string (ISO 8601)",
    "processingPipeline": ["raw", "filtered", "validated", "aggregated"]
  }
}
```

### 6.2 Quality Indicators

```json
{
  "dataQuality": {
    "completeness": "number (percentage 0-100)",
    "accuracy": "number (percentage 0-100)",
    "timeliness": "number (minutes delay)",
    "consistency": "boolean (passed validation)",
    "flags": ["outlier_detected", "sensor_drift", "manual_override"]
  }
}
```

---

## 7. Security and Privacy

### 7.1 Data Classification

- **Public**: Species information, general farm location (country/region)
- **Internal**: Detailed coordinates, production volumes
- **Confidential**: Financial data, carbon credit pricing
- **Regulated**: Personal identifiable information (PII) of operators

### 7.2 Encryption

- At-rest: AES-256-GCM
- In-transit: TLS 1.3
- Field-level encryption for sensitive data (coordinates, financials)

---

## 8. Versioning

### 8.1 Schema Versioning

```json
{
  "schemaVersion": "1.0.0",
  "compatibilityMode": "backward_compatible",
  "deprecatedFields": [],
  "newFields": []
}
```

### 8.2 Backward Compatibility

- All schema changes MUST maintain backward compatibility for 2 major versions
- Deprecated fields retained for 12 months minimum
- New optional fields do not break existing implementations

---

## 9. Interoperability

### 9.1 Related Standards

- WIA-AGRI-012: Smart Aquaculture Standard
- WIA-AGRI-020: Food Traceability Standard
- ISO 19156: Observations and Measurements
- OGC SensorThings API

### 9.2 Export Formats

Support for:
- GeoJSON (spatial data)
- NetCDF (oceanographic time series)
- CSV (bulk data analysis)
- Shapefile (GIS integration)

---

## 10. Implementation Notes

### 10.1 Minimum Viable Implementation

A minimal compliant system MUST support:
1. Farm entity registration
2. Water quality measurements (temperature, salinity, nutrients)
3. Growth monitoring (biomass, length)
4. Harvest records

### 10.2 Recommended Implementation

In addition to minimum requirements:
1. Real-time sensor data streaming
2. Carbon sequestration tracking
3. Automated alerts and notifications
4. Integration with weather/oceanographic services

### 10.3 Advanced Features

- Machine learning for growth prediction
- Computer vision for automated health assessment
- Blockchain for traceability
- Satellite remote sensing integration

---

## Appendix A: Sample Data

### A.1 Complete Farm Record

```json
{
  "farmId": "550e8400-e29b-41d4-a716-446655440000",
  "farmName": "Ocean Harvest Kelp Farm",
  "farmType": "longline",
  "location": {
    "latitude": 45.234567,
    "longitude": -124.056789,
    "waterBody": "Pacific Ocean",
    "depth": 15,
    "coastalRegion": "Oregon Coast",
    "country": "US",
    "timezone": "America/Los_Angeles"
  },
  "capacity": {
    "totalArea": 5.5,
    "cultivationLines": 120,
    "ropeLength": 24000,
    "maxBiomass": 165
  },
  "certifications": ["Organic", "WIA-AGRI-024", "Blue Carbon Verified"],
  "establishedDate": "2023-06-15T00:00:00Z",
  "operator": {
    "name": "Ocean Harvest LLC",
    "contact": "contact@oceanharvest.com",
    "did": "did:wia:ocean-harvest-001",
    "license": "AQ-OR-2023-0456"
  }
}
```

---

## Appendix B: Glossary

- **Epiphyte**: Organism growing on seaweed surface
- **Longline**: Horizontal rope cultivation system
- **Macroalgae**: Large seaweed species (vs microalgae)
- **PAR**: Photosynthetically Active Radiation
- **PSU**: Practical Salinity Unit
- **Sporophyte**: Mature seaweed plant stage

---

**© 2025 WIA Standards | 弘益人間 · Benefit All Humanity**
