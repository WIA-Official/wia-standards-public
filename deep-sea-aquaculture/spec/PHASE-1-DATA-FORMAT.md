# WIA-AGRI-023: Deep Sea Aquaculture Standard
## Phase 1: Data Format Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-023

---

## 1. Overview

This specification defines the standardized data formats for deep-sea aquaculture operations, including environmental monitoring, fish health metrics, feeding systems, and supply chain traceability.

### 1.1 Scope

- Real-time ocean environmental data
- Fish biomass and health metrics
- Automated feeding system data
- Equipment status and maintenance
- Supply chain and traceability records

### 1.2 Design Principles

- **Real-time Capability**: Support for streaming sensor data
- **Scalability**: Handle data from multiple farms and thousands of sensors
- **Interoperability**: JSON-based formats for easy integration
- **Traceability**: Complete audit trail from hatchery to market
- **Sustainability**: Track environmental impact metrics

---

## 2. Core Data Structures

### 2.1 Farm Identity Data

```json
{
  "farmId": "DSA-FARM-{COUNTRY}-{NUMBER}",
  "farmName": "Pacific Deep Aquaculture Farm",
  "operator": {
    "organizationId": "ORG-123456",
    "name": "Oceanic Farms Inc.",
    "license": "AQUA-LIC-2025-001",
    "country": "USA"
  },
  "location": {
    "gps": {
      "latitude": 36.5,
      "longitude": -127.2
    },
    "region": "Pacific Northwest",
    "waterBody": "Pacific Ocean",
    "nearestPort": "Newport, OR"
  },
  "infrastructure": {
    "cageType": "submersible",
    "totalCages": 10,
    "depth": {
      "min": 30,
      "max": 60,
      "unit": "meters"
    },
    "totalVolume": 50000,
    "volumeUnit": "m³"
  }
}
```

### 2.2 Environmental Sensor Data

```json
{
  "farmId": "DSA-FARM-USA-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "sensorId": "ENV-SENSOR-001",
  "location": {
    "cageId": "CAGE-01",
    "depth": 45,
    "depthUnit": "meters"
  },
  "measurements": {
    "waterTemperature": {
      "value": 18.5,
      "unit": "celsius",
      "status": "normal",
      "range": {
        "min": 10,
        "max": 25,
        "optimal": {"min": 15, "max": 20}
      }
    },
    "salinity": {
      "value": 35.2,
      "unit": "ppt",
      "status": "normal",
      "range": {
        "min": 30,
        "max": 40,
        "optimal": {"min": 33, "max": 37}
      }
    },
    "dissolvedOxygen": {
      "value": 7.8,
      "unit": "mg/L",
      "status": "normal",
      "range": {
        "min": 5,
        "max": 12,
        "critical": 4
      }
    },
    "pH": {
      "value": 8.1,
      "unit": "",
      "status": "normal",
      "range": {
        "min": 7.5,
        "max": 8.5,
        "optimal": {"min": 7.8, "max": 8.2}
      }
    },
    "currentSpeed": {
      "value": 1.2,
      "unit": "m/s",
      "direction": 135,
      "directionUnit": "degrees"
    },
    "turbidity": {
      "value": 2.5,
      "unit": "NTU",
      "status": "clear"
    },
    "ammonia": {
      "value": 0.05,
      "unit": "mg/L",
      "status": "safe",
      "threshold": 0.1
    },
    "nitrite": {
      "value": 0.02,
      "unit": "mg/L",
      "status": "safe",
      "threshold": 0.05
    }
  },
  "metadata": {
    "sensorModel": "AquaSense Pro 3000",
    "calibrationDate": "2025-12-01",
    "batteryLevel": 87,
    "signalStrength": -72
  }
}
```

### 2.3 Fish Population and Health Data

```json
{
  "farmId": "DSA-FARM-USA-001",
  "cageId": "CAGE-01",
  "timestamp": "2025-12-26T10:30:00Z",
  "species": {
    "scientificName": "Salmo salar",
    "commonName": "Atlantic Salmon",
    "strain": "Faroe Islands"
  },
  "population": {
    "totalCount": 5000,
    "estimatedBiomass": 12500,
    "biomassUnit": "kg",
    "averageWeight": 2.5,
    "weightUnit": "kg",
    "stocking": {
      "date": "2025-01-15",
      "initialCount": 5100,
      "initialWeight": 0.5
    }
  },
  "health": {
    "mortalityRate": {
      "daily": 0.02,
      "weekly": 0.1,
      "cumulative": 1.96,
      "unit": "percent"
    },
    "growthRate": {
      "current": 3.2,
      "expected": 3.0,
      "unit": "percent/week"
    },
    "feedConversionRatio": {
      "current": 1.15,
      "target": 1.2
    },
    "diseases": [],
    "treatments": [],
    "vaccinations": [
      {
        "type": "ISA",
        "date": "2025-02-01",
        "nextDue": "2026-02-01"
      }
    ],
    "healthScore": 94,
    "veterinaryInspections": [
      {
        "date": "2025-12-20",
        "inspector": "Dr. Jane Smith",
        "findings": "Healthy population",
        "nextInspection": "2026-01-20"
      }
    ]
  },
  "behavior": {
    "swimmingPattern": "normal",
    "feedingResponse": "active",
    "schoolingBehavior": "cohesive",
    "surfaceActivity": "minimal"
  }
}
```

### 2.4 Feeding System Data

```json
{
  "farmId": "DSA-FARM-USA-001",
  "cageId": "CAGE-01",
  "timestamp": "2025-12-26T10:30:00Z",
  "feedingEvent": {
    "eventId": "FEED-20251226-001",
    "startTime": "2025-12-26T10:00:00Z",
    "endTime": "2025-12-26T10:15:00Z",
    "duration": 15,
    "durationUnit": "minutes"
  },
  "feed": {
    "type": "pellets",
    "size": 9,
    "sizeUnit": "mm",
    "brand": "AquaFeed Premium",
    "batchNumber": "BATCH-2025-345",
    "composition": {
      "protein": 45,
      "fat": 22,
      "carbohydrates": 18,
      "ash": 8,
      "moisture": 7,
      "unit": "percent"
    },
    "dispensed": {
      "amount": 62.5,
      "unit": "kg"
    }
  },
  "feedingMetrics": {
    "dailyFeedingRate": 2.5,
    "dailyFeedingRateUnit": "percent of biomass",
    "feedingFrequency": 4,
    "feedingFrequencyUnit": "times/day",
    "uneatenFeed": 1.2,
    "uneatenFeedUnit": "percent",
    "feedingEfficiency": 98.8
  },
  "automation": {
    "mode": "automatic",
    "algorithm": "biomass-adjusted",
    "aiOptimized": true,
    "cameraMonitoring": true,
    "adjustmentFactors": {
      "waterTemp": 0.95,
      "fishActivity": 1.02,
      "weather": 1.0
    }
  }
}
```

### 2.5 Equipment Status Data

```json
{
  "farmId": "DSA-FARM-USA-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "equipment": [
    {
      "equipmentId": "CAGE-01",
      "type": "submersible-cage",
      "status": "operational",
      "depth": 45,
      "integrity": {
        "netCondition": "excellent",
        "lastInspection": "2025-12-15",
        "nextInspection": "2026-01-15"
      },
      "biofouling": {
        "level": "low",
        "lastCleaning": "2025-12-10",
        "nextCleaning": "2026-01-10"
      }
    },
    {
      "equipmentId": "FEEDER-01",
      "type": "automatic-feeder",
      "status": "operational",
      "location": "CAGE-01",
      "metrics": {
        "hopperLevel": 75,
        "hopperCapacity": 200,
        "unit": "kg",
        "dispenseRate": 4.2,
        "dispenseRateUnit": "kg/min"
      },
      "maintenance": {
        "lastService": "2025-12-01",
        "nextService": "2026-03-01",
        "hoursOperation": 1250
      }
    },
    {
      "equipmentId": "ROV-01",
      "type": "underwater-rov",
      "status": "charging",
      "batteryLevel": 45,
      "lastMission": "2025-12-26T08:00:00Z",
      "capabilities": ["inspection", "cleaning", "feeding", "monitoring"]
    }
  ]
}
```

### 2.6 Environmental Impact Data

```json
{
  "farmId": "DSA-FARM-USA-001",
  "reportPeriod": {
    "start": "2025-12-01",
    "end": "2025-12-26"
  },
  "wasteOutput": {
    "solidWaste": {
      "feces": 450,
      "uneatenFeed": 15,
      "unit": "kg/day"
    },
    "dissolvedWaste": {
      "ammonia": 12,
      "nitrate": 8,
      "phosphate": 3,
      "unit": "kg/day"
    }
  },
  "carbonFootprint": {
    "feedProduction": 2500,
    "transportation": 450,
    "operations": 180,
    "total": 3130,
    "unit": "kg CO2e/month"
  },
  "ecosystemImpact": {
    "benthicImpact": "minimal",
    "waterQualityImpact": "acceptable",
    "wildFishInteraction": "low",
    "marineLifeObservations": [
      {"species": "Sea Lions", "frequency": "occasional"},
      {"species": "Dolphins", "frequency": "rare"}
    ]
  },
  "sustainability": {
    "certifications": ["ASC", "BAP"],
    "complianceScore": 92,
    "improvementAreas": [
      "Reduce feed waste by 0.5%",
      "Increase renewable energy usage"
    ]
  }
}
```

### 2.7 Harvest and Traceability Data

```json
{
  "harvestId": "HARVEST-2025-001",
  "farmId": "DSA-FARM-USA-001",
  "cageId": "CAGE-01",
  "harvestDate": "2025-12-26",
  "batch": {
    "batchId": "BATCH-2025-001",
    "species": "Atlantic Salmon",
    "quantity": {
      "count": 500,
      "totalWeight": 1250,
      "averageWeight": 2.5,
      "unit": "kg"
    }
  },
  "quality": {
    "grade": "Premium",
    "freshness": "excellent",
    "fatContent": 12.5,
    "colorScore": 28
  },
  "traceability": {
    "hatchery": {
      "id": "HATCH-NO-001",
      "location": "Norway",
      "eggSource": "Faroe Islands"
    },
    "growthHistory": {
      "stockingDate": "2025-01-15",
      "growthDuration": 345,
      "growthDurationUnit": "days",
      "farms": ["DSA-FARM-USA-001"]
    },
    "feedHistory": {
      "totalFeedConsumed": 1437.5,
      "feedBrands": ["AquaFeed Premium"],
      "feedConversionRatio": 1.15
    },
    "healthHistory": {
      "antibioticUse": "none",
      "vaccinations": ["ISA"],
      "treatments": []
    },
    "certifications": [
      {
        "type": "ASC",
        "certificateId": "ASC-C-12345",
        "validUntil": "2026-12-31"
      }
    ]
  },
  "processing": {
    "processor": "Pacific Seafood Processing",
    "processingDate": "2025-12-26",
    "products": [
      {
        "productId": "PROD-001",
        "type": "whole-fish",
        "weight": 600,
        "packaging": "ice",
        "destination": "Portland, OR"
      },
      {
        "productId": "PROD-002",
        "type": "fillets",
        "weight": 650,
        "packaging": "vacuum",
        "destination": "Seattle, WA"
      }
    ]
  },
  "blockchain": {
    "transactionHash": "0x7f9fade1c0d57a7af66ab4ead79fade1c0d57a7af66ab4ead7c2c2eb7b11a91385",
    "blockNumber": 12345678,
    "network": "ethereum-mainnet"
  }
}
```

---

## 3. Data Validation Rules

### 3.1 Required Fields

All data structures must include:
- `farmId`: Unique farm identifier
- `timestamp`: ISO 8601 formatted timestamp
- `unit`: Measurement units for all numeric values

### 3.2 Value Ranges

Environmental parameters must fall within acceptable ranges:
- Water Temperature: -2°C to 35°C
- Salinity: 0 to 50 ppt
- Dissolved Oxygen: 0 to 15 mg/L
- pH: 0 to 14

### 3.3 Data Quality Indicators

Each sensor reading should include:
- Status flag (normal, warning, critical)
- Acceptable range
- Timestamp of last calibration

---

## 4. Data Encoding

### 4.1 Character Encoding
- UTF-8 for all text data

### 4.2 Date/Time Format
- ISO 8601: `YYYY-MM-DDTHH:mm:ssZ`
- UTC timezone preferred

### 4.3 Numeric Precision
- Temperature: 1 decimal place
- Weight/Mass: 2 decimal places
- Coordinates: 6 decimal places

---

## 5. Compression and Optimization

### 5.1 For Real-time Streaming
- Use Protocol Buffers or MessagePack for sensor data
- Maximum message size: 64KB
- Compression: gzip or snappy

### 5.2 For Historical Storage
- Parquet format for analytics
- Daily partitioning by farmId and date
- Retention: 7 years minimum

---

## 6. Security and Privacy

### 6.1 Data Classification
- **Public**: Certifications, sustainability scores
- **Internal**: Operational metrics, equipment status
- **Confidential**: Financial data, proprietary algorithms

### 6.2 Anonymization
- GPS coordinates rounded to 0.01 degrees for public data
- Farm identity masked in research datasets

---

## 7. Compliance

This data format specification complies with:
- **ISO 8601**: Date and time formats
- **GS1**: Product traceability standards
- **ASC/MSC**: Certification data requirements
- **GDPR**: Data privacy and protection

---

## 8. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-26 | Initial specification |

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
