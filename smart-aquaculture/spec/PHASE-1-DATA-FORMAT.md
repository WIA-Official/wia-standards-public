# WIA-AGRI-012: Smart Aquaculture Standard
## Phase 1 - Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines standardized data formats for smart aquaculture systems, including water quality monitoring, fish health tracking, feeding automation, and sustainable seafood production.

### 1.1 Design Principles

- **Marine-Optimized**: Designed for saltwater and freshwater aquaculture environments
- **Species-Agnostic**: Supports fish, shellfish, crustaceans, and aquatic plants
- **Real-time Monitoring**: Sub-minute latency for critical water quality parameters
- **Traceability**: Full supply chain tracking from hatchery to consumer
- **Sustainability**: Environmental impact and resource efficiency metrics

---

## 2. Core Data Structures

### 2.1 Aquaculture Farm Entity

```json
{
  "farmId": "string (UUID)",
  "farmName": "string",
  "farmType": "string (enum: marine, freshwater, brackish, recirculating)",
  "location": {
    "latitude": "number (decimal degrees)",
    "longitude": "number (decimal degrees)",
    "waterBody": "string (ocean, sea, lake, river, pond)",
    "depth": "number (meters)",
    "address": "string",
    "country": "string (ISO 3166-1 alpha-2)",
    "timezone": "string (IANA timezone)"
  },
  "capacity": {
    "totalVolume": "number (cubic meters)",
    "tankCount": "number",
    "cageCount": "number",
    "pondCount": "number",
    "maxBiomass": "number (kg)"
  },
  "certifications": ["ASC", "BAP", "Organic", "WIA-AGRI-012"],
  "establishedDate": "string (ISO 8601)",
  "operator": {
    "name": "string",
    "contact": "string",
    "did": "string (W3C DID)",
    "license": "string (fishery license number)"
  }
}
```

### 2.2 Water Quality Sensor Data

```json
{
  "sensorId": "string (UUID)",
  "farmId": "string (UUID)",
  "tankId": "string (tank/cage/pond identifier)",
  "timestamp": "string (ISO 8601 with timezone)",
  "sensorType": "string (enum: multiparameter, dissolved_oxygen, temperature, pH)",
  "depth": "number (meters, sensor depth)",
  "measurements": {
    "temperature": {
      "value": "number (Celsius)",
      "unit": "°C",
      "status": "string (normal, warning, critical)"
    },
    "salinity": {
      "value": "number (parts per thousand)",
      "unit": "ppt",
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
    "ammonia": {
      "nh3": "number (mg/L)",
      "nh4": "number (mg/L)",
      "totalAmmonia": "number (mg/L)",
      "status": "string"
    },
    "nitrite": {
      "value": "number (mg/L)",
      "unit": "mg/L",
      "status": "string"
    },
    "nitrate": {
      "value": "number (mg/L)",
      "unit": "mg/L",
      "status": "string"
    },
    "turbidity": {
      "value": "number (NTU)",
      "unit": "NTU",
      "visibility": "number (meters)"
    },
    "alkalinity": {
      "value": "number (mg/L as CaCO3)",
      "unit": "mg/L",
      "bufferCapacity": "string (low, medium, high)"
    },
    "chlorophyll": {
      "value": "number (μg/L)",
      "unit": "μg/L",
      "algaeBloomRisk": "string (low, medium, high)"
    }
  },
  "alerts": [
    {
      "parameter": "string",
      "severity": "string (info, warning, critical)",
      "message": "string",
      "threshold": "number",
      "currentValue": "number"
    }
  ]
}
```

### 2.3 Fish Health & Biomass Data

```json
{
  "batchId": "string (UUID)",
  "farmId": "string (UUID)",
  "tankId": "string",
  "timestamp": "string (ISO 8601)",
  "species": {
    "scientificName": "string (Paralichthys olivaceus)",
    "commonName": "string (Flatfish / 넙치)",
    "geneticLineage": "string (hatchery batch ID)"
  },
  "population": {
    "totalCount": "number (estimated)",
    "density": "number (kg/m³)",
    "stockingDate": "string (ISO 8601)",
    "daysInCulture": "number"
  },
  "biomass": {
    "total": "number (kg)",
    "averageWeight": "number (grams)",
    "weightDistribution": {
      "min": "number (grams)",
      "max": "number (grams)",
      "median": "number (grams)",
      "stdDev": "number"
    }
  },
  "growth": {
    "dailyGrowthRate": "number (g/day)",
    "specificGrowthRate": "number (% per day)",
    "projectedHarvestWeight": "number (grams)",
    "projectedHarvestDate": "string (ISO 8601)"
  },
  "health": {
    "mortalityRate": "number (percentage)",
    "diseaseIncidence": "number (percentage)",
    "parasiteLoad": "string (none, low, medium, high)",
    "abnormalBehavior": "number (percentage)",
    "feedingResponse": "string (excellent, good, fair, poor)",
    "bodyConditionFactor": "number (Fulton's K)",
    "stressLevel": "string (low, medium, high)"
  },
  "vaccinations": [
    {
      "disease": "string (vibriosis, furunculosis)",
      "date": "string (ISO 8601)",
      "vaccine": "string",
      "effectiveness": "number (percentage)"
    }
  ],
  "treatments": [
    {
      "condition": "string",
      "treatment": "string",
      "startDate": "string (ISO 8601)",
      "endDate": "string (ISO 8601)",
      "dosage": "string",
      "withdrawalPeriod": "number (days)"
    }
  ]
}
```

### 2.4 Feeding System Data

```json
{
  "feedingEventId": "string (UUID)",
  "farmId": "string (UUID)",
  "tankId": "string",
  "timestamp": "string (ISO 8601)",
  "feedingMethod": "string (auto_feeder, manual, demand_feeder)",
  "feedType": {
    "productId": "string",
    "manufacturer": "string",
    "pelletSize": "number (mm)",
    "proteinContent": "number (percentage)",
    "fatContent": "number (percentage)",
    "moisture": "number (percentage)",
    "energy": "number (kcal/kg)"
  },
  "feedAmount": {
    "planned": "number (kg)",
    "actual": "number (kg)",
    "waste": "number (kg, estimated)"
  },
  "feedingSchedule": {
    "frequency": "number (times per day)",
    "duration": "number (minutes)",
    "dailyFeedingRate": "number (% of biomass)"
  },
  "performance": {
    "feedConversionRatio": "number (FCR)",
    "economicFCR": "number (eFCR)",
    "feedingEfficiency": "number (percentage)",
    "appetite": "string (excellent, good, fair, poor)"
  },
  "costs": {
    "feedCostPerKg": "number (USD)",
    "totalFeedCost": "number (USD)",
    "costPerKgGrowth": "number (USD)"
  }
}
```

### 2.5 Environmental Monitoring

```json
{
  "monitoringId": "string (UUID)",
  "farmId": "string (UUID)",
  "timestamp": "string (ISO 8601)",
  "weather": {
    "airTemperature": "number (Celsius)",
    "humidity": "number (percentage)",
    "windSpeed": "number (m/s)",
    "windDirection": "number (degrees)",
    "rainfall": "number (mm)",
    "barometricPressure": "number (hPa)",
    "solarRadiation": "number (W/m²)",
    "uvIndex": "number"
  },
  "tides": {
    "currentLevel": "number (meters)",
    "nextHighTide": "string (ISO 8601)",
    "nextLowTide": "string (ISO 8601)",
    "tidalRange": "number (meters)"
  },
  "waterFlow": {
    "inflowRate": "number (L/min)",
    "outflowRate": "number (L/min)",
    "exchangeRate": "number (% per hour)",
    "currentSpeed": "number (m/s)"
  },
  "energyConsumption": {
    "aerators": "number (kWh)",
    "pumps": "number (kWh)",
    "feeders": "number (kWh)",
    "lighting": "number (kWh)",
    "total": "number (kWh)",
    "carbonFootprint": "number (kg CO2e)"
  }
}
```

### 2.6 Harvest & Traceability Data

```json
{
  "harvestId": "string (UUID)",
  "farmId": "string (UUID)",
  "batchId": "string (UUID)",
  "harvestDate": "string (ISO 8601)",
  "species": {
    "scientificName": "string",
    "commonName": "string",
    "marketName": "string"
  },
  "harvest": {
    "totalWeight": "number (kg)",
    "fishCount": "number",
    "averageWeight": "number (grams)",
    "sizeGrades": {
      "small": "number (kg)",
      "medium": "number (kg)",
      "large": "number (kg)",
      "extraLarge": "number (kg)"
    }
  },
  "quality": {
    "freshness": "string (A, B, C grade)",
    "appearance": "string (excellent, good, fair)",
    "colorScore": "number (1-10)",
    "textureScore": "number (1-10)",
    "parasiteCheck": "string (pass, fail)",
    "contaminants": {
      "heavyMetals": "boolean (detected)",
      "antibiotics": "boolean (detected)",
      "microplastics": "boolean (detected)"
    }
  },
  "cultivation": {
    "daysInCulture": "number",
    "cumulativeFCR": "number",
    "survivalRate": "number (percentage)",
    "waterSource": "string",
    "feedSource": "string",
    "antibioticUsed": "boolean",
    "organicCertified": "boolean"
  },
  "traceability": {
    "qrCode": "string (QR data)",
    "blockchainTxId": "string",
    "verifiableCredential": "object (W3C VC)",
    "destinationMarket": "string",
    "transportMethod": "string"
  },
  "economics": {
    "productionCost": "number (USD/kg)",
    "marketPrice": "number (USD/kg)",
    "revenue": "number (USD)",
    "profit": "number (USD)",
    "roi": "number (percentage)"
  }
}
```

---

## 3. Communication Protocols

### 3.1 Underwater Acoustic Telemetry

For submerged sensors in marine cages and ponds:

```json
{
  "protocol": "Acoustic Modem",
  "frequency": "18-36 kHz",
  "dataRate": "10-40 kbps",
  "range": "1-5 km",
  "modulation": "FSK / PSK",
  "errorCorrection": "Reed-Solomon",
  "powerConsumption": "1-5W transmit, <0.1W receive"
}
```

### 3.2 Surface Wireless (LoRa/RF)

For surface buoys and edge devices:

```json
{
  "protocol": "LoRaWAN",
  "frequency": "915 MHz (US) / 868 MHz (EU) / 920 MHz (KR)",
  "dataRate": "0.3-50 kbps",
  "range": "2-15 km",
  "powerConsumption": "<50mA transmit"
}
```

---

## 4. Data Quality & Validation

### 4.1 Sensor Calibration

```json
{
  "calibrationId": "string (UUID)",
  "sensorId": "string (UUID)",
  "calibrationDate": "string (ISO 8601)",
  "nextCalibrationDue": "string (ISO 8601)",
  "calibrationStandards": [
    {
      "parameter": "pH",
      "standard": "pH 7.00 buffer",
      "measuredValue": 7.02,
      "accuracy": "±0.02"
    }
  ],
  "calibrationStatus": "string (valid, expired, out_of_tolerance)"
}
```

### 4.2 Data Validation Rules

| Parameter | Min | Max | Unit | Alert Threshold |
|-----------|-----|-----|------|-----------------|
| Water Temperature | 5 | 35 | °C | <10 or >28 |
| Dissolved Oxygen | 3 | 15 | mg/L | <5.0 |
| pH | 6.0 | 9.5 | - | <6.5 or >8.5 |
| Salinity | 0 | 40 | ppt | Species-specific |
| Ammonia (Total) | 0 | 5 | mg/L | >0.5 |
| Nitrite | 0 | 2 | mg/L | >0.2 |

---

## 5. Korean Aquaculture Context

### 5.1 Common Species (한국 주요 양식 어종)

- **넙치 (Flatfish)**: *Paralichthys olivaceus*
- **광어 (Olive Flounder)**: *Paralichthys olivaceus* (same as 넙치)
- **농어 (Sea Bass)**: *Lateolabrax japonicus*
- **전복 (Abalone)**: *Haliotis discus*
- **굴 (Oyster)**: *Crassostrea gigas*
- **새우 (Shrimp)**: *Penaeus japonicus*
- **조피볼락 (Black Rockfish)**: *Sebastes schlegelii*
- **참돔 (Red Seabream)**: *Pagrus major*

### 5.2 Regulatory Compliance

```json
{
  "regulatoryBody": "국립수산과학원 (National Institute of Fisheries Science)",
  "reportingFrequency": "monthly",
  "requiredData": [
    "수질 데이터 (Water quality)",
    "생산량 (Production volume)",
    "폐사율 (Mortality rate)",
    "사료계수 (Feed conversion ratio)",
    "항생제 사용 (Antibiotic usage)"
  ],
  "foodSafetySystem": "수산물이력제 (Seafood Traceability System)",
  "certifications": ["HACCP", "유기농 수산물 인증"]
}
```

---

## 6. JSON Schema Examples

### 6.1 Complete Water Quality Reading

```json
{
  "sensorId": "aq-sensor-12345",
  "farmId": "farm-tongyeong-001",
  "tankId": "tank-A-03",
  "timestamp": "2025-01-15T14:30:00+09:00",
  "sensorType": "multiparameter",
  "depth": 3.5,
  "measurements": {
    "temperature": {
      "value": 18.5,
      "unit": "°C",
      "status": "normal"
    },
    "salinity": {
      "value": 32.5,
      "unit": "ppt",
      "status": "normal"
    },
    "dissolvedOxygen": {
      "value": 7.8,
      "unit": "mg/L",
      "saturation": 92,
      "status": "normal"
    },
    "pH": {
      "value": 8.1,
      "status": "normal"
    },
    "ammonia": {
      "nh3": 0.002,
      "nh4": 0.045,
      "totalAmmonia": 0.047,
      "status": "normal"
    },
    "nitrite": {
      "value": 0.08,
      "unit": "mg/L",
      "status": "normal"
    },
    "nitrate": {
      "value": 12.3,
      "unit": "mg/L",
      "status": "normal"
    },
    "turbidity": {
      "value": 8.2,
      "unit": "NTU",
      "visibility": 4.5
    }
  },
  "alerts": []
}
```

---

## 7. Compliance & Standards

### 7.1 International Standards

- **ISO 14001**: Environmental Management
- **ASC**: Aquaculture Stewardship Council
- **BAP**: Best Aquaculture Practices
- **GlobalG.A.P.**: Good Agricultural Practices for Aquaculture

### 7.2 Data Privacy

- **GDPR**: For EU export markets
- **Personal Information Protection Act (개인정보보호법)**: Korea
- **W3C Verifiable Credentials**: For traceability and certification

---

## 8. Implementation Notes

### 8.1 Sensor Deployment

- **Multiparameter sondes**: Every 500m³ of water volume
- **DO sensors**: Critical zones (high density areas)
- **Camera systems**: Every tank/cage for behavioral monitoring
- **Underwater acoustic modems**: For marine cage systems
- **Weather stations**: One per farm

### 8.2 Data Storage

- **Time-series database**: InfluxDB, TimescaleDB
- **Retention policy**:
  - Raw data: 90 days
  - Hourly aggregates: 2 years
  - Daily aggregates: 10 years
- **Backup**: Blockchain anchoring for traceability data

---

## 9. Future Extensions

- **Computer Vision**: Fish counting, size estimation, disease detection
- **AI/ML Models**: Growth prediction, disease outbreak forecasting
- **Genomics Integration**: Genetic markers for selective breeding
- **Carbon Credits**: Blue carbon sequestration tracking
- **Blockchain**: Supply chain transparency

---

**Document Status**: ✅ Phase 1 Complete
**Next Phase**: [PHASE-2-API-INTERFACE.md](./PHASE-2-API-INTERFACE.md)

---

© 2025 WIA (World Certification Industry Association)
**License**: MIT
**Philosophy**: 弘益人間 (Benefit All Humanity)
