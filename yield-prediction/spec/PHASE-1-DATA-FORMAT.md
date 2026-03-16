# WIA-AGRI-008: Yield Prediction Standard
## PHASE 1: Data Format Specification

**Version:** 1.0
**Status:** Active
**Category:** AGRI
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines the standardized data formats for agricultural yield prediction systems, ensuring interoperability across different platforms, regions, and crop types.

## 2. Core Data Schemas

### 2.1 Historical Yield Data Format

```json
{
  "farmId": "string (required)",
  "crop": "string (required)",
  "location": {
    "country": "ISO 3166-1 alpha-2",
    "province": "string",
    "district": "string (optional)",
    "coordinates": {
      "latitude": "number (-90 to 90)",
      "longitude": "number (-180 to 180)",
      "elevation": "number (meters, optional)"
    }
  },
  "harvestYear": "integer (YYYY)",
  "plantingDate": "ISO 8601 date (optional)",
  "harvestDate": "ISO 8601 date",
  "yield": {
    "amount": "number (positive)",
    "unit": "kg/ha | tons/ha | bushels/acre",
    "totalArea": "number (hectares)"
  },
  "inputs": {
    "fertilizer": {
      "nitrogen": "number (kg/ha, optional)",
      "phosphorus": "number (kg/ha, optional)",
      "potassium": "number (kg/ha, optional)"
    },
    "irrigation": "number (mm or m³/ha, optional)",
    "pesticides": "number (kg/ha, optional)",
    "laborHours": "number (optional)"
  },
  "weather": {
    "avgTemperature": "number (°C)",
    "rainfall": "number (mm)",
    "sunshineHours": "number (hours)",
    "extremeEvents": ["drought", "flood", "heatwave"] // optional
  },
  "soilData": {
    "pH": "number (0-14, optional)",
    "organicMatter": "number (%, optional)",
    "soilType": "string (optional)",
    "nutrientLevels": "object (optional)"
  },
  "metadata": {
    "dataSource": "manual | sensor | government | third-party",
    "verificationStatus": "verified | unverified | pending",
    "certificationBody": "string (optional)",
    "timestamp": "ISO 8601 datetime"
  }
}
```

### 2.2 Weather Data Integration Format

```json
{
  "stationId": "string",
  "location": {
    "latitude": "number",
    "longitude": "number"
  },
  "period": {
    "start": "ISO 8601 date",
    "end": "ISO 8601 date"
  },
  "measurements": [
    {
      "date": "ISO 8601 date",
      "temperature": {
        "min": "number (°C)",
        "max": "number (°C)",
        "avg": "number (°C)"
      },
      "precipitation": "number (mm)",
      "humidity": "number (%, optional)",
      "windSpeed": "number (m/s, optional)",
      "solarRadiation": "number (MJ/m², optional)",
      "evapotranspiration": "number (mm, optional)"
    }
  ],
  "aggregates": {
    "totalRainfall": "number (mm)",
    "avgTemperature": "number (°C)",
    "growingDegreeDays": "number (optional)"
  }
}
```

### 2.3 Real-time Monitoring Data

```json
{
  "farmId": "string",
  "sensorId": "string",
  "timestamp": "ISO 8601 datetime",
  "measurements": {
    "soilMoisture": "number (%, optional)",
    "soilTemperature": "number (°C, optional)",
    "ndvi": "number (0-1, optional)",
    "leafAreaIndex": "number (optional)",
    "cropHeight": "number (cm, optional)",
    "growthStage": "string (optional)"
  },
  "imageData": {
    "satelliteImage": "URL (optional)",
    "droneImage": "URL (optional)",
    "analysisResults": "object (optional)"
  }
}
```

## 3. Crop-Specific Standards

### 3.1 Supported Crops

| Crop Category | Standard Unit | Common Varieties |
|--------------|---------------|------------------|
| Grains | kg/ha | Wheat, Rice, Corn, Barley |
| Legumes | kg/ha | Soybean, Peas, Lentils |
| Tubers | kg/ha | Potato, Sweet Potato |
| Vegetables | kg/ha | Tomato, Cabbage, Onion |
| Fruits | kg/ha | Apple, Grape, Strawberry |
| Cash Crops | kg/ha | Cotton, Coffee, Tea |

### 3.2 Korean Agricultural Context

**통계청 (KOSIS) Integration:**

```json
{
  "ksicCode": "A011 (작물재배업)",
  "cropCode": {
    "rice": "111",
    "barley": "112",
    "corn": "113",
    "soybean": "121",
    "potato": "131"
  },
  "standardUnits": {
    "단수": "kg/10a (Korean standard)",
    "생산량": "tons (total production)",
    "재배면적": "ha (cultivated area)"
  },
  "conversionFactors": {
    "10a_to_ha": 0.1,
    "kg_10a_to_kg_ha": 10
  }
}
```

**Example Korean Rice Yield Data:**

```json
{
  "farmId": "KR-충남-당진-2025-001",
  "crop": "rice",
  "variety": "신동진벼",
  "location": {
    "country": "KR",
    "province": "충청남도",
    "city": "당진시",
    "coordinates": {
      "latitude": 36.8934,
      "longitude": 126.6469
    }
  },
  "harvestYear": 2025,
  "yield": {
    "amount": 5200,
    "unit": "kg/ha",
    "koreanUnit": {
      "단수": 520,
      "unit": "kg/10a"
    }
  },
  "inputs": {
    "fertilizer": {
      "nitrogen": 90,
      "phosphorus": 45,
      "potassium": 57
    }
  },
  "metadata": {
    "dataSource": "통계청 (KOSIS)",
    "verificationStatus": "verified"
  }
}
```

## 4. Data Quality Requirements

### 4.1 Mandatory Fields

- `farmId`: Unique identifier
- `crop`: Crop type (from standard list)
- `location.coordinates`: GPS coordinates (±10m accuracy)
- `harvestYear`: Year of harvest
- `yield.amount` and `yield.unit`: Actual yield data
- `metadata.timestamp`: Data collection time

### 4.2 Data Validation Rules

1. **Yield Range Validation:**
   - Rice: 2,000 - 12,000 kg/ha
   - Wheat: 1,000 - 10,000 kg/ha
   - Corn: 3,000 - 15,000 kg/ha
   - Values outside range flagged for review

2. **Temporal Validation:**
   - Planting date must precede harvest date
   - Growing season length must be realistic for crop type
   - Data must be within reasonable historical timeframe

3. **Geospatial Validation:**
   - Coordinates must be valid
   - Location must match climate zone for crop
   - Farm area must be positive and reasonable

4. **Weather Data Consistency:**
   - Temperature ranges must be physically possible
   - Rainfall accumulation must match individual measurements
   - No missing data for critical growth periods

## 5. File Formats

### 5.1 Supported Formats

| Format | Use Case | Priority |
|--------|----------|----------|
| JSON | API communication, data exchange | High |
| CSV | Bulk historical data, exports | High |
| GeoJSON | Spatial data with geometry | Medium |
| Parquet | Big data, ML training datasets | Medium |
| XML | Legacy system integration | Low |

### 5.2 CSV Format Example

```csv
farmId,crop,year,latitude,longitude,yield_kg_ha,rainfall_mm,avg_temp_c,fertilizer_n
KR-FARM-001,rice,2023,36.5184,127.2158,5200,850,22.5,90
KR-FARM-001,rice,2024,36.5184,127.2158,5350,920,21.8,85
KR-FARM-002,wheat,2023,37.5665,126.9780,4500,650,18.2,120
```

## 6. API Data Exchange

### 6.1 Request Format

```http
POST /api/v1/yield/submit
Content-Type: application/json
Authorization: Bearer {token}

{
  "farmId": "KR-FARM-2025-001",
  "crop": "rice",
  "harvestYear": 2025,
  "yield": { "amount": 5200, "unit": "kg/ha" },
  "location": { "latitude": 36.5184, "longitude": 127.2158 }
}
```

### 6.2 Response Format

```json
{
  "status": "success",
  "data": {
    "recordId": "rec_2025_001_abc123",
    "farmId": "KR-FARM-2025-001",
    "validationStatus": "passed",
    "qualityScore": 95,
    "warnings": [],
    "timestamp": "2025-12-26T10:30:00Z"
  }
}
```

## 7. Compliance and Certification

### 7.1 WIA Certification Levels

- **Level 1 (Basic):** Minimum required fields present
- **Level 2 (Standard):** Includes weather and soil data
- **Level 3 (Advanced):** Real-time monitoring integration
- **Level 4 (Premium):** Satellite imagery and AI analysis

### 7.2 Interoperability Standards

- Compatible with ISO 19156 (Observations and Measurements)
- Supports OGC SensorThings API
- GS1 traceability standards compatible
- W3C Verifiable Credentials for data authenticity

---

## Appendix A: Korean Agricultural Terms

| English | Korean | Unit |
|---------|--------|------|
| Yield | 수확량 / 단수 | kg/ha, kg/10a |
| Planting | 파종 / 이앙 | - |
| Harvest | 수확 | - |
| Rice paddy | 논 | ha |
| Upland field | 밭 | ha |
| Fertilizer | 비료 | kg/ha |
| Irrigation | 관개 | mm, m³ |

## Appendix B: Sample Datasets

Sample datasets available at:
- https://github.com/WIA-Official/wia-standards/tree/main/yield-prediction/samples
- Korean agricultural data: KOSIS (https://kosis.kr)
- Global yield data: FAO (http://www.fao.org/faostat)

---

**License:** CC BY 4.0
**Maintained by:** WIA Technical Committee
**Contact:** standards@wia.org
