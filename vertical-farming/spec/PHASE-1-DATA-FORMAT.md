# WIA-AGRI-018: Vertical Farming Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines the data formats for vertical farming operations, including environmental monitoring, crop management, harvest tracking, and system control.

### 1.1 Design Principles

- **Scalability**: Support from small urban farms to large commercial facilities
- **Real-time**: Sub-second latency for critical environmental controls
- **Interoperability**: Compatible with IoT platforms, ERP systems, and supply chain networks
- **Sustainability**: Track resource usage and carbon footprint metrics
- **Traceability**: Complete farm-to-table data lineage

---

## 2. Core Data Structures

### 2.1 Farm Configuration Data

Basic information about the vertical farming facility.

```json
{
  "farmConfig": {
    "farmId": "VF-SEOUL-001",
    "farmName": "Seoul Vertical Farm",
    "location": {
      "address": "123 Gangnam-daero, Seoul, South Korea",
      "latitude": 37.5665,
      "longitude": 126.9780,
      "timezone": "Asia/Seoul"
    },
    "infrastructure": {
      "totalArea": 1000,
      "numberOfTiers": 8,
      "tierHeight": 2.5,
      "totalVolume": 20000,
      "powerCapacity": 500,
      "waterCapacity": 5000
    },
    "systems": {
      "growingSystem": "NFT",
      "lightingType": "LED_FULL_SPECTRUM",
      "climateControl": "HVAC_AUTOMATED",
      "waterSystem": "RECIRCULATING",
      "automationLevel": "FULLY_AUTOMATED"
    },
    "certifications": ["ORGANIC", "PESTICIDE_FREE", "CARBON_NEUTRAL"],
    "operationalSince": "2024-01-01",
    "capacity": {
      "annualProduction": 50000,
      "unit": "kg",
      "simultaneousPlants": 8000
    }
  }
}
```

**Field Descriptions:**
- `totalArea`: Total floor space in square meters
- `numberOfTiers`: Vertical stacking levels
- `growingSystem`: NFT (Nutrient Film Technique), DWC (Deep Water Culture), AEROPONICS, or EBB_FLOW
- `automationLevel`: MANUAL, SEMI_AUTOMATED, or FULLY_AUTOMATED

### 2.2 Environmental Monitoring Data

Real-time sensor data from growing environment.

```json
{
  "environmentData": {
    "farmId": "VF-SEOUL-001",
    "tier": 3,
    "zone": "A",
    "timestamp": "2025-01-01T10:30:45.123Z",
    "climate": {
      "temperature": {
        "value": 22.5,
        "unit": "celsius",
        "status": "OPTIMAL",
        "setpoint": 22.0,
        "tolerance": 1.5
      },
      "humidity": {
        "value": 65,
        "unit": "percent",
        "status": "OPTIMAL",
        "setpoint": 65,
        "tolerance": 5
      },
      "co2": {
        "value": 1200,
        "unit": "ppm",
        "status": "OPTIMAL",
        "setpoint": 1200,
        "tolerance": 100
      },
      "airflow": {
        "velocity": 0.5,
        "unit": "m/s",
        "status": "NORMAL"
      }
    },
    "lighting": {
      "intensity": {
        "value": 450,
        "unit": "umol/m2/s",
        "status": "ACTIVE"
      },
      "spectrum": {
        "red": 100,
        "blue": 80,
        "white": 60,
        "farRed": 20
      },
      "photoperiod": {
        "hoursOn": 16,
        "hoursOff": 8,
        "currentState": "ON"
      },
      "powerConsumption": 120,
      "efficiency": 2.7
    },
    "water": {
      "temperature": 20.5,
      "ph": {
        "value": 6.2,
        "status": "OPTIMAL",
        "setpoint": 6.0,
        "tolerance": 0.5
      },
      "ec": {
        "value": 1.8,
        "unit": "mS/cm",
        "status": "OPTIMAL",
        "setpoint": 1.8,
        "tolerance": 0.3
      },
      "dissolvedOxygen": 8.2,
      "flowRate": 2.5
    }
  }
}
```

**Status Values:**
- `OPTIMAL`: Within ideal range for crop growth
- `NORMAL`: Acceptable but not ideal
- `WARNING`: Approaching limits
- `CRITICAL`: Immediate intervention required

### 2.3 Crop Management Data

Tracking individual crop batches and growth cycles.

```json
{
  "cropBatch": {
    "batchId": "BATCH-2025-001",
    "farmId": "VF-SEOUL-001",
    "tier": 3,
    "cropType": {
      "species": "Lactuca sativa",
      "variety": "Green Leaf Lettuce",
      "commonName": "Lettuce",
      "cultivar": "Rex"
    },
    "lifecycle": {
      "seedingDate": "2024-12-15T00:00:00Z",
      "transplantDate": "2024-12-22T00:00:00Z",
      "expectedHarvestDate": "2025-01-15T00:00:00Z",
      "actualHarvestDate": null,
      "currentAge": 17,
      "growthStage": "VEGETATIVE",
      "estimatedDaysToHarvest": 13
    },
    "plantCount": {
      "seeded": 500,
      "transplanted": 485,
      "current": 480,
      "mortality": 5,
      "expectedYield": 216
    },
    "growthParameters": {
      "targetHeight": 20,
      "currentHeight": 12,
      "targetWeight": 450,
      "currentWeight": 280,
      "leafCount": 8,
      "healthScore": 95
    },
    "nutrientSchedule": {
      "formulaId": "LETTUCE-VEG-01",
      "nitrogen": 150,
      "phosphorus": 50,
      "potassium": 200,
      "calcium": 180,
      "magnesium": 50,
      "feedingFrequency": "CONTINUOUS"
    },
    "qualityMetrics": {
      "color": "BRIGHT_GREEN",
      "textureScore": 9.2,
      "tasteScore": 9.5,
      "marketGrade": "PREMIUM",
      "defectRate": 0.02
    }
  }
}
```

**Growth Stages:**
- `SEEDING`: Initial germination phase
- `SEEDLING`: Young plant establishment
- `VEGETATIVE`: Active leaf/stem growth
- `FLOWERING`: Reproductive phase (if applicable)
- `RIPENING`: Final maturation before harvest
- `READY_TO_HARVEST`: Optimal harvest window

### 2.4 Harvest Record Data

Detailed tracking of harvest events and produce quality.

```json
{
  "harvestRecord": {
    "harvestId": "HRV-2025-001",
    "batchId": "BATCH-2025-001",
    "farmId": "VF-SEOUL-001",
    "harvestDate": "2025-01-15T08:30:00Z",
    "operator": {
      "employeeId": "EMP-123",
      "name": "Park Harvest",
      "certifications": ["FOOD_SAFETY", "ORGANIC_HANDLING"]
    },
    "yield": {
      "totalWeight": 216.5,
      "unit": "kg",
      "plantCount": 480,
      "averageWeightPerPlant": 0.451,
      "wasteWeight": 8.2,
      "wastePercentage": 3.6
    },
    "quality": {
      "grade": "PREMIUM",
      "appearance": {
        "color": "EXCELLENT",
        "uniformity": 9.5,
        "damageRate": 0.01
      },
      "freshness": {
        "firmness": 9.8,
        "crispness": 9.7,
        "shelfLifeEstimate": 14
      },
      "safety": {
        "pesticideResidues": "NONE",
        "microbiologyTest": "PASS",
        "heavyMetals": "PASS"
      }
    },
    "packaging": {
      "packageType": "CLAMSHELL_200G",
      "packagesProduced": 1080,
      "labelingCompliance": true,
      "traceabilityQR": true
    },
    "destination": {
      "warehouseId": "WH-SEOUL-01",
      "distributionChannel": "PREMIUM_RETAIL",
      "estimatedDeliveryDate": "2025-01-15T18:00:00Z",
      "customerOrders": ["ORD-2025-045", "ORD-2025-046"]
    },
    "carbonFootprint": {
      "totalEmissions": 12.5,
      "unit": "kg_CO2e",
      "emissionsPerKg": 0.058,
      "offsetCertificate": "OFFSET-2025-001"
    }
  }
}
```

### 2.5 Resource Consumption Data

Tracking water, energy, and nutrient usage for sustainability.

```json
{
  "resourceMetrics": {
    "farmId": "VF-SEOUL-001",
    "period": {
      "startDate": "2025-01-01T00:00:00Z",
      "endDate": "2025-01-31T23:59:59Z",
      "type": "MONTHLY"
    },
    "water": {
      "totalIntake": 15000,
      "recycled": 14250,
      "waste": 750,
      "efficiency": 95.0,
      "unit": "liters",
      "costPerLiter": 0.002,
      "totalCost": 30.0
    },
    "energy": {
      "totalConsumption": 12000,
      "lighting": 7200,
      "hvac": 3600,
      "pumps": 720,
      "other": 480,
      "unit": "kWh",
      "renewablePercentage": 80,
      "costPerKWh": 0.12,
      "totalCost": 1440.0
    },
    "nutrients": {
      "nitrogen": {
        "consumed": 45,
        "recycled": 5,
        "cost": 180
      },
      "phosphorus": {
        "consumed": 15,
        "recycled": 2,
        "cost": 75
      },
      "potassium": {
        "consumed": 60,
        "recycled": 8,
        "cost": 240
      },
      "unit": "kg",
      "totalCost": 495.0
    },
    "production": {
      "totalYield": 4200,
      "unit": "kg",
      "waterPerKg": 3.57,
      "energyPerKg": 2.86,
      "nutrientCostPerKg": 0.12
    },
    "comparison": {
      "traditionalFarming": {
        "waterPerKg": 70,
        "waterSavings": 94.9,
        "landPerKg": 0.025,
        "landSavings": 99.7
      }
    }
  }
}
```

### 2.6 System Alert Data

Automated alerts and notifications for anomalies.

```json
{
  "systemAlert": {
    "alertId": "ALERT-2025-001",
    "farmId": "VF-SEOUL-001",
    "tier": 3,
    "timestamp": "2025-01-01T14:22:15Z",
    "severity": "WARNING",
    "category": "ENVIRONMENTAL",
    "type": "TEMPERATURE_HIGH",
    "message": "Temperature exceeding optimal range on Tier 3",
    "details": {
      "parameter": "temperature",
      "currentValue": 26.5,
      "setpoint": 22.0,
      "threshold": 25.0,
      "deviation": 4.5,
      "duration": 15
    },
    "impact": {
      "affectedBatches": ["BATCH-2025-001", "BATCH-2025-002"],
      "affectedPlants": 960,
      "estimatedYieldLoss": 2.5
    },
    "recommendations": [
      "Increase HVAC cooling output",
      "Verify cooling system function",
      "Check for ventilation blockages",
      "Reduce LED intensity if overheating"
    ],
    "actions": {
      "automated": [
        {
          "action": "INCREASE_COOLING",
          "timestamp": "2025-01-01T14:22:20Z",
          "result": "SUCCESS"
        }
      ],
      "manual": [],
      "resolved": false,
      "resolvedAt": null,
      "resolvedBy": null
    },
    "escalation": {
      "notified": ["supervisor@verticalfarm.com", "ops-manager@verticalfarm.com"],
      "escalationLevel": 1,
      "requiresAcknowledgment": true,
      "acknowledged": false
    }
  }
}
```

**Severity Levels:**
- `INFO`: Informational, no action required
- `WARNING`: Attention needed, automated response initiated
- `CRITICAL`: Immediate intervention required
- `EMERGENCY`: System failure, crop loss imminent

**Alert Categories:**
- `ENVIRONMENTAL`: Temperature, humidity, CO2, light
- `WATER_QUALITY`: pH, EC, temperature, DO
- `EQUIPMENT`: Pump failure, LED malfunction, sensor error
- `BIOLOGICAL`: Pest detection, disease signs, growth anomaly
- `RESOURCE`: Water shortage, power interruption, nutrient depletion

---

## 3. Data Validation Rules

### 3.1 Environmental Ranges

| Parameter | Min | Max | Optimal Range | Unit |
|-----------|-----|-----|---------------|------|
| Temperature | 15 | 30 | 20-24 | °C |
| Humidity | 40 | 85 | 60-70 | % |
| CO2 | 400 | 2000 | 1000-1500 | ppm |
| pH | 5.0 | 7.0 | 5.8-6.5 | pH |
| EC | 0.8 | 3.0 | 1.5-2.2 | mS/cm |
| Light Intensity | 200 | 800 | 400-600 | μmol/m²/s |

### 3.2 Required Fields

All core data structures must include:
- `farmId`: Valid farm identifier
- `timestamp`: ISO 8601 format
- `version`: Data format version

### 3.3 Data Retention

- Real-time sensor data: 90 days
- Daily aggregates: 7 years
- Harvest records: Permanent
- Alert history: 3 years

---

## 4. File Formats

### 4.1 JSON Format (Recommended)

Primary format for API communication and data exchange.

**Advantages:**
- Human-readable
- Wide platform support
- Efficient parsing
- Schema validation available

### 4.2 CSV Format (Bulk Export)

For historical data exports and analytics.

```csv
timestamp,farmId,tier,temperature,humidity,co2,ph,ec
2025-01-01T10:00:00Z,VF-SEOUL-001,3,22.5,65,1200,6.2,1.8
2025-01-01T10:01:00Z,VF-SEOUL-001,3,22.6,65,1195,6.2,1.8
```

### 4.3 Binary Format (IoT Sensors)

Compressed binary for bandwidth-constrained sensors.

**Protocol:** MessagePack or Protocol Buffers
**Compression:** Achieves 40-60% size reduction
**Use Case:** Remote farms with limited connectivity

---

## 5. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial specification release |

---

**© 2025 WIA Standards · MIT License**
**弘益人間 (Benefit All Humanity)**
