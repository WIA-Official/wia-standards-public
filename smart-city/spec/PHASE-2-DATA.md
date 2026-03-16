# WIA-CITY-001: Smart City Standard - Phase 2 Data Format

**Version:** 1.0.0
**Status:** Active
**Category:** CITY (Smart City & Urban Systems)

---

## 1. Data Format Overview

### 1.1 Purpose
Define standardized data formats for all smart city sensor data, IoT devices, and urban infrastructure telemetry to ensure interoperability and consistent data processing across the entire city ecosystem.

### 1.2 Design Principles
- **Semantic Clarity**: Self-describing data with rich context
- **Extensibility**: Support for future sensor types and metrics
- **Efficiency**: Compact representation for bandwidth optimization
- **Validation**: Strong typing and schema enforcement
- **Versioning**: Backward compatibility and migration paths

---

## 2. Core Data Schema

### 2.1 Base Message Format

All WIA-CITY messages follow this base structure:

```json
{
  "@context": "https://wia.org/city/v1",
  "@type": "SmartCityData",
  "messageId": "uuid-v4",
  "timestamp": "ISO-8601 datetime",
  "protocol": {
    "name": "WIA-CITY-001",
    "version": "1.0.0"
  },
  "source": {
    "deviceId": "unique-device-identifier",
    "deviceType": "sensor|actuator|gateway",
    "location": { /* GeoJSON Point */ }
  },
  "payload": { /* Type-specific data */ },
  "metadata": {
    "quality": "high|medium|low",
    "confidence": 0.0-1.0,
    "tags": []
  },
  "security": {
    "signature": "cryptographic-signature",
    "issuer": "did:wia:city:device:xyz"
  }
}
```

---

## 3. Environmental Sensor Data

### 3.1 Air Quality Sensors

```json
{
  "@context": "https://wia.org/city/v1",
  "@type": "AirQualitySensorData",
  "messageId": "air-001-20251225-103000",
  "timestamp": "2025-12-25T10:30:00Z",
  "source": {
    "deviceId": "ENV-AIR-001",
    "deviceType": "AirQualityMonitor",
    "location": {
      "type": "Point",
      "coordinates": [126.9780, 37.5665],
      "zone": "gangnam-district",
      "elevation": 42.5
    },
    "manufacturer": "SmartSensor Corp",
    "model": "AQM-5000",
    "firmware": "2.3.1"
  },
  "measurements": [
    {
      "type": "PM2.5",
      "value": 35.2,
      "unit": "μg/m³",
      "quality": "good",
      "aqi": 42,
      "timestamp": "2025-12-25T10:30:00Z"
    },
    {
      "type": "PM10",
      "value": 58.1,
      "unit": "μg/m³",
      "quality": "moderate",
      "aqi": 55,
      "timestamp": "2025-12-25T10:30:00Z"
    },
    {
      "type": "O3",
      "value": 0.045,
      "unit": "ppm",
      "quality": "good",
      "aqi": 38,
      "timestamp": "2025-12-25T10:30:00Z"
    },
    {
      "type": "NO2",
      "value": 0.025,
      "unit": "ppm",
      "quality": "good",
      "aqi": 32,
      "timestamp": "2025-12-25T10:30:00Z"
    }
  ],
  "aggregatedAQI": {
    "value": 55,
    "category": "Moderate",
    "healthAdvice": "Sensitive groups should reduce prolonged outdoor exertion"
  },
  "metadata": {
    "samplingInterval": 60,
    "calibrationDate": "2025-12-01",
    "dataQuality": "high",
    "confidence": 0.95
  }
}
```

### 3.2 Weather Stations

```json
{
  "@type": "WeatherStationData",
  "measurements": [
    {
      "type": "temperature",
      "value": 22.5,
      "unit": "celsius"
    },
    {
      "type": "humidity",
      "value": 65,
      "unit": "percent"
    },
    {
      "type": "pressure",
      "value": 1013.25,
      "unit": "hPa"
    },
    {
      "type": "windSpeed",
      "value": 12.5,
      "unit": "km/h",
      "direction": 245
    },
    {
      "type": "precipitation",
      "value": 0.0,
      "unit": "mm/h"
    },
    {
      "type": "visibility",
      "value": 10.0,
      "unit": "km"
    }
  ]
}
```

### 3.3 Noise Sensors

```json
{
  "@type": "NoiseSensorData",
  "measurements": [
    {
      "type": "soundLevel",
      "value": 68.5,
      "unit": "dB(A)",
      "averaging": "LAeq,1min"
    },
    {
      "type": "peakLevel",
      "value": 85.2,
      "unit": "dB(A)",
      "timestamp": "2025-12-25T10:29:45Z"
    }
  ],
  "frequencySpectrum": {
    "125Hz": 52.3,
    "250Hz": 58.1,
    "500Hz": 62.4,
    "1kHz": 68.5,
    "2kHz": 64.2,
    "4kHz": 59.8,
    "8kHz": 54.1
  }
}
```

---

## 4. Traffic & Transportation Data

### 4.1 Traffic Cameras

```json
{
  "@type": "TrafficCameraData",
  "source": {
    "deviceId": "TRF-CAM-042",
    "location": {
      "intersection": "Gangnam Station Intersection",
      "coordinates": [127.0276, 37.4979],
      "roadNames": ["Teheran-ro", "Gangnam-daero"]
    }
  },
  "traffic": {
    "vehicleCount": {
      "total": 142,
      "byType": {
        "car": 98,
        "bus": 12,
        "truck": 8,
        "motorcycle": 18,
        "bicycle": 6
      },
      "byDirection": {
        "northbound": 42,
        "southbound": 38,
        "eastbound": 35,
        "westbound": 27
      }
    },
    "speed": {
      "average": 28.5,
      "median": 32.0,
      "unit": "km/h",
      "freeFlowSpeed": 50.0,
      "congestionRatio": 0.57
    },
    "occupancy": {
      "value": 0.68,
      "unit": "percent",
      "status": "moderate"
    },
    "queueLength": {
      "northbound": 15,
      "southbound": 12,
      "eastbound": 8,
      "westbound": 6,
      "unit": "vehicles"
    }
  },
  "incidents": [
    {
      "type": "slowTraffic",
      "severity": "moderate",
      "lane": "northbound-2",
      "description": "Slower than normal traffic flow"
    }
  ],
  "signalState": {
    "currentPhase": "NS-green",
    "timeRemaining": 25,
    "cycleLength": 120
  }
}
```

### 4.2 Parking Sensors

```json
{
  "@type": "ParkingSensorData",
  "source": {
    "facilityId": "PARK-GNST-01",
    "facilityName": "Gangnam Station Parking",
    "facilityType": "underground",
    "totalSpaces": 450
  },
  "occupancy": {
    "occupied": 324,
    "available": 126,
    "reserved": 20,
    "handicap": {
      "total": 15,
      "available": 4
    },
    "ev": {
      "total": 30,
      "available": 8,
      "charging": 18
    }
  },
  "zones": [
    {
      "zoneId": "B1",
      "occupied": 98,
      "available": 52
    },
    {
      "zoneId": "B2",
      "occupied": 112,
      "available": 38
    },
    {
      "zoneId": "B3",
      "occupied": 114,
      "available": 36
    }
  ],
  "pricing": {
    "currentRate": 3000,
    "currency": "KRW",
    "unit": "per hour"
  }
}
```

### 4.3 Public Transport

```json
{
  "@type": "PublicTransportData",
  "source": {
    "vehicleId": "BUS-7426-142",
    "route": "146",
    "operator": "Seoul Bus"
  },
  "location": {
    "type": "Point",
    "coordinates": [127.0276, 37.4979],
    "heading": 85,
    "speed": 32.5
  },
  "service": {
    "nextStop": "Gangnam Station",
    "stopsRemaining": 3,
    "estimatedArrival": "2025-12-25T10:35:00Z",
    "delay": -120,
    "onTime": true
  },
  "capacity": {
    "total": 45,
    "seated": 30,
    "standing": 15,
    "occupied": 38,
    "available": 7,
    "wheelchairSpaces": 2
  }
}
```

---

## 5. Energy System Data

### 5.1 Smart Meters

```json
{
  "@type": "SmartMeterData",
  "source": {
    "meterId": "SM-GN-52103",
    "meterType": "electricity",
    "customer": "encrypted-customer-id",
    "tariff": "residential-tiered"
  },
  "consumption": {
    "current": {
      "power": 2.45,
      "unit": "kW",
      "timestamp": "2025-12-25T10:30:00Z"
    },
    "accumulated": {
      "today": 18.2,
      "thisMonth": 425.8,
      "unit": "kWh"
    },
    "peakDemand": {
      "value": 4.8,
      "timestamp": "2025-12-25T08:15:00Z",
      "unit": "kW"
    }
  },
  "quality": {
    "voltage": 220.5,
    "frequency": 60.0,
    "powerFactor": 0.92,
    "harmonics": {
      "THD": 3.2,
      "unit": "percent"
    }
  },
  "tariffPeriod": "off-peak",
  "cost": {
    "current": 42.5,
    "projected": 65000,
    "currency": "KRW"
  }
}
```

### 5.2 Solar Panels

```json
{
  "@type": "SolarPanelData",
  "source": {
    "siteId": "SOLAR-GN-284",
    "capacity": 50.0,
    "unit": "kW",
    "panelCount": 150
  },
  "generation": {
    "current": 38.5,
    "today": 285.2,
    "thisMonth": 8420.5,
    "unit": "kWh"
  },
  "efficiency": {
    "current": 0.77,
    "averageDaily": 0.82,
    "degradation": 0.995
  },
  "environmental": {
    "irradiance": 850,
    "panelTemp": 42.5,
    "ambientTemp": 22.5,
    "cloudCover": 0.15
  },
  "impact": {
    "co2Avoided": 142.6,
    "unit": "kg",
    "treesEquivalent": 6.4
  }
}
```

### 5.3 EV Charging Stations

```json
{
  "@type": "EVChargingStationData",
  "source": {
    "stationId": "EV-GNST-018",
    "operator": "ChargePoint Korea",
    "location": {
      "coordinates": [127.0276, 37.4979],
      "address": "Gangnam Station Plaza"
    }
  },
  "chargers": [
    {
      "chargerId": "CH-01",
      "type": "DC Fast Charge",
      "power": 50.0,
      "unit": "kW",
      "status": "charging",
      "currentSession": {
        "startTime": "2025-12-25T10:15:00Z",
        "duration": 900,
        "energyDelivered": 12.5,
        "estimatedCompletion": "2025-12-25T10:45:00Z"
      }
    },
    {
      "chargerId": "CH-02",
      "type": "AC Level 2",
      "power": 7.2,
      "unit": "kW",
      "status": "available"
    }
  ],
  "pricing": {
    "rate": 280,
    "unit": "KRW/kWh",
    "parkingFee": 1000,
    "parkingUnit": "KRW/hour"
  }
}
```

---

## 6. Water & Waste Management

### 6.1 Water Quality Sensors

```json
{
  "@type": "WaterQualitySensorData",
  "source": {
    "sensorId": "WATER-142",
    "location": "Han River Monitoring Point 12",
    "waterBody": "Han River",
    "monitoringType": "surface-water"
  },
  "measurements": [
    {
      "type": "pH",
      "value": 7.2,
      "range": "6.5-8.5",
      "status": "normal"
    },
    {
      "type": "dissolvedOxygen",
      "value": 8.5,
      "unit": "mg/L",
      "status": "good"
    },
    {
      "type": "turbidity",
      "value": 2.3,
      "unit": "NTU",
      "status": "clear"
    },
    {
      "type": "conductivity",
      "value": 285,
      "unit": "μS/cm"
    },
    {
      "type": "temperature",
      "value": 18.5,
      "unit": "celsius"
    }
  ],
  "quality": {
    "overallRating": "excellent",
    "drinkable": false,
    "recreationalUse": true
  }
}
```

### 6.2 Smart Waste Bins

```json
{
  "@type": "SmartWasteBinData",
  "source": {
    "binId": "WASTE-1850",
    "location": {
      "coordinates": [127.0276, 37.4979],
      "address": "Gangnam Station Exit 5"
    },
    "binType": "recyclable",
    "capacity": 240
  },
  "status": {
    "fillLevel": 78,
    "unit": "percent",
    "weight": 42.5,
    "weightUnit": "kg",
    "lastCollection": "2025-12-24T06:30:00Z",
    "needsCollection": true,
    "priority": "medium"
  },
  "environmental": {
    "temperature": 22.0,
    "odorLevel": "low",
    "fireRisk": "none"
  },
  "collection": {
    "nextScheduled": "2025-12-26T06:30:00Z",
    "route": "GN-ROUTE-05",
    "estimatedTime": 180
  }
}
```

---

## 7. Public Safety Data

### 7.1 CCTV Cameras

```json
{
  "@type": "CCTVCameraData",
  "source": {
    "cameraId": "CCTV-627",
    "location": {
      "coordinates": [127.0276, 37.4979],
      "coverage": "Gangnam Station Plaza"
    },
    "capabilities": ["motion-detection", "face-blur", "license-plate"]
  },
  "analytics": {
    "pedestrianCount": 124,
    "crowdDensity": "moderate",
    "anomaliesDetected": 0,
    "privacyMode": "active",
    "faceBlurring": true,
    "dataRetention": "7-days"
  },
  "alerts": [],
  "status": {
    "operational": true,
    "lastMaintenance": "2025-12-15",
    "uptime": 0.9995
  }
}
```

### 7.2 Smart Streetlights

```json
{
  "@type": "SmartStreetlightData",
  "source": {
    "lightId": "LIGHT-4230",
    "location": {
      "coordinates": [127.0276, 37.4979]
    },
    "gridZone": "GN-GRID-12"
  },
  "status": {
    "on": true,
    "brightness": 85,
    "unit": "percent",
    "adaptiveDimming": true
  },
  "energy": {
    "power": 42,
    "unit": "W",
    "dailyConsumption": 0.504,
    "dailyUnit": "kWh"
  },
  "sensors": {
    "motionDetected": true,
    "ambientLight": 15,
    "ambientLightUnit": "lux"
  },
  "health": {
    "operational": true,
    "bulbLife": 85,
    "bulbLifeUnit": "percent",
    "lastMaintenance": "2025-11-20"
  }
}
```

---

## 8. Data Quality & Validation

### 8.1 Quality Metrics

All data includes quality indicators:

```json
{
  "dataQuality": {
    "completeness": 0.98,
    "accuracy": 0.95,
    "timeliness": 0.99,
    "consistency": 0.97,
    "validity": 1.0,
    "overallScore": 0.97
  }
}
```

### 8.2 Validation Rules

- **Timestamp**: Must be ISO-8601 format, within ±5 minutes of current time
- **Coordinates**: Valid lat/lon within city bounds
- **Values**: Within sensor-specific ranges
- **Units**: From approved unit registry
- **Required Fields**: All mandatory fields present

### 8.3 Error Handling

```json
{
  "errors": [
    {
      "code": "OUT_OF_RANGE",
      "field": "measurements[0].value",
      "message": "PM2.5 value 999 exceeds maximum threshold 500",
      "severity": "warning",
      "action": "flagged-for-review"
    }
  ]
}
```

---

## 9. Data Aggregation

### 9.1 Time-Series Aggregation

Data can be aggregated at different intervals:

```json
{
  "@type": "AggregatedData",
  "aggregation": {
    "interval": "1-hour",
    "method": "average",
    "dataPoints": 60,
    "startTime": "2025-12-25T10:00:00Z",
    "endTime": "2025-12-25T11:00:00Z"
  },
  "statistics": {
    "min": 32.5,
    "max": 45.8,
    "mean": 38.2,
    "median": 37.5,
    "stdDev": 3.4,
    "percentiles": {
      "p50": 37.5,
      "p95": 43.2,
      "p99": 45.1
    }
  }
}
```

---

## 10. Schema Versioning

### 10.1 Version Management

```json
{
  "schema": {
    "name": "WIA-CITY-AirQualitySensor",
    "version": "1.0.0",
    "compatibleWith": ["0.9.0", "1.0.0"],
    "deprecated": false,
    "migrationPath": null
  }
}
```

---

**Document Version:** 1.0.0
**Last Updated:** 2025-12-25

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
