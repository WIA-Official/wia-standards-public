# WIA-ENE-052: Seawater Desalination - PHASE 2 DATA STANDARDS

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Category:** Energy (ENE)

---

## 1. Overview

### 1.1 Purpose

This document defines standardized data formats, structures, and exchange protocols for seawater desalination systems. It ensures interoperability between different vendors, systems, and monitoring platforms while enabling comprehensive data collection, analysis, and reporting.

### 1.2 Scope

Data standards cover:
- Facility metadata and configuration
- Real-time operational data
- Water quality measurements
- Energy consumption metrics
- Maintenance and event logs
- Performance analytics
- Environmental monitoring data

---

## 2. Data Model Architecture

### 2.1 Core Data Entities

```
┌─────────────────────────────────────────────────────────┐
│                  DESALINATION DATA MODEL                 │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌────────────┐      ┌──────────────┐                  │
│  │  Facility  │──────│  Equipment   │                  │
│  │   Master   │      │   Registry   │                  │
│  └────────────┘      └──────────────┘                  │
│        │                     │                          │
│        ├─────────────────────┼──────────┐              │
│        │                     │          │              │
│  ┌────────────┐      ┌──────────────┐  │              │
│  │Operational │      │Water Quality │  │              │
│  │    Data    │      │     Data     │  │              │
│  └────────────┘      └──────────────┘  │              │
│        │                     │          │              │
│  ┌────────────┐      ┌──────────────┐  │              │
│  │   Energy   │      │ Maintenance  │  │              │
│  │    Data    │      │     Logs     │  │              │
│  └────────────┘      └──────────────┘  │              │
│        │                     │          │              │
│        └─────────────────────┴──────────┘              │
│                      │                                  │
│              ┌──────────────┐                          │
│              │  Analytics   │                          │
│              │  & Reports   │                          │
│              └──────────────┘                          │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

---

## 3. Facility Master Data

### 3.1 Facility Information Schema

```json
{
  "$schema": "https://wia.org/schemas/ene-052/facility/v1",
  "facilityId": "string (unique identifier)",
  "facilityName": "string",
  "owner": {
    "organizationId": "string",
    "organizationName": "string",
    "contact": {
      "email": "string",
      "phone": "string",
      "address": "object"
    }
  },
  "location": {
    "latitude": "number",
    "longitude": "number",
    "country": "string",
    "region": "string",
    "timezone": "string (IANA timezone)"
  },
  "specifications": {
    "technology": "enum[RO, MSF, MED, ED, HYBRID]",
    "designCapacity": {
      "value": "number",
      "unit": "m3/day"
    },
    "feedWaterSource": "enum[SEAWATER, BRACKISH, RIVER]",
    "productWaterUse": "enum[MUNICIPAL, INDUSTRIAL, AGRICULTURAL]",
    "commissioningDate": "ISO8601 date",
    "expectedLifespan": {
      "value": "number",
      "unit": "years"
    }
  },
  "certifications": [
    {
      "type": "string",
      "issuedBy": "string",
      "issuedDate": "ISO8601 date",
      "expiryDate": "ISO8601 date",
      "certificateId": "string"
    }
  ],
  "metadata": {
    "createdAt": "ISO8601 timestamp",
    "updatedAt": "ISO8601 timestamp",
    "version": "string"
  }
}
```

### 3.2 Example: Facility Record

```json
{
  "$schema": "https://wia.org/schemas/ene-052/facility/v1",
  "facilityId": "DESAL-ROE-UAE-001",
  "facilityName": "Dubai Jebel Ali RO Desalination Plant",
  "owner": {
    "organizationId": "ORG-DEWA-001",
    "organizationName": "Dubai Electricity and Water Authority",
    "contact": {
      "email": "operations@dewa.gov.ae",
      "phone": "+971-4-601-9999",
      "address": {
        "street": "Al Hudaiba Street",
        "city": "Dubai",
        "postalCode": "564",
        "country": "UAE"
      }
    }
  },
  "location": {
    "latitude": 25.0125,
    "longitude": 55.0447,
    "country": "United Arab Emirates",
    "region": "Dubai",
    "timezone": "Asia/Dubai"
  },
  "specifications": {
    "technology": "RO",
    "designCapacity": {
      "value": 300000,
      "unit": "m3/day"
    },
    "feedWaterSource": "SEAWATER",
    "productWaterUse": "MUNICIPAL",
    "commissioningDate": "2023-06-01",
    "expectedLifespan": {
      "value": 25,
      "unit": "years"
    }
  },
  "certifications": [
    {
      "type": "ISO 9001:2015",
      "issuedBy": "Bureau Veritas",
      "issuedDate": "2023-08-01",
      "expiryDate": "2026-08-01",
      "certificateId": "ISO9001-2023-UAE-001"
    }
  ],
  "metadata": {
    "createdAt": "2023-05-15T10:00:00Z",
    "updatedAt": "2025-12-25T08:30:00Z",
    "version": "1.2"
  }
}
```

---

## 4. Operational Data

### 4.1 Real-Time Operational Data Schema

```json
{
  "$schema": "https://wia.org/schemas/ene-052/operational/v1",
  "facilityId": "string",
  "timestamp": "ISO8601 timestamp",
  "production": {
    "feedFlow": {
      "value": "number",
      "unit": "m3/h",
      "sensor": "string (sensor ID)"
    },
    "permeateFlow": {
      "value": "number",
      "unit": "m3/h",
      "sensor": "string"
    },
    "concentrateFlow": {
      "value": "number",
      "unit": "m3/h",
      "sensor": "string"
    },
    "recoveryRate": {
      "value": "number",
      "unit": "percent",
      "calculated": true
    }
  },
  "pressure": {
    "feedPressure": {
      "value": "number",
      "unit": "bar",
      "sensor": "string"
    },
    "boostPumpPressure": {
      "value": "number",
      "unit": "bar",
      "sensor": "string"
    },
    "concentratePressure": {
      "value": "number",
      "unit": "bar",
      "sensor": "string"
    }
  },
  "energy": {
    "totalPowerConsumption": {
      "value": "number",
      "unit": "kW",
      "sensor": "string"
    },
    "specificEnergyConsumption": {
      "value": "number",
      "unit": "kWh/m3",
      "calculated": true
    },
    "energyRecovered": {
      "value": "number",
      "unit": "kW",
      "sensor": "string"
    }
  },
  "status": {
    "operationalState": "enum[RUNNING, STANDBY, MAINTENANCE, FAULT]",
    "alarmCount": "integer",
    "availability": "number (percent)"
  }
}
```

### 4.2 Example: Operational Data Record

```json
{
  "$schema": "https://wia.org/schemas/ene-052/operational/v1",
  "facilityId": "DESAL-ROE-UAE-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "production": {
    "feedFlow": {
      "value": 26250,
      "unit": "m3/h",
      "sensor": "FM-001"
    },
    "permeateFlow": {
      "value": 12500,
      "unit": "m3/h",
      "sensor": "FM-002"
    },
    "concentrateFlow": {
      "value": 13750,
      "unit": "m3/h",
      "sensor": "FM-003"
    },
    "recoveryRate": {
      "value": 47.6,
      "unit": "percent",
      "calculated": true
    }
  },
  "pressure": {
    "feedPressure": {
      "value": 5.2,
      "unit": "bar",
      "sensor": "PT-001"
    },
    "boostPumpPressure": {
      "value": 58.5,
      "unit": "bar",
      "sensor": "PT-002"
    },
    "concentratePressure": {
      "value": 56.8,
      "unit": "bar",
      "sensor": "PT-003"
    }
  },
  "energy": {
    "totalPowerConsumption": {
      "value": 42500,
      "unit": "kW",
      "sensor": "EM-001"
    },
    "specificEnergyConsumption": {
      "value": 3.4,
      "unit": "kWh/m3",
      "calculated": true
    },
    "energyRecovered": {
      "value": 15200,
      "unit": "kW",
      "sensor": "EM-ERD-001"
    }
  },
  "status": {
    "operationalState": "RUNNING",
    "alarmCount": 0,
    "availability": 98.5
  }
}
```

---

## 5. Water Quality Data

### 5.1 Water Quality Measurement Schema

```json
{
  "$schema": "https://wia.org/schemas/ene-052/water-quality/v1",
  "facilityId": "string",
  "sampleId": "string (unique)",
  "sampleLocation": "enum[FEED, PERMEATE, CONCENTRATE]",
  "sampleType": "enum[GRAB, COMPOSITE, CONTINUOUS]",
  "timestamp": "ISO8601 timestamp",
  "physicalParameters": {
    "temperature": {
      "value": "number",
      "unit": "celsius",
      "method": "string"
    },
    "turbidity": {
      "value": "number",
      "unit": "NTU",
      "method": "string"
    },
    "color": {
      "value": "number",
      "unit": "TCU",
      "method": "string"
    }
  },
  "chemicalParameters": {
    "pH": {
      "value": "number",
      "method": "string"
    },
    "tds": {
      "value": "number",
      "unit": "mg/L",
      "method": "string"
    },
    "conductivity": {
      "value": "number",
      "unit": "uS/cm",
      "method": "string"
    },
    "chloride": {
      "value": "number",
      "unit": "mg/L",
      "method": "string"
    },
    "sodium": {
      "value": "number",
      "unit": "mg/L",
      "method": "string"
    },
    "calcium": {
      "value": "number",
      "unit": "mg/L",
      "method": "string"
    },
    "magnesium": {
      "value": "number",
      "unit": "mg/L",
      "method": "string"
    },
    "sulfate": {
      "value": "number",
      "unit": "mg/L",
      "method": "string"
    },
    "alkalinity": {
      "value": "number",
      "unit": "mg/L as CaCO3",
      "method": "string"
    },
    "hardness": {
      "value": "number",
      "unit": "mg/L as CaCO3",
      "method": "string"
    }
  },
  "microbiologicalParameters": {
    "totalColiforms": {
      "value": "number",
      "unit": "CFU/100mL",
      "method": "string"
    },
    "eColi": {
      "value": "number",
      "unit": "CFU/100mL",
      "method": "string"
    },
    "heterotrophicPlateCount": {
      "value": "number",
      "unit": "CFU/mL",
      "method": "string"
    }
  },
  "complianceStatus": {
    "standard": "string (e.g., WHO, EPA, EU)",
    "compliant": "boolean",
    "violations": ["array of parameter names"]
  },
  "analyst": {
    "name": "string",
    "labId": "string",
    "certificationId": "string"
  }
}
```

### 5.2 Example: Water Quality Record

```json
{
  "$schema": "https://wia.org/schemas/ene-052/water-quality/v1",
  "facilityId": "DESAL-ROE-UAE-001",
  "sampleId": "WQ-2025-12-25-001",
  "sampleLocation": "PERMEATE",
  "sampleType": "COMPOSITE",
  "timestamp": "2025-12-25T09:00:00Z",
  "physicalParameters": {
    "temperature": {
      "value": 24.5,
      "unit": "celsius",
      "method": "ASTM D1293"
    },
    "turbidity": {
      "value": 0.3,
      "unit": "NTU",
      "method": "EPA 180.1"
    },
    "color": {
      "value": 2,
      "unit": "TCU",
      "method": "APHA 2120B"
    }
  },
  "chemicalParameters": {
    "pH": {
      "value": 7.2,
      "method": "ASTM D1293"
    },
    "tds": {
      "value": 145,
      "unit": "mg/L",
      "method": "ASTM D5907"
    },
    "conductivity": {
      "value": 290,
      "unit": "uS/cm",
      "method": "ASTM D1125"
    },
    "chloride": {
      "value": 45,
      "unit": "mg/L",
      "method": "ASTM D512"
    },
    "sodium": {
      "value": 38,
      "unit": "mg/L",
      "method": "ASTM D4691"
    },
    "calcium": {
      "value": 22,
      "unit": "mg/L",
      "method": "ASTM D511"
    },
    "magnesium": {
      "value": 8,
      "unit": "mg/L",
      "method": "ASTM D511"
    },
    "sulfate": {
      "value": 28,
      "unit": "mg/L",
      "method": "ASTM D516"
    },
    "alkalinity": {
      "value": 65,
      "unit": "mg/L as CaCO3",
      "method": "ASTM D1067"
    },
    "hardness": {
      "value": 85,
      "unit": "mg/L as CaCO3",
      "method": "ASTM D1126"
    }
  },
  "microbiologicalParameters": {
    "totalColiforms": {
      "value": 0,
      "unit": "CFU/100mL",
      "method": "EPA 1103.1"
    },
    "eColi": {
      "value": 0,
      "unit": "CFU/100mL",
      "method": "EPA 1103.1"
    },
    "heterotrophicPlateCount": {
      "value": 12,
      "unit": "CFU/mL",
      "method": "APHA 9215B"
    }
  },
  "complianceStatus": {
    "standard": "WHO Drinking Water Guidelines 2017",
    "compliant": true,
    "violations": []
  },
  "analyst": {
    "name": "Dr. Ahmed Al-Mansouri",
    "labId": "LAB-DEWA-WQ-01",
    "certificationId": "UAE-WQ-CERT-2023-045"
  }
}
```

---

## 6. Energy Data

### 6.1 Energy Consumption Schema

```json
{
  "$schema": "https://wia.org/schemas/ene-052/energy/v1",
  "facilityId": "string",
  "timestamp": "ISO8601 timestamp",
  "interval": "enum[REAL_TIME, 15MIN, HOURLY, DAILY]",
  "totalConsumption": {
    "value": "number",
    "unit": "kWh"
  },
  "breakdown": {
    "highPressurePumps": {
      "value": "number",
      "unit": "kWh",
      "percentage": "number"
    },
    "boosterPumps": {
      "value": "number",
      "unit": "kWh",
      "percentage": "number"
    },
    "pretreatment": {
      "value": "number",
      "unit": "kWh",
      "percentage": "number"
    },
    "posttreatment": {
      "value": "number",
      "unit": "kWh",
      "percentage": "number"
    },
    "auxiliary": {
      "value": "number",
      "unit": "kWh",
      "percentage": "number"
    }
  },
  "energyRecovery": {
    "recovered": {
      "value": "number",
      "unit": "kWh"
    },
    "efficiency": {
      "value": "number",
      "unit": "percent"
    }
  },
  "productionVolume": {
    "value": "number",
    "unit": "m3"
  },
  "sec": {
    "value": "number",
    "unit": "kWh/m3",
    "calculated": true
  },
  "renewableEnergy": {
    "solar": {
      "value": "number",
      "unit": "kWh"
    },
    "wind": {
      "value": "number",
      "unit": "kWh"
    },
    "percentage": "number"
  },
  "carbonFootprint": {
    "co2Emissions": {
      "value": "number",
      "unit": "kg CO2"
    },
    "emissionFactor": {
      "value": "number",
      "unit": "kg CO2/kWh"
    }
  }
}
```

---

## 7. Maintenance Logs

### 7.1 Maintenance Event Schema

```json
{
  "$schema": "https://wia.org/schemas/ene-052/maintenance/v1",
  "eventId": "string (unique)",
  "facilityId": "string",
  "equipmentId": "string",
  "equipmentType": "enum[PUMP, MEMBRANE, VALVE, SENSOR, ERD, OTHER]",
  "maintenanceType": "enum[PREVENTIVE, CORRECTIVE, PREDICTIVE, EMERGENCY]",
  "scheduledDate": "ISO8601 date",
  "actualStartDate": "ISO8601 timestamp",
  "actualEndDate": "ISO8601 timestamp",
  "duration": {
    "value": "number",
    "unit": "hours"
  },
  "downtime": {
    "value": "number",
    "unit": "hours"
  },
  "description": "string",
  "workPerformed": "string",
  "partsReplaced": [
    {
      "partId": "string",
      "partName": "string",
      "quantity": "integer",
      "cost": {
        "value": "number",
        "currency": "string"
      }
    }
  ],
  "technician": {
    "id": "string",
    "name": "string",
    "certifications": ["array of strings"]
  },
  "cost": {
    "labor": "number",
    "parts": "number",
    "total": "number",
    "currency": "string"
  },
  "outcome": "enum[COMPLETED, PENDING, DEFERRED]",
  "nextMaintenanceDue": "ISO8601 date"
}
```

---

## 8. Data Exchange Formats

### 8.1 Supported Formats

- **JSON** (Primary): REST API, web services, cloud platforms
- **XML**: Legacy system integration
- **CSV**: Bulk data export, reporting
- **Protocol Buffers**: High-performance, real-time data streaming
- **MQTT**: IoT sensor data, lightweight messaging

### 8.2 Data Compression

For large datasets:
- JSON: GZIP compression (typical 70-80% reduction)
- Time-series data: Delta encoding
- Binary formats: Protocol Buffers or MessagePack

---

## 9. Data Quality Requirements

### 9.1 Data Validation Rules

| Parameter | Range | Tolerance | Action on Violation |
|-----------|-------|-----------|---------------------|
| Feed Flow | 0 - Design Capacity × 1.1 | ±2% | Flag, verify sensor |
| Permeate Flow | 0 - Feed Flow | ±2% | Flag, verify sensor |
| TDS Output | 0 - 1000 mg/L | ±5% | Alert, check membrane |
| pH | 2 - 12 | ±0.1 | Alert, recalibrate |
| Pressure | 0 - 100 bar | ±1% | Flag, verify |
| Temperature | -10 - 50°C | ±0.5°C | Flag, verify |

### 9.2 Data Completeness

- **Real-time data**: 95% minimum availability
- **Quality samples**: 100% required fields
- **Maintenance logs**: 100% complete records
- **Energy data**: < 1% gaps per month

### 9.3 Data Accuracy

- Sensor calibration: Every 6 months minimum
- Reference standards: Traceable to international standards (NIST, PTB, etc.)
- Quality control: Duplicate samples, blind samples, reference materials

---

## 10. Data Retention

### 10.1 Retention Periods

| Data Type | Online Storage | Archive Storage | Legal Requirement |
|-----------|----------------|-----------------|-------------------|
| Real-time operational | 90 days | 7 years | Regulatory |
| Water quality | 1 year | Permanent | Regulatory |
| Energy consumption | 1 year | 7 years | Regulatory |
| Maintenance logs | 2 years | Equipment lifetime | Best practice |
| Alarms & events | 1 year | 5 years | Regulatory |
| Configuration changes | Permanent | Permanent | Audit trail |

---

## 11. Data Security

### 11.1 Security Requirements

- **Encryption**: TLS 1.3 for data in transit, AES-256 for data at rest
- **Authentication**: OAuth 2.0, API keys with rotation
- **Authorization**: Role-based access control (RBAC)
- **Audit logs**: All data access and modifications logged
- **Backup**: Daily incremental, weekly full backup

### 11.2 Privacy Compliance

- GDPR compliance for EU operations
- Data anonymization for analytics and research
- Consent management for third-party data sharing

---

## 12. API Data Exchange

### 12.1 REST API Endpoints

```
GET    /api/v1/facilities/{facilityId}
GET    /api/v1/facilities/{facilityId}/operational/latest
GET    /api/v1/facilities/{facilityId}/operational?start={ts}&end={ts}
POST   /api/v1/facilities/{facilityId}/operational
GET    /api/v1/facilities/{facilityId}/quality/latest
POST   /api/v1/facilities/{facilityId}/quality
GET    /api/v1/facilities/{facilityId}/energy/daily
GET    /api/v1/facilities/{facilityId}/maintenance
POST   /api/v1/facilities/{facilityId}/maintenance
```

### 12.2 Real-time Data Streaming

**MQTT Topics:**
```
wia/ene052/{facilityId}/operational
wia/ene052/{facilityId}/quality
wia/ene052/{facilityId}/energy
wia/ene052/{facilityId}/alarms
```

**WebSocket:**
```
wss://api.wia.org/v1/facilities/{facilityId}/stream
```

---

**Document Control:**
- **Version:** 1.0
- **Author:** WIA Standards Committee
- **Approved By:** WIA Technical Board
- **Next Review:** 2026-12-25

**License:** CC BY-SA 4.0
**Copyright:** © 2025 SmileStory Inc. / WIA

弘益人間 (홍익인간) - Benefit All Humanity
