# WIA-AGRI-029: Food Security Standard
## Phase 1 - Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines standardized data formats for food security management systems, including strategic reserves tracking, supply chain resilience monitoring, climate adaptation planning, distribution equity assessment, and emergency response coordination.

### 1.1 Design Principles

- **Comprehensive Coverage**: Track food security across all dimensions (availability, access, utilization, stability)
- **Real-time Monitoring**: Continuous monitoring of reserves, supply chains, and vulnerabilities
- **Predictive Capability**: Early warning systems for food security risks
- **Equity Focus**: Ensure fair distribution and access to food resources
- **Climate Resilience**: Integrate climate adaptation and risk mitigation strategies

---

## 2. Core Data Structures

### 2.1 Food Security Region Entity

```json
{
  "regionId": "string (UUID)",
  "regionCode": "string (e.g., ASIA-KR-SEOUL)",
  "regionName": "string",
  "regionType": "string (enum: country, province, city, district, rural)",
  "geography": {
    "area": "number (km²)",
    "climate": "string (tropical, temperate, arid, polar)",
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    },
    "elevation": "number (meters)",
    "arableLand": "number (km²)"
  },
  "population": {
    "total": "number",
    "urban": "number",
    "rural": "number",
    "vulnerable": "number (children, elderly, disabled)",
    "growthRate": "number (percentage)"
  },
  "economicIndicators": {
    "gdpPerCapita": "number (USD)",
    "povertyRate": "number (percentage)",
    "unemploymentRate": "number (percentage)",
    "foodExpenditureShare": "number (percentage of income)"
  },
  "foodSecurityIndex": {
    "overall": "number (0-100)",
    "availability": "number (0-100)",
    "access": "number (0-100)",
    "utilization": "number (0-100)",
    "stability": "number (0-100)"
  },
  "lastUpdated": "string (ISO 8601)",
  "did": "string (W3C DID)"
}
```

### 2.2 Strategic Food Reserve Data

```json
{
  "reserveId": "string (UUID)",
  "regionId": "string (UUID)",
  "foodType": "string (enum: grain, rice, wheat, corn, protein, vegetables, dairy, oil)",
  "category": "string (enum: strategic, emergency, buffer)",
  "quantity": {
    "current": "number (metric tons)",
    "target": "number (metric tons)",
    "minimum": "number (metric tons)",
    "maximum": "number (metric tons)"
  },
  "storageInfo": {
    "facilityId": "string (UUID)",
    "facilityName": "string",
    "location": {
      "address": "string",
      "city": "string",
      "coordinates": {"latitude": "number", "longitude": "number"}
    },
    "capacity": "number (metric tons)",
    "utilizationRate": "number (percentage)",
    "temperature": "number (Celsius)",
    "humidity": "number (percentage)"
  },
  "stockMetrics": {
    "monthsOfSupply": "number",
    "perCapitaReserve": "number (kg per person)",
    "turnoverRate": "number (times per year)",
    "expiryStatus": {
      "nearExpiry": "number (metric tons)",
      "expired": "number (metric tons)"
    }
  },
  "replenishment": {
    "lastDate": "string (ISO 8601)",
    "nextPlanned": "string (ISO 8601)",
    "frequency": "string (monthly, quarterly, annual)",
    "source": "string (domestic, imported)"
  },
  "quality": {
    "grade": "string (A, B, C)",
    "inspectionDate": "string (ISO 8601)",
    "certifications": ["ISO 22000", "HACCP", "Organic"],
    "contaminationStatus": "string (enum: clean, monitored, contaminated)"
  },
  "blockchain": {
    "enabled": "boolean",
    "network": "string",
    "contractAddress": "string",
    "transactionHash": "string"
  },
  "timestamp": "string (ISO 8601 with timezone)"
}
```

### 2.3 Supply Chain Resilience Data

```json
{
  "chainId": "string (UUID)",
  "regionId": "string (UUID)",
  "foodType": "string",
  "supplyRoute": {
    "origin": {
      "regionId": "string (UUID)",
      "countryCode": "string (ISO 3166-1)",
      "producerType": "string (farm, processor, distributor)"
    },
    "destination": {
      "regionId": "string (UUID)",
      "countryCode": "string (ISO 3166-1)"
    },
    "intermediatePoints": [
      {
        "pointId": "string (UUID)",
        "location": "string",
        "function": "string (processing, storage, distribution)"
      }
    ],
    "transportMode": ["road", "rail", "sea", "air"],
    "averageTransitDays": "number",
    "costPerTon": "number (USD)"
  },
  "resilienceMetrics": {
    "diversificationScore": "number (0-100)",
    "redundancy": "number (0-100)",
    "flexibility": "number (0-100)",
    "reliability": "number (0-100)",
    "overallScore": "number (0-100)"
  },
  "vulnerabilities": [
    {
      "type": "string (climate, geopolitical, infrastructure, economic)",
      "severity": "string (low, medium, high, critical)",
      "likelihood": "number (0-100)",
      "impact": "number (0-100)",
      "description": "string",
      "mitigationPlan": "string"
    }
  ],
  "alternatives": [
    {
      "alternativeId": "string (UUID)",
      "source": "string",
      "availability": "number (metric tons)",
      "costDifference": "number (percentage)",
      "activationTime": "number (days)"
    }
  ],
  "performanceHistory": {
    "deliveryReliability": "number (percentage)",
    "qualityConsistency": "number (percentage)",
    "priceStability": "number (percentage)",
    "disruptions": [
      {
        "date": "string (ISO 8601)",
        "cause": "string",
        "duration": "number (days)",
        "impactLevel": "string"
      }
    ]
  },
  "timestamp": "string (ISO 8601)"
}
```

### 2.4 Climate Adaptation Data

```json
{
  "adaptationId": "string (UUID)",
  "regionId": "string (UUID)",
  "climateRisks": [
    {
      "riskType": "string (drought, flood, heatwave, frost, storm)",
      "probability": "number (0-100)",
      "severity": "number (0-100)",
      "expectedImpact": {
        "cropYieldLoss": "number (percentage)",
        "affectedArea": "number (km²)",
        "economicLoss": "number (USD)"
      },
      "season": "string (spring, summer, fall, winter)",
      "historicalFrequency": "number (events per decade)"
    }
  ],
  "adaptationStrategies": [
    {
      "strategyId": "string (UUID)",
      "name": "string",
      "type": "string (crop_diversification, irrigation, storage, insurance)",
      "implementation": {
        "startDate": "string (ISO 8601)",
        "completionDate": "string (ISO 8601)",
        "status": "string (planned, ongoing, completed)",
        "budget": "number (USD)",
        "coverage": "number (km² or hectares)"
      },
      "effectiveness": {
        "riskReduction": "number (percentage)",
        "costBenefit": "number",
        "scalability": "string (low, medium, high)"
      }
    }
  ],
  "weatherMonitoring": {
    "temperature": {
      "current": "number (Celsius)",
      "forecast30Day": {"min": "number", "max": "number"},
      "anomaly": "number (deviation from historical average)"
    },
    "precipitation": {
      "currentMonth": "number (mm)",
      "forecast30Day": "number (mm)",
      "drought_index": "number (0-10)"
    },
    "soilMoisture": "number (percentage)",
    "cropWaterStress": "number (0-100)"
  },
  "earlyWarning": {
    "alerts": [
      {
        "alertId": "string (UUID)",
        "type": "string",
        "severity": "string (advisory, watch, warning, emergency)",
        "affectedArea": "string",
        "validFrom": "string (ISO 8601)",
        "validUntil": "string (ISO 8601)",
        "recommendations": ["string"]
      }
    ]
  },
  "timestamp": "string (ISO 8601)"
}
```

### 2.5 Distribution Equity Data

```json
{
  "distributionId": "string (UUID)",
  "regionId": "string (UUID)",
  "period": {
    "start": "string (ISO 8601)",
    "end": "string (ISO 8601)"
  },
  "populationSegments": [
    {
      "segment": "string (urban, rural, vulnerable, children, elderly)",
      "population": "number",
      "foodAccess": {
        "availability": "number (0-100)",
        "affordability": "number (0-100)",
        "adequacy": "number (0-100)"
      },
      "nutritionalStatus": {
        "calorieIntake": "number (kcal/day per capita)",
        "proteinIntake": "number (g/day per capita)",
        "undernourishment": "number (percentage)",
        "stunting": "number (percentage, children under 5)",
        "wasting": "number (percentage, children under 5)"
      },
      "assistancePrograms": [
        {
          "programId": "string (UUID)",
          "name": "string",
          "type": "string (subsidy, voucher, direct_distribution, school_feeding)",
          "beneficiaries": "number",
          "coverage": "number (percentage of target population)",
          "budget": "number (USD)"
        }
      ]
    }
  ],
  "equityMetrics": {
    "giniCoefficient": "number (0-1)",
    "accessGap": "number (percentage between highest and lowest quintile)",
    "vulnerabilityIndex": "number (0-100)",
    "targetAchievement": "number (percentage of SDG 2 targets met)"
  },
  "interventions": [
    {
      "interventionId": "string (UUID)",
      "type": "string (subsidy, infrastructure, education, market_regulation)",
      "targetGroup": "string",
      "expectedImpact": "number (percentage improvement)",
      "status": "string"
    }
  ],
  "timestamp": "string (ISO 8601)"
}
```

### 2.6 Emergency Response Data

```json
{
  "emergencyId": "string (UUID)",
  "regionId": "string (UUID)",
  "emergencyType": "string (drought, flood, conflict, pandemic, economic_crisis)",
  "severity": "string (level_1, level_2, level_3, catastrophic)",
  "status": "string (active, monitoring, resolved)",
  "timeline": {
    "declared": "string (ISO 8601)",
    "escalated": "string (ISO 8601)",
    "resolved": "string (ISO 8601)"
  },
  "impact": {
    "affectedPopulation": "number",
    "displacedPersons": "number",
    "foodInsecure": "number",
    "estimatedDuration": "number (days)",
    "economicLoss": "number (USD)"
  },
  "responseActions": [
    {
      "actionId": "string (UUID)",
      "type": "string (reserve_release, emergency_import, distribution, cash_transfer)",
      "quantity": "number (metric tons or USD)",
      "beneficiaries": "number",
      "startDate": "string (ISO 8601)",
      "duration": "number (days)",
      "status": "string (planned, ongoing, completed)",
      "provider": {
        "id": "string (UUID)",
        "name": "string",
        "type": "string (government, NGO, international_org)"
      }
    }
  ],
  "coordination": {
    "leadAgency": "string",
    "partnerAgencies": ["string"],
    "internationalSupport": [
      {
        "organization": "string (UN WFP, FAO, bilateral aid)",
        "commitmentAmount": "number (USD or metric tons)",
        "status": "string (pledged, delivered)"
      }
    ]
  },
  "logistics": {
    "distributionPoints": [
      {
        "pointId": "string (UUID)",
        "location": "string",
        "capacity": "number (beneficiaries per day)",
        "operationalStatus": "string"
      }
    ],
    "transportAssets": {
      "trucks": "number",
      "warehouses": "number",
      "staff": "number"
    }
  },
  "monitoring": {
    "peopleReached": "number",
    "foodDistributed": "number (metric tons)",
    "coverageRate": "number (percentage of affected population)",
    "effectivenessScore": "number (0-100)"
  },
  "timestamp": "string (ISO 8601)"
}
```

---

## 3. Metadata Standards

### 3.1 Timestamp Format

All timestamps MUST use ISO 8601 format with timezone:
```
2025-01-15T14:30:00+09:00
```

### 3.2 Unique Identifiers

- UUIDs: RFC 4122 compliant (e.g., `550e8400-e29b-41d4-a716-446655440000`)
- Region Codes: Pattern `[CONTINENT]-[COUNTRY]-[REGION]` (e.g., `ASIA-KR-SEOUL`)
- DIDs: W3C Decentralized Identifier specification

### 3.3 Units of Measurement

- Weight: Metric tons (1000 kg)
- Area: Square kilometers (km²) or hectares (ha)
- Temperature: Celsius (°C)
- Currency: USD (with exchange rate metadata)
- Time: Days, unless otherwise specified

---

## 4. Validation Rules

### 4.1 Required Fields

All entities MUST include:
- Unique identifier (UUID or DID)
- Timestamp (ISO 8601)
- Region identifier

### 4.2 Data Quality Constraints

- Food Security Index: 0-100 range
- Population counts: Non-negative integers
- Percentages: 0-100 range
- Coordinates: Valid latitude (-90 to 90) and longitude (-180 to 180)

### 4.3 Consistency Checks

- Reserve quantity ≤ storage capacity
- Months of supply = current reserve / monthly consumption
- Distribution coverage ≤ 100% of target population
- Supply chain reliability scores must be between 0-100

---

## 5. Security & Privacy

### 5.1 Data Classification

- **Public**: Aggregated food security indices, regional statistics
- **Restricted**: Detailed reserve locations, infrastructure vulnerabilities
- **Confidential**: Individual household food security data, emergency plans

### 5.2 Access Control

Implement role-based access control (RBAC):
- Public Dashboard: Read-only aggregate data
- Regional Managers: Read/write regional data
- National Authorities: Full access to national data
- Emergency Responders: Access to emergency response data

### 5.3 Encryption

- Data at rest: AES-256
- Data in transit: TLS 1.3
- Sensitive fields: Additional field-level encryption

---

## 6. Integration with WIA Standards

### 6.1 WIA-AGRI Standards

- **WIA-AGRI-001**: Precision agriculture data integration
- **WIA-AGRI-009**: Crop monitoring integration for yield predictions
- **WIA-AGRI-019**: Food traceability for supply chain tracking

### 6.2 WIA-CLIMATE Standards

- **WIA-CLIMATE-001**: Climate change mitigation data
- **WIA-CLIMATE-005**: Drought monitoring systems

### 6.3 WIA-BLOCKCHAIN Standards

- Reserve tracking on blockchain
- Supply chain transparency
- Distribution verification

---

## 7. Example Implementation

```json
{
  "regionId": "e7f8g9h0-1234-5678-9abc-def012345678",
  "regionCode": "ASIA-KR-SEOUL",
  "regionName": "Seoul Metropolitan Area",
  "regionType": "city",
  "population": {
    "total": 9776000,
    "urban": 9776000,
    "rural": 0,
    "vulnerable": 1250000
  },
  "foodSecurityIndex": {
    "overall": 82.5,
    "availability": 85.0,
    "access": 78.0,
    "utilization": 84.0,
    "stability": 83.0
  },
  "strategicReserves": [
    {
      "reserveId": "a1b2c3d4-5678-90ab-cdef-1234567890ab",
      "foodType": "rice",
      "quantity": {
        "current": 50000,
        "target": 60000,
        "minimum": 40000
      },
      "stockMetrics": {
        "monthsOfSupply": 6.25,
        "perCapitaReserve": 5.11
      }
    }
  ],
  "lastUpdated": "2025-01-15T14:30:00+09:00"
}
```

---

## 8. Appendix

### 8.1 Food Security Dimensions

1. **Availability**: Physical presence of food (production, reserves, imports)
2. **Access**: Economic and physical access to food (affordability, infrastructure)
3. **Utilization**: Nutritional value and safety of food
4. **Stability**: Consistent access over time (no seasonal or crisis-related disruptions)

### 8.2 Key Performance Indicators (KPIs)

- Months of strategic reserves
- Per capita food availability
- Prevalence of undernourishment
- Supply chain resilience score
- Climate adaptation readiness
- Distribution equity index
- Emergency response time

---

**Document Status**: ✅ Complete
**Next Phase**: [PHASE-2-API-INTERFACE.md](./PHASE-2-API-INTERFACE.md)

---

© 2025 WIA Standards | MIT License
弘益人間 · Benefit All Humanity
