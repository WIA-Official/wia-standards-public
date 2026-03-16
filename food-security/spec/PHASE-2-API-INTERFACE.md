# WIA-AGRI-029: Food Security Standard
## Phase 2 - API Interface Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines RESTful API interfaces for food security management systems, enabling interoperability between government agencies, humanitarian organizations, agricultural systems, and supply chain partners.

### 1.1 Base URL Structure

```
https://api.food-security.wia/{version}/{service}/{resource}
```

Example:
```
https://api.food-security.wia/v1/reserves/status
https://api.food-security.wia/v1/supply-chain/resilience
```

### 1.2 API Versioning

- Current version: `v1`
- Version specified in URL path
- Breaking changes require new version number

---

## 2. Authentication & Authorization

### 2.1 Authentication Methods

**Option 1: OAuth 2.0**
```http
Authorization: Bearer {access_token}
```

**Option 2: API Key**
```http
X-API-Key: {api_key}
X-API-Secret: {api_secret}
```

**Option 3: W3C DID Authentication**
```http
Authorization: DID {did}:{signature}
```

### 2.2 Authorization Scopes

```yaml
scopes:
  - food-security:read         # Read food security data
  - food-security:write        # Update food security data
  - reserves:manage            # Manage strategic reserves
  - emergency:respond          # Emergency response operations
  - distribution:manage        # Manage food distribution
  - analytics:read             # Access analytics and forecasts
  - admin:full                 # Full administrative access
```

---

## 3. Core API Endpoints

### 3.1 Food Security Status

#### GET /api/v1/security/status

Get overall food security status for a region.

**Request:**
```http
GET /api/v1/security/status?regionId=ASIA-KR-SEOUL
Authorization: Bearer {token}
```

**Query Parameters:**
- `regionId` (required): Region identifier
- `date` (optional): Specific date for historical data (ISO 8601)
- `includeSubregions` (optional): Include sub-region data (boolean)

**Response:**
```json
{
  "status": "success",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "regionId": "ASIA-KR-SEOUL",
    "regionName": "Seoul Metropolitan Area",
    "foodSecurityIndex": {
      "overall": 82.5,
      "availability": 85.0,
      "access": 78.0,
      "utilization": 84.0,
      "stability": 83.0
    },
    "status": "adequate",
    "alerts": [],
    "lastUpdated": "2025-01-15T14:00:00+09:00"
  }
}
```

**Status Codes:**
- `200 OK`: Success
- `400 Bad Request`: Invalid parameters
- `401 Unauthorized`: Authentication required
- `404 Not Found`: Region not found

---

### 3.2 Strategic Reserves Management

#### GET /api/v1/reserves/status

Get strategic reserve status.

**Request:**
```http
GET /api/v1/reserves/status?regionId=ASIA-KR-SEOUL&foodType=rice
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "reserves": [
      {
        "reserveId": "a1b2c3d4-5678-90ab-cdef-1234567890ab",
        "foodType": "rice",
        "quantity": {
          "current": 50000,
          "target": 60000,
          "minimum": 40000,
          "utilizationRate": 83.3
        },
        "stockMetrics": {
          "monthsOfSupply": 6.25,
          "perCapitaReserve": 5.11
        },
        "status": "adequate"
      }
    ],
    "totalReserves": 1,
    "overallStatus": "adequate"
  }
}
```

#### POST /api/v1/reserves/monitor

Record reserve monitoring data.

**Request:**
```http
POST /api/v1/reserves/monitor
Authorization: Bearer {token}
Content-Type: application/json

{
  "reserveId": "a1b2c3d4-5678-90ab-cdef-1234567890ab",
  "quantity": 50000,
  "storageInfo": {
    "temperature": 15.5,
    "humidity": 60,
    "inspectionDate": "2025-01-15"
  },
  "quality": {
    "grade": "A",
    "contaminationStatus": "clean"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Reserve monitoring data recorded",
  "monitoringId": "m1n2o3p4-qrst-uvwx-yz12-345678901234",
  "timestamp": "2025-01-15T14:30:00Z"
}
```

#### PUT /api/v1/reserves/{reserveId}/replenish

Replenish strategic reserves.

**Request:**
```http
PUT /api/v1/reserves/a1b2c3d4-5678-90ab-cdef-1234567890ab/replenish
Authorization: Bearer {token}
Content-Type: application/json

{
  "quantity": 10000,
  "source": "domestic",
  "expectedDelivery": "2025-02-01",
  "cost": 5000000
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Replenishment order created",
  "orderId": "ord-123456789",
  "newQuantity": 60000,
  "monthsOfSupply": 7.5
}
```

---

### 3.3 Supply Chain Resilience

#### GET /api/v1/supply-chain/resilience

Get supply chain resilience assessment.

**Request:**
```http
GET /api/v1/supply-chain/resilience?regionId=ASIA-KR-SEOUL
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "regionId": "ASIA-KR-SEOUL",
    "resilienceMetrics": {
      "diversificationScore": 75.0,
      "redundancy": 68.0,
      "flexibility": 82.0,
      "reliability": 91.0,
      "overallScore": 79.0
    },
    "vulnerabilities": [
      {
        "type": "climate",
        "severity": "medium",
        "likelihood": 65,
        "impact": 70,
        "description": "Drought risk affecting rice imports from Southeast Asia"
      }
    ],
    "alternatives": 3,
    "status": "resilient"
  }
}
```

#### POST /api/v1/supply-chain/alert

Create supply chain alert.

**Request:**
```http
POST /api/v1/supply-chain/alert
Authorization: Bearer {token}
Content-Type: application/json

{
  "regionId": "ASIA-KR-SEOUL",
  "alertType": "disruption",
  "severity": "high",
  "foodType": "wheat",
  "description": "Port strike affecting wheat imports",
  "expectedDuration": 7,
  "alternativesAvailable": true
}
```

**Response:**
```json
{
  "status": "success",
  "alertId": "alert-987654321",
  "message": "Supply chain alert created",
  "notificationsSent": 15,
  "timestamp": "2025-01-15T14:30:00Z"
}
```

---

### 3.4 Climate Adaptation

#### GET /api/v1/climate/risks

Get climate-related food security risks.

**Request:**
```http
GET /api/v1/climate/risks?regionId=ASIA-KR-SEOUL&timeframe=90days
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "regionId": "ASIA-KR-SEOUL",
    "timeframe": "90days",
    "risks": [
      {
        "riskType": "drought",
        "probability": 45,
        "severity": 60,
        "expectedImpact": {
          "cropYieldLoss": 15,
          "affectedArea": 250,
          "economicLoss": 12000000
        },
        "season": "summer"
      }
    ],
    "earlyWarning": {
      "alerts": [],
      "weatherOutlook": "normal",
      "recommendations": ["Monitor soil moisture levels", "Prepare irrigation systems"]
    }
  }
}
```

#### POST /api/v1/climate/adaptation-strategy

Submit climate adaptation strategy.

**Request:**
```http
POST /api/v1/climate/adaptation-strategy
Authorization: Bearer {token}
Content-Type: application/json

{
  "regionId": "ASIA-KR-SEOUL",
  "strategyName": "Drought-Resistant Crop Program",
  "type": "crop_diversification",
  "implementation": {
    "startDate": "2025-03-01",
    "budget": 5000000,
    "coverage": 500
  },
  "targetRisk": "drought",
  "expectedEffectiveness": {
    "riskReduction": 30,
    "scalability": "high"
  }
}
```

**Response:**
```json
{
  "status": "success",
  "strategyId": "strat-456789012",
  "message": "Adaptation strategy registered",
  "approvalRequired": true
}
```

---

### 3.5 Distribution Equity

#### GET /api/v1/distribution/equity

Get food distribution equity metrics.

**Request:**
```http
GET /api/v1/distribution/equity?regionId=ASIA-KR-SEOUL
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "regionId": "ASIA-KR-SEOUL",
    "period": {
      "start": "2025-01-01",
      "end": "2025-01-31"
    },
    "equityMetrics": {
      "giniCoefficient": 0.28,
      "accessGap": 22,
      "vulnerabilityIndex": 35,
      "targetAchievement": 78
    },
    "populationSegments": [
      {
        "segment": "vulnerable",
        "population": 1250000,
        "foodAccess": {
          "availability": 72,
          "affordability": 65,
          "adequacy": 70
        }
      }
    ],
    "assistancePrograms": 8,
    "beneficiariesReached": 450000
  }
}
```

#### POST /api/v1/distribution/intervention

Create food distribution intervention.

**Request:**
```http
POST /api/v1/distribution/intervention
Authorization: Bearer {token}
Content-Type: application/json

{
  "regionId": "ASIA-KR-SEOUL",
  "type": "subsidy",
  "targetGroup": "low_income",
  "targetPopulation": 500000,
  "budget": 10000000,
  "duration": 180,
  "expectedImpact": 25
}
```

**Response:**
```json
{
  "status": "success",
  "interventionId": "int-345678901",
  "message": "Distribution intervention created",
  "estimatedBeneficiaries": 500000,
  "startDate": "2025-02-01"
}
```

---

### 3.6 Emergency Response

#### POST /api/v1/emergency/declare

Declare food security emergency.

**Request:**
```http
POST /api/v1/emergency/declare
Authorization: Bearer {token}
Content-Type: application/json

{
  "regionId": "ASIA-KR-AFFECTED",
  "emergencyType": "drought",
  "severity": "level_2",
  "affectedPopulation": 500000,
  "estimatedDuration": 90,
  "requestedSupport": {
    "food": 5000,
    "cash": 5000000,
    "logistics": true
  }
}
```

**Response:**
```json
{
  "status": "success",
  "emergencyId": "emerg-234567890",
  "message": "Emergency declared",
  "responseTeamNotified": true,
  "internationalAlertsent": true,
  "timestamp": "2025-01-15T14:30:00Z"
}
```

#### GET /api/v1/emergency/{emergencyId}/status

Get emergency response status.

**Request:**
```http
GET /api/v1/emergency/emerg-234567890/status
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "emergencyId": "emerg-234567890",
    "status": "active",
    "timeline": {
      "declared": "2025-01-15T14:30:00Z",
      "lastUpdate": "2025-01-15T18:00:00Z"
    },
    "responseActions": [
      {
        "actionId": "act-123456",
        "type": "reserve_release",
        "quantity": 2000,
        "status": "ongoing",
        "beneficiaries": 200000
      }
    ],
    "monitoring": {
      "peopleReached": 150000,
      "foodDistributed": 1500,
      "coverageRate": 30
    }
  }
}
```

#### PUT /api/v1/emergency/{emergencyId}/resolve

Resolve emergency.

**Request:**
```http
PUT /api/v1/emergency/emerg-234567890/resolve
Authorization: Bearer {token}
Content-Type: application/json

{
  "resolutionNotes": "Situation stabilized, normal operations resumed",
  "finalImpactAssessment": {
    "peopleAffected": 500000,
    "foodDistributed": 5000,
    "economicLoss": 25000000
  }
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Emergency resolved",
  "duration": 45,
  "reportGenerated": true
}
```

---

### 3.7 Analytics & Forecasting

#### GET /api/v1/analytics/forecast

Get food security forecasts.

**Request:**
```http
GET /api/v1/analytics/forecast?regionId=ASIA-KR-SEOUL&horizon=6months
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "regionId": "ASIA-KR-SEOUL",
    "forecast": {
      "horizon": "6months",
      "method": "AI_ML_ensemble",
      "confidence": 85,
      "predictions": [
        {
          "month": "2025-02",
          "foodSecurityIndex": 83.0,
          "availability": 86.0,
          "access": 79.0,
          "risks": ["mild_inflation"]
        },
        {
          "month": "2025-03",
          "foodSecurityIndex": 81.5,
          "availability": 84.0,
          "access": 77.0,
          "risks": ["seasonal_demand"]
        }
      ],
      "recommendations": [
        "Increase rice reserves by 10% before Q2",
        "Monitor import prices for wheat"
      ]
    },
    "generatedAt": "2025-01-15T14:30:00Z"
  }
}
```

#### POST /api/v1/analytics/scenario

Run scenario analysis.

**Request:**
```http
POST /api/v1/analytics/scenario
Authorization: Bearer {token}
Content-Type: application/json

{
  "regionId": "ASIA-KR-SEOUL",
  "scenario": {
    "type": "climate_shock",
    "parameters": {
      "droughtSeverity": 8,
      "duration": 90,
      "affectedCrops": ["rice", "vegetables"]
    }
  },
  "timeframe": "12months"
}
```

**Response:**
```json
{
  "status": "success",
  "scenarioId": "scen-567890123",
  "results": {
    "expectedImpact": {
      "foodSecurityIndexDrop": 15,
      "reserveDepletion": 35,
      "priceIncrease": 25,
      "affectedPopulation": 2000000
    },
    "mitigationOptions": [
      {
        "action": "emergency_import",
        "quantity": 10000,
        "cost": 15000000,
        "effectiveness": 70
      }
    ]
  }
}
```

---

## 4. Webhook Notifications

### 4.1 Webhook Registration

```http
POST /api/v1/webhooks/register
Authorization: Bearer {token}
Content-Type: application/json

{
  "url": "https://your-system.com/webhooks/food-security",
  "events": [
    "reserve.low",
    "emergency.declared",
    "supply_chain.disruption",
    "climate.alert"
  ],
  "secret": "your_webhook_secret"
}
```

### 4.2 Webhook Payload Example

```json
{
  "event": "reserve.low",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "reserveId": "a1b2c3d4-5678-90ab-cdef-1234567890ab",
    "foodType": "rice",
    "currentQuantity": 38000,
    "minimumQuantity": 40000,
    "regionId": "ASIA-KR-SEOUL",
    "severity": "warning"
  },
  "signature": "sha256_signature_here"
}
```

---

## 5. Rate Limiting

- **Free Tier**: 100 requests/hour
- **Basic Tier**: 1,000 requests/hour
- **Premium Tier**: 10,000 requests/hour
- **Enterprise**: Custom limits

**Rate Limit Headers:**
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 750
X-RateLimit-Reset: 1642262400
```

---

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INSUFFICIENT_PERMISSIONS",
    "message": "You do not have permission to access this resource",
    "details": {
      "requiredScope": "reserves:manage",
      "yourScope": "food-security:read"
    }
  },
  "timestamp": "2025-01-15T14:30:00Z",
  "requestId": "req-abc123def456"
}
```

### 6.2 Common Error Codes

- `INVALID_REQUEST`: Malformed request
- `AUTHENTICATION_REQUIRED`: Missing or invalid authentication
- `INSUFFICIENT_PERMISSIONS`: Lack of required authorization
- `RESOURCE_NOT_FOUND`: Requested resource does not exist
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `INTERNAL_ERROR`: Server error
- `SERVICE_UNAVAILABLE`: Service temporarily unavailable

---

## 7. SDK & Client Libraries

### 7.1 Official SDKs

- **JavaScript/TypeScript**: `npm install @wia/food-security-sdk`
- **Python**: `pip install wia-food-security`
- **Java**: Maven dependency
- **Go**: `go get github.com/wia/food-security-go`

### 7.2 Example Usage (JavaScript)

```javascript
import { FoodSecurityAPI } from '@wia/food-security-sdk';

const client = new FoodSecurityAPI({
  apiKey: 'your_api_key',
  apiSecret: 'your_api_secret'
});

// Get food security status
const status = await client.security.getStatus({
  regionId: 'ASIA-KR-SEOUL'
});

console.log(status.foodSecurityIndex.overall); // 82.5
```

---

## 8. Integration Examples

### 8.1 WIA-AGRI Integration

```javascript
// Integrate crop yield predictions with food security forecasting
const cropData = await wiaAgri.getCropYield({ regionId: 'ASIA-KR-SEOUL' });
const securityUpdate = await foodSecurity.updateAvailability({
  regionId: 'ASIA-KR-SEOUL',
  expectedYield: cropData.forecast
});
```

### 8.2 WIA-CLIMATE Integration

```javascript
// Sync climate alerts with food security monitoring
wiaClimate.on('drought.alert', async (alert) => {
  await foodSecurity.climate.recordRisk({
    regionId: alert.regionId,
    riskType: 'drought',
    probability: alert.likelihood,
    severity: alert.severity
  });
});
```

---

**Document Status**: ✅ Complete
**Next Phase**: [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md)

---

© 2025 WIA Standards | MIT License
弘益人間 · Benefit All Humanity
