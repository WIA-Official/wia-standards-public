# WIA-TIME-001: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the API interfaces for time travel physics systems in the WIA-TIME-001 standard. All implementations MUST provide these endpoints for temporal calculations, navigation, and safety verification.

## 2. REST API Endpoints

### 2.1 Base URL
```
https://api.wia-time.org/v1/
```

### 2.2 Temporal Calculation Endpoints

#### Calculate Displacement
```http
POST /calculate/displacement
Content-Type: application/json
Authorization: Bearer {token}

{
  "origin": {
    "time": "2025-01-01T00:00:00Z",
    "coordinates": { "x": 0, "y": 0, "z": 0 },
    "referenceFrame": "earth"
  },
  "destination": {
    "time": "2020-01-01T00:00:00Z",
    "coordinates": { "x": 0, "y": 0, "z": 0 },
    "referenceFrame": "earth"
  },
  "travelerMass": 75
}

Response 200:
{
  "displacementId": "uuid",
  "deltaTime": -157766400,
  "energyRequired": 6.75e18,
  "causalityRisk": "low",
  "feasibility": "possible"
}
```

#### Calculate Energy Requirements
```http
POST /calculate/energy
Content-Type: application/json

{
  "displacementId": "uuid",
  "method": "wormhole|alcubierre|tipler"
}

Response 200:
{
  "totalEnergy": 6.75e18,
  "exoticMatter": -1e15,
  "powerDuration": 3.5,
  "sources": [
    { "type": "antimatter", "amount": 75 }
  ]
}
```

### 2.3 Causality Endpoints

#### Check Paradox Risk
```http
POST /causality/check
Content-Type: application/json

{
  "displacementId": "uuid",
  "plannedActions": [
    { "type": "observation", "target": "historical_event" }
  ]
}

Response 200:
{
  "paradoxRisk": "low",
  "probability": 0.001,
  "warnings": [],
  "approved": true
}
```

#### Verify Timeline Integrity
```http
GET /causality/timeline/{timelineId}/integrity
Authorization: Bearer {token}

Response 200:
{
  "timelineId": "TL-PRIME-A1-001",
  "integrity": 0.9999,
  "anomalies": 0,
  "lastVerified": "2025-01-01T00:00:00Z"
}
```

### 2.4 Navigation Endpoints

#### Plot Worldline
```http
POST /navigation/worldline
Content-Type: application/json

{
  "origin": "SpacetimeCoordinate",
  "destination": "SpacetimeCoordinate",
  "constraints": {
    "maxAcceleration": 10,
    "avoidMassive": true
  }
}

Response 200:
{
  "worldlineId": "uuid",
  "segments": [...],
  "totalProperTime": 3600,
  "waypoints": [...]
}
```

## 3. TypeScript SDK

### 3.1 Installation
```bash
npm install @wia/time-travel-sdk
```

### 3.2 Basic Usage
```typescript
import { TemporalClient, Displacement, CausalityChecker } from '@wia/time-travel-sdk';

const client = new TemporalClient({
  apiKey: process.env.WIA_TIME_API_KEY,
  version: 'v1'
});

// Calculate displacement
const displacement = await client.calculate.displacement({
  origin: { time: new Date('2025-01-01'), frame: 'earth' },
  destination: { time: new Date('2020-01-01'), frame: 'earth' },
  travelerMass: 75
});

// Check causality
const causalityCheck = await client.causality.check({
  displacementId: displacement.id,
  plannedActions: [{ type: 'observation' }]
});

if (causalityCheck.approved) {
  console.log('Time travel approved');
}
```

### 3.3 Type Definitions
```typescript
interface SpacetimeCoordinate {
  time: Date | number;
  x: number;
  y: number;
  z: number;
  frame: 'earth' | 'solar' | 'galactic';
}

interface Displacement {
  id: string;
  origin: SpacetimeCoordinate;
  destination: SpacetimeCoordinate;
  deltaTime: number;
  energyRequired: number;
  causalityRisk: 'none' | 'low' | 'medium' | 'high';
}

interface CausalityResult {
  paradoxRisk: string;
  probability: number;
  approved: boolean;
  warnings: string[];
}
```

## 4. Authentication

### 4.1 API Keys
```http
Authorization: Bearer {api_key}
```

### 4.2 OAuth 2.0
```http
Authorization: Bearer {access_token}
```

### 4.3 Temporal Signatures
```http
X-Temporal-Signature: HMAC-SHA256({timestamp}:{request_hash})
X-Temporal-Timestamp: 1704067200
```

## 5. Rate Limits

| Tier | Requests/Hour | Calculations/Day |
|------|---------------|------------------|
| Free | 100 | 10 |
| Basic | 1,000 | 100 |
| Professional | 10,000 | 1,000 |
| Enterprise | Unlimited | Unlimited |

## 6. Error Codes

| Code | Description |
|------|-------------|
| 400 | Invalid parameters |
| 401 | Unauthorized |
| 403 | Causality violation detected |
| 404 | Resource not found |
| 429 | Rate limit exceeded |
| 500 | Temporal calculation error |
| 503 | Timeline unstable |

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
