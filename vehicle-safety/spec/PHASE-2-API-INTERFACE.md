# WIA-AUTO-022 PHASE 2: API Interface

> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27  
> **Focus:** RESTful APIs, TypeScript interfaces, integration patterns

---

## Overview

Phase 2 establishes RESTful API specifications for vehicle safety system integration. Building upon Phase 1's data formats, these APIs enable real-time safety assessments, collision warnings, airbag deployment decisions, and certification verification.

**弘益人間 (Benefit All Humanity)** - Open APIs democratize safety technology, enabling third-party innovation that saves lives.

---

## Core API Endpoints

### 1. Crash Assessment API

**Endpoint:** `POST /api/v1/crash-assessment`

**Purpose:** Calculate injury risk and safety ratings from crash parameters

**Request:**
```typescript
interface CrashAssessmentRequest {
  vehicleMass: number;           // kg
  impactVelocity: number;        // m/s
  impactAngle: number;           // degrees (0=frontal, 90=side)
  crumpleZoneLength: number;     // meters
  occupants: OccupantInfo[];
}
```

**Response:**
```typescript
interface CrashAssessmentResponse {
  impactForce: number;           // Newtons
  deceleration: number;          // g's
  impactDuration: number;        // seconds
  energyAbsorbed: number;        // joules
  occupantInjury: {
    [position: string]: {
      hic15: number;
      chestDeflection: number;   // mm
      femurLoad: number;         // kN
      tibiaIndex: number;
      injuryRisk: 'low' | 'moderate' | 'high' | 'severe';
    }
  };
  safetyRating: 1 | 2 | 3 | 4 | 5;
}
```

**Latency SLA:** < 50ms p99

---

### 2. Airbag Deployment Decision API

**Endpoint:** `POST /api/v1/airbag-deployment`

**Purpose:** Determine optimal airbag deployment strategy in real-time

**Request:**
```typescript
interface AirbagDeploymentRequest {
  crashSeverity: number;         // 0-10 scale
  crashType: 'frontal' | 'side' | 'rear' | 'rollover';
  impactVelocity: number;        // m/s
  occupantPresence: boolean[];   // [driver, passenger, rear_L, rear_R]
  occupantClassification: string[]; // ['adult', 'child', 'infant', 'empty']
  beltStatus: boolean[];
  crashPulse: number[];          // acceleration time series (g)
}
```

**Response:**
```typescript
interface AirbagDeploymentResponse {
  shouldDeploy: {
    driverFrontal: {deploy: boolean, stage: 0 | 1 | 2, timing: number},
    passengerFrontal: {deploy: boolean, stage: 0 | 1 | 2, timing: number},
    driverSide: {deploy: boolean, timing: number},
    curtainLeft: {deploy: boolean, timing: number}
  };
  pretensioners: {
    driver: {activate: boolean, timing: number, force: number},
    passenger: {activate: boolean, timing: number, force: number}
  };
  reasoning: string[];
}
```

**Latency SLA:** < 10ms p99 (safety-critical)

---

### 3. Collision Warning API

**Endpoint:** `POST /api/v1/collision-warning`

**Purpose:** Assess collision threats and recommend interventions

**Request:**
```typescript
interface CollisionWarningRequest {
  ownVehicle: {
    velocity: number;            // m/s
    position: {x: number, y: number};
    heading: number;             // degrees
  };
  obstacles: Array<{
    type: 'vehicle' | 'pedestrian' | 'cyclist' | 'object';
    position: {x: number, y: number};
    velocity: {vx: number, vy: number};
    confidence: number;          // 0-1
  }>;
  roadCondition: 'dry' | 'wet' | 'snow' | 'ice';
}
```

**Response:**
```typescript
interface CollisionWarningResponse {
  threats: Array<{
    obstacleId: number;
    ttc: number;                 // time to collision (seconds)
    severity: 'low' | 'medium' | 'high' | 'critical';
    collisionProbability: number; // 0-1
    recommendedAction: 'none' | 'warning' | 'brake' | 'emergency_brake' | 'evade';
  }>;
  warningLevel: 0 | 1 | 2 | 3;
  interventionRequired: boolean;
}
```

**Latency SLA:** < 20ms p99

---

### 4. Safety Status Query API

**Endpoint:** `GET /api/v1/safety-status/{vin}`

**Purpose:** Retrieve current safety system status for a vehicle

**Response:**
```typescript
interface SafetyStatusResponse {
  timestamp: string;
  vehicle_id: string;
  systems: {
    abs: {status: string, last_test: string, fault_codes: string[]},
    esc: {status: string, calibration_date: string},
    airbags: {[position: string]: {status: string, deployment_count: number}},
    aeb: {status: string, sensitivity: string, range: number},
    ldw: {status: string, last_calibration: string}
  };
}
```

---

### 5. NCAP Rating Query API

**Endpoint:** `GET /api/v1/ncap-rating/{vin}`

**Purpose:** Retrieve vehicle safety ratings and certification

**Response:**
```typescript
interface NCAPRatingResponse {
  program: string;
  year: number;
  vehicle: {make: string, model: string, variant: string};
  overall_rating: 1 | 2 | 3 | 4 | 5;
  scores: {
    adult_occupant: {percentage: number, points: number, max_points: number},
    child_occupant: {percentage: number, points: number, max_points: number},
    vru: {percentage: number, points: number, max_points: number},
    safety_assist: {percentage: number, points: number, max_points: number}
  };
  verifiable_credential: object; // W3C VC format
}
```

---

## Authentication & Authorization

### OAuth 2.0 Client Credentials Flow

**Token Endpoint:** `POST /oauth/token`

**Request:**
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
scope=safety:read safety:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "safety:read safety:write"
}
```

### Scopes

- `safety:read` - Read-only access to safety data and ratings
- `safety:write` - Submit crash assessments and safety evaluations
- `crash:assess` - Access to real-time crash assessment APIs
- `certification:verify` - Verify safety certifications

### Rate Limiting

- Safety-critical APIs (airbag, collision): 1000 req/sec per client
- Informational APIs (status, rating): 100 req/sec per client
- Rate limit headers: `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`

---

## Implementation Example

```typescript
import axios from 'axios';

class WIASafetyClient {
  private baseURL = 'https://api.wia-standards.org';
  private accessToken: string;

  async authenticate(clientId: string, clientSecret: string) {
    const response = await axios.post(`${this.baseURL}/oauth/token`, {
      grant_type: 'client_credentials',
      client_id: clientId,
      client_secret: clientSecret,
      scope: 'crash:assess'
    });
    this.accessToken = response.data.access_token;
  }

  async assessCrash(request: CrashAssessmentRequest): Promise<CrashAssessmentResponse> {
    const response = await axios.post(
      `${this.baseURL}/api/v1/crash-assessment`,
      request,
      {
        headers: {
          'Authorization': `Bearer ${this.accessToken}`,
          'Content-Type': 'application/json'
        }
      }
    );
    return response.data;
  }
}

// Usage
const client = new WIASafetyClient();
await client.authenticate(process.env.CLIENT_ID, process.env.CLIENT_SECRET);

const result = await client.assessCrash({
  vehicleMass: 1500,
  impactVelocity: 15.6,
  impactAngle: 0,
  crumpleZoneLength: 0.8,
  occupants: [{position: 'driver', mass: 75, age: 35, beltStatus: true}]
});

console.log(`Safety Rating: ${result.safetyRating} stars`);
console.log(`Driver HIC-15: ${result.occupantInjury.driver.hic15}`);
```

---

## Best Practices

1. **Error Handling**: Implement exponential backoff for retries
2. **Caching**: Cache GET responses with appropriate TTL
3. **Timeout**: Set reasonable timeouts (5s for most APIs, 1s for critical)
4. **Logging**: Log all API calls for audit and debugging
5. **Monitoring**: Track latency, error rates, and rate limit consumption

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA | MIT License
